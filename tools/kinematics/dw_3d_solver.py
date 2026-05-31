"""
Double-wishbone 3D kinematic solver — Stage B.

Given hardpoints + (wheel_travel, steer_rack_dy), solves the rigid-body
linkage for the knuckle position/orientation, then reads off:
    camber  [rad]   — wheel plane tilt about body x, in y-z projection
    toe     [rad]   — wheel spin axis rotation about body z, in x-y projection
    track_change [m] — wheel center y-displacement from static
    caster  [rad]   — kingpin axis tilt about body y, in x-z projection

Method:
    1. LCA rotates about its chassis axis  (a 1-DOF revolute joint).
    2. UCA rotates about its chassis axis  (1-DOF revolute).
    3. Knuckle is rigidly attached at LK, UK, TK.  TK must lie at fixed
       distance L_tr from the tie-rod inner end (which moves with steering).
    4. For each wheel travel target:
         - solve LCA angle so wheel-z hits target (1D Newton-Raphson)
         - solve UCA angle so |UK − LK| = L_knuckle_UL (2-sphere intersection)
         - solve TK from 3-sphere trilateration:
              spheres at LK (r=L_LT), UK (r=L_UT), TR_inner (r=L_tr).
    5. Knuckle orientation from 3 attach points → wheel center + spin axis.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


def vec3(p):  return np.array([float(p[0]), float(p[1]), float(p[2])])


def rodrigues(axis_unit: np.ndarray, theta: float) -> np.ndarray:
    """3x3 rotation matrix for angle θ about unit-axis."""
    K = np.array([[0, -axis_unit[2], axis_unit[1]],
                  [axis_unit[2], 0, -axis_unit[0]],
                  [-axis_unit[1], axis_unit[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def trilaterate(c1, r1, c2, r2, c3, r3, near=None):
    """Find a point P with |P-c1|=r1, |P-c2|=r2, |P-c3|=r3 (3D).
    Returns the intersection point closer to `near`, or None if none exists.
    """
    # Build local frame: x along c2-c1, y in plane of c1,c2,c3.
    ex = c2 - c1
    d = np.linalg.norm(ex)
    if d < 1e-9: return None
    ex /= d
    tmp = c3 - c1
    i = np.dot(ex, tmp)
    ey = tmp - i * ex
    ny = np.linalg.norm(ey)
    if ny < 1e-9: return None
    ey /= ny
    ez = np.cross(ex, ey)
    j = np.dot(ey, c3 - c1)
    x = (r1**2 - r2**2 + d**2) / (2 * d)
    y = (r1**2 - r3**2 + i**2 + j**2 - 2 * i * x) / (2 * j)
    z_sq = r1**2 - x**2 - y**2
    if z_sq < -1e-9: return None
    z = math.sqrt(max(0.0, z_sq))
    p_pos = c1 + x * ex + y * ey + z * ez
    p_neg = c1 + x * ex + y * ey - z * ez
    if near is None: return p_pos
    return p_pos if np.linalg.norm(p_pos - near) <= np.linalg.norm(p_neg - near) else p_neg


class DW3DSolver:
    def __init__(self, hp: dict):
        self.hp = hp
        self.side = hp.get("side", "left")
        self.wheel_static = vec3(hp["wheel"]["center"])
        self.wheel_spin_axis = vec3(hp["wheel"]["spin_axis"])
        self.r_wheel = float(hp["wheel"]["static_radius"])

        # Control-arm chassis axes
        self.lca_cf = vec3(hp["lca"]["chassis_front"])
        self.lca_cr = vec3(hp["lca"]["chassis_rear"])
        self.lca_axis = self.lca_cr - self.lca_cf
        self.lca_axis /= np.linalg.norm(self.lca_axis)
        self.lca_pivot = self.lca_cf       # any point on axis
        self.lca_knuckle_static = vec3(hp["lca"]["knuckle"])

        self.uca_cf = vec3(hp["uca"]["chassis_front"])
        self.uca_cr = vec3(hp["uca"]["chassis_rear"])
        self.uca_axis = self.uca_cr - self.uca_cf
        self.uca_axis /= np.linalg.norm(self.uca_axis)
        self.uca_pivot = self.uca_cf
        self.uca_knuckle_static = vec3(hp["uca"]["knuckle"])

        # Static linkage lengths
        self.L_LT = None    # knuckle LK-TK
        self.L_UT = None    # knuckle UK-TK
        self.L_LU = float(np.linalg.norm(
            self.uca_knuckle_static - self.lca_knuckle_static))

        # Tie rod
        self.tr_inner_static = vec3(hp["tie_rod"]["rack"])
        self.tr_knuckle_static = vec3(hp["tie_rod"]["knuckle"])
        self.L_tr = float(np.linalg.norm(
            self.tr_knuckle_static - self.tr_inner_static))
        self.L_LT = float(np.linalg.norm(
            self.tr_knuckle_static - self.lca_knuckle_static))
        self.L_UT = float(np.linalg.norm(
            self.tr_knuckle_static - self.uca_knuckle_static))

    # ---- LCA / UCA revolute helpers ----
    def _lca_knuckle_at(self, theta):
        return self.lca_pivot + rodrigues(self.lca_axis, theta) @ (
            self.lca_knuckle_static - self.lca_pivot)

    def _uca_knuckle_at(self, theta):
        return self.uca_pivot + rodrigues(self.uca_axis, theta) @ (
            self.uca_knuckle_static - self.uca_pivot)

    # ---- Inner kinematic solve for given LCA angle ----
    def _kinematic_at(self, theta_l, steer_rack_dy=0.0):
        """Run the full DW solve for a given LCA angle.  Returns a dict with
        lk, uk, tk, R_now, R_0, wheel_pos, spin_axis (all numpy arrays), or
        None if any inner step fails.  This is the canonical kinematics —
        used both by the outer wheel-z Newton and by solve()."""
        lk = self._lca_knuckle_at(theta_l)
        theta_u, uk = self._solve_uca_for_knuckle_length(lk)
        if uk is None:
            return None
        tr_inner = self.tr_inner_static + np.array([0.0, steer_rack_dy, 0.0])
        tk = trilaterate(lk, self.L_LT, uk, self.L_UT,
                         tr_inner, self.L_tr,
                         near=self.tr_knuckle_static)
        if tk is None:
            return None
        # Knuckle frame (kingpin = LK→UK, plane via TK).
        ax = uk - lk
        ax /= np.linalg.norm(ax)
        tk_off = tk - lk
        ay_raw = tk_off - np.dot(tk_off, ax) * ax
        ay = ay_raw / max(1e-9, np.linalg.norm(ay_raw))
        az = np.cross(ax, ay)
        R_now = np.column_stack([ax, ay, az])
        ax0 = self.uca_knuckle_static - self.lca_knuckle_static
        ax0 /= np.linalg.norm(ax0)
        tk_off0 = self.tr_knuckle_static - self.lca_knuckle_static
        ay0_raw = tk_off0 - np.dot(tk_off0, ax0) * ax0
        ay0 = ay0_raw / max(1e-9, np.linalg.norm(ay0_raw))
        az0 = np.cross(ax0, ay0)
        R_0 = np.column_stack([ax0, ay0, az0])
        R_delta = R_now @ R_0.T
        wheel_off_local = R_0.T @ (self.wheel_static - self.lca_knuckle_static)
        wheel_pos = lk + R_now @ wheel_off_local
        spin_axis_world = R_delta @ self.wheel_spin_axis
        return {
            "lk": lk, "uk": uk, "tk": tk,
            "R_now": R_now, "R_0": R_0, "R_delta": R_delta,
            "wheel_pos": wheel_pos, "spin_axis": spin_axis_world,
            "tr_inner": tr_inner,
            "theta_l": theta_l, "theta_u": theta_u,
        }

    # ---- Inverse: Newton on TRUE wheel-z ----
    def _solve_lca_for_wheel_z(self, target_z, steer_rack_dy=0.0,
                                tol=1e-7, max_iter=30):
        """Newton on θ_lca s.t. the *true* wheel z (after full knuckle
        kinematics) equals target_z.  Replaces the previous small-angle
        approximation `wz ≈ LK_z + off_wheel_z` which had ~3 mm error at
        ±0.1 rad LCA angle."""
        theta = 0.0
        for _ in range(max_iter):
            st = self._kinematic_at(theta, steer_rack_dy)
            if st is None: return None
            wz = st["wheel_pos"][2]
            err = wz - target_z
            if abs(err) < tol: return theta
            dth = 1e-5
            st_p = self._kinematic_at(theta + dth, steer_rack_dy)
            if st_p is None: return None
            slope = (st_p["wheel_pos"][2] - wz) / dth
            if abs(slope) < 1e-9: break
            theta -= err / slope
        return theta

    # ---- Solve UCA θ so |UK - LK| = L_LU ----
    def _solve_uca_for_knuckle_length(self, lk_pos, tol=1e-6, max_iter=50):
        theta = 0.0
        for _ in range(max_iter):
            uk = self._uca_knuckle_at(theta)
            d = np.linalg.norm(uk - lk_pos)
            err = d - self.L_LU
            if abs(err) < tol: return theta, uk
            dth = 1e-5
            uk_p = self._uca_knuckle_at(theta + dth)
            slope = (np.linalg.norm(uk_p - lk_pos) - d) / dth
            if abs(slope) < 1e-9: break
            theta -= err / slope
        uk = self._uca_knuckle_at(theta)
        return theta, uk

    # ---- Main solve ----
    def solve(self, wheel_travel: float, steer_rack_dy: float = 0.0) -> dict:
        target_wz = self.wheel_static[2] + wheel_travel
        theta_l = self._solve_lca_for_wheel_z(target_wz, steer_rack_dy)
        if theta_l is None:
            return {"valid": False}
        st = self._kinematic_at(theta_l, steer_rack_dy)
        if st is None:
            return {"valid": False}

        lk, uk, tk = st["lk"], st["uk"], st["tk"]
        wheel_pos = st["wheel_pos"]
        spin_axis_world = st["spin_axis"]

        # Camber: angle of spin axis tilt from +y in y-z plane.
        # Spin axis at static = +y; tilt toward +z (top-of-wheel toward -y for
        # left side) gives positive γ here when we use the same convention as
        # MP (atan2(-z, |y|)).  Right-side sign flip below.
        camber = math.atan2(-spin_axis_world[2], abs(spin_axis_world[1]))
        if self.side == "right": camber = -camber

        # Toe: angle of spin axis in x-y plane from +y.
        toe = math.atan2(spin_axis_world[0], spin_axis_world[1])
        if self.side == "right": toe = -toe

        # Track change: y-displacement of wheel center.
        track_change = wheel_pos[1] - self.wheel_static[1]
        if self.side == "right": track_change = -track_change

        # Caster: kingpin axis (UK - LK) in x-z plane.
        kp = uk - lk
        kp_xz_norm = math.hypot(kp[0], kp[2])
        caster = math.atan2(kp[0], kp[2]) if kp_xz_norm > 1e-9 else 0.0

        return {
            "valid": True,
            "wheel_travel": float(wheel_travel),
            "steer_rack_dy": float(steer_rack_dy),
            "lk": list(map(float, lk)),
            "uk": list(map(float, uk)),
            "tk": list(map(float, tk)),
            "wheel_pos": list(map(float, wheel_pos)),
            "spin_axis": list(map(float, spin_axis_world)),
            "camber": float(camber),
            "toe": float(toe),
            "track_change": float(track_change),
            "caster": float(caster),
        }

    # ---- Sweep over (travel, steer) grid ----
    def sweep(self, travel_range_m: float = 0.08, n_travel: int = 11,
              steer_range_m: float = 0.03, n_steer: int = 5):
        travels = np.linspace(-travel_range_m, travel_range_m, n_travel)
        steers  = np.linspace(-steer_range_m, steer_range_m, n_steer)
        rows = []
        for t in travels:
            for s in steers:
                out = self.solve(t, s)
                rows.append(out)
        return rows, travels, steers


# -----------------------------------------------------------------------------
# Plotting
# -----------------------------------------------------------------------------
def plot_sweep_3d(rows, travels, steers, out_png: Path, title: str = ""):
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax_cam, ax_toe, ax_trk, ax_cas = axes.flatten()
    for s_idx, s in enumerate(steers):
        cs = [r for r in rows if r["valid"] and abs(r["steer_rack_dy"] - s) < 1e-9]
        if not cs: continue
        t  = [r["wheel_travel"] * 1000 for r in cs]
        cm = [math.degrees(r["camber"]) for r in cs]
        to = [math.degrees(r["toe"])    for r in cs]
        tc = [r["track_change"] * 1000  for r in cs]
        ca = [math.degrees(r["caster"]) for r in cs]
        lbl = f"rack dy={s*1000:+.0f} mm"
        ax_cam.plot(t, cm, lw=1.5, label=lbl)
        ax_toe.plot(t, to, lw=1.5, label=lbl)
        ax_trk.plot(t, tc, lw=1.5, label=lbl)
        ax_cas.plot(t, ca, lw=1.5, label=lbl)
    for ax, ylabel in [(ax_cam, "camber [deg]"),
                       (ax_toe, "toe [deg]"),
                       (ax_trk, "track change [mm]"),
                       (ax_cas, "caster [deg]")]:
        ax.set_xlabel("wheel travel [mm]"); ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3); ax.axhline(0, color='#aaa', lw=0.5)
        ax.axvline(0, color='#aaa', lw=0.5)
        ax.legend(fontsize=8, loc='best')
    fig.suptitle(title); fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"[dw3d] sweep -> {out_png}")


def save_sweep_csv(rows, out_csv: Path):
    keys = ["wheel_travel", "steer_rack_dy", "camber", "toe",
            "track_change", "caster"]
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys + ["valid"])
        for r in rows:
            if r.get("valid"):
                w.writerow([r[k] for k in keys] + [1])
            else:
                w.writerow(["", "", "", "", "", "", 0])
    print(f"[dw3d] CSV -> {out_csv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default="docs/tasks/T27_ld4_dw/run3d")
    ap.add_argument("--travel", type=float, default=0.08)
    ap.add_argument("--n_travel", type=int, default=11)
    ap.add_argument("--steer", type=float, default=0.03,
                    help="rack lateral disp range ± [m]")
    ap.add_argument("--n_steer", type=int, default=5)
    args = ap.parse_args()

    with open(args.config) as f: hp = yaml.safe_load(f)
    solver = DW3DSolver(hp)

    # Static sanity check
    static = solver.solve(0.0, 0.0)
    print("--- DW3D static ---")
    for k in ("camber", "toe", "track_change", "caster"):
        print(f"  {k:15s} = {math.degrees(static[k]):+.5f} deg "
              f"({static[k]:+.6f} rad)")

    rows, travels, steers = solver.sweep(args.travel, args.n_travel,
                                          args.steer, args.n_steer)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    save_sweep_csv(rows, out_dir / "sweep_3d.csv")
    plot_sweep_3d(rows, travels, steers, out_dir / "sweep_3d.png",
                  title=Path(args.config).stem + " — 3D sweep")


if __name__ == "__main__":
    main()
