"""
MacPherson strut 3D solver — Stage E (proper kinematic).

Key insight: the strut is a CYLINDRICAL JOINT — tube rigidly on knuckle,
shaft on chassis via spherical top mount, telescoping (length variable)
+ axial rotation allowed.  So the kinematic constraint is NOT "strut length
constant" but "tube axis passes through chassis ST" (the shaft, attached at
ST via spherical, must be coaxial with the tube).

Tube axis direction in body frame is fixed by hardpoints:
    tube_axis_body = (ST_static − SK_static).normalized

After knuckle rotation R, the tube axis in world is R @ tube_axis_body.
Constraint: SK_world must lie on the line through ST in direction
R @ tube_axis_body.  Equivalently:

    (SK_world − ST_chassis) × (R @ tube_axis_body) = 0      (3 scalars, 1 redundant)

Combined with the tie-rod length constraint (1 scalar) and the LK-position
constraint from LCA (3 scalars), the knuckle's 6 DOFs are fully determined
(no free DOF, no regularization needed).  Strut compression length is a
free output of the solve (the diagnostic for spring deflection).

Inputs:  wheel_travel, steer_rack_dy.
Outputs: camber, toe, track_change, caster, strut_length (compression).
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
from scipy.optimize import least_squares


def vec3(p): return np.array([float(p[0]), float(p[1]), float(p[2])])


def rodrigues(axis_unit, theta):
    K = np.array([[0, -axis_unit[2], axis_unit[1]],
                  [axis_unit[2], 0, -axis_unit[0]],
                  [-axis_unit[1], axis_unit[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def euler_to_R(angles):
    """ZYX intrinsic (yaw, pitch, roll)."""
    yaw, pitch, roll = angles
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class MPSolver:
    def __init__(self, hp):
        self.hp = hp
        self.side = hp.get("side", "left")

        self.wheel_static    = vec3(hp["wheel"]["center"])
        self.wheel_spin_axis = vec3(hp["wheel"]["spin_axis"])
        self.r_wheel         = float(hp["wheel"]["static_radius"])

        self.lca_cf = vec3(hp["lca"]["chassis_front"])
        self.lca_cr = vec3(hp["lca"]["chassis_rear"])
        self.lca_axis = self.lca_cr - self.lca_cf
        self.lca_axis /= np.linalg.norm(self.lca_axis)
        self.lca_pivot = self.lca_cf
        self.lca_knuckle_static = vec3(hp["lca"]["knuckle"])

        self.strut_top    = vec3(hp["strut"]["top"])
        self.strut_bottom = vec3(hp["strut"]["bottom"])
        self.L_strut = float(np.linalg.norm(self.strut_bottom - self.strut_top))

        self.tr_inner_static    = vec3(hp["tie_rod"]["rack"])
        self.tr_knuckle_static  = vec3(hp["tie_rod"]["knuckle"])
        self.L_tr = float(np.linalg.norm(
            self.tr_knuckle_static - self.tr_inner_static))

        # Offsets of SK, TK, wheel-center from LK in the knuckle frame (static).
        # We treat the static body frame == the knuckle frame at static.
        self.off_SK = self.strut_bottom - self.lca_knuckle_static
        self.off_TK = self.tr_knuckle_static - self.lca_knuckle_static
        self.off_wheel = self.wheel_static - self.lca_knuckle_static

        # Tube axis direction in body frame: from SK toward ST at static,
        # normalized.  This direction is rigidly fixed in the knuckle's
        # body frame (the tube itself is rigidly bolted to the knuckle).
        tube_vec = self.strut_top - self.strut_bottom
        self.tube_axis_body = tube_vec / np.linalg.norm(tube_vec)

    # ---- LCA helpers ----
    def _lca_knuckle_at(self, theta):
        return self.lca_pivot + rodrigues(self.lca_axis, theta) @ (
            self.lca_knuckle_static - self.lca_pivot)

    def _solve_lca_for_wheel_z(self, target_z, tol=1e-6, max_iter=50):
        offset_z = self.wheel_static[2] - self.lca_knuckle_static[2]
        theta = 0.0
        for _ in range(max_iter):
            lk = self._lca_knuckle_at(theta)
            wz = lk[2] + offset_z
            err = wz - target_z
            if abs(err) < tol: return theta
            dth = 1e-5
            wz_p = self._lca_knuckle_at(theta + dth)[2] + offset_z
            slope = (wz_p - wz) / dth
            if abs(slope) < 1e-9: break
            theta -= err / slope
        return theta

    # ---- Main solve ----
    def solve(self, wheel_travel, steer_rack_dy=0.0,
              x0_axis_angle=None):
        target_wz = self.wheel_static[2] + wheel_travel
        theta_l = self._solve_lca_for_wheel_z(target_wz)
        lk = self._lca_knuckle_at(theta_l)

        tr_inner = self.tr_inner_static + np.array([0.0, steer_rack_dy, 0.0])

        def axis_angle_to_R(v):
            ang = np.linalg.norm(v)
            if ang < 1e-12: return np.eye(3)
            return rodrigues(v / ang, ang)

        def residuals(v):
            R = axis_angle_to_R(v)
            sk = lk + R @ self.off_SK
            tk = lk + R @ self.off_TK
            tube_axis_world = R @ self.tube_axis_body

            # Strut constraint: (SK - ST) must be parallel to tube_axis_world.
            # Cross product = 0 gives 3 scalar residuals (1 redundant, since
            # parallel-vector constraint has only 2 independent components).
            cross = np.cross(sk - self.strut_top, tube_axis_world)

            # Tie rod length constraint (1 scalar).
            r_tie = np.linalg.norm(tk - tr_inner) - self.L_tr

            return [cross[0], cross[1], cross[2], r_tie]

        x0 = np.asarray(x0_axis_angle if x0_axis_angle is not None
                        else [0.0, 0.0, 0.0])
        result = least_squares(residuals, x0=x0,
                               method='lm', max_nfev=300)
        if not result.success:
            return {"valid": False, "axis_angle": x0.tolist()}
        R = axis_angle_to_R(result.x)
        self._last_axis_angle = result.x.tolist()

        sk_world = lk + R @ self.off_SK
        tk_world = lk + R @ self.off_TK
        wheel_pos = lk + R @ self.off_wheel
        spin_axis_world = R @ self.wheel_spin_axis

        # Camber: top of wheel toward +y / −y in y-z plane
        camber = math.atan2(-spin_axis_world[2], abs(spin_axis_world[1]))
        if self.side == "right": camber = -camber

        # Toe: spin axis x-component over y-component (in x-y plane)
        toe = math.atan2(spin_axis_world[0], spin_axis_world[1])
        if self.side == "right": toe = -toe

        track_change = wheel_pos[1] - self.wheel_static[1]
        if self.side == "right": track_change = -track_change

        # Caster: strut axis (SK - ST) projected x-z, angle from +z
        kp = sk_world - self.strut_top
        caster = math.atan2(kp[0], -kp[2])    # strut points DOWN from top, so -z

        strut_length_now = float(np.linalg.norm(sk_world - self.strut_top))
        strut_compression = self.L_strut - strut_length_now  # + : compressed

        return {
            "valid": True,
            "wheel_travel": float(wheel_travel),
            "steer_rack_dy": float(steer_rack_dy),
            "lk": list(map(float, lk)),
            "sk": list(map(float, sk_world)),
            "tk": list(map(float, tk_world)),
            "wheel_pos": list(map(float, wheel_pos)),
            "spin_axis": list(map(float, spin_axis_world)),
            "camber": float(camber),
            "toe": float(toe),
            "track_change": float(track_change),
            "caster": float(caster),
            "strut_compression": strut_compression,
        }

    def sweep(self, travel_range_m=0.08, n_travel=11,
              steer_range_m=0.025, n_steer=5):
        # Continuation: starting from static (0, 0), sweep travel outward in
        # both directions, then for each travel sweep steers from 0 outward.
        # Each call uses the previous result as initial guess to avoid
        # spurious far-flip local minima.
        travels = sorted(np.linspace(-travel_range_m, travel_range_m, n_travel),
                          key=lambda x: abs(x))
        steers  = sorted(np.linspace(-steer_range_m, steer_range_m, n_steer),
                          key=lambda x: abs(x))
        rows = []
        # Cache: best initial guess per travel for each steer level
        guesses = {}    # key (travel, steer) -> axis-angle vector
        for t in travels:
            for s in steers:
                # Use closest already-solved (t, s) as initial guess
                best = None; best_d = 1e9
                for (tk, sk), v in guesses.items():
                    d = abs(t - tk) + abs(s - sk)
                    if d < best_d: best_d, best = d, v
                r = self.solve(t, s, x0_axis_angle=best)
                rows.append(r)
                if r.get("valid"):
                    guesses[(t, s)] = self._last_axis_angle
        return rows, travels, steers


def plot_sweep(rows, travels, steers, out_png, title=""):
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax_cam, ax_toe, ax_trk, ax_cas = axes.flatten()
    for s_idx, s in enumerate(steers):
        cs = [r for r in rows if r.get("valid")
              and abs(r["steer_rack_dy"] - s) < 1e-9]
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
    for ax, ylab in [(ax_cam, "camber [deg]"), (ax_toe, "toe [deg]"),
                     (ax_trk, "track change [mm]"), (ax_cas, "caster [deg]")]:
        ax.set_xlabel("wheel travel [mm]"); ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3); ax.axhline(0, color='#aaa', lw=0.5)
        ax.axvline(0, color='#aaa', lw=0.5)
        ax.legend(fontsize=8, loc='best')
    fig.suptitle(title); fig.tight_layout()
    fig.savefig(out_png, dpi=130); print(f"[mp3d] sweep -> {out_png}")


def save_csv(rows, out_csv):
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
    print(f"[mp3d] CSV -> {out_csv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default="docs/tasks/T28_ld4_mp/run01")
    ap.add_argument("--travel", type=float, default=0.08)
    ap.add_argument("--n_travel", type=int, default=11)
    ap.add_argument("--steer", type=float, default=0.025)
    ap.add_argument("--n_steer", type=int, default=5)
    args = ap.parse_args()

    with open(args.config) as f: hp = yaml.safe_load(f)
    if hp.get("type") != "macpherson":
        raise SystemExit("Expected type=macpherson")
    s = MPSolver(hp)
    static = s.solve(0.0, 0.0)
    print("--- MacPherson static ---")
    for k in ("camber", "toe", "track_change", "caster"):
        print(f"  {k:15s} = {math.degrees(static[k]):+.5f} deg")

    rows, travels, steers = s.sweep(args.travel, args.n_travel,
                                     args.steer, args.n_steer)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    save_csv(rows, out_dir / "sweep_3d.csv")
    plot_sweep(rows, travels, steers, out_dir / "sweep_3d.png",
               title=Path(args.config).stem + " — MacPherson 3D sweep")


if __name__ == "__main__":
    main()
