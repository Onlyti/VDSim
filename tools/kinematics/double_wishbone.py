"""
Double-wishbone front suspension — 2D side-view (y-z plane) kinematic analyzer.

Computes the *static* properties from hardpoints:
    - Instantaneous center (IC) of the wheel-carrier (the point about which
      the knuckle rotates infinitesimally for a given wheel travel).
    - Roll center (RC) height — the intersection of the line from
      tire-contact-patch through the IC with the body centerline (y=0).
    - Camber gain  dγ/dz  [deg / mm]  at the static ride height.
    - Track change  dT/dz   [mm/mm].
    - Anti-dive / anti-squat geometry (only for x-z plane, here Phase 2).

Standard 2D analysis from Milliken "Race Car Vehicle Dynamics" §17.2.
For full 3D wheel-travel sweep with proper closed-form linkage solve, use
the future `dw_3d_solver.py` (Stage B).

Outputs:
    - Side-view PNG with linkage, IC, RC, ground line, wheel.
    - YAML summary of derived quantities.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


# -----------------------------------------------------------------------------
# Geometry helpers (2D in y-z plane — drop the x component)
# -----------------------------------------------------------------------------
def yz(p):
    """Drop body-frame x; keep (y, z)."""
    return np.array([float(p[1]), float(p[2])])


def line_intersect(p1, d1, p2, d2):
    """Intersect two parametric lines  p1 + t·d1  and  p2 + s·d2  in 2D.
    Returns (point, success_flag).  Parallel lines → flag=False.
    """
    A = np.column_stack([d1, -d2])
    b = p2 - p1
    if abs(np.linalg.det(A)) < 1e-9:
        return np.zeros(2), False
    t = np.linalg.solve(A, b)[0]
    return p1 + t * d1, True


# -----------------------------------------------------------------------------
# Double-wishbone 2D analysis
# -----------------------------------------------------------------------------
class DWAnalyzer:
    def __init__(self, hp: dict):
        self.hp = hp
        self.wheel_center  = yz(hp["wheel"]["center"])
        self.tire_radius   = float(hp["wheel"]["static_radius"])
        # The 2D side-view projection averages chassis_front and chassis_rear
        # to get the "effective" instant-center in the y-z plane.  This is the
        # standard simplification when treating DW as planar.
        self.lca_chassis = 0.5 * (yz(hp["lca"]["chassis_front"]) +
                                  yz(hp["lca"]["chassis_rear"]))
        self.lca_knuckle = yz(hp["lca"]["knuckle"])
        self.uca_chassis = 0.5 * (yz(hp["uca"]["chassis_front"]) +
                                  yz(hp["uca"]["chassis_rear"]))
        self.uca_knuckle = yz(hp["uca"]["knuckle"])
        self.side        = hp.get("side", "left")
        self.tire_contact = np.array([self.wheel_center[0],
                                       self.wheel_center[1] - self.tire_radius])

    # ---- Derived quantities ---------------------------------------------------
    def instant_center(self):
        """Intersection of LCA-line and UCA-line — the wheel-carrier IC."""
        d_lca = self.lca_knuckle - self.lca_chassis
        d_uca = self.uca_knuckle - self.uca_chassis
        ic, ok = line_intersect(self.lca_chassis, d_lca,
                                 self.uca_chassis, d_uca)
        return (ic, ok)

    def roll_center_height(self):
        """RC = intersection of (tire-contact → IC) with y=0 line (body
        centerline)."""
        ic, ok = self.instant_center()
        if not ok:
            return None
        d = ic - self.tire_contact
        if abs(d[0]) < 1e-9:        # vertical line — no y=0 intersection
            return None
        t = (-self.tire_contact[0]) / d[0]
        rc_z = self.tire_contact[1] + t * d[1]
        return rc_z

    def camber_gain(self):
        """Linearized camber gain at static position.
        For small wheel travel Δz, the wheel pivots about the IC.  The
        camber angle change Δγ ≈ Δz / (y_distance from wheel center to IC).
        Sign: with IC inside the vehicle (smaller |y| than wheel), positive
        wheel travel (up) gives negative camber (top of wheel inward = the
        desired "camber recovery" behavior).
        """
        ic, ok = self.instant_center()
        if not ok:
            return None
        dy = self.wheel_center[0] - ic[0]
        if abs(dy) < 1e-6:
            return None
        # dγ/dz with sign convention: γ>0 when top of wheel toward +y.
        # Right side (negative y) needs sign flip externally.
        sign = 1.0 if self.side == "left" else -1.0
        return -sign / dy   # [rad/m]  (≈ rad per meter of travel)

    def track_change_gain(self):
        """dT/dz — how much the wheel y-coordinate changes per unit travel.
        Approximated by the slope of (wheel center position) ↔ IC line at
        the wheel center.
        """
        ic, ok = self.instant_center()
        if not ok:
            return None
        dy = self.wheel_center[0] - ic[0]   # remember wheel_center is (y, z)
        dz = self.wheel_center[1] - ic[1]
        # The wheel arcs around IC with radius r = sqrt(dy² + dz²); the
        # tangent at the wheel center has slope dy/dz = -dz/dy (perpendicular
        # to the radius vector).  In small-travel limit this is the gain.
        if abs(dy) < 1e-6:
            return 0.0
        return -dz / dy

    def summary(self):
        ic, ok = self.instant_center()
        rc_z = self.roll_center_height()
        cg = self.camber_gain()
        tcg = self.track_change_gain()
        return {
            "side": self.side,
            "wheel_center_yz": list(map(float, self.wheel_center)),
            "tire_contact_yz": list(map(float, self.tire_contact)),
            "instant_center_yz": (list(map(float, ic)) if ok else None),
            "instant_center_valid": bool(ok),
            "roll_center_height_m": float(rc_z) if rc_z is not None else None,
            "camber_gain_deg_per_mm": (math.degrees(cg) / 1000.0
                                         if cg is not None else None),
            "track_change_per_unit_travel": (float(tcg)
                                              if tcg is not None else None),
        }

    # ---- Nonlinear wheel-travel sweep ----------------------------------------
    def sweep(self, travel_range_m: float = 0.10, n: int = 41):
        """Closed-form 2-bar linkage solve in the y-z plane.
        The LCA is treated as a rigid bar pivoting about its chassis pivot;
        the UCA is a rigid bar pivoting about its chassis pivot.  The knuckle
        is the rigid link connecting LCA-knuckle to UCA-knuckle (constant
        length).  We parameterize by the LCA angle θ_l, solve for the UCA
        angle θ_u that keeps the knuckle length constant, then read wheel
        position + orientation.
        """
        # Static (input) geometry
        L_lca = np.linalg.norm(self.lca_knuckle - self.lca_chassis)
        L_uca = np.linalg.norm(self.uca_knuckle - self.uca_chassis)
        L_knuckle = np.linalg.norm(self.uca_knuckle - self.lca_knuckle)
        wheel_offset_from_lca = self.wheel_center - self.lca_knuckle
        # Static LCA angle (atan2)
        d0 = self.lca_knuckle - self.lca_chassis
        theta_l_0 = math.atan2(d0[1], d0[0])
        # Static knuckle direction (kingpin line)
        kp0 = self.uca_knuckle - self.lca_knuckle
        kp_ang_0 = math.atan2(kp0[1], kp0[0])

        travels = np.linspace(-travel_range_m, travel_range_m, n)
        results = []
        for dz in travels:
            # Move LCA so that the wheel center rises by dz (small-angle approx
            # then closed-form solve).  Iterate by adjusting θ_l in small
            # steps until wheel z matches target.
            # Wheel z = LCA_chassis.z + L_lca · sin(θ_l) + wheel_offset_from_lca.z
            target_wheel_z = self.wheel_center[1] + dz
            # Closed-form for θ_l from required LCA-knuckle z:
            req_lca_knuckle_z = target_wheel_z - wheel_offset_from_lca[1]
            sin_arg = (req_lca_knuckle_z - self.lca_chassis[1]) / L_lca
            if abs(sin_arg) > 1.0:
                results.append((dz, None, None, None, None))
                continue
            # LCA can be above or below; pick the one closest to static.
            theta_l_a = math.asin(sin_arg)
            theta_l_b = math.pi - theta_l_a
            theta_l = theta_l_a if abs(theta_l_a - theta_l_0) < abs(theta_l_b - theta_l_0) else theta_l_b
            lca_knuckle = self.lca_chassis + L_lca * np.array(
                [math.cos(theta_l), math.sin(theta_l)])

            # Solve UCA angle so |UCA_knuckle - LCA_knuckle| = L_knuckle.
            # UCA knuckle on circle of radius L_uca about uca_chassis;
            # must also be on circle of radius L_knuckle about lca_knuckle.
            uca_knuckle = circle_intersect(
                self.uca_chassis, L_uca, lca_knuckle, L_knuckle,
                self.uca_knuckle)
            if uca_knuckle is None:
                results.append((dz, None, None, None, None))
                continue

            # Wheel center: tracks LCA knuckle by its static offset, but also
            # rotates with the knuckle.  For simplicity assume wheel offset
            # from LCA is rigidly attached to the knuckle, which rotates by
            # the new kingpin angle minus the static kingpin angle.
            kp = uca_knuckle - lca_knuckle
            kp_ang = math.atan2(kp[1], kp[0])
            d_kp = kp_ang - kp_ang_0     # incremental rotation
            cos_k, sin_k = math.cos(d_kp), math.sin(d_kp)
            offset_rot = np.array([
                cos_k * wheel_offset_from_lca[0] - sin_k * wheel_offset_from_lca[1],
                sin_k * wheel_offset_from_lca[0] + cos_k * wheel_offset_from_lca[1],
            ])
            wheel_pos = lca_knuckle + offset_rot

            # Camber = angle of kingpin from vertical, sign convention as in
            # camber_gain() (left side: positive when top toward +y).
            # Wheel plane is perpendicular to spin axis which is perpendicular
            # to kingpin in our 2D model.  So camber = kingpin tilt from
            # vertical, measured at the wheel.
            # Vertical reference is (0, 1).  Tilt angle:
            #   γ = arctan2(kp_y, kp_z)   where (kp_y, kp_z) = (kp[0], kp[1])
            #   For pure vertical kingpin (kp_y=0, kp_z=1), γ = 0.
            #   When top leans toward +y (left), γ > 0.
            sign_ext = 1.0 if self.side == "left" else -1.0
            camber = math.atan2(kp[0], kp[1])   # angle from +z toward +y
            # Track change: wheel_pos[0] minus static
            track_change = wheel_pos[0] - self.wheel_center[0]
            results.append((dz, float(camber) * sign_ext,
                            float(track_change),
                            (float(lca_knuckle[0]), float(lca_knuckle[1])),
                            (float(uca_knuckle[0]), float(uca_knuckle[1]))))
        return results


def circle_intersect(c1, r1, c2, r2, near):
    """Intersection of two circles in 2D.  Returns the intersection closer
    to `near`, or None if circles don't intersect."""
    d_vec = c2 - c1
    d = np.linalg.norm(d_vec)
    if d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return None
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h_sq = r1 * r1 - a * a
    if h_sq < 0:
        h_sq = 0.0
    h = math.sqrt(h_sq)
    p = c1 + a * d_vec / d
    perp = np.array([-d_vec[1], d_vec[0]]) / d
    s1 = p + h * perp
    s2 = p - h * perp
    if np.linalg.norm(s1 - near) <= np.linalg.norm(s2 - near):
        return s1
    return s2


# -----------------------------------------------------------------------------
# Rendering
# -----------------------------------------------------------------------------
def plot_side_view(an: DWAnalyzer, out_png: Path, title: str = ""):
    fig, ax = plt.subplots(figsize=(9, 7))
    # Ground
    ax.axhline(0, color="#999999", lw=0.7, alpha=0.7)
    # LCA + UCA
    def link(p1, p2, color, label):
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=2.2, label=label)
        ax.scatter([p1[0], p2[0]], [p1[1], p2[1]],
                   color=color, s=45, zorder=4)
    link(an.lca_chassis, an.lca_knuckle, "#1565c0", "LCA")
    link(an.uca_chassis, an.uca_knuckle, "#c62828", "UCA")
    # Knuckle: line between UCA knuckle and LCA knuckle (kingpin)
    ax.plot([an.uca_knuckle[0], an.lca_knuckle[0]],
            [an.uca_knuckle[1], an.lca_knuckle[1]],
            color="#37474f", lw=1.6, ls="--", label="kingpin")
    # Wheel (circle in side view of y-z plane = ellipse? no, it's truly a
    # circle: the wheel rim seen from end-on.)
    wheel = plt.Circle((an.wheel_center[0], an.wheel_center[1]),
                       an.tire_radius, fill=False, color="#212121", lw=1.5)
    ax.add_patch(wheel)
    ax.plot(an.tire_contact[0], an.tire_contact[1], "ko", ms=6,
            label="contact patch")

    # IC and RC
    ic, ok = an.instant_center()
    if ok:
        ax.scatter(ic[0], ic[1], color="#ef6c00", s=90, marker="X",
                   zorder=5, label=f"IC ({ic[0]:.2f}, {ic[1]:.2f})")
        # Line LCA extended toward IC
        for chassis, knuckle, c in [(an.lca_chassis, an.lca_knuckle, "#1565c0"),
                                     (an.uca_chassis, an.uca_knuckle, "#c62828")]:
            # extend along chassis→knuckle direction by drawing a faint long line
            d = knuckle - chassis
            n = d / (np.linalg.norm(d) + 1e-9)
            far = chassis + n * 3.0
            ax.plot([chassis[0], far[0]], [chassis[1], far[1]],
                    color=c, ls=":", lw=0.9, alpha=0.6)
    rc_z = an.roll_center_height()
    if rc_z is not None:
        ax.scatter(0.0, rc_z, color="#6a1b9a", s=110, marker="*",
                   zorder=5, label=f"RC (0, {rc_z:.3f})")
        ax.plot([an.tire_contact[0], 0.0], [an.tire_contact[1], rc_z],
                color="#6a1b9a", ls=":", lw=0.8, alpha=0.7)

    ax.set_aspect("equal")
    ax.set_xlabel("body y [m] (+ leftward)")
    ax.set_ylabel("body z [m] (+ up)")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"[dw] side view -> {out_png}")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def plot_sweep(an: DWAnalyzer, sweep_rows, out_png: Path, title: str = ""):
    """Plot camber + track change vs wheel travel."""
    travels = np.array([r[0] * 1000 for r in sweep_rows])    # mm
    cambers = np.array([math.degrees(r[1]) if r[1] is not None else np.nan
                         for r in sweep_rows])
    tracks  = np.array([r[2] * 1000 if r[2] is not None else np.nan
                         for r in sweep_rows])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    ax = axes[0]
    ax.plot(travels, cambers, "b-", lw=2)
    ax.axhline(0, color="#999999", lw=0.5)
    ax.axvline(0, color="#999999", lw=0.5)
    ax.set_xlabel("wheel travel [mm]  (+ bump)")
    ax.set_ylabel("camber [deg]")
    ax.set_title("camber vs wheel travel")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(travels, tracks, "r-", lw=2)
    ax.axhline(0, color="#999999", lw=0.5)
    ax.axvline(0, color="#999999", lw=0.5)
    ax.set_xlabel("wheel travel [mm]")
    ax.set_ylabel("track change [mm]  (+ outward)")
    ax.set_title("track change vs wheel travel")
    ax.grid(True, alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"[dw] sweep curves -> {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="hardpoint YAML")
    ap.add_argument("--out", default="docs/tasks/T27_ld4_dw/run01", help="output dir")
    ap.add_argument("--travel", type=float, default=0.08,
                    help="wheel travel range ± [m] (default 80 mm)")
    ap.add_argument("--n", type=int, default=41)
    args = ap.parse_args()

    cfg_path = Path(args.config)
    with open(cfg_path) as f:
        hp = yaml.safe_load(f)
    if hp.get("type") != "double_wishbone":
        raise SystemExit(f"Expected type=double_wishbone, got {hp.get('type')}")

    an = DWAnalyzer(hp)
    summary = an.summary()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "summary.yaml", "w") as f:
        yaml.safe_dump(summary, f, default_flow_style=False)
    plot_side_view(an, out_dir / "side_view.png", title=cfg_path.stem)

    # Wheel-travel sweep
    sweep_rows = an.sweep(args.travel, args.n)
    plot_sweep(an, sweep_rows, out_dir / "travel_sweep.png",
               title=f"{cfg_path.stem} — wheel-travel sweep ±{args.travel*1000:.0f} mm")
    # Save sweep as CSV
    import csv
    with open(out_dir / "travel_sweep.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["travel_m", "camber_rad", "track_change_m",
                    "lca_knuckle_yz", "uca_knuckle_yz"])
        for r in sweep_rows:
            w.writerow([r[0],
                         r[1] if r[1] is not None else "",
                         r[2] if r[2] is not None else "",
                         (f"{r[3][0]:.4f},{r[3][1]:.4f}" if r[3] else ""),
                         (f"{r[4][0]:.4f},{r[4][1]:.4f}" if r[4] else "")])

    print("---- DW analyzer summary ----")
    print(yaml.safe_dump(summary, default_flow_style=False))
    # Linear-vs-nonlinear cross check
    valid_cam = [(r[0], r[1]) for r in sweep_rows if r[1] is not None]
    if len(valid_cam) >= 2:
        # finite-diff slope near 0:
        z = [r[0] for r in valid_cam]; c = [r[1] for r in valid_cam]
        idx0 = min(range(len(z)), key=lambda i: abs(z[i]))
        slope = (c[idx0+1] - c[idx0-1]) / (z[idx0+1] - z[idx0-1])
        print(f"nonlinear camber gain at z=0 : {math.degrees(slope)/1000:+.4f} deg/mm")
        print(f"linear  (IC method)          : {summary['camber_gain_deg_per_mm']:+.4f} deg/mm")


if __name__ == "__main__":
    main()
