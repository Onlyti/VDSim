"""
5-link rear suspension 3D solver — Stage E (continued).

Mechanically the most general multilink: 5 independent 2-ball-joint rigid
links connect chassis to knuckle.  Each link enforces 1 distance constraint
between its chassis-side and knuckle-side ball joints.

Knuckle pose: 6 DOF (3 position + 3 orientation as axis-angle).
Constraints: 5 link lengths.
Net mechanism DOF: 1 (wheel travel).

Solve: outer Newton on wheel travel → inner least_squares on 6-DOF knuckle
pose with 5 length residuals (+ 1 wheel-z residual to close the loop on
travel input).  Total 6 residuals on 6 DOFs.

5 link passive DOFs (each link rotates freely about its own axis): all
passive, no effect on knuckle motion.  Excluded from the active DOF count.

Outputs the same CSV schema as DW / MP / TA.
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


def rodrigues(axis, theta):
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def axis_angle_to_R(v):
    a = np.linalg.norm(v)
    if a < 1e-12: return np.eye(3)
    return rodrigues(v / a, a)


class FiveLinkSolver:
    LINK_NAMES = ("upper_fore", "upper_aft",
                  "lower_fore", "lower_aft", "toe_link")

    def __init__(self, hp):
        self.hp = hp
        self.side = hp.get("side", "left")
        self.wheel_static    = vec3(hp["wheel"]["center"])
        self.wheel_spin_axis = vec3(hp["wheel"]["spin_axis"])
        self.r_wheel         = float(hp["wheel"]["static_radius"])

        # Per link: chassis-side ball (fixed in world), knuckle-side ball
        # (rigidly attached to knuckle at known body-frame offset).
        # Body-frame reference: knuckle's "origin" is its CG at static; here
        # we use the wheel center as the body-frame origin proxy.
        self.chassis_pts = []
        self.knuckle_pts_static = []
        self.link_lengths = []
        for name in self.LINK_NAMES:
            link = hp["links"][name]
            c = vec3(link["chassis"])
            k = vec3(link["knuckle"])
            self.chassis_pts.append(c)
            self.knuckle_pts_static.append(k)
            self.link_lengths.append(float(np.linalg.norm(k - c)))

        # Offsets of each knuckle-side ball relative to the wheel center
        # in body frame (knuckle origin = wheel_static).
        self.knuckle_off = [k - self.wheel_static
                             for k in self.knuckle_pts_static]

    def _kinematic_residuals(self, knuckle_pose, target_wz=None):
        """
        knuckle_pose: 6 vector = [x, y, z, axis_angle (3)].  (x,y,z) is the
        new world position of the wheel-center reference; axis-angle is the
        knuckle's rotation from static.

        Returns 5 link-length residuals (+1 wheel-z residual if target_wz
        is supplied to lock the wheel-travel input).
        """
        pos = knuckle_pose[:3]
        R = axis_angle_to_R(knuckle_pose[3:6])
        residuals = []
        for i in range(5):
            knuckle_world_i = pos + R @ self.knuckle_off[i]
            d = np.linalg.norm(knuckle_world_i - self.chassis_pts[i])
            residuals.append(d - self.link_lengths[i])
        if target_wz is not None:
            residuals.append(pos[2] - target_wz)
        return residuals

    def solve(self, wheel_travel, steer_rack_dy=0.0,
              x0_pose=None):
        target_wz = self.wheel_static[2] + wheel_travel
        # Initial guess: wheel at expected z, no rotation.
        if x0_pose is None:
            x0 = np.concatenate([
                self.wheel_static + np.array([0, 0, wheel_travel]),
                np.zeros(3),
            ])
        else:
            x0 = np.asarray(x0_pose)
        result = least_squares(
            lambda v: self._kinematic_residuals(v, target_wz=target_wz),
            x0=x0, method='lm', max_nfev=400)
        if not result.success or max(abs(r) for r in result.fun) > 1e-4:
            return {"valid": False, "pose": x0.tolist()}

        pos = result.x[:3]
        R = axis_angle_to_R(result.x[3:6])
        spin_axis_world = R @ self.wheel_spin_axis

        camber = math.atan2(-spin_axis_world[2], abs(spin_axis_world[1]))
        if self.side == "right": camber = -camber

        toe = math.atan2(spin_axis_world[0], spin_axis_world[1])
        if self.side == "right": toe = -toe

        track_change = pos[1] - self.wheel_static[1]
        if self.side == "right": track_change = -track_change

        # Caster not well-defined for 5-link (no explicit kingpin); leave 0.
        caster = 0.0

        self._last_pose = result.x.tolist()
        return {
            "valid": True,
            "wheel_travel": float(wheel_travel),
            "steer_rack_dy": float(steer_rack_dy),
            "wheel_pos": list(map(float, pos)),
            "spin_axis": list(map(float, spin_axis_world)),
            "camber": float(camber),
            "toe": float(toe),
            "track_change": float(track_change),
            "caster": float(caster),
            "axis_angle": result.x[3:6].tolist(),
        }

    def sweep(self, travel_range_m=0.08, n_travel=17):
        # Continuation from static, sweeping outward in both directions.
        travels = sorted(np.linspace(-travel_range_m, travel_range_m, n_travel),
                          key=lambda x: abs(x))
        rows = []
        guesses = {}
        for t in travels:
            best, best_d = None, 1e9
            for tk, v in guesses.items():
                if abs(t - tk) < best_d:
                    best_d, best = abs(t - tk), v
            r = self.solve(t, 0.0, x0_pose=best)
            rows.append(r)
            if r.get("valid"):
                guesses[t] = self._last_pose
        return rows, travels


def plot_sweep(rows, out_png, title=""):
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    ax_cam, ax_toe, ax_trk = axes
    valid = [r for r in rows if r["valid"]]
    t  = np.array([r["wheel_travel"] * 1000 for r in valid])
    cm = np.array([math.degrees(r["camber"]) for r in valid])
    to = np.array([math.degrees(r["toe"])    for r in valid])
    tc = np.array([r["track_change"] * 1000  for r in valid])
    for ax, y, ylab in [(ax_cam, cm, "camber [deg]"),
                         (ax_toe, to, "toe [deg]"),
                         (ax_trk, tc, "track change [mm]")]:
        ax.plot(t, y, 'b-', lw=1.8)
        ax.axhline(0, color='#aaa', lw=0.5); ax.axvline(0, color='#aaa', lw=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("wheel travel [mm]"); ax.set_ylabel(ylab)
    fig.suptitle(title); fig.tight_layout()
    fig.savefig(out_png, dpi=130); print(f"[5link] sweep -> {out_png}")


def save_csv(rows, out_csv):
    keys = ["wheel_travel", "steer_rack_dy", "camber", "toe",
            "track_change", "caster"]
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys + ["valid"])
        for r in rows:
            if r.get("valid"):
                w.writerow([r[k] for k in keys] + [1])
    print(f"[5link] CSV -> {out_csv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default="docs/tasks/T30_ld4_5link/run01")
    ap.add_argument("--travel", type=float, default=0.05)
    ap.add_argument("--n_travel", type=int, default=11)
    args = ap.parse_args()

    with open(args.config) as f: hp = yaml.safe_load(f)
    if hp.get("type") != "five_link":
        raise SystemExit("Expected type=five_link")
    s = FiveLinkSolver(hp)
    static = s.solve(0.0, 0.0)
    print("--- 5-link static ---")
    print(f"  link lengths: {[f'{L:.4f}' for L in s.link_lengths]}")
    for k in ("camber", "toe", "track_change", "caster"):
        print(f"  {k:15s} = {math.degrees(static[k]):+.5f} deg")

    rows, travels = s.sweep(args.travel, args.n_travel)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    save_csv(rows, out_dir / "sweep_3d.csv")
    plot_sweep(rows, out_dir / "sweep_3d.png",
               title=Path(args.config).stem + " — 5-link rear sweep")


if __name__ == "__main__":
    main()
