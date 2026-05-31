"""
Semi-trailing arm rear suspension 3D solver — Stage E (continued).

Mechanically the simplest multilink: a single rigid arm rotates about one
chassis-side revolute axis.  The knuckle + wheel are rigidly attached to
the arm.  1-DOF mechanism, no rear steering input.

Camber & toe gain with wheel travel come from the AXIS ORIENTATION:
    - Pure trailing arm (axis ∥ +y): zero camber/toe gain
    - Semi-trailing (axis tilted in x-y or x-y-z): non-zero gain

Solve:
    1. Newton on θ_arm s.t. the true wheel z hits target_z
    2. Read wheel pose, camber, toe, track from the rotated rigid arm

Outputs the same CSV schema as DW / MP so it plugs into the C++
ISuspensionKinematics lookup unchanged.
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


def vec3(p): return np.array([float(p[0]), float(p[1]), float(p[2])])


def rodrigues(axis, theta):
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


class TASolver:
    def __init__(self, hp):
        self.hp = hp
        self.side = hp.get("side", "left")
        self.wheel_static    = vec3(hp["wheel"]["center"])
        self.wheel_spin_axis = vec3(hp["wheel"]["spin_axis"])
        self.r_wheel         = float(hp["wheel"]["static_radius"])
        self.pivot_in  = vec3(hp["arm_pivot"]["chassis_inboard"])
        self.pivot_out = vec3(hp["arm_pivot"]["chassis_outboard"])
        axis_vec = self.pivot_out - self.pivot_in
        self.arm_axis = axis_vec / np.linalg.norm(axis_vec)
        # Pivot point: any point on the axis; use inboard end.
        self.pivot = self.pivot_in
        # Wheel rigid offset from pivot at static (body frame).
        self.wheel_off = self.wheel_static - self.pivot

    def _wheel_at(self, theta):
        return self.pivot + rodrigues(self.arm_axis, theta) @ self.wheel_off

    def _spin_axis_at(self, theta):
        return rodrigues(self.arm_axis, theta) @ self.wheel_spin_axis

    def _solve_arm_for_wheel_z(self, target_z, tol=1e-7, max_iter=30):
        theta = 0.0
        for _ in range(max_iter):
            wz = self._wheel_at(theta)[2]
            err = wz - target_z
            if abs(err) < tol: return theta
            dth = 1e-5
            wz_p = self._wheel_at(theta + dth)[2]
            slope = (wz_p - wz) / dth
            if abs(slope) < 1e-9: break
            theta -= err / slope
        return theta

    def solve(self, wheel_travel, steer_rack_dy=0.0):
        # steer_rack_dy ignored on rear (no tie rod)
        target_wz = self.wheel_static[2] + wheel_travel
        theta = self._solve_arm_for_wheel_z(target_wz)
        wheel_pos = self._wheel_at(theta)
        spin_axis_world = self._spin_axis_at(theta)

        camber = math.atan2(-spin_axis_world[2], abs(spin_axis_world[1]))
        if self.side == "right": camber = -camber

        toe = math.atan2(spin_axis_world[0], spin_axis_world[1])
        if self.side == "right": toe = -toe

        track_change = wheel_pos[1] - self.wheel_static[1]
        if self.side == "right": track_change = -track_change

        # No defined kingpin for trailing arm — caster meaningless; emit 0.
        caster = 0.0

        return {
            "valid": True,
            "wheel_travel": float(wheel_travel),
            "steer_rack_dy": float(steer_rack_dy),
            "wheel_pos": list(map(float, wheel_pos)),
            "spin_axis": list(map(float, spin_axis_world)),
            "camber": float(camber),
            "toe": float(toe),
            "track_change": float(track_change),
            "caster": float(caster),
            "arm_theta": float(theta),
        }

    def sweep(self, travel_range_m=0.08, n_travel=17):
        # No steering — single column at steer_rack_dy = 0 for schema compat.
        travels = np.linspace(-travel_range_m, travel_range_m, n_travel)
        rows = []
        for t in travels:
            rows.append(self.solve(t, 0.0))
        return rows, travels, np.array([0.0])


def plot_sweep(rows, out_png, title=""):
    fig, axes = plt.subplots(1, 4, figsize=(16, 4))
    ax_cam, ax_toe, ax_trk, ax_arm = axes
    t  = np.array([r["wheel_travel"] * 1000 for r in rows if r["valid"]])
    cm = np.array([math.degrees(r["camber"]) for r in rows if r["valid"]])
    to = np.array([math.degrees(r["toe"])    for r in rows if r["valid"]])
    tc = np.array([r["track_change"] * 1000  for r in rows if r["valid"]])
    am = np.array([math.degrees(r["arm_theta"]) for r in rows if r["valid"]])
    for ax, y, ylab in [(ax_cam, cm, "camber [deg]"),
                         (ax_toe, to, "toe [deg]"),
                         (ax_trk, tc, "track change [mm]"),
                         (ax_arm, am, "arm θ [deg]")]:
        ax.plot(t, y, 'b-', lw=1.8)
        ax.axhline(0, color='#aaa', lw=0.5); ax.axvline(0, color='#aaa', lw=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("wheel travel [mm]"); ax.set_ylabel(ylab)
    fig.suptitle(title); fig.tight_layout()
    fig.savefig(out_png, dpi=130); print(f"[ta3d] sweep -> {out_png}")


def save_csv(rows, out_csv):
    keys = ["wheel_travel", "steer_rack_dy", "camber", "toe",
            "track_change", "caster"]
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys + ["valid"])
        for r in rows:
            if r.get("valid"):
                w.writerow([r[k] for k in keys] + [1])
    print(f"[ta3d] CSV -> {out_csv}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default="docs/tasks/T29_ld4_ta/run01")
    ap.add_argument("--travel", type=float, default=0.08)
    ap.add_argument("--n_travel", type=int, default=17)
    args = ap.parse_args()

    with open(args.config) as f: hp = yaml.safe_load(f)
    if hp.get("type") != "trailing_arm":
        raise SystemExit("Expected type=trailing_arm")
    s = TASolver(hp)
    static = s.solve(0.0, 0.0)
    print("--- Trailing arm static ---")
    for k in ("camber", "toe", "track_change", "caster"):
        print(f"  {k:15s} = {math.degrees(static[k]):+.5f} deg")
    print(f"  arm_axis_world  = {s.arm_axis}")
    semi_tilt = math.degrees(math.atan2(s.arm_axis[0], s.arm_axis[1]))
    print(f"  semi-trailing tilt from +y = {semi_tilt:+.2f} deg (around z-axis)")

    rows, travels, _ = s.sweep(args.travel, args.n_travel)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    save_csv(rows, out_dir / "sweep_3d.csv")
    plot_sweep(rows, out_dir / "sweep_3d.png",
               title=Path(args.config).stem + " — trailing arm sweep")


if __name__ == "__main__":
    main()
