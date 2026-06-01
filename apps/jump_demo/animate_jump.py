"""Side-view animation for the 14-DOF jump demo (T23).

Top panel: x-z side view, camera tracks vehicle.  Ground profile drawn as a
filled polygon; ramp + cliff visible.  Vehicle body = pitched rectangle; front
and rear wheels = circles at the unsprung-mass z.

Bottom panels: sprung CG z, pitch, per-corner Fz, airborne flag.
"""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.transforms import Affine2D
import numpy as np
import yaml


def load_csv(path):
    cols = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                cols.setdefault(k, []).append(
                    float(v) if v not in ('True', 'False') else (v == 'True'))
    return cols


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rundir", default="apps/jump_demo/run01")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mp4", default=None)
    args = ap.parse_args()

    rundir = Path(args.rundir)
    d = load_csv(rundir / "telemetry.csv")
    with open(rundir / "ramp.yaml") as f:
        ramp = yaml.safe_load(f)
    with open(rundir / "veh.yaml") as f:
        veh = yaml.safe_load(f)

    a, b = veh["a"], veh["b"]
    r_w = veh["r_wheel"]
    L_body = a + b + 0.3      # add small overhang
    H_body = 0.5
    n = len(d["t"])

    # Sample every 1/fps seconds (CSV is 100 Hz)
    csv_dt = d["t"][1] - d["t"][0]
    stride = max(1, int(round(1.0 / (args.fps * csv_dt))))
    idx = list(range(0, n, stride))

    # Ground polygon (smooth)
    xs = np.linspace(-2, max(d["x"]) + 10, 1000)
    def ground_z(x):
        if x < ramp["x_start"]:
            return 0.0
        if x < ramp["x_top"]:
            u = (x - ramp["x_start"]) / (ramp["x_top"] - ramp["x_start"])
            return ramp["ramp_height"] * 0.5 * (1 - np.cos(np.pi * u))
        if x < ramp["x_lip_end"]:
            return ramp["ramp_height"]
        return 0.0
    zs = np.array([ground_z(x) for x in xs])
    ground_poly_xy = np.column_stack([
        np.concatenate([xs, xs[::-1]]),
        np.concatenate([zs, np.full_like(zs, -1.5)]),
    ])

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(3, 4, height_ratios=[3, 1, 1], hspace=0.45, wspace=0.4)
    ax_view = fig.add_subplot(gs[0, :])
    ax_zs   = fig.add_subplot(gs[1, 0:2])
    ax_pitch= fig.add_subplot(gs[1, 2:4])
    ax_fzf  = fig.add_subplot(gs[2, 0:2])
    ax_fzr  = fig.add_subplot(gs[2, 2:4])

    ax_view.set_aspect('equal')
    ax_view.set_ylim(-0.6, 3.0)
    ax_view.add_patch(Polygon(ground_poly_xy, closed=True,
                              facecolor='#cfd8dc', edgecolor='#37474f'))
    ax_view.axhline(0, color='#90a4ae', lw=0.6, alpha=0.6)
    ax_view.set_xlabel("world x [m]"); ax_view.set_ylabel("world z [m]")
    ax_view.set_title("VDSim Ld3-FourteenDOF (world-z extension) — jump simulation")

    body_patch = Rectangle((-L_body/2, -H_body/2), L_body, H_body,
                           facecolor="#01579b", edgecolor="#002060", alpha=0.92)
    ax_view.add_patch(body_patch)
    wheel_f = Circle((+a, -H_body/2), r_w, color="#212121")
    wheel_r = Circle((-b, -H_body/2), r_w, color="#212121")
    ax_view.add_patch(wheel_f); ax_view.add_patch(wheel_r)
    title_txt = ax_view.text(0.02, 0.94, "", transform=ax_view.transAxes,
                              ha='left', va='top', family='monospace', fontsize=10,
                              bbox=dict(facecolor='white', alpha=0.85, lw=0))

    # Time-series panels (drawn fully, with a moving cursor)
    t_arr = d["t"]
    ax_zs.plot(t_arr, d["z_s"], 'b-', lw=1.3); ax_zs.set_ylabel("z_s [m]")
    ax_pitch.plot(t_arr, [p * 57.3 for p in d["pitch"]], 'm-', lw=1.3)
    ax_pitch.set_ylabel("pitch [deg]")
    ax_fzf.plot(t_arr, d["Fz_FL"], label="FL"); ax_fzf.plot(t_arr, d["Fz_FR"], label="FR")
    ax_fzf.set_ylabel("Fz front [N]"); ax_fzf.legend(loc='upper right', fontsize=8)
    ax_fzr.plot(t_arr, d["Fz_RL"], label="RL"); ax_fzr.plot(t_arr, d["Fz_RR"], label="RR")
    ax_fzr.set_ylabel("Fz rear [N]"); ax_fzr.legend(loc='upper right', fontsize=8)
    for ax in (ax_zs, ax_pitch, ax_fzf, ax_fzr):
        ax.grid(True, alpha=0.3); ax.set_xlabel("t [s]")
    cursors = [ax.axvline(0, color='k', lw=0.8) for ax in (ax_zs, ax_pitch, ax_fzf, ax_fzr)]

    def render_frame(k):
        i = idx[k]
        t = d["t"][i]; x = d["x"][i]; vx = d["vx"][i]
        z_s = d["z_s"][i]; pitch = d["pitch"][i]
        z_uFL = d["z_u_FL"][i]; z_uRL = d["z_u_RL"][i]
        airborne = d["airborne"][i]

        # follow camera
        ax_view.set_xlim(x - 8, x + 18)

        # body transform: VDSim convention is negative pitch = nose UP
        # (z_corner = z_s - rx · sin(pitch)).  Matplotlib's Affine2D rotates
        # counter-clockwise for positive angle (which would visually be nose-up
        # for +x-forward, +z-up).  So negate pitch when feeding the renderer.
        tr = (Affine2D().rotate(-pitch).translate(x, z_s)
              + ax_view.transData)
        body_patch.set_transform(tr)

        # wheels: in body frame at (+a, -H_body/2) etc, but in world we want
        # the unsprung positions.  Side view: average FL+FR for "front", RL+RR rear.
        # Place wheels in world at (x + a*cos(pitch), z_u_front + r_w (axle))
        # For simplicity use small-angle: x_front = x + a*cos(pitch), z_front = z_uFL + r_w
        cp, sp = np.cos(pitch), np.sin(pitch)
        x_front = x + a * cp
        x_rear  = x - b * cp
        wheel_f.center = (x_front, z_uFL + r_w)
        wheel_r.center = (x_rear,  z_uRL + r_w)

        title_txt.set_text(
            f"t={t:5.2f}s   x={x:6.2f}m   vx={vx:5.2f}m/s\n"
            f"z_s={z_s:.3f}m   pitch={pitch*57.3:+6.1f}°"
            f"   {'AIRBORNE' if airborne else 'CONTACT'}")
        title_txt.set_color('#c62828' if airborne else '#1b5e20')

        for cur in cursors:
            cur.set_xdata([t, t])
        return body_patch, wheel_f, wheel_r, title_txt, *cursors

    anim = FuncAnimation(fig, render_frame, frames=len(idx),
                         interval=1000 / args.fps, blit=False)

    out_mp4 = Path(args.mp4) if args.mp4 else rundir / "jump_demo.mp4"
    writer = matplotlib.animation.FFMpegWriter(
        fps=args.fps, bitrate=2500,
        extra_args=['-pix_fmt', 'yuv420p'])
    print(f"[anim] writing {len(idx)} frames @ {args.fps} fps -> {out_mp4}")
    anim.save(str(out_mp4), writer=writer, dpi=100)
    print(f"[anim] done — {out_mp4}")


if __name__ == "__main__":
    main()
