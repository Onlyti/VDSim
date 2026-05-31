"""3D animation for the T24 turning-jump simulation.

Top: 3D chase view.  Vehicle body = pitched / rolled / yawed box.  Wheels =
discs at each unsprung position.  Ramp = sloped polygon strip.

Bottom panels: yaw, roll, pitch, sprung CG z, per-corner Fz (front, rear).
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
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


def rotation_matrix(yaw, pitch, roll):
    cy, sy = math.cos(yaw),   math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll),  math.sin(roll)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,    0, 1]])
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]])
    return Rz @ Ry @ Rx


def make_box_verts(L, W, H):
    """Body-frame box vertices (8 corners)."""
    return np.array([
        (-L/2, -W/2, -H/2), (+L/2, -W/2, -H/2),
        (+L/2, +W/2, -H/2), (-L/2, +W/2, -H/2),
        (-L/2, -W/2, +H/2), (+L/2, -W/2, +H/2),
        (+L/2, +W/2, +H/2), (-L/2, +W/2, +H/2),
    ])


BOX_FACES = [
    [0, 1, 2, 3],   # bottom
    [4, 5, 6, 7],   # top
    [1, 2, 6, 5],   # front
    [0, 3, 7, 4],   # rear
    [3, 2, 6, 7],   # left
    [0, 1, 5, 4],   # right
]


def make_wheel_verts(r, n=20):
    """Body-frame wheel disc — circle in body xz plane."""
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([r * np.cos(ang),
                            np.zeros_like(ang),
                            r * np.sin(ang)])


def ramp_polygon(ramp_info, n=50, y_lateral_pad=0.0):
    """Construct ramp surface polygons covering x_start .. x_lip_end."""
    xs_a = np.linspace(ramp_info["x_start"], ramp_info["x_top"], n)
    xs_b = np.linspace(ramp_info["x_top"], ramp_info["x_lip_end"], 4)
    xs = np.concatenate([xs_a, xs_b])
    h = ramp_info["ramp_height"]
    x0, x1 = ramp_info["x_start"], ramp_info["x_top"]
    zs = []
    for x in xs:
        if x <= x0:
            zs.append(0.0)
        elif x <= x1:
            u = (x - x0) / (x1 - x0)
            zs.append(h * 0.5 * (1 - np.cos(np.pi * u)))
        else:
            zs.append(h)
    zs = np.array(zs)
    y_w = ramp_info["y_half_width"] + y_lateral_pad
    # Build a tubular strip — top face only (good enough visually)
    polys = []
    for i in range(len(xs) - 1):
        polys.append([
            (xs[i],   -y_w, zs[i]),
            (xs[i+1], -y_w, zs[i+1]),
            (xs[i+1], +y_w, zs[i+1]),
            (xs[i],   +y_w, zs[i]),
        ])
    # Front cliff face (vertical drop after lip)
    x_lip = ramp_info["x_lip_end"]
    polys.append([
        (x_lip, -y_w, h), (x_lip, +y_w, h),
        (x_lip, +y_w, 0), (x_lip, -y_w, 0),
    ])
    return polys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rundir", default="apps/jump_demo/run3d")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--mp4", default=None)
    args = ap.parse_args()

    rundir = Path(args.rundir)
    d = load_csv(rundir / "telemetry.csv")
    with open(rundir / "ramp.yaml") as f: ramp = yaml.safe_load(f)
    with open(rundir / "veh.yaml")  as f: veh = yaml.safe_load(f)

    a, b = veh["a"], veh["b"]
    r_w = veh["r_wheel"]
    track = max(veh["track_front"], veh["track_rear"])
    L_body = a + b + 0.3
    W_body = track + 0.1
    H_body = 0.5

    box0   = make_box_verts(L_body, W_body, H_body)
    wheel0 = make_wheel_verts(r_w, n=20)
    wheel_half_width = 0.10
    # Wheel body-frame offsets (FL, FR, RL, RR)
    wheel_off = np.array([
        [+a, +veh["track_front"]/2, 0.0],
        [+a, -veh["track_front"]/2, 0.0],
        [-b, +veh["track_rear"]/2,  0.0],
        [-b, -veh["track_rear"]/2,  0.0],
    ])

    # Sample stride from csv (CSV is 100 Hz)
    n = len(d["t"])
    csv_dt = d["t"][1] - d["t"][0]
    stride = max(1, int(round(1.0 / (args.fps * csv_dt))))
    idx = list(range(0, n, stride))

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(3, 4, height_ratios=[4, 1, 1], hspace=0.40, wspace=0.35)
    ax3d   = fig.add_subplot(gs[0, :], projection='3d')
    ax_yaw = fig.add_subplot(gs[1, 0])
    ax_rol = fig.add_subplot(gs[1, 1])
    ax_pit = fig.add_subplot(gs[1, 2])
    ax_zs  = fig.add_subplot(gs[1, 3])
    ax_fzf = fig.add_subplot(gs[2, 0:2])
    ax_fzr = fig.add_subplot(gs[2, 2:4])

    # Static ground polygon (extends far)
    x_max_world = max(d["x"]) + 12
    ground_polys = [[(-5, -7, 0), (x_max_world, -7, 0),
                     (x_max_world, +7, 0), (-5, +7, 0)]]
    ax3d.add_collection3d(Poly3DCollection(
        ground_polys, facecolor='#cfd8dc', edgecolor='none', alpha=0.6))
    # Ramp
    ramp_polys = ramp_polygon(ramp)
    ax3d.add_collection3d(Poly3DCollection(
        ramp_polys, facecolor='#90a4ae', edgecolor='#37474f',
        linewidths=0.4, alpha=0.95))

    # Reference ramp center line (yellow)
    ax3d.plot([ramp["x_start"], ramp["x_lip_end"]], [0, 0],
              [0, ramp["ramp_height"]], color='#fdd835', lw=2.2, zorder=5)

    # Vehicle artist placeholders (we'll set verts each frame)
    body_collection = Poly3DCollection([], facecolor='#01579b',
                                       edgecolor='#002060', linewidths=0.6)
    ax3d.add_collection3d(body_collection)
    wheel_collections = []
    for _ in range(4):
        wc = Poly3DCollection([], facecolor='#212121', edgecolor='#000',
                               linewidths=0.3)
        ax3d.add_collection3d(wc)
        wheel_collections.append(wc)

    ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
    ax3d.set_box_aspect((4, 2, 1))

    # Time-series plots (drawn once with cursors)
    t = d["t"]
    ax_yaw.plot(t, [math.degrees(v) for v in d["yaw"]], 'g-'); ax_yaw.set_ylabel("yaw [deg]")
    ax_rol.plot(t, [math.degrees(v) for v in d["roll"]], 'r-'); ax_rol.set_ylabel("roll [deg]")
    ax_pit.plot(t, [math.degrees(v) for v in d["pitch"]], 'm-'); ax_pit.set_ylabel("pitch [deg]")
    ax_zs.plot(t, d["z_s"], 'b-'); ax_zs.set_ylabel("z_s [m]")
    ax_fzf.plot(t, d["Fz_FL"], label='FL'); ax_fzf.plot(t, d["Fz_FR"], label='FR')
    ax_fzr.plot(t, d["Fz_RL"], label='RL'); ax_fzr.plot(t, d["Fz_RR"], label='RR')
    ax_fzf.set_ylabel("Fz front [N]"); ax_fzr.set_ylabel("Fz rear [N]")
    for ax in (ax_yaw, ax_rol, ax_pit, ax_zs, ax_fzf, ax_fzr):
        ax.grid(True, alpha=0.3); ax.set_xlabel("t [s]")
    for ax in (ax_fzf, ax_fzr): ax.legend(loc='upper right', fontsize=8)
    cursors = [ax.axvline(0, color='k', lw=0.7)
               for ax in (ax_yaw, ax_rol, ax_pit, ax_zs, ax_fzf, ax_fzr)]

    title_txt = fig.suptitle("", fontsize=11, family='monospace', y=0.995)

    def render(k):
        i = idx[k]
        t_i  = d["t"][i]
        x_w  = d["x"][i];  y_w = d["y"][i]
        yaw  = d["yaw"][i]; pitch = d["pitch"][i]; roll = d["roll"][i]
        z_s  = d["z_s"][i]
        z_u  = [d["z_u_FL"][i], d["z_u_FR"][i], d["z_u_RL"][i], d["z_u_RR"][i]]
        airborne = d["airborne"][i]
        R = rotation_matrix(yaw, pitch, roll)

        # Body verts world
        body_verts_world = box0 @ R.T + np.array([x_w, y_w, z_s])
        body_collection.set_verts([body_verts_world[f] for f in BOX_FACES])

        # Wheels: each is two discs (left side + right side of wheel width) +
        # connecting strip approximated as one rectangle around the rim
        for j in range(4):
            # disc plane: perpendicular to body y-axis (so wheel "side" faces +y)
            # We need a disc oriented with normal along body y direction.
            # Take the wheel0 (in body xz plane) and apply body rotation R.
            disc_body = wheel0.copy()
            # outer / inner side of wheel
            disc_outer = disc_body + np.array([0, +wheel_half_width, 0])
            disc_inner = disc_body + np.array([0, -wheel_half_width, 0])
            # Translate body-frame wheel-center first
            wheel_center_body = wheel_off[j].copy()
            # Set z of wheel center: unsprung world z is the AXLE z, so the body-
            # frame wheel center sits at the axle position; in body frame this is
            # roughly the unsprung z minus z_s (after rotation).  For visual we
            # just place the disc at unsprung world position directly.
            # Build wheel world verts by rotating around body, then translating
            # so that wheel center is at (x_axle, y_axle, z_u + r_w).
            cy, sy = math.cos(yaw), math.sin(yaw)
            # Wheel center world position (yaw-rotated body offset)
            x_axle = x_w + wheel_center_body[0] * cy - wheel_center_body[1] * sy
            y_axle = y_w + wheel_center_body[0] * sy + wheel_center_body[1] * cy
            z_axle = z_u[j] + r_w
            # Disc orientation = body rotation R; we want disc normal along body y
            outer_world = (disc_outer @ R.T) + np.array([x_axle, y_axle, z_axle]) - (wheel0 @ R.T).mean(axis=0)
            inner_world = (disc_inner @ R.T) + np.array([x_axle, y_axle, z_axle]) - (wheel0 @ R.T).mean(axis=0)
            wheel_collections[j].set_verts([outer_world, inner_world])

        # Chase camera (follow vehicle, fixed elev)
        ax3d.view_init(elev=18, azim=math.degrees(yaw) - 130)
        ax3d.set_xlim(x_w - 8, x_w + 14)
        ax3d.set_ylim(y_w - 8, y_w + 8)
        ax3d.set_zlim(-0.5, 3.5)

        for cur in cursors:
            cur.set_xdata([t_i, t_i])
        title_txt.set_text(
            f"t={t_i:5.2f}s  x={x_w:6.2f}  y={y_w:5.2f}  yaw={math.degrees(yaw):+6.1f}°  "
            f"roll={math.degrees(roll):+5.1f}°  pitch={math.degrees(pitch):+5.1f}°  "
            f"z_s={z_s:.3f}m  {'AIRBORNE' if airborne else 'CONTACT'}")
        title_txt.set_color('#c62828' if airborne else '#1b5e20')
        return body_collection, *wheel_collections, title_txt, *cursors

    anim = FuncAnimation(fig, render, frames=len(idx),
                         interval=1000/args.fps, blit=False)
    out_mp4 = Path(args.mp4) if args.mp4 else rundir / "jump_3d_demo.mp4"
    writer = FFMpegWriter(fps=args.fps, bitrate=3500,
                          extra_args=['-pix_fmt', 'yuv420p'])
    print(f"[anim3d] {len(idx)} frames @ {args.fps} fps -> {out_mp4}")
    anim.save(str(out_mp4), writer=writer, dpi=100)
    print(f"[anim3d] done — {out_mp4}")


if __name__ == "__main__":
    main()
