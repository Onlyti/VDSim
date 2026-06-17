"""Animate the L5 spatial-strut demo (C++ core telemetry) — jump then loop.

    python3 animate_strut.py --jump /tmp/strut_jump.csv --loop /tmp/strut_loop.csv \
                             --out /tmp/strut_demo.mp4

Side view (x-z, ISO 8855). Labels/titles in English (Korean font breaks in mpl).
"""
import argparse
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Circle, Rectangle


def load(path):
    # CSV: t,x,z,pitch,vx,vz,fz_sum,fwd_x,fwd_z,comp[4],fz[4],ax,ay,wspin,
    #      then unsprung world x,z for FL,FR,RL,RR (cols 20..27) when present.
    rows = {k: [] for k in ("t", "x", "z", "pitch", "vx", "vz", "fz", "fwx", "fwz")}
    rows["wheels"] = []   # per frame: [(x,z) FL, FR, RL, RR]
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for v in r:
            rows["t"].append(float(v[0])); rows["x"].append(float(v[1]))
            rows["z"].append(float(v[2])); rows["pitch"].append(float(v[3]))
            rows["vx"].append(float(v[4])); rows["vz"].append(float(v[5]))
            rows["fz"].append(float(v[6])); rows["fwx"].append(float(v[7]))
            rows["fwz"].append(float(v[8]))
            if len(v) >= 28:
                rows["wheels"].append([(float(v[20 + 2 * k]), float(v[21 + 2 * k]))
                                       for k in range(4)])
            else:
                rows["wheels"].append([])
    return rows


def ramp_profile(x):
    xs, xt, h, xl = 20.0, 24.0, 0.6, 24.4
    if x < xs: return 0.0
    if x < xt: return h * 0.5 * (1.0 - math.cos(math.pi * (x - xs) / (xt - xs)))
    if x < xl: return h
    return 0.0


def car_corners(x, z, fwx, fwz, L=2.6, H=0.5):
    # rectangle centred at (x,z); local +x (forward) aligns with the body-forward
    # vector (fwx,fwz) in the world x-z plane -> continuous heading through 360 deg
    # (avoids the euler-pitch fold at +-90 deg).
    ang = math.atan2(fwz, fwx)
    c, s = math.cos(ang), math.sin(ang)
    pts = [(-L / 2, -H / 2), (L / 2, -H / 2), (L / 2, H / 2), (-L / 2, H / 2)]
    return [(x + dx * c - dz * s, z + dx * s + dz * c) for dx, dz in pts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jump", default=None)
    ap.add_argument("--loop", required=True)
    ap.add_argument("--out", default="/tmp/strut_demo.mp4")
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--label", default="vertical loop")
    args = ap.parse_args()

    loop = load(args.loop)
    jump = load(args.jump) if args.jump else {k: [] for k in loop}
    nj, nl = len(jump["t"]), len(loop["t"])
    frames = [("jump", i) for i in range(nj)] + [("loop", i) for i in range(nl)]

    fig, ax = plt.subplots(figsize=(9, 5))
    fig.patch.set_facecolor("white")

    def draw_car(xs_pts, color):
        poly = plt.Polygon(xs_pts, closed=True, fc=color, ec="#002060", lw=1.5, zorder=5)
        ax.add_patch(poly)

    def draw_wheels(cx, cz, wheels):
        # Side view (x-z): FL/FR overlap (front), RL/RR overlap (rear). Draw all four
        # wheel-centre particles + the strut line from the body to each wheel.
        labels = ("FL", "FR", "RL", "RR")
        cols = ("#111111", "#444444", "#111111", "#444444")
        for (wx, wz), c in zip(wheels, cols):
            ax.plot([cx, wx], [cz, wz], color="#FFA000", lw=1.2, zorder=4)  # strut
            ax.add_patch(Circle((wx, wz), 0.32, fc=c, ec="#000000", lw=0.8, zorder=6))

    def render(fi):
        ax.clear()
        kind, i = frames[fi]
        ax.set_facecolor("white")
        ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]")
        ax.set_aspect("equal")
        if kind == "jump":
            d = jump
            gx = [20 + 0.1 * k for k in range(120)]
            gx = [10 + 0.5 * k for k in range(150)]
            gz = [ramp_profile(v) for v in gx]
            ax.fill_between(gx, [-1] * len(gx), gz, color="#cfd8dc", zorder=0)
            ax.plot(gx, gz, color="#607d8b", lw=1.0, zorder=1)
            x, z = d["x"][i], d["z"][i]
            ax.set_xlim(x - 14, x + 14); ax.set_ylim(-0.5, 6.0)
            trail = max(0, i - 120)
            ax.plot(d["x"][trail:i + 1], d["z"][trail:i + 1],
                    color="#01A0E9", lw=1.2, zorder=2)
            draw_car(car_corners(x, z, d["fwx"][i], d["fwz"][i]), "#005195")
            if d["wheels"][i]:
                draw_wheels(x, z, d["wheels"][i])
            airborne = d["fz"][i] < 30.0
            ax.set_title(f"L5 spatial-strut  |  ramp jump  |  t={d['t'][i]:.2f}s  "
                         f"{'AIRBORNE' if airborne else 'on ground'}",
                         fontsize=11)
        else:
            d = loop
            R, xc, zc = 10.0, 50.0, 15.0
            ax.add_patch(Circle((xc, zc), R, fill=False, ec="#607d8b",
                                 lw=2.0, zorder=0))
            ax.plot([20, xc + R], [0, 0], color="#607d8b", lw=1.0, zorder=0)
            x, z = d["x"][i], d["z"][i]
            ax.set_xlim(xc - 16, xc + 16); ax.set_ylim(-1.0, 28.0)
            trail = max(0, i - 200)
            ax.plot(d["x"][trail:i + 1], d["z"][trail:i + 1],
                    color="#01A0E9", lw=1.2, zorder=2)
            # detached from the wall (lost the loop) when the contact drops out
            on_track = d["fz"][i] > 50.0
            draw_car(car_corners(x, z, d["fwx"][i], d["fwz"][i]),
                     "#DC291E" if on_track else "#888888")
            if d["wheels"][i]:
                draw_wheels(x, z, d["wheels"][i])
            ax.set_title(f"L5 spatial-strut  |  {args.label}  |  t={d['t'][i]:.2f}s"
                         f"  {'' if on_track else '(detached — falling)'}", fontsize=11)
        ax.grid(True, color="#eceff1", zorder=-1)

    step = 2  # decimate frames for a lighter mp4
    idx = list(range(0, len(frames), step))
    anim = FuncAnimation(fig, lambda k: render(idx[k]), frames=len(idx),
                         interval=1000 / args.fps)
    writer = FFMpegWriter(fps=args.fps, bitrate=3000,
                          metadata={"title": "VDSim L5 strut demo"})
    anim.save(args.out, writer=writer, dpi=110)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
