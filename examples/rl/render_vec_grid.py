#!/usr/bin/env python3
"""Render a ``record_vec_demo.py --npz`` rollout: gym tile grid or one-road overlay.

Grid (default): one tile per env, each a bird's-eye view that follows its car
along the straight training road.  When an episode ends the tile shows the
terminal pose with the termination reason; the label stays for ``--label-s``
seconds over the freshly reset car.

Overlay (``--overlay``): every env's *first* episode on one road, cars as
translucent markers with their trails.  An env whose episode has ended keeps
its terminal marker, greyed.  Use with ``--shared-actions`` recordings to see
the spread that reset and domain randomization alone produce.

A frame is drawn only at a recorded control step (``--every`` picks every n-th
one); nothing is interpolated.  Encoding uses the system ``ffmpeg`` (not
bundled); without it the frames are written as PNGs and the command to encode
them is printed.

    python examples/rl/render_vec_grid.py /tmp/rl_grid.npz \\
        --out docs/assets/rl/vec_grid.mp4
    python examples/rl/render_vec_grid.py /tmp/rl_overlay.npz --overlay \\
        --out docs/assets/rl/vec_overlay.mp4
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

from vdsim_rl import TERM_NAMES  # noqa: E402


def body_corners(x: float, y: float, yaw: float, length: float, width: float):
    """Four corners of the body box centred on the CG (ISO 8855, +y left)."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(length / 2, width / 2), (length / 2, -width / 2),
           (-length / 2, -width / 2), (-length / 2, width / 2)]
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in pts]


def caption(meta: dict, n: int, extra: str = "") -> str:
    """Provenance line burnt into every frame."""
    return (f"vdsim_rl VDSimVecEnv  N={n}  vehicle={meta['vehicle']}  "
            f"tire={meta['tire']}  level={meta['level']}  policy={meta['policy']}  "
            f"seed={meta['seed']}  config={meta['config']}  commit={meta['commit']}"
            + extra)


class Sink:
    """Frames -> system ffmpeg (H.264) and/or a PNG directory."""

    def __init__(self, fig, fps: int, out: Path, png_dir):
        self.fig, self.fps, self.out = fig, fps, out
        self.enc = None
        if shutil.which("ffmpeg"):
            w, h = fig.canvas.get_width_height()
            out.parent.mkdir(parents=True, exist_ok=True)
            self.enc = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                 "-pix_fmt", "rgba", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
                 # yuv420p needs even sides; pad by at most 1 px, never rescale
                 "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264",
                 "-pix_fmt", "yuv420p", "-crf", "20", str(out)], stdin=subprocess.PIPE)
        self.png_dir = png_dir if (png_dir or self.enc) else out.with_suffix("")
        if self.png_dir:
            self.png_dir.mkdir(parents=True, exist_ok=True)
        self.n = 0

    def write(self) -> None:
        self.fig.canvas.draw()
        if self.enc is not None:
            self.enc.stdin.write(self.fig.canvas.buffer_rgba())
        if self.png_dir:
            self.fig.savefig(self.png_dir / f"frame_{self.n:05d}.png")
        self.n += 1

    def close(self) -> None:
        if self.enc is not None:
            self.enc.stdin.close()
            self.enc.wait()
            print(f"wrote {self.out} ({self.n} frames @ {self.fps} fps)")
        else:
            print(f"ffmpeg not found; {self.n} PNGs in {self.png_dir}. Encode with:\n"
                  f"  ffmpeg -r {self.fps} -i {self.png_dir}/frame_%05d.png "
                  f"-c:v libx264 -pix_fmt yuv420p {self.out}")


def render_grid(d, meta, args, body_l: float, body_w: float) -> None:
    pose, code, episode, ep_ret = d["pose"], d["code"], d["episode"], d["ep_return"]
    T, N = code.shape
    dt = meta["control_dt_s"]
    edge, lane_y = meta["max_lateral_m"], meta["lane_y_m"]

    rows = math.ceil(N / args.cols)
    fig, axes = plt.subplots(rows, args.cols, figsize=(3.2 * args.cols, 1.9 * rows + 0.6),
                             dpi=args.dpi, squeeze=False)
    fig.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.90,
                        wspace=0.04, hspace=0.18)
    fig.suptitle(caption(meta, N), fontsize=8)
    clock = fig.text(0.99, 0.925, "", ha="right", fontsize=8)
    fig.text(0.01, 0.925, f"grey band = road, edges = off_track limit +-{edge:g} m",
             ha="left", fontsize=8)

    tiles = []
    for i in range(rows * args.cols):
        ax = axes[i // args.cols][i % args.cols]
        ax.set_xticks([]); ax.set_yticks([])
        if i >= N:
            ax.axis("off")
            continue
        ax.set_aspect("equal", adjustable="box")
        ax.set_ylim(lane_y - edge - 1.5, lane_y + edge + 1.5)
        ax.axhspan(lane_y - edge, lane_y + edge, color="0.85", zorder=0)
        ax.axhline(lane_y + edge, color="0.3", lw=1.2, zorder=1)
        ax.axhline(lane_y - edge, color="0.3", lw=1.2, zorder=1)
        ax.axhline(lane_y, color="0.55", lw=0.8, ls=(0, (6, 6)), zorder=1)
        trail, = ax.plot([], [], color="tab:blue", lw=0.8, alpha=0.6, zorder=2)
        car = Polygon(np.zeros((4, 2)), closed=True, fc="tab:blue", ec="k", lw=0.6, zorder=3)
        ax.add_patch(car)
        hud = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", fontsize=7,
                      family="monospace")
        banner = ax.text(0.5, 0.08, "", transform=ax.transAxes, ha="center",
                         va="bottom", fontsize=8, color="white", weight="bold",
                         bbox=dict(fc="tab:red", ec="none", pad=1.5), visible=False)
        tiles.append((ax, trail, car, hud, banner))

    sink = Sink(fig, round(1.0 / (dt * args.every)), args.out, args.png_dir)
    label_steps = int(round(args.label_s / dt))
    last_end = np.full(N, -10 ** 9)
    last_reason = [""] * N
    ep_start = np.zeros(N, dtype=int)
    for k in range(T):
        for i in range(N):
            if k > 0 and code[k, i] != 0:          # terminal pose recorded at k
                last_end[i] = k
                last_reason[i] = TERM_NAMES.get(int(code[k, i]), str(int(code[k, i])))
            if k > 0 and episode[k, i] != episode[k - 1, i]:
                ep_start[i] = k
        if k % args.every:
            continue
        for i, (ax, trail, car, hud, banner) in enumerate(tiles):
            x, y, yaw = pose[k, i]
            car.set_xy(body_corners(x, y, yaw, body_l, body_w))
            seg = slice(ep_start[i], k + 1)
            trail.set_data(pose[seg, i, 0], pose[seg, i, 1])
            ax.set_xlim(x - args.view_behind, x + args.view_ahead)
            ending = code[k, i] != 0 and k > 0
            car.set_facecolor("tab:red" if ending else "tab:blue")
            hud.set_text(f"env {i:02d}  ep {int(episode[k, i])}\nR {ep_ret[k, i]:+7.1f}")
            show = 0 <= k - last_end[i] <= label_steps
            banner.set_visible(bool(show))
            if show:
                banner.set_text(("done: " if ending else "reset <- ") + last_reason[i])
        clock.set_text(f"t = {k * dt:5.2f} s   (frame = recorded step {k})")
        sink.write()
    sink.close()


def render_overlay(d, meta, args, body_l: float, body_w: float) -> None:
    pose, code, episode = d["pose"], d["code"], d["episode"]
    T, N = code.shape
    dt = meta["control_dt_s"]
    edge, lane_y = meta["max_lateral_m"], meta["lane_y_m"]
    # Last step of each env's first episode: its terminal step, else the end.
    end = np.full(N, T - 1)
    reason = [""] * N
    for i in range(N):
        hit = np.flatnonzero((code[1:, i] != 0) & (episode[1:, i] == 0))
        if hit.size:
            end[i] = hit[0] + 1
            reason[i] = TERM_NAMES.get(int(code[end[i], i]), str(int(code[end[i], i])))

    x_all = np.concatenate([pose[:end[i] + 1, i, 0] for i in range(N)])
    x0, x1 = float(x_all.min()) - 5.0, float(x_all.max()) + 8.0
    fig, ax = plt.subplots(figsize=(12.8, 4.6), dpi=args.dpi)
    fig.subplots_adjust(left=0.05, right=0.99, bottom=0.13, top=0.83)
    fig.suptitle(caption(meta, N, "\nfirst episode of every env on one road"),
                 fontsize=8)
    ax.set_xlim(x0, x1)
    ax.set_ylim(lane_y - edge - 1.5, lane_y + edge + 1.5)
    stretch = ((x1 - x0) / (ax.get_position().width * 12.8)) / (
        (2 * edge + 3.0) / (ax.get_position().height * 4.6))
    ax.set_xlabel("x [m] (odom)")
    ax.set_ylabel(f"y [m] (odom)  -- drawn {stretch:.1f}x taller than x")
    ax.axhspan(lane_y - edge, lane_y + edge, color="0.88", zorder=0)
    for yy in (lane_y - edge, lane_y + edge):
        ax.axhline(yy, color="0.3", lw=1.2, zorder=1)
    ax.axhline(lane_y, color="0.55", lw=0.8, ls=(0, (6, 6)), zorder=1)
    colors = plt.get_cmap("turbo")(np.linspace(0.05, 0.95, N))
    trails = [ax.plot([], [], color=colors[i], lw=0.8, alpha=0.35, zorder=2)[0]
              for i in range(N)]
    dots = ax.scatter(np.zeros(N), np.zeros(N), s=28, c=colors, alpha=0.6,
                      edgecolors="k", linewidths=0.3, zorder=3)
    status = ax.text(0.005, 0.97, "", transform=ax.transAxes, va="top", fontsize=8,
                     family="monospace", bbox=dict(fc="white", ec="0.6", alpha=0.85))

    sink = Sink(fig, round(1.0 / (dt * args.every)), args.out, args.png_dir)
    for k in range(0, T, args.every):
        idx = np.minimum(k, end)
        xy = pose[idx, np.arange(N), :2]
        dots.set_offsets(xy)
        ended = k >= end
        ended &= end < T - 1
        fc = colors.copy()
        fc[ended] = (0.45, 0.45, 0.45, 1.0)
        dots.set_facecolors(fc)
        for i in range(N):
            trails[i].set_data(pose[:idx[i] + 1, i, 0], pose[:idx[i] + 1, i, 1])
        counts = {}
        for i in np.flatnonzero(ended):
            counts[reason[i]] = counts.get(reason[i], 0) + 1
        done_txt = ", ".join(f"{n} {r}" for r, n in sorted(counts.items())) or "-"
        status.set_text(f"t = {k * dt:5.2f} s   driving {N - int(ended.sum()):2d}/{N}"
                        f"   ended (grey): {done_txt}")
        sink.write()
    sink.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="output .mp4")
    ap.add_argument("--overlay", action="store_true",
                    help="all envs' first episode on one road instead of tiles")
    ap.add_argument("--png-dir", type=Path, help="also keep the frames as PNGs")
    ap.add_argument("--every", type=int, default=2, help="draw every n-th step")
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--view-ahead", type=float, default=25.0, help="[m]")
    ap.add_argument("--view-behind", type=float, default=10.0, help="[m]")
    ap.add_argument("--label-s", type=float, default=1.0)
    ap.add_argument("--body", type=float, nargs=2, default=None,
                    metavar=("L", "W"), help="body box [m]; default from the vehicle YAML")
    ap.add_argument("--dpi", type=int, default=100)
    args = ap.parse_args(argv)

    d = np.load(args.npz)
    meta = json.loads(str(d["meta"]))
    if args.body is None:
        import yaml
        from vdsim_plant import resolve_vehicle_config
        lwh = yaml.safe_load(resolve_vehicle_config(meta["vehicle"]).read_text())["body_lwh_m"]
        body_l, body_w = float(lwh[0]), float(lwh[1])
    else:
        body_l, body_w = args.body
    (render_overlay if args.overlay else render_grid)(d, meta, args, body_l, body_w)
    return 0


if __name__ == "__main__":
    sys.exit(main())
