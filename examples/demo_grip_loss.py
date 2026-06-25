#!/usr/bin/env python3
"""Headless grip-loss demo: closed-loop plant, low-mu patch, GIF output.

    pip install "./vdsim-*.whl[plot]"
    python examples/demo_grip_loss.py

Writes demo_grip_loss.gif (or --out path). Pillow writer only (no ffmpeg).
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle

try:
    from vdsim_plant import VDSimPlant
except ImportError:
    import sys

    _repo = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(_repo / "python"), str(_repo / "build" / "python")]
    from vdsim_plant import VDSimPlant

V0 = 16.7
DT = 0.05
PATCH = (60.0, 160.0, 0.5)
BASE_MU = 0.9
WHEELS = ("FL", "FR", "RL", "RR")
CAPTION = (
    "Deterministic VDSim plant: controller hits an unseen low-μ patch, "
    "tyres saturate past peak (drift>1) — real grip loss, not a soft-clamp."
)


def _run_sim(n_steps: int = 120):
    plant = VDSimPlant(
        config="ioniq5_awd.yaml",
        friction_map=[PATCH],
        base_mu=BASE_MU,
        control_dt=DT,
        substep_dt=5e-4,
    )
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    hist = []
    obs = None
    for k in range(n_steps):
        if k == 0:
            delta, fx = 0.0, 0.0
        else:
            delta = 0.12
            fx = -12000.0
        obs = plant.step([delta, fx])
        hist.append(obs)
    return hist


def _default_out() -> Path:
    repo = Path(__file__).resolve().parents[1]
    assets = repo / "docs" / "assets"
    if assets.is_dir():
        return assets / "demo_grip_loss.gif"
    return Path.cwd() / "demo_grip_loss.gif"


def _make_gif(hist: list, out: Path, fps: int = 12):
    xs = [h["X"] for h in hist]
    ys = [h["Y"] for h in hist]
    x0, x1, mu_lo = PATCH

    fig = plt.figure(figsize=(9.0, 5.0), dpi=100)
    fig.suptitle(CAPTION, fontsize=9.0, y=0.98)
    gs = GridSpec(2, 2, figure=fig, width_ratios=[1.2, 1.0], hspace=0.35, wspace=0.28)

    ax_path = fig.add_subplot(gs[:, 0])
    ax_alpha = fig.add_subplot(gs[0, 1])
    ax_fc = fig.add_subplot(gs[1, 1])

    pad = 15.0
    ylo, yhi = min(ys) - 8, max(ys) + 8
    ax_path.set_xlim(min(xs) - pad, max(xs) + pad)
    ax_path.set_ylim(ylo, yhi)
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.set_xlabel("X [m]")
    ax_path.set_ylabel("Y [m]")
    ax_path.set_title("path + μ patch")
    ax_path.add_patch(Rectangle(
        (x0, ylo), x1 - x0, yhi - ylo,
        facecolor="#c44e52", alpha=0.18, edgecolor="#c44e52", linewidth=1.0, zorder=0,
    ))
    ax_path.text(x0 + 4, yhi - 1.5, f"μ≈{mu_lo:.1f}", color="#8b1a1a", fontsize=8)
    (trail,) = ax_path.plot([], [], "k-", lw=1.0, alpha=0.5)
    (car,) = ax_path.plot([], [], "o", color="#1f77b4", ms=8, zorder=5)

    x_ax = np.arange(4)
    bars = ax_alpha.bar(x_ax, [0.0] * 4, color="#4c72b0", width=0.65)
    peak_lines = ax_alpha.axhline(1.0, color="#555", ls="--", lw=0.8, label="|α|/α_peak=1")
    ax_alpha.set_xticks(x_ax)
    ax_alpha.set_xticklabels(WHEELS)
    ax_alpha.set_ylabel("|α| / α_peak")
    ax_alpha.set_title("slip past peak (drift>1)")
    ax_alpha.set_ylim(0.0, max(2.5, 1.0))
    ax_alpha.legend(loc="upper right", fontsize=7)

    ax_fc.set_xlim(-1.1, 1.1)
    ax_fc.set_ylim(-1.1, 1.1)
    ax_fc.set_aspect("equal")
    ax_fc.set_xlabel("Fx / (μ·Fz)")
    ax_fc.set_ylabel("Fy / (μ·Fz)")
    ax_fc.set_title("FL friction usage (normalised)")
    theta = np.linspace(0, 2 * np.pi, 128)
    (circle,) = ax_fc.plot(np.cos(theta), np.sin(theta), "k--", lw=0.8, alpha=0.6)
    (force_pt,) = ax_fc.plot([], [], "o", color="#d62728", ms=9)

    def _update(i):
        h = hist[i]
        trail.set_data(xs[: i + 1], ys[: i + 1])
        car.set_data([h["X"]], [h["Y"]])
        ymax = 1.0
        for j, w in enumerate(h["wheel"]):
            ap = max(float(w["alpha_peak"]), 1e-4)
            ratio = abs(float(w["alpha"])) / ap
            ymax = max(ymax, ratio * 1.05)
            bars[j].set_height(ratio)
            bars[j].set_color("#d62728" if ratio > 1.0 else "#4c72b0")
        ax_alpha.set_ylim(0.0, min(ymax, 3.0))
        w0 = h["wheel"][0]
        cap = max(float(w0["mu"]) * float(w0["Fz"]), 1.0)
        fx_n = float(w0["Fx"]) / cap
        fy_n = float(w0["Fy"]) / cap
        force_pt.set_data([fx_n], [fy_n])
        return trail, car, *bars, force_pt

    anim = FuncAnimation(fig, _update, frames=len(hist), interval=1000 // fps,
                         blit=False, repeat=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(str(out), writer=PillowWriter(fps=fps), savefig_kwargs={"transparent": False})
    plt.close(fig)
    print(f"wrote {out} ({len(hist)} frames, {len(hist) / fps:.1f}s loop)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=None, help="output GIF path")
    ap.add_argument("--fps", type=int, default=12)
    args = ap.parse_args()
    out = args.out or _default_out()
    hist = _run_sim()
    drift_hit = any(
        abs(w["alpha"]) / max(w["alpha_peak"], 1e-4) > 1.0
        for h in hist for w in h["wheel"]
    )
    if not drift_hit:
        raise SystemExit("expected drift>1 on at least one step (scenario regression)")
    _make_gif(hist, out, fps=args.fps)


if __name__ == "__main__":
    main()
