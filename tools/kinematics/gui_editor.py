"""
Hardpoint GUI editor — interactive matplotlib widget tool.

Loads a suspension YAML, displays the hardpoints in a 3D side-view, with
sliders for each coordinate.  As you drag a slider, the camber / toe /
track curves on the right panel update live, recomputed via the C++
ISuspensionKinematics native solver (or the Python solver as fallback).

Currently DW only (uses create_dw_native_kinematics).  Other types follow
the same pattern.

Usage:
    python3 tools/kinematics/gui_editor.py \\
        --config configs/suspensions/dw_front_sports.yaml \\
        [--output edited.yaml]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")    # interactive backend
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import numpy as np
import yaml

# Try C++ native solver first (faster), fall back to Python.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "build" / "python"))
try:
    import vdsim
    USE_NATIVE = True
except ImportError:
    USE_NATIVE = False
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dw_3d_solver import DW3DSolver


# Hardpoints we expose as editable for DW:
EDITABLE_POINTS = [
    ("lca.chassis_front",  "LCA chassis front",  (-0.3, 0.5)),
    ("lca.chassis_rear",   "LCA chassis rear",   (-0.4, 0.4)),
    ("lca.knuckle",        "LCA knuckle",        (-0.1, 0.9)),
    ("uca.chassis_front",  "UCA chassis front",  (-0.3, 0.5)),
    ("uca.chassis_rear",   "UCA chassis rear",   (-0.4, 0.4)),
    ("uca.knuckle",        "UCA knuckle",        (-0.1, 0.9)),
    ("tie_rod.rack",       "TR inner (rack)",    (-0.5, 0.5)),
    ("tie_rod.knuckle",    "TR knuckle",         (-0.5, 0.9)),
]
COMP_RANGE = (-0.5, 1.0)


def get_path(d, path):
    for p in path.split("."):
        d = d[p]
    return d


def set_path(d, path, value):
    parts = path.split(".")
    for p in parts[:-1]:
        d = d[p]
    d[parts[-1]] = list(value)


def make_solver(hp_path):
    if USE_NATIVE:
        return vdsim.create_dw_native_kinematics(hp_path)
    with open(hp_path) as f:
        return DW3DSolver(yaml.safe_load(f))


def compute_sweep(solver, n=21, travel_range=0.05):
    travels = np.linspace(-travel_range, travel_range, n)
    if USE_NATIVE:
        out = []
        for t in travels:
            o = solver.compute(t, 0.0)
            out.append((o.camber, o.toe, o.track_change))
        cm = np.array([x[0] for x in out])
        to = np.array([x[1] for x in out])
        tc = np.array([x[2] for x in out])
    else:
        rows = [solver.solve(float(t), 0.0) for t in travels]
        cm = np.array([r["camber"] for r in rows])
        to = np.array([r["toe"] for r in rows])
        tc = np.array([r["track_change"] for r in rows])
    return travels, cm, to, tc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--output", default=None,
                    help="Save edited YAML here (default: overwrite input)")
    ap.add_argument("--coord", choices=["x", "y", "z"], default="z",
                    help="Which coordinate the sliders adjust")
    args = ap.parse_args()

    with open(args.config) as f: hp = yaml.safe_load(f)
    if hp.get("type") != "double_wishbone":
        raise SystemExit("GUI editor currently supports type=double_wishbone")
    out_path = Path(args.output or args.config)

    # Persistent temp YAML to back the solver (it re-reads from file each
    # update; could be made faster by direct API).
    tmp_yaml = tempfile.NamedTemporaryFile(
        suffix=".yaml", mode="w", delete=False)
    yaml.safe_dump(hp, tmp_yaml)
    tmp_yaml.close()
    print(f"[gui] using {'C++ native' if USE_NATIVE else 'Python'} solver")

    fig = plt.figure(figsize=(13, 8))
    gs = fig.add_gridspec(3, 3, height_ratios=[2.5, 1, 1])
    ax_xy = fig.add_subplot(gs[0, 0])    # top view (x-y)
    ax_yz = fig.add_subplot(gs[0, 1])    # side view (y-z)
    ax_curves_cam = fig.add_subplot(gs[0, 2])
    ax_curves_toe = fig.add_subplot(gs[1, 2])
    ax_curves_trk = fig.add_subplot(gs[2, 2])
    # 8 sliders × 3 colums = 24, here we just do "z" by default for compactness
    coord_idx = {"x": 0, "y": 1, "z": 2}[args.coord]
    sliders = []
    for i, (path, label, rng) in enumerate(EDITABLE_POINTS):
        ax_s = fig.add_subplot(gs[1 + (i // 4), i % 4 if i // 4 == 1 else (i % 4) - 1])
        # We squeeze: only use bottom-left 2x4 area for sliders.  Cleaner
        # implementation would use a dedicated panel via add_axes.
        ax_s.remove()
    # Use add_axes for sliders to get exact placement
    slider_axes = []
    for i in range(len(EDITABLE_POINTS)):
        col = i % 4; row = i // 4
        # Bottom 2 rows, 4 columns, leaving rightmost column for curves
        ax_s = fig.add_axes([0.05 + col * 0.18, 0.20 - row * 0.04,
                              0.16, 0.02])
        slider_axes.append(ax_s)
    for i, ((path, label, rng), ax_s) in enumerate(zip(EDITABLE_POINTS, slider_axes)):
        v = get_path(hp, path)[coord_idx]
        slider = Slider(ax_s, f"{label}.{args.coord}", rng[0], rng[1],
                         valinit=v, valstep=0.005)
        sliders.append((path, slider))

    def draw_geometry():
        for ax in (ax_xy, ax_yz):
            ax.clear()
            ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        # Top view (x-y)
        ax_xy.set_title("Top view (x-y)")
        for path, _, _ in EDITABLE_POINTS:
            p = get_path(hp, path)
            ax_xy.plot(p[0], p[1], 'o')
            ax_xy.annotate(path.split('.')[-1][:3],
                           (p[0], p[1]), fontsize=7)
        # LCA, UCA, TR links
        for chain in [
            ["lca.chassis_front", "lca.knuckle"],
            ["lca.chassis_rear",  "lca.knuckle"],
            ["uca.chassis_front", "uca.knuckle"],
            ["uca.chassis_rear",  "uca.knuckle"],
            ["tie_rod.rack",      "tie_rod.knuckle"],
        ]:
            p1 = get_path(hp, chain[0]); p2 = get_path(hp, chain[1])
            ax_xy.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', lw=1)
        ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
        # Side view (y-z)
        ax_yz.set_title("Side view (y-z)")
        for path, _, _ in EDITABLE_POINTS:
            p = get_path(hp, path)
            ax_yz.plot(p[1], p[2], 'o')
            ax_yz.annotate(path.split('.')[-1][:3],
                           (p[1], p[2]), fontsize=7)
        for chain in [
            ["lca.chassis_front", "lca.knuckle"],
            ["lca.chassis_rear",  "lca.knuckle"],
            ["uca.chassis_front", "uca.knuckle"],
            ["uca.chassis_rear",  "uca.knuckle"],
            ["tie_rod.rack",      "tie_rod.knuckle"],
            ["lca.knuckle",       "uca.knuckle"],   # kingpin
        ]:
            p1 = get_path(hp, chain[0]); p2 = get_path(hp, chain[1])
            ax_yz.plot([p1[1], p2[1]], [p1[2], p2[2]], 'k-', lw=1)
        # Wheel circle
        wc = hp["wheel"]["center"]
        r = hp["wheel"]["static_radius"]
        th = np.linspace(0, 2*np.pi, 50)
        ax_yz.plot(wc[1] + r * np.cos(th), wc[2] + r * np.sin(th), 'b-', lw=0.8)
        ax_yz.set_xlabel("y [m]"); ax_yz.set_ylabel("z [m]")

    def draw_curves():
        # Write current hp to temp YAML, build solver, sweep, plot.
        with open(tmp_yaml.name, "w") as f: yaml.safe_dump(hp, f)
        solver = make_solver(tmp_yaml.name)
        travels, cm, to, tc = compute_sweep(solver)
        for ax, y, ylab, color in [
            (ax_curves_cam, np.degrees(cm), "camber [°]", 'r'),
            (ax_curves_toe, np.degrees(to), "toe [°]", 'g'),
            (ax_curves_trk, tc * 1000,    "track [mm]", 'b'),
        ]:
            ax.clear()
            ax.plot(travels * 1000, y, color=color, lw=1.8)
            ax.axhline(0, color='#aaa', lw=0.5)
            ax.axvline(0, color='#aaa', lw=0.5)
            ax.grid(True, alpha=0.3)
            ax.set_ylabel(ylab)
        ax_curves_trk.set_xlabel("wheel travel [mm]")

    def redraw_all(_=None):
        # Apply slider values to hp
        for path, slider in sliders:
            v = list(get_path(hp, path))
            v[coord_idx] = float(slider.val)
            set_path(hp, path, v)
        draw_geometry()
        draw_curves()
        fig.canvas.draw_idle()

    for _, slider in sliders:
        slider.on_changed(redraw_all)

    # Save button
    ax_save = fig.add_axes([0.85, 0.02, 0.10, 0.04])
    btn_save = Button(ax_save, "Save")
    def on_save(_):
        with open(out_path, "w") as f: yaml.safe_dump(hp, f, sort_keys=False)
        print(f"[gui] saved -> {out_path}")
    btn_save.on_clicked(on_save)

    draw_geometry()
    draw_curves()
    plt.subplots_adjust(left=0.05, right=0.97, top=0.96, bottom=0.30,
                        hspace=0.35, wspace=0.30)
    plt.show()

    os.unlink(tmp_yaml.name)


if __name__ == "__main__":
    main()
