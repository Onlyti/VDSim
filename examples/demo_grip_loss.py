#!/usr/bin/env python3
"""Headless grip-loss demo — records a trace, then renders it.

    pip install "./vdsim-*.whl[plot]"
    python examples/demo_grip_loss.py

The demo is deliberately split in two halves that never talk to each other
except through one file:

1. run the closed-loop plant with recording on, producing ``demo_grip_loss.vdtrace``
2. render that file

Nothing in step 2 knows the scenario. The low-mu patch and the reference path
reach the picture as overlays (contract §4), not as renderer knowledge, which
is what makes the demo a self-check of the trace contract rather than a
demonstration that happens to use it.

Writes docs/assets/demo_grip_loss.gif (or --out) and the trace beside it.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# The in-tree copies win over anything installed. An older site-packages
# vdsim_plant has no trace hook, and vdsim_render/vdsim_trace live only here,
# so a partial install would otherwise half-resolve and fail at the second
# import rather than at the first.
_repo = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_repo / "python"), str(_repo / "build" / "python")]

from vdsim_plant import VDSimPlant  # noqa: E402  (path set above)
import vdsim_render  # noqa: E402
import vdsim_trace  # noqa: E402

V0 = 16.7
DT = 0.05
PATCH = (60.0, 160.0, 0.5)
BASE_MU = 0.9
N_STEPS = 120
CAPTION = (
    "Deterministic VDSim plant: controller hits an unseen low-μ patch, "
    "tyres saturate (utilization -> 1) — real grip loss, not a soft-clamp."
)


def record(trace_path: Path, n_steps: int = N_STEPS) -> Path:
    """Run the plant with tracing on and attach the scenario overlays.

    :param trace_path: output ``.vdtrace``.
    :param n_steps: control steps to simulate.
    :returns: the written trace path.
    """
    plant = VDSimPlant(
        config="ioniq5_awd.yaml",
        friction_map=[PATCH],
        base_mu=BASE_MU,
        control_dt=DT,
        substep_dt=5e-4,
    )
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    plant.enable_trace(trace_path, seed=0, run_id="demo_grip_loss",
                       producer={"name": "demo_grip_loss.py", "version": "0.1"},
                       tags={"scenario": "grip_loss"})
    peak_drift = 0.0
    for k in range(n_steps):
        u = [0.0, 0.0] if k == 0 else [0.12, -12000.0]
        obs = plant.step(u)
        for w in obs["wheel"]:
            peak_drift = max(peak_drift, abs(w["alpha"]) / max(w["alpha_peak"], 1e-4))
    path = plant.finalize_trace()
    if peak_drift <= 1.0:
        raise SystemExit(
            "expected slip past peak on at least one wheel, got %.2f "
            "(scenario regression)" % peak_drift)

    # Scenario knowledge enters as overlays. VDSim stores them and never reads
    # them; the renderer draws them by `kind`, not by name.
    x0, x1, mu_lo = PATCH
    vdsim_trace.attach_overlay(path, {
        "kind": "region", "name": "mu_patch", "mu": mu_lo,
        "polygon": [[x0, -30.0], [x1, -30.0], [x1, 30.0], [x0, 30.0]]})
    vdsim_trace.attach_overlay(path, {
        "kind": "path2d", "name": "intended_path",
        "xy": [[float(i) * 4.0, 0.0] for i in range(60)]})
    print("recorded %s (peak slip/peak-slip ratio %.2f)" % (path, peak_drift))
    return path


def _default_out() -> Path:
    repo = Path(__file__).resolve().parents[1]
    assets = repo / "docs" / "assets"
    if assets.is_dir():
        return assets / "demo_grip_loss.gif"
    return Path.cwd() / "demo_grip_loss.gif"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=None, help="output GIF path")
    ap.add_argument("--trace", type=Path, default=None, help="output .vdtrace path")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--keep-trace", action="store_true",
                    help="keep the .vdtrace next to the GIF")
    args = ap.parse_args()

    out = args.out or _default_out()
    trace_path = args.trace or out.with_suffix(".vdtrace")
    record(trace_path)
    vdsim_render.render(trace_path, out=out, fps=args.fps, stride=1,
                        title=CAPTION)
    if not args.keep_trace and args.trace is None:
        trace_path.unlink()


if __name__ == "__main__":
    main()
