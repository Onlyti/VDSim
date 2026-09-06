#!/usr/bin/env python3
"""Headless multi-run demo — same manoeuvre at three grip levels, overlaid.

    pip install "./vdsim-*.whl[plot]"
    python examples/demo_compare_runs.py

Same split as ``demo_grip_loss.py``, one step further: each run is recorded to
its own ``.vdtrace``, and the picture is made from the files alone. The three
runs share the controller and the command sequence, so every difference in the
animation is the tyres losing grip — which is exactly the claim a comparison
picture has to support.

Writes docs/assets/demo_compare_runs.gif (or --out) and the traces beside it.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# The in-tree copies win over anything installed (see demo_grip_loss.py).
_repo = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(_repo / "python"), str(_repo / "build" / "python")]

from vdsim_plant import VDSimPlant  # noqa: E402  (path set above)
import vdsim_render  # noqa: E402
import vdsim_trace  # noqa: E402

V0 = 16.7
DT = 0.05
PATCH = (60.0, 160.0, 0.5)
N_STEPS = 120
#: (label, base friction) per run — one dry, one damp, one wet.
RUNS = (("mu 0.90 (dry)", 0.90),
        ("mu 0.65 (damp)", 0.65),
        ("mu 0.45 (wet)", 0.45))
CAPTION = ("Same controller, same commands, three friction levels — overlaid "
           "from three .vdtrace files. Divergence is grip, not tuning.")


def record(trace_path: Path, base_mu: float, run_id: str,
           n_steps: int = N_STEPS) -> Path:
    """Run the identical manoeuvre at one friction level.

    :param trace_path: output ``.vdtrace``.
    :param base_mu: friction outside the low-mu patch.
    :param run_id: manifest ``repro.run_id``; the renderer uses it as the label.
    :param n_steps: control steps to simulate.
    :returns: the written trace path.
    """
    plant = VDSimPlant(
        config="ioniq5_awd.yaml",
        friction_map=[PATCH],
        base_mu=base_mu,
        control_dt=DT,
        substep_dt=5e-4,
    )
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    plant.enable_trace(trace_path, seed=0, run_id=run_id,
                       producer={"name": "demo_compare_runs.py", "version": "0.1"},
                       tags={"scenario": "grip_sweep", "base_mu": base_mu})
    for k in range(n_steps):
        # Identical command sequence in every run: the only independent
        # variable is base_mu.
        plant.step([0.0, 0.0] if k == 0 else [0.12, -12000.0])
    path = plant.finalize_trace()

    x0, x1, mu_lo = PATCH
    vdsim_trace.attach_overlay(path, {
        "kind": "region", "name": "mu_patch", "mu": mu_lo,
        "polygon": [[x0, -30.0], [x1, -30.0], [x1, 30.0], [x0, 30.0]]})
    vdsim_trace.attach_overlay(path, {
        "kind": "path2d", "name": "intended_path",
        "xy": [[float(i) * 4.0, 0.0] for i in range(60)]})
    return path


def _default_out() -> Path:
    repo = Path(__file__).resolve().parents[1]
    assets = repo / "docs" / "assets"
    if assets.is_dir():
        return assets / "demo_compare_runs.gif"
    return Path.cwd() / "demo_compare_runs.gif"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=None, help="output GIF path")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--alpha", type=float, default=vdsim_render.BODY_ALPHA,
                    help="body fill alpha (overlapping vehicles)")
    ap.add_argument("--keep-traces", action="store_true",
                    help="keep the .vdtrace files next to the GIF")
    args = ap.parse_args()

    out = args.out or _default_out()
    traces = []
    for label, base_mu in RUNS:
        path = out.with_name("%s_mu%02d.vdtrace"
                             % (out.stem, int(round(base_mu * 100))))
        traces.append(record(path, base_mu, run_id=label))
        print("recorded %s" % (path,))

    res = vdsim_render.render_multi(
        traces, out=out, fps=args.fps, alpha=args.alpha, title=CAPTION)
    # The runs start together; if they never separate, the demo is not showing
    # what it claims and the picture would be a straight-line non-result.
    if res["max_spread_m"] < 0.5:
        raise SystemExit("runs stayed within %.2f m — grip sweep had no effect "
                         "(scenario regression)" % res["max_spread_m"])
    if not args.keep_traces:
        for path in traces:
            path.unlink()


if __name__ == "__main__":
    main()
