#!/usr/bin/env python3
"""Record a 30 s L3 run, then replay it in 3D from the trace file alone.

The two halves are deliberately separate processes' worth of work: once
``run.vdtrace`` exists nothing below re-runs the simulation, which is the whole
claim of the container contract. Delete the record step, keep the file, and the
replay still produces the same video.

    python examples/demo_replay3d.py --out results/replay3d
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python"),
                str(REPO / "build-validation" / "python")]


def record(out: Path, seconds: float, control_dt: float) -> Path:
    """Drive an L3 plant through a slalom over a rough, low-grip patch."""
    from vdsim_plant import VDSimPlant

    trace = out / "run.vdtrace"
    plant = VDSimPlant(config="ioniq5_awd.yaml", control_dt=control_dt,
                       substep_dt=5e-4, base_mu=0.95, level="L3",
                       friction_map=[(120.0, 190.0, 0.45)])
    plant.reset([0.0, 0.0, 0.0, 22.0, 0.0, 0.0])
    dec = plant.enable_trace(trace, seed=20260918, run_id="replay3d",
                             producer={"name": "demo_replay3d.py", "version": "1"},
                             tags={"scenario": "slalom_over_low_mu"})
    n = int(round(seconds / control_dt))
    t0 = time.perf_counter()
    for k in range(n):
        t = k * control_dt
        steer = 0.075 * math.sin(2.0 * math.pi * 0.28 * t)
        fx = 2600.0 if t < 6.0 else (-1800.0 if 18.0 < t < 21.0 else 600.0)
        plant.step([steer, fx])
    wall = time.perf_counter() - t0
    plant.finalize_trace()
    print("recorded %s  (%d steps, decimation %d, %.2f s wall)"
          % (trace, n, dec, wall))
    return trace


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(REPO / "results" / "replay3d"))
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--control-dt", type=float, default=0.005)
    ap.add_argument("--cameras", default="quarter,side,chase")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--rt1", default="force,accel,saturation,normal")
    ap.add_argument("--skip-record", action="store_true",
                    help="replay an existing run.vdtrace (proves the file suffices)")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    trace = out / "run.vdtrace"
    if not args.skip_record or not trace.is_file():
        trace = record(out, args.seconds, args.control_dt)

    import vdsim_render3d as r3

    cams = [c.strip() for c in args.cameras.split(",") if c.strip()]
    tiers = [s.strip() for s in args.rt1.split(",") if s.strip()]
    res = r3.render3d(trace, out, cameras=cams, stride=args.stride,
                      fps=args.fps, tiers=tiers)
    print("\nframes=%d  load+prep=%.2fs  draw=%.2fs  encoder=%s"
          % (res["frames"], res["load_s"], res["draw_s"], res["encoder"]))
    for k, v in res["cameras"].items():
        print("  %-10s %.2f s" % (k, v))
    for p in res["outputs"] + res["previews"]:
        print("  ->", p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
