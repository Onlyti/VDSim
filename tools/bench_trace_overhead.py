#!/usr/bin/env python3
"""Measure the ``step()`` cost of the trace hook (performance gate, §5).

The gate is on the OFF path: a plant that makes every user pay for a feature
they did not enable has transferred a regression to the customer. Recording is
opt-in and must cost <= 1 % of ``step()`` when off.

The baseline is not a guess — it is the *previous committed revision* of
``python/vdsim_plant.py``, extracted with ``git show`` into a temporary module
and benchmarked in the same process, interleaved with the current one so
machine drift hits both conditions equally.

Run::

    python tools/bench_trace_overhead.py --steps 4000 --repeats 7
"""
from __future__ import annotations

import argparse
import importlib.util
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]


def _load_baseline(rev: str, workdir: Path):
    """Import ``python/vdsim_plant.py`` from ``rev`` as a separate module.

    :param rev: git revision holding the pre-hook plant.
    :param workdir: scratch directory for the extracted source.
    :returns: the imported module, or ``None`` when the revision has no such file.
    """
    out = subprocess.run(["git", "-C", str(REPO), "show",
                          "%s:python/vdsim_plant.py" % rev],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if out.returncode != 0:
        print("baseline unavailable at %s: %s" % (rev, out.stderr.decode().strip()))
        return None
    src = workdir / "vdsim_plant_baseline.py"
    src.write_bytes(out.stdout)
    spec = importlib.util.spec_from_file_location("vdsim_plant_baseline", str(src))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["vdsim_plant_baseline"] = mod
    spec.loader.exec_module(mod)
    return mod


def _run(plant_cls, steps, control_dt, trace_path=None, decimation=None):
    plant = plant_cls(config="ioniq5_awd.yaml", base_mu=0.9,
                      control_dt=control_dt, substep_dt=5e-4)
    plant.reset([0.0, 0.0, 0.0, 16.7, 0.0, 0.0])
    if trace_path is not None:
        plant.enable_trace(trace_path, decimation=decimation)
    u = [0.02, -2000.0]
    t0 = time.perf_counter()
    for _ in range(steps):
        plant.step(u)
    dt = time.perf_counter() - t0
    if trace_path is not None:
        plant.finalize_trace()
    return dt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--steps", type=int, default=4000)
    ap.add_argument("--repeats", type=int, default=7)
    ap.add_argument("--control-dt", type=float, default=0.001)
    ap.add_argument("--baseline-rev", default="HEAD",
                    help="git revision providing the pre-hook plant")
    args = ap.parse_args()

    from vdsim_plant import VDSimPlant

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        base_mod = _load_baseline(args.baseline_rev, td)
        base_cls = getattr(base_mod, "VDSimPlant", None) if base_mod else None

        base, off, on1, ondec = [], [], [], []
        for r in range(args.repeats):
            if base_cls is not None:
                base.append(_run(base_cls, args.steps, args.control_dt))
            off.append(_run(VDSimPlant, args.steps, args.control_dt))
            on1.append(_run(VDSimPlant, args.steps, args.control_dt,
                            td / ("on1_%d.vdtrace" % r), 1))
            ondec.append(_run(VDSimPlant, args.steps, args.control_dt,
                              td / ("ond_%d.vdtrace" % r), None))

    us = 1e6 / args.steps

    def show(label, xs, ref=None):
        med = statistics.median(xs)
        rel = "" if ref is None else "   %+.2f%% vs baseline" % (100.0 * (med / ref - 1.0))
        print("  %-22s %8.3f us/step (min %8.3f)%s" % (label, med * us, min(xs) * us, rel))
        return med

    print("steps=%d repeats=%d control_dt=%g baseline=%s"
          % (args.steps, args.repeats, args.control_dt, args.baseline_rev))
    ref = show("baseline (pre-hook)", base) if base else None
    off_med = show("trace OFF", off, ref)
    show("trace ON  dec=1", on1, ref)
    show("trace ON  dec=auto", ondec, ref)

    if ref is not None:
        pct = 100.0 * (off_med / ref - 1.0)
        print("\nOFF-path overhead vs pre-hook baseline: %+.2f%%  (gate: <= 1%%)  -> %s"
              % (pct, "PASS" if pct <= 1.0 else "FAIL"))
        return 0 if pct <= 1.0 else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
