#!/usr/bin/env python3
"""VDSim campaign runner — run a list of experiments and collect results.

Usage:
    python3 tools/campaign_runner.py [experiments...] [--out=results/campaign]
    python3 tools/campaign_runner.py accel_brake step_steer oval_lap rough_road
    python3 tools/campaign_runner.py --all           # run every *.yaml in configs/experiments/

Each experiment reads configs/experiments/<name>.yaml (vehicle + tire + level +
map + maneuver + sensors + duration) via Experiment.from_config(), then runs
Simulation which applies the scenario autopilot automatically.

Output:
    <out>/<name>/run.csv          ground-truth + per-wheel time series
    <out>/<name>/metrics.json     scalar reductions
    <out>/summary.csv             one row per experiment, all metrics
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

from vdsim_lab import Simulation  # noqa: E402

DEFAULT_METRICS = ["peak_ay", "vmax", "dist", "cte_rms", "cte_max", "lap_time"]


def run_experiment(name: str, out_dir: Path) -> dict:
    print(f"  [{name}] loading ...", flush=True)
    sim = Simulation(name)
    duration = sim.duration

    t0 = time.monotonic()
    while not sim.done():
        sim.step()
    elapsed = time.monotonic() - t0

    # which metrics are meaningful
    has_path = getattr(sim.exp, "_line", None) is not None
    mnames = [m for m in DEFAULT_METRICS
              if m not in ("cte_rms", "cte_max", "lap_time") or has_path]
    metrics = sim.metrics(mnames)
    metrics["wall_time_s"] = round(elapsed, 2)
    metrics["steps"] = sim._k

    out_dir.mkdir(parents=True, exist_ok=True)
    sim.to_csv(out_dir / "run.csv")
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2))
    try:
        sim.plot(out_dir / "run.png", signals=("vx", "ay", "r", "xy"))
    except Exception:
        pass

    vmax    = metrics.get("vmax",    float("nan"))
    peak_ay = metrics.get("peak_ay", float("nan"))
    print(f"  [{name}] done  {elapsed:.1f}s wall | "
          f"vmax={vmax:.1f} m/s  peak_ay={peak_ay:.2f} m/s²  steps={sim._k}")
    return {"experiment": name, **metrics}


def main():
    p = argparse.ArgumentParser(description="VDSim campaign runner")
    p.add_argument("experiments", nargs="*", help="experiment names (no .yaml extension)")
    p.add_argument("--all", action="store_true", help="run all configs/experiments/*.yaml")
    p.add_argument("--out", default="results/campaign", help="output root directory")
    args = p.parse_args()

    exp_dir = REPO / "configs" / "experiments"
    if args.all:
        names = sorted(f.stem for f in exp_dir.glob("*.yaml"))
    elif args.experiments:
        names = args.experiments
    else:
        p.print_help(); sys.exit(1)

    if not names:
        print("No experiments found in configs/experiments/ — check --all or names."); sys.exit(1)

    out = Path(args.out)
    print(f"\nCampaign: {len(names)} experiment(s)  →  {out}/")
    rows = []
    for name in names:
        try:
            row = run_experiment(name, out / name)
        except Exception as e:
            print(f"  [{name}] FAILED: {e}")
            row = {"experiment": name, "error": str(e)}
        rows.append(row)

    # summary CSV (all experiments, all metrics)
    all_keys = ["experiment"]
    for r in rows:
        for k in r:
            if k not in all_keys:
                all_keys.append(k)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=all_keys, extrasaction="ignore")
        w.writeheader(); w.writerows(rows)
    print(f"\nSummary  →  {out / 'summary.csv'}")


if __name__ == "__main__":
    main()
