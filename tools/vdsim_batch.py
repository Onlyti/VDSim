#!/usr/bin/env python3
"""VDSim experiment batch/campaign runner (headless, parallel).

Expand a campaign YAML — explicit scenarios + parameter sweeps (cartesian grid) +
Monte Carlo — into a flat run list, run each via vdsim_lab.Experiment.from_config
in a process pool, write per-run CSV + a summary metrics table. See
docs/design/BATCH_RUNNER.md.

    python3 tools/vdsim_batch.py run campaign.yaml
    python3 tools/vdsim_batch.py run campaign.yaml --dry      # list expanded runs

Campaign (see configs/experiments/*.yaml for the referenced scenarios):
    name: fdr_vs_mu
    runs:
      - scenario: skidpad
      - sweep:  { base: skidpad, grid: { vehicle.final_drive_ratio: [4,5,6], mu: [0.8,1.0] } }
      - monte_carlo: { base: skidpad, n: 50,
                       vary: { vehicle.mass: {dist: normal, mean: 1500, std: 50} } }
    metrics: [cte_rms, peak_ay, lap_time, max_Fz]
    output: results/fdr_vs_mu
    parallel: 8
    duration: 30
"""
import argparse
import copy
import itertools
import os
import random
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "python"))


def _set(cfg, key, val):
    """Apply a dotted override into a scenario cfg dict."""
    if key.startswith(("vehicle.", "tire.")) or key == "mu":
        cfg.setdefault("_overrides", {})[key] = val
    else:
        d = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = val


def _load_scenario(name_or_cfg):
    if isinstance(name_or_cfg, dict):
        return copy.deepcopy(name_or_cfg)
    return yaml.safe_load(open(REPO / "configs" / "experiments" / f"{name_or_cfg}.yaml"))


def _sample(spec, rng):
    d = spec.get("dist", "uniform")
    if d == "normal":
        return rng.gauss(spec["mean"], spec["std"])
    return rng.uniform(spec["lo"], spec["hi"])


def expand(campaign):
    """campaign['runs'] -> [(run_id, cfg, params), ...]."""
    out = []
    for item in campaign.get("runs", []):
        if "scenario" in item:
            base = _load_scenario(item["scenario"])
            out.append((str(item["scenario"]), base, {}))
        elif "sweep" in item:
            sw = item["sweep"]
            base = _load_scenario(sw["base"])
            keys = list(sw["grid"])
            for combo in itertools.product(*[sw["grid"][k] for k in keys]):
                cfg = copy.deepcopy(base)
                params = dict(zip(keys, combo))
                for k, v in params.items():
                    _set(cfg, k, v)
                tag = "_".join(f"{k.split('.')[-1]}={v}" for k, v in params.items())
                out.append((f"{sw['base']}__{tag}", cfg, params))
        elif "monte_carlo" in item:
            mc = item["monte_carlo"]
            base = _load_scenario(mc["base"])
            for i in range(int(mc["n"])):
                rng = random.Random(1000 + i)
                cfg = copy.deepcopy(base)
                params = {k: _sample(spec, rng) for k, spec in mc.get("vary", {}).items()}
                for k, v in params.items():
                    _set(cfg, k, v)
                out.append((f"{mc['base']}__mc{i:04d}", cfg, params))
    return out


def _run_one(args):
    run_id, cfg, params, metric_names, outdir, duration = args
    import vdsim_lab as lab
    try:
        exp = lab.Experiment.from_config(cfg)
        res = exp.run(duration or None)
        mets = lab.compute_metrics(res, metric_names, getattr(exp, "_line", None))
        res.to_csv(os.path.join(outdir, run_id + ".csv"))
        row = {"run_id": run_id, "ok": 1}
        row.update({k: round(v, 5) for k, v in params.items()
                    if isinstance(v, (int, float))})
        row.update({k: (round(v, 5) if isinstance(v, float) else v) for k, v in mets.items()})
        return row
    except Exception as e:
        return {"run_id": run_id, "ok": 0, "error": str(e)[:150]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run"])
    ap.add_argument("campaign")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--parallel", type=int, default=0)
    a = ap.parse_args()

    camp = yaml.safe_load(open(a.campaign))
    runs = expand(camp)
    print(f"[batch] {camp.get('name','campaign')}: {len(runs)} runs")
    if a.dry:
        for rid, _cfg, params in runs:
            print("  ", rid, params)
        return

    metrics = camp.get("metrics", ["peak_ay", "vmax", "dist"])
    outdir = str(REPO / camp.get("output", "results/batch"))
    os.makedirs(outdir, exist_ok=True)
    duration = camp.get("duration")
    nproc = a.parallel or camp.get("parallel") or os.cpu_count() or 1

    work = [(rid, cfg, params, metrics, outdir, duration) for rid, cfg, params in runs]
    if nproc > 1 and len(work) > 1:
        from multiprocessing import Pool
        with Pool(nproc) as pool:
            rows = pool.map(_run_one, work)
    else:
        rows = [_run_one(w) for w in work]

    cols = ["run_id", "ok"] + [c for c in dict.fromkeys(
        k for r in rows for k in r if k not in ("run_id", "ok"))]
    import csv
    summ = os.path.join(outdir, "summary.csv")
    with open(summ, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    ok = sum(r.get("ok", 0) for r in rows)
    print(f"[batch] {ok}/{len(rows)} ok -> {summ}")


if __name__ == "__main__":
    main()
