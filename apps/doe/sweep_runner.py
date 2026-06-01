"""
Design-of-Experiments / parameter sweep runner.

Reads a sweep config YAML:

    base:
      vehicle: configs/vehicles/sports.yaml
      tire:    configs/tires/default_pacejka.yaml
      solver:  configs/solvers/default.yaml
      level:   L2     # L1 / L2 / L3
    parameters:
      vehicle.cg_height:        [0.40, 0.45, 0.50, 0.55]
      tire.cornering_stiffness: [60000, 80000, 100000]
    scenarios:
      step_30deg_at_25:
        type: step_steer
        params: {v_target: 25, steer_deg: 30, t_pre: 0.5, t_post: 4.0}
    metrics:
      - peak_yaw_rate
      - yaw_overshoot
      - yaw_settling_time
      - peak_lateral_g
    output:
      csv:  sweep_results.csv
      plots_dir: sweep_plots

Produces:
  - CSV with one row per (parameter combination × scenario)
  - 1-D line plots if only one parameter varies, 2-D heatmaps for two,
    sensitivity bar charts for more.
"""
from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import vdsim
from scenarios import SCENARIO_REGISTRY
from metrics import METRIC_REGISTRY


def make_dynamics(level: str):
    if level == "L1": return vdsim.create_bicycle()
    if level == "L3": return vdsim.create_fourteen_dof()
    return vdsim.create_seven_dof()


def load_base(cfg):
    base = cfg["base"]
    vp = vdsim.VehicleParams.from_yaml(str(REPO / base["vehicle"]))
    tp = vdsim.TireParams.from_yaml(str(REPO / base["tire"]))
    sp_path = base.get("solver")
    sp = (vdsim.SolverParams.from_yaml(str(REPO / sp_path))
          if sp_path else vdsim.SolverParams())
    return vp, tp, sp, base.get("level", "L2")


def apply_param_override(vp, tp, sp, path: str, value):
    """Set a parameter via dotted path.  Supports prefixes:
       vehicle.*, tire.*, solver.*  (writes to the corresponding pybind11
       VehicleParams/TireParams/SolverParams object)."""
    head, _, rest = path.partition(".")
    if head == "vehicle":
        if hasattr(vp, rest):
            setattr(vp, rest, value)
        else:
            raise ValueError(f"VehicleParams has no attribute '{rest}'")
    elif head == "tire":
        if hasattr(tp, rest):
            setattr(tp, rest, value)
        else:
            raise ValueError(f"TireParams has no attribute '{rest}'")
    elif head == "solver":
        if hasattr(sp, rest):
            setattr(sp, rest, value)
        else:
            raise ValueError(f"SolverParams has no attribute '{rest}'")
    else:
        raise ValueError(f"Unknown parameter prefix '{head}' in '{path}'")


def run_one(level, vp, tp, sp, scenario_def):
    dyn = make_dynamics(level)
    dyn.initialize(vp, tp, sp)
    sname = scenario_def["type"]
    sparams = scenario_def.get("params", {})
    if sname not in SCENARIO_REGISTRY:
        raise ValueError(f"Unknown scenario: {sname}")
    traj = SCENARIO_REGISTRY[sname](dyn, sparams)
    return traj, sparams


def compute_metrics(traj, sparams, vp, metric_names):
    out = {}
    for m in metric_names:
        if m not in METRIC_REGISTRY:
            out[m] = None; continue
        fn = METRIC_REGISTRY[m]
        try:
            if "understeer_gradient" in m:
                out[m] = float(fn(traj, sparams, vehicle_params=vp))
            else:
                out[m] = float(fn(traj, sparams))
        except Exception as e:
            print(f"  metric '{m}' failed: {e}")
            out[m] = None
    return out


def run_sweep(cfg: dict) -> pd.DataFrame:
    vp0, tp0, sp0, level = load_base(cfg)
    metric_names = cfg.get("metrics", list(METRIC_REGISTRY.keys()))
    param_paths  = list(cfg["parameters"].keys())
    param_values = [cfg["parameters"][p] for p in param_paths]
    scenarios    = cfg.get("scenarios", {})

    rows = []
    n_combos = 1
    for vs in param_values: n_combos *= len(vs)
    print(f"[doe] {n_combos} combos × {len(scenarios)} scenarios "
          f"= {n_combos * len(scenarios)} runs")

    for combo_idx, combo in enumerate(itertools.product(*param_values)):
        # Rebuild base copies each combo
        vp = vdsim.VehicleParams.from_yaml(str(REPO / cfg["base"]["vehicle"]))
        tp = vdsim.TireParams.from_yaml(str(REPO / cfg["base"]["tire"]))
        sp = (vdsim.SolverParams.from_yaml(str(REPO / cfg["base"]["solver"]))
              if cfg["base"].get("solver") else vdsim.SolverParams())
        for path, val in zip(param_paths, combo):
            apply_param_override(vp, tp, sp, path, val)

        for sname, sdef in scenarios.items():
            traj, sparams = run_one(level, vp, tp, sp, sdef)
            metrics = compute_metrics(traj, sparams, vp, metric_names)
            row = {p: v for p, v in zip(param_paths, combo)}
            row["scenario"] = sname
            row.update(metrics)
            rows.append(row)

        if (combo_idx + 1) % max(1, n_combos // 10) == 0:
            print(f"  {combo_idx + 1}/{n_combos} combos done")

    return pd.DataFrame(rows)


def plot_results(df: pd.DataFrame, param_paths, metric_names, plots_dir: Path,
                  scenario_filter=None):
    plots_dir.mkdir(parents=True, exist_ok=True)
    if scenario_filter is not None:
        df = df[df["scenario"] == scenario_filter]
    if df.empty: return

    sweeping = [p for p in param_paths if df[p].nunique() > 1]

    for metric in metric_names:
        if metric not in df.columns: continue
        # Skip non-numeric or all-None metrics
        if df[metric].isna().all(): continue

        if len(sweeping) == 1:
            # 1-D line plot
            p = sweeping[0]
            fig, ax = plt.subplots(figsize=(7, 4))
            sub = df.groupby(p)[metric].mean().reset_index()
            ax.plot(sub[p], sub[metric], "o-", lw=1.6)
            ax.set_xlabel(p); ax.set_ylabel(metric)
            ax.grid(True, alpha=0.3)
            ax.set_title(f"{metric} vs {p}")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{metric}_vs_{p.replace('.', '_')}.png",
                         dpi=120)
            plt.close(fig)
        elif len(sweeping) == 2:
            # 2-D heatmap
            p1, p2 = sweeping
            pivot = df.pivot_table(values=metric, index=p2, columns=p1,
                                    aggfunc="mean")
            fig, ax = plt.subplots(figsize=(8, 5))
            im = ax.imshow(pivot.values, aspect="auto", origin="lower",
                            extent=[pivot.columns.min(), pivot.columns.max(),
                                    pivot.index.min(), pivot.index.max()],
                            cmap="viridis")
            fig.colorbar(im, label=metric)
            ax.set_xlabel(p1); ax.set_ylabel(p2)
            ax.set_title(f"{metric} (heatmap)")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{metric}_heatmap.png", dpi=120)
            plt.close(fig)
        else:
            # 3+ params: sensitivity = std / mean per parameter
            sensitivity = {}
            for p in sweeping:
                stats = df.groupby(p)[metric].mean()
                m = stats.mean()
                if abs(m) > 1e-9:
                    sensitivity[p] = stats.std() / abs(m)
                else:
                    sensitivity[p] = 0.0
            fig, ax = plt.subplots(figsize=(8, 4))
            keys = list(sensitivity.keys()); vals = list(sensitivity.values())
            ax.barh(keys, vals)
            ax.set_xlabel(f"relative sensitivity of {metric}")
            ax.set_title(f"Sensitivity for {metric}")
            fig.tight_layout()
            fig.savefig(plots_dir / f"{metric}_sensitivity.png", dpi=120)
            plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    df = run_sweep(cfg)

    out_dir = Path(cfg.get("output", {}).get("dir") or
                   Path(args.config).parent / "results")
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / cfg.get("output", {}).get("csv", "sweep_results.csv")
    df.to_csv(csv_path, index=False)
    print(f"[doe] -> {csv_path}  ({len(df)} rows)")

    plots_dir = out_dir / "plots"
    param_paths = list(cfg["parameters"].keys())
    metric_names = cfg.get("metrics", list(METRIC_REGISTRY.keys()))
    for sname in cfg.get("scenarios", {}):
        plot_results(df, param_paths, metric_names,
                     plots_dir / sname, scenario_filter=sname)
    print(f"[doe] plots -> {plots_dir}")


if __name__ == "__main__":
    main()
