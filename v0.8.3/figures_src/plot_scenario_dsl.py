"""
Plot Task 18 figures: 3 example scenarios run via YAML DSL.

Reads CSV from `vdsim_scenario_run` runs.

Outputs:
    docs/tasks/18_scenario_dsl/figures/{scenario}_overview.png
    docs/tasks/18_scenario_dsl/figures/all_scenarios_summary.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "tasks/18_scenario_dsl/figures"
OUT.mkdir(parents=True, exist_ok=True)
RUN = Path("/tmp/vdsim_runs")

scenarios = ["step_steer", "double_lane_change", "throttle_brake_sequence"]
COLOR = "#4F81BD"

for s in scenarios:
    df = pd.read_csv(RUN / f"sc_{s}.csv").iloc[1:]   # drop t=0 diagnostic
    fig, axes = plt.subplots(2, 2, figsize=(11, 6))
    axes[0,0].plot(df["t"], df["vx"], color=COLOR)
    axes[0,0].set_xlabel("t [s]"); axes[0,0].set_ylabel("vx [m/s]")
    axes[0,0].set_title("Longitudinal velocity")
    axes[0,0].grid(True, alpha=0.3)

    axes[0,1].plot(df["t"], df["r"], color="#DC291E")
    axes[0,1].set_xlabel("t [s]"); axes[0,1].set_ylabel("r [rad/s]")
    axes[0,1].set_title("Yaw rate")
    axes[0,1].grid(True, alpha=0.3)

    axes[1,0].plot(df["t"], df["steer"],    label="steer",    color="#345A8A")
    axes[1,0].plot(df["t"], df["throttle"], label="throttle", color="#01A0E9")
    axes[1,0].plot(df["t"], df["brake"],    label="brake",    color="#DC291E")
    axes[1,0].set_xlabel("t [s]"); axes[1,0].set_ylabel("command")
    axes[1,0].set_title("Control profile (from YAML)")
    axes[1,0].grid(True, alpha=0.3); axes[1,0].legend(fontsize=8)

    axes[1,1].plot(df["x"], df["y"], color=COLOR)
    axes[1,1].set_xlabel("x [m]"); axes[1,1].set_ylabel("y [m]")
    axes[1,1].set_title("Trajectory")
    axes[1,1].set_aspect("equal", "datalim")
    axes[1,1].grid(True, alpha=0.3)
    fig.suptitle(s.replace("_", " "), fontsize=12)
    plt.tight_layout()
    plt.savefig(OUT / f"{s}_overview.png", dpi=120)
    plt.close(fig)


# Combined trajectory summary
fig, ax = plt.subplots(figsize=(7, 5))
for s, c in zip(scenarios, ["#4F81BD", "#DC291E", "#345A8A"]):
    df = pd.read_csv(RUN / f"sc_{s}.csv").iloc[1:]
    ax.plot(df["x"], df["y"], label=s.replace("_", " "), color=c)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Trajectories of 3 YAML-defined scenarios (default sedan)")
ax.set_aspect("equal", "datalim")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "all_scenarios_summary.png", dpi=120)
plt.close(fig)

# Final state table
summary = []
for s in scenarios:
    df = pd.read_csv(RUN / f"sc_{s}.csv").iloc[1:]
    summary.append({
        "scenario": s,
        "vx0 [m/s]":      df["vx"].iloc[0],
        "vx(T) [m/s]":    df["vx"].iloc[-1],
        "r peak [rad/s]": df["r"].abs().max(),
        "y peak [m]":     df["y"].abs().max(),
        "x(T) [m]":       df["x"].iloc[-1],
    })
sdf = pd.DataFrame(summary).set_index("scenario")
print(sdf.to_string())
sdf.to_csv(OUT / "scenarios_summary.csv")

print(f"figures -> {OUT}")
