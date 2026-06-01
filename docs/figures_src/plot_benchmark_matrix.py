"""Task 50: 4 vehicles × 3 scenarios benchmark matrix."""
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/50_benchmark_matrix/figures")
OUT.mkdir(parents=True, exist_ok=True)
BENCH = Path("/tmp/bench")

vehicles = ["sedan", "sports", "fsk_formula", "race_car"]
scenarios = ["step_steer", "double_lane_change", "throttle_brake_sequence"]
colors = {"sedan":"#4F81BD", "sports":"#DC291E",
          "fsk_formula":"#345A8A", "race_car":"#01A0E9"}

# Summary metrics
summary = []
for v in vehicles:
    for sc in scenarios:
        df = pd.read_csv(BENCH / f"{v}_{sc}.csv").iloc[1:]
        summary.append({
            "vehicle": v,
            "scenario": sc,
            "vx0": df["vx"].iloc[0],
            "vx_end": df["vx"].iloc[-1],
            "delta_vx": df["vx"].iloc[-1] - df["vx"].iloc[0],
            "r_peak": df["r"].abs().max(),
            "y_extent": df["y"].abs().max(),
        })
sdf = pd.DataFrame(summary)
sdf.to_csv(OUT / "benchmark_summary.csv", index=False)


# 3x4 grid: scenario rows, vehicle columns
fig, axes = plt.subplots(3, 4, figsize=(15, 9), sharey="row")
for i, sc in enumerate(scenarios):
    for j, v in enumerate(vehicles):
        df = pd.read_csv(BENCH / f"{v}_{sc}.csv").iloc[1:]
        ax = axes[i, j]
        if sc == "throttle_brake_sequence":
            ax.plot(df["t"], df["vx"], color=colors[v])
            ax.set_ylabel("vx [m/s]" if j == 0 else "")
        else:
            ax.plot(df["t"], df["r"], color=colors[v])
            ax.set_ylabel("r [rad/s]" if j == 0 else "")
        ax.grid(True, alpha=0.3)
        if i == 0: ax.set_title(v)
        if i == 2: ax.set_xlabel("t [s]")
fig.suptitle("4 vehicles × 3 scenarios — VDSim benchmark matrix", fontsize=13)
plt.tight_layout()
plt.savefig(OUT / "benchmark_grid.png", dpi=120); plt.close(fig)


# Heatmap of step_steer r_peak across vehicles
fig, ax = plt.subplots(figsize=(8, 3.0))
ssdf = sdf[sdf["scenario"] == "step_steer"]
xs = np.arange(len(vehicles))
ax.bar(xs, ssdf["r_peak"], color=[colors[v] for v in vehicles])
for x, r in zip(xs, ssdf["r_peak"]):
    ax.text(x, r + 0.005, f"{r:.3f}", ha="center", fontsize=9)
ax.set_xticks(xs); ax.set_xticklabels(vehicles)
ax.set_ylabel("yaw rate peak [rad/s]")
ax.set_title("Step steer (δ=0.05) — yaw rate peak by vehicle")
ax.grid(True, axis="y", alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "step_steer_by_vehicle.png", dpi=120); plt.close(fig)

print(sdf.to_string(index=False))
print(f"figures -> {OUT}")
