"""
Plot Task 14 example-config verification figures.

Reads CSV time-series produced by `vdsim_bicycle_run` for the two vehicles
(sedan, sports) and three scenarios (step_steer, throttle_step, brake_step).

Outputs:
    docs/tasks/14_W4_example_configs/figures/sedan_vs_sports_step_steer.png
    docs/tasks/14_W4_example_configs/figures/sedan_vs_sports_throttle.png
    docs/tasks/14_W4_example_configs/figures/sedan_vs_sports_brake.png
    docs/tasks/14_W4_example_configs/figures/summary_metrics.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "tasks/14_W4_example_configs/figures"
OUT.mkdir(parents=True, exist_ok=True)

RUN_DIR = Path("/tmp/vdsim_runs")

vehicles = ["sedan", "sports"]
colors   = {"sedan": "#4F81BD", "sports": "#DC291E"}


def load(veh, scen):
    return pd.read_csv(RUN_DIR / f"{veh}_{scen}.csv")


# -----------------------------------------------------------------------------
# Fig 1: step_steer — yaw rate transient
# -----------------------------------------------------------------------------
fig, (ax_r, ax_xy) = plt.subplots(1, 2, figsize=(11, 4.2))
for v in vehicles:
    df = load(v, "step_steer")
    ax_r.plot(df["t"], df["r"], color=colors[v], label=v)
    ax_xy.plot(df["x"], df["y"], color=colors[v], label=v)
ax_r.set_xlabel("time [s]"); ax_r.set_ylabel("yaw rate r [rad/s]")
ax_r.set_title("Step steer (delta = 0.05 rad, vx0 = 10 m/s)")
ax_r.grid(True, alpha=0.3); ax_r.legend(fontsize=9)

ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
ax_xy.set_title("Trajectory (world)")
ax_xy.set_aspect("equal", "datalim")
ax_xy.grid(True, alpha=0.3); ax_xy.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "sedan_vs_sports_step_steer.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 2: throttle step — vx(t)
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7, 4.2))
for v in vehicles:
    df = load(v, "throttle_step")
    ax.plot(df["t"], df["vx"], color=colors[v], label=v)
    dvx = df["vx"].iloc[-1] - df["vx"].iloc[0]
    ax.text(df["t"].iloc[-1], df["vx"].iloc[-1] + 0.05,
            f"+{dvx:.2f} m/s", color=colors[v], ha="right", fontsize=9)
ax.set_xlabel("time [s]"); ax.set_ylabel("vx [m/s]")
ax.set_title("Throttle step (throttle = 0.5, vx0 = 5 m/s, no drag)")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "sedan_vs_sports_throttle.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 3: brake step — vx(t)
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7, 4.2))
for v in vehicles:
    df = load(v, "brake_step")
    ax.plot(df["t"], df["vx"], color=colors[v], label=v)
    avg_dec = (df["vx"].iloc[0] - df["vx"].iloc[-1]) / df["t"].iloc[-1]
    ax.text(df["t"].iloc[-1], df["vx"].iloc[-1] - 0.5,
            f"avg decel {avg_dec:.2f}", color=colors[v], ha="right", fontsize=9)
ax.set_xlabel("time [s]"); ax.set_ylabel("vx [m/s]")
ax.set_title("Brake step (brake = 0.8, vx0 = 20 m/s, no drag)")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "sedan_vs_sports_brake.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 4: summary metrics table-as-bars
# -----------------------------------------------------------------------------
metrics = []
for v in vehicles:
    df_st = load(v, "step_steer")
    df_th = load(v, "throttle_step")
    df_bk = load(v, "brake_step")
    metrics.append({
        "vehicle":           v,
        "SS yaw rate r":     df_st["r"].iloc[-1],
        "+vx (throttle 4s)": df_th["vx"].iloc[-1] - df_th["vx"].iloc[0],
        "decel avg (brake 2s)": (df_bk["vx"].iloc[0] - df_bk["vx"].iloc[-1]) / df_bk["t"].iloc[-1],
    })
mdf = pd.DataFrame(metrics).set_index("vehicle")
print(mdf)

fig, axes = plt.subplots(1, 3, figsize=(12, 3.5))
for ax, col in zip(axes, mdf.columns):
    bars = ax.bar(mdf.index, mdf[col], color=[colors[v] for v in mdf.index])
    ax.set_title(col, fontsize=10)
    ax.grid(True, axis="y", alpha=0.3)
    for b, val in zip(bars, mdf[col]):
        ax.text(b.get_x() + b.get_width()/2, val,
                f"{val:.3f}", ha="center", va="bottom", fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "summary_metrics.png", dpi=120)
plt.close(fig)

mdf.to_csv(OUT / "summary_metrics.csv")

print(f"figures -> {OUT}")
