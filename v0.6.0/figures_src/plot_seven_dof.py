"""
Plot Task 19 figures: L1 vs L2 comparison + per-wheel Fz in L2.

Reads CSV from /tmp/vdsim_runs/l2/<scenario>_{L1,L2}.csv

Outputs:
    docs/tasks/19_l2_seven_dof/figures/l1_vs_l2_<scen>.png
    docs/tasks/19_l2_seven_dof/figures/l2_per_wheel_dlc.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "tasks/19_l2_seven_dof/figures"
OUT.mkdir(parents=True, exist_ok=True)
RUN = Path("/tmp/vdsim_runs/l2")

scenarios = ["step_steer", "double_lane_change", "throttle_brake_sequence"]


def load(name, level):
    return pd.read_csv(RUN / f"{name}_{level}.csv").iloc[1:]


# -----------------------------------------------------------------------------
# Fig 1..N: L1 vs L2 side-by-side
# -----------------------------------------------------------------------------
for sc in scenarios:
    df1 = load(sc, "L1")
    df2 = load(sc, "L2")
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
    axes[0].plot(df1["t"], df1["vx"], color="#4F81BD", label="L1 bicycle")
    axes[0].plot(df2["t"], df2["vx"], color="#DC291E", linestyle="--", label="L2 7-DOF")
    axes[0].set_xlabel("t [s]"); axes[0].set_ylabel("vx [m/s]")
    axes[0].set_title("vx"); axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=8)

    axes[1].plot(df1["t"], df1["r"], color="#4F81BD", label="L1")
    axes[1].plot(df2["t"], df2["r"], color="#DC291E", linestyle="--", label="L2")
    axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("r [rad/s]")
    axes[1].set_title("yaw rate"); axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=8)

    axes[2].plot(df1["x"], df1["y"], color="#4F81BD", label="L1")
    axes[2].plot(df2["x"], df2["y"], color="#DC291E", linestyle="--", label="L2")
    axes[2].set_xlabel("x [m]"); axes[2].set_ylabel("y [m]")
    axes[2].set_aspect("equal", "datalim")
    axes[2].set_title("trajectory"); axes[2].grid(True, alpha=0.3); axes[2].legend(fontsize=8)
    fig.suptitle(f"L1 vs L2 — {sc}", fontsize=11)
    plt.tight_layout()
    plt.savefig(OUT / f"l1_vs_l2_{sc}.png", dpi=120)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Per-wheel Fz in L2 for double_lane_change (shows lateral transfer)
# -----------------------------------------------------------------------------
df = load("double_lane_change", "L2")
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0))
for w, c, lbl in [
    ("Fz_FL", "#4F81BD", "FL"),
    ("Fz_FR", "#DC291E", "FR"),
    ("Fz_RL", "#345A8A", "RL"),
    ("Fz_RR", "#7F0000", "RR"),
]:
    ax1.plot(df["t"], df[w], color=c, label=lbl)
ax1.set_xlabel("t [s]"); ax1.set_ylabel("Fz [N]")
ax1.set_title("L2 per-wheel Fz during double-lane-change")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8, ncol=4)

# Mass conservation
total = df["Fz_FL"] + df["Fz_FR"] + df["Fz_RL"] + df["Fz_RR"]
ax2.plot(df["t"], total, color="black")
ax2.axhline(1500 * 9.80665, color="gray", linestyle=":", label="m g")
ax2.set_xlabel("t [s]"); ax2.set_ylabel("Sum Fz [N]")
ax2.set_title("Mass conservation")
ax2.set_ylim(1500 * 9.80665 * 0.997, 1500 * 9.80665 * 1.003)
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "l2_per_wheel_dlc.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
summary = []
for sc in scenarios:
    d1, d2 = load(sc, "L1"), load(sc, "L2")
    summary.append({
        "scenario": sc,
        "L1 vx(T)": d1["vx"].iloc[-1],
        "L2 vx(T)": d2["vx"].iloc[-1],
        "delta vx %": (d2["vx"].iloc[-1] - d1["vx"].iloc[-1]) / d1["vx"].iloc[-1] * 100 if d1["vx"].iloc[-1] > 0.1 else float("nan"),
        "L1 r_peak":  d1["r"].abs().max(),
        "L2 r_peak":  d2["r"].abs().max(),
    })
sdf = pd.DataFrame(summary).set_index("scenario")
print(sdf.to_string())
sdf.to_csv(OUT / "l1_vs_l2_summary.csv")
print(f"figures -> {OUT}")
