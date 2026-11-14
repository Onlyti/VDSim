"""Task 25: closed-loop ax tracking demo."""
from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path("/home/ailab-12/git/VDSim")
OUT = REPO / "docs/tasks/25_l5_controlconverter/figures"
OUT.mkdir(parents=True, exist_ok=True)

df = pd.read_csv("/tmp/ax_track.csv")

fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
axes[0].plot(df["t"], df["ax_target"], color="black", linestyle="--", label="ax_target")
axes[0].plot(df["t"], df["ax_meas"],   color="#4F81BD", label="ax_meas")
axes[0].set_ylabel("ax [m/s^2]")
axes[0].set_title("Closed-loop ax tracking (L5 PID controller)")
axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)

axes[1].plot(df["t"], df["throttle"], color="#01A0E9", label="throttle")
axes[1].plot(df["t"], df["brake"],    color="#DC291E", label="brake")
axes[1].set_ylabel("command")
axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=9)

axes[2].plot(df["t"], df["vx"], color="#345A8A")
axes[2].set_xlabel("t [s]"); axes[2].set_ylabel("vx [m/s]")
axes[2].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "ax_tracking.png", dpi=120)
plt.close(fig)

# Tracking RMSE on each phase
phases = [(0, 2.0), (2.0, 5.0), (5.0, 8.0), (8.0, 10.0)]
for t0, t1 in phases:
    seg = df[(df["t"] >= t0) & (df["t"] < t1)]
    err = seg["ax_target"] - seg["ax_meas"]
    print(f"  phase [{t0:.1f}, {t1:.1f}): "
          f"target={seg['ax_target'].iloc[0]:+.2f}, "
          f"meas mean={seg['ax_meas'].mean():+.2f}, RMSE={np.sqrt((err**2).mean()):.2f}")

print(f"figures -> {OUT}")
