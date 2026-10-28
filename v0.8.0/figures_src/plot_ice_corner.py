"""Task 67: cornering through ice patch."""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/67_ice_corner/figures")
OUT.mkdir(parents=True, exist_ok=True)
df = pd.read_csv("/tmp/ice_corner.csv").iloc[1:]

fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
axes[0].plot(df["t"], df["r"], color="#4F81BD")
axes[0].axvspan(2.1, 4.0, color="lightgray", alpha=0.5, label="ice (mu=0.2)")
axes[0].set_ylabel("yaw rate r [rad/s]")
axes[0].set_title("Cornering through icy patch (sedan, δ=0.06)")
axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)

axes[1].plot(df["t"], df["vy"], color="#DC291E")
axes[1].axvspan(2.1, 4.0, color="lightgray", alpha=0.5)
axes[1].set_ylabel("side-slip vy [m/s]")
axes[1].grid(True, alpha=0.3)

axes[2].plot(df["x"], df["y"], color="#345A8A")
axes[2].set_xlabel("x [m]"); axes[2].set_ylabel("y [m]")
axes[2].set_title("Trajectory")
axes[2].set_aspect("equal", "datalim"); axes[2].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "ice_corner.png", dpi=120); plt.close(fig)

# Stats
pre  = df[(df["t"] > 1.5) & (df["t"] < 2.0)]
ice  = df[(df["t"] > 2.5) & (df["t"] < 3.8)]
post = df[(df["t"] > 4.5) & (df["t"] < 5.5)]
print(f"r pre-ice:  {pre['r'].mean():.3f}")
print(f"r on-ice:   {ice['r'].mean():.3f}  ({100*(ice['r'].mean()-pre['r'].mean())/pre['r'].mean():+.1f}%)")
print(f"r post-ice: {post['r'].mean():.3f}")
print(f"figures -> {OUT}")
