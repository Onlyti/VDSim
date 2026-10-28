"""Task 30: L3 dynamic suspension transient response."""
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/30_l3_dynamic_suspension/figures")
OUT.mkdir(parents=True, exist_ok=True)

scenarios = ["step_steer", "double_lane_change", "throttle_brake_sequence"]
dfs = {s: pd.read_csv(f"/tmp/l3_{s}.csv").iloc[1:] for s in scenarios}

# Step steer: roll transient
df = dfs["step_steer"]
fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
axes[0].plot(df["t"], np.degrees(df["roll"]), color="#4F81BD", label="roll dynamic")
axes[0].set_ylabel("roll [deg]")
axes[0].set_title("L3 dynamic suspension — step steer (sports, vx=10, delta=0.05)")
axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)

axes[1].plot(df["t"], df["susp_vel_FL"]*1000, color="#345A8A", label="FL")
axes[1].plot(df["t"], df["susp_vel_FR"]*1000, color="#DC291E", label="FR")
axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("susp velocity [mm/s]")
axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "step_steer_dyn.png", dpi=120); plt.close(fig)


# DLC: oscillating roll
df = dfs["double_lane_change"]
fig, ax = plt.subplots(figsize=(9, 4.2))
ax.plot(df["t"], np.degrees(df["roll"]), color="#4F81BD", label="dynamic roll")
ax.plot(df["t"], np.degrees(df["ay"]/9.80665), color="gray",
        linestyle=":", label="ay/g [deg-equivalent]")
ax.set_xlabel("t [s]"); ax.set_ylabel("roll [deg]")
ax.set_title("L3 dynamic suspension — DLC roll trace")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "dlc_roll.png", dpi=120); plt.close(fig)


# Brake: pitch transient
df = dfs["throttle_brake_sequence"]
fig, ax = plt.subplots(figsize=(9, 4.2))
ax.plot(df["t"], np.degrees(df["pitch"]), color="#DC291E", label="dynamic pitch")
ax.axvline(3.0, color="gray", linestyle=":", label="end throttle")
ax.axvline(4.0, color="gray", linestyle="--", label="start brake")
ax.set_xlabel("t [s]"); ax.set_ylabel("pitch [deg]")
ax.set_title("L3 dynamic suspension — throttle->coast->brake pitch")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "brake_pitch.png", dpi=120); plt.close(fig)


# Summary
summary = []
for s in scenarios:
    df = dfs[s]
    summary.append({
        "scenario": s,
        "max |roll| [deg]":  np.degrees(df["roll"]).abs().max(),
        "max |pitch| [deg]": np.degrees(df["pitch"]).abs().max(),
        "max |susp_vel| [mm/s]": df[[f"susp_vel_{w}" for w in ["FL","FR","RL","RR"]]].abs().max().max()*1000,
    })
sdf = pd.DataFrame(summary).set_index("scenario")
print(sdf.to_string())
sdf.to_csv(OUT / "l3_dyn_summary.csv")
print(f"figures -> {OUT}")
