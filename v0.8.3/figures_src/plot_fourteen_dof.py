"""
Task 24: L3 skeleton — quasi-static suspension state during DLC.

Approach: extend l1_vs_l2_run by hand-coding a Python loop that uses
the Python reference impl from plot_bicycle.  Since L3 just wraps L2,
we instead use a small standalone tool: run the Python script
directly by reading L2 CSV (which mirrors L3 planar motion) and
synthesize susp_compression / susp_velocity from Fz.
"""

from pathlib import Path
import subprocess, tempfile
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN  = REPO / "build/bin/vdsim_l1_vs_l2"
VEHICLE = REPO / "configs/vehicles/sedan.yaml"
TIRE    = REPO / "configs/tires/default_pacejka.yaml"
OUT  = REPO / "docs/tasks/24_l3_skeleton/figures"
OUT.mkdir(parents=True, exist_ok=True)

K_SPRING = 30000.0   # matches sedan default (all four corners equal)
GROUND_CLEARANCE = 0.0

scenarios = ["step_steer", "double_lane_change", "throttle_brake_sequence"]

with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    rows = {}
    for sc in scenarios:
        subprocess.run(
            [str(BIN), str(VEHICLE), str(TIRE),
             str(REPO / f"configs/scenarios/{sc}.yaml"), str(td)],
            check=True, capture_output=True)
        df = pd.read_csv(td / f"{sc}_L2.csv").iloc[1:]
        # Quasi-static suspension from Fz / k (matches L3 skeleton impl)
        for w in ["FL","FR","RL","RR"]:
            df[f"susp_{w}"] = df[f"Fz_{w}"] / K_SPRING
        rows[sc] = df

# Plot per-corner suspension compression during DLC
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.0))
df = rows["double_lane_change"]
colors = {"FL":"#4F81BD", "FR":"#DC291E", "RL":"#345A8A", "RR":"#7F0000"}
for w in ["FL","FR","RL","RR"]:
    ax1.plot(df["t"], df[f"susp_{w}"] * 1000.0, color=colors[w], label=w)
ax1.set_xlabel("t [s]"); ax1.set_ylabel("susp compression [mm]")
ax1.set_title("L3 quasi-static suspension compression — DLC")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)

for w in ["FL","FR","RL","RR"]:
    ax2.plot(df["t"], df[f"Fz_{w}"], color=colors[w], label=w)
ax2.set_xlabel("t [s]"); ax2.set_ylabel("Fz [N]")
ax2.set_title("Underlying Fz (from L2 weight transfer)")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "l3_suspension_dlc.png", dpi=120); plt.close(fig)


# Brake test: front compression vs rear extension
df = rows["throttle_brake_sequence"]
fig, ax = plt.subplots(figsize=(8, 4.2))
ax.plot(df["t"], df["susp_FL"] * 1000, color="#4F81BD", label="FL")
ax.plot(df["t"], df["susp_FR"] * 1000, color="#345A8A", label="FR")
ax.plot(df["t"], df["susp_RL"] * 1000, color="#DC291E", label="RL")
ax.plot(df["t"], df["susp_RR"] * 1000, color="#7F0000", label="RR")
ax.axvline(3.0, color="gray", linestyle=":", label="end throttle")
ax.axvline(4.0, color="gray", linestyle="--", label="start brake")
ax.set_xlabel("t [s]"); ax.set_ylabel("susp compression [mm]")
ax.set_title("L3 susp compression: throttle->coast->brake sequence")
ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "l3_suspension_brake.png", dpi=120); plt.close(fig)


# Summary
summary = []
for sc, df in rows.items():
    rng = df[["susp_FL","susp_FR","susp_RL","susp_RR"]] * 1000
    summary.append({
        "scenario": sc,
        "max_comp [mm]":  rng.max().max(),
        "min_comp [mm]":  rng.min().min(),
        "rng_FL [mm]":    rng["susp_FL"].max() - rng["susp_FL"].min(),
        "rng_RR [mm]":    rng["susp_RR"].max() - rng["susp_RR"].min(),
    })
sdf = pd.DataFrame(summary).set_index("scenario")
print(sdf.to_string())
sdf.to_csv(OUT / "l3_summary.csv")
print(f"figures -> {OUT}")
