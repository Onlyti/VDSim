"""
Task 22: Quasi-static roll/pitch trace during step steer + brake.
"""

from pathlib import Path
import subprocess, tempfile
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN_SCN = REPO / "build/bin/vdsim_scenario_run"
VEHICLE = REPO / "configs/vehicles/sedan.yaml"
TIRE    = REPO / "configs/tires/default_pacejka.yaml"
OUT = REPO / "docs/tasks/22_roll_pitch_diag/figures"
OUT.mkdir(parents=True, exist_ok=True)

# scenario_run uses L1 bicycle (no roll/pitch).  Use vdsim_l1_vs_l2 to get L2.
BIN_L1L2 = REPO / "build/bin/vdsim_l1_vs_l2"

with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    # step steer
    subprocess.run([str(BIN_L1L2), str(VEHICLE), str(TIRE),
                    str(REPO / "configs/scenarios/step_steer.yaml"), str(td)],
                   check=True, capture_output=True)
    df_st = pd.read_csv(td / "step_steer_L2.csv").iloc[1:]
    # brake (use throttle_brake_sequence)
    subprocess.run([str(BIN_L1L2), str(VEHICLE), str(TIRE),
                    str(REPO / "configs/scenarios/throttle_brake_sequence.yaml"), str(td)],
                   check=True, capture_output=True)
    df_br = pd.read_csv(td / "throttle_brake_sequence_L2.csv").iloc[1:]
    # double lane change
    subprocess.run([str(BIN_L1L2), str(VEHICLE), str(TIRE),
                    str(REPO / "configs/scenarios/double_lane_change.yaml"), str(td)],
                   check=True, capture_output=True)
    df_dlc = pd.read_csv(td / "double_lane_change_L2.csv").iloc[1:]


# Roll during step steer
fig, axes = plt.subplots(1, 2, figsize=(11, 4.0))
axes[0].plot(df_st["t"], np.degrees(df_st["roll"]), color="#4F81BD")
axes[0].set_xlabel("t [s]"); axes[0].set_ylabel("roll [deg]")
axes[0].set_title(f"Step steer  delta=0.05  vx0=10")
axes[0].grid(True, alpha=0.3)

axes[1].plot(df_st["ay"], np.degrees(df_st["roll"]), color="#4F81BD")
axes[1].set_xlabel("ay_body [m/s^2]"); axes[1].set_ylabel("roll [deg]")
axes[1].set_title("Roll vs lateral accel (quasi-static linear)")
axes[1].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "roll_step_steer.png", dpi=120); plt.close(fig)


# Pitch during brake
fig, axes = plt.subplots(1, 2, figsize=(11, 4.0))
axes[0].plot(df_br["t"], np.degrees(df_br["pitch"]), color="#DC291E")
axes[0].axvline(3.0, color="gray", linestyle=":", label="end throttle")
axes[0].axvline(4.0, color="gray", linestyle="--", label="start brake")
axes[0].set_xlabel("t [s]"); axes[0].set_ylabel("pitch [deg]")
axes[0].set_title("Pitch during throttle->coast->brake")
axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=8)

axes[1].plot(df_br["t"], df_br["ax"], color="#DC291E")
axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("ax_body [m/s^2]")
axes[1].set_title("Longitudinal accel")
axes[1].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "pitch_during_brake.png", dpi=120); plt.close(fig)


# RPY trace during DLC
fig, ax = plt.subplots(figsize=(7, 4.2))
ax.plot(df_dlc["t"], np.degrees(df_dlc["roll"]),  color="#4F81BD", label="roll")
ax.plot(df_dlc["t"], np.degrees(df_dlc["pitch"]), color="#DC291E", label="pitch")
ax.set_xlabel("t [s]"); ax.set_ylabel("angle [deg]")
ax.set_title("Quasi-static roll / pitch  -- double-lane-change scenario")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "roll_pitch_dlc.png", dpi=120); plt.close(fig)


print(f"step_steer max roll: {np.degrees(df_st['roll']).abs().max():.2f} deg")
print(f"brake max |pitch|:   {np.degrees(df_br['pitch']).abs().max():.2f} deg")
print(f"DLC roll range:      {np.degrees(df_dlc['roll']).min():+.2f} .. {np.degrees(df_dlc['roll']).max():+.2f} deg")
print(f"figures -> {OUT}")
