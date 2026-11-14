"""Task 57-58-59: Driver demo + Skidpad analytical + Brake-in-turn."""
import subprocess
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
OUT = REPO / "docs/tasks/57_58_59_driver_skidpad/figures"
OUT.mkdir(parents=True, exist_ok=True)

# Driver demo
df_drv = pd.read_csv("/tmp/driver.csv").iloc[1:]
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
# Trajectory
R = 20.0
t_ref = np.linspace(0, 2*np.pi, 80, endpoint=False)
axes[0].plot(R - R*np.cos(t_ref), R*np.sin(t_ref), "k--", linewidth=0.6, label="reference")
axes[0].plot(-R + R*np.cos(t_ref), R*np.sin(t_ref), "k--", linewidth=0.6)
axes[0].plot(df_drv["x"], df_drv["y"], color="#4F81BD", linewidth=1.0, label="driver+L2")
axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]")
axes[0].set_title("Closed-loop driver model on figure-8")
axes[0].set_aspect("equal", "datalim"); axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=8)
# Steer noise
axes[1].plot(df_drv["t"], df_drv["steer"], color="#DC291E", linewidth=0.7)
axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("steer [rad]")
axes[1].set_title("Steer command (noisy + 150ms delayed)")
axes[1].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "driver_demo.png", dpi=120); plt.close(fig)

# Skidpad analytical comparison
# r_ana = vx * tan(delta) / L ~ vx * delta / L  (small delta linear)
# For skidpad.yaml: delta=0.27, L=2.7
subprocess.run([str(REPO/"build/bin/vdsim_scenario_run"),
                str(REPO/"configs/vehicles/sedan.yaml"),
                str(REPO/"configs/tires/default_pacejka.yaml"),
                str(REPO/"configs/scenarios/skidpad.yaml"),
                "/tmp/skidpad.csv"], check=True, capture_output=True)
df_sp = pd.read_csv("/tmp/skidpad.csv").iloc[1:]
fig, ax = plt.subplots(figsize=(7, 4))
ax.plot(df_sp["t"], df_sp["r"], color="#4F81BD", label="r_sim")
# Quasi-static analytical: vx changes; recompute over time
delta = 0.27; L = 2.7
r_ana = df_sp["vx"] * np.tan(delta) / L
ax.plot(df_sp["t"], r_ana, "k--", label="r = vx tan(δ)/L (kinematic)")
ax.set_xlabel("t [s]"); ax.set_ylabel("r [rad/s]")
ax.set_title("Skidpad: yaw rate vs kinematic Ackerman")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "skidpad_validation.png", dpi=120); plt.close(fig)
print(f"skidpad r at end: sim={df_sp['r'].iloc[-1]:.3f}, ana={r_ana.iloc[-1]:.3f}")

# Brake-in-turn
subprocess.run([str(REPO/"build/bin/vdsim_scenario_run"),
                str(REPO/"configs/vehicles/sedan.yaml"),
                str(REPO/"configs/tires/default_pacejka.yaml"),
                str(REPO/"configs/scenarios/brake_in_turn.yaml"),
                "/tmp/bit.csv"], check=True, capture_output=True)
df_bit = pd.read_csv("/tmp/bit.csv").iloc[1:]
fig, axes = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
axes[0].plot(df_bit["t"], df_bit["vx"], color="#4F81BD")
axes[0].axvline(2.1, color="red", linestyle="--", label="brake start")
axes[0].set_ylabel("vx [m/s]"); axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)
axes[1].plot(df_bit["t"], df_bit["r"], color="#DC291E")
axes[1].axvline(2.1, color="red", linestyle="--")
axes[1].set_ylabel("yaw rate r [rad/s]"); axes[1].set_xlabel("t [s]")
axes[1].grid(True, alpha=0.3)
axes[0].set_title("Brake-in-turn: combined-slip lateral grip loss")
plt.tight_layout()
plt.savefig(OUT / "brake_in_turn.png", dpi=120); plt.close(fig)

# Compute pre-brake r and during-brake r
pre  = df_bit[df_bit["t"] < 2.0]
post = df_bit[df_bit["t"] > 3.0]
print(f"r before brake: {pre['r'].iloc[-1]:.3f}, during brake: {post['r'].iloc[-1]:.3f}")
print(f"figures -> {OUT}")
