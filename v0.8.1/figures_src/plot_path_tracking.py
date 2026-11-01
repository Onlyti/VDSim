"""Task 31/32/33: closed-loop path tracking demo (Pure Pursuit + L6/L5)."""
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/31_l6_l7_l8_control/figures")
OUT.mkdir(parents=True, exist_ok=True)

df = pd.read_csv("/tmp/path_track.csv").iloc[1:]

# Reference path
R = 20.0
n = 80
t = np.linspace(0, 2*np.pi, n, endpoint=False)
ref_x1 = R - R*np.cos(t); ref_y1 = R*np.sin(t)
ref_x2 = -R + R*np.cos(t); ref_y2 = R*np.sin(t)

fig, axes = plt.subplots(2, 2, figsize=(12, 9))

# 1: trajectory + reference
ax = axes[0, 0]
ax.plot(ref_x1, ref_y1, "k--", linewidth=0.8, label="reference (figure-8)")
ax.plot(ref_x2, ref_y2, "k--", linewidth=0.8)
ax.plot(df["x"], df["y"], color="#4F81BD", linewidth=1.2, label="vehicle")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Closed-loop trajectory")
ax.set_aspect("equal", "datalim")
ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

# 2: vx tracking
ax = axes[0, 1]
ax.plot(df["t"], df["vx"], color="#4F81BD", label="vx meas")
ax.axhline(8.0, color="black", linestyle="--", label="v_target = 8 m/s")
ax.set_xlabel("t [s]"); ax.set_ylabel("vx [m/s]")
ax.set_title("L6 vx tracking")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)

# 3: steer command
ax = axes[1, 0]
ax.plot(df["t"], df["steer_cmd"], color="#DC291E", label="steer (L7 Pure Pursuit)")
ax.set_xlabel("t [s]"); ax.set_ylabel("steer [rad]")
ax.set_title("Steer command")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)

# 4: cross-track error
ax = axes[1, 1]
ax.plot(df["t"], df["xtrack_err"], color="#345A8A")
ax.set_xlabel("t [s]"); ax.set_ylabel("distance to lookahead [m]")
ax.set_title("Lookahead distance (proxy cross-track)")
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(OUT / "closed_loop_path.png", dpi=120); plt.close(fig)

print(f"vx mean / std: {df['vx'].mean():.2f} +/- {df['vx'].std():.2f}")
print(f"steer max:     {df['steer_cmd'].abs().max():.3f}")
print(f"xtrack mean:   {df['xtrack_err'].mean():.2f} m")
print(f"figures -> {OUT}")
