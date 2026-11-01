"""
Plot Task 21 figure: Ackerman steering effect on turning radius.
"""

from pathlib import Path
import subprocess
import tempfile
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN  = REPO / "build/bin/vdsim_l1_vs_l2"     # produces both L1 and L2 CSVs
OUT  = REPO / "docs/tasks/21_ackerman/figures"
OUT.mkdir(parents=True, exist_ok=True)
TIRE = REPO / "configs/tires/default_pacejka.yaml"

# Scenario: tight turn at low speed
sc_yaml = """name: tight_turn
initial_vx: 2.0
duration:   8.0
dt:         0.005
mu:         1.0
interpolation: zoh
controls:
  - { t: 0.0, throttle: 0.0, brake: 0.0, steer: 0.35, gear: 1 }
  - { t: 8.0, throttle: 0.0, brake: 0.0, steer: 0.35, gear: 1 }
"""

# Build vehicle YAML with given ackerman percent
def write_vehicle(path, ack_pct):
    base = (REPO / "configs/vehicles/sedan.yaml").read_text()
    # append/override ackerman_percent
    path.write_text(base + f"\nackerman_percent: {ack_pct}\n")


with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    sc_path = td / "tight_turn.yaml"
    sc_path.write_text(sc_yaml)

    results = {}
    for ack in [0, 50, 100]:
        veh_path = td / f"veh_{ack}.yaml"
        write_vehicle(veh_path, ack)
        # vdsim_l1_vs_l2 emits <scenario_name>_L1.csv and _L2.csv in out_dir
        subprocess.run([str(BIN), str(veh_path), str(TIRE), str(sc_path), str(td)],
                       check=True, capture_output=True)
        l2_csv = td / "tight_turn_L2.csv"
        results[ack] = pd.read_csv(l2_csv).iloc[1:]


# ---- Fig 1: trajectories ----
fig, ax = plt.subplots(figsize=(7, 5.5))
for ack, color in zip([0, 50, 100], ["#DC291E", "#345A8A", "#4F81BD"]):
    df = results[ack]
    ax.plot(df["x"], df["y"], color=color, label=f"Ackerman = {ack} %")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Tight-turn trajectory (vx0 = 2 m/s, delta = 0.35 rad)")
ax.set_aspect("equal", "datalim")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "trajectory_vs_ackerman.png", dpi=120)
plt.close(fig)


# ---- Fig 2: yaw rate ----
fig, ax = plt.subplots(figsize=(7, 4.0))
for ack, color in zip([0, 50, 100], ["#DC291E", "#345A8A", "#4F81BD"]):
    df = results[ack]
    ax.plot(df["t"], df["r"], color=color, label=f"Ackerman = {ack} %")
ax.set_xlabel("t [s]"); ax.set_ylabel("yaw rate r [rad/s]")
ax.set_title("Yaw rate transient vs Ackerman percent")
ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "yaw_rate_vs_ackerman.png", dpi=120)
plt.close(fig)


# ---- Summary ----
summary = []
for ack in [0, 50, 100]:
    df = results[ack]
    r_ss = df["r"].iloc[-1]
    summary.append({
        "ackerman_pct": ack,
        "r_ss [rad/s]": r_ss,
        "turning_radius [m]": (df["vx"].iloc[-1] / r_ss) if abs(r_ss) > 1e-6 else float("nan"),
        "vy(T)": df["vy"].iloc[-1],
    })
sdf = pd.DataFrame(summary)
print(sdf.to_string(index=False))
sdf.to_csv(OUT / "ackerman_summary.csv", index=False)
print(f"figures -> {OUT}")
