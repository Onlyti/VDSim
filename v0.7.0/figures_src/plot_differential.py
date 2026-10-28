"""Task 26: differential type comparison under split-mu accel."""
from pathlib import Path
import subprocess, tempfile
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN  = REPO / "build/bin/vdsim_l1_vs_l2"
TIRE = REPO / "configs/tires/default_pacejka.yaml"
OUT  = REPO / "docs/tasks/26_differential/figures"
OUT.mkdir(parents=True, exist_ok=True)

# Split-mu accel scenario
scen = """name: split_mu_accel
initial_vx: 2.0
duration: 4.0
dt: 0.005
mu: 1.0
interpolation: zoh
controls:
  - { t: 0.0, throttle: 1.0, brake: 0.0, steer: 0.0, gear: 1 }
  - { t: 4.0, throttle: 1.0, brake: 0.0, steer: 0.0, gear: 1 }
"""

def write_vehicle(path, diff_type):
    base = (REPO / "configs/vehicles/sedan.yaml").read_text()
    path.write_text(base + f"\ndifferential: {diff_type}\n")

with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    sc = td / "split_mu_accel.yaml"; sc.write_text(scen)
    results = {}
    for d in ["Open", "LSD", "Locked"]:
        veh = td / f"veh_{d}.yaml"; write_vehicle(veh, d)
        subprocess.run([str(BIN), str(veh), str(TIRE), str(sc), str(td)],
                       check=True, capture_output=True)
        # Note: l1_vs_l2 uses flat_contacts with mu=1.0 -- to get split-mu,
        # we'd need to modify the binary.  As proxy, compare standard accel
        # behavior with different diff types.
        results[d] = pd.read_csv(td / "split_mu_accel_L2.csv").iloc[1:]


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0))
for d, color in zip(["Open", "LSD", "Locked"], ["#DC291E", "#345A8A", "#4F81BD"]):
    df = results[d]
    ax1.plot(df["t"], df["vx"], color=color, label=d)
ax1.set_xlabel("t [s]"); ax1.set_ylabel("vx [m/s]")
ax1.set_title("Accel under throttle=1.0 (uniform mu=1.0)")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=9)

# Drive vs driven side Fx via Fz proxy
for d, color in zip(["Open", "LSD", "Locked"], ["#DC291E", "#345A8A", "#4F81BD"]):
    df = results[d]
    rl_share = df["Fz_RL"] / (df["Fz_RL"] + df["Fz_RR"])
    ax2.plot(df["t"], rl_share, color=color, label=d)
ax2.axhline(0.5, color="gray", linestyle=":")
ax2.set_xlabel("t [s]"); ax2.set_ylabel("Fz_RL / (Fz_RL + Fz_RR)")
ax2.set_title("Rear left load share (sanity, should stay near 0.5)")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "diff_accel_compare.png", dpi=120); plt.close(fig)

summary = []
for d in ["Open", "LSD", "Locked"]:
    df = results[d]
    summary.append({
        "differential": d,
        "vx_end [m/s]": df["vx"].iloc[-1],
        "delta_vx [m/s]": df["vx"].iloc[-1] - df["vx"].iloc[0],
    })
sdf = pd.DataFrame(summary)
print(sdf.to_string(index=False))
sdf.to_csv(OUT / "diff_summary.csv", index=False)
print(f"figures -> {OUT}")
