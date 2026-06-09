"""Task 64: brake distance benchmark for 4 vehicles."""
import subprocess
import tempfile
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN  = REPO / "build/bin/vdsim_scenario_run"
TIRE = REPO / "configs/tires/default_pacejka.yaml"
OUT  = REPO / "docs/tasks/64_brake_distance/figures"
OUT.mkdir(parents=True, exist_ok=True)

vehicles = ["sedan", "sports", "fsk_formula", "race_car"]
colors   = {"sedan":"#4F81BD", "sports":"#DC291E",
            "fsk_formula":"#345A8A", "race_car":"#01A0E9"}

# Hard brake from 20 m/s to stop.
scen = """name: brake_test
initial_vx: 20.0
duration: 4.0
dt: 0.005
mu: 1.0
interpolation: zoh
controls:
  - { t: 0.0, throttle: 0.0, brake: 1.0, steer: 0.0 }
  - { t: 4.0, throttle: 0.0, brake: 1.0, steer: 0.0 }
"""

with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    sc = td / "brake.yaml"; sc.write_text(scen)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    summary = []
    for v in vehicles:
        veh = REPO / f"configs/vehicles/{v}.yaml"
        out = td / f"{v}.csv"
        subprocess.run([str(BIN), str(veh), str(TIRE), str(sc), str(out)],
                       capture_output=True)
        df = pd.read_csv(out).iloc[1:]
        # Find time to 1 m/s
        below = df[df["vx"] < 1.0]
        if len(below) > 0:
            t_stop = below["t"].iloc[0]
            d_stop = below["x"].iloc[0]
        else:
            t_stop, d_stop = float("nan"), float("nan")

        axes[0].plot(df["t"], df["vx"], color=colors[v], label=v)
        axes[1].plot(df["x"], df["vx"], color=colors[v], label=v)
        summary.append({
            "vehicle": v,
            "max_decel": -df["vx"].diff().min() / 0.005,
            "t_stop": t_stop,
            "d_stop": d_stop,
        })
    axes[0].set_xlabel("t [s]"); axes[0].set_ylabel("vx [m/s]")
    axes[0].set_title("Brake from 20 m/s to stop"); axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)
    axes[1].set_xlabel("distance [m]"); axes[1].set_ylabel("vx [m/s]")
    axes[1].set_title("vx vs distance"); axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=9)
    plt.tight_layout()
    plt.savefig(OUT / "brake_distance.png", dpi=120); plt.close(fig)

sdf = pd.DataFrame(summary)
print(sdf.to_string(index=False))
sdf.to_csv(OUT / "brake_summary.csv", index=False)
print(f"figures -> {OUT}")
