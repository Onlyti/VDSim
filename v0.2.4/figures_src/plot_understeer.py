"""Task 63: understeer gradient characterization for 4 vehicles."""
import subprocess
import tempfile
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path("/home/ailab-12/git/VDSim")
BIN  = REPO / "build/bin/vdsim_scenario_run"
TIRE = REPO / "configs/tires/default_pacejka.yaml"
OUT  = REPO / "docs/tasks/63_understeer/figures"
OUT.mkdir(parents=True, exist_ok=True)

vehicles = ["sedan", "sports", "fsk_formula", "race_car"]
colors   = {"sedan":"#4F81BD", "sports":"#DC291E",
            "fsk_formula":"#345A8A", "race_car":"#01A0E9"}

# Use skidpad-like scenarios at multiple steer angles, fixed vx.
deltas = [0.02, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
vx0 = 10.0

def make_scenario(td, delta):
    path = td / f"sp_{delta:.3f}.yaml"
    path.write_text(f"""name: skidpad_sweep
initial_vx: {vx0}
duration: 7.0
dt: 0.005
mu: 1.0
interpolation: zoh
controls:
  - {{ t: 0.0, throttle: 0.0, brake: 0.0, steer: {delta:.4f} }}
  - {{ t: 7.0, throttle: 0.0, brake: 0.0, steer: {delta:.4f} }}
""")
    return path

with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    fig, ax = plt.subplots(figsize=(8, 5))
    for v in vehicles:
        veh = REPO / f"configs/vehicles/{v}.yaml"
        ays, deltas_seen = [], []
        for d in deltas:
            sc = make_scenario(td, d)
            out = td / f"{v}_{d:.3f}.csv"
            res = subprocess.run([str(BIN), str(veh), str(TIRE), str(sc), str(out)],
                                 capture_output=True, text=True)
            df = pd.read_csv(out).iloc[-1:]   # last sample = SS
            r_ss = float(df["r"].iloc[0])
            vx_ss = float(df["vx"].iloc[0])
            ay_ss = vx_ss * r_ss
            ays.append(abs(ay_ss)); deltas_seen.append(d)
        ax.plot(deltas_seen, ays, "o-", color=colors[v], label=v, linewidth=2)

    ax.set_xlabel("steer δ [rad]"); ax.set_ylabel("steady-state ay [m/s²]")
    ax.set_title(f"Understeer characterization (vx0 = {vx0} m/s)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(OUT / "understeer.png", dpi=120); plt.close(fig)
    print(f"figures -> {OUT}")
