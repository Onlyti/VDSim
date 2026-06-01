"""
Task 23: measure body-frame Mz aggregation impact on yaw rate.
Compares pneumatic_trail = 0 (Mz off) vs 0.05 (default).
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
OUT  = REPO / "docs/tasks/23_mz_aggregation/figures"
OUT.mkdir(parents=True, exist_ok=True)

# Two tire configs: with / without Mz
tire_off = """# tire with pneumatic_trail = 0 (Mz disabled)
B_long: 10.0
C_long: 1.65
D_long: 1.0
E_long: 0.97
B_lat:  8.0
C_lat:  1.30
D_lat:  1.0
E_lat: -1.0
mu_nominal: 1.0
Fz_nominal: 4000.0
cornering_stiffness: 80000.0
rolling_resistance:  0.015
combined_slip_enabled: true
pneumatic_trail:     0.0
trail_falloff_alpha: 0.20
"""

tire_on = (REPO / "configs/tires/default_pacejka.yaml").read_text()

scenario = REPO / "configs/scenarios/step_steer.yaml"


def run(tire_text, tag, td):
    tp = td / f"tire_{tag}.yaml"
    tp.write_text(tire_text)
    subprocess.run([str(BIN), str(VEHICLE), str(tp), str(scenario), str(td)],
                   check=True, capture_output=True)
    l1 = pd.read_csv(td / "step_steer_L1.csv").iloc[1:]
    l2 = pd.read_csv(td / "step_steer_L2.csv").iloc[1:]
    return l1, l2


with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    l1_off, l2_off = run(tire_off, "off", td)
    # need fresh dir to avoid overwriting; copy step_steer_L1 first
    import shutil
    bk = td / "off"; bk.mkdir()
    shutil.copy(td / "step_steer_L1.csv", bk / "step_steer_L1.csv")
    shutil.copy(td / "step_steer_L2.csv", bk / "step_steer_L2.csv")
    l1_on, l2_on = run(tire_on, "on", td)

# Plot
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0))
ax1.plot(l1_off["t"], l1_off["r"], color="#345A8A", label="L1 Mz=off", linestyle="--")
ax1.plot(l1_on["t"],  l1_on["r"],  color="#4F81BD", label="L1 Mz=on")
ax1.plot(l2_off["t"], l2_off["r"], color="#7F0000", label="L2 Mz=off", linestyle="--")
ax1.plot(l2_on["t"],  l2_on["r"],  color="#DC291E", label="L2 Mz=on")
ax1.set_xlabel("t [s]"); ax1.set_ylabel("yaw rate r [rad/s]")
ax1.set_title("Step steer: with vs without Mz aggregation")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8, ncol=2)

# Mz contribution as % of (Fy translation moment)
# For a SS left turn at small delta, Fy_f * a + Mz_f ~ Mz_r + ... balance.
delta_r = pd.DataFrame({
    "L1_diff_pct": (l1_off["r"] - l1_on["r"]) / l1_off["r"].abs().clip(1e-6, None) * 100,
    "L2_diff_pct": (l2_off["r"] - l2_on["r"]) / l2_off["r"].abs().clip(1e-6, None) * 100,
    "t": l1_off["t"].values,
})
ax2.plot(delta_r["t"], delta_r["L1_diff_pct"], color="#4F81BD", label="L1 r drop %")
ax2.plot(delta_r["t"], delta_r["L2_diff_pct"], color="#DC291E", label="L2 r drop %")
ax2.set_xlabel("t [s]"); ax2.set_ylabel("(r_off - r_on) / r_off [%]")
ax2.set_title("Mz contribution to yaw rate")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "mz_on_off_step_steer.png", dpi=120)
plt.close(fig)

# Summary
print(f"L1 r_end Mz off = {l1_off['r'].iloc[-1]:.5f}")
print(f"L1 r_end Mz on  = {l1_on['r'].iloc[-1]:.5f}  ({100*(l1_off['r'].iloc[-1]-l1_on['r'].iloc[-1])/l1_off['r'].iloc[-1]:+.2f} %)")
print(f"L2 r_end Mz off = {l2_off['r'].iloc[-1]:.5f}")
print(f"L2 r_end Mz on  = {l2_on['r'].iloc[-1]:.5f}  ({100*(l2_off['r'].iloc[-1]-l2_on['r'].iloc[-1])/l2_off['r'].iloc[-1]:+.2f} %)")
print(f"figures -> {OUT}")
