"""Task 26: Split-mu accel demo — 3 diff types."""
from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/26_differential/figures")
OUT.mkdir(parents=True, exist_ok=True)

results = {d: pd.read_csv(f"/tmp/split_{d}.csv") for d in ["Open", "LSD", "Locked"]}

fig, (axL, axR) = plt.subplots(1, 2, figsize=(11, 4.2))
for d, color in zip(["Open", "LSD", "Locked"], ["#DC291E", "#345A8A", "#4F81BD"]):
    df = results[d]
    axL.plot(df["t"], df["vx"], color=color, label=d)
    axR.plot(df["t"], df["omega_RL"] - df["omega_RR"], color=color, label=d)
axL.set_xlabel("t [s]"); axL.set_ylabel("vx [m/s]")
axL.set_title("Split-mu accel: vx (RL on mu=0.2, RR on mu=1.0)")
axL.grid(True, alpha=0.3); axL.legend(fontsize=9)

axR.set_xlabel("t [s]"); axR.set_ylabel("omega_RL - omega_RR [rad/s]")
axR.set_title("Wheel speed mismatch")
axR.grid(True, alpha=0.3); axR.legend(fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "split_mu_compare.png", dpi=120); plt.close(fig)

summary = []
for d in ["Open", "LSD", "Locked"]:
    df = results[d]
    summary.append({
        "differential": d,
        "vx_end": df["vx"].iloc[-1],
        "omega_RL_end": df["omega_RL"].iloc[-1],
        "omega_RR_end": df["omega_RR"].iloc[-1],
        "spread_end":   df["omega_RL"].iloc[-1] - df["omega_RR"].iloc[-1],
    })
sdf = pd.DataFrame(summary).set_index("differential")
print(sdf.to_string())
sdf.to_csv(OUT / "split_mu_summary.csv")
print(f"figures -> {OUT}")
