"""
Plot Task 17 figures: longitudinal weight transfer in L1 bicycle.

Reads CSV from `vdsim_bicycle_run` runs.

Outputs:
    docs/tasks/17_l1_weight_transfer/figures/fz_during_brake.png
    docs/tasks/17_l1_weight_transfer/figures/fz_during_throttle.png
    docs/tasks/17_l1_weight_transfer/figures/fz_axle_summary.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "tasks/17_l1_weight_transfer/figures"
OUT.mkdir(parents=True, exist_ok=True)

RUN = Path("/tmp/vdsim_runs")
GRAVITY = 9.80665
M       = 1500.0
A_F     = 1.20
A_R     = 1.50
L       = 2.70

Fz_f_static = M * GRAVITY * A_R / L
Fz_r_static = M * GRAVITY * A_F / L


def axle_sum(df):
    front = df["Fz_FL"] + df["Fz_FR"]
    rear  = df["Fz_RL"] + df["Fz_RR"]
    total = front + rear
    return front, rear, total


# ---- Brake ----
df = pd.read_csv(RUN / "wt_brake_step.csv").iloc[1:]
front, rear, total = axle_sum(df)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
ax1.plot(df["t"], front, color="#4F81BD", label=f"front axle (static {Fz_f_static:.0f} N)")
ax1.plot(df["t"], rear,  color="#DC291E", label=f"rear axle (static {Fz_r_static:.0f} N)")
ax1.axhline(Fz_f_static, color="#4F81BD", linestyle=":", alpha=0.6)
ax1.axhline(Fz_r_static, color="#DC291E", linestyle=":", alpha=0.6)
ax1.set_xlabel("time [s]"); ax1.set_ylabel("Fz [N]")
ax1.set_title("Brake step (brake = 0.8, vx0 = 20 m/s)")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)

ax2.plot(df["t"], total, color="black", label="Sum (= m g)")
ax2.axhline(M * GRAVITY, color="gray", linestyle=":", label=f"m g = {M*GRAVITY:.1f} N")
ax2.set_xlabel("time [s]"); ax2.set_ylabel("Fz [N]")
ax2.set_title("Mass conservation check")
ax2.set_ylim(M * GRAVITY * 0.997, M * GRAVITY * 1.003)
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "fz_during_brake.png", dpi=120)
plt.close(fig)


# ---- Throttle ----
df = pd.read_csv(RUN / "wt_throttle_step.csv").iloc[1:]
front, rear, total = axle_sum(df)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
ax1.plot(df["t"], front, color="#4F81BD", label="front axle")
ax1.plot(df["t"], rear,  color="#DC291E", label="rear axle")
ax1.axhline(Fz_f_static, color="#4F81BD", linestyle=":", alpha=0.6, label="front static")
ax1.axhline(Fz_r_static, color="#DC291E", linestyle=":", alpha=0.6, label="rear static")
ax1.set_xlabel("time [s]"); ax1.set_ylabel("Fz [N]")
ax1.set_title("Throttle step (throttle = 0.5, vx0 = 5 m/s)")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)

ax2.plot(df["t"], total, color="black")
ax2.axhline(M * GRAVITY, color="gray", linestyle=":")
ax2.set_xlabel("time [s]"); ax2.set_ylabel("Fz [N]")
ax2.set_title("Mass conservation check")
ax2.set_ylim(M * GRAVITY * 0.999, M * GRAVITY * 1.001)
ax2.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "fz_during_throttle.png", dpi=120)
plt.close(fig)


# ---- Summary: peak transfer % vs ax ----
scenarios = ["wt_brake_step", "wt_throttle_step"]
summary = []
for s in scenarios:
    df = pd.read_csv(RUN / f"{s}.csv").iloc[1:]    # skip t=0 (diagnostic uninit)
    front, rear, total = axle_sum(df)
    # estimate average ax over interval
    ax = (df["vx"].diff() / df["t"].diff()).fillna(0)
    summary.append({
        "scenario": s.replace("wt_", ""),
        "ax_peak [m/s^2]": ax.abs().max(),
        "Fz_f_peak/static": front.max() / Fz_f_static,
        "Fz_r_peak/static": rear.max() / Fz_r_static,
        "Fz_f_min/static":  front.min() / Fz_f_static,
        "Fz_r_min/static":  rear.min() / Fz_r_static,
    })
sdf = pd.DataFrame(summary).set_index("scenario")
print(sdf)
sdf.to_csv(OUT / "fz_axle_summary.csv")

fig, ax = plt.subplots(figsize=(9, 3.8))
w = 0.18
xpos = np.arange(len(sdf))
ax.bar(xpos - 1.5*w, sdf["Fz_f_peak/static"], w, label="Fz_f peak / static", color="#4F81BD")
ax.bar(xpos - 0.5*w, sdf["Fz_f_min/static"],  w, label="Fz_f min / static",  color="#345A8A")
ax.bar(xpos + 0.5*w, sdf["Fz_r_peak/static"], w, label="Fz_r peak / static", color="#DC291E")
ax.bar(xpos + 1.5*w, sdf["Fz_r_min/static"],  w, label="Fz_r min / static",  color="#7F0000")
ax.set_xticks(xpos); ax.set_xticklabels(sdf.index)
ax.axhline(1.0, color="gray", linestyle="--", linewidth=0.8)
ax.set_ylabel("Fz / Fz_static  [-]")
ax.set_title("Front / rear axle Fz peak vs static across scenarios")
ax.grid(True, axis="y", alpha=0.3); ax.legend(fontsize=8, ncol=4, loc="upper center")
plt.tight_layout()
plt.savefig(OUT / "fz_axle_summary.png", dpi=120)
plt.close(fig)

print(f"figures -> {OUT}")
