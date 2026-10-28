"""
Task 20: (vx, delta) grid sweep comparing L1 and L2 SS yaw rate, vy,
per-tire Fz balance.

Drives `bin/vdsim_l1_vs_l2` via dynamically generated scenario YAML.

Outputs:
    docs/tasks/20_l1_l2_grid_sweep/figures/sweep_yaw_rate_diff.png
    docs/tasks/20_l1_l2_grid_sweep/figures/sweep_vy_diff.png
    docs/tasks/20_l1_l2_grid_sweep/figures/sweep_lateral_load_transfer.png
    + summary CSV
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
BIN  = REPO / "build/bin/vdsim_l1_vs_l2"
OUT  = REPO / "docs/tasks/20_l1_l2_grid_sweep/figures"
OUT.mkdir(parents=True, exist_ok=True)

VEHICLE = REPO / "configs/vehicles/sedan.yaml"
TIRE    = REPO / "configs/tires/default_pacejka.yaml"

vx_grid    = np.array([5.0, 10.0, 15.0, 20.0])
delta_grid = np.array([0.02, 0.04, 0.06, 0.08, 0.10])
DURATION = 5.0     # let SS settle
DT       = 0.005


def make_scenario(vx, delta, path):
    path.write_text(f"""name: sweep_vx{vx:.1f}_d{delta:.3f}
initial_vx:    {vx}
duration:      {DURATION}
dt:            {DT}
mu:            1.0
interpolation: zoh
controls:
  - {{ t: 0.0, throttle: 0.0, brake: 0.0, steer: {delta:.4f}, gear: 1 }}
  - {{ t: {DURATION}, throttle: 0.0, brake: 0.0, steer: {delta:.4f}, gear: 1 }}
""")


with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    rows = []
    for vx in vx_grid:
        for d in delta_grid:
            sc_path = td / "sc.yaml"
            make_scenario(vx, d, sc_path)
            res = subprocess.run(
                [str(BIN), str(VEHICLE), str(TIRE), str(sc_path), str(td)],
                capture_output=True, text=True, check=True)
            name = f"sweep_vx{vx:.1f}_d{d:.3f}"
            l1 = pd.read_csv(td / f"{name}_L1.csv").iloc[-1]
            l2 = pd.read_csv(td / f"{name}_L2.csv").iloc[-1]
            rows.append({
                "vx": vx, "delta": d,
                "r_L1": l1["r"], "r_L2": l2["r"],
                "vy_L1": l1["vy"], "vy_L2": l2["vy"],
                "Fz_FL": l2["Fz_FL"], "Fz_FR": l2["Fz_FR"],
                "Fz_RL": l2["Fz_RL"], "Fz_RR": l2["Fz_RR"],
            })

df = pd.DataFrame(rows)
df["r_diff_pct"] = (df["r_L2"] - df["r_L1"]) / df["r_L1"] * 100.0
df["vy_diff"]    =  df["vy_L2"] - df["vy_L1"]
df["lat_transfer_f"] = df["Fz_FR"] - df["Fz_FL"]
df["lat_transfer_r"] = df["Fz_RR"] - df["Fz_RL"]
df.to_csv(OUT / "l1_vs_l2_sweep.csv", index=False)


# Reshape into grids
def grid(col):
    arr = np.zeros((len(vx_grid), len(delta_grid)))
    for i, vx in enumerate(vx_grid):
        for j, d in enumerate(delta_grid):
            row = df[(df["vx"] == vx) & (df["delta"] == d)].iloc[0]
            arr[i, j] = row[col]
    return arr


# ---- Fig 1: r_diff_pct heatmap ----
arr = grid("r_diff_pct")
fig, ax = plt.subplots(figsize=(7, 4.5))
vmax = max(0.5, abs(arr).max())
im = ax.imshow(arr, cmap="RdBu_r", vmin=-vmax, vmax=vmax, origin="lower", aspect="auto")
ax.set_xticks(range(len(delta_grid)))
ax.set_xticklabels([f"{d:.03f}" for d in delta_grid])
ax.set_yticks(range(len(vx_grid)))
ax.set_yticklabels([f"{v:.1f}" for v in vx_grid])
ax.set_xlabel("delta [rad]"); ax.set_ylabel("vx [m/s]")
ax.set_title("(r_L2 - r_L1) / r_L1  [%]")
for i in range(len(vx_grid)):
    for j in range(len(delta_grid)):
        ax.text(j, i, f"{arr[i,j]:+.2f}", ha="center", va="center", fontsize=8)
plt.colorbar(im, ax=ax, label="%")
plt.tight_layout()
plt.savefig(OUT / "sweep_yaw_rate_diff.png", dpi=120)
plt.close(fig)


# ---- Fig 2: vy_diff (absolute, m/s) ----
arr = grid("vy_diff")
fig, ax = plt.subplots(figsize=(7, 4.5))
vmax = max(0.01, abs(arr).max())
im = ax.imshow(arr, cmap="RdBu_r", vmin=-vmax, vmax=vmax, origin="lower", aspect="auto")
ax.set_xticks(range(len(delta_grid)))
ax.set_xticklabels([f"{d:.03f}" for d in delta_grid])
ax.set_yticks(range(len(vx_grid)))
ax.set_yticklabels([f"{v:.1f}" for v in vx_grid])
ax.set_xlabel("delta [rad]"); ax.set_ylabel("vx [m/s]")
ax.set_title("vy_L2 - vy_L1  [m/s]")
for i in range(len(vx_grid)):
    for j in range(len(delta_grid)):
        ax.text(j, i, f"{arr[i,j]:+.3f}", ha="center", va="center", fontsize=8)
plt.colorbar(im, ax=ax)
plt.tight_layout()
plt.savefig(OUT / "sweep_vy_diff.png", dpi=120)
plt.close(fig)


# ---- Fig 3: lateral load transfer (front+rear axles) ----
arr_f = grid("lat_transfer_f")
arr_r = grid("lat_transfer_r")
fig, (axL, axR) = plt.subplots(1, 2, figsize=(12, 4.5))
for ax, arr, title in [(axL, arr_f, "Front  Fz_FR - Fz_FL  [N]"),
                       (axR, arr_r, "Rear   Fz_RR - Fz_RL  [N]")]:
    vmax = max(50.0, abs(arr).max())
    im = ax.imshow(arr, cmap="RdBu_r", vmin=-vmax, vmax=vmax, origin="lower", aspect="auto")
    ax.set_xticks(range(len(delta_grid)))
    ax.set_xticklabels([f"{d:.03f}" for d in delta_grid])
    ax.set_yticks(range(len(vx_grid)))
    ax.set_yticklabels([f"{v:.1f}" for v in vx_grid])
    ax.set_xlabel("delta [rad]"); ax.set_ylabel("vx [m/s]")
    ax.set_title(title)
    for i in range(len(vx_grid)):
        for j in range(len(delta_grid)):
            ax.text(j, i, f"{arr[i,j]:+.0f}", ha="center", va="center", fontsize=8)
    plt.colorbar(im, ax=ax)
plt.tight_layout()
plt.savefig(OUT / "sweep_lateral_load_transfer.png", dpi=120)
plt.close(fig)


# Summary
print(f"max |r diff| = {df['r_diff_pct'].abs().max():.2f} %")
print(f"max |vy diff| = {df['vy_diff'].abs().max():.3f} m/s")
print(f"max lat transfer (front) = {df['lat_transfer_f'].abs().max():.0f} N")
print(f"max lat transfer (rear)  = {df['lat_transfer_r'].abs().max():.0f} N")
print(f"figures -> {OUT}")
