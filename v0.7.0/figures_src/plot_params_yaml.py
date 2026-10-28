"""
Plot Task 12 (params YAML I/O) verification figures.

Inputs:
    /tmp/params_roundtrip.csv               (from dump_params_demo)
    docs/tasks/12_W4_params_yaml/sample_default_vehicle.yaml
    docs/tasks/12_W4_params_yaml/sample_sports_vehicle.yaml

Outputs:
    docs/tasks/12_W4_params_yaml/figures/roundtrip_residual.png
    docs/tasks/12_W4_params_yaml/figures/sedan_vs_sports.png
"""

from pathlib import Path
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


TASK_DIR = Path(__file__).resolve().parent.parent / "tasks/12_W4_params_yaml"
FIG_DIR  = TASK_DIR / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)
CSV_PATH = Path("/tmp/params_roundtrip.csv")


# -----------------------------------------------------------------------------
# Fig 1: per-field roundtrip absolute residual
# -----------------------------------------------------------------------------
groups, fields, diffs = [], [], []
with open(CSV_PATH) as f:
    rdr = csv.DictReader(f)
    for row in rdr:
        groups.append(row["group"])
        fields.append(row["field"])
        diffs.append(float(row["abs_diff"]))
diffs = np.asarray(diffs)
N = len(diffs)

# Color by group
unique_groups = []
for g in groups:
    if g not in unique_groups:
        unique_groups.append(g)
cmap = plt.cm.tab10
group_colors = {g: cmap(i % 10) for i, g in enumerate(unique_groups)}
bar_colors = [group_colors[g] for g in groups]

fig, ax = plt.subplots(figsize=(11, 4.2))
display_diffs = np.where(diffs > 0, diffs, 1e-18)
ax.bar(range(N), display_diffs, color=bar_colors)
ax.set_yscale("log")
ax.set_ylim(1e-18, 1e-6)
ax.axhline(2.22e-16, color="gray", linestyle="--", linewidth=0.8,
           label="machine eps (double)")
ax.set_xticks(range(N))
ax.set_xticklabels(fields, rotation=70, fontsize=7, ha="right")
ax.set_ylabel("|saved - loaded|  (log scale)")
ax.set_title(f"VehicleParams + TireParams YAML roundtrip residual  "
             f"(N = {N} fields, max = {diffs.max():.0e})")
ax.grid(True, axis="y", alpha=0.3)

# group legend
handles = [plt.Rectangle((0,0),1,1, color=group_colors[g]) for g in unique_groups]
ax.legend(handles + [plt.Line2D([0],[0], color="gray", linestyle="--")],
          unique_groups + ["machine eps"], fontsize=8, ncol=4, loc="upper right")
plt.tight_layout()
plt.savefig(FIG_DIR / "roundtrip_residual.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 2: sedan vs sports — relative diff on tuning fields
# -----------------------------------------------------------------------------
with open(TASK_DIR / "sample_default_vehicle.yaml") as f:
    sedan = yaml.safe_load(f)
with open(TASK_DIR / "sample_sports_vehicle.yaml") as f:
    sports = yaml.safe_load(f)

compare_fields = [
    "mass", "mass_sprung",
    "wheelbase", "cg_to_front", "cg_to_rear", "cg_height",
    "roll_stiffness_front", "roll_stiffness_rear",
    "max_motor_torque", "max_brake_torque",
    "steering_ratio", "max_steer_angle_wheel",
    "aero_drag_coeff", "frontal_area",
]
sedan_vals  = []
sports_vals = []
for k in compare_fields:
    sedan_vals.append(float(sedan[k]))
    sports_vals.append(float(sports[k]))
sedan_vals  = np.array(sedan_vals)
sports_vals = np.array(sports_vals)
rel_pct = (sports_vals - sedan_vals) / sedan_vals * 100.0

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5),
                                gridspec_kw={"width_ratios": [1.5, 1.0]})

# left: absolute values side by side
xpos = np.arange(len(compare_fields))
w = 0.4
ax1.bar(xpos - w/2, sedan_vals,  width=w, label="sedan (default)", color="#4F81BD")
ax1.bar(xpos + w/2, sports_vals, width=w, label="sports",          color="#DC291E")
ax1.set_yscale("log")
ax1.set_xticks(xpos)
ax1.set_xticklabels(compare_fields, rotation=60, fontsize=7, ha="right")
ax1.set_ylabel("value (log scale, mixed units)")
ax1.set_title("Field values: default sedan vs sports variant")
ax1.grid(True, axis="y", alpha=0.3)
ax1.legend(fontsize=8)

# right: relative diff
colors2 = ["#DC291E" if v > 0 else "#345A8A" for v in rel_pct]
ax2.barh(xpos, rel_pct, color=colors2)
ax2.set_yticks(xpos)
ax2.set_yticklabels(compare_fields, fontsize=8)
ax2.invert_yaxis()
ax2.axvline(0, color="black", linewidth=0.6)
ax2.set_xlabel("sports - sedan  [%]")
ax2.set_title("Relative change")
ax2.grid(True, axis="x", alpha=0.3)

plt.tight_layout()
plt.savefig(FIG_DIR / "sedan_vs_sports.png", dpi=120)
plt.close(fig)

print(f"max_abs_residual = {diffs.max():.3e}  (N={N} fields)")
print(f"figures written to {FIG_DIR}")
