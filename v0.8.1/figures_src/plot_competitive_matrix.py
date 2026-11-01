"""Competitive matrix vs commercial vehicle dynamics simulators."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/69_competitive_matrix/figures")
OUT.mkdir(parents=True, exist_ok=True)

# Axes (0-5 scoring)
axes = [
    "Multibody depth\n(suspension hardpoint)",
    "Simplified VD\n(autonomy domain)",
    "Control ladder\n(L1-L8 abstraction)",
    "Open-core API",
    "External integration\n(CARLA, Python, FMI)",
    "Tire model fidelity\n(MF, FTire, RMod-K)",
    "Free / Open license\n(inverse of price)",
]

solutions = {
    "Adams Car (MSC)":       [5, 1, 1, 0, 2, 5, 0],
    "VI-CarRealTime":        [5, 2, 3, 0, 3, 4, 0],
    "CarMaker (IPG)":        [2, 5, 3, 0, 3, 5, 1],
    "CarSim (MS)":           [2, 5, 3, 0, 3, 4, 1],
    "Simulink VDB":          [1, 4, 3, 1, 5, 3, 2],
    "VDSim (current PoC)":   [1, 5, 5, 5, 4, 3, 5],
    "VDSim (+ L4-L5 plan)":  [4, 5, 5, 5, 5, 4, 5],
}

# Radar chart
N = len(axes)
angles = np.linspace(0, 2*np.pi, N, endpoint=False).tolist()
angles += angles[:1]

fig, ax = plt.subplots(figsize=(11, 9), subplot_kw=dict(polar=True))
colors = {
    "Adams Car (MSC)":       "#7F0000",
    "VI-CarRealTime":        "#DC291E",
    "CarMaker (IPG)":        "#345A8A",
    "CarSim (MS)":           "#4F81BD",
    "Simulink VDB":          "#01A0E9",
    "VDSim (current PoC)":   "#FF8C00",
    "VDSim (+ L4-L5 plan)":  "#FFD700",
}
linewidths = {
    "VDSim (current PoC)":   2.5,
    "VDSim (+ L4-L5 plan)":  2.5,
}

for name, vals in solutions.items():
    vals = vals + vals[:1]
    lw = linewidths.get(name, 1.4)
    ax.plot(angles, vals, "o-", linewidth=lw, color=colors[name], label=name)
    if "VDSim" in name:
        ax.fill(angles, vals, alpha=0.10, color=colors[name])

ax.set_xticks(angles[:-1])
ax.set_xticklabels(axes, fontsize=9)
ax.set_ylim(0, 5)
ax.set_yticks([1, 2, 3, 4, 5])
ax.set_yticklabels(["1", "2", "3", "4", "5"], fontsize=8)
ax.set_title("Competitive matrix — vehicle dynamics simulators (0=none, 5=best)",
             fontsize=12, pad=20)
ax.legend(loc="upper right", bbox_to_anchor=(1.40, 1.05), fontsize=9)
plt.tight_layout()
plt.savefig(OUT / "competitive_radar.png", dpi=120, bbox_inches="tight")
plt.close(fig)


# Tabular bar comparison
fig, ax = plt.subplots(figsize=(12, 5))
order = list(solutions.keys())
totals = [sum(solutions[s]) for s in order]
xs = np.arange(len(order))
bars = ax.bar(xs, totals, color=[colors[s] for s in order])
for b, t in zip(bars, totals):
    ax.text(b.get_x() + b.get_width()/2, t + 0.4, str(t),
            ha="center", fontsize=10, fontweight="bold")
ax.set_xticks(xs); ax.set_xticklabels(order, rotation=20, ha="right", fontsize=9)
ax.set_ylabel("aggregate score (max 35)")
ax.set_title("Aggregate axis-score (subjective; intended for marketing/PT use)")
ax.set_ylim(0, 35)
ax.grid(True, axis="y", alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "aggregate_score.png", dpi=120)
plt.close(fig)


# Per-axis table for the report
import csv
with open(OUT / "competitive_table.csv", "w") as f:
    w = csv.writer(f)
    w.writerow(["solution"] + axes)
    for name, vals in solutions.items():
        w.writerow([name] + vals)

print("figures + CSV ->", OUT)
