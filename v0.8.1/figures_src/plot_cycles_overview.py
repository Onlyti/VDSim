"""Task 62: PoC cycle progression overview figure."""
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/62_cycles_overview/figures")
OUT.mkdir(parents=True, exist_ok=True)

# Cycle | tests | core_cpp | binaries | reports
cycles = [
    ("W1-W4 base",        53,  8, 1, 11),
    ("Cycle1 (12-14)",    71,  8, 1, 14),
    ("Cycle2 (15-19)",    87, 12, 4, 19),
    ("Cycle3 (20-24)",    98, 13, 6, 24),
    ("Cycle4 (25-29)",   109, 13, 8, 29),
    ("Cycle5 (30+ L3+L8)",124,13, 9, 33),
    ("Cycle6 (anti-dive+aero)",127,13,9, 35),
    ("Cycle7 (extras+L3.unsprung)",135,13,9, 38),
    ("Cycle8 (CARLA+pybind)", 138, 13, 12, 40),
    ("Cycle9 (driver+rr)",  140, 13, 13, 43),
]

names = [c[0] for c in cycles]
tests = [c[1] for c in cycles]
binaries = [c[3] for c in cycles]
reports = [c[4] for c in cycles]

xpos = np.arange(len(cycles))

fig, ax1 = plt.subplots(figsize=(13, 5))

color_t = "#4F81BD"
ax1.plot(xpos, tests, "o-", color=color_t, label="tests passing")
ax1.set_xticks(xpos); ax1.set_xticklabels(names, rotation=35, ha="right", fontsize=8)
ax1.set_ylabel("tests passing", color=color_t)
ax1.tick_params(axis="y", labelcolor=color_t)
ax1.grid(True, alpha=0.3)

ax2 = ax1.twinx()
ax2.plot(xpos, reports, "s--", color="#DC291E", label="task reports")
ax2.plot(xpos, binaries, "^--", color="#345A8A", label="example binaries")
ax2.set_ylabel("count")
ax2.legend(loc="lower right", fontsize=9)

ax1.set_title("VDSim PoC — progression by cycle (Task 11 → Task 60+)")
plt.tight_layout()
plt.savefig(OUT / "cycles_progression.png", dpi=120); plt.close(fig)

# Cumulative Phase-2 absorption
phase2_absorbed = [
    ("combined slip + Mz", "Phase 2", "absorbed (Cycle2)"),
    ("Ackerman", "Phase 2", "absorbed (Cycle3)"),
    ("Differential 3 modes", "Phase 2", "absorbed (Cycle4)"),
    ("Aero downforce", "Phase 2", "absorbed (Cycle4)"),
    ("Brake bias + EBD", "Phase 2", "absorbed (Cycle4-5)"),
    ("Pneumatic trail Mz", "Phase 2", "absorbed (Cycle2)"),
    ("Camber thrust API", "Phase 2", "absorbed (Cycle6)"),
    ("Anti-dive", "Phase 2", "absorbed (Cycle6)"),
    ("L3 14-DOF", "W11-W12", "completed (Cycle5+8)"),
    ("L5-L8 controllers", "Phase 2", "absorbed (Cycle1)"),
    ("Pybind11", "Phase 2", "absorbed (Cycle8)"),
    ("CARLA plugin skeleton", "W7-W8", "absorbed (Cycle8)"),
    ("Driver model", "Phase 2", "absorbed (Cycle9)"),
    ("MPC / SMPC", "Phase 2", "deferred"),
    ("CarMaker ERG", "W11-W12", "deferred (license)"),
    ("Full unsprung damper split", "W11-W12", "limit ack'd"),
]
fig, ax = plt.subplots(figsize=(10, 6))
for i, (feature, plan, status) in enumerate(phase2_absorbed):
    color = "#4F81BD" if "absorbed" in status or "completed" in status else "#DC291E"
    ax.scatter(0, len(phase2_absorbed)-i, color=color, s=80)
    ax.text(0.05, len(phase2_absorbed)-i, f"{feature}", va="center", fontsize=10)
    ax.text(2.3, len(phase2_absorbed)-i, status, va="center", fontsize=9,
            color=color, fontweight="bold")
ax.set_xlim(-0.2, 3.5); ax.axis("off")
ax.set_title("Scope absorption status — Phase 2 features pulled into PoC")
plt.tight_layout()
plt.savefig(OUT / "scope_absorption.png", dpi=120); plt.close(fig)

print(f"figures -> {OUT}")
