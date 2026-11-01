"""Task 48: Pacejka MF96 textbook curve validation."""
import sys, os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Use the Python pybind module
BUILD_PY = Path("/home/ailab-12/git/VDSim/build/python")
sys.path.insert(0, str(BUILD_PY))

OUT = Path("/home/ailab-12/git/VDSim/docs/tasks/48_mf96_validation/figures")
OUT.mkdir(parents=True, exist_ok=True)

# Direct Python re-implementation matching core/src/pacejka_mf96.cpp
B_long, C_long, D_long, E_long = 10.0, 1.65, 1.0,  0.97
B_lat,  C_lat,  D_lat,  E_lat  = 8.0,  1.30, 1.0, -1.0
MU_NOM = 1.0

def pacejka_form(B, C, D, E, s):
    t = B * s
    phi = t - E * (t - np.arctan(t))
    return D * np.sin(C * np.arctan(phi))

# Fig 1: Fy vs alpha at multiple Fz
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
alphas = np.linspace(-0.30, 0.30, 121)
for Fz, color in zip([2000, 4000, 6000, 8000],
                     ["#345A8A", "#4F81BD", "#01A0E9", "#DC291E"]):
    Dy = D_lat * Fz * MU_NOM
    Fy = -pacejka_form(B_lat, C_lat, Dy, E_lat, alphas)
    ax1.plot(np.degrees(alphas), Fy, color=color, label=f"Fz = {Fz} N")
ax1.set_xlabel("slip angle [deg]"); ax1.set_ylabel("Fy [N]")
ax1.set_title("Pacejka MF96 lateral force curves")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=9)

# Fig 2: Fx vs kappa
kappas = np.linspace(-0.30, 0.30, 121)
for Fz, color in zip([2000, 4000, 6000, 8000],
                     ["#345A8A", "#4F81BD", "#01A0E9", "#DC291E"]):
    Dx = D_long * Fz * MU_NOM
    Fx = pacejka_form(B_long, C_long, Dx, E_long, kappas)
    ax2.plot(kappas, Fx, color=color, label=f"Fz = {Fz} N")
ax2.set_xlabel("slip ratio kappa [-]"); ax2.set_ylabel("Fx [N]")
ax2.set_title("Pacejka MF96 longitudinal force curves")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=9)

plt.tight_layout()
plt.savefig(OUT / "mf96_curves.png", dpi=120); plt.close(fig)


# Fig 3: friction circle / ellipse at Fz=4000
import math
Fz = 4000.0
fig, ax = plt.subplots(figsize=(6.5, 6.5))
# Sweep alpha at fixed kappa (and vice versa)
for k_fix in [0.0, 0.05, 0.10, 0.15]:
    Fxs, Fys = [], []
    for a in np.linspace(-0.20, 0.20, 80):
        Dy = D_lat * Fz * MU_NOM
        Dx = D_long * Fz * MU_NOM
        Fy = -pacejka_form(B_lat, C_lat, Dy, E_lat, a)
        Fx =  pacejka_form(B_long, C_long, Dx, E_long, k_fix)
        # friction ellipse rescale
        ratio = math.sqrt((Fx/Dx)**2 + (Fy/Dy)**2)
        if ratio > 1.0:
            Fx /= ratio; Fy /= ratio
        Fxs.append(Fx); Fys.append(Fy)
    ax.plot(Fxs, Fys, label=f"kappa={k_fix:.02f}")
# Reference ellipse
th = np.linspace(0, 2*np.pi, 200)
ax.plot(D_long*Fz*np.cos(th), D_lat*Fz*np.sin(th),
        "k--", linewidth=1, label="ellipse boundary")
ax.set_xlabel("Fx [N]"); ax.set_ylabel("Fy [N]")
ax.set_title(f"Friction ellipse traversal at Fz = {Fz} N")
ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
ax.set_aspect("equal", "datalim")
plt.tight_layout()
plt.savefig(OUT / "mf96_friction_ellipse.png", dpi=120); plt.close(fig)

print(f"D_lat peak at Fz=4000: {D_lat * 4000:.0f} N")
print(f"D_long peak at Fz=4000: {D_long * 4000:.0f} N")
print(f"figures -> {OUT}")
