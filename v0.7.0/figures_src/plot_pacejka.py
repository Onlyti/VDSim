"""
Plot Pacejka MF96 lateral / longitudinal curves used in Task 10.

Replicates the C++ implementation in core/src/pacejka_mf96.cpp so the
figure is reproducible from documentation alone.

Output:
    docs/tasks/10_W3_tire_models/figures/pacejka_lateral.png
    docs/tasks/10_W3_tire_models/figures/pacejka_longitudinal.png
    docs/tasks/10_W3_tire_models/figures/pacejka_mu_scaling.png
"""

from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Default TireParams (matches core/include/vdsim/params.hpp)
B_LONG, C_LONG, D_LONG, E_LONG = 10.0, 1.65, 1.0,  0.97
B_LAT,  C_LAT,  D_LAT,  E_LAT  = 8.0,  1.30, 1.0, -1.0
MU_NOMINAL = 1.0


def pacejka_lateral(alpha, Fz, mu_lat=1.0):
    Dy = D_LAT * Fz * MU_NOMINAL * mu_lat
    s  = alpha
    t  = B_LAT * s
    phi = t - E_LAT * (t - np.arctan(t))
    return -Dy * np.sin(C_LAT * np.arctan(phi))  # leading minus = ISO 8855


def pacejka_longitudinal(kappa, Fz, mu_long=1.0):
    Dx = D_LONG * Fz * MU_NOMINAL * mu_long
    s  = kappa
    t  = B_LONG * s
    phi = t - E_LONG * (t - np.arctan(t))
    return Dx * np.sin(C_LONG * np.arctan(phi))


OUT_DIR = Path(__file__).resolve().parent.parent / "tasks/10_W3_tire_models/figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# -----------------------------------------------------------------------------
# Fig 1: Lateral force vs slip angle, multiple Fz
# -----------------------------------------------------------------------------
alpha = np.linspace(-np.deg2rad(15), np.deg2rad(15), 401)
fig, ax = plt.subplots(figsize=(6.5, 4.0))
for Fz in [2000, 4000, 6000, 8000]:
    ax.plot(np.rad2deg(alpha), pacejka_lateral(alpha, Fz) / 1000.0,
            label=f"Fz = {Fz} N")
ax.set_xlabel("slip angle alpha [deg]")
ax.set_ylabel("Fy [kN]")
ax.set_title("Pacejka MF96 lateral force (mu = 1.0)")
ax.axhline(0, color="k", lw=0.5, alpha=0.5)
ax.axvline(0, color="k", lw=0.5, alpha=0.5)
ax.grid(True, alpha=0.3)
ax.legend()
plt.tight_layout()
plt.savefig(OUT_DIR / "pacejka_lateral.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 2: Longitudinal force vs slip ratio
# -----------------------------------------------------------------------------
kappa = np.linspace(-0.30, 0.30, 401)
fig, ax = plt.subplots(figsize=(6.5, 4.0))
for Fz in [2000, 4000, 6000, 8000]:
    ax.plot(kappa, pacejka_longitudinal(kappa, Fz) / 1000.0,
            label=f"Fz = {Fz} N")
ax.set_xlabel("slip ratio kappa [-]")
ax.set_ylabel("Fx [kN]")
ax.set_title("Pacejka MF96 longitudinal force (mu = 1.0)")
ax.axhline(0, color="k", lw=0.5, alpha=0.5)
ax.axvline(0, color="k", lw=0.5, alpha=0.5)
ax.grid(True, alpha=0.3)
ax.legend()
plt.tight_layout()
plt.savefig(OUT_DIR / "pacejka_longitudinal.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 3: Effect of mu scaling (lateral, fixed Fz = 4 kN)
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.5, 4.0))
for mu in [1.0, 0.7, 0.5, 0.3]:
    ax.plot(np.rad2deg(alpha), pacejka_lateral(alpha, 4000, mu_lat=mu) / 1000.0,
            label=f"mu = {mu}")
ax.set_xlabel("slip angle alpha [deg]")
ax.set_ylabel("Fy [kN]")
ax.set_title("Pacejka MF96 lateral force vs surface friction (Fz = 4 kN)")
ax.axhline(0, color="k", lw=0.5, alpha=0.5)
ax.axvline(0, color="k", lw=0.5, alpha=0.5)
ax.grid(True, alpha=0.3)
ax.legend()
plt.tight_layout()
plt.savefig(OUT_DIR / "pacejka_mu_scaling.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Linear-region slope verification (numerical check)
# -----------------------------------------------------------------------------
print("Linear-region slope verification:")
for Fz in [2000, 4000, 6000, 8000]:
    a_small = 1e-4
    Fy = pacejka_lateral(a_small, Fz)
    slope = Fy / a_small
    expected = -B_LAT * C_LAT * D_LAT * Fz * MU_NOMINAL
    err = abs(slope - expected) / abs(expected)
    print(f"  Fz={Fz:5d} N  slope={slope:12.1f} N/rad  expected={expected:12.1f} N/rad  err={err*100:.4f}%")

print(f"\nFigures written to {OUT_DIR}")
