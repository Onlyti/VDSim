"""
Plot Task 15 figures: friction ellipse and Mz aligning moment.

Uses a Python re-implementation matching core/src/pacejka_mf96.cpp.

Outputs:
    docs/tasks/15_combined_slip_mz/figures/friction_ellipse.png
    docs/tasks/15_combined_slip_mz/figures/mz_vs_alpha.png
    docs/tasks/15_combined_slip_mz/figures/combined_vs_pure.png
"""

from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "tasks/15_combined_slip_mz/figures"
OUT.mkdir(parents=True, exist_ok=True)


# Defaults from TireParams
B_long, C_long, D_long, E_long = 10.0, 1.65, 1.0,  0.97
B_lat,  C_lat,  D_lat,  E_lat  = 8.0,  1.30, 1.0, -1.0
MU_NOM = 1.0
PNEU_TRAIL = 0.05
TRAIL_FALLOFF = 0.20


def pacejka_form(B, C, D, E, s):
    t = B * s
    phi = t - E * (t - np.arctan(t))
    return D * np.sin(C * np.arctan(phi))


def tire_forces(kappa, alpha, Fz=4000.0, mu=1.0, combined=True):
    Fx_max = D_long * Fz * mu * MU_NOM
    Fy_max = D_lat  * Fz * mu * MU_NOM
    Fx_pure =  pacejka_form(B_long, C_long, Fx_max, E_long, kappa)
    Fy_pure = -pacejka_form(B_lat,  C_lat,  Fy_max, E_lat,  alpha)
    if combined and Fx_max > 0 and Fy_max > 0:
        rsq = (Fx_pure / Fx_max) ** 2 + (Fy_pure / Fy_max) ** 2
        if rsq > 1.0:
            scale = 1.0 / np.sqrt(rsq)
            Fx_pure *= scale
            Fy_pure *= scale
    a_fo = max(TRAIL_FALLOFF, 1e-6)
    trail = PNEU_TRAIL / np.sqrt(1.0 + (alpha / a_fo) ** 2)
    Mz = -trail * Fy_pure
    return Fx_pure, Fy_pure, Mz


# -----------------------------------------------------------------------------
# Fig 1: friction ellipse — (Fx, Fy) over (kappa, alpha) grid + bounding ellipse
# -----------------------------------------------------------------------------
Fz = 4000.0
Fx_max = D_long * Fz
Fy_max = D_lat  * Fz
kappas = np.linspace(-0.30, 0.30, 25)
alphas = np.linspace(-0.30, 0.30, 25)

Fxs_c, Fys_c, Fxs_p, Fys_p = [], [], [], []
for k in kappas:
    for a in alphas:
        fx, fy, _ = tire_forces(k, a, Fz, combined=True)
        Fxs_c.append(fx); Fys_c.append(fy)
        fx, fy, _ = tire_forces(k, a, Fz, combined=False)
        Fxs_p.append(fx); Fys_p.append(fy)

fig, (axL, axR) = plt.subplots(1, 2, figsize=(11, 5.0))
# Left: without combined slip (decoupled)
axL.scatter(Fxs_p, Fys_p, s=6, alpha=0.45, color="#DC291E", label="(k, a) sweep")
ell_t = np.linspace(0, 2*np.pi, 200)
axL.plot(Fx_max * np.cos(ell_t), Fy_max * np.sin(ell_t),
         "k--", linewidth=1.2, label="friction ellipse")
axL.set_xlabel("Fx [N]"); axL.set_ylabel("Fy [N]")
axL.set_title("Decoupled (combined_slip_enabled = false)")
axL.set_aspect("equal", "datalim")
axL.grid(True, alpha=0.3); axL.legend(fontsize=8)

# Right: with combined slip
axR.scatter(Fxs_c, Fys_c, s=6, alpha=0.45, color="#4F81BD", label="(k, a) sweep")
axR.plot(Fx_max * np.cos(ell_t), Fy_max * np.sin(ell_t),
         "k--", linewidth=1.2, label="friction ellipse")
axR.set_xlabel("Fx [N]"); axR.set_ylabel("Fy [N]")
axR.set_title("Combined slip on (this PR default)")
axR.set_aspect("equal", "datalim")
axR.grid(True, alpha=0.3); axR.legend(fontsize=8)
fig.suptitle(f"Friction ellipse coverage  (Fz={Fz} N, kappa/alpha in +-0.30)",
             fontsize=11)
plt.tight_layout()
plt.savefig(OUT / "friction_ellipse.png", dpi=120)
plt.close(fig)

# Max excursion
max_r_pure = max(np.sqrt((fx/Fx_max)**2 + (fy/Fy_max)**2)
                 for fx, fy in zip(Fxs_p, Fys_p))
max_r_comb = max(np.sqrt((fx/Fx_max)**2 + (fy/Fy_max)**2)
                 for fx, fy in zip(Fxs_c, Fys_c))


# -----------------------------------------------------------------------------
# Fig 2: Mz vs alpha (Fz = 2000, 4000, 6000)
# -----------------------------------------------------------------------------
alpha_grid = np.linspace(-0.30, 0.30, 121)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
for Fz_test, color in zip([2000.0, 4000.0, 6000.0],
                          ["#345A8A", "#4F81BD", "#01A0E9"]):
    Mz, Fy, trail = [], [], []
    for a in alpha_grid:
        fx, fy, mz = tire_forces(0.0, a, Fz_test)
        Mz.append(mz); Fy.append(fy)
        trail.append(PNEU_TRAIL / np.sqrt(1.0 + (a/TRAIL_FALLOFF)**2))
    ax1.plot(alpha_grid, Mz, color=color, label=f"Fz = {Fz_test:.0f} N")
    ax2.plot(alpha_grid, np.array(trail) * 1000.0, color=color,
             label=f"Fz = {Fz_test:.0f} N")
ax1.set_xlabel("alpha [rad]"); ax1.set_ylabel("Mz [N m]")
ax1.set_title("Aligning moment vs slip angle")
ax1.axvline(0, color="gray", linewidth=0.6); ax1.axhline(0, color="gray", linewidth=0.6)
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)
ax2.set_xlabel("alpha [rad]"); ax2.set_ylabel("pneumatic trail [mm]")
ax2.set_title("Pneumatic trail falloff  t_p_0 = 50 mm, alpha_fo = 0.20")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "mz_vs_alpha.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 3: Pure vs combined Fy at fixed alpha = 0.10, sweep kappa
# -----------------------------------------------------------------------------
kappa_grid = np.linspace(-0.30, 0.30, 121)
alpha_fixed = 0.10
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
Fx_p, Fy_p, Fx_c, Fy_c = [], [], [], []
for k in kappa_grid:
    fx, fy, _ = tire_forces(k, alpha_fixed, 4000.0, combined=False)
    Fx_p.append(fx); Fy_p.append(fy)
    fx, fy, _ = tire_forces(k, alpha_fixed, 4000.0, combined=True)
    Fx_c.append(fx); Fy_c.append(fy)
ax1.plot(kappa_grid, Fx_p, "r--", label="Fx pure (decoupled)")
ax1.plot(kappa_grid, Fx_c, "r-",  label="Fx combined")
ax1.plot(kappa_grid, Fy_p, "b--", label="Fy pure")
ax1.plot(kappa_grid, Fy_c, "b-",  label="Fy combined")
ax1.set_xlabel("kappa"); ax1.set_ylabel("force [N]")
ax1.set_title(f"Fixed alpha = {alpha_fixed} rad")
ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8, ncol=2)
ax2.plot(kappa_grid, np.array(Fx_c)/4000.0, "r-", label="Fx/Fz combined")
ax2.plot(kappa_grid, np.array(Fy_c)/4000.0, "b-", label="Fy/Fz combined")
F_mag = np.sqrt(np.array(Fx_c)**2 + np.array(Fy_c)**2) / 4000.0
ax2.plot(kappa_grid, F_mag, "k-", linewidth=1.5, label="|F|/Fz combined")
ax2.axhline(1.0, color="gray", linestyle=":", label="mu = 1.0")
ax2.set_xlabel("kappa"); ax2.set_ylabel("F / Fz")
ax2.set_title("Normalized magnitude")
ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "combined_vs_pure.png", dpi=120)
plt.close(fig)

print(f"max ratio pure     : {max_r_pure:.3f}")
print(f"max ratio combined : {max_r_comb:.3f}")
print(f"figures -> {OUT}")
