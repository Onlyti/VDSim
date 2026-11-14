"""
Plot Task 16 figure: RK4 vs Euler comparison driven by solver YAML.

Reads CSV from `vdsim_bicycle_run` runs with two solver configs.

Outputs:
    docs/tasks/16_tire_solver_yaml/figures/rk4_vs_euler.png
    docs/tasks/16_tire_solver_yaml/figures/yaml_schema_overview.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent.parent / "tasks/16_tire_solver_yaml/figures"
OUT.mkdir(parents=True, exist_ok=True)

RUN = Path("/tmp/vdsim_runs")

# Analytical linear-bicycle SS yaw rate for default sedan + default tire,
# delta = 0.05 rad, vx0 = 10 m/s, no drag (this scenario keeps drag on,
# so vx drifts slightly).
DELTA = 0.05
VX0   = 10.0
# Hard-code reference: matches Task 11 analytical (Cf == Cr neutral steer
# coincidence for default mid-sedan + default Pacejka).
R_ANA_INIT = 0.18519   # vx*delta/L at t=0

df_rk4 = pd.read_csv(RUN / "rk4_1ms_step_steer.csv")
df_eul = pd.read_csv(RUN / "euler_10ms_step_steer.csv")

fig, (ax_r, ax_vx) = plt.subplots(1, 2, figsize=(11, 4.2))
ax_r.plot(df_rk4["t"], df_rk4["r"], color="#4F81BD", label="RK4 (1 ms substep)")
ax_r.plot(df_eul["t"], df_eul["r"], color="#DC291E", label="Euler (10 ms substep)",
          linestyle="--")
ax_r.axhline(R_ANA_INIT, color="black", linestyle=":",
             label=f"analytical (t=0) = {R_ANA_INIT:.4f}")
ax_r.set_xlabel("time [s]"); ax_r.set_ylabel("yaw rate r [rad/s]")
ax_r.set_title("Step steer  delta = 0.05, vx0 = 10 m/s")
ax_r.grid(True, alpha=0.3); ax_r.legend(fontsize=8)

ax_vx.plot(df_rk4["t"], df_rk4["vx"], color="#4F81BD", label="RK4")
ax_vx.plot(df_eul["t"], df_eul["vx"], color="#DC291E", label="Euler", linestyle="--")
ax_vx.set_xlabel("time [s]"); ax_vx.set_ylabel("vx [m/s]")
ax_vx.set_title("vx (drag + tire roll losses)")
ax_vx.grid(True, alpha=0.3); ax_vx.legend(fontsize=8)
plt.tight_layout()
plt.savefig(OUT / "rk4_vs_euler.png", dpi=120)
plt.close(fig)

# Numerical summary
r_rk4_end = df_rk4["r"].iloc[-1]
r_eul_end = df_eul["r"].iloc[-1]
delta_r = r_rk4_end - r_eul_end
print(f"r_RK4_end   = {r_rk4_end:.5f} rad/s")
print(f"r_Euler_end = {r_eul_end:.5f} rad/s")
print(f"delta       = {delta_r:+.5f} ({delta_r / r_rk4_end * 100:+.2f} %)")


# -----------------------------------------------------------------------------
# Schema overview figure — visual count of fields per params struct
# -----------------------------------------------------------------------------
schema = {
    "VehicleParams": 23,   # 23 scalar/array fields
    "TireParams":    15,   # 12 base + 3 combined-slip / Mz
    "SolverParams":  3,
}
fig, ax = plt.subplots(figsize=(7, 3.5))
bars = ax.barh(list(schema.keys()), list(schema.values()),
               color=["#4F81BD", "#01A0E9", "#345A8A"])
ax.set_xlabel("# scalar fields  (each YAML-roundtrip-tested)")
ax.set_title("Params YAML schema coverage after Task 16")
for b, v in zip(bars, schema.values()):
    ax.text(v + 0.3, b.get_y() + b.get_height()/2,
            str(v), va="center", fontsize=10)
ax.set_xlim(0, max(schema.values()) * 1.15)
ax.grid(True, axis="x", alpha=0.3)
plt.tight_layout()
plt.savefig(OUT / "yaml_schema_overview.png", dpi=120)
plt.close(fig)

print(f"figures -> {OUT}")
