"""
Plot L1 bicycle steady-state cornering trajectory + yaw rate transient.

Implements the same bicycle dynamics as core/src/bicycle_dynamics.cpp
(Pacejka MF96, ISO 8855 RH slip angle, RK4 substepping).

Outputs:
    docs/tasks/11_W4_bicycle_dynamics/figures/bicycle_trajectory.png
    docs/tasks/11_W4_bicycle_dynamics/figures/bicycle_yaw_rate.png
    docs/tasks/11_W4_bicycle_dynamics/figures/bicycle_sweep_vs_analytical.png
"""

from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


GRAVITY = 9.80665
RHO_AIR = 1.225


# ------- TireParams (default) -------
class TireParams:
    B_long, C_long, D_long, E_long = 10.0, 1.65, 1.0,  0.97
    B_lat,  C_lat,  D_lat,  E_lat  = 8.0,  1.30, 1.0, -1.0
    mu_nominal = 1.0


# ------- VehicleParams (default) -------
class VehicleParams:
    mass            = 1500.0
    inertia_z       = 2500.0
    wheelbase       = 2.7
    cg_to_front     = 1.2
    cg_to_rear      = 1.5
    wheel_radius    = 0.32
    aero_drag_coeff = 0.0          # disabled for clean SS comparison
    frontal_area    = 2.2


def pacejka_lat(alpha, Fz, mu=1.0):
    tp = TireParams
    Dy = tp.D_lat * Fz * tp.mu_nominal * mu
    t  = tp.B_lat * alpha
    phi = t - tp.E_lat * (t - np.arctan(t))
    return -Dy * np.sin(tp.C_lat * np.arctan(phi))


def pacejka_lon(kappa, Fz, mu=1.0):
    tp = TireParams
    Dx = tp.D_long * Fz * tp.mu_nominal * mu
    t  = tp.B_long * kappa
    phi = t - tp.E_long * (t - np.arctan(t))
    return Dx * np.sin(tp.C_long * np.arctan(phi))


def derivatives(state, delta, vp, mu_long=1.0, mu_lat=1.0):
    """Returns dstate/dt for bicycle. state = [x, y, yaw, vx, vy, r, wf, wr]."""
    x, y, yaw, vx, vy, r, wf, wr = state
    a, b, L = vp.cg_to_front, vp.cg_to_rear, vp.wheelbase
    m, Izz  = vp.mass, vp.inertia_z
    R       = vp.wheel_radius

    Fz_f = m * GRAVITY * b / L
    Fz_r = m * GRAVITY * a / L

    # Velocities at wheel positions
    v_fx_b = vx
    v_fy_b = vy + a * r
    v_rx_b = vx
    v_ry_b = vy - b * r

    cd, sd = np.cos(delta), np.sin(delta)
    v_fx_w =  v_fx_b * cd + v_fy_b * sd
    v_fy_w = -v_fx_b * sd + v_fy_b * cd

    alpha_f = np.arctan2(v_fy_w, v_fx_w)
    alpha_r = np.arctan2(v_ry_b, v_rx_b)

    denom_f = max(abs(v_fx_w), 0.5)
    denom_r = max(abs(v_rx_b), 0.5)
    kappa_f = (R * wf - v_fx_w) / denom_f
    kappa_r = (R * wr - v_rx_b) / denom_r

    Fx_w_f = pacejka_lon(kappa_f, Fz_f, mu_long)
    Fy_w_f = pacejka_lat(alpha_f, Fz_f, mu_lat)
    Fx_w_r = pacejka_lon(kappa_r, Fz_r, mu_long)
    Fy_w_r = pacejka_lat(alpha_r, Fz_r, mu_lat)

    Fx_b_f = Fx_w_f * cd - Fy_w_f * sd
    Fy_b_f = Fx_w_f * sd + Fy_w_f * cd
    Fx_b_r = Fx_w_r
    Fy_b_r = Fy_w_r

    F_aero = 0.5 * RHO_AIR * vp.aero_drag_coeff * vp.frontal_area * vx * abs(vx)

    Fx_total = Fx_b_f + Fx_b_r - F_aero
    Fy_total = Fy_b_f + Fy_b_r
    Mz_total = a * Fy_b_f - b * Fy_b_r

    I_wheel = 0.5 * 40.0 * R * R           # matches C++ default
    dx_w = vx * np.cos(yaw) - vy * np.sin(yaw)
    dy_w = vx * np.sin(yaw) + vy * np.cos(yaw)

    return np.array([
        dx_w,
        dy_w,
        r,
        Fx_total / m + vy * r,
        Fy_total / m - vx * r,
        Mz_total / Izz,
        -Fx_w_f * R / I_wheel,
        -Fx_w_r * R / I_wheel,
    ])


def rk4_step(state, delta, dt, vp, mu_long=1.0, mu_lat=1.0):
    k1 = derivatives(state,                delta, vp, mu_long, mu_lat)
    k2 = derivatives(state + 0.5*dt*k1,    delta, vp, mu_long, mu_lat)
    k3 = derivatives(state + 0.5*dt*k2,    delta, vp, mu_long, mu_lat)
    k4 = derivatives(state + dt*k3,        delta, vp, mu_long, mu_lat)
    return state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)


def simulate(delta, vx0, duration, dt=1e-3, mu_long=1.0, mu_lat=1.0):
    vp = VehicleParams
    w0 = vx0 / vp.wheel_radius
    state = np.array([0.0, 0.0, 0.0, vx0, 0.0, 0.0, w0, w0])
    N = int(round(duration / dt))
    log = np.zeros((N + 1, len(state)))
    log[0] = state
    for i in range(N):
        state = rk4_step(state, delta, dt, vp, mu_long, mu_lat)
        log[i + 1] = state
    t = np.arange(N + 1) * dt
    return t, log


def analytical_yaw_rate(vx, delta, vp=VehicleParams, tp=TireParams):
    """Linear-bicycle steady-state yaw rate."""
    m = vp.mass
    a = vp.cg_to_front
    b = vp.cg_to_rear
    L = vp.wheelbase
    Fz_f = m * GRAVITY * b / L
    Fz_r = m * GRAVITY * a / L
    Cf = tp.B_lat * tp.C_lat * tp.D_lat * Fz_f * tp.mu_nominal
    Cr = tp.B_lat * tp.C_lat * tp.D_lat * Fz_r * tp.mu_nominal

    A11 = (Cf + Cr) / vx
    A12 = (a*Cf - b*Cr) / vx - m * vx
    A21 = (a*Cf - b*Cr) / vx
    A22 = (a*a*Cf + b*b*Cr) / vx
    B1  = Cf * delta
    B2  = a * Cf * delta
    det = A11 * A22 - A12 * A21
    return (A11 * B2 - A21 * B1) / det


OUT_DIR = Path(__file__).resolve().parent.parent / "tasks/11_W4_bicycle_dynamics/figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# -----------------------------------------------------------------------------
# Fig 1: trajectory (x-y) for delta = +/-0.05 at vx = 10 m/s
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(5.5, 5.0))
for d, lbl in [(0.05, "left, delta = +0.05"), (-0.05, "right, delta = -0.05"), (0.0, "straight")]:
    t, log = simulate(d, 10.0, 8.0)
    ax.plot(log[:, 0], log[:, 1], label=lbl)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("L1 Bicycle trajectory (vx0 = 10 m/s, 8 s)")
ax.set_aspect("equal", "datalim")
ax.grid(True, alpha=0.3)
ax.legend()
plt.tight_layout()
plt.savefig(OUT_DIR / "bicycle_trajectory.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 2: yaw rate transient + analytical SS
# -----------------------------------------------------------------------------
delta_test = 0.05
vx0_test   = 10.0
t, log = simulate(delta_test, vx0_test, 5.0)
r_sim = log[:, 5]
r_ana = analytical_yaw_rate(vx0_test, delta_test)

fig, ax = plt.subplots(figsize=(6.5, 4.0))
ax.plot(t, r_sim, label="VDSim L1 bicycle (Pacejka)")
ax.axhline(r_ana, color="r", linestyle="--",
           label=f"analytical SS = {r_ana:.4f} rad/s")
ax.set_xlabel("time [s]")
ax.set_ylabel("yaw rate r [rad/s]")
ax.set_title("Step steer response (delta = 0.05 rad, vx0 = 10 m/s)")
ax.grid(True, alpha=0.3)
ax.legend()

# annotate final value and error
r_end = r_sim[-1]
err = (r_end - r_ana) / r_ana * 100
ax.annotate(f"final r = {r_end:.4f} rad/s\nerror = {err:+.2f}%",
            xy=(t[-1], r_end), xytext=(t[-1]*0.55, r_end*0.45),
            arrowprops=dict(arrowstyle="->", color="gray"),
            fontsize=9)
plt.tight_layout()
plt.savefig(OUT_DIR / "bicycle_yaw_rate.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Fig 3: sweep of delta vs analytical, multiple vx
# -----------------------------------------------------------------------------
delta_sweep = np.linspace(-0.10, 0.10, 21)
vx_list = [5.0, 10.0, 20.0]

fig, ax = plt.subplots(figsize=(6.5, 4.5))
for vx in vx_list:
    r_sim_arr = []
    r_ana_arr = []
    for d in delta_sweep:
        t, log = simulate(d, vx, 6.0)
        r_sim_arr.append(log[-1, 5])
        r_ana_arr.append(analytical_yaw_rate(vx, d))
    ax.plot(delta_sweep, np.array(r_sim_arr), 'o-',
            label=f"sim vx = {vx} m/s", markersize=4)
    ax.plot(delta_sweep, np.array(r_ana_arr), '--', alpha=0.5,
            label=f"analytical vx = {vx} m/s")
ax.set_xlabel("steer delta [rad]")
ax.set_ylabel("steady-state yaw rate r [rad/s]")
ax.set_title("Steady-state yaw rate: VDSim vs linear-bicycle analytical")
ax.grid(True, alpha=0.3)
ax.legend(fontsize=8, ncol=2)
plt.tight_layout()
plt.savefig(OUT_DIR / "bicycle_sweep_vs_analytical.png", dpi=120)
plt.close(fig)


# -----------------------------------------------------------------------------
# Tabular agreement
# -----------------------------------------------------------------------------
print("Steady-state yaw rate: simulation vs analytical")
print(f"{'vx[m/s]':>8} {'delta[rad]':>11} {'r_sim':>10} {'r_ana':>10} {'err%':>8}")
for vx in vx_list:
    for d in [0.025, 0.05, 0.10]:
        t, log = simulate(d, vx, 6.0)
        rs = log[-1, 5]
        ra = analytical_yaw_rate(vx, d)
        e  = (rs - ra) / ra * 100
        print(f"{vx:8.1f} {d:11.4f} {rs:10.4f} {ra:10.4f} {e:+7.2f}")

print(f"\nFigures written to {OUT_DIR}")
