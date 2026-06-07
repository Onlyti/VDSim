#!/usr/bin/env python3
"""Monte Carlo / uncertainty runner — chance-constraint validation.

Samples vehicle/tire/road uncertainty (mass, friction mu, lateral grip,
cornering stiffness) from distributions, runs N closed-loop path-tracking sims,
and aggregates the cross-track error into percentile bands + a constraint
violation rate P(max|e| > bound). This is the empirical side of the thesis axis
"estimated-parameter covariance P -> stochastic-MPC chance constraint": plug
your controller in place of the reference pure-pursuit and the same machinery
reports whether P(violation) <= epsilon holds under the sampled uncertainty.

Usage:
    cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build -j
    python3 examples/monte_carlo.py [N]
"""
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim  # noqa: E402

R_PATH = 30.0       # reference circle radius [m]
V_TARGET = 13.0     # ay = v^2/R ~ 5.6 m/s^2 — brisk but sub-limit, so the
                    # sampled mu/grip uncertainty (not controller bias) drives
                    # the cross-track spread across the corridor.
E_BOUND = 0.4       # cross-track corridor half-width [m] (the chance constraint)


def sample_params(rng):
    """Draw a vehicle/tire/road realization from its uncertainty."""
    from _catalog_load import load_vehicle_tire
    vp, tp = load_vehicle_tire()
    vp.mass *= 1.0 + 0.05 * rng.standard_normal()                 # +-5% mass
    tp.cornering_stiffness *= 1.0 + 0.10 * rng.standard_normal()  # +-10% Calpha
    tp.D_lat *= max(0.6, 1.0 + 0.06 * rng.standard_normal())      # lateral grip
    mu = float(rng.uniform(0.85, 1.05))                           # road friction
    return vp, tp, mu


def pursuit_steer(x, y, yaw, vx, wheelbase):
    """Pure-pursuit toward a lookahead point on the reference circle (center (0,R))."""
    cx, cy = 0.0, R_PATH
    rel = (x - cx, y - cy)
    alpha = math.atan2(rel[1], rel[0])
    Ld = max(3.0, 0.6 * max(vx, 1.0))
    tgt = (cx + R_PATH * math.cos(alpha + Ld / R_PATH),
           cy + R_PATH * math.sin(alpha + Ld / R_PATH))
    dxw, dyw = tgt[0] - x, tgt[1] - y
    dxb = math.cos(yaw) * dxw + math.sin(yaw) * dyw
    dyb = -math.sin(yaw) * dxw + math.cos(yaw) * dyw
    l2 = dxb * dxb + dyb * dyb
    if l2 < 1e-6:
        return 0.0
    return max(-0.6, min(0.6, math.atan(2.0 * dyb / l2 * wheelbase)))


def run_once(vp, tp, mu, dt=0.01, duration=20.0):
    sess = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=dt, mu=mu)
    sess.reset(vdsim.make_init_state(0, 0, 0, V_TARGET, vp.wheel_radius_nominal))
    n = int(duration / dt)
    e = np.zeros(n)
    for k in range(n):
        s = sess.state()
        x, y, yaw, vx = (s.position[0], s.position[1], s.yaw(), s.vx())
        cmd = vdsim.CmdL4()
        cmd.steer_angle_wheel = pursuit_steer(x, y, yaw, vx, vp.wheelbase)
        ax = max(-3.0, min(3.0, 0.8 * (V_TARGET - vx)))
        if ax >= 0:
            cmd.throttle = min(1.0, ax / 3.0)
        else:
            cmd.brake = min(1.0, -ax / 3.0)
        sess.set_input(cmd)
        sess.tick(dt)
        s = sess.state()
        e[k] = math.hypot(s.position[0] - 0.0, s.position[1] - R_PATH) - R_PATH
    return e


def run_monte_carlo(N, seed=0, dt=0.01, duration=20.0):
    rng = np.random.default_rng(seed)
    runs = []
    for i in range(N):
        vp, tp, mu = sample_params(rng)
        runs.append(run_once(vp, tp, mu, dt, duration))
        if (i + 1) % max(1, N // 10) == 0:
            print(f"  ... {i + 1}/{N}")
    return np.array(runs), np.arange(int(duration / dt)) * dt


def main():
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 128
    print(f"=== Monte Carlo: N={N}, circle R={R_PATH} m @ {V_TARGET} m/s ===")
    E, t = run_monte_carlo(N)
    absE = np.abs(E)
    p50 = np.percentile(absE, 50, axis=0)
    p95 = np.percentile(absE, 95, axis=0)
    run_max = absE.max(axis=1)
    viol = float(np.mean(run_max > E_BOUND)) * 100.0
    print(f"  |cross-track| p50(max over t): {np.median(run_max):.3f} m")
    print(f"  |cross-track| p95(max over t): {np.percentile(run_max, 95):.3f} m")
    print(f"  constraint |e| <= {E_BOUND} m  -> violation rate P = {viol:.1f} %")

    out = REPO / "logs"; out.mkdir(exist_ok=True)
    csvp = out / "monte_carlo_bands.csv"
    with open(csvp, "w") as f:
        f.write("t,p50_abs_e,p95_abs_e\n")
        for k in range(len(t)):
            f.write(f"{t[k]:.4f},{p50[k]:.5f},{p95[k]:.5f}\n")
    print(f"  bands -> {csvp}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.5))
        for r in absE[:: max(1, N // 20)]:
            ax.plot(t, r, color="0.8", lw=0.6)
        ax.plot(t, p95, "r", lw=1.6, label="p95")
        ax.plot(t, p50, "b", lw=1.6, label="p50 (median)")
        ax.axhline(E_BOUND, color="k", ls="--", lw=1, label=f"bound {E_BOUND} m")
        ax.set_xlabel("time [s]"); ax.set_ylabel("|cross-track error| [m]")
        ax.set_title(f"Monte Carlo cross-track error (N={N})")
        ax.legend(loc="upper right", fontsize=8); fig.tight_layout()
        png = out / "monte_carlo_bands.png"; fig.savefig(png, dpi=110)
        print(f"  plot  -> {png}")
    except Exception as ex:
        print(f"  (plot skipped: {ex})")


if __name__ == "__main__":
    main()
