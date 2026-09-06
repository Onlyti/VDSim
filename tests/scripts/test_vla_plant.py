#!/usr/bin/env python3
"""VLA plant acceptance smokes (docs/design/VLA_THESIS_PLANT.md)."""
import math
import sys
import os
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim  # noqa: E402
from vdsim_plant import VDSimPlant  # noqa: E402

V0 = 16.7
DT = 0.05
SUB = 5e-4


def _sum_forces_body(plant):
    fb = plant._dyn.tire_forces_body()
    fx = sum(fb[i][0] for i in range(4))
    fy = sum(fb[i][1] for i in range(4))
    return fx, fy


def _friction_usage(obs, slack=50.0):
    for i, w in enumerate(obs["wheel"]):
        fxy = math.hypot(w["Fx"], w["Fy"])
        lim = 1.10 * w["mu"] * w["Fz"] + slack
        if fxy > lim:
            raise AssertionError(
                f"wheel {i}: ||F||={fxy:.1f} > mu*Fz+eps={lim:.1f}")


def test_dry_qualitative_lane_change():
    plant = VDSimPlant(base_mu=0.9, control_dt=DT, substep_dt=SUB)
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    r_peak = 0.0
    for k in range(80):
        t = k * DT
        if t < 1.0:
            delta = 0.0
        elif t < 2.5:
            delta = 0.06
        elif t < 4.0:
            delta = -0.06
        else:
            delta = 0.0
        obs = plant.step([delta, 0.0])
        r_peak = max(r_peak, abs(obs["r"]))
    assert r_peak > 0.02, f"yaw rate too small for lane change: {r_peak}"
    print("smoke 1 dry qualitative: ok (r_peak={:.4f} rad/s)".format(r_peak))


def test_roll_pitch_in_obs():
    plant = VDSimPlant(base_mu=0.9, control_dt=DT, substep_dt=SUB)
    obs = plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    assert "roll" in obs and "pitch" in obs
    roll_peak = 0.0
    for k in range(80):
        t = k * DT
        delta = 0.06 if 1.0 <= t < 3.0 else 0.0
        obs = plant.step([delta, 0.0])
        roll_peak = max(roll_peak, abs(obs["roll"]))
    assert roll_peak > 0.002, f"expected roll under steer, peak={roll_peak}"
    print("smoke roll/pitch obs: ok (roll_peak={:.4f} rad)".format(roll_peak))


def test_polygon_friction_map_2d():
    plant = VDSimPlant(
        friction_map_2d=[{
            "polygon": [(60.0, -3.0), (140.0, -3.0), (140.0, 3.0), (60.0, 3.0)],
            "mu": 0.5,
        }],
        base_mu=0.9,
        control_dt=DT,
        substep_dt=SUB,
    )
    obs = plant.reset([100.0, 0.0, 0.0, V0, 0.0, 0.0])
    low_mu = all(w["mu"] <= 0.51 for w in obs["wheel"])
    assert low_mu, "inside polygon should see patch mu"
    for _ in range(100):
        obs = plant.step([0.0, 0.0])
    assert obs["X"] > 145.0, f"expected to leave polygon, X={obs['X']}"
    high_mu = all(w["mu"] >= 0.85 for w in obs["wheel"])
    assert high_mu, "outside polygon should see base_mu"
    print("smoke polygon friction_map_2d: ok")


def test_patch_brake_turn_grip_loss():
    # Low-mu patch mid-trajectory; aggressive brake + steer over-demands grip.
    plant = VDSimPlant(
        friction_map=[(60.0, 160.0, 0.5)],
        base_mu=0.9,
        control_dt=DT,
        substep_dt=SUB,
    )
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    saturated = False
    slip_seen = False
    y_drift = 0.0
    fx_cmd = -15000.0
    grip_ratio_min = 1.0
    for k in range(120):
        obs = plant.step([0.12, fx_cmd])
        y_drift = obs["Y"]
        fx_body, _ = _sum_forces_body(plant)
        if obs["vx"] > 2.0:
            grip_ratio_min = min(grip_ratio_min, abs(fx_body) / abs(fx_cmd))
        for w in obs["wheel"]:
            fxy = math.hypot(w["Fx"], w["Fy"])
            cap = w["mu"] * w["Fz"]
            if w["Fz"] > 500.0 and fxy > 0.82 * cap:
                saturated = True
            if abs(w["kappa"]) > 0.04 or abs(w["alpha"]) > 0.03:
                slip_seen = True
    assert saturated, "expected per-wheel friction saturation on patch"
    assert slip_seen, "expected measurable slip (kappa/alpha) under combined demand"
    assert abs(y_drift) > 0.5, f"expected lateral departure, Y={y_drift}"
    assert grip_ratio_min < 0.85, (
        f"expected vehicle-level grip loss on patch, |ΣFx|/|Fx_cmd|={grip_ratio_min:.3f}")
    print("smoke 2 patch brake+turn grip-loss: ok "
          f"(sat={saturated} slip={slip_seen} Y={y_drift:.2f} m)")


def test_gt_consistency_and_determinism():
    plant = VDSimPlant(base_mu=0.9, control_dt=DT, substep_dt=SUB)
    traj_u = [(0.0, 0.0), (0.05, 2000.0), (0.08, -8000.0), (0.03, 500.0)] * 25

    def run_once():
        plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
        hist = []
        m = plant._vp.mass
        for d, fx in traj_u:
            obs = plant.step([d, fx])
            _friction_usage(obs)
            fx_sum, fy_sum = _sum_forces_body(plant)
            ax = obs["ax"]
            ay = obs["ay"]
            tol_fx = 2500.0 + 0.15 * m * abs(ax)
            tol_fy = 2500.0 + 0.15 * m * abs(ay)
            if abs(fx_sum - m * ax) > tol_fx:
                raise AssertionError(
                    f"Fx balance: tire_sum={fx_sum:.0f} m*ax={m*ax:.0f} tol={tol_fx:.0f}")
            if abs(fy_sum - m * ay) > tol_fy:
                raise AssertionError(
                    f"Fy balance: tire_sum={fy_sum:.0f} m*ay={m*ay:.0f} tol={tol_fy:.0f}")
            hist.append(tuple(obs[k] for k in ("X", "Y", "psi", "vx", "vy", "r")))
        return hist

    h1 = run_once()
    h2 = run_once()
    for a, b in zip(h1, h2):
        for x, y in zip(a, b):
            if abs(x - y) > 1e-12:
                raise AssertionError("determinism failed")
    print("smoke 3 GT + determinism: ok ({} steps)".format(len(h1)))


def test_cmdl1_element_torque_assign():
    cmd = vdsim.CmdL1()
    cmd.motor_torque[2] = 999.0
    assert list(cmd.motor_torque)[2] == 999.0
    cmd.brake_torque[1] = 42.0
    assert list(cmd.brake_torque)[1] == 42.0
    cmd.motor_torque = [1.0, 2.0, 3.0, 4.0]
    assert list(cmd.motor_torque) == [1.0, 2.0, 3.0, 4.0]
    print("smoke CmdL1 element torque assign: ok")


def test_longitudinal_fx_accel():
    plant = VDSimPlant(base_mu=0.9, control_dt=DT, substep_dt=SUB)
    plant.reset([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
    m = plant._vp.mass
    fx = 0.4 * m * 9.81
    vx0 = 5.0
    obs = None
    for _ in range(40):
        obs = plant.step([0.0, fx])
    assert obs is not None
    assert obs["ax"] > 0.5, f"expected positive ax under +Fx, got {obs['ax']}"
    assert obs["vx"] > vx0 + 0.5, f"expected speed increase, vx={obs['vx']}"
    fx_wheel = sum(w["Fx"] for w in obs["wheel"])
    assert fx_wheel > 500.0, f"expected drive Fx at wheels, got {fx_wheel:.0f} N"
    print("smoke longitudinal Fx accel: ok (ax={:.2f} vx={:.2f})".format(
        obs["ax"], obs["vx"]))


# Wall-clock budget for 5 s of simulated time. The default is the Release
# contract (real-time factor >= 5x). An unoptimised Debug build runs the same
# code 3-6x slower, which says nothing about a performance regression, so the
# caller states the budget that belongs to the build it produced.
def perf_budget_s(default_s):
    """Wall-clock budget in seconds, taken from the environment if set.

    Mirrors ``vdsim::testing::perf_budget_s`` in tests/support/perf_budget.hpp:
    same variable name, same rule. Unset, empty and malformed all mean "not
    stated" and fall back to the Release contract, so a broken CI expression
    cannot silently disable the assertion.
    """
    raw = os.environ.get("VDSIM_PERF_BUDGET_S", "")
    try:
        parsed = float(raw)
    except ValueError:
        return default_s
    return parsed if parsed > 0.0 else default_s


BUDGET_S = perf_budget_s(1.0)


def test_speed_budget():
    plant = VDSimPlant(base_mu=0.9, control_dt=DT, substep_dt=SUB)
    plant.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    t0 = time.perf_counter()
    for _ in range(100):
        plant.step([0.02, 0.0])
    elapsed = time.perf_counter() - t0
    assert elapsed < BUDGET_S, (
        f"5 s traj too slow: {elapsed:.2f} s wall (budget {BUDGET_S:.2f} s, "
        "override with VDSIM_PERF_BUDGET_S)"
    )
    print("speed budget: ok ({:.3f} s for 5 s sim, budget {:.2f} s)".format(
        elapsed, BUDGET_S))


def main():
    test_cmdl1_element_torque_assign()
    test_longitudinal_fx_accel()
    test_dry_qualitative_lane_change()
    test_roll_pitch_in_obs()
    test_polygon_friction_map_2d()
    test_patch_brake_turn_grip_loss()
    test_gt_consistency_and_determinism()
    test_speed_budget()
    print("test_vla_plant: all ok")


if __name__ == "__main__":
    main()
