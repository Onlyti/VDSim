#!/usr/bin/env python3
"""VDSimPlant actuator= : longitudinal + steer channels in the direct-control path."""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim  # noqa: E402
from vdsim_plant import VDSimPlant, build_actuator_params  # noqa: E402

DT, SUB = 0.01, 0.001
V0 = 15.0


def _plant(**kw):
    p = VDSimPlant(config="generic_sedan.yaml", level="L3", control_dt=DT, substep_dt=SUB, **kw)
    p.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    return p


def _fx_for_throttle(p, thr):
    return thr * p._vp.max_motor_torque * p._vp.final_drive_ratio / p._vp.wheel_radius_nominal


def test_default_is_identity():
    a, b = _plant(), _plant(actuator={})
    for k in range(200):
        u = [0.04 * math.sin(0.05 * k), 900.0 * math.sin(0.03 * k)]
        assert repr(a.step(u)) == repr(b.step(u))


def test_throttle_dead_time_and_lag_match_analytic():
    L, tau, thr = 0.10, 0.20, 0.4
    p = _plant(actuator={"throttle": {"dead_time_s": L, "tau_s": tau}})
    fx = _fx_for_throttle(p, thr)
    worst = 0.0
    for k in range(120):
        p.step([0.0, fx])
        t = (k + 1) * DT
        want = 0.0 if t <= L else thr * (1.0 - math.exp(-(t - L) / tau))
        worst = max(worst, abs(p._sess.output().throttle_applied - want))
    assert worst < 1e-9, worst


def test_delay_changes_the_trajectory_and_keeps_wheel_shares():
    plain, slow = _plant(), _plant(actuator={"throttle": {"dead_time_s": 0.3}})
    fx = _fx_for_throttle(plain, 0.3)
    for _ in range(100):
        a, b = plain.step([0.0, fx]), slow.step([0.0, fx])
    assert a["vx"] > b["vx"] + 0.05
    sf = plain._vp.drive_split_front
    wf = sum(slow._dyn.tire_forces_body()[i][0] for i in (0, 1))
    wr = sum(slow._dyn.tire_forces_body()[i][0] for i in (2, 3))
    assert wf > 0 and wr > 0
    assert abs(wf / (wf + wr) - sf) < 0.15


def test_brake_channel_is_applied_and_released_after_delay():
    p = _plant(actuator={"brake": {"dead_time_s": 0.05, "tau_s": 0.0}})
    fx = -0.3 * p._vp.max_brake_torque / p._vp.wheel_radius_nominal
    seen = []
    for k in range(20):
        p.step([0.0, fx])
        seen.append(p._sess.output().brake_applied)
    assert seen[3] == 0.0 and seen[-1] > 0.25


def test_steer_channel_lags():
    p = _plant(actuator={"steer": {"dead_time_s": 0.05}})
    seen = []
    for _ in range(10):
        p.step([0.1, 0.0])
        seen.append(p._sess.output().steer_applied)
    assert seen[3] == 0.0 and abs(seen[-1] - 0.1) < 1e-9


def test_set_actuator_midflight_and_reset_returns_to_built():
    p = _plant()
    for _ in range(20):
        p.step([0.0, 500.0])
    p.set_actuator({"throttle": {"tau_s": 0.5}})
    assert abs(p.actuator.throttle.tau_s - 0.5) < 1e-12
    p.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    p.step([0.0, 500.0])


def test_snapshot_replay_matches_within_ulp():
    act = {"throttle": {"dead_time_s": 0.1, "tau_s": 0.2}, "brake": {"dead_time_s": 0.05}}
    p = _plant(actuator=act)
    fx = _fx_for_throttle(p, 0.4)
    out, snap = [], None
    for k in range(100):
        if k == 40:
            snap = p._sess.snapshot()
        out.append(p.step([0.02, fx if k < 60 else -fx]))
    p._sess.restore(snap)
    for k in range(40, 100):
        o = p.step([0.02, fx if k < 60 else -fx])
        for key in ("X", "Y", "psi", "vx", "vy", "r"):
            assert abs(o[key] - out[k][key]) < 1e-9, (k, key)


def test_bad_specs_raise():
    for bad in ({"thrott": {}}, {"throttle": {"tau": 1.0}}, {"throttle": {"tau_s": float("nan")}}, 3):
        try:
            build_actuator_params(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")
    assert isinstance(build_actuator_params(None), vdsim.ActuatorParams)


def test_backlash_play_operator():
    p = _plant(actuator={"steer": {"backlash": 0.02}})
    def applied(delta):
        for _ in range(3):
            p.step([delta, 0.0])
        return p._sess.output().steer_applied
    assert abs(applied(0.10) - 0.09) < 1e-12      # engaged: follows at u - w/2
    assert abs(applied(0.095) - 0.09) < 1e-12     # inside the band: holds
    assert abs(applied(0.07) - 0.08) < 1e-12      # reversal: follows at u + w/2


class _Gain:
    def __init__(self, k):
        self.k, self.n = k, 0

    def step(self, u, dt):
        self.n += 1
        return self.k * u

    def reset(self):
        self.n = 0

    def get_state(self):
        return [float(self.n)]

    def set_state(self, v):
        self.n = int(v[0])


def test_user_channel_stage_runs_and_is_not_shared():
    proto = _Gain(0.5)
    a = _plant(actuator={"steer": {"user": proto}})
    b = _plant(actuator={"steer": {"user": proto}})
    for _ in range(5):
        a.step([0.1, 0.0])
    assert abs(a._sess.output().steer_applied - 0.05) < 1e-12
    b.step([0.1, 0.0])
    assert proto.n == 0                           # prototype is never stepped
    assert a._sess.actuator().steer.ch.user is proto


class _Drift(_Gain):
    def step(self, u, dt):
        self.n += 1
        return self.k * u + 1e-6 * self.n


def test_user_throttle_stage_scales_force():
    plain = _plant()
    half = _plant(actuator={"throttle": {"user": _Gain(0.5)}})
    fx = _fx_for_throttle(plain, 0.4)
    for _ in range(50):
        plain.step([0.0, fx])
        half.step([0.0, fx])
    assert abs(half._sess.output().throttle_applied - 0.2) < 1e-9
    assert plain.step([0.0, fx])["vx"] > half.step([0.0, fx])["vx"]


def test_user_stage_state_survives_snapshot():
    p = _plant(actuator={"throttle": {"user": _Drift(1.0)}})
    fx = _fx_for_throttle(p, 0.4)
    for _ in range(30):
        p.step([0.0, fx])
    snap = p._sess.snapshot()
    p.step([0.0, fx])
    first = p._sess.output().throttle_applied
    p.step([0.0, fx])
    p._sess.restore(snap)
    p.step([0.0, fx])
    assert p._sess.output().throttle_applied == first


def test_preset_id_and_override():
    p = _plant(actuator="actuator.generic_sedan")
    a = p.actuator
    assert abs(a.steer.ch.dead_time_s - 0.05) < 1e-12 and abs(a.steer.ch.tau_s - 0.08) < 1e-12
    q = _plant(actuator={"preset": "actuator.generic_sedan", "steer": {"dead_time_s": 0.2}})
    assert abs(q.actuator.steer.ch.dead_time_s - 0.2) < 1e-12
    assert abs(q.actuator.steer.ch.tau_s - 0.08) < 1e-12      # untouched field kept
    ideal = build_actuator_params("actuator.ideal")
    assert ideal.steer.ch.dead_time_s == 0.0 and ideal.throttle.tau_s == 0.0
    try:
        build_actuator_params("brake.sedan")
    except ValueError:
        pass
    else:
        raise AssertionError("non-actuator part accepted")


def test_steer_friction_and_brake_thermal_specs():
    ap = build_actuator_params({
        "steer": {"friction": {"Tc": 0.8, "Ts": 1.2}, "servo_kp": 80.0},
        "brake": {"thermal": {"heat_coeff": 1e-3, "cool_coeff": 1e-2,
                              "mu_T_temp": [20.0, 400.0], "mu_T_scale": [1.0, 0.6]}}})
    assert ap.steer.friction.enabled and abs(ap.steer.friction.Tc - 0.8) < 1e-12
    assert ap.brake.thermal_enabled and list(ap.brake.mu_T_scale) == [1.0, 0.6]


def _steer_applied_series(actuator, n=80):
    """Applied road-wheel steer after each step of a one-substep plant.

    The command varies every step, so a pure shift is distinguishable from a
    constant that merely starts late.
    """
    p = VDSimPlant(config="generic_sedan.yaml", level="L3", control_dt=SUB,
                   substep_dt=SUB, actuator=actuator)
    p.reset([0.0, 0.0, 0.0, V0, 0.0, 0.0])
    out = []
    for k in range(n):
        p.step([0.05 + 0.04 * math.sin(0.3 * k), 0.0])
        out.append(p._sess.output().steer_applied)
    return out


def test_steer_dead_time_is_an_exact_substep_shift():
    """B1 (work order 32): steer dead time D delays the applied steer by D/substep.

    Replaces the retired direct-control steer_dead_time_s option: the same
    field is filled through actuator={"steer": {"dead_time_s": D}}. Checked for
    D = 5 ms and 20 ms with and without the first-order lag; a shift one
    substep short must not match.
    """
    for tau in (0.0, 0.02):
        ref = _steer_applied_series({"steer": {"dead_time_s": 0.0, "tau_s": tau}})
        assert any(v != 0.0 for v in ref[:2]), "zero dead time must act at once"
        for D in (0.005, 0.020):
            n = int(round(D / SUB))
            got = _steer_applied_series({"steer": {"dead_time_s": D, "tau_s": tau}})
            assert all(v == 0.0 for v in got[:n]), (tau, D, got[:n + 1])
            assert got[n:] == ref[:len(ref) - n], f"tau={tau} D={D}: not a {n}-substep shift"
            assert got[n - 1:] != ref[:len(ref) - n + 1], f"tau={tau} D={D}: shift test cannot tell n-1"


def test_steer_zero_dead_time_equals_no_actuator():
    """B1: dead_time_s = 0 on the steer channel is bit-identical to actuator=None."""
    a = _plant()
    b = _plant(actuator={"steer": {"dead_time_s": 0.0}})
    for k in range(200):
        u = [0.04 * math.sin(0.05 * k), 900.0 * math.sin(0.03 * k)]
        assert repr(a.step(u)) == repr(b.step(u))


if __name__ == "__main__":
    n = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            n += 1
            print("PASS", name)
    print(f"{n} passed")
