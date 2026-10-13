#!/usr/bin/env python3
"""Python surface of CascadeGains (set_cascade_gains) and SensorNoise.lag_tau_s."""
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim  # noqa: E402


def _session(**kw):
    vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
    s = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=0.005, **kw)
    s.reset(vdsim.make_init_state(v=20.0, wheel_radius=vp.wheel_radius_nominal))
    return s


def _speed_after(s, v_target, seconds):
    cmd = vdsim.CmdL6()
    cmd.v_target = v_target
    for _ in range(int(seconds / 0.005)):
        s.set_input(cmd)
        s.tick(0.005)
    return s.output().state.vx()


def test_cascade_gains_defaults_and_roundtrip():
    g = vdsim.CascadeGains()
    assert (g.vx.kp, g.vx.ki, g.ax.kp, g.ax.ki, g.ax.kff) == (0.8, 0.20, 0.4, 0.6, 0.10)
    assert (g.pp_lookahead_min, g.pp_lookahead_k, g.yaw_kp, g.yaw_ki) == (1.5, 0.40, 0.15, 0.05)
    s = _session()
    assert s.cascade_gains().vx.kp == 0.8
    g.vx.kp, g.ax.kd = 1.2, 0.01
    s.set_cascade_gains(g)
    got = s.cascade_gains()
    assert got.vx.kp == 1.2 and got.ax.kd == 0.01


def test_cascade_gains_default_set_is_bit_identical():
    a, b = _session(), _session()
    b.set_cascade_gains(vdsim.CascadeGains())
    assert _speed_after(a, 10.0, 6.0) == _speed_after(b, 10.0, 6.0)


def test_cascade_gains_change_tracking_and_reject_bad_values():
    off = _session()
    g = vdsim.CascadeGains()
    g.vx.kp = g.vx.ki = 0.0
    off.set_cascade_gains(g)
    assert _speed_after(_session(), 10.0, 15.0) < 11.5
    assert _speed_after(off, 10.0, 15.0) > 15.0
    bad = vdsim.CascadeGains()
    bad.yaw_kp = -1.0
    try:
        off.set_cascade_gains(bad)
    except ValueError:
        pass
    else:
        raise AssertionError("negative gain must be rejected")
    assert off.cascade_gains().yaw_kp == 0.15


def test_vec_session_cascade_gains_broadcast_and_length_check():
    vp, tp = vdsim.VehicleParams(), vdsim.TireParams()
    v = vdsim.make_vec_session(2, vp, tp, "L2", nominal_dt=0.005)
    v.set_cascade_gains([vdsim.CascadeGains()])
    v.set_cascade_gains([vdsim.CascadeGains(), vdsim.CascadeGains()])
    try:
        v.set_cascade_gains([vdsim.CascadeGains()] * 3)
    except ValueError:
        pass
    else:
        raise AssertionError("wrong length must be rejected")


def test_sensor_lag_field_trails_truth_and_yaml_roundtrip():
    sp = vdsim.SensorParams()
    sp.enabled = True
    sp.gnss_vel.lag_tau_s = 0.5
    with tempfile.TemporaryDirectory() as d:
        p = str(Path(d) / "sensors.yaml")
        sp.to_yaml(p)
        assert vdsim.SensorParams.from_yaml(p).gnss_vel.lag_tau_s == 0.5
    lag, plain = _session(sensors=sp), _session()
    cmd = vdsim.CmdL4()
    cmd.throttle = 1.0
    for s in (lag, plain):
        s.set_input(cmd)
        s.tick(0.005)
    assert abs(lag.output().sensors.gnss_vx - 20.0) < 0.1, "first sample must be primed on truth"
    for _ in range(400):
        lag.tick(0.005)
    o = lag.output()
    assert o.sensors.gnss_vx < o.state.vx() - 0.2


if __name__ == "__main__":
    n = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            n += 1
            print("PASS", name)
    print(f"{n} passed")
