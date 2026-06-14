#!/usr/bin/env python3
"""ISO standard-maneuver library — runnable presets with objective metrics.

Implements the maneuvers specced in theory ch15 as one-call functions:
  - ISO 7401 step steer        -> yaw-rate response (steady value, 90% rise, overshoot)
  - ISO 4138 steady-state circle -> understeer gradient K_us [deg/g]
  - ISO 3888-2 double lane change -> path follow, peak yaw rate / lateral accel, pass

Each builds a headless SimSession and returns a metrics dict; `main` runs all
and prints a table. Use for reproducible validation and FSK-relevant tests.

Usage: python3 examples/maneuvers.py
"""
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim  # noqa: E402

G = 9.81


def _veh(level="L2"):
    from _catalog_load import load_vehicle_tire
    return load_vehicle_tire()


def _throttle_to(vx, v_target):
    ax = max(-3.0, min(3.0, 0.8 * (v_target - vx)))
    c = vdsim.CmdL4()
    if ax >= 0:
        c.throttle = min(1.0, ax / 3.0)
    else:
        c.brake = min(1.0, -ax / 3.0)
    return c


def step_steer(v=20.0, steer=0.03, dt=0.005, settle=3.0, hold=4.0, level="L2", veh=None,
               trace=False):
    """ISO 7401: hold speed, step the steer, measure the yaw-rate response."""
    vp, tp = veh if veh is not None else _veh(level)
    sess = vdsim.make_sim_session(vp, tp, level, nominal_dt=dt)
    sess.reset(vdsim.make_init_state(0, 0, 0, v, vp.wheel_radius_nominal))
    t, r = [], []
    n0, n1 = int(settle / dt), int((settle + hold) / dt)
    for k in range(n1):
        s = sess.state()
        c = _throttle_to(s.vx(), v)
        c.steer_angle_wheel = steer if k >= n0 else 0.0
        sess.set_input(c); sess.tick(dt)
        if k >= n0:
            t.append((k - n0) * dt); r.append(sess.state().yaw_rate())
    r = np.array(r); t = np.array(t)
    r_ss = float(np.mean(r[-int(0.5 / dt):]))           # last 0.5 s average
    r_pk = float(np.max(np.abs(r))) * np.sign(r_ss)
    overshoot = (abs(r_pk) - abs(r_ss)) / abs(r_ss) * 100.0 if abs(r_ss) > 1e-6 else 0.0
    rise = next((tt for tt, rr in zip(t, r) if abs(rr) >= 0.9 * abs(r_ss)), float("nan"))
    out = {"maneuver": "ISO7401 step-steer", "r_ss[rad/s]": round(r_ss, 4),
           "rise90[s]": round(rise, 3), "overshoot[%]": round(overshoot, 1)}
    if trace:                                    # yaw-rate(t) for overlay charts
        k = max(1, len(t) // 200)                # downsample to ~200 points
        out["trace"] = {"t": [round(float(x), 4) for x in t[::k]],
                        "r": [round(float(x), 5) for x in r[::k]]}
    return out


def skidpad_understeer(R=40.0, speeds=(8, 12, 16, 19), dt=0.005, level="L2", veh=None):
    """ISO 4138: hold a constant-radius circle at several speeds, fit the
    understeer gradient K_us = d(delta - delta_ack)/d(ay)."""
    vp, tp = veh if veh is not None else _veh(level)
    L = vp.wheelbase
    ay_list, dsw_list = [], []
    for v in speeds:
        sess = vdsim.make_sim_session(vp, tp, level, nominal_dt=dt)
        sess.reset(vdsim.make_init_state(0, 0, 0, v, vp.wheel_radius_nominal))
        steers = []
        for k in range(int(12.0 / dt)):
            s = sess.state()
            x, y, yaw, vx = s.position[0], s.position[1], s.yaw(), s.vx()
            cx, cy = 0.0, R
            a = math.atan2(y - cy, x - cx)
            Ld = max(3.0, 0.5 * max(vx, 1.0))
            tx, ty = cx + R * math.cos(a + Ld / R), cy + R * math.sin(a + Ld / R)
            dxb = math.cos(yaw) * (tx - x) + math.sin(yaw) * (ty - y)
            dyb = -math.sin(yaw) * (tx - x) + math.cos(yaw) * (ty - y)
            l2 = dxb * dxb + dyb * dyb
            c = _throttle_to(vx, v)
            c.steer_angle_wheel = 0.0 if l2 < 1e-6 else max(-0.5, min(0.5, math.atan(2 * dyb / l2 * L)))
            sess.set_input(c); sess.tick(dt)
            if k > int(8.0 / dt):
                steers.append(c.steer_angle_wheel)
        ay = v * v / R
        ay_list.append(ay / G); dsw_list.append(np.degrees(np.mean(steers)))
    # K_us = slope of (delta - delta_ackermann) vs ay[g]; delta_ack = L/R (rad)
    d_ack = math.degrees(L / R)
    slope = float(np.polyfit(ay_list, np.array(dsw_list) - d_ack, 1)[0])
    sign = "understeer" if slope > 0 else ("oversteer" if slope < 0 else "neutral")
    return {"maneuver": "ISO4138 skidpad", "K_us[deg/g]": round(slope, 3),
            "balance": sign, "delta_ack[deg]": round(d_ack, 2)}


def double_lane_change(v=14.0, dt=0.005, level="L2", veh=None):
    """ISO 3888-2 (moose): pure-pursuit through an offset-then-return path."""
    vp, tp = veh if veh is not None else _veh(level)
    L = vp.wheelbase
    # piecewise lateral target: out by 3.5 m between 30..45 m, back by 60 m
    def y_ref(x):
        if x < 15: return 0.0
        if x < 45: return 3.5 * 0.5 * (1 - math.cos(math.pi * min(1, (x - 15) / 15)))
        if x < 70: return 3.5 * 0.5 * (1 + math.cos(math.pi * min(1, (x - 45) / 15)))
        return 0.0
    sess = vdsim.make_sim_session(vp, tp, level, nominal_dt=dt)
    sess.reset(vdsim.make_init_state(0, 0, 0, v, vp.wheel_radius_nominal))
    peak_r, peak_ay = 0.0, 0.0
    for k in range(int(8.0 / dt)):
        s = sess.state()
        x, y, yaw, vx = s.position[0], s.position[1], s.yaw(), s.vx()
        Ld = max(4.0, 0.6 * vx)
        tx = x + Ld
        ty = y_ref(tx)
        dxb = math.cos(yaw) * (tx - x) + math.sin(yaw) * (ty - y)
        dyb = -math.sin(yaw) * (tx - x) + math.cos(yaw) * (ty - y)
        l2 = dxb * dxb + dyb * dyb
        c = _throttle_to(vx, v)
        c.steer_angle_wheel = 0.0 if l2 < 1e-6 else max(-0.5, min(0.5, math.atan(2 * dyb / l2 * L)))
        sess.set_input(c); sess.tick(dt)
        o = sess.output()
        peak_r = max(peak_r, abs(o.state.yaw_rate()))
        peak_ay = max(peak_ay, abs(o.ay))
    completed = bool(sess.state().position[0] > 75.0)
    return {"maneuver": "ISO3888-2 DLC", "v[m/s]": v, "peak_r[rad/s]": round(peak_r, 3),
            "peak_ay[g]": round(peak_ay / G, 2), "completed": completed}


def main():
    results = [step_steer(), skidpad_understeer(), double_lane_change()]
    print("=== ISO maneuver library ===")
    for m in results:
        print("  " + "  ".join(f"{k}={v}" for k, v in m.items()))


if __name__ == "__main__":
    main()
