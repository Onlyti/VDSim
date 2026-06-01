"""Standard maneuver scenarios for DOE / sensitivity analysis.

Each scenario is a function:  scenario(dyn, params) -> trajectory dict
where trajectory has keys: t, vx, vy, r (yaw_rate), yaw, ay, steer, Fz_avg,
etc.  The dynamics object is a vdsim.IVehicleDynamics (L1/L2/L3).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim
import numpy as np


def _flat_contacts(mu=1.0):
    contacts = vdsim.ContactPoint(), vdsim.ContactPoint(), vdsim.ContactPoint(), vdsim.ContactPoint()
    contacts = list(contacts)
    for c in contacts:
        c.is_valid = True
        c.normal = [0, 0, 1]
        c.mu_long = mu
        c.mu_lat = mu
    return contacts


def step_steer(dyn, params):
    """ISO 7401-ish step steer.
    params:
      v_target [m/s]
      steer_deg [deg]
      t_pre  [s]   (settle time before step)
      t_post [s]   (after step)
      dt     [s]
      mu     surface friction (default 1.0)
    """
    v_target = params["v_target"]
    steer_rad = math.radians(params["steer_deg"])
    t_pre  = params.get("t_pre", 0.5)
    t_post = params.get("t_post", 4.0)
    dt     = params.get("dt", 0.005)
    mu     = params.get("mu", 1.0)

    # Initial state
    s0 = vdsim.State()
    s0.velocity = [v_target, 0.0, 0.0]
    # Wheel spin matching v_target
    r_w = 0.33   # close enough for sports.yaml; could read from params
    w = v_target / r_w
    s0.wheel_spin = [w, w, w, w]
    dyn.reset(s0)

    contacts = _flat_contacts(mu)
    cmd = vdsim.CmdL4()
    cmd.throttle = 0.0
    cmd.brake = 0.0
    cmd.steer_angle_wheel = 0.0

    n_pre  = int(t_pre  / dt)
    n_post = int(t_post / dt)

    traj = {k: [] for k in ("t", "vx", "vy", "r", "ay", "steer", "yaw")}
    t = 0.0
    for i in range(n_pre):
        dyn.step(cmd, contacts, dt)
        st = dyn.state()
        traj["t"].append(t); traj["vx"].append(st.vx()); traj["vy"].append(st.vy())
        traj["r"].append(st.yaw_rate()); traj["yaw"].append(st.yaw())
        traj["ay"].append(dyn.ay_body_est()); traj["steer"].append(0.0)
        t += dt

    # Step: apply steer, hold
    cmd.steer_angle_wheel = steer_rad
    for i in range(n_post):
        dyn.step(cmd, contacts, dt)
        st = dyn.state()
        traj["t"].append(t); traj["vx"].append(st.vx()); traj["vy"].append(st.vy())
        traj["r"].append(st.yaw_rate()); traj["yaw"].append(st.yaw())
        traj["ay"].append(dyn.ay_body_est()); traj["steer"].append(steer_rad)
        t += dt

    return {k: np.array(v) for k, v in traj.items()}


def steady_state_circular(dyn, params):
    """ISO 4138-style steady-state circular driving.
    Quasi-static — ramp steer angle slowly to keep on circle of radius R.
    Simpler version: hold a fixed steer and let vehicle settle.
    params:
      v_target [m/s]
      steer_deg [deg]   (steady steer angle)
      t_settle [s]      (time to reach steady state)
      dt
    """
    v_target = params["v_target"]
    steer_rad = math.radians(params["steer_deg"])
    t_settle = params.get("t_settle", 8.0)
    dt = params.get("dt", 0.005)
    mu = params.get("mu", 1.0)

    s0 = vdsim.State()
    s0.velocity = [v_target, 0.0, 0.0]
    r_w = 0.33; w = v_target / r_w
    s0.wheel_spin = [w, w, w, w]
    dyn.reset(s0)

    contacts = _flat_contacts(mu)
    cmd = vdsim.CmdL4()
    cmd.steer_angle_wheel = steer_rad

    n = int(t_settle / dt)
    traj = {k: [] for k in ("t", "vx", "vy", "r", "ay", "steer")}
    t = 0.0
    for i in range(n):
        dyn.step(cmd, contacts, dt)
        st = dyn.state()
        traj["t"].append(t); traj["vx"].append(st.vx()); traj["vy"].append(st.vy())
        traj["r"].append(st.yaw_rate()); traj["ay"].append(dyn.ay_body_est())
        traj["steer"].append(steer_rad)
        t += dt
    return {k: np.array(v) for k, v in traj.items()}


def slow_ramp_steer(dyn, params):
    """ISO 4138-style: ramp steer at fixed v.  Used to characterize
    understeer gradient over a range of lateral g."""
    v_target = params["v_target"]
    steer_max_deg = params["steer_max_deg"]
    ramp_time = params.get("ramp_time", 8.0)
    dt = params.get("dt", 0.005)
    mu = params.get("mu", 1.0)

    s0 = vdsim.State()
    s0.velocity = [v_target, 0.0, 0.0]
    r_w = 0.33; w = v_target / r_w
    s0.wheel_spin = [w, w, w, w]
    dyn.reset(s0)

    contacts = _flat_contacts(mu)
    cmd = vdsim.CmdL4()

    n = int(ramp_time / dt)
    traj = {k: [] for k in ("t", "vx", "vy", "r", "ay", "steer")}
    t = 0.0
    for i in range(n):
        steer_rad = math.radians(steer_max_deg) * (i / n)
        cmd.steer_angle_wheel = steer_rad
        dyn.step(cmd, contacts, dt)
        st = dyn.state()
        traj["t"].append(t); traj["vx"].append(st.vx()); traj["vy"].append(st.vy())
        traj["r"].append(st.yaw_rate()); traj["ay"].append(dyn.ay_body_est())
        traj["steer"].append(steer_rad)
        t += dt
    return {k: np.array(v) for k, v in traj.items()}


SCENARIO_REGISTRY = {
    "step_steer":            step_steer,
    "steady_state_circular": steady_state_circular,
    "slow_ramp_steer":       slow_ramp_steer,
}
