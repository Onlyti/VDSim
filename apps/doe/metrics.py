"""Metric extractors operating on a trajectory dict (see scenarios.py)."""
from __future__ import annotations

import math
import numpy as np


def peak_yaw_rate(traj, params=None):
    """Peak |yaw_rate| over the trajectory [rad/s]."""
    return float(np.max(np.abs(traj["r"])))


def steady_state_yaw_rate(traj, params=None, tail_frac=0.2):
    """Mean yaw rate over the last `tail_frac` of the trajectory."""
    n = len(traj["r"]); tail = max(1, int(n * tail_frac))
    return float(np.mean(traj["r"][-tail:]))


def yaw_overshoot(traj, params=None):
    """Peak / steady-state ratio of yaw rate.  >1.0 indicates overshoot
    (e.g., 1.10 = 10% overshoot)."""
    ss = steady_state_yaw_rate(traj, params)
    if abs(ss) < 1e-6: return 0.0
    peak = float(np.max(traj["r"] * np.sign(ss)))   # signed peak
    return peak / ss


def yaw_settling_time(traj, params=None, band=0.05):
    """Time after step input for yaw rate to enter the ±`band` band around
    steady state and stay there.  Returns seconds from step (assuming step
    occurs at t_pre — but here we simply use the time of step from params).
    """
    t = traj["t"]; r = traj["r"]
    ss = steady_state_yaw_rate(traj, params)
    if abs(ss) < 1e-6: return float(t[-1] - t[0])
    lo, hi = ss * (1 - band), ss * (1 + band)
    if ss < 0: lo, hi = hi, lo
    # find LAST index where r is outside band
    inside = (r >= min(lo, hi)) & (r <= max(lo, hi))
    if not inside.any():
        return float(t[-1] - t[0])
    last_outside = np.where(~inside)[0]
    if len(last_outside) == 0: return 0.0
    settle_idx = last_outside[-1] + 1
    if settle_idx >= len(t): return float(t[-1] - t[0])
    # subtract step time (params.t_pre) if available
    t_step = (params or {}).get("t_pre", 0.0)
    return float(t[settle_idx] - t_step)


def peak_lateral_g(traj, params=None):
    """Peak |a_y| in g."""
    return float(np.max(np.abs(traj["ay"])) / 9.80665)


def steady_state_ay_per_steer(traj, params=None):
    """Lateral g per radian of steer at steady state — a sensitivity metric."""
    ss_ay = float(np.mean(traj["ay"][-len(traj["ay"])//5:]))
    steer = params.get("steer_deg", 0.0)
    if abs(steer) < 1e-3: return 0.0
    return ss_ay / math.radians(steer)


def understeer_gradient_K(traj, params, vehicle_params=None):
    """K [rad/g] understeer gradient: K = δ - L/R, where L = wheelbase,
    R = v/yaw_rate.  Computed in steady state.

    vehicle_params: pybind vdsim.VehicleParams instance (for L).
    """
    if vehicle_params is None: return 0.0
    L = vehicle_params.wheelbase
    # Steady-state averages
    tail = max(1, len(traj["t"]) // 5)
    delta = float(np.mean(traj["steer"][-tail:]))
    vx    = float(np.mean(traj["vx"][-tail:]))
    r     = float(np.mean(traj["r"][-tail:]))
    if abs(r) < 1e-4 or abs(vx) < 0.5: return 0.0
    R_path = vx / r
    ay_g   = (vx * r) / 9.80665
    if abs(ay_g) < 0.01: return 0.0
    delta_kin = L / R_path
    K = (delta - delta_kin) / ay_g
    return K   # rad/g


METRIC_REGISTRY = {
    "peak_yaw_rate":               peak_yaw_rate,
    "steady_state_yaw_rate":       steady_state_yaw_rate,
    "yaw_overshoot":               yaw_overshoot,
    "yaw_settling_time":           yaw_settling_time,
    "peak_lateral_g":              peak_lateral_g,
    "steady_state_ay_per_steer":   steady_state_ay_per_steer,
    "understeer_gradient_K":       understeer_gradient_K,
}
