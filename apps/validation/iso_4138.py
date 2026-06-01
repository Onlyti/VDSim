"""
ISO 4138:2012 — Steady-state circular driving behaviour, constant-radius
or constant-speed methods.

This module uses the constant-speed method (clause 5.2):
    - Maintain v constant.
    - Ramp the steer angle slowly, sweeping a_y from 0 to ~6 m/s² (or limit).
    - Plot δ_road (front wheel steer) vs a_y.
    - Extract:
        K  = d(δ - δ_kin) / d(a_y)       [rad / (m/s²)] — understeer gradient
        δ_kin = L / R                     where R = v² / a_y (or v / yaw_rate)
        Linear-region characteristic, U/O/N tendency.

A positive K means understeer (vehicle "pushes wide" — more steer needed
at higher a_y than the kinematic minimum).  Sports cars typically K ≈
0.001–0.003 rad/(m/s²).  Sedans 0.002–0.008.  Heavy trucks can exceed 0.01.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "apps" / "doe"))

import vdsim
from scenarios import slow_ramp_steer


@dataclass
class ISO4138Result:
    v_target: float
    K_rad_per_g:            float
    K_rad_per_m_per_s2:     float
    handling_type:          str    # "understeer" / "neutral" / "oversteer"
    linear_range_ay_max:    float  # m/s², until linearity breaks
    ay_max_reached:         float
    samples_n:              int
    trajectory:             dict


def run_iso_4138(dyn: "vdsim.IVehicleDynamics",
                  vehicle_params: "vdsim.VehicleParams",
                  v_target_kmh: float = 80.0,
                  steer_max_deg: float = 4.0,
                  ramp_time: float = 12.0,
                  dt: float = 0.002,
                  ay_min_for_fit: float = 1.0,
                  ay_max_for_fit: float = 4.0) -> ISO4138Result:
    """Ramp steer slowly; fit K in the linear region 1 ≤ a_y ≤ 4 m/s²."""
    v_target = v_target_kmh / 3.6
    traj = slow_ramp_steer(dyn, {
        "v_target":      v_target,
        "steer_max_deg": steer_max_deg,
        "ramp_time":     ramp_time,
        "dt":            dt,
    })

    t      = traj["t"]
    steer  = traj["steer"]
    vx_arr = traj["vx"]
    r_arr  = traj["r"]
    ay_arr = traj["ay"]

    L = vehicle_params.wheelbase

    # Kinematic steer:  delta_kin = L / R = L * r / v
    delta_kin = L * r_arr / np.maximum(vx_arr, 1e-3)
    delta_minus_kin = steer - delta_kin

    # Linear fit in the chosen a_y window
    mask = (ay_arr >= ay_min_for_fit) & (ay_arr <= ay_max_for_fit)
    if mask.sum() < 5:
        K_per_ay = 0.0
    else:
        x = ay_arr[mask]; y = delta_minus_kin[mask]
        # least-squares slope through origin would be biased; use polyfit
        coeffs = np.polyfit(x, y, 1)
        K_per_ay = float(coeffs[0])

    K_per_g = K_per_ay * 9.80665

    if K_per_ay > 1e-4:        tendency = "understeer"
    elif K_per_ay < -1e-4:     tendency = "oversteer"
    else:                       tendency = "neutral"

    # Detect end of linear range: deviation from linear fit > 10% of fit
    # Compute residual of linear model over full range
    if mask.sum() >= 5:
        fit = np.polyval(coeffs, ay_arr)
        residual = np.abs(delta_minus_kin - fit)
        threshold = 0.1 * np.max(np.abs(fit)) if np.max(np.abs(fit)) > 0 else 0.01
        breach = np.where((ay_arr > ay_max_for_fit) & (residual > threshold))[0]
        linear_max = float(ay_arr[breach[0]]) if len(breach) else float(ay_arr.max())
    else:
        linear_max = 0.0

    return ISO4138Result(
        v_target=v_target,
        K_rad_per_g=K_per_g,
        K_rad_per_m_per_s2=K_per_ay,
        handling_type=tendency,
        linear_range_ay_max=linear_max,
        ay_max_reached=float(ay_arr.max()),
        samples_n=int(len(t)),
        trajectory=traj,
    )


def format_report(r: ISO4138Result) -> str:
    return (
        f"=== ISO 4138 — steady-state circular (constant speed) ===\n"
        f"  v_target              : {r.v_target:.2f} m/s  ({r.v_target * 3.6:.0f} km/h)\n"
        f"  Handling tendency     : {r.handling_type.upper()}\n"
        f"  K (per g)             : {r.K_rad_per_g * 1000:+.3f} mrad/g\n"
        f"  K (per m/s²)          : {r.K_rad_per_m_per_s2 * 1000:+.3f} mrad·(s²/m)\n"
        f"  Linear range a_y      : up to {r.linear_range_ay_max:.2f} m/s²\n"
        f"  a_y_max in test       : {r.ay_max_reached:.2f} m/s² ({r.ay_max_reached / 9.80665:.2f} g)\n"
    )


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="configs/vehicles/sports.yaml")
    ap.add_argument("--tire",    default="configs/tires/default_pacejka.yaml")
    ap.add_argument("--level",   default="L2")
    ap.add_argument("--v_kmh",   type=float, default=80.0)
    ap.add_argument("--steer_max", type=float, default=4.0)
    args = ap.parse_args()

    vp = vdsim.VehicleParams.from_yaml(str(REPO / args.vehicle))
    tp = vdsim.TireParams.from_yaml(str(REPO / args.tire))
    sp = vdsim.SolverParams()
    dyn = (vdsim.create_bicycle() if args.level == "L1"
           else vdsim.create_fourteen_dof() if args.level == "L3"
           else vdsim.create_seven_dof())
    dyn.initialize(vp, tp, sp)

    r = run_iso_4138(dyn, vp, args.v_kmh, args.steer_max)
    print(format_report(r))
