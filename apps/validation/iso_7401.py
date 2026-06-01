"""
ISO 7401:2003 — Lateral transient response test (step-input).

The standard specifies how to measure the *transient* yaw response of a
vehicle to a sudden steering input.  Reported quantities (per § 7.2):

    psi_dot_ss   : steady-state yaw rate
    T_psi_dot    : yaw rate response time (time from step to 0.9 · psi_dot_ss)
    T_max        : time to peak yaw rate
    U_psi_dot    : peak/SS overshoot ratio  =  psi_dot_peak / psi_dot_ss
    psi_dot_peak : peak yaw rate
    TB           : tangential body slip angle response time (skipped — req. β)

This module:
    - Runs the standard step-input scenario through a vdsim dynamics instance.
    - Extracts the ISO metrics.
    - Optionally plots the response.

Test condition (default, configurable):
    v_target = 80 km/h ≈ 22.22 m/s
    steer step amplitude chosen to give |a_y_ss| ≈ 4 m/s² (sport / passenger)
    step rise time < 0.15 s (we use a near-instant step)
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "apps" / "doe"))

import vdsim
from scenarios import step_steer


@dataclass
class ISO7401Result:
    v_target: float
    steer_deg: float
    psi_dot_ss: float        # rad/s
    psi_dot_peak: float
    U: float                 # overshoot ratio
    T_max: float             # s (from step input)
    T_psi_dot: float         # s (from step to 90% of SS)
    settling_time_5pct: float
    a_y_ss: float            # m/s²
    trajectory: dict


def run_iso_7401(dyn: "vdsim.IVehicleDynamics",
                  vehicle_params: "vdsim.VehicleParams",
                  v_target_kmh: float = 80.0,
                  steer_deg: float = 40.0,
                  t_pre: float = 0.5,
                  t_post: float = 5.0,
                  dt: float = 0.002) -> ISO7401Result:
    v_target = v_target_kmh / 3.6
    traj = step_steer(dyn, {
        "v_target":  v_target,
        "steer_deg": steer_deg,
        "t_pre":     t_pre,
        "t_post":    t_post,
        "dt":        dt,
    })

    t = traj["t"]; r = traj["r"]; ay = traj["ay"]
    # Step occurs at t = t_pre.  Define indices.
    step_idx = int(t_pre / dt)
    t_after = t[step_idx:] - t_pre
    r_after = r[step_idx:]
    ay_after = ay[step_idx:]

    # Steady-state estimate: mean over the last 20%
    n_tail = max(1, len(r_after) // 5)
    psi_dot_ss = float(np.mean(r_after[-n_tail:]))
    a_y_ss     = float(np.mean(ay_after[-n_tail:]))

    # Peak (signed by SS direction)
    sign = 1.0 if psi_dot_ss >= 0 else -1.0
    r_signed = r_after * sign
    peak_idx = int(np.argmax(r_signed))
    psi_dot_peak = float(r_signed[peak_idx] * sign)
    T_max        = float(t_after[peak_idx])
    U            = (psi_dot_peak / psi_dot_ss) if abs(psi_dot_ss) > 1e-6 else 0.0

    # T_psi_dot: time from step to first crossing 0.9 * |psi_dot_ss| (sign-aware)
    target_90 = 0.9 * psi_dot_ss
    if abs(psi_dot_ss) > 1e-6:
        idx_above = np.where(r_after * sign >= target_90 * sign)[0]
        T_psi_dot = float(t_after[idx_above[0]]) if len(idx_above) else float("nan")
    else:
        T_psi_dot = float("nan")

    # 5% settling time (last time outside ±5% band)
    band = 0.05 * abs(psi_dot_ss)
    lo, hi = psi_dot_ss - band, psi_dot_ss + band
    if psi_dot_ss < 0: lo, hi = hi, lo
    outside = (r_after < min(lo, hi)) | (r_after > max(lo, hi))
    if outside.any():
        idx_last = int(np.where(outside)[0][-1]) + 1
        settling = float(t_after[idx_last]) if idx_last < len(t_after) else float(t_after[-1])
    else:
        settling = 0.0

    return ISO7401Result(
        v_target=v_target, steer_deg=steer_deg,
        psi_dot_ss=psi_dot_ss, psi_dot_peak=psi_dot_peak,
        U=U, T_max=T_max, T_psi_dot=T_psi_dot,
        settling_time_5pct=settling, a_y_ss=a_y_ss,
        trajectory=traj,
    )


def format_report(r: ISO7401Result) -> str:
    return (
        f"=== ISO 7401 — step-steer transient response ===\n"
        f"  v_target           : {r.v_target:.2f} m/s  ({r.v_target * 3.6:.0f} km/h)\n"
        f"  steer input        : {r.steer_deg:+.1f} deg (step)\n"
        f"\n"
        f"  psi_dot_ss         : {r.psi_dot_ss * 57.3:+.3f}  deg/s  ({r.psi_dot_ss:+.4f} rad/s)\n"
        f"  psi_dot_peak       : {r.psi_dot_peak * 57.3:+.3f}  deg/s\n"
        f"  U  (peak / SS)     : {r.U:.3f}    -> {(r.U - 1) * 100:+.1f}% overshoot\n"
        f"  T_max  (to peak)   : {r.T_max:.3f}  s\n"
        f"  T_psi_dot (90% SS) : {r.T_psi_dot:.3f}  s\n"
        f"  Settling 5%        : {r.settling_time_5pct:.3f}  s\n"
        f"  a_y_ss             : {r.a_y_ss:.3f}  m/s²  ({r.a_y_ss / 9.80665:.2f} g)\n"
    )


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="configs/vehicles/sports.yaml")
    ap.add_argument("--tire",    default="configs/tires/default_pacejka.yaml")
    ap.add_argument("--level",   default="L2")
    ap.add_argument("--v_kmh",   type=float, default=80.0)
    ap.add_argument("--steer",   type=float, default=40.0)
    args = ap.parse_args()

    vp = vdsim.VehicleParams.from_yaml(str(REPO / args.vehicle))
    tp = vdsim.TireParams.from_yaml(str(REPO / args.tire))
    sp = vdsim.SolverParams()
    dyn = (vdsim.create_bicycle() if args.level == "L1"
           else vdsim.create_fourteen_dof() if args.level == "L3"
           else vdsim.create_seven_dof())
    dyn.initialize(vp, tp, sp)

    r = run_iso_7401(dyn, vp, args.v_kmh, args.steer)
    print(format_report(r))
