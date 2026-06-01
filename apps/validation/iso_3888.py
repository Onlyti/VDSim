"""
ISO 3888-2:2011 — Severe lane-change manoeuvre ("moose test").

The vehicle enters a cone course at fixed entry speed, must navigate
through an offset section and return to the entry lane.  ISO defines
the cone course geometry; we generate the lane center-line waypoints
and follow them with a Pure Pursuit controller.

Cone course (passenger car spec, ISO 3888-2:2011, Annex A):
    Section A (entry):     12.0 m   straight in lane 0 (y=0)
    Section B (transition): 13.5 m  to lane 1 (y = +3.5 m offset)
    Section C (offset):    11.0 m   straight in lane 1
    Section D (return):    12.5 m   back to lane 0
    Section E (exit):       2.0 m + straight
Pass criteria (per § 8): speed loss < 2 km/h; no cone hits (lateral
deviation within prescribed gate widths).

This module measures:
    speed_loss_kmh
    max_lateral_excursion
    peak_yaw_rate
    peak_ay
    success (within gate width)
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))

import vdsim


# ---------- Cone course geometry ------------------------------------------
def dlc_waypoints(x_start: float = 0.0, dx: float = 0.5) -> np.ndarray:
    """Return densely-sampled (x, y) center-line waypoints through the DLC.
    Transition sections use a smooth raised-cosine S-curve rather than a
    sharp linear ramp — this is the path a competent driver would take and
    is what's needed for a Pure Pursuit follower to behave nicely."""
    # Section lengths
    L_A, L_B, L_C, L_D, L_E = 12.0, 13.5, 11.0, 12.5, 12.0
    OFF = 3.5

    pts = []
    x0 = x_start
    # A — straight at y=0
    for x in np.arange(0.0, L_A, dx):
        pts.append((x0 + x, 0.0))
    # B — raised-cosine transition 0 → +OFF
    for x in np.arange(0.0, L_B, dx):
        u = x / L_B
        y = OFF * 0.5 * (1.0 - math.cos(math.pi * u))
        pts.append((x0 + L_A + x, y))
    # C — hold at +OFF
    for x in np.arange(0.0, L_C, dx):
        pts.append((x0 + L_A + L_B + x, OFF))
    # D — raised-cosine transition +OFF → 0
    for x in np.arange(0.0, L_D, dx):
        u = x / L_D
        y = OFF * 0.5 * (1.0 + math.cos(math.pi * u))
        pts.append((x0 + L_A + L_B + L_C + x, y))
    # E — exit at y=0
    for x in np.arange(0.0, L_E, dx):
        pts.append((x0 + L_A + L_B + L_C + L_D + x, 0.0))
    return np.array(pts)


def pure_pursuit_steer(x, y, yaw, vx, waypoints, wheelbase, prev_idx=0):
    """Pure-pursuit lateral controller — find lookahead waypoint, compute
    curvature, return road-wheel steer angle."""
    L_d = max(3.0, 0.5 * vx)    # lookahead distance scales with speed
    idx = prev_idx
    while idx < len(waypoints) - 1:
        dx = waypoints[idx][0] - x
        dy = waypoints[idx][1] - y
        if math.hypot(dx, dy) >= L_d: break
        idx += 1
    if idx >= len(waypoints): idx = len(waypoints) - 1
    # Transform target into body frame
    cp, sp = math.cos(yaw), math.sin(yaw)
    dx_b =  cp * (waypoints[idx][0] - x) + sp * (waypoints[idx][1] - y)
    dy_b = -sp * (waypoints[idx][0] - x) + cp * (waypoints[idx][1] - y)
    L2 = dx_b * dx_b + dy_b * dy_b
    if L2 < 1e-6: return 0.0, idx
    kappa = 2.0 * dy_b / L2
    steer = math.atan(kappa * wheelbase)
    return max(-0.45, min(0.45, steer)), idx


# ---------- Result struct -------------------------------------------------
@dataclass
class ISO3888Result:
    v_entry_kmh:       float
    v_exit_kmh:        float
    speed_loss_kmh:    float
    max_lateral_excursion: float
    peak_yaw_rate_dps: float
    peak_ay_g:         float
    success:           bool
    trajectory:        dict


def _flat_contacts():
    contacts = [vdsim.ContactPoint() for _ in range(4)]
    for c in contacts:
        c.is_valid = True; c.normal = [0, 0, 1]; c.mu_long = 1.0; c.mu_lat = 1.0
    return contacts


def run_iso_3888(dyn: "vdsim.IVehicleDynamics",
                  vehicle_params: "vdsim.VehicleParams",
                  v_entry_kmh: float = 60.0,
                  dt: float = 0.005,
                  duration: float = 10.0) -> ISO3888Result:
    v_entry = v_entry_kmh / 3.6
    waypoints = dlc_waypoints()

    # Init at start of section A, aligned with +x.
    s0 = vdsim.State()
    s0.velocity = [v_entry, 0.0, 0.0]
    r_w = vehicle_params.wheel_radius_nominal
    w = v_entry / r_w
    s0.wheel_spin = [w, w, w, w]
    dyn.reset(s0)

    contacts = _flat_contacts()
    n = int(duration / dt)
    traj = {k: [] for k in ("t", "x", "y", "yaw", "vx", "r", "ay", "steer")}
    prev_idx = 0
    t = 0.0
    for _ in range(n):
        st = dyn.state()
        steer, prev_idx = pure_pursuit_steer(
            st.position[0], st.position[1], st.yaw(), st.vx(),
            waypoints, vehicle_params.wheelbase, prev_idx)
        # Simple proportional speed control: keep entry speed.
        e_v = v_entry - st.vx()
        throttle = max(0.0, min(1.0, 0.15 + 0.40 * e_v))
        brake = 0.0 if e_v > -0.5 else 0.2
        cmd = vdsim.CmdL4()
        cmd.steer_angle_wheel = steer
        cmd.throttle = throttle
        cmd.brake = brake
        u = cmd
        dyn.step(u, contacts, dt)

        st = dyn.state()
        traj["t"].append(t)
        traj["x"].append(st.position[0]); traj["y"].append(st.position[1])
        traj["yaw"].append(st.yaw()); traj["vx"].append(st.vx())
        traj["r"].append(st.yaw_rate()); traj["ay"].append(dyn.ay_body_est())
        traj["steer"].append(steer)
        t += dt
        # Exit when past last waypoint
        if st.position[0] > waypoints[-1][0]: break

    for k in traj: traj[k] = np.array(traj[k])

    v_exit_kmh = traj["vx"][-1] * 3.6
    speed_loss = v_entry_kmh - v_exit_kmh
    # Lateral excursion: max |y - y_lane_target(x)| along the path
    x_arr = traj["x"]; y_arr = traj["y"]
    # Reference y at each x: interpolate along section boundaries
    y_ref = np.interp(x_arr, waypoints[:, 0], waypoints[:, 1])
    excursion = float(np.max(np.abs(y_arr - y_ref)))
    peak_r_dps = float(np.max(np.abs(traj["r"])) * 180 / math.pi)
    peak_ay_g  = float(np.max(np.abs(traj["ay"])) / 9.80665)

    # Approximate success: speed loss < 2 km/h AND excursion < 1.0 m
    success = (speed_loss < 2.0) and (excursion < 1.0)

    return ISO3888Result(
        v_entry_kmh=v_entry_kmh, v_exit_kmh=v_exit_kmh,
        speed_loss_kmh=speed_loss, max_lateral_excursion=excursion,
        peak_yaw_rate_dps=peak_r_dps, peak_ay_g=peak_ay_g,
        success=success, trajectory=traj)


def format_report(r: ISO3888Result) -> str:
    return (
        f"=== ISO 3888-2 — Severe lane-change (DLC / moose test) ===\n"
        f"  v_entry            : {r.v_entry_kmh:.1f} km/h\n"
        f"  v_exit             : {r.v_exit_kmh:.1f} km/h\n"
        f"  Speed loss         : {r.speed_loss_kmh:+.2f} km/h\n"
        f"  Max lateral excursion (vs target lane): {r.max_lateral_excursion:.3f} m\n"
        f"  Peak yaw rate      : {r.peak_yaw_rate_dps:.1f} °/s\n"
        f"  Peak |a_y|         : {r.peak_ay_g:.3f} g\n"
        f"  Verdict            : {'PASS' if r.success else 'FAIL'}\n"
        f"                       (criteria: speed loss < 2.0 km/h AND\n"
        f"                        excursion < 1.0 m vs target lane)\n"
    )


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="configs/vehicles/sports.yaml")
    ap.add_argument("--tire",    default="configs/tires/default_pacejka.yaml")
    ap.add_argument("--level",   default="L2")
    ap.add_argument("--v_kmh",   type=float, default=60.0)
    args = ap.parse_args()

    vp = vdsim.VehicleParams.from_yaml(str(REPO / args.vehicle))
    tp = vdsim.TireParams.from_yaml(str(REPO / args.tire))
    sp = vdsim.SolverParams()
    dyn = (vdsim.create_bicycle() if args.level == "L1"
           else vdsim.create_fourteen_dof() if args.level == "L3"
           else vdsim.create_seven_dof())
    dyn.initialize(vp, tp, sp)

    r = run_iso_3888(dyn, vp, args.v_kmh)
    print(format_report(r))
