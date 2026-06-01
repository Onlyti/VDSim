"""
L3-FourteenDOF *world-z* extension — Phase-2 simulator for jumps / airborne.

Adds the four things the current C++ Ld3-FourteenDOF lacks (see T23 report):
    1. World z integration  (sprung CG world z, 4 unsprung world z each)
    2. Real gravity         (m_s, m_u all feel -g; no g_eff trick)
    3. Ground profile input (ground_z(x), so the tire spring sees a ramp)
    4. Airborne branch      (F_tire = 0 when tire above ground; no negative force)

Planar motion (x, vx) kept simple: drag + rolling resistance only. Longitudinal
dynamics is irrelevant to the jump result we want to demonstrate — what matters
is the vertical / rotational response of the 7-DOF sprung+unsprung subsystem.

Outputs:
    CSV: t, x, vx, z_s, vz_s, pitch, pitch_rate, z_u_FL/FR/RL/RR, Fz_FL/.../RR
    Plus a snapshot at each timestep for the animator.

Units: SI. Frame: ISO 8855 RH (+x forward, +z up). Pitch follows the same
convention as VDSim's L3 14-DOF — *negative* pitch = nose up (front corner
lifts when pitch < 0 because z_corner = z_s − rx · sin(pitch)).
"""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import yaml


# -----------------------------------------------------------------------------
# Vehicle parameters loader
# -----------------------------------------------------------------------------
@dataclass
class VParams:
    m_total: float
    m_sprung: float
    m_unsprung: list       # per corner, len 4
    I_yy: float            # pitch inertia
    I_xx: float            # roll inertia (not used in straight-line jump)
    a: float               # cg to front [m]
    b: float               # cg to rear  [m]
    tw_f: float            # front half-track
    tw_r: float            # rear half-track
    h_cg: float            # cg height above ground (static)
    r_wheel: float         # nominal tire radius
    k_spring: list         # spring stiffness per corner [N/m]
    c_damper: list         # damper coefficient per corner [N s/m]
    k_tire: float          # tire vertical stiffness [N/m]
    Cd: float
    A_front: float
    f_roll: float          # rolling resistance


def load_params(vehicle_yaml: str, tire_yaml: str) -> VParams:
    with open(vehicle_yaml) as f:
        vp = yaml.safe_load(f)
    with open(tire_yaml) as f:
        tp = yaml.safe_load(f)
    m_total = vp["mass"]; m_sprung = vp["mass_sprung"]
    m_u_arr = vp["unsprung_mass"]
    inertia = vp["inertia_diag"]      # [Ixx, Iyy, Izz]
    return VParams(
        m_total=m_total, m_sprung=m_sprung, m_unsprung=list(m_u_arr),
        I_xx=float(inertia[0]), I_yy=float(inertia[1]),
        a=vp["cg_to_front"], b=vp["cg_to_rear"],
        tw_f=vp["track_front"] / 2.0, tw_r=vp["track_rear"] / 2.0,
        h_cg=vp["cg_height"], r_wheel=vp["wheel_radius_nominal"],
        k_spring=list(vp["spring_stiffness"]),
        c_damper=list(vp["damper_coefficient"]),
        k_tire=220_000.0,    # default in core/include/vdsim/params.hpp
        Cd=vp.get("aero_drag_coeff", 0.34),
        A_front=vp.get("frontal_area", 2.05),
        f_roll=0.015,
    )


# -----------------------------------------------------------------------------
# Ramp ground profile  ground_z(x)
# -----------------------------------------------------------------------------
def ramp_profile(x_start: float = 20.0, x_top: float = 25.0,
                 ramp_height: float = 1.2,
                 lip_length: float = 0.4):
    """A take-off ramp:  flat  →  smooth ramp up  →  short lip  →  cliff drop.
       After the lip the ground falls back to 0 immediately (cliff at x_top+lip).
    """
    x_lip_end = x_top + lip_length

    def ground_z(x: float) -> float:
        if x < x_start:
            return 0.0
        if x < x_top:
            # smooth half-cosine from 0 to ramp_height
            u = (x - x_start) / (x_top - x_start)
            return ramp_height * 0.5 * (1.0 - math.cos(math.pi * u))
        if x < x_lip_end:
            return ramp_height
        return 0.0     # cliff drop after lip

    return ground_z, dict(x_start=x_start, x_top=x_top,
                          x_lip_end=x_lip_end, ramp_height=ramp_height)


# -----------------------------------------------------------------------------
# State vector  (12 vertical DOFs + planar x, vx = 14 total)
# -----------------------------------------------------------------------------
#   y = [x, vx,
#        z_s, vz_s,            sprung CG world z + vz
#        pitch, pitch_rate,    body pitch (rad)
#        z_u_FL, z_u_FR, z_u_RL, z_u_RR,         unsprung world z (4)
#        vz_u_FL, vz_u_FR, vz_u_RL, vz_u_RR]    unsprung vz       (4)
# (Roll, vy, yaw all = 0 in straight-line jump.  Could be added later.)
G = 9.80665


def derivatives(y, p: VParams, ground_z, throttle: float):
    (x, vx,
     z_s, vz_s, pitch, pitch_rate,
     z_uFL, z_uFR, z_uRL, z_uRR,
     vz_uFL, vz_uFR, vz_uRL, vz_uRR) = y

    # Corner offsets (body frame, ISO 8855)
    rx = [+p.a, +p.a, -p.b, -p.b]
    ry = [+p.tw_f, -p.tw_f, +p.tw_r, -p.tw_r]
    z_u  = [z_uFL,  z_uFR,  z_uRL,  z_uRR]
    vz_u = [vz_uFL, vz_uFR, vz_uRL, vz_uRR]

    cp, sp = math.cos(pitch), math.sin(pitch)

    # Sprung corner positions (small-angle pitch only — roll = 0):
    #   world x_corner_i ≈ x + rx_i * cp + (z_corner_rel) * sp ≈ x + rx_i (small pitch)
    #   world z_corner_i = z_s + rx_i * (-sin(pitch))  (nose-up pitch lifts front)
    #     here pitch+ = nose up, so front corner z is raised by -rx * sin(pitch) * ...
    #     Actually for ISO 8855 with x+ forward and pitch+ = nose-up: front rises.
    #     z_corner = z_s + (-rx_i) * sin(pitch)  -- when rx_i positive (front), corner
    #     rises with positive pitch. So coefficient is -rx_i.
    z_s_corner = [z_s - rx[i] * sp for i in range(4)]
    vz_s_corner = [vz_s - rx[i] * cp * pitch_rate for i in range(4)]

    # Suspension free length L0 chosen so that static equilibrium under gravity
    # sits at z_s = h_cg and z_u_i at static tire compression.  We use the trick:
    #   F_spring_i = k_i * (delta_i_static - delta_i)
    # where delta_static carries the static load.  Effectively we work in
    # PERTURBATION from the spring force needed to hold the corner up.
    F_corner_static = [p.m_sprung * G * (p.b if rx[i] > 0 else p.a)
                       / (p.a + p.b) / 2.0  for i in range(4)]
    delta_static    = [F_corner_static[i] / p.k_spring[i] for i in range(4)]

    # Spring perturbation force (linear), then clip with droop / bump stops:
    #   F_spring_total ∈ [0, F_bump_factor · F_static] per corner
    # Floor 0 = droop limit (wheel "topped out"; can't pull sprung down).
    # Ceiling = bump stop (suspension bottoming; we use 3× static).
    delta = [(z_s_corner[i] - z_u[i]) for i in range(4)]
    F_spring = []
    for i in range(4):
        F_lin = p.k_spring[i] * (delta_static[i] - delta[i]) + F_corner_static[i]
        # droop / bump clipping
        F_lin = max(0.0, min(3.0 * F_corner_static[i], F_lin))
        F_spring.append(F_lin)

    # Damper: only acts while spring is engaged (not at droop limit).
    # Crude — real suspensions have rebound vs. compression asymmetry.
    v_rel = [vz_s_corner[i] - vz_u[i] for i in range(4)]
    F_damper = []
    for i in range(4):
        if F_spring[i] < 1e-3:           # topped out — wheel hanging
            F_damper.append(0.0)
        else:
            F_damper.append(-p.c_damper[i] * v_rel[i])

    F_susp = [F_spring[i] + F_damper[i] for i in range(4)]

    # Tire vertical force — unsprung mass touches ground via tire spring
    # Compression epsilon = (ground_z + r_wheel) - z_u  (positive when compressed)
    # F_tire = k_tire * epsilon ,  no pull (max with 0)
    z_g_corner = [ground_z(x + rx[i]) for i in range(4)]
    eps = [(z_g_corner[i] + p.r_wheel) - z_u[i] for i in range(4)]
    F_tire = [max(0.0, p.k_tire * eps[i]) for i in range(4)]

    # Sprung mass dynamics
    sum_F_susp = sum(F_susp)
    az_s = sum_F_susp / p.m_sprung - G

    # Pitch dynamics — moment about sprung CG, body y-axis
    # Positive pitch (nose up) is produced by a positive moment when the
    # FRONT corners push UP (positive F_susp) less than the rear.  In body
    # coords:  τ_y = Σ ( -rx_i ) * F_susp_i  →  dpitch_rate/dt = τ/I_yy
    tau_y = sum(-rx[i] * F_susp[i] for i in range(4))
    apitch = tau_y / p.I_yy

    # Per-corner unsprung mass dynamics
    az_u = [(-F_susp[i] + F_tire[i]) / p.m_unsprung[i] - G for i in range(4)]

    # Longitudinal dynamics — minimal model
    # Tractive force = throttle * F_max_per_wheel; aero drag; rolling res
    rho = 1.2
    F_drag = 0.5 * rho * p.Cd * p.A_front * vx * vx
    F_roll = p.f_roll * sum(F_tire)
    F_trac = throttle * 6000.0      # ~ rough thrust at small throttle
    # Lose tractive force when no rear contact (RWD style, sports.yaml)
    rear_on_ground = (F_tire[2] + F_tire[3]) > 1.0
    if not rear_on_ground:
        F_trac = 0.0
    ax = (F_trac - F_drag - F_roll) / p.m_total

    return [
        vx, ax,
        vz_s, az_s,
        pitch_rate, apitch,
        vz_u[0], vz_u[1], vz_u[2], vz_u[3],
        az_u[0], az_u[1], az_u[2], az_u[3],
    ]


def rk4(y, h, p, gz, throttle):
    k1 = derivatives(y, p, gz, throttle)
    y2 = [y[i] + 0.5 * h * k1[i] for i in range(len(y))]
    k2 = derivatives(y2, p, gz, throttle)
    y3 = [y[i] + 0.5 * h * k2[i] for i in range(len(y))]
    k3 = derivatives(y3, p, gz, throttle)
    y4 = [y[i] + h * k3[i] for i in range(len(y))]
    k4 = derivatives(y4, p, gz, throttle)
    return [y[i] + h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0
            for i in range(len(y))]


# -----------------------------------------------------------------------------
# Run
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="configs/vehicles/sports.yaml")
    ap.add_argument("--tire", default="configs/tires/default_pacejka.yaml")
    ap.add_argument("--out", default="apps/jump_demo/run01")
    ap.add_argument("--v0", type=float, default=18.0,    # ~65 km/h
                    help="initial speed [m/s]")
    ap.add_argument("--ramp_h", type=float, default=1.2)
    ap.add_argument("--ramp_start", type=float, default=20.0)
    ap.add_argument("--ramp_top", type=float, default=25.0)
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--dt", type=float, default=0.001)
    args = ap.parse_args()

    p = load_params(args.vehicle, args.tire)
    gz, ramp_info = ramp_profile(args.ramp_start, args.ramp_top, args.ramp_h)

    # Static equilibrium initial state
    z_u0 = [p.r_wheel - ((p.m_sprung / 4 + p.m_unsprung[i]) * G / p.k_tire)
            for i in range(4)]
    # delta_static (set so F_spring at static eq exactly = corner load)
    # delta_static_i = F_corner_static / k_i ; static z_s_corner = z_u + delta_static
    delta_st = [
        ((p.m_sprung * G * (p.b if rx > 0 else p.a) / (p.a + p.b) / 2.0)
         / p.k_spring[i])
        for i, rx in enumerate([+p.a, +p.a, -p.b, -p.b])
    ]
    z_s0 = sum(z_u0[i] + delta_st[i] for i in range(4)) / 4.0

    y = [
        0.0, args.v0,         # x, vx
        z_s0, 0.0, 0.0, 0.0,  # z_s, vz_s, pitch, pitch_rate
        z_u0[0], z_u0[1], z_u0[2], z_u0[3],
        0.0, 0.0, 0.0, 0.0,
    ]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    n = int(args.duration / args.dt)
    log_every = int(0.01 / args.dt)   # log at 100 Hz
    for step in range(n):
        t = step * args.dt
        throttle = 0.1   # light coast
        y = rk4(y, args.dt, p, gz, throttle)

        if step % log_every == 0:
            x = y[0]; vx = y[1]; z_s = y[2]; vz_s = y[3]
            pitch = y[4]; pr = y[5]
            z_u = y[6:10]; vz_u = y[10:14]
            # F_tire snapshot
            rx_corners = [+p.a, +p.a, -p.b, -p.b]
            Fz = []
            for i in range(4):
                z_g = gz(x + rx_corners[i])
                eps = (z_g + p.r_wheel) - z_u[i]
                Fz.append(max(0.0, p.k_tire * eps))
            rows.append({
                "t": round(t, 3),
                "x": x, "vx": vx,
                "z_s": z_s, "vz_s": vz_s,
                "pitch": pitch, "pitch_rate": pr,
                "z_u_FL": z_u[0], "z_u_FR": z_u[1],
                "z_u_RL": z_u[2], "z_u_RR": z_u[3],
                "Fz_FL": Fz[0], "Fz_FR": Fz[1],
                "Fz_RL": Fz[2], "Fz_RR": Fz[3],
                "ground_FL": gz(x + p.a),
                "ground_RL": gz(x - p.b),
                "airborne": all(f < 1.0 for f in Fz),
            })

    csv_path = out_dir / "telemetry.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    # Save ramp metadata too
    with open(out_dir / "ramp.yaml", "w") as f:
        yaml.safe_dump(ramp_info, f)

    # Save vehicle params snapshot for renderer
    with open(out_dir / "veh.yaml", "w") as f:
        yaml.safe_dump({
            "wheelbase": p.a + p.b, "a": p.a, "b": p.b,
            "r_wheel": p.r_wheel, "h_cg": p.h_cg,
            "track_front": 2 * p.tw_f, "track_rear": 2 * p.tw_r,
        }, f)

    # Summary
    airborne = [r["airborne"] for r in rows]
    if any(airborne):
        i_air_start = airborne.index(True)
        i_air_end = len(airborne) - 1 - list(reversed(airborne)).index(True)
        t_air = rows[i_air_end]["t"] - rows[i_air_start]["t"]
        z_s_peak = max(r["z_s"] for r in rows)
        pitch_peak_deg = max(abs(r["pitch"]) for r in rows) * 180 / math.pi
    else:
        t_air, z_s_peak, pitch_peak_deg = 0.0, max(r["z_s"] for r in rows), 0
    print(f"[jump-sim] samples={len(rows)} -> {csv_path}")
    print(f"[jump-sim] airborne duration ≈ {t_air:.3f} s")
    print(f"[jump-sim] sprung CG peak z = {z_s_peak:.3f} m")
    print(f"[jump-sim] peak |pitch| = {pitch_peak_deg:.1f} deg")


if __name__ == "__main__":
    main()
