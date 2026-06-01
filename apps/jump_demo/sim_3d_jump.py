"""
T24 — 3D turning jump simulation.

Extends the T23 14-DOF world-z model with:
    - planar lateral dynamics:  vy, yaw, yaw_rate
    - linear-tire lateral force (combined-slip ignored; cornering stiffness)
    - body roll integrated like pitch (sprung mass moment about body x-axis)
    - steering schedule + ramp aligned with +x

Scenario:  vehicle approaches the ramp going +x, builds a left-turn yaw rate
just before take-off, becomes airborne with non-zero yaw rate, continues to
rotate in the air, lands while still yawing.

Frame: ISO 8855 RH (+x fwd, +y left, +z up).
"""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import yaml


G = 9.80665


# -----------------------------------------------------------------------------
# Vehicle params (re-use T23 + add yaw inertia and cornering stiffness)
# -----------------------------------------------------------------------------
@dataclass
class TireMF96:
    """Simplified Pacejka MF96 + load-sensitive μ.  Transient slip (relaxation
    length) is handled as a state in the host integrator, NOT inside compute().
    """
    B_long: float; C_long: float; D_long: float; E_long: float
    B_lat:  float; C_lat:  float; D_lat:  float; E_lat:  float
    mu_nominal: float
    Fz_nominal: float
    load_sensitivity: float        # μ_eff = μ · (1 − ls · (Fz/Fz_nom − 1))
    relaxation_length_lat: float   # σ_y [m]
    relaxation_length_long: float  # σ_x [m]
    combined_slip: bool = True

    @staticmethod
    def _form(B, C, D, E, s):
        t = B * s
        phi = t - E * (t - math.atan(t))
        return D * math.sin(C * math.atan(phi))

    def mu_eff(self, Fz: float) -> float:
        if Fz < 1.0: return self.mu_nominal
        dfz = Fz / max(1.0, self.Fz_nominal) - 1.0
        return max(0.3, self.mu_nominal * (1.0 - self.load_sensitivity * dfz))

    def compute(self, Fz, kappa, alpha, mu_long=1.0, mu_lat=1.0):
        Fz = max(0.0, Fz)
        if Fz < 1.0:
            return 0.0, 0.0
        mu_e   = self.mu_eff(Fz)
        mu_x   = mu_long * mu_e
        mu_y   = mu_lat  * mu_e
        Fx_max = self.D_long * Fz * mu_x
        Fy_max = self.D_lat  * Fz * mu_y
        Fx_pure =  self._form(self.B_long, self.C_long, Fx_max, self.E_long, kappa)
        Fy_pure = -self._form(self.B_lat,  self.C_lat,  Fy_max, self.E_lat,  alpha)
        Fx, Fy = Fx_pure, Fy_pure
        if self.combined_slip and Fx_max > 0.0 and Fy_max > 0.0:
            rx = Fx_pure / Fx_max
            ry = Fy_pure / Fy_max
            r2 = rx*rx + ry*ry
            if r2 > 1.0:
                s = 1.0 / math.sqrt(r2)
                Fx = Fx_pure * s
                Fy = Fy_pure * s
        return Fx, Fy


@dataclass
class VParams:
    m_total: float; m_sprung: float
    m_unsprung: list
    I_xx: float; I_yy: float; I_zz: float
    I_wheel: float                   # per-wheel spin inertia [kg m^2]
    a: float; b: float
    tw_f: float; tw_r: float
    h_cg: float; r_wheel: float
    k_spring: list; c_damper: list
    k_tire: float
    tire: TireMF96
    Cd: float; A_front: float; f_roll: float
    # Roll/pitch axis heights — standard textbook moment arms
    h_roll_axis: float       # average of front/rear roll center height
    h_pitch_axis: float
    # Drivetrain
    drive_type: str          # 'RWD' / 'FWD' / 'AWD'
    max_motor_torque: float  # total wheel-axis torque at throttle=1 [N m]
    max_brake_torque: float  # total brake torque at brake=1 [N m]
    brake_bias_front: float


def load_params(vehicle_yaml: str, tire_yaml: str) -> VParams:
    with open(vehicle_yaml) as f: vp = yaml.safe_load(f)
    with open(tire_yaml) as f:    tp = yaml.safe_load(f)
    inertia = vp["inertia_diag"]
    h_rc_f = vp.get("roll_center_height_front", 0.0)
    h_rc_r = vp.get("roll_center_height_rear", 0.0)
    tire = TireMF96(
        B_long=tp.get("B_long", 10.0), C_long=tp.get("C_long", 1.65),
        D_long=tp.get("D_long", 1.0),  E_long=tp.get("E_long", 0.97),
        B_lat=tp.get("B_lat", 8.0),    C_lat=tp.get("C_lat", 1.30),
        D_lat=tp.get("D_lat", 1.0),    E_lat=tp.get("E_lat", -1.0),
        mu_nominal=tp.get("mu_nominal", 1.0),
        Fz_nominal=tp.get("Fz_nominal", 4000.0),
        load_sensitivity=tp.get("load_sensitivity", 0.0),
        relaxation_length_lat=tp.get("relaxation_length_lat", 0.0),
        relaxation_length_long=tp.get("relaxation_length_long", 0.0),
    )
    return VParams(
        m_total=vp["mass"], m_sprung=vp["mass_sprung"],
        m_unsprung=list(vp["unsprung_mass"]),
        I_xx=float(inertia[0]), I_yy=float(inertia[1]), I_zz=float(inertia[2]),
        I_wheel=1.0,
        a=vp["cg_to_front"], b=vp["cg_to_rear"],
        tw_f=vp["track_front"]/2.0, tw_r=vp["track_rear"]/2.0,
        h_cg=vp["cg_height"], r_wheel=vp["wheel_radius_nominal"],
        k_spring=list(vp["spring_stiffness"]),
        c_damper=list(vp["damper_coefficient"]),
        k_tire=220_000.0,
        tire=tire,
        Cd=vp.get("aero_drag_coeff", 0.34),
        A_front=vp.get("frontal_area", 2.05),
        f_roll=tp.get("rolling_resistance", 0.015),
        h_roll_axis=(vp["cg_to_rear"] * h_rc_f + vp["cg_to_front"] * h_rc_r)
                    / (vp["cg_to_front"] + vp["cg_to_rear"]),
        h_pitch_axis=vp.get("pitch_center_height", 0.0),
        drive_type=vp.get("drive_type", "RWD"),
        max_motor_torque=vp.get("max_motor_torque", 480.0),
        max_brake_torque=vp.get("max_brake_torque", 3000.0),
        brake_bias_front=vp.get("brake_bias_front", 0.65),
    )


# -----------------------------------------------------------------------------
# Ramp profile (same as T23, narrow band in y so wheels can launch sideways)
# -----------------------------------------------------------------------------
def ramp_profile(x_start=20.0, x_top=24.0, ramp_h=0.6, lip=0.4,
                 y_half_width=3.0):
    x_lip_end = x_top + lip
    def ground_z(x: float, y: float) -> float:
        if abs(y) > y_half_width: return 0.0
        if x < x_start: return 0.0
        if x < x_top:
            u = (x - x_start) / (x_top - x_start)
            return ramp_h * 0.5 * (1 - math.cos(math.pi * u))
        if x < x_lip_end: return ramp_h
        return 0.0
    return ground_z, dict(
        x_start=x_start, x_top=x_top, x_lip_end=x_lip_end,
        ramp_height=ramp_h, y_half_width=y_half_width,
    )


# -----------------------------------------------------------------------------
# State vector (28):
#   y[0..19]  same as before (chassis + suspension)
#   y[20..23] wheel spin ω_FL, FR, RL, RR  [rad/s]
#   y[24..27] transient lateral slip angle α_dyn_FL, FR, RL, RR  [rad]
#             (1st-order lag: σ/|v_long| · α̇_dyn = α_geom − α_dyn)
# -----------------------------------------------------------------------------
def derivatives(y, p: VParams, ground_z, steer: float, throttle: float,
                brake: float = 0.0):
    (x_w, y_w, vx, vy, yaw, yaw_rate,
     z_s, vz_s, pitch, pitch_rate, roll, roll_rate,
     z_uFL, z_uFR, z_uRL, z_uRR,
     vz_uFL, vz_uFR, vz_uRL, vz_uRR,
     w_FL, w_FR, w_RL, w_RR,
     ad_FL, ad_FR, ad_RL, ad_RR) = y

    rx = [+p.a, +p.a, -p.b, -p.b]
    ry = [+p.tw_f, -p.tw_f, +p.tw_r, -p.tw_r]
    z_u  = [z_uFL,  z_uFR,  z_uRL,  z_uRR]
    vz_u = [vz_uFL, vz_uFR, vz_uRL, vz_uRR]
    w    = [w_FL,   w_FR,   w_RL,   w_RR]
    a_dyn = [ad_FL, ad_FR, ad_RL, ad_RR]

    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll),  math.sin(roll)

    # Sprung corner world z (small angles for roll/pitch composition)
    #   z_corner = z_s + ry_i · sin(roll) - rx_i · sin(pitch)
    z_s_corner  = [z_s + ry[i] * sr - rx[i] * sp for i in range(4)]
    vz_s_corner = [vz_s + ry[i] * cr * roll_rate - rx[i] * cp * pitch_rate
                   for i in range(4)]

    # Static spring loads (front/rear weight split)
    F_corner_static = [p.m_sprung * G * (p.b if rx[i] > 0 else p.a)
                       / (p.a + p.b) / 2.0 for i in range(4)]
    delta_static = [F_corner_static[i] / p.k_spring[i] for i in range(4)]

    # Spring + damper, clipped (droop / bump stops)
    F_spring = []
    F_damper = []
    for i in range(4):
        delta = z_s_corner[i] - z_u[i]
        F_lin = p.k_spring[i] * (delta_static[i] - delta) + F_corner_static[i]
        F_lin = max(0.0, min(3.0 * F_corner_static[i], F_lin))
        F_spring.append(F_lin)
        if F_lin < 1e-3:
            F_damper.append(0.0)
        else:
            v_rel = vz_s_corner[i] - vz_u[i]
            F_damper.append(-p.c_damper[i] * v_rel)
    F_susp = [F_spring[i] + F_damper[i] for i in range(4)]

    # Tire vertical force — per wheel ground sampling
    # Wheel world position: rotate body offset by yaw
    cy, sy = math.cos(yaw), math.sin(yaw)
    F_tire = []
    z_g_corners = []
    for i in range(4):
        x_w_corner = x_w + rx[i] * cy - ry[i] * sy
        y_w_corner = y_w + rx[i] * sy + ry[i] * cy
        z_g = ground_z(x_w_corner, y_w_corner)
        z_g_corners.append(z_g)
        eps = (z_g + p.r_wheel) - z_u[i]
        F_tire.append(max(0.0, p.k_tire * eps))

    # Vertical sprung dynamics
    sum_F_susp = sum(F_susp)
    az_s = sum_F_susp / p.m_sprung - G

    # Pitch dynamics (with ax · h_cg coupling)
    tau_y = sum(-rx[i] * F_susp[i] for i in range(4))
    # ax coupling — needs current ax; iterative.  Use simplified estimate.
    # We will compute ax below and feed it back next step (1-step lag for the
    # ax · h_cg term, which is a small correction anyway).
    apitch = tau_y / p.I_yy

    # Roll dynamics — analogous: τ_x = Σ ry_i · F_susp_i + m_s · ay · h_cg
    tau_x = sum(ry[i] * F_susp[i] for i in range(4))
    # ay coupling: use lateral-accel estimate from previous body-frame vy_dot,
    # via 1-step lag below.
    aroll = tau_x / p.I_xx

    # Unsprung vertical
    az_u = [(-F_susp[i] + F_tire[i]) / p.m_unsprung[i] - G for i in range(4)]

    # ----- Per-wheel tire forces via Pacejka MF96 -----
    # Steered wheels (front), rest free-rolling.
    steer_per_wheel = [steer, steer, 0.0, 0.0]
    eps_v = 0.3   # m/s — slip-ratio regularization at low speed
    # Drive torque distribution: throttle * max_motor_torque (total) split per
    # drivetrain.  Braking: brake * max_brake_torque with brake-bias on front.
    T_drive_total = throttle * p.max_motor_torque
    if p.drive_type == "RWD":
        T_drive = [0.0, 0.0, 0.5 * T_drive_total, 0.5 * T_drive_total]
    elif p.drive_type == "FWD":
        T_drive = [0.5 * T_drive_total, 0.5 * T_drive_total, 0.0, 0.0]
    else:    # AWD
        T_drive = [0.25 * T_drive_total] * 4
    T_brake_total = brake * p.max_brake_torque
    T_brake = [
        0.5 * p.brake_bias_front * T_brake_total,
        0.5 * p.brake_bias_front * T_brake_total,
        0.5 * (1 - p.brake_bias_front) * T_brake_total,
        0.5 * (1 - p.brake_bias_front) * T_brake_total,
    ]

    Fx_body = [0.0] * 4    # tire force at each wheel, projected into BODY frame
    Fy_body = [0.0] * 4
    alpha_arr = [0.0] * 4
    kappa_arr = [0.0] * 4
    w_dot = [0.0] * 4
    a_dyn_dot = [0.0] * 4
    sigma_y = max(0.01, p.tire.relaxation_length_lat)
    for i in range(4):
        # Body-frame velocity at wheel center: v_body + ω_z × r_wheel
        v_wx_body = vx - yaw_rate * ry[i]
        v_wy_body = vy + yaw_rate * rx[i]
        # Rotate into wheel frame by steer angle
        cs, ss = math.cos(steer_per_wheel[i]), math.sin(steer_per_wheel[i])
        v_long = v_wx_body * cs + v_wy_body * ss
        v_lat  = -v_wx_body * ss + v_wy_body * cs
        # Geometric slip angle
        v_long_safe = math.copysign(max(eps_v, abs(v_long)), v_long if v_long != 0 else 1.0)
        alpha_geom = math.atan2(v_lat, v_long_safe)
        # Transient slip angle (relaxation length): σ/|v| · α̇_dyn = α_geom − α_dyn
        # Equivalently:   α̇_dyn = (|v_long| / σ) · (α_geom − α_dyn)
        a_dyn_dot[i] = (abs(v_long_safe) / sigma_y) * (alpha_geom - a_dyn[i])
        # Slip ratio (no longitudinal transient — σ_long small relative to dt)
        kappa = (w[i] * p.r_wheel - v_long) / max(eps_v, abs(v_long_safe))
        alpha_arr[i] = a_dyn[i]; kappa_arr[i] = kappa

        # Pacejka uses transient α (and instantaneous κ)
        Fx_w, Fy_w = p.tire.compute(F_tire[i], kappa, a_dyn[i])
        # Project wheel-frame force back to body frame
        Fx_body[i] = Fx_w * cs - Fy_w * ss
        Fy_body[i] = Fx_w * ss + Fy_w * cs

        # Wheel spin dynamics: I · ω_dot = T_drive - T_brake_sign - Fx_w · r_eff
        T_brk_signed = math.copysign(T_brake[i], w[i]) if abs(w[i]) > 0.1 else 0.0
        w_dot[i] = (T_drive[i] - T_brk_signed - Fx_w * p.r_wheel) / p.I_wheel

    # Aero drag + rolling resistance (chassis-level)
    rho = 1.2
    F_drag = 0.5 * rho * p.Cd * p.A_front * vx * abs(vx)
    F_roll_res = p.f_roll * sum(F_tire)

    Fx_total = sum(Fx_body) - F_drag - math.copysign(F_roll_res, vx)
    Fy_total = sum(Fy_body)
    ax_body = Fx_total / p.m_total + vy * yaw_rate
    ay_body = Fy_total / p.m_total - vx * yaw_rate

    # Yaw moment: M_z = Σ (rx_i · Fy_body_i  −  ry_i · Fx_body_i)
    Mz = sum(rx[i] * Fy_body[i] - ry[i] * Fx_body[i] for i in range(4))
    ayaw = Mz / p.I_zz

    # World velocity from body velocity
    vx_world = vx * cy - vy * sy
    vy_world = vx * sy + vy * cy

    # Phenomenological inertial-coupling moment from sprung-mass lateral /
    # longitudinal accel.  Standard textbook form M = m_s · a · (h_cg - h_axis).
    # Gate by tire-contact fraction so it vanishes during airborne (no road
    # reaction → no inertial moment about the suspension roll/pitch axis).
    Fz_total_static = 4 * (F_corner_static[0] + F_corner_static[2]) / 2.0
    contact_frac = min(1.0, sum(F_tire) / max(1.0, Fz_total_static))
    h_arm_roll  = p.h_cg - p.h_roll_axis
    h_arm_pitch = p.h_cg - p.h_pitch_axis
    apitch += contact_frac * p.m_sprung * ax_body * h_arm_pitch / p.I_yy
    aroll  += contact_frac * p.m_sprung * ay_body * h_arm_roll  / p.I_xx

    return [
        vx_world, vy_world, ax_body, ay_body, yaw_rate, ayaw,
        vz_s, az_s, pitch_rate, apitch, roll_rate, aroll,
        vz_u[0], vz_u[1], vz_u[2], vz_u[3],
        az_u[0], az_u[1], az_u[2], az_u[3],
        w_dot[0], w_dot[1], w_dot[2], w_dot[3],
        a_dyn_dot[0], a_dyn_dot[1], a_dyn_dot[2], a_dyn_dot[3],
    ]


def rk4(y, h, p, gz, steer, throttle, brake=0.0):
    k1 = derivatives(y, p, gz, steer, throttle, brake)
    y2 = [y[i] + 0.5 * h * k1[i] for i in range(len(y))]
    k2 = derivatives(y2, p, gz, steer, throttle, brake)
    y3 = [y[i] + 0.5 * h * k2[i] for i in range(len(y))]
    k3 = derivatives(y3, p, gz, steer, throttle, brake)
    y4 = [y[i] + h * k3[i] for i in range(len(y))]
    k4 = derivatives(y4, p, gz, steer, throttle, brake)
    return [y[i] + h * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0
            for i in range(len(y))]


# -----------------------------------------------------------------------------
# Steering schedule
# -----------------------------------------------------------------------------
def steer_schedule(t: float, t_steer_start=1.0, t_steer_end=1.65,
                   steer_max=0.10) -> float:
    """Left-steer pulse spanning ramp ascent — yaw rate is non-zero at take-off
    so the vehicle keeps rotating during airborne."""
    if t < t_steer_start: return 0.0
    if t < t_steer_end:   return steer_max
    return 0.0


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle", default="configs/vehicles/sports.yaml")
    ap.add_argument("--tire", default="configs/tires/default_pacejka.yaml")
    ap.add_argument("--out", default="apps/jump_demo/run3d")
    ap.add_argument("--v0", type=float, default=15.0)
    ap.add_argument("--ramp_h", type=float, default=0.6)
    ap.add_argument("--ramp_start", type=float, default=20.0)
    ap.add_argument("--ramp_top",   type=float, default=24.0)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--dt", type=float, default=0.0005)
    args = ap.parse_args()

    p = load_params(args.vehicle, args.tire)
    gz, ramp_info = ramp_profile(args.ramp_start, args.ramp_top, args.ramp_h)

    # Static-eq initial state
    z_u0 = [p.r_wheel - ((p.m_sprung / 4 + p.m_unsprung[i]) * G / p.k_tire)
            for i in range(4)]
    z_s0 = sum(
        z_u0[i] + (p.m_sprung * G * (p.b if rx > 0 else p.a) / (p.a + p.b) / 2.0) / p.k_spring[i]
        for i, rx in enumerate([+p.a, +p.a, -p.b, -p.b])
    ) / 4.0

    w0 = args.v0 / p.r_wheel    # free-rolling wheel spin matching vx
    y = [
        0.0, 0.0,                      # x_w, y_w
        args.v0, 0.0,                  # vx, vy
        0.0, 0.0,                      # yaw, yaw_rate
        z_s0, 0.0, 0.0, 0.0, 0.0, 0.0, # z_s..roll_rate
        z_u0[0], z_u0[1], z_u0[2], z_u0[3],
        0.0, 0.0, 0.0, 0.0,
        w0, w0, w0, w0,                # wheel spin (initially rolling at vx/r)
        0.0, 0.0, 0.0, 0.0,            # transient α_dyn per wheel (straight)
    ]

    rows = []
    n = int(args.duration / args.dt)
    log_every = int(0.01 / args.dt)
    for step in range(n):
        t = step * args.dt
        steer = steer_schedule(t)
        throttle = 0.1
        brake = 0.0
        y = rk4(y, args.dt, p, gz, steer, throttle, brake)

        if step % log_every == 0:
            (x_w, y_w, vx, vy, yaw, yaw_rate,
             z_s, vz_s, pitch, pitch_rate, roll, roll_rate,
             *_) = y
            z_u = y[12:16]
            wheels = y[20:24]
            a_dyn_log = y[24:28]
            cy, sy = math.cos(yaw), math.sin(yaw)
            rx = [+p.a, +p.a, -p.b, -p.b]
            ry = [+p.tw_f, -p.tw_f, +p.tw_r, -p.tw_r]
            Fz = []; kappa = []; alpha_geom = []; Fx_w = []; Fy_w = []
            for i in range(4):
                x_c = x_w + rx[i]*cy - ry[i]*sy
                y_c = y_w + rx[i]*sy + ry[i]*cy
                z_g = gz(x_c, y_c)
                eps = (z_g + p.r_wheel) - z_u[i]
                Fz_i = max(0.0, p.k_tire * eps)
                Fz.append(Fz_i)
                st = steer if i < 2 else 0.0
                vwx = vx - yaw_rate * ry[i]
                vwy = vy + yaw_rate * rx[i]
                cs, ss = math.cos(st), math.sin(st)
                v_long = vwx * cs + vwy * ss
                v_lat  = -vwx * ss + vwy * cs
                eps_v = 0.3
                vl_safe = math.copysign(max(eps_v, abs(v_long)), v_long if v_long != 0 else 1.0)
                a_g = math.atan2(v_lat, vl_safe)
                k_i = (wheels[i] * p.r_wheel - v_long) / max(eps_v, abs(vl_safe))
                # Force uses transient α_dyn (matches what the integrator used)
                Fx_i, Fy_i = p.tire.compute(Fz_i, k_i, a_dyn_log[i])
                kappa.append(k_i); alpha_geom.append(a_g)
                Fx_w.append(Fx_i); Fy_w.append(Fy_i)
            airborne = all(f < 1.0 for f in Fz)
            rows.append({
                "t": round(t, 3),
                "x": x_w, "y": y_w, "yaw": yaw, "yaw_rate": yaw_rate,
                "vx": vx, "vy": vy,
                "z_s": z_s, "vz_s": vz_s,
                "pitch": pitch, "roll": roll,
                "steer": steer,
                "z_u_FL": z_u[0], "z_u_FR": z_u[1],
                "z_u_RL": z_u[2], "z_u_RR": z_u[3],
                "Fz_FL": Fz[0], "Fz_FR": Fz[1],
                "Fz_RL": Fz[2], "Fz_RR": Fz[3],
                "Fx_FL": Fx_w[0], "Fx_FR": Fx_w[1],
                "Fx_RL": Fx_w[2], "Fx_RR": Fx_w[3],
                "Fy_FL": Fy_w[0], "Fy_FR": Fy_w[1],
                "Fy_RL": Fy_w[2], "Fy_RR": Fy_w[3],
                "kappa_FL": kappa[0], "kappa_FR": kappa[1],
                "kappa_RL": kappa[2], "kappa_RR": kappa[3],
                "alpha_FL": alpha_geom[0], "alpha_FR": alpha_geom[1],
                "alpha_RL": alpha_geom[2], "alpha_RR": alpha_geom[3],
                "alpha_dyn_FL": a_dyn_log[0], "alpha_dyn_FR": a_dyn_log[1],
                "alpha_dyn_RL": a_dyn_log[2], "alpha_dyn_RR": a_dyn_log[3],
                "w_FL": wheels[0], "w_FR": wheels[1],
                "w_RL": wheels[2], "w_RR": wheels[3],
                "airborne": airborne,
            })

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(out_dir / "telemetry.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    with open(out_dir / "ramp.yaml", "w") as f:
        yaml.safe_dump(ramp_info, f)
    with open(out_dir / "veh.yaml", "w") as f:
        yaml.safe_dump({
            "a": p.a, "b": p.b,
            "track_front": 2*p.tw_f, "track_rear": 2*p.tw_r,
            "r_wheel": p.r_wheel, "h_cg": p.h_cg,
        }, f)

    # Summary
    air = [r["airborne"] for r in rows]
    if any(air):
        i0 = air.index(True); i1 = len(air)-1-list(reversed(air)).index(True)
        t_air = rows[i1]["t"] - rows[i0]["t"]
    else:
        t_air = 0.0
    yaw_start = rows[next(i for i, r in enumerate(rows) if r["airborne"])]["yaw"] if any(air) else 0
    yaw_end = rows[max(i for i, r in enumerate(rows) if r["airborne"])]["yaw"] if any(air) else 0
    print(f"[sim3d] samples={len(rows)} -> {out_dir/'telemetry.csv'}")
    print(f"[sim3d] airborne ≈ {t_air:.3f} s ; Δyaw airborne = {math.degrees(yaw_end - yaw_start):+.1f} deg")
    print(f"[sim3d] sprung CG peak z = {max(r['z_s'] for r in rows):.3f} m")
    print(f"[sim3d] peak |roll|  = {math.degrees(max(abs(r['roll']) for r in rows)):.1f} deg")
    print(f"[sim3d] peak |pitch| = {math.degrees(max(abs(r['pitch']) for r in rows)):.1f} deg")


if __name__ == "__main__":
    main()
