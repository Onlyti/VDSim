#!/usr/bin/env python3
"""VDSim web GUI server (MVP+) — stdlib only, no extra pip deps.

Compute runs here (vdsim SimSession); the browser does all visualization and
configuration (vehicle params, tire params, sim settings). State streams via
Server-Sent Events; config/control over REST.

Usage:
    cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build -j
    python3 gui/server.py [--port 8090]
    # open http://<server>:8090
"""
import argparse
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "cosim"))
sys.path.insert(0, str(REPO / "python"))
HERE = Path(__file__).resolve().parent
TIRE_DIR = REPO / "configs" / "tires"
SUSP_DIR = REPO / "configs" / "suspensions"
SUSP_REL_PREFIX = "configs/suspensions"
VEHICLE_SUSP_DEFAULT = {
    "sedan": {"front": "mp_front_sedan", "rear": "ta_rear_sedan"},
    "sports": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
    "fsk_formula": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
    "race_car": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
}


def susp_stem_from_ref(ref):
    if not ref:
        return ""
    return Path(str(ref)).stem


def susp_rel_path(stem_or_ref):
    stem = susp_stem_from_ref(stem_or_ref)
    if not stem:
        return None
    if not (SUSP_DIR / f"{stem}.yaml").is_file():
        return None
    return f"{SUSP_REL_PREFIX}/{stem}.yaml"


def is_l3_kinematics_yaml(path_or_stem):
    stem = susp_stem_from_ref(path_or_stem)
    path = SUSP_DIR / f"{stem}.yaml"
    if not path.is_file():
        return False
    import yaml
    doc = yaml.safe_load(path.read_text()) or {}
    return bool(doc.get("type"))


def list_l3_kinematics_configs():
    return sorted(s for s in list_suspension_configs() if is_l3_kinematics_yaml(s))


def list_suspension_api(preview_all=False):
    l3 = list_l3_kinematics_configs()
    if preview_all:
        return {"samples": l3, "preview": list_suspension_configs()}
    return {"samples": l3}


def _strip_fleet_susp_if_not_l3(spec):
    if str(spec.get("level", "L2")) != "L3":
        spec.pop("front_susp", None)
        spec.pop("rear_susp", None)


def _l3_susp_path_warnings(fleet_spec):
    warnings = []
    for spec in fleet_spec:
        if str(spec.get("level", "L2")) != "L3":
            continue
        vid = int(spec.get("id", 0))
        for side, key in (("front", "front_susp"), ("rear", "rear_susp")):
            stem = spec.get(key)
            if not stem:
                continue
            ref = susp_stem_from_ref(stem)
            path = SUSP_DIR / f"{ref}.yaml"
            if not path.is_file():
                warnings.append(
                    f"[vdsim] vehicle {vid}: {side} susp '{stem}' not found")
            elif not is_l3_kinematics_yaml(ref):
                warnings.append(
                    f"[vdsim] vehicle {vid}: {side} susp '{stem}' "
                    "is not L3-native (topology-only YAML)")
    return warnings


_KIN_WARN_MARKERS = ("kinematics attach failed", "front susp", "rear susp")


def _scan_kinematics_warnings(log_path):
    warnings = []
    try:
        text = Path(log_path).read_text(errors="replace")
    except OSError:
        return warnings
    for line in text.splitlines():
        if "[vdsim_realtime]" not in line:
            continue
        if any(m in line for m in _KIN_WARN_MARKERS):
            warnings.append(line.strip())
    return warnings


def _corner_kinematic_links(doc):
    links = []
    for arm in ("lca", "uca"):
        block = doc.get(arm)
        if not isinstance(block, dict):
            continue
        cf, cr, kn = block["chassis_front"], block["chassis_rear"], block["knuckle"]
        links.extend([[cf, cr], [cf, kn], [cr, kn]])
    st = doc.get("strut")
    if isinstance(st, dict) and "top" in st and "bottom" in st:
        links.append([st["top"], st["bottom"]])
    tr = doc.get("tie_rod")
    if isinstance(tr, dict):
        links.append([tr["rack"], tr["knuckle"]])
    sd = doc.get("spring_damper")
    if isinstance(sd, dict) and "chassis" in sd and "lca" in sd:
        links.append([sd["chassis"], sd["lca"]])
    ap = doc.get("arm_pivot")
    if isinstance(ap, dict):
        pi, po = ap["chassis_inboard"], ap["chassis_outboard"]
        links.append([pi, po])
        wh = (doc.get("wheel") or {}).get("center")
        if wh:
            mid = [(pi[i] + po[i]) / 2.0 for i in range(3)]
            links.append([mid, wh])
    lb = doc.get("links")
    if isinstance(lb, dict):
        for block in lb.values():
            if isinstance(block, dict) and "chassis" in block and "knuckle" in block:
                links.append([block["chassis"], block["knuckle"]])
    hps = doc.get("hardpoints")
    if isinstance(hps, list):
        hp = {h["name"]: h["position"] for h in hps if isinstance(h, dict) and "name" in h}
        for a, b in (
            ("uca_inner_front", "uca_outer_ball"), ("uca_inner_rear", "uca_outer_ball"),
            ("lca_inner_front", "lca_outer_ball"), ("lca_inner_rear", "lca_outer_ball"),
            ("tie_rod_inner", "tie_rod_outer"), ("pushrod_lower", "pushrod_upper"),
            ("damper_top", "damper_bottom"),
        ):
            if a in hp and b in hp:
                links.append([hp[a], hp[b]])
    pts = {}
    wh = (doc.get("wheel") or {}).get("center")
    if wh:
        pts["wheel"] = wh
    return links, pts


def list_suspension_configs():
    if not SUSP_DIR.is_dir():
        return []
    return sorted(p.stem for p in SUSP_DIR.glob("*.yaml"))


def suspension_default_for_vehicle(vehicle):
    stem = Path(str(vehicle)).stem
    return dict(VEHICLE_SUSP_DEFAULT.get(stem, {
        "front": "mp_front_sedan", "rear": "ta_rear_sedan",
    }))


def parts_registry():
    comp = REPO / "configs" / "components" / "suspension"
    presets = sorted(p.stem for p in comp.glob("*.yaml")) if comp.is_dir() else []
    veh = REPO / "configs" / "vehicles"
    tires = REPO / "configs" / "tires"
    return {
        "vehicles": sorted(p.stem for p in veh.glob("*.yaml")),
        "tires": sorted(p.stem for p in tires.glob("*.yaml")),
        "suspensions": list_suspension_configs(),
        "l3_kinematics": list_l3_kinematics_configs(),
        "suspension_presets": presets,
        "vehicle_suspension_defaults": {
            v: suspension_default_for_vehicle(v) for v in VEHICLES
        },
    }


def suspension_schematic(name):
    import yaml
    stem = Path(str(name)).stem
    path = SUSP_DIR / f"{stem}.yaml"
    if not path.is_file():
        raise ValueError(f"unknown suspension: {name}")
    doc = yaml.safe_load(path.read_text()) or {}
    typ = str(doc.get("type") or doc.get("topology") or "unknown")
    links, pts = _corner_kinematic_links(doc)
    return {"name": stem, "type": typ, "links": links, "points": pts}


try:
    import vdsim
except ImportError as e:
    sys.exit(f"import vdsim failed ({e}). Build with -DVDSIM_BUILD_PYTHON=ON.")

import protocol as vds1   # canonical VDS1 wire format (one definition, cosim/protocol.py)

VEHICLES = ["sedan", "sports", "fsk_formula", "race_car"]
LEVELS = ["K", "L1", "L2", "L3"]   # K = kinematic bicycle (no tire/slip)
_ALL = "K,L1,L2,L3"

# Enum value maps (name <-> bound enum) for dropdown fields.
ENUM_MAPS = {
    "drive_type":   {"FWD": vdsim.Drive.FWD, "RWD": vdsim.Drive.RWD,
                     "AWD": vdsim.Drive.AWD},
    "differential": {"Open": vdsim.Differential.Open,
                     "Locked": vdsim.Differential.Locked,
                     "LSD": vdsim.Differential.LSD},
    "integrator":   {"Euler": vdsim.Integrator.Euler, "RK4": vdsim.Integrator.RK4},
}

# Editable parameter schema: (attr, label, group, kind, applicable_levels)
#   kind: "num" scalar | "arr" 4-wheel | "bool" checkbox | "enum" dropdown
VEHICLE_FIELDS = [
    ("mass", "Mass [kg]", "Mass & inertia", "num", _ALL),
    ("mass_sprung", "Sprung mass [kg]", "Mass & inertia", "num", "L3"),
    ("ixx", "Roll inertia Ixx [kg·m²]", "Mass & inertia", "num", "L3"),
    ("iyy", "Pitch inertia Iyy [kg·m²]", "Mass & inertia", "num", "L3"),
    ("izz", "Yaw inertia Izz [kg·m²]", "Mass & inertia", "num", "L1,L2,L3"),
    ("wheelbase", "Wheelbase [m]", "Geometry", "num", _ALL),
    ("cg_to_front", "CG→front [m]", "Geometry", "num", _ALL),
    ("cg_to_rear", "CG→rear [m]", "Geometry", "num", _ALL),
    ("track_front", "Track front [m]", "Geometry", "num", "L2,L3"),
    ("track_rear", "Track rear [m]", "Geometry", "num", "L2,L3"),
    ("cg_height", "CG height [m]", "Geometry", "num", "L1,L2,L3"),
    ("wheel_radius_nominal", "Wheel radius [m]", "Geometry", "num", _ALL),
    ("spring_stiffness", "Spring stiffness [N/m]", "Suspension", "arr", "L2,L3"),
    ("damper_coefficient", "Damper [N·s/m]", "Suspension", "arr", "L3"),
    ("unsprung_mass", "Unsprung mass [kg]", "Suspension", "arr", "L2,L3"),
    ("wheel_inertia", "Wheel inertia [kg·m²] (0=auto)", "Suspension", "arr", "L1,L2,L3"),
    ("arb_stiffness_front", "Anti-roll bar front", "Suspension", "num", "L2,L3"),
    ("arb_stiffness_rear", "Anti-roll bar rear", "Suspension", "num", "L2,L3"),
    ("roll_center_height_front", "Roll center height front [m]", "Suspension", "num", "L2,L3"),
    ("roll_center_height_rear", "Roll center height rear [m]", "Suspension", "num", "L2,L3"),
    ("anti_dive_front", "Anti-dive front [-] (typ 0–1)", "Suspension", "num", "L3"),
    ("anti_squat_rear", "Anti-squat rear [-] (typ 0–1)", "Suspension", "num", "L3"),
    ("camber_per_roll", "Camber/roll gain [rad/rad]", "Suspension", "num", "L3"),
    ("drive_type", "Drive", "Drivetrain", "enum", "L1,L2,L3"),
    ("differential", "Differential", "Drivetrain", "enum", "L2,L3"),
    ("lsd_preload", "LSD preload [-]", "Drivetrain", "num", "L2,L3"),
    ("lsd_ramp", "LSD ramp [-]", "Drivetrain", "num", "L2,L3"),
    ("max_motor_torque", "Max motor torque [N·m]", "Drivetrain", "num", _ALL),
    ("final_drive_ratio", "Final drive ratio [-]", "Drivetrain", "num", _ALL),
    ("drive_deadtime_s", "Throttle deadtime [s]", "Drivetrain", "num", "L2,L3"),
    ("max_brake_torque", "Max brake torque [N·m]", "Drivetrain", "num", _ALL),
    ("brake_bias_front", "Brake bias — front share [0–1] (rear = 1−front)", "Drivetrain", "num", "L1,L2,L3"),
    ("brake_ebd_enabled", "Brake EBD (Fz-based bias)", "Drivetrain", "bool", "L2,L3"),
    ("brake_deadtime_s", "Brake deadtime [s]", "Drivetrain", "num", "L2,L3"),
    ("steering_ratio", "Steering ratio [-]", "Steering", "num", "L1,L2,L3"),
    ("steer_deadtime_s", "Steer deadtime [s]", "Steering", "num", "L2,L3"),
    ("max_steer_angle_wheel", "Max steer [rad]", "Steering", "num", _ALL),
    ("ackerman_percent", "Ackermann [%]", "Steering", "num", "L2,L3"),
    ("aero_drag_coeff", "Drag coeff [-]", "Aero", "num", "L1,L2,L3"),
    ("frontal_area", "Frontal area [m²]", "Aero", "num", "L1,L2,L3"),
    ("aero_lift_front", "Lift coeff front [-]", "Aero", "num", "L1,L2,L3"),
    ("aero_lift_rear", "Lift coeff rear [-]", "Aero", "num", "L1,L2,L3"),
]
TIRE_FIELDS = [
    ("B_long", "B long", "Longitudinal", "num", "L1,L2,L3"),
    ("C_long", "C long", "Longitudinal", "num", "L1,L2,L3"),
    ("D_long", "D long", "Longitudinal", "num", "L1,L2,L3"),
    ("E_long", "E long", "Longitudinal", "num", "L1,L2,L3"),
    ("B_lat", "B lat", "Lateral", "num", "L1,L2,L3"),
    ("C_lat", "C lat", "Lateral", "num", "L1,L2,L3"),
    ("D_lat", "D lat", "Lateral", "num", "L1,L2,L3"),
    ("E_lat", "E lat", "Lateral", "num", "L1,L2,L3"),
    ("mu_nominal", "μ nominal", "General", "num", "L1,L2,L3"),
    ("Fz_nominal", "Fz nominal [N]", "General", "num", "L1,L2,L3"),
    ("cornering_stiffness", "Cornering stiffness [N/rad]", "General", "num", "L1,L2,L3"),
    ("rolling_resistance", "Rolling resistance", "General", "num", "L1,L2,L3"),
    ("load_sensitivity", "Load sensitivity", "General", "num", "L1,L2,L3"),
    ("combined_slip_enabled", "Combined slip (friction ellipse)", "General", "bool", "L1,L2,L3"),
    ("pneumatic_trail", "Pneumatic trail [m]", "Aligning", "num", "L1,L2,L3"),
    ("trail_falloff_alpha", "Trail falloff α [rad]", "Aligning", "num", "L1,L2,L3"),
    ("camber_stiffness", "Camber stiffness [1/rad]", "Camber", "num", "L1,L2,L3"),
    ("relaxation_length_lat", "Relaxation len lat [m]", "Transient", "num", "L1,L2,L3"),
    ("relaxation_length_long", "Relaxation len long [m]", "Transient", "num", "L1,L2,L3"),
    ("tire_vertical_stiffness", "Vertical stiffness [N/m]", "Vertical", "num", "L3"),
]
# Actuator + feedback schema. Dotted paths walk the nested ActuatorParams; the
# two "@" names are handled specially (sensor delay + solver substeps).
ACTUATOR_FIELDS = [
    ("steer.ch.dead_time_s", "Steer dead time [s]", "Steering", "num"),
    ("steer.ch.tau_s", "Steer lag τ [s]", "Steering", "num"),
    ("steer.ch.rate_limit", "Steer rate limit [rad/s] (0=off)", "Steering", "num"),
    ("steer.friction.enabled", "Servo+LuGre mode (off → first-order lag)", "Steering", "bool"),
    ("steer.servo_kp", "Servo kp", "Steering", "num"),
    ("steer.servo_kd", "Servo kd", "Steering", "num"),
    ("throttle.dead_time_s", "Throttle dead time [s]", "Throttle", "num"),
    ("throttle.tau_s", "Throttle lag τ [s]", "Throttle", "num"),
    ("throttle.rate_limit", "Throttle rate limit [1/s] (0=off)", "Throttle", "num"),
    ("throttle.dead_zone", "Throttle dead-zone [-] (pedal tip-in)", "Throttle", "num"),
    ("brake.ch.dead_time_s", "Brake dead time [s]", "Brake", "num"),
    ("brake.ch.tau_s", "Brake lag τ [s]", "Brake", "num"),
    ("brake.ch.dead_zone", "Brake dead-zone [-] (pad clearance)", "Brake", "num"),
    ("brake.thermal_enabled", "Brake thermal fade", "Brake", "bool"),
    ("@sensor_delay_s", "Sensor feedback delay [s]", "Feedback", "num"),
]
# Sensor noise/bias schema (dotted paths into SensorParams). "enabled" is a bool.
SENSOR_FIELDS = [
    ("enabled", "Sensors enabled (off → truth)", "General", "bool"),
    ("imu_accel.noise_std", "IMU accel noise [m/s²]", "IMU", "num"),
    ("imu_accel.bias", "IMU accel bias [m/s²]", "IMU", "num"),
    ("imu_gyro.noise_std", "IMU gyro noise [rad/s]", "IMU", "num"),
    ("imu_gyro.bias", "IMU gyro bias [rad/s]", "IMU", "num"),
    ("imu_gyro.bias_rw", "IMU gyro bias random-walk", "IMU", "num"),
    ("wheel_speed.noise_std", "Wheel-speed noise [rad/s]", "Wheel", "num"),
    ("steer.noise_std", "Steer noise [rad]", "Steer", "num"),
    ("steer.bias", "Steer bias [rad]", "Steer", "num"),
    ("gnss_pos.noise_std", "GNSS position noise [m]", "GNSS", "num"),
    ("gnss_vel.noise_std", "GNSS velocity noise [m/s]", "GNSS", "num"),
]


def _get_dotted(obj, path):
    for p in path.split("."):
        obj = getattr(obj, p)
    return obj


def _set_dotted(obj, path, value):
    parts = path.split(".")
    for p in parts[:-1]:
        obj = getattr(obj, p)
    setattr(obj, parts[-1], value)


# Recorded CSV columns (ground truth + measured-ish + command).
LOG_COLS = ["t", "x", "y", "z", "yaw", "roll", "pitch", "vx", "vy", "r",
            "wx", "wy", "ax", "ay", "steer", "Fz0", "Fz1", "Fz2", "Fz3",
            "cmd_throttle", "cmd_brake", "cmd_steer", "source", "level",
            "m_gnss_x", "m_gnss_y", "m_ax", "m_ay", "m_wz", "m_steer",
            # per-wheel tire ground-truth (FL,FR,RL,RR) for Fz/mu/Calpha estimation
            "Fx0", "Fx1", "Fx2", "Fx3", "Fy0", "Fy1", "Fy2", "Fy3",
            "kappa0", "kappa1", "kappa2", "kappa3",
            "alpha0", "alpha1", "alpha2", "alpha3"]


def euler_to_quat(roll, pitch, yaw):
    """ZYX intrinsic (yaw->pitch->roll) Euler -> (qx,qy,qz,qw), matching coordinate.hpp."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,   # qx
            cr * sp * cy + sr * cp * sy,   # qy
            cr * cp * sy - sr * sp * cy,   # qz
            cr * cp * cy + sr * sp * sy)   # qw


def _field_value(obj, attr, kind):
    if kind == "enum":
        return getattr(obj, attr).name
    if kind == "bool":
        return bool(getattr(obj, attr))
    if kind == "arr":
        return [float(x) for x in getattr(obj, attr)]
    return float(getattr(obj, attr))


def _serialize(obj, fields):
    out = []
    for attr, label, group, kind, *rest in fields:
        levels = rest[0] if rest else "L1,L2,L3"
        d = {"name": attr, "label": label, "group": group, "kind": kind,
             "levels": levels.split(","), "value": _field_value(obj, attr, kind)}
        if kind == "enum":
            d["choices"] = list(ENUM_MAPS[attr].keys())
        out.append(d)
    return out


def _params_dict(obj, fields):
    return {f[0]: _field_value(obj, f[0], f[3]) for f in fields}


def _flat_sensors(sensors):
    out = {}
    for attr, _, _, kind in SENSOR_FIELDS:
        if kind == "bool":
            out[attr] = bool(getattr(sensors, attr))
        else:
            out[attr] = float(_get_dotted(sensors, attr))
    return out


def _flat_actuator(act, sensor_delay):
    out = {}
    for attr, _, _, kind in ACTUATOR_FIELDS:
        if attr == "@sensor_delay_s":
            out[attr] = float(sensor_delay)
        elif kind == "bool":
            out[attr] = bool(_get_dotted(act, attr))
        else:
            out[attr] = float(_get_dotted(act, attr))
    return out


def _apply(obj, fields, data):
    kinds = {f[0]: f[3] for f in fields}
    for k, v in data.items():
        kind = kinds.get(k)
        if kind is None or not hasattr(obj, k):
            continue
        if kind == "enum":
            setattr(obj, k, ENUM_MAPS[k][v])
        elif kind == "bool":
            setattr(obj, k, bool(v))
        elif kind == "arr":
            setattr(obj, k, [float(x) for x in v])
        else:
            setattr(obj, k, float(v))


class WaypointPath:
    """Pure-pursuit over an ordered polyline (loops). Reused for the figure-8
    default and for loaded OpenDRIVE routes."""
    def __init__(self, pts):
        self.pts = list(pts)

    def steer(self, x, y, yaw, vx, wb, prev_idx):
        n = len(self.pts)
        if n < 2:
            return 0.0, prev_idx
        # Anchor to the nearest route point (robust when off-path), then look
        # ahead Ld from there — avoids locking onto a stale point and spinning.
        near, nd = prev_idx, 1e18
        for i in range(n):
            dx = self.pts[i][0] - x
            dy = self.pts[i][1] - y
            d2 = dx * dx + dy * dy
            if d2 < nd:
                nd, near = d2, i
        Ld = max(3.0, 0.6 * max(vx, 1.0))
        idx, cnt = near, 0
        while cnt < n:
            p = self.pts[idx % n]
            if math.hypot(p[0] - x, p[1] - y) >= Ld:
                break
            idx += 1
            cnt += 1
        idx %= n
        cp, sp = math.cos(yaw), math.sin(yaw)
        dxw, dyw = self.pts[idx][0] - x, self.pts[idx][1] - y
        dx = cp * dxw + sp * dyw
        dy = -sp * dxw + cp * dyw
        l2 = dx * dx + dy * dy
        if l2 < 1e-6:
            return 0.0, near
        return max(-0.6, min(0.6, math.atan(2.0 * dy / l2 * wb))), near


def fig8_pts(cx=0.0, cy=0.0, R=20.0, n=80):
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        pts.append((cx + R - R * math.cos(t), cy + R * math.sin(t)))
    for i in range(n):
        t = 2 * math.pi * i / n
        pts.append((cx - R + R * math.cos(t), cy + R * math.sin(t)))
    return pts


class FigureEight(WaypointPath):
    def __init__(self, R=20.0, n=80):
        super().__init__(fig8_pts(0.0, 0.0, R, n))


COSIM_BIN = REPO / "build" / "bin" / ("vdsim_realtime.exe" if os.name == "nt"
                                      else "vdsim_realtime")
COSIM_CMD_PORT = 7401
COSIM_STATE_PORT = 7402


def _pids_on_udp_port(port):
    out = []
    try:
        r = subprocess.run(["ss", "-H", "-ulnp"],
                           capture_output=True, text=True, timeout=2)
    except (OSError, subprocess.TimeoutExpired):
        return out
    needle = f":{int(port)}"
    for line in (r.stdout or "").splitlines():
        if needle not in line:
            continue
        for tok in line.split():
            if "pid=" in line:
                for m in re.finditer(r'pid=(\d+)', line):
                    out.append(int(m.group(1)))
    return out


def _is_vdsim_realtime_pid(pid):
    try:
        raw = Path(f"/proc/{int(pid)}/cmdline").read_bytes()
    except OSError:
        return False
    cmd = raw.replace(b"\x00", b" ").decode(errors="ignore")
    return "vdsim_realtime" in cmd


def _cleanup_stale_plant(cmd_port=7401):
    killed = []
    for pid in _pids_on_udp_port(cmd_port):
        if not _is_vdsim_realtime_pid(pid):
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            killed.append(pid)
        except OSError:
            pass
    if killed:
        time.sleep(0.15)
    return killed


def _write_terrain(path, terrain):
    """Write a baked heightmap to the udp_server's binary terrain format:
    int32 nx, ny, double x0,y0,dx,dy, then nx*ny doubles row-major h[iy*nx+ix]."""
    import struct
    import numpy as np
    H = np.asarray(terrain["H"], dtype="<f8")
    ny, nx = H.shape
    with open(path, "wb") as f:
        f.write(struct.pack("<ii", int(nx), int(ny)))
        f.write(struct.pack("<dddd", float(terrain["x0"]), float(terrain["y0"]),
                            float(terrain["dx"]), float(terrain["dy"])))
        f.write(H.tobytes())   # C-order row-major == h[iy*nx+ix]


class CosimBridge:
    """Launches the binary vdsim_realtime, consumes its STATE packets for the
    3D view, and relays control as CMD packets (per cosim_protocol.hpp).

    The GUI configures and runs the real co-sim server; the Python playground sim
    is bypassed while the bridge is active so there is one source of truth.
    """
    # Private loopback ports for the GUI's own real-time runtime instance. NOT
    # the canonical 7001/7002 (those are the external AutoHYU contract and can
    # collide with other local services, e.g. NoMachine's nxnode).
    DEFAULT = {"level": "L2", "cmd_port": COSIM_CMD_PORT, "state_port": COSIM_STATE_PORT,
               "rate": 200.0, "vx0": 0.0, "cmd_timeout": 0.1}

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.cfg = dict(self.DEFAULT)
        self.started_t = None
        self.last_state = None
        self.last_state_t = None
        self.states = {}
        self._seq = 0
        self._rx = None
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tmp = tempfile.mkdtemp(prefix="vdsim_cosim_")
        self._stop = threading.Event()
        self.attach_only = False
        self.cmd_host = "127.0.0.1"
        self._plant_log = None
        self._run_since = None
        self.kinematics_warnings = []
        self._kin_warn_pre = []

    def set_kinematics_pre_warnings(self, warnings):
        self._kin_warn_pre = list(warnings or [])

    def available(self):
        return COSIM_BIN.exists()

    def running(self):
        with self.lock:
            if self.attach_only:
                return self._rx is not None and not self._stop.is_set()
            return self.proc is not None and self.proc.poll() is None

    def _launch(self, args, state_port):
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(("127.0.0.1", int(state_port)))
        self._rx.settimeout(0.2)
        self._stop.clear()
        self.states = {}
        self.last_state = None
        if self._plant_log:
            try:
                self._plant_log.close()
            except OSError:
                pass
        log_path = os.path.join(self._tmp, "plant.log")
        self._plant_log = open(log_path, "w")
        self.kinematics_warnings = list(self._kin_warn_pre)
        self.proc = subprocess.Popen(args, stdout=self._plant_log,
                                     stderr=subprocess.STDOUT, cwd=str(REPO))
        self.started_t = time.monotonic()
        threading.Thread(target=self._rx_loop, args=(self._rx,), daemon=True).start()

    def refresh_kinematics_warnings(self):
        merged = list(self._kin_warn_pre)
        if self._plant_log is not None:
            for w in _scan_kinematics_warnings(self._plant_log.name):
                if w not in merged:
                    merged.append(w)
        self.kinematics_warnings = merged
        return self.kinematics_warnings

    def _road_cli(self, args, road, terrain, sensors, sensor_delay):
        if terrain is not None:
            tf = os.path.join(self._tmp, "terrain.bin")
            _write_terrain(tf, terrain)
            args.append(f"--terrain={tf}")
        elif road:
            for flag, key in (("--mu=", "mu"), ("--mu-right=", "mu_right"),
                              ("--mu-boundary=", "mu_boundary"), ("--grade=", "grade"),
                              ("--bank=", "bank"), ("--rough-amp=", "rough_amp"),
                              ("--rough-wl=", "rough_wl")):
                if road.get(key) is not None:
                    args.append(f"{flag}{float(road[key])}")
        if sensors is not None:
            sf = os.path.join(self._tmp, "sensors.yaml")
            sensors.to_yaml(sf)
            args.append(f"--sensors={sf}")
        if sensor_delay:
            args.append(f"--sensor-delay={float(sensor_delay)}")

    def _write_world_yaml(self, fleet, road, terrain, sensors, sensor_delay, rate, cmd_timeout):
        wy = os.path.join(self._tmp, "world.yaml")
        lines = [f"rate: {float(rate)}", f"cmd_timeout: {float(cmd_timeout)}"]
        if terrain is not None:
            tf = os.path.join(self._tmp, "terrain.bin")
            _write_terrain(tf, terrain)
            lines.append(f"terrain: {tf}")
        elif road:
            for key in ("mu", "mu_right", "mu_boundary", "grade", "bank",
                        "rough_amp", "rough_wl"):
                if road.get(key) is not None:
                    lines.append(f"{key}: {float(road[key])}")
        if sensors is not None:
            sf = os.path.join(self._tmp, "sensors.yaml")
            sensors.to_yaml(sf)
            lines.append(f"sensors: {sf}")
        if sensor_delay:
            lines.append(f"sensor_delay: {float(sensor_delay)}")
        lines.append("vehicles:")
        for e in fleet:
            lines += [
                f"  - id: {int(e['id'])}",
                f"    vehicle: {e['vehicle_yaml']}",
                f"    tire: {e['tire_yaml']}",
                f"    level: {e.get('level', 'L2')}",
                f"    x0: {float(e.get('x0', 0.0))}",
                f"    y0: {float(e.get('y0', 0.0))}",
                f"    yaw0: {float(e.get('yaw0', 0.0))}",
                f"    vx0: {float(e.get('vx0', 0.0))}",
            ]
            if e.get("front_susp_yaml"):
                lines.append(f"    front_susp: {e['front_susp_yaml']}")
            if e.get("rear_susp_yaml"):
                lines.append(f"    rear_susp: {e['rear_susp_yaml']}")
        Path(wy).write_text("\n".join(lines) + "\n")
        return wy

    def start(self, vp, tp, over, road=None, sensors=None, terrain=None,
              sensor_delay=0.0, pose=None, fleet=None):
        if self.running():
            self.stop()
        with self.lock:
            for k in self.cfg:
                if k in over:
                    self.cfg[k] = over[k]
            self.cfg["level"] = str(self.cfg["level"])
            c = self.cfg
            if fleet and len(fleet) > 1:
                wy = self._write_world_yaml(
                    fleet, road, terrain, sensors, sensor_delay,
                    c["rate"], c["cmd_timeout"])
                args = [str(COSIM_BIN), f"--scenario={wy}",
                        f"--cmd-port={int(c['cmd_port'])}", "--state-ip=127.0.0.1",
                        f"--state-port={int(c['state_port'])}",
                        f"--rate={float(c['rate'])}",
                        f"--cmd-timeout={float(c['cmd_timeout'])}"]
                self._launch(args, c["state_port"])
            else:
                vy = os.path.join(self._tmp, "vehicle.yaml")
                ty = os.path.join(self._tmp, "tire.yaml")
                vp.to_yaml(vy)
                tp.to_yaml(ty)
                args = [str(COSIM_BIN), vy, ty, f"--level={c['level']}",
                        f"--cmd-port={int(c['cmd_port'])}", "--state-ip=127.0.0.1",
                        f"--state-port={int(c['state_port'])}", f"--rate={float(c['rate'])}",
                        f"--vx0={float(c['vx0'])}", f"--cmd-timeout={float(c['cmd_timeout'])}"]
                if fleet:
                    fe = fleet[0]
                    if fe.get("front_susp_yaml"):
                        args.append(f"--front-susp={fe['front_susp_yaml']}")
                    if fe.get("rear_susp_yaml"):
                        args.append(f"--rear-susp={fe['rear_susp_yaml']}")
                self._road_cli(args, road, terrain, sensors, sensor_delay)
                if pose:
                    for flag, key in (("--x0=", "x0"), ("--y0=", "y0"), ("--yaw0=", "yaw0")):
                        if pose.get(key) is not None:
                            args.append(f"{flag}{float(pose[key])}")
                self._launch(args, c["state_port"])
        return self.status()

    def stop(self):
        with self.lock:
            self._stop.set()
            if not self.attach_only and self.proc and self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            self.proc = None
            self.attach_only = False
            self.started_t = None
            if self._rx is not None:
                try:
                    self._rx.close()
                except OSError:
                    pass
                self._rx = None
            self.last_state = None
            self.states = {}
        return self.status()

    def attach(self, host="127.0.0.1", cmd_port=COSIM_CMD_PORT,
               state_port=COSIM_STATE_PORT):
        if self.running():
            self.stop()
        local = str(host).lower() in ("127.0.0.1", "localhost", "::1")
        if local:
            _cleanup_stale_plant(int(cmd_port))
        with self.lock:
            self.attach_only = True
            self.cmd_host = str(host)
            self.cfg["cmd_port"] = int(cmd_port)
            self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            state_port = int(state_port)
            self.cfg["state_port"] = state_port
            if local:
                self._rx.bind(("127.0.0.1", state_port))
            else:
                self._rx.bind(("0.0.0.0", 0))
            self._rx.settimeout(0.2)
            self._stop.clear()
            self.states = {}
            self.last_state = None
            self.last_state_t = None
            self.started_t = time.monotonic()
            threading.Thread(target=self._rx_loop, args=(self._rx,), daemon=True).start()
        self.send_cmd(0.0, 0.0, 0.0, vehicle_id=0)
        return self.status()

    def status(self):
        run = self.running()
        if run and self._plant_log is not None:
            self.refresh_kinematics_warnings()
        return {"available": self.available(), "running": run, "attach": self.attach_only,
                "cfg": dict(self.cfg), "cmd_host": self.cmd_host,
                "pid": (self.proc.pid if run and self.proc else None),
                "uptime": (time.monotonic() - self.started_t if run and self.started_t else None),
                "state_age": (time.monotonic() - self.last_state_t if self.last_state_t else None),
                "vehicles": sorted(self.states),
                "kinematics_warnings": list(self.kinematics_warnings),
                "binary": str(COSIM_BIN)}

    def send_cmd(self, throttle, brake, steer, gear=1, vehicle_id=0):
        if not self.running():
            return
        self._seq += 1
        body = vds1.pack_cmd(self._seq, steer=steer, throttle=throttle, brake=brake,
                             gear=gear, aux_accel=0.0, aux_speed=0.0,
                             timestamp=time.time(), vehicle_id=int(vehicle_id))
        host = self.cmd_host if self.attach_only else "127.0.0.1"
        sock = self._rx if self.attach_only and self._rx else self._tx
        try:
            sock.sendto(body, (host, int(self.cfg["cmd_port"])))
        except OSError:
            pass

    def _rx_loop(self, sock):
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(512)
            except (socket.timeout, OSError):
                continue
            st = self._decode_state(data)
            if st:
                vid = int(st.get("vehicle_id", 0))
                self.states[vid] = st
                self.last_state = st
                self.last_state_t = time.monotonic()

    @staticmethod
    def _decode_state(buf):
        s = vds1.decode_state(buf)
        if s is None:
            return None
        return {"vehicle_id": s.get("vehicle_id", 0),
                "t": s["timestamp"], "x": s["x"], "y": s["y"], "z": s["z"],
                "roll": s["roll"], "pitch": s["pitch"], "yaw": s["yaw"],
                "vx": s["vx"], "vy": s["vy"], "r": s["yaw_rate"],
                "wx": s["roll_rate"], "wy": s["pitch_rate"],
                "ax": s["ax"], "ay": s["ay"],
                "steer": s["steer_applied"], "Fz": s["Fz"], "Ft": s.get("Ft", []),
                "wheel_spin": s.get("wheel_spin", []),
                "rack_torque": s["rack_torque"], "kappa": s["slip_ratio"],
                "alpha": s["slip_angle"], "susp": s["susp"],
                "m_gx": s["m_gnss_x"], "m_gy": s["m_gnss_y"], "m_ax": s["m_ax"],
                "m_ay": s["m_ay"], "m_wz": s["m_wz"], "m_steer": s["m_steer"]}


class VehiclePort:
    """Per-vehicle data-port config: telemetry output + control input.

    Keyed by vehicle id in Runner.ports. Today only the live vehicle (id 0) is
    backed by the single SimSession; the id-keyed structure lets multi-vehicle
    support later add ports/sims without reworking the wire format or GUI.
    """
    def __init__(self):
        self.tx = {"enabled": False, "rate": 50.0, "send_state": True, "send_cmd": True}
        self.targets = []                                            # output: [{ip,port}]
        self.in_cmd = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}  # control input (latched)
        self.applied = {"throttle": 0.0, "brake": 0.0, "steer": 0.0} # last applied to plant
        self.io_last_t = None
        self._tx_last = 0.0


class Runner:
    def __init__(self):
        self.lock = threading.Lock()
        self.cfg = {"level": "L2", "vehicle": "sedan", "v_target": 10.0,
                    "driver": True, "running": False,
                    "init_x": 0.0, "init_y": 0.0, "init_yaw": 0.0, "init_v": 0.0,
                    "road_mu": 1.0, "road_mu_right": -1.0, "road_boundary": 0.0,
                    "road_grade": 0.0, "road_bank": 0.0,
                    "road_rough_amp": 0.0, "road_rough_wl": 4.0,
                    "cosim_attach": False, "cosim_host": "127.0.0.1",
                    "cosim_cmd_port": 7401}
        self.dt = 0.005
        self.time_scale = 1.0
        self.live_vid = 0
        self.ports = {0: VehiclePort()}
        _p0 = suspension_default_for_vehicle("sedan")
        self.fleet_spec = [
            {"id": 0, "vehicle": "sedan", "tire": "default_pacejka", "level": "L2",
             "x0": 0.0, "y0": 0.0, "yaw0": 0.0, "vx0": 0.0,
             "front_susp": _p0["front"], "rear_susp": _p0["rear"]},
        ]
        self.fleet_overrides = {}
        self.plant_error = None
        self._wait_since = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cosim = CosimBridge()         # binary co-sim server (configured + launched here)
        self.rec_on = False                # logging recorder
        self.rec_rows = []
        self.rec_last = {}
        self.latest = {}
        self.path = FigureEight()
        self.vp = vdsim.VehicleParams.from_yaml(
            str(REPO / "configs/vehicles/sedan.yaml"))
        self.tp = vdsim.TireParams.from_yaml(
            str(REPO / "configs/tires/default_pacejka.yaml"))
        self.act = vdsim.ActuatorParams()        # all effects off by default
        self.sensors = vdsim.SensorParams()      # disabled -> measured == truth
        self.solver = vdsim.SolverParams()
        self.sensor_delay = 0.0
        self.terrain = None                      # baked heightmap terrain (or None)
        self.scenery = None                      # parsed building meshes (or None)
        self.tex_dir = None                      # dir holding building textures
        self.path_preset = "figure8"
        self.prev_idx = 0
        self.infra_sensors = []
        threading.Thread(target=self._loop, daemon=True).start()

    def _ensure_ports(self):
        for spec in self.fleet_spec:
            vid = int(spec["id"])
            if vid not in self.ports:
                self.ports[vid] = VehiclePort()

    @staticmethod
    def _ensure_fleet_parts(spec):
        veh = str(spec.get("vehicle", "sedan"))
        defaults = suspension_default_for_vehicle(veh)
        spec.setdefault("front_susp", defaults["front"])
        spec.setdefault("rear_susp", defaults["rear"])

    @staticmethod
    def _susp_yaml_path(stem):
        if not stem:
            return None
        p = SUSP_DIR / f"{Path(str(stem)).stem}.yaml"
        return str(p) if p.is_file() else None

    def _renumber_fleet(self):
        want = list(range(len(self.fleet_spec)))
        have = [int(s["id"]) for s in self.fleet_spec]
        if have == want:
            return
        mapping = {old: i for i, old in enumerate(have)}
        new_spec = []
        for i, s in enumerate(self.fleet_spec):
            ns = dict(s)
            ns["id"] = i
            new_spec.append(ns)
        new_overrides = {mapping[k]: v for k, v in self.fleet_overrides.items()
                         if k in mapping}
        new_ports = {mapping[k]: v for k, v in self.ports.items() if k in mapping}
        self.fleet_spec = new_spec
        self.fleet_overrides = new_overrides
        self.ports = new_ports
        self.live_vid = mapping.get(self.live_vid, 0)
        self._ensure_ports()
        self._sync_live_from_fleet()

    def _spec_for_vid(self, vid):
        vid = int(vid)
        spec = next((f for f in self.fleet_spec if int(f["id"]) == vid), None)
        if spec is None:
            raise ValueError(f"unknown vehicle id {vid}")
        return spec

    def _vp_for_vid(self, vid):
        vid = int(vid)
        if vid == self.live_vid:
            return self.vp
        spec = self._spec_for_vid(vid)
        vp = vdsim.VehicleParams.from_yaml(
            str(REPO / f"configs/vehicles/{spec['vehicle']}.yaml"))
        ov = self.fleet_overrides.get(vid, {}).get("vehicle")
        if ov:
            _apply(vp, VEHICLE_FIELDS, ov)
        return vp

    def _tp_for_vid(self, vid):
        vid = int(vid)
        if vid == self.live_vid:
            return self.tp
        spec = self._spec_for_vid(vid)
        tp = vdsim.TireParams.from_yaml(
            str(REPO / f"configs/tires/{spec['tire']}.yaml"))
        ov = self.fleet_overrides.get(vid, {}).get("tire")
        if ov:
            _apply(tp, TIRE_FIELDS, ov)
        return tp

    def vehicle_geom(self, vid):
        vp = self._vp_for_vid(vid)
        return {"cg_to_front": float(vp.cg_to_front), "cg_to_rear": float(vp.cg_to_rear),
                "track_front": float(vp.track_front), "track_rear": float(vp.track_rear),
                "wheel_radius_nominal": float(vp.wheel_radius_nominal)}

    def _fleet_launch(self):
        fleet = []
        for spec in self.fleet_spec:
            self._ensure_fleet_parts(spec)
            vid = int(spec["id"])
            vy = os.path.join(self.cosim._tmp, f"vehicle_{vid}.yaml")
            ty = os.path.join(self.cosim._tmp, f"tire_{vid}.yaml")
            self._vp_for_vid(vid).to_yaml(vy)
            self._tp_for_vid(vid).to_yaml(ty)
            row = {
                "id": vid,
                "vehicle_yaml": vy,
                "tire_yaml": ty,
                "level": spec.get("level", self.cfg["level"]),
                "x0": float(spec.get("x0", self.cfg["init_x"])),
                "y0": float(spec.get("y0", self.cfg["init_y"])),
                "yaw0": float(spec.get("yaw0", self.cfg["init_yaw"])),
                "vx0": float(spec.get("vx0", self.cfg["init_v"])),
            }
            if str(spec.get("level", self.cfg["level"])) == "L3":
                fs = self._susp_yaml_path(spec.get("front_susp"))
                rs = self._susp_yaml_path(spec.get("rear_susp"))
                if fs:
                    row["front_susp_yaml"] = fs
                if rs:
                    row["rear_susp_yaml"] = rs
            fleet.append(row)
        return fleet

    def _sync_live_from_fleet(self):
        spec = self._spec_for_vid(self.live_vid)
        self.cfg["init_x"] = float(spec.get("x0", 0.0))
        self.cfg["init_y"] = float(spec.get("y0", 0.0))
        self.cfg["init_yaw"] = float(spec.get("yaw0", 0.0))
        self.cfg["init_v"] = float(spec.get("vx0", 0.0))
        if spec.get("level"):
            self.cfg["level"] = str(spec["level"])
        if spec.get("vehicle"):
            self.cfg["vehicle"] = str(spec["vehicle"])
            self.load_vehicle(self.cfg["vehicle"])
        tire_stem = str(spec.get("tire", ""))
        if tire_stem:
            tp = vdsim.TireParams.from_yaml(
                str(REPO / f"configs/tires/{tire_stem}.yaml"))
            ov = self.fleet_overrides.get(self.live_vid, {}).get("tire")
            if ov:
                _apply(tp, TIRE_FIELDS, ov)
            self.tp = tp

    def load_fleet_scenario(self, name):
        path = REPO / "configs" / "scenarios" / f"{name}.yaml"
        if not path.is_file():
            raise ValueError(f"unknown scenario: {name}")
        import yaml
        doc = yaml.safe_load(path.read_text())
        self.fleet_overrides = {}
        self.fleet_spec = []
        for v in doc.get("vehicles", []):
            veh = Path(str(v["vehicle"]))
            tire = Path(str(v["tire"]))
            if not veh.is_absolute():
                veh = REPO / veh
            if not tire.is_absolute():
                tire = REPO / tire
            parts = suspension_default_for_vehicle(veh.stem)
            self.fleet_spec.append({
                "id": int(v["id"]),
                "vehicle": veh.stem,
                "tire": tire.stem,
                "level": str(v.get("level", "L2")),
                "x0": float(v.get("x0", 0.0)),
                "y0": float(v.get("y0", 0.0)),
                "yaw0": float(v.get("yaw0", 0.0)),
                "vx0": float(v.get("vx0", 0.0)),
                "front_susp": susp_stem_from_ref(v.get("front_susp", parts["front"])),
                "rear_susp": susp_stem_from_ref(v.get("rear_susp", parts["rear"])),
            })
            _strip_fleet_susp_if_not_l3(self.fleet_spec[-1])
        self._ensure_ports()
        if self.fleet_spec:
            s0 = self.fleet_spec[0]
            self.live_vid = int(s0["id"])
            self.cfg["init_x"] = float(s0.get("x0", 0.0))
            self.cfg["init_y"] = float(s0.get("y0", 0.0))
            self.cfg["init_yaw"] = float(s0.get("yaw0", 0.0))
            self.cfg["init_v"] = float(s0.get("vx0", 0.0))
            if s0.get("level"):
                self.cfg["level"] = str(s0["level"])
            self._sync_live_from_fleet()
        if doc.get("path_preset"):
            self.set_path_preset(str(doc["path_preset"]))
        elif doc.get("path_pts"):
            pts = [(float(p[0]), float(p[1])) for p in doc["path_pts"]]
            if len(pts) >= 2:
                self.path = WaypointPath(pts)
                self.path_preset = "custom"
        for key, ck in (("mu", "road_mu"), ("grade", "road_grade"), ("bank", "road_bank"),
                        ("v_target", "v_target")):
            if doc.get(key) is not None:
                self.cfg[ck] = float(doc[key])
        if doc.get("road"):
            r = doc["road"]
            for k, ck in (("mu", "road_mu"), ("grade", "road_grade"), ("bank", "road_bank")):
                if k in r:
                    self.cfg[ck] = float(r[k])
        if doc.get("infra_sensors") is not None:
            self.infra_sensors = list(doc["infra_sensors"])

    def _build(self):
        self._ensure_ports()
        over = {"level": self.cfg["level"], "vx0": max(0.0, self.cfg["init_v"]),
                "rate": (1.0 / self.dt if self.dt > 1e-6 else 200.0)}
        road = {"mu": self.cfg["road_mu"], "mu_right": self.cfg["road_mu_right"],
                "mu_boundary": self.cfg["road_boundary"], "grade": self.cfg["road_grade"],
                "bank": self.cfg["road_bank"], "rough_amp": self.cfg["road_rough_amp"],
                "rough_wl": self.cfg["road_rough_wl"]}
        pose = {"x0": self.cfg["init_x"], "y0": self.cfg["init_y"], "yaw0": self.cfg["init_yaw"]}
        sensors = self.sensors if self.sensors.enabled else None
        if self.cfg.get("cosim_attach"):
            self.cosim.attach(self.cfg.get("cosim_host", "127.0.0.1"),
                              int(self.cfg.get("cosim_cmd_port", COSIM_CMD_PORT)),
                              int(self.cfg.get("cosim_state_port", COSIM_STATE_PORT)))
            self.prev_idx = 0
            return
        _cleanup_stale_plant(int(self.cosim.cfg.get("cmd_port", COSIM_CMD_PORT)))
        fleet = self._fleet_launch()
        self.cosim.set_kinematics_pre_warnings(_l3_susp_path_warnings(self.fleet_spec))
        if self.cosim.available():
            self.cosim.start(self.vp, self.tp, over, road=road, sensors=sensors,
                             terrain=self.terrain, sensor_delay=self.sensor_delay,
                             pose=pose, fleet=fleet)
        else:
            print("[VDSim GUI] vdsim_realtime binary missing — build the C++ tree first")
        self.prev_idx = 0

    def _rebuild_if_running(self):
        if self.cfg["running"]:
            self._build()

    def load_vehicle(self, name):
        self.vp = vdsim.VehicleParams.from_yaml(
            str(REPO / f"configs/vehicles/{name}.yaml"))

    def reconfigure(self, **kw):
        with self.lock:
            if "vehicle" in kw and kw["vehicle"] != self.cfg["vehicle"]:
                self.load_vehicle(kw["vehicle"])
            for k in ("level", "vehicle", "v_target", "driver"):
                if k in kw:
                    self.cfg[k] = kw[k]
            self._rebuild_if_running()

    def set_sim(self, dt=None, time_scale=None, integrator=None, max_substeps=None,
                **init):
        with self.lock:
            if dt is not None and dt > 1e-5:
                self.dt = float(dt)
            if time_scale is not None and time_scale > 0:
                self.time_scale = float(time_scale)
            rebuild = False
            if integrator in ENUM_MAPS["integrator"]:
                self.solver.integrator = ENUM_MAPS["integrator"][integrator]
                rebuild = True
            if max_substeps is not None:
                self.solver.max_substeps = max(1, int(float(max_substeps)))
                rebuild = True
            for k in ("init_x", "init_y", "init_yaw", "init_v",
                      "road_mu", "road_mu_right", "road_boundary",
                      "road_grade", "road_bank", "road_rough_amp", "road_rough_wl"):
                if init.get(k) is not None:
                    self.cfg[k] = float(init[k])
                    rebuild = True
            if rebuild:
                self._rebuild_if_running()

    def set_params(self, which, data, vehicle_id=None):
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            if vid == self.live_vid:
                if which == "vehicle":
                    _apply(self.vp, VEHICLE_FIELDS, data)
                elif which == "tire":
                    _apply(self.tp, TIRE_FIELDS, data)
                elif which == "sensors":
                    self._apply_sensors(data)
                else:
                    self._apply_actuator(data)
            elif which in ("vehicle", "tire"):
                bucket = self.fleet_overrides.setdefault(vid, {}).setdefault(which, {})
                bucket.update(data)
            self._rebuild_if_running()

    def serialize_vehicle(self, vehicle_id=None):
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            return _serialize(self._vp_for_vid(vid), VEHICLE_FIELDS)

    def serialize_tire(self, vehicle_id=None):
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            return _serialize(self._tp_for_vid(vid), TIRE_FIELDS)

    def fleet_enriched(self):
        with self.lock:
            out = []
            for spec in self.fleet_spec:
                vid = int(spec["id"])
                row = dict(spec)
                self._ensure_fleet_parts(row)
                row["geom"] = self.vehicle_geom(vid)
                out.append(row)
            return out

    def serialize_sensors(self):
        out = []
        for attr, label, group, kind in SENSOR_FIELDS:
            if kind == "bool":
                val = bool(getattr(self.sensors, attr))
            else:
                val = float(_get_dotted(self.sensors, attr))
            out.append({"name": attr, "label": label, "group": group,
                        "kind": kind, "levels": ["K", "L1", "L2", "L3"], "value": val})
        return out

    def _apply_sensors(self, data):
        kinds = {f[0]: f[3] for f in SENSOR_FIELDS}
        for k, v in data.items():
            kind = kinds.get(k)
            if kind is None:
                continue
            if kind == "bool":
                setattr(self.sensors, k, bool(v))
            else:
                _set_dotted(self.sensors, k, float(v))

    def serialize_actuator(self):
        out = []
        for attr, label, group, kind in ACTUATOR_FIELDS:
            if attr == "@sensor_delay_s":
                val = float(self.sensor_delay)
            elif kind == "bool":
                val = bool(_get_dotted(self.act, attr))
            else:
                val = float(_get_dotted(self.act, attr))
            out.append({"name": attr, "label": label, "group": group,
                        "kind": kind, "levels": ["K", "L1", "L2", "L3"],
                        "value": val})
        return out

    def tire_curves(self, vehicle_id=None):
        with self.lock:
            vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
            tp = self._tp_for_vid(vid)
        return self._tire_plots(tp)

    def _tire_plots(self, tp):
        t = vdsim.create_pacejka_mf96()
        t.initialize(tp)
        Fz0, mu = tp.Fz_nominal, tp.mu_nominal

        def F(kappa, alpha, Fz):
            inp = vdsim.TireInput()
            inp.Fz, inp.kappa, inp.alpha = Fz, kappa, alpha
            inp.mu_long = inp.mu_lat = mu
            inp.Vx_wheel, inp.gamma = 15.0, 0.0
            o = t.compute(inp)
            return o.Fx, o.Fy

        ks = [i / 100.0 for i in range(-25, 26)]
        deg = 57.29578
        loads = [(0.5, "#9bbcff"), (1.0, "#01A0E9"), (1.5, "#002060")]
        sx = [{"label": f"{int(L*100)}% Fz", "color": c, "x": ks,
               "y": [F(k, 0.0, Fz0 * L)[0] for k in ks]} for L, c in loads]
        sy = [{"label": f"{int(L*100)}% Fz", "color": c, "x": [a * deg for a in ks],
               "y": [-F(0.0, a, Fz0 * L)[1] for a in ks]} for L, c in loads]
        kappas = [(0.0, "#002060"), (0.05, "#01A0E9"), (0.10, "#f5a623"), (0.20, "#DC291E")]
        sc = [{"label": f"κ={k:g}", "color": c, "x": [a * deg for a in ks],
               "y": [-F(k, a, Fz0)[1] for a in ks]} for k, c in kappas]
        return [
            {"title": "Longitudinal Fx(κ)", "xlabel": "slip ratio κ", "ylabel": "Fx [N]", "series": sx},
            {"title": "Lateral Fy(α)", "xlabel": "slip angle α [deg]", "ylabel": "Fy [N]", "series": sy},
            {"title": "Combined slip — Fy(α) at fixed κ", "xlabel": "slip angle α [deg]", "ylabel": "Fy [N]", "series": sc},
        ]

    def actuator_step(self):
        with self.lock:
            act = self.act
        chans = [("steer", 0.3, "rad", "#01A0E9"),
                 ("throttle", 1.0, "", "#34c759"),
                 ("brake", 1.0, "", "#DC291E")]
        plots = []
        for ch, amp, unit, col in chans:
            r = vdsim.actuator_step_response(act, ch, amp, 0.002, 0.8, 15.0)
            plots.append({"title": f"{ch} step → {amp}{unit}", "xlabel": "t [s]", "ylabel": ch,
                          "series": [
                              {"label": "cmd", "color": "#b6c2cf", "dash": True,
                               "x": list(r["t"]), "y": list(r["cmd"])},
                              {"label": "realized", "color": col,
                               "x": list(r["t"]), "y": list(r["out"])}]})
        return plots

    def _apply_actuator(self, data):
        kinds = {f[0]: f[3] for f in ACTUATOR_FIELDS}
        for k, v in data.items():
            kind = kinds.get(k)
            if kind is None:
                continue
            if k == "@sensor_delay_s":
                self.sensor_delay = max(0.0, float(v))
            elif kind == "bool":
                _set_dotted(self.act, k, bool(v))
            else:
                _set_dotted(self.act, k, float(v))

    def set_path_preset(self, name):
        self.path_preset = name
        if name == "figure8":
            self.path = FigureEight()
        elif name == "straight":
            self.path = WaypointPath([(-40.0, 0.0), (40.0, 0.0)])

    def get_setup(self):
        with self.lock:
            road = {"mu": self.cfg["road_mu"], "mu_right": self.cfg["road_mu_right"],
                    "mu_boundary": self.cfg["road_boundary"], "grade": self.cfg["road_grade"],
                    "bank": self.cfg["road_bank"], "rough_amp": self.cfg["road_rough_amp"],
                    "rough_wl": self.cfg["road_rough_wl"]}
            fleet = []
            for spec in self.fleet_spec:
                vid = int(spec["id"])
                row = dict(spec)
                self._ensure_fleet_parts(row)
                row["geom"] = self.vehicle_geom(vid)
                fleet.append(row)
            return {
                "running": self.cfg["running"],
                "path_preset": self.path_preset,
                "path_pts": [[float(p[0]), float(p[1])] for p in self.path.pts],
                "fleet": fleet,
                "road": road,
                "v_target": self.cfg["v_target"],
                "level": self.cfg["level"],
                "vehicle": self.cfg["vehicle"],
                "driver": self.cfg["driver"],
                "cosim_attach": self.cfg.get("cosim_attach", False),
                "cosim_host": self.cfg.get("cosim_host", "127.0.0.1"),
                "cosim_cmd_port": int(self.cfg.get("cosim_cmd_port", 7401)),
                "scenarios": self.list_scenarios(),
                "infra_sensors": list(self.infra_sensors),
            }

    def _fleet_add(self):
        ids = [int(s["id"]) for s in self.fleet_spec]
        nid = (max(ids) + 1) if ids else 0
        ref = self.fleet_spec[-1]
        veh = str(ref.get("vehicle", "sedan"))
        parts = suspension_default_for_vehicle(veh)
        self.fleet_spec.append({
            "id": nid,
            "vehicle": veh,
            "tire": str(ref.get("tire", "default_pacejka")),
            "level": str(ref.get("level", self.cfg["level"])),
            "x0": float(ref.get("x0", 0.0)) + 3.0,
            "y0": float(ref.get("y0", 0.0)),
            "yaw0": float(ref.get("yaw0", 0.0)),
            "vx0": float(ref.get("vx0", 0.0)),
            "front_susp": str(ref.get("front_susp", parts["front"])),
            "rear_susp": str(ref.get("rear_susp", parts["rear"])),
        })
        _strip_fleet_susp_if_not_l3(self.fleet_spec[-1])
        self._ensure_ports()

    def _fleet_ids(self):
        return {int(s["id"]) for s in self.fleet_spec}

    def _prune_cosim_states(self):
        allowed = self._fleet_ids()
        for k in list(self.cosim.states.keys()):
            if int(k) not in allowed:
                self.cosim.states.pop(k, None)
        if (self.cosim.last_state is not None
                and int(self.cosim.last_state.get("vehicle_id", 0)) not in allowed):
            self.cosim.last_state = None
            self.cosim.last_state_t = None

    def _fleet_remove(self, vid):
        if len(self.fleet_spec) <= 1:
            return
        vid = int(vid)
        self.fleet_spec = [s for s in self.fleet_spec if int(s["id"]) != vid]
        self.fleet_overrides.pop(vid, None)
        self.ports.pop(vid, None)
        if self.live_vid == vid:
            s0 = self.fleet_spec[0]
            self.live_vid = int(s0["id"])
        self._renumber_fleet()
        self._prune_cosim_states()

    def list_scenarios(self):
        d = REPO / "configs" / "scenarios"
        if not d.is_dir():
            return []
        return sorted(p.stem for p in d.glob("*.yaml"))

    def save_scenario(self, name):
        import yaml
        stem = Path(str(name)).stem
        if not stem:
            raise ValueError("scenario name required")
        path = REPO / "configs" / "scenarios" / f"{stem}.yaml"
        vehs = []
        for s in self.fleet_spec:
            row = {
                "id": int(s["id"]),
                "vehicle": f"configs/vehicles/{s['vehicle']}.yaml",
                "tire": f"configs/tires/{s['tire']}.yaml",
                "level": str(s.get("level", "L2")),
                "x0": float(s.get("x0", 0.0)),
                "y0": float(s.get("y0", 0.0)),
                "yaw0": float(s.get("yaw0", 0.0)),
                "vx0": float(s.get("vx0", 0.0)),
            }
            if str(s.get("level", "L2")) == "L3":
                fp = susp_rel_path(s.get("front_susp"))
                rp = susp_rel_path(s.get("rear_susp"))
                if fp:
                    row["front_susp"] = fp
                if rp:
                    row["rear_susp"] = rp
            vehs.append(row)
        doc = {
            "name": stem,
            "rate": 200,
            "cmd_timeout": 0.1,
            "mu": float(self.cfg["road_mu"]),
            "grade": float(self.cfg["road_grade"]),
            "bank": float(self.cfg["road_bank"]),
            "v_target": float(self.cfg["v_target"]),
            "path_preset": self.path_preset,
            "vehicles": vehs,
        }
        if self.path_preset == "custom":
            doc["path_pts"] = [[float(p[0]), float(p[1])] for p in self.path.pts]
        if self.infra_sensors:
            doc["infra_sensors"] = list(self.infra_sensors)
        path.write_text(yaml.safe_dump(doc, sort_keys=False))
        return {"ok": True, "name": stem}

    def apply_setup(self, data):
        refresh_snap = False
        with self.lock:
            if data.get("fleet_add"):
                self._fleet_add()
                refresh_snap = True
            if data.get("fleet_remove") is not None:
                self._fleet_remove(int(data["fleet_remove"]))
                refresh_snap = True
            if "cosim_attach" in data:
                self.cfg["cosim_attach"] = bool(data["cosim_attach"])
            if "cosim_host" in data:
                self.cfg["cosim_host"] = str(data["cosim_host"])
            if "cosim_cmd_port" in data:
                self.cfg["cosim_cmd_port"] = int(data["cosim_cmd_port"])
            preset = str(data.get("path_preset") or "")
            if preset and preset != "custom":
                self.set_path_preset(preset)
            elif "path_pts" in data and data["path_pts"]:
                pts = [(float(p[0]), float(p[1])) for p in data["path_pts"]]
                if len(pts) >= 2:
                    self.path = WaypointPath(pts)
                    self.path_preset = "custom"
            if "fleet" in data:
                for upd in data["fleet"]:
                    vid = int(upd["id"])
                    spec = next((f for f in self.fleet_spec if int(f["id"]) == vid), None)
                    if spec is None:
                        continue
                    for k in ("x0", "y0", "yaw0", "vx0", "level", "vehicle", "tire",
                              "front_susp", "rear_susp"):
                        if k in upd:
                            spec[k] = upd[k]
                    if "vehicle" in upd and "front_susp" not in upd and "rear_susp" not in upd:
                        d = suspension_default_for_vehicle(str(upd["vehicle"]))
                        spec["front_susp"] = d["front"]
                        spec["rear_susp"] = d["rear"]
                    if "level" in upd and str(upd["level"]) == "L3":
                        self._ensure_fleet_parts(spec)
                    _strip_fleet_susp_if_not_l3(spec)
                    if vid == self.live_vid:
                        self._sync_live_from_fleet()
                refresh_snap = True
            if "road" in data:
                r = data["road"]
                for k, ck in (("mu", "road_mu"), ("mu_right", "road_mu_right"),
                              ("mu_boundary", "road_boundary"), ("grade", "road_grade"),
                              ("bank", "road_bank"), ("rough_amp", "road_rough_amp"),
                              ("rough_wl", "road_rough_wl")):
                    if k in r:
                        self.cfg[ck] = float(r[k])
            if "infra_sensors" in data:
                self.infra_sensors = list(data["infra_sensors"] or [])
                refresh_snap = True
            if "v_target" in data:
                self.cfg["v_target"] = float(data["v_target"])
            if "level" in data:
                self.cfg["level"] = str(data["level"])
            if "vehicle" in data:
                self.cfg["vehicle"] = str(data["vehicle"])
                self.load_vehicle(self.cfg["vehicle"])
            if refresh_snap and not self.cfg["running"]:
                self.latest = self._setup_snapshot()

    def _setup_snapshot(self):
        fleet = {}
        for spec in self.fleet_spec:
            vid = str(int(spec["id"]))
            fleet[vid] = {
                "x": float(spec.get("x0", 0.0)),
                "y": float(spec.get("y0", 0.0)),
                "z": 0.0,
                "yaw": float(spec.get("yaw0", 0.0)),
                "roll": 0.0, "pitch": 0.0,
                "vx": float(spec.get("vx0", 0.0)),
                "vy": 0.0,
                "level": spec.get("level", self.cfg["level"]),
                "vehicle": spec.get("vehicle", ""),
            }
        live = fleet.get(str(self.live_vid), {})
        return {
            "t": 0.0, "running": False, "setup_mode": True,
            "driver": self.cfg["driver"],
            "x": live.get("x", 0.0), "y": live.get("y", 0.0), "z": 0.0,
            "yaw": live.get("yaw", 0.0), "roll": 0.0, "pitch": 0.0,
            "vx": 0.0, "vy": 0.0, "r": 0.0,
            "wx": 0.0, "wy": 0.0, "ax": 0.0, "ay": 0.0,
            "steer": 0.0, "Fz": [0.0, 0.0, 0.0, 0.0],
            "Ft": [], "wheel_spin": [], "susp": [], "rack_torque": 0.0,
            "kappa": [], "alpha": [],
            "level": self.cfg["level"], "vehicle": self.cfg["vehicle"],
            "v_target": self.cfg["v_target"], "dt": self.dt,
            "time_scale": self.time_scale, "source": "setup",
            "fleet": fleet, "live_vid": self.live_vid,
            "npath": len(self.path.pts), "terrain": 1 if self.terrain else 0,
            "grade": self.cfg["road_grade"], "bank": self.cfg["road_bank"],
        }

    def control(self, action):
        start_req = False
        with self.lock:
            if action == "reset":
                self.cfg["running"] = False
                self.plant_error = None
                self.cosim.stop()
                self.prev_idx = 0
            elif action == "stop":
                self.cfg["running"] = False
                self.plant_error = None
                self.cosim.stop()
            elif action == "start":
                self.cosim.stop()
                self._renumber_fleet()
                self._sync_live_from_fleet()
                self._prune_cosim_states()
                self.prev_idx = 0
                self.plant_error = None
                if not self.cfg.get("cosim_attach") and not self.cosim.available():
                    self.cfg["running"] = False
                    self.plant_error = "vdsim_realtime not built (cmake -DVDSIM_BUILD_COSIM=ON)"
                else:
                    start_req = True
        if start_req:
            t0 = time.monotonic()
            self._build()
            deadline = t0 + 2.5
            while time.monotonic() < deadline:
                if (self.cosim.last_state is not None and self.cosim.last_state_t
                        and self.cosim.last_state_t >= t0 - 0.05):
                    break
                if not self.cfg.get("cosim_attach") and not self.cosim.running():
                    break
                time.sleep(0.05)
            with self.lock:
                ok = self.cosim.last_state is not None
                self.cfg["running"] = ok
                if ok:
                    self._run_since = time.monotonic()
                    self._wait_since = None
                    self.cosim.refresh_kinematics_warnings()
                else:
                    hint = "runtime did not respond"
                    if self.cfg.get("cosim_attach"):
                        hint += " — uncheck Attach external or start vdsim_realtime on that host"
                    else:
                        hint += " — orphan vdsim_realtime on 7401? (server now auto-cleans on Play)"
                    self.plant_error = hint
                    self.cosim.stop()
        with self.lock:
            return {"running": self.cfg["running"], "error": self.plant_error}

    def set_manual(self, **kw):
        with self.lock:
            self.ports[self.live_vid].in_cmd.update(
                {k: float(v) for k, v in kw.items()
                 if k in ("throttle", "brake", "steer")})
            self.cfg["driver"] = False     # wheel/pedal takes over from autopilot

    def telemetry_config(self):
        with self.lock:
            return {"vehicles": sorted(self.ports), "live": self.live_vid,
                    "configs": {vid: {"tx": dict(p.tx),
                                      "targets": [dict(t) for t in p.targets]}
                                for vid, p in self.ports.items()}}

    def set_telemetry(self, data):
        vid = int(data.get("vehicle", self.live_vid))
        with self.lock:
            p = self.ports.get(vid)
            if p is None:
                return
            for k in ("enabled", "send_state", "send_cmd"):
                if k in data:
                    p.tx[k] = bool(data[k])
            if "rate" in data:
                p.tx["rate"] = max(1.0, min(200.0, float(data["rate"])))
            if "targets" in data:
                clean = []
                for t in data["targets"]:
                    ip = str(t.get("ip", "")).strip()
                    try:
                        port = int(t.get("port", 0))
                    except (TypeError, ValueError):
                        continue
                    if ip and 0 < port < 65536:
                        clean.append({"ip": ip, "port": port})
                p.targets = clean

    def _telemetry_send(self, snap):
        # Called from the sim loop; per-vehicle paced fan-out. Only the live
        # vehicle has plant state today; other ids (future) send their own.
        now = time.monotonic()
        for vid, p in self.ports.items():
            if not (p.tx["enabled"] and p.targets):
                continue
            if now - p._tx_last < 1.0 / p.tx["rate"]:
                continue
            p._tx_last = now
            live = (vid == self.live_vid)
            payload = {"veh": vid, "t": snap.get("t", 0.0) if live else 0.0}
            if p.tx["send_state"] and live:
                for k in ("x", "y", "z", "yaw", "roll", "pitch", "vx", "vy", "r",
                          "wx", "wy", "ax", "ay", "steer", "Fz"):
                    if k in snap:
                        payload[k] = snap[k]
            if p.tx["send_cmd"]:
                payload["cmd"] = p.applied if live else p.in_cmd
            data = json.dumps(payload).encode()
            for t in p.targets:
                try:
                    self._sock.sendto(data, (t["ip"], t["port"]))
                except OSError:
                    pass

    def io(self, cmd):
        # External data port: route command to the addressed vehicle's input,
        # mark the connection live. Returns the current (live) state snapshot.
        vid = int(cmd.get("vehicle", self.live_vid))
        with self.lock:
            p = self.ports.get(vid)
            if p is None:
                return {"error": f"unknown vehicle {vid}", "vehicles": sorted(self.ports)}
            for k in ("throttle", "brake", "steer"):
                if k in cmd:
                    p.in_cmd[k] = float(cmd[k])
            p.io_last_t = time.monotonic()
            if vid == self.live_vid:
                self.cfg["driver"] = False     # external command drives the plant
        return self.snapshot()

    def _loop(self):
        fps = 60.0
        pending = 0.0
        nxt = time.monotonic()
        while True:
            with self.lock:
                run, driver = self.cfg["running"], self.cfg["driver"]
                vt, dt, ts = self.cfg["v_target"], self.dt, self.time_scale
                man = dict(self.ports[self.live_vid].in_cmd)
                wb = self.vp.wheelbase
            if not run:
                with self.lock:
                    self._wait_since = None
                    self._run_since = None
                snap = self._setup_snapshot()
                with self.lock:
                    self.latest = snap
                nxt += 1.0 / fps
                time.sleep(max(0.0, nxt - time.monotonic()))
                continue
            cosim_on = self.cosim.running()
            if not cosim_on:
                with self.lock:
                    self.cfg["running"] = False
                    if not self.plant_error:
                        self.plant_error = "plant exited — press ▶ Play again"
                    self._wait_since = None
                self.cosim.stop()
                snap = self._setup_snapshot()
                snap["setup_mode"] = True
                snap["plant_error"] = self.plant_error
                with self.lock:
                    self.latest = snap
                nxt += 1.0 / fps
                time.sleep(max(0.0, nxt - time.monotonic()))
                continue
            self._prune_cosim_states()
            allowed = self._fleet_ids()
            all_cs = ({k: v for k, v in self.cosim.states.items() if int(k) in allowed}
                      if cosim_on else {})
            cs = all_cs.get(self.live_vid) if cosim_on else None
            if cs is None and cosim_on and self.cosim.last_state is not None:
                lv = int(self.cosim.last_state.get("vehicle_id", self.live_vid))
                if lv == self.live_vid and lv in allowed:
                    cs = self.cosim.last_state
            if cosim_on and cs:
                # The real-time server (vdsim_realtime) is the plant. Build the
                # command (autopilot uses the server's state for feedback) and
                # relay it as a binary CMD. The GUI owns no sim — it only drives
                # and renders the one real-time runtime.
                if driver:
                    st, self.prev_idx = self.path.steer(
                        cs["x"], cs["y"], cs["yaw"], cs["vx"], wb, self.prev_idx)
                    ax = max(-3.0, min(3.0, 0.8 * (vt - cs["vx"])))
                    t = min(1.0, ax / 3.0) if ax >= 0 else 0.0
                    b = min(1.0, -ax / 3.0) if ax < 0 else 0.0
                else:
                    t, b, st = man["throttle"], man["brake"], man["steer"]
                self.cosim.send_cmd(t, b, st, vehicle_id=self.live_vid)
                self.ports[self.live_vid].applied = {"throttle": t, "brake": b, "steer": st}
            fleet = {}
            if all_cs:
                for vid, st in all_cs.items():
                    spec = next((f for f in self.fleet_spec if int(f["id"]) == int(vid)), {})
                    fleet[str(vid)] = {
                        "x": st["x"], "y": st["y"], "z": st["z"],
                        "yaw": st["yaw"], "roll": st["roll"], "pitch": st["pitch"],
                        "vx": st["vx"], "vy": st["vy"],
                        "level": spec.get("level", self.cfg["level"]),
                        "vehicle": spec.get("vehicle", ""),
                    }
            if cs:
                with self.lock:
                    self._wait_since = None
                snap = {"t": cs["t"], "running": run, "setup_mode": False, "driver": driver,
                        "x": cs["x"], "y": cs["y"], "z": cs["z"],
                        "yaw": cs["yaw"], "roll": cs["roll"], "pitch": cs["pitch"],
                        "vx": cs["vx"], "vy": cs["vy"], "r": cs["r"],
                        "wx": cs["wx"], "wy": cs["wy"], "ax": cs["ax"], "ay": cs["ay"],
                        "steer": cs["steer"], "Fz": cs["Fz"], "Ft": cs.get("Ft", []),
                        "wheel_spin": cs.get("wheel_spin", []),
                        "susp": cs.get("susp", []),
                        "rack_torque": cs.get("rack_torque", 0.0),
                        "kappa": cs.get("kappa", []), "alpha": cs.get("alpha", []),
                        "m_gx": cs.get("m_gx", 0.0), "m_gy": cs.get("m_gy", 0.0),
                        "m_ax": cs.get("m_ax", 0.0), "m_ay": cs.get("m_ay", 0.0),
                        "m_wz": cs.get("m_wz", 0.0), "m_steer": cs.get("m_steer", 0.0),
                        "level": self.cosim.cfg["level"], "vehicle": self.cfg["vehicle"],
                        "v_target": vt, "dt": 1.0 / max(1.0, self.cosim.cfg["rate"]),
                        "time_scale": 1.0, "source": "cosim",
                        "fleet": fleet if all_cs else {}}
            else:
                now = time.monotonic()
                err = None
                with self.lock:
                    if self._wait_since is None:
                        self._wait_since = now
                    waited = now - (self._wait_since or now)
                    if waited > 4.0:
                        if not self.plant_error:
                            self.plant_error = (
                                "no STATE from plant — uncheck Attach external, "
                                "then ▶ Play again")
                        self.cfg["running"] = False
                        self._wait_since = None
                        err = self.plant_error
                if err is not None:
                    self.cosim.stop()
                    snap = self._setup_snapshot()
                    snap["plant_error"] = err
                    with self.lock:
                        self.latest = snap
                    nxt += 1.0 / fps
                    time.sleep(max(0.0, nxt - time.monotonic()))
                    continue
                hold = self.cosim.last_state
                if hold is not None and int(hold.get("vehicle_id", self.live_vid)) in allowed:
                    hx, hy = hold["x"], hold["y"]
                    hyaw, hvx = hold["yaw"], hold.get("vx", 0.0)
                    ht = hold.get("t", 0.0)
                else:
                    hx, hy = self.cfg["init_x"], self.cfg["init_y"]
                    hyaw, hvx, ht = self.cfg["init_yaw"], 0.0, 0.0
                snap = {"t": ht, "running": run, "setup_mode": False, "driver": driver,
                        "x": hx, "y": hy, "z": 0.0,
                        "yaw": hyaw, "roll": 0.0, "pitch": 0.0,
                        "vx": hvx, "vy": 0.0, "r": 0.0, "wx": 0.0, "wy": 0.0,
                        "ax": 0.0, "ay": 0.0, "steer": 0.0, "Fz": [0.0, 0.0, 0.0, 0.0],
                        "Ft": [], "wheel_spin": [], "susp": [], "rack_torque": 0.0,
                        "kappa": [], "alpha": [],
                        "level": self.cfg["level"], "vehicle": self.cfg["vehicle"],
                        "v_target": vt, "dt": dt, "time_scale": 1.0,
                        "source": "waiting", "cosim_up": cosim_on, "fleet": {}}
                snap["plant_error"] = self.plant_error
            snap["grade"] = self.cfg["road_grade"]
            snap["bank"] = self.cfg["road_bank"]
            if self.terrain is not None:        # live slope the contact model sees
                gx, gy = self._terrain_slope(snap["x"], snap["y"])
                ny = snap["yaw"]
                snap["grade"] = gx * math.cos(ny) + gy * math.sin(ny)   # along heading
                snap["bank"] = -gx * math.sin(ny) + gy * math.cos(ny)   # lateral
            snap["npath"] = len(self.path.pts)
            snap["terrain"] = 1 if self.terrain is not None else 0
            with self.lock:
                self.latest = snap
            if self.rec_on and len(self.rec_rows) < 200000:
                Fz = snap.get("Fz", [0, 0, 0, 0])
                Ft = snap.get("Ft", []) or [[0.0, 0.0]] * 4
                kap = snap.get("kappa", []) or [0.0] * 4
                alp = snap.get("alpha", []) or [0.0] * 4
                cmd = self.ports[self.live_vid].applied
                roll, pitch, yaw = snap["roll"], snap["pitch"], snap["yaw"]
                self.rec_rows.append({
                    "t": snap["t"], "pos": (snap["x"], snap["y"], snap.get("z", 0.0)),
                    "quat": euler_to_quat(roll, pitch, yaw),
                    "row": [snap["t"], snap["x"], snap["y"], snap.get("z", 0.0), yaw, roll, pitch,
                            snap["vx"], snap["vy"], snap["r"], snap.get("wx", 0.0), snap.get("wy", 0.0),
                            snap["ax"], snap["ay"], snap["steer"],
                            Fz[0], Fz[1], Fz[2], Fz[3],
                            cmd["throttle"], cmd["brake"], cmd["steer"],
                            snap.get("source", "sim"), snap["level"],
                            snap.get("m_gx", ""), snap.get("m_gy", ""),
                            snap.get("m_ax", ""), snap.get("m_ay", ""),
                            snap.get("m_wz", ""), snap.get("m_steer", ""),
                            Ft[0][0], Ft[1][0], Ft[2][0], Ft[3][0],
                            Ft[0][1], Ft[1][1], Ft[2][1], Ft[3][1],
                            kap[0], kap[1], kap[2], kap[3],
                            alp[0], alp[1], alp[2], alp[3]]})
            self._telemetry_send(snap)
            nxt += 1.0 / fps
            time.sleep(max(0.0, nxt - time.monotonic()))

    def snapshot(self):
        with self.lock:
            snap = dict(self.latest)
            p = self.ports[self.live_vid]
            snap["veh"] = self.live_vid
            snap["live_vid"] = self.live_vid
            snap["vehicles"] = sorted(self.ports)
            snap["fleet_spec"] = list(self.fleet_spec)
            snap["cmd_in"] = dict(p.applied)
            snap["io_age"] = (time.monotonic() - p.io_last_t
                              if p.io_last_t is not None else None)
            snap["cosim"] = self.cosim.running()
            snap["plant_error"] = self.plant_error
            if snap.get("source") == "waiting" and self.plant_error:
                snap["plant_error"] = self.plant_error
            snap["kinematics_warnings"] = list(self.cosim.kinematics_warnings)
            return snap

    def load_map(self, xodr):
        sys.path.insert(0, str(REPO / "examples"))
        import opendrive as od
        roads = od.parse_xodr(xodr)
        try:                                    # follow OpenDRIVE links (handles junctions)
            route = od.route_by_links(roads, od.parse_junctions(xodr))
        except Exception:
            route = []
        if len(route) < 2:                      # fall back to greedy geometric chaining
            route = od.chain_route(roads, step=3.0, gap_tol=50.0)
        if len(route) < 2:
            return {"ok": False, "msg": "no drivable route from " + xodr}
        with self.lock:
            self.terrain = None
            self.scenery = None
            self.path = WaypointPath(route)
            x0, y0 = route[0]
            x1, y1 = route[1]
            self.cfg["init_x"], self.cfg["init_y"] = x0, y0
            self.cfg["init_yaw"] = math.atan2(y1 - y0, x1 - x0)
            self.cfg["driver"] = True
            self.path_preset = "custom"
            self._rebuild_if_running()
        L = sum(math.hypot(route[i + 1][0] - route[i][0], route[i + 1][1] - route[i][1])
                for i in range(len(route) - 1))
        return {"ok": True, "pts": len(route), "length": round(L, 1)}

    def load_rd5(self, rd5, obj="", cell=5.0, buildings=""):
        sys.path.insert(0, str(REPO / "examples"))
        import rd5_route as rr
        route = [(float(p[0]), float(p[1])) for p in rr.route_polyline(rd5)]
        if len(route) < 2:
            return {"ok": False, "msg": "no Route_0 in " + rd5}
        terr = None
        if obj and os.path.exists(obj):         # CarMaker road + terrain share one frame
            import obj_to_heightmap as ob
            H, tx0, ty0, dx, dy, bb = ob.bake_heightmap(obj, cell)
            terr = {"H": H, "x0": tx0, "y0": ty0, "dx": dx, "dy": dy, "bb": bb}
        scn, texdir = None, None
        if buildings and os.path.exists(buildings):
            scn = self._parse_obj_meshes(buildings)
            texdir = scn.pop("_texdir", None)
            scn["loaded"] = True
        with self.lock:
            self.terrain = terr                 # drive the route on the real elevation
            self.scenery = scn                  # buildings/structures (same frame)
            self.tex_dir = texdir               # dir to serve building textures from
            self.path = WaypointPath(route)
            x0, y0 = route[0]
            x1, y1 = route[1]
            self.cfg["init_x"], self.cfg["init_y"] = x0, y0
            self.cfg["init_yaw"] = math.atan2(y1 - y0, x1 - x0)
            self.cfg["driver"] = True
            self.path_preset = "custom"
            self._rebuild_if_running()
        L = sum(math.hypot(route[i + 1][0] - route[i][0], route[i + 1][1] - route[i][1])
                for i in range(len(route) - 1))
        out = {"ok": True, "pts": len(route), "length": round(L, 1)}
        if terr is not None:
            out["terrain"] = {"z": [round(float(bb[4]), 1), round(float(bb[5]), 1)]}
        if scn is not None:
            out["buildings"] = len(scn["groups"])
        return out

    def path_points(self):
        with self.lock:
            return [[float(p[0]), float(p[1])] for p in self.path.pts]

    def load_terrain(self, obj, cell=5.0):
        sys.path.insert(0, str(REPO / "examples"))
        import obj_to_heightmap as ob
        H, x0, y0, dx, dy, bb = ob.bake_heightmap(obj, cell)
        cx, cy = 0.5 * (bb[0] + bb[2]), 0.5 * (bb[1] + bb[3])
        pts = fig8_pts(cx, cy, 50.0)                 # autopilot loop on terrain
        yaw0 = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
        with self.lock:
            self.terrain = {"H": H, "x0": x0, "y0": y0, "dx": dx, "dy": dy, "bb": bb}
            self.scenery = None
            self.cfg["init_x"], self.cfg["init_y"] = pts[0][0], pts[0][1]
            self.cfg["init_yaw"], self.cfg["init_v"] = yaw0, 5.0   # roll onto the path
            self.cfg["v_target"], self.cfg["driver"] = 10.0, True
            self.path = WaypointPath(pts)
            self.path_preset = "custom"
            self._rebuild_if_running()
        return {"ok": True, "nx": int(H.shape[1]), "ny": int(H.shape[0]),
                "z": [round(float(bb[4]), 1), round(float(bb[5]), 1)],
                "center": [round(cx, 1), round(cy, 1)]}

    def clear_terrain(self):
        with self.lock:
            self.terrain = None
            self.scenery = None
            self.path = FigureEight()
            self.path_preset = "figure8"
            self.cfg["init_x"] = self.cfg["init_y"] = self.cfg["init_yaw"] = 0.0
            self._rebuild_if_running()
        return {"ok": True}

    # approximate flat colors for the speedway building materials
    _MAT_COLOR = {
        "building_grey_simple": 0x9a9a9a, "building_white_simple": 0xdcdcd2,
        "building_shutter": 0x6f6f6f, "glass": 0x88aacc, "roof": 0x8a4636,
        "cp_pole": 0x555555, "pole_simple": 0x555555, "plastic_gray": 0x808080,
    }

    @staticmethod
    def _parse_mtl(path):
        # material name -> texture file basename (from map_Kd)
        tex = {}
        if not os.path.exists(path):
            return tex
        cur = None
        with open(path) as f:
            for line in f:
                if line.startswith("newmtl"):
                    cur = line.split()[1]
                elif line.startswith("map_Kd") and cur:
                    tex[cur] = os.path.basename(line.split()[-1])
        return tex

    def _parse_obj_meshes(self, obj):
        # expand to non-indexed per-material groups carrying position + uv, so a
        # vertex shared across faces with different uv stays correct; map each
        # material to its map_Kd texture (served via /tex/<name>).
        verts, uvs, faces, cur, mtllib = [], [], {}, "default", None
        with open(obj) as f:
            for line in f:
                if line.startswith("v "):
                    p = line.split(); verts.append((float(p[1]), float(p[2]), float(p[3])))
                elif line.startswith("vt "):
                    p = line.split(); uvs.append((float(p[1]), float(p[2])))
                elif line.startswith("mtllib"):
                    mtllib = line.split()[1]
                elif line.startswith("usemtl"):
                    cur = line.split()[1] if len(line.split()) > 1 else "default"
                elif line.startswith("f "):
                    vt = []
                    for t in line.split()[1:]:
                        a = t.split("/")
                        vi = int(a[0]) - 1
                        ti = int(a[1]) - 1 if len(a) > 1 and a[1] else -1
                        vt.append((vi, ti))
                    g = faces.setdefault(cur, [])
                    for k in range(1, len(vt) - 1):       # fan-triangulate
                        g += [vt[0], vt[k], vt[k + 1]]
        tex = self._parse_mtl(os.path.join(os.path.dirname(obj), mtllib)) if mtllib else {}
        groups = []
        for m, fl in faces.items():
            if not fl:
                continue
            pos, uv = [], []
            for vi, ti in fl:
                x, y, z = verts[vi]; pos += [round(x, 3), round(y, 3), round(z, 3)]
                if ti >= 0 and ti < len(uvs):
                    uv += [round(uvs[ti][0], 4), round(uvs[ti][1], 4)]
                else:
                    uv += [0.0, 0.0]
            groups.append({"color": self._MAT_COLOR.get(m, 0x999999),
                           "texture": tex.get(m), "pos": pos, "uv": uv})
        return {"groups": groups, "_texdir": os.path.join(os.path.dirname(obj), "textures")}

    def scenery_meshes(self):
        with self.lock:
            return self.scenery if self.scenery is not None else {"loaded": False}

    def _terrain_slope(self, x, y):
        # central-difference dH/dx, dH/dy of the baked heightmap at world (x,y)
        t = self.terrain
        if t is None:
            return 0.0, 0.0
        H, x0, y0, dx, dy = t["H"], t["x0"], t["y0"], t["dx"], t["dy"]
        ny, nx = H.shape

        def h(wx, wy):
            fx = min(max((wx - x0) / dx, 0), nx - 1.001)
            fy = min(max((wy - y0) / dy, 0), ny - 1.001)
            ix, iy = int(fx), int(fy)
            ax, ay = fx - ix, fy - iy
            return ((H[iy, ix] * (1 - ax) + H[iy, ix + 1] * ax) * (1 - ay) +
                    (H[iy + 1, ix] * (1 - ax) + H[iy + 1, ix + 1] * ax) * ay)
        e = 2.0
        return (h(x + e, y) - h(x - e, y)) / (2 * e), (h(x, y + e) - h(x, y - e)) / (2 * e)

    def terrain_grid(self, maxn=100):
        with self.lock:
            if self.terrain is None:
                return {"loaded": False}
            t = self.terrain
            H = t["H"]
            ny, nx = H.shape
            sx, sy = max(1, nx // maxn), max(1, ny // maxn)
            Hs = H[::sy, ::sx]
            return {"loaded": True, "x0": float(t["x0"]), "y0": float(t["y0"]),
                    "dx": float(t["dx"] * sx), "dy": float(t["dy"] * sy),
                    "nx": int(Hs.shape[1]), "ny": int(Hs.shape[0]),
                    "z": [[round(float(v), 3) for v in row] for row in Hs]}

    def log_start(self):
        with self.lock:
            self.rec_rows = []
            self.rec_on = True
        return self.log_status()

    def log_stop(self):
        with self.lock:
            self.rec_on = False
            rows = self.rec_rows
            self.rec_rows = []
        if not rows:
            return {"ok": False, "msg": "no rows recorded"}
        import csv as _csv
        d = REPO / "logs"
        d.mkdir(exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        csvp, tump = d / f"run_{ts}.csv", d / f"run_{ts}.tum"
        with open(csvp, "w", newline="") as fc:
            w = _csv.writer(fc)
            w.writerow(LOG_COLS)
            for r in rows:
                w.writerow(r["row"])
        with open(tump, "w") as ft:   # evo TUM: t x y z qx qy qz qw
            for r in rows:
                p, q = r["pos"], r["quat"]
                ft.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"
                         % (r["t"], p[0], p[1], p[2], q[0], q[1], q[2], q[3]))
        info = {"ok": True, "csv": str(csvp), "tum": str(tump), "rows": len(rows)}
        with self.lock:
            self.rec_last = {"csv": str(csvp), "tum": str(tump), "rows": len(rows)}
        return info

    def log_status(self):
        with self.lock:
            return {"recording": self.rec_on, "rows": len(self.rec_rows),
                    "last": dict(self.rec_last)}

    def start_cosim(self, over):
        # Manual (re)start of the real-time runtime with the current config.
        # B4 makes the GUI always run on it, so this just rebuilds.
        with self.lock:
            if "level" in over:
                self.cfg["level"] = str(over["level"])
            self._build()
        return self.cosim.status()

    def stop_cosim(self):
        return self.cosim.stop()

    def config(self):
        with self.lock:
            c = dict(self.cfg)
            c["dt"] = self.dt
            c["time_scale"] = self.time_scale
            c["integrator"] = self.solver.integrator.name
            c["max_substeps"] = self.solver.max_substeps
            c["live_vid"] = self.live_vid
            return c

    def tire_samples(self):
        return sorted(p.stem for p in TIRE_DIR.glob("*.yaml"))

    def load_tire(self, name, vehicle_id=None):
        stem = Path(str(name)).stem
        path = (TIRE_DIR / f"{stem}.yaml").resolve()
        if not str(path).startswith(str(TIRE_DIR.resolve())) or not path.is_file():
            raise ValueError(f"unknown tire preset: {name}")
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            if vid == self.live_vid:
                self.tp = vdsim.TireParams.from_yaml(str(path))
            else:
                spec = self._spec_for_vid(vid)
                spec["tire"] = stem
                self.fleet_overrides.get(vid, {}).pop("tire", None)
            self._rebuild_if_running()

    def import_tir(self, text, vehicle_id=None):
        from tir_to_yaml import parse_tir_text, tir_to_params
        raw = parse_tir_text(text)
        mapped = tir_to_params(raw)
        if not mapped:
            raise ValueError("no recognized .tir coefficients")
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            if vid == self.live_vid:
                _apply(self.tp, TIRE_FIELDS, mapped)
            else:
                bucket = self.fleet_overrides.setdefault(vid, {}).setdefault("tire", {})
                bucket.update(mapped)
            self._rebuild_if_running()
        return {"parsed": len(raw), "mapped": sorted(mapped.keys())}

    def export_simconfig(self):
        with self.lock:
            fleet = []
            for spec in self.fleet_spec:
                row = dict(spec)
                self._ensure_fleet_parts(row)
                _strip_fleet_susp_if_not_l3(row)
                fleet.append(row)
            return {
                "version": 2,
                "config": self.config(),
                "vehicle": _params_dict(self.vp, VEHICLE_FIELDS),
                "tire": _params_dict(self.tp, TIRE_FIELDS),
                "sensors": _flat_sensors(self.sensors),
                "actuator": _flat_actuator(self.act, self.sensor_delay),
                "fleet_spec": fleet,
                "fleet_overrides": {str(k): v for k, v in self.fleet_overrides.items()},
                "live_vid": self.live_vid,
                "path_preset": self.path_preset,
                "path_pts": [[float(p[0]), float(p[1])] for p in self.path.pts],
                "cosim_attach": bool(self.cfg.get("cosim_attach", False)),
                "cosim_host": str(self.cfg.get("cosim_host", "127.0.0.1")),
                "cosim_cmd_port": int(self.cfg.get("cosim_cmd_port", 7401)),
                "infra_sensors": list(self.infra_sensors),
            }

    def import_simconfig(self, data):
        with self.lock:
            c = data.get("config") or {}
            if c.get("vehicle"):
                self.load_vehicle(c["vehicle"])
            for k in ("level", "vehicle", "v_target", "driver", "running",
                      "init_x", "init_y", "init_yaw", "init_v",
                      "road_mu", "road_mu_right", "road_boundary",
                      "road_grade", "road_bank", "road_rough_amp", "road_rough_wl"):
                if k in c:
                    self.cfg[k] = c[k]
            if "dt" in c and c["dt"] > 1e-5:
                self.dt = float(c["dt"])
            if "time_scale" in c and c["time_scale"] > 0:
                self.time_scale = float(c["time_scale"])
            if c.get("integrator") in ENUM_MAPS["integrator"]:
                self.solver.integrator = ENUM_MAPS["integrator"][c["integrator"]]
            if "max_substeps" in c:
                self.solver.max_substeps = max(1, int(c["max_substeps"]))
            ver = int(data.get("version", 1))
            if ver >= 2 and data.get("fleet_spec"):
                self.fleet_overrides = {
                    int(k): v for k, v in (data.get("fleet_overrides") or {}).items()}
                self.fleet_spec = []
                for spec in data["fleet_spec"]:
                    row = dict(spec)
                    self._ensure_fleet_parts(row)
                    _strip_fleet_susp_if_not_l3(row)
                    self.fleet_spec.append(row)
                self._ensure_ports()
                if "live_vid" in data:
                    self.live_vid = int(data["live_vid"])
                pp = data.get("path_preset")
                if pp == "custom" and data.get("path_pts"):
                    pts = [(float(p[0]), float(p[1])) for p in data["path_pts"]]
                    if len(pts) >= 2:
                        self.path = WaypointPath(pts)
                        self.path_preset = "custom"
                elif pp and pp != "custom":
                    self.set_path_preset(str(pp))
                for k, ck in (("cosim_attach", "cosim_attach"),
                              ("cosim_host", "cosim_host"),
                              ("cosim_cmd_port", "cosim_cmd_port")):
                    if k in data:
                        self.cfg[ck] = data[k]
                self._sync_live_from_fleet()
            if "vehicle" in data:
                _apply(self.vp, VEHICLE_FIELDS, data["vehicle"])
            if "tire" in data:
                _apply(self.tp, TIRE_FIELDS, data["tire"])
            if "sensors" in data:
                self._apply_sensors(data["sensors"])
            if "actuator" in data:
                self._apply_actuator(data["actuator"])
            if "infra_sensors" in data:
                self.infra_sensors = list(data["infra_sensors"] or [])
            self._rebuild_if_running()
        return self.config()


RUNNER = Runner()


def _qvid(qs, default=None):
    v = qs.get("vehicle_id", [None])[0]
    if v is None or v == "":
        return default
    return int(v)


class Handler(BaseHTTPRequestHandler):
    # HTTP/1.1 so the SSE stream (/api/stream) is a persistent connection the
    # browser EventSource can hold open (HTTP/1.0 closes -> stuck "connecting").
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route, qs = urlparse(self.path).path, parse_qs(urlparse(self.path).query)
        if route in ("/", "/index.html", "/app", "/app.html", "/legacy", "/classic", "/classic.html"):
            html = (HERE / "app.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif route.startswith("/vendor/"):
            # locally-vendored JS (Three.js etc.) so the client needs no CDN
            rel = route.lstrip("/").split("?")[0]
            fp = (HERE / rel).resolve()
            if str(fp).startswith(str(HERE / "vendor")) and fp.is_file():
                data = fp.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/javascript")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404); self.end_headers()
        elif route == "/api/config":
            self._json({"config": RUNNER.config(),
                        "vehicles": VEHICLES, "levels": LEVELS})
        elif route == "/api/vehicle":
            self._json({"fields": RUNNER.serialize_vehicle(_qvid(qs))})
        elif route == "/api/tire":
            self._json({"fields": RUNNER.serialize_tire(_qvid(qs))})
        elif route == "/api/actuator":
            self._json({"fields": RUNNER.serialize_actuator()})
        elif route == "/api/sensors":
            self._json({"fields": RUNNER.serialize_sensors()})
        elif route == "/api/tire/curves":
            self._json({"plots": RUNNER.tire_curves(_qvid(qs))})
        elif route == "/api/tire/samples":
            self._json({"samples": RUNNER.tire_samples()})
        elif route == "/api/parts/registry":
            self._json(parts_registry())
        elif route == "/api/suspension/list":
            preview = (qs.get("preview") or ["0"])[0] in ("1", "true", "yes")
            self._json(list_suspension_api(preview_all=preview))
        elif route == "/api/suspension/default":
            veh = (qs.get("vehicle") or ["sedan"])[0]
            self._json(suspension_default_for_vehicle(veh))
        elif route == "/api/suspension/schematic":
            name = (qs.get("name") or [""])[0]
            try:
                self._json(suspension_schematic(name))
            except ValueError as e:
                self._json({"ok": False, "error": str(e)})
        elif route == "/api/simconfig":
            self._json(RUNNER.export_simconfig())
        elif route == "/api/fleet":
            self._json({"fleet": RUNNER.fleet_enriched(), "live_vid": RUNNER.live_vid,
                        "vehicles": sorted(RUNNER.ports)})
        elif route == "/api/setup":
            self._json(RUNNER.get_setup())
        elif route == "/api/scenario/list":
            self._json({"scenarios": RUNNER.list_scenarios()})
        elif route == "/api/actuator/step":
            self._json({"plots": RUNNER.actuator_step()})
        elif route in ("/api/state", "/api/io"):
            self._json(RUNNER.snapshot())
        elif route == "/api/io/targets":
            self._json(RUNNER.telemetry_config())
        elif route == "/api/cosim":
            self._json(RUNNER.cosim.status())
        elif route == "/api/log/status":
            self._json(RUNNER.log_status())
        elif route == "/api/path":
            self._json({"pts": RUNNER.path_points()})
        elif route == "/api/terrain":
            self._json(RUNNER.terrain_grid())
        elif route == "/api/scenery":
            self._json(RUNNER.scenery_meshes())
        elif route.startswith("/tex/"):
            name = os.path.basename(route[len("/tex/"):].split("?")[0])
            tdir = RUNNER.tex_dir
            fp = os.path.join(tdir, name) if tdir else ""
            if fp and os.path.isfile(fp):
                data = Path(fp).read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404); self.end_headers()
        elif route.startswith("/api/log/download"):
            which = "tum" if route.endswith("tum") else "csv"
            path = RUNNER.rec_last.get(which)
            if not path or not Path(path).is_file():
                self.send_error(404)
                return
            data = Path(path).read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Disposition",
                             f'attachment; filename="{Path(path).name}"')
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif route == "/api/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    self.wfile.write(f"data: {json.dumps(RUNNER.snapshot())}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(1.0 / 60.0)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_error(404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/api/config":
            if "live_vid" in body:
                with RUNNER.lock:
                    vid = int(body["live_vid"])
                    if vid in RUNNER.ports:
                        RUNNER.live_vid = vid
            RUNNER.reconfigure(**{k: v for k, v in body.items()
                                  if k not in ("live_vid", "scenario")})
            self._json({"ok": True, "config": RUNNER.config()})
        elif self.path == "/api/fleet":
            try:
                if body.get("scenario"):
                    RUNNER.load_fleet_scenario(str(body["scenario"]))
                    RUNNER._rebuild_if_running()
                elif "live_vid" in body:
                    with RUNNER.lock:
                        vid = int(body["live_vid"])
                        if vid in RUNNER.ports:
                            RUNNER.live_vid = vid
                            RUNNER._sync_live_from_fleet()
                self._json({"ok": True, "fleet": RUNNER.fleet_enriched(),
                            "live_vid": RUNNER.live_vid})
            except ValueError as e:
                self._json({"ok": False, "error": str(e)}, 400)
        elif self.path == "/api/sim":
            RUNNER.set_sim(dt=body.get("dt"), time_scale=body.get("time_scale"),
                           integrator=body.get("integrator"),
                           max_substeps=body.get("max_substeps"),
                           init_x=body.get("init_x"), init_y=body.get("init_y"),
                           init_yaw=body.get("init_yaw"), init_v=body.get("init_v"),
                           road_mu=body.get("road_mu"),
                           road_mu_right=body.get("road_mu_right"),
                           road_boundary=body.get("road_boundary"),
                           road_grade=body.get("road_grade"),
                           road_bank=body.get("road_bank"),
                           road_rough_amp=body.get("road_rough_amp"),
                           road_rough_wl=body.get("road_rough_wl"))
            self._json({"ok": True, "config": RUNNER.config()})
        elif self.path == "/api/vehicle":
            vid = body.get("vehicle_id")
            payload = {k: v for k, v in body.items() if k != "vehicle_id"}
            RUNNER.set_params("vehicle", payload, vehicle_id=vid)
            self._json({"ok": True})
        elif self.path == "/api/tire":
            vid = body.get("vehicle_id")
            payload = {k: v for k, v in body.items() if k != "vehicle_id"}
            RUNNER.set_params("tire", payload, vehicle_id=vid)
            self._json({"ok": True})
        elif self.path == "/api/tire/load":
            try:
                RUNNER.load_tire(body.get("name", ""), vehicle_id=body.get("vehicle_id"))
                self._json({"ok": True, "name": body.get("name")})
            except ValueError as e:
                self._json({"ok": False, "error": str(e)}, 400)
        elif self.path == "/api/tire/import":
            try:
                info = RUNNER.import_tir(body.get("text", ""),
                                        vehicle_id=body.get("vehicle_id"))
                self._json({"ok": True, **info})
            except ValueError as e:
                self._json({"ok": False, "error": str(e)}, 400)
        elif self.path == "/api/scenario/save":
            try:
                self._json(RUNNER.save_scenario(body.get("name", "")))
            except ValueError as e:
                self._json({"ok": False, "error": str(e)}, 400)
        elif self.path == "/api/cosim/attach":
            host = str(body.get("host", "127.0.0.1"))
            port = int(body.get("cmd_port", COSIM_CMD_PORT))
            st_port = int(body.get("state_port", COSIM_STATE_PORT))
            with RUNNER.lock:
                RUNNER.cfg["cosim_attach"] = True
                RUNNER.cfg["cosim_host"] = host
                RUNNER.cfg["cosim_cmd_port"] = port
                RUNNER.cfg["cosim_state_port"] = st_port
                RUNNER.plant_error = None
            RUNNER.cosim.attach(host, port, st_port)
            t0 = time.monotonic()
            while time.monotonic() < t0 + 2.5:
                if RUNNER.cosim.last_state is not None:
                    break
                time.sleep(0.05)
            with RUNNER.lock:
                ok = RUNNER.cosim.last_state is not None
                RUNNER.cfg["running"] = ok
                if not ok:
                    RUNNER.plant_error = (
                        "attach failed — no STATE on :%d (uncheck Attach external?)"
                        % st_port)
                    RUNNER.cosim.stop()
            self._json({"ok": ok, "cosim": RUNNER.cosim.status(),
                        "error": RUNNER.plant_error})
        elif self.path == "/api/simconfig":
            self._json({"ok": True, "config": RUNNER.import_simconfig(body)})
        elif self.path == "/api/actuator":
            RUNNER.set_params("actuator", body)
            self._json({"ok": True})
        elif self.path == "/api/sensors":
            RUNNER.set_params("sensors", body)
            self._json({"ok": True})
        elif self.path == "/api/setup":
            RUNNER.apply_setup(body)
            self._json({"ok": True, "setup": RUNNER.get_setup()})
        elif self.path == "/api/control":
            out = RUNNER.control(body.get("action", ""))
            self._json({"ok": True, **out})
        elif self.path == "/api/manual":
            RUNNER.set_manual(**body)
            self._json({"ok": True})
        elif self.path == "/api/io":
            # External data port: command in -> state out (one round-trip).
            self._json(RUNNER.io(body))
        elif self.path == "/api/io/targets":
            RUNNER.set_telemetry(body)
            self._json({"ok": True, **RUNNER.telemetry_config()})
        elif self.path == "/api/cosim/start":
            self._json(RUNNER.start_cosim(body))
        elif self.path == "/api/cosim/stop":
            self._json(RUNNER.stop_cosim())
        elif self.path == "/api/map/load":
            self._json(RUNNER.load_map(body.get("xodr", "")))
        elif self.path == "/api/map/rd5":
            self._json(RUNNER.load_rd5(body.get("rd5", ""), body.get("obj", ""),
                                       float(body.get("cell", 5.0)),
                                       body.get("buildings", "")))
        elif self.path == "/api/terrain/load":
            self._json(RUNNER.load_terrain(body.get("obj", ""),
                                           float(body.get("cell", 5.0))))
        elif self.path == "/api/terrain/clear":
            self._json(RUNNER.clear_terrain())
        elif self.path == "/api/log/start":
            self._json(RUNNER.log_start())
        elif self.path == "/api/log/stop":
            self._json(RUNNER.log_stop())
        else:
            self.send_error(404)


def _udp_control(host, port):
    # Low-latency control/telemetry for the racing-wheel FFB bridge: a JSON
    # datagram {steer,throttle,brake} in -> drive the live sim; reply with
    # {rack_torque,vx,steer,susp} for force feedback. Same sim the browser views.
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind((host, port))
    except OSError as e:
        # Non-fatal: the FFB port is optional (only a physical wheel uses it).
        # Don't take down the GUI if it's busy (e.g. another instance).
        print(f"[VDSim GUI] FFB UDP port {port} unavailable ({e}); FFB disabled")
        return
    print(f"[VDSim GUI] udp control/ffb on {host}:{port}")
    while True:
        try:
            data, addr = s.recvfrom(1024)
        except OSError:
            continue
        try:
            c = json.loads(data)
            RUNNER.set_manual(steer=c.get("steer", 0.0),
                              throttle=c.get("throttle", 0.0), brake=c.get("brake", 0.0))
        except Exception:
            pass
        snap = RUNNER.latest
        try:
            s.sendto(json.dumps({"rack_torque": snap.get("rack_torque", 0.0),
                                 "vx": snap.get("vx", 0.0), "steer": snap.get("steer", 0.0),
                                 "susp": snap.get("susp", [])}).encode(), addr)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--udp-port", type=int, default=0,
                    help="UDP control/FFB port (default: http port + 1)")
    args = ap.parse_args()
    udp_port = args.udp_port or (args.port + 1)
    threading.Thread(target=_udp_control, args=(args.host, udp_port), daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[VDSim GUI] http://{args.host}:{args.port}  (compute here, view in browser)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[VDSim GUI] stopped.")


if __name__ == "__main__":
    main()
