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
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "cosim"))
HERE = Path(__file__).resolve().parent

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
    ("max_brake_torque", "Max brake torque [N·m]", "Drivetrain", "num", _ALL),
    ("brake_bias_front", "Brake bias — front share [0–1] (rear = 1−front)", "Drivetrain", "num", "L1,L2,L3"),
    ("brake_ebd_enabled", "Brake EBD (Fz-based bias)", "Drivetrain", "bool", "L2,L3"),
    ("steering_ratio", "Steering ratio [-]", "Steering", "num", "L1,L2,L3"),
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


COSIM_BIN = REPO / "build" / "bin" / "vdsim_udp_server"


class CosimBridge:
    """Launches the binary vdsim_udp_server, consumes its STATE packets for the
    3D view, and relays control as CMD packets (per cosim_protocol.hpp).

    The GUI configures and runs the real co-sim server; the Python playground sim
    is bypassed while the bridge is active so there is one source of truth.
    """
    DEFAULT = {"level": "L2", "cmd_port": 7001, "state_port": 7002,
               "rate": 200.0, "vx0": 0.0, "cmd_timeout": 0.1}

    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.cfg = dict(self.DEFAULT)
        self.started_t = None
        self.last_state = None
        self.last_state_t = None
        self._seq = 0
        self._rx = None
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tmp = tempfile.mkdtemp(prefix="vdsim_cosim_")
        self._stop = threading.Event()

    def available(self):
        return COSIM_BIN.exists()

    def running(self):
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def start(self, vp, tp, over):
        if self.running():
            self.stop()
        with self.lock:
            for k in self.cfg:
                if k in over:
                    self.cfg[k] = over[k]
            self.cfg["level"] = str(self.cfg["level"])
            vy = os.path.join(self._tmp, "vehicle.yaml")
            ty = os.path.join(self._tmp, "tire.yaml")
            vp.to_yaml(vy)
            tp.to_yaml(ty)
            c = self.cfg
            args = [str(COSIM_BIN), vy, ty, f"--level={c['level']}",
                    f"--cmd-port={int(c['cmd_port'])}", "--state-ip=127.0.0.1",
                    f"--state-port={int(c['state_port'])}", f"--rate={float(c['rate'])}",
                    f"--vx0={float(c['vx0'])}", f"--cmd-timeout={float(c['cmd_timeout'])}"]
            self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._rx.bind(("127.0.0.1", int(c["state_port"])))
            self._rx.settimeout(0.2)
            self._stop.clear()
            self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
            self.started_t = time.monotonic()
            self.last_state = None
            threading.Thread(target=self._rx_loop, args=(self._rx,), daemon=True).start()
        return self.status()

    def stop(self):
        with self.lock:
            self._stop.set()
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            self.proc = None
            self.started_t = None
            if self._rx is not None:
                try:
                    self._rx.close()
                except OSError:
                    pass
                self._rx = None
            self.last_state = None
        return self.status()

    def status(self):
        run = self.proc is not None and self.proc.poll() is None
        return {"available": self.available(), "running": run, "cfg": dict(self.cfg),
                "pid": (self.proc.pid if run else None),
                "uptime": (time.monotonic() - self.started_t if run and self.started_t else None),
                "state_age": (time.monotonic() - self.last_state_t if self.last_state_t else None),
                "binary": str(COSIM_BIN)}

    def send_cmd(self, throttle, brake, steer, gear=1):
        if not self.running():
            return
        self._seq += 1
        body = vds1.pack_cmd(self._seq, steer=steer, throttle=throttle, brake=brake,
                             gear=gear, aux_accel=0.0, aux_speed=0.0,
                             timestamp=time.time())
        try:
            self._tx.sendto(body, ("127.0.0.1", int(self.cfg["cmd_port"])))
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
                self.last_state = st
                self.last_state_t = time.monotonic()

    @staticmethod
    def _decode_state(buf):
        s = vds1.decode_state(buf)
        if s is None:
            return None
        return {"t": s["timestamp"], "x": s["x"], "y": s["y"], "z": s["z"],
                "roll": s["roll"], "pitch": s["pitch"], "yaw": s["yaw"],
                "vx": s["vx"], "vy": s["vy"], "r": s["yaw_rate"],
                "wx": s["roll_rate"], "wy": s["pitch_rate"],
                "ax": s["ax"], "ay": s["ay"],
                "steer": s["steer_applied"], "Fz": s["Fz"]}


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
                    "driver": True, "running": True,
                    "init_x": 0.0, "init_y": 0.0, "init_yaw": 0.0, "init_v": 10.0,
                    "road_mu": 1.0, "road_mu_right": -1.0, "road_boundary": 0.0,
                    "road_grade": 0.0, "road_bank": 0.0,
                    "road_rough_amp": 0.0, "road_rough_wl": 4.0}
        self.dt = 0.005
        self.time_scale = 1.0
        self.live_vid = 0                  # vehicle id backed by the (single) sim
        self.ports = {0: VehiclePort()}    # data-port config keyed by vehicle id
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
        self._build()
        threading.Thread(target=self._loop, daemon=True).start()

    def _build(self):
        if self.terrain is not None:             # drive on a baked terrain heightmap
            t = self.terrain
            self.sim = vdsim.make_sim_session_heightmap(
                self.vp, self.tp, self.cfg["level"], t["H"],
                x0=t["x0"], y0=t["y0"], dx=t["dx"], dy=t["dy"],
                mu=self.cfg["road_mu"], nominal_dt=self.dt, solver=self.solver)
        else:
            self.sim = vdsim.make_sim_session(
                self.vp, self.tp, self.cfg["level"], nominal_dt=self.dt,
                sensor_delay_s=self.sensor_delay, actuator=self.act,
                solver=self.solver, sensors=self.sensors,
                mu=self.cfg["road_mu"], mu_right=self.cfg["road_mu_right"],
                mu_boundary_y=self.cfg["road_boundary"],
                grade=self.cfg["road_grade"], bank=self.cfg["road_bank"],
                rough_amp=self.cfg["road_rough_amp"], rough_wavelength=self.cfg["road_rough_wl"])
        s0 = vdsim.make_init_state(
            x=self.cfg["init_x"], y=self.cfg["init_y"], yaw=self.cfg["init_yaw"],
            v=max(0.0, self.cfg["init_v"]), wheel_radius=self.vp.wheel_radius_nominal)
        self.sim.reset(s0)
        self.prev_idx = 0

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
            self._build()

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
                self._build()

    def set_params(self, which, data):
        with self.lock:
            if which == "vehicle":
                _apply(self.vp, VEHICLE_FIELDS, data)
            elif which == "tire":
                _apply(self.tp, TIRE_FIELDS, data)
            elif which == "sensors":
                self._apply_sensors(data)
            else:
                self._apply_actuator(data)
            self._build()

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

    def tire_curves(self):
        with self.lock:
            tp = self.tp
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
            # Negate Fy so +alpha -> +cornering force (intuitive reading).
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

    def control(self, action):
        with self.lock:
            if action == "reset":
                self._build()
            elif action == "stop":
                self.cfg["running"] = False
            elif action == "start":
                self.cfg["running"] = True

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
            cosim_on = self.cosim.running()
            cs = self.cosim.last_state if cosim_on else None
            if run and cosim_on:
                # Co-sim mode: the binary server is the plant. Build the command
                # (autopilot uses the server's state for feedback) and relay it as
                # a binary CMD; the Python sim is bypassed.
                t = b = st = 0.0
                if cs:
                    if driver:
                        st, self.prev_idx = self.path.steer(
                            cs["x"], cs["y"], cs["yaw"], cs["vx"], wb, self.prev_idx)
                        ax = max(-3.0, min(3.0, 0.8 * (vt - cs["vx"])))
                        t = min(1.0, ax / 3.0) if ax >= 0 else 0.0
                        b = min(1.0, -ax / 3.0) if ax < 0 else 0.0
                    else:
                        t, b, st = man["throttle"], man["brake"], man["steer"]
                self.cosim.send_cmd(t, b, st)
                self.ports[self.live_vid].applied = {"throttle": t, "brake": b, "steer": st}
            elif run:
                pending = min(pending + (1.0 / fps) * ts, 0.5)  # cap to avoid spiral
                while pending >= dt:
                    s = self.sim.state()
                    cmd = vdsim.CmdL4()
                    if driver:
                        x, y = float(s.position[0]), float(s.position[1])
                        steer, self.prev_idx = self.path.steer(
                            x, y, s.yaw(), s.vx(), wb, self.prev_idx)
                        ax = max(-3.0, min(3.0, 0.8 * (vt - s.vx())))
                        if ax >= 0:
                            cmd.throttle = min(1.0, ax / 3.0)
                        else:
                            cmd.brake = min(1.0, -ax / 3.0)
                        cmd.steer_angle_wheel = steer
                    else:
                        cmd.throttle, cmd.brake = man["throttle"], man["brake"]
                        cmd.steer_angle_wheel = man["steer"]
                    self.sim.set_input(cmd)
                    self.sim.tick(dt)
                    pending -= dt
                    self.ports[self.live_vid].applied = {
                        "throttle": cmd.throttle, "brake": cmd.brake,
                        "steer": cmd.steer_angle_wheel}
            if cosim_on and cs:
                snap = {"t": cs["t"], "running": run, "driver": driver,
                        "x": cs["x"], "y": cs["y"], "z": cs["z"],
                        "yaw": cs["yaw"], "roll": cs["roll"], "pitch": cs["pitch"],
                        "vx": cs["vx"], "vy": cs["vy"], "r": cs["r"],
                        "wx": cs["wx"], "wy": cs["wy"], "ax": cs["ax"], "ay": cs["ay"],
                        "steer": cs["steer"], "Fz": cs["Fz"], "Ft": [], "susp": [],
                        "level": self.cosim.cfg["level"], "vehicle": self.cfg["vehicle"],
                        "v_target": vt, "dt": 1.0 / max(1.0, self.cosim.cfg["rate"]),
                        "time_scale": 1.0, "source": "cosim"}
            else:
                o = self.sim.output()
                s = o.state
                snap = {"t": o.sim_time, "running": run, "driver": driver,
                        "x": float(s.position[0]), "y": float(s.position[1]),
                        "z": float(s.position[2]),
                        "yaw": s.yaw(), "roll": o.roll, "pitch": o.pitch,
                        "vx": s.vx(), "vy": s.vy(), "r": s.yaw_rate(),
                        "wx": float(s.angular_velocity[0]), "wy": float(s.angular_velocity[1]),
                        "ax": o.ax, "ay": o.ay, "steer": o.steer_applied,
                        "rack_torque": o.rack_torque,   # aligning torque -> wheel FFB
                        "Fz": [float(v) for v in o.Fz],
                        "Ft": [[float(f[0]), float(f[1])] for f in o.tire_forces],
                        "kappa": [float(v) for v in o.slip_ratio],
                        "alpha": [float(v) for v in o.slip_angle],
                        "susp": [float(v) for v in s.susp_compression],
                        "level": self.cfg["level"], "vehicle": self.cfg["vehicle"],
                        "v_target": vt, "dt": dt, "time_scale": ts, "source": "sim",
                        "m_gx": o.sensors.gnss_x, "m_gy": o.sensors.gnss_y,
                        "m_ax": o.sensors.ax, "m_ay": o.sensors.ay,
                        "m_wz": o.sensors.wz, "m_steer": o.sensors.steer}
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
            snap["vehicles"] = sorted(self.ports)
            snap["cmd_in"] = dict(p.applied)
            snap["io_age"] = (time.monotonic() - p.io_last_t
                              if p.io_last_t is not None else None)
            snap["cosim"] = self.cosim.running()
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
            self._build()
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
            self._build()
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
            self._build()
        return {"ok": True, "nx": int(H.shape[1]), "ny": int(H.shape[0]),
                "z": [round(float(bb[4]), 1), round(float(bb[5]), 1)],
                "center": [round(cx, 1), round(cy, 1)]}

    def clear_terrain(self):
        with self.lock:
            self.terrain = None
            self.scenery = None
            self.path = FigureEight()
            self.cfg["init_x"] = self.cfg["init_y"] = self.cfg["init_yaw"] = 0.0
            self._build()
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
        with self.lock:
            vp, tp = self.vp, self.tp
            over.setdefault("level", self.cfg["level"])
            over.setdefault("vx0", self.cfg["init_v"])
        return self.cosim.start(vp, tp, over)

    def stop_cosim(self):
        return self.cosim.stop()

    def config(self):
        with self.lock:
            c = dict(self.cfg)
            c["dt"] = self.dt
            c["time_scale"] = self.time_scale
            c["integrator"] = self.solver.integrator.name
            c["max_substeps"] = self.solver.max_substeps
            return c


RUNNER = Runner()


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
        if self.path in ("/", "/index.html"):
            html = (HERE / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif self.path.startswith("/vendor/"):
            # locally-vendored JS (Three.js etc.) so the client needs no CDN
            rel = self.path.lstrip("/").split("?")[0]
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
        elif self.path == "/api/config":
            self._json({"config": RUNNER.config(),
                        "vehicles": VEHICLES, "levels": LEVELS})
        elif self.path == "/api/vehicle":
            self._json({"fields": _serialize(RUNNER.vp, VEHICLE_FIELDS)})
        elif self.path == "/api/tire":
            self._json({"fields": _serialize(RUNNER.tp, TIRE_FIELDS)})
        elif self.path == "/api/actuator":
            self._json({"fields": RUNNER.serialize_actuator()})
        elif self.path == "/api/sensors":
            self._json({"fields": RUNNER.serialize_sensors()})
        elif self.path == "/api/tire/curves":
            self._json({"plots": RUNNER.tire_curves()})
        elif self.path == "/api/actuator/step":
            self._json({"plots": RUNNER.actuator_step()})
        elif self.path in ("/api/state", "/api/io"):
            self._json(RUNNER.snapshot())
        elif self.path == "/api/io/targets":
            self._json(RUNNER.telemetry_config())
        elif self.path == "/api/cosim":
            self._json(RUNNER.cosim.status())
        elif self.path == "/api/log/status":
            self._json(RUNNER.log_status())
        elif self.path == "/api/path":
            self._json({"pts": RUNNER.path_points()})
        elif self.path == "/api/terrain":
            self._json(RUNNER.terrain_grid())
        elif self.path == "/api/scenery":
            self._json(RUNNER.scenery_meshes())
        elif self.path.startswith("/tex/"):
            name = os.path.basename(self.path[len("/tex/"):].split("?")[0])
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
        elif self.path.startswith("/api/log/download"):
            which = "tum" if self.path.endswith("tum") else "csv"
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
        elif self.path == "/api/stream":
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
            RUNNER.reconfigure(**body)
            self._json({"ok": True, "config": RUNNER.config()})
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
            RUNNER.set_params("vehicle", body)
            self._json({"ok": True})
        elif self.path == "/api/tire":
            RUNNER.set_params("tire", body)
            self._json({"ok": True})
        elif self.path == "/api/actuator":
            RUNNER.set_params("actuator", body)
            self._json({"ok": True})
        elif self.path == "/api/sensors":
            RUNNER.set_params("sensors", body)
            self._json({"ok": True})
        elif self.path == "/api/control":
            RUNNER.control(body.get("action", ""))
            self._json({"ok": True})
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
    s.bind((host, port))
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
