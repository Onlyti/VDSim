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
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
HERE = Path(__file__).resolve().parent

try:
    import vdsim
except ImportError as e:
    sys.exit(f"import vdsim failed ({e}). Build with -DVDSIM_BUILD_PYTHON=ON.")

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
    ("spring_stiffness", "Spring stiffness [N/m]", "Suspension", "arr", "L3"),
    ("damper_coefficient", "Damper [N·s/m]", "Suspension", "arr", "L3"),
    ("unsprung_mass", "Unsprung mass [kg]", "Suspension", "arr", "L3"),
    ("wheel_inertia", "Wheel inertia [kg·m²] (0=auto)", "Suspension", "arr", "L1,L2,L3"),
    ("roll_stiffness_front", "Roll stiff front [N·m/rad]", "Suspension", "num", "L2,L3"),
    ("roll_stiffness_rear", "Roll stiff rear [N·m/rad]", "Suspension", "num", "L2,L3"),
    ("anti_dive_front", "Anti-dive front [-]", "Suspension", "num", "L3"),
    ("anti_squat_rear", "Anti-squat rear [-]", "Suspension", "num", "L3"),
    ("camber_per_roll", "Camber/roll gain [rad/rad]", "Suspension", "num", "L3"),
    ("drive_type", "Drive", "Drivetrain", "enum", "L1,L2,L3"),
    ("differential", "Differential", "Drivetrain", "enum", "L2,L3"),
    ("lsd_preload", "LSD preload [-]", "Drivetrain", "num", "L2,L3"),
    ("lsd_ramp", "LSD ramp [-]", "Drivetrain", "num", "L2,L3"),
    ("max_motor_torque", "Max motor torque [N·m]", "Drivetrain", "num", _ALL),
    ("max_brake_torque", "Max brake torque [N·m]", "Drivetrain", "num", _ALL),
    ("brake_bias_front", "Brake bias front [-]", "Drivetrain", "num", "L1,L2,L3"),
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
    ("steer.friction.enabled", "Steer LuGre friction", "Steering", "bool"),
    ("steer.servo_kp", "Servo kp", "Steering", "num"),
    ("steer.servo_kd", "Servo kd", "Steering", "num"),
    ("throttle.dead_time_s", "Throttle dead time [s]", "Throttle", "num"),
    ("throttle.tau_s", "Throttle lag τ [s]", "Throttle", "num"),
    ("throttle.rate_limit", "Throttle rate limit [1/s] (0=off)", "Throttle", "num"),
    ("brake.ch.dead_time_s", "Brake dead time [s]", "Brake", "num"),
    ("brake.ch.tau_s", "Brake lag τ [s]", "Brake", "num"),
    ("brake.dead_zone", "Brake dead-zone [-]", "Brake", "num"),
    ("brake.thermal_enabled", "Brake thermal fade", "Brake", "bool"),
    ("@sensor_delay_s", "Sensor feedback delay [s]", "Feedback", "num"),
    ("@max_substeps", "Solver max substeps", "Solver", "num"),
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


class FigureEight:
    def __init__(self, R=20.0, n=80):
        self.pts = []
        for i in range(n):
            t = 2 * math.pi * i / n
            self.pts.append((R - R * math.cos(t), R * math.sin(t)))
        for i in range(n):
            t = 2 * math.pi * i / n
            self.pts.append((-R + R * math.cos(t), R * math.sin(t)))

    def steer(self, x, y, yaw, vx, wb, prev_idx):
        Ld = max(2.0, 0.45 * max(vx, 1.0))
        n = len(self.pts)
        idx = prev_idx
        while idx < prev_idx + n:
            p = self.pts[idx % n]
            if math.hypot(p[0] - x, p[1] - y) >= Ld:
                break
            idx += 1
        idx %= n
        cp, sp = math.cos(yaw), math.sin(yaw)
        dxw, dyw = self.pts[idx][0] - x, self.pts[idx][1] - y
        dx = cp * dxw + sp * dyw
        dy = -sp * dxw + cp * dyw
        l2 = dx * dx + dy * dy
        if l2 < 1e-6:
            return 0.0, idx
        return max(-0.6, min(0.6, math.atan(2.0 * dy / l2 * wb))), idx


class Runner:
    def __init__(self):
        self.lock = threading.Lock()
        self.cfg = {"level": "L2", "vehicle": "sedan", "v_target": 10.0,
                    "driver": True, "running": True}
        self.dt = 0.005
        self.time_scale = 1.0
        self.manual = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}
        self.latest = {}
        self.path = FigureEight()
        self.vp = vdsim.VehicleParams.from_yaml(
            str(REPO / "configs/vehicles/sedan.yaml"))
        self.tp = vdsim.TireParams.from_yaml(
            str(REPO / "configs/tires/default_pacejka.yaml"))
        self.act = vdsim.ActuatorParams()        # all effects off by default
        self.solver = vdsim.SolverParams()
        self.sensor_delay = 0.0
        self._build()
        threading.Thread(target=self._loop, daemon=True).start()

    def _build(self):
        self.sim = vdsim.make_sim_session(
            self.vp, self.tp, self.cfg["level"], nominal_dt=self.dt,
            sensor_delay_s=self.sensor_delay, actuator=self.act,
            solver=self.solver)
        s0 = vdsim.State()
        s0.velocity = [max(0.1, self.cfg["v_target"]), 0.0, 0.0]
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

    def set_sim(self, dt=None, time_scale=None):
        with self.lock:
            if dt is not None and dt > 1e-5:
                self.dt = float(dt)
            if time_scale is not None and time_scale > 0:
                self.time_scale = float(time_scale)

    def set_params(self, which, data):
        with self.lock:
            if which == "vehicle":
                _apply(self.vp, VEHICLE_FIELDS, data)
            elif which == "tire":
                _apply(self.tp, TIRE_FIELDS, data)
            else:
                self._apply_actuator(data)
            self._build()

    def serialize_actuator(self):
        out = []
        for attr, label, group, kind in ACTUATOR_FIELDS:
            if attr == "@sensor_delay_s":
                val = float(self.sensor_delay)
            elif attr == "@max_substeps":
                val = float(self.solver.max_substeps)
            elif kind == "bool":
                val = bool(_get_dotted(self.act, attr))
            else:
                val = float(_get_dotted(self.act, attr))
            out.append({"name": attr, "label": label, "group": group,
                        "kind": kind, "levels": ["K", "L1", "L2", "L3"],
                        "value": val})
        return out

    def _apply_actuator(self, data):
        kinds = {f[0]: f[3] for f in ACTUATOR_FIELDS}
        for k, v in data.items():
            kind = kinds.get(k)
            if kind is None:
                continue
            if k == "@sensor_delay_s":
                self.sensor_delay = max(0.0, float(v))
            elif k == "@max_substeps":
                self.solver.max_substeps = max(1, int(float(v)))
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
            self.manual.update(kw)

    def _loop(self):
        fps = 60.0
        pending = 0.0
        nxt = time.monotonic()
        while True:
            with self.lock:
                run, driver = self.cfg["running"], self.cfg["driver"]
                vt, dt, ts = self.cfg["v_target"], self.dt, self.time_scale
                man = dict(self.manual)
                wb = self.vp.wheelbase
            if run:
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
            o = self.sim.output()
            s = o.state
            snap = {"t": o.sim_time, "running": run, "driver": driver,
                    "x": float(s.position[0]), "y": float(s.position[1]),
                    "z": float(s.position[2]),
                    "yaw": s.yaw(), "roll": o.roll, "pitch": o.pitch,
                    "vx": s.vx(), "vy": s.vy(), "r": s.yaw_rate(),
                    "wx": float(s.angular_velocity[0]), "wy": float(s.angular_velocity[1]),
                    "ax": o.ax, "ay": o.ay, "steer": o.steer_applied,
                    "Fz": [float(v) for v in o.Fz],
                    "Ft": [[float(f[0]), float(f[1])] for f in o.tire_forces],
                    "susp": [float(v) for v in s.susp_compression],
                    "level": self.cfg["level"], "vehicle": self.cfg["vehicle"],
                    "v_target": vt, "dt": dt, "time_scale": ts}
            with self.lock:
                self.latest = snap
            nxt += 1.0 / fps
            time.sleep(max(0.0, nxt - time.monotonic()))

    def snapshot(self):
        with self.lock:
            return dict(self.latest)

    def config(self):
        with self.lock:
            c = dict(self.cfg)
            c["dt"] = self.dt
            c["time_scale"] = self.time_scale
            return c


RUNNER = Runner()


class Handler(BaseHTTPRequestHandler):
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
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        elif self.path == "/api/config":
            self._json({"config": RUNNER.config(),
                        "vehicles": VEHICLES, "levels": LEVELS})
        elif self.path == "/api/vehicle":
            self._json({"fields": _serialize(RUNNER.vp, VEHICLE_FIELDS)})
        elif self.path == "/api/tire":
            self._json({"fields": _serialize(RUNNER.tp, TIRE_FIELDS)})
        elif self.path == "/api/actuator":
            self._json({"fields": RUNNER.serialize_actuator()})
        elif self.path == "/api/state":
            self._json(RUNNER.snapshot())
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
            RUNNER.set_sim(dt=body.get("dt"), time_scale=body.get("time_scale"))
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
        elif self.path == "/api/control":
            RUNNER.control(body.get("action", ""))
            self._json({"ok": True})
        elif self.path == "/api/manual":
            RUNNER.set_manual(**body)
            self._json({"ok": True})
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[VDSim GUI] http://{args.host}:{args.port}  (compute here, view in browser)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[VDSim GUI] stopped.")


if __name__ == "__main__":
    main()
