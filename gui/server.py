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

# Editable parameter schema: (attr, label, group, is_array, applicable_levels)
VEHICLE_FIELDS = [
    ("mass", "Mass [kg]", "Mass", False, _ALL),
    ("mass_sprung", "Sprung mass [kg]", "Mass", False, "L3"),
    ("wheelbase", "Wheelbase [m]", "Geometry", False, _ALL),
    ("cg_to_front", "CG→front [m]", "Geometry", False, _ALL),
    ("cg_to_rear", "CG→rear [m]", "Geometry", False, _ALL),
    ("track_front", "Track front [m]", "Geometry", False, "L2,L3"),
    ("track_rear", "Track rear [m]", "Geometry", False, "L2,L3"),
    ("cg_height", "CG height [m]", "Geometry", False, "L1,L2,L3"),
    ("wheel_radius_nominal", "Wheel radius [m]", "Geometry", False, _ALL),
    ("spring_stiffness", "Spring stiffness [N/m]", "Suspension", True, "L3"),
    ("damper_coefficient", "Damper [N·s/m]", "Suspension", True, "L3"),
    ("unsprung_mass", "Unsprung mass [kg]", "Suspension", True, "L3"),
    ("roll_stiffness_front", "Roll stiff front [N·m/rad]", "Suspension", False, "L2,L3"),
    ("roll_stiffness_rear", "Roll stiff rear [N·m/rad]", "Suspension", False, "L2,L3"),
    ("anti_dive_front", "Anti-dive front [-]", "Suspension", False, "L3"),
    ("anti_squat_rear", "Anti-squat rear [-]", "Suspension", False, "L3"),
    ("max_motor_torque", "Max motor torque [N·m]", "Drivetrain", False, _ALL),
    ("max_brake_torque", "Max brake torque [N·m]", "Drivetrain", False, _ALL),
    ("brake_bias_front", "Brake bias front [-]", "Drivetrain", False, "L1,L2,L3"),
    ("steering_ratio", "Steering ratio [-]", "Steering", False, "L1,L2,L3"),
    ("max_steer_angle_wheel", "Max steer [rad]", "Steering", False, _ALL),
    ("ackerman_percent", "Ackermann [%]", "Steering", False, "L2,L3"),
    ("aero_drag_coeff", "Drag coeff [-]", "Aero", False, "L1,L2,L3"),
    ("frontal_area", "Frontal area [m²]", "Aero", False, "L1,L2,L3"),
    ("aero_lift_front", "Lift coeff front [-]", "Aero", False, "L1,L2,L3"),
    ("aero_lift_rear", "Lift coeff rear [-]", "Aero", False, "L1,L2,L3"),
]
TIRE_FIELDS = [
    ("B_long", "B long", "Longitudinal", False),
    ("C_long", "C long", "Longitudinal", False),
    ("D_long", "D long", "Longitudinal", False),
    ("E_long", "E long", "Longitudinal", False),
    ("B_lat", "B lat", "Lateral", False),
    ("C_lat", "C lat", "Lateral", False),
    ("D_lat", "D lat", "Lateral", False),
    ("E_lat", "E lat", "Lateral", False),
    ("mu_nominal", "μ nominal", "General", False),
    ("Fz_nominal", "Fz nominal [N]", "General", False),
    ("cornering_stiffness", "Cornering stiffness [N/rad]", "General", False),
    ("rolling_resistance", "Rolling resistance", "General", False),
    ("pneumatic_trail", "Pneumatic trail [m]", "Aligning", False),
    ("trail_falloff_alpha", "Trail falloff α [rad]", "Aligning", False),
    ("camber_stiffness", "Camber stiffness [1/rad]", "Camber", False),
    ("load_sensitivity", "Load sensitivity", "General", False),
    ("relaxation_length_lat", "Relaxation len lat [m]", "Transient", False),
    ("relaxation_length_long", "Relaxation len long [m]", "Transient", False),
]


def _serialize(obj, fields):
    out = []
    for f in fields:
        attr, label, group, is_arr = f[0], f[1], f[2], f[3]
        levels = f[4] if len(f) > 4 else "L1,L2,L3"   # tire: dynamic models only
        v = getattr(obj, attr)
        out.append({"name": attr, "label": label, "group": group,
                    "array": is_arr, "levels": levels.split(","),
                    "value": [float(x) for x in v] if is_arr else float(v)})
    return out


def _apply(obj, fields, data):
    arr = {f[0] for f in fields if f[3]}
    for k, v in data.items():
        if not hasattr(obj, k):
            continue
        if k in arr:
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
        self._build()
        threading.Thread(target=self._loop, daemon=True).start()

    def _build(self):
        self.sim = vdsim.make_sim_session(self.vp, self.tp, self.cfg["level"],
                                          nominal_dt=self.dt)
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
            else:
                _apply(self.tp, TIRE_FIELDS, data)
            self._build()

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
