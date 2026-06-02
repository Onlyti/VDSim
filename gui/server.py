#!/usr/bin/env python3
"""VDSim web GUI server (MVP) — stdlib only, no extra pip deps.

Compute runs here (vdsim SimSession); the browser does all visualization and
configuration. State streams to the browser via Server-Sent Events; config and
control go over plain REST. Run on a server, open the URL from any PC.

Usage:
    cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build
    python3 gui/server.py [--port 8090]
    # open http://<server>:8090  (or via Tailscale / SSH tunnel)
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
LEVELS = ["L1", "L2", "L3"]


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
        steer = math.atan(2.0 * dy / l2 * wb)
        return max(-0.6, min(0.6, steer)), idx


class Runner:
    """Owns the SimSession; runs the sim loop in a background thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self.cfg = {"level": "L2", "vehicle": "sedan", "v_target": 10.0,
                    "driver": True, "running": True,
                    "sensor_delay_s": 0.0, "steer_tau_s": 0.0}
        self.manual = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}
        self.latest = {}
        self.path = FigureEight()
        self._build()
        threading.Thread(target=self._loop, daemon=True).start()

    def _build(self):
        c = self.cfg
        self.vp = vdsim.VehicleParams.from_yaml(
            str(REPO / f"configs/vehicles/{c['vehicle']}.yaml"))
        tp = vdsim.TireParams.from_yaml(
            str(REPO / "configs/tires/default_pacejka.yaml"))
        self.sim = vdsim.make_sim_session(
            self.vp, tp, c["level"], sensor_delay_s=c["sensor_delay_s"])
        s0 = vdsim.State()
        s0.velocity = [max(0.1, c["v_target"]), 0.0, 0.0]
        self.sim.reset(s0)
        self.prev_idx = 0

    def reconfigure(self, **kw):
        with self.lock:
            for k, v in kw.items():
                if k in self.cfg:
                    self.cfg[k] = v
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
        dt, fps = 0.005, 60.0
        spf = max(1, int((1.0 / fps) / dt))
        nxt = time.monotonic()
        while True:
            with self.lock:
                run = self.cfg["running"]
                driver = self.cfg["driver"]
                vt = self.cfg["v_target"]
                man = dict(self.manual)
                wb = self.vp.wheelbase
            if run:
                for _ in range(spf):
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
                        cmd.throttle = man["throttle"]
                        cmd.brake = man["brake"]
                        cmd.steer_angle_wheel = man["steer"]
                    self.sim.set_input(cmd)
                    self.sim.tick(dt)
            o = self.sim.output()
            s = o.state
            snap = {
                "t": o.sim_time, "running": run, "driver": driver,
                "x": float(s.position[0]), "y": float(s.position[1]),
                "z": float(s.position[2]),
                "yaw": s.yaw(), "roll": o.roll, "pitch": o.pitch,
                "vx": s.vx(), "vy": s.vy(), "r": s.yaw_rate(),
                "ax": o.ax, "ay": o.ay, "steer": o.steer_applied,
                "Fz": [float(v) for v in o.Fz],
                "level": self.cfg["level"], "vehicle": self.cfg["vehicle"],
                "v_target": vt,
            }
            with self.lock:
                self.latest = snap
            nxt += 1.0 / fps
            time.sleep(max(0.0, nxt - time.monotonic()))

    def snapshot(self):
        with self.lock:
            return dict(self.latest)

    def config(self):
        with self.lock:
            return dict(self.cfg)


RUNNER = Runner()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
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
                    payload = json.dumps(RUNNER.snapshot())
                    self.wfile.write(f"data: {payload}\n\n".encode())
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
