import os
import re
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path

import protocol as vds1

from runner.config import REPO, gui_run_dir

COSIM_BIN = REPO / "build" / "bin" / ("vdsim_realtime.exe" if os.name == "nt"
                                      else "vdsim_realtime")
COSIM_CMD_PORT = 7401
COSIM_STATE_PORT = 7402

_KIN_WARN_MARKERS = ("kinematics attach failed", "front susp", "rear susp")
_KIN_LOG_TAIL = 16384


def scan_kinematics_warnings(log_path, tail_only=True):
    warnings = []
    try:
        path = Path(log_path)
        size = path.stat().st_size
        with path.open("rb") as f:
            if tail_only and size > _KIN_LOG_TAIL:
                f.seek(-_KIN_LOG_TAIL, 2)
            text = f.read().decode("utf-8", errors="replace")
    except OSError:
        return warnings
    for line in text.splitlines():
        if "[vdsim_realtime]" not in line:
            continue
        if any(m in line for m in _KIN_WARN_MARKERS):
            warnings.append(line.strip())
    return warnings


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


def cleanup_stale_plant(cmd_port=7401):
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


def write_terrain(path, terrain):
    import struct
    import numpy as np
    H = np.asarray(terrain["H"], dtype="<f8")
    ny, nx = H.shape
    with open(path, "wb") as f:
        f.write(struct.pack("<ii", int(nx), int(ny)))
        f.write(struct.pack("<dddd", float(terrain["x0"]), float(terrain["y0"]),
                            float(terrain["dx"]), float(terrain["dy"])))
        f.write(H.tobytes())


class CosimBridge:
    """Launches the binary vdsim_realtime, consumes its STATE packets for the
    3D view, and relays control as CMD packets (per cosim_protocol.hpp).

    The GUI configures and runs the real co-sim server; the Python playground sim
    is bypassed while the bridge is active so there is one source of truth.
    """
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
        self._tmp = str(gui_run_dir())
        self._stop = threading.Event()
        self.attach_only = False
        self.cmd_host = "127.0.0.1"
        self._plant_log = None
        self._run_since = None
        self.kinematics_warnings = []
        self._kin_warn_pre = []
        self._kin_full_scanned = False

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
        self._kin_full_scanned = False
        self.proc = subprocess.Popen(args, stdout=self._plant_log,
                                     stderr=subprocess.STDOUT, cwd=str(REPO))
        self.started_t = time.monotonic()
        threading.Thread(target=self._rx_loop, args=(self._rx,), daemon=True).start()

    def refresh_kinematics_warnings(self):
        merged = list(self._kin_warn_pre)
        if self._plant_log is not None:
            tail_only = self._kin_full_scanned
            for w in scan_kinematics_warnings(self._plant_log.name, tail_only=tail_only):
                if w not in merged:
                    merged.append(w)
            self._kin_full_scanned = True
        self.kinematics_warnings = merged
        return self.kinematics_warnings

    def _road_cli(self, args, road, terrain, sensors, sensor_delay):
        if terrain is not None:
            tf = os.path.join(self._tmp, "terrain.bin")
            write_terrain(tf, terrain)
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
            write_terrain(tf, terrain)
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
            self._kin_warn_pre = []
            self.kinematics_warnings = []
            self._kin_full_scanned = False
        return self.status()

    def attach(self, host="127.0.0.1", cmd_port=COSIM_CMD_PORT,
               state_port=COSIM_STATE_PORT):
        if self.running():
            self.stop()
        local = str(host).lower() in ("127.0.0.1", "localhost", "::1")
        if local:
            cleanup_stale_plant(int(cmd_port))
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
