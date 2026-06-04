"""Communication router — realizes a comms config (configs/comms/*.yaml).

Routes simulation data per a declarative config: each OUT channel packs one
source (ego.state / ego.sensor.<id>) into its template and sends to one or more
destinations (fan-out); each IN channel listens on a port for control from any
sender (fan-in). The source type / template fixes the packet layout; the config
never hand-codes bytes.

    router = Router("hil_setup")                 # configs/comms/hil_setup.yaml
    router.route_out({"ego.state": sim.state(), "ego.sensor.gnss": sim.get_data("gnss")})
    ctrl = router.poll_in()                       # {steer, throttle, brake}

`run_rt(scenario, comms, ...)` is the real-time-comms execution mode: it drives a
Simulation, routes its data out, and applies control from the in-channel.
"""
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
_COMMS = next((c for c in (REPO / "configs" / "comms", Path.cwd() / "configs" / "comms")
               if c.is_dir()), REPO / "configs" / "comms")
_VDS1 = 0x56445331  # "VDS1"


# --------------------------------------------------------------------------- #
# Packet templates: name -> (pack(dict)->bytes, unpack(bytes)->dict | None)
# --------------------------------------------------------------------------- #
def _json_pack(d):
    return json.dumps(d).encode()


def _json_unpack(b):
    try:
        return json.loads(b)
    except Exception:
        return {}


def _nmea_chk(s):
    c = 0
    for ch in s:
        c ^= ord(ch)
    return c


def _nmea_gga(d):
    # sim has local x/y, not lat/lon -> a proprietary $VDSXY sentence (NMEA-framed)
    body = f"VDSXY,{d.get('x', 0):.3f},{d.get('y', 0):.3f},{d.get('vx', 0):.3f},{d.get('vy', 0):.3f}"
    return f"${body}*{_nmea_chk(body):02X}\r\n".encode()


def _vds1_cmd_pack(d):
    return struct.pack("<Iddd", _VDS1, float(d.get("steer", 0)),
                       float(d.get("throttle", 0)), float(d.get("brake", 0)))


def _vds1_cmd_unpack(b):
    if len(b) < 28:
        return {}
    magic, st, th, br = struct.unpack_from("<Iddd", b, 0)
    return {"steer": st, "throttle": th, "brake": br} if magic == _VDS1 else {}


def _imu_raw(d):
    return struct.pack("<dddd", d.get("ax", 0), d.get("ay", 0), d.get("wz", 0), 0.0)


TEMPLATES = {
    "json":       (_json_pack, _json_unpack),
    "vds1_state": (_json_pack, _json_unpack),   # rich state as JSON (alias)
    "vds1_cmd":   (_vds1_cmd_pack, _vds1_cmd_unpack),
    "nmea_gga":   (_nmea_gga, None),
    "imu_raw":    (_imu_raw, None),
}


class Router:
    def __init__(self, cfg):
        if isinstance(cfg, str):
            cfg = yaml.safe_load(open(_COMMS / f"{cfg}.yaml"))
        self.out = []   # (source, pack_fn, [dest], sock)
        self.inp = []   # (sock, unpack_fn)
        for ch in cfg.get("channels", []):
            tmpl = TEMPLATES.get(ch.get("template", "json"), TEMPLATES["json"])
            if ch.get("direction") == "in":
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setblocking(False)
                s.bind(("0.0.0.0", int(ch["listen"]["port"])))
                self.inp.append((s, tmpl[1]))
            else:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                dests = [(d["ip"], int(d["port"])) for d in ch.get("to", [])]
                self.out.append((ch["source"], tmpl[0], dests, s))

    def route_out(self, sources):
        for source, pack, dests, s in self.out:
            d = sources.get(source)
            if d is None:
                continue
            pkt = pack(d)
            for dst in dests:
                try:
                    s.sendto(pkt, dst)
                except OSError:
                    pass

    def poll_in(self):
        ctrl = {}
        for s, unpack in self.inp:
            last = None
            while True:
                try:
                    last = s.recvfrom(2048)[0]
                except (BlockingIOError, OSError):
                    break
            if last is not None and unpack:
                ctrl.update(unpack(last))
        return ctrl

    def close(self):
        for _src, _p, _d, s in self.out:
            s.close()
        for s, _u in self.inp:
            s.close()


def _sources(sim, suite_ids):
    src = {"ego.state": sim.state()}
    for sid in suite_ids:
        src[f"ego.sensor.{sid}"] = sim.get_data(sid)
    return src


def run_rt(scenario, comms, duration=None, rate=100.0, realtime=True):
    """Real-time-comms mode: drive a Simulation, route data out per `comms`, and
    apply control received on the in-channel each step."""
    sys.path.insert(0, str(REPO / "python"))
    import vdsim_lab as lab
    sim = lab.Simulation(scenario)
    suite_ids = list(getattr(sim, "_types", {}).keys())
    router = Router(comms)
    dt = 1.0 / rate
    dur = duration if duration is not None else sim.duration
    try:
        while not sim.done() and sim.time() < dur:
            ctrl = router.poll_in()
            if ctrl:
                sim.set_control(steer=ctrl.get("steer", 0.0),
                                throttle=ctrl.get("throttle", 0.0), brake=ctrl.get("brake", 0.0))
            sim.step(dt)
            router.route_out(_sources(sim, suite_ids))
            if realtime:
                time.sleep(dt)
    finally:
        router.close()
    return sim
