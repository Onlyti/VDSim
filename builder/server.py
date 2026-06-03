#!/usr/bin/env python3
"""VDSim experiment authoring tool — backend.

A dedicated web tool (separate from the PoC sim GUI) to author the four experiment
artifacts as validated YAML configs that vdsim_lab / make_sim_session consume:

    Vehicle   -> configs/vehicles/<name>.yaml      (VehicleParams)
    Sensors   -> configs/sensors/<name>.yaml       (sensor suite: pose+type+noise)
    Map       -> configs/maps/<name>.yaml          (driving line + width + surface)
    Scenario  -> configs/experiments/<name>.yaml   (vehicle+map+maneuver+sensors+run)

Endpoints: list / load / save / validate per kind, plus map + vehicle preview.

    python3 builder/server.py --port 8200    # http://<host>:8200
"""
import argparse
import json
import math
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "examples"))
sys.path.insert(0, str(REPO / "build" / "python"))

KIND_DIR = {
    "vehicles": REPO / "configs" / "vehicles",
    "tires": REPO / "configs" / "tires",
    "roads": REPO / "configs" / "roads",
    "sensors": REPO / "configs" / "sensors",
    "maps": REPO / "configs" / "maps",
    "experiments": REPO / "configs" / "experiments",
}


def _names(kind):
    d = KIND_DIR.get(kind)
    if not d or not d.is_dir():
        return []
    return sorted(p.stem for p in d.glob("*.yaml") if p.name != "README.md")


def _load(kind, name):
    p = KIND_DIR[kind] / f"{name}.yaml"
    return yaml.safe_load(open(p)) if p.is_file() else None


def _save(kind, name, data):
    d = KIND_DIR[kind]
    d.mkdir(parents=True, exist_ok=True)
    safe = "".join(c for c in name if c.isalnum() or c in "_-")
    p = d / f"{safe}.yaml"
    with open(p, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    return str(p.relative_to(REPO)), safe


# --------------------------------------------------------------------------- #
# Map driving-line resolution (shape / import) -> centerline points
# --------------------------------------------------------------------------- #
def resolve_line(spec):
    """spec.driving_line -> [[x,y],...]. Delegates to vdsim_lab so the authoring
    tool and the runner share one resolver."""
    import vdsim_lab
    return vdsim_lab.resolve_line(spec.get("driving_line", {}))


# --------------------------------------------------------------------------- #
# Validation / preview (uses vdsim + vdsim_lab)
# --------------------------------------------------------------------------- #
def validate(kind, data):
    try:
        import vdsim
        if kind == "vehicles":
            tmp = REPO / "configs" / "vehicles" / "_tmp_validate.yaml"
            yaml.safe_dump(data, open(tmp, "w"))
            vdsim.VehicleParams.from_yaml(str(tmp)); tmp.unlink()
            return {"ok": True}
        if kind == "maps":
            pts = resolve_line(data)
            if len(pts) < 2:
                return {"ok": False, "msg": "driving line has < 2 points"}
            return {"ok": True, "pts": len(pts)}
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "msg": str(e)[:200]}


def vehicle_preview(data):
    """Quick metrics: peak accel, top-of-window speed."""
    import vdsim
    tmp = REPO / "configs" / "vehicles" / "_tmp_prev.yaml"
    yaml.safe_dump(data, open(tmp, "w"))
    try:
        vp = vdsim.VehicleParams.from_yaml(str(tmp))
    finally:
        tmp.unlink(missing_ok=True)
    tp = vdsim.TireParams.from_yaml(str(REPO / "configs/tires/default_pacejka.yaml"))
    sess = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=0.005, mu=1.0)
    s = vdsim.make_init_state(x=0, y=0, yaw=0, v=0.0, wheel_radius=vp.wheel_radius_nominal)
    sess.reset(s)
    c = vdsim.CmdL4(); c.throttle = 1.0
    ax0 = None
    for k in range(1400):
        sess.set_input(c); sess.tick(0.005)
        o = sess.output()
        if k == 40:
            ax0 = o.ax
    return {"mass": vp.mass, "wheelbase": vp.wheelbase,
            "peak_ax": round(ax0 or 0.0, 2), "vx_7s": round(o.state.vx(), 1)}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):  # quiet
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n)) if n else {}

    def do_GET(self):
        from urllib.parse import urlparse, parse_qs
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path in ("/", "/index.html"):
            html = (HERE / "index.html").read_bytes()
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html))); self.end_headers()
            self.wfile.write(html)
        elif u.path == "/api/list":
            self._json({"names": _names(q.get("kind", [""])[0])})
        elif u.path == "/api/load":
            self._json({"data": _load(q.get("kind", [""])[0], q.get("name", [""])[0])})
        elif u.path == "/api/presets":
            self._json({k: _names(k) for k in KIND_DIR})
        else:
            self.send_response(404); self.end_headers()

    def do_POST(self):
        b = self._body()
        if self.path == "/api/save":
            v = validate(b["kind"], b["data"])
            if not v.get("ok"):
                self._json({"ok": False, "msg": "validation failed: " + v.get("msg", "")})
                return
            rel, name = _save(b["kind"], b["name"], b["data"])
            self._json({"ok": True, "path": rel, "name": name})
        elif self.path == "/api/validate":
            self._json(validate(b["kind"], b["data"]))
        elif self.path == "/api/preview/map":
            try:
                self._json({"ok": True, "pts": resolve_line(b["data"])})
            except Exception as e:
                self._json({"ok": False, "msg": str(e)[:200]})
        elif self.path == "/api/preview/vehicle":
            try:
                self._json({"ok": True, **vehicle_preview(b["data"])})
            except Exception as e:
                self._json({"ok": False, "msg": str(e)[:200]})
        else:
            self.send_response(404); self.end_headers()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8200)
    args = ap.parse_args()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), H)
    print(f"[builder] authoring tool on http://0.0.0.0:{args.port}")
    srv.serve_forever()


if __name__ == "__main__":
    main()
