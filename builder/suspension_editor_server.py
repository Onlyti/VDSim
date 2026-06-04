"""
WebSocket backend for the browser-based hardpoint editor.

Protocol:
    Client → Server:
        {"action": "load",  "config": "configs/suspensions/dw_front_sports.yaml"}
            → reply: {"hp": <yaml dict>, "type": "...", "sweep": [...]}

        {"action": "update", "path": "lca.knuckle", "value": [x, y, z]}
            → reply: {"sweep": [...], "geometry": {...}}

        {"action": "save",  "path": "configs/suspensions/edited.yaml"}
            → reply: {"saved": true}

Sweep payload: list of {travel, camber, toe, track_change} rows.
Geometry payload: dict of all hardpoint world positions for 3D rendering.

Run:  python3 builder/suspension_editor_server.py [--port 8765]
"""
import argparse
import asyncio
import json
import math
import sys
from pathlib import Path
import tempfile

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "tools" / "kinematics"))

try:
    import websockets
except ImportError:
    print("Install websockets:  pip install websockets")
    sys.exit(1)

try:
    import vdsim
    USE_NATIVE = True
except ImportError:
    USE_NATIVE = False
    print("[server] vdsim binding not found — falling back to Python solver")

from dw_3d_solver import DW3DSolver


def make_solver(yaml_path, type_):
    if USE_NATIVE:
        if type_ == "double_wishbone":
            return vdsim.create_dw_native_kinematics(yaml_path)
        elif type_ == "macpherson":
            return vdsim.create_mp_native_kinematics(yaml_path)
        elif type_ == "trailing_arm":
            return vdsim.create_ta_native_kinematics(yaml_path)
        elif type_ == "five_link":
            return vdsim.create_5link_native_kinematics(yaml_path)
    # Fallback Python solver: only DW supported here
    with open(yaml_path) as f: hp = yaml.safe_load(f)
    return DW3DSolver(hp)


def sweep(solver, travel_range=0.05, n=21):
    rows = []
    import numpy as np
    for t in np.linspace(-travel_range, travel_range, n):
        if USE_NATIVE:
            o = solver.compute(float(t), 0.0)
            rows.append({"travel": float(t),
                          "camber": float(o.camber),
                          "toe": float(o.toe),
                          "track_change": float(o.track_change)})
        else:
            o = solver.solve(float(t), 0.0)
            if o.get("valid"):
                rows.append({"travel": float(t),
                              "camber": o["camber"],
                              "toe": o["toe"],
                              "track_change": o["track_change"]})
    return rows


def geometry_points(hp):
    """Extract all named hardpoints as a flat list of {name, point} dicts."""
    out = {}
    def walk(d, prefix=""):
        if isinstance(d, dict):
            for k, v in d.items():
                walk(v, f"{prefix}.{k}" if prefix else k)
        elif isinstance(d, list) and len(d) == 3 and all(isinstance(x, (int, float)) for x in d):
            out[prefix] = list(d)
    walk(hp)
    return out


# In-memory session state
class Session:
    def __init__(self):
        self.hp = None
        self.type = None
        self.tmp_yaml = None

    def load(self, config_path: str):
        config_path = str(REPO / config_path) if not Path(config_path).is_absolute() else config_path
        with open(config_path) as f: self.hp = yaml.safe_load(f)
        self.type = self.hp.get("type")
        return self.resync()

    def update(self, path: str, value):
        # path like "lca.knuckle"
        parts = path.split(".")
        d = self.hp
        for p in parts[:-1]: d = d[p]
        d[parts[-1]] = list(value)
        return self.resync()

    def resync(self):
        # Write current hp to temp file, build solver, sweep.
        if self.tmp_yaml is None:
            self.tmp_yaml = tempfile.NamedTemporaryFile(
                suffix=".yaml", mode="w", delete=False)
        else:
            self.tmp_yaml = open(self.tmp_yaml.name, "w")
        yaml.safe_dump(self.hp, self.tmp_yaml)
        self.tmp_yaml.close()
        try:
            solver = make_solver(self.tmp_yaml.name, self.type)
            rows = sweep(solver)
        except Exception as e:
            print(f"[server] solver error: {e}")
            rows = []
        return {
            "hp": self.hp,
            "type": self.type,
            "sweep": rows,
            "geometry": geometry_points(self.hp),
        }


async def handler(websocket):
    session = Session()
    print("[server] client connected")
    try:
        async for message in websocket:
            try:
                msg = json.loads(message)
                action = msg.get("action")
                if action == "load":
                    reply = session.load(msg["config"])
                elif action == "update":
                    reply = session.update(msg["path"], msg["value"])
                elif action == "save":
                    out = Path(msg["path"])
                    if not out.is_absolute():
                        out = REPO / out
                    with open(out, "w") as f:
                        yaml.safe_dump(session.hp, f, sort_keys=False)
                    reply = {"saved": True, "path": str(out)}
                else:
                    reply = {"error": f"unknown action: {action}"}
            except Exception as e:
                reply = {"error": str(e)}
            await websocket.send(json.dumps(reply))
    finally:
        print("[server] client disconnected")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    print(f"[server] starting WebSocket on :{args.port}  "
          f"({'C++ native' if USE_NATIVE else 'Python fallback'} solver)")
    async with websockets.serve(handler, "localhost", args.port):
        await asyncio.Future()    # run forever


if __name__ == "__main__":
    asyncio.run(main())
