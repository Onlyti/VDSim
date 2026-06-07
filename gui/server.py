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
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "build" / "python"))
sys.path.insert(0, str(HERE.parent / "cosim"))
sys.path.insert(0, str(HERE.parent / "python"))

from runner.config import REPO, GUI_RUN_DIR, gui_run_dir, prepare_gui_run_dir
from runner.autopilot import WaypointPath, FigureEight, fig8_pts, compute_vehicle_cmd
from runner.cosim_bridge import (
    CosimBridge, COSIM_BIN, COSIM_CMD_PORT, COSIM_STATE_PORT,
    cleanup_stale_plant as _cleanup_stale_plant,
)
from runner.draft import DraftMixin
from runner.params_schema import (
    ACTUATOR_FIELDS, ENUM_MAPS, LEVELS, SENSOR_FIELDS, TIRE_FIELDS,
    VEHICLE_FIELDS, VEHICLES,
)
from runner.params_io import apply_fields, get_dotted, serialize_fields, set_dotted
from runner.catalog_bridge import (
    catalog_resolver,
    fleet_entry_for_cosim,
    normalize_fleet_spec,
)
from runner.suspension import (
    l3_susp_path_warnings,
    list_l3_kinematics_configs,
    list_suspension_api,
    list_suspension_configs,
    strip_fleet_susp_if_not_l3,
    suspension_default_for_vehicle,
    suspension_schematic,
)
from catalog.ids import BLUEPRINTS, DEFAULT_BLUEPRINT, blueprint_for_vehicle
from api.handler import make_handler
from api.routes import ApiContext

try:
    import vdsim
except ImportError as e:
    sys.exit(f"import vdsim failed ({e}). Build with -DVDSIM_BUILD_PYTHON=ON.")

def parts_registry():
    from runner.catalog_api import catalog_legacy_registry
    comp = REPO / "configs" / "components" / "suspension"
    out = catalog_legacy_registry()
    out["suspension_presets"] = (
        sorted(x.stem for x in comp.glob("*.yaml")) if comp.is_dir() else [])
    return out


LOG_COLS = ["t", "x", "y", "z", "yaw", "roll", "pitch", "vx", "vy", "r",
            "wx", "wy", "ax", "ay", "steer", "Fz0", "Fz1", "Fz2", "Fz3",
            "cmd_throttle", "cmd_brake", "cmd_steer", "source", "level",
            "m_gnss_x", "m_gnss_y", "m_ax", "m_ay", "m_wz", "m_steer",
            "Fx0", "Fx1", "Fx2", "Fx3", "Fy0", "Fy1", "Fy2", "Fy3",
            "kappa0", "kappa1", "kappa2", "kappa3",
            "alpha0", "alpha1", "alpha2", "alpha3"]


def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class VehiclePort:
    """Per-vehicle data-port config: telemetry output + control input."""
    def __init__(self):
        self.tx = {"enabled": False, "rate": 50.0, "send_state": True, "send_cmd": True}
        self.targets = []
        self.in_cmd = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}
        self.applied = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}
        self.driver = True
        self.prev_idx = 0
        self.io_last_t = None
        self._tx_last = 0.0


class Runner(DraftMixin):
    def __init__(self):
        self.lock = threading.Lock()
        self.cfg = {"level": "L2", "vehicle": "sedan", "v_target": 10.0,
                    "driver": True, "running": False, "paused": False,
                    "init_x": 0.0, "init_y": 0.0, "init_yaw": 0.0, "init_v": 0.0,
                    "road_mu": 1.0, "road_mu_right": -1.0, "road_boundary": 0.0,
                    "road_grade": 0.0, "road_bank": 0.0,
                    "road_rough_amp": 0.0, "road_rough_wl": 4.0,
                    "cosim_attach": False, "cosim_host": "127.0.0.1",
                    "cosim_cmd_port": COSIM_CMD_PORT,
                    "cosim_state_port": COSIM_STATE_PORT}
        self.dt = 0.005
        self.time_scale = 1.0
        self.live_vid = 0
        self.ports = {0: VehiclePort()}
        self.fleet_spec = [
            {"id": 0, "blueprint": DEFAULT_BLUEPRINT, "parts": {}, "level": "L2",
             "vehicle": "sedan", "tire": "default_pacejka",
             "x0": 0.0, "y0": 0.0, "yaw0": 0.0, "vx0": 0.0},
        ]
        normalize_fleet_spec(self.fleet_spec[0])
        self.fleet_overrides = {}
        self.plant_error = None
        self._wait_since = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cosim = CosimBridge()         # binary co-sim server (configured + launched here)
        self.rec_on = False                # logging recorder
        self.rec_rows = []
        self.rec_last = {}
        self.latest = {}
        self._last_run_config = None
        self.path = FigureEight()
        self._catalog = catalog_resolver()
        _live = self._catalog.resolve_blueprint(
            DEFAULT_BLUEPRINT, out_dir=Path(self.cosim._tmp) / "_boot")
        self.vp = vdsim.VehicleParams.from_yaml(str(_live.vehicle_yaml))
        self.tp = vdsim.TireParams.from_yaml(str(_live.tire_yaml))
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
        self._time_scale_path = None
        threading.Thread(target=self._cmd_loop, daemon=True).start()
        threading.Thread(target=self._loop, daemon=True).start()

    def _ensure_ports(self):
        for spec in self.fleet_spec:
            vid = int(spec["id"])
            if vid not in self.ports:
                self.ports[vid] = VehiclePort()

    @staticmethod
    def _ensure_fleet_parts(spec):
        normalize_fleet_spec(spec)

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
        normalize_fleet_spec(spec)
        out = Path(self.cosim._tmp) / f"_vp_{vid}"
        row = fleet_entry_for_cosim(self._catalog, spec, out, self.fleet_overrides)
        return vdsim.VehicleParams.from_yaml(row["vehicle_yaml"])

    def _tp_for_vid(self, vid):
        vid = int(vid)
        if vid == self.live_vid:
            return self.tp
        spec = self._spec_for_vid(vid)
        normalize_fleet_spec(spec)
        out = Path(self.cosim._tmp) / f"_tp_{vid}"
        row = fleet_entry_for_cosim(self._catalog, spec, out, self.fleet_overrides)
        return vdsim.TireParams.from_yaml(row["tire_yaml"])

    def vehicle_geom(self, vid):
        vp = self._vp_for_vid(vid)
        return {"cg_to_front": float(vp.cg_to_front), "cg_to_rear": float(vp.cg_to_rear),
                "track_front": float(vp.track_front), "track_rear": float(vp.track_rear),
                "wheel_radius_nominal": float(vp.wheel_radius_nominal)}

    def _fleet_launch(self):
        fleet = []
        resolve_dir = Path(self.cosim._tmp) / "_launch"
        resolve_dir.mkdir(parents=True, exist_ok=True)
        for spec in self.fleet_spec:
            normalize_fleet_spec(spec)
            fleet.append(fleet_entry_for_cosim(
                self._catalog, spec, resolve_dir, self.fleet_overrides))
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
        normalize_fleet_spec(spec)
        out = Path(self.cosim._tmp) / "_live_sync"
        row = fleet_entry_for_cosim(
            self._catalog, spec, out, self.fleet_overrides)
        self.vp = vdsim.VehicleParams.from_yaml(row["vehicle_yaml"])
        self.tp = vdsim.TireParams.from_yaml(row["tire_yaml"])

    def _live_run_dir(self):
        if self._time_scale_path:
            return self._time_scale_path.parent
        if self._last_run_config:
            d = Path(self._last_run_config).parent
            if d.is_dir():
                return d
        return Path(self.cosim._tmp)

    def _write_live_time_scale(self):
        try:
            p = self._time_scale_path or (self._live_run_dir() / "time_scale")
            p.parent.mkdir(parents=True, exist_ok=True)
            data = f"{float(self.time_scale):.6g}\n".encode()
            p.write_bytes(data)
            fd = os.open(str(p), os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        except OSError:
            pass

    def set_fleet_driver(self, vid, driver):
        vid = int(vid)
        with self.lock:
            p = self.ports.get(vid)
            if p is None:
                raise ValueError(f"unknown vehicle {vid}")
            p.driver = bool(driver)
            return bool(p.driver)

    def _build(self):
        self._ensure_ports()
        self._sync_cosim_ports()
        if self.cfg.get("cosim_attach"):
            self.cosim.attach(self.cfg.get("cosim_host", "127.0.0.1"),
                              int(self.cfg.get("cosim_cmd_port", COSIM_CMD_PORT)),
                              int(self.cfg.get("cosim_state_port", COSIM_STATE_PORT)))
            self.prev_idx = 0
            return
        _cleanup_stale_plant(int(self.cosim.cfg.get("cmd_port", COSIM_CMD_PORT)))
        self.cosim.set_kinematics_pre_warnings(l3_susp_path_warnings(self.fleet_spec))
        scenario = self._last_run_config
        if not scenario or not Path(scenario).is_file():
            self._materialize_run_config()
            scenario = self._last_run_config
        if self.cosim.available():
            c = self.cosim.cfg
            args = [str(COSIM_BIN), f"--scene={scenario}",
                    f"--cmd-port={int(c['cmd_port'])}", "--state-ip=127.0.0.1",
                    f"--state-port={int(c['state_port'])}",
                    f"--rate={1.0 / self.dt if self.dt > 1e-6 else 200.0}",
                    f"--time-scale={float(self.time_scale)}",
                    f"--cmd-timeout={float(c.get('cmd_timeout', 0.1))}"]
            self._time_scale_path = Path(scenario).resolve().parent / "time_scale"
            self._write_live_time_scale()
            self.cosim._launch(args, int(c["state_port"]))
        else:
            print("[VDSim GUI] vdsim_realtime binary missing — build the C++ tree first")
        self.prev_idx = 0

    def _rebuild_if_running(self):
        if self.cfg["running"]:
            self._build()

    def load_vehicle(self, name):
        bid = blueprint_for_vehicle(str(name), self.cfg.get("level", "L2"))
        resolved = self._catalog.resolve_blueprint(
            bid, out_dir=Path(self.cosim._tmp) / f"_veh_{name}")
        self.vp = vdsim.VehicleParams.from_yaml(str(resolved.vehicle_yaml))

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
        if time_scale is not None and time_scale > 0:
            self._write_live_time_scale()

    def set_params(self, which, data, vehicle_id=None):
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            if vid == self.live_vid:
                if which == "vehicle":
                    apply_fields(self.vp, VEHICLE_FIELDS, data)
                elif which == "tire":
                    apply_fields(self.tp, TIRE_FIELDS, data)
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
            return serialize_fields(self._vp_for_vid(vid), VEHICLE_FIELDS)

    def serialize_tire(self, vehicle_id=None):
        vid = int(vehicle_id) if vehicle_id is not None else self.live_vid
        with self.lock:
            return serialize_fields(self._tp_for_vid(vid), TIRE_FIELDS)

    def fleet_enriched(self):
        with self.lock:
            out = []
            for spec in self.fleet_spec:
                vid = int(spec["id"])
                row = dict(spec)
                self._ensure_fleet_parts(row)
                row["geom"] = self.vehicle_geom(vid)
                row["driver"] = bool(self.ports[vid].driver) if vid in self.ports else True
                out.append(row)
            return out

    def serialize_sensors(self):
        out = []
        for attr, label, group, kind in SENSOR_FIELDS:
            if kind == "bool":
                val = bool(getattr(self.sensors, attr))
            else:
                val = float(get_dotted(self.sensors, attr))
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
                set_dotted(self.sensors, k, float(v))

    def serialize_actuator(self):
        out = []
        for attr, label, group, kind in ACTUATOR_FIELDS:
            if attr == "@sensor_delay_s":
                val = float(self.sensor_delay)
            elif kind == "bool":
                val = bool(get_dotted(self.act, attr))
            else:
                val = float(get_dotted(self.act, attr))
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
                set_dotted(self.act, k, bool(v))
            else:
                set_dotted(self.act, k, float(v))

    def set_path_preset(self, name):
        self.path_preset = name
        if name == "figure8":
            self.path = FigureEight()
        elif name == "straight":
            self.path = WaypointPath([(-40.0, 0.0), (40.0, 0.0)])

    def _default_fleet_spec(self, vid=0, offset_xy=(0.0, 0.0)):
        veh = str(self.cfg.get("vehicle", "sedan"))
        level = str(self.cfg.get("level", "L2"))
        row = {
            "id": int(vid),
            "blueprint": blueprint_for_vehicle(veh, level),
            "parts": {},
            "vehicle": veh,
            "tire": "default_pacejka",
            "level": level,
            "x0": float(self.cfg.get("init_x", 0.0)) + float(offset_xy[0]),
            "y0": float(self.cfg.get("init_y", 0.0)) + float(offset_xy[1]),
            "yaw0": float(self.cfg.get("init_yaw", 0.0)),
            "vx0": 0.0,
        }
        normalize_fleet_spec(row)
        return row

    def _fleet_add(self):
        if not self.fleet_spec:
            row = self._default_fleet_spec(0)
            strip_fleet_susp_if_not_l3(row)
            self.fleet_spec.append(row)
            self.live_vid = 0
            self._ensure_ports()
            return
        ids = [int(s["id"]) for s in self.fleet_spec]
        nid = max(ids) + 1
        ref = self.fleet_spec[-1]
        row = {
            "id": nid,
            "blueprint": str(ref.get("blueprint", DEFAULT_BLUEPRINT)),
            "parts": dict(ref.get("parts") or {}),
            "vehicle": str(ref.get("vehicle", "sedan")),
            "tire": str(ref.get("tire", "default_pacejka")),
            "level": str(ref.get("level", self.cfg["level"])),
            "x0": float(ref.get("x0", 0.0)) + 3.0,
            "y0": float(ref.get("y0", 0.0)),
            "yaw0": float(ref.get("yaw0", 0.0)),
            "vx0": float(ref.get("vx0", 0.0)),
        }
        if ref.get("front_susp"):
            row["front_susp"] = ref["front_susp"]
        if ref.get("rear_susp"):
            row["rear_susp"] = ref["rear_susp"]
        normalize_fleet_spec(row)
        self.fleet_spec.append(row)
        strip_fleet_susp_if_not_l3(self.fleet_spec[-1])
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
        d = REPO / "configs" / "scenes"
        if not d.is_dir():
            return []
        mtime = max((p.stat().st_mtime for p in d.glob("*.yaml")), default=0.0)
        cached = getattr(self, "_scenario_cache", None)
        if cached and cached[0] == mtime:
            return cached[1]
        names = sorted(p.stem for p in d.glob("*.yaml"))
        self._scenario_cache = (mtime, names)
        return names

    def _fleet_rows(self, include_geom=False):
        rows = []
        for spec in self.fleet_spec:
            row = dict(spec)
            self._ensure_fleet_parts(row)
            if include_geom:
                row["geom"] = self.vehicle_geom(int(spec["id"]))
            vid = int(spec["id"])
            row["driver"] = bool(self.ports[vid].driver) if vid in self.ports else True
            rows.append(row)
        return rows

    @staticmethod
    def _norm_spin(st):
        spin = list(st.get("wheel_spin") or [])
        return (spin + [0.0] * 4)[:4]

    def _fleet_row_from_state(self, vid, st, spec=None):
        if spec is None:
            spec = next((f for f in self.fleet_spec if int(f["id"]) == int(vid)), {})
        spin = self._norm_spin(st)
        return {
            "x": st["x"], "y": st["y"], "z": st["z"],
            "yaw": st["yaw"], "roll": st["roll"], "pitch": st["pitch"],
            "vx": st["vx"], "vy": st["vy"], "r": st["r"],
            "steer": st["steer"], "Fz": st.get("Fz", []),
            "Ft": st.get("Ft", []),
            "wheel_spin": spin,
            "susp": st.get("susp", []),
            "kappa": st.get("kappa", []),
            "alpha": st.get("alpha", []),
            "level": spec.get("level", self.cfg["level"]),
            "vehicle": spec.get("vehicle", ""),
        }

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
            "t": 0.0, "running": False, "paused": False, "setup_mode": True,
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
                self.cfg["paused"] = False
                self.plant_error = None
                self.cosim.stop()
                self.prev_idx = 0
            elif action == "stop":
                self.cfg["running"] = False
                self.cfg["paused"] = False
                self.plant_error = None
                self.cosim.stop()
            elif action == "pause":
                if self.cfg["running"]:
                    self.cfg["paused"] = True
            elif action == "resume":
                if self.cfg["running"]:
                    self.cfg["paused"] = False
            elif action == "start":
                self.cosim.stop()
                self._renumber_fleet()
                self._sync_live_from_fleet()
                self._prune_cosim_states()
                self.prev_idx = 0
                self.plant_error = None
                self.cfg["paused"] = False
                if not self.cfg.get("cosim_attach") and not self.cosim.available():
                    self.cfg["running"] = False
                    self.plant_error = "vdsim_realtime not built (cmake -DVDSIM_BUILD_COSIM=ON)"
                else:
                    start_req = True
        if start_req:
            t0 = time.monotonic()
            self._materialize_run_config()
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
            out = {"running": self.cfg["running"], "paused": self.cfg.get("paused", False),
                   "error": self.plant_error, "run_config": self._last_run_config}
            return out

    def set_manual(self, **kw):
        vid = int(kw.pop("vehicle", self.live_vid))
        with self.lock:
            p = self.ports.get(vid)
            if p is None or p.driver:
                return
            p.in_cmd.update(
                {k: float(v) for k, v in kw.items()
                 if k in ("throttle", "brake", "steer")})

    def _sync_cosim_ports(self):
        cmd = int(self.cfg.get("cosim_cmd_port", COSIM_CMD_PORT))
        st = int(self.cfg.get("cosim_state_port", COSIM_STATE_PORT))
        self.cfg["cosim_cmd_port"] = cmd
        self.cfg["cosim_state_port"] = st
        self.cosim.cfg["cmd_port"] = cmd
        self.cosim.cfg["state_port"] = st

    def comms_info(self):
        with self.lock:
            attach = bool(self.cfg.get("cosim_attach"))
            host = str(self.cfg.get("cosim_host", "127.0.0.1")) if attach else "127.0.0.1"
            cmd = int(self.cfg.get("cosim_cmd_port", COSIM_CMD_PORT))
            st = int(self.cfg.get("cosim_state_port", COSIM_STATE_PORT))
        cs = self.cosim.status()
        return {
            "attach": attach, "host": host, "cmd_port": cmd, "state_port": st,
            "protocol": "VDS1", "protocol_version": 4,
            "plant_running": bool(cs.get("running")),
            "plant_attach": bool(cs.get("attach")),
        }

    def telemetry_config(self):
        with self.lock:
            return {"vehicles": sorted(self.ports), "live": self.live_vid,
                    "configs": {vid: {"tx": dict(p.tx),
                                      "targets": [dict(t) for t in p.targets]}
                                for vid, p in self.ports.items()}}

    def _telemetry_export_unlocked(self):
        return {str(vid): {"tx": dict(p.tx),
                           "targets": [dict(t) for t in p.targets]}
                for vid, p in self.ports.items()}

    def telemetry_export(self):
        with self.lock:
            return self._telemetry_export_unlocked()

    def _telemetry_import_unlocked(self, data):
        if not data:
            return
        self._ensure_ports()
        for vid_s, c in data.items():
            try:
                vid = int(vid_s)
            except (TypeError, ValueError):
                continue
            p = self.ports.get(vid)
            if p is None or not isinstance(c, dict):
                continue
            tx = c.get("tx")
            if isinstance(tx, dict):
                for k in ("enabled", "send_state", "send_cmd"):
                    if k in tx:
                        p.tx[k] = bool(tx[k])
                if "rate" in tx:
                    p.tx["rate"] = max(1.0, min(200.0, float(tx["rate"])))
            if "targets" in c:
                clean = []
                for t in c["targets"] or []:
                    ip = str(t.get("ip", "")).strip()
                    try:
                        port = int(t.get("port", 0))
                    except (TypeError, ValueError):
                        continue
                    if ip and 0 < port < 65536:
                        clean.append({"ip": ip, "port": port})
                p.targets = clean

    def telemetry_import(self, data):
        with self.lock:
            self._telemetry_import_unlocked(data)

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
        now = time.monotonic()
        fleet = snap.get("fleet") or {}
        for vid, p in self.ports.items():
            if not (p.tx["enabled"] and p.targets):
                continue
            if now - p._tx_last < 1.0 / p.tx["rate"]:
                continue
            p._tx_last = now
            live = (vid == self.live_vid)
            row = fleet.get(str(vid)) if fleet else None
            payload = {"veh": vid, "t": snap.get("t", 0.0) if live else (row or {}).get("t", 0.0)}
            if p.tx["send_state"]:
                src = snap if live else row
                if src:
                    for k in ("x", "y", "z", "yaw", "roll", "pitch", "vx", "vy", "r",
                              "wx", "wy", "ax", "ay", "steer", "Fz"):
                        if k in src:
                            payload[k] = src[k]
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
            if p is not None:
                p.driver = False
        return self.snapshot()

    def _send_fleet_cmds(self):
        cmds = []
        with self.lock:
            if (not self.cfg.get("running") or self.cfg.get("paused")
                    or not self.cosim.running()):
                return
            vt = float(self.cfg["v_target"])
            allowed = self._fleet_ids()
            all_cs = {k: v for k, v in self.cosim.states.items() if int(k) in allowed}
            for vid in sorted(allowed):
                cs_v = all_cs.get(vid)
                p = self.ports.get(vid)
                if cs_v is None or p is None:
                    continue
                wb = float(self._vp_for_vid(vid).wheelbase)
                t, b, st, p.prev_idx = compute_vehicle_cmd(
                    self.path, cs_v, p.prev_idx, vt, wb, p.driver, p.in_cmd)
                p.applied = {"throttle": t, "brake": b, "steer": st}
                cmds.append((vid, t, b, st))
        for vid, t, b, st in cmds:
            self.cosim.send_cmd(t, b, st, vehicle_id=vid)

    def _cmd_loop(self):
        while True:
            try:
                self._send_fleet_cmds()
            except Exception:
                pass
            time.sleep(0.005)

    def _loop(self):
        fps = 60.0
        pending = 0.0
        nxt = time.monotonic()
        while True:
            with self.lock:
                run = self.cfg["running"]
                paused = self.cfg.get("paused", False)
                vt, dt, ts = self.cfg["v_target"], self.dt, self.time_scale
                focus = self.live_vid
                focus_driver = self.ports[focus].driver if focus in self.ports else True
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
            fleet = {}
            if all_cs:
                for vid, st in all_cs.items():
                    spec = next((f for f in self.fleet_spec if int(f["id"]) == int(vid)), {})
                    fleet[str(vid)] = self._fleet_row_from_state(vid, st, spec)
            if cs:
                with self.lock:
                    self._wait_since = None
                live_spec = next((f for f in self.fleet_spec if int(f["id"]) == self.live_vid), {})
                live_lv = str(live_spec.get("level", self.cfg["level"]))
                snap = {"t": cs["t"], "running": run, "paused": paused, "setup_mode": False,
                        "driver": focus_driver,
                        "x": cs["x"], "y": cs["y"], "z": cs["z"],
                        "yaw": cs["yaw"], "roll": cs["roll"], "pitch": cs["pitch"],
                        "vx": cs["vx"], "vy": cs["vy"], "r": cs["r"],
                        "wx": cs["wx"], "wy": cs["wy"], "ax": cs["ax"], "ay": cs["ay"],
                        "steer": cs["steer"], "Fz": cs["Fz"], "Ft": cs.get("Ft", []),
                        "wheel_spin": self._norm_spin(cs),
                        "susp": cs.get("susp", []),
                        "rack_torque": cs.get("rack_torque", 0.0),
                        "kappa": cs.get("kappa", []), "alpha": cs.get("alpha", []),
                        "m_gx": cs.get("m_gx", 0.0), "m_gy": cs.get("m_gy", 0.0),
                        "m_ax": cs.get("m_ax", 0.0), "m_ay": cs.get("m_ay", 0.0),
                        "m_wz": cs.get("m_wz", 0.0), "m_steer": cs.get("m_steer", 0.0),
                        "level": live_lv, "vehicle": live_spec.get("vehicle", self.cfg["vehicle"]),
                        "v_target": vt, "dt": 1.0 / max(1.0, self.cosim.cfg["rate"]),
                        "time_scale": ts, "source": "cosim",
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
                snap = {"t": ht, "running": run, "paused": paused, "setup_mode": False,
                        "driver": focus_driver,
                        "x": hx, "y": hy, "z": 0.0,
                        "yaw": hyaw, "roll": 0.0, "pitch": 0.0,
                        "vx": hvx, "vy": 0.0, "r": 0.0, "wx": 0.0, "wy": 0.0,
                        "ax": 0.0, "ay": 0.0, "steer": 0.0, "Fz": [0.0, 0.0, 0.0, 0.0],
                        "Ft": [], "wheel_spin": [], "susp": [], "rack_torque": 0.0,
                        "kappa": [], "alpha": [],
                        "level": self.cfg["level"], "vehicle": self.cfg["vehicle"],
                        "v_target": vt, "dt": dt, "time_scale": ts,
                        "source": "waiting", "cosim_up": cosim_on, "fleet": {}}
                snap["plant_error"] = self.plant_error
            snap["paused"] = paused
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
            if (self.cfg.get("running") and self.cosim.running() and self.cosim.states):
                allowed = self._fleet_ids()
                fleet = {}
                for vid, st in self.cosim.states.items():
                    if int(vid) not in allowed:
                        continue
                    fleet[str(vid)] = self._fleet_row_from_state(vid, st)
                if fleet:
                    snap["fleet"] = fleet
            p = self.ports[self.live_vid]
            snap["veh"] = self.live_vid
            snap["live_vid"] = self.live_vid
            snap["driver"] = bool(p.driver)
            snap["fleet_driver"] = {str(vid): bool(pt.driver)
                                    for vid, pt in self.ports.items()}
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
        snap["comms"] = self.comms_info()
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
        c["comms"] = self.comms_info()
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
                apply_fields(self.tp, TIRE_FIELDS, mapped)
            else:
                bucket = self.fleet_overrides.setdefault(vid, {}).setdefault("tire", {})
                bucket.update(mapped)
            self._rebuild_if_running()
        return {"parsed": len(raw), "mapped": sorted(mapped.keys())}

RUNNER = Runner()


def _qvid(qs, default=None):
    v = qs.get("vehicle_id", [None])[0]
    if v is None or v == "":
        return default
    return int(v)


Handler = make_handler(ApiContext(
    RUNNER, HERE, VEHICLES, LEVELS, COSIM_CMD_PORT, COSIM_STATE_PORT,
    _qvid, parts_registry, list_suspension_api, suspension_default_for_vehicle,
    suspension_schematic,
))


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
