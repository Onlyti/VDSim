import os
import re
from pathlib import Path

from runner.autopilot import WaypointPath
from runner.config import REPO, prepare_gui_run_dir
from runner.params_io import apply_fields, flat_actuator, flat_sensors, params_dict
from runner.params_schema import ENUM_MAPS, TIRE_FIELDS, VEHICLE_FIELDS
from runner.suspension import (
    strip_fleet_susp_if_not_l3,
    susp_rel_path,
    susp_stem_from_ref,
    suspension_default_for_vehicle,
    validate_fleet_updates,
)


class DraftMixin:
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
            strip_fleet_susp_if_not_l3(self.fleet_spec[-1])
        if not self.fleet_spec:
            raise ValueError(f"scenario '{name}' has no vehicles")
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

    def _run_config_gui_meta(self):
        return {
            "version": 1,
            "live_vid": self.live_vid,
            "driver": bool(self.cfg["driver"]),
            "dt": float(self.dt),
            "time_scale": float(self.time_scale),
            "integrator": next((k for k, v in ENUM_MAPS["integrator"].items()
                                if v == self.solver.integrator), "rk4"),
            "max_substeps": int(self.solver.max_substeps),
            "fleet_overrides": {str(k): v for k, v in self.fleet_overrides.items()},
            "vehicle": params_dict(self.vp, VEHICLE_FIELDS),
            "tire": params_dict(self.tp, TIRE_FIELDS),
            "sensors": flat_sensors(self.sensors),
            "actuator": flat_actuator(self.act, self.sensor_delay),
            "cosim_attach": bool(self.cfg.get("cosim_attach", False)),
            "cosim_host": str(self.cfg.get("cosim_host", "127.0.0.1")),
            "cosim_cmd_port": int(self.cfg.get("cosim_cmd_port", 7401)),
            "cosim_state_port": int(self.cfg.get("cosim_state_port", 7402)),
            "telemetry": self._telemetry_export_unlocked(),
            "infra_sensors": list(self.infra_sensors),
        }

    def _run_config_draft_doc(self):
        with self.lock:
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
                "name": "run",
                "rate": 1.0 / self.dt if self.dt > 1e-6 else 200.0,
                "cmd_timeout": float(self.cosim.cfg.get("cmd_timeout", 0.1)),
                "mu": float(self.cfg["road_mu"]),
                "grade": float(self.cfg["road_grade"]),
                "bank": float(self.cfg["road_bank"]),
                "v_target": float(self.cfg["v_target"]),
                "path_preset": self.path_preset,
                "vehicles": vehs,
                "gui": self._run_config_gui_meta(),
            }
            if self.path_preset == "custom":
                doc["path_pts"] = [[float(p[0]), float(p[1])] for p in self.path.pts]
            return doc

    def _materialize_run_config(self):
        import yaml
        with self.lock:
            run_dir = prepare_gui_run_dir(Path(self.cosim._tmp))
            self.cosim._tmp = str(run_dir)
            fleet = self._fleet_launch()
            road = {"mu": self.cfg["road_mu"], "mu_right": self.cfg["road_mu_right"],
                    "mu_boundary": self.cfg["road_boundary"], "grade": self.cfg["road_grade"],
                    "bank": self.cfg["road_bank"], "rough_amp": self.cfg["road_rough_amp"],
                    "rough_wl": self.cfg["road_rough_wl"]}
            sensors = self.sensors if self.sensors.enabled else None
            rate = 1.0 / self.dt if self.dt > 1e-6 else 200.0
            cmd_timeout = float(self.cosim.cfg.get("cmd_timeout", 0.1))
            wy = self.cosim._write_world_yaml(
                fleet, road, self.terrain, sensors, self.sensor_delay, rate, cmd_timeout)
            doc = yaml.safe_load(Path(wy).read_text())
            doc["name"] = "run"
            doc["time_scale"] = float(self.time_scale)
            doc["v_target"] = float(self.cfg["v_target"])
            doc["path_preset"] = self.path_preset
            if self.path_preset == "custom":
                doc["path_pts"] = [[float(p[0]), float(p[1])] for p in self.path.pts]
            if self.infra_sensors:
                doc["infra_sensors"] = list(self.infra_sensors)
            doc["gui"] = self._run_config_gui_meta()
            run_path = Path(self.cosim._tmp) / "run_config.yaml"
            run_path.write_text(yaml.safe_dump(doc, sort_keys=False))
            self._last_run_config = str(run_path.resolve())
            self._time_scale_path = run_path.parent / "time_scale"
            self._write_live_time_scale()
            return str(run_path), doc

    def get_setup(self, include_geom=True, include_scenarios=True):
        with self.lock:
            road = {"mu": self.cfg["road_mu"], "mu_right": self.cfg["road_mu_right"],
                    "mu_boundary": self.cfg["road_boundary"], "grade": self.cfg["road_grade"],
                    "bank": self.cfg["road_bank"], "rough_amp": self.cfg["road_rough_amp"],
                    "rough_wl": self.cfg["road_rough_wl"]}
            fleet = self._fleet_rows(include_geom=include_geom)
            out = {
                "running": self.cfg["running"],
                "paused": self.cfg.get("paused", False),
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
                "cosim_state_port": int(self.cfg.get("cosim_state_port", 7402)),
                "infra_sensors": list(self.infra_sensors),
            }
            if include_scenarios:
                out["scenarios"] = self.list_scenarios()
            return out

    def save_scenario(self, name, overwrite=False):
        import yaml
        stem = Path(str(name)).strip().stem
        if not stem:
            raise ValueError("scenario name required")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", stem):
            raise ValueError("name: letters, digits, _ . - only")
        path = REPO / "configs" / "scenarios" / f"{stem}.yaml"
        existed = path.is_file()
        if existed and not overwrite:
            raise ValueError(f"scenario '{stem}' already exists")
        doc = self._run_config_draft_doc()
        doc["name"] = stem
        path.write_text(yaml.safe_dump(doc, sort_keys=False))
        self._scenario_cache = None
        rel = f"configs/scenarios/{stem}.yaml"
        return {"ok": True, "name": stem, "path": rel, "overwritten": existed}

    def apply_setup(self, data):
        if "fleet" in data:
            with self.lock:
                validate_fleet_updates(data["fleet"], self.fleet_spec)
        refresh_snap = False
        with self.lock:
            if self.cfg["running"]:
                if data.get("fleet_add") or data.get("fleet_remove") is not None:
                    raise ValueError("stop simulation before adding or removing vehicles")
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
            if "cosim_state_port" in data:
                self.cfg["cosim_state_port"] = int(data["cosim_state_port"])
            if "cosim_cmd_port" in data or "cosim_state_port" in data:
                self._sync_cosim_ports()
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
                    old_level = str(spec.get("level", "L2"))
                    for k in ("x0", "y0", "yaw0", "vx0", "level", "vehicle", "tire",
                              "front_susp", "rear_susp"):
                        if k in upd:
                            spec[k] = upd[k]
                    if "level" in upd:
                        new_level = str(upd["level"])
                        if new_level == "L3" and old_level != "L3":
                            d = suspension_default_for_vehicle(str(spec.get("vehicle", "sedan")))
                            spec["front_susp"] = d["front"]
                            spec["rear_susp"] = d["rear"]
                    if "vehicle" in upd and "front_susp" not in upd and "rear_susp" not in upd:
                        d = suspension_default_for_vehicle(str(upd["vehicle"]))
                        spec["front_susp"] = d["front"]
                        spec["rear_susp"] = d["rear"]
                    if str(spec.get("level", "L2")) == "L3":
                        self._ensure_fleet_parts(spec)
                    strip_fleet_susp_if_not_l3(spec)
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

    def export_simconfig(self):
        return self._run_config_draft_doc()

    def export_run_config(self):
        return self.export_simconfig()

    def _import_simconfig_v2(self, data):
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
        self.fleet_overrides = {
            int(k): v for k, v in (data.get("fleet_overrides") or {}).items()}
        self.fleet_spec = []
        for spec in data["fleet_spec"]:
            row = dict(spec)
            self._ensure_fleet_parts(row)
            strip_fleet_susp_if_not_l3(row)
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
                      ("cosim_cmd_port", "cosim_cmd_port"),
                      ("cosim_state_port", "cosim_state_port")):
            if k in data:
                self.cfg[ck] = data[k]
        if any(k in data for k in ("cosim_cmd_port", "cosim_state_port")):
            self._sync_cosim_ports()
        self._sync_live_from_fleet()
        if "vehicle" in data:
            apply_fields(self.vp, VEHICLE_FIELDS, data["vehicle"])
        if "tire" in data:
            apply_fields(self.tp, TIRE_FIELDS, data["tire"])
        if "sensors" in data:
            self._apply_sensors(data["sensors"])
        if "actuator" in data:
            self._apply_actuator(data["actuator"])
        if "infra_sensors" in data:
            self.infra_sensors = list(data["infra_sensors"] or [])
        if data.get("telemetry"):
            self._telemetry_import_unlocked(data["telemetry"])

    def _import_run_config_doc(self, data):
        for key, ck in (("mu", "road_mu"), ("grade", "road_grade"), ("bank", "road_bank")):
            if data.get(key) is not None:
                self.cfg[ck] = float(data[key])
        if data.get("v_target") is not None:
            self.cfg["v_target"] = float(data["v_target"])
        if data.get("path_preset"):
            preset = str(data["path_preset"])
            if preset == "custom" and data.get("path_pts"):
                pts = [(float(p[0]), float(p[1])) for p in data["path_pts"]]
                if len(pts) >= 2:
                    self.path = WaypointPath(pts)
                    self.path_preset = "custom"
            elif preset != "custom":
                self.set_path_preset(preset)
        self.fleet_spec = []
        for v in data.get("vehicles", []):
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
            strip_fleet_susp_if_not_l3(self.fleet_spec[-1])
        self._ensure_ports()
        gui = data.get("gui") or {}
        if gui.get("live_vid") is not None:
            self.live_vid = int(gui["live_vid"])
        if "driver" in gui:
            self.cfg["driver"] = bool(gui["driver"])
        if gui.get("dt") and float(gui["dt"]) > 1e-5:
            self.dt = float(gui["dt"])
        if gui.get("time_scale") and float(gui["time_scale"]) > 0:
            self.time_scale = float(gui["time_scale"])
        if gui.get("integrator") in ENUM_MAPS["integrator"]:
            self.solver.integrator = ENUM_MAPS["integrator"][gui["integrator"]]
        if "max_substeps" in gui:
            self.solver.max_substeps = max(1, int(gui["max_substeps"]))
        if gui.get("fleet_overrides"):
            self.fleet_overrides = {int(k): v for k, v in gui["fleet_overrides"].items()}
        for k, ck in (("cosim_attach", "cosim_attach"),
                      ("cosim_host", "cosim_host"),
                      ("cosim_cmd_port", "cosim_cmd_port"),
                      ("cosim_state_port", "cosim_state_port")):
            if k in gui:
                self.cfg[ck] = gui[k]
        if any(k in gui for k in ("cosim_cmd_port", "cosim_state_port")):
            self._sync_cosim_ports()
        self._sync_live_from_fleet()
        if gui.get("vehicle"):
            apply_fields(self.vp, VEHICLE_FIELDS, gui["vehicle"])
        if gui.get("tire"):
            apply_fields(self.tp, TIRE_FIELDS, gui["tire"])
        if gui.get("sensors"):
            self._apply_sensors(gui["sensors"])
        if gui.get("actuator"):
            self._apply_actuator(gui["actuator"])
        if data.get("infra_sensors") is not None:
            self.infra_sensors = list(data["infra_sensors"])
        elif gui.get("infra_sensors") is not None:
            self.infra_sensors = list(gui["infra_sensors"])
        if gui.get("telemetry"):
            self._telemetry_import_unlocked(gui["telemetry"])

    def import_run_config(self, data):
        with self.lock:
            if int(data.get("version", 0)) == 2 and data.get("fleet_spec"):
                self._import_simconfig_v2(data)
            elif data.get("vehicles"):
                self._import_run_config_doc(data)
            else:
                raise ValueError("unrecognized run config")
            self._rebuild_if_running()
        return self.config()

    def import_simconfig(self, data):
        return self.import_run_config(data)
