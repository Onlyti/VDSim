import json
import os
import time
from pathlib import Path
from urllib.parse import urlparse

from api.responses import bytes_response, json_response


class ApiContext:
    __slots__ = (
        "runner", "here", "vehicles", "levels", "level_ladder", "cosim_cmd_port",
        "cosim_state_port", "qvid", "parts_registry", "list_suspension_api",
        "suspension_default_for_vehicle", "suspension_schematic", "suspension_kc_plots",
    )

    def __init__(self, runner, here, vehicles, levels, level_ladder, cosim_cmd_port,
                 cosim_state_port, qvid, parts_registry, list_suspension_api,
                 suspension_default_for_vehicle, suspension_schematic, suspension_kc_plots):
        self.runner = runner
        self.here = here
        self.vehicles = vehicles
        self.levels = levels
        self.level_ladder = level_ladder
        self.cosim_cmd_port = cosim_cmd_port
        self.cosim_state_port = cosim_state_port
        self.qvid = qvid
        self.parts_registry = parts_registry
        self.list_suspension_api = list_suspension_api
        self.suspension_default_for_vehicle = suspension_default_for_vehicle
        self.suspension_schematic = suspension_schematic
        self.suspension_kc_plots = suspension_kc_plots


def _scene_list(h, ctx):
    json_response(h, {"scenes": ctx.runner.list_scenarios(),
                      "scenarios": ctx.runner.list_scenarios()})


def _get_catalog(h, qs):
    from runner.catalog_api import catalog_index
    type_filter = (qs.get("type") or [None])[0]
    query = (qs.get("q") or [None])[0]
    json_response(h, catalog_index(type_filter=type_filter, query=query))


def _get_debug(h, qs, ctx):
    r = ctx.runner
    with r.lock:
        fleet_driver = {str(vid): p.driver for vid, p in r.ports.items()}
        cmd_applied = {str(vid): dict(p.applied) for vid, p in r.ports.items()}
        in_cmd = {str(vid): dict(p.in_cmd) for vid, p in r.ports.items()}
        cfg = dict(r.cfg)
        tsp = str(r._time_scale_path) if r._time_scale_path else None
        plant_error = r.plant_error
        time_scale = float(r.time_scale)
        v_target = float(cfg.get("v_target", 0.0))
        last_run = r._last_run_config
    cosim_st = r.cosim.status()
    json_response(h, {
        "running": cfg.get("running"),
        "paused": cfg.get("paused"),
        "setup_mode": not cfg.get("running"),
        "time_scale": time_scale,
        "time_scale_path": tsp,
        "v_target": v_target,
        "driver": cfg.get("driver"),
        "live_vid": r.live_vid,
        "fleet_driver": fleet_driver,
        "cmd_applied": cmd_applied,
        "in_cmd": in_cmd,
        "cosim": {
            "running": cosim_st.get("running"),
            "attach": cosim_st.get("attach"),
            "state_vehicle_ids": sorted(r.cosim.states.keys()),
            "last_state_vid": (r.cosim.last_state or {}).get("vehicle_id"),
            "state_age": cosim_st.get("state_age"),
            "kinematics_warnings": cosim_st.get("kinematics_warnings", []),
        },
        "last_run_config": last_run,
        "plant_error": plant_error,
        "path_preset": r.path_preset,
        "path_pts": len(r.path.pts),
    })


def _get_assembly(h, qs, ctx):
    vid = ctx.qvid(qs, ctx.runner.live_vid)
    slot = (qs.get("slot") or [None])[0]
    candidate = (qs.get("candidate") or [None])[0]
    try:
        json_response(h, {
            "ok": True,
            "assembly": ctx.runner.fleet_assembly(
                vid, preview_slot=slot, preview_candidate=candidate),
        })
    except Exception as e:
        json_response(h, {"ok": False, "error": str(e)}, 400)


GET_EXACT = {
    "/api/config": lambda h, qs, ctx: json_response(h, {
        "config": ctx.runner.config(),
        "vehicles": ctx.vehicles,
        "levels": ctx.levels,
        "level_ladder": ctx.level_ladder,
    }),
    "/api/vehicle": lambda h, qs, ctx: json_response(h, {
        "fields": ctx.runner.serialize_vehicle(ctx.qvid(qs)),
    }),
    "/api/tire": lambda h, qs, ctx: json_response(h, {
        "fields": ctx.runner.serialize_tire(ctx.qvid(qs)),
    }),
    "/api/actuator": lambda h, qs, ctx: json_response(h, {
        "fields": ctx.runner.serialize_actuator(),
    }),
    "/api/sensors": lambda h, qs, ctx: json_response(h, {
        "fields": ctx.runner.serialize_sensors(),
    }),
    "/api/tire/curves": lambda h, qs, ctx: json_response(h, {
        "plots": ctx.runner.tire_curves(ctx.qvid(qs)),
    }),
    "/api/tire/samples": lambda h, qs, ctx: json_response(h, {
        "samples": ctx.runner.tire_samples(),
    }),
    "/api/parts/registry": lambda h, qs, ctx: json_response(h, ctx.parts_registry()),
    "/api/catalog": lambda h, qs, ctx: _get_catalog(h, qs),
    "/api/catalog/assembly": _get_assembly,
    "/api/simconfig": lambda h, qs, ctx: json_response(h, ctx.runner.export_simconfig()),
    "/api/fleet": lambda h, qs, ctx: json_response(h, {
        "fleet": ctx.runner.fleet_enriched(),
        "live_vid": ctx.runner.live_vid,
        "vehicles": sorted(ctx.runner.ports),
    }),
    "/api/runconfig/draft": lambda h, qs, ctx: json_response(h, {
        "ok": True,
        "draft": ctx.runner.export_run_config(),
    }),
    "/api/runconfig/last": lambda h, qs, ctx: json_response(h, {
        "ok": True,
        "path": ctx.runner._last_run_config,
        "exists": bool(ctx.runner._last_run_config
                       and Path(ctx.runner._last_run_config).is_file()),
    }),
    "/api/setup": lambda h, qs, ctx: json_response(h, ctx.runner.get_setup()),
    "/api/scenario/list": lambda h, qs, ctx: _scene_list(h, ctx),
    "/api/scene/list": lambda h, qs, ctx: _scene_list(h, ctx),
    "/api/actuator/step": lambda h, qs, ctx: json_response(h, {
        "plots": ctx.runner.actuator_step(),
    }),
    "/api/io/targets": lambda h, qs, ctx: json_response(h, ctx.runner.telemetry_config()),
    "/api/comms": lambda h, qs, ctx: json_response(h, ctx.runner.comms_info()),
    "/api/cosim": lambda h, qs, ctx: json_response(h, ctx.runner.cosim.status()),
    "/api/log/status": lambda h, qs, ctx: json_response(h, ctx.runner.log_status()),
    "/api/path": lambda h, qs, ctx: json_response(h, {"pts": ctx.runner.path_points()}),
    "/api/terrain": lambda h, qs, ctx: json_response(h, ctx.runner.terrain_grid()),
    "/api/scenery": lambda h, qs, ctx: json_response(h, ctx.runner.scenery_meshes()),
    "/api/debug": _get_debug,
}


def handle_get(h, route, qs, ctx):
    if route in ("/", "/app", "/app.html"):
        html = (ctx.here / "app.html").read_bytes()
        bytes_response(h, html, "text/html; charset=utf-8",
                       extra_headers={"Cache-Control": "no-store, no-cache, must-revalidate"})
        return True
    if route == "/core.js":
        data = (ctx.here / "core.js").read_bytes()
        bytes_response(h, data, "application/javascript; charset=utf-8",
                       extra_headers={"Cache-Control": "no-store, no-cache, must-revalidate"})
        return True
    if route.startswith("/vendor/"):
        rel = route.lstrip("/").split("?")[0]
        fp = (ctx.here / rel).resolve()
        if str(fp).startswith(str(ctx.here / "vendor")) and fp.is_file():
            bytes_response(h, fp.read_bytes(), "application/javascript")
        else:
            h.send_response(404)
            h.end_headers()
        return True
    fn = GET_EXACT.get(route)
    if fn is not None:
        fn(h, qs, ctx)
        return True
    if route in ("/api/state", "/api/io"):
        json_response(h, ctx.runner.snapshot())
        return True
    if route == "/api/catalog/parts":
        from runner.catalog_api import catalog_parts_list
        type_filter = (qs.get("type") or [None])[0]
        query = (qs.get("q") or [None])[0]
        tag = (qs.get("tag") or [None])[0]
        sort = (qs.get("sort") or ["label"])[0]
        json_response(h, catalog_parts_list(type_filter, query=query, tag=tag, sort=sort))
        return True
    if route == "/api/catalog/parts/types":
        from runner.catalog_api import catalog_part_types
        json_response(h, {"ok": True, **catalog_part_types()})
        return True
    if route == "/api/catalog/parts/editor":
        from runner.catalog_api import catalog_part_editor
        type_name = (qs.get("type") or ["chassis"])[0]
        part_id = (qs.get("part_id") or [None])[0]
        stem = (qs.get("stem") or [None])[0]
        label = (qs.get("label") or [None])[0]
        clone = (qs.get("clone") or ["0"])[0] in ("1", "true", "yes")
        try:
            payload = catalog_part_editor(
                type_name, part_id=part_id, stem=stem, label=label, clone=clone)
            json_response(h, {"ok": True, **payload})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if route.startswith("/api/catalog/parts/"):
        from runner.catalog_api import catalog_part_get
        pid = route.split("/api/catalog/parts/", 1)[1].strip("/")
        try:
            json_response(h, {"ok": True, "part": catalog_part_get(pid)})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if route.startswith("/api/catalog/blueprints/"):
        from runner.catalog_api import catalog_blueprint_export, catalog_blueprint_get
        rest = route.split("/api/catalog/blueprints/", 1)[1].strip("/")
        if rest.endswith("/export"):
            bid = rest[: -len("/export")].strip("/")
            try:
                json_response(h, {"ok": True, **catalog_blueprint_export(bid)})
            except Exception as e:
                json_response(h, {"ok": False, "error": str(e)}, 400)
            return True
        bid = rest
        try:
            json_response(h, {"ok": True, "blueprint": catalog_blueprint_get(bid)})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if route.startswith("/api/scene/") and route not in ("/api/scene/list",):
        name = route.split("/api/scene/", 1)[1].strip("/")
        if name:
            try:
                json_response(h, {"ok": True, "scene": ctx.runner.get_scene_doc(name)})
            except ValueError as e:
                json_response(h, {"ok": False, "error": str(e)}, 400)
            return True
    if route == "/api/suspension/list":
        from runner.catalog_api import catalog_suspension_samples
        preview = (qs.get("preview") or ["0"])[0] in ("1", "true", "yes")
        json_response(h, catalog_suspension_samples(preview_all=preview))
        return True
    if route == "/api/suspension/default":
        veh = (qs.get("vehicle") or ["sedan"])[0]
        json_response(h, ctx.suspension_default_for_vehicle(veh))
        return True
    if route == "/api/suspension/schematic":
        name = (qs.get("name") or [""])[0]
        try:
            json_response(h, ctx.suspension_schematic(name))
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)})
        return True
    if route == "/api/suspension/kc":
        name = (qs.get("name") or [""])[0]
        try:
            json_response(h, ctx.suspension_kc_plots(name))
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if route == "/api/runconfig/draft.yaml":
        import yaml
        doc = ctx.runner.export_run_config()
        data = yaml.safe_dump(doc, sort_keys=False).encode()
        bytes_response(h, data, "text/yaml; charset=utf-8")
        return True
    if route.startswith("/tex/"):
        name = os.path.basename(route[len("/tex/"):].split("?")[0])
        tdir = ctx.runner.tex_dir
        fp = os.path.join(tdir, name) if tdir else ""
        if fp and os.path.isfile(fp):
            bytes_response(h, Path(fp).read_bytes(), "image/png")
        else:
            h.send_response(404)
            h.end_headers()
        return True
    if route.startswith("/api/log/download"):
        which = "tum" if route.endswith("tum") else "csv"
        path = ctx.runner.rec_last.get(which)
        if not path or not Path(path).is_file():
            h.send_error(404)
            return True
        data = Path(path).read_bytes()
        bytes_response(h, data, "text/plain; charset=utf-8", extra_headers={
            "Content-Disposition": f'attachment; filename="{Path(path).name}"',
        })
        return True
    if route == "/api/stream":
        h.send_response(200)
        h.send_header("Content-Type", "text/event-stream")
        h.send_header("Cache-Control", "no-cache")
        h.send_header("Connection", "keep-alive")
        h.end_headers()
        try:
            while True:
                h.wfile.write(f"data: {json.dumps(ctx.runner.snapshot())}\n\n".encode())
                h.wfile.flush()
                time.sleep(1.0 / 60.0)
        except (BrokenPipeError, ConnectionResetError):
            pass
        return True
    return False


def handle_post(h, path, body, ctx):
    r = ctx.runner
    if path == "/api/config":
        if "live_vid" in body:
            with r.lock:
                vid = int(body["live_vid"])
                if vid in r.ports:
                    r.live_vid = vid
        r.reconfigure(**{k: v for k, v in body.items()
                         if k not in ("live_vid", "scenario")})
        json_response(h, {"ok": True, "config": r.config()})
        return True
    if path == "/api/fleet":
        try:
            if body.get("scenario"):
                r.load_fleet_scenario(str(body["scenario"]))
                r._rebuild_if_running()
                json_response(h, {"ok": True, "fleet": r.fleet_enriched(),
                                  "live_vid": r.live_vid,
                                  "stunt": dict(r.cfg.get("stunt") or {})})
            elif "driver" in body:
                vid = int(body.get("vehicle_id", body.get("vehicle", r.live_vid)))
                drv = r.set_fleet_driver(vid, body["driver"])
                json_response(h, {"ok": True, "vehicle_id": vid, "driver": drv})
            elif "live_vid" in body:
                with r.lock:
                    vid = int(body["live_vid"])
                    if vid in r.ports:
                        r.live_vid = vid
                json_response(h, {"ok": True, "live_vid": r.live_vid})
            else:
                json_response(h, {"ok": True, "fleet": r.fleet_enriched(),
                                  "live_vid": r.live_vid})
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/sim":
        r.set_sim(dt=body.get("dt"), time_scale=body.get("time_scale"),
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
        json_response(h, {"ok": True, "config": r.config()})
        return True
    if path == "/api/vehicle":
        vid = body.get("vehicle_id")
        payload = {k: v for k, v in body.items() if k != "vehicle_id"}
        r.set_params("vehicle", payload, vehicle_id=vid)
        json_response(h, {"ok": True})
        return True
    if path == "/api/tire":
        vid = body.get("vehicle_id")
        payload = {k: v for k, v in body.items() if k != "vehicle_id"}
        r.set_params("tire", payload, vehicle_id=vid)
        json_response(h, {"ok": True})
        return True
    if path == "/api/tire/load":
        try:
            r.load_tire(body.get("name", ""), vehicle_id=body.get("vehicle_id"))
            json_response(h, {"ok": True, "name": body.get("name")})
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/tire/import":
        try:
            info = r.import_tir(body.get("text", ""), vehicle_id=body.get("vehicle_id"))
            json_response(h, {"ok": True, **info})
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/catalog/parts/save-fields":
        try:
            out = r.save_catalog_part_fields(
                body.get("type", "chassis"),
                body.get("stem", "custom"),
                body.get("label", "Custom part"),
                body.get("fields") or {},
                base_part_id=body.get("base_part_id"),
                doc=body.get("doc"),
            )
            json_response(h, {"ok": True, **out})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/catalog/parts/import-yaml":
        try:
            out = r.import_catalog_part_yaml(body.get("yaml", ""))
            json_response(h, {"ok": True, **out})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/catalog/parts/import-tir":
        try:
            out = r.import_catalog_part_tir(
                body.get("text", ""),
                body.get("stem", "tir_import"),
                body.get("label", "TIR import"),
            )
            json_response(h, {"ok": True, **out})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/catalog/parts/import-kin":
        try:
            out = r.import_catalog_part_kin(
                body.get("yaml", ""),
                body.get("stem", "kin_import"),
                body.get("label", "Kin import"),
            )
            json_response(h, {"ok": True, **out})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/catalog/parts/delete":
        try:
            out = r.delete_catalog_part(body.get("part_id", ""))
            json_response(h, {"ok": True, **out})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/catalog/blueprint/save":
        try:
            vid = int(body.get("vehicle_id", r.live_vid))
            out = r.save_catalog_blueprint(
                vid,
                body.get("stem", "custom_build"),
                body.get("label", "Custom build"),
            )
            json_response(h, {"ok": True, **out})
        except Exception as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path in ("/api/scenario/save", "/api/scene/save"):
        try:
            json_response(h, r.save_scenario(body.get("name", ""),
                                             overwrite=bool(body.get("overwrite"))))
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/scene":
        try:
            json_response(h, {"ok": True, "config": r.import_simconfig(body)})
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/cosim/attach":
        host = str(body.get("host", "127.0.0.1"))
        port = int(body.get("cmd_port", ctx.cosim_cmd_port))
        st_port = int(body.get("state_port", ctx.cosim_state_port))
        with r.lock:
            r.cfg["cosim_attach"] = True
            r.cfg["cosim_host"] = host
            r.cfg["cosim_cmd_port"] = port
            r.cfg["cosim_state_port"] = st_port
            r.plant_error = None
        r.cosim.attach(host, port, st_port)
        t0 = time.monotonic()
        while time.monotonic() < t0 + 2.5:
            if r.cosim.last_state is not None:
                break
            time.sleep(0.05)
        with r.lock:
            ok = r.cosim.last_state is not None
            r.cfg["running"] = ok
            if not ok:
                r.plant_error = (
                    "attach failed — no STATE on :%d (uncheck Attach external?)"
                    % st_port)
                r.cosim.stop()
        json_response(h, {"ok": ok, "cosim": r.cosim.status(),
                          "error": r.plant_error})
        return True
    if path == "/api/simconfig":
        try:
            json_response(h, {"ok": True, "config": r.import_simconfig(body)})
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/runconfig":
        try:
            json_response(h, {"ok": True, "config": r.import_run_config(body)})
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, 400)
        return True
    if path == "/api/actuator":
        r.set_params("actuator", body)
        json_response(h, {"ok": True})
        return True
    if path == "/api/sensors":
        r.set_params("sensors", body)
        json_response(h, {"ok": True})
        return True
    if path == "/api/setup":
        try:
            r.apply_setup(body)
        except ValueError as e:
            json_response(h, {"ok": False, "error": str(e)}, code=400)
            return True
        json_response(h, {"ok": True,
                          "setup": r.get_setup(include_geom=False, include_scenarios=False)})
        return True
    if path == "/api/run/start":
        cfg = body.get("config")
        if cfg:
            r.import_run_config(cfg)
        out = r.control("start")
        json_response(h, {"ok": True, **out})
        return True
    if path == "/api/control":
        out = r.control(body.get("action", ""))
        json_response(h, {"ok": True, **out})
        return True
    if path == "/api/manual":
        r.set_manual(**body)
        json_response(h, {"ok": True})
        return True
    if path == "/api/io":
        json_response(h, r.io(body))
        return True
    if path == "/api/io/targets":
        r.set_telemetry(body)
        json_response(h, {"ok": True, **r.telemetry_config()})
        return True
    if path == "/api/cosim/start":
        json_response(h, r.start_cosim(body))
        return True
    if path == "/api/cosim/stop":
        json_response(h, r.stop_cosim())
        return True
    if path == "/api/map/load":
        json_response(h, r.load_map(body.get("xodr", "")))
        return True
    if path == "/api/map/rd5":
        json_response(h, r.load_rd5(body.get("rd5", ""), body.get("obj", ""),
                                    float(body.get("cell", 5.0)),
                                    body.get("buildings", "")))
        return True
    if path == "/api/terrain/load":
        json_response(h, r.load_terrain(body.get("obj", ""),
                                        float(body.get("cell", 5.0))))
        return True
    if path == "/api/terrain/clear":
        json_response(h, r.clear_terrain())
        return True
    if path == "/api/log/start":
        json_response(h, r.log_start())
        return True
    if path == "/api/log/stop":
        json_response(h, r.log_stop())
        return True
    return False


def parse_post_body(h, raw, ct, path):
    if "yaml" in ct and path in ("/api/simconfig", "/api/runconfig", "/api/scene"):
        import yaml
        return yaml.safe_load(raw) or {}
    return json.loads(raw or b"{}")
