#!/usr/bin/env python3
"""One-shot v0.2 → v0.3 catalog migration (PARTS_CATALOG §9). Run from repo root."""
from __future__ import annotations

import shutil
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
CFG = REPO / "configs"

CHASSIS_KEYS = {
    "mass", "mass_sprung", "inertia_diag", "wheelbase", "cg_to_front", "cg_to_rear",
    "track_front", "track_rear", "cg_height", "wheel_radius_nominal",
    "spring_stiffness", "damper_coefficient", "unsprung_mass", "wheel_inertia",
    "arb_stiffness_front", "arb_stiffness_rear", "roll_center_height_front",
    "roll_center_height_rear", "anti_dive_front", "anti_squat_rear", "camber_per_roll",
    "pitch_center_height", "aero_drag_coeff", "frontal_area", "aero_lift_front", "aero_lift_rear",
}
DRIVETRAIN_KEYS = {
    "drive_type", "differential", "lsd_preload", "lsd_ramp",
    "max_motor_torque", "final_drive_ratio", "drive_deadtime_s",
}
BRAKE_KEYS = {"max_brake_torque", "brake_bias_front", "brake_ebd_enabled", "brake_deadtime_s"}
STEERING_KEYS = {
    "steering_ratio", "max_steer_angle_wheel", "ackerman_percent", "steer_deadtime_s",
}

VEHICLE_BLUEPRINTS = {
    "sedan": ("vehicle.sedan_comfort", "L2"),
    "sports": ("vehicle.sports_aggressive", "L2"),
    "fsk_formula": ("vehicle.fsk_formula", "L3"),
    "race_car": ("vehicle.race_gt", "L3"),
}

TIRE_IDS = {
    "default_pacejka": "tire.default_pacejka",
    "sport_grip": "tire.sport_grip",
    "low_mu": "tire.low_mu",
}

L3_SUSP = {
    "sedan": ("susp.mp_front_sedan", "susp.ta_rear_sedan"),
    "sports": ("susp.dw_front_sports", "susp.5link_rear_sports"),
    "fsk_formula": ("susp.dw_front_sports", "susp.5link_rear_sports"),
    "race_car": ("susp.dw_front_sports", "susp.5link_rear_sports"),
}


def _split_vehicle(stem: str, doc: dict) -> None:
    chassis = {k: doc[k] for k in CHASSIS_KEYS if k in doc}
    drivetrain = {k: doc[k] for k in DRIVETRAIN_KEYS if k in doc}
    brake = {k: doc[k] for k in BRAKE_KEYS if k in doc}
    steering = {k: doc[k] for k in STEERING_KEYS if k in doc}
    _write_part(f"chassis/{stem}.yaml", f"chassis.{stem}", "chassis", "vehicle_params_v1",
                f"{stem.replace('_', ' ').title()} chassis", chassis)
    dt_label = f"{stem} {drivetrain.get('drive_type', 'RWD')} {drivetrain.get('differential', 'Open')}"
    _write_part(f"drivetrain/{stem}.yaml", f"drivetrain.{stem}", "drivetrain", "drivetrain_v1",
                dt_label, drivetrain)
    _write_part(f"brake/{stem}.yaml", f"brake.{stem}", "brake", "brake_subsystem_v1",
                f"{stem} brakes", brake)
    _write_part(f"steering/{stem}.yaml", f"steering.{stem}", "steering", "steering_subsystem_v1",
                f"{stem} steering", steering)


def _write_part(rel: str, pid: str, ptype: str, schema: str, label: str, body: dict) -> None:
    path = CFG / "parts" / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {
        "id": pid, "type": ptype, "version": 1, "schema": schema,
        "label": label, "tags": [ptype], "body": body,
    }
    path.write_text(yaml.safe_dump(doc, sort_keys=False))


def _migrate_tires() -> None:
    for src in (CFG / "tires").glob("*.yaml"):
        body = yaml.safe_load(src.read_text()) or {}
        pid = TIRE_IDS.get(src.stem, f"tire.{src.stem}")
        _write_part(f"tire/{src.stem}.yaml", pid, "tire", "pacejka_mf96_v1",
                    src.stem.replace("_", " "), body)


def _migrate_suspensions() -> None:
    kin_dir = CFG / "parts" / "susp_kinematics" / "kin"
    topo_dir = CFG / "parts" / "susp_topology"
    kin_dir.mkdir(parents=True, exist_ok=True)
    topo_dir.mkdir(parents=True, exist_ok=True)
    for src in (CFG / "suspensions").glob("*.yaml"):
        raw = yaml.safe_load(src.read_text()) or {}
        if raw.get("type"):
            kin_path = kin_dir / f"{src.stem}.yaml"
            kin_path.write_text(yaml.safe_dump(raw, sort_keys=False))
            _write_part(
                f"susp_kinematics/{src.stem}.yaml",
                f"susp.{src.stem}",
                "susp_kinematics",
                "kinematics_l3_native_v1",
                f"{src.stem} kinematics",
                {"path": f"parts/susp_kinematics/kin/{src.stem}.yaml"},
            )
        else:
            topo_path = topo_dir / f"{src.stem}.yaml"
            topo_path.write_text(yaml.safe_dump(raw, sort_keys=False))
            _write_part(
                f"susp_topology/{src.stem}.yaml",
                f"susp_topo.{src.stem}",
                "susp_topology",
                "topology_preview_v1",
                f"{src.stem} topology preview",
                {"path": f"parts/susp_topology/{src.stem}.yaml"},
            )


def _write_blueprints() -> None:
    bp_dir = CFG / "blueprints"
    bp_dir.mkdir(parents=True, exist_ok=True)
    for stem, (bid, level) in VEHICLE_BLUEPRINTS.items():
        parts = {
            "chassis": f"chassis.{stem}",
            "tire": "tire.default_pacejka",
            "brake": f"brake.{stem}",
            "steering": f"steering.{stem}",
            "drivetrain": f"drivetrain.{stem}",
        }
        if stem == "sports":
            parts["tire"] = "tire.sport_grip"
        if level == "L3":
            f, r = L3_SUSP[stem]
            parts["front_susp_kin"] = f
            parts["rear_susp_kin"] = r
        doc = {
            "id": bid,
            "version": 1,
            "label": bid.replace("vehicle.", "").replace("_", " ").title(),
            "level": level,
            "parts": parts,
            "overrides": {},
        }
        (bp_dir / f"{bid.replace('vehicle.', '')}.yaml").write_text(
            yaml.safe_dump(doc, sort_keys=False))

    sedan_l3 = {
        "id": "vehicle.sedan_l3",
        "version": 1,
        "label": "Sedan L3 kinematics",
        "level": "L3",
        "parts": {
            "chassis": "chassis.sedan",
            "tire": "tire.default_pacejka",
            "brake": "brake.sedan",
            "steering": "steering.sedan",
            "drivetrain": "drivetrain.sedan",
            "front_susp_kin": "susp.mp_front_sedan",
            "rear_susp_kin": "susp.ta_rear_sedan",
        },
        "overrides": {},
    }
    (bp_dir / "sedan_l3.yaml").write_text(yaml.safe_dump(sedan_l3, sort_keys=False))


def _write_manifest() -> None:
    parts = []
    for p in sorted((CFG / "parts").rglob("*.yaml")):
        if "kin" in p.parts and p.parent.name == "kin":
            continue
        if p.parent.name == "susp_topology" and p.parent.parent.name == "parts":
            rel = p.relative_to(CFG)
            doc = yaml.safe_load(p.read_text()) or {}
            parts.append({"id": doc["id"], "path": str(rel).replace("\\", "/")})
            continue
        if p.parent.name in ("chassis", "tire", "drivetrain", "brake", "steering", "susp_kinematics"):
            rel = p.relative_to(CFG)
            doc = yaml.safe_load(p.read_text()) or {}
            parts.append({"id": doc["id"], "path": str(rel).replace("\\", "/")})
    blueprints = []
    for p in sorted((CFG / "blueprints").glob("*.yaml")):
        doc = yaml.safe_load(p.read_text()) or {}
        blueprints.append({"id": doc["id"], "path": f"blueprints/{p.name}"})
    manifest = {
        "catalog_id": "vdsim.builtin",
        "catalog_version": 1,
        "parts": parts,
        "blueprints": blueprints,
        "packages": [],
    }
    (CFG / "catalog" / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))


def _migrate_scenes() -> None:
    scenes = CFG / "scenes"
    maneuvers = CFG / "maneuvers"
    scenes.mkdir(parents=True, exist_ok=True)
    maneuvers.mkdir(parents=True, exist_ok=True)

    two = yaml.safe_load((CFG / "scenarios" / "two_vehicle_race.yaml").read_text())
    scene = {
        "id": "scene.two_vehicle_race",
        "version": 1,
        "label": "Two-car straight launch",
        "rate": two.get("rate", 200),
        "cmd_timeout": two.get("cmd_timeout", 0.1),
        "mu": two.get("mu", 1.0),
        "fleet": [],
    }
    for v in two["vehicles"]:
        veh_stem = Path(v["vehicle"]).stem
        tire_stem = Path(v["tire"]).stem
        bid, _ = VEHICLE_BLUEPRINTS[veh_stem]
        entry = {
            "id": int(v["id"]),
            "blueprint": bid,
            "level": v.get("level", "L2"),
            "x0": v.get("x0", 0.0),
            "y0": v.get("y0", 0.0),
            "yaw0": v.get("yaw0", 0.0),
            "vx0": v.get("vx0", 0.0),
        }
        tire_id = TIRE_IDS[tire_stem]
        if tire_id != "tire.default_pacejka" or veh_stem == "sports":
            entry["parts"] = {"tire": tire_id}
        scene["fleet"].append(entry)
    (scenes / "two_vehicle_race.yaml").write_text(yaml.safe_dump(scene, sort_keys=False))

    l3 = yaml.safe_load((CFG / "scenarios" / "l3_sedan_kinematics.yaml").read_text())
    scene_l3 = {
        "id": "scene.l3_sedan_kinematics",
        "version": 1,
        "label": "L3 sedan kinematics demo",
        "rate": l3.get("rate", 200),
        "cmd_timeout": l3.get("cmd_timeout", 0.1),
        "mu": l3.get("mu", 1.0),
        "fleet": [{
            "id": 0,
            "blueprint": "vehicle.sedan_l3",
            "level": "L3",
            "x0": 0.0, "y0": 0.0, "yaw0": 0.0, "vx0": 0.0,
        }],
    }
    (scenes / "l3_sedan_kinematics.yaml").write_text(yaml.safe_dump(scene_l3, sort_keys=False))

    for src in (CFG / "scenarios").glob("*.yaml"):
        if src.stem in ("two_vehicle_race", "l3_sedan_kinematics"):
            continue
        shutil.copy2(src, maneuvers / src.name)


def main():
    for stem in ("sedan", "sports", "fsk_formula", "race_car"):
        doc = yaml.safe_load((CFG / "vehicles" / f"{stem}.yaml").read_text()) or {}
        _split_vehicle(stem, doc)
    _migrate_tires()
    _migrate_suspensions()
    _write_blueprints()
    _migrate_scenes()
    _write_manifest()
    print("migrate_v02_configs: ok")


if __name__ == "__main__":
    main()
