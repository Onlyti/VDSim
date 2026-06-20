from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import yaml

from .ids import (
    blueprint_for_vehicle,
    susp_id_from_stem,
    susp_stem_from_id,
    tire_id_from_stem,
    tire_stem_from_id,
    vehicle_stem_from_blueprint,
)
from .resolver import CatalogResolver


def find_repo_root(start: Path) -> Path:
    p = Path(start).resolve()
    for _ in range(12):
        if (p / "configs" / "catalog" / "manifest.yaml").is_file():
            return p
        if p.parent == p:
            break
        p = p.parent
    raise ValueError(f"catalog root not found from {start}")


def load_scene_doc(repo: Path, name: str) -> dict:
    path = Path(repo) / "configs" / "scenes" / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"unknown scene: {name}")
    return load_scene_file(path)


def load_scene_file(path: Path) -> dict:
    path = Path(path).resolve()
    if not path.is_file():
        raise ValueError(f"scene not found: {path}")
    doc = yaml.safe_load(path.read_text()) or {}
    if not doc.get("fleet"):
        raise ValueError(f"scene '{path}' has no fleet[]")
    return doc


def is_catalog_scene_file(path: Path) -> bool:
    path = Path(path)
    if not path.is_file():
        return False
    doc = yaml.safe_load(path.read_text()) or {}
    return bool(doc.get("fleet"))


def _scene_vehicle_stem(doc: Mapping[str, Any]) -> str:
    ref = doc.get("vehicle")
    if not ref:
        return "sedan"
    p = Path(str(ref))
    return p.stem or "sedan"


def fleet_spec_from_scene(doc: Mapping[str, Any]) -> List[dict]:
    out = []
    doc_level = str(doc.get("level", "L2"))
    doc_tire = doc.get("tire")
    doc_vehicle = _scene_vehicle_stem(doc)
    for entry in doc["fleet"]:
        level = str(entry.get("level", doc_level))
        bid = str(entry.get("blueprint") or blueprint_for_vehicle(doc_vehicle, level))
        parts = dict(entry.get("parts") or {})
        if doc_tire and "tire" not in parts:
            parts["tire"] = tire_id_from_stem(Path(str(doc_tire)).stem)
        row = {
            "id": int(entry["id"]),
            "blueprint": bid,
            "parts": parts,
            "level": level,
            "x0": float(entry.get("x0", 0.0)),
            "y0": float(entry.get("y0", 0.0)),
            "z0": float(entry.get("z0", 0.0)),
            "yaw0": float(entry.get("yaw0", 0.0)),
            "vx0": float(entry.get("vx0", 0.0)),
            "vehicle": vehicle_stem_from_blueprint(bid),
            "tire": tire_stem_from_id(parts.get("tire", "tire.default_pacejka")),
        }
        if level in ("L3", "L4"):
            bp_path = Path(__file__).resolve().parents[2] / "configs" / "blueprints"
            bp_name = bid.replace("vehicle.", "") + ".yaml"
            bp = yaml.safe_load((bp_path / bp_name).read_text()) or {}
            bp_parts = bp.get("parts") or {}
            f_id = parts.get("front_chassis", bp_parts.get("front_chassis", ""))
            r_id = parts.get("rear_chassis", bp_parts.get("rear_chassis", ""))
            if f_id:
                row["front_susp"] = susp_stem_from_id(str(f_id))
            if r_id:
                row["rear_susp"] = susp_stem_from_id(str(r_id))
        out.append(row)
    return out


def resolve_fleet_entry(
    resolver: CatalogResolver,
    spec: Mapping[str, Any],
    out_dir: Path,
    *,
    overrides: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    vid = int(spec["id"])
    bid = str(spec.get("blueprint") or blueprint_for_vehicle(
        str(spec.get("vehicle", "sedan")), str(spec.get("level", "L2"))))
    instance_parts = dict(spec.get("parts") or {})
    if spec.get("tire") and "tire" not in instance_parts:
        instance_parts["tire"] = tire_id_from_stem(str(spec["tire"]))
    level = str(spec.get("level", "L2"))
    if level in ("L3", "L4"):
        if spec.get("front_susp"):
            instance_parts.setdefault(
                "front_chassis", susp_id_from_stem(str(spec["front_susp"])))
        if spec.get("rear_susp"):
            instance_parts.setdefault(
                "rear_chassis", susp_id_from_stem(str(spec["rear_susp"])))
    sub = out_dir / f"veh_{vid}"
    sub.mkdir(parents=True, exist_ok=True)
    ov = dict(overrides or {})
    resolved = resolver.resolve_blueprint(
        bid, instance_parts=instance_parts, overrides=ov.get("vehicle"), out_dir=sub)
    row = {
        "id": vid,
        "vehicle": str(resolved.vehicle_yaml),
        "tire": str(resolved.tire_yaml),
        "level": level,
        "x0": float(spec.get("x0", 0.0)),
        "y0": float(spec.get("y0", 0.0)),
        "z0": float(spec.get("z0", 0.0)),
        "yaw0": float(spec.get("yaw0", 0.0)),
        "vx0": float(spec.get("vx0", 0.0)),
        "control": str(spec.get("control", "external")),
    }
    if level in ("L3", "L4"):
        if resolved.susp_front:
            row["front_susp"] = str(resolved.susp_front)
        if resolved.susp_rear:
            row["rear_susp"] = str(resolved.susp_rear)
    if resolved.tire_rear_yaml:
        row["tire_rear"] = str(resolved.tire_rear_yaml)
    if resolved.tire_fr_yaml:
        row["tire_fr"] = str(resolved.tire_fr_yaml)
    if resolved.tire_rl_yaml:
        row["tire_rl"] = str(resolved.tire_rl_yaml)
    if resolved.tire_rr_yaml:
        row["tire_rr"] = str(resolved.tire_rr_yaml)
    # per-vehicle sensors: propagate inline list or yaml-path string as-is
    sensors = spec.get("sensors")
    if sensors is not None:
        row["sensors"] = sensors
    return row


def materialize_scene_file(
    scene_path: Path,
    out_path: Path,
    *,
    repo: Optional[Path] = None,
    resolver: Optional[CatalogResolver] = None,
) -> Path:
    scene_path = Path(scene_path).resolve()
    doc = load_scene_file(scene_path)
    root = Path(repo) if repo else find_repo_root(scene_path)
    r = resolver or CatalogResolver(root)
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    world = {
        "name": doc.get("id") or scene_path.stem,
        "rate": float(doc.get("rate", 200.0)),
        "cmd_timeout": float(doc.get("cmd_timeout", 0.1)),
        "mu": float(doc.get("mu", 1.0)),
        "vehicles": [],
    }
    for key in ("grade", "bank", "mu_right", "mu_boundary", "time_scale"):
        if key in doc:
            world[key] = doc[key]
    if doc.get("stunt"):
        world["stunt"] = dict(doc["stunt"])
    for key in ("rough_amp", "rough_wl", "iso_class"):
        if key in doc:
            world[key] = doc[key]
    if doc.get("terrain"):
        tpath = Path(str(doc["terrain"]))
        if not tpath.is_absolute():
            tpath = root / tpath
        world["terrain"] = str(tpath.resolve())
    env = doc.get("environment")
    if isinstance(env, dict):
        for key in ("mu", "grade", "bank"):
            if key in env:
                world[key] = env[key]
    # comms is scenario-level: inline the referenced routing spec into the world
    # so the C++ realtime server sees concrete channels (no repo-root lookup).
    comms = doc.get("comms")
    if isinstance(comms, str):
        cpath = root / "configs" / "comms" / f"{comms}.yaml"
        if not cpath.is_file():
            alt = Path(comms)
            cpath = alt if alt.is_file() else (root / comms)
        if cpath.is_file():
            world["comms"] = yaml.safe_load(cpath.read_text()) or {}
        else:
            raise ValueError(f"unknown comms spec: {comms}")
    elif isinstance(comms, dict):
        world["comms"] = dict(comms)
    tmp = out_path.parent / "_resolve"
    tmp.mkdir(parents=True, exist_ok=True)
    for entry in fleet_spec_from_scene(doc):
        world["vehicles"].append(resolve_fleet_entry(r, entry, tmp))
    out_path.write_text(yaml.safe_dump(world, sort_keys=False))
    return out_path


def materialize_scene_world(
    repo: Path,
    scene_name: str,
    out_path: Path,
    *,
    resolver: Optional[CatalogResolver] = None,
) -> Path:
    repo = Path(repo)
    scene_path = repo / "configs" / "scenes" / f"{scene_name}.yaml"
    return materialize_scene_file(scene_path, out_path, repo=repo, resolver=resolver)
