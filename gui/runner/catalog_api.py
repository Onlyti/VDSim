from __future__ import annotations

from typing import Optional

from runner.catalog_bridge import catalog_resolver
from runner.suspension import list_l3_kinematics_configs, suspension_default_for_vehicle

from catalog.ids import part_suffix, tire_stem_from_id, vehicle_stem_from_blueprint


def catalog_index(type_filter: Optional[str] = None, query: Optional[str] = None) -> dict:
    r = catalog_resolver()
    m = r.load_manifest()
    parts = r.list_parts(type_filter)
    if query:
        q = query.lower()
        parts = [p for p in parts
                 if q in p["id"].lower()
                 or q in str(p.get("label", "")).lower()
                 or any(q in t.lower() for t in p.get("tags", []))]
    blueprints = r.list_blueprints()
    if query:
        q = query.lower()
        blueprints = [b for b in blueprints
                      if q in b["id"].lower() or q in str(b.get("label", "")).lower()]
    return {
        "catalog_id": m.get("catalog_id"),
        "catalog_version": m.get("catalog_version"),
        "parts": parts,
        "blueprints": blueprints,
    }


def catalog_part_get(part_id: str) -> dict:
    return catalog_resolver().load_part(part_id)


def catalog_blueprint_get(blueprint_id: str) -> dict:
    return catalog_resolver().load_blueprint(blueprint_id)


def catalog_parts_list(type_filter: Optional[str] = None) -> dict:
    return {"parts": catalog_resolver().list_parts(type_filter)}


def catalog_legacy_registry() -> dict:
    r = catalog_resolver()
    tires = r.list_parts("tire")
    chassis = r.list_parts("chassis")
    blueprints = r.list_blueprints()
    return {
        "catalog_id": r.load_manifest().get("catalog_id"),
        "blueprints": [b["id"] for b in blueprints],
        "vehicles": [vehicle_stem_from_blueprint(b["id"]) for b in blueprints],
        "tires": [tire_stem_from_id(p["id"]) for p in tires],
        "chassis": [part_suffix(p["id"]) for p in chassis],
        "suspensions": list_l3_kinematics_configs(),
        "l3_kinematics": list_l3_kinematics_configs(),
        "vehicle_suspension_defaults": {
            vehicle_stem_from_blueprint(b["id"]): suspension_default_for_vehicle(
                vehicle_stem_from_blueprint(b["id"]))
            for b in blueprints
        },
    }


def catalog_suspension_samples(preview_all: bool = False) -> dict:
    r = catalog_resolver()
    l3 = [part_suffix(p["id"]) for p in r.list_parts("susp_kinematics")]
    out = {"samples": sorted(l3), "l3_count": len(l3)}
    if preview_all:
        topo = [part_suffix(p["id"]) for p in r.list_parts("susp_topology")]
        out["preview"] = sorted(set(l3) | set(topo))
    return out
