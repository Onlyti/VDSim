from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping

from catalog import CatalogResolver, resolve_fleet_entry
from catalog.ids import (
    DEFAULT_BLUEPRINT,
    blueprint_for_vehicle,
    susp_id_from_stem,
    susp_stem_from_id,
    tire_id_from_stem,
    tire_stem_from_id,
    vehicle_stem_from_blueprint,
)
from catalog.materialize import fleet_spec_from_legacy_vehicle_row, fleet_spec_from_scene

from runner.config import REPO
from runner.suspension import (
    strip_fleet_susp_if_not_l3,
    suspension_default_for_blueprint,
    suspension_default_for_vehicle,
)


def catalog_resolver() -> CatalogResolver:
    return CatalogResolver(REPO)


def normalize_fleet_spec(spec: Dict[str, Any]) -> None:
    level = str(spec.get("level", "L2"))
    if not spec.get("blueprint"):
        veh = str(spec.get("vehicle", "sedan"))
        spec["blueprint"] = blueprint_for_vehicle(veh, level)
    parts = spec.setdefault("parts", {})
    if spec.get("tire"):
        parts.setdefault("tire", tire_id_from_stem(str(spec["tire"])))
    spec["vehicle"] = vehicle_stem_from_blueprint(str(spec["blueprint"]))
    spec["tire"] = tire_stem_from_id(parts.get("tire", "tire.default_pacejka"))
    if level in ("L3", "L4"):
        defaults = suspension_default_for_blueprint(str(spec["blueprint"]))
        if not defaults and spec.get("vehicle"):
            defaults = suspension_default_for_vehicle(spec["vehicle"])
        spec.setdefault("front_susp", defaults.get("front", "mp_front_sedan"))
        spec.setdefault("rear_susp", defaults.get("rear", "ta_rear_sedan"))
        parts.setdefault("front_susp_kin", susp_id_from_stem(str(spec["front_susp"])))
        parts.setdefault("rear_susp_kin", susp_id_from_stem(str(spec["rear_susp"])))
    else:
        spec.pop("front_susp", None)
        spec.pop("rear_susp", None)
        parts.pop("front_susp_kin", None)
        parts.pop("rear_susp_kin", None)


def apply_fleet_field_update(spec: Dict[str, Any], upd: Mapping[str, Any]) -> None:
    old_level = str(spec.get("level", "L2"))
    for k in ("x0", "y0", "z0", "yaw0", "vx0", "level", "vehicle", "tire", "front_susp", "rear_susp"):
        if k in upd:
            spec[k] = upd[k]
    if "blueprint" in upd:
        bid = str(upd["blueprint"])
        spec["blueprint"] = bid
        bp = catalog_resolver().load_blueprint(bid)
        spec["parts"] = dict(bp.get("parts") or {})
        if "level" not in upd and bp.get("level"):
            spec["level"] = str(bp["level"])
    if "parts" in upd and isinstance(upd["parts"], dict):
        spec.setdefault("parts", {}).update(upd["parts"])
    if "vehicle" in upd:
        spec["blueprint"] = blueprint_for_vehicle(
            str(upd["vehicle"]), str(spec.get("level", "L2")))
    if "tire" in upd:
        spec.setdefault("parts", {})["tire"] = tire_id_from_stem(str(upd["tire"]))
    if "level" in upd:
        new_level = str(upd["level"])
        if new_level in ("L3", "L4") and old_level not in ("L3", "L4"):
            d = suspension_default_for_vehicle(str(spec.get("vehicle", "sedan")))
            spec["front_susp"] = d["front"]
            spec["rear_susp"] = d["rear"]
    if "vehicle" in upd and "front_susp" not in upd and "rear_susp" not in upd:
        d = suspension_default_for_vehicle(str(upd["vehicle"]))
        if str(spec.get("level", "L2")) in ("L3", "L4"):
            spec["front_susp"] = d["front"]
            spec["rear_susp"] = d["rear"]
    if "front_susp" in upd:
        spec.setdefault("parts", {})["front_susp_kin"] = susp_id_from_stem(str(upd["front_susp"]))
    if "rear_susp" in upd:
        spec.setdefault("parts", {})["rear_susp_kin"] = susp_id_from_stem(str(upd["rear_susp"]))
    normalize_fleet_spec(spec)
    strip_fleet_susp_if_not_l3(spec)


def fleet_entry_for_cosim(
    resolver: CatalogResolver,
    spec: Mapping[str, Any],
    out_dir: Path,
    fleet_overrides: Mapping[int, Any],
) -> Dict[str, Any]:
    vid = int(spec["id"])
    ov = (fleet_overrides.get(vid) or {})
    vehicle_ov = {}
    if ov.get("vehicle"):
        vehicle_ov = ov["vehicle"]
    row = resolve_fleet_entry(resolver, spec, out_dir, overrides={"vehicle": vehicle_ov})
    if ov.get("tire"):
        import vdsim
        from runner.params_io import apply_fields
        from runner.params_schema import TIRE_FIELDS
        tp = vdsim.TireParams.from_yaml(row["tire"])
        apply_fields(tp, TIRE_FIELDS, ov["tire"])
        tp.to_yaml(row["tire"])
    return {
        "id": vid,
        "vehicle_yaml": row["vehicle"],
        "tire_yaml": row["tire"],
        "level": row["level"],
        "x0": row["x0"],
        "y0": row["y0"],
        "z0": float(row.get("z0", 0.0)),
        "yaw0": row["yaw0"],
        "vx0": row["vx0"],
        **({"front_susp_yaml": row["front_susp"]} if row.get("front_susp") else {}),
        **({"rear_susp_yaml": row["rear_susp"]} if row.get("rear_susp") else {}),
    }


def scene_fleet_row(entry: Mapping[str, Any]) -> Dict[str, Any]:
    rows = fleet_spec_from_scene({"fleet": [entry]})
    return rows[0]


def legacy_vehicle_row(v: Mapping[str, Any]) -> Dict[str, Any]:
    return fleet_spec_from_legacy_vehicle_row(v, REPO)
