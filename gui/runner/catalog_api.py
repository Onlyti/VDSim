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


def catalog_parts_list(
    type_filter: Optional[str] = None,
    query: Optional[str] = None,
    tag: Optional[str] = None,
    sort: str = "label",
) -> dict:
    from catalog.part_cards import part_card_stats
    r = catalog_resolver()
    parts = r.list_parts(type_filter)
    enriched = []
    for p in parts:
        row = dict(p)
        try:
            doc = r.load_part(p["id"])
            row["card"] = part_card_stats(doc)
        except Exception:
            row["card"] = {"tier": "oem", "lines": [], "blurb": ""}
        enriched.append(row)
    if query:
        q = query.lower()
        enriched = [p for p in enriched
                    if q in p["id"].lower()
                    or q in str(p.get("label", "")).lower()
                    or any(q in t.lower() for t in p.get("tags", []))
                    or any(q in ln.lower() for ln in (p.get("card") or {}).get("lines", []))]
    if tag:
        tg = tag.lower()
        enriched = [p for p in enriched
                    if tg in [t.lower() for t in p.get("tags", [])]
                    or (p.get("card") or {}).get("tier", "").lower() == tg]
    if sort == "id":
        enriched.sort(key=lambda p: p["id"])
    elif sort == "tier":
        enriched.sort(key=lambda p: (p.get("card") or {}).get("tier", ""), reverse=True)
    else:
        enriched.sort(key=lambda p: str(p.get("label", p["id"])).lower())
    return {"parts": enriched, "count": len(enriched)}


def catalog_assembly_preview(vehicle_id: int, slot: str, candidate: str, runner) -> dict:
    from catalog.assembly import assembly_preview
    from pathlib import Path
    spec = runner._spec_for_vid(int(vehicle_id))
    out = Path(runner.cosim._tmp) / f"_asm_{vehicle_id}"
    return assembly_preview(
        spec, slot, candidate,
        resolver=runner._catalog, out_dir=out,
        fleet_overrides=runner.fleet_overrides)


def catalog_blueprint_export(blueprint_id: str) -> dict:
    import yaml
    doc = catalog_blueprint_get(blueprint_id)
    return {
        "blueprint": doc,
        "yaml": yaml.safe_dump(doc, sort_keys=False),
    }


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


def catalog_part_editor(type_name: str, part_id: str | None = None,
                        stem: str | None = None, label: str | None = None,
                        clone: bool = False) -> dict:
    from catalog.part_store import part_editor_payload
    from runner.config import REPO
    return part_editor_payload(
        REPO, str(type_name), part_id=part_id, stem=stem, label=label, clone=clone)


def catalog_part_save(doc: dict) -> dict:
    from catalog.part_store import save_user_part
    from runner.config import REPO
    return save_user_part(REPO, doc)


def catalog_part_save_fields(type_name: str, stem: str, label: str,
                             fields: dict, base_part_id: str | None = None,
                             doc: dict | None = None) -> dict:
    from catalog.part_store import apply_editor_fields, new_part_doc, save_user_part
    from runner.catalog_bridge import catalog_resolver
    from runner.config import REPO
    if doc:
        base = dict(doc)
    elif base_part_id:
        try:
            base = catalog_resolver().load_part(base_part_id)
        except Exception:
            base = new_part_doc(type_name, stem, label)
    else:
        base = new_part_doc(type_name, stem, label)
    merged = apply_editor_fields(base, {**fields, "stem": stem, "label": label})
    return save_user_part(REPO, merged)


def catalog_part_import_yaml(text: str) -> dict:
    from catalog.part_store import parse_part_yaml, save_user_part
    from runner.config import REPO
    doc = parse_part_yaml(text)
    return save_user_part(REPO, doc)


def catalog_part_import_tir(text: str, stem: str, label: str) -> dict:
    from catalog.part_store import save_user_part, tir_text_to_tire_part
    from runner.config import REPO
    doc = tir_text_to_tire_part(text, stem, label)
    return save_user_part(REPO, doc)


def catalog_part_import_kin(text: str, stem: str, label: str) -> dict:
    from catalog.part_store import kin_yaml_to_susp_part, save_user_kin_part
    from runner.config import REPO
    doc, kin = kin_yaml_to_susp_part(text, stem, label)
    return save_user_kin_part(REPO, doc, kin)


def catalog_part_delete(part_id: str) -> dict:
    from catalog.part_store import delete_user_part
    from runner.config import REPO
    return delete_user_part(REPO, part_id)


def catalog_part_types() -> dict:
    from catalog.part_store import editable_part_types
    return {"types": editable_part_types()}


def catalog_blueprint_save_fleet(vehicle_id: int, stem: str, label: str, runner) -> dict:
    from catalog.part_store import blueprint_from_fleet, save_user_blueprint
    from runner.config import REPO
    spec = runner._spec_for_vid(int(vehicle_id))
    doc = blueprint_from_fleet(spec, stem, label)
    return save_user_blueprint(REPO, doc)


def catalog_suspension_samples(preview_all: bool = False) -> dict:
    r = catalog_resolver()
    l3 = [part_suffix(p["id"]) for p in r.list_parts("susp_kinematics")]
    out = {"samples": sorted(l3), "l3_count": len(l3)}
    if preview_all:
        topo = [part_suffix(p["id"]) for p in r.list_parts("susp_topology")]
        out["preview"] = sorted(set(l3) | set(topo))
    return out
