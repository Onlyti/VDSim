from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from catalog.ids import part_suffix
from catalog.materialize import resolve_fleet_entry
from catalog.part_cards import level_rank, part_card_stats, part_compat
from catalog.resolver import CatalogResolver
from catalog.slots import slots_for_level

_RECOMMENDED_BLUEPRINTS = (
    "vehicle.sedan_comfort",
    "vehicle.sports_aggressive",
    "vehicle.race_gt",
    "vehicle.fsk_formula",
)

_CATEGORIES: List[Dict[str, Any]] = [
    {"id": "body", "label": "Body & chassis",
     "slots": ["body", "aero", "ride", "front_chassis", "rear_chassis"]},
    {"id": "power", "label": "Powertrain", "slots": ["powertrain", "drivetrain"]},
    {"id": "grip", "label": "Grip", "slots": ["tire"]},
    {"id": "control", "label": "Control", "slots": ["brake", "steering"]},
]

_DELTA_KEYS = (
    "mass_kg", "wheelbase_m", "cg_height_m", "track_front_m", "track_rear_m",
    "wheel_radius_m", "tire_mu",
)


def _part_label(resolver: CatalogResolver, part_id: str) -> str:
    try:
        doc = resolver.load_part(part_id)
        return str(doc.get("label") or part_id)
    except Exception:
        return part_id


def _resolve_summary(
    resolver: CatalogResolver,
    spec: Mapping[str, Any],
    parts: Mapping[str, str],
    out_dir: Path,
    fleet_overrides: Optional[Mapping[int, Any]] = None,
) -> Dict[str, Any]:
    try:
        vid = int(spec["id"])
        ov = (fleet_overrides or {}).get(vid) or {}
        row_spec = dict(spec)
        row_spec["parts"] = dict(parts)
        row = resolve_fleet_entry(resolver, row_spec, out_dir, overrides=ov)
        import vdsim
        vp = vdsim.VehicleParams.from_yaml(row["vehicle"])
        tp = vdsim.TireParams.from_yaml(row["tire"])
        return {
            "mass_kg": float(vp.mass),
            "wheelbase_m": float(vp.cg_to_front + vp.cg_to_rear),
            "cg_height_m": float(vp.cg_height),
            "track_front_m": float(vp.track_front),
            "track_rear_m": float(vp.track_rear),
            "wheel_radius_m": float(vp.wheel_radius_nominal),
            "tire_mu": float(tp.mu_nominal),
            "drive_type": str(vp.drive_type),
        }
    except Exception as exc:
        return {"error": str(exc)}


def _summary_delta(
    base: Mapping[str, Any],
    trial: Mapping[str, Any],
) -> Dict[str, float]:
    if base.get("error") or trial.get("error"):
        return {}
    delta: Dict[str, float] = {}
    for key in _DELTA_KEYS:
        if key in base and key in trial:
            try:
                delta[key] = float(trial[key]) - float(base[key])
            except (TypeError, ValueError):
                pass
    return delta


def _enrich_candidate(
    resolver: CatalogResolver,
    part_id: str,
    slot: str,
    level: str,
) -> Dict[str, Any]:
    try:
        doc = resolver.load_part(part_id)
    except Exception as exc:
        return {
            "id": part_id,
            "label": part_id,
            "tags": [],
            "card": {"tier": "oem", "lines": [], "blurb": ""},
            "compat": [{"level": "error", "msg": str(exc)}],
        }
    compat = part_compat(slot, doc, level)
    return {
        "id": part_id,
        "label": str(doc.get("label") or part_id),
        "tags": list(doc.get("tags") or []),
        "card": part_card_stats(doc),
        "compat": compat,
        "ok": not any(c.get("level") == "error" for c in compat),
    }


def assembly_preview(
    spec: Mapping[str, Any],
    slot: str,
    candidate_id: str,
    *,
    resolver: CatalogResolver,
    out_dir: Path,
    fleet_overrides: Optional[Mapping[int, Any]] = None,
) -> Dict[str, Any]:
    level = str(spec.get("level", "L2"))
    blueprint_id = str(spec.get("blueprint") or f"vehicle.{spec.get('vehicle', 'sedan')}")
    bp_doc = resolver.load_blueprint(blueprint_id)
    parts = dict(bp_doc.get("parts") or {})
    parts.update(dict(spec.get("parts") or {}))
    base = _resolve_summary(resolver, spec, parts, out_dir, fleet_overrides)
    trial_parts = dict(parts)
    trial_parts[str(slot)] = str(candidate_id)
    trial = _resolve_summary(resolver, spec, trial_parts, out_dir, fleet_overrides)
    return {
        "slot": slot,
        "candidate": candidate_id,
        "summary": trial,
        "delta": _summary_delta(base, trial),
        "candidate_info": _enrich_candidate(resolver, candidate_id, slot, level),
    }


def fleet_assembly_view(
    spec: Mapping[str, Any],
    *,
    resolver: Optional[CatalogResolver] = None,
    out_dir: Path,
    fleet_overrides: Optional[Mapping[int, Any]] = None,
    preview_slot: Optional[str] = None,
    preview_candidate: Optional[str] = None,
) -> Dict[str, Any]:
    if resolver is None:
        root = Path(__file__).resolve().parents[2]
        r = CatalogResolver(root)
    else:
        r = resolver
    level = str(spec.get("level", "L2"))
    blueprint_id = str(
        spec.get("blueprint")
        or f"vehicle.{spec.get('vehicle', 'sedan')}")
    bp_doc = r.load_blueprint(blueprint_id)
    parts = dict(bp_doc.get("parts") or {})
    parts.update(dict(spec.get("parts") or {}))

    blueprints = r.list_blueprints()
    bp_ids = {b["id"] for b in blueprints}
    bp_meta = next((b for b in blueprints if b["id"] == blueprint_id), None)
    candidates_by_type: Dict[str, list] = {}
    slots_out = []
    active_slots = {s[0] for s in slots_for_level(level)}
    for slot, label, ptype in slots_for_level(level):
        cand = candidates_by_type.get(ptype)
        if cand is None:
            cand = r.list_parts(ptype)
            candidates_by_type[ptype] = cand
        part_id = str(parts.get(slot, ""))
        slots_out.append({
            "slot": slot,
            "label": label,
            "type": ptype,
            "part_id": part_id,
            "part_label": _part_label(r, part_id) if part_id else "—",
            "part_stem": part_suffix(part_id) if part_id else "",
            "candidates": [
                _enrich_candidate(r, p["id"], slot, level) for p in cand
            ],
        })

    summary = _resolve_summary(r, spec, parts, out_dir, fleet_overrides)
    categories = []
    for cat in _CATEGORIES:
        cat_slots = [s for s in cat["slots"] if s in active_slots]
        if cat_slots:
            categories.append({**cat, "slots": cat_slots})

    recommended = [
        {"id": bid, "label": next((b["label"] for b in blueprints if b["id"] == bid), bid)}
        for bid in _RECOMMENDED_BLUEPRINTS
        if bid in bp_ids
    ]

    out: Dict[str, Any] = {
        "blueprint": {
            "id": blueprint_id,
            "label": (bp_meta or {}).get("label", blueprint_id),
            "default_level": str((bp_meta or {}).get("level", level)),
        },
        "blueprints": [
            {"id": b["id"], "label": b.get("label", b["id"]), "level": b.get("level", "L2")}
            for b in blueprints
        ],
        "recommended": recommended,
        "categories": categories,
        "level": level,
        "parts": parts,
        "slots": slots_out,
        "summary": summary,
        "build_complete": not summary.get("error") and all(
            bool(s["part_id"]) for s in slots_out),
    }

    if preview_slot and preview_candidate:
        out["preview"] = assembly_preview(
            spec, preview_slot, preview_candidate,
            resolver=r, out_dir=out_dir, fleet_overrides=fleet_overrides)

    return out
