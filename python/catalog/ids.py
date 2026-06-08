from __future__ import annotations

DEFAULT_BLUEPRINT = "vehicle.sedan_comfort"

STEM_TO_BLUEPRINT = {
    "sedan": "vehicle.sedan_comfort",
    "sports": "vehicle.sports_aggressive",
    "fsk_formula": "vehicle.fsk_formula",
    "race_car": "vehicle.race_gt",
}

BLUEPRINT_TO_STEM = {v: k for k, v in STEM_TO_BLUEPRINT.items()}

TIRE_STEM_TO_ID = {
    "default_pacejka": "tire.default_pacejka",
    "sport_grip": "tire.sport_grip",
    "low_mu": "tire.low_mu",
    "lugre_on": "tire.lugre_on",
    "kinematic_fallback": "tire.kinematic_fallback",
}

TIRE_ID_TO_STEM = {v: k for k, v in TIRE_STEM_TO_ID.items()}

SUSP_STEM_TO_ID = {
    "mp_front_sedan": "susp.mp_front_sedan",
    "ta_rear_sedan": "susp.ta_rear_sedan",
    "dw_front_sports": "susp.dw_front_sports",
    "5link_rear_sports": "susp.5link_rear_sports",
}

SUSP_ID_TO_STEM = {v: k for k, v in SUSP_STEM_TO_ID.items()}

BLUEPRINTS = sorted({
    DEFAULT_BLUEPRINT,
    "vehicle.sedan_l3",
    *STEM_TO_BLUEPRINT.values(),
})


def blueprint_for_vehicle(vehicle_stem: str, level: str = "L2") -> str:
    if vehicle_stem == "sedan" and str(level) in ("L3", "L4"):
        return "vehicle.sedan_l3"
    return STEM_TO_BLUEPRINT.get(vehicle_stem, DEFAULT_BLUEPRINT)


def vehicle_stem_from_blueprint(blueprint_id: str) -> str:
    if blueprint_id == "vehicle.sedan_l3":
        return "sedan"
    return BLUEPRINT_TO_STEM.get(blueprint_id, "sedan")


def tire_id_from_stem(stem: str) -> str:
    s = str(stem)
    if s.startswith("tire."):
        return s
    return TIRE_STEM_TO_ID.get(s, f"tire.{s}")


def tire_stem_from_id(part_id: str) -> str:
    return TIRE_ID_TO_STEM.get(part_id, part_id.rsplit(".", 1)[-1])


def susp_id_from_stem(stem: str) -> str:
    s = str(stem)
    if s.startswith("susp."):
        return s
    return SUSP_STEM_TO_ID.get(s, f"susp.{s}")


def susp_stem_from_id(part_id: str) -> str:
    return SUSP_ID_TO_STEM.get(part_id, part_id.rsplit(".", 1)[-1])


def part_suffix(part_id: str) -> str:
    return part_id.rsplit(".", 1)[-1]
