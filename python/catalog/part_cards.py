from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional

_LEVEL_RANK = {"K": 0, "Ld0": 0, "L0": 0, "Ld1": 1, "L1": 1, "Ld2": 2, "L2": 2,
               "Ld3": 3, "L3": 3, "Ld4": 4, "L4": 4, "Ld5": 5, "L5": 5}


def level_rank(level: str) -> int:
    return _LEVEL_RANK.get(str(level), 2)


def tier_from_tags(tags: List[str]) -> str:
    low = {t.lower() for t in tags}
    for name in ("race", "sport", "stunt", "experimental", "oem", "comfort"):
        if name in low:
            return name
    return "oem"


def part_card_stats(doc: Mapping[str, Any]) -> Dict[str, Any]:
    t = str(doc.get("type", ""))
    body = doc.get("body") or {}
    lines: List[str] = []
    if t == "body":
        if "mass" in body:
            lines.append(f"{float(body['mass']):.0f} kg")
        if "cg_height" in body:
            lines.append(f"CG {float(body['cg_height']):.2f} m")
        if "wheelbase" in body:
            lines.append(f"WB {float(body['wheelbase']):.2f} m")
        elif "cg_to_front" in body and "cg_to_rear" in body:
            wb = float(body["cg_to_front"]) + float(body["cg_to_rear"])
            lines.append(f"WB {wb:.2f} m")
    elif t == "chassis":
        path = str(body.get("path", ""))
        stem = path.rsplit("/", 1)[-1].replace(".yaml", "") if path else "—"
        lines.append(stem)
        lines.append(str(doc.get("schema", "")).replace("_v1", ""))
    elif t == "tire":
        if "mu_nominal" in body:
            lines.append(f"μ {float(body['mu_nominal']):.2f}")
        lug = body.get("lugre") or {}
        if isinstance(lug, dict) and lug.get("enabled"):
            lines.append("LuGre on")
    elif t == "brake":
        if "max_brake_torque" in body:
            lines.append(f"T_max {float(body['max_brake_torque']):.0f} N·m")
        if "brake_bias_front" in body:
            lines.append(f"bias {float(body['brake_bias_front']):.2f}")
    elif t == "steering":
        if "steering_ratio" in body:
            lines.append(f"ratio {float(body['steering_ratio']):.1f}")
        if "max_steer_angle_wheel" in body:
            lines.append(f"δ_max {float(body['max_steer_angle_wheel']):.2f} rad")
    elif t == "drivetrain":
        if body.get("drive_type"):
            lines.append(str(body["drive_type"]))
        if "max_motor_torque" in body:
            lines.append(f"T {float(body['max_motor_torque']):.0f} N·m")
        if "final_drive_ratio" in body:
            lines.append(f"i_f {float(body['final_drive_ratio']):.1f}")
    elif t == "susp_kinematics":
        path = str(body.get("path", ""))
        stem = path.rsplit("/", 1)[-1].replace(".yaml", "") if path else "—"
        lines.append(stem)
        lines.append(str(doc.get("schema", "")).replace("_v1", ""))
    ui = doc.get("ui") or {}
    blurb = str(ui.get("blurb") or "")
    tier = str(ui.get("tier") or "") or tier_from_tags(list(doc.get("tags") or []))
    return {
        "tier": tier,
        "lines": lines[:3],
        "blurb": blurb,
    }


def part_compat(
    slot: str,
    doc: Mapping[str, Any],
    level: str,
) -> List[Dict[str, str]]:
    issues: List[Dict[str, str]] = []
    schema = str(doc.get("schema", ""))
    kin_slots = ("front_susp_kin", "rear_susp_kin", "front_chassis", "rear_chassis")
    if slot in kin_slots:
        if level not in ("L3", "L4"):
            issues.append({"level": "error", "msg": "Requires L3 or L4"})
        if schema == "topology_preview_v1":
            issues.append({"level": "error", "msg": "Preview-only topology"})
        elif schema != "kinematics_l3_native_v1":
            issues.append({"level": "error", "msg": f"Bad schema: {schema}"})
        pid = str(doc.get("id", "")).lower()
        if slot == "front_chassis" and "rear" in pid and "front" not in pid:
            issues.append({"level": "warn", "msg": "Rear axle part on front slot"})
        if slot == "rear_chassis" and "front" in pid and "rear" not in pid:
            issues.append({"level": "warn", "msg": "Front axle part on rear slot"})
    ui = doc.get("ui") or {}
    compat = ui.get("compat") or {}
    min_level = compat.get("min_level")
    if min_level and level_rank(level) < level_rank(str(min_level)):
        issues.append({"level": "warn", "msg": f"Best for {min_level}+"})
    return issues
