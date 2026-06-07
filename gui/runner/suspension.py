from pathlib import Path

import yaml

from runner.config import REPO

KIN_DIR = REPO / "configs" / "parts" / "susp_kinematics" / "kin"
TOPO_DIR = REPO / "configs" / "parts" / "susp_topology"
KIN_REL = "configs/parts/susp_kinematics/kin"

BLUEPRINT_L3_SUSP = {
    "vehicle.sedan_l3": {"front": "mp_front_sedan", "rear": "ta_rear_sedan"},
    "vehicle.fsk_formula": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
    "vehicle.race_gt": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
}

VEHICLE_SUSP_DEFAULT = {
    "sedan": {"front": "mp_front_sedan", "rear": "ta_rear_sedan"},
    "sports": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
    "fsk_formula": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
    "race_car": {"front": "dw_front_sports", "rear": "5link_rear_sports"},
}


def susp_stem_from_ref(ref):
    if not ref:
        return ""
    s = str(ref)
    if s.startswith("susp."):
        return s.rsplit(".", 1)[-1]
    return Path(s).stem


def kin_yaml_path(stem):
    stem = susp_stem_from_ref(stem)
    if not stem:
        return None
    p = KIN_DIR / f"{stem}.yaml"
    return p if p.is_file() else None


def susp_rel_path(stem_or_ref):
    p = kin_yaml_path(stem_or_ref)
    if p is None:
        return None
    return f"{KIN_REL}/{p.name}"


def is_l3_kinematics_yaml(path_or_stem):
    p = kin_yaml_path(path_or_stem)
    if p is None:
        return False
    doc = yaml.safe_load(p.read_text()) or {}
    return bool(doc.get("type"))


def list_suspension_configs():
    if not KIN_DIR.is_dir() and not TOPO_DIR.is_dir():
        return []
    stems = {p.stem for p in KIN_DIR.glob("*.yaml")}
    stems |= {p.stem for p in TOPO_DIR.glob("*.yaml")}
    return sorted(stems)


def list_l3_kinematics_configs():
    if not KIN_DIR.is_dir():
        return []
    return sorted(p.stem for p in KIN_DIR.glob("*.yaml"))


def list_suspension_api(preview_all=False):
    l3 = list_l3_kinematics_configs()
    out = {"samples": l3, "l3_count": len(l3)}
    if preview_all:
        out["preview"] = list_suspension_configs()
    return out


def strip_fleet_susp_if_not_l3(spec):
    if str(spec.get("level", "L2")) != "L3":
        spec.pop("front_susp", None)
        spec.pop("rear_susp", None)


def l3_susp_path_warnings(fleet_spec):
    warnings = []
    for spec in fleet_spec:
        if str(spec.get("level", "L2")) != "L3":
            continue
        vid = int(spec.get("id", 0))
        for side, key in (("front", "front_susp"), ("rear", "rear_susp")):
            stem = spec.get(key)
            if not stem:
                continue
            ref = susp_stem_from_ref(stem)
            path = kin_yaml_path(ref)
            if path is None:
                warnings.append(
                    f"[vdsim] vehicle {vid}: {side} susp '{stem}' not found")
            elif not is_l3_kinematics_yaml(ref):
                warnings.append(
                    f"[vdsim] vehicle {vid}: {side} susp '{stem}' "
                    "is not L3-native (topology-only YAML)")
    return warnings


def validate_l3_susp_stem(stem, side, vid):
    if not stem:
        return
    ref = susp_stem_from_ref(stem)
    path = kin_yaml_path(ref)
    if path is None:
        raise ValueError(f"vehicle {vid}: {side} suspension '{stem}' not found")
    if not is_l3_kinematics_yaml(ref):
        raise ValueError(
            f"vehicle {vid}: {side} suspension '{stem}' is not L3-native")


def validate_fleet_updates(updates, fleet_spec):
    for upd in updates:
        vid = int(upd["id"])
        spec = next((f for f in fleet_spec if int(f["id"]) == vid), None)
        if spec is None:
            continue
        level = str(upd.get("level", spec.get("level", "L2")))
        if level != "L3":
            continue
        for side, key in (("front", "front_susp"), ("rear", "rear_susp")):
            if key in upd:
                validate_l3_susp_stem(upd[key], side, vid)


def suspension_default_for_vehicle(vehicle):
    stem = Path(str(vehicle)).stem
    return dict(VEHICLE_SUSP_DEFAULT.get(stem, {
        "front": "mp_front_sedan", "rear": "ta_rear_sedan",
    }))


def suspension_default_for_blueprint(blueprint_id):
    return dict(BLUEPRINT_L3_SUSP.get(blueprint_id, {
        "front": "mp_front_sedan", "rear": "ta_rear_sedan",
    }))


def corner_kinematic_links(doc):
    links = []
    for arm in ("lca", "uca"):
        block = doc.get(arm)
        if not isinstance(block, dict):
            continue
        cf, cr, kn = block["chassis_front"], block["chassis_rear"], block["knuckle"]
        links.extend([[cf, cr], [cf, kn], [cr, kn]])
    st = doc.get("strut")
    if isinstance(st, dict) and "top" in st and "bottom" in st:
        links.append([st["top"], st["bottom"]])
    tr = doc.get("tie_rod")
    if isinstance(tr, dict):
        links.append([tr["rack"], tr["knuckle"]])
    sd = doc.get("spring_damper")
    if isinstance(sd, dict) and "chassis" in sd and "lca" in sd:
        links.append([sd["chassis"], sd["lca"]])
    ap = doc.get("arm_pivot")
    if isinstance(ap, dict):
        pi, po = ap["chassis_inboard"], ap["chassis_outboard"]
        links.append([pi, po])
        wh = (doc.get("wheel") or {}).get("center")
        if wh:
            mid = [(pi[i] + po[i]) / 2.0 for i in range(3)]
            links.append([mid, wh])
    lb = doc.get("links")
    if isinstance(lb, dict):
        for block in lb.values():
            if isinstance(block, dict) and "chassis" in block and "knuckle" in block:
                links.append([block["chassis"], block["knuckle"]])
    hps = doc.get("hardpoints")
    if isinstance(hps, list):
        hp = {h["name"]: h["position"] for h in hps if isinstance(h, dict) and "name" in h}
        for a, b in (
            ("uca_inner_front", "uca_outer_ball"), ("uca_inner_rear", "uca_outer_ball"),
            ("lca_inner_front", "lca_outer_ball"), ("lca_inner_rear", "lca_outer_ball"),
            ("tie_rod_inner", "tie_rod_outer"), ("pushrod_lower", "pushrod_upper"),
            ("damper_top", "damper_bottom"),
        ):
            if a in hp and b in hp:
                links.append([hp[a], hp[b]])
    pts = {}
    wh = (doc.get("wheel") or {}).get("center")
    if wh:
        pts["wheel"] = wh
    return links, pts


def suspension_schematic(name):
    stem = susp_stem_from_ref(name)
    path = kin_yaml_path(stem) or (TOPO_DIR / f"{stem}.yaml")
    if not path.is_file():
        raise ValueError(f"unknown suspension: {name}")
    doc = yaml.safe_load(path.read_text()) or {}
    typ = str(doc.get("type") or doc.get("topology") or "unknown")
    links, pts = corner_kinematic_links(doc)
    return {"name": stem, "type": typ, "links": links, "points": pts}
