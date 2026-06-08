"""
Hardpoint importer — convert Adams-Car-style named hardpoints into VDSim YAML.

Adams Car uses `.tpl` and `.adm` text files with hardpoint definitions in
a proprietary format.  For interoperability we accept a simplified neutral
exchange format:

    NAME, X, Y, Z    (CSV with header `name,x,y,z`)

The importer auto-detects the suspension type from the hardpoint names
(Adams convention: `hpl_*` for left side, `hpr_*` for right), maps each
hardpoint to the VDSim YAML schema, and writes the resulting YAML.

Frame conversion:
    Adams typically uses (+x forward, +y outboard-LEFT, +z up).
    VDSim uses ISO 8855 RH:  same.  No conversion needed for left side.
    For right side: y → -y (or rely on the user to specify which side).

Supported types:
    - double_wishbone  (front)
    - macpherson       (front)
    - trailing_arm     (rear)  [planned]
    - five_link        (rear)  [planned]

Usage:
    python3 import_hardpoints.py \
        --input my_hardpoints.csv \
        --type double_wishbone --side left \
        --output configs/suspensions/my_dw_front.yaml
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

import yaml


# -----------------------------------------------------------------------------
# Name maps — Adams convention → VDSim YAML schema field path
# -----------------------------------------------------------------------------
# A leaf maps to (yaml_path_dot_separated, axis flip if right).  E.g.
# "lca.chassis_front" means hp["lca"]["chassis_front"].

DW_MAP: Dict[str, str] = {
    "hpl_lca_front":      "lca.chassis_front",
    "hpl_lca_rear":       "lca.chassis_rear",
    "hpl_lca_outer":      "lca.knuckle",
    "hpl_uca_front":      "uca.chassis_front",
    "hpl_uca_rear":       "uca.chassis_rear",
    "hpl_uca_outer":      "uca.knuckle",
    "hpl_tierod_inner":   "tie_rod.rack",
    "hpl_tierod_outer":   "tie_rod.knuckle",
    "hpl_wheel_center":   "wheel.center",
    "hpl_spring_lower":   "spring_damper.lca",
    "hpl_spring_upper":   "spring_damper.chassis",
    # Aliases (some Adams templates use these)
    "hpl_lca_inner_front": "lca.chassis_front",
    "hpl_lca_inner_rear":  "lca.chassis_rear",
    "hpl_uca_inner_front": "uca.chassis_front",
    "hpl_uca_inner_rear":  "uca.chassis_rear",
}

MP_MAP: Dict[str, str] = {
    "hpl_lca_front":      "lca.chassis_front",
    "hpl_lca_rear":       "lca.chassis_rear",
    "hpl_lca_outer":      "lca.knuckle",
    "hpl_strut_top":      "strut.top",
    "hpl_strut_lower":    "strut.bottom",
    "hpl_tierod_inner":   "tie_rod.rack",
    "hpl_tierod_outer":   "tie_rod.knuckle",
    "hpl_wheel_center":   "wheel.center",
    "hpl_lca_inner_front": "lca.chassis_front",
    "hpl_lca_inner_rear":  "lca.chassis_rear",
}

TA_MAP: Dict[str, str] = {
    "hpl_arm_inner":      "arm_pivot.chassis_inboard",
    "hpl_arm_outer_pivot": "arm_pivot.chassis_outboard",
    "hpl_wheel_center":   "wheel.center",
}

FL_MAP: Dict[str, str] = {
    "hpl_uf_inner":  "links.upper_fore.chassis",
    "hpl_uf_outer":  "links.upper_fore.knuckle",
    "hpl_ua_inner":  "links.upper_aft.chassis",
    "hpl_ua_outer":  "links.upper_aft.knuckle",
    "hpl_lf_inner":  "links.lower_fore.chassis",
    "hpl_lf_outer":  "links.lower_fore.knuckle",
    "hpl_la_inner":  "links.lower_aft.chassis",
    "hpl_la_outer":  "links.lower_aft.knuckle",
    "hpl_tl_inner":  "links.toe_link.chassis",
    "hpl_tl_outer":  "links.toe_link.knuckle",
    "hpl_wheel_center": "wheel.center",
}

SCHEMA_MAP = {
    "double_wishbone": DW_MAP,
    "macpherson":      MP_MAP,
    "trailing_arm":    TA_MAP,
    "five_link":       FL_MAP,
}


# -----------------------------------------------------------------------------
# CSV reading
# -----------------------------------------------------------------------------
def read_csv(path: Path) -> Dict[str, Tuple[float, float, float]]:
    out = {}
    with open(path) as f:
        rd = csv.DictReader(f)
        for row in rd:
            name = row["name"].strip().lower()
            try:
                out[name] = (float(row["x"]), float(row["y"]), float(row["z"]))
            except (ValueError, KeyError):
                continue
    return out


# -----------------------------------------------------------------------------
# Auto-detect type by which keys are present
# -----------------------------------------------------------------------------
def autodetect_type(points: Dict) -> str:
    keys = set(points.keys())
    if {"hpl_uca_outer", "hpl_lca_outer"} <= keys:
        return "double_wishbone"
    if {"hpl_strut_top", "hpl_lca_outer"} <= keys:
        return "macpherson"
    if {"hpl_uf_outer", "hpl_la_outer"} <= keys:
        return "five_link"
    if {"hpl_arm_inner"} <= keys:
        return "trailing_arm"
    raise ValueError(f"Cannot auto-detect type. Keys: {sorted(keys)[:6]}")


# -----------------------------------------------------------------------------
# Build YAML dict from CSV + type
# -----------------------------------------------------------------------------
def build_yaml(points: Dict[str, Tuple[float, float, float]],
                type_: str,
                side: str = "left",
                wheel_radius: float = 0.30,
                spin_axis = (0.0, 1.0, 0.0)) -> dict:
    name_map = SCHEMA_MAP[type_]
    flip = -1.0 if side == "right" else 1.0
    out = {"type": type_, "side": side,
           "wheel": {"spin_axis": list(spin_axis), "static_radius": wheel_radius}}

    def set_path(d: dict, path: str, value: list):
        parts = path.split(".")
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = value

    for adams_name, schema_path in name_map.items():
        if adams_name in points:
            x, y, z = points[adams_name]
            set_path(out, schema_path, [round(x, 6), round(y * flip, 6), round(z, 6)])

    if "center" not in out.get("wheel", {}):
        raise ValueError("Missing hpl_wheel_center in input — required for all types.")
    return out


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def import_csv_to_yaml(input_path: Path, output_path: Path,
                       type_: str = "auto", side: str = "left",
                       wheel_radius: float = 0.30) -> dict:
    points = read_csv(input_path)
    resolved = autodetect_type(points) if type_ == "auto" else type_
    doc = build_yaml(points, resolved, side, wheel_radius)
    with open(output_path, "w") as f:
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)
    return doc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, type=Path,
                    help="CSV with columns name,x,y,z")
    ap.add_argument("--output", required=True, type=Path,
                    help="output YAML path")
    ap.add_argument("--type", choices=list(SCHEMA_MAP) + ["auto"],
                    default="auto")
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--wheel_radius", type=float, default=0.30)
    args = ap.parse_args()

    points = read_csv(args.input)
    type_ = autodetect_type(points) if args.type == "auto" else args.type
    print(f"[import] type = {type_}, side = {args.side}, {len(points)} hardpoints")

    import_csv_to_yaml(args.input, args.output, type_, args.side, args.wheel_radius)
    print(f"[import] wrote {args.output}")


if __name__ == "__main__":
    main()
