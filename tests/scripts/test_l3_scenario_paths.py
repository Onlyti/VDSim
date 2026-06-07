#!/usr/bin/env python3
"""Verify L3 scene materializes attachable suspension paths under catalog kin/."""
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

from catalog import CatalogResolver, materialize_scene_world  # noqa: E402

KIN = REPO / "configs" / "parts" / "susp_kinematics" / "kin"
SCN = "l3_sedan_kinematics"


def is_l3_native(path: Path) -> bool:
    doc = yaml.safe_load(path.read_text()) or {}
    return bool(doc.get("type"))


def main():
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "world.yaml"
        materialize_scene_world(REPO, SCN, out)
        doc = yaml.safe_load(out.read_text()) or {}
    for v in doc.get("vehicles", []):
        if str(v.get("level", "")) != "L3":
            continue
        for key in ("front_susp", "rear_susp"):
            if key not in v:
                print(f"FAIL: L3 vehicle missing {key}")
                return 1
            path = Path(str(v[key]))
            if not path.is_file():
                print(f"FAIL: missing resolved file {path}")
                return 1
            try:
                rel_s = str(path.relative_to(REPO)).replace("\\", "/")
            except ValueError:
                rel_s = str(path).replace("\\", "/")
            if "susp_kinematics/kin" not in rel_s:
                print(f"FAIL: {key} not under catalog kin/: {rel_s}")
                return 1
            if not is_l3_native(path):
                print(f"FAIL: {path.name} is not L3-native")
                return 1
    with tempfile.TemporaryDirectory() as td2:
        r = CatalogResolver(REPO)
        resolved = r.resolve_blueprint("vehicle.sedan_l3", out_dir=Path(td2) / "bp")
        assert resolved.susp_front and resolved.susp_front.is_file()
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
