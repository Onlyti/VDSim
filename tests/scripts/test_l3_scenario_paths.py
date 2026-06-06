#!/usr/bin/env python3
"""Verify L3 sample scenario stores attachable suspension relative paths."""
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
SUSP = REPO / "configs" / "suspensions"
SCN = REPO / "configs" / "scenarios" / "l3_sedan_kinematics.yaml"


def is_l3_native(stem):
    doc = yaml.safe_load((SUSP / f"{stem}.yaml").read_text()) or {}
    return bool(doc.get("type"))


def main():
    if not SCN.is_file():
        print(f"FAIL: missing {SCN.name}")
        return 1
    doc = yaml.safe_load(SCN.read_text()) or {}
    for v in doc.get("vehicles", []):
        if str(v.get("level", "")) != "L3":
            continue
        for key in ("front_susp", "rear_susp"):
            if key not in v:
                print(f"FAIL: L3 vehicle missing {key}")
                return 1
            path = str(v[key])
            if not path.startswith("configs/suspensions/"):
                print(f"FAIL: {key} not under configs/suspensions/: {path}")
                return 1
            stem = Path(path).stem
            if not (SUSP / f"{stem}.yaml").is_file():
                print(f"FAIL: missing file for {path}")
                return 1
            if not is_l3_native(stem):
                print(f"FAIL: {stem} is not L3-native (type: schema)")
                return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
