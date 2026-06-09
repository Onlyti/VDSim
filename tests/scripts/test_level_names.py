#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from catalog.levels import LEVELS, LEVEL_LADDER, level_label, level_title


def test_ladder_ids():
    assert LEVELS == ["K", "L1", "L2", "L3", "L4", "L5"]
    assert [e["ld"] for e in LEVEL_LADDER] == [
        "Ld0", "Ld1", "Ld2", "Ld3", "Ld4", "Ld5"]


def test_labels():
    assert level_label("L2") == "L2 — Seven-DOF"
    assert level_title("K") == "Kinematic bicycle"


if __name__ == "__main__":
    test_ladder_ids()
    test_labels()
    print("ok")
