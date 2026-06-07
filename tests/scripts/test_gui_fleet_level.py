#!/usr/bin/env python3
"""Fleet level must persist via /api/setup and scene materialize."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "cosim"))
sys.path.insert(0, str(REPO / "gui"))

from server import RUNNER  # noqa: E402


def main():
    r = RUNNER
    with r.lock:
        r.fleet_spec[0]["level"] = "L2"
    r.apply_setup({"fleet": [{"id": 0, "level": "L3"}]})
    assert r.fleet_spec[0]["level"] == "L3", r.fleet_spec[0].get("level")
    assert r.cfg["level"] == "L3"
    doc = r.export_run_config()
    assert doc["fleet"][0]["level"] == "L3"
    assert doc["vehicles"][0]["level"] == "L3"
    print("ok")


if __name__ == "__main__":
    main()
