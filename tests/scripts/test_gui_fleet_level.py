#!/usr/bin/env python3
"""Fleet level must persist via /api/setup and scene materialize."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "cosim"))
sys.path.insert(0, str(REPO / "gui"))

from runner.params_io import serialize_fields  # noqa: E402
from runner.params_schema import VEHICLE_FIELDS, TIRE_FIELDS  # noqa: E402
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

    r.apply_setup({"fleet": [{"id": 0, "level": "L4"}]})
    assert r.fleet_spec[0]["level"] == "L4"
    assert r.fleet_spec[0].get("front_susp")
    fields_l4 = serialize_fields(r._vp_for_vid(0), VEHICLE_FIELDS)
    mass_sprung = next(x for x in fields_l4 if x["name"] == "mass_sprung")
    assert "L4" in mass_sprung["levels"]

    r.apply_setup({"fleet": [{"id": 0, "level": "L5"}]})
    fields = serialize_fields(r._vp_for_vid(0), VEHICLE_FIELDS)
    for name in ("mass", "cg_height", "ixx", "izz", "aero_drag_coeff"):
        f = next(x for x in fields if x["name"] == name)
        assert "L5" in f["levels"], (name, f["levels"])
    tire = serialize_fields(r._tp_for_vid(0), TIRE_FIELDS)
    assert "L5" in next(x for x in tire if x["name"] == "mu_nominal")["levels"]
    spring = next(x for x in fields if x["name"] == "spring_stiffness")
    assert "L5" not in spring["levels"]
    print("ok")


if __name__ == "__main__":
    main()
