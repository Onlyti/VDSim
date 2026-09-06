#!/usr/bin/env python3
"""v0.3 M3 — catalog scene materialize + cosim world shape."""
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from catalog import (  # noqa: E402
    CatalogResolver,
    is_catalog_scene_file,
    materialize_scene_file,
)


def main():
    scene = REPO / "configs" / "scenes" / "two_vehicle_race.yaml"
    run_cfg = REPO / "configs" / "scenes" / "l3_sedan_kinematics.yaml"
    assert is_catalog_scene_file(scene)
    assert is_catalog_scene_file(run_cfg)

    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "world.yaml"
        materialize_scene_file(scene, out)
        doc = yaml.safe_load(out.read_text()) or {}
        assert len(doc.get("vehicles", [])) == 2
        for v in doc["vehicles"]:
            assert Path(v["vehicle"]).is_file()
            assert Path(v["tire"]).is_file()
        # scene sensor declarations (mount pose) must survive materialize untouched,
        # or the cosim parser never sees them (CONFIG_GUIDE 2.3).
        s0 = doc["vehicles"][0].get("sensors")
        assert s0, "fleet[0].sensors must reach the world YAML"
        assert [x["id"] for x in s0] == ["gnss_roof", "imu_cg", "enc", "cam_front"]
        assert s0[0]["mount"]["pos"] == [0.20, 0.0, 1.42], s0[0]
        assert s0[3]["mount"]["rpy"] == [0, -0.05, 0], s0[3]
        assert "sensors" not in doc["vehicles"][1]

        l3_out = Path(td) / "l3_world.yaml"
        materialize_scene_file(run_cfg, l3_out)
        l3 = yaml.safe_load(l3_out.read_text()) or {}
        v0 = l3["vehicles"][0]
        assert v0["level"] == "L3"
        assert "front_susp" in v0 and "rear_susp" in v0
        assert Path(v0["front_susp"]).is_file()

        jump = REPO / "configs" / "scenes" / "jump_ramp_demo.yaml"
        jump_out = Path(td) / "jump_world.yaml"
        materialize_scene_file(jump, jump_out)
        jump_doc = yaml.safe_load(jump_out.read_text()) or {}
        assert jump_doc["vehicles"][0]["level"] == "L5"
        assert jump_doc.get("stunt", {}).get("ground") == "ramp"

        terr = REPO / "configs" / "scenes" / "terrain_hill_demo.yaml"
        terr_out = Path(td) / "terrain_world.yaml"
        materialize_scene_file(terr, terr_out)
        terr_doc = yaml.safe_load(terr_out.read_text()) or {}
        assert terr_doc["vehicles"][0]["level"] == "L5"
        assert "terrain" in terr_doc, "terrain path must reach the world YAML"
        assert Path(terr_doc["terrain"]).is_file(), terr_doc["terrain"]

        bank = REPO / "configs" / "scenes" / "banked_grade_demo.yaml"
        bank_out = Path(td) / "bank_world.yaml"
        materialize_scene_file(bank, bank_out)
        bank_doc = yaml.safe_load(bank_out.read_text()) or {}
        assert bank_doc["vehicles"][0]["level"] == "L5"
        assert bank_doc.get("grade") and bank_doc.get("bank"), "grade/bank must reach world"

        oval = REPO / "configs" / "scenes" / "banked_oval.yaml"
        oval_out = Path(td) / "oval_world.yaml"
        materialize_scene_file(oval, oval_out)
        oval_doc = yaml.safe_load(oval_out.read_text()) or {}
        assert oval_doc["vehicles"][0]["level"] == "L5"
        assert oval_doc.get("stunt", {}).get("ground") == "banked"
        assert oval_doc.get("bank"), "bank must reach world for the curved turn"

    r = CatalogResolver(REPO)
    assert r.load_blueprint("vehicle.sedan_comfort")
    print("test_scene_materialize: ok")


if __name__ == "__main__":
    main()
