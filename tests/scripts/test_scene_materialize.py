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

        l3_out = Path(td) / "l3_world.yaml"
        materialize_scene_file(run_cfg, l3_out)
        l3 = yaml.safe_load(l3_out.read_text()) or {}
        v0 = l3["vehicles"][0]
        assert v0["level"] == "L3"
        assert "front_susp" in v0 and "rear_susp" in v0
        assert Path(v0["front_susp"]).is_file()

    r = CatalogResolver(REPO)
    assert r.load_blueprint("vehicle.sedan_comfort")
    print("test_scene_materialize: ok")


if __name__ == "__main__":
    main()
