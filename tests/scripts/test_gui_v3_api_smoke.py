#!/usr/bin/env python3
"""Headless smoke for GUI v3 backend + build artifact (no browser)."""
import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT / "build" / "python", ROOT / "python", ROOT / "gui", ROOT / "cosim"):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

from catalog.assembly import fleet_assembly_view  # noqa: E402
from runner.catalog_api import catalog_kin_save, catalog_part_delete  # noqa: E402
from runner.suspension import (  # noqa: E402
    load_suspension_kin,
    preview_suspension_kin,
    suspension_kc_plots,
    suspension_schematic,
)

STEM = "gui_v3_smoke_kin"


def test_kin_load_preview():
    r = load_suspension_kin(part_id="chassis.mp_front_sedan")
    assert r["ok"]
    assert "lca.knuckle" in r["geometry"]
    doc = copy.deepcopy(r["doc"])
    doc["lca"]["knuckle"][1] += 0.005
    p = preview_suspension_kin(doc)
    assert len(p["plots"]) >= 7
    assert p["schematic"]["links"]


def test_kin_save_roundtrip():
    r = load_suspension_kin(part_id="chassis.mp_front_sedan")
    pid = f"chassis.{STEM}"
    try:
        catalog_part_delete(pid)
    except Exception:
        pass
    out = catalog_kin_save(r["doc"], STEM, "GUI v3 smoke", base_part_id=None)
    assert out["part_id"] == pid
    r2 = load_suspension_kin(part_id=pid)
    assert len(r2["geometry"]) > 5
    catalog_part_delete(pid)


def test_schematic_and_kc():
    sch = suspension_schematic("mp_front_sedan")
    assert sch["links"]
    kc = suspension_kc_plots("mp_front_sedan")
    assert kc["plots"][0]["ylabel"] == "camber [deg]"


def test_l4_assembly():
    spec = {"id": 0, "blueprint": "vehicle.sedan_l3", "level": "L4", "parts": {}}
    asm = fleet_assembly_view(spec, out_dir=ROOT / "build" / "_gui_v3_smoke")
    assert asm["level"] == "L4"
    assert any(s["slot"] == "front_chassis" for s in asm["slots"])


def test_v3_dist_built():
    dist = ROOT / "gui" / "v3" / "dist" / "index.html"
    assert dist.is_file(), "missing gui/v3/dist — run: cd gui/v3 && npm run build"


def test_scenario_templates():
    from server import Runner

    r = Runner()
    for key, preset in (
        ("empty", "custom"),
        ("figure8", "figure8"),
        ("straight", "straight"),
        ("skidpad", "skidpad"),
    ):
        r.apply_scenario_template(key)
        s = r.get_setup(include_geom=False, include_scenarios=False)
        assert len(s["fleet"]) == 1
        assert s["path_preset"] == preset
        assert len(s["path_pts"]) >= 2


def main():
    test_kin_load_preview()
    test_kin_save_roundtrip()
    test_schematic_and_kc()
    test_l4_assembly()
    test_v3_dist_built()
    test_scenario_templates()
    print("ok gui_v3_api_smoke")


if __name__ == "__main__":
    main()
