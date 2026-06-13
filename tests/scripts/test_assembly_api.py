#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "build" / "python"))

from catalog.assembly import fleet_assembly_view
from catalog.ids import DEFAULT_BLUEPRINT


def test_sedan_assembly_slots():
    spec = {
        "id": 0,
        "blueprint": DEFAULT_BLUEPRINT,
        "level": "L2",
        "parts": {},
        "vehicle": "sedan",
        "tire": "default_pacejka",
    }
    asm = fleet_assembly_view(spec, out_dir=ROOT / "build" / "_asm_test")
    assert asm["blueprint"]["id"] == DEFAULT_BLUEPRINT
    slots = {s["slot"] for s in asm["slots"]}
    assert "body" in slots and "tire" in slots
    assert "front_chassis" not in slots
    assert asm["summary"]["mass_kg"] > 500


def test_l3_has_susp_slots():
    spec = {
        "id": 0,
        "blueprint": "vehicle.sedan_l3",
        "level": "L3",
        "parts": {},
    }
    asm = fleet_assembly_view(spec, out_dir=ROOT / "build" / "_asm_test")
    slots = {s["slot"] for s in asm["slots"]}
    assert "front_chassis" in slots and "rear_chassis" in slots


def test_assembly_garage_meta():
    spec = {
        "id": 0,
        "blueprint": DEFAULT_BLUEPRINT,
        "level": "L2",
        "parts": {},
        "vehicle": "sedan",
        "tire": "default_pacejka",
    }
    asm = fleet_assembly_view(spec, out_dir=ROOT / "build" / "_asm_test")
    assert asm.get("categories")
    assert asm.get("recommended")
    tire_slot = next(s for s in asm["slots"] if s["slot"] == "tire")
    assert tire_slot["candidates"][0].get("card")
    assert "ok" in tire_slot["candidates"][0]


def test_assembly_preview_delta():
    from catalog.assembly import assembly_preview
    from catalog.resolver import CatalogResolver
    spec = {
        "id": 0,
        "blueprint": DEFAULT_BLUEPRINT,
        "level": "L2",
        "parts": {},
        "vehicle": "sedan",
        "tire": "default_pacejka",
    }
    r = CatalogResolver(ROOT)
    prev = assembly_preview(
        spec, "tire", "tire.sport_grip",
        resolver=r, out_dir=ROOT / "build" / "_asm_test")
    assert prev["candidate"] == "tire.sport_grip"
    assert "tire_mu" in prev.get("delta", {}) or prev["summary"].get("tire_mu")


if __name__ == "__main__":
    test_sedan_assembly_slots()
    test_l3_has_susp_slots()
    test_assembly_garage_meta()
    test_assembly_preview_delta()
    print("ok")
