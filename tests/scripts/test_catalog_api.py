#!/usr/bin/env python3
"""v0.3 M4 — GUI catalog API and scene v3 export/import."""
import copy
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))
sys.path.insert(0, str(REPO / "cosim"))
sys.path.insert(0, str(REPO / "gui"))

from runner.catalog_api import (  # noqa: E402
    catalog_blueprint_get,
    catalog_index,
    catalog_legacy_registry,
    catalog_part_get,
    catalog_suspension_samples,
)
from server import RUNNER  # noqa: E402


def main():
    idx = catalog_index()
    assert idx.get("catalog_id") == "vdsim.builtin"
    assert any(p["id"] == "body.sedan" for p in idx["parts"])
    assert any(b["id"] == "vehicle.sedan_comfort" for b in idx["blueprints"])

    tires = catalog_index(type_filter="tire")
    assert all(p["type"] == "tire" for p in tires["parts"])

    part = catalog_part_get("tire.default_pacejka")
    assert part["schema"] == "pacejka_mf96_v1"
    bp = catalog_blueprint_get("vehicle.sedan_comfort")
    assert bp["parts"]["body"] == "body.sedan"

    reg = catalog_legacy_registry()
    assert "sedan" in reg["vehicles"]
    assert "default_pacejka" in reg["tires"]

    susp = catalog_suspension_samples()
    assert "mp_front_sedan" in susp["samples"]

    r = RUNNER
    bak_fleet = copy.deepcopy(r.fleet_spec)
    try:
        doc = r.export_simconfig()
        assert doc.get("version") == 3
        assert doc.get("fleet")
        assert doc.get("gui")
        r.import_simconfig(doc)
        assert len(r.fleet_spec) == len(bak_fleet)
    finally:
        r.fleet_spec = bak_fleet

    run_doc = r.export_run_config()
    assert run_doc.get("vehicles")
    assert run_doc.get("fleet")

    try:
        r.import_simconfig({"version": 2, "fleet_spec": []})
        raise AssertionError("expected v2 rejection")
    except ValueError:
        pass

    print("test_catalog_api: ok")


if __name__ == "__main__":
    main()
