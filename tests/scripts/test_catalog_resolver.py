#!/usr/bin/env python3
"""v0.3 M1 — catalog manifest, part envelope, blueprint resolver."""
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim  # noqa: E402
from catalog import CatalogResolver, CatalogError, PartEnvelopeError  # noqa: E402


def main():
    r = CatalogResolver(REPO)
    r.load_manifest()
    parts = r.list_parts()
    ids = {p["id"] for p in parts}
    assert "chassis.sedan" in ids
    assert "tire.default_pacejka" in ids
    assert "susp.mp_front_sedan" in ids

    try:
        CatalogResolver.validate_part_envelope({
            "id": "x",
            "type": "susp_topology",
            "version": 1,
            "schema": "topology_preview_v1",
            "label": "x",
            "body": {"path": "parts/susp_topology/x.yaml"},
        })
    except PartEnvelopeError:
        raise AssertionError("topology_preview_v1 should load in catalog")

    with tempfile.TemporaryDirectory() as td:
        resolved = r.resolve_blueprint("vehicle.sedan_comfort", out_dir=Path(td))
        assert resolved.vehicle_yaml.is_file()
        assert resolved.tire_yaml.is_file()
        vp_cat = vdsim.VehicleParams.from_yaml(str(resolved.vehicle_yaml))
        assert abs(vp_cat.mass - 1500.0) < 1e-6
        assert abs(vp_cat.wheelbase - 2.70) < 1e-6
        assert abs(vp_cat.max_motor_torque - 300.0) < 1e-6
        tp_cat = vdsim.TireParams.from_yaml(str(resolved.tire_yaml))
        assert abs(tp_cat.mu_nominal - 1.0) < 1e-6

        l3 = r.resolve_blueprint("vehicle.sedan_l3", out_dir=Path(td) / "l3")
        assert l3.susp_front and l3.susp_front.is_file()
        assert l3.susp_rear and l3.susp_rear.is_file()

    try:
        r.resolve_blueprint("vehicle.no_such")
        raise AssertionError("expected CatalogError for unknown blueprint")
    except CatalogError:
        pass

    print("test_catalog_resolver: ok")


if __name__ == "__main__":
    main()
