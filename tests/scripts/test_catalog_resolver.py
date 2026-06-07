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

    try:
        CatalogResolver.validate_part_envelope({
            "id": "x",
            "type": "chassis",
            "version": 1,
            "schema": "topology_preview_v1",
            "label": "x",
            "body": {},
        })
        raise AssertionError("expected PartEnvelopeError for topology_preview")
    except PartEnvelopeError:
        pass

    with tempfile.TemporaryDirectory() as td:
        resolved = r.resolve_blueprint("vehicle.sedan_comfort", out_dir=Path(td))
        assert resolved.vehicle_yaml.is_file()
        assert resolved.tire_yaml.is_file()
        vp_cat = vdsim.VehicleParams.from_yaml(str(resolved.vehicle_yaml))
        vp_ref = vdsim.VehicleParams.from_yaml(str(REPO / "configs/vehicles/sedan.yaml"))
        assert abs(vp_cat.mass - vp_ref.mass) < 1e-6
        assert abs(vp_cat.wheelbase - vp_ref.wheelbase) < 1e-6
        assert abs(vp_cat.max_motor_torque - vp_ref.max_motor_torque) < 1e-6
        tp_cat = vdsim.TireParams.from_yaml(str(resolved.tire_yaml))
        tp_ref = vdsim.TireParams.from_yaml(str(REPO / "configs/tires/default_pacejka.yaml"))
        assert abs(tp_cat.mu_nominal - tp_ref.mu_nominal) < 1e-6

    try:
        r.resolve_blueprint("vehicle.no_such")
        raise AssertionError("expected CatalogError for unknown blueprint")
    except CatalogError:
        pass

    print("test_catalog_resolver: ok")


if __name__ == "__main__":
    main()
