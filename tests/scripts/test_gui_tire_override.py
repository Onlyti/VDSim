#!/usr/bin/env python3
"""Live-vehicle tire edits must survive sync/materialize (e.g. lugre.enabled)."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "cosim"))
sys.path.insert(0, str(REPO / "gui"))

import yaml  # noqa: E402
import vdsim  # noqa: E402
from server import RUNNER  # noqa: E402


def main():
    r = RUNNER
    vid = r.live_vid
    with r.lock:
        r.fleet_overrides.pop(vid, None)
        r.tp = vdsim.TireParams.from_yaml(str(REPO / "configs/parts/tire/default_pacejka.yaml"))
        assert r.tp.lugre.enabled

    r.set_params("tire", {"lugre.enabled": False}, vehicle_id=vid)
    assert r.tp.lugre.enabled is False
    assert r.fleet_overrides[vid]["tire"]["lugre.enabled"] is False

    r._sync_live_from_fleet()
    assert r.tp.lugre.enabled is False

    _, doc = r._materialize_run_config()
    tire_path = Path(doc["vehicles"][0]["tire"])
    body = yaml.safe_load(tire_path.read_text())
    assert body["lugre"]["enabled"] is False

    fields = r.serialize_tire(vehicle_id=vid)
    lug = next(f for f in fields if f["name"] == "lugre.enabled")
    assert lug["value"] is False
    print("ok")


if __name__ == "__main__":
    main()
