#!/usr/bin/env python3
"""Run-config draft export/import round-trip (comms + telemetry metadata)."""
import copy
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "cosim"))
sys.path.insert(0, str(REPO / "gui"))

from server import RUNNER  # noqa: E402


def _backup(r):
    with r.lock:
        return {
            "cfg_ports": (r.cfg.get("cosim_cmd_port"), r.cfg.get("cosim_state_port")),
            "cosim_ports": (r.cosim.cfg.get("cmd_port"), r.cosim.cfg.get("state_port")),
            "tx0": copy.deepcopy(r.ports[0].tx),
            "targets0": copy.deepcopy(r.ports[0].targets),
            "infra": list(r.infra_sensors),
        }


def _restore(r, b):
    with r.lock:
        r.cfg["cosim_cmd_port"] = b["cfg_ports"][0]
        r.cfg["cosim_state_port"] = b["cfg_ports"][1]
        r.cosim.cfg["cmd_port"] = b["cosim_ports"][0]
        r.cosim.cfg["state_port"] = b["cosim_ports"][1]
        r.ports[0].tx = copy.deepcopy(b["tx0"])
        r.ports[0].targets = copy.deepcopy(b["targets0"])
        r.infra_sensors = list(b["infra"])


def main():
    r = RUNNER
    bak = _backup(r)
    try:
        with r.lock:
            r.cfg["cosim_cmd_port"] = 7501
            r.cfg["cosim_state_port"] = 7502
            r._sync_cosim_ports()
            r.ports[0].tx["enabled"] = True
            r.ports[0].tx["rate"] = 42.0
            r.ports[0].targets = [{"ip": "10.0.0.5", "port": 9100}]
            r.infra_sensors = [{"id": "tower1", "type": "radar", "label": "tower1"}]

        doc = r.export_run_config()
        gui = doc.get("gui") or {}
        assert gui.get("cosim_cmd_port") == 7501, gui.get("cosim_cmd_port")
        assert gui.get("cosim_state_port") == 7502, gui.get("cosim_state_port")
        tel = gui.get("telemetry") or {}
        assert tel["0"]["tx"]["enabled"] is True
        assert tel["0"]["tx"]["rate"] == 42.0
        assert tel["0"]["targets"][0]["port"] == 9100

        r.import_run_config(doc)
        assert r.cfg["cosim_cmd_port"] == 7501
        assert r.cfg["cosim_state_port"] == 7502
        assert r.cosim.cfg["cmd_port"] == 7501
        assert r.cosim.cfg["state_port"] == 7502
        p0 = r.ports[0]
        assert p0.tx["enabled"] is True
        assert p0.tx["rate"] == 42.0
        assert p0.targets[0]["ip"] == "10.0.0.5"
        assert len(r.infra_sensors) == 1

        fleet_bak = list(r.fleet_spec)
        live_bak = r.live_vid
        try:
            with r.lock:
                r.fleet_spec = []
            r._fleet_add()
            assert len(r.fleet_spec) == 1
            assert r.fleet_spec[0]["id"] == 0
            r._fleet_add()
            assert len(r.fleet_spec) == 2
        finally:
            with r.lock:
                r.fleet_spec = fleet_bak
                r.live_vid = live_bak

        print("test_runconfig_roundtrip: ok")
    finally:
        _restore(r, bak)


if __name__ == "__main__":
    main()
