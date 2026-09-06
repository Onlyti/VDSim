#!/usr/bin/env python3
"""Unit tests for cmrosif ↔ VDS1 conversion (no ROS / no plant)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "cosim"))

from cmrosif_compat import (  # noqa: E402
    control_signal_to_vds1,
    parse_cmremote,
    vds1_state_to_dynamic_info,
)


def test_control_roundtrip_steer():
    ratio = 14.46
    wheel = 0.05
    cmd = control_signal_to_vds1(wheel * ratio, 0.2, 0.3, gear=1, steer_ratio=ratio)
    assert abs(cmd["steer"] - wheel) < 1e-9
    assert cmd["throttle"] == 0.3
    assert cmd["brake"] == 0.2
    assert cmd["gear"] == 1


def test_control_clamp_steer():
    max_sw = 480.0 * math.pi / 180.0
    cmd = control_signal_to_vds1(max_sw * 2.0, 0.0, 0.0, steer_ratio=14.46,
                                 max_steer_wheel_rad=max_sw)
    assert abs(cmd["steer"] - max_sw / 14.46) < 1e-6


def test_state_to_dynamic_info():
    st = {
        "vx": 10.0, "vy": 0.5, "vz": 0.0,
        "roll": 0.01, "pitch": -0.02, "yaw": 1.5,
        "roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.1,
        "ax": 1.0, "ay": -0.5,
        "steer_applied": 0.04,
        "throttle_applied": 0.6,
        "brake_applied": 0.0,
        "wheel_spin": [10.0, 10.1, 9.9, 10.0],
    }
    dyn = vds1_state_to_dynamic_info(st, steer_ratio=14.46, cycleno=42)
    assert dyn["cycleno"] == 42
    assert abs(dyn["Car_vx"] - 10.0) < 1e-9
    assert abs(dyn["Steer_WhlAng"] - 0.04 * 14.46) < 1e-9
    assert dyn["Vhcl_FL_rotv"] == 10.0


def test_cmremote_parse():
    assert parse_cmremote("SimStart", "") == "start"
    assert parse_cmremote("", "stop simulation") == "stop"
    assert parse_cmremote("cmd", "pause") == "pause"
    assert parse_cmremote("foo", "bar") is None


def main() -> int:
    test_control_roundtrip_steer()
    test_control_clamp_steer()
    test_state_to_dynamic_info()
    test_cmremote_parse()
    print("cmrosif_compat: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
