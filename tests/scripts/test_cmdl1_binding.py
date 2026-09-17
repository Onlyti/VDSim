#!/usr/bin/env python3
"""CmdL1 per-wheel torque arrays must support element-wise assignment."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "build" / "python"))

import vdsim  # noqa: E402


def test_element_assign():
    cmd = vdsim.CmdL1()
    cmd.motor_torque[2] = 999.0
    assert list(cmd.motor_torque)[2] == 999.0
    cmd.brake_torque[1] = 42.0
    assert list(cmd.brake_torque)[1] == 42.0
    cmd.motor_torque = [1.0, 2.0, 3.0, 4.0]
    assert list(cmd.motor_torque) == [1.0, 2.0, 3.0, 4.0]


if __name__ == "__main__":
    test_element_assign()
    print("test_cmdl1_binding: ok")
