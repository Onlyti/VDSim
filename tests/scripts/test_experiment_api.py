#!/usr/bin/env python3
"""Lock the experiment-API seam: vdsim_lab.Sim (set_input / run_core_dt) drives
the core, sensors report at the mount, logging + metrics work. PYTHONPATH is set
by ctest (build/python + python)."""
import math
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

from vdsim_lab import Sim, Road, Sensors


def test_throttle_then_brake():
    sim = Sim(level="L2", road=Road.flat(mu=1.0), v0=10.0)
    for _ in range(int(2.0 / sim.dt)):
        sim.set_input(throttle=1.0); sim.run_core_dt()
    v_after_accel = sim.state()["vx"]
    assert v_after_accel > 11.0, f"throttle should accelerate (got {v_after_accel:.2f})"
    for _ in range(int(2.0 / sim.dt)):
        sim.set_input(brake=0.8); sim.run_core_dt()
    assert sim.state()["vx"] < v_after_accel - 1.0, "brake should decelerate"


def test_step_steer_yaws():
    sim = Sim(level="L2", road=Road.flat(mu=1.0), v0=15.0)
    for _ in range(int(3.0 / sim.dt)):
        sim.set_input(steer=0.04, throttle=0.1); sim.run_core_dt()
    assert abs(sim.state()["r"]) > 1e-2, "step steer should produce yaw rate"


def test_cmd_object_accepted():
    import vdsim
    sim = Sim(level="L1", road=Road.flat(), v0=5.0)
    c = vdsim.CmdL4(); c.throttle = 1.0
    sim.set_input(c)
    sim.run_core_dt()
    assert len(sim.rows) == 1


def test_mount_pose_shifts_gnss():
    sim = Sim(level="L2", road=Road.flat(), v0=8.0,
              sensors=Sensors().gnss(pos_std=0.0),    # no noise -> deterministic
              sensor_mounts={"gnss": {"type": "gnss", "pos": [2.0, 0.0, 0.0]}})
    sim.set_input(throttle=0.2); sim.run_core_dt()
    cg = sim.measurements()["gnss"]
    at_mount = sim.measurements("gnss")
    # yaw ~ 0 at start -> mount 2 m ahead in +x: gnss x shifted ~ +2 m vs CG
    assert at_mount["x"] - cg["x"] > 1.5, "mount lever arm should shift GNSS x"


def test_log_metrics_csv():
    sim = Sim(level="L2", road=Road.flat(), v0=12.0)
    for _ in range(int(2.0 / sim.dt)):
        sim.set_input(throttle=0.5); sim.run_core_dt()
    mets = sim.metrics(["peak_ay", "vmax", "dist"])
    assert all(math.isfinite(v) for v in mets.values()), mets
    p = Path(tempfile.gettempdir()) / "vdsim_exp_api.csv"
    sim.to_csv(p)
    lines = p.read_text().splitlines()
    assert len(lines) == len(sim.rows) + 1, "csv = header + one row per step"
    p.unlink()


if __name__ == "__main__":
    test_throttle_then_brake()
    test_step_steer_yaws()
    test_cmd_object_accepted()
    test_mount_pose_shifts_gnss()
    test_log_metrics_csv()
    print("OK test_experiment_api")
