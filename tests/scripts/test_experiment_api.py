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


def test_state_beta():
    sim = Sim(level="L2", road=Road.flat(), v0=15.0)
    for _ in range(int(2.0 / sim.dt)):
        sim.set_input(steer=0.05, throttle=0.1); sim.run_core_dt()
    st = sim.state()
    assert "beta" in st
    import math
    expected = math.atan2(st["vy"], st["vx"])
    assert abs(st["beta"] - expected) < 1e-9


def test_log_extra_in_csv():
    import csv as _csv
    sim = Sim(level="L2", road=Road.flat(), v0=10.0)
    for k in range(10):
        sim.set_input(throttle=0.3); sim.run_core_dt()
        sim.log_extra({"ax_cmd": 0.3, "step_idx": float(k)})
    p = Path(tempfile.gettempdir()) / "vdsim_extra.csv"
    sim.to_csv(p)
    with open(p) as f:
        rows = list(_csv.DictReader(f))
    assert "ax_cmd" in rows[0] and "step_idx" in rows[0]
    assert float(rows[-1]["step_idx"]) == 9.0
    p.unlink()


def test_reset_reuses_plant():
    sim = Sim(level="L2", road=Road.flat(), v0=12.0)
    for _ in range(100): sim.set_input(throttle=1.0); sim.run_core_dt()
    assert len(sim.rows) == 100
    sim.reset()
    assert len(sim.rows) == 0
    assert abs(sim.state()["vx"] - 12.0) < 0.1
    sim.reset(v0=20.0)
    assert abs(sim.state()["vx"] - 20.0) < 0.1


def test_register_metric():
    from vdsim_lab import register_metric, compute_metrics
    register_metric("always_one", lambda res, **_: 1.0)
    sim = Sim(level="L1", road=Road.flat(), v0=10.0)
    for _ in range(50): sim.set_input(throttle=0.2); sim.run_core_dt()
    m = sim.metrics(["always_one", "vmax"])
    assert m["always_one"] == 1.0
    assert math.isfinite(m["vmax"])


def test_plot_comparison():
    sim_a = Sim(level="L2", road=Road.flat(), v0=10.0)
    sim_b = Sim(level="L2", road=Road.flat(), v0=15.0)
    for _ in range(100):
        sim_a.set_input(throttle=0.3); sim_a.run_core_dt()
        sim_b.set_input(throttle=0.3); sim_b.run_core_dt()
    try:
        from vdsim_lab import plot_comparison
        p = Path(tempfile.gettempdir()) / "vdsim_cmp.png"
        plot_comparison({"A": sim_a, "B": sim_b}, path=p, signals=("vx", "ay"))
        assert p.stat().st_size > 0
        p.unlink()
    except RuntimeError:
        pass   # matplotlib not installed — not a failure


def test_ref_point_position():
    sim_cg = Sim(level="L2", road=Road.flat(), v0=15.0)
    sim_ra = Sim(level="L2", road=Road.flat(), v0=15.0, ref_point="rear_axle")
    sim_fa = Sim(level="L2", road=Road.flat(), v0=15.0, ref_point="front_axle")
    for _ in range(int(3.0 / sim_cg.dt)):
        for sim in (sim_cg, sim_ra, sim_fa):
            sim.set_input(steer=0.04, throttle=0.1)
            sim.run_core_dt()
    cg = sim_cg.state(); ra = sim_ra.state(); fa = sim_fa.state()
    b = sim_ra._vp.cg_to_rear; a = sim_fa._vp.cg_to_front
    yaw = cg["yaw"]
    c, s = math.cos(yaw), math.sin(yaw)
    assert abs(ra["x"] - (cg["x"] - c * b)) < 1e-9, "rear_axle x"
    assert abs(ra["y"] - (cg["y"] - s * b)) < 1e-9, "rear_axle y"
    assert abs(fa["x"] - (cg["x"] + c * a)) < 1e-9, "front_axle x"
    # user-defined ref_point
    sim_u = Sim(level="L2", road=Road.flat(), v0=15.0, ref_point=[1.0, 0.5])
    assert sim_u._ref == [1.0, 0.5]


if __name__ == "__main__":
    test_throttle_then_brake()
    test_step_steer_yaws()
    test_cmd_object_accepted()
    test_mount_pose_shifts_gnss()
    test_log_metrics_csv()
    test_state_beta()
    test_log_extra_in_csv()
    test_reset_reuses_plant()
    test_register_metric()
    test_plot_comparison()
    test_ref_point_position()
    print("OK test_experiment_api")
