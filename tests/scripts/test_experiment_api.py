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

import vdsim_lab
from vdsim_lab import Sim, Road, Sensors, Experiment


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


# --- Q20 (i): what actually discriminates L3 from L4 ------------------------
# KinematicFourteenDOFDynamics (core/src/fourteen_dof_dynamics.cpp) overrides
# level() and nothing else, and attach_front/rear_kinematics dynamic_cast to
# the shared base.  So the level label is not the discriminator -- the attach
# is.  These tests keep that fact from being rediscovered as a surprise, and
# are why a trace manifest needs kinematics provenance next to model_level.
HARDPOINTS = {"front": "mp_front_sedan", "rear": "ta_rear_sedan"}


def _tape(k):
    return 0.03 * math.sin(0.02 * k), 0.15


def _l3_l4_rows(kin, n=300):
    """Run one identical input tape on L3 and L4 through Sim; return both tables."""
    out = {}
    for level in ("L3", "L4"):
        sim = Sim(level=level, road=Road.flat(mu=1.0), v0=15.0, kinematics=kin)
        for k in range(n):
            steer, throttle = _tape(k)
            sim.set_input(steer=steer, throttle=throttle)
            sim.run_core_dt()
        out[level] = [list(r) for r in sim.rows]
    return out


def _bare_core_rows(level, n=300, dt=0.005):
    """Same tape on a raw core session with no attach -- below the seam.

    Sim refuses a bare L4 (the H1 guard), so the "label carries no physics"
    fact is measured one layer down, on the session factory _build_session
    itself calls.  Row layout is vdsim_lab's, so it compares with Sim rows.
    """
    import vdsim
    veh, tire = vdsim_lab._as_vehicle("sedan"), vdsim_lab._as_tire("default_pacejka")
    sess = Road.flat(mu=1.0)._session(veh.vp, tire.tp, level, dt, vdsim.SensorParams())
    sess.reset(vdsim.make_init_state(veh.vp, tire.tp, x=0.0, y=0.0, yaw=0.0, v=15.0))
    rows = []
    for k in range(n):
        steer, throttle = _tape(k)
        c = vdsim.CmdL4()
        c.steer_angle_wheel = steer
        c.throttle = throttle
        c.gear = 1
        sess.set_input(c)
        sess.tick(dt)
        rows.append(list(vdsim_lab._make_row(sess.output())))
    return rows


def test_level_label_alone_carries_no_suspension_physics():
    bare = {lv: _bare_core_rows(lv) for lv in ("L3", "L4")}
    assert bare["L3"] == bare["L4"], "bare L4 must be bit-identical to bare L3"
    via_sim = Sim(level="L3", road=Road.flat(mu=1.0), v0=15.0)
    for k in range(300):
        steer, throttle = _tape(k)
        via_sim.set_input(steer=steer, throttle=throttle)
        via_sim.run_core_dt()
    assert [list(r) for r in via_sim.rows] == bare["L3"],         "the raw-session probe must reproduce Sim exactly, or it proves nothing"
    kin = _l3_l4_rows(HARDPOINTS)
    assert kin["L3"] == kin["L4"], \
        "hardpoints attach to both levels -- level() is a label, not physics"
    return bare, kin


def test_hardpoints_are_the_real_discriminator():
    bare, kin = test_level_label_alone_carries_no_suspension_physics()
    d = max(abs(a[i] - b[i])
            for a, b in zip(bare["L4"], kin["L4"]) for i in range(len(a)))
    assert d > 1e-9, \
        f"attaching hardpoints must change the trajectory (max |delta| = {d:.3e})"


def test_bare_l4_is_refused():
    """H1: L4 with no hardpoints is L3 under a false label -- refuse it."""
    for build in (lambda: Sim(level="L4", road=Road.flat()),
                  lambda: Experiment(level="L4").run(0.05)):
        try:
            build()
        except ValueError as e:
            msg = str(e)
            print("H1 refusal: ValueError: %s" % msg)
            assert "kin=" in msg and "level='L3'" in msg, f"unexpected message: {msg}"
        else:
            raise AssertionError("bare L4 must be refused, not run as L3")
    # With hardpoints both labels are allowed, and L3 + hardpoints stays legal.
    Sim(level="L4", road=Road.flat(), kinematics=HARDPOINTS)
    Sim(level="L3", road=Road.flat(), kinematics=HARDPOINTS)


def test_plant_bare_l4_is_refused():
    """Q20 (A): VDSimPlant cannot attach hardpoints, so its L4 is refused too."""
    from vdsim_plant import VDSimPlant
    try:
        Sim(level="L4", road=Road.flat())
    except ValueError as e:
        lab_lead = str(e).split(";")[0]
    try:
        VDSimPlant(level="L4")
    except ValueError as e:
        msg = str(e)
        print("plant L4 refusal: ValueError: %s" % msg)
        assert msg.split(";")[0] == lab_lead, \
            f"plant and _build_session must state the same reason: {msg!r} vs {lab_lead!r}"
        assert "level='L3'" in msg, f"unexpected message: {msg}"
    else:
        raise AssertionError("VDSimPlant(level='L4') must be refused, not run as L3")
    assert VDSimPlant(level="L3").level == "L3"


def test_trace_states_the_attach():
    """Q20 (iii): the manifest records what the attach returned."""
    import vdsim_trace as vt
    with tempfile.TemporaryDirectory() as td:
        got = {}
        for name, kin in (("bare", None), ("kin", HARDPOINTS)):
            exp = Experiment(level="L3")
            if kin:
                exp.kinematics(**kin)
            p = Path(td) / (name + ".vdtrace")
            exp.enable_trace(p, seed=0, run_id=name)
            exp.run(0.2)
            exp.finalize_trace()
            with vt.TraceReader(p) as tr:
                got[name] = (tr.manifest["schema_version"], tr.kinematics_attached)
        assert got["bare"] == (vt.SCHEMA_VERSION, False), got
        assert got["kin"] == (vt.SCHEMA_VERSION, True), got
        exp = Experiment(level="L3")
        exp.enable_trace(Path(td) / "never.vdtrace")
        try:
            exp.finalize_trace()
        except RuntimeError:
            pass
        else:
            raise AssertionError("finalize before run must not invent an attach state")


def test_hardpoints_refused_below_l3():
    try:
        Sim(level="L2", road=Road.flat(), kinematics=HARDPOINTS)
    except RuntimeError as e:
        assert "L3/L4" in str(e), f"unexpected message: {e}"
    else:
        raise AssertionError("L2 must refuse hardpoints, not silently ignore them")


def test_missing_hardpoint_file_is_an_error():
    try:
        Sim(level="L3", road=Road.flat(), kinematics={"front": "no_such_kin"})
    except FileNotFoundError:
        pass
    else:
        raise AssertionError("a missing hardpoint YAML must raise, not skip the attach")


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
    test_level_label_alone_carries_no_suspension_physics()
    test_hardpoints_are_the_real_discriminator()
    test_bare_l4_is_refused()
    test_plant_bare_l4_is_refused()
    test_trace_states_the_attach()
    test_hardpoints_refused_below_l3()
    test_missing_hardpoint_file_is_an_error()
    print("OK test_experiment_api")
