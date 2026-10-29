#!/usr/bin/env python3
"""Level ids at every Python entry point (work order 44, 0.8.1).

* E1 -- each entry point builds K, L0, L1..L5 and raises ``ValueError`` for
  ``"L9"``, ``"l3"`` and ``""`` with the accepted set in the message. Two
  entry points refuse ``"L5"`` by name, with a message distinct from the
  unknown-id one: ``linearize`` (not validated on the free-3D state) and
  ``make_vec_session`` (RL on L5 not validated).
* E2 -- ``vdsim.Level.L5_Stunt`` exists and every L5 session reports it.
* E4 -- ``make_sim_session("L5")`` and ``make_direct_control_session`` with
  level ``"L5"`` reproduce, bit for bit, the C++ reference run of the same
  model built directly (``vdsim_l5_ref_dump``).
* E5 -- 3 s step steer / drive on L5 stays finite for generic_sedan and
  fs_aero_example through the flat, heightmap and PSD sessions, ``Sim`` and
  ``VDSimPlant``; ``solver_substep_clamp_count()`` stays 0 in E4/E5.
* K3 -- ``VDSimPlant`` and ``vdsim_lab`` record the L5 solver settings they
  used; other levels record nothing new.
* K1 -- ``vdsim_realtime`` refuses a scene with an unknown level at start-up.

usage: test_level_parse.py <vdsim_l5_ref_dump binary>
"""
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim  # noqa: E402
import vdsim_lab  # noqa: E402
import vdsim_plant  # noqa: E402

ACCEPTED = ("K", "L0", "L1", "L2", "L3", "L4", "L5")
UNKNOWN = ("L9", "l3", "")
UNKNOWN_MSG = "expected one of K, L0, L1, L2, L3, L4, L5"
LEVEL_OF = {"K": vdsim.Level.Lk_Kinematic, "L0": vdsim.Level.Lk_Kinematic,
            "L1": vdsim.Level.L1_Bicycle, "L2": vdsim.Level.L2_SevenDOF,
            "L3": vdsim.Level.L3_FourteenDOF, "L4": vdsim.Level.L4_Kinematic,
            "L5": vdsim.Level.L5_Stunt}
VEHICLES = ("generic_sedan", "fs_aero_example")

_failures = []


def check(ok, what):
    print(("PASS " if ok else "FAIL ") + what)
    if not ok:
        _failures.append(what)


def _vp(name="generic_sedan"):
    return vdsim.VehicleParams.from_yaml(str(ROOT / "configs" / "vehicles" / f"{name}.yaml"))


def _tp():
    return vdsim.TireParams.from_yaml(str(ROOT / "configs" / "parts" / "tire" / "generic_pacejka.yaml"))


def _entry_points(vp, tp):
    """name -> callable(level) returning a session (or a linearization)."""
    hm = np.zeros((8, 8))
    return {
        "make_sim_session": lambda lv: vdsim.make_sim_session(vp, tp, lv),
        "make_vec_session": lambda lv: vdsim.make_vec_session(2, vp, tp, lv),
        "make_sim_session_heightmap": lambda lv: vdsim.make_sim_session_heightmap(
            vp, tp, lv, hm, x0=-20.0, y0=-20.0, dx=5.0, dy=5.0),
        "make_sim_session_psd": lambda lv: vdsim.make_sim_session_psd(vp, tp, lv, gd_n0=4e-6),
        "linearize": lambda lv: vdsim.linearize(vp, tp, lv, [0, 0, 0, 10, 0, 0], [0, 0], 0.01, 1.0),
        "make_direct_control_session": lambda lv: vdsim.make_direct_control_session(
            vp, vdsim.TireSetup(tp), vdsim.SolverParams(), _dc_opts(lv)),
    }


def _dc_opts(level, dt=5e-4):
    o = vdsim.DirectControlSessionOptions()
    o.level = level
    o.nominal_dt = dt
    return o


#: entry point -> exact refusal message for "L5" (others accept L5)
L5_REFUSED = {
    "linearize": "L5 linearization is not supported in 0.8.1",
    "make_vec_session": "L5 is not supported in VecSession in 0.8.1",
}


def test_e1_e2_entry_points():
    vp, tp = _vp(), _tp()
    check(hasattr(vdsim.Level, "L5_Stunt"), "E2 vdsim.Level.L5_Stunt is exposed")
    for name, make in _entry_points(vp, tp).items():
        for lv in ACCEPTED:
            if lv == "L5" and name in L5_REFUSED:
                try:
                    make(lv)
                    check(False, f"E1 {name}('L5') refused by name")
                except ValueError as e:
                    msg = str(e)
                    check(msg.startswith(L5_REFUSED[name]) and UNKNOWN_MSG not in msg,
                          f"E1 {name}('L5') refused by name, distinct message: {msg!r}")
                continue
            try:
                obj = make(lv)
            except Exception as e:                       # noqa: BLE001
                check(False, f"E1 {name}({lv!r}) builds ({e})")
                continue
            if name == "linearize":
                check(np.isfinite(np.asarray(obj["A"], dtype=float)).all(),
                      f"E1 {name}({lv!r}) builds a finite A")
            elif name == "make_vec_session":
                check(int(obj.num_envs) == 2, f"E1 {name}({lv!r}) builds 2 envs")
            else:
                got = obj.dynamics().level()
                check(got == LEVEL_OF[lv], f"E1/E2 {name}({lv!r}).level() = {got}")
        for lv in UNKNOWN:
            try:
                make(lv)
                check(False, f"E1 {name}({lv!r}) raises ValueError")
            except ValueError as e:
                check(UNKNOWN_MSG in str(e) and f'"{lv}"' in str(e),
                      f"E1 {name}({lv!r}) raises ValueError naming the accepted set")
    # the C++ factory itself, through the binding
    for lv in ACCEPTED:
        check(vdsim.create_dynamics_for_level(lv).level() == LEVEL_OF[lv],
              f"E1 create_dynamics_for_level({lv!r})")
    # high-level entry points
    for lv in UNKNOWN:
        try:
            vdsim_lab.Sim(level=lv)
            check(False, f"E1 vdsim_lab.Sim(level={lv!r}) raises ValueError")
        except ValueError as e:
            check(UNKNOWN_MSG in str(e), f"E1 vdsim_lab.Sim(level={lv!r}) raises ValueError")
        try:
            vdsim_plant.VDSimPlant(level=lv)
            check(False, f"E1 VDSimPlant(level={lv!r}) raises ValueError")
        except ValueError:
            check(True, f"E1 VDSimPlant(level={lv!r}) raises ValueError")
    check(vdsim_lab.Sim(level="L5").sess.dynamics().level() == vdsim.Level.L5_Stunt,
          "E2 vdsim_lab.Sim(level='L5') is L5_Stunt")
    check(vdsim_plant.VDSimPlant(level="L5")._dyn.level() == vdsim.Level.L5_Stunt,
          "E2 VDSimPlant(level='L5') is L5_Stunt")


def _row(o):
    st = o.state
    p, v, w = st.position, st.velocity, st.angular_velocity
    vals = [p[0], p[1], p[2], v[0], v[1], v[2], w[0], w[1], w[2],
            o.roll, o.pitch, st.yaw(), *o.Fz, o.ax, o.ay, *st.wheel_spin]
    return " ".join("%.17g" % float(x) for x in vals)


def _py_session_lines(veh):
    vp, tp = _vp(veh), _tp()
    dt = 0.005
    sp = vdsim_plant.level_solver_params("L5", dt)
    s = vdsim.make_sim_session(vp, tp, "L5", nominal_dt=dt, solver=sp)
    s.reset(vdsim.make_init_state(vp, tp, v=10.0))
    out = []
    for k in range(600):
        u = vdsim.CmdL4()
        u.throttle = 0.3 if k < 100 else 0.2
        u.steer_angle_wheel = 0.0 if k < 100 else 0.05
        s.set_input(u)
        s.tick(dt)
        out.append(_row(s.output()))
    return out


def _py_direct_lines(veh):
    vp, tp = _vp(veh), _tp()
    dt = 5e-4
    sp = vdsim.SolverParams()
    sp.integrator = vdsim.Integrator.RK4
    sp.max_substep_dt = min(dt, sp.max_substep_dt)
    sp = vdsim_plant.level_solver_params("L5", dt, sp)
    s = vdsim.make_direct_control_session(vp, vdsim.TireSetup(tp), sp, _dc_opts("L5", dt))
    s.reset(vdsim.make_init_state(vp, tp, v=10.0))
    out = []
    for k in range(6000):
        u = vdsim.CmdL1()
        tq = 150.0 if k < 1000 else 100.0
        u.motor_torque = [tq] * 4
        u.brake_torque = [0.0] * 4
        u.steer_angle_wheel = 0.0 if k < 1000 else 0.05
        s.set_input(u)
        s.tick(dt)
        if k % 10 == 9:
            out.append(_row(s.output()))
    return out


def test_e4_python_matches_cpp(ref_bin):
    vdsim.reset_solver_substep_clamp_count()
    with tempfile.TemporaryDirectory() as tmp:
        r = subprocess.run([ref_bin, str(ROOT), tmp], capture_output=True, text=True, timeout=600)
        check(r.returncode == 0, f"E4 C++ reference dump rc={r.returncode} {r.stderr[-300:]}")
        check("solver_substep_clamp_count 0" in r.stdout,
              f"E4 C++ reference run clamp count 0 ({r.stdout.strip()})")
        for veh in VEHICLES:
            for kind, fn in (("session", _py_session_lines), ("direct", _py_direct_lines)):
                ref = (Path(tmp) / f"{kind}_{veh}.txt").read_text().splitlines()
                got = fn(veh)
                same = ref == got
                first = next((i for i, (a, b) in enumerate(zip(ref, got)) if a != b), None)
                check(same and len(ref) > 0,
                      f"E4 {kind} {veh}: {len(got)} rows bit-identical to C++"
                      + ("" if same else f" (first diff row {first}, len {len(ref)}/{len(got)})"))
    check(vdsim.solver_substep_clamp_count() == 0,
          f"E4 Python runs clamp count {vdsim.solver_substep_clamp_count()} == 0")


def _finite_rows(step, n):
    bad = 0
    for k in range(n):
        o = step(k)
        st = o.state
        vals = [*st.position, *st.velocity, *st.angular_velocity, o.roll, o.pitch, *o.Fz, o.ax, o.ay]
        bad += sum(0 if math.isfinite(float(x)) else 1 for x in vals)
    return bad


def _drive_session(s, dt, n):
    def step(k):
        u = vdsim.CmdL4()
        u.throttle = 0.3 if k < n // 6 else 0.2
        u.steer_angle_wheel = 0.0 if k < n // 6 else 0.05
        s.set_input(u)
        s.tick(dt)
        return s.output()
    return _finite_rows(step, n)


def test_e5_finite():
    vdsim.reset_solver_substep_clamp_count()
    tp = _tp()
    dt = 0.005
    hm = np.zeros((8, 8))
    for veh in VEHICLES:
        vp = _vp(veh)
        sp = lambda: vdsim_plant.level_solver_params("L5", dt)  # noqa: E731
        builds = {
            "flat": lambda: vdsim.make_sim_session(vp, tp, "L5", nominal_dt=dt, solver=sp()),
            "heightmap": lambda: vdsim.make_sim_session_heightmap(
                vp, tp, "L5", hm, x0=-20.0, y0=-20.0, dx=20.0, dy=5.0,
                nominal_dt=dt, solver=sp()),
            "psd": lambda: vdsim.make_sim_session_psd(vp, tp, "L5", gd_n0=4e-6,
                                                      nominal_dt=dt, solver=sp()),
        }
        for road, build in builds.items():
            s = build()
            s.reset(vdsim.make_init_state(vp, tp, v=10.0))
            bad = _drive_session(s, dt, int(3.0 / dt))
            check(bad == 0, f"E5 {veh} L5 {road} 3 s finite (non-finite values {bad})")
        sim = vdsim_lab.Sim(vehicle=vdsim_lab.Vehicle.from_yaml(
            str(ROOT / "configs" / "vehicles" / f"{veh}.yaml")), level="L5", v0=10.0)
        bad = _drive_session(sim.sess, sim.dt, int(3.0 / sim.dt))
        check(bad == 0, f"E5 {veh} vdsim_lab.Sim L5 3 s finite (non-finite values {bad})")
        plant = vdsim_plant.VDSimPlant(config=f"{veh}.yaml", level="L5")
        plant.reset([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])
        fin = True
        for k in range(int(3.0 / plant.control_dt)):
            obs = plant.step([0.0 if k < 10 else 0.05, 800.0])
            fin &= all(math.isfinite(float(v)) for v in obs.values()
                       if isinstance(v, (int, float)))
        check(fin, f"E5 {veh} VDSimPlant L5 3 s finite")
    check(vdsim.solver_substep_clamp_count() == 0,
          f"E5 clamp count {vdsim.solver_substep_clamp_count()} == 0")


def test_k3_solver_record():
    rec = vdsim_plant.VDSimPlant(level="L5").provenance().get("solver")
    check(rec is not None and rec["stunt_physics"] is True
          and rec["max_substep_dt"] <= 2e-4 and rec["max_substeps"] >= 3,
          f"K3 VDSimPlant L5 provenance records the solver used: {rec}")
    check("solver" not in vdsim_plant.VDSimPlant(level="L2").provenance(),
          "K3 VDSimPlant L2 provenance unchanged (no solver key)")
    info = vdsim_lab.Sim(level="L5")._kin_info
    check(info.get("solver") == {"max_substep_dt": 2e-4, "max_substeps": 25,
                                 "stunt_physics": True},
          f"K3 vdsim_lab L5 records the solver used: {info.get('solver')}")
    check("solver" not in vdsim_lab.Sim(level="L2")._kin_info,
          "K3 vdsim_lab L2 records nothing new")
    sp = vdsim.SolverParams()
    check(vdsim_plant.level_solver_params("L3", 0.005, sp) is sp
          and sp.max_substep_dt == 1e-3 and not sp.stunt_physics,
          "K3 level_solver_params leaves other levels untouched")


def test_k1_realtime_refuses_unknown_level():
    build = Path(os.environ.get("VDSIM_BUILD_DIR", ROOT / "build"))
    exe = build / "bin" / "vdsim_realtime"
    if not exe.exists():
        check(False, f"K1 vdsim_realtime present at {exe}")
        return
    src = (ROOT / "configs" / "scenes" / "two_vehicle_race.yaml").read_text()
    check("level: L2" in src, "K1 template scene has a level line")
    # inside the build tree, so the scene loader still finds the catalog root
    # by walking up from the scene file
    with tempfile.TemporaryDirectory(dir=str(build)) as tmp:
        scene = Path(tmp) / "bad_level.yaml"
        scene.write_text(src.replace("level: L2", "level: L9", 1))
        try:
            r = subprocess.run([str(exe), f"--scene={scene}"],
                               capture_output=True, text=True, timeout=60, cwd=str(ROOT))
        except subprocess.TimeoutExpired:
            check(False, "K1 vdsim_realtime refuses level L9 at start-up (still running after 60 s)")
            return
        check(r.returncode != 0 and 'unknown dynamics level "L9"' in r.stderr,
              f"K1 vdsim_realtime refuses level L9 at start-up (rc={r.returncode}, "
              f"stderr tail {r.stderr.strip()[-160:]!r})")


def main():
    if len(sys.argv) < 2:
        print("usage: test_level_parse.py <vdsim_l5_ref_dump>")
        return 2
    test_e1_e2_entry_points()
    test_e4_python_matches_cpp(sys.argv[1])
    test_e5_finite()
    test_k3_solver_record()
    test_k1_realtime_refuses_unknown_level()
    if _failures:
        print(f"\n{len(_failures)} FAILED")
        return 1
    print("\nALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
