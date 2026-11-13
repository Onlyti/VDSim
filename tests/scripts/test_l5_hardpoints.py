"""48c Python side (no pytest): L5 hardpoint attach, camber/toe/hardpoint output,
rack_per_rad key, and the .vdtrace channels the lab path records.

Gates fixed before the implementation commit:
  K8  SimOutput camber/toe/hardpoints read in Python == the C++ values (read through
      the free functions on the same dynamics), bit for bit, every tick; L5 hardpoint
      attach works from Python.
  K3  (Python) static coordinates == YAML + axle offset (right side mirrored), 1e-9 m.
  C4  vdsim.attach_l5_hardpoints; attach_*_kinematics_from_yaml keep returning False
      on L5; vdsim_lab.attach_kinematics delegates L5 to the new function.
  D3  trace: invalid wheels are NaN; hardpoint channels only for attached corners;
      manifest ``hardpoints`` block (frame, axle offsets, wheel-centre dy, CG, names).
  C3  rack_per_rad: default 0.08, YAML round trip, ValueError for a bad value.
"""
from pathlib import Path
import math
import struct
import sys
import tempfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim  # noqa: E402

SEDAN = ROOT / "configs/vehicles/generic_sedan.yaml"
KIN = ROOT / "configs/parts/susp_kinematics/kin"
DT = 1e-3
TOL_STATIC = 1e-9

NAMES = {
    "macpherson": ["lca.chassis_front", "lca.chassis_rear", "lca.knuckle", "strut.top",
                   "strut.bottom", "tie_rod.rack", "tie_rod.knuckle", "wheel.center"],
    "double_wishbone": ["lca.chassis_front", "lca.chassis_rear", "lca.knuckle",
                        "uca.chassis_front", "uca.chassis_rear", "uca.knuckle",
                        "tie_rod.rack", "tie_rod.knuckle", "spring_damper.chassis",
                        "spring_damper.lca", "wheel.center"],
    "trailing_arm": ["arm_pivot.chassis_inboard", "arm_pivot.chassis_outboard",
                     "wheel.center"],
    "five_link": ["links.upper_fore.chassis", "links.upper_fore.knuckle",
                  "links.upper_aft.chassis", "links.upper_aft.knuckle",
                  "links.lower_fore.chassis", "links.lower_fore.knuckle",
                  "links.lower_aft.chassis", "links.lower_aft.knuckle",
                  "links.toe_link.chassis", "links.toe_link.knuckle", "wheel.center"],
}
PAIRS = (("mp_front_sedan", "ta_rear_sedan"), ("dw_front_sports", "5link_rear_sports"))
# Changed after measurement (PO verdict 2026-11-07, 48c K4 option 1): the five-link
# corner exports no points (layout names empty, excluded reason set).
EXCLUDED = {"five_link": "five_link driven-coordinate least-squares, see CHANGELOG"}


def exported(typ):
    return [] if typ in EXCLUDED else NAMES[typ]
COUNTS = {"cpp_like": 0}


def check(cond, label):
    if not cond:
        raise SystemExit("FAIL " + label)
    COUNTS["cpp_like"] += 1


def load_kin(stem):
    root = yaml.safe_load((KIN / (stem + ".yaml")).read_text())
    pts = {}
    for name in NAMES[root["type"]]:
        node = root
        for part in name.split("."):
            node = node[part]
        pts[name] = np.array([float(c) for c in node])
    return root["type"], pts


def l5_solver(strut=True):
    sp = vdsim.SolverParams()
    sp.stunt_physics = True
    sp.max_substep_dt = 2e-4
    sp.max_substeps = 16
    sp.l5_spatial_suspension = strut
    return sp


def l5_session(vp, strut=True):
    return vdsim.make_sim_session(vp, vdsim.TireParams(), level="L5", nominal_dt=DT,
                                  solver=l5_solver(strut))


def init_state(vp, v=0.0):
    st = vdsim.State()
    st.position = np.array([0.0, 0.0, vp.cg_height])
    st.velocity = np.array([v, 0.0, 0.0])
    w = v / vp.wheel_radius_nominal
    st.wheel_spin = [w, w, w, w]
    return st


def bits(values):
    flat = []
    for v in values:
        if isinstance(v, (list, tuple)):
            flat.extend(float(c) for c in v)
        else:
            flat.append(float(v))
    return struct.pack("<%dd" % len(flat), *flat)


def hp_bits(hp):
    return b"".join(bits(corner) for corner in hp)


def expected_static(vp, front_stem, rear_stem):
    out = []
    for i in range(4):
        front, right = i < 2, i in (1, 3)
        typ, pts = load_kin(front_stem if front else rear_stem)
        wx = pts["wheel.center"][0]
        dx = (0.0 - wx) if front else (-vp.wheelbase - wx)
        out.append([np.array([p[0] + dx, -p[1] if right else p[1], p[2]])
                    for p in (pts[n] for n in exported(typ))])
    return out


def assert_static(vp, hp, valid, pair, label):
    exp = expected_static(vp, *pair)
    for i in range(4):
        check(valid[i], "%s valid[%d]" % (label, i))
        check(len(hp[i]) == len(exp[i]), "%s count[%d]" % (label, i))
        for j, (got, want) in enumerate(zip(hp[i], exp[i])):
            err = float(np.linalg.norm(np.array(got) - want))
            check(err <= TOL_STATIC, "%s K3 wheel %d point %d err %.3e" % (label, i, j, err))


def test_attach_and_k3():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    for pair in PAIRS:
        s = l5_session(vp)
        dyn = s.dynamics()
        # The L3/L4 attach keeps refusing the L5 plant (unchanged contract).
        check(vdsim.attach_front_kinematics_from_yaml(dyn, str(KIN / (pair[0] + ".yaml")))
              is False, "L4 attach on L5 stays False")
        check(vdsim.attach_l5_hardpoints(dyn, front_yaml=str(KIN / (pair[0] + ".yaml")),
                                         rear_yaml=str(KIN / (pair[1] + ".yaml"))) is True,
              "attach_l5_hardpoints True")
        lay = vdsim.free_3d_hardpoint_layout(dyn)
        tf, tr_ = load_kin(pair[0])[0], load_kin(pair[1])[0]
        check(list(lay.names[0]) == exported(tf), "layout names front")
        check(list(lay.names[1]) == exported(tr_), "layout names rear")
        check(lay.excluded[0] == EXCLUDED.get(tf, ""), "layout excluded front")
        check(lay.excluded[1] == EXCLUDED.get(tr_, ""), "layout excluded rear")
        assert_static(vp, vdsim.free_3d_wheel_hardpoints(dyn),
                      vdsim.free_3d_wheel_kin_valid(dyn), pair, pair[0] + " attach")
        o = s.output()
        assert_static(vp, o.hardpoints, o.wheel_kin_valid, pair, pair[0] + " attach/output")
        s.reset_settled(init_state(vp))
        for _ in range(100):
            c = vdsim.CmdL4()
            c.throttle = 0.3
            c.steer_angle_wheel = 0.04
            s.set_input(c)
            s.tick(DT)
        s.reset(init_state(vp))
        o = s.output()
        assert_static(vp, o.hardpoints, o.wheel_kin_valid, pair, pair[0] + " reset/output")


def test_k8_bits_every_tick():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    for pair in PAIRS:
        s = l5_session(vp)
        dyn = s.dynamics()
        check(vdsim.attach_l5_hardpoints(dyn, front_yaml=str(KIN / (pair[0] + ".yaml")),
                                         rear_yaml=str(KIN / (pair[1] + ".yaml"))),
              "attach")
        s.reset_settled(init_state(vp, 10.0))
        moved = 0.0
        for k in range(600):
            c = vdsim.CmdL4()
            c.throttle = 0.25 if k < 300 else 0.0
            c.steer_angle_wheel = 0.0 if k < 100 else 0.05
            s.set_input(c)
            s.tick(DT)
            o = s.output()
            check(bits(o.wheel_camber) == bits(vdsim.free_3d_wheel_camber(dyn)),
                  "K8 camber tick %d" % k)
            check(bits(o.wheel_toe) == bits(vdsim.free_3d_wheel_toe(dyn)), "K8 toe tick %d" % k)
            check(hp_bits(o.hardpoints) == hp_bits(vdsim.free_3d_wheel_hardpoints(dyn)),
                  "K8 hardpoints tick %d" % k)
            check(list(o.wheel_kin_valid) == [True] * 4, "K8 valid tick %d" % k)
            moved = max(moved, max(abs(t) for t in o.wheel_toe))
        check(moved > 0.0, "K8 run produced toe")


def test_invalid_outputs():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    s = vdsim.make_sim_session(vp, vdsim.TireParams(), level="L4", nominal_dt=DT)
    check(vdsim.attach_front_kinematics_from_yaml(
        s.dynamics(), str(KIN / "dw_front_sports.yaml")) is True, "L4 attach")
    s.reset(init_state(vp))
    s.set_input(vdsim.CmdL4())
    s.tick(DT)
    o = s.output()
    check(list(o.wheel_kin_valid) == [False] * 4, "D2 L4 valid false")
    check(list(o.wheel_camber) == [0.0] * 4 and list(o.wheel_toe) == [0.0] * 4,
          "D2 L4 zeros")
    check(all(len(c) == 0 for c in o.hardpoints), "D2 L4 no points")
    check(vdsim.attach_l5_hardpoints(s.dynamics(),
                                     front_yaml=str(KIN / "dw_front_sports.yaml")) is False,
          "attach_l5_hardpoints refuses L4")


def test_rack_per_rad():
    vp = vdsim.VehicleParams()
    check(vp.rack_per_rad == 0.08, "C3 default")
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.rack_per_rad = 0.05
    with tempfile.TemporaryDirectory() as d:
        p = str(Path(d) / "v.yaml")
        vp.to_yaml(p)
        check(vdsim.VehicleParams.from_yaml(p).rack_per_rad == 0.05, "C3 round trip")
        bad = Path(d) / "bad.yaml"
        bad.write_text(SEDAN.read_text() + "\nrack_per_rad: -0.1\n")
        try:
            vdsim.VehicleParams.from_yaml(str(bad))
        except ValueError:
            COUNTS["cpp_like"] += 1
        else:
            raise SystemExit("FAIL C3 yaml ValueError")
    vp.rack_per_rad = 0.0
    try:
        l5_session(vp)
    except ValueError:
        COUNTS["cpp_like"] += 1
    else:
        raise SystemExit("FAIL C3 session ValueError")


def test_lab_trace():
    import vdsim_lab
    import vdsim_trace
    with tempfile.TemporaryDirectory() as d:
        # Front-only attach: rear camber/toe NaN, no rear hardpoint channels.
        path = Path(d) / "l5_front.vdtrace"
        exp = vdsim_lab.Experiment(level="L5", dt=DT).kinematics("mp_front_sedan", None)
        exp.maneuver(vdsim_lab.Maneuver.constant_speed(8.0))
        exp.enable_trace(path, decimation=10, seed=0)
        exp.run(duration=0.3)
        exp.finalize_trace()
        with vdsim_trace.TraceReader(path) as tr:
            m = tr.manifest
            hp = m["hardpoints"]
            check(hp["unit"] == "m" and hp["invalid_wheel"] == "NaN", "D3 manifest unit/NaN")
            check(hp["axle_offset_x"][0] == 0.0, "D1 front offset 0")
            check(abs(hp["wheel_center_dy"][0] - (0.775 - 0.775)) < 1e-12, "dy front")
            check(set(hp["corners"]) == {"fl", "fr"}, "corners attached only")
            check(hp["corners"]["fl"]["type"] == "macpherson", "corner type")
            cg = hp["cg"]
            check(cg["position_m"][0] == -tr.geometry["cg_to_front_m"], "cg x")
            check("cg_height_ref" in cg, "cg_height_ref recorded")
            cam = tr.channel("wheel_camber")
            check(cam.shape[1] == 4, "camber shape")
            check(np.all(np.isnan(cam[:, 2:])) and np.all(np.isfinite(cam[:, :2])),
                  "D3 rear NaN, front finite")
            names = tr.channel_names() if hasattr(tr, "channel_names") else tr.channels
            check(any(n.startswith("hp_fl_") for n in names), "hp_fl channels")
            check(not any(n.startswith("hp_rl_") for n in names), "no hp_rl channels")
            for key, ch in hp["corners"]["fl"]["points"].items():
                check(tr.channel(ch).shape[1] == 3, "hp channel shape " + key)
            wc = tr.channel(hp["corners"]["fl"]["points"]["wheel.center"])
            check(np.all(np.isfinite(wc)), "hp finite")


def test_lab_trace_five_link():
    import vdsim_lab
    import vdsim_trace
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "l5_5link.vdtrace"
        exp = vdsim_lab.Experiment(level="L5", dt=DT).kinematics(
            "dw_front_sports", "5link_rear_sports")
        exp.maneuver(vdsim_lab.Maneuver.constant_speed(8.0))
        exp.enable_trace(path, decimation=10, seed=0)
        exp.run(duration=0.3)
        exp.finalize_trace()
        with vdsim_trace.TraceReader(path) as tr:
            hp = tr.manifest["hardpoints"]
            check(set(hp["corners"]) == {"fl", "fr", "rl", "rr"}, "5link corners listed")
            for w in ("rl", "rr"):
                check(hp["corners"][w]["points"] == {}, "5link %s no points" % w)
                check(hp["corners"][w]["excluded"] == EXCLUDED["five_link"],
                      "5link %s excluded reason" % w)
            check("excluded" not in hp["corners"]["fl"], "front not excluded")
            names = tr.channel_names() if hasattr(tr, "channel_names") else tr.channels
            check(not any(n.startswith(("hp_rl_", "hp_rr_")) for n in names),
                  "no five-link hp channels")
            check(any(n.startswith("hp_fl_") for n in names), "front hp channels")
            cam = tr.channel("wheel_camber")
            check(np.all(np.isfinite(cam)), "five-link camber still recorded")


test_attach_and_k3()
test_k8_bits_every_tick()
test_invalid_outputs()
test_rack_per_rad()
test_lab_trace()
test_lab_trace_five_link()
print("l5_hardpoints python checks OK (%d assertions)" % COUNTS["cpp_like"])
