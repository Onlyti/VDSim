"""Work order 52, Python side: axle tyres (R4) and rotor keys (no pytest).

Fixed before the implementation commit (definition: tests/fixtures/axle_rotor_52/DEFINITION.md).

T2  The four Python session factories (make_sim_session L1-L5, make_vec_session,
    make_sim_session_heightmap, make_sim_session_psd) given TireSetup(tp, tp) run bit for
    bit like the same factory given tp (2000 ticks; L5 both paths).
T3  Front mu 1.0 / rear mu 1.3 with different lateral stiffness through
    make_sim_session(tire=TireSetup(front, rear)), L1-L5 (L5 both paths): at every 50th of
    400 ticks, each wheel's live tyre model evaluated on the input built from that tick's
    output (Fz, slip ratio, slip angle, wheel-forward speed) equals the tyre model of the
    same wheel in a reference session built through the existing single-tyre path with
    that axle's parameters, relative error <= 1e-12 on Fx, Fy, Mz. Discrimination: the
    rear wheels must differ from the front-parameter reference by more than 1e-3
    (relative) on at least one sample, otherwise not determinable = failure.
T4  Lab: Experiment.tire(front, rear) / Sim(..., tire_rear=...) / from_config tire_rear;
    tire_rear without tire raises ValueError; tire_rear equal to tire runs bit for bit
    like the single tyre. A non-tyre object as tire raises TypeError.
Rotor keys: rotor_inertia / rotor_ratio properties alias engine_rotational_inertia /
    final_drive_ratio; to_yaml keeps the alias names; conflicts between written names,
    rotor_ratio <= 0, rotor_inertia < 0, an unknown rotor_axis and powertrain v2 with
    rotor_inertia / rotor_ratio raise ValueError; L1-L4 warn once per process (child process).
"""
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import numpy as np
import vdsim
import vdsim_lab

SEDAN = ROOT / "configs/vehicles/generic_sedan.yaml"
TIRE = ROOT / "configs/parts/tire/generic_pacejka.yaml"
COUNT = {"ok": 0}
LEVELS = [("L1", False), ("L2", False), ("L3", False), ("L4", False),
          ("L5", False), ("L5", True)]


def check(cond, label):
    """Assert one case and count it."""
    if not cond:
        raise SystemExit(f"{label}: FAILED")
    COUNT["ok"] += 1
    print(label, "OK")


def raises(exc, fn):
    """True when fn() raises exc."""
    try:
        fn()
    except exc:
        return True
    return False


def solver(level, strut):
    """Core defaults except L5, which uses the stunt solver settings of the C++ gates."""
    sp = vdsim.SolverParams()
    if level == "L5":
        sp.stunt_physics = True
        sp.max_substep_dt = 2e-4
        sp.max_substeps = 16
        sp.l5_spatial_suspension = strut
    return sp


def front_rear():
    """Front: the generic tyre. Rear: mu 1.3, stiffer laterally."""
    front = vdsim.TireParams.from_yaml(str(TIRE))
    rear = vdsim.TireParams.from_yaml(str(TIRE))
    rear.mu_nominal = 1.3
    rear.cornering_stiffness = 100000.0
    rear.B_lat = 9.0
    return front, rear


def cmd(k):
    """Throttle, steer, brake manoeuvre."""
    u = vdsim.CmdL4()
    u.throttle = 0.3 if k < 900 else 0.0
    u.brake = 0.3 if k >= 1400 else 0.0
    u.steer_angle_wheel = 0.0 if k < 300 else (0.06 if k < 1100 else -0.04)
    return u


def init(sess, vp, tp, v=10.0):
    sess.reset(vdsim.make_init_state(vp, tp, v=v))


def pack(o):
    """Bytes of the state and per-tick outputs."""
    st = o.state
    vals = (list(st.position) + list(st.velocity) + list(st.angular_velocity)
            + list(st.wheel_spin) + list(o.Fz) + [o.ax, o.ay])
    return struct.pack(f"{len(vals)}d", *map(float, vals))


def run_bytes(sess, ticks=2000, dt=1e-3):
    out = bytearray()
    for k in range(ticks):
        sess.set_input(cmd(k))
        sess.tick(dt)
        out += pack(sess.output())
    return bytes(out)


# ---- T2 ---------------------------------------------------------------------------------

def t2():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    tp, _ = front_rear()
    same = vdsim.TireSetup(tp, tp)
    for level, strut in LEVELS:
        a = vdsim.make_sim_session(vp, tp, level=level, nominal_dt=1e-3,
                                   solver=solver(level, strut))
        b = vdsim.make_sim_session(vp, same, level=level, nominal_dt=1e-3,
                                   solver=solver(level, strut))
        init(a, vp, tp)
        init(b, vp, tp)
        check(run_bytes(a) == run_bytes(b), f"T2 make_sim_session {level}{' strut' if strut else ''}")
    hm = np.zeros((8, 8))
    for name, mk in (
            ("make_sim_session_heightmap",
             lambda t: vdsim.make_sim_session_heightmap(vp, t, "L3", hm, x0=-50.0, y0=-50.0,
                                                        dx=20.0, dy=20.0, nominal_dt=1e-3)),
            ("make_sim_session_psd",
             lambda t: vdsim.make_sim_session_psd(vp, t, "L3", nominal_dt=1e-3))):
        a, b = mk(tp), mk(same)
        init(a, vp, tp)
        init(b, vp, tp)
        check(run_bytes(a) == run_bytes(b), f"T2 {name} L3")
    va = vdsim.make_vec_session(2, vp, tp, level="L2", nominal_dt=1e-3)
    vb = vdsim.make_vec_session(2, vp, same, level="L2", nominal_dt=1e-3)
    s0 = vdsim.make_init_state(vp, tp, v=10.0)
    ba, bb = bytearray(), bytearray()
    for v, buf in ((va, ba), (vb, bb)):
        v.reset_all([s0])
        for k in range(2000):
            v.set_inputs([cmd(k)])
            v.tick(1e-3)
            for i in range(2):
                buf += pack(v.at(i).output())
    check(bytes(ba) == bytes(bb), "T2 make_vec_session L2 x2")


# ---- T3 ---------------------------------------------------------------------------------

def tyre_out(model, inp):
    r = model.compute(inp)
    return (r.Fx, r.Fy, r.Mz)


def rel(a, b):
    return max(abs(x - y) / max(abs(y), 1.0) for x, y in zip(a, b))


def t3():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    front, rear = front_rear()
    for level, strut in LEVELS:
        sp = solver(level, strut)
        mixed = vdsim.make_sim_session(vp, vdsim.TireSetup(front, rear), level=level,
                                       nominal_dt=1e-3, solver=sp)
        ref_f = vdsim.make_sim_session(vp, front, level=level, nominal_dt=1e-3, solver=sp)
        ref_r = vdsim.make_sim_session(vp, rear, level=level, nominal_dt=1e-3, solver=sp)
        init(mixed, vp, front)
        worst, seen = 0.0, 0.0
        for k in range(400):
            mixed.set_input(cmd(k + 300))
            mixed.tick(1e-3)
            if k % 50 != 49:
                continue
            o = mixed.output()
            for i in range(4):
                inp = vdsim.TireInput()
                inp.Fz = float(o.Fz[i])
                inp.kappa = float(o.slip_ratio[i])
                inp.alpha = float(o.slip_angle[i])
                inp.Vx_wheel = float(o.state.velocity[0])
                got = tyre_out(mixed.dynamics().tire(i), inp)
                ref = ref_f if i < 2 else ref_r
                worst = max(worst, rel(got, tyre_out(ref.dynamics().tire(i), inp)))
                if i >= 2:
                    seen = max(seen, rel(got, tyre_out(ref_f.dynamics().tire(i), inp)))
        tag = f"{level}{' strut' if strut else ''}"
        print(f"T3 {tag} worst rel {worst:.3e} rear-vs-front-param max rel {seen:.3e}")
        check(seen > 1e-3, f"T3 {tag} determinable (rear differs from front parameters)")
        check(worst <= 1e-12, f"T3 {tag} per-axle tyre == independent axle reference")


# ---- T4 (Lab) ---------------------------------------------------------------------------

def t4_lab():
    front, rear = front_rear()
    ft, rt = vdsim_lab.Tire(front), vdsim_lab.Tire(rear)
    sim = vdsim_lab.Sim(vehicle="sedan", tire=ft, tire_rear=rt, level="L2", dt=1e-3, v0=10.0)
    inp = vdsim.TireInput()
    inp.Fz, inp.alpha, inp.Vx_wheel = 4000.0, 0.05, 10.0
    d = sim.sess.dynamics()
    want_r = tyre_out(vdsim.make_sim_session(sim._vp, rear).dynamics().tire(2), inp)
    want_f = tyre_out(vdsim.make_sim_session(sim._vp, front).dynamics().tire(0), inp)
    check(rel(tyre_out(d.tire(2), inp), want_r) <= 1e-12, "T4 Lab Sim tire_rear reaches RL")
    check(rel(tyre_out(d.tire(0), inp), want_f) <= 1e-12, "T4 Lab Sim tire stays on FL")
    a = vdsim_lab.Sim(vehicle="sedan", tire=ft, level="L2", dt=1e-3, v0=10.0)
    b = vdsim_lab.Sim(vehicle="sedan", tire=ft, tire_rear=ft, level="L2", dt=1e-3, v0=10.0)
    check(run_bytes(a.sess) == run_bytes(b.sess), "T4 Lab Sim tire_rear == tire bit identical")
    exp = vdsim_lab.Experiment(level="L2").tire(ft, rt)
    check(exp is not None, "T4 Lab Experiment.tire(front, rear) accepted")
    check(raises(ValueError, lambda: vdsim_lab.Experiment.from_config(
        {"level": "L2", "vehicle": "sedan", "tire_rear": "default_pacejka"})),
        "T4 Lab from_config tire_rear without tire ValueError")
    check(not raises(Exception, lambda: vdsim_lab.Experiment.from_config(
        {"level": "L2", "vehicle": "sedan", "tire": "default_pacejka",
         "tire_rear": "default_pacejka"})),
        "T4 Lab from_config tire + tire_rear accepted")
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    check(raises(TypeError, lambda: vdsim.make_sim_session(vp, "default_pacejka")),
          "T4 make_sim_session non-tyre object TypeError")


# ---- rotor keys -------------------------------------------------------------------------

def sedan_without(*keys):
    return "".join(l + "\n" for l in SEDAN.read_text().splitlines()
                   if not any(l.startswith(k + ":") for k in keys))


BASE = sedan_without("engine_rotational_inertia", "final_drive_ratio", "drive_type")


def vp_text(text):
    p = Path(tempfile.mkdtemp()) / "v.yaml"
    p.write_text(text)
    return vdsim.VehicleParams.from_yaml(str(p))


def rotor_keys():
    vp = vp_text(BASE + "rotor_inertia: 0.3\nrotor_ratio: 4.2\nrotor_axis: \"+y\"\n")
    check(vp.engine_rotational_inertia == 0.3 and vp.rotor_inertia == 0.3,
          "R-py rotor_inertia alias")
    check(vp.final_drive_ratio == 4.2 and vp.rotor_ratio == 4.2, "R-py rotor_ratio alias")
    check(vp.rotor_axis == "+y", "R-py rotor_axis read")
    out = Path(tempfile.mkdtemp()) / "o.yaml"
    vp.to_yaml(str(out))
    txt = out.read_text()
    check("rotor_inertia" in txt and "rotor_ratio" in txt and "rotor_axis" in txt,
          "R-py to_yaml keeps alias names")
    back = vdsim.VehicleParams.from_yaml(str(out))
    check((back.rotor_inertia, back.rotor_ratio, back.rotor_axis) == (0.3, 4.2, "+y"),
          "R-py round trip")
    plain = vdsim.VehicleParams.from_yaml(str(SEDAN))
    po = Path(tempfile.mkdtemp()) / "p.yaml"
    plain.to_yaml(str(po))
    check("rotor_" not in po.read_text(), "R-py unset writes no rotor key")
    py_set = vdsim.VehicleParams()
    py_set.rotor_inertia = 0.4
    py_set.rotor_ratio = 3.5
    check(py_set.engine_rotational_inertia == 0.4 and py_set.final_drive_ratio == 3.5,
          "R-py property setters write the shared storage")
    so = Path(tempfile.mkdtemp()) / "s.yaml"
    py_set.to_yaml(str(so))
    check("rotor_inertia" in so.read_text() and "rotor_ratio" in so.read_text(),
          "R-py setter names kept by to_yaml")
    pt = "powertrain:\n  engine:\n    inertia: 0.2\n"
    for label, text in (
            ("conflict inertia", "rotor_inertia: 0.3\nengine_rotational_inertia: 0.31\n"),
            ("conflict ratio", "rotor_ratio: 4.2\nfinal_drive_ratio: 4.0\n"),
            ("ratio zero", "rotor_ratio: 0.0\n"),
            ("inertia negative", "rotor_inertia: -0.1\n"),
            ("axis unknown", "rotor_axis: \"+z\"\n"),
            ("powertrain v2 inertia", pt + "rotor_inertia: 0.25\n"),
            ("powertrain v2 ratio", pt + "rotor_ratio: 5.0\n")):
        check(raises(ValueError, lambda t=text: vp_text(BASE + t)), f"R-py ValueError {label}")
    # PO condition (mail c8103d86): range checks only for the rotor_* names.
    for label, text in (("final_drive_ratio zero", "final_drive_ratio: 0.0\n"),
                        ("final_drive_ratio negative", "final_drive_ratio: -5.0\n"),
                        ("engine_rotational_inertia negative",
                         "engine_rotational_inertia: -0.1\n")):
        check(not raises(Exception, lambda t=text: vp_text(BASE + t)),
              f"R-py existing name keeps no range check: {label}")
    check(not raises(Exception, lambda: vp_text(BASE + pt + "rotor_axis: \"+x\"\n")),
          "R-py powertrain v2 with rotor_axis accepted")
    bad = vdsim.VehicleParams()
    bad.rotor_axis = "lateral"
    for level, strut in LEVELS:
        check(raises(ValueError, lambda l=level, s=strut: vdsim.make_sim_session(
            bad, vdsim.TireParams(), level=l, nominal_dt=1e-3, solver=solver(l, s))),
            f"R-py session ValueError unknown axis {level}{' strut' if strut else ''}")


CHILD = r"""
import sys
sys.path.insert(0, sys.argv[1])
import vdsim
vp = vdsim.VehicleParams()
vp.rotor_axis = "+y"
for rep in range(2):
    for lvl in ("L1", "L2", "L3", "L4"):
        vdsim.make_sim_session(vp, vdsim.TireParams(), level=lvl, nominal_dt=1e-3)
"""


def warn_once():
    r = subprocess.run([sys.executable, "-c", CHILD, str(ROOT / "python")],
                       capture_output=True, text=True, timeout=120)
    n = (r.stdout + r.stderr).count("rotor_axis=")
    check(r.returncode == 0 and n == 1, f"R-py L1-L4 warn once per process (count {n})")


if __name__ == "__main__":
    t2()
    t3()
    t4_lab()
    rotor_keys()
    warn_once()
    print(f"ALL OK {COUNT['ok']}")
