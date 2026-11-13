"""Work order 57, Python side: tire_low_speed key (no pytest).

Fixed before the implementation commits.
- YAML -> Python -> to_yaml -> YAML keeps off / relaxed; Python set -> to_yaml; unset is
  "" and writes no key.
- An unknown value raises ValueError from from_yaml and when an L2 or L5 session is built.
- Explicit off == unset: L2 and L5 penalty state bits identical over 1 s from 5 m/s with
  brake pedal 0.3 (the U5 run start).
"""
from pathlib import Path
import struct
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim

SEDAN = ROOT / "configs/vehicles/generic_sedan.yaml"
COUNT = {"ok": 0}


def check(cond, label):
    """Assert one case and count it."""
    if not cond:
        raise SystemExit(f"{label}: FAILED")
    COUNT["ok"] += 1
    print(label, "OK")


def raises_value_error(fn):
    """True when fn() raises ValueError."""
    try:
        fn()
    except ValueError:
        return True
    return False


def session(vp, level):
    """Flat-ground session; L5 is the penalty path."""
    sp = vdsim.SolverParams()
    if level == "L5":
        sp.stunt_physics = True
        sp.max_substep_dt = 2e-4
        sp.max_substeps = 16
    return vdsim.make_sim_session(vp, vdsim.TireParams(), level=level,
                                  nominal_dt=1e-3, solver=sp)


def round_trip():
    """YAML / Python / to_yaml round trip and the unknown-value error from YAML."""
    tmp = Path(tempfile.mkdtemp())
    for v in ("off", "relaxed"):
        src = tmp / f"in_{v}.yaml"
        src.write_text(SEDAN.read_text() + f"\ntire_low_speed: {v}\n")
        vp = vdsim.VehicleParams.from_yaml(str(src))
        check(vp.tire_low_speed == v, f"U57-py yaml->python {v}")
        out = tmp / f"out_{v}.yaml"
        vp.to_yaml(str(out))
        check(vdsim.VehicleParams.from_yaml(str(out)).tire_low_speed == v,
              f"U57-py to_yaml round trip {v}")
        vp2 = vdsim.VehicleParams.from_yaml(str(SEDAN))
        vp2.tire_low_speed = v
        out2 = tmp / f"set_{v}.yaml"
        vp2.to_yaml(str(out2))
        check(vdsim.VehicleParams.from_yaml(str(out2)).tire_low_speed == v,
              f"U57-py python set -> to_yaml {v}")
    plain = vdsim.VehicleParams.from_yaml(str(SEDAN))
    check(plain.tire_low_speed == "", "U57-py unset is empty")
    out = tmp / "plain.yaml"
    plain.to_yaml(str(out))
    check("tire_low_speed" not in out.read_text(), "U57-py unset writes no key")
    bad = tmp / "bad.yaml"
    bad.write_text(SEDAN.read_text() + "\ntire_low_speed: Relaxed\n")
    check(raises_value_error(lambda: vdsim.VehicleParams.from_yaml(str(bad))),
          "U57-py unknown value in YAML -> ValueError")


def unknown_value():
    """An unknown value set from Python fails at session build."""
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.tire_low_speed = "on"
    for lvl in ("L2", "L5"):
        check(raises_value_error(lambda: session(vp, lvl)), f"U57-py unknown value {lvl}")


def bits(vp, level):
    """State bytes over 1 s from 5 m/s with brake pedal 0.3."""
    s = session(vp, level)
    st = vdsim.State()
    st.position = [0.0, 0.0, vp.cg_height]
    st.velocity = [5.0, 0.0, 0.0]
    w = 5.0 / vp.wheel_radius_nominal
    st.wheel_spin = [w, w, w, w]
    s.reset_settled(st)
    out = bytearray()
    for _ in range(1000):
        u = vdsim.CmdL4()
        u.throttle = 0.0
        u.brake = 0.3
        s.set_input(u)
        s.tick(1e-3)
        o = s.output()
        x = o.state
        vals = list(x.position) + list(x.velocity) + list(x.angular_velocity) + list(x.wheel_spin)
        out += struct.pack(f"{len(vals)}d", *vals)
    return bytes(out)


def off_equals_unset():
    """Explicit off writes the same state bits as an unset key (L2, L5 penalty)."""
    plain = vdsim.VehicleParams.from_yaml(str(SEDAN))
    off = vdsim.VehicleParams.from_yaml(str(SEDAN))
    off.tire_low_speed = "off"
    for lvl in ("L2", "L5"):
        check(bits(plain, lvl) == bits(off, lvl), f"U57-py off == unset bits {lvl}")


if __name__ == "__main__":
    round_trip()
    unknown_value()
    off_equals_unset()
    print(f"U57-py {COUNT['ok']} cases OK")
