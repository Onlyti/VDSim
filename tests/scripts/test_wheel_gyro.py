"""Work order 50, Python side: wheel_gyro key (no pytest).

- YAML -> Python -> to_yaml -> YAML keeps legacy / full / off; unset writes no key.
- An unknown value raises ValueError from from_yaml and when any level's session is built.
- L1-L4 read the key and ignore it: one warning per process (checked in a child process).
- L5 (penalty and strut, zero camber): unset == legacy == full bit for bit, off differs.
"""
from pathlib import Path
import os
import subprocess
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


def session(vp, level, strut=False):
    """Build a session on flat ground; L5 uses the strut path when strut is True."""
    sp = vdsim.SolverParams()
    if level == "L5":
        sp.stunt_physics = True
        sp.max_substep_dt = 2e-4
        sp.max_substeps = 16
        sp.l5_spatial_suspension = strut
    return vdsim.make_sim_session(vp, vdsim.TireParams(), level=level,
                                  nominal_dt=1e-3, solver=sp)


def raises_value_error(fn):
    """True when fn() raises ValueError."""
    try:
        fn()
    except ValueError:
        return True
    return False


def round_trip():
    tmp = Path(tempfile.mkdtemp())
    for v in ("legacy", "full", "off"):
        src = tmp / f"in_{v}.yaml"
        src.write_text(SEDAN.read_text() + f"\nwheel_gyro: {v}\n")
        vp = vdsim.VehicleParams.from_yaml(str(src))
        check(vp.wheel_gyro == v, f"V-py yaml->python {v}")
        out = tmp / f"out_{v}.yaml"
        vp.to_yaml(str(out))
        check(vdsim.VehicleParams.from_yaml(str(out)).wheel_gyro == v,
              f"V-py to_yaml round trip {v}")
        vp2 = vdsim.VehicleParams.from_yaml(str(SEDAN))
        vp2.wheel_gyro = v
        out2 = tmp / f"set_{v}.yaml"
        vp2.to_yaml(str(out2))
        check(vdsim.VehicleParams.from_yaml(str(out2)).wheel_gyro == v,
              f"V-py python set -> to_yaml {v}")
    plain = vdsim.VehicleParams.from_yaml(str(SEDAN))
    check(plain.wheel_gyro == "", "V-py unset is empty")
    out = tmp / "plain.yaml"
    plain.to_yaml(str(out))
    check("wheel_gyro" not in out.read_text(), "V-py unset writes no key")
    bad = tmp / "bad.yaml"
    bad.write_text(SEDAN.read_text() + "\nwheel_gyro: Full\n")
    check(raises_value_error(lambda: vdsim.VehicleParams.from_yaml(str(bad))),
          "V-py unknown value in YAML -> ValueError")


def unknown_value():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.wheel_gyro = "on"
    for lvl in ("L1", "L2", "L3", "L4"):
        check(raises_value_error(lambda: session(vp, lvl)), f"V-py unknown value {lvl}")
    check(raises_value_error(lambda: session(vp, "L5", False)), "V-py unknown value L5 penalty")
    check(raises_value_error(lambda: session(vp, "L5", True)), "V-py unknown value L5 strut")


CHILD = r"""
import sys
sys.path.insert(0, sys.argv[1])
import vdsim
def session(vp, level, strut=False):
    sp = vdsim.SolverParams()
    if level == "L5":
        sp.stunt_physics = True; sp.max_substep_dt = 2e-4; sp.max_substeps = 16
        sp.l5_spatial_suspension = strut
    return vdsim.make_sim_session(vp, vdsim.TireParams(), level=level, nominal_dt=1e-3, solver=sp)
vp = vdsim.VehicleParams()
vp.wheel_gyro = "off"
session(vp, "L5", True)
session(vp, "L5", False)
for rep in range(2):
    for lvl in ("L1", "L2", "L3", "L4"):
        session(vp, lvl)
"""


def warns_once():
    r = subprocess.run([sys.executable, "-c", CHILD, str(ROOT / "python")],
                       capture_output=True, text=True, env=dict(os.environ), timeout=120)
    if r.returncode != 0:
        raise SystemExit("V-py warning child failed: " + r.stderr[-2000:])
    n = (r.stdout + r.stderr).count("wheel_gyro=")
    check(n == 1, f"V-py L1-L4 warn once, none on L5 (count {n})")


def state_bits(vp, strut):
    """float.hex of the state after a 300-tick turn-in (exact comparison)."""
    s = session(vp, "L5", strut)
    st = vdsim.State()
    st.position = [0.0, 0.0, vp.cg_height]
    st.velocity = [12.0, 0.0, 0.0]
    w = 12.0 / vp.wheel_radius_nominal
    st.wheel_spin = [w, w, w, w]
    s.reset_settled(st)
    for k in range(300):
        u = vdsim.CmdL4()
        u.throttle = 0.2
        u.steer_angle_wheel = 0.0 if k < 100 else 0.05
        s.set_input(u)
        s.tick(1e-3)
    x = s.state()
    vals = list(x.position) + list(x.velocity) + list(x.angular_velocity) + list(x.wheel_spin)
    return [float(v).hex() for v in vals]


def l5_bits():
    for strut in (False, True):
        tag = "strut" if strut else "penalty"
        base = vdsim.VehicleParams.from_yaml(str(SEDAN))
        unset = state_bits(base, strut)
        for v, same in (("legacy", True), ("full", True), ("off", False)):
            vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
            vp.wheel_gyro = v
            got = state_bits(vp, strut)
            check((got == unset) == same, f"V-py L5 {tag} {v} {'==' if same else '!='} unset")


round_trip()
unknown_value()
warns_once()
l5_bits()
print(f"wheel_gyro python OK ({COUNT['ok']} checks)")
