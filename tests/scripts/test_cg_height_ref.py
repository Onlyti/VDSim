"""Work order 47 G7, Python side: cg_height_ref round trip and ValueError cases (no pytest).

- YAML -> Python -> to_yaml -> YAML keeps "vehicle" / "sprung"; unset writes no key.
- An unknown value raises ValueError when a session is built (every level).
- h_s <= R + 0.01 m raises ValueError on the L5 strut path only; "sprung" accepts it.
- L1-L4 and the L5 penalty path warn once per process (checked in a child process).
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
    for v in ("vehicle", "sprung"):
        src = tmp / f"in_{v}.yaml"
        src.write_text(SEDAN.read_text() + f"\ncg_height_ref: {v}\n")
        vp = vdsim.VehicleParams.from_yaml(str(src))
        check(vp.cg_height_ref == v, f"G7-py yaml->python {v}")
        out = tmp / f"out_{v}.yaml"
        vp.to_yaml(str(out))
        check(vdsim.VehicleParams.from_yaml(str(out)).cg_height_ref == v,
              f"G7-py to_yaml round trip {v}")
        vp2 = vdsim.VehicleParams.from_yaml(str(SEDAN))
        vp2.cg_height_ref = v
        out2 = tmp / f"set_{v}.yaml"
        vp2.to_yaml(str(out2))
        check(vdsim.VehicleParams.from_yaml(str(out2)).cg_height_ref == v,
              f"G7-py python set -> to_yaml {v}")
    plain = vdsim.VehicleParams.from_yaml(str(SEDAN))
    check(plain.cg_height_ref == "", "G7-py unset is empty")
    out = tmp / "plain.yaml"
    plain.to_yaml(str(out))
    check("cg_height_ref" not in out.read_text(), "G7-py unset writes no key")
    bad = tmp / "bad.yaml"
    bad.write_text(SEDAN.read_text() + "\ncg_height_ref: body\n")
    check(raises_value_error(lambda: vdsim.VehicleParams.from_yaml(str(bad))),
          "G7-py unknown value in YAML -> ValueError")


def unknown_value():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.cg_height_ref = "Sprung"
    for lvl in ("L1", "L2", "L3", "L4"):
        check(raises_value_error(lambda: session(vp, lvl)), f"G7-py unknown value {lvl}")
    check(raises_value_error(lambda: session(vp, "L5", False)), "G7-py unknown value L5 penalty")
    check(raises_value_error(lambda: session(vp, "L5", True)), "G7-py unknown value L5 strut")


def body_below_wheel():
    vp = vdsim.VehicleParams()          # m 1500, sum m_u 160, R 0.32 -> h <= 0.32107 fails
    vp.cg_height = 0.321
    check(raises_value_error(lambda: session(vp, "L5", True)), "G7-py h_s <= R+0.01 L5 strut")
    session(vp, "L5", False)
    check(True, "G7-py h_s <= R+0.01 L5 penalty accepted")
    session(vp, "L2")
    check(True, "G7-py h_s <= R+0.01 L2 accepted")
    vp.cg_height_ref = "sprung"
    session(vp, "L5", True)
    check(True, "G7-py h_s <= R+0.01 with sprung accepted")


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
vp.cg_height_ref = "sprung"
session(vp, "L5", True)
for lvl in ("L1", "L2", "L3", "L4"):
    session(vp, lvl)
session(vp, "L5", False)
"""


def warns_once():
    env = dict(os.environ)
    r = subprocess.run([sys.executable, "-c", CHILD, str(ROOT / "python")],
                       capture_output=True, text=True, env=env, timeout=120)
    if r.returncode != 0:
        raise SystemExit("G7-py warning child failed: " + r.stderr[-2000:])
    n = (r.stdout + r.stderr).count("cg_height_ref=")
    check(n == 1, f"G7-py L1-L4 + L5 penalty warn once (count {n})")


round_trip()
unknown_value()
body_below_wheel()
warns_once()
print(f"cg_height_ref python G7 OK ({COUNT['ok']} checks)")
