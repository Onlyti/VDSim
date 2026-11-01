"""48b Python T3/T4/T7: L5 rack_travel ValueError cases (no pytest)."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim

SEDAN = ROOT / "configs/vehicles/generic_sedan.yaml"


def _l5_session(vp):
    sp = vdsim.SolverParams()
    sp.stunt_physics = True
    sp.max_substep_dt = 2e-4
    sp.max_substeps = 16
    sp.l5_spatial_suspension = True
    return vdsim.make_sim_session(
        vp, vdsim.TireParams(), level="L5", nominal_dt=1e-3, solver=sp
    )


def expect_error(fn, label):
    try:
        fn()
    except Exception as e:
        print(label, "OK", type(e).__name__, e)
        return
    raise SystemExit(f"{label}: expected an error")


def t3():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.steer_input = "rack_travel"
    s = _l5_session(vp)
    s.set_input(vdsim.CmdL4())
    s.tick(1e-3)


def t4():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.steer_input = "rack_travel"
    vp.steering_dynamic = True
    _l5_session(vp)


def t7():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    vp.steer_input = "nope"
    _l5_session(vp)


expect_error(t3, "T3")
expect_error(t4, "T4")
expect_error(t7, "T7")
print("l5_rack_travel python T3/T4/T7 OK")
