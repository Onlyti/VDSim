#!/usr/bin/env python3
"""L5 drive / brake torque-reaction keys, Python side (work order 43, 0.8.1).

* Keys -- ``VehicleParams.drive_reaction_*`` / ``brake_reaction_*`` are ``None``
  when unset, accept ``"sprung"`` / ``"unsprung"``, raise ``ValueError`` for any
  other string, and survive a YAML round trip.
* J4 -- ``vdsim.free_3d_strut_force`` gives NaN off the L5 spatial-strut path and
  the finite static preload on it.
* J5 -- ``VDSimPlant`` on L5 records the effective target per axle in its plant
  parameters (unset default on the penalty path: drive sprung, brake equivalent).
"""
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim  # noqa: E402
from vdsim_plant import VDSimPlant  # noqa: E402

SEDAN = ROOT / "configs" / "vehicles" / "generic_sedan.yaml"
KEYS = ("drive_reaction_front", "drive_reaction_rear",
        "brake_reaction_front", "brake_reaction_rear")


def test_keys_none_values_and_round_trip():
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    assert all(getattr(vp, k) is None for k in KEYS)
    vp.drive_reaction_rear = "unsprung"
    vp.brake_reaction_front = "sprung"
    for k in KEYS:
        try:
            setattr(vp, k, "inboard")
        except ValueError:
            pass
        else:
            raise AssertionError("%s accepted 'inboard'" % k)
    assert vp.drive_reaction_rear == "unsprung" and vp.brake_reaction_front == "sprung"
    with tempfile.TemporaryDirectory() as d:
        out = str(Path(d) / "rx.yaml")
        vp.to_yaml(out)
        back = vdsim.VehicleParams.from_yaml(out)
    assert [getattr(back, k) for k in KEYS] == [None, "unsprung", "sprung", None]
    vp.drive_reaction_rear = None
    assert vp.drive_reaction_rear is None
    print("keys: None when unset, ValueError on other strings, YAML round trip kept")


def _l5_session(strut):
    sp = vdsim.SolverParams()
    sp.stunt_physics = True
    sp.max_substep_dt = 2e-4
    sp.max_substeps = 16
    sp.l5_spatial_suspension = strut
    vp = vdsim.VehicleParams.from_yaml(str(SEDAN))
    return vdsim.make_sim_session(vp, vdsim.TireParams(), level="L5",
                                  nominal_dt=1e-3, solver=sp), vp


def test_j4_strut_force_nan_off_path():
    pen, _ = _l5_session(False)
    assert all(math.isnan(f) for f in vdsim.free_3d_strut_force(pen.dynamics()))
    strut, vp = _l5_session(True)
    s0 = vdsim.State()
    pos = s0.position.copy()
    pos[2] = vp.cg_height
    s0.position = pos
    strut.reset_settled(s0)
    f = vdsim.free_3d_strut_force(strut.dynamics())
    assert all(math.isfinite(x) and x > 0.0 for x in f), f
    print("J4: strut force NaN on the penalty path, static preload %.1f N on the strut" % f[0])


def test_j5_plant_records_targets():
    plant = VDSimPlant(config=str(SEDAN), level="L5", control_dt=0.01)
    tr = plant._plant_params["torque_reaction"]
    assert tr == {"drive_front": "sprung", "drive_rear": "sprung",
                  "brake_front": "equivalent", "brake_rear": "equivalent"}, tr
    l2 = VDSimPlant(config=str(SEDAN), level="L2", control_dt=0.01)
    assert "torque_reaction" not in l2._plant_params
    print("J5: L5 plant records %s; L2 records none" % tr)


def main():
    test_keys_none_values_and_round_trip()
    test_j4_strut_force_nan_off_path()
    test_j5_plant_records_targets()
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
