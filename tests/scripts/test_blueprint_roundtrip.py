#!/usr/bin/env python3
"""Round-trip: catalog blueprint -> resolve -> load YAML -> simulate -> sane.

Exercises the full parts pipeline end to end: every blueprint in the catalog must
resolve to a vehicle.yaml/tire.yaml that the core can load and integrate without
producing non-finite state, and a driven vehicle must actually accelerate.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "build" / "python"))

import vdsim
from catalog.resolver import CatalogResolver

OUT = ROOT / "build" / "_roundtrip_test"


def _resolve_and_run(resolver, bp_id, level):
    rv = resolver.resolve_blueprint(bp_id, out_dir=OUT / bp_id.replace(".", "_"))
    assert rv.vehicle_yaml.exists(), f"{bp_id}: vehicle.yaml not written"
    assert rv.tire_yaml.exists(), f"{bp_id}: tire.yaml not written"

    vp = vdsim.VehicleParams.from_yaml(str(rv.vehicle_yaml))
    tp = vdsim.TireParams.from_yaml(str(rv.tire_yaml))
    R = vp.wheel_radius_nominal

    s = vdsim.make_sim_session(vp, tp, rv.level, mu=1.0, nominal_dt=0.005)
    s.reset(vdsim.make_init_state(v=5.0, wheel_radius=R))
    cmd = vdsim.CmdL4()
    cmd.throttle = 0.4
    for _ in range(200):                       # 1 s
        s.set_input(cmd)
        s.tick(0.005)
    st = s.output().state

    for name, val in (("vx", st.vx()), ("vy", st.vy()),
                      ("yaw_rate", st.yaw_rate()), ("x", st.position[0])):
        assert math.isfinite(val), f"{bp_id} ({rv.level}): non-finite {name}={val}"
    assert abs(st.vx()) < 200.0, f"{bp_id}: vx blew up ({st.vx():.1f})"
    if rv.level != "K":                        # kinematic has no drive accel
        assert st.vx() > 5.0, f"{bp_id} ({rv.level}): should accelerate (vx={st.vx():.2f})"
    return rv.level


def test_default_blueprint_roundtrip():
    from catalog.ids import DEFAULT_BLUEPRINT
    r = CatalogResolver(ROOT)
    bps = {b["id"]: b for b in r.list_blueprints()}
    assert DEFAULT_BLUEPRINT in bps, "default blueprint missing from catalog"
    _resolve_and_run(r, DEFAULT_BLUEPRINT, bps[DEFAULT_BLUEPRINT]["level"])


def test_all_blueprints_roundtrip():
    r = CatalogResolver(ROOT)
    bps = r.list_blueprints()
    assert len(bps) >= 3, f"expected >=3 blueprints, got {len(bps)}"
    levels = set()
    for b in bps:
        levels.add(_resolve_and_run(r, b["id"], b["level"]))
    # the catalog should exercise at least the ride model (L3) somewhere
    assert "L3" in levels, f"no L3 blueprint round-tripped (levels={levels})"


if __name__ == "__main__":
    test_default_blueprint_roundtrip()
    test_all_blueprints_roundtrip()
    print("ok")
