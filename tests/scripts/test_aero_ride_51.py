"""Work order 51, Python side: aero ride-height / pitch slopes (no pytest).

- B2: compute_aero_loads(vp, AeroState(vx, dz, pitch)) on the fixed grid equals
  tests/fixtures/aero_ride_51/b2_expected.json (1e-9 relative, 1e-12 for zeros).
- B6: one or two of the three keys, non-finite slopes -> ValueError; L1/L2 with a
  non-zero slope -> ValueError, with the keys and both slopes 0 accepted; to_yaml
  writes the keys only for elements that had them and reads back the same values.
"""
from pathlib import Path
import json
import math
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim

FS = ROOT / "configs/vehicles/fs_aero_example.yaml"
FIX = ROOT / "tests/fixtures/aero_ride_51"
COUNT = {"ok": 0}


def check(cond, label):
    """Assert one case and count it."""
    if not cond:
        raise SystemExit(f"{label}: FAILED")
    COUNT["ok"] += 1
    print(label, "OK")


def rel_close(a, b, tol):
    """|a - b| <= tol |b|, or |a| <= 1e-12 when b == 0."""
    if b == 0.0:
        return abs(a) <= 1e-12
    return abs(a - b) <= tol * abs(b)


def with_slopes(vp, slopes):
    """vp with the slope keys of the named elements set (list re-assigned)."""
    els = list(vp.aero_elements)
    for e in els:
        if e.name in slopes:
            h_ref, dh, dp = slopes[e.name]
            e.h_ref, e.dClA_dh, e.dClA_dpitch = h_ref, dh, dp
    vp.aero_elements = els
    return vp


def session(vp, level):
    """Session on flat ground at 1 ms."""
    sp = vdsim.SolverParams()
    if level == "L5":
        sp.stunt_physics = True
        sp.max_substep_dt = 2e-4
        sp.max_substeps = 16
    return vdsim.make_sim_session(vp, vdsim.TireParams(), level=level,
                                  nominal_dt=1e-3, solver=sp)


def raises_value_error(fn):
    """True when fn() raises ValueError."""
    try:
        fn()
    except ValueError:
        return True
    return False


def b2():
    """B2 on the fixed grid."""
    exp = json.loads((FIX / "b2_expected.json").read_text())
    slopes = {k: (v["h_ref"], v["dClA_dh"], v["dClA_dpitch"]) for k, v in exp["slopes"].items()}
    vp = with_slopes(vdsim.VehicleParams.from_yaml(str(FS)), slopes)
    ok = True
    for c in exp["cases"]:
        loads = vdsim.compute_aero_loads(vp, vdsim.AeroState(exp["vx"], c["dz"], c["theta"]))
        for e, el in zip(vp.aero_elements, loads.elements):
            ok = ok and rel_close(el.fz, c["fz"][e.name], exp["rel_tol"])
            ok = ok and rel_close(el.h, c["h"][e.name], exp["rel_tol"])
    check(ok and len(exp["cases"]) == 9, "B2 grid fz and h (9 cases x 4 elements)")


def b6():
    """B6 rejection and round trip."""
    for keys in ({"h_ref"}, {"dClA_dh"}, {"dClA_dpitch"}, {"h_ref", "dClA_dh"},
                 {"h_ref", "dClA_dpitch"}, {"dClA_dh", "dClA_dpitch"}):
        vp = vdsim.VehicleParams.from_yaml(str(FS))
        els = list(vp.aero_elements)
        for k in keys:
            setattr(els[0], k, 0.1)
        vp.aero_elements = els
        check(raises_value_error(lambda: session(vp, "L3")), f"B6 partial keys {sorted(keys)}")
    for k in ("h_ref", "dClA_dh", "dClA_dpitch"):
        vp = with_slopes(vdsim.VehicleParams.from_yaml(str(FS)), {"undertray": (0.04, -2.0, 0.0)})
        els = list(vp.aero_elements)
        setattr(els[2], k, math.nan)
        vp.aero_elements = els
        check(raises_value_error(lambda: session(vp, "L3")), f"B6 non-finite {k}")
    active = with_slopes(vdsim.VehicleParams.from_yaml(str(FS)), {"front_wing": (0.1, 0.0, 30.0)})
    zero = with_slopes(vdsim.VehicleParams.from_yaml(str(FS)), {"front_wing": (0.1, 0.0, 0.0)})
    for lvl in ("L1", "L2"):
        check(raises_value_error(lambda: session(active, lvl)), f"B6 {lvl} slope rejected")
        session(zero, lvl)
        check(True, f"B6 {lvl} zero slopes accepted")
    vp = with_slopes(vdsim.VehicleParams.from_yaml(str(FS)), {"undertray": (0.04, -2.0, 0.5)})
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "rt.yaml"
        vp.to_yaml(str(p))
        text = p.read_text()
        back = vdsim.VehicleParams.from_yaml(str(p))
    check(text.count("h_ref") == 1, "B6 to_yaml writes the keys once")
    same = all((a.h_ref, a.dClA_dh, a.dClA_dpitch) == (b.h_ref, b.dClA_dh, b.dClA_dpitch)
               for a, b in zip(vp.aero_elements, back.aero_elements))
    check(same, "B6 round trip keeps the slopes (None where unset)")


def main():
    b2()
    b6()
    print(f"PASSED {COUNT['ok']} checks")


if __name__ == "__main__":
    main()
