"""Work order 51, Python side: aero ride-height / pitch slopes (no pytest).

- B2: compute_aero_loads(vp, AeroState(vx, dz, pitch)) on the fixed grid equals
  tests/fixtures/aero_ride_51/b2_expected.json (1e-9 relative, 1e-12 for zeros).
- B4: a Python AeroModule re-implementing the elements (with slopes) reproduces
  the built-in path on L3 and L5 penalty within 1e-9 relative, every tick.
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


class ElementAero(vdsim.AeroModule):
    """Python re-implementation of the built-in elements incl. the ride slopes (B4)."""

    def __init__(self, vp):
        super().__init__()
        self.x_o = -vp.cg_to_front
        self.els = [(e.name, e.ClA, e.CdA, e.x, e.y, e.z, e.h_ref, e.dClA_dh, e.dClA_dpitch)
                    for e in vp.aero_elements]

    def compute(self, ctx, inp):
        vx = float(inp.v_air_body[0])
        q_down = 0.5 * inp.rho * vx * vx
        q_drag = 0.5 * inp.rho * vx * abs(vx)
        out = []
        for name, cla, cda, x, y, z, h_ref, s_h, s_p in self.els:
            if h_ref is not None and (s_h != 0.0 or s_p != 0.0):
                dh = inp.dz_heave - (x - self.x_o) * inp.pitch
                cla = cla + s_h * dh + s_p * inp.pitch
            out.append(vdsim.AeroForce(name, (-cda * q_drag, 0.0, -cla * q_down), (x, y, z)))
        return out


def state_vec(o):
    """State components compared by B4 (attitude as yaw / roll / pitch of the output)."""
    st = o.state
    v = list(st.position) + list(st.velocity) + list(st.angular_velocity)
    v += [st.yaw(), o.roll, o.pitch]
    v += list(st.wheel_spin) + list(st.susp_compression) + list(st.susp_velocity)
    return [float(x) for x in v]


def b4():
    """B4: Python element module == built-in elements, L3 and L5 penalty, 5 s at 60 km/h."""
    v = 60.0 / 3.6
    vp = with_slopes(vdsim.VehicleParams.from_yaml(str(FS)),
                     {"undertray": (0.040, -2.0, 0.0), "front_wing": (0.100, 0.0, 30.0)})
    tp = vdsim.TireParams.from_yaml(str(ROOT / "configs/parts/tire/generic_pacejka.yaml"))
    for lvl in ("L3", "L5"):
        runs = []
        for use_module in (False, True):
            sp = vdsim.SolverParams()
            if lvl == "L5":
                sp.stunt_physics = True
                sp.max_substep_dt = 2e-4
                sp.max_substeps = 16
            s = vdsim.make_sim_session(vp, tp, level=lvl, nominal_dt=1e-3, solver=sp)
            module = ElementAero(vp) if use_module else None
            if use_module:
                check(s.dynamics().set_aero_module(module), f"B4 {lvl} module attached")
            s0 = vdsim.State()
            s0.position = [0.0, 0.0, vp.cg_height]
            s0.velocity = [v, 0.0, 0.0]
            w = v / vp.wheel_radius_nominal
            s0.wheel_spin = [w, w, w, w]
            s.reset_settled(s0)
            cmd = vdsim.CmdL6()
            cmd.v_target = v
            rows = [state_vec(s.output())]
            for _ in range(5000):
                s.set_input(cmd)
                s.tick(1e-3)
                rows.append(state_vec(s.output()))
            runs.append(rows)
        worst = 0.0
        for ra, rb in zip(runs[1], runs[0]):
            for a, b in zip(ra, rb):
                worst = max(worst, abs(a - b) / max(1.0, abs(b)))
        print(f"B4 {lvl} max relative state difference {worst:.3e}")
        check(worst <= 1e-9 and len(runs[0]) == 5001, f"B4 {lvl} module == built-in (1e-9)")


def main():
    b2()
    b6()
    b4()
    print(f"PASSED {COUNT['ok']} checks")


if __name__ == "__main__":
    main()
