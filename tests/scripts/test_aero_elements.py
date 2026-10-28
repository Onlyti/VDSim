#!/usr/bin/env python3
"""Per-element aero, Python side (work order 42, 0.8.0).

* A1 -- ``vdsim.compute_aero_loads`` on the example car gives the wiki check
  vector (485.4 N downforce, 237.9 N drag at 60 km/h, rho 1.165).
* A7 -- ``VehicleParams.from_yaml`` raises ``ValueError`` for elements with
  any of the four keys or ``aero_drag_height``, a duplicate name, CdA < 0 and
  z < 0; on L1/L2 an element ``y`` is ignored with one warning per process.
* E3 -- ``aero_drag_height`` is ``None`` when unset and ``0.0`` survives a
  YAML round trip as ``0.0``.
* A9 -- the opt-in ``"aero"`` trace channels equal the core function at the
  recorded speed; recording them leaves every other channel unchanged; the
  0.5 reader of main before this work opens the trace.
"""
import importlib.util
import math
import os
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim  # noqa: E402
import vdsim_trace as vt  # noqa: E402
from vdsim_plant import VDSimPlant  # noqa: E402

EXAMPLE = ROOT / "configs" / "vehicles" / "fs_aero_example.yaml"
V60 = 60.0 / 3.6

#: main before work order 42; its 0.5 reader must open an aero trace.
MAIN_READER_SHA = "08e2e90"

ELEMENTS = textwrap.dedent("""\
    wheelbase: 1.55
    cg_to_front: 0.82
    cg_to_rear: 0.73
    aero:
      elements:
        - {name: front_wing, ClA: 0.70, CdA: 0.12, x:  0.550, z: 0.100}
        - {name: rear_wing,  ClA: 1.50, CdA: 0.55, x: -1.520, z: 1.000}
    """)


def _rel(a, b):
    return abs(a - b) / abs(b)


def _load_text(tmp, text, name="v.yaml"):
    p = Path(tmp) / name
    p.write_text(text)
    return vdsim.VehicleParams.from_yaml(str(p))


def test_a1_binding_matches_wiki_vector():
    vp = vdsim.VehicleParams.from_yaml(str(EXAMPLE))
    assert [e.name for e in vp.aero_elements] == [
        "front_wing", "rear_wing", "undertray", "body"]
    loads = vdsim.compute_aero_loads(vp, vdsim.AeroState(V60))
    assert _rel(loads.fz_total, 485.4) <= 1e-3, loads.fz_total
    assert _rel(loads.fx_total, 237.9) <= 1e-3, loads.fx_total
    assert abs(100.0 * loads.balance_front - 29.4) <= 0.1, loads.balance_front
    assert len(loads.elements) == 4
    print("A1: %.2f N downforce, %.2f N drag, front %.2f %%"
          % (loads.fz_total, loads.fx_total, 100.0 * loads.balance_front))


def test_a7_double_specification_is_value_error():
    with tempfile.TemporaryDirectory() as tmp:
        _load_text(tmp, ELEMENTS)
        bad = {
            "lift_front": ELEMENTS + "aero_lift_front: 0.3\n",
            "lift_rear": ELEMENTS + "aero_lift_rear: 0.3\n",
            "drag_height": ELEMENTS + "aero_drag_height: 0.3\n",
            "duplicate": ELEMENTS + "    - {name: rear_wing, ClA: 0.1, CdA: 0.0, x: 0.0, z: 0.1}\n",
            "cda_negative": ELEMENTS.replace("CdA: 0.12", "CdA: -0.12"),
            "z_negative": ELEMENTS.replace("z: 0.100", "z: -0.100"),
        }
        for label, text in bad.items():
            try:
                _load_text(tmp, text, label + ".yaml")
            except ValueError:
                continue
            raise AssertionError("%s accepted" % label)
        # A programmatic double specification is refused when a session is built.
        vp = vdsim.VehicleParams.from_yaml(str(EXAMPLE))
        vp.aero_lift_front = 0.2
        try:
            vdsim.validate_aero_params(vp)
        except ValueError:
            pass
        else:
            raise AssertionError("lift + elements accepted by validate_aero_params")
    print("A7: elements with lift keys / drag height, duplicate names, CdA<0, z<0 -> ValueError")


_Y_PROBE = textwrap.dedent("""\
    import sys
    sys.path.insert(0, %(py)r)
    import vdsim
    tp = vdsim.TireParams.from_yaml(%(tire)r)
    def run(y):
        vp = vdsim.VehicleParams.from_yaml(%(car)r)
        els = list(vp.aero_elements)
        els[1].y = y
        vp.aero_elements = els
        s = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=0.001)
        s.reset(vdsim.make_init_state(v=%(v)r, wheel_radius=vp.wheel_radius_nominal))
        cmd = vdsim.CmdL4(); cmd.throttle = 0.2
        for _ in range(500):
            s.set_input(cmd); s.tick(0.001)
        o = s.output()
        return (o.state.vx(), o.state.yaw_rate(), tuple(o.Fz))
    a = run(0.2); b = run(0.2); c = run(0.0)
    print("SAME" if a == c and b == c else "DIFF", flush=True)
    """)


def test_a7_y_ignored_on_l2_with_one_warning():
    code = _Y_PROBE % {"py": str(ROOT / "python"), "car": str(EXAMPLE), "v": V60,
                       "tire": str(ROOT / "configs/parts/tire/generic_pacejka.yaml")}
    env = dict(os.environ)
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                         env=env, timeout=120)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip().endswith("SAME"), out.stdout
    n = (out.stdout + out.stderr).count("is ignored on L1/L2")
    assert n == 1, (n, out.stderr)
    print("A7: element y ignored on L2 (bit-identical to y=0), one warning per process")


def test_e3_drag_height_none_and_zero():
    vp = vdsim.VehicleParams()
    assert vp.aero_drag_height is None
    assert vp.air_density == 1.225
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "unset.yaml"
        vp.to_yaml(str(p))
        assert vdsim.VehicleParams.from_yaml(str(p)).aero_drag_height is None
        vp.aero_drag_height = 0.0
        p = Path(tmp) / "zero.yaml"
        vp.to_yaml(str(p))
        back = vdsim.VehicleParams.from_yaml(str(p)).aero_drag_height
        assert back is not None and back == 0.0, back
        vp.aero_drag_height = None
        assert vp.aero_drag_height is None
    print("E3: aero_drag_height None stays None, 0.0 stays 0.0 through YAML")


def _main_reader(tmp):
    """``vdsim_trace`` as of :data:`MAIN_READER_SHA`, or ``None`` when that
    commit is not in the checkout (shallow clone)."""
    try:
        src = subprocess.run(
            ["git", "-C", str(ROOT), "show", f"{MAIN_READER_SHA}:python/vdsim_trace.py"],
            capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    path = Path(tmp) / "vdsim_trace_main.py"
    path.write_bytes(src)
    spec = importlib.util.spec_from_file_location("vdsim_trace_main", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _record(path, extra):
    plant = VDSimPlant(config=str(EXAMPLE), level="L3", control_dt=0.01)
    plant.reset([0.0, 0.0, 0.0, V60, 0.0, 0.0])
    plant.enable_trace(path, decimation=1, extra_channels=extra)
    for k in range(200):
        plant.step([0.02 if k > 50 else 0.0, 300.0])
    plant.finalize_trace()
    return plant


def test_a9_aero_trace_channels():
    with tempfile.TemporaryDirectory() as tmp:
        on = Path(tmp) / "aero_on.vdtrace"
        off = Path(tmp) / "aero_off.vdtrace"
        plant = _record(on, ("aero",))
        _record(off, ())
        names = vt.aero_channel_names([e.name for e in plant._vp.aero_elements])
        with vt.TraceReader(on) as a, vt.TraceReader(off) as b:
            for n in names:
                assert a.has(n), n
                assert not b.has(n), n
            assert a.channel_meta("aero_fz_total")["unit"] == "N"
            vx = a.channel("v_body")[:, 0]
            worst = 0.0
            for i, v in enumerate(vx):
                ld = vdsim.compute_aero_loads(plant._vp, vdsim.AeroState(float(v)))
                got = a.channel("aero_fz_total")[i]
                worst = max(worst, _rel(got, ld.fz_total))
                assert _rel(a.channel("aero_rear_wing_fx")[i], ld.elements[1].fx) <= 1e-9
                assert _rel(a.channel("aero_balance_front")[i], ld.balance_front) <= 1e-9
            assert worst <= 1e-9, worst
            for n in b.channel_names():
                assert np.array_equal(a.channel(n), b.channel(n), equal_nan=True), n
        main = _main_reader(tmp)
        if main is not None:
            with main.TraceReader(on) as tr:
                assert tr.has("aero_fz_total")
                assert tr.n_steps == 200
            print("A9: main %s 0.5 reader opens the aero trace" % MAIN_READER_SHA)
    print("A9: %d aero channels equal compute_aero_loads (worst rel %.1e); "
          "other channels identical with aero off" % (len(names), worst))


def main():
    test_a1_binding_matches_wiki_vector()
    test_a7_double_specification_is_value_error()
    test_a7_y_ignored_on_l2_with_one_warning()
    test_e3_drag_height_none_and_zero()
    test_a9_aero_trace_channels()
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
