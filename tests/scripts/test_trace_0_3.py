#!/usr/bin/env python3
"""Schema ``0.3`` contract tests (``11_trace_contract_spec`` §3.1.1 / §3.2.1).

Three things are checked that nothing else in the suite covers:

1. the ``0.3`` manifest gates — ``model_level`` / ``contact_scope`` / the five
   geometry keys are **errors** when missing, not warnings;
2. the no-zero-fill rule — a level that does not have a quantity must omit the
   channel rather than store a column of zeros;
3. that the declared ``contact_scope`` is what the compiled core actually does.
   (3) is the one that rots: the constant lives in Python and the behaviour
   lives in C++, so it is measured here rather than asserted.
"""
from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace as vt  # noqa: E402

FIXTURE_0_3 = REPO / "tests" / "fixtures" / "trace" / "golden_v0_3.vdtrace"
FIXTURE_0_2 = REPO / "tests" / "fixtures" / "trace" / "golden_v0_2.vdtrace"
FIXTURE_0_4 = REPO / "tests" / "fixtures" / "trace" / "golden_v0_4.vdtrace"

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


GEOMETRY_0_3 = {
    "wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0,
    "mass_kg": 1800.0, "cg_height_m": 0.55, "wheel_radius_m": 0.32,
    "wheel_width_m": 0.225, "body_lwh_m": [4.6, 1.9, 1.5],
}


def _writer(path, **kw):
    args = dict(
        geometry=dict(GEOMETRY_0_3),
        tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
        repro={"vdsim_version": "test", "git_sha": "x", "param_hash": "sha256:x",
               "seed": 1, "dt_s": 0.01, "run_id": "t"},
        producer={"name": "test_trace_0_3", "version": "0"},
        role="plant", model_level="L3", contact_scope="C2",
        kinematics_attached=False,
    )
    args.update(kw)
    return vt.TraceWriter(path=path, **args)


def _sample(i, names, dt=0.01):
    full = {
        "t": i * dt, "pose": (i * 0.1, 0.0, 0.0), "v_body": (10.0, 0.0),
        "yaw_rate": 0.0, "u_steer": 0.01, "u_fx": -100.0,
        "wheel_F": [(100.0, 50.0, 5000.0)] * 4, "wheel_mu": [0.9] * 4,
        "wheel_kappa": [0.01] * 4, "wheel_alpha": [0.02] * 4,
        "pose_zrp": (0.55, 0.01, -0.002), "a_body": (0.5, 1.2, -0.03),
        "wheel_road_dz": [0.01, -0.01, 0.01, -0.01],
        "wheel_road_normal": [(0.0, -0.1, 0.995)] * 4,
        "wheel_travel": [0.06] * 4,
    }
    return {k: full[k] for k in names}


# ---------------------------------------------------------------- 1. fixture
def test_fixture_0_3():
    """The committed L3 fixture carries every 0.3 channel with real values."""
    check(FIXTURE_0_3.is_file(), "0.3 fixture is committed")
    if not FIXTURE_0_3.is_file():
        return
    with vt.TraceReader(FIXTURE_0_3) as tr:
        check(tr.manifest["schema_version"] == "0.3", "fixture declares schema 0.3")
        check(tr.model_level == "L3", "fixture declares model_level L3")
        check(tr.contact_scope in vt.CONTACT_SCOPES,
              "fixture declares a known contact_scope (%s)" % tr.contact_scope)
        n = tr.n_steps
        for name in vt.CHANNEL_MIN_LEVEL:
            check(tr.has(name), "fixture carries 0.3 channel %r" % name)
            arr = tr.channel(name)
            trailing = vt.CHANNEL_SPECS[name][1]
            check(arr.shape == (n,) + trailing,
                  "%r shape %s == manifest %s" % (name, arr.shape, (n,) + trailing))
        # No zero-fill: every 0.3 column has to actually vary, otherwise the
        # fixture would pass the shape checks while proving nothing.
        for name in ("pose_zrp", "a_body", "wheel_road_dz", "wheel_travel"):
            arr = np.asarray(tr.channel(name))
            spread = float(arr.max(axis=0).max() - arr.min(axis=0).min())
            check(spread > 1e-6, "%r varies over the run (spread %.4g)" % (name, spread))
        nrm = np.asarray(tr.channel("wheel_road_normal"))
        norms = np.linalg.norm(nrm, axis=-1)
        check(float(np.abs(norms - 1.0).max()) < 1e-9,
              "wheel_road_normal rows are unit vectors")
        check(float(np.abs(nrm[..., 1]).max()) > 1e-3,
              "wheel_road_normal leaves (0,0,1) — the bank is actually recorded")
        az = np.asarray(tr.channel("a_body"))[:, 2]
        check(float(np.abs(az).max()) > 1e-3,
              "a_body carries a real az column, not a zero placeholder")


def test_0_2_still_reads():
    """A 0.2 trace keeps opening, and reports no level rather than a guess."""
    if not FIXTURE_0_2.is_file():
        check(False, "0.2 fixture present")
        return
    with vt.TraceReader(FIXTURE_0_2) as tr:
        check(tr.manifest["schema_version"] == "0.2", "0.2 fixture still declares 0.2")
        check(tr.model_level is None,
              "a 0.2 trace reports model_level None rather than an invented level")
        check(tr.contact_scope is None, "a 0.2 trace reports contact_scope None")
        check(not tr.has("pose_zrp"), "a 0.2 trace carries no 3D channels")
        check(tr.role == "plant", "the 0.2 role path is untouched")


# ---------------------------------------------------------- 2. manifest gates
def test_required_fields_are_errors():
    """0.3 rejects a manifest that omits the new required fields."""
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        for kw, what in ((dict(model_level=None), "model_level"),
                         (dict(contact_scope=None), "contact_scope")):
            try:
                _writer(d / "x.vdtrace", **kw)
                check(False, "missing %s is rejected" % what)
            except vt.TraceError:
                check(True, "missing %s is rejected at write time" % what)

        for bad in ("L9", "l3", ""):
            try:
                _writer(d / "x.vdtrace", model_level=bad)
                check(False, "model_level %r is rejected" % bad)
            except vt.TraceError:
                check(True, "model_level %r is rejected" % bad)
        try:
            _writer(d / "x.vdtrace", contact_scope="C3")
            check(False, "contact_scope 'C3' is rejected")
        except vt.TraceError:
            check(True, "contact_scope 'C3' is rejected")

        for key in ("mass_kg", "cg_height_m", "wheel_radius_m",
                    "wheel_width_m", "body_lwh_m"):
            geo = dict(GEOMETRY_0_3)
            geo.pop(key)
            try:
                _writer(d / "x.vdtrace", geometry=geo)
                check(False, "geometry without %s is rejected" % key)
            except vt.TraceError:
                check(True, "geometry without %s is rejected" % key)

        geo = dict(GEOMETRY_0_3, body_lwh_m=[4.6, 1.9])
        try:
            _writer(d / "x.vdtrace", geometry=geo)
            check(False, "a 2-element body_lwh_m is rejected")
        except vt.TraceError:
            check(True, "a 2-element body_lwh_m is rejected")

        geo = dict(GEOMETRY_0_3, mass_kg=0.0)
        try:
            _writer(d / "x.vdtrace", geometry=geo)
            check(False, "mass_kg 0 is rejected")
        except vt.TraceError:
            check(True, "mass_kg 0 is rejected")


def test_no_zero_fill():
    """A level that lacks a quantity may not record the channel at all."""
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        check(set(vt.channels_for_level("L2")) & {"pose_zrp", "wheel_road_dz"} == set(),
              "L2 is not offered the L3+ channels")
        check("a_body" in vt.channels_for_level("L2"),
              "L2 is offered a_body (required from L2 up)")
        check("a_body" not in vt.channels_for_level("L1"),
              "L1 is not offered a_body")
        try:
            _writer(d / "x.vdtrace", model_level="L2",
                    channels=list(vt.channels_for_level("L2")) + ["pose_zrp"])
            check(False, "recording pose_zrp at L2 is refused")
        except vt.TraceError:
            check(True, "recording pose_zrp at L2 is refused (no zero-fill)")

        # Round trip a legitimate L2 run: the 3D channels must be *absent*, and
        # `has()` is what tells a reader apart from a column of zeros.
        p = d / "l2.vdtrace"
        names = vt.channels_for_level("L2")
        w = _writer(p, model_level="L2", channels=names)
        for i in range(5):
            w.append(_sample(i, names))
        w.finalize()
        with vt.TraceReader(p) as tr:
            check(not tr.has("pose_zrp"), "an L2 trace has no pose_zrp channel")
            check(tr.has("a_body"), "an L2 trace does have a_body")
            check(tr.model_level == "L2", "the L2 trace declares its level")


def test_display_only_label_rule():
    """`normal_is_display_only` is driven by the declared scope, one place only."""
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        names = vt.channels_for_level("L3")
        for scope, expect in (("C1", True), ("C0", True), ("C2", False)):
            p = d / ("s_%s.vdtrace" % scope)
            w = _writer(p, model_level="L3", contact_scope=scope, channels=names)
            for i in range(4):
                w.append(_sample(i, names))
            w.finalize()
            with vt.TraceReader(p) as tr:
                check(tr.normal_is_display_only is expect,
                      "contact_scope %s -> display-only label %s" % (scope, expect))


# ------------------------------------------------- 3. declaration vs the core
def _step_with_normal(maker, tilted, bank, n=400, dt=0.002):
    """Run one dynamics model over contacts that differ only in the normal."""
    import vdsim
    import vdsim_plant

    vp_path = vdsim_plant.resolve_vehicle_config("ioniq5_awd.yaml")
    vp = vdsim.VehicleParams.from_yaml(str(vp_path))
    tp = vdsim_plant._load_tire_setup_for_vehicle(vp_path).wheel[0]
    sp = vdsim.SolverParams()
    sp.integrator = vdsim.Integrator.RK4
    dyn = maker()
    dyn.initialize(vp, tp, sp)
    dyn.reset(vdsim.make_init_state(vp, tp, x=0.0, y=0.0, yaw=0.0, v=10.0))
    contacts = []
    for _ in range(4):
        cp = vdsim.ContactPoint()
        cp.is_valid = True
        cp.mu_long = cp.mu_lat = 0.9
        cp.normal = ([0.0, -math.sin(bank), math.cos(bank)] if tilted
                     else [0.0, 0.0, 1.0])
        contacts.append(cp)
    cmd = vdsim.CmdL4()
    cmd.gear = 1
    for _ in range(n):
        dyn.step(cmd, contacts, dt)
    return dyn.state().position[1]


def test_declared_contact_scope_matches_the_core():
    """`CONTACT_SCOPE_BY_LEVEL` must describe what the compiled core does.

    The discriminator is the road normal's lateral component: step the same
    model over contacts that are identical except for the normal. If the path
    moves, x/y reach the physics and the scope is C2; if it does not, only the
    z component is consumed and the scope is C1.
    """
    try:
        import vdsim  # noqa: F401
        import vdsim_plant
    except Exception as exc:                       # noqa: BLE001
        check(False, "compiled core importable for the scope probe (%s)" % exc)
        return

    import vdsim
    bank = math.radians(10.0)
    makers = {
        "L1": vdsim.create_bicycle,
        "L2": vdsim.create_seven_dof,
        "L3": vdsim.create_fourteen_dof,
        "L4": vdsim.create_fourteen_dof_kinematic,
    }
    for level, maker in makers.items():
        moved = abs(_step_with_normal(maker, True, bank)
                    - _step_with_normal(maker, False, bank))
        observed = "C2" if moved > 1e-9 else "C1"
        declared = vdsim_plant.CONTACT_SCOPE_BY_LEVEL[level]
        check(declared == observed,
              "%s declares %s and the core behaves as %s (|dy| = %.3e m)"
              % (level, declared, observed, moved))


# ------------------------------------------------ 4. schema 0.4 (11 §13)
def _rewrite(src, dst, mutate):
    """Copy a trace with its manifest passed through ``mutate``."""
    import json
    import zipfile
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w") as zout:
        for info in zin.infolist():
            data = zin.read(info.filename)
            if info.filename == "manifest.json":
                m = json.loads(data.decode())
                mutate(m)
                data = json.dumps(m).encode()
            zout.writestr(info.filename, data)
    return dst


def test_fixture_0_4():
    """The committed 0.4 fixture states kinematics_attached as a real bool."""
    import warnings
    check(FIXTURE_0_4.is_file(), "0.4 fixture is committed")
    if not FIXTURE_0_4.is_file():
        return
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with vt.TraceReader(FIXTURE_0_4) as tr:
            check(tr.manifest["schema_version"] == "0.4", "fixture declares schema 0.4")
            check(tr.kinematics_attached is False,
                  "fixture declares kinematics_attached false (%r)" % tr.kinematics_attached)
            for name in vt.CHANNEL_MIN_LEVEL:
                check(tr.has(name), "0.4 fixture keeps 0.3 channel %r" % name)
    check(not caught, "a 0.4 trace reads without warnings (%d)" % len(caught))


def test_0_4_requires_kinematics_attached():
    """0.4 without kinematics_attached is an error, never a silent False."""
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        try:
            kw = dict(geometry=dict(GEOMETRY_0_3),
                      tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
                      repro={"vdsim_version": "t", "git_sha": "x", "param_hash": "sha256:x",
                             "seed": 1, "dt_s": 0.01, "run_id": "t"},
                      role="plant", model_level="L3", contact_scope="C2")
            vt.TraceWriter(path=d / "x.vdtrace", **kw)
            check(False, "kinematics_attached is a required TraceWriter argument")
        except TypeError:
            check(True, "kinematics_attached is a required TraceWriter argument")
        for bad in (None, 0, "false"):
            try:
                _writer(d / "x.vdtrace", kinematics_attached=bad)
                check(False, "kinematics_attached %r is rejected" % (bad,))
            except vt.TraceError:
                check(True, "kinematics_attached %r is rejected at write time" % (bad,))

        good = d / "good.vdtrace"
        names = vt.channels_for_level("L3")
        w = _writer(good, channels=names)
        for i in range(3):
            w.append(_sample(i, names))
        w.finalize()
        missing = _rewrite(good, d / "missing.vdtrace",
                           lambda m: m.pop("kinematics_attached"))
        try:
            vt.TraceReader(missing)
            check(False, "a 0.4 manifest without kinematics_attached is rejected")
        except vt.TraceSchemaError as exc:
            print("      0.4 missing -> TraceSchemaError: %s" % exc)
            check("kinematics_attached" in str(exc),
                  "a 0.4 manifest without kinematics_attached is rejected")
        wrong = _rewrite(good, d / "wrong.vdtrace",
                         lambda m: m.__setitem__("kinematics_attached", "yes"))
        try:
            vt.TraceReader(wrong)
            check(False, "a non-bool kinematics_attached is rejected on read")
        except vt.TraceSchemaError:
            check(True, "a non-bool kinematics_attached is rejected on read")


def test_0_3_kinematics_is_unknown_with_one_warning():
    """A 0.3 trace predates the field: unknown (None), warned about once."""
    import warnings
    if not FIXTURE_0_3.is_file():
        check(False, "0.3 fixture present")
        return
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with vt.TraceReader(FIXTURE_0_3) as tr:
            first = tr.kinematics_attached
            again = tr.kinematics_attached
    msgs = [str(x.message) for x in caught if issubclass(x.category, UserWarning)]
    for m in msgs:
        print("      0.3 fallback -> UserWarning: %s" % m)
    check(first is None and again is None,
          "a 0.3 trace reports kinematics_attached None, not False (%r)" % (first,))
    check(len(msgs) == 1, "exactly one warning across two reads (got %d)" % len(msgs))
    check(msgs and "0.3" in msgs[0] and "kinematics_attached" in msgs[0],
          "the warning names the version and the field")


def test_plant_records_0_3():
    """An L3 plant run produces a 0.3 trace that a reader can use as-is."""
    try:
        import vdsim  # noqa: F401
        from vdsim_plant import VDSimPlant
    except Exception as exc:                       # noqa: BLE001
        check(False, "compiled core importable for the plant probe (%s)" % exc)
        return
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "l3.vdtrace"
        plant = VDSimPlant(control_dt=0.01, substep_dt=1e-3, base_mu=0.9, level="L3")
        plant.reset([0.0, 0.0, 0.0, 12.0, 0.0, 0.0])
        plant.enable_trace(p, seed=0, run_id="l3_probe")
        for k in range(120):
            plant.step([0.05 * math.sin(0.25 * k), 1500.0])
        plant.finalize_trace()
        with vt.TraceReader(p) as tr:
            check(tr.model_level == "L3", "recorded manifest declares L3")
            check(tr.contact_scope == "C2", "recorded manifest declares the measured scope")
            check(tr.kinematics_attached is False,
                  "the plant, which never attaches hardpoints, records false")
            for name in vt.CHANNEL_MIN_LEVEL:
                check(tr.has(name), "L3 run records %r" % name)
            zrp = np.asarray(tr.channel("pose_zrp"))
            check(float(np.abs(zrp[:, 1]).max()) > 1e-4,
                  "recorded roll is non-trivial (%.4f rad peak)"
                  % float(np.abs(zrp[:, 1]).max()))
            trav = np.asarray(tr.channel("wheel_travel"))
            check(float(trav.max() - trav.min()) > 1e-6,
                  "recorded wheel_travel moves")
            geo = tr.geometry
            for key in ("mass_kg", "cg_height_m", "wheel_radius_m",
                        "wheel_width_m", "body_lwh_m"):
                check(key in geo, "recorded geometry carries %s" % key)

        # L2 stays a 0.3 trace too, but without the L3 channels.
        p2 = Path(td) / "l2.vdtrace"
        plant2 = VDSimPlant(control_dt=0.01, substep_dt=1e-3, base_mu=0.9)
        plant2.reset([0.0, 0.0, 0.0, 12.0, 0.0, 0.0])
        plant2.enable_trace(p2, seed=0, run_id="l2_probe")
        for _ in range(30):
            plant2.step([0.01, 1000.0])
        plant2.finalize_trace()
        with vt.TraceReader(p2) as tr:
            check(tr.model_level == "L2", "the default plant still records L2")
            check(not tr.has("pose_zrp"),
                  "an L2 run omits pose_zrp instead of writing zeros")


def main():
    test_fixture_0_3()
    test_0_2_still_reads()
    test_required_fields_are_errors()
    test_no_zero_fill()
    test_display_only_label_rule()
    test_declared_contact_scope_matches_the_core()
    test_fixture_0_4()
    test_0_4_requires_kinematics_attached()
    test_0_3_kinematics_is_unknown_with_one_warning()
    test_plant_records_0_3()
    print("\n%d checks failed" % len(FAILURES))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    raise SystemExit(main())
