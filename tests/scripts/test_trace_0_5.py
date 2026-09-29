#!/usr/bin/env python3
"""Schema ``0.5`` contract tests — ``rp_rate``, ``wheel_travel`` and ``channel_validity``.

What is checked, and why each one can rot:

1. the marker rules — a 0.5 trace without ``channel_validity`` is an error, a
   ``not_modeled`` channel must be exactly ``0.0``, and the marker must carry the
   run's own ``model_level``;
2. old traces (0.3 / 0.4) read as ``unknown`` with one warning, never ``modeled``;
3. what the compiled core really declares and what the plant really writes: L1
   and L2 runs hold all-zero ``rp_rate`` / ``wheel_travel`` under
   ``not_modeled@L1`` / ``not_modeled@L2``; L3 holds live values under ``modeled``
   and the rate is the model's own state variable, not a difference of samples.
"""
from __future__ import annotations

import inspect
import json
import math
import sys
import tempfile
import warnings
import zipfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace as vt  # noqa: E402

FIX = REPO / "tests" / "fixtures" / "trace"
FIXTURE_0_3 = FIX / "golden_v0_3.vdtrace"
FIXTURE_0_4 = FIX / "golden_v0_4.vdtrace"
FIXTURE_0_5 = FIX / "golden_v0_5.vdtrace"

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


GEOMETRY = {
    "wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0,
    "mass_kg": 1800.0, "cg_height_m": 0.55, "wheel_radius_m": 0.32,
    "wheel_width_m": 0.225, "body_lwh_m": [4.6, 1.9, 1.5],
}


def _writer(path, level, validity, **kw):
    args = dict(
        geometry=dict(GEOMETRY),
        tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
        repro={"vdsim_version": "test", "git_sha": "x", "param_hash": "sha256:x",
               "seed": 1, "dt_s": 0.01, "run_id": "t"},
        producer={"name": "test_trace_0_5", "version": "0"},
        role="plant", model_level=level, contact_scope="C2",
        kinematics_attached=False, channel_validity=validity,
    )
    args.update(kw)
    return vt.TraceWriter(path=path, **args)


def _sample(i, names, live):
    full = {
        "t": i * 0.01, "pose": (i * 0.1, 0.0, 0.0), "v_body": (10.0, 0.0),
        "yaw_rate": 0.0, "u_steer": 0.01, "u_fx": -100.0,
        "wheel_F": [(100.0, 50.0, 5000.0)] * 4, "wheel_mu": [0.9] * 4,
        "wheel_kappa": [0.01] * 4, "wheel_alpha": [0.02] * 4,
        "pose_zrp": (0.55, 0.01, -0.002), "a_body": (0.5, 1.2, -0.03),
        "wheel_road_dz": [0.01, -0.01, 0.01, -0.01],
        "wheel_road_normal": [(0.0, -0.1, 0.995)] * 4,
        "wheel_travel": [0.06] * 4 if live else [0.0] * 4,
        "rp_rate": (0.03, -0.02) if live else (0.0, 0.0),
    }
    return {k: full[k] for k in names}


def _rewrite(src, dst, mutate):
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w") as zout:
        for info in zin.infolist():
            data = zin.read(info.filename)
            if info.filename == "manifest.json":
                m = json.loads(data.decode())
                mutate(m)
                data = json.dumps(m).encode()
            zout.writestr(info.filename, data)
    return dst


def _write_run(path, level, validity, live):
    names = vt.channels_for_level(level)
    w = _writer(path, level, validity, channels=names)
    for i in range(6):
        w.append(_sample(i, names, live))
    return w.finalize()


# ------------------------------------------------------------ 1. channel table
def test_channels_are_offered_at_every_level():
    check(vt.SCHEMA_VERSION == "0.5", "module is at schema 0.5")
    for lvl in vt.MODEL_LEVELS:
        names = vt.channels_for_level(lvl)
        check("rp_rate" in names and "wheel_travel" in names,
              "%s is offered rp_rate and wheel_travel" % lvl)
    check(vt.CHANNEL_SPECS["rp_rate"] == ("rad/s", (2,)), "rp_rate is rad/s, shape [n,2]")
    check(vt.CHANNEL_SPECS["wheel_travel"] == ("m", (4,)), "wheel_travel is m, shape [n,4]")
    check("pose_zrp" not in vt.channels_for_level("L2"),
          "the other L3 channels are still gated (no zero-fill)")
    check("rp_rate" not in vt.BASE_CHANNELS and "wheel_travel" not in vt.BASE_CHANNELS,
          "the 0.1 base set is unchanged")


# ------------------------------------------------------------- 2. fixture 0.5
def test_fixture_0_5():
    check(FIXTURE_0_5.is_file(), "0.5 fixture is committed")
    if not FIXTURE_0_5.is_file():
        return
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with vt.TraceReader(FIXTURE_0_5) as tr:
            check(tr.manifest["schema_version"] == "0.5", "fixture declares schema 0.5")
            check(tr.model_level == "L2", "fixture declares model_level L2")
            check(tr.channel_validity == {"rp_rate": "not_modeled@L2",
                                          "wheel_travel": "not_modeled@L2"},
                  "fixture marks both channels not_modeled@L2 (%s)" % tr.channel_validity)
            for name in ("rp_rate", "wheel_travel"):
                arr = np.asarray(tr.channel(name))
                check(bool((arr == 0.0).all()), "%s is exactly 0.0 throughout" % name)
                check(tr.channel_label(name) == "not modeled",
                      "%s is labelled 'not modeled'" % name)
            check(tr.validity("pose") == "modeled", "a channel with no marker reads modeled")
    check(not caught, "a 0.5 trace reads without warnings (%d)" % len(caught))


# ------------------------------------------------------------ 3. marker rules
def test_marker_rules():
    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        both = {"rp_rate": "modeled", "wheel_travel": "modeled"}

        try:
            _writer(d / "x.vdtrace", "L3", None)
            check(False, "channel_validity=None is rejected")
        except vt.TraceError:
            check(True, "channel_validity=None is rejected at write time")

        try:
            vt.TraceWriter(
                path=d / "x.vdtrace", geometry=dict(GEOMETRY),
                tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
                repro={"vdsim_version": "t", "git_sha": "x", "param_hash": "sha256:x",
                       "seed": 1, "dt_s": 0.01, "run_id": "t"},
                role="plant", model_level="L3", contact_scope="C2",
                kinematics_attached=False)
            check(False, "channel_validity is a required TraceWriter argument")
        except TypeError:
            check(True, "channel_validity is a required TraceWriter argument")

        for label, bad in (
                ("a missing entry", {"rp_rate": "modeled"}),
                ("an unknown value", dict(both, rp_rate="maybe")),
                ("a marker for another level", dict(both, rp_rate="not_modeled@L1")),
                ("an entry for a channel that is not recorded", dict(both, pose="modeled"))):
            try:
                _writer(d / "x.vdtrace", "L3", bad)
                check(False, "%s is rejected" % label)
            except vt.TraceError:
                check(True, "%s is rejected" % label)

        # A not-modeled channel holding a non-zero value is a producer bug.
        p = d / "l2_bad.vdtrace"
        names = vt.channels_for_level("L2")
        w = _writer(p, "L2", {"rp_rate": "not_modeled@L2", "wheel_travel": "not_modeled@L2"},
                    channels=names)
        w.append(_sample(0, names, live=True))
        try:
            w.finalize()
            check(False, "non-zero data under not_modeled is refused")
        except vt.TraceError as exc:
            check("not_modeled" in str(exc), "non-zero data under not_modeled is refused")

        # Reader side: a manifest edited to claim not_modeled over live data.
        live = _write_run(d / "l3.vdtrace", "L3", both, live=True)
        with vt.TraceReader(live) as tr:
            check(tr.channel_validity == both and tr.channel_label("rp_rate") == "",
                  "an L3 run reads modeled with no label")
            check(float(np.abs(np.asarray(tr.channel("rp_rate"))).max()) > 0.0,
                  "the modeled rate channel carries its values")

        lie = _rewrite(live, d / "lie.vdtrace",
                       lambda m: m["channel_validity"].__setitem__("rp_rate", "not_modeled@L3"))
        with vt.TraceReader(lie) as tr:
            try:
                tr.channel("rp_rate")
                check(False, "reading live data under a not_modeled marker fails")
            except vt.TraceError:
                check(True, "reading live data under a not_modeled marker fails")

        gone = _rewrite(live, d / "gone.vdtrace", lambda m: m.pop("channel_validity"))
        try:
            vt.TraceReader(gone)
            check(False, "a 0.5 manifest without channel_validity is rejected")
        except vt.TraceSchemaError as exc:
            check("channel_validity" in str(exc),
                  "a 0.5 manifest without channel_validity is rejected")

        half = _rewrite(live, d / "half.vdtrace",
                        lambda m: m["channel_validity"].pop("wheel_travel"))
        try:
            vt.TraceReader(half)
            check(False, "a 0.5 manifest missing one entry is rejected")
        except vt.TraceSchemaError:
            check(True, "a 0.5 manifest missing one entry is rejected")

        # L1 traces record the same two channels, marked at their own level.
        l1 = _write_run(d / "l1.vdtrace", "L1",
                        {"rp_rate": "not_modeled@L1", "wheel_travel": "not_modeled@L1"},
                        live=False)
        with vt.TraceReader(l1) as tr:
            check(tr.validity("rp_rate") == "not_modeled@L1", "an L1 run reads not_modeled@L1")


# ------------------------------------------------------------ 4. older schemas
def test_old_traces_are_unknown_not_modeled():
    for fixture, ver in ((FIXTURE_0_3, "0.3"), (FIXTURE_0_4, "0.4")):
        if not fixture.is_file():
            check(False, "%s fixture present" % ver)
            continue
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            with vt.TraceReader(fixture) as tr:
                first = tr.channel_validity
                again = tr.channel_validity
                single = tr.validity("wheel_travel")
        msgs = [str(x.message) for x in caught
                if issubclass(x.category, UserWarning) and "channel_validity" in str(x.message)]
        check(first == {"wheel_travel": "unknown"} and again == first,
              "%s: wheel_travel reads unknown, never modeled (%s)" % (ver, first))
        check(single == "unknown", "%s: validity() agrees" % ver)
        check(len(msgs) == 1, "%s: exactly one warning across reads (got %d)" % (ver, len(msgs)))
        check(msgs and ver in msgs[0], "%s: the warning names the version" % ver)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            with vt.TraceReader(fixture) as tr:
                check(tr.channel_label("wheel_travel") == "model status unknown",
                      "%s: the label says the status is unknown" % ver)


# ------------------------------------------------- 5. the compiled core, measured
def test_plant_records_measured_validity():
    try:
        import vdsim  # noqa: F401
        from vdsim_plant import VDSimPlant, trace_sample
    except Exception as exc:                       # noqa: BLE001
        check(False, "compiled core importable for the plant probe (%s)" % exc)
        return

    with tempfile.TemporaryDirectory() as td:
        for lvl in ("L1", "L2", "L3"):
            p = Path(td) / (lvl + ".vdtrace")
            plant = VDSimPlant(control_dt=0.01, substep_dt=1e-3, base_mu=0.9, level=lvl)
            plant.reset([0.0, 0.0, 0.0, 15.0, 0.0, 0.0])
            plant.enable_trace(p, seed=0, run_id=lvl)
            last = None
            for k in range(300):
                plant.step([0.05 * math.sin(0.25 * k), 1500.0])
                last = plant._sess.output()
            plant.finalize_trace()
            with vt.TraceReader(p) as tr:
                rp = np.asarray(tr.channel("rp_rate"))
                wt = np.asarray(tr.channel("wheel_travel"))
                check(tr.manifest["schema_version"] == "0.5", "%s run is a 0.5 trace" % lvl)
                check(rp.shape == (tr.n_steps, 2) and wt.shape == (tr.n_steps, 4),
                      "%s run: rp_rate [n,2], wheel_travel [n,4]" % lvl)
                if lvl in ("L1", "L2"):
                    check(tr.channel_validity == {"rp_rate": "not_modeled@" + lvl,
                                                  "wheel_travel": "not_modeled@" + lvl},
                          "%s run states not_modeled@%s" % (lvl, lvl))
                    check(bool((rp == 0.0).all()) and bool((wt == 0.0).all()),
                          "%s run: rp_rate and wheel_travel are all 0.0" % lvl)
                else:
                    check(tr.channel_validity == {"rp_rate": "modeled",
                                                  "wheel_travel": "modeled"},
                          "%s run states modeled" % lvl)
                    check(float(np.abs(rp).max()) > 1e-4,
                          "%s run: rp_rate is nonzero in a lateral maneuver (%.4f rad/s peak)"
                          % (lvl, float(np.abs(rp).max())))
                    check(float(wt.max() - wt.min()) > 1e-6, "%s run: wheel_travel moves" % lvl)
            if lvl == "L3":
                # The recorded rate is the model's own published state, exactly.
                validity = {"rp_rate": "modeled", "wheel_travel": "modeled"}
                s = trace_sample(last, 0.0, 0.0, 0.0, ("rp_rate",), validity)
                ang = last.state.angular_velocity
                check(s["rp_rate"] == (float(ang[0]), float(ang[1])),
                      "L3 rp_rate equals state.angular_velocity x,y bit for bit")

    src = inspect.getsource(trace_sample)
    check("np.diff" not in src and "np.gradient" not in src and "numpy" not in src,
          "the producer path contains no numerical differentiation")


# ------------------------------------------------------------------- renderer
def test_renderer_labels_not_modeled():
    if not FIXTURE_0_5.is_file():
        check(False, "0.5 fixture present for the renderer probe")
        return
    try:
        import vdsim_render3d as r3
    except Exception as exc:                       # noqa: BLE001
        check(False, "renderer importable (%s)" % exc)
        return
    scene = r3.Scene3D(FIXTURE_0_5)
    check(tuple(scene.not_modeled) == ("rp_rate", "wheel_travel"),
          "the 3D scene carries the not-modeled channel list (%s)" % (scene.not_modeled,))
    check(scene.not_modeled_label == "not modeled", "the 3D scene label reads 'not modeled'")
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        old = r3.Scene3D(FIXTURE_0_4)
    check(old.not_modeled == () and not caught,
          "a 0.4 trace shows no label and triggers no warning in the renderer")


def main():
    test_channels_are_offered_at_every_level()
    test_fixture_0_5()
    test_marker_rules()
    test_old_traces_are_unknown_not_modeled()
    test_plant_records_measured_validity()
    test_renderer_labels_not_modeled()
    print("\n%d checks failed" % len(FAILURES))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    raise SystemExit(main())
