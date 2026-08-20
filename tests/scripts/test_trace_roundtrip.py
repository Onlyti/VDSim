#!/usr/bin/env python3
"""Container contract tests for ``.vdtrace`` (DoD 1-3).

Covers the golden-fixture round trip, schema-version rejection, the required
``tire`` block, overlay pass-through rules, decimation exactness and the
``param_hash`` scope. Uses only stdlib + numpy: the compiled core is not
needed, which is itself part of the contract (a trace must be readable where
the simulator cannot run).
"""
from __future__ import annotations

import json
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_trace as vt  # noqa: E402

FIXTURE = REPO / "tests" / "fixtures" / "trace" / "golden_v0_1.vdtrace"

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


def _minimal_writer(path, **kw):
    args = dict(
        geometry={"wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0},
        tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
        repro={"vdsim_version": "test", "git_sha": "x", "param_hash": "sha256:x",
               "seed": 1, "dt_s": 0.01, "run_id": "t"},
        producer={"name": "test", "version": "0"},
    )
    args.update(kw)
    return vt.TraceWriter(path=path, **args)


def _sample(i, dt=0.01):
    return {
        "t": i * dt, "pose": (i * 0.1, 0.0, 0.0), "v_body": (10.0, 0.0),
        "yaw_rate": 0.0, "u_steer": 0.01 * i, "u_fx": -100.0 * i,
        "wheel_F": [(100.0, 50.0, 5000.0)] * 4, "wheel_mu": [0.9] * 4,
        "wheel_kappa": [0.01] * 4, "wheel_alpha": [0.02] * 4,
    }


def test_golden_roundtrip():
    """The committed fixture loads, and every channel matches its manifest shape."""
    check(FIXTURE.is_file(), "golden fixture is committed at %s" % FIXTURE.name)
    if not FIXTURE.is_file():
        return
    with vt.TraceReader(FIXTURE) as tr:
        n = tr.n_steps
        check(n > 0, "fixture has %d samples" % n)
        for name, (unit, trailing) in vt.CHANNEL_SPECS.items():
            check(tr.has(name), "fixture carries channel %r" % name)
            arr = tr.channel(name)
            check(arr.shape == (n,) + trailing,
                  "channel %r shape %s == manifest %s" % (name, arr.shape, (n,) + trailing))
            check(arr.dtype == np.dtype("<f8"), "channel %r is float64" % name)
        t = tr.channel("t")
        check(bool(np.all(np.diff(t) > 0)), "channel 't' is strictly increasing")
        util = tr.utilization()
        check(util.shape == (n, 4), "utilization derives to [n,4]")
        check(float(util.max()) > 0.5, "fixture reaches util %.2f (exercises colouring)"
              % float(util.max()))
        check(sorted(tr.overlay_names()) == ["mu_entry", "mu_patch", "ref_path"],
              "fixture carries the three overlays")


def test_write_read_identity():
    """Values written come back bit-identical after a round trip."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "rt.vdtrace"
        w = _minimal_writer(p)
        samples = [_sample(i) for i in range(37)]
        for s in samples:
            w.append(s)
        w.finalize()
        with vt.TraceReader(p) as tr:
            check(tr.n_steps == 37, "n_steps round-trips (37)")
            check(np.array_equal(tr.channel("u_steer"),
                                 np.array([s["u_steer"] for s in samples])),
                  "u_steer round-trips bit-identically")
            check(np.array_equal(tr.channel("wheel_F"),
                                 np.array([s["wheel_F"] for s in samples])),
                  "wheel_F [n,4,3] round-trips bit-identically")
        with zipfile.ZipFile(p) as zf:
            infos = {i.filename: i for i in zf.infolist()}
            check(all(i.compress_type == zipfile.ZIP_STORED for i in infos.values()),
                  "channels are stored uncompressed (frombuffer path)")
            check("manifest.json" in infos, "manifest.json is present")


def test_schema_version_mismatch():
    """A future schema is rejected before any channel is parsed (DoD 2)."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "v.vdtrace"
        w = _minimal_writer(p)
        for i in range(4):
            w.append(_sample(i))
        w.finalize()

        bad = Path(td) / "bad.vdtrace"
        with zipfile.ZipFile(p) as src, zipfile.ZipFile(bad, "w") as dst:
            for info in src.infolist():
                data = src.read(info.filename)
                if info.filename == "manifest.json":
                    m = json.loads(data.decode())
                    m["schema_version"] = "9.9"
                    # Also corrupt a channel: a correct implementation must fail
                    # on the version before it ever touches the channel bytes.
                    data = json.dumps(m).encode()
                elif info.filename.startswith("channels/"):
                    data = b"\x00" * 3
                dst.writestr(info.filename, data)
        try:
            vt.TraceReader(bad)
            check(False, "schema_version 9.9 rejected")
        except vt.TraceSchemaError as exc:
            check("9.9" in str(exc) and "schema_version" in str(exc),
                  "schema_version 9.9 rejected with a clear message")
        except Exception as exc:  # a channel parse error means the order is wrong
            check(False, "version checked before channels (got %r)" % (exc,))


def test_tire_block_required():
    """A trace without a usable tyre block is an error, never a silent default."""
    for bad_tire in (None, {}, {"friction_shape": "square", "mu_aniso": [1, 1]},
                     {"friction_shape": "ellipse"},
                     {"friction_shape": "ellipse", "mu_aniso": [1.0, 0.0]}):
        try:
            _minimal_writer(Path(tempfile.gettempdir()) / "x.vdtrace", tire=bad_tire)
            check(False, "tire=%r rejected" % (bad_tire,))
        except vt.TraceSchemaError:
            check(True, "tire=%r rejected" % (bad_tire,))

    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "ok.vdtrace"
        w = _minimal_writer(p)
        w.append(_sample(0))
        w.finalize()
        stripped = Path(td) / "notire.vdtrace"
        with zipfile.ZipFile(p) as src, zipfile.ZipFile(stripped, "w") as dst:
            for info in src.infolist():
                data = src.read(info.filename)
                if info.filename == "manifest.json":
                    m = json.loads(data.decode())
                    m.pop("tire")
                    data = json.dumps(m).encode()
                dst.writestr(info.filename, data)
        try:
            vt.TraceReader(stripped)
            check(False, "reader rejects a trace with no tire block")
        except vt.TraceSchemaError:
            check(True, "reader rejects a trace with no tire block")


def test_geometry_required():
    """Geometry is the renderer's only source for the body/wheel drawing."""
    try:
        _minimal_writer(Path(tempfile.gettempdir()) / "g.vdtrace",
                        geometry={"wheelbase_m": 2.7})
        check(False, "incomplete geometry rejected")
    except vt.TraceSchemaError as exc:
        check("track_m" in str(exc) and "steer_ratio" in str(exc),
              "incomplete geometry rejected, naming the missing keys")


def test_decimation_exact():
    """decimation=N keeps exactly 1 of every N samples (1 kHz -> 100 Hz)."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "dec.vdtrace"
        w = _minimal_writer(p, decimation=10)
        kept = sum(1 for i in range(1000) if w.append(_sample(i, dt=0.001)))
        w.finalize()
        check(kept == 100, "1000 steps at decimation=10 stored %d (expected 100)" % kept)
        with vt.TraceReader(p) as tr:
            check(tr.n_steps == 100, "manifest n_steps == 100")
            t = tr.channel("t")
            check(abs(float(t[1] - t[0]) - 0.01) < 1e-12,
                  "recorded sample period is 10x the step period")
            check(abs(float(t[0])) < 1e-12, "first step is kept, not dropped")


def test_overlay_contract():
    """Overlays need a kind tag; unknown kinds survive; content is untouched."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "ov.vdtrace"
        w = _minimal_writer(p)
        for i in range(5):
            w.append(_sample(i))
        w.finalize()

        for bad in ({"name": "x", "xy": []},                 # no kind
                    {"kind": "path2d"},                      # no name
                    {"kind": "", "name": "x"},
                    ["not", "a", "dict"]):
            try:
                vt.attach_overlay(p, bad)
                check(False, "overlay %r refused" % (bad,))
            except vt.TraceOverlayError:
                check(True, "overlay %r refused" % (bad,))

        ref = {"kind": "path2d", "name": "ref_path", "xy": [[0.0, 0.0], [1.0, 2.0]]}
        vt.attach_overlay(p, ref)
        future = {"kind": "some_future_kind", "name": "exotic", "payload": {"a": 1}}
        vt.attach_overlay(p, future)
        with vt.TraceReader(p) as tr:
            check(tr.overlay("ref_path") == ref, "path2d overlay stored verbatim")
            check(tr.overlay("exotic") == future,
                  "unknown kind is stored (renderers ignore it), not refused")
            check(tr.n_steps == 5, "channels survive an overlay attach")
            check(np.array_equal(tr.channel("pose"),
                                 np.array([_sample(i)["pose"] for i in range(5)])),
                  "channel data unchanged by overlay attach")
            check(len(tr.overlays(kind="path2d")) == 1, "overlays() filters by kind")

        # Re-attaching the same name replaces rather than duplicates.
        vt.attach_overlay(p, {"kind": "path2d", "name": "ref_path", "xy": [[9.0, 9.0]]})
        with vt.TraceReader(p) as tr:
            check(tr.overlay("ref_path")["xy"] == [[9.0, 9.0]], "overlay replace works")
            check(tr.overlay_names().count("ref_path") == 1, "no duplicate overlay member")
        try:
            vt.attach_overlay(p, ref, replace=False)
            check(False, "replace=False refuses a name clash")
        except vt.TraceOverlayError:
            check(True, "replace=False refuses a name clash")


def test_param_hash_scope():
    """The hash covers plant parameters and is stable, order-independent."""
    a = {"vehicle": {"mass": 2100.0, "wheelbase": 2.97}, "base_mu": 0.9}
    b = {"base_mu": 0.9, "vehicle": {"wheelbase": 2.97, "mass": 2100.0}}
    c = {"vehicle": {"mass": 2101.0, "wheelbase": 2.97}, "base_mu": 0.9}
    check(vt.param_hash(a) == vt.param_hash(b), "param_hash is key-order independent")
    check(vt.param_hash(a) != vt.param_hash(c), "param_hash changes with a plant param")
    check(vt.param_hash(a).startswith("sha256:"), "param_hash is tagged sha256:")


def test_utilization_formula():
    """The friction circle is the mu_aniso=[1,1] special case of the ellipse."""
    F = np.array([[[900.0, 1200.0, 5000.0]]])
    mu = np.array([[0.9]])
    circ = vt.utilization(F, mu, [1.0, 1.0])
    expect = np.hypot(900.0 / (0.9 * 5000.0), 1200.0 / (0.9 * 5000.0))
    check(abs(float(circ[0, 0]) - expect) < 1e-12, "circle case matches the closed form")
    ell = vt.utilization(F, mu, [1.0, 0.92])
    check(float(ell[0, 0]) > float(circ[0, 0]),
          "elliptic k_lat<1 reports higher utilization (%.3f > %.3f) — the reason a "
          "default [1,1] is refused" % (float(ell[0, 0]), float(circ[0, 0])))
    airborne = vt.utilization(np.array([[[0.0, 0.0, 0.0]]]), mu, [1.0, 1.0])
    check(np.isfinite(airborne).all(), "an airborne wheel (Fz=0) stays finite")


def main():
    test_golden_roundtrip()
    test_write_read_identity()
    test_schema_version_mismatch()
    test_tire_block_required()
    test_geometry_required()
    test_decimation_exact()
    test_overlay_contract()
    test_param_hash_scope()
    test_utilization_formula()
    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("\nall trace container checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
