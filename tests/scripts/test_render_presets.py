#!/usr/bin/env python3
"""Render-preset contract tests (11_trace_contract_spec.md §6.3).

A preset is renderer configuration, not trace schema. These checks pin the
four properties the contract actually constrains:

* ``overview`` is the only built-in; ``control``, ``tire_limit`` and
  ``road_contact`` are reserved names, not stubs
* resolution order is CLI > user preset file > built-in default
* a missing *optional* channel drops its panel and is not a failure
* panel positions and the BEV coordinate system are fixed, and nothing drawn
  may leave the frame — checked on the drawn figure, not by eye
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_preset as vp   # noqa: E402
import vdsim_render as vr   # noqa: E402
import vdsim_trace as vt    # noqa: E402

FIXTURE = REPO / "tests" / "fixtures" / "trace" / "golden_v0_2.vdtrace"

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


def _write_trace(path, channels=None, n=40):
    """Write a small synthetic trace carrying ``channels`` only."""
    names = channels or list(vt.CHANNEL_SPECS)
    w = vt.TraceWriter(
        path=path,
        geometry={"wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0},
        tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
        repro={"vdsim_version": "test", "git_sha": "x", "param_hash": "sha256:x",
               "seed": 1, "dt_s": 0.05, "run_id": "preset_test"},
        producer={"name": "test_render_presets", "version": "0"},
        role="plant",
        channels=names)
    full = {
        "t": 0.0, "pose": (0.0, 0.0, 0.0), "v_body": (12.0, 0.4),
        "yaw_rate": 0.05, "u_steer": 0.03, "u_fx": -800.0,
        "wheel_F": [(200.0, 120.0, 5000.0)] * 4, "wheel_mu": [0.9] * 4,
        "wheel_kappa": [0.01] * 4, "wheel_alpha": [0.02] * 4,
    }
    for i in range(n):
        t = i * 0.05
        s = dict(full)
        s["t"] = t
        s["pose"] = (12.0 * t, 0.5 * math.sin(0.6 * t), 0.02 * t)
        s["u_steer"] = 0.05 * math.sin(0.5 * t)
        w.append({k: s[k] for k in names})
    return w.finalize()


# --------------------------------------------------------------------------


def test_builtin_surface():
    """Only ``overview`` ships; the other contracted names stay unoccupied."""
    check(vp.builtin_names() == ("overview",),
          "overview is the only built-in preset (%s)" % (vp.builtin_names(),))
    for name in ("control", "tire_limit", "road_contact"):
        check(name not in vp.BUILTIN_PRESETS,
              "%r is not implemented as a built-in" % name)
        check(name in vp.RESERVED_PRESET_NAMES, "%r is reserved" % name)
    pre = vp.load_preset("overview")
    check([p["channel"] for p in pre["panels"]] == ["speed", "u_steer", "u_fx"],
          "overview declares speed + steer + long. command panels")
    check(pre["bev"]["waypoint"] is True and pre["bev"]["view_half_m"] is None,
          "overview keeps the optional reference path and the geometry window")
    check(vp.load_preset() == pre, "no preset argument resolves to overview")


def test_unknown_and_reserved_names_are_refused():
    """A typo must not silently render something else."""
    try:
        vp.load_preset("overvieww")
        check(False, "an unknown preset name is refused")
    except vp.PresetError as exc:
        check("overvieww" in str(exc), "unknown preset name refused: %s" % exc)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "road_contact.json"
        p.write_text(json.dumps({"panels": ["u_steer"]}), encoding="utf-8")
        try:
            vp.load_preset(p)
            check(False, "a user preset may not occupy a reserved name")
        except vp.PresetError as exc:
            check("reserved" in str(exc),
                  "a user preset named road_contact is refused: %s" % exc)


def test_user_preset_layers_over_the_builtin():
    """A file that changes one thing inherits the rest (``extends``)."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "mine.json"
        p.write_text(json.dumps({
            "name": "mine",
            "panels": [{"channel": "u_steer", "label": "delta",
                        "ylim": [-0.5, 0.5]}],
            "bev": {"waypoint": False, "view_half_m": 45.0},
        }), encoding="utf-8")
        pre = vp.load_preset(p)
        check(pre["name"] == "mine", "user preset keeps its name")
        check([q["channel"] for q in pre["panels"]] == ["u_steer"],
              "user panels replace the built-in list")
        check(pre["panels"][0]["ylim"] == (-0.5, 0.5), "declared ylim survives")
        check(pre["bev"]["waypoint"] is False and pre["bev"]["view_half_m"] == 45.0,
              "declared bev keys override the built-in")
        check(pre["bev"]["trail"] is True and pre["bev"]["colorbar"] is True,
              "undeclared bev keys inherit the built-in default")

        y = Path(td) / "mine.yaml"
        y.write_text("name: yamlpreset\npanels: [u_fx]\n", encoding="utf-8")
        try:
            py = vp.load_preset(y)
            check([q["channel"] for q in py["panels"]] == ["u_fx"],
                  "a YAML preset loads and normalizes bare channel names")
        except vp.PresetError as exc:
            check("PyYAML" in str(exc), "YAML preset without PyYAML fails clearly")

        for body, why in (
                ({"panelz": []}, "unknown top-level key"),
                ({"panels": [{"channel": "u_fx", "colour": "red"}]}, "unknown panel key"),
                ({"panels": [{"channel": "u_fx", "ylim": [1.0, 0.0]}]}, "inverted ylim"),
                ({"bev": {"view_half_m": -3.0}}, "negative view_half_m"),
                ({"extends": "tire_limit"}, "extends an unimplemented preset")):
            b = Path(td) / "bad.json"
            b.write_text(json.dumps(body), encoding="utf-8")
            try:
                vp.load_preset(b)
                check(False, "%s is refused" % why)
            except vp.PresetError:
                check(True, "%s is refused" % why)


def test_cli_overrides_beat_the_preset():
    """§6.3 order: CLI option > user preset > built-in default."""
    pre = vp.load_preset("overview")
    over = vp.apply_overrides(pre, panels=["u_steer"], view_half=60.0)
    check([p["channel"] for p in over["panels"]] == ["u_steer"],
          "--panels replaces the preset's panels")
    check(over["bev"]["view_half_m"] == 60.0, "--view-half replaces the preset window")
    check([p["channel"] for p in pre["panels"]] == ["speed", "u_steer", "u_fx"],
          "the input preset is not mutated by an override")
    untouched = vp.apply_overrides(pre)
    check(untouched == pre, "an unset CLI option overrides nothing")
    check(vp.PRECEDENCE == ("cli", "user-preset", "builtin"),
          "the documented order is cli > user-preset > builtin")


def test_overview_renders_the_declared_screen():
    """The preset drives the panels, and nothing leaves the frame."""
    check(FIXTURE.is_file(), "golden fixture is present")
    if not FIXTURE.is_file():
        return
    with tempfile.TemporaryDirectory() as td:
        res = vr.render(FIXTURE, out=Path(td) / "ov.gif", fps=5, stride=40,
                        quiet=True, preset="overview")
        check(res["preset"] == "overview", "result reports the preset used")
        check(res["panels"] == ["speed", "steer cmd", "long. cmd"],
              "overview plots speed / steer / long. command (%s)" % (res["panels"],))
        check(res["layout_violations"] == [],
              "nothing left the frame: %s" % (res["layout_violations"],))
        png = Path(res["png"])
        check(png.is_file() and png.stat().st_size > 5000,
              "preview png written (%d bytes)" % png.stat().st_size)
        from PIL import Image
        arr = np.asarray(Image.open(png).convert("L"), dtype=float)
        check(float(arr.std()) > 5.0,
              "preview is not a blank frame (std %.1f)" % float(arr.std()))


def test_missing_optional_channel_drops_its_panel():
    """A channel subset must not be a failure — the panel is simply omitted."""
    with tempfile.TemporaryDirectory() as td:
        p = _write_trace(Path(td) / "thin.vdtrace",
                         channels=["t", "pose", "u_steer"])
        res = vr.render(p, out=Path(td) / "thin.gif", fps=5, stride=10,
                        quiet=True, preset="overview")
        check(res["panels"] == ["steer cmd"],
              "speed and long. command are dropped, not failed (%s)" % (res["panels"],))
        check(res["layout_violations"] == [], "the reduced layout still holds the frame")

        empty = _write_trace(Path(td) / "bare.vdtrace", channels=["t", "pose"])
        res2 = vr.render(empty, out=Path(td) / "bare.gif", fps=5, stride=10,
                         quiet=True, preset="overview")
        check(res2["panels"] == [], "a trace with no plottable channel still renders")
        check(res2["layout_violations"] == [],
              "the BEV-only fallback layout also holds the frame")


def test_required_channel_is_an_error():
    """``required: true`` is the opt-in that turns omission back into a failure."""
    with tempfile.TemporaryDirectory() as td:
        bare = _write_trace(Path(td) / "bare2.vdtrace", channels=["t", "pose"])
        try:
            vr.LoadedTrace(bare, panels=[{"channel": "u_fx", "label": None,
                                          "ylim": None, "required": True}])
            check(False, "a required missing channel raises")
        except ValueError as exc:
            check("u_fx" in str(exc),
                  "a required missing channel raises, naming it: %s" % exc)


def test_panel_geometry_is_fixed():
    """Positions follow the panel count, never the data."""
    with tempfile.TemporaryDirectory() as td:
        a = _write_trace(Path(td) / "a.vdtrace")
        b = _write_trace(Path(td) / "b.vdtrace", n=90)
        boxes = []
        for path in (a, b):
            tr = vr.LoadedTrace(path, panels=vp.load_preset("overview")["panels"])
            fig, ax_bev, ax_series = vr._setup_axes(tr, (11.0, 5.6), 100)
            boxes.append((ax_bev.get_position().bounds,
                          [ax.get_position().bounds for ax in ax_series]))
            if path is a:
                vr._draw_static(tr, ax_bev, ax_series,
                                bev=vp.load_preset("overview")["bev"])
                check(ax_bev.get_aspect() == 1.0,
                      "BEV keeps an equal aspect, so X and Y share a scale")
                check((ax_bev.get_xlabel(), ax_bev.get_ylabel()) == ("X [m]", "Y [m]"),
                      "BEV axes are world X/Y in metres")
            import matplotlib.pyplot as plt
            plt.close(fig)
        check(boxes[0] == boxes[1],
              "two runs with the same panel count get identical panel boxes")
        bev_box, panel_boxes = boxes[0]
        check(len(panel_boxes) == 3, "overview lays out three panels")
        check(all(pb[0] > bev_box[0] + bev_box[2] - 1e-9 for pb in panel_boxes),
              "panels sit to the right of the BEV axis")
        heights = [round(pb[3], 6) for pb in panel_boxes]
        check(len(set(heights)) == 1, "panels share one height (%s)" % heights)
        ys = [pb[1] for pb in panel_boxes]
        check(ys == sorted(ys, reverse=True),
              "panels are stacked top-to-bottom in declaration order")


def test_layout_guard_catches_an_escape():
    """The frame check is not vacuous: a deliberately misplaced text is caught."""
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(4.0, 3.0), dpi=80)
    ax = fig.add_subplot(1, 1, 1)
    check(vr.layout_violations(fig) == [], "a plain figure reports no violation")
    ax.text(1.9, 1.6, "escaped", transform=ax.transAxes)
    bad = vr.layout_violations(fig)
    check(len(bad) >= 1, "text pushed past the frame is reported (%d)" % len(bad))
    check(any("text" in b for b in bad), "the report names the offending element")
    plt.close(fig)


def main():
    test_builtin_surface()
    test_unknown_and_reserved_names_are_refused()
    test_user_preset_layers_over_the_builtin()
    test_cli_overrides_beat_the_preset()
    test_overview_renders_the_declared_screen()
    test_missing_optional_channel_drops_its_panel()
    test_required_channel_is_an_error()
    test_panel_geometry_is_fixed()
    test_layout_guard_catches_an_escape()
    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("\nall render-preset checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
