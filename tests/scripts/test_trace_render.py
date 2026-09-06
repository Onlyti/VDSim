#!/usr/bin/env python3
"""Renderer tests (DoD 5-7) — fixture in, GIF/PNG out, no simulation.

The important assertion is not "a picture appeared" but that the BEV window is
a function of manifest ``geometry``. Autoscale would rescale the animation
frame to frame and the failure is invisible in a visual check, so the axis
limits are asserted numerically against the geometry block.
"""
from __future__ import annotations

import math
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_render as vr  # noqa: E402
import vdsim_trace as vt   # noqa: E402

FIXTURE = REPO / "tests" / "fixtures" / "trace" / "golden_v0_1.vdtrace"
FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


def test_axes_come_from_geometry():
    """Window extent is set by manifest geometry, not by the data range."""
    tr = vr.LoadedTrace(FIXTURE)
    wb = float(tr.geometry["wheelbase_m"])
    expect_span = 2.0 * vr.VIEW_HALF_WHEELBASES * wb

    spans = []
    for i in (0, tr.n_frames // 2, len(tr.t) - 1):
        spec = vr.frame_spec(tr, int(i))
        sx = spec["xlim"][1] - spec["xlim"][0]
        sy = spec["ylim"][1] - spec["ylim"][0]
        spans.append((sx, sy))
        check(abs(sx - expect_span) < 1e-9 and abs(sy - expect_span) < 1e-9,
              "frame %d window span %.3f m == 2*%.1f*wheelbase (%.3f m)"
              % (i, sx, vr.VIEW_HALF_WHEELBASES, expect_span))
        cx = 0.5 * (spec["xlim"][0] + spec["xlim"][1])
        check(abs(cx - float(tr.pose[i, 0])) < 1e-9,
              "frame %d window is centred on the vehicle (tracking camera)" % i)
    check(len({(round(a, 9), round(b, 9)) for a, b in spans}) == 1,
          "window scale is constant across frames (no autoscale drift)")

    # A different geometry must move the window — proving the dependency is real.
    doubled = dict(tr.geometry)
    doubled["wheelbase_m"] = 2.0 * wb
    check(abs(vr.view_half_m(doubled) - 2.0 * vr.view_half_m(tr.geometry)) < 1e-9,
          "doubling the wheelbase doubles the window (geometry actually drives it)")


def test_frame_spec_is_pure():
    """frame_spec has no state: same input, same output, any order."""
    tr = vr.LoadedTrace(FIXTURE)
    idx = [0, 137, 42, 137, 0]
    specs = [vr.frame_spec(tr, i) for i in idx]

    def same(a, b):
        if a["hud"] != b["hud"] or a["t"] != b["t"]:
            return False
        for key in ("xlim", "ylim", "body", "wheels", "wheel_util", "arrow"):
            if not np.array_equal(np.asarray(a[key], dtype=float),
                                  np.asarray(b[key], dtype=float)):
                return False
        return True

    check(same(specs[0], specs[4]), "frame 0 renders identically when revisited")
    check(same(specs[1], specs[3]), "frame 137 renders identically out of order")


def test_eight_screen_items():
    """All eight required screen elements are actually produced (DoD 6)."""
    tr = vr.LoadedTrace(FIXTURE)
    peak = int(np.argmax(tr.util.max(axis=1)))
    spec = vr.frame_spec(tr, peak)

    check(len(tr.path2d_overlays()) >= 1, "1. reference path overlay present (dashed)")
    check(len(spec["trail_x"]) == peak + 1 and len(spec["trail_y"]) == peak + 1,
          "2. driven path grows to the current frame")
    bl, bw = vr.body_size(tr.geometry)
    poly = spec["body"]
    edges = [math.hypot(poly[(j + 1) % 4][0] - poly[j][0],
                        poly[(j + 1) % 4][1] - poly[j][1]) for j in range(4)]
    check(len(poly) == 4
          and abs(max(edges) - bl) < 1e-6 and abs(min(edges) - bw) < 1e-6,
          "3. body rectangle edges are %0.2f x %0.2f m, sized from geometry"
          % (bl, bw))

    delta = float(tr.steer[peak])
    yaw = float(tr.pose[peak, 2])
    front, rear = spec["wheels"][0], spec["wheels"][2]

    def heading(rect):
        dx = 0.5 * ((rect[0][0] + rect[1][0]) - (rect[2][0] + rect[3][0]))
        dy = 0.5 * ((rect[0][1] + rect[1][1]) - (rect[2][1] + rect[3][1]))
        return math.atan2(dy, dx)

    check(len(spec["wheels"]) == 4, "4a. four wheels are drawn")
    check(abs(_wrap(heading(front) - heading(rear)) - delta) < 1e-6,
          "4b. front wheels lead the rear by the recorded steer (%.4f rad)" % delta)
    check(abs(delta) > 1e-3, "4c. the fixture actually steers, so 4b is not vacuous")

    ax_, ay_ = spec["arrow"][2], spec["arrow"][3]
    speed = math.hypot(*tr.v_body[peak])
    check(math.hypot(ax_, ay_) > 0.0,
          "5. velocity arrow has length %.2f m at %.1f m/s" % (math.hypot(ax_, ay_), speed))
    check(len(spec["hud"]) >= 5 and "util max" in spec["hud"][-1],
          "6. HUD carries time / speed / slip / yaw-rate / steer / utilization")
    check(len(spec["wheel_util"]) == 4 and max(spec["wheel_util"]) > 0.0,
          "7. four per-wheel utilization values drive the colour (max %.2f)"
          % max(spec["wheel_util"]))
    names = [n for n, _u, _a in tr.series]
    check("u_steer" in names and "u_fx" in names,
          "8a. command panel plots u_steer and u_fx (selected from the manifest)")
    check(spec["t"] == float(tr.t[peak]), "8b. time cursor follows the frame")


def test_series_selection_is_manifest_driven():
    """The panel reads the manifest channel list instead of a hardcoded one."""
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "partial.vdtrace"
        w = vt.TraceWriter(
            path=p,
            geometry={"wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0},
            tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
            repro={"vdsim_version": "t", "git_sha": "x", "param_hash": "sha256:x",
                   "seed": 0, "dt_s": 0.05, "run_id": "partial"},
            channels=["t", "pose", "yaw_rate", "u_steer"])
        for i in range(20):
            w.append({"t": i * 0.05, "pose": (i * 0.5, 0.0, 0.0),
                      "yaw_rate": 0.0, "u_steer": 0.01})
        w.finalize()
        tr = vr.LoadedTrace(p)
        names = [n for n, _u, _a in tr.series]
        check(names == ["u_steer"],
              "a trace without u_fx plots only what it carries (%s)" % names)
        spec = vr.frame_spec(tr, 3)
        check(len(spec["wheel_util"]) == 4 and max(spec["wheel_util"]) == 0.0,
              "a trace without wheel channels renders with zero utilization, not a crash")


def test_angular_series_are_displayed_in_degrees():
    """Plot-facing angles use degrees without changing recorded SI arrays."""
    raw = np.asarray([-math.pi, 0.0, 0.5 * math.pi])
    unit, shown = vr.series_for_display("rad", raw)
    check(unit == "deg" and np.allclose(shown, [-180.0, 0.0, 90.0]),
          "angle series converts rad to deg")
    check(np.allclose(raw, [-math.pi, 0.0, 0.5 * math.pi]),
          "display conversion does not mutate recorded radians")

    rate_unit, rate = vr.series_for_display("rad/s", [math.pi])
    check(rate_unit == "deg/s" and np.allclose(rate, [180.0]),
          "angular-rate series converts rad/s to deg/s")

    linear_unit, linear = vr.series_for_display("N", [1.0, 2.0])
    check(linear_unit == "N" and np.allclose(linear, [1.0, 2.0]),
          "non-angular series keeps its unit and values")

    tr = vr.LoadedTrace(FIXTURE)
    fig, ax_bev, axes = vr._setup_axes(tr, (11.0, 5.6), 100)
    vr._draw_static(tr, ax_bev, axes)
    steer_axis = axes[[n for n, _u, _a in tr.series].index("u_steer")]
    check(steer_axis.get_ylabel() == "u_steer [deg]"
          and np.allclose(steer_axis.lines[0].get_ydata(), np.degrees(tr.steer)),
          "single-run command panel plots steering in deg")
    vr.plt.close(fig)

    scene = vr.MultiScene([FIXTURE, FIXTURE], fps=10, labels=["a", "b"])
    fig, ax_bev, axes = vr._setup_multi_axes(scene, (11.0, 5.6), 100)
    vr._draw_multi_static(scene, ax_bev, axes, vr.BODY_ALPHA, vr.PATH_ALPHA)
    steer_axis = axes[scene.series_names.index("u_steer")]
    check(steer_axis.get_ylabel() == "u_steer [deg]"
          and all(np.allclose(line.get_ydata(), np.degrees(run.steer))
                  for line, run in zip(steer_axis.lines, scene.runs)),
          "multi-run command panel plots every steering series in deg")
    vr.plt.close(fig)


def test_t6_instrument_panel():
    """Steering-wheel ratio and speed gauge are pure, fixed-screen artists."""
    spec = vr.instrument_spec(0.1, 15.0, 10.0, 40.0)
    check(abs(spec["steering_wheel_deg"] - math.degrees(1.5)) < 1e-12,
          "T6 steering-wheel dial applies manifest steer_ratio")
    check(abs(spec["speed_kmh"] - 36.0) < 1e-12,
          "T6 speedometer converts m/s to km/h")
    check(abs(math.degrees(spec["speed_needle_rad"]) - (-18.0)) < 1e-12,
          "T6 speed needle uses the fixed 0..40 km/h scale")

    tr = vr.LoadedTrace(FIXTURE)
    i = min(20, len(tr.t) - 1)
    frame = vr.frame_spec(tr, i)
    expected = math.degrees(float(tr.steer[i]) * float(tr.geometry["steer_ratio"]))
    check(abs(frame["instruments"]["steering_wheel_deg"] - expected) < 1e-12,
          "frame instrument uses recorded road steer times geometry steer_ratio")

    fig, ax_bev, axes = vr._setup_axes(tr, (11.0, 5.6), 100)
    artists = vr._draw_static(tr, ax_bev, axes)
    vr.draw_frame(artists, ax_bev, frame)
    check(len(artists["steer_spokes"].get_xdata()) == 9,
          "steering-wheel dial draws three rotating spokes")
    check("km/h" in artists["speed_text"].get_text()
          and ":1" in artists["steer_text"].get_text(),
          "instrument readouts expose speed unit and steering ratio")
    vr.plt.close(fig)


def test_t6_layout_non_overlap():
    """T6 gauges have zero geometric overlap with the BEV, HUD and panels."""
    tr = vr.LoadedTrace(FIXTURE)
    layout = vr.single_layout_spec(len(tr.series))

    def intersects(a, b):
        return (min(a[0] + a[2], b[0] + b[2]) > max(a[0], b[0])
                and min(a[1] + a[3], b[1] + b[3]) > max(a[1], b[1]))

    instrument_rects = (layout["steering"], layout["speed"])
    check(not intersects(*instrument_rects),
          "steering-wheel and speedometer layout rectangles do not overlap")
    for name, rect in (("BEV", layout["bev"]),
                       *(('series-%d' % k, r)
                         for k, r in enumerate(layout["series"]))):
        check(all(not intersects(rect, gauge) for gauge in instrument_rects),
              "instrument strip does not overlap %s rectangle" % name)

    fig, ax_bev, axes = vr._setup_axes(tr, (11.0, 5.6), 100)
    instrument_axes = fig._vdsim_instrument_axes
    artists = vr._draw_static(tr, ax_bev, axes)
    vr.draw_frame(artists, ax_bev, vr.frame_spec(tr, 0))
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    to_figure = fig.transFigure.inverted()
    actual_bev = ax_bev.get_position().bounds
    actual_panels = [ax.get_position().bounds for ax in axes]
    actual_gauges = [ax.get_position().bounds for ax in instrument_axes.values()]
    check(all(not intersects(gauge, actual_bev) for gauge in actual_gauges),
          "actual instrument axes do not overlap the BEV/HUD axes")
    check(all(not intersects(gauge, panel)
              for gauge in actual_gauges for panel in actual_panels),
          "actual instrument axes do not overlap command panels")
    bev_tight = ax_bev.get_tightbbox(renderer).transformed(to_figure).bounds
    gauge_tight = [ax.get_tightbbox(renderer).transformed(to_figure).bounds
                   for ax in instrument_axes.values()]
    check(all(not intersects(gauge, bev_tight) for gauge in gauge_tight),
          "instrument decorations do not overlap BEV labels")
    hud_bbox = artists["hud"].get_window_extent(renderer).transformed(to_figure).bounds
    check(all(not intersects(gauge, hud_bbox) for gauge in gauge_tight),
          "instrument decorations do not overlap HUD bbox")
    vr.plt.close(fig)

def test_render_from_fixture_only():
    """DoD 5 + 7: fixture in, GIF + PNG out, wall-clock measured."""
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "render.gif"
        t0 = time.time()
        res = vr.render(FIXTURE, out=out, fps=10, stride=20, quiet=True)
        wall = time.time() - t0
        check(res["gif"].is_file() and res["gif"].stat().st_size > 20000,
              "GIF written (%d KiB)" % (res["gif"].stat().st_size // 1024))
        check(res["png"].is_file() and res["png"].stat().st_size > 20000,
              "preview PNG written (%d KiB)" % (res["png"].stat().st_size // 1024))
        check(res["frames"] > 1, "animation has %d frames" % res["frames"])
        print("render wall-clock: %.2fs for %.1fs of trace (%d frames)"
              % (wall, res["trace_duration_s"], res["frames"]))

        from PIL import Image
        im = Image.open(str(res["png"])).convert("RGB")
        arr = np.asarray(im)
        check(arr.shape[0] > 300 and arr.shape[1] > 600,
              "preview is %dx%d px" % (arr.shape[1], arr.shape[0]))
        check(len(np.unique(arr.reshape(-1, 3), axis=0)) > 50,
              "preview is not a blank/uniform frame")


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    if not FIXTURE.is_file():
        print("FAIL: golden fixture missing: %s" % FIXTURE)
        return 1
    test_axes_come_from_geometry()
    test_frame_spec_is_pure()
    test_eight_screen_items()
    test_series_selection_is_manifest_driven()
    test_angular_series_are_displayed_in_degrees()
    test_t6_instrument_panel()
    test_t6_layout_non_overlap()
    test_render_from_fixture_only()
    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("\nall renderer checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
