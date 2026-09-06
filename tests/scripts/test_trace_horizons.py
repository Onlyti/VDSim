#!/usr/bin/env python3
"""Renderer tests for the waypoint / target / prediction split.

The three curves are routinely conflated under the word "reference". They are
not the same object:

* **waypoint** — the whole route, time-invariant, one per scenario. Static.
* **target** — the reference horizon fed to the controller at *this* step.
  Time-varying: its point spacing follows the planned speed.
* **prediction** — the horizon the controller solved for at this step.
  Time-varying; garbage or NaN when the solve failed.

So the assertions here are mostly about *change*: a target that does not move
between frames, or a prediction that is silently held over a failed solve,
would look plausible on screen and be wrong. The sidecar is synthesised here
rather than produced by a controller run, so the test needs no acados.
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_render as vr  # noqa: E402
import vdsim_trace as vt   # noqa: E402

FIXTURE = REPO / "tests" / "fixtures" / "trace" / "golden_v0_1.vdtrace"
FAILURES = []
#: Horizon length of the synthetic sidecar (the real one is the MPC's N+1).
NH = 21


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


# --------------------------------------------------------------------------
# fixture construction
# --------------------------------------------------------------------------
def make_sidecar(dst: Path, t, pose, fail_steps=(), nan_on_fail=False):
    """Write a sidecar shaped like the harness writes one.

    ``target`` is laid ahead of the vehicle with a spacing that grows over the
    run, so a renderer that froze it would be caught. ``prediction`` starts on
    the vehicle, which is the frame check the harness performs.
    """
    t = np.asarray(t, dtype=float)
    K = len(t)
    tgt = np.zeros((K, NH, 2))
    pred = np.zeros((K, NH, 2))
    status = np.zeros(K)
    for k in range(K):
        x, y, yaw = pose[k]
        # spacing grows with k: "the planned speed changed"
        step = 0.5 + 2.0 * k / max(K - 1, 1)
        s = step * np.arange(NH)
        tgt[k, :, 0] = x + s * np.cos(yaw) + 1.0 * np.sin(yaw)
        tgt[k, :, 1] = y + s * np.sin(yaw) - 1.0 * np.cos(yaw)
        pred[k, :, 0] = x + s * np.cos(yaw)
        pred[k, :, 1] = y + s * np.sin(yaw)
    for k in fail_steps:
        status[k] = 4.0
        if nan_on_fail:
            pred[k, :, :] = np.nan
    np.savez_compressed(
        dst, t=t, tgt_XY=tgt, tgt_v=np.full((K, NH), 20.0), pred_XY=pred,
        status=status, solve_ms=np.full(K, 3.5),
        n_fail_cum=np.cumsum(status != 0.0).astype(float))
    return dst


def staged_trace(tmp: Path, fail_steps=(), nan_on_fail=False, with_event=False):
    """Copy the golden trace into ``tmp`` and put a sidecar next to it."""
    dst = tmp / "run.vdtrace"
    dst.write_bytes(FIXTURE.read_bytes())
    with vt.TraceReader(dst) as tr:
        t = np.asarray(tr.channel("t"))
        pose = np.asarray(tr.channel("pose"))
    make_sidecar(tmp / "run.qp.npz", t, pose, fail_steps, nan_on_fail)
    if with_event and len(fail_steps):
        vt.attach_overlay(str(dst), {"kind": "event", "name": "qp_fail",
                                     "t": [float(t[k]) for k in fail_steps]})
    return dst


# --------------------------------------------------------------------------
def test_sidecar_is_optional():
    """A trace with no sidecar renders exactly as before: no horizon artists."""
    tr = vr.LoadedTrace(FIXTURE)
    check(not tr.has_sidecar(), "golden fixture has no sidecar (auto-detect is quiet)")
    spec = vr.frame_spec(tr, 5)
    check(spec["tgt_xy"] is None and spec["pred_xy"] is None,
          "frame_spec reports no horizons rather than raising")
    check(spec["status"] is None, "no sidecar means no solver status")
    check(tr.first_fail_frame() is None, "no sidecar means no first-fail frame")


def test_sidecar_loads_and_aligns():
    """Sidecar steps are matched to trace samples on time, not on index."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        trace = staged_trace(tmp)
        tr = vr.LoadedTrace(trace)
        check(tr.has_sidecar(), "sidecar found next to the trace")
        check(len(tr.sc_map) == len(tr.t),
              "one sidecar step selected per recorded sample")
        # trace and sidecar share a clock here, so the map must be the identity
        check(np.array_equal(tr.sc_map, np.arange(len(tr.t))),
              "identical clocks map 1:1")

        # halve the sidecar rate: every other trace sample must reuse a step
        t_half = np.asarray(tr.t)[::2]
        pose_half = np.asarray(tr.pose)[::2]
        make_sidecar(tmp / "run.qp.npz", t_half, pose_half)
        tr2 = vr.LoadedTrace(trace)
        # Assert the property, not an index formula: exact-midpoint ties break
        # on float rounding of the sample clock and either neighbour is right.
        dt_sc = float(np.median(np.diff(t_half)))
        err = np.abs(t_half[tr2.sc_map] - np.asarray(tr2.t))
        check(err.max() <= dt_sc / 2 + 1e-9,
              "a half-rate sidecar is matched within half a step "
              "(max %.4f s <= %.4f s)" % (err.max(), dt_sc / 2))
        check(len(np.unique(tr2.sc_map)) == len(t_half),
              "every sidecar step is used, none truncated away")

        explicit = vr.LoadedTrace(trace, sidecar=tmp / "run.qp.npz")
        check(explicit.has_sidecar(), "--sidecar path is honoured")
        try:
            vr.LoadedTrace(trace, sidecar=tmp / "nope.qp.npz")
            check(False, "an explicit missing sidecar raises")
        except FileNotFoundError:
            check(True, "an explicit missing sidecar raises")


def test_three_curves_are_distinct():
    """waypoint / target / prediction differ in kind, not only in colour."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        tr = vr.LoadedTrace(staged_trace(tmp))
        a, b = vr.frame_spec(tr, 2), vr.frame_spec(tr, len(tr.t) - 3)

        wps = tr.path2d_overlays()
        check(len(wps) >= 1, "waypoint arrives as a static path2d overlay")

        check(not np.allclose(a["tgt_xy"], b["tgt_xy"]),
              "target moves between frames (it is not the waypoint)")
        check(not np.allclose(a["pred_xy"], b["pred_xy"]),
              "prediction moves between frames")
        check(not np.allclose(a["tgt_xy"], a["pred_xy"]),
              "target and prediction are different curves in the same frame")

        # the point of drawing target with markers: spacing carries meaning
        d_a = np.linalg.norm(np.diff(a["tgt_xy"], axis=0), axis=1).mean()
        d_b = np.linalg.norm(np.diff(b["tgt_xy"], axis=0), axis=1).mean()
        check(d_b > d_a * 1.5,
              "target point spacing tracks the planned speed (%.2f -> %.2f m)"
              % (d_a, d_b))


def test_failed_steps_are_kept_and_flagged():
    """A failed solve is shown as it was recorded, not held or interpolated."""
    fails = (10, 11, 12, 40)
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        tr = vr.LoadedTrace(staged_trace(tmp, fail_steps=fails, nan_on_fail=True))
        check(tr.first_fail_frame() == fails[0],
              "first_fail_frame points at the first failed step (%d)" % fails[0])
        check(len(tr.qp_fail_times()) == len(fails),
              "every failed step contributes a panel marker")

        bad = vr.frame_spec(tr, fails[0])
        good = vr.frame_spec(tr, fails[0] - 1)
        check(bad["status"] == 4 and good["status"] == 0,
              "solver status is carried per frame")
        check(np.isnan(bad["pred_xy"]).all(),
              "a NaN prediction stays NaN (no carry-over from the last good step)")
        check(not np.isnan(good["pred_xy"]).any(),
              "the preceding good step is untouched")
        check(any("FAIL" in line for line in bad["hud"]),
              "HUD says FAIL on a failed frame")
        check(all("FAIL" not in line for line in good["hud"]),
              "HUD does not say FAIL on a solved frame")


def test_draw_switches_style_on_failure():
    """The prediction line turns red and thick exactly on failed frames."""
    import matplotlib.pyplot as plt
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        tr = vr.LoadedTrace(staged_trace(tmp, fail_steps=(7,)))
        fig, ax_bev, ax_series = vr._setup_axes(tr, (11.0, 5.6), 100)
        artists = vr._draw_static(tr, ax_bev, ax_series)
        check(artists["target"] is not None and artists["prediction"] is not None,
              "horizon artists exist when a sidecar is present")

        labels = [t.get_text() for t in ax_bev.get_legend().get_texts()]
        check(vr.WAYPOINT_LABEL in labels,
              "legend calls the global route 'waypoint', not 'reference_path'")
        check(vr.TARGET_LABEL in labels and vr.PRED_LABEL in labels,
              "legend names the MPC input and output horizons: %s" % labels)

        vr.draw_frame(artists, ax_bev, vr.frame_spec(tr, 6))
        ok_color = artists["prediction"].get_color()
        ok_lw = artists["prediction"].get_linewidth()
        check(not artists["qpfail"].get_visible(), "no banner on a solved frame")

        vr.draw_frame(artists, ax_bev, vr.frame_spec(tr, 7))
        check(artists["prediction"].get_color() == vr.PRED_FAIL_COLOR
              and ok_color == vr.PRED_COLOR,
              "prediction switches to the failure colour")
        check(artists["prediction"].get_linewidth() > ok_lw,
              "prediction thickens on failure (%.1f -> %.1f)"
              % (ok_lw, artists["prediction"].get_linewidth()))
        check(artists["qpfail"].get_visible()
              and "status=4" in artists["qpfail"].get_text(),
              "banner names the status: %r" % artists["qpfail"].get_text())
        check(artists["n_qp_fail"] == 1,
              "the command panel got one failure marker")

        # ... and switches back, so the style is per-frame and not sticky
        vr.draw_frame(artists, ax_bev, vr.frame_spec(tr, 8))
        check(artists["prediction"].get_color() == vr.PRED_COLOR
              and not artists["qpfail"].get_visible(),
              "failure styling is per-frame, not latched")
        plt.close(fig)


def test_event_overlay_is_preferred_over_status():
    """A recorded qp_fail overlay drives the markers; status is the fallback."""
    import matplotlib.pyplot as plt
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        trace = staged_trace(tmp, fail_steps=(3, 4, 5), with_event=True)
        tr = vr.LoadedTrace(trace)
        check(len(tr.event_overlays()) >= 1, "qp_fail event overlay recorded")
        fig, ax_bev, ax_series = vr._setup_axes(tr, (11.0, 5.6), 100)
        artists = vr._draw_static(tr, ax_bev, ax_series)
        check(artists["n_qp_fail"] == 3,
              "markers come from the overlay without double-counting the status")
        plt.close(fig)


def test_preview_frame_selection():
    """--preview-frame first-fail moves the PNG, and always writes the evidence."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        trace = staged_trace(tmp, fail_steps=(30,))
        res = vr.render(trace, out=tmp / "a.gif", fps=8, stride=20,
                        preview_frame="first-fail", quiet=True)
        check(res["firstfail_png"] is not None and res["firstfail_png"].is_file(),
              "first-fail PNG written")
        check(res["n_qp_fail"] == 1, "render reports the failure count")

        # a clean run must not invent one, and must not raise
        clean = tmp / "clean.vdtrace"
        clean.write_bytes(FIXTURE.read_bytes())
        res2 = vr.render(clean, out=tmp / "b.gif", fps=8, stride=20,
                         preview_frame="first-fail", quiet=True)
        check(res2["firstfail_png"] is None and res2["png"].is_file(),
              "a run with no failure falls back to util-peak, no phantom PNG")

        try:
            vr.render(clean, out=tmp / "c.gif", preview_frame="nonsense", quiet=True)
            check(False, "an unknown --preview-frame is rejected")
        except ValueError:
            check(True, "an unknown --preview-frame is rejected")


def test_cli_renders_with_sidecar():
    """End to end through the CLI, which is how the meeting artefacts are made."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        trace = staged_trace(tmp, fail_steps=(12,))
        rc = vr.main([str(trace), "--out", str(tmp / "cli.gif"), "--fps", "8",
                      "--stride", "20", "--preview-frame", "first-fail"])
        check(rc == 0, "CLI exits 0")
        check((tmp / "cli.gif").is_file(), "CLI wrote the GIF")
        check((tmp / "cli_firstfail.png").is_file(),
              "CLI wrote the first-fail PNG next to the GIF")


def test_window_must_be_widened_for_horizons():
    """The default window is far shorter than an MPC horizon, and says so."""
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        tr = vr.LoadedTrace(staged_trace(tmp))
        spacing_max = 0.5 + 2.0  # the synthetic sidecar's widest step
        check(tr.horizon_reach > tr.view_half,
              "horizon (%.1f m) outruns the geometry window (%.1f m) — the "
              "reason --view-half exists" % (tr.horizon_reach, tr.view_half))
        check(tr.horizon_reach >= spacing_max * (NH - 1),
              "horizon_reach measures the farthest horizon point, not the mean")

        # default: window unchanged, so the geometry contract still holds
        spec = vr.frame_spec(tr, 4)
        span = spec["xlim"][1] - spec["xlim"][0]
        check(abs(span - 2.0 * tr.view_half) < 1e-9,
              "without --view-half the window is still the geometry window")

        res = vr.render(staged_trace(tmp), out=tmp / "w.gif", fps=8, stride=20,
                        view_half=60.0, quiet=True)
        check(abs(res["view_half_m"] - 60.0 * (1.0 + vr.VIEW_MARGIN_FRAC)) < 1e-9,
              "--view-half is honoured (plus the fixed margin)")
        check(abs(res["horizon_reach_m"] - tr.horizon_reach) < 1e-6,
              "render reports the horizon reach so the operator can size it")


def test_hud_survives_a_degenerate_utilization():
    """mu*Fz = 0 must not print a 60-character number into the HUD box."""
    check(vr._fmt_util(float("inf")) == " n/a", "inf utilization prints n/a")
    check(vr._fmt_util(float("nan")) == " n/a", "NaN utilization prints n/a")
    check(vr._fmt_util(2.5e94) == ">>99", "an absurd value is named, not printed")
    check(vr._fmt_util(0.83) == "0.83", "a normal value is unchanged")
    check(all(len(vr._fmt_util(u)) == 4
              for u in (0.0, 1.0, float("inf"), 2.5e94)),
          "every rendering is 4 characters wide, so the HUD box cannot overrun")


def test_failures_collapse_into_spans():
    """A run that fails everywhere is one band, not one line per step."""
    check(vr.merge_spans([], 0.02) == [], "no failures -> no spans")
    one = vr.merge_spans([1.0, 1.02, 1.04], 0.02)
    check(len(one) == 1, "consecutive failures merge into one span")
    check(abs(one[0][0] - 0.99) < 1e-9 and abs(one[0][1] - 1.05) < 1e-9,
          "the span covers the failed samples plus half a period either side")
    three = vr.merge_spans([1.0, 2.0, 3.0], 0.02)
    check(len(three) == 3, "scattered failures stay separate marks")
    check(len(vr.merge_spans(np.arange(0, 10, 0.02), 0.02)) == 1,
          "500 consecutive failures collapse to a single span")


if __name__ == "__main__":
    test_sidecar_is_optional()
    test_sidecar_loads_and_aligns()
    test_three_curves_are_distinct()
    test_failed_steps_are_kept_and_flagged()
    test_draw_switches_style_on_failure()
    test_event_overlay_is_preferred_over_status()
    test_preview_frame_selection()
    test_cli_renders_with_sidecar()
    test_window_must_be_widened_for_horizons()
    test_hud_survives_a_degenerate_utilization()
    test_failures_collapse_into_spans()
    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        raise SystemExit(1)
    print("\nall horizon-renderer checks passed")
