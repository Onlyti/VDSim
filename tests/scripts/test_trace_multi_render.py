#!/usr/bin/env python3
"""Overlay-renderer tests — several traces, one view.

The single-run renderer can lean on a sample index; an overlay cannot. Runs
are recorded independently, so the contract this file pins down is the *time*
base: every run is interpolated at the frame time, a run that has ended holds
its last pose and stops extending its trail, and the fit camera holds every
run at a constant, square scale. Those are the failures a visual check misses
— a run silently sampled by index would look plausible and be wrong.

Uses only stdlib + numpy + matplotlib: no compiled core, no simulation.
"""
from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_render as vr  # noqa: E402
import vdsim_trace as vt   # noqa: E402

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


# --------------------------------------------------------------------------
# synthetic runs (the fixtures for this file are written, not committed:
# an overlay needs several runs that differ in dt and duration)
# --------------------------------------------------------------------------

def write_run(path, n, dt, y0=0.0, v=15.0, yaw0=0.0, yaw_rate=0.0,
              util_scale=0.5, run_id="run"):
    """Write one straight-ish synthetic ``.vdtrace``.

    :param path: output path.
    :param n: number of samples.
    :param dt: sample period [s] — deliberately different between runs.
    :param y0: lateral offset [m], so overlaid runs are visually separable.
    :param v: longitudinal speed [m/s].
    :param yaw0: initial heading [rad].
    :param yaw_rate: constant yaw rate [rad/s].
    :param util_scale: scales the tyre forces, hence the utilization.
    :param run_id: manifest ``repro.run_id``.
    :returns: the written path.
    """
    w = vt.TraceWriter(
        path=path,
        geometry={"wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0,
                  "body_length_m": 4.6, "body_width_m": 1.9},
        tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
        repro={"vdsim_version": "test", "git_sha": "x", "param_hash": "sha256:x",
               "seed": 1, "dt_s": dt, "run_id": run_id},
        producer={"name": "test_trace_multi_render", "version": "0"},
        role="plant")
    x, y, yaw = 0.0, y0, yaw0
    for i in range(n):
        t = i * dt
        fx = 4000.0 * util_scale * math.sin(0.3 * t)
        w.append({
            "t": t, "pose": (x, y, _wrap(yaw)), "v_body": (v, 0.0),
            "yaw_rate": yaw_rate, "u_steer": 0.05 * math.sin(0.4 * t),
            "u_fx": fx,
            "wheel_F": [(fx * 0.25, fx * 0.1, 5000.0)] * 4,
            "wheel_mu": [0.9] * 4, "wheel_kappa": [0.01] * 4,
            "wheel_alpha": [0.02] * 4,
        })
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += yaw_rate * dt
    return w.finalize()


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def three_runs(td):
    """Three runs that differ in dt, duration, offset and grip."""
    d = Path(td)
    return [
        write_run(d / "base.vdtrace", n=101, dt=0.02, y0=0.0,
                  util_scale=0.4, run_id="base"),
        write_run(d / "tuned.vdtrace", n=41, dt=0.05, y0=1.5, yaw_rate=0.02,
                  util_scale=0.9, run_id="tuned"),
        write_run(d / "short.vdtrace", n=21, dt=0.05, y0=-1.5, yaw_rate=-0.05,
                  util_scale=1.4, run_id="short"),
    ]


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------

def test_common_time_base():
    """Frame k is a time: the base spans every run and steps at speed/fps."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20, speed=1.0)

        durations = [float(r.t[-1] - r.t[0]) for r in scene.runs]
        check(abs(scene.t1 - scene.t0 - max(durations)) < 1e-9,
              "time base spans the longest run (%.2f s of %s)"
              % (scene.t1 - scene.t0, ["%.2f" % d for d in durations]))
        check(abs(float(np.diff(scene.times).max()) - 0.05) < 1e-12,
              "frame step is speed/fps = 0.050 s regardless of any run's dt")
        check(scene.n_frames == int(math.floor((scene.t1 - scene.t0) / 0.05 + 1e-9)) + 1,
              "frame count %d matches span/step" % scene.n_frames)

        fast = vr.MultiScene(paths, fps=20, speed=2.0)
        check(fast.n_frames < scene.n_frames,
              "speed=2.0 halves the frame count (%d < %d)"
              % (fast.n_frames, scene.n_frames))


def test_resampling_matches_the_records():
    """Sampled at a run's own timestamps, interpolation is the identity."""
    with tempfile.TemporaryDirectory() as td:
        path = write_run(Path(td) / "one.vdtrace", n=51, dt=0.02, yaw_rate=0.3)
        run = vr.LoadedTrace(path)
        track = vr.MultiScene.resample_run(run, run.t)
        check(float(np.abs(track["x"] - run.pose[:, 0]).max()) < 1e-12,
              "x at recorded times == recorded x")
        check(float(np.abs(track["y"] - run.pose[:, 1]).max()) < 1e-12,
              "y at recorded times == recorded y")
        dyaw = np.abs((track["yaw"] - run.pose[:, 2] + math.pi)
                      % (2.0 * math.pi) - math.pi)
        check(float(dyaw.max()) < 1e-12, "yaw at recorded times == recorded yaw")
        check(float(np.abs(track["util"] - run.util).max()) < 1e-12,
              "utilization at recorded times == recorded utilization")


def test_yaw_interpolation_crosses_the_seam():
    """Yaw near ±π interpolates the short way, not 2π backwards."""
    with tempfile.TemporaryDirectory() as td:
        # 0.6 rad/s for 21 samples at 0.05 s starting at 2.9 rad: crosses +pi.
        path = write_run(Path(td) / "seam.vdtrace", n=21, dt=0.05,
                         yaw0=2.9, yaw_rate=0.6)
        run = vr.LoadedTrace(path)
        crossed = bool(np.any(np.abs(np.diff(run.pose[:, 2])) > math.pi))
        check(crossed, "fixture actually wraps across ±pi")

        mid = 0.5 * (run.t[:-1] + run.t[1:])
        ours = vr.MultiScene.resample_run(run, mid)["yaw"]
        truth = np.unwrap(run.pose[:, 2])
        truth_mid = 0.5 * (truth[:-1] + truth[1:])
        err = np.abs((ours - truth_mid + math.pi) % (2.0 * math.pi) - math.pi)
        check(float(err.max()) < 1e-9,
              "midpoint yaw error %.2e rad (short way round)" % float(err.max()))

        naive = np.interp(mid, run.t, run.pose[:, 2])
        naive_err = np.abs((naive - truth_mid + math.pi) % (2.0 * math.pi) - math.pi)
        check(float(naive_err.max()) > 1.0,
              "naive wrapped interpolation would err by %.2f rad — the "
              "unwrap is load-bearing" % float(naive_err.max()))


def test_ended_run_holds_and_stops_its_trail():
    """A finished run freezes, fades and stops extending its path."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20)
        short = scene.runs[2]
        t_end = float(short.t[-1])

        k_live = int(np.searchsorted(scene.times, 0.5 * t_end))
        k_dead = scene.n_frames - 1
        live = vr.multi_frame_spec(scene, k_live)["runs"][2]
        dead = vr.multi_frame_spec(scene, k_dead)["runs"][2]

        check(live["active"] and not dead["active"],
              "run 'short' is active at t=%.2f and inactive at t=%.2f"
              % (scene.times[k_live], scene.times[k_dead]))
        check(abs(dead["x"] - float(short.pose[-1, 0])) < 1e-9
              and abs(dead["y"] - float(short.pose[-1, 1])) < 1e-9,
              "ended run holds its final pose instead of extrapolating")
        check(abs(dead["alpha_scale"] - vr.ENDED_ALPHA_SCALE) < 1e-12,
              "ended run is faded to alpha_scale %.2f" % vr.ENDED_ALPHA_SCALE)
        check(len(dead["trail_x"]) < k_dead + 1,
              "ended run's trail stops growing (%d points at frame %d)"
              % (len(dead["trail_x"]), k_dead))

        alive = vr.multi_frame_spec(scene, k_live)["runs"][0]
        check(len(alive["trail_x"]) == k_live + 1,
              "a running run's trail is exactly the frames it has driven")


def test_fit_window_holds_every_run():
    """The default camera is static, square, and contains all routes."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20, follow="fit")
        specs = [vr.multi_frame_spec(scene, k)
                 for k in (0, scene.n_frames // 2, scene.n_frames - 1)]
        spans = {(round(s["xlim"][1] - s["xlim"][0], 9),
                  round(s["ylim"][1] - s["ylim"][0], 9)) for s in specs}
        check(len(spans) == 1, "fit window scale is constant across frames")
        sx, sy = spans.pop()
        check(abs(sx - sy) < 1e-9, "fit window is square (%.2f x %.2f m)" % (sx, sy))

        (x0, x1), (y0, y1) = scene.xlim, scene.ylim
        inside = all(x0 <= float(r.pose[:, 0].min()) and float(r.pose[:, 0].max()) <= x1
                     and y0 <= float(r.pose[:, 1].min()) and float(r.pose[:, 1].max()) <= y1
                     for r in scene.runs)
        check(inside, "every run's whole route is inside the fit window")


def test_follow_tracks_one_run():
    """`--follow k` re-centres on run k with that run's geometric window."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20, follow=1)
        half = vr.VIEW_HALF_WHEELBASES * float(scene.runs[1].geometry["wheelbase_m"])
        for k in (0, scene.n_frames // 2, scene.n_frames - 1):
            spec = vr.multi_frame_spec(scene, k)
            cx = 0.5 * (spec["xlim"][0] + spec["xlim"][1])
            cy = 0.5 * (spec["ylim"][0] + spec["ylim"][1])
            check(abs(cx - spec["runs"][1]["x"]) < 1e-9
                  and abs(cy - spec["runs"][1]["y"]) < 1e-9,
                  "frame %d is centred on run 1" % k)
            check(abs((spec["xlim"][1] - spec["xlim"][0]) - 2.0 * half) < 1e-9,
                  "frame %d window span comes from run 1 geometry" % k)
        bad = 0
        for value in ("7", -1, "left"):
            try:
                vr.MultiScene(paths, follow=value)
            except ValueError:
                bad += 1
        check(bad == 3, "out-of-range / non-numeric --follow is rejected")


def test_identity_is_per_run_and_alpha_is_configurable():
    """Colour and label identify a run; alpha is what makes overlap readable."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20)
        check(scene.labels == ["base", "tuned", "short"],
              "labels default to manifest run ids: %s" % (scene.labels,))
        check(len(set(scene.colors)) == 3,
              "each run gets a distinct colour: %s" % (scene.colors,))

        named = vr.MultiScene(paths, fps=20, labels=["a", "b", "c"],
                              colors=["#000000", "#111111", "#222222"])
        check(named.labels == ["a", "b", "c"] and named.colors[2] == "#222222",
              "explicit labels/colours win")
        wrong = 0
        for kw in ({"labels": ["a"]}, {"colors": ["#000000"]}):
            try:
                vr.MultiScene(paths, **kw)
            except ValueError:
                wrong += 1
        check(wrong == 2, "a short labels/colours list is rejected, not cycled")

        spec = vr.multi_frame_spec(scene, scene.n_frames // 2)
        check(len({r["color"] for r in spec["runs"]}) == 3,
              "the frame description carries one colour per run")
        check(all(0.0 < r["alpha_scale"] <= 1.0 for r in spec["runs"]),
              "alpha_scale stays in (0, 1]")
        check(vr.BODY_ALPHA < 1.0,
              "default body alpha %.2f lets overlapping vehicles show through"
              % vr.BODY_ALPHA)


def test_spec_is_pure():
    """Two calls for the same frame agree, and nothing in the scene moves."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20)
        k = scene.n_frames // 3
        before = [t["x"].copy() for t in scene.tracks]
        a = vr.multi_frame_spec(scene, k)
        b = vr.multi_frame_spec(scene, k)
        same = all(abs(ra["x"] - rb["x"]) < 1e-12 and ra["body"] == rb["body"]
                   for ra, rb in zip(a["runs"], b["runs"]))
        check(same, "multi_frame_spec is repeatable for the same frame")
        check(all(np.array_equal(x0, t["x"]) for x0, t in zip(before, scene.tracks)),
              "multi_frame_spec does not mutate the scene")


def test_preview_frame_is_the_divergence():
    """The still is taken where the runs are furthest apart."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        scene = vr.MultiScene(paths, fps=20)
        peak = scene.preview_frame()
        spreads = [scene.spread(k) for k in range(scene.n_frames)]
        check(abs(spreads[peak] - max(spreads)) < 1e-12,
              "preview frame %d has the maximum spread (%.2f m)"
              % (peak, spreads[peak]))
        check(max(spreads) > 1.0,
              "the synthetic runs do diverge (%.2f m), so the choice matters"
              % max(spreads))


def test_render_multi_writes_outputs():
    """End to end: three traces in, one GIF + preview PNG out."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        out = Path(td) / "compare.gif"
        res = vr.render_multi(paths, out=out, fps=10, dpi=70, alpha=0.5,
                              quiet=True)
        check(out.is_file() and out.stat().st_size > 5000,
              "GIF written (%d bytes)" % (out.stat().st_size if out.is_file() else 0))
        check(res["png"].is_file() and res["png"].stat().st_size > 3000,
              "preview PNG written (%d bytes)"
              % (res["png"].stat().st_size if res["png"].is_file() else 0))
        check(res["runs"] == 3 and res["frames"] > 10,
              "result reports %d runs / %d frames" % (res["runs"], res["frames"]))
        check(res["max_spread_m"] > 1.0,
              "result reports the divergence used for the still (%.2f m)"
              % res["max_spread_m"])


def test_cli_dispatches_on_trace_count():
    """One trace renders single-run; several overlay — same entry point."""
    with tempfile.TemporaryDirectory() as td:
        paths = three_runs(td)
        single = Path(td) / "single.gif"
        rc1 = vr.main([str(paths[0]), "--out", str(single), "--fps", "8",
                       "--dpi", "60"])
        multi = Path(td) / "multi.gif"
        rc2 = vr.main([str(p) for p in paths[:2]]
                      + ["--out", str(multi), "--fps", "8", "--dpi", "60",
                         "--labels", "a,b", "--colors", "#101010,#202020",
                         "--alpha", "0.4", "--follow", "0", "--no-ghost"])
        check(rc1 == 0 and single.is_file(), "single-trace CLI still renders")
        check(rc2 == 0 and multi.is_file(), "multi-trace CLI renders the overlay")
        check(vr._split_list("a, b ,c") == ["a", "b", "c"]
              and vr._split_list("") is None,
              "--labels/--colors parsing splits and trims")
        rejected = False
        try:
            vr.MultiScene(paths[:1])
        except ValueError:
            rejected = True
        check(rejected, "overlay mode refuses a single trace")


def main():
    test_common_time_base()
    test_resampling_matches_the_records()
    test_yaw_interpolation_crosses_the_seam()
    test_ended_run_holds_and_stops_its_trail()
    test_fit_window_holds_every_run()
    test_follow_tracks_one_run()
    test_identity_is_per_run_and_alpha_is_configurable()
    test_spec_is_pure()
    test_preview_frame_is_the_divergence()
    test_render_multi_writes_outputs()
    test_cli_dispatches_on_trace_count()
    if FAILURES:
        print("\n%d check(s) failed" % len(FAILURES))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("\nall overlay-renderer checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
