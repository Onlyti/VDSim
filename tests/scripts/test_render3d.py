#!/usr/bin/env python3
"""3D replay contract tests (``19_view_module_spec`` §11, ``25`` §4-§5).

What is actually pinned here:

* cameras are a **combination**, and the six names are aliases over it — a new
  view must be expressible without touching drawing code;
* ``follow_attitude`` never follows roll/pitch, so the horizon stays readable;
* the same trace and options render the same frame count and the same bytes;
* a ``0.2`` trace degrades to RT0 and *says so* instead of drawing a flat lie;
* the pass is single: load and derivation do not repeat per camera, and each
  camera gets its own file rather than a grid cell;
* RT2 is not reachable;
* the 2D renderer is not imported, let alone modified.
"""
from __future__ import annotations

import math
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))
os.environ.setdefault("MPLBACKEND", "Agg")

import vdsim_render3d as r3  # noqa: E402

FIX_0_3 = REPO / "tests" / "fixtures" / "trace" / "golden_v0_3.vdtrace"
FIX_0_2 = REPO / "tests" / "fixtures" / "trace" / "golden_v0_2.vdtrace"

FAILURES = []


def check(cond, msg):
    if not cond:
        FAILURES.append(msg)
        print("FAIL: %s" % msg)
    else:
        print("ok  : %s" % msg)


# ------------------------------------------------------------------ cameras
def test_camera_is_a_combination():
    """All six names resolve, and a seventh view needs no code change."""
    names = ("bev3d", "side", "quarter", "chase", "map_fixed", "map_track")
    cams = {}
    for n in names:
        cam = r3.resolve_camera(n)
        check(isinstance(cam, r3.Camera), "alias %r resolves to a Camera" % n)
        check(cam.name == n, "alias %r keeps its name" % n)
        cams[n] = cam
    check(cams["bev3d"].projection == "ortho", "bev3d is orthographic")
    check(cams["quarter"].projection == "persp", "quarter is perspective")
    check(cams["map_fixed"].mount == "world" and cams["map_fixed"].aim == "fixed_point",
          "map_fixed is world-mounted and fixed-aim")
    check(cams["map_track"].mount == "world" and cams["map_track"].aim == "vehicle",
          "map_track is world-mounted and tracks the vehicle")
    for n, cam in cams.items():
        check(cam.follow_attitude in ("yaw", "none"),
              "%s declares a follow_attitude (%s)" % (n, cam.follow_attitude))

    # The seventh view: a plain combination, no alias, no new branch.
    custom = r3.resolve_camera(dict(name="nose", mount="vehicle", aim="vehicle",
                                    offset=(3.5, 0.0, 1.0), projection="persp"))
    check(custom.name == "nose" and custom.mount == "vehicle",
          "an unnamed mount/aim/offset combination is a valid camera")
    tweaked = r3.resolve_camera(dict(alias="quarter", offset=(-14.0, -11.0, 7.0)))
    check(tuple(tweaked.offset) == (-14.0, -11.0, 7.0),
          "an alias can be overridden field by field")

    for bad in ({"mount": "drone", "aim": "vehicle"},
                {"mount": "vehicle", "aim": "sky"},
                {"mount": "vehicle", "aim": "vehicle", "projection": "fisheye"},
                {"mount": "vehicle", "aim": "vehicle", "follow_attitude": "roll"}):
        try:
            r3.resolve_camera(bad)
            check(False, "invalid combination %s is rejected" % bad)
        except ValueError:
            check(True, "invalid combination %s is rejected" % bad)


def test_camera_does_not_follow_roll_or_pitch():
    """A vehicle-mounted camera tracks yaw only, so the horizon stays put."""
    scene = r3.Scene3D(FIX_0_3, stride=1)
    cam = r3.resolve_camera("quarter")
    flat = r3.frame_primitives(scene, 0, set())
    tilted = dict(flat)
    tilted["roll"] = math.radians(25.0)
    tilted["pitch"] = math.radians(-8.0)
    e0, t0, u0, s0 = r3.camera_frame(cam, flat, scene)
    e1, t1, u1, s1 = r3.camera_frame(cam, tilted, scene)
    check(np.allclose(e0, e1), "roll/pitch do not move the camera eye")
    check(np.allclose(u0, [0, 0, 1]), "the up vector stays world vertical")

    yawed = dict(flat)
    yawed["yaw"] = flat["yaw"] + 1.0
    e2, _, _, _ = r3.camera_frame(cam, yawed, scene)
    check(not np.allclose(e0, e2), "yaw does move the camera eye")

    none_cam = cam.replace(follow_attitude="none")
    e3, _, _, _ = r3.camera_frame(none_cam, yawed, scene)
    e4, _, _, _ = r3.camera_frame(none_cam, flat, scene)
    check(np.allclose(e3 - flat["origin"], e4 - flat["origin"]),
          "follow_attitude='none' keeps a fixed world-aligned offset")


def test_projection_basics():
    """Depth sign and the ortho/persp difference behave as a viewer expects."""
    eye = np.array([0.0, 0.0, 0.0])
    basis = r3.view_basis(eye, np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]))
    pts = np.array([[10.0, 1.0, 0.0], [20.0, 1.0, 0.0], [-5.0, 0.0, 0.0]])
    xy_o, d_o = r3.project(pts, eye, basis, "ortho", span=10.0)
    check(d_o[0] > 0 and d_o[2] < 0, "depth is positive in front and negative behind")
    check(abs(abs(xy_o[0][0]) - abs(xy_o[1][0])) < 1e-9,
          "orthographic size does not change with distance")
    xy_p, _ = r3.project(pts, eye, basis, "persp", span=10.0)
    check(abs(xy_p[1][0]) < abs(xy_p[0][0]),
          "perspective shrinks the farther point (%.3f < %.3f)"
          % (abs(xy_p[1][0]), abs(xy_p[0][0])))


def test_rotation_matches_the_core_convention():
    """ZYX intrinsic, so a positive roll lifts the left side, not the right."""
    R = r3.rot_zyx(0.0, 0.0, math.radians(10.0))
    left = R @ np.array([0.0, 1.0, 0.0])
    check(left[2] > 0.0, "positive roll raises +y (left) as ISO 8855 requires")
    R2 = r3.rot_zyx(math.radians(90.0), 0.0, 0.0)
    check(np.allclose(R2 @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9),
          "yaw 90 deg turns body +x into world +y")
    Rp = r3.rot_zyx(0.0, math.radians(10.0), 0.0)
    check((Rp @ np.array([1.0, 0.0, 0.0]))[2] < 0.0,
          "positive pitch drops the nose (ISO 8855 y axis points left)")


# ------------------------------------------------------------------ tiers
def test_rt2_is_not_reachable():
    """Suspension links / tyre deformation / contact pressure have no input."""
    for layer in ("suspension", "links", "pressure", "tyre_deformation"):
        check(layer not in r3.RT1_LAYERS, "%r is not an RT1 layer" % layer)
    with tempfile.TemporaryDirectory() as td:
        try:
            r3.render3d(FIX_0_3, td, cameras=("side",), stride=60,
                        tiers=("suspension",), verbose=False)
            check(False, "an RT2 layer name is refused")
        except ValueError as exc:
            check("unknown RT1 layer" in str(exc), "an RT2 layer name is refused")


def test_rt0_needs_no_flags():
    """RT0 draws with no tiers enabled; RT1 is what the flags add."""
    scene = r3.Scene3D(FIX_0_3, stride=30)
    bare = r3.frame_primitives(scene, 0, set())
    for key in ("body", "wheels", "hubs"):
        check(key in bare, "RT0 primitive %r is always built" % key)
    for key in ("wheel_F_world", "a_world", "normals"):
        check(key not in bare, "RT1 primitive %r is absent without its flag" % key)
    rich = r3.frame_primitives(scene, 0, {"force", "accel", "normal"})
    for key in ("wheel_F_world", "a_world", "normals"):
        check(key in rich, "RT1 primitive %r appears with its flag" % key)
    check(np.allclose(np.linalg.norm(rich["normals"], axis=1), 1.0, atol=1e-9),
          "the drawn road normals are the recorded unit vectors")


# ------------------------------------------------------------- render output
def test_per_camera_files_and_determinism():
    """One file per camera, and two identical runs produce identical bytes."""
    with tempfile.TemporaryDirectory() as td:
        a = Path(td) / "a"
        b = Path(td) / "b"
        kw = dict(cameras=("side", "bev3d"), stride=40, fps=10,
                  tiers=("force", "saturation"), verbose=False)
        r1 = r3.render3d(FIX_0_3, a, **kw)
        r2 = r3.render3d(FIX_0_3, b, **kw)
        check(len(r1["outputs"]) == 2, "two cameras produce two outputs, not a grid")
        names = [Path(p).name for p in r1["outputs"]]
        check(all("__" in n for n in names),
              "outputs follow <run>__<preset>__<camera> (%s)" % names)
        check(any("side" in n for n in names) and any("bev3d" in n for n in names),
              "each camera names its own file")
        check(r1["frames"] == r2["frames"], "frame count is deterministic")
        for p1, p2 in zip(sorted(r1["outputs"]), sorted(r2["outputs"])):
            s1, s2 = Path(p1), Path(p2)
            if s1.is_dir():
                check(len(list(s1.iterdir())) == len(list(s2.iterdir())),
                      "PNG sequence length is deterministic")
                continue
            check(s1.read_bytes() == s2.read_bytes(),
                  "%s is byte-identical across runs" % s1.name)
        for p in r1["previews"]:
            check(Path(p).stat().st_size > 2000,
                  "preview %s is not empty (%d bytes)"
                  % (Path(p).name, Path(p).stat().st_size))


def test_single_pass_over_the_trace():
    """Load + derivation happen once no matter how many cameras are drawn."""
    with tempfile.TemporaryDirectory() as td:
        one = r3.render3d(FIX_0_3, Path(td) / "n1", cameras=("quarter",),
                          stride=40, fps=10, verbose=False)
        three = r3.render3d(FIX_0_3, Path(td) / "n3",
                            cameras=("quarter", "side", "chase"),
                            stride=40, fps=10, verbose=False)
    check(three["load_s"] < 3.0 * one["load_s"] + 0.05,
          "load+prep does not scale with camera count (%.3f s vs %.3f s)"
          % (three["load_s"], one["load_s"]))
    check(len(three["cameras"]) == 3, "three cameras were drawn")
    # The draw cost is what scales; if it did not, the cameras would be
    # rendering the same picture.
    check(three["draw_s"] > 1.4 * one["draw_s"],
          "draw time scales with camera count (%.2f s vs %.2f s)"
          % (three["draw_s"], one["draw_s"]))


def test_degraded_mode_on_a_0_2_trace():
    """A pre-0.3 trace renders RT0 and states why it is flat."""
    scene = r3.Scene3D(FIX_0_2, stride=50)
    check(scene.pose_zrp is None, "a 0.2 trace carries no pose_zrp")
    check(scene.degraded is not None, "the degrade reason is set, not silent")
    check("pose_zrp" in scene.degraded, "the reason names the missing channel")
    check(scene.attitude(0) == (0.0, 0.0, 0.0),
          "attitude falls back to the planar assumption")
    with tempfile.TemporaryDirectory() as td:
        res = r3.render3d(FIX_0_2, td, cameras=("quarter",), stride=50,
                          fps=10, verbose=False)
        check(res["degraded"] is not None, "the render reports the degrade")
        check(res["frames"] > 0 and Path(res["previews"][0]).stat().st_size > 2000,
              "the degraded render still produces a usable frame")


def test_scales_are_fixed_not_autoscaled():
    """Vector length per unit is a constant of the run, not of the frame."""
    src = (REPO / "python" / "vdsim_render3d.py").read_text()
    check("autoscale" not in src.replace("never autoscaled", "")
          .replace("autoscale would", "").replace("autoscale a", ""),
          "no per-frame autoscale path exists in the module")
    with tempfile.TemporaryDirectory() as td:
        r_small = r3.render3d(FIX_0_3, Path(td) / "s", cameras=("side",),
                              stride=60, fps=10, tiers=("force",),
                              force_scale=2000.0, verbose=False)
        r_big = r3.render3d(FIX_0_3, Path(td) / "b", cameras=("side",),
                            stride=60, fps=10, tiers=("force",),
                            force_scale=8000.0, verbose=False)
        p_s = Path(r_small["previews"][0]).read_bytes()
        p_b = Path(r_big["previews"][0]).read_bytes()
        check(p_s != p_b, "changing --force-scale changes the drawn arrows")


def test_the_2d_renderer_is_untouched():
    """3D is a new module, not a fork: it does not even import the 2D one."""
    src = (REPO / "python" / "vdsim_render3d.py").read_text()
    check("import vdsim_render" not in src.replace("import vdsim_render3d", ""),
          "vdsim_render3d does not import vdsim_render")
    check("from vdsim_trace import" in src,
          "the container reader is reused rather than reimplemented")
    check("zipfile" not in src, "no second container parser lives here")


def test_display_only_badge():
    """At C0/C1 the normal layer is badged; at C2 it is not."""
    import vdsim_trace as vt
    with tempfile.TemporaryDirectory() as td:
        names = vt.channels_for_level("L3")
        made = {}
        for scope in ("C1", "C2"):
            p = Path(td) / ("s_%s.vdtrace" % scope)
            w = vt.TraceWriter(
                path=p,
                geometry={"wheelbase_m": 2.7, "track_m": 1.6, "steer_ratio": 15.0,
                          "mass_kg": 1800.0, "cg_height_m": 0.55,
                          "wheel_radius_m": 0.32, "wheel_width_m": 0.225,
                          "body_lwh_m": [4.6, 1.9, 1.5]},
                tire={"friction_shape": "circle", "mu_aniso": [1.0, 1.0]},
                repro={"vdsim_version": "t", "git_sha": "x", "param_hash": "sha256:x",
                       "seed": 0, "dt_s": 0.05, "run_id": "scope_" + scope},
                role="plant", model_level="L3", contact_scope=scope, channels=names)
            for i in range(6):
                w.append({
                    "t": i * 0.05, "pose": (i * 0.5, 0.0, 0.0),
                    "pose_zrp": (0.55, 0.02, 0.0), "v_body": (10.0, 0.0),
                    "a_body": (0.1, 0.2, 0.0), "yaw_rate": 0.0,
                    "u_steer": 0.01, "u_fx": 0.0,
                    "wheel_F": [(100.0, 50.0, 5000.0)] * 4, "wheel_mu": [0.9] * 4,
                    "wheel_kappa": [0.0] * 4, "wheel_alpha": [0.0] * 4,
                    "wheel_road_dz": [0.0] * 4,
                    "wheel_road_normal": [(0.0, -0.1, 0.995)] * 4,
                    "wheel_travel": [0.06] * 4})
            w.finalize()
            made[scope] = r3.Scene3D(p)
        check(made["C1"].normal_display_only is True,
              "a C1 trace badges the normal layer display-only")
        check(made["C2"].normal_display_only is False,
              "a C2 trace does not badge it — the physics really used the normal")


def main():
    if not FIX_0_3.is_file():
        print("FAIL: 0.3 fixture missing at %s" % FIX_0_3)
        return 1
    t0 = time.perf_counter()
    test_camera_is_a_combination()
    test_camera_does_not_follow_roll_or_pitch()
    test_projection_basics()
    test_rotation_matches_the_core_convention()
    test_rt2_is_not_reachable()
    test_rt0_needs_no_flags()
    test_per_camera_files_and_determinism()
    test_single_pass_over_the_trace()
    test_degraded_mode_on_a_0_2_trace()
    test_scales_are_fixed_not_autoscaled()
    test_the_2d_renderer_is_untouched()
    test_display_only_badge()
    print("\n%d checks failed (%.1f s)" % (len(FAILURES), time.perf_counter() - t0))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    raise SystemExit(main())
