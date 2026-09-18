"""vdsim_render3d — headless 3D replay of a ``.vdtrace`` (V7 / RT0 + RT1).

One trace file is the whole input: geometry, attitude, contacts and forces all
come from the container, so a replay needs no simulator and no scenario.

    python -m vdsim_render3d run.vdtrace --cameras quarter,side --out out/run

Design boundaries this module keeps (``19_view_module_spec`` §11,
``11_trace_contract_spec`` §3.2.1):

* **It is not a 3D fork of the 2D renderer.** ``vdsim_render`` is untouched and
  its output for a given trace + preset stays byte-identical; the two share the
  container reader in :mod:`vdsim_trace`, not drawing code.
* **Cameras are a combination, not a list.** ``mount`` x ``aim`` x ``offset`` x
  ``projection`` x ``follow_attitude`` is the contract; ``quarter``, ``chase``
  and the rest are aliases over it. Adding a seventh view is a table row.
* **Vector scales are fixed constants**, taken from the CLI. A per-frame
  autoscale would make the same arrow length mean different things at different
  times, which is a lie an animation tells very convincingly.
* **What the physics did not use is labelled.** At ``contact_scope`` C0/C1 the
  core never reads the road normal's x/y, so the normal layer carries a
  ``display-only`` badge. Without it a tilted road drawn over flat physics
  passes review.
* **RT2 (suspension links, tyre deformation, contact pressure) is not drawn.**
  Not "later" — the trace has no input for it.

Backend: the 3D projection here is explicit numpy plus matplotlib 2-D polygon
drawing with painter-ordered faces. No extra dependency, and in particular no
OpenGL/EGL, which is what makes this run on a headless box at all.
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vdsim_trace import TraceReader  # noqa: E402

#: Wheel order of every per-wheel channel.
WHEELS = ("FL", "FR", "RL", "RR")

#: Utilization above which a wheel is drawn as saturated (matches the 2D
#: renderer's threshold so the two views cannot disagree about the same wheel).
UTIL_WARN = 0.8

#: Sides of the polygon that stands in for a wheel. 16 reads as round at every
#: camera distance used here and keeps the face count per frame under 200.
WHEEL_SIDES = 16


# =========================================================================
# camera contract (19 §11.2)
# =========================================================================
class Camera:
    """A camera as the combination the contract defines, not as a name.

    :param name: alias used in filenames and the HUD.
    :param mount: ``"vehicle"`` (rides with the car) or ``"world"`` (bolted to
        the map).
    :param aim: ``"vehicle"`` (always looks at the car) or ``"fixed_point"``.
    :param offset: camera position in the mount frame [m]. For a vehicle mount
        this is body-frame; for a world mount it is a world position.
    :param projection: ``"ortho"`` or ``"persp"``.
    :param follow_attitude: ``"yaw"`` (default) or ``"none"``. Roll and pitch
        are deliberately not followed: if the camera rolled with the body the
        horizon would stay level and the attitude would become unreadable —
        which is the one thing a 3D replay exists to show.
    :param aim_point: world point to look at when ``aim == "fixed_point"``.
    :param fit: world mount only — place and scale the camera so the whole
        trajectory fits one frame.
    :param span: ortho half-width [m]; ``None`` derives it from the vehicle or
        the trajectory.
    """

    __slots__ = ("name", "mount", "aim", "offset", "projection",
                 "follow_attitude", "aim_point", "fit", "span")

    def __init__(self, name, mount, aim, offset, projection="persp",
                 follow_attitude="yaw", aim_point=(0.0, 0.0, 0.0),
                 fit=False, span=None):
        if mount not in ("vehicle", "world"):
            raise ValueError("camera.mount must be 'vehicle' or 'world', got %r" % (mount,))
        if aim not in ("vehicle", "fixed_point"):
            raise ValueError("camera.aim must be 'vehicle' or 'fixed_point', got %r" % (aim,))
        if projection not in ("ortho", "persp"):
            raise ValueError("camera.projection must be 'ortho' or 'persp', got %r"
                             % (projection,))
        if follow_attitude not in ("yaw", "none"):
            raise ValueError("camera.follow_attitude must be 'yaw' or 'none', got %r"
                             % (follow_attitude,))
        self.name = name
        self.mount = mount
        self.aim = aim
        self.offset = np.asarray(offset, dtype=float)
        self.projection = projection
        self.follow_attitude = follow_attitude
        self.aim_point = np.asarray(aim_point, dtype=float)
        self.fit = bool(fit)
        self.span = span

    def replace(self, **kw):
        """Return a copy with fields overridden (cameras stay immutable)."""
        args = dict(name=self.name, mount=self.mount, aim=self.aim,
                    offset=self.offset, projection=self.projection,
                    follow_attitude=self.follow_attitude,
                    aim_point=self.aim_point, fit=self.fit, span=self.span)
        args.update(kw)
        return Camera(**args)

    def __repr__(self):
        return ("Camera(%s mount=%s aim=%s offset=%s proj=%s follow=%s)"
                % (self.name, self.mount, self.aim,
                   np.round(self.offset, 2).tolist(), self.projection,
                   self.follow_attitude))


#: The six requested views, expressed as rows of the combination above.
#: A seventh view is a row here; it is never an ``if`` in the drawing code.
CAMERA_ALIASES = {
    "bev3d":     dict(mount="vehicle", aim="vehicle", offset=(0.0, 0.0, 28.0),
                      projection="ortho", follow_attitude="yaw"),
    "side":      dict(mount="vehicle", aim="vehicle", offset=(0.0, -14.0, 1.2),
                      projection="ortho", follow_attitude="yaw"),
    "quarter":   dict(mount="vehicle", aim="vehicle", offset=(-9.0, -7.5, 4.5),
                      projection="persp", follow_attitude="yaw"),
    "chase":     dict(mount="vehicle", aim="vehicle", offset=(-11.0, 0.0, 3.6),
                      projection="persp", follow_attitude="yaw"),
    "map_fixed": dict(mount="world", aim="fixed_point", offset=None,
                      projection="persp", follow_attitude="none", fit=True),
    "map_track": dict(mount="world", aim="vehicle", offset=None,
                      projection="persp", follow_attitude="none", fit=True),
}


def resolve_camera(spec) -> Camera:
    """Build a :class:`Camera` from an alias name or an explicit dict.

    ``"quarter"`` and ``{"mount": "vehicle", "aim": "vehicle", ...}`` are the
    same kind of thing; the alias table only supplies defaults.

    :param spec: alias name, or a dict of camera fields (``name`` optional).
    :returns: the resolved camera.
    :raises ValueError: on an unknown alias or an invalid combination.
    """
    if isinstance(spec, Camera):
        return spec
    if isinstance(spec, str):
        if spec not in CAMERA_ALIASES:
            raise ValueError("unknown camera %r; aliases are %s, or pass an "
                             "explicit mount/aim/offset/projection combination"
                             % (spec, sorted(CAMERA_ALIASES)))
        fields = dict(CAMERA_ALIASES[spec])
        fields["name"] = spec
    else:
        fields = dict(spec)
        base = fields.pop("alias", None)
        if base is not None:
            merged = dict(CAMERA_ALIASES[base])
            merged.update(fields)
            fields = merged
            fields.setdefault("name", base)
        fields.setdefault("name", "custom")
    fields.setdefault("offset", (-9.0, -7.5, 4.5))
    if fields.get("offset") is None:
        fields["offset"] = (0.0, 0.0, 0.0)
    return Camera(**fields)


# =========================================================================
# geometry helpers
# =========================================================================
def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Body->world rotation for ISO 8855 ZYX intrinsic Euler angles.

    Matches the core convention ``q = Rz(yaw) * Ry(pitch) * Rx(roll)``; using a
    different order here would tilt the drawn body the wrong way on a banked
    corner, which looks plausible and is wrong.
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def box_faces(center, half, R=None):
    """Six quad faces of an axis-aligned box, optionally rotated.

    :param center: box centre in the parent frame.
    :param half: half extents ``(hx, hy, hz)``.
    :param R: 3x3 rotation applied about ``center``.
    :returns: ``(6, 4, 3)`` array of face vertices.
    """
    hx, hy, hz = half
    v = np.array([
        [-hx, -hy, -hz], [+hx, -hy, -hz], [+hx, +hy, -hz], [-hx, +hy, -hz],
        [-hx, -hy, +hz], [+hx, -hy, +hz], [+hx, +hy, +hz], [-hx, +hy, +hz],
    ])
    if R is not None:
        v = v @ R.T
    v = v + np.asarray(center, dtype=float)
    idx = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
           (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return np.array([[v[i] for i in f] for f in idx])


def wheel_faces(center, R, radius, width, sides=WHEEL_SIDES):
    """Two discs and the tread band of a wheel cylinder, in the parent frame.

    The cylinder axis is the wheel's y axis, so ``R`` must already carry the
    steer angle (and the body attitude) for the wheel to point where the tyre
    actually points.
    """
    ang = np.linspace(0.0, 2.0 * math.pi, sides, endpoint=False)
    ring = np.stack([radius * np.cos(ang),
                     np.zeros_like(ang),
                     radius * np.sin(ang)], axis=1)
    left = ring + np.array([0.0, +0.5 * width, 0.0])
    right = ring + np.array([0.0, -0.5 * width, 0.0])
    faces = [left, right]
    for k in range(sides):
        k2 = (k + 1) % sides
        faces.append(np.array([left[k], left[k2], right[k2], right[k]]))
    out = []
    c = np.asarray(center, dtype=float)
    for f in faces:
        out.append((np.asarray(f) @ R.T) + c)
    return out


# =========================================================================
# scene — the one pass over the file
# =========================================================================
class Scene3D:
    """Everything a replay needs, read from the container exactly once.

    The load, the decimation and the ``utilization`` derivation happen here and
    only here. Cameras consume this object; rendering three cameras must not
    re-read or re-derive anything, which is what the single-pass contract of
    §11.3 means in practice.

    :param path: ``.vdtrace`` path.
    :param stride: keep every ``stride``-th recorded sample as one frame.
    """

    def __init__(self, path, stride: int = 1):
        stride = max(1, int(stride))
        self.path = Path(path)
        with TraceReader(path) as tr:
            self.manifest = dict(tr.manifest)
            self.geometry = dict(tr.geometry)
            self.tire = dict(tr.tire)
            self.repro = dict(tr.repro)
            self.role = tr.role
            self.model_level = tr.model_level
            self.contact_scope = tr.contact_scope
            self.normal_display_only = tr.normal_is_display_only
            self.available = tr.channel_names()

            self.t = np.asarray(tr.channel("t"))
            self.pose = np.asarray(tr.channel("pose"))
            n = len(self.t)
            self.v_body = (np.asarray(tr.channel("v_body")) if tr.has("v_body")
                           else np.zeros((n, 2)))
            self.steer = (np.asarray(tr.channel("u_steer")) if tr.has("u_steer")
                          else np.zeros(n))
            self.wheel_F = (np.asarray(tr.channel("wheel_F")) if tr.has("wheel_F")
                            else None)
            self.util = (tr.utilization() if tr.has("wheel_F") and tr.has("wheel_mu")
                         else np.zeros((n, 4)))

            # --- 0.3 channels. Absent is a fact, not a zero. ---
            self.pose_zrp = (np.asarray(tr.channel("pose_zrp"))
                             if tr.has("pose_zrp") else None)
            self.a_body = (np.asarray(tr.channel("a_body"))
                           if tr.has("a_body") else None)
            self.road_dz = (np.asarray(tr.channel("wheel_road_dz"))
                            if tr.has("wheel_road_dz") else None)
            self.road_normal = (np.asarray(tr.channel("wheel_road_normal"))
                                if tr.has("wheel_road_normal") else None)
            self.wheel_travel = (np.asarray(tr.channel("wheel_travel"))
                                 if tr.has("wheel_travel") else None)
            self.overlays = [tr.overlay(nm) for nm in tr.overlay_names()]

        self.stride = stride
        self.frames = np.arange(0, len(self.t), stride, dtype=int)

        g = self.geometry
        self.wheelbase = float(g["wheelbase_m"])
        self.track = float(g["track_m"])
        self.steer_ratio = float(g.get("steer_ratio", 1.0))
        self.a_front = float(g.get("cg_to_front_m", 0.5 * self.wheelbase))
        self.b_rear = float(g.get("cg_to_rear_m", self.wheelbase - self.a_front))
        self.wheel_radius = float(g.get("wheel_radius_m", 0.32))
        self.wheel_width = float(g.get("wheel_width_m", 0.22))
        self.cg_height = float(g.get("cg_height_m", 0.55))
        self.mass = float(g.get("mass_kg", 1500.0))
        lwh = g.get("body_lwh_m")
        self.body_lwh = ([float(v) for v in lwh] if lwh else
                         [self.wheelbase + 1.6, self.track + 0.25, 1.5])

        #: Why the replay is degraded, or ``None``. Printed once and shown on
        #: every frame; a silently flat 3D view is the failure mode this guards.
        self.degraded = None
        if self.pose_zrp is None:
            self.degraded = (
                "trace carries no pose_zrp (schema %s, model_level %s): drawing "
                "z=0, roll=pitch=0 and labelling the frame planar"
                % (self.manifest.get("schema_version"), self.model_level))

        # Wheel hub positions in the body frame, FL FR RL RR. z puts the hub one
        # wheel radius above the ground plane the CG height is measured from.
        hub_z = -(self.cg_height - self.wheel_radius)
        self.hub_body = np.array([
            [+self.a_front, +0.5 * self.track, hub_z],
            [+self.a_front, -0.5 * self.track, hub_z],
            [-self.b_rear,  +0.5 * self.track, hub_z],
            [-self.b_rear,  -0.5 * self.track, hub_z],
        ])
        # The body box is centred on the wheelbase midpoint. That split of the
        # overhangs is the only dimension the manifest does not pin down; every
        # other number above comes from the file.
        self.body_center_body = np.array(
            [0.5 * (self.a_front - self.b_rear), 0.0,
             -self.cg_height + 0.5 * self.body_lwh[2]])

        self.trajectory = self.pose[:, :2]
        self.traj_z = (self.pose_zrp[:, 0] if self.pose_zrp is not None
                       else np.zeros(len(self.t)))

    # -- per-frame state ---------------------------------------------------
    def attitude(self, i):
        """``(z, roll, pitch)`` at sample ``i`` — zeros when the level has none."""
        if self.pose_zrp is None:
            return 0.0, 0.0, 0.0
        z, roll, pitch = self.pose_zrp[i]
        return float(z), float(roll), float(pitch)

    def ref_path(self):
        """The first ``path2d`` overlay as ``(N,2)``, or ``None``.

        The renderer does not interpret the overlay beyond its envelope; it
        draws whatever ``x``/``y`` it was handed (``11`` §4).
        """
        for ov in self.overlays:
            if isinstance(ov, dict) and ov.get("kind") == "path2d":
                x, y = ov.get("x"), ov.get("y")
                if x and y:
                    return np.column_stack([np.asarray(x, float),
                                            np.asarray(y, float)]), ov.get("name", "path")
        return None


# =========================================================================
# world-space primitives (camera independent -> built once)
# =========================================================================
def frame_primitives(scene: Scene3D, i: int, tiers) -> dict:
    """World-space geometry of one frame. Pure: no drawing, no camera.

    Splitting this out is what makes the multi-camera pass honest — the same
    returned dict is drawn N times, so the per-camera cost really is only the
    projection and the draw.

    :param scene: the loaded scene.
    :param i: sample index.
    :param tiers: set of enabled RT1 layer names.
    :returns: dict of primitive arrays in world coordinates.
    """
    x, y, yaw = (float(v) for v in scene.pose[i])
    z, roll, pitch = scene.attitude(i)
    R = rot_zyx(yaw, pitch, roll)
    origin = np.array([x, y, z])

    prims = {"i": int(i), "t": float(scene.t[i]), "origin": origin,
             "yaw": yaw, "roll": roll, "pitch": pitch, "R": R}

    half = (0.5 * scene.body_lwh[0], 0.5 * scene.body_lwh[1],
            0.5 * scene.body_lwh[2])
    prims["body"] = box_faces(origin + R @ scene.body_center_body, half, R)

    steer = float(scene.steer[i])
    spin = 0.0
    wheels, wheel_util = [], []
    for w in range(4):
        # Only the front wheels steer; the recorded u_steer is already the road
        # wheel angle, so the steering ratio must not be applied again here.
        delta = steer if w < 2 else 0.0
        Rw = R @ rot_zyx(delta, 0.0, 0.0) @ rot_zyx(0.0, spin, 0.0)
        hub = origin + R @ scene.hub_body[w]
        wheels.append(wheel_faces(hub, Rw, scene.wheel_radius, scene.wheel_width))
        wheel_util.append(float(scene.util[i][w]) if scene.util is not None else 0.0)
    prims["wheels"] = wheels
    prims["wheel_util"] = wheel_util
    prims["hubs"] = np.array([origin + R @ scene.hub_body[w] for w in range(4)])

    if "force" in tiers and scene.wheel_F is not None:
        # Contact forces are recorded in the wheel/contact frame; rotate them
        # with the same body attitude so the arrow points where the tyre pushes.
        F = scene.wheel_F[i]
        prims["wheel_F_world"] = np.array([R @ np.array([F[w][0], F[w][1], 0.0])
                                           for w in range(4)])
        prims["wheel_Fz"] = np.array([F[w][2] for w in range(4)])
    if "accel" in tiers and scene.a_body is not None:
        prims["a_world"] = R @ np.asarray(scene.a_body[i], dtype=float)
        prims["cg"] = origin
    if "normal" in tiers and scene.road_normal is not None:
        prims["normals"] = np.asarray(scene.road_normal[i], dtype=float)
        prims["contact_pts"] = np.array([
            prims["hubs"][w] - np.array([0.0, 0.0, scene.wheel_radius])
            for w in range(4)])
        if scene.road_dz is not None:
            for w in range(4):
                prims["contact_pts"][w][2] = float(scene.road_dz[i][w])
    return prims


# =========================================================================
# projection
# =========================================================================
def camera_frame(cam: Camera, prims, scene: Scene3D, fit_box=None):
    """Resolve a camera to ``(eye, target, up, span)`` in world coordinates.

    One body for every alias: the alias only chose the row of the table, and
    the branches below are on ``mount``/``aim``, which is the contract.
    """
    origin = prims["origin"]
    if cam.mount == "vehicle":
        yaw = prims["yaw"] if cam.follow_attitude == "yaw" else 0.0
        Rm = rot_zyx(yaw, 0.0, 0.0)          # roll/pitch deliberately dropped
        eye = origin + Rm @ cam.offset
    else:
        if cam.fit and fit_box is not None:
            cx, cy, radius = fit_box
            eye = np.array([cx - 1.15 * radius, cy - 1.15 * radius,
                            max(0.9 * radius, 12.0)])
        else:
            eye = np.asarray(cam.offset, dtype=float)

    if cam.aim == "vehicle":
        target = origin.copy()
    else:
        target = (np.array([fit_box[0], fit_box[1], 0.0]) if
                  (cam.fit and fit_box is not None) else cam.aim_point.copy())

    if cam.span is not None:
        span = float(cam.span)
    elif cam.fit and fit_box is not None:
        span = 1.15 * float(fit_box[2])
    else:
        span = max(1.35 * float(np.linalg.norm(cam.offset)), 6.0)
    return eye, target, np.array([0.0, 0.0, 1.0]), span


def view_basis(eye, target, up):
    """Right-handed look-at basis ``(right, true_up, forward)``."""
    fwd = np.asarray(target, float) - np.asarray(eye, float)
    nrm = np.linalg.norm(fwd)
    if nrm < 1e-9:
        fwd = np.array([1.0, 0.0, 0.0])
        nrm = 1.0
    fwd = fwd / nrm
    right = np.cross(fwd, up)
    if np.linalg.norm(right) < 1e-9:            # looking straight down
        right = np.cross(fwd, np.array([0.0, 1.0, 0.0]))
    right /= np.linalg.norm(right)
    true_up = np.cross(right, fwd)
    return right, true_up, fwd


def project(points, eye, basis, projection, span, fov_deg=42.0):
    """Project world points to screen units plus a depth.

    :returns: ``(xy, depth)`` with ``xy`` in the same units for both projections
        (metres at the target plane), so the axis limits do not depend on which
        projection a camera chose.
    """
    right, up, fwd = basis
    rel = np.asarray(points, dtype=float).reshape(-1, 3) - np.asarray(eye, float)
    cx = rel @ right
    cy = rel @ up
    depth = rel @ fwd
    if projection == "ortho":
        return np.column_stack([cx, cy]), depth
    f = 1.0 / math.tan(math.radians(0.5 * fov_deg))
    d = np.where(depth > 1e-3, depth, 1e-3)
    scale = 0.5 * span * f / d
    return np.column_stack([cx * scale, cy * scale]), depth


# =========================================================================
# drawing
# =========================================================================
def _sat_color(util):
    """Wheel colour by friction utilization; red above the warning threshold."""
    if util >= 1.0:
        return (0.85, 0.10, 0.10)
    if util >= UTIL_WARN:
        return (0.92, 0.55, 0.10)
    g = 0.35 + 0.35 * (1.0 - min(util / UTIL_WARN, 1.0))
    return (0.25, g, 0.75)


def draw_frame(ax, scene: Scene3D, prims, cam: Camera, tiers, scales,
               fit_box=None, hud=True):
    """Draw one frame of one camera onto a cleared axes."""
    from matplotlib.collections import LineCollection, PolyCollection

    ax.clear()
    ax.set_facecolor("white")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)

    eye, target, up, span = camera_frame(cam, prims, scene, fit_box)
    basis = view_basis(eye, target, up)

    def P(pts):
        return project(pts, eye, basis, cam.projection, span)

    faces, colors, depths = [], [], []

    # -- ground grid (RT0). Drawn around the vehicle, aligned to the world axes
    # -- so the grid itself never rotates and heading stays readable.
    gx, gy = prims["origin"][0], prims["origin"][1]
    reach = span if cam.mount == "world" else 26.0
    step = 2.0 if reach < 30.0 else (5.0 if reach < 120.0 else 25.0)
    g0x, g0y = math.floor(gx / step) * step, math.floor(gy / step) * step
    segs = []
    k = int(reach / step) + 1
    for m in range(-k, k + 1):
        segs.append([[g0x + m * step, g0y - reach, 0.0],
                     [g0x + m * step, g0y + reach, 0.0]])
        segs.append([[g0x - reach, g0y + m * step, 0.0],
                     [g0x + reach, g0y + m * step, 0.0]])
    if segs:
        seg = np.asarray(segs)
        p0, d0 = P(seg[:, 0, :])
        p1, d1 = P(seg[:, 1, :])
        keep = (d0 > 0) & (d1 > 0)
        if keep.any():
            ax.add_collection(LineCollection(
                np.stack([p0[keep], p1[keep]], axis=1),
                colors=(0.80, 0.80, 0.84), linewidths=0.5, zorder=1))

    # -- driven trajectory so far (RT0)
    i = prims["i"]
    if i > 1:
        tr = np.column_stack([scene.trajectory[:i + 1], scene.traj_z[:i + 1]])
        pt, dt_ = P(tr)
        m = dt_ > 0
        if m.sum() > 1:
            ax.plot(pt[m, 0], pt[m, 1], color=(0.15, 0.35, 0.75),
                    lw=1.6, zorder=2, label="trajectory")

    # -- ref_path overlay (RT1)
    if "refpath" in tiers:
        rp = scene.ref_path()
        if rp is not None:
            xy, nm = rp
            pr, dr = P(np.column_stack([xy, np.zeros(len(xy))]))
            m = dr > 0
            if m.sum() > 1:
                ax.plot(pr[m, 0], pr[m, 1], color=(0.45, 0.45, 0.45),
                        lw=1.2, ls="--", zorder=2, label=nm)

    # -- body + wheels (RT0), painter-sorted together so a wheel in front of the
    # -- body is actually drawn in front of it.
    # Wheels first and opaque; the body shell over them and translucent. The
    # body box is the vehicle's *overall* envelope (body_lwh_m is measured from
    # the ground), so an opaque shell would swallow the wheels, the contact
    # points and every RT1 vector that starts under the car. Transparency keeps
    # the envelope honest while leaving what happens at the tyres visible.
    wfaces, wcolors, wdepths = [], [], []
    for w, wf in enumerate(prims["wheels"]):
        base = (_sat_color(prims["wheel_util"][w]) if "saturation" in tiers
                else (0.22, 0.22, 0.24))
        for f in wf:
            p, d = P(f)
            if (d > 0).all():
                wfaces.append(p)
                wcolors.append(base + (1.0,))
                wdepths.append(float(d.mean()))
    if wfaces:
        order = np.argsort(wdepths)[::-1]
        ax.add_collection(PolyCollection(
            [wfaces[k] for k in order], facecolors=[wcolors[k] for k in order],
            edgecolors=(0.10, 0.10, 0.10, 0.7), linewidths=0.3, zorder=3))
    for f in prims["body"]:
        p, d = P(f)
        if (d > 0).all():
            faces.append(p)
            colors.append((0.30, 0.42, 0.62, 0.34))
            depths.append(float(d.mean()))
    if faces:
        order = np.argsort(depths)[::-1]
        ax.add_collection(PolyCollection(
            [faces[k] for k in order], facecolors=[colors[k] for k in order],
            edgecolors=(0.16, 0.24, 0.40, 0.85), linewidths=0.7, zorder=4))

    # -- RT1 vectors. Every one uses a fixed screen-metres-per-unit constant.
    def arrow(p0, vec, scale, color, lw=1.8):
        p1 = np.asarray(p0, float) + np.asarray(vec, float) * scale
        pp, dd = P(np.array([p0, p1]))
        if (dd > 0).all():
            ax.plot(pp[:, 0], pp[:, 1], color=color, lw=lw, zorder=6,
                    solid_capstyle="round")

    if "force" in tiers and "wheel_F_world" in prims:
        for w in range(4):
            arrow(prims["hubs"][w] - np.array([0, 0, scene.wheel_radius]),
                  prims["wheel_F_world"][w], scales["force"], (0.85, 0.20, 0.20))
    if "accel" in tiers and "a_world" in prims:
        arrow(prims["cg"], prims["a_world"], scales["accel"], (0.10, 0.55, 0.25), 2.4)
    if "normal" in tiers and "normals" in prims:
        for w in range(4):
            arrow(prims["contact_pts"][w], prims["normals"][w],
                  scales["normal"], (0.55, 0.35, 0.80), 1.4)

    # -- framing. Ortho and persp already share units, so one limit rule works.
    ax.set_xlim(-span, span)
    ax.set_ylim(-span * 0.62, span * 0.62)
    ax.set_aspect("equal")

    if hud:
        lines = ["t = %6.2f s" % prims["t"],
                 "camera = %s (%s/%s, %s)" % (cam.name, cam.mount, cam.aim,
                                              cam.projection),
                 "role = %s   model_level = %s   contact_scope = %s"
                 % (scene.role, scene.model_level, scene.contact_scope),
                 "roll = %+6.2f deg   pitch = %+6.2f deg   z = %+5.3f m"
                 % (math.degrees(prims["roll"]), math.degrees(prims["pitch"]),
                    prims["origin"][2])]
        if "normal" in tiers and scene.normal_display_only:
            lines.append("normal: display-only (%s)" % (scene.contact_scope or "pre-0.3",))
        if scene.degraded:
            lines.append("planar assumption: %s" % scene.degraded.split(":")[0])
        ax.text(0.012, 0.985, "\n".join(lines), transform=ax.transAxes,
                va="top", ha="left", family="monospace", fontsize=7.5,
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="0.7", alpha=0.85),
                zorder=9)
        scale_txt = ("force %.0f N/m   accel %.1f (m/s^2)/m   normal %.1f m"
                     % (1.0 / max(scales["force"], 1e-9),
                        1.0 / max(scales["accel"], 1e-9), scales["normal"]))
        ax.text(0.988, 0.02, scale_txt, transform=ax.transAxes, va="bottom",
                ha="right", family="monospace", fontsize=6.5, color="0.35", zorder=9)
    return ax


# =========================================================================
# render
# =========================================================================
RT1_LAYERS = ("force", "accel", "saturation", "normal", "refpath")


def render3d(path, out_prefix, cameras=("quarter",), stride=1, fps=20,
             tiers=(), force_scale=4000.0, accel_scale=8.0, normal_scale=1.5,
             dpi=110, figsize=(9.0, 5.4), verbose=True):
    """Render one trace to one file per camera. Single pass over the trace.

    :param path: ``.vdtrace`` input.
    :param out_prefix: output directory or path stem.
    :param cameras: camera aliases or explicit specs.
    :param stride: keep every ``stride``-th sample.
    :param fps: fixed output frame rate. The frame count is a function of the
        trace and ``stride`` only, so the same input always yields the same
        number of frames.
    :param tiers: RT1 layers to enable (see :data:`RT1_LAYERS`).
    :param force_scale: screen metres per newton for the contact-force arrows.
    :param accel_scale: screen metres per m/s^2 for the CG vector.
    :param normal_scale: drawn length of the unit road normal [m].
    :returns: dict with the outputs and the measured timings.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_load0 = time.perf_counter()
    scene = Scene3D(path, stride=stride)
    tiers = set(tiers)
    unknown = tiers - set(RT1_LAYERS)
    if unknown:
        raise ValueError("unknown RT1 layer(s): %s (known: %s)"
                         % (", ".join(sorted(unknown)), ", ".join(RT1_LAYERS)))

    # Camera-independent work: this is the whole of the "base" in base + N*draw.
    prims = [frame_primitives(scene, int(i), tiers) for i in scene.frames]
    traj = scene.trajectory
    cx, cy = float(traj[:, 0].mean()), float(traj[:, 1].mean())
    radius = float(max(np.abs(traj[:, 0] - cx).max(), np.abs(traj[:, 1] - cy).max(), 8.0))
    fit_box = (cx, cy, radius)
    t_load = time.perf_counter() - t_load0

    if scene.degraded and verbose:
        print("degraded: %s" % scene.degraded)

    run_id = str(scene.repro.get("run_id", Path(path).stem))
    out_prefix = Path(out_prefix)
    if out_prefix.suffix:
        outdir, stem = out_prefix.parent, out_prefix.stem
    else:
        outdir, stem = out_prefix, run_id
    outdir.mkdir(parents=True, exist_ok=True)
    preset = "rt0" if not tiers else "rt1-" + "+".join(sorted(tiers))

    scales = {"force": force_scale and 1.0 / force_scale or 0.0,
              "accel": 1.0 / max(accel_scale, 1e-9),
              "normal": normal_scale}
    scales["force"] = 1.0 / max(force_scale, 1e-9)

    result = {"frames": len(prims), "load_s": t_load, "cameras": {},
              "outputs": [], "previews": [], "degraded": scene.degraded,
              "encoder": None}
    mid = len(prims) // 2

    for spec in cameras:
        cam = resolve_camera(spec)
        t0 = time.perf_counter()
        fig = plt.figure(figsize=figsize, dpi=dpi)
        ax = fig.add_axes([0, 0, 1, 1])

        base = "%s__%s__%s" % (stem, preset, cam.name)
        preview = outdir / (base + "_preview.png")
        draw_frame(ax, scene, prims[mid], cam, tiers, scales, fit_box)
        fig.savefig(preview, dpi=dpi)
        result["previews"].append(str(preview))

        target, encoder = _write_movie(fig, ax, scene, prims, cam, tiers, scales,
                                       fit_box, outdir, base, fps, dpi)
        plt.close(fig)
        dt = time.perf_counter() - t0
        result["cameras"][cam.name] = dt
        result["outputs"].append(str(target))
        result["encoder"] = encoder
        if verbose:
            print("  %-10s %6.2f s -> %s" % (cam.name, dt, target))
    result["draw_s"] = sum(result["cameras"].values())
    return result


def _write_movie(fig, ax, scene, prims, cam, tiers, scales, fit_box,
                 outdir, base, fps, dpi):
    """Write an mp4 when ffmpeg exists, else a PNG sequence plus the command.

    ffmpeg is never bundled: a wheel that ships an encoder inherits its size and
    its licence questions. Missing ffmpeg degrades the *format*, not the render,
    so this still returns success.
    """
    have_ffmpeg = shutil.which("ffmpeg") is not None
    if have_ffmpeg:
        from matplotlib.animation import FFMpegWriter
        target = outdir / (base + ".mp4")
        writer = FFMpegWriter(fps=fps, codec="libx264",
                              extra_args=["-pix_fmt", "yuv420p"])
        with writer.saving(fig, str(target), dpi):
            for p in prims:
                draw_frame(ax, scene, p, cam, tiers, scales, fit_box)
                writer.grab_frame()
        return target, "ffmpeg"

    seqdir = outdir / (base + "_png")
    seqdir.mkdir(parents=True, exist_ok=True)
    for k, p in enumerate(prims):
        draw_frame(ax, scene, p, cam, tiers, scales, fit_box)
        fig.savefig(seqdir / ("frame_%05d.png" % k), dpi=dpi)
    print("ffmpeg not found; wrote %d PNGs. Assemble with:\n"
          "  ffmpeg -framerate %d -i %s/frame_%%05d.png -pix_fmt yuv420p %s.mp4"
          % (len(prims), fps, seqdir, outdir / base))
    return seqdir, "png-sequence"


# =========================================================================
# CLI
# =========================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog="vdsim-render3d",
        description="Headless 3D replay of a .vdtrace (RT0 always, RT1 opt-in).")
    p.add_argument("trace", help=".vdtrace input")
    p.add_argument("--out", required=True,
                   help="output directory or path stem")
    p.add_argument("--cameras", default="quarter",
                   help="comma-separated camera aliases (%s)"
                        % ",".join(sorted(CAMERA_ALIASES)))
    p.add_argument("--stride", type=int, default=1,
                   help="keep every Nth recorded sample as one frame")
    p.add_argument("--fps", type=int, default=20, help="fixed output frame rate")
    p.add_argument("--rt1", default="",
                   help="comma-separated RT1 layers (%s), or 'all'"
                        % ",".join(RT1_LAYERS))
    p.add_argument("--force-scale", type=float, default=4000.0,
                   help="newtons per drawn metre (fixed, never autoscaled)")
    p.add_argument("--accel-scale", type=float, default=8.0,
                   help="m/s^2 per drawn metre (fixed, never autoscaled)")
    p.add_argument("--normal-scale", type=float, default=1.5,
                   help="drawn length of the unit road normal [m]")
    p.add_argument("--dpi", type=int, default=110)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    tiers = (list(RT1_LAYERS) if args.rt1.strip() == "all"
             else [s.strip() for s in args.rt1.split(",") if s.strip()])
    cams = [s.strip() for s in args.cameras.split(",") if s.strip()]
    res = render3d(args.trace, args.out, cameras=cams, stride=args.stride,
                   fps=args.fps, tiers=tiers, force_scale=args.force_scale,
                   accel_scale=args.accel_scale, normal_scale=args.normal_scale,
                   dpi=args.dpi)
    print("frames=%d load=%.2fs draw=%.2fs encoder=%s"
          % (res["frames"], res["load_s"], res["draw_s"], res["encoder"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
