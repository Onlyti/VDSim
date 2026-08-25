"""vdsim_render — headless renderer for ``.vdtrace`` runs.

Input is trace files and nothing else: no simulation is re-run and no
results file is re-parsed, so the animation is reproducible from the artifact
alone. Output is a GIF plus a preview PNG (MP4 only when ``imageio-ffmpeg``
happens to be installed) using the existing ``[plot]`` extra — matplotlib and
pillow. There is no GUI window and no node/npm/browser dependency.

Screen contents (contract §6.1):

1. reference path, dashed — from a ``path2d`` overlay; omitted when absent
2. driven path
3. body rectangle, sized from manifest ``geometry``
4. front wheels turned by the recorded steering command
5. velocity arrow
6. HUD in fixed screen coordinates
7. per-wheel friction-utilization colour
8. command time series (``u_steer`` / ``u_fx``) with a current-time cursor

Three curves, three different things
------------------------------------
They are routinely conflated under the word "reference"; they are not the
same object and the renderer keeps them apart by name.

* **waypoint** — the global route the run has to follow. Time-invariant,
  one per scenario, carried by a ``path2d`` overlay. Drawn once, dashed grey.
* **target** — the reference horizon handed to the controller at *this*
  step (N+1 points). Time-varying: the point spacing follows the planned
  speed, so it compresses under braking. Blue, with markers, so the
  spacing is visible.
* **prediction** — the horizon the controller solved for at this step
  (N+1 points). Time-varying; on a failed solve it is whatever the solver
  returned, recorded as-is. Orange, switching to thick red while the
  reported solver status is non-zero.

``target`` and ``prediction`` come from a *sidecar* ``<trace>.qp.npz``
written next to the trace: they are ``(K, N+1, 2)`` and the trace
container's channel table is a whitelist of scalar / fixed-width rows. The
sidecar is optional — without it the renderer behaves exactly as before.

Structure: :func:`frame_spec` is a pure function from (loaded trace, frame
index) to a plain description of the frame, and :func:`draw_frame` renders that
description. The split is what makes the view window testable — axis limits
come from manifest ``geometry`` and never from matplotlib autoscale, which
would otherwise drift frame to frame and silently rescale the animation.

Two or more traces switch the renderer into *overlay* mode (§ "multi-run
overlay" below): one camera, one clock, N vehicles drawn on top of each other
in distinct colours with matching driven paths. That mode exists to compare
runs — a controller against its baseline, the same scenario at three friction
levels — so the picture answers "where do they diverge, and when".

CLI::

    python -m vdsim_render run.vdtrace --out run.gif
    python -m vdsim_render a.vdtrace b.vdtrace c.vdtrace --out compare.gif
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")  # headless: no GUI backend, importable over ssh
import matplotlib.pyplot as plt                                    # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter       # noqa: E402
from matplotlib.collections import PolyCollection                  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize   # noqa: E402
from matplotlib.gridspec import GridSpec                           # noqa: E402

try:
    from vdsim_trace import TraceReader, utilization
except ImportError:                                                # in-tree run
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from vdsim_trace import TraceReader, utilization

#: BEV half-window, in wheelbases. The tracking camera's extent is a pure
#: function of manifest `geometry`, which is what keeps the scale constant.
VIEW_HALF_WHEELBASES = 6.0
#: Body rectangle proportions, in wheelbases / tracks. Used only when the
#: manifest does not carry explicit body dimensions.
BODY_LENGTH_PER_WHEELBASE = 1.56
BODY_WIDTH_PER_TRACK = 1.18
#: Drawn tyre width [m] (a drawing proportion, not a vehicle parameter).
TYRE_WIDTH_M = 0.245
#: Utilization at which a wheel is flagged as close to the friction limit.
UTIL_WARN = 0.8
#: Colour scale ceiling; >1.0 keeps saturated wheels distinguishable.
UTIL_MAX = 1.2
#: Velocity arrow length per (m/s), in wheelbases.
ARROW_S_PER_MPS = 0.06
#: Channels the command panel prefers, in order. Anything absent is skipped and
#: the list is intersected with the manifest — never assumed present.
PREFERRED_SERIES = ("u_steer", "u_fx")

UTIL_CMAP = LinearSegmentedColormap.from_list(
    "vdsim_util", ["#2c9c4c", "#9acd32", "#f5c542", "#e8752a", "#c02020"])
UTIL_NORM = Normalize(vmin=0.0, vmax=UTIL_MAX)

#: Suffix of the optional horizons sidecar, resolved from the trace path.
SIDECAR_SUFFIX = ".qp.npz"
#: Overlay names that mean *the global route*. Traces recorded before the
#: waypoint / target / prediction split called it ``reference_path`` or
#: ``ref_path``; the aliases keep them rendering with the right label.
WAYPOINT_NAMES = ("waypoint", "reference_path", "ref_path")
WAYPOINT_LABEL = "waypoint"
#: Grey and dashed, so the two *horizon* curves own the saturated colours.
WAYPOINT_COLOR = "#8c8c8c"
#: MPC input horizon: markers on, so the point spacing (and its
#: compression under braking) is readable.
TARGET_COLOR = "#1e9de8"
TARGET_LABEL = "target (MPC input)"
#: MPC output horizon.
PRED_COLOR = "#e8752a"
PRED_LABEL = "prediction (MPC output)"
#: Prediction styling once the solver reports a non-zero status.
PRED_FAIL_COLOR = "#c02020"
PRED_LW = 1.8
PRED_FAIL_LW = 3.2
#: Margin added to an explicitly requested view half-width.
VIEW_MARGIN_FRAC = 0.05


def load_sidecar(path, required: bool = False):
    """Load the ``.qp.npz`` horizons sidecar that sits next to a trace.

    :param path: the sidecar itself, or the ``.vdtrace`` it belongs to.
    :param required: raise when it is missing instead of returning ``None``.
    :returns: dict of arrays, or ``None`` when there is no sidecar.
    """
    p = Path(path)
    if p.suffix != ".npz":
        p = p.with_suffix(SIDECAR_SUFFIX)
    if not p.is_file():
        if required:
            raise FileNotFoundError("sidecar not found: %s" % (p,))
        return None
    with np.load(str(p)) as z:
        data = {k: np.asarray(z[k]) for k in z.files}
    for key in ("t", "tgt_XY", "pred_XY", "status"):
        if key not in data:
            raise ValueError("sidecar %s is missing %r" % (p, key))
    return data


def nearest_time_index(t_query, t_ref):
    """Index of the nearest ``t_ref`` sample for each ``t_query`` sample.

    The control period and the trace decimation are independent, so the
    sidecar and the trace are aligned on time rather than on index.
    """
    t_ref = np.asarray(t_ref, dtype=float)
    t_query = np.asarray(t_query, dtype=float)
    if t_ref.size <= 1:
        return np.zeros(t_query.shape, dtype=int)
    j = np.clip(np.searchsorted(t_ref, t_query), 1, t_ref.size - 1)
    take_left = (t_query - t_ref[j - 1]) <= (t_ref[j] - t_query)
    return np.where(take_left, j - 1, j).astype(int)


class LoadedTrace:
    """Every array the renderer needs, loaded once from a ``.vdtrace``.

    Holding the decoded arrays separately from the reader keeps
    :func:`frame_spec` free of file I/O, so it stays a pure function.

    :param path: ``.vdtrace`` path.
    :param stride: keep every ``stride``-th recorded sample as one output frame.
    :param sidecar: explicit ``.qp.npz`` with the per-step target and
        prediction horizons. ``None`` looks for ``<trace>.qp.npz`` and
        accepts its absence.
    """

    def __init__(self, path, stride: int = 1, sidecar=None):
        stride = max(1, int(stride))
        with TraceReader(path) as tr:
            self.manifest = tr.manifest
            self.geometry = dict(tr.geometry)
            self.tire = dict(tr.tire)
            self.repro = dict(tr.repro)
            self.wheels = tr.wheels
            self.available = tr.channel_names()
            self.t = np.asarray(tr.channel("t"))
            self.pose = np.asarray(tr.channel("pose"))
            self.v_body = (np.asarray(tr.channel("v_body"))
                           if tr.has("v_body") else np.zeros((len(self.t), 2)))
            self.yaw_rate = (np.asarray(tr.channel("yaw_rate"))
                             if tr.has("yaw_rate") else np.zeros(len(self.t)))
            self.util = (tr.utilization()
                         if tr.has("wheel_F") and tr.has("wheel_mu")
                         else np.zeros((len(self.t), 4)))
            # Command panel series are chosen from the manifest, not hardcoded.
            names = [n for n in PREFERRED_SERIES if tr.has(n)]
            if not names:
                names = [c["name"] for c in self.manifest["channels"]
                         if len(c["shape"]) == 1 and c["name"] != "t"][:2]
            self.series = [(n, tr.channel_meta(n).get("unit", ""),
                            np.asarray(tr.channel(n))) for n in names]
            self.overlays = [tr.overlay(n) for n in tr.overlay_names()]

        self.stride = stride
        self.frames = np.arange(0, len(self.t), stride, dtype=int)
        self.steer = self._series_or_zeros("u_steer")
        self.body_l, self.body_w = body_size(self.geometry)
        self.view_half = view_half_m(self.geometry)
        self.sidecar = load_sidecar(path if sidecar is None else sidecar,
                                    required=sidecar is not None)
        self.sc_map = (nearest_time_index(self.t, self.sidecar["t"])
                       if self.sidecar is not None else None)
        # How far the horizons reach. The default window is a function of
        # manifest geometry only (that is what keeps the scale constant), so
        # this is reported rather than silently applied.
        self.horizon_reach = self._horizon_reach()
        #: Window half-width actually used; ``render`` may override it.
        self.view_half_used = self.view_half

    def _series_or_zeros(self, name):
        for n, _unit, arr in self.series:
            if n == name:
                return arr
        return np.zeros(len(self.t))

    @property
    def n_frames(self) -> int:
        """Number of animation frames after striding."""
        return len(self.frames)

    def path2d_overlays(self):
        """Reference paths to draw dashed (overlay ``kind == "path2d"``)."""
        return [o for o in self.overlays
                if isinstance(o, dict) and o.get("kind") == "path2d"]

    def region_overlays(self):
        """Filled regions to draw under the path (overlay ``kind == "region"``)."""
        return [o for o in self.overlays
                if isinstance(o, dict) and o.get("kind") == "region"]

    def event_overlays(self):
        """Time markers for the command panel (overlay ``kind == "event"``)."""
        return [o for o in self.overlays
                if isinstance(o, dict) and o.get("kind") == "event"]

    def _horizon_reach(self) -> float:
        """Farthest horizon point from the vehicle over the run [m].

        0.0 without a sidecar. Failed steps can hold NaN or absurd values,
        so only finite points count.
        """
        if self.sidecar is None:
            return 0.0
        if "ego_XY" in self.sidecar:
            ego = np.asarray(self.sidecar["ego_XY"], dtype=float)
        else:
            # One pose per *sidecar step*. sc_map runs the other way (one
            # entry per trace sample) and would be the wrong length here.
            ego = self.pose[nearest_time_index(self.sidecar["t"], self.t), 0:2]
        reach = 0.0
        for key in ("tgt_XY", "pred_XY"):
            xy = np.asarray(self.sidecar[key], dtype=float)
            d = np.linalg.norm(xy - ego[:, None, :], axis=2)
            d = d[np.isfinite(d)]
            if d.size:
                reach = max(reach, float(d.max()))
        return reach

    def has_sidecar(self) -> bool:
        """Whether per-step target / prediction horizons were loaded."""
        return self.sidecar is not None

    def qp_fail_times(self):
        """Times [s] whose solve reported a non-zero solver status."""
        if self.sidecar is None:
            return np.zeros(0)
        t = np.asarray(self.sidecar["t"], dtype=float)
        return t[np.asarray(self.sidecar["status"], dtype=float) != 0.0]

    def first_fail_frame(self):
        """Recorded-sample index of the first failed solve; ``None`` if none."""
        if self.sidecar is None:
            return None
        bad = np.asarray(self.sidecar["status"], dtype=float)[self.sc_map] != 0.0
        return int(np.argmax(bad)) if bad.any() else None


# --------------------------------------------------------------------------
# pure geometry / frame description
# --------------------------------------------------------------------------

def view_half_m(geometry: dict) -> float:
    """Half-width of the BEV tracking window [m], derived from ``geometry``.

    Deriving it from the wheelbase — not from the data extent — is what makes
    the animation scale constant across frames and across runs of the same
    vehicle.
    """
    return VIEW_HALF_WHEELBASES * float(geometry["wheelbase_m"])


def body_size(geometry: dict):
    """Body rectangle ``(length, width)`` [m].

    Explicit ``body_length_m`` / ``body_width_m`` win; otherwise the rectangle
    is proportioned from the wheelbase and track already in the manifest.
    """
    wb = float(geometry["wheelbase_m"])
    tr = float(geometry["track_m"])
    length = float(geometry.get("body_length_m", BODY_LENGTH_PER_WHEELBASE * wb))
    width = float(geometry.get("body_width_m", BODY_WIDTH_PER_TRACK * tr))
    return length, width


def _rect(cx, cy, length, width, angle):
    """Corner list of a rectangle centred at (cx, cy), rotated by ``angle`` [rad]."""
    c, s = math.cos(angle), math.sin(angle)
    hl, hw = length * 0.5, width * 0.5
    pts = ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in pts]


def body_and_wheels(geometry: dict, body_l: float, body_w: float,
                    x: float, y: float, yaw: float, delta: float):
    """Body rectangle and the four tyre rectangles for one pose.

    Shared by the single-run and the overlay renderer so both draw a vehicle
    the same way — the overlay would otherwise grow a second, silently
    diverging copy of the wheel placement.

    :param geometry: manifest ``geometry`` block of the run being drawn.
    :param body_l: body rectangle length [m] (see :func:`body_size`).
    :param body_w: body rectangle width [m].
    :param x: CG position X [m], world frame.
    :param y: CG position Y [m], world frame.
    :param yaw: heading [rad].
    :param delta: road-wheel steer angle [rad], applied to the front pair.
    :returns: ``(body_polygon, [FL, FR, RL, RR])`` corner lists in world frame.
    """
    wb = float(geometry["wheelbase_m"])
    track_f = float(geometry.get("track_front_m", geometry["track_m"]))
    track_r = float(geometry.get("track_rear_m", geometry["track_m"]))
    lf = float(geometry.get("cg_to_front_m", 0.5 * wb))
    lr = float(geometry.get("cg_to_rear_m", wb - 0.5 * wb))
    radius = float(geometry.get("wheel_radius_m", 0.34))
    c, s = math.cos(yaw), math.sin(yaw)

    def to_world(bx, by):
        return (x + c * bx - s * by, y + s * bx + c * by)

    # Wheel centres in body frame, FL / FR / RL / RR (ISO 8855: +y left).
    corners = ((lf, 0.5 * track_f, delta), (lf, -0.5 * track_f, delta),
               (-lr, 0.5 * track_r, 0.0), (-lr, -0.5 * track_r, 0.0))
    wheels = []
    for bx, by, steer in corners:
        wx, wy = to_world(bx, by)
        wheels.append(_rect(wx, wy, 2.0 * radius, TYRE_WIDTH_M, yaw + steer))
    body = _rect(*to_world(0.5 * (lf - lr), 0.0), body_l, body_w, yaw)
    return body, wheels


def velocity_arrow(geometry: dict, x: float, y: float, yaw: float,
                   vx: float, vy: float):
    """Velocity arrow ``(x, y, dx, dy)`` [m] for a body-frame velocity."""
    wb = float(geometry["wheelbase_m"])
    c, s = math.cos(yaw), math.sin(yaw)
    speed = math.hypot(vx, vy)
    ax_, ay_ = (c * vx - s * vy, s * vx + c * vy)
    norm = max(math.hypot(ax_, ay_), 1e-9)
    arrow_len = ARROW_S_PER_MPS * wb * speed
    return (x, y, ax_ / norm * arrow_len, ay_ / norm * arrow_len)


def _fmt_util(u: float) -> str:
    """Format one utilization value for the HUD.

    A wheel with mu*Fz = 0 (the sample recorded at reset, before the first
    integration step) divides by zero, and printing the raw 1e94 that comes
    back overran the HUD box. Out-of-range values are named, not shown.
    """
    if not np.isfinite(u):
        return " n/a"
    if abs(u) >= 100.0:
        return ">>99"
    return "%4.2f" % u


def series_for_display(unit: str, values):
    """Return ``(display_unit, values)`` for a plotted trace channel.

    Trace channels keep their native SI representation. Matplotlib-facing
    angular channels are converted here so every visible angle uses degrees,
    while vehicle geometry and interpolation continue to operate in radians.

    :param unit: channel unit from the trace manifest.
    :param values: channel samples in their recorded unit.
    :returns: display unit and a NumPy array of display values.
    """
    raw_unit = str(unit or "")
    normalized = raw_unit.strip().lower()
    for prefix in ("radians", "radian", "rad"):
        if not normalized.startswith(prefix):
            continue
        suffix = normalized[len(prefix):]
        # Accept an angle itself and compound angular units such as rad/s,
        # rad s^-1 and rad/s^2. Reject unrelated words beginning with rad.
        if suffix and suffix[0] not in "/ *":
            continue
        display_unit = "deg" + suffix
        return display_unit, np.degrees(np.asarray(values, dtype=float))
    return raw_unit, np.asarray(values)


def frame_spec(tr: LoadedTrace, i: int) -> dict:
    """Describe frame ``i`` — pure function, no matplotlib, no file access.

    :param tr: loaded trace.
    :param i: index into the *recorded* samples (not the strided frame list).
    :returns: dict with ``xlim``/``ylim`` (from manifest geometry), ``body``
        and ``wheels`` polygons, ``wheel_util`` colours' source values,
        ``arrow``, ``trail``, ``hud`` lines and the time cursor.
    """
    x, y, yaw = (float(v) for v in tr.pose[i])
    half = tr.view_half_used
    delta = float(tr.steer[i])
    body, wheels = body_and_wheels(tr.geometry, tr.body_l, tr.body_w,
                                   x, y, yaw, delta)

    vx, vy = (float(v) for v in tr.v_body[i])
    speed = math.hypot(vx, vy)
    util = [float(u) for u in tr.util[i]]

    hud = [
        "t = %6.2f s" % (float(tr.t[i]),),
        "v = %5.2f m/s  (%5.1f km/h)" % (speed, speed * 3.6),
        "beta = %+5.1f deg" % (math.degrees(math.atan2(vy, max(abs(vx), 1e-6))),),
        "r = %+6.2f deg/s" % (math.degrees(float(tr.yaw_rate[i])),),
        "delta = %+5.2f deg" % (math.degrees(delta),),
        "util max = %s  [%s]" % (
            _fmt_util(max(util) if util else 0.0),
            " ".join("%s %s" % (w, _fmt_util(u))
                     for w, u in zip(tr.wheels, util))),
    ]

    # Horizons, when a sidecar was found. Without one the two artists are
    # never created and these stay None, which is what keeps old traces
    # rendering unchanged.
    tgt_xy = pred_xy = None
    status = None
    solve_ms = float("nan")
    if tr.sidecar is not None:
        j = int(tr.sc_map[i])
        tgt_xy = np.asarray(tr.sidecar["tgt_XY"][j], dtype=float)
        pred_xy = np.asarray(tr.sidecar["pred_XY"][j], dtype=float)
        status = int(tr.sidecar["status"][j])
        if "solve_ms" in tr.sidecar:
            solve_ms = float(tr.sidecar["solve_ms"][j])
        hud.append("solver = %-4s status %d%s"
                   % ("OK" if status == 0 else "FAIL", status,
                      "" if not np.isfinite(solve_ms)
                      else "  %6.1f ms" % solve_ms))

    return {
        "index": int(i),
        "t": float(tr.t[i]),
        "xlim": (x - half, x + half),
        "ylim": (y - half, y + half),
        "body": body,
        "wheels": wheels,
        "wheel_util": util,
        "arrow": velocity_arrow(tr.geometry, x, y, yaw, vx, vy),
        "trail_x": tr.pose[: i + 1, 0],
        "trail_y": tr.pose[: i + 1, 1],
        "tgt_xy": tgt_xy,
        "pred_xy": pred_xy,
        "status": status,
        "solve_ms": solve_ms,
        "hud": hud,
    }


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------

def merge_spans(times, dt: float):
    """Group consecutive failure times into ``(t_start, t_end)`` spans.

    A gap wider than 1.5 sampling periods starts a new span, so a run that
    fails everywhere becomes one shaded band and a run that fails on three
    scattered steps stays three marks.
    """
    t = np.asarray(times, dtype=float)
    if t.size == 0:
        return []
    t = np.sort(t)
    width = max(dt, 1e-9)
    cut = np.nonzero(np.diff(t) > 1.5 * width)[0]
    starts = np.concatenate(([0], cut + 1))
    ends = np.concatenate((cut, [t.size - 1]))
    return [(float(t[a] - width / 2), float(t[b] + width / 2))
            for a, b in zip(starts, ends)]


def _setup_axes(tr: LoadedTrace, figsize, dpi):
    fig = plt.figure(figsize=figsize, dpi=dpi)
    gs = GridSpec(len(tr.series), 2, figure=fig, width_ratios=[1.45, 1.0],
                  hspace=0.32, wspace=0.24,
                  left=0.07, right=0.965, top=0.90, bottom=0.10)
    ax_bev = fig.add_subplot(gs[:, 0])
    ax_series = [fig.add_subplot(gs[k, 1]) for k in range(len(tr.series))]
    return fig, ax_bev, ax_series


def _draw_static(tr: LoadedTrace, ax_bev, ax_series):
    """Draw everything that does not change per frame, and build the artists."""
    ax_bev.set_aspect("equal", adjustable="box")
    ax_bev.set_xlabel("X [m]")
    ax_bev.set_ylabel("Y [m]")
    ax_bev.grid(True, alpha=0.25, lw=0.5)

    for reg in tr.region_overlays():
        poly = reg.get("polygon") or []
        if len(poly) >= 3:
            ax_bev.add_collection(PolyCollection(
                [poly], facecolors="#c44e52", alpha=0.16,
                edgecolors="#c44e52", linewidths=0.8, zorder=0))
    for ref in tr.path2d_overlays():
        xy = np.asarray(ref.get("xy") or [], dtype=float)
        if xy.size:
            # The global route, drawn once. It is *not* the per-step target
            # horizon, so it gets the muted style and the name to match.
            is_wp = (str(ref.get("name", "")).lower() in WAYPOINT_NAMES
                     or str(ref.get("alias", "")).lower() in WAYPOINT_NAMES)
            ax_bev.plot(xy[:, 0], xy[:, 1], ls="--", lw=1.2,
                        color=WAYPOINT_COLOR if is_wp else "#4c72b0",
                        alpha=0.9, zorder=1,
                        label=WAYPOINT_LABEL if is_wp
                        else ref.get("name", "reference"))

    (trail,) = ax_bev.plot([], [], "-", lw=1.4, color="#333333", alpha=0.85, zorder=2)
    body = PolyCollection([], facecolors="#dfe6ee", edgecolors="#1b2733",
                          linewidths=1.2, zorder=3)
    ax_bev.add_collection(body)
    wheels = PolyCollection([], edgecolors="#1b2733", linewidths=0.8, zorder=4)
    wheels.set_cmap(UTIL_CMAP)
    wheels.set_norm(UTIL_NORM)
    ax_bev.add_collection(wheels)
    arrow = ax_bev.annotate(
        "", xy=(0, 0), xytext=(0, 0), zorder=5,
        arrowprops=dict(arrowstyle="-|>", color="#d62728", lw=1.8,
                        shrinkA=0, shrinkB=0))
    hud = ax_bev.text(
        0.015, 0.985, "", transform=ax_bev.transAxes, va="top", ha="left",
        fontsize=8, family="monospace", zorder=10,
        bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#8a97a5", alpha=0.85))

    # Time-varying horizons. These are per-frame Line2D updates, not static
    # draws: the target moves because the reference grid is re-sampled every
    # step, the prediction because the OCP is re-solved every step.
    target = prediction = qpfail = None
    if tr.has_sidecar():
        (target,) = ax_bev.plot([], [], ls="-", lw=1.0, color=TARGET_COLOR,
                                marker="o", ms=2.6, mew=0.0, alpha=0.95,
                                zorder=6, label=TARGET_LABEL)
        (prediction,) = ax_bev.plot([], [], ls="-", lw=PRED_LW,
                                    color=PRED_COLOR, alpha=0.95, zorder=7,
                                    label=PRED_LABEL)
        qpfail = ax_bev.text(
            0.985, 0.015, "", transform=ax_bev.transAxes, va="bottom",
            ha="right", fontsize=11, fontweight="bold", color="white",
            zorder=11,
            bbox=dict(boxstyle="round,pad=0.35", fc=PRED_FAIL_COLOR,
                      ec="none", alpha=0.92))
        qpfail.set_visible(False)

    if ax_bev.get_legend_handles_labels()[0]:
        # Lower left: the HUD owns the top-left corner in screen coordinates.
        ax_bev.legend(loc="lower left", fontsize=7, framealpha=0.6)

    cbar = ax_bev.figure.colorbar(
        plt.cm.ScalarMappable(norm=UTIL_NORM, cmap=UTIL_CMAP),
        ax=ax_bev, fraction=0.035, pad=0.015)
    cbar.set_label("tyre friction utilization [-]", fontsize=8)
    cbar.ax.axhline(UTIL_WARN, color="#111111", lw=1.0, ls="--")
    cbar.ax.tick_params(labelsize=7)

    cursors = []
    events = tr.event_overlays()
    # QP-failure marks: the recorded ``qp_fail`` event overlay when there is
    # one, otherwise derived from the sidecar status so a trace recorded
    # without the overlay still shows them.
    qp_events = [ev for ev in events if str(ev.get("name", "")).lower() == "qp_fail"]
    fail_t = (np.unique(np.concatenate(
        [np.asarray(ev.get("t", []), dtype=float) for ev in qp_events]))
        if qp_events else tr.qp_fail_times())
    fail_spans = merge_spans(fail_t, float(np.median(np.diff(tr.t)))
                             if len(tr.t) > 1 else 0.0)
    for ax, (name, unit, arr) in zip(ax_series, tr.series):
        display_unit, display_arr = series_for_display(unit, arr)
        ax.plot(tr.t, display_arr, lw=1.1, color="#4c72b0")
        ax.set_ylabel("%s [%s]" % (name, display_unit or "-"), fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25, lw=0.5)
        ax.set_xlim(float(tr.t[0]), float(tr.t[-1]))
        for ev in events:
            if ev in qp_events:
                continue
            for et in ev.get("t", []):
                ax.axvline(float(et), color="#c02020", lw=0.8, alpha=0.6)
        for t0, t1 in fail_spans:
            # Shaded spans, not one line per failed step: a run that fails
            # on every step drew 500 lines and filled the panel solid red,
            # which reads as a rendering artifact rather than as a result.
            ax.axvspan(t0, t1, color=PRED_FAIL_COLOR, alpha=0.18, lw=0,
                       zorder=0)
        cursors.append(ax.axvline(float(tr.t[0]), color="#111111", lw=1.0))
    if ax_series:
        ax_series[-1].set_xlabel("time [s]", fontsize=8)
    return dict(trail=trail, body=body, wheels=wheels, arrow=arrow,
                hud=hud, cursors=cursors, target=target,
                prediction=prediction, qpfail=qpfail,
                n_qp_fail=int(len(fail_t)))


def draw_frame(artists, ax_bev, spec: dict):
    """Apply a :func:`frame_spec` description to the prepared artists."""
    ax_bev.set_xlim(*spec["xlim"])
    ax_bev.set_ylim(*spec["ylim"])
    artists["trail"].set_data(spec["trail_x"], spec["trail_y"])
    artists["body"].set_verts([spec["body"]])
    artists["wheels"].set_verts(spec["wheels"])
    artists["wheels"].set_array(np.asarray(spec["wheel_util"]))
    x, y, dx, dy = spec["arrow"]
    artists["arrow"].set_position((x, y))
    artists["arrow"].xy = (x + dx, y + dy)
    artists["hud"].set_text("\n".join(spec["hud"]))
    _draw_horizons(artists, spec)
    for cur in artists["cursors"]:
        cur.set_xdata([spec["t"], spec["t"]])


def _draw_horizons(artists, spec: dict):
    """Update the target / prediction lines and the QP-failure banner.

    No-op for a trace without a sidecar: the artists were never created.
    A failed step is drawn exactly as recorded — NaNs break the line into
    gaps rather than being interpolated over, which is the honest picture
    of a solver that returned nothing usable.
    """
    target, pred = artists.get("target"), artists.get("prediction")
    if target is None or pred is None:
        return
    tgt = spec.get("tgt_xy")
    if tgt is not None and len(tgt):
        tgt = np.asarray(tgt, dtype=float)
        target.set_data(tgt[:, 0], tgt[:, 1])
    else:
        target.set_data([], [])
    pxy = spec.get("pred_xy")
    if pxy is not None and len(pxy):
        pxy = np.asarray(pxy, dtype=float)
        pred.set_data(pxy[:, 0], pxy[:, 1])
    else:
        pred.set_data([], [])
    failed = bool(spec.get("status"))  # 0 and None are both "not failed"
    pred.set_color(PRED_FAIL_COLOR if failed else PRED_COLOR)
    pred.set_linewidth(PRED_FAIL_LW if failed else PRED_LW)
    banner = artists.get("qpfail")
    if banner is not None:
        banner.set_visible(failed)
        if failed:
            banner.set_text("QP FAIL (status=%d)" % int(spec["status"]))


# --------------------------------------------------------------------------
# multi-run overlay
# --------------------------------------------------------------------------

#: Per-run colours, in assignment order. Chosen to stay distinguishable when
#: two bodies overlap at the configured alpha; cycled when runs outnumber them.
RUN_COLORS = ("#4c72b0", "#dd8452", "#55a868", "#c44e52", "#8172b3",
              "#937860", "#da8bc3", "#8c8c8c", "#ccb974", "#64b5cd")
#: Default body fill alpha in overlay mode. Runs of the same scenario overlap
#: for most of their length, so opaque bodies would hide every run but the last.
BODY_ALPHA = 0.55
#: Default driven-trail alpha.
PATH_ALPHA = 0.9
#: Alpha of the faint full-route line drawn once per run under everything else.
GHOST_ALPHA = 0.18
#: Alpha multiplier for a run whose trace has already ended.
ENDED_ALPHA_SCALE = 0.35
#: Fit-camera margin, as a fraction of the fitted span.
FIT_MARGIN_FRAC = 0.08
#: Wheel outline once utilization crosses :data:`UTIL_WARN`. Overlay mode
#: spends body colour on run identity, so grip has to show up on the outline.
WHEEL_EDGE = "#1b2733"
WHEEL_EDGE_WARN = "#c02020"


def _unwrapped_interp(t_query, t, values):
    """Interpolate an angle series [rad] across the ±π seam.

    Interpolating wrapped yaw directly would sweep the vehicle the long way
    round whenever a frame time falls across the seam.
    """
    return np.interp(t_query, t, np.unwrap(np.asarray(values, dtype=float)))


class MultiScene:
    """Several traces resampled onto one common time base.

    Runs are recorded independently — different ``dt``, different durations,
    possibly different vehicles — so an overlay cannot share a sample index.
    Frame ``k`` is a *time*, and every run is interpolated at that time. A run
    whose trace has ended holds its final pose and reports ``active = False``
    so the drawing code fades it instead of pretending it is still driving.

    :param paths: two or more ``.vdtrace`` paths.
    :param labels: legend labels; defaults to run ids, falling back to stems.
    :param colors: matplotlib colours; defaults to :data:`RUN_COLORS`.
    :param fps: output frame rate.
    :param speed: playback speed relative to wall-clock (2.0 = twice as fast).
    :param follow: ``"fit"`` for a static window holding every run, or a run
        index for a tracking camera locked to that run.
    """

    def __init__(self, paths, labels=None, colors=None, fps: int = 20,
                 speed: float = 1.0, follow="fit"):
        paths = [Path(p) for p in paths]
        if len(paths) < 2:
            raise ValueError("overlay mode needs at least two traces, got %d"
                             % (len(paths),))
        step = float(speed) / float(max(int(fps), 1))
        if not step > 0.0:
            raise ValueError("fps and speed must be positive (got fps=%r, speed=%r)"
                             % (fps, speed))

        self.paths = paths
        self.runs = [LoadedTrace(p) for p in paths]
        self.labels = _run_labels(self.runs, paths, labels)
        self.colors = _run_colors(len(self.runs), colors)
        self.follow = self._follow_index(follow)

        self.t0 = min(float(r.t[0]) for r in self.runs)
        self.t1 = max(float(r.t[-1]) for r in self.runs)
        self.step = step
        n = int(math.floor((self.t1 - self.t0) / step + 1e-9)) + 1
        self.times = self.t0 + step * np.arange(max(n, 1))
        self.tracks = [self.resample_run(r, self.times) for r in self.runs]
        self.series_names = self._series_names()
        self.xlim, self.ylim = self._fit_window()

    # -- construction helpers ---------------------------------------------
    def _follow_index(self, follow):
        """Validate ``follow``; ``None`` means the static fit-all window."""
        if follow is None or follow == "fit":
            return None
        try:
            idx = int(follow)
        except (TypeError, ValueError):
            raise ValueError("--follow takes 'fit' or a run index, got %r" % (follow,))
        if not 0 <= idx < len(self.runs):
            raise ValueError("--follow %d is out of range for %d runs"
                             % (idx, len(self.runs)))
        return idx

    @staticmethod
    def resample_run(run: LoadedTrace, times):
        """Interpolate one run onto ``times``; hold the end pose past its end.

        :returns: dict of arrays, all of length ``len(times)``, plus ``active``
            (False once the run's own trace has ended).
        """
        t = np.asarray(run.t, dtype=float)
        util = np.column_stack([np.interp(times, t, run.util[:, w])
                                for w in range(run.util.shape[1])])
        return {
            "x": np.interp(times, t, run.pose[:, 0]),
            "y": np.interp(times, t, run.pose[:, 1]),
            "yaw": _unwrapped_interp(times, t, run.pose[:, 2]),
            "steer": np.interp(times, t, run.steer),
            "vx": np.interp(times, t, run.v_body[:, 0]),
            "vy": np.interp(times, t, run.v_body[:, 1]),
            "yaw_rate": np.interp(times, t, run.yaw_rate),
            "util": util,
            # np.interp clamps, so the pose past the end is the final pose.
            "active": (times >= t[0] - 1e-9) & (times <= t[-1] + 1e-9),
        }

    def _series_names(self):
        """Command channels to panel, in :data:`PREFERRED_SERIES` order.

        The union across runs, not the intersection: a run that lacks a
        channel simply contributes no curve to that panel.
        """
        names = []
        for run in self.runs:
            for name, _unit, _arr in run.series:
                if name not in names:
                    names.append(name)
        order = {n: i for i, n in enumerate(PREFERRED_SERIES)}
        return sorted(names, key=lambda n: (order.get(n, len(order)), n))

    def series_unit(self, name: str) -> str:
        """Unit string for a panelled channel, from the first run carrying it."""
        for run in self.runs:
            for n, unit, _arr in run.series:
                if n == name:
                    return unit
        return ""

    def series_of(self, run: LoadedTrace, name: str):
        """That run's samples for a channel, or ``None`` when it lacks it."""
        for n, _unit, arr in run.series:
            if n == name:
                return arr
        return None

    def _fit_window(self):
        """Static square window covering every run's whole route.

        Square by construction: the BEV keeps ``aspect="equal"``, and a
        non-square limit pair would otherwise be silently re-fitted by
        matplotlib and stop matching what the spec reports.
        """
        xs = np.concatenate([r.pose[:, 0] for r in self.runs])
        ys = np.concatenate([r.pose[:, 1] for r in self.runs])
        pad = max(max(r.body_l, r.body_w) for r in self.runs)
        cx = 0.5 * (float(xs.min()) + float(xs.max()))
        cy = 0.5 * (float(ys.min()) + float(ys.max()))
        span = max(float(xs.max() - xs.min()), float(ys.max() - ys.min()), 1e-6)
        half = 0.5 * span * (1.0 + 2.0 * FIT_MARGIN_FRAC) + pad
        return (cx - half, cx + half), (cy - half, cy + half)

    # -- queries -----------------------------------------------------------
    @property
    def n_frames(self) -> int:
        """Number of overlay frames on the common time base."""
        return len(self.times)

    def window(self, k: int):
        """``(xlim, ylim)`` for frame ``k`` — fit-all, or tracking ``follow``."""
        if self.follow is None:
            return self.xlim, self.ylim
        run, track = self.runs[self.follow], self.tracks[self.follow]
        x, y = float(track["x"][k]), float(track["y"][k])
        half = run.view_half
        return (x - half, x + half), (y - half, y + half)

    def spread(self, k: int) -> float:
        """Largest distance [m] between any two runs at frame ``k``.

        The comparison's whole point is divergence, so this is what picks the
        preview frame — an arbitrary frame would usually show the runs still
        on top of each other.
        """
        pts = [(float(tk["x"][k]), float(tk["y"][k])) for tk in self.tracks]
        worst = 0.0
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                worst = max(worst, math.hypot(pts[i][0] - pts[j][0],
                                              pts[i][1] - pts[j][1]))
        return worst

    def preview_frame(self) -> int:
        """Frame of maximum inter-run spread."""
        return int(np.argmax([self.spread(k) for k in range(self.n_frames)]))


def _run_labels(runs, paths, labels):
    """Legend labels: explicit, else distinct run ids, else file stems."""
    if labels:
        labels = list(labels)
        if len(labels) != len(paths):
            raise ValueError("got %d labels for %d traces"
                             % (len(labels), len(paths)))
        return [str(x) for x in labels]
    ids = [str(r.repro.get("run_id") or "") for r in runs]
    if all(ids) and len(set(ids)) == len(ids):
        return ids
    stems = [p.stem for p in paths]
    if len(set(stems)) == len(stems):
        return stems
    return ["%s [%d]" % (s, i) for i, s in enumerate(stems)]


def _run_colors(n: int, colors=None):
    """``n`` run colours, cycling :data:`RUN_COLORS` when none are given."""
    if colors:
        colors = list(colors)
        if len(colors) < n:
            raise ValueError("got %d colours for %d traces" % (len(colors), n))
        return [str(c) for c in colors[:n]]
    return [RUN_COLORS[i % len(RUN_COLORS)] for i in range(n)]


def multi_frame_spec(scene: MultiScene, k: int) -> dict:
    """Describe overlay frame ``k`` — pure function, no matplotlib, no I/O.

    :param scene: loaded scene.
    :param k: index into :attr:`MultiScene.times`.
    :returns: dict with the shared ``xlim``/``ylim`` and one entry per run
        (``body``, ``wheels``, ``util``, ``arrow``, trail arrays, ``color``,
        ``alpha_scale`` and the HUD row).
    """
    k = int(k)
    t = float(scene.times[k])
    xlim, ylim = scene.window(k)
    runs = []
    for idx, (run, track) in enumerate(zip(scene.runs, scene.tracks)):
        x, y = float(track["x"][k]), float(track["y"][k])
        yaw = float(track["yaw"][k])
        delta = float(track["steer"][k])
        vx, vy = float(track["vx"][k]), float(track["vy"][k])
        speed = math.hypot(vx, vy)
        util = [float(u) for u in track["util"][k]]
        active = bool(track["active"][k])
        body, wheels = body_and_wheels(run.geometry, run.body_l, run.body_w,
                                       x, y, yaw, delta)
        # The trail stops growing at the run's own end, so a finished run does
        # not keep drawing a line while the others carry on.
        last = k if active else int(np.count_nonzero(track["active"])) - 1
        last = max(last, 0)
        runs.append({
            "index": idx,
            "label": scene.labels[idx],
            "color": scene.colors[idx],
            "active": active,
            "alpha_scale": 1.0 if active else ENDED_ALPHA_SCALE,
            "x": x, "y": y, "yaw": yaw, "speed": speed,
            "body": body,
            "wheels": wheels,
            "util": util,
            "arrow": velocity_arrow(run.geometry, x, y, yaw, vx, vy),
            "trail_x": track["x"][: last + 1],
            "trail_y": track["y"][: last + 1],
            "hud": "%-14.14s v %5.1f km/h  util %4.2f%s" % (
                scene.labels[idx], speed * 3.6,
                max(util) if util else 0.0, "" if active else "  (ended)"),
        })
    return {
        "index": k,
        "t": t,
        "xlim": xlim,
        "ylim": ylim,
        "runs": runs,
        "hud_time": "t = %6.2f s" % (t,),
    }


def _setup_multi_axes(scene: MultiScene, figsize, dpi):
    """Figure with the BEV on the left and one panel per channel + utilization."""
    n_panels = len(scene.series_names) + 1
    fig = plt.figure(figsize=figsize, dpi=dpi)
    gs = GridSpec(n_panels, 2, figure=fig, width_ratios=[1.45, 1.0],
                  hspace=0.32, wspace=0.24,
                  left=0.07, right=0.975, top=0.90, bottom=0.10)
    ax_bev = fig.add_subplot(gs[:, 0])
    ax_panels = [fig.add_subplot(gs[k, 1]) for k in range(n_panels)]
    return fig, ax_bev, ax_panels


def _draw_multi_static(scene: MultiScene, ax_bev, ax_panels, alpha: float,
                       path_alpha: float, ghost: bool = True):
    """Static overlay content plus one artist set per run."""
    ax_bev.set_aspect("equal", adjustable="box")
    ax_bev.set_xlabel("X [m]")
    ax_bev.set_ylabel("Y [m]")
    ax_bev.grid(True, alpha=0.25, lw=0.5)

    # Scenario overlays are drawn once even when several runs carry the same
    # ones: they describe the world, not the run.
    seen = set()
    for run in scene.runs:
        for reg in run.region_overlays():
            key = ("region", reg.get("name"))
            poly = reg.get("polygon") or []
            if key in seen or len(poly) < 3:
                continue
            seen.add(key)
            ax_bev.add_collection(PolyCollection(
                [poly], facecolors="#c44e52", alpha=0.16,
                edgecolors="#c44e52", linewidths=0.8, zorder=0))
        for ref in run.path2d_overlays():
            key = ("path2d", ref.get("name"))
            xy = np.asarray(ref.get("xy") or [], dtype=float)
            if key in seen or not xy.size:
                continue
            seen.add(key)
            ax_bev.plot(xy[:, 0], xy[:, 1], ls="--", lw=1.2, color="#555555",
                        alpha=0.8, zorder=1, label=ref.get("name", "reference"))

    artists = []
    for idx, run in enumerate(scene.runs):
        color = scene.colors[idx]
        if ghost:
            ax_bev.plot(run.pose[:, 0], run.pose[:, 1], "-", lw=1.0, color=color,
                        alpha=GHOST_ALPHA, zorder=1)
        (trail,) = ax_bev.plot([], [], "-", lw=1.6, color=color,
                               alpha=path_alpha, zorder=2 + idx,
                               label=scene.labels[idx])
        body = PolyCollection([], facecolors=color, edgecolors=WHEEL_EDGE,
                              linewidths=1.1, alpha=alpha, zorder=10 + idx)
        ax_bev.add_collection(body)
        wheels = PolyCollection([], facecolors=color, edgecolors=WHEEL_EDGE,
                                linewidths=0.9, zorder=30 + idx)
        ax_bev.add_collection(wheels)
        arrow = ax_bev.annotate(
            "", xy=(0, 0), xytext=(0, 0), zorder=50 + idx,
            arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6,
                            shrinkA=0, shrinkB=0))
        hud = ax_bev.text(
            0.015, 0.975 - 0.045 * (idx + 1), "", transform=ax_bev.transAxes,
            va="top", ha="left", fontsize=7.5, family="monospace",
            color=color, zorder=100,
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.75))
        artists.append(dict(trail=trail, body=body, wheels=wheels, arrow=arrow,
                            hud=hud))

    hud_time = ax_bev.text(
        0.015, 0.985, "", transform=ax_bev.transAxes, va="top", ha="left",
        fontsize=8, family="monospace", color="#111111", zorder=100,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#8a97a5", alpha=0.85))
    ax_bev.legend(loc="lower left", fontsize=7, framealpha=0.6, ncol=1)

    for ax, name in zip(ax_panels, scene.series_names):
        source_unit = scene.series_unit(name)
        display_unit = source_unit
        for idx, run in enumerate(scene.runs):
            arr = scene.series_of(run, name)
            if arr is None:
                continue
            display_unit, display_arr = series_for_display(source_unit, arr)
            ax.plot(run.t, display_arr, lw=1.1,
                    color=scene.colors[idx], alpha=0.9)
        ax.set_ylabel("%s [%s]" % (name, display_unit or "-"), fontsize=8)
    ax_util = ax_panels[-1]
    for idx, run in enumerate(scene.runs):
        ax_util.plot(run.t, run.util.max(axis=1), lw=1.1,
                     color=scene.colors[idx], alpha=0.9)
    ax_util.axhline(UTIL_WARN, color="#111111", lw=0.9, ls="--")
    ax_util.set_ylabel("max util [-]", fontsize=8)
    ax_util.set_ylim(0.0, UTIL_MAX)

    lines = []
    for ax in ax_panels:
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25, lw=0.5)
        ax.set_xlim(scene.t0, scene.t1)
        for idx, run in enumerate(scene.runs):
            for ev in run.event_overlays():
                for et in ev.get("t", []):
                    ax.axvline(float(et), color=scene.colors[idx], lw=0.8,
                               alpha=0.5, ls=":")
        lines.append(ax.axvline(scene.t0, color="#111111", lw=1.0))
    ax_panels[-1].set_xlabel("time [s]", fontsize=8)
    return dict(runs=artists, hud_time=hud_time, cursors=lines)


def draw_multi_frame(artists, ax_bev, spec: dict, alpha: float = BODY_ALPHA,
                     path_alpha: float = PATH_ALPHA):
    """Apply a :func:`multi_frame_spec` description to the prepared artists."""
    ax_bev.set_xlim(*spec["xlim"])
    ax_bev.set_ylim(*spec["ylim"])
    artists["hud_time"].set_text(spec["hud_time"])
    for art, r in zip(artists["runs"], spec["runs"]):
        scale = r["alpha_scale"]
        art["trail"].set_data(r["trail_x"], r["trail_y"])
        art["trail"].set_alpha(path_alpha * scale)
        art["body"].set_verts([r["body"]])
        art["body"].set_alpha(alpha * scale)
        art["wheels"].set_verts(r["wheels"])
        art["wheels"].set_edgecolors(
            [WHEEL_EDGE_WARN if u >= UTIL_WARN else WHEEL_EDGE for u in r["util"]])
        art["wheels"].set_alpha(min(1.0, alpha + 0.3) * scale)
        x, y, dx, dy = r["arrow"]
        art["arrow"].set_position((x, y))
        art["arrow"].xy = (x + dx, y + dy)
        art["arrow"].set_alpha(scale)
        art["hud"].set_text(r["hud"])
        art["hud"].set_alpha(0.45 + 0.55 * scale)
    for cur in artists["cursors"]:
        cur.set_xdata([spec["t"], spec["t"]])


def render_multi(trace_paths, out=None, png=None, fps: int = 20,
                 speed: float = 1.0, dpi: int = 100, figsize=(11.0, 5.6),
                 mp4: bool = False, title: str = None, labels=None, colors=None,
                 alpha: float = BODY_ALPHA, path_alpha: float = PATH_ALPHA,
                 follow="fit", ghost: bool = True, quiet: bool = False) -> dict:
    """Render several traces overlaid in one BEV animation.

    :param trace_paths: two or more ``.vdtrace`` paths.
    :param out: output GIF path; defaults to ``<first trace>_overlay.gif``.
    :param png: preview PNG path; defaults to ``<out stem>_preview.png``.
    :param fps: output frame rate.
    :param speed: playback speed relative to wall-clock.
    :param dpi: figure DPI.
    :param figsize: figure size in inches.
    :param mp4: also write an MP4 when ``imageio-ffmpeg`` is installed.
    :param title: figure suptitle.
    :param labels: per-run legend labels.
    :param colors: per-run colours.
    :param alpha: body fill alpha — lower it when runs overlap heavily.
    :param path_alpha: driven-trail alpha.
    :param follow: ``"fit"`` (static window over every run) or a run index.
    :param ghost: draw each run's full route faintly under the animation.
    :param quiet: suppress the progress line.
    :returns: dict with ``gif``, ``png``, ``mp4``, ``frames``, ``runs``,
        ``labels``, ``max_spread_m`` and ``wall_s``.
    """
    t_start = time.time()
    trace_paths = [Path(p) for p in trace_paths]
    out = Path(out) if out else trace_paths[0].with_name(
        trace_paths[0].stem + "_overlay.gif")
    png = Path(png) if png else out.with_name(out.stem + "_preview.png")

    scene = MultiScene(trace_paths, labels=labels, colors=colors, fps=fps,
                       speed=speed, follow=follow)
    if scene.n_frames == 0:
        raise ValueError("no overlay frames: traces carry no samples")

    fig, ax_bev, ax_panels = _setup_multi_axes(scene, figsize, dpi)
    fig.suptitle(title or _default_multi_title(scene), fontsize=9)
    artists = _draw_multi_static(scene, ax_bev, ax_panels, alpha, path_alpha,
                                 ghost=ghost)

    def _update(k):
        draw_multi_frame(artists, ax_bev, multi_frame_spec(scene, k),
                         alpha=alpha, path_alpha=path_alpha)
        return ()

    peak = scene.preview_frame()
    draw_multi_frame(artists, ax_bev, multi_frame_spec(scene, peak),
                     alpha=alpha, path_alpha=path_alpha)
    png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(png), dpi=dpi)

    anim = FuncAnimation(fig, _update, frames=scene.n_frames,
                         interval=1000 // max(fps, 1), blit=False, repeat=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(str(out), writer=PillowWriter(fps=fps))

    mp4_path = None
    if mp4:
        try:
            import imageio_ffmpeg  # noqa: F401
            from matplotlib.animation import FFMpegWriter
            mp4_path = out.with_suffix(".mp4")
            matplotlib.rcParams["animation.ffmpeg_path"] = \
                imageio_ffmpeg.get_ffmpeg_exe()
            anim.save(str(mp4_path), writer=FFMpegWriter(fps=fps))
        except ImportError:
            mp4_path = None
            if not quiet:
                print("mp4 skipped: imageio-ffmpeg not installed (gif written)")
    plt.close(fig)

    wall = time.time() - t_start
    result = {"gif": out, "png": png, "mp4": mp4_path,
              "frames": scene.n_frames, "runs": len(scene.runs),
              "labels": list(scene.labels),
              "max_spread_m": float(scene.spread(peak)),
              "wall_s": wall,
              "trace_duration_s": float(scene.t1 - scene.t0)}
    if not quiet:
        print("wrote %s (%d runs, %d frames, %.1f fps, max spread %.2f m) "
              "+ %s in %.2fs wall-clock"
              % (out, len(scene.runs), scene.n_frames, fps,
                 result["max_spread_m"], png.name, wall))
    return result


def _default_multi_title(scene: MultiScene) -> str:
    return "VDSim overlay — %d runs: %s" % (
        len(scene.runs), ", ".join(scene.labels))


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def render(trace_path, out=None, png=None, fps: int = 20, stride: int = None,
           dpi: int = 100, figsize=(11.0, 5.6), mp4: bool = False,
           title: str = None, quiet: bool = False, sidecar=None,
           preview_frame: str = "util-peak", view_half: float = None) -> dict:
    """Render one trace to a GIF (+ preview PNG).

    :param trace_path: input ``.vdtrace``.
    :param out: output GIF path; defaults to the trace path with ``.gif``.
    :param png: preview PNG path; defaults to ``<out stem>_preview.png``.
    :param fps: output frame rate.
    :param stride: recorded samples per output frame. ``None`` derives it from
        the record rate so the animation plays at wall-clock speed.
    :param dpi: figure DPI.
    :param figsize: figure size in inches.
    :param mp4: also write an MP4; requires ``imageio-ffmpeg`` and is skipped
        with a warning when it is unavailable.
    :param title: figure suptitle.
    :param quiet: suppress the progress line.
    :param sidecar: explicit horizons ``.qp.npz``; ``None`` auto-detects
        ``<trace>.qp.npz`` and renders without horizons when absent.
    :param preview_frame: ``"util-peak"`` (default, unchanged behaviour) or
        ``"first-fail"`` to put the preview on the first failed solve.
    :param view_half: BEV half-window [m]. ``None`` keeps the geometry-derived
        tracking window, which is ~6 wheelbases and therefore much shorter
        than a typical MPC horizon — widen it to put the horizons on screen.
    :returns: dict with ``gif``, ``png``, ``mp4``, ``firstfail_png``,
        ``n_qp_fail``, ``frames`` and ``wall_s``.
    """
    if preview_frame not in ("util-peak", "first-fail"):
        raise ValueError("preview_frame must be util-peak|first-fail, got %r"
                         % (preview_frame,))
    t_start = time.time()
    trace_path = Path(trace_path)
    out = Path(out) if out else trace_path.with_suffix(".gif")
    png = Path(png) if png else out.with_name(out.stem + "_preview.png")

    if stride is None:
        with TraceReader(trace_path) as probe:
            dt = float(probe.repro.get("dt_s") or 0.0)
        stride = max(1, int(round(1.0 / (fps * dt)))) if dt > 0 else 1
    tr = LoadedTrace(trace_path, stride=stride, sidecar=sidecar)
    if tr.n_frames == 0:
        raise ValueError("trace has no samples to render: %s" % (trace_path,))

    if view_half is not None:
        tr.view_half_used = float(view_half) * (1.0 + VIEW_MARGIN_FRAC)
    elif tr.horizon_reach > tr.view_half and not quiet:
        print("note: horizons reach %.0f m but the window is +-%.0f m; pass --view-half to widen"
              % (tr.horizon_reach, tr.view_half))

    fig, ax_bev, ax_series = _setup_axes(tr, figsize, dpi)
    fig.suptitle(title or _default_title(tr), fontsize=9)
    artists = _draw_static(tr, ax_bev, ax_series)

    def _update(k):
        draw_frame(artists, ax_bev, frame_spec(tr, int(tr.frames[k])))
        return ()

    # Preview PNG at peak utilization: the frame where the 8 required screen
    # items are all actually exercised, rather than an arbitrary first frame.
    # ``first-fail`` moves it to the first failed solve instead, which is the
    # frame a QP-failure post-mortem actually wants.
    peak = int(tr.frames[int(np.argmax(tr.util[tr.frames].max(axis=1)))])
    first_fail = tr.first_fail_frame()
    chosen = peak
    if preview_frame == "first-fail":
        if first_fail is None:
            if not quiet:
                print("preview: no failed solve in this run; using util-peak")
        else:
            chosen = first_fail
    draw_frame(artists, ax_bev, frame_spec(tr, chosen))
    png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(png), dpi=dpi)

    # The first failed solve always gets its own PNG when there is one, so
    # the evidence frame does not depend on the preview mode asked for.
    fail_png = None
    if first_fail is not None:
        fail_png = out.with_name(out.stem + "_firstfail.png")
        draw_frame(artists, ax_bev, frame_spec(tr, first_fail))
        fig.savefig(str(fail_png), dpi=dpi)

    anim = FuncAnimation(fig, _update, frames=tr.n_frames,
                         interval=1000 // max(fps, 1), blit=False, repeat=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(str(out), writer=PillowWriter(fps=fps))

    mp4_path = None
    if mp4:
        try:
            import imageio_ffmpeg  # noqa: F401
            from matplotlib.animation import FFMpegWriter
            mp4_path = out.with_suffix(".mp4")
            matplotlib.rcParams["animation.ffmpeg_path"] = \
                imageio_ffmpeg.get_ffmpeg_exe()
            anim.save(str(mp4_path), writer=FFMpegWriter(fps=fps))
        except ImportError:
            mp4_path = None
            if not quiet:
                print("mp4 skipped: imageio-ffmpeg not installed (gif written)")
    plt.close(fig)

    wall = time.time() - t_start
    result = {"gif": out, "png": png, "mp4": mp4_path,
              "firstfail_png": fail_png,
              "view_half_m": float(tr.view_half_used),
              "horizon_reach_m": float(tr.horizon_reach),
              "n_qp_fail": int(artists.get("n_qp_fail", 0)),
              "frames": tr.n_frames, "wall_s": wall,
              "trace_duration_s": float(tr.t[-1] - tr.t[0])}
    if not quiet:
        print("wrote %s (%d frames, %.1f fps) + %s in %.2fs wall-clock"
              % (out, tr.n_frames, fps, png.name, wall))
    return result


def _default_title(tr: LoadedTrace) -> str:
    rep = tr.repro
    return "VDSim trace — run %s | %s | %s" % (
        rep.get("run_id", "?"), rep.get("param_hash", "")[:19],
        rep.get("git_sha", "")[:9])


def main(argv=None) -> int:
    """CLI entry point (``vdsim-render``)."""
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("trace", type=Path, nargs="+",
                    help="input .vdtrace; two or more are overlaid in one view")
    ap.add_argument("--out", type=Path, default=None, help="output GIF path")
    ap.add_argument("--png", type=Path, default=None, help="preview PNG path")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--stride", type=int, default=None,
                    help="single-run only: recorded samples per frame "
                         "(default: wall-clock speed)")
    ap.add_argument("--dpi", type=int, default=100)
    ap.add_argument("--mp4", action="store_true",
                    help="also write MP4 (needs imageio-ffmpeg)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--sidecar", type=Path, default=None,
                    help="single-run only: .qp.npz carrying the MPC target "
                         "and prediction horizons "
                         "(default: <trace>.qp.npz when it exists)")
    ap.add_argument("--view-half", dest="view_half", type=float, default=None,
                    help="single-run only: BEV half-window [m]. The default "
                         "tracking window is ~6 wheelbases, far shorter than "
                         "an MPC horizon, so widen it to see target/prediction")
    ap.add_argument("--preview-frame", dest="preview_frame",
                    default="util-peak",
                    choices=("util-peak", "first-fail"),
                    help="frame shown in the preview PNG "
                         "(default: %(default)s)")
    grp = ap.add_argument_group("overlay mode (2+ traces)")
    grp.add_argument("--labels", default=None,
                     help="comma-separated legend labels, one per trace "
                          "(default: run ids, else file stems)")
    grp.add_argument("--colors", default=None,
                     help="comma-separated matplotlib colours, one per trace")
    grp.add_argument("--alpha", type=float, default=BODY_ALPHA,
                     help="body fill alpha, for overlapping vehicles "
                          "(default: %(default)s)")
    grp.add_argument("--path-alpha", type=float, default=PATH_ALPHA,
                     help="driven-path alpha (default: %(default)s)")
    grp.add_argument("--follow", default="fit",
                     help="'fit' (static window holding every run) or a run "
                          "index to track (default: %(default)s)")
    grp.add_argument("--speed", type=float, default=1.0,
                     help="playback speed vs wall-clock (default: %(default)s)")
    grp.add_argument("--no-ghost", action="store_true",
                     help="do not draw each run's full route faintly")
    args = ap.parse_args(argv)

    if len(args.trace) == 1:
        render(args.trace[0], out=args.out, png=args.png, fps=args.fps,
               stride=args.stride, dpi=args.dpi, mp4=args.mp4,
               title=args.title, sidecar=args.sidecar,
               preview_frame=args.preview_frame, view_half=args.view_half)
        return 0

    if args.stride is not None:
        print("note: --stride is ignored in overlay mode; use --speed instead")
    render_multi(args.trace, out=args.out, png=args.png, fps=args.fps,
                 speed=args.speed, dpi=args.dpi, mp4=args.mp4, title=args.title,
                 labels=_split_list(args.labels), colors=_split_list(args.colors),
                 alpha=args.alpha, path_alpha=args.path_alpha,
                 follow=args.follow, ghost=not args.no_ghost)
    return 0


def _split_list(value):
    """``"a,b"`` -> ``["a", "b"]``; ``None``/empty -> ``None``."""
    if not value:
        return None
    return [part.strip() for part in str(value).split(",") if part.strip()]


if __name__ == "__main__":
    raise SystemExit(main())
