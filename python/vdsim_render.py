"""vdsim_render — headless renderer for a single ``.vdtrace``.

Input is one trace file and nothing else: no simulation is re-run and no
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

Structure: :func:`frame_spec` is a pure function from (loaded trace, frame
index) to a plain description of the frame, and :func:`draw_frame` renders that
description. The split is what makes the view window testable — axis limits
come from manifest ``geometry`` and never from matplotlib autoscale, which
would otherwise drift frame to frame and silently rescale the animation.

CLI::

    python -m vdsim_render run.vdtrace --out run.gif
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


class LoadedTrace:
    """Every array the renderer needs, loaded once from a ``.vdtrace``.

    Holding the decoded arrays separately from the reader keeps
    :func:`frame_spec` free of file I/O, so it stays a pure function.

    :param path: ``.vdtrace`` path.
    :param stride: keep every ``stride``-th recorded sample as one output frame.
    """

    def __init__(self, path, stride: int = 1):
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


def frame_spec(tr: LoadedTrace, i: int) -> dict:
    """Describe frame ``i`` — pure function, no matplotlib, no file access.

    :param tr: loaded trace.
    :param i: index into the *recorded* samples (not the strided frame list).
    :returns: dict with ``xlim``/``ylim`` (from manifest geometry), ``body``
        and ``wheels`` polygons, ``wheel_util`` colours' source values,
        ``arrow``, ``trail``, ``hud`` lines and the time cursor.
    """
    x, y, yaw = (float(v) for v in tr.pose[i])
    half = tr.view_half
    wb = float(tr.geometry["wheelbase_m"])
    track_f = float(tr.geometry.get("track_front_m", tr.geometry["track_m"]))
    track_r = float(tr.geometry.get("track_rear_m", tr.geometry["track_m"]))
    lf = float(tr.geometry.get("cg_to_front_m", 0.5 * wb))
    lr = float(tr.geometry.get("cg_to_rear_m", wb - 0.5 * wb))
    radius = float(tr.geometry.get("wheel_radius_m", 0.34))
    delta = float(tr.steer[i])

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

    vx, vy = (float(v) for v in tr.v_body[i])
    speed = math.hypot(vx, vy)
    arrow_len = ARROW_S_PER_MPS * wb * speed
    ax_, ay_ = (c * vx - s * vy, s * vx + c * vy)
    norm = max(math.hypot(ax_, ay_), 1e-9)
    util = [float(u) for u in tr.util[i]]

    return {
        "index": int(i),
        "t": float(tr.t[i]),
        "xlim": (x - half, x + half),
        "ylim": (y - half, y + half),
        "body": _rect(*to_world(0.5 * (lf - lr), 0.0), tr.body_l, tr.body_w, yaw),
        "wheels": wheels,
        "wheel_util": util,
        "arrow": (x, y, ax_ / norm * arrow_len, ay_ / norm * arrow_len),
        "trail_x": tr.pose[: i + 1, 0],
        "trail_y": tr.pose[: i + 1, 1],
        "hud": [
            "t = %6.2f s" % (float(tr.t[i]),),
            "v = %5.2f m/s  (%5.1f km/h)" % (speed, speed * 3.6),
            "beta = %+5.1f deg" % (math.degrees(math.atan2(vy, max(abs(vx), 1e-6))),),
            "r = %+6.2f deg/s" % (math.degrees(float(tr.yaw_rate[i])),),
            "delta = %+5.2f deg" % (math.degrees(delta),),
            "util max = %4.2f  [%s]" % (
                max(util) if util else 0.0,
                " ".join("%s %.2f" % (w, u) for w, u in zip(tr.wheels, util))),
        ],
    }


# --------------------------------------------------------------------------
# drawing
# --------------------------------------------------------------------------

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
            ax_bev.plot(xy[:, 0], xy[:, 1], ls="--", lw=1.2, color="#4c72b0",
                        alpha=0.9, zorder=1, label=ref.get("name", "reference"))
    if tr.path2d_overlays():
        # Lower left: the HUD owns the top-left corner in screen coordinates.
        ax_bev.legend(loc="lower left", fontsize=7, framealpha=0.6)

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

    cbar = ax_bev.figure.colorbar(
        plt.cm.ScalarMappable(norm=UTIL_NORM, cmap=UTIL_CMAP),
        ax=ax_bev, fraction=0.035, pad=0.015)
    cbar.set_label("tyre friction utilization [-]", fontsize=8)
    cbar.ax.axhline(UTIL_WARN, color="#111111", lw=1.0, ls="--")
    cbar.ax.tick_params(labelsize=7)

    cursors = []
    events = tr.event_overlays()
    for ax, (name, unit, arr) in zip(ax_series, tr.series):
        ax.plot(tr.t, arr, lw=1.1, color="#4c72b0")
        ax.set_ylabel("%s [%s]" % (name, unit or "-"), fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25, lw=0.5)
        ax.set_xlim(float(tr.t[0]), float(tr.t[-1]))
        for ev in events:
            for et in ev.get("t", []):
                ax.axvline(float(et), color="#c02020", lw=0.8, alpha=0.6)
        cursors.append(ax.axvline(float(tr.t[0]), color="#111111", lw=1.0))
    if ax_series:
        ax_series[-1].set_xlabel("time [s]", fontsize=8)
    return dict(trail=trail, body=body, wheels=wheels, arrow=arrow,
                hud=hud, cursors=cursors)


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
    for cur in artists["cursors"]:
        cur.set_xdata([spec["t"], spec["t"]])


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------

def render(trace_path, out=None, png=None, fps: int = 20, stride: int = None,
           dpi: int = 100, figsize=(11.0, 5.6), mp4: bool = False,
           title: str = None, quiet: bool = False) -> dict:
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
    :returns: dict with ``gif``, ``png``, ``mp4``, ``frames`` and ``wall_s``.
    """
    t_start = time.time()
    trace_path = Path(trace_path)
    out = Path(out) if out else trace_path.with_suffix(".gif")
    png = Path(png) if png else out.with_name(out.stem + "_preview.png")

    if stride is None:
        with TraceReader(trace_path) as probe:
            dt = float(probe.repro.get("dt_s") or 0.0)
        stride = max(1, int(round(1.0 / (fps * dt)))) if dt > 0 else 1
    tr = LoadedTrace(trace_path, stride=stride)
    if tr.n_frames == 0:
        raise ValueError("trace has no samples to render: %s" % (trace_path,))

    fig, ax_bev, ax_series = _setup_axes(tr, figsize, dpi)
    fig.suptitle(title or _default_title(tr), fontsize=9)
    artists = _draw_static(tr, ax_bev, ax_series)

    def _update(k):
        draw_frame(artists, ax_bev, frame_spec(tr, int(tr.frames[k])))
        return ()

    # Preview PNG at peak utilization: the frame where the 8 required screen
    # items are all actually exercised, rather than an arbitrary first frame.
    peak = int(tr.frames[int(np.argmax(tr.util[tr.frames].max(axis=1)))])
    draw_frame(artists, ax_bev, frame_spec(tr, peak))
    png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(png), dpi=dpi)

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
    ap.add_argument("trace", type=Path, help="input .vdtrace")
    ap.add_argument("--out", type=Path, default=None, help="output GIF path")
    ap.add_argument("--png", type=Path, default=None, help="preview PNG path")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--stride", type=int, default=None,
                    help="recorded samples per frame (default: wall-clock speed)")
    ap.add_argument("--dpi", type=int, default=100)
    ap.add_argument("--mp4", action="store_true",
                    help="also write MP4 (needs imageio-ffmpeg)")
    ap.add_argument("--title", default=None)
    args = ap.parse_args(argv)
    render(args.trace, out=args.out, png=args.png, fps=args.fps,
           stride=args.stride, dpi=args.dpi, mp4=args.mp4, title=args.title)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
