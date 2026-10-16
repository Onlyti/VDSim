"""Live 3D monitor: watch a running simulation in a browser.

``LiveMonitor`` publishes the vehicle state from any loop that owns a
simulation -- a ``vdsim.RealTimeRunner`` session, a
:class:`vdsim_closedloop.ClosedLoopEval` controller callback, a ROS node -- to
a small page server that draws it with three.js::

    from vdsim_livemonitor import LiveMonitor
    with LiveMonitor(port=8790) as mon:
        while running:
            ...
            mon.push(obs)          # dict with X, Y, psi (or x, y, yaw)

The page server is the :class:`MonitorServer`. It serves one static page plus
the vendored three.js bundle over HTTP and streams the latest state over a
WebSocket at ``rate_hz``. Publishers reach it with JSON datagrams on
``127.0.0.1:<port>`` (UDP), so ``push`` never blocks the simulation loop.

Two ways to run the server:

* embedded -- ``LiveMonitor.start()`` binds the port itself when nothing is
  listening there and serves from a thread of the simulation process. The page
  is reachable only while that process runs.
* standing -- ``python -m vdsim_livemonitor serve --port 8790`` keeps the page
  up with no simulation. The page shows "waiting for simulator" and the state
  appears as soon as a ``LiveMonitor`` on the same host starts publishing; the
  browser reconnects by itself when the server restarts. ``LiveMonitor``
  detects a standing server on its port and publishes to it instead of
  binding.

OSM: ``osm=`` draws an OpenStreetMap ``.osm`` file on the ground, display only.
Ways tagged ``highway=*`` become road ribbons; Lanelet2 line ways
(``type=line_thin|line_thick|line|curbstone|road_border|virtual``) become
lines. Node coordinates are mapped to the simulation frame (x east, y north)
by the local tangent plane at ``origin=(lat, lon[, h])``; without an origin
the OSM file is refused.

Standard library only; no new Python dependency.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import urllib.request
import webbrowser
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

__all__ = ["LiveMonitor", "MonitorServer", "geodetic_to_enu", "load_osm",
           "vehicle_geometry", "write_shortcut", "DEFAULT_PORT"]

#: Default TCP (page) and UDP (ingest) port.
DEFAULT_PORT = 8790
#: Service id answered on ``/health``; a publisher checks it before using a
#: server it did not start.
SERVICE_ID = "vdsim-livemonitor"
PROTOCOL_VERSION = 1
#: A state older than this [s] puts the page in "waiting for simulator".
LIVE_TIMEOUT_S = 1.0
#: Publisher re-sends its scene this often [s], so a server restarted
#: mid-run gets the OSM and vehicle back.
SCENE_RESEND_S = 2.0

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

# WGS84 defining parameters (same values as cosim/comms_templates.hpp).
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

#: Default vehicle drawing geometry [m]; any key can be overridden.
DEFAULT_VEHICLE = {
    "cg_to_front": 1.35, "cg_to_rear": 1.65, "track": 1.6,
    "front_overhang": 0.9, "rear_overhang": 0.95,
    "width": 1.85, "height": 1.45, "wheel_radius": 0.33, "wheel_width": 0.24,
}

#: Lanelet2 line ``type`` values drawn as lines.
LANELET_LINE_TYPES = ("line_thin", "line_thick", "line", "curbstone",
                      "road_border", "virtual")

#: Ribbon width [m] per ``highway`` value; others use ``"default"``.
HIGHWAY_WIDTH_M = {"motorway": 10.0, "trunk": 9.0, "primary": 8.0,
                   "secondary": 7.0, "tertiary": 6.0, "residential": 5.0,
                   "service": 4.0, "footway": 2.0, "path": 1.5,
                   "default": 5.0}


def _web_dir() -> Path:
    """Directory of ``index.html`` / ``monitor.js``.

    :returns: the installed ``vdsim_livemonitor_web`` next to this module.
    """
    return Path(__file__).resolve().parent / "vdsim_livemonitor_web"


def _three_dir() -> Path:
    """Directory holding ``three.module.js`` and ``addons/``.

    :returns: the copy installed inside the web directory (wheel), else the
        repository's ``gui/vendor/three`` (source tree).
    :raises FileNotFoundError: neither exists.
    """
    for d in (_web_dir() / "three",
              Path(__file__).resolve().parents[1] / "gui" / "vendor" / "three"):
        if (d / "three.module.js").is_file():
            return d
    raise FileNotFoundError("three.js bundle not found (gui/vendor/three)")


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------
def geodetic_to_enu(lat_deg: float, lon_deg: float, h_m: float,
                    origin) -> tuple:
    """WGS84 geodetic -> local east/north/up at ``origin`` (tangent plane).

    Exact forward transform (geodetic -> ECEF -> rotation into the ENU frame
    at the origin), the same mapping as a GeographicLib ``LocalCartesian``.

    :param lat_deg: latitude [deg].
    :param lon_deg: longitude [deg].
    :param h_m: ellipsoidal height [m].
    :param origin: ``(lat0 [deg], lon0 [deg])`` or ``(lat0, lon0, h0 [m])``.
    :returns: ``(east, north, up)`` [m].
    :raises ValueError: origin latitude outside [-90, 90] or not finite.
    """
    lat0, lon0 = float(origin[0]), float(origin[1])
    h0 = float(origin[2]) if len(origin) > 2 else 0.0
    if not (math.isfinite(lat0) and math.isfinite(lon0) and -90.0 <= lat0 <= 90.0):
        raise ValueError(f"invalid origin {tuple(origin)}")

    def ecef(lat, lon, h):
        p, l = math.radians(lat), math.radians(lon)
        n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(p) ** 2)
        return ((n + h) * math.cos(p) * math.cos(l),
                (n + h) * math.cos(p) * math.sin(l),
                (n * (1.0 - _WGS84_E2) + h) * math.sin(p))

    x, y, z = ecef(lat_deg, lon_deg, h_m)
    x0, y0, z0 = ecef(lat0, lon0, h0)
    dx, dy, dz = x - x0, y - y0, z - z0
    p0, l0 = math.radians(lat0), math.radians(lon0)
    sp, cp, sl, cl = math.sin(p0), math.cos(p0), math.sin(l0), math.cos(l0)
    east = -sl * dx + cl * dy
    north = -sp * cl * dx - sp * sl * dy + cp * dz
    up = cp * cl * dx + cp * sl * dy + sp * dz
    return east, north, up


def load_osm(path, origin) -> list:
    """Read the drawable ways of an ``.osm`` file in the simulation frame.

    :param path: OSM XML file.
    :param origin: ``(lat0, lon0[, h0])`` of the simulation frame origin.
    :returns: list of ``{"kind": "highway"|"lane", "cls": str,
        "width": float, "pts": [[x, y], ...]}``; ``x`` east, ``y`` north [m].
    :raises ValueError: ``origin`` is None (refused, see module doc) or
        invalid; the file is not OSM XML.
    :raises FileNotFoundError: ``path`` does not exist.
    """
    if origin is None:
        raise ValueError(
            "OSM refused: no origin latitude/longitude. The map can only be "
            "placed in the simulation frame relative to the geodetic origin of "
            "that frame; pass origin=(lat, lon).")
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"OSM file not found: {path}")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        raise ValueError(f"not an OSM XML file: {path} ({e})") from e
    if root.tag != "osm":
        raise ValueError(f"not an OSM XML file: {path} (root <{root.tag}>)")
    nodes = {}
    for n in root.iter("node"):
        try:
            nodes[n.get("id")] = (float(n.get("lat")), float(n.get("lon")))
        except (TypeError, ValueError):
            continue
    cache = {}
    out = []
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.iter("tag")}
        if "highway" in tags:
            kind, cls = "highway", tags["highway"]
            width = HIGHWAY_WIDTH_M.get(cls, HIGHWAY_WIDTH_M["default"])
        elif tags.get("type") in LANELET_LINE_TYPES:
            kind, cls, width = "lane", tags["type"], 0.0
        else:
            continue
        pts = []
        for nd in w.iter("nd"):
            ref = nd.get("ref")
            if ref not in nodes:
                continue
            if ref not in cache:
                e, n_, _ = geodetic_to_enu(nodes[ref][0], nodes[ref][1], 0.0, origin)
                cache[ref] = [round(e, 3), round(n_, 3)]
            pts.append(cache[ref])
        if len(pts) >= 2:
            out.append({"kind": kind, "cls": cls, "width": width, "pts": pts})
    return out


def _finite_or_none(v):
    """JSON-safe number: ``None`` for NaN/inf, ``float`` otherwise."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _compact_state(state: dict) -> dict:
    """Map a state/observation dict to the monitor's wire keys.

    Accepts the :class:`vdsim_plant.VDSimPlant` observation keys
    (``X``, ``Y``, ``psi``, ``vx``, ``delta_rw``, ``roll``, ``pitch``, ``t``)
    or the short ones (``x``, ``y``, ``yaw``, ``v``, ``delta``).

    :param state: the caller's state.
    :returns: dict with ``x``, ``y``, ``yaw`` and any of ``t``, ``vx``,
        ``delta``, ``roll``, ``pitch``, ``z`` that were present.
    :raises KeyError: no position or heading key.
    """
    def pick(*keys):
        for k in keys:
            if k in state and state[k] is not None:
                return state[k]
        return None

    x, y, yaw = pick("X", "x"), pick("Y", "y"), pick("psi", "yaw")
    if x is None or y is None or yaw is None:
        raise KeyError("state needs X/x, Y/y and psi/yaw")
    out = {"x": _finite_or_none(x), "y": _finite_or_none(y),
           "yaw": _finite_or_none(yaw)}
    for wire, keys in (("t", ("t",)), ("vx", ("vx", "v")),
                       ("delta", ("delta_rw", "delta")), ("roll", ("roll",)),
                       ("pitch", ("pitch",)), ("z", ("Z", "z"))):
        v = pick(*keys)
        if v is not None:
            out[wire] = _finite_or_none(v)
    return out


def vehicle_geometry(vp) -> dict:
    """Drawing geometry from a ``vdsim.VehicleParams``.

    :param vp: vehicle parameters (e.g. ``VDSimPlant(...).vehicle.params``).
    :returns: dict for ``LiveMonitor(vehicle=...)``: axle positions, mean
        track and nominal wheel radius [m]; body dimensions keep the defaults.
    """
    return {"cg_to_front": float(vp.cg_to_front), "cg_to_rear": float(vp.cg_to_rear),
            "track": 0.5 * (float(vp.track_front) + float(vp.track_rear)),
            "wheel_radius": float(vp.wheel_radius_nominal)}


# --------------------------------------------------------------------------
# WebSocket (RFC 6455, server side, text frames)
# --------------------------------------------------------------------------
def _ws_accept(key: str) -> str:
    """``Sec-WebSocket-Accept`` value for a client key."""
    return base64.b64encode(hashlib.sha1((key + _WS_GUID).encode()).digest()).decode()


def _ws_frame(opcode: int, payload: bytes) -> bytes:
    """One unmasked, final server frame."""
    n = len(payload)
    if n < 126:
        head = struct.pack("!BB", 0x80 | opcode, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x80 | opcode, 126, n)
    else:
        head = struct.pack("!BBQ", 0x80 | opcode, 127, n)
    return head + payload


def _recv_exact(sock, n: int) -> bytes:
    """Read exactly ``n`` bytes; ``b""`` on EOF."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf


def _ws_read_frame(sock):
    """Read one client frame.

    :returns: ``(opcode, payload)``, or ``None`` on EOF.
    """
    h = _recv_exact(sock, 2)
    if not h:
        return None
    b0, b1 = h
    n = b1 & 0x7F
    if n == 126:
        ext = _recv_exact(sock, 2)
        if not ext:
            return None
        n = struct.unpack("!H", ext)[0]
    elif n == 127:
        ext = _recv_exact(sock, 8)
        if not ext:
            return None
        n = struct.unpack("!Q", ext)[0]
    mask = _recv_exact(sock, 4) if b1 & 0x80 else b""
    if b1 & 0x80 and not mask:
        return None
    data = _recv_exact(sock, n) if n else b""
    if n and not data:
        return None
    if mask:
        data = bytes(c ^ mask[i % 4] for i, c in enumerate(data))
    return b0 & 0x0F, data


class _WsClient:
    """One connected page; sends are serialized by a lock."""

    def __init__(self, sock):
        """:param sock: the upgraded connection socket."""
        self.sock = sock
        self.lock = threading.Lock()
        self.alive = True

    def send_text(self, text: str) -> bool:
        """Send one text frame.

        :returns: False when the connection is gone (the client is then dead).
        """
        if not self.alive:
            return False
        try:
            with self.lock:
                self.sock.sendall(_ws_frame(0x1, text.encode()))
            return True
        except OSError:
            self.alive = False
            return False

    def send_raw(self, opcode: int, payload: bytes) -> None:
        """Send a control frame (pong/close), ignoring a closed socket."""
        try:
            with self.lock:
                self.sock.sendall(_ws_frame(opcode, payload))
        except OSError:
            self.alive = False


# --------------------------------------------------------------------------
# server
# --------------------------------------------------------------------------
class MonitorServer:
    """Page server: HTTP + WebSocket on TCP ``port``, UDP ingest on the same
    port number at ``ingest_host``.

    :param port: TCP/UDP port.
    :param host: TCP bind address of the page (``"0.0.0.0"`` = every
        interface, so other PCs can open it).
    :param ingest_host: UDP bind address for publishers (default loopback:
        only processes on this host can publish).
    :param rate_hz: maximum state frames per second to each page.
    :param scene: initial scene, same keys as a publisher's scene datagram
        (``osm``, ``origin``, ``vehicle``).
    :param log: callable for one-line messages (default ``print`` to stderr).
    """

    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0",
                 ingest_host: str = "127.0.0.1", rate_hz: float = 30.0,
                 scene: dict | None = None, log=None):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        self.port, self.host, self.ingest_host = int(port), host, ingest_host
        self.rate_hz = float(rate_hz)
        self._log = log or (lambda m: print(f"[livemonitor] {m}", file=sys.stderr, flush=True))
        self._lock = threading.Lock()
        self._clients: set = set()
        self._state = None
        self._state_seq = 0
        self._state_wall = 0.0
        self._scene_spec = None
        self._scene = {"version": 0, "vehicle": dict(DEFAULT_VEHICLE),
                       "roads": [], "origin": None, "osm": None, "osm_error": None}
        self._stop = threading.Event()
        self._threads = []
        self._httpd = None
        self._udp = None
        if scene:
            self._apply_scene(scene)

    # ---- lifecycle -------------------------------------------------------
    def start(self) -> "MonitorServer":
        """Bind both sockets and start the serving threads.

        :raises OSError: a port is taken (``errno`` EADDRINUSE).
        """
        srv = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *args):  # quiet
                pass

            def do_GET(self):
                srv._handle_get(self)

        httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        httpd.daemon_threads = True
        try:
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp.bind((self.ingest_host, self.port))
            udp.settimeout(0.2)
        except OSError:
            httpd.server_close()
            raise
        self._httpd, self._udp = httpd, udp
        for target in (httpd.serve_forever, self._ingest_loop, self._broadcast_loop):
            th = threading.Thread(target=target, daemon=True,
                                  name=f"livemonitor-{target.__name__}")
            th.start()
            self._threads.append(th)
        self._log(f"serving http://{self.host}:{self.port}/ (ingest udp {self.ingest_host}:{self.port})")
        return self

    def stop(self) -> None:
        """Close sockets, disconnect pages and join the threads."""
        self._stop.set()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for c in clients:
            c.send_raw(0x8, struct.pack("!H", 1001))
            try:
                c.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        for th in self._threads:
            th.join(timeout=2.0)
        if self._udp is not None:
            self._udp.close()
        self._threads.clear()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # ---- state / scene ---------------------------------------------------
    def status(self) -> dict:
        """Snapshot for ``/health`` and the page status line."""
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> dict:
        """:meth:`status` for callers already holding the lock."""
        age = time.monotonic() - self._state_wall if self._state is not None else None
        return {"service": SERVICE_ID, "v": PROTOCOL_VERSION,
                "live": age is not None and age <= LIVE_TIMEOUT_S,
                "age_s": None if age is None else round(age, 3),
                "scene_version": self._scene["version"],
                "clients": len(self._clients)}

    def _apply_scene(self, spec: dict) -> None:
        """Install a scene spec (``osm``, ``origin``, ``vehicle``); a spec
        equal to the current one is ignored. OSM errors are kept in the scene
        (``osm_error``) and shown on the page instead of stopping the server.
        """
        key = json.dumps(spec, sort_keys=True)
        if key == self._scene_spec:
            return
        vehicle = dict(DEFAULT_VEHICLE)
        vehicle.update({k: float(v) for k, v in (spec.get("vehicle") or {}).items()
                        if k in DEFAULT_VEHICLE and _finite_or_none(v) is not None})
        roads, err = [], None
        osm, origin = spec.get("osm"), spec.get("origin")
        if osm:
            try:
                roads = load_osm(osm, origin)
            except (ValueError, OSError) as e:
                err = str(e)
                self._log(err)
        with self._lock:
            self._scene_spec = key
            self._scene = {"version": self._scene["version"] + 1, "vehicle": vehicle,
                           "roads": roads, "origin": origin, "osm": osm, "osm_error": err}
        self._log(f"scene v{self._scene['version']}: {len(roads)} ways"
                  + (f" from {osm}" if osm else ""))

    def _ingest_loop(self):
        """Receive publisher datagrams until stopped."""
        while not self._stop.is_set():
            try:
                data, _ = self._udp.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(msg, dict):
                continue
            if isinstance(msg.get("scene"), dict):
                self._apply_scene(msg["scene"])
            if isinstance(msg.get("state"), dict):
                with self._lock:
                    self._state = msg["state"]
                    self._state_seq += 1
                    self._state_wall = time.monotonic()

    def _broadcast_loop(self):
        """Send the newest state at ``rate_hz`` and a status line every second."""
        period = 1.0 / self.rate_hz
        sent_seq, last_status = 0, 0.0
        while not self._stop.wait(period):
            now = time.monotonic()
            msgs = []
            with self._lock:
                if self._state_seq != sent_seq and self._state is not None:
                    msgs.append(json.dumps({"type": "state", "state": self._state}))
                    sent_seq = self._state_seq
            if now - last_status >= 1.0:
                msgs.append(json.dumps({"type": "status", **self.status()}))
                last_status = now
            if msgs:
                self._send_all(msgs)

    def _send_all(self, msgs):
        """Send messages to every page, dropping dead ones."""
        with self._lock:
            clients = list(self._clients)
        dead = [c for c in clients if not all(c.send_text(m) for m in msgs)]
        if dead:
            with self._lock:
                self._clients.difference_update(dead)

    # ---- HTTP ------------------------------------------------------------
    def _handle_get(self, h: BaseHTTPRequestHandler) -> None:
        """Route one GET request."""
        path = h.path.split("?", 1)[0]
        if path == "/ws":
            return self._serve_ws(h)
        if path == "/health":
            return self._send(h, 200, "application/json", json.dumps(self.status()).encode())
        if path == "/scene.json":
            with self._lock:
                body = json.dumps(self._scene).encode()
            return self._send(h, 200, "application/json", body)
        files = {"/": (_web_dir() / "index.html", "text/html; charset=utf-8"),
                 "/index.html": (_web_dir() / "index.html", "text/html; charset=utf-8"),
                 "/monitor.js": (_web_dir() / "monitor.js", "text/javascript; charset=utf-8")}
        if path in files:
            f, ctype = files[path]
        elif path.startswith("/three/"):
            rel = path[len("/three/"):]
            if rel not in ("three.module.js", "addons/controls/OrbitControls.js"):
                return self._send(h, 404, "text/plain", b"not found")
            f, ctype = _three_dir() / rel, "text/javascript; charset=utf-8"
        else:
            return self._send(h, 404, "text/plain", b"not found")
        try:
            body = f.read_bytes()
        except OSError:
            return self._send(h, 404, "text/plain", b"not found")
        self._send(h, 200, ctype, body)

    @staticmethod
    def _send(h, code: int, ctype: str, body: bytes) -> None:
        """Write a complete HTTP response (no caching: the page must pick up
        a new server immediately)."""
        h.send_response(code)
        h.send_header("Content-Type", ctype)
        h.send_header("Content-Length", str(len(body)))
        h.send_header("Cache-Control", "no-store")
        h.end_headers()
        h.wfile.write(body)

    def _serve_ws(self, h: BaseHTTPRequestHandler) -> None:
        """Upgrade to WebSocket, register the page, read until it closes."""
        key = h.headers.get("Sec-WebSocket-Key")
        if not key or "websocket" not in (h.headers.get("Upgrade") or "").lower():
            return self._send(h, 400, "text/plain", b"websocket upgrade required")
        h.send_response(101, "Switching Protocols")
        h.send_header("Upgrade", "websocket")
        h.send_header("Connection", "Upgrade")
        h.send_header("Sec-WebSocket-Accept", _ws_accept(key))
        h.end_headers()
        h.wfile.flush()
        h.close_connection = True
        sock = h.connection
        sock.settimeout(None)
        client = _WsClient(sock)
        with self._lock:
            self._clients.add(client)
            hello = {"type": "hello", **self._status_locked()}
            state = self._state
        client.send_text(json.dumps(hello))
        if state is not None:
            client.send_text(json.dumps({"type": "state", "state": state}))
        try:
            while client.alive and not self._stop.is_set():
                fr = _ws_read_frame(sock)
                if fr is None:
                    break
                op, data = fr
                if op == 0x8:
                    client.send_raw(0x8, data[:2])
                    break
                if op == 0x9:
                    client.send_raw(0xA, data)
        except OSError:
            pass
        finally:
            client.alive = False
            with self._lock:
                self._clients.discard(client)


def _probe(port: int, timeout: float = 0.5):
    """``/health`` of whatever listens on ``127.0.0.1:port``.

    :returns: the decoded JSON, or ``None`` when it is not a live monitor.
    """
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=timeout) as r:
            d = json.loads(r.read().decode())
        return d if isinstance(d, dict) and d.get("service") == SERVICE_ID else None
    except (OSError, ValueError):
        return None


# --------------------------------------------------------------------------
# publisher
# --------------------------------------------------------------------------
class LiveMonitor:
    """Publish a simulation's state to a :class:`MonitorServer`.

    :param port: monitor port (TCP page / UDP ingest).
    :param host: page bind address when the server is embedded.
    :param osm: optional ``.osm`` file drawn on the ground.
    :param origin: ``(lat, lon[, h])`` of the simulation frame origin;
        required with ``osm``.
    :param vehicle: drawing geometry overrides, keys of
        :data:`DEFAULT_VEHICLE` [m].
    :param rate_hz: maximum pushes forwarded per second; extra calls to
        :meth:`push` return False without sending.
    :param open_browser: open the page in this host's browser after start
        (ignored without a local display).
    :param embed: start an embedded server when none is listening; with
        False only a standing server is used and :meth:`start` fails without
        one.
    :raises ValueError: ``osm`` without ``origin`` (the reason is in the
        message), or an unreadable OSM file.
    """

    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0",
                 osm=None, origin=None, vehicle: dict | None = None,
                 rate_hz: float = 30.0, open_browser: bool = False,
                 embed: bool = True):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        if osm is not None:
            load_osm(osm, origin)  # refuse early, before any socket opens
        self.port, self.host = int(port), host
        self.rate_hz = float(rate_hz)
        self.open_browser, self.embed = bool(open_browser), bool(embed)
        self.scene = {"osm": None if osm is None else str(Path(osm).resolve()),
                      "origin": None if origin is None else [float(v) for v in origin],
                      "vehicle": dict(vehicle or {})}
        self.server = None          # MonitorServer when embedded
        self.mode = None            # "embedded" | "standing"
        self._sock = None
        self._last_push = -math.inf
        self._last_scene = -math.inf
        self.n_sent = 0

    @property
    def url(self) -> str:
        """Page address on this host."""
        return f"http://127.0.0.1:{self.port}/"

    def start(self) -> "LiveMonitor":
        """Attach to a standing server on ``port`` or start an embedded one.

        :raises RuntimeError: the port is held by something that is not a
            live monitor, or ``embed=False`` and no server is running.
        """
        if _probe(self.port) is not None:
            self.mode = "standing"
        elif not self.embed:
            raise RuntimeError(f"no live monitor server on port {self.port} (embed=False)")
        else:
            try:
                self.server = MonitorServer(self.port, self.host, rate_hz=self.rate_hz,
                                            scene=self.scene).start()
                self.mode = "embedded"
            except OSError as e:
                raise RuntimeError(f"port {self.port} is in use by something that is "
                                   f"not a VDSim live monitor ({e})") from e
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send({"scene": self.scene})
        self._last_scene = time.monotonic()
        if self.open_browser and (os.name == "nt" or os.environ.get("DISPLAY")
                                  or sys.platform == "darwin"):
            webbrowser.open(self.url)
        return self

    def push(self, state: dict) -> bool:
        """Forward one state, at most ``rate_hz`` times per second.

        :param state: dict with ``X``/``x``, ``Y``/``y``, ``psi``/``yaw`` [m,
            rad] and optionally ``t``, ``vx``, ``delta_rw``/``delta``,
            ``roll``, ``pitch`` -- a :class:`vdsim_plant.VDSimPlant`
            observation works as is.
        :returns: True when a datagram was sent.
        :raises RuntimeError: called before :meth:`start`.
        :raises KeyError: position or heading missing.
        """
        if self._sock is None:
            raise RuntimeError("LiveMonitor.push() before start()")
        now = time.monotonic()
        if now - self._last_push < 1.0 / self.rate_hz:
            return False
        msg = {"state": _compact_state(state)}
        if now - self._last_scene >= SCENE_RESEND_S:
            msg["scene"] = self.scene
            self._last_scene = now
        self._last_push = now
        return self._send(msg)

    def _send(self, msg: dict) -> bool:
        """One datagram to the ingest port; a missing server is not an error."""
        try:
            self._sock.sendto(json.dumps(msg, allow_nan=False).encode(),
                              ("127.0.0.1", self.port))
        except OSError:
            return False
        self.n_sent += 1
        return True

    def stop(self) -> None:
        """Close the socket and stop an embedded server (a standing one keeps
        running and shows "waiting for simulator")."""
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        if self.server is not None:
            self.server.stop()
            self.server = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


# --------------------------------------------------------------------------
# Windows shortcut + CLI
# --------------------------------------------------------------------------
def write_shortcut(path, url: str) -> Path:
    """Write a Windows Internet Shortcut (``.url``) that opens ``url``.

    :param path: output file; ``.url`` is appended when missing.
    :param url: ``http://host:port/`` of a standing monitor server.
    :returns: the written path.
    :raises ValueError: ``url`` is not http(s).
    """
    if not url.startswith(("http://", "https://")):
        raise ValueError(f"shortcut url must be http(s): {url}")
    path = Path(path)
    if path.suffix.lower() != ".url":
        path = path.with_name(path.name + ".url")
    path.write_bytes(f"[InternetShortcut]\r\nURL={url}\r\n".encode("ascii"))
    return path


def _parse_origin(text):
    """``"lat,lon[,h]"`` -> tuple, or None."""
    if not text:
        return None
    return tuple(float(v) for v in text.split(","))


def main(argv=None) -> int:
    """``python -m vdsim_livemonitor serve|shortcut``."""
    ap = argparse.ArgumentParser(prog="vdsim_livemonitor", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve", help="standing page server (waits for simulators)")
    s.add_argument("--port", type=int, default=DEFAULT_PORT)
    s.add_argument("--host", default="0.0.0.0")
    s.add_argument("--rate", type=float, default=30.0)
    s.add_argument("--osm", default=None)
    s.add_argument("--origin", default=None, help="lat,lon[,h] of the simulation origin")
    s.add_argument("--open", action="store_true", help="open the page in a local browser")
    k = sub.add_parser("shortcut", help="write a Windows .url shortcut")
    k.add_argument("--url", required=True, help="http://host:port/")
    k.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "shortcut":
        print(write_shortcut(a.out, a.url))
        return 0
    scene = None
    if a.osm:
        try:
            load_osm(a.osm, _parse_origin(a.origin))
        except (ValueError, OSError) as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
        scene = {"osm": str(Path(a.osm).resolve()),
                 "origin": list(_parse_origin(a.origin)), "vehicle": {}}
    srv = MonitorServer(a.port, a.host, rate_hz=a.rate, scene=scene).start()
    if a.open:
        webbrowser.open(f"http://127.0.0.1:{a.port}/")
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
