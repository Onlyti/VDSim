#!/usr/bin/env python3
"""Live 3D monitor (work order 35, second part): checks without a browser.

* L1 ``geodetic_to_enu``: the origin maps to zero; small north/east offsets
  match the meridian and prime-vertical arc lengths.
* L2 ``load_osm``: the synthetic fixture's highway and Lanelet2 ways land on
  the x/y each node was generated from; ``origin=None`` is refused with the
  reason, also by ``LiveMonitor`` before any socket opens.
* L3 page server: ``/``, ``/monitor.js``, the vendored three.js and
  ``/health`` answer; a WebSocket client gets ``hello`` with ``live`` False
  while no simulator publishes ("waiting").
* L4 a ``LiveMonitor`` attaches to the standing server, its states reach the
  page client (non-finite numbers as ``null``) and ``live`` turns True, also
  for a single push sent right after a status line; ``push`` is limited to
  ``rate_hz``.
* L5 after a server restart on the same port a new page connection gets the
  stream again (what the page's reconnect loop relies on).
* L6 no server -> ``LiveMonitor`` embeds one and frees the port on stop; a
  port held by something else is an error; ``embed=False`` needs a server.
* L7 the publisher's OSM scene reaches a standing server (``/scene.json``).
* L8 ``write_shortcut`` writes a Windows Internet Shortcut.
"""
import base64
import json
import math
import os
import socket
import struct
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import vdsim_livemonitor as lm  # noqa: E402

FIXTURE = REPO / "tests" / "fixtures" / "osm" / "livemonitor_sample.osm"
ORIGIN = (37.5, 127.0)
FAILS = []


def check(name, ok, detail=""):
    """Record one assertion."""
    print(f"{'PASS' if ok else 'FAIL'} {name} {detail}", flush=True)
    if not ok:
        FAILS.append(name)


def free_port():
    """A port free for both TCP and UDP on this host."""
    while True:
        t = socket.socket()
        t.bind(("0.0.0.0", 0))
        port = t.getsockname()[1]
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            u.bind(("127.0.0.1", port))
            return port
        except OSError:
            continue
        finally:
            t.close()
            u.close()


def http(port, path):
    """GET -> (status, body bytes)."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=3) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, b""


class WsClient:
    """Minimal WebSocket client for text frames."""

    def __init__(self, port):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall((f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
                        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            c = self.s.recv(1)
            if not c:
                raise ConnectionError("closed during handshake")
            head += c
        self.status = head.split(b"\r\n", 1)[0]
        self.accept_ok = lm._ws_accept(key).encode() in head

    def recv(self, timeout=3.0):
        """Next text message as dict, or None on timeout/close."""
        self.s.settimeout(timeout)
        try:
            fr = lm._ws_read_frame(self.s)
        except (socket.timeout, OSError):
            return None
        if fr is None or fr[0] != 0x1:
            return None
        return json.loads(fr[1].decode())

    def wait_for(self, pred, timeout=3.0):
        """First message satisfying ``pred`` within ``timeout`` [s]."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.recv(max(0.05, end - time.monotonic()))
            if m is not None and pred(m):
                return m
        return None

    def closed_within(self, timeout):
        """True when the server closes the connection within ``timeout`` [s]
        (close frame or EOF); messages before that are skipped."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.s.settimeout(max(0.05, end - time.monotonic()))
            try:
                fr = lm._ws_read_frame(self.s)
            except socket.timeout:
                return False
            except OSError:
                return True
            if fr is None or fr[0] == 0x8:
                return True
        return False

    def close(self):
        """Send a masked close frame and drop the socket."""
        try:
            self.s.sendall(struct.pack("!BB", 0x88, 0x80) + b"\0\0\0\0")
        except OSError:
            pass
        self.s.close()


def l1_geodetic():
    e, n, u = lm.geodetic_to_enu(ORIGIN[0], ORIGIN[1], 0.0, ORIGIN)
    check("L1 origin -> 0", max(abs(e), abs(n), abs(u)) < 1e-9, f"({e:.2e}, {n:.2e}, {u:.2e})")
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    p = math.radians(ORIGIN[0])
    m = a * (1 - e2) / (1 - e2 * math.sin(p) ** 2) ** 1.5      # meridian radius
    nv = a / math.sqrt(1 - e2 * math.sin(p) ** 2)              # prime vertical
    d = 1e-4
    _, n1, _ = lm.geodetic_to_enu(ORIGIN[0] + d, ORIGIN[1], 0.0, ORIGIN)
    e1, _, _ = lm.geodetic_to_enu(ORIGIN[0], ORIGIN[1] + d, 0.0, ORIGIN)
    check("L1 north arc", abs(n1 - m * math.radians(d)) < 1e-3, f"{n1:.4f} m")
    check("L1 east arc", abs(e1 - nv * math.cos(p) * math.radians(d)) < 1e-3, f"{e1:.4f} m")
    try:
        lm.geodetic_to_enu(0, 0, 0, (91.0, 0.0))
        check("L1 bad origin refused", False)
    except ValueError:
        check("L1 bad origin refused", True)


def l2_osm():
    ways = lm.load_osm(FIXTURE, ORIGIN)
    kinds = sorted((w["kind"], w["cls"]) for w in ways)
    check("L2 drawable ways", kinds == [("highway", "primary"), ("highway", "residential"),
                                        ("lane", "line_thick"), ("lane", "line_thin"),
                                        ("lane", "virtual")], str(kinds))
    import re
    want = {}
    for mt in re.finditer(r"lat='([-0-9.]+)' lon='([-0-9.]+)' x='([-0-9.]+)' y='([-0-9.]+)'",
                          FIXTURE.read_text()):
        want[(round(float(mt[1]), 10), round(float(mt[2]), 10))] = (float(mt[3]), float(mt[4]))
    worst = 0.0
    xs = sorted(set(x for x, _ in want.values()))
    for w in ways:
        for x, y in w["pts"]:
            worst = max(worst, min(math.hypot(x - xx, y - yy) for xx, yy in want.values()))
    check("L2 points on generated x/y", worst < 1e-3, f"worst {worst:.2e} m over {len(xs)} x values")
    prim = next(w for w in ways if w["cls"] == "primary")
    check("L2 highway width", prim["width"] == lm.HIGHWAY_WIDTH_M["primary"])
    for label, fn in (("load_osm", lambda: lm.load_osm(FIXTURE, None)),
                      ("LiveMonitor", lambda: lm.LiveMonitor(port=free_port(), osm=FIXTURE))):
        try:
            fn()
            check(f"L2 {label} refuses OSM without origin", False)
        except ValueError as e:
            check(f"L2 {label} refuses OSM without origin", "origin" in str(e), str(e)[:60])


def l3_l4_l5_server():
    port = free_port()
    srv = lm.MonitorServer(port, "127.0.0.1", log=lambda m: None).start()
    try:
        st, body = http(port, "/")
        check("L3 page", st == 200 and b"monitor.js" in body and b"importmap" in body)
        st, body = http(port, "/monitor.js")
        check("L3 script", st == 200 and b"WebSocket" in body)
        st, body = http(port, "/three/three.module.js")
        check("L3 vendored three.js", st == 200 and len(body) > 100000, f"{len(body)} B")
        st, _ = http(port, "/three/addons/controls/OrbitControls.js")
        check("L3 orbit controls", st == 200)
        check("L3 no path escape", http(port, "/three/../../python/vdsim_plant.py")[0] == 404)
        st, body = http(port, "/health")
        h = json.loads(body)
        check("L3 health", st == 200 and h["service"] == lm.SERVICE_ID and h["live"] is False)
        ws = WsClient(port)
        check("L3 websocket upgrade", b" 101 " in ws.status and ws.accept_ok, ws.status.decode())
        hello = ws.recv()
        check("L3 hello while waiting", hello is not None and hello["type"] == "hello"
              and hello["live"] is False, str(hello))

        mon = lm.LiveMonitor(port=port, rate_hz=50.0).start()
        check("L4 attaches to standing server", mon.mode == "standing" and mon.server is None)
        # push right after a periodic status line: the next periodic line is
        # about a second later, so only the waiting -> live edge line can
        # report live for this single push
        ws.wait_for(lambda m: m["type"] == "status", 1.5)
        sent = mon.push({"t": 1.0, "X": 3.0, "Y": -2.0, "psi": 0.25, "vx": 12.0,
                         "delta_rw": float("nan"), "roll": 0.01})
        again = mon.push({"X": 4.0, "Y": -2.0, "psi": 0.25})
        check("L4 push rate limit", sent is True and again is False)
        m = ws.wait_for(lambda m: m["type"] == "state")
        s = (m or {}).get("state", {})
        check("L4 state reaches page", s.get("x") == 3.0 and s.get("y") == -2.0
              and s.get("yaw") == 0.25 and s.get("vx") == 12.0 and s.get("t") == 1.0, str(s))
        check("L4 non-finite -> null", "delta" in s and s["delta"] is None)
        live = ws.wait_for(lambda m: m["type"] == "status" and m["live"] is True, 2.5)
        check("L4 status live", live is not None)
        try:
            mon.push({"X": 1.0})
            check("L4 missing heading refused", False)
        except KeyError:
            check("L4 missing heading refused", True)
        mon.stop()
        gone = ws.wait_for(lambda m: m["type"] == "status" and m["live"] is False, 3.5)
        check("L4 back to waiting after publisher stops", gone is not None)
    finally:
        srv.stop()
    check("L5 page sees server stop", ws.closed_within(2.0))
    ws.close()
    srv2 = lm.MonitorServer(port, "127.0.0.1", log=lambda m: None).start()
    try:
        ws2 = WsClient(port)
        hello = ws2.recv()
        mon = lm.LiveMonitor(port=port).start()
        mon.push({"X": 7.0, "Y": 1.0, "psi": 0.0})
        m = ws2.wait_for(lambda m: m["type"] == "state")
        check("L5 restart on same port streams again",
              hello is not None and m is not None and m["state"]["x"] == 7.0)
        mon.stop()
        ws2.close()
    finally:
        srv2.stop()


def l6_embed():
    port = free_port()
    mon = lm.LiveMonitor(port=port, host="127.0.0.1").start()
    try:
        check("L6 embeds when no server", mon.mode == "embedded" and mon.server is not None)
        check("L6 embedded health", http(port, "/health")[0] == 200)
    finally:
        mon.stop()
    t = socket.socket()
    t.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # as the server binds
    try:
        t.bind(("0.0.0.0", port))
        check("L6 port freed on stop", True)
    except OSError as e:
        check("L6 port freed on stop", False, str(e))
    finally:
        t.close()
    port = free_port()
    blocker = socket.socket()
    blocker.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    blocker.bind(("0.0.0.0", port))
    blocker.listen(1)
    try:
        lm.LiveMonitor(port=port).start()
        check("L6 foreign port refused", False)
    except RuntimeError as e:
        check("L6 foreign port refused", "not a VDSim live monitor" in str(e))
    finally:
        blocker.close()
    try:
        lm.LiveMonitor(port=free_port(), embed=False).start()
        check("L6 embed=False needs a server", False)
    except RuntimeError:
        check("L6 embed=False needs a server", True)


def l7_scene():
    port = free_port()
    srv = lm.MonitorServer(port, "127.0.0.1", log=lambda m: None).start()
    try:
        mon = lm.LiveMonitor(port=port, osm=FIXTURE, origin=ORIGIN,
                             vehicle={"track": 1.7, "bogus": 3}).start()
        sc = {}
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            sc = json.loads(http(port, "/scene.json")[1])
            if sc.get("roads"):
                break
            time.sleep(0.05)
        check("L7 OSM scene on standing server", len(sc.get("roads", [])) == 5
              and sc.get("osm_error") is None, f"{len(sc.get('roads', []))} ways")
        check("L7 vehicle override", sc.get("vehicle", {}).get("track") == 1.7
              and "bogus" not in sc.get("vehicle", {}))
        mon.stop()
    finally:
        srv.stop()


def l8_shortcut():
    with tempfile.TemporaryDirectory() as d:
        p = lm.write_shortcut(Path(d) / "VDSim monitor", "http://10.1.2.3:8790/")
        b = p.read_bytes()
        check("L8 shortcut file", p.suffix == ".url"
              and b == b"[InternetShortcut]\r\nURL=http://10.1.2.3:8790/\r\n", repr(b))
        try:
            lm.write_shortcut(Path(d) / "x.url", "file:///etc/passwd")
            check("L8 non-http refused", False)
        except ValueError:
            check("L8 non-http refused", True)


if __name__ == "__main__":
    for fn in (l1_geodetic, l2_osm, l3_l4_l5_server, l6_embed, l7_scene, l8_shortcut):
        fn()
    print("ALL CHECKS PASSED" if not FAILS else f"FAILED: {FAILS}")
    sys.exit(1 if FAILS else 0)
