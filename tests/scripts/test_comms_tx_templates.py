#!/usr/bin/env python3
"""End-to-end: the `json` and `nmea_gga` comms TX templates reach the wire.

The unit tests (tests/unit/test_comms_templates.cpp) cover the encoders; this
test proves the realtime server's TX gate actually accepts the two text
templates, routes them per channel, and that what lands in a UDP socket is
parseable JSON / a checksum-valid NMEA 0183 GGA sentence whose position tracks
the vehicle.

The comms block is not written by hand here: it is the shipped sample
configs/comms/json_nmea.yaml with only its port numbers remapped onto free
ones, so the file that ships to users cannot silently rot.
"""
import json
import math
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
BIN = REPO / "build" / "bin" / "vdsim_realtime"
SCENE = REPO / "configs" / "scenes" / "two_vehicle_race.yaml"
COMMS_SPEC = REPO / "configs" / "comms" / "json_nmea.yaml"

sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

# WGS84, for the independent forward transform used to check the GGA fix.
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

# GGA prints minutes to 4 decimals: 1e-4 min ~= 0.185 m on the ground. Every
# position comparison below has to live above that quantisation floor.
GGA_QUANTUM_M = 0.185

# Vehicle 0 spawns here in configs/scenes/two_vehicle_race.yaml and drives +x
# (= ENU east) at 12 m/s. With `control: external` and no commands sent, the
# cmd-timeout failsafe brakes it, so it still covers > 10 m in the capture.
SPAWN_X = -15.0
SPAWN_Y = -1.5

CAPTURE_S = 1.5          # wall-clock capture window
MIN_DATAGRAMS = 50       # at 200 Hz this is a very loose floor
MIN_TRAVEL_M = 5.0       # the fix must move at least this far during the capture


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _nmea_checksum(body):
    """XOR of every character between '$' and '*'."""
    sum_ = 0
    for ch in body:
        sum_ ^= ord(ch)
    return sum_


def _geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    """WGS84 geodetic -> ECEF. Forward direction, so it is independent of the
    closed-form inverse in cosim/comms_templates.hpp that we are checking."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return ((n + alt_m) * math.cos(lat) * math.cos(lon),
            (n + alt_m) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - WGS84_E2) + alt_m) * math.sin(lat))


def _fix_to_enu(lat_deg, lon_deg, alt_m, origin):
    """Turn a reported GGA fix back into ENU metres from the channel datum."""
    lat0, lon0, alt0 = origin
    x0, y0, z0 = _geodetic_to_ecef(lat0, lon0, alt0)
    x, y, z = _geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = math.radians(lat0), math.radians(lon0)
    sla, cla, slo, clo = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    east = -slo * dx + clo * dy
    north = -sla * clo * dx - sla * slo * dy + cla * dz
    up = cla * clo * dx + cla * slo * dy + sla * dz
    return east, north, up


def _check_gga(raw, origin):
    """Validate one datagram as a $GPGGA sentence.

    Returns (error_or_None, east_m, north_m) where the offsets are the reported
    fix expressed back in the channel's ENU frame.
    """
    text = raw.decode("ascii", "replace")
    if not text.startswith("$") or not text.endswith("\r\n"):
        return f"framing: {text!r}", None, None
    body, _, csum = text[1:-2].partition("*")
    if len(csum) != 2 or csum != csum.upper():
        return f"checksum digits: {text!r}", None, None
    if int(csum, 16) != _nmea_checksum(body):
        return f"checksum mismatch: {text!r}", None, None
    f = body.split(",")
    if len(f) != 15:
        return f"field count {len(f)}: {text!r}", None, None
    # No field may carry whitespace or a non-numeric float token: that is what a
    # NaN/out-of-range state used to smuggle through with a valid checksum.
    for i, field in enumerate(f):
        if any(c.isspace() for c in field):
            return f"whitespace in field {i}: {text!r}", None, None
        if "nan" in field.lower() or "inf" in field.lower():
            return f"non-finite token in field {i}: {text!r}", None, None
    if f[0] != "GPGGA":
        return f"sentence id: {text!r}", None, None
    if len(f[1]) != 9 or f[1][6] != ".":
        return f"utc field: {text!r}", None, None
    if int(f[1][0:2]) > 23 or int(f[1][2:4]) > 59 or float(f[1][4:]) >= 60.0:
        return f"illegal utc time: {text!r}", None, None
    if f[6] != "1":
        return f"fix quality {f[6]!r} (expected a valid fix): {text!r}", None, None
    if len(f[2]) != 9 or f[2][4] != "." or f[3] not in ("N", "S"):
        return f"latitude field: {text!r}", None, None
    if len(f[4]) != 10 or f[4][5] != "." or f[5] not in ("E", "W"):
        return f"longitude field: {text!r}", None, None
    if f[10] != "M" or f[12] != "M":
        return f"altitude units: {text!r}", None, None
    lat = int(f[2][:2]) + float(f[2][2:]) / 60.0
    lon = int(f[4][:3]) + float(f[4][3:]) / 60.0
    if not (0.0 <= lat <= 90.0):
        return f"latitude out of range: {text!r}", None, None
    if not (0.0 <= lon <= 180.0):
        return f"longitude out of range: {text!r}", None, None
    if f[3] == "S":
        lat = -lat
    if f[5] == "W":
        lon = -lon
    east, north, _ = _fix_to_enu(lat, lon, float(f[9]), origin)
    return None, east, north


def _load_shipped_comms(json_port, gga_port, rx_port):
    """Load configs/comms/json_nmea.yaml, sanity-check it, and remap its ports.

    The shipped sample is what users copy, so the e2e test runs *it* rather than
    an inline lookalike. Only the ports are rewritten (the file targets fixed
    ports that a test must not assume are free); templates, sources and the
    geodetic datum are used exactly as shipped.
    """
    if not COMMS_SPEC.is_file():
        raise AssertionError(f"missing shipped comms sample {COMMS_SPEC}")
    doc = yaml.safe_load(COMMS_SPEC.read_text()) or {}
    if doc.get("name") != "json_nmea":
        raise AssertionError(f"unexpected comms name {doc.get('name')!r}")
    chans = doc.get("channels") or []
    tx = {c.get("template"): c for c in chans if c.get("direction") != "in"}
    rx = [c for c in chans if c.get("direction") == "in"]
    if set(tx) != {"json", "nmea_gga"}:
        raise AssertionError(f"shipped TX templates changed: {sorted(tx)}")
    if len(rx) != 1 or rx[0].get("template") != "vds1_cmd":
        raise AssertionError(f"shipped RX channel changed: {rx}")
    for name, ch in tx.items():
        if ch.get("source") != "0.state":
            raise AssertionError(f"{name}: unexpected source {ch.get('source')!r}")
        if len(ch.get("to") or []) != 1:
            raise AssertionError(f"{name}: expected exactly one destination")
    og = tx["nmea_gga"].get("origin") or {}
    if not {"lat", "lon", "alt"} <= set(og):
        raise AssertionError(f"nmea_gga origin incomplete: {og}")
    tx["json"]["to"][0]["port"] = json_port
    tx["nmea_gga"]["to"][0]["port"] = gga_port
    rx[0]["listen"]["port"] = rx_port
    return doc, (float(og["lat"]), float(og["lon"]), float(og["alt"]))


def _write_world(tmp, json_port, gga_port, bad_port, rx_port):
    """Materialize a catalog scene, then attach the shipped comms spec to it."""
    from catalog import materialize_scene_file  # noqa: E402

    comms, origin = _load_shipped_comms(json_port, gga_port, rx_port)
    world_path = tmp / "world.yaml"
    materialize_scene_file(SCENE, world_path)
    doc = yaml.safe_load(world_path.read_text()) or {}
    doc["sim"] = dict(doc.get("sim") or {})
    doc["sim"]["rate"] = 200.0
    # An unknown template must still be warned about and skipped.
    comms["channels"].append({"source": "0.state", "template": "not_a_template",
                              "to": [{"ip": "127.0.0.1", "port": bad_port}]})
    doc["comms"] = comms
    world_path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return world_path, origin


def _capture(world, json_port, gga_port, bad_port):
    """Run vdsim_realtime for CAPTURE_S and drain the three UDP sinks."""
    rx_json = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_json.bind(("127.0.0.1", json_port))
    rx_json.settimeout(0.05)
    rx_gga = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_gga.bind(("127.0.0.1", gga_port))
    rx_gga.settimeout(0.05)
    rx_bad = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_bad.bind(("127.0.0.1", bad_port))
    rx_bad.settimeout(0.0)

    proc = subprocess.Popen(
        [str(BIN), f"--scene={world}"],
        cwd=str(REPO), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    json_msgs, gga_msgs, bad_msgs = [], [], []
    try:
        t_end = time.time() + CAPTURE_S
        while time.time() < t_end:
            if proc.poll() is not None:
                return None, None, None, f"vdsim_realtime exited early rc={proc.returncode}"
            for sock, sink in ((rx_json, json_msgs), (rx_gga, gga_msgs)):
                try:
                    sink.append(sock.recvfrom(1024)[0])
                except socket.timeout:
                    pass
        while True:
            try:
                bad_msgs.append(rx_bad.recvfrom(1024)[0])
            except (socket.timeout, BlockingIOError):
                break
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        rx_json.close()
        rx_gga.close()
        rx_bad.close()
    return json_msgs, gga_msgs, bad_msgs, None


def _check_json(json_msgs, errs):
    """Validate the JSON stream and return vehicle 0's truth x/y samples."""
    xs, ys = [], []
    for raw in json_msgs:
        try:
            doc = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as exc:
            errs.append(f"json parse: {exc}: {raw!r}")
            return xs, ys
        missing = {"id", "t", "x", "y", "yaw", "vx", "vy", "r", "ax", "ay",
                   "Fz", "alpha", "kappa"} - set(doc)
        if missing:
            errs.append(f"json missing keys {sorted(missing)}")
            return xs, ys
        if len(doc["Fz"]) != 4 or len(doc["alpha"]) != 4 or len(doc["kappa"]) != 4:
            errs.append(f"json per-wheel arrays malformed: {doc}")
            return xs, ys
        if doc["x"] is None or doc["y"] is None:
            errs.append(f"json position went non-finite: {doc}")
            return xs, ys
        xs.append(doc["x"])
        ys.append(doc["y"])
    return xs, ys


def _check_gga_tracks_vehicle(gga_msgs, origin, xs, ys, errs):
    """The reported fix must follow the vehicle, not sit on the datum."""
    easts, norths = [], []
    for raw in gga_msgs:
        err, east, north = _check_gga(raw, origin)
        if err:
            errs.append(f"gga {err}")
            return
        easts.append(east)
        norths.append(north)
    if not easts:
        return
    # 1. The lateral offset pins the fix to the vehicle's spawn lane, well
    #    outside the 0.185 m degree-minute quantum. A GGA hard-wired to the
    #    datum would report 0 here, 1.5 m away.
    tol = 4.0 * GGA_QUANTUM_M
    if abs(min(norths) - SPAWN_Y) > tol or abs(max(norths) - SPAWN_Y) > tol:
        errs.append(f"gga north offset {min(norths):.3f}..{max(norths):.3f} m "
                    f"does not match the spawn lane y={SPAWN_Y}")
    # 2. The along-track offset has to actually move: a constant fix fails here.
    travel = max(easts) - min(easts)
    if travel < MIN_TRAVEL_M:
        errs.append(f"gga east offset only spans {travel:.3f} m: the fix is not "
                    f"tracking the vehicle")
    if abs(min(easts) - SPAWN_X) > 2.0:
        errs.append(f"gga east offset starts at {min(easts):.3f} m, expected the "
                    f"spawn x={SPAWN_X}")
    # 3. And it has to be *this* vehicle: the fix range must match the truth
    #    pose reported on the json channel to within a tick of motion.
    if xs and ys:
        if abs(min(easts) - min(xs)) > 1.0 or abs(max(easts) - max(xs)) > 1.0:
            errs.append(f"gga east {min(easts):.3f}..{max(easts):.3f} m does not "
                        f"match json x {min(xs):.3f}..{max(xs):.3f} m")
        if abs(min(norths) - min(ys)) > tol or abs(max(norths) - max(ys)) > tol:
            errs.append(f"gga north {min(norths):.3f}..{max(norths):.3f} m does not "
                        f"match json y {min(ys):.3f}..{max(ys):.3f} m")


def main():
    if not BIN.is_file():
        print("SKIP: vdsim_realtime not built")
        return 0
    if not SCENE.is_file():
        print(f"FAIL: missing {SCENE}")
        return 1

    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        ports = [_free_udp_port() for _ in range(4)]
        if len(set(ports)) != 4:
            print("SKIP: could not obtain four distinct udp ports")
            return 0
        json_port, gga_port, bad_port, rx_port = ports
        try:
            world, origin = _write_world(tmp, json_port, gga_port, bad_port, rx_port)
        except AssertionError as exc:
            print(f"FAIL: shipped comms sample: {exc}")
            return 1
        json_msgs, gga_msgs, bad_msgs, fatal = _capture(
            world, json_port, gga_port, bad_port)
    if fatal:
        print(f"FAIL: {fatal}")
        return 1

    errs = []
    if len(json_msgs) < MIN_DATAGRAMS:
        errs.append(f"json: only {len(json_msgs)} datagrams received")
    if len(gga_msgs) < MIN_DATAGRAMS:
        errs.append(f"nmea_gga: only {len(gga_msgs)} datagrams received")
    if bad_msgs:
        errs.append(f"unknown template was not skipped: {len(bad_msgs)} datagrams")
    xs, ys = _check_json(json_msgs, errs)
    _check_gga_tracks_vehicle(gga_msgs, origin, xs, ys, errs)

    if errs:
        for e in errs:
            print(f"FAIL: {e}")
        return 1
    print(f"test_comms_tx_templates: ok (json={len(json_msgs)}, gga={len(gga_msgs)}, "
          f"datum={origin[0]},{origin[1]} from {COMMS_SPEC.name})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
