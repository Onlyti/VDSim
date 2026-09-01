#!/usr/bin/env python3
"""End-to-end: the `json` and `nmea_gga` comms TX templates reach the wire.

The unit tests (tests/unit/test_comms_templates.cpp) cover the encoders; this
test proves the realtime server's TX gate actually accepts the two text
templates, routes them per channel, and that what lands in a UDP socket is
parseable JSON / a checksum-valid NMEA 0183 GGA sentence.
"""
import json
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

sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

# Datum used by the nmea_gga channel under test (Seoul), matching the origin
# written into the temporary world below.
ORIGIN_LAT = 37.5665
ORIGIN_LON = 126.9780


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


def _check_gga(raw):
    """Validate one received datagram as a $GPGGA sentence. Returns an error str or None."""
    text = raw.decode("ascii", "replace")
    if not text.startswith("$") or not text.endswith("\r\n"):
        return f"framing: {text!r}"
    body, _, csum = text[1:-2].partition("*")
    if len(csum) != 2 or csum != csum.upper():
        return f"checksum digits: {text!r}"
    if int(csum, 16) != _nmea_checksum(body):
        return f"checksum mismatch: {text!r}"
    f = body.split(",")
    if len(f) != 15:
        return f"field count {len(f)}: {text!r}"
    if f[0] != "GPGGA":
        return f"sentence id: {text!r}"
    if len(f[1]) != 9 or f[1][6] != ".":
        return f"utc field: {text!r}"
    if len(f[2]) != 9 or f[2][4] != "." or f[3] not in ("N", "S"):
        return f"latitude field: {text!r}"
    if len(f[4]) != 10 or f[4][5] != "." or f[5] not in ("E", "W"):
        return f"longitude field: {text!r}"
    if f[10] != "M" or f[12] != "M":
        return f"altitude units: {text!r}"
    # Vehicles spawn within a few hundred metres of the datum, so the projected
    # fix must land in the origin's degree+minute cell.
    lat = int(f[2][:2]) + float(f[2][2:]) / 60.0
    lon = int(f[4][:3]) + float(f[4][3:]) / 60.0
    if abs(lat - ORIGIN_LAT) > 0.05 or abs(lon - ORIGIN_LON) > 0.05:
        return f"fix {lat},{lon} far from datum: {text!r}"
    return None


def _write_world(tmp, json_port, gga_port, bad_port):
    """Materialize a catalog scene, then attach json / nmea_gga / bogus TX channels."""
    from catalog import materialize_scene_file  # noqa: E402

    world_path = tmp / "world.yaml"
    materialize_scene_file(SCENE, world_path)
    doc = yaml.safe_load(world_path.read_text()) or {}
    doc["sim"] = dict(doc.get("sim") or {})
    doc["sim"]["rate"] = 200.0
    doc["comms"] = {
        "name": "json_nmea_e2e",
        "channels": [
            {"source": "0.state", "template": "json",
             "to": [{"ip": "127.0.0.1", "port": json_port}]},
            {"source": "0.state", "template": "nmea_gga",
             "origin": {"lat": ORIGIN_LAT, "lon": ORIGIN_LON, "alt": 38.0},
             "to": [{"ip": "127.0.0.1", "port": gga_port}]},
            # An unknown template must still be warned about and skipped.
            {"source": "0.state", "template": "not_a_template",
             "to": [{"ip": "127.0.0.1", "port": bad_port}]},
        ],
    }
    world_path.write_text(yaml.safe_dump(doc, sort_keys=False))
    return world_path


def main():
    if not BIN.is_file():
        print("SKIP: vdsim_realtime not built")
        return 0
    if not SCENE.is_file():
        print(f"FAIL: missing {SCENE}")
        return 1

    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        json_port, gga_port, bad_port = (_free_udp_port(), _free_udp_port(), _free_udp_port())
        if len({json_port, gga_port, bad_port}) != 3:
            print("SKIP: could not obtain three distinct udp ports")
            return 0
        world = _write_world(tmp, json_port, gga_port, bad_port)

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
            t_end = time.time() + 3.0
            while time.time() < t_end and (len(json_msgs) < 5 or len(gga_msgs) < 5):
                if proc.poll() is not None:
                    print(f"FAIL: vdsim_realtime exited early rc={proc.returncode}")
                    return 1
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

    errs = []
    if len(json_msgs) < 5:
        errs.append(f"json: only {len(json_msgs)} datagrams received")
    if len(gga_msgs) < 5:
        errs.append(f"nmea_gga: only {len(gga_msgs)} datagrams received")
    if bad_msgs:
        errs.append(f"unknown template was not skipped: {len(bad_msgs)} datagrams")
    for raw in json_msgs:
        try:
            doc = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as exc:
            errs.append(f"json parse: {exc}: {raw!r}")
            break
        missing = {"id", "t", "x", "y", "yaw", "vx", "vy", "r", "ax", "ay",
                   "Fz", "alpha", "kappa"} - set(doc)
        if missing:
            errs.append(f"json missing keys {sorted(missing)}")
            break
        if len(doc["Fz"]) != 4 or len(doc["alpha"]) != 4 or len(doc["kappa"]) != 4:
            errs.append(f"json per-wheel arrays malformed: {doc}")
            break
    for raw in gga_msgs:
        err = _check_gga(raw)
        if err:
            errs.append(f"gga {err}")
            break

    if errs:
        for e in errs:
            print(f"FAIL: {e}")
        return 1
    print(f"test_comms_tx_templates: ok (json={len(json_msgs)}, gga={len(gga_msgs)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
