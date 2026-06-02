#!/usr/bin/env python3
"""Smoke test for vdsim_udp_server: send CMD packets, receive + decode STATE.

Verifies the wire protocol round-trips (CRC32 via zlib must match the C++ side)
and that the vehicle responds to a throttle command (vx increases).

Usage:
  build/bin/vdsim_udp_server configs/vehicles/sedan.yaml \
      configs/tires/default_pacejka.yaml --rate=200 --vx0=0 &
  python3 cosim/test_udp_client.py
"""
import socket
import struct
import time
import zlib

MAGIC = 0x56445331
VERSION = 1
CMD, STATE = 1, 2
CMD_PORT, STATE_PORT = 7001, 7002


def pack_cmd(seq, steer, throttle, brake, gear=1):
    # header(24) + payload + crc(4) = 76
    body = struct.pack("<IHHIId", MAGIC, VERSION, CMD, seq, 0, time.time())
    body += struct.pack("<ddd", steer, throttle, brake)
    body += struct.pack("<iB3x", gear, 0)          # gear, handbrake, 3 pad
    body += struct.pack("<dd", float("nan"), float("nan"))  # aux accel/speed
    assert len(body) == 72, len(body)
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def decode_state(buf):
    assert len(buf) == 220, len(buf)
    want = struct.unpack_from("<I", buf, 216)[0]
    if (zlib.crc32(buf[:216]) & 0xFFFFFFFF) != want:
        raise ValueError("STATE CRC mismatch")
    magic, ver, mtype, seq, _pad, ts = struct.unpack_from("<IHHIId", buf, 0)
    assert magic == MAGIC and ver == VERSION and mtype == STATE
    vals = struct.unpack_from("<14d", buf, 24)   # x..ay
    return dict(seq=seq, x=vals[0], y=vals[1], yaw=vals[5],
                vx=vals[6], vy=vals[7], yaw_rate=vals[11], ax=vals[12])


def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", STATE_PORT))
    rx.settimeout(1.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    seq = 1
    vx0 = vx1 = None
    t_end = time.time() + 2.0
    while time.time() < t_end:
        tx.sendto(pack_cmd(seq, steer=0.0, throttle=0.8, brake=0.0),
                  ("127.0.0.1", CMD_PORT))
        seq += 1
        try:
            buf, _ = rx.recvfrom(512)
            st = decode_state(buf)
            if vx0 is None:
                vx0 = st["vx"]
            vx1 = st["vx"]
        except socket.timeout:
            pass
        time.sleep(0.01)

    ok = vx0 is not None and vx1 is not None and vx1 > vx0 + 0.1
    print(f"vx {vx0:.3f} -> {vx1:.3f} m/s under throttle  "
          f"[{'PASS' if ok else 'FAIL'}]")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
