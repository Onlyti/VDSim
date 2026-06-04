#!/usr/bin/env python3
"""Smoke test for vdsim_realtime: send CMD packets, receive + decode STATE.

Verifies the wire protocol round-trips (CRC32 via zlib must match the C++ side)
and that the vehicle responds to a throttle command (vx increases). Uses the
shared protocol module so there is one definition of the wire format.

Usage:
  build/bin/vdsim_realtime configs/vehicles/sedan.yaml \
      configs/tires/default_pacejka.yaml --rate=200 --vx0=0 &
  python3 cosim/test_udp_client.py
"""
import socket
import time

from protocol import CMD_PORT, STATE_PORT, decode_state, pack_cmd


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
            if st is not None:
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
