#!/usr/bin/env python3
"""Simulink UDP bridge — VDSim realtime ↔ Simulink UDP Send/Receive blocks.

This script is the Python side of a Simulink co-sim:
  Simulink (controller) --UDP cmd--> [this bridge] --> vdsim_realtime
  vdsim_realtime --UDP state--> [this bridge] --> Simulink (sensor inputs)

Alternatively, run vdsim_realtime directly and point Simulink UDP blocks
at its cmd/state ports. This script demonstrates the wire format.

Simulink side (R2021b+):
  1. UDP Receive block: port=7002, data type=uint8, length=kStateBytes (see cosim_protocol.hpp)
  2. Byte Unpack block: decode VDS1 state fields (see cosim/protocol.py for offsets)
  3. Your controller subsystem
  4. Byte Pack block: encode VDS1 command
  5. UDP Send block: host=127.0.0.1, port=7001

See docs/SIMULINK_GUIDE.md for full setup.

Usage (Python validation, no Simulink required):
    # Terminal 1: start the sim
    build/bin/vdsim_realtime --scene=configs/scenes/two_vehicle_race.yaml

    # Terminal 2: run this bridge (echoes state + sends a step-steer command)
    python3 examples/simulink/simulink_udp_bridge.py
"""
import socket
import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
sys.path[:0] = [str(REPO / "cosim"), str(REPO / "build" / "python")]

try:
    import protocol
except ImportError:
    protocol = None

# VDS1 command wire format (v4, matches cosim_protocol.hpp CmdFields)
# struct CmdFields { uint32_t magic; uint32_t seq; uint32_t vehicle_id;
#                    float steer; float throttle; float brake; float gear; }
CMD_MAGIC  = 0x56445343   # 'VDSC'
CMD_FORMAT = "<IIIFFF"    # magic, seq, vehicle_id, steer, throttle, brake (no gear field exposed)
CMD_SIZE   = struct.calcsize(CMD_FORMAT)

STATE_HOST = "127.0.0.1"
STATE_PORT = 7002
CMD_HOST   = "127.0.0.1"
CMD_PORT   = 7001


def encode_cmd(seq: int, vehicle_id: int, steer: float, throttle: float, brake: float) -> bytes:
    return struct.pack(CMD_FORMAT, CMD_MAGIC, seq, vehicle_id, steer, throttle, brake)


def decode_state(data: bytes) -> dict | None:
    if protocol:
        return protocol.decode_state(data)
    # Minimal inline decoder (see cosim/protocol.py for full version)
    if len(data) < 16:
        return None
    magic, seq, vid = struct.unpack_from("<III", data, 0)
    if magic != 0x56445356:   # 'VDSV'
        return None
    # x, y, yaw at fixed offsets (see StateFields layout)
    x,   = struct.unpack_from("<f", data, 16)
    y,   = struct.unpack_from("<f", data, 20)
    yaw, = struct.unpack_from("<f", data, 24)
    vx,  = struct.unpack_from("<f", data, 28)
    vy,  = struct.unpack_from("<f", data, 32)
    r,   = struct.unpack_from("<f", data, 36)
    return {"vehicle_id": vid, "x": x, "y": y, "yaw": yaw, "vx": vx, "vy": vy, "r": r}


def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind((STATE_HOST, STATE_PORT))
    rx.settimeout(2.0)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Listening for VDS1 state on {STATE_HOST}:{STATE_PORT}")
    print(f"Sending VDS1 commands to {CMD_HOST}:{CMD_PORT}")
    print("Step-steer test (t>2s: steer=0.03 rad, throttle constant)")
    print("Press Ctrl-C to stop.\n")

    t0 = time.monotonic()
    seq = 0
    try:
        while True:
            try:
                data, _ = rx.recvfrom(2048)
            except socket.timeout:
                print("No state received — is vdsim_realtime running?")
                continue

            st = decode_state(data)
            if not st:
                continue

            t = time.monotonic() - t0
            steer    = 0.03 if t >= 2.0 else 0.0
            throttle = max(0.0, min(1.0, (22.2 - st.get("vx", 0.0)) / 3.0 + 0.05))
            brake    = 0.0
            vid      = st.get("vehicle_id", 0)

            cmd = encode_cmd(seq, vid, steer, throttle, brake)
            tx.sendto(cmd, (CMD_HOST, CMD_PORT))
            seq += 1

            if seq % 200 == 0:   # print at ~1 Hz (200 Hz sim)
                print(f"t={t:.1f}s  vx={st.get('vx',0):.2f}  steer={steer:.3f}  throttle={throttle:.2f}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        rx.close(); tx.close()


if __name__ == "__main__":
    main()
