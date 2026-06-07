#!/usr/bin/env python3
"""Smoke test: catalog scene with 2 vehicles, distinct vehicle_id in STATE."""
import socket
import subprocess
import time
from pathlib import Path

from protocol import pack_cmd, decode_state

REPO = Path(__file__).resolve().parent.parent
BIN = REPO / "build" / "bin" / "vdsim_realtime"
SCENE = REPO / "configs" / "scenes" / "two_vehicle_race.yaml"


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def main():
    if not BIN.is_file():
        print("SKIP: vdsim_realtime not built")
        return 0
    if not SCENE.is_file():
        print(f"FAIL: missing {SCENE}")
        return 1
    cmd_port = _free_udp_port()
    state_port = _free_udp_port()
    while state_port == cmd_port:
        state_port = _free_udp_port()
    proc = subprocess.Popen(
        [str(BIN), f"--scene={SCENE}",
         f"--cmd-port={cmd_port}", "--state-ip=127.0.0.1",
         f"--state-port={state_port}", "--rate=200"],
        cwd=str(REPO), stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        time.sleep(0.8)
        if proc.poll() is not None:
            err = proc.stderr.read().decode() if proc.stderr else ""
            print(f"FAIL: plant exited early\n{err}")
            return 1
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", state_port))
        rx.settimeout(0.05)
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        seen = set()
        t_end = time.time() + 2.5
        seq = 1
        while time.time() < t_end:
            tx.sendto(pack_cmd(seq, vehicle_id=0, throttle=0.5), ("127.0.0.1", cmd_port))
            tx.sendto(pack_cmd(seq, vehicle_id=1, throttle=0.5), ("127.0.0.1", cmd_port))
            seq += 1
            for _ in range(32):
                try:
                    buf, _ = rx.recvfrom(512)
                except socket.timeout:
                    break
                st = decode_state(buf)
                if st:
                    seen.add(st["vehicle_id"])
            time.sleep(0.02)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    ok = seen >= {0, 1}
    print(f"vehicle_ids seen: {sorted(seen)}  [{'PASS' if ok else 'FAIL'}]")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
