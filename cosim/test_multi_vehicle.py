#!/usr/bin/env python3
"""Smoke test: world scenario with 2 vehicles, distinct vehicle_id in STATE."""
import socket
import subprocess
import time
from pathlib import Path

from protocol import pack_cmd, decode_state

REPO = Path(__file__).resolve().parent.parent
BIN = REPO / "build" / "bin" / "vdsim_realtime"
SCN = REPO / "configs" / "scenarios" / "two_vehicle_race.yaml"
CMD_PORT, STATE_PORT = 7411, 7412


def main():
    if not BIN.is_file():
        print("SKIP: vdsim_realtime not built")
        return 0
    proc = subprocess.Popen(
        [str(BIN), f"--scenario={SCN}",
         f"--cmd-port={CMD_PORT}", "--state-ip=127.0.0.1",
         f"--state-port={STATE_PORT}", "--rate=200"],
        cwd=str(REPO), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.8)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", STATE_PORT))
    rx.settimeout(2.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seen = set()
    t_end = time.time() + 2.5
    seq = 1
    while time.time() < t_end:
        tx.sendto(pack_cmd(seq, vehicle_id=0, throttle=0.5), ("127.0.0.1", CMD_PORT))
        tx.sendto(pack_cmd(seq, vehicle_id=1, throttle=0.5), ("127.0.0.1", CMD_PORT))
        seq += 1
        try:
            while True:
                buf, _ = rx.recvfrom(512)
                st = decode_state(buf)
                if st:
                    seen.add(st["vehicle_id"])
        except socket.timeout:
            pass
        time.sleep(0.02)
    proc.terminate()
    proc.wait(timeout=2)
    ok = seen >= {0, 1}
    print(f"vehicle_ids seen: {sorted(seen)}  [{'PASS' if ok else 'FAIL'}]")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
