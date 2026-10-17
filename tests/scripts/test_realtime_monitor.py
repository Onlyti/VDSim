#!/usr/bin/env python3
"""vdsim_realtime --monitor: the browser monitor lives exactly as long as the
simulator (work order 36, K3). POSIX only; on Windows the test is skipped.

* M1 off by default: no monitor line, no child process, the port stays free.
* M2 ``--monitor=PORT``: the page server child answers ``/health`` as the VDSim
  live monitor, a page client gets states with the page's wire keys (finite,
  simulation time increasing) and the scene message's vehicle geometry.
* M3 shutdown, three times each for the end of the scene (``sim.t_end``),
  SIGINT and SIGTERM: the simulator exits with status 0 within 5 s, the child
  is gone and the TCP/UDP port can be bound again within 5 s.
* M4 SIGKILL of the simulator: the child goes too (parent-death signal).
* M5 monitor trouble is one warning line and the scene still runs to its end
  with status 0: interpreter not found, ``vdsim_livemonitor`` not importable
  (child exits right away), port already held by another socket, bad port.
"""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tests" / "scripts"))
sys.path.insert(0, str(REPO / "python"))

from test_livemonitor import WsClient, check, free_port, http, FAILS  # noqa: E402

BUILD = Path(os.environ.get("VDSIM_BUILD_DIR", REPO / "build"))
BIN = BUILD / "bin" / "vdsim_realtime"
SCENE_SRC = REPO / "configs" / "scenes" / "two_vehicle_race.yaml"
WIRE_KEYS = ("x", "y", "yaw", "t", "vx", "delta", "roll", "pitch", "z")
LIMIT_S = 5.0          # K3 shutdown limit [s]
REPEATS = 3


def scene_with_end(tmp, t_end):
    """World file of the shipped two-car scene that stops at ``t_end`` [s] of simulation time.

    The catalog scene is materialized once (tools/materialize_scene.py, the
    server's own step) and ``sim.t_end`` is set on the resulting world file,
    since a catalog scene's own ``sim:`` block is not carried over.
    """
    world = Path(tmp) / "world.yaml"
    if not world.exists():
        subprocess.run([sys.executable, str(REPO / "tools" / "materialize_scene.py"),
                        str(SCENE_SRC), str(world)], cwd=REPO, check=True,
                       stdout=subprocess.DEVNULL)
    d = yaml.safe_load(world.read_text())
    d.setdefault("sim", {})["t_end"] = float(t_end)
    p = Path(tmp) / f"world_tend_{t_end:g}.yaml"
    p.write_text(yaml.safe_dump(d, sort_keys=False))
    return p


def start(scene, *extra):
    """Run the server from the repo root with private command/state ports."""
    args = [str(BIN), f"--scene={scene}", f"--cmd-port={free_port()}",
            f"--state-port={free_port()}", *extra]
    return subprocess.Popen(args, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)


def children(pid):
    """Direct child process ids of ``pid`` (Linux /proc)."""
    out = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            fields = (d / "stat").read_text().rsplit(")", 1)[1].split()
        except OSError:
            continue
        if int(fields[1]) == pid:
            out.append(int(d.name))
    return out


def alive(pid):
    """True while ``pid`` exists and is not a zombie."""
    try:
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except OSError:
        return False
    return state != "Z"


def port_free(port):
    """Both the page (TCP 0.0.0.0) and the ingest (UDP 127.0.0.1) port can be bound."""
    t = socket.socket()
    t.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        t.bind(("0.0.0.0", port))
        u.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        t.close()
        u.close()


def wait_until(pred, timeout):
    """Seconds until ``pred()`` holds, or None after ``timeout``."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return time.monotonic() - t0
        time.sleep(0.02)
    return None


def wait_health(port, timeout=LIMIT_S):
    """Seconds until ``/health`` reports a live VDSim monitor, or None."""
    def ok():
        try:
            st, body = http(port, "/health")
        except OSError:
            return False
        import json
        return st == 200 and json.loads(body).get("service") == "vdsim-livemonitor" \
            and json.loads(body).get("live") is True
    return wait_until(ok, timeout)


def child_of(proc, timeout=LIMIT_S):
    """The page server child of ``proc`` once it exists."""
    found = []
    wait_until(lambda: bool(found.extend(children(proc.pid)) or found), timeout)
    return found[0] if found else None


def m1_off(tmp):
    scene = scene_with_end(tmp, 1.0)
    p = start(scene)
    time.sleep(0.5)
    kids = children(p.pid)
    out, _ = p.communicate(timeout=30)
    check("M1 off: exit 0", p.returncode == 0, f"rc={p.returncode}")
    check("M1 off: no monitor line", "monitor" not in out, out.strip().splitlines()[-1:])
    check("M1 off: no child process", kids == [], str(kids))


def m2_on(tmp):
    scene = scene_with_end(tmp, 6.0)
    port = free_port()
    p = start(scene, f"--monitor={port}")
    t_live = wait_health(port)
    check("M2 health live within 5 s", t_live is not None, f"{t_live}")
    ws = WsClient(port)
    msgs = []
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        m = ws.recv(0.5)
        if m and m.get("type") == "state":
            msgs.append(m["state"])
    ws.close()
    _, scene_body = http(port, "/scene.json")
    import json
    sc = json.loads(scene_body)
    p.send_signal(signal.SIGINT)
    out, _ = p.communicate(timeout=30)
    ok_keys = bool(msgs) and all(all(isinstance(s.get(k), (int, float)) for k in WIRE_KEYS)
                                 for s in msgs)
    check("M2 states with wire keys", ok_keys, f"n={len(msgs)} first={msgs[:1]}")
    ts = [s["t"] for s in msgs]
    # A newly connected page may get the current state twice (hello, then the
    # broadcast), so equal neighbours are allowed; time must never go back.
    check("M2 sim time advances", len(ts) > 10 and all(b >= a for a, b in zip(ts, ts[1:]))
          and ts[-1] - ts[0] > 1.0, f"n={len(ts)} t={ts[0]}..{ts[-1]}")
    check("M2 rate <= 30 Hz", len(msgs) <= 2.0 * 30 + 3, f"n={len(msgs)} in 2 s")
    veh = sc.get("vehicle", {})
    check("M2 scene vehicle geometry", sc.get("version", 0) >= 1
          and all(veh.get(k, 0) > 0 for k in ("cg_to_front", "cg_to_rear", "track", "wheel_radius")),
          str(veh))
    check("M2 log lines", "monitor: page http://" in out and "page server stopped" in out,
          out.strip().splitlines())


def m3_shutdown(tmp):
    scene_run = scene_with_end(tmp, 600.0)
    scene_end = scene_with_end(tmp, 3.0)
    for how in ("t_end", "SIGINT", "SIGTERM"):
        for i in range(1, REPEATS + 1):
            port = free_port()
            p = start(scene_end if how == "t_end" else scene_run, f"--monitor={port}")
            ready = wait_health(port)
            kid = child_of(p)
            t0 = time.monotonic()
            if how == "SIGINT":
                p.send_signal(signal.SIGINT)
            elif how == "SIGTERM":
                p.send_signal(signal.SIGTERM)
            try:
                out, _ = p.communicate(timeout=30 if how == "t_end" else LIMIT_S + 5)
            except subprocess.TimeoutExpired:
                p.kill()
                out, _ = p.communicate()
            # A natural end has no stop request: time the cleanup from the exit.
            t_exit = 0.0 if how == "t_end" else time.monotonic() - t0
            t1 = time.monotonic()
            w_kid = wait_until(lambda: kid is None or not alive(kid), LIMIT_S)
            t_kid = None if w_kid is None else t_exit + w_kid
            w_port = wait_until(lambda: port_free(port), LIMIT_S)
            t_port = None if w_port is None else t_exit + (time.monotonic() - t1)
            check(f"M3 {how} #{i}", ready is not None and kid is not None and p.returncode == 0
                  and t_exit <= LIMIT_S and t_kid is not None and t_kid <= LIMIT_S
                  and t_port is not None and t_port <= LIMIT_S and "page server stopped" in out,
                  f"rc={p.returncode} exit={t_exit:.3f}s child_gone={t_kid} port_free={t_port}")


def m4_sigkill(tmp):
    scene = scene_with_end(tmp, 600.0)
    port = free_port()
    p = start(scene, f"--monitor={port}")
    wait_health(port)
    kid = child_of(p)
    p.kill()
    p.communicate(timeout=10)
    t_kid = wait_until(lambda: kid is not None and not alive(kid), LIMIT_S)
    t_port = wait_until(lambda: port_free(port), LIMIT_S)
    check("M4 SIGKILL takes the page server along", kid is not None and t_kid is not None
          and t_port is not None, f"child_gone={t_kid} port_free={t_port}")


def m5_trouble(tmp):
    scene = scene_with_end(tmp, 2.0)
    wrapper = Path(tmp) / "python_isolated.sh"
    wrapper.write_text(f"#!/bin/sh\nexec {sys.executable} -I \"$@\"\n")
    wrapper.chmod(0o755)
    held = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    held.bind(("127.0.0.1", 0))
    held_port = held.getsockname()[1]
    cases = [
        ("interpreter not found", [f"--monitor={free_port()}", "--monitor-python=/nonexistent/python3"]),
        ("module not importable", [f"--monitor={free_port()}", f"--monitor-python={wrapper}"]),
        ("port held by another socket", [f"--monitor={held_port}"]),
        ("bad port", ["--monitor=notaport"]),
    ]
    for name, extra in cases:
        p = start(scene, *extra)
        out, _ = p.communicate(timeout=30)
        warn = [l for l in out.splitlines() if "continuing without monitor" in l]
        check(f"M5 {name}: one warning, scene runs to its end",
              p.returncode == 0 and len(warn) == 1 and "page server stopped" not in out
              and "stopped." in out, f"rc={p.returncode} warn={warn}")
    held.close()


if __name__ == "__main__":
    if os.name == "nt":
        print("SKIP vdsim_realtime --monitor is POSIX only")
        sys.exit(0)
    if not BIN.exists():
        print(f"FAIL missing {BIN}")
        sys.exit(1)
    with tempfile.TemporaryDirectory() as tmp:
        for fn in (m1_off, m2_on, m3_shutdown, m4_sigkill, m5_trouble):
            fn(tmp)
    print("ALL CHECKS PASSED" if not FAILS else f"FAILED: {FAILS}")
    sys.exit(1 if FAILS else 0)
