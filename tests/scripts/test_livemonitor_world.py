#!/usr/bin/env python3
"""Live monitor in code runs (work order 37): checks without a browser.

* W1 (G1) ``monitor`` not given / ``False``: building and stepping a
  ``VDSimPlant`` and a ``VDSimVecEnv`` adds no thread and no socket and does
  not import ``vdsim_livemonitor``.
* W2 (G1) a monitored ``VDSimVecEnv`` gives the same observations and rewards,
  bit for bit, as an unmonitored one (fixed seed and actions).
* W3 world protocol: the fleet snapshot reaches a page before the states;
  states of ids outside the fleet are dropped; a reset bumps only that id's
  counter; a new session clears the previous run's states; ``finish`` reaches
  the page; a single-vehicle ``state`` publisher still gets the ``state``
  message (G7).
* W4 (G2) ``VDSimPlant(monitor=port)``: the world is on the page before the
  first step, the vehicle ``ego`` arrives with the first state, ``close``
  sends "finished" and frees the port.
* W5 ``VDSimVecEnv(monitor=port)``: K of N envs drawn with distinct colours,
  grid offsets, an env reset bumps only its counter, the pool is read only
  when a send is due.
* W6 (G8) a port held by something else, missing page assets, or a publish
  that raises: one warning line, the run goes on unmonitored.
* W7 (G6, coarse) ``monitor_wait_s`` with no page returns after about that
  long with one warning; caller errors in the options are refused.
"""
import base64
import contextlib
import hashlib
import io
import json
import os
import socket
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

import numpy as np  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    """Record one assertion."""
    print(f"{'PASS' if ok else 'FAIL'} {name} {detail}", flush=True)
    if not ok:
        FAILS.append(name)


def n_sockets():
    """Open sockets of this process (Linux), or None elsewhere."""
    d = Path("/proc/self/fd")
    if not d.is_dir():
        return None
    n = 0
    for f in d.iterdir():
        try:
            n += os.readlink(f).startswith("socket:")
        except OSError:
            pass
    return n


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


def port_free(port):
    """True when the TCP port can be bound again (as the server binds)."""
    t = socket.socket()
    t.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        t.bind(("0.0.0.0", port))
        return True
    except OSError:
        return False
    finally:
        t.close()


class WsClient:
    """Minimal WebSocket client for text frames (connects, retrying)."""

    def __init__(self, port, timeout=5.0):
        end = time.monotonic() + timeout
        while True:
            try:
                self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
                break
            except OSError:
                if time.monotonic() > end:
                    raise
                time.sleep(0.02)
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
        self.log = []

    def recv(self, timeout=3.0):
        """Next text message as dict, or None on timeout/close."""
        import vdsim_livemonitor as lm
        self.s.settimeout(timeout)
        try:
            fr = lm._ws_read_frame(self.s)
        except (socket.timeout, OSError):
            return None
        if fr is None or fr[0] != 0x1:
            return None
        m = json.loads(fr[1].decode())
        self.log.append(m)
        return m

    def wait_for(self, pred, timeout=3.0):
        """First message satisfying ``pred`` within ``timeout`` [s]."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.recv(max(0.05, end - time.monotonic()))
            if m is not None and pred(m):
                return m
        return None

    def close(self):
        """Send a masked close frame and drop the socket."""
        try:
            self.s.sendall(struct.pack("!BB", 0x88, 0x80) + b"\0\0\0\0")
        except OSError:
            pass
        self.s.close()


def rollout(env, n=150):
    """sha256 of the observations and rewards of ``n`` fixed-action steps."""
    h = hashlib.sha256()
    obs, _ = env.reset(seed=7)
    h.update(np.ascontiguousarray(obs).tobytes())
    rng = np.random.default_rng(11)
    for _ in range(n):
        a = rng.uniform(-1, 1, size=(env.num_envs, 2)).astype(np.float32)
        obs, rew, term, trunc, _ = env.step(a)
        h.update(np.ascontiguousarray(obs).tobytes())
        h.update(np.ascontiguousarray(rew).tobytes())
    return h.hexdigest()


def is_world(m):
    return m.get("type") == "world"


# ---------------------------------------------------------------------------
def w1_default_off():
    from vdsim_plant import VDSimPlant
    from vdsim_rl import EnvConfig, VDSimVecEnv
    th0, so0 = threading.active_count(), n_sockets()
    p = VDSimPlant(control_dt=0.01, substep_dt=0.001)
    p.reset([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])
    for _ in range(20):
        p.step([0.01, 500.0])
    p.close()
    q = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=False)
    q.step([0.0, 0.0])
    env = VDSimVecEnv(4, EnvConfig(threads=1), seed=1)
    rollout(env, 20)
    env.close()
    check("W1 no monitor thread", threading.active_count() == th0,
          f"{th0} -> {threading.active_count()}")
    so1 = n_sockets()
    check("W1 no monitor socket", so0 is None or so1 == so0, f"{so0} -> {so1}")
    check("W1 vdsim_livemonitor not imported", "vdsim_livemonitor" not in sys.modules)
    check("W1 attributes off", p.monitor is None and q.monitor is None and env.monitor is None)


def w2_bit_identical():
    from vdsim_rl import EnvConfig, VDSimVecEnv
    cfg = EnvConfig(threads=1, time_limit_s=0.5)       # resets inside the run
    off = VDSimVecEnv(8, cfg, seed=3)
    h_off = rollout(off)
    off.close()
    port = free_port()
    with contextlib.redirect_stderr(io.StringIO()):
        on = VDSimVecEnv(8, cfg, seed=3, monitor=port, monitor_max_vehicles=4)
        h_on = rollout(on)
        on.close()
    check("W2 obs/reward identical with monitor on", h_on == h_off, h_on[:16])


def w3_protocol():
    import vdsim_livemonitor as lm
    port = free_port()
    srv = lm.MonitorServer(port, "127.0.0.1", log=lambda m: None).start()
    try:
        ws = WsClient(port)
        ws.recv()                                    # hello
        mon = lm.LiveMonitor(port=port, rate_hz=200.0).start()
        w = mon.set_world("rl", ["a", "b", "c"], n_total=5,
                          offsets=[[0, 0], [0, 8], [0, 16]])
        m = ws.wait_for(is_world)
        wv = (m or {}).get("world", {})
        check("W3 world reaches page", wv.get("session") == w["session"]
              and [v["id"] for v in wv.get("vehicles", [])] == ["a", "b", "c"]
              and wv.get("n_total") == 5 and wv["vehicles"][2]["offset"] == [0.0, 16.0], str(wv)[:120])
        mon.push_states([("a", {"X": 1, "Y": 2, "psi": 0}), ("b", {"X": 3, "Y": 4, "psi": 0}),
                         ("zz", {"X": 9, "Y": 9, "psi": 0})],
                        {"t_sim": 1.0, "t_wall": 0.5, "rate": 2.0})
        m = ws.wait_for(lambda m: m.get("type") == "states")
        ids = sorted(s["id"] for s in (m or {}).get("states", []))
        check("W3 states by id, unknown id dropped", ids == ["a", "b"], str(ids))
        check("W3 clock forwarded", (m or {}).get("clock", {}).get("rate") == 2.0)
        time.sleep(0.01)
        mon.reset_vehicles(["b"])
        mon.push_states([("b", {"X": 0, "Y": 0, "psi": 0})])
        m = ws.wait_for(is_world)
        r = {v["id"]: v["resets"] for v in (m or {}).get("world", {}).get("vehicles", [])}
        check("W3 reset bumps only that id", r == {"a": 0, "b": 1, "c": 0}, str(r))
        legacy = lm.LiveMonitor(port=port).start()
        legacy.push({"X": 5.0, "Y": 6.0, "psi": 0.1})
        m = ws.wait_for(lambda m: m.get("type") == "state")
        check("W3 single-state publisher unchanged (G7)",
              m is not None and m["state"]["x"] == 5.0, str(m)[:80])
        legacy.stop()
        w2 = mon.set_world("code", ["ego"])
        m = ws.wait_for(is_world)
        hello = WsClient(port)
        got = [hello.recv() for _ in range(3)]
        types = [g.get("type") for g in got if g]
        st = next((g for g in got if g and g.get("type") == "states"), None)
        check("W3 new session drops old states", (m or {}).get("world", {}).get("session") == w2["session"]
              and st is None, str(types))
        hello.close()
        mon.stop()
        m = ws.wait_for(lambda m: is_world(m) and m["world"].get("finished"))
        check("W3 finish reaches page", m is not None)
        ws.close()
    finally:
        srv.stop()


def w4_plant():
    from vdsim_plant import VDSimPlant
    for run in range(3):
        port = free_port()
        box = {}

        def page():
            box["ws"] = WsClient(port, timeout=10.0)

        th = threading.Thread(target=page)
        th.start()
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            p = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=port, monitor_wait_s=10.0)
        th.join()
        ws = box["ws"]
        m = ws.wait_for(is_world, 2.0)
        before = [x for x in ws.log if x.get("type") in ("state", "states")]
        check(f"W4.{run} world before first step",
              m is not None and m["world"]["mode"] == "code"
              and [v["id"] for v in m["world"]["vehicles"]] == ["ego"] and not before)
        p.reset([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])
        t0 = time.monotonic()
        for _ in range(30):
            p.step([0.02, 800.0])
        m = ws.wait_for(lambda m: m.get("type") == "states", 1.0)
        dt = time.monotonic() - t0
        s = ((m or {}).get("states") or [{}])[0]
        check(f"W4.{run} vehicle state within 1 s", m is not None and s.get("id") == "ego"
              and isinstance(s.get("x"), float) and dt <= 1.0, f"{dt:.3f} s")
        check(f"W4.{run} URL line", f":{port}/" in err.getvalue(), err.getvalue().strip()[:90])
        t0 = time.monotonic()
        p.close()
        fin = ws.wait_for(lambda m: is_world(m) and m["world"].get("finished"), 2.0)
        while not port_free(port) and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        dt = time.monotonic() - t0
        check(f"W4.{run} finished then port freed <= 5 s", fin is not None and port_free(port),
              f"{dt:.3f} s")
        ws.close()
        p.step([0.0, 0.0])                            # still usable after close


class _CountingPool:
    """Forwards to a VecSession and counts ``outputs()`` reads."""

    def __init__(self, vs):
        self._vs, self.reads = vs, 0

    def outputs(self):
        self.reads += 1
        return self._vs.outputs()

    def __getattr__(self, name):
        return getattr(self._vs, name)


def w5_vec_env():
    from vdsim_rl import EnvConfig, VDSimVecEnv
    port = free_port()
    cfg = EnvConfig(threads=1, time_limit_s=0.2)
    with contextlib.redirect_stderr(io.StringIO()):
        env = VDSimVecEnv(20, cfg, seed=2, monitor=port, monitor_max_vehicles=16,
                          monitor_layout="grid", monitor_grid_spacing=6.0)
    ws = WsClient(port)
    env.reset(seed=2)
    m = ws.wait_for(is_world)
    vs = (m or {}).get("world", {}).get("vehicles", [])
    check("W5 16 of 20 drawn", len(vs) == 16 and m["world"]["n_total"] == 20
          and m["world"]["mode"] == "rl")
    check("W5 distinct colours", len({v["color"] for v in vs}) == 16)
    check("W5 grid offsets", [v["offset"][1] for v in vs[:3]] == [0.0, 6.0, 12.0])
    m = ws.wait_for(lambda m: m.get("type") == "states")
    check("W5 states of the drawn envs", m is not None and len(m["states"]) == 16
          and m["states"][3]["id"] == "env3")
    att = env.monitor
    base = {v["id"]: v["resets"] for v in att.mon.world["vehicles"]}
    done = np.zeros(20, dtype=bool)
    done[[2, 17]] = True                              # env 17 is not drawn
    env.core.reset_done(done)
    after = {v["id"]: v["resets"] for v in att.mon.world["vehicles"]}
    bumped = sorted(k for k in after if after[k] != base[k])
    check("W5 reset bumps only the reset env", bumped == ["env2"], str(bumped))
    pool = _CountingPool(env.core.vs)
    env.core.vs = pool
    a = np.zeros((20, 2), dtype=np.float32)
    # The rate gate is driven directly so the check does not depend on how
    # fast a step runs (a Debug build steps slower than the monitor rate).
    due_steps = {0, 3, 7}
    for k in range(12):
        att.mon._last_push = -float("inf") if k in due_steps \
            else time.monotonic() + 3600.0
        env.step(a)
    check("W5 pool read only when due", pool.reads == len(due_steps),
          f"{pool.reads} reads / 12 steps, {len(due_steps)} due")
    att.mon._last_push = -float("inf")
    pool.reads = 0
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < 1.0:
        env.step(a)
        n += 1
    el = time.monotonic() - t0
    bound = att.mon.rate_hz * el + 2
    check("W5 pool reads within the monitor rate",
          pool.reads <= min(n, bound),
          f"{pool.reads} reads / {n} steps in {el:.2f} s")
    env.core.vs = pool._vs
    env.close()
    fin = ws.wait_for(lambda m: is_world(m) and m["world"].get("finished"), 2.0)
    check("W5 close -> finished", fin is not None)
    ws.close()


def w6_failures():
    import vdsim_livemonitor as lm
    from vdsim_plant import VDSimPlant
    port = free_port()
    blocker = socket.socket()
    blocker.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    blocker.bind(("0.0.0.0", port))
    blocker.listen(1)
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            p = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=port)
            p.reset([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
            for _ in range(10):
                o = p.step([0.0, 0.0])
    finally:
        blocker.close()
    lines = [x for x in err.getvalue().splitlines() if x.strip()]
    check("W6 foreign port: one warning, run goes on",
          p.monitor is None and len(lines) == 1 and "warning" in lines[0] and o["X"] > 0.0,
          lines[0][:90] if lines else "")
    real = lm._web_dir
    with tempfile.TemporaryDirectory() as d:
        lm._web_dir = lambda: Path(d)
        err = io.StringIO()
        try:
            with contextlib.redirect_stderr(err):
                p = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=free_port())
                p.step([0.0, 0.0])
        finally:
            lm._web_dir = real
    lines = [x for x in err.getvalue().splitlines() if x.strip()]
    check("W6 missing page assets: one warning, run goes on",
          p.monitor is None and len(lines) == 1 and "assets missing" in lines[0],
          lines[0][:90] if lines else "")
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        port = free_port()
        p = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=port)
        mon = p.monitor.mon

        def boom(*a, **k):
            raise ValueError("synthetic publish failure")

        mon.push_states = boom
        p.reset([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
        for _ in range(5):
            p.step([0.0, 0.0])
    lines = [x for x in err.getvalue().splitlines() if "warning" in x]
    check("W6 publish failure detaches, run goes on",
          p.monitor.mon is None and len(lines) == 1 and port_free(port),
          lines[0][:90] if lines else "")
    p.close()


def w7_wait_and_args():
    import vdsim_livemonitor as lm
    from vdsim_plant import VDSimPlant
    err = io.StringIO()
    port = free_port()
    t0 = time.monotonic()
    with contextlib.redirect_stderr(err):
        p = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=port, monitor_wait_s=1.5)
    dt = time.monotonic() - t0
    warn = [x for x in err.getvalue().splitlines() if "no monitor page" in x]
    check("W7 wait_s without a page: waits, one warning, continues",
          1.4 <= dt <= 4.0 and len(warn) == 1 and p.monitor is not None, f"{dt:.2f} s")
    p.close()
    with contextlib.redirect_stderr(io.StringIO()):
        p = VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=free_port(), monitor_pace=1.0)
    time.sleep(0.3)                                   # idle before the run: not paced
    p.reset([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
    t0 = time.monotonic()
    for _ in range(50):
        p.step([0.0, 0.0])
    dt = time.monotonic() - t0
    p.close()
    check("W7 pace 1.0 holds 0.5 s of sim to >= 0.475 s wall", dt >= 0.475, f"{dt:.3f} s")
    bad = [dict(monitor_pace=0.0), dict(monitor_wait_s=-1.0)]
    for kw in bad:
        try:
            VDSimPlant(control_dt=0.01, substep_dt=0.001, monitor=free_port(), **kw)
            check(f"W7 refuses {kw}", False)
        except ValueError:
            check(f"W7 refuses {kw}", True)
    for kw, exc in ((dict(layout="ring"), ValueError), (dict(max_vehicles=0), ValueError),
                    (dict(spacing=-1.0), ValueError)):
        try:
            lm.open_monitor(free_port(), mode="rl", n_vehicles=4, **kw)
            check(f"W7 open_monitor refuses {kw}", False)
        except exc:
            check(f"W7 open_monitor refuses {kw}", True)
    try:
        lm.open_monitor("8790", mode="code", n_vehicles=1)
        check("W7 monitor type refused", False)
    except TypeError:
        check("W7 monitor type refused", True)


if __name__ == "__main__":
    for fn in (w1_default_off, w2_bit_identical, w3_protocol, w4_plant, w5_vec_env,
               w6_failures, w7_wait_and_args):
        fn()
    print("ALL CHECKS PASSED" if not FAILS else f"FAILED: {FAILS}")
    sys.exit(1 if FAILS else 0)
