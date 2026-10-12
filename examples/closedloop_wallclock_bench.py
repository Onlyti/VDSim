#!/usr/bin/env python3
"""Repeat the wall-clock (mode C) checks of work order 31 and log the host load.

Two measurements, each written as one JSON line per run:

* ``s7`` -- the controller blocks for ``3*Tc`` (``time.sleep`` or a pure-Python
  spin that holds the GIL) under overrun policy ``skip`` or ``queue``; the sim
  ticks counted during the block are compared with ``0.8 * 3*Tc/substep``.
* ``s6`` -- a plain lane-keeping run of ``--duration`` seconds; RTF, control
  period p50/p99/max, overrun count and wake jitter are logged.

Every record carries the 1/5/15-minute load average read right before the run
and the CPU affinity the process inherited, so results taken on a busy shared
host can be told apart from a quiet one.

``--max-load`` makes every run wait until the 1-minute load average is below
the given value (the record still stores the value read right before the run).

Usage (Linux, faulty cores excluded)::

    taskset -c 0-9,12-23 python3 examples/closedloop_wallclock_bench.py s7 \\
        --repeat 20 --out s7.jsonl
    taskset -c 0-9,12-23 python3 examples/closedloop_wallclock_bench.py s6 \\
        --repeat 5 --duration 10 --out s6.jsonl
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_closedloop import ClosedLoopEval  # noqa: E402

TC = 0.02            # control period [s]
H = 5e-4             # substep [s]
V0 = 16.7            # initial speed [m/s]
STATE0 = [0.0, 0.0, 0.0, V0, 0.0, 0.0]
PLANT = dict(level="L2", substep_dt=H)
S7_RATIO = 0.8       # pass line of S7 relative to the nominal tick count


def controller(obs, t):
    """Hold the lane (Y = 0) and the speed.

    @param obs observation dict from ClosedLoopEval
    @param t   sample time [s]
    @return (steer [rad], longitudinal force [N])
    """
    delta = max(-0.3, min(0.3, -0.08 * obs["Y"] - 0.9 * obs["psi"]))
    return delta, 2000.0 * (V0 - obs["vx"])


def spin(seconds):
    """Busy-wait in pure Python, holding the GIL except at interpreter switches.

    @param seconds wall time to spin [s]
    @return loop iteration count
    """
    end = time.perf_counter() + seconds
    x = 0
    while time.perf_counter() < end:
        x += 1
    return x


def host_state():
    """Load average and inherited CPU affinity of this process.

    @return dict with ``load1``, ``load5``, ``load15`` and ``cpus`` (None off Linux)
    """
    l1, l5, l15 = os.getloadavg()
    cpus = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
    return {"load1": l1, "load5": l5, "load15": l15, "cpus": cpus}


def wait_for_load(max_load, timeout_s, poll_s=30.0):
    """Block until the 1-minute load average drops below ``max_load``.

    @param max_load  load threshold; None returns immediately
    @param timeout_s give up after this many seconds
    @param poll_s    polling interval [s]
    @return True when the load is below the threshold, False on timeout
    """
    if max_load is None:
        return True
    deadline = time.monotonic() + timeout_s
    while os.getloadavg()[0] >= max_load:
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_s)
    return True


def run_s7(block_name, overrun):
    """One S7 run: block the controller for 3*Tc on its third call.

    @param block_name ``"sleep"`` or ``"gil_spin"``
    @param overrun    overrun policy ``"skip"`` or ``"queue"``
    @return record dict (ticks during the block, nominal, pass flag, stats)
    """
    block = {"sleep": time.sleep, "gil_spin": spin}[block_name]
    ev, log, calls = [None], [], {"n": 0}

    def ctrl(obs, t):
        calls["n"] += 1
        if calls["n"] == 3:
            before = ev[0].sim_ticks()
            block(3.0 * TC)
            log.append((before, ev[0].sim_ticks()))
        return controller(obs, t)

    ev[0] = ClosedLoopEval(PLANT, ctrl, TC, mode="wallclock", duration=1.0,
                           overrun=overrun, state0=STATE0)
    st = ev[0].run().stats
    nominal = 3.0 * TC / H
    advanced = log[0][1] - log[0][0]
    return {"block": block_name, "overrun": overrun, "ticks": advanced,
            "nominal": nominal, "pass_line": S7_RATIO * nominal,
            "pass": advanced >= S7_RATIO * nominal,
            "n_overrun": st["n_overrun"], "n_skipped": st["n_skipped"],
            "sim_max_lag_s": st["sim_max_lag_s"]}


def run_s6(duration):
    """One S6 run: plain lane keeping on the wall clock.

    @param duration run length [s]
    @return record dict with RTF, period and jitter percentiles, overruns
    """
    st = ClosedLoopEval(PLANT, controller, TC, mode="wallclock",
                        duration=duration, state0=STATE0).run().stats
    keys = ("rtf", "n_samples", "n_overrun", "n_skipped", "period_p50_s",
            "period_p99_s", "period_max_s", "wake_jitter_p99_s",
            "sim_late_ticks", "sim_max_lag_s")
    rec = {k: st[k] for k in keys}
    rec["duration"] = duration
    return rec


def main(argv=None):
    """Command-line entry: repeat s6 or s7 runs and append JSON lines.

    @param argv argument list (default sys.argv[1:])
    @return process exit code (0 done, 2 load wait timed out; pass/fail is in the records)
    """
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=("s6", "s7"))
    ap.add_argument("--repeat", type=int, default=20)
    ap.add_argument("--duration", type=float, default=10.0, help="s6 run length [s]")
    ap.add_argument("--out", type=Path, required=True, help="JSON-lines output")
    ap.add_argument("--max-load", type=float, default=None,
                    help="wait before each run until the 1-min load average is below this")
    ap.add_argument("--wait-timeout-h", type=float, default=12.0,
                    help="give up waiting for --max-load after this many hours")
    args = ap.parse_args(argv)
    timeout_s = 3600.0 * args.wait_timeout_h

    with args.out.open("a") as f:
        for i in range(args.repeat):
            if args.what == "s7":
                for block in ("sleep", "gil_spin"):
                    for overrun in ("skip", "queue"):
                        if not wait_for_load(args.max_load, timeout_s):
                            print("load wait timed out", flush=True)
                            return 2
                        rec = dict(host_state(), what="s7", rep=i)
                        rec.update(run_s7(block, overrun))
                        f.write(json.dumps(rec) + "\n")
                        f.flush()
            else:
                if not wait_for_load(args.max_load, timeout_s):
                    print("load wait timed out", flush=True)
                    return 2
                rec = dict(host_state(), what="s6", rep=i)
                rec.update(run_s6(args.duration))
                f.write(json.dumps(rec) + "\n")
                f.flush()
            print(f"[progress] {i + 1}/{args.repeat} rep stage={args.what}",
                  flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
