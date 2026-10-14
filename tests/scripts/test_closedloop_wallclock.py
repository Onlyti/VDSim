#!/usr/bin/env python3
"""Closed-loop evaluation mode C (wallclock): S6-S8.

Acceptance criteria of work order 31 (section 5):

* S6 -- a 10 s run keeps the sim on the wall clock (RTF within 0.98-1.02)
  and reports control-period jitter and overrun counts.
* S7 -- while the controller blocks for ``3*Tc`` the C++ sim thread keeps
  ticking, both when the controller sleeps and when it spins in Python with
  the GIL held, and the late sample is counted as an overrun.
* S8 -- normal end, a controller exception, an early ``stop()`` and a
  KeyboardInterrupt in the caller all join both threads: the Python thread
  count, the native thread count and the child-process count return to their
  baseline, and a fresh interpreter running the mode exits on its own.

Wall-clock results depend on the host; run this file serially (the ctest
entry is RUN_SERIAL) and, on a host with faulty cores, under ``taskset``.
"""
import _thread
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_closedloop import ClosedLoopEval  # noqa: E402

TC = 0.02            # control period [s]
H = 5e-4             # substep [s]
V0 = 16.7            # initial speed [m/s]
STATE0 = [0.0, 0.0, 0.0, V0, 0.0, 0.0]
PLANT = dict(level="L2", substep_dt=H)
RTF_BAND = (0.98, 1.02)


def controller(obs, t):
    """Hold the lane (Y = 0) and the speed; cheap enough to never overrun."""
    delta = max(-0.3, min(0.3, -0.08 * obs["Y"] - 0.9 * obs["psi"]))
    return delta, 2000.0 * (V0 - obs["vx"])


def _eval(ctrl, duration, overrun="skip", **kw):
    return ClosedLoopEval(PLANT, ctrl, TC, mode="wallclock", duration=duration,
                          overrun=overrun, state0=STATE0, **kw)


def _native_threads():
    """Kernel thread count of this process (Linux), else None."""
    task = Path("/proc/self/task")
    return len(os.listdir(task)) if task.is_dir() else None


def _child_processes():
    """PIDs whose parent is this process (Linux /proc scan), else []."""
    me = os.getpid()
    kids = []
    proc = Path("/proc")
    if not proc.is_dir():
        return kids
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
        except OSError:
            continue
        # Field 4 (ppid) follows the parenthesised command name.
        ppid = int(stat.rsplit(")", 1)[1].split()[1])
        if ppid == me:
            kids.append(int(entry.name))
    return kids


def _counts():
    return threading.active_count(), _native_threads(), len(_child_processes())


def _settled_counts(base, timeout=2.0):
    """Poll _counts() until it equals base or timeout expires.

    join() returns once a thread has signalled exit, but the kernel may keep its
    /proc/self/task entry (and the interpreter its OS thread) for a moment
    longer, so a single read right after join can race.  A leaked thread never
    goes away, so waiting a bounded time keeps the check strict.  The poll
    returns as soon as the counts match.

    @return (last counts, seconds waited)
    """
    t0 = time.perf_counter()
    deadline = t0 + timeout
    cur = _counts()
    while cur != base and time.perf_counter() < deadline:
        time.sleep(0.01)
        cur = _counts()
    return cur, time.perf_counter() - t0


def test_s6_wallclock_rtf_and_jitter(duration=10.0):
    """S6: the sim tracks the wall clock; jitter and overruns are reported."""
    res = _eval(controller, duration).run()
    st = res.stats
    report = {k: st[k] for k in (
        "rtf", "sim_ticks", "sim_ticks_planned", "sim_late_ticks", "sim_max_lag_s",
        "n_samples", "n_overrun", "n_skipped", "period_p50_s", "period_p99_s",
        "period_max_s", "wake_jitter_p50_s", "wake_jitter_p99_s",
        "wake_jitter_max_s", "compute_p99_s", "delay_p99_s")}
    print("S6", json.dumps(report))
    assert st["sim_ticks"] == st["sim_ticks_planned"], st
    assert not st["stopped_early"]
    assert RTF_BAND[0] <= st["rtf"] <= RTF_BAND[1], st["rtf"]
    assert abs(st["period_p50_s"] - TC) < 0.1 * TC, st["period_p50_s"]
    assert st["n_samples"] >= 0.9 * duration / TC, st["n_samples"]
    return report


def _blocking_controller(ev, block, log):
    """Controller that blocks for 3*Tc on its third call and logs sim ticks."""
    calls = {"n": 0}

    def ctrl(obs, t):
        calls["n"] += 1
        if calls["n"] == 3:
            before = ev[0].sim_ticks()
            block(3.0 * TC)
            log.append((before, ev[0].sim_ticks()))
        return controller(obs, t)
    return ctrl


def _spin(seconds):
    """Busy-wait in pure Python: holds the GIL except at interpreter switches."""
    end = time.perf_counter() + seconds
    x = 0
    while time.perf_counter() < end:
        x += 1
    return x


def test_s7_sim_runs_while_controller_blocks():
    """S7: sim ticks advance during a 3*Tc block; the late sample is an overrun."""
    expected = 3.0 * TC / H
    for name, block in (("sleep", time.sleep), ("gil_spin", _spin)):
        for overrun in ("skip", "queue"):
            ev, log = [None], []
            ev[0] = _eval(_blocking_controller(ev, block, log), 1.0, overrun=overrun)
            st = ev[0].run().stats
            assert len(log) == 1, log
            advanced = log[0][1] - log[0][0]
            print(f"S7 {name}/{overrun}: ticks during block = {advanced} "
                  f"(nominal {expected:.0f}), overrun = {st['n_overrun']}, "
                  f"skipped = {st['n_skipped']}")
            assert advanced >= 0.8 * expected, (name, overrun, advanced)
            assert st["n_overrun"] >= 1, st
            if overrun == "skip":
                assert st["n_skipped"] >= 2, st
            else:
                assert st["n_skipped"] == 0, st


def test_s8_threads_joined_on_every_exit():
    """S8: normal / exception / stop() / KeyboardInterrupt leave nothing behind."""
    _eval(controller, 0.2).run()            # warm-up: lazy imports, pools
    base = _counts()
    waits = {}

    # Normal end.
    res = _eval(controller, 0.5).run()
    assert not res.stats["stopped_early"]
    assert res.stats["t_stop"] is None           # work order 34: common key
    after, waits["normal"] = _settled_counts(base)
    assert after == base, ("normal", after, base)

    # Controller exception: re-raised on the caller, threads joined.
    def bad(obs, t):
        if t > 0.1:
            raise RuntimeError("controller failed on purpose")
        return controller(obs, t)
    try:
        _eval(bad, 2.0).run()
    except RuntimeError as exc:
        assert "on purpose" in str(exc)
    else:
        raise AssertionError("controller exception was swallowed")
    after, waits["exception"] = _settled_counts(base)
    assert after == base, ("exception", after, base)

    # Early stop() from inside the controller.
    holder = [None]

    def stopper(obs, t):
        if t > 0.1:
            holder[0].stop()
        return controller(obs, t)
    holder[0] = _eval(stopper, 5.0)
    t0 = time.perf_counter()
    res = holder[0].run()
    assert time.perf_counter() - t0 < 2.0
    assert res.stats["stopped_early"], res.stats
    assert res.stats["sim_ticks"] < res.stats["sim_ticks_planned"]
    # t_stop is the session time the run ended at: the closing row's time.
    assert res.stats["t_stop"] == res.trace["t"][-1] > 0.1, res.stats["t_stop"]
    after, waits["stop"] = _settled_counts(base)
    assert after == base, ("stop", after, base)

    # KeyboardInterrupt delivered to the caller's thread mid-run.
    fired = {"done": False}

    def interrupter(obs, t):
        if t > 0.1 and not fired["done"]:
            fired["done"] = True
            _thread.interrupt_main()
        return controller(obs, t)
    try:
        _eval(interrupter, 5.0).run()
    except KeyboardInterrupt:
        pass
    else:
        raise AssertionError("KeyboardInterrupt did not reach the caller")
    after, waits["interrupt"] = _settled_counts(base)
    assert after == base, ("interrupt", after, base)
    print(f"S8: python/native threads and children back to {base} after "
          "normal, exception, stop and KeyboardInterrupt; settle wait [s] = "
          + ", ".join(f"{k} {v:.3f}" for k, v in waits.items()))


def test_s8_check_catches_a_leaked_thread():
    """S8 negative check: a thread leaked during a run still fails after the wait.

    The controller starts one extra thread that outlives run().  The settled
    counts must differ from the baseline after the full 2 s poll, which shows
    the wait does not hide a real leak.  The thread is released afterwards and
    the counts must then return to the baseline.
    """
    _eval(controller, 0.2).run()
    base = _counts()
    release = threading.Event()
    leaked = []

    def leaky(obs, t):
        if not leaked:
            th = threading.Thread(target=release.wait, daemon=True)
            th.start()
            leaked.append(th)
        return controller(obs, t)
    try:
        _eval(leaky, 0.3).run()
        after, waited = _settled_counts(base)
        assert after != base, ("leak not detected", after, base)
        assert waited >= 2.0, waited
    finally:
        release.set()
        for th in leaked:
            th.join(5.0)
    restored, _ = _settled_counts(base)
    assert restored == base, ("after release", restored, base)
    print(f"S8 negative: leaked thread seen after {waited:.2f} s wait "
          f"({after} vs baseline {base}); counts restored after release")


_CHILD = r"""
import sys
sys.path[:0] = sys.argv[1:3]
from vdsim_closedloop import ClosedLoopEval
def c(obs, t):
    if t > 0.1:
        raise RuntimeError("x")
    return (0.0, 0.0)
plant = dict(level="L2", substep_dt=5e-4)
ClosedLoopEval(plant, lambda o, t: (0.0, 0.0), 0.02, mode="wallclock", duration=0.3).run()
try:
    ClosedLoopEval(plant, c, 0.02, mode="wallclock", duration=2.0).run()
except RuntimeError:
    pass
print("child done")
"""


def test_s8_fresh_interpreter_exits():
    """S8: a process that ran mode C (also with an error) exits by itself."""
    proc = subprocess.run(
        [sys.executable, "-c", _CHILD, str(REPO / "build" / "python"),
         str(REPO / "python")],
        capture_output=True, text=True, timeout=60)
    assert proc.returncode == 0, proc.stderr
    assert "child done" in proc.stdout


def test_wallclock_argument_checks():
    """Mode C rejects a latency model; affinity/watchdog are C-only."""
    from vdsim_closedloop import LatencyModel
    bad = [
        dict(mode="wallclock", latency=LatencyModel.fixed(0.0)),
        dict(mode="wallclock", overrun="drop"),
        dict(mode="wallclock", cpu_affinity={"sim": -1, "ctrl": [0]}),
        dict(mode="wallclock", cpu_affinity={"sim": 0}),
        dict(mode="wallclock", watchdog={"timeout_s": 0.0}),
        dict(mode="sync", cpu_affinity={"sim": 0, "ctrl": [1]}),
        dict(mode="sync", watchdog={"timeout_s": 0.1}),
    ]
    for kw in bad:
        try:
            ClosedLoopEval(PLANT, controller, TC, duration=0.1, **kw)
        except ValueError:
            continue
        raise AssertionError(f"accepted {kw}")


def test_watchdog_failsafe_counts():
    """A controller slower than the watchdog timeout triggers the fail-safe."""
    def slow(obs, t):
        time.sleep(0.06)
        return controller(obs, t)
    st = _eval(slow, 0.6, watchdog={"timeout_s": 0.03,
                                    "failsafe_u": (0.0, -3000.0)}).run().stats
    assert st["failsafe_latches"] >= 1, st
    st = _eval(controller, 0.3, watchdog={"timeout_s": 0.2}).run().stats
    assert st["failsafe_latches"] == 0, st


def test_wallclock_trace_timing_channels():
    """Mode C writes one trace row per control sample with its timing.

    Each row holds the state at the sample and the command latched by the
    previous sample, so its timing is that sample's; the first row is the
    initial command (NaN timing).
    """
    import tempfile
    import numpy as np
    import vdsim_trace as vt
    with tempfile.TemporaryDirectory() as tmp:
        res = _eval(controller, 0.5,
                    trace_path=Path(tmp) / "c.vdtrace").run()
        smp = res.samples
        with vt.TraceReader(res.meta["trace_path"]) as tr:
            assert tr.manifest["eval_mode"] == "wallclock"
            assert tr.manifest["overrun_policy"] == "skip"
            n = tr.n_steps
            assert n == smp["k"].size == res.trace["t"].size - 1, (n, smp["k"].size)
            got = {c: np.asarray(tr.channel(c)) for c in vt.EVAL_CHANNELS}
            assert np.isnan([got[c][0] for c in ("t_sample", "t_apply", "c_k")]).all()
            assert got["overrun"][0] == 0.0
            for c in ("t_sample", "t_apply", "c_k"):
                assert np.array_equal(got[c][1:], smp[c][:-1]), c
            assert np.array_equal(got["overrun"][1:], smp["overrun"][:-1].astype(float))
            assert np.array_equal(tr.channel("t"), res.trace["t"][:-1])
            assert np.array_equal(tr.channel("u_steer")[1:], smp["u_steer"][:-1])
            # Rows are the snapshots the controller saw, not one aliased
            # output object read after the run. (Not "X increases": under
            # load two wakes can read the same sim tick.)
            assert np.array_equal(tr.channel("pose")[:, 0], res.trace["X"][:-1])
            assert np.array_equal(tr.channel("pose")[:, 1], res.trace["Y"][:-1])


#: Wall-clock pass/fail lines (S6 RTF band, S7 tick floor). Registered as the
#: ctest ``closedloop_wallclock_timing`` with label ``timing``, which the
#: ``validation`` preset excludes; their evidence is the local low-load run.
TIMING_TESTS = (
    test_s6_wallclock_rtf_and_jitter,
    test_s7_sim_runs_while_controller_blocks,
)

#: Checks whose verdict does not depend on host load.
CORE_TESTS = (
    test_wallclock_argument_checks,
    test_s8_threads_joined_on_every_exit,
    test_s8_check_catches_a_leaked_thread,
    test_s8_fresh_interpreter_exits,
    test_watchdog_failsafe_counts,
    test_wallclock_trace_timing_channels,
)


def main(argv=None):
    """Run one test group.

    @param argv  ``["core"]``, ``["timing"]`` or empty for both.
    """
    argv = sys.argv[1:] if argv is None else argv
    group = argv[0] if argv else "all"
    groups = {"core": CORE_TESTS, "timing": TIMING_TESTS,
              "all": CORE_TESTS + TIMING_TESTS}
    if group not in groups:
        raise SystemExit("usage: test_closedloop_wallclock.py [core|timing]")
    for test in groups[group]:
        test()
    print("test_closedloop_wallclock %s: all ok" % group)


if __name__ == "__main__":
    main()
