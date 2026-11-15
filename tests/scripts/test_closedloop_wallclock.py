#!/usr/bin/env python3
"""Closed-loop evaluation mode C (wallclock): S6-S8.

Acceptance criteria of work order 31 (section 5):

* S6 -- a 10 s run keeps the sim on the wall clock (RTF within 0.98-1.02)
  and reports control-period jitter and overrun counts.
* S7 -- while the controller blocks for ``3*Tc`` the C++ sim thread keeps
  ticking, both when the controller sleeps and when it spins in Python with
  the GIL held, and the late sample is counted as an overrun.
* S8 -- normal end, a controller exception, an early ``stop()`` and a
  KeyboardInterrupt in the caller all join both threads: no Python thread,
  native thread (TID) or child process that was not there before the run is
  left afterwards, and a fresh interpreter running the mode exits on its own.

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


def _native_thread_ids():
    """Kernel thread ids (TIDs) of this process (Linux), else None.

    @return frozenset of int TIDs from /proc/self/task, or None off Linux
    """
    task = Path("/proc/self/task")
    if not task.is_dir():
        return None
    return frozenset(int(name) for name in os.listdir(task))


def _python_thread_ids():
    """Native ids of the live Python threads (``Thread.native_id``).

    The kernel TID is used rather than ``Thread.ident``: ``ident`` is the
    pthread handle, which the C library reuses as soon as a thread is freed.

    @return frozenset of int native ids
    """
    return frozenset(th.native_id for th in threading.enumerate())


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


def _ids():
    """Identifier sets of what this process runs right now.

    @return (Python thread native ids, kernel TIDs or None, child PIDs)
    """
    return (_python_thread_ids(), _native_thread_ids(),
            frozenset(_child_processes()))


def _new_ids(base, cur):
    """Identifiers in ``cur`` that were not in the baseline ``base``.

    Only additions count.  An identifier of the baseline that is gone (a thread
    of the test runner or the interpreter that ended meanwhile) is not a leak,
    so a count comparison would fail on it while this one does not.  Known
    limit: a leaked thread that receives the TID of a baseline thread which
    ended in between is missed; Linux hands out TIDs in increasing order, so in
    the short window of one run this is unlikely.

    @param base  tuple from _ids() taken before the run
    @param cur   tuple from _ids() taken after the run
    @return tuple of three frozensets (Python ids, TIDs, child PIDs)
    """
    return tuple(frozenset() if c is None else c - (b or frozenset())
                 for b, c in zip(base, cur))


def _comm(path):
    """Process or thread name from a /proc ``comm`` file, else '?'."""
    try:
        return Path(path).read_text().strip()
    except OSError:
        return "?"


def _describe(new):
    """Readable form of _new_ids() output for an assertion message.

    @param new  tuple of three frozensets (Python ids, TIDs, child PIDs)
    @return dict kind -> {identifier: name}
    """
    py_names = {th.native_id: th.name for th in threading.enumerate()}
    return {
        "python": {i: py_names.get(i, "?") for i in sorted(new[0])},
        "native": {t: _comm(f"/proc/self/task/{t}/comm") for t in sorted(new[1])},
        "child": {p: _comm(f"/proc/{p}/comm") for p in sorted(new[2])},
    }


def _settled_new_ids(base, timeout=2.0):
    """Poll until no identifier outside the baseline is left, or timeout.

    join() returns once a thread has signalled exit, but the kernel may keep its
    /proc/self/task entry (and the interpreter its OS thread) for a moment
    longer, so a single read right after join can race.  A leaked thread never
    goes away, so waiting a bounded time keeps the check strict.  The poll
    returns as soon as no new identifier remains.

    @param base     tuple from _ids() taken before the run
    @param timeout  longest wait [s]
    @return (last _new_ids() tuple, seconds waited)
    """
    t0 = time.perf_counter()
    deadline = t0 + timeout
    new = _new_ids(base, _ids())
    while any(new) and time.perf_counter() < deadline:
        time.sleep(0.01)
        new = _new_ids(base, _ids())
    return new, time.perf_counter() - t0


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
    base = _ids()
    waits = {}

    # Normal end.
    res = _eval(controller, 0.5).run()
    assert not res.stats["stopped_early"]
    assert res.stats["t_stop"] is None           # work order 34: common key
    new, waits["normal"] = _settled_new_ids(base)
    assert not any(new), ("normal", _describe(new))

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
    new, waits["exception"] = _settled_new_ids(base)
    assert not any(new), ("exception", _describe(new))

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
    new, waits["stop"] = _settled_new_ids(base)
    assert not any(new), ("stop", _describe(new))

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
    new, waits["interrupt"] = _settled_new_ids(base)
    assert not any(new), ("interrupt", _describe(new))
    print("S8: no Python thread, native thread or child process created by "
          "the run outlives it after normal, exception, stop and "
          "KeyboardInterrupt; settle wait [s] = "
          + ", ".join(f"{k} {v:.3f}" for k, v in waits.items()))


def test_s8_check_catches_a_leaked_thread():
    """S8 negative check: a thread leaked during a run still fails after the wait.

    The controller starts one extra thread that outlives run().  After the full
    2 s poll an identifier outside the baseline must still be there, which shows
    the wait does not hide a real leak.  The thread is released afterwards and
    no new identifier may then remain.
    """
    _eval(controller, 0.2).run()
    base = _ids()
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
        new, waited = _settled_new_ids(base)
        assert any(new), ("leak not detected", new)
        assert waited >= 2.0, waited
        seen = _describe(new)
    finally:
        release.set()
        for th in leaked:
            th.join(5.0)
    restored, _ = _settled_new_ids(base)
    assert not any(restored), ("after release", _describe(restored))
    print(f"S8 negative: leaked thread seen after {waited:.2f} s wait "
          f"({seen}); no new identifier left after release")


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


def test_e6a_quality_keys_and_rules():
    """Work order 34 E6-a: load1_start, rtf_windows and quality are reported.

    The verdict of a real run depends on the host, so it is only checked for
    consistency; the fixed rules are exercised on synthetic stats in E6-b1.
    """
    from vdsim_closedloop import _rtf_windows
    st = _eval(controller, 3.5).run().stats
    if sys.platform.startswith("linux"):
        assert isinstance(st["load1_start"], float), st["load1_start"]
    else:
        assert st["load1_start"] is None
    win = st["rtf_windows"]
    assert set(win) == {"p01", "p50", "p99", "min", "n"} and win["n"] >= 2, win
    q = st["quality"]
    assert q["ok"] == (not q["reasons"]), q
    assert set(q["reasons"]) <= {"rtf_band", "sim_max_lag", "sim_ticks_short"}, q
    assert q["warnings"] == (["control_overrun"] if st["n_overrun"] else []), q
    print(f"E6-a: load1_start {st['load1_start']}, rtf_windows {win}, quality {q}")

    half = _rtf_windows([0.0, 1.0, 2.0, 3.0], [0.0, 0.5, 1.0, 1.5])
    assert half["p50"] == 0.5 and half["n"] == 3, half
    assert _rtf_windows([0.0, 0.5, 1.5], [0.0, 0.5, 1.5]) is None


def test_e6b1_quality_rules_on_synthetic_stats():
    """Work order 34 E6-b1: each fixed rule alone, all three, the boundaries,
    and control_overrun as a warning that never touches ``ok``.

    ``|rtf - 1| = RTF_BAND`` cannot be hit exactly in floating point for 0.02
    (no double near 1 differs from 1 by the double 0.02), so the inclusive
    edge is checked twice: at the largest in-band double for the real band,
    and at exact equality under a temporarily widened, exactly representable
    band.
    """
    import numpy as np
    import vdsim_closedloop as vcl
    q = vcl._wall_quality
    good = {"rtf": 1.0, "sim_max_lag_s": 0.0, "stopped_early": False,
            "sim_ticks": 10, "sim_ticks_planned": 10, "n_overrun": 0}
    assert q(good, TC) == {"ok": True, "reasons": [], "warnings": []}
    cases = (
        ({"rtf": 1.0 + 1.5 * vcl.RTF_BAND}, ["rtf_band"]),
        ({"rtf": 1.0 - 1.5 * vcl.RTF_BAND}, ["rtf_band"]),
        ({"rtf": float("nan")}, ["rtf_band"]),
        ({"sim_max_lag_s": 1.5 * TC}, ["sim_max_lag"]),
        ({"sim_ticks": 9}, ["sim_ticks_short"]),
        ({"sim_ticks": 9, "stopped_early": True}, []),
        ({"rtf": 0.5, "sim_max_lag_s": 1.0, "sim_ticks": 3},
         ["rtf_band", "sim_max_lag", "sim_ticks_short"]),
        # Boundaries are inside: lag equal to Tc, ticks equal to the plan.
        ({"sim_max_lag_s": TC}, []),
        ({"sim_ticks": 10, "sim_ticks_planned": 10}, []),
    )
    for change, reasons in cases:
        got = q(dict(good, **change), TC)
        assert got["reasons"] == reasons and got["ok"] == (not reasons), (change, got)
        assert got["warnings"] == [], (change, got)
    for rtf in (1.0 + vcl.RTF_BAND, 1.0 - vcl.RTF_BAND):
        edge = float(np.nextafter(rtf, 1.0))          # largest in-band double
        assert abs(edge - 1.0) <= vcl.RTF_BAND
        assert q(dict(good, rtf=edge), TC)["ok"], edge
    band = vcl.RTF_BAND
    try:
        vcl.RTF_BAND = 0.25                           # 1 +/- 0.25 is exact
        for rtf in (1.25, 0.75):
            assert q(dict(good, rtf=rtf), TC)["ok"], rtf
    finally:
        vcl.RTF_BAND = band
    # A slow controller warns and leaves the environment verdict alone.
    for change in ({}, {"rtf": 0.5}):
        got = q(dict(good, n_overrun=3, **change), TC)
        assert got["warnings"] == ["control_overrun"], got
        assert got["reasons"] == ([] if not change else ["rtf_band"]), got
        assert got["ok"] == (not change), got
    print("E6-b1: each rule alone, all three, inclusive boundaries; "
          "control_overrun only in warnings, ok unchanged")


def test_e6b2_blocked_controller_warns():
    """Work order 34 E6-b2: a controller blocked for 3*Tc (the S7 run) leaves
    ``control_overrun`` in ``quality["warnings"]`` with ``n_overrun >= 1``.

    ``ok`` is not asserted: under load the sim-side rules may fail too. The
    3*Tc sleep makes the measured compute time exceed Tc by construction, so
    the overrun does not depend on host load and the check is not ``timing``.
    """
    ev, log = [None], []
    ev[0] = _eval(_blocking_controller(ev, time.sleep, log), 1.0)
    st = ev[0].run().stats
    assert st["n_overrun"] >= 1, st
    assert "control_overrun" in st["quality"]["warnings"], st["quality"]
    print(f"E6-b2: n_overrun = {st['n_overrun']}, quality = {st['quality']}")


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
    test_e6a_quality_keys_and_rules,
    test_e6b1_quality_rules_on_synthetic_stats,
    test_e6b2_blocked_controller_warns,
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
