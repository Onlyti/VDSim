#!/usr/bin/env python3
"""Closed-loop evaluation extensions of work order 34, part 1: E1 and E2.

* E1 -- ``ClosedLoopEval.stop()`` ends ``sync`` and ``virtual_async`` runs too.
  E1-a a stop inside the controller call at ``t = 3.0 s`` ends a ``sync`` run
  there; E1-b a ``virtual_async`` queue run drops the commands still waiting
  and counts them; E1-c a stop from another thread between two calls ends the
  run at the next grid boundary; E1-e the trace of a stopped run is a valid
  0.5 trace whose last row is ``t_stop``. (E1-d, bit identity of a run that
  never stops against the previous head, is the S1-S5 suite of
  ``test_closedloop_eval.py`` plus the digest comparison in the PR.)
* E2 -- ``obs["delta_rw"]`` is the realized road-wheel steer: E2-a equal to
  the command with no actuator, E2-b moving ``dead_time_s`` after a step with
  a steer dead time; ``delta_sw`` only when the vehicle file states the
  steering ratio; the opt-in ``steer_applied`` trace channel.
"""
import math
import sys
import tempfile
import threading
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim_plant  # noqa: E402
from vdsim_closedloop import ClosedLoopEval, LatencyModel, delay_substeps  # noqa: E402
from vdsim_plant import VDSimPlant  # noqa: E402
import vdsim_trace as vt  # noqa: E402

TC = 0.02            # control period [s]
H = 5e-4             # substep [s]
N_SUB = 40           # TC / H
V0 = 15.0            # initial speed [m/s]
STATE0 = [0.0, 0.0, 0.0, V0, 0.0, 0.0]
PLANT = dict(config="generic_sedan.yaml", level="L2", substep_dt=H)


def controller(obs, t):
    """Slow sine steer plus a speed hold: every sample is a new command."""
    return 0.03 * math.sin(2.0 * math.pi * 0.5 * t), 1500.0 * (V0 - obs["vx"])


def _stop_at(t_stop, holder):
    """Controller that calls ``stop()`` from inside the call at ``t_stop``."""
    def ctrl(obs, t):
        if abs(t - t_stop) < 1e-9:
            holder[0].stop()
        return controller(obs, t)
    return ctrl


class _TickHook:
    """Session proxy that runs ``hook()`` after the ``at``-th ``tick``."""

    def __init__(self, sess, at, hook):
        self._sess, self._at, self._hook, self.n = sess, at, hook, 0

    def tick(self, dt):
        self._sess.tick(dt)
        self.n += 1
        if self.n == self._at:
            self._hook()

    def __getattr__(self, name):
        return getattr(self._sess, name)


def test_e1a_sync_stop_inside_call():
    holder = [None]
    with tempfile.TemporaryDirectory() as tmp:
        ev = ClosedLoopEval(PLANT, _stop_at(3.0, holder), TC, mode="sync",
                            duration=5.0, state0=STATE0,
                            trace_path=Path(tmp) / "a.vdtrace",
                            reference_path=[[0.0, 0.0], [200.0, 0.0]])
        holder[0] = ev
        res = ev.run()
        st, smp = res.stats, res.samples
        assert st["stopped_early"] is True and st["t_stop"] == 3.0, st
        assert res.meta["duration_s"] == 5.0          # nominal, unchanged
        # The stopping sample was taken but its command never applied.
        assert smp["t_sample"][-1] == 3.0 and smp["applied"][-1] == 0
        assert int(smp["applied"].sum()) == 150 and st["n_unapplied"] == 1
        # Result rows: 150 commanded samples + the closing row at t_stop.
        assert res.trace["t"].size == 151 and res.trace["t"][-1] == 3.0
        assert np.isnan(res.trace["u_steer"][-1])
        assert st["e_y_n"] == 150                     # metrics over what ran
        with vt.TraceReader(res.meta["trace_path"]) as tr:
            t = tr.channel("t")
            assert t[-1] <= 3.0 + TC and t[-1] == st["t_stop"]
            assert tr.manifest["stopped_early"] is True
            assert tr.manifest["t_stop"] == 3.0
    print("E1-a sync: stop() inside the t=3.0 s call ends the run at t_stop=3.0, "
          "that command unapplied, 151 rows, trace ends at 3.0")


def test_e1b_virtual_async_queue_drops_waiting():
    holder = [None]
    lat = LatencyModel.fixed(1.5 * TC)
    with tempfile.TemporaryDirectory() as tmp:
        ev = ClosedLoopEval(PLANT, _stop_at(1.0, holder), TC,
                            mode="virtual_async", latency=lat, overrun="queue",
                            duration=3.0, state0=STATE0,
                            trace_path=Path(tmp) / "b.vdtrace")
        holder[0] = ev
        res = ev.run()
        st, smp = res.stats, res.samples
        assert st["stopped_early"] is True, st
        # The stopping sample is the last one started; behind a busy queue
        # its call (and so the stop) happens at its begin substep, not t_k.
        assert smp["t_sample"][-1] == 1.0 and smp["applied"][-1] == 0
        c = delay_substeps(1.5 * TC, H)
        begin = int(round(smp["t_apply"][-1] / H)) - c
        assert st["t_stop"] == begin * H > 1.0, (st["t_stop"], begin)
        # Dropped = started-but-unapplied rows + grid samples never started.
        seen = begin // N_SUB + 1                     # grid samples taken
        waiting = seen - smp["k"].size
        dropped_rows = int((smp["applied"] == 0).sum())
        assert waiting > 0 and dropped_rows > 1, (waiting, dropped_rows)
        assert st["n_unapplied"] == dropped_rows + waiting, st
        with vt.TraceReader(res.meta["trace_path"]) as tr:
            t_apply = tr.channel("t_apply")
            assert tr.channel("t")[-1] == st["t_stop"]
            done = set(smp["t_apply"][smp["applied"] == 1].tolist())
            gone = set(smp["t_apply"][smp["applied"] == 0].tolist())
            shown = set(t_apply[~np.isnan(t_apply)].tolist())
            assert shown <= done and not (shown & gone), (shown - done, shown & gone)
    print(f"E1-b virtual_async queue: stop at t_stop={st['t_stop']:.4f} s "
          f"(call behind the queue), {st['n_unapplied']} dropped = "
          f"{dropped_rows} computed + {waiting} queued, none in the trace")


def test_e1c_stop_from_another_thread():
    for mode, kw in (("sync", {}),
                     ("virtual_async", {"latency": LatencyModel.fixed(0.0)})):
        ev = ClosedLoopEval(PLANT, controller, TC, mode=mode, duration=3.0,
                            state0=STATE0, **kw)

        def from_thread():
            th = threading.Thread(target=ev.stop)
            th.start()
            th.join()
        # Mid-period of sample 50: substeps 2000..2039 belong to it.
        ev.plant._sess = _TickHook(ev.plant._sess, 50 * N_SUB + 10, from_thread)
        res = ev.run()
        st, smp = res.stats, res.samples
        assert st["stopped_early"] and st["t_stop"] == 51 * TC, (mode, st)
        assert smp["k"][-1] == 50 and smp["applied"][-1] == 1, mode
        assert st["n_unapplied"] == 0 and res.trace["t"][-1] == 51 * TC, mode
        assert ev.plant._sess.n == 51 * N_SUB, (mode, ev.plant._sess.n)
    print("E1-c: stop() from another thread mid-period ends sync and "
          "virtual_async at the next grid boundary (t_stop = 51*Tc), no sample there")


def test_e1e_stopped_trace_is_valid_0_5():
    holder = [None]
    with tempfile.TemporaryDirectory() as tmp:
        for mode, kw in (("sync", {}),
                         ("virtual_async", {"latency": LatencyModel.fixed(0.01)})):
            ev = ClosedLoopEval(PLANT, _stop_at(0.5, holder), TC, mode=mode,
                                duration=2.0, state0=STATE0,
                                trace_path=Path(tmp) / f"{mode}.vdtrace", **kw)
            holder[0] = ev
            res = ev.run()
            with vt.TraceReader(res.meta["trace_path"]) as tr:
                m = tr.manifest
                assert m["schema_version"] == "0.5", m["schema_version"]
                assert tr.channel("t")[-1] == res.stats["t_stop"] == 0.5
                assert m["stopped_early"] is True and m["t_stop"] == 0.5
                assert tr.n_steps == 26, (mode, tr.n_steps)   # 25 + closing row
        # A run that is not stopped says so.
        ev = ClosedLoopEval(PLANT, controller, TC, mode="sync", duration=0.2,
                            state0=STATE0, trace_path=Path(tmp) / "full.vdtrace")
        res = ev.run()
        with vt.TraceReader(res.meta["trace_path"]) as tr:
            assert tr.manifest["stopped_early"] is False
            assert tr.manifest["t_stop"] is None and tr.n_steps == 10
    print("E1-e: stopped sync/virtual_async traces read back as 0.5, last row "
          "= t_stop, manifest stopped_early/t_stop; a full run records False/None")


def test_e2a_delta_rw_equals_command_without_actuator():
    plant = VDSimPlant(control_dt=TC, **PLANT)
    obs = plant.reset(STATE0)
    assert obs["delta_rw"] == 0.0
    for k in range(100):
        u = controller(obs, k * TC)
        obs = plant.step(list(u))
        assert obs["delta_rw"] == u[0], (k, obs["delta_rw"], u[0])
    print("E2-a: actuator=None, delta_rw after each step == commanded u[0] "
          "bit for bit (100 steps)")


def test_e2b_dead_time_shows_in_delta_rw():
    dead = 0.15
    plant = VDSimPlant(control_dt=H, actuator={"steer": {"dead_time_s": dead}},
                       **PLANT)
    plant.reset(STATE0)
    t_cmd = 0.5
    n_cmd = int(round(t_cmd / H))
    t_move = None
    for i in range(int(round((t_cmd + 0.3) / H))):
        obs = plant.step([0.05 if i >= n_cmd else 0.0, 0.0])
        if obs["delta_rw"] != 0.0:
            # obs after substep i describes the steer applied over substep i.
            t_move = i * H
            break
    assert t_move is not None, "delta_rw never moved"
    lag = t_move - t_cmd
    assert abs(lag - dead) <= H + 1e-12, lag
    print(f"E2-b: steer dead_time 0.15 s, delta_rw moves {lag:.4f} s after the "
          f"step command (tolerance one substep {H} s)")


def test_e2_delta_sw_only_with_a_stated_ratio():
    plant = VDSimPlant(control_dt=TC, **PLANT)
    obs = plant.reset(STATE0)
    obs = plant.step([0.02, 0.0])
    assert plant.delta_sw_ratio == 15.0               # generic_sedan.yaml
    assert obs["delta_sw"] == obs["delta_rw"] * 15.0
    real = vdsim_plant._load_yaml_sidecar
    vdsim_plant._load_yaml_sidecar = lambda path, key: None   # key absent
    try:
        bare = VDSimPlant(control_dt=TC, **PLANT)
    finally:
        vdsim_plant._load_yaml_sidecar = real
    obs = bare.reset(STATE0)
    assert bare.delta_sw_ratio is None and "delta_sw" not in obs
    assert "delta_sw" not in bare.step([0.02, 0.0])
    ev = ClosedLoopEval(PLANT, controller, TC, duration=0.1, state0=STATE0)
    assert ev.run().meta["delta_sw_ratio"] == 15.0
    print("E2: delta_sw = delta_rw * 15.0 from the vehicle file; no key when "
          "the file states no ratio; ratio recorded in meta")


def test_e2_steer_applied_trace_channel():
    act = {"steer": {"dead_time_s": 0.06}}
    with tempfile.TemporaryDirectory() as tmp:
        ev = ClosedLoopEval(dict(PLANT, actuator=act), controller, TC,
                            mode="virtual_async", latency=LatencyModel.fixed(0.01),
                            duration=1.0, state0=STATE0,
                            trace_path=Path(tmp) / "s.vdtrace",
                            trace_channels=("steer_applied",))
        res = ev.run()
        with vt.TraceReader(res.meta["trace_path"]) as tr:
            sa = tr.channel("steer_applied")
            n = tr.n_steps
            assert np.array_equal(sa, res.trace["delta_rw"][:n])
            assert not np.array_equal(sa, tr.channel("u_steer"))   # 60 ms behind
        # Off by default, and a plant stepping on its own fills it too.
        plain = ClosedLoopEval(PLANT, controller, TC, duration=0.2, state0=STATE0,
                               trace_path=Path(tmp) / "p.vdtrace")
        with vt.TraceReader(plain.run().meta["trace_path"]) as tr:
            assert not tr.has("steer_applied")
        plant = VDSimPlant(control_dt=TC, **PLANT)
        obs = plant.reset(STATE0)
        plant.enable_trace(Path(tmp) / "q.vdtrace", extra_channels=("steer_applied",))
        for k in range(10):
            obs = plant.step(list(controller(obs, k * TC)))
        plant.finalize_trace()
        with vt.TraceReader(Path(tmp) / "q.vdtrace") as tr:
            assert tr.has("steer_applied") and tr.n_steps == 10
    try:
        ClosedLoopEval(PLANT, controller, TC, trace_channels=("t_apply",))
    except ValueError:
        pass
    else:
        raise AssertionError("a timing channel was accepted as trace_channels")
    print("E2: steer_applied trace channel is opt-in, equals delta_rw on the "
          "grid, differs from u_steer under a steer dead time")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"ALL {len(tests)} CHECKS PASSED")


if __name__ == "__main__":
    main()
