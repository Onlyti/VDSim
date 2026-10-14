"""Closed-loop evaluation modes for one controller callback.

The same ``controller(obs, t) -> (delta_rad, Fx_total_N)`` runs under a mode
chosen by a string, so the effect of controller computation delay can be
measured with everything else held equal:

* ``"sync"`` (A) -- today's :class:`vdsim_plant.VDSimPlant` lockstep. The
  command computed from the sample at ``t_k = k*Tc`` is applied at ``t_k`` and
  held for one period. Delay is not modelled.
* ``"virtual_async"`` (B) -- the same virtual clock, but the command computed
  from the sample at ``t_k`` is applied at
  ``t_apply = ceil((t_k + c_k) / substep_dt) * substep_dt``; the previous
  command is held until then (ZOH). ``c_k`` comes from a :class:`LatencyModel`.
  Deterministic latency models make the run bitwise reproducible.

* ``"wallclock"`` (C) -- real time on two threads. A C++
  ``vdsim.RealTimeRunner`` ticks the session every ``substep_dt`` of wall
  clock with the GIL released, while a Python thread wakes every ``Tc``,
  snapshots the session, calls the controller and latches its command. The
  delay is whatever the host really took, OS jitter included, so a run is
  reproducible only statistically.

Model of the controller in B: one worker computes one sample at a time.
Sample ``k`` starts computing at ``max(t_k, finish_{k-1})`` and finishes
``c_k`` later. When a computation runs past the next grid point the run is in
overrun, and the ``overrun`` policy decides what happens to the samples that
fall due meanwhile:

* ``"skip"`` -- they are not taken. The next sample is the first grid point at
  or after the finish of the computation in flight.
* ``"queue"`` -- every grid sample is taken at its own time and computed in
  order once the worker is free, so the delay accumulates while ``c_k > Tc``.

Times are kept as integer substep indices; ``c_k`` is the only real-valued
time and is rounded up to the substep grid once, when it is drawn.

In C the same two policies act on wall-clock slots ``w_k = w_0 + k*Tc``.
``skip`` resumes at the first slot at or after the end of the late
computation; ``queue`` takes every slot in order, back to back while behind.
A real-time loop cannot read the past, so a queued sample is the state at the
moment the controller thread gets to it, not the state at its slot.

Early end (:meth:`ClosedLoopEval.stop`, every mode, from the controller or
any other thread):

* ``sync`` / ``virtual_async`` -- a ``stop()`` made while a controller call
  runs (from the controller itself or from another thread) ends the run at
  the virtual time of that call: the command it returned is
  not applied and the plant is not advanced. That time is the sample time
  ``t_k``, except for a ``queue`` sample computed late behind a busy worker,
  whose call happens at the substep the worker frees up (the plant cannot go
  back to ``t_k``). A ``stop()`` made between calls ends the run at the next
  control-grid boundary, before that sample is taken. Commands still waiting
  for their ``t_apply`` are dropped and counted in ``n_unapplied``.
* ``wallclock`` -- the control thread stops at its next slot and the sim
  thread is joined; the run ends wherever the session clock got to.

Either way ``stats["stopped_early"]`` is ``True`` and ``stats["t_stop"]`` is the
simulated end time [s] (``None`` when the run went the full duration), the
result arrays hold only what ran, and a recorded trace is finalized with a
last row at ``t_stop`` (when that row falls on the decimation grid) and
``stopped_early`` / ``t_stop`` in its manifest.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field

import numpy as np

import vdsim
import vdsim_trace
from vdsim_plant import (SAMPLED_OPTIONAL_CHANNELS, VDSimPlant, _fx_to_cmdl1,
                         trace_sample)

#: Mode strings accepted by :class:`ClosedLoopEval`.
MODES = ("sync", "virtual_async", "wallclock")

#: Overrun policies accepted by :class:`ClosedLoopEval` in ``virtual_async``
#: and ``wallclock``.
OVERRUN_POLICIES = ("skip", "queue")

#: How often [s] the waiting main thread wakes to notice a finished control
#: thread or a KeyboardInterrupt in ``wallclock``.
JOIN_POLL_S = 0.05

#: Slack used when snapping a real time onto the substep grid. A latency that
#: is an exact multiple of ``substep_dt`` in decimal (5 ms on a 0.5 ms grid)
#: lands a few ulp above the integer after division; without the slack the
#: ceiling would add a whole substep that nobody asked for.
GRID_EPS = 1e-9

#: Scalar observation keys copied into the per-grid trace.
TRACE_STATE_KEYS = ("X", "Y", "psi", "vx", "vy", "r", "ax", "ay", "beta",
                    "roll", "pitch", "delta_rw")

#: Timing row of a trace row whose command no sample produced.
_NO_TIMING = (float("nan"), float("nan"), float("nan"), 0.0)


class LatencyModel:
    """Controller computation delay ``c_k`` [s] for the ``virtual_async`` mode.

    Build one with the factory methods; the constructor is internal.
    Distribution models own a seeded generator that is re-created for every
    run, so two runs with the same seed draw the same sequence.
    """

    KINDS = ("fixed", "uniform", "normal", "measured")

    def __init__(self, kind: str, **params):
        if kind not in self.KINDS:
            raise ValueError(f"latency kind {kind!r} is not one of {self.KINDS}")
        self.kind = kind
        self.params = dict(params)

    @classmethod
    def fixed(cls, c: float) -> "LatencyModel":
        """Every sample takes ``c`` seconds.

        :param c: delay [s], ``>= 0``.
        """
        c = float(c)
        if not (math.isfinite(c) and c >= 0.0):
            raise ValueError(f"fixed latency must be finite and >= 0, got {c}")
        return cls("fixed", c=c)

    @classmethod
    def uniform(cls, lo: float, hi: float, seed: int) -> "LatencyModel":
        """``c_k ~ U[lo, hi]`` from a generator seeded with ``seed``.

        :param lo: lower bound [s], ``>= 0``.
        :param hi: upper bound [s], ``>= lo``.
        :param seed: integer seed of the run's generator.
        """
        lo, hi = float(lo), float(hi)
        if not (math.isfinite(lo) and math.isfinite(hi) and 0.0 <= lo <= hi):
            raise ValueError(f"uniform latency needs 0 <= lo <= hi, got [{lo}, {hi}]")
        return cls("uniform", lo=lo, hi=hi, seed=int(seed))

    @classmethod
    def normal(cls, mu: float, sd: float, seed: int) -> "LatencyModel":
        """``c_k = max(0, N(mu, sd^2))`` from a generator seeded with ``seed``.

        Negative draws are clipped to zero because a computation cannot finish
        before it starts; the clip is part of the model and is recorded in
        :meth:`describe`.

        :param mu: mean [s], ``>= 0``.
        :param sd: standard deviation [s], ``>= 0``.
        :param seed: integer seed of the run's generator.
        """
        mu, sd = float(mu), float(sd)
        if not (math.isfinite(mu) and math.isfinite(sd) and mu >= 0.0 and sd >= 0.0):
            raise ValueError(f"normal latency needs mu >= 0 and sd >= 0, got {mu}, {sd}")
        return cls("normal", mu=mu, sd=sd, seed=int(seed), clip_min=0.0)

    @classmethod
    def measured(cls) -> "LatencyModel":
        """``c_k`` = wall time of the controller call (``time.perf_counter``).

        Not reproducible: the delay is whatever the host took this time.
        """
        return cls("measured")

    @property
    def deterministic(self) -> bool:
        """``True`` when the same model gives the same ``c_k`` sequence."""
        return self.kind != "measured"

    def describe(self) -> dict:
        """JSON-ready description, recorded with every result."""
        return {"kind": self.kind, **self.params}

    def sampler(self):
        """Return ``draw(measured_s) -> c_k`` with fresh generator state.

        :returns: a callable taking the measured controller time [s] (used
            only by ``measured``) and returning ``c_k`` [s].
        """
        p = self.params
        if self.kind == "fixed":
            c = p["c"]
            return lambda _measured: c
        if self.kind == "measured":
            return lambda measured: float(measured)
        rng = np.random.Generator(np.random.PCG64(p["seed"]))
        if self.kind == "uniform":
            lo, hi = p["lo"], p["hi"]
            return lambda _measured: float(rng.uniform(lo, hi))
        mu, sd = p["mu"], p["sd"]
        return lambda _measured: max(0.0, float(rng.normal(mu, sd)))


def delay_substeps(c: float, substep_dt: float) -> int:
    """Number of whole substeps a delay occupies on the grid.

    ``ceil(c / substep_dt)`` with :data:`GRID_EPS` slack, so a delay that is a
    decimal multiple of the substep is not pushed one substep late by
    round-off.

    :param c: delay [s], ``>= 0``.
    :param substep_dt: integration step [s].
    :returns: integer substep count, ``>= 0``.
    """
    return max(0, int(math.ceil(c / substep_dt - GRID_EPS)))


@dataclass
class EvalResult:
    """What one :meth:`ClosedLoopEval.run` produced.

    :ivar trace: arrays on the control grid ``t = k*Tc``, ``k = 0..N`` --
        ``t`` and the :data:`TRACE_STATE_KEYS` state, plus ``u_steer`` /
        ``u_fx``, the command *in force* over ``[t_k, t_k + substep_dt)``.
        The last row is the state at ``duration`` (``t_stop`` after an early
        end) and carries no command.
    :ivar samples: one entry per controller sample actually taken --
        ``k`` (grid index), ``t_sample``, ``t_apply``, ``c_k`` [s],
        ``overrun`` (0/1), ``u_steer``, ``u_fx`` (the command computed).
        Samples whose ``t_apply`` falls after ``duration`` keep their row with
        ``applied`` = 0.
    :ivar stats: counts and latency summary.
    :ivar meta: mode, periods and latency model, for the record.
    """

    trace: dict = field(default_factory=dict)
    samples: dict = field(default_factory=dict)
    stats: dict = field(default_factory=dict)
    meta: dict = field(default_factory=dict)


class ClosedLoopEval:
    """Run one controller against a :class:`VDSimPlant` under one mode.

    :param plant_kwargs: forwarded to :class:`VDSimPlant` (``config``,
        ``level``, ``substep_dt``, friction ...). ``control_dt`` is owned by
        ``control_period`` and must not be given here.
    :param controller: ``controller(obs, t) -> (delta_rad, Fx_total_N)``;
        ``obs`` is the observation dict of :class:`VDSimPlant` sampled at
        ``t``.
    :param control_period: ``Tc`` [s]; must be a whole multiple of
        ``substep_dt``.
    :param mode: one of :data:`MODES`.
    :param duration: simulated time [s]; rounded to whole periods.
    :param latency: :class:`LatencyModel`, ``virtual_async`` only.
    :param overrun: one of :data:`OVERRUN_POLICIES`, ``virtual_async`` and
        ``wallclock``.
    :param state0: ``[X, Y, psi, vx, vy, r]`` passed to :meth:`VDSimPlant.reset`.
    :param u0: command held before the first computed command is applied.
    :param cpu_affinity: ``wallclock`` only. ``None`` (default) leaves both
        threads on the process affinity, so ``taskset`` around the process
        decides. ``{"sim": 2, "ctrl": [3]}`` pins the C++ sim thread to one
        core and the Python control thread to a core set (Linux).
    :param watchdog: ``wallclock`` only. ``None`` (default) disables it;
        ``{"timeout_s": 0.1, "failsafe_u": (0.0, -2000.0)}`` latches
        ``failsafe_u`` whenever no command arrived for ``timeout_s`` of wall
        clock, counted in ``stats["failsafe_latches"]``.
    :param trace_path: record the run to this ``.vdtrace`` (schema 0.5).
        One row per control sample: the state at the sample and the command
        in force at that instant. ``sync`` writes exactly what
        :meth:`VDSimPlant.enable_trace` writes; ``virtual_async`` and
        ``wallclock`` add the :data:`vdsim_trace.EVAL_CHANNELS` timing of
        that command. Every mode records ``eval_mode``,
        ``control_period_s``, ``latency_model`` and ``overrun_policy`` in the
        manifest. A run that raises leaves no file.
    :param trace_channels: opt-in channels added to that trace, from
        :data:`vdsim_plant.SAMPLED_OPTIONAL_CHANNELS` (``"steer_applied"``,
        the realized road-wheel steer [rad]). Default: none.
    :param reference_path: ``(M, 2)`` polyline ``[X, Y]`` [m] in the ground
        frame. When given, ``stats`` carries the lateral tracking error of
        :func:`lateral_error` over the control samples (``e_y_rmse_m``,
        ``e_y_max_abs_m``, ``e_y_n``).
    """

    def __init__(self, plant_kwargs, controller, control_period, mode="sync",
                 duration=10.0, latency=None, overrun="skip", state0=None,
                 u0=(0.0, 0.0), cpu_affinity=None, watchdog=None,
                 trace_path=None, reference_path=None, trace_channels=()):
        if mode not in MODES:
            raise ValueError(f"mode={mode!r} is not one of {MODES}")
        plant_kwargs = dict(plant_kwargs or {})
        if "control_dt" in plant_kwargs:
            raise ValueError("pass the control period as control_period, "
                             "not plant_kwargs['control_dt']")
        if not callable(controller):
            raise TypeError("controller must be callable(obs, t)")
        if mode in ("sync", "wallclock"):
            if latency is not None:
                raise ValueError("latency applies to mode='virtual_async' only")
            if mode == "wallclock" and overrun not in OVERRUN_POLICIES:
                raise ValueError(f"overrun={overrun!r} is not one of {OVERRUN_POLICIES}")
        else:
            if latency is None:
                raise ValueError("mode='virtual_async' needs a LatencyModel")
            if not isinstance(latency, LatencyModel):
                raise TypeError("latency must be a LatencyModel")
            if overrun not in OVERRUN_POLICIES:
                raise ValueError(f"overrun={overrun!r} is not one of {OVERRUN_POLICIES}")
        duration = float(duration)
        if not (math.isfinite(duration) and duration > 0.0):
            raise ValueError("duration must be > 0")
        if len(u0) != 2 or not all(math.isfinite(float(v)) for v in u0):
            raise ValueError("u0 must be two finite numbers")
        if mode != "wallclock" and (cpu_affinity is not None or watchdog is not None):
            raise ValueError("cpu_affinity and watchdog apply to mode='wallclock' only")
        self.cpu_affinity = _check_affinity(cpu_affinity)
        self.watchdog = _check_watchdog(watchdog)
        self.trace_path = None if trace_path is None else str(trace_path)
        self.trace_channels = tuple(trace_channels)
        bad = [n for n in self.trace_channels if n not in SAMPLED_OPTIONAL_CHANNELS]
        if bad:
            raise ValueError(f"trace_channels {bad} not in {SAMPLED_OPTIONAL_CHANNELS}")
        self.reference_path = (None if reference_path is None
                               else _check_path(reference_path))

        self.plant_kwargs = plant_kwargs
        self.controller = controller
        self.control_period = float(control_period)
        self.mode = mode
        self.latency = latency
        self.overrun = overrun if mode != "sync" else None
        self.duration = duration
        self.state0 = state0
        self.u0 = (float(u0[0]), float(u0[1]))
        # The plant validates control_period against substep_dt.
        self.plant = VDSimPlant(control_dt=self.control_period, **plant_kwargs)
        self.substep_dt = self.plant.substep_dt
        self.n_sub = self.plant._n_sub
        self.n_steps = max(1, int(round(duration / self.control_period)))
        self._stop = threading.Event()
        self._runner = None

    # ------------------------------------------------------------------ run
    def run(self) -> EvalResult:
        """Reset the plant and run ``duration`` seconds in the chosen mode."""
        self._stop.clear()
        obs = self.plant.reset(self.state0)
        if self.trace_path is not None:
            timing = () if self.mode == "sync" else vdsim_trace.EVAL_CHANNELS
            self.plant.enable_trace(self.trace_path,
                                    extra_channels=self.trace_channels + timing,
                                    manifest_extra=self._eval_manifest())
        try:
            if self.mode == "sync":
                res = self._run_sync(obs)
            elif self.mode == "virtual_async":
                res = self._run_virtual_async(obs)
            else:
                res = self._run_wallclock(obs)
        except BaseException:
            self.plant._trace = None      # a failed run leaves no trace file
            raise
        res.meta = dict(self._eval_manifest(),
                        substep_dt_s=self.substep_dt,
                        duration_s=self.n_steps * self.control_period,
                        level=self.plant.level)
        if self.mode == "wallclock":
            res.meta["cpu_affinity"] = self.cpu_affinity
            res.meta["watchdog"] = self.watchdog
        if self.trace_path is not None:
            self.plant._trace.extra.update(
                stopped_early=res.stats["stopped_early"], t_stop=res.stats["t_stop"])
            res.meta["trace_path"] = str(self.plant.finalize_trace())
        if self.reference_path is not None:
            res.stats.update(_tracking_stats(res.trace, self.reference_path))
        return res

    def _eval_manifest(self) -> dict:
        """Evaluation settings recorded in the result and the trace manifest."""
        return {
            "eval_mode": self.mode,
            "control_period_s": self.control_period,
            "latency_model": None if self.latency is None else self.latency.describe(),
            "overrun_policy": self.overrun,
            # Ratio behind obs["delta_sw"]; None = the vehicle file states
            # none and delta_sw is not observed.
            "delta_sw_ratio": self.plant.delta_sw_ratio,
        }

    def _trace_row(self, out, t, u, timing):
        """Offer one control-sample row to the trace, if one is recorded.

        :param out: ``vdsim.SimOutput`` at the sample.
        :param t: sample time [s].
        :param u: command ``(delta, Fx)`` in force at the sample.
        :param timing: :data:`vdsim_trace.EVAL_CHANNELS` values of that
            command (see :meth:`_SampleRecorder.timing`).
        """
        plant = self.plant
        writer = plant._trace
        if writer is None:
            return
        if not writer.due:
            writer.skip()
            return
        sample = trace_sample(out, t, u[0], u[1], plant._channels, plant._validity)
        sample.update(zip(vdsim_trace.EVAL_CHANNELS, timing))
        writer.append(sample)

    def stop(self):
        """End the running run early, in any mode.

        Callable from any thread, the controller included; :meth:`run`
        returns the partial result with ``stats["stopped_early"] = True`` and
        ``stats["t_stop"]`` (see the module notes for where each mode ends).
        A call before :meth:`run` has no effect: every run starts cleared.
        """
        self._stop.set()

    def sim_ticks(self) -> int:
        """Substeps the wall-clock sim thread has taken in the current run.

        A lock-free counter on the C++ side, so it keeps increasing while the
        controller blocks or computes. ``0`` outside a ``wallclock`` run.
        """
        runner = self._runner
        return 0 if runner is None else int(runner.ticks())

    def _call(self, obs, t):
        """Call the controller, check its output and time the call."""
        t0 = time.perf_counter()
        u = self.controller(obs, t)
        elapsed = time.perf_counter() - t0
        if len(u) != 2:
            raise ValueError("controller must return (delta_rad, Fx_total_N)")
        delta, fx = float(u[0]), float(u[1])
        if not (math.isfinite(delta) and math.isfinite(fx)):
            raise ValueError(f"controller returned a non-finite command at t={t}")
        return (delta, fx), elapsed

    def _run_sync(self, obs) -> EvalResult:
        """Mode A: exactly :meth:`VDSimPlant.step` per period."""
        Tc = self.control_period
        grid = _GridRecorder(self.n_steps)
        samples = _SampleRecorder()
        stop = self._stop
        u_prev = self.u0        # in force for a closing row; nothing applied yet
        for k in range(self.n_steps):
            t = k * Tc
            if stop.is_set():                    # stop() from another thread
                return self._sync_stopped(grid, samples, k, t, obs, u_prev, 0)
            u, elapsed = self._call(obs, t)
            if stop.is_set():                    # stop() during this call
                samples.put(k, t, t, 0.0, 0, u, applied=0, measured=elapsed)
                return self._sync_stopped(grid, samples, k, t, obs, u_prev, 1)
            grid.put(k, t, obs, u)
            samples.put(k, t, t, 0.0, 0, u, applied=1, measured=elapsed)
            obs = self.plant.step(list(u))
            u_prev = u
        grid.put(self.n_steps, self.n_steps * Tc, obs, None)
        stats = samples.stats(Tc, skipped=0, unapplied=0)
        stats.update(stopped_early=False, t_stop=None)
        return EvalResult(trace=grid.arrays(), samples=samples.arrays(), stats=stats)

    def _sync_stopped(self, grid, samples, k, t, obs, u_prev, unapplied):
        """Close a ``sync`` run that stopped at grid sample ``k`` (time ``t``)."""
        grid.put(k, t, obs, None)
        self._trace_row(self.plant._sess.output(), t, u_prev, _NO_TIMING)
        stats = samples.stats(self.control_period, skipped=0, unapplied=unapplied)
        stats.update(stopped_early=True, t_stop=float(t))
        return EvalResult(trace=grid.arrays(k + 1), samples=samples.arrays(),
                          stats=stats)

    def _run_virtual_async(self, obs) -> EvalResult:
        """Mode B: substep loop with commands applied at their ``t_apply``."""
        Tc, h, n_sub = self.control_period, self.substep_dt, self.n_sub
        sess, vp = self.plant._sess, self.plant._vp
        draw = self.latency.sampler()
        total = self.n_steps * n_sub

        grid = _GridRecorder(self.n_steps)
        samples = _SampleRecorder()
        pending = []          # [(apply_index, sample_row, cmd)] in apply order
        backlog = []          # queue mode: (k, obs, t) waiting for the worker
        worker_free = 0       # substep index at which the worker is idle again
        next_sample = 0       # skip mode: earliest substep a sample may be taken
        skipped = 0
        u_now = self.u0
        cmd_now = _fx_to_cmdl1(vp, *u_now)
        in_force = None       # sample row of u_now; None while u0 holds
        stop = self._stop
        stop_at = None        # substep the run ended at after a stop()
        rows = 0              # grid rows recorded so far

        def start(k, obs_k, t_k, begin):
            """Compute sample k on the worker starting at substep ``begin``.

            :returns: ``True`` when ``stop()`` was called meanwhile; the
                sample keeps its row and stays unapplied.
            """
            nonlocal worker_free
            u, elapsed = self._call(obs_k, t_k)
            c = float(draw(elapsed))
            finish = begin + delay_substeps(c, h)
            worker_free = finish
            late = 1 if (finish - k * n_sub) > n_sub else 0
            row = samples.put(k, t_k, finish * h, c, late, u, applied=0,
                              measured=elapsed)
            pending.append((finish, row, u))
            return stop.is_set()

        for i in range(total + 1):
            on_grid = i % n_sub == 0
            if on_grid:
                k = i // n_sub
                out = sess.output()
                obs = self.plant._obs_from(out)
                if i == total:
                    grid.put(k, k * Tc, obs, None)
                    rows += 1
                    break
                if stop.is_set():        # stop() from another thread
                    stop_at = i
                    break
                if self.overrun == "queue":
                    backlog.append((k, obs, k * Tc))
                elif i >= next_sample:
                    if start(k, obs, k * Tc, i):
                        stop_at = i
                        break
                    next_sample = worker_free
                else:
                    skipped += 1
            if self.overrun == "queue":
                # The loop visits every substep, so the worker is handed the
                # next queued sample exactly at the substep it becomes free. A
                # zero-delay computation frees it again in the same substep.
                while backlog and worker_free <= i:
                    k_q, obs_q, t_q = backlog.pop(0)
                    if start(k_q, obs_q, t_q, i):
                        stop_at = i
                        break
                if stop_at is not None:
                    break
            # Apply everything that finished by this substep, oldest first.
            while pending and pending[0][0] <= i:
                _, row, u_now = pending.pop(0)
                samples.mark_applied(row)
                cmd_now = _fx_to_cmdl1(vp, *u_now)
                in_force = row
            if on_grid:
                grid.put(k, k * Tc, obs, u_now)
                rows += 1
                self._trace_row(out, k * Tc, u_now, samples.timing(in_force))
            sess.set_input(cmd_now)
            sess.tick(h)

        # Commands still waiting for their t_apply are dropped at a stop.
        unapplied = len(pending) + len(backlog)
        stats = samples.stats(Tc, skipped=skipped, unapplied=unapplied)
        if stop_at is None:
            stats.update(stopped_early=False, t_stop=None)
            return EvalResult(trace=grid.arrays(), samples=samples.arrays(),
                              stats=stats)
        # The plant has not been ticked at stop_at: the closing row is the
        # state there, with the command that was in force on arrival.
        t_stop = (stop_at // n_sub) * Tc if stop_at % n_sub == 0 else stop_at * h
        out = sess.output()
        grid.put(rows, t_stop, self.plant._obs_from(out), None)
        self._trace_row(out, t_stop, u_now, samples.timing(in_force))
        stats.update(stopped_early=True, t_stop=float(t_stop))
        return EvalResult(trace=grid.arrays(rows + 1), samples=samples.arrays(),
                          stats=stats)


    def _run_wallclock(self, obs) -> EvalResult:
        """Mode C: C++ sim thread paced to the wall clock + Python control thread."""
        Tc, h, n_sub = self.control_period, self.substep_dt, self.n_sub
        sess, vp = self.plant._sess, self.plant._vp
        total = self.n_steps * n_sub

        cfg = vdsim.RealTimeRunnerConfig()
        cfg.dt = h
        cfg.max_ticks = total
        cfg.cpu = -1 if self.cpu_affinity is None else self.cpu_affinity["sim"]
        if self.watchdog is None:
            cfg.cmd_timeout_s = 0.0
        else:
            cfg.cmd_timeout_s = self.watchdog["timeout_s"]
            cfg.failsafe = _fx_to_cmdl1(vp, *self.watchdog["failsafe_u"])
        sess.set_input(_fx_to_cmdl1(vp, *self.u0))
        runner = vdsim.RealTimeRunner(sess, cfg)

        rows = _RowRecorder()
        samples = _SampleRecorder()
        wall = {"slot": [], "sample": [], "apply": []}
        box = {"exc": None, "skipped": 0, "ctrl_affinity": False,
               "last": (self.u0, None)}   # command in force, its sample row
        # Trace rows are kept raw and written after the run so building the
        # trace sample never adds to the control thread's latency.
        trace_rows = [] if self.plant._trace is not None else None
        stop = self._stop
        overrun_policy = self.overrun
        ctrl_cpus = None if self.cpu_affinity is None else self.cpu_affinity["ctrl"]
        clock = time.perf_counter

        def control(w0):
            """Control thread: wake on each slot, sample, compute, latch."""
            try:
                if ctrl_cpus is not None and hasattr(os, "sched_setaffinity"):
                    os.sched_setaffinity(0, ctrl_cpus)   # 0 = this thread on Linux
                    box["ctrl_affinity"] = True
                k = 0
                u_now = self.u0
                in_force = None
                while not stop.is_set() and not runner.finished():
                    slot = w0 + k * Tc
                    wait = slot - clock()
                    if wait > 0.0:
                        time.sleep(wait)
                    if stop.is_set() or runner.finished():
                        break
                    w_sample = clock()
                    out = sess.output()
                    obs_k = self.plant._obs_from(out)
                    t_s = float(out.sim_time)
                    u, elapsed = self._call(obs_k, t_s)
                    sess.set_input(_fx_to_cmdl1(vp, *u))
                    w_apply = clock()
                    t_a = float(sess.sim_time())
                    late = 1 if (w_apply - slot) > Tc else 0
                    rows.put(t_s, obs_k, u_now)
                    if trace_rows is not None:
                        trace_rows.append((out, t_s, u_now, in_force))
                    in_force = samples.put(k, t_s, t_a, elapsed, late, u,
                                           applied=1, measured=elapsed)
                    wall["slot"].append(slot - w0)
                    wall["sample"].append(w_sample - w0)
                    wall["apply"].append(w_apply - w0)
                    u_now = u
                    box["last"] = (u_now, in_force)
                    nxt = k + 1
                    if overrun_policy == "skip":
                        first_free = int(math.ceil((clock() - w0) / Tc - GRID_EPS))
                        if first_free > nxt:
                            box["skipped"] += first_free - nxt
                            nxt = first_free
                    k = nxt
            except BaseException as exc:   # re-raised on the caller's thread
                box["exc"] = exc
                stop.set()

        self._runner = runner
        ctrl = None
        try:
            runner.start()
            ctrl = threading.Thread(target=control, args=(clock(),),
                                    name="vdsim-closedloop-ctrl")
            ctrl.start()
            while ctrl.is_alive():
                ctrl.join(JOIN_POLL_S)
        finally:
            # Normal end, controller error, stop() or KeyboardInterrupt in the
            # caller: every path stops the control thread first, then joins
            # the C++ thread, so nothing outlives run().
            stop.set()
            if ctrl is not None:
                ctrl.join()
            runner.stop()
            self._runner = None
        if box["exc"] is not None:
            raise box["exc"]
        rs = runner.stats()
        if rs.error:
            raise RuntimeError(f"sim thread failed: {rs.error}")

        for out_k, t_k, u_k, row_k in trace_rows or ():
            self._trace_row(out_k, t_k, u_k, samples.timing(row_k))
        out = sess.output()
        t_end = float(out.sim_time)
        rows.put(t_end, self.plant._obs_from(out), None)
        stopped = bool(rs.ticks < total)
        if stopped:
            u_last, row_last = box["last"]
            self._trace_row(out, t_end, u_last, samples.timing(row_last))
        stats = samples.stats(Tc, skipped=box["skipped"], unapplied=0)
        stats.update(_wall_stats(wall))
        stats.update({
            "rtf": (rs.sim_elapsed_s / rs.wall_elapsed_s
                    if rs.wall_elapsed_s > 0.0 else float("nan")),
            "sim_ticks": int(rs.ticks),
            "sim_ticks_planned": int(total),
            "sim_late_ticks": int(rs.late_ticks),
            "sim_max_lag_s": float(rs.max_lag_s),
            "sim_elapsed_s": float(rs.sim_elapsed_s),
            "wall_elapsed_s": float(rs.wall_elapsed_s),
            "failsafe_latches": int(rs.failsafe_latches),
            "sim_affinity_applied": bool(rs.affinity_applied),
            "ctrl_affinity_applied": bool(box["ctrl_affinity"]),
            "stopped_early": stopped,
            "t_stop": t_end if stopped else None,
        })
        smp = samples.arrays()
        for key, vals in wall.items():
            smp[f"wall_{key}"] = np.asarray(vals, dtype=np.float64)
        return EvalResult(trace=rows.arrays(), samples=smp, stats=stats)


def _check_affinity(spec):
    """Validate ``cpu_affinity``; returns ``None`` or ``{"sim": int, "ctrl": [int]}``."""
    if spec is None:
        return None
    if not isinstance(spec, dict) or set(spec) != {"sim", "ctrl"}:
        raise ValueError('cpu_affinity must be {"sim": core, "ctrl": [cores]}')
    sim = spec["sim"]
    ctrl = spec["ctrl"]
    ctrl = [ctrl] if isinstance(ctrl, int) else list(ctrl)
    if not (isinstance(sim, int) and sim >= 0 and ctrl
            and all(isinstance(c, int) and c >= 0 for c in ctrl)):
        raise ValueError("cpu_affinity cores must be non-negative integers")
    return {"sim": int(sim), "ctrl": [int(c) for c in ctrl]}


def _check_watchdog(spec):
    """Validate ``watchdog``; returns ``None`` or ``{"timeout_s", "failsafe_u"}``."""
    if spec is None:
        return None
    if not isinstance(spec, dict) or not set(spec) <= {"timeout_s", "failsafe_u"}:
        raise ValueError('watchdog must be {"timeout_s": s, "failsafe_u": (delta, Fx)}')
    timeout = float(spec.get("timeout_s", 0.0))
    fs = tuple(float(v) for v in spec.get("failsafe_u", (0.0, 0.0)))
    if not (math.isfinite(timeout) and timeout > 0.0):
        raise ValueError("watchdog timeout_s must be > 0")
    if len(fs) != 2 or not all(math.isfinite(v) for v in fs):
        raise ValueError("watchdog failsafe_u must be two finite numbers")
    return {"timeout_s": timeout, "failsafe_u": list(fs)}


def _check_path(path):
    """Validate a reference polyline; returns an ``(M, 2)`` float array."""
    xy = np.asarray(path, dtype=np.float64)
    if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] < 2:
        raise ValueError("reference_path must be an (M >= 2, 2) array of [X, Y]")
    if not np.all(np.isfinite(xy)):
        raise ValueError("reference_path must be finite")
    if np.any(np.hypot(*np.diff(xy, axis=0).T) <= 0.0):
        raise ValueError("reference_path has repeated consecutive points")
    return xy


def lateral_error(x, y, path_xy):
    """Signed lateral offset [m] of points from a polyline path.

    For every point the closest point on the polyline (segments, not just
    vertices) is found; the offset is the distance to it, positive when the
    point lies to the left of the path direction. Points and path share the
    ground frame of the trace.

    :param x: point X [m], array-like of shape ``(N,)``.
    :param y: point Y [m], same shape.
    :param path_xy: ``(M, 2)`` polyline ``[X, Y]`` [m], ``M >= 2``.
    :returns: ``(N,)`` array of ``e_y`` [m].
    """
    xy = _check_path(path_xy)
    p = np.stack([np.asarray(x, dtype=np.float64),
                  np.asarray(y, dtype=np.float64)], axis=-1)
    a = xy[:-1]                                   # (M-1, 2) segment starts
    d = np.diff(xy, axis=0)                       # (M-1, 2) segment vectors
    rel = p[:, None, :] - a[None, :, :]           # (N, M-1, 2)
    s = np.clip((rel * d).sum(-1) / (d * d).sum(-1), 0.0, 1.0)
    gap = rel - s[..., None] * d[None, :, :]
    dist2 = (gap * gap).sum(-1)
    j = np.argmin(dist2, axis=1)
    rows = np.arange(p.shape[0])
    g = gap[rows, j]
    side = np.sign(d[j, 0] * g[:, 1] - d[j, 1] * g[:, 0])
    return side * np.sqrt(dist2[rows, j])


def _tracking_stats(trace, path_xy) -> dict:
    """Lateral tracking error over the control samples of a result trace.

    The rows that carry a command are the control samples; the closing row
    (state at ``duration``, no command) is left out.
    """
    sampled = ~np.isnan(np.asarray(trace["u_steer"]))
    e = lateral_error(trace["X"][sampled], trace["Y"][sampled], path_xy)
    return {
        "e_y_rmse_m": float(np.sqrt(np.mean(e * e))) if e.size else float("nan"),
        "e_y_max_abs_m": float(np.max(np.abs(e))) if e.size else float("nan"),
        "e_y_n": int(e.size),
    }


def _pct(x, q):
    """Percentile of a possibly empty array (NaN when empty)."""
    return float(np.percentile(x, q)) if x.size else float("nan")


def _wall_stats(wall: dict) -> dict:
    """Wall-clock period and wake-jitter summary [s] of a ``wallclock`` run.

    ``period`` = time between consecutive wakes; ``wake_jitter`` = wake time
    minus its scheduled slot (>= 0 up to the sleep resolution).
    """
    sample = np.asarray(wall["sample"], dtype=np.float64)
    slot = np.asarray(wall["slot"], dtype=np.float64)
    period = np.diff(sample)
    jitter = sample - slot
    return {
        "period_p50_s": _pct(period, 50),
        "period_p99_s": _pct(period, 99),
        "period_max_s": float(period.max()) if period.size else float("nan"),
        "wake_jitter_p50_s": _pct(jitter, 50),
        "wake_jitter_p99_s": _pct(jitter, 99),
        "wake_jitter_max_s": float(jitter.max()) if jitter.size else float("nan"),
    }


class _RowRecorder:
    """State and in-force command at each wall-clock sample (variable length)."""

    def __init__(self):
        self._rows = {"t": [], "u_steer": [], "u_fx": []}
        for key in TRACE_STATE_KEYS:
            self._rows[key] = []

    def put(self, t, obs, u):
        """Append one row; ``u=None`` records the command as NaN."""
        r = self._rows
        r["t"].append(t)
        for key in TRACE_STATE_KEYS:
            r[key].append(obs[key])
        r["u_steer"].append(float("nan") if u is None else u[0])
        r["u_fx"].append(float("nan") if u is None else u[1])

    def arrays(self) -> dict:
        return {key: np.asarray(v, dtype=np.float64) for key, v in self._rows.items()}


class _GridRecorder:
    """State and in-force command on the control grid ``t_k = k*Tc``."""

    def __init__(self, n_steps: int):
        n = n_steps + 1
        self._a = {"t": np.zeros(n)}
        for key in TRACE_STATE_KEYS:
            self._a[key] = np.zeros(n)
        self._a["u_steer"] = np.full(n, np.nan)
        self._a["u_fx"] = np.full(n, np.nan)

    def put(self, k, t, obs, u):
        """Record grid row ``k``; ``u=None`` leaves the command NaN."""
        self._a["t"][k] = t
        for key in TRACE_STATE_KEYS:
            self._a[key][k] = obs[key]
        if u is not None:
            self._a["u_steer"][k], self._a["u_fx"][k] = u

    def arrays(self, n_rows=None) -> dict:
        """The grid arrays, cut to the first ``n_rows`` rows when given."""
        if n_rows is None:
            return self._a
        return {key: v[:n_rows] for key, v in self._a.items()}


class _SampleRecorder:
    """One row per controller sample: timing, overrun flag and command."""

    _KEYS = ("k", "t_sample", "t_apply", "c_k", "overrun", "u_steer", "u_fx",
             "applied", "t_compute_s")

    def __init__(self):
        self._rows = {key: [] for key in self._KEYS}

    def put(self, k, t_sample, t_apply, c, overrun, u, applied, measured) -> int:
        """Append a sample row and return its index."""
        r = self._rows
        r["k"].append(k)
        r["t_sample"].append(t_sample)
        r["t_apply"].append(t_apply)
        r["c_k"].append(c)
        r["overrun"].append(overrun)
        r["u_steer"].append(u[0])
        r["u_fx"].append(u[1])
        r["applied"].append(applied)
        r["t_compute_s"].append(measured)
        return len(r["k"]) - 1

    def mark_applied(self, row: int):
        self._rows["applied"][row] = 1

    def timing(self, row):
        """Trace timing ``(t_sample, t_apply, c_k, overrun)`` of a sample row.

        ``row=None`` is the initial command, which no sample produced: NaN
        times and no overrun.
        """
        if row is None:
            return (float("nan"), float("nan"), float("nan"), 0.0)
        r = self._rows
        return (float(r["t_sample"][row]), float(r["t_apply"][row]),
                float(r["c_k"][row]), float(r["overrun"][row]))

    def arrays(self) -> dict:
        out = {}
        for key, vals in self._rows.items():
            dtype = np.int64 if key in ("k", "overrun", "applied") else np.float64
            out[key] = np.asarray(vals, dtype=dtype)
        return out

    def stats(self, Tc: float, skipped: int, unapplied: int) -> dict:
        """Counts plus sample-to-apply delay percentiles [s]."""
        a = self.arrays()
        delay = a["t_apply"] - a["t_sample"]
        def pct(x, q):
            return float(np.percentile(x, q)) if x.size else float("nan")
        return {
            "n_samples": int(a["k"].size),
            "n_overrun": int(a["overrun"].sum()),
            "n_skipped": int(skipped),
            "n_unapplied": int(unapplied),
            "delay_p50_s": pct(delay, 50),
            "delay_p99_s": pct(delay, 99),
            "delay_max_s": float(delay.max()) if delay.size else float("nan"),
            "compute_p50_s": pct(a["t_compute_s"], 50),
            "compute_p99_s": pct(a["t_compute_s"], 99),
            "compute_max_s": (float(a["t_compute_s"].max())
                              if a["t_compute_s"].size else float("nan")),
        }
