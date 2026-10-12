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

A wall-clock mode (C) is a later stage of the same work order and is not in
this module yet.

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
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from vdsim_plant import VDSimPlant, _fx_to_cmdl1, _obs_from_output

#: Mode strings accepted by :class:`ClosedLoopEval`.
MODES = ("sync", "virtual_async")

#: Overrun policies accepted by :class:`ClosedLoopEval` in ``virtual_async``.
OVERRUN_POLICIES = ("skip", "queue")

#: Slack used when snapping a real time onto the substep grid. A latency that
#: is an exact multiple of ``substep_dt`` in decimal (5 ms on a 0.5 ms grid)
#: lands a few ulp above the integer after division; without the slack the
#: ceiling would add a whole substep that nobody asked for.
GRID_EPS = 1e-9

#: Scalar observation keys copied into the per-grid trace.
TRACE_STATE_KEYS = ("X", "Y", "psi", "vx", "vy", "r", "ax", "ay", "beta",
                    "roll", "pitch")


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
        The last row is the state at ``duration`` and carries no command.
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
    :param overrun: one of :data:`OVERRUN_POLICIES`, ``virtual_async`` only.
    :param state0: ``[X, Y, psi, vx, vy, r]`` passed to :meth:`VDSimPlant.reset`.
    :param u0: command held before the first computed command is applied.
    """

    def __init__(self, plant_kwargs, controller, control_period, mode="sync",
                 duration=10.0, latency=None, overrun="skip", state0=None,
                 u0=(0.0, 0.0)):
        if mode not in MODES:
            raise ValueError(f"mode={mode!r} is not one of {MODES}")
        plant_kwargs = dict(plant_kwargs or {})
        if "control_dt" in plant_kwargs:
            raise ValueError("pass the control period as control_period, "
                             "not plant_kwargs['control_dt']")
        if not callable(controller):
            raise TypeError("controller must be callable(obs, t)")
        if mode == "sync":
            if latency is not None:
                raise ValueError("latency applies to mode='virtual_async' only")
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

    # ------------------------------------------------------------------ run
    def run(self) -> EvalResult:
        """Reset the plant and run ``duration`` seconds in the chosen mode."""
        obs = self.plant.reset(self.state0)
        if self.mode == "sync":
            res = self._run_sync(obs)
        else:
            res = self._run_virtual_async(obs)
        res.meta = {
            "eval_mode": self.mode,
            "control_period_s": self.control_period,
            "substep_dt_s": self.substep_dt,
            "duration_s": self.n_steps * self.control_period,
            "latency_model": None if self.latency is None else self.latency.describe(),
            "overrun_policy": self.overrun,
            "level": self.plant.level,
        }
        return res

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
        for k in range(self.n_steps):
            t = k * Tc
            u, elapsed = self._call(obs, t)
            grid.put(k, t, obs, u)
            samples.put(k, t, t, 0.0, 0, u, applied=1, measured=elapsed)
            obs = self.plant.step(list(u))
        grid.put(self.n_steps, self.n_steps * Tc, obs, None)
        return EvalResult(trace=grid.arrays(), samples=samples.arrays(),
                          stats=samples.stats(Tc, skipped=0, unapplied=0))

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

        def start(k, obs_k, t_k, begin):
            """Compute sample k on the worker starting at substep ``begin``."""
            nonlocal worker_free
            u, elapsed = self._call(obs_k, t_k)
            c = float(draw(elapsed))
            finish = begin + delay_substeps(c, h)
            worker_free = finish
            late = 1 if (finish - k * n_sub) > n_sub else 0
            row = samples.put(k, t_k, finish * h, c, late, u, applied=0,
                              measured=elapsed)
            pending.append((finish, row, u))

        for i in range(total + 1):
            on_grid = i % n_sub == 0
            if on_grid:
                k = i // n_sub
                obs = _obs_from_output(sess.output())
                if i == total:
                    grid.put(k, k * Tc, obs, None)
                    break
                if self.overrun == "queue":
                    backlog.append((k, obs, k * Tc))
                elif i >= next_sample:
                    start(k, obs, k * Tc, i)
                    next_sample = worker_free
                else:
                    skipped += 1
            if self.overrun == "queue":
                # The loop visits every substep, so the worker is handed the
                # next queued sample exactly at the substep it becomes free. A
                # zero-delay computation frees it again in the same substep.
                while backlog and worker_free <= i:
                    k_q, obs_q, t_q = backlog.pop(0)
                    start(k_q, obs_q, t_q, i)
            # Apply everything that finished by this substep, oldest first.
            while pending and pending[0][0] <= i:
                _, row, u_now = pending.pop(0)
                samples.mark_applied(row)
                cmd_now = _fx_to_cmdl1(vp, *u_now)
            if on_grid:
                grid.put(k, k * Tc, obs, u_now)
            sess.set_input(cmd_now)
            sess.tick(h)

        unapplied = len(pending) + len(backlog)
        return EvalResult(trace=grid.arrays(), samples=samples.arrays(),
                          stats=samples.stats(Tc, skipped=skipped,
                                              unapplied=unapplied))


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

    def arrays(self) -> dict:
        return self._a


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
