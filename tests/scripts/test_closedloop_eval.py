#!/usr/bin/env python3
"""Closed-loop evaluation modes A (sync) and B (virtual_async): S1-S5.

Acceptance criteria of work order 31 (section 5):

* S1 -- mode A equals a hand-written ``VDSimPlant.step`` loop, bit for bit.
* S2 -- mode B with zero fixed latency equals mode A, bit for bit.
* S3 -- mode B with a fixed latency ``d`` equals a hand-written substep
  lockstep that holds each command for ``d`` before applying it. The same
  comparison against a reference shifted by one substep must fail, which
  shows the comparison can tell the two apart.
* S4 -- a seeded latency distribution repeats bit for bit; another seed
  differs.
* S5 -- with ``c_k = 1.5*Tc`` the skip and queue policies apply commands at
  the times of their expected tables.

Also: the ``.vdtrace`` of each mode (sync identical to the plant's own trace,
timing channels in virtual_async) and the lateral tracking error used by the
S10 demo.
"""
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_closedloop import (  # noqa: E402
    ClosedLoopEval, LatencyModel, TRACE_STATE_KEYS, delay_substeps,
    lateral_error)
from vdsim_plant import VDSimPlant  # noqa: E402
import vdsim_trace as vt  # noqa: E402

TC = 0.02            # control period [s]
H = 5e-4             # substep [s]
N_SUB = 40           # TC / H
V0 = 16.7            # initial speed [m/s]
STATE0 = [0.0, 0.0, 0.0, V0, 0.0, 0.0]
PLANT = dict(level="L2", substep_dt=H)
DURATION = 3.0       # [s]


def y_ref(x):
    """Smooth 3.5 m lane change between x = 15 m and x = 45 m."""
    if x <= 15.0:
        return 0.0
    if x >= 45.0:
        return 3.5
    return 1.75 * (1.0 - math.cos(math.pi * (x - 15.0) / 30.0))


def controller(obs, t):
    """Lateral P on offset and heading, longitudinal P on speed."""
    x, y = obs["X"], obs["Y"]
    slope = (y_ref(x + 0.5) - y_ref(x - 0.5)) / 1.0
    e_y = y_ref(x) - y
    e_psi = math.atan(slope) - obs["psi"]
    delta = max(-0.3, min(0.3, 0.08 * e_y + 0.9 * e_psi))
    fx = 2000.0 * (V0 - obs["vx"])
    return delta, fx


def _max_abs_diff(a, b, keys=TRACE_STATE_KEYS):
    """Largest |a-b| over the named grid arrays (NaN-aware: NaN==NaN)."""
    worst = 0.0
    for key in keys:
        x, y = np.asarray(a[key]), np.asarray(b[key])
        assert x.shape == y.shape, (key, x.shape, y.shape)
        both_nan = np.isnan(x) & np.isnan(y)
        d = np.where(both_nan, 0.0, np.abs(x - y))
        worst = max(worst, float(np.nanmax(d)) if d.size else 0.0)
        if np.any(np.isnan(x) != np.isnan(y)):
            return float("inf")
    return worst


def _run(mode, latency=None, overrun="skip", duration=DURATION, ctrl=controller):
    ev = ClosedLoopEval(PLANT, ctrl, TC, mode=mode, duration=duration,
                        latency=latency, overrun=overrun, state0=STATE0)
    return ev.run()


def _hand_sync(duration=DURATION):
    """Reference for S1: the documented VDSimPlant loop, written out."""
    plant = VDSimPlant(control_dt=TC, **PLANT)
    obs = plant.reset(STATE0)
    n = int(round(duration / TC))
    out = {key: [obs[key]] for key in TRACE_STATE_KEYS}
    for k in range(n):
        obs = plant.step(list(controller(obs, k * TC)))
        for key in TRACE_STATE_KEYS:
            out[key].append(obs[key])
    return {key: np.asarray(v) for key, v in out.items()}


def _hand_delayed(D, duration=DURATION):
    """Reference for S3: one-substep plant, each command applied D substeps late.

    Written without the evaluator's scheduler: a dict keyed by the substep at
    which the command takes over.
    """
    plant = VDSimPlant(control_dt=H, **PLANT)
    obs = plant.reset(STATE0)
    total = int(round(duration / TC)) * N_SUB
    out = {key: [] for key in TRACE_STATE_KEYS}
    u_now = (0.0, 0.0)
    due = {}
    for i in range(total):
        if i % N_SUB == 0:
            for key in TRACE_STATE_KEYS:
                out[key].append(obs[key])
            due[i + D] = controller(obs, (i // N_SUB) * TC)
        if i in due:
            u_now = due.pop(i)
        obs = plant.step(list(u_now))
    for key in TRACE_STATE_KEYS:
        out[key].append(obs[key])
    return {key: np.asarray(v) for key, v in out.items()}


def test_s1_sync_matches_plant_loop():
    a = _run("sync")
    ref = _hand_sync()
    d = _max_abs_diff(a.trace, ref)
    assert d == 0.0, f"S1: sync differs from the VDSimPlant loop by {d}"
    # The run actually manoeuvres, so a zero diff is not two parked cars.
    assert abs(a.trace["Y"][-1]) > 1.0, a.trace["Y"][-1]
    print(f"S1 ok: sync == VDSimPlant loop (max|diff| = 0, Y_end = "
          f"{a.trace['Y'][-1]:.3f} m)")


def test_s2_async_zero_latency_matches_sync():
    a = _run("sync")
    for policy in ("skip", "queue"):
        b = _run("virtual_async", LatencyModel.fixed(0.0), overrun=policy)
        keys = TRACE_STATE_KEYS + ("u_steer", "u_fx")
        d = _max_abs_diff(a.trace, b.trace, keys)
        assert d == 0.0, f"S2 ({policy}): fixed(0) differs from sync by {d}"
        assert b.stats["n_overrun"] == 0 and b.stats["n_skipped"] == 0
        assert np.array_equal(b.samples["t_apply"], b.samples["t_sample"])
    print("S2 ok: virtual_async fixed(0) == sync (skip and queue)")


def test_s3_fixed_delay_matches_hand_delayed_lockstep():
    # 5 ms is 10 substeps exactly; 4.2 ms rounds up to 9 substeps.
    for d_s, D in ((0.005, 10), (0.0042, 9), (0.02, 40)):
        assert delay_substeps(d_s, H) == D
        b = _run("virtual_async", LatencyModel.fixed(d_s))
        ref = _hand_delayed(D)
        diff = _max_abs_diff(b.trace, ref)
        assert diff == 0.0, f"S3 d={d_s}: differs from the reference by {diff}"
        # Negative check: the reference one substep late must not match.
        off = _hand_delayed(D + 1)
        diff_off = _max_abs_diff(b.trace, off)
        assert diff_off > 0.0, f"S3 d={d_s}: a reference shifted by one " \
            "substep also matches, so the comparison cannot see the delay"
        assert b.stats["n_overrun"] == 0
        print(f"S3 ok: d = {d_s*1e3:.1f} ms == hand lockstep with D = {D} "
              f"(shifted reference max|diff| = {diff_off:.3e})")


def test_s4_seeded_distribution_repeats():
    keys = TRACE_STATE_KEYS + ("u_steer", "u_fx")
    for make in (lambda s: LatencyModel.uniform(0.0, 0.015, seed=s),
                 lambda s: LatencyModel.normal(0.006, 0.003, seed=s)):
        runs = [_run("virtual_async", make(7)) for _ in range(3)]
        for r in runs[1:]:
            assert _max_abs_diff(runs[0].trace, r.trace, keys) == 0.0
            assert np.array_equal(runs[0].samples["c_k"], r.samples["c_k"])
        other = _run("virtual_async", make(8))
        assert not np.array_equal(runs[0].samples["c_k"], other.samples["c_k"])
        d = _max_abs_diff(runs[0].trace, other.trace, keys)
        assert d > 0.0, "S4: a different seed gave the same trajectory"
        kind = runs[0].meta["latency_model"]["kind"]
        print(f"S4 ok: {kind} seed 7 x3 identical, seed 8 differs "
              f"(max|diff| = {d:.3e})")


def _applied_command_at(samples, t):
    """Latest computed command whose t_apply <= t (None before the first)."""
    best = None
    for i in np.argsort(samples["t_apply"], kind="stable"):
        if samples["applied"][i] and samples["t_apply"][i] <= t + 1e-12:
            best = (samples["u_steer"][i], samples["u_fx"][i])
    return best


def test_s5_overrun_tables():
    c = 1.5 * TC                  # 30 ms = 60 substeps
    dur = 0.2                     # 10 periods, 400 substeps
    D = 60
    # skip: one sample in flight; the next is the first grid point at or
    # after its finish. Sample j is taken at k = 2j, applied 60 substeps on.
    b = _run("virtual_async", LatencyModel.fixed(c), "skip", duration=dur)
    exp_k = [0, 2, 4, 6, 8]
    exp_apply = [(k * N_SUB + D) for k in exp_k]
    assert b.samples["k"].tolist() == exp_k, b.samples["k"]
    got = np.rint(b.samples["t_apply"] / H).astype(int).tolist()
    assert got == exp_apply, got
    assert b.samples["overrun"].tolist() == [1] * 5
    assert b.samples["applied"].tolist() == [1] * 5
    assert b.stats["n_skipped"] == 5 and b.stats["n_overrun"] == 5
    for k in range(10):
        u = _applied_command_at(b.samples, k * TC)
        held = (b.trace["u_steer"][k], b.trace["u_fx"][k])
        assert held == (u if u is not None else (0.0, 0.0)), (k, held, u)
    print(f"S5 skip ok: k = {exp_k}, t_apply = "
          f"{[round(a * H, 4) for a in exp_apply]} s")

    # queue: every grid sample is taken; the worker runs them back to back,
    # so sample k finishes at (k+1)*60 substeps. Samples 7..9 never reach the
    # worker inside 400 substeps and sample 6 finishes after the end.
    q = _run("virtual_async", LatencyModel.fixed(c), "queue", duration=dur)
    exp_k = list(range(7))
    exp_apply = [(k + 1) * D for k in exp_k]
    assert q.samples["k"].tolist() == exp_k, q.samples["k"]
    got = np.rint(q.samples["t_apply"] / H).astype(int).tolist()
    assert got == exp_apply, got
    assert q.samples["t_sample"].tolist() == [k * TC for k in exp_k]
    assert q.samples["overrun"].tolist() == [1] * 7
    assert q.samples["applied"].tolist() == [1] * 6 + [0]
    assert q.stats["n_unapplied"] == 4 and q.stats["n_skipped"] == 0
    for k in range(10):
        u = _applied_command_at(q.samples, k * TC)
        held = (q.trace["u_steer"][k], q.trace["u_fx"][k])
        assert held == (u if u is not None else (0.0, 0.0)), (k, held, u)
    print(f"S5 queue ok: t_apply = {[round(a * H, 4) for a in exp_apply]} s, "
          "delay grows by Tc/2 per sample")


def test_same_obs_keys_in_both_modes():
    seen = {}

    def spy(mode):
        def ctrl(obs, t):
            seen.setdefault(mode, sorted(obs))
            return controller(obs, t)
        return ctrl

    _run("sync", duration=0.1, ctrl=spy("sync"))
    _run("virtual_async", LatencyModel.fixed(0.004), duration=0.1,
         ctrl=spy("virtual_async"))
    assert seen["sync"] == seen["virtual_async"], seen
    print("obs keys ok: identical in sync and virtual_async")


def test_measured_latency_runs():
    r = _run("virtual_async", LatencyModel.measured(), duration=0.2)
    assert np.array_equal(r.samples["c_k"], r.samples["t_compute_s"])
    assert r.meta["latency_model"] == {"kind": "measured"}
    print("measured latency ok: c_k is the timed controller call")


def test_argument_checks():
    def raises(fn, exc):
        try:
            fn()
        except exc:
            return
        raise AssertionError(f"expected {exc.__name__}")

    raises(lambda: ClosedLoopEval(PLANT, controller, TC, mode="nope"), ValueError)
    raises(lambda: ClosedLoopEval(PLANT, controller, TC, mode="virtual_async"),
           ValueError)
    raises(lambda: ClosedLoopEval(PLANT, controller, TC, mode="sync",
                                  latency=LatencyModel.fixed(0.0)), ValueError)
    raises(lambda: ClosedLoopEval(dict(PLANT, control_dt=TC), controller, TC),
           ValueError)
    raises(lambda: ClosedLoopEval(PLANT, controller, 0.0201), ValueError)
    raises(lambda: LatencyModel.fixed(-1e-3), ValueError)
    raises(lambda: LatencyModel.uniform(0.01, 0.0, seed=1), ValueError)
    print("argument checks ok")


def _lane_change_path():
    """The y_ref lane change as a dense polyline [X, Y] (0.25 m spacing)."""
    xs = np.arange(-10.0, 200.0, 0.25)
    return np.stack([xs, [y_ref(x) for x in xs]], axis=1)


def test_trace_sync_is_the_plant_trace():
    with tempfile.TemporaryDirectory() as tmp:
        ev = ClosedLoopEval(PLANT, controller, TC, mode="sync", duration=1.0,
                            state0=STATE0, trace_path=Path(tmp) / "a.vdtrace")
        res = ev.run()
        # Reference: the documented plant loop with the plant's own trace.
        plant = VDSimPlant(control_dt=TC, **PLANT)
        obs = plant.reset(STATE0)
        plant.enable_trace(Path(tmp) / "ref.vdtrace")
        for k in range(int(round(1.0 / TC))):
            obs = plant.step(list(controller(obs, k * TC)))
        plant.finalize_trace()
        with vt.TraceReader(res.meta["trace_path"]) as a, \
                vt.TraceReader(Path(tmp) / "ref.vdtrace") as ref:
            assert a.channel_names() == ref.channel_names(), a.channel_names()
            assert not any(a.has(n) for n in vt.EVAL_CHANNELS)
            for name in ref.channel_names():
                assert np.array_equal(a.channel(name), ref.channel(name)), name
            m = a.manifest
            assert m["eval_mode"] == "sync" and m["control_period_s"] == TC
            assert m["latency_model"] is None and m["overrun_policy"] is None
            assert a.n_steps == int(round(1.0 / TC))
    print("trace sync ok: channels bit-identical to VDSimPlant.enable_trace, "
          "eval_mode recorded, no timing channels")


def test_trace_virtual_async_timing_channels():
    lat = LatencyModel.fixed(0.03)          # 1.5*Tc: overruns, skip policy
    with tempfile.TemporaryDirectory() as tmp:
        ev = ClosedLoopEval(PLANT, controller, TC, mode="virtual_async",
                            duration=0.4, latency=lat, state0=STATE0,
                            trace_path=Path(tmp) / "b.vdtrace")
        res = ev.run()
        smp = res.samples
        with vt.TraceReader(res.meta["trace_path"]) as tr:
            m = tr.manifest
            assert m["eval_mode"] == "virtual_async"
            assert m["latency_model"] == lat.describe()
            assert m["overrun_policy"] == "skip"
            n = tr.n_steps
            assert n == int(round(0.4 / TC))
            assert np.array_equal(tr.channel("u_steer"), res.trace["u_steer"][:n])
            for k in range(n):
                t_k = k * TC
                # The sample whose command is in force at t_k.
                ok = (smp["applied"] == 1) & (smp["t_apply"] <= t_k + 1e-12)
                got = [tr.channel(c)[k] for c in vt.EVAL_CHANNELS]
                if not ok.any():
                    assert np.isnan(got[:3]).all() and got[3] == 0.0, (k, got)
                    continue
                i = int(np.flatnonzero(ok)[-1])
                want = [smp["t_sample"][i], smp["t_apply"][i], smp["c_k"][i],
                        float(smp["overrun"][i])]
                assert got == want, (k, got, want)
                assert tr.channel("u_steer")[k] == smp["u_steer"][i]
            assert np.nansum(tr.channel("overrun")) > 0
    print("trace virtual_async ok: t_sample/t_apply/c_k/overrun describe the "
          "command in force on every row, NaN before the first one")


def test_trace_timing_needs_eval_mode():
    names = [{"name": n} for n in ("t",) + vt.EVAL_CHANNELS]

    def rejected(manifest):
        try:
            vt._validate_eval_timing(manifest)
        except vt.TraceSchemaError:
            return True
        return False

    good = {"channels": names, "eval_mode": "virtual_async", "control_period_s": TC}
    assert not rejected(good)
    assert rejected(dict(good, eval_mode="sync"))
    assert rejected({"channels": names, "control_period_s": TC})
    assert rejected(dict(good, control_period_s=0.0))
    assert rejected(dict(good, channels=names[:-1]))
    assert not rejected({"channels": [{"name": "t"}]})
    assert all(n not in vt.BASE_CHANNELS for n in vt.EVAL_CHANNELS)
    assert all(n not in vt.channels_for_level("L5") for n in vt.EVAL_CHANNELS)
    print("trace timing validation ok: needs all four channels, a delay mode "
          "and a control period")


def test_lateral_error_and_tracking_stats():
    path = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]
    e = lateral_error([2.0, 5.0, 11.0, 9.0, 12.0], [1.0, -0.5, 4.0, 4.0, 12.0], path)
    # Left of +X is +Y; on the +Y leg left is -X; past the end the distance
    # is to the end point (10, 10): sqrt(8), on the right side.
    want = [1.0, -0.5, -1.0, 1.0, -math.sqrt(8.0)]
    assert np.allclose(e, want, atol=1e-12), e
    r = ClosedLoopEval(PLANT, controller, TC, mode="sync", duration=DURATION,
                       state0=STATE0, reference_path=_lane_change_path()).run()
    st = r.stats
    assert st["e_y_n"] == int(round(DURATION / TC))
    manual = lateral_error(r.trace["X"][:-1], r.trace["Y"][:-1], _lane_change_path())
    assert st["e_y_rmse_m"] == float(np.sqrt(np.mean(manual ** 2)))
    assert 0.0 < st["e_y_rmse_m"] < 1.0, st
    print(f"lateral error ok: signed offsets exact, lane change RMSE = "
          f"{st['e_y_rmse_m']:.4f} m over {st['e_y_n']} samples")


def main():
    test_s1_sync_matches_plant_loop()
    test_s2_async_zero_latency_matches_sync()
    test_s3_fixed_delay_matches_hand_delayed_lockstep()
    test_s4_seeded_distribution_repeats()
    test_s5_overrun_tables()
    test_same_obs_keys_in_both_modes()
    test_measured_latency_runs()
    test_argument_checks()
    test_trace_sync_is_the_plant_trace()
    test_trace_virtual_async_timing_channels()
    test_trace_timing_needs_eval_mode()
    test_lateral_error_and_tracking_stats()
    print("test_closedloop_eval: all ok")


if __name__ == "__main__":
    main()
