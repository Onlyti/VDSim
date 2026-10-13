#!/usr/bin/env python3
"""Closed-loop evaluation demo: one lane-change controller under modes A, B, C.

The same ``controller(obs, t)`` tracks a double lane change at 20 m/s and is
run as

* A ``sync`` -- every command applied at its sample time,
* B ``virtual_async`` with a fixed computation delay of 0, 5, 10 and 20 ms,
* C ``wallclock`` -- real threads, five repetitions,

and the lateral tracking error is compared (work order 31, S10).

Metric: ``e_y`` = signed distance [m] from the vehicle CG to the closest point
of the reference polyline, along the path normal, ground frame
(:func:`vdsim_closedloop.lateral_error`). RMSE over every control sample of
the run (lower is better). C reports the mean and standard deviation of the
per-run RMSE.

Usage::

    taskset -c 0-9,12-23 python3 examples/closed_loop_eval_demo.py --out demo_out

Writes ``results.json``, ``results.md`` and ``closed_loop_eval_demo.png``
into ``--out``. Mode C runs in real time (``--c-runs`` x ``--duration``
seconds) and its numbers depend on the host load, which is recorded.
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_closedloop import ClosedLoopEval, LatencyModel, lateral_error  # noqa: E402

TC = 0.02                    # control period [s]
H = 5e-4                     # substep [s]
V0 = 20.0                    # target speed [m/s]
PLANT = dict(level="L2", substep_dt=H)
STATE0 = [0.0, 0.0, 0.0, V0, 0.0, 0.0]
LANE = 3.5                   # lane offset [m]
B_DELAYS_MS = (0, 5, 10, 20)
K_PSI = 1.5                  # heading gain [-]
K_Y = 10.0                   # cross-track gain [1/s]


def y_ref(x):
    """Double lane change: out over x = 40..70 m, back over x = 130..160 m."""
    def ramp(x0, x1):
        if x <= x0:
            return 0.0
        if x >= x1:
            return 1.0
        return 0.5 * (1.0 - math.cos(math.pi * (x - x0) / (x1 - x0)))
    return LANE * (ramp(40.0, 70.0) - ramp(130.0, 160.0))


def reference_path():
    """The reference as a dense polyline [X, Y] [m] (0.25 m spacing)."""
    xs = np.arange(-20.0, 260.0, 0.25)
    return np.stack([xs, [y_ref(x) for x in xs]], axis=1)


def controller(obs, t):
    """Stanley-type lateral law on y_ref plus a speed P loop.

    The gains are high on purpose: a tight loop is what computation delay
    erodes first, so the B rows separate visibly.

    :returns: ``(delta [rad], Fx_total [N])``.
    """
    x, y, vx = obs["X"], obs["Y"], max(obs["vx"], 1.0)
    heading = math.atan((y_ref(x + 0.5) - y_ref(x - 0.5)) / 1.0)
    e_psi = heading - obs["psi"]
    e_y = y_ref(x) - y
    delta = K_PSI * e_psi + math.atan2(K_Y * e_y, vx)
    return max(-0.4, min(0.4, delta)), 2000.0 * (V0 - obs["vx"])


def run(mode, duration, latency=None):
    """One evaluation; returns its stats and the e_y(t) series."""
    path = reference_path()
    res = ClosedLoopEval(PLANT, controller, TC, mode=mode, duration=duration,
                         latency=latency, state0=STATE0,
                         reference_path=path).run()
    sampled = ~np.isnan(res.trace["u_steer"])
    t = res.trace["t"][sampled]
    e = lateral_error(res.trace["X"][sampled], res.trace["Y"][sampled], path)
    return res.stats, t, e


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="closed_loop_eval_demo_out")
    ap.add_argument("--duration", type=float, default=10.0, help="[s]")
    ap.add_argument("--c-runs", type=int, default=5)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rows, series = [], {}
    st, t, e = run("sync", args.duration)
    rows.append({"case": "A sync", "mode": "sync", "delay_ms": 0, **st})
    series["A sync"] = (t, e)
    for d in B_DELAYS_MS:
        st, t, e = run("virtual_async", args.duration, LatencyModel.fixed(d / 1e3))
        name = f"B fixed {d} ms"
        rows.append({"case": name, "mode": "virtual_async", "delay_ms": d, **st})
        series[name] = (t, e)

    load_before = os.getloadavg() if hasattr(os, "getloadavg") else None
    affinity = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
    c_runs = []
    for i in range(args.c_runs):
        st, t, e = run("wallclock", args.duration)
        c_runs.append(st)
        if i == 0:
            series["C wallclock (run 1)"] = (t, e)
    c_rmse = np.array([s["e_y_rmse_m"] for s in c_runs])
    summary = {
        "case": f"C wallclock x{args.c_runs}", "mode": "wallclock",
        "e_y_rmse_m": float(c_rmse.mean()),
        "e_y_rmse_sd_m": float(c_rmse.std(ddof=1)) if c_rmse.size > 1 else 0.0,
        "e_y_max_abs_m": float(max(s["e_y_max_abs_m"] for s in c_runs)),
        "e_y_n": int(round(np.mean([s["e_y_n"] for s in c_runs]))),
        "n_overrun": int(sum(s["n_overrun"] for s in c_runs)),
        "delay_p50_ms": 1e3 * float(np.mean([s["delay_p50_s"] for s in c_runs])),
        "period_p99_ms": 1e3 * float(max(s["period_p99_s"] for s in c_runs)),
        "rtf_min": float(min(s["rtf"] for s in c_runs)),
        "rtf_max": float(max(s["rtf"] for s in c_runs)),
    }

    result = {
        "metric": "e_y RMSE [m]: CG distance to the closest point of the "
                  "reference polyline along its normal, ground frame, over "
                  "every control sample; lower is better",
        "control_period_s": TC, "substep_dt_s": H, "speed_mps": V0,
        "duration_s": args.duration, "level": PLANT["level"],
        "host": {"loadavg_before_c": load_before, "cpu_affinity": affinity},
        "cases": rows, "c_runs": c_runs, "c_summary": summary,
    }
    (out / "results.json").write_text(json.dumps(result, indent=2))

    lines = ["| case | e_y RMSE [m] | max abs e_y [m] | samples | overruns |",
             "|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['case']} | {r['e_y_rmse_m']:.4f} | "
                     f"{r['e_y_max_abs_m']:.4f} | {r['e_y_n']} | {r['n_overrun']} |")
    lines.append(f"| {summary['case']} | {summary['e_y_rmse_m']:.4f} "
                 f"± {summary['e_y_rmse_sd_m']:.4f} | {summary['e_y_max_abs_m']:.4f} | "
                 f"{summary['e_y_n']} (mean) | {summary['n_overrun']} (total) |")
    table = "\n".join(lines)
    (out / "results.md").write_text(table + "\n")
    print(table)
    print(f"C: sample-to-apply delay p50 {summary['delay_p50_ms']:.3f} ms, "
          f"period p99 max {summary['period_p99_ms']:.2f} ms, RTF "
          f"{summary['rtf_min']:.5f}..{summary['rtf_max']:.5f}, "
          f"load before C {load_before}")

    plot(series, rows, summary, out / "closed_loop_eval_demo.png")
    print(f"wrote {out}/results.json, results.md, closed_loop_eval_demo.png")


def plot(series, rows, summary, path):
    """Two panels: e_y(t) of three cases and the RMSE of every case."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, muted, grid = "#0b0b0b", "#52514e", "#e4e3df"
    colors = {"A sync": "#2a78d6", "B fixed 20 ms": "#eb6834",
              "C wallclock (run 1)": "#1baf7a"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2),
                                   gridspec_kw={"width_ratios": [1.6, 1]})
    for name, color in colors.items():
        t, e = series[name]
        ax1.plot(t, e, color=color, lw=2, label=name)
    ax1.set_xlabel("time [s]", color=muted)
    ax1.set_ylabel("lateral error e_y [m] (ground frame)", color=muted)
    ax1.set_title("Lateral tracking error, double lane change at 20 m/s",
                  color=ink, fontsize=11, loc="left")
    ax1.legend(frameon=False, fontsize=9)

    names = [r["case"] for r in rows] + [summary["case"]]
    vals = [r["e_y_rmse_m"] for r in rows] + [summary["e_y_rmse_m"]]
    ypos = np.arange(len(names))[::-1]
    ax2.barh(ypos, vals, color="#2a78d6", height=0.6)
    # Only C repeats, so only C carries a spread.
    ax2.errorbar(vals[-1], ypos[-1], xerr=summary["e_y_rmse_sd_m"], fmt="none",
                 ecolor=ink, capsize=3, lw=1)
    for y, v in zip(ypos, vals):
        ax2.text(v, y, f"  {v:.3f}", va="center", fontsize=8, color=ink)
    ax2.set_yticks(ypos, names, fontsize=9)
    ax2.set_xlabel("e_y RMSE [m] (C: mean ± sd)", color=muted)
    ax2.set_title("RMSE by mode and delay", color=ink, fontsize=11, loc="left")
    ax2.set_xlim(0, max(vals) * 1.3)
    for ax in (ax1, ax2):
        ax.grid(True, color=grid, lw=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(colors=muted, labelsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
