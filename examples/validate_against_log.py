#!/usr/bin/env python3
"""Measured-data validation pipeline — replay a recorded log through VDSim.

Loads a driving log, replays its recorded commands (throttle/brake/steer)
through VDSim with the vehicle's parameters, and scores sim-vs-measured signals
(NRMSE, max error) so the model (and your estimator's plant) can be validated
against real data — e.g. the lab's CarMaker ERG / ADMA / rosbag captures
(TUR targets NRMSE < 3%).

Log format: a universal CSV with columns
    t, throttle, brake, steer, vx, yaw_rate, ax, ay
(throttle/brake 0..1, steer [rad], the rest the measured signals to compare).
Real captures plug in via thin loaders:
    .erg  -> CarMaker (use `cmerg` / CarmakerErgDataViewer)        [stub]
    ADMA  -> INS/GNSS device export                                 [stub]
    .bag  -> ROS1 rosbag                                            [stub]

Confidentiality: never commit measured tire/vehicle data or .erg/ADMA/bag files
(see .gitignore). This script only reads logs you point it at; the self-test
below uses generic sedan params.

Usage:
    python3 examples/validate_against_log.py              # self-test
    python3 examples/validate_against_log.py run.csv      # validate a real CSV log
"""
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim  # noqa: E402

SIGNALS = ["vx", "yaw_rate", "ax", "ay"]


def load_log(path):
    p = Path(path)
    ext = p.suffix.lower()
    if ext == ".csv":
        return _load_csv(p)
    if ext == ".erg":
        raise NotImplementedError(
            "CarMaker .erg: parse with `cmerg` (see ~/git/CarmakerErgDataViewer) "
            "and emit the universal CSV columns, then pass the CSV here.")
    if ext in (".bag",):
        raise NotImplementedError(
            "ROS1 rosbag: extract the command + state topics to the CSV columns.")
    raise NotImplementedError(f"no loader for '{ext}' (ADMA/other -> export to CSV)")


def _load_csv(path):
    import csv
    cols = {}
    with open(path) as f:
        rd = csv.DictReader(f)
        for k in rd.fieldnames:
            cols[k] = []
        for row in rd:
            for k, v in row.items():
                cols[k].append(float(v))
    return {k: np.array(v) for k, v in cols.items()}


def replay(log, vp, tp, level="L2", v0=None):
    """Feed the log's commands through VDSim; return sim signals per timestamp."""
    t = log["t"]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.005
    sess = vdsim.make_sim_session(vp, tp, level, nominal_dt=dt)
    if v0 is None:
        v0 = float(log["vx"][0]) if "vx" in log else 0.0
    sess.reset(vdsim.make_init_state(0, 0, 0, v0, vp.wheel_radius_nominal))
    out = {s: np.zeros(len(t)) for s in SIGNALS}
    for k in range(len(t)):
        c = vdsim.CmdL4()
        c.throttle = float(log.get("throttle", np.zeros(len(t)))[k])
        c.brake = float(log.get("brake", np.zeros(len(t)))[k])
        c.steer_angle_wheel = float(log.get("steer", np.zeros(len(t)))[k])
        sess.set_input(c); sess.tick(dt)
        o = sess.output()
        out["vx"][k] = o.state.vx()
        out["yaw_rate"][k] = o.state.yaw_rate()
        out["ax"][k] = o.ax
        out["ay"][k] = o.ay
    return out


def metrics(sim, meas):
    rows = []
    for s in SIGNALS:
        if s not in meas:
            continue
        e = sim[s] - meas[s]
        rng = float(np.max(meas[s]) - np.min(meas[s]))
        rmse = float(np.sqrt(np.mean(e ** 2)))
        nrmse = 100.0 * rmse / rng if rng > 1e-9 else 0.0
        rows.append((s, rmse, nrmse, float(np.max(np.abs(e)))))
    return rows


def _make_reference_csv(path, mass_scale=1.0):
    """Generate a synthetic 'measured' log from a VDSim run (sine steer)."""
    from _catalog_load import load_vehicle_tire
    vp, tp = load_vehicle_tire()
    vp.mass *= mass_scale
    dt, n = 0.005, 2400
    sess = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=dt)
    sess.reset(vdsim.make_init_state(0, 0, 0, 15.0, vp.wheel_radius_nominal))
    with open(path, "w") as f:
        f.write("t,throttle,brake,steer,vx,yaw_rate,ax,ay\n")
        for k in range(n):
            tk = k * dt
            thr = 0.25
            st = 0.05 * math.sin(2 * math.pi * 0.2 * tk)
            c = vdsim.CmdL4(); c.throttle = thr; c.steer_angle_wheel = st
            sess.set_input(c); sess.tick(dt)
            o = sess.output()
            f.write("%.4f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f\n" % (
                tk, thr, 0.0, st, o.state.vx(), o.state.yaw_rate(), o.ax, o.ay))


def main():
    if len(sys.argv) > 1:                       # validate a real log
        log = load_log(sys.argv[1])
        from _catalog_load import load_vehicle_tire
        vp, tp = load_vehicle_tire()
        sim = replay(log, vp, tp)
        print(f"=== validation: {sys.argv[1]} ===")
        for s, rmse, nrmse, mx in metrics(sim, log):
            print(f"  {s:9s} NRMSE {nrmse:6.2f}%   RMSE {rmse:.4f}   max|e| {mx:.4f}")
        return

    # self-test: (1) exact replay -> NRMSE ~ 0 (pipeline correct);
    #            (2) +10% mass mismatch -> NRMSE > 0 (pipeline discriminates).
    out = REPO / "logs"; out.mkdir(exist_ok=True)
    ref = out / "synthetic_measured.csv"
    _make_reference_csv(ref, mass_scale=1.0)
    log = load_log(ref)
    from _catalog_load import load_vehicle_tire
    vp, tp = load_vehicle_tire()
    print("=== self-test (1): exact params -> expect ~0 ===")
    for s, rmse, nrmse, mx in metrics(replay(log, vp, tp), log):
        print(f"  {s:9s} NRMSE {nrmse:7.3f}%   max|e| {mx:.5f}")
    vp.mass *= 1.10
    print("=== self-test (2): +10% mass mismatch -> expect > 0 ===")
    for s, rmse, nrmse, mx in metrics(replay(log, vp, tp), log):
        print(f"  {s:9s} NRMSE {nrmse:7.3f}%   max|e| {mx:.5f}")
    print(f"log -> {ref}")


if __name__ == "__main__":
    main()
