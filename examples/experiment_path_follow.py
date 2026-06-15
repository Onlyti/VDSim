#!/usr/bin/env python3
"""Path-following experiment: an evaluator's pure-pursuit controller drives the
plant around a reference line, scored by cross-track error (CTE).

Shows how to (a) bring your own controller, (b) read a reference path, and
(c) get path metrics + an evidence figure.

    PYTHONPATH=build/python:python python3 examples/experiment_path_follow.py
"""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

from vdsim_lab import Sim, Road, resolve_line


class PurePursuit:
    """Minimal evaluator-side controller: steer toward a lookahead point,
    hold a target speed. The kind of code a VDSim user writes themselves."""

    def __init__(self, line, v_target=14.0, lookahead=8.0, wheelbase=2.7, max_steer=0.6):
        self.line = line
        self.v = v_target
        self.ld = lookahead
        self.L = wheelbase
        self.max_steer = max_steer

    def __call__(self, state):
        px, py, yaw = state["x"], state["y"], state["yaw"]
        pts = self.line
        i0 = min(range(len(pts)), key=lambda i: (pts[i][0] - px) ** 2 + (pts[i][1] - py) ** 2)
        j, acc = i0, 0.0
        while acc < self.ld:
            nj = (j + 1) % len(pts)
            acc += math.hypot(pts[nj][0] - pts[j][0], pts[nj][1] - pts[j][1])
            j = nj
            if j == i0:
                break
        tx, ty = pts[j]
        dx = math.cos(-yaw) * (tx - px) - math.sin(-yaw) * (ty - py)
        dy = math.sin(-yaw) * (tx - px) + math.cos(-yaw) * (ty - py)
        ld = max(1.0, math.hypot(dx, dy))
        steer = max(-self.max_steer, min(self.max_steer, math.atan2(2.0 * self.L * dy, ld * ld)))
        throttle = max(0.0, min(1.0, (self.v - state["vx"]) / 3.0 + 0.05))
        return steer, throttle, 0.0


def main():
    line = resolve_line({"source": "shape", "shape": "oval", "R": 25.0, "L": 80.0, "n": 240})
    x0, y0 = line[0]
    yaw0 = math.atan2(line[1][1] - y0, line[1][0] - x0)
    sim = Sim(level="L2", road=Road.flat(mu=1.0), v0=14.0, x0=x0, y0=y0, yaw0=yaw0)
    ctrl = PurePursuit(line, v_target=14.0, wheelbase=sim._vp.wheelbase)

    while not sim.done(30.0):
        steer, throttle, brake = ctrl(sim.state())
        sim.set_input(steer=steer, throttle=throttle, brake=brake)
        sim.run_core_dt()

    print(sim.result().summary())
    print("path metrics:", sim.metrics(["cte_rms", "cte_max", "lap_time", "peak_ay"], line=line))
    out = REPO / "results" / "path_follow"
    out.mkdir(parents=True, exist_ok=True)
    sim.to_csv(out / "run.csv")
    try:
        sim.plot(out / "run.png", signals=("xy", "vx", "ay"))
        print("wrote", out / "run.csv", "+ run.png")
    except RuntimeError as e:
        print("wrote", out / "run.csv", "(plot skipped:", e, ")")


if __name__ == "__main__":
    main()
