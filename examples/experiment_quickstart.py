#!/usr/bin/env python3
"""Quickstart: the smallest possible VDSim experiment loop.

A constant-speed + brake test on flat ground. Shows the four seam calls:
state() -> set_input() -> run_core_dt(), then to_csv().

    PYTHONPATH=build/python:python python3 examples/experiment_quickstart.py

Pip installs: use examples/quickstart.py or vdsim-quickstart instead.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

from vdsim_lab import Sim, Road


def main():
    sim = Sim(level="L2", road=Road.flat(mu=1.0), v0=20.0)
    while not sim.done(8.0):
        st = sim.state()
        # accelerate to 25 m/s, then full brake after 4 s
        if st["t"] < 4.0:
            sim.set_input(throttle=max(0.0, (25.0 - st["vx"]) / 3.0))
        else:
            sim.set_input(brake=0.8)
        sim.run_core_dt()

    print(sim.result().summary())
    out = REPO / "results" / "quickstart"
    out.mkdir(parents=True, exist_ok=True)
    sim.to_csv(out / "run.csv")
    print("wrote", out / "run.csv")


if __name__ == "__main__":
    main()
