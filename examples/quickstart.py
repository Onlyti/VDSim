#!/usr/bin/env python3
"""Wheel-native quickstart: accelerate then brake on flat ground.

For pip installs only — no repo paths. Writes run.csv and run.png to the cwd.

    pip install "./vdsim-*.whl[plot]"
    vdsim-quickstart
    # or: python quickstart.py   (if quickstart.py is on PATH / copied from site-packages)
"""
from vdsim_lab import Sim, Road


def main():
    sim = Sim(level="L2", road=Road.flat(mu=1.0), v0=20.0)
    while not sim.done(8.0):
        st = sim.state()
        if st["t"] < 4.0:
            sim.set_input(throttle=max(0.0, (25.0 - st["vx"]) / 3.0))
        else:
            sim.set_input(brake=0.8)
        sim.run_core_dt()

    print(sim.result().summary())
    sim.to_csv("run.csv")
    sim.plot("run.png", signals=("vx", "ay", "r"))
    print("wrote run.csv run.png")


if __name__ == "__main__":
    main()
