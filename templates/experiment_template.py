#!/usr/bin/env python3
"""VDSim experiment template — copy this file and write your controller.

VDSim gives you the simulation seam; YOU own the loop and the algorithm:

    sim.state()              ground-truth dict (x, y, yaw, vx, vy, r, ax, ay, Fz, slip_*)
    sim.measurements(id)     noisy sensor readout (at the mount pose, if registered)
    sim.set_input(...)       inject the action (steer [rad], throttle/brake [0..1])
    sim.run_core_dt()        advance one core step (dt)

Run:
    PYTHONPATH=build/python:python python3 templates/experiment_template.py

Then build your own copy: change `controller(...)` and the Sim(...) setup.
"""
import sys
from pathlib import Path

# Make vdsim_lab + the built vdsim module importable from a repo checkout.
REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]

from vdsim_lab import Sim, Road, Sensors


# --------------------------------------------------------------------------- #
# YOUR ALGORITHM — replace the body. Inputs are what the sim exposes each step;
# return the action (steer [rad] at the wheel, throttle [0..1], brake [0..1]).
# --------------------------------------------------------------------------- #
def controller(state, meas):
    v_target = 18.0
    throttle = max(0.0, min(1.0, (v_target - state["vx"]) / 3.0 + 0.05))
    steer = 0.03 if state["t"] > 2.0 else 0.0          # step steer at t = 2 s
    brake = 0.0
    return steer, throttle, brake


def main():
    # --- set up the plant (vehicle / tire / level / road / sensors) ---
    sim = Sim(
        vehicle="sedan",            # preset name | "*.yaml" path | Vehicle(...)
        tire="default_pacejka",
        level="L2",                 # L1 bicycle | L2 7DOF | L3/L4 14DOF | L5 stunt
        road=Road.iso8608("C"),     # .flat() .inclined() .split_mu() .iso8608() .preset()
        sensors=Sensors().gnss(pos_std=0.3).imu(),
        v0=12.0,                    # initial speed [m/s]
        sensor_mounts={             # optional: measure at the mount, not the CG
            "gnss": {"type": "gnss", "pos": [1.4, 0.0, 1.0]},
            "imu":  {"type": "imu",  "pos": [0.0, 0.0, 0.4]},
        },
    )

    # --- the loop is yours ---
    duration = 12.0
    while not sim.done(duration):
        state = sim.state()
        meas = {"gnss": sim.measurements("gnss"), "imu": sim.measurements("imu")}
        steer, throttle, brake = controller(state, meas)
        sim.set_input(steer=steer, throttle=throttle, brake=brake)
        sim.run_core_dt()

    # --- evidence ---
    print(sim.result().summary())
    print("metrics:", sim.metrics(["peak_ay", "cte_max", "vmax", "dist"]))
    out = REPO / "results" / "experiment_template"
    out.mkdir(parents=True, exist_ok=True)
    sim.to_csv(out / "run.csv")
    try:
        sim.plot(out / "run.png", signals=("vx", "ay", "r", "xy"))
        print("wrote", out / "run.csv", "+ run.png")
    except RuntimeError as e:
        print("wrote", out / "run.csv", "(plot skipped:", e, ")")


if __name__ == "__main__":
    main()
