#!/usr/bin/env python3
"""Drive a VDSim plant in real time and watch it in the live 3D monitor.

The plant follows a lane that weaves between two lanes 3.5 m apart at a held
speed; every control period it is advanced once, paced to the wall clock, and
its observation is pushed to :class:`vdsim_livemonitor.LiveMonitor`. Open the
printed address in a browser (or start a standing server first with
``python -m vdsim_livemonitor serve`` and keep the page open).

    python examples/live_monitor_demo.py --duration 60
    python examples/live_monitor_demo.py \\
        --osm tests/fixtures/osm/livemonitor_sample.osm --origin 37.5,127.0
"""
import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

from vdsim_livemonitor import DEFAULT_PORT, LiveMonitor, vehicle_geometry  # noqa: E402
from vdsim_plant import VDSimPlant  # noqa: E402

TC = 0.02            # control period [s]
V_REF = 14.0         # held speed [m/s]
LANE = 3.5           # lane spacing [m]
WEAVE_M = 160.0      # one lane change and back [m]
K_PSI = 1.5          # heading gain [-]
K_Y = 0.6            # cross-track gain [1/s]
K_V = 0.8            # speed gain [1/s]


def lane_ref(x):
    """Reference lateral position [m] and heading [rad] at ``x`` [m]."""
    w = 2.0 * math.pi / WEAVE_M
    return 0.5 * LANE * (1.0 - math.cos(w * x)), math.atan(0.5 * LANE * w * math.sin(w * x))


def main(argv=None):
    """Run the demo; returns the process exit code."""
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--duration", type=float, default=60.0, help="[s] of simulated time")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--config", default="generic_sedan.yaml")
    ap.add_argument("--osm", default=None)
    ap.add_argument("--origin", default=None, help="lat,lon of the simulation origin")
    ap.add_argument("--open", action="store_true", help="open the page in a local browser")
    a = ap.parse_args(argv)

    plant = VDSimPlant(config=a.config, level="L2", control_dt=TC)
    mass = float(plant.vehicle.params.mass)
    obs = plant.reset([0.0, 0.0, 0.0, V_REF, 0.0, 0.0])
    origin = tuple(float(v) for v in a.origin.split(",")) if a.origin else None
    mon = LiveMonitor(port=a.port, osm=a.osm, origin=origin,
                      vehicle=vehicle_geometry(plant.vehicle.params),
                      open_browser=a.open)
    with mon:
        print(f"live monitor ({mon.mode}): http://<this host>:{a.port}/", flush=True)
        t0 = time.monotonic()
        for k in range(int(round(a.duration / TC))):
            y_ref, psi_ref = lane_ref(obs["X"])
            v = max(obs["vx"], 1.0)
            delta = (-K_PSI * (obs["psi"] - psi_ref)
                     - math.atan(K_Y * (obs["Y"] - y_ref) * 10.0 / v))
            fx = K_V * (V_REF - obs["vx"]) * mass
            obs = plant.step([max(-0.5, min(0.5, delta)), fx])
            mon.push({**obs, "t": (k + 1) * TC})
            lag = t0 + (k + 1) * TC - time.monotonic()
            if lag > 0:
                time.sleep(lag)
    return 0


if __name__ == "__main__":
    sys.exit(main())
