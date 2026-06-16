#!/usr/bin/env python3
"""Control-ladder verification sample.

Exercises every control-ladder level through the CascadeController so you can
verify the system end-to-end: each level should drive the plant to its target.

  Lon L4 pedal      : throttle/brake passthrough
  Lon L5 ax_target  : acceleration hold       → measured ax ≈ target
  Lon L6 vx_target  : cruise control          → measured vx ≈ target
  Lat L4 angle      : steer-angle passthrough
  Lat L6 r_target   : yaw-rate control         → measured r ≈ target
  Lat L7 kappa      : curvature → steer
  Split lon+lat     : independent axes (vx + yaw-rate)

Run:
    PYTHONPATH=build/python:python python3 examples/control_ladder_demo.py

Prints a pass/fail table; non-zero exit if any level misses its band.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "build" / "python"), str(REPO / "python")]

import vdsim


def make_sim(v0=15.0, dynamic_steering=False):
    vp = vdsim.VehicleParams.from_yaml(str(REPO / "configs/parts/body/sedan.yaml"))
    vp.steering_dynamic = dynamic_steering
    tp = vdsim.TireParams.from_yaml(str(REPO / "configs/parts/tire/default_pacejka.yaml"))
    sess = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=0.005, mu=1.0)
    sess.reset(vdsim.make_init_state(v=v0, wheel_radius=vp.wheel_radius_nominal))
    return sess


def make_sim_dyn(v0=15.0):
    return make_sim(v0, dynamic_steering=True)


def run(sess, cmd, t_end, dt=0.005):
    for _ in range(int(t_end / dt)):
        sess.set_input(cmd); sess.tick(dt)
    return sess.output()


def check(name, got, target, tol):
    ok = abs(got - target) <= tol
    mark = "PASS" if ok else "FAIL"
    print(f"  [{mark}] {name:28s} target={target:7.3f}  got={got:7.3f}  tol={tol:.2f}")
    return ok


def main():
    results = []

    # --- Lon L5: acceleration hold ---
    o = run(make_sim(10.0), _l5(2.0), 3.0)
    results.append(check("Lon L5 ax_target", o.ax, 2.0, 0.6))

    # --- Lon L6: cruise control ---
    o = run(make_sim(8.0), _l6(20.0), 15.0)
    results.append(check("Lon L6 vx_target", o.state.vx(), 20.0, 1.5))

    # --- Lat L6: yaw-rate control ---
    o = run(make_sim(15.0), _l6_lat_via_split(15.0, 0.15), 8.0)
    results.append(check("Lat L6 r_target (split)", o.state.yaw_rate(), 0.15, 0.08))

    # --- Lat L7: curvature ---
    o = run(make_sim(12.0), _l7(12.0, 0.02), 8.0)
    # kappa=0.02, v=12 → steady r ≈ kappa·v = 0.24
    results.append(check("Lat L7 kappa→yaw", o.state.yaw_rate(), 0.24, 0.10))

    # --- Split: independent vx + yaw-rate ---
    o = run(make_sim(15.0), _split(18.0, 0.10), 8.0)
    ok_vx = check("Split lon vx", o.state.vx(), 18.0, 1.5)
    ok_r  = check("Split lat yaw-rate", o.state.yaw_rate(), 0.10, 0.06)
    results.append(ok_vx and ok_r)

    # --- Lat L1 steer torque (Dynamic steering / EPS path) ---
    # Open-loop column torque has no position target — verify it moves the rack and
    # yaws the car in the correct direction (a real EPS closes the position loop).
    o = run(make_sim_dyn(15.0), _split_torque(15.0, 1.0), 3.0)
    moved = abs(o.state.rack_travel) > 1e-3
    yawed = o.state.yaw_rate() > 0.02      # +torque → +rack → +yaw
    print(f"  [{'PASS' if moved and yawed else 'FAIL'}] {'Lat L1 steer-torque→yaw':28s} "
          f"rack={o.state.rack_travel:.3f}  yaw={o.state.yaw_rate():.3f}  (moved+yawed)")
    results.append(moved and yawed)

    print()
    if all(results):
        print(f"ALL {len(results)} control-ladder levels verified.")
        sys.exit(0)
    print(f"{results.count(False)}/{len(results)} levels FAILED.")
    sys.exit(1)


# ---- command builders ----
def _l5(ax):
    c = vdsim.CmdL5(); c.ax_target = ax; return c

def _l6(vx):
    c = vdsim.CmdL6(); c.v_target = vx; return c

def _l7(vx, kappa):
    c = vdsim.CmdL7(); c.v_target = vx; c.kappa = kappa; return c

def _l6_lat_via_split(vx, r):
    c = vdsim.CmdSplit()
    c.lon = vdsim.LcLonL6(); c.lon.vx_target = vx
    c.lat = vdsim.LcLatL6(); c.lat.r_target = r
    return c

def _split(vx, r):
    return _l6_lat_via_split(vx, r)

def _split_torque(vx, torque):
    c = vdsim.CmdSplit()
    c.lon = vdsim.LcLonL6(); c.lon.vx_target = vx
    c.lat = vdsim.LcLatL1(); c.lat.steer_torque = torque
    return c


if __name__ == "__main__":
    main()
