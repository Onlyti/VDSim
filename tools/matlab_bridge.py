#!/usr/bin/env python3
"""MATLAB Engine API bridge — Python ↔ MATLAB workspace variable exchange.

Transfers vdsim_lab run results into MATLAB workspace, or retrieves
MATLAB controller outputs (steer, throttle) back into Python.

Prerequisites:
    pip install matlabengine    # installs the MATLAB Engine for Python
    # or from MATLAB: cd(matlabroot); cd("extern/engines/python"); pyenv; system("python setup.py install")

Usage:
    # Start MATLAB Engine from Python and run a simulation
    python3 tools/matlab_bridge.py --demo

    # Export existing CSV to MATLAB workspace
    python3 tools/matlab_bridge.py --csv results/campaign/accel_brake/run.csv

    # Co-sim: Python (plant) + MATLAB (controller) over shared workspace
    python3 tools/matlab_bridge.py --cosim --matlab-script examples/simulink/pid_controller.m
"""
import argparse
import csv
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]


def _check_matlab():
    try:
        import matlab.engine
        return matlab.engine
    except ImportError:
        print("MATLAB Engine for Python not installed.")
        print("Install: pip install matlabengine")
        print("Or from MATLAB: cd(matlabroot); cd('extern/engines/python'); system('python setup.py install')")
        sys.exit(1)


def csv_to_workspace(csv_path: Path, engine, var_prefix: str = "vdsim"):
    """Load a vdsim_lab CSV into MATLAB workspace variables."""
    rows = list(csv.DictReader(open(csv_path)))
    cols = list(rows[0].keys())
    import matlab
    for col in cols:
        vals = [float(r[col]) for r in rows]
        engine.workspace[f"{var_prefix}_{col}"] = matlab.double(vals)
    print(f"Loaded {len(cols)} signals × {len(rows)} steps → MATLAB workspace ({var_prefix}_*)")
    return cols


def demo(engine):
    """Run a VDSim step-steer and push results to MATLAB for plotting."""
    from vdsim_lab import Sim, Road
    import matlab

    sim = Sim(level="L2", road=Road.flat(), v0=22.2)
    for k in range(int(8.0 / sim.dt)):
        steer = 0.03 if sim.time() >= 2.0 else 0.0
        sim.set_input(steer=steer, throttle=0.1)
        sim.run_core_dt()

    rows = sim.rows
    t  = matlab.double([r[0] for r in rows])
    vx = matlab.double([r[4] for r in rows])
    ay = matlab.double([r[8] for r in rows])
    r_yaw = matlab.double([r[6] for r in rows])

    engine.workspace["t"]     = t
    engine.workspace["vx"]    = vx
    engine.workspace["ay"]    = ay
    engine.workspace["r_yaw"] = r_yaw

    engine.eval("figure; subplot(2,1,1); plot(t,ay); xlabel('t [s]'); ylabel('ay [m/s^2]'); title('Step-steer ay');", nargout=0)
    engine.eval("subplot(2,1,2); plot(t,r_yaw); xlabel('t [s]'); ylabel('r [rad/s]'); title('Yaw rate');", nargout=0)
    print(f"Peak ay = {max(abs(v) for v in rows):.2f}  (plotted in MATLAB)")


def cosim_loop(engine, matlab_script: str, dt: float = 0.01, T: float = 8.0):
    """Co-simulation: VDSim (Python, plant) ↔ MATLAB (controller).

    MATLAB script is expected to:
      - Read workspace variables: 'vx', 'vy', 'yaw_rate', 'ay'
      - Write back: 'steer_cmd', 'throttle_cmd', 'brake_cmd'
    """
    from vdsim_lab import Sim, Road
    import matlab

    sim = Sim(level="L2", road=Road.flat(), v0=22.2)
    engine.run(matlab_script, nargout=0)   # initialise controller

    N = round(T / dt)
    for k in range(N):
        st = sim.state()
        engine.workspace["vx"]       = matlab.double([st["vx"]])
        engine.workspace["vy"]       = matlab.double([st["vy"]])
        engine.workspace["yaw_rate"] = matlab.double([st["r"]])
        engine.workspace["ay"]       = matlab.double([st["ay"]])
        engine.workspace["t_k"]      = matlab.double([st["t"]])

        engine.eval("vdsim_controller;", nargout=0)   # user MATLAB function

        steer    = float(engine.workspace["steer_cmd"][0])
        throttle = float(engine.workspace["throttle_cmd"][0])
        brake    = float(engine.workspace["brake_cmd"][0])

        sim.set_input(steer=steer, throttle=throttle, brake=brake)
        sim.run_core_dt(dt)

    print(f"Co-sim done. Steps={N}  vmax={max(r[4] for r in sim.rows):.2f} m/s")
    sim.to_csv(REPO / "results" / "matlab_cosim.csv")


def main():
    p = argparse.ArgumentParser(description="VDSim ↔ MATLAB Engine bridge")
    p.add_argument("--demo",        action="store_true", help="Run step-steer demo and plot in MATLAB")
    p.add_argument("--csv",         help="Push existing CSV into MATLAB workspace")
    p.add_argument("--cosim",       action="store_true", help="Python plant + MATLAB controller co-sim loop")
    p.add_argument("--matlab-script", default="", help="MATLAB script for controller (co-sim mode)")
    args = p.parse_args()

    me = _check_matlab()
    print("Starting MATLAB Engine ...")
    eng = me.start_matlab()

    try:
        if args.csv:
            csv_to_workspace(Path(args.csv), eng)
        elif args.cosim:
            cosim_loop(eng, args.matlab_script)
        else:
            demo(eng)
    finally:
        eng.quit()


if __name__ == "__main__":
    main()
