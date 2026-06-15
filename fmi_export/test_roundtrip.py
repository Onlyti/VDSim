"""
Round-trip equivalence: VDSim native SimSession vs FMU co-simulation.

Both paths use the same vehicle (sedan) + tire (default_pacejka) + L2 dynamics
with identical step size (dt=0.02 s) and control inputs.  Maximum state deviation
must be < 1e-3 (sub-mm/ms² numerical noise only).

Run:
    # 1. build the FMU first
    bash fmi_export/build_fmu.sh
    # 2. run the test
    PYTHONPATH=build/python python3 fmi_export/test_roundtrip.py
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO / "build" / "python"), str(REPO / "python"),
                str(Path(__file__).resolve().parent)]

import vdsim
from fmu_master import FMUMaster


def main():
    fmu_path = REPO / "build" / "fmi_export" / "vdsim_l2.fmu"
    if not fmu_path.exists():
        print(f"Missing {fmu_path} — run fmi_export/build_fmu.sh first")
        sys.exit(1)

    veh_yaml  = str(REPO / "configs/parts/body/sedan.yaml")
    tire_yaml = str(REPO / "configs/parts/tire/default_pacejka.yaml")

    # ---- Path 1: VDSim native SimSession (L2) ----
    vp = vdsim.VehicleParams.from_yaml(veh_yaml)
    tp = vdsim.TireParams.from_yaml(tire_yaml)
    sp = vdsim.SolverParams()
    sess = vdsim.make_sim_session(vp, tp, "L2", nominal_dt=0.02)
    s0 = vdsim.make_init_state(v=0.0, wheel_radius=vp.wheel_radius_nominal)
    sess.reset(s0)

    cmd = vdsim.CmdL4()
    cmd.steer_angle_wheel = 0.05
    cmd.throttle = 0.30
    dt = 0.02; N = 100
    native_traj = []
    for _ in range(N):
        sess.set_input(cmd); sess.tick(dt)
        o = sess.output(); s = o.state
        native_traj.append((s.vx(), s.vy(), s.yaw_rate(), o.ay))

    # ---- Path 2: FMU via ctypes ----
    fmu = FMUMaster.load(fmu_path)
    fmu.initialize(0.0)
    fmu.set("steer_angle_wheel", 0.05)
    fmu.set("throttle", 0.30)
    fmu_traj = []
    for k in range(N):
        fmu.do_step(k * dt, dt)
        st = fmu.get_many("vx", "vy", "yaw_rate", "ay_body")
        fmu_traj.append((st["vx"], st["vy"], st["yaw_rate"], st["ay_body"]))
    fmu.free()

    # ---- Compare ----
    labels = ("vx", "vy", "yaw_rate", "ay")
    max_err = [0.0] * 4
    for nat, fv in zip(native_traj, fmu_traj):
        for i in range(4):
            max_err[i] = max(max_err[i], abs(nat[i] - fv[i]))

    print("Round-trip equivalence check (native SimSession vs FMU):")
    for lab, e in zip(labels, max_err):
        print(f"  max |Δ {lab}| = {e:.3e}")

    if all(e < 1e-3 for e in max_err):
        print("PASS")
        sys.exit(0)
    else:
        print("FAIL — non-trivial divergence")
        sys.exit(1)


if __name__ == "__main__":
    main()
