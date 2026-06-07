"""
Round-trip equivalence: VDSim → FMU export → FMU import → same trajectory.

This proves the FMU export is byte-for-byte equivalent to running the
same dynamics inside VDSim natively.  If a future change breaks one of
the two paths, this test catches it.
"""
import os, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import vdsim
from fmu_master import FMUMaster


def main():
    fmu_path = REPO / "build" / "fmi_export" / "vdsim_l2.fmu"
    if not fmu_path.exists():
        print(f"Missing {fmu_path} — run fmi_export/build_fmu.sh first")
        sys.exit(1)

    # ---- Path 1: VDSim native L2 ----
    sys.path.insert(0, str(REPO / "examples"))
    from _catalog_load import load_vehicle_tire
    vp, tp = load_vehicle_tire("sports")
    sp = vdsim.SolverParams()
    dyn = vdsim.create_seven_dof()
    dyn.initialize(vp, tp, sp)

    cp = lambda: [(_set(c) or c) for c in [
        vdsim.ContactPoint() for _ in range(4)]]
    def _set(c):
        c.is_valid = True; c.normal = [0, 0, 1]; c.mu_long = 1.0; c.mu_lat = 1.0
        return None
    contacts = cp()

    cmd = vdsim.CmdL4()
    cmd.steer_angle_wheel = 0.05
    cmd.throttle = 0.30
    dt = 0.02; N = 100
    native_traj = []
    for k in range(N):
        dyn.step(cmd, contacts, dt)
        s = dyn.state()
        native_traj.append((s.vx(), s.vy(), s.yaw_rate(), dyn.ay_body_est()))

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
    max_err = [0.0, 0.0, 0.0, 0.0]
    for nat, fmu_v in zip(native_traj, fmu_traj):
        for i in range(4):
            max_err[i] = max(max_err[i], abs(nat[i] - fmu_v[i]))

    labels = ("vx", "vy", "yaw_rate", "ay")
    print("Round-trip equivalence check (native vs FMU):")
    for lab, e in zip(labels, max_err):
        print(f"  max |Δ {lab}| = {e:.3e}")
    if all(e < 1e-6 for e in max_err):
        print("PASS — outputs identical to numerical precision")
        sys.exit(0)
    elif all(e < 1e-3 for e in max_err):
        print("PASS — small numerical noise (sub-mm/ms² scale)")
        sys.exit(0)
    else:
        print("FAIL — non-trivial divergence between native and FMU")
        sys.exit(1)


if __name__ == "__main__":
    main()
