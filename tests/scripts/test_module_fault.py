"""Work order 51b M3: Python user modules that raise or return NaN (no pytest).

Fixed before the implementation (PO mail 16d10bd2). Every case runs in a child
interpreter (this file with --case), so a fault that still kills the process (rc -6)
fails that case only.
- Single session, generic sedan + generic tyre, 1 ms ticks, 10 m/s, CmdL4 throttle 0.1;
  the module is installed after reset_settled(); ticks 1-4 unarmed, armed before tick 5. Kinds brake, steering,
  drivetrain, aero on L2 and suspension, antirollbar on L4 (the levels that accept them)
  x {raise, NaN}. NaN fields reachable from Python: brake wheel_torque[2], steering
  rack_travel and roadwheel_angle, drivetrain wheel_torque[1], suspension force,
  antirollbar force[1], aero force_body.z and point.x.
  Pass: tick 5 raises ValueError whose text has the kind, the Python class name
  (Py<Kind>Fault), "tick 5" and the cause ("pyboom-<kind>#1" or the field name), and the
  interpreter keeps running Python code afterwards.
- VecSession path: 4 envs, threads 4, only env 2 faulty (raise and NaN brake module):
  vs.tick() raises ValueError naming the kind; the interpreter survives (no crash, no
  deadlock within the child timeout 120 s).
The pybind11 version is printed.
"""
from pathlib import Path
import math
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim

SEDAN = str(ROOT / "configs/vehicles/generic_sedan.yaml")
TIRE = str(ROOT / "configs/parts/tire/generic_pacejka.yaml")
DT = 1e-3
NAN = float("nan")


class Arm:
    """Shared arming flag and armed call count."""
    def __init__(self, kind, fault, field):
        self.kind, self.fault, self.field = kind, fault, field
        self.armed, self.n = False, 0

    def hit(self):
        """Count a call; raise when armed in raise mode. True = write NaN."""
        if not self.armed:
            return False
        self.n += 1
        if self.fault == "raise":
            raise RuntimeError(f"pyboom-{self.kind}#{self.n}")
        return True


class PyBrakeFault(vdsim.BrakeModule):
    def __init__(self, arm):
        vdsim.BrakeModule.__init__(self)
        self.arm = arm

    def wheel_torque(self, ctx):
        t = [0.0, 0.0, 0.0, 0.0]
        if self.arm.hit():
            t[2] = NAN
        return t


class PySteeringFault(vdsim.SteeringModule):
    def __init__(self, arm):
        vdsim.SteeringModule.__init__(self)
        self.arm = arm

    def apply(self, ctx):
        o = vdsim.SteeringOutput()
        if self.arm.hit():
            setattr(o, self.arm.field, NAN)
        return o


class PyDrivetrainFault(vdsim.DrivetrainModule):
    def __init__(self, arm):
        vdsim.DrivetrainModule.__init__(self)
        self.arm = arm

    def apply(self, ctx):
        o = vdsim.DrivetrainOutput()
        if self.arm.hit():
            o.wheel_torque = [0.0, NAN, 0.0, 0.0]
        return o


class PySuspensionFault(vdsim.SuspensionModule):
    def __init__(self, arm):
        vdsim.SuspensionModule.__init__(self)
        self.arm = arm

    def force(self, ctx, c):
        f = -30000.0 * c.defl - 3000.0 * c.defl_rate
        return NAN if self.arm.hit() else f


class PyAntiRollBarFault(vdsim.AntiRollBarModule):
    def __init__(self, arm):
        vdsim.AntiRollBarModule.__init__(self)
        self.arm = arm

    def force(self, ctx, d):
        return (0.0, NAN) if self.arm.hit() else (0.0, 0.0)


class PyAeroFault(vdsim.AeroModule):
    def __init__(self, arm):
        vdsim.AeroModule.__init__(self)
        self.arm = arm

    def compute(self, ctx, inp):
        if not self.arm.hit():
            return []
        fb = [0.0, 0.0, NAN] if self.arm.field == "force_body.z" else [0.0, 0.0, 0.0]
        pt = [NAN, 0.0, 0.0] if self.arm.field == "point.x" else [0.0, 0.0, 0.0]
        return [vdsim.AeroForce("probe", fb, pt)]


MODS = {
    "brake": (PyBrakeFault, "set_brake_module", "L2", ["wheel_torque[2]"]),
    "steering": (PySteeringFault, "set_steering_module", "L2", ["rack_travel", "roadwheel_angle"]),
    "drivetrain": (PyDrivetrainFault, "set_drivetrain_module", "L2", ["wheel_torque[1]"]),
    "suspension": (PySuspensionFault, "set_suspension_module", "L4", ["force"]),
    "antirollbar": (PyAntiRollBarFault, "set_antirollbar_module", "L4", ["force[1]"]),
    "aero": (PyAeroFault, "set_aero_module", "L2", ["force_body.z", "point.x"]),
}


def install(dyn, kind, mod):
    """Attach mod to dyn; True when the level accepts it."""
    setter = getattr(dyn, MODS[kind][1])
    return setter(0, mod) if kind == "antirollbar" else setter(mod)


def cmd():
    c = vdsim.CmdL4()
    c.throttle = 0.1
    return c


def start():
    return vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)


def child_single(kind, fault, field):
    """One single-session case; exit code 0 = pass."""
    cls, _, level, _ = MODS[kind]
    vp = vdsim.VehicleParams.from_yaml(SEDAN)
    tp = vdsim.TireParams.from_yaml(TIRE)
    s = vdsim.make_sim_session(vp, tp, level=level, nominal_dt=DT)
    arm = Arm(kind, fault, field)
    mod = cls(arm)
    s.reset_settled(start())  # before install: a reset restores the built-in modules
    if not install(s.dynamics(), kind, mod):
        print(f"M3 {kind} not accepted on {level}")
        return 8
    for _ in range(4):
        s.set_input(cmd())
        s.tick(DT)
    arm.armed = True
    msg, code = "", 1
    try:
        s.set_input(cmd())
        s.tick(DT)
    except ValueError as e:
        msg, code = str(e), 0
    except Exception as e:  # noqa: BLE001 - any other type is a failure code
        msg, code = f"{type(e).__name__}: {e}", 2
    cause = f"pyboom-{kind}#1" if fault == "raise" else field
    if code == 0:
        for c, need in ((3, kind), (4, cls.__name__), (5, "tick 5"), (6, cause)):
            if need not in msg:
                code = c
                break
    alive = sum(range(10)) == 45  # interpreter still runs Python code
    print(f"M3 {level} {kind:11s} {fault:5s} {field or '-':16s} code={code} alive={alive} msg=\"{msg}\"")
    return code if alive else 9


def child_vec(fault):
    """VecSession case: 4 envs, threads 4, env 2 faulty."""
    vp = vdsim.VehicleParams.from_yaml(SEDAN)
    tp = vdsim.TireParams.from_yaml(TIRE)
    vs = vdsim.make_vec_session(4, vp, tp, level="L2", nominal_dt=DT, threads=4)
    vs.reset_all([start() for _ in range(4)])
    arm = Arm("brake", fault, "wheel_torque[2]")
    mods = [PyBrakeFault(Arm("brake", "raise", "")) for _ in range(4)]
    mods[2] = PyBrakeFault(arm)
    for i in range(4):
        install(vs.at(i).dynamics(), "brake", mods[i])
    vs.set_input_all(cmd())
    vs.tick(DT, 4)
    arm.armed = True
    msg, code = "", 1
    try:
        vs.tick(DT, 1)
    except ValueError as e:
        msg, code = str(e), (0 if "brake" in str(e) else 3)
    except Exception as e:  # noqa: BLE001
        msg, code = f"{type(e).__name__}: {e}", 2
    vx = [vs.at(i).output().state.velocity[0] for i in (0, 1, 3)]
    print(f"M3 vec {fault} code={code} msg=\"{msg}\" other vx={vx} (info)")
    return code


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--case":
        kind, fault, field = sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv) > 4 else ""
        sys.exit(child_vec(fault) if kind == "vec" else child_single(kind, fault, field))
    try:
        import pybind11
        print("pybind11", pybind11.__version__)
    except ImportError:
        print("pybind11 (python package) not importable")
    cases = []
    for kind, (_, _, _, flds) in MODS.items():
        cases.append((kind, "raise", ""))
        cases += [(kind, "nan", f) for f in flds]
    cases += [("vec", "raise", ""), ("vec", "nan", "")]
    ok = 0
    failed = []
    for c in cases:
        try:
            r = subprocess.run([sys.executable, __file__, "--case", *c], capture_output=True,
                               text=True, timeout=120)
            rc, out = r.returncode, (r.stdout + r.stderr).strip().splitlines()
        except subprocess.TimeoutExpired:
            rc, out = "timeout", []
        print(out[-1] if out else "", f"[rc={rc}]")
        if rc == 0:
            ok += 1
        else:
            failed.append((c, rc))
    print(f"module_fault: {ok}/{len(cases)} cases OK")
    if failed:
        raise SystemExit(f"FAILED {failed}")


if __name__ == "__main__":
    main()
