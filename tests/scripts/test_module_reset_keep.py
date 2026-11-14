"""Work order 51c R3: Python user modules stay attached across reset (no pytest).

Fixed before the implementation (PROP-2794 option 1, work order 51c section 3).
Every case runs in a child interpreter (this file with --case), so a crash or a deadlock
fails that case only (child timeout 120 s).
- Single session, generic sedan + generic tyre, L2, 1 ms ticks, start 10 m/s, CmdL4
  throttle 0.1. Python subclasses of BrakeModule, SteeringModule and DrivetrainModule
  count their force calls and their reset() calls. Install after reset_settled(), 50 ticks,
  reset_settled(), 50 ticks. Pass: calls after the second reset > 0, the Python reset()
  override ran at least once since install, the interpreter still runs Python code.
- VecSession: 4 envs, threads 4, a counting brake module per env; 10 ticks, reset_all(),
  10 ticks. Pass: per env calls after reset_all > 0 and reset() >= 1, interpreter alive.
- reset raising: a brake module whose reset() raises RuntimeError. reset_settled()
  returns (the interpreter survives) and the next tick raises ValueError whose text has
  "brake" and "reset".
"""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim

SEDAN = str(ROOT / "configs/vehicles/generic_sedan.yaml")
TIRE = str(ROOT / "configs/parts/tire/generic_pacejka.yaml")
DT = 1e-3


class Counter:
    """Force-call and reset() counts shared with the test."""
    def __init__(self):
        self.calls, self.resets = 0, 0


class PyKeepBrake(vdsim.BrakeModule):
    def __init__(self, c, raise_on_reset=False):
        vdsim.BrakeModule.__init__(self)
        self.c, self.raise_on_reset = c, raise_on_reset

    def wheel_torque(self, ctx):
        self.c.calls += 1
        return [0.0, 0.0, 0.0, 0.0]

    def reset(self):
        self.c.resets += 1
        if self.raise_on_reset:
            raise RuntimeError("pyboom-reset")


class PyKeepSteering(vdsim.SteeringModule):
    def __init__(self, c):
        vdsim.SteeringModule.__init__(self)
        self.c = c

    def apply(self, ctx):
        self.c.calls += 1
        return vdsim.SteeringOutput()

    def reset(self):
        self.c.resets += 1


class PyKeepDrivetrain(vdsim.DrivetrainModule):
    def __init__(self, c):
        vdsim.DrivetrainModule.__init__(self)
        self.c = c

    def apply(self, ctx):
        self.c.calls += 1
        o = vdsim.DrivetrainOutput()
        o.wheel_torque = [0.0, 0.0, 30.0, 30.0]
        return o

    def reset(self):
        self.c.resets += 1


MODS = {
    "brake": (PyKeepBrake, "set_brake_module"),
    "steering": (PyKeepSteering, "set_steering_module"),
    "drivetrain": (PyKeepDrivetrain, "set_drivetrain_module"),
}


def cmd():
    c = vdsim.CmdL4()
    c.throttle = 0.1
    return c


def start():
    return vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)


def session():
    vp = vdsim.VehicleParams.from_yaml(SEDAN)
    tp = vdsim.TireParams.from_yaml(TIRE)
    return vdsim.make_sim_session(vp, tp, level="L2", nominal_dt=DT)


def run(s, n):
    for _ in range(n):
        s.set_input(cmd())
        s.tick(DT)


def child_single(kind):
    """One module kind on one session; exit code 0 = pass."""
    cls, setter = MODS[kind]
    s = session()
    c = Counter()
    s.reset_settled(start())
    if not getattr(s.dynamics(), setter)(cls(c)):
        print(f"R3 {kind} not accepted")
        return 8
    run(s, 50)
    s.reset_settled(start())
    n0 = c.calls
    run(s, 50)
    after = c.calls - n0
    alive = sum(range(10)) == 45
    code = 0 if (after > 0 and c.resets >= 1) else 1
    print(f"R3 L2 {kind:10s} calls_after={after} resets={c.resets} code={code} alive={alive}")
    return code if alive else 9


def child_vec():
    """VecSession: 4 envs, threads 4, reset_all keeps every env's module."""
    vp = vdsim.VehicleParams.from_yaml(SEDAN)
    tp = vdsim.TireParams.from_yaml(TIRE)
    vs = vdsim.make_vec_session(4, vp, tp, level="L2", nominal_dt=DT, threads=4)
    vs.reset_all([start() for _ in range(4)])
    cs = [Counter() for _ in range(4)]
    for i in range(4):
        vs.at(i).dynamics().set_brake_module(PyKeepBrake(cs[i]))
    vs.set_input_all(cmd())
    vs.tick(DT, 10)
    vs.reset_all([start() for _ in range(4)])
    n0 = [c.calls for c in cs]
    vs.set_input_all(cmd())
    vs.tick(DT, 10)
    after = [cs[i].calls - n0[i] for i in range(4)]
    resets = [c.resets for c in cs]
    alive = sum(range(10)) == 45
    code = 0 if all(a > 0 for a in after) and all(r >= 1 for r in resets) else 1
    print(f"R3 vec calls_after={after} resets={resets} code={code} alive={alive}")
    return code if alive else 9


def child_reset_raises():
    """reset() raising: the interpreter survives, the next tick raises ValueError."""
    s = session()
    c = Counter()
    s.reset_settled(start())
    s.dynamics().set_brake_module(PyKeepBrake(c, raise_on_reset=True))
    run(s, 4)
    s.reset_settled(start())
    msg, code = "", 1
    try:
        run(s, 1)
    except ValueError as e:
        msg = str(e)
        code = 0 if ("brake" in msg and "reset" in msg) else 3
    except Exception as e:  # noqa: BLE001 - any other type is a failure code
        msg, code = f"{type(e).__name__}: {e}", 2
    alive = sum(range(10)) == 45
    print(f"R3 reset_raises code={code} alive={alive} msg=\"{msg}\"")
    return code if alive else 9


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--case":
        k = sys.argv[2]
        if k == "vec":
            sys.exit(child_vec())
        if k == "reset_raises":
            sys.exit(child_reset_raises())
        sys.exit(child_single(k))
    cases = list(MODS) + ["vec", "reset_raises"]
    ok, failed = 0, []
    for c in cases:
        try:
            r = subprocess.run([sys.executable, __file__, "--case", c], capture_output=True,
                               text=True, timeout=120)
            rc, out = r.returncode, (r.stdout + r.stderr).strip().splitlines()
        except subprocess.TimeoutExpired:
            rc, out = "timeout", []
        print(out[-1] if out else "", f"[rc={rc}]")
        if rc == 0:
            ok += 1
        else:
            failed.append((c, rc))
    print(f"module_reset_keep: {ok}/{len(cases)} cases OK")
    if failed:
        raise SystemExit(f"FAILED {failed}")


if __name__ == "__main__":
    main()
