"""Work order 51c addendum K1: Python user modules stay alive without a user reference.

Fixed before the implementation (PROP-2796, PO mail b208d841). No pytest.
- Six module kinds: brake, steering, drivetrain, aero (L2) and suspension, anti-roll bar
  (front axle) (L4), generic sedan + generic tyre, 1 ms ticks, start 10 m/s, CmdL4
  throttle 0.1. Each module returns a fixed non-zero output so that a lost module
  changes the run.
- Two variants per kind, each in its own child interpreter (crash or deadlock fails that
  case only, child timeout 120 s):
  direct: reset_settled(), then dynamics().set_*_module(Cls()) with no variable holding
          the module, gc.collect(), 2000 ticks.
  kept:   the same, but the module is held in a local variable for the whole run.
- A tick that raises (fault, ValueError) or any other exception counts as a fault and
  ends the run.
- Digest: sha256 over float.hex() of every float / float list attribute of state()
  after every tick.
- Pass per kind: both variants exit 0, faults 0 and 2000 ticks in each, and the direct
  digest equals the kept digest (bit identical).
"""
from pathlib import Path
import gc
import hashlib
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import vdsim

SEDAN = str(ROOT / "configs/vehicles/generic_sedan.yaml")
TIRE = str(ROOT / "configs/parts/tire/generic_pacejka.yaml")
DT = 1e-3
TICKS = 2000


class KABrake(vdsim.BrakeModule):
    def __init__(self):
        vdsim.BrakeModule.__init__(self)

    def wheel_torque(self, ctx):
        return [20.0, 20.0, 20.0, 20.0]


class KASteering(vdsim.SteeringModule):
    def __init__(self):
        vdsim.SteeringModule.__init__(self)

    def apply(self, ctx):
        o = vdsim.SteeringOutput()
        o.roadwheel_angle = 0.01
        return o


class KADrivetrain(vdsim.DrivetrainModule):
    def __init__(self):
        vdsim.DrivetrainModule.__init__(self)

    def apply(self, ctx):
        o = vdsim.DrivetrainOutput()
        o.wheel_torque = [0.0, 0.0, 30.0, 30.0]
        return o


class KAAero(vdsim.AeroModule):
    def __init__(self):
        vdsim.AeroModule.__init__(self)

    def compute(self, ctx, inp):
        return [vdsim.AeroForce("probe", [0.0, 0.0, -200.0], [1.0, 0.0, 0.3])]


class KASuspension(vdsim.SuspensionModule):
    def __init__(self):
        vdsim.SuspensionModule.__init__(self)

    def force(self, ctx, c):
        return -30000.0 * c.defl - 3000.0 * c.defl_rate


class KAAntiRollBar(vdsim.AntiRollBarModule):
    def __init__(self):
        vdsim.AntiRollBarModule.__init__(self)

    def force(self, ctx, d):
        return (100.0, -100.0)


MODS = {
    "brake": (KABrake, "set_brake_module", "L2"),
    "steering": (KASteering, "set_steering_module", "L2"),
    "drivetrain": (KADrivetrain, "set_drivetrain_module", "L2"),
    "aero": (KAAero, "set_aero_module", "L2"),
    "suspension": (KASuspension, "set_suspension_module", "L4"),
    "antirollbar": (KAAntiRollBar, "set_antirollbar_module", "L4"),
}


def cmd():
    c = vdsim.CmdL4()
    c.throttle = 0.1
    return c


def start():
    return vdsim.make_init_state(0.0, 0.0, 0.0, 10.0)


def state_floats(st):
    """Every float or float-list attribute of a state object, in name order."""
    out = []
    for name in sorted(n for n in dir(st) if not n.startswith("_")):
        try:
            v = getattr(st, name)
        except TypeError:  # attributes without a Python conversion (e.g. a quaternion)
            continue
        if isinstance(v, float):
            out.append(v.hex())
        elif isinstance(v, (list, tuple)) and v and all(isinstance(x, float) for x in v):
            out.extend(x.hex() for x in v)
    return out


def child(kind, variant):
    """One kind, one variant; prints one JSON line, exit 0 when the run was accepted."""
    cls, setter, level = MODS[kind]
    vp = vdsim.VehicleParams.from_yaml(SEDAN)
    tp = vdsim.TireParams.from_yaml(TIRE)
    s = vdsim.make_sim_session(vp, tp, level=level, nominal_dt=DT)
    s.reset_settled(start())
    held = None
    if variant == "direct":
        if kind == "antirollbar":
            ok = getattr(s.dynamics(), setter)(0, cls())
        else:
            ok = getattr(s.dynamics(), setter)(cls())
    else:
        held = cls()
        ok = getattr(s.dynamics(), setter)(0, held) if kind == "antirollbar" \
            else getattr(s.dynamics(), setter)(held)
    if not ok:
        print(json.dumps({"kind": kind, "variant": variant, "accepted": False}))
        return 8
    gc.collect()
    h = hashlib.sha256()
    faults, ticks, msg = 0, 0, ""
    for _ in range(TICKS):
        try:
            s.set_input(cmd())
            s.tick(DT)
        except Exception as e:  # noqa: BLE001 - any raise is a fault for K1
            faults, msg = 1, f"{type(e).__name__}: {e}"
            break
        ticks += 1
        h.update(",".join(state_floats(s.state())).encode())
    print(json.dumps({"kind": kind, "variant": variant, "accepted": True, "faults": faults,
                      "ticks": ticks, "digest": h.hexdigest(), "msg": msg[:160]}))
    return 0


def run_child(kind, variant):
    try:
        r = subprocess.run([sys.executable, __file__, "--case", kind, variant],
                           capture_output=True, text=True, timeout=120)
        rc, lines = r.returncode, (r.stdout + r.stderr).strip().splitlines()
    except subprocess.TimeoutExpired:
        return "timeout", None
    rec = None
    for ln in lines:
        if ln.startswith("{"):
            rec = json.loads(ln)
    return rc, rec


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--case":
        sys.exit(child(sys.argv[2], sys.argv[3]))
    ok, failed = 0, []
    for kind in MODS:
        rd, d = run_child(kind, "direct")
        rk, k = run_child(kind, "kept")
        good = (rd == 0 and rk == 0 and d and k and d.get("accepted") and k.get("accepted")
                and d["faults"] == 0 and k["faults"] == 0
                and d["ticks"] == TICKS and k["ticks"] == TICKS
                and d["digest"] == k["digest"])
        dd = (d or {}).get("digest", "-")[:12]
        kd = (k or {}).get("digest", "-")[:12]
        print(f"K1 {kind:11s} direct rc={rd} faults={(d or {}).get('faults')} "
              f"ticks={(d or {}).get('ticks')} digest={dd} | kept rc={rk} "
              f"faults={(k or {}).get('faults')} ticks={(k or {}).get('ticks')} digest={kd} "
              f"same={dd == kd} {'PASS' if good else 'FAIL'} {(d or {}).get('msg', '')}")
        if good:
            ok += 1
        else:
            failed.append(kind)
    print(f"module_keep_alive: {ok}/{len(MODS)} kinds OK")
    if failed:
        raise SystemExit(f"FAILED {failed}")


if __name__ == "__main__":
    main()
