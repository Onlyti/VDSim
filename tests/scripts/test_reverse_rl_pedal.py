"""Work order 57 U3c (PROP-2785 (b)): RL direct-control pedal path while reversing (no pytest).

Fixed before the implementation commit.
One env, EnvConfig(level L2, generic_sedan + generic_pacejka, action_mode "pedal"),
reset at vx = -3 m/s (speed_range (-3, -3), no lateral / yaw offset), terminations that
a reversing car trips by construction switched off (beta near pi, stall, lane, time).
dt 0.005 s x action_repeat 4 = 0.02 s per step, 1.5 s = 75 steps. Values are the
SimOutput of the env at the end of each step (vs.outputs()).
- reverse acceleration action (action[1] = +0.5, throttle): brake_applied == 0 at every
  step end and the mean ax over step ends with t in [0.5, 1.5) s < 0 (drive torque
  backward).
- reverse deceleration action (action[1] = -0.5, brake): brake_applied > 0 at some step
  end and |vx(1.5 s)| < |vx(0.5 s)|.
- every value finite. Accel mode (CmdL5, action[1] = -0.125 x max_accel 4) is printed as
  information (covered by the B fix).
Forward-segment RL observation / reward digests (pedal, accel) are U1 (measured by the
U1 driver against the base tree).
"""
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import numpy as np

from vdsim_rl import EnvConfig, _Core

COUNT = {"ok": 0}


def check(cond, label):
    """Assert one case and count it."""
    if not cond:
        raise SystemExit(f"{label}: FAILED")
    COUNT["ok"] += 1
    print(label, "OK")


def run(action_long, mode="pedal"):
    """Reverse run from -3 m/s with a constant action; returns per-step records."""
    cfg = EnvConfig(level="L2", vehicle="generic_sedan", tire="generic_pacejka",
                    action_mode=mode, speed_range=(-3.0, -3.0), lateral_range=0.0,
                    yaw_range=0.0, min_speed=0.0, max_beta=0.0, max_yaw_rate=0.0,
                    max_lateral=0.0, time_limit_s=0.0, threads=1)
    core = _Core(1, cfg, seed=0)
    core.reset_all()
    rec = []
    step_t = cfg.dt * cfg.action_repeat
    for k in range(75):
        core.step(np.array([[0.0, action_long]], dtype=np.float32))
        o = core.vs.outputs()[0]
        rec.append(dict(t=(k + 1) * step_t, vx=o.state.velocity[0], ax=o.ax,
                        brake=o.brake_applied, throttle=o.throttle_applied,
                        term=int(core.term[0])))
    return rec


def window_mean_ax(rec):
    """Mean ax over step ends with t in [0.5, 1.5)."""
    xs = [r["ax"] for r in rec if 0.5 - 1e-9 <= r["t"] < 1.5 - 1e-9]
    return sum(xs) / len(xs)


def at(rec, t):
    """vx at the step end nearest t."""
    return min(rec, key=lambda r: abs(r["t"] - t))["vx"]


def finite(rec):
    return all(math.isfinite(r[k]) for r in rec for k in ("vx", "ax", "brake", "throttle"))


acc = run(+0.5)
m = window_mean_ax(acc)
bmax = max(r["brake"] for r in acc)
print(f"U3c accel pedal +0.5: mean_ax={m:+.6f} vx(0.5)={at(acc, 0.5):+.6f} "
      f"vx(1.5)={at(acc, 1.5):+.6f} brake_max={bmax:.6f} term={max(r['term'] for r in acc)}")
check(finite(acc), "U3c accel finite")
check(bmax == 0.0, "U3c accel brake_applied == 0 at every step end")
check(m < 0.0, "U3c accel mean ax < 0 (backward drive)")

dec = run(-0.5)
m = window_mean_ax(dec)
bmax = max(r["brake"] for r in dec)
v05, v15 = at(dec, 0.5), at(dec, 1.5)
print(f"U3c decel pedal -0.5: mean_ax={m:+.6f} vx(0.5)={v05:+.6f} vx(1.5)={v15:+.6f} "
      f"brake_max={bmax:.6f}")
check(finite(dec), "U3c decel finite")
check(bmax > 0.0, "U3c decel uses the brake")
check(abs(v15) < abs(v05), "U3c decel |vx| falls")

info = run(-0.125, mode="accel")
print(f"U3c accel-mode a=-0.5 m/s^2 (information): mean_ax={window_mean_ax(info):+.6f} "
      f"brake_max={max(r['brake'] for r in info):.6f}")

print(f"reverse_rl_pedal: {COUNT['ok']} checks OK")
