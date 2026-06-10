#!/usr/bin/env python3
"""User-defined subsystem module — a custom brake coded in Python.

VDSim lets you replace any built-in subsystem (brake / steering / drivetrain /
suspension / anti-roll bar) with your own by subclassing the matching module base
and installing it on the model:

    class MyBrake(vdsim.BrakeModule):
        def wheel_torque(self, ctx): ...      # required
        def begin_step(self, ctx, dt): ...    # optional, once per step()
        def reset(self): ...                  # optional

    dyn.set_brake_module(MyBrake())

Call cadence:
  - begin_step(ctx, dt) runs once per step() — put step-coherent state here.
  - wheel_torque(ctx) runs once per RK4 stage — keep it a pure function of ctx
    (read step-state cached in begin_step if you need a single decision per step).

Sign convention (brake & drivetrain): wheel_torque returns a SIGNED torque on each
wheel-spin DOF [N m], FL,FR,RL,RR. A brake must OPPOSE spin, i.e. -sign(wheel_spin)*|T|;
returning a positive magnitude would drive the wheel and accelerate the car.

This script runs a hard stop with the default brake and with a custom front-biased
brake that adds a crude slip limiter (an ABS stand-in), and prints the stopping speed.
"""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "python"))

import vdsim


def flat_contacts():
    cs = [vdsim.ContactPoint() for _ in range(4)]
    for c in cs:
        c.is_valid = True
        c.normal = [0.0, 0.0, 1.0]
        c.mu_long = 1.0
        c.mu_lat = 1.0
    return cs


class AbsBrake(vdsim.BrakeModule):
    """Front-biased brake with a per-wheel slip-ratio limiter (toy ABS).

    Holds no integrator across RK4 stages: the single per-step decision (here just
    a cached pedal value) is taken in begin_step; wheel_torque stays pure.
    """

    def __init__(self, vp, bias_front=0.65, slip_limit=0.12):
        super().__init__()
        self.max_t = vp.max_brake_torque
        self.radius = vp.wheel_radius_nominal
        self.bias = bias_front
        self.slip_limit = slip_limit
        self.pedal = 0.0
        self.calls = 0

    def begin_step(self, ctx, dt):
        self.pedal = ctx.cmd.brake          # one pedal read per step

    def wheel_torque(self, ctx):
        self.calls += 1
        st = ctx.state
        vx = max(0.1, st.vx())
        out = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            axle_bias = self.bias if i < 2 else (1.0 - self.bias)
            demand = axle_bias * self.pedal * self.max_t
            # Crude ABS: back off if this wheel's longitudinal slip exceeds the limit.
            w = st.wheel_spin[i]
            slip = (w * self.radius - vx) / vx          # <0 under braking
            if slip < -self.slip_limit:
                demand *= 0.2
            out[i] = -math.copysign(1.0, w) * demand    # oppose spin
        return out


def hard_stop(install_abs):
    vp = vdsim.VehicleParams.from_yaml(str(REPO / "configs/parts/chassis/sedan.yaml"))
    tp = vdsim.TireParams.from_yaml(str(REPO / "configs/parts/tire/default_pacejka.yaml"))
    dyn = vdsim.create_seven_dof()
    dyn.initialize(vp, tp, vdsim.SolverParams())

    s0 = vdsim.State()
    s0.velocity = [28.0, 0.0, 0.0]
    w = 28.0 / vp.wheel_radius_nominal
    s0.wheel_spin = [w, w, w, w]
    dyn.reset(s0)

    mod = None
    if install_abs:
        mod = AbsBrake(vp)
        assert dyn.set_brake_module(mod), "L2 must accept a brake module"

    cs = flat_contacts()
    for _ in range(700):                                # 1.4 s at dt=2 ms
        cmd = vdsim.CmdL4()
        cmd.throttle = 0.0
        cmd.brake = 1.0
        dyn.step(cmd, cs, 0.002)
    return dyn.state().vx(), mod


def main():
    v_def, _ = hard_stop(install_abs=False)
    v_abs, mod = hard_stop(install_abs=True)
    print(f"default brake : end vx = {v_def:6.3f} m/s")
    print(f"custom ABS    : end vx = {v_abs:6.3f} m/s   (wheel_torque calls = {mod.calls})")
    print("Both decelerate; the custom module is a plain Python class installed at runtime.")


if __name__ == "__main__":
    main()
