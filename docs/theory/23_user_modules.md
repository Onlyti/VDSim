# Ch.23 — User-defined subsystem modules

Every built-in subsystem in VDSim sits behind a small interface. A user can replace any
of them with a custom implementation — a C++ subclass or a Python subclass — and install
it on the model at runtime. The plant then calls the custom module exactly where it would
have called the built-in one, so a wrapper that just delegates reproduces the baseline
trajectory bit-for-bit (verified by `UserModules.WrapperIsTransparent*`).

This is the general form of the [Drivetrain v2](22_drivetrain_v2.md) shift policy: there a
single function returned a gear; here a whole module returns forces/torques and may keep
its own state.

## 1. The five modules

| Module base (Python / C++)            | Override            | Returns                              | Hosted on |
|---------------------------------------|---------------------|--------------------------------------|-----------|
| `BrakeModule` / `IBrakeSystem`        | `wheel_torque(ctx)` | signed wheel torque `[FL,FR,RL,RR]` (N m) | L2, L3, L4, L5 |
| `SteeringModule` / `ISteeringSystem`  | `apply(ctx)`        | `SteeringOutput(roadwheel_angle, rack_travel)` | L2, L3, L4, L5 |
| `DrivetrainModule` / `IDrivetrain`    | `apply(ctx)`        | `DrivetrainOutput(wheel_torque[4])` (signed) | L2, L3, L4, L5 |
| `SuspensionModule` / `ISuspension`    | `force(ctx, corner)`| corner force (N)                     | L3, L4 |
| `AntiRollBarModule` / `IAntiRollBar`  | `force(ctx, axle)`  | `(left, right)` wheel forces (N)     | L3, L4 |

The brake/steering/drivetrain hooks are available wherever the plant routes through the
pluggable subsystem objects: L2 (seven-DOF), L3 (fourteen-DOF, via its inner L2), L4
(kinematic, subclass of L3), and L5 (free-3D stunt). Suspension and ARB exist only on the
L3/L4 ride model. **L1** (bicycle) computes brake/drive inline and hosts no modules.

Install hooks on the model return `False` when the level does not host that module
(suspension/ARB exist only on the L3 ride model; L1 hosts none):

```python
dyn.set_brake_module(MyBrake())
dyn.set_steering_module(MySteering())
dyn.set_drivetrain_module(MyDrivetrain())
dyn.set_suspension_module(MySuspension())     # L3 only
dyn.set_antirollbar_module(0, MyARB())        # axle 0=front, 1=rear; L3 only
```

On L3 the brake/steering/drivetrain hooks forward to the inner planar model; suspension
and ARB are hosted on the ride model itself.

## 2. The context

Each callback receives a `SubsystemContext`:

- `ctx.state` — the full vehicle `State` (positions, velocities, `wheel_spin`, suspension
  travel; helpers `vx()`, `vy()`, `yaw()`, `yaw_rate()`, `beta()`).
- `ctx.cmd` — the `DriverCmd` (`handwheel_angle`, `throttle`, `brake`, `gear`, `handbrake`).
- `ctx.dt` — the step size; `ctx.Fz` — per-wheel vertical load `[N]`.

Suspension and ARB also receive the per-corner / per-axle deflection (`CornerInput`,
`AxleDefl`) for the current solver stage.

## 3. Call cadence and state (important)

The integrator is RK4 with optional substeps. The two entry points are called at
different rates:

- **`begin_step(ctx, dt)`** — once per `step()` call, before the RK4 stages. This is the
  place for a single per-step decision and any step-coherent state you want to keep
  (an ABS pedal read, a controller update, a latched mode).
- **`apply()` / `wheel_torque()` / `force()`** — once per RK4 stage (so several times per
  `step()`, and again per substep). Treat it as a pure function of its inputs: reading the
  stage `ctx` is fine, but do not advance an integrator here — it would be stepped multiple
  times with intermediate states.

`SuspensionModule` and `AntiRollBarModule` have no `begin_step`: a suspension/ARB force is
a memoryless function of the instantaneous deflection and must be evaluated every stage so
the vertical/roll ODE stays consistent.

`reset()` is called when the model is reset; clear your module state there.

## 4. Sign convention

`BrakeModule.wheel_torque` and `DrivetrainModule.apply().wheel_torque` are **signed**
torques applied to each wheel-spin DOF, ordered `FL, FR, RL, RR`. A brake must oppose
rotation:

$$
T_i = -\operatorname{sign}(\omega_{\mathrm{spin},i})\,\lvert T_i \rvert
$$

Returning a positive magnitude would drive the wheel and accelerate the vehicle. The
built-in brake uses a smoothed sign near zero spin to avoid chatter at a standstill; a
custom module can do the same or use `copysign`.

## 5. Example

`examples/user_brake_module.py` subclasses `BrakeModule` to implement a front-biased brake
with a crude per-wheel slip limiter (an ABS stand-in), installs it on an L2 model, and
compares the stop against the built-in brake. The module is a plain Python class — no
recompilation. For real-time / co-simulation use, prefer a C++ subclass: a Python module is
called under the GIL once per stage and is intended for offline studies and authoring.
