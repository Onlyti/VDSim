# Ch.22 — Drivetrain v2: engine torque map + gearbox + shift policy

VDSim's default drivetrain applies a flat motor torque (`throttle x max_motor_torque x
final_drive_ratio`). Drivetrain v2 replaces that, **opt-in**, with a real engine + gearbox:
a 2D engine torque map, an N-speed gearbox coupled to engine RPM, and a shift policy that
can be a user-defined function. It is enabled by a `powertrain:` block in the vehicle
config; absent that block the legacy flat torque is used and the ISO baseline is unchanged.

Scope: L2 (seven-DOF) and L3 (fourteen-DOF, via its inner L2). L1 keeps the flat torque.

## 1. Engine torque map

Peak torque is a 2D map `T_peak(rpm, throttle)`, bilinearly interpolated and clamped to
the table domain. The actual torque is read directly from the map at the current
(rpm, throttle); the **closed-throttle row** (`throttle = 0`) carries the engine-braking
(motoring) torque, which is negative, so coasting produces a retarding axle torque.

$$
T_\mathrm{eng} = \mathrm{bilinear}\bigl(\text{torque\_map};\ \mathrm{rpm},\ \theta\bigr)
$$

## 2. Gearbox coupling

Engine speed follows the driven-wheel speed through the selected gear and the final drive:

$$
\omega_\mathrm{eng} = |\omega_\mathrm{wheel}|\, |i_g|\, i_\mathrm{fd},
\qquad
\mathrm{rpm} = \omega_\mathrm{eng}\,\frac{60}{2\pi}
$$

clamped to `[idle_rpm, redline_rpm]`. The axle drive torque is the engine torque amplified
by the gear and final drive, less mechanical loss:

$$
T_\mathrm{axle} = T_\mathrm{eng}\, i_g\, i_\mathrm{fd}\, \eta
$$

The engine inertia reflected to the wheel is **gear-dependent**,
$I_\mathrm{eng}\,(i_g\, i_\mathrm{fd})^2$, so a low gear has a larger effective rotating
inertia (slower spin-up) than a high gear — replacing the v1 final-drive-only reflection.

This reflected inertia $I_e$ enters the wheel-spin ODE through the differential. On an
**open** diff it is geared to the carrier, whose speed is the wheel mean
$\omega_c = (\omega_L + \omega_R)/2$, giving the coupled axle mass matrix

$$
\begin{bmatrix} I_L + I_e/4 & I_e/4 \\ I_e/4 & I_R + I_e/4 \end{bmatrix}
\begin{bmatrix} \dot\omega_L \\ \dot\omega_R \end{bmatrix} =
\begin{bmatrix} T_L \\ T_R \end{bmatrix}.
$$

Symmetric acceleration therefore feels the engine inertia ($\dot\omega = T/(I + I_e/2)$)
while a wheel-to-wheel speed difference does not (the spinning wheel is free) — the
defining open-diff behaviour. Locked/LSD axles carry $I_e/2$ rigidly on each wheel.

**Launch (idle floor + slipping clutch).** When the wheel is too slow to keep the engine
above idle (`rpm_geom < idle_rpm`), the clutch slips: the engine revs to a throttle-scaled
target between idle and a stall RPM, and the transmitted torque is the positive part of the
map (a slipping launch clutch does not transmit engine braking, so there is no reverse
creep at standstill).

## 3. Shift policy

Every step the gearbox asks a **shift policy** for the desired gear, then enforces a
`shift_time` torque interrupt (clutch open) on a change and locks out further shifts until
it elapses. Two built-ins:

- `manual` — follow the driver's commanded gear.
- `auto_rpm` — upshift above `upshift_rpm`, downshift below `downshift_rpm` (hysteresis).

### User-defined shift function

The policy can be **any function** `f(ShiftContext) -> desired_gear`, installed on the model:

```python
dyn.set_shift_policy(lambda ctx: 2 if ctx.engine_rpm > 5800 and ctx.current_gear < ctx.num_gears
                                   else ctx.current_gear)
```

`ShiftContext` carries `engine_rpm`, `current_gear`, `vehicle_speed`, `throttle`, `brake`,
`num_gears`. It is called once per step (except during the shift lock-out). In C++ it is a
`std::function<int(const ShiftContext&)>` via `IVehicleDynamics::set_shift_policy`; the
pybind layer accepts a Python callable directly. A custom policy overrides the declarative
`mode`.

## 4. Config

```yaml
powertrain:
  engine:
    idle_rpm: 800
    redline_rpm: 6500
    inertia: 0.20
    rpm_breaks:      [1000, 2500, 4000, 5500, 6500]
    throttle_breaks: [0.0, 0.5, 1.0]
    torque_map:                 # rows = throttle_breaks, cols = rpm_breaks
      - [-15, -20, -25, -30, -35]   # closed throttle = engine braking
      - [ 95, 150, 165, 150, 120]
      - [180, 300, 330, 300, 250]
  gearbox:
    gear_ratios: [3.40, 2.05, 1.40, 1.00, 0.82, 0.68]
    reverse_ratio: 3.20
    final_drive: 4.10
    efficiency: 0.92
    shift_time: 0.25
  shift:
    mode: auto_rpm              # manual | auto_rpm
    upshift_rpm: 6000
    downshift_rpm: 2000
    start_gear: 1
```

Runnable sample: `configs/powertrain_sedan_demo.yaml`. Telemetry: `dyn.engine_rpm()`,
`dyn.current_gear()`.

## 5. Validation

Headless tests: `EngineMap.*` / `PowertrainYaml.*` (map interpolation + config),
`EngineGearbox.*` (coupling, gear-dependent inertia, auto/custom shifting, launch, engine
braking), `DrivetrainV2.*` (full-throttle accel upshifts with a bounded RPM sawtooth,
programmatic policy override, launch from rest). The default preset (no `powertrain:`
block) is unchanged, so the ISO `IsoBaseline` gate is unaffected.
