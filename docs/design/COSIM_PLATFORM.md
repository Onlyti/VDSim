# Design: component-test co-sim platform (subsystem modularization)

Status: DRAFT for alignment. Task #138.

## 1. Goal & philosophy

VDSim is the **platform**, not the high-fidelity component models. An OEM
component team should be able to **replace only their subsystem** (steering box,
suspension, powertrain, brakes, tire) with their own model — in-process plugin
or an external co-sim solver (e.g. brake-disc thermal CFD) — while every other
subsystem keeps a **good-enough VDSim default**. VDSim does not ship a brake-fade
or vapor-lock model; it ships the brake subsystem *interface* with the hooks that
let a fade model (theirs) drive it.

Revenue thesis (separate doc): sell the validated platform + turnkey coupling,
not the component models.

## 2. Current state

| subsystem | today | modular? |
|---|---|---|
| Tire | `ITireModel` (Pacejka MF / linear) | **yes** — clean interface |
| Suspension kinematics | `ISuspensionKinematics` (Ld4 native/lookup) | **yes** |
| Suspension ride | L3 spring/damper inline | partial |
| Steering | ackerman + ratio inline in dynamics (`seven_dof_dynamics.cpp:285`) | no |
| Powertrain | `throttle*max_motor_torque*final_drive` inline (`:368`) | no |
| Brakes | `brake*max_brake_torque*bias` inline (`:417`) | no |

So the B work = give **steering / powertrain / brakes** the same pluggable
treatment tire/suspension already have, plus a **bidirectional coupling port** so
a subsystem can be an external solver with feedback.

## 3. Proposed subsystem interfaces

Each is a small `compute(in)->out` contract with a built-in default. Signals are
named (so a coupling can map them). Per-wheel where relevant (FL,FR,RL,RR).

- **IPowertrain**: in `{throttle, gear, wheel_speed[4], dt}` → out
  `{drive_torque[4]}`. Default = current throttle·max_motor·final_drive + diff.
- **IBrakes**: in `{brake_cmd, wheel_speed[4], Fz[4], dt, feedback{...}}` → out
  `{brake_torque[4], heat_power[4]}`. Default = current bias·max_brake, constant
  μ, `heat_power = torque·ω`. **The `feedback` channel** carries externally
  supplied values (e.g. disc temperature or a μ-scale) so a fade model can act —
  VDSim just applies whatever μ-scale/torque-limit the feedback provides.
- **ISteering**: in `{driver_steer, v, dt}` → out `{road_wheel_angle[2], toe[4]}`.
  Default = ratio + ackerman (+ optional rack-force/servo already in actuator).

(Tire, suspension stay as-is. Suspension-ride extraction is later.)

## 4. Plug mechanisms (how a team swaps their scope)

A subsystem slot accepts one of:

1. **built-in default** (C++) — nothing to do.
2. **in-process plugin** — a Python (pybind trampoline) or C++ class implementing
   the interface. Good for prototyping / 1-D models. Python-in-loop is slow but
   fine offline.
3. **external co-sim port** — the subsystem is backed by an external solver
   exchanging **named signals per macro-step**. Two transports:
   - **FMI 2.0 co-sim** (preferred): the team exports their model as an FMU;
     VDSim acts as FMI master for that slot. (We have FMI *export* + `fmu_master.py`;
     this needs FMI *import* of a slave FMU.)
   - **socket / shared-mem bridge** for solvers that can't be FMUs.

   The port is **bidirectional**: VDSim writes subsystem inputs (e.g. brake
   heat_power, slip, Fz) and reads outputs/feedback (e.g. disc temperature) each
   macro-step. Macro-step ≠ micro-step: the heavy solver runs at a coarser rate;
   VDSim holds/extrapolates feedback between exchanges (offline co-sim, not RT).

## 5. Worked example — brake-disc thermal closed loop

```
VDSim (track laps, micro-step ~5 ms)
  brake subsystem (proxy) ── macro-step (e.g. 50–100 ms) ──▶
      out: brake_torque, heat_power[4], slip, Fz, v
  thermal/CFD FMU  ──▶  disc_temp[4]
  brake proxy feedback: μ_scale = user_fade_curve(disc_temp)   ← user's model
  -> brake_torque limited by μ_scale, vapor-lock flag, etc.
```

VDSim supplies the **loads and the hook**; the fade curve `μ(T)` and the thermal
model are the team's. VDSim's brake default (no feedback) = constant μ.

## 6. Sandbox scenario schema (builds on vdsim_lab Experiment)

Declarative run = the #139 Experiment + subsystem overrides + coupling:

```yaml
vehicle: sedan
level: L3
road: { preset: minor_road }       # or a track / driving line
maneuver: { path: laps.csv, v: 30, laps: 20 }
subsystems:
  brakes:
    coupling: fmu                  # default | plugin | fmu | socket
    fmu: brake_thermal.fmu
    macro_dt: 0.05
    map_out: { heat_power: q_in, slip: slip, Fz: Fz }   # VDSim -> solver
    map_in:  { disc_temp: T }                            # solver -> VDSim
log: [t, v, Fz, brake_torque, heat_power, disc_temp, mu_scale]
```

## 7. Phasing

1. **P1** — extract **IBrakes** (the example): interface + C++ default +
   Python-plugin trampoline + a `feedback{μ_scale}` input. Wire into L1/L2/L3.
   Tests: default == current behavior (re-baseline gate); a Python fade plugin
   changes stopping behavior.
2. **P2** — bidirectional coupling port + a reference 1-D thermal **plugin** (not
   shipped as product, just to demo/test the loop) and the sandbox YAML runner.
3. **P3** — **IPowertrain**, **ISteering** extraction (same pattern).
4. **P4** — FMI *import* (slave FMU) transport; socket bridge.

## 8. Open decisions (need alignment)

1. **Macro-step coupling** confirmed offline (not real-time)? CFD/thermal is slow,
   so yes by default — agree?
2. **First subsystem = brakes** (matches the example), then powertrain/steering?
3. **Transport priority**: in-process Python plugin first (fast to ship, lets us
   demo the brake loop end-to-end), FMI-import next? Or FMI-import first because
   that's what OEM solvers actually are?
4. **Feedback granularity** for brakes: pass raw `disc_temp` and let the plugin
   hold the μ(T) curve, or pass a ready `μ_scale`? (Former is more general.)
5. Keep subsystem interfaces **C++ with pybind trampolines** (so both C++ and
   Python plugins work), agreed?
