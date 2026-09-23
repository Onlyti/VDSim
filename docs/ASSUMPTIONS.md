# Assumptions & limitations

A model is trustworthy partly because its boundaries are stated. This page lists what
each dynamics rung (Ld1–Ld3, plus the Ld4 hardpoint add-on) represents and what it does
not, so that a result is never read outside its validity envelope.

Rung names follow the theory chapters: [Ld1-Bicycle](theory/04_ld1_bicycle.md),
[Ld2-SevenDOF](theory/05_ld2_seven_dof.md), [Ld3-FourteenDOF](theory/06_ld3_fourteen_dof.md),
[Ld4 hardpoint kinematics](theory/14_hardpoint_kinematics.md).

## Per-rung validity envelope

Legend: **yes** = modeled, **no** = not modeled, text = modeled with the stated limitation.

| Effect | Ld1 | Ld2 | Ld3 | Source |
|---|---|---|---|---|
| Path / heading kinematics | yes | yes | yes | theory 04–06 |
| Tire lateral force from slip | yes | yes | yes | theory 03 |
| Per-wheel forces | no | yes | yes | theory 05–06 |
| Combined slip (friction ellipse) | <!-- PO-VERIFY: Ld1 combined slip yes/no --> | yes | yes | theory 03, 25 |
| Load transfer | no | quasi-static | dynamic | theory 05–06 |
| Roll / pitch / heave | no | quasi-static roll / pitch estimate; no attitude or heave state | yes | theory 05–06 |
| Suspension transients | no | no | yes | theory 06 |
| Suspension kinematics from hardpoints | no | no | only with Ld4 hardpoints attached | theory 13–14 |
| Driveline (engine / gearbox / differential) | flat motor torque split by `drive_type`; opt-in engine map + gearbox; no differential (axle average) | flat motor torque; opt-in engine map + gearbox; open / locked / LSD differential | as Ld2 (through its inner Ld2) | theory 04, 05, 22 |
| Aerodynamics | drag + front/rear lift | drag + front/rear lift | drag + front/rear lift | `params.hpp` `aero_*` |

!!! note "Ld4 is not a separate plant without hardpoints"
    Selecting level `L4` without attaching suspension hardpoints is rejected at session
    build time: without hardpoints the plant is bit-identical to Ld3. Trace manifests
    record `kinematics_attached` so that a stored run shows which one it was.

## Global assumptions

- **Rigid bodies** — sprung and unsprung masses are rigid; no structural compliance.
- **Road input** — flat by default. Graded, banked, rough (ISO 8608 PSD) and split-μ
  surfaces enter through a contact provider; see
  [Tire contact & interface](theory/25_tire_contact_interface.md).
- **Tire transient** — the Magic Formula is steady-state. First-order relaxation length
  is available but off by default; LuGre / brush and belt-transient models are separate
  options ([19](theory/19_lugre_dynamic_tire.md), [21](theory/21_belt_transient.md)).
- **No tire thermal or wear state.** The Magic Formula coefficient set is isothermal;
  turn-slip, inflation-pressure and thermal terms are omitted. The only temperature state
  in the core is the optional brake-fade model of the actuator layer.
- **Friction** — a scalar μ scale, with an optional anisotropic ellipse
  (`mu_aniso`) and split-μ via the contact provider.
- **Fixed-step integration** — results depend on the substep size. See
  [Numerical integration](theory/11_numerical_integration.md) and the next section.

## Known numerical limitation: first-order load channel

The state integrator is RK4, but some forcing terms are held constant over the four
RK4 stages of a substep. The most visible one is the load-transfer forcing in Ld2
(`seven_dof_dynamics.cpp`, marked P3-7 in the source). As a result, wheel loads, pitch
and roll converge **first-order** in the substep size, not fourth-order.

Practical meaning:

- Yaw rate and lateral acceleration are accurate at the default substep.
- Pitch and roll are the least accurate outputs. Do not build a reward, cost or
  estimator target on pitch or roll without checking convergence at your substep.
- The measured convergence tables are in the RL guide's accuracy section
  ([RL guide §6](RL_GUIDE.md)); this page does not repeat the numbers.

## What this means for results

- Use the **lowest rung that contains the effect you study.** A result outside a
  rung's envelope (for example, dynamic load transfer on Ld2) is an artifact, not a
  finding.
- **Verified is not validated.** Agreement with reference solvers is verification;
  matching a real vehicle is a separate claim. See
  [Validation](VALIDATION.md#verification-vs-validation).
- **Sub-limit is not limit.** Linear-region agreement does not certify the saturated
  region.

## Out of scope (by design)

VDSim is an experimental research **plant**. It is not a production sign-off tool and
not a validated real-vehicle digital twin, and it makes no commercial tire-product
parity or commercial K&C claim. See [Validation — Honest limitations](VALIDATION.md)
and the [roadmap](ROADMAP.md).
