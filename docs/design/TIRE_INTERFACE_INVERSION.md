# Tire interface inversion (Phase 2) — design + handoff

Status: LANDED for the canonical path (364/364 ctest green, byte-stable across IsoBaseline /
Lugre / Belt / ChronoPac02 / SevenDOF). The inverted interface (evaluate / advance_bristle /
advance_relaxation) is defined ONCE in the `ITireModel` base (`tire_model.cpp`), dispatching
the force law through the virtual `compute()`; the base owns `params_` (via the
`on_initialize()` hook) so it works for every backend (pacejka / linear / MF2002).
`seven_dof_dynamics.cpp` (and 14-DOF, which wraps it) own only `Transient[NUM_WHEELS]` +
`ContactInput[NUM_WHEELS]`, hand kinematics to `evaluate()` per RK4 stage, advance the
bristle per stage and the carcass relaxation per substep — all inline slip/Re/belt/lugre/
relaxation blocks and their scratch members are deleted. The vehicle keeps only integrator
stabilization (lambda / Fx_hold / combined-slip clamp / kinematic blend / Fx*Re).
`test_tire_inversion.cpp` locks the backend against an independent oracle (4 force paths +
3 transient integrators + the linear backend via base). bicycle (L1) and free_3d (L5) keep
the Phase-1 shared path by design (see "Deferred" below).

## Next step (resume here)

1. DONE — `pacejka_mf96.cpp::evaluate()/advance()` + `test_tire_inversion.cpp`.
   `Transient` gained `belt_vlong/belt_vlat` (belt+LuGre stacking needs the relaxed slip
   *velocities*, which the original 5-field contract omitted).
2. DONE — `seven_dof_dynamics.cpp` switched. `advance()` was split into `advance_bristle()`
   (LuGre z, per RK4 stage at hz) and `advance_relaxation()` (belt_kappa/alpha +
   belt_vlong/vlat + alpha_dyn, once per substep at h) to preserve the mixed cadence
   byte-for-byte. Note: the Euler integrator branch advances only the bristle (no
   advance_relaxation), matching the pre-existing behaviour (a latent quirk for Euler+belt —
   not exercised by any RK4 test). Airborne wheels feed a neutral ContactInput, so the
   bristle/relaxation do not evolve off-ground (was stale `*_last_` before; no test covers
   it either way).
3. DONE (and re-architected) — instead of per-backend evaluate(), the inverted interface
   is now defined ONCE in the `ITireModel` base (`core/src/tire_model.cpp`): it computes
   slip/Re/camber from the shared kinematics, applies the transient, and dispatches the
   force law through the virtual `compute()`. The base stores `params_` (set by
   `initialize()`, which forwards to the backend `on_initialize()` hook), so Re / crown /
   belt / relaxation / LuGre work for EVERY backend. pacejka_mf96 / linear_tire /
   magic_formula_tire (MF2002) all inherit evaluate()/advance_* with their own compute().
   This also fixed a latent bug: after step 2, injecting a non-pacejka tire into
   seven_dof/14-DOF (e.g. `create_seven_dof(create_magic_formula_tire_from_tir(...))`,
   exposed in pybind) would have hit the old zero stub. A new tire is now one compute()
   override.

## L1 bicycle — switched (per-tire individual contact)

`bicycle_dynamics.cpp` now uses the inverted interface. Resolution of the old axle-lump
load split: model each axle as a per-tire contact at the per-wheel load
(`ContactInput.Fz = 0.5*Fz_axle`) and double the returned wrench for the axle force. Force
law AND Re then see a consistent load, and the load-sensitive mu is applied at the true
per-tire load (more correct than the old full-axle-load `compute()`). Byte-stable on
BicycleSteadyState (load_sensitivity defaults to 0 -> 2*F(0.5Fz) == F(Fz)); gated under
LuGre by `LuGreTire.HighSpeedLongitudinalNearPacejka` (L1) + the L1 no-phantom (reff) test.
Owns `Transient[2]` + `ContactInput[2]` (front/rear); the LuGre slip velocity now uses Re
consistently (was R0). 14-DOF/L2 already inverted (Phase 2).

## free_3d (L5) — switched (phase A of the 6-DOF re-founding)

L5 now uses the inverted interface too: `evaluate()` per RK4 stage + `advance_bristle()`
(per stage) / `advance_relaxation()` (per substep). The old loop-specific slip-ratio
denominator floor is gone — the contact-frame longitudinal velocity `v_long_k` (project the
hub velocity onto the wheel-heading tangent; fall back to the track-tangent speed near a
loop top where the heading tangent passes through zero) is fed as `ContactInput.Vx`, and the
tire owns the slip definition with the standard `max(|Vx|,eps)`. This also fixed a latent
bug: free_3d never integrated the LuGre bristle `z` (it stayed 0); it now advances it per
stage like L2/L3. Stunt loop tests (`FreeLoopCompletesLap`, `LoopSlipAngleFrontRearBalanced`)
still pass — the contact-frame `v_long_k` alone keeps the loop well-posed; the dropped floor
was not load-bearing. All L1-L5 + every backend now share one tire definition.

Remaining for L5 (separate, validation-gated — see `docs/design/L5_6DOF_MULTIBODY.md`):
phase B (attach the L4 hard-joint corner DAE suspension to the 6-DOF body) and phase C
(promote from "experimental" with jump/loop/cross-model validation evidence).

## Goal

Invert `ITireModel` from "slip-in -> force-out" to "kinematics-in -> wrench-out", and move
the transient state (relaxation / belt / LuGre) into the tire so that ALL tire physics
lives in the tire module. A new tire (different slip law, transient, camber model) is then
one class, not edits across 3 dynamics models.

## Current state (after Phase 1)

- `core/include/vdsim/tire_contact.hpp` — `tire_contact_kinematics(Vx,Vy,omega,Fz,gamma,tp,R0)`
  returns `{kappa, alpha, Re, vsx, vsy, contact_dy, contact_dz}`. Single definition of slip /
  Re / camber offset. seven_dof, bicycle, free_3d already call it (14-DOF via inner seven_dof).
- `ITireModel::Output` has `Mx` (default 0). `IVehicleDynamics::wheel_overturning_moment()`
  added; seven_dof returns `Fz*contact_dy`; 14-DOF feeds `Σ Mx` into the roll DOF.
- `TireParams.crown_radius` (camber migration) and `reff_*` (Re) are both opt-in (0 = legacy).
- Transient state (belt_kappa/alpha, lugre_z_long/lat, alpha_dyn) STILL lives in each
  dynamics model. This is what Phase 2 moves.

## Target interface

```cpp
struct TireInput  { double Fz, Vx, Vy, omega, gamma, mu_long, mu_lat, R0; };
struct TireTransient { double belt_kappa, belt_alpha, lugre_z_long, lugre_z_lat, alpha_dyn; };
struct TireOutput { double Fx, Fy, Mx, My, Mz, Re, kappa, alpha, contact_dy; };

class ITireModel {
  virtual TireOutput  evaluate(const TireInput&, const TireTransient&) const = 0; // RK4 stage (frozen)
  virtual TireTransient advance(const TireInput&, const TireTransient&, double dt) const = 0; // once/substep
};
```

Key decision: tire stays STATELESS/polymorphic; the `TireTransient` is OWNED and stored
per-wheel by the dynamics (RK4 needs to checkpoint/restore it). This preserves the existing
"freeze within RK4 stage, advance once per substep" pattern and avoids per-wheel tire
instances / clone(). evaluate() is const (called 4x/stage); advance() is const, returns the
next transient.

## What STAYS vehicle-side (do NOT move into the tire)

- Wheel-frame velocity projection (steer angle, yaw-rate, wheel position) — vehicle kinematics.
- Wheel rotational ODE and the moment arm Fx·Re — vehicle (uses Re returned by the tire).
- Low-speed numerics: stick-blend `lambda`, `Fx_hold` creep, combined-slip saturation clamp.
  These are integrator stabilization, not tire physics. Keep in the dynamics.
- free_3d stunt-loop denom specialization.

## Backends to port (3)

`core/src/pacejka_mf96.cpp`, `core/src/linear_tire.cpp`, `core/src/magic_formula_tire.cpp`
(MF2002). Move into each: the slip computation (call tire_contact_kinematics internally),
the relaxation (alpha_dyn) update, the belt transient (belt_tire.hpp), and the LuGre path
(lugre_tire.hpp) — currently invoked from the dynamics. evaluate() returns the full wrench;
advance() integrates belt/lugre/relaxation.

## Call-site rewrites (3 models)

`seven_dof_dynamics.cpp` (canonical, 4-wheel), `bicycle_dynamics.cpp` (2 axle "tires"),
`free_3d_dynamics.cpp` (4-wheel, 3D contact). Each: store `TireTransient[NUM_WHEELS]`,
call evaluate() per RK4 stage with frozen transient, call advance() once per substep, drop
the now-dead belt/lugre/relaxation/slip blocks. 14-DOF wraps seven_dof — no separate tire.

## Regression gate (non-negotiable)

When the new evaluate/advance reproduce the old equations, these must stay byte-stable:
`ctest -R "IsoBaseline|Lugre|Belt|ChronoPac02Parity|SevenDOF|BicycleSteadyState"` and full
354/354. The MF-vs-CarMaker parity (benchmark #15) and LuGre ISO baseline are the canaries.
Port one backend + one model first (seven_dof + pacejka), prove green, then the rest.

## Risks

- Belt+LuGre stacking (T2.3) and the dual belt paths (MF relaxes kappa/alpha; LuGre relaxes
  slip velocity) are subtle — port verbatim, test against current LuGre baseline before
  refactoring further.
- RK4 freeze/advance ordering must match exactly or transient tests drift.
