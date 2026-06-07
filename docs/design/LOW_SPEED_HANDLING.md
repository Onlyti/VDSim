# Low-speed / standstill dynamics — research note + design

Status: implemented (2026-06) in L1/L2/L3. See "Implemented" below.
Motivation: under manual low-speed steering the L2/L3 model lurches in yaw,
drifts laterally, and oscillates left/right when coming to a stop. A series of
band-aids (lateral-force fade, lateral-velocity damping) reduced but did not
remove the artifacts — because this is a known structural problem, not a tuning
bug.

## The problem (root cause)

A slip-based tire model computes
- slip ratio  κ = (Ω·R − Vx) / Vx
- slip angle  α = arctan(Vy / Vx)   (per axle: αf = (Vy + lf·r)/Vx − δ, αr = (Vy − lr·r)/Vx)

Both divide by the longitudinal speed Vx, so as Vx → 0 the slip terms blow up and
the lateral/yaw states explode numerically. The current `kSpeedEps` floor on the
denominator caps the magnitude but the resulting force is still ill-conditioned
(snaps to the friction limit from velocity noise; over-large yaw moment with no
speed-dependent yaw damping → the spin/wiggle).

## Established solutions (literature)

1. **Kinematic–dynamic blending** (Kong et al. 2015; Polack et al. 2017). Below a
   speed threshold use the *kinematic* bicycle (all slip angles = 0; the car
   follows the steering geometry exactly: r = Vx·tan(δ)/L, β = arctan(lr·tan δ/L)),
   above it use the dynamic (force-based) model, blended on speed. Physically
   correct: at parking speed tires don't slip. Most widely used; removes the
   singularity by construction. Blend parameter is usually Vx (some use lateral
   acceleration).

2. **Implicit / backward-Euler discretisation of the lateral states**
   (arXiv 2011.09612, 2411.17334). Integrating Vy and r with backward Euler turns
   the update denominator from `1/Vx` into `m·Vx − Ts·(Cf+Cr)` (and
   `Iz·Vx − Ts·(lf²Cf+lr²Cr)`), which stays finite as Vx → 0. Keeps the dynamic
   model at all speeds, no blend. **Derived for a *linear* tire (constant
   cornering stiffness)** — not directly applicable to VDSim's nonlinear Pacejka.

3. **Low-speed damping** (Pacejka MF-Tyre "VLOW"): add a force proportional to the
   slip *velocity* below a threshold. Numerical-stability band-aid only — does not
   give correct low-speed behaviour or standstill hold. (≈ what the current
   fade/damping patches do.)

4. **Brush / LuGre stick** (Canudas de Wit 1995; Deur; Velenis): a contact-patch
   bristle deflection gives true static friction / standstill hold (incl. holding
   on a slope). Most physical but needs the bristle state + careful params, and on
   its own (no yaw damping) it spun the vehicle at low speed in our tests.

## Recommendation for VDSim

Adopt **(1) kinematic–dynamic blending** as the primary fix (works with the
nonlinear Pacejka model; matches the physics of manual low-speed steering; is the
textbook answer). Optionally layer **(4)** later for true slope standstill hold.

Concrete design (L2 7-DOF; L3 reuses the inner L2; L1 bicycle analogous):

- Kinematic targets from the front road-wheel angle δ and wheelbase L = a+b:
  - `r_kin  = Vx · tan(δ) / L`
  - `vy_kin = r_kin · b`            (lateral velocity at the CG, rear-ref bicycle)
- Blend factor `λ = smoothstep(Vx / V_blend)`, `V_blend ≈ 3 m/s` (λ=0 at rest → 1
  at V_blend).
- Apply as a constraint *relaxation* on the lateral state derivatives so it
  composes with RK4 (no post-hoc state jump):
  - `dVy += (1−λ) · (vy_kin − Vy) / τ`
  - `dr  += (1−λ) · (r_kin  − r ) / τ`,  τ ≈ 0.05 s
  At rest (λ=0) this pulls (Vy, r) onto the kinematic curve → the car turns
  smoothly per the steering and cannot oscillate/spin; at speed (λ=1) the term
  vanishes and the validated dynamic model is unchanged.
- Remove the lateral-force fade and lateral-velocity damping band-aids.
- Keep the existing standstill force cleanliness (forces still computed; the
  kinematic constraint governs the motion).

## Implemented (2026-06)

The shipped solution is the kinematic blend (1) plus a viscous brake-hold creep
damper in place of the brush/LuGre stick (4). The stick was prototyped and
dropped: an elastic bristle (sigma0*z) stores and releases energy and, layered on
the blend, rang at the stop and re-accelerated the car. A purely viscous damper is
unconditionally dissipative — it cannot ring — and gives a slope hold that is
good enough (cm/s creep), so it is the better engineering choice here.

Applied identically in L1 (`bicycle_dynamics.cpp`), L2 (`seven_dof_dynamics.cpp`),
and inherited by L3 (delegates the planar solve to L2):

- Blend factor `lambda = smoothstep(|V| / kStickBlend)`, `kStickBlend = 3 m/s`.
- Lateral states are *blended in the derivative*, not added on top:
  `dVy = lambda·dVy_dyn + (1-lambda)·(vy_kin - Vy)/tau`,
  `dr  = lambda·dr_dyn  + (1-lambda)·(r_kin  - r )/tau`, `tau = 0.05 s`,
  with `r_kin = Vx·tan(delta)/L`, `vy_kin = r_kin·lr`. At rest this is *pure*
  kinematic, so no force-induced yaw (the cause of the old spin/stop-oscillation);
  at speed (lambda=1) the validated dynamics are untouched. Longitudinal Vx stays
  fully dynamic.
- Brake-hold creep damper (longitudinal only): when on the brake and off the
  throttle, add `-kStickC·v_x_wheel` per wheel (`kStickC = 6e4 N·s/m`), gated by
  `(1-lambda)·clamp(brake-throttle, 0, 1)` and clamped to `mu·Fz`. A braked wheel
  does not fully lock at low speed, so its dynamic force fades to zero as `Vx->0`
  and the car would creep down a grade; this damper opposes the residual creep,
  which settles at `gravity/damping`. It never fights drive torque (gated off when
  throttle > brake) and vanishes above the blend speed.
- `kSpeedEps` slip-denominator floor tightened 0.5 -> 0.15 (lateral noise is now
  handled by the blend, not the floor, so the floor can be smaller for a sharper
  low-speed longitudinal force).
## LuGre opt-in (shipped 2026-06)

**Default (2026-06):** catalog `tire.default_pacejka` has `lugre.enabled: true` — LuGre
at all speeds; MF96 shapes `g()` (asymmetric floors). **Fallback:** set
`lugre.enabled: false` (`tire.kinematic_fallback`, `--no-lugre`) to use the blend +
brake-hold path below.

Theory: [`docs/theory/19_lugre_dynamic_tire.md`](../theory/19_lugre_dynamic_tire.md).
Design: [`V0.2_TIRE_LUGRE.md`](V0.2_TIRE_LUGRE.md), user guide
[`docs/CATALOG_AND_PHYSICS.md`](../CATALOG_AND_PHYSICS.md).

- The lateral tire force is faded by `lambda` in the *dynamics* (not just the
  display): as `Vx->0` the slip angle is ill-conditioned, so the raw lateral force
  chatters; left unfaded it leaks into `ay` and makes the left/right load transfer
  (`Fz`) tremble even though the car is at rest. Fading it keeps `ay -> 0` at a
  standstill, so `Fz` is clean. The low-speed lateral motion is carried by the
  kinematic relaxation, which is the physically correct low-speed behaviour. At
  `lambda=1` (above the blend speed) the validated force is unchanged.
- The reported longitudinal force additionally fades the raw slip part for the
  rendered arrows / logs (the slip ratio is ill-conditioned as `Vx->0`, so the
  per-wheel `Fx` chatters even though the net `ax` is smooth). This is
  display-only — the equations of motion use the full `Fx` so the car still
  self-arrests. The brake-hold damper keys off the body-longitudinal speed (not
  the steer-rotated wheel speed) so a steered front wheel cannot flip its sign.

Measured (sedan preset, mu=1.0): braked grade hold creep ~1-2 cm/s on 8%,
~2-3 cm/s on 12-20%; brake-to-stop settles with no left/right oscillation;
no-brake car still rolls down a grade (no phantom hold); ISO 7401/4138/3888-2 and
all 190 ctest checks unchanged (the blend is inactive above 3 m/s).

With **LuGre off** (default): a true zero-creep static hold still needs brush
stick or brake static friction; the viscous damper leaves small grade-proportional
creep. With **LuGre on**: presliding `z` + gated `g_y` give slope hold without
the blend path — enable via `tire.lugre_on` or GUI toggle (see Ch. 19).

## Sources
- Kong, Pfeiffer, Schildbach, Borrelli, "Kinematic and dynamic vehicle models for
  autonomous driving control design," IEEE IV 2015.
- Polack et al., "The kinematic bicycle model: a consistent model for planning
  feasible trajectories for autonomous vehicles?," IEEE IV 2017.
- "Numerically Stable Dynamic Bicycle Model for Discrete-time Control," arXiv:2011.09612.
- "An Explicit Discrete-Time Dynamic Vehicle Model with Assured Numerical
  Stability," arXiv:2411.17334.
- Pacejka, *Tyre and Vehicle Dynamics* (low-speed / MF-Tyre VLOW).
- Canudas de Wit et al., "A new model for control of systems with friction"
  (LuGre), IEEE TAC 1995.
