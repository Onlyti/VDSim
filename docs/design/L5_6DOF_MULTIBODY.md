# L5 — re-found as the full 6-DOF multibody model (design + roadmap)

Status: DESIGN (not implemented). This supersedes the earlier "remove L5" option.

## Why

L1-L4 cannot jump. L1/L2 are planar (no vertical state); L3/L4 model ride/suspension
travel about the road plane (heave/roll/pitch + unsprung, tire-spring coupled) but the body
pose stays planar `(x, y, yaw)` — there is no free-flight CG world-z + 3D attitude, so a
vehicle cannot leave the ground ballistically and land. `free_3d` (L5) is the ONLY model
with a full 6-DOF spatial rigid body (world position incl. z, quaternion roll/pitch/yaw,
penalty contact), hence the only airborne/loop-capable rung.

Today L5 is a stunt plant: 6-DOF body + penalty-glued wheels + capped/stiffened contact +
a loop-specific slip-ratio denominator, and NO suspension. L4 already carries the
high-fidelity multibody DYNAMICS (hard-joint corner DAE: revolute + Baumgarte travel
constraint) — but only on the planar+ride base. So the coherent ladder is:

```
L1 planar single-track -> L2 planar 4-wheel -> L3 14-DOF ride
   -> L4 hardpoint multibody (planar+ride base)
   -> L5 = L4's multibody on a full 6-DOF spatial body   <-- target
```

Re-founding L5 this way turns jump/loop from a robust-tuned demo hack into a real,
validatable model, and lets L5 share the inverted tire interface like L2/L3.

## Target architecture

1. **6-DOF spatial body** — Newton-Euler in world frame: translational (incl. z, gravity,
   contact + tire forces) + rotational (quaternion + body inertia, suspension/tire moments).
   free_3d already integrates position+quaternion+vel+omega; reuse it.
2. **L4 corner suspension on the 6-DOF body** — attach the hard-joint corner DAE
   (`multibody.hpp::IHardJointCornerDae`) per wheel, with the corner mount kinematics
   expressed in the 6-DOF body frame; the corner constraint/spring/damper forces feed the
   spatial Newton-Euler EoM. (Currently the DAE attaches to the 14-DOF planar+ride base.)
3. **Inverted tire interface** — replace the inline penalty/loop tire with
   `tire_->evaluate()` + `advance_bristle`/`advance_relaxation`, like L2/L3/L1.
4. **Contact-frame slip (the key fix)** — replace the loop-specific slip-ratio denominator
   with a *contact-frame* slip: project the hub velocity onto the contact tangent plane
   (normal from the surface), and define slip from the contact-tangential speed. This is
   physically correct on arbitrary 3D surfaces (flat, banked, loop) and uses the standard
   `max(|V_tangent|, eps)` denominator — so L5 joins the inverted contract with no
   special-case knob. The loop denom was a workaround for the missing contact-frame
   projection.
5. **Honest 3D contact** — keep a penalty normal force (needed for airborne/landing) but
   drop the stunt-specific caps where the contact-frame slip + real tire make them
   unnecessary; document any remaining numerical guards.

## Phasing

- **A — DONE** — 6-DOF body + contact-frame slip + inverted tire (suspension still
  rigid/penalty). free_3d now routes the tire through `evaluate()` / `advance_bristle()` /
  `advance_relaxation()`; the loop-denominator floor is replaced by the contact-frame
  `v_long_k` (hub velocity onto the wheel-heading tangent, track-tangent fallback near a loop
  top), so L5 joins the inverted contract with the standard slip denominator. Also fixed:
  the LuGre bristle `z` is now integrated (was degenerate / never advanced). All stunt/L5/
  terrain tests pass (loop tests included); 364/364 ctest green.
- **B1 — DONE** — spatial sprung/unsprung strut on the 6-DOF body, opt-in behind
  `SolverParams::l5_spatial_suspension` (default false keeps the penalty-at-hub path
  byte-stable; all 364 prior tests unchanged). Per corner: an unsprung mass `m_u[i]` with a
  travel DOF reusing `State::susp_compression/velocity[i]` along the body-up (strut) axis.
  Tire-spring acts on the unsprung via an effective penetration `pen = pen_rigid -
  comp·(ez·n)` (the wheel centre rides `comp` above the rigid hub the contact provider
  assumed); the strut spring+damper carries the sprung corner with a static preload so
  `comp=0` is the ride position, tops out at `F_susp=0` (no coilover tension), bump/droop
  clamps on travel. The vertical body↔wheel coupling goes through the strut (the strut-axis
  component of the tire force is replaced by `F_susp`); in-plane tire force is reacted
  rigidly. Body translational mass is anisotropic on the strut path: m_sprung along the
  strut axis (the unsprung decouples there via the travel DOF), total mass in-plane (the
  unsprung is rigidly carried), so flat-ground handling stays matched to planar L2/L3;
  rotational inertia uses `inertia_diag` as L2/L3 do. Isolation evidence (tests
  `L5Strut.*`): settles to static equilibrium (Fz_sum=14808 N vs weight 14710 N, comp≈0,
  z=0.533 m = ride height − tire static deflection); heave ride frequency 1.33 Hz measured
  vs 1.41 Hz quarter-car analytic (k_eff = 4·k_s·k_t/(k_s+k_t) over m_sprung).
- **B2** — feed `comp` (+ rate) as `PrescribedCornerMotion.travel_z` to the L4 corner DAE,
  set the returned `WheelPose` toe/camber into the tire `ContactInput`.
- **C** — validation (below).

## Phase B — implementation design (spatial suspension on the 6-DOF body)

Grounding (studied): the L4 corner DAE (`multibody.hpp::IHardJointDaeModel::step`) is a
per-corner suspension *kinematics/quasi-dynamics*: it consumes a PRESCRIBED travel
(`PrescribedCornerMotion{travel_z, travel_z_dot, ...}`) + the wheel `WheelLoad`, integrates
the revolute + Baumgarte travel constraint, and returns a `WheelPose` (toe/camber/caster).
The 14-DOF feeds it `travel = z_u[i] - z_corner_s` (unsprung world z minus sprung corner z)
from its OWN heave/roll/pitch + unsprung vertical dynamics, then applies the camber to the
tire via `set_camber_per_wheel`. So the DAE does NOT generate the vertical force law — the
host owns the sprung/unsprung dynamics that produce `travel`.

free_3d today has NO suspension: wheels are glued to the body at fixed `r_body_[i]` and held
up by a penalty contact (`Fz = k_tire*penetration + c*vn`, capped). Phase B therefore must
add a spatial sprung/unsprung suspension to the 6-DOF body before the DAE has a `travel` to
consume. Two sub-pieces:

- **B1 — spatial strut dynamics (the large, new piece).** Per corner add an unsprung mass
  `m_u[i]` with one travel DOF `z_strut[i]` along the strut axis expressed in the BODY frame
  (so it rotates with the 6-DOF attitude). Forces: a strut spring+damper between the body
  mount and the unsprung (`F_susp = k_s*z_strut + c_s*z_strut_dot`), and the tire vertical
  via the existing contact (`Fz` from the road, now acting on the unsprung not the body).
  The strut reaction feeds the 6-DOF body Newton-Euler (force at the mount point `r_body_[i]`,
  moment `r_body_[i] x F`); the unsprung gets `m_u*z_strut_ddot = Fz_tire - F_susp - m_u*g·n`.
  This generalises the 14-DOF heave/roll/pitch+unsprung to a free 6-DOF body: the "sprung
  mass" is the full spatial body, the strut axis is body-fixed. State grows by `z_strut[i]`,
  `z_strut_dot[i]` (8 scalars). The penalty-at-fixed-hub contact is replaced by tire-spring
  on the unsprung.
- **B2 — corner DAE wiring (small, once B1 exists).** Feed `travel = z_strut[i]` (and rate)
  as `PrescribedCornerMotion` to `create_hard_joint_dae_model(topo)` per corner, get the
  `WheelPose`, set `gamma[i] = camber_rad` and `toe[i]` into the tire `ContactInput` (camber
  already flows through `evaluate()`; toe rotates the wheel-heading tangent). Reuse the
  14-DOF wiring pattern (`step_hard_joint_dae`, `axle_prescribed_motion`).

Risk: B1 changes how the car is supported (strut+tire-spring instead of penalty-at-hub), so
every stunt/L5/terrain test rebaselines and the loop contact must stay stable through the
attitude sweep. This is why phase C (below) gates B.

## Validation plan (gating, per evidence policy)

L5 is currently "plausible-but-not-validated". To promote it to a model, evidence is
mandatory (figure / numeric table):

- **Flat-ground cross-model**: at low excitation L5 must match L3/L2 handling
  (yaw rate, ay) within tolerance — the spatial model reduces to the planar one on flat road.
- **Ballistic jump**: off a ramp, the airborne CG must follow projectile motion
  (parabola, energy-conserving) to numerical tolerance.
- **Loop completion**: the car completes a vertical loop above the critical speed and
  loses it below — the centripetal condition is emergent, not imposed.
- **Suspension (phase B)**: corner travel / camber gain under jounce matches the L4 corner
  DAE on the shared topology.

Until C passes, L5 stays labelled experimental in VALIDATION.md.

## Risks

- Loop stability without the denom hack: contact-frame slip must stay well-posed as the
  contact normal rotates through the loop (low tangential speed near the top).
- Constraint stiffness of the corner DAE on a free 6-DOF body (Baumgarte tuning).
- Contact normal estimation on arbitrary meshes (already provided by IContactProvider).

## Scope note

This is a multi-step, validation-gated build, not a relabel. The C++ stunt tests
(`test_stunt`, `test_l5_driving`, `test_terrain_l5`) will be rebaselined against the new
(physically-grounded) behaviour as part of phase A/C, since the current numbers are tuned to
the penalty/denom hack and have no validated reference.
