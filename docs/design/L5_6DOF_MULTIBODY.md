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

- **A** — 6-DOF body + contact-frame slip + inverted tire (suspension still rigid/penalty).
  Closes the L5 inversion gap; L5 becomes a real 6-DOF vehicle with shared tires.
- **B** — attach the L4 corner DAE to the 6-DOF body (true spatial multibody).
- **C** — validation (below).

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
