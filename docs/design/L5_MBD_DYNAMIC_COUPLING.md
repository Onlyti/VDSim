# L5 — dynamic MBD suspension coupled to the 6-DOF body (EoM derivation)

> **SUPERSEDED (2026-06-17)** — the reduced-coordinate 10-DOF coupled solve here gave no
> quasi-static roll stiffness (dU/d(body)=0) and was abandoned. The shipped L5 model is
> [`L5_MBD_FREE3D_UNSPRUNG.md`](L5_MBD_FREE3D_UNSPRUNG.md). Kept for the EoM derivation.

Status: DESIGN / derivation for review (not yet implemented). Supersedes the B1
"spatial strut" (lumped 1-DOF vertical unsprung + one-way pseudo-force coupling),
whose semi-coupled solve injected energy in the loop (~85 kJ; see below).

Principle (per the high-fidelity requirement): the MODEL may be simplified (each
corner is a 1-DOF linkage mechanism), but the chosen model's equations of motion
must be solved EXACTLY and CONSISTENTLY — no lumped pseudo-forces, no one-way
coupling, no direction-dependent mass. Energy must be conserved to numerical
precision (only the damper and tire slip dissipate).

## Why B1 was wrong (numerically, not as a model)

B1 integrated each corner as a vertical point-mass slider with
`comp'' = (Fz - F_susp)/m_u - g.ez - a_body.ez`, where `a_body.ez` was the body CG
acceleration injected as a one-way pseudo-force (the unsprung's reaction was never
fed back to the body), the corner frame acceleration neglected its rotational part
(omega x (omega x r), Coriolis), and the body used a direction-dependent mass
(m_sprung along ez, m_total in-plane). On flat ground (small body rotation, small
body accel) these are negligible and all tests pass. In the loop the body rotates
360 deg under 6-13 g, and the inconsistency pumps ~85 kJ (verified dt-independent
at 2e-4 and 2e-5; ~69 kJ from the vertical/strut path, ~15 kJ from the tangential
tire force). The fix is to solve the true coupled rigid-body dynamics.

## System and generalized coordinates

- Sprung body B: mass `m_s`, body-frame inertia `I_b` about its CG, pose `(p, R)`
  with `p` the world CG position and `R` the body->world rotation.
- Per corner `i` (i = FL,FR,RL,RR): a 1-DOF linkage with generalized coordinate
  `theta_i` (the corner DAE's travel coordinate — the LCA/lower-aft revolute angle).
  The DAE maps `theta_i` to the body-frame wheel-centre path `w_i(theta_i)` and the
  wheel pose (toe/camber/caster).
- Unsprung model (stated approximation): a point mass `m_u,i` at the wheel centre
  `w_i(theta_i)`, plus an optional scalar link rotational inertia `J_lnk,i` about the
  travel coordinate. (The wheel spin remains a separate DOF as today; it does not
  couple into the suspension travel inertia here.)

Generalized velocity (10 DOF):

```
u = [ v ; omega ; thetadot ]   in R^10
    v        = world CG velocity of the sprung body        (3)
    omega    = world angular velocity of the sprung body   (3)
    thetadot = [thetadot_FL, thetadot_FR, thetadot_RL, thetadot_RR]  (4)
```

## Kinematics (from the DAE)

The DAE supplies, per corner, in the BODY frame:

```
w_i(theta_i)      wheel-centre position            [m]
w'_i  = dw_i/dtheta_i      (travel direction)      [m/rad]   (finite-diff in DAE)
w''_i = d2w_i/dtheta_i^2   (travel curvature)       [m/rad^2] (finite-diff)
```

World quantities:

```
r_i  = R w_i           lever arm CG->wheel (world)
s_i  = R w'_i          travel direction (world)
x_i  = p + r_i         unsprung world position
xdot_i = v + omega x r_i + s_i thetadot_i
```

Acceleration, split into the u-dot part and the velocity-dependent bias:

```
xddot_i = [ vdot + omegadot x r_i + s_i thetaddot_i ]            (depends on udot)
        + [ omega x (omega x r_i) + 2 omega x (s_i thetadot_i)
            + (R w''_i) thetadot_i^2 ]                            (bias a_bias,i)
```

## Kinetic energy and mass matrix

```
T = 1/2 m_s |v|^2 + 1/2 omega . I_w omega + sum_i 1/2 m_u,i |xdot_i|^2
                                            + sum_i 1/2 J_lnk,i thetadot_i^2
I_w = R I_b R^T   (world inertia of the sprung body)
```

Writing `T = 1/2 u^T M(q) u`, with `[a]x` the skew (cross-product) matrix:

```
M = [ M_t    M_tr    M_tq  ]
    [ M_tr^T M_r     M_rq  ]
    [ M_tq^T M_rq^T  M_q   ]

M_t  = (m_s + sum_i m_u,i) I_3                              (3x3)
M_tr = - sum_i m_u,i [r_i]x                                 (3x3)
M_tq = [ ... m_u,i s_i ... ]   (column i)                   (3x4)
M_r  = I_w - sum_i m_u,i [r_i]x [r_i]x                      (3x3)
M_rq = [ ... m_u,i (r_i x s_i) ... ]   (column i)           (3x4)
M_q  = diag( m_u,i |s_i|^2 + J_lnk,i )                      (4x4)
```

`M` is symmetric positive-definite (10x10); the off-diagonal blocks are the
floating-base <-> suspension-travel inertial coupling that B1 omitted.

## Bias force (centrifugal / Coriolis / gyroscopic)

From the velocity-dependent acceleration terms,

```
a_bias,i = omega x (omega x r_i) + 2 omega x (s_i thetadot_i) + (R w''_i) thetadot_i^2

b_t      = sum_i m_u,i a_bias,i                                   (3)
b_r      = omega x (I_w omega) + sum_i m_u,i (r_i x a_bias,i)      (3)
b_q,i    = m_u,i (s_i . a_bias,i)                                 (4)
b = [b_t; b_r; b_q]
```

## Generalized forces  Q = [F; tau; Gamma]

Gravity (`g_vec = (0,0,-g)`):

```
F_grav    = (m_s + sum_i m_u,i) g_vec
tau_grav  = sum_i m_u,i (r_i x g_vec)
Gamma_grav,i = m_u,i (s_i . g_vec)
```

Suspension spring + damper + bump/rebound stop — acts only on the travel DOF
(internal; the body reaction is produced by the M coupling). Using a wheel-rate
spring on the vertical travel `z_i = w_i(theta_i).z` (body frame), with `J_z,i =
w'_i.z`:

```
F_susp_i = F_preload_i + k_s,i (z_i - z_i0) + c_s,i zdot_i + F_stop_i
zdot_i   = J_z,i thetadot_i
Gamma_susp,i = - F_susp_i * J_z,i     (generalized force on theta_i)
```

`F_stop_i` = stiff bump/rebound stop as before (force element, not a clamp).
`F_preload_i` set so the static corner load gives the design ride travel.

Tire force `F_tire,i` (world; normal + tangential from the inverted tire) applied at
the CONTACT PATCH `x_c,i` (NOT the wheel centre), with lever `r_c,i = x_c,i - p` and
contact-patch travel sensitivity `s_c,i = R d w_c,i/dtheta_i`:

```
x_c,i  = wheel_centre_i - R0_eff * (wheel-down)   (patch ~ one tyre radius below hub)
F_tire     += F_tire,i
tau_tire   += r_c,i x F_tire,i
Gamma_tire,i = s_c,i . F_tire,i
```

Applying the force at the contact patch (not the hub) is what makes anti-dive /
anti-squat / jacking EMERGE from geometry: the linkage travel path `w_i(theta_i)`
has a fore-aft component (`dw/dtheta` is not purely vertical), so the longitudinal
tyre force `Fx` does work on `theta_i` via `s_c,i . F_tire,i` — i.e. braking/accel
presses or lifts the corner through the real side-view swing-arm geometry. No
separate anti-dive term is needed. (Refinement: the brake caliper reaction torque on
the knuckle — the outboard-brake share of anti-dive — is an additional term on top
of the contact-patch geometry; deferred.)

The tire NORMAL force uses the emergent penetration (generalises B1):

```
pen_i = pen_nominal,i - [ R (w_i(theta_i) - w_i(0)) ] . n_i
```

so the contact, the travel `theta_i`, and the body pose are all consistent — the
travel EMERGES from the load instead of being prescribed.

## Equation of motion and integration

```
M(q) udot = Q(q,u) - b(q,u)
```

Solve the 10x10 SPD system each RK4 stage (Cholesky / LDLT); then integrate
`p, R(quaternion), v, omega, theta_i, thetadot_i` and the wheel spins as today.
Cost: one 10x10 solve x 4 stages x substeps — negligible.

This is the exact Lagrangian dynamics of the chosen model, so along any trajectory

```
d/dt ( T + U_grav + U_spring + U_stop ) = P_damper + P_tire_contact
```

i.e. mechanical energy changes ONLY through the suspension damper and the tire
contact (slip dissipation + the work the road does) — there is no frame-coupling /
mass-anisotropy leak. This is the acceptance test (energy audit) for the loop.

## What is reused vs replaced

- REUSED: the corner DAE kinematics `w_i(theta_i)`, Jacobian `w'_i`, curvature
  `w''_i`, link inertia `J_lnk,i = corner_inertia_about_axis`, and the wheel pose
  (toe/camber) for the inverted tire. The DAE stops being "prescribed-travel": we
  read its kinematic maps and integrate `theta_i` ourselves from the load.
- REPLACED: the B1 lumped vertical unsprung, the anisotropic body mass, the one-way
  `a_body.ez` pseudo-force, and the apply()-level travel clamp. `comp/comp_dot`
  state is reused to store `theta_i/thetadot_i` (or a dedicated field).
- UNCHANGED: the default penalty path (`l5_spatial_suspension = false`); the
  generalized LoopGround contact; the wheel-spin / drivetrain dynamics.

## Assumptions (reviewed)

1. Unsprung = point mass `m_u` at the wheel centre on the DAE travel path, + optional
   scalar travel inertia `J_lnk`. REFINEMENT (later): model the unsprung as a rigid
   body with its CG path AND full 3x3 inertia tensor `I_u` (from the DAE knuckle
   orientation vs travel), so the unsprung rotational KE `1/2 omega_u . I_u omega_u`
   is exact instead of the scalar `J_lnk`. ("link tensor inertia" = this full `I_u`.)
2. One travel DOF per corner — exact now: no compliance (bushings rigid) and steering
   not modelled. Adding steer/compliance would add DOFs later.
3. Wheel spin stays a separate DOF handled on the tyre side (slip/drivetrain). Its
   gyroscopic coupling into suspension travel is neglected (2nd order). The
   longitudinal-force JACKING (anti-dive/squat) is captured via the contact-patch
   force geometry above, NOT via the spin DOF.
4. Spring = wheel-rate spring on the vertical travel component. NEXT TARGET: replace
   with an along-the-spring-line force and a motion ratio MR(theta) (the spring is
   structured as `Gamma = -F_susp * (dl_spring/dtheta)` so MR drops straight in).
5. `w'_i, w''_i` from the DAE by finite difference (already used internally).

## Validation / acceptance

- Energy audit (loop, throttle=0): `E_total` conserved to < O(0.1%) over the run
  (vs +85 kJ today). The car cannot exceed its energy-allowed height.
- Re-pass: settle (Fz=weight, comp~0), heave ride frequency, flat cross-model vs L2,
  ballistic jump (g within tol), loop critical speed (emergent), corner camber vs L4
  DAE. All current `L5Strut*` / `L5StrutValidation*` / `Stunt*` tests.
