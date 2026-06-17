# L5 — rigorous MBD formulation (14DOF structure lifted to a 6-DOF free body)

Status: DESIGN (to implement). Supersedes the reduced-coordinate 10-DOF coupled
solve (`L5_MBD_DYNAMIC_COUPLING.md` + the WIP in `L5_MBD_COUPLED_WIP.patch`), whose
root flaw is documented below and confirmed by Cursor's MBD review + our diagnosis.

## Why the current 10-DOF coupled solve is wrong (root cause)

The unsprung mass is parameterized as a body-RELATIVE coordinate:
`x_u,i = p + R(rb_i + w_i(theta_i))`, and the suspension spring potential is
`U = U(theta_i)` — a function of the travel coordinate ONLY. Therefore
`dU/d(body p,R) = 0`: the spring exerts NO generalized force on the sprung body.
The body only feels the spring through the inertial off-diagonal block
`M_rq·thetä`, which VANISHES in the quasi-static limit (`thetä -> 0`). Net effect:
in steady cornering the spring's roll-restoring moment is never delivered to the
body — the roll is held only by the tire vertical-load transfer (which the soft
travel relieves), giving ~5x too little roll stiffness and rollover at ~0.15 g.

The fix is a coordinate change: the unsprung vertical must be an INERTIAL
(ground-referenced) coordinate `z_u,i`, so the spring deflection
`delta_i = z_corner,i(p,R) - z_u,i` depends on the body pose and the spring
delivers a direct roll/pitch/heave wrench to the body. This is exactly the
(proven-stable) Ld3 14-DOF structure (`fourteen_dof_dynamics.cpp:356-422`) — here
lifted from a small-angle planar body to a full 6-DOF free body.

## System and state

- Sprung body B: mass `m_s`, body-frame inertia `I_b`, pose `(p, R)`, velocity
  `(v_body, omega_body)`. Full 6-DOF (no small-angle assumption).
- Per corner i (FL,FR,RL,RR): unsprung point mass `m_u,i` with ONE inertial
  vertical DOF `z_u,i` (world z of the wheel centre). Its horizontal position is
  carried rigidly by the body (the links are on the chassis); the linkage's
  lateral/fore-aft wheel travel and the wheel orientation (toe/camber) come from
  the DAE `travel_maps` as a function of the vertical travel — used for the tire
  force DIRECTION and contact point, not for an extra unsprung DOF.
- Wheel spin: 4 DOF, integrated separately as today.

Integration state (per RK4 stage): `p, R(quat), v_body, omega_body, z_u_i, zu_dot_i`
(+ wheel spins). NOTE: this REPLACES `susp_compression/velocity` semantics — those
fields now hold `z_u` deviation from static and `zu_dot` (still metres, contract
preserved: suspension travel is the real vertical wheel travel).

## Mass matrix: BLOCK DIAGONAL (no 10x10 solve)

Because the unsprung has only a vertical inertial DOF coupled to the body purely by
the spring/damper FORCE (not by a shared coordinate), the mass matrix decouples:

```
sprung body:   M_body * udot_body = F_body            (standard 6-DOF Newton-Euler)
unsprung i:    m_u,i * zu_ddot_i  = Fz_i - F_susp_i - m_u,i*g   (4 scalar ODEs)
```

The unsprung mass is small; its horizontal motion (rigid with the body) is folded
into the body translational mass and inertia (parallel axis), matching how Ld3
lumps the unsprung into the planar total. So: drop the LDLT 10x10 solve; the body
uses the same explicit Euler integration as the penalty path.

## Kinematics

```
mount_i        = p + R*rb_i                 wheel/strut mount, world  [m]
z_corner_i     = mount_i . ez               world-vertical of the mount
v_corner_i     = (v_world + omega_w x (R rb_i)) . ez    its vertical velocity
delta_i        = z_corner_i - z_u_i         suspension deflection (compression>0 when corner sinks toward wheel)
ddelta_i       = v_corner_i - zu_dot_i
```

Linkage (from DAE `travel_maps(z_v=travel_i)`, travel_i = z_u_i - z_u_static_i):
```
wheel lateral/fore-aft offset, toe_i, camber_i, motion ratio MR_i = dl_spring/dz_v
contact patch  x_c,i  (one rolling radius below the linkage wheel centre)
```

## Forces

Spring + damper (wheel-rate; motion ratio MR folds in later, MR=1 first cut):
```
F_susp_i = F_preload_i + k_s,i*(delta_i) + c_s,i*ddelta_i + F_stop_i
```
(`delta_i` measured from the static ride so `F_preload` balances the static load.)

Tire NORMAL on the unsprung (vertical), tire TANGENTIAL on the body at the patch:
```
Fz_i      = k_tire*(pen_i) + c_tire*pen_dot_i ,  pen_i = road_z_i - (z_u_i - R0)
F_tan_i   = Fx_i * t_long + Fy_i * t_lat        (from the inverted tire, dir from DAE toe/camber)
```

Body wrench (world):
```
F_body  = -m_sum*g*ez + aero + rolling
        + sum_i  F_susp_i * ez            (spring pushes the chassis up at each mount)
        + sum_i  F_tan_i                  (lateral/longitudinal tire force, rigid horizontal link)
tau_body= sum_i (R rb_i) x (F_susp_i*ez)  (-> heave/ROLL/pitch: this is the missing term)
        + sum_i  rcp_i x F_tan_i          (yaw + the lateral overturning)
        + sum_i  mz_i * ez                (self-aligning)
```
Roll moment from the springs `sum_i (R rb_i)_y * F_susp_i` is the direct restoring
that the old solve lacked. Standard 6-DOF: `m_sum*vdot = F_body`,
`I_w*omegadot + omega x (I_w omega) = tau_body`.

Unsprung (each corner, scalar):
```
m_u,i * zu_ddot_i = Fz_i - F_susp_i - m_u,i*g
```

## Energy

```
d/dt ( T_body + T_unsprung + U_grav + U_spring ) = P_damper + P_tire_contact
```
Springs/dampers are explicit two-point force elements (body mount <-> unsprung), so
their virtual work depends only on the relative rate `ddelta_i` — energy-consistent
by construction (the same property Ld3 has and the old reduced model lost).

## What is reused

- DAE `travel_maps` / `pose_at_travel` (committed) for the linkage wheel path,
  toe/camber, motion ratio — the real link kinematics are kept.
- The Pacejka inverted-tire path, contact providers, RCPC contact-patch lever.
- The penalty-path body Euler integration (now used for the strut path too).

## What is removed/replaced

- The 10x10 `M u̇ = Q - b` LDLT solve and the `M_tr/M_rq/M_tq` coupling blocks
  (the reduced-coordinate inertial coupling) — replaced by block-diagonal body
  Euler + 4 scalar `z_u` ODEs.
- `Gamma_susp` on a body-relative travel DOF — replaced by the explicit two-point
  spring wrench on body + unsprung.
- The penetration `pen = pen_rigid - comp*(ez.n)` correction — now `pen` is a plain
  function of the inertial `z_u`.

## Acceptance

- Static settle: `z_u` and body settle, `sum Fz = weight`, zero accel.
- Flat steady cornering: stable to a physical limit (rollover threshold > ~0.7 g),
  `sum Fz/W -> 1`, roll a few deg/g; roll stiffness ~ `axle_roll_stiffness()` series
  value (~26 kN·m/rad), NOT ~5.7.
- Re-pass `FlatCrossModelMatchesL2`, `FreeLoopCompletesLap`, `CornerCamberMatchesL4Dae`,
  ride/jump/loop-critical, all `L5Strut*`/`Stunt*` tests.
- Loop energy audit: `E_total` conserved (damper + slip only).

## Open / second cut

- Unsprung horizontal inertia coupling (folded into the body now) — make exact later.
- Spring along the real strut axis with motion ratio `MR(travel)` instead of
  wheel-rate vertical.
- `s_c` contact-patch travel sensitivity for anti-dive/squat (vs the `R*w'` approx).
