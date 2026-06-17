# L5 — free-3D inertial unsprung (general-surface, energy-consistent strut)

Status: IMPLEMENTED (`core/src/free_3d_dynamics.cpp`, strut path). Supersedes both the
reduced-coordinate 10-DOF coupled solve (`L5_MBD_DYNAMIC_COUPLING.md`) and the
world-vertical `z_u` design (`L5_MBD_RIGOROUS.md`). All 377 tests green.

## Why this architecture is forced

Requirements: (1) arbitrary-surface contact (loop / bank / hill — tire normal along the
real surface normal, body gets centripetal), (2) quasi-static roll stiffness from the
spring, (3) structural energy consistency, (4) no world-vertical assumption.

Key fact: the spring delivers a roll moment to the body ONLY if its deflection depends on
the body pose.
- Reduced coordinate (suspension travel `theta` or strut stroke `s` as the unsprung DOF):
  deflection = the independent DOF, so `dU/d(body) = 0`. The body feels the spring only
  through the inertial off-diagonal `M_rq`, which vanishes in the QS limit -> ~5x too little
  roll stiffness, ~0.15 g rollover. (This killed the 10-DOF coupled solve.)
- World-vertical `z_u` (Ld3 lifted): deflection `z_corner(pose) - z_u` IS pose-dependent, so
  QS roll works, BUT routing the tire NORMAL only into a world-z DOF gives zero body
  centripetal on a loop side / bank -> fails general-surface acceptance.

Discarding world-z (requirement 4) leaves exactly one option that satisfies all four: each
unsprung is a genuine FREE 3-D inertial particle. Then the strut deflection
`delta = (mount - x_u).u_hat` depends on the body pose (mount, `u_hat`) -> QS roll; the tire
acts on the free particle along the real normal -> general surface; every body<->wheel force
is a two-point element -> energy-consistent.

## Model

Per corner i, world frame. `u_hat = R.col(2)` (strut axis = body up; MR = 1 first cut).
- DOF (in `State`): `unsprung_pos[i]` (x_u), `unsprung_vel[i]` (v_u). `susp_compression /
  velocity` are now DERIVED strut-axis travel `(x_u-mount).u_hat` (DAE / FMI / cosim
  contract preserved).
- mount = `p + R*rb_i`; `v_mount = R*(v_body + omega x rb_i)`.
- Strut (soft, along u_hat): `F_susp = F_preload + k*comp + c*comp_dot + F_stop`,
  `comp = (x_u-mount).u_hat`. Pushes wheel down (`-F_susp u_hat`), body up (`+F_susp u_hat`).
- Bushing (stiff perpendicular = rigid links, penalty): `F_bush = -k_link*rel_perp -
  c_link*vrel_perp`, `k_link = 1e8 N/m`, critically damped.
- Tire on the wheel: the contact provider samples the surface AT the wheel particle
  (`wheel_world_positions` returns `unsprung_pos` when the model maintains it), so
  `contacts[i].penetration` is the true penetration of the actual wheel on any curved surface
  — no planar extrapolation from the rigid hub. `Fz = k_tire*pen + c_t*pen_dot`. If the
  provider reports `penetration <= 0` the wheel is off the surface -> `Fz = 0` (this is what
  makes a too-slow car cleanly leave a loop/bank instead of sticking via a false contact).
  When in contact, a within-step correction `pen = pen_rigid - (x_u - mount_query).n`
  (mount_query = the wheel position the once-per-step query sampled) tracks the small wheel
  motion through the substeps. Slip from `v_u`. Force `Fx*t_long + Fy*t_lat + Fz*n`, on the
  wheel. `unsprung_pos` is the canonical wheel-centre world position, maintained every step on
  BOTH paths (strut = integrated particle, penalty = rigid with the body), so the contact is
  always sampled at the real wheel — no path-dependent guards.
- Unsprung Newton: `m_u * a_u = F_tire - F_susp*u_hat + F_bush + m_u*g_world`. No
  frame-coupling shortcut — x_u is a true inertial particle.
- Body gets ONLY the mount connection reaction `F_conn = -(F_strut_on_u + F_bush)` (+ its
  moment `rb x R^T F_conn`, + tire `mz` on yaw). The tire reaches the chassis THROUGH the
  bushing/strut. Body mass = `mass - sum(unsprung)` in ALL directions; body + 4 unsprung sum
  to the total mass, so flat handling still matches the planar L2/L3 models (verified by
  `FlatCrossModelMatchesL2`).

## Energy

All body<->wheel forces are two-point spring/dampers -> conservative (springs) / dissipative
(dampers) by construction. Evidence (`apps/jump_demo/strut_demo_dump`, full ledger
= body KE + 4 unsprung KE + rot KE + grav PE(all) + strut PE):
- Flat steady steer: `E_tot` strictly monotonic decreasing (0/300 samples increase) — no
  injection. The structural leak of the old B1 strut (~85 kJ loop injection, dt-INDEPENDENT)
  is gone.
- Loop coast (throttle 0) with the ONCE-PER-STEP frozen contact: small NET injection at some
  speeds (e.g. +4.9 kJ over 3.3 turns at V0=1.3) — a real residual from the stale contact
  normal as the body rotates within the step.
- Loop coast with PER-SUBSTEP contact re-query (`free_3d_attach_contact_provider`): cleanly
  DISSIPATIVE at every speed (net -18 to -40 kJ over ~3-4 turns) and the load-correlated
  ledger oscillation collapses ~20x (peak +18 kJ -> +0.9 kJ). So the residual WAS the
  frozen-contact error, and per-substep re-query removes it. Recommended mode for sims on
  curved surfaces; the acceptance tests pass either way (they feed a frozen ContactArray).

## Acceptance (377/377)

Flat cross-model = L2, ballistic jump (g within 5 %), loop critical speed emergent, corner
camber = L4 DAE, settle sum Fz = weight, heave ride frequency, terrain bank/hill, all
`L5*` / `Stunt*`. `Stunt.FreeLoopCompletesLap` attitude check rebaselined from the
instantaneous final pitch (timing-sensitive at a fixed step count) to the PEAK pitch over
the lap.

## Per-substep contact re-query (implemented)

`free_3d_attach_contact_provider(dyn, provider)` makes the strut path re-query the contact at
each internal substep pose (refreshing normal + penetration + the frozen rigid-hub
reference) instead of holding the once-per-step query. Non-owning, opt-in, default off
(null -> use the passed ContactArray, unchanged). Removes the loop frozen-contact energy
residual (see Energy). Sims/cosim on curved surfaces should attach their provider.

## Link stiffness / why not a true hard constraint

The perpendicular link is a stiff penalty bushing, default `kLinkStiffness = 1e8 N/m`
(~40 um compliance under cornering load — effectively rigid; was 1e7 = 0.4 mm).
A TRUE hard constraint is deliberately NOT used: in the k->inf limit the wheel collapses onto
the strut line (`x_u = mount + s*u_hat`), making the spring deflection the independent
coordinate -> the reduced-coordinate loss of QS roll stiffness (the same failure as the old
coupled solve). The stiff penalty is that limit minus a few um and still delivers the link
reaction to the body, so roll stiffness is preserved. Swept stable to 1e9 (4 um, all tests
green); 1e10 is RK4-unstable (`omega*dt > 2.8`). Env override `VDSIM_KLINK`. Measured roll
gradient 2.73 deg/g (realistic), settled (std 1e-4).

## High-fidelity linkage geometry (implemented when a corner DAE is attached)

The strut bushing constrains the wheel particle to the REAL hardpoint travel path from the
committed DAE `travel_maps`, not a straight body-up line:
- Per corner (once per outer step, like the toe/camber update): query `travel_maps(z_v)` for
  the wheel-centre migration `Δw = w(z_v) - w(0)` and the path tangent `w'(z_v)` (y mirrored
  on the right side). `w0` is cached at attach; the absolute DAE/free_3d frame origins differ
  but the axes align, so only the migration (frame-independent) is used.
- Bushing target = `mount + R*Δw`, penalised PERPENDICULAR to the tangent `R*w'`. Because the
  tangent carries the linkage's fore-aft/lateral slope, tyre Fx/Fy produce a vertical
  reaction through the constraint -> anti-dive/squat and roll-centre migration emerge from the
  geometry (generalized force on travel = `F_tyre . tangent`). toe/camber already flow from
  the DAE. No hardpoints attached -> straight body-up strut (constant motion ratio).
- Verified: static (no brake) DAE == straight (no spurious coupling); MacPherson front
  (w'.x~0) shows no fore-aft coupling; trailing-arm rear (w'.x=0.19) shows strong travel
  coupling under braking, magnitude ~ `Fx*w'.x/k_wheel` — emergent from the hardpoints, scales
  with the real path slope. All 377 tests green.

## Progressive coil rate from spring hardpoints (implemented)

When the DAE exposes the spring eye-to-eye geometry, the strut spring is a real coil-over,
not a constant wheel rate:
- `IHardJointDaeModel::spring_length(z_v)` returns the eye-to-eye length from the spring
  hardpoints (MacPherson strut top->knuckle `sk`; DoubleWishbone `spring_damper` chassis->LCA
  point, which swings with the LCA). `travel_maps` finite-diffs it -> `spring_len` and
  `motion_ratio = dl/dz_v`. Topologies without spring hardpoints return <0 (fallback).
- free_3d: `F_susp = k_coil*(l - l_free)*MR`, linearised in comp within the step (local wheel
  rate = `k_coil*MR^2`). `k_coil = ks/MR0^2` and `l_free` are cached at attach so the STATIC
  load and rate match the wheel-rate model, then the rate turns progressive + bump/rebound
  asymmetric. No spring geometry -> wheel-rate fallback (`F_preload + ks*comp`).
- Verified (real hardpoints): MacPherson wheel rate -7.0% (rebound 60mm) .. +9.7% (bump);
  DoubleWishbone -6.9% .. +11.7%; DW MR0=-0.53 (inboard spring) -> k_coil~3.6*ks. Static load
  matched, all 377 tests green.

## Done (minor follow-ups, 2026-06-17)

- Progressive damper: the coil damper scales with `MR^2` too (`c_eff = cs*(MR/MR0)^2`), matched
  to `cs` at static. Bump/rebound stops stay on the wheel travel (comp) — they engage at the
  real travel limit, so that is correct.
- Per-substep contact re-query is now the default in every `SimSession`-based run (cosim,
  headless, batch): the session attaches its ground provider to the dynamics after init
  (no-op for non-free_3d). Standalone acceptance tests still drive the frozen ContactArray.

## Gyroscopic wheel-spin coupling (implemented)

Each spinning wheel carries spin angular momentum `H_i = I_wheel*omega_spin` along the lateral
(body-y) axis. A body rotating at `omega` must supply `omega x H`, so the wheels exert the
reaction `-omega x H` on the body, added to `tau_body`. This couples yaw<->roll at speed
(yawing a fast-spinning wheel set produces a roll moment, and vice versa). The spin axis is
body-lateral, tilted by the steer angle (rotated about body-z, so a steered spinning wheel's
momentum points partly fore-aft); camber tilt is omitted (smaller, sign-sensitive). Verified
airborne (no tyre forces): a yaw rate + spinning wheels develops a roll rate; non-spinning
does not (`Stunt.GyroscopicWheelSpinCouplesYawToRoll`).

## Open / next cut

- KC compliance under load (bushing deflection -> compliance steer/camber). Attempted with the
  existing lumped model (`compliance_targets_rad`) + default bushings, but it produced ~1.5
  deg/g of compliance steer — ~5-10x too large and unvalidated — so it was reverted. Needs the
  Adams/KC reference (Tier 1) to set the bushing rates and validate sign/magnitude before
  shipping; tuning it blind is a guess.
- Gyro camber-axis tilt and the wheel spin-UP reaction. The spin-up reaction is really the
  DRIVE-torque reaction routed through the drivetrain mounts (a free wheel's bearing carries no
  spin-axis torque), so it belongs to the drivetrain model, not the unsprung.
