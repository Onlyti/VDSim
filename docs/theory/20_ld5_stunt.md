# 20. Ld5 — free 3D body (stunt: jump & vertical loop)

## Learning objectives

After this chapter you can:

1. State why L1–L3 (planar pose + quasi-static attitude) cannot represent a ramp
   jump or a vertical loop, and what extra degrees of freedom Ld5 adds.
2. Write the Newton–Euler equations of motion for a 6-DOF rigid body with a unit
   quaternion attitude, in the body frame VDSim integrates.
3. Describe the penalty contact model — penetration spring–damper normal force,
   the airborne condition, and how tire forces are built in the **wheel contact
   frame** so they remain valid when the car is inverted.
4. Derive the friction-free vertical-loop entry speed $v_{\min} \approx
   \sqrt{5 g R}$ and explain why the shipped loop is an *emergent* (not enforced)
   centripetal balance.
5. List the shipped tuning/limitations that make the loop a robust demo rather
   than a quantitatively validated maneuver.

## Prerequisites

- **Chapter 02** — rigid-body dynamics, inertia tensor, angular momentum.
- **Chapter 06** — Ld3 fourteen-DOF: the planar + quasi-static vertical model Ld5
  replaces with a true 3D body.
- **Chapter 11** — RK4 substep integration; quaternion renormalization.
- **Chapter 19** — LuGre tire (the default force law used at each contact).
- **Design note** — [`design/V0.4_SLOPE_JUMP_DYNAMICS.md`](../design/V0.4_SLOPE_JUMP_DYNAMICS.md)
  (contact architecture) and [`VALIDATION.md`](../VALIDATION.md) "Stunt" limits.

Source: `core/src/free_3d_dynamics.cpp` (`Free3DDynamics`, `level() == L5_Stunt`),
contact in `core/src/contact_providers.cpp` (`RampGround`, `LoopGround`).

---

## 20.1 Motivation — when planar attitude is not enough

L1–L3 carry pose as $(x, y, \psi)$ on a ground plane; body roll/pitch in L3 are
**quasi-static** perturbations about a static equilibrium, and the vertical state
is a small heave around ride height. That is correct for normal road driving but
breaks for stunts:

- A **ramp jump** needs a true world-$z$ with full gravity and an *airborne* phase
  (all tire forces vanish, the body follows a ballistic arc).
- A **vertical loop** puts the body at arbitrary attitude — inverted at the apex —
  so a small-angle attitude and a "down is $-z$" tire model are both invalid.

Ld5 (`Free3DDynamics`) is a free **6-DOF rigid body**: world position, a unit
quaternion attitude, body-frame linear and angular velocity, plus the four wheel
spin states. Flat L2/L3 road behaviour is unchanged — Ld5 is opt-in via
`level = L5` and a stunt scenario.

---

## 20.2 State and frames

| Symbol | Code | Frame | Meaning |
|--------|------|-------|---------|
| $\mathbf{p}$ | `state.position` | world | CG position |
| $\mathbf{q}$ | `state.orientation` | world←body | unit quaternion attitude |
| $\mathbf{v}$ | `state.velocity` | body | linear velocity |
| $\boldsymbol{\omega}$ | `state.angular_velocity` | body | angular velocity |
| $\omega_i$ | `state.wheel_spin[i]` | — | wheel spin, $i \in \{$FL,FR,RL,RR$\}$ |

Let $\mathbf{R} = \mathbf{R}(\mathbf{q})$ be the body→world rotation. Wheel $i$ has
a fixed body-frame mount $\mathbf{r}_i$ (track/wheelbase offsets, with vertical
$-(h_{cg}-R_w)$ so the hub sits a wheel radius below the CG). ISO 8855 RH applies;
wheel order FL=0, FR=1, RL=2, RR=3 (Chapter 01).

---

## 20.3 Equations of motion (Newton–Euler, body frame)

Collect the world-frame resultant force on the CG,

$$
\mathbf{F}^{w} = -m g\,\hat{\mathbf{z}}
   \;+\; \sum_{i} \mathbf{F}_i^{w}
   \;-\; \mathbf{R}\,(F_{\text{aero}} + F_{rr})\,\hat{\mathbf{x}}_b ,
$$

where $\mathbf{F}_i^{w}$ is the contact force of wheel $i$ (§20.4–20.5), aero drag
$F_{\text{aero}} = \tfrac12 \rho C_d A\, v_x |v_x|$ and rolling resistance act along
the body longitudinal axis. The **body-frame** linear acceleration follows from
differentiating $\mathbf{v}^{w} = \mathbf{R}\mathbf{v}$ in a rotating frame:

$$
\dot{\mathbf{v}} = \mathbf{R}^{\!\top}\frac{\mathbf{F}^{w}}{m}
                   \;-\; \boldsymbol{\omega}\times\mathbf{v}.
$$

The attitude obeys Euler's equation with a diagonal inertia
$\mathbf{I} = \mathrm{diag}(I_x, I_y, I_z)$ and the gyroscopic coupling:

$$
\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1}\!\left(
   \boldsymbol{\tau} - \boldsymbol{\omega}\times \mathbf{I}\boldsymbol{\omega}
\right),
\qquad
\boldsymbol{\tau} = \sum_i \mathbf{r}_i\times\mathbf{F}_i^{b}
                    \;+\; \Big(\textstyle\sum_i M_{z,i}\Big)\hat{\mathbf{z}}_b ,
$$

with $\mathbf{F}_i^{b} = \mathbf{R}^{\!\top}\mathbf{F}_i^{w}$ the body-frame wheel
force and $M_{z,i}$ the tire aligning moment about the body yaw axis. Position and
attitude integrate as

$$
\dot{\mathbf{p}} = \mathbf{R}\,\mathbf{v}, \qquad
\mathbf{q}_{n+1} = \big(\mathbf{q}_n \otimes \delta\mathbf{q}(\boldsymbol{\omega}\,h)\big)
                    \big/\,\lVert\cdot\rVert ,
$$

where $\delta\mathbf{q}$ is the axis–angle increment over step $h$ and the quaternion
is renormalized every step. Each wheel spin integrates $\dot\omega_i = T_{\text{net},i}/I_i$
(drive + brake − tire reaction; open-diff carrier coupling per `drivetrain_inertia.hpp`,
Chapter 06).

---

## 20.4 Contact: penalty normal force and the airborne condition

A contact provider returns, per wheel, a surface point, outward unit normal
$\hat{\mathbf{n}}$, penetration $\delta$, and an `is_valid` flag. The hub world
position is $\mathbf{p} + \mathbf{R}\mathbf{r}_i$ and the hub velocity
$\mathbf{v}^{w}_{\text{hub}} = \mathbf{R}(\mathbf{v} + \boldsymbol{\omega}\times\mathbf{r}_i)$.
The normal force is a clamped spring–damper,

$$
F_{z,i} = \mathrm{clamp}\!\left(k_t\,\delta_i
          + c_v\,(-\,\mathbf{v}^{w}_{\text{hub},i}\!\cdot\hat{\mathbf{n}}_i),
          \;0,\; F_z^{\text{cap}}\right),
\qquad c_v = 2\sqrt{k_t\, m/4},
$$

i.e. near-critical vertical damping. The wheel is **airborne** when
`!is_valid || penetration <= 0`; then $F_{z,i}=0$ and §20.5 contributes no shear
force, so the body is a projectile under gravity (and aero). The clamp at
$F_z^{\text{cap}}$ and the floor at $0$ are what let the same code carry both a
landing impact and a free-flight phase without branching the EOM.

---

## 20.5 Tire forces at arbitrary attitude

Because the body may be inverted, the wheel longitudinal/lateral axes are built in
the **contact plane**, not from a global "forward/right":

1. Project the body forward axis onto the contact plane:
   $\mathbf{t}_{\text{long}} = \widehat{\mathbf{f}_b - (\mathbf{f}_b\!\cdot\hat{\mathbf{n}})\hat{\mathbf{n}}}$,
   then rotate it by the road-wheel steer angle $\delta_i$ **about the contact
   normal** (Ackermann split on the front axle).
2. Lateral axis $\mathbf{t}_{\text{lat}} = \hat{\mathbf{n}}\times\mathbf{t}_{\text{long}}$.
3. Slip ratio $\kappa_i$ and slip angle $\alpha_i$ use the hub velocity projected
   onto $(\mathbf{t}_{\text{long}}, \mathbf{t}_{\text{lat}})$ and the spun wheel
   speed $R_w\omega_i$.
4. $F_x, F_y$ from the tire law (LuGre default, Chapter 19; MF96 + low-speed blend
   when LuGre is off), clipped to the friction ellipse $\sqrt{F_x^2+F_y^2}\le \mu F_z$.
5. World force $\mathbf{F}_i^{w} = F_x\,\mathbf{t}_{\text{long}} + F_y\,\mathbf{t}_{\text{lat}}
   + F_{z,i}\,\hat{\mathbf{n}}$.

Building the frame from $\hat{\mathbf{n}}$ is the key step that makes traction at
the loop apex behave the same as on flat ground.

---

## 20.6 Vertical loop: entry speed and emergent centripetal balance

For a loop of radius $R$ (hub circle), the classic friction-free minimum speed
comes from two conditions. At the **apex**, gravity must supply at least the
centripetal acceleration without the track pulling outward:

$$
\frac{m\,v_{\text{top}}^2}{R} \ge m g
\quad\Longrightarrow\quad v_{\text{top}}^2 \ge g R .
$$

Energy conservation from the bottom (height gain $2R$) gives
$v_{\text{bot}}^2 = v_{\text{top}}^2 + 2g(2R)$, so

$$
\boxed{\,v_{\min} = \sqrt{5 g R}\,}
$$

which is exactly `LoopGround::v_min()`. This is a *lower bound*: real entry needs
more for tire/rolling losses. VDSim does **not** impose the centripetal condition —
it falls out of the penalty normal force keeping the wheels on the circular
surface. Below $v_{\min}$ the car decelerates up the back side and falls away; it
is never teleported.

To keep the contact numerically robust on the closed loop, the shipped path
(`surface_id == 2`) applies: a higher $F_z$ cap ($18\,mg/4$ vs $6\,mg/4$ on flat),
a capped penetration ($\le 0.016$ m) with softened $k_t$, a track-tangent
longitudinal-slip estimate near the apex (where the projected forward speed is
small), and — with the guide rail off (default) — a radial penalty force that
pulls each hub toward the loop surface radius $R + R_w$. These keep the loop
stable but make it a *plausible demo*, not a validated load case (§20.8).

---

## 20.7 Numerical integration

`step()` lowers the control input to L4, then runs $N$ substeps of size
$h = \Delta t / N$ with $N = \lceil \Delta t / \texttt{max\_substep\_dt}\rceil$
(capped by `max_substeps`). Each substep is RK4 over $(\mathbf{p}, \mathbf{v},
\boldsymbol{\omega}, \omega_i)$; the quaternion is advanced from the averaged
$\boldsymbol{\omega}$ and renormalized in `apply()`. Stunt scenarios use a small
substep (high contact stiffness is stiff) — see `realtime_server` flags in the
v0.5 terrain note.

---

## 20.8 Shipped status and limitations

- **Default road sims are unaffected.** Ld5 is a separate plant (`create_stunt_dof`);
  `level = L5` + a stunt scenario is required to reach it.
- **Validation tier.** Jump (`Stunt.JumpAirborneInterval`, `JumpLandingNoSink`),
  loop completion and contact (`Stunt.FreeLoopCompletesLap`,
  `LoopMidArcKeepsContact`, `LoopSlipAngleFrontRearBalanced`,
  `LoopAccelWheelSpinBounded`), and flat-ground L5 driving (`tests/integration/
  test_l5_driving.cpp`) are green. There is **no** quantitative cross-validation of
  loop contact loads — the stiffened/capped contact and radial penalty are
  robust-tuned, not measured (see [`VALIDATION.md`](../VALIDATION.md) "Stunt").
- **Not yet (→ v0.5):** general curved/banked track (`CurvedGround`, banked oval)
  and a GUI panel to *author* stunt scenes; today the GUI renders loaded ramp/loop
  scenes only. See [`design/V0.5_TERRAIN_L5.md`](../design/V0.5_TERRAIN_L5.md).

---

## Cross-references

- [`06_ld3_fourteen_dof.md`](06_ld3_fourteen_dof.md) — planar + quasi-static vertical model.
- [`19_lugre_dynamic_tire.md`](19_lugre_dynamic_tire.md) — default contact force law.
- [`design/V0.4_SLOPE_JUMP_DYNAMICS.md`](../design/V0.4_SLOPE_JUMP_DYNAMICS.md) — contact target architecture.
- [`design/V0.5_TERRAIN_L5.md`](../design/V0.5_TERRAIN_L5.md) — Ld5 generalized to arbitrary terrain.
