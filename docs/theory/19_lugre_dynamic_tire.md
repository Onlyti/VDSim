# 19. Tire — LuGre / brush-dynamic model

## Learning objectives

After this chapter you can:

1. Explain why quasi-static Pacejka becomes ill-conditioned as $|v_x| \to 0$
   (slip ratio / slip angle singularities) and what physical behaviour is missing.
2. Derive the LuGre bristle state equation and identify the roles of $z$,
   $v_r$, $\sigma_0$, $\sigma_1$, $\sigma_2$, and the breakaway envelope $g$.
3. Show how MF96 can serve as $g(\cdot)$ so steady sliding recovers the same
   slip curve as Chapter 03.
4. Contrast **presliding (stick)**, **micro-slip**, and **macro-slip (sliding)**
   operating points without switching models.
5. Describe VDSim's shipped adoption: decoupled long/lat LuGre, asymmetric
   breakaway floors, peak-axis friction ellipse, RK4 $z$ sub-advances, ISO
   lateral sign, and opt-in via `lugre.enabled`.

## Prerequisites

- **Chapter 01** — slip angle $\alpha$, slip ratio $\kappa$, ISO 8855 signs.
- **Chapter 03** — Pacejka MF96 pure slip, friction ellipse, relaxation length.
- **Chapter 11** — substep integration; why stiff states need implicit updates.
- **Design note** — [`docs/design/LOW_SPEED_HANDLING.md`](../design/LOW_SPEED_HANDLING.md)
  (kinematic blend alternative in VDSim default config).

---

## 19.1 Motivation — one model from rest to slide

Chapter 03 treats tire force as an **algebraic** map $(\kappa, \alpha, F_z, \mu)
\mapsto (F_x, F_y)$. That is correct in the **steady sliding** regime: the contact
patch has reached a quasi-static deflection distribution and the Pacejka curve is
an empirical fit to that limit.

Two gaps appear at low speed:

1. **Singularity** — $\kappa = (R\omega - v_x)/v_x$ and $\alpha =
   \mathrm{atan2}(v_y, v_x)$ divide by $v_x$. Any numerical floor on $v_x$
   injects noise into forces and yaw moment at parking speeds.
2. **Missing physics** — before the patch slides, tread elements deflect
   elastically (brush / bristle behaviour). Static hold on a grade, stick-slip,
   and presliding hysteresis live in this regime; they are not in a pure slip
   curve.

The **brush model** is the physical origin of the slip curve: micro-elements in
the adhesion zone deflect proportionally to slip; in the sliding zone they
saturate at $\mu F_z$. Integrating that picture over the patch yields a
Pacejka-shaped steady state. Making bristle deflection a **dynamic state** (LuGre)
unifies rest, presliding, and sliding in one ODE instead of blending to a
kinematic bicycle at low speed.

```
  v_r ≈ 0, |z| small          v_r large, z → g/σ₀
  ─────────────────           ─────────────────────
  F ≈ σ₀ z  (elastic stick)   F ≈ g(v_r)  (MF96 envelope)
        presliding                    macro-slip
```

---

## 19.2 Assumptions

| Assumption | Meaning | Breaks when |
|---|---|---|
| Point-contact bristle | one $z$ per force component per wheel | patch length dynamics, carcass modes |
| $g$ from steady MF96 | breakaway force envelope from Ch.03 | need temperature / wear in $g$ |
| Decoupled long / lat (VDSim) | separate $z_x$, $z_y$; ellipse rescale after | combined brush (Deur 2004) preferred |
| Rigid wheel / flat road | $v_r$ from rigid kinematics | large compliance, obstacle impacts |
| No camber-induced $v_r$ coupling | camber enters MF96 $g$ only via Ch.03 inputs | full 3D brush |
| Host integrates $z$ | `ITireModel` stays stateless | — |

---

## 19.3 Physical picture — brush and slip velocity

Imagine one representative tread element (bristle) in the contact plane.

- **Stick / presliding:** the element tip deflects by $z$ [m] while the base
  moves with the wheel; force is elastic $F \approx \sigma_0 z$.
- **Sliding:** deflection saturates; force approaches the friction limit $g$.
- **Relative slip velocity** $v_r$ [m/s] drives how fast $z$ tries to follow the
  kinematic mismatch between wheel and ground.

For one wheel in the wheel frame:

| Direction | Slip velocity $v_r$ (VDSim) |
|---|---|
| Longitudinal | $v_{r,x} = R\omega - v_{x,\mathrm{wheel}}$ |
| Lateral | $v_{r,y} = v_{y,\mathrm{wheel}}$ |

Sign convention follows Chapter 01 / 03: positive $F_x$ drives the vehicle
forward when $\kappa > 0$; positive $F_y$ is ISO 8855 lateral force (restoring
— opposite to the direction of sideslip velocity at the contact).

---

## 19.4 LuGre state equations

Canudas de Wit *et al.* (1995) model friction through an internal deflection
$z$ with dynamics:

$$
\dot z = v_r - \frac{\sigma_0\,|v_r|}{g(v_r)}\,z
$$

$$
F = \sigma_0\,z + \sigma_1\,\dot z + \sigma_2\,v_r
$$

| Symbol | Role | Typical unit |
|---|---|---|
| $z$ | bristle deflection (internal state) | m |
| $v_r$ | relative slip velocity at contact | m/s |
| $\sigma_0$ | bristle stiffness | N/m |
| $\sigma_1$ | micro-damping (critical $\approx 2\sqrt{\sigma_0 m_{\mathrm{eff}}}$) | N·s/m |
| $\sigma_2$ | viscous friction | N·s/m |
| $g(v_r)$ | breakaway force magnitude (steady envelope) | N |

The nonlinear term $\sigma_0 |v_r| z / g$ drives $z \to g/\sigma_0$ when sliding
persists, so $F \to g$ — recovering a **steady slip curve**.

### Operating limits

**Rest ($v_r \to 0$, constant $z$):** $\dot z = 0$ requires no motion of $z$
from the ODE alone; force is $F = \sigma_0 z$ (elastic stick). External load
balance must set $z$ — in practice initialization and load transients matter.

**Steady sliding ($v_r \neq 0$, constant $z$):** $z_{\mathrm{ss}} = g/\sigma_0$,
hence $F_{\mathrm{ss}} = g(v_r)$ when $\sigma_2$ is small.

**Low-speed singularity removed:** forces use $v_r$ directly, not $\kappa =
v_r/v_x$, so there is no $1/v_x$ in the constitutive law.

---

## 19.5 Choosing $g$ — Pacejka as steady-state envelope

VDSim sets the breakaway envelope from the **existing MF96 evaluator** (Chapter
03) at the current $(\kappa, \alpha, F_z, \mu_{\mathrm{long}}, \mu_{\mathrm{lat}})$,
with **axis-specific floors** so presliding stiffness exists without phantom
forces at rest.

### Longitudinal

$$
g_x = \max\bigl(|F_x^{\mathrm{MF96}}|,\; 1\,\mathrm{N}\bigr)
$$

There is **no** $\mu_{\mathrm{long}} F_z$ floor on $g_x$. At $\kappa \approx 0$
the MF longitudinal force is near zero; a Coulomb floor would cap $|z|$ as if the
wheel were sliding while stationary, producing phantom $F_x$ and wheel-spin
chatter with zero throttle.

### Lateral

$$
g_y = \max\Bigl(|F_y^{\mathrm{MF96}}|,\;
  w(\alpha)\,\mu_{\mathrm{lat}} F_z,\; 1\,\mathrm{N}\Bigr),
\quad
w(\alpha) = \exp\!\left(-\left(\frac{\alpha}{0.03\,\mathrm{rad}}\right)^2\right)
$$

When $\alpha \to 0$, MF96 sends $|F_y^{\mathrm{MF}}| \to 0$ but cornering still
needs presliding stiffness — the gated $\mu_{\mathrm{lat}} F_z$ floor supplies it.
When $|\alpha|$ grows, $w \to 0$ and $|F_y^{\mathrm{MF}}|$ governs $g_y$, so
turn+throttle does not get an artificial lateral saturation from the floor.

LuGre changes **how fast** force approaches these envelopes (transient +
low-speed), not the high-slip steady curve used in ISO validation.

### ISO lateral sign

Raw bristle force from $v_{r,y} = v_{y,\mathrm{wheel}}$ has the same sign as
$\alpha$ at moderate speed, while Pacejka uses $F_y = -D\sin(\cdots)$. The host
applies $F_y = -F_{\mathrm{raw}}(z_{\mathrm{lat}}, v_{r,y}, g_y)$ so LuGre matches
MF96 sign and magnitude bounds.

### Combined slip

When `combined_slip_enabled` (catalog default), decoupled $(F_x, F_y)$ are
rescaled onto the **peak** friction ellipse (Chapter 03 / task 15):

$$
F_{x,\max} = D_{\mathrm{long}}\,\mu_{\mathrm{long}}\,\mu_{\mathrm{eff}}\,F_z, \quad
F_{y,\max} = D_{\mathrm{lat}}\,\mu_{\mathrm{lat}}\,\mu_{\mathrm{eff}}\,F_z
$$

not onto instantaneous $|F_x^{\mathrm{MF}}|$, $|F_y^{\mathrm{MF}}|$. With LuGre on,
the host also **skips** the extra circular $\sqrt{F_x^2+F_y^2} \le \mu F_z$ clip
so grip is not reduced three times in combined slip.

Aligning moment: $M_z = -t_p(\alpha)\,F_y$ with pneumatic trail falloff (Chapter 03).

---

## 19.6 Time discretization — semi-implicit $z$

The term $\sigma_0 |v_r| z / g$ makes $\dot z$ stiff at large $\sigma_0$.
Explicit Euler on $z$ can go unstable at $\Delta t = 5\,\mathrm{ms}$ (race /
real-time step).

VDSim uses the same **semi-implicit** bristle update as steering LuGre
(Chapter 17):

$$
z_{k+1} = \frac{z_k + \Delta t\, v_r}{1 + \Delta t\,\sigma_0\,|v_r|/g}
$$

then clamp $|z_{k+1}| \le g/\sigma_0$. Force uses $\dot z \approx (z_{k+1} -
z_k)/\Delta t$ in the $\sigma_1$ term.

### Coupling to the vehicle integrator

Within one `derivatives()` call, $z$ is **frozen** (all RK4 substates in that
evaluation see the same bristle state).

For **RK4**, $z$ is advanced **after each of the four stages** with
$\Delta t_{\mathrm{stage}} = h/4$, so the total bristle integration per substep
is $h$. There is no fifth advance at step end.

For **Euler**, one `derivatives` + one `advance_lugre_states(h)`.

Wheel spin uses the same clipped longitudinal force as the body:
$d\omega = (T_{\mathrm{drive}} + T_{\mathrm{brake}} - F_x R) / I_{\mathrm{wheel}}$
with $F_x$ from LuGre after the friction ellipse — no separate MF split on
`fx_kin`.

---

## 19.7 Longitudinal / lateral coupling

**Textbook target (Deur *et al.*, 2004):** a 2D bristle field in the contact
plane so the friction ellipse emerges from geometry instead of a post-scale.

**VDSim shipped model:** independent LuGre channels per wheel for $x$ and $y$,
then peak-axis friction-ellipse rescale when `combined_slip_enabled`. Simpler
than Deur's 2D brush and tuned for RWD throttle+steer, grade hold, and coast
without phantom longitudinal force — but **does not** capture shear coupling in
the adhesion zone (e.g. simultaneous brake-in-turn transient path).

---

## 19.8 Comparison with VDSim default low-speed path

When `lugre.enabled: false` (catalog default), VDSim uses:

- kinematic–dynamic blend on $(v_y, r)$ below $V_{\mathrm{blend}} \approx 3\,\mathrm{m/s}$;
- lambda fade on lateral (and display longitudinal) tire force;
- viscous brake-hold damper on longitudinal creep.

When `lugre.enabled: true`, those paths are **disabled**; LuGre forces feed the
full dynamic EOM at all speeds. Catalog default remains `enabled: false` so ISO
and blend baselines are unchanged. Tuned preset: `tire.lugre_on`
($\sigma_0 = 3\times 10^5\,\mathrm{N/m}$, $\sigma_2 = 120\,\mathrm{N{\cdot}s/m}$).
Implementation detail: [`V0.2_TIRE_LUGRE.md`](../design/V0.2_TIRE_LUGRE.md).

| Aspect | MF96 + blend (default) | LuGre + MF96 $g$ (opt-in) |
|---|---|---|
| Low-speed lateral | kinematic constraint | bristle elasticity |
| Standstill on grade | brake-hold damper (small creep) | bristle $z$ + presliding $g_y$ |
| High-speed curve | MF96 | MF96 steady limit |
| State in host | $\alpha_{\mathrm{dyn}}$ optional | $z_x$, $z_y$ per wheel |
| Singularity | avoided by blend, not physics | avoided in constitutive law |

---

## 19.9 Validation and limits

| Check | Expectation (ctest `LuGreTire/*`) |
|---|---|
| Steady deflection | $z = g/\sigma_0 \Rightarrow F \approx g$ |
| High-speed cruise | $|F_{\mathrm{LuGre}} - F_{\mathrm{MF96}}|$ small (`HighSpeedLongitudinalNearPacejka`) |
| Grade creep | less creep than blend (`LessGradeCreepThanKinematicBlend`) |
| Coast at rest | no phantom $F_x$ (`CoastFlatNoPhantomLongitudinal`) |
| Throttle + steer | longitudinal not “iced” (`LongitudinalWithSmallSteer`) |
| RWD $\kappa$ | bounded under throttle (`RearSlipBoundedUnderThrottle`) |
| ISO $F_y$ sign | matches MF96 (`LateralForceOpposesSlipIso8855`) |
| Yaw from small $r_0$ | does not run away (`LowSpeedYawDoesNotRunaway`) |

**Not modeled:** tire temperature, wear, pressure transient, belt dynamics,
full MF2002, relaxation of carcass independent of LuGre $z$, force-dependent
camber feedback, reverse-driving $\alpha$ wrap.

---

## 19.10 VDSim implementation notes

> **[VDSim impl] — state and parameters**
>
> - `TireParams.lugre`: `enabled`, `sigma0`, `sigma1`, `sigma2`, `m_eff`.
> - YAML: `lugre:` under `configs/parts/tire/*.yaml`; default `enabled: false`.
> - Per wheel: `z_long`, `z_lat` in the dynamics host (not in `ITireModel`).

> **[VDSim impl] — force pipeline** (`lugre_tire.hpp`)
>
> 1. MF96 `compute(in)` → `lugre_breakaway()` → $g_x$, $g_y$.
> 2. `lugre_force(z, v_r, g, p)` → $F_x$; $F_y = -F_{\mathrm{raw,lat}}$.
> 3. `lugre_friction_ellipse()` if `combined_slip_enabled`.
> 4. $M_z = -t_p(\alpha)\,F_y$; `fx_kin = Fx` for wheel spin.

> **[VDSim impl] — hosts**
>
> - L1 `bicycle_dynamics.cpp`, L2 `seven_dof_dynamics.cpp`, L3 → L2.
> - LuGre on: skip kinematic blend, $\lambda$ fades, brake-hold; ignore
>   `relaxation_length_*`.
> - LuGre + `combined_slip_enabled`: skip outer $\mu F_z$ circular clip.

> **[VDSim impl] — time stepping**
>
> - RK4: `advance_lugre_states_(h/4)` after k1, k2, k3, k4 (four times).
> - Euler: one advance after the step.
> - $z$ constant inside each `derivatives()` evaluation.

> **[VDSim impl] — enable paths**
>
> - Catalog `tire.lugre_on`, CLI `--lugre` / `--no-lugre`, GUI Tire panel (persisted
>   in scene `fleet_overrides`), Python `TireParams.lugre.enabled`.

> **[VDSim impl] — tests & user guide**
>
> - `ctest -R LuGreTire`; `TireYaml.LuGreRoundtrip`.
> - [`docs/CATALOG_AND_PHYSICS.md`](../CATALOG_AND_PHYSICS.md),
>   [`docs/design/V0.2_TIRE_LUGRE.md`](../design/V0.2_TIRE_LUGRE.md).

> **[VDSim impl] — steering LuGre (different module)**
>
> Chapter 17 §17.6: steering-column friction in N·m, Stribeck $g(\omega)$.
> Do not mix with tire `lugre` (N/m, contact slip in m/s).

---

## 19.11 Self-check

1. Why does $\dot z = v_r - (\sigma_0 |v_r|/g) z$ imply $F \to g$ in steady sliding?
2. What is $v_r$ for the rear wheel in a wheel-aligned frame when the vehicle
   sideslips but does not spin ($\omega = 0$)?
3. Why is semi-implicit integration preferred over explicit Euler for $z$?
4. Why does VDSim omit the $\mu F_z$ floor on $g_x$ but gate it on $g_y$ with $w(\alpha)$?
5. Why is $F_y$ negated after `lugre_force` for the lateral channel?
6. What is lost by decoupled long/lat LuGre + peak ellipse vs Deur's 2D brush?

---

## 19.12 References

1. Canudas de Wit, Olsson, Åström, Lischinsky, "A new model for control of
   systems with friction," *IEEE TAC* 45(3), 1995.
2. Canudas-de-Wit, Tsiotras, Velenis, Basset, Gissinger, "Dynamic Friction Models
   for Road/Tire Longitudinal Interaction," *Vehicle System Dynamics* 39(3), 2003.
3. Deur, Asgari, Hrovat, "A 3D brush-type dynamic tire friction model,"
   *Vehicle System Dynamics* 41(2), 2004.
4. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Elsevier — Ch.3 brush model.
5. VDSim design: [`docs/design/V0.2_TIRE_LUGRE.md`](../design/V0.2_TIRE_LUGRE.md).

---

## 19.13 Link to next topics

- **Chapter 04–06** — LuGre forces enter the same Newton–Euler summation as MF96.
- **Chapter 15** — ISO step-steer transients: compare relaxation length (§3.9) vs
  LuGre when re-baselining validation.
- **Chapter 17** — actuator LuGre shares the semi-implicit $z$ trick but not the
  tire kinematics.
