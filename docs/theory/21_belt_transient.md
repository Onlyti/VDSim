# 21. Tire — belt / carcass transient (first-order relaxation)

## Learning objectives

After this chapter you can:

1. Explain why a steady Magic Formula (Chapter 03) cannot reproduce the lag in
   measured $F_y(t)$ after a step steer, and which physical effect supplies it.
2. Write the first-order belt relaxation ODE and its exact discrete update, and
   identify the time constant $\tau = \sigma/|V_x|$.
3. Distinguish three different transient mechanisms — relaxation length, the LuGre
   bristle state, and the belt — and say which regime each fixes.
4. Describe VDSim's shipped adoption: a backend-agnostic slip filter that feeds
   relaxed $\kappa,\alpha$ to the MF path and relaxed slip velocity to the LuGre
   path, opt-in per wheel.
5. State what is validated and the honest limit (first order only, no higher belt
   eigenmodes).

## Prerequisites

- **Chapter 03** — Pacejka MF96 pure/combined slip; the steady $(\kappa,\alpha)\mapsto F$ map.
- **Chapter 19** — LuGre bristle $z$ (contact friction state).
- **Chapter 11** — substep integration.
- **Design** — [`design/TIRE_ROADMAP.md`](../design/TIRE_ROADMAP.md) §3 (belt phase T2).

Source: `core/include/vdsim/belt_tire.hpp` (`belt_relax`), wired in
`seven_dof_dynamics.cpp` and `free_3d_dynamics.cpp`; params `TireParams::belt`.

---

## 21.1 Motivation — the carcass deflects before the patch slides

A steady Magic Formula maps the **instantaneous** slip to force. In a step steer
or a fast brake release, the **measured** lateral force lags the **geometric**
slip angle by tens of milliseconds: the **carcass / belt** must first deflect
before the contact patch reaches the steady deflection distribution that the
empirical curve fits. An algebraic map produces this lateral force the same
instant the slip appears — too early.

The lag length is set by the **relaxation length** $\sigma$ [m] (a carcass
compliance property): the tire travels roughly $\sigma$ before the force reaches
$1-1/e$ of its steady value, so the time constant is

$$
\tau = \frac{\sigma}{|V_x|}.
$$

It shrinks with speed — the same deflection is covered faster — which is the
signature the validation checks (§21.4).

---

## 21.2 Three transients, three regimes

| Mechanism | Physics | Fixes | VDSim |
|-----------|---------|-------|-------|
| Relaxation length | first-order $\alpha_{\mathrm{dyn}}\to\alpha_{\mathrm{geom}}$ | lateral transient (MF) | legacy `relaxation_length_lat` (lateral, MF only) |
| LuGre $z$ | bristle stick–slip at the contact | rest / presliding / grade hold | Chapter 19 (`lugre.enabled`) |
| **Belt transient** | **carcass compliance filters slip before the law** | **step-steer / brake-in-turn transient shape** | **this chapter (`belt.enabled`)** |

LuGre fixes the **low-speed / presliding** end; the belt fixes the **high-speed
transient shape**. They are complementary, not alternatives — VDSim lets the belt
filter the slip that feeds the LuGre bristle (§21.3).

---

## 21.3 Model and discrete update

Each wheel carries a relaxed slip component $s_{\mathrm{eff}}$ (for $\kappa$ and
$\alpha$, or for the slip *velocity* on the LuGre path) obeying

$$
\frac{\sigma}{|V_x|}\,\dot s_{\mathrm{eff}} = s_{\mathrm{geom}} - s_{\mathrm{eff}}.
$$

Integrated exactly over a substep $h$ at frozen $V_x$ (unconditionally stable, and
correct at standstill where $|V_x|\to 0$ **freezes** the state instead of dividing):

$$
\boxed{\,s_{\mathrm{eff}} \leftarrow s_{\mathrm{geom}}
   + (s_{\mathrm{eff}} - s_{\mathrm{geom}})\,e^{-|V_x|\,h/\sigma}\,}
$$

This is `belt_relax(s_eff, s_geom, Vx, sigma, dt)`. In the dynamics host:

- **MF path** (`!lugre`): the relaxed $\kappa,\alpha$ feed `ITireModel::compute`.
- **LuGre path**: the relaxed slip **velocities** $v_{r,\text{long}}, v_{r,\text{lat}}$
  feed the bristle (belt → LuGre, complementary).

The belt state advances **once per substep** (between integrator steps) and is held
frozen inside an RK4 step so all four stages see the same slip linearization — the
same pattern as the LuGre $z$ sub-advance. It is opt-in via
`TireParams::belt {enabled, sigma_lat, sigma_long}`; default off leaves the steady
behaviour (and the published ISO baseline) unchanged.

```
  step steer at t=0, fixed Vx
  s_geom ──┐ (instant)
           │      s_eff(t) = s_geom (1 - e^{-t/tau}),  tau = sigma/|Vx|
  ─────────┴───────────────────────────►  t
           0      tau            5·tau
                  63%            ~100%
```

---

## 21.4 Validation

`tests/integration/test_belt_validation.cpp` checks the L2 step-steer response
against the relaxation analytic (a fixed-time $|a_y|$ metric — time-to-fraction is
corrupted by the underdamped yaw overshoot):

- **Steady state unchanged** — the belt is transient-only; $a_{y,\mathrm{ss}}$ within 8%.
- **Early response suppressed** — at $t=\tau$ the belt-on $|a_y|$ is below 95% of
  belt-off (the slip lag), while still building.
- **More lag at lower speed** — at a fixed early time the slower car (larger $\tau$)
  is more suppressed, i.e. the belt-on/off ratio is smaller: the $\sigma/|V_x|$
  scaling.

The `belt_relax` primitive itself is unit-tested against the exact exponential
(`BeltTire.*`): instant when $\sigma=0$, frozen at standstill, $63\%$ at $\tau$ and
steady by $5\tau$, monotone with no overshoot.

---

## 21.5 Shipped status and limitations

- **Opt-in, default off** — no ISO 7401/4138 drift on the default preset; the belt
  is a layer on top of the steady law (MF96 / MF2002) or LuGre.
- **Wired on** L2 (`seven_dof`, also the L3 inner) and L5 (`free_3d`), both the MF
  and LuGre paths. **L1 bicycle** wiring is pending (lowest priority).
- **First order only.** This is the carcass *relaxation* (one time constant), not
  the higher belt eigenmodes / rigid-ring + enveloping behaviour of FTire / MF-Swift
  (commercial, up to 100–150 Hz). Out of scope; see
  [`design/TIRE_ROADMAP.md`](../design/TIRE_ROADMAP.md) §2.
- **Longitudinal** relaxation uses the same primitive with `sigma_long`; combined
  with the MF2002 backend (Chapter 03 / T1) it filters both $\kappa$ and $\alpha$.

---

## Cross-references

- [`03_tire_pacejka_mf96.md`](03_tire_pacejka_mf96.md) — steady force the belt feeds.
- [`19_lugre_dynamic_tire.md`](19_lugre_dynamic_tire.md) — contact bristle state (complementary).
- [`design/TIRE_ROADMAP.md`](../design/TIRE_ROADMAP.md) — tire phases T1–T6; build-vs-adopt decision.
