# Tire model roadmap — dynamic layers beyond MF96 / LuGre

Status: **plan** (2026-06). Shipped baseline unchanged until each phase lands with
ctest + ISO re-baseline where applicable.

**Master checklist:** [`docs/ROADMAP.md`](../ROADMAP.md) §2 (tire).

**Shipped today:** [`V0.2_TIRE_LUGRE.md`](V0.2_TIRE_LUGRE.md),
[`LOW_SPEED_HANDLING.md`](LOW_SPEED_HANDLING.md), theory
[`19_lugre_dynamic_tire.md`](../theory/19_lugre_dynamic_tire.md).

**Positioning (external):** VDSim will grow an **open, layered tire stack** —
Pacejka-family steady forces, optional brush dynamics (LuGre), and future
**belt transient** and extended effects — without shipping proprietary TNO
binaries. Default catalog behaviour stays frozen per phase until explicitly
re-baselined.

---

## 0. Build vs adopt — library survey (decided 2026-06-09)

Surveyed reusable tire libraries before extending our own. **Decision: keep VDSim's
own lean tire stack; do not take a third-party tire library as a dependency.**

| Option | Lang / License | Has | Verdict |
|--------|----------------|-----|---------|
| **Project Chrono / Chrono::Vehicle** | C++ / **BSD-3** | Pac89, **Pac02 (MF2002)**, **TMeasy**, Fiala; `.tir`→JSON | **Reference only.** Permissive, but it's a large multibody engine — adopting it adds a heavy dependency against the open-core "light core" positioning. Use Chrono Pac02/TMeasy output as a **cross-validation reference** for our T1 parity gate; BSD-3 permits lifting specific equations with attribution if useful. |
| teasit magic-formula-tyre-library/-tool | MATLAB / **GPL-3.0** | MF6.1 | **Excluded.** GPL is incompatible with open-core (permissive) linking; MATLAB; archived 2026-03. Fitting/coefficient reference at most. |
| TNO MF-Tyre/MF-Swift, FTire, TMeasy (commercial), CarSim/Adams | closed | belt / rigid-ring / enveloping (SOTA) | **Cannot use code.** They define the belt/transient state of the art but are commercial. |

Consequences:

- **Steady MF (T1):** we already ship our own MF2002 evaluator + `.tir` parser
  (`magic_formula_tire.cpp`). Keep it; gate it against Chrono Pac02 as an independent
  reference rather than depending on Chrono.
- **Belt transient (T2 — the "more dynamic" goal):** **no permissive open-source belt /
  rigid-ring model exists.** So T2 is genuine in-house development from Pacejka's published
  equations (3rd ed. Ch.7/9: relaxation length, belt deflection states) — the physics is
  public, only the commercial *code* is not. This validates the "open layered tire" build.
- **License red line:** GPL and commercial sources are reference-only, never linked or
  copied. BSD/MIT/Apache (e.g. Chrono BSD-3) may be lifted with attribution.

---

## 1. Current stack (v0.3+)

| Layer | Implementation | Default | Role |
|-------|----------------|---------|------|
| Steady forces | `pacejka_mf96.cpp` | **on** | BCDE + friction-ellipse combined slip |
| Steady forces (measured) | `magic_formula_tire.cpp` + `.tir` | lib only | MF2002-style $G_{x\alpha}, G_{y\kappa}$, full $M_z$ |
| Low-speed host | kinematic blend + λ fade + brake-hold | **on** when LuGre off | parking / ISO baseline |
| Contact brush | `lugre_tire.hpp` + host $z$ integration | opt-in | presliding → slide; MF shapes $g(\cdot)$ |
| Lateral 1st-order lag | `relaxation_length_lat` → `alpha_dyn_` | off when LuGre on | quasi-transient (MF-Tyre lite) |

Gaps vs a full dynamic tire product (MF-Tyre-class):

- No **belt / carcass transient** states (step-steer lag from carcass, not only brush).
- No **VLOW** block tied to unified slip variables.
- Combined slip on LuGre path: peak ellipse, not MF2002 $G$ weighting.
- No turn-slip, inflation, temperature, wear.

---

## 2. Target architecture (future)

```text
  wheel kinematics (κ, α_geom, Fz, μ, γ)
           │
           ▼
  ┌─────────────────────┐
  │ Belt transient      │  ← Phase T2 (new states per wheel)
  │ (carcass deflection)│
  └─────────┬───────────┘
            │  κ_eff, α_eff  (or u,v belt states)
            ▼
  ┌─────────────────────┐     ┌──────────────────┐
  │ Steady constitutive │ OR  │ LuGre brush z    │
  │ MF96 / MF2002 .tir  │     │ (presliding)     │
  └─────────┬───────────┘     └────────┬─────────┘
            └────────────┬─────────────┘
                         ▼
              Fx, Fy, Mz  →  vehicle EOM + wheel spin
```

**Design rules**

- `ITireModel::compute()` stays **stateless**; belt / LuGre states live in dynamics host
  (same pattern as `lugre_z_*`, `alpha_dyn_`).
- New physics **opt-in** via `TireParams` / catalog part; default preset unchanged until
  phase sign-off.
- Each phase: unit + integration tests; ISO 7401/4138 re-run when forces can move.

---

## 3. Belt transient — what it is (Phase T2 basis)

### 3.1 Problem

Steady Pacejka maps **instantaneous** slip $(\kappa, \alpha)$ to force. In a step steer
or fast brake release, **measured** $F_y(t)$ lags **geometric** $\alpha(t)$ by tens of ms
because the **carcass / belt** deflects before the contact patch reaches the steady
deflection distribution.

That lag is **not** the same as:

| Mechanism | Physics | VDSim today |
|-----------|---------|-------------|
| Relaxation length | 1st-order $\alpha_{\mathrm{dyn}} \to \alpha_{\mathrm{geom}}$ | `relaxation_length_lat` (LuGre off) |
| LuGre $z$ | bristle stick–slip at contact | `lugre.enabled` |
| Belt transient | **carcass** compliance filters slip **before** MF | **planned T2** |

LuGre fixes low-speed / presliding; belt fixes **high-speed transient shape** (step steer,
load transfer + steer, brake-in-turn onset).

### 3.2 Typical model structure (Pacejka / MF-Tyre family)

Per wheel, introduce carcass states (symbol names vary by source):

- Lateral belt deflection $q_y$ [m] or filtered slip $\alpha_{\mathrm{eff}}$
- Longitudinal belt deflection $q_x$ [m] or $\kappa_{\mathrm{eff}}$

Example (conceptual — coefficients to be identified in T2 spec):

$$
\dot q_y = -\frac{|V_x|}{\sigma_y}\, q_y + C_y\, V_x\, \alpha_{\mathrm{geom}}
$$

$$
F_y = F_y^{\mathrm{MF}}\bigl(\kappa_{\mathrm{eff}}, \alpha_{\mathrm{eff}}(\alpha_{\mathrm{geom}}, q_y), F_z, \ldots\bigr)
$$

$\sigma_y$ is the **relaxation length** [m]; time constant $\tau_y \approx \sigma_y / |V_x|$.

MF-Tyre couples this with steady MF2002 internally; VDSim would **feed effective slip**
into existing `ITireModel` or LuGre $v_r$.

### 3.3 Integration with LuGre

Not mutually exclusive:

- **Belt** — carcass filtering (fast step, highway transient).
- **LuGre** — contact friction state (rest, grade, presliding).

Planned default pairing for “full dynamic” catalog part: belt → MF2002 or belt → LuGre $g(\cdot)$,
with a compatibility matrix in T2 design note.

---

## 4. Phased roadmap

### Phase T1 — Steady MF2002 in catalog (medium effort)

> **Progress (2026-06-09, `feat/tire-t1-mf2002`):** T1.1 — MF2002/.tir evaluator
> sanity tests on a runtime synthetic `.tir` (`Mf2002Catalog.*`; no `.tir` committed).
> T1.2 — `TireParams.backend`/`tir_path` + `create_tire_from_params()` dispatch wired
> into every dynamics `initialize()` (default `mf96` unchanged; `magic_formula` loads
> MF2002 from `tir_path`; `linear` fallback). T1.3 — a tire YAML selects the backend
> via `from_yaml` (`Mf2002Catalog.YamlSelectsBackend`); since cosim + batch load tires
> through `from_yaml`, the **realtime/batch "`.tir` runs" exit is covered** (set
> `backend: magic_formula` + `tir_path: <your .tir>` in the tire part; `.tir` stays
> uncommitted). T1.4 — the MF2002 evaluator already does combined slip
> ($G_{x\alpha},G_{y\kappa}$+SVyk); added `TireParams::model_provides_combined_slip()`
> and gated the host friction-ellipse clip on it so MF2002 (and LuGre) are not
> re-clipped (MF2002's peak $F_x$ exceeds $\mu_{nom}F_z$). 279 ctest.
> **Remaining:** GUI `.tir` import (v0.5.1 GUI bundle); parity gate vs Chrono Pac02
> reference (needs a Chrono build — external env). Headless T1 core is complete.

| Item | Deliverable |
|------|-------------|
| Wire `create_*_from_tir()` | catalog tire parts + GUI import |
| Combined slip | $G_{x\alpha}, G_{y\kappa}$ on default measured path |
| LuGre $g()$ | optional: sample from MF2002 $F_{x0}, F_{y0}$ not MF96 |
| Tests | parity vs `magic_formula_tire` reference; ellipse → $G$ migration tests |
| Docs | Ch.03 §3.16 promoted to user guide |

**Exit:** one sample `.tir` (synthetic/public coefficients) runs in realtime + batch.

### Phase T2 — Belt transient (high effort) ★ core “more dynamics”

> **Progress (2026-06-09, `feat/tire-t2-belt`, on top of `feat/tire-t1-mf2002`):**
> T2.1 — `belt_relax()` primitive in `vdsim/belt_tire.hpp` (first-order, exact
> exponential, frozen at standstill) + unit tests. T2.2 — wired into seven_dof
> (L2/L3-inner) MF path (relax kappa/alpha). T2.3 — LuGre stacking (relax the slip
> velocity feeding v_r). T2.4 — free_3d (L5) MF + LuGre. Opt-in `BeltTireParams
> {enabled, sigma_lat, sigma_long}` (default off -> no ISO/L2/L3 drift). **Validation:**
> `test_belt_validation.cpp` — steady unchanged, early response suppressed at t=tau,
> more lag at lower speed (the sigma/|Vx| scaling). 291 ctest. Theory:
> [`theory/21_belt_transient.md`](../theory/21_belt_transient.md).
> **Remaining:** bicycle (L1) wiring (lowest value). Then T2 can merge with T1
> (T1 -> T2 ff order).

| Item | Deliverable |
|------|-------------|
| States | `belt_qx`, `belt_qy` or $\kappa_{\mathrm{eff}}, \alpha_{\mathrm{eff}}$ per wheel in L2/L1 host |
| Params | `belt.relaxation_length_lat/long`, optional stiffness coupling (YAML) |
| Force path | filtered slip → `ITireModel::compute` or LuGre |
| Integrator | semi-implicit or RK4 sub-advances (mirror LuGre pattern) |
| Validation | step-steer lag vs relaxation-length analytic; ISO 7401 shape check |
| Theory | new theory chapter or Ch.03 §3.9 extension |

**Exit:** step-steer peak/time-to-steady closer to reference transient without LuGre on;
no regression on default MF96 preset.

### Phase T3 — Unified low speed (VLOW-class) (high effort)

| Item | Deliverable |
|------|-------------|
| Goal | single slip definition $0 \to |V_x|$ without kinematic blend |
| Options | (a) extend LuGre + belt, or (b) Pacejka VLOW-style damping on $v_r$ |
| Retire? | kinematic blend only when `tire.dynamic_mode: legacy` |

**Exit:** grade hold + parking steer without blend path; document trade vs LuGre-only.

### Phase T4 — Combined slip upgrade on dynamic path (medium)

| Item | Deliverable |
|------|-------------|
| LuGre + dynamic | replace peak ellipse with MF2002 $G$ weighting when `.tir` or extended MF96 |
| 2D brush (optional) | research spike: Deur coupled bristle vs $G$ weighting |

**Exit:** brake-in-turn integration test band; no triple-clip artefacts.

### Phase T5 — Extended effects (long / optional)

| Block | Priority | Notes |
|-------|----------|-------|
| Longitudinal relaxation | medium | today only lateral `alpha_dyn_` |
| Turn-slip | low | oval track / severe slow corner |
| Inflation pressure | low | $F_z$, stiffness scaling |
| Temperature | low | thermal states; racing |
| Rolling radius dynamics | low | ties to belt |
| Wear | out of scope v1 | — |

### Phase T6 — Ecosystem (parallel)

| Item | Deliverable |
|------|-------------|
| External tire FMU | co-sim hook (customer MF-Tyre / FTire) |
| Workshop | `.tir` import UI per `V0.2_WORKSHOPS.md` |
| Public benchmark | one anonymized or synthetic cross-check report |

---

## 5. Validation gates (every phase)

1. `cd build && ctest` — all tests green; **no silent ISO number drift** on default preset.
2. `apps/validation/run_validation.py` — document before/after for 7401/4138 when tire forces change.
3. New tests named `BeltTire/*`, `Mf2002Catalog/*`, etc.
4. Theory doc + `CATALOG_AND_PHYSICS.md` user section updated.
5. Confidential `.tir` values never committed.

---

## 6. Timeline (indicative, not committed)

| Phase | Horizon | Depends on |
|-------|---------|------------|
| T1 | next tire sprint | catalog/GUI bandwidth |
| T2 | +1 major release | T1 optional; core integrator work |
| T3 | after T2 or parallel LuGre track | low-speed acceptance criteria |
| T4 | after T1 | MF2002 in loop |
| T5 | demand-driven | OEM / motorsport |
| T6 | ongoing | partnerships |

v0.4 remains **stunt / Ld5** per [`V0.4_PLAN.md`](V0.4_PLAN.md); tire Phases T1–T2 are
**not** v0.4 blockers unless explicitly scheduled.

---

## 7. External messaging (approved wording)

- **Now:** “Pacejka MF steady tire with optional LuGre brush dynamics; ISO-validated
  vehicle stack; `.tir` steady MF evaluator in library.”
- **Planned:** “Open layered tire model: measured MF2002 steady forces, belt transient
  for carcass lag, and extended combined-slip / environmental effects — integrated with
  the same real-time L1–L3 core.”
- **Avoid:** “MF-Tyre clone” or “100% real-vehicle tire” until Phase T6 benchmark exists.

---

## 8. References

- Pacejka, *Tire and Vehicle Dynamics*, 3rd ed. — Ch.3 brush, Ch.4 MF, relaxation / belt.
- Canudas de Wit *et al.*, LuGre (1995) — contact friction (shipped).
- Deur *et al.*, 3D brush-type dynamic friction (2004) — coupled slip research direction.
- TNO MF-Tyre / MF-Swift product docs — **reference architecture only** (not shipped code).
