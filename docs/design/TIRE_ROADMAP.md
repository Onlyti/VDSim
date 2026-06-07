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

| Item | Deliverable |
|------|-------------|
| Wire `create_*_from_tir()` | catalog tire parts + GUI import |
| Combined slip | $G_{x\alpha}, G_{y\kappa}$ on default measured path |
| LuGre $g()$ | optional: sample from MF2002 $F_{x0}, F_{y0}$ not MF96 |
| Tests | parity vs `magic_formula_tire` reference; ellipse → $G$ migration tests |
| Docs | Ch.03 §3.16 promoted to user guide |

**Exit:** one sample `.tir` (synthetic/public coefficients) runs in realtime + batch.

### Phase T2 — Belt transient (high effort) ★ core “more dynamics”

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
