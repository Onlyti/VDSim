# Vehicle Physics Pro — hands-on evaluation plan

**Status:** plan written, hands-on **not started** (blocked; see §5) · **Date:** 2026-09-17
**Type:** discussion document (current state → evidence → open issues → options → next action)

## 1. Why we are looking at it

VDSim's credibility gap is external cross-validation: the roadmap's validation
section still lists "published commercial cross-val" and "external Adams /
physical KC-rig" as open. Vehicle Physics Pro (VPP) is the closest thing to a
peer product that is (a) reachable without a commercial license negotiation and
(b) built around an explicit multibody-ish vehicle model rather than an arcade
approximation. It is a *reference point*, not a ground truth — VPP is another
simulator, so the same caveat we already apply to the Chrono KC cross-check
applies here.

The question this evaluation answers is narrow: **where does VDSim's fidelity
and workflow sit relative to a mature commercial vehicle-dynamics package, and
which of its features are table stakes that we are missing?**

## 2. What is already established (no install needed)

Editions and cost ([vehiclephysics.com/about/licensing](https://vehiclephysics.com/about/licensing/)):

| Edition | Cost | Scope |
|---------|------|-------|
| Community | free, via Unity Asset Store | single vehicle, desktop only — "ideal for evaluation" |
| Professional | €590 + €190/yr | unlimited vehicles, all build targets, custom extensions |
| Enterprise | €5,900 + €1,900/yr | full source, motion platform; requires >€200k annual revenue |

Feature surface ([vehiclephysics.com/about/features](https://vehiclephysics.com/about/features/)):

- **Tire:** selectable friction models — flat, linear, smooth, parametric,
  **Pacejka** — with lateral deflection and rolling friction. Same family as our
  MF96 / MF2002 path; no `.tir` ingest is advertised.
- **Suspension:** spring / damper / ARB with bump-rebound and slow-fast damper
  splits, variable-rate springs, ride-height-preserving "dynamic suspension".
  Parameter-level, **not hardpoint-level** — no advertised KC / anti-dive /
  roll-centre emergence, which is exactly where our L5 free-3D MBD sits.
- **Drivetrain:** open / locked / LSD / Torsen differentials, torque splitters,
  manual + automatic transmissions, FWD/RWD/AWD. Torsen and the torque splitter
  are ahead of our open-diff-only drivetrain v2.
- **Solver:** per-vehicle substeps for outer rates from 16 Hz to 2 kHz — the same
  substep-vs-outer-step contract we just made explicit in `solver_substeps()`.
- **Telemetry:** live telemetry, suspension charts, CSV export.
- **Validation:** **none advertised.** No real-vehicle or ISO-manoeuvre
  comparison is claimed anywhere in the feature documentation.

That last point is the most important finding so far, and it reframes the
evaluation: VPP is a *game/production* physics package with a richer drivetrain
and authoring workflow, not a validated reference. Our ISO-gated baseline is a
differentiator, not a gap.

## 3. What the hands-on actually has to produce

Running their demo is not an evaluation. The hands-on is worth doing only if it
produces a side-by-side on manoeuvres we already have baselines for:

1. **ISO 7401 step steer** — yaw-rate response time, overshoot, steady gain.
   We have a CI-gated baseline (`IsoBaseline`); VPP output comes from its CSV
   telemetry export.
2. **ISO 4138 understeer gradient** — steady-state circle, gradient in deg/g.
3. **ISO 3888-2 double lane change** — max entry speed, path deviation.
4. **Coast-down** — pure resistance stack, isolates road load from tire.
5. **Authoring workflow cost** — wall-clock from "vehicle parameters in hand" to
   "manoeuvre result plotted", measured the same way on both. This is the axis
   where a commercial package usually wins and we should know by how much.

Each comparison is reported with the metric definition, units, and how VPP's
parameters were matched to ours — an unmatched-parameter comparison is worthless
and should be labelled as such rather than published.

Known limits to state up front: Community Edition is single-vehicle and
desktop-only, so fleet and multi-vehicle behaviour cannot be evaluated; without
source access (Enterprise) the tire and solver internals stay a black box, so
any disagreement can be attributed but not diagnosed.

## 4. What the hands-on costs

- Unity Hub + Editor (LTS): ~10 GB, GUI host required. `ailab-12` is a headless
  research box; the natural host is the Windows machine `jiwon`.
- A Unity account sign-in is required to pull the Community Edition from the
  Asset Store.
- Estimated effort: ~0.5 day to a running demo scene, ~1-2 days to the four
  manoeuvre comparisons including parameter matching (the matching is the slow
  part, not the driving).

## 5. Blocking issue

The hands-on cannot be started autonomously. Downloading the Community Edition
requires signing in to the user's Unity account, and installing a ~10 GB GUI
editor on a lab workstation is a host-level decision, not a repo change.

## 6. Options

| Option | Cost | What it buys |
|--------|------|--------------|
| A. Community Edition on `jiwon` | free, ~2 days | The four manoeuvre comparisons + workflow timing. Single vehicle, black-box internals. |
| B. Desk research only | ~2 h | Feature-matrix comparison from public docs. No numbers, no workflow measurement. |
| C. Professional (€590) | paid | Multi-vehicle + all targets. Buys nothing the evaluation needs; source is still closed. |
| D. Drop VPP, spend the time on Chrono KC | free | Chrono is already scaffolded (`external/chrono_kc/`) and is an *independent MBD*, which is the stronger cross-check. VPP has no validation claim to cross-check against. |

## 7. Recommended next action

**D, with B as a cheap companion.** The finding in §2 — VPP advertises no
validation — means a VPP cross-check would compare us against an unvalidated
peer, which does not close the roadmap's credibility gap. The already-scaffolded
Chrono KC cross-validation does. Keep a short public-docs feature comparison (B)
for competitive positioning, and re-open A only if the goal changes from
*credibility* to *authoring-workflow benchmarking*, where VPP is genuinely ahead.
