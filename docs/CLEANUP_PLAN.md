# Code cleanup plan (dev-stage)

Premise (2026-06-09): **no backward-compat / no legacy support until v1.0** — single
canonical path only; delete all legacy shims/dual-paths. Invariant: **ctest green**
(currently 267) + behaviour unchanged. verify-then-delete (grep callers + build/test
confirm dead) ; one commit per unit ; GUI changes need the user's visual review.

Distinguish real compat code (delete) from descriptive "legacy" comments (keep, e.g.
`params.hpp` "0 = legacy" param-behaviour notes, `tire.kinematic_fallback` (a real
part for the ISO baseline / stunt scenes), pacejka "legacy tests preserved").

## Phase A — Legacy purge
1. **DONE** (`da3b855`) GUI dead routes `/legacy` `/classic` `/index.html` -> `/` `/app` only.
3. **DONE** (`42c82f3`) `create_legacy_stunt_dof` + entire fourteen_dof stunt_ mode +
   kinematic loop rail + legacy test deleted; stunt = Free3D (Ld5) only. 266 green.
2. **DONE** (`ed62b53`) old "vehicles"-format scene loaders removed:
   `legacy_vehicle_row` / `fleet_spec_from_legacy_vehicle_row` + the `elif
   data.get("vehicles")` dispatch. `import_scene_v3` now requires `fleet`;
   `_import_run_config_doc` kept as the single canonical fleet loader (output
   "vehicles" key unchanged). 266 green; fleet import smoke verified.
4. **DONE** Re-grep `from_legacy`/`*_compat`: no dead residue. `catalog_legacy_registry`
   is a LIVE GUI part-registry endpoint (legacy-*shaped* name only) — rename in Phase B,
   not dead. `part_compat` = slot-fit feature (keep). `sensors.hpp`/`actuator.hpp`
   "backward-compatible" are descriptive comments (keep).

## Phase D — Quick wins (interleave)
- **DONE** Refreshed stale ctest counts in `VALIDATION.md` (210/201 -> 266, 2026-06-09).
- TODO `-Wunused` + grep: dead code / unused includes / unused functions. Tidy stale comments.

## Phase C — Dynamics dedup
- **DONE** (`c1939bf`) low-speed trio kStickBlend/kStickC/kKinTau -> `vdsim/low_speed.hpp`
  (byte-identical in bicycle+seven_dof). 266 green.
- NOT extracted (not byte-identical): brake-hold *application* (bicycle uses 2*kStickC
  damper, seven_dof uses -kStickC*vx*gate), ARB (per-wheel, L3-only), drivetrain
  open-diff split (L2 detail). Leave per-source — false dedup would change behaviour.

## Phase B — GUI monolith split
- **B1 DONE** (`aa26ee4`) app.html 4873 L -> 246 L shell. `<style>` -> static/app.css,
  `<script type=module>` -> static/app.js (byte-identical, cmp-verified). /static/ route
  in routes.py. importmap + core.js stay in HTML. Relocation only; smoke 200s, 266 green.
  **Needs user browser verify** (agent has no browser) before B2.
- **B2 TODO (high risk)** split static/app.js (4365 L, single module scope: scene/camera/
  renderer + many shared `let`) into ES sub-modules (scene/stream/sidebar/modal/minimap/
  telemetry) with explicit import/export. Headless gate is weak (node v10 can't ESM-check;
  no esbuild) -> needs an esbuild/rollup bundle gate + user browser verify. Do only after B1
  is browser-confirmed. Then trim server.py/routes.py/draft.py.

## Phase E — Wrap
- Merge `feat/v0.4-slope-jump-m5` -> main + tag. Refresh HANDOFF / doc index.

Order: **A -> D -> C -> B -> E**.
