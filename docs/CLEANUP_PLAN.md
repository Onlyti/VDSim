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
- **B2a DONE** (`192dbb6`) pure leaf helpers -> static/util.js ($, paintConnState,
  doc-help, f/fmtArr, post/postJson/getJson). Gate: esbuild bundle (node20 @ ~/.nvm,
  `npx esbuild@0.21.5 app.js --bundle --format=esm --external:three --external:three/addons/*`)
  -> bundle identical to pre-split except esbuild path-comments. Browser-confirmed by user.
- **B2b #1 DONE** (`f7ced8c`) mini-map -> static/minimap.js (drawMinimap + resetMinimap,
  encapsulated canvas/trail). Browser-confirmed.
- **B2b #2 DONE** (`340c1ba`) field-row / plot builders -> static/fields.js (pure DOM/canvas,
  zero imports). esbuild bundle == original baseline (only B2b#1 resetMinimap differs).
- Pure-leaf extractions now ~exhausted (util/minimap/fields). app.js 4873 -> 3997 L.
- **B2b #3 DONE** (`a02b07b`) manual driving input -> static/manual.js via
  initManualControl(deps). Read-only shared state (manualMode/simRunning/selectedVid) injected
  as closures (DI) instead of moved, so no circular import / no state rewrite. esbuild diff =
  DI indirection + buffer relocation + a cosmetic man/man2 rename. Pattern established: when a
  section only READS shared state, use DI closures; only WRITE-shared state forces a state.js.
- **B2b #4+ TODO (higher risk)** remaining: three.js scene (biggest, ~1000 L, owns scene/
  camera/renderer + reassigned pathLine/terrainGrid... read by the animate loop + telemetry),
  on-screen track edit, telemetry+SSE, scenario setup panel, vehicle tree, modal. These WRITE
  shared state across sections -> need either DI with setters or a `state.js` holder object
  (replace the reassigned `let`s with `S.x` everywhere). Architectural; do with the esbuild
  gate AND a user browser check per increment. Then trim server/routes/draft.

## Phase E — Wrap
- Merge `feat/v0.4-slope-jump-m5` -> main + tag. Refresh HANDOFF / doc index.

Order: **A -> D -> C -> B -> E**.
