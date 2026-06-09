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
2. **TODO (entangled — careful)** old "vehicles"-format scene loaders: `legacy_vehicle_row`
   / `fleet_spec_from_legacy_vehicle_row` AND the separate `_import_run_config_doc`
   path + the `elif data.get("vehicles")` dispatch in `import_run_config`
   (gui/runner/draft.py). All current scenes are v3/fleet, so these are dead, but it
   is a multi-loader GUI scene-load surgery (no visual verify here) — do via Cursor
   with the scene ctests + a fleet-scene import smoke test as gate, or a focused
   session. test_scene_materialize asserts the *output* "vehicles" key (keep that).
4. Re-grep `from_legacy` / `*_compat` / compat `*_fallback` -> delete the dead ones.

## Phase D — Quick wins (interleave)
- `-Wunused` + grep: dead code / unused includes / unused functions. Tidy stale comments.

## Phase C — Dynamics dedup
- Shared low-speed blend / brake-hold / ARB / drivetrain-split across bicycle /
  seven_dof / fourteen_dof -> shared header. byte-identical (267 green).

## Phase B — GUI monolith split
- `app.html` (4873 L) inline JS -> `gui/static/*.js` (scene / stream / sidebar /
  modal / minimap / telemetry); HTML = shell. Trim `server.py`/`routes.py`/`draft.py`.

## Phase E — Wrap
- Merge `feat/v0.4-slope-jump-m5` -> main + tag. Refresh HANDOFF / doc index.

Order: **A -> D -> C -> B -> E**.
