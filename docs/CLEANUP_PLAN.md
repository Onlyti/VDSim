# Code cleanup plan (dev-stage)

Premise (2026-06-09): **no backward-compat / no legacy support until v1.0** — single
canonical path only; delete all legacy shims/dual-paths. Invariant: **ctest green**
(currently 267) + behaviour unchanged. verify-then-delete (grep callers + build/test
confirm dead) ; one commit per unit ; GUI changes need the user's visual review.

Distinguish real compat code (delete) from descriptive "legacy" comments (keep, e.g.
`params.hpp` "0 = legacy" param-behaviour notes, `tire.kinematic_fallback` (a real
part for the ISO baseline / stunt scenes), pacejka "legacy tests preserved").

## Phase A — Legacy purge
1. GUI dead routes `/legacy` `/classic` `/index.html` (index.html deleted) -> `/app` only.
2. `legacy_vehicle_row` / `fleet_spec_from_legacy_vehicle_row` (catalog_bridge) + the
   draft.py caller -> migrate to native catalog, delete the bridge.
3. `create_legacy_stunt_dof` (interfaces.hpp + fourteen_dof) — if 0 callers, delete
   (+ `apply_loop_kinematics` rail if only reachable via it). Stunt = Free3D only.
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
