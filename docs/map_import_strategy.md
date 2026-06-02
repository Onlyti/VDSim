# VDSim Map / Track Import Strategy

Status: DECISION RECORD.
Scope: how VDSim ingests external road/track data.
Note: the IP boundary section is a design constraint, not legal advice — see the
final section.

---

## 1. Decision

VDSim officially supports three map entry points:

- OpenDRIVE (`.xodr`)
- CarMaker (via its official OpenDRIVE export)
- Blender (standard mesh export: OBJ / glTF)

VDSim does NOT reference Assetto Corsa anywhere — not in code, docs, examples,
tutorials, bundled tools, or links. See Section 5.

---

## 2. Three entry points, two importers

The three supported sources collapse onto two importers:

| Entry point | Reduces to | Importer | Feeds |
|---|---|---|---|
| OpenDRIVE (.xodr) | itself | OpenDRIVE parser | road logic (centerline, lanes, curvature) |
| CarMaker | official `.xodr` export (`roadutil.exe` / Scenario Editor) | same OpenDRIVE parser | road logic |
| Blender | standard mesh (OBJ / glTF) | mesh importer | surface geometry (elevation, banking, mu by material) |

So the implementation work is two parsers:
1. OpenDRIVE parser (covers CarMaker too) — use a standard lib (e.g. libOpenDRIVE / esmini).
2. Standard-mesh importer (Blender export) — feeds the contact/raycast path.

CarMaker's native `.rd5` is NOT parsed directly; rely on its vendor-provided
OpenDRIVE export. That is what makes CarMaker support clean and supportable.

---

## 3. Two independent axes — do not couple them

Road logic and surface geometry are separate concerns. Do not force both at once.

| Axis | Source | VDSim hook (existing) | When needed |
|---|---|---|---|
| Road logic (path/lanes) | OpenDRIVE | `PurePursuitController` takes (x,y) arrays today; extend with kappa / v profile | path-tracking / AD validation (1st priority) |
| Surface geometry (z, banking, mu) | Blender mesh | `IContactProvider` callback (see `RaycastContactProvider` pattern) | only when ride / load-transfer / mu matters |

For closed-loop autonomous-driving validation, the OpenDRIVE centerline alone is
usually enough; the flat-ground contact provider (`create_flat_ground`) stays the
baseline. Add the mesh surface only when terrain dynamics are in scope.

---

## 4. Existing VDSim hooks (confirmed) and coordinate convention

- Surface: `IContactProvider::query(State, VehicleParams, ContactArray&)`
  (`core/include/vdsim/interfaces.hpp`). Reference impls: `create_flat_ground`,
  `RaycastContactProvider` (`carla_integration/plugin/`). A new
  `MeshContactProvider` (Blender mesh + ray×triangle) plugs in here.
- Path: `PurePursuitController::update(x,y,psi,vx, path_x[], path_y[], n)`
  (`core/include/vdsim/control_converter.hpp`). The OpenDRIVE centerline feeds these
  arrays; extend later for curvature / speed profile.
- Coordinate frame: world = ENU right-handed, meters; body = ISO 8855. All imported
  geometry MUST be converted to ENU [m] before injection.
- Alignment with AutoHYU: AutoHYU planning consumes lanelet2 (osm). Provide an
  OpenDRIVE <-> lanelet2 conversion so the planning map and the dynamics map come
  from the same source and stay registered. Same ENU origin on both sides.

---

## 5. IP boundary (binding design constraint)

The map strategy is deliberately built on vendor-blessed / neutral paths only:

- OpenDRIVE is an open standard. CarMaker provides official OpenDRIVE export — the
  user converts their own licensed data. Both are clean.
- Blender is a generic DCC tool, unrelated to any specific game. VDSim imports its
  standard mesh export and does not know where that mesh came from.

Hard rules (keep VDSim a neutral tool, keep inducement risk off):
- No Assetto Corsa mention anywhere (code / docs / examples / marketing).
- "Blender support" means the generic Blender standard-mesh export — NOT any
  AC-specific `.kn5` import addon. VDSim does not bundle, link, recommend, or
  document such an addon.
- Whatever a user does upstream of the standard mesh (including extracting content
  from any game) is the user's responsibility. Content copyright and any third-party
  EULA compliance stay with the user, not VDSim.

This makes the VDSim tool itself neutral. It does NOT make any specific upstream
workflow legal — that is the user's burden. Before commercial release, have IP
counsel review this boundary (especially the inducement line and any real-circuit
content licensing).
