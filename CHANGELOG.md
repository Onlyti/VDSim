# Changelog

All notable changes to VDSim are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow
[SemVer](https://semver.org/).

## [0.5.0] — 2026-06-09 · terrain + L5 general driving (headless / batch / cosim)

Generalizes the v0.4 Ld5 stunt body to driving on arbitrary ground. GUI terrain
load + L5 Play and a stunt-scene authoring panel are deferred to v0.5.1.

### Added
- **L5 on terrain**: hub-consistent per-wheel contact unified across Flat / SplitMu /
  Inclined / Rough / **Heightmap** (`wheel_world_positions` + `hub_penetration`).
- **CurvedGround**: banked circular turn in x-y (radius about a centre, banked inward;
  centripetal normal) — `create_curved_ground`, cosim `stunt.ground == banked`.
- Terrain scenes: `terrain_hill_demo.yaml` (heightmap), `banked_grade_demo.yaml`
  (inclined plane), `banked_oval.yaml` (curved banked turn).
- `tools/bake_synthetic_hill.py` + `assets/terrain/hill_demo.bin` (61×41 Gaussian hill).
- `materialize_scene_file` forwards `terrain` (resolved absolute) + `rough_amp/rough_wl/iso_class`.
- Theory **ch.20** (Ld5 stunt EOM + loop entry speed).
- Tests `tests/integration/test_terrain_l5.cpp`: no-sink, hill climb, settle on
  flank, brief airborne over a cliff, uphill coast, bank-induced roll, banked turn
  holds line. **273/273 ctest green.**

### Deferred (v0.5.1)
- M4 GUI terrain load + L5 Play; M5c GUI stunt authoring panel (render-only today).

## [0.4.0] — 2026-06-09 · stunt (Ld5) + multibody (Ld4); folds v0.3 catalog + drivetrain

(v0.3 was never tagged; its catalog + drivetrain + LuGre work ships in this release
alongside the v0.4 stunt/multibody additions below.)

### Added — v0.4 (3D stunt + multibody)
- **Ld5 free 3D body** (`Free3DDynamics`, `level=L5`): 6-DOF quaternion attitude,
  Newton–Euler body-frame EOM, per-wheel MF96/LuGre in the wheel **contact frame**
  (valid inverted), penalty normal-force contact + airborne phase.
- **Stunt scenarios**: ramp jump (`jump_ramp_demo.yaml`) and vertical loop
  (`vertical_loop_demo.yaml`); `RampGround` / `LoopGround` providers; loop entry
  speed `v_min = √(5gR)`.
- **Ld4 hard-joint multibody M1–M7** (`multibody.hpp`): TA / MP / DW / 5-link
  topologies, Baumgarte travel, K&C sweep charts, Adams CSV cross-check (5 gains, 5% rtol).
- **Theory ch.20** (Ld5 stunt EOM + loop entry speed); multibody ch.13/14.
- Tests: `Stunt/*` (jump airborne interval + landing, loop completes / mid-arc
  contact / slip balance / wheel-spin bound), `tests/integration/test_l5_driving.cpp`.

### Changed — v0.4 (cleanup)
- GUI `app.html` (4873 L) externalized: `<style>`→`static/app.css`,
  `<script>`→`static/{util,minimap,fields,manual}.js` (shell ~250 L); `/static/` route.
- Removed legacy `vehicles`-format scene loaders, legacy L3 stunt rail mode, and dead
  GUI route aliases; hoisted shared low-speed blend constants to `vdsim/low_speed.hpp`.

### Added — v0.3 (catalog + drivetrain)
- User guide `docs/CATALOG_AND_PHYSICS.md` (catalog, scenes, GUI API, drivetrain,
  LuGre); mkdocs nav + README links.
- Drivetrain engine rotational inertia (`engine_rotational_inertia`) reflected to
  wheels; open-diff carrier coupling (`drivetrain_inertia.hpp`).
- LuGre dynamic tire (`TireParams.lugre`, MF96 steady-state envelope); opt-in.
- ctest `EngineInertiaSlowsLowMuWheelSpinup`, `LuGreTire/*`, `TireYaml.LuGreRoundtrip`.

### Added (v0.3)
- Parts catalog (`configs/catalog/`, scenes, maneuvers); `vdsim_realtime --scene=` only.
- GUI catalog API (`/api/catalog`, `/api/scene`, simconfig v3).
- `tools/import_part_pack.py` stub + `python/catalog/pack_import.py` (collision check, optional install).
- ctest: `catalog_resolver`, `scene_materialize`, `catalog_api`, `import_part_pack`.

### Changed
- Legacy `configs/vehicles|tires|scenarios/` removed; examples/docs use catalog presets via `vdsim_lab`.

### Tests
- 201/201 ctest green.

## [0.2.3] — polish

### Fixed
- Full kinematics warning list in sidebar; setup API rejects invalid L3 suspension stems (400).
- L3 level switch resets suspension defaults; fleet launch uses relative susp paths in world YAML.
- `plant.log` tail scan after first full read; cosim stop clears stale warnings.

### Added
- Sample `configs/scenarios/l3_sedan_kinematics.yaml`; ctest `l3_scenario_susp_paths`.

### Tests
- 191/191 ctest green.

## [0.2.2] — modal + cwd + pre-launch warnings

### Fixed
- Edit modal kinematics dropdown uses L3-native list only (preview API unchanged for schematic).
- `vdsim_realtime` launched with `cwd=REPO` for relative suspension paths.
- L3 fleet with missing or topology-only suspension stems surfaces pre-launch warnings.

## [0.2.1] — parts contract hotfix

### Fixed
- `fsk_formula` fleet defaults use L3-native kinematics YAML (`dw_front_sports` / `5link_rear_sports`).
- Saved scenarios persist `configs/suspensions/*.yaml` paths; cosim resolves stem-only refs.
- Fleet suspension selectors gated to L3; infra sensors labelled authoring-only.

### Added
- L3 attachable suspension filter on `/api/suspension/list`; kinematics attach warnings in GUI status.
- ctest `cosim_multi_vehicle` smoke; `SuspensionFactory.RejectsTopologyOnlyYaml`.

### Tests
- 190/190 ctest green.

## [0.2.0] — composable vehicle + scene GUI

### GUI & authoring
- Full-screen `app.html`: fleet tree, 3-tab scenario setup, telemetry, edit modal
  (chassis/suspension/brake/steer/drivetrain/tire/actuator/sensors).
- Suspension 3D preview + hardpoint links; wheel roll animation.
- Data-comms panel (HTTP I/O, UDP telemetry, co-sim controls).
- simconfig v2 (fleet, path, cosim, infra sensors); scenario YAML save/load.
- Part registry API (`GET /api/parts/registry`); fleet `front_susp` / `rear_susp`.

### Runtime
- Multi-vehicle shared world in `vdsim_realtime` (VDS1 vehicle_id).
- L3 native kinematics attach from suspension YAML (`--front-susp` / `--rear-susp`).
- Subsystem deadtime on brake/steer/drive channels.

### Removed
- Legacy `gui/index.html` viewer (all routes serve `app.html`).

### Tests
- 188/188 ctest green (adds `SuspensionFactory.DispatchesByYamlType`).

## [0.1.0] — first public release

Experimental / pre-release. Validated on analytic + ISO standard +
cross-model/cross-tool self-consistency evidence (see `docs/VALIDATION.md`); not
yet cross-validated against a commercial reference on real-vehicle data.

### Vehicle dynamics (C++ core, ISO 8855 RH, wheel order FL=0,FR=1,RL=2,RR=3)
- Ladder models: Lk kinematic, Ld1 bicycle, Ld2 7-DOF, Ld3 14-DOF.
- Pacejka MF96 tire (load sensitivity, relaxation length, camber thrust/Mz,
  combined-slip friction ellipse) + linear fallback.
- Ld4 hardpoint kinematics: double-wishbone / MacPherson / trailing-arm /
  5-link, lookup + native solvers; Adams CSV importer.
- Contact providers: flat, split-mu, inclined, two-tone rough, ISO 8608 PSD,
  heightmap terrain.
- Drivetrain (FWD/RWD/AWD, open/locked/LSD diff, final drive), brake bias/EBD,
  aero, anti-roll bars, road slope/bank load transfer + jacking.
- Low-speed handling (L1/L2/L3): kinematic-dynamic blend below 3 m/s (lateral
  states cross-fade to the slip-free kinematic bicycle, removing the tire-slip
  singularity so the car no longer wobbles, drifts or oscillates when steering or
  stopping at parking speed) plus a viscous brake-hold creep damper that holds the
  car on a grade (cm/s creep) without ringing. Validated dynamics above 3 m/s are
  unchanged. See `docs/design/LOW_SPEED_HANDLING.md`.

### Tooling & interfaces
- Python API (`vdsim` pybind module) + fluent experiment layer (`vdsim_lab`):
  Vehicle / Tire / Road / Maneuver / Sensors builders, metrics, CSV/TUM logging.
- Batch / campaign runner (sweep + Monte Carlo) → summary CSV.
- Sensor models (GNSS/INS/IMU/wheel-speed/steer, noise+bias+delay, mount-pose).
- Operating-point linearization (A,B export); in-loop observer slot.
- Real-time runtime (`vdsim_realtime`, VDS1 binary protocol) — VDSim's single
  real-time application for SIL/HIL/co-sim; the web viewer subscribes to it and
  relays control (one plant, one source of truth). Python protocol mirror.
- FMI 2.0 Co-Simulation export (L2/L3 FMUs) + import of any CS FMU.
- Three.js web viewer + experiment authoring builder + suspension editor.
- CARLA bridge (VDSim physics ↔ CARLA render/sensors).
- ISO 7401 / 4138 / 3888-2 validation maneuvers; DoE runner.

### Platforms
- Cross-platform: builds and runs on Linux (g++/clang) and Windows (MSVC;
  Winsock2 sockets) — 187/187 ctest verified on both.

### Validation
- 187/187 ctest green; FMI round-trip Δ=0; ISO 8608 PSD RMS within tolerance per
  road class. See `docs/VALIDATION.md` for the benchmark matrix and honest
  limitations.

[0.2.0]: https://github.com/Onlyti/VDSim/releases/tag/v0.2.0
[0.1.0]: https://github.com/Onlyti/VDSim/releases/tag/v0.1.0
