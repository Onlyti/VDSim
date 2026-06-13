# Changelog

All notable changes to VDSim are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow
[SemVer](https://semver.org/).

## [Unreleased]

### Added — L1 powertrain (drivetrain v2 on the bicycle)
- The drivetrain v2 engine map + N-speed gearbox + shift policy now runs on the L1 bicycle
  (previously L2/L3 only). Opt-in via the `powertrain:` block; the engine is advanced once
  per RK4 substep and its gear-dependent reflected inertia is added to the front/rear axle
  spin inertia. `engine_rpm()` / `current_gear()` / `set_shift_policy()` are live on L1.
  Default-off keeps the legacy flat torque, so the ISO baseline is unchanged.
- Tests `DrivetrainV2.L1*` (accel/upshift/sawtooth, idle-clutch launch, programmatic shift
  policy, flat-path-has-no-engine-state). Theory ch.22 scope updated.

### Changed — shared multibody math (Ld4 v0.6 M1)
- Extracted the rotation + corner-inertia math duplicated between the hard-joint corner DAE
  and the Featherstone revolute tree into `vdsim/multibody_math.{hpp,cpp}` (`rodrigues`,
  `axis_angle_to_R`, `corner_inertia_about_axis`, `lump_corner_about_axis`). Single source
  for the inertia model; behavior bit-identical (332 ctest).

### Added — user module plugins (build / check / register)
- Ship a C++ subsystem module as a runtime-loadable `.so` without forking the build. Plugin
  ABI `vdsim/module_plugin.hpp` (`VDSIM_REGISTER_*_MODULE` macros) + dlopen loader
  `load_module_plugin()` / `install_module()`. Templates in `templates/modules/`.
- Workshop tool `tools/module_workshop.py`: build a user `.cpp` (compiler errors surfaced as
  the cause), contract-check it with `vdsim_module_check` (ABI / kind / finite-shaped probe
  output → pass/fail + cause), and register the passed `.so` as a `module_plugin_v1` catalog
  part under the `gui_custom` package. Theory ch.24.
- Runtime consumption: a blueprint's `module_plugins:` list resolves into the vehicle config,
  and `install_module_plugins_from_yaml()` loads + installs each plugin after
  `initialize()` (wired into `vdsim_realtime`). So a registered module is usable in a scene
  like any part — uniform for all five kinds (incl. suspension/ARB).
- Tests: `ModulePlugin.*` (load + install + run + from-YAML), `module_workshop` (build/check/
  register + fail-with-cause). Sample `examples/modules/custom_brake.cpp`. **328 ctest.**
- GUI **Module Workshop** (setup "Modules" tab): pick a kind, download the template, point at a
  folder + module name, **Build & Check** (적격/문제 + cause), and **Register**. Endpoints
  `/api/module/{template,build_check,register}` wrap `tools/module_workshop.py`.

### Added — split powertrain / drivetrain parts (re-taxonomy step 1)
- Engine and transmission are now separable catalog parts: a `powertrain` part type
  (`powertrain_v1`, the engine block) and a `drivetrain` part (`drivetrain_v3`, gearbox +
  shift + diff). The resolver shallow-merges the `powertrain:` block by top key
  (engine/gearbox/shift) so the two parts compose into one powertrain. Additive — the
  combined `drivetrain_v2` part still works. Parts `powertrain.sedan_engine` +
  `drivetrain.sedan_gearbox`, blueprint `vehicle.sedan_split_powertrain`; covered by
  `test_catalog_resolver`.

### Added — catalog drivetrain_v2 part
- Schema `drivetrain_v2`: a drivetrain catalog part can carry a `powertrain:` block, which
  the resolver merges into the vehicle config (lands at the root for `parse_powertrain`).
  Part `drivetrain.sedan_v2` + blueprint `vehicle.sedan_powertrain`; previously the
  powertrain demo had to be loaded standalone via `from_yaml`. Covered by
  `test_catalog_resolver` (resolved L2 model runs a real engine).

### Fixed — open-diff reflected engine inertia
- The open differential now uses the correct coupled 2x2 axle mass matrix
  (`open_axle_spin_accel`): the carrier inertia `I_e` is geared to the wheel mean, so
  symmetric acceleration feels `I_e/2` per wheel while wheel-to-wheel differences stay
  free. The old `couple_open_axle_spin` blend was a no-op under symmetric accel (engine
  inertia invisible to straight-line launch) and pulled its inertia from the legacy
  final-drive reflection while the per-wheel divisor used the gearbox value — so with a
  powertrain enabled the reflected inertia could be dropped entirely. Both seven_dof (L2)
  and free_3d (L5). Legacy/no-powertrain path is bit-identical (tight ISO/accel/weight
  gates unchanged). Tests `DrivetrainV2.OpenDiffReflectsGearInertiaIntoSpin`,
  `OpenDiffInertia.EngineInertiaSlowsSymmetricLaunch`. Theory ch.22. **323 ctest.**

### Added — User-defined subsystem modules
- Replace any built-in subsystem with a custom one (C++ subclass or Python subclass):
  `BrakeModule` / `SteeringModule` / `DrivetrainModule` (L2/L3/L4/L5), `SuspensionModule` /
  `AntiRollBarModule` (L3/L4). Install via `model.set_*_module(obj)`; returns False where the
  level does not host that module. The interfaces (`vdsim/subsystems.hpp`) are exposed to
  Python via pybind trampolines; `SubsystemContext`/`DriverCmd`/`SteeringOutput`/
  `CornerInput`/`AxleDefl` are bound.
- Cadence: `begin_step(ctx, dt)` once per step (step-coherent state); `apply()`/
  `wheel_torque()`/`force()` once per RK4 stage (pure). Brake/drivetrain `wheel_torque` is a
  **signed** torque opposing wheel spin (FL,FR,RL,RR).
- A delegating wrapper reproduces the baseline bit-for-bit (`UserModules.WrapperIsTransparent
  {L2,L3}`); effect + cadence + level-scoping gated by `UserModules.*`. Sample
  `examples/user_brake_module.py` (Python ABS-style brake). Theory **ch.23**. **321 ctest.**

### Added — Drivetrain v2 (engine + gearbox, opt-in)
- `powertrain:` vehicle-config block enables a real powertrain on L2/L3 (absent -> legacy
  flat torque, ISO baseline unchanged). `powertrain.hpp`.
- **2D engine torque map** `T_peak(rpm, throttle)` (bilinear, domain-clamped; the
  closed-throttle row is engine braking). **N-speed gearbox**: engine RPM coupled to the
  driven-wheel speed, axle torque = `T_eng * gear * final_drive * efficiency`,
  **gear-dependent** reflected inertia `I*(gear*fd)^2`, idle floor + slipping launch clutch.
- **Shift policy**: built-in `manual` / `auto_rpm` (hysteresis) plus a **user-defined
  function** `f(ShiftContext) -> gear` via `set_shift_policy` (C++ `std::function`; pybind
  accepts a Python callable). Shift-time torque interrupt + lock-out.
- Accessors `engine_rpm()` / `current_gear()` (+ pybind). Theory **ch.22**; sample
  `configs/powertrain_sedan_demo.yaml`.
- Tests `EngineMap.*`, `PowertrainYaml.*`, `EngineGearbox.*`, `DrivetrainV2.*`.

### Added — tire / validation (post-0.5.1)
- Bicycle (L1) belt transient (MF + LuGre) — belt now on L1–L5. `BeltTransient.L1*`.
- `tire_forces_wheel()` accessor (tire-frame per-wheel force) + pybind; `TireFrame.*`.
- Chrono Pac02 parity gate (`external/chrono_parity/`, isolated): pure-long Fx ~2% +
  pure-lat Fy ~1% gated; `ctest -R ChronoPac02Parity`.
- ISO re-baseline (flat) + CI gate `IsoBaseline`.

## [0.5.1] — 2026-06-10 · tire layers: MF2002 `.tir` backend (T1) + belt transient (T2)

Adds a measured-coefficient steady-force backend and a carcass/belt transient layer
on top of the MF96 + LuGre stack. Both opt-in; the default MF96/LuGre preset is
unchanged (no ISO/L2/L3 force drift). GUI `.tir` import is deferred to v0.5.2.

### Added — T1 MF2002 backend
- `TireParams.backend` (`"mf96"` default | `"magic_formula"` | `"linear"`) + `tir_path`;
  `create_tire_from_params()` dispatch wired into every dynamics `initialize()`.
- MF2002 evaluator (`magic_formula_tire.cpp`) full combined slip ($G_{x\alpha}, G_{y\kappa}$
  + SVyk, combined $M_z$); `model_provides_combined_slip()` gates the host friction-ellipse
  clip so MF2002 / LuGre forces are not re-clipped.
- `from_yaml` selects the backend, so cosim + batch run a `.tir` directly. `.tir` files
  stay uncommitted (gitignore + confidential).
- Tests `Mf2002Catalog.*` (synthetic `.tir` written to temp at runtime).

### Added — T2 belt transient
- `vdsim/belt_tire.hpp` `belt_relax()`: first-order slip relaxation, $\tau = \sigma/|V_x|$,
  exact exponential update, frozen at standstill.
- Opt-in `TireParams.belt {enabled, sigma_lat, sigma_long}`; wired into seven_dof (L2 /
  L3-inner) and free_3d (L5), both the MF path (relax κ/α) and the LuGre path (relax slip
  velocity). **Default off.**
- Theory **ch.21** (belt transient). Tests `BeltTire.*`, `BeltTransient.*`,
  `BeltValidation.*` (steady unchanged, early response suppressed at t=τ, more lag at
  lower speed). **291/291 ctest green.**

### Validation
- **ISO re-baseline (flat)**: refreshed the sedan L2 LuGre ISO 7401/4138/3888 numbers
  in `VALIDATION.md` (stale neutral/0.63 g table → understeer 24.8 mrad/g, 0.85 g) after
  the LuGre-default + drivetrain-inertia force changes.
- **CI gate** `tests/integration/test_iso_baseline.cpp` (`ctest -R IsoBaseline`): locks
  the four force-sensitive step-steer metrics (ψ̇_ss, peak, overshoot, a_y_ss) on the
  shipped default preset so future force drift fails the build. **292/292 ctest.**

### Decision
- Keep VDSim's own lean tire stack; Chrono Pac02 (BSD-3) is a cross-validation
  *reference*, not a dependency. No permissive OSS belt model exists → T2 is in-house
  from Pacejka 3rd ed. Ch.7/9.

### Deferred
- bicycle (L1) belt wiring (lowest value); Chrono Pac02 parity gate (needs Chrono build);
  GUI `.tir` import (v0.5.2 GUI bundle).

## [0.5.0] — 2026-06-09 · terrain + L5 general driving (headless / batch / cosim)

Generalizes the v0.4 Ld5 stunt body to driving on arbitrary ground. GUI terrain
load + L5 Play and a stunt-scene authoring panel are deferred to v0.5.2.

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

### Deferred (v0.5.2)
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
