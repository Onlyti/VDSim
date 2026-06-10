# VDSim 핸드오프

작성: **2026-06-10** · `main` @ `f6b1d71` (ahead of `v0.5.1`) · **317/317 ctest green**

## 1. 문서 인덱스

| Topic | User / ops | Design spec |
|-------|------------|-------------|
| Catalog + scenes | [`CATALOG_AND_PHYSICS.md`](CATALOG_AND_PHYSICS.md) | [`design/PARTS_CATALOG.md`](design/PARTS_CATALOG.md) |
| **Ld4 multibody M1–M7** | § L4 in catalog doc | **[`design/LD4_MULTIBODY.md`](design/LD4_MULTIBODY.md)** |
| v0.4 slope / jump / loop | [`RUNNING.md`](RUNNING.md) | [`design/V0.4_SLOPE_JUMP_DYNAMICS.md`](design/V0.4_SLOPE_JUMP_DYNAMICS.md) |
| v0.5 terrain + L5 | — | [`design/V0.5_TERRAIN_L5.md`](design/V0.5_TERRAIN_L5.md) |
| LuGre tire | theory ch.19 | [`design/V0.2_TIRE_LUGRE.md`](design/V0.2_TIRE_LUGRE.md) |
| Roadmap | [`ROADMAP.md`](ROADMAP.md) | [`design/TIRE_ROADMAP.md`](design/TIRE_ROADMAP.md) |
| Validation | [`VALIDATION.md`](VALIDATION.md) | — |

## 2. 브랜치 스냅샷

### v0.4.0 stunt (Ld5) — shipped + tagged

- `create_stunt_dof()` → `Free3DDynamics` (`free_3d_dynamics.cpp`): pos+quat+ω, 4-wheel MF, no rail snap. **유일한 stunt plant** — `create_legacy_stunt_dof()`(L3+L2 rail loop)는 cleanup 에서 제거됨.
- Scenes `jump_ramp_demo.yaml` / `vertical_loop_demo.yaml`; cosim `stunt:` → `make_ground`; GUI stunt mesh + z/pitch/roll telemetry; `settle_spawn_on_ground`.
- LuGre default retune for L5 loop (`sigma0` 9e4, `sigma2` 75). Theory ch.20.

### v0.5.0 terrain + L5 — shipped + tagged (headless/batch/cosim)

- Hub contact 통일: Flat/SplitMu/Inclined/Rough/Heightmap → `wheel_world_positions` + `hub_penetration`.
- **`CurvedGround`** banked turn (`create_curved_ground`, cosim `stunt.ground == banked`).
- Scenes `terrain_hill_demo` / `banked_grade_demo` / `banked_oval`; `tools/bake_synthetic_hill.py` + `assets/terrain/hill_demo.bin`; materialize `terrain:` forwarding.
- Tests `tests/integration/test_terrain_l5.cpp` (7) + `test_l5_driving.cpp`.
- **v0.5.2 deferred (browser):** M4 GUI terrain Play, M5c GUI stunt 저작.

### GUI cleanup (이번 세션)

- `app.html` 4873→~250L shell; inline CSS/JS → `gui/static/{app.css, app.js, util.js, minimap.js, fields.js, manual.js}`; `/static/` route.
- legacy scene loaders 제거, low-speed 상수 → `vdsim/low_speed.hpp`. 검증: esbuild bundle 게이트(node20 @ ~/.nvm).

### Ld4 multibody (M1–M7) — shipped

상세: [`design/LD4_MULTIBODY.md`](design/LD4_MULTIBODY.md)

| M | Runtime | Summary |
|---|---------|---------|
| M1 | L4 | FK attach, `create_fourteen_dof_kinematic()` |
| M2 | — | Topology graph from kin YAML |
| M3 | design | Quasi-static bushing compliance |
| **M4** | **L4 step** | Hard-joint DAE (TA/MP/DW/5-link) + Baumgarte travel |
| M5 | GUI/py | K&C sweep charts |
| M6 | CLI/py | Adams CSV import x-check (5 gains, 5% rtol) |
| M7 | offline | Lumped 1-DOF revolute RNEA (TA/MP/DW/5-link) |

### GUI / cosim (v0.2 carry-over)

- Vehicle Edit: Chassis / Tire / Suspension tabs; FL FR RL RR
- VDS1 protocol **v5** (`throttle_applied`, `brake_applied`)
- L4 in `make_dyn("L4")`, catalog, `l4_sedan_kinematics.yaml`

## 3. 빌드·검증

```bash
cmake --build build -j
cd build && ctest --output-on-failure   # 317/317
python3 gui/server.py                   # http://127.0.0.1:8080
```

## 4. 다음 작업

**Shipped:** v0.4.0 (stunt Ld5 + Ld4 multibody + theory ch.20) and **v0.5.0**
(terrain + L5, headless/batch/cosim) are tagged on `main`. v0.5.0 work landed on
`feat/v0.5-terrain-m1`: M0 hub contact · M1 heightmap (no-sink/climb/flank) · M2 cliff
airborne · M3 terrain scene+bake · M5 inclined/banked · M5b CurvedGround banked turn ·
M6 docs. 273 ctest. Scenes: `terrain_hill_demo`, `banked_grade_demo`, `banked_oval`.

**Tire stack (this session — T1 + T2, merged to `main`):** see
[`design/TIRE_ROADMAP.md`](design/TIRE_ROADMAP.md) §0/§4 + theory ch.21.
- **T1 MF2002**: `TireParams.backend` ("mf96" default | "magic_formula" | "linear")
  + `tir_path`; `create_tire_from_params` dispatch in every dynamics `initialize()`;
  MF2002 combined slip bypasses the host friction ellipse. `.tir` stays uncommitted
  (gitignore + confidential). Tests `Mf2002Catalog.*`.
- **T2 belt transient**: `vdsim/belt_tire.hpp` `belt_relax()` (tau=sigma/|Vx|, exact
  exp); opt-in `TireParams.belt {enabled, sigma_lat, sigma_long}`; wired in seven_dof
  (L2/L3) + free_3d (L5), both MF (relax kappa/alpha) and LuGre (relax slip velocity).
  Tests `BeltTire.*`, `BeltTransient.*`, `BeltValidation.*`. **Default off -> no drift.**
- **Decision (locked):** keep own lean tire stack; Chrono Pac02 (BSD-3) is a
  cross-validation *reference*, not a dependency. Belt = own (no permissive OSS exists).
- **Chrono Pac02 parity (DONE):** isolated cross-check in `external/chrono_parity/`
  (Chrono 8.0.0 built from source at `~/build_ext/chrono_src`; never in VDSim's CMake).
  Generator `gen_reference.cpp` (ChTireTestRig) -> committed `reference/pac02_reference.csv`;
  VDSim gate `ctest -R ChronoPac02Parity`. **Pure-long Fx ~2% AND pure-lat Fy ~1%** vs an
  independent Pac02 — both gated (backbone + load sensitivity + cornering stiffness
  validated). Combined cross-terms differ but that is largely a rig-frame artifact
  (`ChTireTestRig` reports global-frame force on a yawed wheel) mixed with weighting —
  report-only. See `external/chrono_parity/README.md`.
- **Tire T2 complete:** bicycle (L1) belt wiring done (MF + LuGre) — belt now on
  L1/L2/L3/L5, default off. Tests `BeltTransient.L1*`. **Remaining tire:** GUI `.tir`
  import (v0.5.2 GUI). (Combined-slip Pac02 alignment is low priority — pure axes already
  match; the residual is mostly the rig-frame decomposition, not our model.)

**v0.5.2 (deferred — needs browser, no headless path):**
1. **M4 GUI terrain load + L5 Play** — chase cam uses `position.z`, spawn on mesh.
2. **M5c GUI stunt authoring** — author ramp/loop/banked scenes (today render-only).
3. GUI `.tir` import (Tire T1 tail).
4. **Tire-force arrows in wheel frame** — model now exposes `tire_forces_wheel()`
   (un-rotated, validated by `TireFrame.*`; also in pybind). The GUI still draws the
   body-frame `tire_forces_body()` along body axes (arrows don't tilt with steer); to
   show tire-frame, send the wheel-frame force over VDS1 and parent the arrows to the
   steered wheel mesh (`gui/static/app.js makeForceArrow` / update L793-794).

**Done this session (post-tire):**
- ISO re-baseline (flat, sedan L2 LuGre) — VALIDATION.md table refreshed + CI gate
  `tests/integration/test_iso_baseline.cpp` (`ctest -R IsoBaseline`) locks the 7401 signature.
- **Drivetrain v2 (opt-in `powertrain:` block, L2/L3):** 2D engine torque map + N-speed
  gearbox (RPM coupling, gear-dependent reflected inertia, idle-floor/launch clutch) +
  shift policy (manual / auto-RPM / **user-defined function**, pybind callback). Accessors
  `engine_rpm()`/`current_gear()`. `powertrain.hpp` `EngineGearbox`,
  `EngineGearboxDrivetrain` in default_subsystems. Theory ch.22, sample
  `configs/powertrain_sedan_demo.yaml`. Default flat torque unchanged -> ISO green.
  Tests `EngineMap.*`/`PowertrainYaml.*`/`EngineGearbox.*`/`DrivetrainV2.*`. **317 ctest.**
  **Remaining:** catalog `drivetrain_v2` part (materialize powertrain into the chassis);
  L1 + engine-map GUI workshop.

**v0.6+:**
1. Ld4 v0.6 — shared inertia helpers; full loop dynamics; Featherstone in step (optional).
2. Drivetrain: catalog `drivetrain_v2` part + engine-map workshop (UI). Core done.
3. Tire: T2 belt is **complete** (L1–L5); higher belt eigenmodes (rigid-ring/FTire) out of scope.

## 5. 주의

- Hyundai (TUR) `.tir` **confidential** — 커밋 금지
- Default subsystem **숫자 리베이스 금지**
- Wheel: **FL=0, FR=1, RL=2, RR=3** · ISO 8855 RH
- Cosim plant **v5** ↔ rebuilt `vdsim_realtime`
- `configs/.resolve_cache/` — 커밋하지 않음

## 6. Git

- `main` @ `f6b1d71`, **all pushed**, 317/317 ctest. Last tag **`v0.5.1`**; main is ahead
  of it (post-0.5.1, **untagged**): tire tail (L1 belt, `tire_forces_wheel`, Chrono parity)
  + ISO re-baseline/`IsoBaseline` + **Drivetrain v2** (D1–D5). Tag a release when the next
  milestone closes (suggest `v0.6.0` once catalog `drivetrain_v2` or Ld4 v0.6 lands).
- Tags: v0.1.0 / v0.2.0 / v0.2.4 / v0.4.0 / v0.5.0 / **v0.5.1**.
- All feature work committed straight on `main` this session (no branches/rewrite).
  push/merge/tag는 명시 요청 시에만.
- **External build (not in repo):** Chrono 8.0.0 source at `~/build_ext/chrono_src`
  (vehicle module only) — only needed to *regenerate* the parity CSV, not to run the gate.
- Next: new feature branch off `main`.

## 7. Next pickups (fresh session)

1. **Catalog `drivetrain_v2` part** — materialize a `powertrain:` block into the chassis
   params via the catalog resolve pipeline (today the demo loads `powertrain_sedan_demo.yaml`
   directly through `from_yaml`). Headless.
2. **Ld4 v0.6** — shared inertia helpers; full loop dynamics; Featherstone in step (optional).
3. **v0.5.2 GUI (browser, user-verified):** terrain Play (M4), stunt authoring (M5c),
   `.tir` import, tire-force arrows in wheel frame (`tire_forces_wheel()` ready).
4. Brake/steer physics (booster/MDPS/ABS); V2V collision + VDS1 multi-vehicle I/O.
