# VDSim 핸드오프

작성: **2026-06-21** · `main` (local) · **388/388 ctest green**

> **Structured review (2026-06-21):** portable fixes on `main` worktree — see
> [`docs/evidence/review/REVIEW_2026-06-21.md`](evidence/review/REVIEW_2026-06-21.md).
> VLA plant lane lives on `VDSim-Thesis` only.

## 0. 2026-06-23 CmdL1 per-wheel torque binding fix — DONE

**Bug:** `def_readwrite` on `std::array<double,4>` returned a copy — `cmd.motor_torque[i]=x` was silent no-op.  
**Fix:** `MutableWheelArray` in-place view (`__getitem__`/`__setitem__`) on `CmdL1.motor_torque` / `brake_torque`.  
**Test:** `tests/scripts/test_cmdl1_binding.py` (ctest `cmdl1_binding`).

## 0. 2026-06-20 core quality + T3 VLOW — DONE

**Shipped:**
- **`SessionKind`** — `Standard` / `DirectControl` on `SimConfig` (replaces `direct_control_path`)
- **KC compliance** — bushing toe/camber under lateral tire load in L3/L4 hard-joint step
- **T3 VLOW** — `TireParams.low_speed_mode` + `vlow_speed_eps`; tests `TireVlow.*`
- **Chrono KC** — `gen_kc_reference` fills reference CSV; `ChronoKcParity` **active** (rebound reach ~−14 mm)

**Verify:** `cmake --build build -j && cd build && ctest --output-on-failure`

## 0. 2026-06-20 thesis → main core port (partial cherry-pick) — DONE

**Tagged:** `vdsim-thesis-beta` @ `6623f55` — VLA plant beta delivery freeze on `VDSim-Thesis`.

**Cherry-picked to `main`:** `c4b5c0a` (+ fixup) — core only, **no** `python/vdsim_plant.py` / VLA integration tests.
- TireSetup per-axle/per-wheel + catalog/cosim/pybind + PerAxle/PerWheel tests
- `IVehicleDynamics::tire(wheel)`, `make_direct_control_session`, `FrictionMapConfig`
- steer_deadtime via ActuatorModel; `ladder_lowering.hpp`; SimOutput wheel GT fields
- `VehicleParams.plant_path` (default false; required by L2 plant-path branch)

**Still thesis-only:** VLA plant wrapper (24 commits + `e5fb2c6`), `drive_split_front` (#6 deferred).

**Verify:** `cmake --build build -j && cd build && ctest --output-on-failure`

## 0. 2026-06-14 세션 인수인계 (fresh 세션은 여기부터)

**목표였던 것:** 멀티차량 비교(헤드리스+GUI) + 다나와식 차량조립 UI + 외부 AI 디자인 리뷰 반영. 완료.

**현재 상태:** `main` local (ahead of origin) · **385/385 green** · thesis core port done · VLA on `VDSim-Thesis` only.

**이번 세션 main 에 들어간 것:**
- **헤드리스 멀티차량 비교** `tools/vdsim_compare.py` — preset N대에 같은 ISO maneuver(step-steer/skidpad/DLC) 돌려 지표 나란히 표 + bar-chart(compare.csv/png). `examples/maneuvers.py` 함수에 optional `veh=(vp,tp)` + `trace=True`(step_steer yaw-rate 시계열).
- **다나와식 build-sheet** (assembly 모달 재설계) — 좌측 슬롯 리스트(Body&chassis/Powertrain/Grip/Control 그룹, 각 행=부품+change) → 클릭 시 우측 그 슬롯 라이브러리 + RESOLVED STATS. `assembly.py` 카테고리 새 taxonomy 로 재그룹.
- **GUI Compare 대시보드** — VEHICLES 패널 ⊟ → blueprint 체크박스 → `/api/compare`(routes.py, `vdsim_compare` 래핑) → **step-steer yaw-rate overlay SVG 차트** + **Δ%(baseline 대비) 표**(유효숫자 통일, 정성/정량 분리). `tests/scripts/test_compare.py`(ctest `multi_vehicle_compare`).
- **GUI 폴리싱**(디자인 리뷰 반영) — RESOLVED STATS 단위, active 행 회색+좌측 accent, 3D 프리뷰 우측, Sync-draft 표준 크기.

**다음 할 일 후보 (우선순위 순):** see [`ROADMAP.md`](ROADMAP.md) §15.

1. **P0** — ~~tag **v0.6.1**~~ / ~~**v0.6.2**~~ **done** (`v0.6.1` @ `5cdca4e`, `v0.6.2` @ `9a40c1b`).
2. **P1** — **#1 Fx→τ** C++ contract, **#6 drive_split** / brake bias design.
3. **P2** — ~~Chrono KC `gen_kc_reference` API fill-in~~ **done** — CSV committed, `ChronoKcParity` active.
4. **P3** — v0.5.2 GUI: terrain Play **or** `.tir` import; **GUI v2** framework decision.
5. **Thesis** — VLA hotfix only on `VDSim-Thesis`; plant → main PR after client sign-off.

**주의·함정:**
- retax2 **breaking** — 옛 `chassis.*`/`susp.*` part id, `front_susp_kin` 슬롯 제거됨.
- `from_yaml` 은 envelope(`body:`)를 unwrap 안 함 — IsoBaseline 은 `body/sedan.yaml`(→struct defaults) 로딩. 건드리지 말 것.
- ISO/Default subsystem **숫자 리베이스 금지**. 영상 git 커밋 금지. 현대 `.tir` 대외비.
- GUI 서버 띄울 때 **`run_in_background` 사용**. `pkill -f "gui/server.py"` 는 자기 명령줄 self-match 로 죽으니 `pkill -f "server.py --port"` 처럼 좁혀서.
- 멀티차량 = **비교용**, V2V 충돌 scope 밖 (메모리에 기록됨).

**관련 경로:** `tools/vdsim_compare.py`·`examples/maneuvers.py` (비교) / `gui/static/app.js`(openCompareModal/renderCompareChart/renderAssemblyPane)·`gui/api/routes.py`(/api/compare)·`python/catalog/assembly.py` (GUI) / `cosim/realtime_server.cpp` (멀티차량 런타임).

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
cd build && ctest --output-on-failure   # 388/388 (main)
python3 gui/server.py                   # http://127.0.0.1:8080
```

## 4. 다음 작업

**Current sprint:** [`ROADMAP.md`](ROADMAP.md) §15 (development priorities).

**Shipped on `main` (2026-06-20):** SessionKind, KC compliance, T3 VLOW. **388/388 ctest.**

**Deferred (v0.5.2, browser):** GUI terrain Play, stunt authoring, `.tir` import;
tire-force arrows in wheel frame in 3D view.

<details>
<summary>Historical archive (v0.4–v0.5 tire stack)</summary>
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

**User-defined subsystem modules (this session):** any built-in subsystem can be
replaced by a C++ or Python subclass — `BrakeModule`/`SteeringModule`/`DrivetrainModule`
(L2/L3/L4/L5), `SuspensionModule`/`AntiRollBarModule` (L3/L4). Install via
`model.set_*_module()` (`IVehicleDynamics`, mirrors `set_shift_policy`; seven_dof + free_3d
hold brake/steer/drivetrain, fourteen_dof holds suspension/ARB + delegates the rest to
`inner_`, L4 inherits from it). L1 computes brake/drive inline -> hosts none. Subsystem members are
`shared_ptr` so a pybind-trampoline Python subclass stays alive. Cadence: `begin_step` once
per step, `apply/wheel_torque/force` per RK4 stage; brake/drivetrain torque is **signed**
(opposes spin). Tests `UserModules.*` (transparency L2/L3 to 1e-9 + effect + scoping),
sample `examples/user_brake_module.py`, theory **ch.23**. **321 ctest.** **Remaining:** GUI
module workshop; L1 has no subsystem objects (out of scope).

**v0.6+:**
1. Ld4 v0.6 — shared inertia helpers **(done)**; full loop dynamics / Featherstone in step
   deemed not needed (hard DAE already solves loop kinematics — see LD4_MULTIBODY.md §7).
2. Drivetrain: catalog `drivetrain_v2` part **(done)** + L1 powertrain **(done)**;
   engine-map workshop (UI) remaining.
3. Tire: T2 belt is **complete** (L1–L5); higher belt eigenmodes (rigid-ring/FTire) out of scope.

</details>

## 5. 주의

- Hyundai (TUR) `.tir` **confidential** — 커밋 금지
- Default subsystem **숫자 리베이스 금지**
- Wheel: **FL=0, FR=1, RL=2, RR=3** · ISO 8855 RH
- Cosim plant **v5** ↔ rebuilt `vdsim_realtime`
- `configs/.resolve_cache/` — 커밋하지 않음

## 6. Git

- `main` local, **ahead of origin** · **388/388 ctest** · tags **`v0.6.0`** (MBD) ·
  **v0.6.1** (core port) + **v0.6.2** (quality/T3) after commit/tag.
- **External build (not in repo):** Chrono 8.0.0 source at `~/build_ext/chrono_src`
  (vehicle module only) — only needed to *regenerate* the parity CSV, not to run the gate.
- Next: new feature branch off `main`.

## 7. Next pickups (fresh session)

See [`ROADMAP.md`](ROADMAP.md) §15. Summary: tag v0.6 → Fx→τ contract → KC compliance → GUI v0.5.2 pick-one.
