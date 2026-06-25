# VDSim 핸드오프

작성: **2026-06-25** · **`VDSim-Thesis`** (active) · **404/404 ctest green**

> **Governance (2026-06-25):** demand-gate active — new fidelity PRs need VDSim-PO named puller (`CLAUDE.md`).

## 0. 2026-06-25 Product freeze boundary — ACTIVE

**GUI v3:** preserved @ P0 parity gap 0 (commit `feat(gui): v3 preserve`).  
**FROZEN** — no further GUI polish/features until ≥2 scenario-authoring segment adoption failures cite GUI absence.

**Plant API (P1 wedge):** open — thesis / external MPC (`vdsim_plant`, CmdL1 path).

**Wheel CI (P0-1):** `.github/workflows/wheels.yml` — tag `v*` → cibuildwheel + PyPI (trusted publisher).  
**HOLD:** no `v0.7.0` tag / PyPI publish / main merge until PO decision-4 gate (PO signal).  
Pre-publish: `python3 tools/verify_wheel_packaging.py wheelhouse/*.whl` (1 public `.tir`, 0 measured).

**P0-2 quickstart:** `examples/quickstart.py` + `vdsim-quickstart`; README cites **~6 s cold** (measured, not "5 min").  
**P0-3 demo:** `examples/demo_grip_loss.py` → `docs/assets/demo_grip_loss.gif` (headless, pillow).

## 0. 2026-06-23 CmdL1 per-wheel torque binding fix — DONE

**Bug:** `def_readwrite` on `std::array<double,4>` returned a copy — `cmd.motor_torque[i]=x` was silent no-op.  
**Fix:** `MutableWheelArray` in-place view (`__getitem__`/`__setitem__`) + `vdsim_plant._fx_to_cmdl1` whole-array assign.  
**Tests:** `test_cmdl1_element_torque_assign`, `test_longitudinal_fx_accel` in `test_vla_plant.py`.

## 0. 2026-06-23 CO-00 scenario template gallery — DONE

**Templates:** Empty · Figure-8 · Straight · Skidpad (R=40 circle)  
**API:** `POST /api/setup/template` · Scenario inspector 2×2 chip gallery  
**E2E:** template chip `button.tpl` locator (accessible name includes hint text)

## 0. 2026-06-23 GUI v3 sim time + RN-02 + SH-05 — DONE

**Sim time:** stream `t` = plant timestamp − first frame (not wall epoch)  
**RN-02:** Run Autopilot/Manual + manbar (↑↓←→ + touch) → `/api/fleet` + `/api/manual`  
**SH-05:** `HelpLink` → mkdocs theory pages (Scenario road/path, Run pose/tire)

## 0. 2026-06-22 GUI v3 Run/Scenario polish — DONE

**RN-06:** per-wheel force triad (Fx/Fy/Fz arrows) in `RunViewport` · Forces toggle  
**Run telemetry:** sim time `t` · per-wheel Fz/Fx/Fy/κ/α table  
**CO-06:** waypoint summary table (downsampled) in Scenario inspector  
**Docs:** concept art README — require live capture pairing for thesis figures

## 0. 2026-06-22 GUI v3 visual P0 — DONE

**P0 fixes (vs concept art review):**
- Scenario map: path from `setup.path_pts` (no 0-point race) · grid axis labels (m) · waypoint # in edit mode
- Run idle: preview vehicle mesh + path at fleet spawn (`RunViewport` setup_mode)
- Play → Run navigation hardened · capture flow waits sync+start · `06-run-sim-*.png`

**Review:** [`docs/evidence/review/GUI_V3_VISUAL_REVIEW.md`](evidence/review/GUI_V3_VISUAL_REVIEW.md) · captures `docs/evidence/review/gui-captures/`

Regenerate: `python3 tools/gui_v3_capture.py` · eval: `python3 tools/gui_v3_eval_bundle.py`

## 0. 2026-06-22 GUI v3 scaffold + Scenario API — DONE

**Concept art:** [`docs/design/assets/`](design/assets/) (6 PNG + README).

**Scaffold:** `gui/v3/` — Svelte 5 + Vite · `#/scenario` default.

**Scenario mode (wired):** `GET/POST /api/setup`, `/api/path`, `/api/fleet` (load scenario),
`/api/scenario/save`, map canvas (path + fleet drag), road μ/grade/bank, path preset, ▶ Play → `/api/run/start`.

**Still placeholder:** — (L4 hardpoint editor shipped in Build mode).

**Scenario path edit (CO-07):** map toolbar **Edit path** · drag ● · click add · Del · Esc.

**Analyze mode (v3.0-beta):** `/api/compare` · ISO maneuvers · yaw-rate overlay · Δ% table · CSV export.

**Build mode (v3.0-beta):** assembly · blueprint **Export/Register** · **Parts library** · Δ stats · linkage SVG · **L4 hardpoint editor** (pick/drag y–z, K&C preview, save to gui_custom).

**Run mode (v3.0-beta):** `/api/stream` SSE · Three.js fleet mesh · chase/orbit/top · telemetry · Stop/Reset.

```bash
python3 gui/server.py --port 8095
cd gui/v3 && npm run build   # or npm run dev
python3 tools/gui_v3_eval_bundle.py   # headless eval → docs/evidence/review/
# http://localhost:8095/v3/#/scenario
```

## 0. 2026-06-21 GUI v3 planning — DONE (superseded by scaffold above)

Direction locked: Scenario-first · Run-second · Svelte 5 + Vite ([`GUI_V3_ADR.md`](design/GUI_V3_ADR.md)).

## 0. 2026-06-20 thesis freeze + core → main port — DONE

**Tag:** `vdsim-thesis-beta` @ `6623f55` (VLA plant beta before core commits).

**Commits on thesis (not all on main):**
- `c4b5c0a` — core (#3–9, per-wheel tire) → **cherry-picked to `main`** (`38b1955` + `5cdca4e`)
- `e5fb2c6` — VDSimPlant wiring → **thesis-only**

**Develop on `main` going forward;** thesis = VLA delivery / hotfix only.

## 0. 2026-06-18 pre-main cleanup (agreed items 3–5,7–9) — DONE

**Shipped (1/6/2 deferred — interface discussion pending):**
- **#4 `tire(wheel)`** — `IVehicleDynamics::tire(int)` on L1/L2/L5; pybind `dyn.tire(wheel=0)`
- **#5 steer lag** — `vp.steer_deadtime_s` → `ActuatorModel` (direct path uses actuator, removed `direct_steer_lag_`)
- **#8 ladder lowering** — `ladder_lowering.hpp` centralises 1500×5 / 600 / 4000 (ISO frozen)
- **#7 friction canonical** — `create_friction_patch_ground` → wide-y polygon + `PolygonFrictionGround`; `make_friction_ground()` + `FrictionMapConfig`
- **#3 session factory** — `make_direct_control_session(vp, TireSetup, sp, opts)`; `make_vla_plant_session` = shim
- **#9 thin plant** — `SimOutput` wheel GT fields; `vdsim_plant._obs_from_output(sess.output())`

**Verify:** `cmake --build build -j && cd build && ctest --output-on-failure &&
python3 tests/scripts/test_vla_plant.py`

## 0. 2026-06-18 per-wheel tire (advanced) — DONE

**Shipped:** FL/FR/RL/RR can each use different `TireParams` on L1/L2/L3/L5 force loops.
Unified (one `TireParams`) and per-axle `(front, rear)` constructors remain backward
compatible. Catalog optional slots: `tire_rear` (RL+RR when no corner slot),
`tire_fr` / `tire_rl` / `tire_rr` → resolved `tire_*.yaml` in fleet row.

**Deliverables:**
- **`TireSetup`** (`params.hpp`) — `wheel[4]` + `for_wheel` / `for_axle`; corner ctor +
  `from_corner_yaml_paths`
- **Dynamics:** 4× `ITireModel` via `init_wheel_tire_models` (was per-axle)
- **Resolver / materialize:** corner yaml paths on `ResolvedVehicle` + fleet row keys
- **Cosim:** `VehicleSpawn.tire_{fr,rl,rr}_yaml` + `load_tire_setup`
- **pybind:** `wheel`, `front`/`rear` properties, `from_corner_yaml_paths`
- **Tests:** `PerWheelTire.L2FrontLeftRightDifferentGrip` (+ existing `PerAxleTire`)

**Verify:** `cmake --build build -j && cd build && ctest -R 'PerAxleTire|PerWheelTire' --output-on-failure`

## 0. 2026-06-18 per-axle tire (core) — DONE

**Shipped:** front/rear axle can use different `TireParams` (MF96/MF2002/LuGre/belt) on
L1/L2/L3/L5 force loops. Single `TireParams` still duplicates to all corners (backward
compatible). Catalog optional `tire_rear` slot → `tire_rear.yaml` in resolved fleet.

**Deliverables:**
- **`TireSetup`** — per-axle ctor / `from_yaml_paths`; shim `initialize(vp, TireParams, sp)`
- **Dynamics:** per-wheel `ITireModel` routing (same axle shares params unless corner slot set)
- **Resolver:** `tire_rear` optional blueprint slot; `ResolvedVehicle.tire_rear_yaml`
- **Cosim:** `VehicleSpawn.tire_rear_yaml` → `TireSetup::from_yaml_paths`
- **pybind:** `TireSetup`, `initialize_setup`
- **Tests:** `PerAxleTire.L2FrontRearDifferentGrip`, `PerAxleTire.L1UnifiedMatchesSingleTireParam`

**Verify:** `ctest -R PerAxleTire --output-on-failure`

## 0. 2026-06-18 VLA plant P2 — smooth 1-D friction blend + steer lag docs — DONE

**Shipped (customer P2, `docs/PLANT_CUSTOMER_FEEDBACK.md`):**
- **1-D `friction_map` smooth transition** — `FrictionPatchGround` linear blend over **1.0 m**
  before `x0` and after `x1`; outside `x < x0−1` / `x > x1+1` → `base_mu`; overlapping
  patches → min μ (same rule as 2-D polygon).
- **`VehicleParams.steer_deadtime_s`** — 1st-order steer lag (τ) on VLA direct-control path
  (`SimSession::tick` when `direct_control_path`); default 0 = instantaneous ZOH δ.
- **Docs:** `docs/VDSIM_PLANT_TUTORIAL.md` §6a (1-D blend), §7 (steer lag manual workflow).

**Deliverables:**
- `core/src/contact_providers.cpp` — `FrictionPatchGround` blend
- `core/src/sim_session.cpp` — direct-path steer lag from `vp.steer_deadtime_s`
- **Test:** `VlaPlant.FrictionPatchSmoothTransition`

**Verify:** `cmake --build build -j && cd build && ctest -R VlaPlant --output-on-failure`

## 0. 2026-06-18 VLA plant P1 — roll/pitch obs + 2D polygon friction map — DONE

**Shipped (customer P1, `docs/PLANT_CUSTOMER_FEEDBACK.md`):**
- **`obs["roll"]`, `obs["pitch"]`** [rad] — quasi-static angles from `roll_angle_qs()` /
  `pitch_angle_qs()` (L2–L5; L1→0). Already on `IVehicleDynamics` + pybind; wired in
  `python/vdsim_plant.py` `_obs()`.
- **`friction_map_2d`** — `[{"polygon": [(x,y),...], "mu": 0.5}, ...]` on `VDSimPlant`
  (mutually exclusive with 1-D `friction_map`). C++ `PolygonFrictionGround` +
  `create_polygon_friction_ground()` in `contact_providers.cpp`: per-wheel world (x,y),
  point-to-polygon boundary distance, linear blend to `base_mu` over **1.0 m** global;
  overlapping patches → min μ.

**Deliverables:**
- `core/include/vdsim/interfaces.hpp` — `PolygonMuPatch`, factory decl
- `core/src/contact_providers.cpp` — `PolygonFrictionGround`
- `python/bindings.cpp` — `PolygonMuPatch`, `create_polygon_friction_ground`,
  `make_vla_plant_session(..., poly_patches, blend_distance)`
- `python/vdsim_plant.py` — `friction_map_2d`, roll/pitch in obs
- **Tests:** `VlaPlant.PolygonFrictionInsideAndOutside`, `PolygonFrictionBlendZone`;
  Python `test_roll_pitch_in_obs`, `test_polygon_friction_map_2d`
- **Docs:** `docs/VDSIM_PLANT_TUTORIAL.md` §6b (2D polygon), §4 roll/pitch

**Verify:** `cmake --build build -j && cd build && ctest -R VlaPlant --output-on-failure &&
python3 tests/scripts/test_vla_plant.py`

## 0. 2026-06-18 Dynamics ladder bench L0–L5 — DONE

**Shipped:** `tools/bench_dynamics_levels.py` + `examples/bench_dynamics_levels.cpp`
(`vdsim_bench_levels`) — ioniq5_awd, dt=5e-4, 500 step, SimSession.tick() ms/step.

| level | ms/step (AILAB-12, Release) |
|-------|----------------------------:|
| L0 kinematic | 0.0005 |
| L1 bicycle | 0.0162 |
| L2 seven_dof | 0.0313 |
| L3 fourteen_dof | 0.0317 |
| L4 fourteen_dof_kinematic | 0.0321 |
| L5 Free3D stunt | 0.0354 |

L0–L4 via pybind `make_sim_session`; L5 via C++ binary (not in pybind session factory).

**Run:** `cmake --build build -j && python3 tools/bench_dynamics_levels.py`

## 0. 2026-06-18 VLA plant per-wheel peak slip (alpha_peak, kappa_peak) — DONE

**Shipped:** per-wheel MF pure-slip peak positions (`alpha_peak` [rad], `kappa_peak` [-]) at
current Fz/load — raw GT for rising/peak/sliding (drift) classification vs current slip.

**Deliverables:**
- **`mf_peak_slip(B,C,E)`** in `interfaces.hpp` (Newton on MF peak equation)
- **`ITireModel::Output/Wrench::alpha_peak/kappa_peak`** + `IVehicleDynamics::wheel_alpha_peak/kappa_peak()` (Ld2)
- MF2002 / MF96 / linear `compute()` fill; `evaluate()` + Ld2 step propagate
- **`python/bindings.cpp`** + **`python/vdsim_plant.py`** `wheel[i]["alpha_peak"]`, `["kappa_peak"]`
- **Test** `VlaPlant.PeakSlipMatchesNumericalArgmax`

**Verify:** `cmake --build build -j && cd build && ctest -R VlaPlant --output-on-failure`

## 0a. 2026-06-18 VLA plant realized peak mu (useGT option A) — DONE

**Shipped:** per-wheel `mu_peak` (load-dependent MF realized peak coefficient, force/Fz) exposed
alongside existing contact `wheel.mu`. Friction-circle GT metric
`useGT = ||[Fx,Fy]|| / (mu_peak*Fz)` is now <= 1 by construction; contact `mu` preserved.

**Deliverables:**
- **`ITireModel::Output/Wrench::mu_peak`** + `IVehicleDynamics::wheel_mu_peak()` (Ld2)
- MF2002 / MF96 / linear `compute()` fill `mu_peak`; `evaluate()` propagates to wrench
- **`python/bindings.cpp`** + **`python/vdsim_plant.py`** `wheel[i]["mu_peak"]`
- **Test** `VlaPlant.RealizedPeakMuBoundsFrictionCircle`

**Verify:** `cmake --build build -j && cd build && ctest -R VlaPlant --output-on-failure`

## 0b. 2026-06-18 VLA plant tyre → MF2002 (T-IVb) — DONE

**Shipped:** Ioniq5-class VLA plant now uses **MF2002** (load-dependent `PKY*` cornering stiffness +
`PDY2` peak-μ load sensitivity) instead of MF96.

**Deliverables:**
- **`configs/parts/tire/ioniq5_pac2002.tir`** — public-synthetic Pacejka-2002 (gitignore-excepted)
- **`configs/parts/tire/ioniq5_pac2002.yaml`** — `backend: mf2002` + `tir_path`
- **`configs/vehicles/ioniq5_awd.yaml`** — `tire_yaml: parts/tire/ioniq5_pac2002.yaml`
- MF96 yaml **`ioniq5_pacejka.yaml`** kept (not deleted)
- **`python/vdsim_plant.py`** — resolves relative `tir_path` from configs root
- **`python/bindings.cpp`** — exposes `TireParams.backend` / `tir_path` to Python
- **Tests** (`test_vla_plant.cpp`): `DryHandlingMatchesLinearBicycleRealMf2002Tire`,
  `Mf2002LoadSensitivityConcaveStiffnessAndMu`, `Mf2002FrictionPatchHalvesPeakGrip`,
  `CombinedBrakeTurnGripLossOnPatchMf2002` (+ legacy MF96 inline tests unchanged)

**Verify:** `cmake --build build -j && cd build && ctest -R VlaPlant --output-on-failure`

**Spec:** `docs/design/VLA_THESIS_PLANT.md`

## 0c. 2026-06-18 VLA thesis plant (T-IV) — DONE

**Shipped:** `vdsim_plant.VDSimPlant` — first external client plant API for closed-loop MPC.

**Deliverables:**
- **#0** Ld2 native `CmdL1` torque path (Fx→per-wheel τ, bypass throttle map) + `plant_path` kinematic-blend bypass
- **#1** `create_friction_patch_ground(base_mu, [(x0,x1,mu)...])` + pybind
- **#2** `wheel_mu()` on Ld2 + pybind
- **#3** `configs/vehicles/ioniq5_awd.yaml` + `configs/parts/tire/ioniq5_pacejka.yaml`
- **#4** `python/vdsim_plant.py` on `vdsim_lab` / `make_vla_plant_session` (direct CmdL1, ZOH two-tier dt)
- **#5** smokes: `tests/scripts/test_vla_plant.py` + `examples/vla_plant_demo.py` + `python/VDSIM_PLANT_README.md`

**Verify:** `cmake --build build -j && cd build && ctest -R vla_plant --output-on-failure` (headline #2 brake+turn on μ=0.5 patch).

**Spec:** `docs/design/VLA_THESIS_PLANT.md`

## 0d. 2026-06-14 세션 인수인계 (archive)

**목표였던 것:** 멀티차량 비교(헤드리스+GUI) + 다나와식 차량조립 UI + 외부 AI 디자인 리뷰 반영. 완료.

**현재 상태:** `main` 에 feature 머지+push 완료(`b9064c4`), 336/336 green, 작업 트리 clean, 이번 세션 브랜치 삭제(로컬+원격). **미완 작업 없음 — 깨끗한 중단점.**

**이번 세션 main 에 들어간 것:**
- **헤드리스 멀티차량 비교** `tools/vdsim_compare.py` — preset N대에 같은 ISO maneuver(step-steer/skidpad/DLC) 돌려 지표 나란히 표 + bar-chart(compare.csv/png). `examples/maneuvers.py` 함수에 optional `veh=(vp,tp)` + `trace=True`(step_steer yaw-rate 시계열).
- **다나와식 build-sheet** (assembly 모달 재설계) — 좌측 슬롯 리스트(Body&chassis/Powertrain/Grip/Control 그룹, 각 행=부품+change) → 클릭 시 우측 그 슬롯 라이브러리 + RESOLVED STATS. `assembly.py` 카테고리 새 taxonomy 로 재그룹.
- **GUI Compare 대시보드** — VEHICLES 패널 ⊟ → blueprint 체크박스 → `/api/compare`(routes.py, `vdsim_compare` 래핑) → **step-steer yaw-rate overlay SVG 차트** + **Δ%(baseline 대비) 표**(유효숫자 통일, 정성/정량 분리). `tests/scripts/test_compare.py`(ctest `multi_vehicle_compare`).
- **GUI 폴리싱**(디자인 리뷰 반영) — RESOLVED STATS 단위, active 행 회색+좌측 accent, 3D 프리뷰 우측, Sync-draft 표준 크기.

**다음 할 일 후보 (우선순위 순):**
1. **GUI 재아키텍처 (전략 결정 후)** — 외부 AI 리뷰 4/10 "프로토타입" 판정. 큰 항목: dockable/resizable 패널 + 모달 제거(분할화면), 글로벌 모드 nav(Setup-Sim-Analyze), 전사 디자인 시스템. **stdlib http.server + 손코딩 app.js 의 한계 — React/Svelte + 디자인 패스(Figma) 결정이 선행.** 지금 찔끔 하지 말 것.
2. **#158 GUI 멀티차량 3D render** — preset picker 로 spawn 은 됨, 3D 뷰 N대 동시 표시(VDS1 vehicle_id demux) 남음.
3. cosmetic — part 라벨 "Fsk Formula chassis body" 잔재, Session tune 탭 라벨.

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
cd build && ctest --output-on-failure   # 328/328
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

## 5. 주의

- Hyundai (TUR) `.tir` **confidential** — 커밋 금지
- Default subsystem **숫자 리베이스 금지**
- Wheel: **FL=0, FR=1, RL=2, RR=3** · ISO 8855 RH
- Cosim plant **v5** ↔ rebuilt `vdsim_realtime`
- `configs/.resolve_cache/` — 커밋하지 않음

## 6. Git

- `main` @ `f6b1d71`, **all pushed**, 328/328 ctest. Last tag **`v0.5.1`**; main is ahead
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

1. **(DONE)** Catalog `drivetrain_v2` part — `drivetrain.sedan_v2` carries a `powertrain:`
   block through the resolver into the vehicle config; blueprint `vehicle.sedan_powertrain`.
   **(DONE)** L1 powertrain — the engine map + gearbox + shift policy now runs on the
   bicycle (opt-in, advanced once per substep, gear-dependent reflected inertia; flat path
   default-off keeps ISO green). Tests `DrivetrainV2.L1*`. Remaining drivetrain: engine-map
   workshop (GUI).
2. **Ld4 v0.6** — shared inertia helpers; full loop dynamics; Featherstone in step (optional).
3. **v0.5.2 GUI (browser, user-verified):** terrain Play (M4), stunt authoring (M5c),
   `.tir` import, tire-force arrows in wheel frame (`tire_forces_wheel()` ready).
4. Brake/steer physics (booster/MDPS/ABS); V2V collision + VDS1 multi-vehicle I/O.
5. **User module plugins — DECIDED = C++ `.so`; toolchain shipped, consumption pending.**
   Decision: C++ only (no Python authoring). Built: plugin ABI `vdsim/module_plugin.hpp`
   (`VDSIM_REGISTER_*_MODULE`), `dlopen` loader `load_module_plugin()`/`install_module()`,
   `templates/modules/*`, checker `vdsim_module_check`, `tools/module_workshop.py`
   (build/check/register → `module_plugin_v1` part under `gui_custom`). Tests `ModulePlugin.*`
   + `module_workshop`. Theory ch.24. **Consumption DONE:** a blueprint `module_plugins:`
   list resolves into the vehicle config and `install_module_plugins_from_yaml()` loads +
   installs after `initialize()` (wired into `vdsim_realtime`); `ModulePlugin.InstallFromYaml`
   proves register→blueprint→resolve→install→effect. **GUI (MW7) DONE** (server-verified;
   browser pass pending): setup "Modules" tab + `/api/module/{template,build_check,register}`
   wrapping `tools/module_workshop.py` (`gui/app.html` + `gui/static/app.js`). Module-plugin
   feature complete end-to-end.
6. **(FIXED `c6b2476`) open-diff engine inertia under symmetric accel.** The old
   `couple_open_axle_spin` carrier blend was a no-op when both wheels turned equally, so
   straight-line accel felt no reflected engine inertia. Replaced with the coupled 2x2 axle
   mass matrix (`open_axle_spin_accel`): symmetric accel now feels `I_e/2` per wheel,
   differential motion still doesn't. The tight gates stayed green (traction-locked spin), so
   no numeric re-baseline. Test `OpenDiffInertia.EngineInertiaSlowsSymmetricLaunch`.
8. **Part re-taxonomy (agreed; step 1 DONE).** Target: vehicle = body (mass/inertia) +
   chassis (suspension links/hardpoints/knuckle, = today's susp_kinematics) + powertrain
   (engine) + drivetrain (transmission+diff) + tire/brake/steering. **Step 1 shipped:**
   powertrain/drivetrain split — `powertrain` part type (`powertrain_v1`) + `drivetrain_v3`
   (gearbox+diff); resolver deep-merges the `powertrain:` block; `vehicle.sedan_split_powertrain`.
   **Remaining:** split `body` (mass/inertia) out of the `chassis` part; rename/reframe
   `susp_kinematics` → `chassis` (links + hardpoints + knuckle/knuckle-arm points); migrate
   existing blueprints. These are breaking renames — do with the user (slot names + migration).
9. **Repo hygiene (low priority, needs a call):** the two `sweep_runner.py`
   (`python/` catalog-binary sweep vs `apps/doe/` config DoE) are different tools with
   different YAML schemas, not a dead duplicate — decide which is canonical before merging.
   Demo `*.mp4` (~5.4M) are linked from `docs/tasks/T22/T23` reports; move to LFS/releases
   only with the report links updated. Cosmetic: rename `gui/runner/draft.py` (live code,
   misleading name); merge the two `builder/` servers.
