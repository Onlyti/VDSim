# VDSim 핸드오프

작성: 2026-06-06. **v0.2.3** `f3f9815` push 완료. v0.3 = drivetrain + LuGre.

## 1. 목표
v0.2 "composable vehicle": 차량 = chassis + parts, 워크샵 + 씬 UI. **`gui/app.html`**
단일 GUI. `docs/design/V0.2_PLAN.md`.

## 2. 현재 상태
- **v0.2.0** 태그 = 로컬 HEAD (push 후 origin 갱신).
- 빌드 `cmake --build build -j`; **ctest 192/192** (v0.2 GUI 잔여 후).
- v0.1.0 태그 `4c77d7f`. GitHub Topics / 데모 GIF (#155) = 사용자 UI.

## 3. v0.2.0 완료
- **WS1** 서브시스템 골격 + deadtime (#159-162).
- **WS2** edit 모달 workshops (tire .tir, susp presets, actuator curves); Engine stub → v0.3.
- **WS3** 3-tab setup, data-comms, minimap, simconfig v2, fleet tree, legacy GUI 제거.
- **WS4** fleet parts (`vehicle`/`tire`/`front_susp`/`rear_susp`), `/api/parts/registry`,
  L3 `attach_*_kinematics` in `vdsim_realtime`, infra sensor list (authoring).
- **멀티차량** #157/#158, VDS1 vehicle_id, shared-world scenarios.

## 4. v0.2.1 완료 (`1c979c6`)
- fsk_formula L3 susp default, scenario susp 경로, L3-only fleet UI, attach warnings.
- `cosim_multi_vehicle` ctest; topology-only YAML attach 거부 테스트.

## 5. v0.2.2 완료 (`ee53f31`)
- 모달 kinematics 드롭다운 L3-native only; `Popen(cwd=REPO)`.
- L3 missing/topology-only stem → pre-launch `kinematics_warnings`.

## 6. v0.2.3 완료 (`f3f9815`)
- 경고 패널, setup API L3 검증, L3 전환 default, log tail, rel world susp paths.
- `l3_sedan_kinematics.yaml` + `l3_scenario_susp_paths` ctest.

## 6c. v0.2 GUI 잔여 (로컬, 미커밋)
- **Data & comms (Options):** VDS1 CMD/STATE 포트 표시, plant 상태, HTTP I/O + UDP fan-out
  (차량별 telemetry; fleet `snap.fleet` 기반 non-live state 전송).
- **Run config:** `cosim_state_port` + `telemetry` gui 메타 export/import; `GET /api/comms`.
- **SIM 패널:** Load cfg / Save cfg (run config YAML).
- **Scenario load:** dirty draft 확인, status 갱신.
- **ctest:** `runconfig_roundtrip` (192/192 green).

## 6b. GUI (로컬, 미커밋)
- **역할 (RUNTIME_ARCH [1]):** 시나리오 편집 + ▶/⏹ 실행 제어 + 3D/텔레메트리. plant step 없음.
- **Setup = draft** · **▶ Play** → `runs/live/run_config.yaml` materialize + `vdsim_realtime --scenario=`.
- **Composition UX (A):** status strip (draft synced/changed · loaded scenario · last run path);
  **Sync draft** button; scenario Save modal (name validation + overwrite); Vehicle tab
  Vehicle tab: spawn-only (all fleet sizes); inline **Edit** on vehicle row.
- **REST** = draft/설정 (`/api/setup`, `/api/runconfig`, `/api/control`, …).
- **SSE** `/api/stream` = 서버→브라우저 state bus (~60 Hz). Setup=`source:setup`(spawn 미러), Running=`source:cosim`(plant relay).
- 단일 **run config** (YAML): scenario `vehicles[]` + `gui:`(GUI 전용). 구 simconfig v2 JSON import 호환.
- `GET /api/runconfig/draft.yaml` · `POST /api/runconfig` · bottom **Run composition**
  bar: Load/Save scenario (`configs/scenarios/`), Export/Import YAML.
- Setup tabs: **Map · Vehicle · Options** (fixed height); co-sim Start/Stop removed
  (use ▶/⏹; Options = attach external + comms only).
- **`gui/core.js`**: Three.js 실패 시 Play/Stop/conn fallback.
- SSE API 폭주·spawn 드래그 깜빡임 수정 (`gui/app.html`).
- **PR1 runner split:** `gui/runner/autopilot.py` (WaypointPath, FigureEight,
  `compute_vehicle_cmd`), `gui/runner/cosim_bridge.py` (CosimBridge, plant launch),
  `gui/runner/config.py` (REPO, `gui_run_dir`). `server.py` import-only for these.
- **PR2 HTTP split:** `gui/api/routes.py` (GET/POST route table), `gui/api/handler.py`
  (`make_handler`), `GET /api/debug` (time_scale, fleet_driver, cosim state keys).
- **PR3 draft split:** `gui/runner/draft.py` (DraftMixin: materialize, import/export,
  apply_setup), `runner/suspension.py`, `runner/params_schema.py`, `runner/params_io.py`.
- **버그픽스 (로컬):** `/api/manual` 40ms 폴링이 `set_manual()`에서 `p.driver=False`
  강제 → autopilot 즉시 해제. 서버: manual은 `p.driver`일 때만 in_cmd 갱신; 브라우저:
  Manual UI on일 때만 POST.
- **멀티 fleet viz (로컬):** `wheel_spin` + `time_scale` 바퀴 적분; fleet snapshot 갱신.

## 7. 다음 (v0.3+)
- **Parts catalog cutover** — `docs/design/PARTS_CATALOG.md` (big-bang, no legacy).
  `parts/` + `blueprints/` + `scenes/`; delete v0.2 stem paths.
- Drivetrain inertia + torque–RPM (`V0.2_DRIVETRAIN.md`) as `drivetrain` part type.
- LuGre tire (`V0.2_TIRE_LUGRE.md`) as `tire` schema variant.
- Engine workshop (실구현).
- Infra sensor runtime mount (`SIM_CONFIG_ARCH.md` host field).
- v0.4 스턴트 (`V0.4_PLAN.md` M1–M6).

## 8. 주의
- GUI 시각검증 = 유저. 에이전트 = markup/API/ctest.
- **191 green** on default modules.
- TUR tire confidential. Simon Q/R. VDS1 v4.

## 9. 경로
- GUI: `gui/app.html`, `gui/server.py`, `gui/core.js`, `gui/runner/`, `gui/api/`
- 아키: `docs/design/RUNTIME_ARCH.md`, `docs/gui_architecture.md`
- Parts: `configs/components/`, `configs/suspensions/`
- 설계: `docs/design/V0.2_*.md`, `V0.4_PLAN.md`
