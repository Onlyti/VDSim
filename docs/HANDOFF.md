# VDSim 핸드오프

작성: 2026-06-06. **v0.2.0** 로컬 완료 (push/tag 후 origin 동기화). v0.3 = drivetrain + LuGre.

## 1. 목표
v0.2 "composable vehicle": 차량 = chassis + parts, 워크샵 + 씬 UI. **`gui/app.html`**
단일 GUI. `docs/design/V0.2_PLAN.md`.

## 2. 현재 상태
- **v0.2.0** 태그 = 로컬 HEAD (push 후 origin 갱신).
- 빌드 `cmake --build build -j`; **ctest 188/188**.
- v0.1.0 태그 `4c77d7f`. GitHub Topics / 데모 GIF (#155) = 사용자 UI.

## 3. v0.2.0 완료
- **WS1** 서브시스템 골격 + deadtime (#159-162).
- **WS2** edit 모달 workshops (tire .tir, susp presets, actuator curves); Engine stub → v0.3.
- **WS3** 3-tab setup, data-comms, minimap, simconfig v2, fleet tree, legacy GUI 제거.
- **WS4** fleet parts (`vehicle`/`tire`/`front_susp`/`rear_susp`), `/api/parts/registry`,
  L3 `attach_*_kinematics` in `vdsim_realtime`, infra sensor list (authoring).
- **멀티차량** #157/#158, VDS1 vehicle_id, shared-world scenarios.

## 4. 다음 (v0.3+)
- Drivetrain inertia + torque–RPM (`V0.2_DRIVETRAIN.md`).
- LuGre tire (`V0.2_TIRE_LUGRE.md`).
- Engine workshop (실구현).
- Infra sensor runtime mount (`SIM_CONFIG_ARCH.md` host field).
- v0.4 스턴트 (`V0.4_PLAN.md` M1–M6).

## 5. 주의
- GUI 시각검증 = 유저. 에이전트 = markup/API/ctest.
- **188 green** on default modules.
- TUR tire confidential. Simon Q/R. VDS1 v4.

## 6. 경로
- GUI: `gui/app.html`, `gui/server.py`
- Parts: `configs/components/`, `configs/suspensions/`
- 설계: `docs/design/V0.2_*.md`, `V0.4_PLAN.md`
