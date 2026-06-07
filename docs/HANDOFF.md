# VDSim 핸드오프

작성: 2026-06-06. **v0.2.4** 태그 = v0.2 마감. **v0.3** 진행 중 (M1 catalog resolver).

## 1. 목표
v0.3 "parts catalog + drivetrain + LuGre": 차량 = blueprint(parts), 씬 기반 실행.
`docs/design/PARTS_CATALOG.md`, `V0.2_DRIVETRAIN.md`, `V0.2_TIRE_LUGRE.md`.

## 2. 현재 상태
- **v0.2.4** = v0.2 라인 마감 (GUI split, data-comms, run config, fleet viz).
- 빌드 `cmake --build build -j`; **ctest 193/193** (v0.3 M1 `catalog_resolver` 추가 후).
- v0.1.0 태그 `4c77d7f`. GitHub Topics / 데모 GIF (#155) = 사용자 UI.

## 3. v0.2 완료 요약
- **WS1–WS4** 서브시스템 골격, 워크샵, 3-tab setup, fleet parts, 멀티차량 (#157/#158).
- **v0.2.1–v0.2.3** L3 susp UX, 경고 패널, kinematics 검증.
- **v0.2.4** runner/API split, autopilot·manual fix, `wheel_spin`+`time_scale` 바퀴,
  data-comms (VDS1 + UDP fan-out), run config save/load, `_fleet_add` empty-fleet fix.

## 4. v0.3 진행 (M1 시작)
| Phase | 상태 | 내용 |
|-------|------|------|
| **M1** | **진행** | `configs/catalog/manifest.yaml`, `configs/parts/**`, `configs/blueprints/**`,
  `python/catalog/resolver.py`, envelope validator, ctest `catalog_resolver` |
| M2 | 대기 | built-in migrate + delete v0.2 config trees |
| M3 | 대기 | `vdsim_realtime --scene=` only |
| M4 | 대기 | GUI `/api/catalog`, scene simconfig v3 |
| M5 | 대기 | docs, `tools/import_part_pack` stub |

병행 (catalog 위에 part type으로 추가):
- Drivetrain inertia + torque–RPM (`V0.2_DRIVETRAIN.md`)
- LuGre tire schema variant (`V0.2_TIRE_LUGRE.md`)
- Engine workshop 실구현, infra sensor runtime mount

## 5. 주의
- GUI 시각검증 = 유저. 에이전트 = markup/API/ctest.
- **193 green** on default modules (M1 후).
- TUR tire confidential. Simon Q/R. VDS1 v4.
- ISO/benchmark 리베이스 금지 until drivetrain lands (then re-validate per `VALIDATION.md`).

## 6. 경로
- Catalog: `configs/catalog/`, `configs/parts/`, `configs/blueprints/`
- Resolver: `python/catalog/resolver.py`
- GUI: `gui/app.html`, `gui/server.py`, `gui/runner/`, `gui/api/`
- Legacy (M2 삭제 예정): `configs/vehicles/`, `configs/tires/`, `configs/scenarios/`
- 설계: `docs/design/PARTS_CATALOG.md`, `V0.2_*.md`, `V0.4_PLAN.md`
