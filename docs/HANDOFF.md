# VDSim 핸드오프

작성: **2026-06-05** (v0.4 slope/jump M0–M5 on `feat/v0.4-slope-jump-m5`).

## 1. 문서 인덱스

| Topic | User guide | Design spec |
|-------|------------|-------------|
| Catalog + scenes + CLI | [`CATALOG_AND_PHYSICS.md`](CATALOG_AND_PHYSICS.md) | [`design/PARTS_CATALOG.md`](design/PARTS_CATALOG.md) |
| GUI catalog API | § GUI in above | `PARTS_CATALOG.md` §8 |
| External packs | § External packs | `PARTS_CATALOG.md` §7 |
| Drivetrain inertia | § Drivetrain | [`design/V0.2_DRIVETRAIN.md`](design/V0.2_DRIVETRAIN.md) |
| LuGre tire | § LuGre · **theory ch.19** | [`theory/19_lugre_dynamic_tire.md`](theory/19_lugre_dynamic_tire.md), [`design/V0.2_TIRE_LUGRE.md`](design/V0.2_TIRE_LUGRE.md) |
| Product roadmap | [`ROADMAP.md`](ROADMAP.md) | [`design/TIRE_ROADMAP.md`](design/TIRE_ROADMAP.md) |
| Low-speed (default tire) | — | [`design/LOW_SPEED_HANDLING.md`](design/LOW_SPEED_HANDLING.md) |
| Run modes / cosim | [`RUNNING.md`](RUNNING.md) | [`design/RUNTIME_ARCH.md`](design/RUNTIME_ARCH.md) |
| Validation | [`VALIDATION.md`](VALIDATION.md) | — |

## 2. 오늘 작업 (2026-06-05)

### 2.1 L2 Ackermann 우회전 버그 수정
- **증상:** 우회전만 전륜 Fy 리플·슬립각 비대칭 (좌회전은 양호).
- **원인:** `d < 0`일 때 inner/outer `atan` 크기 관계 뒤집힘 + FL/FR 매핑 오류 → 바깥 FL 과조향.
- **수정:** `core/src/seven_dof_dynamics.cpp` — `|tan(d)|`로 inner/outer 계산 후 부호 복원, 좌회전 `d>0` / 우회전 `d<0` 각각 FL/FR 매핑.
- **테스트:** `SevenDOF.AckermanInnerWheelLargerSlipBothTurnDirections` (`tests/integration/test_seven_dof.cpp`).

### 2.2 GUI — Vehicle 편집 UX
- Run composition **Vehicle** 탭: 스폰(x,y,ψ,vx) + **Edit**만 유지.
- **Edit 모달** 탭 분리:
  - **Chassis** — Level, Dynamics(질량·관성·공력 등)
  - **Tire** — tire catalog / override
  - **Suspension** — front·rear kinematics, 4휠 배열 **FL FR RL RR** 라벨
- `appendChassisDynamicsTools`, `appendSuspensionFleetTools`, `modalFleetUpdate` 등 (`gui/app.html`).
- 스크립트: `tests/scripts/test_gui_fleet_level.py`, `test_gui_tire_override.py` (수동/CI).

### 2.3 GUI 텔레메트리 — cmd → applied
- 실행 중 우측 패널 **Controls (cmd → applied)**:
  - throttle / brake: `45 % → 42 %`
  - steer: `0.050 rad (2.9°) → 0.048 rad (2.7°)`
- **cmd:** `fleet_cmd` (멀티 플릿 per-vehicle) / `cmd_in` (live).
- **applied:** cosim STATE (`steer`, `throttle_applied`, `brake_applied`).

### 2.4 Cosim VDS1 **v5** (throttle/brake applied)
- `cosim/cosim_protocol.hpp`, `cosim/protocol.py`: STATE **452 B**, version **5**; tail에 `throttle_applied`, `brake_applied`.
- `SimOutput` + `sim_session.cpp`: actuator 이후 realized pedal 저장.
- `cosim/realtime_server.cpp`, `gui/runner/cosim_bridge.py`, `gui/server.py` (`protocol_version: 5`) 연동.
- **주의:** GUI Play 시 **재빌드된 `vdsim_realtime`** 필요 (v4 plant면 applied pedal 0으로 보임).

### 2.5 선회 Fy 리플 — 원인 분석 (코드 미수정)
| 항목 | 결론 |
|------|------|
| 우회전만 심함 | Ackermann 버그 → **§2.1로 해결** |
| 남는 리플 | LuGre 아님 (`kinematic_fallback` 동일); **L2 quasi-static Fz**가 `ay_prev_` 1-step lag |
| dt 축소 | 10→1 ms **효과 없음** (동일 std/pkpk) |
| 2~3 Hz 관찰 | FFT: 지배 **0.5~1 Hz** + 배음 1.5~2.5 Hz; δ≈3°·α≈0 분기에서 큼 |
| 조치 | **없음** (시각적 리플 수준; LPF/Fz 경로 수정 불필요) |

### 2.6 Validation (LuGre baseline, 이전 세션 포함·uncommitted)
- `apps/validation/run_validation.py` — sedan L2 + `default_pacejka` 재실행.
- 결과·수치: `docs/VALIDATION.md` **LuGre baseline** 절, `apps/validation/results/lugre_sedan_l2/`.

## 3. 구현 상태 (누적)

### v0.3 catalog (M1–M5) — shipped
- `configs/catalog/`, `parts/`, `blueprints/`, `scenes/`, `maneuvers/`
- CLI: `vdsim_realtime --scene=` only; GUI `/api/catalog`, simconfig v3
- Legacy `configs/vehicles|tires|scenarios/` removed

### Physics (post–v0.3)
- Drivetrain: `engine_rotational_inertia` + open-diff carrier
- Tire: LuGre default (`default_pacejka`); `kinematic_fallback` / `--no-lugre`
- L2 Ackermann: 좌·우 대칭 inner-wheel larger \|α\| (§2.1)

### GUI / runtime
- v0.2 fleet, data-comms, simconfig round-trip
- Vehicle Edit 모달 Chassis/Tire/Suspension (§2.2)
- 텔레메트리 cmd/applied (§2.3); VDS1 v5 (§2.4)

### Tests
- **`210/210` ctest green** (`cmake --build build -j` → `cd build && ctest --output-on-failure`)
- 신규/관련: `SevenDOF.AckermanInnerWheelLargerSlipBothTurnDirections`, `LuGreTire/*`, `cosim_multi_vehicle`

## 4. v0.4 slope/jump (브랜치 `feat/v0.4-slope-jump-m5`)

| M | 상태 | 내용 |
|---|------|------|
| M0 | done | L2 `is_valid` → Fz/Fx/Fy off per wheel |
| M1 | done | `Stunt.GradeL2CoastSlowsOnUphill` |
| M2–M3 | done | `SolverParams::stunt_physics` world-z, −g, tire compression airborne |
| M4 | partial | `RampGround`, scenes `jump_ramp_demo.yaml` / `vertical_loop_demo.yaml` |
| M5 | done | `L5_Stunt`, `LoopGround`, loop rail guide + `Stunt.VerticalLoopCompletesLap` |

- 코어: `create_ramp_ground`, `create_loop_ground`, `create_stunt_dof()`
- 루프: educative rail (`loop_radius` + θ integration) — full free Ld5는 v0.4 후속
- **214/214** ctest (`Stunt/*` +4)

## 5. 다음 작업

1. **ISO re-baseline** — drivetrain/LuGre (`run_validation.py`).
2. Scene loader에 `stunt.ground` → `make_ground` 연동 (M4 GUI).
3. 타이어 T1–T2 · free Ld5 loop (물리 루프, rail off).
4. `main` ← PR merge after review.

## 6. 주의

- Hyundai (TUR) `.tir` **confidential** — 커밋 금지.
- Default subsystem **숫자 리베이스 금지** (ISO/benchmark 이동 시 중단·보고).
- GUI 시각 검증 = 유저. Cosim **v5** ↔ `vdsim_realtime` 버전 일치.
- Wheel index: **FL=0, FR=1, RL=2, RR=3**; ISO 8855 RH.

## 7. 빌드·검증

```bash
cmake --build build -j
cd build && ctest --output-on-failure   # 214/214
# GUI
python3 gui/server.py   # http://127.0.0.1:8080
```

## 8. Git

브랜치 `feat/v0.4-slope-jump-m5` — **main 미병합**. push/merge는 명시 요청 시.
