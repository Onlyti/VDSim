# Task 53-56 — Driver model + .tir importer + scenarios + RR integration

| Field | Value |
|---|---|
| Task ID | IM-W11-8 (cluster) |
| Type | Impl + Tooling |
| Date | 2026-05-29 |
| Status | completed |

## 1. 진척

| Task | 산출물 |
|---|---|
| 53 — Driver model | `DriverModel` (latency + Gaussian noise on steer/throttle), 2 unit test |
| 54 — AVL .tir importer | `python/tir_to_yaml.py`, 12-field subset, end-to-end run 검증 |
| 55 — Scenarios | `j_turn.yaml`, `skidpad.yaml`, `brake_in_turn.yaml` |
| 56 — Rolling resistance | bicycle / L2 의 Fx 에 `f_rr · Fz_total · tanh(vx/0.5)` 추가, default 0 으로 기존 회귀 보호 |

## 2. 구현 디테일

### Driver model
- Reaction time delay: ring buffer (size = `round(reaction/dt)`), default 150 ms.
- Steer noise: σ ≈ 0.005 rad (Box-Muller 변환).
- Throttle/brake noise: σ ≈ 0.02.
- 외부에서 두 uniform(0,1) 입력 (deterministic / reproducible).
- 내부 Pure Pursuit + L6 vx PID + L5 ax PID 통합.

### AVL .tir importer
- FIELD_MAP: BBX1/CFX1/DFX1/EFX1 (long), BBY1/CFY1/... (lat), LMUX (mu), FNOMIN, VERTICAL_STIFFNESS, QSY1 (rolling), QSGZ1 (camber).
- 12 fields parsed, mapped to TireParams.
- 미매핑 키는 default 유지 → backward-compat.

### Scenarios
- `skidpad.yaml` (δ=0.27, R=10 m): r=0.561 → R=vx/r=9.95 m ≈ 10 m 정확 일치.
- `j_turn.yaml`: 1.5s straight → 2.5s brake+steer combination.
- `brake_in_turn.yaml`: 2s straight cornering → 3s brake under cornering.

### Rolling resistance
- Bicycle: `F_rr = f_rr · (Fz_f + Fz_r) · tanh(vx/0.5)`.
- L2 7-DOF: `F_rr = f_rr · Σ Fz_i · tanh(vx/0.5)`.
- Default `f_rr=0` for backward compat. 기존 analytical bicycle test (drag-only) 영향 없음.

## 3. 검증

### 3.1 새 tests

| Test | 항목 | Pass |
|---|---|---|
| DriverModel.ReactionTimeDelaysSteer | 100 ms 대기 후 steer | initial 0 + 정상 출력 |
| DriverModel.NoiseIsBoundedByRMS | 200 ticks, RMS=0.005 | max < 0.04 (8σ) |
| LongScenarios.RollingResistanceReducesCoastDistance | RR 0 vs 0.015 | with_RR vx < no_RR vx |

전체 139/139 통과.

### 3.2 End-to-end

- `.tir` import + sedan + step_steer: vx 10→9.745, r=0.1768 (default tire 와 거의 동일 → import 식 정상)
- 3 새 scenarios 모두 실행 + CSV 정상.

## 4. 판단

- 결과: **pass**
- 근거:
  - 3 새 test 통과, 누적 139/139.
  - Driver model 의 latency / noise 가 reproducible.
  - .tir importer 가 12 표준 필드 매핑.
  - skidpad 의 turning radius 가 이론값과 정확 일치.
- 미해결 / Follow-up:
  - **Driver model 의 closed-loop scenario** — DriverModel demo binary.
  - **MF2002 .tir 전체 필드** — Phase 2.
  - **차종별 rolling_resistance 실측 값** — TUR/FSK telemetry.
  - **J-turn 의 reverse vx** — brake 이후 vx 부호 처리 검증 필요 (별도 follow-up).
