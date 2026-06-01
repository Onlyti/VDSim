# Task 29 — Brake bias (front/rear distribution)

| Field | Value |
|---|---|
| Task ID | IM-W5-15 |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

기존 fixed 50:50 front/rear brake torque 분배 한계 해소. `brake_bias_front` 추가:
- Sedan: 보통 ~60-65% (front-heavy, 정적 무게 고려).
- Race / sports: 일부 60:40, 일부 70:30.
- FSK car: 일부 50:50 (electronic balance), 일부 65:35.

이게 빠지면:
- 차종별 brake balance 차이 시뮬 불가.
- 사용자 가 brake 영역의 차량 거동 (over/under-braking, ABS 동작) 평가 불가.

## 2. 구현 방법

### 2.1 코드 변경

| 위치 | 변경 |
|---|---|
| `core/include/vdsim/params.hpp` | `brake_bias_front` (double, default 0.5) |
| `core/src/params.cpp` | YAML pull / emit |
| `core/src/bicycle_dynamics.cpp` | `Tb_f_mag = bias · cmd.brake · max_T`, `Tb_r_mag = (1-bias)·...` |
| `core/src/seven_dof_dynamics.cpp` | per-axle 분배 후 axle 별 좌우 50:50 |
| `tests/unit/test_params_yaml.cpp` | BrakeBiasRoundtrip |
| `tests/integration/test_weight_transfer.cpp` | BrakeBiasInfluencesFrontFx (front-heavy 가 vx_end 더 작음) |

### 2.2 결정 / 한계

| 결정 | 채택 | 근거 |
|---|---|---|
| Default 0.5 | yes | 기존 동작 그대로, backward-compat |
| Clamp [0, 1] | yes | invalid 입력 방어 |
| Axle 내 좌우 50:50 | yes | ABS / EBD 는 별도 task |
| Dynamic balance | 미구현 | 부하 따라 자동 조정 (EBD) — Phase 2 |

## 3. 검증

### 3.1 2 새 test

| Test | 조건 | Pass 기준 |
|---|---|---|
| BrakeBiasRoundtrip | 0.65 save/load | 정확 회복 |
| BrakeBiasInfluencesFrontFx | bias 0.8 vs 0.2, brake=0.5, 0.5 s | front-heavy 의 vx_end < rear-heavy |

전체 114/114 통과 (이전 112 + 2 new).

## 4. 판단

- 결과: **pass**
- 근거: 2/2 새 test 통과, 누적 114/114. backward-compat 유지.
- Follow-up:
  - **차종별 default** — sedan.yaml ≈ 0.62, sports.yaml ≈ 0.65.
  - **EBD (Dynamic balance)** — load 따른 동적 조정.
  - **ABS** — 별도 task.
  - **Left-right brake bias** (left turn 시 outer-loaded) — 매우 specialized.
