# Task 27 — Aerodynamic downforce / lift coefficient

| Field | Value |
|---|---|
| Task ID | IM-W5-13 |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

`VehicleParams` 에 axle-level lift / downforce 계수 (`aero_lift_front`, `aero_lift_rear`) 추가. 고속에서 Fz 증가 → tire grip 한계 향상.

이게 빠지면:
- F1 / GT3 클래스 차량의 고속 cornering grip 모델 불가.
- sedan vs sports 차별화에서 downforce 효과 없음.
- 200 km/h 이상 의 stability behavior 비현실적.

## 2. 구현 방법

### 2.1 식

Per axle (L1) / per tire (L2):
```
q = 0.5 · rho_air · frontal_area · vx · |vx|        (dynamic pressure × A)
Fz_aero_front = aero_lift_front · q                  (positive = downforce)
Fz_aero_rear  = aero_lift_rear  · q
```

L2 의 per-tire 분배: `Fz_aero_per_tire = 0.5 · axle_value`.

`vx · |vx|` 로 후진 시 부호 보존 (대칭).

### 2.2 코드 변경

| 위치 | 변경 |
|---|---|
| `core/include/vdsim/params.hpp` | VehicleParams 에 `aero_lift_front`, `aero_lift_rear` (default 0) |
| `core/src/params.cpp` | YAML pull / emit |
| `core/src/bicycle_dynamics.cpp` | axle Fz 에 aero downforce 가산 |
| `core/src/seven_dof_dynamics.cpp` | per-tire Fz_static 에 가산 |
| `tests/unit/test_params_yaml.cpp` | AeroLiftRoundtrip |
| `tests/integration/test_seven_dof.cpp` | 2 새 test (40 m/s downforce, 0 m/s rest 검증) |

### 2.3 결정

| 결정 | 채택 | 근거 |
|---|---|---|
| Default 0 | yes | backward-compat (기존 vehicle YAML 영향 없음) |
| Per-axle 분리 | yes | front wing / rear wing 독립 tunable |
| sign convention: 양수 = downforce | yes | "lift" 라는 이름은 거꾸로지만 motorsport convention |
| Drag 와 같은 q 사용 | yes | 단일 frontal_area 가정 (separate area 도입은 별도) |
| L1, L2 모두 적용 | yes | 일관 grip behavior |

### 2.4 한계

- **Pitch dependence 무시** — anti-dive 후 nose 압력 분포 변화는 미반영.
- **Yaw dependence 무시** — side wind / yaw 시 aero balance 변화.
- **Roll dependence** — body 기울임 시 wing AOA 변화.
- **Frontal area 와 lift area 의 분리** — 일부 차는 분리해야 정확.

## 3. 검증 방법 + 결과

### 3.1 3 새 test

| Test | 항목 | Pass 기준 |
|---|---|---|
| VehicleYaml.AeroLiftRoundtrip | `Cl_f=1.2, Cl_r=1.85` save/load | 정확 회복 |
| SevenDOF.AeroDownforceIncreasesFzAtSpeed | vx=40, Cl=(2.0, 2.5) | sum(Fz) > m·g |
| SevenDOF.AeroDownforceZeroAtRest | vx=0, 동일 Cl | sum(Fz) ≈ m·g |

전체 112/112 통과 (이전 109 + 본 task 3).

### 3.2 정량

`Cl_f = 2.0, Cl_r = 2.5, frontal_area = 2.2, vx = 40 m/s`:
- q = 0.5 · 1.225 · 2.2 · 40 · 40 = 2156 N (dyn pressure × A)
- Fz_aero_total = (2.0 + 2.5) · 2156 = **9702 N** 추가 downforce
- m·g (sedan) = 14710 N → sum Fz ≈ 24412 N (66% 증가)

## 4. 판단

- 결과: **pass**
- 근거: 3/3 새 test 통과, 누적 112/112.
- Follow-up:
  - **Pitch / yaw / roll dependent aero** — Phase 2.
  - **Separate lift_area vs frontal_area** — 필요 시.
  - **sports.yaml 의 Cl_f, Cl_r 측정값 채우기** — 별도.
