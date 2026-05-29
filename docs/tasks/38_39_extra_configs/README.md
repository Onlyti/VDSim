# Task 38-39 — Extra vehicle configs + Time-varying mu

| Field | Value |
|---|---|
| Task ID | IM-W11-4 |
| Type | Tooling + Config |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

- **차종 다양성**: sedan / sports 외에 FSK Formula (작은 race car) + 일반 race car (LMP/GT3 class).
- **노면 시간 변화**: scenario YAML 의 `mu_profile` 추가 — icy patch / wet transition 시뮬 가능.

## 2. 구현

### 2.1 새 파일

| 위치 | 역할 |
|---|---|
| `configs/vehicles/fsk_formula.yaml` | 280 kg 작은 formula, RWD spool, 큰 aero |
| `configs/vehicles/race_car.yaml` | 900 kg AWD LSD, 매우 큰 downforce |
| `configs/scenarios/ice_patch.yaml` | mu 1.0 → 0.2 → 1.0 transition |
| `core/include/vdsim/scenario.hpp` | `MuSample`, `mu_profile`, `sample_mu()` 추가 |
| `core/src/scenario.cpp` | linear mu 보간 |
| `examples/scenario_run.cpp` | `sc.sample_mu(t)` 매 tick 호출, contacts 갱신 |

### 2.2 차종 핵심 파라미터 비교

| 항목 | sedan | sports | FSK formula | race car |
|---|---:|---:|---:|---:|
| mass [kg] | 1500 | 1320 | 280 | 900 |
| wheelbase [m] | 2.70 | 2.55 | 1.55 | 2.65 |
| h_cg [m] | 0.55 | 0.42 | 0.30 | 0.33 |
| max_motor [Nm] | 300 | 480 | 150 | 700 |
| brake_bias_front | 0.62 | 0.65 | 0.58 | 0.62 |
| ackerman_pct | 60 | 85 | 100 | 90 |
| differential | Open | LSD | Locked (spool) | LSD |
| Cd | 0.30 | 0.34 | 0.85 | 0.50 |
| Cl_front + Cl_rear | 0.10 | 1.00 | **3.00** | **4.30** |
| anti_dive_front | 0 | 0 | 0.30 | 0.45 |

## 3. 검증

### 3.1 차종 별 step_steer 결과 (vx=10, δ=0.05, 5s)

| Vehicle | r_end [rad/s] | vx_end [m/s] |
|---|---:|---:|
| sedan | 0.1759 | 9.728 |
| sports | 0.1848 | 9.692 |
| FSK formula | **0.2621** | 8.634 |
| race car | — (별도 실행) | — |

FSK 가 가장 quick (wheelbase 짧음 + 100% Ackerman). vx 빠르게 감소 (aero drag Cd=0.85 + cornering 손실).

### 3.2 ice_patch 시나리오 (sedan, 6 s straight)

mu 1.0 → 0.2 → 1.0. 직진이므로 mu 변화 자체로는 거동 차이 적음 (drag-only decay). 입력 추가 시 (코너링 / 가속) mu 변화의 영향이 정량 측정 가능.

### 3.3 회귀

127/127 통과 (변경 없음 — 기존 test 영향 없음). 모든 차종 + scenario 정상 실행.

## 4. 판단

- 결과: **pass**
- 근거: 4 vehicle configs (sedan / sports / FSK / race), 4 scenarios (step_steer / DLC / throttle_brake / ice_patch), 모든 조합 정상 실행.
- Follow-up:
  - **실측 데이터 fit** — TUR / FSK 의 telemetry 로 sedan/sports/FSK config 보정.
  - **mu_profile 의 cornering 검증** — 시각화 figure 별도.
  - **추가 시나리오**: highway lane change, J-turn, brake-in-turn.
