# Task 06 — D17 검증 baseline 시나리오 명세

| Field | Value |
|---|---|
| Task ID | D17 |
| Type | Design |
| Date | 2026-05-28 |
| Commit | `1e15c5e` |
| Status | completed (명세) / partial (시나리오 1 구현됨, 3 개 예정) |

## 1. 목적

vehicle dynamics 의 "맞다"를 무엇으로 증명할지 사전에 정의. 사후에 결과를 보고 통과 기준을 정하면 cherry-picking 위험. 사용자 결정: 해석해 + CarMaker ERG (ERGAccess C API), 4 시나리오.

## 2. 구현 방법

### 시나리오 4종

| # | 이름 | Input | 해석해 / 기준 |
|---|---|---|---|
| 1 | Steady-state cornering | δ=const, vx=const | r = vx·δ/L · 1/(1+K_us·vx²) (linear bicycle SS) |
| 2 | Step steer response | δ=step, vx=const | r(s)/δ(s) = K·(1+τ_z·s) / (s²+2ζω_n·s+ω_n²) |
| 3 | Constant throttle accel | δ=0, throttle=const | m·v̇ = F_x_drive − F_aero − F_rr |
| 4 | Constant brake | δ=0, brake=const | m·v̇ = −F_brake − F_rr |

### 통과 기준 (per level)

| 사다리 | 해석해 | CarMaker ERG |
|---|---|---|
| L1 Bicycle | linear region 4 시나리오 | 1, 3, 4 (linear bicycle 한계로 deviation) |
| L2 7-DOF | 4 시나리오 모두 | 4 시나리오 모두 |
| L3 14-DOF | 4 + roll/pitch 정량 | 4 + roll/pitch |

### CarMaker ERG 비교 메트릭

| 채널 | 메트릭 | 통과 기준 (PoC) |
|---|---|---|
| Vehicle.x, .y | trajectory RMSE | < 0.5 m at 100 m drive |
| Vehicle.Yaw | yaw RMSE | < 2° |
| Vehicle.YawRate | rate RMSE | < 10 % |
| Vehicle.ax, .ay | accel RMSE | < 0.5 m/s² |
| Tire.Fz[FL..RR] | 정상 분배 RMSE | < 10 % |

### 디렉토리 layout

```
tests/
├── unit/                          # task 11 까지 사용
├── integration/                   # task 11 첫 사용
│   └── test_bicycle_steady_state.cpp
└── validation/                    # task 13 부터
    ├── analytical/
    │   ├── test_step_steer.cpp
    │   ├── test_acceleration.cpp
    │   └── test_brake.cpp
    └── carmaker/                  # Phase 2 (ERGAccess license dependent)
        ├── data/                  # .erg files
        ├── erg_reader.{hpp,cpp}
        └── test_*.cpp
```

### ERG 파일 접근

사용자 결정: **ERGAccess (CarMaker 공식 C API)**. Lab 의 CarMaker 라이선스 사용. CMake 옵션 `VDSIM_WITH_ERGACCESS=OFF` default. PoC 는 OFF, Phase 2 에 활성화.

## 3. 검증 방법 (근거)

명세 단계는 "통과 기준이 명확한가, cherry-picking 방지 가능한가" 검증.

| 기준 | 평가 |
|---|---|
| 해석해 closed-form 존재 | yes (linear bicycle 4 시나리오 모두) |
| RMSE / 시간 / 거리 등 정량 메트릭 | yes (시나리오별 표) |
| Sign convention 명시 | yes (ISO 8855 RH, task 01) |
| Tolerance 사전 명시 | yes (10%, 5%, 2°, 0.5m, ...) |

## 4. 검증 결과

| 시나리오 | 명세 완료 | 구현 완료 | 통과 |
|---|---|---|---|
| #1 SS cornering | yes | yes (task 11) | **pass** (10% tol 이내) |
| #2 Step steer | yes | task 13 예정 | — |
| #3 Accel | yes | task 13 예정 | — |
| #4 Brake | yes | task 13 예정 | — |

자세한 #1 결과는 [task 11 보고서](../11_W4_bicycle_dynamics/README.md) §4.

## 5. 판단

- 결과: **partial** (명세 완료 + 시나리오 #1 구현 + 통과, 3 개 시나리오 미구현)
- 근거: D17 의 사전-설정 통과 기준이 #1 시나리오에서 시뮬-해석 10% 이내로 충족. 나머지 3 개는 task 13 일정에 포함.
- Follow-up:
  - Step steer (task 13).
  - Acceleration / brake (task 13).
  - CarMaker ERG 비교: Phase 2, lab 라이선스 + ERGAccess SDK 위치 확인 필요.
  - L2 / L3 검증: W9-W12.
