# Task 40 — VDSim PoC W1-W12 종합 요약

| Field | Value |
|---|---|
| Task ID | RPT-PoC-Summary |
| Type | Report |
| Date | 2026-05-29 |
| Status | snapshot |

## 1. 목적

W1-W12 PoC 의 전체 진척, 코드/테스트 산출물, 기능 매트릭스, 미해결 항목 / Phase 2 backlog 를 한 보고서로 정리. 시연 / 발표 / 인수인계용.

## 2. 산출물 정량

| 카테고리 | 수치 |
|---|---:|
| Total tasks completed | 32 (D7~D11 design 6, IM-W1~IM-W5 + W11 impl 26+) |
| Reports authored | 32 |
| Source files (core/src) | 12 cpp |
| Headers (core/include/vdsim) | 9 |
| Test files | 14 (unit + integration) |
| **Tests passing** | **127 / 127 (100%)** |
| Example binaries | 9 (vdsim_unit_tests, vdsim_integration_tests, vdsim_bicycle_run, vdsim_scenario_run, vdsim_l1_vs_l2, vdsim_ax_track_demo, vdsim_split_mu_demo, vdsim_l3_demo, vdsim_path_tracking) |
| Vehicle configs | 4 (sedan, sports, FSK formula, race car) |
| Tire configs | 1 (default Pacejka MF96) |
| Solver configs | 2 (RK4 1ms, Euler 10ms) |
| Scenario configs | 4 (step_steer, double_lane_change, throttle_brake_sequence, ice_patch) |
| Sweep configs | 1 (aero × vx) |

## 3. 사다리 구현 현황

### 3.1 Dynamics 사다리 (D8)

| Level | DOF | 구현 상태 | 검증 |
|---|---|---|---|
| L1 Bicycle | 5 (planar 3 + 2 wheel) | **Full** | analytical SS (±2 % linear region), longitudinal weight transfer, brake bias |
| L2 7-DOF | 7 (planar 3 + 4 wheel) | **Full** | per-tire Fz, lateral + longitudinal weight transfer, differential 3 modes, Ackerman |
| L3 14-DOF | 14 (planar 3 + 4 wheel + 3 sprung vertical + 4 unsprung) | **70%** (3 sprung vertical DOF dynamic; unsprung mass 4 DOF deferred) | roll oscillation, pitch transient, anti-dive, mass conservation |

### 3.2 Control 사다리 (D11)

| Level | 입력 | 구현 |
|---|---|---|
| L1 motor_torque per wheel | N·m × 4 | Variant 정의, dispatch 미구현 |
| L2 axle drive_torque | N·m | Variant 정의, dispatch 미구현 |
| L3 Fx_total + steer | N + rad | Variant 정의, dispatch 미구현 |
| **L4 throttle/brake/steer** | [0,1] × rad | **Full PoC** — L1/L2/L3 의 step() 직접 처리 |
| **L5 ax_target** | m/s² | **Full** — PI + feed-forward |
| **L6 v_target** | m/s | **Full** — cascade PI → L5 |
| **L7 Pure Pursuit (path curvature)** | waypoint | **Full** — lookahead + atan(κ·L) |
| **L8 waypoint path** | path[N] | **Full** — figure-eight closed-loop demo |

L1~L3 direct dispatch / variant visit pattern은 Phase 2 (CARLA plugin 통합 시).

### 3.3 Tire model

| 항목 | 구현 |
|---|---|
| Pacejka MF96 simple form (long, lat) | yes |
| Linear tire (fallback) | yes |
| **Combined slip (friction ellipse)** | yes |
| **Aligning moment Mz (pneumatic trail + falloff)** | yes |
| **Mz body-frame aggregation** | yes (L1, L2) |
| Camber thrust (linear) | yes (API + MF96) |
| MF2002 / .tir importer | Phase 2 |
| Camber-Mz coupling | Phase 2 |

## 4. W1-W12 계획 대비 진척

| Week | 계획 | 진척 | % |
|---|---|---|---:|
| W1 | Skeleton + CI | done | 100 |
| W2 | Headers + coordinate | done | 100 |
| W3 | Tire models | done | 100 |
| W4 | L1 + params YAML + scenarios | done | 100 |
| W5-W6 (자체 보강) | combined slip / Mz / weight transfer / scenario DSL / L2 skeleton | done | 100 |
| **W7-W8 (CARLA plugin)** | raycast contact provider + UE4 통합 | **not started** | **0** |
| W9-W10 (L2) | full L2 impl + Ackerman + diff | done | 100 |
| W11-W12 (L3 + ride/handling) | full 14-DOF + CarMaker ERG | partial (3/4 vertical DOF dynamic) | 70 |
| **Phase 2 미리 흡수** | combined slip / Ackerman / diff / aero / brake bias | done | — |

**PoC W1-W12 전체 진척**: ~ **80%** (CARLA, full unsprung mass, CarMaker ERG 미반영).

## 5. 차종별 거동 비교 (step_steer δ=0.05, vx=10, 5 s)

| Vehicle | r_end [rad/s] | vx_end [m/s] | 비고 |
|---|---:|---:|---|
| sedan | 0.1759 | 9.728 | RWD Open diff, 60 % Ackerman |
| sports | 0.1848 | 9.692 | LSD, 85 % Ackerman, downforce |
| FSK formula | 0.2621 | 8.634 | 매우 짧은 wheelbase, spool diff |
| race car | (DLC 별도) | — | AWD LSD, 큰 downforce |

## 6. 코드 베이스 통계

```
core/include/vdsim/
├── contact.hpp
├── control.hpp
├── control_converter.hpp
├── coordinate.hpp
├── interfaces.hpp
├── params.hpp
├── scenario.hpp
├── state.hpp
├── types.hpp
└── version.hpp

core/src/
├── bicycle_dynamics.cpp        L1
├── seven_dof_dynamics.cpp      L2
├── fourteen_dof_dynamics.cpp   L3 (3 sprung vertical DOF)
├── pacejka_mf96.cpp            tire (combined slip + Mz + camber)
├── linear_tire.cpp             tire fallback
├── contact_providers.cpp       flat ground
├── control_converter.cpp       L5/L6/L7
├── scenario.cpp                YAML scenario DSL
├── coordinate.cpp              quat/euler/frame conversions
├── params.cpp                  Vehicle/Tire/Solver YAML I/O
├── version.cpp
└── dynamics_stubs.cpp          (현재 사실상 빈 stub — all factories 활성화)
```

## 7. 미해결 / Phase 2 backlog

| Priority | 항목 | 예상 노력 |
|---|---|---|
| H | **CARLA plugin** (W7-W8 의 통째 미수행) | 2-3 주 |
| H | **L3 unsprung mass 4 DOF** | 1-2 주 |
| H | **CarMaker ERG 비교** (D17 의 RMSE 기준 검증) | 2 주 (license 필요) |
| M | **MPC / SMPC** path tracking (PP 대체) | 3-4 주 (HPIPM 통합) |
| M | **MF2002** tire (Mzr + camber-Mz coupling) | 1-2 주 |
| M | **AVL .tir importer** | 1 주 |
| M | **Python bindings (pybind11)** | 1 주 |
| L | **Camber-from-roll wiring** (L3 phi → wheel gamma) | 2-3 일 |
| L | **Dynamic Ackerman** (vx-dependent) | 1-2 일 |
| L | **Brake EBD (dynamic balance)** | 1-2 일 |
| L | **차종별 실측 fit** (TUR / FSK / NGII) | 1 주 each |
| L | **Logging 의 spdlog systematic 활용** | 2-3 일 |
| L | **NaN/Inf 입력 guards** in dynamics | 1 일 |

## 8. 강점 / 약점

### 강점
- Dynamics 사다리 L1-L3 전 부분 동작 (L3 70%).
- Control 사다리 L4-L8 cascade 동작 (closed-loop path tracking 검증).
- Tire physics 의 Phase 2 항목 (combined slip / Mz / camber API) 모두 흡수.
- 127 tests, 모든 task 보고서 (목적/구현/검증/판단), 시각화 그래프 누적.
- 차종별 distinct configs (sedan / sports / FSK / race), distinct 거동 검증.
- Scenario YAML DSL + time-varying mu + sweep wrapper — 외부 분석 자동화 가능.

### 약점
- **CARLA plugin 통째 0** — D8/D11 의 차별화 메시지 (UE5 / PhysX 대안) 의 시연 베이스 없음.
- **CarMaker ERG 비교 미수행** — D17 의 RMSE 기준 만족 여부 미확인.
- **L3 의 unsprung mass 4 DOF 미구현** — 14-DOF의 마지막 30%.
- **MPC 미구현** — SMPC paper 와의 직접 연동 base 없음.
- **Python binding 미구현** — 외부 사용성 떨어짐.
- **실측 데이터 calibration 0** — 모든 차종 default 가 indicative.

## 9. 결론

PoC W1-W12 의 약 **80%** 완성. dynamics + control + tire physics 측면에서는 Phase 2 의 핵심 항목까지 미리 흡수했으며, 외부 통합 (CARLA, CarMaker) 측면이 미진행. 

**시연 가능 시나리오**:
- 단순: step steer / DLC / throttle-brake (sedan / sports / FSK / race 4 차종)
- 중급: closed-loop figure-eight path tracking (L5-L8 cascade)
- 고급: L3 의 dynamic roll/pitch transient (3 sprung vertical DOF)

**다음 우선 작업** (Phase 2 시작):
1. CARLA plugin skeleton (raycast IContactProvider impl)
2. L3 unsprung mass 추가 (full 14-DOF)
3. CarMaker ERG 비교 환경 구축
