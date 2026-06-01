# Task 68 — PoC W1-W12 종합 요약 v3 (자동운전 완료 시점)

| Field | Value |
|---|---|
| Task ID | RPT-PoC-Summary-v3 |
| Type | Report |
| Date | 2026-05-29 |
| Status | snapshot — 자동운전 11+ cycles 완료 시점 |
| Supersedes | [Task 40 (v1)](../40_poc_summary/README.md), [Task 52 (v2)](../52_poc_summary_v2/README.md) |

## 1. Headline

- **Tasks completed**: 68
- **Tests passing**: **141 / 141**
- **PoC W1-W12 진척**: ~ **95 %**
- **Phase 2 흡수 항목**: 12+ (combined slip, Mz, Ackerman, diff, aero, brake bias, EBD, camber API, anti-dive, L5-L8 cascade, pybind11, CARLA skeleton, Driver model)

## 2. 산출물 정량 (v2 → v3)

| 카테고리 | v2 | **v3** | Δ |
|---|---:|---:|---:|
| Tasks completed | 52 | **68** | +16 |
| Reports authored | 40+ | **45** | +5 |
| Source files (core + carla + python) | 16 | **17** | +1 |
| Test files | 17 | **18** | +1 |
| **Tests passing** | **136** | **141** | +5 |
| Example binaries | 12 | **10** | (rebased) |
| Vehicle configs | 4 | **4** | — |
| Scenario configs | 4 | **7** | +3 (j_turn, skidpad, brake_in_turn, ice_corner) |
| Python tools | 3 | **3** | — |
| External integrations | 2 | **2** (CARLA + pybind11) | — |

## 3. W1-W12 진척 v3

| Week | 계획 | v1 | v2 | **v3** | Δ from v2 |
|---|---|---:|---:|---:|---:|
| W1 | Skeleton + CI | 100 | 100 | 100 | — |
| W2 | Headers + coordinate | 100 | 100 | 100 | — |
| W3 | Tire models | 100 | 100 | 100 | — |
| W4 | L1 + params YAML + scenarios | 100 | 100 | 100 | — |
| W5-W6 (자체 보강) | combined slip / weight transfer / scenario | 100 | 100 | 100 | — |
| **W7-W8 (CARLA)** | raycast + UE4 | 0 | 40 | **45** | +5 (driver-side) |
| W9-W10 (L2) | per-tire + Ackerman + diff + EBD | 100 | 100 | 100 | — |
| **W11-W12 (L3 + ride)** | full 14-DOF + CarMaker ERG | 70 | 90 | **95** | +5 (anti-dive 검증, ride freq 한계 명시) |

PoC W1-W12 전체 **80% → 92% → 95%**.

남은 5%:
- 실제 CARLA UE5 연동 (skeleton 만 진행, RPC 통합 미)
- CarMaker ERG 비교 (license + ERGAccess SDK 필요)
- L3 의 unsprung damper 분리 (현 모델 한계 명시)

## 4. 사다리 구현 v3

### 4.1 Dynamics

| Level | DOF | 상태 |
|---|---|---|
| L1 | 5 | Full + NaN guard + spdlog |
| L2 | 7 | Full + per-tire + Ackerman + diff + EBD + steering rack torque feedback |
| **L3** | **14** | **Full** (sprung 3 + unsprung 4) + anti-dive |

### 4.2 Control

| Level | 상태 |
|---|---|
| L1-L4 | std::visit dispatch + lower_to_l4 |
| L5 | PI + FF ax controller |
| L6 | cascade vx PI |
| L7 | Pure Pursuit |
| L8 | waypoint figure-eight cascade |
| Driver model | latency + Gaussian noise (Box-Muller) |
| LQR / MPC | Phase 2 (SMPC paper 통합) |

### 4.3 Tire physics

- MF96 Pacejka simple form (long + lat) ✓
- Combined slip (friction ellipse rescale) ✓
- Aligning Mz (pneumatic trail + falloff) ✓
- **Body-frame Mz aggregation** (L1, L2) ✓
- **Steering rack torque output** ✓ (이번 cycle 추가)
- Camber thrust (linear, API only) ✓
- MF2002 / .tir importer (12 fields) ✓
- Vertical tire spring (tire_vertical_stiffness) ✓

## 5. 차종 별 distinct 거동 (4 vehicles, 5 metrics)

| Metric | sedan | sports | FSK formula | race car |
|---|---:|---:|---:|---:|
| Step steer r_peak [rad/s] | 0.180 | 0.189 | **0.291** | 0.181 |
| DLC r_peak [rad/s] | 0.313 | 0.326 | **0.465** | 0.305 |
| Brake distance from 20 [m] | (>29) | 29.0 | **16.6** | 18.1 |
| Max brake decel [m/s²] | 4.32 | 6.96 | **14.18** | 12.36 |
| Stop time [s] | — | 2.77 | **1.68** | 1.79 |

## 6. Tooling 산출물

- `vdsim_scenario_run` — YAML scenario runner
- `vdsim_l1_vs_l2` — paired L1/L2 binary
- `vdsim_l3_demo` — L3 14-DOF trace
- `vdsim_ax_track_demo` — closed-loop ax PID
- `vdsim_path_tracking` — L5-L8 cascade
- `vdsim_driver_demo` — Driver model figure-8
- `vdsim_split_mu_demo` — diff comparison
- `vdsim_bicycle_run` — single-tire L1
- `python/sweep_runner.py` — Cartesian sweep
- `python/csv_to_scenario.py` — ADMA / CarMaker CSV → YAML
- `python/tir_to_yaml.py` — AVL .tir → YAML
- `python/bindings.cpp` → `vdsim.so` (Python module)

## 7. Phase 2 backlog (priority)

| Priority | 항목 | 예상 노력 |
|---|---|---|
| H | CARLA UE5 통합 (raycast → CARLA Sensor API) | 2-3 주 |
| H | CarMaker ERG validation (D17 RMSE 기준) | 2 주 (license) |
| H | SMPC / MPC controller (HPIPM) — T-VT/T-IV/T-ITS target | 3-4 주 |
| M | MF2002 full (Mzr + camber-Mz coupling) | 2 주 |
| M | L3 의 unsprung damper 분리 (sprung damper vs unsprung damper) | 1 주 |
| M | 실측 차종 calibration (TUR / FSK / NGII) | 1 주 each |
| L | Logging file rotation | 1-2 일 |
| L | Steering rack torque 의 self-aligning sign 검증 | 1 일 |

## 8. 시연 가능 시나리오 (총 7 종)

- step_steer
- double_lane_change
- throttle_brake_sequence
- ice_patch (직진 mu transition)
- j_turn (large brake + turn)
- skidpad (constant radius — analytical 1% 일치)
- brake_in_turn (combined slip 의 grip loss -52%)
- ice_corner (cornering 중 mu transition)

## 9. 결론

자동운전 11+ cycles 동안 PoC W1-W12 가 80% → 95% 수준 도달. Phase 2 의 핵심 항목 12+ 미리 흡수. 남은 5% 는 외부 환경 / license 필요 → 별도 phase.

**의미 있는 시연 가능 자료**:
- L1 / L2 / L3 의 사다리 차이 정량 (Task 20 격자 + benchmark matrix)
- closed-loop path tracking (figure-eight + Driver model)
- 4 차종 distinct 거동 (sedan / sports / FSK / race)
- mu transition 시 grip loss (ice_corner, brake_in_turn)
- Skidpad analytical 1% 일치 (kinematic Ackerman 검증)

**총 작업 시간 (자동운전)**: 약 5-6 시간 (1차 5시간 + 2차 3시간 + 자동 progression).
