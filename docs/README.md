# VDSim — Task Reports

각 task 의 목적 / 구현 / 검증 방법 / 검증 결과 / 판단을 기록한 보고서.
순서는 작업 시간순.

| # | Task | Type | Commit | Status |
|---|------|------|--------|--------|
| 01 | [D7 좌표계/단위 표준](tasks/01_D7_coordinate/README.md) | Design | `d267221` | completed |
| 02 | [D10 핵심 데이터 구조](tasks/02_D10_data_structures/README.md) | Design | `d267221` | completed |
| 03 | [D9 인터페이스 4종](tasks/03_D9_interfaces/README.md) | Design | `d267221` | completed |
| 04 | [D8 동역학 사다리 L1/L2/L3](tasks/04_D8_dynamics_ladder/README.md) | Design | `d267221` | completed |
| 05 | [D11 계층적 제어 API L1~L8](tasks/05_D11_control_api/README.md) | Design | `d267221` | completed |
| 06 | [D17 검증 baseline 시나리오](tasks/06_D17_validation_baseline/README.md) | Design | `d267221` | completed |
| 07 | [IM-W1 Monorepo skeleton + CI](tasks/07_W1_skeleton_and_ci/README.md) | Impl | `4449cf6` | completed |
| 08 | [IM-W2 인터페이스 헤더 7종](tasks/08_W2_headers/README.md) | Impl | `1e15c5e` | completed |
| 09 | [coordinate.cpp 구현](tasks/09_W2_coordinate_impl/README.md) | Impl | `56cc48a` | completed |
| 10 | [Pacejka MF96 + Linear tire](tasks/10_W3_tire_models/README.md) | Impl | `81f1dfc` | completed |
| 11 | [L1 Bicycle dynamics + 검증](tasks/11_W4_bicycle_dynamics/README.md) | Impl | `04ce58e` | completed |
| 12 | [params.cpp YAML I/O](tasks/12_W4_params_yaml/README.md) | Impl | TBD | completed |
| 13 | [Step steer + accel 시나리오](tasks/13_W4_validation_scenarios/README.md) | Impl | TBD | completed |
| 14 | [Example + sample configs](tasks/14_W4_example_configs/README.md) | Impl | TBD | completed |
| 15 | [Pacejka combined slip + Mz](tasks/15_combined_slip_mz/README.md) | Impl | TBD | completed |
| 16 | [TireParams::to_yaml + SolverParams YAML](tasks/16_tire_solver_yaml/README.md) | Impl | TBD | completed |
| 17 | [L1 longitudinal weight transfer](tasks/17_l1_weight_transfer/README.md) | Impl | TBD | completed |
| 18 | [Scenario YAML DSL + runner](tasks/18_scenario_dsl/README.md) | Impl | TBD | completed |
| 19 | [L2 7-DOF skeleton + weight transfer](tasks/19_l2_seven_dof/README.md) | Impl | TBD | completed |
| 20 | [L1/L2 격자 SS sweep 정량 비교](tasks/20_l1_l2_grid_sweep/README.md) | Validation | TBD | completed |
| 21 | [Per-axle Ackerman steering geometry](tasks/21_ackerman/README.md) | Impl | TBD | completed |
| 22 | [Roll/pitch state diagnostics](tasks/22_roll_pitch_diag/README.md) | Impl | TBD | completed |
| 23 | [Body-frame Mz aggregation L1/L2](tasks/23_mz_aggregation/README.md) | Impl | TBD | completed |
| 24 | [L3 14-DOF skeleton](tasks/24_l3_skeleton/README.md) | Impl | TBD | completed |
| 25 | [L5 ControlConverter ax PID](tasks/25_l5_controlconverter/README.md) | Impl | TBD | completed |
| 26 | [Differential Open/Locked/LSD](tasks/26_differential/README.md) | Impl | TBD | completed |
| 27 | [Aerodynamic downforce/lift](tasks/27_aero_downforce/README.md) | Impl | TBD | completed |
| 28 | [Scenario sweep DSL](tasks/28_scenario_sweep_dsl/README.md) | Tooling | TBD | completed |
| 29 | [Brake bias distribution](tasks/29_brake_bias/README.md) | Impl | TBD | completed |
| 30 | [L3 dynamic suspension (3 DOF)](tasks/30_l3_dynamic_suspension/README.md) | Impl | TBD | completed |
| 31-33 | [L6/L7/L8 control cascade](tasks/31_l6_l7_l8_control/README.md) | Impl | TBD | completed |
| 36-37 | [Anti-dive + camber thrust](tasks/36_anti_dive_camber/README.md) | Impl | TBD | completed |
| 38-39 | [Extra configs + time-varying mu](tasks/38_39_extra_configs/README.md) | Config | TBD | completed |
| 40 | [PoC summary report (v1, 80%)](tasks/40_poc_summary/README.md) | Report | TBD | completed |
| 41-43 | [Full 14-DOF + CARLA + L1-L3 dispatch](tasks/41_42_43_cycle4/README.md) | Impl | TBD | completed |
| 44-46 | [EBD + Pybind11 + Spdlog](tasks/44_45_46_cycle5/README.md) | Impl | TBD | completed |
| 48-49 | [MF96 validation + CSV importer](tasks/47_48_49_cycle6/README.md) | Tooling | TBD | completed |
| 50-51 | [Benchmark matrix + L3 ride freq](tasks/50_benchmark_matrix/README.md) | Validation | TBD | completed |
| 52 | **[PoC summary v2 (92%)](tasks/52_poc_summary_v2/README.md)** | Report | TBD | completed |

## 보고서 구조

모든 보고서는 [REPORT_TEMPLATE.md](REPORT_TEMPLATE.md) 형식.

1. 헤더 (메타데이터)
2. 목적
3. 구현 방법
4. 검증 방법 (근거)
5. 검증 결과 (표 / 그래프)
6. 판단 (pass/fail + 근거)

## 그래프 소스

`docs/figures_src/` 에 matplotlib 스크립트. 빌드한 후 실행해 `tasks/*/figures/*.png` 생성.
