# PoC task reports (index)

W1-W12 PoC 진행 동안 작성된 task 보고서 색인.

- **목적**: 각 task 의 의도 / 구현 / 검증 / 판단을 self-contained 로.
- **PoC progress**: 약 95 % (Phase 2 항목 12+ 흡수).
- **141 / 141 tests passing**.

## Highlight reports

| # | Title |
|---|---|
| [68 — PoC summary v3 (95 %)](68_poc_summary_v3/README.md) | 종합 현황, m × n cascade 매트릭스, Phase 2 backlog |
| [69 — Competitive matrix](69_competitive_matrix/README.md) | Adams / VI / CarMaker / CarSim / Simulink 비교 + multibody M0 stub |
| [62 — Cycles overview](62_cycles_overview/README.md) | 자동운전 cycle 별 산출물 timeline + Phase 2 흡수 |
| [50 — Benchmark matrix (4 vehicles × 3 scenarios)](50_benchmark_matrix/README.md) | distinct vehicle 거동 |
| [57-59 — Driver / Skidpad / Brake-in-turn](57_58_59_driver_skidpad/README.md) | closed-loop demo + analytical validation |
| [20 — L1 vs L2 격자 SS sweep](20_l1_l2_grid_sweep/README.md) | linear region ↔ nonlinear region 비교 |
| [30 — L3 dynamic suspension](30_l3_dynamic_suspension/README.md) | sprung 3 DOF dynamic + anti-dive |
| [31-33 — L5/L6/L7/L8 control cascade](31_l6_l7_l8_control/README.md) | Lc5 → Lc8 figure-8 path tracking |

## All reports

W1-W12 cycle 별로 정렬. `docs/tasks/NN_*/` 구조.

01 ~ 06: Design 명세 (D7-D11, D17).
07 ~ 14: W1-W4 implementation (skeleton, headers, coordinate, tire, Ld1-Bicycle, scenarios, configs).
15 ~ 19: combined slip / Mz / weight transfer / scenario DSL / Ld2-SevenDOF.
20 ~ 24: Ackerman / roll-pitch / Mz aggregation / Ld3 skeleton.
25 ~ 29: Lc5 / differential / aero / sweep / brake bias.
30 ~ 39: Ld3 dynamic / Lc6-Lc8 / anti-dive + camber / extra configs.
40 ~ 49: PoC summary v1 / Cycle 4-6 (CARLA + pybind + EBD + spdlog + MF96 validation + CSV importer).
50 ~ 59: benchmark / L3 ride freq / PoC summary v2 / Driver + .tir + scenarios / Driver demo + skidpad.
60 ~ 69: NaN guards / understeer / brake distance / steering torque / ice corner / PoC summary v3 / competitive matrix.

전체 list 는 GitHub 에서 directory 탐색.
