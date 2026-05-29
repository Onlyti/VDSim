# Task 47-49 — MF96 validation + CSV importer + MPC (deferred)

| Field | Value |
|---|---|
| Task ID | IM-W11-7 (cluster) |
| Type | Tooling + Validation |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed (47 deferred to Phase 2 SMPC paper) |

## 1. 진척

- **Task 47 (MPC LQR baseline)** — **Deferred**. SMPC paper / HPIPM 통합과 함께 본격 구현 (T-VT/T-IV/T-ITS target). 본 cycle 에서는 simulator-side baseline 만 (PurePursuit) 으로 충분.
- **Task 48 (MF96 validation curves)** — Done. textbook 일치 시각화.
- **Task 49 (CSV importer)** — Done. ADMA / CarMaker 출력 import 가능.

## 2. 구현

### 2.1 Task 48 — MF96 validation

`docs/figures_src/plot_mf96_validation.py`:
- Fy vs alpha 곡선 (Fz ∈ {2, 4, 6, 8 kN})
- Fx vs kappa 곡선 동일
- Friction ellipse traversal at Fz = 4 kN, kappa = {0, 0.05, 0.10, 0.15}

수치: D_lat·Fz=4000 = 4000 N at peak. matches mu·Fz = 1.0·4000 = 4000 N. ✓

### 2.2 Task 49 — CSV importer

`python/csv_to_scenario.py`:
- CSV row → scenario.yaml control 시퀀스
- 컬럼 자동 매핑 (time / throttle / brake / steer 등)
- linear / zoh interp 선택
- subsample 옵션 (telemetry 대량 dataset 처리)

End-to-end test:
```
$ python3 csv_to_scenario.py /tmp/test_meas.csv /tmp/imported.yaml
wrote 4 samples to /tmp/imported.yaml

$ bin/vdsim_scenario_run sedan.yaml tire.yaml /tmp/imported.yaml /tmp/run.csv
[vdsim_scenario_run] imported_step_steer: vx 10.000 -> 9.923 m/s, r 0.1796 rad/s
```

## 3. 검증

- 136/136 test 그대로 통과 (회귀 없음).
- MF96 figure 정량값 일치: D·Fz·mu peak = 4000 N at Fz=4000.
- CSV importer end-to-end pipeline 동작.

## 4. 판단

- 결과: **pass** (47 defer, 48/49 done)
- Follow-up:
  - **Task 47 MPC LQR** — Phase 2 의 SMPC paper 와 함께. 2x2 kinematic bicycle CARE + gain scheduling.
  - **MF2002 의 advanced peak / curvature** — Phase 2.
  - **ADMA / CarMaker 실측 데이터** — TUR / FSK telemetry 와의 fit.
