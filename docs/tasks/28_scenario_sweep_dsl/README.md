# Task 28 — Scenario sweep DSL (parameter grid runner)

| Field | Value |
|---|---|
| Task ID | IM-W5-14 |
| Type | Tooling |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

Task 13/18 의 single scenario YAML 위에 **parameter sweep** 정의. 격자 (vx, delta, mu, aero_lift, differential 등) 전체 sweep 을 한 명령으로 실행. C++ 코드 수정 없이 외부 sweep 정의 가능.

이게 빠지면:
- 격자 분석 (Task 20 의 L1/L2 비교 sweep) 마다 Python 스크립트 작성 필요.
- 차종별 / mu별 batch 회귀 검증이 번거로움.

## 2. 구현 방법

### 2.1 코드 추가

| 위치 | 역할 |
|---|---|
| `python/sweep_runner.py` | sweep YAML 읽고 Cartesian product → 각 cell 마다 binary 호출 |
| `configs/sweeps/aero_vs_vx.yaml` | 예제 sweep (3×3 = 9 cells) |

### 2.2 Schema

```yaml
base_vehicle:  configs/vehicles/sedan.yaml
base_tire:     configs/tires/default_pacejka.yaml
base_scenario: configs/scenarios/step_steer.yaml
binary:        vdsim_l1_vs_l2

sweep:
  - param:  vehicle.aero_drag_coeff
    values: [0.20, 0.30, 0.40]
  - param:  scenario.initial_vx
    values: [5.0, 10.0, 15.0]
```

`param` 의 prefix (`vehicle.` / `scenario.`) 가 어느 YAML 에 적용할지 결정.

### 2.3 출력

`<out_dir>/<tag>/` 폴더 마다 CSV (binary 별 명명 규칙 유지). `sweep_index.yaml` 에 모든 cell 정보 누적.

### 2.4 결정 / 한계

| 결정 | 채택 | 근거 |
|---|---|---|
| Python wrapper | yes | YAML / 격자 분석은 Python 으로 빠르게 처리. C++ binary 호출은 subprocess |
| Cartesian product | yes | linear sweep / Sobol / Latin hypercube 는 별도 task |
| In-place mutation | YAML deep merge | 사용자가 일부 필드만 override 가능 |

한계:
- **Parallel 실행 안 함** — 9 cells 직렬. 대규모 sweep 에는 multiprocessing 추가 필요.
- **Random sampling** — uniform grid 만. Sobol / LH 별도.
- **Output aggregation 안 함** — sweep_index.yaml 만, 본격 분석은 별도 plot 스크립트.

## 3. 검증

`configs/sweeps/aero_vs_vx.yaml` 으로 9 runs 정상 종료. 각 cell 의 `<tag>/step_steer_L1.csv`, `_L2.csv` 생성.

## 4. 판단

- 결과: **pass**
- 근거: 9/9 cell 정상 실행, sweep_index.yaml 생성.
- Follow-up:
  - **Parallel multiprocessing** — `pool.map` 으로 cell-level parallel.
  - **Sobol / LH sampling** — 고차원 sweep 시.
  - **Auto-aggregate to one DataFrame** — `pandas.concat` 헬퍼.
