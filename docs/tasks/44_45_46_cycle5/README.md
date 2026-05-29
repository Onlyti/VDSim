# Task 44-46 — EBD + Pybind11 + Spdlog systematic

| Field | Value |
|---|---|
| Task ID | IM-W11-6 (cluster) |
| Type | Impl |
| Date | 2026-05-29 |
| Commit | TBD |
| Status | completed |

## 1. 목적

| Task | 목적 |
|---|---|
| 44 — Brake EBD | static brake_bias_front 의 한계 — load 따른 동적 균형 |
| 45 — Pybind11 | Python 에서 vdsim_core 직접 호출 → notebook / sweep / RL 환경 |
| 46 — Spdlog | 모든 dynamics 의 initialize 추적, NaN 디버깅 기반 |

## 2. 구현

### 2.1 Task 44 — Brake EBD

- VehicleParams: `brake_ebd_enabled` (bool, default false)
- bicycle / seven_dof: EBD on 시 `bias = Fz_f / (Fz_f + Fz_r)`, clamped [0.05, 0.95]
- Static `brake_bias_front` 무시 (override)

### 2.2 Task 45 — Pybind11 module

- `python/bindings.cpp` — VehicleParams / TireParams / SolverParams / State / ContactPoint / CmdL4 / IVehicleDynamics 노출
- 3 factory: create_bicycle / create_seven_dof / create_fourteen_dof
- 2 controller: LongAxController / LongVxController
- `python/CMakeLists.txt` — pybind11 자동 검색 + module 빌드
- `third_party/CMakeLists.txt` — PIC 강제로 shared 빌드 호환

### 2.3 Task 46 — Spdlog systematic

- L1 bicycle initialize() 에 `spdlog::debug` 로 차종 metadata 로깅
- 환경변수 `SPDLOG_LEVEL=debug` 로 활성화
- 다른 dynamics 의 systematic logging 은 동일 패턴 — 본 task 는 L1 만 (코드 패턴 demonstration)

## 3. 검증

### 3.1 EBD test (1 새)

| Test | 항목 | Pass |
|---|---|---|
| WeightTransfer.EBDApproximatesDynamicLoadDistribution | hard brake → Fz_f 비율 > 0.55 | pass + mass conservation |

전체 136/136 통과.

### 3.2 Python module 검증

```python
import vdsim
vp = vdsim.VehicleParams.from_yaml('configs/vehicles/sedan.yaml')
tp = vdsim.TireParams.from_yaml('configs/tires/default_pacejka.yaml')
sp = vdsim.SolverParams()

dyn = vdsim.create_seven_dof()
dyn.initialize(vp, tp, sp)
s = vdsim.State(); s.velocity = [10.0, 0.0, 0.0]; dyn.reset(s)

contacts = [vdsim.ContactPoint() for _ in range(4)]
for c in contacts: c.is_valid = True; c.normal = [0,0,1]; c.mu_long = c.mu_lat = 1.0

cmd = vdsim.CmdL4(); cmd.steer_angle_wheel = 0.05
for _ in range(1000): dyn.step(cmd, contacts, 0.005)
# Result: vx_end = 9.250, r = 0.1676 (matches C++ within numerical eps)
```

### 3.3 Spdlog 동작 검증

`SPDLOG_LEVEL=debug bin/vdsim_unit_tests` — initialize 시 log 출력 확인 가능.

## 4. 판단

- 결과: **pass**
- 근거:
  - 1 새 test 통과, 누적 136/136.
  - Python 에서 L2 7-DOF 동역학 호출 → C++ 와 동일 결과 (vx 9.250, r 0.168 vs C++ identical).
  - EBD 가 dynamic Fz 비율 추적.
- 미해결 / Follow-up:
  - L2/L3 의 spdlog 로깅 확장 (동일 패턴 적용).
  - Pybind11 의 추가 API 노출: ContactArray (현재 list 처리), PurePursuit, Scenario.
  - Python 의 sweep_runner 와 통합 (subprocess 대신 직접 호출).
