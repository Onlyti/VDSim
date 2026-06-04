# VDSim

[English](README.md) · **한국어**

차량 설계와 자율주행 평가를 잇는 오픈코어 차량 동역학 시뮬레이션 플랫폼.

📖 **문서 (이론 + 리포트):** https://onlyti.github.io/VDSim/

> *포지셔닝*: 외부 시각화/센서는 외부 도구에 위임 (CARLA 등). VDSim 은
> **정확하고 검증된 차량 동역학** + **하드포인트 기반 설계 검증** +
> **FMI 2.0 양방향 통합** 을 책임진다.

## 디렉토리 구조

| 디렉토리 | 내용 |
|---|---|
| `core/` | `libvdsim_core` — C++17 standalone 라이브러리. Ld1 Bicycle / Ld2 7-DOF / Ld3 14-DOF 동역학 + Pacejka MF96 타이어 (load sensitivity + relaxation length + camber thrust/Mz) + Lc5-Lc8 control converter. Ld4 hardpoint kinematics (DW/MP/TA/5-link, lookup + native solver). ISO 8855 RH. |
| `python/` | pybind11 바인딩 (`vdsim` 모듈): VehicleParams / TireParams / SolverParams, ITireModel, ISuspensionKinematics + attach helper, 전 동역학 + Lc 제어기. |
| `tools/kinematics/` | Offline hardpoint solver (DW 2D/3D, MacPherson, trailing arm, 5-link), 진단 도구 + Adams CSV importer + matplotlib GUI. |
| `gui/` | Three.js 실시간 웹 뷰어 (PoC) — 라이브 sim 을 구독해 3D 뷰 / 노면 / telemetry 렌더링. |
| `builder/` | 실험 저작 웹 툴 (vehicle / sensor / map / comms / scenario) · 라이브 운동학 곡선이 있는 웹 기반 suspension editor. |
| `carla_integration/` | Python bridge — CARLA actor 를 VDSim 동역학으로 구동; raycast contact; Ld4 kinematics attach 지원. |
| `apps/jump_demo/` | T23/T24 — 2D + 3D 선회 점프 시뮬레이터 (Phase-2 14-DOF prototype, world-z + airborne + Pacejka). |
| `apps/doe/` | Design-of-Experiments 러너 — 다중 파라미터 × 다중 시나리오 sweep → CSV + heatmap. |
| `apps/validation/` | ISO 7401 (step steer), ISO 4138 (정상상태 선회), ISO 3888-2 (double lane change) — metric 자동 추출 + 리포트. |
| `fmi_export/` | FMI 2.0 Co-Simulation export (L2 + L3 FMU) 와 import (`fmu_master.py` — ctypes 로 임의 FMI 2.0 CS FMU 로드). |
| `configs/` | `vehicles/`, `tires/`, `suspensions/` (DW/MP/TA/5-link YAML), `scenarios/`. |
| `tests/` | `unit/` + `integration/` (187 tests, 100% 통과). |

## 빌드

```bash
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release \
    -DVDSIM_BUILD_PYTHON=ON -DVDSIM_BUILD_CARLA_PLUGIN=ON
cmake --build build -j
(cd build && ctest --output-on-failure)
```

요구사항: cmake ≥ 3.16, ninja, g++ ≥ 9 / clang ≥ 8, python3 + pybind11.

## 빠른 둘러보기

### 1. 동역학 + 제어 (C++)
```bash
bin/vdsim_scenario_run configs/vehicles/sports.yaml \
                       configs/tires/default_pacejka.yaml \
                       configs/scenarios/double_lane_change.yaml /tmp/out.csv
```

### 2. Hardpoint suspension 설계 (Python)
```bash
# Hardpoint 운동학 곡선 계산
python3 tools/kinematics/dw_3d_solver.py \
        --config configs/suspensions/dw_front_sports.yaml

# 자기일관성 진단
python3 tools/kinematics/diagnose.py \
        --config configs/suspensions/dw_front_sports.yaml

# 인터랙티브 웹 에디터
python3 builder/suspension_editor_server.py &
( cd builder && python3 -m http.server 8090 )
# 브라우저 → http://localhost:8090/suspension_editor.html
```

### 3. 파라미터 sweep / 설계 탐색
```bash
python3 apps/doe/sweep_runner.py --config apps/doe/example_sweep.yaml
# → CSV + 1-D plot / 2-D heatmap / sensitivity bar
```

### 4. ISO maneuver 검증
```bash
python3 apps/validation/run_validation.py \
        --vehicle configs/vehicles/sports.yaml \
        --tire    configs/tires/default_pacejka.yaml \
        --level   L3 \
        --out     /tmp/validation_report
# → REPORT.md + ISO 7401 + ISO 4138 + ISO 3888-2 metric + plot
```

### 5. FMI export (산업 co-simulation)
```bash
# L2 FMU 빌드
bash fmi_export/build_fmu.sh
# 출력: build/fmi_export/vdsim_l2.fmu

# Ld4 kinematics 포함 L3 FMU 빌드
FRONT_KIN_CSV=docs/tasks/T27_ld4_dw/run3d/sweep_3d.csv \
REAR_KIN_CSV=docs/tasks/T30_ld4_5link/run01/sweep_3d.csv \
bash fmi_export/build_l3_fmu.sh
# 출력: build/fmi_export/vdsim_l3.fmu  (4.6 MB)

# Round-trip 동등성 검증 (native VDSim 대비)
python3 fmi_export/test_roundtrip.py
# → max |Δvx| = 0.000e+00  (수치 정밀도)
```

### 6. FMI import (ctypes 로 임의 FMU 로드)
```python
from fmi_export.fmu_master import FMUMaster
fmu = FMUMaster.load("any_compliant.fmu")
fmu.initialize(0.0)
fmu.set("throttle", 0.3); fmu.set("steer_angle_wheel", 0.05)
fmu.do_step(0.0, 0.02)
print(fmu.get("vx"), fmu.get("yaw_rate"))
```
→ 우리 `vdsim_l2.fmu`, `vdsim_l3.fmu`, Chrono Vehicle FMU, CarMaker FMU export, Modelica 생성 FMU 등과 호환.

### 7. CARLA + VDSim bridge
```bash
# CARLA 서버 시작 (~ /path/to/CarlaUE4.sh)
python3 carla_integration/python/run_demo.py \
        --vehicle configs/vehicles/sports.yaml \
        --tire    configs/tires/default_pacejka.yaml \
        --level   L3 \
        --kinematics_front docs/tasks/T27_ld4_dw/run3d/sweep_3d.csv \
        --kinematics_rear  docs/tasks/T30_ld4_5link/run01/sweep_3d.csv \
        --driver --duration 15 --v_target 10
```

## 규약

- 단위: SI (m, kg, s, rad, N, N·m). 내부에서 cm / deg 안 씀.
- Body frame: ISO 8855 RH — X 전방, Y 좌측, Z 상방.
- World frame: ENU RH.
- Quaternion: body → world (Eigen convention).
- Euler: ZYX intrinsic (yaw → pitch → roll).
- Wheel index: FL = 0, FR = 1, RL = 2, RR = 3.

## 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│ 외부 도구                                                      │
│  · CARLA (센서 + 렌더링 + 시나리오)                            │
│  · CarMaker / dSPACE / Modelica (FMI 2.0 경유)                │
│  · Chrono Vehicle (FMI 2.0 경유)                              │
└──────────────────┬─────────────────────────────┬─────────────┘
       FMI import  │                  CARLA      │
       (fmu_master)│                  bridge     │  FMI export
                   ▼                             ▼  (build_fmu.sh)
┌──────────────────────────────────────────────────────────────┐
│ VDSim core (C++17)                                            │
│  · Ld1 Bicycle / Ld2 7-DOF / Ld3 14-DOF                       │
│  · Pacejka MF96 (load sens + relaxation + camber)             │
│  · Ld4 hardpoint kinematics (DW / MP / TA / 5-link)           │
│  · Lc5-Lc8 control cascade (pure pursuit, vx PID, ax PID)     │
└──────────────────┬─────────────────────────────┬─────────────┘
                   │                             │
                   ▼                             ▼
        Python bindings                Validation + DOE
        (pybind11)                     (ISO 7401/4138/3888-2)
                                       Web GUI editor
```

## 검증 현황 (sports.yaml @ L3)

| 시험 | 결과 |
|---|---|
| ISO 7401 step-steer (6°, 80 km/h) | U = 1.21 (20.6% overshoot), T_ψ̇ = 0.20 s |
| ISO 4138 understeer gradient | K = +9.69 mrad/g (UNDERSTEER, sports 표준) |
| ISO 3888-2 DLC @ 60 km/h | FAIL (excursion 1.3 m, speed loss 5.5 km/h) |
| ISO 3888-2 DLC @ 40 km/h | PASS (excursion 0.3 m, speed loss 1.0 km/h) |
| FMU export round-trip | max \|Δoutput\| = 0 (수치 정밀도) |
| ctest | **187 / 187 통과** |

## 문서

전체 이론 reference + PoC 리포트가 MkDocs Material (MathJax 수식, 검색,
다크모드) 로 **https://onlyti.github.io/VDSim/** 에 배포됨.

이론 챕터 (각 챕터는 수식 + 구현의 `file:line` + 검증 테스트를 짝지음):

| # | 챕터 | # | 챕터 |
|---|---|---|---|
| 01 | 좌표계 & 규약 | 09 | Pure pursuit / path |
| 02 | 강체 동역학 | 10 | Driver model |
| 03 | 타이어 (Pacejka MF96) | 11 | 수치 적분 |
| 04 | Ld1-Bicycle | 12 | 소프트웨어 아키텍처 |
| 05 | Ld2-SevenDOF | 13 | Multibody (Ld4) 개요 |
| 06 | Ld3-FourteenDOF | 14 | Hardpoint kinematics |
| 07 | Control ladder Lc1-Lc8 | 15 | 검증 & DOE |
| 08 | PID 제어기 | 16 | FMI 2.0 통합 |

로컬에서 문서 빌드:
```bash
pip install mkdocs-material pymdown-extensions mike
mkdocs serve        # → http://localhost:8000/VDSim/
```

## 상태

| Phase | 마일스톤 | 상태 |
|---|---|---|
| 0 | Skeleton + 빌드 sanity | ✅ |
| 1 | Core 인터페이스 + bicycle | ✅ |
| 2 | CARLA 통합 | ✅ Python bridge + raycast |
| 3 | 7-DOF + raycast contact | ✅ |
| 4 | 14-DOF + ride dynamics | ✅ Sprung 3 + Unsprung 4 DOF |
| 5 | Control cascade L4-L8 | ✅ Pure pursuit, vx/ax PID, Driver |
| 6 | Pybind11 모듈 | ✅ |
| 7 | 타이어 업그레이드 | ✅ Load sens + relaxation + camber Mz |
| 8 | Ld4 hardpoint framework | ✅ 4 suspension type + native solver |
| 9 | DOE / 파라미터 sweep | ✅ |
| 10 | ISO 검증 (7401/4138/3888-2) | ✅ |
| 11 | FMI 2.0 export (L2 + L3) | ✅ |
| 12 | FMI 2.0 import (generic master) | ✅ |
| 13 | SMPC / MPC 제어기 | Phase 2 (HPIPM 통합) |
| 14 | Ld5 compliance (부싱) | Phase 2 |

## 라이센스

- Core / kinematics / tools / validation / FMI export (본 repo): Apache-2.0.
- FMI 2.0 헤더 (`fmi_export/fmi2/`): BSD-2-Clause (Modelica Association).
- Third-party (Eigen / yaml-cpp / spdlog / gtest): 각 오픈소스 라이센스, FetchContent 로 vendored.
