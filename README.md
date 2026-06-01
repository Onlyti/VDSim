# VDSim

차량 설계와 자율주행 평가를 잇는 오픈코어 차량 동역학 시뮬레이션 플랫폼.

> *Positioning*: 외부 시각화/센서 (CARLA 등) 의존, 그 대비 정확하고 검증된
> 차량 동역학 + 하드포인트 기반 설계 검증 + FMI 양방향 통합.

## Layout

| Directory | Contents |
|---|---|
| `core/` | `libvdsim_core` — C++17 standalone library. Ld1 Bicycle / Ld2 7-DOF / Ld3 14-DOF dynamics + Pacejka MF96 tire (with load sensitivity + relaxation length + camber thrust/Mz) + Lc5-Lc8 control converters. Ld4 hardpoint kinematics (DW/MP/TA/5-link, lookup + native solvers). ISO 8855 RH. |
| `python/` | pybind11 bindings (`vdsim` module): VehicleParams / TireParams / SolverParams, ITireModel, ISuspensionKinematics + attach helpers, all dynamics + Lc controllers. |
| `tools/kinematics/` | Offline hardpoint solvers (DW 2D/3D, MacPherson, trailing arm, 5-link), diagnostic + Adams CSV importer + matplotlib GUI. |
| `viewer/` | Three.js 3D viewer + WebSocket realtime · web-based suspension editor with live kinematic curves. |
| `carla_integration/` | Python bridge — drives a CARLA actor with VDSim dynamics; raycast contacts; supports Ld4 kinematics attach. |
| `apps/jump_demo/` | T23/T24 — 2D + 3D turning-jump simulators (Phase-2 14-DOF prototype with world-z + airborne + Pacejka). |
| `apps/doe/` | Design-of-Experiments runner — multi-parameter × multi-scenario sweeps → CSV + heatmaps. |
| `apps/validation/` | ISO 7401 (step steer), ISO 4138 (steady-state circular), ISO 3888-2 (double lane change) — automated metric extraction + report. |
| `fmi_export/` | FMI 2.0 Co-Simulation export (L2 + L3 FMUs) and import (`fmu_master.py` — load any FMI 2.0 CS FMU via ctypes). |
| `configs/` | `vehicles/`, `tires/`, `suspensions/` (DW/MP/TA/5-link YAML), `scenarios/`. |
| `tests/` | `unit/` + `integration/` (165 tests). |

## Build

```bash
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release \
    -DVDSIM_BUILD_PYTHON=ON -DVDSIM_BUILD_CARLA_PLUGIN=ON
cmake --build build -j
(cd build && ctest --output-on-failure)
```

Requires cmake ≥ 3.16, ninja, g++ ≥ 9 / clang ≥ 8, python3 + pybind11.

## Quick tour

### 1. Dynamics + control (C++)
```bash
bin/vdsim_scenario_run configs/vehicles/sports.yaml \
                       configs/tires/default_pacejka.yaml \
                       configs/scenarios/double_lane_change.yaml /tmp/out.csv
```

### 2. Hardpoint suspension design (Python)
```bash
# Compute hardpoint kinematic curves
python3 tools/kinematics/dw_3d_solver.py \
        --config configs/suspensions/dw_front_sports.yaml

# Diagnostic check
python3 tools/kinematics/diagnose.py \
        --config configs/suspensions/dw_front_sports.yaml

# Interactive web editor
python3 viewer/suspension_editor_server.py &
( cd viewer && python3 -m http.server 8090 )
# Browser → http://localhost:8090/suspension_editor.html
```

### 3. Parameter sweep / design exploration
```bash
python3 apps/doe/sweep_runner.py --config apps/doe/example_sweep.yaml
# → CSV + 1-D plots / 2-D heatmaps / sensitivity bars
```

### 4. ISO maneuver validation
```bash
python3 apps/validation/run_validation.py \
        --vehicle configs/vehicles/sports.yaml \
        --tire    configs/tires/default_pacejka.yaml \
        --level   L3 \
        --out     /tmp/validation_report
# → REPORT.md + ISO 7401 + ISO 4138 + ISO 3888-2 metrics + plots
```

### 5. FMI export (industrial co-simulation)
```bash
# Build L2 FMU
bash fmi_export/build_fmu.sh
# Output: build/fmi_export/vdsim_l2.fmu

# Build L3 FMU with Ld4 kinematics
FRONT_KIN_CSV=docs/tasks/T27_ld4_dw/run3d/sweep_3d.csv \
REAR_KIN_CSV=docs/tasks/T30_ld4_5link/run01/sweep_3d.csv \
bash fmi_export/build_l3_fmu.sh
# Output: build/fmi_export/vdsim_l3.fmu  (4.6 MB)

# Round-trip equivalence check (vs native VDSim)
python3 fmi_export/test_roundtrip.py
# → max |Δvx| = 0.000e+00  (numerical precision)
```

### 6. FMI import (load any FMU via ctypes)
```python
from fmi_export.fmu_master import FMUMaster
fmu = FMUMaster.load("any_compliant.fmu")
fmu.initialize(0.0)
fmu.set("throttle", 0.3); fmu.set("steer_angle_wheel", 0.05)
fmu.do_step(0.0, 0.02)
print(fmu.get("vx"), fmu.get("yaw_rate"))
```
→ Works with our `vdsim_l2.fmu`, `vdsim_l3.fmu`, Chrono Vehicle FMU, CarMaker FMU export, Modelica-generated FMUs, etc.

### 7. CARLA + VDSim bridge
```bash
# Start CARLA server (~ /path/to/CarlaUE4.sh)
python3 carla_integration/python/run_demo.py \
        --vehicle configs/vehicles/sports.yaml \
        --tire    configs/tires/default_pacejka.yaml \
        --level   L3 \
        --kinematics_front docs/tasks/T27_ld4_dw/run3d/sweep_3d.csv \
        --kinematics_rear  docs/tasks/T30_ld4_5link/run01/sweep_3d.csv \
        --driver --duration 15 --v_target 10
```

## Conventions

- Units: SI (m, kg, s, rad, N, N·m). No cm / deg internally.
- Body frame: ISO 8855 RH — X forward, Y left, Z up.
- World frame: ENU RH.
- Quaternion: body → world (Eigen convention).
- Euler: ZYX intrinsic (yaw → pitch → roll).
- Wheel index: FL = 0, FR = 1, RL = 2, RR = 3.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│ External tools                                                │
│  · CARLA (sensors + rendering + scenarios)                    │
│  · CarMaker / dSPACE / Modelica (via FMI 2.0)                 │
│  · Chrono Vehicle (via FMI 2.0)                               │
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

## Validation status (sports.yaml @ L3)

| Test | Result |
|---|---|
| ISO 7401 step-steer (6°, 80 km/h) | U = 1.21 (20.6% overshoot), T_ψ̇ = 0.20 s |
| ISO 4138 understeer gradient | K = +9.69 mrad/g (UNDERSTEER, sports-typical) |
| ISO 3888-2 DLC @ 60 km/h | FAIL (excursion 1.3 m, speed loss 5.5 km/h) |
| ISO 3888-2 DLC @ 40 km/h | PASS (excursion 0.3 m, speed loss 1.0 km/h) |
| FMU export round-trip | max \|Δoutput\| = 0 (numerical precision) |
| ctest | **165 / 165 passing** |

## Status

| Phase | Milestone | Status |
|---|---|---|
| 0 | Skeleton + build sanity | ✅ |
| 1 | Core interfaces + bicycle | ✅ |
| 2 | CARLA integration | ✅ Python bridge + raycast |
| 3 | 7-DOF + raycast contact | ✅ |
| 4 | 14-DOF + ride dynamics | ✅ Sprung 3 + Unsprung 4 DOF |
| 5 | Control cascade L4-L8 | ✅ Pure pursuit, vx/ax PID, Driver |
| 6 | Pybind11 module | ✅ |
| 7 | Tire upgrades | ✅ Load sens + relaxation + camber Mz |
| 8 | Ld4 hardpoint framework | ✅ 4 suspension types + native solvers |
| 9 | DOE / parameter sweep | ✅ |
| 10 | ISO validation (7401/4138/3888-2) | ✅ |
| 11 | FMI 2.0 export (L2 + L3) | ✅ |
| 12 | FMI 2.0 import (generic master) | ✅ |
| 13 | SMPC / MPC controller | Phase 2 (HPIPM integration) |
| 14 | Ld5 compliance (bushings) | Phase 2 |

## License

- Core / kinematics / tools / validation / FMI export (this repo): Apache-2.0.
- FMI 2.0 headers (`fmi_export/fmi2/`): BSD-2-Clause (Modelica Association).
- Third-party (Eigen / yaml-cpp / spdlog / gtest): respective open-source licenses, vendored via FetchContent.
