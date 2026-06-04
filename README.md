# VDSim

**English** · [한국어](README.ko.md)

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)

An open-core vehicle-dynamics simulation platform bridging chassis design and
autonomous-driving evaluation.

**Why not just CARLA?** CARLA gives you a world and sensors but a game-engine
vehicle. VDSim gives you the vehicle: validated L1–L3 dynamics with a real
Pacejka tire, hardpoint suspension kinematics you can design against, and
bidirectional FMI 2.0 so it drops into a co-simulation — while delegating
rendering/sensors to CARLA. It is the chassis-accurate half that perception
stacks lack.

> **Status:** v0.1.0, experimental / pre-release. Validated on analytic + ISO
> standard + cross-model/cross-tool self-consistency evidence
> (see [VALIDATION](docs/VALIDATION.md)); not yet cross-validated against a
> commercial reference on real-vehicle data. Not for production use.

📖 **Documentation (theory + reports):** https://onlyti.github.io/VDSim/
🏃 **How to run every mode (API / rt-comms / batch / GUI / FMI):** [docs/RUNNING.md](docs/RUNNING.md)

> *Positioning*: external visualization / sensors are delegated (CARLA, etc.);
> VDSim owns **accurate, validated vehicle dynamics** + **hardpoint-based design
> validation** + **bidirectional FMI 2.0 integration**.

## Layout

| Directory | Contents |
|---|---|
| `core/` | `libvdsim_core` — C++17 standalone library. Ld1 Bicycle / Ld2 7-DOF / Ld3 14-DOF dynamics + Pacejka MF96 tire (with load sensitivity + relaxation length + camber thrust/Mz) + Lc5-Lc8 control converters. Ld4 hardpoint kinematics (DW/MP/TA/5-link, lookup + native solvers). ISO 8855 RH. |
| `python/` | pybind11 bindings (`vdsim` module): VehicleParams / TireParams / SolverParams, ITireModel, ISuspensionKinematics + attach helpers, all dynamics + Lc controllers. |
| `tools/kinematics/` | Offline hardpoint solvers (DW 2D/3D, MacPherson, trailing arm, 5-link), diagnostic + Adams CSV importer + matplotlib GUI. |
| `gui/` | Three.js real-time web viewer (PoC) — subscribes to the live sim and renders 3D view / road / telemetry. |
| `builder/` | Experiment authoring web tool (vehicle / sensor / map / comms / scenario) · web-based suspension editor with live kinematic curves. |
| `carla_integration/` | Python bridge — drives a CARLA actor with VDSim dynamics; raycast contacts; supports Ld4 kinematics attach. |
| `apps/jump_demo/` | T23/T24 — 2D + 3D turning-jump simulators (Phase-2 14-DOF prototype with world-z + airborne + Pacejka). |
| `apps/doe/` | Design-of-Experiments runner — multi-parameter × multi-scenario sweeps → CSV + heatmaps. |
| `apps/validation/` | ISO 7401 (step steer), ISO 4138 (steady-state circular), ISO 3888-2 (double lane change) — automated metric extraction + report. |
| `fmi_export/` | FMI 2.0 Co-Simulation export (L2 + L3 FMUs) and import (`fmu_master.py` — load any FMI 2.0 CS FMU via ctypes). |
| `configs/` | `vehicles/`, `tires/`, `suspensions/` (DW/MP/TA/5-link YAML), `scenarios/`. |
| `tests/` | `unit/` + `integration/` (187 tests, 100% green). |

## Install (Python)

Prerequisites: a C++17 compiler + CMake ≥ 3.20, and **Python ≥ 3.10 with a
modern `pip`** (on Ubuntu 20.04's stock 3.8 you must `apt install python3-venv`
and `pip install -U pip` first — older pip can't drive the scikit-build-core
backend).

```bash
pip install ".[plot]"         # builds the vdsim extension (scikit-build-core); [plot] adds matplotlib
python -c "import vdsim; print('ok')"
```

Quickstart, measured end-to-end (clean venv → `pip install` → run a scenario →
CSV → trajectory plot): **first result in ~16 s** on a warm dev machine (cold
first build compiles the core, still well under a minute).
`import vdsim` then gives the full API (`make_sim_session`, `VehicleParams`,
`create_*_ground`, `make_sim_session_psd`, hardpoint kinematics, `linearize`, …).

For experiments, `python/vdsim_lab.py` adds fluent builders — assemble a run from
Vehicle / Road / Maneuver / Sensors and get a time-series with per-wheel
ground truth (Fz, slip α/κ, tire Fx/Fy):

```python
from vdsim_lab import Experiment, Vehicle, Road, Maneuver, Sensors
res = (Experiment(level="L3")
       .vehicle(Vehicle.preset("sedan"))
       .road(Road.preset("belgian_pave"))            # or .iso8608("C"), .inclined(...)
       .maneuver(Maneuver.step_steer(v=20, steer=0.03))  # or .path(line, v=15), .constant_speed(...)
       .sensors(Sensors().gnss().imu())
       .run(duration=8.0))
res.to_csv("run.csv");  print(res.summary())
```

## Build (full tree: C++ tests, examples, CARLA, co-sim)

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
python3 builder/suspension_editor_server.py &
( cd builder && python3 -m http.server 8090 )
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
What "validated dynamics" means, the reproducible benchmark matrix (analytic /
ISO / cross-model / FMI / ISO 8608) and the honest limits: [docs/VALIDATION.md](docs/VALIDATION.md).

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

## Documentation

Full theory reference + PoC reports are published with MkDocs Material
(MathJax equations, search, dark mode) at **https://onlyti.github.io/VDSim/**.

Theory chapters (each pairs equations with the `file:line` of the
implementation and a verification test):

| # | Chapter | # | Chapter |
|---|---|---|---|
| 01 | Frames & conventions | 09 | Pure pursuit / path |
| 02 | Rigid-body dynamics | 10 | Driver model |
| 03 | Tire (Pacejka MF96) | 11 | Numerical integration |
| 04 | Ld1-Bicycle | 12 | Software architecture |
| 05 | Ld2-SevenDOF | 13 | Multibody (Ld4) overview |
| 06 | Ld3-FourteenDOF | 14 | Hardpoint kinematics |
| 07 | Control ladder Lc1-Lc8 | 15 | Validation & DOE |
| 08 | PID controllers | 16 | FMI 2.0 integration |

Build the docs locally:
```bash
pip install mkdocs-material pymdown-extensions mike
mkdocs serve        # → http://localhost:8000/VDSim/
```

## License

- Core / kinematics / tools / validation / FMI export (this repo): Apache-2.0.
- FMI 2.0 headers (`fmi_export/fmi2/`): BSD-2-Clause (Modelica Association).
- Third-party (Eigen / yaml-cpp / spdlog / gtest): respective open-source licenses, vendored via FetchContent.
