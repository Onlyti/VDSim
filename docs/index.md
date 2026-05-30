# VDSim

**Open-core vehicle dynamics simulator** — bridging the autonomy stack
(throttle / pedal / path) and chassis design (hardpoint / multibody) in a
single C++17 + Python ABI.

[GitHub](https://github.com/Onlyti/VDSim){ .md-button .md-button--primary }
[PoC summary v3](tasks/68_poc_summary_v3/README.md){ .md-button }
[Theory](theory/README.md){ .md-button }

---

## Status

- **PoC W1-W12 progress**: ~ 95 %
- **Tests**: 144 / 144 passing
- **4 vehicles** (sedan / sports / FSK formula / race) × **8 scenarios**
  (step_steer / DLC / throttle_brake / ice_patch / j_turn / skidpad /
   brake_in_turn / ice_corner).
- **10 CLI demo binaries**, **Python bindings**, **CARLA-ready raycast ABI**,
  **3D Three.js viewer** (CSV replay + WebSocket realtime).

## Two ladders

VDSim 의 차별화 — single ABI 안에서 두 사다리 m × n 매트릭스.

### Dynamics ladder (fidelity)

| Tier | DOF | Status |
|---|---|---|
| Ld1-Bicycle | 5 | done |
| Ld2-SevenDOF | 7 | done |
| Ld3-FourteenDOF | 14 (sprung 3 + unsprung 4) | done |
| Ld4-MultibodyKinematic | hardpoint-driven | M0 stub (planned M1-M7) |
| Ld5-MultibodyCompliant | DAE constrained | planned |

### Control ladder (abstraction)

| Tier | Input | Status |
|---|---|---|
| Lc1-PerWheel | motor / brake torque per wheel | via lowering |
| Lc2-AxleTorque | axle drive / brake | via lowering |
| Lc3-FxTotal | longitudinal force | via lowering |
| **Lc4-Pedal** | **throttle / brake / steer** (CARLA-호환) | **primary** |
| Lc5-AxTarget | ax_target | done (PI + FF) |
| Lc6-VTarget | v_target | done (cascade PI) |
| Lc7-PathCurvature | v_target + κ (Pure Pursuit) | done |
| Lc8-Waypoint | path + lookahead | done (figure-8 demo) |

## Quick start

```bash
git clone https://github.com/Onlyti/VDSim
cd VDSim
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Debug \
      -DVDSIM_BUILD_PYTHON=ON -DVDSIM_BUILD_CARLA_PLUGIN=ON
cmake --build build -j

# Run a scenario
build/bin/vdsim_scenario_run \
    configs/vehicles/sports.yaml \
    configs/tires/default_pacejka.yaml \
    configs/scenarios/skidpad.yaml /tmp/out.csv

# Launch the 3D viewer
python3 -m http.server -d viewer 8080 &
python3 viewer/realtime_server.py --driver --level L2 --v_target 13 &
# → browse http://localhost:8080
```

## What to read

- **First time**: [Theory overview](theory/README.md) → chapters 01-04 sequentially.
- **Cherry-pick**: jump to [Ld3-FourteenDOF](theory/06_ld3_fourteen_dof.md)
  or [Pure Pursuit](theory/09_pure_pursuit_path.md).
- **Sales / PT**: [PoC summary v3](tasks/68_poc_summary_v3/README.md)
  and [Competitive matrix](tasks/69_competitive_matrix/README.md).
- **Implementation tour**: [Software architecture](theory/12_software_architecture.md).
- **Roadmap to multibody**: [Multibody outlook](theory/13_multibody_outlook.md).
