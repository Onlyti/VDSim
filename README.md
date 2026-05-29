# VDSim

차량 설계와 자율주행 평가를 잇는 오픈코어 차량 동역학 시뮬레이션 플랫폼.

## Layout

| Directory | Contents |
|---|---|
| `core/` | `libvdsim_core` — C++17 standalone library. L1 Bicycle / L2 7-DOF / L3 14-DOF dynamics + Pacejka MF96 tire + L5-L8 control converters. ISO 8855 RH convention. |
| `python/` | pybind11 bindings (`vdsim` module) + CSV / .tir importers + sweep_runner. |
| `carla_integration/plugin/` | `RaycastContactProvider` — `IContactProvider` impl backed by an injectable raycast function (CARLA-ready). |
| `examples/` | CLI binaries: `vdsim_bicycle_run`, `vdsim_scenario_run`, `vdsim_l1_vs_l2`, `vdsim_ax_track_demo`, `vdsim_split_mu_demo`, `vdsim_l3_demo`, `vdsim_path_tracking`, `vdsim_driver_demo`. |
| `configs/` | `vehicles/` (sedan, sports, fsk_formula, race_car), `tires/`, `solvers/`, `scenarios/` (step_steer, double_lane_change, throttle_brake_sequence, j_turn, skidpad, brake_in_turn, ice_patch). |
| `third_party/` | FetchContent-managed Eigen / yaml-cpp / spdlog / gtest |
| `tests/` | `unit/`, `integration/`, `validation/{analytical,carmaker}/` |
| `scripts/` | Build / sync / deployment helpers |
| `docs/` | Specs and design notes |

## Build

```bash
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Debug \
    -DVDSIM_BUILD_PYTHON=ON \
    -DVDSIM_BUILD_CARLA_PLUGIN=ON
cmake --build build -j
(cd build && ctest --output-on-failure)
```

First configure pulls Eigen / yaml-cpp / spdlog / gtest via FetchContent (~5 min).

Requires: cmake ≥ 3.16, ninja, g++ ≥ 9 (or clang ≥ 8), git, python3 + pybind11 (`pip install pybind11`). C++17.

### Quick tour after build

```bash
# 1. Run a scenario on L1 bicycle
bin/vdsim_scenario_run configs/vehicles/sedan.yaml \
                       configs/tires/default_pacejka.yaml \
                       configs/scenarios/step_steer.yaml /tmp/out.csv

# 2. Closed-loop path tracking (L5-L8 cascade)
bin/vdsim_path_tracking configs/vehicles/sports.yaml \
                        configs/tires/default_pacejka.yaml /tmp/path.csv

# 3. Compare L1 vs L2 dynamics
bin/vdsim_l1_vs_l2 configs/vehicles/sedan.yaml \
                   configs/tires/default_pacejka.yaml \
                   configs/scenarios/double_lane_change.yaml /tmp/

# 4. Python bindings (in build/python/)
python3 -c "import vdsim; vp = vdsim.VehicleParams.from_yaml('configs/vehicles/sedan.yaml'); print(vp.mass)"

# 5. Import AVL .tir tire data
python3 python/tir_to_yaml.py my_tire.tir my_tire.yaml

# 6. Sweep parameter grid
python3 python/sweep_runner.py configs/sweeps/aero_vs_vx.yaml /tmp/sweep_out
```

## Conventions

- Units: SI (m, kg, s, rad, N, N·m). No cm / deg internally.
- Body frame: ISO 8855 RH — X forward, Y left, Z up.
- World frame: ENU RH.
- Quaternion: body → world (Eigen convention).
- Euler: ZYX intrinsic (yaw → pitch → roll).
- Wheel index: FL = 0, FR = 1, RL = 2, RR = 3.

## Status (PoC W1-W12)

| Phase | Milestone | Status |
|---|---|---|
| 0 | Skeleton + build sanity | done |
| 1 | Core interfaces + bicycle | done |
| 2 | CARLA integration ABI | skeleton + mock test (real UE5 = Phase 2) |
| 3 | 7-DOF + raycast contact | done (raycast injectable) |
| 4 | 14-DOF + validation | done (sprung 3 + unsprung 4 DOF) |
| 5 | Control cascade L4-L8 | done (Pure Pursuit, vx PID, ax PID, Driver model) |
| 6 | Pybind11 module | done |
| 7 | External data import (.tir, ADMA CSV) | done |
| 8 | CarMaker ERG validation | Phase 2 (license) |
| 9 | SMPC / MPC controller | Phase 2 (HPIPM integration) |

**140/140 tests pass.** See [`docs/`](docs/README.md) for the per-task report log
and [`docs/tasks/52_poc_summary_v2/README.md`](docs/tasks/52_poc_summary_v2/README.md)
for the W1-W12 progress summary (≈ 92 %).

## License

- Core (`core/`, `python/`, `tests/`): Apache-2.0.
- Integration / plugin (`carla_integration/`): separate commercial.
