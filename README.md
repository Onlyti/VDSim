# VDSim

차량 설계와 자율주행 평가를 잇는 오픈코어 차량 동역학 시뮬레이션 플랫폼.

## Layout

| Directory | Contents |
|---|---|
| `core/` | `libvdsim_core` — C++17 standalone library. Bicycle / 7-DOF / 14-DOF dynamics + Pacejka MF tire. ISO 8855 RH convention |
| `python/` | pybind11 bindings (Phase 2) |
| `carla_integration/` | CARLA `UVDSimMovementComponent` + RPC patches (W7+) |
| `third_party/` | FetchContent-managed Eigen / yaml-cpp / spdlog / gtest |
| `tests/` | `unit/`, `integration/`, `validation/{analytical,carmaker}/` |
| `scripts/` | Build / sync / deployment helpers |
| `docs/` | Specs and design notes |

## Build

```bash
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
(cd build && ctest --output-on-failure)
```

First configure pulls Eigen / yaml-cpp / spdlog / gtest via FetchContent (~5 min).

Requires: cmake ≥ 3.16, ninja, g++ ≥ 9 (or clang ≥ 8), git. C++17.

## Conventions

- Units: SI (m, kg, s, rad, N, N·m). No cm / deg internally.
- Body frame: ISO 8855 RH — X forward, Y left, Z up.
- World frame: ENU RH.
- Quaternion: body → world (Eigen convention).
- Euler: ZYX intrinsic (yaw → pitch → roll).
- Wheel index: FL = 0, FR = 1, RL = 2, RR = 3.

## Targets

| Phase | Milestone | Weeks |
|---|---|---|
| 0 | Skeleton + build sanity | W1 |
| 1 | Core interfaces + bicycle | W2–W6 |
| 2 | CARLA integration RPC | W7–W8 |
| 3 | 7-DOF + raycast contact | W9–W10 |
| 4 | 14-DOF + validation | W11–W12 |

## License

- Core (`core/`, `python/`, `tests/`): Apache-2.0.
- Integration / plugin (`carla_integration/`): separate commercial.
