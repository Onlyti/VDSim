# VDSim

**English** · [한국어](README.ko.md)

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)

Open-core vehicle-dynamics simulator. VDSim owns the **vehicle** — validated
L1–L3 dynamics with a real Pacejka tire, hardpoint suspension kinematics you can
design against, and bidirectional FMI 2.0 — and delegates rendering/sensors to
tools like CARLA. It is the chassis-accurate half that perception stacks lack.

> v0.1.0 — experimental / pre-release. Validated on analytic + ISO + cross-model
> self-consistency (see [VALIDATION](docs/VALIDATION.md)); not for production use.

Docs (theory + reports): **https://onlyti.github.io/VDSim/** · all run modes
(API / batch / FMI): [docs/RUNNING.md](docs/RUNNING.md)

## Install

Prerequisites: a C++17 compiler, CMake ≥ 3.20, Python ≥ 3.10.
- Linux: `g++ ≥ 9` or `clang ≥ 10`.
- Windows: Visual Studio 2019+ with "Desktop development with C++" (MSVC).

Python package (Linux + Windows):
```bash
pip install ".[plot]"
python -c "import vdsim; print('ok')"
```

Full C++ tree (tests, real-time runtime, FMI, CARLA bridge):
```bash
cmake -B build -DVDSIM_BUILD_PYTHON=ON          # add -G Ninja on Linux
cmake --build build --config Release            # -j on Linux
ctest --test-dir build -C Release               # 191/191 ; binaries in build/bin/
```

A quick experiment in Python:
```python
from vdsim_lab import Experiment, Vehicle, Road, Maneuver, Sensors
res = (Experiment(level="L3").vehicle(Vehicle.preset("sedan"))
       .road(Road.preset("belgian_pave")).maneuver(Maneuver.step_steer(v=20, steer=0.03))
       .sensors(Sensors().gnss().imu()).run(8.0))
res.to_csv("run.csv"); print(res.summary())
```

## Visualization — Web GUI

```bash
python3 gui/server.py --port 8100        # Windows: python gui\server.py --port 8100
```
Open `http://localhost:8100`. The GUI auto-starts the real-time runtime and
renders it: 3D view (orbit / chase / cockpit), road & terrain, per-wheel Fz /
slip (κ, α) / tire-force vectors, and a telemetry HUD. Drive with the keyboard
(↑↓ throttle/brake, ←→ steer) or a gamepad-wheel — force feedback via
`python tools/wheel_ffb_sdl.py --server <host> --udp-port 8101`.

## Control — real-time UDP runtime

`vdsim_realtime` is VDSim's real-time application: it runs the same core against
a wall clock and exchanges fixed-format binary UDP packets — **CMD**
(steer / throttle / brake) in, **STATE** (pose, velocities, Fz, slip, tire
forces, measured sensors, …) out. This is the SIL / HIL / co-sim boundary; the
GUI and any external controller are just clients of it.

```bash
build/bin/vdsim_realtime --scene=configs/scenes/two_vehicle_race.yaml \
    --cmd-port=7001 --state-port=7002 --rate=200
```
The wire format is the canonical VDS1 binary protocol (`cosim/cosim_protocol.hpp`;
contract in [docs/vdsim_bridge_interface_requirements.md](docs/vdsim_bridge_interface_requirements.md)).
Python clients encode CMD / decode STATE through `cosim/protocol.py` to stay
byte-compatible.

## Conventions

SI units · body frame ISO 8855 RH (X forward, Y left, Z up) · world frame ENU ·
wheel index **FL = 0, FR = 1, RL = 2, RR = 3**.

## License

Apache-2.0 (core / kinematics / tools / validation / FMI export). FMI 2.0 headers
are BSD-2-Clause (Modelica Association). Third-party deps (Eigen / yaml-cpp /
spdlog / GoogleTest) keep their own licenses; fetched via CMake FetchContent.
