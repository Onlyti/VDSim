# VDSim

**English** · [한국어](README.ko.md)

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)

Open-core vehicle-dynamics simulator. VDSim owns the **vehicle** — validated
L1–L5 dynamics with real Pacejka MF / LuGre / belt-transient tires, hardpoint
suspension kinematics you can design against, and bidirectional FMI 2.0 — and
delegates rendering/sensors to tools like CARLA. It is the chassis-accurate half
that perception stacks lack.

> v0.5.1+ — experimental / pre-release. Validated on analytic + ISO + cross-model
> (Chrono Pac02) self-consistency (see [VALIDATION](docs/VALIDATION.md)); not for production use.

Docs (theory + reports): **https://onlyti.github.io/VDSim/** · all run modes
(API / batch / FMI): [docs/RUNNING.md](docs/RUNNING.md) · catalog & physics options:
[docs/CATALOG_AND_PHYSICS.md](docs/CATALOG_AND_PHYSICS.md)

## Quickstart (pip wheel)

Prerequisites: Python 3.10–3.12. Install a pre-built wheel (or build locally — see
[from source](#from-source) below).

```bash
pip install "./vdsim-*.whl[plot]"
vdsim-quickstart          # writes run.csv + run.png in the current directory
```

Measured on a clean conda env (Python 3.11, Linux x86_64, 2026-06-25): **pip install
`[plot]` + `vdsim-quickstart` → `run.csv` + `run.png` in ~6 s wall-clock** on a
typical lab network (matplotlib wheel download included). Repeat runs ~1.5 s.

Verify the install:

```bash
python -c "import vdsim; from vdsim_lab import Sim; Sim(vehicle='sedan', level='L2')"
```

Source for the script: [`examples/quickstart.py`](examples/quickstart.py) — uses only
`from vdsim_lab import Sim, Road` (no repo paths).

## From source

## From source

Prerequisites: a C++17 compiler, CMake ≥ 3.20, Python ≥ 3.10.
- Linux: `g++ ≥ 9` or `clang ≥ 10`.
- Windows: Visual Studio 2019+ with "Desktop development with C++" (MSVC).

Python package (editable / local wheel build):
```bash
pip install ".[plot]"
python -c "import vdsim; print('ok')"
```

Full C++ tree (tests, real-time runtime, FMI, CARLA bridge):
```bash
cmake -B build -DVDSIM_BUILD_PYTHON=ON          # add -G Ninja on Linux
cmake --build build --config Release            # -j on Linux
ctest --test-dir build -C Release               # 328/328 ; binaries in build/bin/
```

## Run an experiment in Python (write your own controller)

VDSim gives you the **simulation seam**; you own the loop and the algorithm. Build
the plant straight from the core (vehicle / tire / level / road) — no scenario file,
no network — then drive it: `set_input(action) → run_core_dt()`, read
`state()` / `measurements(id)`.

```python
from vdsim_lab import Sim, Road, Sensors

sim = Sim(vehicle="sedan", level="L2", road=Road.iso8608("C"),
          sensors=Sensors().gnss(pos_std=0.3).imu(), v0=12.0,
          sensor_mounts={"gnss": {"type": "gnss", "pos": [1.4, 0, 1.0]}})

while not sim.done(12.0):
    st = sim.state()                                  # ground truth
    gnss = sim.measurements("gnss")                   # noisy, at the mount
    steer, throttle, brake = my_controller(st, gnss)  # <-- YOUR algorithm
    sim.set_input(steer=steer, throttle=throttle, brake=brake)
    sim.run_core_dt()                                 # advance one core step

sim.to_csv("run.csv")                                 # ground-truth + per-wheel log
sim.metrics(["peak_ay", "cte_rms", "lap_time"])       # scalar reductions
sim.plot("run.png", signals=("vx", "ay", "r", "xy"))  # optional (needs matplotlib)
```

Copy `templates/experiment_template.py`, replace `controller(...)`, and run from a
cloned tree:
```bash
PYTHONPATH=build/python:python python3 templates/experiment_template.py
```
Runnable examples (require clone + build): `examples/experiment_quickstart.py`,
`examples/experiment_path_follow.py` (pure-pursuit + CTE). Pip users: use
[`examples/quickstart.py`](examples/quickstart.py) / `vdsim-quickstart` instead. Full reference (the
seam, `Sim(...)` options, evidence): **[docs/EXPERIMENT_API.md](docs/EXPERIMENT_API.md)**.
This is the same `set_input → tick` seam the real-time server and batch runner use —
in real-time mode the action arrives over UDP, here it comes from your function.

## Visualization — Web GUI

```bash
python3 gui/server.py --port 8100        # Windows: python gui\server.py --port 8100
```
Open `http://localhost:8100`. The GUI auto-starts the real-time runtime and
renders it: 3D view (orbit / chase / cockpit), road & terrain, per-wheel Fz /
slip (κ, α) / tire-force vectors, and a telemetry HUD. Drive with the keyboard
(↑↓ throttle/brake, ←→ steer) or a gamepad-wheel — force feedback via
`python tools/wheel_ffb_sdl.py --server <host> --udp-port 8101`.

## Config — parts catalog & scenes (v0.3)

Vehicles are **blueprints** over `configs/parts/` (chassis, tire, drivetrain, …).
Runs are **scenes** (`configs/scenes/*.yaml`) with a `fleet[]` of blueprint + part
overrides. See [CATALOG_AND_PHYSICS](docs/CATALOG_AND_PHYSICS.md) for layout,
GUI API, drivetrain inertia, and opt-in LuGre tire.

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
