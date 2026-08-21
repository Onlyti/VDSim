# VDSim

**English** · [한국어](README.ko.md)

[![build](https://github.com/Onlyti/VDSim/actions/workflows/build.yml/badge.svg)](https://github.com/Onlyti/VDSim/actions/workflows/build.yml)
[![license](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![status](https://img.shields.io/badge/status-experimental%20pre--release-orange.svg)
![tests](https://img.shields.io/badge/ctest-404%2F404-green.svg)

> **Experimental / pre-release — not for production.** Evidence and limits:
> [VALIDATION.md](docs/VALIDATION.md) (v0.5.1+).
> **Validated:** analytic + ISO maneuvers + L1↔L3 self-consistency + pure-slip same-`.tir`
> cross-check (CarMaker &lt;0.1%, Chrono Pac02 ~0.8%) — not MF-Tyre product parity.
> **NOT yet:** full-vehicle commercial cross-val, real-vehicle data, production sign-off.

![Grip-loss demo](docs/assets/demo_grip_loss.gif)

*Deterministic VDSim plant: controller hits an unseen low-μ patch, tyres saturate past
peak (drift>1) — real grip loss, not a soft-clamp.* Reproduce:
`python examples/demo_grip_loss.py` (`pip install vdsim[plot]` or local wheel).

Open-core vehicle-dynamics simulator. VDSim owns the **vehicle** — validated
L1–L5 dynamics with real Pacejka MF / LuGre / belt-transient tires, hardpoint
suspension kinematics you can design against, and bidirectional FMI 2.0 — and
delegates rendering/sensors to tools like CARLA. It is the chassis-accurate half
that perception stacks lack.

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

Measured on a clean conda env (Python 3.11, Linux x86_64, 2026-06-25): **first result in
seconds** — pip install `[plot]` + `vdsim-quickstart` → `run.csv` + `run.png` in **~6 s
wall-clock** (cold; lab network, matplotlib wheel included). Repeat quickstart ~1.5 s.

Verify the install:

```bash
python -c "import vdsim; from vdsim_lab import Sim; Sim(vehicle='sedan', level='L2')"
```

Source for the script: [`examples/quickstart.py`](examples/quickstart.py) — uses only
`from vdsim_lab import Sim, Road` (no repo paths).

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

## Visualization — headless trace render

A run can be recorded to a single `.vdtrace` file and rendered later, with no GUI,
no node/npm and no browser. Recording is opt-in and off unless enabled:

```python
plant.enable_trace("run.vdtrace", seed=0, run_id="demo")   # off unless called
...                                                        # step() as usual
path = plant.finalize_trace()
```

- `enable_trace(path, decimation=None, seed=None, run_id=None, producer=None, tags=None)` —
  one sample is offered per `step()`, taken *before* the step is integrated, so the
  pose is the state at `t` and `u_steer` / `u_fx` are the command held over
  `[t, t+control_dt)`. Returns the resolved decimation.
- `decimation=None` picks the smallest N whose record rate stays at or above 100 Hz
  (1 kHz control → N=10; a 20 Hz loop stays at N=1 and loses nothing).
- `finalize_trace()` flushes channels, freezes the manifest and closes the file;
  it returns the written path (`None` when recording was never enabled).
- The container is a zip: `manifest.json` + `channels/*.f64` + `overlays/*.json`.
  That one file is enough to render — no re-simulation, no results file.

Scenario knowledge is attached after the run as **overlays**. VDSim validates the
`kind` / `name` envelope and stores the object without interpreting it, so a newer
producer can write overlays that today's renderer ignores:

```python
import vdsim_trace
vdsim_trace.attach_overlay(path, {"kind": "path2d", "name": "intended_path",
                                  "xy": [[0.0, 0.0], [4.0, 0.0]]})
vdsim_trace.attach_overlay(path, {"kind": "region", "name": "mu_patch", "mu": 0.35,
                                  "polygon": [[40, -30], [60, -30], [60, 30], [40, 30]]})
```

The renderer takes one trace and writes a GIF plus a preview PNG (matplotlib +
pillow, from the `[plot]` extra):

```bash
vdsim-render run.vdtrace --out run.gif --fps 20     # installed wheel
PYTHONPATH=python python3 -m vdsim_render run.vdtrace --out run.gif   # cloned tree
```

Options: `--png` preview path · `--stride` frame stride · `--fps` · `--dpi` ·
`--title` · `--mp4` (only when `imageio-ffmpeg` is installed). Controller
horizons add `--sidecar` / `--view-half` / `--preview-frame` (below).

The bird's-eye frame shows the reference path (dashed, from a `path2d` overlay;
omitted when absent), the driven path, a body rectangle sized from manifest
`geometry`, front wheels turned by the recorded steering command, a velocity
arrow, per-wheel friction-utilization colour, a HUD in fixed screen coordinates,
and the `u_steer` / `u_fx` time series with a current-time cursor. Axis limits
come from `geometry`, never from autoscale, so the scale never drifts between
frames.

End to end — record, attach overlays, render:

```bash
PYTHONPATH=build/python:python python3 examples/demo_grip_loss.py \
    --out demo.gif --keep-trace
```

### Overlaying several runs

Pass two or more traces and the renderer draws them in one view — one camera, one
clock, a colour per run and the driven path in that same colour:

```bash
vdsim-render base.vdtrace tuned.vdtrace wet.vdtrace --out compare.gif \
    --labels "base,tuned,wet" --alpha 0.5
```

- Runs are aligned by **time**, not by sample index: every run is interpolated at
  the frame time, so traces recorded at different `dt`, or of different length,
  still overlay correctly. Yaw is unwrapped first, so a heading near ±π does not
  swing the long way round.
- A run whose trace ends early holds its final pose, fades to 35 % alpha, is
  marked `(ended)` in the HUD, and stops extending its path.
- `--alpha` (body fill, default 0.55) and `--path-alpha` (default 0.9) set how
  much of an overlapped vehicle shows through. `--colors` overrides the palette,
  `--labels` the legend (default: manifest `run_id`, else the file stem).
- Camera: `--follow fit` (default) is a static square window holding every route;
  `--follow 1` tracks that run with the usual geometry-derived window.
- `--speed` sets playback rate relative to wall-clock (`--stride` is single-run
  only). Each run's whole route is drawn faintly underneath; `--no-ghost` removes it.
- Body colour is spent on run identity, so grip moves to the wheels: a tyre
  outline turns red once its utilization crosses 0.8. The right-hand panels
  overlay every run's `u_steer` / `u_fx` and max utilization on one shared time axis.
- The preview PNG is taken at the frame of **maximum divergence** between runs,
  not at an arbitrary time.

End to end — the same manoeuvre at three friction levels, recorded to three
traces and overlaid from the files alone:

```bash
PYTHONPATH=build/python:python python3 examples/demo_compare_runs.py --out compare.gif
```

### Controller horizons — waypoint / target / prediction

Three curves get routinely conflated under the word "reference". They are
different objects, and the renderer keeps them apart by name — in the code, in
the sidecar keys and in the legend:

- **waypoint** — the global route the run has to follow. Time-invariant, one per
  scenario, carried by a `path2d` overlay and drawn once as a dashed grey line.
  The overlay `name` may be `waypoint`, `reference_path` or `ref_path`; the
  first is preferred, the other two stay accepted for older traces.
- **target (MPC input)** — the reference horizon handed to the controller at
  *this* step (N+1 points). Time-varying: the point spacing follows the planned
  speed, so it compresses under braking. Blue with markers, so the spacing is
  readable.
- **prediction (MPC output)** — the horizon the controller solved for at this
  step (N+1 points). Time-varying; on a failed solve it is whatever the solver
  returned. Orange, switching to thick red while the solver status is non-zero.

`target` and `prediction` are `(K, N+1, 2)` arrays and the trace container's
channel table is a whitelist of scalar / fixed-width rows, so they travel in a
**sidecar** `.npz` beside the trace rather than inside the container:

```
run.vdtrace          # the trace, renderable on its own
run.qp.npz           # optional horizons sidecar, picked up automatically
```

| key | shape | meaning |
| --- | --- | --- |
| `t` | (K,) | step time [s] — matched to trace samples by nearest time, not by index |
| `tgt_XY` | (K, N+1, 2) | target horizon, world frame [m] |
| `pred_XY` | (K, N+1, 2) | prediction horizon, world frame [m] |
| `status` | (K,) | solver status, `0` = success |
| `solve_ms` | (K,) | solve time [ms] — optional, shown in the HUD |
| `tgt_v` | (K, N+1) | planned speed along the target [m/s] — optional |
| `ego_XY` | (K, 2) | rear-axle position per step — optional, used to fit the view |

Only the first four keys are required, and the sidecar as a whole is optional:
without one the renderer behaves exactly as it did before, and the two horizon
artists are never created.

- Write the horizons in the **world** frame. A controller solving in an ego or
  Frenet frame has to undo that transform first. The check is that
  `pred_XY[k, 0]` lands within ~0.1 m of the vehicle's rear-axle position at
  step `k` — if it does not, the frame is wrong.
- Record a failed solve as it happened: keep that step's `pred_XY`, NaN
  included, instead of holding the previous one. The renderer skips non-finite
  points; overwriting them hides the failure the video is meant to show.

```bash
vdsim-render run.vdtrace --out run.gif --view-half 100 --preview-frame first-fail
```

- `--sidecar` names the `.npz` explicitly. The default is the trace path with
  its suffix swapped for `.qp.npz` (`run.vdtrace` -> `run.qp.npz`) when that
  file exists, and no sidecar otherwise; passing the flag turns a missing file
  into an error instead of a silent skip.
- `--view-half` sets the bird's-eye half-window [m]. The default window is
  derived from the wheelbase (~6 wheelbases) and is far shorter than an MPC
  horizon, so a 100 m horizon leaves the frame unless the window is widened.
- `--preview-frame util-peak` (default, unchanged) puts the preview PNG at the
  friction-utilization peak; `first-fail` puts it at the first non-zero solver
  status, printing a note and falling back to the peak when the run never
  failed.
- A run containing any failed solve also gets `<out>_firstfail.png` written next
  to the GIF, whatever `--preview-frame` says, and the BEV carries a
  `QP FAIL (status=N)` banner for as long as the status is non-zero.
- Failure times are marked on the command panel as vertical spans, merged over
  consecutive failing steps. They come from an `event` overlay named `qp_fail`
  when the producer attached one, and from the sidecar `status` array otherwise.

All three flags are single-run only — overlay mode ignores them.

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
