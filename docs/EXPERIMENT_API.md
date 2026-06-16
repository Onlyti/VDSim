# Experiment API — drive the VDSim core from Python

For algorithm developers (control / estimation / planning). VDSim gives you the
**simulation seam**; you write the controller and own the loop. No scenario file,
no network — just the core stepped from your code.

- Harness: `python/vdsim_lab.py` → `Sim`
- Template to copy: `templates/experiment_template.py`
- Runnable examples: `examples/experiment_quickstart.py`,
  `examples/experiment_path_follow.py`
- Batch / sweeps over many runs: `tools/vdsim_batch.py` (see docs/CONFIG_GUIDE.md §1)
- Real-time / external controller over UDP instead of in-process: docs/CONFIG_GUIDE.md §3

## The seam (4 calls)

| call | meaning |
|---|---|
| `sim.state()` | ground-truth dict: `t, x, y, yaw, vx, vy, r, ax, ay, Fz[4], slip_angle[4], slip_ratio[4]` |
| `sim.measurements(id)` | noisy sensor readout; transported to the mount pose if registered (CG bundle if `id` omitted) |
| `sim.set_input(steer=, throttle=, brake=, gear=)` | inject the action (steer [rad] at the wheel, throttle/brake [0..1]); also accepts a `vdsim.CmdL4` |
| `sim.run_core_dt(dt=None)` | advance one core step (default `dt`), records a log row, returns the `SimOutput` |

This is exactly the path the real-time server and batch runner use internally:
`set_input → tick`. In real-time mode the action arrives over UDP; here it comes
from your function. Same core, same seam.

## Minimal loop

```python
import sys; from pathlib import Path
REPO = Path.cwd()
sys.path[:0] = [str(REPO / "python"), str(REPO / "build" / "python")]
from vdsim_lab import Sim, Road, Sensors

sim = Sim(vehicle="sedan", tire="default_pacejka", level="L2",
          road=Road.iso8608("C"), sensors=Sensors().gnss(pos_std=0.3).imu(),
          v0=12.0,
          sensor_mounts={"gnss": {"type": "gnss", "pos": [1.4, 0, 1.0]}})

while not sim.done(12.0):
    st = sim.state()
    gnss = sim.measurements("gnss")
    steer, throttle, brake = my_controller(st, gnss)   # YOUR algorithm
    sim.set_input(steer=steer, throttle=throttle, brake=brake)
    sim.run_core_dt()

sim.to_csv("run.csv")                                  # ground-truth + per-wheel log
sim.metrics(["peak_ay", "cte_rms", "lap_time"])        # scalar reductions
sim.plot("run.png", signals=("vx", "ay", "r", "xy"))   # optional (needs matplotlib)
```

Run the template directly:

```sh
PYTHONPATH=build/python:python python3 templates/experiment_template.py
```

## Control ladder — command at any abstraction level

`set_input` accepts more than pedals. The CascadeController converts any ladder
level to the realized pedal/steer each tick, using measured-state feedback. So
you can hand the sim a high-level intent and let it close the inner loops.

| level | command | longitudinal | lateral |
|---|---|---|---|
| L4 | `CmdL4` | throttle/brake | steer angle [rad] |
| L5 | `CmdL5` | `ax_target` [m/s²] | (steer angle) |
| L6 | `CmdL6` | `v_target` [m/s] (cruise) | (steer angle) |
| L7 | `CmdL7` | `v_target` | `kappa` [1/m] curvature |
| split | `CmdSplit` | any `LcLon*` | any `LcLat*` (independent) |

```python
import vdsim
sim.set_input(vdsim.CmdL6(v_target=20.0))           # cruise control (lon L6)

c = vdsim.CmdSplit()                                  # independent axes
c.lon = vdsim.LcLonL6(); c.lon.vx_target = 18.0       # speed control
c.lat = vdsim.LcLatL6(); c.lat.r_target  = 0.15       # yaw-rate control
sim.set_input(c)
```

Lateral levels: `LcLatL4` angle · `LcLatL5` `ay_target` · `LcLatL6` `r_target`
(yaw rate) · `LcLatL7` `kappa`. Levels below L4 (steer torque / rate) are the
steering subsystem's territory (Dynamic mode), not the cascade.

Verify all levels end-to-end:
```sh
PYTHONPATH=build/python:python python3 examples/control_ladder_demo.py
```

## Building the plant — `Sim(...)`

| arg | values |
|---|---|
| `vehicle` | preset name (`"sedan"`), a `*.yaml` path, or a `Vehicle(...)` |
| `tire` | preset name (`"default_pacejka"`), a `*.yaml` path, or a `Tire(...)` |
| `level` | `L1` bicycle · `L2` 7DOF · `L3`/`L4` 14DOF · `L5` stunt |
| `road` | `Road.flat(mu)` · `.inclined(grade, bank, mu)` · `.split_mu(...)` · `.iso8608("C")` · `.preset("belgian_pave")` |
| `sensors` | `Sensors().gnss(...).imu().wheel_speed().steer()` (or a raw `vdsim.SensorParams`) |
| `dt` | core step [s] (default 0.005) |
| `x0,y0,yaw0,v0` | initial pose + speed |
| `sensor_mounts` | `{id: {type, pos:[x,y,z], yaw:deg}}` → `measurements(id)` reports at the mount |

## Evidence (figures are optional)

- `sim.to_csv(path)` — always available, no deps.
- `sim.metrics(names, line=None)` — `peak_ay, cte_rms, cte_max, vmax, dist,
  lap_time, max_Fz, rms_slip` (CTE/lap need a reference `line`).
- `sim.plot(path, signals=...)` / `vdsim_lab.plot_result(res, ...)` — basic PNG
  (time series + `"xy"` trajectory). Needs `matplotlib`; raises a clear error if
  missing, so CSV-only workflows are unaffected. Labels are English.
