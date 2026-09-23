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

## Command & observation reference

What you command each step, and what the plant returns. The per-wheel block is the part
a kinematic model cannot give you. Axes and signs follow ISO 8855
([Frames & conventions](theory/01_frames_and_conventions.md)): x forward, y left,
z up, positive yaw counter-clockwise; angles in rad, everything else SI. Wheel arrays are
ordered FL, FR, RL, RR.

### Commands

Every command below is a `vdsim` object accepted by `sim.set_input(cmd)`.

| Command | Fields | Units | Notes |
|---|---|---|---|
| `CmdL1` | `motor_torque[4]`, `brake_torque[4]`, `steer_angle_wheel` | N·m, N·m (≥ 0), rad | per-wheel torque; the direct path of `VDSimPlant` |
| `CmdL3` | `Fx_total`, `steer_angle_wheel` | N (+ drive / − brake), rad | total longitudinal force |
| `CmdL4` | `throttle`, `brake`, `steer_angle_wheel`, `gear` | [0, 1], [0, 1], rad, +1 fwd / 0 N / −1 rev | the default; `set_input(steer=, throttle=, brake=, gear=)` builds one |
| `CmdL5` | `ax_target`, `steer_angle_wheel` | m/s², rad | longitudinal acceleration target |
| `CmdL6` | `v_target`, `steer_angle_wheel` | m/s, rad | speed target |
| `CmdL7` | `v_target`, `kappa` | m/s, 1/m | speed target + path curvature |
| `CmdSplit` | `lon` (`LcLonL4`–`LcLonL6`), `lat` (`LcLatL1`–`LcLatL7`) | per axis level | independent axes; see [Control ladder](#control-ladder-command-at-any-abstraction-level) |

`steer_angle_wheel` is the road-wheel angle, not the hand-wheel angle.

### Body state

| Field | Symbol | Units | Frame | Available from |
|---|---|---|---|---|
| `t` | $t$ | s | — | `sim.state()` |
| `x`, `y` | $x, y$ | m | earth-fixed | `sim.state()` |
| `yaw` | $\psi$ | rad | earth-fixed | `sim.state()` |
| `vx`, `vy` | $v_x, v_y$ | m/s | body | `sim.state()` |
| `r` | $r$ | rad/s | body z | `sim.state()` |
| `beta` | $\beta = \operatorname{atan2}(v_y, v_x)$ | rad | body | `Sim.state()` |
| `ax`, `ay` | $a_x, a_y$ | m/s² | body | `sim.state()`; `SimOutput.ax`, `.ay` |
| `roll`, `pitch` | $\phi, \theta$ | rad | body | `SimOutput`; 0 on Ld1, quasi-static on Ld2, state on Ld3 |
| `az` | $a_z$ | m/s² | body | `SimOutput`; Ld3 only (0 elsewhere) |
| `heave_z` | $z_s$ | m | about the settled ride height | `SimOutput`; Ld3 only (0 elsewhere) |

Position, velocity and acceleration refer to the centre of gravity. When `Sim` is built
with a reference point, `Sim.state()` transports them to that point (lever arm through
yaw rate and yaw acceleration); `SimOutput` always stays at the CG.

### Per-wheel state (FL, FR, RL, RR)

| Field | Symbol | Units | Frame | Available from |
|---|---|---|---|---|
| `slip_angle` | $\alpha$ | rad | wheel | `sim.state()`; `SimOutput` |
| `slip_ratio` | $\kappa$ | — | wheel | `sim.state()`; `SimOutput` |
| `Fz` | $F_z$ | N | normal load | `sim.state()`; `SimOutput` |
| `tire_forces` | $(F_x, F_y, F_z)$ | N | body | `SimOutput` |
| `tire_forces_wheel` | $(F_x, F_y, F_z)$ | N | contact / wheel | `SimOutput` |
| `wheel_spin` | $\omega$ | rad/s | wheel axis | `SimOutput.state.wheel_spin` |
| `wheel_mu` | $\mu$ | — | — | `SimOutput` |
| `wheel_mu_peak` | $\mu_\text{peak}$ | — | — | `SimOutput` |
| `wheel_alpha_peak` | $\alpha_\text{peak}$ | rad | wheel | `SimOutput` |
| `wheel_kappa_peak` | $\kappa_\text{peak}$ | — | wheel | `SimOutput` |

Friction utilisation is **derived**, not a plant output: `vdsim_trace.utilization()`
computes it from the per-wheel force, μ and the `mu_aniso` ellipse.

!!! note "Why the per-wheel block matters"
    Per-wheel slip angle, slip ratio and normal load, and the slip at which force
    peaks, are what make VDSim a control-research plant rather than a trajectory model.

### Where the same fields appear

- **Step output** — `sim.run_core_dt()` returns the `SimOutput` above; `sim.state()` is
  the dict view (`t, x, y, yaw, vx, vy, r, beta, ax, ay, Fz, slip_angle, slip_ratio`).
- **RL observations** — `EnvConfig.obs_fields` takes the names `x, y, z, yaw, roll,
  pitch, vx, vy, vz, speed, beta, yaw_rate, roll_rate, pitch_rate, ax, ay, sim_time,
  steer_applied, throttle_applied, brake_applied, rack_travel, rack_velocity` and the
  per-wheel `wheel_spin, slip_ratio, slip_angle, fz, susp_compression, susp_velocity,
  wheel_mu, tire_fx, tire_fy`; see the [RL guide](RL_GUIDE.md).
- **`.vdtrace` channels** (schema 0.4) — `t, pose, v_body, yaw_rate, u_steer, u_fx,
  wheel_F, wheel_mu, wheel_kappa, wheel_alpha`, plus `a_body` (Ld2 and up) and
  `pose_zrp, wheel_road_dz, wheel_road_normal, wheel_travel` (Ld3 and up).

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
