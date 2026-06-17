# vdsim_plant — VLA thesis closed-loop plant

> **Full step-by-step tutorial:** [`docs/VDSIM_PLANT_TUTORIAL.md`](../docs/VDSIM_PLANT_TUTORIAL.md)
> (build → import → quickstart → obs schema → low-μ patches → closed-loop → YAML params →
> API → troubleshooting). This file is the terse quick-reference.

Supported Python API for swapping VDSim in place of a dynamic-bicycle plant
(`from vdsim_plant import VDSimPlant`). MPC stays unchanged; only the plant is
replaced.

## Install / import

From the VDSim repo (after `cmake --build build -j`):

```python
import sys
sys.path.insert(0, "/path/to/VDSim/build/python")
sys.path.insert(0, "/path/to/VDSim/python")
from vdsim_plant import VDSimPlant
```

Or `pip install` the wheel (includes `vdsim` + configs under `vdsim_configs`).

## Quickstart

```bash
PYTHONPATH=build/python:python python3 examples/vla_plant_demo.py
```

## API contract (pinned)

```python
plant = VDSimPlant(
    config="ioniq5_awd.yaml",   # preset under configs/vehicles/
    friction_map=[(x0, x1, mu), ...],  # optional x-interval patches
    base_mu=0.9,
    control_dt=0.05,            # MPC sample (ZOH); must be >= substep_dt
    substep_dt=5e-4,            # internal integrator; must divide control_dt
)
plant.reset(state0=[X, Y, psi, vx, vy, r])   # wheel_spin = vx/R internally
obs = plant.step([delta_roadwheel_rad, Fx_total_N])
```

### Observation dict (every step, true state, ISO 8855)

| Key | Unit | Frame / note |
|-----|------|----------------|
| `X`, `Y` | m | world |
| `psi` | rad | yaw (+ left) |
| `vx`, `vy` | m/s | body |
| `r` | rad/s | yaw rate |
| `ax`, `ay` | m/s² | body estimated |
| `beta` | rad | sideslip |
| `wheel[i]` | — | i=0 FL, 1 FR, 2 RL, 3 RR |
| `wheel[i].Fx`, `Fy` | N | **contact / wheel frame** |
| `wheel[i].Fz` | N | normal load |
| `wheel[i].alpha`, `kappa` | rad, - | slip |
| `wheel[i].mu` | - | road-surface (contact) μ |
| `wheel[i].mu_peak` | - | realized tyre peak coeff at this Fz; friction saturation ratio `sat=‖F‖/(mu_peak·Fz)≤1` (not a monotone utilization — pair with slip; see tutorial §4) |

No usage / violation metrics in the plant (thesis-side only).

## Preset: `ioniq5_awd`

`configs/vehicles/ioniq5_awd.yaml` + `configs/parts/tire/ioniq5_pac2002.yaml` (+ `.tir`)  
m=2359 kg, Iz=3400, lf=1.17 m, lr=1.80 m, μ_nom=0.9, AWD 50/50
(`drive_split_front=0.5`). MF2002 `.tir` with load-dependent coefficients: a single tyre
recovers axle `Ca_f≈218k / Ca_r≈159k` from each axle's static Fz (`PKY1=-24.6, PKY2=2.56`),
cornering stiffness concave in load, peak μ falls with load (`PDY2=-0.10`).

## Physics path

- **Ld2 7DOF** (`create_seven_dof`), LuGre OFF, MF2002 `.tir` combined slip (load-dependent).
- **Fx_total → per-wheel torque** (CmdL1), bypassing throttle / drivetrain map.
- **Friction patches** via `create_friction_patch_ground` (per-wheel μ by contact x).
- **Plant path** bypasses vehicle kinematic low-speed blend (tyre VLOW only).
- **Deterministic** fixed `substep_dt`, no RNG.

## Acceptance (run)

```bash
cd build && ctest -R vla_plant --output-on-failure
```

1. Dry μ=0.9 lane change — qualitative yaw response.
2. **Headline:** brake+turn on μ=0.5 patch — friction saturation + slip + departure.
3. GT: ‖Fx,Fy‖ ≤ μ·Fz; ΣF ≈ m·a; bit-identical repeat.

Typical 5 s trajectory ≪ 1 s wall-clock on a desktop CPU.

## Drop-in snippet (sibling MPC project)

```python
from vdsim_plant import VDSimPlant

plant = VDSimPlant(config="ioniq5_awd.yaml", base_mu=0.9,
                   control_dt=0.1, substep_dt=1e-3)
plant.reset(state0=[0, 0, 0, 16.7, 0, 0])
for k in range(N):
    u = mpc.solve(...)           # [delta, Fx] — unchanged
    obs = plant.step(u)
    # feed obs to estimator / logging
```
