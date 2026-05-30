# Examples — CLI demo binaries

VDSim 의 10 CLI binaries.  build/bin/ 에 위치.

| Binary | Purpose |
|---|---|
| `vdsim_bicycle_run` | Single tier (Ld1-Bicycle, Lc4-Pedal) scenario runner |
| `vdsim_scenario_run` | Scenario YAML → 14-column time-series CSV |
| `vdsim_l1_vs_l2` | 동일 scenario 의 Ld1 vs Ld2 paired CSV |
| `vdsim_ax_track_demo` | Lc5-AxTarget PID step profile on Ld2 |
| `vdsim_path_tracking` | Lc4 ← Lc5 ← Lc6 ← Lc7 cascade on figure-eight |
| `vdsim_driver_demo` | DriverModel (latency + Gaussian noise) on Ld2 |
| `vdsim_l3_demo` | Ld3-FourteenDOF transient trace with susp_velocity dump |
| `vdsim_split_mu_demo` | Open / Locked / LSD differential comparison |
| `vdsim_l1_vs_l2` (paired) | identical scenario, two dynamics tiers side by side |
| `vdsim_scenario_run` | flexible scenario runner with time-varying mu |

## Quick example

```bash
build/bin/vdsim_path_tracking \
    configs/vehicles/sports.yaml \
    configs/tires/default_pacejka.yaml \
    /tmp/path.csv 8.0
```

Outputs `/tmp/path.csv` with `t, x, y, yaw, vx, ...` columns. Visualize with
either:

- Python: `matplotlib` on the CSV.
- Browser: load the CSV into the [3D viewer](https://github.com/Onlyti/VDSim/tree/main/viewer).

## Python (pybind11)

```python
import sys; sys.path.insert(0, "build/python")
import vdsim

vp = vdsim.VehicleParams.from_yaml("configs/vehicles/sedan.yaml")
tp = vdsim.TireParams.from_yaml("configs/tires/default_pacejka.yaml")
sp = vdsim.SolverParams()

dyn = vdsim.create_seven_dof()
dyn.initialize(vp, tp, sp)

s = vdsim.State(); s.velocity = [10.0, 0.0, 0.0]; dyn.reset(s)
contacts = [vdsim.ContactPoint() for _ in range(4)]
for c in contacts: c.is_valid = True; c.normal = [0,0,1]; c.mu_long = c.mu_lat = 1.0

cmd = vdsim.CmdL4(); cmd.steer_angle_wheel = 0.05
for _ in range(1000): dyn.step(cmd, contacts, 0.005)
print(f"vx_end = {dyn.state().vx():.3f}, r = {dyn.state().yaw_rate():.4f}")
```

## Sweep across parameters

```bash
python3 python/sweep_runner.py \
    configs/sweeps/aero_vs_vx.yaml \
    /tmp/sweep_out
```

3 × 3 = 9 cells (aero_drag_coeff × initial_vx) → `/tmp/sweep_out/<tag>/` per cell.
