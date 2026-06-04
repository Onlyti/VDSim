# VDSim Experiment Builder

A dedicated **authoring web tool** (separate from the PoC sim GUI in `gui/`) to
build the four experiment artifacts as validated YAML, which `vdsim_lab` then runs.

```sh
python3 builder/server.py --port 8200      # http://<host>:8200
```

Four tabs, each saves a validated config:

| tab | output | schema |
|---|---|---|
| **Vehicle** | `configs/vehicles/<n>.yaml` | VehicleParams (mass, geometry, powertrain, brakes, steering, aero). Preview runs a full-throttle launch and reports peak accel / 7 s speed. |
| **Sensors** | `configs/sensors/<n>.yaml` | suite: `{sensors:[{id,type,mount[x,y,z],yaw,rate,noise_std}]}` (gnss/imu/wheel_speed/steer/camera/lidar) |
| **Map** | `configs/maps/<n>.yaml` | `{driving_line:{source: shape\|waypoints\|xodr\|rd5, ...}, road:{width, surface:{ref}}}`; 2D canvas preview |
| **Comms** | `configs/comms/<n>.yaml` | data routing: channels `{source, template, to:[ip:port]}` (fan-out) or `{direction:in, listen:{port}}` (fan-in). Templates: json/vds1_state/vds1_cmd/nmea_gga/imu_raw |
| **Scenario** | `configs/experiments/<n>.yaml` | compose `{vehicle, tire, level, map, maneuver, sensors, duration, run:{mode: api\|rt_comms, comms}}` |

Saves are gated by validation (e.g. a vehicle config is loaded through
`VehicleParams.from_yaml`; a map's driving line must resolve to ≥2 points).

## Closing the loop — run an authored scenario

```python
import sys; sys.path.insert(0, "python")
from vdsim_lab import Experiment
res = Experiment.from_config("my_experiment")   # configs/experiments/my_experiment.yaml
res.to_csv("run.csv"); print(res.summary())
```

`from_config` resolves the map (driving line + surface), the maneuver, and the
sensor suite, then runs — the same `resolve_line` is shared with this tool so the
preview and the run agree.

## Map driving-line sources

- `shape`: `oval` (R, L), `circle` (R), `figure8` (R)
- `waypoints`: explicit `points: [[x,y], ...]`
- `xodr`: OpenDRIVE file → link-following centerline (`examples/opendrive.py`)
- `rd5`: CarMaker IPGRoad `Route_0` (`examples/rd5_route.py`)
