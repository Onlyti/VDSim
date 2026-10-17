# Live 3D monitor

Watch a running simulation in a browser: a 5 m white/grey checker ground under
a sky-blue background, a box-and-wheels vehicle that follows the state, an
optional OpenStreetMap layer, a chase camera you can orbit with the mouse and a
one-line HUD (time, speed, road-wheel steer).

The monitor is display only. It reads the state you push and never feeds
anything back into the simulation.

## Publish from a simulation loop

```python
from vdsim_livemonitor import LiveMonitor, vehicle_geometry

with LiveMonitor(port=8790, vehicle=vehicle_geometry(plant.vehicle.params)) as mon:
    for k in range(n):
        obs = plant.step(u)
        mon.push({**obs, "t": (k + 1) * dt})
```

`push(state)` takes a `VDSimPlant` observation as it is (`X`, `Y`, `psi`,
`vx`, `delta_rw`, `roll`, `pitch`) or the short keys `x`, `y`, `yaw`, `v`,
`delta`. Position and heading are required; everything else is optional. The
call sends one UDP datagram to `127.0.0.1:<port>` and returns at once; calls
faster than `rate_hz` (default 30) are dropped and return `False`, so it is
safe inside a 100 Hz controller or a `ClosedLoopEval` callback.

`examples/live_monitor_demo.py` drives a plant in real time along a weaving
lane and publishes every control period.

## When the monitor is on

The page server lives with the simulator. Nothing is installed as a
service: there is no monitor while no simulation runs.

| way of running VDSim | monitor turns on | monitor turns off |
|---|---|---|
| from your code with the `monitor=` option (`VDSimPlant`, `VDSimEnv`, `VDSimVecEnv`) | when the object is built with `monitor=` set; off by default (see "Turning the monitor on from code") | at `close()` of that object / process exit |
| from your code with your own `LiveMonitor` (any loop, e.g. a `ClosedLoopEval` callback) | when your code calls `LiveMonitor.start()` (or enters `with LiveMonitor(...)`) | at `LiveMonitor.stop()` / leaving the `with` block / process exit |
| as a separate program: `vdsim_realtime --monitor[=PORT]` | when the program starts (`--monitor` absent = off) | when the program ends: end of the scene, Ctrl-C (SIGINT), `kill` (SIGTERM); also when it is killed outright (Linux) |

Without `monitor=` and without a `LiveMonitor` the library starts no page
server, no thread and no socket.

### From your code

`LiveMonitor.start()` asks `http://127.0.0.1:<port>/health` first. When a
monitor server already answers on the port it publishes there; otherwise it
binds the port itself (`embed=False` turns that off). A port held by anything
else is an error.

### As a separate program

```
vdsim_realtime --scene=configs/scenes/two_vehicle_race.yaml --monitor
```

starts `python -m vdsim_livemonitor serve --port 8790` as a child process and
sends the first vehicle of the scene to it at up to 30 Hz over loopback UDP
(the same datagram `LiveMonitor.push()` sends). The page is
`http://<sim-host>:8790/`.

* `--monitor=PORT` picks another port. `--monitor-python=<python>` picks the
  interpreter; the default is the one the build was configured with, and the
  source tree's `python/` directory is put on its `PYTHONPATH`.
* Monitor trouble never stops the simulation: no interpreter, no
  `vdsim_livemonitor`, a port already in use or a page server that exits is
  one `[vdsim_realtime] monitor: ... continuing without monitor` line and the
  scene runs on.
* On exit the child gets SIGTERM, then SIGKILL after 3 s; the port is free
  again when `vdsim_realtime` returns. On Linux the child also receives
  SIGTERM if `vdsim_realtime` itself is killed with SIGKILL.
* POSIX only. On Windows `--monitor` prints the one warning line and the
  simulation runs without the monitor.

`python -m vdsim_livemonitor serve --port 8790` still runs a page server by
hand, e.g. to watch several short runs in a row from one tab.

### Opening the page

* A tab opened while no simulation runs gets the browser's own "connection
  refused" page. That is expected: start the simulation and reload.
* A tab left open from an earlier run keeps retrying (a lost WebSocket is
  retried every second, a connection silent for 3 s is dropped and retried)
  and shows "monitor server unreachable, retrying" meanwhile. It picks the
  next run up by itself: in the work order 36 check (simulation host and
  browser on the lab network) the state appeared 1.5 s after the simulator
  was launched, the same as on a freshly opened tab.

Publishers must run on the same host as the page server: the UDP ingest port
is bound to the loopback interface. The page itself is served on every
interface (`--host` to restrict a hand-started `serve`).

## Turning the monitor on from code

A code run can bring up the page by itself: the world (ground, roads) is on
the page as soon as the object is built, a vehicle appears with its first
state and the page keeps the last frame under "simulation finished" after
`close()`.

### One vehicle

```python
from vdsim_plant import VDSimPlant

plant = VDSimPlant(control_dt=0.01, monitor=True)     # page: http://<host>:8790/
plant.reset([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])
for k in range(500):
    plant.step([0.0, 900.0])
plant.close()
```

### Many envs (RL)

```python
from vdsim_rl import VDSimVecEnv

env = VDSimVecEnv(64, monitor=True, monitor_max_vehicles=16, monitor_layout="grid")
obs, _ = env.reset(seed=0)
for _ in range(1000):
    obs, rew, term, trunc, info = env.step(policy(obs))
env.close()
```

`examples/rl_live_monitor_demo.py` runs a short search over a lane-keeping
policy on 16 envs with the monitor on.

| option | meaning |
|---|---|
| `monitor` | `None` / `False` = off (default); `True` = port 8790; an `int` = that port; a `LiveMonitor` = publish through it (not stopped by `close()`) |
| `monitor_max_vehicles` (`VDSimVecEnv`, default 16) | the first K envs are drawn, each in its own colour; the HUD shows "K / N shown" |
| `monitor_layout` (`VDSimVecEnv`) | `"overlay"` (default): all envs in the same world; `"grid"`: env i shifted by `i * monitor_grid_spacing` [m] (default 8) to the left, display only |
| `monitor_pace` | `None` (default): run as fast as the simulation steps; a number, e.g. `1.0`, sleeps inside `step()` to hold that sim/wall ratio |
| `monitor_wait_s` | wait up to this long [s] for a page before returning from the constructor, so a short run is not over before the tab opens; on time-out one warning line, then the run goes on (default 0 = do not wait) |

`VDSimEnv` takes `monitor`, `monitor_pace` and `monitor_wait_s`.

What the page shows:

* Each env is one vehicle with a trail. An env that ends its episode and
  restarts (auto-reset in `step()`) has only its own trail cleared.
* The HUD's second line: simulated time, wall time since the first step, the
  sim/wall rate over the last second (`x N`), vehicles shown and the run kind
  (`code` for a plant, `rl` for an env).
* A run faster than real time is shown fast; that is the intended view of an
  unpaced run. Poses are read from the env pool only when a send is due (at
  most 30 per wall second, newest state only); the frames in between are not
  sent. In the work order 37 check (64 envs, 16 drawn, one page connected,
  same seed and 1000 steps, median of 5 runs) the monitor cost about 2 % of
  the steps per second.

Monitor trouble never stops a code run: a port held by something else,
missing page assets or a failing send is one `warning:` line on stderr and
the run goes on without the monitor.

## One-click shortcut (Windows)

```
python -m vdsim_livemonitor shortcut --url http://<sim-host>:8790/ --out "VDSim monitor.url"
```

writes an Internet Shortcut. Double-clicking it on any PC that can reach the
simulation host opens the page in the default browser while a simulation with
the monitor on is running (see "Opening the page" above).

`LiveMonitor(open_browser=True)` / `serve --open` open the page on the
simulation host itself when it has a display.

## OSM layer

```python
LiveMonitor(osm="map.osm", origin=(lat0, lon0))
```

* `origin` is the geodetic position of the simulation frame origin (x east,
  y north). Without it the OSM file is refused with that reason: the map
  cannot be placed in the simulation frame otherwise.
* Ways tagged `highway=*` are drawn as grey ribbons (width by class) with a
  centre line; Lanelet2 line ways (`type=line_thin`, `line_thick`, `line`,
  `curbstone`, `road_border`, `virtual`) as thin lines, `virtual` dashed.
  Other ways (buildings, relations) are ignored.
* Nodes are mapped with the exact WGS84 local tangent plane at the origin
  (geodetic → ECEF → ENU), the transform a GeographicLib `LocalCartesian`
  uses.
* An OSM error does not stop the page server; the reason is shown on the
  HUD.

## What the monitor does not promise

* The page frame rate is the browser's; states are sent at up to `rate_hz`
  and only the newest one is kept. A late page skips states, it does not
  slow the simulation.
* UDP on the loopback interface is not acknowledged. A datagram lost under
  extreme load is a skipped frame.
* No authentication. Anyone who can reach the port can watch.
* The vehicle is drawn from axle positions, mean track and nominal wheel
  radius; body size is a fixed default unless you pass `vehicle={...}`.
* three.js is the vendored copy in `gui/vendor/three` (installed with the
  wheel); no CDN is used.
