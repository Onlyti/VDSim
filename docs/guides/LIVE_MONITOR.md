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
| from your code (`VDSimPlant`, `ClosedLoopEval`, RL envs) | when your code calls `LiveMonitor.start()` (or enters `with LiveMonitor(...)`); the library never starts one by itself | at `LiveMonitor.stop()` / leaving the `with` block / process exit |
| as a separate program: `vdsim_realtime --monitor[=PORT]` | when the program starts (`--monitor` absent = off) | when the program ends: end of the scene, Ctrl-C (SIGINT), `kill` (SIGTERM); also when it is killed outright (Linux) |

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
