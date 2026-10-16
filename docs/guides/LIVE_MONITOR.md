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

## Embedded or standing server

| | embedded | standing |
|---|---|---|
| started by | `LiveMonitor.start()` when nothing listens on the port | `python -m vdsim_livemonitor serve --port 8790` |
| page reachable | only while the simulation process runs | always |
| with no simulator | — | page shows "waiting for simulator" |
| simulator restarts | page reconnects once the port is back | state appears within one push |

`LiveMonitor` asks `http://127.0.0.1:<port>/health` first. When a standing
server answers it publishes there; otherwise it binds the port itself
(`embed=False` turns that off). A port held by anything else is an error.

The page reconnects by itself: a lost WebSocket is retried every second, and a
connection that has been silent for 3 s is dropped and retried. Keep a
standing server running on the simulation host so that the page opens before
any simulator does, e.g. as a user service:

```ini
# ~/.config/systemd/user/vdsim-livemonitor.service
[Service]
ExecStart=/usr/bin/python3 -m vdsim_livemonitor serve --port 8790
Restart=on-failure
[Install]
WantedBy=default.target
```

Publishers must run on the same host as the server: the UDP ingest port is
bound to the loopback interface. The page itself is served on every interface
(`--host` to restrict it).

## One-click shortcut (Windows)

```
python -m vdsim_livemonitor shortcut --url http://<sim-host>:8790/ --out "VDSim monitor.url"
```

writes an Internet Shortcut. Double-clicking it on any PC that can reach the
simulation host opens the page in the default browser. With a standing server
it shows "waiting for simulator" until a simulation starts publishing.

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
* An OSM error on a standing server does not stop it; the reason is shown on
  the HUD.

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
