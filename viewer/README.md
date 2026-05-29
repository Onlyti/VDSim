# VDSim 3D Viewer (PoC v1)

Browser-based Three.js viewer for VDSim trajectories. Two modes:

1. **CSV replay** — load a recorded run and scrub through it.
2. **Realtime simulation** — connect to a Python WebSocket server that drives the
   `vdsim` pybind11 module live.

## Quick start

### 1. Build VDSim with Python bindings (one time)

```bash
cd ~/git/VDSim
cmake -G Ninja -B build -DVDSIM_BUILD_PYTHON=ON
cmake --build build -j
```

This produces `build/python/vdsim*.so`.

### 2. Serve the viewer (static HTML)

```bash
cd ~/git/VDSim
python3 -m http.server -d viewer 8080
```

Open `http://localhost:8080` (or via VSCode tunnel, the port is forwarded
automatically — just click the URL VSCode shows when port 8080 starts listening).

### 3a. CSV replay

In any other terminal:
```bash
build/bin/vdsim_path_tracking \
    configs/vehicles/sports.yaml \
    configs/tires/default_pacejka.yaml /tmp/run.csv 8.0
```

In the viewer, click **Choose File** → pick `/tmp/run.csv`. Auto-plays.

Compatible with output of:
- `vdsim_scenario_run`
- `vdsim_path_tracking`
- `vdsim_l1_vs_l2`
- `vdsim_l3_demo`
- `vdsim_driver_demo`

### 3b. Realtime simulation

```bash
pip install websockets   # one time
python3 viewer/realtime_server.py --driver --level L2 --v_target 10
```

In the viewer click **Connect ws://localhost:8765**. The vehicle starts
driving a figure-eight (Pure Pursuit + Lc6-VTarget PI on Ld2-SevenDOF) and
the browser renders the state at ~30 FPS.

Server options:
```
--vehicle  configs/vehicles/{sedan,sports,fsk_formula,race_car}.yaml
--tire     configs/tires/default_pacejka.yaml
--level    L1 | L2 | L3
--fps      30
--driver   (closed-loop PP + PI; omit for square-steer demo)
--v_target 10.0
```

## Controls

| UI                | Action                                       |
|---|---|
| Camera: Orbit     | Mouse drag (Three.js OrbitControls).         |
| Camera: Follow    | Chase from behind, fixed offset.             |
| Camera: Chase close | Tighter follow.                            |
| Camera: Top-down  | Straight down at 60 m.                       |
| Play / Pause / Rewind | CSV-only timeline control.               |
| Speed 0.25x–10x   | Speed multiplier for CSV replay.             |
| Timeline slider   | Scrub through CSV.                           |
| Trail: Clear / show | Toggles the orange world-frame path.       |

HUD shows t, vx, vy, r, ax, ay, roll, pitch, steer, throttle, brake live.

## CSV format

```
t,x,y,yaw,vx,vy,r,throttle,brake,steer,...,ax,ay,roll,pitch
```

Any superset is fine — the viewer reads what it needs by name. Missing columns
fall back to 0 / displayed as `—`.

## Known limits (PoC v1)

- Vehicle mesh is a fixed-size box + cylinder wheels (does not auto-resize from
  VehicleParams; takes the default sedan dimensions).
- Suspension link visualization is **planned for Ld4-MultibodyKinematic** integration.
- No per-wheel Fz overlay yet — column space already exists in CSV; v2.
- `--driver` realtime mode uses a built-in Pure Pursuit; for full control choose
  a different scenario or wire Lc8-Waypoint through the binding.

## Files

| Path                          | Role                                       |
|---|---|
| `viewer/index.html`           | Single-page Three.js viewer.               |
| `viewer/realtime_server.py`   | asyncio + websockets + vdsim live sim.     |
| `viewer/README.md`            | This file.                                 |
