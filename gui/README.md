# VDSim Web GUI (MVP)

Compute runs on the server (the `vdsim` SimSession); the browser does all
visualization and configuration. Run on a server, open the URL from any PC.

## Run

```bash
# 1) build the Python module once
cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build -j

# 2) start the GUI server (compute happens here)
python3 gui/server.py --port 8090

# 3) open from any device's browser
#    http://<server-ip>:8090   (or via Tailscale / `ssh -L 8090:localhost:8090`)
```

Stdlib only — no pip dependencies. Three.js loads from a CDN (client needs net),
state streams via Server-Sent Events, config/control via REST.

## What you can do

- Pick vehicle (sedan / sports / fsk_formula / race_car) and dynamics level
  (L1 / L2 / L3) — live, the sim rebuilds.
- Set target speed; Autopilot drives a figure-eight (Pure Pursuit + speed P).
- Manual mode: arrow keys (↑/↓ throttle/brake, ←/→ steer).
- Start / Stop / Reset, follow camera.
- HUD: vx, vy, yaw rate, ax, ay, roll, pitch, steer, per-wheel Fz.

## Architecture / wire contract

See `docs/gui_architecture.md`. The browser depends only on the REST + SSE
contract, so the backend can later move from Python to C++ without touching the
frontend. WebGPU rendering is a later (quality) swap; the MVP uses WebGL2.

## Files

| Path | Role |
|---|---|
| `gui/server.py`  | stdlib HTTP server: REST config/control + SSE state; runs SimSession |
| `gui/app.html`   | full-screen Three.js viewer + fleet/setup/telemetry + edit modal |
