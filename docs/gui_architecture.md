# VDSim GUI / Web Architecture

Status: **aligned with v0.2** (2026-06-06). See `docs/design/RUNTIME_ARCH.md` for
branch [1] vs [2] split. Concept: the **plant** (C++ `vdsim_realtime`) computes
only; the browser edits scenarios, controls run lifecycle, and reconstructs viz
from compact state — not pixel streaming.

This is an accessibility / deployment differentiator vs license-locked desktop
VD tools (CarMaker / CarSim / Adams), not a research novelty. The technical fit
is that VDSim's output is compact physics state (not pixels), so the client
reconstructs the visualization locally — lightweight, unlike rendered-video
streaming.

---

## 1. Decisions

- Backend: Python first (FastAPI + websockets, wrapping the `vdsim` pybind
  module / SimSession), migrate to C++ later. The hard real-time loop stays in
  C++ (SimSession); Python orchestrates, serves, relays.
- 3D: Three.js with the WebGPU renderer and automatic WebGL2 fallback (WebGPU
  not hard-required; the vehicle scene is light).
- MVP scope: run on a server, configure + watch from another PC's browser.
  Single client, no auth. (Multi-client read-only + auth are later, product.)

## 2. The migration-proof principle (Python -> C++)

The frontend depends ONLY on a stable wire contract — REST endpoints + WS
message schema — never on the backend implementation. Same decoupling as the
UDP co-sim protocol. Define the API spec first, implement it in Python, and a
future C++ server implements the identical contract -> frontend unchanged.
Migration = backend swap, no frontend change.

## 3. Architecture (realized: v0.2)

```
[gui/server.py Runner]                         [browser: gui/app.html]
  in-memory run-config draft (Setup)
  on Play: runs/live/run_config.yaml → vdsim_realtime
  CosimBridge: UDP STATE in, CMD out
        |  REST  : /api/setup, /api/runconfig, /api/control, /api/vehicle, …
        |  SSE   : /api/stream @ ~60 Hz  ---> applyState → Three.js + telemetry
```

| Concern | Transport | Direction | Notes |
|---|---|---|---|
| Scenario / draft (fleet, path, road, parts) | REST | browser → server | `POST /api/setup`; YAML via `/api/runconfig` |
| Run lifecycle (start/stop/pause/reset) | REST | browser → server | `POST /api/control`; server spawns plant |
| Manual / io control while running | REST | browser → server → UDP CMD | relayed to [2] |
| State stream (viz, telemetry, minimap) | **SSE** | server → browser | Setup: draft snapshot; Running: relayed STATE |

**GUI does not step SimSession.** Python `vdsim` on the server loads YAML for
forms, geometry, and draft validation only.

Original sketch used FastAPI + WebSocket; realized stack is stdlib `http.server` +
SSE — same logical split, different transport.

## 4. Wire contract (to be frozen as the migration boundary)

REST (draft):
- `GET  /api/config/vehicle` / `PUT /api/config/vehicle` — VehicleParams (JSON)
- `GET/PUT /api/config/tire`, `/api/config/solver`, `/api/config/scenario`
- `POST /api/sim/start`, `/api/sim/stop`, `/api/sim/reset`
- `POST /api/sim/input` — manual CmdL4 (throttle/brake/steer/gear)
- `GET  /api/schema/vehicle` ... — JSON schema driving the config forms

WS / SSE (migration boundary — **schema** matters more than transport):
- `/api/stream` (SSE, realized) — server pushes JSON snapshots @ ~60 Hz.
  Fields mirror cosim STATE + `setup_mode`, `fleet_spec`, `live_vid`, errors.
  Setup: synthetic pose from spawn; Running: relayed plant output.
- (Future) `/ws/state` — optional binary WebSocket at sim rate; same field set.

Frozen field names/units mirror the existing CSV/STATE-packet conventions
(ENU world [m], ISO 8855 body, wheel FL=0..RR=3).

## 5. Frontend

SPA realized in `gui/index.html` (Three.js). Add:
- WebGPU renderer + WebGL2 fallback (Three.js `WebGPURenderer` with capability
  check).
- Config forms generated from JSON schema (so the UI stays in sync with the C++
  params instead of being hand-maintained).
- Time-series plots (charts) for vx, yaw rate, ax/ay, susp, etc.
- The planned actuator-nonlinearity tuner (sliders + step-response plot) is a
  panel here.

## 6. Existing assets reused

| Asset | Role in this design |
|---|---|
| `core` SimSession (set_input/tick/output) | compute kernel the server drives |
| `python` pybind `vdsim` module | Python access to SimSession (Phase 1) |
| `gui/app.html` + `gui/server.py` | scenario editor, run control, SSE relay |
| `cosim` STATE/CMD field set | the wire-contract field conventions |

## 7. Phased plan

1. Expose SimSession to pybind (`make_sim_session`, set_input/tick/output). [Phase 1]
2. FastAPI server: REST (config CRUD + start/stop) + WS (state stream), single client.
3. Frontend: schema-driven config forms + 3D viz (WebGPU/WebGL2) + plots.
4. Param JSON schema auto/derived from C++ params (UI sync).
5. Multi-client read-only broadcast + auth (product stage).
6. (Later) C++ backend implementing the same wire contract; retire Python server.

## 8. Risks / open items

- WebGPU maturity -> WebGL2 fallback mandatory (decided).
- Full param-surface config UI is significant work; schema-driven generation
  contains it.
- Source of truth: web UI reads/writes YAML via the API; YAML files remain the
  canonical store.
- Security: lab use over Tailscale is fine; product needs auth + TLS.
- Sensor/camera/lidar visualization stays delegated to CARLA (out of scope);
  VDSim streams physics only.
