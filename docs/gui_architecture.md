# VDSim GUI / Web Architecture

Status: DESIGN (decided). Concept: VDSim computes only; all visualization and
configuration happen in a web browser on the receiving device. The simulator
sends/receives physics + config over the network and renders nothing itself.

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

## 3. Architecture

```
[VDSim compute server]                         [any device: browser]
  C++ core: SimSession (set_input / tick / output)   <- hard real-time loop
        |
  Web/API server (Python: FastAPI + websockets, pybind vdsim)
        |  REST  : config CRUD, scenario, start/stop/reset  <--- config UI (forms)
        |  WS    : state stream (binary/JSON) @ sim rate     ---> 3D viz (Three.js)
        |  WS    : time-series                               ---> plots
```

Three concerns, same split as the udp_server:

| Concern | Transport | Direction |
|---|---|---|
| Configuration (vehicle/tire/solver/scenario/actuator) | REST (HTTP) | browser -> server; validate -> YAML |
| Control (start/stop/reset/manual input) | REST or WS | browser -> server -> set_input |
| State stream (viz/plots) | WebSocket (high-rate) | server -> browser, from output() |

## 4. Wire contract (to be frozen as the migration boundary)

REST (draft):
- `GET  /api/config/vehicle` / `PUT /api/config/vehicle` — VehicleParams (JSON)
- `GET/PUT /api/config/tire`, `/api/config/solver`, `/api/config/scenario`
- `POST /api/sim/start`, `/api/sim/stop`, `/api/sim/reset`
- `POST /api/sim/input` — manual CmdL4 (throttle/brake/steer/gear)
- `GET  /api/schema/vehicle` ... — JSON schema driving the config forms

WS:
- `/ws/state` — server pushes SimOutput frames (state + ax/ay/roll/pitch/Fz/
  steer_applied) at the sim rate. Binary preferred for high rate.

Frozen field names/units mirror the existing CSV/STATE-packet conventions
(ENU world [m], ISO 8855 body, wheel FL=0..RR=3).

## 5. Frontend

SPA seeded from `viewer/index.html` (Three.js). Add:
- WebGPU renderer + WebGL2 fallback (Three.js `WebGPURenderer` with capability
  check).
- Config forms generated from JSON schema (so the UI stays in sync with the C++
  params instead of being hand-maintained).
- Time-series plots (charts) for vx, yaw rate, ax/ay, susp, etc.
- The planned actuator-nonlinearity tuner (sliders + step-response plot, see
  `viewer/README.md`) is a panel here.

## 6. Existing assets reused

| Asset | Role in this design |
|---|---|
| `core` SimSession (set_input/tick/output) | compute kernel the server drives |
| `python` pybind `vdsim` module | Python access to SimSession (Phase 1) |
| `viewer/index.html` (Three.js) | frontend seed |
| `viewer/realtime_server.py` (ws + pybind) | backend seed -> FastAPI |
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
