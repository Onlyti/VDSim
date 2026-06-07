# Runtime architecture — three branches over one core

Status: **aligned with v0.2 GUI** (2026-06-06). Historical sprawl notes below;
**[1] GUI role updated** — scenario editor + run controller + viewer (see
§ Target). Supersedes the ad-hoc wiring that accreted across `gui/`, `viewer/`,
`cosim/`, `python/vdsim_comms.py`.

## Why this doc exists

The runtime grew by accretion. Concretely, today there are **three independent
owners of a live simulation** and **multiple incompatible comm paths**:

| owner | sim instance | transport | protocol |
|---|---|---|---|
| `cosim/realtime_server.cpp` | own `SimSession` + `RealTimeRunner` | UDP | VDS1 binary, CRC32, versioned (`cosim_protocol.hpp`) — canonical |
| `python/vdsim_comms.py` | own `Simulation` (via lab) | UDP | toy templates (`vds1_cmd = <Iddd`, json, nmea) — divergent, NOT wire-compatible with the above |
| `gui/server.py` | **no SimSession step**; orchestrates draft + plant lifecycle | HTTP REST + SSE `/api/stream` | JSON (draft snapshot or relayed STATE) |

Additional sprawl:
- Two overlapping web viewers: `gui/` and `viewer/` (`realtime_server.py`,
  `suspension_editor`).
- `gui/server.py` holds an in-memory **run-config draft** (fleet, path, road),
  materializes `run_config.yaml` on ▶ Play, spawns/attaches `vdsim_realtime`,
  relays UDP STATE to the browser via SSE. It does **not** integrate the plant.
- Legacy sprawl still to trim: `/api/manual`, FFB UDP (8101), `/api/io` fan-out
  ideally move to branch [2] clients/router (see Consolidation).
- Two definitions of "VDS1" that cannot talk to each other.

This is the sprawl to remove before adding more features.

## Target: one core, three thin adapters

```
                  +-----------------------------------+
                  |  core: vdsim.SimSession (C++)      |   single source of truth
                  |  == vdsim_lab.Simulation (python)  |   step / set_control / get_data
                  +------------------+----------------+
        +----------------------------+----------------------------+
   [3] Sync API                 [2] Real-time runtime          [1] Web GUI
   direct calls                 vdsim_realtime + VDS1 UDP      scenario editor +
   batch, embedded              CMD in / STATE out              run controller + viewer
   (foundation of the other 2)  wheel / HIL / external node     **no physics step**
```

Three rules:

1. **[3] Sync API is the foundation.** `Simulation.step / set_control /
   get_data` is the only stepping path for offline/batch. (`python/vdsim_lab.py`,
   `tools/vdsim_batch.py`, the `vdsim` .so.)

2. **[2] is the real-time runtime, not merely "comms."** `vdsim_realtime` drives
   the *same* SimSession core as [3], paced by `RealTimeRunner` against a wall
   clock, with UDP as its I/O. [3] and [2] are offline/deterministic vs
   real-time. UDP wire spec: `cosim/cosim_protocol.hpp`, Python mirror =
   `cosim/protocol.py`.

3. **[1] Web GUI — scenario editor, run controller, viewer.** The browser never
   steps SimSession. `gui/server.py` is a thin **orchestrator**:
   - **Setup (draft):** edit fleet/path/road/parts in memory; preview in Three.js;
     SSE pushes a synthetic snapshot (`setup_mode`, `source: "setup"`) built from
     spawn poses — not plant telemetry.
   - **▶ Play:** write `run_config.yaml`, start or attach `vdsim_realtime`; relay
     CMD (autopilot/manual/io); SSE pushes relayed plant STATE (`source: "cosim"`).
   - **REST** = authoring + lifecycle (`/api/setup`, `/api/runconfig`, `/api/control`, …).
   - **SSE** = one-way live state bus to the browser (~60 Hz); see § SSE below.

   A human with a wheel is still a control **source** on branch [2] (UDP CMD), not
   physics inside the web server. The GUI *starts/stops* the plant and *edits* the
   scenario; it does not *integrate* it.

**Invariant:** physics stepping only in [2] (real-time) or [3] (batch). [1] may
command and configure [2], but never replaces it as the plant.

## SSE (`GET /api/stream`)

**Server-Sent Events** — HTTP long-lived connection; server → browser only.

| | REST `/api/*` | SSE `/api/stream` |
|---|---|---|
| Direction | request/response | server push |
| Rate | on user/action | ~60 Hz |
| Setup | draft CRUD (`POST /api/setup`, …) | draft mirror: `fleet_spec` spawn → pose fields |
| Running | start/stop/pause, manual, io | relayed VDS1 STATE + `fleet`, telemetry |

Browser: `EventSource('/api/stream')` → `applyState(json)` drives 3D, minimap,
connection badge, right-hand telemetry. Same JSON schema in both modes; **`setup_mode`
and `source`** distinguish draft preview from live plant data.

Implementation note: `gui_architecture.md` originally specified WebSocket for the
state stream; the realized server uses SSE (stdlib `http.server`, sufficient for
60 Hz JSON). A future backend swap must keep the **field schema**, transport may
change.

## Consolidation actions (remaining)
- `viewer/` vs `gui/`: pick one web viewer; fold `suspension_editor` in as a
  panel or move it to `builder/` (authoring concern).
- VDS1: one wire format. `cosim_protocol.hpp` is canonical; Python mirrors it.
  Delete `vds1_cmd = <Iddd` and the alias templates.
- `vdsim_comms.run_rt` vs `cosim/udp_server`: keep both, but justified by role —
  Python router = config-driven reference; C++ server = high-rate / embedded —
  sharing the one protocol. Otherwise drop one.
- `tools/wheel_ffb*.py`: these are control SOURCES (clients) for branch [2], not
  part of the viewer. Group them as clients.

## Repo layout (module boundaries, not separate packages)

Open-core single product — module boundaries inside the repo are enough; do not
split into separate pip packages yet.

No physical directory rename (avoids churning the `vdsim::cosim` namespace).
The existing `cosim/` is branch [2].

```
core/                  # C++ engine (unchanged)
python/
  vdsim (.so)          # bindings
  vdsim_lab.py         # [3] Sync API: Simulation + Experiment + metrics
tools/vdsim_batch.py   # [3] batch over the API
tools/wheel_ffb*.py    # control SOURCES (clients) for branch [2]
cosim/                 # [2] the one comms layer
  cosim_protocol.hpp   # canonical VDS1 wire format (C++)
  protocol.py          # Python mirror of it (encode CMD / decode STATE)
  realtime_server.cpp       # the single comms server (CMD in / STATE out)
  test_udp_client.py   # smoke test, uses protocol.py
configs/comms/*.yaml   # routing spec (fan-out not yet realized in the server)
gui/                   # [1] app.html + server.py: draft, Play/Stop, SSE relay
builder/               # hardpoint / suspension authoring (offline)
```

## Decisions taken (2026-06-04)

- Single web viewer: `gui/` (kept), `viewer/` retired (suspension editor folded
  into `builder/`). DONE.
- Single comms layer: C++ `vdsim_realtime`. Python router (`vdsim_comms.py`)
  retired; its toy templates deleted. Python participants use `cosim/protocol.py`.
  DONE.

## B-track progress (2026-06-04, post-v0.1.0)

- B1 STATE v2 (rack_torque/slip/susp/measured); B2 server road/terrain config;
  B3 server SensorParams yaml; B4 `gui/server.py` no longer steps SimSession —
  auto-starts `vdsim_realtime`, relays CMD, SSE relays STATE. DONE.
- Real-time app renamed `vdsim_realtime` (core pacing class stays `RealTimeRunner`).

## v0.2 GUI alignment (2026-06-06)

- **Setup = draft session:** `Runner.apply_setup()` updates in-memory run spec;
  `_setup_snapshot()` feeds SSE until ▶ Play.
- **Play:** `_materialize_run_config()` → `runs/live/run_config.yaml` → `vdsim_realtime
  --scenario=…`; SSE switches to cosim STATE. Override dir: env `VDSIM_RUN_DIR`.
- **Single page:** `gui/app.html` — SIM bar, fleet tree, setup panel, 3D, telemetry.
- See also `docs/design/V0.2_GUI_REDESIGN.md`, `docs/HANDOFF.md` §6b.

## Open items

- Branch [2] fan-out: one CMD-in port + one STATE-out destination (CLI).
  Config-driven multi-destination fan-out (viewer + logger + HIL at once) is the
  configs/comms intent, not yet realized in the server.
- Parity gaps deferred from B4: ActuatorParams and SolverParams
  (integrator/substeps) are not yet passed to the server, so the GUI's
  actuator/solver panels don't affect the plant. rack_torque is emitted at L2
  but reads 0 at L3 (FFB on L3).
