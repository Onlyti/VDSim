# Runtime architecture — three branches over one core

Status: design / alignment (not yet executed). Supersedes the ad-hoc wiring that
accreted across `gui/`, `viewer/`, `cosim/`, `python/vdsim_comms.py`.

## Why this doc exists

The runtime grew by accretion. Concretely, today there are **three independent
owners of a live simulation** and **multiple incompatible comm paths**:

| owner | sim instance | transport | protocol |
|---|---|---|---|
| `cosim/udp_server.cpp` | own `SimSession` + `RealTimeRunner` | UDP | VDS1 binary, CRC32, versioned (`cosim_protocol.hpp`) — canonical |
| `python/vdsim_comms.py` | own `Simulation` (via lab) | UDP | toy templates (`vds1_cmd = <Iddd`, json, nmea) — divergent, NOT wire-compatible with the above |
| `gui/server.py` | own `RUNNER` | HTTP SSE + HTTP `/api/manual` + UDP 8101 (FFB) + HTTP `/api/io` fan-out | ad-hoc JSON |

Additional sprawl:
- Two overlapping web viewers: `gui/` and `viewer/` (`realtime_server.py`,
  `suspension_editor`).
- `gui/server.py` alone fuses all three branches (viz + control + signal
  fan-out + its own sim loop). The FFB-over-UDP port (8101) bolted onto it is
  the clearest symptom.
- Two definitions of "VDS1" that cannot talk to each other.

This is the sprawl to remove before adding more features.

## Target: one core, three thin adapters, one-way control

```
                  +-----------------------------------+
                  |  core: vdsim.SimSession (C++)      |   single source of truth
                  |  == vdsim_lab.Simulation (python)  |   step / set_control / get_data
                  +------------------+----------------+
        +----------------------------+----------------------------+
   [3] Sync API                 [2] UDP comms                [1] Web viewer
   direct calls                 control in / signal out      subscribe only (read-only)
   batch, embedded              ONE protocol + router         never owns a sim
   (foundation of the other 2)  wheel / HIL / external node   no control logic
```

Three rules:

1. **[3] Sync API is the foundation.** `Simulation.step / set_control /
   get_data` is the only stepping path. Batch and embedded callers use only
   this. (`python/vdsim_lab.py`, `tools/vdsim_batch.py`, the `vdsim` .so.)

2. **[2] is the real-time runtime, not merely "comms."** `vdsim_udp_server` is
   VDSim's real-time application: it drives the *same* SimSession core as [3],
   but paced by `RealTimeRunner` against a wall clock, with UDP as its I/O. So
   [3] and [2] are the two ways to run one core — offline/deterministic vs
   real-time (CarMaker's ERG-batch vs HIL analogue). UDP is just the app's I/O
   surface: control SOURCES (autopilot, wheel, external node, AutoHYU bridge) in,
   signal SINKS (viewer, logger, HIL, FFB) out. One wire spec, canonical =
   `cosim/cosim_protocol.hpp`, Python mirror = `cosim/protocol.py`. Bringing the
   server to feature parity with the in-process sim (road/terrain/sensors,
   extended STATE) is what makes it the *full* runtime — until then it is a
   flat-ground half (see Open items).

3. **[1] Web viewer observes only.** It subscribes to the signal stream (the
   same data branch [2] emits, or a read-only state feed) and renders. It holds
   no authoritative control. A human driving with a wheel is a control SOURCE on
   branch [2], not logic living in the web server.

Control always flows in through [2]'s in-channel or [3]'s `set_control`. The web
never holds control. This is the invariant that keeps the branches from
re-tangling.

## Consolidation actions

- `gui/server.py`: strip to viz-only (SSE + static). Move `/api/manual`,
  `_udp_control` (8101), `/api/io` + `/api/io/targets` out to branch [2].
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
  udp_server.cpp       # the single comms server (CMD in / STATE out)
  test_udp_client.py   # smoke test, uses protocol.py
configs/comms/*.yaml   # routing spec (fan-out not yet realized in the server)
gui/                   # [1] the one web viewer: SSE + static, subscribes
builder/               # authoring tool (incl. suspension editor), separate concern
```

## Decisions taken (2026-06-04)

- Single web viewer: `gui/` (kept), `viewer/` retired (suspension editor folded
  into `builder/`). DONE.
- Single comms layer: C++ `vdsim_udp_server`. Python router (`vdsim_comms.py`)
  retired; its toy templates deleted. Python participants use `cosim/protocol.py`.
  DONE.

## Open items

- Branch [2] fan-out: the C++ server currently has one CMD-in port + one
  STATE-out destination (CLI). Config-driven multi-destination fan-out (so
  viewer + logger + HIL can all subscribe) is the configs/comms intent, not yet
  realized in the server.
- Branch [1]: `gui/server.py` still owns its own sim + control + fan-out (task
  #147). Convert it to subscribe to the udp_server STATE and drop its control
  ownership; wheel clients send CMD to the udp_server directly.
