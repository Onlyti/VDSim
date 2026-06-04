# Design: simulation configuration architecture

Status: DRAFT for alignment. How a VDSim run is *set up by configuration only*
(MORAI / CarMaker-CLI style): separate vehicle / sensor / map configs, a scenario
that composes them, a communication config for data routing, and two execution
modes (real-time comms, or embedded API).

## 1. Principle

Setup = configuration, no code. Concerns are separate files, the scenario
references them, and the same scenario runs headless (CLI/batch), over the
network (real-time), or embedded in experiment code (API). The GUI/builder only
*authors these config files*; it is not the runtime.

## 2. Config taxonomy

```
vehicle.yaml ─┐
tire.yaml ────┤
sensor_suite ─┤──▶ scenario.yaml ──▶ [run mode] ──▶ results
map.yaml ─────┤        ▲
comms.yaml ───┘        └─ references the others by name
```

| config | owns | notes vs today |
|---|---|---|
| `vehicles/*` | level (L1/L2/L3) + full spec (제원) | exists |
| `tires/*` | tire model + params | exists |
| `sensors/*` | a **suite**: per sensor `{id, type, host, mount[x,y,z], yaw, rate, params}` | exists; **add `host`** = which vehicle id or `infra:<name>` (sensor on a car vs roadside) |
| `maps/*` | which map + **driving path** (centerline) | exists; the path is the reference line → enables **CTE / lap-time** performance eval |
| `scenarios/*` (experiments) | composes vehicle+tire+sensors+map+maneuver+duration | exists; **add `comms:`** ref + run-mode |
| `comms/*` | **data routing** (sources → destinations) | **NEW** (see §4) |

## 3. Sensor & map refinements

- **Sensor host**: `host: ego` (a vehicle id) or `host: infra:cam_tower_1` (fixed
  roadside). Mount pose is relative to the host frame. Lets one suite mix on-board
  and infrastructure sensors.
- **Map path → performance eval**: the driving line is the reference for
  cross-track error (CTE), heading error, lap time. The batch metrics read it.

## 4. Communication config (NEW)

Declares, per run, what data flows where. A **source** (a vehicle's state, a
vehicle's command input, or a specific sensor) maps to one or more **channels**
(transport + ip:port). The **source type fixes the packet template**.

```yaml
name: hil_setup
channels:
  - source: ego.state          # vehicle dynamics state
    template: vds1_state       # template auto-selected by source type
    transport: udp
    to: [ {ip: 10.0.0.5, port: 7002}, {ip: 10.0.0.9, port: 7100} ]   # fan-out
  - source: ego.sensor.gnss    # one sensor -> two consumers
    template: nmea_gga
    transport: udp
    to: [ {ip: 10.0.0.5, port: 9001}, {ip: 127.0.0.1, port: 9100} ]
  - source: ego.cmd            # control INTO the sim (fan-in: many senders, one port)
    direction: in
    template: vds1_cmd
    transport: udp
    listen: { port: 7001 }     # any source may send control here
```

- **fan-out**: one source → many `to` destinations (a sensor feeds perception +
  a logger).
- **fan-in**: a `direction: in` channel listens on one port; multiple controllers
  may send (last-writer / ZOH, as the current `vdsim_udp_server` already does).
- **templates** keyed by source type: `vds1_state` / `vds1_cmd` (the existing
  `cosim/cosim_protocol.hpp` VDS1 binary), plus sensor templates (NMEA for GNSS,
  raw IMU struct, etc.). New source type → new template; the config never hand-
  codes byte layout.

This formalizes today's live, per-vehicle `Port.targets` (`/api/io/targets`) into a
saved, multi-source routing file.

## 5. Execution modes (same scenario, two ways to drive it)

- **A. Real-time comms** (real-vehicle-equivalent; current `vdsim_udp_server` /
  GUI data port): the sim free-runs; data flows per `comms.yaml`; control arrives
  over UDP (ZOH-latched, fail-safe on timeout). Batch can run this when the loop
  involves external SIL/HIL nodes.
- **B. Embedded API** (synchronous, no network; for experiment code / batch):
  ```python
  sim = vdsim.Simulation("scenarios/yongin_lap.yaml")
  while not sim.done():
      sim.set_control(steer=.., throttle=.., brake=..)   # or an autopilot
      sim.step(dt)
      data = sim.get_data("gnss")        # per-sensor measurement
  ```
  This wraps the existing `make_sim_session` + `set_input`/`tick`/`output` behind
  a scenario-loading facade; deterministic, fast, no comms config needed.

The scenario declares `run: {mode: api | rt_comms, comms: hil_setup}`. Batch
(#BATCH_RUNNER) picks the mode per run; control may differ between batch and RT.

## 6. Roles

- **GUI / builder** — authors vehicle/sensor/map/scenario/comms config files (the
  builder gets a Comms tab and sensor `host`/map-path editing).
- **CLI / batch** — loads a scenario (+comms) and runs it headless (API or RT).
- **API** — `vdsim.Simulation` facade embedded in experiment code.

## 7. Exists vs new

- Exists: vehicle/tire/sensor/map/scenario configs (builder #141), VDS1 UDP
  template + RT server + per-vehicle fan-out targets, `vdsim_lab.Experiment`.
- New: `comms/*` config + a router that realizes it; sensor `host`; map-path CTE
  metrics; the `vdsim.Simulation` API facade; run-mode in the scenario.

## 8. Open decisions

1. **comms config** as a separate file referenced by scenario (recommended), vs
   inlined in the scenario? Separate = reusable HIL rig across scenarios.
2. **Sensor templates**: ship a few standard ones (VDS1 state/cmd, NMEA GNSS, raw
   IMU) and let users add; agree on starting set?
3. **API facade `vdsim.Simulation`** — build it now (it mostly wraps existing
   calls + scenario load + per-sensor getData) as the batch's default mode?
4. Build order: (a) `vdsim.Simulation` API + scenario run-mode, (b) map-path CTE
   metrics, (c) comms config + router, (d) builder Comms tab. Agree?
