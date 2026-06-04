# Running VDSim

A run book for every way to drive the simulator. Setup is configuration-only
(see [SIM_CONFIG_ARCH](design/SIM_CONFIG_ARCH.md)); this page is *how to execute*.

## 0. Install

```sh
pip install .            # Python API (scikit-build-core builds the vdsim extension)
# or full C++ tree:
cmake -G Ninja -B build -DVDSIM_BUILD_PYTHON=ON && cmake --build build -j
```

## 1. Author the configs (web tool)

```sh
python3 builder/server.py --port 8200          # http://<host>:8200
```
Tabs save validated YAML: **Vehicle** → `configs/vehicles/`, **Sensors** (suite) →
`configs/sensors/`, **Map** (driving line + surface) → `configs/maps/`, **Comms**
(data routing) → `configs/comms/`, **Scenario** (composes all + run mode) →
`configs/experiments/`. (See [builder/README](../builder/README.md).)

## 2. Run an experiment — four modes (same scenario)

### A. Embedded API (synchronous, no network) — for experiment code & notebooks
```python
import sys; sys.path.insert(0, "python")
from vdsim_lab import Simulation
sim = Simulation("yongin_lap")                 # configs/experiments/yongin_lap.yaml
while not sim.done():
    sim.set_control(steer=.., throttle=.., brake=..)   # omit -> scenario autopilot
    sim.step()
    gnss = sim.get_data("gnss")                # per-sensor measurement
    gt   = sim.state()                         # ground truth (x,y,vx,Fz,slip,…)
```
Or assemble inline with the fluent builders and collect a time-series:
```python
from vdsim_lab import Experiment, Vehicle, Road, Maneuver, Sensors
res = (Experiment(level="L3").vehicle(Vehicle.preset("sedan"))
       .road(Road.preset("belgian_pave")).maneuver(Maneuver.step_steer(v=20, steer=.03))
       .sensors(Sensors().gnss().imu()).run(8.0))
res.to_csv("run.csv"); print(res.summary())
```

### B. Real-time comms (UDP; SIL/HIL nodes in the loop)
The C++ `vdsim_realtime` is the single comms layer: it runs a real-time
`SimSession`, receives CMD packets on one port, and emits STATE on another. The
wire format is the canonical VDS1 binary protocol (`cosim/cosim_protocol.hpp`,
CRC32, 76B CMD / 220B STATE).
```sh
build/bin/vdsim_realtime configs/vehicles/sedan.yaml \
    configs/tires/default_pacejka.yaml --level=L3 \
    --cmd-port=7001 --state-ip=127.0.0.1 --state-port=7002 --rate=200
```
Python participants (wheel/pedal clients, viewer bridge, HIL harnesses) encode
CMD / decode STATE through `cosim/protocol.py` — one definition of the wire
format, byte-compatible with the server. (See `cosim/test_udp_client.py`.)

### C. Batch / campaign (headless, parallel)
```sh
python3 tools/vdsim_batch.py run campaign.yaml          # run
python3 tools/vdsim_batch.py run campaign.yaml --dry    # list expanded runs
```
A campaign expands explicit scenarios + parameter `sweep` (cartesian grid) +
`monte_carlo` into runs, executes them in a process pool, and writes per-run CSV +
`summary.csv` (params + metrics). Metrics: `cte_rms, cte_max, peak_ay, max_Fz,
vmax, dist, lap_time, rms_slip`. (See [BATCH_RUNNER](design/BATCH_RUNNER.md).)

### D. Interactive GUI (real-time viewer, drive-by-feel)
```sh
python3 gui/server.py --port 8100              # browser viewer + live sim
```
- Browser: 3D view (Orbit/Chase/Top/**Cockpit**), road/terrain, telemetry; **🎮
  Wheel/pedals** (Gamepad API), **🔊 Sound** (engine/tire/wind).
- **Force feedback** (real wheel, on the client PC) — UDP to the GUI at http-port+1:
  ```sh
  python3 tools/wheel_ffb_sdl.py --server <host> --udp-port 8101   # Windows+Linux (pysdl2)
  python3 tools/wheel_ffb.py     --server <host> --udp-port 8101   # Linux evdev
  ```
  Autostart units in `tools/autostart/`. FFB torque = VDSim aligning moment
  (`rack_torque`). (See [tools/autostart/README](../tools/autostart/README.md).)

## 3. Industrial / interop

- **FMI 2.0**: build `vdsim_l2.fmu` / `vdsim_l3.fmu` (`fmi_export/build_*.sh`),
  import any CS FMU via `fmi_export/fmu_master.py`. See theory ch16.
- **CARLA bridge**: `carla_integration/` (VDSim physics ↔ CARLA render/sensors).
- **ISO maneuver validation**: `python3 apps/validation/run_validation.py` →
  ISO 7401/4138/3888-2 metrics + plots. Credibility: [VALIDATION](VALIDATION.md).

## 4. Which mode?

| want | mode |
|---|---|
| drive a controller/estimator from your own code | **A. API** |
| connect external SIL/HIL/perception over the network | **B. UDP comms** |
| sweep params / Monte Carlo / regression metrics | **C. batch** |
| watch it, or drive with a wheel + FFB | **D. GUI** |
| plug VDSim physics into a co-sim tool | **FMI** |
