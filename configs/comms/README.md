# Communication configs

Data-routing for real-time-comms runs (realized by `python/vdsim_comms.py`).
A scenario references one via `run: {mode: rt_comms, comms: <name>}`.

Each channel maps a **source** (`ego.state`, `ego.sensor.<id>`) to a **template**
and **destinations** (fan-out), or listens (`direction: in`) for control on one
port (fan-in). Templates: `json`, `vds1_state`, `vds1_cmd`, `nmea_gga`, `imu_raw`.
Schema: see builder/README.md + docs/design/SIM_CONFIG_ARCH.md.
