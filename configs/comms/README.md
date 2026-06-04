# Communication configs

Declarative routing spec for the UDP comms layer: each channel maps a **source**
(`ego.state`, `ego.sensor.<id>`) to **destinations** (fan-out), or listens
(`direction: in`) for control (fan-in). The authoring builder (Comms tab) writes
these; schema in docs/design/SIM_CONFIG_ARCH.md.

Realization: the single comms layer is the C++ `vdsim_realtime` (see
docs/RUNNING.md §B), which speaks the canonical VDS1 binary protocol
(`cosim/cosim_protocol.hpp`). Today it serves one CMD-in port and one STATE-out
destination via CLI flags; config-driven multi-destination fan-out is a planned
extension (see docs/design/RUNTIME_ARCH.md). Python participants use
`cosim/protocol.py` to stay byte-compatible.
