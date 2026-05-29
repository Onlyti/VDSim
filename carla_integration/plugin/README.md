# VDSim CARLA plugin (skeleton)

This directory hosts the bridge between `vdsim_core` and a CARLA / UE4 host
process. The PoC ships the bridge as an offline-buildable C++17 skeleton
without requiring CARLA at build time. When integrated into UE4 the same
sources compile against the engine, with the only addition being the actual
raycast call backing `query()` and an actor-state read/write loop.

Layout:
- `raycast_contact_provider.{hpp,cpp}` — implements `IContactProvider`. A function
  pointer is injected at construction time for the raycast itself, so the same
  code works in CARLA, in unit tests (mock callback), and in CARLA-less builds.
- `tick_bridge.{hpp,cpp}` — wraps a `vdsim::IVehicleDynamics` instance, runs a
  fixed-rate dynamics step at every CARLA tick, and maps the CARLA actor's
  6-DOF pose / vehicle command back and forth.

This level of detail is enough to validate the ABI surface; the actual UE4
hookup lives in a separate (closed) workspace.
