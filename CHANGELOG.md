# Changelog

All notable changes to VDSim are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/); versions follow
[SemVer](https://semver.org/).

## [0.1.0] — first public release

Experimental / pre-release. Validated on analytic + ISO standard +
cross-model/cross-tool self-consistency evidence (see `docs/VALIDATION.md`); not
yet cross-validated against a commercial reference on real-vehicle data.

### Vehicle dynamics (C++ core, ISO 8855 RH, wheel order FL=0,FR=1,RL=2,RR=3)
- Ladder models: Lk kinematic, Ld1 bicycle, Ld2 7-DOF, Ld3 14-DOF.
- Pacejka MF96 tire (load sensitivity, relaxation length, camber thrust/Mz,
  combined-slip friction ellipse) + linear fallback.
- Ld4 hardpoint kinematics: double-wishbone / MacPherson / trailing-arm /
  5-link, lookup + native solvers; Adams CSV importer.
- Contact providers: flat, split-mu, inclined, two-tone rough, ISO 8608 PSD,
  heightmap terrain.
- Drivetrain (FWD/RWD/AWD, open/locked/LSD diff, final drive), brake bias/EBD,
  aero, anti-roll bars, road slope/bank load transfer + jacking.

### Tooling & interfaces
- Python API (`vdsim` pybind module) + fluent experiment layer (`vdsim_lab`):
  Vehicle / Tire / Road / Maneuver / Sensors builders, metrics, CSV/TUM logging.
- Batch / campaign runner (sweep + Monte Carlo) → summary CSV.
- Sensor models (GNSS/INS/IMU/wheel-speed/steer, noise+bias+delay, mount-pose).
- Operating-point linearization (A,B export); in-loop observer slot.
- Real-time UDP runtime (`vdsim_udp_server`, VDS1 binary protocol) for
  SIL/HIL/co-sim; Python protocol mirror.
- FMI 2.0 Co-Simulation export (L2/L3 FMUs) + import of any CS FMU.
- Web viewer (Three.js PoC) + experiment authoring builder + suspension editor.
- CARLA bridge (VDSim physics ↔ CARLA render/sensors).
- ISO 7401 / 4138 / 3888-2 validation maneuvers; DoE runner.

### Validation
- 187/187 ctest green; FMI round-trip Δ=0; ISO 8608 PSD RMS within tolerance per
  road class. See `docs/VALIDATION.md` for the benchmark matrix and honest
  limitations.

[0.1.0]: https://github.com/Onlyti/VDSim/releases/tag/v0.1.0
