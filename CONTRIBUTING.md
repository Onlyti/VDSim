# Contributing to VDSim

VDSim is v0.1.0 (experimental / pre-release). Issues, reproductions, and PRs are
welcome — especially validation cases and tire/vehicle data that can be shared
openly.

## Build & test

```bash
cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build -j
cd build && ctest --output-on-failure        # 187 checks, must stay 100% green
```
Python package: `pip install ".[plot]"` (Python ≥ 3.10 with a modern `pip`).
CI (GitHub Actions) builds gcc-9 + clang-10 and runs ctest on every push/PR.

## Conventions (non-negotiable — they keep results comparable)

- Frame: ISO 8855 right-handed. Wheel index order **FL=0, FR=1, RL=2, RR=3**.
- C++17; parameters are YAML-based; keep new configs in the same form.
- A change to physics must come with a test (analytic / ISO / cross-model
  self-consistency) and must not regress the suite.
- Don't commit confidential or measured tire/vehicle data; presets only.

## Issues

Include: what you ran (command + config), expected vs actual, OS / compiler /
Python versions, and a minimal reproduction. For a dynamics discrepancy, attach
the CSV or the failing `ctest -R <name>` output.

## Pull requests

1. One focused change per PR; describe the why.
2. `ctest` green locally + CI green.
3. New behavior documented (docs/ or the relevant theory chapter) and, if it
   changes a validated number, update `docs/VALIDATION.md`.
4. No new wire-protocol/format change without a version bump and a spec update.

## License

By contributing you agree your contributions are licensed under Apache-2.0
(see `LICENSE`).
