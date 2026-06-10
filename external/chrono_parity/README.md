# Chrono Pac02 parity gate

Independent cross-check of VDSim's MF2002 (`.tir`) tire evaluator against
[Project Chrono](https://projectchrono.org/)'s `ChPac02Tire` (BSD-3). Confirms our
Pacejka-2002 forces match a third-party implementation **without taking Chrono as a
dependency**.

## Isolation (why nothing mixes)

```
external/chrono_parity/        <- Chrono touched ONLY here, built separately
  sample_pac02.tir             <- public synthetic coeffs, fed to BOTH evaluators
  gen_reference.cpp            <- links Chrono; ChTireTestRig sweep -> reference CSV
  CMakeLists.txt               <- standalone; find_package(Chrono), NOT VDSim's tree
  reference/pac02_reference.csv<- the only artifact that crosses back into the repo
tests/parity/                  <- VDSim side: reads the CSV, NO Chrono link
  test_chrono_pac02_parity.cpp
```

- VDSim's CMake / `libvdsim_core` never `find_package(Chrono)` and never link it.
- The C++ gate (`ctest -R ChronoPac02Parity`) reads the CSV; if the CSV is absent it
  **SKIPs** (never fails the build for a missing external artifact).
- `sample_pac02.tir` is the single shared input. The generator parses it and emits the
  equivalent Chrono Pac02 JSON (Chrono inlines coeffs in JSON), so the coefficients are
  identical on both sides — no transcription. The repo `.gitignore` blocks `*.tir`
  (measured coeffs are confidential); this one public-synthetic file has an exception.

## Result (committed reference)

| Regime | VDSim vs Chrono Pac02 |
|--------|------------------------|
| Pure longitudinal slip, Fz 2–6 kN | **within ~2%** (load sensitivity + long backbone agree) — *gated* |
| Near-pure lateral | within ~6% |
| Strong combined slip (large κ *and* α) | diverges (mean \|ΔFx\|≈39%, \|ΔFy\|≈17%) — different combined-slip weighting; *reported, not gated* |

So the gate `PureLongitudinalMatchesChrono` is the validated cross-check (and a
regression lock); `CombinedSlipReported` records the combined-slip divergence and only
guards against a gross regression (sign flip / >3×). The combined-slip weighting
difference is a known MF-variant gap, not a bug — investigating whether to align
VDSim's `Gxa`/`Gyk` with Pac02 is a separate task.

## Regenerating the reference (needs a Chrono build)

Chrono's file-driven Pac02 (`ReadTireJSON`) is only exposed in the C++ API and in
pychrono ≥ 8 (whose conda binary needs GLIBC ≥ 2.32 — it does not run on Ubuntu 20.04).
On 20.04 / GLIBC 2.31, build Chrono from source (its libs match the host GLIBC):

```bash
# 1. Build Chrono (vehicle module only, minimal) — done once, outside the repo.
git clone --depth 1 --branch 8.0.0 https://github.com/projectchrono/chrono.git ~/build_ext/chrono_src
cmake -S ~/build_ext/chrono_src -B ~/build_ext/chrono_src/build -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_MODULE_VEHICLE=ON -DENABLE_MODULE_IRRLICHT=OFF -DENABLE_MODULE_POSTPROCESS=OFF \
  -DENABLE_MODULE_PYTHON=OFF -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3
cmake --build ~/build_ext/chrono_src/build --target ChronoEngine ChronoEngine_vehicle -j

# 2. Build + run the generator (links that Chrono build).
cd external/chrono_parity
cmake -B build -DChrono_DIR=~/build_ext/chrono_src/build/cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
LD_LIBRARY_PATH=~/build_ext/chrono_src/build/lib ./build/gen_reference "$(pwd)"
#   -> reference/pac02_reference.csv  (commit it)

# 3. Run the gate from VDSim.
cd ../.. && cmake --build build -j && (cd build && ctest -R ChronoPac02Parity --output-on-failure)
```

The CSV is committed, so the gate runs everywhere from the checked-in artifact — Chrono
is only needed to *regenerate* it, never to *run* the gate.

### Notes / caveats baked into the generator

- **Use the actual slip/load Chrono reports**, not the commanded grid: `ChTireTestRig`'s
  commanded longitudinal slip differs from Pac02's internal slip (effective rolling
  radius), and the rig's contact Fz ripples; the generator records `GetLongitudinalSlip`,
  `GetSlipAngle`, and the in-contact mean Fz so both models are evaluated at the same point.
- **Frame**: forces are rotated from Chrono's global frame into the wheel/ISO frame
  (`Fy < 0` for `alpha > 0`) to match VDSim.
- **Vertical damping** in `sample_pac02.tir` is set high (8000) only so the rig's vertical
  mode settles; VDSim's evaluator takes Fz directly and ignores it.
- **Mz** (aligning) differs more across MF implementations; the gate compares Fx/Fy only.
