# Chrono Pac02 parity gate

Independent cross-check of VDSim's MF2002 (`.tir`) tire evaluator against
[Project Chrono](https://projectchrono.org/)'s `ChPac02Tire` (BSD-3). Confirms our
Pacejka-2002 forces match a third-party implementation **without taking Chrono as a
dependency**.

## Isolation (why nothing mixes)

```
external/chrono_parity/        <- Chrono touched ONLY here, in a separate conda env
  sample_pac02.tir             <- public synthetic coeffs, fed to BOTH evaluators
  gen_pac02_reference.py       <- imports pychrono, writes reference/pac02_reference.csv
  reference/pac02_reference.csv<- the only artifact that crosses back into the repo
tests/parity/                  <- VDSim side: reads the CSV, NO Chrono link
  test_chrono_pac02_parity.cpp
```

- VDSim's CMake / `libvdsim_core` never `find_package(Chrono)` and never link it.
- The C++ gate (`ctest -R ChronoPac02Parity`) reads the CSV; if the CSV is absent it
  **SKIPs** (never fails the build for a missing external artifact).
- `sample_pac02.tir` is the single shared input. The repo `.gitignore` blocks `*.tir`
  (measured coeffs are confidential); this one file has an explicit allow exception
  because it is public synthetic data.

## Gate states

| State | `OurSideLoadsSampleTir` | `ForcesMatchReference` |
|-------|-------------------------|------------------------|
| no CSV (default)         | PASS (our evaluator loads the shared .tir) | SKIP |
| CSV present, in band     | PASS | PASS |
| CSV present, drift       | PASS | FAIL (prints worst Fx/Fy points) |

## Generating the reference

The generator needs a pychrono with the **file-driven** Pac02 tire (`ReadTireJSON` /
`Pac02Tire(json)`), i.e. **pychrono >= 8.0**.

```bash
conda create -n chrono8 -c projectchrono -c conda-forge pychrono python=3.10
conda activate chrono8
python external/chrono_parity/gen_pac02_reference.py
# -> external/chrono_parity/reference/pac02_reference.csv
cd build && ctest -R ChronoPac02Parity --output-on-failure
```

### Known environment blocker (this host)

- **Ubuntu 20.04 = GLIBC 2.31.** The pychrono 8.0 conda binary requires GLIBC 2.32+
  (`_core.so: version GLIBC_2.32 not found`), so it does **not** run here.
- **pychrono 7.0** (conda-forge) *does* run on 20.04, but exposes only the abstract
  `ChPac02Tire` with no file loader (`ReadTireJSON` absent) — it cannot consume an
  arbitrary `.tir`, so it is not usable for shared-`.tir` parity.

Therefore generate the CSV on one of:
1. a host with GLIBC >= 2.32 (Ubuntu 22.04+) running pychrono >= 8, or
2. a local **Chrono source build** (its libs match the host GLIBC) — write the same
   grid with the C++ `Pac02Tire(json)` API and emit the CSV.

The CSV is committed once generated, so the gate then runs everywhere from the
checked-in artifact (no Chrono needed to *run* the gate, only to *regenerate* it).

## Caveats when first comparing

- **MF revision / scaling**: ensure both sides use Pacejka-2002 with the same scaling
  factors (the `.tir` `[SCALING_COEFFICIENTS]` are all 1.0 here). A mismatch shows up
  as a systematic bias, not noise — do not mistake it for a VDSim bug.
- **Sign convention**: the generator emits forces in the ISO tire frame to match
  VDSim (`Fy < 0` for `alpha > 0`). Flip in the generator, not the gate.
- **Mz**: aligning-moment / pneumatic-trail models differ more across MF
  implementations; the gate compares Fx/Fy and only records Mz.
