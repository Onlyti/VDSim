# Structured agent review process (VDSim)

Status: **2026-06-21** · thesis + main shared

## Goal

Detect bugs before merge, prove validation with **non-code evidence**, and keep
implementations minimal after review.

## Parallel review lanes (3 agents minimum)

| Lane | Scope | Self-review hook |
|------|--------|------------------|
| **A — Validation** | ISO/Chrono parity gates | `ctest -R 'ChronoKcParity|IsoBaseline'` |
| **B — Core session** | `make_direct_control_session`, CmdL1, `TireSetup`, friction ground | `ctest -R 'PerWheel|PerAxle|VlaPlant|SimSession'` |
| **C — Catalog/GUI** | `assembly.py`, `part_cards.py`, `catalog_bridge.py` | `ctest -R 'catalog|assembly|blueprint|multi_vehicle'` |

Each lane returns the same template:

1. **BUGS** — severity, location, fix
2. **SIMPLIFICATION** — dead code / over-abstraction
3. **VERIFICATION_MATRIX** — feature → test → pass/fail
4. **GAPS** — features with no structural test

## Structural verification rule

Every shipped feature must map to **at least one** automated check:

- C++ gtest (`ctest -R Name`)
- Python script under `tests/scripts/` (wired in CTest)
- Parity gate (Chrono / CarMaker) with committed reference

No check → either add test or mark **GAP** in the review report.

## Deliverables (per review round)

| Artifact | Path |
|----------|------|
| Synthesis report | `docs/evidence/review/REVIEW_YYYY-MM-DD.md` |
| Machine log | `docs/evidence/review/YYYY-MM-DD/ctest_summary.txt` |
| Feature matrix | `docs/evidence/review/YYYY-MM-DD/verification_matrix.json` |
| Process spec | this file |

Generate bundle:

```bash
python3 tools/review_evidence_bundle.py --tag thesis-2026-06-21
```

## Post-code simplification pass

After bug fixes:

1. Remove dead symbols flagged by review
2. Collapse duplicate helpers (same file only; no cross-module abstraction unless ≥3 call sites)
3. Re-run lane tests + full `ctest`
4. Update `docs/HANDOFF.md` with review date and test count

## Acceptance

- All lane tests **PASS**
- Full `ctest` green (388 on `main`, 402 on `VDSim-Thesis`)
- Review report committed or attached to PR
- No ISO/benchmark number re-baseline unless explicitly approved
