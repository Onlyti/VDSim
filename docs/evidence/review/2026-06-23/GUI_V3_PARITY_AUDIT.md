# GUI v3 parity audit — 2026-06-23

Automated static audit (no browser). P0 gaps: **0**

| Status | Count |
|--------|-------|
| PASS | 42 |
| PARTIAL | 2 |
| GAP | 4 |

| ID | Prio | Status | Note |
|----|------|--------|------|
| SH-01 | P0 | PASS | hash routes scenario/run/build/analyze |
| SH-02 | P0 | PASS | scenario save/load in ScenarioMode |
| SH-03 | P0 | PASS | Play → run |
| SH-04 | P0 | PARTIAL | kin warnings in Run status only |
| SH-05 | P0 | PASS | mkdocs help links |
| SH-06 | P1 | GAP | panel resize persist |
| CO-00 | P0 | PASS | template gallery |
| CO-01 | P0 | PASS | road μ/grade/bank |
| CO-02 | P1 | GAP | OpenDRIVE xodr |
| CO-03 | P1 | GAP | terrain obj |
| CO-04 | P0 | PASS | fleet table |
| CO-05 | P0 | PASS | map drag |
| CO-06 | P0 | PASS | path preset + wp table |
| CO-07 | P0 | PASS | path edit |
| CO-08 | P0 | PASS | static map path; Run viewport separate |
| CO-09 | P1 | GAP | cosim attach UI |
| CO-14 | P1 | PARTIAL | scenario list from API |
| CO-15 | P0 | PASS | build?vid= |
| CO-16 | P0 | PASS | save/load scenario |
| BU-01 | P0 | PASS | blueprint presets |
| BU-02 | P0 | PASS | categories |
| BU-03 | P0 | PASS | preview Δ + install |
| BU-04 | P0 | PASS | stats panel |
| BU-05 | P0 | PASS | build_complete surfaced |
| BU-06 | P0 | PASS | L1–L5 |
| BU-07 | P1 | PASS | compat badges |
| BU-08 | P0 | PASS | blueprint confirm |
| BU-09 | P1 | PASS | export blueprint |
| BU-10 | P0 | PASS | linkage SVG |
| BU-11 | P0 | PASS | no Three.js in BuildMode |
| BU-13 | P2 | PASS | K&C in HardpointEditor |
| BU-14 | P1 | PASS | PartsLibrary |
| BU-HP | P0 | PASS | HardpointEditor |
| RN-00 | P0 | PASS | Run mode route |
| RN-01 | P0 | PASS | stop/reset |
| RN-02 | P0 | PASS | autopilot/manual + manbar |
| RN-03 | P0 | PASS | time scale |
| RN-04 | P0 | PASS | cam modes |
| RN-05 | P0 | PASS | SSE viewport |
| RN-06 | P1 | PASS | wheel force triad |
| RN-07 | P0 | PASS | telemetry |
| AN-01 | P0 | PASS | fleet/catalog picks |
| AN-02 | P0 | PASS | maneuvers |
| AN-03 | P0 | PASS | yaw chart |
| AN-04 | P0 | PASS | delta table |
| AN-05 | P1 | PASS | CSV export |
| AN-06 | P0 | PASS | separate Analyze mode route |
| NF-BUILD | P0 | PASS | v3 dist present |

Regenerate: `python3 tools/gui_v3_parity_audit.py`
API smoke: `python3 tests/scripts/test_gui_v3_api_smoke.py`
