# GUI v3 — visual review vs concept art

Date: **2026-06-22**  
Method: Playwright headless user flow + `gui/server.py` (ephemeral port)  
Captures: [`gui-captures/`](gui-captures/)  
Concept art: [`docs/design/assets/`](../../design/assets/README.md)

## Executive verdict (cold)

| Lens | Score | Comment |
|------|-------|---------|
| vs **concept art** (target UX) | **4/10** | IA matches; visual storytelling and “studio” density do not |
| vs **v1 external review** (~4/10 prototype) | **6/10** | Mode shell + no modals is real progress; still reads wireframe |
| **Analyze** (no dedicated concept PNG) | **7/10** | Yaw overlay + Δ% table deliver the validation story |
| **Concept art as docs** | **5/10** | Good north-star; risk of overselling if shown without “not implemented” labels |

**Bottom line:** Concept illustrations are **not** adequately matched by the current GUI for onboarding or thesis figures. They work as **aspirational** art only. For honest communication, pair each concept PNG with a **live capture** side-by-side or mark concept as “target”.

---

## Capture inventory

| File | Flow step |
|------|-----------|
| `01-scenario-l4.png` | Scenario, `l4_sedan_kinematics` loaded |
| `02-build-default.png` | Build, L4, body slot |
| `03-build-l4-hardpoint.png` | Build, front chassis + HP editor |
| `04-analyze-idle.png` | Analyze, vehicle picks |
| `05-analyze-results.png` | Compare: sedan_l3 vs fsk_formula, step-steer chart |
| `06-run-sim-t3.png` | Run after ▶ Play (~3 s sim) |
| `07-run-sim-t8.png` | Run ~8 s sim |
| `06-run-idle.png` | (legacy) idle before Play fix |

Regenerate: `python3 tools/gui_v3_capture.py`

---

## Mode-by-mode vs concept art

### Scenario — `vdsim-v3-scenario-studio-concept.png`

| Concept shows | Implementation shows | Gap |
|---------------|---------------------|-----|
| Figure-8 **drawn on map**, numbered waypoints 1–6 | Path line + 160 pts; edit mode shows wp # | **Improved** (was P0 empty) |
| Grid axes −80…80 m, compass ISO 8855 | Grid + **axis tick labels (m)** | Medium (no compass) |
| Floating map toolbar (select, draw, waypoint) | μ/grade/bank/v + “Edit path” only | Medium |
| Waypoint **table** (x, y, heading, speed) | **Downsampled x,y table** + map edit | Medium (no heading/speed cols) |
| Two agents on track | One agent; map marker minimal | Small |

**Qualitative:** A new user cannot infer “scenario studio” from the capture. The concept art **does** explain the intended workflow; the app **does not** yet illustrate path/fleet composition visually.

---

### Run — `vdsim-v3-run-mode-concept.png`

| Concept shows | Implementation shows | Gap |
|---------------|---------------------|-----|
| Chase camera on **mesh** sedan, track environment | Idle: **preview mesh** at spawn; sim captures show chase | Medium (RN-06 forces still GAP) |
| Per-wheel **force arrows** (Fx/Fy/Fz) | **Fx/Fy/Fz triad** at wheels when sim running | **Closed** (RN-06) |
| Telemetry: pose, velocity, **per-wheel slip table** | Pose + **FL–RR table** (Fz, Fx, Fy, κ, α°) + sim **t** | Medium (no timeline scrub) |
| Timeline scrubber, 1.0x transport | Stop/Reset + time scale; **no timeline bar** | Medium |
| “Running” + sim time | Capture is pre-Play (`06-run-idle.png`) | Test gap |

**Qualitative:** Concept sells “runtime engineering viewer.” Current Run is a **credible layout** but the illustration layer (car + forces + environment) is missing — the screenshot looks **empty**, not “simulation.”

---

### Build — `vdsim-v3-build-parts-assembly-concept.png`

| Concept shows | Implementation shows | Gap |
|---------------|---------------------|-----|
| **L3 7-DOF** annotated schematic (masses, springs, dims) | Block diagram + **minimal y–z linkage SVG** | Medium–large |
| Breadcrumb `Scenario > Agent 0` | Vehicle build header + V0 chip only | Small |
| Part cards with **tire images**, COMPATIBLE badges | Text cards, tier label; compat via border tone | Medium |
| Hover **Δ mass** on cards | Preview Δ in stats panel (good) | OK |
| `build_complete: true` footer strip | Incomplete warning only when false | Small |

**Qualitative:** **Best structural match** among modes — three columns, slots, schematic, stats. Illustrations are **schematic-minimal** vs concept’s textbook diagram; still defensible for L4 **if** linkage SVG is labeled as side-view kinematics.

---

### L4 hardpoints — `vdsim-v3-l4-hardpoint-editor-concept.png`, pick-mode PNG

| Concept shows | Implementation shows | Gap |
|---------------|---------------------|-----|
| Full DW **side view** with dimension Δx/Δz | Line segments + draggable dots; no dims on canvas | Medium |
| Hardpoint **tree** (lca/uca/…) | Flat pick list (short names) | Small |
| Status: “18 DOF valid”, loop closure | Not surfaced in UI | Medium |
| Live K&C with “native solver” badge | Two small K&C charts — **functionally present** | **Smallest gap** |
| Revert / Save YAML header actions | Save & install only | Small |

**Qualitative:** Only area where **function ≈ concept**. Visual polish is far below mockup; for thesis/docs, **live HP screenshot is more honest than concept art** here.

---

### Build drawer from Scenario — `vdsim-v3-build-drawer-from-scenario-concept.png`

| Concept | Implementation |
|---------|----------------|
| Drawer overlay on map, edit parts in context | **Not implemented** — `Edit parts →` navigates to `#/build` full page |

**Qualitative:** Concept art describes a UX that **does not exist**; do not use this PNG without caption “future”.

---

### Analyze (no concept PNG)

Capture `05-analyze-results.png`: fleet + catalog picks, maneuver toggles, yaw-rate overlay, Δ% table — **matches requirements AN-01–AN-04** well. This mode needs its own concept art or reuse compare dashboard from v1; current UI is **self-explanatory** without illustration.

---

## Are illustrations “sufficient” for explanation?

| Audience | Concept art alone | Live GUI today | Recommendation |
|----------|-------------------|----------------|----------------|
| Thesis committee | **Insufficient** (oversells) | **Partial** (Analyze/Build OK; Scenario/Run weak) | Side-by-side **concept | capture | gap** figure |
| New OSS user | Misleading for map/Run | Needs tooltips + path on map | Fix path render; add one annotated screenshot tour |
| Internal team | Good north star | Good IA checklist | Keep concepts in `docs/design/assets/` with README disclaimer |

---

## P0 fixes before using GUI in figures

1. **Scenario:** render preset path on map (`0 points · figure8` → actual polyline).
2. **Run:** ensure vehicle mesh visible after Play; capture `06-run-sim-*.png` in review bundle.
3. **Docs:** caption all concept PNGs as **“target mockup — not pixel spec”** (README already says this; enforce in mkdocs/thesis).

---

## Automation

```bash
python3 tools/gui_v3_capture.py          # screenshots
python3 tools/gui_v3_eval_bundle.py      # + API/E2E/parity
```

---

*Reviewer: automated capture + manual compare to `docs/design/assets/*.png`.*
