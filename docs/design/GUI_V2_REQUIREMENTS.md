# GUI v2 — requirements catalog (restart baseline)

Status: **superseded for product direction** by [`GUI_V3_REQUIREMENTS.md`](GUI_V3_REQUIREMENTS.md) (2026-06-21).  
Keep §3.1 as the **v1 parity checklist** for v3 launch gate.  
v2 incremental `/v2/app.html` plan is **cancelled**.  
Wire contract: [`../gui_architecture.md`](../gui_architecture.md) (REST + SSE, unchanged)  
Catalog model: [`PARTS_CATALOG.md`](PARTS_CATALOG.md)

---

## 0. Restart principles

| # | Principle |
|---|-----------|
| R0 | **Not game UI** — Notion/Linear-like light OSS; teaching + engineering credibility |
| R1 | **Wire contract frozen** — frontend depends on REST/SSE schema only, not Python internals |
| R2 | **Three layers stay separate** — (1) vehicle build, (2) experiment/scene, (3) compare/validation |
| R3 | **Incremental migration** — v2 surfaces behind flag/route; v1 `app.js` runs until parity |
| R4 | **Offline-first** — self-host fonts/icons (`gui/vendor/`); no CDN hard dependency |
| R5 | **mkdocs help** — `[?]` on non-obvious labels; no long prose in modals |
| R6 | **No physics in browser** — server resolves catalog → params; GUI edits manifests + session overrides |

---

## 1. Product positioning

| Item | Requirement |
|------|-------------|
| Role | Browser **authoring + Play** for VDSim plant; not CAD, not tire coefficient workshop |
| Differentiator | Compact physics state → local Three.js reconstruction (not video stream) |
| Audience | Maintainer, external evaluators, Formula Student, autonomous/ADAS students |
| Success | ≤5 clicks from fleet → built vehicle → run or compare; citeable ISO numbers |

---

## 2. Personas & jobs-to-be-done

| Persona | Primary job | v2 must support |
|---------|-------------|-----------------|
| **Maintainer** | Ship credible demos without wire churn | Stable API, fast iteration, ctest-backed catalog |
| **External customer** | Evaluate plant without install | Browser-only, export YAML, clear limits documented |
| **Formula Student** | Team car spec → step-steer/skidpad numbers | Build-sheet → compare modal → CSV/metrics |
| **Autonomous student** | Realistic plant + cosim | Fleet + UDP targets, repeatable scenarios |

---

## 3. Information architecture — **three composition layers** (v1 already has all three)

v1 is **not** “build-sheet only”. The GUI already separates:

| Layer | Korean (팀 내부) | VDSim object | v1 surface | v2 doc gap (fixed below) |
|-------|------------------|--------------|------------|---------------------------|
| **A. Vehicle build** | 차량 구성 | `blueprint` + catalog `parts` | Modal → **Assembly** | Was over-emphasized as sole MVP |
| **B. Run / scene compose** | 지도·실험 구성 | `scene.yaml`: map, path, fleet spawn, road, comms | **Setup bar** (4 tabs) + sidebar fleet | Was “v2.1 carry-over” one-liner only |
| **C. Analyze** | 검증·비교 | ISO maneuver on N manifests | **Compare modal** (⊟) | Listed but not inventory’d |

```
┌──────────────────────────────────────────────────────────────────┐
│ SIDEBAR          │  MAIN VIEWPORT          │  TELEMETRY (run)   │
│ · Sim transport  │  · Three.js scene       │  · pose / tire /   │
│ · Fleet tree     │  · path edit (top view) │    controls        │
│ · Infra sensors  │  · spawn drag           │                    │
│ · Minimap        │  · stunt/terrain mesh   │                    │
├──────────────────┴─────────────────────────┴────────────────────┤
│ SETUP BAR — Run composition (layer B)                              │
│  [Map] [Vehicle] [Options] [Modules]  · scenario load/save        │
│  · Road μ/grade/bank  · xodr / terrain  · fleet spawn rows       │
│  · Path preset / waypoints  · cosim UDP  · IO targets  · C++ MW   │
├──────────────────────────────────────────────────────────────────┤
│ MODALS                                                             │
│  · Vehicle Edit → Assembly | Parts lib | Session tune (layer A)   │
│  · Compare ISO metrics (layer C)                                   │
└──────────────────────────────────────────────────────────────────┘
```

| Surface | In v2.0 new shell? | Notes |
|---------|-------------------|-------|
| **Build-sheet** (Assembly) | **Yes — rewrite** | Danawa rows + schematic preview |
| **Setup bar** (Map/Vehicle/Options/Modules) | **No — v1 embed or link** until v2.1 | Full feature list §3.1 |
| **Play viewport + minimap** | **No — v1** until v2.2 | Regression = unacceptable |
| **Compare modal** | **No — v1** | |
| **Session tune tabs** | **No — v1** | Chassis/tire/K&C plots |

---

## 3.1 v1 implemented inventory (baseline — **parity checklist**)

Everything below exists in `gui/app.html` + `gui/static/app.js` today. v2 restart **must not drop** these without an explicit deprecation entry.

### A. Vehicle build (차량 구성)

| ID | Feature | UI location | API / backend |
|----|---------|-------------|---------------|
| V-01 | Fleet vehicle **+** add from blueprint preset | Sidebar Vehicles | `POST /api/setup` fleet_add |
| V-02 | Per-vehicle **Edit** modal | Tree → Edit | `openModal(vid)` |
| V-03 | **Assembly** build-sheet (Danawa rows, categories) | Modal Assembly | `/api/catalog/assembly` |
| V-04 | Part cards + install + resolved stats | Assembly center/right | `assembly.py` resolve |
| V-05 | Blueprint / Level selectors + recommended chips | Assembly top | `POST /api/setup` |
| V-06 | **Parts library** browse/import | Modal Parts | `/api/catalog`, custom kin |
| V-07 | **Session tune**: chassis, suspension, brake, steer, drivetrain, tire | Modal tune dropdown | `/api/vehicle`, `/api/tire`, fields |
| V-08 | Suspension **K&C plots** (L3/L4) | Suspension tab | `/api/suspension/kc` |
| V-09 | Actuator + **Sensors** workshop | Modal tabs | `/api/actuator`, `/api/sensors` |
| V-10 | Export / register blueprint | Assembly actions | export API |
| V-11 | Per-vehicle level, vehicle stem, tire, L3 susp stems | Fleet + modal | fleet spec normalize |

### B. Run / scene composition (지도·실험·플릿 구성)

| ID | Feature | UI location | API / backend |
|----|---------|-------------|---------------|
| S-01 | **Scenario** load / save / export / import | Setup bar header | `/api/scene`, save modal |
| S-02 | Draft sync badge + Play → `runs/live/` | Setup status | `POST /api/setup`, `/api/control` |
| S-03 | **Map tab**: road μ, grade°, bank°, v_target | Setup → Map | `setup.road`, `v_target` |
| S-04 | **OpenDRIVE** `.xodr` load | Setup → Map | `/api/map/load` |
| S-05 | **Terrain** `.obj` load + clear | Setup → Map | `/api/terrain/load`, `/api/terrain/clear` |
| S-06 | **Vehicle tab**: fleet spawn rows (x0,y0,yaw0,vx0…) | Setup → Vehicle | fleet fields in setup |
| S-07 | **Spawn drag** in 3D top view | Viewport + hint | updates fleet pose |
| S-08 | **Path preset**: figure-8 / straight / custom | Setup → Options | `path_preset`, `path_pts` |
| S-09 | Waypoint **table** + add row | Setup → Options | `path_pts` |
| S-10 | **Path edit mode** on ground (drag ●, add, Del) | Cam bar “Edit track” | posts path to setup |
| S-11 | **Minimap** trail | Sidebar canvas | `minimap.js` + SSE |
| S-12 | **Cosim attach** host + cmd/state ports | Setup → Options | cosim start/stop |
| S-13 | **IO / telemetry targets** | Setup → Options comms | `/api/io/targets` |
| S-14 | **C++ module workshop** (build & register) | Setup → Modules | `/api/io` module check |
| S-15 | **Infra sensors** tree (authoring) | Sidebar | `infra_sensors` in setup |
| S-16 | Multi-vehicle **+ add 2nd** | Setup Vehicle | `fleet_add` |
| S-17 | **Stunt** ground viz (ramp / loop) when scene has `stunt:` | 3D overlay | scene stunt block |
| S-18 | Sim cfg load/save (sidebar) | SIM card | `/api/runconfig` |

### C. Play runtime (실행·시각화)

| ID | Feature | UI location |
|----|---------|-------------|
| P-01 | Play / Stop / Reset | Sidebar |
| P-02 | Autopilot vs **manual** drive (+ touch manbar) | Sidebar + manbar |
| P-03 | Realtime **time scale** slider | Sidebar |
| P-04 | Cam **orbit / chase / top** | Cam bar |
| P-05 | SSE ~60 Hz state → fleet meshes | `#view` Three.js |
| P-06 | Wheel **force vectors** Fx/Fy/Fz | On wheel triad |
| P-07 | Telemetry panel (pose, tire, controls) | Right aside |
| P-08 | Kinematics warnings | Sidebar `#kin_warnings` |
| P-09 | L5 stunt pose (z, roll, pitch) | telemetry + mesh |

### D. Analyze / compare (검증)

| ID | Feature | UI location | API |
|----|---------|-------------|-----|
| A-01 | **Compare vehicles** modal (⊟) | Sidebar | `/api/compare` |
| A-02 | Blueprint multi-select | Compare modal | fleet blueprints |
| A-03 | ISO maneuver picker (step-steer, …) | Compare | `vdsim_compare` |
| A-04 | Yaw-rate **overlay chart** (SVG) | Compare results | trace in response |
| A-05 | **Δ% table** vs baseline | Compare results | compare CSV logic |

### E. v1 requested / known gaps (HANDOFF — not done)

| ID | Feature | Status |
|----|---------|--------|
| G-01 | **Multi-fleet 3D** — N vehicles simultaneous render (#158) | Partial (spawn OK, demux polish) |
| G-02 | **Dockable / resizable** panels; drop monolithic modal | Design review — deferred to v2+ |
| G-03 | Global nav **Setup | Sim | Analyze** | Not implemented |
| G-04 | GUI **terrain Play** polish (chase on mesh, spawn on surface) | v0.5.2 deferred |
| G-05 | GUI **stunt authoring** (not just render) | v0.5.2 deferred |
| G-06 | In-browser **`.tir` import** | v0.5.2 deferred |
| G-07 | Belt transient **force arrow** viz fix | open |
| G-08 | **powertrain** slot in assembly (split blueprint) | catalog gap |
| G-09 | Blueprint switch **confirm** (silent parts wipe) | fix in review commit |

---

## 3.2 Why the first v2 draft felt incomplete

| You expected (v1 dev) | First `GUI_V2_REQUIREMENTS` draft | Correction |
|------------------------|-----------------------------------|------------|
| 차량 구성 (assembly) | BS-* detailed | ✓ kept |
| 지도·terrain·road 구성 | One row “EX-01” | **§3.1 S-03–S-05** |
| 경로·waypoint 구성 | One row “EX-01” | **§3.1 S-08–S-10** |
| 플릿 spawn·멀티차량 | FL-* thin | **§3.1 S-06–S-07, S-16** |
| 시나리오 save/load | Mentioned | **§3.1 S-01** |
| Compare 대시보드 | CP-* thin | **§3.1 A-01–A-05** |
| Session tune (타이어·서스) | “v1 carry” | **§3.1 V-07–V-09** |
| Module workshop | Missing | **§3.1 S-14** |

**v2.0 scope stays “new build-sheet shell”** — but **full product requirements = v1 parity list + BS/PV rewrite**. Setup/map/vehicle composition is **layer B**, not optional.

---

## 4. Functional requirements (by feature) — v2 **delta** only

### 4.1 Vehicle build-sheet (MVP core)

| ID | Requirement | Priority | Acceptance |
|----|-------------|----------|------------|
| BS-01 | Entry: Fleet vehicle → **Build** opens modal, **Assembly** tab default | P0 | 1 click from tree |
| BS-02 | **Preset-first**: recommended blueprints as chips; then per-slot edit | P0 | FS/sedan/sports chips |
| BS-03 | Slot list grouped: Body & chassis / Powertrain / Grip / Control | P0 | Matches `assembly.py` categories |
| BS-04 | Row = installed part + **change**; click row → part cards for that slot | P0 | Single-click install (not double-click) |
| BS-05 | **Resolved stats** panel: mass, WB, CG h, μ, drive — from physics resolve | P0 | Not hand-typed |
| BS-06 | Hover card → **Δ stats** preview; click → install | P1 | Debounced `/api/catalog/assembly?preview=` |
| BS-07 | `build_complete` = required slots only (optional corner tires excluded) | P0 | L2 default blueprint true |
| BS-08 | Level L1–L5 selector updates slots (L3+ chassis slots appear) | P0 | `slots_for_level` |
| BS-09 | Compat badges on cards (`front_chassis` schema, axle hint) | P1 | `part_compat` |
| BS-10 | Footer: **Sync draft** + **Export blueprint**; link to Setup for experiment | P0 | No scene editing in build-sheet |
| BS-11 | Blueprint change: **confirm** before replacing custom `parts` | P0 | Dialog lists slots that will reset |

### 4.2 Build-sheet preview (schematic, not 3D car)

| ID | Requirement | Priority | Acceptance |
|----|-------------|----------|------------|
| PV-01 | **No** runtime vehicle mesh in build-sheet modal | P0 | Removed Three.js side preview |
| PV-02 | **L1–L2**: textbook side view — sprung mass, CG, WB, wheels | P0 | SVG |
| PV-03 | **L3**: 7-DOF schematic — spring/damper per corner | P0 | SVG |
| PV-04 | **L4–L5**: Adams-style linkage side view (ISO x–z) from kin YAML | P0 | `/api/suspension/schematic` |
| PV-05 | Active chassis slot highlights matching axle panel | P1 | front/rear highlight |
| PV-06 | Preview updates on hover Δ stats | P1 | Same data as stats panel |

### 4.3 Session tune (vehicle modal, non-assembly tabs)

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| ST-01 | Chassis / tire / brake / steer / drivetrain field forms from schema | P1 | v1 `fields.js` |
| ST-02 | Overrides are **session** only; assembly note points to build-sheet | P1 | |
| ST-03 | Suspension K&C plots at L3/L4 | P2 | existing API |
| ST-04 | Engine tab stub or hidden until powertrain part wired | P2 | |

### 4.4 Experiment / scene (Setup bar — layer B, **v1 parity**)

Full IDs: **§3.1 S-01–S-18**. v2.1 re-skins this bar; v2.0 must keep v1 route working.

| ID | Requirement | Priority | v2 phase |
|----|-------------|----------|----------|
| EX-01 | Scenario load/save/export/import | P0 | v1 until v2.1 |
| EX-02 | Map: μ, grade, bank, v_target | P0 | v2.1 UX pass |
| EX-03 | xodr + terrain obj load/clear | P1 | v2.1 |
| EX-04 | Path preset + waypoint table + 3D path edit | P0 | v2.1 |
| EX-05 | Fleet spawn table + drag in view | P0 | v2.1 |
| EX-06 | Cosim attach + IO targets | P1 | v1 |
| EX-07 | C++ module workshop tab | P2 | v1 |
| EX-08 | Infra sensors authoring tree | P2 | v1 |
| EX-09 | Stunt scene viz (ramp/loop) | P2 | v1 render; G-05 authoring later |
| EX-10 | Link from build-sheet footer → **scroll/highlight Setup** | P1 | v2.0 |

### 4.5 Play / runtime visualization

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| PL-01 | SSE state @ ~60 Hz → Three.js fleet + chase/top/orbit | P0 | v1 |
| PL-02 | Wheel force vectors (Fx/Fy/Fz) offset outside tread | P1 | triad common origin |
| PL-03 | Minimap trail, spawn hints, path edit mode | P1 | v1 |
| PL-04 | Telemetry panel (Fz, Ft, slip, …) | P1 | v1 |
| PL-05 | WebGPU optional; WebGL2 fallback | P2 | architecture doc |

### 4.6 Compare / validation

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| CP-01 | Select N fleet blueprints → ISO maneuver (step-steer, …) | P1 | `vdsim_compare` |
| CP-02 | Overlay chart + Δ% table vs baseline | P1 | v1 |
| CP-03 | Separate from build-sheet (not embedded) | P0 | IA rule |

### 4.7 Fleet management

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| FL-01 | Add vehicle from blueprint preset | P0 | |
| FL-02 | Multi-vehicle fleet, live_vid selection | P1 | v1 |
| FL-03 | Per-vehicle color, spawn pose | P1 | v1 |
| FL-04 | Remove vehicle (sim stopped) | P1 | v1 |

### 4.8 Parts library & import

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| PT-01 | Browse by type: body, aero, ride, chassis, tire, … | P1 | |
| PT-02 | Import kin YAML → `gui_custom` | P2 | |
| PT-03 | Read-only YAML preview in UI | P2 | |
| PT-04 | Register blueprint from current fleet build | P2 | |

### 4.9 Help & documentation

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| HP-01 | `.v2-help` icon → mkdocs (`docHref`) | P0 | |
| HP-02 | Slot row help → PARTS_CATALOG glossary | P1 | |
| HP-03 | Kinematics warnings → doc link, amber in conn bar | P1 | v1 |

### 4.10 Cosim / external IO

| ID | Requirement | Priority | Notes |
|----|-------------|----------|-------|
| IO-01 | UDP cosim start/stop, status in sidebar | P1 | v1 |
| IO-02 | Telemetry export targets | P2 | v1 |
| IO-03 | FFB UDP port optional | P3 | |

---

## 5. Non-functional requirements

| ID | Category | Requirement |
|----|----------|-------------|
| NF-01 | Performance | Build-sheet modal interactive < 200 ms perceived (resolve cached) |
| NF-02 | Performance | Play viewport 30+ FPS on integrated GPU (1–4 vehicles) |
| NF-03 | Reliability | No silent catalog resolve failures — show `summary.error` |
| NF-04 | Accessibility | Keyboard: Esc closes modal; focus trap in modal |
| NF-05 | i18n | UI English; Korean README/docs OK; no hard-coded Korean in v2 shell |
| NF-06 | Security | No secrets in browser; Hyundai `.tir` never exposed in GUI |
| NF-07 | Test | `assembly_api`, `multi_vehicle_compare` ctest green after GUI API changes |
| NF-08 | Offline | Lab LAN works without outbound internet |

---

## 6. Visual / design system (summary)

Full tokens: to be re-authored in `gui/static/tokens.css` + `GUI_V2_DESIGN_SYSTEM.md`.

| Token area | Requirement |
|------------|-------------|
| Mood | Light, Notion-like; accent `#01A0E9` |
| Modal build-sheet | max-width ~1100px, `.build-sheet` |
| Chips | Preset pills `.v2-chip` / `.v2-chip-on` |
| Typography | System UI 10–15px scale |
| Density | Danawa-style dense rows; resolved stats always visible |

---

## 7. Technical constraints (do not break)

| Constraint | Detail |
|------------|--------|
| Wheel order | FL=0, FR=1, RL=2, RR=3 |
| Frame | ISO 8855 right-handed |
| Backend | `gui/server.py` Runner + `catalog/assembly.py` |
| No SimSession in browser | Play uses `vdsim_realtime` / cosim relay |
| REST+SSE | No WebSocket requirement for MVP |

---

## 8. Phased delivery (revised — v1 parity explicit)

| Phase | Scope | Exit criteria |
|-------|-------|---------------|
| **v2.0** | `/v2/app.html` + **build-sheet rewrite** (BS-*, PV-*); v1 `/app.html` **unchanged** for Setup/Play/Compare | Build-sheet done; **§3.1 S/P/A lists still pass on v1** |
| **v2.1** | **Run composition** shell — Map / Vehicle / Options tabs in v2 design system; scenario presets (FS/Auto) | S-01–S-11 re-skinned; no feature loss vs §3.1 |
| **v2.2** | Play viewport + telemetry + minimap in v2 shell; optional global nav Setup·Sim·Analyze | P-01–P-09 parity |
| **v2.3** | Framework (Svelte/React) if justified; G-02 dockable panels | Same wire contract |

**Regression rule:** Before any v2 phase ships, run manual checklist **§3.1** on the target route.

**Explicitly out of all v2 phases (until product decision):** CMake GUI, CAD export, in-browser `.tir` editing (G-06), AAA game visuals.

---

## 9. v1 → v2 gap (current codebase)

| Area | v1 state | v2 target |
|------|----------|-----------|
| `app.js` size | ~4k LOC monolith | Split: `build_viz.js`, shell, routes |
| Build-sheet UX | Double-click install (fixed in stash) | Single-click + hover preview |
| Preview | Three.js car in modal (stash removed) | Level-aware SVG |
| `build_complete` | Was false on L2 (fixed in review commit) | Required slots only |
| `part_compat` | Old slot names (fixed) | `front_chassis` / `body` |
| Design tokens | Partial in stash | Full `tokens.css` |
| powertrain slot | Category without slot | Add slot or remove category |

---

## 10. Locked decisions (2026-06-21)

| # | Decision | Choice | Rationale |
|---|----------|--------|-----------|
| **D1** | Blueprint switch + custom parts | **(B) Confirm dialog** — show which slots reset; default cancel | Review: silent wipe on blueprint change was a bug |
| **D2** | v2 entry | **`/v2/app.html`** (separate shell); v1 `/app.html` unchanged until parity | Safe incremental migration; no breaking lab bookmarks |
| **D3** | Framework | **(A) Vanilla ES modules** for v2.0; Svelte/React re-evaluate at v2.3 | `app.js` already modularizing; wire contract is the boundary |
| **D4** | Session tune in MVP | **(A) v1 tabs on v1 route**; v2.0 only build-sheet | Setup/map stay on v1 until v2.1 |
| **D5** | Compare modal | **(A) Keep v1** — no redesign in v2.0 | Compare already works; polish in v2.1+ |
| **D6** | `powertrain` slot | **(A) Add to `slots.py`** + assembly category | Split-powertrain blueprints editable in build-sheet |

---

## 11. Open decisions (deferred)

| # | Question | When |
|---|----------|------|
| — | Merge-by-slot blueprint policy (alternative to confirm+replace) | v2.1 if users want non-destructive preset switch |
| — | Svelte vs React | v2.3 framework pass |
| — | WebGPU default renderer | After v2.0 shell stable |

---

## 12. Traceability matrix (feature → verification)

| Feature | Automated check |
|---------|-----------------|
| Assembly slots L2/L3 | `ctest -R assembly_api` |
| Blueprint resolve | `ctest -R blueprint_roundtrip` |
| Multi-vehicle compare API | `ctest -R multi_vehicle_compare` |
| Catalog index | `ctest -R catalog_api` |
| Schematic API | manual / `GET /api/suspension/schematic` |
| GUI smoke | `tests/scripts/test_compare.py` (headless) |

---

## 13. Related artifacts

| Doc | Role |
|-----|------|
| `GUI_V2_DESIGN_SYSTEM.md` | Visual spec (to recreate on restart) |
| `GUI_V2_SPEC.md` | Prior round decisions (archive after merge into this doc) |
| `docs/HANDOFF.md` | Session handoff |
| Stash `wip: main GUI v2 + build_viz` | Prototype code — **do not merge blindly**; cherry-pick per requirement ID |
