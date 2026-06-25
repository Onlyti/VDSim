# GUI v3 — feedback synthesis & requirements

Status: **planning baseline** (2026-06-21)  
Supersedes **`GUI_V2_REQUIREMENTS.md` for product direction** — v2 incremental route is **cancelled**; v1 `/app.html` remains the production surface until v3 parity.  
Wire contract: [`../gui_architecture.md`](../gui_architecture.md) (REST + SSE)  
Catalog: [`PARTS_CATALOG.md`](PARTS_CATALOG.md) · Scenario: [`../CONFIG_GUIDE.md`](../CONFIG_GUIDE.md)

---

## 0. Charter — why v3, not v2

| Approach | What we tried | Outcome |
|----------|---------------|---------|
| **v0.2 → v1** | Full-screen `app.html`, sidebar + modal workshops + setup bar | **Feature-complete prototype** — all three composition layers shipped, but IA debt |
| **v2** | `/v2/app.html`, rewrite build-sheet only; rest stays on v1 | **Requirements felt incomplete** — map/scene/fleet/compare treated as “carry-over”; two UIs, no unified shell |
| **v3** | **Greenfield shell** — modes, dockable layout, design system first; **one** app route when ready | Target: credible OSS engineering UI, not a patched monolith |

**v3 rule:** Do not ship another partial skin. Ship a **coherent workspace** with explicit parity against [`GUI_V2_REQUIREMENTS.md` §3.1](GUI_V2_REQUIREMENTS.md) (v1 inventory).

### Product priority (2026-06-21 — locked)

| Rank | GUI role | v3 mode | Investment |
|------|----------|---------|------------|
| **1 — Main** | **시나리오 생성기** — map, road, path, fleet/agents, save/load `configs/scenes/` | **Scenario** | ~60% UX / v3.0-alpha |
| **2 — Second** | **Realtime 시각화** — Play 중 Three.js + telemetry (plant state reconstruction) | **Run** | ~30% / v3.0-beta |
| 3 — Supporting | 차량 부품 조립 (agent `blueprint` + parts) | **Build** (panel in Scenario + deep-edit route) | ~10% / v3.0-rc |
| 4 — Validation | ISO compare, metrics | **Analyze** | polish / v3.1 |

The GUI is **not** a game client or a CAD tool — it is a **scenario authoring studio** with an attached **runtime viewer**.

---

## 1. Feedback synthesis (v1 development → v2 planning)

### 1.1 External / design review (HANDOFF 2026-06)

| Score / verdict | Finding |
|-----------------|---------|
| **~4/10 “prototype”** | Looks bolted-on; not a product-grade authoring tool |
| Layout | Need **dockable / resizable** panels; stop stacking unrelated concerns in fixed columns |
| Navigation | Need **global mode nav** — user mental model: **Compose → Build → Run → Analyze** (HANDOFF: Setup · Sim · Analyze) |
| Modals | **Large vehicle modal** hides context; build-sheet + session tune + parts library trapped behind one overlay |
| Design | Need **design system** (tokens, components, density) before more hand-coded CSS |
| Stack | `stdlib http.server` + **4.2k LOC `app.js`** monolith is a maintenance ceiling — framework decision **before** more UI code |

### 1.2 What worked (keep in v3)

| Area | Evidence | v3 carry-forward |
|------|----------|------------------|
| **Three composition layers** | Vehicle build / scene compose / compare are distinct VDSim objects | Becomes **top-level modes**, not sidebar + setup bar + modals |
| **Danawa build-sheet** | Slot rows, categories, resolved stats, preset chips | **Build mode** primary surface (not a modal) |
| **Schematic preview** | Level-aware SVG (L1–L5), Adams linkage from API — stash removed 3D car in modal | **Build mode** center panel; no runtime mesh in authoring |
| **Compare dashboard** | `/api/compare`, yaw-rate overlay, Δ% table | **Analyze mode** full page |
| **Wire contract** | REST draft + SSE stream; GUI does not step physics | **Frozen** — v3 is frontend-only replacement |
| **Catalog assembly API** | `assembly.py`, preview Δ stats, `build_complete` | Same backend; v3 client uses typed API wrapper |
| **Setup bar concepts** | Map / fleet / path / save | **Scenario mode** inspector + map |
| **Play viewport** | Three.js from SSE | **Run mode** (secondary product) |
| **mkdocs help** | `[?]` → theory/docs | All non-obvious labels |

### 1.3 What failed or hurt (fix in v3)

| Pain | Symptom | v3 response |
|------|---------|-------------|
| **Modal-centric IA** | Vehicle edit = full-screen modal; compare = another modal; context loss | **Mode pages** + optional **drawers**; no full-screen modal for primary workflows |
| **Duplicate entry points** | Blueprint/level in fleet tree, assembly modal, setup vehicle tab | **Single source of truth** per field; cross-links only |
| **Split attention** | Sidebar sim + bottom setup bar + center 3D + right telemetry | **Mode-specific layout** — only show panels relevant to current job |
| **Incremental v2 trap** | Build-sheet on `/v2`, map/spawn on `/app.html` | **One route** (`/v3/` or `/app.html` swap) at parity gate |
| **UX bugs (found in review)** | Double-click to install; silent blueprint parts wipe; `build_complete` false on L2 | Fixed in core/catalog; v3 must **confirm destructive edits** (D1) |
| **Monolith JS** | `app.js` 4235 LOC — scene, assembly, setup, compare, path edit intertwined | **Framework + modules** by mode and domain |
| **Requirements doc drift** | v2 doc over-indexed build-sheet; map/fleet under-documented until late | v3 doc inherits **full §3.1 parity IDs** (see §8) |
| **Visual polish without system** | One-off CSS, inconsistent spacing/typography | **Design tokens + component library** before feature sprint |

### 1.4 User / session feedback (condensed)

| Request | v1 state | v3 requirement |
|---------|----------|----------------|
| 차량 구성 (parts, presets, stats) | Assembly modal | **Build mode** — full width |
| 지도·도로·terrain 구성 | Setup → Map | **Scenario** mode |
| 경로·waypoint 편집 | Setup table + 3D edit | **Scenario** map tools |
| 플릿 spawn | Setup Vehicle + drag | **Scenario** fleet inspector |
| 시나리오 save/load | Setup header | **Scenario** shell (CO-16) |
| Realtime 3D 시각화 | Sidebar + viewport | **Run** mode (▶ from Scenario) |
| ISO 비교 | Compare modal | **Analyze mode** |
| Session tune (타이어·서스) | Modal tabs | **Build mode** sub-inspector (not separate modal) |
| Single-click part install | Fixed in stash | **Build mode** default |
| Hover Δ stats preview | Stash / API | **Build mode** |
| Teaching credibility | Schematic not game mesh | **NF-10** — engineering aesthetic |

### 1.5 Deferred from v0.5.2 / HANDOFF (still in v3 backlog)

| ID | Item | v3 phase |
|----|------|----------|
| G-01 | Multi-fleet 3D render polish (#158) | Run mode P1 |
| G-04 | Terrain Play — chase on mesh, spawn on surface | Run mode P1 |
| G-05 | Stunt **authoring** (not render-only) | Compose P2 |
| G-06 | GUI `.tir` import | Build P2 (metadata only; no coefficient editor) |
| G-07 | Belt transient force arrow viz | Run P2 |
| G-08 | `powertrain` catalog slot | Backend + Build P0 |
| G-09 | Blueprint switch confirm | Build P0 |

---

## 2. Product positioning

| Item | Requirement |
|------|-------------|
| **Primary product** | **Scenario generator** — author `scene.yaml`: shared map/road/comms + per-agent fleet, path, spawn |
| **Secondary product** | **Realtime visualization** — while plant runs, reconstruct pose/tires/forces from SSE (not pixel stream) |
| Not | CAD, tire coefficient workshop, AAA game UI, primary 3D editor |
| Differentiator | YAML-native scenarios + compact physics state → local Three.js |
| Audience | Maintainer, FS/Auto teams, ADAS students composing experiments |
| Success (primary) | New user loads template → edits map/path/fleet → saves scenario → **≤3 min** without reading source |
| Success (secondary) | ▶ Play → Run mode shows credible motion + telemetry within **2 s** of sync |

**VDSim object the GUI owns:** `configs/scenes/*.yaml` (and derived `runs/live/` on Play).  
Vehicle physics manifests (`blueprints/`, `parts/`) are **edited in service of** scenario agents, not as a standalone app goal.

---

## 3. v3 principles

| # | Principle |
|---|-----------|
| P0 | **Scenario-first** — default mode, default route, most polish budget |
| P1 | **Run-second** — visualization serves the scenario being played, not a separate “game” |
| P2 | **One workspace** — Scenario · Run · (Build · Analyze); one design system |
| P3 | **Wire contract frozen** — REST/SSE schema is the migration boundary |
| P4 | **Authoring ≠ runtime** — Scenario edits draft; Play syncs → `runs/live/` → Run visualizes |
| P5 | **No physics in browser** — server resolves catalog; GUI edits manifests + session overrides |
| P6 | **Offline-first** — `gui/vendor/` fonts/icons; lab LAN without CDN |
| P7 | **Parity before cutover** — v1 stays at `/app.html` until §8 checklist passes on `/v3/` |
| P8 | **Design system before features** — Storybook for scenario inspector components first |
| P9 | **Not game UI** — Notion/Linear-like light OSS; `#01A0E9` accent; dense engineering rows |

---

## 4. Information architecture (v3)

### 4.1 Global shell — Scenario-centric

**Default route:** `/v3/#/scenario` (app opens in scenario studio).

```
┌─────────────────────────────────────────────────────────────────────────┐
│ VDSim   [ Scenario ★ ] [ Run ]     Build · Analyze (secondary)   scenario ▾ │
├─────────────────────────────────────────────────────────────────────────┤
│  SCENARIO STUDIO (default) — map · path · fleet · road · save/load       │
│  or RUN VIEWPORT — Three.js + transport (entered via ▶ or Run tab)       │
├─────────────────────────────────────────────────────────────────────────┤
│  draft synced · [Sync] [▶ Play → Run] · last run · warnings · [?]      │
└─────────────────────────────────────────────────────────────────────────┘
```

| Mode | Priority | User job | Primary viewport |
|------|----------|----------|------------------|
| **Scenario** | **1 — Main** | 시나리오 생성·편집 | **Map canvas** — road, terrain, path, spawn, fleet agents |
| **Run** | **2 — Second** | Realtime 시각화 | **Three.js** — chase/top/orbit, force vectors, telemetry drawer |
| **Build** | 3 — Supporting | Agent 차량 부품 조립 | Schematic + slot list (opens from Scenario fleet row or `#/build`) |
| **Analyze** | 4 — Validation | ISO compare | Charts + Δ% table |

**Cross-mode rules**

- Scenario name + draft sync **always visible**; **▶ Play** syncs draft then switches to **Run**.
- **Build** is scoped to selected fleet agent (`vid`); entry from Scenario fleet inspector, not the default landing.
- Run mode is **read-mostly** for scene params — “back to Scenario” for edits.
- No modal larger than confirm / save-as / import.

### 4.2 Scenario studio layout (main surface)

```
┌──────────────── map / path editor (70%) ────────┬─ inspector (30%) ─┐
│  · road μ / grade / bank preview                   │  Scenario meta      │
│  · fleet spawn markers (drag)                      │  Road                 │
│  · path waypoints (●)                              │  Map / terrain files  │
│  · optional light 3D ground (not full Run viz)     │  Fleet agents[]       │
│                                                    │    → blueprint link   │
│                                                    │    → [Edit parts]     │
│                                                    │  Path preset / wps     │
│                                                    │  Comms / cosim (adv)  │
└────────────────────────────────────────────────────┴───────────────────────┘
```

Fleet agent row **Edit parts** opens Build as **split pane or drawer** without leaving Scenario context when possible (deep edit → `#/build?vid=`).

### 4.3 Run layout (visualization surface)

```
┌──────────────────── Three.js viewport (100%) ────────────────────────────┐
│  transport: ⏹ ⟲ · time scale · cam orbit/chase/top · manual (optional)    │
│  overlay: spawn hint off · wheel forces · selected vehicle telemetry →    │
└──────────────────────────────────────────────────────────────────────────┘
```

Minimap trail lives in Run (or PiP); Scenario map shows **static** fleet/path for authoring.

### 4.4 v1 → v3 surface mapping

| v1 location | v3 |
|-------------|-----|
| Setup bar (Map/Vehicle/Options) | **Scenario** mode |
| Sidebar Play + center 3D | **Run** mode (+ ▶ from shell) |
| Modal → Assembly | **Build** (from Scenario agent) |
| Compare modal | **Analyze** |
| Sidebar fleet tree | Scenario **Fleet agents** inspector |

### 4.5 Default layouts

| Mode | Layout |
|------|--------|
| **Scenario** | `map (70%)` \| `inspector (30%)` — resizable |
| **Run** | `viewport (100%)` — telemetry slide-over |
| Build | `slots (25%)` \| `schematic (45%)` \| `picker + stats (30%)` |
| Analyze | `config top` \| `chart (60%)` \| `table (40%)` |

Docking: CSS split panes (ADR-006).

---

## 5. Functional requirements

IDs extend v2 where noted. **P0** = v3 launch blocker.

### 5.1 Shell & navigation

| ID | Requirement | P | Acceptance |
|----|-------------|---|------------|
| SH-01 | **Scenario** default route `#/scenario`; **Run** `#/run`; Build/Analyze secondary | P0 | Opens on Scenario |
| SH-02 | Scenario selector + load/save/export/import in shell | P0 | Replaces setup header |
| SH-03 | Draft sync + **▶ Play** → sync → auto-switch to **Run** | P0 | Same as S-02 |
| SH-04 | Connection + kinematics warnings in status strip | P0 | v1 parity |
| SH-05 | mkdocs `[?]` on labels | P0 | HP-01 |
| SH-06 | Panel resize persist (localStorage) | P1 | Per mode |
| SH-07 | Keyboard: Esc closes drawer; `?` opens help | P1 | NF-04 |

### 5.2 Scenario mode (시나리오 생성기 — **P0 main**)

Was “Compose”. IDs unchanged (`CO-*`).

| ID | Requirement | P | v1 ref |
|----|-------------|---|--------|
| CO-00 | **New scenario** template gallery (empty, skidpad, straight, figure-8) | P0 | new |
| CO-01 | Road μ, grade°, bank°, v_target | P0 | S-03 |
| CO-02 | OpenDRIVE `.xodr` load | P1 | S-04 |
| CO-03 | Terrain `.obj` load + clear | P1 | S-05 |
| CO-04 | Fleet spawn table (x0,y0,yaw0,vx0…) | P0 | S-06 |
| CO-05 | Map drag spawn + multi-vehicle add | P0 | S-07, S-16 |
| CO-06 | Path preset + waypoint table | P0 | S-08, S-09 |
| CO-07 | Path edit on map (drag ●, add, Del) | P0 | S-10 |
| CO-08 | Static fleet + path on Scenario map; trail overlay in **Run** only | P0 | S-11 |
| CO-09 | Cosim attach host/ports | P1 | S-12 |
| CO-10 | IO / telemetry targets | P2 | S-13 |
| CO-11 | C++ module workshop | P2 | S-14 |
| CO-12 | Infra sensors tree | P2 | S-15 |
| CO-13 | Stunt viz when scene defines `stunt:` | P2 | S-17 |
| CO-14 | Named scenario presets in repo (`configs/scenes/`) surfaced as chips | P1 | new |
| CO-15 | Fleet agent row → **Edit parts** (Build drawer or `#/build?vid=`) | P0 | cross-mode |
| CO-16 | **Save** writes `configs/scenes/<name>.yaml`; load/export/import in shell | P0 | S-01 |

### 5.3 Build mode (vehicle — **supporting**, entry from Scenario)

Deep-edit route for catalog assembly. Not the default landing.

| ID | Requirement | P | v1 ref |
|----|-------------|---|--------|
| BU-01 | Preset blueprint chips → slot list | P0 | BS-02 |
| BU-02 | Categories: Body&chassis / Powertrain / Grip / Control | P0 | BS-03 |
| BU-03 | Single-click row → part cards; hover Δ stats | P0 | BS-04–06 |
| BU-04 | Resolved stats panel (mass, WB, CG, μ, drive) | P0 | BS-05 |
| BU-05 | `build_complete` on required slots only | P0 | BS-07 |
| BU-06 | Level L1–L5 updates slot set | P0 | BS-08 |
| BU-07 | Compat badges on cards | P1 | BS-09 |
| BU-08 | Blueprint change **confirm** (slots reset list) | P0 | BS-11, G-09 |
| BU-09 | Export / register blueprint | P1 | V-10 |
| BU-10 | Schematic: L1–L2 textbook, L3 7-DOF, L4–L5 Adams SVG | P0 | PV-01–04 |
| BU-11 | No Three.js vehicle mesh in Build | P0 | PV-01 |
| BU-12 | Session tune tabs in inspector (chassis, tire, susp, …) | P1 | V-07–V-09 |
| BU-13 | K&C plots L3/L4 | P2 | V-08 |
| BU-14 | Parts library drawer (browse/import) | P1 | V-06, PT-* |
| BU-15 | `powertrain` slot when catalog ships | P0 | G-08 |

### 5.4 Run mode (realtime 시각화 — **P0 second**)

| ID | Requirement | P | v1 ref |
|----|-------------|---|--------|
| RN-00 | Enter via **▶ Play** (sync → Run) or **Run** tab when sim active | P0 | SH-03 |
| RN-01 | Play / Stop / Reset | P0 | P-01 |
| RN-02 | Autopilot vs manual + manbar / keyboard | P0 | P-02 |
| RN-03 | Realtime time scale | P0 | P-03 |
| RN-04 | Cam orbit / chase / top | P0 | P-04 |
| RN-05 | SSE ~60 Hz → fleet meshes | P0 | P-05 |
| RN-06 | Wheel force triad (common origin) | P1 | P-06 |
| RN-07 | Telemetry drawer | P0 | P-07 |
| RN-08 | Multi-fleet 3D (#158) | P1 | G-01 |
| RN-09 | Terrain-aware chase / spawn on mesh | P1 | G-04 |
| RN-10 | Sim cfg load/save | P1 | S-18 |
| RN-11 | WebGPU optional, WebGL2 fallback | P2 | PL-05 |

### 5.5 Analyze mode (was Compare modal)

| ID | Requirement | P | v1 ref |
|----|-------------|---|--------|
| AN-01 | Multi blueprint select from fleet | P0 | A-01–A-02 |
| AN-02 | ISO maneuver picker | P0 | A-03 |
| AN-03 | Yaw-rate overlay chart (SVG or canvas) | P0 | A-04 |
| AN-04 | Δ% table vs baseline | P0 | A-05 |
| AN-05 | Export compare CSV | P1 | headless parity |
| AN-06 | Separate from Build (no embedded compare) | P0 | CP-03 |

---

## 6. Design system requirements

| ID | Requirement |
|----|-------------|
| DS-01 | Token file: color, spacing, radius, typography (`gui/v3/tokens.css` or CSS vars from build) |
| DS-02 | Components: Button, Chip, DenseRow, Panel, InspectorField, StatusBadge, ChartFrame |
| DS-03 | Light theme default; accent `#01A0E9`; system font stack |
| DS-04 | Danawa-density slot rows; resolved stats always visible in Build |
| DS-05 | Design review gate: external or maintainer sign-off **before** Run mode polish |
| DS-06 | Optional: Figma file linked from repo; or Storybook in `gui/v3/storybook/` |

**Reference mood:** Notion / Linear / OSS engineering — not racing game HUD.

---

## 7. Technical architecture

### 7.1 Stack (decisions required in week 0)

| # | Decision | Options | v3 recommendation |
|---|----------|---------|-------------------|
| T1 | Framework | SvelteKit, React+Vite, Vue | **Svelte 5 + Vite SPA** — see [`GUI_V3_ADR.md`](GUI_V3_ADR.md) ADR-001 |
| T2 | Entry route | `/v3/`, `/app.html` swap | **`/v3/index.html`** until parity → redirect `/app.html` |
| T3 | 3D | Three.js ES module | Keep; extract `packages/viewport/` or `gui/v3/lib/viewport/` |
| T4 | API client | hand `fetch` | Thin typed wrapper from route list in `gui/api/` |
| T5 | State | mode-local stores | Compose draft = server truth; optimistic UI with sync errors surfaced |
| T6 | Build | Vite | Bundle to `gui/v3/dist/` served by `server.py` |
| T7 | Backend | `gui/server.py` | **No change** for v3.0 |

### 7.2 Module boundaries (target)

```
gui/v3/
  app/           # shell, router, mode layouts
  modes/
    scenario/    # P0 — scenario studio (was compose/)
    run/         # P1 — realtime viz
    build/       # supporting
    analyze/
  lib/
    api/         # fetch wrappers
    viewport/    # Three.js (Run)
    schematic/   # SVG build preview
    charts/      # Analyze
  components/    # design system
  tokens.css
```

v1 `gui/static/app.js` **not extended** — maintenance mode only until v3 cutover.

### 7.3 Wire contract

| Rule | Detail |
|------|--------|
| Unchanged | `POST /api/setup`, `/api/control`, `/api/stream`, `/api/catalog/*`, `/api/compare`, … |
| Optional v3 additions | `GET /api/schema/*` expansion for form codegen (future); not blocking v3.0 |
| Wheel order | FL=0, FR=1, RL=2, RR=3 |
| Frame | ISO 8855 RH |

---

## 8. Parity checklist (v3 launch gate)

All items from [`GUI_V2_REQUIREMENTS.md` §3.1](GUI_V2_REQUIREMENTS.md) must pass on `/v3/`. Summary:

| Group | Count | v3 mode |
|-------|-------|---------|
| V-* vehicle build | 11 | Build (from Scenario) |
| S-* scene compose | 18 | **Scenario** |
| P-* play runtime | 9 | **Run** |
| A-* analyze | 5 | Analyze |
| G-* gaps | 9 | per table §1.5 (P0 items block launch) |

**Automated:** `ctest -R 'assembly|catalog|multi_vehicle_compare|gui_v3_api_smoke|gui_v3_e2e'` + `python3 tools/gui_v3_eval_bundle.py` (npm + Playwright + parity audit).

---

## 9. Non-functional requirements

| ID | Category | Requirement |
|----|----------|-------------|
| NF-01 | Performance | Build interactions < 200 ms perceived (cached resolve) |
| NF-02 | Performance | Run viewport 30+ FPS integrated GPU, 1–4 vehicles |
| NF-03 | Reliability | Show `summary.error` on catalog resolve failure |
| NF-04 | Accessibility | Focus trap in drawers; keyboard transport in Run |
| NF-05 | i18n | UI English |
| NF-06 | Security | No Hyundai `.tir` secrets in browser |
| NF-07 | Test | API ctests green after any `gui/api` change |
| NF-08 | Offline | Lab LAN without internet |
| NF-09 | Maintainability | No single file > 800 LOC (enforced in review) |
| NF-10 | Aesthetic | Engineering schematic > game mesh for authoring |

---

## 10. Phased delivery (scenario-first)

| Phase | Deliverable | Exit |
|-------|-------------|------|
| **v3.0-alpha** | Shell + **Scenario studio** CO-00–CO-08, CO-16 + templates | Author/save scenario without v1 |
| **v3.0-beta** | **Run visualization** RN-00–RN-07 (▶ → sync → Run) | Realtime viz credible |
| **v3.0-rc** | **Build** drawer/route BU-01–BU-11 + **Analyze** AN-01–AN-04 | §8 parity manual pass |
| **v3.0** | Redirect `/app.html` → `/v3/#/scenario`; v1 → `gui/legacy/` | Cutover |
| **v3.1** | G-01/G-04 Run polish, CO-09–CO-13, dock persist | |
| **v3.2** | G-05 stunt authoring, G-06, time-series Analyze | |

**Investment order:** Scenario **→** Run **→** Build **→** Analyze.

**Explicitly out of v3.0:** CMake GUI, CAD export, in-browser coefficient editing, AAA visuals.

---

## 11. Open decisions (week 0)

| # | Question | Owner | Blocker for |
|---|----------|-------|-------------|
| ~~O1~~ | ~~SvelteKit vs React+Vite~~ | — | **Closed → ADR-001: Svelte 5 + Vite SPA** |
| O2 | Figma emphasis vs Storybook-only | Maintainer | DS-05 (ADR-007: Storybook primary) |
| ~~O3~~ | Docking library vs fixed split panes | — | **Closed → ADR-006: CSS splits v3.0** |
| ~~O4~~ | `/v3/` URL vs hash routing | — | **Closed → ADR-003: hash** |
| O5 | Merge-by-slot blueprint (non-destructive preset) | Product | BU-08 alternative |

---

## 12. Traceability

| Source doc | Relationship |
|------------|--------------|
| `GUI_V2_REQUIREMENTS.md` | Parity inventory §3.1; v2 phased plan **cancelled** |
| `V0.2_GUI_REDESIGN.md` | Historical v1 layout intent |
| `docs/gui_architecture.md` | Wire contract |
| `docs/HANDOFF.md` | Session state + design review quote |
| Stash `wip: main GUI v2 + build_viz` | Cherry-pick → v3 Build |
| **Concept art** | [`docs/design/assets/`](assets/) — 6 PNG mockups |

---

## 13. ADR

**Accepted:** [`GUI_V3_ADR.md`](GUI_V3_ADR.md) — incl. **ADR-011 Scenario-first**.


*End of v3 requirements baseline.*
