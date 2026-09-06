# GUI v3 — Architecture Decision Records

Status: **accepted** (2026-06-21)  
Parent: [`GUI_V3_REQUIREMENTS.md`](GUI_V3_REQUIREMENTS.md)  
Resolves open decisions **O1, O3, O4** from v3 requirements §11.

---

## ADR-001 — Application framework: **Svelte 5 + Vite SPA**

### Context

- v1 is a **4235 LOC** vanilla `app.js` monolith on stdlib `http.server`.
- v3 needs four modes, a component design system, typed API client, and maintainable
  module boundaries (NF-09: no file > 800 LOC).
- Backend stays Python (`gui/server.py`); **no SSR**, no Node server in production.
- Lab constraints: offline-first, integrated GPU, SSE @ ~60 Hz telemetry.

### Options considered

| Option | Pros | Cons |
|--------|------|------|
| **A. Svelte 5 + Vite SPA** | Fine-grained updates (SSE-friendly); low boilerplate for dense rows/forms; small bundles; single-file components | Smaller hiring pool vs React; Three.js lifecycle needs discipline |
| **B. React 19 + Vite SPA** | Largest ecosystem (docking, charts); most agent/human familiarity | More boilerplate; larger runtime; easy to over-abstract |
| **C. SvelteKit** | File routing, adapters | SSR/adapters unused; couples to Kit conventions for a static lab SPA |
| **D. Vue 3 + Vite** | Good DX | Weaker fit for planned docking/chart libs; team has no Vue history |
| **E. Stay vanilla ES modules** | Zero build step | Already failed at v1 scale; rejected in v3 charter |

### Decision

**A — Svelte 5 + Vite SPA** (not SvelteKit).

### Rationale

1. **No SSR benefit** — app is a local authoring client over REST/SSE; static `dist/` is enough.
2. **SSE / Run mode** — compiler-driven fine-grained DOM updates avoid re-rendering large React trees
   on every 16 ms state tick (telemetry drawer can subscribe narrowly).
3. **Build + Compose density** — Danawa slot rows, inspector fields, and status chips are
   faster to ship in Svelte with less state-plumbing than React hooks/context.
4. **Bundle size** — lab LAN + offline vendor fonts; smaller initial JS helps cold load.
5. **Monolith split** — `.svelte` per panel maps cleanly to `modes/{scenario,run,build,analyze}/`.

### Consequences

- Add `gui/v3/package.json`, `vite.config.ts`, `svelte.config.js`.
- **Dev prerequisite:** Node **≥ 20** (system has v24 — OK). Python server unchanged.
- `gui/v3/dist/` is **build output** — not committed; documented in README + CMake optional target later.
- Port v1 Three.js scene logic to `gui/v3/src/lib/viewport/` as imperative TS module
  (no `@react-three/fiber` — keep v1 approach, less magic).
- Agents/contributors need Svelte 5 runes/docs; link in `gui/v3/README.md`.

### Revisit if

- Team standardizes on React for other products and won't maintain Svelte.
- A required P0 library is React-only **and** has no Svelte port (none identified today).

---

## ADR-002 — Language: **TypeScript (strict)**

### Decision

All v3 app code in **TypeScript** with `strict: true`.

### Rationale

Wire contract has many endpoints (`/api/setup`, `/api/catalog/assembly`, `/api/compare`, …).
Typed `api/*.ts` modules catch field drift at compile time; aligns with NF-07 API stability.

### Consequences

- Hand-maintained types in `gui/v3/src/lib/api/types.ts` initially.
- Optional later: generate from OpenAPI if `gui/api` documents JSON shapes.

---

## ADR-003 — Routing: **hash mode** (`/v3/#/scenario`)

### Context

`gui/api/routes.py` serves exact paths; no generic SPA fallback today.

### Options

| Option | Server change | Bookmarkable |
|--------|---------------|--------------|
| Hash `#/build` | Serve single `index.html` at `/v3/` only | Yes (full URL) |
| History `/v3/build` | Catch-all → `index.html` for every `/v3/*` | Cleaner URLs |

### Decision

**Hash routing** for v3.0-alpha through v3.0-rc. Default hash: **`#/scenario`**.

### Rationale

- **Minimal server diff** — one new route `/v3/` + static asset mount for `dist/assets/`.
- Avoids accidental shadowing of future `/api/*` paths.
- Upgrade to history API in v3.1 is a client-only change + small routes.py fallback.

### Implementation

- Router: lightweight (`svelte-spa-router` or ~50 LOC hash store).
- Modes: `#/scenario` | `#/run` | `#/build` | `#/analyze` — **default `#/scenario`**.

---

## ADR-004 — Build & deploy layout

### Decision

```
gui/v3/
  package.json
  vite.config.ts          # base: '/v3/', outDir: 'dist'
  src/
    main.ts
    App.svelte            # shell + nav
    modes/ ...
    lib/api/ ...
    lib/viewport/ ...
    components/ ...
  dist/                   # gitignored — npm run build
```

Vite `base: '/v3/'` so asset URLs work when served under `/v3/`.

### Server integration (`gui/api/routes.py` — planned)

| Route | Serves |
|-------|--------|
| `GET /v3`, `GET /v3/` | `gui/v3/dist/index.html` |
| `GET /v3/assets/*` | `gui/v3/dist/assets/*` |
| `GET /app.html` | v1 (until v3.0 cutover) |

v1 `/static/*` and `/vendor/*` unchanged.

### Build commands

```bash
cd gui/v3 && npm ci && npm run build    # production
cd gui/v3 && npm run dev                # Vite dev server; proxy /api → :8095
```

### Consequences

- CI (future): add Node job `npm run check && npm run build` — does not block ctest today.
- README: v3 dev is optional; default users still use v1 until cutover.

---

## ADR-005 — Three.js: **keep vendored copy**

### Decision

Import Three.js from existing `gui/vendor/three/` via Vite alias — **no CDN**, no duplicate npm three unless vendor is removed later.

```ts
// vite.config.ts (sketch)
resolve: {
  alias: {
    three: path.resolve(__dirname, '../vendor/three/three.module.js'),
  },
},
```

### Rationale

P5 offline-first; v1 already ships vendor tree; pin version when upgrading.

### Consequences

- WebGPU renderer path same as v1 (`three/addons/` from vendor).
- NF-02 perf validation stays comparable to v1 baseline.

---

## ADR-006 — Layout / docking: **CSS split panes first**

### Context

Design review asked for dockable panels (G-02). Libraries like `flexlayout-react` are React-only.

### Decision

v3.0-alpha/beta: **CSS grid + `resize` handles** per mode default layout (requirements §4.3).
No docking library until v3.1 **if** users need persisted arbitrary layouts.

### Rationale

- Ships faster; satisfies “resizable panels” for default Compose/Build splits.
- Keeps framework choice from being forced by a React-only dock lib.
- SH-06 (localStorage layout persist) stores pane widths only, not arbitrary dock graphs.

### Revisit if

- FS/Auto users request IDE-style free docking → evaluate `golden-layout` (framework-agnostic)
  or migrate layout shell to React (would trigger ADR-001 revisit).

---

## ADR-007 — Design system workflow: **in-repo tokens + Storybook**

### Context

O2: Figma vs Storybook. HANDOFF: “design pass before more hand-coded CSS.”

### Decision

1. **`gui/v3/src/tokens.css`** — source of truth for colors, spacing, typography (DS-01).
2. **Storybook 8 + Svelte** in `gui/v3/.storybook/` for core components (DS-02).
3. **Figma optional** — link in README if maintainer maintains one; not a merge blocker.

### Rationale

- OSS repo: Storybook lives with code, reviewable in PRs, no external tool lock-in.
- Components (Chip, DenseRow, InspectorField) get visual regression baseline for agents.

### Consequences

- `npm run storybook` for local design review (DS-05 gate).
- No Figma-to-code automation in v3.0.

---

## ADR-008 — v1 deprecation & cutover

### Decision

| Milestone | Policy |
|-----------|--------|
| v3.0-alpha … rc | v1 `gui/app.html` = **production**; `/v3/` = preview |
| v3.0 | `/app.html` redirects to `/v3/`; move v1 to `gui/legacy/` (read-only, one release) |
| After v3.0 +1 | Delete `gui/legacy/` if no rollback request |

### Rationale

P6 parity-before-cutover; lab bookmarks on `/app.html` get redirect, not silent break.

---

## ADR-009 — State management

### Decision

| Scope | Approach |
|-------|----------|
| Server draft (fleet, scene, path) | **Server truth** — `GET/POST /api/setup`; UI shows sync errors |
| SSE stream (Run) | Dedicated `viewport` module + narrow Svelte stores for telemetry |
| Build assembly | Fetch on action; debounced preview query param |
| Cross-mode | Shell store: `scenarioName`, `liveVid`, `connStatus` |

No Redux/Pinia. Avoid global client cache of physics params beyond session.

### Rationale

R6 — physics resolves on server; duplicating resolve client-side caused v1 bugs.

---

## ADR-010 — API client pattern

### Decision

Thin module per domain:

```
gui/v3/src/lib/api/
  client.ts      # fetchJson, error envelope
  setup.ts
  catalog.ts
  control.ts
  compare.ts
  stream.ts      # EventSource wrapper
  types.ts
```

Generated from `gui/api/routes.py` route list — manual types until OpenAPI exists.

### Consequences

- Any new `/api/*` route: update `types.ts` + ctest; NF-07 unchanged.

---

---

## ADR-011 — Product priority: **Scenario studio first, Run viz second**

Status: **accepted** (2026-06-21)

### Context

User direction: the GUI’s **main job** is **scenario generation** (`configs/scenes/`).
**Second** is **realtime visualization** while the plant runs — not the other way around.
v1 felt “3D viewer with setup bar attached”; v3 must invert that emphasis.

### Decision

| Priority | Mode | Route | v3 phase |
|----------|------|-------|----------|
| **1** | **Scenario** (시나리오 생성기) | `#/scenario` (default) | v3.0-alpha |
| **2** | **Run** (realtime 시각화) | `#/run` (via ▶ or tab) | v3.0-beta |
| 3 | Build (agent vehicle parts) | `#/build` or drawer from Scenario | v3.0-rc |
| 4 | Analyze | `#/analyze` | v3.0-rc |

### Rationale

- Matches VDSim runtime split: [1] gui = scenario editor + run controller ([`RUNTIME_ARCH.md`](RUNTIME_ARCH.md)).
- Scenario authoring is the differentiated OSS surface; Run reuses existing SSE + Three.js port.
- Build stays necessary (fleet `blueprint` + `parts`) but is **in service of** scenario agents.

### Consequences

- Shell nav highlights **Scenario** and **Run**; Build/Analyze de-emphasized.
- **▶ Play** syncs draft then **auto-navigates to Run** (SH-03).
- Storybook / component priority: map canvas, fleet inspector, path editor before schematic/build-sheet.
- Implementation order in ADR “Next steps” follows Scenario → Run → Build.

---

## Summary table

| ADR | Decision |
|-----|----------|
| 001 | **Svelte 5 + Vite SPA** |
| 002 | TypeScript strict |
| 003 | Hash router `/v3/#/scenario` (default) |
| 004 | Build to `gui/v3/dist/`; server serves `/v3/` |
| 005 | Three.js from `gui/vendor/` |
| 006 | CSS split panes; no dock lib in v3.0 |
| 007 | tokens.css + Storybook (Figma optional) |
| 008 | v1 until v3.0 parity → redirect |
| 009 | Server-truth draft; minimal client state |
| 010 | Typed `lib/api/*` modules |
| **011** | **Scenario-first, Run-second product priority** |

---

## Rejected: React + Vite (record)

React remains a **credible alternate**. Not chosen because:

- v3 UI is form-dense + SSE-heavy, not a large component tree with shared global state.
- Planned v3.0 does not need React-only libs (docking deferred).
- Bundle and boilerplate cost matters for a sidecar lab UI.

If ADR-001 is reopened, use **React 19 + Vite + TypeScript** with the same ADR-003–010
(hash routing, vendor three, server layout, deprecation policy).

---

## Next implementation steps

1. Scaffold `gui/v3/` (`npm create vite@latest` svelte-ts template, adapt).
2. Add `gui/v3/.gitignore` (`node_modules`, `dist`).
3. Patch `gui/api/routes.py` — `/v3/` + `/v3/assets/*` (small PR).
4. Shell: default `#/scenario` + **Run** tab + `api/client.ts` health check.
5. Storybook + `tokens.css` — **Scenario** components first (map, fleet row, path wp).
6. **Scenario mode** CO-00–CO-08, CO-16 (v3.0-alpha).
7. **Run mode** RN-00–RN-07 + viewport port from v1 (v3.0-beta).
8. Build drawer + Analyze (v3.0-rc).

---

## Related

| Doc | Role |
|-----|------|
| [`GUI_V3_REQUIREMENTS.md`](GUI_V3_REQUIREMENTS.md) | Functional requirements |
| [`GUI_V2_REQUIREMENTS.md`](GUI_V2_REQUIREMENTS.md) | v1 parity §3.1 |
| [`../gui_architecture.md`](../gui_architecture.md) | Wire contract |
| [`../HANDOFF.md`](../HANDOFF.md) | Session handoff |
