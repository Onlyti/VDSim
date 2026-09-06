# VDSim GUI v3 (Svelte 5 + Vite)

Scenario-first web UI per [`docs/design/GUI_V3_ADR.md`](../../docs/design/GUI_V3_ADR.md).

| Mode | Route | Status |
|------|-------|--------|
| **Scenario** | `#/scenario` (default) | **API + map** — fleet drag, path preset/**waypoint edit**, save/load |
| **Run** | `#/run` | **SSE + Three.js** chase/orbit/top, telemetry, stop/reset |
| **Build** | `#/build` | **catalog assembly** — slots, blueprint export/register, **parts library**, Δ stats, linkage SVG, **L4 hardpoint editor** |
| **Analyze** | `#/analyze` | **ISO compare** — fleet/catalog picks, maneuvers, yaw-rate chart, Δ% table, CSV |

Concept art: [`docs/design/assets/`](../../docs/design/assets/).

## Prerequisites

- Node **≥ 20**
- Python GUI server on port **8095** (or adjust `vite.config.ts` proxy)

## Develop

```bash
# terminal 1 — API + SSE
python3 gui/server.py --port 8095

# terminal 2 — Vite HMR (proxies /api → 8095)
cd gui/v3 && npm install && npm run dev
# open http://localhost:5173/v3/
```

## Production build (served by server.py)

```bash
cd gui/v3 && npm run build
python3 gui/server.py --port 8095
# open http://localhost:8095/v3/
```

## Headless E2E (Playwright)

```bash
cd gui/v3 && npm install && npm run test:e2e:install
python3 tests/scripts/test_gui_v3_e2e.py   # starts server on free port + runs browser
# or: ctest -R gui_v3_e2e
```

`gui/v3/dist/` is gitignored — run `npm run build` after clone.

## Layout

```
src/
  modes/scenario/   # P0 — scenario studio
  modes/run/        # P1 — Three.js viewport (port from v1)
  modes/build/      # parts + L4 hardpoint editor
  modes/analyze/
  lib/
    api/          # setup.ts, client.ts, types.ts
    map/          # viewport.ts — 2D scenario map
    setupStore.svelte.ts
  components/
    MapCanvas.svelte
```

v1 remains at `/app.html` until v3 parity ([`GUI_V3_REQUIREMENTS.md`](../../docs/design/GUI_V3_REQUIREMENTS.md) §8).
