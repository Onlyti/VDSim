# GUI v3 — concept art

Target UI mockups for [`GUI_V3_REQUIREMENTS.md`](../GUI_V3_REQUIREMENTS.md) and
[`GUI_V3_ADR.md`](../GUI_V3_ADR.md). AI-generated reference images (not pixel-perfect spec).

**Do not use in thesis or onboarding without a live capture side-by-side** — several
panels (Scenario waypoint table, Run force triads, Build drawer) differ from or exceed
what is implemented. See [`GUI_V3_VISUAL_REVIEW.md`](../../evidence/review/GUI_V3_VISUAL_REVIEW.md).

| File | Mode | Description |
|------|------|-------------|
| `vdsim-v3-scenario-studio-concept.png` | **Scenario** (P0) | Map + path + fleet inspector — default landing |
| `vdsim-v3-run-mode-concept.png` | **Run** (P1) | Three.js viewport + telemetry |
| `vdsim-v3-build-parts-assembly-concept.png` | **Build** | Danawa slot list + schematic + part cards |
| `vdsim-v3-build-drawer-from-scenario-concept.png` | **Build** drawer | Edit parts without leaving Scenario |
| `vdsim-v3-l4-hardpoint-editor-concept.png` | **L4 hardpoints** | DW linkage editor + K&C live curves |
| `vdsim-v3-l4-hardpoint-pick-mode-concept.png` | **L4 pick** | Drag hardpoint on schematic |

Implementation: `gui/v3/` (Svelte 5 + Vite).
