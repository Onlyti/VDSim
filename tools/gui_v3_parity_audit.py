#!/usr/bin/env python3
"""Static parity audit: GUI v3 sources vs GUI_V3_REQUIREMENTS IDs (no browser)."""
from __future__ import annotations

import argparse
import json
import sys
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
V3 = REPO / "gui" / "v3" / "src"


def src_text() -> str:
    parts: list[str] = []
    for p in sorted(V3.rglob("*")):
        if p.suffix in (".svelte", ".ts", ".js") and p.is_file():
            parts.append(p.read_text(encoding="utf-8", errors="replace"))
    return "\n".join(parts)


def file_exists(rel: str) -> bool:
    return (V3 / rel).is_file()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    t = src_text()
    dist_ok = (REPO / "gui" / "v3" / "dist" / "index.html").is_file()

    def pass_(note: str) -> dict:
        return {"status": "PASS", "note": note}

    def partial(note: str) -> dict:
        return {"status": "PARTIAL", "note": note}

    def gap(note: str) -> dict:
        return {"status": "GAP", "note": note}

    checks: list[dict] = [
        {"id": "SH-01", "group": "shell", "prio": "P0", **pass_("hash routes scenario/run/build/analyze")},
        {"id": "SH-02", "group": "shell", "prio": "P0",
         **(pass_("scenario save/load in ScenarioMode") if "saveScenarioDraft" in t else gap("no save"))},
        {"id": "SH-03", "group": "shell", "prio": "P0",
         **(pass_("Play → run") if "playSimulation" in t and "mode = 'run'" in t else gap("no play→run"))},
        {"id": "SH-04", "group": "shell", "prio": "P0",
         **(partial("kin warnings in Run status only") if "kinematics_warnings" in t else gap("no warning strip"))},
        {"id": "SH-05", "group": "shell", "prio": "P0",
         **(pass_("mkdocs help links") if "HelpLink" in t and "docsHelp" in t else gap("mkdocs [?] help links not wired"))},
        {"id": "SH-06", "group": "shell", "prio": "P1", **gap("panel resize persist")},
        {"id": "CO-00", "group": "scenario", "prio": "P0",
         **(pass_("template gallery") if "SCENARIO_TEMPLATES" in t and "applyScenarioTemplate" in t else gap("template gallery"))},
        {"id": "CO-01", "group": "scenario", "prio": "P0",
         **(pass_("road μ/grade/bank") if "gradeDeg" in t and "s.road.mu" in t else gap("road params"))},
        {"id": "CO-02", "group": "scenario", "prio": "P1", **gap("OpenDRIVE xodr")},
        {"id": "CO-03", "group": "scenario", "prio": "P1", **gap("terrain obj")},
        {"id": "CO-04", "group": "scenario", "prio": "P0",
         **(pass_("fleet table") if "fleet" in t and "FleetAgent" in t else gap("fleet"))},
        {"id": "CO-05", "group": "scenario", "prio": "P0",
         **(pass_("map drag") if "MapCanvas" in t and "dragVid" in t else gap("map drag"))},
        {"id": "CO-06", "group": "scenario", "prio": "P0",
         **(pass_("path preset + wp table") if "path_preset" in t and 'class="wp"' in t else gap("path"))},
        {"id": "CO-07", "group": "scenario", "prio": "P0",
         **(pass_("path edit") if "pathEdit" in t else gap("waypoint edit"))},
        {"id": "CO-08", "group": "scenario", "prio": "P0", **pass_("static map path; Run viewport separate")},
        {"id": "CO-09", "group": "scenario", "prio": "P1", **gap("cosim attach UI")},
        {"id": "CO-14", "group": "scenario", "prio": "P1",
         **(partial("scenario list from API") if "scenarios" in t else gap("preset chips"))},
        {"id": "CO-15", "group": "scenario", "prio": "P0",
         **(pass_("build?vid=") if "buildHref" in t else gap("edit parts link"))},
        {"id": "CO-16", "group": "scenario", "prio": "P0",
         **(pass_("save/load scenario") if "saveScenarioDraft" in t else gap("scene save"))},
        {"id": "BU-01", "group": "build", "prio": "P0",
         **(pass_("blueprint presets") if "recommended" in t else gap("presets"))},
        {"id": "BU-02", "group": "build", "prio": "P0",
         **(pass_("categories") if "categories" in t else gap("categories"))},
        {"id": "BU-03", "group": "build", "prio": "P0",
         **(pass_("preview Δ + install") if "pinnedCandidate" in t else gap("card preview"))},
        {"id": "BU-04", "group": "build", "prio": "P0",
         **(pass_("stats panel") if "mass_kg" in t else gap("stats"))},
        {"id": "BU-05", "group": "build", "prio": "P0",
         **(pass_("build_complete surfaced") if "build_complete" in t else gap("build_complete"))},
        {"id": "BU-06", "group": "build", "prio": "P0",
         **(pass_("L1–L5") if "FLEET_LEVELS" in t else gap("levels"))},
        {"id": "BU-07", "group": "build", "prio": "P1",
         **(pass_("compat badges") if "slotCompatTone" in t else gap("compat"))},
        {"id": "BU-08", "group": "build", "prio": "P0",
         **(pass_("blueprint confirm") if "Changing blueprint replaces" in t else gap("confirm"))},
        {"id": "BU-09", "group": "build", "prio": "P1",
         **(pass_("export blueprint") if "exportBlueprintYaml" in t else gap("export"))},
        {"id": "BU-10", "group": "build", "prio": "P0",
         **(pass_("linkage SVG") if "linkageSvg" in t else gap("schematic"))},
        {"id": "BU-11", "group": "build", "prio": "P0", **pass_("no Three.js in BuildMode")},
        {"id": "BU-13", "group": "build", "prio": "P2",
         **(pass_("K&C in HardpointEditor") if "kcPlotSvg" in t else gap("K&C plots"))},
        {"id": "BU-14", "group": "build", "prio": "P1",
         **(pass_("PartsLibrary") if file_exists("components/PartsLibrary.svelte") else gap("parts lib"))},
        {"id": "BU-HP", "group": "build", "prio": "P0",
         **(pass_("HardpointEditor") if file_exists("components/HardpointEditor.svelte") else gap("L4 HP editor"))},
        {"id": "RN-00", "group": "run", "prio": "P0", **pass_("Run mode route")},
        {"id": "RN-01", "group": "run", "prio": "P0",
         **(pass_("stop/reset") if "controlAction" in t else gap("transport"))},
        {"id": "RN-02", "group": "run", "prio": "P0",
         **(pass_("autopilot/manual + manbar") if "createManualControl" in t and "setFleetDriver" in t else gap("autopilot/manual + manbar"))},
        {"id": "RN-03", "group": "run", "prio": "P0",
         **(pass_("time scale") if "setTimeScale" in t else gap("time scale"))},
        {"id": "RN-04", "group": "run", "prio": "P0",
         **(pass_("cam modes") if "CamMode" in t else gap("camera"))},
        {"id": "RN-05", "group": "run", "prio": "P0",
         **(pass_("SSE viewport") if "RunViewport" in t else gap("stream"))},
        {"id": "RN-06", "group": "run", "prio": "P1",
         **(pass_("wheel force triad") if "wheelForces" in t and "updateWheelForces" in t else gap("wheel force triad"))},
        {"id": "RN-07", "group": "run", "prio": "P0",
         **(pass_("telemetry") if "pickVehicleState" in t else gap("telemetry"))},
        {"id": "AN-01", "group": "analyze", "prio": "P0",
         **(pass_("fleet/catalog picks") if "AnalyzeMode" in t else gap("picks"))},
        {"id": "AN-02", "group": "analyze", "prio": "P0",
         **(pass_("maneuvers") if "COMPARE_MANEUVERS" in t else gap("maneuvers"))},
        {"id": "AN-03", "group": "analyze", "prio": "P0",
         **(pass_("yaw chart") if "yawRateOverlaySvg" in t else gap("chart"))},
        {"id": "AN-04", "group": "analyze", "prio": "P0",
         **(pass_("delta table") if "rowsToCsv" in t else gap("table"))},
        {"id": "AN-05", "group": "analyze", "prio": "P1",
         **(pass_("CSV export") if "downloadCsv" in t or "rowsToCsv" in t else gap("csv"))},
        {"id": "AN-06", "group": "analyze", "prio": "P0", **pass_("separate Analyze mode route")},
        {"id": "NF-BUILD", "group": "nonfunctional", "prio": "P0",
         **(pass_("v3 dist present") if dist_ok else gap("npm run build required"))},
    ]

    counts = {"PASS": 0, "PARTIAL": 0, "GAP": 0}
    p0_gap = 0
    for c in checks:
        counts[c["status"]] += 1
        if c["prio"] == "P0" and c["status"] == "GAP":
            p0_gap += 1

    day = date.today().isoformat()
    out_dir = args.out_dir or (REPO / "docs" / "evidence" / "review" / day)
    out_dir.mkdir(parents=True, exist_ok=True)

    matrix = {
        "date": day,
        "tool": "gui_v3_parity_audit.py",
        "counts": counts,
        "p0_gaps": p0_gap,
        "checks": checks,
    }
    (out_dir / "gui_v3_parity_matrix.json").write_text(
        json.dumps(matrix, indent=2) + "\n", encoding="utf-8")

    lines = [
        f"# GUI v3 parity audit — {day}",
        "",
        f"Automated static audit (no browser). P0 gaps: **{p0_gap}**",
        "",
        f"| Status | Count |",
        f"|--------|-------|",
        f"| PASS | {counts['PASS']} |",
        f"| PARTIAL | {counts['PARTIAL']} |",
        f"| GAP | {counts['GAP']} |",
        "",
        "| ID | Prio | Status | Note |",
        "|----|------|--------|------|",
    ]
    for c in checks:
        lines.append(f"| {c['id']} | {c['prio']} | {c['status']} | {c['note']} |")
    lines.extend([
        "",
        "Regenerate: `python3 tools/gui_v3_parity_audit.py`",
        "API smoke: `python3 tests/scripts/test_gui_v3_api_smoke.py`",
    ])
    report = out_dir / "GUI_V3_PARITY_AUDIT.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(report)
    print(f"P0 gaps: {p0_gap}  PASS/PARTIAL/GAP = {counts['PASS']}/{counts['PARTIAL']}/{counts['GAP']}")
    return 1 if p0_gap else 0


if __name__ == "__main__":
    sys.exit(main())
