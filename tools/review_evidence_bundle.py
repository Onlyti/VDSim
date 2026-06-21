#!/usr/bin/env python3
"""Collect ctest + lane smokes into docs/evidence/review/ for audit trail."""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

LANES = [
    ("validation", ["ChronoKcParity", "IsoBaseline"]),
    ("core_session", ["PerWheel", "PerAxle", "SimSession", "DragCoast"]),
    ("catalog", ["catalog", "assembly", "blueprint", "multi_vehicle"]),
]


def run_ctest(build: Path, regex: str) -> dict:
    cmd = ["ctest", "-R", regex, "--output-on-failure"]
    p = subprocess.run(cmd, cwd=build, capture_output=True, text=True)
    out = (p.stdout or "") + (p.stderr or "")
    passed = failed = 0
    for line in out.splitlines():
        if "tests passed" in line and "failed" in line:
            m = re.search(r"(\d+)\s*/\s*(\d+)\s+tests passed", line)
            if m:
                passed, failed = int(m.group(1)), int(m.group(2)) - int(m.group(1))
            else:
                m2 = re.search(r"(\d+)\s+tests passed,\s*(\d+)\s+tests failed", line)
                if m2:
                    passed, failed = int(m2.group(1)), int(m2.group(2))
                elif line.strip().startswith("100%"):
                    passed, failed = 1, 0
    return {"regex": regex, "exit_code": p.returncode, "passed": passed,
            "failed": failed, "log_tail": "\n".join(out.splitlines()[-12:])}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", type=Path, default=REPO / "build")
    ap.add_argument("--tag", default=date.today().isoformat())
    args = ap.parse_args()
    if not (args.build / "CTestTestfile.cmake").is_file():
        print("build dir missing; run cmake first", file=sys.stderr)
        return 1

    day = args.tag if args.tag.count("-") == 2 else date.today().isoformat()
    out_dir = REPO / "docs" / "evidence" / "review" / day
    out_dir.mkdir(parents=True, exist_ok=True)

    matrix = {"tag": args.tag, "date": day, "lanes": {}}
    summary_lines = [f"# Review evidence — {day}", "", f"Tag: `{args.tag}`", ""]

    for lane, patterns in LANES:
        lane_res = []
        for pat in patterns:
            lane_res.append(run_ctest(args.build, pat))
        matrix["lanes"][lane] = lane_res
        ok = all(r["exit_code"] == 0 and r["failed"] == 0 for r in lane_res)
        summary_lines.append(f"## Lane: {lane} — {'PASS' if ok else 'FAIL'}")
        for r in lane_res:
            summary_lines.append(
                f"- `{r['regex']}`: {r['passed']} passed, {r['failed']} failed "
                f"(exit {r['exit_code']})")
        summary_lines.append("")

    full = run_ctest(args.build, ".")
    matrix["full_ctest"] = full
    summary_lines.append("## Full ctest")
    summary_lines.append(
        f"- passed={full['passed']} failed={full['failed']} exit={full['exit_code']}")
    summary_lines.append("")
    summary_lines.append("Regenerate: `python3 tools/review_evidence_bundle.py`")

    (out_dir / "verification_matrix.json").write_text(
        json.dumps(matrix, indent=2) + "\n")
    (out_dir / "ctest_summary.txt").write_text(
        "\n".join(summary_lines) + "\n")
    report = REPO / "docs" / "evidence" / "review" / f"REVIEW_{day}.md"
    if not report.exists():
        report.write_text("\n".join(summary_lines) + "\n")

    print(report)
    return 0 if full["exit_code"] == 0 and full["failed"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
