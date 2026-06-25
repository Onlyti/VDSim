#!/usr/bin/env python3
"""Run headless GUI v3 evaluation bundle (no human)."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def run(cmd: list[str], cwd: Path | None = None, env: dict | None = None) -> dict:
    p = subprocess.run(cmd, cwd=cwd or REPO, capture_output=True, text=True, env=env)
    return {
        "cmd": " ".join(cmd),
        "exit_code": p.returncode,
        "stdout_tail": "\n".join((p.stdout or "").splitlines()[-8:]),
        "stderr_tail": "\n".join((p.stderr or "").splitlines()[-8:]),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-npm", action="store_true")
    ap.add_argument("--skip-ctest", action="store_true")
    ap.add_argument("--skip-e2e", action="store_true", help="skip Playwright headless browser")
    args = ap.parse_args()

    day = date.today().isoformat()
    out_dir = REPO / "docs" / "evidence" / "review" / day
    out_dir.mkdir(parents=True, exist_ok=True)

    results: dict = {"date": day, "steps": []}
    ok = True

    py_env = {
        **dict(__import__("os").environ),
        "PYTHONPATH": ":".join([
            str(REPO / "build" / "python"),
            str(REPO / "python"),
            str(REPO / "gui"),
            str(REPO / "cosim"),
        ]),
    }

    if not args.skip_npm:
        for label, cmd in (
            ("npm_check", ["npm", "run", "check"]),
            ("npm_build", ["npm", "run", "build"]),
        ):
            r = run(cmd, cwd=REPO / "gui" / "v3")
            r["step"] = label
            results["steps"].append(r)
            ok = ok and r["exit_code"] == 0

    r = run([sys.executable, "tests/scripts/test_gui_v3_api_smoke.py"], env=py_env)
    r["step"] = "gui_v3_api_smoke"
    results["steps"].append(r)
    ok = ok and r["exit_code"] == 0

    if not getattr(args, "skip_e2e", False):
        r = run([sys.executable, "tests/scripts/test_gui_v3_e2e.py"], env=py_env)
        r["step"] = "gui_v3_e2e"
        results["steps"].append(r)
        ok = ok and r["exit_code"] == 0

    r = run([sys.executable, "tools/gui_v3_parity_audit.py", "--out-dir", str(out_dir)])
    r["step"] = "parity_audit"
    results["steps"].append(r)
    parity_exit = r["exit_code"]
    results["p0_gaps_blocking"] = parity_exit != 0

    if not args.skip_ctest:
        build = REPO / "build"
        if (build / "CTestTestfile.cmake").is_file():
            for name, regex in (
                ("ctest_assembly", "assembly"),
                ("ctest_catalog", "catalog"),
                ("ctest_compare", "multi_vehicle_compare"),
            ):
                r = run(["ctest", "-R", regex, "--output-on-failure"], cwd=build)
                r["step"] = name
                results["steps"].append(r)
                ok = ok and r["exit_code"] == 0
        else:
            results["steps"].append({
                "step": "ctest_skipped",
                "exit_code": 0,
                "note": "build/ missing — run cmake",
            })

    (out_dir / "gui_v3_eval_bundle.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8")

    lines = [
        f"# GUI v3 eval bundle — {day}",
        "",
        f"Overall automation: **{'PASS' if ok else 'FAIL'}**",
        f"P0 parity gaps (static): **{'yes' if parity_exit else 'no'}**",
        "",
        "| Step | Exit |",
        "|------|------|",
    ]
    for s in results["steps"]:
        lines.append(f"| {s.get('step', '?')} | {s.get('exit_code', '?')} |")
    lines.extend([
        "",
        "Artifacts:",
        f"- `{out_dir / 'gui_v3_parity_matrix.json'}`",
        f"- `{out_dir / 'GUI_V3_PARITY_AUDIT.md'}`",
        f"- `{out_dir / 'gui_v3_eval_bundle.json'}`",
        "",
        "Run: `python3 tools/gui_v3_eval_bundle.py`",
    ])
    (out_dir / "GUI_V3_EVAL_BUNDLE.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(out_dir / "GUI_V3_EVAL_BUNDLE.md")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
