#!/usr/bin/env python3
"""Adams CSV import + K&C gain cross-check vs reference VDSim kin YAML."""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "build" / "python"))
sys.path.insert(0, str(REPO / "tools" / "kinematics"))

import vdsim  # noqa: E402
from import_hardpoints import import_csv_to_yaml  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Adams hardpoint import K&C X-check")
    ap.add_argument("--csv", required=True, type=Path, help="Adams-style CSV")
    ap.add_argument("--reference", required=True, type=Path,
                    help="Reference kin YAML (catalog or golden)")
    ap.add_argument("--rtol", type=float, default=0.05,
                    help="Relative tolerance per gain metric (default 5%%)")
    ap.add_argument("--type", default="auto")
    ap.add_argument("--side", default="left", choices=["left", "right"])
    ap.add_argument("--wheel-radius", type=float, default=0.33)
    ap.add_argument("--json", action="store_true", help="Print JSON report")
    args = ap.parse_args()

    if not args.reference.is_file():
        print(f"reference not found: {args.reference}", file=sys.stderr)
        return 2
    if not args.csv.is_file():
        print(f"csv not found: {args.csv}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="vdsim_adams_xcheck_") as td:
        imported = Path(td) / "imported.yaml"
        import_csv_to_yaml(args.csv, imported, args.type, args.side,
                           args.wheel_radius)
        rep = vdsim.run_kc_xcheck(str(args.reference), str(imported), args.rtol)

    if args.json:
        payload = {
            "all_ok": rep.all_ok,
            "deltas": [
                {
                    "name": d.name,
                    "reference": d.reference,
                    "candidate": d.candidate,
                    "rel_error": d.rel_error,
                    "ok": d.ok,
                }
                for d in rep.deltas
            ],
        }
        print(json.dumps(payload, indent=2))
    else:
        print(f"Adams X-check: {'PASS' if rep.all_ok else 'FAIL'}")
        for d in rep.deltas:
            mark = "ok" if d.ok else "FAIL"
            print(f"  [{mark}] {d.name}: ref={d.reference:.6g} "
                  f"cand={d.candidate:.6g} rel={d.rel_error:.4g}")
    return 0 if rep.all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
