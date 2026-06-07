#!/usr/bin/env python3
"""Inspect or install an external catalog package (v0.3 M5 stub).

Pack layout:
  my_pack/
    manifest.yaml    # package_id, parts[], blueprints[]
    parts/...
    blueprints/...

Default is dry-run (validate + collision check only).
"""
import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "python"))

from catalog.pack_import import inspect_part_pack, install_part_pack  # noqa: E402
from catalog.resolver import CatalogError  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="VDSim external catalog pack importer")
    ap.add_argument("pack", type=Path, help="path to unpacked catalog package root")
    ap.add_argument("--install", action="store_true",
                    help="copy pack to configs/catalog/packages/<package_id>")
    ap.add_argument("--name", help="override install directory name")
    ap.add_argument("--json", action="store_true", help="print report as JSON")
    args = ap.parse_args()

    try:
        if args.install:
            dest = install_part_pack(args.pack, REPO, dest_name=args.name)
            out = {"ok": True, "installed": str(dest)}
        else:
            out = inspect_part_pack(args.pack, REPO)
    except CatalogError as e:
        print(f"import_part_pack: {e}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(out, indent=2))
    else:
        if args.install:
            print(f"installed: {out['installed']}")
        else:
            print(f"package: {out['package_id']}")
            print(f"parts: {len(out['parts'])}  blueprints: {len(out['blueprints'])}")
            if out["collisions"]:
                print("collisions:", ", ".join(out["collisions"]))
            else:
                print("collisions: none")
            print("ok:", out["ok"])
    return 0 if out.get("ok", True) else 2


if __name__ == "__main__":
    raise SystemExit(main())
