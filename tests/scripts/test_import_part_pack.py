#!/usr/bin/env python3
"""v0.3 M5 — external catalog pack inspect/install stub."""
import shutil
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "python"))

from catalog.pack_import import inspect_part_pack, install_part_pack  # noqa: E402
from catalog.resolver import CatalogError  # noqa: E402


def _write_pack(root: Path, pid: str, bid: str) -> None:
    (root / "parts").mkdir(parents=True, exist_ok=True)
    (root / "blueprints").mkdir(parents=True, exist_ok=True)
    manifest = {
        "package_id": "demo.pack",
        "package_version": 1,
        "parts": [{"id": pid, "path": f"parts/{pid.replace('.', '_')}.yaml"}],
        "blueprints": [{"id": bid, "path": f"blueprints/{bid.replace('.', '_')}.yaml"}],
    }
    (root / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))


def main():
    with tempfile.TemporaryDirectory() as td:
        ok_root = Path(td) / "ok_pack"
        _write_pack(ok_root, "tire.demo_pack", "vehicle.demo_pack")
        rep = inspect_part_pack(ok_root, REPO)
        assert rep["ok"]
        assert "tire.demo_pack" in rep["parts"]

        bad_root = Path(td) / "bad_pack"
        _write_pack(bad_root, "tire.default_pacejka", "vehicle.demo_bad")
        rep2 = inspect_part_pack(bad_root, REPO)
        assert not rep2["ok"]
        assert rep2["collisions"]

        inst_root = Path(td) / "install_pack"
        _write_pack(inst_root, "tire.pack_install", "vehicle.pack_install")
        dest = install_part_pack(inst_root, REPO, dest_name="test_pack_install")
        assert dest.is_dir()
        shutil.rmtree(dest)

    try:
        install_part_pack(bad_root, REPO)
        raise AssertionError("expected collision install to fail")
    except CatalogError:
        pass

    print("test_import_part_pack: ok")


if __name__ == "__main__":
    main()
