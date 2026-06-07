from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Set

import yaml

from .resolver import CatalogError, CatalogResolver


def _read_manifest(path: Path) -> dict:
    if not path.is_file():
        raise CatalogError(f"pack manifest not found: {path}")
    doc = yaml.safe_load(path.read_text()) or {}
    if not isinstance(doc, dict):
        raise CatalogError("pack manifest root must be a mapping")
    if not doc.get("package_id"):
        raise CatalogError("pack manifest requires package_id")
    return doc


def _ids_from_manifest(doc: dict) -> Dict[str, Set[str]]:
    parts = {str(e["id"]) for e in doc.get("parts") or [] if e.get("id")}
    blueprints = {str(e["id"]) for e in doc.get("blueprints") or [] if e.get("id")}
    return {"parts": parts, "blueprints": blueprints}


def inspect_part_pack(pack_root: Path, repo_root: Path) -> dict:
    pack_root = Path(pack_root).resolve()
    manifest_path = pack_root / "manifest.yaml"
    pack_doc = _read_manifest(manifest_path)
    pack_ids = _ids_from_manifest(pack_doc)

    builtin = CatalogResolver(repo_root)
    builtin.load_manifest()
    builtin_parts = {p["id"] for p in builtin.list_parts()}
    builtin_blueprints = {b["id"] for b in builtin.list_blueprints()}

    collisions: List[str] = []
    for pid in sorted(pack_ids["parts"]):
        if pid in builtin_parts:
            collisions.append(f"part:{pid}")
    for bid in sorted(pack_ids["blueprints"]):
        if bid in builtin_blueprints:
            collisions.append(f"blueprint:{bid}")

    return {
        "package_id": pack_doc.get("package_id"),
        "package_version": pack_doc.get("package_version"),
        "root": str(pack_root),
        "parts": sorted(pack_ids["parts"]),
        "blueprints": sorted(pack_ids["blueprints"]),
        "collisions": collisions,
        "ok": not collisions,
    }


def install_part_pack(pack_root: Path, repo_root: Path, dest_name: str | None = None) -> Path:
    report = inspect_part_pack(pack_root, repo_root)
    if not report["ok"]:
        raise CatalogError("id collision with builtin catalog: " + ", ".join(report["collisions"]))
    pack_root = Path(pack_root).resolve()
    name = dest_name or str(report["package_id"])
    dest = Path(repo_root) / "configs" / "catalog" / "packages" / name
    if dest.exists():
        raise CatalogError(f"package already installed: {dest}")
    import shutil
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(pack_root, dest)
    return dest
