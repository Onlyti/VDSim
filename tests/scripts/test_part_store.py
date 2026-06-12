#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "build" / "python"))

from catalog.part_store import (
    USER_PACKAGE_ID,
    blueprint_from_fleet,
    delete_user_part,
    kin_yaml_to_susp_part,
    new_part_doc,
    parse_part_yaml,
    save_user_blueprint,
    save_user_kin_part,
    save_user_part,
    user_package_root,
)
from catalog.resolver import CatalogResolver

TEST_STEM = "pytest_gui_part"
TEST_ID = f"body.{TEST_STEM}"


def _cleanup(repo: Path) -> None:
    pkg = user_package_root(repo)
    part_path = pkg / "parts" / "body" / f"{TEST_STEM}.yaml"
    if part_path.is_file():
        part_path.unlink()
    manifest = pkg / "manifest.yaml"
    if manifest.is_file():
        import yaml
        doc = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
        doc["parts"] = [e for e in (doc.get("parts") or [])
                        if str(e.get("id")) != TEST_ID]
        manifest.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    r = CatalogResolver(repo)
    r.clear_cache()


def test_save_and_reload_body():
    _cleanup(ROOT)
    doc = new_part_doc("body", TEST_STEM, "Pytest body")
    doc["body"]["mass"] = 1420.0
    out = save_user_part(ROOT, doc)
    assert out["part_id"] == TEST_ID
    assert out["package"] == USER_PACKAGE_ID
    r = CatalogResolver(ROOT)
    r.clear_cache()
    loaded = r.load_part(TEST_ID)
    assert float(loaded["body"]["mass"]) == 1420.0
    assert any(p["id"] == TEST_ID for p in r.list_parts("body"))
    _cleanup(ROOT)


def test_save_brake_with_meta():
    _cleanup(ROOT)
    stem = "pytest_brake"
    pid = f"brake.{stem}"
    doc = new_part_doc("brake", stem, "Pytest brake")
    doc = __import__("catalog.part_store", fromlist=["apply_editor_fields"]).apply_editor_fields(
        doc, {"extra_tags": "sport, race", "ui_tier": "sport", "ui_blurb": "test brake"})
    save_user_part(ROOT, doc)
    r = CatalogResolver(ROOT)
    r.clear_cache()
    loaded = r.load_part(pid)
    assert "sport" in loaded.get("tags", [])
    assert loaded.get("ui", {}).get("tier") == "sport"
    path = user_package_root(ROOT) / "parts" / "brake" / f"{stem}.yaml"
    if path.is_file():
        path.unlink()
    import yaml
    manifest = user_package_root(ROOT) / "manifest.yaml"
    mdoc = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
    mdoc["parts"] = [e for e in (mdoc.get("parts") or []) if str(e.get("id")) != pid]
    manifest.write_text(yaml.safe_dump(mdoc, sort_keys=False), encoding="utf-8")
    r.clear_cache()


def test_import_kin_and_blueprint():
    _cleanup(ROOT)
    kin_path = ROOT / "configs" / "parts" / "susp_kinematics" / "kin" / "mp_front_sedan.yaml"
    kin_text = kin_path.read_text(encoding="utf-8")
    doc, kin = kin_yaml_to_susp_part(kin_text, "pytest_kin", "Pytest kin")
    out = save_user_kin_part(ROOT, doc, kin)
    assert out["part_id"] == "chassis.pytest_kin"
    r = CatalogResolver(ROOT)
    r.clear_cache()
    loaded = r.load_part("chassis.pytest_kin")
    assert "path" in loaded.get("body", {})

    spec = {
        "level": "L2",
        "parts": {
            "body": "body.sedan",
            "aero": "aero.sedan",
            "ride": "ride.sedan",
            "tire": "tire.default_pacejka",
            "brake": "brake.sedan",
            "steering": "steering.sedan",
            "drivetrain": "drivetrain.sedan",
        },
    }
    bp = blueprint_from_fleet(spec, "pytest_build", "Pytest build")
    bout = save_user_blueprint(ROOT, bp)
    assert bout["blueprint_id"] == "vehicle.pytest_build"
    r.clear_cache()
    assert r.load_blueprint("vehicle.pytest_build")["parts"]["tire"] == "tire.default_pacejka"

    delete_user_part(ROOT, "chassis.pytest_kin")
    bp_path = user_package_root(ROOT) / "blueprints" / "pytest_build.yaml"
    if bp_path.is_file():
        bp_path.unlink()
    import yaml
    manifest = user_package_root(ROOT) / "manifest.yaml"
    mdoc = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
    mdoc["parts"] = [e for e in (mdoc.get("parts") or []) if str(e.get("id")) != "chassis.pytest_kin"]
    mdoc["blueprints"] = [e for e in (mdoc.get("blueprints") or [])
                          if str(e.get("id")) != "vehicle.pytest_build"]
    manifest.write_text(yaml.safe_dump(mdoc, sort_keys=False), encoding="utf-8")
    r.clear_cache()


def test_import_yaml_roundtrip():
    _cleanup(ROOT)
    doc = new_part_doc("tire", "pytest_tire_tmp", "Tmp tire")
    text = __import__("yaml").safe_dump(doc, sort_keys=False)
    parsed = parse_part_yaml(text)
    parsed["id"] = "tire.pytest_tire_tmp"
    save_user_part(ROOT, parsed)
    r = CatalogResolver(ROOT)
    r.clear_cache()
    assert r.load_part("tire.pytest_tire_tmp")["type"] == "tire"
    part_path = user_package_root(ROOT) / "parts" / "tire" / "pytest_tire_tmp.yaml"
    if part_path.is_file():
        part_path.unlink()
    import yaml
    manifest = user_package_root(ROOT) / "manifest.yaml"
    if manifest.is_file():
        mdoc = yaml.safe_load(manifest.read_text(encoding="utf-8")) or {}
        mdoc["parts"] = [e for e in (mdoc.get("parts") or [])
                         if str(e.get("id")) != "tire.pytest_tire_tmp"]
        manifest.write_text(yaml.safe_dump(mdoc, sort_keys=False), encoding="utf-8")
    r.clear_cache()


if __name__ == "__main__":
    test_save_and_reload_body()
    test_save_brake_with_meta()
    test_import_kin_and_blueprint()
    test_import_yaml_roundtrip()
    print("ok")
