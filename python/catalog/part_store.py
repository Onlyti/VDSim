from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

import yaml

from catalog.resolver import CatalogError, CatalogResolver, PartEnvelopeError

USER_PACKAGE_ID = "gui_custom"
_STEM_RE = re.compile(r"^[a-z][a-z0-9_]{0,47}$")

CHASSIS_BODY_KEYS = (
    "mass", "mass_sprung", "cg_height", "cg_to_front", "cg_to_rear",
    "wheelbase", "track_front", "track_rear", "wheel_radius_nominal",
    "inertia_diag", "spring_stiffness", "damper_coefficient", "unsprung_mass",
    "aero_drag_coeff", "frontal_area", "aero_lift_front", "aero_lift_rear",
)

TIRE_BODY_KEYS = (
    "mu_nominal", "Fz_nominal", "B_long", "C_long", "D_long", "E_long",
    "B_lat", "C_lat", "D_lat", "E_lat", "rolling_resistance",
    "cornering_stiffness", "load_sensitivity",
    "relaxation_length_lat", "relaxation_length_long",
)

EDITABLE_TYPES = frozenset({
    "chassis", "tire", "brake", "steering", "drivetrain", "susp_kinematics",
})

_DEFAULT_CLONE: Dict[str, str] = {
    "chassis": "chassis.sedan",
    "tire": "tire.default_pacejka",
    "brake": "brake.sedan",
    "steering": "steering.sedan",
    "drivetrain": "drivetrain.sedan",
    "susp_kinematics": "susp.mp_front_sedan",
}

META_FIELDS: List[Tuple[str, str, str]] = [
    ("extra_tags", "Tags (comma-separated)", "text"),
    ("ui_tier", "Tier (oem / sport / race / stunt)", "text"),
    ("ui_blurb", "Short description", "text"),
]

EDITOR_SCHEMAS: Dict[str, Dict[str, Any]] = {
    "chassis": {
        "schema": "vehicle_params_v1",
        "fields": [
            ("mass", "Mass [kg]", "num"),
            ("cg_height", "CG height [m]", "num"),
            ("cg_to_front", "CG to front [m]", "num"),
            ("cg_to_rear", "CG to rear [m]", "num"),
            ("track_front", "Track front [m]", "num"),
            ("track_rear", "Track rear [m]", "num"),
            ("wheel_radius_nominal", "Wheel radius [m]", "num"),
            ("aero_drag_coeff", "Drag coeff", "num"),
            ("frontal_area", "Frontal area [m²]", "num"),
        ],
        "array_fields": [
            ("spring_stiffness", "Spring k [N/m] FL,FR,RL,RR", 4),
            ("damper_coefficient", "Damper c [N·s/m] FL,FR,RL,RR", 4),
            ("inertia_diag", "Inertia Ixx,Iyy,Izz [kg·m²]", 3),
        ],
    },
    "tire": {
        "schema": "pacejka_mf96_v1",
        "fields": [
            ("mu_nominal", "μ nominal", "num"),
            ("Fz_nominal", "Fz nominal [N]", "num"),
            ("B_long", "B long", "num"),
            ("C_long", "C long", "num"),
            ("D_long", "D long", "num"),
            ("E_long", "E long", "num"),
            ("B_lat", "B lat", "num"),
            ("C_lat", "C lat", "num"),
            ("D_lat", "D lat", "num"),
            ("E_lat", "E lat", "num"),
            ("rolling_resistance", "Rolling resistance", "num"),
        ],
        "bool_fields": [
            ("lugre.enabled", "LuGre enabled"),
        ],
    },
    "brake": {
        "schema": "brake_subsystem_v1",
        "fields": [
            ("max_brake_torque", "Max brake torque [N·m]", "num"),
            ("brake_bias_front", "Front brake bias", "num"),
        ],
    },
    "steering": {
        "schema": "steering_subsystem_v1",
        "fields": [
            ("steering_ratio", "Steering ratio", "num"),
            ("max_steer_angle_wheel", "Max steer angle [rad]", "num"),
            ("ackerman_percent", "Ackermann %", "num"),
        ],
    },
    "drivetrain": {
        "schema": "drivetrain_v1",
        "fields": [
            ("drive_type", "Drive type", "text"),
            ("differential", "Differential", "text"),
            ("max_motor_torque", "Max motor torque [N·m]", "num"),
            ("final_drive_ratio", "Final drive ratio", "num"),
            ("engine_rotational_inertia", "Engine inertia [kg·m²]", "num"),
            ("lsd_preload", "LSD preload", "num"),
            ("lsd_ramp", "LSD ramp", "num"),
        ],
    },
    "susp_kinematics": {
        "schema": "kinematics_l3_native_v1",
        "fields": [
            ("path", "Kinematics YAML path (repo-relative)", "text"),
        ],
    },
}


def user_package_root(repo_root: Path) -> Path:
    return Path(repo_root) / "configs" / "catalog" / "packages" / USER_PACKAGE_ID


def _catalog_root(repo_root: Path) -> Path:
    return Path(repo_root) / "configs"


def _main_manifest_path(repo_root: Path) -> Path:
    return _catalog_root(repo_root) / "catalog" / "manifest.yaml"


def _package_manifest_path(repo_root: Path) -> Path:
    return user_package_root(repo_root) / "manifest.yaml"


def normalize_stem(stem: str) -> str:
    s = str(stem).strip().lower().replace("-", "_").replace(" ", "_")
    s = re.sub(r"[^a-z0-9_]", "", s)
    if not s or not _STEM_RE.match(s):
        raise CatalogError(
            "part stem must be 1–48 chars: lowercase letters, digits, underscore")
    return s


def part_id_for(type_name: str, stem: str) -> str:
    t = str(type_name).strip()
    if t not in EDITABLE_TYPES:
        raise CatalogError(f"part editor unsupported type: {t}")
    if t == "susp_kinematics":
        return f"susp.{normalize_stem(stem)}"
    return f"{t}.{normalize_stem(stem)}"


def ensure_user_package(repo_root: Path) -> Path:
    root = user_package_root(repo_root)
    root.mkdir(parents=True, exist_ok=True)
    pkg_manifest = _package_manifest_path(repo_root)
    if not pkg_manifest.is_file():
        doc = {
            "package_id": USER_PACKAGE_ID,
            "package_version": 1,
            "label": "GUI-registered parts",
            "parts": [],
            "blueprints": [],
        }
        pkg_manifest.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    _register_package_in_main_manifest(repo_root)
    return root


def _register_package_in_main_manifest(repo_root: Path) -> None:
    main = _main_manifest_path(repo_root)
    doc = yaml.safe_load(main.read_text(encoding="utf-8")) or {}
    packages = list(doc.get("packages") or [])
    rel = f"catalog/packages/{USER_PACKAGE_ID}/manifest.yaml"
    if not any(str(p.get("id")) == USER_PACKAGE_ID for p in packages):
        packages.append({"id": USER_PACKAGE_ID, "path": rel})
        doc["packages"] = packages
        main.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")


def _load_pkg_manifest(repo_root: Path) -> dict:
    return yaml.safe_load(_package_manifest_path(repo_root).read_text(encoding="utf-8")) or {}


def _write_pkg_manifest(repo_root: Path, doc: dict) -> None:
    _package_manifest_path(repo_root).write_text(
        yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")


def _part_path_in_package(repo_root: Path, type_name: str, stem: str) -> Path:
    return user_package_root(repo_root) / "parts" / type_name / f"{normalize_stem(stem)}.yaml"


def _is_user_part(repo_root: Path, part_id: str) -> bool:
    pkg = _load_pkg_manifest(repo_root)
    return any(str(e.get("id")) == part_id for e in (pkg.get("parts") or []))


def new_part_doc(
    type_name: str,
    stem: str,
    label: str,
    *,
    clone_from: Optional[Mapping[str, Any]] = None,
) -> dict:
    t = str(type_name)
    if t not in EDITOR_SCHEMAS:
        raise CatalogError(f"unsupported part type: {t}")
    meta = EDITOR_SCHEMAS[t]
    pid = part_id_for(t, stem)
    if clone_from:
        doc = copy.deepcopy(dict(clone_from))
        doc["id"] = pid
        doc["label"] = str(label)
        doc["type"] = t
        doc["schema"] = meta["schema"]
        doc.setdefault("version", 1)
        doc.setdefault("tags", [t, USER_PACKAGE_ID])
        doc["body"] = copy.deepcopy(dict(clone_from.get("body") or {}))
        return doc
    body: Dict[str, Any] = {}
    if t == "chassis":
        resolver = CatalogResolver(Path(__file__).resolve().parents[2])
        try:
            ref = resolver.load_part("chassis.sedan")
            body = copy.deepcopy(dict(ref.get("body") or {}))
        except CatalogError:
            body = {k: 0.0 for k in CHASSIS_BODY_KEYS if k not in (
                "spring_stiffness", "damper_coefficient", "unsprung_mass", "inertia_diag")}
            body["spring_stiffness"] = [30000.0] * 4
            body["damper_coefficient"] = [3000.0] * 4
            body["unsprung_mass"] = [40.0] * 4
            body["inertia_diag"] = [500.0, 2000.0, 2500.0]
            body["mass"] = 1500.0
            body["cg_height"] = 0.55
    elif t == "tire":
        resolver = CatalogResolver(Path(__file__).resolve().parents[2])
        try:
            ref = resolver.load_part("tire.default_pacejka")
            body = copy.deepcopy(dict(ref.get("body") or {}))
        except CatalogError:
            body = {"mu_nominal": 1.0, "Fz_nominal": 4000.0, "lugre": {"enabled": False}}
    else:
        resolver = CatalogResolver(Path(__file__).resolve().parents[2])
        ref_id = _DEFAULT_CLONE.get(t, "")
        try:
            ref = resolver.load_part(ref_id) if ref_id else None
            body = copy.deepcopy(dict(ref.get("body") or {})) if ref else {}
        except CatalogError:
            body = {}
    return {
        "id": pid,
        "type": t,
        "version": 1,
        "schema": meta["schema"],
        "label": str(label),
        "tags": [t, USER_PACKAGE_ID],
        "body": body,
    }


def _merge_tags(doc: dict, extra_tags: str) -> None:
    base = {str(doc.get("type", "")), USER_PACKAGE_ID}
    merged = list(base)
    for raw in str(extra_tags).split(","):
        t = raw.strip().lower()
        if t and t not in merged:
            merged.append(t)
    doc["tags"] = merged


def _apply_meta_fields(doc: dict, fields: Mapping[str, Any]) -> dict:
    out = copy.deepcopy(doc)
    if "extra_tags" in fields:
        _merge_tags(out, str(fields["extra_tags"]))
    ui = dict(out.get("ui") or {})
    if fields.get("ui_tier"):
        ui["tier"] = str(fields["ui_tier"]).strip().lower()
    if "ui_blurb" in fields:
        ui["blurb"] = str(fields["ui_blurb"])
    if ui:
        out["ui"] = ui
    return out


def apply_editor_fields(doc: dict, fields: Mapping[str, Any]) -> dict:
    out = _apply_meta_fields(doc, fields)
    body = dict(out.get("body") or {})
    for key, val in fields.items():
        if key in ("extra_tags", "ui_tier", "ui_blurb"):
            continue
        if key == "label":
            out["label"] = str(val)
            continue
        if key == "stem":
            out["id"] = part_id_for(out["type"], str(val))
            continue
        if "." in key:
            head, tail = key.split(".", 1)
            node = dict(body.get(head) or {})
            if isinstance(node, dict):
                node[tail] = val
                body[head] = node
            continue
        if isinstance(val, list):
            body[key] = [float(x) for x in val]
        elif isinstance(val, bool):
            body[key] = bool(val)
        elif isinstance(val, str) and key not in ("drive_type", "differential", "path"):
            try:
                body[key] = float(val)
            except (TypeError, ValueError):
                body[key] = val
        else:
            body[key] = val
    out["body"] = body
    return out


def parse_part_yaml(text: str) -> dict:
    doc = yaml.safe_load(text)
    if not isinstance(doc, dict):
        raise PartEnvelopeError("YAML root must be a mapping")
    CatalogResolver.validate_part_envelope(doc)
    if doc["type"] not in EDITABLE_TYPES:
        raise CatalogError(f"import unsupported type: {doc['type']}")
    return doc


def kin_yaml_to_susp_part(
    text: str,
    stem: str,
    label: str,
    *,
    clone_from: Optional[dict] = None,
) -> dict:
    kin = yaml.safe_load(text)
    if not isinstance(kin, dict):
        raise CatalogError("kinematics YAML root must be a mapping")
    if not kin.get("type"):
        raise CatalogError("kinematics YAML needs top-level type (e.g. macpherson)")
    base = clone_from or new_part_doc("susp_kinematics", stem, label)
    base["id"] = part_id_for("susp_kinematics", stem)
    base["label"] = str(label)
    return base, kin


def save_user_kin_part(
    repo_root: Path,
    doc: Mapping[str, Any],
    kin_body: Mapping[str, Any],
) -> dict:
    ensure_user_package(repo_root)
    part = copy.deepcopy(dict(doc))
    stem = part["id"].split(".", 1)[1]
    kin_dir = user_package_root(repo_root) / "parts" / "susp_kinematics" / "kin"
    kin_dir.mkdir(parents=True, exist_ok=True)
    kin_path = kin_dir / f"{normalize_stem(stem)}.yaml"
    kin_path.write_text(yaml.safe_dump(dict(kin_body), sort_keys=False), encoding="utf-8")
    rel = f"catalog/packages/{USER_PACKAGE_ID}/parts/susp_kinematics/kin/{kin_path.name}"
    body = dict(part.get("body") or {})
    body["path"] = rel
    part["body"] = body
    return save_user_part(repo_root, part)


def tir_text_to_tire_part(
    text: str,
    stem: str,
    label: str,
    *,
    clone_from: Optional[dict] = None,
) -> dict:
    from tir_to_yaml import parse_tir_text, tir_to_params
    mapped = tir_to_params(parse_tir_text(text))
    if not mapped:
        raise CatalogError("no recognized .tir coefficients")
    base = clone_from or new_part_doc("tire", stem, label)
    body = dict(base.get("body") or {})
    body.update(mapped)
    base["body"] = body
    base["id"] = part_id_for("tire", stem)
    base["label"] = str(label)
    return base


def save_user_part(repo_root: Path, doc: Mapping[str, Any], *, allow_overwrite: bool = True) -> dict:
    ensure_user_package(repo_root)
    part = copy.deepcopy(dict(doc))
    CatalogResolver.validate_part_envelope(part)
    if part["type"] not in EDITABLE_TYPES:
        raise CatalogError(f"save unsupported type: {part['type']}")
    pid = str(part["id"])
    ptype = part["type"]
    if ptype == "susp_kinematics":
        if not pid.startswith("susp."):
            raise CatalogError("susp_kinematics id must start with susp.")
    elif not pid.startswith(ptype + "."):
        raise CatalogError(f"part id must start with {ptype}.")

    resolver = CatalogResolver(repo_root)
    resolver.load_manifest()
    exists = pid in resolver._part_index
    if exists and not _is_user_part(repo_root, pid):
        raise CatalogError(f"builtin part id collision: {pid}")
    if exists and not allow_overwrite:
        raise CatalogError(f"part already exists: {pid}")

    stem = pid.split(".", 1)[1]
    path = _part_path_in_package(repo_root, part["type"], stem)
    path.parent.mkdir(parents=True, exist_ok=True)
    rel = f"parts/{part['type']}/{path.name}"
    path.write_text(yaml.safe_dump(part, sort_keys=False), encoding="utf-8")

    pkg = _load_pkg_manifest(repo_root)
    entries = [e for e in (pkg.get("parts") or []) if str(e.get("id")) != pid]
    entries.append({"id": pid, "path": rel})
    entries.sort(key=lambda e: str(e.get("id")))
    pkg["parts"] = entries
    _write_pkg_manifest(repo_root, pkg)

    resolver.clear_cache()
    return {"part_id": pid, "path": str(path), "package": USER_PACKAGE_ID}


MODULE_KINDS = frozenset({"brake", "steering", "drivetrain", "suspension", "antirollbar"})


def save_module_plugin_part(
    repo_root: Path,
    kind: str,
    stem: str,
    label: str,
    so_path: str,
    *,
    axle: int = 0,
    allow_overwrite: bool = True,
) -> dict:
    """Register a built+checked C++ subsystem-module .so as a `module_plugin_v1` part.

    The part is filed under the user package as type `module`; `body.kind` carries the
    subsystem category. Module plugins are referenced by a blueprint's `module_plugins:`
    list (not the per-slot mechanism), so all five kinds register uniformly.
    """
    k = str(kind)
    if k not in MODULE_KINDS:
        raise CatalogError(f"unknown module kind: {k} (expected one of {sorted(MODULE_KINDS)})")
    ensure_user_package(repo_root)
    s = normalize_stem(stem)
    pid = f"module.{s}"
    body: Dict[str, Any] = {"kind": k, "plugin_so": str(so_path), "abi": 1}
    if k == "antirollbar":
        body["axle"] = int(axle)
    doc = {
        "id": pid,
        "type": "module",
        "version": 1,
        "schema": "module_plugin_v1",
        "label": str(label),
        "tags": ["module", k],
        "body": body,
    }
    CatalogResolver.validate_part_envelope(doc)

    resolver = CatalogResolver(repo_root)
    resolver.load_manifest()
    exists = pid in resolver._part_index
    if exists and not _is_user_part(repo_root, pid):
        raise CatalogError(f"builtin part id collision: {pid}")
    if exists and not allow_overwrite:
        raise CatalogError(f"part already exists: {pid}")

    path = _part_path_in_package(repo_root, "module", s)
    path.parent.mkdir(parents=True, exist_ok=True)
    rel = f"parts/module/{path.name}"
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")

    pkg = _load_pkg_manifest(repo_root)
    entries = [e for e in (pkg.get("parts") or []) if str(e.get("id")) != pid]
    entries.append({"id": pid, "path": rel})
    entries.sort(key=lambda e: str(e.get("id")))
    pkg["parts"] = entries
    _write_pkg_manifest(repo_root, pkg)

    resolver.clear_cache()
    return {"part_id": pid, "path": str(path), "package": USER_PACKAGE_ID, "kind": k}


def delete_user_part(repo_root: Path, part_id: str) -> dict:
    pid = str(part_id)
    if not _is_user_part(repo_root, pid):
        raise CatalogError(f"not a user-registered part: {pid}")
    pkg = _load_pkg_manifest(repo_root)
    entry = next((e for e in (pkg.get("parts") or []) if str(e.get("id")) == pid), None)
    if not entry:
        raise CatalogError(f"part not in user package manifest: {pid}")
    part_path = user_package_root(repo_root) / str(entry.get("path", ""))
    if part_path.is_file():
        try:
            doc = yaml.safe_load(part_path.read_text(encoding="utf-8")) or {}
            if doc.get("type") == "susp_kinematics":
                kin_rel = str((doc.get("body") or {}).get("path", ""))
                if kin_rel.startswith(f"catalog/packages/{USER_PACKAGE_ID}/"):
                    kin_fp = Path(repo_root) / "configs" / kin_rel
                    if kin_fp.is_file():
                        kin_fp.unlink()
        except Exception:
            pass
        part_path.unlink()
    entries = [e for e in (pkg.get("parts") or []) if str(e.get("id")) != pid]
    pkg["parts"] = entries
    _write_pkg_manifest(repo_root, pkg)
    CatalogResolver(repo_root).clear_cache()
    return {"deleted": pid, "package": USER_PACKAGE_ID}


def blueprint_from_fleet(spec: Mapping[str, Any], stem: str, label: str) -> dict:
    s = normalize_stem(stem)
    parts = dict(spec.get("parts") or {})
    return {
        "id": f"vehicle.{s}",
        "version": 1,
        "label": str(label),
        "level": str(spec.get("level", "L2")),
        "parts": parts,
        "overrides": {},
    }


def _is_user_blueprint(repo_root: Path, blueprint_id: str) -> bool:
    pkg = _load_pkg_manifest(repo_root)
    return any(str(e.get("id")) == blueprint_id for e in (pkg.get("blueprints") or []))


def save_user_blueprint(repo_root: Path, doc: Mapping[str, Any], *, allow_overwrite: bool = True) -> dict:
    ensure_user_package(repo_root)
    bp = copy.deepcopy(dict(doc))
    bid = str(bp.get("id", ""))
    if not bid.startswith("vehicle."):
        raise CatalogError("blueprint id must start with vehicle.")
    if "parts" not in bp or not isinstance(bp["parts"], Mapping):
        raise CatalogError("blueprint missing parts mapping")
    stem = bid.split(".", 1)[1]
    resolver = CatalogResolver(repo_root)
    resolver.load_manifest()
    exists = bid in resolver._blueprint_index
    if exists and not _is_user_blueprint(repo_root, bid):
        raise CatalogError(f"builtin blueprint id collision: {bid}")
    if exists and not allow_overwrite:
        raise CatalogError(f"blueprint already exists: {bid}")

    path = user_package_root(repo_root) / "blueprints" / f"{normalize_stem(stem)}.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    rel = f"blueprints/{path.name}"
    path.write_text(yaml.safe_dump(bp, sort_keys=False), encoding="utf-8")

    pkg = _load_pkg_manifest(repo_root)
    entries = [e for e in (pkg.get("blueprints") or []) if str(e.get("id")) != bid]
    entries.append({"id": bid, "path": rel})
    entries.sort(key=lambda e: str(e.get("id")))
    pkg["blueprints"] = entries
    _write_pkg_manifest(repo_root, pkg)
    resolver.clear_cache()
    return {"blueprint_id": bid, "path": str(path), "package": USER_PACKAGE_ID}


def editable_part_types() -> List[Dict[str, Any]]:
    out = []
    for t in sorted(EDITABLE_TYPES):
        schema = dict(EDITOR_SCHEMAS[t])
        schema["meta_fields"] = list(META_FIELDS)
        out.append({
            "type": t,
            "label": t.replace("_", " ").title(),
            "schema": schema,
            "import_kin": t == "susp_kinematics",
            "import_tir": t == "tire",
        })
    return out


def part_editor_payload(
    repo_root: Path,
    type_name: str,
    *,
    part_id: Optional[str] = None,
    stem: Optional[str] = None,
    label: Optional[str] = None,
    clone: bool = False,
) -> dict:
    resolver = CatalogResolver(repo_root)
    if part_id and (clone or (stem and label)):
        src = resolver.load_part(str(part_id))
        if src["type"] != type_name:
            raise CatalogError(f"part {part_id} is type {src['type']}, not {type_name}")
        doc = new_part_doc(
            type_name,
            stem or f"{src['id'].split('.', 1)[1]}_copy",
            label or f"{src.get('label', part_id)} (copy)",
            clone_from=src,
        )
    elif part_id:
        doc = resolver.load_part(str(part_id))
        if doc["type"] != type_name:
            raise CatalogError(f"part {part_id} is type {doc['type']}, not {type_name}")
    else:
        doc = new_part_doc(
            type_name,
            stem or "custom",
            label or f"Custom {type_name}",
        )
    schema = dict(EDITOR_SCHEMAS[type_name])
    schema["meta_fields"] = list(META_FIELDS)
    body = doc.get("body") or {}
    stem = doc["id"].split(".", 1)[1] if "." in doc["id"] else doc["id"]
    sys_tags = {str(type_name), USER_PACKAGE_ID}
    extra = [t for t in (doc.get("tags") or []) if t not in sys_tags]
    ui = doc.get("ui") or {}
    values: Dict[str, Any] = {
        "stem": stem,
        "label": doc.get("label", doc["id"]),
        "extra_tags": ", ".join(extra),
        "ui_tier": ui.get("tier", ""),
        "ui_blurb": ui.get("blurb", ""),
    }
    for key, _, _kind in schema.get("fields", []):
        if key in body:
            values[key] = body[key]
    for key, _, n in schema.get("array_fields", []):
        if key in body:
            values[key] = list(body[key])[:n]
    for dotted, _lab in schema.get("bool_fields", []):
        if "." in dotted:
            h, t = dotted.split(".", 1)
            node = body.get(h) or {}
            if isinstance(node, dict) and t in node:
                values[dotted] = bool(node[t])
    return {
        "doc": doc,
        "schema": schema,
        "values": values,
        "editable": True if (clone or not part_id) else _is_user_part(repo_root, doc["id"]),
        "yaml": yaml.safe_dump(doc, sort_keys=False),
    }
