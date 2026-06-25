from __future__ import annotations

import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import yaml

PART_TYPES = frozenset({
    # Re-taxonomy step 2: the old monolithic "chassis" (vehicle_params_v1) split
    # into body / aero / ride; "chassis" now denotes the suspension linkage (the
    # former susp_kinematics role). susp_kinematics kept in the allowed set for
    # any external package, but the built-in parts use "chassis".
    "body", "aero", "ride", "chassis",
    "tire", "susp_kinematics", "susp_topology", "susp_ride", "brake",
    "steering", "drivetrain", "powertrain", "actuator", "sensor_suite", "module",
})

SCHEMAS = frozenset({
    "vehicle_params_v1",
    "body_v1",
    "aero_v1",
    "ride_v1",
    "pacejka_mf96_v1",
    "kinematics_l3_native_v1",
    "spring_damper_v1",
    "brake_subsystem_v1",
    "steering_subsystem_v1",
    "drivetrain_v1",
    "drivetrain_v2",
    "drivetrain_v3",
    "powertrain_v1",
    "module_plugin_v1",
    "actuator_v1",
    "sensor_suite_v1",
    "topology_preview_v1",
})

L1L2_PART_SLOTS = ("body", "aero", "ride", "tire", "brake", "steering", "drivetrain")
L3_EXTRA_SLOTS = ("front_chassis", "rear_chassis")

ENVELOPE_KEYS = ("id", "type", "version", "schema", "label", "body")


class CatalogError(ValueError):
    pass


class PartEnvelopeError(CatalogError):
    pass


@dataclass
class ResolvedVehicle:
    blueprint_id: str
    level: str
    part_ids: Dict[str, str]
    vehicle_yaml: Path
    tire_yaml: Path
    tire_rear_yaml: Optional[Path] = None
    tire_fr_yaml: Optional[Path] = None
    tire_rl_yaml: Optional[Path] = None
    tire_rr_yaml: Optional[Path] = None
    susp_front: Optional[Path] = None
    susp_rear: Optional[Path] = None


@dataclass
class CatalogResolver:
    repo_root: Path
    manifest_path: Optional[Path] = None
    _manifest: Optional[dict] = field(default=None, init=False, repr=False)
    _part_index: Dict[str, Path] = field(default_factory=dict, init=False, repr=False)
    _blueprint_index: Dict[str, Path] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        self.repo_root = Path(self.repo_root).resolve()
        if self.manifest_path is None:
            wheel = self.repo_root / "vdsim_configs" / "catalog" / "manifest.yaml"
            dev = self.repo_root / "configs" / "catalog" / "manifest.yaml"
            self.manifest_path = wheel if wheel.is_file() else dev
        else:
            self.manifest_path = Path(self.manifest_path).resolve()
        self.catalog_root = self.manifest_path.parent.parent

    def load_manifest(self) -> dict:
        if self._manifest is not None:
            return self._manifest
        if not self.manifest_path.is_file():
            raise CatalogError(f"manifest not found: {self.manifest_path}")
        with open(self.manifest_path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        if not isinstance(doc, dict):
            raise CatalogError("manifest root must be a mapping")
        self._part_index.clear()
        for entry in doc.get("parts") or []:
            pid = entry.get("id")
            rel = entry.get("path")
            if not pid or not rel:
                raise CatalogError("manifest parts entry needs id and path")
            path = (self.catalog_root / rel).resolve()
            if pid in self._part_index:
                raise CatalogError(f"duplicate part id in manifest: {pid}")
            self._part_index[str(pid)] = path
        self._blueprint_index.clear()
        for entry in doc.get("blueprints") or []:
            bid = entry.get("id")
            rel = entry.get("path")
            if not bid or not rel:
                raise CatalogError("manifest blueprints entry needs id and path")
            path = (self.catalog_root / rel).resolve()
            if bid in self._blueprint_index:
                raise CatalogError(f"duplicate blueprint id in manifest: {bid}")
            self._blueprint_index[str(bid)] = path
        self._merge_packages(doc)
        self._manifest = doc
        return doc

    def clear_cache(self) -> None:
        self._manifest = None
        self._part_index.clear()
        self._blueprint_index.clear()

    def _merge_packages(self, doc: Mapping[str, Any]) -> None:
        catalog_root = self.catalog_root
        entries: List[dict] = list(doc.get("packages") or [])
        auto_root = catalog_root / "catalog" / "packages"
        if auto_root.is_dir():
            for d in sorted(auto_root.iterdir()):
                if not d.is_dir():
                    continue
                mf = d / "manifest.yaml"
                if not mf.is_file():
                    continue
                rel = f"catalog/packages/{d.name}/manifest.yaml"
                if not any(str(e.get("id")) == d.name for e in entries):
                    entries.append({"id": d.name, "path": rel})
        for entry in entries:
            rel = str(entry.get("path", ""))
            if not rel:
                continue
            pkg_manifest = (catalog_root / rel).resolve()
            if not pkg_manifest.is_file():
                continue
            pkg_doc = self._read_yaml(pkg_manifest)
            pkg_root = pkg_manifest.parent
            for part in pkg_doc.get("parts") or []:
                pid = str(part.get("id", ""))
                prel = str(part.get("path", ""))
                if not pid or not prel:
                    raise CatalogError(f"package part entry needs id and path: {pkg_manifest}")
                if pid in self._part_index:
                    raise CatalogError(f"duplicate part id across catalog: {pid}")
                self._part_index[pid] = (pkg_root / prel).resolve()
            for bp in pkg_doc.get("blueprints") or []:
                bid = str(bp.get("id", ""))
                brel = str(bp.get("path", ""))
                if not bid or not brel:
                    raise CatalogError(f"package blueprint entry needs id and path: {pkg_manifest}")
                if bid in self._blueprint_index:
                    raise CatalogError(f"duplicate blueprint id across catalog: {bid}")
                self._blueprint_index[bid] = (pkg_root / brel).resolve()

    def list_blueprints(self) -> List[dict]:
        self.load_manifest()
        out: List[dict] = []
        for bid, path in sorted(self._blueprint_index.items()):
            doc = self._read_yaml(path)
            if doc.get("id") != bid:
                raise CatalogError(f"blueprint id mismatch: {doc.get('id')!r} != {bid!r}")
            out.append({
                "id": bid,
                "label": doc.get("label", bid),
                "level": doc.get("level", "L2"),
                "parts": dict(doc.get("parts") or {}),
            })
        return out

    def list_parts(self, type_filter: Optional[str] = None) -> List[dict]:
        self.load_manifest()
        out: List[dict] = []
        for pid, path in sorted(self._part_index.items()):
            doc = self._read_yaml(path)
            self.validate_part_envelope(doc, expected_id=pid)
            if type_filter and doc.get("type") != type_filter:
                continue
            out.append({
                "id": pid,
                "type": doc["type"],
                "schema": doc["schema"],
                "label": doc.get("label", pid),
                "tags": doc.get("tags") or [],
            })
        return out

    def load_part(self, part_id: str) -> dict:
        self.load_manifest()
        path = self._part_index.get(part_id)
        if path is None:
            raise CatalogError(f"unknown part id: {part_id}")
        doc = self._read_yaml(path)
        self.validate_part_envelope(doc, expected_id=part_id)
        return doc

    @staticmethod
    def validate_part_envelope(doc: Mapping[str, Any], *, expected_id: Optional[str] = None) -> None:
        if not isinstance(doc, Mapping):
            raise PartEnvelopeError("part file root must be a mapping")
        missing = [k for k in ENVELOPE_KEYS if k not in doc]
        if missing:
            raise PartEnvelopeError(f"part envelope missing keys: {', '.join(missing)}")
        if expected_id is not None and doc["id"] != expected_id:
            raise PartEnvelopeError(f"part id mismatch: file {doc['id']!r} != manifest {expected_id!r}")
        if doc["type"] not in PART_TYPES:
            raise PartEnvelopeError(f"unknown part type: {doc['type']}")
        if doc["schema"] not in SCHEMAS:
            raise PartEnvelopeError(f"unknown schema: {doc['schema']}")
        if not isinstance(doc["body"], Mapping):
            raise PartEnvelopeError("part body must be a mapping")

    def load_blueprint(self, blueprint_id: str) -> dict:
        self.load_manifest()
        path = self._blueprint_index.get(blueprint_id)
        if path is None:
            raise CatalogError(f"unknown blueprint id: {blueprint_id}")
        doc = self._read_yaml(path)
        if doc.get("id") != blueprint_id:
            raise CatalogError(f"blueprint id mismatch: {doc.get('id')!r} != {blueprint_id!r}")
        if "parts" not in doc or not isinstance(doc["parts"], Mapping):
            raise CatalogError(f"blueprint {blueprint_id} missing parts mapping")
        return doc

    def resolve_blueprint(
        self,
        blueprint_id: str,
        *,
        instance_parts: Optional[Mapping[str, str]] = None,
        overrides: Optional[Mapping[str, Any]] = None,
        out_dir: Optional[Path] = None,
    ) -> ResolvedVehicle:
        bp = self.load_blueprint(blueprint_id)
        level = str(bp.get("level", "L2"))
        merged_parts = dict(bp["parts"])
        if instance_parts:
            merged_parts.update(instance_parts)
        self._validate_blueprint_slots(level, merged_parts)

        vehicle_body: Dict[str, Any] = {}
        tire_fl_body: Optional[Dict[str, Any]] = None
        tire_fr_body: Optional[Dict[str, Any]] = None
        tire_rl_body: Optional[Dict[str, Any]] = None
        tire_rr_body: Optional[Dict[str, Any]] = None
        tire_rear_body: Optional[Dict[str, Any]] = None
        part_ids: Dict[str, str] = {}
        susp_front: Optional[Path] = None
        susp_rear: Optional[Path] = None

        for slot, part_id in merged_parts.items():
            part = self.load_part(str(part_id))
            part_ids[slot] = str(part_id)
            body = dict(part["body"])
            if slot in ("body", "aero", "ride"):
                _merge_part_body(vehicle_body, body)
            elif slot == "tire":
                tire_fl_body = body
            elif slot == "tire_fr":
                tire_fr_body = body
            elif slot == "tire_rl":
                tire_rl_body = body
            elif slot == "tire_rr":
                tire_rr_body = body
            elif slot == "tire_rear":
                tire_rear_body = body
            elif slot in ("brake", "steering", "drivetrain", "powertrain"):
                _merge_part_body(vehicle_body, body)
            elif slot == "front_chassis":
                susp_front = self._kinematics_path(body, part["schema"], slot)
            elif slot == "rear_chassis":
                susp_rear = self._kinematics_path(body, part["schema"], slot)
            elif slot in ("front_susp_ride", "rear_susp_ride"):
                _merge_part_body(vehicle_body, body)

        if tire_fl_body is None:
            raise CatalogError(f"blueprint {blueprint_id} has no tire part")

        for slot, dotted in (bp.get("overrides") or {}).items():
            _apply_dotted(vehicle_body, str(slot), dotted)
        if overrides:
            for slot, dotted in overrides.items():
                _apply_dotted(vehicle_body, str(slot), dotted)

        # User C++ subsystem-module plugins (module_plugin_v1 parts). Emitted as a
        # `module_plugins:` list the runtime loads + installs after initialize(). This is
        # the uniform path for all five kinds (incl. suspension/ARB, which have no slot).
        module_plugins = []
        for mp_id in (bp.get("module_plugins") or []):
            mpart = self.load_part(str(mp_id))
            if mpart.get("schema") != "module_plugin_v1":
                raise CatalogError(
                    f"module_plugins entry {mp_id} is not a module_plugin_v1 part")
            module_plugins.append(dict(mpart["body"]))
        if module_plugins:
            vehicle_body["module_plugins"] = module_plugins

        root = Path(out_dir) if out_dir else Path(tempfile.mkdtemp(prefix="vdsim_resolve_"))
        root.mkdir(parents=True, exist_ok=True)
        vehicle_yaml = root / "vehicle.yaml"
        tire_yaml = root / "tire.yaml"
        tire_rear_yaml: Optional[Path] = None
        tire_fr_yaml: Optional[Path] = None
        tire_rl_yaml: Optional[Path] = None
        tire_rr_yaml: Optional[Path] = None
        with open(vehicle_yaml, "w", encoding="utf-8") as f:
            yaml.safe_dump(vehicle_body, f, sort_keys=False)
        with open(tire_yaml, "w", encoding="utf-8") as f:
            yaml.safe_dump(tire_fl_body, f, sort_keys=False)
        if tire_rear_body is not None and tire_rl_body is None and tire_rr_body is None:
            tire_rear_yaml = root / "tire_rear.yaml"
            with open(tire_rear_yaml, "w", encoding="utf-8") as f:
                yaml.safe_dump(tire_rear_body, f, sort_keys=False)
        if tire_fr_body is not None:
            tire_fr_yaml = root / "tire_fr.yaml"
            with open(tire_fr_yaml, "w", encoding="utf-8") as f:
                yaml.safe_dump(tire_fr_body, f, sort_keys=False)
        if tire_rl_body is not None:
            tire_rl_yaml = root / "tire_rl.yaml"
            with open(tire_rl_yaml, "w", encoding="utf-8") as f:
                yaml.safe_dump(tire_rl_body, f, sort_keys=False)
        if tire_rr_body is not None:
            tire_rr_yaml = root / "tire_rr.yaml"
            with open(tire_rr_yaml, "w", encoding="utf-8") as f:
                yaml.safe_dump(tire_rr_body, f, sort_keys=False)

        return ResolvedVehicle(
            blueprint_id=blueprint_id,
            level=level,
            part_ids=part_ids,
            vehicle_yaml=vehicle_yaml,
            tire_yaml=tire_yaml,
            tire_rear_yaml=tire_rear_yaml,
            tire_fr_yaml=tire_fr_yaml,
            tire_rl_yaml=tire_rl_yaml,
            tire_rr_yaml=tire_rr_yaml,
            susp_front=susp_front,
            susp_rear=susp_rear,
        )

    def _validate_blueprint_slots(self, level: str, parts: Mapping[str, str]) -> None:
        for slot in L1L2_PART_SLOTS:
            if slot not in parts:
                raise CatalogError(f"blueprint missing required part slot: {slot}")
        if level in ("L3", "L4"):
            for slot in L3_EXTRA_SLOTS:
                if slot not in parts:
                    raise CatalogError(f"L3/L4 blueprint missing required slot: {slot}")

    def _kinematics_path(self, body: Mapping[str, Any], schema: str, slot: str) -> Path:
        if schema == "topology_preview_v1":
            raise CatalogError(f"{slot} cannot use topology_preview_v1 part")
        if schema != "kinematics_l3_native_v1":
            raise CatalogError(f"susp kinematics slot requires kinematics_l3_native_v1, got {schema}")
        path = body.get("path")
        if not path:
            raise CatalogError("susp kinematics body needs path")
        p = Path(str(path))
        if not p.is_absolute():
            p = (self.catalog_root / p).resolve()
        return p

    @staticmethod
    def _read_yaml(path: Path) -> dict:
        if not path.is_file():
            raise CatalogError(f"config not found: {path}")
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        if not isinstance(doc, dict):
            raise CatalogError(f"yaml root must be a mapping: {path}")
        return doc


def _merge_part_body(vehicle_body: Dict[str, Any], body: Mapping[str, Any]) -> None:
    """Merge a part body into the vehicle config. The `powertrain:` block is shallow-merged
    by its top keys (engine / gearbox / shift) so a `powertrain` part (engine) and a
    `drivetrain` part (gearbox + shift + diff) can each contribute their piece; everything
    else is a flat overwrite (last writer wins)."""
    for k, v in body.items():
        if k == "powertrain" and isinstance(v, Mapping):
            cur = vehicle_body.get("powertrain")
            vehicle_body["powertrain"] = {**cur, **v} if isinstance(cur, dict) else dict(v)
        else:
            vehicle_body[k] = v


def _apply_dotted(root: Dict[str, Any], key: str, value: Any) -> None:
    if "." not in key:
        root[key] = value
        return
    head, tail = key.split(".", 1)
    node = root.setdefault(head, {})
    if not isinstance(node, dict):
        raise CatalogError(f"override path conflict at {head}")
    _apply_dotted(node, tail, value)
