"""vdsim_trace — ``.vdtrace`` container I/O (writer, reader, overlay attach).

A trace is the time-series record of **one** simulation run, stored as a single
zip container so that a renderer can reproduce the run's animation from that
file alone — no ``results.json`` re-parse, no re-running the simulation.

Layout (zip, ``ZIP_STORED`` so channels stay ``mmap``/``frombuffer`` friendly)::

    run.vdtrace
    |-- manifest.json          # schema / metadata / channel table
    |-- channels/<name>.f64    # C-order little-endian float64 raw
    +-- overlays/<name>.json   # externally injected, pass-through only

Design contract (``11_trace_contract_spec.md``, source of truth):

* Recording and rendering are separated — the renderer's only input is the file.
* VDSim records **plant** concepts only. Reference paths, lateral error and
  pass/fail verdicts are not plant concepts; they arrive as overlays (§4) and
  VDSim never interprets them.
* A trace declares its ``role`` (``plant`` or ``predictor``) so the same code
  used as both the simulator under test and an optimiser's internal model
  cannot be presented as one undeclared thing.
* The schema is independent of the tyre model. Model-dependent information
  leaks out through exactly two manifest constants, ``tire.mu_aniso``.
* Friction utilization is a *derived* value computed by the consumer
  (:func:`utilization`), never a recorded channel.

This module depends on the standard library and numpy only. It deliberately
does **not** import ``vdsim`` (the compiled core), so a trace can be read and
rendered on a machine that cannot run the simulator.
"""
from __future__ import annotations

import hashlib
import io
import json
import os
import warnings
import zipfile
from pathlib import Path

import numpy as np

#: Container schema version written by this module. ``0.x`` is unstable —
#: see the release notes. ``0.2`` added the required ``role`` field (§3.1).
SCHEMA_VERSION = "0.2"

#: ``0.x`` minors this reader accepts. Each ``0.x`` minor is its own line, so
#: readability is opt-in rather than inferred: ``0.1`` stays readable only
#: because :func:`_resolve_role` defines a fallback for its missing ``role``.
READABLE_SCHEMA_VERSIONS = ("0.1", "0.2")

#: Declared role of the run a trace records (§3.1, decided 2026-09-02).
#: ``plant`` is the simulator under verification; ``predictor`` is the same
#: code used as the internal model of an optimiser/MPC. The pair
#: ``role`` + ``param_hash`` is what makes an *undeclared* dual role — a run
#: whose plant and predictor are the same parameters presented as a
#: verification result — mechanically detectable.
ROLES = ("plant", "predictor")

#: Role assumed for a ``0.1`` trace, which predates the field.
LEGACY_DEFAULT_ROLE = "plant"

#: Wheel order used by every per-wheel channel.
WHEELS = ("FL", "FR", "RL", "RR")

_MANIFEST_NAME = "manifest.json"
_CHANNEL_DIR = "channels"
_OVERLAY_DIR = "overlays"
_DTYPE = np.dtype("<f8")

#: Channel table of the v0.1 contract (§3.2): name -> (unit, trailing shape).
#: A channel's stored shape is ``(n_steps,) + trailing``. Adding channels is
#: backwards compatible; consumers must ignore channels they do not know.
CHANNEL_SPECS = {
    "t":           ("s",       ()),
    "pose":        ("m,m,rad", (3,)),
    "v_body":      ("m/s",     (2,)),
    "yaw_rate":    ("rad/s",   ()),
    "u_steer":     ("rad",     ()),
    "u_fx":        ("N",       ()),
    "wheel_F":     ("N",       (4, 3)),
    "wheel_mu":    ("-",       (4,)),
    "wheel_kappa": ("-",       (4,)),
    "wheel_alpha": ("rad",     (4,)),
}

#: Overlay kinds the container validates. Unknown kinds are stored verbatim
#: (a renderer must ignore them); a missing ``kind`` is rejected.
OVERLAY_KINDS = ("path2d", "timeseries", "event", "region")

_REQUIRED_MANIFEST_KEYS = (
    "schema_version", "producer", "repro", "n_steps",
    "wheels", "geometry", "tire", "channels",
)
_REQUIRED_REPRO_KEYS = ("vdsim_version", "git_sha", "param_hash", "seed", "dt_s", "run_id")
_REQUIRED_GEOMETRY_KEYS = ("wheelbase_m", "track_m", "steer_ratio")


class TraceError(ValueError):
    """Base class for every ``.vdtrace`` contract violation."""


class TraceSchemaError(TraceError):
    """Manifest is missing required fields or declares an incompatible schema."""


class TraceOverlayError(TraceError):
    """Overlay object violates the §4 pass-through contract."""


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _canonical_json(obj) -> str:
    """Serialise ``obj`` deterministically (stable key order, no whitespace drift)."""
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def param_hash(plant_params: dict) -> str:
    """Hash the **plant** parameter set that defines the input/output relation.

    Scope is fixed by the contract (§10.2): vehicle geometry/mass, tyre
    parameters, road friction and the integration step are included; scenario
    definitions, reference paths, controller gains and overlays are excluded.
    Two runs of the same vehicle over different scenarios therefore share a
    hash, which is what makes them comparable.

    :param plant_params: JSON-serialisable plant parameters.
    :returns: ``"sha256:<hex>"``.
    """
    digest = hashlib.sha256(_canonical_json(plant_params).encode("utf-8")).hexdigest()
    return "sha256:" + digest


def _schema_compatible(found: str) -> bool:
    """Return True if a trace written at ``found`` can be read by this module.

    ``0.x`` is an unstable line, so compatibility there requires an exact
    match. From ``1.0`` on, a matching major version is enough.
    """
    try:
        f_major, f_minor = (int(p) for p in str(found).split(".")[:2])
        c_major, c_minor = (int(p) for p in SCHEMA_VERSION.split(".")[:2])
    except (ValueError, TypeError):
        return False
    if f_major != c_major:
        return False
    if c_major == 0:
        return "%d.%d" % (f_major, f_minor) in READABLE_SCHEMA_VERSIONS
    return f_minor <= c_minor


def _resolve_role(manifest) -> str:
    """Return the run's declared role, applying the ``0.1`` fallback.

    ``role`` became required in schema ``0.2``. A ``0.1`` trace predates the
    field, so a missing ``role`` there resolves to :data:`LEGACY_DEFAULT_ROLE`
    with one warning; the same omission at ``0.2`` is an error, because a
    silent default would reintroduce exactly the undeclared dual role the
    field exists to expose.

    :param manifest: parsed ``manifest.json``.
    :returns: one of :data:`ROLES`.
    :raises TraceSchemaError: on a missing ``role`` at ``0.2``+ or an
        unknown role value at any version.
    """
    role = manifest.get("role")
    if role is None:
        if str(manifest.get("schema_version")) == "0.1":
            warnings.warn(
                "trace schema_version 0.1 declares no 'role'; assuming %r. "
                "Re-record with vdsim_trace %s so the run states whether it "
                "was the plant or a predictor."
                % (LEGACY_DEFAULT_ROLE, SCHEMA_VERSION),
                UserWarning, stacklevel=3)
            return LEGACY_DEFAULT_ROLE
        raise TraceSchemaError(
            "manifest is missing 'role' — schema %s requires one of %s so a "
            "plant run and a predictor run of the same code cannot be "
            "presented as the same evidence"
            % (manifest.get("schema_version"), list(ROLES)))
    if role not in ROLES:
        raise TraceSchemaError(
            "manifest.role must be one of %s, got %r" % (list(ROLES), role))
    return role


def validate_overlay(obj) -> str:
    """Validate an overlay's envelope only; never its content.

    The container checks that ``kind`` and ``name`` exist and that ``kind`` is a
    string. It does not interpret coordinates, units or semantics — the moment
    it did, "a channel called ref_path is drawn dashed" would become a rule and
    the scenario coupling this contract removes would be back.

    :param obj: overlay dict.
    :returns: the overlay name.
    :raises TraceOverlayError: when the envelope is invalid.
    """
    if not isinstance(obj, dict):
        raise TraceOverlayError("overlay must be a JSON object")
    kind = obj.get("kind")
    if not kind or not isinstance(kind, str):
        raise TraceOverlayError(
            "overlay requires a non-empty string 'kind' tag "
            "(one of %s, or a future kind renderers will ignore)" % (", ".join(OVERLAY_KINDS),))
    name = obj.get("name")
    if not name or not isinstance(name, str):
        raise TraceOverlayError("overlay requires a non-empty string 'name'")
    if any(c in name for c in "/\\"):
        raise TraceOverlayError("overlay name must not contain path separators: %r" % (name,))
    return name


def utilization(wheel_F, wheel_mu, mu_aniso, eps: float = 1.0) -> np.ndarray:
    """Friction utilization per wheel — the derived quantity of §3.3.

    ``util = sqrt((Fx / (k_lon*mu*Fz))^2 + (Fy / (k_lat*mu*Fz))^2)``; ``1.0``
    means the tyre is on its friction limit. The friction circle is the special
    case ``mu_aniso == [1, 1]``, so no circle/ellipse branch is needed.

    This is not a channel on purpose: the definition depends on how the tyre
    model handles combined slip, and freezing it into the schema would break
    the schema whenever the tyre model changes.

    :param wheel_F: ``[n,4,3]`` contact forces (Fx, Fy, Fz) in N.
    :param wheel_mu: ``[n,4]`` road friction coefficient used that step.
    :param mu_aniso: ``(k_lon, k_lat)`` mu multipliers from manifest ``tire``.
    :param eps: floor applied to Fz [N] so an airborne wheel cannot divide by zero.
    :returns: ``[n,4]`` utilization.
    """
    F = np.asarray(wheel_F, dtype=float)
    mu = np.asarray(wheel_mu, dtype=float)
    k_lon, k_lat = (float(mu_aniso[0]), float(mu_aniso[1]))
    Fz = np.maximum(F[..., 2], eps)
    denom = mu * Fz
    denom = np.where(denom > eps, denom, eps)
    nx = F[..., 0] / (k_lon * denom)
    ny = F[..., 1] / (k_lat * denom)
    return np.sqrt(nx * nx + ny * ny)


# --------------------------------------------------------------------------
# writer
# --------------------------------------------------------------------------

class TraceWriter:
    """Accumulates channel samples and writes one ``.vdtrace`` on :meth:`finalize`.

    The writer owns the *simulation* write path only. Overlays are attached
    afterwards through :func:`attach_overlay`; keeping the two paths apart is
    the practical guarantee that VDSim never computes overlay content.

    :param path: output ``.vdtrace`` path.
    :param geometry: manifest ``geometry`` block (``wheelbase_m``, ``track_m``,
        ``steer_ratio`` required; body dimensions optional but recommended).
    :param tire: manifest ``tire`` block (``friction_shape``, ``mu_aniso``).
    :param repro: reproduction block; ``vdsim_version``, ``git_sha``,
        ``param_hash``, ``seed``, ``dt_s``, ``run_id``.
    :param producer: ``{"name": ..., "version": ...}`` of the writing script.
    :param decimation: keep 1 of every ``N`` :meth:`append` calls (``N>=1``).
    :param channels: channel subset to record; defaults to the full v0.1 table.
    :param extra: additional manifest keys merged verbatim (never required).
    :param role: keyword-only and **required** — one of :data:`ROLES`. It has
        no default on purpose: the field exists to catch a producer that
        silently records a predictor run as if it were the plant, and a
        default would be that silence.
    """

    def __init__(self, path, geometry, tire, repro, producer=None,
                 decimation: int = 1, channels=None, extra=None, *, role):
        decimation = int(decimation)
        if decimation < 1:
            raise TraceError("decimation must be >= 1, got %r" % (decimation,))
        names = tuple(channels) if channels is not None else tuple(CHANNEL_SPECS)
        unknown = [n for n in names if n not in CHANNEL_SPECS]
        if unknown:
            raise TraceError("unknown channel(s): %s" % (", ".join(unknown),))
        _validate_geometry(geometry)
        _validate_tire(tire)
        if role not in ROLES:
            raise TraceError("role must be one of %s, got %r" % (list(ROLES), role))

        self.path = Path(path)
        self.role = role
        self.decimation = decimation
        self.geometry = dict(geometry)
        self.tire = dict(tire)
        self.repro = dict(repro or {})
        self.producer = dict(producer or {"name": "unknown", "version": ""})
        self.extra = dict(extra or {})
        self._names = names
        self._buf = {n: [] for n in names}
        self._calls = 0
        self._finalized = False

    # -- recording ---------------------------------------------------------
    @property
    def n_steps(self) -> int:
        """Number of samples actually stored so far (after decimation)."""
        first = self._names[0]
        return len(self._buf[first])

    @property
    def due(self) -> bool:
        """True when the next :meth:`append` would be stored rather than dropped.

        Lets a producer skip building the sample at all on decimated steps —
        the dict construction, not the storage, is what costs time in the
        recording path.
        """
        return (self._calls % self.decimation) == 0

    def skip(self) -> None:
        """Advance the decimation counter without storing a sample.

        Must be called exactly once per simulation step that was not appended,
        otherwise the kept steps stop being evenly spaced.
        """
        if self._finalized:
            raise TraceError("skip() after finalize()")
        self._calls += 1

    def append(self, sample: dict) -> bool:
        """Offer one simulation step to the trace.

        Every call advances the decimation counter; only every ``decimation``-th
        call is stored. Recording 1 kHz physics at ``decimation=10`` therefore
        yields exactly ``n_steps = calls / 10``.

        :param sample: mapping channel name -> value, covering every configured
            channel. Shapes follow :data:`CHANNEL_SPECS`.
        :returns: True when the sample was stored, False when decimated away.
        """
        if self._finalized:
            raise TraceError("append() after finalize()")
        keep = (self._calls % self.decimation) == 0
        self._calls += 1
        if not keep:
            return False
        for name in self._names:
            if name not in sample:
                raise TraceError("sample is missing channel %r" % (name,))
            self._buf[name].append(sample[name])
        return True

    # -- output ------------------------------------------------------------
    def finalize(self) -> Path:
        """Flush channels, freeze the manifest and write the container.

        :returns: the written path.
        """
        if self._finalized:
            raise TraceError("finalize() called twice")
        n = self.n_steps
        arrays = {}
        chan_meta = []
        for name in self._names:
            unit, trailing = CHANNEL_SPECS[name]
            arr = np.asarray(self._buf[name], dtype=_DTYPE).reshape((n,) + trailing)
            arrays[name] = np.ascontiguousarray(arr)
            chan_meta.append({
                "name": name, "unit": unit, "dtype": "<f8",
                "shape": [n] + list(trailing),
            })

        manifest = dict(self.extra)
        manifest.update({
            "schema_version": SCHEMA_VERSION,
            "producer": self.producer,
            "role": self.role,
            "repro": self.repro,
            "n_steps": int(n),
            "wheels": list(WHEELS),
            "geometry": self.geometry,
            "tire": self.tire,
            "channels": chan_meta,
        })
        _validate_manifest(manifest)

        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + ".part")
        with zipfile.ZipFile(tmp, "w", compression=zipfile.ZIP_STORED) as zf:
            zf.writestr(_MANIFEST_NAME, json.dumps(manifest, indent=2, sort_keys=True))
            for name, arr in arrays.items():
                zf.writestr("%s/%s.f64" % (_CHANNEL_DIR, name), arr.tobytes(order="C"))
        os.replace(str(tmp), str(self.path))
        self._finalized = True
        return self.path


def _validate_geometry(geometry):
    if not isinstance(geometry, dict):
        raise TraceSchemaError("manifest.geometry must be an object")
    missing = [k for k in _REQUIRED_GEOMETRY_KEYS if k not in geometry]
    if missing:
        raise TraceSchemaError(
            "manifest.geometry is missing %s — the renderer draws the body "
            "rectangle, front-wheel position and steering angle from this block "
            "and must never guess vehicle parameters" % (", ".join(missing),))


def _validate_tire(tire):
    """Reject a trace whose tyre block is absent or malformed.

    Silently substituting ``[1, 1]`` would under-report utilization on an
    elliptic tyre model (0.84 shown as 0.78), so a wheel that should cross the
    0.8 warning threshold would render in a safe colour. That failure is
    invisible, so it is an error instead.
    """
    if not isinstance(tire, dict):
        raise TraceSchemaError("manifest.tire is required and must be an object")
    shape = tire.get("friction_shape")
    if shape not in ("circle", "ellipse"):
        raise TraceSchemaError(
            "manifest.tire.friction_shape must be 'circle' or 'ellipse', got %r" % (shape,))
    aniso = tire.get("mu_aniso")
    if (not isinstance(aniso, (list, tuple)) or len(aniso) != 2
            or not all(isinstance(v, (int, float)) and float(v) > 0.0 for v in aniso)):
        raise TraceSchemaError(
            "manifest.tire.mu_aniso must be [k_lon, k_lat] with both > 0, got %r" % (aniso,))


def _validate_manifest(manifest):
    if not isinstance(manifest, dict):
        raise TraceSchemaError("manifest.json must be an object")
    missing = [k for k in _REQUIRED_MANIFEST_KEYS if k not in manifest]
    if missing:
        raise TraceSchemaError("manifest is missing required key(s): %s" % (", ".join(missing),))
    missing = [k for k in _REQUIRED_REPRO_KEYS if k not in manifest["repro"]]
    if missing:
        raise TraceSchemaError(
            "manifest.repro is missing %s — without it 'deterministic replay' "
            "is an unverifiable claim" % (", ".join(missing),))
    _validate_geometry(manifest["geometry"])
    _validate_tire(manifest["tire"])
    if not isinstance(manifest["channels"], list) or not manifest["channels"]:
        raise TraceSchemaError("manifest.channels must be a non-empty list")


# --------------------------------------------------------------------------
# reader
# --------------------------------------------------------------------------

class TraceReader:
    """Read-only view over a ``.vdtrace``; channels load lazily.

    Usage::

        with TraceReader("run.vdtrace") as tr:
            pose = tr.channel("pose")            # [n,3]
            util = tr.utilization()              # [n,4]
            ref  = tr.overlay("ref_path")        # or None

    :param path: ``.vdtrace`` path.
    :raises TraceSchemaError: on a version mismatch or a missing required block.
        The schema is checked *before* any channel is parsed.
    """

    def __init__(self, path):
        self.path = Path(path)
        if not self.path.is_file():
            raise TraceError("trace not found: %s" % (self.path,))
        self._zf = zipfile.ZipFile(str(self.path), "r")
        try:
            raw = self._zf.read(_MANIFEST_NAME)
        except KeyError:
            self._zf.close()
            raise TraceSchemaError("%s is not a .vdtrace: no manifest.json" % (self.path,))
        try:
            manifest = json.loads(raw.decode("utf-8"))
        except ValueError as exc:
            self._zf.close()
            raise TraceSchemaError("manifest.json is not valid JSON: %s" % (exc,))

        found = manifest.get("schema_version")
        if not _schema_compatible(found):
            self._zf.close()
            raise TraceSchemaError(
                "trace schema_version %r is not readable by vdsim_trace %s "
                "(0.x traces carry no compatibility guarantee); re-record the run "
                "or use a matching VDSim release" % (found, SCHEMA_VERSION))
        try:
            _validate_manifest(manifest)
            role = _resolve_role(manifest)
        except TraceError:
            self._zf.close()
            raise
        self.manifest = manifest
        self._role = role
        self._cache = {}

    # -- lifecycle ---------------------------------------------------------
    def close(self):
        """Close the underlying zip handle."""
        self._zf.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -- metadata ----------------------------------------------------------
    @property
    def n_steps(self) -> int:
        """Number of recorded samples."""
        return int(self.manifest["n_steps"])

    @property
    def geometry(self) -> dict:
        """Manifest ``geometry`` block."""
        return self.manifest["geometry"]

    @property
    def tire(self) -> dict:
        """Manifest ``tire`` block."""
        return self.manifest["tire"]

    @property
    def repro(self) -> dict:
        """Manifest ``repro`` block."""
        return self.manifest["repro"]

    @property
    def role(self) -> str:
        """Declared run role — ``plant`` or ``predictor`` (§3.1).

        Resolved once at open time, so a ``0.1`` trace reports the legacy
        default here and warns exactly once rather than on every access.
        """
        return self._role

    @property
    def wheels(self):
        """Wheel labels in per-wheel channel order."""
        return tuple(self.manifest.get("wheels", WHEELS))

    def channel_names(self):
        """Names of the channels this trace actually carries, in manifest order."""
        return tuple(c["name"] for c in self.manifest["channels"])

    def channel_meta(self, name: str) -> dict:
        """Manifest entry for ``name``.

        :raises KeyError: when the trace does not carry that channel.
        """
        for c in self.manifest["channels"]:
            if c["name"] == name:
                return c
        raise KeyError(name)

    def has(self, name: str) -> bool:
        """True when the trace carries channel ``name``."""
        return any(c["name"] == name for c in self.manifest["channels"])

    # -- data --------------------------------------------------------------
    def channel(self, name: str) -> np.ndarray:
        """Load one channel as a read-only ``float64`` array of its manifest shape.

        Loading goes through ``np.frombuffer`` so the decode cost is a reshape,
        not a parse.
        """
        if name in self._cache:
            return self._cache[name]
        meta = self.channel_meta(name)
        raw = self._zf.read("%s/%s.f64" % (_CHANNEL_DIR, name))
        shape = tuple(int(s) for s in meta["shape"])
        expect = int(np.prod(shape)) if shape else 0
        arr = np.frombuffer(raw, dtype=_DTYPE)
        if arr.size != expect:
            raise TraceError(
                "channel %r has %d values but manifest declares shape %s"
                % (name, arr.size, shape))
        arr = arr.reshape(shape)
        arr.flags.writeable = False
        self._cache[name] = arr
        return arr

    def utilization(self) -> np.ndarray:
        """Per-wheel friction utilization ``[n,4]`` — see :func:`utilization`."""
        return utilization(self.channel("wheel_F"), self.channel("wheel_mu"),
                           self.tire["mu_aniso"])

    # -- overlays ----------------------------------------------------------
    def overlay_names(self):
        """Names of attached overlays."""
        pre, suf = _OVERLAY_DIR + "/", ".json"
        return tuple(sorted(
            n[len(pre):-len(suf)] for n in self._zf.namelist()
            if n.startswith(pre) and n.endswith(suf)))

    def overlay(self, name: str):
        """Return one overlay object, or ``None`` when it is not attached."""
        try:
            raw = self._zf.read("%s/%s.json" % (_OVERLAY_DIR, name))
        except KeyError:
            return None
        return json.loads(raw.decode("utf-8"))

    def overlays(self, kind=None):
        """Return every overlay, optionally filtered by ``kind``.

        :param kind: overlay kind to keep, or ``None`` for all.
        :returns: list of overlay dicts.
        """
        out = []
        for name in self.overlay_names():
            obj = self.overlay(name)
            if kind is None or (isinstance(obj, dict) and obj.get("kind") == kind):
                out.append(obj)
        return out


# --------------------------------------------------------------------------
# overlay attach (post-simulation write path)
# --------------------------------------------------------------------------

def attach_overlay(path, obj, replace: bool = True) -> str:
    """Attach one overlay to an existing ``.vdtrace``, after the run.

    VDSim stores the object and never interprets it. The envelope is checked
    (:func:`validate_overlay`); unknown ``kind`` values are stored so that a
    newer producer can write them and today's renderer can ignore them.

    :param path: existing ``.vdtrace``.
    :param obj: overlay dict carrying at least ``kind`` and ``name``.
    :param replace: overwrite an overlay of the same name when present.
    :returns: the overlay name.
    :raises TraceOverlayError: on an invalid envelope, or a name clash with
        ``replace=False``.
    """
    name = validate_overlay(obj)
    p = Path(path)
    member = "%s/%s.json" % (_OVERLAY_DIR, name)
    payload = json.dumps(obj, sort_keys=True).encode("utf-8")

    with zipfile.ZipFile(str(p), "r") as zf:
        existing = zf.namelist()
        if member in existing and not replace:
            raise TraceOverlayError("overlay %r already attached" % (name,))
        keep = [(info, zf.read(info.filename)) for info in zf.infolist()
                if info.filename != member]

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", compression=zipfile.ZIP_STORED) as out:
        for info, data in keep:
            out.writestr(info.filename, data)
        out.writestr(member, payload)
    tmp = p.with_suffix(p.suffix + ".part")
    tmp.write_bytes(buf.getvalue())
    os.replace(str(tmp), str(p))
    return name
