"""VDSimPlant — Ld2 7DOF closed-loop plant API for external MPC (VLA thesis).

Contract (stable):
  Control u = [delta_roadwheel_rad, Fx_total_N]  (ISO 8855, force intent at CG).
  Observation dict each step (true state, contact-frame tyre forces, FL0..RR3):

    X, Y, psi, vx, vy, r, ax, ay, beta, roll, pitch   — vehicle [m, rad, m/s, m/s²]
    wheel: list[4] of {Fx, Fy, Fz, alpha, kappa, mu, mu_peak, alpha_peak, kappa_peak}
      Fx, Fy — tyre contact / wheel frame [N]  (+Fx drive, +Fy left)
      Fz [N], alpha [rad], kappa [-], mu [-] contact friction used this step
      mu_peak [-] realized load-dependent peak coefficient (force/Fz)
      alpha_peak [rad], kappa_peak [-] MF pure-slip peak slip at this Fz
    delta_rw [rad] — realized road-wheel steer (after the actuator chain)
    delta_sw [rad] — delta_rw × steering_ratio; only when the vehicle file
      states steering_ratio (no default stands in for it)

Built on vdsim_lab config resolution + SimSession (direct CmdL1 torque path).

Trace recording (opt-in, default OFF) writes one ``.vdtrace`` per run::

    plant.enable_trace("runs/r001/run.vdtrace")
    ...                                   # step() as usual
    plant.finalize_trace()

See :mod:`vdsim_trace` for the container contract.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
from vdsim_guard import load_core  # noqa: E402  (shadowed-core check, one place)

vdsim = load_core([REPO / "build" / "python"])

def _conf_root():
    for c in (REPO / "configs", Path.cwd() / "configs",
              Path(__file__).resolve().parent / "vdsim_configs"):
        if c.is_dir():
            return c
    return REPO / "configs"


#: Environment variable naming a preset root outside the repository, for cars
#: whose parameters must not be published.  Same layout as ``configs/``:
#: ``<root>/vehicles/<stem>.yaml``, ``<root>/parts/tire/<stem>.yaml``.
PRIVATE_ROOT_ENV = "VDSIM_PRIVATE_CONFIGS"


def _private_root() -> Path | None:
    """``$VDSIM_PRIVATE_CONFIGS`` as a directory, or None when it is unset.

    :raises FileNotFoundError: the variable is set but names no directory.
    """
    raw = os.environ.get(PRIVATE_ROOT_ENV, "").strip()
    if not raw:
        return None
    root = Path(raw).expanduser()
    if not root.is_dir():
        raise FileNotFoundError(f"{PRIVATE_ROOT_ENV}={raw!r} is not a directory")
    return root


def preset_roots() -> list:
    """Preset roots in search order: ``$VDSIM_PRIVATE_CONFIGS`` (when set), then
    :func:`_conf_root`.  The one search rule shared by this plant and
    ``vdsim_rl``, so a private car loads the same way in both.
    """
    return [r for r in (_private_root(), _conf_root()) if r is not None]


def find_preset(rel: str, what: str = "preset") -> Path:
    """First ``<root>/<rel>`` over :func:`preset_roots` that is a file.

    :param rel: path inside a preset root (``vehicles/x.yaml``, ``parts/tire/x.yaml``).
    :param what: word for the error message (``vehicle``, ``tire``).
    :raises FileNotFoundError: no root holds ``rel``.
    """
    roots = preset_roots()
    for root in roots:
        if (root / rel).is_file():
            return root / rel
    raise FileNotFoundError(f"{what} preset '{rel}' not found under "
                            + ", ".join(str(r) for r in roots))


def resolve_vehicle_config(name: str = "generic_sedan") -> Path:
    """Resolve a vehicle preset YAML (e.g. ``generic_sedan``).

    Searches ``vehicles/<stem>.yaml`` (then ``<stem>.yaml``) in each of
    :func:`preset_roots`, so ``$VDSIM_PRIVATE_CONFIGS`` is looked at first.
    """
    stem = Path(name).stem
    roots = preset_roots()
    for root in roots:
        for p in (root / "vehicles" / f"{stem}.yaml", root / f"{stem}.yaml"):
            if p.is_file():
                return p
    raise FileNotFoundError(
        f"vehicle config '{name}' not found under vehicles/ of "
        + ", ".join(str(r) for r in roots))


def _load_yaml_sidecar(path: Path, key: str) -> str | None:
    try:
        import yaml
        data = yaml.safe_load(path.read_text())
        if isinstance(data, dict) and key in data:
            return str(data[key])
    except Exception:
        pass
    return None


def _resolve_tir_path(tp_path: Path, tir_rel: str) -> str:
    if not tir_rel:
        return tir_rel
    p = Path(tir_rel)
    if p.is_absolute():
        return str(p)
    for base in (tp_path.parent, _conf_root()):
        cand = (base / tir_rel).resolve()
        if cand.is_file():
            return str(cand)
    return str((_conf_root() / tir_rel).resolve())


def _load_tire_setup_for_vehicle(vp_path: Path) -> vdsim.TireSetup:
    """Tyre setup named by the vehicle file's ``tire_yaml``.

    ``tire_yaml`` is looked up beside the vehicle file's own root first (a
    private car keeps its private tyre), then over :func:`preset_roots`; a
    vehicle file without ``tire_yaml`` gets ``parts/tire/generic_pacejka.yaml``.
    """
    rel = _load_yaml_sidecar(vp_path, "tire_yaml") or "parts/tire/generic_pacejka.yaml"
    sibling = vp_path.parent.parent / rel
    tp_path = sibling if sibling.is_file() else find_preset(rel, "tire")
    tp = vdsim.TireParams.from_yaml(str(tp_path))
    if tp.tir_path:
        tp.tir_path = _resolve_tir_path(tp_path, tp.tir_path)
    return vdsim.TireSetup(tp)


#: Record rate the default `decimation` targets [Hz]. 100 Hz is >3x a 30 fps
#: render and keeps 1 kHz control loops at a 10x storage saving.
TRACE_TARGET_HZ = 100.0

#: Opt-in trace channels :func:`trace_sample` fills from the session output
#: (a subset of ``vdsim_trace.OPTIONAL_CHANNELS``).
SAMPLED_OPTIONAL_CHANNELS = ("steer_applied",)

#: Ladder levels this plant can be built at. The control contract
#: ``u = [delta_rad, Fx_total_N]`` is the same at every one of them; only the
#: model behind the seam changes, and with it which trace channels exist.
#: ``"L4"`` is listed so the name is recognised, and then refused in
#: ``VDSimPlant.__init__``: without hardpoints it is L3 under another label.
#: ``"L5"`` is the free-3D model on its default penalty contact (no hardpoints).
PLANT_LEVELS = ("L1", "L2", "L3", "L4", "L5")

#: Largest substep the L5 free-3D model runs at when the caller does not pick a
#: finer one -- the value ``vdsim_realtime`` uses for an L5 vehicle
#: (``cosim/realtime_server.cpp``). The ladder default of 1 ms is too coarse for
#: the free body's penalty contact.
L5_MAX_SUBSTEP_DT = 2e-4


def level_solver_params(level, outer_dt, sp=None):
    """Solver settings a high-level entry point uses for ``level``.

    Levels other than ``"L5"`` get ``sp`` back untouched (a fresh
    ``SolverParams`` when ``sp`` is None), so their results do not change. For
    ``"L5"`` it applies the ``vdsim_realtime`` L5 settings -- ``stunt_physics``
    on and ``max_substep_dt`` at most :data:`L5_MAX_SUBSTEP_DT` -- and raises
    ``max_substeps`` to the count one outer step of ``outer_dt`` needs, with the
    same ``ceil(dt / max_substep_dt)`` the core uses, so the substep is never
    silently widened (``solver_substep_clamp_count`` stays 0).

    :param level: ladder id.
    :param outer_dt: [s] outer step the session is ticked with.
    :param sp: ``vdsim.SolverParams`` to adjust in place, or None.
    :returns: the adjusted ``vdsim.SolverParams``.
    """
    if sp is None:
        sp = vdsim.SolverParams()
    if level != "L5":
        return sp
    sp.stunt_physics = True
    sp.max_substep_dt = min(sp.max_substep_dt, L5_MAX_SUBSTEP_DT)
    sp.max_substeps = max(sp.max_substeps, int(math.ceil(outer_dt / sp.max_substep_dt)))
    return sp


def solver_record(sp):
    """The solver settings a run actually used, as a JSON-ready dict.

    :param sp: ``vdsim.SolverParams``.
    :returns: ``{"max_substep_dt", "max_substeps", "stunt_physics"}``.
    """
    return {"max_substep_dt": float(sp.max_substep_dt),
            "max_substeps": int(sp.max_substeps),
            "stunt_physics": bool(sp.stunt_physics)}

#: How far the road normal is coupled into the physics, per level
#: (``11_trace_contract_spec`` §3.1.1 ``contact_scope``).
#:
#: This is a **measurement, not an aspiration**. It is measured by stepping the
#: dynamics twice with contact arrays that differ only in the normal — once the
#: true banked normal, once ``(0,0,1)`` — and asking whether the trajectory
#: moves. On ``origin/main`` (D1/D2 merged) every level moves, so every level is
#: ``C2``. ``tests/scripts/test_trace_0_3.py`` re-runs that comparison and fails
#: if the declaration and the code ever drift apart, which is the only thing
#: that keeps a hand-written constant honest. L5 (free-3D) is measured the same
#: way with every contact at its static-equilibrium penetration (the free-3D
#: contact reads zero penetration as no contact) and is declared from that run.
CONTACT_SCOPE_BY_LEVEL = {
    "L1": "C2",
    "L2": "C2",
    "L3": "C2",
    "L4": "C2",
    "L5": "C2",
}


def body_dimensions(vp_path):
    """Read the body box and wheel width [m] from the vehicle YAML.

    Schema ``0.3`` needs ``body_lwh_m`` and ``wheel_width_m`` because a 3D
    renderer draws a body box and wheel cylinders (§3.1.1). They are vehicle
    facts, not render settings, so they live in the vehicle file — and they have
    **no default**: a guessed body length would put the wheels in the wrong place
    on screen and nothing would report it.

    :param vp_path: vehicle YAML path.
    :returns: ``(body_lwh_m, wheel_width_m)``.
    :raises KeyError: when the vehicle file declares neither.
    """
    import yaml
    data = yaml.safe_load(Path(vp_path).read_text()) or {}
    missing = [k for k in ("body_lwh_m", "wheel_width_m") if k not in data]
    if missing:
        raise KeyError(
            "vehicle config %s declares no %s; trace schema 0.3 requires them "
            "(11 §3.1.1). Add them to the vehicle YAML rather than defaulting "
            "them here — a guessed body box is a silently wrong picture."
            % (Path(vp_path).name, ", ".join(missing)))
    lwh = [float(v) for v in data["body_lwh_m"]]
    if len(lwh) != 3 or not all(v > 0.0 for v in lwh):
        raise ValueError("body_lwh_m must be [length, width, height] in m, all > 0")
    return lwh, float(data["wheel_width_m"])


def measure_mu_aniso(tp: vdsim.TireParams, Fz: float = None, n: int = 601):
    """Measure the friction-ellipse mu multipliers of a tyre parameter set.

    The renderer needs ``util = sqrt((Fx/(k_lon*mu*Fz))^2 + (Fy/(k_lat*mu*Fz))^2)``
    but must not know the tyre model. So the producer measures the two
    constants here, from pure-slip sweeps at unit road friction, and writes
    them into the manifest:

    * ``k_lon = max_kappa |Fx| / Fz`` at ``alpha = 0``
    * ``k_lat = max_alpha |Fy| / Fz`` at ``kappa = 0``

    Because ``mu_long = mu_lat = 1`` during the sweep, the result is a property
    of the tyre model alone and is independent of the road friction map.

    :param tp: tyre parameters (``tir_path`` is used when the backend is MF2002).
    :param Fz: sweep load [N]; defaults to the tyre's nominal load.
    :param n: samples per sweep.
    :returns: ``(friction_shape, [k_lon, k_lat])`` with ``friction_shape`` set
        to ``"circle"`` when the two multipliers agree to 1 %, else ``"ellipse"``.
    """
    Fz = float(Fz if Fz is not None else tp.Fz_nominal)
    if not (Fz > 0.0):
        raise ValueError("measure_mu_aniso needs Fz > 0")
    if getattr(tp, "tir_path", ""):
        model = vdsim.create_magic_formula_tire_from_tir(tp.tir_path)
    else:
        # The binding takes no argument; the parameter set is applied by
        # `initialize` below, exactly as for the TIR-backed model.
        model = vdsim.create_pacejka_mf96()
    model.initialize(tp)

    def _peak(kappa_max, alpha_max, pick):
        best = 0.0
        for i in range(n):
            frac = i / float(n - 1)
            inp = vdsim.TireInput()
            inp.Fz = Fz
            inp.kappa = kappa_max * frac
            inp.alpha = alpha_max * frac
            inp.mu_long = 1.0
            inp.mu_lat = 1.0
            inp.Vx_wheel = 16.0
            inp.gamma = 0.0
            best = max(best, abs(pick(model.compute(inp))))
        return best / Fz

    k_lon = _peak(1.0, 0.0, lambda o: o.Fx)
    k_lat = _peak(0.0, 0.6, lambda o: o.Fy)
    if k_lon <= 0.0 or k_lat <= 0.0:
        raise ValueError("mu_aniso sweep produced a non-positive peak "
                         f"(k_lon={k_lon}, k_lat={k_lat})")
    shape = "circle" if abs(k_lat - k_lon) <= 0.01 * k_lon else "ellipse"
    return shape, [float(k_lon), float(k_lat)]


def _git_sha(repo: Path = REPO) -> str:
    """Git SHA the bindings were built from, or ``"unknown"``.

    The build bakes the source HEAD into ``vdsim.__git_sha__``; a run-time
    ``git rev-parse`` of ``repo`` is the fallback for a module built without
    it (no git at build time).
    """
    baked = getattr(vdsim, "__git_sha__", "unknown")
    if baked != "unknown":
        return baked
    try:
        out = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=5)
        if out.returncode == 0:
            return out.stdout.decode().strip()
    except Exception:
        pass
    return "unknown"


def _vdsim_version() -> str:
    """Package version of the loaded module, or ``"source"`` for an in-tree build.

    The build bakes it into ``vdsim.__version__``, so an in-tree module is not
    labelled with the version of some other installed ``vdsim``. A module
    built before that attribute existed falls back to the installed
    distribution metadata.
    """
    baked = getattr(vdsim, "__version__", None)
    if baked is not None:
        return baked
    try:
        from importlib.metadata import version  # py3.8+
        return version("vdsim")
    except Exception:
        return "source"


def _obs_from_output(o: vdsim.SimOutput, steer_ratio=None, ref_offset=None) -> dict:
    """Observation dict of one session output (see the module contract).

    :param o: ``vdsim.SimOutput``.
    :param steer_ratio: steering ratio [-] read from the vehicle file, or
        ``None`` when the file does not state one; ``delta_sw`` is present
        only when it is given.
    :param ref_offset: ``(x_fwd, y_left)`` [m] of a body point from the CG
        (ISO 8855), or ``None``. When given, ``X_ref`` / ``Y_ref`` (ground
        position [m]) and ``vx_ref`` / ``vy_ref`` (body-frame velocity [m/s],
        rigid body ``v_P = v_CG + r x p``) of that point are added.
    """
    st = o.state
    wheels = []
    for i in range(4):
        fw = o.tire_forces_wheel[i]
        wheels.append({
            "Fx": float(fw[0]),
            "Fy": float(fw[1]),
            "Fz": float(o.Fz[i]),
            "alpha": float(o.slip_angle[i]),
            "kappa": float(o.slip_ratio[i]),
            "mu": float(o.wheel_mu[i]),
            "mu_peak": float(o.wheel_mu_peak[i]),
            "alpha_peak": float(o.wheel_alpha_peak[i]),
            "kappa_peak": float(o.wheel_kappa_peak[i]),
        })
    obs = {
        "X": float(st.position[0]),
        "Y": float(st.position[1]),
        "psi": float(st.yaw()),
        "vx": float(st.vx()),
        "vy": float(st.vy()),
        "r": float(st.yaw_rate()),
        "ax": float(o.ax),
        "ay": float(o.ay),
        "beta": float(st.beta()),
        "roll": float(o.roll),
        "pitch": float(o.pitch),
        "wheel": wheels,
        # Road-wheel steer the dynamics integrated over the last substep, i.e.
        # the command after the actuator chain (dead time, lag, rate limit).
        "delta_rw": float(o.steer_applied),
    }
    if steer_ratio is not None:
        obs["delta_sw"] = obs["delta_rw"] * float(steer_ratio)
    if ref_offset is not None:
        px, py = ref_offset
        c, s = math.cos(obs["psi"]), math.sin(obs["psi"])
        obs["X_ref"] = obs["X"] + c * px - s * py
        obs["Y_ref"] = obs["Y"] + s * px + c * py
        obs["vx_ref"] = obs["vx"] - obs["r"] * py
        obs["vy_ref"] = obs["vy"] + obs["r"] * px
    return obs


def _validate_friction_map(friction_map):
    if friction_map is None:
        return
    if not isinstance(friction_map, (list, tuple)):
        raise TypeError("friction_map must be a list of (x0, x1, mu) tuples")
    for i, seg in enumerate(friction_map):
        if not isinstance(seg, (list, tuple)) or len(seg) != 3:
            raise ValueError(f"friction_map[{i}]: expected (x0, x1, mu)")
        x0, x1, mu = float(seg[0]), float(seg[1]), float(seg[2])
        if not (math.isfinite(x0) and math.isfinite(x1) and math.isfinite(mu)):
            raise ValueError(f"friction_map[{i}]: non-finite value")
        if x0 >= x1:
            raise ValueError(f"friction_map[{i}]: x0>=x1 ({x0}>={x1})")
        if not (0.0 < mu <= 1.2):
            raise ValueError(f"friction_map[{i}]: mu={mu} outside (0, 1.2]")

def _validate_friction_map_2d(friction_map_2d):
    if friction_map_2d is None:
        return
    if not isinstance(friction_map_2d, (list, tuple)):
        raise TypeError("friction_map_2d must be a list of {polygon, mu} dicts")
    for i, entry in enumerate(friction_map_2d):
        if not isinstance(entry, dict):
            raise ValueError(f"friction_map_2d[{i}]: expected dict with polygon, mu")
        if "polygon" not in entry or "mu" not in entry:
            raise ValueError(f"friction_map_2d[{i}]: keys 'polygon' and 'mu' required")
        poly = entry["polygon"]
        mu = float(entry["mu"])
        if not isinstance(poly, (list, tuple)) or len(poly) < 3:
            raise ValueError(f"friction_map_2d[{i}]: polygon needs >= 3 vertices")
        for j, vtx in enumerate(poly):
            if not isinstance(vtx, (list, tuple)) or len(vtx) != 2:
                raise ValueError(f"friction_map_2d[{i}].polygon[{j}]: expected (x, y)")
            x, y = float(vtx[0]), float(vtx[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f"friction_map_2d[{i}].polygon[{j}]: non-finite vertex")
        if not math.isfinite(mu) or not (0.0 < mu <= 1.2):
            raise ValueError(f"friction_map_2d[{i}]: mu={mu} outside (0, 1.2]")


def _poly_patches_from_map_2d(friction_map_2d):
    out = []
    for entry in friction_map_2d:
        p = vdsim.PolygonMuPatch()
        p.polygon = [(float(x), float(y)) for x, y in entry["polygon"]]
        p.mu = float(entry["mu"])
        out.append(p)
    return out


_CHANNEL_KEYS = ("dead_time_s", "tau_s", "rate_limit", "dead_zone", "out_min", "out_max",
                 "backlash")
_STEER_EXTRA = ("friction", "inertia", "servo_kp", "servo_kd")
_BRAKE_EXTRA = ("thermal",)
_LUGRE_KEYS = ("sigma0", "sigma1", "sigma2", "Tc", "Ts", "ws")
_THERMAL_KEYS = ("heat_coeff", "cool_coeff", "T_ambient", "mu_T_temp", "mu_T_scale")


def _finite(v, where):
    v = float(v)
    if not math.isfinite(v):
        raise ValueError(f"actuator[{where}] must be finite")
    return v


def _fill_channel(dst, spec, where, extra=()):
    if not isinstance(spec, dict):
        raise ValueError(f"actuator[{where}] must be a dict of {list(_CHANNEL_KEYS)}")
    for k, v in spec.items():
        if k == "user":
            dst.user = v
        elif k in _CHANNEL_KEYS:
            setattr(dst, k, _finite(v, f"{where}][{k}"))
        elif k not in extra:
            raise ValueError(f"actuator[{where}]: unknown key {k!r}; "
                             f"allowed {list(_CHANNEL_KEYS) + ['user'] + list(extra)}")


def _conf_catalog_root():
    pkg = Path(__file__).resolve().parent
    if (pkg / "vdsim_configs" / "catalog" / "manifest.yaml").is_file():
        return pkg
    for c in (REPO, Path.cwd()):
        if (c / "configs" / "catalog" / "manifest.yaml").is_file():
            return c
    return REPO


def load_actuator_preset(preset_id: str) -> dict:
    """Body of a catalog part ``type: actuator`` (schema ``actuator_v1``) as a spec dict."""
    if str(pkg_dir := Path(__file__).resolve().parent) not in sys.path:
        sys.path.insert(0, pkg_dir.as_posix())
    from catalog import CatalogResolver
    doc = CatalogResolver(_conf_catalog_root()).load_part(preset_id)
    if doc["type"] != "actuator" or doc["schema"] != "actuator_v1":
        raise ValueError(f"{preset_id!r} is a {doc['type']}/{doc['schema']} part, not actuator/actuator_v1")
    return dict(doc["body"])


def _merge_spec(base, over):
    out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.items()}
    for ch, spec in over.items():
        if isinstance(spec, dict) and isinstance(out.get(ch), dict):
            out[ch].update(spec)
        else:
            out[ch] = spec
    return out


def build_actuator_params(spec):
    """Turn ``spec`` into a ``vdsim.ActuatorParams``.

    ``spec`` is one of
      * ``None`` -- identity (no actuator effect);
      * a ready ``vdsim.ActuatorParams``;
      * a catalog id string, e.g. ``"actuator.passenger_ev_nominal"``;
      * a dict with any of ``steer`` / ``throttle`` / ``brake`` (each a dict of
        ``dead_time_s``, ``tau_s``, ``rate_limit``, ``dead_zone``, ``out_min``,
        ``out_max``, ``backlash``, ``user``; steer also ``friction`` / ``inertia`` /
        ``servo_kp`` / ``servo_kd``; brake also ``thermal``). A ``"preset"`` key
        names a catalog id the other keys are laid over.
    Unlisted fields keep the identity default.
    """
    if spec is None:
        return vdsim.ActuatorParams()
    if isinstance(spec, vdsim.ActuatorParams):
        return spec
    if isinstance(spec, str):
        spec = {"preset": spec}
    if not isinstance(spec, dict):
        raise ValueError("actuator must be None, a preset id, vdsim.ActuatorParams or a dict")
    spec = dict(spec)
    if "preset" in spec:
        spec = _merge_spec(load_actuator_preset(spec.pop("preset")), spec)
    unknown = set(spec) - {"steer", "throttle", "brake"}
    if unknown:
        raise ValueError(f"actuator: unknown channel(s) {sorted(unknown)}; "
                         "allowed ['steer', 'throttle', 'brake']")
    ap = vdsim.ActuatorParams()
    if "steer" in spec:
        st = spec["steer"]
        _fill_channel(ap.steer.ch, st, "steer", _STEER_EXTRA)
        if isinstance(st.get("friction"), dict):
            ap.steer.friction.enabled = True
            for k, v in st["friction"].items():
                if k not in _LUGRE_KEYS:
                    raise ValueError(f"actuator[steer][friction]: unknown key {k!r}; allowed {list(_LUGRE_KEYS)}")
                setattr(ap.steer.friction, k, _finite(v, f"steer][friction][{k}"))
        for k in ("inertia", "servo_kp", "servo_kd"):
            if k in st:
                setattr(ap.steer, k, _finite(st[k], f"steer][{k}"))
    if "throttle" in spec:
        _fill_channel(ap.throttle, spec["throttle"], "throttle")
    if "brake" in spec:
        br = spec["brake"]
        _fill_channel(ap.brake.ch, br, "brake", _BRAKE_EXTRA)
        if isinstance(br.get("thermal"), dict):
            ap.brake.thermal_enabled = True
            for k, v in br["thermal"].items():
                if k not in _THERMAL_KEYS:
                    raise ValueError(f"actuator[brake][thermal]: unknown key {k!r}; allowed {list(_THERMAL_KEYS)}")
                if k in ("mu_T_temp", "mu_T_scale"):
                    setattr(ap.brake, k, [_finite(x, f"brake][thermal][{k}") for x in v])
                else:
                    setattr(ap.brake, k, _finite(v, f"brake][thermal][{k}"))
    return ap


def _validate_dt(control_dt: float, substep_dt: float):
    if not (control_dt > 0.0):
        raise ValueError("control_dt must be > 0")
    if not (substep_dt > 0.0):
        raise ValueError("substep_dt must be > 0")
    ratio = control_dt / substep_dt
    n = round(ratio)
    if abs(ratio - n) > 1e-9 or n < 1:
        raise ValueError("substep_dt must divide control_dt evenly")


def _fx_to_cmdl1(vp: vdsim.VehicleParams, delta: float, fx: float) -> vdsim.CmdL1:
    r_eff = vp.wheel_radius_nominal
    sf = vp.drive_split_front
    sr = 1.0 - sf
    taus = (
        sf * 0.5 * fx * r_eff,
        sf * 0.5 * fx * r_eff,
        sr * 0.5 * fx * r_eff,
        sr * 0.5 * fx * r_eff,
    )
    cmd = vdsim.CmdL1()
    cmd.steer_angle_wheel = delta
    if fx >= 0.0:
        cmd.motor_torque = [float(t) for t in taus]
        cmd.brake_torque = [0.0, 0.0, 0.0, 0.0]
    else:
        cmd.motor_torque = [0.0, 0.0, 0.0, 0.0]
        cmd.brake_torque = [abs(float(t)) for t in taus]
    return cmd


#: Meanings of the second command element ``u[1]`` accepted by
#: :class:`VDSimPlant` (``longitudinal=``): total longitudinal force [N] or
#: longitudinal acceleration target [m/s^2].
LONGITUDINAL_MODES = ("force", "accel")

#: Named observation reference points (``ref_point=``).
REF_POINTS = ("cg", "rear_axle", "front_axle")


def _ref_offset(spec, vp: vdsim.VehicleParams):
    """Body-frame offset ``(x_fwd, y_left)`` [m] from the CG of ``ref_point``.

    :param spec: one of :data:`REF_POINTS` or ``[x_fwd, y_left]`` [m] (ISO 8855).
    :param vp: vehicle whose ``cg_to_front`` / ``cg_to_rear`` place the axles.
    :returns: tuple of two floats.
    """
    if isinstance(spec, str):
        if spec not in REF_POINTS:
            raise ValueError(f"ref_point={spec!r} is not one of {REF_POINTS} "
                             "or [x_fwd, y_left]")
        return {"cg": (0.0, 0.0),
                "rear_axle": (-float(vp.cg_to_rear), 0.0),
                "front_axle": (float(vp.cg_to_front), 0.0)}[spec]
    try:
        px, py = (float(v) for v in spec)
    except (TypeError, ValueError):
        raise ValueError("ref_point must be a name or [x_fwd, y_left] in metres")
    if not (math.isfinite(px) and math.isfinite(py)):
        raise ValueError("ref_point offsets must be finite")
    return (px, py)


def _drive_brake_refs(vp: vdsim.VehicleParams, drive_torque_ref, brake_torque_ref):
    """Full-pedal total wheel torques [Nm] a direct-control session uses.

    Mirrors the fallback in ``SimSession``'s constructor (``sim_session.cpp``):
    a non-positive reference means ``max_motor_torque * final_drive_ratio``
    for drive and ``max_brake_torque`` for brake. Used for the record only.
    """
    drive = (float(drive_torque_ref) if drive_torque_ref > 0.0
             else float(vp.max_motor_torque * vp.final_drive_ratio))
    brake = (float(brake_torque_ref) if brake_torque_ref > 0.0
             else float(vp.max_brake_torque))
    return drive, brake


def trace_sample(o, t, u_steer, u_fx, channels, validity, vp=None):
  """Map one session output onto a trace sample (3.2 / 3.2.1).

  Shared by the plant path (:class:`VDSimPlant`) and the scenario path
  (``vdsim_lab.Experiment``) so the channel mapping has a single definition.
  ``channels`` decides what is written: a quantity the producer does not have
  is left out rather than zero-filled.

  :param o: ``vdsim.SimOutput`` at time ``t``.
  :param t: sample time [s].
  :param u_steer: commanded road-wheel steer [rad].
  :param u_fx: longitudinal command -- total force [N], written to ``u_fx``;
      or, when ``channels`` holds ``u_ax`` instead, the acceleration target
      [m/s^2] of a ``longitudinal="accel"`` run. Ignored when neither is in
      ``channels`` (pedal-commanded producers have no such value).
  :param channels: channel names this run records.
  :param vp: ``vdsim.VehicleParams`` of the run; needed only for the opt-in
      aero channels (``vdsim_trace.aero_channel_names``), which are evaluated
      with ``vdsim.compute_aero_loads`` at the sample's body-x speed.
  :param validity: ``channel_validity`` of the run (see
      :func:`channel_validity_from_dynamics`); a not-modeled channel is written
      as exactly ``0.0``.
  :returns: sample dict accepted by ``vdsim_trace.TraceWriter.append``.
  """
  st = o.state
  sample = {
      "t": float(t),
      "pose": (float(st.position[0]), float(st.position[1]), float(st.yaw())),
      "v_body": (float(st.vx()), float(st.vy())),
      "yaw_rate": float(st.yaw_rate()),
      "u_steer": float(u_steer),
      "wheel_F": [(float(o.tire_forces_wheel[i][0]),
                   float(o.tire_forces_wheel[i][1]),
                   float(o.Fz[i])) for i in range(4)],
      "wheel_mu": [float(o.wheel_mu[i]) for i in range(4)],
      "wheel_kappa": [float(o.slip_ratio[i]) for i in range(4)],
      "wheel_alpha": [float(o.slip_angle[i]) for i in range(4)],
  }
  if "u_fx" in channels:
      sample["u_fx"] = float(u_fx)
  elif "u_ax" in channels:
      # An acceleration target has its own channel; u_fx stays a force.
      sample["u_ax"] = float(u_fx)
  if "steer_applied" in channels:
      # Opt-in channel (vdsim_trace.OPTIONAL_CHANNELS): the realized steer, not
      # the command, so the actuator chain is visible next to u_steer.
      sample["steer_applied"] = float(o.steer_applied)
  # 0.3 channels. `channels` already excludes anything this level does not
  # produce, so no branch here can write a zero standing in for a quantity the
  # model does not have (3.2.1).
  if "a_body" in channels:
      # Straight from the dynamics -- never a difference of `v_body`, which at
      # dt=1 ms would hand the CG vector a noise floor larger than the signal
      # and would change size with `decimation`.
      sample["a_body"] = (float(o.ax), float(o.ay), float(o.az))
  if "pose_zrp" in channels:
      # z = settled CG height + the ride model's heave about it.
      sample["pose_zrp"] = (float(st.position[2]) + float(o.heave_z),
                            float(o.roll), float(o.pitch))
  if "wheel_road_dz" in channels:
      sample["wheel_road_dz"] = [float(o.contacts[i].road_dz) for i in range(4)]
  if "wheel_road_normal" in channels:
      sample["wheel_road_normal"] = [
          (float(o.contacts[i].normal[0]),
           float(o.contacts[i].normal[1]),
           float(o.contacts[i].normal[2])) for i in range(4)]
  # 0.5 channels. Recorded at every level; a model that does not compute the
  # quantity gets a literal 0.0 here, not whatever its state field happens to
  # hold, so a stale value can never pass as a measurement.
  if "wheel_travel" in channels:
      if _is_modeled(validity, "wheel_travel"):
          sample["wheel_travel"] = [float(st.susp_compression[i]) for i in range(4)]
      else:
          sample["wheel_travel"] = [0.0, 0.0, 0.0, 0.0]
  if "rp_rate" in channels:
      # State variables of the model (integrated roll/pitch rate), never a
      # difference of successive roll/pitch samples.
      if _is_modeled(validity, "rp_rate"):
          sample["rp_rate"] = (float(st.angular_velocity[0]),
                               float(st.angular_velocity[1]))
      else:
          sample["rp_rate"] = (0.0, 0.0)
  aero_names = [n for n in channels if n.startswith("aero_")]
  if aero_names:
      sample.update(_aero_sample(vp, float(st.vx()), aero_names))
  return sample


def _aero_sample(vp, vx, names):
  """Aero channel values at body-x speed ``vx`` -- the same core function the
  dynamics use, so the trace is the applied load, never a re-derivation.

  :param vp: ``vdsim.VehicleParams`` of the run.
  :param vx: body-x speed [m/s].
  :param names: aero channel names to fill.
  :returns: ``{name: value}``.
  """
  if vp is None:
      raise ValueError("aero trace channels need the run's VehicleParams (vp=)")
  loads = vdsim.compute_aero_loads(vp, vdsim.AeroState(vx))
  vals = {"aero_fz_total": float(loads.fz_total),
          "aero_balance_front": float(loads.balance_front),
          "aero_ld": float(loads.ld)}
  for e, el in zip(vp.aero_elements, loads.elements):
      vals["aero_%s_fz" % e.name] = float(el.fz)
      vals["aero_%s_fx" % e.name] = float(el.fx)
  return {n: vals[n] for n in names}


_VALIDITY_CHANNEL_ENUM = {"rp_rate": "RollPitchRate", "wheel_travel": "WheelTravel"}


def _is_modeled(validity, name):
  """True when ``validity[name]`` is ``modeled``; a missing entry is an error."""
  import vdsim_trace

  return validity[name] == vdsim_trace.VALIDITY_MODELED


def channel_validity_from_dynamics(dyn, model_level, channels):
  """Build the manifest ``channel_validity`` from what the model itself declares.

  Asks ``dyn.models_channel(...)`` -- the pure virtual each dynamics class in
  the core implements -- and never derives the answer from ``model_level``.
  ``not_modeled@<level>`` uses the run's own level so the marker and the
  manifest cannot disagree.

  :param dyn: ``vdsim.IVehicleDynamics`` of the built session.
  :param model_level: the run's ladder level.
  :param channels: channel names the trace records.
  :returns: ``{channel: "modeled" | "not_modeled@<level>"}`` for the recorded
      channels that carry a validity.
  """
  import vdsim_trace

  out = {}
  for name in vdsim_trace.VALIDITY_CHANNELS:
      if name not in channels:
          continue
      which = getattr(vdsim.ModeledChannel, _VALIDITY_CHANNEL_ENUM[name])
      out[name] = (vdsim_trace.VALIDITY_MODELED if dyn.models_channel(which)
                   else vdsim_trace.not_modeled_marker(model_level))
  return out


class _TireView:
    def __init__(self, dyn, ts: vdsim.TireSetup, wheel: int = 0):
        self._dyn = dyn
        self._ts = ts
        self._wheel = wheel

    @property
    def model(self):
        return self._dyn.tire(self._wheel)

    @property
    def params(self):
        return self._ts.wheel[self._wheel]


class _VehicleView:
    """Read-only vehicle -> part -> physics access path for data delivery.

    The simulation runtime is flat (VehicleParams + a tyre model in the dynamics);
    this is only a hierarchical *view* over those handles, added for user data access.

    TODO(parts): expose the other parts on this path — brake / steering / drivetrain /
    suspension / anti-roll-bar — as `plant.vehicle.<part>.model` / `.params`.
    Why: only the tyre has a live model handle today; the rest are either pluggable
    modules (IBrakeSystem etc., set via set_*_module with no getter) or flat fields under
    VehicleParams, so there is no per-part object to surface yet.
    Generalization path: add const getters on IVehicleDynamics for the installed modules
    (mirror tire()), bind them, then add a _PartView per subsystem here. Their params can
    already be read today via `plant.vehicle.params` (flat VehicleParams).
    """
    def __init__(self, vp, tire_view):
        self._vp = vp
        self.tire = tire_view

    @property
    def params(self):
        """VehicleParams (mass/geometry/drivetrain/steering/brake/aero/suspension)."""
        return self._vp


class VDSimPlant:
  """Pacejka plant with a direct Fx→torque path (no throttle map).

  ``level`` selects the model behind the seam — ``"L2"`` (7-DOF, the default and
  the historical behaviour), ``"L3"`` (14-DOF ride model), ``"L1"``
  (bicycle). ``"L4"`` raises ``ValueError``: this plant cannot attach
  suspension hardpoints, and without them L4 is L3 under another label. The control contract ``u = [delta_rad, Fx_total_N]`` does not
  change with the level; what changes is which quantities exist, and therefore
  which trace channels a run at that level is allowed to record.

  ``longitudinal="accel"`` turns ``u[1]`` into a longitudinal acceleration
  target [m/s^2]: the session's L5 cascade (``LongAxController``, the same
  class and gains as the RL ``accel`` action) maps it to pedals, and the
  pedals to wheel torque on the linear references ``drive_torque_ref`` /
  ``brake_torque_ref``. :meth:`conversion` describes the setting in force.

  ``ref_point`` adds the position and velocity of one body point to every
  observation (``X_ref``, ``Y_ref``, ``vx_ref``, ``vy_ref``); ``X`` / ``Y``
  stay at the CG.

  ``monitor`` (off by default) shows the run in the browser live monitor:
  ``True`` = port 8790, an ``int`` = that port, or a
  ``vdsim_livemonitor.LiveMonitor``. The world is declared here, the vehicle
  appears with the first :meth:`reset` / :meth:`step` and :meth:`close` marks
  the run finished. States go out at most 30 times per wall second, so a run
  faster than real time is shown fast. ``monitor_pace`` (e.g. 1.0) sleeps in
  :meth:`step` to hold that sim/wall ratio; ``monitor_wait_s`` waits up to that
  long for a page before returning. Both apply only with the monitor on. A
  monitor that cannot start prints one warning and the plant runs without it.
  """

  def __init__(
      self,
      config: str = "generic_sedan.yaml",
      friction_map=None,
      friction_map_2d=None,
      base_mu: float = 0.9,
      control_dt: float = 0.05,
      substep_dt: float = 5e-4,
      level: str = "L2",
      actuator=None,
      drive_torque_ref: float = 0.0,
      brake_torque_ref: float = 0.0,
      ref_point=None,
      longitudinal: str = "force",
      monitor=None,
      monitor_pace=None,
      monitor_wait_s: float = 0.0,
  ):
      if longitudinal not in LONGITUDINAL_MODES:
          raise ValueError(
              f"longitudinal={longitudinal!r} is not one of {list(LONGITUDINAL_MODES)}")
      if level not in PLANT_LEVELS:
          raise ValueError(
              f"level={level!r} is not one of {list(PLANT_LEVELS)}")
      # The accel converter feeds back the measured ax, which the L1 bicycle
      # does not compute (ax_body_est stays 0): its integrator would wind up on
      # a constant error instead of tracking the target.
      if longitudinal == "accel" and level == "L1":
          raise ValueError(
              "longitudinal='accel' needs a level that reports ax; the L1 "
              "bicycle reports ax = 0, so the ax feedback cannot track -- use L2 or L3")
      # This plant has no hardpoint input, so an L4 plant would be L3 physics
      # recorded as model_level L4. Same refusal as vdsim_lab._build_session;
      # the leading sentence is shared on purpose and a test compares the two.
      if level == "L4":
          raise ValueError(
              "level='L4' with no suspension hardpoints attached is L3 under another "
              "label (the two are bit-identical until an attach); VDSimPlant has no "
              "hardpoint input -- use level='L3', or vdsim_lab.Experiment with "
              "kinematics={'front': <stem>, 'rear': <stem>}")
      if not math.isfinite(base_mu) or not (0.0 < base_mu <= 1.2):
          raise ValueError(f"base_mu={base_mu} outside (0, 1.2]")
      if friction_map is not None and friction_map_2d is not None:
          raise ValueError("pass friction_map or friction_map_2d, not both")
      _validate_friction_map(friction_map)
      _validate_friction_map_2d(friction_map_2d)
      _validate_dt(control_dt, substep_dt)

      vp_path = resolve_vehicle_config(config)
      self._vp = vdsim.VehicleParams.from_yaml(str(vp_path))
      self._vp.plant_path = True
      self._ts = _load_tire_setup_for_vehicle(vp_path)
      for i in range(4):
          self._ts.wheel[i].lugre.enabled = False

      patches = [(float(a), float(b), float(m)) for a, b, m in (friction_map or [])]
      poly_patches = _poly_patches_from_map_2d(friction_map_2d or [])

      sp = vdsim.SolverParams()
      sp.integrator = vdsim.Integrator.RK4
      sp.max_substep_dt = min(substep_dt, sp.max_substep_dt)
      sp = level_solver_params(level, substep_dt, sp)
      self._solver = solver_record(sp) if level == "L5" else None

      opts = vdsim.DirectControlSessionOptions()
      opts.nominal_dt = float(substep_dt)
      opts.friction.base_mu = float(base_mu)
      opts.friction.x_bands = patches
      opts.friction.polygons = poly_patches
      opts.friction.blend_distance = 1.0
      opts.level = level
      if actuator is not None:
          opts.actuator = build_actuator_params(actuator)
      opts.drive_torque_ref = float(drive_torque_ref)
      opts.brake_torque_ref = float(brake_torque_ref)

      self.level = level
      self.longitudinal = longitudinal
      self._ref_point = ref_point
      self._ref_offset = None if ref_point is None else _ref_offset(ref_point, self._vp)
      self._actuator_built = self._actuator_spec = actuator
      self._torque_refs = _drive_brake_refs(self._vp, drive_torque_ref, brake_torque_ref)
      self.control_dt = float(control_dt)
      self.substep_dt = float(substep_dt)
      self._n_sub = int(round(control_dt / substep_dt))
      self._sess = vdsim.make_direct_control_session(self._vp, self._ts, sp, opts)
      self._dyn = self._sess.dynamics()

      # Trace recording is opt-in; a plant that always records hands its
      # regression to the customer. `_trace is None` is the OFF fast path.
      self._trace = None
      self._t = 0.0
      self._vp_path = vp_path
      # delta_sw needs a ratio the vehicle file states; VehicleParams falls
      # back to a default when the key is absent, and that default must not
      # turn into an observation.
      ratio = _load_yaml_sidecar(Path(vp_path), "steering_ratio")
      self._delta_sw_ratio = (float(self._vp.steering_ratio)
                              if ratio is not None else None)
      self._plant_params = {
          "config": Path(vp_path).name,
          "level": level,
          "base_mu": float(base_mu),
          "friction_map": [[float(a), float(b), float(m)] for a, b, m in (friction_map or [])],
          "friction_map_2d": [
              {"polygon": [[float(x), float(y)] for x, y in e["polygon"]],
               "mu": float(e["mu"])}
              for e in (friction_map_2d or [])],
          "substep_dt": float(substep_dt),
          "integrator": "RK4",
          "vehicle": {
              "mass": float(self._vp.mass),
              "wheelbase": float(self._vp.wheelbase),
              "cg_to_front": float(self._vp.cg_to_front),
              "cg_to_rear": float(self._vp.cg_to_rear),
              "track_front": float(self._vp.track_front),
              "track_rear": float(self._vp.track_rear),
              "cg_height": float(self._vp.cg_height),
              "wheel_radius_nominal": float(self._vp.wheel_radius_nominal),
              "steering_ratio": float(self._vp.steering_ratio),
              "drive_split_front": float(self._vp.drive_split_front),
          },
          "tire": {
              "backend": str(self._ts.wheel[0].backend),
              "mu_nominal": float(self._ts.wheel[0].mu_nominal),
              "Fz_nominal": float(self._ts.wheel[0].Fz_nominal),
              "tir": Path(self._ts.wheel[0].tir_path).name if self._ts.wheel[0].tir_path else "",
          },
      }

      # Live monitor is opt-in; `_mon is None` is the OFF path (nothing
      # imported, no socket, no thread).
      self._mon = None
      self._mon_t = 0.0       # simulated time since construction, not reset
      if monitor is not None and monitor is not False:
          import vdsim_livemonitor
          self._mon = vdsim_livemonitor.open_monitor(
              monitor, mode="code", n_vehicles=1, ids=[vdsim_livemonitor.LEGACY_ID],
              vehicle=vdsim_livemonitor.vehicle_geometry(self._vp),
              pace=monitor_pace, wait_s=monitor_wait_s)

  @property
  def actuator(self):
      """The ``vdsim.ActuatorParams`` currently in force (a copy)."""
      return self._sess.actuator()

  def set_actuator(self, actuator):
      """Swap the actuator on a live plant, keeping its memory (delay line, lag).

      Accepts what ``actuator=`` accepts. The next :meth:`reset` returns to the
      as-built actuator.
      """
      self._sess.set_actuator(build_actuator_params(actuator))
      self._actuator_spec = actuator

  def reset(self, state0=None):
      """Seed pose [X, Y, psi, vx, vy, r]; wheel spin = vx/R (make_init_state)."""
      if state0 is None:
          state0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      if len(state0) != 6:
          raise ValueError("state0 must have length 6: [X, Y, psi, vx, vy, r]")
      x, y, psi, vx, vy, r = (float(v) for v in state0)
      if not all(math.isfinite(v) for v in (x, y, psi, vx, vy, r)):
          raise ValueError("state0 values must be finite")
      s0 = vdsim.make_init_state_6dof(
          self._vp, self._ts.wheel[0], x=x, y=y, yaw=psi, vx=vx, vy=vy, r=r)
      self._sess.reset(s0)
      self._actuator_spec = self._actuator_built   # reset restores the as-built actuator
      self._t = 0.0
      obs = self._obs()
      if self._mon is not None:
          self._mon.reset([0])
          self._mon.publish([{**obs, "t": self._t}], self._mon_t)
      return obs

  def step(self, u):
      """Advance one control period (ZOH); integrate internally at substep_dt.

      :param u: ``[delta_rad, Fx_total_N]``, or ``[delta_rad, ax_mps2]`` when
          the plant was built with ``longitudinal="accel"``.
      """
      if not isinstance(u, (list, tuple)) or len(u) != 2:
          raise ValueError("step(u): u must be [delta_rad, Fx_total_N] "
                           "([delta_rad, ax_mps2] with longitudinal='accel')")
      delta, fx = float(u[0]), float(u[1])
      if not math.isfinite(delta) or not math.isfinite(fx):
          raise ValueError("delta and the longitudinal command must be finite")
      if self._trace is not None:
          # Ask before building the sample: on a decimated step the dict
          # construction is the whole cost, so skipping it is the saving.
          if self._trace.due:
              self._record(delta, fx)
          else:
              self._trace.skip()
      cmd = self._command(delta, fx)
      for _ in range(self._n_sub):
          self._sess.set_input(cmd)
          self._sess.tick(self.substep_dt)
      self._t += self.control_dt
      obs = self._obs()
      if self._mon is not None:
          self._mon_t += self.control_dt
          if self._mon.due():
              self._mon.publish([{**obs, "t": self._t}], self._mon_t)
          self._mon.hold_pace(self._mon_t)
      return obs

  @property
  def monitor(self):
      """The ``vdsim_livemonitor.MonitorAttachment`` of this plant, or None."""
      return self._mon

  def close(self):
      """Mark a monitored run finished and release its monitor (the page
      keeps the last frame). Without a monitor this does nothing; the plant
      stays usable either way."""
      if self._mon is not None:
          self._mon.close()
          self._mon = None

  def _command(self, delta: float, lon: float):
      """Session command of one ``(delta, u[1])`` pair, per ``longitudinal``.

      :param delta: road-wheel steer [rad].
      :param lon: total longitudinal force [N] (``"force"``) or acceleration
          target [m/s^2] (``"accel"``).
      :returns: ``vdsim.CmdL1`` or ``vdsim.CmdL5``.
      """
      if self.longitudinal == "force":
          return _fx_to_cmdl1(self._vp, delta, lon)
      cmd = vdsim.CmdL5()
      cmd.ax_target = lon
      cmd.steer_angle_wheel = delta
      return cmd

  def conversion(self) -> dict:
      """How ``u[1]`` becomes wheel torque, for the result and trace record.

      ``longitudinal`` is ``"force"`` or ``"accel"``; ``accel_conversion`` is
      ``None`` for force and otherwise names the converter, its gains (read
      from the session), the pedal-to-torque references and what it does not
      do (no resistance feed-forward, no ``ax`` clamp).
      """
      if self.longitudinal == "force":
          return {"longitudinal": "force", "accel_conversion": None}
      g = self._sess.cascade_gains().ax
      drive, brake = self._torque_refs
      return {
          "longitudinal": "accel",
          "accel_conversion": {
              "kind": "cmdl5_feedback",
              "controller": "LongAxController",
              "law": "PI + feed-forward on measured ax "
                     "(u = kp*e + ki*int(e) + kd*de/dt + kff*ax_target)",
              "gains": {"kp": float(g.kp), "ki": float(g.ki), "kd": float(g.kd),
                        "kff": float(g.kff), "i_max": float(g.i_max)},
              "pedal_to_torque": "linear",
              "drive_torque_ref_Nm": drive,
              "brake_torque_ref_Nm": brake,
              "drive_split_front": float(self._vp.drive_split_front),
              "resistance_compensation": False,
              "ax_limit_mps2": None,
          },
      }

  def ref_point_record(self):
      """``ref_point`` setting for the record, or ``None`` when not set."""
      if self._ref_offset is None:
          return None
      return {"spec": (self._ref_point if isinstance(self._ref_point, str)
                       else list(self._ref_offset)),
              "offset_m": list(self._ref_offset),
              "cg_to_front_m": float(self._vp.cg_to_front),
              "cg_to_rear_m": float(self._vp.cg_to_rear)}

  def provenance(self) -> dict:
      """Version and input identity of this plant, from the functions that
      fill the trace ``repro`` block.

      ``actuator`` is ``None`` (built-in), ``{"preset": id}`` or
      ``{"param_hash": ...}`` of a dict spec, for the actuator in force
      (:meth:`set_actuator` included).
      """
      import vdsim_trace

      act = self._actuator_spec
      if act is None:
          act_rec = None
      elif isinstance(act, str):
          act_rec = {"preset": act}
      else:
          # A dict spec is hashed; a ready ActuatorParams or a dict holding a
          # callable has no JSON form, so its hash is recorded as None.
          try:
              act_rec = {"param_hash": vdsim_trace.param_hash(act)}
          except (TypeError, ValueError):
              act_rec = {"param_hash": None}
      rec = {
          "vdsim_version": _vdsim_version(),
          "vdsim_git_sha": _git_sha(),
          "vdsim_git_dirty": getattr(vdsim, "__git_dirty__", None),
          "vehicle_config": str(self._vp_path),
          "param_hash": vdsim_trace.param_hash(self._plant_params),
          "level": self.level,
          "actuator": act_rec,
      }
      # L5 runs at solver settings the plant picked (level_solver_params);
      # record what was used. Other levels keep their earlier record unchanged.
      if self._solver is not None:
          rec["solver"] = dict(self._solver)
      return rec

  # ---- trace recording (opt-in) ---------------------------------------
  def enable_trace(self, path, decimation=None, seed=None, run_id=None,
                   producer=None, tags=None, role="plant", extra_channels=(),
                   manifest_extra=None):
      """Start recording this run to a ``.vdtrace``. Off unless called.

      One sample is offered per :meth:`step`, taken *before* the step is
      integrated: the pose/forces are the state at time ``t`` and ``u_steer`` /
      ``u_fx`` are the command held over ``[t, t+control_dt)``.

      :param path: output ``.vdtrace`` path.
      :param decimation: keep 1 of every N samples. ``None`` picks the smallest
          N whose record rate stays at or above ``TRACE_TARGET_HZ`` (100 Hz),
          which is ample for a 30 fps render — 1 kHz control gives N=10, while
          a 20 Hz control loop stays at N=1 and loses nothing.
      :param seed: RNG seed of the run, recorded for replay.
      :param run_id: run identifier; defaults to the trace file stem.
      :param producer: ``{"name","version"}`` of the calling script.
      :param tags: free-form dict merged into the manifest as ``tags``.
      :param role: manifest ``role`` (§3.1). Defaults to ``"plant"`` because
          this class *is* the plant; pass ``"predictor"`` when the same object
          is driven as an optimiser's internal model rather than as the run
          under verification.
      :param extra_channels: channels beyond the level's own set. Names in
          :data:`SAMPLED_OPTIONAL_CHANNELS` (``"steer_applied"``) are filled
          by every sample, :meth:`step` included. ``"aero"`` adds the aero
          channels of :func:`vdsim_trace.aero_channel_names` the same way. Any other name is for a
          caller that drives the session itself and appends its samples to
          ``_trace`` (``vdsim_closedloop`` timing channels); :meth:`step`
          cannot fill those, so leave them out when stepping the plant.
      :param manifest_extra: manifest keys merged next to ``tags``.
      :returns: the resolved decimation.
      """
      import vdsim_trace

      if self._trace is not None:
          raise RuntimeError("trace already enabled; call finalize_trace() first")
      if decimation is None:
          decimation = max(1, int(round(1.0 / (TRACE_TARGET_HZ * self.control_dt))))
      decimation = int(decimation)

      shape, aniso = measure_mu_aniso(self._ts.wheel[0])
      body_lwh, wheel_width = body_dimensions(self._vp_path)
      path = Path(path)
      geometry = {
          "wheelbase_m": float(self._vp.wheelbase),
          "track_m": float(self._vp.track_front),
          "steer_ratio": float(self._vp.steering_ratio),
          "track_front_m": float(self._vp.track_front),
          "track_rear_m": float(self._vp.track_rear),
          "cg_to_front_m": float(self._vp.cg_to_front),
          "cg_to_rear_m": float(self._vp.cg_to_rear),
          "wheel_radius_m": float(self._vp.wheel_radius_nominal),
          # -- 0.3 (§3.1.1): the 3D renderer sizes the body box, the wheel
          # -- cylinders and the CG acceleration vector from these.
          "mass_kg": float(self._vp.mass),
          "cg_height_m": float(self._vp.cg_height),
          "wheel_width_m": wheel_width,
          "body_lwh_m": body_lwh,
      }
      repro = {
          "vdsim_version": _vdsim_version(),
          "git_sha": _git_sha(),
          "param_hash": vdsim_trace.param_hash(self._plant_params),
          "seed": seed,
          "dt_s": float(self.control_dt * decimation),
          "run_id": run_id or path.stem,
          "control_dt_s": float(self.control_dt),
          "substep_dt_s": float(self.substep_dt),
          "decimation": decimation,
      }
      # Opt-in channels trace_sample() can fill itself join the per-step set,
      # so step() records them too; the rest stay with the caller.
      if "u_ax" in extra_channels:
          raise ValueError("u_ax is recorded by longitudinal='accel' plants "
                           "only; do not list it in extra_channels")
      own = tuple(n for n in extra_channels if n in SAMPLED_OPTIONAL_CHANNELS)
      if "aero" in extra_channels:
          # One switch for the aero channels: per element fz/fx (names from
          # the vehicle file) plus the totals; the four-key path has totals only.
          own += vdsim_trace.aero_channel_names(
              [e.name for e in self._vp.aero_elements])
      extra_channels = tuple(n for n in extra_channels
                             if n not in own and n != "aero")
      if self.longitudinal == "accel":
          # The command is an acceleration target [m/s^2]: it goes to u_ax and
          # u_fx, a force [N] to every reader, is left out, not mislabelled.
          base = tuple(n for n in vdsim_trace.BASE_CHANNELS if n != "u_fx")
          self._channels = (vdsim_trace.channels_for_level(self.level, base=base)
                            + ("u_ax",) + own)
      else:
          self._channels = vdsim_trace.channels_for_level(self.level) + own
      self._validity = channel_validity_from_dynamics(
          self._dyn, self.level, self._channels)
      extra = dict(manifest_extra or {})
      extra["tags"] = dict(tags or {})
      # Settings that change what u_fx and the observation mean; a default
      # plant (force command, no reference point) writes neither key.
      if self.longitudinal != "force":
          extra.update(self.conversion())
      if self._ref_offset is not None:
          extra["ref_point"] = self.ref_point_record()
      self._trace = vdsim_trace.TraceWriter(
          path=path,
          geometry=geometry,
          model_level=self.level,
          contact_scope=CONTACT_SCOPE_BY_LEVEL[self.level],
          # This plant builds a direct-control session and has no attach call,
          # so no hardpoints are ever behind it; the manifest must say so.
          kinematics_attached=False,
          channel_validity=self._validity,
          channels=self._channels + tuple(extra_channels),
          tire={"friction_shape": shape, "mu_aniso": aniso,
                "mu_aniso_source": "measured"},
          repro=repro,
          producer=producer or {"name": Path(sys.argv[0]).name or "python",
                                "version": _vdsim_version()},
          decimation=decimation,
          extra=extra,
          role=role,
      )
      return decimation

  def finalize_trace(self):
      """Flush channels, freeze the manifest and close the trace.

      :returns: the written path, or ``None`` when recording was never enabled.
      """
      if self._trace is None:
          return None
      writer, self._trace = self._trace, None
      return writer.finalize()

  @property
  def trace_path(self):
      """Path of the trace currently being recorded, or ``None``."""
      return None if self._trace is None else self._trace.path

  def _record(self, delta: float, fx: float):
      """Offer one sample of the current state to the writer."""
      self._trace.append(trace_sample(self._sess.output(), self._t, delta, fx,
                                      self._channels, self._validity, self._vp))

  @property
  def vehicle(self):
      return _VehicleView(self._vp, _TireView(self._dyn, self._ts))

  @property
  def tire_model(self):
      return self._dyn.tire(0)

  @property
  def delta_sw_ratio(self):
      """Steering ratio [-] behind ``obs["delta_sw"]``, or ``None`` when the
      vehicle file states none and ``delta_sw`` is therefore not observed."""
      return self._delta_sw_ratio

  def _obs_from(self, out) -> dict:
      """Observation dict of a ``vdsim.SimOutput`` of this plant's session."""
      return _obs_from_output(out, self._delta_sw_ratio, self._ref_offset)

  def _obs(self) -> dict:
      return self._obs_from(self._sess.output())
