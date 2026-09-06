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

Built on vdsim_lab config resolution + SimSession (direct CmdL1 torque path).

Trace recording (opt-in, default OFF) writes one ``.vdtrace`` per run::

    plant.enable_trace("runs/r001/run.vdtrace")
    ...                                   # step() as usual
    plant.finalize_trace()

See :mod:`vdsim_trace` for the container contract.
"""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
try:
    import vdsim
except ImportError:
    sys.path.insert(0, str(REPO / "build" / "python"))
    import vdsim

def _conf_root():
    for c in (REPO / "configs", Path.cwd() / "configs",
              Path(__file__).resolve().parent / "vdsim_configs"):
        if c.is_dir():
            return c
    return REPO / "configs"


def resolve_vehicle_config(name: str = "ioniq5_awd") -> Path:
    """Resolve a vehicle preset YAML (e.g. ``ioniq5_awd``)."""
    stem = Path(name).stem
    root = _conf_root()
    for sub in ("vehicles", ""):
        p = root / sub / f"{stem}.yaml" if sub else root / f"{stem}.yaml"
        if p.is_file():
            return p
    raise FileNotFoundError(
        f"vehicle config '{name}' not found under configs/vehicles/ "
        f"(searched {_conf_root()})")


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
    rel = _load_yaml_sidecar(vp_path, "tire_yaml")
    if not rel:
        tp_path = _conf_root() / "parts/tire/ioniq5_pac2002.yaml"
    else:
        tp_path = _conf_root() / rel
    if not tp_path.is_file():
        raise FileNotFoundError(f"tire yaml '{rel}' not found (from {vp_path})")
    tp = vdsim.TireParams.from_yaml(str(tp_path))
    if tp.tir_path:
        tp.tir_path = _resolve_tir_path(tp_path, tp.tir_path)
    return vdsim.TireSetup(tp)


#: Record rate the default `decimation` targets [Hz]. 100 Hz is >3x a 30 fps
#: render and keeps 1 kHz control loops at a 10x storage saving.
TRACE_TARGET_HZ = 100.0


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
        model = vdsim.create_pacejka_mf96(tp)
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
    """Short git SHA of the working tree, or ``"unknown"`` outside a checkout."""
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
    """Installed distribution version, or ``"source"`` for an in-tree build."""
    try:
        from importlib.metadata import version  # py3.8+
        return version("vdsim")
    except Exception:
        return "source"


def _obs_from_output(o: vdsim.SimOutput) -> dict:
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
    return {
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
    }


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
  """Ld2 7DOF Pacejka plant with direct Fx→torque path (no throttle map)."""

  def __init__(
      self,
      config: str = "ioniq5_awd.yaml",
      friction_map=None,
      friction_map_2d=None,
      base_mu: float = 0.9,
      control_dt: float = 0.05,
      substep_dt: float = 5e-4,
  ):
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

      opts = vdsim.DirectControlSessionOptions()
      opts.nominal_dt = float(substep_dt)
      opts.friction.base_mu = float(base_mu)
      opts.friction.x_bands = patches
      opts.friction.polygons = poly_patches
      opts.friction.blend_distance = 1.0

      self.control_dt = float(control_dt)
      self.substep_dt = float(substep_dt)
      self._n_sub = int(round(control_dt / substep_dt))
      self._sess = vdsim.make_direct_control_session(self._vp, self._ts, sp, opts)
      self._dyn = self._sess.dynamics()

      # Trace recording is opt-in; a plant that always records hands its
      # regression to the customer. `_trace is None` is the OFF fast path.
      self._trace = None
      self._t = 0.0
      self._plant_params = {
          "config": Path(vp_path).name,
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
      self._t = 0.0
      return _obs_from_output(self._sess.output())

  def step(self, u):
      """Advance one control period (ZOH); integrate internally at substep_dt."""
      if not isinstance(u, (list, tuple)) or len(u) != 2:
          raise ValueError("step(u): u must be [delta_rad, Fx_total_N]")
      delta, fx = float(u[0]), float(u[1])
      if not math.isfinite(delta) or not math.isfinite(fx):
          raise ValueError("delta and Fx_total must be finite")
      if self._trace is not None:
          # Ask before building the sample: on a decimated step the dict
          # construction is the whole cost, so skipping it is the saving.
          if self._trace.due:
              self._record(delta, fx)
          else:
              self._trace.skip()
      cmd = _fx_to_cmdl1(self._vp, delta, fx)
      for _ in range(self._n_sub):
          self._sess.set_input(cmd)
          self._sess.tick(self.substep_dt)
      self._t += self.control_dt
      return self._obs()

  # ---- trace recording (opt-in) ---------------------------------------
  def enable_trace(self, path, decimation=None, seed=None, run_id=None,
                   producer=None, tags=None, role="plant"):
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
      :returns: the resolved decimation.
      """
      import vdsim_trace

      if self._trace is not None:
          raise RuntimeError("trace already enabled; call finalize_trace() first")
      if decimation is None:
          decimation = max(1, int(round(1.0 / (TRACE_TARGET_HZ * self.control_dt))))
      decimation = int(decimation)

      shape, aniso = measure_mu_aniso(self._ts.wheel[0])
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
      self._trace = vdsim_trace.TraceWriter(
          path=path,
          geometry=geometry,
          tire={"friction_shape": shape, "mu_aniso": aniso,
                "mu_aniso_source": "measured"},
          repro=repro,
          producer=producer or {"name": Path(sys.argv[0]).name or "python",
                                "version": _vdsim_version()},
          decimation=decimation,
          extra={"tags": dict(tags or {})},
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
      o = self._sess.output()
      st = o.state
      self._trace.append({
          "t": self._t,
          "pose": (float(st.position[0]), float(st.position[1]), float(st.yaw())),
          "v_body": (float(st.vx()), float(st.vy())),
          "yaw_rate": float(st.yaw_rate()),
          "u_steer": delta,
          "u_fx": fx,
          "wheel_F": [(float(o.tire_forces_wheel[i][0]),
                       float(o.tire_forces_wheel[i][1]),
                       float(o.Fz[i])) for i in range(4)],
          "wheel_mu": [float(o.wheel_mu[i]) for i in range(4)],
          "wheel_kappa": [float(o.slip_ratio[i]) for i in range(4)],
          "wheel_alpha": [float(o.slip_angle[i]) for i in range(4)],
      })

  @property
  def vehicle(self):
      return _VehicleView(self._vp, _TireView(self._dyn, self._ts))

  @property
  def tire_model(self):
      return self._dyn.tire(0)

  def _obs(self) -> dict:
      return _obs_from_output(self._sess.output())
