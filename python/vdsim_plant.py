"""VDSimPlant — Ld2 7DOF closed-loop plant API for external MPC (VLA thesis).

Contract (stable):
  Control u = [delta_roadwheel_rad, Fx_total_N]  (ISO 8855, force intent at CG).
  Observation dict each step (true state, contact-frame tyre forces, FL0..RR3):

    X, Y, psi, vx, vy, r, ax, ay, beta   — vehicle [m, rad, m/s, m/s²]
    wheel: list[4] of {Fx, Fy, Fz, alpha, kappa, mu, mu_peak, alpha_peak, kappa_peak}
      Fx, Fy — tyre contact / wheel frame [N]  (+Fx drive, +Fy left)
      Fz [N], alpha [rad], kappa [-], mu [-] contact friction used this step
      mu_peak [-] realized load-dependent peak coefficient (force/Fz)
      alpha_peak [rad], kappa_peak [-] MF pure-slip peak slip at this Fz

Built on vdsim_lab config resolution + SimSession (direct CmdL1 torque path).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
try:
    import vdsim
except ImportError:
    sys.path.insert(0, str(REPO / "build" / "python"))
    import vdsim

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
_EPS = 1e-9


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


def _load_tire_for_vehicle(vp_path: Path) -> vdsim.TireParams:
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
    return tp


def _validate_friction_map(friction_map, base_mu: float):
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
    for i in range(4):
        if fx >= 0.0:
            cmd.motor_torque[i] = taus[i]
            cmd.brake_torque[i] = 0.0
        else:
            cmd.motor_torque[i] = 0.0
            cmd.brake_torque[i] = abs(taus[i])
    return cmd


class VDSimPlant:
  """Ld2 7DOF Pacejka plant with direct Fx→torque path (no throttle map)."""

  def __init__(
      self,
      config: str = "ioniq5_awd.yaml",
      friction_map=None,
      base_mu: float = 0.9,
      control_dt: float = 0.05,
      substep_dt: float = 5e-4,
  ):
      if not math.isfinite(base_mu) or not (0.0 < base_mu <= 1.2):
          raise ValueError(f"base_mu={base_mu} outside (0, 1.2]")
      _validate_friction_map(friction_map, base_mu)
      _validate_dt(control_dt, substep_dt)

      vp_path = resolve_vehicle_config(config)
      self._vp = vdsim.VehicleParams.from_yaml(str(vp_path))
      self._vp.plant_path = True
      self._tp = _load_tire_for_vehicle(vp_path)
      self._tp.lugre.enabled = False

      patches = [(float(a), float(b), float(m)) for a, b, m in (friction_map or [])]

      sp = vdsim.SolverParams()
      sp.integrator = vdsim.Integrator.RK4
      sp.max_substep_dt = min(substep_dt, sp.max_substep_dt)

      self.control_dt = float(control_dt)
      self.substep_dt = float(substep_dt)
      self._n_sub = int(round(control_dt / substep_dt))
      self._sess = vdsim.make_vla_plant_session(
          self._vp, self._tp, 0.0, float(base_mu), patches, sp, substep_dt)
      self._dyn = self._sess.dynamics()

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
          self._vp, self._tp, x=x, y=y, yaw=psi, vx=vx, vy=vy, r=r)
      self._sess.reset(s0)
      return self._obs()

  def step(self, u):
      """Advance one control period (ZOH); integrate internally at substep_dt."""
      if not isinstance(u, (list, tuple)) or len(u) != 2:
          raise ValueError("step(u): u must be [delta_rad, Fx_total_N]")
      delta, fx = float(u[0]), float(u[1])
      if not math.isfinite(delta) or not math.isfinite(fx):
          raise ValueError("delta and Fx_total must be finite")
      cmd = _fx_to_cmdl1(self._vp, delta, fx)
      for _ in range(self._n_sub):
          self._sess.set_input(cmd)
          self._sess.tick(self.substep_dt)
      return self._obs()

  @property
  def tire_model(self):
      """The live tire model this plant runs (ITireModel). Single source of truth for
      offline queries — friction ellipse / combined-slip peak via .compute(TireInput).
      Same instance the integrator uses; no re-load from the .tir."""
      return self._dyn.tire()

  def _obs(self) -> dict:
      st = self._sess.state()
      dyn = self._dyn
      fw = dyn.tire_forces_wheel()
      fz = dyn.tire_Fz()
      alpha = dyn.wheel_slip_angle()
      kappa = dyn.wheel_slip_ratio()
      mu_w = dyn.wheel_mu()
      mu_peak_w = dyn.wheel_mu_peak()
      alpha_peak_w = dyn.wheel_alpha_peak()
      kappa_peak_w = dyn.wheel_kappa_peak()
      wheels = []
      for i in range(4):
          wheels.append({
              "Fx": float(fw[i][0]),
              "Fy": float(fw[i][1]),
              "Fz": float(fz[i]),
              "alpha": float(alpha[i]),
              "kappa": float(kappa[i]),
              "mu": float(mu_w[i]),
              "mu_peak": float(mu_peak_w[i]),
              "alpha_peak": float(alpha_peak_w[i]),
              "kappa_peak": float(kappa_peak_w[i]),
          })
      return {
          "X": float(st.position[0]),
          "Y": float(st.position[1]),
          "psi": float(st.yaw()),
          "vx": float(st.vx()),
          "vy": float(st.vy()),
          "r": float(st.yaw_rate()),
          "ax": float(dyn.ax_body_est()),
          "ay": float(dyn.ay_body_est()),
          "beta": float(st.beta()),
          "wheel": wheels,
      }
