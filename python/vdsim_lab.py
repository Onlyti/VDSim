"""vdsim_lab — fluent experiment builders for VDSim.

Assemble a run from four pieces — Vehicle, Road, Maneuver, Sensors — and execute
it, instead of hand-writing make_sim_session + the step loop each time. Wraps the
low-level `vdsim` API (works whether vdsim is pip-installed or built in tree).

    from vdsim_lab import Vehicle, Tire, Road, Maneuver, Sensors, Experiment

    res = (Experiment(level="L3")
           .vehicle(Vehicle.preset("sedan").set(final_drive_ratio=5.0))
           .road(Road.iso8608("C"))
           .maneuver(Maneuver.step_steer(v=20, steer=0.03))
           .sensors(Sensors().gnss(pos_std=0.5).imu())
           .run(duration=8.0))
    res.to_csv("/tmp/run.csv")          # ground-truth + per-wheel (Fz, alpha, kappa, Fx, Fy)
    print(res.summary())
"""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
try:
    import vdsim
except ImportError:
    sys.path.insert(0, str(REPO / "build" / "python"))
    import vdsim


def _conf_root():
    """Config root: dev tree (REPO/configs), else the caller's project
    (./configs), else the presets bundled into the wheel (vdsim_configs)."""
    for c in (REPO / "configs", Path.cwd() / "configs",
              Path(__file__).resolve().parent / "vdsim_configs"):
        if c.is_dir():
            return c
    return REPO / "configs"


_CONF = _conf_root()
_ROAD = _CONF / "roads"
_MAP = _CONF / "maps"
_SENS = _CONF / "sensors"
_EXP = _CONF / "experiments"
_ISO = {"A": 0, "B": 1, "C": 2, "D": 3, "E": 4, "F": 5, "G": 6, "H": 7}


def resolve_line(dl):
    """A map's driving_line spec -> [[x, y], ...] (shape / waypoints / xodr / rd5).
    Shared by the authoring tool and the runner so they agree."""
    src = dl.get("source", "shape")
    if src == "waypoints":
        return [[float(p[0]), float(p[1])] for p in dl.get("points", [])]
    if src == "shape":
        kind = dl.get("shape", "oval")
        R = float(dl.get("R", 50.0)); n = int(dl.get("n", 160))
        if kind == "circle":
            return [[R*math.cos(2*math.pi*i/n), R*math.sin(2*math.pi*i/n)] for i in range(n)]
        if kind == "figure8":
            return [[R + R*math.sin(2*math.pi*i/n)*math.cos(2*math.pi*i/n)*2,
                     R*math.sin(2*math.pi*i/n)] for i in range(n)]
        L = float(dl.get("L", 150.0)); q = max(1, n // 4); pts = []
        for i in range(q): pts.append([L*i/q - L/2, -R])
        for i in range(q): a = -math.pi/2 + math.pi*i/q; pts.append([L/2 + R*math.cos(a), R*math.sin(a)])
        for i in range(q): pts.append([L/2 - L*i/q, R])
        for i in range(q): a = math.pi/2 + math.pi*i/q; pts.append([-L/2 + R*math.cos(a), R*math.sin(a)])
        return pts
    if src == "xodr":
        import opendrive as od
        roads = od.parse_xodr(dl["path"])
        try:
            return [list(p) for p in od.route_by_links(roads, od.parse_junctions(dl["path"]))]
        except Exception:
            return [list(p) for p in od.chain_route(roads)]
    if src == "rd5":
        import rd5_route as rr
        return [list(p) for p in rr.route_polyline(dl["path"])]
    return []


# --------------------------------------------------------------------------- #
# Vehicle / Tire
# --------------------------------------------------------------------------- #
def _catalog_root():
    pkg = Path(__file__).resolve().parent
    if (pkg / "vdsim_configs" / "catalog" / "manifest.yaml").is_file():
        return pkg
    for c in (REPO, REPO.parent if (REPO / "configs").is_dir() else None):
        if c and (c / "configs" / "catalog" / "manifest.yaml").is_file():
            return c
    return REPO


def _resolve_preset(vehicle="sedan", tire="default_pacejka"):
    root = _catalog_root()
    pkg = Path(__file__).resolve().parent
    for extra in (root / "python", pkg):
        sp = str(extra)
        if extra.is_dir() and sp not in sys.path:
            sys.path.insert(0, sp)
    from catalog import CatalogResolver
    from catalog.ids import blueprint_for_vehicle, tire_id_from_stem
    cache = _CONF / ".resolve_cache" / f"{vehicle}_{tire}"
    cache.mkdir(parents=True, exist_ok=True)
    r = CatalogResolver(_catalog_root())
    rv = r.resolve_blueprint(
        blueprint_for_vehicle(vehicle),
        instance_parts={"tire": tire_id_from_stem(tire)},
        out_dir=cache,
    )
    return rv.vehicle_yaml, rv.tire_yaml


class Vehicle:
    def __init__(self, vp):
        self.vp = vp

    @classmethod
    def preset(cls, name="sedan"):
        vp_path, _ = _resolve_preset(name)
        return cls(vdsim.VehicleParams.from_yaml(str(vp_path)))

    @classmethod
    def from_yaml(cls, path):
        return cls(vdsim.VehicleParams.from_yaml(str(path)))

    def set(self, **kw):
        for k, v in kw.items():
            setattr(self.vp, k, v)
        return self


class Tire:
    def __init__(self, tp):
        self.tp = tp

    @classmethod
    def preset(cls, name="default_pacejka"):
        _, tp_path = _resolve_preset("sedan", name)
        return cls(vdsim.TireParams.from_yaml(str(tp_path)))

    @classmethod
    def from_yaml(cls, path):
        p = Path(path)
        tp = vdsim.TireParams.from_yaml(str(p))
        if tp.tir_path and not Path(tp.tir_path).is_absolute():
            tp.tir_path = str((p.parent / tp.tir_path).resolve())
        return cls(tp)

    def lugre(self, enabled=True, **kw):
        self.tp.lugre.enabled = bool(enabled)
        for k, v in kw.items():
            setattr(self.tp.lugre, k, v)
        return self


# --------------------------------------------------------------------------- #
# Sensors
# --------------------------------------------------------------------------- #
class Sensors:
    def __init__(self, seed=1):
        self.sp = vdsim.SensorParams()
        self.sp.enabled = True
        self.sp.seed = seed

    @staticmethod
    def _set(noise, std, bias, rw):
        noise.noise_std = std
        if bias is not None:
            noise.bias = bias
        if rw is not None:
            noise.bias_rw = rw

    def imu(self, accel_std=0.05, gyro_std=0.002, accel_bias=None, gyro_bias=None, rw=None):
        self._set(self.sp.imu_accel, accel_std, accel_bias, rw)
        self._set(self.sp.imu_gyro, gyro_std, gyro_bias, rw)
        return self

    def gnss(self, pos_std=0.3, vel_std=0.05, pos_bias=None):
        self._set(self.sp.gnss_pos, pos_std, pos_bias, None)
        self._set(self.sp.gnss_vel, vel_std, None, None)
        return self

    def wheel_speed(self, std=0.05):
        self._set(self.sp.wheel_speed, std, None, None)
        return self

    def steer(self, std=0.001):
        self._set(self.sp.steer, std, None, None)
        return self

    def disabled(self):
        self.sp.enabled = False
        return self

    @classmethod
    def from_suite(cls, suite):
        """Map an authored sensor suite (list of {type, mount, noise_std, ...}) onto
        the sim's SensorParams. gnss/imu/wheel/steer drive the in-sim noise; camera/
        lidar mounts are kept for the (future) render coupling, not the dynamics."""
        s = cls()
        for sen in suite.get("sensors", []):
            t, std = sen.get("type"), float(sen.get("noise_std", 0.0))
            if t == "gnss":
                s.gnss(pos_std=std)
            elif t == "imu":
                s.imu(accel_std=std or 0.05)
            elif t == "wheel_speed":
                s.wheel_speed(std=std or 0.05)
            elif t == "steer":
                s.steer(std=std or 0.001)
        return s


# --------------------------------------------------------------------------- #
# Road — a spec the Experiment turns into the right make_sim_session* call
# --------------------------------------------------------------------------- #
class Road:
    def __init__(self, kind, **p):
        self.kind, self.p = kind, p

    @classmethod
    def flat(cls, mu=1.0):
        return cls("flat", mu=mu)

    @classmethod
    def inclined(cls, grade=0.0, bank=0.0, mu=1.0):
        return cls("inclined", grade=grade, bank=bank, mu=mu)

    @classmethod
    def split_mu(cls, mu_left=1.0, mu_right=0.5, boundary_y=0.0):
        return cls("split", mu=mu_left, mu_right=mu_right, mu_boundary_y=boundary_y)

    @classmethod
    def iso8608(cls, road_class="C", mu=1.0):
        return cls("iso", iso_class=_ISO.get(str(road_class).upper(), 2), mu=mu)

    @classmethod
    def psd(cls, gd_n0, waviness=2.0, n_break=0.0, waviness_high=2.0, n_max=4.0, mu=1.0):
        return cls("psd", gd_n0=gd_n0, waviness=waviness, n_break=n_break,
                   waviness_high=waviness_high, n_max=n_max, mu=mu)

    @classmethod
    def preset(cls, name):
        import yaml
        cfg = yaml.safe_load(open(_ROAD / f"{name}.yaml"))
        psd = cfg["psd"]
        mu = float(cfg.get("mu", 1.0))
        if psd.get("type") == "table":
            return cls("psd_table", n=[float(v) for v in psd["n"]],
                       gd=[float(v) for v in psd["gd"]],
                       n_max=float(psd.get("n_max", 10.0)), mu=mu)
        return cls("psd", gd_n0=float(psd["gd_n0"]), waviness=float(psd.get("waviness", 2.0)),
                   n_break=float(psd.get("n_break", 0.0)),
                   waviness_high=float(psd.get("waviness_high", 2.0)),
                   n_max=float(psd.get("n_max", 4.0)), mu=mu)

    def _session(self, vp, tp, level, dt, sensors):
        k, p = self.kind, self.p
        if k == "iso":
            return vdsim.make_sim_session(vp, tp, level, nominal_dt=dt,
                                          sensors=sensors, mu=p["mu"], iso_class=p["iso_class"])
        if k == "psd":
            return vdsim.make_sim_session_psd(vp, tp, level, mu=p["mu"], gd_n0=p["gd_n0"],
                                              waviness=p["waviness"], n_break=p["n_break"],
                                              waviness_high=p["waviness_high"],
                                              n_max=p["n_max"], nominal_dt=dt)
        if k == "psd_table":
            return vdsim.make_sim_session_psd(vp, tp, level, mu=p["mu"], n=p["n"], gd=p["gd"],
                                              n_max=p["n_max"], nominal_dt=dt)
        # flat / inclined / split share make_sim_session
        kw = dict(nominal_dt=dt, sensors=sensors, mu=p.get("mu", 1.0))
        if k == "inclined":
            kw.update(grade=p["grade"], bank=p["bank"])
        elif k == "split":
            kw.update(mu_right=p["mu_right"], mu_boundary_y=p["mu_boundary_y"])
        return vdsim.make_sim_session(vp, tp, level, **kw)


# --------------------------------------------------------------------------- #
# Maneuver — produces a driver(k, out, vp) -> CmdL4
# --------------------------------------------------------------------------- #
def _throttle_hold(vx, v_target):
    return max(0.0, min(1.0, (v_target - vx) / 3.0 + 0.05))


class Maneuver:
    def __init__(self, driver, init_v=0.0, init_yaw=0.0):
        self.driver = driver
        self.init_v, self.init_yaw = init_v, init_yaw

    @classmethod
    def constant_speed(cls, v=20.0):
        def d(k, o, vp):
            c = vdsim.CmdL4(); c.throttle = _throttle_hold(o.state.vx(), v); return c
        return cls(d, init_v=v)

    @classmethod
    def accel(cls, throttle=1.0):
        def d(k, o, vp):
            c = vdsim.CmdL4(); c.throttle = throttle; return c
        return cls(d, init_v=0.0)

    @classmethod
    def brake(cls, brake=0.8, init_v=25.0):
        def d(k, o, vp):
            c = vdsim.CmdL4(); c.brake = brake; return c
        return cls(d, init_v=init_v)

    @classmethod
    def step_steer(cls, v=20.0, steer=0.03, t0=2.0, dt=0.005):
        n0 = int(t0 / dt)
        def d(k, o, vp):
            c = vdsim.CmdL4(); c.throttle = _throttle_hold(o.state.vx(), v)
            c.steer_angle_wheel = steer if k >= n0 else 0.0
            return c
        return cls(d, init_v=v)

    @classmethod
    def path(cls, waypoints, v=15.0, lookahead=8.0):
        """Pure-pursuit over [(x,y), ...] (autonomous driving line); loops."""
        pts = list(waypoints)
        def d(k, o, vp):
            s = o.state; px, py, yaw = s.position[0], s.position[1], s.yaw()
            # nearest anchor + lookahead
            i0 = min(range(len(pts)), key=lambda i: (pts[i][0]-px)**2 + (pts[i][1]-py)**2)
            j, acc = i0, 0.0
            while acc < lookahead:
                nj = (j + 1) % len(pts)
                acc += math.hypot(pts[nj][0]-pts[j][0], pts[nj][1]-pts[j][1]); j = nj
                if j == i0: break
            tx, ty = pts[j]
            dx = math.cos(-yaw)*(tx-px) - math.sin(-yaw)*(ty-py)
            dy = math.sin(-yaw)*(tx-px) + math.cos(-yaw)*(ty-py)
            ld = max(1.0, math.hypot(dx, dy))
            c = vdsim.CmdL4(); c.throttle = _throttle_hold(s.vx(), v)
            c.steer_angle_wheel = max(-vp.max_steer_angle_wheel,
                                      min(vp.max_steer_angle_wheel,
                                          math.atan2(2.0*vp.wheelbase*dy, ld*ld)))
            return c
        yaw0 = math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0]) if len(pts) > 1 else 0.0
        m = cls(d, init_v=v, init_yaw=yaw0)
        m.start = pts[0]
        return m


# --------------------------------------------------------------------------- #
# Experiment + Result
# --------------------------------------------------------------------------- #
_COLS = ["t", "x", "y", "yaw", "vx", "vy", "r", "ax", "ay", "steer",
         "Fz0", "Fz1", "Fz2", "Fz3", "Fx0", "Fx1", "Fx2", "Fx3",
         "Fy0", "Fy1", "Fy2", "Fy3", "a0", "a1", "a2", "a3",
         "k0", "k1", "k2", "k3"]


def _make_row(o):
    st, Ft = o.state, o.tire_forces
    return [o.sim_time, st.position[0], st.position[1], st.yaw(),
            st.vx(), st.vy(), st.yaw_rate(), o.ax, o.ay, o.steer_applied,
            *o.Fz, *[Ft[i][0] for i in range(4)], *[Ft[i][1] for i in range(4)],
            *o.slip_angle, *o.slip_ratio]


class Result:
    def __init__(self, rows):
        self.rows = rows

    def col(self, name):
        i = _COLS.index(name)
        return [r[i] for r in self.rows]

    def to_csv(self, path):
        with open(path, "w") as f:
            f.write(",".join(_COLS) + "\n")
            for r in self.rows:
                f.write(",".join(f"{v:.6g}" for v in r) + "\n")
        return path

    def summary(self):
        vx = self.col("vx"); ay = self.col("ay"); r = self.col("r")
        return (f"{len(self.rows)} steps, t={self.rows[-1][0]:.1f}s | "
                f"vx {min(vx):.1f}..{max(vx):.1f} | |ay|max {max(abs(v) for v in ay):.2f} | "
                f"|r|max {max(abs(v) for v in r):.3f}")


# --------------------------------------------------------------------------- #
# Metrics — scalar reductions on a Result. Map-path metrics (CTE / heading) use
# the scenario's reference driving line; others are pure trajectory reductions.
# --------------------------------------------------------------------------- #
def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _cte_series(res, line):
    if not line or len(line) < 2:
        return []
    xs, ys = res.col("x"), res.col("y")
    return [min(_seg_dist(px, py, line[i][0], line[i][1], line[i+1][0], line[i+1][1])
                for i in range(len(line) - 1)) for px, py in zip(xs, ys)]


def m_cte_rms(res, line=None, **_):
    c = _cte_series(res, line)
    return (sum(v * v for v in c) / len(c)) ** 0.5 if c else float("nan")


def m_cte_max(res, line=None, **_):
    c = _cte_series(res, line)
    return max(c) if c else float("nan")


def m_peak_ay(res, **_):
    return max(abs(v) for v in res.col("ay"))


def m_max_fz(res, **_):
    return max(max(res.col("Fz%d" % i)) for i in range(4))


def m_vmax(res, **_):
    return max(res.col("vx"))


def m_dist(res, **_):
    xs, ys = res.col("x"), res.col("y")
    return sum(math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1]) for i in range(1, len(xs)))


def m_lap_time(res, **_):
    xs, ys, t = res.col("x"), res.col("y"), res.col("t")
    x0, y0, left = xs[0], ys[0], False
    for i in range(len(xs)):
        d = math.hypot(xs[i] - x0, ys[i] - y0)
        if d > 15: left = True
        elif left and d < 8: return t[i]
    return float("nan")


def m_rms_slip(res, **_):
    n = len(res.rows)
    a = [max(abs(res.col("a%d" % i)[k]) for i in range(4)) for k in range(n)]
    return (sum(v * v for v in a) / n) ** 0.5 if n else 0.0


METRICS = {"cte_rms": m_cte_rms, "cte_max": m_cte_max, "peak_ay": m_peak_ay,
           "max_Fz": m_max_fz, "vmax": m_vmax, "dist": m_dist,
           "lap_time": m_lap_time, "rms_slip": m_rms_slip}


def register_metric(name, fn):
    """Register a custom scalar metric: fn(res, **kw) -> float.
    After registration it is available by name in sim.metrics([...]) /
    compute_metrics(). fn receives a Result and keyword args (line=, ...).

        from vdsim_lab import register_metric
        def viol_count(res, threshold=0.4, **_):
            return sum(1 for v in res.col("ay") if abs(v) > threshold)
        register_metric("viol_count", viol_count)
    """
    METRICS[name] = fn


def compute_metrics(res, names, line=None):
    out = {}
    for n in names:
        fn = METRICS.get(n)
        try:
            out[n] = fn(res, line=line) if fn else float("nan")
        except Exception:
            out[n] = float("nan")
    return out


class Experiment:
    def __init__(self, level="L2", dt=0.005):
        self.level, self.dt = level, dt
        self._veh = Vehicle.preset("sedan")
        self._tire = Tire.preset()
        self._road = Road.flat()
        self._man = Maneuver.constant_speed(15.0)
        self._sensors = None

    def vehicle(self, v): self._veh = v; return self
    def tire(self, t): self._tire = t; return self
    def road(self, r): self._road = r; return self
    def maneuver(self, m): self._man = m; return self
    def sensors(self, s): self._sensors = s; return self

    @classmethod
    def from_config(cls, name_or_cfg):
        """Build an Experiment from an authored scenario YAML (configs/experiments/
        <name>.yaml or a dict): vehicle + tire + level + map(surface+driving line)
        + maneuver + sensor suite. Closes the loop with the authoring tool."""
        import yaml
        cfg = name_or_cfg
        if isinstance(cfg, str):
            cfg = yaml.safe_load(open(_EXP / f"{cfg}.yaml"))
        exp = cls(level=cfg.get("level", "L2"))
        exp.vehicle(Vehicle.preset(cfg.get("vehicle", "sedan")))
        exp.tire(Tire.preset(cfg.get("tire", "default_pacejka")))
        line = None
        if cfg.get("map"):
            m = yaml.safe_load(open(_MAP / f"{cfg['map']}.yaml"))
            line = resolve_line(m.get("driving_line", {}))
            surf = (m.get("road") or {}).get("surface")
            _ref = surf.get("ref") if surf else None
            if not _ref or _ref == "flat":
                exp.road(Road.flat(mu=surf.get("mu", 1.0) if surf else 1.0))
            elif _ref.startswith("iso8608"):
                _cls = surf.get("class", "C")
                exp.road(Road.iso8608(_cls, mu=surf.get("mu", 1.0)))
            else:
                exp.road(Road.preset(_ref))
        mc = cfg.get("maneuver", {})
        t, v = mc.get("type", "constant_speed"), float(mc.get("v", 20.0))
        if t == "path" and line:
            exp.maneuver(Maneuver.path(line, v=v))
        elif t == "step_steer":
            exp.maneuver(Maneuver.step_steer(v=v, steer=float(mc.get("steer", 0.03))))
        elif t == "accel":
            exp.maneuver(Maneuver.accel())
        elif t == "brake":
            exp.maneuver(Maneuver.brake())
        else:
            exp.maneuver(Maneuver.constant_speed(v))
        if cfg.get("sensors"):
            suite = yaml.safe_load(open(_SENS / f"{cfg['sensors']}.yaml"))
            exp.sensors(Sensors.from_suite(suite))
            exp._suite = suite
        exp._duration = float(cfg.get("duration", 10.0))
        exp._run = cfg.get("run", {"mode": "api"})
        exp._line = line                      # map reference line (for CTE/lap metrics)
        for k, v in (cfg.get("_overrides") or {}).items():   # batch sweep/MC overrides
            if k.startswith("vehicle."):
                setattr(exp._veh.vp, k.split(".", 1)[1], v)
            elif k.startswith("tire."):
                setattr(exp._tire.tp, k.split(".", 1)[1], v)
            elif k == "mu":
                exp._road.p["mu"] = v
        return exp

    def run(self, duration=None):
        if duration is None:
            duration = getattr(self, "_duration", 10.0)
        vp, tp = self._veh.vp, self._tire.tp
        sp = self._sensors.sp if self._sensors else vdsim.SensorParams()
        sess = self._road._session(vp, tp, self.level, self.dt, sp)
        x0, y0 = getattr(self._man, "start", (0.0, 0.0))
        s0 = vdsim.make_init_state(vp, tp, x=x0, y=y0, yaw=self._man.init_yaw,
                                   v=self._man.init_v)
        sess.reset(s0)
        rows, n = [], int(duration / self.dt)
        for k in range(n):
            o = sess.output()
            sess.set_input(self._man.driver(k, o, vp))
            sess.tick(self.dt)
            o = sess.output(); st = o.state
            Ft = o.tire_forces
            rows.append([o.sim_time, st.position[0], st.position[1], st.yaw(),
                         st.vx(), st.vy(), st.yaw_rate(), o.ax, o.ay, o.steer_applied,
                         *o.Fz, *[Ft[i][0] for i in range(4)], *[Ft[i][1] for i in range(4)],
                         *o.slip_angle, *o.slip_ratio])
        return Result(rows)


class Simulation:
    """Embedded synchronous API over an authored scenario (run mode "api"): the
    sim is a plain object you step from your own code, no network.

        sim = Simulation("yongin_lap")          # name / cfg dict / Experiment
        while not sim.done():
            sim.set_control(steer=.., throttle=.., brake=..)   # omit -> scenario autopilot
            sim.step()
            gnss = sim.get_data("gnss")          # per-sensor measurement
            gt   = sim.state()                   # ground truth
    """
    def __init__(self, scenario, dt=None, duration=None):
        exp = scenario if isinstance(scenario, Experiment) else Experiment.from_config(scenario)
        self.exp = exp
        self._vp = exp._veh.vp
        sp = exp._sensors.sp if exp._sensors else vdsim.SensorParams()
        self.dt = dt or exp.dt
        self.duration = duration if duration is not None else getattr(exp, "_duration", 1e18)
        self.sess = exp._road._session(self._vp, exp._tire.tp, exp.level, self.dt, sp)
        man = exp._man
        x0, y0 = getattr(man, "start", (0.0, 0.0))
        self.sess.reset(vdsim.make_init_state(self._vp, exp._tire.tp,
                                              x=x0, y=y0, yaw=man.init_yaw, v=man.init_v))
        self._k = 0
        self._ext = None
        self._prev_r = 0.0
        self._alpha = 0.0                      # yaw acceleration (for IMU lever arm)
        self._rows = []                        # time-series log for metrics/csv/plot
        suite = getattr(exp, "_suite", {}).get("sensors", [])
        self._types = {s["id"]: s.get("type") for s in suite}
        self._mounts = {s["id"]: {"pos": s.get("mount", [0, 0, 0]),
                                  "yaw": math.radians(s.get("yaw", 0.0))} for s in suite}

    def set_control(self, steer=0.0, throttle=0.0, brake=0.0):
        c = vdsim.CmdL4(); c.steer_angle_wheel = steer; c.throttle = throttle; c.brake = brake
        self._ext = c

    # seam aliases (consistent with Sim): inject an action / advance one core step
    def set_input(self, cmd=None, *, steer=0.0, throttle=0.0, brake=0.0):
        if cmd is not None:
            self._ext = cmd                       # a vdsim.CmdL4
        else:
            self.set_control(steer=steer, throttle=throttle, brake=brake)
        return self

    def run_core_dt(self, dt=None):
        return self.step(dt)

    def step(self, dt=None):
        dt = dt or self.dt
        cmd = self._ext if self._ext is not None else self.exp._man.driver(self._k, self.sess.output(), self._vp)
        self._ext = None
        self.sess.set_input(cmd); self.sess.tick(dt); self._k += 1
        o = self.sess.output(); s = o.state
        r = s.yaw_rate()
        self._alpha = (r - self._prev_r) / dt if dt > 0 else 0.0
        self._prev_r = r
        self._rows.append(_make_row(o))        # log for metrics/csv/plot
        return o

    def output(self):
        return self.sess.output()

    def result(self):
        return Result(self._rows)

    def metrics(self, names=None, line=None):
        names = names or ["peak_ay", "vmax", "dist"]
        _line = line or getattr(self.exp, "_line", None)
        return compute_metrics(Result(self._rows), names, _line)

    def to_csv(self, path):
        Result(self._rows).to_csv(path)

    def plot(self, path=None, signals=("vx", "ay", "r"), title=None):
        plot_result(Result(self._rows), path, signals, title)

    def state(self):
        o = self.sess.output(); s = o.state
        return {"t": o.sim_time, "x": s.position[0], "y": s.position[1], "yaw": s.yaw(),
                "vx": s.vx(), "vy": s.vy(), "r": s.yaw_rate(), "ax": o.ax, "ay": o.ay,
                "Fz": list(o.Fz), "slip_angle": list(o.slip_angle), "slip_ratio": list(o.slip_ratio)}

    def get_data(self, sensor_id=None):
        """Per-sensor measurement. With a sensor_id whose suite entry has a mount
        pose, the CG measurement is transported to the mount: GNSS position by the
        world lever arm p+Rz(yaw)*mount, GNSS velocity and IMU accel by the body
        lever arm (centripetal -r^2*mount + tangential alpha x mount), IMU rotated
        into the sensor frame. Gyro/wheel/steer are mount-invariant."""
        o = self.sess.output(); m = o.sensors; s = o.state
        cg = {"gnss": {"x": m.gnss_x, "y": m.gnss_y, "vx": m.gnss_vx, "vy": m.gnss_vy},
              "imu": {"ax": m.ax, "ay": m.ay, "wz": m.wz},
              "wheel_speed": {"w": list(m.wheel_speed)},
              "steer": {"angle": m.steer}}
        if sensor_id is None:
            return cg
        t = self._types.get(sensor_id, sensor_id)
        mnt = self._mounts.get(sensor_id, {"pos": [0, 0, 0], "yaw": 0.0})
        mx, my = float(mnt["pos"][0]), float(mnt["pos"][1])
        r, a = s.yaw_rate(), self._alpha
        if t == "gnss":
            yaw = s.yaw(); c, sn = math.cos(yaw), math.sin(yaw)
            rwx, rwy = c * mx - sn * my, sn * mx + c * my          # mount in world
            return {"x": m.gnss_x + rwx, "y": m.gnss_y + rwy,
                    "vx": m.gnss_vx - r * my, "vy": m.gnss_vy + r * mx}  # +omega x r (body)
        if t == "imu":
            ax = m.ax - r * r * mx - a * my                         # centripetal + tangential
            ay = m.ay - r * r * my + a * mx
            cm, sm = math.cos(mnt["yaw"]), math.sin(mnt["yaw"])
            return {"ax": cm * ax + sm * ay, "ay": -sm * ax + cm * ay, "wz": m.wz}
        return cg.get(t, {})

    def done(self):
        return self.sess.output().sim_time >= self.duration

    def time(self):
        return self.sess.output().sim_time


# --------------------------------------------------------------------------- #
# Sim — direct core-driven harness for algorithm evaluation.
#
# The evaluator owns the loop and the controller; VDSim provides the seam:
#   set_input(action) -> run_core_dt() -> read state()/measurements().
# Build straight from the core (vehicle/tire/level/road), no scenario file needed.
# --------------------------------------------------------------------------- #
def _resolve_ref_point(ref, vp):
    """Resolve ref_point to [rx, ry] in body frame [m] (ISO 8855: X fwd, Y left).
    None / "cg" / [0,0] -> no transform. "rear_axle" -> [-b, 0]. "front_axle" -> [a, 0].
    """
    if ref is None or ref == "cg":
        return None
    if ref == "rear_axle":
        return [-vp.cg_to_rear, 0.0]
    if ref == "front_axle":
        return [vp.cg_to_front, 0.0]
    r = list(ref)
    if len(r) == 2 and r[0] == 0.0 and r[1] == 0.0:
        return None
    return [float(r[0]), float(r[1])]


def _as_vehicle(v):
    if isinstance(v, Vehicle):
        return v
    if isinstance(v, str) and v.endswith((".yaml", ".yml")):
        return Vehicle.from_yaml(v)
    stem = Path(v or "sedan").stem
    bundled = _CONF / "vehicles" / f"{stem}.yaml"
    if bundled.is_file():
        return Vehicle.from_yaml(bundled)
    return Vehicle.preset(stem)


def _as_tire(t):
    if isinstance(t, Tire):
        return t
    if isinstance(t, str) and t.endswith((".yaml", ".yml")):
        return Tire.from_yaml(t)
    stem = Path(t or "default_pacejka").stem
    bundled = _CONF / "parts" / "tire" / f"{stem}.yaml"
    if bundled.is_file():
        return Tire.from_yaml(bundled)
    return Tire.preset(stem)


class Sim:
    """Core-driven simulation for algorithm evaluation. You write the controller
    and own the loop; the sim exposes the action seam set_input() and the step
    seam run_core_dt().

        from vdsim_lab import Sim, Road, Sensors
        sim = Sim(vehicle="sedan", level="L2", road=Road.iso8608("C"),
                  sensors=Sensors().gnss().imu(), v0=15.0,
                  sensor_mounts={"gnss": {"type": "gnss", "pos": [1.4, 0, 1.0]}})
        while not sim.done(20.0):
            st = sim.state()                       # ground truth (dict)
            gn = sim.measurements("gnss")          # noisy measurement at the mount
            steer, throttle, brake = my_controller(st, gn)
            sim.set_input(steer=steer, throttle=throttle, brake=brake)
            sim.run_core_dt()                      # advance one core step (dt)
        sim.to_csv("run.csv")                      # ground-truth + per-wheel log
        sim.plot("run.png", signals=("vx", "ay", "r"))   # optional (matplotlib)

    set_input also accepts a vdsim.CmdL4 directly: sim.set_input(cmd).
    """

    def __init__(self, vehicle="sedan", tire="default_pacejka", level="L2",
                 road=None, sensors=None, dt=0.005,
                 x0=0.0, y0=0.0, yaw0=0.0, v0=0.0, sensor_mounts=None,
                 ref_point=None):
        self._veh = _as_vehicle(vehicle)
        self._tire = _as_tire(tire)
        self._road = road or Road.flat()
        self.level, self.dt = level, dt
        vp, tp = self._veh.vp, self._tire.tp
        if isinstance(sensors, Sensors):
            sp = sensors.sp
        elif sensors is None:
            sp = vdsim.SensorParams()
        else:
            sp = sensors                           # raw vdsim.SensorParams
        self.sess = self._road._session(vp, tp, level, dt, sp)
        self.sess.reset(vdsim.make_init_state(vp, tp, x=x0, y=y0, yaw=yaw0, v=v0))
        self._vp = vp
        self._tp = tp
        self._cmd = vdsim.CmdL4()
        self._k = 0
        self._prev_r = 0.0
        self._alpha = 0.0                          # yaw accel (IMU lever arm)
        self.rows = []
        self._extras = []                          # per-step controller side-state (log_extra)
        self._x0 = x0; self._y0 = y0; self._yaw0 = yaw0; self._v0 = v0
        self._ref = _resolve_ref_point(ref_point, vp)   # [rx, ry] body frame or None (=CG)
        mounts = sensor_mounts or {}
        self._types = {sid: m.get("type", sid) for sid, m in mounts.items()}
        self._mounts = {sid: {"pos": m.get("pos", [0, 0, 0]),
                              "yaw": math.radians(m.get("yaw", 0.0))}
                        for sid, m in mounts.items()}

    # ---- action injection seam ----
    def set_input(self, cmd=None, *, steer=0.0, throttle=0.0, brake=0.0, gear=1):
        if cmd is not None:                        # a vdsim.CmdL4
            self._cmd = cmd
            return self
        c = vdsim.CmdL4()
        c.steer_angle_wheel = steer
        c.throttle = throttle
        c.brake = brake
        c.gear = gear
        self._cmd = c
        return self

    # ---- core stepping seam ----
    def run_core_dt(self, dt=None):
        dt = dt or self.dt
        self.sess.set_input(self._cmd)
        self.sess.tick(dt)
        self._k += 1
        o = self.sess.output()
        r = o.state.yaw_rate()
        self._alpha = (r - self._prev_r) / dt if dt > 0 else 0.0
        self._prev_r = r
        self._record(o)
        self._extras.append({})                    # placeholder; log_extra() fills this
        return o

    step = run_core_dt                             # alias

    def _record(self, o):
        self.rows.append(_make_row(o))

    # ---- readouts ----
    def output(self):
        return self.sess.output()

    def time(self):
        return self.sess.output().sim_time

    def done(self, duration):
        return self.sess.output().sim_time >= duration

    def state(self):
        o = self.sess.output(); s = o.state
        vx, vy = s.vx(), s.vy()
        r_yaw = s.yaw_rate()
        yaw = s.yaw()
        x, y = s.position[0], s.position[1]
        ax, ay = o.ax, o.ay

        if self._ref is not None:
            rx, ry = self._ref[0], self._ref[1]
            # position: p_ref = p_cg + R_yaw * [rx, ry]
            c, sn = math.cos(yaw), math.sin(yaw)
            x  = x  + c * rx - sn * ry
            y  = y  + sn * rx + c * ry
            # velocity (body frame): v_ref = v_cg + omega_z x r
            vx = vx - r_yaw * ry
            vy = vy + r_yaw * rx
            # acceleration (body frame): a_ref = a_cg + alpha x r - omega^2 * r
            alpha = self._alpha                    # yaw acceleration [rad/s²]
            ax = ax - alpha * ry - r_yaw * r_yaw * rx
            ay = ay + alpha * rx - r_yaw * r_yaw * ry

        beta = math.atan2(vy, vx) if abs(vx) > 1e-3 else 0.0
        return {"t": o.sim_time, "x": x, "y": y, "yaw": yaw,
                "vx": vx, "vy": vy, "r": r_yaw, "ax": ax, "ay": ay,
                "beta": beta,                      # sideslip at the reference point [rad]
                "Fz": list(o.Fz), "slip_angle": list(o.slip_angle),
                "slip_ratio": list(o.slip_ratio)}

    def measurements(self, sensor_id=None):
        """Noisy sensor readout. With a sensor_id that has a registered mount
        pose, the CG measurement is transported to the mount (GNSS world lever
        arm; GNSS vel + IMU accel body lever arm; IMU rotated into sensor frame).
        Without a mount, returns the CG measurement bundle."""
        o = self.sess.output(); m = o.sensors; s = o.state
        cg = {"gnss": {"x": m.gnss_x, "y": m.gnss_y, "vx": m.gnss_vx, "vy": m.gnss_vy},
              "imu": {"ax": m.ax, "ay": m.ay, "wz": m.wz},
              "wheel_speed": {"w": list(m.wheel_speed)},
              "steer": {"angle": m.steer}}
        if sensor_id is None:
            return cg
        t = self._types.get(sensor_id, sensor_id)
        mnt = self._mounts.get(sensor_id)
        if mnt is None:
            return cg.get(t, {})
        mx, my = float(mnt["pos"][0]), float(mnt["pos"][1])
        r, a = s.yaw_rate(), self._alpha
        if t == "gnss":
            yaw = s.yaw(); c, sn = math.cos(yaw), math.sin(yaw)
            rwx, rwy = c * mx - sn * my, sn * mx + c * my
            return {"x": m.gnss_x + rwx, "y": m.gnss_y + rwy,
                    "vx": m.gnss_vx - r * my, "vy": m.gnss_vy + r * mx}
        if t == "imu":
            ax = m.ax - r * r * mx - a * my
            ay = m.ay - r * r * my + a * mx
            cm, sm = math.cos(mnt["yaw"]), math.sin(mnt["yaw"])
            return {"ax": cm * ax + sm * ay, "ay": -sm * ax + cm * ay, "wz": m.wz}
        return cg.get(t, {})

    def log_extra(self, d):
        """Attach controller side-state to the most recent step so it lands in
        to_csv() alongside the ground-truth columns.

            sim.set_input(steer=steer, throttle=thr)
            sim.run_core_dt()
            sim.log_extra({"ax_cmd": ax_cmd, "w_norm": w_norm})
        """
        if self._extras:
            self._extras[-1].update(d)
        return self

    def reset(self, v0=None, x0=None, y0=None, yaw0=None):
        """Reset the plant in-place (same vehicle/tire/road/dt) so the same Sim
        object can be reused with a different controller without rebuilding.

            for ctrl in [ctrl_A, ctrl_B, ctrl_C]:
                sim.reset()
                while not sim.done(T): ...
        """
        x  = x0  if x0  is not None else self._x0
        y  = y0  if y0  is not None else self._y0
        yaw = yaw0 if yaw0 is not None else self._yaw0
        v  = v0  if v0  is not None else self._v0
        self.sess.reset(vdsim.make_init_state(self._vp, self._tp, x=x, y=y, yaw=yaw, v=v))
        self._cmd = vdsim.CmdL4()
        self._k = 0; self._prev_r = 0.0; self._alpha = 0.0
        self.rows = []; self._extras = []
        return self

    # ---- results / evidence ----
    def result(self):
        return Result(self.rows)

    def to_csv(self, path):
        """Write ground-truth + per-wheel log. If log_extra() was called during
        the run, the extra columns are appended after the standard ones (NaN for
        steps where log_extra was not called)."""
        extra_keys = list(dict.fromkeys(k for d in self._extras for k in d))
        if not extra_keys:
            return Result(self.rows).to_csv(path)
        import csv as _csv
        nan = float("nan")
        with open(path, "w", newline="") as f:
            w = _csv.writer(f)
            w.writerow(_COLS + extra_keys)
            for row, ex in zip(self.rows, self._extras):
                w.writerow([f"{v:.6g}" for v in row] +
                           [f"{ex.get(k, nan):.6g}" for k in extra_keys])
        return path

    def metrics(self, names, line=None):
        return compute_metrics(Result(self.rows), names, line)

    def plot(self, path=None, signals=("vx", "ay", "r"), title=None):
        return plot_result(Result(self.rows), path=path, signals=signals, title=title)


def plot_result(res, path=None, signals=("vx", "ay", "r"), title=None):
    """Basic evidence figure (optional; needs matplotlib). signals are _COLS
    names; the special signal "xy" draws the x-y trajectory. Saves to `path`
    (PNG) or returns the Figure. Labels are English (Korean fonts break)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:                          # pragma: no cover - optional dep
        raise RuntimeError("matplotlib is required for plotting (pip install matplotlib)") from e
    sigs = list(signals)
    t = res.col("t")
    fig, axes = plt.subplots(len(sigs), 1, figsize=(8, 2.3 * len(sigs)))
    if len(sigs) == 1:
        axes = [axes]
    for ax, sig in zip(axes, sigs):
        if sig == "xy":
            ax.plot(res.col("x"), res.col("y"))
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal", "datalim")
        else:
            ax.plot(t, res.col(sig)); ax.set_ylabel(sig); ax.set_xlabel("time [s]")
        ax.grid(True, alpha=0.3)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return path
    return fig


def plot_comparison(runs, path=None, signals=("vx", "ay", "r"), title=None):
    """Overlay multiple runs on the same axes for A/B/C ablation figures.

        runs: dict  {"A": sim_or_result, "B": ..., "C": ...}
              or list [sim_or_result, ...]   (labels = 0, 1, 2, ...)

        from vdsim_lab import plot_comparison
        plot_comparison({"baseline": sim_A, "ours": sim_B},
                        path="ablation.png", signals=("ay", "r", "xy"))
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:                          # pragma: no cover
        raise RuntimeError("matplotlib required for plotting") from e
    if isinstance(runs, (list, tuple)):
        runs = {str(i): r for i, r in enumerate(runs)}
    sigs = list(signals)
    fig, axes = plt.subplots(len(sigs), 1, figsize=(8, 2.3 * len(sigs)))
    if len(sigs) == 1:
        axes = [axes]
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for ax, sig in zip(axes, sigs):
        for ci, (label, run) in enumerate(runs.items()):
            res = run.result() if isinstance(run, Sim) else run
            c = colors[ci % len(colors)]
            if sig == "xy":
                ax.plot(res.col("x"), res.col("y"), label=label, color=c)
                ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal", "datalim")
            else:
                ax.plot(res.col("t"), res.col(sig), label=label, color=c)
                ax.set_ylabel(sig); ax.set_xlabel("time [s]")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return path
    return fig


def main():
    print("=== vdsim_lab demo ===")
    for road, man, lvl in [
        (Road.flat(), Maneuver.step_steer(v=20, steer=0.03), "L2"),
        (Road.iso8608("C"), Maneuver.constant_speed(20), "L3"),
        (Road.preset("belgian_pave"), Maneuver.constant_speed(15), "L3"),
        (Road.inclined(grade=math.radians(6)), Maneuver.constant_speed(15), "L2"),
    ]:
        res = (Experiment(level=lvl).road(road).maneuver(man)
               .sensors(Sensors().gnss().imu()).run(6.0))
        print(f"  {road.kind:9s} {lvl}: {res.summary()}")


if __name__ == "__main__":
    main()
