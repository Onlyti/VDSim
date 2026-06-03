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

_VEH = REPO / "configs" / "vehicles"
_TIRE = REPO / "configs" / "tires"
_ROAD = REPO / "configs" / "roads"
_ISO = {"A": 0, "B": 1, "C": 2, "D": 3, "E": 4, "F": 5, "G": 6, "H": 7}


# --------------------------------------------------------------------------- #
# Vehicle / Tire
# --------------------------------------------------------------------------- #
class Vehicle:
    def __init__(self, vp):
        self.vp = vp

    @classmethod
    def preset(cls, name="sedan"):
        return cls(vdsim.VehicleParams.from_yaml(str(_VEH / f"{name}.yaml")))

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
        return cls(vdsim.TireParams.from_yaml(str(_TIRE / f"{name}.yaml")))

    @classmethod
    def from_yaml(cls, path):
        return cls(vdsim.TireParams.from_yaml(str(path)))


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

    def run(self, duration=10.0):
        vp, tp = self._veh.vp, self._tire.tp
        sp = self._sensors.sp if self._sensors else vdsim.SensorParams()
        sess = self._road._session(vp, tp, self.level, self.dt, sp)
        x0, y0 = getattr(self._man, "start", (0.0, 0.0))
        s0 = vdsim.make_init_state(x=x0, y=y0, yaw=self._man.init_yaw,
                                   v=self._man.init_v, wheel_radius=vp.wheel_radius_nominal)
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
