#!/usr/bin/env python3
"""In-loop estimator testbench — run an observer against VDSim ground truth.

VDSim becomes an estimator bench: the plant runs with sensor noise/bias (the
SensorModel from task #124), the measurements are fed to an estimator each step,
and the estimate is scored against the true state (RMSE + NEES consistency).

Plug your own estimator (e.g. the TUR Fz/mu/Calpha UKF) by passing any object
with this duck-typed interface:

    est.reset(x0)                 # x0: initial state guess (array-like)
    x_hat, P = est.step(meas, cmd, dt)   # meas: vdsim.SensorMeas, cmd: vdsim.CmdL4
                                          # returns state estimate + covariance

NEES = e^T P^-1 e should sit near dim(x) (chi-square) if the filter is
consistent; the summary reports the mean and the fraction inside the 95% band.

Usage:
    cmake -DVDSIM_BUILD_PYTHON=ON -B build && cmake --build build -j
    python3 examples/estimator_in_loop.py
"""
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class KinematicPoseEKF:
    """Reference EKF over [X, Y, psi, v]: dead-reckon on IMU gyro + accel,
    correct with GNSS position. A self-contained demonstrator of the slot."""

    def __init__(self, gnss_std=0.1, q=(0.02, 0.02, 0.002, 0.2)):
        self.R = np.diag([gnss_std ** 2, gnss_std ** 2])
        self.Q = np.diag(q)
        self.reset([0, 0, 0, 0])

    def reset(self, x0):
        self.x = np.array(x0, float)
        self.P = np.diag([1.0, 1.0, 0.1, 1.0])

    def step(self, meas, cmd, dt):
        X, Y, psi, v = self.x
        r, a = meas.wz, meas.ax                      # gyro yaw-rate, long. accel
        # --- predict (kinematic dead reckoning) ---
        self.x = np.array([X + v * math.cos(psi) * dt,
                           Y + v * math.sin(psi) * dt,
                           psi + r * dt,
                           v + a * dt])
        F = np.array([[1, 0, -v * math.sin(psi) * dt, math.cos(psi) * dt],
                      [0, 1,  v * math.cos(psi) * dt, math.sin(psi) * dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        self.P = F @ self.P @ F.T + self.Q * dt
        # --- update (GNSS position) ---
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        y = np.array([meas.gnss_x, meas.gnss_y]) - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return self.x.copy(), self.P.copy()


def run_estimation(est, level="L2", duration=25.0, dt=0.01,
                   gnss_std=0.1, gyro_std=0.005, accel_std=0.05):
    vp = vdsim.VehicleParams.from_yaml(str(REPO / "configs/vehicles/sedan.yaml"))
    tp = vdsim.TireParams.from_yaml(str(REPO / "configs/tires/default_pacejka.yaml"))
    sp = vdsim.SensorParams()
    sp.enabled = True
    sp.gnss_pos.noise_std = gnss_std
    sp.imu_gyro.noise_std = gyro_std
    sp.imu_accel.noise_std = accel_std
    sess = vdsim.make_sim_session(vp, tp, level, nominal_dt=dt, sensors=sp)
    v0 = 10.0
    sess.reset(vdsim.make_init_state(0, 0, 0, v0, vp.wheel_radius_nominal))
    est.reset([0, 0, 0, v0])

    n = int(duration / dt)
    t = np.zeros(n)
    truth = np.zeros((n, 4))
    est_x = np.zeros((n, 4))
    nees = np.zeros(n)
    for k in range(n):
        tk = k * dt
        cmd = vdsim.CmdL4()
        cmd.throttle = 0.2
        cmd.steer_angle_wheel = 0.06 * math.sin(2 * math.pi * 0.15 * tk)
        sess.set_input(cmd)
        sess.tick(dt)
        o = sess.output()
        xt = np.array([o.state.position[0], o.state.position[1],
                       o.state.yaw(), o.state.vx()])
        xh, P = est.step(o.sensors, cmd, dt)
        e = xt - xh
        e[2] = wrap(e[2])
        t[k], truth[k], est_x[k] = tk, xt, xh
        nees[k] = float(e @ np.linalg.solve(P, e))
    return {"t": t, "truth": truth, "est": est_x, "nees": nees}


def main():
    res = run_estimation(KinematicPoseEKF(gnss_std=0.1))
    err = res["truth"] - res["est"]
    err[:, 2] = (err[:, 2] + math.pi) % (2 * math.pi) - math.pi
    pos_rmse = math.sqrt(np.mean(err[:, 0] ** 2 + err[:, 1] ** 2))
    yaw_rmse = math.sqrt(np.mean(err[:, 2] ** 2))
    v_rmse = math.sqrt(np.mean(err[:, 3] ** 2))
    # 95% chi-square band for dof = 4
    lo, hi = 0.484, 11.143
    inside = np.mean((res["nees"] >= lo) & (res["nees"] <= hi)) * 100.0
    print("=== estimator-in-loop (KinematicPoseEKF, 4-state) ===")
    print(f"  position RMSE : {pos_rmse:6.3f} m")
    print(f"  yaw RMSE      : {math.degrees(yaw_rmse):6.3f} deg")
    print(f"  speed RMSE    : {v_rmse:6.3f} m/s")
    print(f"  mean NEES     : {np.mean(res['nees']):6.2f}  (target ~4)")
    print(f"  NEES in 95% band [{lo}, {hi}] : {inside:5.1f} %")

    out = REPO / "logs"
    out.mkdir(exist_ok=True)
    p = out / "estimator_in_loop.csv"
    with open(p, "w") as f:
        f.write("t,Xtrue,Ytrue,psitrue,vtrue,Xest,Yest,psiest,vest,nees\n")
        for k in range(len(res["t"])):
            tr, es = res["truth"][k], res["est"][k]
            f.write("%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f\n" % (
                res["t"][k], tr[0], tr[1], tr[2], tr[3],
                es[0], es[1], es[2], es[3], res["nees"][k]))
    print(f"  log -> {p}")


if __name__ == "__main__":
    main()
