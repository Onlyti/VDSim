#!/usr/bin/env python3
"""Independent knuckle pitch-rate reference for work order 43 (J6a / R3).

Solves the linkage from the YAML hard points with numpy only; it does not
import vdsim or link any VDSim kinematics code.

Double wishbone: lower knuckle point LK rotates about the LCA chassis axis,
the upper point UK follows the UCA chassis axis with |UK - LK| fixed, the tie-rod
knuckle point TK is found by trilateration (|TK-LK|, |TK-UK|, |TK-rack| fixed).
The knuckle frame is built from LK, UK, TK; its rotation between wheel-centre
travel +dz and -dz gives omega / (2 dz) [rad/m] (body axes, ISO 8855).
Trailing arm: the knuckle is the arm, omega_y / dz = axis_y / (d z_wc / d theta).

Usage: l5_knuckle_reference.py <yaml> [<yaml> ...]
"""
import sys

import numpy as np
import yaml


def _v(n):
    return np.array([float(n[0]), float(n[1]), float(n[2])])


def _rot(axis, th, v):
    """Rodrigues rotation of v about the unit axis by th [rad]."""
    k = axis / np.linalg.norm(axis)
    return (v * np.cos(th) + np.cross(k, v) * np.sin(th)
            + k * np.dot(k, v) * (1.0 - np.cos(th)))


def _rotvec(R):
    """Rotation vector (axis * angle) of a 3x3 rotation matrix."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-12:
        return 0.5 * w
    return ang / (2.0 * np.sin(ang)) * w


class DoubleWishbone:
    def __init__(self, d):
        self.lcf, self.lcr, self.lk0 = (_v(d["lca"][k]) for k in ("chassis_front", "chassis_rear", "knuckle"))
        self.ucf, self.ucr, self.uk0 = (_v(d["uca"][k]) for k in ("chassis_front", "chassis_rear", "knuckle"))
        self.rack, self.tk0 = _v(d["tie_rod"]["rack"]), _v(d["tie_rod"]["knuckle"])
        self.wc0 = _v(d["wheel"]["center"])
        n = np.linalg.norm
        self.L_LU = n(self.uk0 - self.lk0)
        self.L_LT = n(self.tk0 - self.lk0)
        self.L_UT = n(self.tk0 - self.uk0)
        self.L_TR = n(self.tk0 - self.rack)

    def lk(self, t):
        return self.lcr + _rot(self.lcf - self.lcr, t, self.lk0 - self.lcr)

    def uk(self, l):
        a = self.ucf - self.ucr
        f = lambda t: np.linalg.norm(self.ucr + _rot(a, t, self.uk0 - self.ucr) - l) - self.L_LU
        t = 0.0
        for _ in range(60):
            r = f(t)
            if abs(r) < 1e-14:
                break
            t -= r / ((f(t + 1e-7) - r) / 1e-7)
        return self.ucr + _rot(a, t, self.uk0 - self.ucr)

    def tk(self, l, u):
        p = self.tk0 + (l - self.lk0)
        for _ in range(60):
            r = np.array([np.linalg.norm(p - l) - self.L_LT, np.linalg.norm(p - u) - self.L_UT,
                          np.linalg.norm(p - self.rack) - self.L_TR])
            if np.linalg.norm(r) < 1e-14:
                break
            J = np.vstack([(p - l) / np.linalg.norm(p - l), (p - u) / np.linalg.norm(p - u),
                           (p - self.rack) / np.linalg.norm(p - self.rack)])
            p = p - np.linalg.solve(J, r)
        return p

    @staticmethod
    def frame(l, u, t):
        e1 = (u - l) / np.linalg.norm(u - l)
        e2 = np.cross(e1, t - l)
        e2 /= np.linalg.norm(e2)
        return np.column_stack([e1, e2, np.cross(e1, e2)])

    def pose(self, th):
        l = self.lk(th)
        u = self.uk(l)
        t = self.tk(l, u)
        F = self.frame(l, u, t)
        R = F @ self.frame(self.lk0, self.uk0, self.tk0).T
        return l, u, F, l + R @ (self.wc0 - self.lk0)

    def theta_for_dz(self, dz):
        th = 0.0
        for _ in range(80):
            f = self.pose(th)[3][2] - self.wc0[2] - dz
            if abs(f) < 1e-14:
                break
            th -= f / ((self.pose(th + 1e-7)[3][2] - self.pose(th)[3][2]) / 1e-7)
        return th

    def rates(self, dz):
        lp, up, Fp, wp = self.pose(self.theta_for_dz(dz))
        lm, um, Fm, wm = self.pose(self.theta_for_dz(-dz))
        w = _rotvec(Fp @ Fm.T) / (2.0 * dz)
        side = lambda l, u: np.arctan2(u[0] - l[0], u[2] - l[2])
        return {"omega": w, "dxdz": (wp[0] - wm[0]) / (2.0 * dz),
                "side_angle_rate": (side(lp, up) - side(lm, um)) / (2.0 * dz)}


class TrailingArm:
    def __init__(self, d):
        self.pi = _v(d["arm_pivot"]["chassis_inboard"])
        self.po = _v(d["arm_pivot"]["chassis_outboard"])
        self.wc0 = _v(d["wheel"]["center"])

    def rates(self, dz):
        a = (self.po - self.pi) / np.linalg.norm(self.po - self.pi)
        # rigid arm: rotate about the pivot axis until the wheel centre moves +/- dz
        def wc(th):
            return self.pi + _rot(a, th, self.wc0 - self.pi)
        def th_for(z):
            th = 0.0
            for _ in range(80):
                f = wc(th)[2] - self.wc0[2] - z
                if abs(f) < 1e-15:
                    break
                th -= f / ((wc(th + 1e-7)[2] - wc(th)[2]) / 1e-7)
            return th
        tp, tm = th_for(dz), th_for(-dz)
        return {"omega": a * (tp - tm) / (2.0 * dz), "dxdz": (wc(tp)[0] - wc(tm)[0]) / (2.0 * dz),
                "side_angle_rate": float("nan")}


def model(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    return {"double_wishbone": DoubleWishbone, "trailing_arm": TrailingArm}[d["type"]](d)


def main(paths):
    for p in paths:
        m = model(p)
        for dz in (0.5e-3, 1.0e-3):
            r = m.rates(dz)
            print("[ref] %s dz %.1f mm | omega x %.6f y %.6f z %.6f rad/m | dx/dz %.6f | side-angle rate %.6f rad/m"
                  % (p.split("/")[-1], dz * 1e3, r["omega"][0], r["omega"][1], r["omega"][2],
                     r["dxdz"], r["side_angle_rate"]))


if __name__ == "__main__":
    main(sys.argv[1:])
