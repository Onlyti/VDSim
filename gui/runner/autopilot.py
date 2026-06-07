import math


class WaypointPath:
    """Pure-pursuit over an ordered polyline (loops). Reused for the figure-8
    default and for loaded OpenDRIVE routes."""
    def __init__(self, pts):
        self.pts = list(pts)

    def steer(self, x, y, yaw, vx, wb, prev_idx):
        n = len(self.pts)
        if n < 2:
            return 0.0, prev_idx
        near, nd = prev_idx, 1e18
        for i in range(n):
            dx = self.pts[i][0] - x
            dy = self.pts[i][1] - y
            d2 = dx * dx + dy * dy
            if d2 < nd:
                nd, near = d2, i
        Ld = max(3.0, 0.6 * max(vx, 1.0))
        idx, cnt = near, 0
        while cnt < n:
            p = self.pts[idx % n]
            if math.hypot(p[0] - x, p[1] - y) >= Ld:
                break
            idx += 1
            cnt += 1
        idx %= n
        cp, sp = math.cos(yaw), math.sin(yaw)
        dxw, dyw = self.pts[idx][0] - x, self.pts[idx][1] - y
        dx = cp * dxw + sp * dyw
        dy = -sp * dxw + cp * dyw
        l2 = dx * dx + dy * dy
        if l2 < 1e-6:
            return 0.0, near
        return max(-0.6, min(0.6, math.atan(2.0 * dy / l2 * wb))), near


def fig8_pts(cx=0.0, cy=0.0, R=20.0, n=80):
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        pts.append((cx + R - R * math.cos(t), cy + R * math.sin(t)))
    for i in range(n):
        t = 2 * math.pi * i / n
        pts.append((cx - R + R * math.cos(t), cy + R * math.sin(t)))
    return pts


class FigureEight(WaypointPath):
    def __init__(self, R=20.0, n=80):
        super().__init__(fig8_pts(0.0, 0.0, R, n))


def compute_vehicle_cmd(path, cs_v, prev_idx, v_target, wheelbase, driver, in_cmd):
    if driver:
        st, prev_idx = path.steer(
            cs_v["x"], cs_v["y"], cs_v["yaw"], cs_v["vx"], wheelbase, prev_idx)
        ax = max(-3.0, min(3.0, 0.8 * (v_target - cs_v["vx"])))
        t = min(1.0, ax / 3.0) if ax >= 0 else 0.0
        b = min(1.0, -ax / 3.0) if ax < 0 else 0.0
    else:
        t = in_cmd["throttle"]
        b = in_cmd["brake"]
        st = in_cmd["steer"]
    return t, b, st, prev_idx
