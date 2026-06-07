#!/usr/bin/env python3
"""Bake a Wavefront .obj terrain mesh to a heightmap for VDSim physics.

Parses the mesh vertices, interpolates z onto a regular (x,y) grid (scipy
griddata), and drives a SimSession on it via make_sim_session_heightmap so the
terrain's slopes feed the slope-gravity in the dynamics. A self-demo drives
straight across the mesh and reports how the local grade changes the speed.

NOTE: a CarMaker/IPG .obj terrain and an OpenDRIVE .xodr are usually in
different coordinate frames (different origin/units) — they do NOT overlay
without an alignment transform. This drives on the terrain in its own frame.

Usage:
    python3 examples/obj_to_heightmap.py <mesh.obj> [cell_m]
"""
import math
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import griddata

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "build" / "python"))
import vdsim  # noqa: E402


def load_obj_vertices(path):
    xs, ys, zs = [], [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                xs.append(float(x)); ys.append(float(y)); zs.append(float(z))
    return np.array(xs), np.array(ys), np.array(zs)


def bake_heightmap(path, cell=5.0):
    x, y, z = load_obj_vertices(path)
    x0, y0, x1, y1 = x.min(), y.min(), x.max(), y.max()
    nx = max(2, int((x1 - x0) / cell) + 1)
    ny = max(2, int((y1 - y0) / cell) + 1)
    gx, gy = np.meshgrid(np.linspace(x0, x1, nx), np.linspace(y0, y1, ny))
    H = griddata((x, y), z, (gx, gy), method="linear")
    Hn = griddata((x, y), z, (gx, gy), method="nearest")   # fill outside hull
    H = np.where(np.isnan(H), Hn, H)
    dx = (x1 - x0) / (nx - 1); dy = (y1 - y0) / (ny - 1)
    return H, x0, y0, dx, dy, (x0, y0, x1, y1, z.min(), z.max())


def main():
    if len(sys.argv) < 2:
        print("usage: obj_to_heightmap.py <mesh.obj> [cell_m]"); return
    path = sys.argv[1]
    cell = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    H, x0, y0, dx, dy, bb = bake_heightmap(path, cell)
    print(f"=== {path} -> heightmap {H.shape[1]}x{H.shape[0]} @ {cell} m ===")
    print(f"  xy bbox [{bb[0]:.0f},{bb[2]:.0f}]x[{bb[1]:.0f},{bb[3]:.0f}] m, "
          f"elevation [{bb[4]:.1f},{bb[5]:.1f}] m")

    from _catalog_load import load_vehicle_tire
    vp, tp = load_vehicle_tire()
    sess = vdsim.make_sim_session_heightmap(vp, tp, "L2", H,
                                            x0=x0, y0=y0, dx=dx, dy=dy, mu=1.0,
                                            nominal_dt=0.005)
    # start near the mesh centre, heading +x, coast (throttle just holds ~target)
    cx, cy = 0.5 * (bb[0] + bb[2]), 0.5 * (bb[1] + bb[3])
    s = vdsim.State(); s.position = [cx, cy, 0.0]; s.velocity = [12.0, 0.0, 0.0]
    sess.reset(s)
    print("  driving +x across the mesh (coast) — speed reacts to local grade:")
    c = vdsim.CmdL4()
    for k in range(1, 1201):
        sess.set_input(c); sess.tick(0.005)
        if k % 200 == 0:
            o = sess.output()
            print(f"    t={k*0.005:4.1f}s  x={o.state.position[0]:7.1f}  "
                  f"vx={o.state.vx():5.2f}  ax={o.ax:+5.2f}")


if __name__ == "__main__":
    main()
