#!/usr/bin/env python3
"""Bake a small analytic terrain heightmap (.bin) for VDSim L5 terrain demos.

Output format matches the loader in cosim/realtime_server.cpp make_ground():
  int32 nx, int32 ny, float64 x0, y0, dx, dy, then nx*ny float64 heights
  row-major as h[iy*nx + ix] at world (x0 + ix*dx, y0 + iy*dy).

No confidential data — a closed-form Gaussian hill. Keep grids small (<=64x64)
so the committed blob stays tiny.
"""
from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path


def gaussian_hill(x: float, y: float, h_max: float, xc: float, yc: float,
                  sigma: float) -> float:
    r2 = (x - xc) ** 2 + (y - yc) ** 2
    return h_max * math.exp(-r2 / (sigma * sigma))


def bake(out: Path, nx: int, ny: int, x0: float, y0: float, dx: float, dy: float,
         h_max: float, xc: float, yc: float, sigma: float) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("wb") as f:
        f.write(struct.pack("<ii", nx, ny))
        f.write(struct.pack("<dddd", x0, y0, dx, dy))
        for iy in range(ny):
            y = y0 + iy * dy
            for ix in range(nx):
                x = x0 + ix * dx
                f.write(struct.pack("<d", gaussian_hill(x, y, h_max, xc, yc, sigma)))


def main() -> None:
    ap = argparse.ArgumentParser(description="Bake a Gaussian-hill heightmap .bin")
    ap.add_argument("--out", type=Path,
                    default=Path("assets/terrain/hill_demo.bin"))
    ap.add_argument("--nx", type=int, default=61)
    ap.add_argument("--ny", type=int, default=41)
    ap.add_argument("--x0", type=float, default=-15.0)
    ap.add_argument("--y0", type=float, default=-20.0)
    ap.add_argument("--dx", type=float, default=1.0)
    ap.add_argument("--dy", type=float, default=1.0)
    ap.add_argument("--h-max", type=float, default=1.5)
    ap.add_argument("--xc", type=float, default=30.0)
    ap.add_argument("--yc", type=float, default=0.0)
    ap.add_argument("--sigma", type=float, default=8.0)
    a = ap.parse_args()
    bake(a.out, a.nx, a.ny, a.x0, a.y0, a.dx, a.dy, a.h_max, a.xc, a.yc, a.sigma)
    print(f"baked {a.nx}x{a.ny} hill -> {a.out} "
          f"({a.out.stat().st_size} bytes, h_max={a.h_max} m)")


if __name__ == "__main__":
    main()
