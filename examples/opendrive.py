#!/usr/bin/env python3
"""OpenDRIVE (.xodr) reference-line parser -> sampled path + grade/bank.

Reads a road network's planView geometry and samples each road's reference line
to a polyline (x, y, heading, s, curvature) usable as a tracking reference for
the pure-pursuit / maneuvers / Monte-Carlo harnesses, plus elevation (grade) and
superelevation (bank) along s for the road surface (feeds slope-gravity).

Supported geometry: line, arc (constant curvature). spiral (clothoid) and
poly3/paramPoly3 are not yet implemented (raise NotImplementedError) — the
common straight+arc roads and the self-test below work today.

Usage:
    python3 examples/opendrive.py            # self-test on a synthetic .xodr
    python3 examples/opendrive.py road.xodr  # parse a real file
"""
import math
import sys
import xml.etree.ElementTree as ET


class Geometry:
    def __init__(self, s0, x0, y0, hdg0, length, kind, curvature=0.0,
                 curv0=0.0, curv1=0.0):
        self.s0, self.x0, self.y0, self.hdg0 = s0, x0, y0, hdg0
        self.length, self.kind, self.k = length, kind, curvature
        self.curv0, self.curv1 = curv0, curv1   # spiral: linear curvature start->end

    def pose(self, s):                       # s: arc length from this geometry's start
        ds = s - self.s0
        if self.kind == "spiral":            # clothoid: curvature linear in s
            return self._spiral_pose(ds)
        if self.kind == "line" or abs(self.k) < 1e-9:
            return (self.x0 + ds * math.cos(self.hdg0),
                    self.y0 + ds * math.sin(self.hdg0),
                    self.hdg0)
        k, h0 = self.k, self.hdg0            # arc: constant curvature
        x = self.x0 + (math.sin(h0 + k * ds) - math.sin(h0)) / k
        y = self.y0 - (math.cos(h0 + k * ds) - math.cos(h0)) / k
        return (x, y, h0 + k * ds)

    def _spiral_pose(self, ds):              # numeric integration (trapezoidal)
        L = max(1e-9, self.length)
        n = max(1, int(ds / 0.2))
        h = ds / n
        x, y, hdg, cs = self.x0, self.y0, self.hdg0, 0.0
        for _ in range(n):
            k0 = self.curv0 + (self.curv1 - self.curv0) * (cs / L)
            k1 = self.curv0 + (self.curv1 - self.curv0) * ((cs + h) / L)
            hmid = hdg + 0.5 * k0 * h
            x += h * math.cos(hmid); y += h * math.sin(hmid)
            hdg += 0.5 * (k0 + k1) * h
            cs += h
        return (x, y, hdg)


class _Poly:                                  # cubic a+b*ds+c*ds^2+d*ds^3 from s0
    def __init__(self, s0, a, b, c, d):
        self.s0, self.a, self.b, self.c, self.d = s0, a, b, c, d
    def value(self, s):
        ds = s - self.s0
        return self.a + self.b*ds + self.c*ds*ds + self.d*ds**3
    def deriv(self, s):
        ds = s - self.s0
        return self.b + 2*self.c*ds + 3*self.d*ds*ds


class Road:
    def __init__(self, geoms, elevs, supers, length):
        self.geoms, self.elevs, self.supers, self.length = geoms, elevs, supers, length

    @staticmethod
    def _pick(polys, s):
        cur = None
        for p in polys:
            if p.s0 <= s + 1e-9:
                cur = p
        return cur

    def pose(self, s):
        g = self.geoms[0]
        for gg in self.geoms:
            if gg.s0 <= s + 1e-9:
                g = gg
        return g.pose(s)

    def grade(self, s):                       # dz/ds from elevation profile [rad-ish slope]
        p = self._pick(self.elevs, s)
        return math.atan(p.deriv(s)) if p else 0.0

    def bank(self, s):                        # superelevation angle [rad]
        p = self._pick(self.supers, s)
        return p.value(s) if p else 0.0

    def sample(self, step=1.0):
        out, s = [], 0.0
        while s < self.length - 1e-9:
            x, y, hdg = self.pose(s)
            out.append((x, y, hdg, s, self.grade(s), self.bank(s)))
            s += step
        x, y, hdg = self.pose(self.length)          # always include the exact end
        out.append((x, y, hdg, self.length, self.grade(self.length), self.bank(self.length)))
        return out


def parse_xodr(src):
    root = ET.fromstring(src) if src.lstrip().startswith("<") else ET.parse(src).getroot()
    roads = []
    for road in root.findall("road"):
        length = float(road.get("length", 0.0))
        geoms = []
        for g in road.find("planView").findall("geometry"):
            s0 = float(g.get("s")); x0 = float(g.get("x")); y0 = float(g.get("y"))
            hdg = float(g.get("hdg")); L = float(g.get("length"))
            if g.find("line") is not None:
                geoms.append(Geometry(s0, x0, y0, hdg, L, "line"))
            elif g.find("arc") is not None:
                k = float(g.find("arc").get("curvature"))
                geoms.append(Geometry(s0, x0, y0, hdg, L, "arc", k))
            elif g.find("spiral") is not None:
                sp = g.find("spiral")
                geoms.append(Geometry(s0, x0, y0, hdg, L, "spiral",
                                      curv0=float(sp.get("curvStart")),
                                      curv1=float(sp.get("curvEnd"))))
            else:
                raise NotImplementedError(
                    "geometry type not supported (line/arc/spiral); got "
                    + ",".join(c.tag for c in g))
        def polys(parent, tag):
            out = []
            el = road.find(parent)
            if el is not None:
                for e in el.findall(tag):
                    out.append(_Poly(float(e.get("s")), float(e.get("a")),
                                     float(e.get("b")), float(e.get("c")), float(e.get("d"))))
            return out
        elevs = polys("elevationProfile", "elevation")
        supers = polys("lateralProfile", "superelevation")
        roads.append(Road(geoms, elevs, supers, length))
    return roads


def chain_route(roads, step=2.0, gap_tol=30.0):
    """Recover one ordered drivable polyline [(x,y)] from a road network by
    greedily chaining segments end-to-start (reversing where needed). Seeds at
    the longest road; stops when no unused road starts within gap_tol."""
    polys = [[(p[0], p[1]) for p in r.sample(step)] for r in roads]
    polys = [p for p in polys if len(p) >= 2]
    if not polys:
        return []
    seed = max(range(len(polys)), key=lambda i: len(polys[i]))
    used = {seed}
    route = list(polys[seed])
    def d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    while True:
        end = route[-1]
        best, best_d, rev = -1, gap_tol, False
        for i, p in enumerate(polys):
            if i in used:
                continue
            ds, de = d(end, p[0]), d(end, p[-1])
            if ds < best_d:
                best, best_d, rev = i, ds, False
            if de < best_d:
                best, best_d, rev = i, de, True
        if best < 0:
            break
        seg = polys[best][::-1] if rev else polys[best]
        route.extend(seg)
        used.add(best)
    return route


def _synthetic_xodr():
    # one road: 50 m straight, then a quarter circle R=30 (curvature +1/30),
    # +2% grade, 3 deg bank on the arc portion.
    L_line, R = 50.0, 30.0
    L_arc = 0.5 * math.pi * R
    total = L_line + L_arc
    return f"""<OpenDRIVE>
  <road length="{total}" id="1">
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="{L_line}"><line/></geometry>
      <geometry s="{L_line}" x="{L_line}" y="0" hdg="0" length="{L_arc}">
        <arc curvature="{1.0/R}"/></geometry>
    </planView>
    <elevationProfile><elevation s="0" a="0" b="0.02" c="0" d="0"/></elevationProfile>
    <lateralProfile><superelevation s="{L_line}" a="0.0524" b="0" c="0" d="0"/></lateralProfile>
  </road>
</OpenDRIVE>"""


def main():
    if len(sys.argv) > 1:
        import os
        path = sys.argv[1]
        roads = parse_xodr(path)
        total = sum(r.length for r in roads)
        xs, ys, rows = [], [], []
        for rid, r in enumerate(roads):
            for (x, y, hdg, s, gr, bk) in r.sample(2.0):
                xs.append(x); ys.append(y)
                rows.append((rid, x, y, hdg, s, gr, bk))
        print(f"=== {path} ===")
        print(f"  roads {len(roads)} | total length {total:.1f} m | {len(rows)} pts")
        print(f"  bbox x[{min(xs):.1f}, {max(xs):.1f}]  y[{min(ys):.1f}, {max(ys):.1f}] m")
        out = os.path.join(os.path.dirname(__file__), "..", "logs")
        os.makedirs(out, exist_ok=True)
        csvp = os.path.join(out, os.path.splitext(os.path.basename(path))[0] + "_centerline.csv")
        with open(csvp, "w") as f:
            f.write("road,x,y,hdg,s,grade,bank\n")
            for r in rows:
                f.write("%d,%.4f,%.4f,%.5f,%.3f,%.5f,%.5f\n" % r)
        print(f"  centerline CSV -> {os.path.normpath(csvp)}")
        return
    roads = parse_xodr(_synthetic_xodr())
    r = roads[0]
    pts = r.sample(1.0)
    xe, ye, hdg_e, se, *_ = pts[-1]
    # left-turning quarter arc (curvature +1/30) from (50,0) heading +x about
    # center (50,30): ends at (80,30) heading +90 deg.
    print("=== OpenDRIVE self-test (line 50 m + quarter arc R=30, left) ===")
    print(f"  road length      : {r.length:.2f} m  (expect {50 + 0.5*math.pi*30:.2f})")
    print(f"  end point        : ({xe:.2f}, {ye:.2f})  (expect 80.00, 30.00)")
    print(f"  end heading      : {math.degrees(hdg_e):.1f} deg  (expect 90)")
    print(f"  grade @ s=0      : {math.degrees(r.grade(0)):.2f} deg  (expect atan(0.02)=1.15)")
    print(f"  bank  @ arc      : {math.degrees(r.bank(60)):.2f} deg  (expect ~3.00)")


if __name__ == "__main__":
    main()
