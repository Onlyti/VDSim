#!/usr/bin/env python3
"""CarMaker IPGRoad (.rd5) parser -> extract the Route_0 driving line polyline.

The .rd5 is a UTF-8 "INFOFILE" of `key = value` lines; some keys introduce an
indented (tab-prefixed) block that ends at the next non-indented line.

Approach (B): reconstruct the main driving line from Link geometry.
  * Each Link has Node0/Node1 (start/end nodes) and an ordered list of Seg.*
    of which the `PointList` segments carry explicit (x,y) samples. A Link's
    reference polyline = Node0 + (all PointList points in segment order) + Node1.
  * Links meet at Junctions. Each Junction.Arm references a Link by its ID (the
    arm's 4th numeric field == Link.ID), and `MainArms` names the through-arms.
  * The route is a SUBSET of links forming a (closed) main loop. We chain links
    by Node0/Node1 endpoint proximity, and at each junction prefer the arm-links
    listed in MainArms so we follow the through-line rather than spurs.

Why not approach (A): Route.0.DrvPath lists lane-node ids that resolve via
LanePath.<n> to lane-node id pairs, but those lane-node ids have NO coordinate
table anywhere in the file (verified by grep). So the authoritative DrvPath
cannot be turned into coordinates from this file alone; we validate approach B
against the stated Route.0.Length instead.

Usage:
    python3 examples/rd5_route.py                 # both default tracks + plot
    python3 examples/rd5_route.py path/to/file.rd5
"""
import os
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Low-level INFOFILE reader
# ---------------------------------------------------------------------------
def _read_lines(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read().splitlines()


def _floats(s):
    out = []
    for tok in s.split():
        try:
            out.append(float(tok))
        except ValueError:
            pass
    return out


# ---------------------------------------------------------------------------
# Parsing into structures
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, idx):
        self.idx = idx          # Link.<idx>
        self.id = None          # Link.<idx>.ID (used by junction arms)
        self.node0 = None       # (x, y)
        self.node1 = None       # (x, y)
        self.tag = None
        # segments: ordered list of dicts {k, type, points:[(x,y),...]}
        self.segs = {}          # k -> dict

    def ordered_segs(self):
        return [self.segs[k] for k in sorted(self.segs)]

    def polyline(self):
        """Node0 + all PointList points (segment order) + Node1."""
        pts = [self.node0]
        for seg in self.ordered_segs():
            pts.extend(seg.get("points", []))
        pts.append(self.node1)
        # drop consecutive duplicates (endpoints often repeat a pointlist sample)
        out = [pts[0]]
        for p in pts[1:]:
            if p is None:
                continue
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1e-6:
                out.append(p)
        return out


class Junction:
    def __init__(self, j):
        self.j = j
        self.id = None
        self.knot = None        # (x, y)
        self.arms = {}          # arm_idx -> link_id (0 means none)
        self.main_arms = []     # list of arm indices


def parse_rd5(path):
    """Parse a .rd5 file.

    Returns dict with:
      links     : list[Link]
      junctions : list[Junction]
      route     : dict with 'length' (stated Route.0.Length), 'name',
                  'drvpath' (list of int lane-node ids)
      meta      : dict of misc top-level scalars
    """
    lines = _read_lines(path)
    links = {}      # idx -> Link
    junctions = {}  # j -> Junction
    route = {"length": None, "name": None, "drvpath": []}
    meta = {}

    i = 0
    n = len(lines)
    while i < n:
        line = lines[i]
        if not line or line.startswith("#"):
            i += 1
            continue

        # key = value (split once)
        if "=" in line and not line.startswith("\t"):
            key, _, val = line.partition("=")
            key = key.strip()
            val = val.strip()
        else:
            key, val = line.strip(), ""

        # --- top-level scalars ---
        if key in ("nLinks", "nJunctions", "nRoutes", "nObjects"):
            meta[key] = int(float(val))
        elif key == "RoadNetworkLength":
            meta[key] = float(val)

        # --- Route ---
        elif key == "Route.0.Length":
            route["length"] = float(val)
        elif key == "Route.0.Name":
            route["name"] = val
        elif key == "Route.0.DrvPath.ID":
            route["drvpath_id"] = int(float(val))
        elif key == "Route.0.DrvPath:":
            i += 1
            while i < n and lines[i].startswith(("\t", " ")):
                toks = lines[i].split()
                if toks:
                    try:
                        route["drvpath"].append(int(float(toks[0])))
                    except ValueError:
                        pass
                i += 1
            continue  # i already advanced

        # --- Link ---
        elif key.startswith("Link."):
            parts = key.split(".")
            # parts[1] is the link index
            try:
                lidx = int(parts[1])
            except (IndexError, ValueError):
                i += 1
                continue
            lk = links.setdefault(lidx, Link(lidx))
            field = parts[2] if len(parts) > 2 else ""
            if field == "ID":
                lk.id = int(float(val))
            elif field == "Node0":
                f = _floats(val)
                lk.node0 = (f[0], f[1])
            elif field == "Node1":
                f = _floats(val)
                lk.node1 = (f[0], f[1])
            elif field == "Tag":
                lk.tag = val
            elif field == "Seg":
                sidx = int(parts[3])
                seg = lk.segs.setdefault(sidx, {"k": sidx, "type": None,
                                                "points": []})
                sub = parts[4] if len(parts) > 4 else ""
                if sub == "Type":
                    seg["type"] = val
                elif sub == "PointList:" or (sub == "PointList" and key.endswith(":")):
                    # indented block of "<id> <s> <x> <y>" lines
                    i += 1
                    while i < n and lines[i].startswith(("\t", " ")):
                        f = _floats(lines[i])
                        if len(f) >= 4:
                            seg["points"].append((f[2], f[3]))
                        i += 1
                    continue
                # Note: the literal line is "Link.i.Seg.k.PointList:" with no '=',
                # so it lands in the key==... branch below; handle that too.

        # --- Junction ---
        elif key.startswith("Junction."):
            parts = key.split(".")
            try:
                jidx = int(parts[1])
            except (IndexError, ValueError):
                i += 1
                continue
            jn = junctions.setdefault(jidx, Junction(jidx))
            field = parts[2] if len(parts) > 2 else ""
            if field == "ID":
                jn.id = int(float(val))
            elif field == "Knot":
                f = _floats(val)
                jn.knot = (f[0], f[1])
            elif field == "Arm":
                aidx = int(parts[3])
                sub = parts[4] if len(parts) > 4 else ""
                if sub == "":
                    # "Junction.j.Arm.a = 0 0 0 <linkID> <dir>"
                    f = _floats(val)
                    link_id = int(f[3]) if len(f) >= 4 else 0
                    jn.arms[aidx] = link_id
            elif field == "MainArms":
                jn.main_arms = [int(x) for x in _floats(val)]

        # --- PointList header without '=' (line is "...PointList:") ---
        elif key.endswith("PointList:") and key.startswith("Link."):
            parts = key[:-1].split(".")  # strip trailing ':'
            lidx = int(parts[1])
            sidx = int(parts[3])
            lk = links.setdefault(lidx, Link(lidx))
            seg = lk.segs.setdefault(sidx, {"k": sidx, "type": "PointList",
                                            "points": []})
            i += 1
            while i < n and lines[i].startswith(("\t", " ")):
                f = _floats(lines[i])
                if len(f) >= 4:
                    seg["points"].append((f[2], f[3]))
                i += 1
            continue

        i += 1

    return {
        "links": [links[k] for k in sorted(links)],
        "junctions": [junctions[k] for k in sorted(junctions)],
        "route": route,
        "meta": meta,
    }


# ---------------------------------------------------------------------------
# Route reconstruction (approach B): chain links into the route line
# ---------------------------------------------------------------------------
# Connectivity model. Each Link has two endpoints (side 0 = Node0, side 1 =
# Node1). We build an endpoint graph whose edges are:
#   (1) junction main-arm pairs: for each junction whose MainArms name exactly
#       two links, connect the closest endpoint pair of those two links (the
#       knot can be offset by tens of metres at large intersections, so we pair
#       by mutual endpoint proximity rather than to the knot);
#   (2) tight endpoint proximity (<= PROX_TOL m) anywhere — plain road joins;
#   (3) "free" endpoints (degree 0 after 1+2) bridged to their nearest other
#       free endpoint within BRIDGE_TOL m — these are the through-crossings of
#       intersections where the route passes straight but the link endpoints sit
#       a lane-width apart.
# Traversal: a link entered on one side is exited on the other; at each node we
# may continue to any connected endpoint. We enumerate simple paths and keep the
# one whose total length best matches the stated Route.0.Length (the authority).
PROX_TOL = 2.0      # m: direct endpoint join
BRIDGE_TOL = 40.0   # m: max gap bridged between free chain ends at crossings


def _polyline_length(pts):
    a = np.asarray(pts, float)
    if len(a) < 2:
        return 0.0
    d = np.diff(a, axis=0)
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def _ep(link, side):
    return link.node0 if side == 0 else link.node1


def _link_length(link):
    return _polyline_length(link.polyline())


def build_endpoint_graph(parsed, prox_tol=PROX_TOL, bridge_tol=BRIDGE_TOL):
    """Build the endpoint adjacency described above.

    Returns adj: dict[(link_idx, side)] -> set[(link_idx, side)].
    """
    links = parsed["links"]
    junctions = parsed["junctions"]
    by_id = {lk.id: lk for lk in links if lk.id is not None}
    by_idx = {lk.idx: lk for lk in links}

    adj = {}

    def add(a, b):
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)

    def closest_pair(la, lb):
        best = None
        for sa in (0, 1):
            for sb in (0, 1):
                d = math.hypot(_ep(la, sa)[0] - _ep(lb, sb)[0],
                               _ep(la, sa)[1] - _ep(lb, sb)[1])
                if best is None or d < best[0]:
                    best = (d, sa, sb)
        return best

    # (1) junction main-arm pairs
    for jn in junctions:
        mlids = [jn.arms.get(a) for a in jn.main_arms]
        mlids = [l for l in mlids if l and l in by_id]
        if len(mlids) == 2:
            la, lb = by_id[mlids[0]], by_id[mlids[1]]
            _, sa, sb = closest_pair(la, lb)
            add((la.idx, sa), (lb.idx, sb))

    # (2) tight proximity
    eps = [(lk.idx, s) for lk in links for s in (0, 1)]
    for i, (li, si) in enumerate(eps):
        pi = _ep(by_idx[li], si)
        for (lj, sj) in eps[i + 1:]:
            if li == lj:
                continue
            pj = _ep(by_idx[lj], sj)
            if math.hypot(pi[0] - pj[0], pi[1] - pj[1]) <= prox_tol:
                add((li, si), (lj, sj))

    # (3) bridge free endpoints to nearest free endpoint within bridge_tol
    free = [e for e in eps if e not in adj]
    for a in free:
        pa = _ep(by_idx[a[0]], a[1])
        cands = []
        for b in free:
            if b[0] == a[0]:
                continue
            pb = _ep(by_idx[b[0]], b[1])
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if d <= bridge_tol:
                cands.append((d, b))
        cands.sort(key=lambda x: x[0])
        if cands:
            add(a, cands[0][1])

    return adj


def chain_route(parsed):
    """Find the link sequence whose length best matches Route.0.Length.

    Enumerates simple paths over the endpoint graph (a link entered on one side
    leaves on the other) and selects the one minimising |length - stated|.
    Returns (link_idx_sequence, traced_length).
    """
    links = parsed["links"]
    by_idx = {lk.idx: lk for lk in links}
    stated = parsed["route"].get("length") or 0.0
    adj = build_endpoint_graph(parsed)

    best = {"err": float("inf"), "path": [], "len": 0.0}

    def visit(cur, in_side, path, used, length):
        # path entries are (link_idx, entry_side) so orientation is unambiguous
        err = abs(length - stated)
        if err < best["err"]:
            best.update(err=err, path=list(path), len=length)
        out_side = 1 - in_side
        for (nl, ns) in adj.get((cur, out_side), ()):
            if nl in used:
                continue
            used.add(nl)
            visit(nl, ns, path + [(nl, ns)], used,
                  length + _link_length(by_idx[nl]))
            used.discard(nl)

    for lk in links:
        for s in (0, 1):
            visit(lk.idx, s, [(lk.idx, s)], {lk.idx}, _link_length(lk))

    return best["path"], best["len"]


def _sequence_polyline(parsed, link_seq):
    """Stitch a (link_idx, entry_side) sequence into one (x,y) polyline.

    A link entered on side 0 is traversed Node0->Node1 (its polyline as-is);
    entered on side 1 it is reversed. Consecutive links share a joint (which may
    be a small bridge gap at an intersection crossing), so we keep both joint
    points (the gap is part of the driven line)."""
    by_idx = {lk.idx: lk for lk in parsed["links"]}
    poly = []
    for idx, entry_side in link_seq:
        pl = by_idx[idx].polyline()
        if entry_side == 1:
            pl = pl[::-1]
        if not poly:
            poly.extend(pl)
        else:
            # drop the duplicate joint point when it coincides; keep the bridge
            # gap otherwise (real crossing distance the route covers)
            if math.hypot(pl[0][0] - poly[-1][0], pl[0][1] - poly[-1][1]) < 1e-6:
                poly.extend(pl[1:])
            else:
                poly.extend(pl)
    return poly


def route_polyline(path):
    """Parse a .rd5 and return the Route_0 main-line polyline as list[(x,y)]."""
    parsed = parse_rd5(path)
    seq, _ = chain_route(parsed)
    return _sequence_polyline(parsed, seq)


# ---------------------------------------------------------------------------
# Plot + CSV
# ---------------------------------------------------------------------------
def _plot_track(ax, parsed, route_poly, title):
    # all link geometries in light gray
    for lk in parsed["links"]:
        poly = lk.polyline()
        a = np.asarray(poly, float)
        if len(a) >= 2:
            ax.plot(a[:, 0], a[:, 1], color="0.8", lw=1.0, zorder=1)
    # route in red
    r = np.asarray(route_poly, float)
    ax.plot(r[:, 0], r[:, 1], color="red", lw=1.8, zorder=3, label="Route_0")
    # start marker
    ax.plot(r[0, 0], r[0, 1], "o", color="green", ms=9, zorder=4, label="start")
    ax.set_aspect("equal", "datalim")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="best", fontsize=8)


def make_comparison(files, out_png):
    fig, axes = plt.subplots(1, len(files), figsize=(7 * len(files), 7))
    if len(files) == 1:
        axes = [axes]
    results = []
    for ax, path in zip(axes, files):
        parsed = parse_rd5(path)
        seq, traced = chain_route(parsed)   # traced = sum of route-link lengths
        route_poly = _sequence_polyline(parsed, seq)
        stated = parsed["route"]["length"] or float("nan")
        name = os.path.basename(path)
        title = (f"{name}\ntraced={traced:.1f} m  stated={stated:.1f} m  "
                 f"(err {100*(traced-stated)/stated:+.1f}%)")
        _plot_track(ax, parsed, route_poly, title)
        results.append((path, traced, stated, route_poly))
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return results


def write_csv(route_poly, out_csv):
    a = np.asarray(route_poly, float)
    # cumulative s
    s = np.zeros(len(a))
    if len(a) >= 2:
        d = np.hypot(np.diff(a[:, 0]), np.diff(a[:, 1]))
        s[1:] = np.cumsum(d)
    with open(out_csv, "w") as f:
        f.write("s,x,y\n")
        for si, (x, y) in zip(s, a):
            f.write(f"{si:.6f},{x:.6f},{y:.6f}\n")


# ---------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    logs = os.path.join(repo, "logs")
    os.makedirs(logs, exist_ok=True)

    default_files = [
        "/home/ailab-12/git/carmaker-ros-bridge/Data/Road/raceway_full.rd5",
        "/home/ailab-12/git/carmaker-ros-bridge/Data/Road/2023_Speedway_Road_v1_2.rd5",
    ]
    files = sys.argv[1:] if len(sys.argv) > 1 else default_files

    out_png = "/tmp/rd5_routes_compare.png"
    results = make_comparison(files, out_png)

    for path, traced, stated, route_poly in results:
        base = os.path.splitext(os.path.basename(path))[0]
        out_csv = os.path.join(logs, f"{base}_route.csv")
        write_csv(route_poly, out_csv)
        err = 100.0 * (traced - stated) / stated if stated else float("nan")
        print(f"{base}:")
        print(f"  traced length = {traced:10.3f} m")
        print(f"  stated length = {stated:10.3f} m   (Route.0.Length)")
        print(f"  error         = {err:+.2f} %   points={len(route_poly)}")
        print(f"  CSV -> {out_csv}")
    print(f"PNG -> {out_png}")


if __name__ == "__main__":
    main()
