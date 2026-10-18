"""Audit junction boundaries against OpenDRIVE 1.8.0 §12.10 (and the grid rule of §12.11).

Measured:
  closed          "Segments shall close the entire junction boundary": the end of each segment meets the start of the
                  next (kTol); a joint runs across all lanes of its road end (jointLaneStart .. jointLaneEnd when given,
                  Table 78), a lane segment along the outer edge of
                  boundaryLane (the reference line for 0) from sStart to sEnd ("start" / "end" or a number)
  empty_segment   a lane segment whose sStart and sEnd are the same point (no length: it closes nothing)
  counter_clockwise  "Segments shall be ordered counter clockwise": positive signed area of the outline
  reaches_all_roads  "Segments shall be defined to reach the start or end of all roads connected to the junction": a
                  joint for every plain road end linked to the junction
  contains_lanes  the outline encloses both borders and the centre of every lane of the junction's roads ("Junction
                  boundaries enclose the area intended for traffic. This also includes the sidewalks for pedestrians.")
  simple          the outline does not cross itself (a closed boundary around one area)
  folded          no lane segment's edge runs against its road's direction (an offset past a turn's centre: a cusp)
  ref_line_crosses  junctions.geometry.correct_junction_boundry (1.8.0 §12.9): the junction reference line crosses the
                  boundary or touches it
  grid_covers     junctions.elevation_grid.valid_for_entire_boundry: every outline point inside the grid squares
  helper_junction_id, helper_linkage, helper_points_inside  junctions.boundary.close_gap_with_new_roads for the additional
                  roads (name JunctionBoundary): @junction, road linkage to a road of the junction whose end cross-section
                  (reference line or lanes) holds the helper's end, and both ends pointing from the linked road into the junction
usage: python boundary_audit.py file.xodr [...]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402

TOL = 0.10


def edge_t(r, s, lane):
    if lane == 0:
        return float(r.lane_offset(np.array([s]))[0]) if hasattr(r.lane_offset(np.array([s])), "__len__") else float(r.lane_offset(s))
    tin, tout = r.borders_at(s, r.section_at(s))[lane]
    return tout


def pt(r, s, t):
    x, y = r.point(np.array([s]), t)
    return np.array([float(x[0]), float(y[0])])


def s_of(r, v):
    return 0.0 if v == "start" else r.length if v == "end" else float(v)


def inside(poly, p):
    x, y, c = p[0], p[1], False
    for (x0, y0), (x1, y1) in zip(poly, np.roll(poly, -1, axis=0)):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            c = not c
    return c


def dist_to_poly(poly, p):
    best = 1e300
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        ab = b - a
        u = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
        best = min(best, float(np.linalg.norm(a + u * ab - p)))
    return best


def crossing(a, b, c, d):
    o = lambda p, q, r: (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return o(a, b, c) * o(a, b, d) < 0 and o(c, d, a) * o(c, d, b) < 0


def self_crossing(P):
    n = len(P)
    for i in range(n):
        for k in range(i + 2, n):
            if i == 0 and k == n - 1:
                continue
            if crossing(P[i], P[(i + 1) % n], P[k], P[(k + 1) % n]):
                return True
    return False


def audit(path, quiet=False):
    """{"boundaries": n, "fails": Counter, "by_junction": {junction id: Counter}}; prints one summary line unless quiet"""
    od = G.OpenDrive(path)
    root = ET.parse(path).getroot()
    R = od.roads
    fails, n, gaps, outside, pointing = Counter(), 0, [], [], []
    by_junction = defaultdict(Counter)

    def add(check, jid):
        fails[check] += 1
        by_junction[jid][check] += 1
    for j in root.findall("junction"):
        segs = j.findall("boundary/segment")
        if not segs:
            continue
        n += 1
        jid = j.get("id")
        # each segment's points; a joint's two ends are ordered after the others: it ends where the next one starts
        seg_pts = []
        for sg in segs:
            r = R.get(sg.get("roadId"))
            if r is None:
                add("unknown_road", jid)
                seg_pts.append(None)
            elif sg.get("type") == "joint":
                s = 0.0 if sg.get("contactPoint") == "start" else r.length
                b = r.borders_at(s, r.section_at(s))
                if sg.get("jointLaneStart") is not None:  # across the lanes jointLaneStart .. jointLaneEnd (Table 78)
                    lo, hi = sorted((int(sg.get("jointLaneStart")), int(sg.get("jointLaneEnd"))))
                    b = {k: v for k, v in b.items() if k != 0 and lo <= k <= hi}
                ts = [t for v in b.values() for t in v]
                seg_pts.append([pt(r, s, min(ts)), pt(r, s, max(ts))])
            else:
                lane = int(sg.get("boundaryLane"))
                s0, s1 = s_of(r, sg.get("sStart")), s_of(r, sg.get("sEnd"))
                if abs(s1 - s0) < 1e-3:  # empty_segment: a lane segment of no length (sStart = sEnd)
                    add("empty_segment", jid)
                ss = np.linspace(s0, s1, max(2, int(abs(s1 - s0) / 0.5) + 1))
                pts = [pt(r, s, edge_t(r, s, lane)) for s in ss]
                # folded: the edge runs against its road's direction somewhere (an offset past a turn's centre)
                for k in range(len(ss) - 1):
                    h = float(r.ref(ss[k])[2])
                    if float(np.dot(pts[k + 1] - pts[k], [math.cos(h), math.sin(h)])) * (s1 - s0) < 0:
                        add("folded", jid)
                        break
                seg_pts.append(pts)
        if any(p is None for p in seg_pts):
            continue
        for k, sg in enumerate(segs):
            if sg.get("type") == "joint":
                nxt = seg_pts[(k + 1) % len(segs)]
                target = nxt[0] if segs[(k + 1) % len(segs)].get("type") != "joint" else None
                if target is not None and np.linalg.norm(seg_pts[k][0] - target) < np.linalg.norm(seg_pts[k][1] - target):
                    seg_pts[k] = seg_pts[k][::-1]
        poly, chain = [], []
        for pts in seg_pts:
            chain.append((pts[0], pts[-1]))
            if poly and np.linalg.norm(pts[0] - poly[-1]) <= TOL:
                poly.pop()  # a join (its gap checked below) is one corner: two points this close would cross by a hair
            poly.extend(pts)
        for k in range(len(chain)):
            d = float(np.linalg.norm(chain[k][1] - chain[(k + 1) % len(chain)][0]))
            if d > TOL:
                add("closed", jid)
                gaps.append(round(d, 3))
        P = np.array(poly)
        keep = [0] + [i for i in range(1, len(P)) if np.linalg.norm(P[i] - P[i - 1]) > 1e-9]  # repeated points
        P = P[keep]
        if np.linalg.norm(P[0] - P[-1]) <= TOL:
            P = P[:-1]
        if self_crossing(P[::2] if len(P) > 1500 else P):
            add("simple", jid)
        area = 0.5 * float(np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]))
        if area <= 0:
            add("counter_clockwise", jid)
        joints = {(sg.get("roadId"), sg.get("contactPoint")) for sg in segs if sg.get("type") == "joint"}
        for r in R.values():
            if r.junction != "-1":
                continue
            for link, cp in ((r.pred, "start"), (r.succ, "end")):
                if link is not None and link.type == "junction" and link.id == jid and (r.id, cp) not in joints:
                    add("reaches_all_roads", jid)
        # additional roads (close_gap_with_new_roads): @junction this junction; road linkage (§10.3) to a road of the
        # junction whose end cross-section holds the helper's reference line end (along it within TOL, across it within
        # the road's lanes + TOL); "Start and end ... shall point away from both incoming and/or outgoing roads to the
        # inside of the junction": at either end, the helper's direction into its own length points into the junction
        for rid in {sg.get("roadId") for sg in segs if sg.get("type") == "lane"}:
            h = R.get(rid)
            if h is None or h.name != "JunctionBoundary":
                continue
            if h.junction != jid:
                add("helper_junction_id", jid)
            for link, sh, sign in ((h.pred, 0.0, 1.0), (h.succ, h.length, -1.0)):
                o = R.get(link.id) if link is not None and link.type == "road" else None
                if o is None or link.contact not in ("start", "end"):
                    add("helper_linkage", jid)
                    continue
                so = 0.0 if link.contact == "start" else o.length
                xo, yo, ho = (float(v) for v in o.ref(so))
                xh, yh, hh = (float(v) for v in h.ref(sh))
                do = np.array([math.cos(ho), math.sin(ho)])
                into = do if link.contact == "end" else -do  # from the linked road into the junction
                d = np.array([xh - xo, yh - yo])
                ts = [0.0] + [t for v in o.borders_at(so).values() for t in v]  # its reference line or lanes
                lat =float(d @ np.array([-do[1], do[0]]))
                if abs(float(d @ do)) > TOL or not (min(ts) - TOL <= lat <= max(ts) + TOL):
                    add("helper_linkage", jid)
                if sign * float(np.array([math.cos(hh), math.sin(hh)]) @ into) <= 0.0:
                    add("helper_points_inside", jid)
                pointing.append(sign * float(np.array([math.cos(hh), math.sin(hh)]) @ into))
        # lane centres of the connecting roads (not the helper / crossing roads)
        for r in R.values():
            if r.junction != jid or r.el.find("lanes/laneSection/right/lane") is None and r.el.find("lanes/laneSection/left/lane") is None:
                continue
            for s in np.linspace(0, r.length, max(2, int(r.length / 2) + 1)):
                for lid, (tin, tout) in r.borders_at(s, r.section_at(s)).items():
                    for t in (tin, 0.5 * (tin + tout), tout):
                        p = pt(r, s, t)
                        if not inside(P, p):
                            d = dist_to_poly(P, p)
                            if d > TOL:
                                add("contains_lanes", jid)
                                outside.append(round(d, 2))
        # grid coverage
        g = j.find("planView/geometry")
        eg = j.find("elevationGrid")
        if g is not None:
            x0, y0, hdg, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
            a, b = np.array([x0, y0]), np.array([x0 + L * math.cos(hdg), y0 + L * math.sin(hdg)])
            if not any(crossing(a, b, P[i], P[(i + 1) % len(P)]) for i in range(len(P))) and \
                    min(dist_to_poly(P, a + (b - a) * u) for u in np.linspace(0, 1, 21)) > TOL:
                add("ref_line_crosses", jid)
        if g is not None and eg is not None:
            d, nl = np.array([math.cos(hdg), math.sin(hdg)]), np.array([-math.sin(hdg), math.cos(hdg)])
            gs, s0, rows = float(eg.get("gridSpacing")), float(eg.get("sStart") or 0.0), eg.findall("elevation")
            for p in P:
                q = p - np.array([x0, y0])
                ps, pt_ = float(q @ d) - s0, float(q @ nl)  # rows at sStart + k gridSpacing
                k = min(max(int(math.floor(ps / gs + 1e-9)), 0), max(len(rows) - 2, 0))
                # the point's square needs both of its rows to reach it
                pair = rows[k:k + 2]
                nl_ = min(len((r_.get("left") or "").split()) for r_ in pair)
                nr_ = min(len((r_.get("right") or "").split()) for r_ in pair)
                if not (-1e-6 <= ps <= (len(rows) - 1) * gs + 1e-6 and -nr_ * gs - 1e-6 <= pt_ <= nl_ * gs + 1e-6):
                    add("grid_covers", jid)
                    break
    o = np.array(outside) if outside else np.zeros(1)
    if not quiet:
        print(f"{path}: {n} boundaries; fails {dict(fails)}; closure gaps max {max(gaps) if gaps else 0}; lane centres outside: "
          f"{len(outside)} (max {o.max():.2f} m); helper ends: {len(pointing)}, min cos(direction, into the junction) "
          f"{min(pointing) if pointing else 0:.3f}")
    return {"boundaries": n, "fails": fails, "by_junction": dict(by_junction)}


if __name__ == "__main__":
    for p in sys.argv[1:]:
        audit(p)
