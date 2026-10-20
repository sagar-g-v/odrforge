"""How far the drawn carriageway edge lies from the OSM kerb lines (the proof measure of kerb-anchored edges).

    python kerb_gap.py <file.osm> <file.xodr> [--json out.json]

Definition agreed with the viewer session (odrforge-viewer src/review/osmEdges.ts 2026-10-18, junction roads added
2026-10-20 as its kerbGapEdges): every open barrier=kerb way, sampled every 1 m; at each sample the signed distance to
the nearest carriageway edge within 10 m whose direction lies within 25 deg of the kerb's. The edges: every plain road's,
and every junction connecting road's on its sides that carry carriageway lanes. The edge is the outer border of the
outermost lane that is not sidewalk, walking, curb, border, none, median or restricted; on a plain road a side without
such lanes while the other side has some (one carriageway of a dual road) has its edge at the lane-0 line (laneOffset)
- a connecting road has no such edge (it would run along the turn's centre line). Signed by the edge's outward normal:
+ the kerb lies outside the edge (the road is drawn too narrow). A kerb is left out when half or more of its samples
lie inside or within 0.5 m of a traffic island (area:highway=traffic_island, footway=traffic_island) or a pedestrian
area (area:highway=pedestrian, highway=pedestrian + area=yes).
Self-test with planted defects: tests/kerb_gap_selftest.py.
"""
import argparse
import json
import math
import sys

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else __file__.rsplit("/", 1)[0])
import odrgeom as G  # noqa: E402

NOT_CARRIAGEWAY = {"sidewalk", "walking", "curb", "border", "none", "median", "restricted"}
STEP, REACH, PARALLEL, NEAR_AREA = 1.0, 10.0, math.radians(25.0), 0.5


def edges(od, step=1.0):
    """carriageway edge polylines, one per section side: [(LineString along s, side +1 left / -1 right)]. Every plain
    road (an empty side's edge at the lane-0 line) and every junction connecting road on its sides that carry
    carriageway lanes only (no lane-0 fallback: it would lay a false edge along the turn's centre line - agreed with
    the viewer session, kerbGapEdges, 2026-10-20)"""
    out = []
    for r in od.roads.values():
        plain = r.junction == "-1"
        for sec in r.sections:
            if sec.e - sec.s < 0.1:
                continue
            s = np.linspace(sec.s, sec.e, max(2, int(math.ceil((sec.e - sec.s) / step)) + 1))
            carr = {side: [l for l in lanes if l.type not in NOT_CARRIAGEWAY] for side, lanes in ((1, sec.left), (-1, sec.right))}
            if not carr[1] and not carr[-1]:
                continue
            for side in (1, -1):
                if not carr[side] and not plain:
                    continue
                if carr[side]:
                    outer = max(carr[side], key=lambda l: abs(l.id))
                    t = np.array([r.borders_at(float(v), sec)[outer.id][1] for v in s])
                else:  # one-sided carriageway: its edge on this side is the lane-0 line
                    t = r.lane_offset(s)
                x, y = r.point(s, t)
                out.append((LineString(list(zip(x, y))), side))
    return out


def islands(osm):
    polys = []
    for nds, t in osm.ways.values():
        isl = t.get("area:highway") in ("traffic_island", "pedestrian") or t.get("footway") == "traffic_island" or (
            t.get("highway") == "pedestrian" and t.get("area") == "yes")
        if not isl or len(nds) < 4 or nds[0] != nds[-1]:
            continue
        pts = [osm.nodes[n] for n in nds if n in osm.nodes]
        if len(pts) == len(nds):
            p = Polygon(pts)
            polys.append(p if p.is_valid else p.buffer(0))
    return polys


def measure(osm_path, xodr_path, rows=None):
    """rows: a list to receive every sample of the kerbs not left out, (way, i, x, y, gap or None when unmatched)"""
    od = G.OpenDrive(xodr_path)
    osm = G.Osm(osm_path, od.lat0, od.lon0)
    EL = edges(od)
    E = [e for e, _ in EL]
    etree = STRtree(E)
    I = islands(osm)
    itree = STRtree(I) if I else None
    gaps, kerbs, dropped, samples, matched = [], 0, 0, 0, 0
    for wid, (nds, t) in osm.ways.items():
        if t.get("barrier") != "kerb" or len(nds) < 2 or nds[0] == nds[-1]:
            continue
        pts = [osm.nodes[n] for n in nds if n in osm.nodes]
        if len(pts) < 2:
            continue
        K = LineString(pts)
        if K.length < STEP:
            continue
        kerbs += 1
        u = np.arange(0.5 * STEP, K.length, STEP)
        P = [K.interpolate(v) for v in u]
        if itree is not None:
            near = sum(any(I[i].distance(p) <= NEAR_AREA for i in itree.query(p.buffer(NEAR_AREA))) for p in P)
            if 2 * near >= len(P):
                dropped += 1
                continue
        for v, p in zip(u, P):
            samples += 1
            a, b = K.interpolate(max(0.0, v - 0.5)), K.interpolate(min(K.length, v + 0.5))
            hk = math.atan2(b.y - a.y, b.x - a.x)
            best = None
            for i in etree.query(p.buffer(REACH)):
                e = E[i]
                d = e.distance(p)
                if d > REACH or (best is not None and d >= best[0]):
                    continue
                s = e.project(p)
                q0, q1 = e.interpolate(max(0.0, s - 0.5)), e.interpolate(min(e.length, s + 0.5))
                he = math.atan2(q1.y - q0.y, q1.x - q0.x)
                dh = abs((hk - he + math.pi / 2) % math.pi - math.pi / 2)  # undirected angle
                if dh > PARALLEL:
                    continue
                best = (d, e, s, q0, q1, EL[i][1])
            if best is None:
                if rows is not None:
                    rows.append((wid, v, p.x, p.y, None))
                continue
            matched += 1
            d, e, s, q0, q1, side = best
            # sign by the edge's outward normal: the polyline runs along s, its left normal times the side
            f = e.interpolate(s)
            nx, ny = -(q1.y - q0.y), q1.x - q0.x
            gaps.append(d if ((p.x - f.x) * nx + (p.y - f.y) * ny) * side >= 0 else -d)
            if rows is not None:
                rows.append((wid, v, p.x, p.y, gaps[-1]))
    return od, gaps, kerbs, dropped, samples, matched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("osm")
    ap.add_argument("xodr")
    ap.add_argument("--json")
    ap.add_argument("--csv", help="every sample: way,i,x,y,gap (gap empty when unmatched; the viewer's KERB_DUMP format)")
    a = ap.parse_args()
    rows = [] if a.csv else None
    od, gaps, kerbs, dropped, samples, matched = measure(a.osm, a.xodr, rows)
    if a.csv:
        with open(a.csv, "w", encoding="utf-8", newline="") as f:
            f.write("way,i,x,y,gap\n")
            for w, i, x, y, g in rows:
                f.write(f"{w},{int(i / STEP)},{x:.3f},{y:.3f},{'' if g is None else f'{g:.4f}'}\n")  # i: sample index
    g = np.array(gaps)
    summary = {"kerbs": kerbs, "dropped": dropped, "samples": samples,
               "matched_share": round(matched / samples, 3) if samples else None,
               "median": round(float(np.median(g)), 2), "abs_p95": round(float(np.percentile(np.abs(g), 95)), 2),
               "within_0.3": round(float(np.mean(np.abs(g) <= 0.3)), 3),
               "within_0.5": round(float(np.mean(np.abs(g) <= 0.5)), 3)}
    print(json.dumps(summary))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(summary, f, indent=1)


if __name__ == "__main__":
    main()
