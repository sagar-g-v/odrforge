"""How much of each mapped street-side parking area the generated OpenDRIVE draws as parking lanes.

    python parking_coverage.py <file.osm> <file.xodr> [--clip lat0,lon0,lat1,lon1] [--json out.json]

OSM maps street-side parking as its own area next to the road: amenity=parking + parking=street_side | lane
(wiki Key:parking). OpenDRIVE has no free area: street parking is a lane of type parking along the road
(OpenDRIVE 1.8.0 §11.7.1 lane type parking; 1.4H laneType parking). The check lays out every lane of every
road and junction connecting road as a polygon (borders sampled every 1 m) and measures, per OSM polygon, the
share of its area lying under lanes of each type. With --clip (the converter's clip box), only areas lying wholly inside
it count: the converter cuts ways at the box, an area across its edge is no area there.

Per area it prints: polygons, polygons >= 80 % covered by parking lanes ("covered"), the mean parking share,
and the share of polygon area under driving / walking lanes (parking drawn as the wrong surface).
"""
import argparse
import json
import math
import sys
from collections import defaultdict

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else __file__.rsplit("/", 1)[0])
import odrgeom as G  # noqa: E402

COVERED = 0.8  # a polygon counts as drawn when this share of its area lies under parking lanes
STREET_SIDE = ("street_side", "lane", "layby")
GROUPS = {"parking": {"parking"}, "driving": {"driving", "bidirectional", "bus", "taxi", "HOV", "entry", "exit",
                                               "onRamp", "offRamp", "connectingRamp", "slipLane"},
          "walking": {"walking", "sidewalk", "curb", "border"}}


def street_side_polygons(osm, box=None):
    out = {}
    for wid, (nds, t) in osm.ways.items():
        if t.get("amenity") != "parking" or t.get("parking") not in STREET_SIDE or len(nds) < 4 or nds[0] != nds[-1]:
            continue
        pts = [osm.nodes[n] for n in nds if n in osm.nodes]
        if len(pts) != len(nds):
            continue  # clipped at the extract edge
        if box and not all(box[0] <= x <= box[2] and box[1] <= y <= box[3] for x, y in pts):
            continue  # across the converter's clip box
        p = Polygon(pts)
        if not p.is_valid:
            p = p.buffer(0)
        if p.area >= 1.0:
            out[wid] = p
    return out


def lane_polygons(od, step=1.0):
    """{group: [Polygon, ...]} of every lane of every road, grouped by GROUPS"""
    out = defaultdict(list)
    for r in od.roads.values():
        for sec in r.sections:
            if sec.e - sec.s < 0.05:
                continue
            s = np.linspace(sec.s, sec.e, max(2, int(math.ceil((sec.e - sec.s) / step)) + 1))
            b = [r.borders_at(float(v), sec) for v in s]
            for l in sec.lanes():
                g = next((k for k, v in GROUPS.items() if l.type in v), None)
                if g is None:
                    continue
                ti = np.array([x[l.id][0] for x in b])
                to = np.array([x[l.id][1] for x in b])
                if np.max(np.abs(to - ti)) < 0.05:
                    continue
                x1, y1 = r.point(s, ti)
                x2, y2 = r.point(s, to)
                ring = list(zip(np.concatenate([x1, x2[::-1]]), np.concatenate([y1, y2[::-1]])))
                p = Polygon(ring)
                if not p.is_valid:
                    p = p.buffer(0)
                if p.area > 0.01:
                    out[g].append(p)
    return out


def measure(osm_path, xodr_path, clip=None):
    od = G.OpenDrive(xodr_path)
    osm = G.Osm(osm_path, od.lat0, od.lon0)
    box = None
    if clip:
        la0, lo0, la1, lo1 = (float(v) for v in clip.split(","))
        x0, y0 = G.tm_forward(la0, lo0, od.lat0, od.lon0)
        x1, y1 = G.tm_forward(la1, lo1, od.lat0, od.lon0)
        box = (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
    polys = street_side_polygons(osm, box)
    lanes = lane_polygons(od)
    trees = {g: (STRtree(v), v) for g, v in lanes.items() if v}
    rows = {}
    for wid, p in polys.items():
        row = {"area": round(p.area, 1)}
        for g in GROUPS:
            if g not in trees:
                row[g] = 0.0
                continue
            tree, geoms = trees[g]
            hits = [geoms[i] for i in tree.query(p)]
            row[g] = round(unary_union([h.intersection(p) for h in hits]).area / p.area, 3) if hits else 0.0
        rows[wid] = row
    n = len(rows)
    summary = {"polygons": n, "covered": sum(r["parking"] >= COVERED for r in rows.values()),
               "mean_parking_share": round(float(np.mean([r["parking"] for r in rows.values()])), 3) if n else None,
               "mean_driving_share": round(float(np.mean([r["driving"] for r in rows.values()])), 3) if n else None,
               "mean_walking_share": round(float(np.mean([r["walking"] for r in rows.values()])), 3) if n else None,
               "parking_lanes": len(lanes.get("parking", []))}
    return summary, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("osm")
    ap.add_argument("xodr")
    ap.add_argument("--clip")
    ap.add_argument("--json")
    a = ap.parse_args()
    summary, rows = measure(a.osm, a.xodr, a.clip)
    print(json.dumps(summary))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"summary": summary, "polygons": rows}, f, indent=1)


if __name__ == "__main__":
    main()
