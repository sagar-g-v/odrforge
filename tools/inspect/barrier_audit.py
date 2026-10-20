"""Audit barrier objects (OSM guard rails, jersey and cable barriers) against OpenDRIVE 1.8.0 §13.14.1 Table 108 and
§13.4 (repeating objects), the 1.9.0 rule road.object.repeating.valid_s_length, the road's lanes and, with --osm, the
OSM ways they come from.

Per <object type="barrier">:
  kind           Table 108 subtype (hedge, guardRail, jerseyBarrier, wall, railing, fence, noiseProtections, other),
                 orientation none
  repeat         one or more <repeat>, each with distance 0 (Table 108: "<repeat> with @distance>0 not allowed"),
                 length > 0, and its size (heightStart/End, widthStart/End > 0: "size is defined in <repeat>")
  on_road        1.9.0 road.object.repeating.valid_s_length: 0 <= s and s + length <= the road length
  continuous     one barrier: each repeat starts where the one before ends (s, t, zOffset), and the object's s, t,
                 zOffset are those of its first repeat
  no_object_box  no length / width / height / radius on the object (Table 108: a bounding box "not recommended")
  clear_of_lanes along every repeat (0.5 m steps, t / width interpolated linearly, §13.4) the footprint
                 [t - width/2, t + width/2] overlaps no lane other than shoulder, border, curb, median, none by more
                 than TOL m (a barrier never blocks a lane anyone travels or parks on)
  clear_of_other_roads  the same at every 1 m point for every other road (connecting roads too) whose surface there
                 is within OTHER_LEVEL m in height (roads further apart pass over or under the barrier)
  alongside      the footprint lies at most REACH + TOL m beyond the road's outermost lane edge
  ground         the barrier's bottom (zOffset, relative to the reference line elevation, 1.8.0 §13.1) is between the
                 ground and GROUND_MAX m above it; the ground at t is the cross slope (1.8.0 §10.5.2 superelevation)
                 plus the height (§11.6) of the lane under the footprint or, beyond the road, the outer height of the
                 outermost lane on that side
  follows_osm    (with --osm) every 1 m point of the barrier lies within OSM_TOL m of an OSM way of its kind
                 (barrier=guard_rail for guardRail, jersey_barrier for jerseyBarrier, cable_barrier for other)
usage: python barrier_audit.py file.xodr [--osm file.osm]
"""
import argparse
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402

SUBTYPES = {"hedge", "guardRail", "jerseyBarrier", "wall", "railing", "fence", "noiseProtections", "other"}
MAY_STAND_ON = {"shoulder", "border", "curb", "median", "none"}
OSM_KIND = {"guardRail": "guard_rail", "jerseyBarrier": "jersey_barrier", "other": "cable_barrier"}
TOL = 0.06        # m: the converter's chord tolerance (0.05) plus rounding
REACH = 5.0       # m
GROUND_MAX = 0.5  # m
OSM_TOL = 0.2     # m
OTHER_LEVEL = 2.5  # m
REPEAT_ATTRS = ("s", "length", "distance", "tStart", "tEnd", "zOffsetStart", "zOffsetEnd", "heightStart", "heightEnd",
                "widthStart", "widthEnd")


def _num(el, k, d=0.0):
    v = el.get(k)
    return float(v) if v is not None else d


def _lane_heights(road_el):
    """{(section s, lane id): [(sOffset, inner, outer), ...]} from <height> (1.8.0 §11.6: a record holds until the next)"""
    out = {}
    for sec in road_el.find("lanes").findall("laneSection"):
        for side in ("left", "right"):
            se = sec.find(side)
            for l in (se.findall("lane") if se is not None else []):
                hs = sorted((_num(h, "sOffset"), _num(h, "inner"), _num(h, "outer")) for h in l.findall("height"))
                if hs:
                    out[(float(sec.get("s")), int(l.get("id")))] = hs
    return out


def _height(heights, sec, lid, ds, f):
    inner = outer = 0.0
    for so, a, b in heights.get((sec.s, lid), []):
        if so <= ds + 1e-9:
            inner, outer = a, b
    return inner + (outer - inner) * min(max(f, 0.0), 1.0)


def _osm_segments(osm_path, od):
    """{osm barrier value: (A, B) segment arrays} in the file's local frame"""
    osm = G.Osm(osm_path, od.lat0, od.lon0)
    segs = defaultdict(list)
    for wid, (_, tags) in osm.ways.items():
        b = tags.get("barrier")
        if b not in OSM_KIND.values():
            continue
        P = osm.polyline(wid)
        for k in range(len(P) - 1):
            segs[b].append((P[k], P[k + 1]))
    return {b: (np.array([a for a, _ in v]), np.array([c for _, c in v])) for b, v in segs.items()}


def _dist_to_segments(p, AB):
    A, B = AB
    d = B - A
    L2 = np.maximum((d ** 2).sum(1), 1e-12)
    u = np.clip(((p - A) * d).sum(1) / L2, 0, 1)
    return float(np.sqrt((((A + d * u[:, None]) - p) ** 2).sum(1)).min())


def audit(path, osm=None, quiet=False):
    """{"barriers": n, "fails": Counter, "by_object": {object id: Counter}, "stats": {...}}"""
    od = G.OpenDrive(path)
    osm_segs = _osm_segments(osm, od) if osm else None
    fails, by_object = Counter(), defaultdict(Counter)
    stats = {"length": 0.0, "repeats": 0, "max_overlap": 0.0, "max_beyond": 0.0, "min_ground": 1e9,
             "max_ground": -1e9, "max_osm": 0.0}
    n = 0
    sups = {}  # road id -> superelevation records

    def add(check, oid):
        fails[check] += 1
        by_object[oid][check] += 1

    for r in od.roads.values():
        objs = r.el.find("objects")
        if objs is None:
            continue
        sup = sorted((float(e.get("s")), G._abcd(e)) for e in r.el.iter("superelevation"))
        heights = _lane_heights(r.el)
        for o in objs.findall("object"):
            if o.get("type") != "barrier":
                continue
            n += 1
            oid = o.get("id")
            sub = o.get("subtype", "")
            if sub not in SUBTYPES or o.get("orientation", "none") != "none":
                add("kind", oid)
            reps = [{k: _num(e, k, float("nan")) for k in REPEAT_ATTRS} for e in o.findall("repeat")]
            if not reps or any(not (rp["distance"] == 0 and rp["length"] > 0 and rp["heightStart"] > 0 and
                                    rp["heightEnd"] > 0 and rp["widthStart"] > 0 and rp["widthEnd"] > 0) for rp in reps):
                add("repeat", oid)
            if any(not (rp["s"] >= -1e-6 and rp["s"] + rp["length"] <= r.length + 1e-6) for rp in reps):
                add("on_road", oid)
            joins = all(abs(a["s"] + a["length"] - b["s"]) < 1e-6 and abs(a["tEnd"] - b["tStart"]) < 1e-6 and
                        abs(a["zOffsetEnd"] - b["zOffsetStart"]) < 1e-6 for a, b in zip(reps, reps[1:]))
            if reps and not (joins and abs(_num(o, "s") - reps[0]["s"]) < 1e-6 and abs(_num(o, "t") - reps[0]["tStart"]) < 1e-6
                             and abs(_num(o, "zOffset") - reps[0]["zOffsetStart"]) < 1e-6):
                add("continuous", oid)
            if any(o.get(k) is not None for k in ("length", "width", "height", "radius")):
                add("no_object_box", oid)
            bad = Counter()
            for rp in reps:
                if not (rp["length"] > 0) or any(math.isnan(v) for v in rp.values()):
                    continue
                stats["length"] += rp["length"]
                stats["repeats"] += 1
                for ds in np.append(np.arange(0.0, rp["length"], 0.5), rp["length"]):
                    s = rp["s"] + ds
                    if s < 0 or s > r.length:
                        continue
                    u = ds / rp["length"]
                    t = rp["tStart"] + u * (rp["tEnd"] - rp["tStart"])
                    w = rp["widthStart"] + u * (rp["widthEnd"] - rp["widthStart"])
                    z = rp["zOffsetStart"] + u * (rp["zOffsetEnd"] - rp["zOffsetStart"])
                    f0, f1 = t - w / 2, t + w / 2
                    sec = r.section_at(s)
                    borders = r.borders_at(s, sec)
                    lo = min(min(v) for v in borders.values())
                    hi = max(max(v) for v in borders.values())
                    on_h, on_lane, beyond_h, outermost = 0.0, False, 0.0, -1e9
                    for l in sec.lanes():
                        a, b = borders[l.id]
                        if abs(b - a) < 1e-6:
                            continue
                        ov = min(f1, max(a, b)) - max(f0, min(a, b))
                        if ov > 1e-6:
                            on_lane = True
                            on_h = max(on_h, _height(heights, sec, l.id, s - sec.s, (t - a) / (b - a)))
                            if l.type not in MAY_STAND_ON:
                                stats["max_overlap"] = max(stats["max_overlap"], ov)
                                if ov > TOL:
                                    bad["clear_of_lanes"] = 1
                        reach = b if b > a else -b
                        if (t - b) * (b - a) > 0 and reach > outermost:
                            outermost, beyond_h = reach, _height(heights, sec, l.id, s - sec.s, 1.0)
                    beyond = f0 - hi if t > hi else lo - f1 if t < lo else 0.0
                    stats["max_beyond"] = max(stats["max_beyond"], beyond)
                    if beyond > REACH + TOL:
                        bad["alongside"] = 1
                    ground = t * math.tan(float(r._piecewise(sup, np.array([s]))[0])) + (on_h if on_lane else beyond_h)
                    stats["min_ground"] = min(stats["min_ground"], z - ground)
                    stats["max_ground"] = max(stats["max_ground"], z - ground)
                    if not (-0.01 <= z - ground <= GROUND_MAX):
                        bad["ground"] = 1
                    if abs(ds - round(ds)) < 1e-9:
                        x, y = (float(v[0]) for v in r.point(np.array([s]), np.array([t])))
                        z_here = float(r.z(np.array([s]))[0]) + t * math.tan(float(r._piecewise(sup, np.array([s]))[0]))
                        for q in od.roads_near(x, y, 30.0):
                            if q is r:
                                continue
                            qs, qt, _ = q.project(x, y)
                            if qs <= 1e-3 or qs >= q.length - 1e-3:
                                continue
                            qx, qy, qh = (float(v[0]) for v in q.ref(np.array([qs])))
                            if abs((x - qx) * math.cos(qh) + (y - qy) * math.sin(qh)) > 0.5:
                                continue
                            if q.id not in sups:
                                sups[q.id] = sorted((float(e.get("s")), G._abcd(e)) for e in q.el.iter("superelevation"))
                            zq = float(q.z(np.array([qs]))[0]) + qt * math.tan(float(q._piecewise(sups[q.id], np.array([qs]))[0]))
                            if abs(zq - z_here) > OTHER_LEVEL:
                                continue
                            qb = q.borders_at(qs)
                            for l in q.section_at(qs).lanes():
                                a2, b2 = min(qb[l.id]), max(qb[l.id])
                                if l.type not in MAY_STAND_ON and b2 - a2 > 1e-6 and min(qt + w / 2, b2) - max(qt - w / 2, a2) > TOL:
                                    bad["clear_of_other_roads"] = 1
                    if osm_segs is not None and abs(ds - round(ds)) < 1e-9:
                        AB = osm_segs.get(OSM_KIND.get(sub, ""))
                        x, y = r.point(np.array([s]), np.array([t]))
                        d = _dist_to_segments(np.array([float(x[0]), float(y[0])]), AB) if AB is not None else 1e9
                        stats["max_osm"] = max(stats["max_osm"], d)
                        if d > OSM_TOL:
                            bad["follows_osm"] = 1
            for check in bad:
                add(check, oid)
    if not quiet:
        print(f"{n} barriers ({stats['length']:.0f} m, {stats['repeats']} repeats); fails {dict(fails)}; "
              f"max lane overlap {stats['max_overlap']:.3f} m; max beyond edge {stats['max_beyond']:.2f} m; "
              f"bottom above ground {stats['min_ground'] if n else 0:.3f}..{stats['max_ground'] if n else 0:.3f} m"
              + (f"; max distance to OSM {stats['max_osm']:.3f} m" if osm else ""))
    return {"barriers": n, "fails": fails, "by_object": dict(by_object), "stats": stats}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr", nargs="+")
    ap.add_argument("--osm")
    a = ap.parse_args()
    for p in a.xodr:
        audit(p, a.osm)
