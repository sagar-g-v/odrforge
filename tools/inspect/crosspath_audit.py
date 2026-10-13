"""Audit every <crossPath> of an OpenDRIVE file against the rules of OpenDRIVE 1.9 section 12.5 and 12.7.1.

Rules measured (rule ids verbatim from the spec):
  junctions.cross_path.correct_junction_id       crossing road @junction == the junction's id
  junctions.cross_path.only_connect_correct_type linked lanes (from) and crossing lanes (to) walking / biking
  junctions.cross_path.lane_linkage              start / end of the crossing road reach the linked lanes
  junctions.cross_path.start_end_contained       start / end points of the crossing road and its lanes fully
                                                 contained within the linked lanes
  junctions.virtual.crossPath.cross_road_check_s_t  crossing road within the s and t range of the main road
  junctions.cross_path.within_junction_area      (virtual) crossing road within sStart .. sEnd
usage: python crosspath_audit.py file.xodr [...]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from invariants import Road  # noqa: E402

EPS = 1e-6


def to_st(road, x, y, s_guess):
    """(s, t) of point x, y on road by Newton on the reference line from s_guess"""
    s = s_guess
    for _ in range(30):
        px, py, h, _, _ = (float(v[0]) for v in road.pose(s))
        dx, dy = x - px, y - py
        ds = dx * math.cos(h) + dy * math.sin(h)
        s = min(max(s + ds, 0.0), road.length)
        if abs(ds) < 1e-9:
            break
    px, py, h, _, _ = (float(v[0]) for v in road.pose(s))
    return s, -(x - px) * math.sin(h) + (y - py) * math.cos(h)


def lane_span(road, s, lane_id):
    """(t_lo, t_hi, type) of lane lane_id at s"""
    k = road.section_index(s)
    b = road.borders(k, s)
    if lane_id not in b:
        return None
    t_in, t_out = float(b[lane_id][0][0]), float(b[lane_id][1][0])
    lane = next(l for l in road.sections[k].side(1 if lane_id > 0 else -1) if l.id == lane_id)
    return min(t_in, t_out), max(t_in, t_out), lane.type


def outer_span(road, s):
    k = road.section_index(s)
    ts = [float(v[j][0]) for v in road.borders(k, s).values() for j in (0, 1)]
    return min(ts), max(ts)


def audit(path):
    root = ET.parse(path).getroot()
    roads = {r.get("id"): Road(r) for r in root.findall("road")}
    raw = {r.get("id"): r for r in root.findall("road")}
    fails, margins, n = Counter(), [], 0
    for j in root.findall("junction"):
        for cp in j.findall("crossPath"):
            n += 1
            x = roads[cp.get("crossingRoad")]
            if x.junction != j.get("id"):
                fails["correct_junction_id"] += 1
            # the crossing road's lanes and their lateral extent (its footprint along the main road)
            sec = x.sections[0]
            xl = list(sec.side(-1)) + list(sec.side(1))
            if any(l.type not in ("walking", "biking", "sidewalk") for l in xl):
                fails["only_connect_correct_type(to)"] += 1
            b0, b1 = x.borders(0, 0.0), x.borders(0, x.length, "left")
            for tag, rid, at_s in (("startLaneLink", cp.get("roadAtStart"), 0.0), ("endLaneLink", cp.get("roadAtEnd"), x.length)):
                ll = cp.find(tag)
                R, frm, s_link = roads[rid], int(ll.get("from")), float(ll.get("s"))
                bb = b0 if at_s == 0.0 else b1
                ts = [float(v[j_][0]) for v in bb.values() for j_ in (0, 1)]
                corners = []
                for t in (min(ts), max(ts)):
                    px, py, h, _, _ = (float(v[0]) for v in x.pose(at_s, "right" if at_s == 0.0 else "left"))
                    corners.append(to_st(R, px - t * math.sin(h), py + t * math.cos(h), s_link))
                sp = [lane_span(R, s_, frm) for s_, _ in corners]
                if any(v is None for v in sp):
                    fails["lane_missing"] += 1
                    continue
                if any(v[2] not in ("walking", "biking", "sidewalk") for v in sp):
                    fails["only_connect_correct_type(from)"] += 1
                # margin: how far inside the linked lane each corner lies laterally (negative = outside)
                m = min(min(t_ - lo, hi - t_) for (s_, t_), (lo, hi, _) in zip(corners, sp))
                margins.append(m)
                if m < -1e-3:
                    fails["lane_linkage"] += 1
                if m <= 1e-3:
                    fails["start_end_contained"] += 1
                # main road s / t range
                for s_, t_ in corners:
                    lo, hi = outer_span(R, s_)
                    if not (-EPS <= s_ <= R.length + EPS and lo - 1e-3 <= t_ <= hi + 1e-3):
                        fails["cross_road_check_s_t"] += 1
                    if j.get("type") == "virtual" and not (float(j.get("sStart")) - 1e-3 <= s_ <= float(j.get("sEnd")) + 1e-3):
                        fails["within_junction_area(sStart..sEnd)"] += 1
    m = np.array(margins) if margins else np.zeros(1)
    print(f"{path}: {n} cross paths; fails {dict(fails)}; end margin inside linked lane min {m.min():.3f} "
          f"median {np.median(m):.3f} m")


for p in sys.argv[1:]:
    audit(p)
