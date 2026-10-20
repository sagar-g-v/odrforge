"""Audit direct and virtual junctions and lane height records against OpenDRIVE 1.9 rules none of the other
checkers measure (rule ids verbatim from the spec):

  junctions.direct.road_connectivity   (1.8.0, §12.6) "Direct junctions shall connect one road on one side with
                                       multiple roads on the other side."
  junctions.direct.split_or_merge      (1.8.0, §12.6) "... only be used for splitting or merging roads without
                                       crossing traffic." - measured: within a connection the lane links keep their
                                       order (no two links cross), and no two connections link the same lane
  junctions.direct.road_ramp_heading   (1.7.0, §12.6) "The junction shall be placed where the headings of road, ramp,
                                       or slip lane are identical." - heading of incoming and linked road at the joint
  junctions.direct.overlap_zone_*      (1.8.0, §12.6) overlapZone only in direct junctions
  junctions.common.virtual_junction_attributes (1.9.0, §12.2) mainRoad / orientation / sStart / sEnd only on virtual
  junctions.virtual.no_controllers     (1.9.0, §12.7) "Virtual junctions shall not have controllers ..."
  1.8.0 §12.1 Table 54                 junctions with traffic lights are common: no controller in a direct junction
  junctions.virtual.only_one_start_end (1.9.0, §12.7) one sStart, one sEnd: here sStart <= sEnd within the main road
  road.lane.height.elem_asc_order      (§11.7) "<height> elements shall be defined in ascending order according to the
                                       s-coordinate."
usage: python junction_rules_audit.py file.xodr [...]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from invariants import Road  # noqa: E402


def contact_heading(road, at_start):
    """heading of travel along +s at the road end"""
    _, _, h, _, _ = road.pose(0.0 if at_start else road.length, "right" if at_start else "left")
    return float(h[0])


def joint_end(road, jid):
    """True: road joins the junction at its start, False: at its end, None: not linked"""
    if road.pred is not None and road.pred.type == "junction" and road.pred.id == jid:
        return True
    if road.succ is not None and road.succ.type == "junction" and road.succ.id == jid:
        return False
    return None


def audit(path):
    root = ET.parse(path).getroot()
    raw = {r.get("id"): r for r in root.findall("road")}
    roads = {}

    def road(rid):
        if rid not in roads:
            roads[rid] = Road(raw[rid])
        return roads[rid]

    fails, worst_heading, n_direct, n_virtual = Counter(), 0.0, 0, 0
    for j in root.findall("junction"):
        jt, jid = j.get("type", "default"), j.get("id")
        if jt != "virtual" and any(j.get(a) is not None for a in ("mainRoad", "orientation", "sStart", "sEnd")):
            fails["junctions.common.virtual_junction_attributes"] += 1
        if jt != "direct" and any(l.get("overlapZone") is not None for l in j.iter("laneLink")):
            fails["junctions.common.direct_junction_attributes"] += 1
        if jt == "virtual":
            n_virtual += 1
            if j.find("controller") is not None:
                fails["junctions.virtual.no_controllers"] += 1
            m = j.get("mainRoad")
            if m not in raw or not (0.0 <= float(j.get("sStart")) <= float(j.get("sEnd")) <= road(m).length + 1e-6):
                fails["junctions.virtual.only_one_start_end(range)"] += 1
        if jt != "direct":
            continue
        n_direct += 1
        # OpenDRIVE 1.8.0 §12.1 Table 54: junctions with traffic lights -> common
        if j.find("controller") is not None:
            fails["junctions.direct.traffic_lights(1.8.0 Table 54)"] += 1
        conns = j.findall("connection")
        pairs = [(c.get("incomingRoad"), c.get("linkedRoad")) for c in conns]
        # one road on one side, several on the other: one road in every connection, the others distinct
        common = set(pairs[0]) if pairs else set()
        for p in pairs[1:]:
            common &= set(p)
        others = {x for p in pairs for x in p} - common
        if len(common) != 1 or len(others) < 2 or len(others) != len(pairs):
            fails["junctions.direct.road_connectivity"] += 1
        # no crossing traffic: lane links of one connection keep their order - lane ids keep their order where the
        # two roads run the same way at the joint (one joins at its end, the other at its start), else reverse it;
        # no lane of the common road linked twice
        used = Counter()
        for c in conns:
            links = sorted((int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink"))
            tos = [t for _, t in links]
            ea, eb = joint_end(road(c.get("incomingRoad")), jid), joint_end(road(c.get("linkedRoad")), jid)
            if ea is not None and eb is not None and tos != sorted(tos, reverse=(ea == eb)):
                fails["junctions.direct.split_or_merge(crossing links)"] += 1
            trunk_is_incoming = c.get("incomingRoad") in common
            for f, t in links:
                used[f if trunk_is_incoming else t] += 1
        if any(v > 1 for v in used.values()):
            fails["junctions.direct.split_or_merge(lane linked twice)"] += 1
        # headings equal at the joint
        for c in conns:
            a, b = c.get("incomingRoad"), c.get("linkedRoad")
            ea, eb = joint_end(road(a), jid), joint_end(road(b), jid)
            if ea is None or eb is None:
                fails["direct.link_missing"] += 1
                continue
            # travel out of a through its joint end and into b from its joint end must be the same direction
            ha = contact_heading(road(a), ea) + (math.pi if ea else 0.0)
            hb = contact_heading(road(b), eb) + (0.0 if eb else math.pi)
            d = abs((ha - hb + math.pi) % (2 * math.pi) - math.pi)
            worst_heading = max(worst_heading, d)
            if d > 1e-3:
                fails["junctions.direct.road_ramp_heading"] += 1
    # lane height records ascending
    for r in raw.values():
        for lane in r.iter("lane"):
            hs = [float(h.get("sOffset")) for h in lane.findall("height")]
            if hs != sorted(hs):
                fails["road.lane.height.elem_asc_order"] += 1
    print(f"{path}: direct {n_direct}, virtual {n_virtual}; fails {dict(fails)}; worst direct heading {worst_heading:.2e} rad")


for p in sys.argv[1:]:
    audit(p)
