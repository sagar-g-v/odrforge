"""junction_audit - check an .xodr's junctions against the ASAM OpenDRIVE junction rules
(docs/reference/opendrive_junction_rules.md). Every check is a rule of the specification, reported by
its rule id with the junction / road / lane it concerns.

    python junction_audit.py out.xodr [--json out.json]
"""
import argparse
import collections
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__file__))
import odrgeom as G  # noqa: E402

DRIVABLE = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional"}


def audit(path):
    od = G.OpenDrive(path)
    root = ET.parse(path).getroot()
    out = collections.defaultdict(list)
    jids = [j.get("id") for j in root.findall("junction")]
    for jid, n in collections.Counter(jids).items():
        if n > 1:
            out["junction.id_unique"].append({"junction": jid, "count": n})
    jel = {j.get("id"): j for j in root.findall("junction")}
    # road ends attached to each junction, and the connecting roads it owns
    ends = collections.defaultdict(set)
    for r in od.plain_roads():
        for lk, end in ((r.pred, "start"), (r.succ, "end")):
            if lk is not None and lk.type == "junction":
                ends[lk.id].add((r.id, end))
                if lk.contact:
                    out["road.linkage.junc_link_attribute_usage"].append({"road": r.id, "end": end, "junction": lk.id})
                if lk.id not in jel:
                    out["road.linkage.target_exists"].append({"road": r.id, "junction": lk.id})
    conn_roads = collections.defaultdict(list)
    for r in od.connecting_roads():
        conn_roads[r.junction].append(r)
    for jid, j in jel.items():
        jt = j.get("type", "default")
        conns = j.findall("connection")
        # junctions.common.not_only_two: a junction joins three or more road ends
        if jt == "default" and len(ends[jid]) <= 2:
            out["junctions.common.not_only_two"].append({"junction": jid, "ends": sorted(ends[jid])})
        seen = collections.Counter()
        ids = collections.Counter(c.get("id") for c in conns)
        for cid, n in ids.items():
            if n > 1:
                out["junctions.connection.id_unique"].append({"junction": jid, "connection": cid})
        for c in conns:
            inc, cr, cp = c.get("incomingRoad"), c.get("connectingRoad"), c.get("contactPoint")
            seen[(inc, cr)] += 1
            if cr is None or cr not in od.roads:
                out["junctions.connection.connecting_road_exists"].append({"junction": jid, "connection": c.get("id")})
                continue
            con = od.roads[cr]
            if con.junction != jid:
                out["junctions.connection.connecting_road_in_junction"].append({"junction": jid, "road": cr})
            if (inc, "start") not in ends[jid] and (inc, "end") not in ends[jid]:
                out["junctions.connection.incoming_links_to_junction"].append({"junction": jid, "incoming": inc})
            # contactPoint: the connecting road end that touches the incoming road
            near = con.pred if cp == "start" else con.succ
            if near is None or near.type != "road" or near.id != inc:
                out["junctions.connection.contact_point_matches"].append({"junction": jid, "connecting": cr, "incoming": inc, "contactPoint": cp})
            links = c.findall("laneLink")
            if not links:
                out["junctions.connection.laneLink_present"].append({"junction": jid, "connecting": cr})
            inc_road = od.roads.get(inc)
            for ll in links:
                fr, to = int(ll.get("from")), int(ll.get("to"))
                if inc_road is None:
                    continue
                end = "end" if (inc, "end") in ends[jid] else "start"
                sec = inc_road.sections[-1] if end == "end" else inc_road.sections[0]
                lane = sec.lane(fr)
                if lane is None:
                    out["junctions.laneLink.from_exists"].append({"junction": jid, "incoming": inc, "from": fr})
                csec = con.sections[0] if cp == "start" else con.sections[-1]
                if csec.lane(to) is None:
                    out["junctions.laneLink.to_exists"].append({"junction": jid, "connecting": cr, "to": to})
        for (inc, cr), n in seen.items():
            if n > 1:
                out["junctions.connection.one_link_to_incoming"].append({"junction": jid, "incoming": inc, "connecting": cr})
    # connecting roads shall not be incoming roads of a junction
    for r in od.connecting_roads():
        for lk in (r.pred, r.succ):
            if lk is not None and lk.type == "junction":
                out["junctions.connection.connect_road_no_incoming_road"].append({"road": r.id, "junction": lk.id})
    # road.lane.link.no_link: lanes ending in a junction carry no link toward it
    for r in od.plain_roads():
        for lk, sec, attr in ((r.pred, r.sections[0], "pred"), (r.succ, r.sections[-1], "succ")):
            if lk is not None and lk.type == "junction":
                for l in sec.lanes():
                    if getattr(l, attr) is not None:
                        out["road.lane.link.no_link"].append({"road": r.id, "lane": l.id, "junction": lk.id})
    # every drivable lane entering a junction has a laneLink (else it has no path through it)
    linked = collections.defaultdict(set)
    for jid, j in jel.items():
        for c in j.findall("connection"):
            for ll in c.findall("laneLink"):
                linked[(jid, c.get("incomingRoad"))].add(int(ll.get("from")))
    rule = {}
    for r in od.plain_roads():
        rule_lht = r.el.get("rule") == "LHT"
        for lk, sec, end in ((r.pred, r.sections[0], "start"), (r.succ, r.sections[-1], "end")):
            if lk is None or lk.type != "junction":
                continue
            for l in sec.lanes():
                if l.type not in DRIVABLE or l.type == "bidirectional":
                    continue
                # lanes travelling into the junction: RHT right lanes at the end, left lanes at the start
                into = (l.id < 0) == (end == "end")
                if rule_lht:
                    into = not into
                ds = (r.length - sec.s) if end == "end" else 0.0
                if into and float(l.width(np.array([ds]))[0]) > 1e-3 and l.id not in linked[(lk.id, r.id)]:
                    out["junctions.laneLink.entering_lane_without_path"].append({"junction": lk.id, "road": r.id, "lane": l.id})
    # junctions.no_overlap: connecting-road footprints of two junctions intersect (sampled reference
    # lines in 1 m cells) on the same level. Junctions stacked above each other (interchanges) do not
    # overlap: OpenDRIVE 1.9 road.overlap_inside_junction excludes roads on different levels.
    same_level = 3.0  # m, below any vertical clearance
    cells = collections.defaultdict(list)
    for jid, rs in conn_roads.items():
        for r in rs:
            if not any(l.type in DRIVABLE for s in r.sections for l in s.lanes()):
                continue
            for s in np.linspace(0, r.length, max(2, int(r.length / 1.0))):
                x, y, _ = r.ref([s])
                cells[(int(x[0] // 1.0), int(y[0] // 1.0))].append((jid, float(r.z(np.array([s]))[0])))
    pairs = set()
    for samples in cells.values():
        for ja, za in samples:
            for jb, zb in samples:
                if ja < jb and abs(za - zb) < same_level:
                    pairs.add((ja, jb))
    for a, b in sorted(pairs):
        out["junctions.no_overlap"].append({"junctions": [a, b]})
    return {"file": path, "junctions": len(jel), "counts": {k: len(v) for k, v in sorted(out.items())}, "instances": out}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("--json")
    a = ap.parse_args()
    res = audit(a.xodr)
    print(f"{res['junctions']} junctions")
    for k, n in res["counts"].items():
        print(f"  {n:6d}  {k}")
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
