#!/usr/bin/env python3
"""laneflow_plant - planted-defect self test for laneflow.py.

Copies a frozen OpenDRIVE file, plants one known defect per copy (python xml), runs laneflow on the
original and on every copy, and diffs the findings: each planted defect must appear in its expected
invariant(s) at the planted road / lane / s, and no new *defect* finding may appear anywhere else
(cascades named in the expectation are allowed; they are listed separately).

    python laneflow_plant.py --base munich_base.xodr --osm munich.osm --clip 48.132,11.560,48.145,11.585 \
        --out C:/mywork/osm2carla/build/tmp/spec/laneflow/planted

Plants (each documented in PLANTS): junction laneLink deleted, junction connection deleted, road
successor link removed, lane successor link removed (section boundary), lane width shifted by 0.5 m
at a section start (geometry: laneflow must stay silent), spiral curvature jump (geometry: silent),
sidewalk cut out of a middle section, connecting-road lane id sign flipped, laneLink 'from' sign
flipped, road split needlessly (contact start) and split + second half reversed (contact end), a
driving lane set to zero width, a lane narrowed below any car, a plain road deleted, and the same with a
lane-less road (no surface) left on its reference line.
"""
import argparse
import copy
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import laneflow as LF  # noqa: E402
import odrgeom as G  # noqa: E402


# ----------------------------------------------------------------------------- xml helpers
def roads_by_id(root):
    return {r.get("id"): r for r in root.findall("road")}


def secs(road_el):
    return sorted(road_el.find("lanes").findall("laneSection"), key=lambda e: float(e.get("s")))


def lane_el(sec_el, lid):
    for l in sec_el.iter("lane"):
        if int(l.get("id")) == lid:
            return l
    return None


def set_link(parent, tag, etype, eid, contact=None):
    lk = parent.find("link")
    if lk is None:
        lk = ET.SubElement(parent, "link")
        parent.remove(lk)
        parent.insert(0, lk)
    old = lk.find(tag)
    if old is not None:
        lk.remove(old)
    e = ET.Element(tag, {"elementType": etype, "elementId": eid} if etype else {"id": str(eid)})
    if contact:
        e.set("contactPoint", contact)
    if tag == "predecessor":
        lk.insert(0, e)
    else:
        lk.append(e)


def set_lane_link(lane, tag, lid):
    lk = lane.find("link")
    if lk is None:
        lk = ET.Element("link")
        lane.insert(0, lk)
    old = lk.find(tag)
    if old is not None:
        lk.remove(old)
    if lid is None:
        return
    e = ET.Element(tag, {"id": str(lid)})
    if tag == "predecessor":
        lk.insert(0, e)
    else:
        lk.append(e)


def shift_poly(a, b, c, d, s0):
    """re-expand a + b u + c u^2 + d u^3 around u = s0."""
    return (a + b * s0 + c * s0 ** 2 + d * s0 ** 3, b + 2 * c * s0 + 3 * d * s0 ** 2, c + 3 * d * s0, d)


def _abcd(e):
    return tuple(float(e.get(k, 0)) for k in ("a", "b", "c", "d"))


def _set_abcd(e, v):
    for k, x in zip("abcd", v):
        e.set(k, repr(float(x)))


def cut_records(parent, tag, s_key, s0, s1, shift):
    """keep records of [s0, s1) re-expanded so the record active at s0 starts there; shift s by -shift."""
    recs = sorted(parent.findall(tag), key=lambda e: float(e.get(s_key)))
    keep = []
    for i, e in enumerate(recs):
        a = float(e.get(s_key))
        b = float(recs[i + 1].get(s_key)) if i + 1 < len(recs) else 1e18
        if b <= s0 + 1e-9 or a >= s1 - 1e-9:
            parent.remove(e)
            continue
        if a < s0:
            _set_abcd(e, shift_poly(*_abcd(e), s0 - a))
            e.set(s_key, repr(s0 - shift))
        else:
            e.set(s_key, repr(a - shift))
        keep.append(e)
    return keep


# ----------------------------------------------------------------------------- plants
class Plant:
    def __init__(self, name, desc, expect, allowed=(), silent=False):
        self.name, self.desc = name, desc
        self.expect = expect      # [(invariant prefix, road, lane or None, s or None)]
        self.allowed = allowed    # invariant prefixes allowed to change (cascades)
        self.silent = silent      # geometry plant: laneflow must report no change at all


def choose(lf):
    """pick plant targets on the baseline network (deterministic)."""
    net, g = lf.net, lf.motor
    R = net.roads
    t = {}
    plain = sorted([r for r in R if net.is_plain(r)], key=LF._rid_key)
    # a junction laneLink whose incoming lane has other connections (so only the connecting lane loses
    # its feed): deleting it must orphan exactly that connecting lane
    for jid in sorted(net.junctions, key=LF._rid_key):
        conns = net.junctions[jid]["conns"]
        for c in conns:
            C = R.get(c["connecting"] or "")
            if C is None or len(C.sections) != 1 or not c["lanes"]:
                continue
            f, to = c["lanes"][0]
            other = [cc for cc in conns if cc is not c and cc["incoming"] == c["incoming"]
                     and any(ff == f for ff, _ in cc["lanes"])]
            fed = [cc for cc in conns if cc["connecting"] == c["connecting"] and any(tt == to for _, tt in cc["lanes"])]
            if other and len(fed) == 1 and sum(1 for _, tt in c["lanes"] if tt == to) == 1:
                t["laneLink_del"] = (jid, c["incoming"], C.id, (f, to))
                break
        if "laneLink_del" in t:
            break
    # unique road-level move realised by one connecting road, OSM has it, single-lane connection
    T = lf.T_lane
    via = {}
    for jid in sorted(net.junctions, key=LF._rid_key):
        for c in net.junctions[jid]["conns"]:
            C = R.get(c["connecting"] or "")
            if C is None or C.succ is None or C.succ.type != "road" or not net.is_plain(C.succ.id):
                continue
            via.setdefault((c["incoming"], C.succ.id), []).append((jid, c))
    for (a, b), lst in sorted(via.items(), key=lambda kv: (LF._rid_key(kv[0][0]), LF._rid_key(kv[0][1]))):
        if len(lst) != 1 or a == b:
            continue
        jid, c = lst[0]
        ra = R[a]
        moves = [m for m in lf.T_osm if m[0][0] == a and m[1][0] == b]
        if not moves or moves[0] not in T:
            continue
        # angle well below a U-turn
        (sa_, da), (sb_, db) = moves[0]
        if abs(_move_angle(R, moves[0])) > 120 or len(c["lanes"]) != 1:
            continue
        others = [cc for cc in net.by_incoming[(jid, a)] if cc is not c]
        if not others:
            continue
        t["conn_del"] = (jid, c["incoming"], c["connecting"], moves[0])
        break
    for rid in plain:
        r = R[rid]
        L = r.succ
        if "succ_break" not in t and L is not None and L.type == "junction" and len(r.sections) == 1 and \
                any(l.type == "driving" for l in r.sections[0].right) and \
                lf.ends[(rid, True)]["osm_node"] is not None and not lf.ends[(rid, True)]["boundary"] and \
                not lf.ends[(rid, True)].get("bjunc") and all(g.fwd[i] and g.bwd[i] for i, k in enumerate(g.key)
                                                               if k[0] == rid):
            t["succ_break"] = rid
        if "lane_succ_break" not in t and len(r.sections) >= 2:
            a, b = r.sections[0], r.sections[1]
            for l in a.right:
                if l.type == "driving" and l.succ is not None and b.lane(l.succ) is not None and \
                        b.lane(l.succ).type == "driving" and len([q for q in a.right if q.type == "driving"]) == 1:
                    t["lane_succ_break"] = (rid, l.id, a.e)
                    break
        if "sidewalk_cut" not in t and len(r.sections) >= 3:
            for k0 in range(len(r.sections) - 2):
                trio = r.sections[k0:k0 + 3]
                for sg in (1, -1):
                    ids = [max((abs(q.id) for q in (s_.left if sg > 0 else s_.right)), default=0) * sg for s_ in trio]
                    if ids[0] != 0 and len(set(ids)) == 1 and all(s_.lane(ids[0]).type == "sidewalk" for s_ in trio)                             and trio[0].lane(ids[0]).succ == ids[0] and trio[2].lane(ids[0]).pred == ids[0]:
                        t["sidewalk_cut"] = (rid, ids[0], k0, trio[1].s, trio[1].e)
                        break
                if "sidewalk_cut" in t:
                    break
        if "edge_unlink" not in t and len(r.sections) >= 2 and ("sidewalk_cut" not in t or t["sidewalk_cut"][0] != rid):
            a_, b_ = r.sections[0], r.sections[1]
            for q in a_.lanes():
                if q.type == "sidewalk" and q.succ is not None and b_.lane(q.succ) is not None and                         b_.lane(q.succ).type == "sidewalk":
                    t["edge_unlink"] = (rid, q.id, q.succ, a_.e)
                    break
        if "width_shift" not in t and len(r.sections) >= 2:
            b = r.sections[1]
            for l in b.right:
                if l.type == "driving":
                    t["width_shift"] = (rid, l.id, b.s)
                    break
        if "zero_width" not in t and len(r.sections) == 1:
            dr = [l for l in r.sections[0].right if l.type == "driving"]
            if len(dr) >= 2 and all(g.fwd[g.idx[(rid, 0, l.id, net.dirs(rid, 0, l)[0])]] for l in dr):
                t["zero_width"] = (rid, min(l.id for l in dr))
        if "split" not in t and len(r.sections) == 1 and r.pred is not None and r.succ is not None and \
                r.pred.type == "junction" and r.succ.type == "junction" and len(r.geoms) >= 3 and \
                all(gm[2] in ("line", "arc", "spiral") for gm in r.geoms) and \
                all(len(l.widths) == 1 and l.widths[0][1][1:] == (0, 0, 0) for l in r.sections[0].lanes()) and \
                all(o[1] == (0, 0, 0, 0) for o in r.offsets) and \
                sorted(l.id for l in r.sections[0].left) == sorted(-l.id for l in r.sections[0].right) and \
                all(r.sections[0].lane(l.id).type == r.sections[0].lane(-l.id).type for l in r.sections[0].left) and \
                r.length > 60:
            gs = [gm[0] for gm in r.geoms if 25 < gm[0] < r.length - 25]
            if gs:
                t["split"] = (rid, gs[0])
        if "narrow" not in t and len(r.sections) == 1 and r.pred is not None and r.succ is not None:
            dr = [l for l in r.sections[0].lanes() if l.type == "driving"]
            if len(dr) == 2 and {l.id for l in dr} == {1, -1}:
                t["narrow"] = (rid, -1)
        if "spiral_jump" not in t and len(r.geoms) >= 3 and r.geoms[1][2] == "spiral" and r.geoms[1][1] > 30:
            t["spiral_jump"] = (rid, r.geoms[1][0])
    for rid in plain:
        r = R[rid]
        if len(r.sections) >= 3:
            m = r.sections[1]
            dr = [l for l in m.right if l.type == "driving"]
            if len(m.right) == 1 and len(dr) == 1 and not m.left and r.sections[0].lane(dr[0].id) is not None                     and r.sections[2].lane(dr[0].id) is not None:
                t["sec_lane_flip"] = (rid, dr[0].id, 1)
                break
    # a taper lane that starts at zero width (entered only by a lane change), not adjacent to the centre
    for rid in plain:
        r = R[rid]
        for k, sec in enumerate(r.sections):
            for l in sec.right:
                if l.type != "driving" or l.id >= -1 or l.pred is not None or k == 0:
                    continue
                if LF.lane_width(sec, l, sec.s) > LF.WIDTH_ZERO:
                    continue
                nb = sec.lane(l.id + 1)
                if nb is None or nb.type != "driving":
                    continue
                t["lanechange_block"] = (rid, k, l.id, l.id + 1)
                break
            if "lanechange_block" in t:
                break
        if "lanechange_block" in t:
            break
    # flip a connecting road lane: single-lane connecting road, -1 -> 1
    for rid in sorted(R, key=LF._rid_key):
        r = R[rid]
        if r.junction != "-1" and len(r.sections) == 1 and [l.id for l in r.sections[0].lanes()] == [-1] and \
                r.pred is not None and r.succ is not None and r.pred.type == "road" and r.succ.type == "road" and \
                r.pred.contact == "end" and r.succ.contact == "start":
            t["conn_lane_flip"] = rid
            break
    # flip the 'from' of a single laneLink (from -1 -> +1) on a two-way incoming road
    for jid in sorted(net.junctions, key=LF._rid_key):
        for c in net.junctions[jid]["conns"]:
            I = R.get(c["incoming"])
            if I is None or I.succ is None or I.succ.type != "junction" or I.succ.id != jid:
                continue
            if len(c["lanes"]) == 1 and c["lanes"][0][0] == -1 and I.sections[-1].lane(1) is not None and \
                    I.sections[-1].lane(1).type == "driving" and I.pred is not None and I.pred.id != jid:
                t["from_flip"] = (jid, c["incoming"], c["connecting"])
                break
        if "from_flip" in t:
            break
    # delete a plain two-way road that only one OSM way names
    for rid in plain:
        r = R[rid]
        if r.pred is not None and r.succ is not None and r.pred.type == "junction" and r.succ.type == "junction" and \
                len(r.osm_ways) == 1 and 40 < r.length < 120 and \
                sum(1 for q in R.values() if r.osm_ways[0] in q.osm_ways) == 1:
            t["road_del"] = rid
            break
    return t


def _move_angle(R, mv):
    (a, da), (b, db) = mv
    ra, rb = R[a], R[b]
    _, _, ha = ra.ref(np.array([ra.length if da > 0 else 0.0]))
    _, _, hb = rb.ref(np.array([0.0 if db > 0 else rb.length]))
    ha = float(ha[0]) + (0 if da > 0 else math.pi)
    hb = float(hb[0]) + (0 if db > 0 else math.pi)
    return math.degrees(LF._wrap(hb - ha))


def plant(base, name, tgt, lf):
    tree = ET.parse(base)
    root = tree.getroot()
    RE = roads_by_id(root)
    JE = {j.get("id"): j for j in root.findall("junction")}
    R = lf.net.roads
    if name == "laneLink_del":
        jid, inc, con, (f, to) = tgt
        for c in JE[jid].findall("connection"):
            if c.get("incomingRoad") == inc and c.get("connectingRoad") == con:
                for ll in c.findall("laneLink"):
                    if int(ll.get("from")) == f and int(ll.get("to")) == to:
                        c.remove(ll)
        r = R[con]
        s = 0.0 if [c for c in lf.net.junctions[jid]["conns"] if c["connecting"] == con][0]["contact"] == "start" \
            else r.length
        p = Plant(name, f"junction {jid}: laneLink {f}->{to} of connection {inc}->{con} deleted",
                  [("link.junction_asymmetric", con, to, None)],
                  allowed=("motor.flow.unreachable", "motor.island", "motor.trap_region", "pairs.", "osm.flow.lost",
                           "turn.missing", "motor.dead_end.junction_no_connection"))
    elif name == "conn_del":
        jid, inc, con, mv = tgt
        j = JE[jid]
        for c in j.findall("connection"):
            if c.get("incomingRoad") == inc and c.get("connectingRoad") == con:
                j.remove(c)
        root.remove(RE[con])
        p = Plant(name, f"junction {jid}: connection {inc}->{con} and road {con} deleted (move {mv})",
                  [("turn.missing", inc, None, None)],
                  allowed=("motor.dead_end.junction", "motor.flow.", "motor.trap_region", "pairs.", "osm.flow.lost",
                           "motor.orphan.junction", "motor.island"))
    elif name == "succ_break":
        rid = tgt
        lk = RE[rid].find("link")
        lk.remove(lk.find("successor"))
        p = Plant(name, f"road {rid}: successor (junction {R[rid].succ.id}) link removed",
                  [("motor.dead_end.unlinked_road_end", rid, None, None),
                   ("link.junction_not_linked", rid, None, None)],
                  allowed=("motor.flow.", "motor.trap_region", "pairs.", "osm.flow.lost", "turn.missing",
                           "motor.orphan.", "motor.island", "link.connecting_not_reciprocated"))
    elif name == "lane_succ_break":
        rid, lid, s = tgt
        set_lane_link(lane_el(secs(RE[rid])[0], lid), "successor", None)
        p = Plant(name, f"road {rid}: lane {lid} successor at section boundary s={s:.2f} removed",
                  [("link.lane_asymmetric", rid, lid, s)])
    elif name == "width_shift":
        rid, lid, s = tgt
        le = lane_el(secs(RE[rid])[1], lid)
        w = sorted(le.findall("width"), key=lambda e: float(e.get("sOffset")))[0]
        w.set("a", repr(float(w.get("a")) + 0.5))
        p = Plant(name, f"road {rid}: lane {lid} width +0.5 m at section start s={s:.2f} (geometry only)", [],
                  silent=True)
    elif name == "spiral_jump":
        rid, s0 = tgt
        for gm in RE[rid].find("planView").findall("geometry"):
            if abs(float(gm.get("s")) - s0) < 1e-9:
                sp = gm[0]
                sp.set("curvStart", repr(float(sp.get("curvStart")) + 0.01))
        p = Plant(name, f"road {rid}: spiral at s={s0:.2f} curvStart +0.01 1/m (geometry only)", [],
                  silent=True, allowed=("osm.",))
    elif name == "sidewalk_cut":
        rid, lid, k0, s1, s2 = tgt
        S = secs(RE[rid])[k0:k0 + 3]
        S[1].find("left" if lid > 0 else "right").remove(lane_el(S[1], lid))
        set_lane_link(lane_el(S[0], lid), "successor", None)
        set_lane_link(lane_el(S[2], lid), "predecessor", None)
        p = Plant(name, f"road {rid}: sidewalk lane {lid} removed from section 1 [{s1:.2f}, {s2:.2f}]",
                  [("edge.break", rid, lid, s1), ("edge.break", rid, lid, s2)],
                  allowed=("link.",))
    elif name == "lanechange_block":
        rid, k, t, n = tgt
        S = secs(RE[rid])[k]
        le = lane_el(S, n)
        marks = le.findall("roadMark")
        if not marks:
            m = ET.SubElement(le, "roadMark", {"sOffset": "0", "type": "solid", "weight": "standard",
                                               "color": "standard", "width": "0.12"})
            marks = [m]
        for m in marks:
            m.set("laneChange", "none")
        p = Plant(name, f"road {rid} section {k}: lane {n}'s roadMark laneChange=none, isolating the taper lane {t}",
                  # the lane starts at zero width at the section start: in a +s lane that is its entry
                  # (unreachable), in a -s lane (LHT right side) its exit (trapped)
                  [("motor.flow.unreachable" if lf.net.dirs(rid, k, R[rid].sections[k].lane(t))[0] > 0
                    else "motor.flow.trapped", rid, t, None)],
                  allowed=("motor.orphan.", "motor.dead_end.", "motor.flow.", "motor.island", "motor.trap_region",
                           "pairs.", "osm.flow.lost", "turn.missing"))
    elif name == "edge_unlink":
        rid, lid, lid2, s1 = tgt
        S = secs(RE[rid])
        set_lane_link(lane_el(S[0], lid), "successor", None)
        set_lane_link(lane_el(S[1], lid2), "predecessor", None)
        p = Plant(name, f"road {rid}: sidewalk {lid} -> {lid2} lane links removed at s={s1:.2f}",
                  [("edge.unlinked", rid, lid, s1)])
    elif name == "sec_lane_flip":
        rid, lid, k = tgt
        S = secs(RE[rid])
        le = lane_el(S[k], lid)
        old = S[k].find("right" if lid < 0 else "left")
        old.remove(le)
        le.set("id", str(-lid))
        new = S[k].find("left" if lid < 0 else "right")
        if new is None:
            new = ET.SubElement(S[k], "left" if lid < 0 else "right")
        new.append(le)
        if k > 0:
            set_lane_link(lane_el(S[k - 1], lid), "successor", -lid)
        if k + 1 < len(S):
            set_lane_link(lane_el(S[k + 1], lid), "predecessor", -lid)
        p = Plant(name, f"road {rid}: lane {lid} of section {k} renamed {-lid} (other side, links follow)",
                  [("motor.link.direction", rid, None, None)],
                  allowed=("motor.dead_end", "motor.orphan", "motor.flow.", "motor.trap_region", "motor.island",
                           "pairs.", "osm.flow.lost", "osm.direction", "turn.missing", "unusable."))
    elif name == "conn_lane_flip":
        rid = tgt
        r = RE[rid]
        S = secs(r)[0]
        le = lane_el(S, -1)
        S.find("right").remove(le)
        le.set("id", "1")
        lf_ = S.find("left")
        if lf_ is None:
            lf_ = ET.Element("left")
            S.insert(0, lf_)
        lf_.append(le)
        jid = R[rid].junction
        for c in JE[jid].findall("connection"):
            if c.get("connectingRoad") == rid:
                for ll in c.findall("laneLink"):
                    if ll.get("to") == "-1":
                        ll.set("to", "1")
        p = Plant(name, f"connecting road {rid}: lane -1 renamed 1 (travels against its links)",
                  [("motor.link.direction", None, None, None)],
                  allowed=("motor.link.direction", "motor.dead_end", "motor.orphan", "motor.flow.", "motor.island",
                           "motor.trap_region", "pairs.", "osm.flow.lost", "turn.missing"))
    elif name == "from_flip":
        jid, inc, con = tgt
        for c in JE[jid].findall("connection"):
            if c.get("incomingRoad") == inc and c.get("connectingRoad") == con:
                for ll in c.findall("laneLink"):
                    ll.set("from", "1")
        p = Plant(name, f"junction {jid}: laneLink from -1 -> +1 on connection {inc}->{con}",
                  [("motor.link.direction", inc, 1, None)],
                  allowed=("motor.orphan.connecting_road_unlinked", "motor.dead_end", "motor.flow.", "motor.island",
                           "motor.trap_region", "pairs.", "osm.flow.lost", "turn.missing"))
    elif name == "zero_width":
        rid, lid = tgt
        for w in lane_el(secs(RE[rid])[0], lid).findall("width"):
            _set_abcd(w, (0, 0, 0, 0))
        p = Plant(name, f"road {rid}: driving lane {lid} width 0 over the whole road",
                  [("unusable.zero_width_lane", rid, lid, 0.0)],
                  allowed=("motor.flow.", "motor.dead_end", "motor.orphan", "motor.island", "motor.trap_region",
                           "pairs.", "osm.flow.lost", "turn.missing"))
    elif name == "narrow":
        rid, lid = tgt
        for w in lane_el(secs(RE[rid])[0], lid).findall("width"):
            _set_abcd(w, (1.2, 0, 0, 0))
        p = Plant(name, f"road {rid}: lane {lid} 1.2 m wide over the whole road",
                  [("unusable.narrow_lane", rid, lid, 0.0)])
    elif name == "road_del":
        rid = tgt
        root.remove(RE[rid])
        gone = {rid}
        for cid, ce in list(RE.items()):
            if ce.get("junction", "-1") != "-1":
                lk = ce.find("link")
                if lk is not None and any(e.get("elementId") == rid and e.get("elementType") == "road" for e in lk):
                    root.remove(ce)
                    gone.add(cid)
        for j in JE.values():
            for c in j.findall("connection"):
                if c.get("incomingRoad") in gone or c.get("connectingRoad") in gone:
                    j.remove(c)
        p = Plant(name, f"road {rid} ({R[rid].osm_ways}) and its {len(gone) - 1} connecting roads deleted",
                  [("osm.way_dropped", None, None, None)],
                  allowed=("turn.", "motor.", "pairs.", "osm.flow.lost", "osm.direction"))
    elif name == "road_del_laneless":
        # road_del, plus a road without lanes (centre lane only, no OSM way, no links) along the deleted road's reference
        # line, extended 60 m straight beyond both ends so that it runs over the whole OSM way into the junctions, and
        # belonging to a junction -- as a junction boundary helper road (OpenDRIVE 1.8.0 §12.10). It has no surface, so
        # the way stays dropped.
        rid, ext = tgt, 60.0
        r = R[rid]
        bare = copy.deepcopy(RE[rid])
        bare.set("id", str(max(int(i) for i in RE if i.isdigit()) + 1))
        bare.set("name", "laneless")
        jlink = [e for e in RE[rid].find("link") if e.get("elementType") == "junction"]
        bare.set("junction", jlink[0].get("elementId"))
        for tag in ("link", "objects", "signals", "userData"):
            for e in bare.findall(tag):
                bare.remove(e)
        lanes = bare.find("lanes")
        for e in lanes.findall("laneOffset"):
            lanes.remove(e)
        for sec in lanes.findall("laneSection"):
            for side in ("left", "right"):
                for e in sec.findall(side):
                    sec.remove(e)
        pv = bare.find("planView")
        g0 = pv.find("geometry")
        x0, y0, h0 = float(g0.get("x")), float(g0.get("y")), float(g0.get("hdg"))
        xe, ye, he = (float(v[0]) for v in r.ref(np.array([r.length])))
        for g in pv.findall("geometry"):
            g.set("s", repr(float(g.get("s")) + ext))
        before = ET.Element("geometry", {"s": "0", "x": repr(x0 - ext * math.cos(h0)), "y": repr(y0 - ext * math.sin(h0)),
                                         "hdg": repr(h0), "length": repr(ext)})
        ET.SubElement(before, "line")
        after = ET.Element("geometry", {"s": repr(r.length + ext), "x": repr(xe), "y": repr(ye), "hdg": repr(he), "length": repr(ext)})
        ET.SubElement(after, "line")
        pv.insert(0, before)
        pv.append(after)
        bare.set("length", repr(r.length + 2 * ext))
        tree, p = plant(base, "road_del", tgt, lf)
        tree.getroot().append(bare)
        p = Plant(name, f"road {rid} deleted, a lane-less junction road left along its reference line (+{ext:.0f} m each end)",
                  p.expect, allowed=p.allowed)
    elif name in ("split", "split_rev"):
        rid, s0 = tgt
        p = _split(root, RE, JE, R, rid, s0, reverse=(name == "split_rev"))
    else:
        raise KeyError(name)
    return tree, p


def _split(root, RE, JE, R, rid, s0, reverse):
    """split road rid at a geometry boundary s0 into rid [0, s0] and a new road [s0, L]; with reverse
    the new road runs the other way (end-end contact): geometry reversed, lanes mirrored."""
    A = RE[rid]
    r = R[rid]
    L = r.length
    B = copy.deepcopy(A)
    bid = str(900000 + int(rid))
    B.set("id", bid)
    # --- A: [0, s0]
    pv = A.find("planView")
    for gm in list(pv.findall("geometry")):
        if float(gm.get("s")) >= s0 - 1e-9:
            pv.remove(gm)
    A.set("length", repr(s0))
    for tag, parent in (("laneOffset", A.find("lanes")),):
        cut_records(parent, tag, "s", 0.0, s0, 0.0)
    ep = A.find("elevationProfile")
    if ep is not None:
        cut_records(ep, "elevation", "s", 0.0, s0, 0.0)
    for tag in ("objects", "signals", "lateralProfile"):
        e = A.find(tag)
        if e is not None:
            A.remove(e)
    # --- B: [s0, L]
    pv = B.find("planView")
    gms = []
    for gm in list(pv.findall("geometry")):
        if float(gm.get("s")) < s0 - 1e-9:
            pv.remove(gm)
        else:
            gm.set("s", repr(float(gm.get("s")) - s0))
            gms.append(gm)
    B.set("length", repr(L - s0))
    cut_records(B.find("lanes"), "laneOffset", "s", s0, L, s0)
    ep = B.find("elevationProfile")
    if ep is not None:
        cut_records(ep, "elevation", "s", s0, L, s0)
    for tag in ("objects", "signals", "lateralProfile"):
        e = B.find(tag)
        if e is not None:
            B.remove(e)
    old_succ = A.find("link").find("successor")
    jid = old_succ.get("elementId")
    sec_b = secs(B)[0]
    if reverse:
        # geometry reversed (line / arc / spiral)
        new = []
        acc = 0.0
        for gm in reversed(gms):
            Lg = float(gm.get("length"))
            x, y = G._eval_geometry(gm, np.array([Lg]))
            kind = gm[0]
            h0 = float(gm.get("hdg"))
            if kind.tag == "line":
                h1 = h0
            elif kind.tag == "arc":
                k = float(kind.get("curvature"))
                h1 = h0 + k * Lg
                kind.set("curvature", repr(-k))
            else:
                k0, k1 = float(kind.get("curvStart")), float(kind.get("curvEnd"))
                h1 = h0 + k0 * Lg + 0.5 * (k1 - k0) * Lg
                kind.set("curvStart", repr(-k1))
                kind.set("curvEnd", repr(-k0))
            gm.set("x", repr(float(x[0])))
            gm.set("y", repr(float(y[0])))
            gm.set("hdg", repr(math.atan2(math.sin(h1 + math.pi), math.cos(h1 + math.pi))))
            gm.set("s", repr(acc))
            acc += Lg
            new.append(gm)
        for gm in gms:
            pv.remove(gm)
        for gm in new:
            pv.append(gm)
        # lanes mirrored (constant widths, zero offset: checked by choose())
        left, right = sec_b.find("left"), sec_b.find("right")
        for side in (left, right):
            if side is not None:
                for le in side.findall("lane"):
                    le.set("id", str(-int(le.get("id"))))
        if left is not None:
            left.tag = "right_tmp"
        if right is not None:
            right.tag = "left"
        if left is not None:
            left.tag = "right"
        # B: pred = junction (old successor), succ = A end
        lk = B.find("link")
        for e in list(lk):
            lk.remove(e)
        set_link(B, "predecessor", "junction", jid)
        set_link(B, "successor", "road", rid, "end")
        set_link(A, "successor", "road", bid, "end")
        for le in sec_b.iter("lane"):
            if int(le.get("id")) != 0:
                set_lane_link(le, "predecessor", None)
                set_lane_link(le, "successor", -int(le.get("id")))
        for le in secs(A)[-1].iter("lane"):
            if int(le.get("id")) != 0:
                set_lane_link(le, "successor", -int(le.get("id")))
        flip = -1
    else:
        set_link(B, "predecessor", "road", rid, "end")
        set_link(A, "successor", "road", bid, "start")
        for le in sec_b.iter("lane"):
            if int(le.get("id")) != 0:
                set_lane_link(le, "predecessor", int(le.get("id")))
        for le in secs(A)[-1].iter("lane"):
            if int(le.get("id")) != 0:
                set_lane_link(le, "successor", int(le.get("id")))
        flip = 1
    # junction side: connecting roads that linked rid's end now link B
    for ce in RE.values():
        if ce.get("junction", "-1") != jid:
            continue
        lk = ce.find("link")
        for e in (lk if lk is not None else []):
            if e.get("elementType") == "road" and e.get("elementId") == rid and e.get("contactPoint") == "end":
                e.set("elementId", bid)
                if reverse:
                    e.set("contactPoint", "start")
                tag = e.tag
                if flip == -1:
                    for le in ce.iter("lane"):
                        l2 = le.find("link")
                        q = l2.find(tag) if l2 is not None else None
                        if q is not None:
                            q.set("id", str(-int(q.get("id"))))
    for c in JE[jid].findall("connection"):
        if c.get("incomingRoad") == rid:
            c.set("incomingRoad", bid)
            if flip == -1:
                for ll in c.findall("laneLink"):
                    ll.set("from", str(-int(ll.get("from"))))
    root.insert(list(root).index(A) + 1, B)
    return Plant("split_rev" if reverse else "split",
                 f"road {rid} split at s={s0:.2f} into {rid} + {bid}" + (" (second half reversed, end-end)" if reverse
                                                                          else " (start contact)"),
                 [("split.road_needless", rid, None, s0)], allowed=("split.section_redundant",))


# ----------------------------------------------------------------------------- diff
def inst_key(name, i):
    return (name, str(i.get("road")), str(i.get("lane")), i.get("s"), str(i.get("detail"))[:160],
            str(i.get("frm")), str(i.get("to")), str(i.get("osm_node")), str(i.get("way")), str(i.get("view")))


def diff(base, new):
    """per invariant: (count delta, added instances, removed instances)."""
    out = {}
    for k in sorted(set(base) | set(new)):
        if k.startswith("_"):
            continue
        a = base.get(k, {"count": 0, "instances": [], "severity": new.get(k, {}).get("severity")})
        b = new.get(k, {"count": 0, "instances": [], "severity": a.get("severity")})
        ka = {inst_key(k, i) for i in a["instances"]}
        kb = {inst_key(k, i) for i in b["instances"]}
        add = [i for i in b["instances"] if inst_key(k, i) not in ka]
        rem = [i for i in a["instances"] if inst_key(k, i) not in kb]
        if add or rem or a["count"] != b["count"]:
            out[k] = {"severity": b.get("severity") or a.get("severity"), "delta": b["count"] - a["count"],
                      "added": add, "removed": rem}
    return out


def judge(p, d, extra_ok=()):
    """did the plant show up where expected, and nothing else (outside allowed cascades)?"""
    res = {"plant": p.name, "desc": p.desc, "expected": [], "unexpected_defects": []}
    for inv, road, lane, s in p.expect:
        hits = [i for k, v in d.items() if k.startswith(inv) for i in v["added"]
                if (road is None or str(i.get("road")) == str(road)) and (lane is None or i.get("lane") == lane) and
                (s is None or i.get("s") is None or abs(i["s"] - s) <= 0.05)]
        res["expected"].append({"inv": inv, "road": road, "lane": lane, "s": s, "hits": len(hits),
                                "names": sorted({k for k, v in d.items() if k.startswith(inv) for i in v["added"]
                                                 if i in hits})})
    for k, v in d.items():
        if v["severity"] != "defect":
            continue
        if any(k.startswith(e[0]) for e in p.expect) or any(k.startswith(a) for a in p.allowed) or                 (k == "rule.osm" and v["delta"] == 0):  # file-level vote, its fraction moves with any edit
            continue
        res["unexpected_defects"].append({"inv": k, "delta": v["delta"], "added": len(v["added"]),
                                          "removed": len(v["removed"]),
                                          "first": (v["added"] or v["removed"])[:2]})
    if p.silent:
        res["silent_ok"] = not any(v["severity"] == "defect" and not any(k.startswith(a) for a in p.allowed)
                                   for k, v in d.items())
    res["changed"] = {k: {"sev": v["severity"], "delta": v["delta"], "added": len(v["added"]),
                          "removed": len(v["removed"])} for k, v in d.items()}
    res["pass"] = all(e["hits"] >= 1 for e in res["expected"]) and not res["unexpected_defects"] and \
        res.get("silent_ok", True)
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--base", required=True)
    ap.add_argument("--osm", required=True)
    ap.add_argument("--clip", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", help="comma list of plant names")
    ap.add_argument("--rule", choices=["RHT", "LHT"], help="evaluate under this traffic rule (laneflow --rule)")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    with open(a.base, encoding="utf-8") as f:
        head = f.read(4000)
    import re
    lat0 = float(re.search(r"\+lat_0=([-0-9.eE]+)", head).group(1))
    lon0 = float(re.search(r"\+lon_0=([-0-9.eE]+)", head).group(1))
    box = LF.clip_box_from_latlon([float(v) for v in a.clip.split(",")], lat0, lon0)
    lf0, d0 = LF.run(a.base, a.osm, box, rule=a.rule)
    tg = choose(lf0)
    names = ["laneLink_del", "conn_del", "succ_break", "lane_succ_break", "width_shift", "spiral_jump", "sidewalk_cut",
             "edge_unlink", "sec_lane_flip", "lanechange_block", "conn_lane_flip", "from_flip", "zero_width", "narrow", "split", "split_rev", "road_del",
             "road_del_laneless"]
    if a.only:
        names = [n for n in names if n in a.only.split(",")]
    stem = os.path.splitext(os.path.basename(a.base))[0] + ("_" + a.rule if a.rule else "")
    report = {"base": a.base, "targets": {k: str(v) for k, v in tg.items()}, "results": []}
    for nm in names:
        key = "split" if nm == "split_rev" else "road_del" if nm == "road_del_laneless" else nm
        if key not in tg:
            report["results"].append({"plant": nm, "skipped": "no target found"})
            continue
        tree, p = plant(a.base, nm, tg[key], lf0)
        path = os.path.join(a.out, f"{stem}__{nm}.xodr")
        tree.write(path, encoding="utf-8", xml_declaration=True)
        _, d1 = LF.run(path, a.osm, box, rule=a.rule)
        dd = diff(d0, d1)
        res = judge(p, dd)
        res["file"] = path
        report["results"].append(res)
        with open(os.path.join(a.out, f"{stem}__{nm}.diff.json"), "w", encoding="utf-8") as f:
            json.dump(dd, f, indent=1, default=str)
        print(f"{nm:16s} {'PASS' if res['pass'] else 'FAIL'}  " +
              "; ".join(f"{e['inv']}@{e['road']}/{e['lane']}: {e['hits']}" for e in res["expected"]) +
              ("" if not res["unexpected_defects"] else "  UNEXPECTED " +
               ", ".join(f"{u['inv']}({u['delta']:+d})" for u in res["unexpected_defects"])))
        print("    changed:", {k: (v["sev"][0], v["delta"]) for k, v in res["changed"].items()})
    with open(os.path.join(a.out, f"{stem}__report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1, default=str)
    return report


if __name__ == "__main__":
    main()
