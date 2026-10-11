"""invariants_selftest -- plant known defects into a real OdrForge output and check that invariants.py
reports each of them, at the planted place, and nothing new anywhere else.

    python invariants_selftest.py --area munich --out DIR [--only NAME[,NAME]]
    python invariants_selftest.py BASE.xodr AREA.osm --clip lat0,lon0,lat1,lon1 --out DIR

The base file is copied first (DIR/base.xodr), so a converter run that rewrites the area output in the
meantime does not change the experiment. For every plant the planted file is written to
DIR/<plant>.xodr, invariants.run_all() is run on it and its instances are diffed against the base run
(key = invariant, road, lane, kind/family/edge, to_road/to_lane, s rounded to 0.5 m).

A plant PASSES when
  * every expected (invariant, road[, lane][, s +- 1 m]) appears among the NEW instances exactly the
    expected number of times, and
  * no other new instance appears farther than `radius` m from the planted spot (collateral findings next
    to a planted defect are listed, not failed: a broken link legitimately also starves the lanes behind
    it) -- every new instance outside that radius is a FAIL (the tool reacted somewhere untouched).
Exit status 1 when any plant fails.  DIR/selftest.json holds the full diff per plant.

Plants are written against the Munich reference output (road / junction ids looked up by structure,
not hard-coded where it can be avoided), so they keep working when the converter changes.
"""
import argparse
import collections
import copy
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import invariants as I  # noqa: E402


# ----------------------------------------------------------------------------- xml helpers
def roads_of(root):
    return {r.get("id"): r for r in root.findall("road")}


def junctions_of(root):
    return {j.get("id"): j for j in root.findall("junction")}


def sections(road):
    return road.findall("lanes/laneSection")


def lanes_of(sec):
    return {int(l.get("id")): l for l in sec.iter("lane")}


def set_link(lane, which, lid):
    """set (lid int) or remove (lid None) a lane's predecessor / successor."""
    lk = lane.find("link")
    if lk is None:
        if lid is None:
            return
        lk = ET.Element("link")
        lane.insert(0, lk)
    e = lk.find(which)
    if lid is None:
        if e is not None:
            lk.remove(e)
        if not len(lk):
            lane.remove(lk)
        return
    if e is None:
        e = ET.SubElement(lk, which)
        if which == "predecessor" and len(lk) > 1:  # keep predecessor before successor
            lk.remove(e)
            lk.insert(0, e)
    e.set("id", str(lid))


def recenter(rec, d0):
    """cubic a + b d + c d^2 + e d^3 re-expanded around d0."""
    a, b, c, e = rec
    return (a + b * d0 + c * d0 * d0 + e * d0 ** 3, b + 2 * c * d0 + 3 * e * d0 * d0, c + 3 * e * d0, e)


def fnum(v):
    return repr(float(v))


def xy_of(path, rid, s):
    net = I.Network(path)
    return I._xy(net.roads[rid], s)


def pick_section_lane(root):
    """(road id, section k, lane id): first plain road (by id) with >= 3 lane sections whose section 1
    (>= 4 m long) has a driving lane linked on both sides -- the target of the width plants."""
    R = roads_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        secs = sections(r)
        if r.get("junction") != "-1" or len(secs) < 3:
            continue
        if float(secs[2].get("s")) - float(secs[1].get("s")) < 4.0:
            continue
        for lid, l in sorted(lanes_of(secs[1]).items(), key=lambda kv: abs(kv[0])):
            if lid != 0 and l.get("type") == "driving" and l.find("link/predecessor") is not None                     and l.find("link/successor") is not None and len(l.findall("width")) == 1:
                return rid, 1, lid
    raise RuntimeError("no multi-section road")


def pick_line(root, need=12.0):
    """(road id, s): a plain road with a line element >= need m that starts after s = 0; s = 2 m into it."""
    R = roads_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        if r.get("junction") != "-1":
            continue
        for g in r.findall("planView/geometry"):
            if g[0].tag == "line" and float(g.get("length")) >= need and float(g.get("s")) > 1.0:
                return rid, float(g.get("s")) + 2.0
    raise RuntimeError("no line element")


def pick_split(root):
    """road id: a plain road with an element join 20 m or more from both ends."""
    R = roads_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        L = float(r.get("length"))
        if r.get("junction") == "-1" and any(20 < float(g.get("s")) < L - 20 for g in r.findall("planView/geometry")[1:]):
            return rid
    raise RuntimeError("no splittable road")


def _rid(v):
    return (0, int(v)) if v.lstrip("-").isdigit() else (1, v)


# ----------------------------------------------------------------------------- plants
# each plant(root) -> dict(desc, expect=[dict(inv, road, lane=None, s=None, n=1, kind=None)],
#                          at=(road, s) planted spot for the collateral radius, radius=m)

def p_del_laneLink(root):
    """delete the junction laneLink of the first connection whose incoming lane is drivable."""
    R, J = roads_of(root), junctions_of(root)
    for jid in sorted(J, key=_rid):
        for c in J[jid].findall("connection"):
            lls = c.findall("laneLink")
            if len(lls) == 1 and R[c.get("connectingRoad")].get("junction") == jid:
                inc = c.get("incomingRoad")
                f = int(lls[0].get("from"))
                c.remove(lls[0])
                end = "end" if (R[inc].find("link/successor") is not None and
                                R[inc].find("link/successor").get("elementId") == jid) else "start"
                L = float(R[inc].get("length"))
                return dict(desc=f"junction {jid} connection {c.get('id')}: laneLink {inc}:{f} -> "
                                 f"{c.get('connectingRoad')} deleted",
                            expect=[dict(inv="topology", road=c.get("connectingRoad"), kind="laneLink_missing")],
                            at=(inc, 0.0 if end == "start" else L), radius=60)


def p_width_shift(root, dw=0.5):
    """widen one lane by dw over a whole interior lane section (step at both section boundaries)."""
    rid, k, lane = pick_section_lane(root)
    R = roads_of(root)
    secs = sections(R[rid])
    ln = lanes_of(secs[k])[lane]
    w = ln.find("width")
    w.set("a", fnum(float(w.get("a")) + dw))
    s0, s1 = float(secs[k].get("s")), float(secs[k + 1].get("s"))
    up = int(ln.find("link/predecessor").get("id"))  # the G0 instance is keyed by the upstream lane
    return dict(desc=f"road {rid} section {k} lane {lane}: width +{dw} m",
                expect=[dict(inv="G0_border", road=rid, lane=up, s=s0, n=1),
                        dict(inv="G0_border", road=rid, lane=lane, s=s1, n=1)],
                at=(rid, (s0 + s1) / 2), radius=60)


def p_break_successor(root):
    """remove the road-level successor (a junction) of a two-way plain road."""
    R = roads_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        sc = r.find("link/successor")
        if r.get("junction") == "-1" and sc is not None and sc.get("elementType") == "junction":
            last = lanes_of(sections(r)[-1])
            if any(l > 0 for l in last) and any(l < 0 for l in last):
                r.find("link").remove(sc)
                return dict(desc=f"road {rid}: successor junction {sc.get('elementId')} removed",
                            expect=[dict(inv="topology", road=rid, kind="incoming_road_not_linked_to_junction")],
                            at=(rid, float(r.get("length"))), radius=60)


def p_curvature_jump(root, k=0.03, a=1.0):
    """insert an S of arcs (k, a) (-k, 2a) (k, a) into a line element: lateral offset exactly 0 by
    symmetry, end shortfall ~ 2 k^2 a^3 / 3 = 0.6 mm (< 1 mm, so no G0 finding), curvature steps
    k, 2k, 2k, k at four joins -> four G2_ref instances."""
    rid, at_s = pick_line(root)
    R = roads_of(root)
    r = R[rid]
    pv = r.find("planView")
    for g in pv.findall("geometry"):
        s0, L = float(g.get("s")), float(g.get("length"))
        if g[0].tag == "line" and s0 + 1 < at_s and at_s + 4 * a + 1 < s0 + L:
            break
    else:
        raise RuntimeError("no long line element")
    x0, y0, h0 = float(g.get("x")), float(g.get("y")), float(g.get("hdg"))
    idx = list(pv).index(g)
    pv.remove(g)
    pieces = [("line", at_s - s0, 0.0), ("arc", a, k), ("arc", 2 * a, -k), ("arc", a, k),
              ("line", s0 + L - at_s - 4 * a, 0.0)]
    s, x, y, h = s0, x0, y0, h0
    for n, (kind, ln, kk) in enumerate(pieces):
        e = ET.Element("geometry", s=fnum(s), x=fnum(x), y=fnum(y), hdg=fnum(h), length=fnum(ln))
        if kind == "line":
            ET.SubElement(e, "line")
            x, y = x + ln * math.cos(h), y + ln * math.sin(h)
        else:
            ET.SubElement(e, "arc", curvature=fnum(kk))
            x += (math.sin(h + kk * ln) - math.sin(h)) / kk
            y += (math.cos(h) - math.cos(h + kk * ln)) / kk
            h += kk * ln
        pv.insert(idx + n, e)
        s += ln
    joins = [at_s, at_s + a, at_s + 3 * a, at_s + 4 * a]
    return dict(desc=f"road {rid}: arc S-curve (k={k}) inserted at s={at_s}",
                expect=[dict(inv="G2_ref", road=rid, s=j) for j in joins], at=(rid, at_s + 2 * a), radius=40)


def p_cut_edge(root):
    """unlink a sidewalk lane across an interior lane-section boundary (lane stays, link removed)."""
    R = roads_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        if r.get("junction") != "-1":
            continue
        secs = sections(r)
        for k in range(len(secs) - 1):
            for lid, l in sorted(lanes_of(secs[k]).items()):
                sc = l.find("link/successor")
                if l.get("type") == "sidewalk" and sc is not None:
                    t = int(sc.get("id"))
                    set_link(l, "successor", None)
                    set_link(lanes_of(secs[k + 1])[t], "predecessor", None)
                    sb = float(secs[k + 1].get("s"))
                    return dict(desc=f"road {rid}: sidewalk lane {lid} unlinked at section boundary s={sb:.2f}",
                                expect=[dict(inv="continuity_nondriving", road=rid, lane=lid, s=sb, kind="section"),
                                        dict(inv="G0_open_lane", road=rid, lane=lid, s=sb, kind="vanishes"),
                                        dict(inv="G0_open_lane", road=rid, lane=t, s=sb, kind="appears")],
                                at=(rid, sb), radius=30)


def p_flip_laneLink(root):
    """flip the sign of a junction laneLink 'from' (incoming lane now one that travels away)."""
    R, J = roads_of(root), junctions_of(root)
    for jid in sorted(J, key=_rid):
        for c in J[jid].findall("connection"):
            inc = R[c.get("incomingRoad")]
            ll = c.findall("laneLink")
            if len(ll) != 1:
                continue
            f = int(ll[0].get("from"))
            end = "end" if (inc.find("link/successor") is not None and
                            inc.find("link/successor").get("elementId") == jid) else "start"
            sec = sections(inc)[-1 if end == "end" else 0]
            lanes = lanes_of(sec)
            if -f in lanes and lanes[-f].get("type") == "driving" and lanes[f].get("type") == "driving":
                ll[0].set("from", str(-f))
                L = float(inc.get("length"))
                con = c.get("connectingRoad")
                return dict(desc=f"junction {jid}: laneLink {inc.get('id')}:{f} -> {con} flipped to {-f}",
                            expect=[dict(inv="flow_direction", road=inc.get("id"), lane=-f),
                                    dict(inv="topology", road=con, kind="laneLink_disagrees_with_lane_link"),
                                    dict(inv="G0_border", road=inc.get("id"), lane=-f, n=2)],
                            at=(inc.get("id"), 0.0 if end == "start" else L), radius=60)


def p_del_connecting_lane_link(root):
    """remove a connecting road lane's predecessor (the junction laneLink stays)."""
    R, J = roads_of(root), junctions_of(root)
    for jid in sorted(J, key=_rid):
        for c in J[jid].findall("connection"):
            con = R[c.get("connectingRoad")]
            if c.get("contactPoint") != "start" or len(c.findall("laneLink")) != 1:
                continue
            t = int(c.find("laneLink").get("to"))
            ln = lanes_of(sections(con)[0]).get(t)
            if ln is not None and ln.find("link/predecessor") is not None:
                set_link(ln, "predecessor", None)
                return dict(desc=f"connecting road {con.get('id')} lane {t}: predecessor removed",
                            expect=[dict(inv="topology", road=con.get("id"), lane=t,
                                         kind="connecting_lane_link_missing")],
                            at=(con.get("id"), 0.0), radius=60)


def p_flip_road_lanes(root):
    """mirror every lane of a one-way plain road (all driving lanes on one side) to the other side:
    id -> -id in the road, in the junction laneLinks and in the connecting roads' lane links. Links
    stay mutually consistent, but the carriageway now travels against them (head-on) under the same
    rule, and its lane count per direction no longer matches the oneway tag."""
    R, J = roads_of(root), junctions_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        if r.get("junction") != "-1" or len(sections(r)) != 1:
            continue
        ls = lanes_of(sections(r)[0])
        drv = [l for l, e in ls.items() if e.get("type") == "driving"]
        if not drv or (any(l > 0 for l in drv) and any(l < 0 for l in drv)):
            continue
        if not all(r.find(f"link/{w}") is not None and r.find(f"link/{w}").get("elementType") == "junction"
                   for w in ("predecessor", "successor")):
            continue
        sec = sections(r)[0]
        left, right = sec.find("left"), sec.find("right")
        new = {}
        for side, el in (("right", left), ("left", right)):
            ne = ET.Element(side)
            if el is not None:
                for l in el.findall("lane"):
                    l.set("id", str(-int(l.get("id"))))
                    for e in l.iter():
                        if e.tag in ("predecessor", "successor"):
                            e.set("id", str(-int(e.get("id"))))
                for l in sorted(el.findall("lane"), key=lambda l: -abs(int(l.get("id"))) if side == "left"
                                else abs(int(l.get("id")))):
                    ne.append(l)
                sec.remove(el)
            new[side] = ne
        if len(new["left"]):
            sec.insert(0, new["left"])
        if len(new["right"]):
            sec.append(new["right"])
        for j in J.values():
            for c in j.findall("connection"):
                if c.get("incomingRoad") == rid:
                    for ll in c.findall("laneLink"):
                        ll.set("from", str(-int(ll.get("from"))))
        for o in R.values():
            if o.get("junction") == "-1":
                continue
            for w, pos in (("predecessor", 0), ("successor", -1)):
                e = o.find(f"link/{w}")
                if e is not None and e.get("elementType") == "road" and e.get("elementId") == rid:
                    for l in lanes_of(sections(o)[pos]).values():
                        q = l.find(f"link/{w}")
                        if q is not None:
                            q.set("id", str(-int(q.get("id"))))
        L = float(r.get("length"))
        return dict(desc=f"road {rid}: one-way carriageway mirrored to the other side (ids negated, links kept)",
                    expect=[dict(inv="flow_direction", road=rid, n=None)],
                    at=(rid, L / 2), radius=L / 2 + 60)


def _split_poly(recs, d0):
    """records [(sOffset, a, b, c, d)] -> (head: records before d0, tail: records from d0, rebased)."""
    recs = sorted(recs)
    head = [r for r in recs if r[0] < d0 - 1e-9]
    tail = [(r[0] - d0,) + r[1:] for r in recs if r[0] >= d0 - 1e-9]
    if head and (not tail or tail[0][0] > 1e-9):
        act = head[-1]
        tail.insert(0, (0.0,) + recenter(act[1:], d0 - act[0]))
    return head or [recs[0]], tail


def p_needless_split(root, new_id="900000"):
    """split one plain road at a planView element join into two roads linked road-to-road with the
    identical lane layout (nothing in OSM branches there)."""
    rid = pick_split(root)
    R, J = roads_of(root), junctions_of(root)
    A = R[rid]
    L = float(A.get("length"))
    gs = A.findall("planView/geometry")
    cut = next(float(g.get("s")) for g in gs[1:] if 20 < float(g.get("s")) < L - 20)
    B = copy.deepcopy(A)
    B.set("id", new_id)
    A.set("length", fnum(cut))
    B.set("length", fnum(L - cut))
    # planView
    for g in A.findall("planView/geometry"):
        if float(g.get("s")) >= cut - 1e-9:
            A.find("planView").remove(g)
    for g in B.findall("planView/geometry"):
        if float(g.get("s")) < cut - 1e-9:
            B.find("planView").remove(g)
        else:
            g.set("s", fnum(float(g.get("s")) - cut))
    # elevation / lateral profiles, objects, signals: not judged by invariants -> keep start records
    for road, keep_head in ((A, True), (B, False)):
        for tag in ("elevationProfile", "lateralProfile", "objects", "signals"):
            e = road.find(tag)
            if e is not None:
                road.remove(e)
    # laneOffset
    lanesA, lanesB = A.find("lanes"), B.find("lanes")
    for ln, keep in ((lanesA, "head"), (lanesB, "tail")):
        offs = ln.findall("laneOffset")
        recs = [tuple(float(o.get(k)) for k in ("s", "a", "b", "c", "d")) for o in offs]
        head, tail = _split_poly(recs, cut) if recs else ([], [])
        for o in offs:
            ln.remove(o)
        for i, r in enumerate(head if keep == "head" else tail):
            ln.insert(i, ET.Element("laneOffset", s=fnum(r[0]), a=fnum(r[1]), b=fnum(r[2]), c=fnum(r[3]),
                                    d=fnum(r[4])))
    # lane sections
    def split_sections(lanes_el, keep):
        secs = lanes_el.findall("laneSection")
        ks = max(i for i, s in enumerate(secs) if float(s.get("s")) <= cut + 1e-9)
        for i, s in enumerate(secs):
            s0 = float(s.get("s"))
            if keep == "head":
                if i > ks:
                    lanes_el.remove(s)
                elif i == ks:
                    for l in lanes_of(s).values():
                        ws = l.findall("width")
                        recs = [tuple(float(w.get(k)) for k in ("sOffset", "a", "b", "c", "d")) for w in ws]
                        if recs:
                            head, _ = _split_poly(recs, cut - s0)
                            for w in ws[len(head):]:
                                l.remove(w)
                        if l.get("id") != "0":
                            set_link(l, "successor", int(l.get("id")))
            else:
                if i < ks:
                    lanes_el.remove(s)
                elif i == ks:
                    s.set("s", "0")
                    for l in lanes_of(s).values():
                        ws = l.findall("width")
                        recs = [tuple(float(w.get(k)) for k in ("sOffset", "a", "b", "c", "d")) for w in ws]
                        if recs:
                            _, tail = _split_poly(recs, cut - s0)
                            pos = list(l).index(ws[0])
                            for w in ws:
                                l.remove(w)
                            for n, r in enumerate(tail):
                                l.insert(pos + n, ET.Element("width", sOffset=fnum(r[0]), a=fnum(r[1]), b=fnum(r[2]),
                                                             c=fnum(r[3]), d=fnum(r[4])))
                        for rm in l.findall("roadMark") + l.findall("speed") + l.findall("material"):
                            if float(rm.get("sOffset", 0)) > 0:
                                rm.set("sOffset", fnum(max(0.0, float(rm.get("sOffset")) - (cut - s0))))
                        if l.get("id") != "0":
                            set_link(l, "predecessor", int(l.get("id")))
                else:
                    s.set("s", fnum(s0 - cut))
    split_sections(lanesA, "head")
    split_sections(lanesB, "tail")
    for t in B.findall("type"):
        t.set("s", "0")
    # road links
    old_succ = copy.deepcopy(A.find("link/successor"))
    linkA, linkB = A.find("link"), B.find("link")
    for e in list(linkA):
        if e.tag == "successor":
            linkA.remove(e)
    ET.SubElement(linkA, "successor", elementType="road", elementId=new_id, contactPoint="start")
    for e in list(linkB):
        if e.tag == "predecessor":
            linkB.remove(e)
    linkB.insert(0, ET.Element("predecessor", elementType="road", elementId=rid, contactPoint="end"))
    # everything that referenced the old end of rid now references new_id
    if old_succ is not None and old_succ.get("elementType") == "junction":
        jid = old_succ.get("elementId")
        for c in J[jid].findall("connection"):
            con = R[c.get("connectingRoad")]
            for w in ("predecessor", "successor"):
                e = con.find(f"link/{w}")
                if e is not None and e.get("elementId") == rid and e.get("contactPoint") == "end":
                    e.set("elementId", new_id)
                    if c.get("incomingRoad") == rid and ((w == "predecessor") == (c.get("contactPoint") == "start")):
                        c.set("incomingRoad", new_id)
    root.insert(list(root).index(A) + 1, B)
    return dict(desc=f"road {rid} split at element join s={cut:.3f} into {rid} + {new_id} (same layout)",
                expect=[dict(inv="needless_split", road=rid, s=cut)], at=(rid, cut), radius=30,
                alias={new_id: (rid, cut)})


def p_del_connection(root):
    """delete a whole junction connection (the connecting road and its own lane links stay)."""
    R, J = roads_of(root), junctions_of(root)
    for jid in sorted(J, key=_rid):
        for c in J[jid].findall("connection"):
            con = R[c.get("connectingRoad")]
            ll = c.findall("laneLink")
            if c.get("contactPoint") == "start" and len(ll) == 1 and con.get("junction") == jid:
                J[jid].remove(c)
                t = int(ll[0].get("to"))
                return dict(desc=f"junction {jid}: connection {c.get('id')} ({c.get('incomingRoad')} -> "
                                 f"{con.get('id')}) deleted",
                            expect=[dict(inv="topology", road=con.get("id"), lane=t, kind="laneLink_missing")],
                            at=(con.get("id"), 0.0), radius=60)


def p_isolated_stub(root, new_id="900001"):
    """a copy of a plain road with no links at all (an unusable island road)."""
    rid = pick_section_lane(root)[0]
    R = roads_of(root)
    B = copy.deepcopy(R[rid])
    B.set("id", new_id)
    lk = B.find("link")
    if lk is not None:
        B.remove(lk)
    for sec in sections(B):
        for l in lanes_of(sec).values():
            for e in l.findall("link"):
                l.remove(e)
    # keep the in-road lane-section links so only the road ends are open
    for k, (sa, sb) in enumerate(zip(sections(R[rid])[:-1], sections(R[rid])[1:])):
        for lid, l in lanes_of(sa).items():
            q = l.find("link/successor")
            if q is not None and lid != 0:
                set_link(lanes_of(sections(B)[k])[lid], "successor", int(q.get("id")))
        for lid, l in lanes_of(sb).items():
            q = l.find("link/predecessor")
            if q is not None and lid != 0:
                set_link(lanes_of(sections(B)[k + 1])[lid], "predecessor", int(q.get("id")))
    root.insert(list(root).index(R[rid]) + 1, B)
    L = float(B.get("length"))
    return dict(desc=f"road {new_id}: unlinked copy of road {rid}",
                expect=[dict(inv="flow_closed_not_strong", road=new_id, n=None),
                        dict(inv="flow_dead_end", road=new_id, n=None),
                        dict(inv="flow_dead_start", road=new_id, n=None)],
                at=(new_id, L / 2), radius=L / 2 + 30)


def p_drop_driving_link(root):
    """unlink a driving lane across an interior lane-section boundary (both directions of the link)."""
    R = roads_of(root)
    for rid in sorted(R, key=_rid):
        r = R[rid]
        if r.get("junction") != "-1":
            continue
        secs = sections(r)
        for k in range(len(secs) - 1):
            for lid, l in sorted(lanes_of(secs[k]).items()):
                sc = l.find("link/successor")
                if l.get("type") == "driving" and lid < 0 and sc is not None:
                    t = int(sc.get("id"))
                    set_link(l, "successor", None)
                    set_link(lanes_of(secs[k + 1])[t], "predecessor", None)
                    sb = float(secs[k + 1].get("s"))
                    return dict(desc=f"road {rid}: driving lane {lid} unlinked at section boundary s={sb:.2f}",
                                expect=[dict(inv="G0_open_lane", road=rid, lane=lid, s=sb, kind="vanishes"),
                                        dict(inv="G0_open_lane", road=rid, lane=t, s=sb, kind="appears"),
                                        dict(inv="flow_dead_end", road=rid, lane=lid, s=sb),
                                        dict(inv="flow_dead_start", road=rid, lane=t, s=sb)],
                                at=(rid, sb), radius=80)  # lanes that only feed the dead lane are trapped too


def p_width_slope_jump(root):
    """split one width record into two whose slopes differ by 0.01 (value continuous): a kink of the
    lane border inside the road -> one width_record_jump (slope) instance."""
    rid, k, lane = pick_section_lane(root)
    R = roads_of(root)
    sec = sections(R[rid])[k]
    l = lanes_of(sec)[lane]
    w = l.find("width")
    L = float(sections(R[rid])[k + 1].get("s")) - float(sec.get("s"))
    a = float(w.get("a"))
    mid = L / 2
    w.set("b", fnum(0.0)), w.set("c", fnum(0.0)), w.set("d", fnum(0.0))
    w2 = ET.Element("width", sOffset=fnum(mid), a=fnum(a), b=fnum(0.01), c="0", d="0")
    l.insert(list(l).index(w) + 1, w2)
    s = float(sec.get("s")) + mid
    return dict(desc=f"road {rid} lane {lane}: width slope 0 -> 0.01 at s={s:.2f} (value continuous)",
                expect=[dict(inv="width_record_jump", road=rid, lane=lane, s=s, kind="slope"),
                        dict(inv="G0_border", road=rid, lane=lane, n=None)],
                at=(rid, s), radius=40)


PLANTS = {
    "del_laneLink": p_del_laneLink,
    "width_shift": p_width_shift,
    "break_successor": p_break_successor,
    "curvature_jump": p_curvature_jump,
    "cut_edge": p_cut_edge,
    "flip_laneLink": p_flip_laneLink,
    "del_connecting_lane_link": p_del_connecting_lane_link,
    "flip_road_lanes": p_flip_road_lanes,
    "needless_split": p_needless_split,
    "del_connection": p_del_connection,
    "isolated_stub": p_isolated_stub,
    "drop_driving_link": p_drop_driving_link,
    "width_slope_jump": p_width_slope_jump,
}


# ----------------------------------------------------------------------------- diff
def keyset(inv, alias=None):
    """multiset of instance keys; alias maps a planted road id to (base road id, s offset in the base)."""
    al = alias or {}
    out = collections.Counter()
    full = collections.defaultdict(list)
    for name, v in inv.items():
        for it in v["instances"]:
            rd, off = al.get(it["road"], (it["road"], 0.0))
            to = al.get(it.get("to_road"), (it.get("to_road"), 0.0))[0]
            k = (name, rd, str(it["lane"]), it.get("kind"), it.get("family"), it.get("edge"), to,
                 str(it.get("to_lane")), round((it["s"] + off) * 2) / 2)
            out[k] += 1
            full[k].append(it)
    return out, full


RULE = "file"


def run(path, osm, clip):
    net = I.Network(path)
    res = I.run_all(net, osm, I.clip_xy_of(net, clip), RULE)
    return net, res.to_json()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr", nargs="?")
    ap.add_argument("osm", nargs="?")
    ap.add_argument("--area", default=None)
    ap.add_argument("--clip", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", default=None)
    ap.add_argument("--rule", default="file", choices=["file", "rht", "lht"])
    a = ap.parse_args()
    global RULE
    RULE = a.rule
    clip = a.clip
    if a.area:
        x, o, c = I.clip_from_gate(a.area)
        a.xodr, a.osm, clip = a.xodr or x, a.osm or o, clip or c
    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, "base.xodr")
    if os.path.abspath(a.xodr) != os.path.abspath(base):
        shutil.copyfile(a.xodr, base)
    import odrgeom as G
    net0 = I.Network(base)
    osm = G.Osm(a.osm, net0.lat0, net0.lon0) if a.osm else None
    _, inv0 = run(base, osm, clip)
    k0, _ = keyset(inv0)
    report, failed = {}, []
    for name, fn in PLANTS.items():
        if a.only and name not in a.only.split(","):
            continue
        tree = ET.parse(base)
        plant = fn(tree.getroot())
        if plant is None:
            report[name] = dict(status="SKIP", why="no candidate")
            continue
        p = os.path.join(a.out, name + ".xodr")
        tree.write(p, encoding="utf-8", xml_declaration=True)
        net, inv = run(p, osm, clip)
        k1, full = keyset(inv, plant.get("alias"))
        gone = sorted(set(k0) - set(k1))
        rid, s = plant["at"]
        px, py = I._xy(net.roads[rid], s)
        ok, notes, used = True, [], set()
        new = []  # new instances, annotated with their invariant name
        for k, n in (k1 - k0).items():
            for it in full[k][-n:]:
                new.append(dict(it, _inv=k[0]))
        for e in plant["expect"]:
            hits = [i for i, it in enumerate(new)
                    if it["_inv"] == e["inv"] and it["road"] == e["road"]
                    and (e.get("lane") is None or it["lane"] == e["lane"])
                    and (e.get("kind") is None or it.get("kind") == e["kind"])
                    and (e.get("s") is None or abs(it["s"] - e["s"]) <= 0.25)]
            want = e.get("n", 1)
            used |= set(hits)
            if (want is None and not hits) or (want is not None and len(hits) != want):
                ok = False
                notes.append(f"expected {want if want is not None else '>=1'} x {e} -> got {len(hits)}")
        far, near = [], []
        for i, it in enumerate(new):
            if i in used:
                continue
            d = math.hypot(it["x"] - px, it["y"] - py)
            (near if d <= plant["radius"] else far).append(dict(it, dist=round(d, 1)))
        if far:
            ok = False
            notes.append(f"{len(far)} new instance(s) farther than {plant['radius']} m from the plant")
        st = "PASS" if ok else "FAIL"
        if not ok:
            failed.append(name)
        report[name] = dict(status=st, desc=plant["desc"], notes=notes, matched=[new[i] for i in sorted(used)],
                            collateral_near=near, unexpected_far=far, base_instances_gone=len(gone),
                            gone=[list(g) for g in gone[:20]])
        print(f"{st:4s} {name:26s} {plant['desc']}")
        for n in notes:
            print("       ", n)
        for it in near:
            print("        near:", it["_inv"], it["road"], it["lane"], it["s"], it.get("kind"), it.get("dist"))
        for it in far[:10]:
            print("        FAR :", it["_inv"], it["road"], it["lane"], it["s"], it.get("kind"), it.get("dist"))
        if gone:
            print(f"        {len(gone)} base instance(s) gone, e.g. {gone[:3]}")
    with open(os.path.join(a.out, "selftest.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
