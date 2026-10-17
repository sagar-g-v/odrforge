"""det_link_continuity -- lane-link continuity detector for OdrForge OpenDRIVE output.

Every lane link in an OpenDRIVE network is a promise that two lane surfaces are glued together.
A correct converter therefore produces, at every linked contact:

  * the two borders of the linked lanes at the same xy position          (border gap   <= GAP_TOL)
  * the same lane width on both sides                                     (width jump   <= GAP_TOL)
  * the same border direction (no kink in the lane edge)                  (border kink  <= KINK_TOL)
  * the same reference-line direction when the ref lines touch            (ref kink     <= KINK_TOL)

and it never leaves a lane with real width hanging at a road / section end without a link (an
"open lane end" = the notch a driver or simulator falls into).

Checked link families
  J  junction laneLink        incoming road end (side that links to the junction) -> connecting road
                              contact end (<junction><connection><laneLink from to>)
  D  direct junction laneLink incoming road end -> linked road at the connection's contactPoint
                              (OpenDRIVE 1.8.0 12.6 <junction type="direct">, no connecting road)
  R  road->road lane link     connecting road start/end lane <predecessor>/<successor> -> lane of the
                              linked road at its contactPoint (covers connecting -> outgoing road)
  S  lane-section boundary    lane <successor> in section k -> lane in section k+1 of the same road
  O  laneOffset records       value / slope continuity between consecutive <laneOffset> pieces
  W  lane width records       value / slope continuity between consecutive <width> pieces of a lane
Per linked contact the border tangent is analytic, heading = hdg + atan2(t', 1 - k t); a border with
1 - k t <= 0 folds back on itself (ref curvature radius smaller than the border offset) - reported
as border_fold_{road_end | jog | turn} depending on which road folds and the connecting road shape.
Open ends
  open_lane_end_junction      plain-road driving lane at a junction end that no laneLink (default or
                              direct junction) and no connecting-road lane link reaches; its flow
                              (inflow / outflow) follows road@rule (LHT: lanes with id > 0 travel +s)
  open_lane_end_connecting    connecting-road lane with no predecessor / successor at a linked end
  open_lane_end_section       lane that ends at a section boundary with width > WIDTH_TOL and no
                              successor (or starts with width > WIDTH_TOL and no predecessor)
  unlinked_road_end           road end without any link whose ref point lies on another road's
                              carriageway (a T / continuation that was never linked)
Bookkeeping
  dangling_link               link that names a lane id that does not exist on the target
  link_inconsistent           junction laneLink and the connecting road's own lane link disagree

Thresholds (principled, not tuned to LA):
  GAP_TOL   = 0.05 m   "a few cm": exact evaluation of the stored polynomials leaves < 1 mm; 5 cm is
                       well below the 0.3 m basemap pixel and below any visible notch.
  KINK_TOL  = 3 deg    A G1-continuous fit (what clothoid_fit promises) gives < 0.5 deg; 3 deg is a
                       visible corner in a lane edge (0.5 m lateral error over 10 m).
  WIDTH_TOL = 0.10 m   a lane narrower than 10 cm at its end is a finished taper, not a notch.
  severity  minor   gap / jump in (0.05, 0.25] m or kink in (3, 10] deg
            major   gap / jump in (0.25, 1.0] m or kink > 10 deg or open end width <= 1.0 m
            critical gap / jump > 1.0 m or an open lane end wider than 1.0 m

CLI
  python det_link_continuity.py LA.xodr area.osm --out DIR [--basemap DIR] [--crops N]
writes DIR/findings.json, DIR/summary.txt and DIR/crops/*.png (worst + typical instances).
"""
import argparse
import collections
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402

GAP_TOL = 0.05
KINK_TOL = 3.0
WIDTH_TOL = 0.10
LINKED_TYPES = {"driving", "bidirectional", "entry", "exit", "onRamp", "offRamp", "connectingRamp"}


# ----------------------------------------------------------------------------- end-state helpers
def end_s(road, end):
    return 0.0 if end == "start" else road.length


def end_section(road, end):
    return road.sections[0] if end == "start" else road.sections[-1]


def _ang(a, b):
    """unsigned angle between two unit vectors, degrees"""
    c = max(-1.0, min(1.0, a[0] * b[0] + a[1] * b[1]))
    return math.degrees(math.acos(c))


def curvature(road, s):
    """exact reference-line curvature at s from the planView record (line / arc / spiral);
    at a record boundary the record that contains the contact side is used."""
    recs = road.geoms
    pick = recs[0]
    for r in recs:
        if r[0] <= s + 1e-9 and r[1] > 1e-9:
            pick = r
    if s >= road.length - 1e-9:  # end contact: last non-degenerate record
        pick = [r for r in recs if r[1] > 1e-9][-1]
    s0, L, tag, at = pick
    if tag == "line":
        return 0.0
    if tag == "arc":
        return float(at["curvature"])
    if tag == "spiral":
        c0, c1 = float(at["curvStart"]), float(at["curvEnd"])
        return c0 + (c1 - c0) * min(1.0, max(0.0, (s - s0) / L))
    # fallback: numeric derivative of the dense heading
    return float(np.interp(s, road.S, np.gradient(road.H, road.S)))


def lane_state(road, end, lid, sec=None, s=None):
    """Border geometry of lane `lid` at a road end (or at section boundary s with section sec).

    Returns dict(p=[inner xy, outer xy], d=[inner dir, outer dir], w=width, ref=xy, refd=dir,
    fold=min(1 - k t) over the two borders) with directions pointing *into* the road from the
    contact (so a smooth joint has d_A = -d_B).  Border tangent is analytic:
        dP/ds = (1 - k t) T + t' N   ->  heading = hdg + atan2(t', 1 - k t)
    with k the exact ref curvature and t' from the lane polynomials (1 cm one-sided difference
    inside the section).  1 - k t <= 0 means the border folds back on itself (curvature radius of
    the reference line smaller than the border offset).
    """
    sec = sec or end_section(road, end)
    s = end_s(road, end) if s is None else s
    if sec.lane(lid) is None:
        return None
    sgn = 1.0 if end == "start" else -1.0          # direction into the road along s
    h = min(0.01, max(1e-4, 0.5 * (sec.e - sec.s)))
    b1, b2 = road.borders_at(s, sec), road.borders_at(s + sgn * h, sec)
    rx, ry, rh = (float(v[0]) for v in road.ref(np.array([s])))
    k = curvature(road, s)
    pts, dirs, folds = [], [], []
    for i in (0, 1):
        t = b1[lid][i]
        dt = (b2[lid][i] - t) / (sgn * h)
        f = 1.0 - k * t
        a = rh + math.atan2(dt, f)
        pts.append((rx - t * math.sin(rh), ry + t * math.cos(rh)))
        dirs.append((sgn * math.cos(a), sgn * math.sin(a)))
        folds.append(f)
    refd = (sgn * math.cos(rh), sgn * math.sin(rh))
    return {"p": pts, "d": dirs, "w": abs(b1[lid][1] - b1[lid][0]), "ref": (rx, ry), "refd": refd,
            "fold": min(folds), "k": k}


def compare(A, B):
    """metrics between two linked lane states (pairing of borders chosen geometrically, which makes
    the comparison independent of contactPoint orientation and lane id sign flips)."""
    best = None
    for pair in ((0, 1), (1, 0)):
        g = [math.dist(A["p"][k], B["p"][pair[k]]) for k in (0, 1)]
        if best is None or max(g) < max(best[0]):
            best = (g, pair)
    g, pair = best
    kinks = [_ang(A["d"][k], (-B["d"][pair[k]][0], -B["d"][pair[k]][1])) for k in (0, 1)]
    ref_gap = math.dist(A["ref"], B["ref"])
    ref_kink = _ang(A["refd"], (-B["refd"][0], -B["refd"][1]))
    return {"gap_inner": g[0], "gap_outer": g[1], "gap": max(g), "width_a": A["w"], "width_b": B["w"],
            "width_jump": abs(A["w"] - B["w"]), "kink": max(kinks), "ref_gap": ref_gap,
            "ref_kink": ref_kink if ref_gap < 0.05 else None, "fold": min(A["fold"], B["fold"]),
            "curv_a": A["k"], "curv_b": B["k"]}


def severity_link(m):
    g = max(m["gap"], m["width_jump"])
    k = max(m["kink"], m["ref_kink"] or 0.0)
    if g > 1.0:
        return "critical"
    if g > 0.25 or k > 10.0 or m["fold"] <= 0:
        return "major"
    if g > GAP_TOL or k > KINK_TOL:
        return "minor"
    return None


def severity_open(w):
    return "critical" if w > 1.0 else "major"


# ----------------------------------------------------------------------------- detector
def direct_junctions(path):
    """{junction id: [(incoming road, linked road, contactPoint on the linked road, [(from, to)])]} of the
    OpenDRIVE 1.8.0 direct junctions (12.6: connection@linkedRoad instead of a connecting road)"""
    import xml.etree.ElementTree as ET
    out = {}
    for _, el in ET.iterparse(path):
        if el.tag == "junction" and el.get("type") == "direct":
            out[el.get("id")] = [(c.get("incomingRoad"), c.get("linkedRoad"), c.get("contactPoint") or "start",
                                  [(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")])
                                 for c in el.findall("connection")]
        if el.tag == "road":
            el.clear()
    return out


def travels_plus_s(road, lid):
    """road@rule: RHT lanes with id < 0 travel +s, LHT lanes with id > 0"""
    return (lid < 0) != ((road.el.get("rule") or "RHT").upper() == "LHT")


class Detector:
    def __init__(self, od, direct=None):
        self.od = od
        self.direct = direct or {}
        self.findings = []
        self.links_checked = collections.Counter()
        self.all_metrics = collections.defaultdict(list)  # family -> [gap]
        self.seen = set()

    def add(self, kind, roads, xy, sev, **metrics):
        f = {"id": len(self.findings), "kind": kind, "roads": roads, "x": round(xy[0], 2), "y": round(xy[1], 2),
             "severity": sev}
        f.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in metrics.items()})
        self.findings.append(f)

    def junction_end(self, road, jid, near_xy):
        ends = [e for e, l in (("start", road.pred), ("end", road.succ)) if l and l.type == "junction" and l.id == jid]
        if not ends:
            return None
        if len(ends) == 1:
            return ends[0]
        return min(ends, key=lambda e: math.dist(lane_ref_xy(road, e), near_xy))

    def check_pair(self, family, ra, ea, la, rb, eb, lb, extra=None):
        key = tuple(sorted([(ra.id, ea, la), (rb.id, eb, lb)]))
        if key in self.seen:
            return
        self.seen.add(key)
        A, B = lane_state(ra, ea, la), lane_state(rb, eb, lb)
        if A is None or B is None:
            missing = (ra.id, la) if A is None else (rb.id, lb)
            xy = (B or A)["p"][0] if (A or B) else lane_ref_xy(ra, ea)
            self.add("dangling_link", [ra.id, rb.id], xy, "major", family=family, missing_lane=list(missing))
            return
        self.links_checked[family] += 1
        m = compare(A, B)
        if m["fold"] <= 0:  # which side folds: the plain road's own end or the connecting road
            fr = ra if A["fold"] <= B["fold"] else rb
            m["fold_road"] = fr.id
            m["fold_on_connecting"] = fr.junction != "-1"
        for rr in (ra, rb):  # shape of the connecting road (tells a lateral jog from a too-tight turn)
            if rr.junction != "-1":
                m.update(conn_shape(rr))
                break
        self.all_metrics[family].append(m)
        sev = severity_link(m)
        if sev:
            xy = A["p"][0] if m["gap_inner"] >= m["gap_outer"] else A["p"][1]
            lt = (ra.section_at(end_s(ra, ea)).lane(la).type if ea else None)
            self.add("link_discontinuity", [ra.id, rb.id], xy, sev, family=family,
                     lanes=[la, lb], ends=[ea, eb], lane_type=lt,
                     junction=[ra.junction, rb.junction], **{k: v for k, v in m.items()}, **(extra or {}))

    def _record_join(self, fam, r, lane, prev, cur, s_abs, s_base):
        """C0 / slope continuity between two consecutive cubic records (laneOffset or lane width)."""
        (p0, (a, b, c, d)), (q0, (a2, b2, _, _)) = prev, cur
        ds = cur[0] - p0
        v = a + b * ds + c * ds ** 2 + d * ds ** 3
        sl = b + 2 * c * ds + 3 * d * ds ** 2
        jump = abs(v - a2)
        kink = abs(math.degrees(math.atan(sl) - math.atan(b2)))
        self.links_checked[fam] += 1
        self.all_metrics[fam].append({"gap": jump, "width_jump": jump if fam == "W" else 0.0, "kink": kink})
        if jump <= GAP_TOL and kink <= KINK_TOL:
            return
        sev = "critical" if jump > 1.0 else "major" if (jump > 0.25 or kink > 10) else "minor"
        s = min(max(s_abs, 0.0), r.length)
        b_ = r.borders_at(s)
        t = b_[lane.id][1] if lane is not None and lane.id in b_ else b_[0][0]
        x, y = (float(q[0]) for q in r.point(np.array([s]), np.array([t])))
        self.add("record_discontinuity", [r.id], (x, y), sev, family=fam, s=s,
                 lane=lane.id if lane is not None else 0, lane_type=lane.type if lane is not None else None,
                 gap=jump, width_jump=jump if fam == "W" else 0.0, kink=kink, ref_gap=0.0, ref_kink=None,
                 value_before=v, value_after=a2, slope_before=sl, slope_after=b2)

    def run(self):
        od = self.od
        # ---- J: junction laneLinks (incoming road -> connecting road)
        reach = collections.defaultdict(set)  # (plain road id, end) -> lane ids reached by any link
        for jid, conns in od.junctions.items():
            for c in conns:
                inc, con = od.roads.get(c["incoming"]), od.roads.get(c["connecting"])
                if inc is None or con is None:
                    continue
                ce = c["contact"]
                ie = self.junction_end(inc, jid, lane_ref_xy(con, ce))
                own_lk = con.pred if ce == "start" else con.succ
                if ie is not None and own_lk is not None and own_lk.id == inc.id and own_lk.contact:
                    # a loop road with both ends in this junction: the connecting road names the end
                    ie = own_lk.contact
                if ie is None:
                    # incoming road does not link to this junction at all
                    self.add("link_inconsistent", [inc.id, con.id], lane_ref_xy(con, ce), "major",
                             family="J", detail="incoming road has no link to junction %s" % jid)
                    continue
                csec = end_section(con, ce)
                for f, t in c["lanes"]:
                    reach[(inc.id, ie)].add(f)
                    cl = csec.lane(t)
                    if cl is not None:
                        own = cl.pred if ce == "start" else cl.succ
                        lk = con.pred if ce == "start" else con.succ
                        if own != f or lk is None or lk.id != inc.id:
                            self.add("link_inconsistent", [inc.id, con.id], lane_ref_xy(con, ce), "minor",
                                     family="J", lanes=[f, t], connecting_lane_link=own,
                                     detail="junction laneLink vs connecting-road lane link")
                    self.check_pair("J", inc, ie, f, con, ce, t, {"junction_id": jid})
        # ---- D: direct junctions (1.8.0 12.6): incoming road lane -> linked road lane, no connecting road
        for jid, conns in self.direct.items():
            for inc_id, lin_id, cp, pairs in conns:
                inc, lin = od.roads.get(inc_id), od.roads.get(lin_id)
                if inc is None or lin is None:
                    continue
                ie = self.junction_end(inc, jid, lane_ref_xy(lin, cp))
                if ie is None:
                    self.add("link_inconsistent", [inc.id, lin.id], lane_ref_xy(lin, cp), "major",
                             family="D", detail="incoming road has no link to direct junction %s" % jid)
                    continue
                for f, t in pairs:
                    reach[(inc.id, ie)].add(f)
                    reach[(lin.id, cp)].add(t)
                    self.check_pair("D", inc, ie, f, lin, cp, t, {"junction_id": jid})
        # ---- R: road -> road lane links (connecting roads and any plain road->road links)
        for r in od.roads.values():
            for end, lk in (("start", r.pred), ("end", r.succ)):
                if lk is None or lk.type != "road":
                    continue
                other = od.roads.get(lk.id)
                if other is None:
                    self.add("dangling_link", [r.id, lk.id], lane_ref_xy(r, end), "critical", family="R",
                             detail="linked road missing")
                    continue
                oe = lk.contact or ("start" if end == "end" else "end")
                sec = end_section(r, end)
                for l in sec.lanes():
                    tgt = l.pred if end == "start" else l.succ
                    if tgt is None:
                        w = lane_width_at(r, end, l.id)
                        if w > WIDTH_TOL and (r.junction != "-1" or l.type in LINKED_TYPES):
                            st = lane_state(r, end, l.id)
                            self.add("open_lane_end_connecting" if r.junction != "-1" else "open_lane_end_road",
                                     [r.id, other.id], st["p"][1], severity_open(w), lane=l.id, end=end,
                                     width=w, lane_type=l.type)
                        continue
                    if other.junction == "-1":
                        reach[(other.id, oe)].add(tgt)
                    self.check_pair("R", r, end, l.id, other, oe, tgt)
        # ---- S: lane-section boundaries inside a road
        for r in od.roads.values():
            for k in range(len(r.sections) - 1):
                a, b = r.sections[k], r.sections[k + 1]
                s = b.s
                for l in a.lanes():
                    wa = float(l.width(np.array([a.e - a.s]))[0])
                    if l.succ is None or b.lane(l.succ) is None:
                        if abs(wa) > WIDTH_TOL and l.type in LINKED_TYPES:
                            st = section_state(r, a, l.id, s, -1)
                            self.add("open_lane_end_section", [r.id], st["p"][1], severity_open(abs(wa)), lane=l.id,
                                     s=s, width=abs(wa), lane_type=l.type, dangling=l.succ)
                        continue
                    A, B = section_state(r, a, l.id, s, -1), section_state(r, b, l.succ, s, +1)
                    self.links_checked["S"] += 1
                    m = compare(A, B)
                    self.all_metrics["S"].append(m)
                    sev = severity_link(m)
                    if sev:
                        self.add("link_discontinuity", [r.id], A["p"][0], sev, family="S", lanes=[l.id, l.succ],
                                 s=s, lane_type=l.type, **m)
                linked_into = {l.succ for l in a.lanes() if l.succ is not None}
                for l in b.lanes():
                    wb = float(l.width(np.array([0.0]))[0])
                    if (l.pred is None or a.lane(l.pred) is None) and l.id not in linked_into \
                            and abs(wb) > WIDTH_TOL and l.type in LINKED_TYPES:
                        st = section_state(r, b, l.id, s, +1)
                        self.add("open_lane_start_section", [r.id], st["p"][1], severity_open(abs(wb)), lane=l.id,
                                 s=s, width=abs(wb), lane_type=l.type)
        # ---- O / W: laneOffset records and lane width records (the polynomial pieces that the
        #      section-level links glue together must themselves be C0 and G1-continuous)
        for r in od.roads.values():
            recs = r.offsets
            for i in range(1, len(recs)):
                self._record_join("O", r, None, recs[i - 1], recs[i], recs[i][0], 0.0)
            for sec in r.sections:
                for l in sec.lanes():
                    for i in range(1, len(l.widths)):
                        self._record_join("W", r, l, l.widths[i - 1], l.widths[i], sec.s + l.widths[i][0], sec.s)
        # ---- open lane ends of plain roads at junctions
        for r in od.plain_roads():
            for end, lk in (("start", r.pred), ("end", r.succ)):
                if lk is None or lk.type != "junction":
                    continue
                for l in end_section(r, end).lanes():
                    if l.type not in LINKED_TYPES or l.id in reach[(r.id, end)]:
                        continue
                    w = lane_width_at(r, end, l.id)
                    if w <= WIDTH_TOL:
                        continue
                    st = lane_state(r, end, l.id)
                    # traffic direction of the lane (road@rule decides which side follows +s; bidirectional: both)
                    if l.type == "bidirectional":
                        flow, sev = "bidirectional", "minor"
                    else:
                        flow = "inflow" if travels_plus_s(r, l.id) == (end == "end") else "outflow"
                        sev = severity_open(w)
                    self.add("open_lane_end_junction", [r.id], st["p"][1], sev, lane=l.id, end=end,
                             width=w, lane_type=l.type, junction_id=lk.id, flow=flow,
                             lanes_at_end=len(end_section(r, end).lanes()),
                             lanes_reached=len(reach[(r.id, end)]))
        # ---- road ends without any link that sit on another road's carriageway
        for r in od.plain_roads():
            for end, lk in (("start", r.pred), ("end", r.succ)):
                if lk is not None:
                    continue
                x, y = lane_ref_xy(r, end)
                hit = None
                for o in od.roads_near(x, y, 30):
                    if o.id == r.id:
                        continue
                    s, t, d = o.project(x, y)
                    if s <= 1e-3 or s >= o.length - 1e-3:
                        # beyond the other road's ends: only an end-to-end touch counts
                        if d > 0.5:
                            continue
                    b = o.borders_at(s)
                    ts = [v for lid, p in b.items() if lid != 0 and o.section_at(s).lane(lid).type in LINKED_TYPES
                          for v in p]
                    # same level only: an end above/below another road is a grade separation (or
                    # the map boundary cutting a bridge), not a missed link
                    dz = abs(float(r.z(np.array([end_s(r, end)]))[0]) - float(o.z(np.array([s]))[0]))
                    if ts and min(ts) - 0.05 <= t <= max(ts) + 0.05 and dz < 1.0:
                        hit = (o, s, t, d)
                        break
                if hit:
                    o, s, t, d = hit
                    self.add("unlinked_road_end", [r.id, o.id], (x, y), "major", end=end, other_s=s, other_t=t,
                             other_is_connecting=o.junction != "-1")
        return self.findings


def conn_shape(road):
    """end-to-end shape of a (connecting) road in its start frame: heading change, lateral and
    longitudinal displacement, peak |curvature| of the reference line"""
    x0, y0, h0 = (float(v[0]) for v in road.ref(np.array([0.0])))
    x1, y1, h1 = (float(v[0]) for v in road.ref(np.array([road.length])))
    dx, dy = x1 - x0, y1 - y0
    kmax = 0.0
    for _, _, tag, at in road.geoms:
        for key in ("curvature", "curvStart", "curvEnd"):
            if key in at:
                kmax = max(kmax, abs(float(at[key])))
    return {"conn_road": road.id, "conn_len": road.length,
            "conn_dh": math.degrees((h1 - h0 + math.pi) % (2 * math.pi) - math.pi),
            "conn_lat": -dx * math.sin(h0) + dy * math.cos(h0), "conn_lon": dx * math.cos(h0) + dy * math.sin(h0),
            "conn_kmax": kmax}


def lane_ref_xy(road, end):
    x, y, _ = road.ref(np.array([end_s(road, end)]))
    return float(x[0]), float(y[0])


def lane_width_at(road, end, lid):
    b = road.borders_at(end_s(road, end), end_section(road, end))
    return abs(b[lid][1] - b[lid][0]) if lid in b else 0.0


def section_state(road, sec, lid, s, direction):
    """lane state at section boundary s evaluated with `sec`; direction -1 = looking back into sec"""
    return lane_state(road, "end" if direction < 0 else "start", lid, sec=sec, s=s)


# ----------------------------------------------------------------------------- reporting
def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def summarize(det, findings):
    out = []
    out.append("link_continuity detector -- thresholds: gap/width jump > %.2f m, kink > %.0f deg, open lane width > %.2f m"
               % (GAP_TOL, KINK_TOL, WIDTH_TOL))
    out.append("links checked: " + ", ".join("%s=%d" % kv for kv in sorted(det.links_checked.items())))
    for fam, ms in sorted(det.all_metrics.items()):
        g = [m["gap"] for m in ms]
        wj = [m["width_jump"] for m in ms]
        kk = [m["kink"] for m in ms]
        out.append("  %s: gap p50=%.3f p90=%.3f p99=%.3f max=%.2f | width jump p99=%.3f max=%.2f | border kink p99=%.1f max=%.1f deg"
                   % (fam, pct(g, 50), pct(g, 90), pct(g, 99), max(g), pct(wj, 99), max(wj), pct(kk, 99), max(kk)))
    out.append("total findings: %d" % len(findings))
    c = collections.Counter((f["subclass"], f["severity"]) for f in findings)
    for k, v in sorted(c.items()):
        out.append("  %-38s %-9s %5d" % (k[0], k[1], v))
    ld = [f for f in findings if f["kind"] == "link_discontinuity"]
    if ld:
        out.append("link_discontinuity breakdown (dominant metric):")
        sub = collections.Counter(classify(f) for f in ld)
        for k, v in sub.most_common():
            out.append("  %-34s %5d" % (k, v))
        g = [f["gap"] for f in ld]
        out.append("  gap histogram (m): " + " ".join(
            "%s:%d" % (lab, sum(1 for x in g if lo < x <= hi)) for lab, lo, hi in
            (("<=0.05", -1, 0.05), ("0.05-0.25", 0.05, 0.25), ("0.25-1", 0.25, 1), ("1-3", 1, 3), (">3", 3, 1e9))))
    return "\n".join(out)


def classify(f):
    if f["kind"] == "open_lane_end_junction":
        return f["kind"] + ":" + f["flow"]
    if f["kind"] == "record_discontinuity":
        return f["family"] + (":value_jump" if f["gap"] > GAP_TOL else ":slope_kink")
    if f["kind"] != "link_discontinuity":
        return f["kind"]
    fam = f["family"]
    if f.get("fold", 1) <= 0:
        if not f.get("fold_on_connecting", True):
            return fam + ":border_fold_road_end"
        return fam + (":border_fold_jog" if abs(f.get("conn_dh", 0)) < 35 else ":border_fold_turn")
    if f["gap"] > GAP_TOL and f["width_jump"] > GAP_TOL:
        return fam + ":gap+width_jump"
    if f["gap"] > GAP_TOL:
        return fam + (":ref_offset" if f["ref_gap"] > GAP_TOL else ":lateral_offset")
    return fam + (":ref_kink" if (f["ref_kink"] or 0) > KINK_TOL else ":border_kink")


SEV_RANK = {"critical": 3, "major": 2, "minor": 1}


def score(f):
    """ranking: severity first, then magnitude (m; a kink counts 1 m per 20 deg)"""
    return SEV_RANK[f["severity"]] * 100 + max(f.get("gap", 0), f.get("width_jump", 0), f.get("width", 0)) + max(f.get("kink", 0), f.get("ref_kink") or 0) / 20


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr")
    ap.add_argument("osm")
    ap.add_argument("--out", required=True)
    ap.add_argument("--basemap")
    ap.add_argument("--crops", type=int, default=12, help="number of evidence crops (0 = none)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    od = G.OpenDrive(a.xodr)
    det = Detector(od, direct_junctions(a.xodr))
    findings = det.run()
    for f in findings:
        f["subclass"] = classify(f)
    findings.sort(key=lambda f: -score(f))
    for i, f in enumerate(findings):
        f["id"] = i
    json.dump(findings, open(os.path.join(a.out, "findings.json"), "w"), indent=1)
    txt = summarize(det, findings)
    open(os.path.join(a.out, "summary.txt"), "w").write(txt + "\n")
    print(txt)
    if a.crops and findings:
        osm = G.Osm(a.osm, od.lat0, od.lon0)
        cd = os.path.join(a.out, "crops")
        os.makedirs(cd, exist_ok=True)
        # evidence: the worst instance of every subclass, then a typical (median) one, then the
        # overall worst until the budget is used
        bysub = collections.defaultdict(list)
        for f in findings:
            bysub[f["subclass"]].append(f)
        subs = sorted(bysub, key=lambda k: -SEV_RANK[bysub[k][0]["severity"]] * 1000 - len(bysub[k]))
        pick, ids = [], set()
        for cand in [bysub[k][0] for k in subs] + [bysub[k][len(bysub[k]) // 2] for k in subs] + findings:
            if len(pick) >= a.crops:
                break
            if cand["id"] not in ids:
                pick.append(cand), ids.add(cand["id"])
        for f in pick:
            half = 15 if score(f) < 1 else 30
            p = os.path.join(cd, "%03d_%s_%s.png" % (f["id"], f["subclass"].replace(":", "-"), "_".join(f["roads"])))
            G.crop(od, osm, (f["x"], f["y"]), half, p, basemap=a.basemap, marks=[(f["x"], f["y"], str(f["id"]))],
                   title="#%d %s %s roads %s" % (f["id"], f["subclass"], f["severity"], ",".join(f["roads"])))
            f["crop"] = p
        json.dump(findings, open(os.path.join(a.out, "findings.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
