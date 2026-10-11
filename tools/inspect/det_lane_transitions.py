#!/usr/bin/env python
"""det_lane_transitions.py -- detector for lane-count / lane-width transitions in an OdrForge xodr.

Defect class "lane_transitions": wherever the number or the width of lanes changes (an OSM lanes tag
change between consecutive ways, a laneSection boundary, a width taper, a gore stretch), check

(a) LOCATION -- per road and carriageway side, the converter's driving-lane count n(s) (lanes >= 1.25 m
    wide) is compared with the count the OSM way covering s explicitly tags (lanes / lanes:forward /
    lanes:backward, same per-direction rules as src/lanes.cpp).  Taper intervals (+-1 m) are "don't
    care" (a lane is half-open there).  Every contiguous run of disagreement longer than LOC_TOL = 5 m
    is a finding:  shifted_transition (a taper exists but sits beside the OSM node),
    missing_transition (the OSM change has no taper in the road at all), count_mismatch (the road
    disagrees with the tag along a way without a change node nearby).  major when > 15 m.
    Road tapers with no explicit OSM change nearby are reported as default_count_change (one of the
    ways has no lanes tag, converter default differs), unexplained_transition / extra_transition.
    Tapers outside the OSM coverage (gore stretches) are not location-checked.
(b) SIDE -- the added / dropped lane must sit where OSM says:
    * at an OSM tag change: turn:lanes of the wider way -- the run of pure left-ish pocket lanes
      (left / slight_left / merge_to_right) at the left end must be added / dropped at the inner end,
      the run of pure right-ish lanes (right / slight_right / merge_to_left) at the right end at the
      outer end (shared lanes like through;right are not pockets); else a single *_link way attached at
      the node (its side).  No evidence -> no finding.
    * in a gore stretch: a lane dropped (merge) must be one fed by the branch road, not by the trunk
      (the incoming road feeding most lanes) while a branch-fed lane survives; a lane added (diverge)
      must lead to the branch road, not the trunk.  (wrong_side_gore)
(c) TAPER -- an added / dropped driving lane must open / close over an adequate length.
    R = L / W, L = length over which the width goes 0 -> W (1 cm tolerance), W = full width.
    Required ratio (MUTCD 6C-1 L = W*S (S >= 45 mph) or W*S^2/60 (S < 45 mph) for merging / lane
    drops; half of that -- the MUTCD shifting taper -- for lane additions; floor 8:1, the AASHTO minimum
    urban turn-bay taper).  S = road speed limit from the xodr (class default otherwise).
    critical R < 2 (a step), major R < R_req/2, minor R < R_req.
    Also: width jump between width records of a lane, border jump of a linked lane across a section
    boundary (critical > 0.5 m, major > 0.05 m), lanes that start / end with non-zero width at a section
    boundary (abrupt_lane_start / end), and full-width driving lanes outside 2.5..4.5 m
    (major outside 2.2..5.0).
(d) CONTINUITY -- during a transition, every persistent driving lane (identity followed through lane
    links) must keep its lateral position (centre t) within JOG_TOL = 0.2 m, because a zero-movement
    solution exists whenever the changing lane is the outer one or the road is one-way (move the
    laneOffset).  For an inner add / drop on a two-way road some lanes must shift; then only the
    shifting rate is checked against the shifting taper R_req/2 (through_lane_shift_rate).
    through_lane_jog major when > 1 m.
(e) LANEOFFSET -- laneOffset must be continuous (value jump > 0.05 m between records).
(f) CONTACT -- at connecting-road contacts where a linked lane changes width, the lane borders must
    meet in world coordinates (gap > 0.10 m).

A correct converter (taper on the OSM node, on the indicated side, speed-appropriate length, through
lanes untouched, continuous offsets and borders) produces zero findings.

CLI:  python det_lane_transitions.py XODR OSM --out DIR [--basemap DIR] [--crops N]
Writes DIR/findings.json (sorted worst first), DIR/summary.txt and (with --crops) DIR/crops/*.png.
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

# ------------------------------------------------------------------ thresholds (see module doc)
DS = 0.25                 # sampling step for widths (m)
DSL = 0.5                 # sampling step for lane counts (m)
LOC_TOL = 5.0             # m of count disagreement tolerated next to an OSM node
LOC_MAJOR = 15.0
MATCH_RADIUS = 80.0       # m: OSM change and road taper farther apart are unrelated
STEP_TOL = 0.05           # m: width / offset / border jump that is a visible notch
STEP_CRIT = 0.5
JOG_TOL = 0.20            # m: lateral movement of a persistent lane during a transition
JOG_MAJOR = 1.0
CONTACT_TOL = 0.10        # m: border gap at a road contact
W_OK = (2.5, 4.5)         # sane driving lane width
W_HARD = (2.2, 5.0)
W_PRESENT = 1.25          # a lane counts as present when at least this wide
R_FLOOR = 8.0
DRIVE = {"driving", "bidirectional", "entry", "exit", "onRamp", "offRamp", "connectingRamp"}
COUNT = DRIVE - {"bidirectional"}   # lanes counted against OSM lanes:forward / backward (both_ways excluded)
DEFAULT_MPH = {"motorway": 65, "trunk": 55, "motorway_link": 45, "trunk_link": 40, "primary": 35,
               "secondary": 35, "tertiary": 30, "residential": 25, "service": 15}
LINKS = {"motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
RIGHTISH = {"right", "slight_right", "sharp_right", "merge_to_left"}
LEFTISH = {"left", "slight_left", "sharp_left", "merge_to_right"}


def required_ratio(mph, kind):
    drop = mph if mph >= 45 else mph * mph / 60.0
    return max(R_FLOOR, drop if kind == "drop" else drop / 2.0)


# ------------------------------------------------------------------ OSM helpers
def osm_counts(tags):
    """explicit (forward, backward) driving lane counts, None without a lanes tag (src/lanes.cpp rules)"""
    def i(k):
        try:
            return int(tags[k].split(";")[0])
        except (KeyError, ValueError):
            return None
    hw, ow = tags.get("highway", ""), tags.get("oneway", "")
    oneway = ow in ("yes", "true", "1", "-1", "reverse") or tags.get("junction") in ("roundabout", "circular") \
        or (hw in ("motorway", "motorway_link") and ow != "no")
    tot, f, b, both = i("lanes"), i("lanes:forward"), i("lanes:backward"), i("lanes:both_ways") or 0
    if oneway:
        if tot is None:
            return None
        return (tot, 0) if ow not in ("-1", "reverse") else (0, tot)
    if f is not None and b is not None:
        return (f, b)
    if tot is None:
        return None
    rem = max(2, tot - both)
    if f is not None:
        return (f, max(1, rem - f))
    if b is not None:
        return (max(1, rem - b), b)
    return ((rem + 1) // 2, rem - (rem + 1) // 2)


def turn_list(tags, direction, oneway):
    v = tags.get("turn:lanes") if oneway else (tags.get("turn:lanes:" + direction))
    return [set(x.split(";")) for x in v.split("|")] if v else None


# ------------------------------------------------------------------ road view
def side_lanes(sec, side):
    """lanes of one side, inner -> outer"""
    return sec.right if side == "R" else sec.left


class RoadView:
    """sampled widths, lane chains (identity through lane links), counts"""

    def __init__(self, r, osm):
        self.r = r
        rht = r.el.get("rule", "RHT") != "LHT"
        self.tsign = {"R": 1 if rht else -1, "L": -1 if rht else 1}   # traffic along +s ?
        self.hw = next((osm.ways[w][1]["highway"] for w in r.osm_ways if w in osm.ways and "highway" in osm.ways[w][1]),
                       None)
        self.mph = self._speed()
        self.samples = []
        for sec in r.sections:
            n = max(2, int(math.ceil((sec.e - sec.s) / DS)) + 1)
            s = np.linspace(sec.s, sec.e - 1e-6, n)
            self.samples.append((sec, s, r.lane_offset(s), {l.id: l.width(s - sec.s) for l in sec.lanes()}))
        # chains
        self.chain = []          # per section: {lane id: chain id}
        nxt = 0
        for k, sec in enumerate(r.sections):
            cur = {}
            for l in sec.lanes():
                if k and l.pred is not None and l.pred in self.chain[k - 1] and \
                        r.sections[k - 1].lane(l.pred) is not None and r.sections[k - 1].lane(l.pred).succ in (l.id, None):
                    cur[l.id] = self.chain[k - 1][l.pred]
                else:
                    cur[l.id] = nxt
                    nxt += 1
            self.chain.append(cur)

    def _speed(self):
        t = self.r.el.find("type")
        if t is not None and t.find("speed") is not None:
            sp = t.find("speed")
            try:
                v = float(sp.get("max"))
                u = sp.get("unit", "m/s")
                return v if u == "mph" else v / 1.609344 if u == "km/h" else v / 0.44704
            except (TypeError, ValueError):
                pass
        return DEFAULT_MPH.get(self.hw, 30)

    def sec_index(self, s):
        k = 0
        for i, sec in enumerate(self.r.sections):
            if sec.s <= s + 1e-9:
                k = i
        return k

    def counts(self, side, ss):
        out = np.zeros(len(ss), int)
        for i, sv in enumerate(ss):
            k = self.sec_index(sv)
            sec = self.r.sections[k]
            out[i] = sum(1 for l in side_lanes(sec, side)
                         if l.type in COUNT and float(l.width(np.array([sv - sec.s]))[0]) >= W_PRESENT)
        return out

    def chain_tmid(self, ss):
        """{chain id: (mask, t centre)} over samples ss, driving lanes only"""
        res = {}
        for i, sv in enumerate(ss):
            k = self.sec_index(sv)
            sec = self.r.sections[k]
            b = self.r.borders_at(float(sv), sec)
            for l in sec.lanes():
                if l.type not in DRIVE:
                    continue
                c = self.chain[k][l.id]
                if c not in res:
                    res[c] = (np.zeros(len(ss), bool), np.zeros(len(ss)), np.zeros(len(ss)))
                res[c][0][i] = True
                res[c][1][i] = 0.5 * (b[l.id][0] + b[l.id][1])
                res[c][2][i] = abs(b[l.id][1] - b[l.id][0])
        return res


# ------------------------------------------------------------------ the detector
class Detector:
    def __init__(self, od, osm):
        self.od, self.osm = od, osm
        self.findings = []
        self.stats = collections.Counter()
        self.node_ways = collections.defaultdict(list)
        for wid, (nds, tg) in osm.ways.items():
            if "highway" in tg:
                for k, n in enumerate(nds):
                    self.node_ways[n].append((wid, k))
        self.views = {}
        # junction feeders: (road, 'start'|'end') -> {lane id: other road id}
        self.contacts = collections.defaultdict(dict)
        for c in od.connecting_roads():
            for my_end, mine, other in (("start", c.pred, c.succ), ("end", c.succ, c.pred)):
                if mine is None or other is None or mine.type != "road" or other.type != "road":
                    continue
                for sec in (c.sections[0], c.sections[-1]):
                    pass
                sec = c.sections[0] if my_end == "start" else c.sections[-1]
                for l in sec.lanes():
                    tgt = l.pred if my_end == "start" else l.succ
                    if tgt is not None:
                        self.contacts[(mine.id, mine.contact)].setdefault(tgt, set()).add(other.id)

    def view(self, r):
        if r.id not in self.views:
            self.views[r.id] = RoadView(r, self.osm)
        return self.views[r.id]

    def add(self, check, sub, sev, roads, x, y, **m):
        self.findings.append(dict(id=len(self.findings), check=check, sub=sub, severity=sev,
                                  roads=[str(r) for r in roads], x=round(float(x), 2), y=round(float(y), 2),
                                  **{k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items()}))

    # -------------------------------------------------------- tapers
    def lane_events(self, v):
        r = v.r
        ev = []
        for k, (sec, s, off, W) in enumerate(v.samples):
            for side in ("R", "L"):
                lanes = side_lanes(sec, side)
                drv = [l for l in lanes if l.type in COUNT]
                for rank, l in enumerate(lanes):
                    if l.type not in DRIVE:
                        continue
                    w = W[l.id]
                    full = float(np.max(w))
                    if full < 1.0 or float(np.min(w)) > 0.5 * full:
                        continue
                    lo, hi = 1e-4, full - 1e-4
                    above = w >= 0.5 * full
                    for e in np.nonzero(np.diff(above.astype(int)))[0]:
                        grow_s = bool(above[e + 1])
                        a = e
                        while a > 0 and (w[a] > lo if grow_s else w[a] < hi):
                            a -= 1
                        b = e + 1
                        while b < len(s) - 1 and (w[b] < hi if grow_s else w[b] > lo):
                            b += 1
                        L = float(s[b] - s[a])
                        seg = slice(a, b + 1)
                        slope = np.abs(np.gradient(w[seg], s[seg])) if b - a >= 2 else np.array([full / max(L, 1e-3)])
                        kind = "add" if grow_s == (v.tsign[side] > 0) else "drop"
                        drank = drv.index(l) if l in drv else 0   # a centre two-way lane counts as inner
                        ev.append(dict(k=k, sec=sec, side=side, lane=l.id, chain=v.chain[k][l.id], drank=drank,
                                       ndrv=len(drv), W=full, sa=float(s[a]), sb=float(s[b]), L=L, kind=kind,
                                       peak=float(np.max(slope)), s_mid=float(s[e])))
        return ev

    def check_road(self, v):
        r, mph, hw = v.r, v.mph, v.hw
        base = dict(mph=round(mph, 1), highway=hw, junction=r.junction)
        # width record discontinuities inside a section
        for sec, s, off, W in v.samples:
            for l in sec.lanes():
                for i in range(1, len(l.widths)):
                    s0 = l.widths[i][0]
                    if s0 <= 1e-6 or s0 >= sec.e - sec.s - 1e-6:
                        continue
                    a, b = (float(l.width(np.array([s0 + d]))[0]) for d in (-1e-4, 1e-4))
                    if abs(a - b) > STEP_TOL:
                        x, y = r.point(sec.s + s0, 0.0)
                        self.add("c_taper", "width_step_in_section", "critical" if abs(a - b) > STEP_CRIT else "major",
                                 [r.id], x, y, s=sec.s + s0, lane=l.id, lane_type=l.type, jump=abs(a - b), **base)
        # section boundaries
        for k in range(1, len(v.samples)):
            pa, pb = v.samples[k - 1][0], v.samples[k][0]
            sB = pb.s
            bA, bB = r.borders_at(sB - 1e-4, pa), r.borders_at(sB, pb)
            for side in ("R", "L"):
                la, lb = side_lanes(pa, side), side_lanes(pb, side)
                ida, idb = {l.id for l in la}, {l.id for l in lb}
                for l in la:
                    wend = abs(bA[l.id][1] - bA[l.id][0])
                    if l.succ is not None and l.succ in idb:
                        j = max(abs(bA[l.id][0] - bB[l.succ][0]), abs(bA[l.id][1] - bB[l.succ][1]))
                        if j > STEP_TOL:
                            wa, wb = wend, abs(bB[l.succ][1] - bB[l.succ][0])
                            x, y = r.point(sB, 0.5 * (bA[l.id][0] + bA[l.id][1]))
                            self.add("c_taper", "border_step_at_section", "critical" if j > STEP_CRIT else "major",
                                     [r.id], x, y, s=sB, lane=l.id, to_lane=l.succ, lane_type=l.type, jump=j,
                                     width_before=wa, width_after=wb, **base)
                    elif wend > 0.10:
                        x, y = r.point(sB, 0.5 * (bA[l.id][0] + bA[l.id][1]))
                        self.add("c_taper", "abrupt_lane_end", "critical" if wend > STEP_CRIT else "major", [r.id], x, y,
                                 s=sB, lane=l.id, lane_type=l.type, jump=wend, **base)
                for l in lb:
                    w0 = abs(bB[l.id][1] - bB[l.id][0])
                    if not (l.pred is not None and l.pred in ida) and w0 > 0.10:
                        x, y = r.point(sB, 0.5 * (bB[l.id][0] + bB[l.id][1]))
                        self.add("c_taper", "abrupt_lane_start", "critical" if w0 > STEP_CRIT else "major", [r.id], x, y,
                                 s=sB, lane=l.id, lane_type=l.type, jump=w0, **base)
        # laneOffset continuity
        for i in range(1, len(r.offsets)):
            s0 = r.offsets[i][0]
            a, b = (float(r.lane_offset(np.array([s0 + d]))[0]) for d in (-1e-4, 1e-4))
            if abs(a - b) > STEP_TOL:
                x, y = r.point(s0, 0.0)
                self.add("e_laneoffset", "offset_step", "critical" if abs(a - b) > STEP_CRIT else "major", [r.id], x, y,
                         s=s0, jump=abs(a - b), **base)
        # tapers
        events = self.lane_events(v)
        for ev in events:
            R = ev["L"] / max(ev["W"], 1e-3)
            req = required_ratio(mph, ev["kind"])
            ev["R"], ev["R_req"] = R, req
            self.stats["taper_events"] += 1
            sev = "critical" if R < 2.0 else "major" if R < req / 2 else "minor" if R < req else None
            if sev:
                sm = 0.5 * (ev["sa"] + ev["sb"])
                b = r.borders_at(sm)
                x, y = r.point(sm, 0.5 * sum(b[ev["lane"]]))
                self.add("c_taper", "short_taper", sev, [r.id], x, y, s=sm, s_from=ev["sa"], s_to=ev["sb"],
                         lane=ev["lane"], kind=ev["kind"], width=ev["W"], taper_len=ev["L"], ratio=R, ratio_req=req,
                         required_len=req * ev["W"], peak_slope=ev["peak"], **base)
        # driving lane widths away from tapers
        for sec, s, off, W in v.samples:
            for l in sec.lanes():
                if l.type not in DRIVE:
                    continue
                w = W[l.id]
                full = float(np.max(w))
                hold = w[w >= 0.95 * full]
                if full < 1.0 or hold.size == 0:
                    continue
                bad = float(np.max(hold)) if np.max(hold) > W_OK[1] else float(np.min(hold)) if np.min(hold) < W_OK[0] else None
                if bad is not None:
                    hard = bad > W_HARD[1] or bad < W_HARD[0]
                    sm = 0.5 * (sec.s + sec.e)
                    x, y = r.point(sm, 0.0)
                    self.add("c_taper", "lane_width_out_of_range", "major" if hard else "minor", [r.id], x, y, s=sm,
                             lane=l.id, width=bad, **base)
        # continuity of persistent lanes
        if r.junction == "-1":
            done = set()
            for ev in events:  # one check per simultaneous transition (lanes added / dropped together)
                key = (ev["side"], round(ev["sa"]), round(ev["sb"]))
                if key not in done:
                    done.add(key)
                    self.check_continuity(v, ev, events, base)
        return events

    def check_continuity(self, v, ev, events, base):
        r = v.r
        a, b = max(0.0, ev["sa"] - 5.0), min(r.length, ev["sb"] + 5.0)
        ss = np.arange(a, b + 1e-9, DS)
        if len(ss) < 3:
            return
        opp = "L" if ev["side"] == "R" else "R"
        both = any(e2["side"] == opp and e2["sb"] > a and e2["sa"] < b for e2 in events)
        if both:
            return
        tm = v.chain_tmid(ss)
        moves = []
        for c, (mask, t, w) in tm.items():
            if c == ev["chain"] or not mask.all():
                continue
            if any(e2["chain"] == c for e2 in events if e2["sb"] > a and e2["sa"] < b):
                continue  # itself tapering (incl. lanes added / dropped together with this one)
            if float(w.min()) < W_PRESENT:
                continue
            mv = float(t.max() - t.min())
            rate = float(np.max(np.abs(np.gradient(t, ss))))
            moves.append((mv, rate, c))
        if not moves:
            return
        mv, rate, c = max(moves)
        sec = ev["sec"]
        has_opp = any(l.type in DRIVE for l in side_lanes(sec, opp))
        outer = ev["drank"] == ev["ndrv"] - 1
        inner = ev["drank"] == 0
        ev["jog"] = mv
        sm = 0.5 * (ev["sa"] + ev["sb"])
        x, y = r.point(sm, 0.0)
        offc = float(abs(r.lane_offset(np.array([b]))[0] - r.lane_offset(np.array([a]))[0]))
        m = dict(s=sm, lane=ev["lane"], kind=ev["kind"], event_rank=ev["drank"], n_drive=ev["ndrv"], move=mv,
                 move_rate=rate, offset_change=offc, **base)
        if outer or not has_opp or not inner:
            if mv > JOG_TOL:
                self.add("d_continuity", "through_lane_jog", "major" if mv > JOG_MAJOR else "minor", [r.id], x, y,
                         position="outer" if outer else "inner" if inner else "middle", **m)
        else:
            req = required_ratio(v.mph, "drop") / 2.0
            if mv > JOG_TOL and rate > 1.0 / req:
                self.add("d_continuity", "through_lane_shift_rate", "major" if rate > 2.0 / req else "minor", [r.id], x, y,
                         shift_ratio=1.0 / max(rate, 1e-6), shift_ratio_req=req, **m)

    # -------------------------------------------------------- OSM comparison
    def way_cover(self, r, wid):
        pl = self.osm.polyline(wid)
        if len(pl) >= 2:  # densify to 2 m so a way that leaves the road is clipped where it leaves
            seg = np.hypot(*np.diff(pl, axis=0).T)
            u = np.concatenate([[0], np.cumsum(seg)])
            uu = np.linspace(0, u[-1], max(2, int(u[-1] / 2.0) + 1))
            pl = np.stack([np.interp(uu, u, pl[:, 0]), np.interp(uu, u, pl[:, 1])], 1)
        q = [r.project(*p) for p in pl]
        q = [(s, d) for s, t, d in q if d < 15.0]
        if len(q) < 1:
            return None
        ss = [s for s, d in q]
        return (min(ss), max(ss), 1 if (len(q) < 2 or q[-1][0] >= q[0][0]) else -1)

    def check_osm(self, v, events):
        r, osm = v.r, self.osm
        events = [e for e in events if e["sec"].lane(e["lane"]).type != "bidirectional"]  # lanes:both_ways
        ways = [w for w in r.osm_ways if w in osm.ways]
        cov = {}
        for w in ways:
            c = self.way_cover(r, w)
            if c and c[1] - c[0] > 0.5:
                cov[w] = c
        # explicit tag changes between consecutive ways
        changes = []
        for wa, wb in zip(ways, ways[1:]):
            if wa not in cov or wb not in cov:
                continue
            na, ta = osm.ways[wa]
            nb, tb = osm.ways[wb]
            common = [n for n in (na[0], na[-1]) if n in (nb[0], nb[-1])] or [n for n in na if n in set(nb)]
            if not common or common[0] not in osm.nodes:
                continue
            node = common[0]
            s_node, _, dist = r.project(*osm.nodes[node])
            for side in ("R", "L"):
                ka, kb = self.per_side(osm_counts(ta), cov[wa][2], side), self.per_side(osm_counts(tb), cov[wb][2], side)
                changes.append(dict(node=node, side=side, s=s_node, ka=ka, kb=kb, wa=wa, wb=wb, ta=ta, tb=tb,
                                    da=cov[wa][2], db=cov[wb][2], x=osm.nodes[node][0], y=osm.nodes[node][1], dist=dist))
        # (a) count disagreement runs
        ss = np.arange(0.0, r.length, DSL)
        for side in ("R", "L"):
            exp = np.full(len(ss), -1)
            for w, (lo, hi, d) in cov.items():
                k = self.per_side(osm_counts(osm.ways[w][1]), d, side)
                if k is None:
                    continue
                m = (ss > lo + 1.0) & (ss < hi - 1.0)
                exp[m] = k
            if (exp < 0).all():
                continue
            n = v.counts(side, ss)
            care = exp >= 0
            for e in events:
                if e["side"] == side:
                    care &= ~((ss >= e["sa"] - 1.0) & (ss <= e["sb"] + 1.0))
            bad = care & (n != exp)
            # runs
            idx = np.nonzero(bad)[0]
            if not len(idx):
                continue
            splits = np.nonzero(np.diff(idx) > 1)[0]
            for run in np.split(idx, splits + 1):
                lo_s, hi_s = ss[run[0]], ss[run[-1]] + DSL
                length = hi_s - lo_s
                if length <= LOC_TOL:
                    continue
                chs = [c for c in changes if c["side"] == side and c["ka"] is not None and c["kb"] is not None
                       and c["ka"] != c["kb"] and lo_s - 2.0 <= c["s"] <= hi_s + 2.0]
                adj = [e for e in events if e["side"] == side and (abs(e["sa"] - hi_s) < 2.5 or abs(e["sb"] - lo_s) < 2.5)]
                near = [e for e in events if e["side"] == side and min(abs(e["sa"] - lo_s), abs(e["sb"] - hi_s)) < MATCH_RADIUS]
                if chs and adj:
                    sub = "shifted_transition"
                elif chs and not near:
                    sub = "missing_transition"
                elif chs:
                    sub = "shifted_transition"
                else:
                    sub = "count_mismatch"
                sm = 0.5 * (lo_s + hi_s)
                c0 = chs[0] if chs else None
                x, y = (c0["x"], c0["y"]) if c0 else r.point(sm, 0.0)
                self.add("a_location", sub, "major" if length > LOC_MAJOR else "minor", [r.id], x, y, s=sm,
                         s_from=float(lo_s), s_to=float(hi_s), extent=float(length), side=side,
                         road_lanes=int(np.bincount(n[run]).argmax()), osm_lanes=int(np.bincount(exp[run]).argmax()),
                         node=c0["node"] if c0 else None, ways=[c0["wa"], c0["wb"]] if c0 else None,
                         road_len=round(r.length, 1), mph=round(v.mph, 1), highway=v.hw)
        # match explicit changes to tapers; (b) side at tag changes
        matched = set()
        for ch in changes:
            ka, kb = ch["ka"], ch["kb"]
            if ka is None or kb is None or ka == kb or ch["dist"] > 15.0:
                continue
            if ch["s"] < 1.0 or ch["s"] > r.length - 1.0:
                self.stats["osm_change_at_road_end"] += 1
                continue
            self.stats["osm_changes_inside"] += 1
            kind = "add" if (kb > ka) == (v.tsign[ch["side"]] > 0) else "drop"
            cands = [e for e in events if e["side"] == ch["side"] and e["kind"] == kind and id(e) not in matched]

            def err(e):
                return 0.0 if e["sa"] - 1e-6 <= ch["s"] <= e["sb"] + 1e-6 else min(abs(ch["s"] - e["sa"]), abs(ch["s"] - e["sb"]))
            cands.sort(key=err)
            if not cands or err(cands[0]) > MATCH_RADIUS:
                continue
            group = [e for e in cands if abs(e["s_mid"] - cands[0]["s_mid"]) < 1.0][:abs(kb - ka)]
            for e in group:
                matched.add(id(e))
            exp = self.expected_side(r, v, ch, kind)
            if exp is None:
                continue
            want_in, want_out, evidence = exp
            ndrv, kchg = group[0]["ndrv"], len(group)
            at_inner = sum(1 for e in group if e["drank"] < kchg)
            at_outer = sum(1 for e in group if e["drank"] >= ndrv - kchg and e["drank"] >= kchg)
            need_in, need_out = min(want_in, kchg), min(want_out, max(0, kchg - min(want_in, kchg)))
            if at_inner < need_in or at_outer < need_out:
                self.add("b_side", "wrong_side", "major", [r.id], ch["x"], ch["y"], s=ch["s"],
                         lanes=[e["lane"] for e in group], kind=kind, expected_inner=need_in, expected_outer=need_out,
                         got_inner=at_inner, got_outer=at_outer, evidence=evidence, ways=[ch["wa"], ch["wb"]],
                         highway=v.hw, mph=round(v.mph, 1),
                         through_lane_move=max([e.get("jog", 0.0) for e in group]))
        # tapers without an explicit OSM change
        covered = list(cov.values())
        for e in events:
            if id(e) in matched:
                continue
            sm = 0.5 * (e["sa"] + e["sb"])
            if not any(lo - 2 <= sm <= hi + 2 for lo, hi, d in covered):
                self.stats["taper_outside_osm_coverage"] += 1
                continue
            if self.branch_role(v, e)[0] == "branch":
                self.stats["gore_branch_lane_taper"] += 1  # lane-level gore: the branch lanes ride on the trunk
                continue
            near = [c for c in changes if c["side"] == e["side"] and abs(c["s"] - sm) < MATCH_RADIUS]
            tagdiff = any(c["ka"] is not None and c["kb"] is not None and c["ka"] != c["kb"] for c in near)
            default = any(c["ka"] is None or c["kb"] is None for c in near)
            sub = "extra_transition" if tagdiff else "default_count_change" if default else "unexplained_transition"
            x, y = r.point(sm, 0.0)
            self.add("a_location", sub, "minor" if sub == "default_count_change" else "major", [r.id], x, y, s=sm,
                     lane=e["lane"], kind=e["kind"], side=e["side"], highway=v.hw, mph=round(v.mph, 1),
                     ways=sorted({c["wa"] for c in near} | {c["wb"] for c in near}),
                     osm_lanes=[(c["wa"], c["ka"], c["wb"], c["kb"]) for c in near])

    @staticmethod
    def per_side(c, d, side):
        if c is None or d is None:
            return None
        return c[0] if (side == "R") == (d > 0) else c[1]

    def expected_side(self, r, v, ch, kind):
        """(#changed lanes expected at the inner end, # at the outer end, evidence) or None.
        inner = the driver's left in RHT.  A pocket lane is one whose turn values are all left-ish
        (right-ish); shared lanes such as through;right are not pockets."""
        wide_tags, wide_dir = (ch["ta"], ch["da"]) if ch["ka"] > ch["kb"] else (ch["tb"], ch["db"])
        c = osm_counts(wide_tags)
        oneway = c is not None and 0 in c
        direction = "forward" if (ch["side"] == "R") == (wide_dir > 0) else "backward"
        tl = turn_list(wide_tags, direction, oneway)
        if tl and len(tl) >= 2:
            nl = next((i for i, t in enumerate(tl) if not (t <= LEFTISH)), len(tl))
            nr = next((i for i, t in enumerate(reversed(tl)) if not (t <= RIGHTISH)), len(tl))
            if (nl or nr) and nl + nr < len(tl):
                return (nl, nr, "turn:lanes=" + "|".join(";".join(sorted(x)) for x in tl))
        links = [(w, k) for w, k in self.node_ways[ch["node"]] if self.osm.ways[w][1].get("highway") in LINKS
                 and w not in (ch["wa"], ch["wb"])]
        if len(links) != 1 or self.osm.ways[ch["wa"]][1].get("highway") in LINKS:
            return None
        w, k = links[0]
        nds = self.osm.ways[w][0]
        other = nds[k + 1] if k + 1 < len(nds) else nds[k - 1]
        if other not in self.osm.nodes:
            return None
        _, t, _ = r.project(*self.osm.nodes[other])
        left_of_travel = (t > 0) == (v.tsign[ch["side"]] > 0)
        return (1, 0, "link way %s at node %s" % (w, ch["node"])) if left_of_travel else             (0, 1, "link way %s at node %s" % (w, ch["node"]))

    # -------------------------------------------------------- gore side (merge / diverge in uncovered stretches)
    def branch_role(self, v, e):
        """where the transition lane connects: ('branch'|'trunk'|None, trunk road, {lane: roads}, section index).
        Drops are looked up at the upstream end, adds at the downstream end (traffic direction); trunk = the
        road connected to most lanes there."""
        r = v.r
        ts = v.tsign[e["side"]]
        if e["kind"] == "drop":
            k_end, end = (0, "start") if ts > 0 else (len(r.sections) - 1, "end")
        else:
            k_end, end = (len(r.sections) - 1, "end") if ts > 0 else (0, "start")
        feed = self.contacts.get((r.id, end))
        if not feed:
            return None, None, None, k_end
        tally = collections.Counter(o for lid, os_ in feed.items() for o in os_)
        if len(tally) < 2:
            return None, None, feed, k_end
        trunk = tally.most_common(1)[0][0]
        ids = {c: lid for lid, c in v.chain[k_end].items()}
        if e["chain"] not in ids or ids[e["chain"]] not in feed:
            return None, trunk, feed, k_end
        mine = feed[ids[e["chain"]]]
        return ("trunk" if trunk in mine else "branch"), trunk, feed, k_end

    def check_gore_side(self, v, events):
        r = v.r
        for e in events:
            role, trunk, feed, k_end = self.branch_role(v, e)
            if role != "trunk" or len(feed[{c: l for l, c in v.chain[k_end].items()}[e["chain"]]]) != 1:
                continue
            # a persistent (non-event) chain on the same side served by a branch
            ev_chains = {x["chain"] for x in events}
            sec_end = r.sections[k_end]
            branch_lanes = [lid for lid in feed if lid in v.chain[k_end] and v.chain[k_end][lid] not in ev_chains
                            and sec_end.lane(lid) is not None and sec_end.lane(lid).type in DRIVE
                            and (lid < 0) == (e["side"] == "R") and trunk not in feed[lid]]
            if branch_lanes:
                sm = 0.5 * (e["sa"] + e["sb"])
                x, y = r.point(sm, 0.0)
                self.add("b_side", "wrong_side_gore", "major", [r.id, trunk], x, y, s=sm, lane=e["lane"], kind=e["kind"],
                         lane_serves=trunk, branch_lane=branch_lanes[0], branch_road=sorted(feed[branch_lanes[0]])[0],
                         highway=v.hw, mph=round(v.mph, 1))

    # -------------------------------------------------------- contacts
    def check_contacts(self):
        od = self.od
        for c in od.connecting_roads():
            for end, link in (("start", c.pred), ("end", c.succ)):
                if link is None or link.type != "road" or link.id not in od.roads:
                    continue
                o = od.roads[link.id]
                s_c = 1e-4 if end == "start" else c.length - 1e-4
                s_o = 1e-4 if link.contact == "start" else o.length - 1e-4
                sec_c = c.sections[0] if end == "start" else c.sections[-1]
                sec_o = o.sections[0] if link.contact == "start" else o.sections[-1]
                bc, bo = c.borders_at(s_c, sec_c), o.borders_at(s_o, sec_o)
                for l in sec_c.lanes():
                    tgt = l.pred if end == "start" else l.succ
                    if tgt is None or tgt not in bo:
                        continue
                    wc, wo = abs(bc[l.id][1] - bc[l.id][0]), abs(bo[tgt][1] - bo[tgt][0])
                    if abs(wc - wo) <= STEP_TOL:
                        continue
                    pc = [np.array(c.point(np.array([s_c]), np.array([t]))).ravel() for t in bc[l.id]]
                    po = [np.array(o.point(np.array([s_o]), np.array([t]))).ravel() for t in bo[tgt]]
                    gap = float(min(max(np.hypot(*(pc[0] - po[0])), np.hypot(*(pc[1] - po[1]))),
                                    max(np.hypot(*(pc[0] - po[1])), np.hypot(*(pc[1] - po[0])))))
                    if gap > CONTACT_TOL:
                        x, y = pc[0]
                        self.add("f_contact", "contact_width_step", "critical" if gap > STEP_CRIT else "major", [c.id, o.id],
                                 x, y, contact=end, lane=l.id, to_lane=tgt, lane_type=l.type, gap=gap, width_conn=wc,
                                 width_road=wo, highway=self.view(o).hw, mph=round(self.view(o).mph, 1))

    def run(self):
        for r in self.od.roads.values():
            v = self.view(r)
            events = self.check_road(v)
            if r.junction == "-1":
                self.check_gore_side(v, events)
                if r.osm_ways:
                    self.check_osm(v, events)
        self.check_contacts()
        return self.findings


# ------------------------------------------------------------------ reporting
SEV_RANK = {"critical": 0, "major": 1, "minor": 2}


def magnitude(f):
    """defect size relative to its own threshold (comparable across sub-patterns)"""
    if f["sub"] == "short_taper":
        return f["ratio_req"] / max(f["ratio"], 0.1)
    if f["sub"] == "through_lane_shift_rate":
        return f["shift_ratio_req"] / max(f["shift_ratio"], 0.1)
    for k, tol in (("jump", STEP_TOL), ("extent", LOC_TOL), ("gap", CONTACT_TOL), ("move", JOG_TOL)):
        if f.get(k) is not None:
            return float(f[k]) / tol
    return 1.0


def score(f):
    hw = f.get("highway") or ""
    fw = hw.startswith("motorway") or hw.startswith("trunk")
    return (SEV_RANK[f["severity"]], not fw, -magnitude(f))


def summarize(findings, stats):
    out = ["lane_transitions findings: %d   (critical %d, major %d, minor %d)" % (
        len(findings), *(sum(1 for f in findings if f["severity"] == s) for s in ("critical", "major", "minor")))]
    by = collections.Counter((f["check"], f["sub"], f["severity"]) for f in findings)
    for (c, s, v), n in sorted(by.items()):
        out.append("  %-14s %-26s %-8s %5d" % (c, s, v, n))
    out.append("by sub-pattern x highway class:")
    hw = collections.Counter((f["sub"], f.get("highway")) for f in findings)
    for (s, h), n in sorted(hw.items(), key=lambda kv: (str(kv[0][0]), str(kv[0][1]))):
        out.append("  %-26s %-16s %5d" % (s, h, n))
    out.append("stats: " + json.dumps(dict(stats)))
    sites = []
    for f in sorted(findings, key=score):
        if not any(math.hypot(f["x"] - x, f["y"] - y) < 25.0 for x, y in sites):
            sites.append((f["x"], f["y"]))
    out.append("distinct sites (findings merged within 25 m): %d" % len(sites))
    tp = [f for f in findings if f["sub"] == "short_taper"]
    if tp:
        r, L = np.array([f["ratio"] for f in tp]), np.array([f["taper_len"] for f in tp])
        out.append("short_taper ratio p10/50/90 = %.1f / %.1f / %.1f   length p10/50/90 = %.1f / %.1f / %.1f m"
                   % (*np.percentile(r, [10, 50, 90]), *np.percentile(L, [10, 50, 90])))
        for kind in ("add", "drop"):
            q = [f for f in tp if f["kind"] == kind]
            if q:
                out.append("   %s: n=%d  median len %.1f m, median required %.1f m" % (
                    kind, len(q), np.median([f["taper_len"] for f in q]), np.median([f["required_len"] for f in q])))
    for sub, key in (("shifted_transition", "extent"), ("missing_transition", "extent"), ("count_mismatch", "extent"),
                     ("through_lane_jog", "move"), ("border_step_at_section", "jump")):
        q = [f[key] for f in findings if f["sub"] == sub]
        if q:
            out.append("%s %s p50/90/max = %.2f / %.2f / %.2f" % (sub, key, *np.percentile(q, [50, 90]), max(q)))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xodr")
    ap.add_argument("osm")
    ap.add_argument("--out", required=True)
    ap.add_argument("--basemap")
    ap.add_argument("--crops", type=int, default=0, help="render the N worst findings plus 2 typical per sub-pattern")
    ap.add_argument("--half", type=float, default=40.0, help="crop half size (m)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    od = G.OpenDrive(a.xodr)
    osm = G.Osm(a.osm, od.lat0, od.lon0)
    det = Detector(od, osm)
    findings = det.run()
    findings.sort(key=score)
    for i, f in enumerate(findings):
        f["rank"] = i
    with open(os.path.join(a.out, "findings.json"), "w") as fh:
        json.dump(findings, fh, indent=1)
    txt = summarize(findings, det.stats)
    with open(os.path.join(a.out, "summary.txt"), "w") as fh:
        fh.write(txt + "\n")
    print(txt)
    if a.crops:
        cd = os.path.join(a.out, "crops")
        os.makedirs(cd, exist_ok=True)
        picks, seen_loc = [], []
        for f in findings:  # worst, de-duplicated by location
            if len(picks) >= a.crops:
                break
            if any(math.hypot(f["x"] - p["x"], f["y"] - p["y"]) < 30 for p in seen_loc):
                continue
            picks.append(f), seen_loc.append(f)
        per = collections.Counter()
        for f in findings:
            if f in picks:
                continue
            if per[f["sub"]] < 2 and f["rank"] > len(findings) // 3:
                per[f["sub"]] += 1
                picks.append(f)
        for f in picks:
            p = os.path.join(cd, "%03d_%s_r%s.png" % (f["rank"], f["sub"], "-".join(f["roads"])))
            G.crop(od, osm, (f["x"], f["y"]), a.half, p, basemap=a.basemap, marks=[(f["x"], f["y"], f["sub"])],
                   title="#%d %s %s road %s %s" % (f["rank"], f["severity"], f["sub"], ",".join(f["roads"]),
                                                   f.get("highway") or ""))
            highlight(od, f, p, a.half)


def highlight(od, f, path, half, px=900):
    """overlay the subject road's lane borders (yellow; the transition lane orange) and the taper / run
    interval (green ticks) on a crop rendered by odrgeom.crop (same framing)."""
    from PIL import Image, ImageDraw
    r = od.roads.get(f["roads"][0])
    if r is None:
        return
    x0, y1, sc = f["x"] - half, f["y"] + half, px / (2 * half)
    img = Image.open(path).convert("RGB")
    dr = ImageDraw.Draw(img)

    def P(xs, ys):
        return [((x - x0) * sc, (y1 - y) * sc) for x, y in zip(xs, ys)]
    for sec in r.sections:
        s = np.linspace(sec.s, sec.e - 1e-6, max(2, int((sec.e - sec.s) / 0.5) + 1))
        for l in sec.lanes():
            to = np.array([r.borders_at(float(sv), sec)[l.id][1] for sv in s])
            xb, yb = r.point(s, to)
            hot = l.id == f.get("lane") or l.id in (f.get("lanes") or [])
            dr.line(P(xb, yb), fill=(255, 140, 0) if hot else (255, 230, 0), width=3 if hot else 2)
        to = np.array([r.borders_at(float(sv), sec)[0][0] for sv in s])
        xb, yb = r.point(s, to)
        dr.line(P(xb, yb), fill=(255, 255, 255), width=1)
    for key in ("s_from", "s_to"):
        if f.get(key) is not None:
            b = r.borders_at(min(max(f[key], 0), r.length - 1e-6))
            ts = [v for pair in b.values() for v in pair]
            xa, ya = r.point(np.array([f[key]] * 2), np.array([min(ts) - 1.5, max(ts) + 1.5]))
            dr.line(P(xa, ya), fill=(0, 220, 0), width=3)
    dr.text((4, 22), "yellow = lane borders of road %s (orange = flagged lane), green = interval, white = laneOffset"
            % r.id, fill=(255, 255, 0))
    img.save(path)


if __name__ == "__main__":
    main()
