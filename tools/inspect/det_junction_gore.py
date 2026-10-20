"""det_junction_gore - junction and lane-level gore quality detector for OdrForge output.

Checks every junction (plain intersections and the lane-level gores built by src/gore.cpp) of an
OpenDRIVE file against itself and against the OSM source it was built from. Every check is stated
so that a correct converter produces zero findings on it.

Connecting roads (every lane of a connecting road, measured on the lane's own centre line - laneOffset
and lane widths applied: a connecting road may carry several lanes and its reference line need not be a
lane centre; one finding per connecting road and kind, for its worst lane; connecting roads without lanes,
e.g. 1.8.0 junction boundary helper roads, carry no path; lane directions follow road@rule, RHT or LHT):
  conn_discontinuity  connecting lane start/end does not meet the linked lane centre of the plain road
                      (gap > 0.10 m or direction step > 2 deg, directions compared as lines).
                      Spec: lanes link C0/G1.
  conn_wiggle         the path turns more than it has to: total |turning| - |net heading change| > 20 deg
                      (an S-bend or loop; a G1 clothoid path between two poses never needs it unless the
                      ends are misaligned).
  conn_jog            straight movement (|net heading change| < 35 deg) whose exit lane centre is offset
                      laterally from the entry lane's line: |offset| > 0.5 m and offset/chord steeper
                      than a lane-change taper (1:15 on motorway/trunk/link roads, 1:6 elsewhere).
  conn_curvature      straight movement whose peak curvature exceeds the curvature of the adjacent lanes
                      by more than 1 / minRadiusFor(class) (the converter's own class minimum radius,
                      src/network.cpp:40) - a notch.
  conn_osm_dev        straight or *_link/motorway movement whose path leaves the OSM ways of the two roads
                      it joins by more than 2.0 m beyond the linear blend of its end distances.
  conn_crossing       two connecting lanes from the same incoming road (or into the same outgoing road),
                      with distinct start and distinct end points, cross each other (lane order inverted).
Lane links:
  lane_no_successor   a driving lane entering a junction that some movement leaves from (the road has
                      connections) but this lane has none.
  lane_no_predecessor a driving lane leaving a junction that is fed by no connecting lane while the road
                      itself is fed.
  turn_lanes          at the end of the OSM way, an incoming lane tagged (turn:lanes) right-only connected
                      to the leftmost of several targets (not itself a right turn), left-only to the
                      rightmost, or through-only to a turn of more than 55 deg.
Gores / freeway splits and merges (junctions where a motorway/trunk/*_link road splits or merges):
  gore_separation     where the branch road begins (diverge) / ends (merge), its driving carriageway
                      overlaps the main road's by > 0.5 m (junction too early) or the gap between the two
                      exceeds 3.0 m (the connecting lanes must jog across it: notch / junction too late).
  gore_osm_sep        at that point the OSM ways themselves say otherwise: the OSM branch way is more than
                      2.5 m closer to or further from the OSM main way than the two carriageway half widths
                      (more than OSM tracing error, ~1-2 m) - the separation point is misplaced versus OSM.
  gore_kink           a lane-level gore (connectors <= 8 m carrying lanes 1:1) whose connecting lanes turn (net or peak heading
                      deviation) more than a 100 m-radius ramp would over their length + 1 deg (AASHTO
                      minimum radius at 35 mph): the ramp's divergence angle / lateral offset is folded
                      into the 4 m junction - a notch at the gore nose. One finding per junction.
  ramp_follow         the first 60 m of a *_link road after/before such a junction deviate from its OSM
                      way by more than 1.5 m (carriageway centre vs OSM centre line).
  osm_uncovered       a sample of a motorway/trunk/*_link OSM way within 80 m of a junction lies more than
                      1.0 m outside the driving lanes of every road built from that way (or its junction's
                      connecting lanes, or the trunk that carries its lanes): the network does not follow it.
Junction size:
  junction_size       (plain junctions) a road end lies further from the OSM junction node(s) than the
                      overlap distance the roads' widths and meeting angle require (+ corner + 15 m slack).

  junction_span       (street junctions) a connecting lane longer than 45 m - the widest LA carriageway
                      (8 x 3.5 m) + 2 corner radii + 2 stop-line setbacks: several OSM intersections were
                      clustered into one junction whose lanes then sweep across them, ignoring the streets.

CLI:  python det_junction_gore.py XODR OSM --out DIR [--basemap DIR] [--crops N]
Writes DIR/findings.json (list of findings) and DIR/summary.txt; with --basemap, evidence crops for the
worst and some typical findings into DIR/crops/.
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

D2R = math.pi / 180
FREEWAY = {"motorway", "motorway_link", "trunk", "trunk_link"}
RANK = {"motorway": 10, "trunk": 9, "primary": 8, "secondary": 7, "tertiary": 6, "motorway_link": 9, "trunk_link": 8,
        "primary_link": 7, "secondary_link": 6, "tertiary_link": 5, "unclassified": 4, "residential": 4,
        "living_street": 2, "service": 1, "busway": 3, "road": 3}
NONROAD = {"footway", "path", "cycleway", "steps", "pedestrian", "corridor", "elevator", "bridleway", "track",
           "construction", "proposed", "platform"}
EDGE = 40.0  # m: OSM samples this close to the converted area's edge are skipped (clipped ways)
DRIVING = {"driving", "bus", "entry", "exit", "onRamp", "offRamp", "connectingRamp"}
WALKING = {"sidewalk", "walking"}  # continuity only (a walking connecting lane has no driven shape)

# thresholds (see module doc)
T = dict(gap=0.10, hdg=2.0 * D2R, wiggle=20 * D2R, straight=35 * D2R, jog_min=0.5, jog_fw=1 / 15.0, jog_city=1 / 6.0,
         osm_dev=2.0, sep_overlap=0.5, sep_gap=3.0, sep_gap_gore=1.0, osm_sep=2.5, ramp_dev=1.5, ramp_len=60.0, uncovered=1.0,
         uncovered_zone=80.0, size_slack=15.0, span=45.0, gore_radius=100.0)


def min_radius(hw):  # src/network.cpp:40 minRadiusFor
    if hw in ("motorway", "trunk"):
        return 60.0
    if hw in ("motorway_link", "trunk_link"):
        return 25.0
    if hw in ("primary", "secondary"):
        return 15.0
    return 8.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ----------------------------------------------------------------------------- geometry helpers
def seg_dist(P, A):
    """distance from points P (n,2) to polyline A (m,2)"""
    if len(A) < 2:
        return np.hypot(P[:, 0] - A[0, 0], P[:, 1] - A[0, 1]) if len(A) else np.full(len(P), 1e9)
    a, b = A[:-1], A[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    d = P[:, None, :] - a[None, :, :]
    u = np.clip((d * ab[None]).sum(2) / L2[None], 0, 1)
    q = a[None] + u[..., None] * ab[None]
    return np.hypot(P[:, None, 0] - q[..., 0], P[:, None, 1] - q[..., 1]).min(1)


def dist_ways(P, polys):
    """distance from points P to the nearest of several polylines (each measured on its own: stacking
    them would invent segments between the end of one way and the start of the next)"""
    return np.min([seg_dist(P, A) for A in polys], axis=0)


def seg_intersect(A, B, skip=0.5):
    """first intersection point of polylines A, B (n,2), ignoring points within `skip` of either start/end"""
    for i in range(len(A) - 1):
        p, r = A[i], A[i + 1] - A[i]
        q, s = B[:-1], B[1:] - B[:-1]
        den = r[0] * s[:, 1] - r[1] * s[:, 0]
        ok = np.abs(den) > 1e-12
        qp = q - p
        t = np.where(ok, (qp[:, 0] * s[:, 1] - qp[:, 1] * s[:, 0]) / np.where(ok, den, 1), -1)
        u = np.where(ok, (qp[:, 0] * r[1] - qp[:, 1] * r[0]) / np.where(ok, den, 1), -1)
        hit = ok & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
        if hit.any():
            j = int(np.argmax(hit))
            x = p + t[j] * r
            ends = [A[0], A[-1], B[0], B[-1]]
            if min(np.hypot(*(x - e)) for e in ends) > skip:
                return x
    return None


class Net:
    """OpenDRIVE + OSM with the lookups the checks need."""

    def __init__(self, xodr, osm_path):
        self.od = G.OpenDrive(xodr)
        self.osm = G.Osm(osm_path, self.od.lat0, self.od.lon0)
        self.poly = {}
        h = self.od.header
        # converted area (header extents): OSM beyond it, or within EDGE of it, was clipped away
        self.bbox = tuple(float(h.get(k)) for k in ("west", "south", "east", "north"))
        self.way_roads = collections.defaultdict(set)
        for r in self.od.plain_roads():
            for w in r.osm_ways:
                self.way_roads[w].add(r.id)
        # junction -> connecting roads, plain road ends
        self.jconn = collections.defaultdict(list)
        for r in self.od.connecting_roads():
            self.jconn[r.junction].append(r)
        self.jends = collections.defaultdict(list)  # junction -> [(road, atEnd)]
        for r in self.od.plain_roads():
            if r.pred is not None and r.pred.type == "junction":
                self.jends[r.pred.id].append((r, False))
            if r.succ is not None and r.succ.type == "junction":
                self.jends[r.succ.id].append((r, True))

    def wpoly(self, w):
        if w not in self.poly:
            self.poly[w] = self.osm.polyline(w) if w in self.osm.ways else np.zeros((0, 2))
        return self.poly[w]

    def hw(self, r):
        """highest class among the road's OSM ways (connecting road: of its two plain roads)"""
        if r.junction != "-1":
            ids = [lk.id for lk in (r.pred, r.succ) if lk is not None and lk.type == "road"]
            hs = [self.hw(self.od.roads[i]) for i in ids if i in self.od.roads]
            return max(hs, key=lambda h: RANK.get(h, 0)) if hs else "?"
        hs = [self.osm.ways[w][1].get("highway", "?") for w in r.osm_ways if w in self.osm.ways]
        return max(hs, key=lambda h: RANK.get(h, 0)) if hs else "?"

    def adjacent_ways(self, ways):
        """the ways plus every highway way sharing a node with them (a gore trunk runs on along the
        main way's line, but its osmWays name only its own chain)"""
        if not hasattr(self, "_node_ways"):
            self._node_ways = collections.defaultdict(set)
            for w, (nds, tg) in self.osm.ways.items():
                if "highway" in tg:
                    for x in nds:
                        self._node_ways[x].add(w)
        out = set(ways)
        for w in ways:
            if w in self.osm.ways:
                for x in self.osm.ways[w][0]:
                    out |= self._node_ways[x]
        return out

    def end_way(self, r, at_end):
        if not r.osm_ways:
            return None
        return r.osm_ways[-1] if at_end else r.osm_ways[0]


def is_lht(r):
    return (r.el.get("rule") or "RHT").upper() == "LHT"


def lane_pose(r, s, lid):
    """lane centre (x, y, travel heading) of a plain road lane at s (road@rule: RHT lanes with id < 0
    travel +s, LHT lanes with id > 0)"""
    b = r.borders_at(s)
    t = 0.5 * (b[lid][0] + b[lid][1])
    x, y, h = (float(v) for v in r.ref(s))
    px, py = x - t * math.sin(h), y + t * math.cos(h)
    return px, py, h if (lid < 0) != is_lht(r) else h + math.pi


def axis_diff(a, b):
    """angle between two directions taken as lines (0..pi/2): a lane path is compared with the lane it
    links to without assuming which way either is driven"""
    return abs((a - b + math.pi / 2) % math.pi - math.pi / 2)


def lane_centre_t(r, sec, lid, s):
    """t of the centre of lane lid at the s values (laneOffset and every lane width between the
    reference line and the lane)"""
    t = r.lane_offset(s)
    ds = s - sec.s
    for l in (sec.left if lid > 0 else sec.right):
        w = l.width(ds) * (1 if lid > 0 else -1)
        if l.id == lid:
            return t + 0.5 * w
        t = t + w
    raise KeyError(lid)


def lane_path(c, lid, step=0.25):
    """centre line of lane lid of road c through its sections (following the lane's successors):
    (s, points (n,2), heading along +s, the lane id in the last section reached)"""
    S, X, Y = [], [], []
    for k, sec in enumerate(c.sections):
        if k:
            prev = c.sections[k - 1].lane(lid)
            if prev is None or prev.succ is None or sec.lane(prev.succ) is None:
                break
            lid = prev.succ
        if sec.lane(lid) is None:
            break
        if sec.e - sec.s <= 1e-6:
            continue
        ss = np.linspace(sec.s, sec.e, max(3, int((sec.e - sec.s) / step) + 1))
        x, y = c.point(ss, lane_centre_t(c, sec, lid, ss))
        if S:
            ss, x, y = ss[1:], x[1:], y[1:]
        S.append(ss), X.append(x), Y.append(y)
    if not S:
        return None
    S, X, Y = np.concatenate(S), np.concatenate(X), np.concatenate(Y)
    if len(S) < 3:
        return None
    h = np.unwrap(np.arctan2(np.gradient(Y, S, edge_order=2), np.gradient(X, S, edge_order=2)))
    return S, np.c_[X, Y], h, lid


def band_path(c, step=1.0):
    """the centre line of road c's driving lanes (one path per connecting road for the crossing test)"""
    lanes = [l.id for l in c.sections[0].lanes() if l.type in DRIVING]
    paths = [p for p in (lane_path(c, lid, step) for lid in (lanes[0], lanes[-1])) if p is not None] if lanes else []
    if not paths:
        return None
    if len(paths) == 2 and len(paths[0][1]) == len(paths[1][1]):
        return 0.5 * (paths[0][1] + paths[1][1])
    return paths[0][1]


def driving_ids(r, s, side):
    sec = r.section_at(s)
    lanes = sec.right if side < 0 else sec.left
    return [l.id for l in lanes if l.type in DRIVING]


def drive_extent(r, s):
    """(t_min, t_max) of the driving lanes at s"""
    b = r.borders_at(s)
    sec = r.section_at(s)
    v = [x for l in sec.lanes() if l.type in DRIVING for x in b[l.id]]
    return (min(v), max(v)) if v else (b[0][0], b[0][0])


def incoming_lanes(r, at_end):
    """driving lanes that run into this road end (RHT: the right side at the end; LHT: the left side)"""
    s = r.length if at_end else 0.0
    return driving_ids(r, min(max(s, 1e-3), r.length - 1e-3), -1 if at_end != is_lht(r) else 1)


def outgoing_lanes(r, at_end):
    s = r.length if at_end else 0.0
    return driving_ids(r, min(max(s, 1e-3), r.length - 1e-3), 1 if at_end != is_lht(r) else -1)


def lane_level_gore(conns):
    """a lane-level gore (src/gore.cpp, a kJunctionLength = 4 m junction): short connectors (<= 8 m)
    that carry the lanes of one carriageway into two, or of two into one - the main road continues
    the trunk's reference line (one of the connectors runs straight on), and each lane continues in
    lanes of its own (gore.cpp splits the trunk lanes into a main and a branch set). Short connectors
    that only turn, or that bring two roads into one lane (one lane into two roads), are a plain
    compact junction (a corner of parking aisles, a merge at a right angle), not a gore"""
    if not conns or max(c.length for c in conns) > 8.0:
        return False
    def net_turn(c):
        h = np.unwrap(conn_samples(c, 0.1)[2])
        return abs(float(h[-1] - h[0]))

    if not any(net_turn(c) < T["straight"] for c in conns):
        return False
    other = collections.defaultdict(set)  # lane end at the junction -> roads it is connected with
    for c in conns:
        if c.pred is None or c.succ is None:
            continue
        for l in c.sections[0].left + c.sections[0].right:
            if l.pred is not None:
                other[("in", c.pred.id, c.pred.contact, l.pred)].add(c.succ.id)
        for l in c.sections[-1].left + c.sections[-1].right:
            if l.succ is not None:
                other[("out", c.succ.id, c.succ.contact, l.succ)].add(c.pred.id)
    return all(len(v) == 1 for v in other.values())


def conn_samples(c, step=0.5):
    s = np.linspace(0, c.length, max(3, int(c.length / step) + 1))
    x, y, h = c.ref(s)
    return s, np.c_[x, y], h


# ----------------------------------------------------------------------------- checks
class Detector:
    def __init__(self, net):
        self.n = net
        self.od = net.od
        self.F = []

    def add(self, kind, roads, x, y, severity, metric, **kw):
        self.F.append(dict(id=f"{kind}:{len(self.F)}", kind=kind, roads=[str(r) for r in roads], x=round(float(x), 2),
                           y=round(float(y), 2), severity=severity, metric=round(float(metric), 3),
                           **{k: (round(v, 3) if isinstance(v, float) else v) for k, v in kw.items()}))

    # -- connecting roads
    def check_connecting(self):
        """every lane of every connecting road, measured on the lane's own centre line (laneOffset and
        widths applied): a connecting road may carry several lanes and its reference line need not be a
        lane centre. A connecting road with no lanes (a 1.8.0 junction boundary helper road) has no path to
        check. One finding per connecting road and kind: its worst lane."""
        od, n = self.od, self.n
        for c in od.connecting_roads():
            if c.pred is None or c.succ is None or c.pred.id not in od.roads or c.succ.id not in od.roads:
                continue
            lanes = [l for l in c.sections[0].lanes() if l.id and l.type in DRIVING | WALKING]
            if not lanes:
                continue
            P, Q = od.roads[c.pred.id], od.roads[c.succ.id]
            hw = n.hw(c)
            fw = hw in FREEWAY
            sP = P.length if c.pred.contact == "end" else 0.0
            sQ = Q.length if c.succ.contact == "end" else 0.0
            worst = {}  # kind -> (score, args, kwargs)

            def keep(kind, score, *args, **kw):
                if kind not in worst or score > worst[kind][0]:
                    worst[kind] = (score, args, kw)

            for lane in lanes:
                path = lane_path(c, lane.id)
                if lane.pred is None or path is None:
                    continue
                s, pts, h, end_id = path
                end_lane = c.sections[-1].lane(end_id)
                if end_lane is None or end_lane.succ is None:
                    continue
                try:
                    ax, ay, ah = lane_pose(P, sP, lane.pred)
                    bx, by, bh = lane_pose(Q, sQ, end_lane.succ)
                except KeyError:
                    keep("conn_discontinuity", 99.0, [c.id, P.id, Q.id], *pts[0], "critical", 99.0, junction=c.junction,
                         lane=lane.id, note="linked lane id does not exist", hw=hw)
                    continue
                # (1) continuity of the lane centre with the linked lane centres: position, and direction taken
                # as a line (the path heading comes from the sampled centre, whose error grows with curvature)
                g0 = math.hypot(pts[0, 0] - ax, pts[0, 1] - ay)
                g1 = math.hypot(pts[-1, 0] - bx, pts[-1, 1] - by)
                dh0, dh1 = axis_diff(h[0], ah), axis_diff(h[-1], bh)
                k0 = abs(float(h[2] - h[0])) / max(float(s[2] - s[0]), 1e-6)
                k1 = abs(float(h[-1] - h[-3])) / max(float(s[-1] - s[-3]), 1e-6)
                if max(g0, g1) > T["gap"] or dh0 > T["hdg"] + 0.5 * k0 or dh1 > T["hdg"] + 0.5 * k1:
                    at0 = g0 + dh0 > g1 + dh1
                    keep("conn_discontinuity", max(g0, g1) + max(dh0, dh1), [c.id, P.id, Q.id],
                         pts[0, 0] if at0 else pts[-1, 0], pts[0, 1] if at0 else pts[-1, 1], "critical", max(g0, g1),
                         junction=c.junction, lane=lane.id, gap_start=g0, gap_end=g1, hdg_start_deg=dh0 / D2R,
                         hdg_end_deg=dh1 / D2R, hw=hw)
                if lane.type not in DRIVING:
                    continue
                # (2) shape of the driven path, from its own headings along s (the travel direction does not
                # change these magnitudes)
                a0, a1 = float(h[0]), float(h[-1])
                dh = np.diff(h)
                net = wrap(a1 - a0)
                total = float(np.abs(dh).sum())
                chord = float(math.hypot(*(pts[-1] - pts[0])))
                # least turning any path between the two poses needs: it must be parallel to the chord
                # somewhere (mean value theorem), so >= |chord - start| + |end - chord| (two arcs achieve it)
                phi = math.atan2(pts[-1, 1] - pts[0, 1], pts[-1, 0] - pts[0, 0])
                need = abs(wrap(phi - a0)) + abs(wrap(a1 - phi)) if chord > 0.05 else abs(net)
                wig = total - need
                if wig > T["wiggle"]:
                    i = int(np.argmax(np.abs(dh)))
                    keep("conn_wiggle", wig, [c.id, P.id, Q.id], pts[i, 0], pts[i, 1], "critical" if fw else "major",
                         wig / D2R, junction=c.junction, lane=lane.id, excess_turn_deg=wig / D2R, net_turn_deg=net / D2R,
                         length=c.length, chord=chord, hw=hw)
                straight = abs(net) < T["straight"]
                if straight and chord > 0.5:
                    # lateral offset of the exit lane centre from where a constant-curvature arc with the
                    # same net turn would put it (the arc's chord runs along the mean heading)
                    lat = chord * abs(math.sin(wrap(phi - (a0 + 0.5 * net))))
                    slope = lat / chord
                    lim = T["jog_fw"] if fw else T["jog_city"]
                    if lat > T["jog_min"] and slope > lim:
                        keep("conn_jog", lat, [c.id, P.id, Q.id], *pts[len(pts) // 2],
                             "critical" if fw and slope > 0.15 else "major" if fw or slope > 0.3 else "minor", lat,
                             junction=c.junction, lane=lane.id, lateral=lat, slope=slope, length=c.length,
                             net_turn_deg=net / D2R, hw=hw)
                    # curvature notch
                    kc = np.abs(dh) / np.maximum(np.diff(s), 1e-6)
                    kmax = float(np.percentile(kc, 98)) if len(kc) > 4 else float(kc.max())
                    lim_k = max(curv_at(P, sP), curv_at(Q, sQ)) + 1.0 / min_radius(hw)
                    if kmax > lim_k:
                        i = int(np.argmax(kc))
                        keep("conn_curvature", kmax / lim_k, [c.id, P.id, Q.id], pts[i, 0], pts[i, 1],
                             "critical" if fw else "major" if kmax > 2 * lim_k else "minor", kmax / lim_k,
                             junction=c.junction, lane=lane.id, kmax=kmax, radius=1 / max(kmax, 1e-9),
                             limit_radius=1 / lim_k, length=c.length, hw=hw)
                # (3) OSM deviation for straight / ramp movements
                if (straight or fw or hw.endswith("_link")) and c.length > 2.0:
                    ways = set(P.osm_ways[-1:] if c.pred.contact == "end" else P.osm_ways[:1])
                    ways |= set(Q.osm_ways[-1:] if c.succ.contact == "end" else Q.osm_ways[:1])
                    ways |= internal_ways(n, P, Q, c)
                    polys = [n.wpoly(w) for w in ways if len(n.wpoly(w)) >= 2]
                    if polys:
                        d = np.min([seg_dist(pts, A) for A in polys], axis=0)
                        u = s / s[-1]
                        blend = d[0] * (1 - u) + d[-1] * u
                        ex = d - blend
                        i = int(np.argmax(ex))
                        if ex[i] > T["osm_dev"]:
                            keep("conn_osm_dev", float(ex[i]), [c.id, P.id, Q.id], pts[i, 0], pts[i, 1],
                                 "critical" if fw and ex[i] > 4 else "major" if fw or ex[i] > 4 else "minor", float(ex[i]),
                                 junction=c.junction, lane=lane.id, excess=float(ex[i]), d_osm=float(d[i]),
                                 length=c.length, net_turn_deg=net / D2R, hw=hw)
            for kind, (_, args, kw) in worst.items():
                self.add(kind, *args, **kw)

    def check_crossings(self):
        n = self.n
        for jid, conns in n.jconn.items():
            P = {}
            for c in conns:
                if c.pred is None or c.succ is None:
                    continue
                pts = band_path(c, 1.0)  # the driving lanes' centre, not the reference line
                if pts is not None and len(pts) >= 2:
                    P[c.id] = (c, pts)
            items = list(P.values())
            seen = set()
            for i in range(len(items)):
                ci, pi = items[i]
                for j in range(i + 1, len(items)):
                    cj, pj = items[j]
                    same_in = ci.pred.id == cj.pred.id and ci.pred.contact == cj.pred.contact
                    same_out = ci.succ.id == cj.succ.id and ci.succ.contact == cj.succ.contact
                    if not (same_in or same_out):
                        continue
                    if np.hypot(*(pi[0] - pj[0])) < 0.5 or np.hypot(*(pi[-1] - pj[-1])) < 0.5:
                        continue  # share an end (one lane to two targets, or two lanes into one)
                    # bounding-box prefilter
                    if (pi[:, 0].max() < pj[:, 0].min() or pj[:, 0].max() < pi[:, 0].min() or
                            pi[:, 1].max() < pj[:, 1].min() or pj[:, 1].max() < pi[:, 1].min()):
                        continue
                    x = seg_intersect(pi, pj)
                    if x is None:
                        continue
                    key = (ci.pred.id, ci.succ.id, cj.succ.id) if same_in else (ci.succ.id, ci.pred.id, cj.pred.id)
                    hw = n.hw(ci)
                    self.add("conn_crossing", [ci.id, cj.id, ci.pred.id, ci.succ.id, cj.succ.id if same_in else cj.pred.id],
                             x[0], x[1], "critical" if hw in FREEWAY or same_in and same_out else "major",
                             1.0, junction=jid, shared="in+out" if same_in and same_out else "in" if same_in else "out",
                             hw=hw, dup=key in seen)
                    seen.add(key)

    # -- lane links
    def check_lane_links(self):
        n, od = self.n, self.od
        for jid, ends in n.jends.items():
            conns = n.jconn.get(jid, [])
            out_from = collections.defaultdict(set)  # (road, contact) -> lane ids that have a successor
            in_to = collections.defaultdict(set)
            for c in conns:  # every lane of every connecting road (a connecting road may carry several)
                if c.pred is not None:
                    for lane in c.sections[0].lanes():
                        if lane.id and lane.pred is not None:
                            out_from[(c.pred.id, c.pred.contact)].add(lane.pred)
                if c.succ is not None:
                    for lane in c.sections[-1].lanes():
                        if lane.id and lane.succ is not None:
                            in_to[(c.succ.id, c.succ.contact)].add(lane.succ)
            for r, at_end in ends:
                key = (r.id, "end" if at_end else "start")
                s = r.length if at_end else 0.0
                hw = n.hw(r)
                fw = hw in FREEWAY
                ins = incoming_lanes(r, at_end)
                if ins and key in out_from:
                    miss = [l for l in ins if l not in out_from[key]]
                    if miss:
                        x, y, _ = lane_pose(r, s, miss[0])
                        self.add("lane_no_successor", [r.id], x, y, "critical" if fw else "major", len(miss), junction=jid,
                                 lanes=miss, n_in=len(ins), hw=hw)
                outs = outgoing_lanes(r, at_end)
                if outs and key in in_to:
                    miss = [l for l in outs if l not in in_to[key]]
                    if miss:
                        x, y, _ = lane_pose(r, s, miss[0])
                        self.add("lane_no_predecessor", [r.id], x, y, "critical" if fw else "minor", len(miss), junction=jid,
                                 lanes=miss, n_out=len(outs), hw=hw)

    def check_turn_lanes(self):
        n, od = self.n, self.od
        for jid, ends in n.jends.items():
            conns = n.jconn.get(jid, [])
            for r, at_end in ends:
                ins = incoming_lanes(r, at_end)
                if len(ins) < 2:
                    continue
                w = n.end_way(r, at_end)
                if w is None or w not in n.osm.ways:
                    continue
                tags = n.osm.ways[w][1]
                tl = tags.get("turn:lanes") if tags.get("oneway") in ("yes", "1", "true") or tags.get("highway", "").startswith("motorway") else None
                if tl is None:
                    tl = tags.get("turn:lanes:forward" if at_end else "turn:lanes:backward")
                if not tl:
                    continue
                spec = [set(x.split(";")) for x in tl.split("|")]
                if len(spec) != len(ins):
                    continue
                # turn:lanes count from the driver's left: the inner lane (|id| small) under RHT, the outer under LHT
                byleft = sorted(ins, key=abs, reverse=is_lht(r))
                s = r.length if at_end else 0.0
                ax, ay, ah = lane_pose(r, s, byleft[0])
                mine = [c for c in conns if c.pred is not None and c.pred.id == r.id and c.pred.contact == ("end" if at_end else "start")
                        and any(l.type in DRIVING for l in c.sections[0].lanes())]
                if not mine:
                    continue
                # turn:lanes describe the lanes where the way ends; a junction the way runs on through
                # (a driveway joining mid-way) is not where they apply
                if any(w in od.roads[c.succ.id].osm_ways for c in mine if c.succ.id in od.roads):
                    continue
                # targets: net turn and lateral position of their far point
                tg = {}
                for c in mine:
                    _, pts, h = conn_samples(c, 1.0)
                    dl = next(l for l in c.sections[0].lanes() if l.type in DRIVING)
                    fwd = (dl.id < 0) != is_lht(c if c.el.get("rule") else r)  # the movement runs along +s
                    net = wrap(h[-1] - h[0] if fwd else h[0] - h[-1])
                    lat = -(pts[-1, 0] - ax) * math.sin(ah) + (pts[-1, 1] - ay) * math.cos(ah)
                    tg.setdefault((c.succ.id, c.succ.contact), []).append((net, lat))
                info = {k: (np.mean([a for a, _ in v]), np.mean([b for _, b in v])) for k, v in tg.items()}
                order = sorted(info, key=lambda k: -info[k][1])  # left -> right
                LEFT = {"left", "sharp_left", "slight_left"}
                RIGHT = {"right", "sharp_right", "slight_right"}
                for i, lid in enumerate(byleft):
                    sp = spec[i] - {"", "none"}
                    if not sp:
                        continue
                    outs = {(c.succ.id, c.succ.contact) for c in mine if any(l.pred == lid for l in c.sections[0].lanes())}
                    bad = []
                    for k in outs:
                        rank = order.index(k)
                        if sp <= RIGHT and len(order) > 1 and rank == 0 and info[k][0] > -T["straight"]:
                            bad.append(k)  # right-only lane to the leftmost target that is not a right turn
                        if sp <= LEFT and len(order) > 1 and rank == len(order) - 1 and info[k][0] < T["straight"]:
                            bad.append(k)
                        if sp == {"through"} and abs(info[k][0]) > T["straight"] + 20 * D2R:
                            bad.append(k)
                    missing = []  # a lane with no movement at all is lane_no_successor's business
                    if bad or missing:
                        x, y, _ = lane_pose(r, s, lid)
                        hw = n.hw(r)
                        self.add("turn_lanes", [r.id] + [k[0] for k in bad + missing], x, y,
                                 "critical" if hw in FREEWAY else "major", len(bad) + len(missing), junction=jid, lane=lid,
                                 tag="|".join(";".join(sorted(z)) for z in spec), lane_index=i,
                                 wrong=[k[0] for k in bad], missing=[k[0] for k in missing], hw=hw)

    # -- gores / freeway splits & merges
    def freeway_splits(self):
        """junctions with one road in and >=2 out (diverge) or >=2 in and one out (merge), freeway class"""
        n = self.n
        out = []
        for jid, ends in n.jends.items():
            if len(ends) != 3:
                continue
            hws = [n.hw(r) for r, _ in ends]
            if not any(h in FREEWAY for h in hws):
                continue
            # oneway roads only: count roads whose driving lanes enter the junction
            entering = [(r, e) for r, e in ends if incoming_lanes(r, e)]
            leaving = [(r, e) for r, e in ends if outgoing_lanes(r, e)]
            if any(incoming_lanes(r, e) and outgoing_lanes(r, e) for r, e in ends):
                continue
            if len(entering) == 1 and len(leaving) == 2:
                out.append((jid, "diverge", entering[0], leaving))
            elif len(entering) == 2 and len(leaving) == 1:
                out.append((jid, "merge", leaving[0], entering))
        return out

    def check_gores(self):
        n = self.n
        self.splits = self.freeway_splits()
        for jid, kind, (T_, te), pair in self.splits:
            # main = branch continuing straightest relative to the trunk (smaller heading change), ties: more lanes
            sT = T_.length if te else 0.0
            hT = float(T_.ref(sT)[2]) + (0 if te else math.pi)
            cand = []
            for r, e in pair:
                s = r.length if e else 0.0
                h = float(r.ref(s)[2]) + (math.pi if e else 0)  # travel heading away from junction... normalise
                d = abs(wrap((h if kind == "diverge" else h + math.pi) - (hT if kind == "diverge" else hT + math.pi)))
                nl = len(outgoing_lanes(r, e) if kind == "diverge" else incoming_lanes(r, e))
                cand.append((d, -nl, r, e))
            cand.sort(key=lambda z: (round(z[0] / (2 * D2R)), z[1]))
            (_, _, M, me), (_, _, B, be) = cand
            sM = M.length if me else 0.0
            sB = B.length if be else 0.0
            # (a) output separation at the branch end: gap between driving carriageways
            bx, by, _ = (float(v) for v in B.ref(sB))
            b0, b1 = drive_extent(B, sB)
            # branch driving carriageway corners at the junction end
            corners = [B.point(sB, t) for t in (b0, b1)]
            corners = [(float(x), float(y)) for x, y in corners]
            # both roads end at the junction at about the same station: measure laterally in the frame of
            # the main road's end pose (its tangent line), against its driving band there
            mx, my, mh = (float(v) for v in M.ref(sM))
            m0, m1 = drive_extent(M, sM)
            ts = [-(x - mx) * math.sin(mh) + (y - my) * math.cos(mh) for x, y in corners]
            along = (bx - mx) * math.cos(mh) + (by - my) * math.sin(mh)
            lo, hi = min(ts), max(ts)
            if lo >= m1:
                gap = lo - m1
            elif hi <= m0:
                gap = m0 - hi
            else:
                gap = -min(hi - m0, m1 - lo)  # overlap
            beyond = abs(along) > 10.0  # the two ends are not side by side: no lateral relation
            hw = n.hw(B)
            cx, cy = 0.5 * (corners[0][0] + corners[1][0]), 0.5 * (corners[0][1] + corners[1][1])
            # lane-level gore (src/gore.cpp, a kJunctionLength = 4 m junction): the branch must start where its
            # carriageway just clears the main one (gore.cpp asks for 0.3 m), so the carried lanes run
            # straight into it; a plain junction may legitimately span a wider gap
            lane_level = lane_level_gore(n.jconn.get(jid, []))
            gmax = T["sep_gap_gore"] if lane_level else T["sep_gap"]
            if not beyond and (gap < -T["sep_overlap"] or gap > gmax):
                self.add("gore_separation", [B.id, M.id, T_.id], cx, cy,
                         "critical" if gap < -1.5 or gap > (2.0 if lane_level else 6.0) else "major",
                         gap, junction=jid, move=kind, gap=gap, note="overlap" if gap < 0 else "gap", hw=hw)
            # (b) OSM-based separation at the same point
            wb = n.end_way(B, be)
            wm_list = M.osm_ways
            if wb and wm_list:
                pb = n.wpoly(wb)
                pm_all = [n.wpoly(w) for w in wm_list if len(n.wpoly(w)) >= 2]
                if len(pb) >= 2 and pm_all:
                    # point on the OSM branch way nearest the branch road end
                    q = nearest_on(pb, np.array([bx, by]))
                    d_osm = float(dist_ways(q[None], pm_all)[0])
                    need = 0.5 * (b1 - b0) + 0.5 * (m1 - m0)
                    mis = d_osm - need
                    if not beyond and abs(mis) > T["osm_sep"]:
                        self.add("gore_osm_sep", [B.id, M.id, T_.id], q[0], q[1], "major" if abs(mis) < 4 else "critical", mis,
                                 junction=jid, move=kind, d_osm=d_osm, need=need, mismatch=mis, gap_out=gap, hw=hw)
            # (c) branch follows its OSM way over the first 60 m
            if hw.endswith("_link") or hw in FREEWAY:
                Lr = min(T["ramp_len"], B.length)
                ss = np.linspace(B.length - Lr, B.length, 31) if be else np.linspace(0, Lr, 31)
                # the ways B was built from, those of the roads meeting it at either end (a gore trunk runs
                # on along the next gore's main way, which its osmWays do not name) and their neighbours
                ways = set(B.osm_ways)
                for lk in (B.pred, B.succ):
                    if lk is not None and lk.type == "junction":
                        for r2, _ in n.jends.get(lk.id, []):
                            ways |= set(r2.osm_ways)
                pb = [n.wpoly(w) for w in sorted(n.adjacent_ways(ways)) if len(n.wpoly(w)) >= 2 and
                      n.osm.ways[w][1].get("highway", "") in FREEWAY | {"primary_link", "secondary_link"}]
                cen = []
                for sv in ss:
                    e0, e1 = drive_extent(B, float(sv))
                    x, y = B.point(float(sv), 0.5 * (e0 + e1))
                    cen.append((float(x), float(y)))
                cen = np.array(cen)
                if not pb:
                    continue
                d = dist_ways(cen, pb)
                i = int(np.argmax(d))
                if d[i] > T["ramp_dev"]:
                    self.add("ramp_follow", [B.id, M.id], cen[i, 0], cen[i, 1], "major" if d[i] < 3 else "critical", float(d[i]),
                             junction=jid, move=kind, dev=float(d[i]), at_m=float(abs(ss[i] - sB)), hw=hw)

    def check_uncovered(self):
        """OSM freeway/link way samples near junctions that no driving lane of their roads covers"""
        n, od = self.n, self.od
        jpts = []
        for jid, conns in n.jconn.items():
            for c in conns:
                if n.hw(c) in FREEWAY or n.hw(c).endswith("_link"):
                    x, y, _ = c.ref(np.linspace(0, c.length, 5))
                    jpts.append(np.c_[x, y])
        if not jpts:
            return
        jpts = np.vstack(jpts)
        # junction connecting roads by plain road id
        conn_of = collections.defaultdict(set)
        for c in od.connecting_roads():
            for lk in (c.pred, c.succ):
                if lk is not None:
                    conn_of[lk.id].add(c.id)
        node_ways = collections.defaultdict(set)
        for w, (nds, tg) in n.osm.ways.items():
            if "highway" in tg:
                for x in nds:
                    node_ways[x].add(w)
        for w, (nds, tags) in n.osm.ways.items():
            hw = tags.get("highway", "")
            if not (hw in FREEWAY or hw.endswith("_link")) or w not in n.way_roads:
                continue
            A = n.wpoly(w)
            if len(A) < 2:
                continue
            seg = np.hypot(*np.diff(A, axis=0).T)
            cum = np.r_[0, np.cumsum(seg)]
            ss = np.arange(0, cum[-1], 2.0)
            P = np.c_[np.interp(ss, cum, A[:, 0]), np.interp(ss, cum, A[:, 1])]
            # only near junctions
            near = np.zeros(len(P), bool)
            for k in range(0, len(P)):
                near[k] = (np.hypot(jpts[:, 0] - P[k, 0], jpts[:, 1] - P[k, 1]) < T["uncovered_zone"]).any()
            w0, s0, e0_, n0 = n.bbox
            near &= (P[:, 0] > w0 + EDGE) & (P[:, 0] < e0_ - EDGE) & (P[:, 1] > s0 + EDGE) & (P[:, 1] < n0 - EDGE)
            if not near.any():
                continue
            # candidate roads: built from w or a way touching it, + their connecting roads
            ways = {w} | {v for x in (nds[0], nds[-1]) for v in node_ways[x]} | {v for x in nds for v in node_ways[x]}
            cand = set()
            for v in ways:
                cand |= n.way_roads.get(v, set())
            cand |= {c for r in list(cand) for c in conn_of[r]}
            roads = [od.roads[i] for i in cand]
            out = np.full(len(P), np.inf)
            for k in np.nonzero(near)[0]:
                best = np.inf
                for r in roads:
                    if not ((np.abs(r.X - P[k, 0]) < 40) & (np.abs(r.Y - P[k, 1]) < 40)).any():
                        continue
                    s, t, dist = r.project(*P[k])
                    x, y, _ = r.ref(s)
                    if (s <= 1e-3 or s >= r.length - 1e-3) and dist > abs(t) + 0.3:
                        continue  # beyond the road end
                    e0, e1 = drive_extent(r, s)
                    best = min(best, max(0.0, e0 - t, t - e1))
                    if best == 0:
                        break
                out[k] = best
            # inf: no candidate road within reach at all - the way runs outside the converted (clipped) area
            bad = near & (out > T["uncovered"]) & np.isfinite(out)
            # report runs
            k = 0
            while k < len(P):
                if not bad[k]:
                    k += 1
                    continue
                j = k
                while j + 1 < len(P) and bad[j + 1]:
                    j += 1
                seg_out = out[k:j + 1]
                fin = np.where(np.isfinite(seg_out), seg_out, 50.0)
                m = int(np.argmax(fin))
                self.add("osm_uncovered", sorted(cand & n.way_roads.get(w, set())), P[k + m, 0], P[k + m, 1],
                         "critical" if fin[m] > 3 else "major", float(fin[m]), way=w, hw=hw, run_m=float((j - k + 1) * 2.0),
                         outside=float(fin[m]))
                k = j + 1

    def check_size(self):
        n, od = self.n, self.od
        for jid, ends in n.jends.items():
            if len(ends) < 2:
                continue
            # a lane-level gore sits at the separation point by design, often far from the OSM node;
            # the gore checks judge where it sits
            if lane_level_gore(n.jconn.get(jid, [])):
                continue
            # OSM junction nodes: nodes shared by the end ways of two or more roads
            cnt = collections.Counter()
            for r, e in ends:
                w = n.end_way(r, e)
                if w and w in n.osm.ways:
                    for x in set(n.osm.ways[w][0]):
                        cnt[x] += 1
            nodes = [x for x, k in cnt.items() if k >= 2 and x in n.osm.nodes]
            if not nodes:
                continue
            NP = np.array([n.osm.nodes[x] for x in nodes])
            for r, e in ends:
                s = r.length if e else 0.0
                x, y, h = (float(v) for v in r.ref(s))
                d = float(np.hypot(NP[:, 0] - x, NP[:, 1] - y).min())
                # required: overlap distance with every other road at the node
                e0, e1 = drive_extent(r, s)
                hwid = 0.5 * (e1 - e0)
                hr = h if e else h + math.pi
                need = 0.0
                for r2, e2 in ends:
                    if r2 is r:
                        continue
                    s2 = r2.length if e2 else 0.0
                    f0, f1 = drive_extent(r2, s2)
                    h2 = float(r2.ref(s2)[2]) + (0 if e2 else math.pi)
                    ang = abs(math.sin(wrap(hr - h2)))
                    need = max(need, (0.5 * (f1 - f0) + hwid * abs(math.cos(wrap(hr - h2)))) / max(ang, math.sin(8 * D2R)))
                hwc = n.hw(r)
                lim = need + 1.5 * 10.0 + T["size_slack"] + (160.0 if hwc in FREEWAY else 0.0) * 0
                if d > lim and d > 30.0:
                    self.add("junction_size", [r.id], x, y, "major" if d > 2 * lim else "minor", d / lim, junction=jid, dist=d,
                             need=need, n_roads=len(ends), hw=hwc)


def check_span(self):
    """plain street junctions whose connecting lanes run much longer than one intersection needs:
    several OSM intersections clustered into one junction (src/network.cpp clusterJunctions)"""
    n = self.n
    node_ways = collections.Counter()
    for w, (nds, tg) in n.osm.ways.items():
        if tg.get("highway", "footway") not in NONROAD:
            for x in set(nds):
                node_ways[x] += 1
    inter = [x for x, k in node_ways.items() if k >= 2 and x in n.osm.nodes]
    IP = np.array([n.osm.nodes[x] for x in inter])
    for jid, conns in n.jconn.items():
        if not conns or any(n.hw(c) in FREEWAY or n.hw(c).endswith("_link") for c in conns):
            continue
        L = max(c.length for c in conns)
        if L <= T["span"]:
            continue
        pts = np.vstack([np.c_[c.ref(np.linspace(0, c.length, 9))[:2]] for c in conns])
        lo, hi = pts.min(0) - 2, pts.max(0) + 2
        cand = IP[(IP[:, 0] > lo[0]) & (IP[:, 0] < hi[0]) & (IP[:, 1] > lo[1]) & (IP[:, 1] < hi[1])]
        k = 0
        for q in cand:
            if (np.hypot(pts[:, 0] - q[0], pts[:, 1] - q[1]) < 4.0).any():
                k += 1
        c = max(conns, key=lambda c: c.length)
        x, y, _ = c.ref(0.5 * c.length)
        self.add("junction_span", [c.id, c.pred.id if c.pred else "", c.succ.id if c.succ else ""], float(x), float(y),
                 "major" if k >= 3 or L > 2 * T["span"] else "minor", L / T["span"], junction=jid, max_conn_len=float(L),
                 osm_intersections=k, n_conn=len(conns), hw=n.hw(c))


Detector.check_span = check_span


def check_gore_kink(self):
    """lane-level gores: the connectors (4 m, src/gore.cpp kJunctionLength) continue lanes of one
    carriageway, so they may turn no more than a ramp of 100 m radius would over their length
    (AASHTO minimum radius for a 35 mph ramp, e = 8 %) plus 1 deg; the excess is the ramp's
    divergence angle (or a lateral jog) folded into the junction: a kink at the gore nose"""
    n = self.n
    for jid, conns in n.jconn.items():
        if not lane_level_gore(conns):
            continue
        worst = None
        for c in conns:
            s, pts, h = conn_samples(c, 0.1)
            hu = np.unwrap(h)
            net = abs(float(hu[-1] - hu[0]))
            dev = float(np.abs(hu - hu[0]).max())
            allow = c.length / T["gore_radius"] + 1.0 * D2R
            if max(net, dev) > allow and (worst is None or max(net, dev) > worst[0]):
                worst = (max(net, dev), net, dev, c, pts[len(pts) // 2])
        if worst:
            m, net, dev, c, (x, y) = worst
            nbad = sum(1 for c2 in conns if np.abs(np.unwrap(conn_samples(c2, 0.1)[2]) - conn_samples(c2, 0.1)[2][0]).max()
                       > c2.length / T["gore_radius"] + D2R)
            self.add("gore_kink", [c.id, c.pred.id, c.succ.id], x, y, "critical" if m > 10 * D2R else "major", m / D2R,
                     junction=jid, net_turn_deg=net / D2R, peak_dev_deg=dev / D2R, length=c.length, kinked_connectors=nbad,
                     n_conn=len(conns), hw=n.hw(c))


Detector.check_gore_kink = check_gore_kink


def curv_at(r, s):
    """|curvature| of a plain road reference line near s (finite difference over 2 m)"""
    a, b = max(0.0, s - 2.0), min(r.length, s + 2.0)
    if b - a < 0.2:
        return 0.0
    _, _, h = r.ref(np.array([a, b]))
    return abs(float(h[1] - h[0])) / (b - a)


def nearest_on(A, p):
    a, b = A[:-1], A[1:]
    ab = b - a
    u = np.clip(((p - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
    q = a + u[:, None] * ab
    i = int(np.argmin(np.hypot(*(q - p).T)))
    return q[i]


def internal_ways(n, P, Q, c):
    """OSM ways that lie inside the junction (no road of their own) and touch both roads' end ways"""
    out = set()
    wp = P.osm_ways[-1:] if c.pred.contact == "end" else P.osm_ways[:1]
    wq = Q.osm_ways[-1:] if c.succ.contact == "end" else Q.osm_ways[:1]
    if not wp or not wq or wp[0] not in n.osm.ways or wq[0] not in n.osm.ways:
        return out
    np_ = set(n.osm.ways[wp[0]][0])
    nq = set(n.osm.ways[wq[0]][0])
    if np_ & nq:
        return out
    for w, (nds, tg) in n.osm.ways.items():
        if "highway" in tg and w not in n.way_roads and set(nds) & np_ and set(nds) & nq:
            out.add(w)
    return out


# ----------------------------------------------------------------------------- driver
def summarize(F, splits):
    by = collections.Counter(f["kind"] for f in F)
    sev = collections.Counter((f["kind"], f["severity"]) for f in F)
    fw = collections.Counter(f["kind"] for f in F if f.get("hw") in FREEWAY or str(f.get("hw", "")).endswith("_link"))
    lines = [f"junction_gore findings: {len(F)}", f"freeway diverge/merge junctions examined: {len(splits)}", ""]
    lines.append(f"{'kind':22s} {'total':>6s} {'crit':>5s} {'major':>6s} {'minor':>6s} {'fw/link':>8s}  metric p50/p90/max")
    for k in sorted(by, key=lambda k: -by[k]):
        m = np.array([f["metric"] for f in F if f["kind"] == k])
        lines.append(f"{k:22s} {by[k]:6d} {sev[(k, 'critical')]:5d} {sev[(k, 'major')]:6d} {sev[(k, 'minor')]:6d} {fw[k]:8d}  "
                     f"{np.percentile(m, 50):.2f}/{np.percentile(m, 90):.2f}/{m.max():.2f}")
    lines += ["", "thresholds: " + json.dumps({k: round(v, 4) for k, v in T.items()})]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr")
    ap.add_argument("osm")
    ap.add_argument("--out", required=True)
    ap.add_argument("--basemap")
    ap.add_argument("--crops", type=int, default=0, help="render crops for the N worst findings per kind")
    ap.add_argument("--only", help="comma separated check names")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    net = Net(a.xodr, a.osm)
    d = Detector(net)
    checks = dict(connecting=d.check_connecting, crossings=d.check_crossings, lane_links=d.check_lane_links,
                  turn_lanes=d.check_turn_lanes, gores=d.check_gores, uncovered=d.check_uncovered, size=d.check_size,
                  span=d.check_span, gore_kink=d.check_gore_kink)
    d.splits = []
    for name, fn in checks.items():
        if a.only and name not in a.only.split(","):
            continue
        fn()
    F = d.F
    with open(os.path.join(a.out, "findings.json"), "w") as f:
        json.dump(F, f, indent=1)
    txt = summarize(F, d.splits)
    with open(os.path.join(a.out, "summary.txt"), "w") as f:
        f.write(txt + "\n")
    print(txt)
    if a.crops:
        rank = {"critical": 0, "major": 1, "minor": 2}
        for k in sorted({f["kind"] for f in F}):
            fs = sorted([f for f in F if f["kind"] == k], key=lambda f: (rank[f["severity"]], -f["metric"]))
            pick = fs[:a.crops] + fs[len(fs) // 2:len(fs) // 2 + 2]
            for f in pick:
                p = os.path.join(a.out, "crops", f"{f['id'].replace(':', '_')}.png")
                G.crop(net.od, net.osm, (f["x"], f["y"]), 45, p, basemap=a.basemap, marks=[(f["x"], f["y"], f["kind"])],
                       title=f"{f['id']} roads {','.join(f['roads'][:4])} {f['severity']} m={f['metric']}")


if __name__ == "__main__":
    main()
