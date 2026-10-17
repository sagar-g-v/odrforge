#!/usr/bin/env python3
"""laneflow - "drive every lane": exact flow / topology invariants of an OpenDRIVE network, checked
against the OSM it was built from (docs/SPEC_GEOMETRY.md FLOW-0..6, NAT-1..4, TOP-1/2, JCT-1/2).

The user's natural-flow rule, made executable: a car that starts at the start of any lane must be
able to drive on (longitudinally, or by changing into an adjacent lane of the same direction) and
reach the edge of the map, without ever being dropped. Every lane must be usable: reachable from the
map edge and able to leave it again, unless OSM itself is a dead end or isolated there.

Directed lane graph (spec FLOW-0)
    node  = (road, laneSection index, lane id, travel direction d)   d = +1 along s, -1 against s
    travel direction from the lane id and the traffic rule: in RHT right lanes (id < 0) run +s, left
    lanes -s; in LHT the opposite (road@rule, default RHT, OpenDRIVE 1.5+); lane@direction
    ("reversed" flips, "both" = both) and type bidirectional give both directions.
    edges ('sec')  section -> section through lane predecessor / successor; the joint exists if
                   either side declares it (this lane's record or the next lane's record pointing
                   back); a one-sided declaration is link.lane_asymmetric (FLOW-2)
          ('road') road -> road through road links + contactPoint (a contact at 'end' means the
                   target lane is entered at its end and driven -s: the id direction flips)
          ('junc') incoming road -> connecting road only through junction connection laneLinks
          ('lat')  lane change into the adjacent lane of the same direction in the same section,
                   both wider than WIDTH_ZERO somewhere, in a direction the separating roadMark's
                   laneChange allows in at least one record (inner lane's mark; the centre lane's
                   for 1 / -1; no mark = both)
          ('uturn') turning round at a mapped dead end (OSM noexit / turning circle / dead-end street)
    motor graph: driving, entry, exit, onRamp, offRamp, connectingRamp, bidirectional, slipLane,
    mwyEntry, mwyExit, bus, taxi, HOV.  bike graph: biking (own graph, informative).
    sidewalk / shoulder / curb / border / walking / parking / biking continuity is checked (NAT-4).

OSM reference (same rules, independent of the converter's output)
    the OSM motor network clipped exactly like the converter clips it (Liang-Barsky at the clip box,
    junction nodes up to 30 m outside kept), as an edge-based directed graph (state = directed OSM
    segment) with oneway (incl. implied: motorway, motorway_link, roundabout), turn restrictions
    (via-node), no reversing on a segment, no U-turns sharper than 150 deg between different ways
    unless it is the only move (converter policy), turning round at OSM dead ends. A clipped piece
    lying wholly in the boundary band and touching no other way is an edge sliver and is dropped.
    every OSM sample point (each 2 m) is mapped onto the road surface of the road that names its way
    in <userData code="osmWays">, giving per road / direction the OSM truth for direction,
    reachability and road-to-road transitions.

Map edge: an unlinked road end is at the network edge when its end cross-section reaches within
eps_B = 2.0 m of the clip box (spec 0.2); an OSM way end is an edge node within the converter's
1.0 m onBoundary band (snap.cpp). The clip box comes from gate.py's area definition, --clip, or the
header extents (fallback, reported).

Invariants (severity defect = must be 0 for a correct output; ok / info = classification only).
Each output entry also carries its spec rule id ('spec') and level (MUST / SHOULD / INFO).
    dead_end.*        lane with no way on (a)       orphan.*        lane with no way in (b)
    flow.unreachable  not reachable from the edge (c) flow.trapped   cannot reach the edge
    trap_region       sink SCC with no exit (d)       island.*       components off the edge (e)
    link.direction    a link record joins opposite directions, once per record (f)
    link.*asymmetric  one-sided declarations          link.*         bad references
    unusable.*        zero length / zero width / narrower than W_P / section w/o motor lane (g)
    osm.direction.*   lanes vs OSM oneway (h)         osm.flow.*     OSM can drive it, lanes cannot (h)
    turn.missing      OSM allows road A -> road B at a junction, the lanes do not (JCT-1; turn:lanes
                      classes by order, JCT-2)
    pairs.missing     boundary entry -> exit pairs OSM can drive and the lanes cannot; pairs.*_unmapped
                      edge entries / exits OSM has and no lane has
    edge.*            sidewalk / shoulder / curb / parking / biking continuity (NAT-4)
    split.*           road_needless (NAT-3), road_to_road (NAT-1), section_redundant (NAT-2)
    rule.osm          the traffic rule the file declares contradicts OSM oneway evidence (TOP-1, SIM-3)

    python laneflow.py --area london                 # areas from gate.py
    python laneflow.py X.xodr X.osm --clip MINLAT,MINLON,MAXLAT,MAXLON --out DIR
    python laneflow.py --area london --rule LHT      # evaluate under an assumed rule (diagnosis)
    from laneflow import run; lf, findings = run(xodr, osm, clip_box_local)
    python laneflow_plant.py ...                     # planted-defect self test (see that file)
"""
import argparse
import collections
import json
import math
import os
import re
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import odrgeom as G  # noqa: E402

# ----------------------------------------------------------------------------- constants
MOTOR_TYPES = frozenset({"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional",
                         "slipLane", "mwyEntry", "mwyExit", "bus", "taxi", "HOV"})
BIKE_TYPES = frozenset({"biking"})
EDGE_TYPES = frozenset({"sidewalk", "shoulder", "curb", "border", "walking"})
# lanes whose continuity across section / road-to-road boundaries is checked (spec NAT-4 adds biking
# and parking to the edge lanes)
CONTINUITY_TYPES = EDGE_TYPES | frozenset({"biking", "parking"})
# converter's motor highway set and exclusions (BuildOptions::highways / excludeService); the OSM
# reference uses the same set so both sides describe the same network
HIGHWAYS = frozenset({"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
                      "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street",
                      "service", "road", "busway"})
EXCLUDE_SERVICE = frozenset({"parking_aisle", "drive-through", "emergency_access"})
EDGE_TAG_KEYS = ("sidewalk", "sidewalk:left", "sidewalk:right", "sidewalk:both", "shoulder", "shoulder:left",
                 "shoulder:right", "shoulder:both", "kerb", "curb", "cycleway", "cycleway:left", "cycleway:right",
                 "cycleway:both", "parking:left", "parking:right", "parking:both", "parking:lane:left",
                 "parking:lane:right", "parking:lane:both")
SPLIT_TAG_KEYS = ("highway", "name", "ref", "maxspeed", "lanes", "lanes:forward", "lanes:backward", "oneway",
                  "bridge", "tunnel", "layer", "surface", "width", "turn:lanes", "turn:lanes:forward",
                  "turn:lanes:backward", "sidewalk", "cycleway", "junction", "access")

BOUNDARY_BAND = 1.0      # m: converter's onBoundary band (snap.cpp); the clip cuts exactly at the box
#                          (used for OSM way ends: a way end this close to the box is an edge node)
ROAD_END_BAND = 2.0      # m: spec eps_B (docs/SPEC_GEOMETRY.md 0.2): an unlinked road end within it of
#                          the box (or outside) is a boundary end
KEEP_OUTSIDE = 30.0      # m: clipOsm keeps a junction node this far outside the box (network.cpp)
MAP_TOL = 1.5            # m: PolylineFitOptions::maxCornerDeviation, the most a reference line may
#                          leave the OSM polyline; an OSM point within it of a road surface lies on it
WIDTH_ZERO = 1e-3        # m: a width below 1 mm is zero (far above float rounding of written cubics,
#                          far below any physical lane)
MIN_VEHICLE_WIDTH = 2.13  # m: W_P of docs/SPEC_GEOMETRY.md FLOW-6, the AASHTO passenger-car design
#                          vehicle width (7 ft, AASHTO Green Book Table 2-1a); a drivable lane whose run
#                          never reaches it is 'narrow'. (The first version used 1.48 m, the kei-car
#                          limit, which disagreed with the spec.)
UTURN_DEG = 150.0        # deg: converter policy (network_junctions.cpp classify): sharper is a U-turn
ABSORB_MAX = 300.0       # m: longest OSM stretch inside one junction followed between two mapped roads
SAMPLE_STEP = 2.0        # m: OSM sample spacing for mapping OSM onto roads
ABSORB_JUNCTION = 30.0   # m: a road end this much (beyond KEEP_OUTSIDE) from an edge node, through a
#                          junction, still belongs to that edge (junction trim: corner radius x major
#                          road scaling + stop-line setback stay well below it in the converter)
TAPER_LEN = 30.0         # m: BuildOptions::taperLength, the converter's lane add / drop transition
ZERO_LEN = 1e-6          # m: a section shorter than this has no extent (degenerate)

SEVERITY = {}            # invariant -> defect / ok / info (filled by Findings.add)


# ----------------------------------------------------------------------------- findings
class Findings:
    """{invariant: {severity, count, instances[{road, lane, s, x, y, value, limit, ...}]}}"""

    def __init__(self):
        self.d = collections.OrderedDict()

    def add(self, name, severity, road=None, lane=None, s=None, x=None, y=None, value=None, limit=None, **kw):
        e = self.d.setdefault(name, {"severity": severity, "count": 0, "instances": []})
        if e["severity"] != severity:  # one invariant, one severity (a mixed one would hide defects)
            raise ValueError(f"{name}: severity {severity} != {e['severity']}")
        e["count"] += 1
        inst = {"road": road, "lane": lane, "s": _r(s), "x": _r(x), "y": _r(y), "value": _r(value), "limit": _r(limit)}
        inst.update({k: (_r(v) if isinstance(v, float) else v) for k, v in kw.items()})
        e["instances"].append(inst)

    def touch(self, name, severity):
        self.d.setdefault(name, {"severity": severity, "count": 0, "instances": []})

    def as_dict(self, cap=5000):
        out = collections.OrderedDict()
        for k in sorted(self.d):
            e = self.d[k]
            inst = sorted(e["instances"], key=lambda i: (str(i.get("road")), str(i.get("lane")), i.get("s") or 0,
                                                          json.dumps(i, sort_keys=True, default=str)))
            out[k] = {"severity": e["severity"], "count": e["count"], "instances": inst[:cap]}
        return out


def _r(v, n=3):
    if v is None:
        return None
    if isinstance(v, (float, np.floating)):
        return round(float(v), n)
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


# ----------------------------------------------------------------------------- OpenDRIVE side
def lane_dirs(ltype, lid, rule, direction):
    """travel directions (+1 along s, -1 against) of a lane under a traffic rule."""
    if ltype == "bidirectional" or direction == "both":
        return (1, -1)
    d = 1 if lid < 0 else -1
    if rule == "LHT":
        d = -d
    if direction == "reversed":
        d = -d
    return (d,)


def load_junctions(path):
    """{junction id: {type, conns[{incoming, connecting, linked, contact, lanes[(from, to)]}]}} -
    odrgeom keeps no junction type / linkedRoad, so they are read here (streaming)."""
    out = {}
    for ev, el in ET.iterparse(str(path), events=("end",)):
        if el.tag == "junction" and el.get("id") is not None:
            conns = []
            for c in el.findall("connection"):
                conns.append({"incoming": c.get("incomingRoad"), "connecting": c.get("connectingRoad"),
                              "linked": c.get("linkedRoad"), "contact": c.get("contactPoint"),
                              "lanes": [(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")]})
            out[el.get("id")] = {"type": el.get("type", "default"), "conns": conns}
            el.clear()
        elif el.tag == "road":
            el.clear()
    return out


class Network:
    """OpenDRIVE + per-lane attributes odrgeom does not keep (lane direction, road rule)."""

    def __init__(self, xodr, rule_override=None):
        self.path = xodr
        self.od = G.OpenDrive(xodr)
        self.roads = self.od.roads
        self.junctions = load_junctions(xodr)
        self.rule_declared = {}
        self.rule = {}
        self.lane_attr = {}  # (road, k, lid) -> direction attribute
        self.center_marks = {}  # (road, k) -> [roadMark attributes] of the centre lane
        for rid, r in self.roads.items():
            decl = r.el.get("rule") or "RHT"
            self.rule_declared[rid] = decl
            self.rule[rid] = rule_override or decl
            secs = sorted(r.el.find("lanes").findall("laneSection"), key=lambda e: float(e.get("s")))
            for k, se in enumerate(secs):
                for le in se.iter("lane"):
                    if le.get("direction"):
                        self.lane_attr[(rid, k, int(le.get("id")))] = le.get("direction")
                c = se.find("center")
                cl = c.find("lane") if c is not None else None
                self.center_marks[(rid, k)] = [dict(m.attrib) for m in cl.findall("roadMark")] if cl is not None else []
        self.by_incoming = collections.defaultdict(list)  # (junction, incoming road) -> connections
        for jid, j in self.junctions.items():
            for c in j["conns"]:
                self.by_incoming[(jid, c["incoming"])].append(c)

    def dirs(self, rid, k, lane):
        return lane_dirs(lane.type, lane.id, self.rule[rid], self.lane_attr.get((rid, k, lane.id)))

    def is_plain(self, rid):
        return self.roads[rid].junction == "-1"

    def lane_change(self, rid, k, a, b):
        """(a -> b allowed, b -> a allowed) for two adjacent lanes a, b of section k (a.id > b.id).
        The separating mark is the roadMark of the inner lane of the pair (the centre lane's for 1 / -1);
        roadMark@laneChange 'increase' allows a move toward the higher id, 'decrease' toward the lower,
        'both' or a missing attribute both, 'none' neither. A pair is linked in a direction if at least
        one record of the mark allows it; no mark at all links both (spec FLOW-0)."""
        if a.id > 0 and b.id < 0:
            marks = self.center_marks.get((rid, k), [])
        else:
            inner = b if a.id > 0 else a  # left: the lower id is inner; right: the higher id is inner
            marks = inner.marks
        if not marks:
            return True, True
        vals = [m.get("laneChange") for m in marks]
        down = any(v in (None, "both", "decrease") for v in vals)  # a (higher id) -> b (lower id)
        up = any(v in (None, "both", "increase") for v in vals)
        return down, up


def lane_width(sec, lane, s):
    return float(lane.width(np.array([s - sec.s]))[0])


def lane_widths(sec, lane, n=9):
    ss = np.linspace(sec.s, sec.e, n) if sec.e > sec.s else np.array([sec.s])
    return lane.width(ss - sec.s)


def lane_max_width(sec, lane):
    """exact maximum of a lane's width over its section: every cubic width record evaluated at its
    endpoints and at the real roots of its derivative inside its range (spec 0.3: exact evaluation)."""
    L = sec.e - sec.s
    best = -math.inf
    for i, (s0, (a, b, c, d)) in enumerate(lane.widths):
        s1 = lane.widths[i + 1][0] if i + 1 < len(lane.widths) else L
        lo = 0.0 if i == 0 else s0  # odrgeom: the first record also covers ds < its sOffset
        hi = min(s1, L)
        if hi < lo:
            continue
        us = [lo - s0, hi - s0]
        if abs(d) > 1e-15:
            disc = 4 * c * c - 12 * d * b
            if disc >= 0:
                us += [(-2 * c + sg * math.sqrt(disc)) / (6 * d) for sg in (1, -1)]
        elif abs(c) > 1e-15:
            us.append(-b / (2 * c))
        for u in us:
            if lo - s0 - 1e-12 <= u <= hi - s0 + 1e-12:
                best = max(best, a + b * u + c * u * u + d * u ** 3)
    return best


def lane_centre(r, sec, lid, s):
    """(x, y, width) of a lane's centre at s inside the given section."""
    b = r.borders_at(s, sec)
    ti, to = b[lid]
    x, y = r.point(np.array([s]), np.array([(ti + to) / 2]))
    return float(x[0]), float(y[0]), abs(to - ti)


def extent_many(r, s):
    """outer (t_lo, t_hi) of all lanes at many s (vectorised borders)."""
    s = np.asarray(s, float)
    off = r.lane_offset(s)
    lo, hi = off.copy(), off.copy()
    for i, sec in enumerate(r.sections):
        m = (s >= sec.s - 1e-9) & ((s < sec.e) if i + 1 < len(r.sections) else np.ones_like(s, bool))
        if i == 0:
            m = (s < sec.e) | (len(r.sections) == 1)
        if not m.any():
            continue
        ds = s[m] - sec.s
        wl = sum((l.width(ds) for l in sec.left), np.zeros(m.sum()))
        wr = sum((l.width(ds) for l in sec.right), np.zeros(m.sum()))
        hi[m] = off[m] + wl
        lo[m] = off[m] - wr
    return lo, hi


def project_many(r, px, py):
    """nearest (s, t) on the reference line for many points (dense search + 3 Newton steps)."""
    px, py = np.asarray(px, float), np.asarray(py, float)
    x0, x1, y0, y1 = px.min() - 60, px.max() + 60, py.min() - 60, py.max() + 60
    w = np.nonzero((r.X >= x0) & (r.X <= x1) & (r.Y >= y0) & (r.Y <= y1))[0]
    if len(w) == 0:
        w = np.arange(len(r.S))
    d2 = (r.X[w][None, :] - px[:, None]) ** 2 + (r.Y[w][None, :] - py[:, None]) ** 2
    s = r.S[w][np.argmin(d2, axis=1)]
    for _ in range(3):
        x, y, h = r.ref(s)
        s = np.clip(s + (px - x) * np.cos(h) + (py - y) * np.sin(h), 0, r.length)
    x, y, h = r.ref(s)
    t = -(px - x) * np.sin(h) + (py - y) * np.cos(h)
    return s, t, h


# ----------------------------------------------------------------------------- clip box
def clip_box_from_latlon(clip, lat0, lon0):
    """converter's --clip: min/max of the four projected corners (tools/osm2xodr.cpp)."""
    a, b, c, d = clip
    xs, ys = G.tm_forward(np.array([a, c, a, c]), np.array([b, d, d, b]), lat0, lon0)
    return (float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()))


def inside_dist(box, x, y):
    """signed distance to the box edge (positive inside)."""
    return min(x - box[0], box[2] - x, y - box[1], box[3] - y)


# ----------------------------------------------------------------------------- OSM reference
def osm_oneway(tags):
    ow = tags.get("oneway", "")
    if ow in ("-1", "reverse"):
        return -1
    if ow in ("yes", "true", "1"):
        return 1
    if ow == "no":
        return 0
    if tags.get("junction") in ("roundabout", "circular") or tags.get("highway") in ("motorway", "motorway_link"):
        return 1
    return 0


def _yes(v):
    return v in ("yes", "true", "1")


class OsmNet:
    """OSM motor network as the converter sees it (clipped), as an edge-based directed graph."""

    def __init__(self, osm, box, extra_ways=frozenset()):
        self.osm = osm
        self.box = box
        pos = dict(osm.nodes)
        use = collections.Counter()
        for wid, (nds, tags) in osm.ways.items():
            if "highway" in tags:
                use.update(nds)
        inside = (lambda p: box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3])
        self.pieces = []  # {id (xodr osmWays spelling), orig, nodes, tags, oneway}
        synth = [0]

        def new_node(p):
            synth[0] += 1
            k = f"c{synth[0]}"
            pos[k] = (float(p[0]), float(p[1]))
            return k

        for wid in sorted(osm.ways, key=lambda w: int(w)):
            nds, tags = osm.ways[wid]
            if not self._included(wid, nds, tags, extra_ways, pos if box is None else None):
                continue
            if box is None:
                parts = [list(nds)]
            else:
                parts = [[]]
                for i in range(len(nds) - 1):
                    if nds[i] not in pos or nds[i + 1] not in pos:
                        continue
                    a = np.array(pos[nds[i]]), np.array(pos[nds[i + 1]])
                    d = a[1] - a[0]
                    t0, t1 = 0.0, 1.0
                    p = (-d[0], d[0], -d[1], d[1])
                    q = (a[0][0] - box[0], box[2] - a[0][0], a[0][1] - box[1], box[3] - a[0][1])
                    vis = True
                    for k in range(4):
                        if abs(p[k]) < 1e-12:
                            vis = q[k] >= 0
                            if not vis:
                                break
                        elif p[k] < 0:
                            t0 = max(t0, q[k] / p[k])
                        else:
                            t1 = min(t1, q[k] / p[k])
                    if not vis or t0 > t1:
                        if parts[-1]:
                            parts.append([])
                        continue

                    def keep(n, cut):
                        return use[n] >= 2 and math.hypot(pos[n][0] - cut[0], pos[n][1] - cut[1]) <= KEEP_OUTSIDE

                    ca, cb = a[0] + d * t0, a[0] + d * t1
                    na = nds[i] if t0 <= 1e-12 or keep(nds[i], ca) else new_node(ca)
                    nb = nds[i + 1] if t1 >= 1 - 1e-12 or keep(nds[i + 1], cb) else new_node(cb)
                    if not parts[-1] or parts[-1][-1] != na:
                        if parts[-1]:
                            parts.append([])
                        parts[-1].append(na)
                    parts[-1].append(nb)
                    if nb != nds[i + 1]:
                        parts.append([])
            first = True
            for k, pt in enumerate(p for p in parts if len(p) >= 2):
                self.pieces.append({"id": wid if first else f"{wid}#{k}", "orig": wid, "first": first,
                                    "nodes": pt, "tags": tags, "oneway": osm_oneway(tags)})
                first = False
        self.pos = pos
        self.inside = inside
        if box is not None:
            # a clipped piece lying wholly inside the converter's onBoundary band and sharing no node
            # with any other piece is on the edge line itself (e.g. a 0.3 m sliver between a cut point
            # and a way end just inside the box): no road can be built along it and it connects
            # nothing, so it is not part of the network. A band piece that joins another way is an
            # entry into the network and stays.
            cnt = collections.Counter(n for pc in self.pieces for n in set(pc["nodes"]))
            self.edge_slivers = [pc["id"] for pc in self.pieces
                                 if all(inside_dist(box, *pos[n]) <= BOUNDARY_BAND and cnt[n] == 1
                                        for n in pc["nodes"])]
            sl = set(self.edge_slivers)
            self.pieces = [pc for pc in self.pieces if pc["id"] not in sl]
        else:
            self.edge_slivers = []
        # segments, incidence
        self.seg = []  # (piece index, i, a, b, length, heading)
        self.at = collections.defaultdict(list)  # node -> segment indices
        for pi, pc in enumerate(self.pieces):
            for i in range(len(pc["nodes"]) - 1):
                a, b = pc["nodes"][i], pc["nodes"][i + 1]
                (xa, ya), (xb, yb) = pos[a], pos[b]
                g = len(self.seg)
                self.seg.append((pi, i, a, b, math.hypot(xb - xa, yb - ya), math.atan2(yb - ya, xb - xa)))
                self.at[a].append(g)
                self.at[b].append(g)
        # network-edge nodes: clip cut points, nodes kept outside the box (junctions just beyond the cut)
        # and way ends within the converter's onBoundary band; a way's interior node near the edge is
        # driven through, it is no entry
        self.synthetic = {n for n in pos if isinstance(n, str) and n.startswith("c")}
        self.boundary = set()
        if box is not None:
            for n in self.at:
                d = inside_dist(box, *pos[n])
                if n in self.synthetic or d < 0 or (d <= BOUNDARY_BAND and len(self.at[n]) == 1):
                    self.boundary.add(n)
        self._restrictions()
        self._states()

    @staticmethod
    def _included(wid, nds, tags, extra, pos_full):
        hw = tags.get("highway")
        if hw not in HIGHWAYS or len(nds) < 2 or _yes(tags.get("area", "")):
            return False
        if pos_full is not None and any(n not in pos_full for n in nds):
            return False  # unclipped: the converter drops ways with missing nodes
        if wid in extra:
            return True
        if hw == "service" and tags.get("service") in EXCLUDE_SERVICE:
            return False
        if tags.get("access") == "no" and "bus" not in tags and "psv" not in tags:
            return False
        return True

    def _restrictions(self):
        self.restr = collections.defaultdict(list)  # via node -> [(from, to, only)]
        for tags, members in self.osm.relations:
            if tags.get("type") != "restriction" or not tags.get("restriction"):
                continue
            fr = [m[1] for m in members if m[2] == "from" and m[0] == "way"]
            to = [m[1] for m in members if m[2] == "to" and m[0] == "way"]
            via = [m[1] for m in members if m[2] == "via" and m[0] == "node"]
            if fr and to and via:
                self.restr[via[0]].append((fr[0], to[0], tags["restriction"].startswith("only_")))

    # state = 2 * segment + (0 forward a->b, 1 backward b->a)
    def state_ok(self, st):
        ow = self.pieces[self.seg[st >> 1][0]]["oneway"]
        return ow == 0 or (ow == 1) == ((st & 1) == 0)

    def s_from(self, st):
        g = self.seg[st >> 1]
        return g[2] if st & 1 == 0 else g[3]

    def s_to(self, st):
        g = self.seg[st >> 1]
        return g[3] if st & 1 == 0 else g[2]

    def s_head(self, st):
        h = self.seg[st >> 1][5]
        return h if st & 1 == 0 else h + math.pi

    def _states(self):
        n = 2 * len(self.seg)
        self.valid = [self.state_ok(st) for st in range(n)]
        raw = [[] for _ in range(n)]  # candidate moves with a U-turn flag
        for st in range(n):
            if not self.valid[st]:
                continue
            v = self.s_to(st)
            g = st >> 1
            pin = self.pieces[self.seg[g][0]]
            deg = len(self.at[v])
            for g2 in self.at[v]:
                for st2 in (2 * g2, 2 * g2 + 1):
                    if not self.valid[st2] or self.s_from(st2) != v:
                        continue
                    if g2 == g:  # reversing on the same segment: only at a dead end of the network
                        if deg == 1 and v not in self.boundary:
                            raw[st].append((st2, False))
                        continue
                    pout = self.pieces[self.seg[g2][0]]
                    same_piece_cont = pout is pin and abs(self.seg[g2][1] - self.seg[g][1]) == 1
                    if self._forbidden(pin["orig"], v, pout["orig"]):
                        continue
                    turn = abs(math.degrees(_wrap(self.s_head(st2) - self.s_head(st))))
                    raw[st].append((st2, turn > UTURN_DEG and not same_piece_cont and pout is not pin))
                    if turn > UTURN_DEG and pout is pin and not same_piece_cont:
                        raw[st].pop()  # back into the same way (loop way): never (converter policy)
        has_reg_out = [any(not u for _, u in raw[st]) for st in range(n)]
        has_reg_in = [False] * n
        for st in range(n):
            for st2, u in raw[st]:
                if not u:
                    has_reg_in[st2] = True
        self.succ = [[] for _ in range(n)]
        self.pred = [[] for _ in range(n)]
        for st in range(n):
            for st2, u in raw[st]:
                if u and has_reg_out[st] and has_reg_in[st2]:
                    continue  # a U-turn where regular moves exist
                self.succ[st].append(st2)
                self.pred[st2].append(st)
        self.entries = [st for st in range(n) if self.valid[st] and self.s_from(st) in self.boundary]
        self.exits = [st for st in range(n) if self.valid[st] and self.s_to(st) in self.boundary]
        self.fwd = _bfs(self.entries, self.succ, n)
        self.bwd = _bfs(self.exits, self.pred, n)

    def _forbidden(self, frm, via, to):
        for f, t, only in self.restr.get(via, ()):
            if f == frm and ((only and t != to) or (not only and t == to)):
                return True
        return False

    def dead_end_node(self, n):
        """mapped dead end: OSM degree 1 (not at the edge), noexit, turning circle / loop, barrier, entrance."""
        if n in self.boundary:
            return False
        t = self.osm.node_tags.get(n, {})
        if _yes(t.get("noexit", "")) or t.get("highway") in ("turning_circle", "turning_loop") or "barrier" in t \
                or "entrance" in t:
            return True
        if len(self.at[n]) == 1:
            return True
        return any(_yes(self.pieces[self.seg[g][0]]["tags"].get("noexit", "")) for g in self.at[n])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _bfs(starts, adj, n):
    seen = np.zeros(n, bool)
    st = list(starts)
    for s in st:
        seen[s] = True
    while st:
        u = st.pop()
        for v in adj[u]:
            if not seen[v]:
                seen[v] = True
                st.append(v)
    return seen


def _scc(n, adj):
    """iterative Tarjan: component index per node, components in reverse topological order."""
    index = [-1] * n
    low = [0] * n
    on = [False] * n
    comp = [-1] * n
    stack, out = [], 0
    counter = 0
    for root in range(n):
        if index[root] != -1:
            continue
        work = [(root, 0)]
        index[root] = low[root] = counter
        counter += 1
        stack.append(root)
        on[root] = True
        while work:
            v, i = work[-1]
            if i < len(adj[v]):
                work[-1] = (v, i + 1)
                w = adj[v][i]
                if index[w] == -1:
                    index[w] = low[w] = counter
                    counter += 1
                    stack.append(w)
                    on[w] = True
                    work.append((w, 0))
                elif on[w]:
                    low[v] = min(low[v], index[w])
            else:
                work.pop()
                if work:
                    low[work[-1][0]] = min(low[work[-1][0]], low[v])
                if low[v] == index[v]:
                    while True:
                        w = stack.pop()
                        on[w] = False
                        comp[w] = out
                        if w == v:
                            break
                    out += 1
    return comp, out


# ----------------------------------------------------------------------------- lane graph
class LaneGraph:
    """directed lane graph of one lane-type family (motor or bike)."""

    def __init__(self, net, types, F, prefix):
        self.net, self.types, self.F, self.p = net, types, F, prefix
        self.idx, self.key = {}, []
        roads = net.roads
        for rid in sorted(roads, key=_rid_key):
            r = roads[rid]
            for k, sec in enumerate(r.sections):
                for l in sec.lanes():
                    if l.type in types:
                        for d in net.dirs(rid, k, l):
                            self.idx[(rid, k, l.id, d)] = len(self.key)
                            self.key.append((rid, k, l.id, d))
        n = len(self.key)
        self.out = [[] for _ in range(n)]
        self.inn = [[] for _ in range(n)]
        self.dir_errors = 0
        self.lat_blocked = 0  # adjacent same-direction pairs a road mark forbids in one or both ways
        for i, (rid, k, lid, d) in enumerate(self.key):
            for j, kind in self._longitudinal(i):
                self._edge(i, j, kind)
        self._lateral()

    def _edge(self, i, j, kind):
        self.out[i].append((j, kind))
        self.inn[j].append((i, kind))

    def lane(self, i):
        rid, k, lid, d = self.key[i]
        return self.net.roads[rid], self.net.roads[rid].sections[k], self.net.roads[rid].sections[k].lane(lid)

    def exit_s(self, i):
        rid, k, lid, d = self.key[i]
        sec = self.net.roads[rid].sections[k]
        return sec.e if d > 0 else sec.s

    def entry_s(self, i):
        rid, k, lid, d = self.key[i]
        sec = self.net.roads[rid].sections[k]
        return sec.s if d > 0 else sec.e

    def at_road_exit(self, i):
        rid, k, lid, d = self.key[i]
        return (k == len(self.net.roads[rid].sections) - 1) if d > 0 else k == 0

    def at_road_entry(self, i):
        rid, k, lid, d = self.key[i]
        return k == 0 if d > 0 else k == len(self.net.roads[rid].sections) - 1

    def _target(self, i, rid2, k2, lid2, d2, kind):
        """resolve a link target; record reference / type / direction errors."""
        net, F, p = self.net, self.F, self.p
        rid, k, lid, d = self.key[i]
        r2 = net.roads.get(rid2)
        x, y = self._xy(i, self.exit_s(i))
        if r2 is None:
            F.add(p + "link.bad_reference", "defect", rid, lid, self.exit_s(i), x, y, detail=f"road {rid2} missing")
            return None
        sec2 = r2.sections[k2]
        l2 = sec2.lane(lid2)
        if l2 is None:
            F.add(p + "link.bad_reference", "defect", rid, lid, self.exit_s(i), x, y,
                  detail=f"{kind}: lane {lid2} not in road {rid2} section {k2}")
            return None
        if l2.type not in self.types:  # the lane flows into a lane of another family: dropped here
            F.add(p + "link.to_non_traversable", "defect", rid, lid, self.exit_s(i), x, y,
                  detail=f"{kind}: -> road {rid2} lane {lid2} type {l2.type}")
            return None
        dirs2 = net.dirs(rid2, k2, l2)
        if d2 not in dirs2:
            # a link against the traffic is no edge; it is reported once per link record by
            # LaneFlow.link_directions (static pass), not here (a traversal sees some records twice
            # and never sees the records of lanes that travel away from them)
            self.dir_errors += 1
            return None
        return self.idx[(rid2, k2, lid2, d2)]

    def _xy(self, i, s):
        rid, k, lid, d = self.key[i]
        r = self.net.roads[rid]
        x, y, _ = r.ref(np.array([s]))
        return float(x[0]), float(y[0])

    def _longitudinal(self, i):
        net = self.net
        rid, k, lid, d = self.key[i]
        r = net.roads[rid]
        lane = r.sections[k].lane(lid)
        nsec = len(r.sections)
        out = []
        if (d > 0 and k + 1 < nsec) or (d < 0 and k > 0):
            nid = lane.succ if d > 0 else lane.pred
            # spec FLOW-0 link declaration: the joint exists if either side declares it (this lane's
            # record, or the next lane's record pointing back); one-sided = FLOW-2/asymmetric
            nids = [nid] if nid is not None else                 [m.id for m in r.sections[k + d].lanes() if getattr(m, "pred" if d > 0 else "succ") == lid]
            for q in nids:
                j = self._target(i, rid, k + d, q, d, "sec")
                if j is not None:
                    out.append((j, "sec"))
            return out
        link = r.succ if d > 0 else r.pred
        nid = lane.succ if d > 0 else lane.pred
        if link is None:
            return out
        if link.type == "road":
            if link.id not in net.roads:
                return out
            T = net.roads[link.id]
            k2, d2 = (0, 1) if link.contact == "start" else (len(T.sections) - 1, -1)
            if nid is not None:
                nids = [nid]
            else:  # declared by the other road's lane records, if that road links back to this end
                back = T.pred if link.contact == "start" else T.succ
                ok = back is not None and back.type == "road" and back.id == rid and                     back.contact == ("end" if d > 0 else "start")
                attr = "pred" if link.contact == "start" else "succ"
                nids = [m.id for m in T.sections[k2].lanes() if getattr(m, attr) == lid] if ok else []
            for q in nids:
                j = self._target(i, link.id, k2, q, d2, "road")
                if j is not None:
                    out.append((j, "road"))
        elif link.type == "junction":
            for c in net.by_incoming.get((link.id, rid), ()):
                tgt = c["connecting"] or c["linked"]
                if tgt not in net.roads:
                    continue
                T = net.roads[tgt]
                k2, d2 = (0, 1) if c["contact"] == "start" else (len(T.sections) - 1, -1)
                for f, t in c["lanes"]:
                    if f == lid:
                        j = self._target(i, tgt, k2, t, d2, "junc")
                        if j is not None:
                            out.append((j, "junc"))
        return out

    def _lateral(self):
        """lane change into the adjacent lane (same section, same direction, both with width)."""
        net = self.net
        for rid in sorted(net.roads, key=_rid_key):
            r = net.roads[rid]
            for k, sec in enumerate(r.sections):
                order = sorted(sec.left, key=lambda l: -l.id) + sorted(sec.right, key=lambda l: -l.id)
                ws = {l.id: lane_widths(sec, l) for l in order}
                order = [l for l in order if ws[l.id].max() > WIDTH_ZERO]  # zero-width lanes separate nothing
                for a, b in zip(order, order[1:]):
                    if a.type not in self.types or b.type not in self.types:
                        continue
                    if not ((ws[a.id] > WIDTH_ZERO) & (ws[b.id] > WIDTH_ZERO)).any():
                        continue
                    down, up = net.lane_change(rid, k, a, b)
                    for d in sorted(set(net.dirs(rid, k, a)) & set(net.dirs(rid, k, b))):
                        i, j = self.idx[(rid, k, a.id, d)], self.idx[(rid, k, b.id, d)]
                        if down:
                            self._edge(i, j, "lat")
                        if up:
                            self._edge(j, i, "lat")
                        if not (down and up):
                            self.lat_blocked += 1

    def has_long_out(self, i):
        return any(kd != "lat" for _, kd in self.out[i])

    def has_long_in(self, i):
        return any(kd != "lat" for _, kd in self.inn[i])

    def has_lat(self, i):
        return any(kd == "lat" for _, kd in self.out[i])

    def has_lat_in(self, i):
        return any(kd == "lat" for _, kd in self.inn[i])


def _rid_key(rid):
    try:
        return (0, int(rid))
    except ValueError:
        return (1, rid)


# ----------------------------------------------------------------------------- OSM <-> road mapping
class Mapping:
    """every OSM sample (2 m) on the road surface of the road that names its way."""

    def __init__(self, net, on):
        self.net, self.on = net, on
        od = net.od
        self.by_way = collections.defaultdict(set)
        neg = set()
        for rid, r in net.roads.items():
            if r.junction != "-1":
                continue
            for w in r.osm_ways:
                if w.startswith("-"):
                    neg.add(rid)
                else:
                    self.by_way[w].add(rid)
        self.seg_seq = []        # per segment: [(road, sdir, s, px, py)] along a->b (mapped samples)
        self.seg_dropped = []    # per segment: some sample on no road surface at all
        self.seg_cover = []      # per segment: {sample: junction id / "road:id"} for unmapped samples
        self.seg_samples = []    # per segment: per sample (road, sdir, s, x, y) or None, a -> b
        self.seg_unmapped = []   # per segment: no sample mapped to its own road
        self.road_osm = collections.defaultdict(list)  # road -> [(s, dirs allowed by OSM, piece)]
        self.road_sd = collections.defaultdict(list)   # road -> [(s, way direction along s +-1, piece)]
        for g, (pi, i, a, b, L, hd) in enumerate(on.seg):
            pc = on.pieces[pi]
            (xa, ya), (xb, yb) = on.pos[a], on.pos[b]
            n = max(1, int(math.ceil(L / SAMPLE_STEP)))
            u = (np.arange(n) + 0.5) / n
            px, py = xa + (xb - xa) * u, ya + (yb - ya) * u
            mx, my = (xa + xb) / 2, (ya + yb) / 2
            near = od.roads_near(mx, my, L / 2 + 40)
            cands = [r for r in near if r.id in (self.by_way.get(pc["id"], set()) if pc["first"] else neg)]
            if pc["first"] and not cands:
                cands = [r for r in near if r.id in self.by_way.get(pc["orig"], set())]
            best = [None] * n
            bestq = np.full(n, np.inf)
            for r in cands:
                s, t, h = project_many(r, px, py)
                lo, hi = extent_many(r, s)
                out = np.maximum(lo - t, 0) + np.maximum(t - hi, 0)
                ok = (s > 1e-6) & (s < r.length - 1e-6) & (out <= MAP_TOL)
                q = out + 1e-3 * np.abs(t)
                sd = np.where(np.cos(h - hd) >= 0, 1, -1)
                for m in np.nonzero(ok & (q < bestq))[0]:
                    bestq[m] = q[m]
                    best[m] = (r.id, int(sd[m]), float(s[m]), float(px[m]), float(py[m]))
            seq = [b_ for b_ in best if b_ is not None]
            dropped = False
            cover = {}  # sample index -> junction id / "road:id" whose surface holds the unmapped sample
            if len(seq) < n:  # unmapped samples: inside some junction / other road surface, or nowhere
                miss = [m for m in range(n) if best[m] is None]
                cov = np.zeros(len(miss), bool)
                for r in sorted(near, key=lambda q: q.junction == "-1"):  # junction surfaces first
                    if r.el.find("lanes/laneSection/left/lane") is None and r.el.find("lanes/laneSection/right/lane") is None:
                        continue  # no surface: a junction boundary helper road (OpenDRIVE 1.8 §12.10) has no lanes
                    s, t, h = project_many(r, px[miss], py[miss])
                    lo, hi = extent_many(r, s)
                    out = np.maximum(lo - t, 0) + np.maximum(t - hi, 0)
                    hit = (s > 1e-6) & (s < r.length - 1e-6) & (out <= MAP_TOL)
                    for q in np.nonzero(hit & ~cov)[0]:
                        cover[miss[q]] = r.junction if r.junction != "-1" else "road:" + r.id
                    cov |= hit
                    if cov.all():
                        break
                dropped = not cov.all()
            self.seg_cover.append(cover)
            self.seg_samples.append(best)
            self.seg_seq.append(seq)
            self.seg_dropped.append(dropped)
            self.seg_unmapped.append(not seq)
            ow = pc["oneway"]
            for rid, sd, s, x, y in seq:
                allowed = set()
                if ow >= 0:
                    allowed.add(sd)
                if ow <= 0:
                    allowed.add(-sd)
                self.road_osm[rid].append((s, frozenset(allowed), pi))
                self.road_sd[rid].append((s, sd, pi))

    def state_seq(self, st):
        """[(road, d)] compressed, in the state's travel direction."""
        seq = self.seg_seq[st >> 1]
        if st & 1:
            items = [(r, -sd) for r, sd, *_ in reversed(seq)]
        else:
            items = [(r, sd) for r, sd, *_ in seq]
        out = []
        for it in items:
            if not out or out[-1] != it:
                out.append(it)
        return out


# ----------------------------------------------------------------------------- the checks
class LaneFlow:
    def __init__(self, xodr, osm_path, box=None, rule=None, box_source=None):
        t0 = time.time()
        self.F = Findings()
        self.net = Network(xodr, rule)
        od = self.net.od
        self.box_source = box_source or "given"
        if box is None:
            h = od.header
            box = (float(h.get("west")), float(h.get("south")), float(h.get("east")), float(h.get("north")))
            self.box_source = "header extents (fallback)"
        self.box = box
        self.osm = G.Osm(osm_path, od.lat0, od.lon0) if osm_path else None
        extra = set()
        if self.osm is not None:
            for r in od.roads.values():
                for w in r.osm_ways:
                    if w in self.osm.ways and self.osm.ways[w][1].get("service") in EXCLUDE_SERVICE:
                        extra.add(w)
            self.on = OsmNet(self.osm, box, frozenset(extra))
            self.map = Mapping(self.net, self.on)
        else:
            self.on = self.map = None
        self.timing = {"load": round(time.time() - t0, 1)}
        self._road_ends()
        self._boundary_junctions()

    def _boundary_junctions(self):
        """junctions at the network edge: an OSM edge node (cut point, or a junction node kept just
        outside the box) whose way runs into a junction before reaching any plain road. Every road end
        linked to such a junction is an edge entry / exit (traffic comes from / leaves to outside)."""
        self.bjunc = {}
        on, mp, net = self.on, self.map, self.net
        if on is None:
            return
        for b in sorted(on.boundary, key=str):
            bx, by = on.pos[b]
            for g0 in on.at[b]:
                # walk the way inward from the edge node up to its first sample on a plain road
                pi, i0 = on.seg[g0][0], on.seg[g0][1]
                step = 1 if on.seg[g0][2] == b else -1
                nseg = len(on.pieces[pi]["nodes"]) - 1
                base = g0 - i0  # segments of a piece are consecutive
                walked, found = 0.0, None
                i = i0
                while 0 <= i < nseg and walked <= KEEP_OUTSIDE + ABSORB_JUNCTION and found is None:
                    g = base + i
                    smp = mp.seg_samples[g]
                    order = range(len(smp)) if step > 0 else range(len(smp) - 1, -1, -1)
                    for q in order:
                        if smp[q] is not None:
                            found = smp[q]
                            break
                        c = mp.seg_cover[g].get(q)
                        if c is not None and not c.startswith("road:"):
                            found = ("junction", c)
                            break
                    walked += on.seg[g][4]
                    i += step
                if found is None:
                    continue
                if found[0] == "junction":
                    self.bjunc.setdefault(found[1], b)
                    continue
                rid, sd, s, px, py = found
                r = net.roads[rid]
                at_end = s > r.length / 2
                L = r.succ if at_end else r.pred
                if L is not None and L.type == "junction":
                    self.bjunc.setdefault(L.id, b)
        for rid, r in net.roads.items():
            for at_end, L in ((True, r.succ), (False, r.pred)):
                if L is not None and L.type == "junction" and L.id in self.bjunc:
                    self.ends[(rid, at_end)]["bjunc"] = self.bjunc[L.id]

    # ---- road ends: network edge / OSM node
    def _road_ends(self):
        """per (road, end): boundary flag, end cross-section, nearest OSM node on the road's ways."""
        self.ends = {}
        on = self.on
        grid = collections.defaultdict(list)
        if on is not None:
            for nd in on.at:
                x, y = on.pos[nd]
                grid[(int(x // 20), int(y // 20))].append(nd)
        for rid, r in self.net.roads.items():
            for at_end in (False, True):
                s = r.length if at_end else 0.0
                sec = r.sections[-1] if at_end else r.sections[0]
                b = r.borders_at(s, sec)
                ts = [v for pair in b.values() for v in pair]
                tlo, thi = min(ts), max(ts)
                (x1, x2), (y1, y2) = r.point(np.array([s, s]), np.array([tlo, thi]))
                xr, yr, hr = (float(v[0]) for v in r.ref(np.array([s])))
                din = min(inside_dist(self.box, x1, y1), inside_dist(self.box, x2, y2), inside_dist(self.box, xr, yr))
                e = {"x": xr, "y": yr, "h": hr, "half": max(abs(tlo), abs(thi)), "inside": din,
                     "boundary": din <= ROAD_END_BAND, "osm_node": None, "osm_dist": None, "dead_end": False}
                if on is not None and r.junction == "-1":
                    rad = e["half"] + MAP_TOL
                    best, bd = None, None
                    for i in range(int((xr - rad) // 20), int((xr + rad) // 20) + 1):
                        for j in range(int((yr - rad) // 20), int((yr + rad) // 20) + 1):
                            for nd in grid.get((i, j), ()):
                                dd = math.hypot(on.pos[nd][0] - xr, on.pos[nd][1] - yr)
                                if dd <= rad and (bd is None or dd < bd):
                                    best, bd = nd, dd
                    e["osm_node"], e["osm_dist"] = best, bd
                    e["dead_end"] = best is not None and on.dead_end_node(best)
                    if best is not None and best in on.boundary:
                        e["boundary_osm"] = True
                self.ends[(rid, at_end)] = e

    # ---- main
    def run(self):
        t = time.time()
        F = self.F
        for name, sev in tuple((f"motor.{side}.{cls}", "defect") for side in ("dead_end", "orphan")
                               for cls in ("unlinked_road_end", "section_unlinked", "no_osm_node")) + \
                tuple((f"motor.dead_end.{cls}", "defect") for cls in ("road_link_without_lane_link",
                                                                      "junction_no_connection",
                                                                      "connecting_road_unlinked")) + \
                (("motor.dead_end.junction_lane_without_connection", "defect"), ("pairs.exit_unmapped", "defect"),
                 ("pairs.entry_unmapped", "defect"), ("edge.unlinked", "defect"),
                 ("motor.flow.unreachable", "defect"), ("motor.flow.trapped", "defect"),
                          ("motor.trap_region", "defect"), ("motor.island.missing_link", "defect"),
                          ("motor.link.direction", "defect"), ("osm.direction.wrong_way", "defect"),
                          ("osm.direction.missing", "defect"), ("osm.flow.lost", "defect"),
                          ("turn.missing", "defect"), ("pairs.missing", "defect"), ("rule.osm", "defect"),
                          ("unusable.zero_width_lane", "defect"), ("unusable.zero_length_section", "defect"),
                          ("unusable.narrow_lane", "defect"), ("unusable.section_without_motor_lane", "defect"),
                          ("edge.break.unexplained", "defect"), ("split.road_needless", "defect"),
                          ("split.road_to_road", "defect"), ("link.lane_asymmetric", "defect"),
                          ("split.section_redundant", "defect"),
                          ("link.junction_asymmetric", "defect"),
                          ("osm.way_dropped", "defect")):
            F.touch(name, sev)
        self.rule_check()
        self.motor = LaneGraph(self.net, MOTOR_TYPES, F, "motor.")
        self.bike = LaneGraph(self.net, BIKE_TYPES, F, "bike.")
        self.link_symmetry()
        self.junction_link_directions()
        self.lane_link_directions()
        if self.on is not None:
            self.osm_transitions()
        self.add_uturns(self.motor)
        self.ends_check(self.motor, "motor.")
        self.ends_check(self.bike, "bike.")
        self.reach(self.motor, "motor.")
        self.reach(self.bike, "bike.")
        self.usable()
        if self.on is not None:
            self.osm_direction()
            self.osm_flow()
            self.turns()
            self.pairs()
            self.osm_coverage()
        self.edge_continuity()
        self.splits()
        self.timing["checks"] = round(time.time() - t, 1)
        return F

    # ---- traffic rule vs OSM
    def rule_check(self):
        """declared rule vs OSM oneway evidence: on a oneway OSM way the lanes must travel with it."""
        if self.on is None:
            return
        votes = collections.Counter()
        net = self.net
        for rid, samples in self.map.road_osm.items():
            r = net.roads[rid]
            for s, allowed, pi in samples[::5]:
                if len(allowed) != 1:
                    continue
                d = next(iter(allowed))
                sec = r.section_at(s)
                k = r.sections.index(sec)
                for l in sec.lanes():
                    if l.type in MOTOR_TYPES and l.type != "bidirectional":
                        dr = lane_dirs(l.type, l.id, "RHT", net.lane_attr.get((rid, k, l.id)))[0]
                        votes["RHT" if dr == d else "LHT"] += 1
        decl = collections.Counter(net.rule_declared.values())
        declared = decl.most_common(1)[0][0] if decl else "RHT"
        tot = sum(votes.values())
        self.rule_evidence = {"declared": dict(decl), "osm_votes": dict(votes)}
        if tot:
            want = votes.most_common(1)[0][0]
            frac = votes[want] / tot
            if want != declared and frac > 0.5:
                self.F.add("rule.osm", "defect", None, None, None, None, None, value=frac, limit=0.5,
                           detail=f"file declares {declared} (road@rule absent = RHT) but {votes[want]}/{tot} lane "
                                  f"samples on OSM oneway ways only travel the OSM direction under {want}: every "
                                  f"reader drives this network the wrong way round")

    # ---- link symmetry
    def link_symmetry(self):
        F, net = self.F, self.net
        for rid in sorted(net.roads, key=_rid_key):
            r = net.roads[rid]
            for at_end, L in ((True, r.succ), (False, r.pred)):
                if L is None or L.type != "road":
                    continue
                e = self.ends[(rid, at_end)]
                T = net.roads.get(L.id)
                if T is None:
                    F.add("link.road_missing_target", "defect", rid, None, r.length if at_end else 0, e["x"], e["y"],
                          detail=f"links road {L.id} which does not exist")
                    continue
                back = T.pred if L.contact == "start" else T.succ
                if r.junction == "-1" and T.junction != "-1":
                    F.add("link.plain_to_connecting", "defect", rid, None, r.length if at_end else 0, e["x"], e["y"],
                          detail=f"plain road links connecting road {T.id} directly (must link the junction)")
                    continue
                if T.junction != "-1" or r.junction != "-1":
                    continue  # connecting roads link out; their counterpart is the junction
                ok = back is not None and back.type == "road" and back.id == rid and \
                    back.contact == ("end" if at_end else "start")
                if not ok:
                    F.add("link.road_asymmetric", "defect", rid, None, r.length if at_end else 0, e["x"], e["y"],
                          detail=f"{'successor' if at_end else 'predecessor'} road {T.id}/{L.contact} does not link back")

    # ---- (f) junction laneLinks must run with the traffic
    def junction_link_directions(self):
        """every connection laneLink: 'from' must travel into the junction on the incoming road, 'to'
        must travel away from the contact point on the connecting road (under the traffic rule)."""
        F, net = self.F, self.net
        not_linked = set()
        bad_ll = set()  # (junction, connecting road, to-lane) of wrong-way laneLinks
        for jid in sorted(net.junctions, key=_rid_key):
            for c in net.junctions[jid]["conns"]:
                R, C = net.roads.get(c["incoming"]), net.roads.get(c["connecting"] or c["linked"])
                if R is None or C is None:
                    F.add("link.junction_bad_reference", "defect", c["incoming"], None, None, None, None,
                          detail=f"junction {jid}: incoming {c['incoming']} / connecting {c['connecting']} missing")
                    continue
                ends = [e for e, L in ((True, R.succ), (False, R.pred)) if L is not None and L.type == "junction"
                        and L.id == jid]
                if not ends:
                    if (jid, R.id) not in not_linked:  # once per (junction, road), not per connection
                        not_linked.add((jid, R.id))
                        e = self.ends[(R.id, True)]
                        F.add("link.junction_not_linked", "defect", R.id, None, None, e["x"], e["y"],
                              detail=f"junction {jid} lists road {R.id} as incoming, the road does not link it")
                    continue
                kc, dc = (0, 1) if c["contact"] == "start" else (len(C.sections) - 1, -1)
                for f, to in c["lanes"]:
                    ok_from = False
                    for at_end in ends:
                        k = len(R.sections) - 1 if at_end else 0
                        lf_ = R.sections[k].lane(f)
                        if lf_ is not None and (1 if at_end else -1) in net.dirs(R.id, k, lf_):
                            ok_from = True
                    lt = C.sections[kc].lane(to)
                    if lt is not None and lt.type in EDGE_TYPES:
                        continue  # a pedestrian link (sidewalk round a corner): no direction of travel
                    ok_to = lt is not None and dc in net.dirs(C.id, kc, lt)
                    if lt is None or not any(R.sections[len(R.sections) - 1 if ae else 0].lane(f) is not None
                                             for ae in ends):
                        e = self.ends[(R.id, ends[0])]
                        F.add("link.junction_bad_reference", "defect", R.id, f, R.length if ends[0] else 0.0,
                              e["x"], e["y"], detail=f"junction {jid}: laneLink {f}->{to} into {C.id}: "
                                                     f"{'to-lane' if lt is None else 'from-lane'} does not exist")
                        continue
                    if not (ok_from and ok_to):
                        bad_ll.add((jid, C.id, to))
                        e = self.ends[(R.id, ends[0])]
                        F.add("motor.link.direction", "defect", R.id, f, R.length if ends[0] else 0.0, e["x"], e["y"],
                              detail=f"junction {jid}: laneLink {f}->{to} into {C.id}/{c['contact']}: "
                                     f"{'from-lane does not travel into the junction' if not ok_from else ''}"
                                     f"{' and ' if not ok_from and not ok_to else ''}"
                                     f"{'to-lane does not travel away from the contact' if not ok_to else ''}")
        # spec FLOW-0 / FLOW-2: entry into a connecting road comes only from a laneLink; a connecting
        # lane whose own record at its entry end names an incoming lane with no laneLink into it is
        # FLOW-2/asymmetric (the record is one-sided)
        self.junction_one_sided = set()  # (connecting road, lane) whose feed is declared one-sidedly
        links = collections.defaultdict(set)  # (junction, incoming, connecting) -> {(from, to)}
        for jid, j in net.junctions.items():
            for c in j["conns"]:
                links[(jid, c["incoming"], c["connecting"] or c["linked"])].update(c["lanes"])
        for cid in sorted(net.roads, key=_rid_key):
            C = net.roads[cid]
            if C.junction == "-1":
                continue
            for at_start in (True, False):
                L = C.pred if at_start else C.succ
                if L is None or L.type != "road" or L.id not in net.roads:
                    continue
                k = 0 if at_start else len(C.sections) - 1
                for l in C.sections[k].lanes():
                    if l.type not in MOTOR_TYPES | BIKE_TYPES:
                        continue
                    rec = l.pred if at_start else l.succ
                    if rec is None or (1 if at_start else -1) not in net.dirs(cid, k, l):
                        continue  # not an entry end of this lane
                    if (rec, l.id) in links.get((C.junction, L.id, cid), ()):
                        continue
                    if (C.junction, cid, l.id) in bad_ll:
                        continue  # a laneLink into this lane exists but runs the wrong way: link.direction
                    Rr = net.roads[L.id]
                    kr = len(Rr.sections) - 1 if L.contact == "end" else 0
                    lr = Rr.sections[kr].lane(rec)
                    if lr is None or (1 if L.contact == "end" else -1) not in net.dirs(Rr.id, kr, lr):
                        continue  # the record names a missing / wrong-way lane: link.direction / dangling
                    self.junction_one_sided.add((cid, l.id))
                    x, y, _ = lane_centre(C, C.sections[k], l.id, 0.0 if at_start else C.length)
                    F.add("link.junction_asymmetric", "defect", cid, l.id, 0.0 if at_start else C.length, x, y,
                          junction=C.junction,
                          detail=f"connecting road {cid} lane {l.id} names road {L.id} lane {rec} as its "
                                 f"{'predecessor' if at_start else 'successor'}, but junction {C.junction} has "
                                 f"no laneLink {rec}->{l.id} from road {L.id} into it")

    # ---- (f) lane predecessor / successor records must run with the traffic, once per record
    def lane_link_directions(self):
        """every lane predecessor / successor record of a driven lane (motor or bike family), inside a
        road (section to section) and across a road link: the linked lane must travel on in the same
        sense. Crossing a successor into a road's start (or a predecessor into a road's end) keeps the
        sense along s (flip +1); successor -> end or predecessor -> start reverses it (flip -1). The
        record is consistent if some travel direction d of the lane has d * flip among the travel
        directions of the linked lane. Junction laneLinks are checked by junction_link_directions.
        Also reciprocity of driven lanes (spec FLOW-2/asymmetric, MUST): a record whose counterpart
        is missing or names another lane; the edge itself exists either way (FLOW-0 declaration)."""
        F, net = self.F, self.net
        driven = MOTOR_TYPES | BIKE_TYPES
        asym = set()
        for rid in sorted(net.roads, key=_rid_key):
            r = net.roads[rid]
            n = len(r.sections)
            for k, sec in enumerate(r.sections):
                for l in sec.lanes():
                    for attr in ("succ", "pred"):
                        tid = getattr(l, attr)
                        if tid is None:
                            continue
                        inner = (attr == "succ" and k + 1 < n) or (attr == "pred" and k > 0)
                        if inner:
                            T, k2, flip, back = r, k + (1 if attr == "succ" else -1), 1, \
                                ("pred" if attr == "succ" else "succ")
                        else:
                            L = r.succ if attr == "succ" else r.pred
                            if L is None or L.type != "road" or L.id not in net.roads:
                                continue  # lane links toward a junction carry no meaning (laneLinks do)
                            T = net.roads[L.id]
                            k2 = 0 if L.contact == "start" else len(T.sections) - 1
                            flip = 1 if (attr == "succ") == (L.contact == "start") else -1
                            back = "pred" if L.contact == "start" else "succ"
                        l2 = T.sections[k2].lane(tid)
                        s_at = sec.e if attr == "succ" else sec.s
                        if l2 is None:
                            continue  # reported as link.bad_reference by the traversal
                        # reciprocity (plain roads and sections; a connecting road's counterpart is the junction)
                        if (inner or (r.junction == "-1" and T.junction == "-1")) and getattr(l2, back) != l.id:
                            key = tuple(sorted([(rid, k, l.id), (T.id, k2, l2.id)]))
                            if key not in asym and l.type in driven and l2.type in driven:
                                asym.add(key)
                                x, y, _ = lane_centre(r, sec, l.id, s_at)
                                F.add("link.lane_asymmetric", "defect", rid, l.id, s_at, x, y, section=k,
                                      detail=f"lane {l.id} {attr} -> road {T.id} lane {l2.id}, which "
                                             f"{'has no ' + back if getattr(l2, back) is None else back + ' ' + str(getattr(l2, back))}")
                        if l.type not in driven or l2.type not in driven:
                            continue
                        d1, d2 = net.dirs(rid, k, l), net.dirs(T.id, k2, l2)
                        if any(d * flip in d2 for d in d1):
                            continue
                        x, y, _ = lane_centre(r, sec, l.id, s_at)
                        p = "bike." if l.type in BIKE_TYPES else "motor."
                        F.add(p + "link.direction", "defect", rid, l.id, s_at, x, y, value=d1[0], limit=d1[0] * flip,
                              section=k, detail=f"{'section' if inner else 'road'} {attr}: road {rid} lane {l.id} "
                                                f"(travel {'/'.join(f'{q:+d}' for q in d1)}) -> road {T.id} lane "
                                                f"{l2.id} (travel {'/'.join(f'{q:+d}' for q in d2)}), must travel "
                                                f"{d1[0] * flip:+d}")

    # ---- U-turns at mapped dead ends (both models turn round there)
    def add_uturns(self, g):
        self.uturn_ends = set()
        for (rid, at_end), e in self.ends.items():
            r = self.net.roads[rid]
            link = r.succ if at_end else r.pred
            if link is not None or not e["dead_end"] or e["boundary"]:
                continue
            k = len(r.sections) - 1 if at_end else 0
            arrive, leave = [], []
            for l in r.sections[k].lanes():
                for d in (1, -1):
                    i = g.idx.get((rid, k, l.id, d))
                    if i is None:
                        continue
                    (arrive if (d > 0) == at_end else leave).append(i)
            for i in arrive:
                for j in leave:
                    g._edge(i, j, "uturn")
            if arrive and leave:
                self.uturn_ends.add((rid, at_end))

    # ---- (a) dead ends and (b) orphans
    def ends_check(self, g, p):
        F, net = self.F, self.net
        T_out = getattr(self, "osm_out", {})
        T_in = getattr(self, "osm_in", {})
        groups = {}
        for i, (rid, k, lid, d) in enumerate(g.key):
            r, sec, lane = g.lane(i)
            for side in ("exit", "entry"):
                has = g.has_long_out(i) if side == "exit" else g.has_long_in(i)
                if has:
                    continue
                if side == "exit" and any(kd == "uturn" for _, kd in g.out[i]):
                    continue
                if side == "entry" and any(kd == "uturn" for _, kd in g.inn[i]):
                    continue
                s = g.exit_s(i) if side == "exit" else g.entry_s(i)
                at_road = g.at_road_exit(i) if side == "exit" else g.at_road_entry(i)
                at_end = (d > 0) == (side == "exit")  # which road end the lane meets
                w = lane_width(sec, lane, s if s < sec.e else sec.e)
                x, y, _ = lane_centre(r, sec, lid, s)
                cls, sev = self._classify_end(g, i, side, at_road, at_end, w, T_out if side == "exit" else T_in)
                name = (p + ("dead_end." if side == "exit" else "orphan.") + cls)
                if at_road and (r.succ if at_end else r.pred) is None:
                    # an end with no road link affects every lane there: one instance per road end
                    # (spec 0.3), its lanes listed in value
                    grp = groups.setdefault((name, rid, at_end), {"sev": sev, "lanes": [], "w": []})
                    grp["lanes"].append(lid)
                    grp["w"].append(w)
                    continue
                F.add(name, sev, rid, lid, s, x, y, value=w, limit=WIDTH_ZERO, section=k, dir=d,
                      junction=r.junction if r.junction != "-1" else None, lane_type=lane.type)
        for (name, rid, at_end), grp in sorted(groups.items(), key=lambda kv: (kv[0][0], _rid_key(kv[0][1]),
                                                                              kv[0][2])):
            e = self.ends[(rid, at_end)]
            r = net.roads[rid]
            F.add(name, grp["sev"], rid, None, r.length if at_end else 0.0, e["x"], e["y"], value=sorted(grp["lanes"]),
                  limit=None, lanes=sorted(grp["lanes"]), max_width=max(grp["w"]), end="end" if at_end else "start",
                  junction=r.junction if r.junction != "-1" else None, inside_box=e["inside"],
                  osm_node=e.get("osm_node"))

    def _classify_end(self, g, i, side, at_road, at_end, w, T):
        net = self.net
        rid, k, lid, d = g.key[i]
        r = net.roads[rid]
        lane = r.sections[k].lane(lid)
        bike = g.types is BIKE_TYPES
        lat = g.has_lat(i) if side == "exit" else g.has_lat_in(i)  # leave / enter by a lane change
        if w <= WIDTH_ZERO and lat:
            return ("taper_merge" if side == "exit" else "taper_diverge"), "ok"
        if w <= WIDTH_ZERO and not bike:
            # spec FLOW-0 ends (iii): a zero-width end is neither source nor sink; with no lane change
            # into / out of it the lane is unusable, which FLOW-4 (flow.* / island) reports
            return ("taper_no_lane_change_out" if side == "exit" else "taper_no_lane_change_in"), "info"
        if bike and w <= WIDTH_ZERO:
            return ("lane_tapers_out" if side == "exit" else "lane_tapers_in"), "info"
        if not at_road:
            ahead = (d > 0) == (side == "exit")  # the neighbouring section lies at larger s
            nxt = r.sections[k + (1 if ahead else -1)]
            if not bike and self.map is not None:
                # OSM itself ends / starts this direction there (oneway <-> two-way change)
                sb = nxt.s if ahead else nxt.e
                lo_, hi_ = (nxt.s, nxt.e) if ahead else (nxt.s, nxt.e)
                beyond = [a for s_, a, _ in self.map.road_osm.get(rid, ()) if lo_ - 1e-6 <= s_ <= hi_ + 1e-6]
                if not beyond:
                    beyond = [a for s_, a, _ in self.map.road_osm.get(rid, ())
                              if (sb <= s_ <= sb + 10 if ahead else sb - 10 <= s_ <= sb)]
                if beyond and any(d not in a for a in beyond):
                    return ("osm_direction_ends" if side == "exit" else "osm_direction_starts"), "ok"
            if side == "entry" and self._one_sided(g, i, nxt, rid, k + (1 if ahead else -1)):
                # this lane's own record points back at a lane whose forward record is missing: that
                # lane's dead end is the defect (one root cause, reported once)
                return "section_link_one_sided", "info"
            same = nxt.lane(lid) is not None and nxt.lane(lid).type in g.types
            return ("section_unlinked_same_id" if same else "section_unlinked"), "defect"
        e = self.ends[(rid, at_end)]
        if e["boundary"]:
            return "boundary", "ok"
        if e.get("bjunc"):
            return "boundary_junction", "ok"
        link = r.succ if at_end else r.pred
        # spec FLOW-1 / JCT-1: an exit end with no lane link is MUST (defect); an entry end with none is
        # SHOULD ('entry-no-lane-link', reported as info here), because whether the lane is usable at
        # all is decided by FLOW-4 (flow.unreachable / island), which reports it as a defect
        entry_sev = "info" if side == "entry" else "defect"
        if r.junction != "-1":
            if side == "entry" and (rid, lid) in getattr(self, "junction_one_sided", ()):
                return "connecting_road_link_one_sided", "info"  # the defect is link.junction_asymmetric
            return "connecting_road_unlinked", entry_sev
        if bike:
            if link is not None and link.type == "junction":
                return "at_junction", "info"  # cyclists continue in the junction's general traffic
            if link is None:
                return ("mapped_dead_end" if e["dead_end"] else "road_end"), "info"
            return "road_link_without_lane_link", "defect"
        osm_moves = T.get((rid, d)) if T is not None else None
        if link is None:
            if e["dead_end"]:
                return "mapped_dead_end", "ok"
            if self.on is not None and e["osm_node"] is None:
                return "no_osm_node", "defect"
            if self.on is not None and not osm_moves:
                return "osm_no_move", "info"
            return "unlinked_road_end", "defect"
        if link.type == "road":
            return "road_link_without_lane_link", entry_sev
        # junction
        if self.on is not None and not osm_moves:
            return "junction_osm_no_move", "info"
        if self.on is not None:
            # lanes that are not the junction's movement lane but whose neighbours connect
            if lat:
                # a lane (width > WIDTH_ZERO; zero-width ends are tapers, classified above) that reaches
                # the junction with no movement at all drops every car that follows it to the stop
                # line: defect (spec FLOW-1/exit-no-lane-link, MUST). A lane leaving the junction that
                # no movement feeds is entered by a lane change behind it: info (entry-no-lane-link,
                # SHOULD; FLOW-4 decides whether it is usable).
                if side == "exit":
                    return "junction_lane_without_connection", "defect"
                return "junction_lane_without_connection", "info"
        return "junction_no_connection", entry_sev

    def _one_sided(self, g, i, sec2, rid2, k2):
        """orphan lane i whose own backward record names a lane of sec2 (road rid2, section k2) that
        travels the same way and has no longitudinal way on: the missing record is that lane's."""
        rid, k, lid, d = g.key[i]
        lane = self.net.roads[rid].sections[k].lane(lid)
        rec = lane.pred if d > 0 else lane.succ
        if rec is None or sec2.lane(rec) is None:
            return False
        j = g.idx.get((rid2, k2, rec, d))  # same road: the predecessor lane travels the same way
        return j is not None and not g.has_long_out(j)

    # ---- (c) (d) (e) reachability, traps, islands
    def reach(self, g, p):
        F, net = self.F, self.net
        n = len(g.key)
        if n == 0:
            return
        succ = [[j for j, _ in g.out[i]] for i in range(n)]
        pred = [[j for j, _ in g.inn[i]] for i in range(n)]
        entries, exits = [], []
        for i, (rid, k, lid, d) in enumerate(g.key):
            if g.at_road_entry(i):
                e = self.ends[(rid, d < 0)]
                if (e["boundary"] and not g.has_long_in(i)) or e.get("bjunc"):
                    entries.append(i)
            if g.at_road_exit(i):
                e = self.ends[(rid, d > 0)]
                if (e["boundary"] and not g.has_long_out(i)) or e.get("bjunc"):
                    exits.append(i)
        fwd = _bfs(entries, succ, n)
        bwd = _bfs(exits, pred, n)
        g.fwd, g.bwd, g.entries, g.exits = fwd, bwd, entries, exits
        # weak components
        comp = [-1] * n
        nc = 0
        for s0 in range(n):
            if comp[s0] != -1:
                continue
            st = [s0]
            comp[s0] = nc
            while st:
                u = st.pop()
                for v in succ[u] + pred[u]:
                    if comp[v] == -1:
                        comp[v] = nc
                        st.append(v)
            nc += 1
        touch = [False] * nc
        for i in entries + exits:
            touch[comp[i]] = True
        g.comp = comp
        # islands (tested on their own)
        members = collections.defaultdict(list)
        for i in range(n):
            if not touch[comp[i]]:
                members[comp[i]].append(i)
        island_nodes = set()
        for c, nodes in sorted(members.items(), key=lambda kv: g.key[kv[1][0]]):
            island_nodes.update(nodes)
            roads = sorted({g.key[i][0] for i in nodes}, key=_rid_key)
            sub = {v: q for q, v in enumerate(nodes)}
            _, nscc = _scc(len(nodes), [[sub[j] for j in succ[v] if j in sub] for v in nodes])
            plain = [rd for rd in roads if net.is_plain(rd)]
            osm_conn = None
            if self.on is not None and p == "motor.":
                osm_conn = any(self.osm_ok.get((rd, d), False) for rd in plain for d in (1, -1))
            x, y = self._road_xy(roads[0])
            if p != "motor.":
                name, sev = p + "island", "info"
            elif osm_conn:
                name, sev = p + "island.missing_link", "defect"
            elif self.on is not None and not plain:
                name, sev = p + "island.connecting_only", "defect"
            else:
                name, sev = p + "island.osm_isolated", "ok"
            F.add(name, sev, roads[0], None, None, x, y, value=len(nodes), limit=0, roads=roads[:50],
                  scc=nscc, strongly_connected=nscc == 1,
                  detail="lanes not touching the map edge; OSM connects these ways to the edge" if osm_conn else
                  "lanes not touching the map edge")
        # unreachable / trapped (outside islands), grouped into regions
        for name, bad in ((p + "flow.unreachable", ~fwd), (p + "flow.trapped", ~bwd)):
            nodes = [i for i in range(n) if bad[i] and i not in island_nodes]
            self._regions(g, nodes, succ, pred, name, p)
        # (d) trap regions: sink SCCs with more than one lane and no exit to the edge
        cid, ncomp = _scc(n, succ)
        has_out = [False] * ncomp
        has_exit = [False] * ncomp
        size = collections.Counter(cid)
        for i in range(n):
            for j in succ[i]:
                if cid[j] != cid[i]:
                    has_out[cid[i]] = True
        for i in exits:
            has_exit[cid[i]] = True
        byc = collections.defaultdict(list)
        for i in range(n):
            if i not in island_nodes and not has_out[cid[i]] and not has_exit[cid[i]] and size[cid[i]] > 1:
                byc[cid[i]].append(i)
        for c, nodes in sorted(byc.items(), key=lambda kv: g.key[kv[1][0]]):
            roads = sorted({g.key[i][0] for i in nodes}, key=_rid_key)
            x, y = self._road_xy(roads[0])
            F.add(p + "trap_region", "defect", roads[0], None, None, x, y, value=len(nodes), limit=1,
                  roads=roads[:50], detail="strongly connected lanes with no way out to the map edge")
        g.cid, g.ncomp = cid, ncomp

    def _regions(self, g, nodes, succ, pred, name, p):
        F, net = self.F, self.net
        S = set(nodes)
        seen = set()
        for s0 in nodes:
            if s0 in seen:
                continue
            reg, st = [], [s0]
            seen.add(s0)
            while st:
                u = st.pop()
                reg.append(u)
                for v in succ[u] + pred[u]:
                    if v in S and v not in seen:
                        seen.add(v)
                        st.append(v)
            roads = sorted({g.key[i][0] for i in reg}, key=_rid_key)
            plain = [rd for rd in roads if net.is_plain(rd)]
            osm = None
            if self.on is not None and p == "motor.":
                key = "osm_fwd" if name.endswith("unreachable") else "osm_bwd"
                osm = any(getattr(self, key).get((g.key[i][0], g.key[i][3]), False) for i in reg)
            # representative (spec FLOW-4): a lane of the region whose end has no longitudinal edge
            # (no way in for an unreachable region, no way on for a trapped one), lowest numeric road id
            ends_ = [i for i in reg if not (g.has_long_in(i) if name.endswith("unreachable") else g.has_long_out(i))]
            i0 = min(ends_ or reg, key=lambda i: (_rid_key(g.key[i][0]), g.key[i][1], g.key[i][2], g.key[i][3]))
            r, sec, lane = g.lane(i0)
            x, y, _ = lane_centre(r, sec, lane.id, (sec.s + sec.e) / 2)
            sev = "defect"
            nm = name
            if p == "motor." and osm is False:
                nm, sev = name + ".osm_same", "ok"   # OSM cannot do it either: not a converter defect
            elif p == "bike.":
                sev = "info"
            F.add(nm, sev, g.key[i0][0], g.key[i0][2], (sec.s + sec.e) / 2, x, y, value=len(reg), limit=0,
                  roads=roads[:40], plain_roads=len(plain), osm_can=osm)

    def _road_xy(self, rid):
        r = self.net.roads[rid]
        x, y, _ = r.ref(np.array([r.length / 2]))
        return float(x[0]), float(y[0])

    # ---- (g) unusable lanes
    def usable(self):
        F, net = self.F, self.net
        g = self.motor
        for rid in sorted(net.roads, key=_rid_key):
            r = net.roads[rid]
            for k, sec in enumerate(r.sections):
                motor = [l for l in sec.lanes() if l.type in MOTOR_TYPES]
                if sec.e - sec.s <= ZERO_LEN and sec.lanes():
                    x, y, _ = r.ref(np.array([sec.s]))
                    F.add("unusable.zero_length_section", "defect", rid, None, sec.s, float(x[0]), float(y[0]),
                          value=sec.e - sec.s, limit=ZERO_LEN, section=k)
                    continue
                if not motor and r.junction == "-1":
                    x, y, _ = r.ref(np.array([(sec.s + sec.e) / 2]))
                    F.add("unusable.section_without_motor_lane", "defect", rid, None, sec.s, float(x[0]), float(y[0]),
                          section=k, detail="section of a motor road carries no motor lane")
                for l in motor:
                    wm = lane_max_width(sec, l)
                    if wm <= WIDTH_ZERO:
                        x, y, _ = lane_centre(r, sec, l.id, (sec.s + sec.e) / 2)
                        F.add("unusable.zero_width_lane", "defect", rid, l.id, sec.s, x, y, value=float(wm),
                              limit=WIDTH_ZERO, section=k, length=sec.e - sec.s)
        # narrow: the lane's longitudinal chain inside its road never reaches a car's width
        done = set()
        for i, key in enumerate(g.key):
            if i in done:
                continue
            chain, st = [], [i]
            done.add(i)
            ext = False
            while st:
                u = st.pop()
                chain.append(u)
                for v, kd in g.out[u] + g.inn[u]:
                    if kd == "sec" and v not in done:
                        done.add(v)
                        st.append(v)
                    elif kd in ("road", "junc"):
                        ext = True
            wmax = 0.0
            for u in chain:
                r, sec, lane = g.lane(u)
                wmax = max(wmax, float(lane_max_width(sec, lane)))
            if wmax < MIN_VEHICLE_WIDTH and wmax > WIDTH_ZERO:
                u = min(chain, key=lambda q: g.key[q])
                r, sec, lane = g.lane(u)
                x, y, _ = lane_centre(r, sec, lane.id, (sec.s + sec.e) / 2)
                F.add("unusable.narrow_lane", "defect", g.key[u][0], g.key[u][2], sec.s, x, y, value=wmax,
                      limit=MIN_VEHICLE_WIDTH, sections=len(chain), linked_out=ext, lane_type=lane.type)

    # ---- (h) OSM direction / flow
    def osm_direction(self):
        """per section: the motor lane directions must equal the OSM directions on that stretch."""
        F, net, mp = self.F, self.net, self.map
        votes = collections.Counter()
        for rid in sorted(mp.road_osm, key=_rid_key):
            r = net.roads[rid]
            samples = mp.road_osm[rid]
            for k, sec in enumerate(r.sections):
                inside = [(s, a) for s, a, _ in samples if sec.s - 1e-6 <= s <= sec.e + 1e-6]
                if not inside:
                    continue
                osm_dirs = set().union(*[a for _, a in inside])
                lane_d = set()
                for l in sec.lanes():
                    if l.type in MOTOR_TYPES and lane_widths(sec, l).max() > WIDTH_ZERO:
                        lane_d.update(net.dirs(rid, k, l))
                sm = (sec.s + sec.e) / 2
                x, y, _ = r.ref(np.array([sm]))
                for d in sorted(lane_d - osm_dirs):
                    F.add("osm.direction.wrong_way", "defect", rid, None, sm, float(x[0]), float(y[0]), value=d,
                          limit=sorted(osm_dirs), section=k,
                          detail=f"lanes travel {d:+d} along s, OSM only allows {sorted(osm_dirs)}")
                for d in sorted(osm_dirs - lane_d):
                    F.add("osm.direction.missing", "defect", rid, None, sm, float(x[0]), float(y[0]), value=d,
                          limit=sorted(osm_dirs), section=k,
                          detail=f"OSM allows travel {d:+d} along s, no motor lane does")

    def osm_flow(self):
        """an (road, direction) OSM can drive edge-to-edge must be drivable edge-to-edge in lanes."""
        F, g = self.F, self.motor
        lane_ok = collections.defaultdict(lambda: [False, False])
        for i, (rid, k, lid, d) in enumerate(g.key):
            v = lane_ok[(rid, d)]
            v[0] |= bool(g.fwd[i])
            v[1] |= bool(g.bwd[i])
        for (rid, d) in sorted(self.osm_ok, key=lambda q: (_rid_key(q[0]), q[1])):
            if not self.osm_ok[(rid, d)] or (rid, d) not in lane_ok:
                continue
            f, b = lane_ok[(rid, d)]
            if f and b:
                continue
            x, y = self._road_xy(rid)
            why = ("not reachable from the map edge" if not f else "") + \
                  (" and " if not f and not b else "") + ("cannot reach the map edge" if not b else "")
            F.add("osm.flow.lost", "defect", rid, None, None, x, y, value=d, limit=None,
                  detail=f"OSM drives this road ({d:+d}) edge to edge, its lanes are {why}",
                  reachable=f, can_exit=b)

    # ---- road-level transitions OSM vs lanes
    def osm_transitions(self):
        on, mp = self.on, self.map
        n = len(on.valid)
        seqs = [mp.state_seq(st) if on.valid[st] else [] for st in range(n)]
        self.state_seqs = seqs
        T = {}
        self.osm_fwd, self.osm_bwd, self.osm_ok = {}, {}, {}
        for st in range(n):
            if not on.valid[st]:
                continue
            for item in seqs[st]:
                self.osm_fwd[item] = self.osm_fwd.get(item, False) or bool(on.fwd[st])
                self.osm_bwd[item] = self.osm_bwd.get(item, False) or bool(on.bwd[st])
                self.osm_ok[item] = self.osm_ok.get(item, False) or bool(on.fwd[st] and on.bwd[st])
        for st in range(n):
            sq = seqs[st]
            if not sq:
                continue
            for a, b in zip(sq, sq[1:]):
                if a[0] != b[0]:
                    T.setdefault((a, b), {"node": None, "dropped": False, "seg": st >> 1, "in_state": st})
            last = sq[-1]
            stack = [(st, 0.0, False)]
            seen = {st}
            while stack:
                u, dist, dropped = stack.pop()
                for v in on.succ[u]:
                    if v in seen:
                        continue
                    seen.add(v)
                    if seqs[v]:
                        first = seqs[v][0]
                        if first != last and first[0] != last[0]:
                            info = T.setdefault((last, first), {"node": on.s_from(v), "dropped": dropped,
                                                                "seg": v >> 1, "in_state": st})
                            info["dropped"] = info["dropped"] and dropped
                    else:
                        L = on.seg[v >> 1][4]
                        if dist + L <= ABSORB_MAX:
                            stack.append((v, dist + L, dropped or mp.seg_dropped[v >> 1]))
        self.T_osm = T
        self.osm_out = collections.defaultdict(list)
        self.osm_in = collections.defaultdict(list)
        for (a, b) in T:
            self.osm_out[a].append(b)
            self.osm_in[b].append(a)

    def lane_transitions(self):
        """road-level moves the lanes allow: plain (road, d) -> plain (road, d) through connecting roads."""
        g, net = self.motor, self.net
        T = {}
        for i, (rid, k, lid, d) in enumerate(g.key):
            if not net.is_plain(rid) or not g.at_road_exit(i):
                continue
            st = [j for j, kd in g.out[i] if kd in ("road", "junc")]
            seen = set(st)
            while st:
                u = st.pop()
                ru = g.key[u][0]
                if net.is_plain(ru):
                    if ru != rid:
                        T[((rid, d), (ru, g.key[u][3]))] = True
                    continue
                for v, kd in g.out[u]:
                    if v not in seen:
                        seen.add(v)
                        st.append(v)
        return T

    def turns(self):
        F, net, on = self.F, self.net, self.on
        Tx = self.lane_transitions()
        self.T_lane = Tx
        g = self.motor
        have = {(rid, d) for (rid, k, lid, d) in g.key}

        def move_angle(a, b):
            ra, rb = net.roads[a[0]], net.roads[b[0]]
            _, _, ha = ra.ref(np.array([ra.length if a[1] > 0 else 0.0]))
            _, _, hb = rb.ref(np.array([0.0 if b[1] > 0 else rb.length]))
            ha = float(ha[0]) + (0 if a[1] > 0 else math.pi)
            hb = float(hb[0]) + (0 if b[1] > 0 else math.pi)
            return math.degrees(_wrap(hb - ha))

        # spec JCT-2: turn classes by order among the OSM exits of the same approach
        exits = collections.defaultdict(list)
        for (a, b) in self.T_osm:
            if a in have and b in have:
                exits[a].append(move_angle(a, b))
        for (a, b), info in sorted(self.T_osm.items(), key=lambda kv: (_rid_key(kv[0][0][0]), kv[0][0][1],
                                                                        _rid_key(kv[0][1][0]), kv[0][1][1])):
            if (a, b) in Tx or a not in have or b not in have:
                continue
            ra = net.roads[a[0]]
            sa = ra.length if a[1] > 0 else 0.0
            ang = move_angle(a, b)
            xa, ya, _ = ra.ref(np.array([sa]))
            nd = info["node"]
            x, y = (on.pos[nd] if nd is not None else (float(xa[0]), float(ya[0])))
            lanes_tag = self._turn_lanes(info["in_state"])
            if abs(ang) > UTURN_DEG:
                name, sev = "turn.missing_uturn", "info"
            elif info["dropped"]:
                name, sev = "turn.missing_via_dropped_osm", "info"
            elif lanes_tag is not None and not _turn_permitted(lanes_tag, ang, exits[a]):
                name, sev = "turn.missing.turn_lanes_forbid", "ok"
            else:
                name, sev = "turn.missing", "defect"
            F.add(name, sev, a[0], None, sa, x, y, value=round(ang, 1), limit=UTURN_DEG,
                  frm=f"{a[0]}{a[1]:+d}", to=f"{b[0]}{b[1]:+d}", osm_node=nd,
                  link=_link_str(ra, a[1] > 0), turn_lanes=lanes_tag)
        # moves the lanes make that OSM does not (info; was unreachable code in the first version)
        for (a, b) in sorted(Tx, key=lambda q: (_rid_key(q[0][0]), q[0][1], _rid_key(q[1][0]), q[1][1])):
            if (a, b) in self.T_osm or a not in self.osm_ok or b not in self.osm_ok:
                continue
            ra = net.roads[a[0]]
            sa = ra.length if a[1] > 0 else 0.0
            x, y, _ = ra.ref(np.array([sa]))
            F.add("turn.extra", "info", a[0], None, sa, float(x[0]), float(y[0]), frm=f"{a[0]}{a[1]:+d}",
                  to=f"{b[0]}{b[1]:+d}", detail="lanes connect roads OSM does not connect in this direction")

    def _turn_lanes(self, st):
        """turn:lanes value that governs the OSM way the movement starts on, in its travel direction."""
        on = self.on
        pc = on.pieces[on.seg[st >> 1][0]]
        tg = pc["tags"]
        fwd = (st & 1) == 0
        if pc["oneway"] != 0:
            return tg.get("turn:lanes") or tg.get("turn")
        return tg.get("turn:lanes:forward" if fwd else "turn:lanes:backward")

    # ---- boundary pairs: every entry must reach every exit OSM reaches
    def pairs(self):
        F, on, g, net = self.F, self.on, self.motor, self.net
        bnodes = sorted(on.boundary, key=str)
        bidx = {b: q for q, b in enumerate(bnodes)}
        # OSM: reach bitsets over the condensation
        n = len(on.valid)
        cid, nc = _scc(n, on.succ)
        bits = [0] * nc
        for st in on.exits:
            bits[cid[st]] |= 1 << bidx[on.s_to(st)]
        order = collections.defaultdict(list)
        for st in range(n):
            order[cid[st]].append(st)
        for c in range(nc):  # Tarjan emits components in reverse topological order: successors first
            acc = bits[c]
            for st in order[c]:
                for v in on.succ[st]:
                    if cid[v] != c:
                        acc |= bits[cid[v]]
            bits[c] = acc
        osm_reach = collections.defaultdict(int)
        for st in on.entries:
            osm_reach[on.s_from(st)] |= bits[cid[st]]
        # lanes: boundary road ends -> OSM boundary node
        def end_node(rid, at_end):
            e = self.ends[(rid, at_end)]
            if e.get("bjunc"):
                return e["bjunc"]
            nd = e.get("osm_node")
            if nd in on.boundary:
                return nd
            best, bd = None, None
            for b in bnodes:
                x, y = on.pos[b]
                dd = math.hypot(x - e["x"], y - e["y"])
                if dd <= e["half"] + MAP_TOL and (bd is None or dd < bd):
                    best, bd = b, dd
            return best
        m = len(g.key)
        succ = [[j for j, _ in g.out[i]] for i in range(m)]
        lcid, lnc = _scc(m, succ)
        lbits = [0] * lnc
        exit_map = {}
        for i in g.exits:
            rid, k, lid, d = g.key[i]
            nd = end_node(rid, d > 0)
            exit_map[i] = nd
            if nd is not None:
                lbits[lcid[i]] |= 1 << bidx[nd]
        lorder = collections.defaultdict(list)
        for i in range(m):
            lorder[lcid[i]].append(i)
        for c in range(lnc):
            acc = lbits[c]
            for i in lorder[c]:
                for v in succ[i]:
                    if lcid[v] != c:
                        acc |= lbits[lcid[v]]
            lbits[c] = acc
        lane_reach = collections.defaultdict(int)
        mapped_entries = set()
        for i in g.entries:
            rid, k, lid, d = g.key[i]
            nd = end_node(rid, d < 0)
            if nd is not None:
                lane_reach[nd] |= lbits[lcid[i]]
                mapped_entries.add(nd)
        exits_mapped = 0
        for nd in exit_map.values():
            if nd is not None:
                exits_mapped |= 1 << bidx[nd]
        total_osm = total_lane = 0
        lost_by_exit = collections.defaultdict(list)
        for nd in sorted(osm_reach, key=str):
            o = osm_reach[nd] & exits_mapped
            total_osm += bin(o).count("1")
            if nd not in mapped_entries:
                if o:
                    x, y = on.pos[nd]
                    through = inside_dist(self.box, x, y) < 0 and len(on.at[nd]) >= 2 and nd not in on.synthetic
                    F.add("pairs.entry_unmapped" + (".road_passes_edge_node" if through else ""),
                          "info" if through else "defect", None, None, None, x, y, value=bin(o).count("1"),
                          limit=0, osm_node=nd,
                          detail="edge node outside the box that the road runs through (its other ways lie "
                                 "outside the clip)" if through else
                          "OSM enters the map here, no lane starts at this edge point")
                continue
            l = lane_reach[nd]
            total_lane += bin(o & l).count("1")
            miss = o & ~l
            for q in range(len(bnodes)):
                if miss >> q & 1:
                    lost_by_exit[bnodes[q]].append(nd)
        # the mirror of entry_unmapped: an edge exit OSM reaches where no lane leaves the map (pairs to
        # it are not in pairs.missing, whose exits are the mapped ones)
        reach_count = collections.Counter()
        for nd, o in osm_reach.items():
            for q in range(len(bnodes)):
                if o >> q & 1:
                    reach_count[q] += 1
        for q in sorted(reach_count):
            if exits_mapped >> q & 1:
                continue
            nd = bnodes[q]
            x, y = on.pos[nd]
            through = inside_dist(self.box, x, y) < 0 and len(on.at[nd]) >= 2 and nd not in on.synthetic
            F.add("pairs.exit_unmapped" + (".road_passes_edge_node" if through else ""),
                  "info" if through else "defect", None, None, None, x, y, value=reach_count[q], limit=0, osm_node=nd,
                  detail="edge node outside the box that the road runs through" if through else
                  f"OSM leaves the map here (reached from {reach_count[q]} edge entries), no lane ends at this edge "
                  f"point")
        lost_by_entry = collections.defaultdict(list)
        for ex, ents in lost_by_exit.items():
            for en in ents:
                lost_by_entry[en].append(ex)
        lost = sum(len(v) for v in lost_by_exit.values())
        # one root cause shows either as an entry that reaches too little or an exit reached too
        # rarely: both views are listed, the count is the number of lost (entry, exit) pairs
        for kind, table in (("entry", lost_by_entry), ("exit", lost_by_exit)):
            for nd in sorted(table, key=lambda q: (-len(table[q]), str(q))):
                x, y = on.pos[nd]
                F.add("pairs.missing", "defect", None, None, None, x, y, value=len(table[nd]), limit=0,
                      osm_node=nd, view=kind, other=[str(e) for e in table[nd][:20]],
                      detail=(f"from this edge entry the lanes miss {len(table[nd])} edge exits OSM reaches"
                              if kind == "entry" else
                              f"the lanes miss this edge exit from {len(table[nd])} edge entries OSM reaches it from"))
        self.F.d.setdefault("pairs.missing", {"severity": "defect", "count": 0, "instances": []})["count"] = lost
        self.pair_stats = {"osm_pairs": total_osm, "lane_pairs_of_those": total_lane,
                           "boundary_nodes": len(bnodes), "lane_entries": len(g.entries), "lane_exits": len(g.exits)}

    def osm_coverage(self):
        """every OSM motor way piece must be represented by lanes (mapped, or inside a junction)."""
        F, on, mp = self.F, self.on, self.map
        per_piece = collections.defaultdict(lambda: [0, 0, 0.0])
        for gi, (pi, i, a, b, L, hd) in enumerate(on.seg):
            v = per_piece[pi]
            v[2] += L
            if not mp.seg_unmapped[gi]:
                v[0] += 1
            if mp.seg_dropped[gi]:
                v[1] += 1
        for pi in sorted(per_piece):
            mapped, dropped, L = per_piece[pi]
            if mapped == 0 and dropped:
                pc = on.pieces[pi]
                nds = pc["nodes"]
                x, y = on.pos[nds[len(nds) // 2]]
                F.add("osm.way_dropped", "defect", None, None, None, x, y, value=L, limit=0, way=pc["id"],
                      highway=pc["tags"].get("highway"),
                      detail="OSM motor way has no road and does not lie on any road / junction surface")

    # ---- curb / shoulder / sidewalk continuity
    def edge_continuity(self):
        """at every section boundary of a plain road and every plain road-to-road link (either end, any
        contact), each edge lane (sidewalk / shoulder / curb / border / walking) on either side must
        continue on the other side as the same type: a break where it stops ('end') and where it
        starts ('start') are both breaks of the same edge; junctions are excluded."""
        net = self.net
        done = set()
        for rid in sorted(net.roads, key=_rid_key):
            r = net.roads[rid]
            if r.junction != "-1":
                continue
            for k in range(len(r.sections) - 1):
                a, b = r.sections[k], r.sections[k + 1]
                self._edge_boundary(r, a, "succ", a.e, r, b, "pred", b.s, 1, "section")
            for at_end, L in ((True, r.succ), (False, r.pred)):
                if L is None or L.type != "road" or L.id not in net.roads or net.roads[L.id].junction != "-1":
                    continue
                T = net.roads[L.id]
                key = frozenset([(rid, at_end), (T.id, L.contact == "end")])
                if key in done:
                    continue
                done.add(key)
                secA = r.sections[-1] if at_end else r.sections[0]
                secB = T.sections[0] if L.contact == "start" else T.sections[-1]
                attrB = "pred" if L.contact == "start" else "succ"
                flip = 1 if at_end == (L.contact == "start") else -1
                self._edge_boundary(r, secA, "succ" if at_end else "pred", r.length if at_end else 0.0,
                                    T, secB, attrB, 0.0 if L.contact == "start" else T.length, flip, "road")

    def _edge_boundary(self, rA, secA, attrA, sA, rB, secB, attrB, sB, flip, where):
        """edge lanes across one boundary: A lanes name B lanes through attrA, B lanes name A lanes
        through attrB; flip -1 when the side (sign of the id) changes across an end-end / start-start link."""
        F = self.F

        def linked(l, sec_o, attr_o, attr_l):
            rec = getattr(l, attr_l)
            if rec is not None and sec_o.lane(rec) is not None and sec_o.lane(rec).type == l.type:
                return True
            return any(m.type == l.type and getattr(m, attr_o) == l.id for m in sec_o.lanes())

        unlinked_a = set()
        for side, (r, sec, attr, s_at, r_o, sec_o, attr_o, f) in enumerate((
                (rA, secA, attrA, sA, rB, secB, attrB, flip), (rB, secB, attrB, sB, rA, secA, attrA, flip))):
            for l in sec.lanes():
                if l.type not in CONTINUITY_TYPES or linked(l, sec_o, attr_o, attr):
                    continue
                cand = [q for q in sec_o.lanes() if q.type == l.type and (q.id > 0) == ((l.id > 0) == (f > 0))]
                kind = "end" if abs(s_at - sec.e) < 1e-9 and sec.e > sec.s else "start"  # in the road's s
                x, y, _ = lane_centre(r, sec, l.id, s_at)
                if cand:
                    if side == 1 and any((rA.id, q.id) in unlinked_a for q in cand):
                        continue  # the same missing link, already reported from the A side
                    if side == 0:
                        unlinked_a.add((rA.id, l.id))
                    tgt = sec_o.lane(getattr(l, attr)) if getattr(l, attr) is not None else None
                    F.add("edge.unlinked", "defect", r.id, l.id, s_at, x, y, lane_type=l.type, at=where,
                          detail=f"{l.type} continues across the {where} boundary (road {r_o.id} lane "
                                 f"{cand[0].id}) but no lane link joins them"
                                 f"{'' if tgt is None else ' (link points at a ' + tgt.type + ')'}")
                    continue
                side_o = (1 if l.id > 0 else -1) * f
                says = self._edge_osm_side(r_o, sec_o, side_o, l.type)
                if says is True:
                    cls, dist = "unexplained", None  # OSM tags this edge on that side of the gap
                elif says is False:
                    cls, dist = "osm_absent", None   # OSM tags no such edge on that side of the gap
                else:
                    cls, dist = self._edge_osm_reason(r, s_at)
                F.add("edge.break." + cls, "defect" if cls == "unexplained" else "info", r.id, l.id, s_at, x, y,
                      value=dist, limit=TAPER_LEN, lane_type=l.type, at=where, kind=kind, osm_side=says,
                      detail=f"{l.type} {'stops' if kind == 'end' else 'starts'} at the {where} boundary; "
                             + ("OSM tags it on that side of the gap" if says is True else
                                "OSM tags none on that side of the gap" if says is False else
                                f"OSM has no side tag there; nearest OSM change of its tags "
                                f"{'none' if dist is None else f'{dist:.1f} m away'}"))

    def _edge_osm_side(self, r, sec, side, ltype):
        """what the OSM ways mapped onto section sec of road r say about an edge of type ltype on
        road side `side` (+1 left of s, -1 right): True if some way tags it present there, False if
        every way tags it absent, None if no way has an explicit tag (defaults are the converter's)."""
        if self.map is None or ltype not in OSM_EDGE_KEYS:
            return None
        votes = []
        # samples near the section ends may come from the neighbouring way (OSM node vs section start
        # differ by up to one sample step + the reference-line tolerance): only the interior votes
        m = SAMPLE_STEP + MAP_TOL
        for s, sd, pi in self.map.road_sd.get(r.id, ()):
            if sec.s + m <= s <= sec.e - m:
                votes.append(_osm_edge_tag(self.on.pieces[pi]["tags"], ltype, "left" if side * sd > 0 else "right"))
        if any(v is True for v in votes):
            return True
        if votes and all(v is False for v in votes):
            return False
        return None

    def _edge_osm_reason(self, r, s_at):
        """(class, distance): where along the road OSM changes the tags the edge depends on
        (sidewalk / shoulder / kerb keys and the highway class, whose defaults decide them). A change
        within one transition length (BuildOptions::taperLength) of the break explains it."""
        if self.map is None:
            return "unexplained", None
        samples = sorted(self.map.road_osm.get(r.id, ()), key=lambda q: q[0])
        nxt = []
        if s_at >= r.length - 1e-6 and r.succ is not None and r.succ.type == "road" and r.succ.id in self.net.roads:
            T = self.net.roads[r.succ.id]
            for s, a, pi in self.map.road_osm.get(T.id, ()):
                nxt.append((r.length + (s if r.succ.contact == "start" else T.length - s), a, pi))
        prv = []
        if s_at <= 1e-6 and r.pred is not None and r.pred.type == "road" and r.pred.id in self.net.roads:
            T = self.net.roads[r.pred.id]
            for s, a, pi in self.map.road_osm.get(T.id, ()):
                prv.append(((s - T.length) if r.pred.contact == "end" else -s, a, pi))
        seq = sorted(prv, key=lambda q: q[0]) + samples + sorted(nxt, key=lambda q: q[0])
        if not seq:
            return "osm_unknown", None
        key = (lambda pi: tuple(self.on.pieces[pi]["tags"].get(k) for k in EDGE_TAG_KEYS + ("highway",)))
        changes = [(seq[i][0] + seq[i + 1][0]) / 2 for i in range(len(seq) - 1)
                   if seq[i][2] != seq[i + 1][2] and key(seq[i][2]) != key(seq[i + 1][2])]
        if not changes:
            return "unexplained", None
        d = min(abs(c - s_at) for c in changes)
        return ("osm_tag_change" if d <= TAPER_LEN else "unexplained"), d

    # ---- the user's 'same lanes = same road' rule
    def splits(self):
        F, net = self.F, self.net
        seen = set()
        for rid in sorted(net.roads, key=_rid_key):
            r = net.roads[rid]
            if r.junction != "-1":
                continue
            # redundant sections
            for k in range(len(r.sections) - 1):
                a, b = r.sections[k], r.sections[k + 1]
                # a lane rule (1.8.0 §11.9) has no end of its own: a change of rules needs the section (NAT-2)
                if [(l.id, l.type, l.rules) for l in a.lanes()] == [(l.id, l.type, l.rules) for l in b.lanes()] and \
                        all(l.succ == l.id for l in a.lanes()) and all(l.pred == l.id for l in b.lanes()):
                    x, y, _ = r.ref(np.array([b.s]))
                    F.add("split.section_redundant", "defect", rid, None, b.s, float(x[0]), float(y[0]),
                          section=k + 1, detail="same lanes, identity links: widths could continue in one section "
                                                "(spec NAT-2: a removable section boundary is a defect)")
            # plain road -> plain road at either end, any contact (end-end / start-start flip the ids)
            for at_end, L in ((True, r.succ), (False, r.pred)):
                if L is None or L.type != "road" or L.id not in net.roads:
                    continue
                T = net.roads[L.id]
                if T.junction != "-1":
                    continue
                key = frozenset([(rid, at_end), (T.id, L.contact == "end")])
                if key in seen:
                    continue
                seen.add(key)
                a = r.sections[-1] if at_end else r.sections[0]
                b = T.sections[0] if L.contact == "start" else T.sections[-1]
                flip = 1 if at_end == (L.contact == "start") else -1
                # the user's rule counts traffic lanes: same motor / bike lanes (id, type, travel
                # direction) on both sides = the same road; sidewalks / shoulders appearing or ending
                # belong in a new lane section of the same road
                traffic = MOTOR_TYPES | BIKE_TYPES
                ka = len(r.sections) - 1 if at_end else 0
                kb = 0 if L.contact == "start" else len(T.sections) - 1
                sig_a = sorted((l.id * flip, l.type, tuple(sorted(d * flip for d in net.dirs(rid, ka, l))))
                               for l in a.lanes() if l.type in traffic)
                sig_b = sorted((l.id, l.type, tuple(sorted(net.dirs(T.id, kb, l))))
                               for l in b.lanes() if l.type in traffic)
                if sig_a != sig_b or not sig_a:
                    # spec NAT-1: two non-junction roads MUST NOT be linked road-to-road; a change of
                    # the lane layout inside a chain is a lane section, not a new road
                    sa_ = r.length if at_end else 0.0
                    x, y, _ = r.ref(np.array([sa_]))
                    F.add("split.road_to_road", "defect", rid, None, sa_, float(x[0]), float(y[0]),
                          value=len(sig_b) - len(sig_a), limit=0, next_road=T.id,
                          detail=f"road {rid} -> road {T.id} linked road-to-road with a different traffic lane "
                                 f"layout ({len(sig_a)} -> {len(sig_b)} lanes): a lane section of one road")
                    continue
                all_equal = sorted((l.id * flip, l.type) for l in a.lanes()) == sorted((l.id, l.type) for l in b.lanes())
                sa_ = r.length if at_end else 0.0
                x, y, _ = r.ref(np.array([sa_]))
                wa = {l.id * flip: lane_width(a, l, a.e if at_end else a.s) for l in a.lanes()}
                wb = {l.id: lane_width(b, l, b.s if L.contact == "start" else b.e) for l in b.lanes()}
                dw = max([abs(wa[q] - wb[q]) for q in wa if q in wb] + [0])
                ways_a = set(r.osm_ways)
                ways_b = set(T.osm_ways)
                if ways_a & ways_b:
                    cls, why = "same_osm_way", sorted(ways_a & ways_b)
                elif self.osm is not None:
                    ta = self._joint_tags(r, at_end)
                    tb = self._joint_tags(T, L.contact == "end")
                    why = sorted(k for k in SPLIT_TAG_KEYS if ta.get(k) != tb.get(k))
                    cls = "osm_tags_differ" if why else "osm_tags_equal"
                else:
                    cls, why = "unknown", []
                F.add("split.road_needless", "defect", rid, None, sa_, float(x[0]), float(y[0]), value=dw,
                      limit=None, cls=cls, keys=why, next_road=T.id, all_lanes_equal=all_equal,
                      lanes_linked_identically=all(getattr(l, "succ" if at_end else "pred") == l.id * flip
                                                   for l in a.lanes()),
                      detail=f"road {rid} -> road {T.id} with the same traffic lanes (widths differ by up to "
                             f"{dw:.2f} m{'' if all_equal else ', edge lanes differ'}): one road should carry "
                             f"on with a new lane section ({cls})")

    def _joint_tags(self, r, at_end):
        if self.map is None:
            return {}
        cand = [(s, pi) for s, a, pi in self.map.road_osm.get(r.id, ())]
        if not cand:
            ws = [w for w in r.osm_ways if w in self.osm.ways]
            return self.osm.ways[ws[-1 if at_end else 0]][1] if ws else {}
        s, pi = max(cand) if at_end else min(cand)
        return self.on.pieces[pi]["tags"]

    # ---- game summary
    def summary(self):
        g, b = self.motor, self.bike
        out = {"motor_nodes": len(g.key), "motor_edges": sum(len(o) for o in g.out),
               "edge_kinds": dict(collections.Counter(kd for o in g.out for _, kd in o)),
               "boundary_entries": len(g.entries), "boundary_exits": len(g.exits),
               "reachable_from_edge": int(g.fwd.sum()), "can_reach_edge": int(g.bwd.sum()),
               "both": int((g.fwd & g.bwd).sum()), "bike_nodes": len(b.key),
               "box": [round(v, 2) for v in self.box], "box_source": self.box_source,
               "rule": getattr(self, "rule_evidence", None), "timing": self.timing}
        # edges a boundary entry driver can use (every edge driven once)
        used = sum(1 for i in range(len(g.key)) if g.fwd[i] for _ in g.out[i])
        out["edges_driven_from_edge"] = used
        if self.on is not None:
            out["osm_states"] = len(self.on.valid)
            out["osm_boundary_nodes"] = len(self.on.boundary)
            out["osm_edge_slivers"] = self.on.edge_slivers
            out["osm_transitions"] = len(self.T_osm)
            out["lane_transitions"] = len(self.T_lane)
            out["pairs"] = getattr(self, "pair_stats", None)
        return out


# OSM keys per continuity lane type: (key prefixes tried as <prefix>:<side>, <prefix>:both, <prefix>),
# values meaning present, values meaning absent ('separate' = drawn as its own way, no lane here)
OSM_EDGE_KEYS = {
    "sidewalk": (("sidewalk",), {"yes", "both"}, {"no", "none", "separate"}),
    "shoulder": (("shoulder",), {"yes", "both"}, {"no", "none"}),
    "biking": (("cycleway",), {"lane", "track", "opposite_lane", "opposite_track", "yes"},
               {"no", "none", "separate", "shared_lane", "share_busway", "opposite"}),
    "parking": (("parking", "parking:lane"), {"lane", "street_side", "on_kerb", "half_on_kerb", "shoulder", "yes",
                                              "parallel", "diagonal", "perpendicular", "marked"},
                {"no", "none", "separate", "no_parking", "no_stopping", "fire_lane"}),
}


def _osm_edge_tag(tags, ltype, way_side):
    """does OSM tag an edge lane of type ltype on one side of a way (relative to the way's direction)?
    True / False / None (untagged). Keys <k>:<side>, <k>:both, then <k> whose value may also name the
    side (both / left / right)."""
    prefixes, yes, no = OSM_EDGE_KEYS[ltype]
    for pre in prefixes:
        for k in (f"{pre}:{way_side}", f"{pre}:both"):
            v = tags.get(k)
            if v in yes:
                return True
            if v in no:
                return False
    for pre in prefixes:
        v = tags.get(pre)
        if v == "both":
            return True
        if v in ("left", "right"):
            return v == way_side
        if v in yes:
            return True if ltype != "sidewalk" else None  # sidewalk=yes does not say which side
        if v in no:
            return False
    return None


def _turn_permitted(turn_lanes, ang, exits=()):
    """can any lane of an OSM turn:lanes value make a movement turning ang degrees (+ = left)?
    Spec JCT-2: turn classes by order, not by angle thresholds. Among the angles of all OSM exits of
    the same approach (exits), 'through' is the one with the smallest |angle|; left / slight_left /
    sharp_left permit any exit left of it (larger angle), right* any exit right of it (the slight /
    plain / sharp rank is not used: generous); merge_to_* count as through; reverse permits a U-turn
    (> UTURN_DEG); 'none' / empty = no arrow = any movement."""
    thr = min(list(exits) + [ang], key=abs)
    for lane in turn_lanes.split("|"):
        for v in (lane.split(";") if lane else [""]):
            v = v.strip()
            if v in ("", "none"):
                return True
            if v in ("through", "merge_to_left", "merge_to_right") and abs(ang - thr) < 1e-9:
                return True
            if v in ("left", "slight_left", "sharp_left") and ang > thr + 1e-9:
                return True
            if v in ("right", "slight_right", "sharp_right") and ang < thr - 1e-9:
                return True
            if v == "reverse" and abs(ang) > UTURN_DEG:
                return True
    return False


def _link_str(r, at_end):
    L = r.succ if at_end else r.pred
    return None if L is None else f"{L.type}:{L.id}" + (f"/{L.contact}" if L.contact else "")


# ----------------------------------------------------------------------------- crops / jev
def make_crops(lf, findings, out, n, basemap=None):
    """evidence crops of the first n instances of every defect invariant: lanes by type, reference
    lines, the OSM motor ways (dashed) and labels on the roads the instance names."""
    import copy
    od = lf.net.od
    osm = None
    if lf.osm is not None:
        osm = copy.copy(lf.osm)
        osm.ways = {k: v for k, v in lf.osm.ways.items() if v[1].get("highway") in HIGHWAYS}
    made = []
    for name, e in findings.items():
        if name.startswith("_") or e["severity"] != "defect" or not e["count"]:
            continue
        for q, inst in enumerate([i for i in e["instances"] if i.get("x") is not None][:n]):
            p = os.path.join(out, "crops", f"{name}_{q}.png")
            roads = [inst.get("road")] + [str(inst.get(k))[:-2] for k in ("frm", "to") if inst.get(k)] +                 (inst.get("roads") or [])[:6]
            marks = [(inst["x"], inst["y"], name.split(".")[-1])]
            for rid in dict.fromkeys(r for r in roads if r in od.roads):
                r = od.roads[rid]
                x, y, _ = r.ref(np.array([r.length / 2]))
                marks.append((float(x[0]), float(y[0]), "r" + rid))
            title = f"{name} road {inst.get('road')} lane {inst.get('lane')} {inst.get('detail') or ''}"[:150]
            G.crop(od, osm, (inst["x"], inst["y"]), 45, p, basemap=basemap, marks=marks, title=title)
            inst["crop"] = p
            made.append(p)
    return made


def jev_opinions(findings, n):
    """secondary opinion (never a gate): ask Jev whether the measured evidence describes a defect."""
    try:
        from jev import ask
    except Exception as ex:  # pragma: no cover
        return f"jev unavailable: {ex}"
    for name, e in findings.items():
        if name.startswith("_") or e["severity"] != "defect":
            continue
        for inst in e["instances"][:n]:
            state = (f"OpenDRIVE lane-graph check '{name}'. Road {inst.get('road')}, lane {inst.get('lane')}, "
                     f"s={inst.get('s')}. Measured: value={inst.get('value')} limit={inst.get('limit')}. "
                     f"{inst.get('detail') or ''} Other facts: " +
                     ", ".join(f"{k}={v}" for k, v in inst.items()
                               if k not in ("road", "lane", "s", "x", "y", "value", "limit", "detail", "crop", "roads")))
            try:
                a = ask(state[:1800], {"defect": {"type": "noul", "instructions":
                                       "Is this a real defect of a lane-level road network produced from OSM (a "
                                       "lane a car cannot drive on / out of, or a wrong connection), rather than "
                                       "legitimate road topology?"}})
                inst["jev_defect"] = round(float(a["defect"]["noul"]), 3)
            except Exception as ex:  # network etc.
                inst["jev_defect"] = f"error: {ex}"[:120]
    return "ok"


# ----------------------------------------------------------------------------- CLI / API
def area_paths(area):
    import gate
    a = gate.AREAS[area]
    args = a["args"]
    clip = args[args.index("--clip") + 1] if "--clip" in args else a.get("clip")
    basemap = args[args.index("--basemap") + 1] if "--basemap" in args else None
    return a["xodr"], a["osm"], [float(v) for v in clip.split(",")] if clip else None, basemap


# laneflow invariant (prefix, first match wins) -> docs/SPEC_GEOMETRY.md rule id; 'level' follows the
# spec (MUST for a defect, SHOULD where the spec says SHOULD, INFO for classifications)
SPEC_MAP = (
    ("motor.dead_end.unlinked_road_end", "FLOW-1/exit-unlinked", None),
    ("motor.dead_end.no_osm_node", "FLOW-1/exit-unlinked", None),
    ("motor.orphan.unlinked_road_end", "FLOW-1/entry-unlinked", None),
    ("motor.orphan.no_osm_node", "FLOW-1/entry-unlinked", None),
    ("motor.dead_end.section_unlinked", "FLOW-1/abrupt-end", None),
    ("motor.orphan.section_unlinked", "FLOW-1/abrupt-start", None),
    ("motor.dead_end.", "FLOW-1/exit-no-lane-link", None),
    ("motor.orphan.junction_lane_without_connection", "FLOW-1/entry-no-lane-link", "SHOULD"),
    ("motor.orphan.junction_no_connection", "FLOW-1/entry-no-lane-link", "SHOULD"),
    ("motor.orphan.road_link_without_lane_link", "FLOW-1/entry-no-lane-link", "SHOULD"),
    ("motor.orphan.connecting_road_unlinked", "FLOW-1/entry-no-lane-link", "SHOULD"),
    ("motor.orphan.", "FLOW-1/entry", None),
    ("motor.flow.unreachable", "FLOW-4/orphan", None),
    ("motor.flow.trapped", "FLOW-4/trap", None),
    ("motor.trap_region", "FLOW-4/trap", None),
    ("motor.island.missing_link", "FLOW-3/missing-link", None),
    ("motor.island", "FLOW-3/independent", None),
    ("motor.link.direction", "FLOW-2/wrong-way", None),
    ("bike.link.direction", "FLOW-2/wrong-way", None),
    ("link.lane_asymmetric", "FLOW-2/asymmetric", None),
    ("link.junction_asymmetric", "FLOW-2/asymmetric", None),
    ("link.road_asymmetric", "FLOW-2/asymmetric", None),
    ("link.junction_not_linked", "FLOW-2/asymmetric", None),
    ("link.plain_to_connecting", "FLOW-2/dangling", None),
    ("link.", "FLOW-2/dangling", None),
    ("motor.link.", "FLOW-2/dangling", None),
    ("unusable.narrow_lane", "FLOW-6/narrow", None),
    ("unusable.zero_width_lane", "FLOW-6/zero", None),
    ("unusable.zero_length_section", "FLOW-6/zero", None),
    ("unusable.section_without_motor_lane", "FLOW-6/no-drivable-lane", None),
    ("osm.direction.wrong_way", "TOP-2", None),
    ("osm.direction.missing", "TOP-2", None),
    ("osm.flow.lost", "FLOW-4/unreachable", None),
    ("osm.way_dropped", "FLOW-3/missing-link", None),
    ("turn.missing", "JCT-1", None),
    ("turn.extra", "JCT-2", None),
    ("pairs.", "FLOW-4/unreachable", None),
    ("edge.", "NAT-4", None),
    ("split.road_needless", "NAT-3", None),
    ("split.road_to_road", "NAT-1", None),
    ("split.section_redundant", "NAT-2", None),
    ("rule.osm", "TOP-1/SIM-3", None),
    ("bike.", "FLOW (bike graph, informative)", None),
)


def spec_of(name, severity):
    for pre, rid, lvl in SPEC_MAP:
        if name.startswith(pre):
            return rid, ("MUST" if severity == "defect" else (lvl or "INFO"))
    return None, ("MUST" if severity == "defect" else "INFO")


def run(xodr, osm=None, box=None, rule=None, box_source=None):
    """importable entry: returns (LaneFlow, findings dict)."""
    lf = LaneFlow(xodr, osm, box, rule, box_source)
    F = lf.run()
    d = F.as_dict()
    for k, e in d.items():
        e["spec"], e["level"] = spec_of(k, e["severity"])
    d["_meta"] = {"xodr": xodr, "osm": osm, "rule_override": rule, "summary": lf.summary(),
                  "constants": {"BOUNDARY_BAND": BOUNDARY_BAND, "ROAD_END_BAND": ROAD_END_BAND, "MAP_TOL": MAP_TOL,
                                "WIDTH_ZERO": WIDTH_ZERO,
                                "MIN_VEHICLE_WIDTH": MIN_VEHICLE_WIDTH, "UTURN_DEG": UTURN_DEG,
                                "ABSORB_MAX": ABSORB_MAX, "SAMPLE_STEP": SAMPLE_STEP}}
    return lf, d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr", nargs="?")
    ap.add_argument("osm", nargs="?")
    ap.add_argument("--area", help="reference area from gate.py (la, munich, london, frankfurt_kreuz, stelvio)")
    ap.add_argument("--clip", help="MINLAT,MINLON,MAXLAT,MAXLON as given to osm2xodr --clip")
    ap.add_argument("--rule", choices=["RHT", "LHT"], help="evaluate under this traffic rule instead of road@rule")
    ap.add_argument("--out", help="output directory (default build/tmp/spec/laneflow/<name>)")
    ap.add_argument("--crops", type=int, default=3, help="evidence crops per defect invariant")
    ap.add_argument("--jev", type=int, default=0, help="ask Jev about this many instances per defect invariant")
    a = ap.parse_args(argv)
    basemap = None
    clip = None
    if a.area:
        xodr, osm, clip, basemap = area_paths(a.area)
        xodr, osm = a.xodr or xodr, a.osm or osm
        name = a.area
    else:
        xodr, osm, name = a.xodr, a.osm, os.path.splitext(os.path.basename(a.xodr))[0]
    if a.clip:
        clip = [float(v) for v in a.clip.split(",")]
    if a.rule:
        name += "_" + a.rule
    out = a.out or os.path.join(r"C:\mywork\osm2carla\build\tmp\spec\laneflow", name)
    os.makedirs(out, exist_ok=True)
    box = None
    if clip:
        with open(xodr, encoding="utf-8") as f:
            head = f.read(4000)
        lat0 = float(re.search(r"\+lat_0=([-0-9.eE]+)", head).group(1))
        lon0 = float(re.search(r"\+lon_0=([-0-9.eE]+)", head).group(1))
        box = clip_box_from_latlon(clip, lat0, lon0)
    lf, d = run(xodr, osm, box, a.rule, "clip " + ",".join(map(str, clip)) if clip else None)
    if a.crops:
        make_crops(lf, d, out, a.crops, basemap)
    if a.jev:
        d["_meta"]["jev"] = jev_opinions(d, a.jev)
    with open(os.path.join(out, "findings.json"), "w", encoding="utf-8") as f:
        json.dump(d, f, indent=1, default=str)
    print(f"{name}: {os.path.join(out, 'findings.json')}")
    for k, e in d.items():
        if k.startswith("_"):
            continue
        if e["count"] or e["severity"] == "defect":
            print(f"  {e['severity']:6s} {e['count']:6d}  {k}")
    s = d["_meta"]["summary"]
    print("  summary:", json.dumps({k: v for k, v in s.items() if k not in ("timing",)}, default=str))
    print("  timing:", s["timing"])
    return d


if __name__ == "__main__":
    main()
