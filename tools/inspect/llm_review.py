"""Optional LLM review of OdrForge outputs: a reviewer (a Claude Code session or claude -p) judges what the
deterministic checkers measured, from every real-world perspective (car, pedestrian, wheelchair, cyclist, simulator,
visual, logical), and its verdicts land next to each output as <stem>.review.json for the viewer.

    python llm_review.py prepare OUT.xodr [...] --out DIR [--snapshot DIR] [--run-checkers] [--spots 20] [--samples 4]
    python llm_review.py prompt                              # the versioned prompt (llm_review_prompt.py)
    python llm_review.py run DIR --runner session            # what a Claude Code session reviews next, and how
    python llm_review.py run DIR --runner claude-cli         # claude -p, headless, one chunk per call
    python llm_review.py merge DIR [--dest DIR]              # verdicts -> <stem>.review.json next to each output
    python llm_review.py check-prompt [--xodr F ...]         # warn when the prompt is older than the checkers / specs

prepare writes one bundle per output: manifest.json, facts.json (the review's own measured network facts: open lane
ends, dead ends against OSM, zero-width lanes, gaps and kinks at joints, lane drops and their tapers, crosswalk ends,
junction turns OSM allows, wrong-way links, the driving-lane graph), candidates.json (those facts plus every finding
of the deterministic checkers, normalised), and chunks of a few spots each (chunk_NN.md / .json, img/spot_*.png plan
views). Nothing here sends data anywhere; the claude-cli runner calls the local Claude Code CLI only when asked.
Images need matplotlib (the osm2carla venv has none: use a Python that has it, or pass --no-images).
"""
import argparse
import collections
import datetime
import glob
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import odrgeom as G  # noqa: E402
import llm_review_prompt as P  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
DATA = r"C:\mywork\osm2carla"
GATE_WORK = os.path.join(DATA, "build", "tmp", "inspect", "gate")
VENV_PY = os.path.join(DATA, "venv", "Scripts", "python.exe")  # the gate's interpreter: has what the checkers import

DRIVE = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "slipLane", "bus", "taxi", "HOV",
         "bidirectional", "mwyEntry", "mwyExit"}
WALK = {"sidewalk", "walking"}
BIKE = {"biking"}
MOVE = DRIVE | WALK | BIKE
VEHICLE_HW = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street",
              "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link", "road", "busway"}
MAJOR_HW = {"motorway", "trunk", "primary", "secondary", "motorway_link", "trunk_link", "primary_link", "secondary_link"}
FOOT_HW = {"footway", "path", "pedestrian", "steps", "cycleway", "bridleway", "corridor", "crossing"}

# Measured-fact limits (metres, degrees). They decide what becomes a candidate for the reviewer, never a verdict.
# Hardening rule: never loosen them to make a file look clean.
W_OPEN = 0.3          # a lane ending wider than this with no continuation is an open end (narrower: a taper)
GAP_MINOR = 0.02      # border gap at a lane joint
GAP_MAJOR = 0.10
KINK_DEG = 2.0        # heading change at a road joint
KINK_MAJOR_DEG = 5.0
BOUNDARY_M = 20.0     # a dead end this close to the clip box edge is the area boundary
OSM_NODE_M = 40.0     # search radius for the OSM node of a dead end
NEAR_END_M = 15.0     # another road's end this close to a dead end hints at a missing link
SLIVER_M = 1.5        # driving lane never wider than this over a whole section (>= SLIVER_LEN) is a sliver
SLIVER_LEN = 10.0
ZERO_W = 0.05
# Real-world limits: docs/research/ROAD_DESIGN_SENSIBILITY.md ("Quick thresholds", sources in brackets)
DRIVE_W_MIN, DRIVE_W_MAX = 2.5, 4.5   # driving lane width outside tapers and junctions (urban 2.75-3.5 m; [S3][S4][S27][S35])
TIGHT_R = 5.0         # lane-centre radius (1 m average) on a connecting road: a car's turning centreline is >= 6.4 m [S12]
TIGHT_R_MAJOR = 3.0   # ... below this no passenger car follows the lane at all
WALK_TOUCH = 0.5      # a crosswalk end touches a walking lane within this distance
TURN_UTURN_DEG = 165.0  # a turn sharper than this between two arms is a U-turn, not reported
SHARP_TURN_DEG = 135.0  # a missing turn sharper than this is reported as info only (often not drivable)
SEV_W = {"critical": 8.0, "major": 4.0, "minor": 2.0, "info": 0.5}

DETECTORS = ["osm_fidelity", "link_continuity", "edge_smoothness", "junction_gore", "lane_transitions"]
INV_SEV = {"fold": "major", "width_negative": "major", "width_pinch": "major", "G0_border": "major", "G0_ref": "major",
           "G0_open_lane": "critical", "G1_ref": "minor", "G1_border": "minor", "topology": "major",
           "flow_direction": "critical", "flow_dead_end": "critical", "flow_dead_start": "major", "flow_orphan": "major",
           "flow_trap": "critical", "flow_closed_not_strong": "major", "flow_narrow": "minor",
           "flow_no_drivable": "major", "sidewalk_on_driving": "major", "osm_ref_far": "minor", "osm_off_road": "minor",
           "lane_count": "minor", "continuity_nondriving": "minor", "continuity_junction_corner": "minor",
           "degenerate": "minor", "laneOffset_jump": "major", "width_record_jump": "major", "shape_unchanged": "info",
           "needless_split": "info", "flow_boundary_short": "info"}


# ----------------------------------------------------------------------------- small helpers
def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def text_fingerprint(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read().replace(b"\r\n", b"\n")).hexdigest()[:12]


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def dump(obj, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=1, ensure_ascii=False)
    os.replace(tmp, path)


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def r3(v):
    return None if v is None else round(float(v), 3)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def area_of(stem):
    """reference area of an output name (gate.py AREAS): munich_standard -> munich, LA_DTLA_odrforge -> la"""
    s = re.sub(r"_standard$", "", stem)
    return "la" if s.lower().startswith("la_dtla") else s


def gate_areas():
    try:
        import gate
        return gate.AREAS
    except Exception:  # gate imports the terrain / metrics tools; the review must work without them
        return {}


# ----------------------------------------------------------------------------- the network
def bands(r, sec, s):
    """{lane id: (t_inner, t_outer) arrays} at the s values (vectorised Road.borders_at)"""
    s = np.atleast_1d(np.asarray(s, float))
    off = r.lane_offset(s)
    ds = s - sec.s
    out = {0: (off, off)}
    t = off.copy()
    for l in sec.left:
        w = l.width(ds)
        out[l.id] = (t, t + w)
        t = t + w
    t = off.copy()
    for l in sec.right:
        w = l.width(ds)
        out[l.id] = (t, t - w)
        t = t - w
    return out


class Net:
    """odrgeom's OpenDrive plus what the review needs: lane directions, traffic rule, speeds, direct junctions,
    crosswalks, and the continuation of every lane end."""

    def __init__(self, path):
        self.path = path
        self.od = G.OpenDrive(path)
        self.roads = self.od.roads
        root = ET.parse(path).getroot()
        h = root.find("header")
        self.header = {k: h.get(k) for k in ("revMajor", "revMinor", "name", "vendor", "date",
                                              "north", "south", "east", "west")}
        self.jtype, self.direct = {}, {}
        for j in root.findall("junction"):
            jid = j.get("id")
            self.jtype[jid] = j.get("type", "default")
            if self.jtype[jid] == "direct":
                self.direct[jid] = [(c.get("incomingRoad"), c.get("linkedRoad"), c.get("contactPoint") or "start",
                                     [(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")])
                                    for c in j.findall("connection")]
        self.in_junction = collections.defaultdict(list)
        self.rule, self.speed, self.dirattr = {}, {}, {}
        for r in self.roads.values():
            if r.junction != "-1":
                self.in_junction[r.junction].append(r)
            self.rule[r.id] = (r.el.get("rule") or "RHT").upper()
            sp = r.el.find("type/speed")
            v = None
            if sp is not None and sp.get("max") not in (None, "no limit", "undefined"):
                try:
                    v = float(sp.get("max")) * {"mph": 0.44704, "m/s": 1.0}.get(sp.get("unit", "km/h"), 1 / 3.6)
                except ValueError:
                    v = None
            self.speed[r.id] = v
            secs = sorted(r.el.find("lanes").findall("laneSection"), key=lambda e: float(e.get("s")))
            for k, se in enumerate(secs):
                for l in se.iter("lane"):
                    if l.get("direction"):
                        self.dirattr[(r.id, k, int(l.get("id")))] = l.get("direction")

    # -- ends and directions
    @staticmethod
    def end_link(r, E):
        return r.pred if E == "start" else r.succ

    @staticmethod
    def end_section(r, E):
        return (0, r.sections[0]) if E == "start" else (len(r.sections) - 1, r.sections[-1])

    @staticmethod
    def end_s(r, E):
        return 0.0 if E == "start" else r.length

    def travel(self, r, k, l):
        """+1 along s, -1 against s, 0 both ways, None for lanes nothing travels on"""
        if l.type in WALK or l.type == "bidirectional":
            return 0
        if l.type not in DRIVE and l.type not in BIKE:
            return None
        d = self.dirattr.get((r.id, k, l.id), "standard")
        if d == "both":
            return 0
        sgn = (1 if l.id < 0 else -1) if self.rule[r.id] != "LHT" else (1 if l.id > 0 else -1)
        return -sgn if d == "reversed" else sgn

    def flow_at(self, r, k, l, E):
        """'in' (the lane runs into end E), 'out', 'both' or None"""
        d = self.travel(r, k, l)
        if d is None:
            return None
        if d == 0:
            return "both"
        return "in" if (d == 1) == (E == "end") else "out"

    def junction_end(self, r, jid):
        for E in ("start", "end"):
            lk = self.end_link(r, E)
            if lk is not None and lk.type == "junction" and lk.id == jid:
                return E
        return None

    def targets(self, r, E, l):
        """where lane l at end E of road r continues: ([(road id, contact, lane id)], how)"""
        lk = self.end_link(r, E)
        if lk is None:
            return [], "none"
        if lk.type == "road":
            m = l.pred if E == "start" else l.succ
            return ([(lk.id, lk.contact or "start", m)] if m is not None else []), "road"
        jid = lk.id
        out = []
        if self.jtype.get(jid) == "direct":
            for inc, linked, C, pairs in self.direct.get(jid, []):
                if inc == r.id:
                    out += [(linked, C, b) for a, b in pairs if a == l.id]
                elif linked == r.id and inc in self.roads:
                    CX = self.junction_end(self.roads[inc], jid)
                    if CX:
                        out += [(inc, CX, a) for a, b in pairs if b == l.id]
            return sorted(set(out)), "direct"
        for c in self.od.junctions.get(jid, []):
            cr = self.roads.get(c["connecting"])
            if c["incoming"] != r.id or cr is None:
                continue
            C = c["contact"] or "start"
            cl = self.end_link(cr, C)
            if cl is not None and cl.type == "road" and cl.id == r.id and cl.contact and cl.contact != E:
                continue  # the connection belongs to the road's other end
            out += [(cr.id, C, b) for a, b in c["lanes"] if a == l.id]
        for cr in self.in_junction.get(jid, []):
            for CE in ("start", "end"):
                cl = self.end_link(cr, CE)
                if cl is None or cl.type != "road" or cl.id != r.id or (cl.contact or E) != E:
                    continue
                _, sec = self.end_section(cr, CE)
                for cl_lane in sec.lanes():
                    if (cl_lane.pred if CE == "start" else cl_lane.succ) == l.id:
                        out.append((cr.id, CE, cl_lane.id))
        return sorted(set(out)), "junction"

    def lane_at_end(self, rid, C, lid):
        T = self.roads.get(rid)
        if T is None:
            return None, None, None
        k, sec = self.end_section(T, C)
        return T, k, sec.lane(lid)

    def end_points(self, r, E, sec, lid):
        """(inner xy, outer xy, centre xy, width) of a lane at road end E"""
        s = self.end_s(r, E)
        b = bands(r, sec, [s])
        ti, to = float(b[lid][0][0]), float(b[lid][1][0])
        xs, ys = r.point(np.array([s, s, s]), np.array([ti, to, 0.5 * (ti + to)]))
        return (xs[0], ys[0]), (xs[1], ys[1]), (xs[2], ys[2]), abs(to - ti)

    def heading_at(self, r, s):
        return float(r.ref(np.array([s]))[2][0])


# ----------------------------------------------------------------------------- OSM index
class OsmIndex:
    def __init__(self, osm):
        self.osm = osm
        self.hw = {wid: (nds, tags) for wid, (nds, tags) in osm.ways.items() if "highway" in tags}
        self.node_ways = collections.defaultdict(list)
        for wid, (nds, _) in self.hw.items():
            for i, n in enumerate(nds):
                self.node_ways[n].append((wid, i, i in (0, len(nds) - 1)))
        self.grid = collections.defaultdict(list)
        for n in self.node_ways:
            if n in osm.nodes:
                x, y = osm.nodes[n]
                self.grid[(int(x // 20), int(y // 20))].append(n)
        self.restrictions = []
        for tags, members in osm.relations:
            if tags.get("type") != "restriction":
                continue
            val = next((v for k, v in tags.items() if k.startswith("restriction")), "")
            frm = [ref for t, ref, role in members if role == "from" and t == "way"]
            to = [ref for t, ref, role in members if role == "to" and t == "way"]
            via = [(t, ref) for t, ref, role in members if role == "via"]
            self.restrictions.append((val, set(frm), set(to), via))

    def nodes_near(self, x, y, radius):
        n = int(math.ceil(radius / 20))
        ci, cj = int(x // 20), int(y // 20)
        out = []
        for i in range(ci - n, ci + n + 1):
            for j in range(cj - n, cj + n + 1):
                for nd in self.grid.get((i, j), ()):
                    px, py = self.osm.nodes[nd]
                    d = math.hypot(px - x, py - y)
                    if d <= radius:
                        out.append((d, nd))
        return sorted(out)

    def via_xy(self, via):
        for t, ref in via:
            if t == "node" and ref in self.osm.nodes:
                return self.osm.nodes[ref]
            if t == "way" and ref in self.osm.ways:
                pl = self.osm.polyline(ref)
                if len(pl):
                    return tuple(pl.mean(axis=0))
        return None

    def tags(self, wid):
        w = self.osm.ways.get(str(wid))
        return w[1] if w else {}

    def foot_dist(self, x, y, radius=15.0):
        """(distance, way id) of the nearest OSM footway / path / pedestrian way or crossing within radius"""
        best = (None, None)
        ways = {w for _, n in self.nodes_near(x, y, radius + 30) for w, _, _ in self.node_ways[n]
                if self.hw[w][1].get("highway") in FOOT_HW or self.hw[w][1].get("footway")}
        for w in ways:
            pl = self.osm.polyline(w)
            if len(pl) < 2:
                continue
            a, b = pl[:-1], pl[1:]
            ab = b - a
            L2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
            u = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / L2, 0, 1)
            d = float(np.hypot(a[:, 0] + u * ab[:, 0] - x, a[:, 1] + u * ab[:, 1] - y).min())
            if d <= radius and (best[0] is None or d < best[0]):
                best = (round(d, 2), w)
        return best


def way_label(oi, wid):
    t = oi.tags(wid) if oi else {}
    keys = ("highway", "name", "lanes", "lanes:forward", "lanes:backward", "oneway", "turn:lanes", "turn:lanes:forward",
            "turn:lanes:backward", "width", "sidewalk", "sidewalk:left", "sidewalk:right", "sidewalk:both", "cycleway",
            "cycleway:left", "cycleway:right", "cycleway:both", "busway", "bus:lanes", "vehicle:backward",
            "vehicle:forward", "motor_vehicle", "psv", "footway", "crossing", "junction", "access", "service", "noexit",
            "layer", "bridge", "tunnel", "maxspeed")
    return f"{wid}[" + " ".join(f"{k}={t[k]}" for k in keys if k in t) + "]"


# ----------------------------------------------------------------------------- the review's own facts
def boundary_box(net, clip):
    if clip:
        la0, lo0, la1, lo1 = (float(v) for v in clip.split(","))
        xs, ys = G.tm_forward(np.array([la0, la0, la1, la1]), np.array([lo0, lo1, lo0, lo1]), net.od.lat0, net.od.lon0)
        return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())
    h = net.header
    if all(h.get(k) for k in ("west", "south", "east", "north")):
        return float(h["west"]), float(h["south"]), float(h["east"]), float(h["north"])
    xs = np.concatenate([r.X for r in net.roads.values()])
    ys = np.concatenate([r.Y for r in net.roads.values()])
    return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())


def boundary_dist(box, x, y):
    x0, y0, x1, y1 = box
    if x < x0 or x > x1 or y < y0 or y > y1:
        return 0.0
    return min(x - x0, x1 - x, y - y0, y1 - y)


def network_facts(net, oi, box):
    """Every measured fact the review adds to the checkers. Returns (facts, candidates)."""
    F = collections.defaultdict(list)
    C = []
    counts = collections.Counter()

    def cand(kind, sev, px, py, persp, **kw):  # kw may repeat x / y (fact items carry them): the position wins
        C.append(dict(tool="llm_review", kind=kind, severity=sev, x=r3(px), y=r3(py), perspective=persp,
                      **{k: v for k, v in kw.items() if v is not None and k not in ("x", "y")}))

    road_ends = []  # (x, y, road id, end) of every road end, for "another road ends here"
    for r in net.roads.values():
        for E in ("start", "end"):
            x, y, _ = r.ref(np.array([net.end_s(r, E)]))
            road_ends.append((float(x[0]), float(y[0]), r.id, E))
    end_grid = collections.defaultdict(list)
    for e in road_ends:
        end_grid[(int(e[0] // 20), int(e[1] // 20))].append(e)

    def ends_near(x, y, radius, skip):
        out = []
        for i in range(int(x // 20) - 1, int(x // 20) + 2):
            for j in range(int(y // 20) - 1, int(y // 20) + 2):
                for e in end_grid.get((i, j), ()):
                    d = math.hypot(e[0] - x, e[1] - y)
                    if e[2] != skip and d <= radius:
                        out.append((round(d, 2), e[2], e[3]))
        return sorted(out)

    seen_joint = set()
    lanes_by_type = collections.Counter()
    length_by_type = collections.Counter()
    for r in net.roads.values():
        # ---- lanes by type, widths, slivers
        for k, sec in enumerate(r.sections):
            L = sec.e - sec.s
            if L <= 1e-6:
                continue
            ss = np.linspace(sec.s, sec.e, 9)
            for l in sec.lanes():
                if l.id == 0:
                    continue
                lanes_by_type[l.type] += 1
                length_by_type[l.type] += L
                w = l.width(ss - sec.s)
                x, y = r.point(np.array([0.5 * (sec.s + sec.e)]), np.array([0.0]))
                where = dict(road=r.id, lane=l.id, s=r3(0.5 * (sec.s + sec.e)), junction=r.junction,
                             osm_ways=r.osm_ways or None, lane_type=l.type)
                if w.min() < -1e-3:
                    cand("negative_width", "major", x[0], y[0], ["visual", "logical"], min_width=r3(w.min()), **where)
                elif w.max() < ZERO_W and L > 0.5 and l.type in MOVE:
                    counts["zero_width_lanes"] += 1
                    F["zero_width_lanes"].append(dict(where, length=r3(L), max_width=r3(w.max())))
                    cand("zero_width_lane", "minor", x[0], y[0], ["visual", "simulator"], length=r3(L),
                         max_width=r3(w.max()), **where)
                elif l.type in DRIVE and w.max() < SLIVER_M and L >= SLIVER_LEN:
                    cand("sliver_lane", "minor", x[0], y[0], ["visual", "car"], length=r3(L), max_width=r3(w.max()),
                         **where)
                elif (l.type == "driving" and r.junction == "-1" and L >= 20 and w.min() >= 0.8 * w.max()
                      and not (DRIVE_W_MIN <= float(np.median(w)) <= DRIVE_W_MAX)):
                    cand("width_implausible", "minor", x[0], y[0], ["car"], median_width=r3(np.median(w)),
                         length=r3(L), **where)
        # ---- section transitions inside the road
        for k in range(len(r.sections) - 1):
            A, B = r.sections[k], r.sections[k + 1]
            sb = B.s
            ba, bb = bands(r, A, [sb]), bands(r, B, [sb])
            x, y, _ = r.ref(np.array([sb]))
            for l in A.lanes():
                if l.id == 0 or l.type not in MOVE:
                    continue
                w = abs(float(ba[l.id][1][0] - ba[l.id][0][0]))
                m = B.lane(l.succ) if l.succ is not None else None
                where = dict(road=r.id, lane=l.id, s=r3(sb), junction=r.junction, osm_ways=r.osm_ways or None,
                             lane_type=l.type)
                if m is None:
                    if w > W_OPEN:
                        px, py = r.point(np.array([sb]), np.array([0.5 * (ba[l.id][0][0] + ba[l.id][1][0])]))
                        kind = "sidewalk_ends" if l.type in WALK else "lane_ends_mid_road"
                        sev = "minor" if l.type in WALK else "major"
                        cand(kind, sev, px[0], py[0], ["pedestrian", "wheelchair"] if l.type in WALK else ["car", "simulator"],
                             width=r3(w), **where)
                        F["open_ends"].append(dict(where, x=r3(px[0]), y=r3(py[0]), width=r3(w), how="section"))
                    elif l.type in DRIVE:
                        taper_fact(net, r, A, l, "drop", F, cand)
                    continue
                gap = max(abs(float(ba[l.id][0][0] - bb[m.id][0][0])), abs(float(ba[l.id][1][0] - bb[m.id][1][0])))
                if gap > GAP_MINOR:
                    cand("section_jump", "major" if gap > GAP_MAJOR else "minor", x[0], y[0], ["visual", "car"],
                         gap=r3(gap), to_lane=m.id, **where)
            for m in B.lanes():
                if m.id == 0 or m.type not in MOVE:
                    continue
                if m.pred is not None and A.lane(m.pred) is not None:
                    continue
                w = abs(float(bb[m.id][1][0] - bb[m.id][0][0]))
                where = dict(road=r.id, lane=m.id, s=r3(sb), junction=r.junction, osm_ways=r.osm_ways or None,
                             lane_type=m.type)
                if w > W_OPEN:
                    px, py = r.point(np.array([sb]), np.array([0.5 * (bb[m.id][0][0] + bb[m.id][1][0])]))
                    kind = "sidewalk_starts" if m.type in WALK else "lane_starts_mid_road"
                    cand(kind, "minor" if m.type in WALK else "major", px[0], py[0],
                         ["pedestrian"] if m.type in WALK else ["car", "simulator"], width=r3(w), **where)
                    F["open_ends"].append(dict(where, x=r3(px[0]), y=r3(py[0]), width=r3(w), how="section"))
                elif m.type in DRIVE:
                    taper_fact(net, r, B, m, "add", F, cand)
        # ---- road ends: dead ends, open lane ends, joints
        for E in ("start", "end"):
            k, sec = net.end_section(r, E)
            s_end = net.end_s(r, E)
            lk = net.end_link(r, E)
            b = bands(r, sec, [s_end])
            live = [l for l in sec.lanes() if l.id != 0 and l.type in MOVE
                    and abs(float(b[l.id][1][0] - b[l.id][0][0])) > W_OPEN]
            if lk is None:
                if live:
                    dead_end_fact(net, oi, box, r, E, live, F, cand, ends_near)
                continue
            for l in live:
                tg, how = net.targets(r, E, l)
                valid = []
                for (tid, cc, mid) in tg:
                    T, kT, m = net.lane_at_end(tid, cc, mid)
                    if m is None:
                        (_, _, (cx, cy), w) = net.end_points(r, E, sec, l.id)
                        cand("link_to_missing_lane", "critical", cx, cy, ["car", "simulator", "logical"], road=r.id,
                             lane=l.id, end=E, to=f"{tid}:{cc}:{mid}", junction=r.junction, lane_type=l.type,
                             osm_ways=r.osm_ways or None, spec="1.8.0 11.5")
                        continue
                    valid.append((T, kT, m, cc))
                    key = tuple(sorted([(r.id, E, l.id), (T.id, cc, m.id)]))
                    if key in seen_joint:
                        continue
                    seen_joint.add(key)
                    joint_fact(net, r, E, sec, l, T, cc, m, how, F, cand)
                if valid:
                    continue
                (_, _, (cx, cy), w) = net.end_points(r, E, sec, l.id)
                flow = net.flow_at(r, k, l, E)
                where = dict(road=r.id, lane=l.id, end=E, junction=r.junction, osm_ways=r.osm_ways or None,
                             lane_type=l.type, width=r3(w), link=f"{lk.type}:{lk.id}", flow=flow)
                F["open_ends"].append(dict(where, x=r3(cx), y=r3(cy), how=how))
                counts[f"open_end:{how}:{l.type}"] += 1
                if l.type in WALK:
                    cand("sidewalk_ends", "minor", cx, cy, ["pedestrian", "wheelchair"], **where)
                elif l.type in BIKE:
                    cand("cycle_lane_ends", "minor", cx, cy, ["cyclist"], **where)
                elif flow == "in" or flow == "both":
                    cand("lane_leads_nowhere", "critical", cx, cy, ["car", "simulator"], spec="1.8.0 11.5", **where)
                else:
                    cand("lane_unreachable_end", "major", cx, cy, ["car", "simulator"], **where)
    F["lanes_by_type"] = dict(lanes_by_type)
    F["length_by_type_km"] = {k: round(v / 1000, 2) for k, v in length_by_type.items()}
    junction_facts(net, oi, F, cand)
    crosswalk_facts(net, oi, F, cand)
    graph_facts(net, box, F, cand)
    connecting_radius_facts(net, cand)
    if oi is not None:
        osm_missing_facts(net, oi, box, F, cand)
    F["counts"] = dict(counts)
    return dict(F), C


def taper_fact(net, r, sec, l, what, F, cand):
    """a lane that starts (add) or ends (drop) at zero width: how long its taper is against the speed's need"""
    ss = np.linspace(sec.s, sec.e, max(3, int((sec.e - sec.s) / 0.5) + 1))
    w = l.width(ss - sec.s)
    wmax = float(w.max())
    if wmax < 2.0:
        return
    full = np.nonzero(w >= 0.9 * wmax)[0]
    if what == "drop":
        taper = float(ss[-1] - ss[full[-1]])
        s_at = float(ss[-1])
    else:
        taper = float(ss[full[0]] - ss[0])
        s_at = float(ss[0])
    v = net.speed.get(r.id) or 50 / 3.6
    S, W = v * 2.23694, wmax * 3.28084
    need = (W * S if S >= 45 else W * S * S / 60) / 3.28084  # MUTCD 6C.08 merging taper, metres
    item = dict(road=r.id, lane=l.id, s=r3(s_at), what=what, taper=r3(taper), width=r3(wmax),
                speed_kmh=round(v * 3.6), mutcd_merge_taper=r3(need), junction=r.junction, osm_ways=r.osm_ways or None)
    F["tapers"].append(item)
    if taper < 2 * wmax:
        x, y = r.point(np.array([s_at]), np.array([0.0]))
        cand("lane_drop_without_taper" if what == "drop" else "lane_add_without_taper", "major", x[0], y[0],
             ["car", "visual"], **item)


def joint_fact(net, r, E, sec, l, T, C, m, how, F, cand):
    """border gap and heading kink where lane l (end E of r) continues as lane m (end C of T)"""
    a_in, a_out, a_c, wa = net.end_points(r, E, sec, l.id)
    _, secT = net.end_section(T, C)
    b_in, b_out, b_c, wb = net.end_points(T, C, secT, m.id)
    gap = max(math.hypot(a_in[0] - b_in[0], a_in[1] - b_in[1]), math.hypot(a_out[0] - b_out[0], a_out[1] - b_out[1]))
    ha = math.degrees(net.heading_at(r, net.end_s(r, E)))
    hb = math.degrees(net.heading_at(T, net.end_s(T, C)))
    kink = abs(wrap_deg(ha - hb + (180.0 if E == C else 0.0)))
    where = dict(road=r.id, lane=l.id, end=E, to=f"{T.id}:{C}:{m.id}", junction=r.junction if r.junction != "-1" else T.junction,
                 osm_ways=(r.osm_ways or []) + [w for w in T.osm_ways if w not in r.osm_ways] or None, lane_type=l.type,
                 gap=r3(gap), kink_deg=round(kink, 2), widths=[r3(wa), r3(wb)], how=how)
    if gap > GAP_MINOR or kink > KINK_DEG:
        F["joints"].append(dict(where, x=r3(a_c[0]), y=r3(a_c[1])))
        sev = "major" if gap > GAP_MAJOR or kink > KINK_MAJOR_DEG else "minor"
        cand("joint_gap_or_kink", sev, a_c[0], a_c[1], ["visual", "car"], spec="1.8.0 10.3 / 11.5", **where)
    # link symmetry between plain roads (11.5: the other lane links back)
    if how == "road" and r.junction == "-1" and T.junction == "-1":
        back = net.end_link(T, C)
        mb = m.pred if C == "start" else m.succ
        if back is None or back.type != "road" or back.id != r.id or (back.contact or "start") != E or mb != l.id:
            cand("link_asymmetric", "major", a_c[0], a_c[1], ["logical", "simulator"], spec="1.8.0 10.3 / 11.5",
                 back=f"{back.type}:{back.id}:{back.contact}" if back else None, back_lane=mb, **where)


def dead_end_fact(net, oi, box, r, E, live, F, cand, ends_near):
    s = net.end_s(r, E)
    x, y, _ = r.ref(np.array([s]))
    x, y = float(x[0]), float(y[0])
    bd = boundary_dist(box, x, y)
    item = dict(road=r.id, end=E, x=r3(x), y=r3(y), junction=r.junction, osm_ways=r.osm_ways or None,
                lanes=[f"{l.id}:{l.type}" for l in live], boundary_dist=r3(bd))
    # other road ends close by, except the road's own neighbours (its links, its junctions' connecting roads):
    # those are near on a short road and say nothing about a missing link
    mine = {lk.id for lk in (r.pred, r.succ) if lk is not None}
    near = []
    for d, rid, e in ends_near(x, y, NEAR_END_M, r.id):
        o = net.roads[rid]
        ol = net.end_link(o, e)
        if rid in mine or o.junction in mine or (ol is not None and ol.id == r.id):
            continue
        near.append((d, rid, e))
    if near:
        item["road_ends_near"] = [f"{rid}:{e}@{d}m" for d, rid, e in near[:4]]
    if bd <= BOUNDARY_M:
        item["status"] = "boundary"
        F["dead_ends"].append(item)
        return
    status, why = "no_osm", ""
    if oi is not None:
        drive = any(l.type in DRIVE for l in live)
        nodes = oi.nodes_near(x, y, OSM_NODE_M)
        mine = [(d, n) for d, n in nodes if any(w in r.osm_ways for w, _, _ in oi.node_ways[n])] or nodes
        if mine:
            d, n = mine[0]
            ways = oi.node_ways[n]
            if drive:
                ways = [w for w in ways if oi.tags(w[0]).get("highway") in VEHICLE_HW | {"service", "track"}] or ways
            ntags = oi.osm.node_tags.get(n, {})
            interior = any(not is_end for _, _, is_end in ways)
            if ntags.get("noexit") == "yes" or ntags.get("highway") in ("turning_circle", "turning_loop"):
                status, why = "osm_dead_end", f"node {n} {ntags.get('highway') or 'noexit=yes'}"
            elif len({w for w, _, _ in ways}) <= 1 and not interior:
                status, why = "osm_dead_end", f"node {n} ends way {ways[0][0] if ways else '?'} with no other highway"
            else:
                status, why = "osm_continues", (f"node {n} ({d:.1f} m away) is on ways "
                                                + ",".join(sorted({w for w, _, _ in ways})))
                cont = {w for w, _, _ in ways if w not in r.osm_ways}
                own_on = any(not e for w, _, e in ways if w in r.osm_ways)
                if drive and cont and not own_on and all(oi.tags(w).get("highway") == "track" for w in cont):
                    # only a track goes on: a real road may stop here; whether to model the track is a judgement
                    status = "osm_continues_track"
                    why += " (all highway=track: " + ", ".join(
                        f"{w} motor_vehicle={oi.tags(w).get('motor_vehicle', oi.tags(w).get('access', '-'))} "
                        f"tracktype={oi.tags(w).get('tracktype', '-')}" for w in sorted(cont)) + ")"
            item["osm_node"] = n
            item["osm_node_dist"] = r3(d)
    item["status"], item["osm_why"] = status, why
    F["dead_ends"].append(item)
    drive = any(l.type in DRIVE for l in live)
    if status == "osm_continues" or (near and status != "osm_dead_end"):
        # a walking-only end (e.g. an unlinked crossing walkway) matters to pedestrians, not to traffic: minor
        cand("dead_end_osm_continues" if status == "osm_continues" else "dead_end_near_road", "critical" if drive else "minor",
             x, y, ["car", "simulator"] if drive else ["pedestrian"], **item)
    elif status == "osm_continues_track":
        cand("dead_end_track_continues", "minor", x, y, ["car", "simulator"], **item)
    elif status == "no_osm":
        cand("dead_end_unexplained", "major" if drive else "minor", x, y, ["car", "simulator"], **item)
    else:
        cand("dead_end_mapped", "info", x, y, ["car"], **item)


def junction_facts(net, oi, F, cand):
    """per default junction: attached roads, connections, turns OSM allows but nothing connects, wrong-way links"""
    attached_to = collections.defaultdict(list)
    for r in net.roads.values():
        if r.junction != "-1":
            continue
        for E in ("start", "end"):
            lk = net.end_link(r, E)
            if lk is not None and lk.type == "junction":
                attached_to[lk.id].append((r, E))
    for jid, conns in net.od.junctions.items():
        if net.jtype.get(jid) == "direct":
            continue
        attached = attached_to.get(jid, [])
        if not attached:
            continue
        pts = [r.ref(np.array([net.end_s(r, E)])) for r, E in attached]
        cx, cy = float(np.mean([p[0][0] for p in pts])), float(np.mean([p[1][0] for p in pts]))
        lanes_io = {}  # (road id, end) -> {flow: [lane ids]}; a road may meet the junction with both ends
        for r, E in attached:
            k, sec = net.end_section(r, E)
            b = bands(r, sec, [net.end_s(r, E)])
            io = collections.defaultdict(list)
            for l in sec.lanes():
                if l.id and l.type in DRIVE and abs(float(b[l.id][1][0] - b[l.id][0][0])) > W_OPEN:
                    io[net.flow_at(r, k, l, E)].append(l.id)
            lanes_io[(r.id, E)] = io
        pairs = set()
        for c in conns:
            cr = net.roads.get(c["connecting"])
            A = net.roads.get(c["incoming"])
            if cr is None or A is None or not c["lanes"]:
                continue
            C0 = c["contact"] or "start"
            other = net.end_link(cr, "end" if C0 == "start" else "start")
            if other is not None and other.type == "road":
                pairs.add((A.id, other.id))
            # the incoming road's end this connection starts from: the one the connecting road links back to
            cl = net.end_link(cr, C0)
            ends = [E for r, E in attached if r.id == A.id]
            EA = cl.contact if (cl is not None and cl.type == "road" and cl.id == A.id and cl.contact in ends) else \
                (ends[0] if len(ends) == 1 else None)
            kA = 0 if EA == "start" else len(A.sections) - 1
            kc, secc = net.end_section(cr, C0)
            for a, bl in c["lanes"]:
                la, lc = A.sections[kA].lane(a), secc.lane(bl)
                if la is None or lc is None or EA is None:
                    continue
                fa = net.flow_at(A, kA, la, EA)
                dc = net.travel(cr, kc, lc)
                away = 1 if C0 == "start" else -1
                if la.type in DRIVE and (fa == "out" or (dc not in (None, 0) and dc != away)):
                    (_, _, (px, py), _) = net.end_points(A, EA, A.sections[kA], a)
                    cand("wrong_way_link", "critical", px, py, ["car", "simulator", "logical"], road=A.id, lane=a,
                         junction=jid, connecting=cr.id, connecting_lane=bl, incoming_flow=fa,
                         osm_ways=A.osm_ways or None, spec="1.8.0 12.4")
        missing = []
        for A, EA in attached:
            ins = lanes_io[(A.id, EA)].get("in", []) + lanes_io[(A.id, EA)].get("both", [])
            if not ins:
                continue
            hA = math.degrees(net.heading_at(A, net.end_s(A, EA))) + (180.0 if EA == "start" else 0.0)
            for B, EB in attached:
                if B.id == A.id or (A.id, B.id) in pairs:
                    continue
                outs = lanes_io[(B.id, EB)].get("out", []) + lanes_io[(B.id, EB)].get("both", [])
                if not outs:
                    continue
                hB = math.degrees(net.heading_at(B, net.end_s(B, EB))) + (180.0 if EB == "end" else 0.0)
                turn = wrap_deg(hB - hA)
                if abs(turn) > TURN_UTURN_DEG:
                    continue
                restricted = ""
                if oi is not None:
                    for val, frm, to, via in oi.restrictions:
                        if not (frm & set(A.osm_ways)):
                            continue
                        vxy = oi.via_xy(via)
                        if vxy is not None and math.hypot(vxy[0] - cx, vxy[1] - cy) > 60:
                            continue
                        if val.startswith("no_") and to & set(B.osm_ways):
                            restricted = val
                        elif val.startswith("only_") and not (to & set(B.osm_ways)):
                            restricted = val
                hwA = {oi.tags(w).get("highway") for w in A.osm_ways} if oi else set()
                hwB = {oi.tags(w).get("highway") for w in B.osm_ways} if oi else set()
                item = dict(junction=jid, from_road=A.id, to_road=B.id, turn_deg=round(turn, 1),
                            from_ways=A.osm_ways, to_ways=B.osm_ways, restriction=restricted or None,
                            from_highway=sorted(h for h in hwA if h), to_highway=sorted(h for h in hwB if h))
                missing.append(item)
                if not restricted:
                    x, y, _ = A.ref(np.array([net.end_s(A, EA)]))
                    sev = "major" if abs(turn) < 30 else "minor" if abs(turn) <= SHARP_TURN_DEG else "info"
                    cand("missing_turn", sev, x[0], y[0], ["car", "simulator"],
                         road=A.id, osm_ways=(A.osm_ways or []) + (B.osm_ways or []) or None, **{k: v for k, v in item.items()
                                                                                                 if k not in ("from_ways", "to_ways")})
        F["junctions"].append(dict(id=jid, x=r3(cx), y=r3(cy), roads=[f"{r.id}:{E}" for r, E in attached],
                                   connections=len(conns), connecting_roads=len(net.in_junction.get(jid, [])),
                                   pairs=len(pairs), turns_not_connected=missing))


def crosswalk_facts(net, oi, F, cand):
    """does each crosswalk end on a walking lane (or within WALK_TOUCH of one), what does it cross"""
    for r in net.roads.values():
        objs = r.el.find("objects")
        if objs is None:
            continue
        for o in objs.findall("object"):
            if o.get("type") != "crosswalk":
                continue
            s0, t0 = float(o.get("s", 0)), float(o.get("t", 0))
            hdg = float(o.get("hdg", 0) or 0)
            cl = [(float(c.get("u", 0)), float(c.get("v", 0))) for c in o.iter("cornerLocal")]
            if cl:
                us, vs = np.array([c[0] for c in cl]), np.array([c[1] for c in cl])
            else:
                hw, hl = float(o.get("width", 0) or 0) / 2, float(o.get("length", 0) or 0) / 2
                us, vs = np.array([-hl, hl, hl, -hl]), np.array([-hw, -hw, hw, hw])
            uc = float(us.mean())

            def st(u, v):
                return s0 + u * math.cos(hdg) - v * math.sin(hdg), t0 + u * math.sin(hdg) + v * math.cos(hdg)

            ends = []
            tmid = st(uc, 0.5 * (float(vs.min()) + float(vs.max())))[1]
            for v in (float(vs.min()), float(vs.max())):
                se, te = st(uc, v)
                se = min(max(se, 0), r.length)
                px, py = r.point(np.array([se]), np.array([te]))
                e = crosswalk_end(net, oi, r, se, te, 1.0 if te >= tmid else -1.0, float(px[0]), float(py[0]))
                ends.append(dict(e, x=r3(px[0]), y=r3(py[0])))
            spans = []
            for v in np.linspace(float(vs.min()) + 0.2, float(vs.max()) - 0.2, 7):
                se, te = st(uc, v)
                se = min(max(se, 0), r.length)
                sec = r.section_at(se)
                b = bands(r, sec, [se])
                for lid, (ti, to) in b.items():
                    lo, hi = sorted((float(ti[0]), float(to[0])))
                    if lid and lo <= te <= hi:
                        spans.append(sec.lane(lid).type)
                        break
            reached = sum(1 for e in ends if e["reach"] in ("walk", "walk_via"))
            mx, my = r.point(np.array([min(max(st(uc, 0)[0], 0), r.length)]), np.array([st(uc, 0)[1]]))
            item = dict(road=r.id, object=o.get("id"), s=r3(s0), x=r3(mx[0]), y=r3(my[0]), junction=r.junction,
                        osm_ways=r.osm_ways or None, ends=ends, ends_on_walk=reached,
                        crosses=[k for k, _ in collections.Counter(spans).most_common()])
            if oi is not None:
                near = [(d, n) for d, n in oi.nodes_near(float(mx[0]), float(my[0]), 15)
                        if oi.osm.node_tags.get(n, {}).get("highway") == "crossing"]
                item["osm_crossing_node"] = near[0][1] if near else None
            F["crosswalks"].append(item)
            reach = {e["reach"] for e in ends}
            if reach & {"driving_between", "nothing"}:
                cand("crosswalk_unreached", "major", mx[0], my[0], ["pedestrian", "wheelchair"],
                     spec="JG-1.0.0 10 / plausibility", **item)
            elif "footway_only" in reach:
                cand("crosswalk_to_unmodelled_footway", "minor", mx[0], my[0], ["pedestrian", "simulator"], **item)
    # one crossing written twice: crosswalk centres closer than DUP_CROSSWALK_M (any roads)
    cws = F.get("crosswalks", [])
    grid = collections.defaultdict(list)
    for i, c in enumerate(cws):
        grid[(int(c["x"] // 5), int(c["y"] // 5))].append(i)
    for i, c in enumerate(cws):
        for a in range(int(c["x"] // 5) - 1, int(c["x"] // 5) + 2):
            for b in range(int(c["y"] // 5) - 1, int(c["y"] // 5) + 2):
                for j in grid.get((a, b), ()):
                    o = cws[j]
                    d = math.hypot(o["x"] - c["x"], o["y"] - c["y"])
                    if j > i and d <= DUP_CROSSWALK_M:
                        cand("duplicate_crosswalk", "minor", c["x"], c["y"], ["visual", "simulator", "logical"],
                             road=c["road"], junction=c["junction"], osm_ways=c.get("osm_ways"),
                             objects=[f"{c['road']}:{c['object']}", f"{o['road']}:{o['object']}"], distance=r3(d))


# lanes a pedestrian may cross between a crosswalk's end and the walking lane (parking is banned next to crossings)
WALK_VIA = {"curb", "border", "shoulder", "parking", "biking", "median", "restricted", "none"}
WALK_VIA_M = 8.0
DUP_CROSSWALK_M = 1.0  # two crosswalk objects whose centres are this close are one crossing written twice
FOOTWAY_M = 8.0  # a separately mapped sidewalk's centre line lies up to about this far beyond the carriageway edge


def crosswalk_end(net, oi, r, s, t, out, x, y):
    """what a crosswalk end at (s, t) of road r reaches going outward (out = +1 / -1 in t): a walking lane directly
    ('walk'), across lanes a pedestrian may cross ('walk_via'), only across driving lanes ('driving_between'), only an
    OSM footway the network does not model ('footway_only'), or nothing"""
    sec = r.section_at(s)
    b = bands(r, sec, [s])
    lanes = []
    for lid, (ti, to) in b.items():
        if lid:
            lo, hi = sorted((float(ti[0]), float(to[0])))
            if hi - lo > 0.05:
                lanes.append((lo, hi, sec.lane(lid).type))
    lanes.sort(key=lambda q: q[0] * out)
    via, res = [], None
    for lo, hi, typ in lanes:
        near_edge, far_edge = (lo, hi) if out > 0 else (hi, lo)
        if (far_edge - t) * out <= 0.05:  # behind the end
            continue
        if typ in WALK:
            d = max(0.0, (near_edge - t) * out)
            res = dict(reach="walk" if d <= WALK_TOUCH else "walk_via" if set(via) <= WALK_VIA and d <= WALK_VIA_M
                       else "driving_between", walk_dist=r3(d), via=via)
            break
        via.append(typ)
    if res is None:
        # off this road's edge: a walking lane of another road (a junction corner, a separate sidewalk road)
        edge = max((q[1] for q in lanes), default=t) if out > 0 else min((q[0] for q in lanes), default=t)
        ex, ey = r.point(np.array([s]), np.array([edge + 0.3 * out]))
        d, at = walk_distance(net, float(ex[0]), float(ey[0]), skip=r.id)
        edge_d = abs(edge - t)
        if d is not None and d + edge_d <= WALK_VIA_M and set(via) <= WALK_VIA:
            res = dict(reach="walk" if d + edge_d <= WALK_TOUCH else "walk_via", walk_dist=r3(d + edge_d), via=via,
                       other_road=at)
        else:
            res = dict(reach="driving_between" if set(via) - WALK_VIA else "nothing", walk_dist=r3(d), via=via)
    if oi is not None:
        fd, fw = oi.foot_dist(x, y)
        res["osm_footway_dist"], res["osm_footway"] = fd, fw
        if res["reach"] == "nothing" and fd is not None and fd <= FOOTWAY_M:
            res["reach"] = "footway_only"
    return res


def walk_distance(net, x, y, radius=12.0, skip=None):
    """distance from a point to the nearest walking lane (0 inside one), and the road:lane:type it is on"""
    best, at = None, None
    for r in net.od.roads_near(x, y, radius):
        if r.id == skip:
            continue
        s, t, _ = r.project(x, y)
        rx, ry, rh = (float(v[0]) for v in r.ref(np.array([s])))
        along = (x - rx) * math.cos(rh) + (y - ry) * math.sin(rh)
        if abs(along) > 0.5:
            continue
        sec = r.section_at(s)
        b = bands(r, sec, [s])
        for lid, (ti, to) in b.items():
            if not lid:
                continue
            lo, hi = sorted((float(ti[0]), float(to[0])))
            ln = sec.lane(lid)
            if ln.type in WALK and hi - lo > 0.1:
                d = 0.0 if lo - 0.05 <= t <= hi + 0.05 else min(abs(t - lo), abs(t - hi))
                if best is None or d < best:
                    best, at = d, f"{r.id}:{lid}:{ln.type}"
    return best, at


def graph_facts(net, box, F, cand):
    """driving-lane graph: nodes (road, section, lane), edges in the travel direction. Strongly connected
    components, closed loops a vehicle cannot leave, islands it cannot enter, and the largest component's share."""
    nodes, length = {}, {}
    for r in net.roads.values():
        for k, sec in enumerate(r.sections):
            for l in sec.lanes():
                if l.id and l.type in DRIVE:
                    d = net.travel(r, k, l)
                    if d is not None:
                        nodes[(r.id, k, l.id)] = d
                        length[(r.id, k, l.id)] = sec.e - sec.s
    adj = collections.defaultdict(set)
    wrong = set()

    def step(n, d):
        rid, k, lid = n
        r = net.roads[rid]
        sec = r.sections[k]
        l = sec.lane(lid)
        out = []
        last = len(r.sections) - 1
        if d == 1 and k < last:
            m = r.sections[k + 1].lane(l.succ) if l.succ is not None else None
            return [(rid, k + 1, m.id)] if m is not None else []
        if d == -1 and k > 0:
            m = r.sections[k - 1].lane(l.pred) if l.pred is not None else None
            return [(rid, k - 1, m.id)] if m is not None else []
        E = "end" if d == 1 else "start"
        for tid, C, mid in net.targets(r, E, l)[0]:
            T, kT, m = net.lane_at_end(tid, C, mid)
            if m is None or (T.id, kT, m.id) not in nodes:
                continue
            dT = nodes[(T.id, kT, m.id)]
            if dT == 0 or dT == (1 if C == "start" else -1):
                out.append((T.id, kT, m.id))
            else:  # the link sends the lane into a lane that runs toward it
                wrong.add((n, E, (T.id, kT, m.id)))
        return out

    open_fwd = set()  # lane pieces whose end (in a travel direction) no link continues; lane changes do not count
    for n, d in nodes.items():
        for dd in ((1, -1) if d == 0 else (d,)):
            nx = step(n, dd)
            adj[n].update(nx)
            if not nx:
                open_fwd.add(n)
    # lane changes: adjacent lanes of one side that travel the same way (turn bays, tapers are entered this way)
    for r in net.roads.values():
        for k, sec in enumerate(r.sections):
            for side in (sec.left, sec.right):
                for a, b in zip(side, side[1:]):
                    na, nb = (r.id, k, a.id), (r.id, k, b.id)
                    if na in nodes and nb in nodes and nodes[na] == nodes[nb]:
                        adj[na].add(nb)
                        adj[nb].add(na)
    said = set()
    for (rid, k, lid), E, to in sorted(wrong):
        if frozenset([(rid, k, lid), to]) in said:  # both lanes see the same wrong link
            continue
        said.add(frozenset([(rid, k, lid), to]))
        r = net.roads[rid]
        (_, _, (px, py), _) = net.end_points(r, E, r.sections[k], lid)
        cand("wrong_way_link", "critical", px, py, ["car", "simulator", "logical"], road=rid, lane=lid, end=E,
             to=f"{to[0]}:{to[2]}", junction=r.junction, osm_ways=r.osm_ways or None, spec="1.8.0 11.5")
    # iterative Tarjan
    index, low, onst, st, comp = {}, {}, set(), [], {}
    counter = 0
    ncomp = 0
    for root in nodes:
        if root in index:
            continue
        work = [(root, iter(adj[root]))]
        index[root] = low[root] = counter
        counter += 1
        st.append(root)
        onst.add(root)
        while work:
            v, it = work[-1]
            advanced = False
            for w in it:
                if w not in index:
                    index[w] = low[w] = counter
                    counter += 1
                    st.append(w)
                    onst.add(w)
                    work.append((w, iter(adj[w])))
                    advanced = True
                    break
                if w in onst:
                    low[v] = min(low[v], index[w])
            if advanced:
                continue
            work.pop()
            if work:
                low[work[-1][0]] = min(low[work[-1][0]], low[v])
            if low[v] == index[v]:
                while True:
                    w = st.pop()
                    onst.discard(w)
                    comp[w] = ncomp
                    if w == v:
                        break
                ncomp += 1
    members = collections.defaultdict(list)
    for n, c in comp.items():
        members[c].append(n)
    has_out, has_in = set(), set()
    for v, ws in adj.items():
        for w in ws:
            if comp[v] != comp[w]:
                has_out.add(comp[v])
                has_in.add(comp[w])
    comp_len = {c: sum(length[n] for n in ms) for c, ms in members.items()}
    total = sum(comp_len.values()) or 1.0
    big = max(comp_len, key=comp_len.get) if comp_len else None
    traps = []
    for c, ms in members.items():
        if len(ms) < 2 or c in has_out or any(n in open_fwd for n in ms):
            continue
        rid, k, lid = ms[0]
        r = net.roads[rid]
        x, y = r.point(np.array([0.5 * (r.sections[k].s + r.sections[k].e)]), np.array([0.0]))
        item = dict(road=rid, lane=lid, lanes=len(ms), length=r3(comp_len[c]), x=r3(x[0]), y=r3(y[0]),
                    roads=sorted({n[0] for n in ms})[:12], osm_ways=r.osm_ways or None)
        traps.append(item)
        cand("closed_loop_trap", "critical", x[0], y[0], ["car", "simulator"], **item)
    # can a vehicle keep driving (reach a loop or leave the area where it is clipped / where OSM ends the road)?
    # can it get here (from such an entry or a loop)? Open ends elsewhere are dead ends.
    legit = {(d["road"], d["end"]) for d in F.get("dead_ends", []) if d["status"] in ("boundary", "osm_dead_end")}

    def at_legit_end(n, forward):
        rid, k, lid = n
        r = net.roads[rid]
        d = nodes[n]
        if forward:
            E = "end" if d >= 0 and k == len(r.sections) - 1 else "start" if d <= 0 and k == 0 else None
        else:
            E = "start" if d >= 0 and k == 0 else "end" if d <= 0 and k == len(r.sections) - 1 else None
        return E is not None and (rid, E) in legit

    rev = collections.defaultdict(set)
    for v, ws in adj.items():
        for w in ws:
            rev[w].add(v)
    looped = {n for c, ms in members.items() if len(ms) > 1 for n in ms}

    def closure(seed, graph):
        seen, todo = set(seed), list(seed)
        while todo:
            v = todo.pop()
            for w in graph[v]:
                if w not in seen:
                    seen.add(w)
                    todo.append(w)
        return seen

    can_go = closure(looped | {n for n in open_fwd if at_legit_end(n, True)}, rev)
    entries = {n for n in nodes if not rev[n] and at_legit_end(n, False)}
    reached = closure(looped | entries, adj)
    stuck = [n for n in nodes if n not in can_go]
    unreach = [n for n in nodes if n not in reached]
    F["drive_graph"] = dict(nodes=len(nodes), components=ncomp, largest_scc_share=round(comp_len.get(big, 0) / total, 4),
                            largest_scc_km=round(comp_len.get(big, 0) / 1000, 2), traps=traps,
                            open_forward_ends=len(open_fwd),
                            stuck_share=round(sum(length[n] for n in stuck) / total, 4), stuck_pieces=len(stuck),
                            unreachable_share=round(sum(length[n] for n in unreach) / total, 4),
                            unreachable_pieces=len(unreach))


def curvature(r, window=1.0):
    """reference-line curvature at r.S from the geometry records (exact for line, arc, spiral; numeric for
    paramPoly3), averaged over `window` metres so a single-sample spike at an element joint is not a turn"""
    k = np.zeros_like(r.S)
    num = np.gradient(r.H, r.S) if len(r.S) > 2 else np.zeros_like(r.S)
    for s0, L, tag, at in r.geoms:
        m = (r.S >= s0 - 1e-9) & (r.S <= s0 + L + 1e-9)
        if tag == "line":
            k[m] = 0.0
        elif tag == "arc":
            k[m] = float(at.get("curvature", 0))
        elif tag == "spiral" and L > 0:
            k0, k1 = float(at.get("curvStart", 0)), float(at.get("curvEnd", 0))
            k[m] = k0 + (k1 - k0) * (r.S[m] - s0) / L
        else:
            k[m] = num[m]
    n = max(1, int(round(window / G.DENSE)))
    return np.convolve(k, np.ones(n) / n, mode="same") if len(k) > n else k


def connecting_radius_facts(net, cand):
    """smallest turning radius on the centre of each driving lane of a connecting road (a car's rear axle cannot
    follow a lane centre tighter than about TIGHT_R)"""
    for r in net.od.connecting_roads():
        if r.length < 2.0 or len(r.S) < 5:
            continue
        S = r.S
        kap = curvature(r)
        worst = None
        for sec in r.sections:
            sm = (S >= sec.s) & (S <= sec.e)
            if not sm.any():
                continue
            b = bands(r, sec, S[sm])
            for l in sec.lanes():
                if not l.id or l.type not in DRIVE:
                    continue
                tc = 0.5 * (b[l.id][0] + b[l.id][1])
                k = kap[sm]
                with np.errstate(divide="ignore"):
                    rad = np.abs(np.where(np.abs(k) > 1e-9, 1.0 / k, np.inf) - tc)  # offset curve radius
                i = int(np.argmin(rad))
                if worst is None or rad[i] < worst[0]:
                    worst = (float(rad[i]), float(S[sm][i]), l.id, float(tc[i]))
        if worst and worst[0] < TIGHT_R:
            x, y = r.point(np.array([worst[1]]), np.array([worst[3]]))
            cand("tight_turn", "major" if worst[0] < TIGHT_R_MAJOR else "minor", x[0], y[0], ["car", "simulator"],
                 road=r.id, lane=worst[2], junction=r.junction,
                 s=r3(worst[1]), lane_centre_radius=r3(worst[0]), length=r3(r.length), osm_ways=r.osm_ways or None)


def osm_missing_facts(net, oi, box, F, cand):
    have = {w for r in net.roads.values() for w in r.osm_ways}
    x0, y0, x1, y1 = box
    for wid, (nds, tags) in oi.hw.items():
        hw = tags.get("highway")
        if wid in have or tags.get("area") == "yes" or hw not in VEHICLE_HW | {"service"}:
            continue
        pl = oi.osm.polyline(wid)
        if len(pl) < 2:
            continue
        inside = (pl[:, 0] > x0 + 5) & (pl[:, 0] < x1 - 5) & (pl[:, 1] > y0 + 5) & (pl[:, 1] < y1 - 5)
        seg = np.hypot(np.diff(pl[:, 0]), np.diff(pl[:, 1]))
        L = float((seg * (inside[1:] & inside[:-1])).sum())
        if L < 30:
            continue
        mid = pl[inside][len(pl[inside]) // 2]
        item = dict(way=wid, highway=hw, length_inside=r3(L), x=r3(mid[0]), y=r3(mid[1]), osm_ways=[wid],
                    access=tags.get("access"), name=tags.get("name"))
        F["osm_ways_missing"].append(item)
        if hw != "service":
            cand("osm_way_missing", "major" if hw in MAJOR_HW else "minor", mid[0], mid[1], ["car", "logical"], **item)


# ----------------------------------------------------------------------------- the checkers' findings
def kind_of(f):
    k = f.get("kind") or f.get("type") or f.get("check") or "?"
    sub = f.get("subtype") or f.get("sub") or f.get("flow")
    return f"{k}" + (f":{sub}" if sub else "")


def norm_sev(v):
    v = (v or "").lower()
    return v if v in SEV_W else {"defect": "major", "warning": "minor", "error": "major"}.get(v, "minor")


def compact(f, skip):
    """the scalar fields of a finding (and of nested dicts such as edge_smoothness's 'metrics'), at most 16"""
    out = {}
    items = list(f.items())
    for k, v in items:
        if isinstance(v, dict):
            items += [(f"{k}.{kk}", vv) for kk, vv in v.items()]
    for k, v in items:
        if k in skip or len(out) >= 16:
            continue
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            out[k] = round(v, 3) if isinstance(v, float) else v
        elif isinstance(v, str) and len(v) <= 60:
            out[k] = v
        elif isinstance(v, list) and 0 < len(v) <= 8 and all(isinstance(q, (int, float, str)) for q in v):
            out[k] = v  # short id lists, e.g. the lanes a finding is about
    return out


def mark_helper_roads(net, cands):
    """a finding on a road without lanes (a 1.8.0 junction boundary segment and the like) says nothing a road user
    meets: it stays a candidate, marked, and counts as info when spots are chosen"""
    helpers = {r.id for r in net.roads.values() if not any(l.id for sec in r.sections for l in sec.lanes())}
    for c in cands:
        if c.get("road") in helpers:
            c["on_lane_less_road"] = True


SKIP = {"x", "y", "id", "kind", "type", "check", "subtype", "sub", "flow", "severity", "road", "roads", "lane",
        "junction", "junction_id", "s", "osm_ways", "way"}


def checker_candidates(cdir):
    """normalised findings of every checker result found under cdir (gate layout: <cdir>/<tool>/findings.json)"""
    out, used = [], []
    for name in DETECTORS:
        f = os.path.join(cdir, name, "findings.json")
        if not os.path.exists(f):
            continue
        used.append(name)
        d = load(f)
        items = d["findings"] if isinstance(d, dict) else d
        for i, it in enumerate(items):
            if it.get("x") is None or it.get("y") is None:
                continue
            roads = it.get("roads") or ([it["road"]] if it.get("road") is not None else [])
            ways = it.get("osm_ways") or ([it["way"]] if it.get("way") else [])
            out.append(dict(id=f"{name}:{it.get('id', i)}", tool=name, kind=kind_of(it), severity=norm_sev(it.get("severity")),
                            x=r3(it["x"]), y=r3(it["y"]), road=str(roads[0]) if roads else None,
                            roads=[str(r) for r in roads][:6], lane=it.get("lane"),
                            junction=str(it.get("junction") or it.get("junction_id") or "") or None, s=it.get("s"),
                            osm_ways=[str(w) for w in ways] or None, metric=compact(it, SKIP)))
    f = os.path.join(cdir, "exact_invariants", "invariants.json")
    if os.path.exists(f):
        used.append("invariants")
        d = load(f)
        for name, inv in d.get("invariants", {}).items():
            sev = INV_SEV.get(name) or ("info" if name.startswith("G2") else
                                        "major" if inv.get("level", "").startswith("MUST") else "minor")
            for i, it in enumerate(inv.get("instances", [])):
                if it.get("x") is None:
                    continue
                out.append(dict(id=f"invariants:{name}:{i}", tool="invariants", kind=name, severity=sev,
                                x=r3(it["x"]), y=r3(it["y"]), road=str(it.get("road")), roads=[str(it.get("road"))],
                                lane=it.get("lane"), junction=it.get("junction"), s=it.get("s"),
                                osm_ways=[str(it["way"])] if it.get("way") else ([str(it["on_way"])] if it.get("on_way") else None),
                                metric=compact(it, SKIP), spec=inv.get("spec")))
    f = os.path.join(cdir, "exact_laneflow", "findings.json")
    if os.path.exists(f):
        used.append("laneflow")
        d = load(f)
        for name, v in d.items():
            if not isinstance(v, dict) or "instances" not in v or v.get("severity") == "ok":
                continue
            sev = norm_sev(v.get("severity"))
            for i, it in enumerate(v["instances"]):
                if it.get("x") is None:
                    continue
                out.append(dict(id=f"laneflow:{name}:{i}", tool="laneflow", kind=name, severity=sev, x=r3(it["x"]),
                                y=r3(it["y"]), road=str(it.get("road")), roads=[str(it.get("road"))], lane=it.get("lane"),
                                junction=it.get("junction"), s=it.get("s"), osm_ways=None, metric=compact(it, SKIP),
                                spec=v.get("spec")))
    return out, used


def run_checkers(xodr, osm, clip, out, py, missing_only=False):
    """run the detectors and the exact checkers into out/ (gate layout), in parallel, with the checkers' Python
    (they need what the gate's venv has, e.g. shapely). A checker succeeded when it wrote its result file (the exact
    checkers exit 1 when they find defects); otherwise its error is recorded and shown to the reviewer."""
    jobs = []
    for name in DETECTORS:
        jobs.append((name, os.path.join(out, name, "findings.json"),
                     [py, os.path.join(HERE, f"det_{name}.py"), xodr, osm, "--out", os.path.join(out, name), "--crops", "0"]))
    for tool in ("invariants", "laneflow"):
        res = os.path.join(out, "exact_" + tool, "invariants.json" if tool == "invariants" else "findings.json")
        cmd = [py, os.path.join(HERE, tool + ".py"), xodr, osm, "--out", os.path.join(out, "exact_" + tool), "--crops", "0"]
        if clip:
            cmd += ["--clip", clip]
        jobs.append((tool, res, cmd))

    def go(job):
        name, res, cmd = job
        t = time.time()
        r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
        ok = os.path.exists(res)
        err = "" if ok else ((r.stderr or r.stdout or "").strip().splitlines() or ["no output"])[-1][-300:]
        return dict(tool=name, rc=r.returncode, ok=ok, seconds=round(time.time() - t), err=err)

    if missing_only:
        jobs = [j for j in jobs if not os.path.exists(j[1])]
    with ThreadPoolExecutor(4) as ex:
        return list(ex.map(go, jobs))


# ----------------------------------------------------------------------------- spots and chunks
def cand_weight(c):
    """how much a candidate counts for choosing and ordering: its severity, as info on a road without lanes"""
    return SEV_W["info"] if c.get("on_lane_less_road") else SEV_W[c["severity"]]


def select_spots(cands, n_spots, join=30.0):
    """cluster candidates into spots (greedy, strongest first) and pick n_spots, round-robin over the spots'
    leading kinds so every kind of problem gets looked at before any kind gets a second look"""
    weight = cand_weight
    order = sorted(range(len(cands)), key=lambda i: (-weight(cands[i]), cands[i]["id"]))
    spots, grid = [], collections.defaultdict(list)
    for i in order:
        c = cands[i]
        x, y = c["x"], c["y"]
        gi, gj = int(x // join), int(y // join)
        hit = None
        for a in range(gi - 1, gi + 2):
            for b in range(gj - 1, gj + 2):
                for si in grid.get((a, b), ()):
                    sp = spots[si]
                    if math.hypot(sp["x"] - x, sp["y"] - y) <= join:
                        hit = si
                        break
                if hit is not None:
                    break
            if hit is not None:
                break
        if hit is None:
            spots.append(dict(x=x, y=y, members=[i], lead=f"{c['tool']}:{c['kind']}"))
            grid[(gi, gj)].append(len(spots) - 1)
        else:
            spots[hit]["members"].append(i)
    for sp in spots:
        best = {}
        for i in sp["members"]:
            key = (cands[i]["tool"], cands[i]["kind"])
            best[key] = max(best.get(key, 0), weight(cands[i]))
        sp["score"] = round(sum(best.values()) + 0.05 * min(len(sp["members"]), 40), 2)
    by_lead = collections.defaultdict(list)
    for si, sp in enumerate(spots):
        by_lead[sp["lead"]].append(si)
    for k in by_lead:
        by_lead[k].sort(key=lambda si: -spots[si]["score"])
    leads = sorted(by_lead, key=lambda k: -spots[by_lead[k][0]]["score"])
    # a cluster whose centre another picked spot already shows is skipped (its candidates are in that view)
    picked, cursor = [], {k: 0 for k in leads}
    while len(picked) < n_spots and any(cursor[k] < len(by_lead[k]) for k in leads):
        for k in leads:
            while cursor[k] < len(by_lead[k]) and len(picked) < n_spots:
                si = by_lead[k][cursor[k]]
                cursor[k] += 1
                if all(math.hypot(spots[si]["x"] - spots[p]["x"], spots[si]["y"] - spots[p]["y"]) > 1.5 * join
                       for p in picked):
                    picked.append(si)
                    break
    return [spots[i] for i in picked], spots


def sample_junctions(facts, cands, spots, n, seed):
    """ordinary junctions, so unknown issues can surface: not in a chosen spot, from the least flagged third (sum of
    candidate weights within 40 m, info excluded), spread over junction sizes"""
    if n <= 0:
        return []
    grid = collections.defaultdict(list)
    for c in cands:
        if c["severity"] != "info":
            grid[(int(c["x"] // 40), int(c["y"] // 40))].append(c)

    def weight(j):
        w = 0.0
        for a in range(int(j["x"] // 40) - 1, int(j["x"] // 40) + 2):
            for b in range(int(j["y"] // 40) - 1, int(j["y"] // 40) + 2):
                w += sum(SEV_W[c["severity"]] for c in grid.get((a, b), ())
                         if math.hypot(c["x"] - j["x"], c["y"] - j["y"]) <= 40)
        return w

    pool = [j for j in facts.get("junctions", []) if j["connections"] >= 2
            and all(math.hypot(j["x"] - s["x"], j["y"] - s["y"]) > s["half"] + 20 for s in spots)]
    if not pool:
        return []
    pool.sort(key=lambda j: (weight(j), hashlib.md5((seed + j["id"]).encode()).hexdigest()))
    quiet = sorted(pool[:max(n, len(pool) // 3)], key=lambda j: (j["connections"], j["id"]))
    idx = sorted({int(round(q)) for q in np.linspace(0, len(quiet) - 1, min(n, len(quiet)))})
    return [quiet[i] for i in idx]


def roads_in_view(net, cx, cy, half, limit=30):
    out = []
    for r in net.od.roads_near(cx, cy, half * 1.2):
        d = np.hypot(r.X - cx, r.Y - cy)
        if d.min() <= half:
            out.append((float(d.min()), r))
    return [r for _, r in sorted(out, key=lambda t: t[0])[:limit]]


def road_lines(net, oi, r, cx, cy, half):
    """compact text of a road: kind, length, OSM, links, and the lanes at each end inside the view"""
    kind = f"junction {r.junction}" if r.junction != "-1" else "plain"
    if not any(l.id for sec in r.sections for l in sec.lanes()):
        kind += f", NO LANES: a helper road ('{r.name}', e.g. a 1.8.0 junction boundary segment), not drivable"
    sp = net.speed.get(r.id)
    z0, z1 = (float(v) for v in r.z(np.array([0.0, r.length])))
    head = (f"road {r.id} ({kind}, {r.length:.1f} m, {net.rule[r.id]}"
            + (f", {sp * 3.6:.0f} km/h" if sp else "") + (f", z {z0:.1f}->{z1:.1f} m" if r.elev else "") + ")"
            + (" OSM " + "; ".join(way_label(oi, w) for w in r.osm_ways[:3]) if r.osm_ways else " OSM -"))
    lines = [head]
    for E in ("start", "end"):
        s = net.end_s(r, E)
        x, y, _ = r.ref(np.array([s]))
        if abs(float(x[0]) - cx) > half or abs(float(y[0]) - cy) > half:
            continue
        lk = net.end_link(r, E)
        k, sec = net.end_section(r, E)
        b = bands(r, sec, [s])
        parts = []
        for l in sec.lanes():
            if not l.id:
                continue
            w = abs(float(b[l.id][1][0] - b[l.id][0][0]))
            tg, how = net.targets(r, E, l) if lk is not None else ([], "none")
            fl = net.flow_at(r, k, l, E)
            dest = ",".join(f"{t}:{m}" for t, _, m in tg[:4]) or ("OPEN" if w > W_OPEN and l.type in MOVE else "-")
            parts.append(f"{l.id} {l.type} {w:.2f}m{(' ' + fl) if fl else ''} -> {dest}")
        link = f"{lk.type} {lk.id}" + (f" ({lk.contact})" if lk.contact else "") if lk else "NONE"
        lines.append(f"  {E} @({float(x[0]):.1f},{float(y[0]):.1f}) link {link}: " + "; ".join(parts))
    if len(r.sections) > 1:
        lines.append("  sections at s=" + ", ".join(
            f"{sec.s:.1f} ({sum(1 for l in sec.lanes() if l.id and l.type in DRIVE)} driving of "
            f"{sum(1 for l in sec.lanes() if l.id)} lanes)" for sec in r.sections[:8]))
    return lines


def in_box(item, cx, cy, half):
    return item.get("x") is not None and abs(item["x"] - cx) <= half and abs(item["y"] - cy) <= half


# ----------------------------------------------------------------------------- rendering
LANE_COLORS = {"driving": "#3c4658", "walking": "#bdbdbd", "sidewalk": "#bdbdbd", "biking": "#2e8b57",
               "parking": "#3f6fa8", "shoulder": "#b49a6a", "border": "#c8b48a", "curb": "#c8b48a",
               "median": "#8a5aa8", "restricted": "#8a5aa8", "bus": "#7a3b3b", "entry": "#4a5670", "exit": "#4a5670",
               "onRamp": "#4a5670", "offRamp": "#4a5670", "connectingRamp": "#4a5670", "slipLane": "#4a5670"}


def basemap_image(basemap, x0, y0, x1, y1):
    """(RGB array, extent) of the finest basemap level over a box, satellite layer first"""
    from PIL import Image
    meta = load(os.path.join(basemap, "basemap.json"))
    layers = {l["id"]: l for l in meta["layers"]}
    lay = next((layers[k] for k in ("naip", "sat", "aerial", "map") if k in layers), meta["layers"][0])
    res, T = lay["res"], lay["tile"]
    lx0, ly1 = lay["x0"], lay["y1"]
    L0 = lay["levels"][0]
    i0, i1 = max(0, int((x0 - lx0) / (T * res))), min(L0["nx"] - 1, int((x1 - lx0) / (T * res)))
    j0, j1 = max(0, int((ly1 - y1) / (T * res))), min(L0["ny"] - 1, int((ly1 - y0) / (T * res)))
    if i1 < i0 or j1 < j0:
        return None
    canvas = Image.new("RGB", ((i1 - i0 + 1) * T, (j1 - j0 + 1) * T), (242, 239, 233))
    any_tile = False
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            f = os.path.join(basemap, lay["dir"], "0", f"{i}_{j}.{lay['ext']}")
            if os.path.exists(f):
                canvas.paste(Image.open(f).convert("RGB").resize((T, T)), ((i - i0) * T, (j - j0) * T))
                any_tile = True
    if not any_tile:
        return None
    ext = (lx0 + i0 * T * res, lx0 + (i1 + 1) * T * res, ly1 - (j1 + 1) * T * res, ly1 - j0 * T * res)
    return np.asarray(canvas), ext, lay["id"]


def render(net, oi, spot, path, basemap=None, open_ends=(), crosswalks=(), title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection, PolyCollection
    from matplotlib.patches import Polygon as MPoly

    cx, cy, half = spot["x"], spot["y"], spot["half"]
    x0, x1, y0, y1 = cx - half, cx + half, cy - half, cy + half
    fig = plt.figure(figsize=(9, 9), dpi=100)
    ax = fig.add_axes([0.07, 0.05, 0.91, 0.9])
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    bg = None
    if basemap:
        try:
            bg = basemap_image(basemap, x0, y0, x1, y1)
        except Exception:
            bg = None
    if bg is not None:
        ax.imshow(bg[0], extent=bg[1], alpha=0.55, zorder=0, interpolation="bilinear")
    else:
        ax.set_facecolor("#f2efe9")
    step = max(0.4, half / 120)
    polys, cols, borders, refl, conn = [], [], [], [], []
    arrows = []
    for r in net.od.roads_near(cx, cy, half * 1.5):
        if not ((r.X >= x0 - 20) & (r.X <= x1 + 20) & (r.Y >= y0 - 20) & (r.Y <= y1 + 20)).any():
            continue
        jn = r.junction != "-1"
        for k, sec in enumerate(r.sections):
            if sec.e - sec.s <= 1e-6:
                continue
            s = np.linspace(sec.s, sec.e, max(2, int((sec.e - sec.s) / step) + 1))
            b = bands(r, sec, s)
            for l in sec.lanes():
                if not l.id:
                    continue
                ti, to = b[l.id]
                xa, ya = r.point(s, ti)
                xb, yb = r.point(s, to)
                polys.append(np.column_stack([np.concatenate([xa, xb[::-1]]), np.concatenate([ya, yb[::-1]])]))
                cols.append(matplotlib.colors.to_rgba(LANE_COLORS.get(l.type, "#a07090"), 0.35 if jn else 0.75))
                borders.append(np.column_stack([xb, yb]))
                if l.type in DRIVE:
                    tm = 0.5 * (ti + to)
                    xm, ym = r.point(s, tm)
                    if jn:
                        conn.append(np.column_stack([xm, ym]))
                    d = net.travel(r, k, l)
                    if d and len(s) >= 3:  # an arrow every ~third of the view along the lane
                        every = max(1, int(max(8.0, half / 3) / step))
                        for i in range(len(s) // 2 % every, len(s), every):
                            j = min(len(s) - 1, i + 1) if d == 1 else max(0, i - 1)
                            if i != j:
                                arrows.append((xm[i], ym[i], xm[j], ym[j]))
        xr, yr, _ = r.ref(np.linspace(0, r.length, max(2, int(r.length / step))))
        refl.append(np.column_stack([xr, yr]))
    ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors="none", zorder=2))
    ax.add_collection(LineCollection(borders, colors="white", linewidths=0.5, alpha=0.8, zorder=3))
    ax.add_collection(LineCollection(refl, colors="#e628c8", linewidths=0.7, alpha=0.8, zorder=4))
    if conn:
        ax.add_collection(LineCollection(conn, colors="#ff8c00", linewidths=0.9, alpha=0.9, zorder=5))
    alen = max(1.5, half / 25)
    for (ax0, ay0, ax1, ay1) in arrows:
        if not (x0 <= ax0 <= x1 and y0 <= ay0 <= y1):
            continue
        dx, dy = ax1 - ax0, ay1 - ay0
        n = math.hypot(dx, dy) or 1
        ax.annotate("", xy=(ax0 + dx / n * alen, ay0 + dy / n * alen), xytext=(ax0, ay0),
                    arrowprops=dict(arrowstyle="-|>", color="black", lw=0.7, mutation_scale=7), zorder=6)
    for cw in crosswalks:
        pts = cw.get("_poly")
        if pts:
            ax.add_patch(MPoly(pts, closed=True, fill=False, hatch="////", edgecolor="white", lw=0.8, zorder=6))
    if oi is not None:
        labels = 0
        for wid, (nds, tags) in oi.hw.items():
            pl = oi.osm.polyline(wid)
            if len(pl) < 2:
                continue
            m = (pl[:, 0] >= x0 - 30) & (pl[:, 0] <= x1 + 30) & (pl[:, 1] >= y0 - 30) & (pl[:, 1] <= y1 + 30)
            if not m.any():
                continue
            foot = tags.get("highway") in FOOT_HW
            ax.plot(pl[:, 0], pl[:, 1], linestyle=":" if foot else "--", color="#00b8e6", lw=0.9 if foot else 1.3,
                    alpha=0.9, zorder=7)
            if not foot and labels < 30:
                inside = pl[(pl[:, 0] >= x0) & (pl[:, 0] <= x1) & (pl[:, 1] >= y0) & (pl[:, 1] <= y1)]
                if len(inside):
                    p = inside[len(inside) // 2]
                    ax.text(p[0], p[1], wid, fontsize=6, color="#006f8c", zorder=9,
                            bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.6))
                    labels += 1
    oe = [(o["x"], o["y"]) for o in open_ends if in_box(o, cx, cy, half)]
    if oe:
        ax.scatter([p[0] for p in oe], [p[1] for p in oe], marker="x", s=70, c="red", linewidths=2, zorder=10)
    for i, c in enumerate(spot["cands"][:30], 1):
        ax.scatter([c["x"]], [c["y"]], s=150, facecolors="none", edgecolors="red", linewidths=1.4, zorder=11)
        ax.text(c["x"], c["y"], f" {i}", fontsize=7, color="red", fontweight="bold", zorder=12)
    ax.set_title(title + (f"   [bg: {bg[2]}]" if bg is not None else ""), fontsize=8)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


def crosswalk_polys(net, crosswalks):
    for cw in crosswalks:
        r = net.roads.get(cw["road"])
        if r is None:
            continue
        o = next((o for o in r.el.iter("object") if o.get("id") == cw["object"]), None)
        if o is None:
            continue
        s0, t0, hdg = float(o.get("s", 0)), float(o.get("t", 0)), float(o.get("hdg", 0) or 0)
        cl = [(float(c.get("u", 0)), float(c.get("v", 0))) for c in o.iter("cornerLocal")]
        if not cl:
            continue
        pts = []
        for u, v in cl:
            s = min(max(s0 + u * math.cos(hdg) - v * math.sin(hdg), 0), r.length)
            t = t0 + u * math.sin(hdg) + v * math.cos(hdg)
            x, y = r.point(np.array([s]), np.array([t]))
            pts.append((float(x[0]), float(y[0])))
        cw["_poly"] = pts


# ----------------------------------------------------------------------------- prepare
def find_basemap(xodr, stem):
    d = os.path.dirname(os.path.abspath(xodr))
    best = None
    for p in glob.glob(os.path.join(d, "*_basemap")):
        pre = os.path.basename(p)[: -len("_basemap")]
        if stem.startswith(pre) and os.path.exists(os.path.join(p, "basemap.json")):
            if best is None or len(pre) > len(best[0]):
                best = (pre, p)
    return best[1] if best else None


def snapshot(src, snapdir, stable):
    age = time.time() - os.path.getmtime(src)
    if age < stable:
        raise SystemExit(f"{src} changed {age:.0f} s ago (< {stable} s): it may still be written; retry later")
    os.makedirs(snapdir, exist_ok=True)
    dst = os.path.join(snapdir, os.path.basename(src))
    shutil.copy2(src, dst)
    h = sha256(dst)
    if h != sha256(src):
        raise SystemExit(f"{src} changed while it was copied; retry later")
    return dst, h


def checker_dir_for(original, area, rev_minor):
    """the gate's results for this output when they are newer than it (the gate runs on the carla profile)"""
    d = os.path.join(GATE_WORK, area)
    if rev_minor != 4 or not os.path.isdir(d):
        return None
    fs = glob.glob(os.path.join(d, "*", "findings.json")) + glob.glob(os.path.join(d, "*", "invariants.json"))
    if fs and min(os.path.getmtime(f) for f in fs) >= os.path.getmtime(original):
        return d
    return None


def prepare_one(original, a):
    stem = os.path.splitext(os.path.basename(original))[0]
    area = area_of(stem)
    A = gate_areas().get(area, {})
    osm = a.osm or A.get("osm")
    clip = a.clip or next((A["args"][i + 1] for i, v in enumerate(A.get("args", [])) if v == "--clip"), None)
    out = os.path.join(a.out, stem)
    old = load(os.path.join(out, "manifest.json")) if os.path.exists(os.path.join(out, "manifest.json")) else None
    if old and not (a.force or a.reuse):
        print(f"{stem}: bundle exists (--force rebuilds it, --reuse rebuilds it from the same file and checker results)")
        return
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    reuse = a.reuse and old and os.path.exists(old["reviewed_file"]) and sha256(old["reviewed_file"]) == old["sha256"]
    if a.reuse and not reuse:
        print(f"{stem}: nothing to reuse (no bundle, or its reviewed file changed): building it new")
    if reuse:
        xodr, h = old["reviewed_file"], old["sha256"]
    elif a.snapshot:
        xodr, h = snapshot(original, a.snapshot, a.stable_seconds)
    else:
        xodr, h = original, sha256(original)
    if not reuse and os.path.isdir(os.path.join(out, "checkers")):
        shutil.rmtree(os.path.join(out, "checkers"))  # results of another file
    # a rebuilt bundle has new spots: old chunks and images go, old verdicts are kept aside, never merged
    for f in glob.glob(os.path.join(out, "chunk_*")):
        os.remove(f)
    shutil.rmtree(os.path.join(out, "img"), ignore_errors=True)
    vd = os.path.join(out, "verdicts")
    if os.path.isdir(vd) and os.listdir(vd):
        os.replace(vd, os.path.join(out, "verdicts.prev." + datetime.datetime.now().strftime("%Y%m%dT%H%M%S")))
    net = Net(xodr)
    rev_minor = int(net.header.get("revMinor") or 0)
    oi = None
    if osm and os.path.exists(osm):
        oi = OsmIndex(G.Osm(osm, net.od.lat0, net.od.lon0))
    box = boundary_box(net, clip)
    facts, own = network_facts(net, oi, box)
    if reuse and os.path.isdir(os.path.join(out, "checkers")):
        cdir, csrc = os.path.join(out, "checkers"), old.get("checkers_source")
    else:
        cdir = csrc = a.checkers or checker_dir_for(original, area, rev_minor)
    if cdir:  # keep the results that were reviewed: the gate overwrites its own on the next run
        for f in glob.glob(os.path.join(cdir, "*", "findings.json")) + glob.glob(os.path.join(cdir, "*", "invariants.json")):
            dst = os.path.join(out, "checkers", os.path.basename(os.path.dirname(f)), os.path.basename(f))
            if os.path.abspath(dst) != os.path.abspath(f):
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copy2(f, dst)
        cdir = os.path.join(out, "checkers")
    ran = (old or {}).get("checkers_run") if reuse else None
    if a.run_checkers and osm and (cdir is None or (reuse and ran is not None)):
        # a new run, or, when reusing a bundle whose checkers were run here, the ones whose result is missing
        cdir = os.path.join(out, "checkers")
        fresh = run_checkers(xodr, osm, clip, cdir, a.checker_python, missing_only=reuse)
        done = {r["tool"] for r in fresh}
        ran = [r for r in (ran or []) if r["tool"] not in done] + fresh
        for r in ran:  # results present now count as done, whatever an earlier run recorded
            res = os.path.join(cdir, r["tool"] if r["tool"] in DETECTORS else "exact_" + r["tool"],
                               "invariants.json" if r["tool"] == "invariants" else "findings.json")
            r["ok"] = os.path.exists(res)
            if r["ok"]:
                r["err"] = ""
            elif not r.get("err"):
                r["err"] = "no result file"
    checks, used = checker_candidates(cdir) if cdir else ([], [])
    for i, c in enumerate(own):
        c["id"] = f"llm_review:{c['kind']}:{i}"
    cands = own + checks
    for c in cands:
        c["severity"] = norm_sev(c.get("severity"))
    mark_helper_roads(net, cands)
    spots, clusters = select_spots(cands, a.spots)
    for k, sp in enumerate(spots, 1):
        sp["id"] = f"S{k:02d}"
        sp["cands"] = sorted((cands[i] for i in sp["members"]), key=lambda c: (-SEV_W[c["severity"]], c["id"]))
        xs = [c["x"] for c in sp["cands"]]
        ys = [c["y"] for c in sp["cands"]]
        sp["x"], sp["y"] = sp["cands"][0]["x"], sp["cands"][0]["y"]
        sp["half"] = float(np.clip(max(max(xs) - min(xs), max(ys) - min(ys)) / 2 + 20, 25, 70))
        sp["reason"] = f"{len(sp['cands'])} candidates, led by {sp['lead']} ({sp['cands'][0]['severity']})"
        # every candidate the image shows belongs to the spot: the cluster first, then the rest of the view
        mine = {c["id"] for c in sp["cands"]}
        others = sorted((c for c in cands if c["id"] not in mine and in_box(c, sp["x"], sp["y"], sp["half"])),
                        key=lambda c: (-SEV_W[c["severity"]], c["id"]))
        sp["cands"] = sorted(sp["cands"] + others, key=lambda c: (-cand_weight(c), c["id"]))  # the 30 numbered: strongest
        if others:
            sp["reason"] += f", plus {len(others)} more in view"
    for k, j in enumerate(sample_junctions(facts, cands, spots, a.samples, stem), 1):
        rs = net.in_junction.get(j["id"], [])
        ext = max((float(np.hypot(r.X - j["x"], r.Y - j["y"]).max()) for r in rs), default=20.0)
        half = float(np.clip(ext + 15, 25, 80))
        inside = sorted((c for c in cands if in_box(c, j["x"], j["y"], half)), key=lambda c: (-SEV_W[c["severity"]], c["id"]))
        spots.append(dict(id=f"J{k:02d}", x=j["x"], y=j["y"], half=half, cands=inside, members=[], lead="sample", score=0,
                          reason=f"sample of an ordinary junction {j['id']} ({j['connections']} connections), "
                                 f"not chosen for its candidates ({len(inside)} in view)"))
    crosswalk_polys(net, facts.get("crosswalks", []))
    images = not a.no_images
    if images:
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            print("matplotlib is missing: no images (use a Python that has it, or --no-images)", file=sys.stderr)
            images = False
    basemap = None if a.basemap == "none" else (a.basemap or find_basemap(original, stem))
    file_summary = summary_text(net, facts, cands, used, rev_minor, stem)
    picked = {id(sp) for sp in spots}
    severe = sorted((c for c in clusters if id(c) not in picked
                     and any(cands[i]["severity"] in ("critical", "major") for i in c["members"])),
                    key=lambda c: -c["score"])
    if severe:
        file_summary += (f"\nNot in any spot: {len(severe)} more clusters with critical / major candidates; the strongest "
                         "(no image; mention them if the facts make them matter): " + "; ".join(
                             f"({c['x']:.0f},{c['y']:.0f}) {c['lead']} x{len(c['members'])}" for c in severe[:15]))
    failed = [r for r in (ran or []) if not r.get("ok", True)]
    if failed:
        file_summary += "\nCheckers that FAILED on this file (no candidates from them): " + "; ".join(
            f"{r['tool']}: {r['err']}" for r in failed)
        print(f"{stem}: checkers failed: " + "; ".join(f"{r['tool']}: {r['err']}" for r in failed), file=sys.stderr)
    chunks = []
    for ci in range(0, len(spots), a.chunk):
        cid = f"chunk_{ci // a.chunk + 1:02d}"
        part = spots[ci:ci + a.chunk]
        md = [f"# {stem} - {cid} (prompt {P.PROMPT_VERSION})", "", file_summary, ""]
        js = []
        for sp in part:
            img = f"img/spot_{sp['id']}.png"
            if images:
                render(net, oi, sp, os.path.join(out, img), basemap=basemap, open_ends=facts.get("open_ends", []),
                       crosswalks=[c for c in facts.get("crosswalks", []) if in_box(c, sp["x"], sp["y"], sp["half"] + 10)],
                       title=f"{stem} {sp['id']}: {sp['reason'][:90]}")
            md += spot_text(net, oi, facts, sp, img if images else None)
            js.append(dict(id=sp["id"], x=r3(sp["x"]), y=r3(sp["y"]), half=r3(sp["half"]), reason=sp["reason"],
                           image=img if images else None, candidates=[c["id"] for c in sp["cands"]],
                           roads=[r.id for r in roads_in_view(net, sp["x"], sp["y"], sp["half"], 200)]))
        md += ["", f"Write your verdicts to verdicts/{cid}.json as PROMPT.md specifies."]
        with open(os.path.join(out, cid + ".md"), "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(md) + "\n")
        dump(dict(chunk=cid, file=stem, spots=js), os.path.join(out, cid + ".json"))
        chunks.append(cid)
    os.makedirs(os.path.join(out, "verdicts"), exist_ok=True)
    with open(os.path.join(out, "PROMPT.md"), "w", encoding="utf-8", newline="\n") as f:
        f.write(P.text())
    for cw in facts.get("crosswalks", []):
        cw.pop("_poly", None)
    dump(facts, os.path.join(out, "facts.json"))
    dump(cands, os.path.join(out, "candidates.json"))
    manifest = dict(schema=1, stem=stem, area=area, output=os.path.abspath(original), reviewed_file=os.path.abspath(xodr),
                    sha256=h, size=os.path.getsize(xodr), output_mtime=os.path.getmtime(original), header=net.header,
                    osm=osm, clip=clip, basemap=basemap, checkers_dir=cdir, checkers_source=csrc, checkers_used=used,
                    checkers_run=ran,
                    prompt_version=P.PROMPT_VERSION, odrforge_version=read_version(), created=now(),
                    candidates=len(cands), own_candidates=len(own), clusters=len(clusters), spots=len(spots),
                    chunks=chunks, review_path=os.path.join(os.path.dirname(os.path.abspath(original)), stem + ".review.json"),
                    roads=sorted(net.roads), seconds=round(time.time() - t0, 1))
    dump(manifest, os.path.join(out, "manifest.json"))
    print(f"{stem}: {len(net.roads)} roads, {len(cands)} candidates ({len(own)} own, checkers {','.join(used) or 'none'}), "
          f"{len(spots)} spots in {len(chunks)} chunks, {manifest['seconds']} s")


def summary_text(net, facts, cands, used, rev_minor, stem):
    c = collections.Counter(f"{x['tool']}:{x['kind']}:{x['severity']}" for x in cands)
    top = ", ".join(f"{k} {v}" for k, v in c.most_common(25))
    g = facts.get("drive_graph", {})
    de = collections.Counter(d["status"] for d in facts.get("dead_ends", []))
    oe = collections.Counter(f"{o['lane_type']}/{o.get('how')}/{o.get('flow')}" for o in facts.get("open_ends", []))
    cw = facts.get("crosswalks", [])
    return "\n".join([
        f"## File facts: {stem}",
        f"OpenDRIVE {net.header.get('revMajor')}.{rev_minor} ({P.TARGET_REV_MINORS.get(rev_minor, 'UNKNOWN target')}), "
        f"vendor {net.header.get('vendor')}, {len(net.roads)} roads ({len(net.od.connecting_roads())} in junctions), "
        f"{len(net.od.junctions)} junctions ({sum(1 for v in net.jtype.values() if v == 'direct')} direct), "
        f"traffic {collections.Counter(net.rule.values()).most_common(1)[0][0]}.",
        f"Lanes by type: {facts.get('lanes_by_type')}; km by type: {facts.get('length_by_type_km')}.",
        f"Driving-lane graph (links, junction connections and lane changes): {g.get('nodes')} lane pieces; "
        f"{100 * g.get('stuck_share', 0):.2f} % of the lane length ({g.get('stuck_pieces')} pieces) can only end in a "
        f"dead end that is neither the clip boundary nor mapped by OSM; {100 * g.get('unreachable_share', 0):.2f} % "
        f"({g.get('unreachable_pieces')} pieces) cannot be reached from the boundary or a loop; "
        f"{len(g.get('traps', []))} closed loops with no way out; largest strongly connected component "
        f"{100 * g.get('largest_scc_share', 0):.1f} % (a clipped corridor legitimately has several). Lane flow 'in' / "
        f"'out' below: the lane runs toward / away from that road end.",
        f"Dead ends: {dict(de)}. Open lane ends (type/how/flow): {dict(oe.most_common(12))}.",
        f"Crosswalks: {len(cw)}, ends on a walking lane: {dict(collections.Counter(x['ends_on_walk'] for x in cw))}. "
        f"Joints with a gap or kink: {len(facts.get('joints', []))}. OSM vehicle ways (30 m or more inside the area) "
        f"with no road: {len([w for w in facts.get('osm_ways_missing', []) if w['highway'] != 'service'])} street ways, "
        f"{len([w for w in facts.get('osm_ways_missing', []) if w['highway'] == 'service'])} service ways "
        f"(driveways, parking aisles; listed in facts, not candidates).",
        f"Checkers read: {', '.join(used) or 'none (only the review facts)'}. Candidates by tool:kind:severity "
        f"(top 25): {top}.",
    ])


def spot_text(net, oi, facts, sp, img):
    cx, cy, half = sp["x"], sp["y"], sp["half"]
    out = [f"## Spot {sp['id']} at ({cx:.1f}, {cy:.1f}), view {2 * half:.0f} m", f"Why: {sp['reason']}"]
    if img:
        out.append(f"Image: {img} (red numbers = the candidate list below)")
    out.append("### Roads in view")
    for r in roads_in_view(net, cx, cy, half):
        out += road_lines(net, oi, r, cx, cy, half)
    js = [j for j in facts.get("junctions", []) if in_box(j, cx, cy, half)]
    if js:
        out.append("### Junctions in view")
        for j in js:
            miss = "; ".join(f"{m['from_road']}->{m['to_road']} turn {m['turn_deg']} deg"
                             + (f" (OSM {m['restriction']})" if m.get("restriction") else "") for m in j["turns_not_connected"][:8])
            out.append(f"junction {j['id']}: roads {', '.join(j['roads'])}; {j['connections']} connections, "
                       f"{j['connecting_roads']} connecting roads, {j['pairs']} road pairs connected; "
                       f"turns between attached roads with lanes for them but no connection: {miss or 'none'}")
    facts_here = []
    for key in ("dead_ends", "crosswalks", "joints", "tapers", "osm_ways_missing"):
        for it in facts.get(key, []):
            if in_box(it, cx, cy, half):
                facts_here.append(f"{key}: " + json.dumps({k: v for k, v in it.items() if not k.startswith('_')},
                                                          ensure_ascii=False))
    if facts_here:
        out.append("### Measured facts in view (llm_review)")
        out += facts_here[:25]
    if sp["cands"]:
        out.append(f"### Candidates ({len(sp['cands'])}; numbered as in the image)")
        for i, c in enumerate(sp["cands"][:30], 1):
            extra = {k: c[k] for k in ("road", "lane", "junction", "s", "osm_ways", "spec") if c.get(k) not in (None, [], "")}
            m = c.get("metric") or {k: v for k, v in c.items() if k not in ("id", "tool", "kind", "severity", "x", "y",
                                                                          "road", "lane", "junction", "s", "osm_ways",
                                                                          "spec", "roads", "perspective")}
            out.append(f"{i}. [{c['id']}] {c['tool']} {c['kind']} {c['severity']} at ({c['x']}, {c['y']}) "
                       + json.dumps(extra, ensure_ascii=False) + " " + json.dumps(m, ensure_ascii=False)[:500])
        if len(sp["cands"]) > 30:
            rest = sp["cands"][30:]
            out.append(f"... and {len(rest)} more (not numbered in the image; ids with position): "
                       + ", ".join(f"{c['id']} ({c['x']:.0f},{c['y']:.0f})" for c in rest[:120])
                       + (f", ... {len(rest) - 120} more in candidates.json" if len(rest) > 120 else ""))
    out.append("")
    return out


def read_version():
    try:
        with open(os.path.join(REPO, "VERSION"), encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return None


# ----------------------------------------------------------------------------- run
def find_claude(explicit):
    if explicit:
        return explicit
    w = shutil.which("claude")
    if w:
        return w
    cands = glob.glob(os.path.join(os.environ.get("APPDATA", ""), "Claude", "claude-code", "*", "*", "claude.exe"))
    def ver(p):
        return tuple(int(x) for x in re.findall(r"\d+", os.path.basename(os.path.dirname(os.path.dirname(p)))))
    return max(cands, key=ver) if cands else None


def bundles(d):
    """the bundles under d, or d itself when it is one"""
    if os.path.exists(os.path.join(d, "manifest.json")):
        return [d]
    return sorted(os.path.dirname(m) for m in glob.glob(os.path.join(d, "*", "manifest.json")))


def pending(b):
    m = load(os.path.join(b, "manifest.json"))
    return [c for c in m["chunks"] if not os.path.exists(os.path.join(b, "verdicts", c + ".json"))]


def cmd_run(a):
    bs = bundles(a.dir)
    if not bs:
        print(f"no bundles under {a.dir} (run prepare first)")
        return 1
    if a.runner == "session":
        n = 0
        print(f"LLM review, prompt {P.PROMPT_VERSION}. For every chunk below: read the bundle's PROMPT.md once, then the "
              f"chunk's .md and every image it names, judge each spot, and write verdicts/<chunk>.json. When all are done: "
              f"python {os.path.relpath(__file__)} merge {a.dir}")
        for b in bs:
            p = pending(b)
            n += len(p)
            print(f"{b}: {len(p)} pending: {', '.join(p) or '-'}")
        print(f"{n} chunks pending")
        return 0
    exe = find_claude(a.claude)
    if not exe:
        print("claude CLI not found (pass --claude PATH)", file=sys.stderr)
        return 1
    failures = 0
    for b in bs:
        for cid in pending(b)[: a.limit or None]:
            instr = (f"You are the reviewer of an OdrForge LLM review. Read PROMPT.md and follow it exactly. Review "
                     f"{cid}.md: read it and view every image it references with the Read tool. Write your verdicts as "
                     f"JSON to verdicts/{cid}.json. Do not modify any other file.")
            cmd = [exe, "-p", instr, "--allowedTools", "Read,Write", "--add-dir", os.path.abspath(b)]
            if a.model:
                cmd += ["--model", a.model]
            if a.dry_run:
                print(" ".join(f'"{c}"' if " " in c else c for c in cmd), f"(cwd {b})")
                continue
            t = time.time()
            r = subprocess.run(cmd, cwd=b, capture_output=True, text=True, encoding="utf-8", errors="replace",
                               timeout=a.timeout)
            errs = chunk_errors(b, cid)
            if errs and errs != ["no verdict file"]:  # one chance to fix what merge would reject
                fix = instr + (" Your verdicts file exists but merge rejects these verdicts; fix them in place and keep "
                               "the valid ones: " + " | ".join(errs[:20]))
                subprocess.run([exe, "-p", fix] + cmd[3:], cwd=b, capture_output=True, text=True, encoding="utf-8",
                               errors="replace", timeout=a.timeout)
                errs = chunk_errors(b, cid)
            failures += bool(errs)
            print(f"{os.path.basename(b)} {cid}: {'ok' if not errs else '; '.join(errs[:3])} ({time.time() - t:.0f} s)"
                  + ("" if os.path.exists(os.path.join(b, "verdicts", cid + ".json")) else f" {r.stdout[-300:]} {r.stderr[-300:]}"))
    return 1 if failures else 0


def chunk_errors(b, cid):
    """what merge would reject in one chunk's verdict file"""
    f = os.path.join(b, "verdicts", cid + ".json")
    if not os.path.exists(f):
        return ["no verdict file"]
    m = load(os.path.join(b, "manifest.json"))
    try:
        d = load(f)
    except ValueError as e:
        return [f"not JSON ({e})"]
    spots = {sp["id"]: sp for c in m["chunks"] for sp in load(os.path.join(b, c + ".json"))["spots"]}
    cand_ids = {c["id"] for c in load(os.path.join(b, "candidates.json"))}
    errs = [] if d.get("prompt_version") == m["prompt_version"] else [f"prompt_version must be {m['prompt_version']}"]
    for i, v in enumerate(d.get("verdicts", [])):
        e = check_verdict(v, spots, set(m["roads"]), cand_ids)
        if e:
            errs.append(f"verdict {i}: " + "; ".join(e))
    return errs


# ----------------------------------------------------------------------------- merge
REQUIRED = ("spot", "verdict", "category", "kind", "perspective", "severity", "x", "y", "evidence", "reasoning",
            "confidence", "note")


def resolve_candidates(v, spots):
    """a candidate given as its number in the spot's list (the numbered circles of the image) -> its id"""
    sp = spots.get(v.get("spot"))
    out = []
    for c in v.get("candidates") or []:
        if isinstance(c, int) or (isinstance(c, str) and c.isdigit()):
            i = int(c)
            out.append(sp["candidates"][i - 1] if sp and 1 <= i <= min(30, len(sp["candidates"])) else c)
        else:
            out.append(c)
    v["candidates"] = out


def check_verdict(v, spots, roads, cand_ids):
    """reasons a verdict is unusable ([] = fine): schema, vocabulary, and that it points where the chunk looked"""
    resolve_candidates(v, spots)
    errs = [f"missing {k}" for k in REQUIRED if k not in v]
    if errs:
        return errs
    if v["verdict"] not in P.VERDICTS:
        errs.append(f"verdict {v['verdict']!r}")
    if v["category"] not in P.CATEGORIES:
        errs.append(f"category {v['category']!r}")
    if v["kind"] not in P.KINDS:
        errs.append(f"kind {v['kind']!r}")
    if v["severity"] not in P.SEVERITIES:
        errs.append(f"severity {v['severity']!r}")
    ps = v["perspective"] if isinstance(v["perspective"], list) else [v["perspective"]]
    errs += [f"perspective {p!r}" for p in ps if p not in P.PERSPECTIVES]
    try:
        x, y, conf = float(v["x"]), float(v["y"]), float(v["confidence"])
    except (TypeError, ValueError):
        return errs + ["x / y / confidence not numbers"]
    if not 0 <= conf <= 1:
        errs.append(f"confidence {conf}")
    sp = spots.get(v["spot"])
    if sp is None:
        errs.append(f"unknown spot {v['spot']!r}")
    elif max(abs(x - sp["x"]), abs(y - sp["y"])) > sp["half"] * 1.5:
        errs.append(f"x/y ({x:.1f}, {y:.1f}) outside spot {v['spot']} (centre {sp['x']}, {sp['y']}, half {sp['half']})")
    if v.get("road") not in (None, "") and str(v["road"]) not in roads:
        errs.append(f"road {v['road']} is not in the file")
    unknown = [c for c in v.get("candidates") or [] if c not in cand_ids]
    if unknown:
        errs.append(f"unknown candidates {unknown[:3]}")
    if not isinstance(v["evidence"], list) or (v["verdict"] != "legitimate" and not v["evidence"]):
        errs.append("evidence must be a non-empty list of measured facts")
    return errs


def merge_one(b, dest=None):
    m = load(os.path.join(b, "manifest.json"))
    roads = set(m["roads"])
    cand_ids = {c["id"] for c in load(os.path.join(b, "candidates.json"))}
    spots = {}
    for cid in m["chunks"]:
        for sp in load(os.path.join(b, cid + ".json"))["spots"]:
            spots[sp["id"]] = sp
    flags, legit, errors, reviewers, done = [], [], [], set(), []
    for cid in m["chunks"]:
        f = os.path.join(b, "verdicts", cid + ".json")
        if not os.path.exists(f):
            continue
        try:
            d = load(f)
        except ValueError as e:
            errors.append(f"{cid}: not JSON ({e})")
            continue
        done.append(cid)
        reviewers.add(d.get("reviewer", "?"))
        if d.get("prompt_version") != m["prompt_version"]:
            errors.append(f"{cid}: prompt_version {d.get('prompt_version')} != bundle {m['prompt_version']}: "
                          f"all its verdicts rejected")
            continue
        for i, v in enumerate(d.get("verdicts", [])):
            errs = check_verdict(v, spots, roads, cand_ids)
            if errs:
                errors.append(f"{cid} verdict {i}: " + "; ".join(errs))
                continue
            item = dict(x=float(v["x"]), y=float(v["y"]), road=str(v["road"]) if v.get("road") not in (None, "") else None,
                        lane=v.get("lane"), laneType=v.get("laneType"),
                        junction=str(v["junction"]) if v.get("junction") not in (None, "", "-1") else None,
                        s=v.get("s"), category=v["category"], note=v["note"], created=now(), source="llm-review",
                        verdict=v["verdict"], kind=v["kind"], severity=v["severity"],
                        perspective=v["perspective"] if isinstance(v["perspective"], list) else [v["perspective"]],
                        osm_ways=[str(w) for w in v.get("osm_ways") or []], evidence=v["evidence"],
                        reasoning=v["reasoning"], spec=v.get("spec", ""), confidence=float(v["confidence"]),
                        candidates=v.get("candidates") or [], spot=v["spot"], chunk=cid,
                        prompt_version=m["prompt_version"])
            (legit if v["verdict"] == "legitimate" else flags).append(item)
    for i, f in enumerate(flags, 1):
        f["id"] = i
        f["status"] = "open"
    for i, f in enumerate(legit, 1):
        f["id"] = i
    current = os.path.exists(m["output"]) and sha256(m["output"]) == m["sha256"]
    out = dict(schema=1, source="llm-review", file=os.path.basename(m["output"]), sha256=m["sha256"],
               matches_current_output=current, prompt_version=m["prompt_version"], odrforge_version=m["odrforge_version"],
               header=m["header"], reviewed=now(), reviewers=sorted(reviewers), chunks_reviewed=done,
               chunks_total=len(m["chunks"]),
               counts=dict(verdict=dict(collections.Counter(f["verdict"] for f in flags + legit)),
                           category=dict(collections.Counter(f["category"] for f in flags)),
                           severity=dict(collections.Counter(f["severity"] for f in flags)),
                           kind=dict(collections.Counter(f["kind"] for f in flags))),
               flags=flags, legitimate=legit, rejected=errors)
    path = os.path.join(dest, m["stem"] + ".review.json") if dest else m["review_path"]
    dump(out, path)
    print(f"{m['stem']}: {len(done)}/{len(m['chunks'])} chunks, {len(flags)} flags, {len(legit)} legitimate, "
          f"{len(errors)} rejected -> {path}" + ("" if current else "  (WARNING: the output changed since the review)"))
    for e in errors:
        print("   rejected:", e)
    return len(errors)


FLAG_ID0 = 1001  # review flags take ids from here: a viewer page left open numbers its new flags max(loaded) + 1


def add_flags(review_path, flags_path):
    """append the review's defect / uncertain verdicts to the viewer's <stem>.flags.json as open flags, so they
    are listed and answered like hand-made ones (flags.py, viz_server.py). Existing flags are never changed or
    removed; a verdict already added (same llm_key) is not added again. Returns the number added."""
    rv = load(review_path)
    data = load(flags_path) if os.path.exists(flags_path) else {}
    flags = data.setdefault("flags", [])
    have = {f.get("llm_key") for f in flags if isinstance(f, dict)}
    nxt = max([FLAG_ID0 - 1] + [f["id"] for f in flags if isinstance(f, dict) and isinstance(f.get("id"), int)]) + 1
    added = 0
    for f in rv["flags"]:
        key = f"{rv['prompt_version']}|{rv['sha256'][:12]}|{f['chunk']}|{f['spot']}|{f['kind']}|{f['x']:.1f}|{f['y']:.1f}"
        if key in have:
            continue
        note = (f"[LLM review {f['verdict']}, {f['severity']}, confidence {f['confidence']:.2f}] {f['note']}"
                + (f" (OSM ways {', '.join(f['osm_ways'])})" if f["osm_ways"] else ""))
        flags.append(dict(id=nxt, x=f["x"], y=f["y"], road=f["road"], lane=f["lane"], laneType=f.get("laneType"),
                          junction=f["junction"], s=f["s"], category=f["category"], note=note, created=f["created"],
                          status="open", source="llm-review", llm_key=key, kind=f["kind"], severity=f["severity"],
                          perspective=f["perspective"], osm_ways=f["osm_ways"], evidence=f["evidence"],
                          reasoning=f["reasoning"], spec=f["spec"], confidence=f["confidence"],
                          reviewed_sha256=rv["sha256"], review_file=os.path.basename(review_path)))
        have.add(key)
        nxt += 1
        added += 1
    if added:
        dump(data, flags_path)
    return added


def cmd_merge(a):
    bs = bundles(a.dir)
    if not bs:
        print(f"no bundles under {a.dir}")
        return 1
    bad = sum(merge_one(b, a.dest) for b in bs)
    if a.as_flags:
        for b in bs:
            m = load(os.path.join(b, "manifest.json"))
            rp = os.path.join(a.dest, m["stem"] + ".review.json") if a.dest else m["review_path"]
            fp = os.path.join(os.path.dirname(rp), m["stem"] + ".flags.json")
            n = add_flags(rp, fp)
            print(f"{m['stem']}: {n} review flags added to {fp}")
    return 1 if bad else 0


# ----------------------------------------------------------------------------- prompt staleness
def classify_inspect(inspect_dir):
    files = sorted(os.path.basename(p) for p in glob.glob(os.path.join(inspect_dir, "*.py")))
    return {f: text_fingerprint(os.path.join(inspect_dir, f)) for f in files
            if not f.endswith("_selftest.py") and f not in P.NOT_CHECKERS}


def stale_reasons(inspect_dir=HERE, version=None, xodrs=()):
    """why the prompt may be outdated: new / changed / removed checkers, another OdrForge version, a header revMinor
    the prompt does not target"""
    out = []
    cur = classify_inspect(inspect_dir)
    for f, fp in cur.items():
        if f not in P.CHECKERS:
            out.append(f"new checker {f}: add it to llm_review_prompt.CHECKERS (or NOT_CHECKERS) with what it measures")
        elif P.CHECKERS[f][0] != fp:
            out.append(f"checker {f} changed ({P.CHECKERS[f][0]} -> {fp}): check the prompt still describes it")
    for f in P.CHECKERS:
        if f not in cur:
            out.append(f"checker {f} is gone: remove it from the prompt")
    version = version if version is not None else read_version()
    if version and version != P.ODRFORGE_VERSION:
        out.append(f"OdrForge is {version}, the prompt was revised for {P.ODRFORGE_VERSION}")
    for x in xodrs:
        with open(x, encoding="utf-8", errors="replace") as f:
            head = f.read(4000)
        m = re.search(r'revMinor="(\d+)"', head)
        if m and int(m.group(1)) not in P.TARGET_REV_MINORS:
            out.append(f"{os.path.basename(x)} is OpenDRIVE 1.{m.group(1)}, the prompt targets "
                       f"{sorted(P.TARGET_REV_MINORS)}")
        v = re.search(r'vendor="OdrForge ([^"]+)"', head)
        if v and v.group(1) != P.ODRFORGE_VERSION:
            out.append(f"{os.path.basename(x)} was written by OdrForge {v.group(1)}, the prompt by {P.ODRFORGE_VERSION}")
    return out


def cmd_check_prompt(a):
    if a.print_fingerprint:
        for f, fp in classify_inspect(HERE).items():
            what = P.CHECKERS.get(f, ("", "TODO: what it measures"))[1]
            print(f'    "{f}": ("{fp}", "{what}"),')
        return 0
    rs = stale_reasons(xodrs=a.xodr or [])
    print(f"prompt {P.PROMPT_VERSION} (revised {P.REVISED}, OdrForge {P.ODRFORGE_VERSION}): "
          + ("current" if not rs else f"{len(rs)} reasons to revise it"))
    for r in rs:
        print("  -", r)
    return 1 if rs else 0


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("prepare", help="build review bundles")
    p.add_argument("xodr", nargs="+")
    p.add_argument("--out", required=True)
    p.add_argument("--osm", help="OSM source (default: the gate area's)")
    p.add_argument("--clip", help="lat0,lon0,lat1,lon1 conversion clip (default: the gate area's)")
    p.add_argument("--checkers", help="directory of checker results in the gate layout (default: the gate's, if newer)")
    p.add_argument("--run-checkers", action="store_true", help="run the checkers when no current results exist")
    p.add_argument("--checker-python", default=VENV_PY if os.path.exists(VENV_PY) else sys.executable,
                   help="interpreter for the checkers (default: the gate's venv)")
    p.add_argument("--snapshot", help="copy each output here first and review the copy")
    p.add_argument("--stable-seconds", type=float, default=60, help="refuse to snapshot a file younger than this")
    p.add_argument("--spots", type=int, default=20)
    p.add_argument("--samples", type=int, default=4, help="ordinary junctions to sample")
    p.add_argument("--chunk", type=int, default=4, help="spots per chunk")
    p.add_argument("--basemap", help="basemap directory, or none (default: <area>_basemap next to the output)")
    p.add_argument("--no-images", action="store_true")
    p.add_argument("--force", action="store_true", help="rebuild an existing bundle (new snapshot, new checker results)")
    p.add_argument("--reuse", action="store_true",
                   help="rebuild an existing bundle from its reviewed file and checker results (e.g. after a prompt revision)")
    sub.add_parser("prompt", help="print the review prompt")
    p = sub.add_parser("run", help="review the bundles")
    p.add_argument("dir")
    p.add_argument("--runner", choices=["session", "claude-cli"], default="session")
    p.add_argument("--claude", help="claude executable (default: PATH, then the desktop app's)")
    p.add_argument("--model")
    p.add_argument("--limit", type=int, default=0, help="chunks per bundle (0 = all)")
    p.add_argument("--timeout", type=int, default=1200)
    p.add_argument("--dry-run", action="store_true")
    p = sub.add_parser("merge", help="verdicts -> <stem>.review.json next to each output")
    p.add_argument("dir")
    p.add_argument("--dest", help="write the review files here instead of next to the outputs")
    p.add_argument("--as-flags", action="store_true",
                   help="also append the defect / uncertain verdicts to <stem>.flags.json as open flags (ids from 1001)")
    p = sub.add_parser("check-prompt", help="is the prompt older than the checkers, specs or OdrForge?")
    p.add_argument("--xodr", nargs="*")
    p.add_argument("--print-fingerprint", action="store_true")
    a = ap.parse_args()
    if a.cmd == "prompt":
        print(P.text())
        return 0
    if a.cmd == "prepare":
        for x in a.xodr:
            prepare_one(x, a)
        return 0
    if a.cmd == "run":
        return cmd_run(a)
    if a.cmd == "merge":
        return cmd_merge(a)
    return cmd_check_prompt(a)


if __name__ == "__main__":
    sys.exit(main() or 0)
