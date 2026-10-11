"""Synthetic OSM fuzzer for OdrForge: proves converter behaviour on ANY OSM data, not five cities.

Generates small, valid, seeded OSM networks that cover the feature space systematically (straight /
curved / hairpin ways, 1..6 lanes, asymmetric lanes:forward/backward, oneway (incl. oneway=-1 and
implied oneway), lane-count changes, turn:lanes / change:lanes / placement, width tags, T/X/Y/skewed
/5-way junctions, roundabouts, motorway diverge/merge with gores, dual carriageways, dead ends,
short ways, closely spaced nodes) in right-hand (US, DE) and left-hand (GB, JP, AU) countries,
converts each with build/Release/osm2xodr.exe and checks the result with EXACT invariants.

Because the fuzzer wrote the OSM, it knows the intended answer (lane counts per direction, which
arms of a junction must be connected, which lane may turn where, where the way lies in the cross
section). That makes every check a pass/fail oracle instead of a heuristic.

    python fuzz.py --cases 200 --seed 1 --out C:/mywork/osm2carla/build/tmp/spec/fuzz/run1
    python fuzz.py --cases 20 --only roundabout,motorway --shrink
    python fuzz.py --replay DIR/cases/0042_roundabout     # re-run one stored case

Per case DIR/cases/<id>_<kind>/ holds in.osm, out.xodr, log.txt, meta.json, result.json and (with
--shrink) min/in.osm, the smallest OSM that still shows the first failing check.
result.json: {"checks": {invariant: {count, instances[{road, lane, s, x, y, value, limit}]}}, ...}
DIR/summary.json aggregates failures per kind, per feature and per invariant.

Natural-flow rules of the user are part of the spec and are checked here (see CHECKS):
  follow the road; split only where OSM splits/merges; lanes keep their direction by id; every lane
  is usable (reachable from a network entry and able to reach a network exit, the "game": a car
  starting at any lane start reaches every reachable end); curbs/shoulders/sidewalks never break;
  same lane count -> no new road or section; widths change inside a section.

If tools/inspect/invariants.py or laneflow.py exist they are run too (their JSON is merged under
"ext:<module>:<invariant>"); the built-in checks below always run.

The game is played per start point (check_exit_game): for every network entry, the set of exits the lane
graph reaches must equal the set the same game on the OSM (clipped exactly as the converter clips) reaches:
missed exits -> exit_unreachable (exit_unreachable_uturn when only U-turns are missing), extra exits ->
exit_illegal (wrong-way travel or a movement OSM does not have). Each missed / extra (entry, exit) pair is
attributed to a minimal set of causes (an exit or an entry), so one root defect is one instance.

Tolerances are the ones of docs/SPEC_GEOMETRY.md section 0.2; every compared value is evaluated exactly
(exact_pose / elem_xy), never from odrgeom's interpolated reference-line samples. Lane links are read with
all <predecessor>/<successor> elements (lane_links) and a joint declared by either lane is an edge.
"""
import argparse
import collections
import copy
import importlib
import json
import math
import os
import random
import re
import shutil
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import odrgeom as G  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
EXE = os.path.join(REPO, "build", "Release", "osm2xodr.exe")
XSD_DIR = os.path.join(REPO, "tests", "xsd", "opendrive_1_8_1")
ODRPLOT = r"C:\mywork\tools\esmini\esmini\bin\odrplot.exe"
DEFAULT_OUT = r"C:\mywork\osm2carla\build\tmp\spec\fuzz"

# ----------------------------------------------------------------------------- tolerances
# Every tolerance is either numeric precision or a documented contract, never tuned on a city. The numeric
# ones are the values of docs/SPEC_GEOMETRY.md section 0.2 (one table for every checker, spec O-5).
POS_TOL = 1e-3      # m, spec eps_pos. Geometry is printed with >= 12 significant digits (|x|,|y| < 1e5 m:
                    # rounding <= 1e-7 m) and every compared value is evaluated exactly (exact_pose: closed
                    # form for line / arc, 16-point Gauss-Legendre per <= 0.25 rad of heading change for
                    # spirals, error < 1e-9 m; widths / offsets are polynomials). road.ref() samples are used
                    # for searching only (spec 0.3).
HDG_TOL = 1e-4      # rad, spec eps_hdg (= eps_pos over 10 m of road; headings are analytic).
WIDTH_EPS = 1e-3    # m, spec eps_w: a lane narrower than this is zero width (a taper end).
TAG_TOL = 0.01      # m. OSM width tags carry at most centimetres.
W_P = 2.13          # m, spec W_P: AASHTO P design vehicle width (7 ft) [Green Book Table 2-1a]. A drivable lane
                    # that never reaches it along its run is unusable (spec FLOW-6 'narrow').
CORNER_TOL = 1.5    # m. osm2xodr --corner-tolerance default: how far a rounded corner may leave the
                    # mapped corner (the converter's own documented contract).
PLACEMENT_TOL = 0.05  # m. Placement is checked on straight ways only, where the reference line must lie
                      # exactly on the way: OSM coordinates are written with 1e-8 deg (~1 mm).
JUNCTION_ZONE = 100.0  # m. Shape is not compared within this distance of an OSM node where >= 3 arms meet:
                       # there OSM draws all arms from one point, while a lane-level map must spread them
                       # (gores, corner radii); lane continuity / flow checks cover those areas exactly.
UTURN_DEG = 150.0   # deg. Turns sharper than this are U-turns (the same class boundary as missing_uturn).
DEADEND_TOL = 2.0   # m. A road end without links must be at an OSM dead-end node (corner tolerance + 0.5).
DRIVE = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "slipLane", "bidirectional"}
# lanes the flow game drives on; a bidirectional (centre turn) lane is entered and left sideways, so it
# is not required to connect longitudinally
FLOW_DRIVE = DRIVE - {"bidirectional"}
EDGE_TYPES = {"sidewalk", "shoulder", "curb", "border", "biking", "walking"}
CAR_HW = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
          "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street",
          "service", "road"}

# check name -> (class, description). class: defect | flow | natural | interpretation | crash
CHECKS = {
    "conversion_failed": ("crash", "converter crashed / timed out / wrote no file"),
    "converter_validation_errors": ("defect", "the converter's own read-back validation reported errors"),
    "xsd_invalid": ("defect", "asam18 output does not validate against the OpenDRIVE 1.8.1 XSD"),
    "esmini_load_error": ("defect", "esmini odrplot reports errors loading the carla output"),
    "link_target_missing": ("defect", "road/junction/lane link points to an id that does not exist"),
    "link_not_reciprocal": ("defect", "road A links to road B but B does not link back to A at that end"),
    "road_link_gap": ("defect", "reference lines of linked roads do not meet (position or heading)"),
    "lane_link_gap": ("defect", "linked lanes' borders do not coincide at the link (lane continuity)"),
    "lane_direction_flip": ("flow", "a lane link joins lanes of opposite travel direction"),
    "unlinked_lane": ("defect", "a lane of non-zero width ends/starts where the road continues, unlinked"),
    "edge_type_break": ("natural", "a sidewalk/shoulder/curb continues as a different lane type"),
    "width_jump": ("defect", "lane width or laneOffset jumps between consecutive records"),
    "negative_width": ("defect", "a lane width polynomial is negative"),
    "zero_width_drive_lane": ("flow", "a driving lane is (near) zero width along a whole section: unusable"),
    "border_fold": ("defect", "a lane border folds over itself (1 - k*t <= 0): curve tighter than the lane"),
    "unlinked_road_end": ("flow", "road end without any link where OSM has no dead end (missing link)"),
    "connecting_road_unlinked": ("flow", "a junction connecting road lacks its road link to in/out road"),
    "lane_into_junction_unconnected": ("flow", "a driving lane enters a junction with no connection"),
    "lane_from_junction_unconnected": ("interpretation", "a driving lane leaves a junction but no connection "
                                       "feeds it (usable only by a lane change)"),
    "unreachable_lane": ("flow", "no path from any network entry reaches this driving lane"),
    "trap_lane": ("flow", "no path from this driving lane reaches any network exit"),
    "road_without_drive_lane": ("flow", "a road of a car way carries no driving lane: unusable road"),
    "way_dropped": ("flow", "a car way without access restriction has no road in the output"),
    "missing_connection": ("flow", "OSM allows driving from arm A into arm B at a node; no lane path exists"),
    "missing_uturn": ("interpretation", "OSM allows the U-turn (no restriction) at a node; no lane path"),
    "turn_lane_violation": ("interpretation", "a lane connects to an arm its turn:lanes value forbids"),
    "turn_movement_missing": ("interpretation", "a turn:lanes movement has no lane connection"),
    "lane_count_mismatch": ("interpretation", "driving lanes per direction differ from the OSM lane tags"),
    "width_mismatch": ("interpretation", "lane widths differ from width / width:lanes tags"),
    "placement_offset": ("interpretation", "the OSM way is not at the cross-section position placement=* says"),
    "shape_deviation": ("defect", "reference line leaves its OSM way by more than the corner tolerance"),
    "superfluous_section": ("natural", "lane section split with identical lanes on both sides"),
    "superfluous_road_split": ("natural", "road split (plain road-road link) with identical lanes and identity "
                               "lane links; only a change of the road's identity (OSM name/ref) justifies it"),
    "plain_road_link": ("natural", "two non-junction roads are linked road-to-road where the lanes change "
                        "(spec NAT-1: that is a lane section, a road ends only at a junction / boundary / dead end)"),
    "narrow_drive_lane": ("flow", "a drivable lane never reaches the design vehicle width W_P along its run "
                          "(spec FLOW-6 'narrow')"),
    "exit_illegal": ("flow", "the lane game from an entry reaches an exit that OSM cannot reach from it even "
                     "with U-turns (wrong-way travel or a forbidden movement, spec FLOW-4 'illegal-reach')"),
    # added by the adversarial review (planted-defect test showed these defects passed silently)
    "planview_gap": ("defect", "consecutive planView elements of one road do not join (position / heading) or "
                     "their s / length do not add up"),
    "lane_id_invalid": ("defect", "lane ids of a section are not 1..n on the left, -1..-m on the right, 0 centre"),
    "lane_link_not_reciprocal": ("defect", "lane a links to lane b, but b names no predecessor/successor that "
                                 "links back to b (one-sided lane link)"),
    "junction_link_mismatch": ("defect", "junction laneLink and the connecting road's own lane link disagree"),
    "lane_pinch": ("defect", "a lane narrows to zero width and widens again: the lane (curb, shoulder, sidewalk "
                   "or driving lane) breaks while it continues"),
    "traffic_rule_mismatch": ("target", "roads carry no / the wrong road@rule for the country's traffic side "
                              "(absent rule means RHT in OpenDRIVE)"),
    "exit_unreachable": ("flow", "the lane game from one network entry misses exits OSM can reach from it"),
    "exit_unreachable_uturn": ("interpretation", "as exit_unreachable, but every missed exit needs a U-turn "
                               "(> 150 deg, possibly composite across a merged junction)"),
    "entry_missing": ("flow", "OSM lets traffic enter the network at this road end, but no lane starts there"),
}


# ----------------------------------------------------------------------------- projection helpers
COUNTRIES = {  # code: (lat, lon, left-hand traffic)
    "US": (34.05, -118.25, False), "DE": (48.14, 11.57, False),
    "GB": (51.51, -0.12, True), "JP": (35.68, 139.76, True), "AU": (-33.87, 151.21, True),
}


def tm_inverse(x, y, lat0, lon0):
    """Exact inverse of odrgeom.tm_forward by Newton iteration (numeric Jacobian), vectorised."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    lat = lat0 + y / 111132.0
    lon = lon0 + x / (111320.0 * math.cos(math.radians(lat0)))
    h = 1e-6
    for _ in range(4):
        fx, fy = G.tm_forward(lat, lon, lat0, lon0)
        ax, ay = G.tm_forward(lat + h, lon, lat0, lon0)
        bx, by = G.tm_forward(lat, lon + h, lat0, lon0)
        j11, j12, j21, j22 = (bx - fx) / h, (ax - fx) / h, (by - fy) / h, (ay - fy) / h  # d(x,y)/d(lon,lat)
        ex, ey = x - fx, y - fy
        det = j11 * j22 - j12 * j21
        lon = lon + (ex * j22 - ey * j12) / det
        lat = lat + (ey * j11 - ex * j21) / det
    return lat, lon


# ----------------------------------------------------------------------------- OSM builder
class Net:
    """A tiny OSM network in local metres. Nodes at identical positions (1 mm grid) are shared."""

    def __init__(self):
        self.nodes = {}   # id -> [x, y, tags]
        self.ways = []    # [id, [node ids], tags]
        self._key = {}
        self._nid, self._wid = 1, 1000

    def node(self, p, tags=None):
        k = (round(p[0], 3), round(p[1], 3))
        if k in self._key:
            nid = self._key[k]
            if tags:
                self.nodes[nid][2].update(tags)
            return nid
        nid = self._nid
        self._nid += 1
        self.nodes[nid] = [float(p[0]), float(p[1]), dict(tags or {})]
        self._key[k] = nid
        return nid

    def way(self, pts, tags):
        ids = []
        for p in pts:
            n = p if isinstance(p, int) else self.node(p)
            if not ids or ids[-1] != n:
                ids.append(n)
        wid = self._wid
        self._wid += 1
        self.ways.append([wid, ids, dict(tags)])
        return wid

    def tag_node(self, p, tags):
        return self.node(p, tags)

    def write(self, path, lat0, lon0):
        used = sorted({n for _, ids, _ in self.ways for n in ids})
        xs = np.array([self.nodes[n][0] for n in used])
        ys = np.array([self.nodes[n][1] for n in used])
        lat, lon = tm_inverse(xs, ys, lat0, lon0)
        m = 0.0005
        out = ['<?xml version="1.0" encoding="UTF-8"?>', '<osm version="0.6" generator="odrforge-fuzz">',
               f'  <bounds minlat="{lat.min() - m:.8f}" minlon="{lon.min() - m:.8f}" '
               f'maxlat="{lat.max() + m:.8f}" maxlon="{lon.max() + m:.8f}"/>']
        for n, la, lo in zip(used, lat, lon):
            tags = self.nodes[n][2]
            attrs = f'id="{n}" lat="{la:.9f}" lon="{lo:.9f}" version="1" visible="true"'
            if tags:
                out.append(f"  <node {attrs}>")
                out += [f'    <tag k="{_x(k)}" v="{_x(v)}"/>' for k, v in sorted(tags.items())]
                out.append("  </node>")
            else:
                out.append(f"  <node {attrs}/>")
        for wid, ids, tags in self.ways:
            out.append(f'  <way id="{wid}" version="1" visible="true">')
            out += [f'    <nd ref="{n}"/>' for n in ids]
            out += [f'    <tag k="{_x(k)}" v="{_x(v)}"/>' for k, v in sorted(tags.items())]
            out.append("  </way>")
        out.append("</osm>")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(out) + "\n")


def _x(s):
    return str(s).replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;")


def P(x, y):
    return np.array([x, y], float)


def unit(a):
    return P(math.cos(a), math.sin(a))


def pts_line(p0, p1, step):
    n = max(1, int(math.ceil(np.linalg.norm(p1 - p0) / step)))
    return [p0 + (p1 - p0) * i / n for i in range(n + 1)]


def pts_arc(c, r, a0, a1, step):
    n = max(2, int(math.ceil(r * abs(a1 - a0) / step)))
    return [c + r * unit(a0 + (a1 - a0) * i / n) for i in range(n + 1)]


# ----------------------------------------------------------------------------- tag generators
HW_TOWN = ["residential", "tertiary", "secondary", "primary", "unclassified", "trunk"]


def lane_profile(rng, max_side=3, oneway_p=0.3, feats=None, allow_rev=True, allow_implicit=True):
    """Random lane tags; returns (tags, expectation {fwd, bwd} or None when OSM leaves it open)."""
    t, f = {}, feats if feats is not None else []
    if rng.random() < oneway_p:
        n = rng.randint(1, min(6, 2 * max_side))
        t["lanes"] = str(n)
        if allow_rev and rng.random() < 0.15:
            t["oneway"] = "-1"
            f.append("oneway=-1")
            return t, {"fwd": 0, "bwd": n}
        t["oneway"] = "yes"
        f.append("oneway")
        return t, {"fwd": n, "bwd": 0}
    fw, bw = rng.randint(1, max_side), rng.randint(1, max_side)
    r = rng.random()
    if r < 0.55:
        t.update({"lanes": str(fw + bw), "lanes:forward": str(fw), "lanes:backward": str(bw)})
        f.append("lanes:fwd/bwd" + ("_asym" if fw != bw else ""))
        exp = {"fwd": fw, "bwd": bw}
    elif r < 0.85 or not allow_implicit:
        t["lanes"] = str(2 * fw)
        f.append("lanes_even")
        exp = {"fwd": fw, "bwd": fw}
    elif r < 0.93:
        t["lanes"] = str(2 * fw + 1)
        f.append("lanes_odd_unsplit")
        exp = {"total": 2 * fw + 1}
    else:
        f.append("no_lane_tags")
        exp = None
    if exp and "fwd" in exp and rng.random() < 0.1:
        t["lanes:both_ways"] = "1"
        t["lanes"] = str(int(t["lanes"]) + 1)
        f.append("lanes:both_ways")
    return t, exp


def sidewalk_tags(rng, feats):
    r = rng.random()
    if r < 0.25:
        feats.append("sidewalk=both")
        return {"sidewalk": "both"}
    if r < 0.35:
        s = rng.choice(["left", "right"])
        feats.append("sidewalk=" + s)
        return {"sidewalk": s}
    if r < 0.45:
        feats.append("sidewalk=no")
        return {"sidewalk": "no"}
    if r < 0.55:
        feats.append("shoulder")
        return {"shoulder": rng.choice(["yes", "both", "right"])}
    return {}


# ----------------------------------------------------------------------------- case generators
# Each generator fills Net `n` and returns params; it appends feature strings to `feats` and
# expectations to `exp` (dict way id -> {...}).
def g_straight(rng, n, feats, exp):
    L = rng.uniform(150, 400)
    a = rng.uniform(-math.pi, math.pi)
    tags, e = lane_profile(rng, feats=feats)
    tags["highway"] = rng.choice(HW_TOWN)
    tags.update(sidewalk_tags(rng, feats))
    w = n.way(pts_line(P(0, 0), L * unit(a), rng.choice([10, 30, 80, L])), tags)
    exp[w] = dict(e or {}, straight=True)
    return {"length": L}


def g_curve(rng, n, feats, exp):
    R = rng.choice([8, 10, 12, 15, 20, 35, 60, 120, 300])
    sweep = math.radians(rng.uniform(30, 200))
    side = rng.choice([-1, 1])
    max_side = max(1, min(3, int((0.8 * R - 2) / 3.5)))  # half carriageway < 0.8 R: physically drivable
    tags, e = lane_profile(rng, max_side=max_side, oneway_p=0.3 if max_side > 1 else 0.5, feats=feats)
    tags["highway"] = rng.choice(HW_TOWN)
    step = rng.choice([1.5, 3, 6, 10])
    feats += [f"R={R}", f"node_step={step}", "sweep>180" if sweep > math.pi else "sweep<=180"]
    lead = rng.uniform(40, 80)
    p0, a0 = P(0, 0), rng.uniform(-math.pi, math.pi)
    p1 = p0 + lead * unit(a0)
    c = p1 + R * unit(a0 + side * math.pi / 2)
    s0 = a0 - side * math.pi / 2
    arc = pts_arc(c, R, s0, s0 + side * sweep, step)
    a1 = a0 + side * sweep
    end = arc[-1] + lead * unit(a1)
    w = n.way(pts_line(p0, p1, lead)[:-1] + arc + [end], tags)
    exp[w] = dict(e or {})
    return {"R": R, "sweep_deg": math.degrees(sweep)}


def g_hairpin(rng, n, feats, exp):
    R = rng.choice([8, 9, 10, 12, 15])
    step = rng.choice([2, 3, 5, 8])
    legs = rng.randint(2, 4)
    tags = {"highway": rng.choice(["secondary", "tertiary", "unclassified"])}
    if rng.random() < 0.7:
        tags["lanes"] = "2"
        exp_e = {"fwd": 1, "bwd": 1}
    else:
        exp_e = {}
        feats.append("no_lane_tags")
    feats += [f"R={R}", f"node_step={step}", f"legs={legs}"]
    a = rng.uniform(-math.pi, math.pi)
    p = P(0, 0)
    pts = [p]
    side = 1
    for i in range(legs):
        L = rng.uniform(40, 90)
        q = p + L * unit(a)
        pts += pts_line(p, q, 20)[1:]
        if i < legs - 1:
            c = q + R * unit(a + side * math.pi / 2)
            s0 = a - side * math.pi / 2
            pts += pts_arc(c, R, s0, s0 + side * math.pi, step)[1:]
            p = pts[-1]
            a = a + side * math.pi
            side = -side
    w = n.way(pts, tags)
    exp[w] = exp_e
    return {"R": R}


def g_lane_change(rng, n, feats, exp):
    """2-3 collinear ways (optionally gently bent) with different lane counts per direction."""
    oneway = rng.random() < 0.35
    k = rng.randint(2, 3)
    a = rng.uniform(-math.pi, math.pi)
    p = P(0, 0)
    hw = rng.choice(HW_TOWN)
    prev = None
    for i in range(k):
        L = rng.uniform(150, 260)
        a += math.radians(rng.uniform(-8, 8))
        q = p + L * unit(a)
        if oneway:
            m = rng.randint(1, 5)
            while prev is not None and m == prev[0]:
                m = rng.randint(1, 5)
            t = {"highway": hw, "oneway": "yes", "lanes": str(m)}
            e = {"fwd": m, "bwd": 0}
            prev = (m, 0)
        else:
            fw, bw = rng.randint(1, 3), rng.randint(1, 3)
            if prev is not None:
                mode = rng.choice(["fwd_only", "bwd_only", "both"])
                fw = prev[0] if mode == "bwd_only" else (fw if fw != prev[0] else fw % 3 + 1)
                bw = prev[1] if mode == "fwd_only" else (bw if bw != prev[1] else bw % 3 + 1)
                feats.append("change_" + mode)
            t = {"highway": hw, "lanes": str(fw + bw), "lanes:forward": str(fw), "lanes:backward": str(bw)}
            e = {"fwd": fw, "bwd": bw}
            prev = (fw, bw)
        if rng.random() < 0.3:
            t["sidewalk"] = "both"
        w = n.way(pts_line(p, q, 50), t)
        exp[w] = e
        p = q
    feats.append("oneway" if oneway else "twoway")
    return {"ways": k}


def g_split_same(rng, n, feats, exp):
    """A road mapped as several ways with identical tags: must stay ONE road (natural flow)."""
    k = rng.randint(2, 4)
    tags, e = lane_profile(rng, feats=feats, allow_implicit=False)
    tags["highway"] = rng.choice(HW_TOWN)
    tags.update(sidewalk_tags(rng, feats))
    a = rng.uniform(-math.pi, math.pi)
    p = P(0, 0)
    curved = rng.random() < 0.4
    for i in range(k):
        L = rng.uniform(60, 200)
        if curved:
            a += math.radians(rng.uniform(-15, 15))
        q = p + L * unit(a)
        w = n.way(pts_line(p, q, 25), tags)
        exp[w] = dict(e or {})
        p = q
    feats.append(f"ways={k}" + ("_curved" if curved else ""))
    return {"ways": k}


def _arm_tags(rng, feats, oneway_p=0.25):
    t, e = lane_profile(rng, max_side=2, oneway_p=oneway_p, feats=feats)
    t["highway"] = rng.choice(HW_TOWN)
    return t, e


def _junction(rng, n, feats, exp, angles, through=None, L=(90, 180)):
    """Arms from the origin at the given angles. through=(i, j): arms i and j form one way passing
    through the node."""
    c = P(0, 0)
    arms = []
    for i, a in enumerate(angles):
        arms.append(rng.uniform(*L) * unit(a))
    done = set()
    if through:
        i, j = through
        t, e = _arm_tags(rng, feats)
        w = n.way(pts_line(arms[i], c, 40) + pts_line(c, arms[j], 40)[1:], t)
        exp[w] = dict(e or {})
        done |= {i, j}
        feats.append("through_way")
    for i, a in enumerate(arms):
        if i in done:
            continue
        t, e = _arm_tags(rng, feats)
        w = n.way(pts_line(c, a, 40), t)
        exp[w] = dict(e or {})
    return {"angles_deg": [round(math.degrees(x), 1) for x in angles]}


def g_junction_T(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    return _junction(rng, n, feats, exp, [a, a + math.pi, a + math.pi / 2 * rng.choice([-1, 1])],
                     through=(0, 1) if rng.random() < 0.5 else None)


def g_junction_X(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    return _junction(rng, n, feats, exp, [a, a + math.pi / 2, a + math.pi, a + 3 * math.pi / 2],
                     through=(0, 2) if rng.random() < 0.5 else None)


def g_junction_Y(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    sp = math.radians(rng.uniform(20, 50))
    return _junction(rng, n, feats, exp, [a, a + math.pi - sp, a + math.pi + sp])


def g_junction_skew(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    sk = math.radians(rng.uniform(20, 40)) * rng.choice([-1, 1])
    feats.append(f"skew={round(math.degrees(sk))}")
    return _junction(rng, n, feats, exp, [a, a + math.pi, a + math.pi / 2 + sk, a + 3 * math.pi / 2 + sk],
                     through=(0, 1) if rng.random() < 0.5 else None)


def g_junction_5(rng, n, feats, exp):
    while True:
        angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(5))
        gaps = np.diff(angs + [angs[0] + 2 * math.pi])
        if gaps.min() > math.radians(40):
            break
    return _junction(rng, n, feats, exp, angs)


def g_turn_lanes(rng, n, feats, exp):
    """Approach with turn:lanes into a T or X junction with clean 90/180 deg arm angles."""
    a = rng.uniform(-math.pi, math.pi)
    approach_dir = a  # arm 0 lies at angle a from the node; traffic drives toward the node
    X = rng.random() < 0.5
    angles = [a, a + math.pi]
    left_ok, right_ok = True, True
    if X:
        angles += [a + math.pi / 2, a - math.pi / 2]
    else:
        s = rng.choice([1, -1])
        angles += [a + s * math.pi / 2]
        # arriving along -unit(a): turning toward angle a+pi/2 is a right turn ((-u) x (u+90) < 0)
        right_ok, left_ok = (s == 1), (s == -1)
    c = P(0, 0)
    oneway = rng.random() < 0.4
    nl = rng.randint(2, 4)
    movements = []
    for i in range(nl):
        opts = ["through"]
        if i == 0 and left_ok:
            opts = ["left", "left;through"]
        if i == nl - 1 and right_ok:
            opts = ["right", "through;right"]
        movements.append(rng.choice(opts))
    turn = "|".join(movements)
    t = {"highway": rng.choice(["primary", "secondary", "tertiary"])}
    if oneway:
        t.update({"oneway": "yes", "lanes": str(nl), "turn:lanes": turn})
        e = {"fwd": nl, "bwd": 0}
        feats.append("turn:lanes_oneway")
    else:
        bw = rng.randint(1, 2)
        t.update({"lanes": str(nl + bw), "lanes:forward": str(nl), "lanes:backward": str(bw),
                  "turn:lanes:forward": turn})
        e = {"fwd": nl, "bwd": bw}
        feats.append("turn:lanes:forward")
    if rng.random() < 0.4:
        ch = "|".join(rng.choice(["no", "yes", "not_left", "not_right"]) for _ in range(nl))
        t["change:lanes" + ("" if oneway else ":forward")] = ch
        feats.append("change:lanes")
    w = n.way(pts_line(-200 * unit(approach_dir) * -1, c, 40), t)  # from arm point toward node
    exp[w] = dict(e, turn={"node_end": True, "lanes": movements})
    for ang in angles[1:]:
        tt, ee = _arm_tags(rng, feats, oneway_p=0.0)
        ww = n.way(pts_line(c, rng.uniform(100, 160) * unit(ang), 40), tt)
        exp[ww] = dict(ee or {})
    feats.append("X" if X else "T")
    return {"turn": turn}


def g_placement(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    L = rng.uniform(150, 300)
    if rng.random() < 0.5:
        m = rng.randint(1, 4)
        t = {"highway": rng.choice(HW_TOWN), "oneway": "yes", "lanes": str(m)}
        e = {"fwd": m, "bwd": 0}
        if rng.random() < 0.8:
            k = rng.randint(1, m)
            v = rng.choice(["left_of", "right_of", "middle_of"]) + f":{k}"
            t["placement"] = v
            e["placement"] = v
            feats.append("placement=" + v.split(":")[0] + "_oneway")
        else:
            e["placement"] = "default"
            feats.append("placement_default_oneway")
    else:
        fw, bw = rng.randint(1, 3), rng.randint(1, 3)
        t = {"highway": rng.choice(HW_TOWN), "lanes": str(fw + bw), "lanes:forward": str(fw),
             "lanes:backward": str(bw)}
        e = {"fwd": fw, "bwd": bw}
        if rng.random() < 0.6:
            k = rng.randint(1, fw)
            v = rng.choice(["left_of", "right_of", "middle_of"]) + f":{k}"
            t["placement:forward"] = v
            e["placement"] = v
            feats.append("placement:forward=" + v.split(":")[0])
        else:
            e["placement"] = "default"
            feats.append("placement_default_twoway" + ("_asym" if fw != bw else ""))
    w = n.way(pts_line(P(0, 0), L * unit(a), 50), t)
    exp[w] = dict(e, straight=True)
    return {}


def g_width(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    L = rng.uniform(150, 300)
    oneway = rng.random() < 0.5
    t = {"highway": rng.choice(HW_TOWN)}
    if oneway:
        m = rng.randint(1, 4)
        t.update({"oneway": "yes", "lanes": str(m)})
        e = {"fwd": m, "bwd": 0}
    else:
        fw, bw = rng.randint(1, 3), rng.randint(1, 3)
        t.update({"lanes": str(fw + bw), "lanes:forward": str(fw), "lanes:backward": str(bw)})
        e = {"fwd": fw, "bwd": bw}
    r = rng.random()
    if r < 0.4:
        W = round(rng.uniform(2.8, 3.8) * (e["fwd"] + e["bwd"]), 1)
        t["width"] = f"{W:g}"
        e["width"] = W
        feats.append("width")
    elif oneway:
        ws = [round(rng.uniform(2.6, 4.0), 1) for _ in range(e["fwd"])]
        t["width:lanes"] = "|".join(f"{x:g}" for x in ws)
        e["width_lanes_fwd"] = ws
        feats.append("width:lanes")
    else:
        wf = [round(rng.uniform(2.6, 4.0), 1) for _ in range(e["fwd"])]
        wb = [round(rng.uniform(2.6, 4.0), 1) for _ in range(e["bwd"])]
        t["width:lanes:forward"] = "|".join(f"{x:g}" for x in wf)
        t["width:lanes:backward"] = "|".join(f"{x:g}" for x in wb)
        e["width_lanes_fwd"], e["width_lanes_bwd"] = wf, wb
        feats.append("width:lanes:fwd/bwd")
    if rng.random() < 0.3:  # width change mid-road: second way, same lanes
        t2 = dict(t)
        for k in list(t2):
            if k.startswith("width"):
                del t2[k]
        q = L * unit(a)
        w1 = n.way(pts_line(P(0, 0), q, 50), t)
        w2 = n.way(pts_line(q, q + 150 * unit(a), 50), t2)
        exp[w1] = dict(e, straight=True)
        exp[w2] = {"fwd": e["fwd"], "bwd": e["bwd"], "straight": True}
        feats.append("width_change_same_lanes")
    else:
        w = n.way(pts_line(P(0, 0), L * unit(a), 50), t)
        exp[w] = dict(e, straight=True)
    return {}


def g_roundabout(rng, n, feats, exp):
    R = rng.uniform(14, 32)
    k = rng.randint(3, 5)
    ring_lanes = rng.choice([1, 1, 2])
    while True:
        angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(k))
        if np.diff(angs + [angs[0] + 2 * math.pi]).min() > math.radians(50):
            break
    c = P(0, 0)
    step = rng.choice([3, 5, 8])
    ring_pts = pts_arc(c, R, 0, 2 * math.pi, step)[:-1]
    # snap arm attachment points onto the ring polygon by inserting them
    att = [c + R * unit(x) for x in angs]
    allp = sorted([(math.atan2(*(p - c)[::-1]) % (2 * math.pi), p) for p in ring_pts + att], key=lambda z: z[0])
    seq = [p for _, p in allp]
    # counter-clockwise ring for RHT, clockwise for LHT (set by caller through feats country flag)
    lht = "LHT" in feats
    if lht:
        seq = seq[::-1]
    split = rng.random() < 0.5
    rt = {"highway": rng.choice(["primary", "secondary", "tertiary"]), "junction": "roundabout",
          "lanes": str(ring_lanes)}
    if split:  # ring split between arms
        idx = [next(i for i, p in enumerate(seq) if np.allclose(p, a)) for a in att]
        idx = sorted(idx)
        for i in range(len(idx)):
            i0, i1 = idx[i], idx[(i + 1) % len(idx)]
            part = seq[i0:i1 + 1] if i1 > i0 else seq[i0:] + seq[:i1 + 1]
            w = n.way(part, rt)
            exp[w] = {"fwd": ring_lanes, "bwd": 0}
        feats.append("ring_split")
    else:
        w = n.way(seq + [seq[0]], rt)
        exp[w] = {"fwd": ring_lanes, "bwd": 0}
        feats.append("ring_closed")
    for a in att:
        d = (a - c) / R
        tt, ee = _arm_tags(rng, feats, oneway_p=0.0)
        ww = n.way([a] + pts_line(a + 0 * d, a + rng.uniform(90, 150) * d, 40)[1:], tt)
        exp[ww] = dict(ee or {})
    feats += [f"ring_lanes={ring_lanes}", f"arms={k}"]
    return {"R": R}


def g_motorway(rng, n, feats, exp):
    """Motorway mainline with a diverge (exit) and/or merge (entry) motorway_link."""
    lht = "LHT" in feats
    a = rng.uniform(-math.pi, math.pi)
    u = unit(a)
    nrm = P(-u[1], u[0])  # left of travel
    lanes = rng.randint(2, 4)
    implicit = rng.random() < 0.3
    mt = {"highway": "motorway", "lanes": str(lanes)}
    if not implicit:
        mt["oneway"] = "yes"
    else:
        feats.append("implied_oneway")
    do_div, do_merge = rng.random() < 0.7, rng.random() < 0.7
    if not do_div and not do_merge:
        do_div = True
    # normal side: right exits in RHT, left in LHT; 20 % on the unusual side
    normal = -1 if not lht else 1
    side = normal if rng.random() < 0.8 else -normal
    feats.append("exit_side=" + ("normal" if side == normal else "unusual"))
    D, M = 500.0, 1100.0
    segs = []
    cur = lanes
    marks = []
    if do_div:
        marks.append(("div", D))
    if do_merge:
        marks.append(("merge", M))
    pos = 0.0
    for kind, at in marks:
        segs.append((pos, at, cur))
        if kind == "div":
            drop = rng.random() < 0.4
            cur = cur - 1 if drop and cur > 1 else cur
            feats.append("lane_drop" if drop else "no_lane_drop")
        else:
            add = rng.random() < 0.4
            cur = cur + 1 if add else cur
            feats.append("lane_add" if add else "no_lane_add")
        pos = at
    segs.append((pos, pos + 400, cur))
    for s0, s1, ln in segs:
        t = dict(mt, lanes=str(ln))
        w = n.way(pts_line(s0 * u, s1 * u, 100), t)
        exp[w] = {"fwd": ln, "bwd": 0}
    link_lanes = rng.randint(1, 3)
    lt = {"highway": "motorway_link", "oneway": "yes", "lanes": str(link_lanes)}
    feats.append(f"link_lanes={link_lanes}")
    ang = math.radians(rng.uniform(4, 10))
    if do_div:
        p = D * u
        d = math.cos(ang) * u + side * math.sin(ang) * nrm
        q = p + 120 * d
        c = q + 60 * side * nrm
        pts = pts_line(p, q, 30) + pts_arc(c, 60, math.atan2(*(q - c)[::-1]), math.atan2(*(q - c)[::-1]) + side * 1.2, 8)[1:]
        w = n.way(pts, lt)
        exp[w] = {"fwd": link_lanes, "bwd": 0}
        feats.append("diverge")
    if do_merge:
        p = M * u
        d = math.cos(ang) * u - side * math.sin(ang) * nrm  # arriving from the side
        q = p - 120 * d
        c = q + 60 * side * nrm
        a0 = math.atan2(*(q - c)[::-1])
        pts = pts_arc(c, 60, a0 - side * 1.2, a0, 8) + pts_line(q, p, 30)[1:]
        w = n.way(pts, lt)
        exp[w] = {"fwd": link_lanes, "bwd": 0}
        feats.append("merge")
    if rng.random() < 0.4 and side == normal:  # opposite carriageway, on the far side from the links
        off = 25 * (1 if not lht else -1) * nrm
        w = n.way(pts_line(1500 * u + off, off, 100), dict(mt, lanes=str(lanes)))
        exp[w] = {"fwd": lanes, "bwd": 0}
        feats.append("dual")
    feats.append(f"main_lanes={lanes}")
    return {}


def g_dual_carriageway(rng, n, feats, exp):
    lht = "LHT" in feats
    a = rng.uniform(-math.pi, math.pi)
    u = unit(a)
    nrm = P(-u[1], u[0])
    gap = rng.uniform(8, 20)
    L = 300
    lanes = rng.randint(1, 3)
    hw = rng.choice(["primary", "secondary", "trunk"])
    # RHT: the carriageway on the right of the centreline runs along +u; LHT: on the left
    sgn = -1 if not lht else 1
    pa, pb = sgn * gap / 2 * nrm, -sgn * gap / 2 * nrm
    t = {"highway": hw, "oneway": "yes", "lanes": str(lanes)}
    xs = rng.uniform(120, 180)
    for off, d in ((pa, 1), (pb, -1)):
        p0, p1 = off - L / 2 * u, off + L / 2 * u
        mid = off + (xs - L / 2) * u
        seq = pts_line(p0, mid, 50) + pts_line(mid, p1, 50)[1:]
        if d < 0:
            seq = seq[::-1]
        w = n.way(seq, t)
        exp[w] = {"fwd": lanes, "bwd": 0}
    m1, m2 = pa + (xs - L / 2) * u, pb + (xs - L / 2) * u
    ct, ce = _arm_tags(rng, feats, oneway_p=0.0)
    outer1 = m1 + 120 * sgn * nrm
    outer2 = m2 - 120 * sgn * nrm
    split_cross = rng.random() < 0.5
    if split_cross:
        w1 = n.way(pts_line(outer1, m1, 40), ct)
        w2 = n.way(pts_line(m1, m2, 100), ct)  # the short median crossing
        w3 = n.way(pts_line(m2, outer2, 40), ct)
        for w in (w1, w2, w3):
            exp[w] = dict(ce or {})
        feats.append("median_way_short" if gap < 10 else "median_way")
    else:
        w = n.way(pts_line(outer1, m1, 40) + [m2] + pts_line(m2, outer2, 40)[1:], ct)
        exp[w] = dict(ce or {})
    feats += [f"gap={round(gap)}", f"lanes={lanes}"]
    return {"gap": gap}


def g_dead_end(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    tags, e = lane_profile(rng, max_side=2, oneway_p=0.0, feats=feats)
    tags["highway"] = rng.choice(["residential", "service", "unclassified", "tertiary"])
    L = rng.uniform(60, 200)
    end = L * unit(a)
    kind = rng.choice(["plain", "noexit", "turning_circle", "turning_loop"])
    if kind == "noexit":
        tags["noexit"] = "yes"
    w = n.way(pts_line(P(0, 0), end, 30), tags)
    if kind in ("turning_circle", "turning_loop"):
        n.tag_node(end, {"highway": kind})
    exp[w] = dict(e or {})
    # attach to a through street so the dead end is part of a network
    b = a + math.pi / 2
    tt, ee = _arm_tags(rng, feats, oneway_p=0.0)
    w2 = n.way(pts_line(-150 * unit(b), P(0, 0), 50) + pts_line(P(0, 0), 150 * unit(b), 50)[1:], tt)
    exp[w2] = dict(ee or {})
    feats.append("deadend_" + kind)
    return {}


def g_short_way(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    u = unit(a)
    short = rng.uniform(1.0, 9.5)
    tags, e = lane_profile(rng, max_side=2, oneway_p=0.2, feats=feats, allow_implicit=False)
    tags["highway"] = rng.choice(HW_TOWN)
    mode = rng.choice(["chain", "close_junctions"])
    p1, p2 = 150 * u, (150 + short) * u
    if mode == "chain":  # A(150) - B(short) - C(150), identical tags
        for s0, s1 in ((P(0, 0), p1), (p1, p2), (p2, p2 + 150 * u)):
            w = n.way(pts_line(s0, s1, 50), tags)
            exp[w] = dict(e or {})
    else:  # two side streets attached 'short' metres apart
        w = n.way(pts_line(P(0, 0), p1, 50) + [p2] + pts_line(p2, p2 + 150 * u, 50)[1:], tags)
        exp[w] = dict(e or {})
        nrm = P(-u[1], u[0])
        for p, s in ((p1, 1), (p2, rng.choice([1, -1]))):
            tt, ee = _arm_tags(rng, feats, oneway_p=0.0)
            ww = n.way(pts_line(p, p + s * 100 * nrm, 40), tt)
            exp[ww] = dict(ee or {})
    feats += [f"short={short:.1f}", "short_" + mode]
    return {"short": short}


def g_close_nodes(rng, n, feats, exp):
    a = rng.uniform(-math.pi, math.pi)
    u = unit(a)
    nrm = P(-u[1], u[0])
    tags, e = lane_profile(rng, max_side=2, feats=feats)
    tags["highway"] = rng.choice(HW_TOWN)
    pts = []
    s = 0.0
    jitter = rng.choice([0.0, 0.05, 0.2])
    while s < 250:
        pts.append(s * u + rng.uniform(-jitter, jitter) * nrm)
        if rng.random() < 0.3:  # a cluster of nodes a few cm .. 0.8 m apart
            for _ in range(rng.randint(1, 3)):
                s += rng.uniform(0.05, 0.8)
                pts.append(s * u + rng.uniform(-jitter, jitter) * nrm)
        s += rng.uniform(3, 15)
    w = n.way(pts, tags)
    exp[w] = dict(e or {})
    feats.append(f"jitter={jitter}")
    return {"nodes": len(pts)}


KINDS = {
    "straight": g_straight, "curve": g_curve, "hairpin": g_hairpin, "lane_change": g_lane_change,
    "split_same": g_split_same, "turn_lanes": g_turn_lanes, "placement": g_placement, "width": g_width,
    "junction_T": g_junction_T, "junction_X": g_junction_X, "junction_Y": g_junction_Y,
    "junction_skew": g_junction_skew, "junction_5": g_junction_5, "roundabout": g_roundabout,
    "motorway": g_motorway, "dual_carriageway": g_dual_carriageway, "dead_end": g_dead_end,
    "short_way": g_short_way, "close_nodes": g_close_nodes,
}


def make_case(seed, idx, kinds):
    """Deterministic case idx of a run: the kind cycles, the country is balanced RHT/LHT."""
    kind = kinds[idx % len(kinds)]
    rng = random.Random(f"{seed}:{idx}:{kind}")
    country = list(COUNTRIES)[(idx % len(kinds) + idx // len(kinds)) % len(COUNTRIES)]  # every kind cycles
    lat0, lon0, lht = COUNTRIES[country]
    n, feats, exp = Net(), ["LHT" if lht else "RHT"], {}
    params = KINDS[kind](rng, n, feats, exp)
    return {"id": f"{idx:04d}_{kind}", "kind": kind, "country": country, "lht": lht, "origin": (lat0, lon0),
            "features": sorted(set(feats)), "params": params, "expect": {str(k): v for k, v in exp.items()}}, n


# ----------------------------------------------------------------------------- conversion
def run(cmd, timeout=120, cwd=None):
    t = time.time()
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace",
                           timeout=timeout, cwd=cwd)
        return r.returncode, r.stdout + r.stderr, time.time() - t
    except subprocess.TimeoutExpired as e:
        return -999, f"TIMEOUT after {timeout}s\n{e.stdout or ''}", time.time() - t


def convert(osm, xodr, country, origin, target="carla", exe=EXE):
    for _ in range(30):  # the other agent may be rebuilding the executable
        if os.path.exists(exe):
            break
        time.sleep(2)
    cmd = [exe, osm, xodr, "--country", country, "--target", target, "--origin", f"{origin[0]},{origin[1]}"]
    if os.path.exists(xodr):
        os.remove(xodr)
    rc, log, dt = run(cmd)
    # rc 1 with a written file = conversion done but the read-back validation found errors
    info = {"cmd": cmd, "rc": rc, "seconds": round(dt, 3), "ok": os.path.exists(xodr) and "validation:" in log}
    m = re.search(r"validation: (\d+) errors, (\d+) warnings", log)
    info["validation_errors"] = int(m.group(1)) if m else None
    info["validation_warnings"] = int(m.group(2)) if m else None
    rules = {}
    if m:
        for l in log[m.end():].splitlines()[1:]:
            mm = re.match(r"\s+(\d+)\s+([\w.]+)\s*$", l)
            if not mm:
                break
            rules[mm.group(2)] = int(mm.group(1))
    info["rules"] = rules
    m = re.search(r"fit to OSM nodes: mean ([\d.]+) m, p95 ([\d.]+) m, max ([\d.]+) m", log)
    info["fit_max"] = float(m.group(3)) if m else None
    return info, log


_XSD = None


def xsd_errors(path, limit=5):
    """OpenDRIVE 1.8.1 XSD validation (needs the xmlschema package; None when unavailable)."""
    global _XSD
    try:
        import xmlschema
    except ImportError:
        return None
    if _XSD is None:
        _XSD = xmlschema.XMLSchema11(os.path.join(XSD_DIR, "OpenDRIVE_Core.xsd"))  # 1.8 uses xs:alternative
    out = []
    for e in _XSD.iter_errors(path):
        out.append(str(e.reason or e.message)[:200] + f" @ {e.path}")
        if len(out) >= limit:
            break
    return out


# ----------------------------------------------------------------------------- exact OpenDRIVE helpers
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _pp3_param(g, L):
    """cumulative arc length table for paramPoly3 / poly3 (p values and s values)."""
    tag, at = g[2], g[3]
    if tag == "paramPoly3":
        cU = [float(at[k]) for k in ("aU", "bU", "cU", "dU")]
        cV = [float(at[k]) for k in ("aV", "bV", "cV", "dV")]
        pmax = 1.0 if at.get("pRange", "normalized") == "normalized" else L
    else:
        a, b, c, d = (float(at.get(k, 0)) for k in ("a", "b", "c", "d"))
        cU, cV, pmax = [0, 1, 0, 0], [a, b, c, d], L * 1.5 + 1
    p = np.linspace(0, pmax, max(400, int(L * 30)))
    du = cU[1] + 2 * cU[2] * p + 3 * cU[3] * p * p
    dv = cV[1] + 2 * cV[2] * p + 3 * cV[3] * p * p
    sp = np.hypot(du, dv)
    cum = np.concatenate([[0], np.cumsum(0.5 * (sp[1:] + sp[:-1]) * np.diff(p))])
    return p, cum, cU, cV


def heading_curv(road, s):
    """exact heading and curvature of the reference line at s (vectorised over s)."""
    s = np.atleast_1d(np.asarray(s, float))
    h, k = np.zeros_like(s), np.zeros_like(s)
    gs = road.geoms
    if not hasattr(road, "_hdg0"):
        road._hdg0 = [float(g.get("hdg")) for g in road.el.find("planView").findall("geometry")]
    starts = np.array([g[0] for g in gs])
    idx = np.clip(np.searchsorted(starts, s + 1e-9, side="right") - 1, 0, len(gs) - 1)
    for i in np.unique(idx):
        m = idx == i
        s0, L, tag, at = gs[i]
        ds = np.clip(s[m] - s0, 0, L)
        h0 = road._hdg0[i]
        if tag == "line":
            h[m], k[m] = h0, 0
        elif tag == "arc":
            c = float(at["curvature"])
            h[m], k[m] = h0 + c * ds, c
        elif tag == "spiral":
            k0, k1 = float(at["curvStart"]), float(at["curvEnd"])
            dk = (k1 - k0) / L if L > 0 else 0.0
            h[m], k[m] = h0 + k0 * ds + 0.5 * dk * ds * ds, k0 + dk * ds
        else:
            p, cum, cU, cV = _pp3_param(gs[i], L)
            if tag == "paramPoly3" and cum[-1] > 0:
                cum = cum * (L / cum[-1])
            pp = np.interp(ds, cum, p)
            du = cU[1] + 2 * cU[2] * pp + 3 * cU[3] * pp * pp
            dv = cV[1] + 2 * cV[2] * pp + 3 * cV[3] * pp * pp
            ddu = 2 * cU[2] + 6 * cU[3] * pp
            ddv = 2 * cV[2] + 6 * cV[3] * pp
            h[m] = h0 + np.arctan2(dv, du)
            k[m] = (du * ddv - dv * ddu) / np.maximum(np.hypot(du, dv) ** 3, 1e-18)
    return h, k


def exact_pose(road, s, prefer_prev=False):
    """exact (x, y, heading) of the reference line at s: the planView element is evaluated at its own
    local ds (odrgeom._eval_geometry; spirals integrated at <= 0.02 m steps) and the heading is analytic.
    road.ref() interpolates 0.2 m samples linearly and takes a finite-difference heading, which is off by
    centimetres where the curvature is high (e.g. 0.065 m at a 4.4 m connecting spiral with |k| = 2.7 1/m
    in munich), more than POS_TOL; every check against POS_TOL uses this instead. prefer_prev picks the
    element ENDING at s when s is an element boundary (the end of a section / road)."""
    gels = road.el.find("planView").findall("geometry")
    starts = [g[0] for g in road.geoms]
    s = min(max(float(s), 0.0), road.length)
    i = int(np.searchsorted(starts, s - 1e-9 if prefer_prev else s + 1e-9, side="right")) - 1
    i = min(max(i, 0), len(gels) - 1)
    ds = min(max(s - starts[i], 0.0), road.geoms[i][1])
    x, y = elem_xy(gels[i], ds)
    h = float(_elem_hk(road, i, [ds])[0][0])
    return x, y, h


_GL_X, _GL_W = np.polynomial.legendre.leggauss(16)


def elem_xy(g, ds):
    """exact world (x, y) of planView element g at local ds. Line / arc closed form; spiral by composite
    16-point Gauss-Legendre over pieces of <= 0.25 rad heading change and <= 20 m (theta is quadratic, so
    the integrand is analytic; the error is far below 1e-9 m); paramPoly3 / poly3 via odrgeom (exact at
    ds = 0 and ds = length, the only places the checks compare them)."""
    tag = g[0].tag
    if tag != "spiral":
        xs, ys = G._eval_geometry(g, np.array([0.0, float(ds)]))
        return float(xs[1]), float(ys[1])
    x0, y0, h0, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    k0, k1 = float(g[0].get("curvStart")), float(g[0].get("curvEnd"))
    dk = (k1 - k0) / L if L > 0 else 0.0
    dth = abs(k0) * ds + 0.5 * abs(dk) * ds * ds
    n = max(1, int(math.ceil(dth / 0.25)), int(math.ceil(ds / 20.0)))
    edges = np.linspace(0.0, float(ds), n + 1)
    a, b = edges[:-1, None], edges[1:, None]
    u = 0.5 * (b - a) * _GL_X[None, :] + 0.5 * (a + b)
    th = h0 + k0 * u + 0.5 * dk * u * u
    wts = 0.5 * (b - a) * _GL_W[None, :]
    return x0 + float((np.cos(th) * wts).sum()), y0 + float((np.sin(th) * wts).sum())


def end_pose(road, at_end):
    return exact_pose(road, road.length if at_end else 0.0, prefer_prev=at_end)


def lane_dir(road, lid, lht):
    """+1: travels along s, -1: against s, 0: both (bidirectional) — OpenDRIVE 1.8 semantics with the
    road's rule (RHT: right lanes (id<0) along s; LHT: left lanes (id>0) along s)."""
    rule = road.el.get("rule")
    left_hand = (rule == "LHT") if rule else lht
    if lid == 0:
        return None
    along = (lid < 0) != left_hand
    return 1 if along else -1


def sec_bounds(road, k):
    sec = road.sections[k]
    return sec.s, sec.e


def lane_w(lane, ds):
    return float(lane.width(np.array([ds]))[0])


def border_xy(road, k, lid, at_end):
    """world (inner, outer) border points of lane lid in section k at its start or end."""
    s0, s1 = sec_bounds(road, k)
    s = s1 if at_end else s0
    sec = road.sections[k]
    # evaluate widths at the section's own ds (end uses the section length, not the next section)
    ds = s - sec.s
    off = float(road._piecewise(road.offsets, np.array([s - (1e-9 if at_end else 0)]))[0])
    t, pos = off, {}
    for l in sec.left:
        w = lane_w(l, ds)
        pos[l.id] = (t, t + w)
        t += w
    t = off
    for l in sec.right:
        w = lane_w(l, ds)
        pos[l.id] = (t, t - w)
        t -= w
    if lid not in pos:
        return None
    x, y, h = exact_pose(road, s, prefer_prev=at_end)
    nx, ny = -math.sin(h), math.cos(h)
    ti, to = pos[lid]
    return (x + ti * nx, y + ti * ny), (x + to * nx, y + to * ny), abs(to - ti)


def lane_links(od):
    """{(road id, section index, lane id): ([pred ids], [succ ids])} with ALL predecessor / successor
    elements (OpenDRIVE 1.8 allows several, t_road_lanes_laneSection_lcr_lane_link maxOccurs=unbounded;
    odrgeom.Lane keeps only the first). Section indices follow odrgeom's stable sort by s."""
    if getattr(od, "_lane_links", None) is not None:
        return od._lane_links
    out = {}
    for r in od.roads.values():
        secs = sorted(r.el.find("lanes").findall("laneSection"), key=lambda e: float(e.get("s")))
        for k, se in enumerate(secs):
            for ln in se.iter("lane"):
                lk = ln.find("link")
                pr = [int(e.get("id")) for e in lk.findall("predecessor")] if lk is not None else []
                su = [int(e.get("id")) for e in lk.findall("successor")] if lk is not None else []
                out[(r.id, k, int(ln.get("id")))] = (pr, su)
    od._lane_links = out
    return out


def _elem_hk(road, i, ds):
    """exact heading and curvature of planView element i at local ds (array)."""
    s0, L, tag, at = road.geoms[i]
    ds = np.clip(np.atleast_1d(np.asarray(ds, float)), 0, L)
    h0 = float(road.el.find("planView").findall("geometry")[i].get("hdg"))
    if tag == "line":
        return np.full_like(ds, h0), np.zeros_like(ds)
    if tag == "arc":
        c = float(at["curvature"])
        return h0 + c * ds, np.full_like(ds, c)
    if tag == "spiral":
        k0, k1 = float(at["curvStart"]), float(at["curvEnd"])
        dk = (k1 - k0) / L if L > 0 else 0.0
        return h0 + k0 * ds + 0.5 * dk * ds * ds, k0 + dk * ds
    p, cum, cU, cV = _pp3_param(road.geoms[i], L)
    if tag == "paramPoly3" and cum[-1] > 0:
        cum = cum * (L / cum[-1])
    pp = np.interp(ds, cum, p)
    du = cU[1] + 2 * cU[2] * pp + 3 * cU[3] * pp * pp
    dv = cV[1] + 2 * cV[2] * pp + 3 * cV[3] * pp * pp
    ddu = 2 * cU[2] + 6 * cU[3] * pp
    ddv = 2 * cV[2] + 6 * cV[3] * pp
    return h0 + np.arctan2(dv, du), (du * ddv - dv * ddu) / np.maximum(np.hypot(du, dv) ** 3, 1e-18)


def check_planview(r, F):
    """planview_gap: element i must end where element i+1 starts (G0 position, G1 heading) and the s
    values must chain: s_{i+1} = s_i + length_i, sum of lengths = road length. Exact element evaluation
    (odrgeom._eval_geometry at ds = length) - the road's dense samples cannot show this, because they are
    merged by s and a jump between two elements at the same s is silently dropped."""
    gels = r.el.find("planView").findall("geometry")
    for i in range(len(gels) - 1):
        g, gn = gels[i], gels[i + 1]
        L = float(g.get("length"))
        ex, ey = elem_xy(g, L)
        eh = float(_elem_hk(r, i, [L])[0][0])
        sx, sy, sh = float(gn.get("x")), float(gn.get("y")), float(gn.get("hdg"))
        gap, dh = math.hypot(sx - ex, sy - ey), abs(_wrap(sh - eh))
        ds = abs(float(g.get("s")) + L - float(gn.get("s")))
        if gap > POS_TOL or dh > HDG_TOL or ds > POS_TOL:
            F.add("planview_gap", road=r.id, s=float(gn.get("s")), x=sx, y=sy,
                  value=f"gap {gap:.4f} m, dhdg {dh:.4f} rad, ds {ds:.4f} m (element {i} -> {i + 1})",
                  limit=f"{POS_TOL} m / {HDG_TOL} rad")
    tot = float(gels[-1].get("s")) + float(gels[-1].get("length")) if gels else 0.0
    if abs(tot - r.length) > POS_TOL:
        x, y, _ = r.ref(r.length)
        F.add("planview_gap", road=r.id, s=r.length, x=x, y=y,
              value=f"planView ends at s={tot:.4f}, road length {r.length:.4f}", limit=POS_TOL)


def check_lane_ids(r, F):
    """lane_id_invalid: per section left ids exactly 1..n, right exactly -1..-m, centre exactly {0}
    (OpenDRIVE 1.8 9.5.1: lanes numbered consecutively from the centre, positive left, negative right)."""
    for k, se in enumerate(sorted(r.el.find("lanes").findall("laneSection"), key=lambda e: float(e.get("s")))):
        bad = []
        for side, sign in (("left", 1), ("right", -1), ("center", 0)):
            el = se.find(side)
            ids = sorted(int(l.get("id")) for l in el.findall("lane")) if el is not None else []
            want = [0] if sign == 0 else sorted(sign * i for i in range(1, len(ids) + 1))
            if ids != want:
                bad.append(f"{side} {ids}")
        if bad:
            x, y, _ = r.ref(r.sections[k].s)
            F.add("lane_id_invalid", road=r.id, s=r.sections[k].s, x=x, y=y, value="; ".join(bad), section=k)


def check_pinch(r, F, LL):
    """lane_pinch: a lane whose width falls below WIDTH_EPS at some s and is wider than WIDTH_EPS both
    before and after that s, within one section or across an identity-linked section boundary of the
    same road. A lane may start or end with a zero-width taper, but it may not break and resume: that is
    the user's 'curbs, shoulders, sidewalks continue with no breaks' rule (and an unusable driving lane)."""
    for k, sec in enumerate(r.sections):
        L = sec.e - sec.s
        if L <= 0:
            continue
        ds = np.unique(np.concatenate([np.linspace(0, L, max(3, int(L / 0.5) + 1)),
                                       [w[0] for l in sec.lanes() for w in l.widths if 0 < w[0] < L]]))
        for ln in sec.lanes():
            if ln.id == 0:
                continue
            w = ln.width(ds)
            wide = w > WIDTH_EPS
            if not wide.any():
                continue
            first, last = int(np.argmax(wide)), len(w) - 1 - int(np.argmax(wide[::-1]))
            inner = np.nonzero(~wide[first:last + 1])[0]
            pinch = None
            if len(inner):
                pinch = sec.s + ds[first + inner[0]]
            elif k + 1 < len(r.sections) and not wide[-1]:
                for t in LL.get((r.id, k, ln.id), ([], []))[1]:
                    nl = r.sections[k + 1].lane(t)
                    if nl is None:
                        continue
                    L2 = r.sections[k + 1].e - r.sections[k + 1].s
                    w2 = nl.width(np.linspace(0, L2, max(3, int(L2 / 0.5) + 1)))
                    if w2[0] <= WIDTH_EPS and (w2 > WIDTH_EPS).any():
                        pinch = sec.e
            if pinch is not None:
                x, y, _ = r.ref(pinch)
                F.add("lane_pinch", road=r.id, lane=ln.id, s=pinch, x=x, y=y, value=f"{ln.type} width < {WIDTH_EPS}",
                      limit=WIDTH_EPS, type=ln.type)


def lane_max_width(ln, L):
    """exact maximum of a lane's piecewise-cubic width over [0, L]: piece ends and the real roots of the
    derivative inside each piece."""
    best = -1e18
    for i, (s0, (a, b, c, d)) in enumerate(ln.widths):
        s1 = ln.widths[i + 1][0] if i + 1 < len(ln.widths) else L
        if s1 <= s0 and i + 1 < len(ln.widths):
            continue
        h = max(0.0, min(s1, L) - s0)
        cand = [0.0, h]
        if abs(d) > 1e-15:
            disc = 4 * c * c - 12 * b * d
            if disc >= 0:
                cand += [(-2 * c + sg * math.sqrt(disc)) / (6 * d) for sg in (1, -1)]
        elif abs(c) > 1e-15:
            cand.append(-b / (2 * c))
        for u in cand:
            if 0 <= u <= h:
                best = max(best, a + b * u + c * u * u + d * u ** 3)
    return best


def check_narrow(r, F, LL):
    """narrow_drive_lane (spec FLOW-6 'narrow'): the run of a drivable lane (its chain across the section
    boundaries of one road, by lane links declared on either side) never reaches W_P. Runs that stay
    below WIDTH_EPS are zero_width_drive_lane instead."""
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for k, sec in enumerate(r.sections):
        for ln in sec.lanes():
            if ln.type in DRIVE:
                find((k, ln.id))
                if k + 1 < len(r.sections):
                    for t in LL.get((r.id, k, ln.id), ([], []))[1]:
                        if r.sections[k + 1].lane(t) is not None and r.sections[k + 1].lane(t).type in DRIVE:
                            parent[find((k, ln.id))] = find((k + 1, t))
                if k > 0:
                    for t in LL.get((r.id, k, ln.id), ([], []))[0]:
                        if r.sections[k - 1].lane(t) is not None and r.sections[k - 1].lane(t).type in DRIVE:
                            parent[find((k, ln.id))] = find((k - 1, t))
    runs = collections.defaultdict(list)
    for node in list(parent):
        runs[find(node)].append(node)
    for nodes in runs.values():
        wmax = max(lane_max_width(r.sections[k].lane(lid), r.sections[k].e - r.sections[k].s) for k, lid in nodes)
        if WIDTH_EPS <= wmax < W_P:
            k, lid = min(nodes)
            x, y, _ = r.ref(r.sections[k].s)
            F.add("narrow_drive_lane", road=r.id, lane=lid, s=r.sections[k].s, x=x, y=y, value=round(wmax, 3),
                  limit=W_P, sections=sorted({n[0] for n in nodes}))


def inst(road=None, lane=None, s=None, x=None, y=None, value=None, limit=None, **extra):
    d = {"road": road, "lane": lane, "s": None if s is None else round(float(s), 3),
         "x": None if x is None else round(float(x), 2), "y": None if y is None else round(float(y), 2),
         "value": value if not isinstance(value, float) else round(value, 4), "limit": limit}
    d.update(extra)
    return d


# ----------------------------------------------------------------------------- checks
class Findings:
    def __init__(self):
        self.d = collections.OrderedDict()
        self.info = collections.Counter()  # how much each oracle actually evaluated (coverage)

    def add(self, name, **kw):
        e = self.d.setdefault(name, {"count": 0, "instances": []})
        e["count"] += 1
        if len(e["instances"]) < 25:
            e["instances"].append(inst(**kw))

    def result(self):
        return dict(self.d)


def check_structure(od, F, lht):
    """link integrity, reciprocity, road/lane link geometry, width records, borders, folds."""
    roads, juncs = od.roads, od.junctions
    LL = lane_links(od)
    # traffic side: OpenDRIVE road@rule, default RHT when absent (1.5+; 1.4 has no attribute, so a 1.4
    # file can only express RHT). One instance per file: it is a single target-level decision.
    wrong = [r.id for r in roads.values() if (r.el.get("rule") or "RHT") != ("LHT" if lht else "RHT")]
    if wrong:
        F.add("traffic_rule_mismatch", road=wrong[0], value=f"{len(wrong)} of {len(roads)} roads read as "
              f"{'RHT' if lht else 'LHT'} (rule={roads[wrong[0]].el.get('rule')}, header revMinor="
              f"{od.header.get('revMinor')})", limit="LHT" if lht else "RHT")
    for r in roads.values():
        check_planview(r, F)
        check_lane_ids(r, F)
        check_pinch(r, F, LL)
        check_narrow(r, F, LL)
        for end, lk in (("end", r.succ), ("start", r.pred)):
            if lk is None:
                continue
            x, y, h = end_pose(r, end == "end")
            if lk.type == "road":
                T = roads.get(lk.id)
                if T is None or lk.contact not in ("start", "end"):
                    F.add("link_target_missing", road=r.id, s=r.length if end == "end" else 0, x=x, y=y,
                          value=f"road {lk.id} contact {lk.contact}")
                    continue
                back = T.pred if lk.contact == "start" else T.succ
                ok = back is not None and ((back.type == "road" and back.id == r.id and back.contact == end)
                                           or (back.type == "junction" and r.junction == back.id))
                if not ok and r.junction == "-1":
                    F.add("link_not_reciprocal", road=r.id, s=r.length if end == "end" else 0, x=x, y=y,
                          value=f"{r.id}.{end} -> {T.id}.{lk.contact}; back = "
                                f"{(back.type, back.id, back.contact) if back else None}")
                tx, ty, th = end_pose(T, lk.contact == "end")
                gap = math.hypot(tx - x, ty - y)
                # outward heading of r at this end vs inward heading of T at its contact
                ho = h if end == "end" else h + math.pi
                hi = th if lk.contact == "start" else th + math.pi
                dh = abs(_wrap(ho - hi))
                # reference lines must meet only between plain roads: a connecting road's reference line
                # may legitimately start at a lane border of the incoming road (lane_link_gap checks lanes)
                plain = r.junction == "-1" and T.junction == "-1"
                if (plain and gap > POS_TOL) or dh > HDG_TOL:
                    F.add("road_link_gap", road=r.id, s=r.length if end == "end" else 0, x=x, y=y,
                          value=f"gap {gap:.4f} m, dhdg {dh:.4f} rad -> road {T.id}.{lk.contact}",
                          limit=f"{POS_TOL} m / {HDG_TOL} rad")
            elif lk.type == "junction":
                if lk.id not in juncs:
                    F.add("link_target_missing", road=r.id, x=x, y=y, value=f"junction {lk.id}")
        # width / laneOffset continuity between records, negative widths, border folds
        for name, recs in (("laneOffset", r.offsets),):
            for (sa, ra), (sb, rb) in zip(recs, recs[1:]):
                va, vb = G._poly(ra, sb - sa), rb[0]
                if abs(va - vb) > POS_TOL:
                    x, y, _ = r.ref(sb)
                    F.add("width_jump", road=r.id, s=sb, x=x, y=y, value=f"{name} {va:.3f} -> {vb:.3f}",
                          limit=POS_TOL)
        for k, sec in enumerate(r.sections):
            L = sec.e - sec.s
            for ln in sec.lanes():
                for (sa, ra), (sb, rb) in zip(ln.widths, ln.widths[1:]):
                    if sb > L + 1e-6:
                        continue
                    va, vb = G._poly(ra, sb - sa), rb[0]
                    if abs(va - vb) > POS_TOL:
                        x, y, _ = r.ref(sec.s + sb)
                        F.add("width_jump", road=r.id, lane=ln.id, s=sec.s + sb, x=x, y=y,
                              value=f"width {va:.3f} -> {vb:.3f}", limit=POS_TOL)
                ds = np.linspace(0, L, max(3, int(L / 0.5) + 1))
                w = ln.width(ds)
                if w.min() < -1e-6:
                    i = int(np.argmin(w))
                    x, y, _ = r.ref(sec.s + ds[i])
                    F.add("negative_width", road=r.id, lane=ln.id, s=sec.s + ds[i], x=x, y=y,
                          value=float(w[i]), limit=0)
                if ln.type in DRIVE and w.max() < WIDTH_EPS and L > 0.5:
                    x, y, _ = r.ref(sec.s + L / 2)
                    F.add("zero_width_drive_lane", road=r.id, lane=ln.id, s=sec.s, x=x, y=y,
                          value=float(w.max()), limit=WIDTH_EPS, section=k)
        # fold: 1 - k t must stay > 0 on every border (exact tangential factor of the offset curve)
        s = np.unique(np.concatenate([np.linspace(0, r.length, max(3, int(r.length / 0.25) + 1)),
                                      [g[0] for g in r.geoms]]))
        _, kap = heading_curv(r, s)
        off = r.lane_offset(s)
        worst = None
        for k, sec in enumerate(r.sections):
            m = (s >= sec.s - 1e-9) & (s <= sec.e + 1e-9)
            if not m.any():
                continue
            ds = s[m] - sec.s
            for side, sign in ((sec.left, 1), (sec.right, -1)):
                t = off[m].copy()
                for ln in side:
                    t = t + sign * ln.width(ds)
                    f = 1 - kap[m] * t
                    i = int(np.argmin(f))
                    if f[i] <= 0 and (worst is None or f[i] < worst[0]):
                        worst = (float(f[i]), ln.id, float(s[m][i]), float(t[i]), float(kap[m][i]))
        if worst:
            x, y, _ = r.ref(worst[2])
            F.add("border_fold", road=r.id, lane=worst[1], s=worst[2], x=x, y=y,
                  value=f"1-k*t = {worst[0]:.3f} (k={worst[4]:.4f}, t={worst[3]:.2f})", limit="> 0")


class Flow:
    """Lane graph of the network: nodes (road, section, lane, dir), longitudinal + lane-change edges.
    The 'game' of the user: every drivable lane must be reachable from a network entry and must reach a
    network exit; every link joins lanes geometrically and keeps the travel direction."""

    def __init__(self, od, lht, F, osmflow=None):
        self.od, self.lht, self.F, self.osmflow = od, lht, F, osmflow
        self.LL = lane_links(od)
        # spec FLOW-0 'link declaration': a joint declared by either lane (upstream successor or downstream
        # predecessor, or across a road-road link) is an edge; a one-sided one is also reported
        # (lane_link_not_reciprocal). rev[(road, sec, lane, at_end)] = lanes (road, sec, lane) naming this end.
        # Junction entries come only from <laneLink> (a connecting lane's own predecessor never makes one).
        self.rev = collections.defaultdict(set)
        for (rid, k, lid), (pr, su) in self.LL.items():
            r = od.roads[rid]
            for at_end, ids in ((True, su), (False, pr)):
                last = k == (len(r.sections) - 1 if at_end else 0)
                if not last:
                    kk = k + 1 if at_end else k - 1
                    for t in ids:
                        self.rev[(rid, kk, t, not at_end)].add((rid, k, lid))
                    continue
                lk = r.succ if at_end else r.pred
                T = od.roads.get(lk.id) if lk is not None and lk.type == "road" else None
                if T is None or lk.contact not in ("start", "end") or r.junction != "-1" or T.junction != "-1":
                    continue
                kk = 0 if lk.contact == "start" else len(T.sections) - 1
                for t in ids:
                    self.rev[(T.id, kk, t, lk.contact == "end")].add((rid, k, lid))
        self.nodes, self.succ, self.lat = set(), collections.defaultdict(set), collections.defaultdict(set)
        self.sources, self.sinks = set(), set()
        # dedup: every physical lane link is seen from both of its lanes; each fact is reported once
        self._pairs_seen = set()      # (check, frozenset of the two lane ends)
        self._gaps = {}               # physical boundary side -> [(gap, |lane|, instance kwargs)]
        self._unlinked = []           # border points of reported unlinked lane ends
        # unlinked plain road ends: (road id, at_end) -> {"src": lane nodes entering, "snk": leaving}
        self.terminals = collections.defaultdict(lambda: {"src": set(), "snk": set()})
        self._build()
        self._report_gaps()
        self.by_rs = collections.defaultdict(list)
        for n in self.nodes:
            self.by_rs[(n[0], n[1])].append(n)

    def dirs(self, r, ln):
        if ln.type == "bidirectional":
            return [1, -1]
        d = lane_dir(r, ln.id, self.lht)
        return [d] if d else []

    def links(self, r, k, lid, at_end):
        """all lane ids lane lid of section k names at its end (successors) or start (predecessors)."""
        pr, su = self.LL.get((r.id, k, lid), ([], []))
        return su if at_end else pr

    def _once(self, check, e1, e2):
        key = (check, frozenset((e1, e2)))
        if key in self._pairs_seen:
            return False
        self._pairs_seen.add(key)
        return True

    def _build(self):
        od = self.od
        conns = collections.defaultdict(list)  # incoming road -> connections
        self.conn_by_road = collections.defaultdict(list)  # connecting road -> [(junction, connection)]
        for jid, cs in od.junctions.items():
            for c in cs:
                conns[c["incoming"]].append((jid, c))
                self.conn_by_road[c["connecting"]].append((jid, c))
        for r in od.roads.values():
            nsec = len(r.sections)
            for k, sec in enumerate(r.sections):
                for ln in sec.lanes():
                    if ln.id == 0:
                        continue
                    drive = ln.type in FLOW_DRIVE
                    for d in self.dirs(r, ln) if drive else []:
                        self.nodes.add((r.id, k, ln.id, d))
                    # lane-change edges between adjacent same-direction drivable lanes
                    if drive:
                        nb = sec.lane(ln.id + (1 if ln.id > 0 else -1))
                        if nb is not None and nb.type in FLOW_DRIVE:
                            for d in set(self.dirs(r, ln)) & set(self.dirs(r, nb)):
                                self.lat[(r.id, k, ln.id, d)].add((r.id, k, nb.id, d))
                                self.lat[(r.id, k, nb.id, d)].add((r.id, k, ln.id, d))
                    # longitudinal links at both ends of the lane (geometry + structure for every lane type)
                    for at_end in (True, False):
                        self._link(r, k, nsec, ln, at_end, conns, drive)
        self._check_junction_links()

    def _unlinked_once(self, b):
        """an unlinked lane end is reported once even when the lane on the other side of the same
        boundary is unlinked too (same inner and outer border points, either orientation)."""
        for p, q in self._unlinked:
            if (math.dist(p, b[0]) <= POS_TOL and math.dist(q, b[1]) <= POS_TOL) or \
                    (math.dist(p, b[1]) <= POS_TOL and math.dist(q, b[0]) <= POS_TOL):
                return False
        self._unlinked.append((b[0], b[1]))
        return True

    def _reciprocal(self, r, k, at_end, T, kk, tid2, t_end):
        """weak reciprocity of a non-junction lane link (own lane -> T:tid2): the target must name links
        back at the touching end, and every lane it names must link to it. 'Weak' because a 2 -> 1 merge
        written as OpenDRIVE 1.4 can name only one predecessor; with 1.8 all are named."""
        back = self.links(T, kk, tid2, t_end)
        if not back:
            return False
        for b2 in back:
            if r.sections[k].lane(b2) is None or tid2 not in self.links(r, k, b2, at_end):
                return False
        return True

    def _link(self, r, k, nsec, ln, at_end, conns, drive):
        od, F = self.od, self.F
        tids = self.links(r, k, ln.id, at_end)
        last = (k == nsec - 1) if at_end else (k == 0)
        sec = r.sections[k]
        b = border_xy(r, k, ln.id, at_end)
        width = b[2] if b else 0.0
        s_here = sec.e if at_end else sec.s
        x, y = b[0] if b else (None, None)
        travel = [d for d in self.dirs(r, ln) if (d == 1) == at_end] if drive else []  # dirs leaving here
        enter = [d for d in self.dirs(r, ln) if (d == 1) != at_end] if drive else []   # dirs arriving here
        targets = []  # (road, sec, lane, at_end_of_target, plain lane link)
        rev = self.rev.get((r.id, k, ln.id, at_end), set())
        if not last:
            kk = k + 1 if at_end else k - 1
            for tid in tids:
                if r.sections[kk].lane(tid) is None:
                    F.add("link_target_missing", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                          value=f"lane {tid} not in section {kk}")
                else:
                    targets.append((r, kk, tid, not at_end, True))
            for (_, k2, t2) in sorted(rev):  # declared only by the other lane
                if t2 not in tids and k2 == kk:
                    targets.append((r, kk, t2, not at_end, True))
            if not tids and not rev and width > WIDTH_EPS and self._unlinked_once(b):
                F.add("unlinked_lane", road=r.id, lane=ln.id, s=s_here, x=x, y=y, value=round(width, 3),
                      limit=WIDTH_EPS, where="section boundary", type=ln.type)
        else:
            lk = r.succ if at_end else r.pred
            if lk is None:
                if r.junction != "-1":
                    if drive:
                        F.add("connecting_road_unlinked", road=r.id, lane=ln.id, s=s_here, x=x, y=y)
                else:
                    for d in travel:
                        self.sinks.add((r.id, k, ln.id, d))
                        self.terminals[(r.id, at_end)]["snk"].add((r.id, k, ln.id, d))
                    for d in enter:
                        self.sources.add((r.id, k, ln.id, d))
                        self.terminals[(r.id, at_end)]["src"].add((r.id, k, ln.id, d))
            elif lk.type == "road":
                T = od.roads.get(lk.id)
                if T is not None and lk.contact in ("start", "end"):
                    kk = 0 if lk.contact == "start" else len(T.sections) - 1
                    plain = r.junction == "-1" and T.junction == "-1"
                    for tid in tids:
                        if T.sections[kk].lane(tid) is None:
                            F.add("link_target_missing", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                                  value=f"lane {tid} not in road {T.id}")
                        else:
                            # incoming <-> connecting road lane links are judged against the junction
                            # laneLinks (junction_link_mismatch), plain ones by reciprocity
                            targets.append((T, kk, tid, lk.contact == "end", plain))
                    rev_here = [t2 for (r2, k2, t2) in sorted(rev) if r2 == T.id and k2 == kk and t2 not in tids]
                    for t2 in rev_here:  # declared only by the other road's lane
                        targets.append((T, kk, t2, lk.contact == "end", plain))
                    if not tids and not rev_here and width > WIDTH_EPS and (plain or drive) \
                            and self._unlinked_once(b):
                        F.add("unlinked_lane", road=r.id, lane=ln.id, s=s_here, x=x, y=y, value=round(width, 3),
                              limit=WIDTH_EPS, where=f"road link -> {T.id}", type=ln.type)
            elif lk.type == "junction":
                found = False
                for jid, c in conns.get(r.id, []):
                    if jid != lk.id:
                        continue
                    C = od.roads.get(c["connecting"])
                    if C is None:
                        continue
                    back = C.pred if c["contact"] == "start" else C.succ
                    if back is None or back.id != r.id or back.contact != ("end" if at_end else "start"):
                        continue  # this connection belongs to the other end of a looped road
                    for f, t in c["lanes"]:
                        if f == ln.id:
                            kk = 0 if c["contact"] == "start" else len(C.sections) - 1
                            if C.sections[kk].lane(t) is None:
                                F.add("link_target_missing", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                                      value=f"laneLink to {C.id} lane {t} missing")
                            else:
                                targets.append((C, kk, t, c["contact"] == "end", False))
                                found = True
                if drive and not found:
                    for d in travel:
                        ok_out = self.osmflow.expect(r, 0.5 * (sec.s + sec.e), d)[1] if self.osmflow else True
                        if ok_out:
                            F.add("lane_into_junction_unconnected", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                                  value=f"junction {lk.id}")
                            break
                    # arriving lanes are fed by connecting roads' own links; checked in the reachability pass
        e_here = (r.id, k, ln.id, at_end)
        for (T, kk, tid2, t_end, plain) in targets:
            tl = T.sections[kk].lane(tid2)
            e_there = (T.id, kk, tid2, t_end)
            bt = border_xy(T, kk, tid2, t_end)
            if b and bt and self._once("gap", e_here, e_there):
                gap = max(math.dist(b[0], bt[0]), math.dist(b[1], bt[1]))
                if gap > POS_TOL:
                    # one physical boundary = the two touching section ends + the side of the reference line
                    ends = sorted([(r.id, k, at_end), (T.id, kk, t_end)], key=str)
                    lid0 = ln.id if ends[0] == (r.id, k, at_end) else tid2
                    self._gaps.setdefault((tuple(ends), lid0 > 0), []).append(
                        (gap, abs(ln.id), dict(road=r.id, lane=ln.id, s=s_here, x=x, y=y, value=round(gap, 4),
                                               limit=POS_TOL, to=f"{T.id}:{tid2}", type=ln.type)))
            if plain and not self._reciprocal(r, k, at_end, T, kk, tid2, t_end) and \
                    self._once("recip", e_here, e_there):
                F.add("lane_link_not_reciprocal", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                      value=f"{r.id}:{ln.id} {'succ' if at_end else 'pred'} -> {T.id}:{tid2}, which links back "
                            f"to {self.links(T, kk, tid2, t_end)}")
            if ln.type in EDGE_TYPES and tl.type != ln.type and r.junction == "-1" and T.junction == "-1" \
                    and self._once("edge", e_here, e_there):
                self.F.add("edge_type_break", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                           value=f"{ln.type} -> {tl.type}", to=f"{T.id}:{tid2}")
            if not drive or tl.type not in FLOW_DRIVE:
                continue
            for d in travel:
                # arriving at target's start means it must travel along s there, at its end against s
                need = -1 if t_end else 1
                tdirs = self.dirs(T, tl)
                if need not in tdirs:
                    if self._once("flip", e_here, e_there):
                        self.F.add("lane_direction_flip", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                                   value=f"{r.id}:{ln.id} dir {d} -> {T.id}:{tid2} dirs {tdirs}")
                    continue
                self.succ[(r.id, k, ln.id, d)].add((T.id, kk, tid2, need))
            if not travel:
                # the arriving side of a link: the flow edge is made from the other lane, but a link seen
                # only from here (the other lane names no link back) must keep the direction as well
                for d in enter:
                    need = 1 if t_end else -1  # T's lane must travel toward the touching end
                    if need not in self.dirs(T, tl) and self._once("flip", e_here, e_there):
                        self.F.add("lane_direction_flip", road=r.id, lane=ln.id, s=s_here, x=x, y=y,
                                   value=f"{T.id}:{tid2} dirs {self.dirs(T, tl)} -> {r.id}:{ln.id} dir {d}")

    def _report_gaps(self):
        """lane_link_gap: one instance per physical boundary side. All lanes outside a broken border inherit
        its offset, so they are listed (instance 'lanes'), not counted; the innermost lane is the location."""
        for key, lst in sorted(self._gaps.items(), key=lambda kv: str(kv[0])):
            lst.sort(key=lambda g: (g[1], -g[0]))
            kw = dict(lst[0][2])
            kw["value"] = round(max(g[0] for g in lst), 4)
            kw["lanes"] = [f"{g[2]['road']}:{g[2]['lane']}->{g[2]['to']}" for g in lst][:12]
            self.F.add("lane_link_gap", **kw)

    def _check_junction_links(self):
        """junction_link_mismatch (OpenDRIVE 1.8 junction connection + lane linkage: both describe the same
        lane connection): for every connection (incoming I, connecting C, contactPoint) C's road link at
        that contact must name I, and the laneLink set {(from, to)} must equal the set of (I lane, C lane)
        pairs given by C's own lane links there. A connecting road must appear in some connection."""
        od = self.od
        for C in od.connecting_roads():
            lst = self.conn_by_road.get(C.id, [])
            if not lst:
                x, y, _ = end_pose(C, False)
                self.F.add("junction_link_mismatch", road=C.id, s=0, x=x, y=y,
                           value=f"connecting road of junction {C.junction} is in no connection")
            for jid, c in lst:
                at_end = c["contact"] == "end"
                lk = C.succ if at_end else C.pred
                x, y, _ = end_pose(C, at_end)
                if lk is None or lk.type != "road" or lk.id != c["incoming"]:
                    self.F.add("junction_link_mismatch", road=C.id, s=C.length if at_end else 0, x=x, y=y,
                               value=f"connection {c['incoming']}->{C.id} ({c['contact']}) but road link "
                                     f"{(lk.type, lk.id) if lk else None}")
                    continue
                kk = len(C.sections) - 1 if at_end else 0
                own = {(f, ln.id) for ln in C.sections[kk].lanes() if ln.id != 0
                       for f in self.links(C, kk, ln.id, at_end)}
                jl = set(c["lanes"])
                if own != jl:
                    self.F.add("junction_link_mismatch", road=C.id, s=C.length if at_end else 0, x=x, y=y,
                               value=f"junction {jid} laneLinks {sorted(jl)} vs road {C.id} lane links "
                                     f"{sorted(own)} to road {c['incoming']}")

    def reach(self, starts, fwd=True, lateral=True):
        pred = None
        if not fwd:
            pred = collections.defaultdict(set)
            for a, bs in self.succ.items():
                for b in bs:
                    pred[b].add(a)
        seen, stack = set(starts), list(starts)
        while stack:
            n = stack.pop()
            nxt = self.succ.get(n, ()) if fwd else pred.get(n, ())
            if lateral:
                nxt = list(nxt) + list(self.lat.get(n, ()))
            for m in nxt:
                if m not in seen:
                    seen.add(m)
                    stack.append(m)
        return seen

    def check(self, osmflow=None):
        od, F = self.od, self.F

        pred = collections.defaultdict(set)
        for a, bs in self.succ.items():
            for b in bs:
                pred[b].add(a)
        memo = {}

        def expect(n, depth=0):
            """OSM expectation; a junction connecting lane inherits it from the lanes that feed it
            (reachability) and the lanes it feeds (exit)."""
            if osmflow is None:
                return True, True
            if n in memo:
                return memo[n]
            r = od.roads[n[0]]
            sec = r.sections[n[1]]
            if r.junction == "-1" or depth > 4:
                e = osmflow.expect(r, 0.5 * (sec.s + sec.e), n[3])
            else:
                memo[n] = (True, True)  # cycle guard
                ins = [expect(m, depth + 1)[0] for m in pred.get(n, ())]
                outs = [expect(m, depth + 1)[1] for m in self.succ.get(n, ())]
                e = (any(ins) if ins else True, any(outs) if outs else True)
            memo[n] = e
            return e
        fed = set(b for bs in self.succ.values() for b in bs)
        for n in self.nodes:
            r = od.roads[n[0]]
            # lanes leaving a junction (first section, arriving from a junction link) must be fed
            k, lid, d = n[1], n[2], n[3]
            at_start_end = (d == 1 and k == 0) or (d == -1 and k == len(r.sections) - 1)
            lk = r.pred if d == 1 else r.succ
            if at_start_end and lk is not None and lk.type == "junction" and n not in fed and expect(n)[0]:
                b = border_xy(r, k, lid, d == -1)
                if b and b[2] > WIDTH_EPS:
                    F.add("lane_from_junction_unconnected", road=r.id, lane=lid,
                          s=0 if d == 1 else r.length, x=b[0][0], y=b[0][1], value=f"junction {lk.id}")
        fwd = self.reach(self.sources, True)
        bwd = self.reach(self.sinks, False)
        seen = set()
        for n in sorted(self.nodes, key=str):
            r = od.roads[n[0]]
            sec = r.sections[n[1]]
            ln = sec.lane(n[2])
            if ln is not None:
                L = sec.e - sec.s
                if float(ln.width(np.linspace(0, L, 9)).max()) < WIDTH_EPS:
                    continue  # zero-width lanes are reported by zero_width_drive_lane
            key = (n[0], n[2], n[3])
            x, y, _ = r.ref((sec.s + sec.e) / 2)
            ex_in, ex_out = expect(n)
            if n not in fwd and ex_in and ("u",) + key not in seen:
                seen.add(("u",) + key)
                F.add("unreachable_lane", road=n[0], lane=n[2], s=sec.s, x=x, y=y, value=f"dir {n[3]}",
                      section=n[1])
            if n not in bwd and ex_out and ("t",) + key not in seen:
                seen.add(("t",) + key)
                F.add("trap_lane", road=n[0], lane=n[2], s=sec.s, x=x, y=y, value=f"dir {n[3]}", section=n[1])
        return fwd, bwd


# ----------------------------------------------------------------------------- OSM oracle checks
def drivable_ways(osm, od=None):
    """car ways of the OSM; with od, only those the output carries (a way the converter leaves out by
    policy, e.g. access=no, does not make a neighbouring road end a missing link; way_dropped reports
    car ways that vanish without such a reason)."""
    ways = {w: (nds, t) for w, (nds, t) in osm.ways.items() if t.get("highway") in CAR_HW}
    if od is not None:
        conv = {w for r in od.roads.values() for w in r.osm_ways}
        ways = {w: v for w, v in ways.items() if w in conv}
    return ways


NO_ACCESS = {"no", "private"}


ABSORB_TOL = 3.0  # m: a way all of whose points lie this close to junction connecting roads became junction


def check_dropped(od, osm, meta, F):
    """A car way may vanish only into a junction (its whole length covered by connecting roads, i.e.
    every point within ABSORB_TOL of a connecting-road lane) or behind an access restriction."""
    conv = {w for r in od.roads.values() for w in r.osm_ways}
    clip = meta.get("clip_xy")
    for w, (nds, t) in drivable_ways(osm).items():
        if w in conv or any(t.get(k) in NO_ACCESS for k in ("access", "vehicle", "motor_vehicle", "motorcar")):
            continue
        if t.get("area") == "yes" or t.get("service") in ("parking_aisle", "driveway") or len(nds) < 2:
            continue
        p = osm.polyline(w)
        if clip is not None and not (inside_box(clip, p[:, 0], p[:, 1]) > 0).any():
            continue
        seg = np.diff(p, axis=0)
        dense = np.concatenate([p[k] + np.outer(np.linspace(0, 1, 10, endpoint=False), seg[k])
                                for k in range(len(seg))] + [p[-1:]])
        cx = od.roads_near(float(dense[:, 0].mean()), float(dense[:, 1].mean()),
                           float(np.hypot(*(dense.max(0) - dense.min(0)))) / 2 + 30)
        best = np.full(len(dense), np.inf)
        for r in cx:
            if r.junction == "-1":
                continue
            ext = max((abs(v) for sec in r.sections for v in r.borders_at(0.5 * (sec.s + sec.e), sec)
                       .get(min(sec.lanes(), key=lambda l: -abs(l.id)).id if sec.lanes() else 0, (0, 0))), default=0)
            d = np.min(np.hypot(dense[:, 0, None] - r.X[None, ::5], dense[:, 1, None] - r.Y[None, ::5]), axis=1)
            best = np.minimum(best, d - ext)
        if (best <= ABSORB_TOL).all():
            continue
        F.add("way_dropped", x=p[0][0], y=p[0][1], value=w, highway=t.get("highway"),
              length=round(float(np.hypot(*seg.T).sum()), 1))


def way_oneway(t):
    """+1 / -1 / 0. Implied oneway (OSM wiki Key:oneway): junction=roundabout|circular, highway=motorway and
    highway=motorway_link (the same reading as the converter, src/lanes.cpp crossSectionFromTags, and as
    laneflow.osm_oneway); oneway=reverse is the deprecated spelling of -1."""
    v = t.get("oneway")
    if v in ("yes", "true", "1"):
        return 1
    if v in ("-1", "reverse"):
        return -1
    if v in ("no", "false", "0"):
        return 0
    if t.get("junction") in ("roundabout", "circular") or t.get("highway") in ("motorway", "motorway_link"):
        return 1
    return 0


def seg_dist(px, py, poly):
    """distance from points to a polyline and the (unit) direction of the nearest segment."""
    a, b = poly[:-1], poly[1:]
    d = b - a
    l2 = np.maximum((d * d).sum(1), 1e-12)
    P_ = np.stack([px, py], 1)[:, None, :]
    t = np.clip(((P_ - a[None]) * d[None]).sum(2) / l2[None], 0, 1)
    q = a[None] + t[..., None] * d[None]
    dist = np.hypot(*(P_ - q).transpose(2, 0, 1))
    i = dist.argmin(1)
    du = d[i] / np.sqrt(l2[i])[:, None]
    return dist[np.arange(len(px)), i], du, q[np.arange(len(px)), i]


def way_roads(od):
    m = collections.defaultdict(list)
    for r in od.plain_roads():
        for w in r.osm_ways:
            m[w].append(r)
    return m


def travel_order(r, sec, s, d, lht, kinds=DRIVE):
    """drivable lanes of section sec travelling in direction d at s, ordered left -> right in travel."""
    b = r.borders_at(s, sec)
    lanes = [ln for ln in sec.lanes() if ln.type in kinds and ln.id != 0 and
             (ln.type == "bidirectional" or lane_dir(r, ln.id, lht) == d)]
    mid = lambda ln: 0.5 * (b[ln.id][0] + b[ln.id][1])  # noqa: E731
    return sorted(lanes, key=lambda ln: -d * mid(ln)), b



class OsmFlow:
    """The same 'game' on the OSM source: directed way segments, oneway (incl. implied), any move to
    another segment at a shared node except reversing on the same segment (allowed only at a dead
    end). Entries / exits are dead-end nodes (degree 1). A lane whose OSM segment+direction cannot be
    reached (or cannot leave) in OSM itself is an OSM property, not a converter defect."""

    def __init__(self, osm, ways):
        self.osm, self.ways = osm, ways
        deg = collections.Counter()
        for w, (nds, t) in ways.items():
            for i, nd in enumerate(nds):
                deg[nd] += 1 if i in (0, len(nds) - 1) else 2
        self.deg = deg
        starts, ends = collections.defaultdict(list), collections.defaultdict(list)
        self.states = []
        for w, (nds, t) in ways.items():
            ow = way_oneway(t)
            for i in range(len(nds) - 1):
                for d in ((1,) if ow == 1 else (-1,) if ow == -1 else (1, -1)):
                    st = (w, i, d)
                    a, b = (nds[i], nds[i + 1]) if d == 1 else (nds[i + 1], nds[i])
                    self.states.append(st)
                    starts[a].append(st)
                    ends[b].append(st)
        self.succ = collections.defaultdict(list)
        for nd, ins in ends.items():
            for st in ins:
                for nx in starts.get(nd, []):
                    if nx[0] == st[0] and nx[1] == st[1] and nx[2] == -st[2] and deg[nd] != 1:
                        continue
                    self.succ[st].append(nx)
        src = [st for nd, lst in starts.items() if deg[nd] == 1 for st in lst]
        snk = [st for nd, lst in ends.items() if deg[nd] == 1 for st in lst]
        self.fwd = self._reach(src, self.succ)
        pred = collections.defaultdict(list)
        for a, bs in self.succ.items():
            for b in bs:
                pred[b].append(a)
        self.bwd = self._reach(snk, pred)
        self._poly = {w: osm.polyline(w) for w in ways}

    @staticmethod
    def _reach(starts, adj):
        seen, stack = set(starts), list(starts)
        while stack:
            for m in adj.get(stack.pop(), ()):
                if m not in seen:
                    seen.add(m)
                    stack.append(m)
        return seen

    def state_at(self, road, s, d):
        """OSM (way, segment, direction) a lane of `road` driving direction d at s runs on, or None."""
        cand = [w for w in road.osm_ways if w in self._poly and len(self._poly[w]) >= 2]
        if not cand:
            return None
        x, y, h = (float(v) for v in road.ref(s))
        best = None
        for w in cand:
            P_ = self._poly[w]
            dist, du, _ = seg_dist(np.array([x]), np.array([y]), P_)
            a, b = P_[:-1], P_[1:]
            dd = b - a
            l2 = np.maximum((dd * dd).sum(1), 1e-12)
            t = np.clip(((np.array([x, y]) - a) * dd).sum(1) / l2, 0, 1)
            i = int(np.argmin(np.hypot(*(a + t[:, None] * dd - np.array([x, y])).T)))
            if best is None or dist[0] < best[0]:
                sign = 1 if (math.cos(h) * dd[i][0] + math.sin(h) * dd[i][1]) * d > 0 else -1
                best = (dist[0], (w, i, sign))
        return best[1]

    def expect(self, road, s, d):
        """(should be reachable from an entry, should reach an exit) per OSM for this lane."""
        st = self.state_at(road, s, d)
        if st is None:
            return True, True
        return st in self.fwd, st in self.bwd


def inside_box(box, x, y):
    """distance of points inside the clip box (x0, y0, x1, y1) to its edge (negative outside)."""
    x0, y0, x1, y1 = box
    return np.minimum.reduce([x - x0, x1 - x, y - y0, y1 - y])


# ----------------------------------------------------------------------------- the exit game (per entry)
def tarjan(nodes, adj):
    """iterative Tarjan SCC. Returns (comp_of {node: comp index}, comps [[nodes]]). Components are emitted
    in reverse topological order: every component reachable from C is emitted before C."""
    index, low, onstack, stack, comps, comp_of = {}, {}, set(), [], [], {}
    counter = 0
    for v0 in nodes:
        if v0 in index:
            continue
        index[v0] = low[v0] = counter
        counter += 1
        stack.append(v0)
        onstack.add(v0)
        work = [(v0, iter(adj(v0)))]
        while work:
            v, it = work[-1]
            pushed = False
            for w in it:
                if w not in index:
                    index[w] = low[w] = counter
                    counter += 1
                    stack.append(w)
                    onstack.add(w)
                    work.append((w, iter(adj(w))))
                    pushed = True
                    break
                if w in onstack:
                    low[v] = min(low[v], index[w])
            if pushed:
                continue
            work.pop()
            if work:
                u = work[-1][0]
                low[u] = min(low[u], low[v])
            if low[v] == index[v]:
                comp = []
                while True:
                    w = stack.pop()
                    onstack.discard(w)
                    comp_of[w] = len(comps)
                    comp.append(w)
                    if w == v:
                        break
                comps.append(comp)
    return comp_of, comps


def reach_bits(nodes, adj, own):
    """{node: int bitset} of the exit bits (own[node]) reachable from each node, via SCC condensation."""
    comp_of, comps = tarjan(nodes, adj)
    bits = [0] * len(comps)
    for ci, comp in enumerate(comps):  # reverse topological: successors are already final
        b = 0
        for v in comp:
            b |= own.get(v, 0)
            for w in adj(v):
                cw = comp_of.get(w)
                if cw is not None and cw != ci:
                    b |= bits[cw]
        bits[ci] = b
    return {v: bits[comp_of[v]] for v in comp_of}


def _arm_dir(osm, nds, i, step):
    """unit direction from node i along the way (step +1/-1) to the first node >= 3 m away."""
    N = np.array(osm.nodes[nds[i]])
    j, q = i + step, None
    while 0 <= j < len(nds):
        if nds[j] in osm.nodes:
            q = np.array(osm.nodes[nds[j]])
            if np.linalg.norm(q - N) >= 3:
                break
        j += step
    if q is None:
        return None
    v = q - N
    return v / max(np.linalg.norm(v), 1e-9)


KEEP_OUTSIDE = 30.0  # m: clipOsm keepOutside (src/network.cpp clipOsm), mirrored to clip OSM like the converter
SNAP_REACH = 15.0    # m: upper bound of the converter's dangling-end snap distance (snapDanglingEnds = 3 m plus
                     # the target's half carriageway width); used only to make the permissive game MORE
                     # permissive, never to require a connection


def clip_segments(osm, ways, clip, hw_use):
    """OSM way segments clipped exactly as the converter's clipOsm (src/network.cpp): Liang-Barsky against
    the box; a segment end outside the box stays a real node when >= 2 highway ways use it and it lies
    within KEEP_OUTSIDE of the cut, else it becomes a cut end. Returns
    [(way, segment index, node a, node b, a real, b real, a or its cut point, b or its cut point)]."""
    out = []
    for w, (nds, t) in sorted(ways.items()):
        for i in range(len(nds) - 1):
            a, b = nds[i], nds[i + 1]
            if a not in osm.nodes or b not in osm.nodes:
                continue
            pa, pb = np.array(osm.nodes[a], float), np.array(osm.nodes[b], float)
            if clip is None:
                out.append((w, i, a, b, True, True, pa, pb))
                continue
            x0, y0, x1, y1 = clip
            d = pb - pa
            t0, t1, visible = 0.0, 1.0, True
            for p, q in ((-d[0], pa[0] - x0), (d[0], x1 - pa[0]), (-d[1], pa[1] - y0), (d[1], y1 - pa[1])):
                if abs(p) < 1e-12:
                    visible = visible and q >= 0
                elif p < 0:
                    t0 = max(t0, q / p)
                else:
                    t1 = min(t1, q / p)
            if not visible or t0 > t1:
                continue
            ca, cb = pa + d * t0, pa + d * t1
            ra = t0 <= 1e-12 or (hw_use[a] >= 2 and float(np.linalg.norm(pa - ca)) <= KEEP_OUTSIDE)
            rb = t1 >= 1 - 1e-12 or (hw_use[b] >= 2 and float(np.linalg.norm(pb - cb)) <= KEEP_OUTSIDE)
            out.append((w, i, a, b, ra, rb, ca if not ra else pa, cb if not rb else pb))
    return out


def repair_joins(osm, segs, deg):
    """node pairs the converter may join although OSM does not: a dangling end (degree 1) and the nodes of
    any other way's segment within SNAP_REACH (snapDanglingEnds), and the end nodes of two segments of
    different ways and the same layer that cross without a shared node (joined crossings)."""
    cell = 25.0
    grid = collections.defaultdict(list)
    for k, (w, i, a, b, ra, rb, pa, pb) in enumerate(segs):
        lo, hi = np.minimum(pa, pb) - SNAP_REACH, np.maximum(pa, pb) + SNAP_REACH
        for gx in range(int(lo[0] // cell), int(hi[0] // cell) + 1):
            for gy in range(int(lo[1] // cell), int(hi[1] // cell) + 1):
                grid[(gx, gy)].append(k)
    layer = {w: osm.ways[w][1].get("layer", "0") for w in {s[0] for s in segs}}
    out = set()
    for k, (w, i, a, b, ra, rb, pa, pb) in enumerate(segs):
        for n, rn, p in ((a, ra, pa), (b, rb, pb)):
            if not rn or deg[n] != 1:
                continue
            for k2 in grid.get((int(p[0] // cell), int(p[1] // cell)), ()):
                w2, _, a2, b2, ra2, rb2, qa, qb = segs[k2]
                if w2 == w:
                    continue
                dd = qb - qa
                tt = float(np.clip((p - qa) @ dd / max(dd @ dd, 1e-12), 0, 1))
                if float(np.linalg.norm(qa + tt * dd - p)) <= SNAP_REACH:
                    out |= {(n, m) for m, rm in ((a2, ra2), (b2, rb2)) if rm}
    for k, (w, i, a, b, ra, rb, pa, pb) in enumerate(segs):
        cands = set()
        for gx in range(int(min(pa[0], pb[0]) // cell), int(max(pa[0], pb[0]) // cell) + 1):
            for gy in range(int(min(pa[1], pb[1]) // cell), int(max(pa[1], pb[1]) // cell) + 1):
                cands.update(grid.get((gx, gy), ()))
        for k2 in cands:
            if k2 <= k:
                continue
            w2, _, a2, b2, ra2, rb2, qa, qb = segs[k2]
            if w2 == w or layer[w] != layer[w2] or {a, b} & {a2, b2}:
                continue
            r, s_ = pb - pa, qb - qa
            den = r[0] * s_[1] - r[1] * s_[0]
            if abs(den) < 1e-12:
                continue
            u = ((qa[0] - pa[0]) * s_[1] - (qa[1] - pa[1]) * s_[0]) / den
            v = ((qa[0] - pa[0]) * r[1] - (qa[1] - pa[1]) * r[0]) / den
            if 0 <= u <= 1 and 0 <= v <= 1:
                out |= {(m1, m2) for m1, r1 in ((a, ra), (b, rb)) if r1 for m2, r2 in ((a2, ra2), (b2, rb2)) if r2}
    return sorted(out)


def check_exit_game(od, osm, meta, F, flow, ways, debug=None):
    """The user's game, per start point: a car entering the network at an entry (an unlinked plain road
    end that is an OSM dead end, or a cut of the --clip box) must be able to reach EVERY exit that the
    same game on OSM can reach from that entry, taking every path once (reachability is computed exactly
    with SCC condensation + exit bitsets, both on the lane graph and on the OSM graph).

    OSM game: directed way segments of the converted car ways; oneway incl. implied; no reversal on the same
    segment anywhere (U-turns are an interpretation, see missing_uturn); via-node turn restrictions; a
    turn:lanes approach only into movements one of its lanes indicates; dead-end nodes and box cuts end the
    game. Lane game: lane succ/pred, road links, junction laneLinks and lane changes (Flow).
    exit_unreachable: an entry whose lanes miss an exit OSM reaches from it (only exits that exist as lane
    ends are required; a missing road end is way_dropped / unlinked_road_end).
    entry_missing: OSM lets traffic in at an entry, a road ends there, but no lane starts there."""
    clip = meta.get("clip_xy")
    hw_use = collections.Counter(nd for _, (nds, t) in osm.ways.items() if "highway" in t for nd in nds)
    r_no, r_only = turn_restrictions(osm)
    # ---- OSM states of the converted ways, clipped exactly as the converter clips (clip_segments)
    segs = clip_segments(osm, ways, clip, hw_use)
    deg, inside = collections.Counter(), {}
    cross_pts = collections.defaultdict(list)  # way -> [(segment index, cut point)]
    for w, i, a, b, ra, rb, ca, cb in segs:
        for n, rn, cp in ((a, ra, ca), (b, rb, cb)):
            if rn:
                deg[n] += 1
                inside[n] = True
            else:
                cross_pts[w].append((i, cp))
    starts, ends, entries, exit_of, states = (collections.defaultdict(list), collections.defaultdict(list), {},
                                              {}, [])
    for w, i, a, b, ra, rb, ca, cb in segs:
        ow = way_oneway(ways[w][1])
        for d in ((1,) if ow == 1 else (-1,) if ow == -1 else (1, -1)):
            st = (w, i, d)
            (u, ru), (v, rv) = ((a, ra), (b, rb)) if d == 1 else ((b, rb), (a, ra))
            states.append(st)
            if ru:
                starts[u].append(st)
                if deg[u] == 1:
                    entries.setdefault(f"n:{u}", []).append(st)
            else:
                entries.setdefault(f"e:{w}:{i}", []).append(st)
            if rv:
                ends[v].append(st)
                if deg[v] == 1:
                    exit_of[st] = f"n:{v}"
            else:
                exit_of[st] = f"e:{w}:{i}"
    succ = collections.defaultdict(list)
    for nd, ins in ends.items():
        if deg[nd] == 1:
            continue
        for st in ins:
            if st in exit_of:
                continue
            wa, ia, da = st
            nds_a, ta = ways[wa]
            i_nd = ia + 1 if da == 1 else ia
            in_dir = _arm_dir(osm, nds_a, i_nd, -da)
            mv = turn_tag(ta, da == 1)
            for nx in starts.get(nd, []):
                if nx[0] == wa and nx[1] == ia:
                    continue  # reversal on the same segment
                if nx[0] != wa and ((wa, nd, nx[0]) in r_no or r_only.get((wa, nd), nx[0]) != nx[0]):
                    continue
                if mv and in_dir is not None:
                    nds_b = ways[nx[0]][0]
                    out_dir = _arm_dir(osm, nds_b, nx[1] if nx[2] == 1 else nx[1] + 1, nx[2])
                    if out_dir is not None:
                        theta = math.degrees(_wrap(math.atan2(out_dir[1], out_dir[0]) -
                                                   math.atan2(-in_dir[1], -in_dir[0])))
                        if not turn_allows(mv, theta):
                            continue
                succ[st].append(nx)
    # ---- permissive OSM game for exit_illegal: every car way of the OSM (also ones the converter left
    # out or absorbed into a junction), clipped the same way; U-turns / reversal allowed everywhere (also
    # at dead ends); turn:lanes ignored; and the converter's documented OSM repairs are allowed moves:
    # a dangling way end may continue onto any car way within SNAP_REACH (snapDanglingEnds, 3 m + the
    # target's half width) and two car ways of the same layer may turn into each other where they cross
    # without a shared node (joined crossings). Only oneway and turn restrictions bind. Exit ids stay
    # those of the converted network.
    all_ways = drivable_ways(osm)
    psegs = clip_segments(osm, all_ways, clip, hw_use)
    p_deg = collections.Counter()
    for w, i, a, b, ra, rb, ca, cb in psegs:
        p_deg[a] += ra
        p_deg[b] += rb
    p_starts, p_ends, p_states, p_exit = collections.defaultdict(list), collections.defaultdict(list), [], {}
    for w, i, a, b, ra, rb, ca, cb in psegs:
        ow = way_oneway(all_ways[w][1])
        for d in ((1,) if ow == 1 else (-1,) if ow == -1 else (1, -1)):
            st = (w, i, d)
            (u, ru), (v, rv) = ((a, ra), (b, rb)) if d == 1 else ((b, rb), (a, ra))
            p_states.append(st)
            if ru:
                p_starts[u].append(st)
            if rv:
                p_ends[v].append(st)
                if w in ways and deg[v] == 1:
                    p_exit[st] = f"n:{v}"
            elif w in ways:
                p_exit[st] = f"e:{w}:{i}"
    p_succ = collections.defaultdict(list)
    for nd, ins in p_ends.items():
        for st in ins:
            for nx in p_starts.get(nd, []):
                if nx[0] != st[0] and ((st[0], nd, nx[0]) in r_no or r_only.get((st[0], nd), nx[0]) != nx[0]):
                    continue
                p_succ[st].append(nx)
    for n1, n2 in repair_joins(osm, psegs, p_deg):
        for st in p_ends.get(n1, []):
            p_succ[st].extend(p_starts.get(n2, []))
        for st in p_ends.get(n2, []):
            p_succ[st].extend(p_starts.get(n1, []))
    # ---- exit ids of the lane graph's terminals
    dead = [nd for nd in deg if deg[nd] == 1 and nd in osm.nodes and inside.get(nd)]
    dead_xy = np.array([osm.nodes[nd] for nd in dead]) if dead else np.zeros((0, 2))
    term_id = {}
    for (rid, at_end) in flow.terminals:
        r = od.roads[rid]
        x, y, _ = end_pose(r, at_end)
        # the nearest of: an OSM dead-end node, or a box cut of one of the road's own ways (the converter
        # cuts exactly on the box); both within DEADEND_TOL (spec eps_B). Nearest wins, so a clipped stub
        # shorter than DEADEND_TOL maps its two ends to the dead end and to the cut respectively.
        cands = []
        if len(dead_xy):
            dd = np.hypot(dead_xy[:, 0] - x, dead_xy[:, 1] - y)
            j = int(np.argmin(dd))
            cands.append((float(dd[j]), f"n:{dead[j]}"))
        for w in r.osm_ways:
            for i, p in cross_pts.get(w, []):
                cands.append((math.hypot(p[0] - x, p[1] - y), f"e:{w}:{i}"))
        cands = [c for c in cands if c[0] <= DEADEND_TOL]
        term_id[(rid, at_end)] = min(cands)[1] if cands else None
    lane_exit_ids = sorted({term_id[k] for k, v in flow.terminals.items() if v["snk"] and term_id[k]})
    bit = {e: 1 << i for i, e in enumerate(lane_exit_ids)}
    # ---- OSM reachability
    own_o = {st: bit.get(e, 0) for st, e in exit_of.items()}
    ob = reach_bits(states, lambda v: succ.get(v, ()), own_o)
    pb = reach_bits(p_states, lambda v: p_succ.get(v, ()), {st: bit.get(e, 0) for st, e in p_exit.items()})
    # ---- lane reachability
    own_l = {}
    for k, v in flow.terminals.items():
        b = bit.get(term_id[k], 0)
        for n in v["snk"]:
            own_l[n] = own_l.get(n, 0) | b
    lb = reach_bits(sorted(flow.nodes, key=str),
                    lambda v: list(flow.succ.get(v, ())) + list(flow.lat.get(v, ())), own_l)
    # U-turns are an interpretation (missing_uturn), also when they are composite: OSM may route A -> short
    # cross street -> B with two legal turns that the converter merges into ONE junction, where A -> B is a
    # reversal. Second lane game with virtual U-turn edges at every junction: from each lane arriving at
    # the junction to each lane leaving it whose travel heading differs by more than UTURN_DEG. A miss that
    # disappears with them is exit_unreachable_uturn (interpretation); one that stays is exit_unreachable.
    uturn = collections.defaultdict(set)
    arr, dep = collections.defaultdict(list), collections.defaultdict(list)
    for r in od.plain_roads():
        for at_end, lk in ((True, r.succ), (False, r.pred)):
            if lk is None or lk.type != "junction":
                continue
            k = len(r.sections) - 1 if at_end else 0
            h = float(heading_curv(r, r.length if at_end else 0.0)[0][0])
            for n in flow.by_rs.get((r.id, k), []):
                hd = h if n[3] == 1 else h + math.pi
                ((arr if (n[3] == 1) == at_end else dep)[lk.id]).append((n, hd))
    for j, lst in arr.items():
        for n, ha in lst:
            for m, hd in dep.get(j, []):
                if abs(math.degrees(_wrap(hd - ha))) > UTURN_DEG:
                    uturn[n].add(m)
    lb_u = reach_bits(sorted(flow.nodes, key=str), lambda v: list(flow.succ.get(v, ())) + list(flow.lat.get(v, ()))
                      + list(uturn.get(v, ())), own_l)
    # ---- compare per entry (an entry = one OSM entry id; all lane terminals mapped to it together)
    term_by_id = collections.defaultdict(list)
    for k in sorted(flow.terminals, key=str):
        if term_id[k]:
            term_by_id[term_id[k]].append(k)
    F.info["game_entries"] += sum(1 for E in term_by_id if E in entries)
    F.info["game_exits"] += len(lane_exit_ids)

    def where(E, kind):
        """location of an entry / exit id: its first lane terminal with source / sink lanes."""
        for k in term_by_id[E]:
            lanes = flow.terminals[k]["src" if kind == "entry" else "snk"]
            if lanes:
                r = od.roads[k[0]]
                x, y, _ = end_pose(r, k[1])
                return dict(road=r.id, lane=min(n[2] for n in lanes), s=r.length if k[1] else 0.0, x=x, y=y)
        k = term_by_id[E][0]
        r = od.roads[k[0]]
        x, y, _ = end_pose(r, k[1])
        return dict(road=r.id, s=r.length if k[1] else 0.0, x=x, y=y)

    strict, soft, illegal, reach_osm = set(), set(), set(), collections.Counter()
    for E in sorted(term_by_id):
        src = set().union(*(flow.terminals[k]["src"] for k in term_by_id[E]))
        if src:
            got = 0
            for n in src:
                got |= lb.get(n, 0)
            perm = 0
            for st in entries.get(E, ()):
                perm |= pb.get(st, 0)
            for X in lane_exit_ids:
                if got & bit[X] and not perm & bit[X] and X != E:  # E == X: entry and exit at one OSM place
                    illegal.add((E, X))
        if E not in entries:
            continue
        want = 0
        for st in entries[E]:
            want |= ob.get(st, 0)
        if not want:
            continue
        for X in lane_exit_ids:
            if want & bit[X]:
                reach_osm[X] += 1
        if not src:
            F.add("entry_missing", value=E, limit=f"OSM reaches {bin(want).count('1')} exits from here",
                  **where(E, "entry"))
            continue
        got = got_u = 0
        for n in src:
            got |= lb.get(n, 0)
            got_u |= lb_u.get(n, 0)
        got_u |= got
        for X in lane_exit_ids:
            if X == E and E.startswith("e:"):
                continue  # both ends of one clipped stub: one OSM place, not a route
            if want & bit[X] and not got_u & bit[X]:
                strict.add((E, X))
            elif want & bit[X] and not got & bit[X]:
                soft.add((E, X))
    # Report missed (entry, exit) pairs as a minimal set of causes: greedy set cover by 'exit X is missed
    # from these entries' or 'entry E misses these exits' (one root defect next to an exit makes it
    # unreachable from every entry; one instance names it). Deterministic tie-break.
    blocked = set()
    for name, pairs in (("exit_unreachable", strict), ("exit_unreachable_uturn", soft), ("exit_illegal", illegal)):
        if name == "exit_unreachable_uturn":  # misses that share an entry/exit with a strict cause
            pairs = {p for p in pairs if ("entry", p[0]) not in blocked and ("exit", p[1]) not in blocked}
        if name == "exit_illegal":
            blocked = set()
        left = set(pairs)
        while left:
            by = collections.defaultdict(set)
            for E, X in left:
                by[("exit", X)].add((E, X))
                by[("entry", E)].add((E, X))
            (kind, gid), cov = min(by.items(), key=lambda kv: (-len(kv[1]), kv[0][0] != "exit", kv[0][1]))
            left -= cov
            blocked.add((kind, gid))
            if kind == "exit":
                other = sorted(E for E, _ in cov)
                val = (f"exit {gid} reached from {len(cov)} entries that cannot reach it in OSM" if
                       name == "exit_illegal" else
                       f"exit {gid} unreachable from {len(cov)} of {reach_osm[gid]} entries that reach it in OSM")
            else:
                other = sorted(X for _, X in cov)
                val = (f"entry {gid} reaches {len(cov)} exits OSM cannot reach from it" if name == "exit_illegal"
                       else f"entry {gid} misses {len(cov)} exits OSM reaches from it")
            F.add(name, value=val, limit=0, cause=kind, id=gid, pairs=len(cov), others=other[:8],
                  **where(gid, "exit" if kind == "exit" else "entry"))
        F.info[name + "_pairs"] += len(pairs)
    if debug is not None:  # introspection for tests / triage
        debug.update(locals())


def check_osm(od, osm, meta, F, flow, log_info):
    lht = meta["lht"]
    clip = meta.get("clip_xy")
    ways = drivable_ways(osm, od)
    check_dropped(od, osm, meta, F)
    wr = way_roads(od)
    exp = meta["expect"]
    # --- shape: reference line vs its OSM ways
    node_ways = collections.defaultdict(set)
    arms = collections.Counter()
    for w, (nds, t) in ways.items():
        for i, nd in enumerate(nds):
            node_ways[nd].add(w)
            arms[nd] += 1 if i in (0, len(nds) - 1) else 2
    jn = [osm.nodes[nd] for nd, c in arms.items() if c >= 3 and nd in osm.nodes]
    jn = np.array(jn) if jn else np.zeros((0, 2))
    for r in od.plain_roads():
        own = [w for w in r.osm_ways if w in ways]
        # the road may run on into the way that continues its own (e.g. a gore moves a split downstream)
        near = set(own) | {v for w in own for nd in (ways[w][0][0], ways[w][0][-1]) for v in node_ways[nd]}
        polys = [osm.polyline(w) for w in near]
        polys = [p for p in polys if len(p) >= 2]
        if not polys or r.length < 1:
            continue
        s = np.linspace(0, r.length, max(3, int(r.length / 1.0) + 1))
        x, y, h = r.ref(s)
        d = np.min([seg_dist(x, y, p)[0] for p in polys], axis=0)
        if len(jn):  # junction areas (>= 3 arms within JUNCTION_ZONE) are covered by the lane-level checks
            m = ((jn[:, 0] > x.min() - JUNCTION_ZONE) & (jn[:, 0] < x.max() + JUNCTION_ZONE) &
                 (jn[:, 1] > y.min() - JUNCTION_ZONE) & (jn[:, 1] < y.max() + JUNCTION_ZONE))
            if m.any():
                jj = jn[m]
                dj = np.min(np.hypot(x[:, None] - jj[None, :, 0], y[:, None] - jj[None, :, 1]), axis=1)
                d = np.where(dj > JUNCTION_ZONE, d, 0.0)
        if clip is not None:  # the clip cut is a network edge, not a shape change
            d = np.where(inside_box(clip, x, y) > CORNER_TOL, d, 0.0)
        i = int(np.argmax(d))
        straight = all(exp.get(w, {}).get("straight") for w in r.osm_ways)
        if any("placement" in exp.get(w, {}) for w in r.osm_ways):
            continue  # the reference line may legitimately sit off the way; placement_offset checks it
        lim = PLACEMENT_TOL if straight else CORNER_TOL
        if d[i] > lim:
            F.add("shape_deviation", road=r.id, s=s[i], x=x[i], y=y[i], value=float(d[i]), limit=lim,
                  note="straight way" if straight else "")
        if straight:  # a straight OSM way must give a straight road: zero curvature everywhere
            _, kap = heading_curv(r, s)
            j = int(np.argmax(np.abs(kap)))
            k0 = 8 * POS_TOL / max(r.length, 1.0) ** 2  # spec NAT-5 kappa_0: sagitta of eps_pos over the length
            if abs(kap[j]) > k0:
                F.add("shape_deviation", road=r.id, s=s[j], x=x[j], y=y[j], value=float(kap[j]),
                      limit=float(f"{k0:.3g}"), note="curvature on a straight way")
    # --- every road of a car way must carry a drivable lane; road ends without links at OSM dead ends only
    deg = collections.Counter()
    ends = set()
    for w, (nds, t) in ways.items():
        for i, nd in enumerate(nds):
            deg[nd] += 1 if i in (0, len(nds) - 1) else 2
        ends |= {nds[0], nds[-1]}
    dead = [osm.nodes[nd] for nd in ends if deg[nd] == 1 and nd in osm.nodes]
    dead = np.array(dead) if dead else np.zeros((0, 2))
    for r in od.plain_roads():
        if not any(ln.type in DRIVE for sec in r.sections for ln in sec.lanes()):
            x, y, _ = r.ref(r.length / 2)
            F.add("road_without_drive_lane", road=r.id, x=x, y=y, value=",".join(r.osm_ways))
        for at_end, lk in ((True, r.succ), (False, r.pred)):
            if lk is not None:
                continue
            x, y, _ = end_pose(r, at_end)
            dd = float(np.hypot(dead[:, 0] - x, dead[:, 1] - y).min()) if len(dead) else 1e9
            if clip is not None and float(inside_box(clip, np.array([x]), np.array([y]))[0]) <= DEADEND_TOL:
                continue  # cut by the --clip box: the map edge
            if dd > DEADEND_TOL:
                F.add("unlinked_road_end", road=r.id, s=r.length if at_end else 0, x=x, y=y,
                      value=round(dd, 2), limit=DEADEND_TOL, note="nearest OSM dead end (m)")
    # --- lane counts, widths, placement per expected way (majority over the way's length)
    for w, e in exp.items():
        if w not in osm.ways or w not in wr:
            continue
        poly = osm.polyline(w)
        stats = collections.Counter()
        wl_f, wl_b, wsum, place = [], [], [], []
        for r in wr[w]:
            others = [osm.polyline(o) for o in r.osm_ways if o != w and o in osm.ways]
            s = np.linspace(0, r.length, max(3, int(r.length / 2.0) + 1))
            x, y, h = r.ref(s)
            d, du, q = seg_dist(x, y, poly)
            mine = np.ones(len(s), bool)  # samples belong to the nearest of the road's ways
            for o in others:
                mine &= d <= seg_dist(x, y, o)[0]
            for j in np.nonzero(mine)[0]:
                sv = float(s[j])
                sec = r.section_at(sv)
                aligned = 1 if math.cos(h[j]) * du[j][0] + math.sin(h[j]) * du[j][1] > 0 else -1
                fl, b = travel_order(r, sec, sv, aligned, lht)
                bl, _ = travel_order(r, sec, sv, -aligned, lht)
                fl = [l for l in fl if l.type != "bidirectional" and b[l.id][1] != b[l.id][0]]
                bl = [l for l in bl if l.type != "bidirectional" and b[l.id][1] != b[l.id][0]]
                stats[(len(fl), len(bl))] += 1
                if "width_lanes_fwd" in e or "width_lanes_bwd" in e or "width" in e or "placement" in e:
                    wf = [abs(b[l.id][1] - b[l.id][0]) for l in fl]
                    wb = [abs(b[l.id][1] - b[l.id][0]) for l in bl]
                    wl_f.append(wf), wl_b.append(wb)
                    allb = [v for l in fl + bl for v in b[l.id]]
                    wsum.append(sum(wf) + sum(wb))
                    # OSM line position in road t (left of the road's s direction)
                    t_osm = -(q[j][0] - x[j]) * math.sin(h[j]) + (q[j][1] - y[j]) * math.cos(h[j])
                    place.append((sv, x[j], y[j], aligned, t_osm, [(l.id, b[l.id]) for l in fl],
                                  (min(allb), max(allb)) if allb else None, r.id))
        if not stats:
            continue
        F.info["ways_lane_checked"] += 1
        (fc, bc), _ = stats.most_common(1)[0]
        mid_r = wr[w][0]
        x, y, _ = mid_r.ref(mid_r.length / 2)
        if "fwd" in e and (fc, bc) != (e["fwd"], e["bwd"]):
            F.add("lane_count_mismatch", road=",".join(r.id for r in wr[w]), x=x, y=y,
                  value=f"fwd/bwd {fc}/{bc}", limit=f"{e['fwd']}/{e['bwd']}", way=w)
        elif "total" in e and fc + bc != e["total"]:
            F.add("lane_count_mismatch", road=",".join(r.id for r in wr[w]), x=x, y=y, value=f"total {fc + bc}",
                  limit=e["total"], way=w)
        # widths (median over samples with the majority lane layout)
        if wl_f:
            sel = [i for i in range(len(wl_f)) if (len(wl_f[i]), len(wl_b[i])) == (fc, bc)]
            if sel:
                for key, arr in (("width_lanes_fwd", wl_f), ("width_lanes_bwd", wl_b)):
                    if key in e and len(e[key]) == len(arr[sel[0]]):
                        got = np.median(np.array([arr[i] for i in sel]), axis=0)
                        if np.abs(got - np.array(e[key])).max() > TAG_TOL:
                            F.add("width_mismatch", x=x, y=y, value=[round(v, 2) for v in got], limit=e[key],
                                  way=w, tag=key)
                if "width" in e:
                    got = float(np.median([wsum[i] for i in sel]))
                    if abs(got - e["width"]) > TAG_TOL:
                        F.add("width_mismatch", x=x, y=y, value=round(got, 3), limit=e["width"], way=w,
                              tag="width (sum of driving lanes)")
                if "placement" in e:
                    devs = []
                    for i in sel:
                        sv, px, py, aligned, t_osm, fl, cw, rid = place[i]
                        v = e["placement"]
                        if v == "default":
                            tt = 0.5 * (cw[0] + cw[1]) if cw else None
                        else:
                            kind, kk = v.split(":")
                            kk = int(kk)
                            if kk > len(fl):
                                continue
                            lid, (ti, to) = fl[kk - 1]
                            # left/right in travel direction: left = +t when aligned with s
                            lo, hi = sorted((ti, to))
                            left, right = (hi, lo) if aligned == 1 else (lo, hi)
                            tt = {"left_of": left, "right_of": right, "middle_of": 0.5 * (ti + to)}[kind]
                        if tt is not None:
                            devs.append((abs(tt - t_osm), px, py, rid, sv, tt, t_osm))
                    if devs:
                        devs.sort()
                        med = devs[len(devs) // 2]
                        if med[0] > PLACEMENT_TOL:
                            F.add("placement_offset", road=med[3], s=med[4], x=med[1], y=med[2],
                                  value=round(med[0], 3), limit=PLACEMENT_TOL, way=w, placement=e["placement"],
                                  expected_t=round(med[5], 3), osm_t=round(med[6], 3))
    # --- junction connectivity from the OSM topology (and turn:lanes)
    check_arms(od, osm, meta, F, flow, ways)
    # --- the exit game per entry (user's natural-flow game, all start points)
    check_exit_game(od, osm, meta, F, flow, ways)
    # --- natural flow: superfluous sections and road splits
    for r in od.plain_roads():
        for k in range(len(r.sections) - 1):
            a, b = r.sections[k], r.sections[k + 1]
            # spec NAT-2: removable when both sections have the same lane ids and types and every lane
            # links 1:1 to the same id (declared by either side, nothing else named). Widths, marks, speed
            # do not matter (they have sOffset records); a taper cut in two at the boundary is removable too.
            ia = {(l.id, l.type) for l in a.lanes()}
            ib = {(l.id, l.type) for l in b.lanes()}
            LLs = lane_links(od)
            ident = all(set(LLs.get((r.id, k, l.id), ([], []))[1]) | set(LLs.get((r.id, k + 1, l.id), ([], []))[0])
                        == {l.id} for l in a.lanes())
            if ia == ib and ident:
                x, y, _ = r.ref(b.s)
                F.add("superfluous_section", road=r.id, s=b.s, x=x, y=y, value=f"{len(ia)} identical lanes")
    # Road-to-road links between plain roads, at BOTH ends of every plain road (contact start-start /
    # end-end links exist when the next road runs the other way), each link once.
    #  superfluous_road_split (spec NAT-3, the user's rule): identical lanes (ids mirrored for an opposite-s
    #    contact, types) and identity lane links: the road id changed although the lanes did not. Width,
    #    speed, surface, name, bridge or class changes are s-records / road attributes (spec NAT-1).
    #  plain_road_link (spec NAT-1): any other plain road-road link. A road ends only at a junction, the map
    #    boundary, a dead end or a gore junction; a lane-count change inside a chain is a lane section.
    LL = lane_links(od)
    seen_split = set()
    for r in od.plain_roads():
        for at_end, lk in ((True, r.succ), (False, r.pred)):
            if lk is None or lk.type != "road":
                continue
            T = od.roads.get(lk.id)
            if T is None or T.junction != "-1" or lk.contact not in ("start", "end"):
                continue
            key = frozenset(((r.id, at_end), (T.id, lk.contact == "end")))
            if key in seen_split:
                continue
            seen_split.add(key)
            ka = len(r.sections) - 1 if at_end else 0
            kb = 0 if lk.contact == "start" else len(T.sections) - 1
            a, b = r.sections[ka], T.sections[kb]
            # same direction of s (end->start or start->end) keeps ids; opposite s mirrors them
            flip = 1 if at_end == (lk.contact == "start") else -1
            ta = sorted((l.id, l.type) for l in a.lanes() if l.id != 0)
            tb = sorted((l.id * flip, l.type) for l in b.lanes() if l.id != 0)
            # identity links, declared by either side: r's lane names only its mirror on T, T's mirror lane
            # names only r's lane, and at least one of them declares the link
            fwd = {l.id: set(LL.get((r.id, ka, l.id), ([], []))[1 if at_end else 0]) for l in a.lanes()}
            back = {l.id: set(LL.get((T.id, kb, l.id * flip), ([], []))[0 if lk.contact == "start" else 1])
                    for l in a.lanes()}
            ok_links = all(fwd[i] <= {i * flip} and back[i] <= {i} and (fwd[i] or back[i]) for i in fwd)
            tags_a = [osm.ways[w][1] for w in r.osm_ways if w in osm.ways]
            tags_b = [osm.ways[w][1] for w in T.osm_ways if w in osm.ways]
            diff = sorted({k for ta_ in tags_a for tb_ in tags_b for k in set(ta_) | set(tb_)
                           if ta_.get(k) != tb_.get(k)})
            x, y, _ = end_pose(r, at_end)
            same = ta == tb and ok_links and (r.el.get("rule") or "") == (T.el.get("rule") or "") and T.id != r.id
            F.add("superfluous_road_split" if same else "plain_road_link", road=r.id,
                  s=r.length if at_end else 0.0, x=x, y=y,
                  value=f"{r.id}.{'end' if at_end else 'start'} -> {T.id}.{lk.contact}",
                  ways=sorted(set(r.osm_ways) | set(T.osm_ways)), tag_diff=diff[:12])


def turn_restrictions(osm):
    """OSM turn restrictions with a via node: ({(from, via, to)} forbidden, {(from, via): to} only)."""
    no, only = set(), {}
    for tags, members in osm.relations:
        if tags.get("type") != "restriction":
            continue
        v = tags.get("restriction") or tags.get("restriction:motorcar") or tags.get("restriction:vehicle") or ""
        fr = [r for t, r, role in members if t == "way" and role == "from"]
        to = [r for t, r, role in members if t == "way" and role == "to"]
        via = [r for t, r, role in members if t == "node" and role == "via"]
        if not (fr and to and via):
            continue  # via-way restrictions are not modelled here (the pair is then not judged)
        if v.startswith("no_"):
            no.add((fr[0], via[0], to[0]))
        elif v.startswith("only_"):
            only[(fr[0], via[0])] = to[0]
    return no, only


def check_arms(od, osm, meta, F, flow, ways):
    """For every OSM node where car ways meet (or one way changes), every legal movement arm A -> arm B
    must have a lane path; lanes with turn:lanes may only reach the arms their indication allows."""
    r_no, r_only = turn_restrictions(osm)
    occ = collections.defaultdict(list)
    for w, (nds, t) in ways.items():
        for i, nd in enumerate(nds):
            occ[nd].append((w, i))
    wr = way_roads(od)
    for nd, lst in occ.items():
        if len(lst) < 2 or nd not in osm.nodes:
            continue
        N = np.array(osm.nodes[nd])
        arms = []
        for w, i in lst:
            nds, t = ways[w]
            ow = way_oneway(t)
            for step in (-1, 1):
                j = i + step
                if j < 0 or j >= len(nds):
                    continue
                # direction from N along the arm, using the first node >= 3 m away
                q = None
                while 0 <= j < len(nds):
                    q = np.array(osm.nodes[nds[j]])
                    if np.linalg.norm(q - N) >= 3:
                        break
                    j += step
                dvec = (q - N) / max(np.linalg.norm(q - N), 1e-9)
                # entering N from this arm means travelling against `step` along the way
                can_in = ow == 0 or (ow == 1 and step == -1) or (ow == -1 and step == 1)
                can_out = ow == 0 or (ow == 1 and step == 1) or (ow == -1 and step == -1)
                anc = arm_anchor(wr.get(w, []), N, dvec)
                arms.append({"way": w, "step": step, "dir": dvec, "in": can_in, "out": can_out, "anchor": anc})
        if len(arms) < 2:
            continue
        # a node inside a single way with nothing else is not a junction
        if len({a["way"] for a in arms}) == 1 and len(arms) == 2:
            continue
        for a in arms:
            if a["anchor"] is None:
                continue
            a["in_nodes"], a["out_nodes"] = arm_nodes(od, a["anchor"], flow)
        local = arm_local_filter(od, N)
        for A in arms:
            if not A["in"] or A["anchor"] is None or not A.get("in_nodes"):
                continue
            other_out = set()
            for B in arms:
                if B.get("out_nodes"):
                    other_out |= set(B["out_nodes"])
            for B in arms:
                if B is A or not B["out"] or B["anchor"] is None:
                    continue
                theta = math.degrees(_wrap(math.atan2(B["dir"][1], B["dir"][0]) - math.atan2(-A["dir"][1], -A["dir"][0])))
                if not B.get("out_nodes"):
                    continue  # the arm has no outgoing lanes at all -> lane_count / flow checks report it
                mv = turn_tag(ways[A["way"]][1], A["step"] == -1)
                if mv and not turn_allows(mv, theta):
                    continue  # turn:lanes of the approach forbid this movement
                if (A["way"], nd, B["way"]) in r_no or r_only.get((A["way"], nd), B["way"]) != B["way"]:
                    continue  # forbidden by a turn restriction relation
                ok = path_exists(flow, A["in_nodes"], set(B["out_nodes"]), other_out - set(B["out_nodes"]), local)
                F.info["arm_pairs"] += 1
                if not ok:
                    uturn = abs(theta) > 150 or (A["way"] == B["way"] and A["step"] == B["step"])
                    x, y = N
                    F.add("missing_uturn" if uturn else "missing_connection", x=x, y=y,
                          value=f"way {A['way']} -> way {B['way']} (turn {theta:.0f} deg)", node=nd,
                          road=f"{A['anchor'][0].id}->{B['anchor'][0].id}")
            # turn:lanes
            mv = turn_tag(ways[A["way"]][1], A["step"] == -1)
            if mv and A["in_nodes"]:
                check_turn_lanes(od, flow, A, arms, mv, N, F, local, other_out, meta["lht"])


def arm_anchor(roads, N, dvec):
    """(road, s at the junction side, +1/-1 direction along s leading away from N into the arm)."""
    best = None
    for r in roads:
        for at_end in (False, True):
            s = r.length if at_end else 0.0
            x, y, _ = r.ref(s)
            d = math.hypot(x - N[0], y - N[1])
            sin = min(r.length, 5.0)
            x2, y2, _ = r.ref(r.length - sin if at_end else sin)
            v = np.array([x2 - x, y2 - y])
            if np.linalg.norm(v) < 1e-6 or float(v @ dvec) / np.linalg.norm(v) < 0.5:
                continue
            # the road must lie on the arm's side of N (a short road beyond N points the same way)
            if float(np.array([x2 - N[0], y2 - N[1]]) @ dvec) <= 0:
                continue
            if best is None or d < best[0]:
                best = (d, r, s, -1 if at_end else 1)
        # the way may pass through N inside a road (no split at N)
        s_n, t_n, dist = r.project(N[0], N[1])
        if dist < CORNER_TOL + 0.5 and 1.0 < s_n < r.length - 1.0:
            x, y, h = r.ref(s_n)
            along = 1 if math.cos(h) * dvec[0] + math.sin(h) * dvec[1] > 0 else -1
            if best is None or dist < best[0]:
                best = (dist, r, s_n, along)
    if best is None or best[0] > 400:
        return None
    return best[1], best[2], best[3]


def arm_nodes(od, anchor, flow):
    r, s, along = anchor
    sv = min(max(s + along * 0.05, 0), r.length)
    k = max(i for i, sec in enumerate(r.sections) if sec.s <= sv + 1e-9)
    here = flow.by_rs.get((r.id, k), [])
    return [n for n in here if n[3] == -along], [n for n in here if n[3] == along]


def arm_local_filter(od, N, radius=450.0):
    """roads with a reference-line sample within `radius` of N (the search stays at this junction)."""
    return {r.id for r in od.roads_near(N[0], N[1], radius)
            if np.hypot(r.X[::10] - N[0], r.Y[::10] - N[1]).min() <= radius}


def path_exists(flow, starts, targets, forbidden, local, lateral=True, limit=20000):
    starts = set(starts)
    seen, stack = set(starts), list(starts)
    while stack and len(seen) < limit:
        n = stack.pop()
        if n in targets:
            return True
        if n in forbidden and n not in starts:
            continue
        nxt = list(flow.succ.get(n, ()))
        if lateral:
            nxt += list(flow.lat.get(n, ()))
        for m in nxt:
            if m not in seen and m[0] in local:
                seen.add(m)
                stack.append(m)
    return any(n in targets for n in seen)


def turn_tag(tags, forward):
    """turn:lanes value list for travel along (forward) or against the way, or None."""
    ow = way_oneway(tags)
    keys = (["turn:lanes"] if ow == (1 if forward else -1) else []) +         ["turn:lanes:" + ("forward" if forward else "backward")]
    for k in keys:
        if k in tags:
            return tags[k].split("|")
    return None


def turn_allows(movements, theta):
    """does any lane's indication allow a movement at signed angle theta (deg, + = left)?"""
    any_specific = False
    for mv in movements:
        for m in mv.split(";"):
            if not m or m == "none":
                return True  # an unmarked lane: no restriction from this tag
            any_specific = True
            lo, hi = TURN_RANGES.get(m, (-180, 180))
            if lo <= theta <= hi:
                return True
    return not any_specific


TURN_RANGES = {  # degrees, positive = left (counter-clockwise), applies to RHT and LHT alike
    "through": (-45, 45), "left": (20, 160), "slight_left": (5, 70), "sharp_left": (90, 180),
    "right": (-160, -20), "slight_right": (-70, -5), "sharp_right": (-180, -90),
    "merge_to_left": (-45, 45), "merge_to_right": (-45, 45), "reverse": (150, 180),
}


def check_turn_lanes(od, flow, A, arms, movements, N, F, local, other_out, meta_lht):
    """turn:lanes order is left -> right in travel direction (OSM wiki Key:turn)."""
    r, s, along = A["anchor"]
    d = -along
    sv = min(max(s + along * 0.05, 0), r.length)
    sec = r.section_at(sv)
    k = r.sections.index(sec)
    lanes, bb = travel_order(r, sec, sv, d, meta_lht)
    # turn:lanes counts the lanes of this direction only: a lanes:both_ways centre lane is not in it, and a
    # lane that has not opened yet (zero-width taper at the stop line) is not a lane there
    lanes = [ln for ln in lanes if ln.type != "bidirectional" and abs(bb[ln.id][1] - bb[ln.id][0]) > WIDTH_EPS]
    if len(lanes) != len(movements):
        F.add("turn_movement_missing", road=r.id, x=N[0], y=N[1], value=f"{len(lanes)} approach lanes",
              limit=f"{len(movements)} turn:lanes entries")
        return
    reach = {}
    F.info["turn_lane_approaches"] += 1
    for B in arms:
        if B is A or not B["out"] or not B.get("out_nodes"):
            continue
        theta = math.degrees(_wrap(math.atan2(B["dir"][1], B["dir"][0]) - math.atan2(-A["dir"][1], -A["dir"][0])))
        for ln in lanes:
            ok = path_exists(flow, [(r.id, k, ln.id, d)], set(B["out_nodes"]), other_out - set(B["out_nodes"]),
                             local, lateral=False)
            reach.setdefault(ln.id, []).append((theta, ok, B["way"]))
    for ln, mv in zip(lanes, movements):
        allowed = [m for m in mv.split(";") if m and m != "none"]
        if not allowed:
            continue
        for theta, ok, bw in reach.get(ln.id, []):
            fits = any(TURN_RANGES.get(m, (-180, 180))[0] <= theta <= TURN_RANGES.get(m, (-180, 180))[1] for m in allowed)
            if ok and not fits:
                F.add("turn_lane_violation", road=r.id, lane=ln.id, x=N[0], y=N[1],
                      value=f"reaches way {bw} at {theta:.0f} deg", limit=mv)
        for m in allowed:
            lo, hi = TURN_RANGES.get(m, (-180, 180))
            cands = [(theta, ok) for theta, ok, _ in reach.get(ln.id, []) if lo <= theta <= hi]
            if cands and not any(ok for _, ok in cands):
                F.add("turn_movement_missing", road=r.id, lane=ln.id, x=N[0], y=N[1],
                      value=f"'{m}' has no connection", limit=mv)


# ----------------------------------------------------------------------------- external checkers
def run_external(xodr, osm, meta, work):
    """invariants.py / laneflow.py from this directory, if present (imported, evaluated under the case's
    traffic rule because the carla target writes no road@rule). Instances are merged under
    ext:<module>:<invariant>; laneflow's 'ok' / 'info' classifications are not failures."""
    out, errors = {}, {}
    rule = "LHT" if meta["lht"] else "RHT"
    if os.path.exists(os.path.join(HERE, "invariants.py")):
        try:
            inv = importlib.import_module("invariants")
            net = inv.Network(xodr)
            o = G.Osm(osm, net.lat0, net.lon0)
            res = inv.run_all(net, o, None, rule.lower()).to_json()
            for k, v in res.items():
                if isinstance(v, dict) and v.get("count"):
                    out[f"ext:invariants:{k}"] = {"count": v["count"], "instances": v.get("instances", [])[:10]}
        except Exception:
            errors["invariants"] = traceback.format_exc()[-600:]
    if os.path.exists(os.path.join(HERE, "laneflow.py")):
        try:
            lfm = importlib.import_module("laneflow")
            _, d = lfm.run(xodr, osm, None, rule)
            for k, v in d.items():
                if k.startswith("_") or not isinstance(v, dict) or not v.get("count"):
                    continue
                if v.get("severity", "defect") != "defect":
                    continue
                out[f"ext:laneflow:{k}"] = {"count": v["count"], "instances": v.get("instances", [])[:10]}
        except Exception:
            errors["laneflow"] = traceback.format_exc()[-600:]
    return out, errors


# ----------------------------------------------------------------------------- one case
def check_case(xodr, osm_path, meta, log_info, work=None, external=True):
    F = Findings()
    od = G.OpenDrive(xodr)
    osm = G.Osm(osm_path, od.lat0 or meta["origin"][0], od.lon0 or meta["origin"][1])
    if meta.get("clip") and not meta.get("clip_xy"):
        c = [float(v) for v in meta["clip"].split(",")]
        # min / max over the four projected corners, exactly as tools/osm2xodr.cpp builds opt.clipBox
        xs, ys = G.tm_forward(np.array([c[0], c[0], c[2], c[2]]), np.array([c[1], c[3], c[1], c[3]]), od.lat0, od.lon0)
        meta = dict(meta, clip_xy=(float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())))
    check_structure(od, F, meta["lht"])
    osmflow = OsmFlow(osm, drivable_ways(osm, od))
    flow = Flow(od, meta["lht"], F, osmflow)
    flow.check(osmflow)
    check_osm(od, osm, meta, F, flow, log_info)
    res = F.result()
    ext_err = {}
    if external and work:
        ext, ext_err = run_external(xodr, osm_path, meta, work)
        res.update(ext)
    stats = {"roads": len(od.plain_roads()), "connecting": len(od.connecting_roads()),
             "junctions": len(od.junctions), "lane_nodes": len(flow.nodes),
             "sources": len(flow.sources), "sinks": len(flow.sinks), **F.info}
    return res, stats, ext_err


def run_case(job):
    """job: (case dir, meta, options). Converts and checks one case; writes result.json."""
    cdir, meta, opt = job
    t0 = time.time()
    osm = os.path.join(cdir, "in.osm")
    xodr = os.path.join(cdir, "out.xodr")
    res = {"id": meta["id"], "kind": meta["kind"], "country": meta["country"], "features": meta["features"]}
    checks = {}
    info, log = convert(osm, xodr, meta["country"], meta["origin"], "carla", opt.get("exe", EXE))
    open(os.path.join(cdir, "log.txt"), "w", encoding="utf-8").write(" ".join(info["cmd"]) + "\n" + log)
    res["convert"] = {k: info[k] for k in ("rc", "seconds", "validation_errors", "validation_warnings", "rules", "fit_max")}
    if not info["ok"]:
        checks["conversion_failed"] = {"count": 1, "instances": [inst(value=f"rc={info['rc']}", limit=0,
                                                                         log=log[-400:])]}
    else:
        if info["validation_errors"]:
            checks["converter_validation_errors"] = {"count": info["validation_errors"], "instances": [
                inst(value=f"{k}: {v}") for k, v in info["rules"].items()][:10]}
        try:
            c, stats, ext_err = check_case(xodr, osm, meta, info, cdir, opt.get("external", True))
            checks.update(c)
            res["stats"] = stats
            if ext_err:
                res["external_errors"] = ext_err
        except Exception:
            checks["checker_crash"] = {"count": 1, "instances": [inst(value=traceback.format_exc()[-800:])]}
        if opt.get("xsd"):
            x18 = os.path.join(cdir, "out18.xodr")
            i18, l18 = convert(osm, x18, meta["country"], meta["origin"], "asam18", opt.get("exe", EXE))
            if not i18["ok"]:
                checks["conversion_failed"] = {"count": 1, "instances": [inst(value="asam18 rc=%s" % i18["rc"],
                                                                                 log=l18[-400:])]}
            else:
                errs = xsd_errors(x18)
                if errs:
                    checks["xsd_invalid"] = {"count": len(errs), "instances": [inst(value=e) for e in errs]}
                if not opt.get("keep"):
                    os.remove(x18)
        if opt.get("esmini") and os.path.exists(ODRPLOT):
            rc, lg, _ = run([ODRPLOT, xodr, os.path.join(cdir, "track.csv")], timeout=60, cwd=cdir)
            bad = [l for l in lg.splitlines() if re.search(r"error|fail|exception", l, re.I)
                   and "traffic_signals" not in l]
            if bad or rc != 0:
                checks["esmini_load_error"] = {"count": max(1, len(bad)),
                                               "instances": [inst(value=b[:200]) for b in bad[:5]] or [inst(value=f"rc={rc}")]}
            tc = os.path.join(cdir, "track.csv")
            if os.path.exists(tc):
                os.remove(tc)
    res["checks"] = checks
    res["failed"] = sorted(k for k, v in checks.items() if v["count"])
    res["seconds"] = round(time.time() - t0, 2)
    json.dump(res, open(os.path.join(cdir, "result.json"), "w", encoding="utf-8"), indent=1)
    return res


# ----------------------------------------------------------------------------- shrinking
def _load_osm_xml(path):
    return ET.parse(path)


def _fails(osm_path, meta, check, opt, work):
    xodr = os.path.join(work, "s.xodr")
    info, log = convert(osm_path, xodr, meta["country"], meta["origin"], "carla", opt.get("exe", EXE))
    if check == "conversion_failed":
        return not info["ok"]
    if not info["ok"]:
        return False
    if check == "converter_validation_errors":
        return bool(info["validation_errors"])
    try:
        c, _, _ = check_case(xodr, osm_path, meta, info, None, external=False)
    except Exception:
        return check == "checker_crash"
    return bool(c.get(check, {}).get("count"))


def shrink(cdir, meta, check, opt, budget=60):
    """Greedy delta debugging: drop ways, then non-highway tags, while `check` keeps failing.
    Writes cdir/min/in.osm and returns its path (or None)."""
    work = os.path.join(cdir, "min")
    os.makedirs(work, exist_ok=True)
    cur = os.path.join(work, "in.osm")
    shutil.copy(os.path.join(cdir, "in.osm"), cur)
    trial = os.path.join(work, "trial.osm")
    tries = 0

    def attempt(tree):
        nonlocal tries
        tries += 1
        root = tree.getroot()
        used = {nd.get("ref") for w in root.iter("way") for nd in w.findall("nd")}
        for n in list(root.findall("node")):
            if n.get("id") not in used:
                root.remove(n)
        tree.write(trial, encoding="utf-8", xml_declaration=True)
        m2 = dict(meta)
        if _fails(trial, m2, check, opt, work):
            shutil.copy(trial, cur)
            return True
        return False

    if not _fails(cur, meta, check, opt, work):
        return None
    changed = True
    while changed and tries < budget:
        changed = False
        tree = _load_osm_xml(cur)
        ways = tree.getroot().findall("way")
        for w in ways:
            if tries >= budget or len(tree.getroot().findall("way")) <= 1:
                break
            t2 = _load_osm_xml(cur)
            root = t2.getroot()
            for ww in root.findall("way"):
                if ww.get("id") == w.get("id"):
                    root.remove(ww)
            if attempt(t2):
                changed = True
                break
    tree = _load_osm_xml(cur)
    for w in tree.getroot().findall("way"):
        for tg in w.findall("tag"):
            if tries >= budget or tg.get("k") == "highway":
                continue
            t2 = _load_osm_xml(cur)
            for ww in t2.getroot().findall("way"):
                if ww.get("id") == w.get("id"):
                    for x in ww.findall("tag"):
                        if x.get("k") == tg.get("k"):
                            ww.remove(x)
            attempt(t2)
    for f in ("trial.osm", "s.xodr"):
        p = os.path.join(work, f)
        if os.path.exists(p):
            os.remove(p)
    tree = _load_osm_xml(cur)
    return {"path": cur, "ways": len(tree.getroot().findall("way")), "tries": tries, "check": check}


# ----------------------------------------------------------------------------- self-test of the oracle
def _mutations():
    """(name, expected check, mutator(root) -> bool applied). Each breaks one exact property."""
    def plain_roads(root):
        return [r for r in root.findall("road") if r.get("junction", "-1") == "-1"]

    def drop_lanelink(root):
        for j in root.findall("junction"):
            for c in j.findall("connection"):
                for ll in c.findall("laneLink"):
                    c.remove(ll)
                    if not c.findall("laneLink"):
                        j.remove(c)
                    return True
        return False

    def shift_road(root):
        for r in plain_roads(root):
            if r.find("link") is not None and len(r.find("link")):
                for g in r.find("planView").findall("geometry"):
                    g.set("x", repr(float(g.get("x")) + 0.3))
                return True
        return False

    def negative_width(root):
        for ln in root.iter("lane"):
            w = ln.find("width")
            if w is not None and ln.get("type") == "driving":
                w.set("a", "-0.5")
                return True
        return False

    def drop_road_link(root):
        for r in plain_roads(root):
            lk = r.find("link")
            if lk is not None and lk.find("successor") is not None:
                lk.remove(lk.find("successor"))
                return True
        return False

    def bad_lane_target(root):
        for r in root.findall("road"):
            if r.get("junction", "-1") != "-1":
                for ln in r.iter("lane"):
                    lk = ln.find("link")
                    if lk is not None and lk.find("successor") is not None:
                        lk.find("successor").set("id", "37")
                        return True
        return False

    def drop_drive_lane(root):
        for r in plain_roads(root):
            for sec in r.find("lanes").findall("laneSection"):
                right = sec.find("right")
                lanes = right.findall("lane") if right is not None else []
                drv = [l for l in lanes if l.get("type") == "driving"]
                if len(drv) >= 1 and len(r.find("lanes").findall("laneSection")) == 1:
                    # remove the outermost driving lane; renumber the lanes outside it
                    out = min(int(l.get("id")) for l in drv)
                    for l in lanes:
                        if int(l.get("id")) == out:
                            right.remove(l)
                        elif int(l.get("id")) < out:
                            l.set("id", str(int(l.get("id")) + 1))
                    return True
        return False

    def widen_lane(root):
        for r in plain_roads(root):
            for ln in r.iter("lane"):
                w = ln.find("width")
                if w is not None and ln.get("type") == "driving":
                    w.set("a", repr(float(w.get("a")) + 0.4))
                    return True
        return False

    def offset_ref(root):
        for r in plain_roads(root):
            lo = r.find("lanes").find("laneOffset")
            if lo is not None:
                lo.set("a", repr(float(lo.get("a")) + 1.0))
                return True
        return False

    def planview_shift(root):
        for r in root.findall("road"):
            gs = r.find("planView").findall("geometry")
            if len(gs) >= 2:
                gs[1].set("x", repr(float(gs[1].get("x")) + 0.3))
                return True
        return False

    def sidewalk_pinch(root):
        for r in plain_roads(root):
            secs = r.find("lanes").findall("laneSection")
            if len(secs) != 1:
                continue
            for ln in secs[0].iter("lane"):
                w = ln.find("width")
                if ln.get("type") in ("sidewalk", "shoulder") and w is not None and len(ln.findall("width")) == 1:
                    L = float(r.get("length"))
                    a = float(w.get("a"))
                    w.set("b", repr(-2 * a / L))
                    w2 = ET.Element("width", sOffset=repr(L / 2), a="0", b=repr(2 * a / L), c="0", d="0")
                    ln.insert(list(ln).index(w) + 1, w2)
                    return True
        return False

    def duplicate_section(root):
        for r in plain_roads(root):
            lanes = r.find("lanes")
            secs = lanes.findall("laneSection")
            if len(secs) != 1 or any(len(l.findall("width")) != 1 or float(l.find("width").get("b")) != 0
                                     for l in secs[0].iter("lane") if l.get("id") != "0"):
                continue
            s2 = copy.deepcopy(secs[0])
            s2.set("s", repr(float(r.get("length")) / 2))
            lanes.append(s2)
            for sec, kind in ((secs[0], "successor"), (s2, "predecessor")):
                for ln in sec.iter("lane"):
                    if ln.get("id") == "0":
                        continue
                    lk = ln.find("link")
                    if lk is None:
                        lk = ET.Element("link")
                        ln.insert(0, lk)
                    for e in lk.findall(kind):
                        lk.remove(e)
                    ET.SubElement(lk, kind, id=ln.get("id"))
            return True
        return False

    def lane_id_sign(root):
        for r in plain_roads(root):
            right = r.find("lanes").find("laneSection").find("right")
            if right is not None and right.findall("lane"):
                right.findall("lane")[0].set("id", str(abs(int(right.findall("lane")[0].get("id")))))
                return True
        return False

    def drop_connecting_lane_pred(root):
        for r in root.findall("road"):
            if r.get("junction", "-1") == "-1":
                continue
            for ln in r.find("lanes").find("laneSection").iter("lane"):
                lk = ln.find("link")
                if lk is not None and lk.find("predecessor") is not None:
                    lk.remove(lk.find("predecessor"))
                    return True
        return False

    def one_sided_lane_link(root):
        for r in plain_roads(root):
            secs = r.find("lanes").findall("laneSection")
            if len(secs) < 2:
                continue
            for ln in secs[1].iter("lane"):
                lk = ln.find("link")
                if lk is not None and lk.find("predecessor") is not None and ln.get("type") == "driving":
                    lk.remove(lk.find("predecessor"))
                    return True
        return False

    return [("planView_element_shift_0.3m", ("planview_gap",), planview_shift),
            ("sidewalk_pinch", ("lane_pinch",), sidewalk_pinch),
            ("duplicate_lane_section", ("superfluous_section",), duplicate_section),
            ("lane_id_sign_flip", ("lane_id_invalid",), lane_id_sign),
            ("drop_connecting_lane_pred", ("junction_link_mismatch",), drop_connecting_lane_pred),
            ("one_sided_lane_link", ("lane_link_not_reciprocal",), one_sided_lane_link),
            ("drop_junction_laneLink", ("lane_into_junction_unconnected", "trap_lane", "missing_connection",
                                         "lane_from_junction_unconnected", "unreachable_lane"), drop_lanelink),
            ("shift_linked_road_0.3m", ("lane_link_gap", "road_link_gap"), shift_road),
            ("negative_lane_width", ("negative_width",), negative_width),
            ("drop_road_successor", ("unlinked_road_end", "link_not_reciprocal"), drop_road_link),
            ("lane_link_to_missing_lane", ("link_target_missing",), bad_lane_target),
            ("remove_driving_lane", ("lane_count_mismatch", "unlinked_lane", "link_target_missing"), drop_drive_lane),
            ("widen_driving_lane_0.4m", ("width_mismatch", "lane_link_gap", "placement_offset"), widen_lane),
            ("laneOffset_+1m", ("placement_offset", "lane_link_gap", "unlinked_lane"), offset_ref)]


def selftest(run_dir, limit=6):
    """Mutate passing outputs of a finished run and require the oracle to notice: proves each check can
    fail (a check that never fires is otherwise indistinguishable from a blind one)."""
    res = []
    cases = sorted(glob_dirs(os.path.join(run_dir, "cases")))
    for name, expect, fn in _mutations():
        tried = hit = 0
        for cdir in cases:
            r = json.load(open(os.path.join(cdir, "result.json"), encoding="utf-8"))
            if "conversion_failed" in r["failed"]:
                continue
            if name.startswith("widen") and "width" not in r["kind"]:
                continue
            if name.startswith("laneOffset") and r["kind"] != "placement":
                continue
            meta = json.load(open(os.path.join(cdir, "meta.json"), encoding="utf-8"))
            tree = ET.parse(os.path.join(cdir, "out.xodr"))
            if not fn(tree.getroot()):
                continue
            mx = os.path.join(cdir, "mutant.xodr")
            tree.write(mx, encoding="utf-8", xml_declaration=True)
            info = {"fit_max": None}
            try:
                c, _, _ = check_case(mx, os.path.join(cdir, "in.osm"), meta, info, None, external=False)
            except Exception:
                c = {"checker_crash": {"count": 1}}
            os.remove(mx)
            before = {k: v["count"] for k, v in r["checks"].items()}
            grew = [k for k in expect if c.get(k, {}).get("count", 0) > before.get(k, 0)]
            tried += 1
            hit += bool(grew)
            if tried >= limit:
                break
        res.append({"mutation": name, "expected_any_of": list(expect), "cases": tried, "detected": hit})
    return res


def glob_dirs(d):
    return [os.path.join(d, x) for x in os.listdir(d) if os.path.isdir(os.path.join(d, x))]


# ----------------------------------------------------------------------------- Jev second opinion
def jev_opinions(run_dir, summ, per_check=2):
    """Calibrated yes/no second opinion (jev.py, text only) on measured evidence of each built-in
    failure class. Never a gate: the exact checks decide; this only annotates the summary."""
    try:
        from jev import ask
    except Exception as ex:  # noqa: BLE001
        return {"error": repr(ex)}
    out = {}
    for c, v in summ["cases_by_check"].items():
        if c.startswith("ext:") or c not in CHECKS:
            continue
        ans = []
        for cid in v["examples"][:per_check]:
            r = json.load(open(os.path.join(run_dir, "cases", cid, "result.json"), encoding="utf-8"))
            i0 = r["checks"][c]["instances"][0]
            state = (f"Synthetic OpenStreetMap input ({r['kind']}, country {r['country']}, features "
                     f"{', '.join(r['features'])}) converted to an OpenDRIVE lane-level road network. Check "
                     f"'{c}': {CHECKS[c][1]}. Measured: {json.dumps(i0)}. Tolerance basis: positions 0.01 m "
                     f"(numeric precision), shape 1.5 m (the converter's documented corner tolerance).")
            try:
                a = ask(state, {"real": {"type": "noul", "instructions": "Is this a real defect of the generated "
                                         "lane-level road network (not an artefact of the measurement)?"}})
                ans.append({"case": cid, "p_real": a["real"].get("noul")})
            except Exception as ex:  # noqa: BLE001
                ans.append({"case": cid, "error": repr(ex)[:200]})
        out[c] = ans
    return out


# ----------------------------------------------------------------------------- reference areas
LHT_COUNTRIES = {"GB", "IE", "JP", "AU", "NZ", "IN", "ZA", "SG", "MY", "HK", "TH", "ID", "KE", "MT", "CY"}


def check_area(name, out_dir):
    """The same built-in checks on a gate.py reference area (its existing carla output)."""
    import gate
    a = gate.AREAS[name]
    args = a["args"]
    country = args[args.index("--country") + 1]
    origin = tuple(float(v) for v in args[args.index("--origin") + 1].split(","))
    meta = {"id": name, "kind": "area", "country": country, "lht": country in LHT_COUNTRIES, "origin": origin,
            "features": [], "expect": {}, "clip": args[args.index("--clip") + 1] if "--clip" in args else None}
    t0 = time.time()
    res, stats, _ = check_case(a["xodr"], a["osm"], meta, {"fit_max": None}, None, external=False)
    out = {"area": name, "xodr": a["xodr"], "seconds": round(time.time() - t0, 1), "stats": stats,
           "counts": {k: v["count"] for k, v in res.items()}, "checks": res}
    os.makedirs(out_dir, exist_ok=True)
    json.dump(out, open(os.path.join(out_dir, name + ".json"), "w", encoding="utf-8"), indent=1)
    return out


# ----------------------------------------------------------------------------- summary
def summarize(results):
    by_check = collections.Counter()
    cases_by_check = collections.defaultdict(list)
    by_kind = collections.defaultdict(lambda: {"cases": 0, "failed": 0, "checks": collections.Counter()})
    by_feat = collections.defaultdict(lambda: {"cases": 0, "failed": collections.Counter()})
    by_country = collections.defaultdict(lambda: {"cases": 0, "checks": collections.Counter()})
    for r in results:
        k = by_kind[r["kind"]]
        k["cases"] += 1
        k["failed"] += bool(r["failed"])
        by_country[r["country"]]["cases"] += 1
        for c in r["failed"]:
            by_check[c] += r["checks"][c]["count"]
            cases_by_check[c].append(r["id"])
            k["checks"][c] += 1
            by_country[r["country"]]["checks"][c] += 1
        for f in r["features"]:
            by_feat[f]["cases"] += 1
            for c in r["failed"]:
                by_feat[f]["failed"][c] += 1
    # feature combinations that fail far more often than the check's base rate
    n = max(1, len(results))
    hot = []
    for c, ids in cases_by_check.items():
        base = len(ids) / n
        for f, v in by_feat.items():
            if v["cases"] >= 3:
                rate = v["failed"][c] / v["cases"]
                if rate >= 0.5 and rate >= 1.5 * base:
                    hot.append({"check": c, "feature": f, "rate": round(rate, 2), "cases": v["cases"],
                                "base_rate": round(base, 2)})
    hot.sort(key=lambda h: (-h["rate"], h["check"]))
    return {
        "cases": len(results), "cases_failed": sum(bool(r["failed"]) for r in results),
        "instances_by_check": dict(by_check.most_common()),
        "cases_by_check": {c: {"n": len(v), "class": CHECKS.get(c, ("ext", ""))[0], "examples": v[:8]}
                           for c, v in sorted(cases_by_check.items(), key=lambda z: -len(z[1]))},
        "by_kind": {k: {"cases": v["cases"], "failed": v["failed"], "checks": dict(v["checks"].most_common())}
                    for k, v in sorted(by_kind.items())},
        "by_country": {k: {"cases": v["cases"], "checks": dict(v["checks"].most_common())}
                       for k, v in sorted(by_country.items())},
        "feature_hotspots": hot[:60],
        "check_definitions": {k: {"class": v[0], "definition": v[1]} for k, v in CHECKS.items()},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cases", type=int, default=200)
    ap.add_argument("--seed", default="1")
    ap.add_argument("--out", default=os.path.join(DEFAULT_OUT, "run"))
    ap.add_argument("--only", default="", help="comma list of kinds: " + ",".join(KINDS))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--exe", default=EXE)
    ap.add_argument("--no-xsd", action="store_true", help="skip the asam18 conversion + XSD validation")
    ap.add_argument("--esmini", action="store_true", help="also load every case in esmini odrplot")
    ap.add_argument("--no-external", action="store_true", help="do not run invariants.py / laneflow.py")
    ap.add_argument("--shrink", action="store_true", help="minimise one failing case per (kind, check)")
    ap.add_argument("--shrink-budget", type=int, default=40)
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--jev", type=int, default=0, help="ask Jev (secondary opinion) about N examples per failing check")
    ap.add_argument("--replay", default="", help="re-run the checks of one stored case directory")
    ap.add_argument("--selftest", default="", help="mutation test of the checks on a finished run directory")
    ap.add_argument("--areas", default="", help="also run the built-in checks on gate.py areas (comma list)")
    a = ap.parse_args()
    opt = {"exe": a.exe, "xsd": not a.no_xsd, "esmini": a.esmini, "external": not a.no_external, "keep": a.keep}
    if a.areas:
        for name in a.areas.split(","):
            o = check_area(name, os.path.join(a.out, "areas"))
            print(name, o["seconds"], "s", json.dumps(o["counts"]))
        return
    if a.selftest:
        st = selftest(a.selftest)
        json.dump(st, open(os.path.join(a.selftest, "selftest.json"), "w", encoding="utf-8"), indent=1)
        for x in st:
            print(f"{x['mutation']:<28} detected {x['detected']}/{x['cases']}  ({', '.join(x['expected_any_of'])})")
        return
    if a.replay:
        meta = json.load(open(os.path.join(a.replay, "meta.json"), encoding="utf-8"))
        r = run_case((a.replay, meta, opt))
        print(json.dumps({k: r[k] for k in ("id", "failed")}, indent=1))
        for c in r["failed"]:
            print(c, json.dumps(r["checks"][c]["instances"][:3]))
        return
    kinds = [k for k in (a.only.split(",") if a.only else KINDS) if k]
    for k in kinds:
        if k not in KINDS:
            sys.exit(f"unknown kind {k}; known: {', '.join(KINDS)}")
    os.makedirs(os.path.join(a.out, "cases"), exist_ok=True)
    jobs = []
    for i in range(a.cases):
        meta, net = make_case(a.seed, i, kinds)
        cdir = os.path.join(a.out, "cases", meta["id"])
        os.makedirs(cdir, exist_ok=True)
        net.write(os.path.join(cdir, "in.osm"), *meta["origin"])
        json.dump(meta, open(os.path.join(cdir, "meta.json"), "w", encoding="utf-8"), indent=1)
        jobs.append((cdir, meta, opt))
    t0 = time.time()
    results = []
    with ProcessPoolExecutor(max_workers=a.jobs) as pool:
        for r in pool.map(run_case, jobs, chunksize=2):
            results.append(r)
            print(f"{r['id']:<28} {r['country']} {r['seconds']:6.2f}s  {' '.join(r['failed']) or 'ok'}", flush=True)
    summ = summarize(results)
    summ["seed"], summ["seconds"], summ["exe"] = a.seed, round(time.time() - t0, 1), a.exe
    if a.shrink:
        mins = {}
        todo = {}
        for r in results:
            for c in r["failed"]:
                todo.setdefault((r["kind"], c), r)
        for (kind, c), r in sorted(todo.items()):
            cdir = os.path.join(a.out, "cases", r["id"])
            meta = json.load(open(os.path.join(cdir, "meta.json"), encoding="utf-8"))
            m = shrink(cdir, meta, c, opt, a.shrink_budget)
            if m:
                mins[f"{kind}:{c}"] = m
                print(f"shrunk {r['id']} {c}: {m['ways']} ways -> {m['path']}", flush=True)
        summ["minimal_reproducers"] = mins
    if a.jev:
        summ["jev_second_opinion"] = jev_opinions(a.out, summ, a.jev)
    json.dump(summ, open(os.path.join(a.out, "summary.json"), "w", encoding="utf-8"), indent=1)
    print(json.dumps({k: summ[k] for k in ("cases", "cases_failed", "instances_by_check")}, indent=1))
    print("summary:", os.path.join(a.out, "summary.json"))


if __name__ == "__main__":
    main()
