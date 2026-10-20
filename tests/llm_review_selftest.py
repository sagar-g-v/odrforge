"""llm_review_selftest -- deterministic tests of tools/inspect/llm_review.py: prepare must surface every planted
defect as a candidate at the planted place (and the clean network must not have it there), merge must keep valid
verdicts and reject invalid ones, and check-prompt must notice new / changed / removed checkers, another OdrForge
version and an untargeted OpenDRIVE revision.

    python tests/llm_review_selftest.py [--keep DIR]

The network is synthetic (written here): a T junction of three roads with six connecting roads, a plain road-to-road
link, sidewalks, a crosswalk, and the OSM ways the roads come from. No converter, no network access, no LLM.
"""
import argparse
import copy
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
INSPECT = os.path.join(os.path.dirname(HERE), "tools", "inspect")
sys.path.insert(0, INSPECT)
import llm_review as L  # noqa: E402
import llm_review_prompt as P  # noqa: E402
import odrgeom as G  # noqa: E402

LAT0, LON0 = 48.0, 11.0
W_DRIVE, W_WALK = 3.5, 2.0
NEAR = 6.0  # a planted defect must be reported within this distance of where it was planted


# ----------------------------------------------------------------------------- the synthetic network
def lanes_xml(sec_s, left, right):
    """left / right: [(id, type, a, b, pred, succ)]"""
    def lane(i, t, a, b, p, s):
        link = ""
        if p is not None or s is not None:
            link = ("<link>" + (f'<predecessor id="{p}"/>' if p is not None else "")
                    + (f'<successor id="{s}"/>' if s is not None else "") + "</link>")
        return (f'<lane id="{i}" type="{t}" level="false">{link}'
                f'<width sOffset="0" a="{a}" b="{b}" c="0" d="0"/></lane>')
    return (f'<laneSection s="{sec_s}">'
            + ("<left>" + "".join(lane(*q) for q in left) + "</left>" if left else "")
            + '<center><lane id="0" type="none" level="false"/></center>'
            + ("<right>" + "".join(lane(*q) for q in right) + "</right>" if right else "")
            + "</laneSection>")


def road_xml(rid, junction, geoms, length, sections, pred=None, succ=None, osm=None, objects=""):
    link = ""
    for tag, v in (("predecessor", pred), ("successor", succ)):
        if v:
            et, eid, cp = v
            link += f'<{tag} elementType="{et}" elementId="{eid}"' + (f' contactPoint="{cp}"' if cp else "") + "/>"
    g = "".join(f'<geometry s="{s}" x="{x}" y="{y}" hdg="{h}" length="{L_}">'
                + ('<line/>' if k == 0 else f'<arc curvature="{k}"/>') + "</geometry>"
                for s, x, y, h, L_, k in geoms)
    ud = f'<userData code="osmWays" value="{osm}"/>' if osm else ""
    return (f'<road name="" length="{length}" id="{rid}" junction="{junction}" rule="RHT">'
            f'<link>{link}</link><type s="0" type="town"><speed max="50" unit="km/h"/></type>'
            f'<planView>{g}</planView><lanes><laneOffset s="0" a="0" b="0" c="0" d="0"/>{"".join(sections)}</lanes>'
            f'<objects>{objects}</objects>{ud}</road>')


def plain_lanes(pred, succ, walk=True):
    """two-way street: 1 / -1 driving, 2 / -2 walking; pred / succ: lane link ids are the same lane"""
    p = (lambda i: i) if pred else (lambda i: None)
    s = (lambda i: i) if succ else (lambda i: None)
    left = [(1, "driving", W_DRIVE, 0, p(1), s(1))] + ([(2, "walking", W_WALK, 0, p(2), s(2))] if walk else [])
    right = [(-1, "driving", W_DRIVE, 0, p(-1), s(-1))] + ([(-2, "walking", W_WALK, 0, p(-2), s(-2))] if walk else [])
    return left, right


def crosswalk(oid, s, width):
    hw = width / 2
    return (f'<object id="{oid}" name="Crosswalk" type="crosswalk" s="{s}" t="0" zOffset="0" hdg="0" pitch="0" '
            f'roll="0" orientation="none" length="3" width="{width}" height="0"><outlines><outline id="0" closed="true">'
            f'<cornerLocal id="0" u="-1.5" v="{-hw}" z="0" height="0"/><cornerLocal id="1" u="1.5" v="{-hw}" z="0" height="0"/>'
            f'<cornerLocal id="2" u="1.5" v="{hw}" z="0" height="0"/><cornerLocal id="3" u="-1.5" v="{hw}" z="0" height="0"/>'
            f'</outline></outlines></object>')


Q = math.pi / 2
ARC = Q * 10  # quarter circle of radius 10


def base_roads():
    """{road id: xml}; the junction separately"""
    R = {}
    l, r = plain_lanes(False, False)
    # road 1: (0,0) -> (100,0) east, starts at the boundary, ends at junction 100
    R["1"] = road_xml("1", "-1", [(0, 0, 0, 0, 100, 0)], 100, [lanes_xml(0, l, r)], succ=("junction", "100", None),
                      osm="10", objects=crosswalk(1, 50, 2 * W_DRIVE))
    # road 2: (120,0) -> (200,0) east, from the junction to the boundary
    R["2"] = road_xml("2", "-1", [(0, 120, 0, 0, 80, 0)], 80, [lanes_xml(0, l, r)], pred=("junction", "100", None),
                      osm="10")
    # road 4: (110,-10) -> (110,-50) south, then road 7: (110,-50) -> (110,-100) to the boundary
    l4, r4 = plain_lanes(False, True)
    R["4"] = road_xml("4", "-1", [(0, 110, -10, -Q, 40, 0)], 40, [lanes_xml(0, l4, r4)], pred=("junction", "100", None),
                      succ=("road", "7", "start"), osm="12")
    l7, r7 = plain_lanes(True, False)
    R["7"] = road_xml("7", "-1", [(0, 110, -50, -Q, 50, 0)], 50, [lanes_xml(0, l7, r7)], pred=("road", "4", "end"),
                      osm="12")
    # connecting roads: one driving lane -1 each (incoming lane -> outgoing lane)
    conns = [  # id, geometry (x, y, hdg, length, curvature), (incoming road, contact), (outgoing road, contact), lanes
        ("101", (100, 0, 0, 20, 0), ("1", "end"), ("2", "start"), (-1, -1)),        # straight east
        ("102", (120, 0, math.pi, 20, 0), ("2", "start"), ("1", "end"), (1, 1)),     # straight west
        ("103", (100, 0, 0, ARC, -0.1), ("1", "end"), ("4", "start"), (-1, -1)),     # 1 -> 4 right turn
        ("104", (110, -10, Q, ARC, -0.1), ("4", "start"), ("2", "start"), (1, -1)),  # 4 -> 2 right turn
        ("105", (120, 0, math.pi, ARC, 0.1), ("2", "start"), ("4", "start"), (1, -1)),  # 2 -> 4 left turn
        ("106", (110, -10, Q, ARC, 0.1), ("4", "start"), ("1", "end"), (1, 1)),     # 4 -> 1 left turn
    ]
    J = []
    for i, (cid, (x, y, h, L_, k), (a, ca), (b, cb), (la, lb)) in enumerate(conns):
        R[cid] = road_xml(cid, "100", [(0, x, y, h, L_, k)], L_,
                          [lanes_xml(0, [], [(-1, "driving", W_DRIVE, 0, la, lb)])],
                          pred=("road", a, ca), succ=("road", b, cb))
        J.append(f'<connection id="{i}" incomingRoad="{a}" connectingRoad="{cid}" contactPoint="start">'
                 f'<laneLink from="{la}" to="-1"/></connection>')
    return R, '<junction id="100" name="" type="default">' + "".join(J) + "</junction>"


def write_xodr(path, roads, junction, rev_minor=8):
    xml = ('<?xml version="1.0" encoding="UTF-8"?>\n<OpenDRIVE><header revMajor="1" revMinor="%d" name="selftest" '
           'version="1" north="150" south="-100" east="200" west="0" vendor="OdrForge %s">'
           '<geoReference><![CDATA[+proj=tmerc +lat_0=%s +lon_0=%s +k=1 +x_0=0 +y_0=0 +ellps=WGS84]]></geoReference>'
           '</header>%s%s</OpenDRIVE>\n') % (rev_minor, P.ODRFORGE_VERSION, LAT0, LON0,
                                            "".join(roads[k] for k in sorted(roads, key=int)), junction)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(xml)


def latlon(x, y):
    """inverse of odrgeom.tm_forward by Newton steps (mm-exact over a few hundred metres)"""
    lat, lon = LAT0 + y / 111200.0, LON0 + x / (111200.0 * math.cos(math.radians(LAT0)))
    for _ in range(6):
        fx, fy = G.tm_forward(lat, lon, LAT0, LON0)
        e = 1e-7
        ax, ay = G.tm_forward(lat + e, lon, LAT0, LON0)
        bx, by = G.tm_forward(lat, lon + e, LAT0, LON0)
        j = [[(ax - fx) / e, (bx - fx) / e], [(ay - fy) / e, (by - fy) / e]]
        det = j[0][0] * j[1][1] - j[0][1] * j[1][0]
        dx, dy = x - fx, y - fy
        lat += (j[1][1] * dx - j[0][1] * dy) / det
        lon += (-j[1][0] * dx + j[0][0] * dy) / det
    return float(lat), float(lon)


def write_osm(path, ways):
    """ways: {way id: ([(x, y)], tags)}; nodes shared where coordinates match"""
    nodes, ids, out = {}, {}, []
    for wid, (pts, tags) in ways.items():
        for p in pts:
            if p not in ids:
                ids[p] = str(1000 + len(ids))
                nodes[ids[p]] = latlon(*p)
    out.append('<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6">')
    out += [f'<node id="{i}" lat="{la:.9f}" lon="{lo:.9f}"/>' for i, (la, lo) in nodes.items()]
    for wid, (pts, tags) in ways.items():
        out.append(f'<way id="{wid}">' + "".join(f'<nd ref="{ids[p]}"/>' for p in pts)
                   + "".join(f'<tag k="{k}" v="{v}"/>' for k, v in tags.items()) + "</way>")
    out.append("</osm>\n")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out))


BASE_OSM = {"10": ([(0, 0), (100, 0), (110, 0), (120, 0), (200, 0)], {"highway": "residential", "sidewalk": "both"}),
            "12": ([(110, 0), (110, -10), (110, -50), (110, -100)], {"highway": "residential", "sidewalk": "both"})}
WAY_11 = {"11": ([(20, 100), (40, 100), (80, 100), (120, 100)], {"highway": "residential"})}


# ----------------------------------------------------------------------------- plants
def edit(roads, rid, fn):
    el = ET.fromstring(roads[rid])
    fn(el)
    roads[rid] = ET.tostring(el, encoding="unicode")


def lane_el(road_el, lid, sec=0):
    return [l for l in road_el.find("lanes").findall("laneSection")[sec].iter("lane") if l.get("id") == str(lid)][0]


def p_cut_lane_link(R, J):
    # road 1 lane -1 loses every continuation into the junction
    for cid in ("101", "103"):
        edit(R, cid, lambda e: lane_el(e, -1).find("link").remove(lane_el(e, -1).find("link/predecessor")))
    J = J.replace('connectingRoad="101" contactPoint="start"><laneLink from="-1" to="-1"/>',
                  'connectingRoad="101" contactPoint="start">')
    J = J.replace('connectingRoad="103" contactPoint="start"><laneLink from="-1" to="-1"/>',
                  'connectingRoad="103" contactPoint="start">')
    return R, J, None


def p_sidewalk_ends(R, J):
    # road 2 gets a second section from s=40 without walking lanes
    l, r = plain_lanes(False, False)
    l2, r2 = plain_lanes(False, False, walk=False)
    l = [(1, "driving", W_DRIVE, 0, None, 1), (2, "walking", W_WALK, 0, None, None)]
    r = [(-1, "driving", W_DRIVE, 0, None, -1), (-2, "walking", W_WALK, 0, None, None)]
    l2 = [(1, "driving", W_DRIVE, 0, 1, None)]
    r2 = [(-1, "driving", W_DRIVE, 0, -1, None)]
    R["2"] = road_xml("2", "-1", [(0, 120, 0, 0, 80, 0)], 80, [lanes_xml(0, l, r), lanes_xml(40, l2, r2)],
                      pred=("junction", "100", None), osm="10")
    return R, J, None


def p_interior_dead_end(R, J):
    l, r = plain_lanes(False, False)
    R["3"] = road_xml("3", "-1", [(0, 40, 100, 0, 40, 0)], 40, [lanes_xml(0, l, r)], osm="11")
    return R, J, WAY_11


TRACKS = {"13": ([(40, 100), (80, 100)], {"highway": "residential"}),
          "14": ([(20, 100), (40, 100)], {"highway": "track", "tracktype": "grade3"}),
          "15": ([(80, 100), (120, 100)], {"highway": "track"})}


def p_track_dead_end(R, J):
    l, r = plain_lanes(False, False)
    R["3"] = road_xml("3", "-1", [(0, 40, 100, 0, 40, 0)], 40, [lanes_xml(0, l, r)], osm="13")
    return R, J, TRACKS


def p_duplicate_crosswalk(R, J):
    edit(R, "1", lambda e: e.find("objects").append(ET.fromstring(crosswalk(2, 50.3, 2 * W_DRIVE))))
    return R, J, None


def p_osm_way_missing(R, J):
    return R, J, WAY_11


def p_crosswalk_unreached(R, J):
    edit(R, "1", lambda e: e.find("objects").find("object").set("width", "3"))

    def corners(e):
        for c in e.iter("cornerLocal"):
            c.set("v", str(math.copysign(1.5, float(c.get("v")))))
    edit(R, "1", corners)
    return R, J, None


def p_zero_width_lane(R, J):
    edit(R, "2", lambda e: e.find("lanes/laneSection/right").append(ET.fromstring(
        '<lane id="-3" type="driving" level="false"><width sOffset="0" a="0" b="0" c="0" d="0"/></lane>')))
    return R, J, None


def p_joint_gap(R, J):
    edit(R, "7", lambda e: e.find("planView/geometry").set("x", "110.5"))
    return R, J, None


def p_link_asymmetric(R, J):
    def f(e):
        lane_el(e, 1).find("link/predecessor").set("id", "2")
        lane_el(e, 2).find("link/predecessor").set("id", "1")
    edit(R, "7", f)
    return R, J, None


def p_wrong_way_connection(R, J):
    J = J.replace('connectingRoad="101" contactPoint="start"><laneLink from="-1"',
                  'connectingRoad="101" contactPoint="start"><laneLink from="1"')
    edit(R, "101", lambda e: lane_el(e, -1).find("link/predecessor").set("id", "1"))
    return R, J, None


def p_wrong_way_road(R, J):
    # road 4 lane -1 (southbound) continues into road 7 lane 1 (northbound)
    edit(R, "4", lambda e: lane_el(e, -1).find("link/successor").set("id", "1"))
    edit(R, "7", lambda e: lane_el(e, 1).find("link/predecessor").set("id", "-1"))
    return R, J, None


def p_missing_turn(R, J):
    del R["106"]
    J = J.replace('<connection id="5" incomingRoad="4" connectingRoad="106" contactPoint="start">'
                  '<laneLink from="1" to="-1"/></connection>', "")
    return R, J, None


def p_tight_turn(R, J):
    R["103"] = road_xml("103", "100", [(0, 100, 0, 0, Q * 2.5, -0.4)], Q * 2.5,
                        [lanes_xml(0, [], [(-1, "driving", W_DRIVE, 0, -1, -1)])],
                        pred=("road", "1", "end"), succ=("road", "4", "start"))
    return R, J, None


def p_negative_width(R, J):
    edit(R, "7", lambda e: lane_el(e, 2).find("width").set("a", "-0.5"))
    return R, J, None


def p_sliver(R, J):
    edit(R, "7", lambda e: lane_el(e, -1).find("width").set("a", "1.0"))
    return R, J, None


def p_width_implausible(R, J):
    edit(R, "7", lambda e: lane_el(e, -1).find("width").set("a", "6.5"))
    return R, J, None


def p_lane_ends_mid_road(R, J):
    l = [(1, "driving", W_DRIVE, 0, None, 1), (2, "walking", W_WALK, 0, None, 2)]
    r = [(-1, "driving", W_DRIVE, 0, None, None), (-2, "walking", W_WALK, 0, None, -1)]
    l2 = [(1, "driving", W_DRIVE, 0, 1, None), (2, "walking", W_WALK, 0, 2, None)]
    r2 = [(-1, "walking", W_WALK, 0, -2, None)]
    R["2"] = road_xml("2", "-1", [(0, 120, 0, 0, 80, 0)], 80, [lanes_xml(0, l, r), lanes_xml(40, l2, r2)],
                      pred=("junction", "100", None), osm="10")
    return R, J, None


def p_lane_drop_without_taper(R, J):
    l = [(1, "driving", W_DRIVE, 0, None, 1), (2, "walking", W_WALK, 0, None, 2)]
    r = [(-1, "driving", W_DRIVE, 0, None, -1), (-2, "walking", W_WALK, 0, None, -2)]
    rt = [(-1, "driving", W_DRIVE, -W_DRIVE / 2, -1, None), (-2, "walking", W_WALK, 0, -2, -1)]
    l3 = [(1, "driving", W_DRIVE, 0, 1, None), (2, "walking", W_WALK, 0, 2, None)]
    r3 = [(-1, "walking", W_WALK, 0, -2, None)]
    R["2"] = road_xml("2", "-1", [(0, 120, 0, 0, 80, 0)], 80,
                      [lanes_xml(0, l, r), lanes_xml(38, [(1, "driving", W_DRIVE, 0, 1, 1), (2, "walking", W_WALK, 0, 2, 2)], rt),
                       lanes_xml(40, l3, r3)], pred=("junction", "100", None), osm="10")
    return R, J, None


def p_closed_loop(R, J):
    # a one-way ring of two half circles (radius 20 around (60, 60)) linked only to itself
    half = math.pi * 20
    lanes = [lanes_xml(0, [], [(-1, "driving", W_DRIVE, 0, -1, -1)])]
    R["5"] = road_xml("5", "-1", [(0, 40, 60, -Q, half, 0.05)], half, lanes, pred=("road", "6", "end"),
                      succ=("road", "6", "start"))
    R["6"] = road_xml("6", "-1", [(0, 80, 60, Q, half, 0.05)], half, lanes, pred=("road", "5", "end"),
                      succ=("road", "5", "start"))
    return R, J, None


# name, plant, candidate kind that must appear, (x, y) where
PLANTS = [
    ("cut_lane_link", p_cut_lane_link, "lane_leads_nowhere", (100, -1.75)),
    ("sidewalk_ends", p_sidewalk_ends, "sidewalk_ends", (160, -4.5)),
    ("interior_dead_end", p_interior_dead_end, "dead_end_osm_continues", (40, 100)),
    ("osm_way_missing", p_osm_way_missing, "osm_way_missing", (80, 100)),
    ("track_dead_end", p_track_dead_end, "dead_end_track_continues", (40, 100)),
    ("duplicate_crosswalk", p_duplicate_crosswalk, "duplicate_crosswalk", (50, 0)),
    ("crosswalk_unreached", p_crosswalk_unreached, "crosswalk_unreached", (50, 0)),
    ("zero_width_lane", p_zero_width_lane, "zero_width_lane", (160, 0)),
    ("joint_gap", p_joint_gap, "joint_gap_or_kink", (110, -50)),
    ("link_asymmetric", p_link_asymmetric, "link_asymmetric", (110, -50)),
    ("wrong_way_connection", p_wrong_way_connection, "wrong_way_link", (100, 1.75)),
    ("wrong_way_road", p_wrong_way_road, "wrong_way_link", (110, -50)),
    ("missing_turn", p_missing_turn, "missing_turn", (110, -10)),
    ("tight_turn", p_tight_turn, "tight_turn", (101, -1)),
    ("negative_width", p_negative_width, "negative_width", (110, -75)),
    ("sliver", p_sliver, "sliver_lane", (110, -75)),
    ("width_implausible", p_width_implausible, "width_implausible", (110, -75)),
    ("lane_ends_mid_road", p_lane_ends_mid_road, "lane_ends_mid_road", (160, -1.75)),
    ("lane_drop_without_taper", p_lane_drop_without_taper, "lane_drop_without_taper", (160, 0)),
    ("closed_loop", p_closed_loop, "closed_loop_trap", (60, 60), 21.0),  # reported on the ring (radius 20)
]


# ----------------------------------------------------------------------------- running
def run(*args):
    r = subprocess.run([sys.executable, os.path.join(INSPECT, "llm_review.py")] + list(args), capture_output=True,
                       text=True, encoding="utf-8", errors="replace")
    return r.returncode, r.stdout + r.stderr


def prepare(work, name, roads, junction, osm_extra, images=False):
    d = os.path.join(work, "in")
    os.makedirs(d, exist_ok=True)
    x, o = os.path.join(d, name + ".xodr"), os.path.join(d, name + ".osm")
    write_xodr(x, roads, junction)
    write_osm(o, dict(BASE_OSM, **(osm_extra or {})))
    rc, out = run("prepare", x, "--out", os.path.join(work, "bundles"), "--osm", o, "--basemap", "none",
                  "--force", *([] if images else ["--no-images"]))
    if rc != 0:
        raise AssertionError(f"prepare {name} failed:\n{out}")
    return os.path.join(work, "bundles", name)


def near(cands, kind, xy, radius=NEAR):
    return [c for c in cands if c["kind"] == kind and math.hypot(c["x"] - xy[0], c["y"] - xy[1]) <= radius]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep", help="keep the work directory here")
    a = ap.parse_args()
    work = a.keep or tempfile.mkdtemp(prefix="llm_review_selftest_")
    if a.keep and os.path.exists(work):
        shutil.rmtree(work)
    fails = []

    def check(ok, what):
        print(("ok    " if ok else "FAIL  ") + what)
        if not ok:
            fails.append(what)

    # 1. the clean network
    R, J = base_roads()
    try:
        import matplotlib  # noqa: F401
        images = True
    except ImportError:
        images = False
    b = prepare(work, "base", R, J, None, images=images)
    base = L.load(os.path.join(b, "candidates.json"))
    facts = L.load(os.path.join(b, "facts.json"))
    check(facts["drive_graph"]["stuck_pieces"] == 0 and facts["drive_graph"]["unreachable_pieces"] == 0,
          f"base: every driving lane can go on and be reached ({facts['drive_graph']})")
    check(all(d["status"] == "boundary" for d in facts["dead_ends"]) and len(facts["dead_ends"]) == 3,
          f"base: the three dead ends are the clip boundary ({[(d['road'], d['end'], d['status']) for d in facts['dead_ends']]})")
    cw = facts["crosswalks"]
    check(len(cw) == 1 and cw[0]["ends_on_walk"] == 2, "base: the crosswalk reaches a walking lane on both sides")
    check(not facts.get("joints"), "base: no gap or kink at any lane joint")
    if images:
        pngs = glob.glob(os.path.join(b, "img", "*.png"))
        check(len(pngs) > 0 and all(os.path.getsize(p) > 10000 for p in pngs), f"base: {len(pngs)} plan views rendered")
    else:
        print("skip  images (no matplotlib in this Python)")
    m = L.load(os.path.join(b, "manifest.json"))
    check(m["header"]["revMinor"] == "8" and m["prompt_version"] == P.PROMPT_VERSION and m["chunks"],
          "base: manifest names the revision, the prompt version and its chunks")
    md = open(os.path.join(b, m["chunks"][0] + ".md"), encoding="utf-8").read()
    check("## File facts" in md and "### Roads in view" in md and "Write your verdicts" in md,
          "base: chunk text has file facts, roads in view and the instruction")

    # 2. every planted defect is surfaced where it was planted, and is not there in the clean network
    for name, plant, kind, xy, *rad in PLANTS:
        R, J = base_roads()
        R2, J2, osm = plant(copy.deepcopy(R), J)
        pb = prepare(work, name, R2, J2, osm)
        c = L.load(os.path.join(pb, "candidates.json"))
        shutil.rmtree(pb)
        hit = near(c, kind, xy, *rad)
        check(bool(hit) and not near(base, kind, xy, *rad),
              f"plant {name}: {kind} at {xy} " + (f"found ({hit[0]['severity']})" if hit else
                                                  f"MISSING; got {sorted({x['kind'] for x in c})}"))

    # 3. merge keeps valid verdicts, rejects invalid ones, writes the viewer schema
    chunk = L.load(os.path.join(b, m["chunks"][0] + ".json"))
    sp = chunk["spots"][0]
    good = dict(spot=sp["id"], candidates=sp["candidates"][:1], verdict="defect", category="lanes",
                kind="sidewalk_ends", perspective=["pedestrian"], severity="minor", x=sp["x"], y=sp["y"], road="1",
                lane=-2, osm_ways=["10"], evidence=["walking lane -2 of road 1 ends 2.00 m wide at the junction"],
                reasoning="pedestrians walk into the junction with no sidewalk", spec="", confidence=0.8,
                note="sidewalk ends at the junction")
    bad_kind = dict(good, kind="bogus")
    far = dict(good, x=sp["x"] + 5000)
    bad_road = dict(good, road="99999")
    vdir = os.path.join(b, "verdicts")
    L.dump(dict(chunk=m["chunks"][0], prompt_version=P.PROMPT_VERSION, reviewer="selftest",
                verdicts=[good, bad_kind, far, bad_road, dict(good, verdict="legitimate", evidence=[])]),
           os.path.join(vdir, m["chunks"][0] + ".json"))
    dest = os.path.join(work, "reviews")
    rc, out = run("merge", os.path.join(work, "bundles"), "--dest", dest)
    rv = L.load(os.path.join(dest, "base.review.json"))
    check(rc == 1 and len(rv["flags"]) == 1 and len(rv["legitimate"]) == 1 and len(rv["rejected"]) == 3,
          f"merge: 1 flag, 1 legitimate, 3 rejected, exit 1 (got rc {rc}, {len(rv['flags'])}, {len(rv['legitimate'])}, "
          f"{len(rv['rejected'])})")
    f = rv["flags"][0]
    check(rv["source"] == "llm-review" and f["source"] == "llm-review" and f["status"] == "open" and f["id"] == 1
          and f["osm_ways"] == ["10"] and f["road"] == "1" and f["category"] in P.CATEGORIES and 0 <= f["confidence"] <= 1
          and all(k in f for k in ("x", "y", "lane", "junction", "s", "note", "created", "evidence", "reasoning")),
          "merge: flag carries the viewer fields plus source, osm_ways, evidence, reasoning, confidence")
    check(rv["sha256"] == m["sha256"] and rv["matches_current_output"], "merge: records the reviewed file's sha256")
    L.dump(dict(chunk=m["chunks"][0], prompt_version=P.PROMPT_VERSION, reviewer="selftest", verdicts=[good]),
           os.path.join(vdir, m["chunks"][0] + ".json"))
    rc, out = run("merge", os.path.join(work, "bundles"), "--dest", dest)
    check(rc == 0, "merge: all verdicts valid -> exit 0")
    L.dump(dict(chunk=m["chunks"][0], prompt_version=P.PROMPT_VERSION, reviewer="selftest",
                verdicts=[dict(good, candidates=[1])]), os.path.join(vdir, m["chunks"][0] + ".json"))
    rc, out = run("merge", os.path.join(work, "bundles"), "--dest", dest)
    rv = L.load(os.path.join(dest, "base.review.json"))
    check(rc == 0 and rv["flags"][0]["candidates"] == sp["candidates"][:1],
          "merge: a candidate given by its list number resolves to that spot's candidate id")
    check(L.chunk_errors(b, m["chunks"][0]) == [], "chunk_errors: the claude-cli runner's per-chunk check agrees")
    fpath = os.path.join(dest, "base.flags.json")
    L.dump({"flags": [{"id": 3, "x": 1.0, "y": 2.0, "category": "geometry", "note": "mine", "status": "fixed"}]}, fpath)
    rc, out = run("merge", os.path.join(work, "bundles"), "--dest", dest, "--as-flags")
    rc2, out2 = run("merge", os.path.join(work, "bundles"), "--dest", dest, "--as-flags")
    fl = L.load(fpath)["flags"]
    mine = [f for f in fl if f["id"] == 3]
    new = [f for f in fl if f.get("source") == "llm-review"]
    check(rc == 0 and len(fl) == 2 and mine and mine[0]["note"] == "mine" and mine[0]["status"] == "fixed"
          and len(new) == 1 and new[0]["id"] == L.FLAG_ID0 and new[0]["status"] == "open"
          and new[0]["note"].startswith("[LLM review defect") and "10" in new[0]["note"],
          "merge --as-flags: review verdicts appended to <stem>.flags.json as open flags from id 1001, the user's "
          "flag untouched, a second run adds nothing")
    L.dump(dict(chunk=m["chunks"][0], prompt_version="0.0.0", reviewer="selftest", verdicts=[good]),
           os.path.join(vdir, m["chunks"][0] + ".json"))
    rc, out = run("merge", os.path.join(work, "bundles"), "--dest", dest)
    rv = L.load(os.path.join(dest, "base.review.json"))
    check(rc == 1 and not rv["flags"], "merge: verdicts made with another prompt version are rejected")
    rc, out = run("prepare", os.path.join(work, "in", "base.xodr"), "--out", os.path.join(work, "bundles"),
                  "--osm", os.path.join(work, "in", "base.osm"), "--basemap", "none", "--no-images", "--reuse")
    m2 = L.load(os.path.join(b, "manifest.json"))
    check(rc == 0 and m2["sha256"] == m["sha256"] and not os.listdir(vdir)
          and glob.glob(os.path.join(b, "verdicts.prev.*")),
          "prepare --reuse: same reviewed file, earlier verdicts moved aside (never merged into new spots)")

    # 4. check-prompt notices what makes the prompt stale
    tmp = os.path.join(work, "inspect")
    shutil.copytree(INSPECT, tmp, ignore=shutil.ignore_patterns("__pycache__"))
    r0 = set(L.stale_reasons(tmp, version=P.ODRFORGE_VERSION))
    with open(os.path.join(tmp, "det_planted.py"), "w") as fo:
        fo.write('"""a new checker"""\n')
    with open(os.path.join(tmp, "invariants.py"), "a") as fo:
        fo.write("\n# changed\n")
    os.remove(os.path.join(tmp, "annex_f.py"))
    x9 = os.path.join(work, "in", "rev9.xodr")
    R, J = base_roads()
    write_xodr(x9, R, J, rev_minor=9)
    r1 = set(L.stale_reasons(tmp, version="9.9.9", xodrs=[x9])) - r0
    for what in ("new checker det_planted.py", "checker invariants.py changed", "checker annex_f.py is gone",
                 "OdrForge is 9.9.9", "is OpenDRIVE 1.9"):
        check(any(what in r for r in r1), f"check-prompt: '{what}'")
    rc, out = run("check-prompt")
    print("      current repository: " + out.strip().splitlines()[0])
    text = P.text()
    check(all(p in text for p in P.PERSPECTIVES) and P.PROMPT_VERSION in text and "Not a defect" in text,
          "prompt: names every perspective, its version and the legitimate cases")

    print(f"\n{len(fails)} failures" + ("" if a.keep else f" (work dir {work} removed)"))
    if not a.keep:
        shutil.rmtree(work, ignore_errors=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
