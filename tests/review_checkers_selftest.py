"""review_checkers_selftest -- planted-defect tests for the checker fixes the first LLM review asked for
(docs/LLM_REVIEW.md): each clean synthetic network must give no finding of the kind, and each planted defect
must be reported at its place.

    python tests/review_checkers_selftest.py [--keep DIR]

det_junction_gore   a connecting road without lanes (1.8.0 junction boundary helper) does not crash it;
                    connecting lanes are measured on their own centre (laneOffset 0 is not a gap), a real
                    0.5 m gap still is; every lane of a multi-lane connecting road counts (RHT and LHT)
det_link_continuity direct junctions (1.8.0 12.6) link lanes; an open lane at one is still found, with its
                    flow by road@rule
det_osm_fidelity    LHT lanes with id > 0 travel +s; onRamp lanes count; a carriageway built against
                    a oneway is still travel_direction_wrong
det_lane_transitions LHT: the way's forward lanes are on the left; a missing lane is still count_mismatch
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
# CHECKERS_DIR runs the tests against another copy of the checkers (e.g. an older commit, to see them fail)
INSPECT = os.environ.get("CHECKERS_DIR") or os.path.join(os.path.dirname(HERE), "tools", "inspect")
sys.path.insert(0, HERE)
import llm_review_selftest as S  # noqa: E402  (the synthetic network writers)

W = 3.5
Q = math.pi / 2


# ----------------------------------------------------------------------------- builders
def lane(i, t="driving", a=W, pred=None, succ=None):
    return (i, t, a, 0, pred, succ)


def road(rid, geoms, length, sections, pred=None, succ=None, osm=None, junction="-1", rule="RHT", offset=0.0):
    xml = S.road_xml(rid, junction, geoms, length, sections, pred=pred, succ=succ, osm=osm)
    xml = xml.replace('rule="RHT"', f'rule="{rule}"')
    if offset:
        xml = xml.replace('<laneOffset s="0" a="0"', f'<laneOffset s="0" a="{offset}"')
    return xml


def two_lane_junction(lht, drop=False):
    """road 1 (0,0)->(100,0), junction 100, road 2 (120,0)->(200,0); two driving lanes each way.
    drop: lane 2 of the eastbound connecting road loses its link (one incoming lane without a successor)"""
    rule = "LHT" if lht else "RHT"
    e1, e2 = (1, 2) if lht else (-1, -2)        # eastbound lanes of the plain roads (travel +s)
    w1, w2 = (-1, -2) if lht else (1, 2)        # westbound lanes
    c1, c2 = (1, 2) if lht else (-1, -2)        # connecting lanes on the travel side of their own +s
    plain = lambda pred, succ: [S.lanes_xml(0, [lane(i, pred=i if pred else None, succ=i if succ else None) for i in (1, 2)],
                                            [lane(-i, pred=-i if pred else None, succ=-i if succ else None) for i in (1, 2)])]
    R = {"1": road("1", [(0, 0, 0, 0, 100, 0)], 100, plain(False, False), succ=("junction", "100", None), osm="10", rule=rule),
         "2": road("2", [(0, 120, 0, 0, 80, 0)], 80, plain(False, False), pred=("junction", "100", None), osm="10", rule=rule)}

    def conn(cid, x, h, a_lanes, b_lanes, inc, out, ca, cb):
        ls = [lane(cl, pred=a, succ=b) for cl, a, b in zip((c1, c2), a_lanes, b_lanes)]
        if drop and cid == "101":
            ls[1] = lane(c2, pred=None, succ=b_lanes[1])
        sec = S.lanes_xml(0, ls if lht else [], [] if lht else ls)
        return road(cid, [(0, x, 0, h, 20, 0)], 20, [sec], pred=("road", inc, ca), succ=("road", out, cb), junction="100",
                    rule=rule)

    R["101"] = conn("101", 100, 0, (e1, e2), (e1, e2), "1", "2", "end", "start")
    R["102"] = conn("102", 120, math.pi, (w1, w2), (w1, w2), "2", "1", "start", "end")
    links = lambda pairs: "".join(f'<laneLink from="{a}" to="{b}"/>' for a, b in pairs)
    east = [(e1, c1)] + ([] if drop else [(e2, c2)])
    J = ('<junction id="100" name="" type="default">'
         f'<connection id="0" incomingRoad="1" connectingRoad="101" contactPoint="start">{links(east)}</connection>'
         f'<connection id="1" incomingRoad="2" connectingRoad="102" contactPoint="start">{links([(w1, c1), (w2, c2)])}</connection>'
         '</junction>')
    return R, J


def direct_junction(lht, drop=False):
    """road 1 (0,0)->(100,0) and road 2 (100,0)->(200,0) joined by a direct junction 200; one lane each way.
    drop: the westbound connection is missing"""
    rule = "LHT" if lht else "RHT"
    ls = [S.lanes_xml(0, [lane(1)], [lane(-1)])]
    R = {"1": road("1", [(0, 0, 0, 0, 100, 0)], 100, ls, succ=("junction", "200", None), osm="10", rule=rule),
         "2": road("2", [(0, 100, 0, 0, 100, 0)], 100, ls, pred=("junction", "200", None), osm="10", rule=rule)}
    e, w = (1, -1) if lht else (-1, 1)
    J = ('<junction id="200" name="" type="direct">'
         f'<connection id="0" incomingRoad="1" linkedRoad="2" contactPoint="start"><laneLink from="{e}" to="{e}"/></connection>'
         + ("" if drop else f'<connection id="1" incomingRoad="2" linkedRoad="1" contactPoint="end"><laneLink from="{w}" to="{w}"/></connection>')
         + '</junction>')
    return R, J


def oneway_road(lht, lanes):
    """road 1 (0,0)->(200,0) on OSM way 10 (primary, oneway=yes, lanes=2); lanes: [(id, type)], centred"""
    left = [lane(i, t) for i, t in lanes if i > 0]
    right = [lane(i, t) for i, t in lanes if i < 0]
    off = -W * len(left) / 2 + W * len(right) / 2
    return {"1": road("1", [(0, 0, 0, 0, 200, 0)], 200, [S.lanes_xml(0, left, right)], osm="10",
                      rule="LHT" if lht else "RHT", offset=off)}, ""


ONEWAY_OSM = {"10": ([(0, 0), (50, 0), (100, 0), (150, 0), (200, 0)],
                     {"highway": "primary", "oneway": "yes", "lanes": "2"})}
STREET_OSM = {"10": ([(0, 0), (100, 0), (110, 0), (120, 0), (200, 0)], {"highway": "residential"})}


# ----------------------------------------------------------------------------- running
def run(work, name, tool, roads, junction, osm):
    d = os.path.join(work, name)
    os.makedirs(d, exist_ok=True)
    x, o = os.path.join(d, "net.xodr"), os.path.join(d, "net.osm")
    S.write_xodr(x, roads, junction)
    S.write_osm(o, osm)
    out = os.path.join(d, tool)
    r = subprocess.run([sys.executable, os.path.join(INSPECT, f"det_{tool}.py"), x, o, "--out", out, "--crops", "0"],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    f = os.path.join(out, "findings.json")
    if not os.path.exists(f):
        return None, (r.stderr.strip().splitlines() or ["no output"])[-1]
    data = json.load(open(f, encoding="utf-8"))
    return (data["findings"] if isinstance(data, dict) else data), ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep")
    a = ap.parse_args()
    work = a.keep or tempfile.mkdtemp(prefix="review_checkers_")
    if a.keep and os.path.exists(work):
        shutil.rmtree(work)
    fails = []

    def check(ok, what):
        print(("ok    " if ok else "FAIL  ") + what)
        if not ok:
            fails.append(what)

    def kinds(F, kind, **match):
        return [f for f in (F or []) if f.get("kind") == kind and all(f.get(k) == v for k, v in match.items())]

    # --- det_junction_gore
    R, J = S.base_roads()
    F, err = run(work, "jg_base", "junction_gore", R, J, STREET_OSM)
    check(F is not None and not kinds(F, "conn_discontinuity"),
          f"junction_gore: connecting lanes with laneOffset 0 meet their lanes - no conn_discontinuity ({err or len(F)} findings)")
    R, J = S.base_roads()
    R["150"] = road("150", [(0, 100, 5, 0, 20, 0)], 20, [S.lanes_xml(0, [], [])], pred=("road", "1", "end"),
                    succ=("road", "2", "start"), junction="100")
    F, err = run(work, "jg_helper", "junction_gore", R, J, STREET_OSM)
    check(F is not None, f"junction_gore: a connecting road without lanes does not crash it ({err or 'ran'})")
    R, J = S.base_roads()
    S.edit(R, "101", lambda e: e.find("planView/geometry").set("y", "0.5"))
    F, err = run(work, "jg_gap", "junction_gore", R, J, STREET_OSM)
    hit = [f for f in kinds(F, "conn_discontinuity") if "101" in f["roads"]]
    check(bool(hit) and abs(max(hit[0]["gap_start"], hit[0]["gap_end"]) - 0.5) < 0.05,
          f"junction_gore: a planted 0.5 m gap at connecting road 101 is conn_discontinuity "
          f"({[round(max(h['gap_start'], h['gap_end']), 3) for h in hit]})")
    for lht in (False, True):
        tag = "LHT" if lht else "RHT"
        R, J = two_lane_junction(lht)
        F, err = run(work, f"jg_2lane_{tag}", "junction_gore", R, J, STREET_OSM)
        bad = kinds(F, "lane_no_successor") + kinds(F, "lane_no_predecessor") + kinds(F, "conn_discontinuity")
        check(F is not None and not bad, f"junction_gore {tag}: two-lane connecting roads, every lane linked - no "
                                         f"lane_no_* / conn_discontinuity ({err or [(f['kind'], f.get('lanes')) for f in bad]})")
        R, J = two_lane_junction(lht, drop=True)
        F, err = run(work, f"jg_2lane_{tag}_drop", "junction_gore", R, J, STREET_OSM)
        want = 2 if lht else -2
        hit = [f for f in kinds(F, "lane_no_successor") if f["roads"] == ["1"] and f.get("lanes") == [want]]
        check(bool(hit), f"junction_gore {tag}: incoming lane {want} of road 1 without a successor is lane_no_successor "
                         f"({err or [(f['kind'], f['roads'], f.get('lanes')) for f in F if f['kind'].startswith('lane_no')]})")

    # --- det_link_continuity
    for lht in (False, True):
        tag = "LHT" if lht else "RHT"
        R, J = direct_junction(lht)
        F, err = run(work, f"lc_direct_{tag}", "link_continuity", R, J, STREET_OSM)
        check(F is not None and not kinds(F, "open_lane_end_junction"),
              f"link_continuity {tag}: lanes linked through a direct junction are not open ends "
              f"({err or [(f['roads'], f.get('lane')) for f in kinds(F, 'open_lane_end_junction')]})")
        R, J = direct_junction(lht, drop=True)
        F, err = run(work, f"lc_direct_{tag}_drop", "link_continuity", R, J, STREET_OSM)
        lid = -1 if lht else 1  # the westbound lane of road 1
        hit = kinds(F, "open_lane_end_junction", lane=lid, end="end")
        check(bool(hit) and hit[0]["roads"] == ["1"] and hit[0]["flow"] == "outflow",
              f"link_continuity {tag}: westbound lane {lid} of road 1 without a connection is an open end, outflow "
              f"at its end ({err or [(f['roads'], f.get('lane'), f.get('end'), f.get('flow')) for f in kinds(F, 'open_lane_end_junction')]})")

    # --- det_osm_fidelity
    def lane_count(F):
        return [(f.get("subpattern"), f.get("actual_with_way"), f.get("actual_against_way")) for f in kinds(F, "lane_count")]

    for name, lht, lanes, expect in (
            ("RHT 2 lanes", False, [(-1, "driving"), (-2, "driving")], None),
            ("LHT 2 lanes", True, [(1, "driving"), (2, "driving")], None),
            ("RHT driving + onRamp", False, [(-1, "driving"), (-2, "onRamp")], None),
            ("LHT lanes against the oneway", True, [(-1, "driving"), (-2, "driving")], "travel_direction_wrong"),
            ("RHT 1 lane of 2", False, [(-1, "driving")], "other")):
        R, J = oneway_road(lht, lanes)
        F, err = run(work, "of_" + name.replace(" ", "_"), "osm_fidelity", R, J, ONEWAY_OSM)
        lc = lane_count(F)
        ok = F is not None and (not lc if expect is None else any(sub == expect or (expect == "other" and sub) for sub, _, _ in lc))
        check(ok, f"osm_fidelity {name}: " + ("no lane_count finding" if expect is None else f"lane_count {expect}")
              + f" ({err or lc})")

    # --- det_lane_transitions
    def mismatch(F):
        return [(f.get("side"), f.get("road_lanes"), f.get("osm_lanes")) for f in F or []
                if f.get("check") == "a_location" and f.get("sub") == "count_mismatch"]

    for name, lht, lanes, side in (("RHT 2 lanes", False, [(-1, "driving"), (-2, "driving")], None),
                                   ("LHT 2 lanes", True, [(1, "driving"), (2, "driving")], None),
                                   ("LHT 1 lane of 2", True, [(1, "driving")], "L")):
        R, J = oneway_road(lht, lanes)
        F, err = run(work, "lt_" + name.replace(" ", "_"), "lane_transitions", R, J, ONEWAY_OSM)
        mm = mismatch(F)
        ok = F is not None and (not mm if side is None else any(s == side and rl == 1 and ol == 2 for s, rl, ol in mm))
        check(ok, f"lane_transitions {name}: " + ("no count_mismatch" if side is None else
                                                  f"count_mismatch on side {side} (1 lane, OSM 2)") + f" ({err or mm})")

    print(f"\n{len(fails)} failures" + ("" if a.keep else f" (work dir {work} removed)"))
    if not a.keep:
        shutil.rmtree(work, ignore_errors=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
