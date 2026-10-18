"""boundary_audit_selftest -- plant known defects into a real OdrForge output and check that boundary_audit.py reports each
of them at the planted junction, and nothing at any other junction (the same idea as invariants_selftest.py).

    python boundary_audit_selftest.py BASE.xodr --out DIR

BASE is a standard-profile output with junction boundaries (e.g. the Munich test area). It must audit clean. One junction J
is chosen that has a grid, at least two joints and a helper road that is a corner curve (line / arc, no laneOffset). Each
plant edits a copy (DIR/<plant>.xodr) and PASSES when every expected check is reported for J and no other junction gets any
finding. Plants (expected checks):
  reversed_order          segments of J in reverse order                     counter_clockwise
  joint_removed           J's first joint deleted                            reaches_all_roads, closed
  segment_cut             a lane segment of J ends half way                  closed
  segment_emptied         a lane segment of J ends where it starts           empty_segment, closed
  helper_dent             the helper's line dented 3 m into the junction     contains_lanes
  helper_through          the helper's line pushed 80 m across the junction  simple
  helper_folded           the helper offset past its arc's centre            folded
  helper_wrong_junction   the helper's @junction set to another junction     helper_junction_id
  helper_link_far         the helper's predecessor set to a far road         helper_linkage
  helper_turned_out       the helper's first geometry turned 180 degrees     helper_points_inside
  ref_line_moved          J's reference line moved 500 m                     ref_line_crosses, grid_covers
  grid_shrunk             J's grid rows cut to one value each side           grid_covers
  helper_raised           the helper's elevation + 0.5 m (grid_audit.py)     |grid - helper surface| >= 0.49 m
  joint_lanes_narrowed    a joint across some lanes (jointLaneStart ..        closed
                          jointLaneEnd, Table 78) of a junction traced along its lanes' contour narrowed to one lane fewer
                          (that junction is the planted one for this plant)
Exit status 1 when any plant fails; DIR/selftest.json holds the findings per plant.
"""
import argparse
import copy
import json
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import boundary_audit  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def helper_of(root, j):
    """a corner-curve helper road of junction j: name JunctionBoundary, no laneOffset, an arc"""
    roads = {r.get("id"): r for r in root.findall("road")}
    for sg in j.findall("boundary/segment"):
        r = roads.get(sg.get("roadId"))
        if (sg.get("type") == "lane" and r is not None and r.get("name") == "JunctionBoundary" and r.find("lanes/laneOffset") is None
                and r.find("planView/geometry/arc") is not None):
            return sg, r
    return None, None


def choose(root):
    for j in root.findall("junction"):
        if j.find("elevationGrid") is None or len(j.findall("boundary/segment[@type='joint']")) < 2:
            continue
        sg, h = helper_of(root, j)
        if h is not None:
            return j
    raise SystemExit("no junction with a grid, two joints and a corner-curve helper in the base file")


def narrow_joint(root):
    """the first joint across some lanes spanning two or more (a boundary traced along its lanes' contour), narrowed by
    one lane at its end; its junction's id, or None"""
    for j in root.findall("junction"):
        for sg in j.findall("boundary/segment[@type='joint']"):
            a, b = sg.get("jointLaneStart"), sg.get("jointLaneEnd")
            if a is None or a == b:
                continue
            a, b = int(a), int(b)
            step = 1 if b > a else -1
            nb = b - step if b - step != 0 else b - 2 * step  # lane ids skip 0
            if (nb - a) * step < 0:
                continue
            sg.set("jointLaneEnd", str(nb))
            return j.get("id")
    return None


def plant(name, root, j, sg, h):
    roads = {r.get("id"): r for r in root.findall("road")}
    segs = j.find("boundary")
    L = float(h.get("length"))
    forward = sg.get("sStart") == "start"
    if name == "reversed_order":
        items = list(segs)
        for e in items:
            segs.remove(e)
        for e in reversed(items):
            segs.append(e)
    elif name == "joint_removed":
        segs.remove(segs.find("segment[@type='joint']"))
    elif name == "segment_cut":
        r = roads[sg.get("roadId")]
        half = "%.6f" % (0.5 * float(r.get("length")))
        if sg.get("sEnd") == "end":
            sg.set("sEnd", half)
        else:
            sg.set("sStart", half)
    elif name == "segment_emptied":
        sg.set("sEnd", sg.get("sStart"))
    elif name in ("helper_dent", "helper_through"):
        # t(s) = c s^2 + d s^3, zero at both ends, peak D at s = 2L/3; the inside is left of the outline's course
        D = 3.0 if name == "helper_dent" else 80.0
        sign = 1.0 if forward else -1.0
        c = sign * 27.0 * D / (4.0 * L * L)
        lanes = h.find("lanes")
        lo = ET.Element("laneOffset", {"s": "0", "a": "0", "b": "0", "c": repr(c), "d": repr(-c / L)})
        lanes.insert(0, lo)
    elif name == "helper_folded":
        k = float(h.find("planView/geometry/arc").get("curvature"))
        lanes = h.find("lanes")
        lanes.insert(0, ET.Element("laneOffset", {"s": "0", "a": repr(2.0 / k), "b": "0", "c": "0", "d": "0"}))
    elif name == "helper_wrong_junction":
        other = next(x.get("id") for x in root.findall("junction") if x.get("id") != j.get("id"))
        h.set("junction", other)
    elif name == "helper_link_far":
        p = h.find("link/predecessor")
        here = roads[p.get("elementId")]
        hx, hy = (float(here.find("planView/geometry").get(k)) for k in ("x", "y"))
        far = max((r for r in roads.values() if r.get("junction") == "-1"),
                  key=lambda r: math.hypot(float(r.find("planView/geometry").get("x")) - hx, float(r.find("planView/geometry").get("y")) - hy))
        p.set("elementId", far.get("id"))
    elif name == "helper_turned_out":
        # the helper's first geometry turned round: at its start it points out of the junction, back along the arm
        g = h.find("planView/geometry")
        g.set("hdg", repr(float(g.get("hdg")) + math.pi))
    elif name == "ref_line_moved":
        g = j.find("planView/geometry")
        g.set("x", repr(float(g.get("x")) + 500.0))
    elif name == "grid_shrunk":
        for row in j.findall("elevationGrid/elevation"):
            for side in ("left", "right"):
                v = (row.get(side) or "").split()
                if v:
                    row.set(side, v[0])
    elif name == "helper_raised":
        for e in h.findall("elevationProfile/elevation"):
            e.set("a", repr(float(e.get("a")) + 0.5))
    else:
        raise ValueError(name)


EXPECT = {
    "reversed_order": {"counter_clockwise"}, "joint_removed": {"reaches_all_roads", "closed"}, "segment_cut": {"closed"},
    "segment_emptied": {"empty_segment", "closed"},
    "helper_dent": {"contains_lanes"}, "helper_through": {"simple"}, "helper_folded": {"folded"},
    "helper_wrong_junction": {"helper_junction_id"}, "helper_link_far": {"helper_linkage"},
    "helper_turned_out": {"helper_points_inside"}, "ref_line_moved": {"ref_line_crosses", "grid_covers"},
    "grid_shrunk": {"grid_covers"},
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("base")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, "base.xodr")
    shutil.copyfile(a.base, base)
    clean = boundary_audit.audit(base, quiet=True)
    if clean["fails"]:
        raise SystemExit(f"base file does not audit clean: {dict(clean['fails'])}")
    tree0 = ET.parse(base)
    jid = choose(tree0.getroot()).get("id")
    print(f"base: {clean['boundaries']} boundaries, clean; planted junction {jid}")
    results, failed = {}, 0
    for name in list(EXPECT) + ["helper_raised"]:
        tree = copy.deepcopy(tree0)
        root = tree.getroot()
        j = next(x for x in root.findall("junction") if x.get("id") == jid)
        sg, h = helper_of(root, j)
        plant(name, root, j, sg, h)
        path = os.path.join(a.out, name + ".xodr")
        tree.write(path, encoding="utf-8", xml_declaration=True)
        if name == "helper_raised":
            out = subprocess.run([sys.executable, os.path.join(HERE, "grid_audit.py"), path], capture_output=True, text=True).stdout
            m = re.search(r"\|grid - boundary helper\| on the boundary: median [0-9.]+ p99 [0-9.]+ max ([0-9.]+) m", out)
            got = float(m.group(1)) if m else None
            ok = got is not None and got >= 0.49
            results[name] = {"pass": ok, "max_helper_dz": got}
        else:
            r = boundary_audit.audit(path, quiet=True)
            here = set(r["by_junction"].get(jid, {}))
            elsewhere = {k: dict(v) for k, v in r["by_junction"].items() if k != jid}
            ok = EXPECT[name] <= here and not elsewhere
            results[name] = {"pass": ok, "expected": sorted(EXPECT[name]), "at_junction": dict(r["by_junction"].get(jid, {})),
                             "elsewhere": elsewhere}
        failed += not ok
        print(f"{'PASS' if ok else 'FAIL'} {name}: {results[name]}")
    # a traced junction's joint across some lanes, narrowed: not closed at that junction, nothing elsewhere
    tree = copy.deepcopy(tree0)
    jt = narrow_joint(tree.getroot())
    if jt is None:
        results["joint_lanes_narrowed"] = {"pass": False, "reason": "no joint across two or more of some lanes in the base"}
    else:
        path = os.path.join(a.out, "joint_lanes_narrowed.xodr")
        tree.write(path, encoding="utf-8", xml_declaration=True)
        r = boundary_audit.audit(path, quiet=True)
        here = set(r["by_junction"].get(jt, {}))
        elsewhere = {k: dict(v) for k, v in r["by_junction"].items() if k != jt}
        results["joint_lanes_narrowed"] = {"pass": "closed" in here and not elsewhere, "junction": jt,
                                           "at_junction": dict(r["by_junction"].get(jt, {})), "elsewhere": elsewhere}
    failed += not results["joint_lanes_narrowed"]["pass"]
    print(f"{'PASS' if results['joint_lanes_narrowed']['pass'] else 'FAIL'} joint_lanes_narrowed: {results['joint_lanes_narrowed']}")
    with open(os.path.join(a.out, "selftest.json"), "w", encoding="utf-8") as f:
        json.dump({"base": a.base, "junction": jid, "plants": results}, f, indent=1)
    print(f"{len(results) - failed}/{len(results)} plants detected as expected")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
