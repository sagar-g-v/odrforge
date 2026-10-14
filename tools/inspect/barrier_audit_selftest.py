"""barrier_audit_selftest -- plant known defects into a real OdrForge output and check that barrier_audit.py reports
each of them for the planted barrier object and nothing for any other object.

    python barrier_audit_selftest.py BASE.xodr --osm BASE.osm --out DIR

BASE is a standard-profile output with barriers (e.g. the Frankfurt Kreuz test area) and the OSM file it was converted
from; it must audit clean. Two objects are planted in turn: the first guardRail and the first jerseyBarrier with three
or more repeats. Plants, one per copy (expected check):
  subtype_unknown   subtype not in Table 108 ("guardrail")                               kind
  oriented          orientation "+"                                                     kind
  discrete          the first repeat's distance 5 (Table 108: distance > 0 not allowed)  repeat
  no_size           the second repeat's widthStart 0                                    repeat
  past_road_end     the last repeat runs 10 m past the road's end                       on_road
  gap               the second repeat starts 1 m later                                  continuous
  object_box        length 10 on the object                                             no_object_box
  on_lane           every repeat moved to the centre of a lane in use                   clear_of_lanes
  far_off           every repeat moved 8 m further out                                  alongside
  sunk              every zOffset lowered until the bottom is 0.3 m in the ground        ground
  floating          every zOffset 1 m higher                                            ground
  sideways          every repeat moved 0.5 m further out (still beside the road)        follows_osm
  wrong_kind        subtype swapped between guardRail and jerseyBarrier (the OSM way
                    there is the other kind)                                            follows_osm
Exit status 1 when any plant fails; DIR/selftest.json holds the findings per plant.
"""
import argparse
import copy
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import barrier_audit  # noqa: E402
import odrgeom as G  # noqa: E402

EXPECT = {
    "subtype_unknown": "kind", "oriented": "kind", "discrete": "repeat", "no_size": "repeat",
    "past_road_end": "on_road", "gap": "continuous", "object_box": "no_object_box", "on_lane": "clear_of_lanes",
    "far_off": "alongside", "sunk": "ground", "floating": "ground", "sideways": "follows_osm", "wrong_kind": "follows_osm",
}


def _shift(obj, reps, f):
    """move every repeat (and the object) sideways: t -> f(t)"""
    for rp in reps:
        rp.set("tStart", repr(f(float(rp.get("tStart")))))
        rp.set("tEnd", repr(f(float(rp.get("tEnd")))))
    obj.set("t", reps[0].get("tStart"))


def _lift(obj, reps, dz):
    for rp in reps:
        for k in ("zOffsetStart", "zOffsetEnd"):
            rp.set(k, repr(float(rp.get(k)) + dz))
    obj.set("zOffset", reps[0].get("zOffsetStart"))


def plant(name, root, oid, od):
    road_el, obj = next((r, o) for r in root.findall("road") for o in r.findall("objects/object") if o.get("id") == oid)
    reps = obj.findall("repeat")
    out = 1.0 if float(reps[0].get("tStart")) > 0 else -1.0  # away from the reference line
    if name == "subtype_unknown":
        obj.set("subtype", "guardrail")
    elif name == "oriented":
        obj.set("orientation", "+")
    elif name == "discrete":
        reps[0].set("distance", "5")
    elif name == "no_size":
        reps[1].set("widthStart", "0")
    elif name == "past_road_end":
        reps[-1].set("length", repr(float(road_el.get("length")) - float(reps[-1].get("s")) + 10.0))
    elif name == "gap":
        reps[1].set("s", repr(float(reps[1].get("s")) + 1.0))
    elif name == "object_box":
        obj.set("length", "10")
    elif name == "on_lane":
        r = od.roads[road_el.get("id")]
        s = float(reps[0].get("s")) + 0.5 * float(reps[0].get("length"))
        bd = r.borders_at(s)
        lane = next(l for l in r.section_at(s).lanes() if l.type not in barrier_audit.MAY_STAND_ON and abs(bd[l.id][1] - bd[l.id][0]) > 2.0)
        tc = 0.5 * sum(bd[lane.id])
        _shift(obj, reps, lambda t: tc)
    elif name == "far_off":
        _shift(obj, reps, lambda t: t + 8.0 * out)
    elif name == "sunk":
        # its bottom 0.3 m below the ground (a guard rail's beam starts 0.44 m above it)
        _lift(obj, reps, -0.3 - (0.44 if obj.get("subtype") == "guardRail" else 0.0))
    elif name == "floating":
        _lift(obj, reps, 1.0)
    elif name == "sideways":
        _shift(obj, reps, lambda t: t + 0.5 * out)
    elif name == "wrong_kind":
        obj.set("subtype", "jerseyBarrier" if obj.get("subtype") == "guardRail" else "guardRail")
    else:
        raise ValueError(name)


def other_road_target(od):
    """the first barrier whose start has another road on the same level (|dz| < 1 m) with a lane in use (> 2 m wide)
    whose centre lies within 12 m sideways: (object id, the t shift that puts the barrier's start on that lane centre)"""
    for r in od.roads.values():
        objs = r.el.find("objects")
        for o in (objs.findall("object") if objs is not None else []):
            if o.get("type") != "barrier":
                continue
            rp = o.find("repeat")
            s, t = float(rp.get("s")), float(rp.get("tStart"))
            x, y = (float(v[0]) for v in r.point(np.array([s]), np.array([t])))
            z = float(r.z(np.array([s]))[0])
            for q in od.roads_near(x, y, 30.0):
                if q is r:
                    continue
                qs, qt, _ = q.project(x, y)
                qx, qy, qh = (float(v[0]) for v in q.ref(np.array([qs])))
                if not (1e-3 < qs < q.length - 1e-3) or abs((x - qx) * np.cos(qh) + (y - qy) * np.sin(qh)) > 0.5 \
                        or abs(float(q.z(np.array([qs]))[0]) - z) > 1.0:
                    continue
                bd = q.borders_at(qs)
                for l in q.section_at(qs).lanes():
                    a2, b2 = min(bd[l.id]), max(bd[l.id])
                    if l.type in barrier_audit.MAY_STAND_ON or b2 - a2 < 2.0:
                        continue
                    cx, cy = (float(v[0]) for v in q.point(np.array([qs]), np.array([0.5 * (a2 + b2)])))
                    s2, t2, _ = r.project(cx, cy)
                    if abs(s2 - s) < 0.5 and abs(t2 - t) <= 12.0:
                        return o.get("id"), t2 - t
    raise SystemExit("no barrier with another road's lane within 12 m")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("base")
    ap.add_argument("--osm", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, "base.xodr")
    shutil.copyfile(a.base, base)
    clean = barrier_audit.audit(base, a.osm, quiet=True)
    if clean["fails"] or not clean["barriers"]:
        raise SystemExit(f"base file needs barriers and a clean audit: {clean['barriers']}, {dict(clean['fails'])}")
    od = G.OpenDrive(base)
    tree0 = ET.parse(base)
    objs = [o for o in tree0.getroot().iter("object") if o.get("type") == "barrier" and len(o.findall("repeat")) >= 3]
    targets = [next(o.get("id") for o in objs if o.get("subtype") == sub) for sub in ("guardRail", "jerseyBarrier")]
    print(f"base: {clean['barriers']} barriers, clean; planted objects {targets}")
    results, failed = {}, 0
    for oid in targets:
        for name, check in EXPECT.items():
            tree = copy.deepcopy(tree0)
            plant(name, tree.getroot(), oid, od)
            path = os.path.join(a.out, f"{oid}_{name}.xodr")
            tree.write(path, encoding="utf-8", xml_declaration=True)
            r = barrier_audit.audit(path, a.osm, quiet=True)
            here = dict(r["by_object"].get(oid, {}))
            elsewhere = {k: dict(v) for k, v in r["by_object"].items() if k != oid}
            ok = check in here and not elsewhere
            results[f"{oid}:{name}"] = {"pass": ok, "expected": check, "found_there": here, "elsewhere": elsewhere}
            failed += not ok
            print(f"{'PASS' if ok else 'FAIL'} {oid} {name}: expected {check}, found {here}, elsewhere {elsewhere}")
    # onto_other_road: a barrier moved sideways onto a lane in use of another road on the same level
    oid, delta = other_road_target(od)
    tree = copy.deepcopy(tree0)
    obj = next(o for o in tree.getroot().iter("object") if o.get("id") == oid)
    _shift(obj, obj.findall("repeat"), lambda t: t + delta)
    path = os.path.join(a.out, f"{oid}_onto_other_road.xodr")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    r = barrier_audit.audit(path, a.osm, quiet=True)
    here = dict(r["by_object"].get(oid, {}))
    elsewhere = {k: dict(v) for k, v in r["by_object"].items() if k != oid}
    ok = "clear_of_other_roads" in here and not elsewhere
    results[f"{oid}:onto_other_road"] = {"pass": ok, "expected": "clear_of_other_roads", "shift": delta,
                                         "found_there": here, "elsewhere": elsewhere}
    failed += not ok
    print(f"{'PASS' if ok else 'FAIL'} {oid} onto_other_road (t {delta:+.2f} m): expected clear_of_other_roads, found {here}, elsewhere {elsewhere}")
    with open(os.path.join(a.out, "selftest.json"), "w", encoding="utf-8") as f:
        json.dump({"base": a.base, "objects": targets, "plants": results}, f, indent=1)
    print(f"{len(results) - failed}/{len(results)} plants detected as expected")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
