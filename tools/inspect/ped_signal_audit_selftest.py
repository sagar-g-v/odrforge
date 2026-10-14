"""ped_signal_audit_selftest -- plant known defects into a real OdrForge output and check that ped_signal_audit.py
reports each of them for the planted crossing (or the file-level check) and nothing for any other crossing.

    python ped_signal_audit_selftest.py BASE.xodr --out DIR

BASE is a standard-profile output with pedestrian lights on cross paths (e.g. the Tokyo test area); it must audit clean.
The first crossing road with pedestrian lights is planted. Plants (expected):
  light_removed          one pedestrian light deleted                          pair
  light_flipped          the + light turned to -                               pair
  reference_removed      a light's reference to its stop line deleted          stop_line
  stop_line_type         the stop line's type set to 1000001                   stop_line
  stop_line_semantics    the stop line's priority set to stopLine              stop_line
  dependency_removed     the stop line's dependency on its light deleted       dependency
  stop_line_moved        the + stop line moved to the far kerb                  order
  controller_split       one light taken out of the crossing's controller      controller
  junction_unlisted      the crossing's controller removed from the junction   controller
  controller_dangling    a controller controls a signal id that does not exist controller_valid (file)
  lane_one_way           the walking lane's @direction set to standard          lane_direction
  validity_wrong_side    the stop line of the wrong side given <validity>     stop_line
                         over lane -1 ("-" in RHT, "+" in LHT: road.signal.validity.*_hand_traffic_lane_ids)
Exit status 1 when any plant fails; DIR/selftest.json holds the findings per plant.
"""
import argparse
import copy
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ped_signal_audit  # noqa: E402

EXPECT = {
    "light_removed": ("pair", False), "light_flipped": ("pair", False), "reference_removed": ("stop_line", False),
    "stop_line_type": ("stop_line", False), "stop_line_semantics": ("stop_line", False),
    "dependency_removed": ("dependency", False), "stop_line_moved": ("order", False), "controller_split": ("controller", False),
    "junction_unlisted": ("controller", False), "controller_dangling": ("controller_valid", True),
    "lane_one_way": ("lane_direction", False), "validity_wrong_side": ("stop_line", False),
}


def plant(name, root, rid):
    road = next(r for r in root.findall("road") if r.get("id") == rid)
    sigs = road.find("signals")
    lights = [s for s in sigs.findall("signal") if s.get("type") == "1000002"]
    plus = next(s for s in lights if s.get("orientation") == "+")
    by_id = {s.get("id"): s for s in sigs.findall("signal")}
    stop = by_id[plus.find("reference").get("elementId")]
    ctrl = next(c for c in root.findall("controller") if any(x.get("signalId") == plus.get("id") for x in c.findall("control")))
    if name == "light_removed":
        sigs.remove(plus)
        for c in root.findall("controller"):
            for x in c.findall("control"):
                if x.get("signalId") == plus.get("id"):
                    c.remove(x)
        for d in stop.findall("dependency"):
            stop.remove(d)
    elif name == "light_flipped":
        plus.set("orientation", "-")
    elif name == "reference_removed":
        plus.remove(plus.find("reference"))
    elif name == "stop_line_type":
        stop.set("type", "1000001")
    elif name == "stop_line_semantics":
        stop.find("semantics/priority").set("type", "stopLine")
    elif name == "dependency_removed":
        for d in stop.findall("dependency"):
            stop.remove(d)
    elif name == "stop_line_moved":
        # the + waiting line moved to the far kerb (the - line's place): still before its light, but not where + walkers wait
        minus = next(s for s in lights if s.get("orientation") == "-")
        stop.set("s", by_id[minus.find("reference").get("elementId")].get("s"))
    elif name == "controller_split":
        ctrl.remove(next(x for x in ctrl.findall("control") if x.get("signalId") == plus.get("id")))
        new = ET.SubElement(root, "controller", {"id": "999999", "name": "planted"})
        ET.SubElement(new, "control", {"signalId": plus.get("id")})
    elif name == "junction_unlisted":
        for j in root.findall("junction"):
            for c in j.findall("controller"):
                if c.get("id") == ctrl.get("id"):
                    j.remove(c)
    elif name == "controller_dangling":
        new = ET.SubElement(root, "controller", {"id": "999998", "name": "planted"})
        ET.SubElement(new, "control", {"signalId": "no-such-signal"})
    elif name == "validity_wrong_side":
        # the walking lane is -1: "-" may not span it in right-hand traffic, "+" may not in left-hand traffic
        wrong = "+" if road.get("rule") == "LHT" else "-"
        minus = next(s for s in lights if s.get("orientation") == wrong)
        mstop = by_id[minus.find("reference").get("elementId")]
        walk = [int(ln.get("id")) for ln in road.findall("lanes/laneSection/*/lane") if ln.get("type") == "walking"]
        v = ET.Element("validity", {"fromLane": str(min(walk)), "toLane": str(max(walk))})
        mstop.insert(0, v)
    elif name == "lane_one_way":
        for ln in road.findall("lanes/laneSection/*/lane"):
            if ln.get("type") == "walking":
                ln.set("direction", "standard")
    else:
        raise ValueError(name)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("base")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    base = os.path.join(a.out, "base.xodr")
    shutil.copyfile(a.base, base)
    clean = ped_signal_audit.audit(base, quiet=True)
    if clean["fails"] or not clean["crossings"]:
        raise SystemExit(f"base file needs crossings with pedestrian lights and a clean audit: {clean['crossings']}, {dict(clean['fails'])}")
    tree0 = ET.parse(base)
    roots = tree0.getroot()
    crossing = {cp.get("crossingRoad") for j in roots.findall("junction") for cp in j.findall("crossPath")}
    rid = next(r.get("id") for r in roots.findall("road")
               if r.get("id") in crossing and any(s.get("type") == "1000002" for s in r.findall("signals/signal")))
    print(f"base: {clean['crossings']} crossings with pedestrian lights, clean; planted crossing road {rid}")
    results, failed = {}, 0
    for name, (check, file_level) in EXPECT.items():
        tree = copy.deepcopy(tree0)
        plant(name, tree.getroot(), rid)
        path = os.path.join(a.out, name + ".xodr")
        tree.write(path, encoding="utf-8", xml_declaration=True)
        r = ped_signal_audit.audit(path, quiet=True)
        where = "file" if file_level else rid
        here = dict(r["by_road"].get(where, {}))
        elsewhere = {k: dict(v) for k, v in r["by_road"].items() if k not in (rid, "file")}
        ok = check in here and not elsewhere
        results[name] = {"pass": ok, "expected": check, "at": where, "found_there": here, "elsewhere": elsewhere}
        failed += not ok
        print(f"{'PASS' if ok else 'FAIL'} {name}: {results[name]}")
    with open(os.path.join(a.out, "selftest.json"), "w", encoding="utf-8") as f:
        json.dump({"base": a.base, "crossing_road": rid, "plants": results}, f, indent=1)
    print(f"{len(results) - failed}/{len(results)} plants detected as expected")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
