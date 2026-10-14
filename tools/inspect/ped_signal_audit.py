"""Audit pedestrian traffic lights on cross paths against the ASAM Junction Guideline 1.0.0 §6.3.3 (Codes 20 - 23),
OpenDRIVE 1.8.0 §14 and the ASAM Signal reference 1.0.0.

Per crossing road (a cross path's crossingRoad) that carries a pedestrian light (type 1000002):
  pair            two lights, type 1000002, country OpenDRIVE, dynamic, one per walking direction (orientation + and -)
  stop_line       each light references (elementType signal, type stopLine) one signal on the same road that is an
                  invisible stop line: type 1100001, country OpenDRIVE, the light's orientation, semantics priority
                  waitingLine, s on the road; no <validity> (it then holds for all lanes in its orientation, 1.8.0
                  §14.2), or one covering the walking lane on the side road.signal.validity.*_hand_traffic_lane_ids
                  (1.7.0) allows for its orientation ("+" negative ids in RHT / positive in LHT, "-" the reverse)
  dependency      that stop line depends on that light (<dependency id=light>)
  order           pedestrians wait at the kerb they start from and see their light across the road: for + the stop
                  line lies in the first half of the crossing road and its light in the second, for - the reverse
  controller      both lights are controlled by one controller holding only these two lights, and the junction lists
                  that controller (<junction><controller id>)
  lane_direction  the walking lane is bi-directional (@direction both, OpenDRIVE 1.8.0 §11.2), so signals of either
                  orientation may apply to it
Over the file:
  controller_valid  1.7.0 road.signal.controller.valid_for_signals: every controller controls one or more signals,
                  each an existing signal
  virtual_no_controllers  1.9.0 junctions.virtual.no_controllers
  links_resolve   every <reference> / <dependency> id of a pedestrian light or its stop line names an existing signal
usage: python ped_signal_audit.py file.xodr [...]
"""
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict


def audit(path, quiet=False):
    """{"crossings": n lit crossing roads, "fails": Counter, "by_road": {crossing road id: Counter}}"""
    root = ET.parse(path).getroot()
    fails, by_road = Counter(), defaultdict(Counter)

    def add(check, rid):
        fails[check] += 1
        by_road[rid][check] += 1

    signals = {}  # id -> (signal element, road element)
    for r in root.findall("road"):
        for s in r.findall("signals/signal"):
            signals[s.get("id")] = (s, r)
    controllers = {c.get("id"): [x.get("signalId") for x in c.findall("control")] for c in root.findall("controller")}
    owner = defaultdict(list)  # signal id -> controller ids
    for cid, sids in controllers.items():
        for sid in sids:
            owner[sid].append(cid)
        if not sids or any(sid not in signals for sid in sids):
            add("controller_valid", "file")
    junction_ctrl = {j.get("id"): {c.get("id") for c in j.findall("controller")} for j in root.findall("junction")}
    for j in root.findall("junction"):
        if j.get("type") == "virtual" and j.find("controller") is not None:
            add("virtual_no_controllers", "file")
    crossing_roads = {cp.get("crossingRoad"): j.get("id") for j in root.findall("junction") for cp in j.findall("crossPath")}
    n = 0
    for r in root.findall("road"):
        rid = r.get("id")
        if rid not in crossing_roads:
            continue
        lights = [s for s in r.findall("signals/signal") if s.get("type") == "1000002"]
        if not lights:
            continue
        n += 1
        L = float(r.get("length"))
        if len(lights) != 2 or {s.get("orientation") for s in lights} != {"+", "-"} or \
                any(s.get("country") != "OpenDRIVE" or s.get("dynamic") != "yes" for s in lights):
            add("pair", rid)
        walk = [ln for ln in r.findall("lanes/laneSection/*/lane") if ln.get("type") == "walking"]
        if not walk or any(ln.get("direction") != "both" for ln in walk):
            add("lane_direction", rid)
        walk_ids = {int(ln.get("id")) for ln in walk}
        for lt in lights:
            refs = [x for x in lt.findall("reference") if x.get("elementType") == "signal" and x.get("type") == "stopLine"]
            stops = [signals.get(x.get("elementId")) for x in refs]
            if len(refs) != 1 or stops[0] is None or stops[0][1] is not r:
                add("stop_line", rid)
                continue
            st = stops[0][0]
            lht = r.get("rule") == "LHT"
            ok_validity = True
            for val in st.findall("validity"):
                lo, hi = int(val.get("fromLane")), int(val.get("toLane"))
                negative = (st.get("orientation") == "+") != lht  # the side this orientation may span
                ok_validity = ok_validity and (hi < 0 if negative else lo > 0) and all(lo <= w <= hi for w in walk_ids)
            sem = st.find("semantics/priority")
            if (st.get("type") != "1100001" or st.get("country") != "OpenDRIVE" or st.get("orientation") != lt.get("orientation")
                    or not ok_validity or sem is None or sem.get("type") != "waitingLine" or not (0.0 <= float(st.get("s")) <= L)):
                add("stop_line", rid)
            if not any(d.get("id") == lt.get("id") for d in st.findall("dependency")):
                add("dependency", rid)
            ss, sl, half = float(st.get("s")), float(lt.get("s")), 0.5 * L
            if (lt.get("orientation") == "+" and not (ss < half < sl)) or (lt.get("orientation") == "-" and not (sl < half < ss)):
                add("order", rid)
            for x in list(lt.findall("reference")) + list(st.findall("dependency")):
                if (x.get("elementId") or x.get("id")) not in signals:
                    add("links_resolve", rid)
        cids = [set(owner.get(lt.get("id"), [])) for lt in lights]
        common = set.intersection(*cids) if cids else set()
        if len(common) != 1 or any(len(c) != 1 for c in cids):
            add("controller", rid)
        else:
            cid = next(iter(common))
            if set(controllers[cid]) != {lt.get("id") for lt in lights} or cid not in junction_ctrl.get(crossing_roads[rid], set()):
                add("controller", rid)
    if not quiet:
        print(f"{path}: {n} crossings with pedestrian lights; fails {dict(fails)}")
    return {"crossings": n, "fails": fails, "by_road": dict(by_road)}


if __name__ == "__main__":
    for p in sys.argv[1:]:
        audit(p)
