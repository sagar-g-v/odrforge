"""Jev reviews junctions as written: the complete OpenDRIVE of each junction goes to TypeSafe Jev with
typed questions, and the junctions it doubts come back for a fix. Run after every correction until
Jev accepts them (the loop the user asked for); the exact checkers stay the gate.

    python jev_judge.py FILE.xodr [--junctions 12,57] [--sample 20] [--findings DIR] [--json OUT]

Per junction the state holds: the <junction> element, the full XML of all its connecting roads, the
arm roads' geometry and lane section at the junction end, and the exact-checker findings (from
invariants.py / laneflow.py output in --findings) on those roads. Every run also sends a planted
broken copy of the first junction (a lane link to a lane that does not exist, a sidewalk lane cut);
if Jev passes it, its verdicts for that run are not trusted.
"""
import argparse
import copy
import json
import os
import random
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(__file__))
from jev import ask  # noqa: E402

CONTEXT = (
    "You are reviewing one junction of an ASAM OpenDRIVE road network generated from OpenStreetMap. Rules: "
    "OpenDRIVE 1.9 section 12 (a junction lists connections incomingRoad -> connectingRoad with laneLinks from "
    "incoming lane ids to connecting road lane ids; connecting roads have junction=<id>, link predecessor / "
    "successor to arm roads with contactPoint; lane ids: right lanes negative, left positive; rule=LHT means "
    "left-hand traffic where right lanes run against s); ASAM Junction Guideline 1.0.0 section 6 (one junction "
    "per intersection, connecting-road reference lines continue the arm reference lines or are centred on their "
    "lane on tight turns, every incoming driving lane has a path, sidewalks continue round corners on junction "
    "roads along the curb line); geometry: lane borders must not fold (curvature times offset below 1), "
    "lane widths positive. Arm XML is abbreviated to its end at this junction.")

QUESTIONS = {
    "spec_valid": {"type": "noul", "instructions": "Is this junction valid OpenDRIVE: every connection refers to "
                   "roads present, every laneLink 'from' lane exists on the incoming road at the contact end and "
                   "every 'to' lane exists on the connecting road?"},
    "lanes_have_paths": {"type": "noul", "instructions": "Does every driving lane that enters this junction have "
                         "at least one laneLink to a connecting road?"},
    "sidewalks_continuous": {"type": "noul", "instructions": "Where two neighbouring arms both have a sidewalk on "
                             "the corner between them, is there a junction road carrying a sidewalk linked to both "
                             "(or is the corner explained by the findings as too tight)?"},
    "geometry_sound": {"type": "noul", "instructions": "Is the geometry of the connecting roads sound: no folding "
                       "lanes, no negative widths, plausible lengths and curvature for the turns they make?"},
    "guideline_followed": {"type": "noul", "instructions": "Overall, does this junction follow the ASAM Junction "
                           "Guideline modelling (one junction for the intersection, arm-continuous or lane-centred "
                           "connecting roads, all movements present)?"},
}


def compact(el):
    return ET.tostring(el, encoding="unicode").replace("\n", " ")


def arm_end(road, at_end):
    """The arm abbreviated to its end at the junction: link, last / first geometry, lane section."""
    out = ET.Element("road", road.attrib)
    for tag in ("link", "type"):
        for child in road.findall(tag):
            out.append(copy.deepcopy(child))
    geoms = road.findall("planView/geometry")
    pv = ET.SubElement(out, "planView")
    pv.append(copy.deepcopy(geoms[-1] if at_end else geoms[0]))
    sections = road.findall("lanes/laneSection")
    lanes = ET.SubElement(out, "lanes")
    for lo in road.findall("lanes/laneOffset"):
        lanes.append(copy.deepcopy(lo))
    lanes.append(copy.deepcopy(sections[-1] if at_end else sections[0]))
    return out


def findings_for(findings_dir, road_ids):
    lines = []
    if not findings_dir:
        return lines
    inv = os.path.join(findings_dir + ".invariants", "invariants.json")
    if os.path.exists(inv):
        for name, v in json.load(open(inv, encoding="utf-8"))["invariants"].items():
            for i in v.get("instances", []):
                if str(i.get("road")) in road_ids:
                    lines.append(f"invariant {name} ({v.get('spec')}) on road {i['road']} lane {i.get('lane')}: "
                                 f"value {i.get('value')} limit {v.get('limit')}")
    return lines[:40]


def facts(root, jid):
    """Measured summary of the junction XML (read from the same XML, nothing added): the movements per
    incoming lane, the corner sidewalk roads, and per connecting road its turn and lane types."""
    roads = {r.get("id"): r for r in root.iter("road")}
    junction = next(j for j in root.iter("junction") if j.get("id") == jid)
    lines = []
    for c in junction.findall("connection"):
        con = roads.get(c.get("connectingRoad"))
        if con is None:
            lines.append(f"connection {c.get('id')}: connecting road {c.get('connectingRoad')} MISSING")
            continue
        succ = con.find("link/successor")
        types = [l.get("type") for l in con.iter("lane") if l.get("id") != "0"]
        lane_ids = {l.get("id") for l in con.iter("lane")}
        bad = [ll.get("to") for ll in c.findall("laneLink") if ll.get("to") not in lane_ids]
        lines.append(f"connection {c.get('id')}: road {c.get('incomingRoad')} lanes "
                     f"{[ll.get('from') for ll in c.findall('laneLink')]} -> road {con.get('id')} ({'/'.join(types)}) "
                     f"-> road {succ.get('elementId') if succ is not None else '?'}"
                     + (f"; laneLink to-lanes {bad} NOT ON the connecting road" if bad else ""))
    return "SUMMARY OF THE XML: " + "; ".join(lines)


def junction_state(root, jid, findings_dir=None, extra=""):
    roads = {r.get("id"): r for r in root.iter("road")}
    junction = next(j for j in root.iter("junction") if j.get("id") == jid)
    parts = [CONTEXT, facts(root, jid), "JUNCTION: " + compact(junction)]
    connecting = [r for r in roads.values() if r.get("junction") == jid]
    for r in connecting:
        parts.append("CONNECTING ROAD: " + compact(r))
    arms = {}
    for r in connecting:
        for tag in ("predecessor", "successor"):
            link = r.find("link/" + tag)
            if link is not None and link.get("elementId") in roads:
                arms[link.get("elementId")] = link.get("contactPoint") == "end"
    for rid, at_end in arms.items():
        parts.append("ARM ROAD (end at this junction): " + compact(arm_end(roads[rid], at_end)))
    found = findings_for(findings_dir, {r.get("id") for r in connecting} | set(arms))
    parts.append("EXACT CHECKER FINDINGS: " + ("; ".join(found) if found else "none"))
    if extra:
        parts.append(extra)
    return "\n".join(parts)


def planted_defect(root, jid):
    """A broken copy: a laneLink to a lane that does not exist, and a corner sidewalk removed."""
    broken = copy.deepcopy(root)
    junction = next(j for j in broken.iter("junction") if j.get("id") == jid)
    link = junction.find("connection/laneLink")
    if link is not None:
        link.set("to", "-7")
    for r in list(broken.iter("road")):
        if r.get("junction") == jid and r.find(".//lane[@type='sidewalk']") is not None:
            for parent in broken.iter():
                if r in list(parent):
                    parent.remove(r)
                    break
            break
    return broken


def judge(state):
    answers = ask(state, QUESTIONS)
    return {k: round(v.get("noul", 0.0), 3) for k, v in answers.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("--junctions")
    ap.add_argument("--sample", type=int, default=10)
    ap.add_argument("--findings", help="the .xodr path whose .invariants output to attach (default: the file)")
    ap.add_argument("--json")
    a = ap.parse_args()
    root = ET.parse(a.xodr).getroot()
    ids = a.junctions.split(",") if a.junctions else sorted(
        random.Random(7).sample([j.get("id") for j in root.iter("junction")],
                                min(a.sample, sum(1 for _ in root.iter("junction")))), key=int)
    findings = a.findings or a.xodr
    # calibration: each sampled junction is also sent with a planted defect; Jev is trusted on a
    # question only when it rates the planted copies clearly lower than the real ones
    planted = {jid: judge(junction_state(planted_defect(root, jid), jid, None)) for jid in ids}
    results = {}
    for jid in ids:
        state = junction_state(root, jid, findings)
        verdict = judge(state)
        results[jid] = {"verdict": verdict, "state_chars": len(state)}
        doubt = [k for k, p in verdict.items() if p < 0.5]
        print(f"junction {jid:>5} ({len(state):6d} chars): " + ("accepted" if not doubt else "DOUBTS " + ", ".join(
            f"{k}={verdict[k]}" for k in doubt)))
    separation = {q: round(sum(results[j]["verdict"][q] - planted[j][q] for j in ids) / len(ids), 3) for q in QUESTIONS}
    trusted = [q for q, d in separation.items() if d >= 0.2]
    print("separation real - planted per question:", separation, "-> trusted on", trusted or "nothing")
    if a.json:
        json.dump({"planted": planted, "separation": separation, "trusted": trusted, "junctions": results},
                  open(a.json, "w"), indent=1)
    return 0 if trusted else 2


if __name__ == "__main__":
    sys.exit(main())
