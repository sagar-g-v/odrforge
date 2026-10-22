"""Planted-defect test of the NAT-2 lane-rule case in both checkers (tools/inspect/laneflow.py split.section_redundant,
tools/inspect/invariants.py needless_split): a street tagged parking:both=lane across a junction is converted
(odrforge build, standard profile); within the country's no-parking distance of the junction its parking lanes carry the lane
rule "no stopping at any time" (OpenDRIVE 1.8.0 §11.9) in a section of their own. Such a boundary is not redundant (a
rule has no end of its own). Planted: the rules removed - the same boundaries then are redundant, and both checkers
must say so.
usage: python lane_rule_selftest.py <odrforge.exe>   (exit 0 = pass; ctest)"""
import math
import os
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
import invariants as INV  # noqa: E402
import laneflow as LF  # noqa: E402

failures = []
LAT0, LON0 = 48.1, 11.5
NO_STOPPING = "no stopping at any time"


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def ll(x, y):  # local metres -> lat, lon (equirectangular about the origin; ample over 300 m)
    return LAT0 + y / 111132.954, LON0 + x / (111319.49 * math.cos(math.radians(LAT0)))


def write_osm(path):
    nodes, ways, nid = [], [], [1]

    def node(x, y):
        lat, lon = ll(x, y)
        nodes.append(f'<node id="{nid[0]}" lat="{lat:.9f}" lon="{lon:.9f}"/>')
        nid[0] += 1
        return nid[0] - 1

    street = [node(-150, 0), node(0, 0), node(150, 0)]
    ways.append((100, street, {"highway": "residential", "parking:both": "lane"}))
    ways.append((102, [node(0, -100), street[1], node(0, 100)], {"highway": "residential"}))
    body = "\n".join(nodes + [f'<way id="{w}">' + "".join(f'<nd ref="{n}"/>' for n in nds)
                              + "".join(f'<tag k="{k}" v="{v}"/>' for k, v in t.items()) + "</way>" for w, nds, t in ways])
    with open(path, "w", encoding="utf-8") as f:
        f.write(f'<?xml version="1.0"?>\n<osm version="0.6">\n{body}\n</osm>\n')


def convert(exe, osm, xodr):
    """odrforge build, standard profile; once more after a pause if it fails (a just-linked exe can be held by a scan)"""
    args = [exe, "build", osm, xodr, "--target", "standard", "--country", "DE", "--origin", f"{LAT0},{LON0}"]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        time.sleep(3)
        r = subprocess.run(args, capture_output=True, text=True)
    return r


def redundant(xodr):
    """(laneflow split.section_redundant count, invariants needless_split kind=section count)"""
    lf = LF.LaneFlow(xodr, None)
    lf.splits()
    a = lf.F.d.get("split.section_redundant", {}).get("count", 0)
    res = INV.run_all(INV.Network(xodr))
    b = sum(1 for i in res.items.get("needless_split", []) if i.get("kind") == "section")
    return a, b


def main():
    exe = os.path.abspath(sys.argv[1])
    with tempfile.TemporaryDirectory() as tmp:
        osm, xodr = os.path.join(tmp, "cross.osm"), os.path.join(tmp, "cross.xodr")
        write_osm(osm)
        r = convert(exe, osm, xodr)
        check(r.returncode == 0 and "no stopping at any time" in r.stdout, f"odrforge build writes the rule ({r.stdout[-300:]})")
        tree = ET.parse(xodr)
        ruled = [lane for lane in tree.getroot().iter("lane") if lane.get("type") == "parking" and
                 any(u.get("value") == NO_STOPPING for u in lane.findall("rule"))]
        check(len(ruled) >= 4, f"parking lanes beside the junction carry the rule ({len(ruled)})")
        a, b = redundant(xodr)
        check(a == 0 and b == 0, f"a section boundary where the lane rule changes is not redundant ({a}, {b})")
        # planted: the rules removed - the boundaries at the no-stopping stretches are redundant now
        for lane in tree.getroot().iter("lane"):
            for u in lane.findall("rule"):
                lane.remove(u)
        bare = os.path.join(tmp, "bare.xodr")
        tree.write(bare, encoding="utf-8", xml_declaration=True)
        a, b = redundant(bare)
        check(a >= 2 and b >= 2, f"without the rules both checkers flag the boundaries ({a}, {b})")
    print(f"lane-rule section self-test: {len(failures)} failures")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
