"""Planted-defect test of laneflow's street-side parking explanation (tools/inspect/laneflow.py edge.break.osm_area):
a street with a street-side parking area beside part of it is converted (odrforge build, standard profile); the parking lane
(and its strip) starting and stopping at the area grow from / fall to zero width there and are explained by the area.
Planted: the area missing from the OSM laneflow reads, or the lane starting at full width (a jump) - neither may be
explained by the area (the tiny synthetic map gives laneflow no road-to-OSM mapping, so such a break is classed
osm_unknown here, unexplained in the reference areas).
usage: python laneflow_area_selftest.py <odrforge.exe>   (exit 0 = pass; ctest)"""
import math
import os
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
import laneflow as LF  # noqa: E402
import parking_coverage  # noqa: E402

failures = []
LAT0, LON0 = 48.1, 11.5


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def ll(x, y):  # local metres -> lat, lon (equirectangular about the origin; ample over 300 m)
    return LAT0 + y / 111132.954, LON0 + x / (111319.49 * math.cos(math.radians(LAT0)))


def write_osm(path, with_area):
    nodes, ways, nid = [], [], [1]

    def node(x, y):
        lat, lon = ll(x, y)
        nodes.append(f'<node id="{nid[0]}" lat="{lat:.9f}" lon="{lon:.9f}"/>')
        nid[0] += 1
        return nid[0] - 1

    ways.append((100, [node(-150, 0), node(0, 0), node(150, 0)], {"highway": "residential"}))
    if with_area:  # 0.2 m beyond a 3 m carriageway edge, 2.4 m deep, ends cut at 45 deg
        yi, yo = -3.4, -5.8
        ring = [node(-40, yi), node(20, yi), node(17.6, yo), node(-37.6, yo)]
        ways.append((101, ring + [ring[0]], {"amenity": "parking", "parking": "street_side"}))
    body = "\n".join(nodes + [f'<way id="{w}">' + "".join(f'<nd ref="{n}"/>' for n in nds)
                              + "".join(f'<tag k="{k}" v="{v}"/>' for k, v in t.items()) + "</way>" for w, nds, t in ways])
    with open(path, "w", encoding="utf-8") as f:
        f.write(f'<?xml version="1.0"?>\n<osm version="0.6">\n{body}\n</osm>\n')


def convert(exe, osm, xodr, extra=()):
    """odrforge build, standard profile; once more after a pause if it fails (a just-linked exe can be held by a scan)"""
    args = [exe, "build", osm, xodr, "--target", "standard", "--country", "DE", "--origin", f"{LAT0},{LON0}", *extra]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        time.sleep(3)
        r = subprocess.run(args, capture_output=True, text=True)
    return r


def breaks(xodr, osm):
    lf = LF.LaneFlow(xodr, osm)
    lf.edge_continuity()
    out = {}
    for name, e in lf.F.d.items():
        if name.startswith("edge.break."):
            out[name] = [i for i in e["instances"] if i.get("lane_type") in ("parking", "border")]
    return out


def main():
    exe = os.path.abspath(sys.argv[1])
    with tempfile.TemporaryDirectory() as tmp:
        osm, bare, xodr = (os.path.join(tmp, f) for f in ("area.osm", "bare.osm", "out.xodr"))
        write_osm(osm, True)
        write_osm(bare, False)
        r = convert(exe, osm, xodr)
        check(r.returncode == 0 and "street-side parking: 1 mapped areas drawn" in r.stdout,
              f"odrforge build draws the area ({r.stdout[-400:]}{r.stderr[-400:]})")
        b = breaks(xodr, osm)
        explained, unexplained = b.get("edge.break.osm_area", []), b.get("edge.break.unexplained", [])
        check(len(explained) >= 2, f"the lane's start and end are explained by the area ({len(explained)})")
        check(not unexplained, f"no unexplained parking / border break ({unexplained[:2]})")
        # planted: laneflow reads the OSM without the area - the same breaks are not explained by it
        b = breaks(xodr, bare)
        others = [i for k, v in b.items() if k != "edge.break.osm_area" for i in v]
        check(not b.get("edge.break.osm_area") and len(others) >= 2, "without the area the breaks are not explained")
        # planted: the parking lane starts at full width (a jump, not a growth from zero)
        tree = ET.parse(xodr)
        jumped = 0
        for lane in tree.getroot().iter("lane"):
            link = lane.find("link")
            if lane.get("type") == "parking" and (link is None or link.find("predecessor") is None):
                widths = lane.findall("width")
                for w in widths[1:]:
                    lane.remove(w)
                widths[0].attrib.update({"sOffset": "0", "a": "2.4", "b": "0", "c": "0", "d": "0"})
                jumped += 1
        tree.write(xodr, encoding="utf-8", xml_declaration=True)
        check(jumped >= 1, "a parking lane starting in a section was found to plant the jump into")
        b = breaks(xodr, osm)
        check(not any(i.get("kind") == "start" for i in b.get("edge.break.osm_area", [])) and
              any(i.get("kind") == "start" for v in b.values() for i in v),
              "a parking lane starting at full width is not explained by the area")
        # the clip box in the header (odrforge build --clip) decides which areas the coverage measure counts
        for (x0, y0, x1, y1), want in (((-120, -60, 120, 60), 1), ((30, -60, 120, 60), 0)):
            (la0, lo0), (la1, lo1) = ll(x0, y0), ll(x1, y1)
            box = f"{la0:.6f},{lo0:.6f},{la1:.6f},{lo1:.6f}"
            cx = os.path.join(tmp, "clip.xodr")
            r = convert(exe, osm, cx, ("--clip", box))
            head = ET.parse(cx).getroot().find("header")
            written = [u.get("value") for u in head.findall("userData") if u.get("code") == "clip"]
            check(written == [box], f"the header carries the clip box ({written} vs {box})")
            summary, _ = parking_coverage.measure(osm, cx)
            check(summary["polygons"] == want, f"areas wholly inside the header's box: {summary['polygons']} (want {want})")
    print(f"laneflow area self-test: {len(failures)} failures")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
