"""Planted-defect test of tools/inspect/kerb_gap.py (the proof measure of kerb-anchored carriageway edges) on a converted
crossroads: a street with kerb lines 4 m either side of it (clear of the junction), a cross street, and a traffic
island ringed by a kerb of its own.
- converted (odrforge build, standard profile): the edges lie on the kerbs - |median| <= 0.05 m, >= 90 % within 0.3 m, the
  island's kerb left out;
- planted: converted with --no-kerb-edges (the lanes at their tagged widths) - the measure must see the edges inside
  the kerbs (median >= +0.3 m, < 20 % within 0.3 m).
usage: python kerb_gap_selftest.py <odrforge.exe>   (exit 0 = pass; ctest)"""
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
import kerb_gap as KG  # noqa: E402

failures = []
LAT0, LON0 = 51.5, -0.1


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
    ways.append((100, street, {"highway": "residential", "lanes": "2"}))
    ways.append((101, [node(0, -100), street[1], node(0, 100)], {"highway": "residential", "lanes": "2"}))
    w = 200
    for y in (4.0, -4.0):
        for x0, x1 in ((-140.0, -25.0), (25.0, 140.0)):
            ways.append((w, [node(x0, y), node(x1, y)], {"barrier": "kerb"}))
            w += 1
    ring = [node(60, 10), node(100, 10), node(100, 14), node(60, 14)]
    ways.append((w, ring + [ring[0]], {"area:highway": "traffic_island"}))
    ways.append((w + 1, [node(60, 10.1), node(100, 10.1)], {"barrier": "kerb"}))  # the island's own kerb
    body = "\n".join(nodes + [f'<way id="{i}">' + "".join(f'<nd ref="{n}"/>' for n in nds)
                              + "".join(f'<tag k="{k}" v="{v}"/>' for k, v in t.items()) + "</way>" for i, nds, t in ways])
    with open(path, "w", encoding="utf-8") as f:
        f.write(f'<?xml version="1.0"?>\n<osm version="0.6">\n{body}\n</osm>\n')


def convert(exe, osm, xodr, extra=()):
    """odrforge build, standard profile; once more after a pause if it fails (a just-linked exe can be held by a scan)"""
    args = [exe, "build", osm, xodr, "--target", "standard", "--country", "GB", "--origin", f"{LAT0},{LON0}", *extra]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        time.sleep(3)
        r = subprocess.run(args, capture_output=True, text=True)
    return r


def measure(osm, xodr):
    _, gaps, kerbs, dropped, samples, matched = KG.measure(osm, xodr)
    g = np.array(gaps)
    return {"kerbs": kerbs, "dropped": dropped, "matched": matched, "median": float(np.median(g)) if len(g) else math.nan,
            "within": float(np.mean(np.abs(g) <= 0.3)) if len(g) else 0.0}


def main():
    exe = os.path.abspath(sys.argv[1])
    with tempfile.TemporaryDirectory() as tmp:
        osm = os.path.join(tmp, "kerbs.osm")
        write_osm(osm)
        good, plain = os.path.join(tmp, "kerbs.xodr"), os.path.join(tmp, "plain.xodr")
        r = convert(exe, osm, good)
        check(r.returncode == 0 and "kerb edges" in r.stdout, f"odrforge build follows the kerbs ({r.stdout[-300:]})")
        r = convert(exe, osm, plain, ("--no-kerb-edges",))
        check(r.returncode == 0, f"odrforge build --no-kerb-edges ({r.stdout[-300:]})")
        a, b = measure(osm, good), measure(osm, plain)
        print("kerb edges:", a)
        print("planted (no kerb edges):", b)
        check(a["kerbs"] == 5 and a["dropped"] == 1, f"5 open kerbs, the island's left out ({a})")
        check(a["matched"] > 400, f"the kerb samples find the edges ({a})")
        check(abs(a["median"]) <= 0.05 and a["within"] >= 0.9, f"edges on the kerbs: median ~0, >= 90 % within 0.3 m ({a})")
        check(b["median"] >= 0.3 and b["within"] < 0.2, f"planted: the edges inside the kerbs are seen ({b})")
    print("kerb_gap_selftest:", "FAIL" if failures else "ok")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
