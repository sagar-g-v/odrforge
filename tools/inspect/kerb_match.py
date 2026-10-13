"""How closely the sidewalks' curb-side edges follow the OSM kerb lines (barrier=kerb).

    python kerb_match.py FILE.xodr AREA

Samples every 2 m the curb-side (inner) edge of every sidewalk lane on plain roads; where an OSM kerb
line lies within 2 m of it, records the distance to that line. Reports the share within 0.2 m.
"""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__file__))
import odrgeom as G  # noqa: E402
from gate import AREAS  # noqa: E402


def transverse_mercator(lat0, lon0):
    """The converter's projection (src/projection.cpp): WGS84 transverse Mercator (Krueger series) on
    lon0, y measured from lat0."""
    a, f = 6378137.0, 1.0 / 298.257223563
    n = f / (2.0 - f)
    A = a / (1.0 + n) * (1.0 + n * n / 4.0 + n ** 4 / 64.0)
    alpha = [n / 2.0 - 2.0 * n * n / 3.0 + 5.0 * n ** 3 / 16.0, 13.0 * n * n / 48.0 - 3.0 * n ** 3 / 5.0,
             61.0 * n ** 3 / 240.0]
    e2n = 2.0 * math.sqrt(n) / (1.0 + n)

    def raw(lat, lon):
        phi, lam = math.radians(lat), math.radians(lon - lon0)
        t = math.sinh(math.atanh(math.sin(phi)) - e2n * math.atanh(e2n * math.sin(phi)))
        xi, eta = math.atan2(t, math.cos(lam)), math.atanh(math.sin(lam) / math.sqrt(1.0 + t * t))
        E, N = eta, xi
        for j in range(1, 4):
            E += alpha[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
            N += alpha[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
        return A * E, A * N

    y0 = raw(lat0, lon0)[1]
    return lambda lat, lon: (raw(lat, lon)[0], raw(lat, lon)[1] - y0)


def kerb_segments(osm_path, lat0, lon0):
    root = ET.parse(osm_path).getroot()
    project = transverse_mercator(lat0, lon0)
    pos = {}
    for n in root.iter("node"):
        pos[n.get("id")] = project(float(n.get("lat")), float(n.get("lon")))
    segs = []
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("barrier") != "kerb":
            continue
        ids = [nd.get("ref") for nd in w.findall("nd")]
        if len(ids) < 2 or ids[0] == ids[-1]:
            continue
        pts = [pos[i] for i in ids if i in pos]
        segs += [(pts[k], pts[k + 1]) for k in range(len(pts) - 1)]
    return np.array(segs)


def point_to_segments(p, segs):
    a, b = segs[:, 0, :], segs[:, 1, :]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
    return np.hypot(*(a + ab * t[:, None] - p).T).min()


def main(xodr, area):
    args = AREAS[area]["args"]
    lat0, lon0 = map(float, args[args.index("--origin") + 1].split(","))
    segs = kerb_segments(AREAS[area]["osm"], lat0, lon0)
    od = G.OpenDrive(xodr)
    gaps = []
    for r in (od.roads.values() if isinstance(od.roads, dict) else od.roads):
        if r.el.get("junction") != "-1":
            continue
        for k, sec in enumerate(r.sections):
            s1 = r.sections[k + 1].s if k + 1 < len(r.sections) else r.length
            types = {l.id: l.type for l in sec.lanes()}
            for lane in sec.lanes():
                # only sidewalks attached from mapped geometry: a verge (border) lies inside them
                inner = lane.id - 1 if lane.id > 0 else lane.id + 1
                if lane.type not in ("sidewalk", "walking") or types.get(inner) != "border":
                    continue
                for s in np.arange(sec.s + 1.0, s1 - 1.0, 2.0):
                    tin = r.borders_at(s, sec)[lane.id][0]
                    px, py = r.point(np.array([s]), tin)
                    d = point_to_segments(np.array([px[0], py[0]]), segs)
                    if d < 2.0:
                        gaps.append(d)
    gaps = np.array(gaps)
    if not len(gaps):
        print("no sidewalk edge within 2 m of a kerb line")
        return
    print(f"{len(gaps)} curb-edge samples near kerb lines: within 0.2 m {np.mean(gaps < 0.2):.1%}, "
          f"median {np.median(gaps):.2f} m, 90th percentile {np.percentile(gaps, 90):.2f} m")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
