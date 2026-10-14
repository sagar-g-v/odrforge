"""Audit junction reference lines and elevation grids (OpenDRIVE 1.8.0 §12.9 / §12.11) against the connecting roads.

Rules measured (rule ids verbatim):
  junctions.geometry.only_one_line_element   one <geometry> with one <line>
  junctions.geometry.ref_line_definition     every junction point reachable by a perpendicular from the line: each lane
                                             border point of the junction's connecting roads projects onto [0, length]
  junctions.elevation_grid.only_one_elev_grid  at most one grid
  coverage                                   every such point lies inside the grid's complete squares
  "the grid shall override any elevation values derived from any roads that are part of the junction": the grid
  height (bilinear in each square; exact for bicubic §12.11.1 when the grid values are planar - checked) is compared
  with the connecting roads' own surface height (elevation + t tan superelevation) at their lane centres.
  Also reported: the same comparison along the boundary line of §12.10 helper roads (JunctionBoundary).
usage: python grid_audit.py file.xodr [...]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402


def superelevation(road):
    lp = road.el.find("lateralProfile")
    recs = sorted([(float(e.get("s")), G._abcd(e)) for e in lp.findall("superelevation")]) if lp is not None else []
    return lambda s: float(road._piecewise(recs, np.array([s]))[0]) if recs else 0.0


def audit(path):
    od = G.OpenDrive(path)
    root = ET.parse(path).getroot()
    fails, dz_all, planar_res, n = Counter(), [], [], 0
    dz_boundary = []  # |grid - boundary helper road surface| along the boundary
    for j in root.findall("junction"):
        pv, eg = j.findall("planView"), j.findall("elevationGrid")
        if not pv and not eg:
            continue
        n += 1
        if len(eg) > 1:
            fails["junctions.elevation_grid.only_one_elev_grid"] += 1
        geos = pv[0].findall("geometry") if pv else []
        if len(geos) != 1 or len(list(geos[0])) != 1 or geos[0][0].tag != "line":
            fails["junctions.geometry.only_one_line_element"] += 1
            continue
        g = geos[0]
        x0, y0, hdg, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
        d, nl = np.array([math.cos(hdg), math.sin(hdg)]), np.array([-math.sin(hdg), math.cos(hdg)])
        grid = eg[0]
        gs, s0 = float(grid.get("gridSpacing")), float(grid.get("sStart"))
        rows = grid.findall("elevation")
        Z = {}  # (k, j) -> z ; j > 0 left, j < 0 right
        for k, r in enumerate(rows):
            Z[(k, 0)] = float(r.get("center"))
            for jj, v in enumerate((r.get("left") or "").split(), 1):
                Z[(k, jj)] = float(v)
            for jj, v in enumerate((r.get("right") or "").split(), 1):
                Z[(k, -jj)] = float(v)
        # planarity of the grid values: then the bicubic interpolation of §12.11.1 equals the bilinear one
        A = np.array([[1.0, s0 + k * gs, jj * gs] for (k, jj) in Z])
        zz = np.array([Z[key] for key in Z])
        coef = np.linalg.lstsq(A, zz, rcond=None)[0]
        planar_res.append(float(np.abs(A @ coef - zz).max()))

        def zgrid(s, t):
            k = int(math.floor((s - s0) / gs)); jj = int(math.floor(t / gs))
            u, v = (s - s0) / gs - k, t / gs - jj
            c = [Z.get(q) for q in ((k, jj), (k + 1, jj), (k, jj + 1), (k + 1, jj + 1))]
            if any(x is None for x in c):
                return None
            return c[0] * (1 - u) * (1 - v) + c[1] * u * (1 - v) + c[2] * (1 - u) * v + c[3] * u * v

        for r in od.roads.values():
            if r.junction != j.get("id") or r.el.find("lanes/laneSection") is None:
                continue
            roll = superelevation(r)
            if r.el.get("name") == "JunctionBoundary":  # §12.10 helper: its centre lane line is the boundary
                for s in np.linspace(0.0, r.length, max(2, int(r.length) + 1)):
                    lo = r.lane_offset(np.array([s]))
                    t = float(lo[0]) if hasattr(lo, "__len__") else float(lo)
                    x, y = r.point(np.array([s]), t)
                    p = np.array([float(x[0]), float(y[0])]) - np.array([x0, y0])
                    zg = zgrid(float(p @ d), float(p @ nl))
                    if zg is not None:
                        dz_boundary.append(abs(zg - (float(r.z(np.array([s]))[0]) + t * math.tan(roll(s)))))
                continue
            for s in np.linspace(0.0, r.length, max(2, int(r.length) + 1)):
                sec = r.section_at(s)
                for lid, (tin, tout) in r.borders_at(s, sec).items():
                    if lid == 0:
                        continue
                    for t, centre in ((tout, False), (0.5 * (tin + tout), True)):
                        x, y = r.point(np.array([s]), t)
                        p = np.array([float(x[0]), float(y[0])]) - np.array([x0, y0])
                        ps, pt = float(p @ d), float(p @ nl)
                        if not (-1e-6 <= ps <= L + 1e-6):
                            fails["junctions.geometry.ref_line_definition"] += 1
                        zg = zgrid(ps, pt)
                        if zg is None:
                            fails["coverage(point outside the grid squares)"] += 1
                        elif centre:
                            zr = float(r.z(np.array([s]))[0]) + t * math.tan(roll(s))
                            dz_all.append(abs(zg - zr))
    dz = np.array(dz_all) if dz_all else np.zeros(1)
    pr = np.array(planar_res) if planar_res else np.zeros(1)
    db = np.array(dz_boundary) if dz_boundary else np.zeros(1)
    print(f"{path}: {n} junctions with grid; fails {dict(fails)}; grid planarity residual max {pr.max():.2e} m; "
          f"|grid - connecting road| at lane centres: median {np.median(dz):.4f} p99 {np.percentile(dz, 99):.4f} max {dz.max():.4f} m; "
          f"|grid - boundary helper| on the boundary: median {np.median(db):.4f} p99 {np.percentile(db, 99):.4f} max {db.max():.4f} m")


for p in sys.argv[1:]:
    audit(p)
