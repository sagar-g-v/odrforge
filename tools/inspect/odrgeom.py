"""Shared foundation for the OdrForge inspectors: exact OpenDRIVE evaluation, the OSM source in the
same projection, and evidence crops rendered over the HD basemap.

Everything is in the network's local transverse Mercator metres (the header geoReference), so an
OpenDRIVE road, its OSM way and the basemap pixels can be compared directly.

    import odrgeom as G
    od = G.OpenDrive("out.xodr")          # roads, lanes, links, junctions
    osm = G.Osm("area.osm", od.lat0, od.lon0)
    r = od.roads["42"]; x, y, h = r.ref(s); b = r.borders_at(s)   # {lane id: (t_inner, t_outer)}
    G.crop(od, osm, (cx, cy), 60, "out.png", basemap="build/.../LA_DTLA_basemap", marks=[...])
"""
import json
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np

DENSE = 0.2  # m, reference-line sampling (exact element evaluation, linear in between)


# ----------------------------------------------------------------------------- projection
A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)
EP2 = E2 / (1 - E2)


def meridian(phi):
    e4, e6 = E2 * E2, E2 ** 3
    return A * ((1 - E2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi - (3 * E2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * np.sin(2 * phi)
                + (15 * e4 / 256 + 45 * e6 / 1024) * np.sin(4 * phi) - (35 * e6 / 3072) * np.sin(6 * phi))


def tm_forward(lat, lon, lat0, lon0):
    """Transverse Mercator, k = 1 (OdrForge Projection / header geoReference)."""
    d = math.pi / 180
    phi = np.asarray(lat, float) * d
    sn, cs, tn = np.sin(phi), np.cos(phi), np.tan(phi)
    N = A / np.sqrt(1 - E2 * sn * sn)
    T, C = tn * tn, EP2 * cs * cs
    Aa = (np.asarray(lon, float) - lon0) * d * cs
    x = N * (Aa + (1 - T + C) * Aa ** 3 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * EP2) * Aa ** 5 / 120)
    y = (meridian(phi) - meridian(lat0 * d) + N * tn * (Aa * Aa / 2 + (5 - T + 9 * C + 4 * C * C) * Aa ** 4 / 24
                                                       + (61 - 58 * T + T * T + 600 * C - 330 * EP2) * Aa ** 6 / 720))
    return x, y


# ----------------------------------------------------------------------------- OpenDRIVE
def _poly(rec, ds):
    return rec[0] + rec[1] * ds + rec[2] * ds * ds + rec[3] * ds * ds * ds


def _abcd(e):
    return tuple(float(e.get(k, 0)) for k in ("a", "b", "c", "d"))


def _eval_geometry(g, ds):
    x0, y0, h0, L = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
    kind = g[0]
    tag = kind.tag
    if tag == "line":
        u, v = ds, np.zeros_like(ds)
    elif tag == "arc":
        k = float(kind.get("curvature"))
        if abs(k) < 1e-12:
            u, v = ds, np.zeros_like(ds)
        else:
            u, v = np.sin(k * ds) / k, (1 - np.cos(k * ds)) / k
    elif tag == "spiral":
        k0, k1 = float(kind.get("curvStart")), float(kind.get("curvEnd"))
        n = max(200, int(L / 0.02))
        t = np.linspace(0, L, n)
        th = k0 * t + 0.5 * (k1 - k0) / L * t * t
        cu = np.concatenate([[0], np.cumsum((np.cos(th[1:]) + np.cos(th[:-1])) * 0.5 * np.diff(t))])
        cv = np.concatenate([[0], np.cumsum((np.sin(th[1:]) + np.sin(th[:-1])) * 0.5 * np.diff(t))])
        u, v = np.interp(ds, t, cu), np.interp(ds, t, cv)
    elif tag == "paramPoly3":
        au, bu, cu_, du = (float(kind.get(k)) for k in ("aU", "bU", "cU", "dU"))
        av, bv, cv_, dv = (float(kind.get(k)) for k in ("aV", "bV", "cV", "dV"))
        pmax = 1.0 if kind.get("pRange", "normalized") == "normalized" else L
        p = np.linspace(0, pmax, max(400, int(L * 30)))
        uu, vv = au + bu * p + cu_ * p ** 2 + du * p ** 3, av + bv * p + cv_ * p ** 2 + dv * p ** 3
        cum = np.concatenate([[0], np.cumsum(np.hypot(np.diff(uu), np.diff(vv)))])
        if cum[-1] > 0:
            cum *= L / cum[-1]
        u, v = np.interp(ds, cum, uu), np.interp(ds, cum, vv)
    elif tag == "poly3":
        a, b, c, d = _abcd(kind)
        uu = np.linspace(0, L * 1.5 + 1, max(400, int(L * 30)))
        vv = a + b * uu + c * uu ** 2 + d * uu ** 3
        cum = np.concatenate([[0], np.cumsum(np.hypot(np.diff(uu), np.diff(vv)))])
        u, v = np.interp(ds, cum, uu), np.interp(ds, cum, vv)
    else:
        raise ValueError(f"unsupported geometry {tag}")
    c, s = math.cos(h0), math.sin(h0)
    return x0 + u * c - v * s, y0 + u * s + v * c


class Lane:
    def __init__(self, el):
        self.id = int(el.get("id"))
        self.type = el.get("type", "none")
        self.widths = sorted([(float(w.get("sOffset", 0)), _abcd(w)) for w in el.findall("width")]) or [(0.0, (0, 0, 0, 0))]
        lk = el.find("link")
        self.pred = int(lk.find("predecessor").get("id")) if lk is not None and lk.find("predecessor") is not None else None
        self.succ = int(lk.find("successor").get("id")) if lk is not None and lk.find("successor") is not None else None
        self.marks = [dict(m.attrib) for m in el.findall("roadMark")]

    def width(self, ds):
        ds = np.asarray(ds, float)
        out = np.zeros_like(ds)
        for i, (s0, rec) in enumerate(self.widths):
            s1 = self.widths[i + 1][0] if i + 1 < len(self.widths) else 1e18
            m = (ds >= s0 - 1e-9) & (ds < s1) if i else (ds < s1)
            out[m] = _poly(rec, ds[m] - s0)
        return out


class Section:
    def __init__(self, el):
        self.s = float(el.get("s"))
        self.e = None
        self.left, self.right = [], []
        for side, store in (("left", self.left), ("right", self.right)):
            se = el.find(side)
            if se is not None:
                store.extend(Lane(l) for l in se.findall("lane"))
                store.sort(key=lambda l: abs(l.id))

    def lanes(self):
        return self.left + self.right

    def lane(self, lid):
        for l in self.lanes():
            if l.id == lid:
                return l
        return None


class Link:
    def __init__(self, el):
        self.type = el.get("elementType")
        self.id = el.get("elementId")
        self.contact = el.get("contactPoint")


class Road:
    def __init__(self, el):
        self.el = el
        self.id = el.get("id")
        self.name = el.get("name", "")
        self.length = float(el.get("length"))
        self.junction = el.get("junction", "-1")
        lk = el.find("link")
        self.pred = Link(lk.find("predecessor")) if lk is not None and lk.find("predecessor") is not None else None
        self.succ = Link(lk.find("successor")) if lk is not None and lk.find("successor") is not None else None
        ud = [u.get("value") for u in el.iter("userData") if u.get("code") == "osmWays"]
        self.osm_ways = ud[0].split(",") if ud else []
        S, X, Y = [], [], []
        self.geoms = []
        for g in el.find("planView").findall("geometry"):
            s0, L = float(g.get("s")), float(g.get("length"))
            self.geoms.append((s0, L, g[0].tag, dict(g[0].attrib)))
            if L <= 1e-9:
                continue
            ds = np.linspace(0, L, max(2, int(math.ceil(L / DENSE)) + 1))
            x, y = _eval_geometry(g, ds)
            S.append(s0 + ds), X.append(x), Y.append(y)
        self.S, self.X, self.Y = np.concatenate(S), np.concatenate(X), np.concatenate(Y)
        o = np.argsort(self.S, kind="stable")
        self.S, self.X, self.Y = self.S[o], self.X[o], self.Y[o]
        keep = np.concatenate([[True], np.diff(self.S) > 1e-6])
        self.S, self.X, self.Y = self.S[keep], self.X[keep], self.Y[keep]
        if len(self.S) > 2:
            dx, dy = np.gradient(self.X, self.S, edge_order=2), np.gradient(self.Y, self.S, edge_order=2)
        else:
            dx, dy = np.full(len(self.S), self.X[-1] - self.X[0]), np.full(len(self.S), self.Y[-1] - self.Y[0])
        self.H = np.unwrap(np.arctan2(dy, dx))
        lanes = el.find("lanes")
        self.offsets = sorted([(float(e.get("s")), _abcd(e)) for e in lanes.findall("laneOffset")])
        self.sections = sorted((Section(s) for s in lanes.findall("laneSection")), key=lambda s: s.s)
        for i, sec in enumerate(self.sections):
            sec.e = self.sections[i + 1].s if i + 1 < len(self.sections) else self.length
        ep = el.find("elevationProfile")
        self.elev = sorted([(float(e.get("s")), _abcd(e)) for e in ep.findall("elevation")]) if ep is not None else []

    # -- evaluation
    def ref(self, s):
        s = np.clip(np.asarray(s, float), 0, self.length)
        return np.interp(s, self.S, self.X), np.interp(s, self.S, self.Y), np.interp(s, self.S, self.H)

    @staticmethod
    def _piecewise(records, s):
        s = np.asarray(s, float)
        out = np.zeros_like(s)
        for i, (s0, rec) in enumerate(records):
            s1 = records[i + 1][0] if i + 1 < len(records) else 1e18
            m = (s >= s0 - 1e-9) & (s < s1) if i else (s < s1)
            out[m] = _poly(rec, s[m] - s0)
        return out

    def lane_offset(self, s):
        return self._piecewise(self.offsets, s)

    def z(self, s):
        return self._piecewise(self.elev, s)

    def section_at(self, s):
        sec = self.sections[0]
        for x in self.sections:
            if x.s <= s + 1e-9:
                sec = x
        return sec

    def borders_at(self, s, sec=None):
        """{lane id: (t_inner, t_outer)} at a scalar s (inner = toward lane 0)."""
        sec = sec or self.section_at(s)
        ds = s - sec.s
        off = float(self.lane_offset(np.array([s]))[0])
        out = {0: (off, off)}
        t = off
        for l in sec.left:
            w = float(l.width(np.array([ds]))[0])
            out[l.id] = (t, t + w)
            t += w
        t = off
        for l in sec.right:
            w = float(l.width(np.array([ds]))[0])
            out[l.id] = (t, t - w)
            t -= w
        return out

    def point(self, s, t):
        x, y, h = self.ref(s)
        return x - t * np.sin(h), y + t * np.cos(h)

    def project(self, px, py):
        """nearest (s, t, distance) of a point (dense-sample search, refined)."""
        d2 = (self.X - px) ** 2 + (self.Y - py) ** 2
        i = int(np.argmin(d2))
        s = self.S[i]
        for _ in range(3):
            x, y, h = (float(v) for v in self.ref(s))
            ds = (px - x) * math.cos(h) + (py - y) * math.sin(h)
            s = float(np.clip(s + ds, 0, self.length))
        x, y, h = (float(v) for v in self.ref(s))
        t = -(px - x) * math.sin(h) + (py - y) * math.cos(h)
        return s, t, math.hypot(px - x, py - y)

    def outline(self, step=1.0, types=None):
        """carriageway outline polygon(s) per section: list of (xs, ys) closed rings."""
        rings = []
        for sec in self.sections:
            s = np.linspace(sec.s, sec.e, max(2, int((sec.e - sec.s) / step) + 1))
            lo, hi = [], []
            for sv in s:
                b = self.borders_at(float(sv), sec)
                vals = [v for lid, pair in b.items() if lid != 0 and (types is None or sec.lane(lid).type in types) for v in pair]
                if not vals:
                    vals = [b[0][0]]
                lo.append(min(vals)), hi.append(max(vals))
            x1, y1 = self.point(s, np.array(lo))
            x2, y2 = self.point(s, np.array(hi))
            rings.append((np.concatenate([x1, x2[::-1]]), np.concatenate([y1, y2[::-1]])))
        return rings


class OpenDrive:
    def __init__(self, path):
        root = ET.parse(str(path)).getroot()
        self.header = root.find("header")
        geo = self.header.findtext("geoReference") or ""
        m0, m1 = re.search(r"\+lat_0=([-0-9.eE]+)", geo), re.search(r"\+lon_0=([-0-9.eE]+)", geo)
        self.lat0 = float(m0.group(1)) if m0 else 0.0
        self.lon0 = float(m1.group(1)) if m1 else 0.0
        self.roads = {}
        for el in root.findall("road"):
            r = Road(el)
            self.roads[r.id] = r
        self.junctions = {}
        for j in root.findall("junction"):
            conns = []
            for c in j.findall("connection"):
                conns.append({"incoming": c.get("incomingRoad"), "connecting": c.get("connectingRoad"),
                              "contact": c.get("contactPoint"),
                              "lanes": [(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")]})
            self.junctions[j.get("id")] = conns
        self._grid = None

    def plain_roads(self):
        return [r for r in self.roads.values() if r.junction == "-1"]

    def connecting_roads(self):
        return [r for r in self.roads.values() if r.junction != "-1"]

    def grid(self, cell=25.0):
        """spatial index of reference-line samples: {(i, j): [(road id, sample index), ...]}"""
        if self._grid is None:
            g = {}
            for r in self.roads.values():
                for k in range(0, len(r.S), 5):
                    g.setdefault((int(r.X[k] // cell), int(r.Y[k] // cell)), set()).add(r.id)
            self._grid = (cell, g)
        return self._grid

    def roads_near(self, x, y, radius):
        cell, g = self.grid()
        out = set()
        n = int(math.ceil(radius / cell))
        ci, cj = int(x // cell), int(y // cell)
        for i in range(ci - n, ci + n + 1):
            for j in range(cj - n, cj + n + 1):
                out |= g.get((i, j), set())
        return [self.roads[r] for r in out]


# ----------------------------------------------------------------------------- OSM
class Osm:
    """OSM XML in the same local frame: nodes {id: (x, y)}, ways {id: (node ids, tags)}."""

    def __init__(self, path, lat0, lon0):
        root = ET.parse(path).getroot()
        ids, lats, lons, tags = [], [], [], {}
        for n in root.iter("node"):
            ids.append(n.get("id")), lats.append(float(n.get("lat"))), lons.append(float(n.get("lon")))
            t = {x.get("k"): x.get("v") for x in n.findall("tag")}
            if t:
                tags[n.get("id")] = t
        x, y = tm_forward(np.array(lats), np.array(lons), lat0, lon0)
        self.nodes = {i: (a, b) for i, a, b in zip(ids, x, y)}
        self.node_tags = tags
        self.ways = {}
        for w in root.iter("way"):
            self.ways[w.get("id")] = ([n.get("ref") for n in w.findall("nd")], {t.get("k"): t.get("v") for t in w.findall("tag")})
        self.relations = []
        for r in root.iter("relation"):
            self.relations.append(({t.get("k"): t.get("v") for t in r.findall("tag")},
                                   [(m.get("type"), m.get("ref"), m.get("role")) for m in r.findall("member")]))

    def polyline(self, way_id):
        nds, _ = self.ways[way_id]
        pts = [self.nodes[n] for n in nds if n in self.nodes]
        return np.array(pts) if pts else np.zeros((0, 2))


# ----------------------------------------------------------------------------- evidence crops
def crop(od, osm, center, half, path, basemap=None, marks=(), title=None, px=900, osm_ways=True):
    """Render a square crop (half-size in metres) around center: HD basemap (if given), OpenDRIVE
    lanes (translucent, by type), lane borders, reference lines (magenta), OSM way centre lines
    (dashed cyan) and marks [(x, y, label)]. Returns the path."""
    from PIL import Image, ImageDraw
    cx, cy = center
    x0, y0, x1, y1 = cx - half, cy - half, cx + half, cy + half
    sc = px / (2 * half)

    def P(xs, ys):
        return [((x - x0) * sc, (y1 - y) * sc) for x, y in zip(xs, ys)]

    img = Image.new("RGB", (px, px), (242, 239, 233))
    if basemap:
        meta = json.load(open(os.path.join(basemap, "basemap.json")))
        lay = next((l for l in meta["layers"] if l["id"] == "map"), meta["layers"][0])
        res, T = lay["res"], lay["tile"]
        lx0, ly1 = lay["x0"], lay["y1"]
        i0, i1 = int((x0 - lx0) / (T * res)), int((x1 - lx0) / (T * res))
        j0, j1 = int((ly1 - y1) / (T * res)), int((ly1 - y0) / (T * res))
        L0 = lay["levels"][0]
        for i in range(max(0, i0), min(L0["nx"] - 1, i1) + 1):
            for j in range(max(0, j0), min(L0["ny"] - 1, j1) + 1):
                f = os.path.join(basemap, lay["dir"], "0", f"{i}_{j}.{lay['ext']}")
                if not os.path.exists(f):
                    continue
                t = Image.open(f).convert("RGB")
                tx0, ty1 = lx0 + i * T * res, ly1 - j * T * res
                w, h = t.size
                sx, sy = (tx0 - x0) * sc, (y1 - ty1) * sc
                t = t.resize((max(1, int(round(w * res * sc))), max(1, int(round(h * res * sc)))))
                img.paste(t, (int(round(sx)), int(round(sy))))
    ov = Image.new("RGBA", (px, px), (0, 0, 0, 0))
    dr = ImageDraw.Draw(ov)
    colors = {"driving": (60, 70, 90, 150), "shoulder": (120, 100, 60, 130), "sidewalk": (150, 150, 150, 110),
              "biking": (40, 140, 90, 130), "parking": (60, 100, 150, 130)}
    near = od.roads_near(cx, cy, half * 1.5)
    for r in near:
        if not ((r.X >= x0 - 30) & (r.X <= x1 + 30) & (r.Y >= y0 - 30) & (r.Y <= y1 + 30)).any():
            continue
        for sec in r.sections:
            s = np.linspace(sec.s, sec.e, max(2, int((sec.e - sec.s) / 0.5) + 1))
            for l in sec.lanes():
                ti, to = [], []
                for sv in s:
                    b = r.borders_at(float(sv), sec)
                    ti.append(b[l.id][0]), to.append(b[l.id][1])
                xa, ya = r.point(s, np.array(ti))
                xb, yb = r.point(s, np.array(to))
                col = colors.get(l.type, (110, 80, 130, 120))
                if r.junction != "-1":
                    col = (col[0], col[1], col[2], 70)
                dr.polygon(P(np.concatenate([xa, xb[::-1]]), np.concatenate([ya, yb[::-1]])), fill=col)
                dr.line(P(xb, yb), fill=(255, 255, 255, 160), width=1)
        xr, yr, _ = r.ref(np.linspace(0, r.length, max(2, int(r.length / 0.5))))
        dr.line(P(xr, yr), fill=(230, 40, 200, 220) if r.junction == "-1" else (230, 40, 200, 90), width=2)
    if osm_ways and osm is not None:
        for wid, (nds, tags) in osm.ways.items():
            if "highway" not in tags:
                continue
            pts = [osm.nodes[n] for n in nds if n in osm.nodes]
            if len(pts) < 2:
                continue
            a = np.array(pts)
            if not ((a[:, 0] >= x0 - 50) & (a[:, 0] <= x1 + 50) & (a[:, 1] >= y0 - 50) & (a[:, 1] <= y1 + 50)).any():
                continue
            seg = P(a[:, 0], a[:, 1])
            for k in range(len(seg) - 1):  # dashed
                (ax, ay), (bx, by) = seg[k], seg[k + 1]
                L = math.hypot(bx - ax, by - ay)
                n = max(1, int(L / 8))
                for q in range(0, n, 2):
                    u0, u1 = q / n, min(1, (q + 1) / n)
                    dr.line([(ax + (bx - ax) * u0, ay + (by - ay) * u0), (ax + (bx - ax) * u1, ay + (by - ay) * u1)],
                            fill=(0, 200, 230, 255), width=2)
            for nd in nds:
                if nd in osm.nodes:
                    (qx, qy), = P([osm.nodes[nd][0]], [osm.nodes[nd][1]])
                    dr.ellipse((qx - 2, qy - 2, qx + 2, qy + 2), outline=(0, 160, 200, 255))
    for mx, my, label in marks:
        (qx, qy), = P([mx], [my])
        dr.ellipse((qx - 9, qy - 9, qx + 9, qy + 9), outline=(255, 0, 0, 255), width=3)
        if label:
            dr.text((qx + 11, qy - 6), str(label), fill=(255, 0, 0, 255))
    img = Image.alpha_composite(img.convert("RGBA"), ov).convert("RGB")
    d2 = ImageDraw.Draw(img)
    d2.rectangle((0, 0, px, 18), fill=(255, 255, 255))
    d2.text((4, 3), (title or "") + f"   [{2 * half:.0f} m]  magenta=ref line  cyan dashed=OSM way", fill=(0, 0, 0))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    img.save(path)
    return path
