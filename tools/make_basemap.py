"""Offline HD reference backgrounds for the OdrForge viewer, in the network's own frame.

  python make_basemap.py --origin LAT,LON --bounds MINLAT,MINLON,MAXLAT,MAXLON --out DIR
      [--osm area.osm] [--naip] [--res 0.3] [--margin 60]

* map:  a carto-style map rendered from the OSM file itself (landuse, water, parks, buildings,
        roads by class, paths, rail) - no tiles are downloaded, nothing from the tile servers.
* naip: USGS NAIP orthoimagery (public domain, 0.6 m) from The National Map image service, fetched
        in its native Web Mercator and resampled into the local transverse Mercator, so it lines up
        with the roads exactly.

Both are written as tile pyramids (512 px tiles, each level half the resolution of the one below)
plus DIR/basemap.json. Pass DIR to `osm2xodr --basemap DIR` / `xodrviz --basemap DIR`; the viewer
then offers them as backgrounds and loads only the tiles in view.
Requires numpy and Pillow; --naip needs network access to imagery.nationalmap.gov.
"""
import argparse
import io
import json
import math
import os
import sys
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

Image.MAX_IMAGE_PIXELS = None
A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)
EP2 = E2 / (1 - E2)


def meridian(phi):
    e4, e6 = E2 * E2, E2 ** 3
    return A * ((1 - E2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi - (3 * E2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * np.sin(2 * phi)
                + (15 * e4 / 256 + 45 * e6 / 1024) * np.sin(4 * phi) - (35 * e6 / 3072) * np.sin(6 * phi))


def tm_forward(lat, lon, lat0, lon0):
    """Transverse Mercator (k = 1), as OdrForge's Projection / the header geoReference."""
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


def tm_wkt(lat0, lon0):
    return ('PROJCS["OdrForge local",GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,298.257223563]],'
            'PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]],PROJECTION["Transverse_Mercator"],'
            f'PARAMETER["False_Easting",0.0],PARAMETER["False_Northing",0.0],PARAMETER["Central_Meridian",{lon0!r}],'
            f'PARAMETER["Scale_Factor",1.0],PARAMETER["Latitude_Of_Origin",{lat0!r}],UNIT["Meter",1.0]]')


# ------------------------------------------------------------------------------- tile pyramid
def write_pyramid(img, out_dir, layer, ext, quality=88):
    """img: full-resolution PIL image. Level 0 = full resolution; tiles of 512 px."""
    T = 512
    levels = []
    level, cur = 0, img
    while True:
        w, h = cur.size
        nx, ny = math.ceil(w / T), math.ceil(h / T)
        d = os.path.join(out_dir, layer, str(level))
        os.makedirs(d, exist_ok=True)
        for j in range(ny):
            for i in range(nx):
                t = cur.crop((i * T, j * T, min(w, (i + 1) * T), min(h, (j + 1) * T)))
                path = os.path.join(d, f"{i}_{j}.{ext}")
                if ext == "jpg":
                    t.convert("RGB").save(path, quality=quality, optimize=True)
                else:
                    t.save(path, optimize=True)
        levels.append({"nx": nx, "ny": ny, "w": w, "h": h})
        if nx <= 1 and ny <= 1:
            break
        cur = cur.resize((max(1, w // 2), max(1, h // 2)), Image.LANCZOS)
        level += 1
    return levels


# ------------------------------------------------------------------------------- NAIP
def tm_inverse(x, y, lat0, lon0):
    """Transverse Mercator inverse (Snyder 8-12 .. 8-18), k = 1."""
    d = math.pi / 180
    e1 = (1 - math.sqrt(1 - E2)) / (1 + math.sqrt(1 - E2))
    M = meridian(lat0 * d) + np.asarray(y, float)
    mu = M / (A * (1 - E2 / 4 - 3 * E2 ** 2 / 64 - 5 * E2 ** 3 / 256))
    phi1 = (mu + (3 * e1 / 2 - 27 * e1 ** 3 / 32) * np.sin(2 * mu) + (21 * e1 ** 2 / 16 - 55 * e1 ** 4 / 32) * np.sin(4 * mu)
            + (151 * e1 ** 3 / 96) * np.sin(6 * mu) + (1097 * e1 ** 4 / 512) * np.sin(8 * mu))
    s1, c1, t1 = np.sin(phi1), np.cos(phi1), np.tan(phi1)
    C1, T1 = EP2 * c1 * c1, t1 * t1
    N1 = A / np.sqrt(1 - E2 * s1 * s1)
    R1 = A * (1 - E2) / (1 - E2 * s1 * s1) ** 1.5
    D = np.asarray(x, float) / N1
    phi = phi1 - (N1 * t1 / R1) * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * EP2) * D ** 4 / 24
                                   + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * EP2 - 3 * C1 * C1) * D ** 6 / 720)
    lam = (D - (1 + 2 * T1 + C1) * D ** 3 / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * EP2 + 24 * T1 * T1) * D ** 5 / 120) / c1
    return phi / d, lon0 + lam / d


def merc(lat, lon):
    return A * np.radians(lon), A * np.log(np.tan(np.pi / 4 + np.radians(lat) / 2))


def fetch_naip(x0, y0, x1, y1, res, lat0, lon0, service, workers=4):
    """Fetch the image service in its native Web Mercator (no server-side reprojection: the service
    times out on that), then resample into the local transverse Mercator frame (bilinear)."""
    from concurrent.futures import ThreadPoolExecutor
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    # source mosaic covering the output's corners and edges (+ margin)
    ex = np.linspace(x0, x1, 9)
    ey = np.linspace(y0, y1, 9)
    gx, gy = np.meshgrid(ex, ey)
    la, lo = tm_inverse(gx.ravel(), gy.ravel(), lat0, lon0)
    mx, my = merc(la, lo)
    k = 1.0 / math.cos(math.radians(lat0))  # Web Mercator units per ground metre
    pix = res * k
    sx0, sx1 = mx.min() - 20 * pix, mx.max() + 20 * pix
    sy0, sy1 = my.min() - 20 * pix, my.max() + 20 * pix
    SW, SH = int(math.ceil((sx1 - sx0) / pix)), int(math.ceil((sy1 - sy0) / pix))
    src = np.zeros((SH, SW, 3), np.uint8)
    CH = 1000
    jobs = [(px, py, min(CH, SW - px), min(CH, SH - py)) for py in range(0, SH, CH) for px in range(0, SW, CH)]

    def fetch(url):
        rq = urllib.request.Request(url, headers={"User-Agent": "OdrForge-make_basemap"})
        with urllib.request.urlopen(rq, timeout=240) as r:
            return r.read()

    def xyz_block(bx0, by0, bx1, by1, w, h):
        """mosaic of Web Mercator XYZ tiles at the zoom just finer than the block's pixel size"""
        R = 20037508.342789244
        z = max(0, min(20, math.ceil(math.log2(2 * R / 256 / pix))))
        ts = 2 * R / 2 ** z
        tx0, tx1 = int((bx0 + R) // ts), int((bx1 + R) // ts)
        ty0, ty1 = int((R - by1) // ts), int((R - by0) // ts)
        mos = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
        n = 0
        for tx in range(tx0, tx1 + 1):
            for ty in range(ty0, ty1 + 1):
                try:
                    data = fetch(service.format(z=z, x=tx, y=ty))
                    mos.paste(Image.open(io.BytesIO(data)).convert("RGB"), ((tx - tx0) * 256, (ty - ty0) * 256))
                    n += len(data)
                except urllib.error.HTTPError as e:
                    if e.code != 404:  # 404: no imagery there (left black)
                        raise
        s = 256 / ts
        box = ((bx0 + R - tx0 * ts) * s, (R - by1 - ty0 * ts) * s, (bx1 + R - tx0 * ts) * s, (R - by0 - ty0 * ts) * s)
        return mos.resize((w, h), Image.BILINEAR, box=box), n

    def get(job):
        px, py, w, h = job
        bx0, bx1 = sx0 + px * pix, sx0 + (px + w) * pix
        by1, by0 = sy1 - py * pix, sy1 - (py + h) * pix
        if "{bbox}" in service:  # OGC WMS GetMap template in EPSG:3857
            url = service.format(bbox=f"{bx0},{by0},{bx1},{by1}", w=w, h=h)
        elif "{z}" not in service:  # ArcGIS ImageServer (NAIP)
            q = {"bbox": f"{bx0},{by0},{bx1},{by1}", "bboxSR": "3857", "imageSR": "3857", "size": f"{w},{h}",
                 "format": "jpg", "compressionQuality": "92", "interpolation": "RSP_BilinearInterpolation", "f": "image"}
            url = service + "/exportImage?" + urllib.parse.urlencode(q)
        for attempt in range(8):
            try:
                if "{z}" in service:
                    img, nb = xyz_block(bx0, by0, bx1, by1, w, h)
                    return job, np.asarray(img), nb
                data = fetch(url)
                img = Image.open(io.BytesIO(data)).convert("RGB")
                if img.size != (w, h):
                    img = img.resize((w, h), Image.LANCZOS)
                return job, np.asarray(img), len(data)
            except Exception as e:  # noqa: BLE001 - retried with backoff
                last = e
                time.sleep(5 * (attempt + 1))
        raise SystemExit(f"NAIP export failed after retries: {last}")

    done = 0
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for (px, py, w, h), arr, nbytes in pool.map(get, jobs):
            src[py:py + h, px:px + w] = arr
            done += 1
            print(f"  naip chunk {done}/{len(jobs)} ({w} x {h} px, {nbytes / 1e6:.1f} MB)", flush=True)
    # resample into the local frame, row blocks
    out = np.zeros((H, W, 3), np.uint8)
    xs = x0 + (np.arange(W) + 0.5) * res
    B = 256
    for r0 in range(0, H, B):
        r1 = min(H, r0 + B)
        ys = y1 - (np.arange(r0, r1) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        la, lo = tm_inverse(X, Y, lat0, lon0)
        mx, my = merc(la, lo)
        u = (mx - sx0) / pix - 0.5
        v = (sy1 - my) / pix - 0.5
        u0 = np.clip(np.floor(u).astype(int), 0, SW - 2)
        v0 = np.clip(np.floor(v).astype(int), 0, SH - 2)
        fu = np.clip(u - u0, 0, 1)[..., None]
        fv = np.clip(v - v0, 0, 1)[..., None]
        a00 = src[v0, u0].astype(np.float32); a01 = src[v0, u0 + 1].astype(np.float32)
        a10 = src[v0 + 1, u0].astype(np.float32); a11 = src[v0 + 1, u0 + 1].astype(np.float32)
        out[r0:r1] = np.clip((a00 * (1 - fu) + a01 * fu) * (1 - fv) + (a10 * (1 - fu) + a11 * fu) * fv + 0.5, 0, 255).astype(np.uint8)
    return Image.fromarray(out)


# ------------------------------------------------------------------------------- OSM map
BG = (242, 239, 233)
AREA_STYLE = [  # (tag test, fill, outline) in drawing order
    (lambda t: t.get("landuse") in ("residential",), (224, 223, 223), None),
    (lambda t: t.get("landuse") in ("commercial", "retail") or t.get("amenity") in ("marketplace",), (242, 218, 217), None),
    (lambda t: t.get("landuse") in ("industrial", "railway"), (235, 219, 232), None),
    (lambda t: t.get("landuse") in ("construction", "brownfield"), (199, 199, 180), None),
    (lambda t: t.get("amenity") in ("school", "university", "college", "hospital"), (255, 255, 229), None),
    (lambda t: t.get("leisure") in ("park", "garden", "playground", "pitch", "sports_centre")
     or t.get("landuse") in ("grass", "recreation_ground", "village_green"), (200, 250, 204), (170, 220, 175)),
    (lambda t: t.get("natural") in ("wood", "scrub") or t.get("landuse") == "forest", (173, 209, 158), None),
    (lambda t: t.get("natural") == "water" or t.get("waterway") == "riverbank" or t.get("landuse") == "reservoir", (170, 211, 223), None),
    (lambda t: t.get("amenity") == "parking" and t.get("parking") not in ("underground", "multi-storey"), (238, 238, 238), (220, 220, 220)),
    (lambda t: t.get("highway") in ("pedestrian", "footway") and t.get("area") == "yes", (221, 221, 232), (190, 190, 205)),
    (lambda t: "building" in t and t.get("building") != "no", (217, 208, 201), (196, 182, 171)),
]
ROAD_STYLE = {  # class: (width m, fill, casing)
    "motorway": (11, (232, 146, 162), (196, 96, 112)), "motorway_link": (6, (232, 146, 162), (196, 96, 112)),
    "trunk": (10, (249, 178, 156), (200, 120, 100)), "trunk_link": (6, (249, 178, 156), (200, 120, 100)),
    "primary": (10, (252, 214, 164), (190, 150, 90)), "primary_link": (6, (252, 214, 164), (190, 150, 90)),
    "secondary": (9, (247, 250, 191), (170, 170, 110)), "secondary_link": (6, (247, 250, 191), (170, 170, 110)),
    "tertiary": (8, (255, 255, 255), (180, 180, 180)), "tertiary_link": (5, (255, 255, 255), (180, 180, 180)),
    "residential": (7, (255, 255, 255), (185, 185, 185)), "unclassified": (7, (255, 255, 255), (185, 185, 185)),
    "living_street": (6, (237, 237, 237), (185, 185, 185)), "road": (6, (221, 221, 221), (185, 185, 185)),
    "busway": (6, (180, 200, 240), (120, 140, 190)), "service": (4, (255, 255, 255), (190, 190, 190)),
    "pedestrian": (5, (221, 221, 232), (170, 170, 185)), "track": (3, (172, 132, 52), None),
}
PATHS = {"footway": (1.6, (250, 128, 114)), "path": (1.4, (250, 128, 114)), "steps": (2.0, (250, 128, 114)),
         "cycleway": (1.6, (60, 60, 255)), "bridleway": (1.4, (0, 128, 0)), "corridor": (1.2, (250, 128, 114))}
ROAD_ORDER = ["service", "track", "living_street", "road", "unclassified", "residential", "busway", "pedestrian",
              "tertiary_link", "tertiary", "secondary_link", "secondary", "primary_link", "primary",
              "trunk_link", "trunk", "motorway_link", "motorway"]


def load_osm(path, lat0, lon0):
    root = ET.parse(path).getroot()
    ids, lats, lons = [], [], []
    for n in root.iter("node"):
        ids.append(n.get("id"))
        lats.append(float(n.get("lat")))
        lons.append(float(n.get("lon")))
    x, y = tm_forward(np.array(lats), np.array(lons), lat0, lon0)
    pos = {i: (a, b) for i, a, b in zip(ids, x, y)}
    ways = {}
    for w in root.iter("way"):
        nd = [n.get("ref") for n in w.findall("nd")]
        pts = [pos[r] for r in nd if r in pos]
        if len(pts) >= 2:
            ways[w.get("id")] = (np.array(pts), {t.get("k"): t.get("v") for t in w.findall("tag")}, nd[0] == nd[-1])
    rels = []
    for r in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in r.findall("tag")}
        if tags.get("type") != "multipolygon":
            continue
        outers = [m.get("ref") for m in r.findall("member") if m.get("type") == "way" and m.get("role") in ("outer", "")]
        inners = [m.get("ref") for m in r.findall("member") if m.get("type") == "way" and m.get("role") == "inner"]
        rels.append((tags, outers, inners))
    return ways, rels


def render_map(osm, lat0, lon0, x0, y0, x1, y1, res):
    ways, rels = load_osm(osm, lat0, lon0)
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    SS = 2  # supersampling (anti-aliasing)
    TILE = 2048
    out = Image.new("RGB", (W, H), BG)
    # features with bounding boxes (local metres)
    areas, roads, paths, rails = [], [], [], []
    for wid, (pts, tags, closed) in ways.items():
        bb = (pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max())
        for order, (test, fill, line) in enumerate(AREA_STYLE):
            if closed and test(tags):
                areas.append((order, [pts], [], fill, line, bb))
                break
        hw = tags.get("highway")
        if hw in ROAD_STYLE and tags.get("area") != "yes":
            roads.append((ROAD_ORDER.index(hw) if hw in ROAD_ORDER else 0, pts, hw, bb, tags))
        elif hw in PATHS and tags.get("area") != "yes":
            paths.append((pts, hw, bb))
        if tags.get("railway") in ("rail", "light_rail", "subway", "tram") and tags.get("tunnel") != "yes":
            rails.append((pts, bb))
    for tags, outers, inners in rels:
        for order, (test, fill, line) in enumerate(AREA_STYLE):
            if test(tags):
                po = [ways[o][0] for o in outers if o in ways]
                pi = [ways[i][0] for i in inners if i in ways]
                if po:
                    allp = np.vstack(po)
                    areas.append((order, po, pi, fill, line,
                                  (allp[:, 0].min(), allp[:, 1].min(), allp[:, 0].max(), allp[:, 1].max())))
                break
    areas.sort(key=lambda a: a[0])
    roads.sort(key=lambda r: (r[4].get("layer", "0") if r[4].get("layer", "0").lstrip("-").isdigit() else "0", r[0]))
    nt = math.ceil(W / TILE) * math.ceil(H / TILE)
    k = 0
    for ty in range(0, H, TILE):
        for tx in range(0, W, TILE):
            k += 1
            w, h = min(TILE, W - tx), min(TILE, H - ty)
            # this tile in local metres (with a margin for wide strokes)
            mx0, mx1 = x0 + tx * res, x0 + (tx + w) * res
            my1, my0 = y1 - ty * res, y1 - (ty + h) * res
            pad = 20
            img = Image.new("RGB", (w * SS, h * SS), BG)
            dr = ImageDraw.Draw(img)
            s = SS / res

            def P(pts):
                return [((px - mx0) * s, (my1 - py) * s) for px, py in pts]

            def hit(bb):
                return not (bb[2] < mx0 - pad or bb[0] > mx1 + pad or bb[3] < my0 - pad or bb[1] > my1 + pad)

            for order, po, pi, fill, line, bb in areas:
                if not hit(bb):
                    continue
                for poly in po:
                    if len(poly) >= 3:
                        dr.polygon(P(poly), fill=fill, outline=line)
                for poly in pi:
                    if len(poly) >= 3:
                        dr.polygon(P(poly), fill=BG)
            for pts, bb in rails:
                if hit(bb):
                    dr.line(P(pts), fill=(112, 112, 112), width=max(1, int(1.6 * s)))
            for pts, hw, bb in paths:
                if hit(bb):
                    wd, col = PATHS[hw]
                    dr.line(P(pts), fill=col, width=max(1, int(wd * s)))
            # casings first, then fills (junctions merge cleanly)
            vis = [r for r in roads if hit(r[3])]
            for _, pts, hw, bb, tags in vis:
                wd, fill, casing = ROAD_STYLE[hw]
                if casing:
                    lw = max(1, int((wd + 1.2) * s))
                    dr.line(P(pts), fill=casing, width=lw, joint="curve")
            for _, pts, hw, bb, tags in vis:
                wd, fill, casing = ROAD_STYLE[hw]
                lw = max(1, int(wd * s))
                dr.line(P(pts), fill=fill, width=lw, joint="curve")
                r = lw / 2
                for px, py in P(pts[[0, -1]]):
                    dr.ellipse((px - r, py - r, px + r, py + r), fill=fill)
            out.paste(img.resize((w, h), Image.LANCZOS), (tx, ty))
            print(f"  map tile {k}/{nt}", flush=True)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--origin", required=True, help="lat,lon of the OdrForge projection (osm2xodr --origin)")
    ap.add_argument("--bounds", required=True, help="minlat,minlon,maxlat,maxlon")
    ap.add_argument("--out", required=True)
    ap.add_argument("--osm", help="OSM XML file: render the map background")
    ap.add_argument("--naip", action="store_true", help="download USGS NAIP Plus orthoimagery (public domain)")
    ap.add_argument("--res", type=float, default=0.3, help="metres per pixel at full resolution")
    ap.add_argument("--margin", type=float, default=60.0)
    ap.add_argument("--naip-service", default="https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer",
                    help="image service (USGSNAIPPlus has up to 0.3 m where HRO exists but is often overloaded)")
    ap.add_argument("--sat-name", help="name of a non-NAIP imagery source given as --naip-service: an ArcGIS ImageServer, "
                    "a WMS GetMap URL template in EPSG:3857 with {bbox} {w} {h}, or an XYZ tile template with {z} {x} {y}")
    ap.add_argument("--sat-attribution", help="attribution / licence line shown in the viewer for --sat-name imagery")
    ap.add_argument("--naip-res", type=float, default=0.6, help="metres per pixel of the stored imagery (NAIP is 0.6 m)")
    ap.add_argument("--naip-dir", default="naip", help="folder of the imagery pyramid (written fresh, then listed)")
    a = ap.parse_args()
    lat0, lon0 = map(float, a.origin.split(","))
    b = list(map(float, a.bounds.split(",")))
    cx, cy = tm_forward(np.array([b[0], b[0], b[2], b[2]]), np.array([b[1], b[3], b[1], b[3]]), lat0, lon0)
    x0, x1 = float(cx.min()) - a.margin, float(cx.max()) + a.margin
    y0, y1 = float(cy.min()) - a.margin, float(cy.max()) + a.margin
    # snap the extent to whole pixels
    x1 = x0 + math.ceil((x1 - x0) / a.res) * a.res
    y0 = y1 - math.ceil((y1 - y0) / a.res) * a.res
    os.makedirs(a.out, exist_ok=True)
    meta_path = os.path.join(a.out, "basemap.json")
    meta = {"layers": []}
    if os.path.exists(meta_path):
        meta = json.load(open(meta_path))
    layers = {l["id"]: l for l in meta["layers"]}
    W, H = round((x1 - x0) / a.res), round((y1 - y0) / a.res)
    print(f"extent {x1 - x0:.0f} x {y1 - y0:.0f} m, {W} x {H} px at {a.res} m/px")
    if a.osm:
        print("rendering the OSM map ...")
        img = render_map(a.osm, lat0, lon0, x0, y0, x1, y1, a.res)
        levels = write_pyramid(img, a.out, "map", "png")
        layers["map"] = {"id": "map", "name": "HD map (offline, rendered from OSM)", "dir": "map", "ext": "png",
                         "x0": x0, "y1": y1, "res": a.res, "tile": 512, "levels": levels,
                         "attribution": "Map data © OpenStreetMap contributors (ODbL), rendered by OdrForge"}
    if a.naip:
        plus = "NAIPPlus" in a.naip_service
        src = a.sat_name or ("USGS NAIP Plus" if plus else "USGS NAIP")
        print(f"downloading {src} at {a.naip_res} m/px ...")
        nres = a.naip_res
        nx1 = x0 + math.ceil((x1 - x0) / nres) * nres
        ny0 = y1 - math.ceil((y1 - y0) / nres) * nres
        img = fetch_naip(x0, ny0, nx1, y1, nres, lat0, lon0, a.naip_service)
        levels = write_pyramid(img, a.out, a.naip_dir, "jpg")
        layers["naip"] = {"id": "naip", "name": f"HD satellite (offline, {src} {nres:g} m)", "dir": a.naip_dir, "ext": "jpg",
                          "x0": x0, "y1": y1, "res": nres, "tile": 512, "levels": levels,
                          "attribution": a.sat_attribution or f"Imagery: {src} (USDA / USGS), The National Map (public domain)"}
    # merge with what is on disk now (another run may have added a layer meanwhile)
    if os.path.exists(meta_path):
        for l in json.load(open(meta_path))["layers"]:
            if l["id"] not in (["map"] if a.osm else []) + (["naip"] if a.naip else []):
                layers[l["id"]] = l
    meta = {"origin": [lat0, lon0], "layers": list(layers.values())}
    json.dump(meta, open(meta_path, "w"), indent=1)
    total = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(a.out) for f in fs)
    print(f"wrote {meta_path} ({total / 1e6:.1f} MB in {a.out})")


if __name__ == "__main__":
    main()
