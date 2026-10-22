"""OdrForge pipeline: fetch every input of an area from its source and build the OpenDRIVE outputs.

    python odrforge_pipeline.py CONFIG.toml [--area NAME] [--stages osm,ground,decks,terrain,convert] [--yes] [--plan]

Stages (in order; each is skipped when its output exists, unless the stage's `force` is set):
  osm      the area's OpenStreetMap data (Overpass API)                         -> <work>/<area>.osm
  ground   a bare-earth elevation model (DEM / DTM) from the best source here     -> <data>/dem/<area>_ground/*.tif
  decks    a surface model (DSM) for bridge decks, where one is open              -> <data>/dem/<area>_decks/*.tif
  terrain  OdrForge grids from them (make_terrain.py)                             -> <out>/<area>_ground.odrz, _decks.odrz
  convert  odrforge build per profile, plus the ground conformed to the roads     -> <out>/<area>[_standard].xodr, ...

The config is TOML (Python tomllib). Every key has a default (DEFAULTS below); a config file only states what differs:

    [paths]                 # where things go (relative paths: to the config file's folder)
    data = "data"
    out = "build/areas"
    [download]
    confirm = true          # list what will be downloaded and ask first (--yes: do not ask)
    [areas.munich]
    bbox = [48.132, 11.560, 48.145, 11.585]   # min lat, min lon, max lat, max lon (required)
    origin = [48.1385, 11.5725]               # default: the bbox centre
    country = "DE"                            # default: from the elevation source's country, else XX
    ground = "auto"                           # auto | <provider name> | path/to/dem.tif | none
    decks = "auto"                            # auto | <provider name> | path/to/dsm.tif | none
    profiles = ["standard", "carla"]
    cell = 0.5                                # grid cell (m); default: from the source's resolution

`[areas.<name>]` keys not given fall back to `[area_defaults]`, then to DEFAULTS. `auto` picks the finest provider
whose coverage holds the whole bbox (PROVIDERS, finest first; Copernicus GLO-30 covers the world), adding the next
coarser one where its data ends inside the bbox (a region's border). Each provider
states its CRS, resolution and vertical datum; the datum goes into the grids and, through odrforge build, into the
geoReference (PROJ +geoidgrids) and header userData.
"""
import argparse
import copy
import json
import math
import os
import subprocess
import sys
import tomllib
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

DEFAULTS = {
    "paths": {
        "data": "data",            # downloads (osm/, dem/<area>_ground, dem/<area>_decks)
        "out": "build/areas",      # grids and OpenDRIVE outputs
        "odrforge": os.path.join(REPO, "build", "Release", "odrforge.exe" if os.name == "nt" else "odrforge"),
        "python": sys.executable,  # runs make_terrain.py (needs numpy, pyproj, rasterio)
    },
    "download": {
        "confirm": True,
        "timeout": 600,
        "overpass": "https://overpass-api.de/api/interpreter",
    },
    "area_defaults": {
        "ground": "auto",
        "decks": "auto",
        "profiles": ["standard", "carla"],
        "margin": 60.0,           # m of terrain beyond the bbox (make_terrain.py --margin)
        "conformed": True,        # also write <area>_ground_conformed.odrz (odrforge build --ground-out)
        "build_args": [],         # extra odrforge build arguments
        "viz": False,             # also write the HTML viewer page
    },
    "stages": {s: {"force": False} for s in ("osm", "ground", "decks", "terrain", "convert")},
}

# ---- elevation sources. bbox = coverage (min lat, min lon, max lat, max lon); finest first. kind:
#   tiles1km: 1 km GeoTIFF tiles in a metric CRS, url with {e} {n} in km;  wcs: OGC WCS 2.0.1 GetCoverage of the bbox;
#   gsi: GSI text tiles (zoom 15);  glo30: Copernicus GLO-30 1 degree tiles (AWS open data)
PROVIDERS = [
    dict(name="bavaria_dgm1", layer="ground", country="DE", res=1.0, datum="DHHN2016", crs="EPSG:25832",
         bbox=(47.27, 8.97, 50.57, 13.84), kind="tiles1km",
         url="https://download1.bayernwolke.de/a/dgm/dgm1/{e}_{n}.tif",
         note="Bavarian surveying administration, DGM1, open data (CC BY 4.0)"),
    dict(name="hesse_dgm1", layer="ground", country="DE", res=1.0, datum="DHHN2016", crs="EPSG:25832",
         bbox=(49.39, 7.77, 51.66, 10.24), kind="wcs", url="https://inspire-hessen.de/raster/dgm1/ows",
         coverage="he_dgm1", axes=("E", "N"), note="HVBG Hesse, INSPIRE WCS DGM1, dl-zero-de/2.0"),
    dict(name="hesse_dom1", layer="decks", country="DE", res=1.0, datum="DHHN2016", crs="EPSG:25832",
         bbox=(49.39, 7.77, 51.66, 10.24), kind="wcs", url="https://inspire-hessen.de/raster/dom1/ows",
         coverage="dom1", axes=("E", "N"), note="HVBG Hesse, INSPIRE WCS DOM1 (surface model), dl-zero-de/2.0"),
    dict(name="south_tyrol_dtm2_5", layer="ground", country="IT", res=2.5, datum="Genova1942", crs="EPSG:25832",
         bbox=(46.21, 10.38, 47.09, 12.48), kind="wcs", url="https://geoservices9.civis.bz.it/geoserver/ows",
         coverage="p_bz-Elevation__DigitalTerrainModel-2.5m", axes=("E", "N"),
         note="Autonomous Province of Bolzano, DTM 2.5 m (heights above Genova 1942 mean sea level), CC0"),
    dict(name="ea_dtm1", layer="ground", country="GB", res=1.0, datum="ODN", crs="EPSG:27700",
         bbox=(49.9, -6.4, 55.8, 1.8), kind="wcs",
         url="https://environment.data.gov.uk/spatialdata/lidar-composite-digital-terrain-model-dtm-1m/wcs",
         coverage="13787b9a-26a4-4775-8523-806d13af58fc__Lidar_Composite_Elevation_DTM_1m", axes=("E", "N"),
         note="Environment Agency LIDAR Composite DTM 1 m, Open Government Licence"),
    dict(name="ea_dsm1", layer="decks", country="GB", res=1.0, datum="ODN", crs="EPSG:27700",
         bbox=(49.9, -6.4, 55.8, 1.8), kind="wcs",
         url="https://environment.data.gov.uk/spatialdata/lidar-composite-digital-surface-model-last-return-dsm-1m/wcs",
         coverage="9ba4d5ac-d596-445a-9056-dae3ddec0178__Lidar_Composite_Elevation_LZ_DSM_1m", axes=("E", "N"),
         note="Environment Agency LIDAR Composite DSM 1 m (last return), Open Government Licence"),
    dict(name="gsi_dem5a", layer="ground", country="JP", res=5.0, datum="JGD2011 (T.P.)", crs="EPSG:3857",
         bbox=(24.0, 122.0, 46.0, 154.0), kind="gsi", url="https://cyberjapandata.gsi.go.jp/xyz/dem5a/{z}/{x}/{y}.txt",
         note="GSI Japan, DEM5A (laser survey), GSI terms of use"),
    dict(name="copernicus_glo30", layer="ground", country=None, res=30.0, datum="EGM2008", crs="EPSG:4326",
         bbox=(-90.0, -180.0, 90.0, 180.0), kind="glo30",
         url="https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif",
         note="Copernicus DEM GLO-30 (a surface model: buildings and trees included), ESA licence"),
]
PROVIDER = {p["name"]: p for p in PROVIDERS}


# ---------------------------------------------------------------------------------------------------- config
def deep_merge(base, over):
    out = copy.deepcopy(base)
    for k, v in over.items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load_config(path):
    with open(path, "rb") as f:
        user = tomllib.load(f)
    # keys renamed with the command line (osm2xodr -> odrforge build): stop rather than ignore them
    old = ["paths.osm2xodr"] if "osm2xodr" in user.get("paths", {}) else []
    old += [f"{sec}.osm2xodr_args" for sec, v in [("area_defaults", user.get("area_defaults", {}))]
            + [(f"areas.{n}", a) for n, a in user.get("areas", {}).items()] if "osm2xodr_args" in v]
    if old:
        sys.exit(f"{path}: renamed keys {', '.join(old)} - use paths.odrforge and build_args")
    cfg = deep_merge(DEFAULTS, user)
    base = os.path.dirname(os.path.abspath(path))
    for k in ("data", "out"):
        if not os.path.isabs(cfg["paths"][k]):
            cfg["paths"][k] = os.path.normpath(os.path.join(base, cfg["paths"][k]))
    areas = {}
    for name, a in cfg.get("areas", {}).items():
        a = deep_merge(cfg["area_defaults"], a)
        if "bbox" not in a or len(a["bbox"]) != 4:
            sys.exit(f"area {name}: bbox = [min lat, min lon, max lat, max lon] is required")
        s, w, n, e = map(float, a["bbox"])
        if not (s < n and w < e):
            sys.exit(f"area {name}: bbox must be [min lat, min lon, max lat, max lon]")
        a["bbox"] = (s, w, n, e)
        a.setdefault("origin", [(s + n) / 2, (w + e) / 2])
        areas[name] = a
    if not areas:
        sys.exit("no [areas.<name>] in the config")
    cfg["areas"] = areas
    return cfg


# ---------------------------------------------------------------------------------------------------- sources
def covers(p, bbox):
    s, w, n, e = bbox
    return p["bbox"][0] <= s and p["bbox"][1] <= w and n <= p["bbox"][2] and e <= p["bbox"][3]


def pick(layer, choice, bbox):
    """the provider (dict), a local file (str) or None for `choice` (auto | provider name | path | none)"""
    if choice in (None, "none", False):
        return None
    if choice == "auto":
        for p in PROVIDERS:
            if p["layer"] == layer and covers(p, bbox):
                return p
        return None
    if choice in PROVIDER:
        p = PROVIDER[choice]
        if not covers(p, bbox):
            sys.exit(f"source {choice} does not cover the bbox {bbox}")
        return p
    if os.path.exists(choice):
        return choice
    sys.exit(f"unknown source {choice!r} (auto, none, a provider: {', '.join(PROVIDER)}, or an existing file)")


def next_provider(p, bbox):
    """the next coarser provider of p's layer covering the bbox (fills what p leaves empty), or None"""
    later = PROVIDERS[PROVIDERS.index(p) + 1:]
    return next((q for q in later if q["layer"] == p["layer"] and covers(q, bbox)), None)


def coverage(files, bbox, n=40):
    """share of the bbox where one of the files has data, on an n x n lattice of cell centres. A provider's bbox is
    the rectangle round its region (a state, a province); the data itself ends at the region's border"""
    import numpy as np
    import rasterio
    from pyproj import Transformer
    s, w, nn, e = bbox
    k = (np.arange(n) + 0.5) / n
    lat, lon = np.meshgrid(s + k * (nn - s), w + k * (e - w))
    have = np.zeros(lat.size, dtype=bool)
    for f in files:
        with rasterio.open(f) as src:
            x, y = Transformer.from_crs("EPSG:4326", src.crs, always_xy=True).transform(lon.ravel(), lat.ravel())
            v = np.array([val[0] for val in src.sample(zip(x, y))], dtype=float)
            ok = np.isfinite(v) if src.nodata is None or np.isnan(src.nodata) else (v != src.nodata) & np.isfinite(v)
            l, b, r, t = src.bounds
            have |= ok & (x >= l) & (x <= r) & (y >= b) & (y <= t)
    return float(have.mean())


def projected_box(p, bbox, pad=0.0):
    from pyproj import Transformer
    t = Transformer.from_crs("EPSG:4326", p["crs"], always_xy=True)
    s, w, n, e = bbox
    xs, ys = zip(*[t.transform(lo, la) for lo in (w, e) for la in (s, n)])
    return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad


def plan_downloads(p, bbox, margin, folder):
    """[(url, local path)] of one provider for the bbox (+ margin metres)"""
    pad_deg = margin / 111000.0
    s, w, n, e = bbox[0] - pad_deg, bbox[1] - pad_deg / math.cos(math.radians(bbox[0])), bbox[2] + pad_deg, \
        bbox[3] + pad_deg / math.cos(math.radians(bbox[2]))
    out = []
    if p["kind"] == "tiles1km":
        x0, y0, x1, y1 = projected_box(p, (s, w, n, e))
        for ek in range(int(x0 // 1000), int(x1 // 1000) + 1):
            for nk in range(int(y0 // 1000), int(y1 // 1000) + 1):
                out.append((p["url"].format(e=ek, n=nk), os.path.join(folder, f"{ek}_{nk}.tif")))
    elif p["kind"] == "wcs":
        x0, y0, x1, y1 = projected_box(p, (s, w, n, e))
        x0, y0, x1, y1 = math.floor(x0), math.floor(y0), math.ceil(x1), math.ceil(y1)
        ax, ay = p["axes"]
        q = urllib.parse.urlencode({"SERVICE": "WCS", "REQUEST": "GetCoverage", "VERSION": "2.0.1",
                                    "COVERAGEID": p["coverage"], "FORMAT": "image/tiff"})
        url = f"{p['url']}?{q}&SUBSET={ax}({x0},{x1})&SUBSET={ay}({y0},{y1})"
        out.append((url, os.path.join(folder, f"{p['name']}_{x0}_{y0}_{x1}_{y1}.tif")))
    elif p["kind"] == "gsi":
        z = 15
        nt = 2 ** z

        def tile(la, lo):
            return int((lo + 180.0) / 360.0 * nt), int((1.0 - math.asinh(math.tan(math.radians(la))) / math.pi) / 2.0 * nt)
        tx0, ty0 = tile(n, w)
        tx1, ty1 = tile(s, e)
        for ty in range(ty0, ty1 + 1):
            for tx in range(tx0, tx1 + 1):
                out.append((p["url"].format(z=z, x=tx, y=ty), os.path.join(folder, f"dem5a_{z}_{tx}_{ty}.txt")))
    elif p["kind"] == "glo30":
        for la in range(math.floor(s), math.floor(n) + 1):
            for lo in range(math.floor(w), math.floor(e) + 1):
                ns, ew = ("N" if la >= 0 else "S"), ("E" if lo >= 0 else "W")
                tile = f"Copernicus_DSM_COG_10_{ns}{abs(la):02d}_00_{ew}{abs(lo):03d}_00_DEM"
                out.append((p["url"].format(tile=tile), os.path.join(folder, tile + ".tif")))
    return out


def gsi_mosaic(folder):
    """GSI text tiles of a folder -> one GeoTIFF (EPSG:3857); returns its path"""
    import numpy as np
    import rasterio
    from rasterio.transform import from_origin
    tiles = []
    for f in os.listdir(folder):
        if f.startswith("dem5a_") and f.endswith(".txt"):
            _, z, x, y = f[:-4].split("_")
            tiles.append((int(z), int(x), int(y), os.path.join(folder, f)))
    z = tiles[0][0]
    xs, ys = [t[1] for t in tiles], [t[2] for t in tiles]
    x0, y0 = min(xs), min(ys)
    mos = np.full(((max(ys) - y0 + 1) * 256, (max(xs) - x0 + 1) * 256), np.nan, dtype="float32")
    for _, x, y, path in tiles:
        rows = open(path, encoding="ascii").read().strip().splitlines()
        a = np.array([[np.nan if v == "e" else float(v) for v in r.split(",")] for r in rows], dtype="float32")
        mos[(y - y0) * 256:(y - y0 + 1) * 256, (x - x0) * 256:(x - x0 + 1) * 256] = a
    R = 6378137.0
    size = 2 * math.pi * R / 2 ** z
    tif = os.path.join(folder, "dem5a_mosaic.tif")
    with rasterio.open(tif, "w", driver="GTiff", width=mos.shape[1], height=mos.shape[0], count=1, dtype="float32",
                       crs="EPSG:3857", transform=from_origin(x0 * size - math.pi * R, math.pi * R - y0 * size,
                                                               size / 256, size / 256), nodata=np.nan) as d:
        d.write(mos, 1)
    return tif


def fetch(items, cfg, what):
    """download the missing (url, path) items after asking (unless confirm is off or --yes)"""
    todo = [(u, p) for u, p in items if not os.path.exists(p)]
    if not todo:
        return True
    print(f"  {what}: {len(todo)} file(s) to download")
    for u, p in todo[:8]:
        print(f"    {u}\n      -> {p}")
    if len(todo) > 8:
        print(f"    ... {len(todo) - 8} more")
    if cfg["download"]["confirm"] and not cfg["_yes"]:
        if not sys.stdin.isatty() or input("  download? [y/N] ").strip().lower() != "y":
            print("  skipped (not confirmed)")
            return False
    for u, p in todo:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        tmp = p + ".part"
        req = urllib.request.Request(u, headers={"User-Agent": "OdrForge pipeline"})
        with urllib.request.urlopen(req, timeout=cfg["download"]["timeout"]) as r, open(tmp, "wb") as f:
            while chunk := r.read(1 << 20):
                f.write(chunk)
        os.replace(tmp, p)
        print(f"    got {os.path.basename(p)} ({os.path.getsize(p) / 1e6:.1f} MB)")
    return True


def raster_files(folder):
    return sorted(os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".tif")) if os.path.isdir(folder) else []


# ---------------------------------------------------------------------------------------------------- stages
def stage_osm(name, a, cfg):
    path = os.path.join(cfg["paths"]["data"], "osm", name + ".osm")
    if "osm" in a:  # a local OSM file given in the config
        return a["osm"]
    if os.path.exists(path) and not cfg["stages"]["osm"]["force"]:
        return path
    s, w, n, e = a["bbox"]
    q = f"[out:xml][timeout:300];(node({s},{w},{n},{e});way({s},{w},{n},{e});relation({s},{w},{n},{e}););(._;>;);out meta;"
    url = cfg["download"]["overpass"] + "?" + urllib.parse.urlencode({"data": q})
    if not fetch([(url, path)], cfg, "OpenStreetMap (Overpass API, ODbL)"):
        sys.exit("no OSM data")
    return path


def stage_source(name, a, cfg, layer):
    """local raster files of the ground / decks layer, finest first, the main source and each file's vertical datum
    (None, None, None when there is none). With auto, a source whose data ends inside the bbox (a regional model at its
    border) is completed by the next coarser one; make_terrain.py fills the gap with it"""
    p = pick(layer, a[layer], a["bbox"])
    if p is None:
        return None, None, None
    if isinstance(p, str):
        return [p], a.get("datum", ""), [a.get("datum", "")]
    files, datums, q = [], [], p
    while q is not None:
        folder = os.path.join(cfg["paths"]["data"], "dem", f"{name}_{layer}_{q['name']}")
        print(f"  {layer}: {q['name']} - {q['note']} ({q['res']:g} m, {q['datum']})")
        if not fetch(plan_downloads(q, a["bbox"], a["margin"], folder), cfg, f"{layer} ({q['name']})"):
            break
        got = [gsi_mosaic(folder)] if q["kind"] == "gsi" else raster_files(folder)
        files += got
        datums += [q["datum"]] * len(got)
        if a[layer] != "auto" or (share := coverage(files, a["bbox"])) >= 0.995:
            break
        q = next_provider(q, a["bbox"])
        print(f"  {layer}: data on {share * 100:.0f} % of the bbox; the rest from "
              f"{q['name'] if q else 'nowhere (no other source)'}")
    return (files, p, datums) if files else (None, None, None)


def stage_terrain(name, a, cfg, osm_path, ground, decks):
    out = cfg["paths"]["out"]
    os.makedirs(out, exist_ok=True)
    gfile, dfile = os.path.join(out, f"{name}_ground.odrz"), os.path.join(out, f"{name}_decks.odrz")
    if not ground[0]:
        return None, None
    if os.path.exists(gfile) and not cfg["stages"]["terrain"]["force"]:
        return gfile, dfile if os.path.exists(dfile) else None
    gp, dp = ground[1], decks[1]
    datum = a.get("datum") or (gp["datum"] if isinstance(gp, dict) else "")
    cell = a.get("cell") or (0.5 if isinstance(gp, dict) and gp["res"] <= 1.0 else 1.0)
    s, w, n, e = a["bbox"]
    cmd = [cfg["paths"]["python"], "-I", os.path.join(HERE, "make_terrain.py"), "--origin", f"{a['origin'][0]},{a['origin'][1]}",
           "--bounds", f"{s},{w},{n},{e}", "--ground", gfile, "--cell", str(cell), "--margin", str(a["margin"]), "--datum", datum]
    for f, dt in zip(ground[0], ground[2]):  # each file with its datum: the grid's label says which its heights are in
        cmd += ["--dem", f, "--dem-datum", dt]
    if decks[0]:
        for f in decks[0]:
            cmd += ["--dsm", f]
        cmd += ["--deck-osm", osm_path, "--structures", dfile]
    print("  terrain:", " ".join(os.path.basename(c) if os.sep in c else c for c in cmd[2:]))
    subprocess.run(cmd, check=True)
    return gfile, dfile if decks[0] and os.path.exists(dfile) else None


def stage_convert(name, a, cfg, osm_path, gfile, dfile, ground_source):
    out = cfg["paths"]["out"]
    country = a.get("country") or (ground_source.get("country") if isinstance(ground_source, dict) else None) or "XX"
    s, w, n, e = a["bbox"]
    for prof in a["profiles"]:
        xodr = os.path.join(out, f"{name}.xodr" if prof == "carla" else f"{name}_{prof}.xodr")
        if os.path.exists(xodr) and not cfg["stages"]["convert"]["force"]:
            continue
        cmd = [cfg["paths"]["odrforge"], "build", osm_path, xodr, "--target", prof, "--country", country, "--name", name,
               "--origin", f"{a['origin'][0]},{a['origin'][1]}", "--clip", f"{s},{w},{n},{e}"]
        if gfile:
            cmd += ["--terrain", gfile]
            if a["conformed"] and prof == "standard":
                cmd += ["--ground-out", os.path.join(out, f"{name}_ground_conformed.odrz")]
        if dfile:
            cmd += ["--structures", dfile]
        if a["viz"]:
            cmd += ["--viz", os.path.splitext(xodr)[0] + ".html"]
        cmd += list(a["build_args"])
        print(f"  convert ({prof}) -> {xodr}")
        r = subprocess.run(cmd, capture_output=True, text=True)
        tail = [l for l in r.stdout.splitlines() if "validation:" in l or "elevation vs" in l]
        for l in tail:
            print("   ", l.strip())
        if r.returncode not in (0, 1):  # 1: written, with validation errors
            sys.exit(f"odrforge build failed ({r.returncode}):\n{r.stderr[-2000:]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config")
    ap.add_argument("--area", action="append", help="only these areas (default: all in the config)")
    ap.add_argument("--stages", default="osm,ground,decks,terrain,convert")
    ap.add_argument("--yes", action="store_true", help="download without asking")
    ap.add_argument("--plan", action="store_true", help="print the resolved config and sources, do nothing")
    args = ap.parse_args()
    cfg = load_config(args.config)
    cfg["_yes"] = args.yes
    stages = set(args.stages.split(","))
    names = args.area or list(cfg["areas"])
    for name in names:
        if name not in cfg["areas"]:
            sys.exit(f"no area {name} in the config")
        a = cfg["areas"][name]
        g, d = pick("ground", a["ground"], a["bbox"]), pick("decks", a["decks"], a["bbox"])
        print(f"== {name}: bbox {a['bbox']}, ground {g['name'] if isinstance(g, dict) else g}, "
              f"decks {d['name'] if isinstance(d, dict) else d}")
        if args.plan:
            print(json.dumps({k: v for k, v in a.items()}, indent=1, default=str))
            for layer, p in (("ground", g), ("decks", d)):
                if isinstance(p, dict):
                    files = plan_downloads(p, a["bbox"], a["margin"], os.path.join(cfg["paths"]["data"], "dem", f"{name}_{layer}_{p['name']}"))
                    have = sum(os.path.exists(f) for _, f in files)
                    print(f"  {layer} {p['name']}: {len(files)} file(s), {have} already present")
            continue
        osm_path = stage_osm(name, a, cfg) if "osm" in stages else a.get("osm", os.path.join(cfg["paths"]["data"], "osm", name + ".osm"))
        ground = stage_source(name, a, cfg, "ground") if "ground" in stages or "terrain" in stages else (None, None, None)
        decks = stage_source(name, a, cfg, "decks") if "decks" in stages or "terrain" in stages else (None, None, None)
        gfile = dfile = None
        if "terrain" in stages:
            gfile, dfile = stage_terrain(name, a, cfg, osm_path, ground, decks)
        if "convert" in stages:
            stage_convert(name, a, cfg, osm_path, gfile, dfile, ground[1])


if __name__ == "__main__":
    main()
