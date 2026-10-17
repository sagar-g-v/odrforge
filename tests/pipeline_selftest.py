"""Self-test of tools/odrforge_pipeline.py: config defaults and merging, source choice by coverage, download plans.
DEM fill seam. No network. usage: python pipeline_selftest.py   (exit 0 = pass)"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import odrforge_pipeline as P  # noqa: E402

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def write(text):
    f = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False, encoding="utf-8")
    f.write(text)
    f.close()
    return f.name


# 1. defaults fill what the config leaves out; [area_defaults] and the area override them
cfg = P.load_config(write("""
[paths]
out = "o"
[area_defaults]
profiles = ["standard"]
[areas.m]
bbox = [48.132, 11.560, 48.145, 11.585]
cell = 1.0
"""))
a = cfg["areas"]["m"]
check(cfg["download"]["confirm"] is True, "download.confirm defaults to true")
check(os.path.isabs(cfg["paths"]["out"]) and cfg["paths"]["out"].endswith("o"), "relative paths resolve against the config folder")
check(a["profiles"] == ["standard"], "[area_defaults] overrides the built-in profiles")
check(a["ground"] == "auto" and a["decks"] == "auto" and a["margin"] == 60.0, "missing area keys take the defaults")
check(abs(a["origin"][0] - 48.1385) < 1e-9 and abs(a["origin"][1] - 11.5725) < 1e-9, "origin defaults to the bbox centre")
check(a["cell"] == 1.0, "an area key overrides")

# 2. auto picks the finest provider covering the whole bbox; GLO-30 everywhere else
pick = lambda layer, bbox: (P.pick(layer, "auto", bbox) or {}).get("name")  # noqa: E731
check(pick("ground", (48.132, 11.560, 48.145, 11.585)) == "bavaria_dgm1", "Munich ground: Bavaria DGM1")
check(pick("ground", (50.045, 8.585, 50.060, 8.615)) == "hesse_dgm1", "Frankfurt ground: Hesse DGM1")
check(pick("decks", (50.045, 8.585, 50.060, 8.615)) == "hesse_dom1", "Frankfurt decks: Hesse DOM1")
check(pick("ground", (51.505, -0.135, 51.517, -0.115)) == "ea_dtm1", "London ground: EA DTM")
check(pick("ground", (35.652, 139.690, 35.664, 139.708)) == "gsi_dem5a", "Tokyo ground: GSI DEM5A")
check(pick("ground", (46.515, 10.440, 46.535, 10.465)) == "south_tyrol_dtm2_5", "Stelvio ground: South Tyrol DTM")
check((P.next_provider(P.PROVIDER["south_tyrol_dtm2_5"], (46.515, 10.440, 46.535, 10.465)) or {}).get("name")
      == "copernicus_glo30", "Stelvio: GLO-30 fills past the South Tyrol border")
check(P.next_provider(P.PROVIDER["copernicus_glo30"], (46.515, 10.440, 46.535, 10.465)) is None, "nothing after GLO-30")
check(pick("decks", (46.515, 10.440, 46.535, 10.465)) is None, "no open deck source at Stelvio")
check(P.pick("ground", "none", (0, 0, 1, 1)) is None, "none: no source")

# 3. download plans
glo = P.plan_downloads(P.PROVIDER["copernicus_glo30"], (46.515, 10.440, 46.535, 10.465), 60.0, "d")
check([os.path.basename(f) for _, f in glo] == ["Copernicus_DSM_COG_10_N46_00_E010_00_DEM.tif"], "GLO-30 tile name N46 E010")
gsi = P.plan_downloads(P.PROVIDER["gsi_dem5a"], (35.652, 139.690, 35.664, 139.708), 60.0, "d")
check(len(gsi) == 6 and all("/15/" in u for u, _ in gsi), "GSI: 6 zoom-15 tiles for the Tokyo area")
try:
    import pyproj  # noqa: F401
    bav = P.plan_downloads(P.PROVIDER["bavaria_dgm1"], (48.132, 11.560, 48.145, 11.585), 0.0, "d")
    names = sorted(os.path.basename(f) for _, f in bav)
    check({"690_5334.tif", "692_5335.tif"} <= set(names), "Bavaria: the 1 km tiles round Munich centre")
    wcs = P.plan_downloads(P.PROVIDER["hesse_dgm1"], (50.045, 8.585, 50.060, 8.615), 60.0, "d")
    check(len(wcs) == 1 and "COVERAGEID=he_dgm1" in wcs[0][0] and "SUBSET=E(" in wcs[0][0], "Hesse: one WCS GetCoverage")
except ImportError:
    print("pyproj missing: projected plans not checked")

# 4. a config without a bbox is refused
try:
    P.load_config(write("[areas.x]\norigin = [1, 2]\n"))
    check(False, "an area without bbox is refused")
except SystemExit:
    pass

# 5. a coarser DEM filling past a finer one's border is shifted onto it: no step at the border; far from it the
#    median difference where that is a consistent offset (another datum), else the fill's own heights
#    (make_terrain.shifted_fill)
try:
    import numpy as np
    import scipy  # noqa: F401
    import make_terrain as T
    x = np.arange(400.0)[None, :].repeat(50, axis=0)
    fine = np.where(x < 200, 100.0 + 0.1 * x + np.where(x > 150, 0.02 * (x - 150), 0.0), np.nan)  # 3.98 m apart at x 199
    coarse = 97.0 + 0.1 * x  # another datum: 3 m lower
    moved, onto = T.shifted_fill(fine, coarse, 100.0)
    filled = np.where(np.isnan(fine), moved, fine)
    check(onto, "a consistent offset moves the fill onto the finer source's datum")
    check(abs(filled[25, 200] - filled[25, 199] - 0.1) < 0.05, "no step where the fill takes over")
    check(abs(filled[25, 399] - (coarse[25, 399] + 3.0)) < 0.05, "far from the border: shifted by a consistent offset")
    check(np.all(np.abs(np.diff(filled[25])) < 0.2), "the faded shift keeps the slope continuous")
    # same datum, a coarse model scattered round the fine one (median +1, IQR ~2.7: no consistent offset - Stelvio:
    # median +1.26, IQR 2.34): far from the border the fill keeps its own heights, the border still has no step
    scatter = 1.0 + 2.0 * np.sin(0.03 * x) * np.cos(0.13 * np.arange(50.0)[:, None])  # smooth: no step of its own
    coarse2 = 100.0 + 0.1 * x - scatter
    moved2, onto2 = T.shifted_fill(fine, coarse2, 100.0)
    filled2 = np.where(np.isnan(fine), moved2, fine)
    check(not onto2, "same datum, scattered: the fill keeps its own datum")
    check(abs(filled2[25, 399] - coarse2[25, 399]) < 0.05, "same datum, scattered: far from the border the fill as it is")
    check(abs(filled2[25, 200] - filled2[25, 199] - 0.1) < 0.1, "same datum, scattered: no step at the border")
except ImportError:
    print("numpy / scipy / pyproj missing: the DEM fill not checked")

# 6. decks of streets on structures OSM does not call bridges (make_terrain.raised_smooth, on_top): a terrace's smooth
#    surface over the bare earth is a deck; a rough tree crown is not; a tunnel / covered / underground way gets none
try:
    import numpy as np
    import scipy  # noqa: F401
    import make_terrain as T
    g = np.full((40, 80), 6.0)
    dsm = g.copy()
    dsm[:, 10:30] = 13.5  # a terrace: 7.5 m over the bare earth, flat
    dsm[:, 50:70] = g[:, 50:70] + 8.0 + np.random.default_rng(1).normal(0.0, 1.5, (40, 20))  # a tree crown
    m = T.raised_smooth(dsm, g, 0.5)
    check(m[20, 15:25].all(), "a terrace's flat surface over the bare earth is a deck")
    check(not m[20, 52:68].any(), "a rough tree crown is not")
    check(not m[20, 35:45].any(), "the ground itself is not")
    check(T.on_top({"highway": "residential"}) and T.on_top({"highway": "primary", "layer": "1"}), "streets on top")
    check(not T.on_top({"highway": "service", "tunnel": "yes", "layer": "-1"}), "a tunnel is under something")
    check(not T.on_top({"highway": "steps", "covered": "yes"}) and not T.on_top({"highway": "service", "layer": "-1"}),
          "covered and underground ways are under something")
except ImportError:
    print("numpy / scipy / pyproj missing: street decks not checked")

# 7. coverage: the share of the bbox where a source's files have data (decides whether a coarser one is added)
try:
    import numpy as np
    import rasterio
    from rasterio.transform import from_origin
    box = (46.0, 10.0, 46.1, 10.1)
    tif = os.path.join(tempfile.mkdtemp(), "half.tif")
    a = np.full((100, 100), 1500.0, dtype="float32")
    a[:, 50:] = -9999.0  # the region ends half way east: no data beyond
    with rasterio.open(tif, "w", driver="GTiff", width=100, height=100, count=1, dtype="float32", crs="EPSG:4326",
                       transform=from_origin(10.0, 46.1, 0.001, 0.001), nodata=-9999.0) as d:
        d.write(a, 1)
    share = P.coverage([tif], box)
    check(0.4 < share < 0.6, f"coverage of a half-empty raster ~0.5 (got {share:.2f})")
    check(P.coverage([tif], (46.0, 10.0, 46.1, 10.04)) >= 0.995, "coverage 1 where the raster has data everywhere")
    check(P.coverage([tif], (46.0, 10.3, 46.1, 10.4)) == 0.0, "coverage 0 outside the raster")
except ImportError:
    print("numpy / rasterio / pyproj missing: coverage not checked")

print("pipeline self-test:", "FAILED" if failures else "ok")
sys.exit(1 if failures else 0)
