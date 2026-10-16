"""Self-test of tools/odrforge_pipeline.py: config defaults and merging, source choice by coverage, download plans.
No network. usage: python pipeline_selftest.py   (exit 0 = pass)"""
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
check(pick("ground", (46.515, 10.440, 46.535, 10.465)) == "copernicus_glo30", "Stelvio ground: GLO-30")
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

print("pipeline self-test:", "FAILED" if failures else "ok")
sys.exit(1 if failures else 0)
