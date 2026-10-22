# Area pipeline (`tools/odrforge_pipeline.py`)

One command fetches every input of an area from its source and builds the OpenDRIVE outputs:

```
python tools/odrforge_pipeline.py my_areas.toml                  # all areas, all stages
python tools/odrforge_pipeline.py my_areas.toml --area munich --plan   # what it would use and download
python tools/odrforge_pipeline.py my_areas.toml --stages terrain,convert --yes
```

It needs Python 3.11+ (tomllib) with numpy, pyproj and rasterio (for `make_terrain.py`), and a built `odrforge` command line.

## Stages

| stage | what | output |
|---|---|---|
| `osm` | OpenStreetMap data of the bbox (Overpass API), or a local file (`osm = "..."`) | `<data>/osm/<area>.osm` |
| `ground` | bare-earth elevation model from the best source covering the area | `<data>/dem/<area>_ground_<source>/` |
| `decks` | surface model for bridge decks, where an open one exists | `<data>/dem/<area>_decks_<source>/` |
| `terrain` | OdrForge grids (`make_terrain.py`; decks only near OSM road bridges) | `<out>/<area>_ground.odrz`, `<area>_decks.odrz` |
| `convert` | `odrforge build` per profile (standard: `<area>_standard.xodr`, carla: `<area>.xodr`), and the ground conformed to the roads | `<out>/...xodr`, `<area>_ground_conformed.odrz` |

A stage whose output exists is skipped (set `[stages.<name>] force = true` to redo it). Downloads are listed
first and need a yes (`[download] confirm = false` or `--yes` to skip the question).

## Config

TOML. Every key has a built-in default (`DEFAULTS` in the tool); a file states only what differs. An area's
missing keys come from `[area_defaults]`, then from the built-in defaults. Only `bbox` is required.

```toml
[paths]
data = "data"              # downloads (relative paths: to this file's folder)
out = "build/areas"        # grids and OpenDRIVE outputs
# odrforge = "...", python = "..."  (defaults: the repository build, the running Python)

[download]
confirm = true
# timeout = 600, overpass = "https://overpass-api.de/api/interpreter"

[area_defaults]
profiles = ["standard", "carla"]
# ground = "auto", decks = "auto", margin = 60.0, conformed = true, viz = false, build_args = []

[areas.munich]
bbox = [48.132, 11.560, 48.145, 11.585]   # min lat, min lon, max lat, max lon
origin = [48.1385, 11.5725]               # default: bbox centre
# country = "DE"   default: the elevation source's country, else XX
# ground = "auto" | "<source>" | "path/to/dem.tif" | "none";  decks likewise
# cell = 0.5       default: 0.5 m for 1 m sources, else 1 m;  datum = "..." for local files
```

`tools/pipeline.example.toml` holds the six reference areas.

## Sources (`auto`: the finest one whose coverage holds the whole bbox)

A source's coverage is the rectangle round its region; its data ends at the region's border. When the data holds
less than 99.5 % of the bbox (Stelvio: South Tyrol's DTM stops at the Lombardy border), `auto` adds the next
coarser source, and `make_terrain.py` fills the gap with it, shifted onto the finer data by the difference at the
nearest cell where both have data, so the border shows no step. Over `--seam` 100 m that shift fades to their median
difference where it is a consistent offset (|median| > IQR: another vertical datum), else to 0, so the fill keeps its
own heights (Stelvio: median +1.26 m, IQR 2.34 m - no shift). The grid keeps the finer source's datum.

| name | layer | coverage | resolution | vertical datum | access |
|---|---|---|---|---|---|
| `bavaria_dgm1` | ground | Bavaria | 1 m | DHHN2016 | 1 km GeoTIFF tiles, open data |
| `hesse_dgm1` | ground | Hesse | 1 m | DHHN2016 | INSPIRE WCS `he_dgm1` |
| `hesse_dom1` | decks | Hesse | 1 m | DHHN2016 | INSPIRE WCS `dom1` |
| `ea_dtm1` | ground | England | 1 m | ODN | Environment Agency WCS |
| `ea_dsm1` | decks | England | 1 m | ODN | Environment Agency WCS (last return) |
| `south_tyrol_dtm2_5` | ground | South Tyrol | 2.5 m | Genova1942 | Province of Bolzano WCS, CC0 |
| `gsi_dem5a` | ground | Japan | 5 m | JGD2011 (T.P.) | GSI zoom-15 text tiles |
| `copernicus_glo30` | ground | world | 30 m | EGM2008 | AWS open data (a surface model) |

The datum goes into the grids; `odrforge build` writes it into the geoReference (PROJ `+geoidgrids`) and the header
userData (a datum without an open PROJ grid, like Genova1942, only in the userData). New sources are one entry in `PROVIDERS` (bbox, CRS, resolution, datum, kind, URL).
