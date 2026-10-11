# OdrForge

OdrForge converts OpenStreetMap into ASAM OpenDRIVE. It is a C++17 library plus the `osm2xodr`
command-line tool.

- **Standard geometry only.** Reference lines are clothoid splines of `line`, `arc` and `spiral`
  elements, with continuous heading and curvature.
- **Native OpenDRIVE structure.** Roads are two-way. Lane sections taper widths, and `laneOffset`
  follows OSM `placement`. Lane-level connecting roads are built from `turn:lanes` and turn
  restrictions.
- **Simulator-ready traffic assets.** Traffic lights come with controllers that are referenced from
  their junctions. Stop and give-way signs follow OSM direction tags, speed-limit signs appear where
  limits change, and crosswalks, street lamps and trees are placed as objects.
- **Any country.** JSON rule files cover driving side, widths, implicit speeds, marking styles and
  sign catalogues. Included: `XX` (generic), `US`, `CA`, `DE`, `FR`, `GB`, `IN`, `JP`, `AU`.
- **Targets.** `asam` (OpenDRIVE 1.6), `asam18` (1.8: `walking` lanes, `direction`, ISO country
  codes), `carla` (1.4 plus CARLA conventions), `esmini`.
- **Connected network.** OSM near-misses are joined, and parking lots that link streets are
  driven through. Freeway exits and entrances are lane-level splits and merges at the gore, so road
  surfaces never overlap there.
- **Validated on every run.** Checks cover geometry gaps, lane-level link gaps and headings,
  references and ids.

See [docs/DESIGN.md](docs/DESIGN.md) for the approach and the measured comparison with SUMO
`netconvert`.

## Build (Windows, MSVC 2022)

```bat
build.bat Release
```

On other platforms:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && ctest --test-dir build
```

Dependencies (pugixml and nlohmann/json) are fetched at configure time and pinned by SHA-256.

## Use

```bat
osm2xodr area.osm out.xodr --country US --target carla --origin 34.054,-118.2575
```

```cpp
#include <odrforge/network.hpp>
using namespace odrforge;
OsmData osm = OsmData::readXml("area.osm");
BuildOptions opt;
opt.country = CountryProfile::load("DE");
opt.target = TargetProfile::byName("asam18");
opt.elevation = [&](double x, double y) { return dem.height(x, y); };  // optional terrain
BuildReport rep;
odr::OpenDrive od = buildOpenDrive(osm, Projection(lat0, lon0), opt, &rep);
odr::write(od, "out.xodr", {opt.target.revMinor});
```

### Real elevation

```bat
python tools\make_terrain.py --origin 34.054,-118.2575 --bounds 34.044,-118.27,34.064,-118.245 ^
    --dem usgs_1m.tif --lidar tile.laz --ground ground.odrz --structures decks.odrz --datum NAVD88
osm2xodr area.osm out.xodr --country US --target carla --origin 34.054,-118.2575 ^
    --terrain ground.odrz --structures decks.odrz --clip 34.044,-118.27,34.064,-118.245
```

Roads follow the measured ground, and bridges follow their measured decks. Roads under another
road keep 4.5 m clearance. Each junction is one plane shared by all its connecting roads, and
crossfall is written as `superelevation`. Pass `--no-superelevation` for consumers that ignore it.
The vertical datum of the grids is recorded in the header `userData`. See
[docs/DESIGN.md](docs/DESIGN.md#real-elevation-elevationcpp).

## Verify against the map and imagery

```bat
python tools\make_basemap.py --origin 34.054,-118.2575 --bounds 34.044,-118.27,34.064,-118.245 ^
    --out out_basemap --osm area.osm --naip
osm2xodr area.osm out.xodr ... --viz out.html --basemap out_basemap
```

The viewer's Background selector then offers:
- an HD map rendered from the OSM file,
- USGS NAIP satellite imagery (offline, 0.6 m),
- live Esri imagery.

All of them are placed in the network's own projection, under the roads. A path on the map with no
lanes over it, or lanes beside the street in the imagery, is a difference to look at.

## Visualise

```bat
osm2xodr area.osm out.xodr --country US --viz out.html
xodrviz any.xodr view.html
```

Both commands write one self-contained HTML file. Nothing is fetched from the web unless you turn
on the optional OpenStreetMap layer (tiles placed with the file's projection, to compare the
generated roads with the map):
- pan and zoom, plus layer toggles: lanes by type, junction areas, road marks, reference lines,
  curvature heat map, signals, objects
- hover and click inspection of lanes and roads, and road search
- the validation status of the file

## Conformance

`validate()` checks the ASAM OpenDRIVE rules listed in `src/validate.cpp`. Every message carries the
ASAM rule id or the spec section it enforces. The test suite (`ctest`) runs the formulas, writer,
reader and 9 end-to-end OSM scenarios against these rules.

## Versioning

The library follows [Semantic Versioning](https://semver.org/). The version lives in `VERSION` and
is exposed as `ODRFORGE_VERSION` (`<odrforge/version.hpp>`). It is written into the OpenDRIVE header
`vendor` attribute so every file records the generator version. Releases are git tags `vX.Y.Z`; see
[CHANGELOG.md](CHANGELOG.md).

## License

Proprietary. See [LICENSE](LICENSE).
