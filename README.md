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

## Visualise

```bat
osm2xodr area.osm out.xodr --country US --viz out.html
xodrviz any.xodr view.html
```

Both commands write one self-contained HTML file, with no external resources:
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
