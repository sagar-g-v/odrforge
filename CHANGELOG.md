# Changelog

All notable changes are listed here. Versions follow [Semantic Versioning](https://semver.org/).

## [0.4.0] - 2026-10-11

### Added
- **Lane-level gores** (`BuildOptions::laneLevelGores`, on by default).
  - Freeway exits and entrances become splits and merges at the gore. The trunk carries the branch
    lanes, with a taper, up to where the driving lanes separate, and a 4 m junction there links the
    lanes 1:1.
  - The freeway is one continuous reference line through the OSM node. Chained gores (exit, entry,
    exit) are supported.
  - Downtown Los Angeles: 46 splits and merges.
- **OSM near-miss joining** (`BuildOptions::snapDanglingEnds`, 3 m). A way end that stops short of
  the road it points at is connected to it. Real dead ends are kept: `noexit`, turning circles,
  building, garage and barrier entrances, and ends beyond the kerb.
- **Parking-lot through routes** (`BuildOptions::connectParkingLots`). Surface-lot aisles become
  roads where the lot connects two or more streets; single-entrance lots and garage aisles stay out.
- Report and CLI: `snappedEnds`, `parkingAisles`, `gores`.
- **6 tests:** exit and entrance gores (lane counts, short junction, branch-lane links, no overlaps),
  the hook driveway, near-miss joining with the kept dead ends, parking lots, and a road measured at
  a single point.

### Fixed
- Roads whose only movement is sharper than 150° (a driveway drawn alongside the street that hooks
  into it at the last node) were dead ends. Such a movement is now built as a tight turn.
- The overlap resolver could push a road into its neighbour on the other side. Shifts are now
  limited by the free room on both sides, and gore partners count as obstacles.
- The Whittaker profile of a road measured at a single point extrapolated an arbitrary slope (one
  junction reached -73 m). Such a road is now level.
- Junctions with no measured road around them (underground garages) take their height outward from
  the measured network, nearest known neighbour first, with the 15 % gradient cap. Placeholder zeros
  are no longer used.

## [0.3.0] - 2026-10-11

### Added
- **Real elevation from open data.**
  - `tools/make_terrain.py` builds ground and bridge-deck grids (`.odrz`, with stacked deck levels)
    from a DEM plus lidar.
  - `osm2xodr --terrain/--structures` uses them. Roads follow their measured surface, chosen by
    dynamic programming over ground and deck levels.
  - Grade separation keeps 4.5 m clearance.
  - Covered ways and tunnels are not read from the surface above them.
  - The terrain crossfall is written as `superelevation`.
- **Junction surfaces.** Each junction is one plane chosen by consensus of the measured evidence.
  Roads are re-fitted to it (C1), and connecting roads take height and banking from it.
- **Overlap resolution** for near-parallel roads drawn too close in OSM: a bounded, tapered extra
  `laneOffset`.
- **`--clip`:** cuts the network to a lat/lon box, with ways split exactly at the boundary.
- **`--no-superelevation`:** flat cross sections and level junctions for consumers that ignore
  `superelevation`.
- **Quality rules:** `elevation_c1`, `superelevation_c0`, `max_grade`, `link_grade`,
  `junction_surface`, `road_overlap`.
- **ASAM XSD conformance test.** OpenDRIVE 1.8.1 schemas are included; `xmlschema` is needed.
- **Tests (42 cases):** terrain grid, sloped terrain, bridge deck, overlap, clip, covered way,
  wall-top outlier, flat mode, untagged crossing.

### Fixed
- Junction nodes on different measured levels (for example a street and a trenched freeway joined
  by a ramp) are no longer merged into one junction.
- Freeway and ramp ends may be trimmed back up to 160 m, so gores no longer overlap.
- `roadMark` `height` is only written when it is positive (XSD).
- Header `userData` order (XSD).

## [0.2.0] - 2026-10-11

### Added
- **Spec conformance checker.** `validate()` checks ~60 ASAM OpenDRIVE rules, each tagged with its
  ASAM rule id or spec section. Areas covered: geometry sums, gaps, kinks and minimum lengths;
  elevation and laneOffset continuity; lane sections, ids, types, widths, marks, speeds and links;
  links on both sides; connecting-lane fit (position, heading, width, z); signals; controllers;
  junctions; objects.
- **Test suite (33 cases).** Covers the spec formulas (line, arc, spiral, poly3, paramPoly3 in both
  pRange modes, width, laneOffset, elevation, taper cubic), writer required attributes, the 1.8
  mappings, precision, a reader round trip, negative validator tests, and 9 end-to-end OSM scenarios:
  crossroads, T junction with a stop sign, lane taper, left-hand traffic, CARLA, 1.8, curve and
  roundabout.
- **OpenDRIVE reader.** Handles 1.4 to 1.8 files, including poly3 / paramPoly3 geometry and border
  lanes.
- **Interactive HTML viewer.** Available as `osm2xodr --viz out.html` and as the `xodrviz` tool.
  Features: pan and zoom, layers, lane and road inspection, curvature heat map, signals, objects
  and validation status.

### Fixed
- Signal `validity` now satisfies fromLane <= toLane.
- Output precision raised to 12 significant digits.
- Geometries shorter than 1 mm are folded into neighbouring elements.
- Roads link only to junctions that actually connect them.
- Lane width profiles for lanes that both appear and disappear.
- Lane-offset transitions.
- Clothoid fitting: spurious-root rejection, recursive subdivision, a curvature limit tied to the
  road half-width, and best-of connecting paths.
- The end-to-end run is 8.6x faster (projection seeding).

## [0.1.0] - 2026-10-11

### Added

- **Geometry core:** OpenDRIVE `line`, `arc` and `spiral` evaluation (Gauss–Legendre generalised
  Fresnel integrals), and G1 Hermite clothoid fitting (Bertolazzi & Frego). G2 clothoid-spline
  fitting of OSM polylines includes noise removal, corner fillets and an oscillation fallback.
- **Inputs:** WGS84 transverse Mercator projection (Krüger series, matches PROJ to under 1 mm) and an
  OSM XML reader.
- **Lane model from OSM tags:** lanes, turn lanes, widths, sidewalks, cycle lanes, parking, shoulders,
  bus lanes, speeds and placement.
- **Country rules:** XX, US, CA, DE, FR, GB, IN, JP, AU. Output targets: asam, asam18, carla, esmini.
- **Network builder:**
  - chained roads with tapered lane sections
  - geometric junction clustering and trimming
  - lane-level clothoid connecting roads (turn lanes, restrictions)
  - elevation from a terrain callback, including bridges and tunnels
  - traffic lights with controllers, stop, give-way and speed-limit signs
  - crosswalks, street lamps and trees
- **Output and checks:** an OpenDRIVE 1.4 / 1.6 / 1.8 writer and a validator.
- **Tooling:** the `osm2xodr` command-line tool, and unit tests for geometry, projection and tag
  parsing.
