# Changelog

All notable changes are listed here. Versions follow [Semantic Versioning](https://semver.org/).

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
