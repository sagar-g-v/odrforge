# Changelog

All notable changes are listed here. Versions follow [Semantic Versioning](https://semver.org/).

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
