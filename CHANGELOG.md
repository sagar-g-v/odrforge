# Changelog

All notable changes are listed here. Versions follow [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- **Separate sidewalks (experimental, `--separate-sidewalks`)**: a sidewalk mapped as its own way
  next to a road is measured and the road's cross-section reaches out to it (a border verge with a
  curb mark, then the sidewalk at its mapped position). It extends 2131 road sides in LA, 1114 in
  London and lets corner sidewalks continue round 958 / 517 corners there, but verge widths do not
  yet blend across way segments and wide verges fold in tight bends (folds LA 69 -> 100), so it is
  off by default. Verges (then sidewalks, down to 1 m) now narrow per section where the bend towards
  them is tighter than 0.8 / their reach (decided with Jev: per-section cap 0.63 vs sample-wise
  narrowing 0.22): folds LA 100 -> 78, London 42 -> 32, Tokyo 26 -> 19, Munich 35 -> 28. The verge
  lane stays (at least 5 cm) wherever the sidewalk is attached, so segments keep one lane layout (LA
  unexplained edge breaks 51 -> 36). OSM kerb lines (barrier=kerb) place the sidewalk's curb-side edge
  where a mapped footway runs beyond them on the same side: the sidewalk spans from the kerb to the
  footway's outer edge, both measured (a kerb alone is not used). Once a road's reference line is
  fitted, every attached side is measured again from that line, not the raw way (which lies 0.31 m off
  it on average in London). Curb edges within 0.2 m of a London kerb line (new `tools/inspect/kerb_match.py`):
  no kerbs 20.9 % (median 0.55 m) -> 45.2 % (median 0.24 m, no bias); folds 30 -> 31. Kept with Jev
  0.88 (re-measuring 0.85). The rest is the kerb distance varying within a section (median p10-p90
  spread 0.60 m) while verge widths are constant per section.
- `ODRFORGE_VERBOSE` prints every build stage with its run time. Arm trimming now runs per junction
  in parallel (LA: 2 x 9.4 s -> 2 x 0.9 s, output byte-identical); overlap resolution (2 x 5 s) is a
  sequential sweep whose result depends on its order and stays single-threaded.
- **Curb returns** (ASAM Junction Guideline §6): every arm is cut where its curb return starts. At
  a corner whose facing sides end in sidewalks the curb lines meet at a corner point; a return of
  radius max(corner radius, 2.5 sidewalk widths) starts R tan(turn / 2) before it, and the arm cut
  moves back that far (at most 40 % of the road per end). The corner sidewalk follows the return as
  a fillet - straight, circular arc, straight - the largest radius up to the wanted one that fits.
  Acute corners (forks, turning more than 120 degrees) end in a nose and get no corner sidewalk.
  Corners too tight for their sidewalk: 133 -> 29 over the six test areas.
- **Roundabout junction groups** (OpenDRIVE 1.9 §12.13, valid from 1.4): junctions joined by ring
  roads (OSM junction=roundabout / circular) are listed in one junctionGroup of type roundabout.
- **Sidewalks continue round junction corners** (ASAM Junction Guideline §6, OpenDRIVE 1.9 §12.4).
  Between neighbouring arms whose facing sides end in a sidewalk, a junction road runs along the
  curb line carrying the sidewalk (raised by the curb height, written as walking in 1.8), linked to
  both arms' sidewalks, with the arms' sideways slope at its ends. Where the curb line bends
  tighter than the sidewalk is wide the sidewalk narrows towards the middle of the corner; a corner
  whose arm cuts leave no room for the sidewalk to turn at all is counted, not built
  ("corner sidewalks: N continued, M too tight" in the report). Corners continued: LA 126, Munich 267,
  London 33, Frankfurter Kreuz 78, Stelvio 19, Tokyo 1219; too tight: 133 in all.
- The invariant and lane-flow checkers treat pedestrian-only roads and pedestrian lane links as
  usable without a direction of travel; the unit-test runner prints unbuffered.

### Changed
- **One junction per intersection** (ASAM Junction Guideline §6, OpenDRIVE 1.9 Annex F
  junctions.no_overlap). Where the arm cuts of two junctions leave less than 3 m of a road no longer
  than 15 m between them (the two halves of a dual-carriageway crossing, a station square of short
  links), the two are built as one junction: junctions LA 973 -> 936, London 358 -> 326, Frankfurter
  Kreuz 280 -> 242, Tokyo 630 -> 594; no overlapping junctions left in LA, London and Frankfurter
  Kreuz. Longer roads between two nodes of one junction stay roads. Inside such junctions OSM turn
  restrictions bind only the movements that pass their via node, a restriction into an absorbed way
  leads on to the arms beyond it, and U-turns across a median of at least 8 m are kept. Lane flow:
  Frankfurter Kreuz edge pairs OSM drives and the lanes miss 129 -> 1, lost OSM flows 43 -> 1, traps
  and dead ends 0 in Frankfurter Kreuz and Tokyo.
- **Connecting-road frames that would distort their lanes are not used.** The arm-to-arm reference
  line is taken only while every lane border stays within half the turning radius of it
  (|kappa t| <= 0.5); otherwise the lane gets a lane-centred road, or the arm frame if the
  lane-centred one would fold. Lane borders follow the arms' sideways slope in both cases. This
  restores 0 sideways discontinuities in CARLA for LA (26 with the previous release).
- **Connecting roads continue the arm reference lines** (OpenDRIVE 1.9 §12.4, ASAM Junction
  Guideline §6). A connecting road's reference line now starts and ends on the incoming and
  outgoing roads' reference lines, in position and heading, in the direction of travel; the lanes
  sit beside it through laneOffset and widths that blend from the incoming to the outgoing lanes
  with the arms' own sideways slope at both ends. Adjacent lanes going to adjacent lanes share one
  connecting road (§12.4 lane_change_one_con_road). Where that frame would fold (a lane edge past
  the centre of curvature) the lanes keep their own lane-centred roads. Connecting roads on the arm
  reference lines: 0 - 28 % -> 94 - 99 % over the six test areas; lane-edge heading breaks at
  links (Annex D): LA 132 -> 60, Tokyo 140 -> 36, Frankfurter Kreuz 234 -> 153, London 14 -> 5;
  folds LA 70 -> 62. Reference-line curvature jumps at junction links are now counted (the lines
  meet); curvature continuity is not required by Annex D and is left for phase 3.

### Fixed
- **Bus lanes in the carla (1.4) profile**: the access restrictions passengerCar / truck / motorcycle
  are not in the 1.4 vocabulary (simulator, autonomous traffic, pedestrian, none) and were schema
  errors; vehicle classes are written from 1.6 on with access@rule, a 1.4 bus lane is a plain driving
  lane. Kept with Jev 0.89.
- **Road mark laneChange** (OpenDRIVE 1.9 §11.9 Table 48 / 173, same in 1.4): derived from the final
  mark type - solid lines are not crossed, a double line only from its broken side. The center line
  next to a US two-way left-turn lane was written laneChange none (its type was overridden after the
  value was set), and the turn lane's own edge put the solid line next to it; both edges are now
  broken next to the turn lane (MUTCD §3B.03), crossable both ways. Bike-lane solid edges: both ->
  none. LA lane flow: trapped 1 -> 0, islands with a missing link 17 -> 2. Kept with Jev 0.83.
- **Removable lane-section boundaries**: placing the sections on the trimmed road (end sections
  stretched, tiny ones dropped) could leave two equal neighbouring sections; the second is now
  dropped (OpenDRIVE 1.8 §11.3 needs a new section only where lanes change). London 8 -> 6, Munich
  10 -> 8 removable boundaries, nothing else changes. Kept with Jev 0.87.
- **Unmeasured road ends** (a tunnel or covered way running to the road end, an unmeasured deck): the
  profile continued at the smoother's end slope, which the last one or two samples set - in LA a 0.4 m
  outlier lifted an 80 m tunnel and its underground junction by 6.7 m, 1.8 m under the street above
  (road_overlap error with --separate-sidewalks). It now continues at the grade of a line through the
  measurements within 20 m. LA: 38 of 1778 roads move > 1 cm (5 > 2 m, on the West 4th Street viaduct
  whose ends are unmeasured), invariants unchanged, no crossing under 4.5 m clearance (minimum 6.2 m
  before and after); the other areas have no terrain and are unchanged. Kept with Jev 0.80.
- **Lane widths blend at a section start even when the lane ends at the section's end.** The start
  blend was dropped for such lanes, so their borders jumped sideways at the boundary (with
  `--separate-sidewalks`, London: lane-edge position breaks 11 -> 0, heading breaks 21 -> 10; default
  output unchanged on the test areas).
- **Every lane entering a junction has a path through it** (OpenDRIVE 1.9 §12.3, Junction Guideline
  §6). A lane whose tagged turn had no road to take, or that a lane split skipped, ended at the
  junction. It now takes the target on its own side of the approach. Lanes without a path:
  73 -> 1 over the six test areas; dead-end lanes LA 55 -> 26, Munich 30 -> 1, Frankfurter Kreuz
  17 -> 5, Tokyo 8 -> 2.
- **OpenDRIVE 1.4 schema conformance of the carla profile** (found by the official ASAM checker):
  road types `townExpressway` .. `townPlayStreet` are written as `town` before 1.5; tunnels are
  written before bridges (every schema fixes that order); junctions left without any connection are
  removed. A new CTest validates every scenario against the OpenDRIVE 1.4H XSD as well as 1.8.1.
  `rule="LHT"` stays on left-hand-traffic roads in 1.4 files (documented deviation: without it
  readers drive every lane against its traffic).
- **Redundant records, by the curve they describe**: an elevation / superelevation / lane offset
  record is left out when the kept polynomial reproduces it over its whole span within 0.1 mm /
  1e-5 rad / 1e-6 m (LA standard output 18 % smaller); headings are written in (-pi, pi].
- **Redundant records** (Annex F `performance.avoid_redundant_info`): elevation, superelevation,
  lane offset and lane width records that only continue the previous cubic are left out. Files are
  10 - 18 % smaller; every curve is unchanged (largest difference 5e-8 m over the six test areas).

### Added
- `tools/inspect/annex_f.py` (57 Annex F rules on the written file, with a mutation self-test) and
  `tools/inspect/asam_qc.py` (the official ASAM qc-opendrive checker). `docs/RULES.md` now lists
  116 of the 274 Annex F rules as checked, 144 as not applicable, 14 open (phases 1 - 3).

### Changed
- **Two output profiles.** `standard` (OpenDRIVE 1.8, the default) and `carla` (OpenDRIVE 1.4 plus
  CARLA conventions). `asam`, `asam18` and `esmini` are accepted as `standard`; an unknown name is
  an error. Writers ask `SpecRevision` named questions (lane direction, access restriction
  elements, outlines wrapper, ...) that cite the OpenDRIVE 1.9 section and the revision that
  introduced or deprecated each feature, instead of comparing `revMinor`.

### Fixed
- **Grade separations without elevation data.** Without terrain every road lay at z = 0, so bridges,
  tunnels and stacked interchange ramps overlapped the roads they cross (Frankfurter Kreuz: 190
  overlaps). The ground stays flat and is left only where roads of different OSM layers cross
  without a junction: one storey apart (`BuildOptions::levelHeight`, 6 m, or the lower road's
  `maxheight` + 1 m for garage decks), with ramps whose steepest point is a class design grade (5 %
  motorway, 6 % primary to tertiary, 8 % residential, 12 % service). Every crossing has a level of
  its own (a viaduct or a ramp mapped as one bridge is not one height along its length); the
  stretches of one layer that meet in junctions prefer one level and stay up between their
  crossings. The levels are solved as difference constraints, upward from the ground first: a ramp
  too short to climb a storey lifts the junction at its foot instead of pulling the structure, and
  everything beneath it, down. Structures above the ground never go below it, tunnels never above
  it, streets stay on it. Where the mapped structures lie closer than any ramp allows, the
  contradiction is loosened as little as needed - the street beneath dips into a cutting of at most
  a storey, then the storey shrinks - and reported. Tokyo (Shuto expressway over Route 246): no
  bridge below the ground (was 642 m), streets more than 3 m deep only next to tunnel portals
  (826 m, was 3534 m).
- **`tunnel=building_passage`** is a passage through a building at ground level: implied layer 0,
  not -1 (it was sunk a storey under the roads around it).
- **An inner lane that merges away from the divider** (`turn:lanes` `merge_to_right`, LHT
  `merge_to_left`, on a oneway) ends at the inner side and the other lanes run straight, lane 0
  following its taper; before, the outermost lane ended and the others shifted over by a lane width
  (with `placement` against the mapped line: Frankfurter Kreuz, a trunk link pushed into the trunk).
- **Overlap resolution:** two roads are a gore (left to their junction) only when they leave it
  heading the same way; a road that loops back alongside another is pushed apart from it. A
  realignment ramp ends before a lane addition or drop instead of running through its taper (no
  shorter than a shifting taper at the posted speed).
- **osm2xodr** prints the warnings by kind (the first three of each and a count), so a rare kind is
  not hidden behind many of a common one.
- **Lane width changes between ways** (different `width` / `lanes` tags) no longer step at the lane
  section boundary (a sideways jump of the lane border for CARLA and any vehicle following the lane).
  The width, and the lane offset with it, blends across the boundary, up to half a taper on each
  side.
- **No undrivable connecting roads:** where the connecting path falls back to the straight chord
  between two nearly touching road ends, a chord that meets the lanes at an angle tighter than a
  5 m turning circle (for example two streets cut by the clip box half a metre past the
  intersection, a metre apart at a right angle) is left out instead of joining the lanes at an angle;
  a chord that a vehicle can follow is kept.
- **A single lane going straight on** into a road with more lanes across a short junction continues
  in the outgoing lane it lines up with (the added lanes begin beside it), instead of always the
  leftmost one: no sideways jog of a lane width inside the junction.

## [0.6.0] - 2026-10-11

### Added
- **Reference backgrounds for verification** (`tools/make_basemap.py`, `osm2xodr` / `xodrviz --basemap DIR`).
  Two offline layers are produced as tile pyramids of 512 px tiles:
  - **HD map:** a carto-style map rendered from the OSM file itself (landuse, water, parks,
    buildings, roads by class, paths, rail) at 0.3 m per pixel, supersampled. Nothing is downloaded
    from tile servers.
  - **HD satellite:** USGS NAIP orthoimagery (public domain, 0.6 m), fetched in its native Web
    Mercator and resampled into the network's transverse Mercator. The inverse projection
    round-trips to 0.02 mm.
  - The viewer embeds the pyramid description and loads only the tiles in view, with the overview
    level drawn first so nothing is blank.
- **Viewer Background selector:** None, HD map, HD satellite, or live satellite (Esri World
  Imagery). Live tiles are requested only inside the network's boundary. Background and road
  opacity are adjustable, and the attribution is shown for each source.

### Removed
- The live OpenStreetMap tile layer from 0.5.1. The tile servers block requests from a local file
  (no referrer, per their usage policy), and the HD map rendered from the same OSM data replaces it.

## [0.5.1] - 2026-10-11

### Added
- **Viewer: optional OpenStreetMap basemap underneath the network** (the "OpenStreetMap (Carto)
  underneath" layer). Use it to find missed paths and differences.
  - Tiles are placed with the file's own transverse Mercator geoReference (ellipsoidal forward
    projection).
  - Map and road opacity are adjustable, and the tile URL is configurable (kept per browser).
  - The OSM attribution is shown while the layer is on.
  - Off by default: it is the only part of the viewer that fetches from the web.
- `xodrviz` prints the validation rule counts (and every message with `ODRFORGE_VERBOSE`).

### Fixed
- Connecting roads that are numerically straight (solver noise, curvature ~1e-8) are exact lines.
  LibCarla evaluates arcs about their centre in 32-bit floats, and a radius of 10^7 m landed on a
  1 m grid. The in-memory geometry is changed, so the file stays self-consistent.
- The viewer's origin is written at full precision.

## [0.5.0] - 2026-10-11

### Added
- **Braided and chained gores.**
  - A branch line drawn across the main line before it leaves no longer blocks a gore. An
    excursion inside the shared carriageway is drawing noise; only a branch that fully clears the
    other side first is a braid.
  - A short branch that splits again right away becomes a gore once the split after it is modelled.
    The gore stage repeats until nothing changes.
  - Downtown Los Angeles: 49 gores and 0 road overlaps.
- **Lane splits and merges at plain junctions divide the lanes by side.** When a road continues
  straight on as several roads, its lanes go left to right in the order the roads leave, and merges
  likewise; connecting lanes no longer cross.
- **Corner fidelity.** Fillets are symmetric clothoid pairs (G2: curvature 0 on the legs), sized
  by the class radius, a corner tolerance (`maxCornerDeviation`, 1.5 m) and the lane-fold limit.
  Fillet tangent points keep their leg heading, so straight legs stay straight.
  - Fit to OSM nodes: mean 0.31 → 0.22 m, p95 2.4 → 1.8 m.
- **Topology.**
  - The missing intersection node is added where minor roads cross on one level without one.
  - The clip keeps a junction lying within 30 m outside the area, so roads no longer end side by
    side at the boundary.
- **CLI:** `--no-gores`, `--no-snap`, `--no-parking-lots`, `--no-overlap-shift`,
  `--corner-tolerance`.
- **8 tests (56 cases):** braid noise, chained gores, lane-split partition, clip junction, missing
  crossing node, corner fidelity, writer length guarantee, line classification.

### Fixed
- Freeways no longer snake sideways. The overlap resolver now only acts on driving lanes, so
  touching shoulders are allowed. Pushes on the same side within 150 m merge into one plateau,
  tapers are at most 2 %, and a shift never goes past a bend's centre of curvature. A second pass
  once heights are known handles roads on different OSM layers that are not really separated.
  - Roads with wobbling edges: 46 → 23. Overlap shifts: 99 → 61.
- At a gore, the lanes move into line with the next road over the whole stretch from the OSM node
  to the gore, instead of in a 30 m jog.
- A loop road with both ends at one junction was trimmed down to a 2 m stub (it overlapped
  itself). It now keeps its length, and no movement leads back into the same road.
- Connecting-road paths that loop are replaced (sanity check on turning and length).
- **CARLA compatibility.**
  - The written road length covers the written geometry, and every s position is written exactly
    on a 2^-24 m grid, because LibCarla asserts `s <= length` strictly on (L - s0) + s0.
  - Practically straight elements are written as lines: LibCarla evaluates an arc of radius
    10^7 m in 32-bit floats on a 1 m grid.
  - Downtown Los Angeles: 0 sideways discontinuities when walking every lane.
- Geometry elements under 1 cm are folded into a neighbour.
- Validator: for a loop road at a junction, the incoming end is taken from the connecting road's
  link.

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
