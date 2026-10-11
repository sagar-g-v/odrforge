# OdrForge design

OdrForge converts OpenStreetMap into ASAM OpenDRIVE by building OpenDRIVE structures directly. It
does not go through a traffic-simulation network the way SUMO `netconvert` does. Each OSM concept maps
onto the OpenDRIVE construct the standard defines for it, so it can work for any road in any country,
driven by per-country rule files.

The research behind this design is in `docs/research/`:
- `opendrive_spec_notes.md`: ASAM OpenDRIVE 1.4–1.9 reference, plus CARLA and esmini constraints.
- `sumo_opendrive_analysis.md`: how SUMO's OSM → OpenDRIVE path works, with 25 defects and their root causes.

## Principles

1. **Only standard reference-line geometry.** The plan view uses `line`, `arc` and `spiral` (clothoid)
   only, never polylines, `poly3` or `paramPoly3`. Every curve is defined by its curvature profile, so
   heading is continuous and curvature changes linearly within each element.
2. **Two-way roads, as in OpenDRIVE.** One road carries both directions. The OSM way is the reference
   line, and `laneOffset` places the lanes (honouring OSM `placement`). SUMO instead makes one road per
   direction joined by fake junctions.
3. **Lane changes are lane sections, not junctions.** Chained OSM ways become one road. A change in
   lane count is a new `laneSection` where the added or dropped lane tapers its width to or from zero,
   with correct lane links.
4. **Junctions only where roads really meet.** Intersections are clustered from carriageway geometry,
   and roads end where their carriageways stop overlapping, plus a curb-return allowance.
5. **Lane-level connecting roads.** Every permitted lane-to-lane movement is one connecting road whose
   reference line is the lane-centre path, a clothoid built from the incoming and outgoing lane poses.
   Permissions come from `turn:lanes`, then turn-restriction relations, then positional defaults.
6. **Signals that simulators can use.** Controllers pair opposite approaches into phases and are
   referenced from their junction. Signals sit on incoming roads with lane validity. Stop and give-way
   signs come from OSM direction tags, and speed signs appear where the limit changes. Ids are short
   and numeric.
7. **Country rules are data.** `rules/countries/<ISO>.json` holds the driving side, lane widths,
   implicit speeds and zones, lane counts, marking colours and styles, curb height, and the sign
   catalogue. Targets (`asam`, `asam18`, `carla`, `esmini`) add consumer quirks.
8. **Validate every build.** Checks cover geometry continuity, lane-level gaps and headings across
   links, references, ids and the lane-section order.

## Pipeline (`src/network*.cpp`)

| Stage | What it does |
|---|---|
| topology preparation (`snap.cpp`) | Joins OSM near-misses: a way end that stops short of another road (within that road's half-width plus 3 m, pointing at it, same layer) gets connected to it. Real dead ends are kept: `noexit`, turning circles and loops, ends at buildings, garages and barriers, ends beyond the kerb. Surface parking-lot aisles become roads where the lot connects two or more streets. |
| collect segments | Selects road ways (`highway` set, `service` exclusions, `access`) and splits them at shared nodes. Builds the cross-section from tags (`lanes.cpp`). |
| junction nodes | Marks nodes with 3+ segment ends, and 2-end nodes where two oneways conflict. |
| chains | Joins pass-through segments into roads, oriented along the oneway traffic. |
| cluster junctions | Union-find merges intersection nodes joined by segments shorter than the sum of their carriageway half-widths plus slack, which turns dual-carriageway crossings into one junction. Internal segments are dropped. With terrain, two nodes merge only if some measured surface at each (ground or a deck level) is within a junction grade of the other, so a ramp between a street and a trenched freeway stays a road. |
| reference lines | Fits a clothoid spline through each chain polyline (`clothoid_fit.cpp`). Lane sections start at way joints, and every OSM node gets its `s`. |
| lane-level gores (`gore.cpp`) | A freeway split or merge at a shallow angle (three oneway roads, branches within 30°) is modelled as it is built. The freeway becomes one reference line through the OSM node, and the trunk carries the branch lanes beside the main lanes until the driving lanes separate. Those lanes are added with a taper for an exit, or dropped after an entrance. A 4 m junction sits at that point: main lanes continue 1:1, and the branch lanes become the ramp. |
| trim roads | Scans each road end outward from its junction until neither carriageway edge overlaps another road of the junction, then adds the stop-line setback and the curb return. Freeways and ramps may be trimmed back up to 160 m, because ramps leave at shallow angles. |
| resolve overlaps | Near-parallel roads on the same layer that OSM draws closer than their lanes are wide get pushed apart by a smooth extra `laneOffset`: at most 3 m, tapered at no more than about 5 %. A shift never exceeds the free room on that side, and a road squeezed from both sides is left in place. Gore partners count as obstacles and are never pushed apart. |
| roads | Builds lane sections with tapers and lane links, `laneOffset` held across one-sided changes, road types, speeds, and bridges and tunnels. |
| elevation | Fits measured terrain to the roads (see [Real elevation](#real-elevation-elevationcpp)). The output is C1 cubic Hermite `elevation` records and `superelevation` (crossfall). |
| junctions | A movement sharper than 150° is a U-turn and is not built, unless it is the only way out of (or into) a road end and leads to another road; such a movement is built as a tight turn instead (for example a driveway hooking into the street at its last node). Builds connecting roads per movement: a single G1 clothoid, or a curvature-continuous 2-piece curve when it would be tighter than `minConnectingRadius`. Width and `laneOffset` follow cubics. Height and banking come from the junction surface, so every connecting road of a junction lies on one plane. |
| signals | Creates traffic lights, controllers and junction references, stop and give-way signs, and speed-limit signs. |
| objects | Adds crosswalks (mapped crossings, plus every approach of signalised junctions), street lamps and trees. |

## Geometry (`clothoid_fit.cpp`)

The geometry is built in five steps:

1. **Clean the polyline.** Drop duplicate points, run Douglas–Peucker with `noiseTolerance` (0.25 m,
   OSM digitising noise), and fillet sharp corners: corners turning more than 20° between legs get two
   tangent points at the class minimum radius.
2. **Set headings at the vertices.** Inner vertices take the circle tangent. End vertices take the
   fixed continuation headings, or mirrored tangents.
3. **Solve each segment.** Every segment is one G1 Hermite clothoid, using Bertolazzi & Frego (2015)
   with Gauss–Legendre generalised Fresnel integrals. Looping roots are rejected, and failures split
   recursively at a chord midpoint.
4. **Make curvature continuous.** Newton iterations on the inner headings, with a tridiagonal finite-
   difference Jacobian and damping, make curvature continuous at every vertex (G2). Where G2 would
   oscillate (short fillet legs next to long legs), the local G1 spline is kept.
5. **Merge elements.** Near-zero curvature becomes `line`; near-constant curvature becomes `arc`.

Unit tests (`tests/test_geometry.cpp`) cover:
- exact arcs and spirals
- G1 accuracy on 2,000 random cases
- a circle becoming a single `arc`
- noise becoming a single `line`
- G2 continuity at a filleted corner
- the `project`, `slice` and `reverse` operations

## Lanes from OSM (`lanes.cpp`)

| Topic | OSM input → OpenDRIVE output |
|---|---|
| Lane counts | `lanes`, `lanes:forward` / `lanes:backward`, `lanes:both_ways` (becomes `bidirectional`, or `direction="both"` in 1.8), oneway / reverse / roundabout / motorway defaults |
| Per-lane data | `turn:lanes*`, `width:lanes*`, carriageway `width` (feet and inches handled), `bus:lanes` / `psv:lanes` (driving lanes with access restrictions) |
| Side lanes | `cycleway:*` lanes; parking in both schemes (`parking:<side>` with orientation, and `parking:lane:<side>`); country shoulders; sidewalks (`sidewalk*` and widths, assumed on untagged urban streets, raised to curb height) |
| Speeds | `maxspeed`, `maxspeed:forward` / `maxspeed:backward`; units km/h, mph and knots; zone codes such as `DE:urban` |
| Placement | `placement` / `placement:forward` (`left_of`, `middle_of`, `right_of`) sets `laneOffset`. The default is the middle of the carriageway. |
| Road marks | Broken lines between same-direction lanes; the centre line by country style (US `solid solid` yellow); solid edge lines next to bike and parking lanes and on motorways; `bold` edges for bus lanes. |

## Real elevation (`elevation.cpp`)

Heights come from open data. Use national lidar where it exists (USGS 3DEP, UK EA, NL AHN, FR IGN
LiDAR HD) and a DEM elsewhere (3DEP 1 m, or Copernicus GLO-30 / FABDEM worldwide).
`tools/make_terrain.py` converts these into two grids in the OdrForge frame:
- **ground:** the bare-earth DEM, refined by lidar ground points that agree with it within 0.5 m.
- **decks:** lidar class 17, plus "ground" points more than 2 m above the DEM (overpasses the
  classifier missed). Stacked decks are kept as up to 4 levels per cell.

From those grids, heights are built in nine steps:

1. **Candidates.** Every 2 m along a road, the ground and every deck level at the carriageway centre
   are candidate surfaces. OSM bridges may only use decks. Tunnels and `covered=yes` ways have no
   candidates, so the surface above them never becomes the road.
2. **Surface choice.** Dynamic programming along the road picks the candidate sequence with the
   fewest jumps. Unobserved stretches are allowed (skip states), and grades above 30 % are penalised.
3. **Grade separation.** Roads that overlap in plan without a junction keep 4.5 m clearance. The
   lower OSM layer gives way; the layer within 30 m counts, because bridge ways often end short of
   the crossing. If neither road is tagged, the one already lower gives way, and on a tie the lower
   road class. An excluded level continues along the lower road while its surface stays in the band
   (shoulders and gores of the upper structure).
4. **Smoothing.** A Whittaker smoother (second differences, banded LDLᵀ) with Tukey reweighting
   rejects walls, kerbs and vehicles. A grade guard adds smoothing where the profile is steeper than
   the terrain supports.
5. **Crossfall.** The terrain cross slope over the carriageway becomes `superelevation`, smoothed and
   limited to 8 %.
6. **Junction surfaces.** Each junction is one plane. Its level is the height supported by the most
   evidence: road ends weighted by class and by how close to the junction they were measured, plus
   the ground at the centre. An end counts as agreeing if a 12 % grade can reach it. The plane is
   then fitted through the agreeing ends and the measured ground on that level (robust, gradient
   ≤ 15 %).
7. **Re-fit to the planes.** Every road's measured profile is fitted again with the planes' heights
   and gradients as boundary conditions, so a level difference spreads over the road instead of
   forming a step. A short C1 Hermite blend removes the centimetre residue. Stubs shorter than 6 m
   simply continue the plane.
8. **Connecting roads.** Height and banking are sampled from the plane every 2 m, so all lanes of a
   junction share one surface and match the road ends exactly.
9. **Flat cross sections (`--no-superelevation`).** For consumers that ignore `superelevation`
   (LibCarla's own meshing), there is no crossfall and junctions are level, so overlapping lanes
   still agree. This mode is steeper next to junctions on hills.

Measured on downtown Los Angeles with USGS 3DEP 1 m and LA County 2023 lidar:
- lane centres sit 0.09 m from the measured ground on average (p95 0.20 m);
- 0 junction-surface mismatches;
- link height gaps under 1 cm;
- 0 position jumps in CARLA (previously 133 vertical and 190 horizontal);
- 0 ASAM 1.8.1 XSD errors.

The remaining grades above 20 % are private garage ramps and driveways through the Bunker Hill
slope, where the measured terrain itself is that steep.

## Measured on Los Angeles downtown (16 MB OSM, CARLA target, US rules)

Both files were loaded with CARLA's own parser (LibCarla 0.10 Python API). Every driving lane was then
walked in 1 m steps: `scripts/compare_xodr_carla.py` in MirrorMile.

| Metric | SUMO netconvert (MirrorMile 01–02f) | OdrForge 0.1.0 |
|---|---|---|
| Position jumps > 1.6 m | 490 | 261 |
| Worst jump | 4.12 m | 2.39 m |
| Heading kinks > 12°/m | 6,493 | 4,276 |
| Heading change p99 / p99.9 | 9.87 / 21.9 °/m | 7.94 / 14.5 °/m |
| Junctions with waypoints | 820 | 1,149 |
| Validation errors | – | 0 |
| Run time | about 2 min (netconvert plus 5 Python fix-up steps) | 11.9 s |
| Fit to OSM nodes | – | mean 0.21 m, p95 1.0 m |

## Roadmap

- **Remaining kinks and jumps:** tight connecting roads at acute corners, short roads squeezed
  between close junctions, and the 18.7 m fit outlier (motorway-radius fillets).
- **Ramps:** `direct` junctions for motorway ramps in the 1.8 target (CARLA ignores them, so the
  CARLA target keeps default junctions).
- **Markings:** stop lines (`1100001` invisible stop-line signals, plus `roadMark` objects),
  crosswalk `<markings>`, turn-arrow markings from `turn:lanes`.
- **Data sources:** read junction areas from `area:highway`, junction shapes from `junction=yes`
  areas, and destination signs.
- **Signals:** per-lane-group heads, signal groups for protected turns, and `dependency` /
  `reference` links.
- **Inputs:** OSM PBF via libosmium, and GeoTIFF terrain read directly (today it goes through
  `make_terrain.py` to `.odrz`).
- **Braided ramps:** a branch whose OSM line crosses the main line before it separates (flyover
  ramps in the Four Level Interchange) is not a gore. It needs the vertical separation known when
  the junction is shaped.
- **Python bindings (pybind11)** and MirrorMile integration, replacing steps 01–02f.
- **More country rule files,** plus the US state and EU zone speed tables.
