# Changelog

All notable changes are listed here. Versions follow [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- **Ground conformed to the roads** (`osm2xodr --ground-out C.odrz`, src/conform.cpp): the --terrain ground with the road
  surface under every lane and 1:1.5 cut / fill slopes beside it, for viewers and 3D scenes that drape terrain round the
  roads (a 30 m DEM cannot hold a mountain road's bench). Paved edges vs grid: Stelvio p95 3.55 -> 0.28 m, Munich
  0.19 -> 0.002 m, London 0.22 -> 0.003 m. Jev 0.80.
- **Area pipeline** (`tools/odrforge_pipeline.py`, docs/PIPELINE.md): one TOML config (built-in defaults for every key)
  drives OSM download, ground / deck sources picked by coverage (Bavaria DGM1, Hesse DGM1 / DOM1, EA DTM / DSM, GSI
  DEM5A, Copernicus GLO-30), terrain grids and conversion in each profile; downloads are listed and confirmed first.

### Fixed
- **Corner sidewalks follow the turning lane** (Munich review flags 4 / 5; Junction Guideline 1.0.0 §6.3): a corner
  sidewalk's curb is the outer driving-lane border of the connecting road turning round that corner, instead of a fixed
  6 m curb return a tighter turn ran over. Sidewalk-on-driving overlaps (JCT-8): Munich 175 -> 68, Tokyo 1008 -> 378,
  LA 101 -> 63, Stelvio 10 -> 0; more junction boundaries written (Tokyo 499 -> 529). Crosswalk footprints are no longer
  accepted past a sidewalk's end (a clamped projection). Option cornerCurbAlongTurns (default on). Jev 0.81.
- **Real terrain**: the Munich, London and Stelvio test areas build on national bare-earth DEMs (Bavaria DGM1, EA
  LIDAR DTM 1 m, Copernicus GLO-30): Munich 498 - 521 m, London 0.7 - 43 m, Stelvio 2408 - 2783 m (were relative,
  Stelvio flat). Over measured terrain the relative level solver now also runs: tunnels and bridges without deck data
  take its levels as offsets from the measured ground (Munich Altstadtringtunnel no longer 2 m under the street:
  validation 4 -> 2); building passages follow the ground; a road wholly covered at ground level is no longer left at
  height 0. The vertical datum goes into the geoReference as PROJ +geoidgrids (+vunits=m), and stays in userData.
  make_terrain.py: bridge decks from a surface model near OSM road bridges (--dsm --deck-osm). invariants.py: JCT-8
  sidewalk_on_driving. Frankfurt Kreuz (an isolated underground car park) and Tokyo (no open deck data) stay relative.
  Jev 0.80.
- **Review server: local cross-origin access** (`tools/inspect/viz_server.py`): the new review app
  ([odrforge-viewer](https://github.com/sagar-g-v/odrforge-viewer), React + deck.gl + MapLibre) runs on its own local
  port; the server grants CORS to localhost / 127.0.0.1 / [::1] origins only, so no web page can read networks or
  write flags through it. Jev 0.84.
- **Viewer: popups and panel close; 3D view**: the flag popup closes on Cancel / Save / Delete (its CSS display
  overrode `hidden`); the side panel toggles at every width; new 3D view (button 3D / key V) of the lanes on screen at
  their OpenDRIVE heights (elevation, superelevation, curb / sidewalk lane height; page data gains per-point z, +17 %).
- **Opposite carriageways drawn too close are pushed apart at junctions too**: the gore exemption of the overlap shift
  now requires the two roads' traffic to merge there (both into or both out of the junction); an arriving and a leaving
  carriageway of one street are a drawing offset. Tokyo roads 734 / 977 overlap fixed (validation 2 -> 1); G2 costs
  none (LA G2_ref_junction -7, London +1). Jev 0.86.
- **Shifted roads keep G2 lane edges** (Annex D.1.3): the laneOffset of a road pushed apart from a neighbour drawn too
  close is a cubic spline through its ~10 m knots (equal second derivative at every join; original slope and second
  derivative at both ends) instead of C1 pieces whose curvature stepped at every knot. Lane-edge curvature steps
  (G2_lane): LA 2359 -> 735, Munich 326 -> 173, Frankfurt 397 -> 177, Tokyo 243 -> 54, London 66 -> 60; junction joins
  LA -10. Cost: one Frankfurt road start 0.16 m off its OSM line (osm_off_road 37 -> 38). Jev 0.82.
- **Left-hand traffic corner sidewalks link from lanes leading into the junction** (Annex F.4.6.6
  junctions.connection.one_link_to_incoming): they were built the right-hand-traffic way round everywhere; under LHT
  they are now traversed from the other arm with their lanes on the left - same surface (0.0000 m, 0 m in z).
  London checker errors 68 -> 0; London 34 and Tokyo 1175 corner sidewalks; cross paths unchanged. Jev 0.87.
- **No repeated straight geometry** (ASAM qc-opendrive, Annex F.5.1 performance.avoid_redundant_info): consecutive
  line elements on the same heading are written as one. Frankfurt 7 and LA 15 redundant pairs -> 0; geometry, s and
  every exact check unchanged. Jev 0.87.
- **asam_qc.py: contact_point rule runs again**: the checker stopped on roads without the (optional) `<link>`; it now
  reads a copy with an empty `<link/>` there. All its reports are connecting roads (open item: Junction Guideline
  connecting-road reference lines); plain roads 0.
- **Corner sidewalks beside a straight-on movement follow the road** (Munich review flag 7): where the arms differ by
  less than 35 deg, the curb runs along the outer lane border of the connecting road carrying that movement (else the
  smoothest clothoid path, else the Junction Guideline curb return); boundary corner curves there use the smooth path.
  Corners of 35 deg or more are unchanged. Kinked straight-on sidewalks (worst 2 m window > 60 % of the turning) HEAD
  Tokyo 171, Munich 48, LA 24, Frankfurt 14, London 10, Stelvio 10 -> 0; inv:G2_ref_junction drops in every area; no
  checker count rises. Jev 0.87.
- **Lane counts of untagged ramp pieces** (LA review flags 1 - 3): an untagged `*_link` piece takes the lanes of the
  tagged piece of the same class it continues one to one (83 % of tagged continuations in the test areas agree), and an
  untagged branch of a split the trunk lanes whose `turn:lanes` bear toward it (95 % of tagged branches agree), instead of
  the motorway class default of 2. LA: a ramp tagged `lanes=1` no longer has 2 lanes on its untagged pieces; a `lanes=3`
  ramp no longer 4; a split ramp 2 -> 3 lanes instead of 2 -> 4. 17 pieces in the six areas; Tokyo overlaps 3 -> 2.

### Added
- **Road barriers** (standard, OpenDRIVE 1.8.0 §13.14.1 Table 108): OSM guard rails, jersey barriers and cable
  barriers become `<object type="barrier">` (guardRail / jerseyBarrier / other) on the plain road they line, sized
  by continuous `<repeat distance="0">` records that follow the OSM line to within 0.05 m (§13.4) and sit on the
  ground to within 0.005 m (cross slope and lane height added to zOffset; the barrier breaks where the ground
  steps). A barrier stays off lanes in use of its road and of every other road on the same level (dropped
  there) and at most 5 m beyond the road edge; the OSM layer prefers a road but does not exclude one. Sizes are
  assumed typical values (OSM carries none). New `odr::ObjectRepeat` (writer, reader), validator rule 1.9.0
  `road.object.repeating.valid_s_length`; object@subtype is written from 1.6 on only (not in XSD 1.4H). Frankfurt
  Kreuz: 226 barriers, 27.7 km; other areas and carla unchanged. Drawn barriers on the 300 fuzz networks and
  beside 401 superelevated LA roads audit clean. `barrier_audit.py` with a planted-defect self-test.
- **Pedestrian lights on junction cross paths** (standard): where an OSM crossing at the arm's end is
  signalised (crossing=traffic_signals, crossing:signals=yes, UK signalised crossing_ref), the crossing road gets
  per walking direction an invisible stop line (type 1100001, semantics waitingLine) at the kerb pedestrians start
  from and a pedestrian light (type 1000002) across the road that it depends on and that references it, plus one
  controller per crossing listed in the junction (Junction Guideline §6.3.3 Codes 20 - 23; Signal reference 1.0.0
  types). No `<validity>` (1.8.0 §14.2 default; road.signal.validity.*_hand_traffic_lane_ids). Crossing roads'
  walking lanes are bi-directional (`direction="both"`, 1.8.0 §11.2). LA 16, Munich 1, Tokyo 47 crossings; carla
  identical. `ped_signal_audit.py` with a planted-defect self-test (12/12 in LA and Tokyo). Jev 0.86 / 0.80.
- **Junction boundary** (standard, OpenDRIVE 1.8.0 §12.10): a counter-clockwise `<boundary>` for common junctions
  with an elevation grid - a joint across every arm end, corners along an existing lane edge, else a helper road
  (`JunctionBoundary`, centre lane only, the junction's id) copying the connecting road between the two arms along
  its outer edge, else a corner curve. Written only when closed, simple, fold-free and around every lane border
  of the junction's roads (helpers pushed out past lanes beyond them); otherwise none (optional element). Six
  areas: 2087 of 2463 grid junctions; 0 audit failures, XSD 1.8.1 valid, carla identical; 300 fuzz networks: 185
  of 216, 0 failures. The validator exempts lane-less boundary roads from `lane_section_drivable` (1.9.0
  `lane_sect_first`). Jev 0.85.
- **`boundary_audit.py`** with a planted-defect self-test (`boundary_audit_selftest.py`): checks every written
  junction boundary against the §12.10 rules (closed, counter-clockwise, a joint per connected road end, all lanes
  enclosed, simple, no fold, helper road id / linkage / direction) and the §12.9 / §12.11 rules (reference line
  crosses it, grid covers it); `grid_audit.py` also compares helper surfaces with the grid. 12 planted defects
  detected at the planted junction and nowhere else in Munich, LA and Tokyo. Jev 0.81.
- **Junction boundaries in the viewer**: the standard pages carry the written boundaries (curvature-adaptive
  samples; within 0.17 m of the written outline, p99 0.06 m), drawn as a dashed layer; the rest of every page
  is byte-identical to before. Jev 0.87.
- **Review flags in the viewer**: press M (or ⚑) and click the map to flag something wrong, with a category
  and a note; the flag records the OpenDRIVE x / y and the road, lane, junction and s under it.
  `tools/inspect/viz_server.py` serves the pages and keeps each page's flags in `<page>.flags.json`;
  `tools/inspect/flags.py` lists them and writes a status (fixed / question / wontfix) and a reply, shown
  back in the viewer. Without the server, flags stay in the browser (Copy JSON). The viewer also draws
  junction boundaries when the data carries them. Jev 0.80.

### Fixed
- **laneflow: lane-less roads are no road surface** (`osm.way_dropped`): a junction boundary helper road's line
  had hidden one LA finding (3 -> 2); now 3 again. Unchanged on 12 outputs without lane-less roads; new plant
  `road_del_laneless` in `laneflow_plant.py` fails with the old version, passes with the new. Jev 0.88.
- **No second marking for a crossing at a junction mouth** (standard): an OSM crossing node within 3 m of an
  approach end whose crosswalk is a junction cross path is that cross path - it no longer also becomes an
  approach crosswalk (and a mid-block cross path). LA 16, Tokyo 83 duplicates removed, each within 4.3 m of
  its junction cross path. Jev 0.84.
- **Pitch kinks at junction entries** (OpenDRIVE 1.9 Annex D.1.5 "should have no vertical kinks"): a road's
  roll rate at a junction plane is now the plane's analytic rate along it (as on the connecting roads), not
  a finite difference after the blend. LA lane links with a pitch break > 0.01: 2655 -> 23 (worst 0.241 ->
  0.109); other areas have no superelevation. `annex_d_vertical.py` measures D.1.5 pitch now (folded edges
  skipped). Jev 0.84.
- **Traffic lights at their physical location** (Junction Guideline §9.2) in the standard profile: zOffset
  2.42 m, the guideline examples' value for pole-mounted type 1000001 heads (was 0; carla keeps 0, CARLA
  spawns poles). Jev 0.83.
- **Signal semantics** (OpenDRIVE 1.8.0 §14.8, XSD 1.8.1 order): `<semantics><priority>` stopLine /
  trafficLight / stop / yield, as in Junction Guideline §6.3.3 Codes 13 / 14 / 20; 1.8+ only. Jev 0.80.
- **Spec atlas corrections**: curb since 1.6.0 (not 1.4), connectingRamp not in 1.4H, the double road-mark
  side rule (1.8.0 Table 167), and the signal-dependency wording that is already 1.8.0 §14.3. Jev 0.81.
- **Header licence** (OpenDRIVE 1.8.0 §6.4.2 Table 9: `text` is "The full license text"): `<license>` keeps
  name / resource / spdxid; the ODbL attribution "Contains data © OpenStreetMap contributors" moved to header
  `<userData code="attribution">`, now in both profiles (1.4H header allows userData). Jev 0.85.
- **Lowered-curb height records** carry a hold record 1 cm before each change: same profile under 1.8.0
  §11.6 (a record holds until the next) and for 1.4 readers that interpolate (1.4H §5.3.7.2.1.1.9 gives
  heights "at discrete positions"; Junction Guideline §6.3.2 writes hold records). Jev 0.83.
- **Code citations**: 21 comments citing the internal notes ("spec 3", "spec 6.1, pitfalls") and two wrong
  sections now cite OpenDRIVE 1.8.0 sections, verified against the local spec; 1.9-only rule ids marked;
  an unsupported "spec: avoid geometries below 1 mm" is now stated as our own guard. Jev 0.85.
- **`exact_diff.py`** ran the checkers on relative paths from another working directory: after cache
  deletion both sides failed and compared "unchanged", invalidating regression claims made earlier today.
  Fixed (absolute paths, a failed checker reported as such); re-verified against c6818fb rebuilt in a
  worktree: carla unchanged, standard differences only at the two signalised merges. Jev 0.89.
- **Motorway lane types follow OpenDRIVE 1.8.0 §11.7.1 and Junction Guideline §7** (spec review found
  ramp types on tertiary / service / primary roads, motorway-to-motorway links typed offRamp / onRamp,
  entry / exit lanes on ramps, never-merging lanes typed entry):
  - every one-way motorway_link road is typed by what lies beyond it through link roads in travel direction
    (a street junction ends the search): motorway both sides connectingRamp, upstream offRamp, downstream
    onRamp, neither (data edge) driving;
  - entry / exit lanes only on motorway / trunk main roads; entry only where the lane narrows to zero (a lane
    gain stays driving, a weaving lane is typed by its exit);
  - the validator's road_overlap check counted only `driving` lanes, so retyped ramps escaped it; it now
    counts every vehicle lane type. Jev 0.85.
- **Traffic-light control follows OpenDRIVE 1.8.0 and the Junction Guideline** (multi-source review):
  - 1.8.0 §12.1 Table 54 (junctions with traffic lights are common): signal control is decided before the
    direct junctions; a signalised merge stays a common junction (Munich J247, London J86 had lights and
    controllers in direct junctions);
  - 1.8.0 §14.1: approaches whose lanes are onRamp / offRamp / entry / exit / connectingRamp now get their
    light (LA 2, Munich 1, London 2 were uncontrolled);
  - Junction Guideline §6.3.3: stop lines lay on crosswalks (LA 52, Munich 26, London 58): crosswalks are now
    built first and each stop line is pushed 1 m clear upstream of every crosswalk between it and the
    junction; an approach too short for that gets no stop line and its light keeps the lane validity
    (1.8.0 §14.4 form). New `tools/inspect/signal_rules_audit.py`. Jev 0.82.
- **Cross paths follow OpenDRIVE 1.8.0 and the Junction Guideline** (checked against 1.8.0 §12.1 / §12.5 /
  §12.7, XSD 1.8.1, Junction Guideline 1.0.0 §6.3.3, §6.4.3, §10.3.3 and the spec atlas):
  - crosswalks on junction approaches are no longer virtual junctions on the approach road: the guideline
    puts crossings at a junction inside the common junction's area (`f761a2a` reverted; their crosswalk
    objects stay; modelling them inside the junction is open);
  - "Currently only flat virtual junctions can be modeled" (1.8.0 §12.7): a mid-block cross path only where
    the road surface over the junction and across the crossing varies by at most 0.02 m (LA, whose roads
    carry superelevation, keeps none; Munich 76, London 37, Frankfurt Kreuz 11, Stelvio 4, Tokyo 315);
  - guideline §10.3.3: `<priority high=road for general traffic low=crossing road>`, sStart / sEnd 0.5 m
    clear of the crossing. Jev 0.80 (reading "flat" along s only: 0.43); revert 0.88.
- **Cross paths now hold the rules of OpenDRIVE 1.9 §12.5 / §12.7.1** (rule ids of 1.8.0): measured, the
  first version broke junctions.cross_path.start_end_contained (crossing road ends on the sidewalk's
  border), junctions.cross_path.lane_linkage (ends up to 0.16 m outside the sidewalk on curves) and
  junctions.cross_path.within_junction_area (footprint beyond sStart..sEnd). The crossing road now runs
  sidewalk centre to sidewalk centre, its footprint checked inside the linked sidewalk lanes, sStart..sEnd
  spanning the footprint; a crosswalk where that cannot hold keeps only its object (London 2). New
  `tools/inspect/crosspath_audit.py` measures the rules: 0 failures in all six areas. Jev 0.88.
- **`tools/inspect/junction_rules_audit.py`** measures rules no other checker covered (OpenDRIVE 1.9
  §12.2, §12.6, §12.7, §11.7): junctions.direct.road_connectivity / split_or_merge / road_ramp_heading,
  overlapZone and virtual-junction attributes only on their junction types, junctions.virtual.
  no_controllers, sStart..sEnd inside the main road, road.lane.height.elem_asc_order. All six areas: 0
  failures (direct junction headings equal to 1e-8 rad); each rule verified to flag a planted violation.

### Added
- **Crosswalks at junctions as cross paths inside the junction** (standard profile; Junction Guideline 1.0.0
  §6.3.3 "Add roads for the pedestrian crossings inside of the junction area", §6.4.3 "Choose the road
  reference lines to stop before any cross path in the junction"; OpenDRIVE 1.8.0 §12.5 Figure 85): an
  approach of a signalised / crossing junction ends 3.1 m earlier where a corner sidewalk will exist on both
  sides and its lanes stay the same; a crossing road of the junction runs across the arm's mouth between the
  corner sidewalks' walking lanes (3 m wide, 2.5 / 2 m in tight corners), carries the crosswalk marking, and
  the corner sidewalks are lowered over it; elsewhere the crosswalk stays on the approach. LA 28, Munich 13,
  Frankfurt Kreuz 1, Tokyo 164 junction cross paths; no crosswalk lost; carla unchanged. Pedestrian lights on
  the crossings are not modelled (OSM has no pedestrian phases). Jev 0.82.
- **Junction reference line and elevation grid** (standard profile, OpenDRIVE 1.8.0 §12.9 / §12.11, common
  junctions): a straight `<planView>` line along the principal axis of the connecting roads, reaching every
  lane by a perpendicular, and an `<elevationGrid>` of 5 m squares covering them, sampled from the junction
  plane the connecting roads lie on (bicubic interpolation of planar values is exact, so the grid equals
  their heights: max 8.9 mm). No boundary yet. `tools/inspect/grid_audit.py` measures the rules; 0 failures
  in all six areas. Heights unchanged: LA's steep short roads are real walls / garage ramps in the terrain
  data, which a junction surface cannot fix. Jev 0.76; committed on the user's decision.
- **Junction approach crosswalks as cross paths** (standard profile, OpenDRIVE 1.9 §12.5 / §12.7): the
  crosswalks on the approaches of signalised and crossing junctions lie on the approach road, short of
  the junction's connecting roads, so each gets a virtual junction on that road round the crosswalk,
  holding its cross path - the same scheme as mid-block crossings. LA 104, Munich 52, London 23,
  Frankfurt Kreuz 20, Tokyo 291. Kept with Jev 0.80 (a cross path of the intersection junction itself
  scored 0.29: its crossing road would lie outside that junction's area).
- **Mid-block crossings as cross paths** (standard profile, OpenDRIVE 1.9 §12.5 cross paths, introduced
  1.8.0, in a §12.7 virtual junction; Junction Guideline §10.3): an OSM crossing node on a road with a
  sidewalk on both sides becomes a virtual junction on the uncut road (mainRoad, sStart..sEnd round the
  crosswalk) holding a crossPath - a one-lane walking road straight across between the sidewalks' inner
  edges, linked to the outermost sidewalk on each side. The crosswalk object stays. Writers for
  revisions before 1.8 leave such junctions and their crossing roads out; carla output unchanged. The
  validator and invariants checker accept crossing roads (linked by lane links, not road links). LA 83,
  Munich 76, London 39, Frankfurt Kreuz 19, Tokyo 342, Stelvio 4. Committed on the user's decision.
- **Lowered curbs at crosswalks** (both profiles, OpenDRIVE 1.9 §11.8 lane height records): over each
  crosswalk the curb lane and the sidewalk's inner edge drop to 0.02 m, the sidewalk ramping down to
  the crossing. Jev 0.75; committed on the user's decision.
- **Curb lanes** (standard profile, OpenDRIVE 1.9 §11.8 lane type curb, introduced 1.6): every raised
  sidewalk - also round junction corners - gets a 0.15 m curb lane at kerb height, taken from the
  sidewalk so its outer edge stays; lane links and junction laneLinks carried over; revisions before 1.6
  write border (and driving for connectingRamp). New checker findings only duplicate places already
  flagged for the sidewalk. Jev 0.78; committed on the user's decision.
- **Ramp lane types** (standard profile, Junction Guideline §7.2.2.3, OpenDRIVE 1.9 §11.8): at direct
  junctions a ramp's lanes are offRamp, onRamp or connectingRamp (motorway to motorway), the trunk
  lanes leading into an exit are exit, those an entry becomes are entry. carla keeps driving (CARLA's
  default lane filter). Kept with Jev 0.81.
- **Direct junctions at freeway splits and merges** (standard profile, OpenDRIVE 1.9 §12.6, Junction
  Guideline §7.2): each gore's 4 m connecting road is joined into the ramp / main road it leads to or
  comes from, the arm's unreached lanes (shoulders, sidewalks) opening over >= 3 m per m of width, and
  the junction becomes type="direct". 54 of 113 gores (an arm reached by several connecting roads, a
  fold risk for carried lanes, or an unexpected road end keep the common junction). The carla (1.4)
  profile is unchanged. Jev 0.79; committed on the user's decision.
- **Header licence** (standard profile, OpenDRIVE 1.9 §6.4.2, introduced 1.8.0): the ODbL 1.0 licence
  of the OpenStreetMap data with its attribution. Kept with Jev 0.89.
- **Stop lines** (standard profile, ASAM Junction Guideline §9.2, OpenDRIVE 1.9 §14.3 / §14.4): every
  traffic light has a stop line on its approach - OpenDRIVE type 1100001, valid for the incoming
  lanes, 1 m before the approach crosswalk, dependent on the light - and references it; the light
  carries no lane validity of its own. The carla (1.4) profile, which has no signal references, keeps
  lights with validity. Kept with Jev 0.84.
- `tools/inspect/annex_d_vertical.py`: OpenDRIVE 1.9 Annex D.1.4 vertical gaps - z and roll of the
  touching points at every lane link (road links and junction lane links). Detects a planted 0.2 m
  step and a planted 0.02 rad roll; the six areas have none (LA 14,538 links checked).
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
- **Direct junctions in the checkers** (OpenDRIVE 1.9 §12.6): the exact network checker and the
  validator now follow `<junction type="direct">` (the connection's `@linkedRoad`; lanes compared at
  the cut). Maps without direct junctions check byte-identically; a test conversion with direct
  junctions lost all its false topology / flow findings. Kept with Jev 0.90.
- **Signal phases by traffic axis**: a phase (controller) holds an approach, the one opposite it and
  the approaches running parallel to either (a second carriageway, a slip road); before, parallel
  approaches waited in separate phases (23 junctions). No phase mixes crossing directions. Kept
  with Jev 0.86.
- **Lights on approaches OSM does not signalise**: a junction signalised through per-approach light
  nodes gave every approach a light. An approach without a light of its own now keeps only the stop /
  give-way sign OSM gives it (it had a light and the sign), a service road or living street gets a
  give-way sign instead (driveways, alleys, parking exits); other unmapped streets keep a light.
  Lights 1,518 -> 1,350 over five areas, give-way signs +129. No junction <priority> records: the
  signs carry the priority (1.9 §12.4, junctions.priority.no_signals). Kept with Jev 0.86.
- **Lane lines along tapers** (OpenDRIVE 1.9 §11.9, sOffset records): the line between a lane being
  added (or ending) and the lane it opens from was drawn along the whole taper, on top of the edge
  or center line where the new lane is centimetres wide. It now starts where the lane is fully open
  (ends where it starts closing). LA 208, Munich 45, FK 29, London 20, Tokyo 9 lanes. Kept with Jev 0.87.
- **Lane-edge kinks at junctions** (OpenDRIVE 1.9 Annex D.1.3): overlap resolution rebuilt an arm's
  laneOffset with a one-sided slope at its ends, so a push still ramping there left a sloped offset;
  on a curve each lane edge then turned by a different angle the connecting road could not follow.
  The push is now level at the road ends. Lane-edge heading breaks 263 -> 0 over the six areas (FK
  127, LA 66, Tokyo 36, Munich 29, London 5). Kept with Jev 0.88.
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
