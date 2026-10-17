# Progress and open decisions

Branch `geometry-quality` (pushed). Plan: `docs/reference/spec_atlas.plan.html`. Rule coverage:
`docs/RULES.md`. Every step was checked on the six test areas (LA, Munich, London, Frankfurter Kreuz,
Stelvio, Tokyo) with the exact checkers (`tools/inspect/exact_diff.py`), the junction audit, the
Annex F checks, the 1.4H and 1.8.1 XSD, esmini and CARLA's parser.

## Done (2026-10-12 / 13)

| Phase | What | Effect on the six areas |
|---|---|---|
| 0.1 | Two output profiles: `standard` (OpenDRIVE 1.8), `carla` (1.4); `SpecRevision` names each versioned feature | - |
| 0.2 | Rule catalogue from 1.9 Annex F; validator ids = Annex F paths | 116 of 274 rules checked, 144 not applicable, 14 open |
| 0.4 | Own Annex F checker (57 rules), official ASAM qc-opendrive, 1.4H XSD test | carla files valid 1.4 (except the LHT rule, see below) |
| 1.1 | One junction per intersection: squeezed junctions joined, restrictions / U-turns through merged junctions | FK edge pairs lost 129 -> 1, lost flows 43 -> 1 |
| 1.2 | Connecting roads continue the arm reference lines (or lane-centred on tight turns) | 77 - 90 % on the arm lines; lane-edge heading breaks halved |
| 1.3 | Sidewalks continue round junction corners; curb-return arm cuts; fillet curbs; noses at acute corners | 1,661 corners continued, 29 too tight, 170 noses |
| 1.4 | Crosswalk objects at OSM crossings and on signalised arms, outside the curb return (already existed) | - |
| 1.7 | Roundabout junction groups | - |
| 3.1 | Overlap shifts level at road ends (Annex D.1.3 no kinks) | lane-edge heading breaks 263 -> 0 |
| 3.2 | Annex D.1.4 vertical check (`annex_d_vertical.py`): z and roll at every lane link | 0 gaps, 0 roll mismatches in all six areas |
| 2.4 | Lane lines start / end with the taper (sOffset mark records) | 311 lanes over five areas |
| 1.3c | Lowered curbs at crosswalks (both profiles; Jev 0.75, user decision) | Tokyo 986 / LA 264 crosswalk sides |
| 1.3d | Mid-block crossings as cross paths in virtual junctions (standard, 1.8+; user decision) | LA 83 / Tokyo 342 cross paths |
| 1.3e | Junction approach crosswalks as cross paths, virtual junction on the approach (standard; Jev 0.80) | LA 104 / Tokyo 291 cross paths |
| 1.3f | Cross paths fixed to the §12.5 / §12.7.1 rules, `crosspath_audit.py` (Jev 0.88) | 0 rule failures, 6 areas |
| 1.3g | Cross paths to 1.8.0 + Junction Guideline: flat only, priority, approach ones reverted (Jev 0.80 / 0.88) | Tokyo 315, LA 0 (banked) |
| 1.3h | Crosswalks at junctions as cross paths inside the junction (standard; Jev 0.82) | LA 28 / Tokyo 164 junction cross paths |
| 1.7 | Junction reference line + elevation grid from the junction plane (standard; Jev 0.76, user decision) | 2463 junctions, 0 audit failures |
| 1.7b | Junction boundary (standard, 1.8.0 §12.10; Jev 0.85): joints, lane edges, helper roads, self-checked | 2087 of 2463 junctions, 0 failures in two independent audits |
| 1.3i | Pedestrian lights on junction cross paths OSM signalises (Guideline §6.3.3 Codes 20 - 23; Jev 0.86), `ped_signal_audit.py` + self-test (0.80) | LA 16, Munich 1, Tokyo 47 crossings; 0 audit failures |
| 1.8 | Road barriers from OSM guard rails / jersey / cable barriers (1.8.0 §13.14.1, continuous repeats; standard), `barrier_audit.py` + self-test | Frankfurt Kreuz 226 barriers, 27.7 km; 300 fuzz + LA drawn barriers; 0 audit failures |
| 4.4 | Signal control fixes from the spec review: no lights in direct junctions, ramp approaches lit, stop lines clear of crosswalks (Jev 0.82) | 0 audit failures |
| 1.5b | Motorway lane types re-derived per 1.8.0 §11.7.1 / Guideline §7; validator overlap over all vehicle lanes (Jev 0.85) | ramp types only on motorway_link |
| 1.3b | Curb lanes before every sidewalk, round corners (standard; Jev 0.78, user decision) | Tokyo 2,841, Munich 793, LA 464 |
| 1.5b | Ramp lane types onRamp / offRamp / connectingRamp / entry / exit (standard) | FK 108 lanes, LA 58 |
| 1.5 | Direct junctions at freeway gores (standard; committed at Jev 0.79 by user decision) | 54 of 113 gores |
| 4.4 | Header `<license>` ODbL-1.0 with OSM attribution (standard) | - |
| 4.3 | Signal phases by traffic axis (parallel approaches share a phase) | split parallel approaches 23 -> 0 junctions |
| 4.2 | Lights only on approaches OSM signalises; driveways yield; own signs not doubled with a light | lights 1,518 -> 1,350, give-way +129 |
| 4.1 | Stop lines with light -> stop line reference and stop line -> light dependency (standard) | 1,518 stop lines over five areas |
| 2.3 | Road mark laneChange from the final mark type (double lines crossable from the broken side) | LA trapped flows 1 -> 0, missing-link islands 17 -> 2 |
| 6 | Stage timing; arm trimming in parallel | LA trimming 9.4 s -> 0.9 s per pass |
| tools | `jev_judge.py` (Jev reviews whole junctions, calibrated against planted defects) | Jev trusted on spec validity only |
| tools | Review flags in the viewer (mark what is wrong, `viz_server.py` keeps them, `flags.py` answers; Jev 0.80) | closed review loop with the user |

## Decisions (2026-10-13)

Pushed; keep `rule="LHT"` in carla files; separate sidewalks next; 1.8-only features medium
priority; Jev consulted for every decision; kerb lines used where mapped.

## Needs your input (answered above)

1. **Push** `geometry-quality` to GitHub? Nothing has been pushed.
2. **LHT in 1.4 files**: the carla profile writes `rule="LHT"` although OpenDRIVE 1.4 has no such
   attribute (schema error in strict validators); without it readers drive London / Tokyo lanes against
   traffic. Keep this deviation, or drop LHT from carla files?
3. **Separate sidewalks** (`--separate-sidewalks`, off by default): extends 2,131 road sides in LA and
   1,114 in London to their mapped sidewalks, but raises folds (LA 69 -> 100) and lane-edge breaks until
   verge widths blend across way segments and stay fold-safe in tight bends. Worth finishing next?
4. **Standard-profile-only features** that CARLA ignores (crossPath walking roads, direct junctions with
   overlapZone for ramps, junction boundary / elevation grid): how much priority versus CARLA-visible
   work?
5. **Jev**: on whole-junction XML it separates broken from correct junctions only on spec validity
   (0.71); its sidewalk / geometry / lane-path verdicts do not. Keep using it that way, or feed it
   something else (rendered images are not supported - Jev is text only)?
6. **Kerb lines** (OSM `barrier=kerb`, 705 in London, almost none elsewhere): use them to place
   carriageway edges where mapped?

## Known open items

- Junction boundary (committed, Jev 0.85): left out where the outline fails its checks - 376 of 2463 grid
  junctions in the six areas (helper crosses a joint 133, no corner curve or radius < 0.25 m 87, lanes beyond
  an arm's joint 81, other 75), 31 of 216 in 300 fuzz networks (all 15 motorway gores). Acute arms (30 deg)
  whose ends overlap come from the arm trimming, not the boundary. G2_ref_junction +45 - +1078 per area,
  all on helper roads (76 - 92 % repeat the connecting road's own step). The boundary tools followed once
  each had evidence of its own: `boundary_audit.py` + planted-defect self-test (Jev 0.24 -> 0.81), the laneflow
  lane-less-road fix + plant (0.43 -> 0.88), the viewer's boundary layer + drawn = written check (0.47 -> 0.87).

- Folds on loop streets kept as roads (Munich 6), lane-edge breaks LA +6 since phase 1.1.
- Munich `pairs.missing` 118 is two boundary artefacts counted per pair: the entry on road 743 (a 2 m
  stub cut by the clip box at c70, junction 315) reaches no exit, and the exit at node 7832370184 (a
  driveway ending on a highway=path) is reached by no entry.
- Separate sidewalks: the lane-edge position breaks (London 11) do not come from the verge appearing
  / disappearing between segments (keeping a 5 cm verge everywhere changed nothing); next suspect is
  the sidewalk distance changing between segments at plain road-to-road links without a blend.
- Kerb lines: within a lane section the verge width is constant while the kerb distance varies
  (median p10-p90 spread 0.60 m, London). Width records varying along the section would follow it;
  Jev 0.52 (not decided), parked at medium priority.
- Separate sidewalks, lane edges that start/end at section boundaries (LA 36 with the option vs 13
  without): mostly roads tagged sidewalk:both=separate where one segment's measurement was rejected
  or found nothing. Tried (diff parked in build/tmp/sidewalk_continuity_parked.diff): rejected
  measurement counts as none so continuity applies; verge-less sections take the nearest measured
  verge; sections differing only in verge widths merge and are re-measured. LA 36 -> 24, London
  13 -> 12, Munich 15 -> 13, moved curbs closer to OSM kerb lines (within 0.2 m 18.6 % -> 35.9 %),
  folds London 31 -> 29; costs London lane_count +1, Munich G2 +4. Jev 0.68 (parts: 0.50 / 0.69) -
  below 0.8, not committed. Needs your view.
- Edge line next to a shoulder off motorways (driving lane edge and a one-way's left edge): solid per
  MUTCD 3B.06 / StVO Z.295; changes one place in the six areas (Munich Frauenstrasse); Jev 0.74 - not
  committed.
- CARLA height steps LA (131): almost all on 5 garage-entrance service tunnels a few metres long
  that drop ~6 m (35-55 % grade) into underground car parks; continuous (Annex D holds), but steeper
  than the loader's 25 % step test. Spreading the drop needs the garage aisles beyond the portal.
- Objects on banked roads need no z correction: 1.9 §8.3/8.4 rotate the t-axis by superelevation and
  objects sit in that frame (Jev 0.91).
- Slip lanes (Junction Guideline §8): 149 OSM slip lanes in the six areas (`build/tmp/slip_find.py`);
  103 are their own roads between small junctions, 46 short ones are absorbed into a joined junction.
  Dropping the slip's turn from the central junction (triangle test, slip bending >= 20 deg) removes
  11 repeated relations but some matches are doubtful (a -5 deg service-road "turn", a street onto
  itself); Jev 0.70 - parked (`build/tmp/slip_relation_parked.diff`). slipLane lane type, entry/exit
  lanes and direct junctions with overlapZone are 1.8-only (medium priority), not started.
- Direct junctions (1.5): 59 of 113 gores stay common junctions (28 arm reached by several connecting
  roads, 28 fold risk for carried lanes, 3 unexpected end); 5 OSM-distance findings on the joined first
  metres.
- Curb lanes: curb lanes duplicate the
  sidewalk's existing fold / corner / edge-break findings (Stelvio folds 98 -> 140, same places).
- Pre-existing plain-road overlaps: Munich 2, Tokyo 3 (`quality.road_overlap`).
- Steep short roads (LA Bunker Hill: 2 m roads climbing 3.4 m, CARLA loader 131 height jumps): each junction
  is one near-flat plane while the measured terrain rises 3 - 4 m across a 12 m junction area, so the climb
  lands on the short road between two planes. Limiting the level difference between planes (road length x
  maxGrade) cut CARLA's jumps 131 -> 16 but moved measured street junctions by up to 2.5 m (Jev 0.75): parked
  in build/tmp/plane_grade_parked.diff. The faithful fix is the junction surface item (elevation grid with
  transition lengths, 1.9 §12.9 - 12.11): a decision for the user.
- Direct junctions: exit lanes typed from the road start, not where the lane begins (no spec rule);
  motorway_link roads whose origin lies outside the area keep driving. (Two "ramp -> shoulder" lane links
  reported earlier were an artifact of the counting script - it took lane types from the wrong section.)
- Free road ends after a junction-plane refit: the refit's smoother extrapolates the last slope beyond the
  last measurement instead of 236aa76's 20 m least-squares grade. Applying the extension there too changed 18
  LA roads (max 0.42 m) but a hold-out on 672 road ends (last 25 m hidden) did not show it more accurate
  (worse in 429; the test's truth is our smoothed output, raw samples are not in the output) - Jev 0.77,
  parked in build/tmp/end_extension_refit_parked.diff.
- Not changed, with reason: redundant lane-section boundaries where only laneOffset differs (LA 218 / 275 /
  298) break no verified rule (1.8.0 §11.3 requires a new section only when the lane count changes; the
  atlas' 1.8.1 performance.avoid_redundant_info cannot be checked locally); bus lanes as access deny vs the
  1.8.0 example's allow bus - no test area has a bus lane (0 <access> in the outputs).
- Official checker bugs: `road.geometry.contact_point` skips only junction id 1; crashes on roads
  without `<link>`.
- Overlap resolution (2 x 5 s on LA) is an order-dependent sweep; making it parallel needs a redesign.
- CARLA height steps on LA (LibCarla ignores superelevation) and `jumps2.py` sampling every third
  waypoint (counts shift when roads are added).
- Gap-closing plan (user, 2026-10-14), worked one by one in this order; each with measured before / after
  evidence on all six areas, both profiles, Jev >= 0.8:
  1. Official ASAM checker (asam-qc-opendrive) added to the checks; its findings triaged and fixed.
  2. Same-level road overlaps (quality.road_overlap: Munich 2, Tokyo 3) fixed.
  3. Review flags: Munich flag 7 rework (the corner sidewalk built along the adjacent driving lane's edge,
     no wedge; the straight-on curve rule is uncommitted); LA flags 1 - 5 (lanes added before ramp splits
     against OSM lanes=, a ramp with 2 lanes where OSM says 1, an unmodelled restricted area, a road meeting
     South Figueroa not connected).
  4. Spec coverage: the 279 rule ids of 1.9 (174 listed as not mechanically checked, list partly stale)
     reviewed one by one: not applicable / checked by an existing tool / check added.
  5. Design choices that are not spec text, each justified with evidence or replaced by OSM / measured data:
     the 0.25 m Douglas-Peucker thinning of OSM geometry and the corner fillets (fit to OSM nodes: p95
     0.9 - 1.9 m, max up to 10 m); minimum road radii per class (15 m primary, 60 m motorway); curb radius 6 m;
     the 35 degree straight-on threshold; barrier sizes.
  6. Junction Guideline 1.0.0 completed: every section checked against the implementation, the missing
     parts implemented. Known gap (ASAM checker, 2026-10-15): connecting roads start their reference line
     at the lane they serve (gap to the incoming reference line p50 1.5 - 1.95 m; Munich 1546, London 865,
     Frankfurt 776 road.geometry.contact_point reports, all on connecting roads); the guideline's T example
     starts them on the incoming reference line with laneOffset a = the width of the skipped lanes.
  Status 2026-10-15: 1. checker triaged - contact_point wrapper fixed, redundant lines fixed (4257157),
  LHT corner sidewalks mirrored (F.4.6.6), road@rule in 1.4H deliberate (CARLA reads it); LA and Tokyo
  standard checked 2026-10-15 (current outputs): Tokyo only 3125 contact_point (connecting roads); LA 6354
  contact_point (connecting roads), 16 performance.avoid_redundant_info and 36
  lane_smoothness.contact_point_no_horizontal_gaps - both checked as checker false positives: the 16 records
  differ by up to 7.9e-5 rad / 0.4 mm over their span (the checker compares raw coefficients to 1e-6 whatever the
  span; the writer keeps a record unless the previous one reproduces it within 1e-5 rad / 1e-6 m), and the 36
  lane pairs meet within 5 mm (median 0) at their contact points (checker limit 1 cm). Overlaps: Tokyo 734 / 977
  fixed (c0bbebd); laneOffset shifts G2 (a217a32). 3. Munich flag 7 fixed (86cec86); LA flag 4 re-measured, not a gore
  (question to the user: ramp shoulders); LA flag 5 waits for the user.
- Real terrain for the test areas (2026-10-17): ground grids `build/testareas/<area>_ground.odrz` (make_terrain.py,
  local TM frame at each area's origin, 60 m margin), bridge decks `<area>_decks.odrz` from a surface model near OSM
  road bridges (make_terrain.py --dsm --deck-osm). Sources (data in `C:\mywork\osm2carla\data\dem`):
  | area | ground (bare earth) | decks | cell | vertical datum |
  |---|---|---|---|---|
  | munich | Bavaria DGM1 1 m, 6 tiles (open data) | - | 0.5 m | DHHN2016 |
  | frankfurt_kreuz | Hesse DGM1 1 m (INSPIRE WCS he_dgm1) | Hesse DOM1 1 m (WCS dom1) | 0.5 m | DHHN2016 |
  | london | EA LIDAR Composite DTM 1 m (WCS) | - | 0.5 m | ODN |
  | tokyo | GSI DEM5A 5 m (tile service, zoom 15) | none open | 1 m | JGD2011 (T.P.) |
  | stelvio | South Tyrol DTM 2.5 m (WCS, CC0; 54 % of the box) + Copernicus GLO-30 for the Lombardy part (own heights beyond a 100 m seam) | - | 1 m | Genova1942+EGM2008 (no single EPSG code) |
  gate.py builds munich, london and stelvio on terrain. Frankfurter Kreuz with ground + decks validates clean (0 errors,
  7 grade warnings vs 31 errors with ground alone) but a three-level underground car park (layers -1 .. -3, maxheight
  2.1 m, 50 roads) connects to no measured road and keeps height 0 - open: anchor such a network through the roads it
  crosses (a first attempt left decks 2.9 m under their access tunnel, inside the validator's 3 m same-level band).
  Tokyo stays relative (its stacked Shuto flyovers would need deck data: 8 errors with ground alone). Elevation over measured terrain now runs the
  relative-mode level solver too and uses its levels as offsets in unmeasured stretches (elevation.cpp 4.).
- Audit of the Stelvio DEM change (2026-10-17, user request: scores earned, measured facts only): the 5ef8b71 brief
  carried an unmeasured explanation (OSM line ~2 m off the bench, retracted) and the fill's median shift (+1.26 m) was
  shown wrong by OSM spot heights (Lombardy fill +1.32 m vs +0.06 m without it). Corrected: the fill keeps its own heights
  unless the difference is a consistent offset (|median| > IQR), and the grid is labelled with the datums it holds.
  Jev re-score of the corrected change with a fully transparent brief: 0.79 (below 0.8); committed by the user's
  decision. Open: finer Lombardy data (TINITALY 10 m, INGV, CC BY 4.0 - its server refused connections on 2026-10-17).
- Frankfurter Kreuz on terrain (2026-10-17): the isolated car park P15 is anchored under the lowest measured surface
  over it; it lands too deep in places (median 11-12 m, max 55 m) and one point is 0.38 m above ground. Rework: anchor
  locally (per junction cluster / road) instead of one level 0 per network. A verification pitfall found the same day:
  restoring a source with a timestamp-preserving copy left a mutated osm2xodr.exe in place (ninja saw nothing newer) -
  always touch restored files; the per-run metrics history showed the difference.
- Terrain contract (2026-10-17): docs/TERRAIN_CONTRACT.md holds the proven terrain numbers as absolute gate limits
  with a planted-defect test per check (user rule: harden every proven improvement). The gate baseline itself
  (gate_baseline.json, 2026-10-12) is stale: its deltas (G2 invariants, JCT-8, max_grade from real terrain) come from
  commits reviewed one by one with exact_diff; refreshing it needs the user's go-ahead.
- Remaining plan items, not started. The design-level Jev scores below (2026-10-13) are NOT reliable: the
  briefs held untested claims and no spec / guideline text. Each item is to be re-assessed after reading the
  atlas, 1.8.0, 1.9, the Junction Guideline and the XSDs, with measured evidence:
  - walls and fences as barrier objects (the road barrier stage, 1.8, could carry them; most OSM walls and
    fences bound properties, not roads): 0.35;
  - barriers: samples on lanes in use are dropped (Frankfurt 1,806 of 31,400, a median 0.7 m deep - OSM line
    vs estimated lane widths); moving them to the lane edge instead is a user decision. Barriers beside
    junction connecting roads are not placed (18 samples in Frankfurt);
  - driveways as virtual junctions (standard; uncut street, turn roads linked with elementS / elementDir):
    0.64 (0.67 as an idea);
  - roundabout junction groups exist already; level crossings need railways, which are not converted
    (0.61 as an idea);
  - junction surface (boundary / elevationGrid) 0.50, pedestrian network from separate footways 0.48,
    G2 lane-border curvature 0.38, signalReference on connecting roads 0.21.
