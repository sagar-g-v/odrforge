# Progress and open decisions

Branch `geometry-quality` (not pushed). Plan: `docs/reference/spec_atlas.plan.html`. Rule coverage:
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
- Curb lanes: lowered curbs at crosswalks (lane height records) not done yet; curb lanes duplicate the
  sidewalk's existing fold / corner / edge-break findings (Stelvio folds 98 -> 140, same places).
- Pre-existing plain-road overlaps: Munich 2, Tokyo 3 (`quality.road_overlap`).
- Official checker bugs: `road.geometry.contact_point` skips only junction id 1; crashes on roads
  without `<link>`.
- Overlap resolution (2 x 5 s on LA) is an order-dependent sweep; making it parallel needs a redesign.
- CARLA height steps on LA (LibCarla ignores superelevation) and `jumps2.py` sampling every third
  waypoint (counts shift when roads are added).
- Not yet started: 1.5 direct junctions, 1.6 slip lanes, virtual junctions (driveways), 1.8 junction
  surface, Phase 2 lane vocabulary / road marks, Phase 3 G2 smoothness, Phase 4 signals and stop lines,
  Phase 5 pedestrian network.
