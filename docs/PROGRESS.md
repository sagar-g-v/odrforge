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
| 6 | Stage timing; arm trimming in parallel | LA trimming 9.4 s -> 0.9 s per pass |
| tools | `jev_judge.py` (Jev reviews whole junctions, calibrated against planted defects) | Jev trusted on spec validity only |

## Needs your input

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
- Separate sidewalks: the lane-edge position breaks (London 11) do not come from the verge appearing
  / disappearing between segments (keeping a 5 cm verge everywhere changed nothing); next suspect is
  the sidewalk distance changing between segments at plain road-to-road links without a blend.
- Pre-existing plain-road overlaps: Munich 2, Tokyo 3 (`quality.road_overlap`).
- Official checker bugs: `road.geometry.contact_point` skips only junction id 1; crashes on roads
  without `<link>`.
- Overlap resolution (2 x 5 s on LA) is an order-dependent sweep; making it parallel needs a redesign.
- CARLA height steps on LA (LibCarla ignores superelevation) and `jumps2.py` sampling every third
  waypoint (counts shift when roads are added).
- Not yet started: 1.5 direct junctions, 1.6 slip lanes, virtual junctions (driveways), 1.8 junction
  surface, Phase 2 lane vocabulary / road marks, Phase 3 G2 smoothness, Phase 4 signals and stop lines,
  Phase 5 pedestrian network.
