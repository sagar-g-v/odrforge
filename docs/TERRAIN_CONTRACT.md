# Terrain contract

The proven terrain results of the reference areas, held as absolute limits by
`tools/inspect/terrain_contract.py`. `gate.py` checks them on every run that includes a terrain area (Munich, London,
Frankfurter Kreuz, Stelvio), after regenerating the carla output, the standard output and the conformed ground from the same inputs.
Unlike the gate's baseline counts, these limits do not move when a baseline is refreshed.

**Rules for changing this contract**

- Never loosen a limit to let a change pass. A limit moves only with new evidence, and this document, the limit in
  `CONTRACT` and the evidence change in the same commit.
- Tighten a limit when a change proves a better result. The margin over the proven value stays small.
- Every check has a planted-defect test in `tests/terrain_contract_selftest.py`: the clean outputs pass, and a copy
  with that one defect fails. The gate runs it. A check that no longer fails on its defect is a broken check.

## Checks and limits

All heights are in metres. Sampling is every 2 m along every road; "plain" means not a connecting road.

| check | what is measured | area | proven (2026-10-17) | limit |
|---|---|---|---|---|
| fresh | `<area>.xodr`, `<area>_standard.xodr` and `<area>_ground_conformed.odrz` are newer than the OSM file, the ground and deck grids, and `osm2xodr.exe` | all | (London road 389 floated 39 m in a carla file written before its deck grid) | none older |
| conformed | paved edges (0.3 m inside the outermost lane borders, z = elevation + t·tan(superelevation), OpenDRIVE 1.8.0 §10.5) vs the conformed ground, standard profile, outside bridges / tunnels | munich | p99 0.006 | p99 ≤ 0.02 |
| | | london | p99 0.008 | p99 ≤ 0.02 |
| | | frankfurt_kreuz | p99 0.044 | p99 ≤ 0.06 |
| | | stelvio | p95 0.132 | p95 ≤ 0.20 |
| centre | reference line vs the raw ground grid, plain roads outside bridges / tunnels | munich | p50 0.005, p95 0.045 | p50 ≤ 0.05, p95 ≤ 0.40 |
| | | london | p50 0.008, p95 0.077 | p50 ≤ 0.05, p95 ≤ 0.40 |
| | | frankfurt_kreuz | p50 0.005, p95 0.039 (0.044 before its covered underpasses were left out) | p50 ≤ 0.05, p95 ≤ 0.40 |
| | | stelvio | p50 0.025, p95 0.160 | p50 ≤ 0.05, p95 ≤ 0.25 |
| decks | bridge roads vs the measured deck where the deck grid has a value under them and no other road passes more than 2 m above within 4 m (a surface model sees the top deck only) | london | p95 0.144 (Waterloo Bridge 13.5–15.0 m), 100 % within 0.5 m | p95 ≤ 0.30, ≥ 98 % within 0.5 m |
| | the share of those samples within 0.5 m of the deck: a structure above a road bridge that is not a road (the ICE viaduct) and narrow bridges drawn beside their deck make a quantile meaningless there | frankfurt_kreuz | 92.9 % within 0.5 m (86.7 % before the street-deck rule and the smooth-deck filter were applied to Frankfurt) | ≥ 92 % |
| superelev | plain roads more than 30 m from their ends: the terrain cross slope is capped at 8 % (elevation.cpp), atan = 0.0798 rad, plus cubic overshoot | all | max 0.0822 | ≤ 0.085 rad |
| | everywhere: junction planes are capped at a 15 % gradient (elevation.cpp), and road ends bank into them | all | max 0.1489 = atan(15 %) | ≤ atan(15 %) + 2 mrad |
| steep | roads with a grade over 20 % at a 5 m step (the validator's `quality.max_grade`) | munich | 2 | ≤ 2 |
| | | london | 4 (9 before the street decks, 5 before the local grade guard) | ≤ 4 |
| | | frankfurt_kreuz | 4 (7 before deck cells needed a smooth surface near OSM bridges, make_terrain.py) | ≤ 4 |
| | | stelvio | 1 (12 on GLO-30 alone) | ≤ 1 |
| objects | trees and street lamps off bridge stretches: base = elevation(s) + zOffset (XSD 1.8.1 @zOffset) vs the conformed ground under them; osm2xodr seats them on that ground | all | 0 off (Munich 679, London 598, Frankfurt 276, Stelvio 2 objects; before: Munich 141 of 679 more than 0.3 m off, worst 8.2 m) | none more than 0.3 m off |
| buried | plain roads more than 2 m under the raw ground over 10 m or more, outside bridges / tunnels; samples under a bridge road (more than 2 m above within 4 m: the raw grid there may hold the deck, Frankfurt roads 57 and 276 under bridges 34 and 58) neither count nor end a run. The conformed ground trenches such a road. The parts of covered=yes stretches that deep are written as underpasses (elevation.cpp `coveredUnderpasses`, OpenDRIVE 1.8.0 §13.11). Measured over every covered stretch of Munich, London, Frankfurt and LA (36): at grade never more than 0.81 m deep at any sample, under a deck or hill a median of 5.54 m or more, so any limit between makes the same decisions. A 10 m run catches all five Frankfurt stretches before their underpasses (a 20 m run caught two) | all | 0 (before: Frankfurt 5 - roads 35, 136, 154, 224, 431 - and London 1, the Strand Underpass running 3.8-4.3 m under the street for 12 m past its OSM tunnel way; tunnel records are now extended over such ends, elevation.cpp `coveredUnderpasses`) | 0 |
| pits | junction roads with a point (2 m steps) outside tunnel / bridge records where the conformed ground lies more than 2 m under the raw ground: a junction under cover without a tunnel record. Connecting roads between tunnel ends get a tunnel record, and at a portal the part under the terrain (elevation.cpp `junctionsUnderCover`) | all | 0 (before: Frankfurt 134 roads / 723 points - the underground car park P15 - London 5, Munich 1) | 0 |
| cliffs | roads of 0.1 m and more (junction roads too) with a grade over 50 % between samples 0.25 m apart: short stubs the steep check's 5 m steps cannot see; no public road reaches 50 % (the steepest streets are about 35-37 %) | munich, frankfurt_kreuz | 0 (Frankfurt before the smooth-deck filter: 3, the 2 m stubs 97 / 366 / 367 climbing 5-6 m) | 0 |
| | | london | 1: road 250, a 5.5 m service stub climbing 2.3 m from junction 194 (at ground level, 3.8 m) to the terrain step beside it (open) | ≤ 1 |
| | | stelvio | 1: road 31, a 4.1 m service stub climbing 1.6 m out of junction 18, whose surface (capped at 15 %) cannot follow the hillside (open) | ≤ 1 |
| named | max grade of named streets on structures OSM does not tag as bridges | london | Adam Street 4 %, Lancaster Place 10 % | ≤ 10 % each |

The 99th-percentile Stelvio edge tail (3.2 m) is not limited: it is road 22's hairpin legs, which overlap in plan at
different heights; a 2-D ground cannot hold both.

## Evidence behind the numbers

- Conformed ground (src/conform.cpp, `osm2xodr --ground-out`): CHANGELOG, "Ground conformed to the roads".
- Stelvio 2.5 m DTM with the GLO-30 fill shifted onto it: CHANGELOG, "Stelvio on South Tyrol's 2.5 m DTM".
- Street decks: the London Adelphi terrace / Lancaster Place approach (make_terrain.py `raised_smooth`, `on_top`):
  CHANGELOG, "Streets on structures OSM does not tag as bridges".
- Planted defects (`tests/terrain_contract_selftest.py`):
  - a ground grid newer than the outputs;
  - the conformed ground 0.5 m under the roads;
  - every road lifted 1 m;
  - a plain road banked at 0.2 rad;
  - three roads at 30 %;
  - London bridges 3 m above their deck;
  - Adam Street at 30 %;
  - London trees 2 m under the ground;
  - a Stelvio road 3 m under the ground;
  - Frankfurt's underpasses removed (the covered stretches as plain roads again);
  - a 3 m pit in the conformed ground around one Stelvio junction road;
  - one more short Stelvio road (3-20 m) at an 80 % grade.

  Each one fails its check.

Frankfurter Kreuz (added 2026-10-17 with the car park anchoring, Jev 0.78, committed by the user's decision): the
limits hold the measured state, which has known flaws not limited here - its underground car park P15 lies a median
11-12 m under the ground at its own position (up to 55 m; one point 0.38 m above it). Rework: anchor the car park
locally instead of one level 0 for the whole network.
