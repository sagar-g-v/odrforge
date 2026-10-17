# OSM fidelity contract

The converter's main mode builds the road network from every piece of OSM geometry, each as strictly as the
reference line, within the OpenDRIVE schema (user decision 2026-10-18). This contract holds the proven results of
that work to absolute limits, measured on the reference areas by `tools/inspect/osm_contract.py`. `gate.py` checks
them on every run that includes one of these areas, on the outputs the run is about to publish. Unlike the gate's
baseline counts, these limits do not move when a baseline is refreshed.

**Rules for changing this contract** (the same as for the [terrain contract](TERRAIN_CONTRACT.md))

- Never loosen a limit to let a change pass. A limit moves only with new evidence, and this document, the limit in
  `CONTRACT` and the evidence change in the same commit.
- Tighten a limit when a change proves a better result.
- Every check has a planted-defect test in `tests/osm_contract_selftest.py`: the clean outputs pass, and a copy with
  that one defect fails. The gate runs it. A check that no longer fails on its defect is a broken check.

## Checks and limits

| check | what is measured | area | proven (2026-10-18, standard / carla) | limit |
|---|---|---|---|---|
| parking | Street-side parking mapped as its own area (`amenity=parking` + `parking=street_side`, `lane` or `layby`, closed, wholly inside the area's clip box: the converter cuts ways at the box) drawn as parking lanes (OpenDRIVE 1.8.0 §11.7.1 lane type `parking`; 1.4H `laneType parking`). Every lane of every road is laid out as a polygon (borders every 1 m) and clipped exactly against each area (`parking_coverage.py`); an area counts as drawn with at least 80 % of its area under parking lanes. Limits: that count, and the mean share over the areas. The viewer's independent check (odrforge-viewer `src/review/osmEdges.ts`, 0.25 m raster) measured the same baseline. | munich | 20 of 30, mean 0.707 / same | ≥ 20, mean ≥ 0.70 |
| | | london | 12 of 17, mean 0.828 / same | ≥ 12, mean ≥ 0.82 |
| | | frankfurt_kreuz | 11 of 12, mean 0.895 / same | ≥ 11, mean ≥ 0.89 |
| | | stelvio | 1 of 8, mean 0.541 / same | ≥ 1, mean ≥ 0.54 |

Before the street-side parking change (`src/parking.cpp`) the same measure gave Munich 0 of 30, London 0 of 17,
Frankfurt 5 of 12 and Stelvio 0 of 8 areas drawn. Tokyo and LA have no street-side area inside their clip boxes.

**Open parking cases** (why the counts are not higher; they are not hidden by the limits):

- The area overlaps the drawn driving lanes, so the drawn carriageway is wider than the mapped one: Munich 342701001
  and 1157983619 on a four-lane street, London 256387555 and 256387556, Munich 482006127. Kerb-anchored carriageway
  edges (the next step of the plan) are the fix; until then the lane reaches from the drawn edge to the area's outer
  edge, or the area is left out.
- The area belongs to a road that is not converted: Munich 137087387 and 449417494 beside excluded service ways, and
  1485729896 drawn over a service road.
- Part of the area lies inside a junction, where it cannot be a lane of a road: Munich 575124084.
- The area's outline is irregular or wider than one lane follows: the lane has the median measured width over its
  stretch, so Stelvio's lay-bys (6 to 12 m deep, curving) are covered 35 to 75 %.
