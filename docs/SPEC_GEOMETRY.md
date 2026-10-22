# OdrForge output specification (normative)

This document defines what correct OdrForge output is, for any OSM input anywhere: right- or
left-hand traffic, and any country's tagging. Each rule is numbered and uses MUST or SHOULD. Each
rule maps to one checkable invariant, and the rule ID is also the invariant's key in checker output.
A MUST violation is a defect. A SHOULD violation is reported with its value but does not fail the
gate.

Scope:
- **In scope:** interpreting OSM correctly, the OpenDRIVE mathematics, and conformance with the
  simulators that consume the output.
- **Out of scope:** OSM data errors (a oneway that starts nowhere, a wrong `lanes` count). Checkers
  report these as `OSM-ERR`, separately from defects.

Conventions are the ones in `docs/DESIGN.md` and `docs/research/opendrive_spec_notes.md`. When this
document and the code disagree, this document is the reference and the code is the defect.

---

## 0. Notation, numeric tolerances, checker contract

### 0.1 Notation

| Symbol | Meaning |
|---|---|
| `s`, `t` | Reference-line coordinates. `t` is positive to the left. |
| `θ(s)`, `κ(s)` | Reference-line heading and curvature. |
| `o(s)` | `laneOffset`. |
| `w_l(ds)` | Lane width polynomial. |
| `B_l(s)` | Outer border of lane `l`. |
| `c_l(s)` | Lane centre: `t = (B_inner + B_outer)/2`. |
| `P_l(s)` | The plan point of the lane centre: `ref(s) + c_l(s)·n(s)`. |
| drivable lane | `type ∈ {driving, bidirectional, entry, exit, onRamp, offRamp, connectingRamp, slipLane, bus, taxi, HOV}`, or `restricted` carrying an `<access>` allow. |
| travel sign | +1 if the lane travels along +s, −1 if against s, ±1 if `direction="both"`. |

The travel sign follows from `road@rule` (RHT is the default):
- **RHT:** lanes with `id < 0` travel +s.
- **LHT:** lanes with `id > 0` travel +s.
- `direction="reversed"` flips the sign (1.8). It MUST NOT be used to encode the traffic side.

### 0.2 Tolerances

Every tolerance below is justified by numeric precision or by a cited design standard. None is
fitted to a test area.

| Name | Value | Justification |
|---|---|---|
| `ε_pos` | 1e-3 m | Output prints ≥ 12 significant digits and \|x\|,\|y\| < 1e5 m (rounding ≤ 1e-7 m). Exact clothoid evaluation error is ≤ 1e-9·L. 1 mm is ≥ 1e4 times the numeric noise and 10 times below the 1 cm resolution of CARLA and esmini waypoints. |
| `ε_hdg` | 1e-4 rad | Equals `ε_pos` over 10 m of road. At 30 m/s the lateral-velocity step is 3 mm/s. Recommended in spec notes §5. |
| `ε_κ` | 1e-5 1/m | Curvature is printed to ≥ 12 significant digits, and Newton/fit residuals are ≤ 1e-9. At 30 m/s the lateral-acceleration step is v²·ε_κ = 0.009 m/s², an order of magnitude below the jerk AASHTO accepts over one 0.1 s step (C = 0.3 m/s³ [A1 §3.3]). Any larger step is a real kink in the driven path. |
| `ε_w` | 1e-3 m | Defines "zero width" (same reasoning as `ε_pos`). |
| `ε_s` | 1e-2 m | Position along s of an OSM node: the projection converges to 1e-9 m, and `sOffset` and `s` are printed with ≥ 9 decimals. 1 cm covers rounding when s is re-based per section. |
| `ε_B` | 2.0 m | Map-boundary allowance, applied only to an *unlinked* road end. `clipOsm` cuts ways exactly on the box (Liang-Barsky), so a boundary road should end on the box (`d ≤ ε_pos`). Some boundary roads instead end at the last real OSM node inside the box (the clipped tail is not emitted; cause not yet traced in the code). An unlinked end within `ε_B` of the box, or outside it (`keepOutside` = 30 m keeps junctions just beyond the cut), is a boundary end. If it is more than `ε_pos` inside the box, it is also reported as SHOULD `FLOW-1/boundary-short`. Measured on the 2026-10-12 outputs: boundary ends are at most 1.66 m inside the box, and the nearest other unlinked end is at least 22.4 m from it, so the result does not depend on the exact value anywhere between 1.7 and 22 m. (An earlier justification cited the "shortened to 2.0 m" warnings. Those concern roads shortened by junctions, not the boundary.) |
| `τ` | 0.25 m | Digitising noise. Equals the Douglas–Peucker tolerance `noiseTolerance`. This is the only converter parameter used as a tolerance, and it is used only where GEO-1 allows "OSM noise". |

Design-standard constants:

| Constant | Value | Source |
|---|---|---|
| `W_P` | 2.13 m | Design passenger-car width (7 ft) [A1 Table 2-1a] |
| `R_P` | 6.4 m | Passenger-car centreline turning radius (21 ft) [A1 Table 2-2a] |
| `L(W,S)` | see TRN-2 | Taper length [M1 §3B.09; M1 Table 6C-4] |

### 0.3 Checker contract

- Each checker is a CLI with importable functions that writes JSON:
  `{RULE-ID[/sub-code]: {count, level, instances: [{road, lane, s, x, y, value, limit}]}}`.
  - `level` is `MUST`, `SHOULD` or `INFO`.
  - A sub-code names the failing case of a rule (e.g. `FLOW-1/exit-no-lane-link`).
  - Every field is present in every instance (`null` when it does not apply).
- One defect is one instance under its primary rule. Where a defect at a road end affects every lane
  (e.g. an unlinked road end), it is one instance per road end, with the lanes listed in `value`.
  Derived consequences (e.g. a FLOW-4 trap caused by a FLOW-1 end) are reported under their own rule at
  the same location.
- Output is deterministic: sorted by road id (numeric), then lane, then s.
- Geometry MUST be evaluated exactly: closed form for line and arc, Fresnel or series for spiral, and
  analytic polynomials for widths and offsets at the query `s`.
- Interpolated samples (`odrgeom.Road.ref`, 0.2 m linear) may be used for searching only, never for
  the value that is compared with a limit.

---

## 1. Topology and lane semantics from OSM (TOP)

**TOP-1 (MUST): traffic side.**
- Every road has `rule="LHT"` iff the country profile is LHT (`drivingSide: left`).
- The side is a property of the country, never of the way.
- OSM `oneway`, `lanes:*`, `turn:lanes` and `placement` are always read in the driving direction, so
  the side changes only which OpenDRIVE side the lanes go on.
- Check: `@rule` on every `<road>`.

**TOP-2 (MUST): oneway.**
The following ways are oneway:
- `oneway=yes|true|1` (along the way)
- `oneway=-1|reverse` (against the way)
- `junction=roundabout|circular`
- `highway=motorway`, unless `oneway=no`
- `highway=motorway_link`, unless `oneway=no` (documented default D-5)

`oneway=reversible` and `alternating` have no fixed direction: build them as `direction="both"`
(1.8), or as one lane per direction when the profile has no `direction`.

For each OSM oneway way, every drivable lane of its road MUST travel in the way's direction, with the
reference-line orientation taken into account. A oneway has no drivable lane in the other direction.

Check: project the way's mid-segment points onto the road and compare `sign(dθ_way · dθ_ref)` with
each lane's travel sign.

**TOP-3 (MUST): lane counts.**
At every point of a road outside tapers (TRN) and outside the junction trim, the number of drivable
lanes travelling in each direction equals the OSM count:
- From `lanes:forward` and `lanes:backward` when tagged.
- Else from `lanes` minus `lanes:both_ways`, split forward = ⌈n/2⌉, backward = ⌊n/2⌋ (D-2).
- Else from the country default (D-1).
- `lanes:both_ways=1` is one lane with `direction="both"` (1.8) or `bidirectional` (≤ 1.7). It is
  adjacent to lane 0.

Not counted: bus and psv lanes listed in `bus:lanes` and `psv:lanes` are part of `lanes` (driving
plus `<access>`). Cycle lanes and parking are not.

Check: sample at the midpoint of every OSM segment.

**TOP-4 (MUST): per-lane lists.**
- `turn:lanes*`, `width:lanes*`, `bus:lanes*`, `psv:lanes*` and `access:lanes*` run left to right in
  the driving direction.
- Lane `k` of the list is the k-th drivable lane counted from the left of the traffic flow, whatever
  the traffic side:
  - RHT: the innermost lane is listed first.
  - LHT: the outermost lane is listed first.
- A list whose length differs from the lane count is ignored (OSM-ERR). It is never applied
  partially.

**TOP-5 (MUST): placement.**
- `placement` (oneway) or `placement:forward` (two-way): `left_of:N`, `middle_of:N`, `right_of:N`,
  `transition`.
- It sets where the OSM way lies across the carriageway: `o(s)` is chosen so that the way's `t`
  equals that position.
- Without the tag, the way is the middle of the carriageway (drivable lanes, cycle and bus lanes).
  Sidewalks, parking and shoulders are excluded (D-7).
- `transition` means the position changes linearly between the neighbouring ways' placements.
- Check: `|t_way(s) − t_placement(s)| ≤ ε_pos` at OSM nodes outside offset transitions.

**TOP-6 (MUST): turn semantics.**
`turn:lanes` values constrain the lane's junction movements (JCT-2):
- `left`, `slight_left`, `sharp_left`, `through`, `right`, `slight_right`, `sharp_right`, `reverse`,
  `none` (= through).
- `merge_to_left` / `merge_to_right` mark a lane that is dropped (TRN-4).
- A lane without a value gets positional defaults (D-9).

**TOP-7 (SHOULD): `change:lanes`.**
`change:lanes*` (`yes`, `no`, `not_left`, `not_right`) maps to the `roadMark` between the lanes:
- `no` on both sides gives `solid` with `laneChange="none"`.
- `not_left` / `not_right` gives a `laneChange` of `increase` or `decrease` in id terms, derived
  with the travel sign.

Lane changes forbidden by `change:lanes` remove the lateral edges in the lane graph (FLOW-0).

**TOP-8 (SHOULD): `destination:lanes`.**
`destination:lanes*` does not change geometry or counts. At a split (JCT-7) it is the tie-breaker
for which lanes continue on which branch, after `turn:lanes`.

**TOP-9 (MUST): roundabouts.**
- `junction=roundabout` ways are oneway in the way direction. The circulation sense comes from the
  way and is never re-derived from the country.
- Every entry and exit is a junction on the ring.
- The ring's lanes travel along the way.
- A 1.8 profile SHOULD reference the ring junctions in a `<junctionGroup type="roundabout">`.

**TOP-10 (MUST): access.**
- Ways with no motor-vehicle access (`access`, `motor_vehicle` or `vehicle` = `no`, `private`
  excluded per options) are not drivable roads.
- `bus=designated` lanes are drivable with `<access>`.
- Turn-restriction relations decide which movements exist (JCT-2):
  - `no_*` forbids the movement.
  - `only_*` forbids every other movement from that `from` way at that `via`.
  - `except=*` limits a restriction to the listed users.

---

## 2. Natural flow: roads follow OSM (NAT)

The converter follows each road as OSM draws it. It only splits a road where OSM diverges, splits or
merges.

**NAT-1 (MUST): road boundaries only where needed.**
A non-junction road ends only at one of these:
- (a) a junction (three or more carriageway ends meet, or a conflict node);
- (b) the map boundary (`ε_B`);
- (c) a real dead end (§4.2);
- (d) a lane-level gore junction (JCT-7).

Two non-junction roads MUST NOT be linked road-to-road. A change of lane count, width, speed, name,
bridge, tunnel or layer inside a chain is a lane section or a road attribute, never a new road.

Check: count `<link>` elements whose `elementType="road"` joins two roads with `junction="-1"`.

**NAT-2 (MUST): sections only where the lane layout changes.**
A new `laneSection` starts only where one of these changes: the set of lanes (count, type, travel
sign), or a lane-level attribute that has no `sOffset` form.

The following MUST stay inside the section, as `sOffset` records:
- width (a polynomial),
- speed,
- road mark,
- material,
- height,
- access.

A section boundary is removable, and so a defect, when all of the following hold:
- both sections have the same lane ids;
- the lanes have the same types;
- every lane links 1:1 to the lane with the same id (`successor` = own id on one side, `predecessor`
  = own id on the other).
- the lanes carry the same lane rules (OpenDRIVE 1.8.0 §11.9 `<rule>`). A rule has an `sOffset` but no end
  of its own, so a stretch with a rule (e.g. "no stopping at any time" within the legal distance of a junction)
  is a section of its own (user decision 2026-10-18).

Widths, marks, speed and the other attributes do not matter: they all have an `sOffset` form. The
checker reports what differs (`identical`, `width-poly <id>`, `width-jump <id>`, `mark <id>`) so it can
be triaged. (An earlier wording, "same ids, types and width records", missed boundaries that split a
width polynomial, such as a taper cut in two.)

Check: compare consecutive sections.

**NAT-3 (MUST): road identity.**
A road's id and its OSM-way trace (`userData code="osmWays"`) stay the same along every stretch
where the lane count does not change. A road is never split or renumbered for a width-only change.

**NAT-4 (MUST): edges continue.**
A curb, shoulder, sidewalk/walking, border, biking or parking lane present at the end of a section
(or road) continues at the start of the next section (or linked road):
- same type,
- same side,
- outer border continuous (`|ΔB| ≤ ε_pos`, `|ΔB'| ≤ ε_hdg`),
- unless the OSM tags change at that node (e.g. `sidewalk=no` on the next way),
- or (user decision 2026-10-18) a street-side parking area mapped apart from the road starts or ends there: its
  parking lane, and the road-level border strip before it, grows from zero width at the area's start and falls to
  zero at its end, so the borders beyond stay continuous (laneflow `edge.break.osm_area`).

At junctions, edge lanes end, and the crossing is represented by a crosswalk object or a 1.8
`crossPath`. Every edge-lane end that is not at a junction, the map boundary, a dead end, or an OSM
tag change is a break.

**NAT-5 (MUST): no invented shape or direction.**
The reference line does not change its turning direction where the OSM way does not.

Take two consecutive retained OSM vertices `i, i+1` whose deflections `δ_i, δ_{i+1}` have the same
sign, or a run of collinear vertices. Between them, `κ(s)` has no sign change. Values below
`κ_0,i = 8·ε_pos/ℓ_i²` count as zero. This is the curvature whose sagitta over the segment length
`ℓ_i` is 1 mm.

The road's overall direction (its start and end headings, within GEO-1's lateral bound) is the way's
direction. It is reversed only to orient a chain along oneway traffic (DESIGN, chains).

**NAT-6 (SHOULD): heading containment.**
At each retained OSM vertex `i`, `θ(s_i)` lies between the incoming and outgoing segment headings
`φ_{i−1}` and `φ_i`, widened by `asin(min(1, (d_{i−1}+d_{i+1})/ℓ))`, where the `d` are the GEO-1
deviations allowed at the neighbouring vertices.

---

## 3. Lane graph and flow, the "game" (FLOW)

This section validates flow, not geometry. Treat the network as a game. A car placed at the start of
any lane, at any legal entry, must be able to reach every end the OSM network allows. It must get
there without dropping off or being trapped, along every path, running each path once. Every lane
that exists must be usable.

### 3.1 The lane graph (FLOW-0, definition)

**Nodes** are `(road, laneSection index, lane id)` for every drivable lane.

**Edges** go in the travel direction:
- **Longitudinal**, inside a road: from section k to section k±1 through the lane's `successor` (for
  +s travel) or `predecessor` (for −s travel).
- **Longitudinal**, road to road: through the road link plus the lane link at that end. The target
  section is the first one if `contactPoint="start"`, the last one if `"end"`.
- **Longitudinal**, into a junction: through every `<connection incomingRoad=R>` `<laneLink from=l>`
  to the connecting road's lane. Out of a connecting road, through its own road and lane links.
- **Lateral (lane change):** between adjacent drivable lanes of the same section with the same travel
  sign, where both have width > `ε_w` somewhere and the separating `roadMark@laneChange` allows that
  direction in at least one of its records. If the profile has no `laneChange`, every adjacent pair is
  linked.
  - The separating mark is the `roadMark` of the inner lane of the pair. For lanes 1 and −1, it is the
    centre lane's mark.
  - OpenDRIVE numbers lanes ascending from right to left. `increase` allows a move toward the higher
    id, `decrease` toward the lower id, and a missing attribute means `both`.

**Link declaration.** A joint between two lane ends can be declared by either side: the upstream
lane's `successor`, the downstream lane's `predecessor`, or a road link on each road. The edge exists if
either side declares it, because a reader that follows one side still connects the lanes. A joint
where both sides can declare the link but only one does is `FLOW-2/asymmetric`.

The entry into a connecting road comes only from the junction's `<laneLink>`. The connecting lane's own
`predecessor` toward the incoming road never creates that edge. If it names a lane that has no
`laneLink` into it, that is `FLOW-2/asymmetric`. The exit from a connecting road comes from its own
road link and lane links, because the outgoing road links to the junction.

When both ends of a road are in the same junction (a loop road), a connection is assigned to an end in
this order:
1. by the connecting road's own link back to the road (its `contactPoint`);
2. otherwise by the travel sign of the `laneLink from` lanes;
3. as a last resort, by distance.

**Ends:**
- A lane's *entry end* is where it starts in its travel direction; its *exit end* is where it ends.
- A **legal source** or **legal sink** end is one of:
  - (i) at the map boundary (`ε_B`);
  - (ii) at a real dead end (JCT-5), where the entry of one lane and the exit of the other direction
    are paired;
  - (iii) a zero-width end (`w ≤ ε_w`): a taper end inside a road, or a zero-width connection end at a
    road end (XS-1). It is reached and left laterally, so it is neither a source nor a sink.
- The allowances (i) and (ii) apply only to an **unlinked** road end. A road that is linked at that
  end, for example to a junction kept just outside the box, must link every lane there.
- **Directional dead end.** This is an OSM node where a two-way way continues only as a oneway in the
  other direction and no other selected way meets. OSM leaves one direction no way on except turning
  round there. D-10 requires a U-turn there, since it is the only way out. Until the converter builds
  one, the stopped direction's taper end is a trap. It is reported as
  `FLOW-4/trap-oneway-change` (and `orphan-oneway-change` for the started direction) so it can be told
  apart from converter-made traps (Open item O-4).

### 3.2 Rules

**FLOW-1 (MUST): no trap, no orphan.**
Every drivable lane end without a longitudinal edge is a legal source or sink.

Sub-codes:
- `exit-unlinked` / `entry-unlinked` (MUST): the road end has no link at all, and is not a boundary or
  a dead end. One instance per road end, with its lanes in `value`.
- `exit-no-lane-link` (MUST): the road is linked at that end, but this lane has no lane link or
  connection. A car that drives the lane to its end is dropped, even if it could have changed lanes
  earlier.
- `entry-no-lane-link` (SHOULD): the road is linked, but nothing leads into this lane. The lane can be
  entered by a lane change inside the road, so it is not necessarily unusable. FLOW-4 decides whether
  it is (`orphan`). It is usually a JCT-1 completeness gap. (An earlier version made this a MUST,
  which contradicted JCT-1.)
- `abrupt-end` / `abrupt-start` (MUST): a lane ends or starts inside a road at width > `ε_w` with no
  link.
- `boundary-short` (SHOULD): an unlinked boundary end that stops more than `ε_pos` inside the box
  (§0.2 `ε_B`).

**FLOW-2 (MUST): edges are well formed.**
- Every link target exists and is drivable (`dangling`).
- Linked lanes have compatible travel signs (`wrong-way`):
  - Road-to-road: target travel sign = +1 iff `contactPoint="start"`.
  - Junction: the connecting lane's sign agrees with the connection's `contactPoint`.
- Links are symmetric (`asymmetric`). If `a.successor = b`, then b's link that points back to a's
  road and end names a.
- A lane linked across a section boundary is the same lane in the lane graph, so it MUST keep its
  side and its travel sign.

**FLOW-3 (MUST): connectivity matches OSM.**
Let `C_O` be the weakly connected components of the OSM carriageway graph:
- the selected ways (those the converter emitted, from `osmWays`), clipped exactly as `clipOsm` clips
  them: segments are cut at the box, and a node outside it is kept only when two or more highway ways
  use it and it lies within 30 m of the cut;
- nodes shared by ways.

Let `C_L` be the **road-level** weak components of the lane graph: contract each road's lanes to one
node, and join two roads when a longitudinal edge joins any of their lanes. Every road whose OSM ways
are in one `C_O` component MUST be in one `C_L` component. The location reported is the shared OSM node
nearest an unlinked end.

Components are taken per road, not per lane, because the two directions of a two-way road with no
junction (a road crossing the map) are two lane-level components, though it is one correct road.
Gaps inside a connected road component are FLOW-1 findings.

- A lane outside the main lane component whose ways are connected to the main OSM component is a
  **missing link** (defect).
- A lane whose OSM component is itself separate (an island, a clipped fragment) is **independent**.
  It is checked on its own with FLOW-1, -2 and -4. It is not a defect.

**FLOW-4 (MUST): reachability is preserved (the game).**

1. For every legal source lane, compute the set of legal sinks reachable in the lane graph.
2. Compute the reachable set in the OSM directed graph. Its nodes are way segments, and its edges are
   movements allowed by TOP-2, TOP-10 and the U-turn default D-10.
3. Compare the two sets, mapped to boundary and dead-end locations.
4. Every OSM-reachable sink MUST be lane-reachable (`unreachable`).
5. Every lane-reachable sink MUST be OSM-reachable (`illegal-reach`: an extra connection, wrong-way
   travel, or a forbidden turn).

Additional requirements:
- **Edge coverage:** every longitudinal edge lies on at least one source→sink path, or in a strongly
  connected component that has one. This is "running once at each path".
- **Closed networks:** for components with no legal source or sink, the whole component MUST be
  strongly connected.

Path enumeration is not needed. The exact check is:
1. Condense the graph (longitudinal and lateral edges) into strongly connected components (SCCs).
2. Compute forward reachability from the sources and backward reachability from the sinks.
3. A lane that is not in both is unusable.

Edge coverage follows from this. If u is reachable and v reaches a sink, then u→v lies on a
source→sink path.

Each root cause is reported once, as an SCC:
- `FLOW-4/trap`: an SCC with no outgoing condensation edge that contains no legal sink;
- `FLOW-4/orphan`: an SCC with no incoming edge that contains no legal source;
- `FLOW-4/closed-not-strong`: a weak component with neither sources nor sinks that has more than one
  SCC.

The representative lane is the one whose end has no longitudinal edge. Counts of unusable lanes are
statistics, not separate instances. The OSM-reachability comparison (steps 2–5 of the list above) is
not covered by `flowprobe`; `laneflow.py` covers it.

**FLOW-5 (MUST): splits and merges are traceable.**
At every place where lanes divide or join, the mapping from upstream lanes to downstream lanes MUST
satisfy all of the following. Such places are section boundaries, gore junctions (JCT-7) and lane
drops.
- **Conservation:** `n_down = n_up + added − dropped + options`.
  - `added` and `dropped` are lanes with zero width at that boundary (TRN).
  - `options` are lanes with two successors at a split, allowed only where `turn:lanes` gives the lane
    both directions (`through;slight_right`).
- **Order:** if lane a is left of lane b upstream, in the travel frame, then a's successors are not
  right of b's. No crossing links.
- **Side:** lanes added or dropped by a turn pocket are on the side given by TRN-4. Branch lanes are
  on the branch side (JCT-7).
- Every lane at the boundary that has nonzero width has at least one successor or predecessor.

**FLOW-6 (MUST): no unusable lane or road.**
Each of the following is a defect:
- a drivable lane that does not reach a width of at least `W_P` in its run (`narrow`):
  - the run is the chain of lanes linked across section boundaries of one road;
  - the maximum is exact: cubic endpoints and critical points;
  - `W_P` is the country design vehicle (default AASHTO P, 2.13 m). A `width:lanes` value tagged below
    `W_P` is applied exactly (XS-2) and reported as OSM-ERR, not as a defect;
- a lane or section whose width is ≤ `ε_w` over its whole length (`zero`);
- a road or connecting road shorter than 1.0 m. This is reported only as SIM-5 (`road-short`), not
  twice;
- a road with no drivable lane that is not a pure footway or cycleway emitted on purpose;
- any FLOW-4 unusable lane.

---

## 4. Geometry (GEO)

### 4.1 Reference line

**GEO-1 (MUST): fit to OSM.**
For every OSM node `P_i` of a road's chain, outside the junction trim and outside gore stretches
(JCT-7), the distance to the reference line is `d_i ≤ D_i`, where:

```
D_i = max( w_i/2 ,  R_i · (sec(δ_i/2) − 1) )
```

- `w_i/2` is the semantic bound. `w_i` is the narrowest drivable lane bordering the OSM line's
  lateral position (TOP-5). A larger deviation puts the mapped way in a different lane than the one
  it represents.
- `R_i·(sec(δ_i/2) − 1)` is the geometric minimum. It is the external distance of a circular arc
  through a corner of deflection `δ_i`. `R_i` is the smallest radius at which the cross-section does
  not fold (GEO-5): the largest `|t|` of any lane border on the inside of the corner at `P_i`. No
  curvature-bounded smooth curve that follows both legs can pass closer to the corner.
- Between nodes, the Hausdorff distance between the reference line and the OSM segment
  `[P_i, P_{i+1}]` is ≤ `max(D_i, D_{i+1})`.
- **SHOULD** (fit quality, reported): `d_i ≤ τ + R_i·(sec(δ_i/2) − 1)`.

Reason: the reference line *is* the OSM way (DESIGN principle 2). Only noise and the unavoidable
corner rounding may move it.

**GEO-2 (MUST): plan-view validity.**

| Requirement | Tolerance |
|---|---|
| Elements in ascending s | — |
| `s_k = Σ L_{j<k}` | `ε_s` |
| `Σ L` = road `@length` | `ε_s` |
| Element length | ≥ 1e-3 m (tools divide by it) |
| `arc` curvature | ≠ 0 |
| `spiral` | `curvStart ≠ curvEnd` |
| Element types | `line`, `arc` and `spiral` only (DESIGN principle 1) |
| `hdg` | in (−π, π] |

**GEO-3 (MUST): continuity inside a road.**
At every element joint:

| Quantity | Tolerance |
|---|---|
| Position | `ε_pos` |
| Heading | `ε_hdg` (G1) |
| Curvature | `ε_κ` (G2) |

The documented G1 fallback (DESIGN, Geometry step 4) is a violation of this rule. It is reported with
the value `v²·Δκ`, using the road's speed, so it can be triaged.

### 4.2 Lane-centre paths (the path a vehicle follows)

**GEO-4 (MUST): G0/G1/G2 of every drivable lane-centre path at every link.**
For every longitudinal lane-graph edge u→v, compare `P_u` at its exit with `P_v` at its entry:

| Quantity | Tolerance |
|---|---|
| Position | `ε_pos` |
| Travel heading | `ε_hdg` |
| Signed curvature of the centre path | `ε_κ` |

Edges covered: section boundaries, road-to-road links, and incoming road → connecting road →
outgoing road.

The same G0/G1/G2 tolerances also apply at every point *inside* a lane wherever `o`, `w` or `κ`
change their record.

`c_l(s)` includes `o(s)` and the widths, so this rule makes lateral profiles C2. A cubic-Hermite
blend has a jump in `t''` of `6Δ/T²` at its ends, which is a lane-centre curvature step. A lateral
transition therefore has to be C2. For example, use piecewise cubics with a piecewise-linear `t''`,
which OpenDRIVE width and laneOffset records can express.

Exemption: the centre path of a lane inside its own taper (width growing from or shrinking to zero),
because no vehicle follows that path.

Curvature of `P(s) = ref(s) + c(s)·n(s)`, to be evaluated analytically:

```
P'  = (1 − κc)·T + c'·N
P'' = (−κ'c − 2κc')·T + (κ(1 − κc) + c'')·N
κ_P = (P'_x·P''_y − P'_y·P''_x) / |P'|³
```

**GEO-5 (MUST): no folding.**
At every s:
- For every lane border `t = B(s)`: `1 − κ(s)·B(s) > 0`.
- For drivable lanes, the centre-path radius `1/|κ_P|` is ≥ `R_P` = 6.4 m (AASHTO P vehicle). A
  turn tighter than this cannot be driven. Exception: an OSM-forced last-resort U-turn (D-10), which
  is reported.
- For every lane border and every lane centre, `1 − κ·t ≥ 0` is checked exactly on each polynomial
  piece, using the extrema of κ·t, not samples.

**GEO-6 (MUST): elevation continuity.**
At every link of the lane graph and at every record joint:
- `|Δz| ≤ ε_pos`;
- `|Δz'| ≤ ε_hdg` (C1, as DESIGN states for the Hermite elevation records);
- superelevation is continuous within `ε_hdg`.

`elevation` and `superelevation` records start at s = 0, ascending.

---

## 5. Cross section (XS)

**XS-1 (MUST): width validity.**
- `w ≥ 0` everywhere (ASAM `road.lane.width.lane_width_validity`).
- There is a width record at `sOffset = 0`.
- The centre lane has no width.
- Zero width (≤ `ε_w`) is allowed only at the zero-width end of a taper (TRN) and at a zero-width
  connection end (spec notes §4.4).

**XS-2 (MUST / SHOULD): width ranges.**
- **MUST:** a drivable lane at full width is ≥ `W_P` (the country design vehicle; default AASHTO P,
  2.13 m), since the design passenger car cannot fit in less. A tagged `width:lanes` below `W_P` is
  still applied exactly (see below) and is reported as OSM-ERR. These two rules contradicted each other
  before this exception was added.
- **SHOULD:** a drivable lane is ≤ 4.6 m (two `W_P` plus 0.3 m clearance: wider lanes carry two
  vehicles abreast and misstate capacity). Exceptions: an OSM `width:lanes` value that is larger,
  connecting roads (swept-path widening), and `bidirectional` lanes.
- **SHOULD:** sidewalk/walking lanes are ≥ 0.915 m (36 in clear width [A2 §403.5.1]).
- **SHOULD:** biking lanes are ≥ 1.2 m (4 ft [A3]).
- Untagged widths come from the country file (D-1) and so are always in range.
- A tagged `width:lanes` value is applied exactly. A tagged carriageway `width` is divided by the
  lane count.

**XS-3 (MUST): laneOffset continuity.**
`o(s)` is C0 (`ε_pos`) and C1 (`ε_hdg`) at every record joint. It is C2 wherever it moves drivable
lane centres (GEO-4). There is one `laneOffset` record at s = 0, and no `laneOffset` together with
`<border>`.

**XS-4 (MUST): lateral shift rate.**
Every change of the lateral position `c_l` of a lane that continues, through `o` or through an inner
lane's width, of magnitude `Δ` over length `T` satisfies `T ≥ 0.5·L(|Δ|, S)`. This is the MUTCD
shifting-taper minimum [M1 Table 6C-4], with L from TRN-2.

Overlap-resolution shifts (DESIGN, resolve overlaps) are subject to this rule as well.

**XS-5 (MUST): link width.**
At every longitudinal lane-graph edge, `|w_u(exit) − w_v(entry)| ≤ ε_pos`, and the border slope is
continuous within `ε_hdg`. The only exceptions are zero-width taper ends, which have no link.

---

## 6. Transitions: lane add / drop (TRN)

**TRN-1 (MUST): location.**
- A lane is added or dropped at the OSM node `N` where the lane count changes in the travel
  direction.
- The zero-width end of its taper lies at `s_N` (within `ε_s`). `s_N` is the projection of N.
- The taper lies on the side of N where OSM has the lane, so the lane never exists where OSM has
  none (D-11).
- Section boundaries MUST NOT be moved to make room for a taper.
- Changes closer together than 1.0 m (SIM-5) may be merged into the later one. The merge is
  reported.

**TRN-2 (MUST): taper length.**
For a lane of full width W where the speed is S, the length L(W,S) is
[M1 §3B.09, lane-reduction transition; M1 Table 6C-4]:

```
L = W·S²/60      (S < 45 mph;  L, W in ft)    ≡  L = W·S²/155.4   (S < 72.4 km/h; L, W in m)
L = W·S          (S ≥ 45 mph)                  ≡  L = W·S/1.609    (S ≥ 72.4 km/h)
```

- A **dropped lane** (merge) tapers over ≥ L.
- An **added lane** tapers over ≥ 0.5·L (shifting taper: drivers move into a new lane; they are
  not forced to merge).
- When the stretch available between N and the next change point, junction trim or road end is
  shorter than required, the taper MUST use all of it. It is then reported as SHOULD
  `TRN-2 short-taper`, with value = available length and limit = required length.

S is the road's `maxspeed`. If that is missing, it is the country `defaultSpeed[class]` (D-4).

German guidelines (RAA 2008 for motorways, RAL 2012 for rural roads, RASt 06 for urban streets) give
taper lengths in their own tables. Those tables are not yet encoded or verified, so the MUTCD formula
with the local speed applies until `rules/countries/DE.json` carries them (Open item O-2).

**TRN-3 (MUST): taper shape.**
- Width goes monotonically from 0 to W (or from W to 0).
- The width slope is zero at both ends (C1).
- Neighbouring lanes keep their positions: `o` is held, so the existing lanes do not move when a lane
  is added or dropped at one side (DESIGN).
- A recentring of `o` follows XS-4.

**TRN-4 (MUST): side of add / drop.**
In this order:

1. `turn:lanes` `merge_to_left` / `merge_to_right` names the dropped lane and the neighbour it merges
   into.
2. A new inner lane whose turn is toward the divider (`left` / `slight_left` in RHT, `right` /
   `slight_right` in LHT) is a turn pocket added at the inner side.
3. Otherwise lanes are added or dropped at the outer side (D-12).

The FLOW-5 order rule must hold.

**TRN-5 (MUST): width-only change.**
- Where only widths change (same lane set), the lanes keep their ids.
- There is no new section, or, if another attribute needs one, the section has identical lanes
  (NAT-2).
- The width moves to its new value continuously (XS-5) over ≥ 0.5·L(|ΔW|, S) (XS-4).

---

## 7. Junctions and gores (JCT)

**JCT-1 (MUST): movement completeness.**
- Every OSM-legal movement (approach way → exit way through the junction's nodes, allowed by TOP-2,
  TOP-10 and D-10) has at least one lane path through the junction.
- Every drivable lane arriving at the junction has at least one connecting lane (FLOW-1).
- Every drivable lane leaving the junction SHOULD have at least one. A lane that is reachable only by
  a lane change that its own road allows is reported as SHOULD `FLOW-1/entry-no-lane-link`. If it is
  not reachable at all, it is MUST `FLOW-4/orphan`.

**JCT-2 (MUST): no forbidden movement.**
None of the following is built:
- a connecting lane against a oneway;
- a connecting lane forbidden by a restriction relation;
- a connecting lane outside its lane's `turn:lanes` set;
- a U-turn, except as D-10 allows.

Turn classes are defined by order, not by angle thresholds:
- Sort the exits by relative turn angle.
- `through` is the exit with the smallest |angle|.
- `left` / `slight_left` / `sharp_left` are exits on the left of it, ranked by angle. Right is
  symmetric.
- A lane with `left` may use any exit left of `through`. With several left exits, the rank (slight,
  plain, sharp) picks among them.

**JCT-3 (MUST): no crossing inside one approach.**
- Two connecting lanes from the same incoming road and travel direction MUST NOT intersect in plan,
  except within the first and last `w/2` where they share an edge.
- Their exit order matches their entry order (FLOW-5 order).
- Crossings between different approaches are expected (a left turn crosses the opposing through
  traffic).
- The exception is when `turn:lanes` itself orders the lanes inversely (rare). The checker then
  reports the `turn:lanes` value as the reason.

**JCT-4 (MUST): smooth connecting paths.**
- GEO-4 at both ends (G0, G1 and G2 of the lane-centre path).
- GEO-5 inside the junction.
- XS-5 widths at both ends.
- Connecting roads have one lane in the CARLA profile.
- The reference line follows the travel direction.
- `contactPoint` is given explicitly on every road link of a connecting road.

**JCT-5 (MUST): real dead ends.**
A road end is a real dead end iff its OSM end node meets all of these:
- it is used by exactly one selected highway way, counting only motor-vehicle classes (footways
  excluded);
- it is not within `ε_B` of the map boundary;
- it is not a snap candidate (DESIGN, topology preparation).

Such an end is confirmed by any of: `noexit=yes`, `highway=turning_circle` / `turning_loop`, or by
having no other way at that node.

At a dead end of a two-way road, the exit of one direction and the entry of the other are legal
(FLOW-1 ii). A oneway that starts or ends at such a node is OSM-ERR.

**JCT-6 (MUST): junction structure.**
- A junction exists only where three or more carriageway ends meet, or at a conflict node (ASAM
  `junctions.common.not_only_two`).
  - Check `JCT-6/not-only-two`: the distinct (road, contactPoint) ends that the junction's connecting
    roads link to number ≤ 2. A typical cause is a third OSM way at the node that the converter did not
    emit, which leaves a needless junction (and a NAT-1/NAT-3 road split).
  - Check `JCT-6/empty`: a `<junction>` with no `<connection>`, for example one that carries only a
    `<controller>`.
- There is one `<connection>` per (incoming road, connecting road).
- `laneLink` covers only lanes that lead into the junction.
- Connecting roads are never incoming roads.
- `priority` elements are written only without signals.

**JCT-7 (MUST): gores (lane-level splits and merges).**
At a freeway split with a shallow branch (DESIGN, lane-level gores):

- **Lane assignment.** The trunk's upstream lanes partition into main and branch lanes, using
  `turn:lanes` (`slight_right` / `right` → branch on the right; `slight_left` → left), then
  `destination:lanes`, then the branch's lane count taken from the branch side.
- **Option lanes.** An option lane (`through;slight_right`) has two successors.
- **Conservation and order** as in FLOW-5.
- **Branch position.** Throughout the gore stretch, the branch's OSM way lies within the branch lanes
  laterally: `|t_way − t_branchcentre| ≤ D` (GEO-1 bound).
- **Separation point.** The junction sits where the drivable lanes separate (no surface overlap).
  Main lanes continue 1:1.
- A merge is the mirror image of a split.

**JCT-8 (MUST): sidewalks beside the carriageway, not on it.**
Inside a junction, no walking or curb lane of a connecting road overlaps a driving lane of another connecting road of
the same junction by more than 0.5 m² (plan view). The Junction Guideline 1.0.0 §6.3 (T junction) carries the curb and
walking lanes round a corner on the connecting road that turns there, beside its driving lane, so the two share a border.
Crossing roads (a cross path's walking lane over the carriageway, ASAM OpenDRIVE 1.8.0 §12.6 crossPath) are exempt.
- Check `sidewalk_on_driving`: one instance per (walking/curb lane, driving lane) pair, value = overlap area in m².

---

## 8. Simulator conformance (SIM)

**SIM-1 (MUST): schema.**
- The `standard` profile validates against the ASAM OpenDRIVE 1.8.1 XSD with 0 errors.
- The `carla` profile validates against 1.4, except for the deviations listed in SIM-3.
- The built-in validator (`validate.cpp`) reports 0 errors.

**SIM-2 (MUST): ids and attributes.**
- Road, junction, connection, signal, controller and object ids are numeric, unique per class, and
  ≤ uint32.
- In the carla profile, ids have ≤ 32 characters.
- Every attribute that readers default silently is written explicitly: `junction="-1"`,
  `contactPoint`, `pRange`, `unit`.

**SIM-3 (MUST): traffic side in every profile.**
TOP-1 holds in every target. 1.4 has no `rule` attribute, so the carla profile MUST still write
`rule="LHT"`, as a documented schema deviation, or refuse LHT input.

Without it, a 1.4/RHT reader runs every lane of an LHT map in the wrong direction. Whether LibCarla
honours `rule` is listed as "verify" in the spec notes. Until it is verified on a CARLA build, LHT in
CARLA is Open item O-1.

**SIM-4 (MUST): CARLA.**
The map loads in LibCarla (`carla.Map(name, xodr)`) without exceptions, and:
- `generate_waypoints(1.0)` covers every drivable lane of the lane graph;
- `get_topology()` contains every FLOW longitudinal edge between distinct roads;
- lane types are within CARLA's accepted set, mapped as in spec notes §4.3;
- traffic lights have controllers referenced from their junction.

**SIM-5 (MUST): esmini and degenerate sizes.**
- `esmini` / `odrplot` load the file with no error lines.
- Roads and connecting roads are ≥ 1.0 m long (spec notes, pitfall 7).
- Sections are ≥ 1.0 m long.
- Plan-view elements are ≥ 1e-3 m long (GEO-2).

---

## 9. Not derivable from OSM: documented defaults

The converter MUST use exactly these defaults. Each must be overridable by the country file or a
CLI option, and must be stated in the output header `userData`.

| ID | Unknown in OSM | Default |
|---|---|---|
| D-1 | Lane width, untagged | `rules/countries/<CC>.json` `laneWidth[class]`. Unknown country → `XX.json` (e.g. IT falls back to XX today). |
| D-2 | Lane count, untagged | `defaultLanesOneway` / `defaultLanesTwoway[class]`. With only `lanes` on a two-way road: forward = ⌈n/2⌉. |
| D-3 | Traffic side | Country `drivingSide`, per run (not per OSM boundary relation). |
| D-4 | Speed, `maxspeed` missing | Country `defaultSpeed[class]` in `speedUnit`. Zone codes (`DE:urban`) from `maxspeedZones`. |
| D-5 | Oneway of `motorway_link` | Implied oneway in the way direction. |
| D-6 | Shoulders, sidewalks, curbs | Country `shoulderWidth[class]`. Sidewalks assumed on untagged urban streets (`--no-sidewalks` disables). `curbHeight` 0.15 m. |
| D-7 | Way position across the road | Middle of the drivable carriageway (TOP-5). |
| D-8 | Elevation | Measured terrain (DESIGN, elevation), else flat with storey offsets for `layer` crossings. |
| D-9 | Turns without `turn:lanes` | Leftmost lane: left + through. Rightmost lane: right + through. Inner lanes: through. A single lane: every allowed movement. |
| D-10 | U-turns | Not built at junctions, unless it is the only way out of, or into, a road end and it leads to another road (DESIGN, junctions). `restriction=no_u_turn` always wins. |
| D-11 | Where on the ground a tagged lane change happens | At the node: the zero-width taper end lies there, and the taper lies inside the way that has the lane (TRN-1). |
| D-12 | Side of an add / drop with no turn hint | Outer side (TRN-4). |
| D-13 | Taper length | MUTCD formula with S from D-4 (TRN-2). |
| D-14 | Junction extent | Carriageway overlap plus stop-line setback 1 m and curb return 6 m, scaled for major roads (DESIGN). |
| D-15 | Connecting-path shape | G2 lane-centre path between the incoming and outgoing lane poses (JCT-4). |
| D-16 | Road marks | Country centre-line and edge styles. Lane-change permission from `change:lanes`, else allowed. |
| D-17 | Map boundary | The `--clip` box. The converter's own projection defines its metres. |

---

## 10. Invariant index (rule → check)

| Rule | Check (exact) | Inputs |
|---|---|---|
| TOP-1, SIM-3 | `@rule` on every road vs country side | xodr, country |
| TOP-2 | Lane travel sign vs oneway way direction | xodr, OSM |
| TOP-3, TOP-4 | Drivable lanes per direction at OSM segment midpoints vs tags | xodr, OSM |
| TOP-5 | `t` of the way vs placement position | xodr, OSM |
| NAT-1 | Road-to-road links between non-junction roads | xodr |
| NAT-2 | Identical consecutive sections | xodr |
| NAT-3 | OSM way trace vs lane-count stretches | xodr, OSM |
| NAT-4 | Edge-lane presence and border continuity at boundaries and links | xodr, OSM |
| NAT-5, NAT-6 | Curvature sign changes / heading between OSM vertices | xodr, OSM |
| FLOW-1 … FLOW-6 | Lane graph, SCC condensation, reachability vs OSM directed graph | xodr, OSM, clip |
| GEO-1 | `d_i` vs `D_i` at OSM nodes; Hausdorff per segment | xodr, OSM |
| GEO-2, GEO-3 | Element bookkeeping; joint G0/G1/G2 | xodr |
| GEO-4 | Lane-centre G0/G1/G2 across every lane-graph edge and record joint | xodr |
| GEO-5 | `1 − κB > 0` per polynomial piece; `R_P` on drivable centres | xodr |
| GEO-6 | z, z′, superelevation continuity | xodr |
| XS-1 … XS-5 | Width records, ranges, `o` continuity, shift rate, link widths | xodr |
| TRN-1 … TRN-5 | Change node `s_N` vs zero-width end; taper length vs L(W,S); shape; side | xodr, OSM, country |
| JCT-1 … JCT-7 | Movement sets, restrictions, plan intersections, dead ends, gore partition | xodr, OSM |
| SIM-1 … SIM-5 | XSD, LibCarla, esmini, ids, minimum sizes | xodr, tools |

---

## 11. Evidence: flowprobe run on the gate outputs (2026-10-12, outputs written 05:25)

`flowprobe` (scratch checker, `build/tmp/spec/spec/flowprobe.py`) implements FLOW-0..4 and FLOW-6,
NAT-1/2/4, XS-5 at links, JCT-6, TOP-1/SIM-3 and SIM-5 as written above. Boundaries come from the
gate's `--clip` boxes, projected as `odrforge build` projects them (min/max of the four corners). Dead ends
come from the degree of OSM nodes over the emitted ways.

| Rule | la | munich | london | frankfurt_kreuz | stelvio |
|---|---|---|---|---|---|
| drivable lanes | 12,502 | 3,801 | 2,981 | 2,274 | 274 |
| `FLOW-1/exit-unlinked` (road ends) | 0 | 5 | 0 | 10 | 0 |
| `FLOW-1/entry-unlinked` (road ends) | 0 | 8 | 0 | 3 | 0 |
| `FLOW-1/exit-no-lane-link` | 55 | 25 | 7 | 8 | 0 |
| `FLOW-1/entry-no-lane-link` (SHOULD) | 222 | 75 | 35 | 44 | 0 |
| `FLOW-1/boundary-short` (SHOULD) | 2 | 4 | 1 | 5 | 0 |
| `FLOW-3/missing-link` | 0 | 1 | 0 | 3 | 0 |
| `FLOW-3/island` (INFO) | 16 | 9 | 15 | 7 | 0 |
| `FLOW-4/trap` / `trap-oneway-change` | 3 / 1 | 4 / 3 | 1 / 5 | 9 / 0 | 0 |
| `FLOW-4/orphan` / `orphan-oneway-change` | 7 / 5 | 13 / 6 | 3 / 8 | 3 / 0 | 0 |
| unusable lanes (unreachable / trapped) | 64 (53 / 28) | 93 (66 / 30) | 52 (38 / 15) | 60 (7 / 54) | 0 |
| `NAT-2` | 13 | 11 | 8 | 59 | 0 |
| `JCT-6/not-only-two` / `empty` | 1 / 0 | 10 / 1 | 0 / 0 | 5 / 2 | 0 |
| `SIM-3` (no `rule="LHT"`) | 0 | 0 | 2,375 | 0 | 0 |
| `SIM-5/road-short` / `section-short` | 0 / 0 | 2 / 2 | 4 / 4 | 1 / 1 | 0 |

The following were 0 in every area: FLOW-1 `abrupt-*`, FLOW-2 (dangling, wrong-way, asymmetric),
FLOW-4 `closed-not-strong`, FLOW-5 `order`, FLOW-6, NAT-1, NAT-4 and XS-5.

Planted defects: ten mutations of munich and nine of london (P5 had no target in the LHT area). Each
was reported at the planted road, lane and s, with no new finding elsewhere and no baseline finding
lost:
- a deleted `laneLink`, both when the lane has another connection and when it does not;
- a 0.5 m width step at a section start;
- a dropped road `successor`;
- a cut sidewalk;
- a flipped lane-id sign;
- a needless section split;
- a one-sided lane link;
- a cut in-road lane link;
- `rule="LHT"` on one road.

Verified examples (raw XML and OSM; crops in `build/tmp/spec/spec/crops/`):
- **Trapped turn lanes, frankfurt_kreuz.** turn:lanes is ignored at these junctions:
  - Road 389 (way 701701444, `lanes=6`, `turn:lanes=left|left|through|through|right|right`): only
    lanes −3 and −4 have connections in junction 39. Lanes −1, −2, −5 and −6 end there with no way on.
  - Road 42 (`through|through|right`): lane −3 has no connection in junction 20.
  - Road 190 (`…|right|right`): lanes −4 and −5 have no connection in junction 24.
- **Backward direction dropped at a junction, la road 399.** Junction 907 has no connection from road
  399, so lanes 1 and 2 (the −s direction, two lanes) are a trap (`FLOW-4/trap`, plus
  `exit-no-lane-link` ×2).
- **Missing junctions.**
  - frankfurt_kreuz road 233 (trunk_link 234004959, 2.0 m long) is unlinked at node 2256109040, which
    four oneway ways share.
  - munich road 632 (way 316009876, 12.3 m) is unlinked at both ends; node 1120257790 is shared by
    three residential ways.
- **Directional dead end, munich road 8.** Ways 30750675 and 247649059/060 are two-way, and
  356582887/3792170 between them are oneway. The −s lane tapers to zero at s = 73.6 with no way on,
  and the same −s lane re-appears from zero at s = 63.4 with no way in (`trap-oneway-change` and
  `orphan-oneway-change`, O-4).
- **Needless junction, munich junction 233.** It joins only roads 386 and 387. The third way at the
  node (service 386936421) was not emitted, and the bike lanes break across the junction.
- **Empty junctions.** munich 250 and frankfurt_kreuz 41 carry only a `<controller>`;
  frankfurt_kreuz 156 is completely empty.
- **LHT lost (SIM-3).** None of the 2,375 london roads has `rule="LHT"` (carla profile, revMinor 4),
  while the lanes are laid out LHT (oneways use left lanes). The writer emits `rule` only for
  revMinor ≥ 5.
- **NAT-2.** In la road 210 (ways 24716931 and 148385879), the section at s = 30.0 m is identical to
  the one at s = 0 (3 × 3.66 m, same marks and speed). This is the "end sections get one taper
  length" shift; TRN-1 also forbids it.
- **TRN-2.** The fixed `taperLength` of 30 m is shorter than L(W,S) for every class except low-speed
  residential: US 35 mph with W = 3.35 m gives L = 68 m; 65 mph with W = 3.66 m gives L = 238 m.

## 12. Open items

- **O-1:** Verify on a real CARLA build whether LibCarla reads `road@rule`. If it does not, LHT
  cannot be represented in the carla profile, and the converter must say so.
- **O-2:** Encode the RAA 2008, RAL 2012 and RASt 06 taper and auxiliary-lane tables in the DE rules
  after checking them against the published guidelines. The values could not be verified from
  public sources in this pass.
- **O-3:** The exact lane-change marking semantics for LHT `laneChange` (increase/decrease in id
  terms) need a test with esmini.
- **O-4:** Directional dead ends (§3.1). OSM makes the stopped direction turn round. The converter
  must build the D-10 U-turn there: a junction at the change node with a U-turn connecting road, or a
  documented equivalent. Until it does, `FLOW-4/*-oneway-change` instances are MUST defects, kept
  under their own sub-code.
- **O-5:** Cross-tool consistency. The sibling tools use other values for the same quantities:
  - `invariants.py` `TOL_HDG` = 1e-3 rad, against `ε_hdg` = 1e-4 here;
  - `laneflow.py` `BOUNDARY_BAND` = 1.0 m, against `ε_B` = 2.0 m here. Measured boundary ends lie up
    to 1.66 m inside the box, so 1.0 m would misread some of them as unlinked inner ends;
  - `laneflow.py` `MIN_VEHICLE_WIDTH` = 1.48 m (kei car), against `W_P` = 2.13 m here. This document
    makes `W_P` a country value to reconcile the two;
  - `laneflow.py` `UTURN_DEG` = 150° (a converter threshold), while JCT-2 here ranks turns by order
    and uses no threshold.

  One tolerance table (§0.2) should be imported by every checker.

## References

- [M1] FHWA, *Manual on Uniform Traffic Control Devices*, 2009 ed.:
  - §3B.09, lane-reduction transition markings: L = WS²/60 below 45 mph, L = WS at 45 mph or more;
  - Table 6C-4, taper lengths: merging ≥ L, shifting ≥ 0.5 L.
- [A1] AASHTO, *A Policy on Geometric Design of Highways and Streets* (Green Book), 7th ed. 2018:
  - Table 2-1a and Table 2-2a, P design vehicle: width 7 ft, centreline turning radius 21 ft;
  - §3.3, transition design controls: rate of change of lateral acceleration.
- [A2] U.S. DOJ, *2010 ADA Standards for Accessible Design*, §403.5.1, clear width 36 in.
- [A3] AASHTO, *Guide for the Development of Bicycle Facilities*, 4th ed. 2012, bike lane width.
- [FGSV] RAA 2008 (motorways), RAL 2012 (rural roads), RASt 06 (urban streets): German equivalents
  (O-2).
- ASAM OpenDRIVE 1.8.1 / 1.9.0 specification and checker rules, summarised in
  `docs/research/opendrive_spec_notes.md`.
