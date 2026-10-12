# ASAM OpenDRIVE junction rules (reference)

Source: ASAM OpenDRIVE 1.9.0 specification (released 2026-05-19), chapters 10.3, 11.6 and 12, at
`https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/`.
The full 1.7.0 specification is stored next to this file (`ASAM_OpenDRIVE_BS_V1-7-0.html` / `.pdf`,
downloaded from asam.net). The 1.8 / 1.9 publication server's TLS certificate had expired when this was
written, so the 1.9.0 rules below were read through a summarising fetch and are restated in our own words
with their rule ids. The PDF of 1.9.0 is free but requires a download form at
https://www.asam.net/standards/detail/opendrive/ (to be added here by a person).

## What a junction is
- A junction is its own element with its own `@id` (required, unique). It has **no** predecessor or
  successor (`junctions.common.junctions_no_pred_succ`).
- It connects the ends of several roads and provides the paths between them. In a common junction
  (type `default`) each path is a **connecting road** (`road@junction` = the junction id). Connecting
  roads are the only roads whose surfaces may overlap.
- Types: `default` (common: traffic may cross), `direct` (split / merge only, no crossing, no connecting
  roads), `virtual` (main road not interrupted, e.g. a driveway), `crossing` (traffic cannot change roads,
  e.g. a level crossing of two roads / a pedestrian crossing).
- `junctions.no_overlap`: no junctions of any type shall overlap each other.

## When to use one
- `junctions.common.when_to_use`: only where roads cannot be linked directly (the predecessor or
  successor would be ambiguous). `road.linkage.is_junction_needed`: two roads are linked directly only if
  the linkage is clear.
- `junctions.common.not_only_two` (1.9.0, should): not where only two roads meet - link them directly.
- `junctions.direct.split_or_merge` / `road_connectivity`: a direct junction joins one road on one side
  with several on the other, only to split or merge with no crossing traffic, placed where the headings of
  the road and the ramp / slip lane are identical (`junctions.direct.road_ramp_heading`).

## Incoming roads, outgoing roads, connecting roads
- Incoming roads end at the junction: their `<predecessor>` / `<successor>` is
  `elementType="junction" elementId=<junction id>` with **no** `contactPoint`
  (`road.linkage.junc_link_attribute_usage`). Outgoing roads are not declared: they are the roads the
  connecting roads lead into. Incoming roads serve as outgoing roads too.
- `junctions.connection.connect_road_no_incoming_road`: connecting roads shall not be incoming roads.
- Connecting road: `@junction` = the junction id; predecessor / successor are the incoming / outgoing
  roads (`elementType="road"` with `contactPoint`). Its reference line should follow the driving direction
  where that is unique. It may have left and right lanes.
- `junctions.connection.smooth_fit`: linked lanes fit smoothly (position, heading, width, elevation) as for
  road linkage.

## `<connection>` and `<laneLink>`
- `<connection>`: `@id` unique within the junction; `@incomingRoad`; `@connectingRoad` (common junctions
  only; never in direct ones - `junctions.connection.no_connecting_road_direct`); `@contactPoint` of the
  connecting road (`start` when it runs along the lane linkage, `end` when against:
  `junctions.connection.start_along_linkage` / `end_opposite_linkage`).
- `junctions.connection.one_link_to_incoming`: one `<connection>` per (incomingRoad, connectingRoad) pair;
  its `<laneLink>`s cover only lanes that lead into the junction.
- `<laneLink from to>`: from = incoming-road lane, to = connecting-road lane. Omitting laneLinks is
  deprecated. **A lane without a laneLink has no path through the junction.**
- One laneLink per connecting road: no lane change inside the junction
  (`no_lane_change_for_mult_con_roads`); several laneLinks on one connecting road allow lane changes
  (`lane_change_one_con_road`).
- Direct junctions: `@linkedRoad` instead of `@connectingRoad`; `@overlapZone` on one laneLink pair
  marks where the split / merge lanes overlap; only flat entries / exits.

## Lane linkage (11.6)
- Lanes link only where they physically connect at both ends with non-zero width and are semantically
  the same lane; ambiguous -> junction (`road.lane.link.use_junctions`).
- Lanes that start or end in a junction carry no `<link>` toward it (`road.lane.link.no_link`): the path is
  given by the junction's laneLinks and the connecting road's own lane links.
- Zero width at a section start -> no predecessor, at an end -> no successor (`zero_width_at_start/end`).
- A lane that appears beside an existing one is not linked to the original lane (`new_lane_appear`).
- Continuing lanes across lane sections are linked both ways (`lanes_across_laneSections`).

## Priorities, boundary, surface
- `<priority high low>` only where no signals are defined (`junctions.priority.no_signals`).
- `<boundary>` (1.8+, optional, common junctions only): counter-clockwise `lane` / `joint` segments
  reaching every connected road end and closing the outline; it should enclose the traffic area including
  sidewalks. `<elevationGrid>` gives the junction surface inside it.

## The illustrated model (OpenDRIVE 1.6.0 Quick Read, figures 35-37, 59, 68, 74-81)
`Quick_Read_ASAM_OpenDRIVE_BS_V1-6-0.html` (asam.net dumpFile 3495) is stored next to this file.
- **Road linkage (fig. 35, 36):** linked roads meet with continuous reference lines; a gap or overlap of
  the reference lines is prohibited; small lane gaps / overlaps are a warning; the target is a smooth
  continuation. Roads may meet end-start, end-end or start-start (lane ids then flip sign); a direct link
  whose lane layouts are shifted (different lane offsets, lanes not lining up) is prohibited.
- **A split needs a junction (fig. 37):** one road dividing into two (a ramp leaving) is a junction whose
  connecting roads start together at the end of the incoming road, one per lane movement (lane -1 to the
  ramp, lane -2 straight on into the continuing road's -1), and end at the outgoing roads.
- **Junction area (fig. 74):** a polygon around the junction; every incoming road ends on its edge;
  connecting roads run inside it from the lane ends of the incoming roads to the lane starts of the
  outgoing roads. A oneway road that only leaves the junction is not an incoming road (no connection
  starts on it); connecting roads lead into it.
- **Connecting roads per lane and per movement (fig. 76):** each incoming lane is connected according to
  its turn arrow - the left-turn lane to the left road, through lanes straight across into the matching
  lanes, the right-turn lane to the right road; one connecting road per lane pair (no lane change in the
  junction); left turns end in the near lane of the target road, movements of one approach do not cross.
- **Lane links across roads (fig. 59):** a lane's predecessor / successor id follows the road contact:
  end-to-start keeps the id, end-to-end flips its sign.
- **Motorway entries / exits (fig. 68):** the auxiliary lane on the motorway is lane type `exit` /
  `entry`, the ramp is `connectingRamp` (between motorways) or `offRamp` / `onRamp`.
- **Virtual junction (fig. 77, 78):** a driveway / parking entry joins a road without interrupting it:
  the main road runs on; connecting roads branch off its lanes at given s.
- **Roundabout (fig. 79):** one junction per arm where it meets the ring, the ring between them plain
  roads, all of them grouped in a junction group of type roundabout.
- **Controllers (fig. 81):** the signals of one phase share a controller; opposite approaches one
  controller each, referenced by the junction.
