"""The versioned prompt of the optional LLM review (llm_review.py), kept in code so every run uses a known text.

    python llm_review.py prompt           # print it
    python llm_review.py check-prompt     # is it stale? (checkers, spec targets or OdrForge version changed)

Keep evolving it: whenever check-prompt warns, read what changed (a new checker, a changed checker, a new header
revMinor, a new OdrForge version), revise the text below where it no longer holds, raise PROMPT_VERSION, set REVISED,
and record the new fingerprint (python llm_review.py check-prompt --print-fingerprint prints the CHECKERS lines).
Never edit the fingerprint without reading what changed: the fingerprint is the record that the prompt was reviewed
against those versions.
"""

PROMPT_VERSION = "1.1.2"  # 1.0.1: four checkers fixed (lane centres, LHT, direct junctions); 1.1.0: underpasses, buried roads; 1.1.2: junction pits
REVISED = "2026-10-17"
ODRFORGE_VERSION = "0.6.0"  # the repository's VERSION file when the prompt was last revised

# OpenDRIVE versions the outputs are written in (header revMinor -> profile), and the documents a verdict may cite
TARGET_REV_MINORS = {8: "standard profile (ASAM OpenDRIVE 1.8.0)", 4: "carla profile (OpenDRIVE 1.4H, CARLA's loader)"}
SPECS = {
    "1.8.0": "ASAM OpenDRIVE 1.8.0 (docs/reference/asam_opendrive_1.8.0): the standard profile's target",
    "1.9.0": "ASAM OpenDRIVE 1.9.0 Annex F checker rules (docs/rules/annex_f_rules.json, docs/RULES.md)",
    "1.4H": "OpenDRIVE 1.4 Rev. H as read by CARLA 0.9.x: the carla profile's target",
    "JG-1.0.0": "ASAM OpenDRIVE Junction Guideline 1.0.0 (docs/reference/asam_opendrive_junction_guideline_1.0.0)",
}

# The deterministic inspectors the prompt knows: file -> (fingerprint of its text when the prompt was revised, what it
# measures). check-prompt warns on a new, changed or removed file. Fingerprint: sha256 of the file with LF line ends,
# first 12 hex digits.
CHECKERS = {
    "annex_d_vertical.py": ("a1260a87cbfc", "1.9 Annex D.1.4/1.5: z, roll and pitch match where drivable lanes connect"),
    "annex_f.py": ("f0ab5df9b3a8", "1.9 Annex F checker rules on the written file"),
    "asam_qc.py": ("e46f2cd1eea8", "the official ASAM Quality Checker (qc-opendrive) on the output"),
    "barrier_audit.py": ("5fb7c3866e87", "barrier objects vs 1.8.0 13.14.1 / 13.4 and the OSM barriers"),
    "boundary_audit.py": ("be0a7fdca3b1", "junction boundaries vs 1.8.0 12.10 / 12.11"),
    "crosspath_audit.py": ("839eab453412", "crossPath elements vs 1.9 12.5 / 12.7.1"),
    "det_edge_smoothness.py": ("3b580567d2c7", "lane borders C1-smooth, no folds"),
    "det_junction_gore.py": ("c7eaff690c1e", "junction and gore quality on every connecting lane's centre, RHT and LHT"),
    "det_lane_transitions.py": ("9be2c8e15144", "lane count / width transitions, jogs, tapers; RHT and LHT"),
    "det_link_continuity.py": ("cef8bb005d62", "lane-link continuity, open lane ends at default and direct junctions"),
    "det_osm_fidelity.py": ("e04783e4e79f", "plain roads on their OSM ways, lane counts as tagged; RHT and LHT"),
    "gate.py": ("e775b3a6da6c", "runs everything per reference area against a baseline"),
    "grid_audit.py": ("c92fc7521596", "junction reference lines and elevation grids vs 1.8.0 12.9 / 12.11"),
    "invariants.py": ("0eb83fe54a39", "exact G0/G1/G2, widths, folds, flow, sidewalk on driving"),
    "junction_audit.py": ("786a25120ea2", "junction rules of docs/reference/opendrive_junction_rules.md"),
    "junction_rules_audit.py": ("7cdb757ad532", "direct / virtual junctions, lane heights vs 1.9"),
    "kerb_match.py": ("986803c51bbb", "sidewalk curb edges vs OSM barrier=kerb"),
    "laneflow.py": ("9be67690813f", "drive every lane: dead ends, orphans, traps, unreachable, islands"),
    "metrics.py": ("04266a58e2b2", "metrics snapshot per output, before / after"),
    "odrgeom.py": ("ba69e6c1160a", "exact OpenDRIVE evaluation and OSM projection used by the review facts"),
    "ped_signal_audit.py": ("975da8df6dae", "pedestrian lights on cross paths vs JG 6.3.3"),
    "signal_rules_audit.py": ("4e80dfc679c0", "traffic-light control vs 1.8.0 14 and JG"),
    "terrain_contract.py": ("05d227008d9f", "absolute terrain limits of the reference areas"),
}
# files of tools/inspect that are not checkers (selftests, planting tools, servers, meta tools and the review itself)
NOT_CHECKERS = {"exact_diff.py", "flags.py", "fuzz.py", "jev.py", "jev_judge.py", "laneflow_plant.py", "llm_review.py",
                "llm_review_prompt.py", "rule_catalogue.py", "same_output.py", "viz_server.py"}

# Verdict vocabulary. category: the viewer's flag categories (odrforge-viewer src/review/flags.ts CATEGORIES), so a
# review flag shows like a hand-made one; kind: the finer class; perspective: whose real-world use it breaks.
CATEGORIES = ("geometry", "lanes", "connection", "junction", "missing", "signal", "elevation", "other")
KINDS = ("lane_ends_abruptly", "leads_nowhere", "unreachable", "missing_turn", "impossible_turn", "wrong_way_link",
         "merge_without_taper", "lane_drop_without_taper", "width_implausible", "curvature_implausible",
         "junction_path_odd", "gap_or_kink", "overlap_or_sliver", "fold", "sidewalk_ends", "crosswalk_unreached",
         "walk_through_traffic", "cycle_path_broken", "spawn_or_route_problem", "link_asymmetric",
         "osm_turn_not_covered", "osm_deviation", "spec_violation", "other")
PERSPECTIVES = ("car", "pedestrian", "wheelchair", "cyclist", "simulator", "visual", "logical")
SEVERITIES = ("critical", "major", "minor", "info")
VERDICTS = ("defect", "uncertain", "legitimate")


def text():
    """The prompt given to the reviewer (a Claude Code session or claude -p) together with one chunk of a bundle."""
    checkers = "\n".join(f"  - {k}: {v[1]}" for k, v in sorted(CHECKERS.items()))
    targets = "\n".join(f"  - revMinor {k}: {v}" for k, v in sorted(TARGET_REV_MINORS.items()))
    specs = "\n".join(f"  - {k}: {v}" for k, v in SPECS.items())
    return f"""# OdrForge LLM review, prompt {PROMPT_VERSION} (revised {REVISED}, OdrForge {ODRFORGE_VERSION})

You review OpenDRIVE road networks that OdrForge generated from OpenStreetMap. Deterministic checkers have already
measured the files; your job is the judgement they cannot make: would this network work in the real world, for every
road user, and does it look and behave like the place it models? You review one chunk at a time. A chunk holds a few
spots; each spot has measured facts, the candidate findings of the checkers there, and a rendered plan view.

## Targets
{targets}
Read the header revMinor in the chunk's file facts and judge against that version. Features a version does not have
(1.8.0 junction boundaries, elevation grids, direct junctions, the walking lane type) are not defects in the other.
Documents you may cite:
{specs}

## What you get
- chunk_NN.md: the file facts (counts, lanes by type, open ends, connectivity), then per spot: why it was chosen,
  measured facts (roads, lanes and widths at the ends, links, junction connections, gaps, OSM ways and their tags),
  and the candidates of the checkers below.
- spot_*.png: plan view in the file's own metres (x east, y north, scale bar and axis ticks). Colours: driving dark
  grey, walking / sidewalk light grey, biking green, parking blue, shoulder / border / curb tan, median / restricted
  purple, other lanes mauve; lanes inside a junction drawn lighter. Thin white lines: lane borders. Magenta: reference
  lines. Small dark arrows: driving direction of each driving lane. Orange lines: junction connections (connecting
  road centre lines). Red X: an open lane end (a lane that stops with width > 0.3 m and no continuation). Red circles
  numbered: the candidates. Cyan dashed: OSM highway ways with their way id; dotted cyan: OSM footways / crossings.
  Crosswalks: white hatched rectangles. A satellite or map background is drawn when the bundle has one.
- The checkers whose findings appear as candidates:
{checkers}
  And the review's own facts (llm_review.py): open lane ends, dead ends against the OSM node degree, zero-width
  lanes, gaps and kinks at lane joints, lane drops / merges and their taper, crosswalk ends against walking lanes,
  junction turns OSM allows but the file does not connect, wrong-way links, the driving-lane graph's components.

## How to judge
Separate what is measured from what you conclude. Every verdict lists its evidence as measured facts copied from the
chunk (numbers with units, ids) and then, separately, your reasoning. Never invent a number; if you read something off
the image, say so ("seen in spot_3.png"). Where a rule of a specification applies, cite it (e.g. "1.8.0 11.5: lane
links only where both lanes have non-zero width at the connection"). Where none applies, say the judgement is
real-world plausibility.

Judge every spot from each perspective that applies:
- Car driver: does each lane continue to somewhere a car can go? Lanes that end abruptly (width > 0.3 m and no link),
  lead nowhere (dead end where OSM continues), are unreachable, missing turns OSM allows (no turn restriction), turns
  that are impossible (lane-centre radius under 5 m - a car's turning centreline is at least 6.4 m - or a path that
  leaves the carriageway), merges or lane drops with no taper or a taper far too short for the speed, links that send
  a lane against its direction, implausible widths (driving lane under 2.5 m or over 4.5 m outside a taper or a
  junction, unexplained), sudden curvature, and junction paths that cross where real paths would not (e.g. connecting
  roads of one approach crossing each other, a left turn through a median).
- Pedestrian and wheelchair user: sidewalks (walking lanes) that end in the middle of a block, crosswalks that do not
  reach a walking lane (or a refuge island, or a mapped footway) on both sides, walking lanes running through driving
  lanes, missing curb / kerb where the walking lane meets traffic, steps or gaps a wheelchair cannot pass.
- Cyclist: cycle lanes that start or end with nowhere to go, that cut across traffic without a junction, or that
  are drawn where OSM has no cycleway.
- Simulator (CARLA, esmini): a vehicle spawned on any driving lane must be able to drive on (no trap) and every lane
  must be reachable; dead ends must be where the area is clipped or where OSM maps a dead end; tiny islands of lanes
  break route planning; very short roads and lane sections with no length break spawn points.
- Visual: overlaps of carriageways that are not a junction, slivers (lanes a few cm wide over metres), gaps between
  lanes that should touch, folds (a border crossing itself), kinks where two roads meet.
- Logical / programmatic: link symmetry (if A's successor is B, B links back to A), every OSM turn at a junction
  covered by a connection unless a restriction or the geometry forbids it, junction membership, the rules above.

## Not a defect (legitimate real-world geometry)
Mark these legitimate, with the reason, so they stop coming back:
- dead ends at the clip boundary of the area (the facts say "at boundary"), and dead ends OSM maps itself
  (noexit=yes, a way that ends with no other highway way at its last node, turning circles);
- a lane that ends with width reaching 0 m (a taper) and a lane that ends at a junction whose connections take it on;
- turns forbidden by an OSM turn restriction, by a one-way, or by a median / barrier in OSM; U-turns not modelled;
- sidewalks that end where OSM's sidewalk tag or footway ends, at a driveway, a building entrance, a car park or at
  the clip boundary; crosswalks to a refuge island or a mapped footway rather than a sidewalk;
- slip lanes, gores, braided ramps, multi-level interchanges (roads crossing at different heights do not overlap),
  hairpins on mountain roads (Stelvio), narrow old-town streets, left-hand traffic (London, Tokyo: lanes drive on the
  left, ids > 0 drive along s);
- a checker candidate the picture and facts show is correct; say which fact clears it.
When you cannot tell from the facts and the image, the verdict is "uncertain" with what would decide it.

## Real-world ranges (what is built; docs/research/ROAD_DESIGN_SENSIBILITY.md has the sources [Sn])
Judge against these ranges, not against one country's rule; say which region's practice you apply. A value outside
a range is a reason to look, not a verdict by itself.
- Lane widths: urban 2.75-3.5 m (US 3.0-3.6, NACTO 3.05; Japan 3.25 / 3.0, small-car roads 2.75; DE two-lane
  carriageway 6.5-7.0 m) [S3][S4][S27][S35]; motorway 3.5-3.75 m [S17][S37]; turn lanes 2.5-3.5 m, at most 5.0 m
  urban [S19][S27]; bus lanes 3.0-4.5 m, 2.75 m at stops [S7][S8]. Two fused lanes look like one 5-7 m lane.
- Tapers: MUTCD lane-reduction taper L = W*S^2/60 below 45 mph, W*S from 45 mph (ft, mph; 12 ft lane: ~38 m at
  25 mph, ~55 m at 30 mph) [S2]; width change 1:25 at 50 km/h to 1:55 at 120 km/h [S17]; urban turn bays 25-40 m
  including a 5 m direct taper are legitimate [S19]; motorway merges 75-205 m [S20].
- Turning: a car's turning centreline radius is >= 6.4 m (minimum turning radius 7.25 m), a WB-62 truck 13.7 m
  [S11][S12]; urban kerb radii 1.5-6 m (effective 4.5-6 m) [S5][S6][S19]; minimum curve radius ~80-180 m at
  50 km/h and 380-720 m at 100 km/h [S18][S26]; tight mountain hairpins are legitimate (the Stelvio has 48 + 34
  numbered hairpins [S33]; no hairpin radius rule was found, and the Italian road norm excludes mountain roads on
  difficult terrain [S32]); roundabouts ICD 28-100 m, mini up to 28 m [S21].
- Pedestrians: footway 1.5-2.5 m (1.0 m at short obstacles) [S22][S36]; US pedestrian access route >= 1.22 m, cross
  slope <= 2.1 %, kerb ramp <= 8.3 % [S15]; refuge island >= 1.5 m deep [S22]; crosswalk >= 1.8 m wide, stop line
  >= 1.2 m before it [S2]; urban crossing spacing 80-100 m, gaps over 200 m are unusual [S48]. A crosswalk must end
  on a footway, a refuge island or a dropped kerb on both sides; parking next to a crossing is banned, so crossing a
  parking strip or a curb lane to reach the footway is normal.
- Cyclists: cycle lane / track 1.5-2.0 m, under 1.25 m is unusual [S23][S24][S35]; cycle lanes legitimately end at
  junctions and resume after them, or end in an advanced stop box.
- Vertical: grades 5-10 % on streets, 3-4 % on motorways, more on mountain passes [S18][S26][S32]; superelevation
  2-7 % (Japan up to 10 %) [S18][S26]; clearance under structures 4.5-5.3 m [S17][S26].
- Networks: a typical neighbourhood has 10-30 % dead-end nodes (cul-de-sacs, hammerheads, bollard-filtered
  streets) [S46]; above 40 % suggests broken links.
- Simulators: CARLA's Traffic Manager routes over connected waypoints and picks a random direction at junctions, so
  a junction lane with no laneLink stops or removes vehicles; abruptly ending roads at the map border can crash it
  in OSM mode (legitimate at the clip boundary, but say so for the simulator perspective) [S61][S62]; CARLA walkers
  only use sidewalk and crosswalk surfaces [S63]; esmini ignores virtual junctions, lane height and crossfall [S64].
Looks wrong but is legitimate: short urban turn bays, staggered T-junctions 40-130 m apart, overlapping roads at
different levels, lane lines that kink at a stop line where a lane splits into turn lanes, width steps of 0.25-0.5 m
between sections, single-lane old-town streets without sidewalks, opposing turns crossing in the junction centre
(both conventions exist [S25]), mini-roundabouts, slip lanes behind islands, scramble crossings, staggered
crossings, bus gates, tram tracks in a traffic lane, sidewalks on one side only, a road under the terrain inside a
<tunnel> record (type "underpass": OSM covered=yes stretches 2 m or more under the terrain, elevation.cpp
coveredUnderpasses) with the ground surface above it.
Looks fine but is wrong: a lane link across a median, kerbed island or tram reservation; a crosswalk ending in a
driving lane; a turn from a through-only lane or against turn:lanes / a restriction; a dual carriageway merged into
one two-way road; a road ending 1-10 m before another with no barrier (missing link); a sidewalk running through a
junction or across a ramp without a crossing; left-hand-traffic areas built with right-hand lane orientation;
elevation steps at links; incoming junction lanes with no laneLink; a plain road more than 2 m under the raw ground over
10 m or more with no tunnel / bridge record and no bridge over it (the terrain contract's "buried" check; tunnel records
are extended past a short OSM tunnel way to where the road meets the terrain); a connecting road between tunnel roads
with no tunnel record, the ground dug down to it (the contract's "pits" check).

## Output
For each chunk, write verdicts/chunk_NN.json (the run command tells you where): a JSON object
{{"chunk": "chunk_NN", "prompt_version": "{PROMPT_VERSION}", "reviewer": "<model or session name>",
  "verdicts": [ ... ]}}
One verdict per distinct issue (several candidates of one issue: one verdict listing all their ids), and one per
candidate you judge legitimate. Fields:
- "spot": the spot id; "candidates": the candidate ids it settles, copied from the square brackets of the chunk's
  list (e.g. "link_continuity:12"; the list number of the spot is accepted too); empty for an issue you found yourself;
- "verdict": {" | ".join(VERDICTS)};
- "category": {" | ".join(CATEGORIES)} (the viewer's categories);
- "kind": {" | ".join(KINDS)};
- "perspective": list of {" | ".join(PERSPECTIVES)};
- "severity": {" | ".join(SEVERITIES)} (critical: a road user or simulator cannot use it at all - a trap, a lane
  into nothing, a broken route; major: unrealistic and likely to cause wrong behaviour; minor: visible but usable;
  info: worth knowing);
- "x", "y": where in the file's metres (from the facts, not guessed from pixels); "road", "lane", "junction", "s"
  where known; "osm_ways": the OSM way ids involved (road ids change between builds; OSM ids do not);
- "evidence": list of measured facts (strings with numbers and units, copied from the chunk);
- "reasoning": the real-world judgement, per perspective;
- "spec": the rule cited, or "" for plausibility;
- "confidence": 0..1, your probability that the verdict is right (0.9+: facts and image agree and settle it; 0.6-0.8:
  likely; below 0.5: say uncertain instead);
- "note": one line for the viewer's list ("Left-turn lane of road 120 ends 2.8 m wide with no connection").
Output only the JSON file; do not edit the bundle or the network.
"""
