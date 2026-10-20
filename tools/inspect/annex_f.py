"""ASAM OpenDRIVE 1.9 Annex F rules checked on a written .xodr file.

    python annex_f.py FILE.xodr [...]          # violations per rule, first examples
    python annex_f.py --json OUT.json FILE...  # {file: {rule path: count}}

Each rule is one function registered with @rule("<Annex F rule path>"); the function yields a short
description per violation. tools/inspect/rule_catalogue.py marks every registered path as checked.
Rules that need geometry beyond simple XML (overlaps, smoothness) are checked elsewhere: see
docs/RULES.md. Section numbers are OpenDRIVE 1.9.0.
"""
import argparse
import collections
import json
import xml.etree.ElementTree as ET

RULES = {}


def rule(path):
    def register(check):
        RULES[path] = check
        return check
    return register


def num(element, attribute, default=0.0):
    value = element.get(attribute)
    return float(value) if value is not None else default


def road_label(road):
    return f"road {road.get('id')}"


def ascending(elements, attribute):
    """Index pairs where the attribute decreases (equal values allowed)."""
    values = [num(e, attribute) for e in elements]
    return [i for i in range(1, len(values)) if values[i] < values[i - 1] - 1e-9]


def lanes_of(section):
    for side in ("left", "center", "right"):
        group = section.find(side)
        if group is not None:
            for lane in group.findall("lane"):
                yield side, lane


def traffic_rule(road):
    return road.get("rule", "RHT")


# ---- ordering: every record list in ascending s (§10, §11, §13)

def order_rule(path, find_parents, child, attribute="s"):
    @rule(path)
    def check(root):
        for road in root.iter("road"):
            for parent in find_parents(road):
                if ascending(parent.findall(child), attribute):
                    yield f"{road_label(road)}: <{child}> not ascending in {attribute}"
    return check


order_rule("road.elevation.elem_asc_order", lambda r: r.findall("elevationProfile"), "elevation")
order_rule("road.superelevation.elem_asc_order", lambda r: r.findall("lateralProfile"), "superelevation")
order_rule("road.type.elem_asc_order", lambda r: [r], "type")
order_rule("road.lanes.lane_offset.elem_asc_order", lambda r: r.findall("lanes"), "laneOffset")
order_rule("road.lane_section.elem_asc_order", lambda r: r.findall("lanes"), "laneSection")
for _record in ("access", "height", "material", "speed", "rule"):
    order_rule(f"road.lane.{_record}.elem_asc_order", lambda r: r.iter("lane"), _record, "sOffset")
order_rule("road.lane.road_mark.elem_asc_order", lambda r: r.iter("lane"), "roadMark", "sOffset")


@rule("road.lane.lane_properties.elem_asc_order")
def lane_properties_ascending(root):
    for road in root.iter("road"):
        for lane in road.iter("lane"):
            for kind in ("width", "border"):
                if ascending(lane.findall(kind), "sOffset"):
                    yield f"{road_label(road)} lane {lane.get('id')}: <{kind}> not ascending"


# ---- §9 geometry

@rule("road.length_sum_geometries")
def length_is_sum_of_geometries(root):
    for road in root.iter("road"):
        total = sum(num(g, "length") for g in road.iter("geometry"))
        if abs(total - num(road, "length")) > 1e-6:
            yield f"{road_label(road)}: length {num(road, 'length')} vs geometries {total:.9f}"


@rule("road.geometry.arc.no_zero_curvature")
def arc_curvature_not_zero(root):
    for road in root.iter("road"):
        for arc in road.iter("arc"):
            if num(arc, "curvature") == 0.0:
                yield f"{road_label(road)}: arc with zero curvature"


@rule("road.geometry.spiral.curvature_change")
def spiral_curvature_changes(root):
    for road in root.iter("road"):
        for spiral in road.iter("spiral"):
            if num(spiral, "curvStart") == num(spiral, "curvEnd"):
                yield f"{road_label(road)}: spiral with equal start and end curvature"


# ---- §10 roads and linkage

@rule("road.linkage.road_link_attribute_usage")
def road_links_have_contact_point(root):
    for road in root.iter("road"):
        for end in ("predecessor", "successor"):
            link = road.find(f"link/{end}")
            if link is not None and link.get("elementType") == "road":
                if not link.get("elementId") or link.get("contactPoint") not in ("start", "end"):
                    yield f"{road_label(road)}: {end} road link without elementId / contactPoint"


@rule("road.type.create_new_type_in_parent")
def road_types_in_road(root):
    for road in root.iter("road"):
        types = road.findall("type")
        if road.get("junction", "-1") == "-1" and not types:
            yield f"{road_label(road)}: no <type>"


# ---- §11 lanes

@rule("road.lane.center_lane")
def every_section_has_center_lane(root):
    for road in root.iter("road"):
        for section in road.iter("laneSection"):
            if section.find("center/lane") is None:
                yield f"{road_label(road)} section s={section.get('s')}: no center lane"


@rule("road.lane.lane_listing")
def lanes_listed_left_to_right(root):
    for road in root.iter("road"):
        for section in road.iter("laneSection"):
            ids = [int(lane.get("id")) for _, lane in lanes_of(section)]
            if ids != sorted(ids, reverse=True):
                yield f"{road_label(road)} section s={section.get('s')}: lanes not listed by descending id"


@rule("road.lane.height.center_lane_no_height")
def center_lane_not_raised(root):
    for road in root.iter("road"):
        for lane in road.iterfind(".//center/lane"):
            if lane.find("height") is not None:
                yield f"{road_label(road)}: center lane with <height>"


@rule("road.lane.material.center_lane_no_material")
def center_lane_no_material(root):
    for road in root.iter("road"):
        for lane in road.iterfind(".//center/lane"):
            if lane.find("material") is not None:
                yield f"{road_label(road)}: center lane with <material>"


@rule("road.lane.level_true_one_side")
def level_lanes_reach_the_edge(root):
    for road in root.iter("road"):
        for section in road.iter("laneSection"):
            for side in ("left", "right"):
                lanes = sorted(section.findall(f"{side}/lane"), key=lambda lane: abs(int(lane.get("id"))))
                levels = [lane.get("level") == "true" for lane in lanes]
                if any(levels) and not all(levels[levels.index(True):]):
                    yield f"{road_label(road)} s={section.get('s')} {side}: level lane inside a non-level lane"


@rule("road.lane.access.no_mix_of_deny_or_allow")
def access_rules_not_mixed(root):
    for road in root.iter("road"):
        for lane in road.iter("lane"):
            by_s = collections.defaultdict(set)
            for access in lane.findall("access"):
                by_s[access.get("sOffset")].add(access.get("rule", "deny"))
            if any(len(rules) > 1 for rules in by_s.values()):
                yield f"{road_label(road)} lane {lane.get('id')}: deny and allow at one s"


@rule("road.lane.road_mark.only_outer")
def one_mark_set_per_lane_edge(root):
    """Marks describe the lane's outer edge: never two marks of one lane at the same sOffset."""
    for road in root.iter("road"):
        for lane in road.iter("lane"):
            offsets = [m.get("sOffset") for m in lane.findall("roadMark")]
            if len(offsets) != len(set(offsets)):
                yield f"{road_label(road)} lane {lane.get('id')}: two roadMarks at one sOffset"


@rule("road.lane.width.width_defined_whole_section")
def width_from_section_start(root):
    for road in root.iter("road"):
        for section in road.iter("laneSection"):
            for side, lane in lanes_of(section):
                widths = lane.findall("width")
                if side != "center" and (not widths or num(widths[0], "sOffset") > 1e-9):
                    yield f"{road_label(road)} lane {lane.get('id')}: width not defined from the section start"


@rule("road.lane.lane_reverse_left_right")
def reverse_not_used_for_traffic_side(root):
    for road in root.iter("road"):
        lanes = [lane for lane in road.iter("lane") if lane.get("type") == "driving" and lane.get("id") != "0"]
        if lanes and all(lane.get("direction") == "reversed" for lane in lanes):
            yield f"{road_label(road)}: every driving lane reversed (use road@rule for the traffic side)"


# ---- §12 junctions

@rule("junctions.type_default_no_linked_road")
def common_junctions_use_connecting_roads(root):
    for junction in root.iter("junction"):
        if junction.get("type", "default") in ("default", "virtual"):
            for connection in junction.findall("connection"):
                if connection.get("linkedRoad") is not None:
                    yield f"junction {junction.get('id')}: linkedRoad in a {junction.get('type', 'default')} junction"


@rule("junctions.type_direct_no_conn_road")
def direct_junctions_use_linked_roads(root):
    for junction in root.iter("junction"):
        if junction.get("type") == "direct":
            for connection in junction.findall("connection"):
                if connection.get("connectingRoad") is not None:
                    yield f"junction {junction.get('id')}: connectingRoad in a direct junction"


RULES["junctions.connection.no_connecting_road_direct"] = direct_junctions_use_linked_roads
RULES["junctions.direct.connecting_road_attribute_usage"] = direct_junctions_use_linked_roads


@rule("junctions.direct.correct_type_linked_road_usage")
def linked_road_only_in_direct(root):
    yield from common_junctions_use_connecting_roads(root)


@rule("junctions.common.junctions_no_pred_succ")
def junctions_have_no_links(root):
    for junction in root.iter("junction"):
        if junction.find("link") is not None:
            yield f"junction {junction.get('id')}: junction with <link>"


@rule("junctions.priority.high_and_low_attr")
def priorities_name_both_roads(root):
    for junction in root.iter("junction"):
        for priority in junction.findall("priority"):
            if not priority.get("high") or not priority.get("low"):
                yield f"junction {junction.get('id')}: priority without high and low"


@rule("junctions.priority.no_signals")
def priorities_only_without_signals(root):
    signalled = {road.get("junction") for road in root.iter("road")
                 if road.find("signals/signal[@dynamic='yes']") is not None}
    controlled = {j.get("id") for j in root.iter("junction") if j.find("controller") is not None}
    for junction in root.iter("junction"):
        if junction.find("priority") is not None and junction.get("id") in signalled | controlled:
            yield f"junction {junction.get('id')}: priorities and traffic lights together"


# ---- §13 objects, §14 signals: lane validity

def validity_ranges(parent):
    for validity in parent.findall("validity"):
        yield int(validity.get("fromLane")), int(validity.get("toLane"))


def lower_equal_rule(path, find):
    @rule(path)
    def check(root):
        for road in root.iter("road"):
            for element in find(road):
                for low, high in validity_ranges(element):
                    if low > high:
                        yield f"{road_label(road)} {element.tag} {element.get('id')}: fromLane {low} > toLane {high}"
    return check


lower_equal_rule("road.object.validty.from_lower_equal_to", lambda r: r.iterfind("objects/object"))
lower_equal_rule("road.object.bridges.from_lower_equal_to", lambda r: r.iterfind("objects/bridge"))
lower_equal_rule("road.object.tunnels.from_lower_equal_to", lambda r: r.iterfind("objects/tunnel"))
lower_equal_rule("road.object.reference.from_lower_equal_to", lambda r: r.iterfind("objects/objectReference"))
lower_equal_rule("road.signal.reference.from_lower_equal_to", lambda r: r.iterfind("signals/signalReference"))


def lane_side_rule(path, find, traffic):
    """§13.6 / §14.2: an element valid in s direction ("+") covers right-hand lanes (negative ids) in RHT,
    left lanes (positive ids) in LHT, and the reverse for "-"."""
    @rule(path)
    def check(root):
        for road in root.iter("road"):
            if traffic_rule(road) != traffic:
                continue
            for element in find(road):
                orientation = element.get("orientation")
                if orientation not in ("+", "-"):
                    continue
                forward_negative = traffic == "RHT"
                wants_negative = forward_negative if orientation == "+" else not forward_negative
                for low, high in validity_ranges(element):
                    if (wants_negative and high > 0) or (not wants_negative and low < 0):
                        yield (f"{road_label(road)} {element.tag} {element.get('id')}: orientation {orientation} "
                               f"in {traffic} but lanes {low}..{high}")
    return check


for _traffic, _side in (("LHT", "left"), ("RHT", "right")):
    lane_side_rule(f"road.object.validty.{_side}_hand_traffic_lane_ids", lambda r: r.iterfind("objects/object"), _traffic)
    lane_side_rule(f"road.signal.validity.{_side}_hand_traffic_lane_ids", lambda r: r.iterfind("signals/signal"), _traffic)
    lane_side_rule(f"road.signal.reference.{_side}_hand_traffic_lane_ids",
                   lambda r: r.iterfind("signals/signalReference"), _traffic)


@rule("road.object.orientation")
def objects_state_orientation(root):
    for road in root.iter("road"):
        for obj in road.iterfind("objects/object"):
            if obj.get("orientation") is None:
                yield f"{road_label(road)} object {obj.get('id')}: no orientation"


@rule("road.object.s_t_coords")
def objects_have_s_and_t(root):
    for road in root.iter("road"):
        for obj in road.iterfind("objects/object"):
            if obj.get("s") is None or obj.get("t") is None:
                yield f"{road_label(road)} object {obj.get('id')}: no s / t"


@rule("road.object.bridges.type_definition")
def bridges_have_type(root):
    for road in root.iter("road"):
        for bridge in road.iterfind("objects/bridge"):
            if not bridge.get("type"):
                yield f"{road_label(road)} bridge {bridge.get('id')}: no type"


@rule("road.object.tunnels.type_definition")
def tunnels_have_type(root):
    for road in root.iter("road"):
        for tunnel in road.iterfind("objects/tunnel"):
            if not tunnel.get("type"):
                yield f"{road_label(road)} tunnel {tunnel.get('id')}: no type"


@rule("road.corner_local.first_id_zero")
def corner_ids_start_at_zero(root):
    for road in root.iter("road"):
        for outline in road.iter("outline"):
            corners = outline.findall("cornerLocal")
            if corners and corners[0].get("id") not in (None, "0"):
                yield f"{road_label(road)}: first cornerLocal id is {corners[0].get('id')}"


@rule("road.corner_local.sequential_id_values")
def corner_ids_sequential(root):
    for road in root.iter("road"):
        for outline in road.iter("outline"):
            ids = [c.get("id") for c in outline.findall("cornerLocal")]
            if all(i is not None for i in ids) and [int(i) for i in ids] != list(range(len(ids))):
                yield f"{road_label(road)}: cornerLocal ids not 0, 1, 2, ..."


@rule("road.object.outline.outline_followed_by_corner")
def outlines_have_two_corners(root):
    for road in root.iter("road"):
        for outline in road.iter("outline"):
            if len(outline.findall("cornerLocal")) + len(outline.findall("cornerRoad")) < 2:
                yield f"{road_label(road)}: outline with fewer than two corners"


@rule("road.object.outline.exactly_one_outer")
def one_outer_outline(root):
    for road in root.iter("road"):
        for outlines in road.iter("outlines"):
            outer = [o for o in outlines.findall("outline") if o.get("outer", "true") == "true"]
            if len(outer) != 1:
                yield f"{road_label(road)}: {len(outer)} outer outlines"


@rule("road.signal.use_country_code")
def signals_name_country(root):
    for road in root.iter("road"):
        for signal in road.iterfind("signals/signal"):
            if not signal.get("country"):
                yield f"{road_label(road)} signal {signal.get('id')}: no country"


@rule("road.signal.reference.specify_direction")
def signal_references_have_orientation(root):
    for road in root.iter("road"):
        for reference in road.iterfind("signals/signalReference"):
            if reference.get("orientation") not in ("+", "-", "none"):
                yield f"{road_label(road)}: signalReference {reference.get('id')} without orientation"


@rule("road.signal.reference.used_for_signals_only")
def signal_references_point_at_signals(root):
    signal_ids = {s.get("id") for s in root.iter("signal")}
    for road in root.iter("road"):
        for reference in road.iterfind("signals/signalReference"):
            if reference.get("id") not in signal_ids:
                yield f"{road_label(road)}: signalReference {reference.get('id')} is not a signal"


# ---- header

@rule("header.offset.centered_coords")
def coordinates_near_origin(root):
    xs = [num(g, "x") for g in root.iter("geometry")]
    ys = [num(g, "y") for g in root.iter("geometry")]
    if xs and max(max(map(abs, xs)), max(map(abs, ys))) > 1e5:
        yield "coordinates exceed 100 km from the origin: use <offset>"


def check_file(path):
    root = ET.parse(path).getroot()
    return {name: list(check(root)) for name, check in RULES.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+")
    parser.add_argument("--json")
    args = parser.parse_args()
    summary = {}
    for path in args.files:
        violations = check_file(path)
        summary[path] = {name: len(v) for name, v in violations.items() if v}
        print(path, "clean" if not summary[path] else "")
        for name, found in violations.items():
            if found:
                print(f"  {len(found):6d}  {name}   e.g. {found[0]}")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=1)


if __name__ == "__main__":
    main()
