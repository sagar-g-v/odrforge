"""annex_f.py must flag planted violations: each mutation of a clean file trips exactly its rule.

    python annex_f_selftest.py CLEAN.xodr
"""
import sys
import xml.etree.ElementTree as ET

import annex_f


def first(root, path):
    found = root.find(path)
    assert found is not None, f"sample has no {path}"
    return found


def swap_first_two(parent, tag):
    a, b = parent.findall(tag)[:2]
    a_s, b_s = a.get("s"), b.get("s")
    a.set("s", b_s)
    b.set("s", a_s)


MUTATIONS = {
    "road.elevation.elem_asc_order": lambda root: swap_first_two(
        next(p for p in root.iter("elevationProfile") if len(p.findall("elevation")) >= 2), "elevation"),
    "road.length_sum_geometries": lambda root: first(root, "road").set("length", "123456"),
    "road.geometry.arc.no_zero_curvature": lambda root: first(root, ".//arc").set("curvature", "0"),
    "road.lane.center_lane": lambda root: first(root, ".//laneSection/center").remove(
        first(root, ".//laneSection/center/lane")),
    "road.lane.width.width_defined_whole_section": lambda root: first(root, ".//right/lane/width").set("sOffset", "1"),
    "road.linkage.road_link_attribute_usage": lambda root: next(
        e for e in root.iter() if e.tag in ("predecessor", "successor") and e.get("elementType") == "road"
    ).attrib.pop("contactPoint"),
    "junctions.type_default_no_linked_road": lambda root: first(root, ".//junction/connection").set("linkedRoad", "1"),
}


def main(path):
    failures = 0
    for name, mutate in MUTATIONS.items():
        root = ET.parse(path).getroot()
        mutate(root)
        found = list(annex_f.RULES[name](root))
        status = "ok " if found else "MISSED"
        failures += not found
        print(f"{status} {name}: {len(found)}")
    clean = annex_f.check_file(path)
    noise = {name: len(v) for name, v in clean.items() if v}
    print("clean file:", noise or "clean")
    return 1 if failures or noise else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
