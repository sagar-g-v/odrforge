"""Rule catalogue: every ASAM OpenDRIVE checker rule (1.9.0 Annex F) and where OdrForge checks it.

    python rule_catalogue.py            # writes docs/rules/annex_f_rules.json and docs/RULES.md

Sources
- docs/reference/asam_opendrive_1.9.0/16_annexes/map_rules.html: the Annex F rules (UID, section,
  description). A UID reads asam.net:xodr:<version introduced>:<rule path>.
- src/validate.cpp: rule ids passed to err(...) / warn(...). An id equal to an Annex F rule path is
  that rule's check.
- docs/rules/rule_map.json (hand-kept): rule path -> {"status", "check", "note"} for rules checked
  elsewhere (tools/inspect, tests) or that do not apply to generated maps.

Status per rule: checked (an exact check exists), not_applicable (the generator never writes the
construct, with the reason), open (no check yet).
"""
import html
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
from annex_f import RULES as ANNEX_F_CHECKS  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MAP_RULES = os.path.join(REPO, "docs", "reference", "asam_opendrive_1.9.0", "16_annexes", "map_rules.html")
VALIDATOR = os.path.join(REPO, "src", "validate.cpp")
RULE_MAP = os.path.join(REPO, "docs", "rules", "rule_map.json")
RULES_JSON = os.path.join(REPO, "docs", "rules", "annex_f_rules.json")
RULES_MD = os.path.join(REPO, "docs", "RULES.md")

# Rules the official ASAM checker (asam_qc.py, qc-opendrive 8baf0a5) implements.
ASAM_QC_CHECKS = {
    "road.geometry.contact_point", "road.geometry.elem_asc_order", "performance.avoid_redundant_info",
    "road.lane.level_true_one_side", "road.lane.access.no_mix_of_deny_or_allow",
    "road.lane.link.lanes_across_laneSections", "road.lane.link.new_lane_appear",
    "road.lane.link.zero_width_at_start", "road.lane.link.zero_width_at_end", "road.linkage.is_junction_needed",
    "junctions.connection.connect_road_no_incoming_road", "junctions.connection.one_link_to_incoming",
    "junctions.connection.start_along_linkage", "junctions.connection.end_opposite_linkage",
    "road.lane.border.overlap_with_inner_lanes", "road.geometry.paramPoly3.length_match",
    "road.geometry.paramPoly3.arcLength_range", "road.geometry.paramPoly3.normalized_range",
    "road.geometry.paramPoly3.valid_parameters",
}

UID = re.compile(r"asam\.net:xodr:([0-9.]+):([A-Za-z0-9_.\-]+)")
SECTION = re.compile(r"^(F(?:\.\d+)+)\s+(\S.*)$")


def page_lines(path):
    text = open(path, encoding="utf-8").read()
    text = re.sub(r"<script.*?</script>|<style.*?</style>", "", text, flags=re.S)
    text = html.unescape(re.sub(r"<[^>]+>", "\n", text))
    return [line.strip() for line in text.split("\n") if line.strip()]


def annex_f_rules():
    """Rules in document order: section, uid, version, path, level (shall / should), description."""
    lines = page_lines(MAP_RULES)
    rules, section = [], ""
    for i, line in enumerate(lines):
        heading = SECTION.match(line)
        if heading:
            section = heading.group(1)
        uid = UID.fullmatch(line)
        if not uid:
            continue
        description = []
        for following in lines[i + 2:]:
            if SECTION.match(following) or following == "UID":
                break
            description.append(following)
        text = re.sub(r"\s+([.,;:)])", r"\1", " ".join(description))
        text = re.sub(r"<\s+(\w+)\s+>", r"<\1>", text)
        level = "shall" if " shall " in f" {text} " else "should" if " should " in f" {text} " else "info"
        rules.append(dict(section=section, uid=line, version=uid.group(1), path=uid.group(2),
                          level=level, description=text))
    return rules


def strip_next_heading(rules):
    """The page puts the next rule's short name right after a description; drop it."""
    for rule, following in zip(rules, rules[1:]):
        leaf = following["path"].rsplit(".", 1)[-1]
        if rule["description"].endswith(" " + leaf):
            rule["description"] = rule["description"][: -len(leaf) - 1]


def hand_entry(hand, path):
    """Exact path first, then the longest matching "prefix.*" entry."""
    if path in hand:
        return hand[path]
    prefixes = [key[:-1] for key in hand if key.endswith(".*") and path.startswith(key[:-1])]
    return hand[max(prefixes, key=len) + "*"] if prefixes else {}


def validator_rule_ids():
    source = open(VALIDATOR, encoding="utf-8").read()
    return sorted(set(re.findall(r'\b(?:err|warn)\("([^"]+)"', source)))


def catalogue():
    rules = annex_f_rules()
    strip_next_heading(rules)
    validator_ids = set(validator_rule_ids())
    hand = json.load(open(RULE_MAP, encoding="utf-8")) if os.path.exists(RULE_MAP) else {}
    for rule in rules:
        entry = hand_entry(hand, rule["path"])
        checks = [name for name, implemented in (
            ("validate.cpp", rule["path"] in validator_ids),
            ("annex_f.py", rule["path"] in ANNEX_F_CHECKS),
            ("ASAM qc-opendrive", rule["path"] in ASAM_QC_CHECKS)) if implemented]
        if entry.get("status") == "checked":
            checks.append(entry["check"])
        if checks:
            rule.update(status="checked", check="; ".join(checks), note=entry.get("note", ""))
        else:
            rule.update(status=entry.get("status", "open"), check=entry.get("check", ""), note=entry.get("note", ""))
    own = sorted(validator_ids - {r["path"] for r in rules})
    return rules, own


def write_markdown(rules, own):
    counts = {s: sum(r["status"] == s for r in rules) for s in ("checked", "not_applicable", "open")}
    out = ["# Rule catalogue",
           "",
           "Generated by `tools/inspect/rule_catalogue.py` from ASAM OpenDRIVE 1.9.0 Annex F "
           "(`docs/reference/asam_opendrive_1.9.0/16_annexes/map_rules.html`), `src/validate.cpp` and "
           "`docs/rules/rule_map.json`. Do not edit by hand.",
           "",
           f"{len(rules)} Annex F rules: **{counts['checked']} checked**, {counts['not_applicable']} not applicable "
           f"to generated maps, **{counts['open']} open**.",
           "",
           "| Section | Rule | Since | Level | Status | Check / reason |",
           "|---|---|---|---|---|---|"]
    for r in rules:
        detail = r["check"] or r["note"]
        out.append(f"| {r['section']} | `{r['path']}` | {r['version']} | {r['level']} | {r['status']} | {detail} |")
    out += ["",
            "## OdrForge rules outside Annex F",
            "",
            "Rule ids in `src/validate.cpp` that are not Annex F paths (quality checks, CARLA limits, and older "
            "ids still to be renamed to their Annex F path).",
            ""]
    out += [f"- `{rule_id}`" for rule_id in own]
    with open(RULES_MD, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    return counts


def main():
    rules, own = catalogue()
    os.makedirs(os.path.dirname(RULES_JSON), exist_ok=True)
    with open(RULES_JSON, "w", encoding="utf-8", newline="\n") as f:
        json.dump(rules, f, indent=1, ensure_ascii=False)
    counts = write_markdown(rules, own)
    print(f"{len(rules)} Annex F rules: {counts}; {len(own)} OdrForge rule ids outside Annex F")


if __name__ == "__main__":
    main()
