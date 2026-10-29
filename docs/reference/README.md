# docs/reference

OdrForge's own notes on the ASAM OpenDRIVE standard:

- `opendrive_junction_rules.md`: the junction rules OdrForge follows (read by `tools/inspect/junction_audit.py`).
- `spec_study/*.json`, `opendrive_spec_atlas.html` (+ `spec_atlas.plan.html`, `spec_atlas.template.html`): a
  section-by-section study of OpenDRIVE 1.7 - 1.9 and the Junction Guideline 1.0.0 in our own words - summaries,
  rules, element lists, what changed between versions - with section numbers into the specification.

The specifications themselves (ASAM OpenDRIVE 1.6 - 1.9 and the ASAM OpenDRIVE Junction Guideline 1.0.0) are
copyright ASAM e.V. and are not included. Get them from ASAM:
https://www.asam.net/standards/detail/opendrive/

Some inspection tools read a local copy of the HTML specification if you have one: put it in
`docs/reference/asam_opendrive_1.8.0/`, `asam_opendrive_1.9.0/` or `asam_opendrive_junction_guideline_1.0.0/`
(these folders are ignored by git), e.g. `tools/inspect/rule_catalogue.py` reads the Annex F rules from
`asam_opendrive_1.9.0/16_annexes/map_rules.html`.
