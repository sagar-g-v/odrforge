"""Validate generated OpenDRIVE files against the official ASAM XSD (XSD 1.1).

usage: python -I xsd_validate.py [--allow-lht-rule] <schema dir or .xsd file> <file or directory> [...]
Exit code 1 if any file has schema errors.

--allow-lht-rule: OpenDRIVE 1.4 has no road@rule (added in 1.5). The carla profile still writes
rule="LHT" on left-hand-traffic roads, because without it a reader drives every lane against its
traffic; this documented deviation is the only error the option accepts.
"""
import glob
import os
import sys

import xmlschema


def is_lht_rule(error):
    return (error.path or "").split("/")[-1].startswith("road") and "'rule' attribute not allowed" in (error.reason or "")         and error.elem is not None and error.elem.get("rule") == "LHT"


def main():
    args = sys.argv[1:]
    allow_lht_rule = "--allow-lht-rule" in args
    args = [a for a in args if a != "--allow-lht-rule"]
    schema_path = args[0] if args[0].endswith(".xsd") else os.path.join(args[0], "OpenDRIVE_Core.xsd")
    schema = xmlschema.XMLSchema11(schema_path)
    files = []
    for a in args[1:]:
        files += sorted(glob.glob(os.path.join(a, "*.xodr"))) if os.path.isdir(a) else [a]
    if not files:
        print("no .xodr files to validate")
        return 1
    bad = 0
    for f in files:
        errors = [e for e in schema.iter_errors(f) if not (allow_lht_rule and is_lht_rule(e))]
        print(f"{os.path.basename(f)}: {len(errors)} schema errors")
        for e in errors[:5]:
            print(f"    {e.path}: {(e.reason or '')[:160]}")
        bad += bool(errors)
    print(f"{len(files) - bad}/{len(files)} files valid against {os.path.basename(schema_path)}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
