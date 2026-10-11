"""Validate generated OpenDRIVE files against the official ASAM XSD (XSD 1.1).

usage: python -I xsd_validate.py <schema dir> <file or directory> [...]
Exit code 1 if any file has schema errors.
"""
import glob
import os
import sys

import xmlschema


def main():
    schema = xmlschema.XMLSchema11(os.path.join(sys.argv[1], "OpenDRIVE_Core.xsd"))
    files = []
    for a in sys.argv[2:]:
        files += sorted(glob.glob(os.path.join(a, "*.xodr"))) if os.path.isdir(a) else [a]
    if not files:
        print("no .xodr files to validate")
        return 1
    bad = 0
    for f in files:
        errors = list(schema.iter_errors(f))
        print(f"{os.path.basename(f)}: {len(errors)} schema errors")
        for e in errors[:5]:
            print(f"    {e.path}: {(e.reason or '')[:160]}")
        bad += bool(errors)
    print(f"{len(files) - bad}/{len(files)} files valid against {os.path.basename(sys.argv[1])}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
