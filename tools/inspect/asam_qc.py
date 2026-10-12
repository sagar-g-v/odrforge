"""ASAM Quality Checker (qc-opendrive) on OdrForge output: an independent official check of Annex F rules.

    python asam_qc.py FILE.xodr [FILE.xodr ...]     # writes FILE.xqar next to a summary per file
    python asam_qc.py --json OUT.json FILE.xodr ... # also writes {file: {rule uid: issue count}}

The checker lives in its own Python 3.12 environment (C:/mywork/tools/qc-opendrive-venv, installed from
github.com/asam-ev/qc-opendrive at commit 8baf0a5, MPL-2.0). It covers ~22 Annex F rules plus XSD
validation (1.1 - 1.8.1); docs/RULES.md lists which rules OdrForge checks itself.

Known checker limitation: road.geometry.contact_point skips only roads of junction id 1, so it also
reports connecting roads. Those reports are kept: the ASAM Junction Guideline requires connecting-road
reference lines to continue the incoming road's reference line as well.
"""
import argparse
import collections
import concurrent.futures
import json
import os
import subprocess
import xml.etree.ElementTree as ET

CHECKER = r"C:\mywork\tools\qc-opendrive-venv\Scripts\qc_opendrive.exe"
CONFIG = """<?xml version="1.0" encoding="UTF-8"?>
<Config>
  <Param name="InputFile" value="{xodr}"/>
  <CheckerBundle application="xodrBundle">
    <Param name="resultFile" value="{xqar}"/>
  </CheckerBundle>
</Config>
"""


def run_checker(xodr):
    xodr = os.path.abspath(xodr)
    stem = os.path.splitext(xodr)[0]
    config, xqar = stem + ".qc.xml", stem + ".xqar"
    with open(config, "w", encoding="utf-8") as f:
        f.write(CONFIG.format(xodr=xodr, xqar=xqar))
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    subprocess.run([CHECKER, "-c", config], cwd=os.path.dirname(xodr), env=env,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return xqar


def summarise(xqar):
    """Issue count per rule uid, plus checkers that did not complete."""
    root = ET.parse(xqar).getroot()
    counts = collections.Counter(issue.get("ruleUID") for issue in root.iter("Issue"))
    not_completed = [c.get("checkerId") for c in root.iter("Checker") if c.get("status") not in ("completed", "skipped")]
    return dict(counts), not_completed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+")
    parser.add_argument("--json")
    args = parser.parse_args()
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        results = dict(zip(args.files, pool.map(lambda f: summarise(run_checker(f)), args.files)))
    for xodr, (counts, not_completed) in results.items():
        print(os.path.basename(xodr), "clean" if not counts else "")
        for uid, n in sorted(counts.items(), key=lambda kv: -kv[1]):
            print(f"  {n:7d}  {uid}")
        for checker in not_completed:
            print("  NOT COMPLETED", checker)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({xodr: counts for xodr, (counts, _) in results.items()}, f, indent=1)


if __name__ == "__main__":
    main()
