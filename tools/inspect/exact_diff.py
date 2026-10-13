"""Exact checkers (invariants.py, laneflow.py, annex_f.py) on two sets of converted areas, and every count
that changed. Used to judge one converter change without a full gate run.

    python exact_diff.py BEFORE_DIR AFTER_DIR [--profile carla] [--areas la,munich]

The directories hold <area>.<profile>.xodr files as written by same_output.py.
"""
import argparse
import concurrent.futures
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import annex_f  # noqa: E402
from gate import AREAS, DATA  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def clip_of(area):
    args = AREAS[area]["args"]
    return args[args.index("--clip") + 1] if "--clip" in args else None


def exact_counts(xodr, area):
    counts = {}
    for tool in ("invariants", "laneflow"):
        out = xodr + "." + tool
        cmd = [sys.executable, os.path.join(HERE, tool + ".py"), xodr, AREAS[area]["osm"], "--out", out, "--crops", "0"]
        if clip_of(area):
            cmd += ["--clip", clip_of(area)]
        subprocess.run(cmd, cwd=DATA, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        result = os.path.join(out, "invariants.json" if tool == "invariants" else "findings.json")
        if not os.path.exists(result):
            counts[f"{tool}:CRASHED"] = 1
            continue
        data = json.load(open(result, encoding="utf-8"))
        if tool == "invariants":
            counts.update({f"inv:{k}": v for k, v in data["summary"].items() if isinstance(v, int)})
        else:
            counts.update({f"lf:{k}": v["count"] for k, v in data.items()
                           if isinstance(v, dict) and v.get("severity") not in ("ok", "info") and "count" in v})
    counts.update({f"annexF:{k}": len(v) for k, v in annex_f.check_file(xodr).items() if v})
    return counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("before")
    ap.add_argument("after")
    ap.add_argument("--profile", default="carla")
    ap.add_argument("--areas", default=",".join(AREAS))
    args = ap.parse_args()
    areas = args.areas.split(",")
    jobs = [(area, d) for area in areas for d in (args.before, args.after)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        # absolute paths: the checkers run with the data directory as their working directory
        results = list(pool.map(lambda j: exact_counts(os.path.abspath(os.path.join(j[1], f"{j[0]}.{args.profile}.xodr")), j[0]), jobs))
    worse = 0
    for k, area in enumerate(areas):
        before, after = results[2 * k], results[2 * k + 1]
        changes = {key: (before.get(key, 0), after.get(key, 0)) for key in sorted(set(before) | set(after))
                   if before.get(key, 0) != after.get(key, 0)}
        crashed = sorted(k for k in set(before) | set(after) if k.endswith(":CRASHED"))
        if crashed:  # a checker that did not run proves nothing, also when it failed on both sides
            print(f"== {area}: CHECKER FAILED ({', '.join(crashed)})")
            worse += 1
            continue
        print(f"== {area}: " + ("unchanged" if not changes else ""))
        for key, (b, a) in changes.items():
            print(f"   {key:55s} {b:7d} -> {a:7d}")
            worse += a > b
    return 1 if worse else 0


if __name__ == "__main__":
    sys.exit(main())
