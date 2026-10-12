"""Refactor guard: the six gate areas, converted in both output profiles, must stay byte-identical.

    python same_output.py baseline   # convert with the current build into <work>/baseline
    python same_output.py check      # convert again into <work>/current and compare

The header date is the only line allowed to differ.
"""
import concurrent.futures
import filecmp
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
from gate import AREAS, DATA, EXE  # noqa: E402

WORK = os.path.join(DATA, "build", "tmp", "inspect", "same_output")
PROFILES = ["carla", "standard"]
HEADER_DATE = re.compile(rb' date="[^"]*"')


def conversion_args(area):
    """The gate's arguments without the viewer outputs, which both profiles would share."""
    args, skip = [], False
    for arg in AREAS[area]["args"]:
        if skip:
            skip = False
        elif arg in ("--viz", "--basemap"):
            skip = True
        else:
            args.append(arg)
    return args


def convert(area, profile, out_dir):
    out = os.path.join(out_dir, f"{area}.{profile}.xodr")
    subprocess.run([EXE, AREAS[area]["osm"], out, "--target", profile] + conversion_args(area), cwd=DATA,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(out, "rb") as f:
        text = HEADER_DATE.sub(b' date=""', f.read(), count=1)
    with open(out, "wb") as f:
        f.write(text)
    return out


def convert_all(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    jobs = [(area, profile) for area in AREAS for profile in PROFILES]
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(lambda job: convert(*job, out_dir), jobs))
    return [f"{area}.{profile}.xodr" for area, profile in jobs]


def main(mode):
    if mode == "baseline":
        convert_all(os.path.join(WORK, "baseline"))
        print("baseline written")
        return 0
    names = convert_all(os.path.join(WORK, "current"))
    changed = [n for n in names if not filecmp.cmp(os.path.join(WORK, "baseline", n),
                                                   os.path.join(WORK, "current", n), shallow=False)]
    for n in changed:
        print("CHANGED", n)
    print(f"{len(names) - len(changed)}/{len(names)} identical")
    return 1 if changed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "check"))
