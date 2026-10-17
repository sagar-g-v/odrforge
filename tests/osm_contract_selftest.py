"""Planted-defect test of tools/inspect/osm_contract.py on the real reference outputs (build/testareas, or a gate run's
staged copies; areas without outputs are skipped): every check must pass on the outputs as they are and fail on a copy
with its defect planted.
usage: python osm_contract_selftest.py [area ...]   (exit 0 = pass; run by gate.py)"""
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
import osm_contract as C  # noqa: E402
import terrain_contract  # noqa: E402

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def parking_to_border(path):  # the parking lanes drawn as plain road-level border lanes: no street parking at all
    tree = ET.parse(path)
    for lane in tree.getroot().iter("lane"):
        if lane.get("type") == "parking":
            lane.set("type", "border")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main():
    areas = sys.argv[1:] or list(C.CONTRACT)
    ran = 0
    for area in areas:
        f = terrain_contract.files(area, C.TOUT)
        if not os.path.exists(f["standard"]):
            continue
        with tempfile.TemporaryDirectory() as tmp:
            shutil.copy2(f["standard"], tmp)
            clean, _ = C.check(area, tmp, profiles=("standard",))
            check(not clean, f"{area} parking: passes on the output as it is ({clean[:1]})")
            parking_to_border(os.path.join(tmp, os.path.basename(f["standard"])))
            planted, _ = C.check(area, tmp, profiles=("standard",))
            check(any(" parking:" in p for p in planted), f"{area} parking: fails with the parking lanes removed")
        ran += 1
    print(f"osm contract self-test: {ran} areas, {len(failures)} failures")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
