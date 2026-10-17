"""OSM fidelity contract: absolute limits on how much of the reference areas' mapped OSM geometry the outputs draw
(docs/OSM_CONTRACT.md).

    python osm_contract.py [--areas munich,london] [--dir build/testareas] [--json out.json]

The converter's main mode builds the road network from every piece of OSM geometry, each as strictly as the reference
line, within the OpenDRIVE schema (user decision 2026-10-18). This contract holds the proven results of that work to
absolute limits, like terrain_contract.py does for heights: they are the measured numbers of the change that proved
them, tightened with new evidence and never loosened to let a change pass. Checks per area, on the standard and the
carla output:
  parking      street-side parking areas (amenity=parking + parking=street_side | lane | layby, wholly inside the
               area's clip box) drawn as parking lanes (OpenDRIVE 1.8.0 §11.7.1 lane type parking; 1.4H laneType
               parking): the number of areas with at least 80 % of their area under parking lanes, and the mean share
               (parking_coverage.py)
Exit 0 when every limit holds, 1 otherwise.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import parking_coverage  # noqa: E402
import terrain_contract  # noqa: E402

TOUT = terrain_contract.TOUT
# parking: (areas >= 80 % covered, mean parking share) - the street-side parking change (parking.cpp), measured
# 2026-10-18 on both profiles
CONTRACT = {
    "munich": dict(parking=(20, 0.70)),     # proven 20 of 30, mean 0.707 (both profiles)
    "london": dict(parking=(12, 0.82)),     # 12 of 17, 0.828
    "frankfurt_kreuz": dict(parking=(11, 0.89)),  # 11 of 12, 0.895
    "stelvio": dict(parking=(1, 0.54)),     # 1 of 8, 0.541
}


def clip_of(area):
    """the area's --clip argument in the gate (the converter cuts ways at that box)"""
    import gate  # here: gate.py imports this module
    args = gate.AREAS[area]["args"]
    return args[args.index("--clip") + 1] if "--clip" in args else None


def check(area, d=TOUT, profiles=("standard", "carla")):
    """(problems, metrics) of one area against its contract"""
    lim, f, problems, metrics = CONTRACT[area], terrain_contract.files(area, d), [], {}
    for prof in profiles:
        if not os.path.exists(f[prof]):
            problems.append(f"{area} {prof}: output missing ({f[prof]})")
            continue
        summary, _ = parking_coverage.measure(f["osm"], f[prof], clip_of(area))
        metrics[prof] = {"parking": summary}
        if "parking" in lim:
            need, share = lim["parking"]
            if summary["covered"] < need or (summary["mean_parking_share"] or 0.0) < share:
                problems.append(f"{area} {prof} parking: {summary['covered']} of {summary['polygons']} street-side areas "
                                f"drawn (>= {need}), mean share {summary['mean_parking_share']} (>= {share})")
    return problems, metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--areas", default=",".join(CONTRACT))
    ap.add_argument("--dir", default=TOUT)
    ap.add_argument("--json")
    a = ap.parse_args()
    out, bad = {}, False
    for area in a.areas.split(","):
        problems, metrics = check(area, a.dir)
        out[area] = {"problems": problems, "metrics": metrics}
        for p in problems:
            print("FAIL", p)
        bad |= bool(problems)
        print(area, json.dumps({k: v["parking"] for k, v in metrics.items()}))
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(out, fh, indent=1)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
