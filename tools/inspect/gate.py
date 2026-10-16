"""Quality gate for a converter change: build + unit tests, convert every reference area, run every
inspector, the read-back validation, the CARLA loader check and the esmini loader check, and compare
all counts with a stored baseline.

    python gate.py --baseline            # record the current state as the baseline
    python gate.py                       # run and compare; exit 1 on any regression
    python gate.py --skip-build          # reuse the current executables
    python gate.py --areas la,munich     # subset (comparison only covers those areas)

Reference areas (chosen to cover different road cultures, not to tune for one city):
  la              Los Angeles downtown: US freeways, ramps, grid, terrain + structures
  munich          Munich centre: European urban, trams, dense lane tagging
  london          central London: left-hand traffic, irregular junctions
  frankfurt_kreuz Frankfurter Kreuz: complex motorway interchange, many gores
  stelvio         Stelvio pass: mountain hairpins
  tokyo           Shibuya, Tokyo: left-hand traffic, Japanese tagging, elevated Shuto expressway over streets

Gating kinds: the exact checkers (exact:inv:* from invariants.py, exact:lf:* from laneflow.py,
exact:annexF:<rule path> from annex_f.py, the ASAM OpenDRIVE 1.9 Annex F rules) and osm_fidelity:*; the other heuristic detector counts are reported as informative deltas only.
A regression is any gating (area, kind) whose count grows by more than --tolerance, any new
validation error rule count, a failing unit test, a conversion crash, new CARLA sideways
discontinuities or esmini load errors.
Terrain areas are also converted in the standard profile with the ground conformed to the roads (what the review
viewer draws), and all their outputs are held to the absolute limits of terrain_contract.py (docs/TERRAIN_CONTRACT.md):
those never move with the baseline, so a proven terrain result cannot regress through a baseline refresh.
"""
import argparse
import collections
import json
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import annex_f  # noqa: E402
import terrain_contract  # noqa: E402
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = r"C:\mywork\osm2carla"
WORK = os.path.join(DATA, "build", "tmp", "inspect", "gate")
BASELINE = os.path.join(DATA, "build", "tmp", "inspect", "gate_baseline.json")
CARLA_PY = r"C:\Users\venki\AppData\Local\Programs\Python\Python38\python.exe"
ODRPLOT = r"C:\mywork\tools\esmini\esmini\bin\odrplot.exe"
EXE = os.path.join(REPO, "build", "Release", "osm2xodr.exe")
DETECTORS = ["osm_fidelity", "link_continuity", "edge_smoothness", "junction_gore", "lane_transitions"]
LA_OUT = os.path.join(DATA, "build", "odrforge")
TA = os.path.join(DATA, "data", "testareas")
TOUT = os.path.join(DATA, "build", "testareas")
AREAS = {
    "la": dict(osm=os.path.join(DATA, "data", "area.osm"), xodr=os.path.join(LA_OUT, "LA_DTLA_odrforge.xodr"),
               args=["--country", "US", "--origin", "34.054,-118.2575", "--name", "LA_DTLA",
                     "--terrain", os.path.join(LA_OUT, "ground.odrz"), "--structures", os.path.join(LA_OUT, "decks.odrz"),
                     "--clip", "34.044,-118.27,34.064,-118.245", "--viz", os.path.join(LA_OUT, "LA_DTLA_odrforge.html"),
                     "--basemap", os.path.join(LA_OUT, "LA_DTLA_basemap")]),
    # terrain: absolute ground heights from national bare-earth DEMs (build/testareas/<area>_ground.odrz, written by
    # tools/make_terrain.py; DEM, cell and vertical datum in docs/PROGRESS.md), and bridge decks from a surface model
    # where one is open (<area>_decks.odrz). Frankfurter Kreuz stays relative for now: with ground + decks it is clean, but
    # a three-level underground car park (layers -1 .. -3, 50 roads) connects to no measured road and would keep height 0.
    # Tokyo stays relative: its stacked flyovers have no open deck data
    "munich": dict(country="DE", origin="48.1385,11.5725", clip="48.132,11.560,48.145,11.585", terrain=True),
    "london": dict(country="GB", origin="51.511,-0.125", clip="51.505,-0.135,51.517,-0.115", terrain=True),
    "frankfurt_kreuz": dict(country="DE", origin="50.0525,8.600", clip="50.045,8.585,50.060,8.615"),
    "stelvio": dict(country="IT", origin="46.525,10.4525", clip="46.515,10.440,46.535,10.465", terrain=True),
    "tokyo": dict(country="JP", origin="35.658,139.699", clip="35.652,139.690,35.664,139.708"),
}
for _k, _a in AREAS.items():
    if "args" not in _a:
        _a.update(osm=os.path.join(TA, _k + ".osm"), xodr=os.path.join(TOUT, _k + ".xodr"),
                  args=["--country", _a["country"], "--origin", _a["origin"], "--name", _k, "--clip", _a["clip"]]
                  + (["--terrain", os.path.join(TOUT, _k + "_ground.odrz")] if _a.get("terrain") else [])
                  + (["--structures", os.path.join(TOUT, _k + "_decks.odrz")]
                     if _a.get("terrain") and os.path.exists(os.path.join(TOUT, _k + "_decks.odrz")) else [])
                  + ["--viz", os.path.join(TOUT, _k + ".html")]
                  + (["--basemap", os.path.join(TOUT, _k + "_basemap")] if os.path.isdir(os.path.join(TOUT, _k + "_basemap")) else []))


def run(cmd, **kw):
    t = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)
    return r, time.time() - t


def kind_of(name, f):
    k = f.get("kind") or f.get("type") or f.get("check") or "?"
    sub = f.get("subtype") or f.get("sub") or f.get("flow")
    return f"{name}:{k}" + (f":{sub}" if sub else "")


def without_views(args):
    """osm2xodr arguments without the HTML view options (--viz FILE, --basemap DIR)"""
    out, skip = [], False
    for x in args:
        if skip:
            skip = False
        elif x in ("--viz", "--basemap"):
            skip = True
        else:
            out.append(x)
    return out


def convert(area):
    a = AREAS[area]
    os.makedirs(os.path.dirname(a["xodr"]), exist_ok=True)
    r, dt = run([EXE, a["osm"], a["xodr"], "--target", "carla"] + a["args"], cwd=DATA)
    log = r.stdout + r.stderr
    out = {"seconds": round(dt), "crashed": not os.path.exists(a["xodr"]) or "validation:" not in log}
    m = re.search(r"validation: (\d+) errors, (\d+) warnings", log)
    out["validation_errors"] = int(m.group(1)) if m else None
    out["validation_warnings"] = int(m.group(2)) if m else None
    rules = {}
    if m:  # per-rule counts printed after the validation line: "   189  quality.road_overlap"
        for l in log[m.end():].splitlines()[1:]:
            mm = re.match(r"\s+(\d+)\s+([\w.]+)\s*$", l)
            if not mm:
                break
            rules[mm.group(2)] = int(mm.group(1))
    out["rules"] = rules
    if a.get("terrain") and not out["crashed"]:
        # the standard profile and the conformed ground the viewer draws, from the same inputs (terrain_contract.py:
        # outputs never older than their inputs)
        f = terrain_contract.files(area, TOUT)
        rs, _ = run([EXE, a["osm"], f["standard"], "--target", "standard"] + without_views(a["args"])
                    + ["--ground-out", f["conformed"]], cwd=DATA)
        if not os.path.exists(f["standard"]) or "validation:" not in rs.stdout + rs.stderr:
            out["crashed"] = True
            log = rs.stdout + rs.stderr
    if out["crashed"]:
        out["log_tail"] = log[-1500:]
    return area, out


def detect(job):
    area, name = job
    out = os.path.join(WORK, area, name)
    r, dt = run([sys.executable, os.path.join(HERE, f"det_{name}.py"), AREAS[area]["xodr"], AREAS[area]["osm"],
                 "--out", out, "--crops", "0"])
    if r.returncode != 0:
        return area, name, None, r.stderr[-1500:]
    d = json.load(open(os.path.join(out, "findings.json"), encoding="utf-8"))
    items = d["findings"] if isinstance(d, dict) else d
    return area, name, dict(collections.Counter(kind_of(name, f) for f in items)), None


def exact(area):
    """exact pass/fail checkers (docs/SPEC_GEOMETRY.md): invariants.py summary counts and laneflow.py
    defect counts (severity other than ok / info)"""
    kinds = {}
    a = AREAS[area]
    for tool in ("invariants", "laneflow"):
        out = os.path.join(WORK, area, "exact_" + tool)
        r, _ = run([sys.executable, os.path.join(HERE, tool + ".py"), "--area", area, "--out", out]
                   + (["--crops", "0"] if tool == "laneflow" else []))
        f = os.path.join(out, "invariants.json" if tool == "invariants" else "findings.json")
        if r.returncode not in (0, 1) or not os.path.exists(f):
            kinds[f"exact:{tool}:CRASHED"] = 1
            continue
        d = json.load(open(f, encoding="utf-8"))
        if tool == "invariants":
            for k, v in d["summary"].items():
                if isinstance(v, int):
                    kinds[f"exact:inv:{k}"] = v
        else:
            for k, v in d.items():
                if isinstance(v, dict) and v.get("severity") not in ("ok", "info") and "count" in v:
                    kinds[f"exact:lf:{k}"] = v["count"]
    for rule_path, violations in annex_f.check_file(a["xodr"]).items():
        if violations:
            kinds[f"exact:annexF:{rule_path}"] = len(violations)
    return area, kinds


def loaders(area):
    xodr = AREAS[area]["xodr"]
    out = {}
    if os.path.exists(ODRPLOT):
        r, _ = run([ODRPLOT, xodr, os.path.join(WORK, area, "esmini_track.csv")], cwd=os.path.join(WORK, area))
        bad = [l for l in (r.stdout + r.stderr).splitlines()
               if re.search(r"error|fail|exception", l, re.I) and "traffic_signals" not in l]
        out["esmini_rc"], out["esmini_errors"] = r.returncode, len(bad)
        if bad:
            out["esmini_sample"] = bad[:3]
    if os.path.exists(CARLA_PY):
        r, _ = run([CARLA_PY, os.path.join(DATA, "build", "tmp", "jumps2.py"), xodr], cwd=DATA)
        line = next((l for l in r.stdout.splitlines() if l.startswith("discontinuities")), None)
        out["carla"] = line or ("load failed: " + (r.stderr or r.stdout)[-300:])
        out["carla_sideways"] = int(line.split("sideways")[1].split()[0]) if line else 10 ** 6
    return area, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", action="store_true")
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--skip-carla", action="store_true")
    ap.add_argument("--areas", default=",".join(AREAS))
    ap.add_argument("--tolerance", type=int, default=0)
    a = ap.parse_args()
    areas = [x for x in a.areas.split(",") if x]
    res = {"ok": True, "problems": []}
    if not a.skip_build:
        r, _ = run(["cmd", "/c", os.path.join(REPO, "build.bat")], cwd=REPO)
        tail = [l for l in (r.stdout + r.stderr).splitlines() if "tests passed" in l or "FAILED" in l or " error" in l.lower()][-6:]
        res["build"] = tail
        if r.returncode != 0 or not any("100% tests passed" in l for l in tail):
            res["ok"] = False
            res["problems"].append("build or unit tests failed: " + " | ".join(tail))
            print(json.dumps(res, indent=1))
            sys.exit(1)
    for ar in areas:
        os.makedirs(os.path.join(WORK, ar), exist_ok=True)
    with ThreadPoolExecutor(max_workers=3) as pool:
        conv = dict(pool.map(convert, areas))
    for ar, c in conv.items():
        if c["crashed"]:
            res["ok"] = False
            res["problems"].append(f"{ar}: conversion crashed: {c.get('log_tail', '')[-600:]}")
    good = [ar for ar in areas if not conv[ar]["crashed"]]
    with ThreadPoolExecutor(max_workers=6) as pool:
        dets = list(pool.map(detect, [(ar, n) for ar in good for n in DETECTORS]))
        lds = dict(pool.map(loaders, good)) if not a.skip_carla else {}
        exs = dict(pool.map(exact, good))
    state = {}
    for ar in areas:
        state[ar] = {"validation_errors": conv[ar]["validation_errors"], "rules": conv[ar]["rules"],
                     "kinds": {}, "totals": {}, **lds.get(ar, {})}
    for ar, kinds in exs.items():
        state[ar]["kinds"].update(kinds)
        state[ar]["totals"]["exact"] = sum(kinds.values())
    for ar, name, kinds, err in dets:
        if kinds is None:
            res["ok"] = False
            res["problems"].append(f"{ar}: detector {name} crashed: {err}")
            continue
        state[ar]["kinds"].update(kinds)
        state[ar]["totals"][name] = sum(kinds.values())
    res["areas"] = {ar: {"validation_errors": s["validation_errors"], "totals": s["totals"],
                         "carla": s.get("carla"), "esmini_errors": s.get("esmini_errors")} for ar, s in state.items()}
    # absolute terrain limits (not relative to the baseline, also with --baseline)
    for ar in good:
        if ar in terrain_contract.CONTRACT:
            problems, metrics = terrain_contract.check(ar, TOUT, EXE)
            res["areas"][ar]["terrain"] = {prof: {k: m[k] for k in ("centre", "conformed", "decks", "superelev")}
                                           | {"steep": len(m["steep"])} for prof, m in metrics.items()}
            for p in problems:
                res["ok"] = False
                res["problems"].append("terrain contract " + p)
    # the contract's own checks still bite: every one must fail on a copy with its defect planted
    if any(ar in terrain_contract.CONTRACT for ar in good):
        r, _ = run([sys.executable, os.path.join(REPO, "tests", "terrain_contract_selftest.py")])
        if r.returncode != 0:
            res["ok"] = False
            res["problems"].append("terrain contract self-test failed: " + (r.stdout + r.stderr)[-800:])
    if a.baseline:
        base = json.load(open(BASELINE)) if os.path.exists(BASELINE) and set(areas) != set(AREAS) else {}
        base.update(state)
        json.dump(base, open(BASELINE, "w"), indent=1)
        print(json.dumps(res["areas"], indent=1))
        print("baseline stored for", ", ".join(areas), "| problems:", res["problems"] or "none")
        return
    base = json.load(open(BASELINE)) if os.path.exists(BASELINE) else {}
    delta = {}
    for ar in areas:
        b, n = base.get(ar), state[ar]
        if not b:
            res["problems"].append(f"{ar}: no baseline (run --baseline)")
            continue
        for key, bd, nd in (("kind", b.get("kinds", {}), n["kinds"]), ("rule", b.get("rules", {}), n["rules"])):
            for k in sorted(set(bd) | set(nd)):
                bv, nv = bd.get(k, 0), nd.get(k, 0)
                if bv != nv:
                    delta[f"{ar}:{k}"] = f"{bv} -> {nv}"
                informative = key == "kind" and not (k.startswith("exact:") or k.startswith("osm_fidelity:"))
                if nv > bv + a.tolerance and not informative:
                    res["ok"] = False
                    res["problems"].append(f"regression {ar} {key} {k}: {bv} -> {nv}")
        for k in ("carla_sideways", "esmini_errors"):
            bv, nv = b.get(k), n.get(k)
            if bv is not None and nv is not None and nv > bv:
                res["ok"] = False
                res["problems"].append(f"regression {ar} {k}: {bv} -> {nv}")
        res["areas"][ar]["totals_was"] = b.get("totals")
        res["areas"][ar]["validation_errors_was"] = b.get("validation_errors")
    res["delta"] = delta
    print(json.dumps(res, indent=1))
    sys.exit(0 if res["ok"] else 1)


if __name__ == "__main__":
    main()
