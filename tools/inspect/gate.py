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
Every run stores a detailed metrics snapshot of its outputs (metrics.py: structure, vertical, exact counts, with
commit and time) in build/tmp/inspect/metrics_history/ and reports the difference to the previous run; next to every
output it writes <file>.metrics.json (schema 1: run, build, the file's sha256 / size / mtime, its metrics).
Terrain areas are also converted in the standard profile with the ground conformed to the roads (what the review
viewer draws), and all their outputs are held to the absolute limits of terrain_contract.py (docs/TERRAIN_CONTRACT.md):
those never move with the baseline, so a proven terrain result cannot regress through a baseline refresh.
Outputs are published atomically: a run writes and measures them in build/tmp/inspect/gate_stage and moves them into
build/testareas / build/odrforge together, followed by their build records, only at its end (a crashed area keeps its
last outputs).
"""
import argparse
import collections
import json
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import annex_f  # noqa: E402
import metrics  # noqa: E402
import osm_contract  # noqa: E402
import terrain_contract  # noqa: E402
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = r"C:\mywork\osm2carla"
WORK = os.path.join(DATA, "build", "tmp", "inspect", "gate")
BASELINE = os.path.join(DATA, "build", "tmp", "inspect", "gate_baseline.json")
METRICS = os.path.join(DATA, "build", "tmp", "inspect", "metrics_history")  # one snapshot per run (metrics.py)
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
    # where one is open (<area>_decks.odrz). Frankfurter Kreuz's three-level underground car park, which OSM connects to
    # no street, keeps its storeys under the lowest measured surface over it (elevation.cpp junctionSurfaces).
    # Tokyo stays relative: its stacked flyovers have no open deck data
    "munich": dict(country="DE", origin="48.1385,11.5725", clip="48.132,11.560,48.145,11.585", terrain=True),
    "london": dict(country="GB", origin="51.511,-0.125", clip="51.505,-0.135,51.517,-0.115", terrain=True),
    "frankfurt_kreuz": dict(country="DE", origin="50.0525,8.600", clip="50.045,8.585,50.060,8.615", terrain=True),
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


# Atomic publishing: a run converts and measures in a staging folder and moves the outputs into place (carla file, HTML
# view, standard file, conformed ground) only at its end, followed by their build records - the review folder never
# holds a half-published build, and a page opened during a run shows the last complete one. The checkers the gate
# starts (invariants.py, laneflow.py and the contract self-test import this module or terrain_contract.py) read
# STAGE_ENV, so they measure the staged files too.
STAGE_ENV = "ODRFORGE_GATE_STAGE"
STAGE = os.path.join(DATA, "build", "tmp", "inspect", "gate_stage")
FINAL = {k: dict(xodr=a["xodr"], viz=next((a["args"][i + 1] for i, x in enumerate(a["args"][:-1]) if x == "--viz"), None))
         for k, a in AREAS.items()}


def staged(path, stage):
    """the staged twin of a published output: <stage>/<its folder name>/<its file name>"""
    return os.path.join(stage, os.path.basename(os.path.dirname(path)), os.path.basename(path))


def use_stage(stage):
    """point every area's outputs (carla file, HTML view) at the staging folder"""
    for k, a in AREAS.items():
        a["xodr"] = staged(FINAL[k]["xodr"], stage)
        if FINAL[k]["viz"]:
            a["args"][a["args"].index("--viz") + 1] = staged(FINAL[k]["viz"], stage)


if os.environ.get(STAGE_ENV):
    use_stage(os.environ[STAGE_ENV])


def publish(areas, stage):
    """move the run's outputs of `areas` into place, file by file (os.replace: atomic on one volume); returns
    {staged folder: published folder} for the build records"""
    folders = {}
    for ar in areas:
        pairs = [(AREAS[ar]["xodr"], FINAL[ar]["xodr"])]
        if FINAL[ar]["viz"]:
            pairs.append((staged(FINAL[ar]["viz"], stage), FINAL[ar]["viz"]))
        if AREAS[ar].get("terrain"):
            src, dst = terrain_contract.files(ar, TOUT), terrain_contract.files(ar, TOUT, out=TOUT)
            pairs += [(src[k], dst[k]) for k in ("standard", "conformed")]
        for src, dst in pairs:
            if os.path.exists(src):
                os.replace(src, dst)
                folders[os.path.dirname(os.path.abspath(src))] = os.path.dirname(os.path.abspath(dst))
    return folders


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
    shutil.rmtree(STAGE, ignore_errors=True)  # a fresh staging folder; published outputs stay untouched until publish()
    os.environ[STAGE_ENV] = STAGE
    use_stage(STAGE)
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
    contract_numbers = {}
    for ar in good:
        if ar in terrain_contract.CONTRACT:
            problems, measured = terrain_contract.check(ar, TOUT, EXE)
            contract_numbers[ar] = measured
            res["areas"][ar]["terrain"] = {prof: {k: m[k] for k in ("centre", "conformed", "decks", "superelev")}
                                           | {"steep": len(m["steep"])} for prof, m in measured.items()}
            for p in problems:
                res["ok"] = False
                res["problems"].append("terrain contract " + p)
    # the contract's own checks still bite: every one must fail on a copy with its defect planted
    if any(ar in terrain_contract.CONTRACT for ar in good):
        r, _ = run([sys.executable, os.path.join(REPO, "tests", "terrain_contract_selftest.py")])
        if r.returncode != 0:
            res["ok"] = False
            res["problems"].append("terrain contract self-test failed: " + (r.stdout + r.stderr)[-800:])
    # absolute limits on the mapped OSM geometry the outputs draw (osm_contract.py), and its planted-defect self-test
    osm_areas = [ar for ar in good if ar in osm_contract.CONTRACT]
    for ar in osm_areas:
        problems, measured = osm_contract.check(ar, TOUT)
        res["areas"][ar]["osm"] = measured
        for p in problems:
            res["ok"] = False
            res["problems"].append("osm contract " + p)
    if osm_areas:
        r, _ = run([sys.executable, os.path.join(REPO, "tests", "osm_contract_selftest.py")] + osm_areas)
        if r.returncode != 0:
            res["ok"] = False
            res["problems"].append("osm contract self-test failed: " + (r.stdout + r.stderr)[-800:])
    # detailed metrics of every output of this run (metrics.py), stored with commit and time, and their difference to
    # the previous run over the same areas: what a change did, measured on every run
    outputs = [AREAS[ar]["xodr"] for ar in good]
    outputs += [terrain_contract.files(ar, TOUT)["standard"] for ar in good if AREAS[ar].get("terrain")
                and ar in terrain_contract.CONTRACT]
    snap = metrics.snapshot(None, "gate " + ",".join(areas), [p for p in outputs if os.path.exists(p)])
    for ar in good:  # the run's own counts: exact checkers / detectors and validation rules of the carla output
        m = snap["files"].get(os.path.basename(AREAS[ar]["xodr"]))
        if m is not None:
            m.update({"check:" + k: v for k, v in state[ar]["kinds"].items()})
            m.update({"rule:" + k: v for k, v in (state[ar]["rules"] or {}).items()})
            m["validation_errors"] = state[ar]["validation_errors"]
        for prof, tm in contract_numbers.get(ar, {}).items():  # terrain fit numbers (terrain_contract.py)
            f = os.path.basename(AREAS[ar]["xodr"] if prof == "carla" else terrain_contract.files(ar, TOUT)["standard"])
            if f in snap["files"]:
                for key in ("centre", "conformed", "decks"):
                    for q, v in tm[key].items():
                        if v == v:  # not NaN
                            snap["files"][f][f"terrain:{key}_p{q}"] = round(v, 4)
                near = tm.get("decks_near")
                if isinstance(near, float) and near == near:
                    snap["files"][f]["terrain:decks_within_0.5m"] = round(near, 4)
    # publish: every measured output of the converted areas moves into place, then the records name the published folder
    folders = publish(good, STAGE)
    for ident in snap["identity"].values():
        ident["folder"] = folders.get(os.path.abspath(ident["folder"]), ident["folder"])
    if snap.get("folder") in folders:
        snap["folder"] = folders[snap["folder"]]
    res["published"] = sorted(set(folders.values()))
    os.makedirs(METRICS, exist_ok=True)
    stamp = snap["time"].replace(":", "")
    previous = metrics.latest(METRICS, "_" + "-".join(sorted(areas)) + ".json")  # before this run's file is written
    path = os.path.join(METRICS, f"{stamp}_{snap['commit'].replace('+', '_')}_{'-'.join(sorted(areas))}.json")
    json.dump(snap, open(path, "w", encoding="utf-8"), indent=1)
    res["metrics"] = path
    # provenance next to every published output (<file>.metrics.json): which run, build and bytes these numbers belong to
    for name, ident in snap["identity"].items():
        page = os.path.join(ident["folder"], name)
        json.dump(metrics.page_record(snap, name), open(page + ".metrics.json", "w", encoding="utf-8"), indent=1)
    if previous:
        lines, n = metrics.diff(json.load(open(previous, encoding="utf-8")), snap)
        res["metrics_vs_previous_run"] = lines
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
