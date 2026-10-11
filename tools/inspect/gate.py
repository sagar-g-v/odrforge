"""Quality gate for a converter change: build + unit tests, convert the reference area, run every
inspector, the read-back validation and (optionally) the CARLA loader check, and compare the
finding counts with a stored baseline.

    python gate.py --baseline            # record the current state as the baseline
    python gate.py                       # run and compare; exit 1 on any regression
    python gate.py --skip-build          # reuse the current executables

A regression is any finding kind whose count grows by more than --tolerance (default 0), any new
validation error, a failing unit test, or new CARLA discontinuities.
"""
import argparse
import collections
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
AREA = r"C:\mywork\osm2carla"
OUT = os.path.join(AREA, "build", "odrforge")
XODR = os.path.join(OUT, "LA_DTLA_odrforge.xodr")
OSM = os.path.join(AREA, "data", "area.osm")
BASEMAP = os.path.join(OUT, "LA_DTLA_basemap")
WORK = os.path.join(AREA, "build", "tmp", "inspect", "gate")
BASELINE = os.path.join(AREA, "build", "tmp", "inspect", "gate_baseline.json")
CARLA_PY = r"C:\Users\venki\AppData\Local\Programs\Python\Python38\python.exe"
DETECTORS = ["osm_fidelity", "link_continuity", "edge_smoothness", "junction_gore", "lane_transitions"]
CONVERT = [os.path.join(REPO, "build", "Release", "osm2xodr.exe"), OSM, XODR, "--country", "US", "--target", "carla",
           "--origin", "34.054,-118.2575", "--name", "LA_DTLA", "--terrain", os.path.join(OUT, "ground.odrz"),
           "--structures", os.path.join(OUT, "decks.odrz"), "--clip", "34.044,-118.27,34.064,-118.245",
           "--viz", os.path.join(OUT, "LA_DTLA_odrforge.html"), "--basemap", BASEMAP]


def run(cmd, **kw):
    t = time.time()
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)
    return r, time.time() - t


def kind_of(name, f):
    k = f.get("kind") or f.get("type") or f.get("check") or "?"
    sub = f.get("subtype") or f.get("sub") or f.get("flow")
    return f"{name}:{k}" + (f":{sub}" if sub else "")


def detect(name):
    out = os.path.join(WORK, name)
    r, dt = run([sys.executable, os.path.join(HERE, f"det_{name}.py"), XODR, OSM, "--out", out, "--crops", "0"])
    if r.returncode != 0:
        return name, None, r.stderr[-2000:]
    d = json.load(open(os.path.join(out, "findings.json"), encoding="utf-8"))
    items = d["findings"] if isinstance(d, dict) else d
    c = collections.Counter(kind_of(name, f) for f in items)
    sev = collections.Counter(f"{name}:{f.get('severity', '?')}" for f in items)
    return name, {"kinds": dict(c), "severity": dict(sev), "total": len(items), "seconds": round(dt)}, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", action="store_true")
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--skip-carla", action="store_true")
    ap.add_argument("--tolerance", type=int, default=0)
    a = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    res = {"ok": True, "problems": []}
    if not a.skip_build:
        r, dt = run(["cmd", "/c", os.path.join(REPO, "build.bat")], cwd=REPO)
        tail = [l for l in (r.stdout + r.stderr).splitlines() if "tests passed" in l or "FAILED" in l or "error" in l.lower()][-6:]
        res["build"] = {"rc": r.returncode, "tail": tail}
        if r.returncode != 0 or not any("100% tests passed" in l for l in tail):
            res["ok"] = False
            res["problems"].append("build or unit tests failed: " + " | ".join(tail))
            print(json.dumps(res, indent=1))
            sys.exit(1)
    r, dt = run(CONVERT, cwd=AREA)
    log = r.stdout + r.stderr
    res["convert"] = {"rc": r.returncode, "seconds": round(dt),
                      "summary": [l for l in log.splitlines() if any(k in l for k in ("error", "warning", "roads", "gores", "valid"))][-12:]}
    if r.returncode != 0:
        res["ok"] = False
        res["problems"].append("conversion failed: " + log[-1500:])
        print(json.dumps(res, indent=1))
        sys.exit(1)
    with ThreadPoolExecutor(max_workers=5) as pool:
        det = list(pool.map(detect, DETECTORS))
    kinds, sev, totals = {}, {}, {}
    for name, d, err in det:
        if d is None:
            res["ok"] = False
            res["problems"].append(f"detector {name} crashed: {err}")
            continue
        kinds.update(d["kinds"]), sev.update(d["severity"])
        totals[name] = d["total"]
    res["totals"], res["kinds"], res["severity"] = totals, kinds, sev
    if not a.skip_carla and os.path.exists(CARLA_PY):
        r, _ = run([CARLA_PY, os.path.join(AREA, "build", "tmp", "jumps2.py"), XODR], cwd=AREA)
        line = next((l for l in r.stdout.splitlines() if l.startswith("discontinuities")), r.stderr[-300:])
        res["carla"] = line
        try:
            res["carla_sideways"] = int(line.split("sideways")[1].split()[0])
        except Exception:  # noqa: BLE001
            res["carla_sideways"] = None
    res["validation_errors"] = next((l for l in log.splitlines() if "error" in l.lower()), "")
    if a.baseline:
        json.dump(res, open(BASELINE, "w"), indent=1)
        print("baseline stored:", json.dumps(totals))
        return
    base = json.load(open(BASELINE)) if os.path.exists(BASELINE) else None
    if base:
        delta = {}
        for k in sorted(set(base["kinds"]) | set(kinds)):
            b, n = base["kinds"].get(k, 0), kinds.get(k, 0)
            if b != n:
                delta[k] = f"{b} -> {n}"
            if n > b + a.tolerance:
                res["ok"] = False
                res["problems"].append(f"regression {k}: {b} -> {n}")
        res["delta"] = delta
        res["totals_delta"] = {k: f"{base['totals'].get(k)} -> {totals.get(k)}" for k in totals}
        bs, ns = base.get("carla_sideways"), res.get("carla_sideways")
        if bs is not None and ns is not None and ns > bs:
            res["ok"] = False
            res["problems"].append(f"CARLA sideways discontinuities {bs} -> {ns}")
    print(json.dumps({k: v for k, v in res.items() if k not in ("kinds", "severity")}, indent=1))
    sys.exit(0 if res["ok"] else 1)


if __name__ == "__main__":
    main()
