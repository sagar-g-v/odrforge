"""Detailed metrics of OpenDRIVE outputs, recorded on every run of a change, and their difference before / after.

    python metrics.py snapshot DIR [--label L] [--out FILE.json]   # every *.xodr in DIR (and its .invariants / .laneflow)
    python metrics.py diff BEFORE.json AFTER.json [--all]          # what changed, per file and metric
    python metrics.py compare BEFORE_EXE AFTER_EXE OUT_DIR [--areas a,b] [--label-before L] [--label-after L]
        # every reference area x profile converted with both builds from the same inputs (gate.py's arguments), the
        # exact checkers run on both (exact_diff.py), a snapshot of each side and OUT_DIR/report.txt with the difference

A snapshot holds, per file (<area>.<profile>.xodr or <area>[_standard].xodr):
  structure   roads, connecting roads, junctions (by type, with a boundary), lane sections, lanes by type, geometry elements by type,
              network length (plain / connecting), signals, objects, junction-boundary helpers, connecting-road lanes
              opening from width 0 (no predecessor), lanes without any link
  vertical    steepest grade (5 m steps, as validate.cpp quality.max_grade), roads over 20 %, banking maxima (plain-road
              interiors more than 30 m from their ends / everywhere), height range
  exact       the invariant and lane-flow counts of invariants.py / laneflow.py where their output sits next to the file
  file        size in bytes, header revision
and the git commit, the build's timestamp and the time of the snapshot. gate.py stores one per run in
build/tmp/inspect/metrics_history/ and prints the difference to the previous run; the HEAD-vs-current verification
stores one for each side.
"""
import argparse
import datetime
import glob
import hashlib
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
COUNTS = ("lanes_", "geom_", "junctions_", "check:", "rule:", "inv:", "lf:")  # metric names that count occurrences
REPO = os.path.dirname(os.path.dirname(HERE))


def git_commit(repo=REPO):
    try:
        sha = subprocess.run(["git", "-C", repo, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", repo, "status", "--porcelain", "--untracked-files=no"], capture_output=True,
                               text=True).stdout.strip()
        return sha + ("+dirty" if dirty else "")
    except OSError:
        return ""


def piecewise(road, tag, parent):
    el = road.el.find(parent)
    recs = sorted([(float(e.get("s")), G._abcd(e)) for e in el.findall(tag)]) if el is not None else []
    return (lambda s: road._piecewise(recs, np.asarray(s, dtype=float))) if recs else (lambda s: np.zeros(np.shape(s)))


def structure(root):
    m = {}
    roads = root.findall("road")
    plain = [r for r in roads if r.get("junction") == "-1"]
    conn = [r for r in roads if r.get("junction") != "-1"]
    m["roads"], m["connecting_roads"] = len(plain), len(conn)
    m["length_plain_m"] = round(sum(float(r.get("length")) for r in plain), 1)
    m["length_connecting_m"] = round(sum(float(r.get("length")) for r in conn), 1)
    js = root.findall("junction")
    m["junctions"] = len(js)
    for j in js:
        k = "junctions_" + (j.get("type") or "default")
        m[k] = m.get(k, 0) + 1
    m["connections"] = sum(len(j.findall("connection")) for j in js)
    m["junctions_with_boundary"] = sum(1 for j in js if j.find("boundary") is not None)
    m["boundary_helpers"] = sum(1 for r in roads if r.get("name") == "JunctionBoundary")
    sections = lanes = opening = unlinked = 0
    for r in roads:
        for g in r.findall("planView/geometry"):
            for c in g:
                m["geom_" + c.tag] = m.get("geom_" + c.tag, 0) + 1
        for ls in r.findall("lanes/laneSection"):
            sections += 1
            for side in ("left", "right"):
                for ln in ls.findall(f"{side}/lane"):
                    lanes += 1
                    t = ln.get("type")
                    m["lanes_" + t] = m.get("lanes_" + t, 0) + 1
                    link = ln.find("link")
                    pre = link is not None and link.find("predecessor") is not None
                    suc = link is not None and link.find("successor") is not None
                    if not pre and not suc:
                        unlinked += 1
                    w = ln.find("width")
                    if r.get("junction") != "-1" and t == "driving" and not pre and w is not None and \
                            abs(float(w.get("a"))) < 1e-6 and float(ls.get("s")) == 0.0:
                        opening += 1
    m["lane_sections"], m["lanes"] = sections, lanes
    m["elevation_records"] = sum(len(r.findall("elevationProfile/elevation")) for r in roads)
    m["superelevation_records"] = sum(len(r.findall("lateralProfile/superelevation")) for r in roads)
    m["laneOffset_records"] = sum(len(r.findall("lanes/laneOffset")) for r in roads)
    m["connecting_lanes_opening_from_0"], m["lanes_without_link"] = opening, unlinked
    m["signals"] = sum(len(r.findall("signals/signal")) for r in roads)
    m["objects"] = sum(len(r.findall("objects/object")) for r in roads)
    zo = [abs(float(o.get("zOffset", 0.0))) for r in roads for o in r.findall("objects/object")]
    m["objects_zoffset_nonzero"] = sum(1 for v in zo if v > 1e-6)
    m["objects_zoffset_max"] = round(max(zo), 3) if zo else 0.0
    m["bridges"] = sum(len(r.findall("objects/bridge")) for r in roads)
    m["tunnels"] = sum(len(r.findall("objects/tunnel")) for r in roads)
    return m


def vertical(path):
    od = G.OpenDrive(path)
    roads = od.roads.values() if isinstance(od.roads, dict) else od.roads
    steep, gmax, se_plain, se_all, zmin, zmax = 0, 0.0, 0.0, 0.0, math.inf, -math.inf
    for r in roads:
        if r.length < 0.5:
            continue
        s5 = np.arange(0.0, r.length + 1e-9, 5.0)
        h = 1e-3
        a, b = np.clip(s5 - h, 0, r.length), np.clip(s5 + h, 0, r.length)
        slope = np.abs((r.z(b) - r.z(a)) / np.maximum(b - a, 1e-12))
        gmax = max(gmax, float(slope.max()))
        steep += int(np.any(slope > 0.20))
        ss = np.linspace(0.0, r.length, max(3, int(r.length / 1.0)))
        z = r.z(ss)
        zmin, zmax = min(zmin, float(z.min())), max(zmax, float(z.max()))
        roll = np.abs(piecewise(r, "superelevation", "lateralProfile")(ss))
        se_all = max(se_all, float(roll.max()))
        inner = (ss > 30.0) & (ss < r.length - 30.0)
        if r.el.get("junction") == "-1" and inner.any():
            se_plain = max(se_plain, float(roll[inner].max()))
    return {"grade_max": round(gmax, 4), "roads_over_20pct": steep, "superelev_plain_interior_max": round(se_plain, 4),
            "superelev_max": round(se_all, 4), "z_min": round(zmin, 2), "z_max": round(zmax, 2)}


def exact(path):
    m = {}
    inv = os.path.join(path + ".invariants", "invariants.json")
    if os.path.exists(inv):
        d = json.load(open(inv, encoding="utf-8"))
        for k, v in d.get("summary", {}).items():
            if isinstance(v, (int, float)):
                m["inv:" + k] = v
    lf = os.path.join(path + ".laneflow", "findings.json")
    if os.path.exists(lf):
        for k, v in json.load(open(lf, encoding="utf-8")).items():
            if isinstance(v, dict) and "count" in v:
                m["lf:" + k] = v["count"]
    return m


def identity(path):
    """what makes a record belong to exactly one file: its bytes (sha256, size), modification time and folder"""
    data = open(path, "rb").read()
    return {"folder": os.path.dirname(os.path.abspath(path)), "file": os.path.basename(path),
            "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data),
            "mtime": datetime.datetime.fromtimestamp(os.path.getmtime(path)).isoformat(timespec="seconds")}


def snapshot(folder, label="", paths=None):
    """metrics of every *.xodr in folder (or of the given paths); each file's identity in the identity map"""
    files, ident = {}, {}
    paths = sorted(paths if paths is not None else glob.glob(os.path.join(folder, "*.xodr")))
    for path in paths:
        root = ET.parse(path).getroot()
        h = root.find("header")
        m = {"file_bytes": os.path.getsize(path), "revMinor": int(h.get("revMinor")) if h is not None else None}
        m.update(structure(root))
        m.update(vertical(path))
        m.update(exact(path))
        files[os.path.basename(path)] = m
        ident[os.path.basename(path)] = identity(path)
    exe = os.path.join(REPO, "build", "Release", "osm2xodr.exe")
    folders = {i["folder"] for i in ident.values()}
    return {"label": label, "time": datetime.datetime.now().isoformat(timespec="seconds"), "commit": git_commit(),
            "osm2xodr_built": datetime.datetime.fromtimestamp(os.path.getmtime(exe)).isoformat(timespec="seconds")
            if os.path.exists(exe) else None,
            "folder": os.path.abspath(folder) if folder else (folders.pop() if len(folders) == 1 else None),
            "files": files, "identity": ident}


def page_record(snap, name):
    """the provenance record of one output (schema 1, written next to it as <file>.metrics.json): the run, the file's
    identity and its metrics - the viewer shows the numbers only when the sha256 matches the bytes it loaded"""
    return {"schema": 1, "label": snap["label"], "time": snap["time"], "commit": snap["commit"],
            "osm2xodr_built": snap["osm2xodr_built"], **snap["identity"][name], "metrics": snap["files"][name]}


def compare(before_exe, after_exe, out, areas=None, labels=("before", "after")):
    """convert, check and snapshot with both builds; returns the report lines"""
    import concurrent.futures
    from gate import AREAS, DATA
    from same_output import conversion_args
    areas = areas or list(AREAS)
    exes = {"before": os.path.abspath(before_exe), "after": os.path.abspath(after_exe)}

    def run(job):
        tag, area, prof = job
        os.makedirs(os.path.join(out, tag), exist_ok=True)
        dst = os.path.join(out, tag, f"{area}.{prof}.xodr")
        subprocess.run([exes[tag], AREAS[area]["osm"], dst, "--target", prof] + conversion_args(area), cwd=DATA,
                       capture_output=True)
        return dst
    with concurrent.futures.ThreadPoolExecutor(4) as pool:
        list(pool.map(run, [(t, a, p) for t in exes for a in areas for p in ("standard", "carla")]))
    exact_lines = []
    for prof in ("standard", "carla"):
        r = subprocess.run([sys.executable, os.path.join(HERE, "exact_diff.py"), os.path.join(out, "before"),
                            os.path.join(out, "after"), "--profile", prof], capture_output=True, text=True)
        exact_lines += [f"-- exact checkers, {prof}"] + r.stdout.strip().splitlines()[-16:]
    snaps = {}
    for tag, label in zip(("before", "after"), labels):
        snaps[tag] = snapshot(os.path.join(out, tag), f"{label} ({exes[tag]})")
        json.dump(snaps[tag], open(os.path.join(out, f"metrics_{tag}.json"), "w", encoding="utf-8"), indent=1)
    lines, _ = diff(snaps["before"], snaps["after"])
    report = exact_lines + ["-- metrics"] + lines
    open(os.path.join(out, "report.txt"), "w", encoding="utf-8").write("\n".join(report) + "\n")
    return report


def diff(before, after, show_all=False):
    """lines describing every metric that differs (per file), and the number of changed metrics"""
    out, changed = [], 0
    out.append(f"before: {before.get('label') or ''} {before.get('commit')} {before.get('time')}")
    out.append(f"after:  {after.get('label') or ''} {after.get('commit')} {after.get('time')}")
    for f in sorted(set(before["files"]) | set(after["files"])):
        a, b = before["files"].get(f), after["files"].get(f)
        if a is None or b is None:
            out.append(f"== {f}: {'added' if a is None else 'removed'}")
            changed += 1
            continue
        rows = []
        for k in sorted(set(a) | set(b)):
            # a count that does not occur is 0 (no bus lanes, no finding of a kind); any other metric missing on one
            # side was not measured there
            zero = 0 if k.startswith(COUNTS) else "absent"
            va, vb = a.get(k, zero), b.get(k, zero)
            if va == vb and not show_all:
                continue
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)) and va is not None and vb is not None:
                d = vb - va
                rows.append(f"   {k:44s} {va:>14} -> {vb:<14} ({'+' if d > 0 else ''}{round(d, 4)})")
            else:
                rows.append(f"   {k:44s} {va!s:>14} -> {vb!s}")
            changed += va != vb
        out.append(f"== {f}: " + ("unchanged" if not rows else ""))
        out.extend(rows)
    out.append(f"{changed} metric(s) changed")
    return out, changed


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("snapshot")
    s.add_argument("folder")
    s.add_argument("--label", default="")
    s.add_argument("--out")
    d = sub.add_parser("diff")
    d.add_argument("before")
    d.add_argument("after")
    d.add_argument("--all", action="store_true")
    c = sub.add_parser("compare")
    c.add_argument("before_exe")
    c.add_argument("after_exe")
    c.add_argument("out")
    c.add_argument("--areas")
    c.add_argument("--label-before", default="before")
    c.add_argument("--label-after", default="after")
    a = ap.parse_args()
    if a.cmd == "compare":
        print("\n".join(compare(a.before_exe, a.after_exe, a.out, a.areas.split(",") if a.areas else None,
                                (a.label_before, a.label_after))))
        return
    if a.cmd == "snapshot":
        snap = snapshot(a.folder, a.label)
        txt = json.dumps(snap, indent=1)
        if a.out:
            os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
            open(a.out, "w", encoding="utf-8").write(txt)
            print(f"snapshot of {len(snap['files'])} file(s) -> {a.out}")
        else:
            print(txt)
    else:
        lines, _ = diff(json.load(open(a.before, encoding="utf-8")), json.load(open(a.after, encoding="utf-8")), a.all)
        print("\n".join(lines))


if __name__ == "__main__":
    main()
