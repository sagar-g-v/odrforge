"""Terrain contract: absolute limits on the proven terrain results of the reference areas (docs/TERRAIN_CONTRACT.md).

    python terrain_contract.py [--areas munich,london,stelvio] [--dir build/testareas] [--json out.json]

Unlike gate.py's baseline (counts relative to the last stored run), these limits do not move when a baseline is
refreshed: a converter change that lifts roads off the ground, lets a deck road float, banks a road past the cap or
leaves a stale output fails here. The limits are the proven numbers plus a small margin; they are tightened with new
evidence, never loosened to let a change pass (docs/TERRAIN_CONTRACT.md). Checks per area:
  fresh        every output (carla <area>.xodr, <area>_standard.xodr, <area>_ground_conformed.odrz) is newer than its
               inputs (OSM, ground and deck grids, osm2xodr.exe) - London road 389 floated 39 m in a carla file written
               before its deck grid existed
  conformed    paved edges vs the conformed ground (OpenDRIVE 1.8.0 §10.5: z = elevation + t tan(superelevation))
  centre       carriageway reference line vs the raw ground, plain roads outside bridges / tunnels
  decks        bridge roads vs the measured deck where the deck grid has a value under them and no other road passes
               over them (a surface model sees the top deck only: under a flyover the lower bridge would be measured
               against the upper deck): a quantile limit, and / or the share of samples within 0.5 m
  superelev    plain roads more than 30 m from their ends: |superelevation| <= 0.085 rad (elevation.cpp caps the terrain
               cross slope at 8 %, atan = 0.0798, plus the cubic records' overshoot); everywhere <= atan(15 %) + 2 mrad
               (junction planes are capped at a 15 % gradient, elevation.cpp step 4, and road ends bank into them)
  steep        roads with a grade over 20 % at a 5 m step (exactly the validator's quality.max_grade warning)
  buried       plain roads more than 2 m under the raw ground over 10 m outside bridges / tunnels and not under a
               bridge (Frankfurt: the Hugo-Eckener-Ring under the terminal forecourt, covered=yes, ran 7 m under it as a
               plain road and the conformed ground trenched it; elevation.cpp coveredUnderpasses)
  objects      trees and street lamps off bridge stretches stand within 0.3 m of the conformed ground under them
               (base = elevation(s) + zOffset, XSD 1.8.1 @zOffset; osm2xodr seats them on that ground)
  pits         no junction road outside tunnels / bridges where the conformed ground lies more than 2 m under the raw
               ground (Frankfurt's underground car park: connecting roads without tunnel records dug 723 pit points into
               the published ground; elevation.cpp junctionsUnderCover)
  named       named roads with their own limit (London: Adam Street on the Adelphi terrace)
Exit 0 when every limit holds, 1 otherwise.
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = r"C:\mywork\osm2carla"
TOUT = os.path.join(DATA, "build", "testareas")
EXE = os.path.join(REPO, "build", "Release", "osm2xodr.exe")
SE_PLAIN = 0.085            # plain-road interior: atan(8 %) = 0.0798 + cubic overshoot (measured max 0.0822)
SE_ALL = math.atan(0.15) + 2e-3  # junction-plane cap (measured max 0.1489 = atan(15 %))
OBJECT_TOL = 0.3  # m: a tree / street lamp base vs the conformed ground under it (off bridge stretches)
SE_END = 30.0               # m: a plain road's end blend into its junction plane (opt.junctionBlend) lies within
# buried: a plain road more than BURIED_DEPTH under the raw ground outside bridges / tunnels, and not under a bridge,
# over BURIED_RUN or more (elevation.cpp coveredUnderpasses writes covered stretches that deep as underpasses;
# the conformed ground trenches whatever stays a plain road)
BURIED_DEPTH = 2.0  # m
# m: catches all five Frankfurt covered stretches before their underpasses were written (20 m caught two); samples
# under a bridge road neither count nor end a run
BURIED_RUN = 10.0
PIT_DEPTH = 2.0  # m: conformed ground this far under the raw ground on a junction road outside tunnels / bridges

# limits: proven value (2026-10-17, docs/TERRAIN_CONTRACT.md) -> limit. q = (quantile, |metric| limit in m)
CONTRACT = {
    "munich": dict(conformed=(99, 0.02), centre=[(50, 0.05), (95, 0.40)], steep=2),
    "london": dict(conformed=(99, 0.02), centre=[(50, 0.05), (95, 0.40)], decks=(95, 0.30), decks_near=0.98, steep=4,
                   named={"Adam Street": 0.10, "Lancaster Place": 0.10}),
    # Frankfurter Kreuz: decks as a share - a surface model sees the top structure only (the ICE viaduct over road
    # bridges) and narrow service bridges drawn a few metres off their deck sample the ground beside it
    "frankfurt_kreuz": dict(conformed=(99, 0.06), centre=[(50, 0.05), (95, 0.40)], decks_near=0.85, steep=7),
    "stelvio": dict(conformed=(95, 0.20), centre=[(50, 0.05), (95, 0.25)], steep=1),
}


def read_grid(path):
    """.odrz -> (originX, originY, cell, z[levels, ny, nx]) (tools/make_terrain.py write_odrz)"""
    with open(path, "rb") as f:
        magic = f.read(8)
        ox, oy, cell, nx, ny = struct.unpack("<dddii", f.read(32))
        levels = struct.unpack("<i", f.read(4))[0] if magic == b"ODRZGRD2" else 1
        f.read(32)
        z = np.frombuffer(f.read(nx * ny * 4 * levels), dtype="<f4").reshape(levels, ny, nx).astype(float)
    return ox, oy, cell, z


def sampler(grid, level=0):
    """bilinear height of one level at (x, y); NaN outside or where a corner has no data"""
    ox, oy, cell, z = grid
    Z = z[level]
    ny, nx = Z.shape

    def f(x, y):
        fx, fy = (x - ox) / cell, (y - oy) / cell
        i, j = int(math.floor(fx)), int(math.floor(fy))
        if i < 0 or j < 0 or i + 1 >= nx or j + 1 >= ny:
            return math.nan
        u, v = fx - i, fy - j
        return float(Z[j, i] * (1 - u) * (1 - v) + Z[j, i + 1] * u * (1 - v) + Z[j + 1, i] * (1 - u) * v
                     + Z[j + 1, i + 1] * u * v)
    return f


def piecewise(road, tag, parent):
    el = road.el.find(parent)
    recs = sorted([(float(e.get("s")), G._abcd(e)) for e in el.findall(tag)]) if el is not None else []
    return (lambda s: road._piecewise(recs, np.asarray(s, dtype=float))) if recs else (lambda s: np.zeros(np.shape(s)))


def structures(road):
    o = road.el.find("objects")
    out = []
    if o is not None:
        for c in o:
            if c.tag in ("bridge", "tunnel"):
                out.append((c.tag, float(c.get("s")), float(c.get("s")) + float(c.get("length"))))
    return out


def on_structure(sts, s, kind=None):
    return any(a - 1e-6 <= s <= b + 1e-6 and (kind is None or k == kind) for k, a, b in sts)


def quantiles(v, qs):
    a = np.abs(np.asarray(v, dtype=float))
    return {q: float(np.percentile(a, q)) if len(a) else math.nan for q in qs}


def measure(xodr, ground=None, conformed=None, decks=None, named=()):
    """metrics of one OpenDRIVE file against its grids (every 2 m along every road)"""
    od = G.OpenDrive(xodr)
    roads = od.roads.values() if isinstance(od.roads, dict) else od.roads
    g = sampler(read_grid(ground)) if ground else None
    c = sampler(read_grid(conformed)) if conformed else None
    dk = read_grid(decks) if decks else None
    dks = [sampler(dk, l) for l in range(dk[3].shape[0])] if dk else []
    centre, edges, deck = [], [], []
    tops = {}  # 5 m cells -> centre-line samples (x, y, z) of every road, for "another road passes over"
    for r in roads:
        sts = structures(r)
        for s in np.arange(0.0, r.length, 2.0):
            x, y = r.point(np.array([s]), 0.0)
            x, y = float(np.ravel(x)[0]), float(np.ravel(y)[0])
            tops.setdefault((int(x // 5), int(y // 5)), []).append(
                (x, y, float(r.z(np.array([s]))[0]), r.id, on_structure(sts, s, "bridge")))

    def covered(x, y, z, rid, bridges_only=False):
        """another road (on a bridge stretch: bridges_only) passes more than 2 m above (x, y, z) within 4 m"""
        cx, cy = int(x // 5), int(y // 5)
        return any(q[3] != rid and q[2] > z + 2.0 and (q[4] or not bridges_only) and math.hypot(q[0] - x, q[1] - y) < 4.0
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1) for q in tops.get((cx + dx, cy + dy), ()))
    se_plain = se_all = 0.0
    steep, named_grade, buried = [], {n: 0.0 for n in named}, []
    for r in roads:
        if r.length < 0.5:
            continue
        sts = structures(r)
        se = piecewise(r, "superelevation", "lateralProfile")
        ss = np.linspace(0.0, r.length, max(3, int(r.length / 0.5)))
        zz = r.z(ss)
        grade = np.abs(np.gradient(zz, ss))
        roll = np.abs(se(ss))
        se_all = max(se_all, float(roll.max()))
        inner = (ss > SE_END) & (ss < r.length - SE_END)
        if r.el.get("junction") == "-1" and inner.any():
            se_plain = max(se_plain, float(roll[inner].max()))
        s5 = np.arange(0.0, r.length + 1e-9, 5.0)  # validate.cpp quality.max_grade: every 5 m, slope > 20 %
        h = 1e-3
        slope = (r.z(np.clip(s5 + h, 0, r.length)) - r.z(np.clip(s5 - h, 0, r.length))) /             np.maximum(np.clip(s5 + h, 0, r.length) - np.clip(s5 - h, 0, r.length), 1e-12)
        if np.any(np.abs(slope) > 0.20):
            steep.append(r.id)
        name = r.el.get("name")
        if name in named_grade and r.el.get("junction") == "-1":
            named_grade[name] = max(named_grade[name], float(grade.max()))
        if r.el.get("junction") != "-1":
            continue
        run = 0.0  # buried: consecutive 2 m samples outside structures more than BURIED_DEPTH under the raw ground
        for s in np.arange(1.0, r.length - 1.0, 2.0):
            z0 = float(r.z(np.array([s]))[0])
            x, y = r.point(np.array([s]), 0.0)
            x, y = float(np.ravel(x)[0]), float(np.ravel(y)[0])
            if on_structure(sts, s, "bridge"):
                lv = [f(x, y) for f in dks]
                lv = [v for v in lv if not math.isnan(v)]
                if lv and not covered(x, y, z0, r.id):
                    deck.append(min((z0 - v for v in lv), key=abs))
                continue
            if on_structure(sts, s):
                run = 0.0
                continue
            if g:
                v = g(x, y)
                if not math.isnan(v):
                    centre.append(z0 - v)
                # under a bridge is not evidence either way: the raw grid there may hold the deck (Frankfurt roads 57,
                # 276 under bridges 34, 58), so such a sample neither counts nor ends a run - a buried stretch that
                # passes partly under a bridge is still caught. A plain road above does not excuse it (Hugo-Eckener-Ring
                # under the forecourt roads)
                deep = not math.isnan(v) and z0 - v < -BURIED_DEPTH
                if not (deep and covered(x, y, z0, r.id, bridges_only=True)):
                    run = run + 2.0 if deep else 0.0
                if run >= BURIED_RUN and r.id not in buried:
                    buried.append(r.id)
            if c:
                b = r.borders_at(s)
                lanes = [v for k, v in b.items() if k]
                if not lanes:
                    continue
                lo, hi = min(min(v) for v in lanes), max(max(v) for v in lanes)
                roll = float(se(np.array([s]))[0])
                for t in (lo + 0.3, hi - 0.3):
                    xe, ye = r.point(np.array([s]), t)
                    v = c(float(np.ravel(xe)[0]), float(np.ravel(ye)[0]))
                    if not math.isnan(v):
                        edges.append(z0 + t * math.tan(roll) - v)
    near = float(np.mean(np.abs(np.asarray(deck)) <= 0.5)) if deck else math.nan
    # trees and street lamps (point objects without outline / repeat) off bridge stretches: base = elevation(s) +
    # zOffset (XSD 1.8.1 t_road_objects_object @zOffset) vs the conformed ground under them
    objects = []
    if c:
        for r in roads:
            sts = structures(r)
            for o in r.el.findall("objects/object"):
                if o.get("type") not in ("tree", "pole", "streetLamp") or o.find("outline") is not None or                         o.find("repeat") is not None:
                    continue
                s = float(o.get("s"))
                if on_structure(sts, s, "bridge"):
                    continue
                x, y = r.point(np.array([s]), float(o.get("t")))
                v = c(float(np.ravel(x)[0]), float(np.ravel(y)[0]))
                if not math.isnan(v):
                    objects.append(float(r.z(np.array([s]))[0]) + float(o.get("zOffset", 0.0)) - v)
    # pits: junction-road points (2 m steps on the reference line) outside tunnel / bridge records where the conformed
    # ground lies more than PIT_DEPTH under the raw ground - a junction under cover without a tunnel record
    pits = []
    if c and g:
        for r in roads:
            if r.el.get("junction") == "-1" or r.length < 0.5:
                continue
            sts = structures(r)
            for s in np.arange(0.25, r.length, 2.0):
                if on_structure(sts, s):
                    continue
                x, y = r.point(np.array([s]), 0.0)
                x, y = float(np.ravel(x)[0]), float(np.ravel(y)[0])
                vg, vc = g(x, y), c(x, y)
                if not (math.isnan(vg) or math.isnan(vc)) and vc < vg - PIT_DEPTH and r.id not in pits:
                    pits.append(r.id)
    return dict(pits=sorted(pits, key=lambda x: int(x) if x.isdigit() else 0),centre=quantiles(centre, (50, 95, 99)), conformed=quantiles(edges, (50, 95, 99)),
                objects=dict(n=len(objects), off=int(np.sum(np.abs(np.asarray(objects)) > OBJECT_TOL)) if objects else 0,
                             worst=float(np.max(np.abs(objects))) if objects else 0.0),
                decks=quantiles(deck, (50, 95, 99)), decks_near=near, n=dict(centre=len(centre), conformed=len(edges), decks=len(deck)),
                superelev=dict(plain=se_plain, all=se_all), steep=sorted(steep, key=lambda x: int(x) if x.isdigit() else 0),
                buried=sorted(buried, key=lambda x: int(x) if x.isdigit() else 0), named=named_grade)


def files(area, d):
    p = lambda f: os.path.join(d, f)  # noqa: E731
    decks = p(f"{area}_decks.odrz")
    return dict(osm=os.path.join(DATA, "data", "testareas", area + ".osm"), ground=p(f"{area}_ground.odrz"),
                decks=decks if os.path.exists(decks) else None, carla=p(f"{area}.xodr"),
                standard=p(f"{area}_standard.xodr"), conformed=p(f"{area}_ground_conformed.odrz"))


def stale(f, exe=EXE):
    """outputs older than an input they are built from"""
    inputs = [x for x in (f["osm"], f["ground"], f["decks"], exe) if x and os.path.exists(x)]
    newest = max(os.path.getmtime(x) for x in inputs)
    return [os.path.basename(o) for o in (f["carla"], f["standard"], f["conformed"])
            if not os.path.exists(o) or os.path.getmtime(o) < newest]


def check(area, d=TOUT, exe=EXE, profiles=("standard", "carla")):
    """(problems, metrics) of one area against its contract (the outputs of `profiles` measured)"""
    lim, f, problems = CONTRACT[area], files(area, d), []
    old = stale(f, exe)
    if old:
        problems.append(f"{area} fresh: older than their inputs (OSM, grids, osm2xodr): {', '.join(old)}")
    named = tuple(lim.get("named", {}))
    metrics = {}
    for prof in profiles:
        if not os.path.exists(f[prof]):
            continue
        m = measure(f[prof], f["ground"], f["conformed"] if prof == "standard" else None, f["decks"], named)
        metrics[prof] = m
        tag = f"{area} {prof}"
        for q, v in lim["centre"]:
            if not m["centre"][q] <= v:
                problems.append(f"{tag} centre: |reference line - ground| p{q} {m['centre'][q]:.3f} m > {v} m")
        if prof == "standard":
            q, v = lim["conformed"]
            if not m["conformed"][q] <= v:
                problems.append(f"{tag} conformed: |paved edge - conformed ground| p{q} {m['conformed'][q]:.3f} m > {v} m")
            if m["objects"]["off"]:
                problems.append(f"{tag} objects: {m['objects']['off']} of {m['objects']['n']} trees / street lamps more than "
                                f"{OBJECT_TOL} m off the ground (worst {m['objects']['worst']:.2f} m)")
            if m["pits"]:
                problems.append(f"{tag} pits: the conformed ground lies more than {PIT_DEPTH} m under the raw ground on "
                                f"{len(m['pits'])} junction roads outside tunnels / bridges ({', '.join(m['pits'][:8])})")
        if lim.get("decks"):
            q, v = lim["decks"]
            if not m["decks"][q] <= v:
                problems.append(f"{tag} decks: |bridge road - measured deck| p{q} {m['decks'][q]:.3f} m > {v} m")
        if lim.get("decks_near") is not None and not m["decks_near"] >= lim["decks_near"]:
            problems.append(f"{tag} decks: {m['decks_near'] * 100:.1f} % of bridge samples within 0.5 m of the measured deck "
                            f"< {lim['decks_near'] * 100:.0f} %")
        if m["superelev"]["plain"] > SE_PLAIN:
            problems.append(f"{tag} superelev: plain roads {m['superelev']['plain']:.4f} rad > {SE_PLAIN} (8 % cap)")
        if m["superelev"]["all"] > SE_ALL:
            problems.append(f"{tag} superelev: {m['superelev']['all']:.4f} rad > atan(15 %) (junction-plane cap)")
        if len(m["buried"]) > lim.get("buried", 0):
            problems.append(f"{tag} buried: {len(m['buried'])} plain roads more than {BURIED_DEPTH} m under the ground over "
                            f"{BURIED_RUN:.0f} m outside bridges / tunnels and not under a bridge > {lim.get('buried', 0)} "
                            f"({', '.join(m['buried'][:8])})")
        if lim.get("steep") is not None and len(m["steep"]) > lim["steep"]:
            problems.append(f"{tag} steep: {len(m['steep'])} roads over 20 % > {lim['steep']} ({', '.join(m['steep'][:8])})")
        for name, v in lim.get("named", {}).items():
            if not m["named"].get(name, 0.0) <= v:
                problems.append(f"{tag} named: {name} max grade {m['named'][name] * 100:.0f} % > {v * 100:.0f} %")
    return problems, metrics


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--areas", default=",".join(CONTRACT))
    ap.add_argument("--dir", default=TOUT)
    ap.add_argument("--json")
    a = ap.parse_args()
    allp, allm = [], {}
    for area in [x for x in a.areas.split(",") if x]:
        p, m = check(area, a.dir)
        allp += p
        allm[area] = m
        print(f"{area}: {'ok' if not p else f'{len(p)} problem(s)'}")
        for prof, mm in m.items():
            print(f"  {prof}: centre p50/p95 {mm['centre'][50]:.3f}/{mm['centre'][95]:.3f}, conformed p95/p99 "
                  f"{mm['conformed'][95]:.3f}/{mm['conformed'][99]:.3f}, decks p95 {mm['decks'][95]:.3f} (n {mm['n']['decks']}), "
                  f"superelev plain/all {mm['superelev']['plain']:.4f}/{mm['superelev']['all']:.4f}, steep {len(mm['steep'])}, "
                  f"buried {len(mm['buried'])}, named {mm['named']}")
    for p in allp:
        print("FAIL", p)
    if a.json:
        json.dump(dict(problems=allp, metrics=allm), open(a.json, "w"), indent=1, default=str)
    sys.exit(1 if allp else 0)


if __name__ == "__main__":
    main()
