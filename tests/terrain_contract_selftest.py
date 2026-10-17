"""Planted-defect test of tools/inspect/terrain_contract.py on the real reference outputs (build/testareas; skipped when
absent): every check must pass on the outputs as they are and fail on a copy with one defect planted.
usage: python terrain_contract_selftest.py   (exit 0 = pass; run by gate.py with the terrain areas)"""
import os
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "inspect"))
import terrain_contract as C  # noqa: E402

failures = []


def check(cond, what):
    if not cond:
        failures.append(what)
        print("FAIL:", what)


def copy_area(area, tmp):
    """the area's outputs and grids copied to tmp, outputs stamped newer than every input"""
    f = C.files(area, C.TOUT)
    for k in ("ground", "decks", "carla", "standard", "conformed"):
        if f[k]:
            shutil.copy2(f[k], tmp)
    now = time.time()
    for k in ("carla", "standard", "conformed"):
        os.utime(os.path.join(tmp, os.path.basename(f[k])), (now, now))
    return C.files(area, tmp)


def edit_xodr(path, fn):
    tree = ET.parse(path)
    fn(tree.getroot())
    tree.write(path, encoding="utf-8", xml_declaration=True)
    os.utime(path, (time.time(), time.time()))


def plain(root, name=None):
    return [r for r in root.findall("road") if r.get("junction") == "-1" and (name is None or r.get("name") == name)
            and float(r.get("length")) > 80.0]


def problems_of(area, d, kind):
    p, _ = C.check(area, d, exe=os.path.join(d, "none"), profiles=() if kind == "fresh" else ("standard",))
    return [x for x in p if f" {kind}:" in x]


def planted(area, kind, plant):
    """True when `kind` fails on a copy with the defect planted (and passes on the clean copy)"""
    with tempfile.TemporaryDirectory() as tmp:
        f = copy_area(area, tmp)
        clean = problems_of(area, tmp, kind)
        check(not clean, f"{area} {kind}: passes on the outputs as they are ({clean[:1]})")
        plant(f)
        return bool(problems_of(area, tmp, kind))


def lift_all(f):  # every road 1 m above where it was: off the ground
    def fn(root):
        for e in root.iter("elevationProfile"):
            for el in e.findall("elevation"):
                el.set("a", str(float(el.get("a")) + 1.0))
    edit_xodr(f["standard"], fn)


def sink_conformed(f):  # the conformed ground 0.5 m under the roads
    grid = C.read_grid(f["conformed"])
    with open(f["conformed"], "r+b") as h:
        magic = h.read(8)
        head = 8 + 32 + (4 if magic == b"ODRZGRD2" else 0) + 32
        h.seek(head)
        h.write((grid[3] - 0.5).astype("<f4").tobytes())


def dig_pit(f):  # the conformed ground 3 m deeper over 6 m around the middle of one open junction road
    od = C.G.OpenDrive(f["standard"])
    roads = od.roads.values() if isinstance(od.roads, dict) else od.roads
    r = next(r for r in roads if r.el.get("junction") != "-1" and r.length > 4.0 and not C.structures(r))
    x, y = r.point(np.array([r.length / 2]), 0.0)
    x, y = float(np.ravel(x)[0]), float(np.ravel(y)[0])
    ox, oy, cell, z = C.read_grid(f["conformed"])
    i, j = int(round((x - ox) / cell)), int(round((y - oy) / cell))
    k = int(round(3.0 / cell))
    z[0, max(0, j - k):j + k + 1, max(0, i - k):i + k + 1] -= 3.0
    with open(f["conformed"], "r+b") as h:
        magic = h.read(8)
        h.seek(8 + 32 + (4 if magic == b"ODRZGRD2" else 0) + 32)
        h.write(z.astype("<f4").tobytes())


def bank(f):  # one plain road banked at 0.2 rad (11 deg) along its middle
    def fn(root):
        r = plain(root)[0]
        lp = r.find("lateralProfile")
        if lp is None:
            lp = ET.SubElement(r, "lateralProfile")
        for e in list(lp.findall("superelevation")):
            lp.remove(e)
        ET.SubElement(lp, "superelevation", s="0", a="0.2", b="0", c="0", d="0")
    edit_xodr(f["standard"], fn)


def steepen(f, name=None):  # a 30 % grade on plain roads (or the named one)
    def fn(root):
        for r in plain(root, name)[: None if name else 3]:
            for e in r.find("elevationProfile").findall("elevation"):
                e.set("b", "0.3"), e.set("c", "0"), e.set("d", "0")
    edit_xodr(f["standard"], fn)


def float_bridge(f):  # bridge roads 3 m above their measured deck
    def fn(root):
        for r in root.findall("road"):
            o = r.find("objects")
            if o is not None and o.find("bridge") is not None:
                for e in r.find("elevationProfile").findall("elevation"):
                    e.set("a", str(float(e.get("a")) + 3.0))
    edit_xodr(f["standard"], fn)


def sink_trees(f):  # every tree 2 m under the ground (zOffset as if the road's height were the ground's)
    def fn(root):
        for o in root.iter("object"):
            if o.get("type") == "tree":
                o.set("zOffset", str(float(o.get("zOffset", 0.0)) - 2.0))
    edit_xodr(f["standard"], fn)


def sink_road(f):  # one plain road 3 m under the ground over its whole length
    def fn(root):
        for e in plain(root)[0].find("elevationProfile").findall("elevation"):
            e.set("a", str(float(e.get("a")) - 3.0))
    edit_xodr(f["standard"], fn)


def cliff(f):  # one short plain road (3-20 m) at an 80 % grade: a stub the 5 m steep check can miss
    def fn(root):
        r = next(r for r in root.findall("road") if r.get("junction") == "-1" and 3.0 < float(r.get("length")) < 20.0)
        for e in r.find("elevationProfile").findall("elevation"):
            e.set("b", "0.8"), e.set("c", "0"), e.set("d", "0")
    edit_xodr(f["standard"], fn)


def drop_underpasses(f):  # covered stretches written as plain roads again (before elevation.cpp coveredUnderpasses)
    def fn(root):
        for o in root.iter("objects"):
            for t in [t for t in o.findall("tunnel") if t.get("type") == "underpass"]:
                o.remove(t)
    edit_xodr(f["standard"], fn)


def stale_ground(f):  # the ground grid rebuilt after the outputs (London road 389)
    later = time.time() + 60
    os.utime(f["ground"], (later, later))


have = {a: all(os.path.exists(C.files(a, C.TOUT)[k]) for k in ("ground", "carla", "standard", "conformed"))
        for a in C.CONTRACT}
if not have.get("stelvio"):
    print("terrain contract self-test: reference outputs missing, skipped")
    sys.exit(0)
check(planted("stelvio", "fresh", stale_ground), "fresh: a ground grid newer than the outputs is caught")
check(planted("stelvio", "conformed", sink_conformed), "conformed: a conformed ground 0.5 m under the roads is caught")
check(planted("stelvio", "centre", lift_all), "centre: roads lifted 1 m off the ground are caught")
check(planted("stelvio", "superelev", bank), "superelev: a road banked at 0.2 rad is caught")
check(planted("stelvio", "steep", steepen), "steep: more roads over 20 % than the contract allows is caught")
check(planted("stelvio", "buried", sink_road), "buried: a road 3 m under the ground is caught")
check(planted("stelvio", "pits", dig_pit), "pits: a 3 m pit in the conformed ground at a junction road is caught")
check(planted("stelvio", "cliffs", cliff), "cliffs: one more short road at 80 % is caught")
if have.get("london"):
    check(planted("london", "objects", sink_trees), "objects: trees 2 m under the ground are caught")
    check(planted("london", "decks", float_bridge), "decks: bridges 3 m above their measured deck are caught")
    check(planted("london", "named", lambda f: steepen(f, "Adam Street")), "named: Adam Street off its terrace is caught")
else:
    print("london outputs missing: decks / named not checked")
if have.get("frankfurt_kreuz"):  # its deck check is a share of samples within 0.5 m (a surface model sees top decks only)
    check(planted("frankfurt_kreuz", "decks", float_bridge), "decks share: Frankfurt bridges 3 m above their deck are caught")
    check(planted("frankfurt_kreuz", "buried", drop_underpasses), "buried: Frankfurt covered stretches without underpass are caught")
print("terrain contract self-test:", "FAILED" if failures else "ok")
sys.exit(1 if failures else 0)
