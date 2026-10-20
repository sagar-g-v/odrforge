"""OpenDRIVE 1.9 Annex D.1.4 vertical gaps: where two drivable lanes connect, the z and roll of their touching
points (the lane's start / end at its inner and outer edge) shall match. Annex D.1.5 vertical kinks ("should"):
their pitch - the surface slope along the lane edge - should match too (--dpitch, reported separately).

    python annex_d_vertical.py FILE.xodr [--dz 0.01] [--droll 0.002] [--dpitch 0.01] [--list N]

Touching points are compared for every lane link between roads (road predecessor / successor, connecting
roads included) and every junction lane link (incoming road lane -> connecting road lane). Surface height z(s, t) = elevation(s) +
t tan(superelevation(s)) (src/odr_query.cpp surfaceZ). Roll is the superelevation, sign-flipped where the two
roads meet head-on. Pairs whose touching points do not match in plan (> 5 cm, a horizontal gap) are left to the
plan-view checks.
"""
import argparse
import math
import sys

import numpy as np

sys.path.insert(0, __import__("os").path.dirname(__file__))
import odrgeom as G  # noqa: E402

DRIVABLE = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "slipLane", "biking", "bidirectional", "bus"}


def superelevation(road):
    lp = road.el.find("lateralProfile")
    recs = sorted([(float(e.get("s")), G._abcd(e)) for e in lp.findall("superelevation")]) if lp is not None else []
    return lambda s: float(road._piecewise(recs, np.array([s]))[0]) if recs else 0.0


def touching(road, s, lane_id, roll):
    """[(x, y, z), (x, y, z)] at the lane's inner and outer edge, and the road heading there."""
    sec = road.section_at(s)
    if sec.lane(lane_id) is None:
        return None, None
    tin, tout = road.borders_at(s, sec)[lane_id]
    z0, phi = float(road.z(np.array([s]))[0]), roll(s)
    pts = []
    for t in (tin, tout):
        x, y = road.point(np.array([s]), t)
        pts.append((float(x[0]), float(y[0]), z0 + t * math.tan(phi)))
    return pts, float(road.ref(s)[2])


def edge_slope(road, s, lane_id, roll, inner):
    """slope dz / d(plan distance) of the lane's inner or outer edge, measured from s 2 cm into the road"""
    h = 0.02 if s < road.length / 2 else -0.02
    zs = []
    for si in (s, s + h):
        sec = road.section_at(si)
        if sec.lane(lane_id) is None:
            return None
        t = road.borders_at(si, sec)[lane_id][0 if inner else 1]
        x, y = road.point(np.array([si]), t)
        zs.append((float(x[0]), float(y[0]), float(road.z(np.array([si]))[0]) + t * math.tan(roll(si))))
    d = math.hypot(zs[1][0] - zs[0][0], zs[1][1] - zs[0][1])
    # an edge that barely moves while s advances is folded (tight curve at large t; the invariants report
    # folds): its pitch is not defined
    return (zs[1][2] - zs[0][2]) / d if d > 0.5 * abs(h) else None


def pairs(od):
    """(road a, s a, lane a, road b, s b, lane b) for every lane link."""
    R = od.roads
    for r in R.values():  # road links, connecting roads to their outgoing roads included
        for link, s_here, attr in ((r.succ, r.length, "succ"), (r.pred, 0.0, "pred")):
            if link is None or link.type != "road" or link.id not in R:
                continue
            o = R[link.id]
            s_there = o.length if link.contact == "end" else 0.0
            sec = r.sections[-1] if attr == "succ" else r.sections[0]
            for l in sec.lanes():
                to = l.succ if attr == "succ" else l.pred
                if to is not None and l.type in DRIVABLE:
                    yield r, s_here, l.id, o, s_there, to
    for jid, conns in od.junctions.items():
        for c in conns:
            inc, con = R.get(c["incoming"]), R.get(c["connecting"])
            if inc is None or con is None:
                continue
            s_con = 0.0 if c["contact"] == "start" else con.length
            # the incoming road touches the junction at the end that links to it
            s_inc = inc.length if inc.succ is not None and inc.succ.type == "junction" and inc.succ.id == jid else 0.0
            for a, b in c["lanes"]:
                yield inc, s_inc, a, con, s_con, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xodr")
    ap.add_argument("--dz", type=float, default=0.01)
    ap.add_argument("--droll", type=float, default=0.002)
    ap.add_argument("--dpitch", type=float, default=0.01)
    ap.add_argument("--list", type=int, default=0)
    a = ap.parse_args()
    od = G.OpenDrive(a.xodr)
    rolls = {rid: superelevation(r) for rid, r in od.roads.items()}
    checked, gaps, rolls_bad, kinks, out, worst_kink = 0, 0, 0, 0, [], []
    seen = set()
    for ra, sa, la, rb, sb, lb in pairs(od):
        key = tuple(sorted([(ra.id, sa, la), (rb.id, sb, lb)]))
        if key in seen:
            continue
        seen.add(key)
        pa, ha = touching(ra, sa, la, rolls[ra.id])
        pb, hb = touching(rb, sb, lb, rolls[rb.id])
        if pa is None or pb is None:
            continue
        # match each touching point of a with the nearest of b in plan
        worst_dz, plan_ok = 0.0, True
        for p in pa:
            q = min(pb, key=lambda q: math.hypot(q[0] - p[0], q[1] - p[1]))
            if math.hypot(q[0] - p[0], q[1] - p[1]) > 0.05:
                plan_ok = False
            worst_dz = max(worst_dz, abs(q[2] - p[2]))
        if not plan_ok:
            continue
        checked += 1
        head_on = abs(((ha - hb + math.pi) % (2 * math.pi)) - math.pi) > math.pi / 2
        droll = abs(rolls[ra.id](sa) - (-1 if head_on else 1) * rolls[rb.id](sb))
        if worst_dz > a.dz:
            gaps += 1
        if droll > a.droll:
            rolls_bad += 1
        if worst_dz > a.dz or droll > a.droll:
            out.append((worst_dz, droll, ra.id, la, rb.id, lb, pa[0][:2]))
        # D.1.5: slopes measured into each road cancel where the edge continues without a kink
        kink = 0.0
        for k, p in enumerate(pa):
            j = min(range(len(pb)), key=lambda j: math.hypot(pb[j][0] - p[0], pb[j][1] - p[1]))
            ga = edge_slope(ra, sa, la, rolls[ra.id], k == 0)
            gb = edge_slope(rb, sb, lb, rolls[rb.id], j == 0)
            if ga is not None and gb is not None:
                kink = max(kink, abs(ga + gb))
        if kink > a.dpitch:
            kinks += 1
            worst_kink.append((kink, ra.id, la, rb.id, lb, pa[0][:2]))
    print(f"{checked} lane links checked: z gap > {a.dz} m at {gaps}, roll mismatch > {a.droll} rad at {rolls_bad}")
    print(f"  D.1.5 pitch break > {a.dpitch} at {kinks}" + (f" (worst {max(worst_kink)[0]:.3f})" if worst_kink else ""))
    for row in sorted(worst_kink, reverse=True)[: a.list]:
        print(f"  pitch {row[0]:.3f} road {row[1]} lane {row[2]} -> road {row[3]} lane {row[4]} at ({row[5][0]:.0f}, {row[5][1]:.0f})")
    for row in sorted(out, reverse=True)[: a.list]:
        print(f"  dz {row[0]:.3f} m droll {row[1]:.4f} road {row[2]} lane {row[3]} -> road {row[4]} lane {row[5]} at ({row[6][0]:.0f}, {row[6][1]:.0f})")


if __name__ == "__main__":
    main()
