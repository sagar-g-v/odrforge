"""det_osm_fidelity -- does every plain road sit on its OSM way, with the lanes OSM says?

For each plain (non-junction) road the OSM ways listed in its <userData code="osmWays"> are densified
(1 m), projected onto the road's reference line and binned along s (1 m bins). Per bin we evaluate

  fit    t_osm             lateral distance of the OSM line from the reference line (left +). This is
                           the clothoid-fit error; it should be ~0 except where the converter must
                           fillet a corner sharper than its minimum radius (allowance below).
  band   e = t_osm - t_exp where t_exp is where the OSM way should lie across the lanes: the middle of
                           the carriageway lanes (driving / bidirectional / biking, actual widths --
                           exactly the converter's own convention, src/lanes.cpp:262-286, OSM default
                           placement), or the lane border named by placement / placement:forward /
                           placement:backward.
  lanes  driving-lane count per travel direction (full-width lanes only) vs the way's
         lanes / lanes:forward / lanes:backward / lanes:both_ways tags (oneway semantics as in
         src/lanes.cpp:150-160). Untagged ways are compared only where OSM has a documented default
         (*_link and oneway service = 1 lane).
  width  per-lane width of full driving lanes vs width:lanes / width tags (only where tagged).

Legitimate converter behaviour that is NOT flagged:
  * fillets: |fit| up to R*(1/cos(dTheta/2)-1) + 0.35 m, R = the converter's minimum corner radius
    for that road (max(class radius, half width + 3), src/network.cpp:340-346), dTheta the OSM heading
    change over a window, plus the 0.25 m Douglas-Peucker noise tolerance;
  * lane add/drop tapers: bins where any driving lane is between 0.05 m and 90 % of its full width are
    "transition" bins and skipped for count / width and judged with a looser band tolerance;
  * section boundaries moved by up to taperLength (30 m) + 5 m from the OSM way boundary
    (src/network.cpp:507-520): a count mismatch matching the neighbouring way's count is tolerated
    within 35 m of that boundary;
  * gore extensions: stretches of a trunk road not covered by its own OSM ways (it carries branch
    lanes up to the separation, src/gore.cpp) are reported only as coverage, never as lane errors.

Thresholds (a correct converter gives zero findings):
  band   |e| > 0.75 m over >= 10 m of non-transition bins, or |e| > 1.5 m anywhere outside
         transitions (one lane = 3.0-3.7 m, so 0.75 m ~ 1/5 lane: the way is visibly off the centre).
  fit    |t_osm| - allowance > 0.5 m over >= 5 m.
  lanes  count differs over >= 15 m of non-transition bins (outside the 35 m boundary tolerance).
  width  |w - w_tag| > 0.25 m over >= 15 m.
Severity: band critical >= 3.0 m (the way lies in another lane / off the road), major >= 1.5 m, else
minor; lanes critical when a direction is missing or the difference >= 2, major otherwise (minor when
the expectation comes from an OSM default, not a tag); fit major >= 2 m else minor; width minor.

CLI: python det_osm_fidelity.py XODR OSM --out DIR [--basemap DIR] [--crops N]
Writes DIR/findings.json, DIR/summary.txt and (with --basemap) DIR/crops/*.png.
"""
import argparse
import collections
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import odrgeom as G  # noqa: E402

BIN = 1.0
CARRIAGEWAY = {"driving", "bidirectional", "biking"}
BAND_TOL, BAND_HARD, BAND_MINLEN = 0.75, 1.5, 10.0
FIT_TOL, FIT_MINLEN = 0.5, 5.0
COUNT_MINLEN, BOUNDARY_TOL = 15.0, 35.0
WIDTH_TOL, WIDTH_MINLEN = 0.25, 15.0
CLASS_RADIUS = {"motorway": 60.0, "trunk": 60.0, "motorway_link": 25.0, "trunk_link": 25.0,
                "primary": 15.0, "secondary": 15.0}
LINK_DEFAULT_1 = {"motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
SKIP_HW = {"busway", "bus_guideway", "construction", "proposed", "raceway"}


# ----------------------------------------------------------------------------- OSM tag semantics
def _int(v):
    try:
        return int(str(v).split(";")[0].strip())
    except (TypeError, ValueError):
        return None


def _length(v):
    if v is None:
        return None
    v = str(v).strip()
    try:
        if "'" in v:
            ft, rest = v.split("'", 1)
            inch = float(rest.strip('" ')) if rest.strip('" ') else 0.0
            return float(ft) * 0.3048 + inch * 0.0254
        x = float(v.replace("m", "").replace("ft", "").strip())
        if "ft" in v:
            x *= 0.3048
        return x if 0.5 < x < 60 else None
    except ValueError:
        return None


def way_spec(tags):
    """OSM expectation of one way, in way direction:
    {oneway, reverse, n_with, n_against, both, total, tagged, widths_with, widths_against, placement}"""
    hw = tags.get("highway", "")
    ow = tags.get("oneway", "")
    reverse = ow in ("-1", "reverse")
    rb = tags.get("junction") in ("roundabout", "circular")
    oneway = ow in ("yes", "true", "1") or reverse or rb or (hw in ("motorway", "motorway_link") and ow != "no")
    total, f, b = _int(tags.get("lanes")), _int(tags.get("lanes:forward")), _int(tags.get("lanes:backward"))
    both = _int(tags.get("lanes:both_ways")) or 0
    sp = dict(hw=hw, oneway=oneway, reverse=reverse, both=0, total=total, tagged=True, n_with=None,
              n_against=None, default=False)
    if oneway:
        n = total
        if n is None and (hw in LINK_DEFAULT_1 or hw == "service"):
            n, sp["default"] = 1, True
        if n is None:
            sp["tagged"] = False
        sp["n_with"], sp["n_against"] = (0, n) if reverse else (n, 0)
    else:
        sp["both"] = both
        if f is not None and b is not None:
            sp["n_with"], sp["n_against"] = f, b
        elif total is not None:
            rem = total - both
            if f is not None:
                sp["n_with"], sp["n_against"] = f, rem - f
            elif b is not None:
                sp["n_with"], sp["n_against"] = rem - b, b
            # else: only the sum is known (n_with/n_against None, total set)
        else:
            sp["tagged"] = False
    # widths
    fs = "" if oneway else ":forward"

    def per_lane(key):
        v = tags.get(key)
        return [(_length(p) if p.strip() else None) for p in v.split("|")] if v else None

    sp["w_lanes_with"] = per_lane("width:lanes" + fs)
    sp["w_lanes_against"] = per_lane("width:lanes:backward") if not oneway else None
    sp["width"] = _length(tags.get("width"))
    # placement (forward = the way's direction of travel)
    pl = {}
    for k in ("placement", "placement:forward", "placement:backward"):
        v = tags.get(k)
        if v and ":" in v:
            mode, n = v.split(":", 1)
            pl[k] = (mode, _int(n))
        elif v:
            pl[k] = (v, None)
    sp["placement"] = pl
    tl = tags.get("turn:lanes" + fs)
    sp["turn_n"] = len(tl.split("|")) if tl else None
    return sp


# ----------------------------------------------------------------------------- geometry helpers
def densify(pts, step):
    """dense points, unit tangents and cumulative length of a polyline"""
    out, tan, cum = [], [], []
    acc = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        d = b - a
        L = float(np.hypot(*d))
        if L < 1e-6:
            continue
        n = max(1, int(math.ceil(L / step)))
        u = np.arange(n) / n
        out.append(a + u[:, None] * d), tan.append(np.repeat((d / L)[None], n, 0)), cum.append(acc + u * L)
        acc += L
    if not out:
        return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0)
    out.append(pts[-1:]), tan.append(tan[-1][-1:]), cum.append(np.array([acc]))
    return np.concatenate(out), np.concatenate(tan), np.concatenate(cum)


def fillet_allowance(P, T, C, R):
    """per dense OSM point: the deviation a fillet of radius R is forced to make (see module doc)"""
    if len(P) < 3:
        return np.zeros(len(P))
    h = np.unwrap(np.arctan2(T[:, 1], T[:, 0]))
    allow = np.zeros(len(P))
    for w in (4.0, 8.0, 16.0, 32.0, 64.0, 128.0):
        i0 = np.searchsorted(C, C - w / 2)
        i1 = np.clip(np.searchsorted(C, C + w / 2), 0, len(C) - 1)
        th = np.abs(h[i1] - h[i0])
        th = np.minimum(th, math.radians(170))
        lt = R * np.tan(th / 2)  # tangent length the fillet needs
        dev = R * (1 / np.cos(th / 2) - 1)
        forced = np.where(lt > w / 2, dev * (1 - (w / 2) / np.maximum(lt, 1e-9)), 0.0)
        allow = np.maximum(allow, forced)
    return allow


def chain_allowance(osm, ways, R):
    """fillet allowance along the whole chain of ways (corners at way joints count too)"""
    chain = None
    for w in ways:
        nds = [n for n in osm.ways[w][0] if n in osm.nodes]
        if len(nds) < 2:
            continue
        if chain is None:
            chain = list(nds)
            continue
        if nds[0] == chain[-1]:
            chain += nds[1:]
        elif nds[-1] == chain[-1]:
            chain += nds[::-1][1:]
        elif nds[-1] == chain[0]:
            chain = nds[:-1] + chain
        elif nds[0] == chain[0]:
            chain = nds[::-1][:-1] + chain
        else:
            chain += nds  # not contiguous (should not happen): no corner credit at the gap
    if not chain:
        return np.zeros((0, 2)), np.zeros(0)
    P, T, C = densify(np.array([osm.nodes[n] for n in chain]), 1.0)
    return P, fillet_allowance(P, T, C, R)


def lookup_allowance(P, CP, CA):
    if len(CP) == 0:
        return np.zeros(len(P))
    out = np.empty(len(P))
    for k in range(0, len(P), 256):
        p = P[k:k + 256]
        d2 = (CP[None, :, 0] - p[:, :1]) ** 2 + (CP[None, :, 1] - p[:, 1:2]) ** 2
        out[k:k + 256] = CA[np.argmin(d2, axis=1)]
    return out


def project_points(r, P):
    """(s, t, dist) of many points on road r (nearest dense sample + one Newton step)"""
    S, X, Y, H = r.S, r.X, r.Y, r.H
    idx = np.empty(len(P), int)
    for k in range(0, len(P), 256):
        p = P[k:k + 256]
        d2 = (X[None, :] - p[:, :1]) ** 2 + (Y[None, :] - p[:, 1:2]) ** 2
        idx[k:k + 256] = np.argmin(d2, axis=1)
    s = S[idx]
    for _ in range(2):
        x, y, h = r.ref(s)
        s = np.clip(s + (P[:, 0] - x) * np.cos(h) + (P[:, 1] - y) * np.sin(h), 0, r.length)
    x, y, h = r.ref(s)
    t = -(P[:, 0] - x) * np.sin(h) + (P[:, 1] - y) * np.cos(h)
    return s, t, np.hypot(P[:, 0] - x, P[:, 1] - y), h


def runs(mask):
    """[(i0, i1)] inclusive index runs of True"""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


# ----------------------------------------------------------------------------- per road
def road_halfwidth(r):
    hw = 0.0
    for sec in r.sections:
        for sv in (sec.s, 0.5 * (sec.s + sec.e), max(sec.s, sec.e - 1e-3)):
            b = r.borders_at(sv, sec)
            hw = max(hw, max(abs(v) for p in b.values() for v in p))
    return hw


GORE_CONN_MAX = 8.0  # a lane-level gore junction is kJunctionLength = 4 m long (src/gore.cpp:25)


def gore_junctions(od):
    """junction ids that are lane-level gores: every connecting road short, exactly three plain roads"""
    out = set()
    for jid, cs in od.junctions.items():
        if not cs:
            continue
        lens = [od.roads[c["connecting"]].length for c in cs if c["connecting"] in od.roads]
        plain = set()
        for c in cs:
            plain.add(c["incoming"])
            cr = od.roads.get(c["connecting"])
            for lk in (cr.pred, cr.succ) if cr else ():
                if lk and lk.type == "road":
                    plain.add(lk.id)
        if lens and max(lens) <= GORE_CONN_MAX and len(plain) == 3:
            out.add(jid)
    return out


def gore_mask(r, sb, gores, od):
    """bins of the section next to a gore junction on the TRUNK (the road that carries the branch
    lanes up to the separation: 1 road -> 2 roads at a diverge, 2 roads -> 1 at a merge)"""
    m = np.zeros(len(sb), bool)

    def partners(jid):
        p = set()
        for c in od.junctions[jid]:
            cr = od.roads.get(c["connecting"])
            far = None
            if cr is not None:
                for lk in (cr.pred, cr.succ):
                    if lk and lk.type == "road" and lk.id != c["incoming"]:
                        far = lk.id
            if c["incoming"] == r.id and far:
                p.add(far)
            elif far == r.id:
                p.add(c["incoming"])
        return p

    if r.pred and r.pred.type == "junction" and r.pred.id in gores and len(partners(r.pred.id)) == 2:
        e = r.sections[1].s if len(r.sections) > 1 else r.length
        m |= sb < e
    if r.succ and r.succ.type == "junction" and r.succ.id in gores and len(partners(r.succ.id)) == 2:
        b = r.sections[-1].s if len(r.sections) > 1 else 0.0
        m |= sb >= b
    return m


def converter_default(sp):
    """lanes the converter assumes for an untagged way (rules/countries/US.json, src/lanes.cpp:163-183)"""
    hw = sp["hw"]
    base = hw[:-5] if hw.endswith("_link") else hw
    if sp["oneway"]:
        n = 2 if base in ("motorway", "trunk") else 1
        return (0, n) if sp["reverse"] else (n, 0)
    n = 4 if base in ("motorway", "trunk") else 2
    if sp["total"] is not None:
        n = max(2, sp["total"] - sp["both"])
    return ((n + 1) // 2, n - (n + 1) // 2)


def analyze_road(r, osm, gores=frozenset(), od=None):
    L = r.length
    if L < 2.0:
        return None
    ways = [w for w in r.osm_ways if w in osm.ways]
    if not ways:
        return None
    hwy = osm.ways[ways[0]][1].get("highway", "")
    if hwy in SKIP_HW:
        return None
    R = max(CLASS_RADIUS.get(hwy, 8.0), road_halfwidth(r) + 3.0)
    nb = max(1, int(L / BIN))
    sb = (np.arange(nb) + 0.5) * L / nb
    best = np.full(nb, np.inf)
    t_osm = np.full(nb, np.nan)
    allow = np.zeros(nb)
    wayat = np.full(nb, -1)
    dirat = np.zeros(nb)
    specs = [way_spec(osm.ways[w][1]) for w in ways]
    way_end_s = []  # s of the way ends on this road (for the boundary tolerance)
    CP, CA = chain_allowance(osm, ways, R)
    for wi, w in enumerate(ways):
        poly = osm.polyline(w)
        if len(poly) < 2:
            continue
        P, T, C = densify(poly, 1.0)
        A = lookup_allowance(P, CP, CA)
        s, t, d, h = project_points(r, P)
        cosang = T[:, 0] * np.cos(h) + T[:, 1] * np.sin(h)
        ok = (s > 0.3) & (s < L - 0.3) & (d < 25.0) & (np.abs(cosang) > 0.7)
        if ok.sum() >= 2:
            way_end_s += [float(s[ok].min()), float(s[ok].max())]
        k = np.clip((s / L * nb).astype(int), 0, nb - 1)
        for kk, tt, aa, cc in zip(k[ok], t[ok], A[ok], cosang[ok]):
            if abs(tt) < best[kk]:
                best[kk], t_osm[kk], allow[kk], wayat[kk], dirat[kk] = abs(tt), tt, aa, wi, 1 if cc > 0 else -1
    cov = ~np.isnan(t_osm)
    if cov.sum() < 2:
        return None
    # fill single-bin holes (1 m sampling of a polyline vertex can skip a bin)
    for i in range(1, nb - 1):
        if not cov[i] and cov[i - 1] and cov[i + 1] and wayat[i - 1] == wayat[i + 1]:
            t_osm[i] = 0.5 * (t_osm[i - 1] + t_osm[i + 1])
            allow[i], wayat[i], dirat[i], cov[i] = max(allow[i - 1], allow[i + 1]), wayat[i - 1], dirat[i - 1], True
    # lane model per bin
    lo = r.lane_offset(sb)
    c_mid = np.full(nb, np.nan)
    t_exp = np.full(nb, np.nan)
    trans = np.zeros(nb, bool)
    n_r = np.zeros(nb, int)  # full driving lanes on the right (travel +s)
    n_l = np.zeros(nb, int)
    n_bi = np.zeros(nb, int)
    w_r, w_l = [None] * nb, [None] * nb
    secidx = np.zeros(nb, int)
    full_w = {}
    for si, sec in enumerate(r.sections):
        ss = np.linspace(sec.s, sec.e, 9)
        for l in sec.lanes():
            full_w[(si, l.id)] = float(np.max(l.width(ss - sec.s)))
    for i in range(nb):
        s = float(sb[i])
        sec = r.section_at(s)
        si = r.sections.index(sec)
        secidx[i] = si
        b = r.borders_at(s, sec)
        vals = []
        wr, wl = [], []
        for l in sec.lanes():
            ti, to = b[l.id]
            w = abs(to - ti)
            if l.type in CARRIAGEWAY and w > 0.01:
                vals += [ti, to]
            if l.type in ("driving", "bidirectional"):
                fw = full_w[(si, l.id)]
                if 0.05 < w < 0.9 * fw:
                    trans[i] = True
                if w >= 0.9 * fw and w > 1.5:
                    if l.type == "bidirectional":
                        n_bi[i] += 1
                    elif l.id < 0:
                        n_r[i] += 1
                        wr.append((abs(l.id), w))
                    else:
                        n_l[i] += 1
                        wl.append((l.id, w))
        w_r[i], w_l[i] = [w for _, w in sorted(wr)], [w for _, w in sorted(wl)]
        if not vals:
            continue
        c_mid[i] = 0.5 * (min(vals) + max(vals))
        t_exp[i] = c_mid[i]
        if cov[i]:
            sp = specs[wayat[i]]
            pl = sp["placement"]
            if pl:
                te = placement_t(sp, pl, dirat[i], b, sec)
                if te is not None:
                    t_exp[i] = te
    e = t_osm - t_exp
    # a fillet the converter is forced to make moves the lanes with the reference line: not a band error
    e_adj = np.sign(e) * np.maximum(0.0, np.abs(e) - allow)
    # laneOffset that would centre the carriageway lanes at each bin (lo - c_mid), per section
    centring = lo - c_mid
    sec_centring = {}
    for si, sec in enumerate(r.sections):
        m = (secidx == si) & ~trans & ~np.isnan(centring)
        if m.any():
            sec_centring[si] = float(np.median(centring[m]))
    # vertex positions (for the fit sub-pattern): |t| - allowance of every OSM vertex on this road
    vtx = []
    for wi, w in enumerate(ways):
        poly = osm.polyline(w)
        if len(poly) < 2:
            continue
        s, t, d, h = project_points(r, poly)
        for sv, tv, dv in zip(s, t, d):
            if 0.3 < sv < L - 0.3 and dv < 25.0:
                k = min(nb - 1, int(sv / L * nb))
                vtx.append((float(sv), float(abs(tv) - allow[k])))
    gore = gore_mask(r, sb, gores, od) if od is not None else np.zeros(nb, bool)
    return dict(gore=gore, e_adj=e_adj, sec_centring=sec_centring, vtx=vtx, road=r, ways=ways, specs=specs, sb=sb, cov=cov, t_osm=t_osm, allow=allow, wayat=wayat,
                dirat=dirat, lo=lo, c_mid=c_mid, t_exp=t_exp, e=e, trans=trans, n_r=n_r, n_l=n_l, n_bi=n_bi,
                w_r=w_r, w_l=w_l, secidx=secidx, way_end_s=sorted(way_end_s), hwy=hwy, R=R)


def placement_t(sp, pl, dr, b, sec):
    """t where the way should lie from placement tags (None if not applicable). Lanes counted from
    the driver's left = from the inner side (RHT)."""
    if sp["oneway"]:
        key = "placement"
        side = -1 if (dr > 0) != sp["reverse"] else 1  # right side lanes travel +s
    else:
        key = "placement:forward" if "placement:forward" in pl else "placement:backward"
        fwd = key == "placement:forward"
        side = -1 if (dr > 0) == fwd else 1
    if key not in pl:
        return None
    mode, n = pl[key]
    if mode == "transition" or n is None:
        return None
    lanes = [l for l in (sec.right if side < 0 else sec.left) if l.type in CARRIAGEWAY]
    if not (1 <= n <= len(lanes)):
        return None
    ti, to = b[lanes[n - 1].id]
    return {"left_of": ti, "right_of": to, "middle_of": 0.5 * (ti + to)}.get(mode)


# ----------------------------------------------------------------------------- findings
def classify_band(a, i):
    """the mechanisms behind a band offset at bin i (from the road's own data):
    ref_fit            the OSM line is off the reference line beyond the fillet allowance
    overlap_shift      laneOffset rewritten as ~10 m cubic pieces by resolveOverlaps (src/overlap.cpp:209-229)
    no_recentre        laneOffset kept from another section although this section's lanes centre elsewhere
                       (one-sided lane-count change, src/network.cpp:567-578)
    gore_hold          the road carries a gore stretch (part not covered by its own OSM ways) and the
                       offset runs a long hermite toward / from it (src/network.cpp:584-596)
    lane_count_mismatch / placement / asymmetric_twoway   context"""
    r = a["road"]
    sp = a["specs"][a["wayat"][i]]
    tags = []
    if abs(a["t_osm"][i]) - a["allow"][i] > FIT_TOL:
        tags.append("ref_fit")
    band = a["t_exp"][i]
    dense = len(r.offsets) > 2 * len(r.sections) + 2
    if dense:
        tags.append("overlap_shift")
    si = a["secidx"][i]
    lo = a["lo"][i]
    own = a["sec_centring"].get(si)
    if abs(band) > 0.5 and len(r.sections) > 1 and not dense:
        # the offset at this bin is the value another section's lanes would be centred with
        others = [v for k, v in a["sec_centring"].items() if k != si]
        starts = [float(r.lane_offset(np.array([sec.s + 1e-3]))[0]) for k, sec in enumerate(r.sections) if k != si]
        if any(abs(lo - v) < 0.2 for v in others + starts):
            tags.append("no_recentre")
    if sp["placement"] and abs(a["t_exp"][i] - a["c_mid"][i]) > 0.5 and abs(a["t_osm"][i] - a["t_exp"][i]) > 0.5:
        tags.append("placement_ignored")
    if (~a["cov"]).sum() * BIN > 20 and len(r.sections) > 1:
        tags.append("gore_or_uncovered")
    exp_with, exp_ag = sp["n_with"], sp["n_against"]
    dr = a["dirat"][i]
    act_with, act_ag = (a["n_r"][i], a["n_l"][i]) if dr > 0 else (a["n_l"][i], a["n_r"][i])
    if exp_with is not None and (act_with != exp_with or act_ag != exp_ag):
        tags.append("lane_count_mismatch")
    if sp["placement"]:
        tags.append("placement")
    if not sp["oneway"] and a["n_r"][i] != a["n_l"][i]:
        tags.append("asymmetric_twoway")
    return tags


def make_findings(a):
    r = a["road"]
    sb, cov, trans = a["sb"], a["cov"], a["trans"]
    out = []

    def xy(i, t=0.0):
        x, y = r.point(np.array([sb[i]]), np.array([t]))
        return float(x[0]), float(y[0])

    # (a) band offset
    e = a["e_adj"]
    # the lane band itself must be off the reference line (|t_exp| > tol); an OSM line off the reference
    # line with the lanes centred on that line is a ref_fit finding, not a band one
    bad = cov & ~trans & ~a["gore"] & ~np.isnan(e) & (np.abs(e) > BAND_TOL) & (np.abs(np.nan_to_num(a["t_exp"])) > BAND_TOL)
    for i0, i1 in runs(bad):
        seg = np.abs(e[i0:i1 + 1])
        Ls = (i1 - i0 + 1) * BIN
        if Ls < BAND_MINLEN and seg.max() < BAND_HARD:
            continue
        k = i0 + int(np.argmax(seg))
        mx = float(seg.max())
        sev = "critical" if mx >= 3.0 else "major" if mx >= 1.5 else "minor"
        x, y = xy(k, a["t_osm"][k])
        out.append(dict(kind="band_offset", road=r.id, way=a["ways"][a["wayat"][k]], highway=a["hwy"],
                        x=round(x, 2), y=round(y, 2), s0=round(float(sb[i0]), 1), s1=round(float(sb[i1]), 1),
                        length=Ls, max_offset=round(mx, 2), median_offset=round(float(np.median(seg)), 2),
                        signed_at_max=round(float(e[k]), 2), t_osm=round(float(a["t_osm"][k]), 2),
                        carriageway_centre=round(float(a["c_mid"][k]), 2), lane_offset=round(float(a["lo"][k]), 2),
                        lanes_right=int(a["n_r"][k]), lanes_left=int(a["n_l"][k]),
                        fit_component=round(float(a["t_osm"][k]), 2), band_component=round(float(a["t_exp"][k]), 2),
                        allowance=round(float(a["allow"][k]), 2),
                        primary="fit" if abs(a["t_osm"][k]) - a["allow"][k] >= abs(a["t_exp"][k]) else "band",
                        causes=classify_band(a, k), severity=sev))
    # (b) reference-line fit
    ex = np.abs(np.nan_to_num(a["t_osm"])) - a["allow"]
    bad = cov & (ex > FIT_TOL)
    for i0, i1 in runs(bad):
        Ls = (i1 - i0 + 1) * BIN
        if Ls < FIT_MINLEN:
            continue
        seg = ex[i0:i1 + 1]
        k = i0 + int(np.argmax(seg))
        x, y = xy(k, a["t_osm"][k])
        mx = float(abs(a["t_osm"][k]))
        s0, s1 = sb[i0] - BIN, sb[i1] + BIN
        vin = [d for sv, d in a["vtx"] if s0 <= sv <= s1]
        sub = "vertex_off" if any(d > FIT_TOL for d in vin) else "bow_between_vertices"
        out.append(dict(kind="ref_fit", subpattern=sub, osm_vertices_in_run=len(vin), road=r.id, way=a["ways"][a["wayat"][k]], highway=a["hwy"], x=round(x, 2),
                        y=round(y, 2), s0=round(float(sb[i0]), 1), s1=round(float(sb[i1]), 1), length=Ls,
                        max_fit=round(mx, 2), allowance=round(float(a["allow"][k]), 2), R=round(a["R"], 1),
                        severity="major" if mx - a["allow"][k] >= 2.0 else "minor"))
    # (c) lane counts
    mism = np.zeros(len(sb), bool)
    exp_desc = [None] * len(sb)
    for i in range(len(sb)):
        if not cov[i] or trans[i] or a["gore"][i]:
            continue
        sp = a["specs"][a["wayat"][i]]
        dr = a["dirat"][i]
        aw, ag = (a["n_r"][i], a["n_l"][i]) if dr > 0 else (a["n_l"][i], a["n_r"][i])
        if sp["n_with"] is not None:
            ok = aw == sp["n_with"] and ag == sp["n_against"]
            exp_desc[i] = (sp["n_with"], sp["n_against"], sp["both"], sp["default"])
        elif sp["total"] is not None:
            ok = aw + ag + a["n_bi"][i] == sp["total"]
            exp_desc[i] = ("sum", sp["total"], sp["both"], sp["default"])
        else:
            continue
        if ok and sp["both"] != a["n_bi"][i] and not sp["oneway"]:
            ok = False
        if not ok and near_boundary_ok(a, i, aw, ag):
            ok = True
        mism[i] = not ok
    for i0, i1 in runs(mism):
        Ls = (i1 - i0 + 1) * BIN
        if Ls < COUNT_MINLEN:
            continue
        k = (i0 + i1) // 2
        sp = a["specs"][a["wayat"][k]]
        dr = a["dirat"][k]
        aw, ag = (int(a["n_r"][k]), int(a["n_l"][k])) if dr > 0 else (int(a["n_l"][k]), int(a["n_r"][k]))
        ed = exp_desc[k]
        if ed[0] == "sum":
            diff = abs(aw + ag - ed[1])
            missing = False
        else:
            diff = abs(aw - ed[0]) + abs(ag - ed[1])
            missing = (ed[0] > 0 and aw == 0) or (ed[1] > 0 and ag == 0)
        sev = "critical" if (missing or diff >= 2) else "major"
        if sp["default"] and sev == "major":
            sev = "minor"
        sub = "other"
        sec_here = r.sections[a["secidx"][k]]
        if sp["n_with"] is not None and ((sp["n_with"] > 0) != (aw > 0) or (sp["n_against"] > 0) != (ag > 0)):
            sub, sev = "travel_direction_wrong", "critical"
        elif len(r.sections) == 1 and r.length < 50.0 and len({(q["n_with"], q["n_against"], q["total"]) for q in a["specs"]}) > 1:
            sub = "short_road_single_layout"
        elif sp["default"] and sp["hw"] in LINK_DEFAULT_1:
            sub = "untagged_link_defaults_to_parent_class"
        else:
            for q in a["specs"]:
                if not q["tagged"] or q["n_with"] is None:
                    dw, dg = converter_default(q)
                    if {(dw, dg), (dg, dw)} & {(aw, ag)}:
                        sub = "untagged_way_layout_spread"
                        break
        x, y = xy(k)
        out.append(dict(kind="lane_count", subpattern=sub, road=r.id, way=a["ways"][a["wayat"][k]], highway=a["hwy"], x=round(x, 2),
                        y=round(y, 2), s0=round(float(sb[i0]), 1), s1=round(float(sb[i1]), 1), length=Ls,
                        actual_with_way=aw, actual_against_way=ag, actual_bidirectional=int(a["n_bi"][k]),
                        expected=("total=%d" % ed[1]) if ed[0] == "sum" else "%d/%d" % (ed[0], ed[1]),
                        expected_from_default=bool(sp["default"]), osm_lanes=sp["total"], turn_lanes_n=sp["turn_n"],
                        n_sections=len(r.sections), road_length=round(r.length, 1), severity=sev))
    # (d) explicit width tags
    wbad = np.zeros(len(sb), bool)
    wdesc = [None] * len(sb)
    for i in range(len(sb)):
        if not cov[i] or trans[i] or a["gore"][i]:
            continue
        sp = a["specs"][a["wayat"][i]]
        dr = a["dirat"][i]
        aw_w, ag_w = (a["w_r"][i], a["w_l"][i]) if dr > 0 else (a["w_l"][i], a["w_r"][i])
        exp = None
        if sp["w_lanes_with"] and len(sp["w_lanes_with"]) == len(aw_w):
            exp = [(x, y) for x, y in zip(aw_w, sp["w_lanes_with"]) if y]
        elif sp["width"] and (aw_w or ag_w):
            n = len(aw_w) + len(ag_w) + int(a["n_bi"][i])
            wl = min(max(sp["width"] / max(1, n), 2.2), 4.5)
            exp = [(x, wl) for x in aw_w + ag_w]
        if exp:
            dv = max(abs(x - y) for x, y in exp)
            wdesc[i] = dv
            wbad[i] = dv > WIDTH_TOL
    for i0, i1 in runs(wbad):
        Ls = (i1 - i0 + 1) * BIN
        if Ls < WIDTH_MINLEN:
            continue
        k = (i0 + i1) // 2
        x, y = xy(k)
        out.append(dict(kind="lane_width", road=r.id, way=a["ways"][a["wayat"][k]], highway=a["hwy"], x=round(x, 2),
                        y=round(y, 2), s0=round(float(sb[i0]), 1), s1=round(float(sb[i1]), 1), length=Ls,
                        max_width_error=round(float(wdesc[k]), 2), severity="minor"))
    return out


def near_boundary_ok(a, i, aw, ag):
    """a count that matches another way of this road within BOUNDARY_TOL is the converter's section
    shift / taper placement, not a fidelity error"""
    s = a["sb"][i]
    if not any(abs(s - q) < BOUNDARY_TOL for q in a["way_end_s"]) and not (s < BOUNDARY_TOL or s > a["road"].length - BOUNDARY_TOL):
        return False
    lo, hi = s - BOUNDARY_TOL, s + BOUNDARY_TOL
    sb = a["sb"]
    for j in np.nonzero((sb > lo) & (sb < hi) & a["cov"])[0]:
        sp = a["specs"][a["wayat"][j]]
        if sp["n_with"] is None:
            continue
        dr = a["dirat"][j]
        ew, eg = (sp["n_with"], sp["n_against"]) if dr == a["dirat"][i] else (sp["n_against"], sp["n_with"])
        if (aw, ag) == (ew, eg):
            return True
    return False


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xodr")
    ap.add_argument("osm")
    ap.add_argument("--out", required=True)
    ap.add_argument("--basemap")
    ap.add_argument("--crops", type=int, default=12, help="worst findings to render (+ a few typical)")
    ap.add_argument("--roads", help="comma separated road ids (debug)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    od = G.OpenDrive(args.xodr)
    osm = G.Osm(args.osm, od.lat0, od.lon0)
    roads = od.plain_roads()
    if args.roads:
        want = set(args.roads.split(","))
        roads = [r for r in roads if r.id in want]
    gores = gore_junctions(od)
    findings, stats = [], collections.Counter()
    cov_gap = []
    allE = collections.defaultdict(list)
    allF = collections.defaultdict(list)
    for r in roads:
        a = analyze_road(r, osm, gores, od)
        if a is None:
            stats["roads_skipped"] += 1
            continue
        stats["roads_checked"] += 1
        stats["length_checked_m"] += r.length
        stats["gore_stretch_m"] += float(a["gore"].sum()) * BIN
        unc = (~a["cov"]).sum() * BIN
        if unc > 20:
            cov_gap.append((r.id, unc))
        good = a["cov"] & ~a["trans"] & ~a["gore"] & ~np.isnan(a["e_adj"])
        allE[a["hwy"]].append(np.abs(a["e_adj"][good]))
        fit = np.abs(a["t_osm"][a["cov"]]) - a["allow"][a["cov"]]
        allF[a["hwy"]].append(np.maximum(fit, 0))
        findings += make_findings(a)
    for n, f in enumerate(findings):
        f["id"] = "osm_fidelity-%04d" % n
        if f["kind"] == "band_offset":
            f["subpattern"] = None
    sevrank = {"critical": 0, "major": 1, "minor": 2}

    def score(f):
        m = f.get("max_offset") or f.get("max_fit") or 0
        if f["kind"] == "lane_count":
            m = 2.0 + abs(f["actual_with_way"] + f["actual_against_way"] - (f["osm_lanes"] or 0))
        return (sevrank[f["severity"]], -m * min(f["length"], 200))
    findings.sort(key=score)
    json.dump(dict(thresholds=dict(band_tol=BAND_TOL, band_hard=BAND_HARD, band_minlen=BAND_MINLEN, fit_tol=FIT_TOL,
                                   fit_minlen=FIT_MINLEN, count_minlen=COUNT_MINLEN, boundary_tol=BOUNDARY_TOL,
                                   width_tol=WIDTH_TOL, width_minlen=WIDTH_MINLEN),
                   stats=dict(stats), findings=findings), open(os.path.join(args.out, "findings.json"), "w"), indent=1)
    # summary
    lines = ["osm_fidelity  roads checked %d (skipped %d), %.1f km" % (stats["roads_checked"], stats["roads_skipped"],
                                                                      stats["length_checked_m"] / 1000)]
    by = collections.Counter((f["kind"], f["severity"]) for f in findings)
    lines.append("findings %d" % len(findings))
    for k in sorted(by):
        lines.append("  %-12s %-8s %d" % (k[0], k[1], by[k]))
    lines.append("by kind x highway:")
    for k, v in sorted(collections.Counter((f["kind"], f["highway"]) for f in findings).items()):
        lines.append("  %-12s %-14s %d" % (k[0], k[1], v))
    lines.append("band-offset causes (tags co-occurring at the worst bin):")
    for k, v in collections.Counter(c for f in findings if f["kind"] == "band_offset" for c in f["causes"]).most_common():
        lines.append("  %-22s %d" % (k, v))
    lines.append("sub-patterns (kind / subpattern or primary cause):")
    def key(f):
        if f["kind"] == "band_offset":
            c = set(f["causes"])
            for k in ("placement_ignored", "overlap_shift", "no_recentre", "gore_or_uncovered"):
                if k in c and f["primary"] == "band":
                    return k
            return "ref_fit" if f["primary"] == "fit" else "band_other"
        return f.get("subpattern", "-")
    agg = collections.defaultdict(lambda: [0, 0.0, collections.Counter()])
    for f in findings:
        g = agg[(f["kind"], key(f))]
        g[0] += 1
        g[1] += f["length"]
        g[2][f["severity"]] += 1
    for k, (n, L, sv) in sorted(agg.items(), key=lambda kv: -kv[1][1]):
        lines.append("  %-12s %-40s n=%4d  len=%7.0f m  %s" % (k[0], k[1], n, L, dict(sv)))
    lines.append("lane_count expected->actual (with/against way):")
    for k, v in collections.Counter((f["highway"], f["expected"], "%d/%d" % (f["actual_with_way"], f["actual_against_way"]))
                                    for f in findings if f["kind"] == "lane_count").most_common(25):
        lines.append("  %-14s exp %-8s act %-6s %d" % (k[0], k[1], k[2], v))
    lines.append("|band offset| percentiles per highway over all checked bins (p50 / p95 / p99 / max, m):")
    for hw, arrs in sorted(allE.items()):
        v = np.concatenate(arrs) if arrs else np.zeros(0)
        if len(v):
            lines.append("  %-14s n=%7d  %.2f / %.2f / %.2f / %.2f" % (hw, len(v), *np.percentile(v, [50, 95, 99]), v.max()))
    lines.append("|ref-line fit error| beyond fillet allowance per highway (p50 / p95 / p99 / max, m):")
    for hw, arrs in sorted(allF.items()):
        v = np.concatenate(arrs) if arrs else np.zeros(0)
        if len(v):
            lines.append("  %-14s n=%7d  %.2f / %.2f / %.2f / %.2f" % (hw, len(v), *np.percentile(v, [50, 95, 99]), v.max()))
    lines.append("roads with > 20 m not covered by their own OSM ways (gore extensions etc.): %d" % len(cov_gap))
    open(os.path.join(args.out, "summary.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    # crops
    if args.basemap and args.crops:
        cd = os.path.join(args.out, "crops")
        os.makedirs(cd, exist_ok=True)
        # the worst overall, then per sub-pattern its worst and its median (typical) instance
        pick, seen = [], set()
        for f in findings[:args.crops]:
            if f["road"] not in seen:
                pick.append(f), seen.add(f["road"])
        groups = collections.defaultdict(list)
        for f in findings:
            groups[(f["kind"], key(f))].append(f)
        for g in groups.values():
            for f in (g[0], g[len(g) // 2]):
                if f["road"] not in seen:
                    pick.append(f), seen.add(f["road"])
        for f in findings:
            f["pattern"] = key(f)
        for f in pick:
            half = 60 if f["highway"] in ("motorway", "motorway_link", "trunk") else 40
            G.crop(od, osm, (f["x"], f["y"]), half, os.path.join(cd, f["id"] + ".png"), basemap=args.basemap,
                   marks=[(f["x"], f["y"], f["kind"])],
                   title="%s %s/%s road %s way %s %s" % (f["id"], f["kind"], key(f), f["road"], f["way"], f["severity"]))
        json.dump(dict(thresholds=dict(band_tol=BAND_TOL, band_hard=BAND_HARD, band_minlen=BAND_MINLEN, fit_tol=FIT_TOL,
                                       fit_minlen=FIT_MINLEN, count_minlen=COUNT_MINLEN, boundary_tol=BOUNDARY_TOL,
                                       width_tol=WIDTH_TOL, width_minlen=WIDTH_MINLEN),
                       stats=dict(stats), findings=findings), open(os.path.join(args.out, "findings.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
