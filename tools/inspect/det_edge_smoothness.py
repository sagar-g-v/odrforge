"""det_edge_smoothness -- lane-border smoothness inspector for OdrForge OpenDRIVE output.

Every lane border of every road must be a smooth (C1) curve that follows the reference line.
This detector evaluates the OpenDRIVE description *analytically* (polynomial values and
derivatives, element headings / curvatures) and reports every place where that contract is
broken.  Checks (finding "type" values):

  ref_discontinuity   planView element joins: position gap > 0.01 m or heading jump > 0.2 deg.
                      Every border inherits such a kink.
  border_jump         lateral discontinuity of a border (|dt| > 0.01 m) at a breakpoint: laneSection
                      start (lanes followed through their predecessor links), width-record start,
                      laneOffset-record start.  Sub-types: offset_jump (lane 0), lane_continuation
                      (linked lane), lane_appears (new lane starting with width > 0), lane_vanishes
                      (lane ending with width > 0), width_record (jump between width records).
  border_kink         heading discontinuity of a border at a breakpoint, psi = hdg + atan2(t', 1 - k t);
                      |dpsi| > 0.5 deg.  (Same breakpoints; slope discontinuities of laneOffset/width.)
  negative_width      any lane width < -0.01 m (borders crossing each other).
  fold                a border on the inside of a curve with k*t >= 1 (offset curve cusps / reverses):
                      sub-type folded; or border radius (1 - k t)/|k| < 1 m: sub-type near_fold.
  border_sharp_bend   border turning relative to the reference line, d/ds atan2(t', 1 - k t), above
                      0.1 rad/m (a lateral jog with < 10 m radius that the reference line does not have).
  width_wobble        a width or laneOffset polynomial overshoots the range of its end values by
                      > 0.02 m (non-monotone transition), or a lane's width changes direction more
                      than once inside one laneSection.
  steep_transition    lateral rate |dt/ds| of any lane border (laneOffset shift, width taper, or a lane
                      pushed sideways by tapers of lanes inside it) above the
                      MUTCD-derived limit for the posted speed S (mph):
                          mean shifting-taper rate  r = 2/S          (S >= 45)
                                                    r = 120/S^2      (S <  45, S floored at 20)
                      (MUTCD 6C: merging taper L = W*S resp. W*S^2/60 ft, shifting taper >= L/2).
                      Smooth (Hermite) transitions peak at 1.5x their mean rate, so the flagged peak
                      rate is |dt/ds| > 1.5 r.  Connecting roads (inside junctions) are exempt from this
                      check: their width / offset transitions are the junction's own geometry.
  ref_outside_carriageway  (plain roads) the reference line (= OSM way) lies > 0.5 m outside the band
                      spanned by the driving lanes: the lanes no longer follow the reference line.

Severity: critical = visible geometric breakage (jump >= 0.5 m, kink >= 5 deg, fold, negative width
< -0.1 m, rate > 4x limit); major = jump >= 0.1 m, kink >= 2 deg, rate > 2x limit, wobble >= 0.2 m;
minor otherwise.  A correct converter produces zero findings: every threshold is above float/print
precision and below anything a legitimate (C1, tag-driven, MUTCD-compliant) geometry produces.

CLI:  python det_edge_smoothness.py OUT.xodr AREA.osm --out DIR [--basemap DIR] [--crops N]
Writes DIR/findings.json, DIR/summary.txt and evidence crops DIR/crops/*.png.
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

STEP = 0.5            # m, sampling along s
JUMP_TOL = 0.01       # m, lateral discontinuity
KINK_TOL = math.radians(0.5)
REF_POS_TOL = 0.01    # m
REF_HDG_TOL = math.radians(0.2)
NEG_TOL = -0.01       # m
WOBBLE_TOL = 0.02     # m
PLATEAU = 5.0         # m, constant width between two width changes that makes them separate transitions
FOLD_TOL = 1.0        # k*t
NEAR_FOLD_R = 1.0     # m, minimum border radius on the inside of a curve
OUTSIDE_TOL = 0.5     # m
BEND_TOL = 0.1        # 1/m, border curvature in excess of the ref line's parallel curve (R_lateral < 10 m)
MIN_SPEED = 20.0      # mph


# ----------------------------------------------------------------------------- helpers
def poly(rec, x, d=0):
    a, b, c, dd = rec
    if d == 0:
        return a + b * x + c * x * x + dd * x ** 3
    if d == 1:
        return b + 2 * c * x + 3 * dd * x * x
    return 2 * c + 6 * dd * x


def pw(recs, x, d=0, left=False):
    """piecewise cubic (records [(start, abcd)]) at x; left=True takes the record ending at x."""
    x = np.atleast_1d(np.asarray(x, float))
    if not recs:  # no records (a road without <laneOffset>): 0
        return np.zeros_like(x)
    starts = np.array([r[0] for r in recs])
    idx = np.searchsorted(starts, x - (1e-9 if left else -1e-9), side="right") - 1
    idx = np.clip(idx, 0, len(recs) - 1)
    out = np.empty_like(x)
    for i in np.unique(idx):
        m = idx == i
        out[m] = poly(recs[i][1], x[m] - recs[i][0], d)
    return out


def rate_limit(speed_mph):
    """MUTCD shifting-taper mean lateral rate (m/m) for a posted speed."""
    S = max(MIN_SPEED, speed_mph or 25.0)
    return 2.0 / S if S >= 45 else 120.0 / (S * S)


class RefLine:
    """Analytic heading / curvature / curvature-derivative of a road's planView."""

    def __init__(self, road):
        self.g = []
        for e in road.el.find("planView").findall("geometry"):
            s0, L, h0 = float(e.get("s")), float(e.get("length")), float(e.get("hdg"))
            k = e[0]
            if k.tag == "line":
                k0 = k1 = 0.0
            elif k.tag == "arc":
                k0 = k1 = float(k.get("curvature"))
            elif k.tag == "spiral":
                k0, k1 = float(k.get("curvStart")), float(k.get("curvEnd"))
            else:
                k0 = k1 = None  # numeric fallback
            self.g.append((s0, L, h0, k0, k1, k.tag, e))
        self.starts = np.array([g[0] for g in self.g])
        self.road = road

    def _idx(self, s, left=False):
        s = np.atleast_1d(np.asarray(s, float))
        i = np.searchsorted(self.starts, s - (1e-9 if left else -1e-9), side="right") - 1
        return np.clip(i, 0, len(self.g) - 1), s

    def hk(self, s, left=False):
        """heading, curvature, d curvature/ds"""
        idx, s = self._idx(s, left)
        h, k, dk = np.empty_like(s), np.empty_like(s), np.empty_like(s)
        for i in np.unique(idx):
            m = idx == i
            s0, L, h0, k0, k1, tag, _ = self.g[i]
            ds = s[m] - s0
            if k0 is None:
                _, _, hh = self.road.ref(s[m])
                h[m], k[m], dk[m] = hh, 0.0, 0.0
                continue
            c = (k1 - k0) / L if L > 0 else 0.0
            h[m] = h0 + k0 * ds + 0.5 * c * ds * ds
            k[m] = k0 + c * ds
            dk[m] = c
        return h, k, dk

    def joins(self):
        """[(s, position gap, heading jump)] at element boundaries"""
        out = []
        for i in range(1, len(self.g)):
            s0, L, h0, k0, k1, tag, e = self.g[i - 1]
            s1, _, h1, *_ = self.g[i]
            if L <= 1e-9:
                continue
            xe, ye = G._eval_geometry(e, np.array([L]))
            dx = float(xe[0]) - float(self.g[i][6].get("x"))
            dy = float(ye[0]) - float(self.g[i][6].get("y"))
            if k0 is None:
                he = float(self.road.ref(s1 - 1e-3)[2])
            else:
                he = h0 + k0 * L + 0.5 * (k1 - k0) * L
            dh = (h1 - he + math.pi) % (2 * math.pi) - math.pi
            out.append((s1, math.hypot(dx, dy), dh))
        return out


def road_speed(road):
    """posted speed (mph) per road type record; lane speed as fallback"""
    recs = []
    for t in road.el.findall("type"):
        sp = t.find("speed")
        if sp is not None and sp.get("max") not in (None, "no limit", "undefined"):
            v = float(sp.get("max"))
            u = sp.get("unit", "m/s")
            mph = v if u == "mph" else v / 1.609344 if u == "km/h" else v / 0.44704
            recs.append((float(t.get("s")), mph))
    if not recs:
        for l in road.el.iter("lane"):
            sp = l.find("speed")
            if sp is not None:
                v, u = float(sp.get("max")), sp.get("unit", "m/s")
                recs.append((0.0, v if u == "mph" else v / 1.609344 if u == "km/h" else v / 0.44704))
                break
    return sorted(recs) or [(0.0, 25.0)]


def speed_at(recs, s):
    v = recs[0][1]
    for s0, x in recs:
        if s0 <= s + 1e-9:
            v = x
    return v


# ----------------------------------------------------------------------------- per-road analysis
class RoadCheck:
    def __init__(self, road, emit):
        self.r = road
        self.emit = emit
        self.ref = RefLine(road)
        self.speed = road_speed(road)
        self.plain = road.junction == "-1"

    def borders(self, sec, s, d=0, left=False):
        """{lane id: t^(d) of the lane's OUTER border} (lane 0 = laneOffset) at array s within sec"""
        ds = s - sec.s
        off = pw(self.r.offsets, s, d, left)
        out = {0: off}
        for side in (sec.left, sec.right):
            sign = 1.0 if side is sec.left else -1.0
            t = off.copy()
            for l in side:
                t = t + sign * pw(l.widths, ds, d, left)
                out[l.id] = t
        return out

    def widths(self, sec, s, d=0, left=False):
        return {l.id: pw(l.widths, s - sec.s, d, left) for l in sec.lanes()}

    def psi(self, s, t, t1, left=False):
        h, k, _ = self.ref.hk(s, left)
        return h + np.arctan2(t1, 1 - k * t)

    def dpsi(self, sa, tb, db, ta, da):
        """wrapped heading difference right-limit minus left-limit at scalar array sa"""
        d = float(self.psi(sa, tb, db)[0] - self.psi(sa, ta, da, True)[0])
        return (d + math.pi) % (2 * math.pi) - math.pi

    def xy(self, s, t=0.0):
        x, y = self.r.point(np.atleast_1d(s), np.atleast_1d(t))
        return float(x[0]), float(y[0])

    def run(self):
        r = self.r
        # 1. reference line joins
        for s, gap, dh in self.ref.joins():
            if gap > REF_POS_TOL or abs(dh) > REF_HDG_TOL:
                sev = "critical" if gap >= 0.5 or abs(dh) >= math.radians(5) else \
                      "major" if gap >= 0.1 or abs(dh) >= math.radians(2) else "minor"
                self.emit("ref_discontinuity", "element_join", r, s, None, sev,
                          {"gap_m": gap, "kink_deg": math.degrees(dh)})
        # 2. breakpoints inside sections (width records, laneOffset records, element joins)
        off_bps = [o[0] for o in r.offsets[1:]]
        geo_bps = [g[0] for g in self.ref.g[1:]]
        for sec in r.sections:
            bps = {}
            for s in off_bps:
                if sec.s + 1e-6 < s < sec.e - 1e-6:
                    bps.setdefault(round(s, 6), set()).add("offset_record")
            for l in sec.lanes():
                for so, _ in l.widths[1:]:
                    s = sec.s + so
                    if sec.s + 1e-6 < s < sec.e - 1e-6:
                        bps.setdefault(round(s, 6), set()).add("width_record")
            for s in geo_bps:
                if sec.s + 1e-6 < s < sec.e - 1e-6:
                    bps.setdefault(round(s, 6), set()).add("geometry")
            for s, why in sorted(bps.items()):
                sa = np.array([s])
                tl, tr = self.borders(sec, sa, 0, True), self.borders(sec, sa, 0, False)
                dl, dr = self.borders(sec, sa, 1, True), self.borders(sec, sa, 1, False)
                jumps, kinks = {}, {}
                for lid in tl:
                    j = float(tr[lid][0] - tl[lid][0])
                    if abs(j) > JUMP_TOL:
                        jumps[lid] = j
                    if "geometry" in why and len(why) == 1:
                        continue  # pure element joins: heading continuity handled by ref check
                    k = self.dpsi(sa, tr[lid], dr[lid], tl[lid], dl[lid])
                    if abs(k) > KINK_TOL:
                        kinks[lid] = k
                sub = "offset_jump" if 0 in jumps else "width_record"
                self._emit_bp("border_jump", sub, s, jumps, why)
                self._emit_bp("border_kink", "offset_record" if "offset_record" in why else "width_record", s, kinks, why)
        # 3. section boundaries (lanes followed through predecessor links)
        for a, b in zip(r.sections, r.sections[1:]):
            s = b.s
            sa = np.array([s])
            ta, tb = self.borders(a, sa, 0, True), self.borders(b, sa, 0, False)
            da, db = self.borders(a, sa, 1, True), self.borders(b, sa, 1, False)
            wa, wb = self.widths(a, sa, 0, True), self.widths(b, sa, 0, False)
            jumps = collections.defaultdict(dict)
            kinks = {}
            jo = float(tb[0][0] - ta[0][0])
            if abs(jo) > JUMP_TOL:
                jumps["offset_jump"][0] = jo
            for l in b.lanes():
                if l.pred is None or a.lane(l.pred) is None:
                    if abs(wb[l.id][0]) > JUMP_TOL:
                        jumps["lane_appears"][l.id] = float(wb[l.id][0])
                    continue
                j = float(tb[l.id][0] - ta[l.pred][0])
                if abs(j) > JUMP_TOL:
                    jumps["lane_continuation"][l.id] = j
                k = self.dpsi(sa, tb[l.id], db[l.id], ta[l.pred], da[l.pred])
                if abs(k) > KINK_TOL:
                    kinks[l.id] = k
            succ_ids = {l.pred for l in b.lanes() if l.pred is not None}
            for l in a.lanes():
                if l.id not in succ_ids and (l.succ is None or b.lane(l.succ) is None):
                    if abs(wa[l.id][0]) > JUMP_TOL:
                        jumps["lane_vanishes"][l.id] = float(wa[l.id][0])
            for sub, d in jumps.items():
                self._emit_bp("border_jump", sub, s, d, {"section"}, lane_types=self._types(a, b))
            self._emit_bp("border_kink", "section", s, kinks, {"section"}, lane_types=self._types(a, b))
        # 4. sampled checks per section
        for sec in r.sections:
            n = max(2, int(math.ceil((sec.e - sec.s) / STEP)) + 1)
            s = np.linspace(sec.s, sec.e, n)
            # planView element ends are where curvature peaks (spirals): sample them on both sides
            gb = [g[0] for g in self.ref.g[1:] if sec.s + 1e-3 < g[0] < sec.e - 1e-3]
            s = np.unique(np.concatenate([s, np.array(gb) - 1e-6, np.array(gb) + 1e-6]))
            s[0] += 1e-6
            s[-1] -= 1e-6
            h, k, dk = self.ref.hk(s)
            W = self.widths(sec, s)
            T = self.borders(sec, s)
            T1 = self.borders(sec, s, 1)
            types = {l.id: l.type for l in sec.lanes()}
            # negative width
            for lid, w in W.items():
                self._intervals(s, w < NEG_TOL, -w, "negative_width", types[lid], lid,
                                lambda m: "critical" if m > 0.1 else "major", "neg_width_m")
            # fold: border on the inner side of a curve with k t >= 1 (folded: the offset curve has a
            # cusp and runs backwards), or a border radius (1 - k t)/|k| < 1 m (near_fold: a corner)
            for lid, t in T.items():
                kt = k * t
                rb = np.where(np.abs(k) > 1e-9, (1 - kt) / np.maximum(np.abs(k), 1e-9), 1e9)
                ex = lambda i, lid=lid, rb=rb: {"ref_radius_m": 1 / abs(k[i]) if k[i] else None, "t_m": float(T[lid][i]),
                                                 "border_radius_m": float(rb[i])}
                self._intervals(s, kt >= FOLD_TOL, kt, "fold", types.get(lid, "center"), lid,
                                lambda m: "critical", "k_times_t", extra=ex, sub="folded")
                self._intervals(s, (kt < FOLD_TOL) & (kt > 0) & (rb < NEAR_FOLD_R), kt, "fold", types.get(lid, "center"), lid,
                                lambda m: "major", "k_times_t", extra=ex, sub="near_fold")
            # sharp bend: turning of the border relative to the reference line per metre of s,
            # e = d/ds atan2(t', 1 - k t); > BEND_TOL means the border bends by itself (a lateral
            # jog / short S).  Evaluated where 1 - k t >= 0.5 (closer to a fold the fold check rules).
            T2 = self.borders(sec, s, 2)
            for lid, t in T.items():
                if types.get(lid) in ("sidewalk", "border", "curb"):
                    continue
                a1, b1 = T1[lid], 1 - k * t
                da1, db1 = T2[lid], -(dk * t + k * T1[lid])
                q = a1 * a1 + b1 * b1
                e = np.abs((da1 * b1 - a1 * db1) / q)
                self._intervals(s, (e > BEND_TOL) & (b1 >= 0.5), e, "border_sharp_bend", types.get(lid, "center"), lid,
                                lambda m: "critical" if m > 0.5 else "major" if m > 0.2 else "minor", "excess_curv_1pm",
                                extra=lambda i, a1=a1: {"dt_ds": float(a1[i]), "ref_curv": float(k[i])})
            # wobble: per-record overshoot
            for l in sec.lanes():
                for i, (so, rec) in enumerate(l.widths):
                    se = l.widths[i + 1][0] if i + 1 < len(l.widths) else sec.e - sec.s
                    ov = self._overshoot(rec, max(0.0, se - so))
                    if ov > WOBBLE_TOL:
                        self.emit("width_wobble", "width_overshoot", r, sec.s + so + 0.5 * (se - so), [l.id],
                                  "major" if ov >= 0.2 else "minor", {"overshoot_m": ov, "lane_type": l.type})
                # direction changes within the section; a change of direction across a constant-width
                # stretch (PLATEAU m or more) is two width transitions (consecutive ways tagged with
                # different widths), not a wobble
                w1 = W[l.id]
                dw = np.diff(w1)
                nz = np.nonzero(np.abs(dw) > 1e-4)[0]
                sg = np.sign(dw[nz])
                flips = (sg[1:] != sg[:-1]) & (s[nz[1:]] - s[nz[:-1] + 1] < PLATEAU) if len(sg) > 1 else np.zeros(0, bool)
                ch = int(np.sum(flips))
                first = sg[int(np.argmax(flips))] if ch else 0  # direction before the first reversal
                if ch > 1 or (ch == 1 and first < 0):  # fall-then-rise, or more than one reversal
                    i = int(np.argmin(w1)) if first < 0 else int(np.argmax(w1))
                    self.emit("width_wobble", "width_reversal", r, float(s[i]), [l.id], "major",
                              {"reversals": ch, "w_min": float(w1.min()), "w_max": float(w1.max()), "lane_type": l.type})
            # steep transitions (plain roads): lateral rate of every lane border (lane 0 = laneOffset),
            # i.e. laneOffset shifts, width tapers and lanes pushed sideways by tapers inside them
            if self.plain:
                lim = np.array([rate_limit(speed_at(self.speed, x)) for x in s])
                for lid, t1 in T1.items():
                    if types.get(lid) in ("sidewalk", "border", "curb", "none"):
                        continue
                    rt = np.abs(t1)
                    rw = np.abs(pw(sec.lane(lid).widths, s - sec.s, 1)) if lid else np.zeros_like(s)
                    ro = np.abs(pw(r.offsets, s, 1))
                    self._intervals(s, rt > 1.5 * lim, rt / lim, "steep_transition", types.get(lid, "center"), lid,
                                    self._rate_sev, "rate_over_limit",
                                    extra=lambda i, rt=rt, rw=rw, ro=ro: {
                                        "peak_rate": float(rt[i]), "limit_mean_rate": float(lim[i]),
                                        "speed_mph": speed_at(self.speed, float(s[i])),
                                        "own_width_rate": float(rw[i]), "laneOffset_rate": float(ro[i]),
                                        "section_len": float(sec.e - sec.s),
                                        "width_record_len": self._rec_len(sec.lane(lid).widths if lid else None, float(s[i]) - sec.s, sec.e - sec.s),
                                        "offset_record_len": self._rec_len(r.offsets, float(s[i]), r.length)})
                # reference line outside the driving band
                drive = [l.id for l in sec.lanes() if l.type in ("driving", "bidirectional")]
                if drive:
                    lo_t = np.min([np.minimum(T[i], T[i] - (W[i] if i > 0 else -W[i])) for i in drive], axis=0)
                    hi_t = np.max([np.maximum(T[i], T[i] - (W[i] if i > 0 else -W[i])) for i in drive], axis=0)
                    outside = np.maximum(lo_t, -hi_t)
                    self._intervals(s, outside > OUTSIDE_TOL, outside, "ref_outside_carriageway", "driving", None,
                                    lambda m: "critical" if m > 3 else "major" if m > 1 else "minor", "dist_m")
        # laneOffset record overshoot
        for i, (so, rec) in enumerate(r.offsets):
            se = r.offsets[i + 1][0] if i + 1 < len(r.offsets) else r.length
            ov = self._overshoot(rec, max(0.0, se - so))
            if ov > WOBBLE_TOL:
                self.emit("width_wobble", "offset_overshoot", r, so + 0.5 * (se - so), [0],
                          "major" if ov >= 0.2 else "minor", {"overshoot_m": ov})

    @staticmethod
    def _rec_len(recs, x, end):
        """length of the polynomial record active at x (None for a constant record)"""
        if not recs:
            return None
        i = max(k for k in range(len(recs)) if recs[k][0] <= x + 1e-9) if recs[0][0] <= x + 1e-9 else 0
        if all(abs(c) < 1e-12 for c in recs[i][1][1:]):
            return None
        e = recs[i + 1][0] if i + 1 < len(recs) else end
        return round(e - recs[i][0], 3)

    @staticmethod
    def _rate_sev(m):
        return "critical" if m > 4 else "major" if m > 2 else "minor"

    @staticmethod
    def _types(a, b):
        return {**{l.id: l.type for l in a.lanes()}, **{l.id: l.type for l in b.lanes()}}

    @staticmethod
    def _overshoot(rec, L):
        if L <= 1e-6:
            return 0.0
        a, b, c, d = rec
        xs = [0.0, L]
        if abs(d) > 1e-15:
            disc = 4 * c * c - 12 * d * b
            if disc >= 0:
                for sg in (1, -1):
                    x = (-2 * c + sg * math.sqrt(disc)) / (6 * d)
                    if 0 < x < L:
                        xs.append(x)
        elif abs(c) > 1e-15:
            x = -b / (2 * c)
            if 0 < x < L:
                xs.append(x)
        v = [poly(rec, x) for x in xs]
        lo, hi = min(v[0], v[1]), max(v[0], v[1])
        return max(0.0, max(v) - hi, lo - min(v))

    def _emit_bp(self, typ, sub, s, d, why, lane_types=None):
        if not d:
            return
        lid, v = max(d.items(), key=lambda kv: abs(kv[1]))
        if typ == "border_kink":
            deg = abs(math.degrees(v))
            sev = "critical" if deg >= 5 else "major" if deg >= 2 else "minor"
            met = {"kink_deg": math.degrees(v), "per_lane_deg": {str(k): round(math.degrees(x), 3) for k, x in d.items()}}
        else:
            a = abs(v)
            sev = "critical" if a >= 0.5 else "major" if a >= 0.1 else "minor"
            met = {"jump_m": v, "per_lane_m": {str(k): round(x, 4) for k, x in d.items()}}
        met["breakpoint"] = sorted(why)
        if lane_types:
            met["lane_type"] = lane_types.get(lid)
        self.emit(typ, sub, self.r, s, sorted(d), sev, met, t=0.0)

    def _intervals(self, s, mask, metric, typ, ltype, lid, sevf, mname, extra=None, sub=None):
        if not mask.any():
            return
        idx = np.flatnonzero(mask)
        groups = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        for g in groups:
            i = g[int(np.argmax(metric[g]))]
            m = float(metric[i])
            met = {mname: m, "s_from": float(s[g[0]]), "s_to": float(s[g[-1]]), "lane_type": ltype}
            if extra:
                met.update(extra(i))
            self.emit(typ, sub, self.r, float(s[i]), [lid] if lid is not None else None, sevf(m), met)


# ----------------------------------------------------------------------------- driver
SEV_RANK = {"critical": 3, "major": 2, "minor": 1}


def magnitude(f):
    m = f["metrics"]
    for k, scale in (("jump_m", 1.0), ("kink_deg", 0.1), ("neg_width_m", 2.0), ("k_times_t", 1.0),
                     ("overshoot_m", 1.0), ("excess_curv_1pm", 1.0), ("rate_over_limit", 0.25), ("dist_m", 0.5), ("gap_m", 1.0)):
        if k in m and m[k] is not None:
            return abs(m[k]) * scale
    return 0.0


def merge_intervals(findings):
    """one finding per (type, subtype, road) and overlapping s-interval: several lanes of the same
    place are one defect; keep the worst instance's location and metrics, collect the lanes."""
    out, groups = [], collections.defaultdict(list)
    for f in findings:
        if "s_from" in f["metrics"]:
            groups[(f["type"], f["subtype"], f["road"])].append(f)
        else:
            out.append(f)
    for fs in groups.values():
        fs.sort(key=lambda f: f["metrics"]["s_from"])
        cur = []
        for f in fs + [None]:
            if cur and (f is None or f["metrics"]["s_from"] > max(c["metrics"]["s_to"] for c in cur) + 1.0):
                w = max(cur, key=lambda c: (SEV_RANK[c["severity"]], magnitude(c)))
                w = dict(w, metrics=dict(w["metrics"]))
                w["lanes"] = sorted({l for c in cur for l in (c["lanes"] or [])}) or None
                w["metrics"]["s_from"] = min(c["metrics"]["s_from"] for c in cur)
                w["metrics"]["s_to"] = max(c["metrics"]["s_to"] for c in cur)
                w["metrics"]["lane_types"] = sorted({c["metrics"].get("lane_type") for c in cur})
                out.append(w)
                cur = []
            if f is not None:
                cur.append(f)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr")
    ap.add_argument("osm")
    ap.add_argument("--out", required=True)
    ap.add_argument("--basemap")
    ap.add_argument("--crops", type=int, default=8, help="worst instances per finding type to render")
    ap.add_argument("--no-crops", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    od = G.OpenDrive(a.xodr)
    findings = []

    def emit(typ, sub, road, s, lanes, sev, metrics, t=None):
        if t is None and lanes:
            sec = road.section_at(s)
            b = road.borders_at(min(max(s, 0.0), road.length), sec)
            t = b.get(lanes[0], (0, 0))[1] if lanes[0] in b else 0.0
        x, y = road.point(np.array([min(max(s, 0.0), road.length)]), np.array([t or 0.0]))
        findings.append({"type": typ, "subtype": sub, "road": road.id, "junction": road.junction,
                         "osm_ways": road.osm_ways[:6], "s": round(float(s), 3), "x": round(float(x[0]), 2),
                         "y": round(float(y[0]), 2), "lanes": lanes, "severity": sev,
                         "metrics": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in metrics.items()}})

    for r in od.roads.values():
        RoadCheck(r, emit).run()
    findings[:] = merge_intervals(findings)
    findings.sort(key=lambda f: (-SEV_RANK[f["severity"]], -magnitude(f)))
    for i, f in enumerate(findings):
        f["id"] = f"ES{i + 1:05d}"
    json.dump(findings, open(os.path.join(a.out, "findings.json"), "w"), indent=1)

    # summary
    L = []
    L.append(f"edge_smoothness on {a.xodr}")
    L.append(f"roads: {len(od.roads)} (plain {len(od.plain_roads())}, connecting {len(od.connecting_roads())})")
    L.append(f"findings: {len(findings)}")
    by = collections.Counter((f["type"], f["subtype"] or "-", "conn" if f["junction"] != "-1" else "plain", f["severity"]) for f in findings)
    L.append("\ncount by type / subtype / road kind / severity:")
    for k, v in sorted(by.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2], -SEV_RANK[kv[0][3]])):
        L.append(f"  {v:6d}  {k[0]:24s} {k[1]:18s} {k[2]:6s} {k[3]}")
    L.append("\nroads affected per type:")
    for typ in sorted({f["type"] for f in findings}):
        rs = {f["road"] for f in findings if f["type"] == typ}
        L.append(f"  {typ:24s} {len(rs)} roads")
    L.append("\nmetric distributions (quantiles 50/90/99/max):")
    for typ, key in (("border_jump", "jump_m"), ("border_kink", "kink_deg"), ("steep_transition", "rate_over_limit"),
                     ("width_wobble", "overshoot_m"), ("negative_width", "neg_width_m"), ("fold", "k_times_t"),
                     ("ref_outside_carriageway", "dist_m"), ("border_sharp_bend", "excess_curv_1pm"), ("ref_discontinuity", "kink_deg")):
        v = np.array([abs(f["metrics"][key]) for f in findings if f["type"] == typ and f["metrics"].get(key) is not None])
        if len(v):
            q = np.quantile(v, [0.5, 0.9, 0.99, 1.0])
            L.append(f"  {typ:24s} {key:16s} n={len(v):5d}  " + " / ".join(f"{x:.3f}" for x in q))
    L.append("\nworst 40:")
    for f in findings[:40]:
        L.append(f"  {f['id']} {f['severity']:8s} {f['type']}/{f['subtype']} road {f['road']} s={f['s']} "
                 f"({f['x']},{f['y']}) lanes={f['lanes']} {json.dumps({k: v for k, v in f['metrics'].items() if not isinstance(v, dict)})}")
    open(os.path.join(a.out, "summary.txt"), "w").write("\n".join(L) + "\n")
    print("\n".join(L[:60]))

    if a.no_crops:
        return
    osm = G.Osm(a.osm, od.lat0, od.lon0)
    cdir = os.path.join(a.out, "crops")
    os.makedirs(cdir, exist_ok=True)
    picked = []
    for typ in sorted({f["type"] for f in findings}):
        fs = [f for f in findings if f["type"] == typ]
        picked += fs[:a.crops]
        if len(fs) > a.crops:  # a couple of typical (median) ones
            picked += [fs[len(fs) // 2], fs[(3 * len(fs)) // 4]]
    for f in picked:
        half = 25 if f["type"] in ("border_jump", "border_kink", "width_wobble", "fold", "negative_width") else 45
        G.crop(od, osm, (f["x"], f["y"]), half, os.path.join(cdir, f"{f['id']}_{f['type']}.png"), basemap=a.basemap,
               marks=[(f["x"], f["y"], f["id"])],
               title=f"{f['id']} {f['type']}/{f['subtype']} road {f['road']} s={f['s']} {f['severity']}")


if __name__ == "__main__":
    main()
