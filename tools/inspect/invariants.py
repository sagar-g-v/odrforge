"""invariants -- exact pass/fail definition of a correct OdrForge OpenDRIVE network.

Every check below is a mathematical statement about the OpenDRIVE description itself (plus, where
named, its OSM source) that a correct converter satisfies with ZERO instances on ANY input, in any
country, right- or left-hand traffic. Nothing is a heuristic score: an instance is a measured value
that violates a stated limit, and every limit is numeric precision or a cited design rule. Rule ids
and tolerances follow docs/SPEC_GEOMETRY.md (section 0.2 table; each invariant carries its "spec" ids).

Geometry is evaluated exactly, not from samples:
  line / arc       closed form
  spiral           heading and curvature closed form (k = k0 + c ds), position by composite
                   10-point Gauss-Legendre quadrature on <= 1 m panels (error < 1e-12 m)
  paramPoly3       closed-form derivatives, s -> p by arc-length inversion (Newton on the
                   Gauss-Legendre arc length; endpoints are exact for both pRange conventions)
  poly3            same machinery with u = p (deprecated element, still accepted)
  lane borders     t(s) = laneOffset(s) + sum of lane width cubics; t', t'' analytic, one-sided limits at
                   every record start; border heading psi = hdg + atan2(t', 1 - k t);
                   border curvature  kappa = (k (a^2+b^2) + a b' - b a') / (a^2+b^2)^1.5,
                   a = 1 - k t, b = t', a' = -(k' t + k t'), b' = t''   (offset-curve formula)

Invariants (JSON keys; [spec rule]):
  G0_border          lane-border gap at every lane link [m]: lane-section boundary, road-road link
                     (lane link + contactPoint) and junction incoming -> connecting (laneLink); both edges
                     of every linked lane pair, sides matched by traversal direction. [XS-5, NAT-4]
  G0_ref             reference-line gap at element joins [m]. [GEO-3]
  G0_open_lane       a lane with width > eps_w at an in-road section boundary that has no successor (or
                     predecessor): the carriageway edge steps there. [FLOW-1/abrupt]
  G1_ref, G1_border  heading step of the reference line (element joins, touching road links) and of every
                     linked lane border [rad]; folded edges are left to "fold". [GEO-3, XS-5]
  G2_ref, G2_lane    curvature step |dk| [1/m] > eps_kappa of the reference line (element joins, touching
                     road links) and of every linked drivable lane-centre path (the vehicle path), plus
                     (family "record") the lane-centre path at laneOffset / width record joins inside a
                     lane (a cubic-Hermite blend steps t''). Lanes inside their own taper are exempt.
                     Context per instance: v (posted speed, 50 km/h untagged), lat_accel_step = v^2 |dk|,
                     aashto = lat_accel_step >= 1.3 m/s^2 (Green Book 3.3.8: a spiral would be required).
                     Plain roads only. [GEO-3, GEO-4]
  G2_*_junction      the same where a junction connecting road takes part. [GEO-4, JCT-4]
  fold               k(s) t(s) >= 1 for a lane border or laneOffset t: the offset curve cusps or runs
                     backwards. Exact per polynomial piece (k linear, t cubic -> quartic maximum).
                     [GEO-5]
  width_negative     exact cubic minimum of a width record < -eps_w. [XS-1]
  width_pinch        a drivable lane <= eps_w anywhere but the free end of a taper (interior, zero_tail,
                     zero_head, linked_zero, zero_lane). [XS-1, FLOW-6/zero]
  laneOffset_jump, width_record_jump   value (eps_pos) or slope (eps_hdg) step between records. [XS-3/5]
  topology           broken links (dangling, non-reciprocal, laneLink vs lane link disagreement, a
                     connecting lane link without its junction laneLink = laneLink_missing and the
                     converse connecting_lane_link_missing, road ends used by a junction but not linked to
                     it -- one instance per road end), empty junctions, duplicate connections, connecting
                     roads used as incoming roads, plan-view bookkeeping and element validity (GEO-2:
                     element >= 1 mm, arc k != 0, spiral k varies, line/arc/spiral only, hdg in (-pi, pi],
                     s ascending, sum of lengths = road length within eps_s). [FLOW-2, JCT-6, GEO-2]
  osm_ref_far        road -> OSM directed Hausdorff: a reference-line point farther from its osmWays than
                     the farthest drivable border at that s. kind userData_incomplete / geometry_off. [GEO-1]
  osm_off_road       OSM -> road directed Hausdorff: a point of the OSM way portion the road covers that
                     lies outside the drivable band [t_min, t_max]. [GEO-1]
  lane_count         drivable full-width lanes per travel direction vs lanes, lanes:forward/backward/
                     both_ways, oneway (incl. implied oneway of motorway / roundabout), every 2 m;
                     expected = sum over the road's ways crossing the cross-section inside the drivable band;
                     tag in [full, full + tapering]; cross-sections containing a way-end node are skipped.
                     [TOP-2, TOP-3]
  continuity_nondriving  a shoulder / curb / sidewalk / border / biking / walking / parking lane present
                     on the same side of both sides of a section boundary or plain road-road link and not
                     linked through. [NAT-4]
  continuity_junction_corner (SHOULD)  two angularly consecutive junction arms both carry an edge lane of
                     type T on their facing sides and no connecting road between them carries T (U-turn
                     noses > 150 deg excluded). The user's no-break rule; NAT-4 lets edge lanes end at
                     junctions, so this is reported apart.
  needless_split     kind road: two plain roads linked road-to-road with identical lane layout and no OSM
                     branching node; kind section: a removable lane-section boundary (same lane ids and
                     types, every lane linked 1:1 to its own id), with what differs (identical /
                     width-poly / width-jump); kind junction_not_only_two: a junction joining <= 2 road
                     ends, osm_branch = a branching OSM node in its footprint if any. [NAT-1/2/3, JCT-6]
  shape_unchanged    heading step at a plain road-road link (vacuous while no such links exist). [NAT-5]
  flow_*             the natural-flow game on the lane graph (spec FLOW-0): nodes = (road, section, lane,
                     travel sign from the id sign and road@rule); edges = in-road and road-road lane links,
                     junction laneLinks (the ONLY entry into a connecting road), and lane changes between
                     adjacent same-direction drivable lanes allowed by the separating roadMark@laneChange.
     flow_direction    a link joining opposite travel directions (head-on). [FLOW-2/wrong-way]
     flow_dead_end / flow_dead_start   a lane end with nothing after / before it that is not a legal sink /
                       source; code exit-/entry-unlinked (unlinked road end, one instance per road end, lanes
                       listed), exit-/entry-no-lane-link (road linked, lane not; entry is SHOULD),
                       abrupt-end/-start (inside a road). Legal: an UNLINKED road end outside the clip box
                       or within eps_B of it, or at an OSM dead end (degree-1 node of the road's own ways).
                       A zero-width (taper) end is neither a source nor a sink. [FLOW-1]
     flow_boundary_short (SHOULD) an unlinked boundary end more than eps_pos inside the box.
     flow_orphan / flow_trap   SCCs of the lane graph with no incoming / outgoing condensation edge and no
                       legal source / sink: one instance per root cause, value = lanes in the SCC. [FLOW-4]
     flow_closed_not_strong    a component with neither sources nor sinks that is not one SCC with a cycle.
     flow_narrow       a drivable lane chain whose exact maximum width is < W_P. [FLOW-6/narrow]
     flow_no_drivable  a road with no drivable lane on a motor-traffic way. [FLOW-6]
     meta.flow         lane nodes, edges, splits, merges, unreachable / trapped / unusable lanes.
  degenerate         road or lane section shorter than 1.0 m. [SIM-5]

Tolerances (docs/SPEC_GEOMETRY.md 0.2, not tuned to an area):
  eps_pos 1e-3 m, eps_w 1e-3 m   numeric precision (>= 12 significant digits printed, exact evaluator)
  eps_hdg 1e-4 rad               eps_pos over 10 m of road
  eps_kappa 1e-5 1/m             v^2 eps_kappa = 0.009 m/s^2 at 30 m/s
  eps_s 1e-2 m                   plan-view s bookkeeping
  eps_B 2.0 m                    map-boundary allowance for unlinked road ends (spec 0.2 measurement)
  W_P 2.13 m                     AASHTO P design vehicle width [Green Book Table 2-1a]
  1.0 m                          SIM-5 minimum road / section length
  fold                           exact geometric condition, no tolerance

Limits: OpenDRIVE before 1.5 cannot carry road@rule: a left-hand traffic network written as 1.4 is RHT
by the standard (--rule file); --rule lht shows the intended semantics. paramPoly3 / poly3 pieces fall
back to 0.25 m sampling in the fold check.

    python invariants.py OUT.xodr AREA.osm --out DIR [--clip lat0,lon0,lat1,lon1] [--rule file|rht|lht]
    python invariants.py --area london --out DIR                 (paths/clip from gate.py)
    python invariants.py --area la --out DIR --crops 3           (+ G.crop evidence images)
    python invariants_selftest.py --area munich --out DIR        (planted-defect self test)

    import invariants as I
    net = I.Network("out.xodr"); res = I.run_all(net, osm, clip_xy, rule)

Output: DIR/invariants.json = {"meta": {...}, "summary": {invariant: count},
        "invariants": {invariant: {"count", "limit", "unit", "level", "spec", "instances": [
            {"road", "lane", "s", "x", "y", "value", "limit", ...}]}}} -- sorted, deterministic.
"""
import argparse
import collections
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Tolerances: docs/SPEC_GEOMETRY.md section 0.2 (one table for every checker; justification there)
TOL_POS = 1e-3      # m       eps_pos
TOL_HDG = 1e-4      # rad     eps_hdg  (= eps_pos over 10 m of road)
TOL_K = 1e-5        # 1/m     eps_kappa (G2): v^2 * eps_kappa = 0.009 m/s^2 at 30 m/s
LAT_JERK = 1.3      # m/s^2   AASHTO Green Book 3.3.8 spiral-need threshold: triage context only
V_DEFAULT = 50 / 3.6
TOL_W = 1e-3        # m       eps_w, a width <= this is "zero"
EPS_B = 2.0         # m       eps_B, map-boundary allowance for UNLINKED road ends
W_P = 2.13          # m       AASHTO P design-vehicle width [A1 Table 2-1a] (FLOW-6 narrow)
MIN_LEN = 1.0       # m       SIM-5 minimum road / lane-section length (esmini, CARLA)
FOLD_STEP = 0.25    # m       sampling fallback for paramPoly3 / poly3 only

DRIVABLE = {"driving", "bidirectional", "bus", "taxi", "HOV", "entry", "exit", "onRamp", "offRamp",
            "connectingRamp", "slipLane", "mwyEntry", "mwyExit"}
EDGE_TYPES = {"shoulder", "curb", "sidewalk", "border", "biking", "walking", "parking"}  # NAT-4
MOTOR_HIGHWAYS = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
                  "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
                  "living_street", "service", "road", "busway"}

GL_X, GL_W = np.polynomial.legendre.leggauss(10)


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def _f(e, k, d=0.0):
    v = e.get(k)
    return float(v) if v is not None else d


# ============================================================================ exact planView
class Geom:
    """One planView geometry element with exact evaluation: eval(ds) -> x, y, hdg, k, dk/ds."""

    def __init__(self, el):
        self.s0, self.L = _f(el, "s"), _f(el, "length")
        self.x0, self.y0, self.h0 = _f(el, "x"), _f(el, "y"), _f(el, "hdg")
        g = el[0]
        self.kind = g.tag
        self.c0, self.s0n = math.cos(self.h0), math.sin(self.h0)
        if self.kind == "arc":
            self.k = _f(g, "curvature")
        elif self.kind == "spiral":
            self.k0, self.k1 = _f(g, "curvStart"), _f(g, "curvEnd")
            self.cc = (self.k1 - self.k0) / self.L if self.L > 0 else 0.0
            n = max(1, int(math.ceil(self.L / 1.0)))
            self.hseg = self.L / n if self.L > 0 else 1.0
            u = np.arange(n + 1) * self.hseg
            cx, cy = self._spiral_int(u[:-1], u[1:])
            self.knot_x = np.concatenate([[0.0], np.cumsum(cx)])
            self.knot_y = np.concatenate([[0.0], np.cumsum(cy)])
            self.nseg = n
        elif self.kind in ("paramPoly3", "poly3"):
            if self.kind == "paramPoly3":
                self.U = [_f(g, k) for k in ("aU", "bU", "cU", "dU")]
                self.V = [_f(g, k) for k in ("aV", "bV", "cV", "dV")]
                self.pmax = 1.0 if g.get("pRange", "normalized") == "normalized" else self.L
            else:
                a, b, c, d = (_f(g, k) for k in ("a", "b", "c", "d"))
                self.U, self.V = [0.0, 1.0, 0.0, 0.0], [a, b, c, d]
                self.pmax = None
            self._pp_setup()
        elif self.kind != "line":
            raise ValueError("unsupported geometry " + self.kind)

    # -- spiral
    def _theta(self, u):
        return self.k0 * u + 0.5 * self.cc * u * u

    def _spiral_int(self, a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        mid, half = (a + b) / 2, (b - a) / 2
        u = mid[:, None] + half[:, None] * GL_X[None, :]
        th = self._theta(u)
        return (np.cos(th) * GL_W).sum(1) * half, (np.sin(th) * GL_W).sum(1) * half

    # -- parametric cubic
    @staticmethod
    def _pv(c, p, d=0):
        a, b, cc, dd = c
        if d == 0:
            return a + b * p + cc * p * p + dd * p ** 3
        if d == 1:
            return b + 2 * cc * p + 3 * dd * p * p
        if d == 2:
            return 2 * cc + 6 * dd * p
        return 6 * dd + 0 * p

    def _speed(self, p):
        return np.hypot(self._pv(self.U, p, 1), self._pv(self.V, p, 1))

    def _arclen(self, a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        mid, half = (a + b) / 2, (b - a) / 2
        p = mid[:, None] + half[:, None] * GL_X[None, :]
        return (self._speed(p) * GL_W).sum(1) * half

    def _pp_setup(self):
        if self.pmax is None:  # poly3: find u_end with arc length L
            pe = self.L
            for _ in range(30):
                n = 64
                kn = np.linspace(0, pe, n + 1)
                S = self._arclen(kn[:-1], kn[1:]).sum()
                pe -= (S - self.L) / max(float(self._speed(np.array([pe]))[0]), 1e-9)
            self.pmax = pe
        n = 128
        self.pk = np.linspace(0, self.pmax, n + 1)
        self.Sk = np.concatenate([[0.0], np.cumsum(self._arclen(self.pk[:-1], self.pk[1:]))])
        self.Stot = self.Sk[-1]

    def _p_of(self, ds):
        target = np.clip(ds, 0, self.L) * (self.Stot / self.L if self.L > 0 else 1.0)
        p = np.interp(target, self.Sk, self.pk)
        for _ in range(4):
            j = np.clip(np.searchsorted(self.pk, p, "right") - 1, 0, len(self.pk) - 2)
            S = self.Sk[j] + self._arclen(self.pk[j], p)
            p = p - (S - target) / np.maximum(self._speed(p), 1e-12)
        return p

    # -- evaluation
    def eval(self, ds):
        ds = np.atleast_1d(np.asarray(ds, float))
        z = np.zeros_like(ds)
        if self.kind == "line":
            u, v, h, k, dk = ds, z, z, z, z
        elif self.kind == "arc":
            k = self.k + z
            if abs(self.k) < 1e-15:
                u, v = ds, z
            else:
                u, v = np.sin(self.k * ds) / self.k, (1 - np.cos(self.k * ds)) / self.k
            h, dk = self.k * ds, z
        elif self.kind == "spiral":
            j = np.clip(np.floor(ds / self.hseg).astype(int), 0, self.nseg - 1)
            a = j * self.hseg
            cx, cy = self._spiral_int(a, ds)
            u, v = self.knot_x[j] + cx, self.knot_y[j] + cy
            h, k, dk = self._theta(ds), self.k0 + self.cc * ds, self.cc + z
        else:
            p = self._p_of(ds)
            u, v = self._pv(self.U, p), self._pv(self.V, p)  # local frame incl. aU / aV
            u1, v1 = self._pv(self.U, p, 1), self._pv(self.V, p, 1)
            u2, v2 = self._pv(self.U, p, 2), self._pv(self.V, p, 2)
            u3, v3 = self._pv(self.U, p, 3), self._pv(self.V, p, 3)
            sp2 = u1 * u1 + v1 * v1
            sp = np.sqrt(sp2)
            h = np.arctan2(v1, u1)
            cr = u1 * v2 - v1 * u2
            k = cr / np.maximum(sp2 * sp, 1e-300)
            dkdp = ((u1 * v3 - v1 * u3) * sp2 - 3 * cr * (u1 * u2 + v1 * v2)) / np.maximum(sp2 ** 2.5, 1e-300)
            dk = dkdp / np.maximum(sp, 1e-300)
        x = self.x0 + u * self.c0 - v * self.s0n
        y = self.y0 + u * self.s0n + v * self.c0
        return x, y, self.h0 + h, k, dk


# ============================================================================ lanes
class Poly:
    """Piecewise cubic records [(start, a, b, c, d)] -> value and first two derivatives."""

    def __init__(self, recs):
        self.recs = sorted(recs) or [(0.0, 0.0, 0.0, 0.0, 0.0)]
        self.st = np.array([r[0] for r in self.recs])
        self.C = np.array([r[1:] for r in self.recs])

    def idx(self, x, side="right"):
        x = np.atleast_1d(np.asarray(x, float))
        if side == "right":
            j = np.searchsorted(self.st, x + 1e-12, "right") - 1
        else:  # limit from the left: a record starting exactly at x does not apply
            j = np.searchsorted(self.st, x - 1e-12, "left") - 1
        return np.clip(j, 0, len(self.st) - 1)

    def ev(self, x, side="right", j=None):
        x = np.atleast_1d(np.asarray(x, float))
        j = self.idx(x, side) if j is None else np.full(x.shape, j)
        d = x - self.st[j]
        a, b, c, e = self.C[j].T
        return (a + b * d + c * d * d + e * d ** 3, b + 2 * c * d + 3 * e * d * d, 2 * c + 6 * e * d)


class Lane:
    def __init__(self, el):
        self.id = int(el.get("id"))
        self.type = el.get("type", "none")
        self.direction = el.get("direction", "standard")
        self.width = Poly([(_f(w, "sOffset"), _f(w, "a"), _f(w, "b"), _f(w, "c"), _f(w, "d")) for w in el.findall("width")])
        self.has_border = el.find("border") is not None
        lk = el.find("link")
        p = lk.find("predecessor") if lk is not None else None
        q = lk.find("successor") if lk is not None else None
        self.pred = int(p.get("id")) if p is not None else None
        self.succ = int(q.get("id")) if q is not None else None
        # roadMark@laneChange of the lane's outer border (FLOW-0 lateral edges); missing = both
        self.lane_change = {m.get("laneChange", "both") for m in el.findall("roadMark")} or {"both"}
        # spec 0.1: a restricted lane carrying an <access rule="allow"> is drivable
        self.access_allow = any(a.get("rule") == "allow" for a in el.findall("access"))

    def allows(self, toward_higher_id):
        """does this lane's outer road mark allow crossing toward the higher (or lower) lane id?"""
        return "both" in self.lane_change or ("increase" if toward_higher_id else "decrease") in self.lane_change

    @property
    def drivable(self):
        return self.type in DRIVABLE or (self.type == "restricted" and self.access_allow)


class Section:
    def __init__(self, el):
        self.s = _f(el, "s")
        self.e = None
        self.lanes = {}
        for side in ("left", "right"):
            se = el.find(side)
            if se is not None:
                for l in se.findall("lane"):
                    ln = Lane(l)
                    self.lanes[ln.id] = ln
        c = el.find("center/lane")
        self.center = Lane(c) if c is not None else None   # its roadMark separates lanes 1 and -1

    def side(self, sign):
        return sorted((l for l in self.lanes.values() if (l.id > 0) == (sign > 0)), key=lambda l: abs(l.id))


class LinkRef:
    def __init__(self, el):
        self.type = el.get("elementType")
        self.id = el.get("elementId")
        self.contact = el.get("contactPoint")


def _speed_ms(el):
    if el is None:
        return None
    v = el.get("max")
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    u = el.get("unit", "m/s")
    return v / 3.6 if u == "km/h" else v * 0.44704 if u == "mph" else v


class Road:
    def __init__(self, el):
        self.id = el.get("id")
        self.length = _f(el, "length")
        self.junction = el.get("junction", "-1")
        self.rule = el.get("rule")
        lk = el.find("link")
        p = lk.find("predecessor") if lk is not None else None
        q = lk.find("successor") if lk is not None else None
        self.pred = LinkRef(p) if p is not None else None
        self.succ = LinkRef(q) if q is not None else None
        ud = [u.get("value") for u in el.iter("userData") if u.get("code") == "osmWays"]
        self.osm_ways = [w for w in ud[0].split(",") if w] if ud else []
        sp = [_speed_ms(t.find("speed")) for t in el.findall("type")]
        sp = [v for v in sp if v]
        self.speed = min(sp) if sp else V_DEFAULT
        self.raw_geoms = [(_f(g, "s"), _f(g, "length"), _f(g, "hdg"), g[0].tag if len(g) else "none", dict(g[0].attrib)
                           if len(g) else {}) for g in el.find("planView").findall("geometry")]
        self.geoms = [Geom(g) for g in el.find("planView").findall("geometry")]
        self.geoms = [g for g in self.geoms if g.L > 1e-12] or self.geoms[:1]
        self.gs = np.array([g.s0 for g in self.geoms])
        lanes = el.find("lanes")
        self.offset = Poly([(_f(e, "s"), _f(e, "a"), _f(e, "b"), _f(e, "c"), _f(e, "d")) for e in lanes.findall("laneOffset")])
        self.sections = sorted((Section(s) for s in lanes.findall("laneSection")), key=lambda s: s.s)
        for i, sec in enumerate(self.sections):
            sec.e = self.sections[i + 1].s if i + 1 < len(self.sections) else self.length

    @property
    def plain(self):
        return self.junction == "-1"

    def pose(self, s, side="right"):
        """exact reference-line pose at s: x, y, hdg, k, dk (arrays). side='left' takes the element
        ending at s when s is an element boundary."""
        s = np.atleast_1d(np.asarray(s, float))
        if side == "right":
            j = np.searchsorted(self.gs, s + 1e-12, "right") - 1
        else:
            j = np.searchsorted(self.gs, s - 1e-12, "left") - 1
        j = np.clip(j, 0, len(self.geoms) - 1)
        out = [np.zeros_like(s) for _ in range(5)]
        for gi in np.unique(j):
            m = j == gi
            g = self.geoms[gi]
            for o, v in zip(out, g.eval(s[m] - g.s0)):
                o[m] = v
        return out

    def lane_dir(self, lane, rule):
        """travel direction of a lane relative to +s: [+1], [-1] or [+1, -1] (both)."""
        if lane.type == "bidirectional" or lane.direction == "both":
            return [1, -1]
        lht = (rule == "lht") or (rule == "file" and (self.rule or "").upper() == "LHT")
        d = (1 if lane.id < 0 else -1) * (-1 if lht else 1)
        return [-d] if lane.direction == "reversed" else [d]

    def borders(self, k, s, side="right"):
        """{lane id: (t_in, t_out, dt_in, dt_out, ddt_in, ddt_out)} over s (arrays) in section k."""
        sec = self.sections[k]
        s = np.atleast_1d(np.asarray(s, float))
        o, o1, o2 = self.offset.ev(s, side)
        out = {0: (o, o, o1, o1, o2, o2)}
        ds = s - sec.s
        for sign in (1, -1):
            t, t1, t2 = o, o1, o2
            for l in sec.side(sign):
                w, w1, w2 = l.width.ev(ds, side)  # side: one-sided limit at a width-record start too
                nt, nt1, nt2 = t + sign * w, t1 + sign * w1, t2 + sign * w2
                out[l.id] = (t, nt, t1, nt1, t2, nt2)
                t, t1, t2 = nt, nt1, nt2
        return out

    def in_taper(self, k, lane_id, s):
        """is lane lane_id of section k inside its own taper (opening from / closing to zero) at s?
        GEO-4 exempts the centre path there (no vehicle follows it)."""
        if not hasattr(self, "_tap"):
            self._tap = lane_tapers(self)
        return any(a - 1e-9 <= s <= b + 1e-9 for a, b in self._tap.get((k, lane_id), []))

    def section_index(self, s, side="right"):
        st = np.array([x.s for x in self.sections])
        if side == "right":
            j = np.searchsorted(st, s + 1e-12, "right") - 1
        else:
            j = np.searchsorted(st, s - 1e-12, "left") - 1
        return int(np.clip(j, 0, len(st) - 1))


def border_pose(P, t, t1, t2):
    """pose of an offset curve at lateral t(s): X, Y, psi, kappa (all along +s)."""
    x, y, h, k, dk = P
    a, b = 1 - k * t, t1
    a1, b1 = -(dk * t + k * t1), t2
    n2 = a * a + b * b
    psi = h + np.arctan2(b, a)
    kap = (k * n2 + a * b1 - b * a1) / np.maximum(n2 ** 1.5, 1e-300)
    return x - t * np.sin(h), y + t * np.cos(h), psi, kap


class Network:
    def __init__(self, path):
        root = ET.parse(str(path)).getroot()
        self.path = str(path)
        hdr = root.find("header")
        self.rev = (int(hdr.get("revMajor", 1)), int(hdr.get("revMinor", 0)))
        geo = hdr.findtext("geoReference") or ""
        import re
        m0, m1 = re.search(r"\+lat_0=([-0-9.eE]+)", geo), re.search(r"\+lon_0=([-0-9.eE]+)", geo)
        self.lat0 = float(m0.group(1)) if m0 else 0.0
        self.lon0 = float(m1.group(1)) if m1 else 0.0
        self.roads = {}
        for el in root.findall("road"):
            r = Road(el)
            self.roads[r.id] = r
        self.junctions = {}
        self.cross_paths = {}
        for j in root.findall("junction"):
            conns = []
            direct = j.get("type") == "direct"
            for c in j.findall("connection"):
                # a direct junction (OpenDRIVE 1.9 section 12.6) links the incoming road to @linkedRoad itself
                conns.append(dict(id=c.get("id"), incoming=c.get("incomingRoad"),
                                  connecting=c.get("linkedRoad") if direct else c.get("connectingRoad"),
                                  contact=c.get("contactPoint"), direct=direct,
                                  lanes=[(int(l.get("from")), int(l.get("to"))) for l in c.findall("laneLink")]))
            self.junctions[j.get("id")] = conns
            # OpenDRIVE 1.9 section 12.5 cross paths (1.8): crossing road, the roads at its ends, lane links with s
            cps = []
            for cp in j.findall("crossPath"):
                st, en = cp.find("startLaneLink"), cp.find("endLaneLink")
                cps.append(dict(id=cp.get("id"), crossing=cp.get("crossingRoad"), at_start=cp.get("roadAtStart"),
                                at_end=cp.get("roadAtEnd"),
                                links=[(int(x.get("from")), float(x.get("s")), int(x.get("to"))) for x in (st, en) if x is not None]))
            self.cross_paths[j.get("id")] = cps

    def plain_roads(self):
        return [r for r in self.roads.values() if r.plain]


# ============================================================================ instances
def _r(v, n=6):
    if isinstance(v, (float, np.floating)):
        return round(float(v), n)
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


def inst(road, lane, s, x, y, value, limit, **extra):
    d = dict(road=str(road), lane=_r(lane), s=_r(s, 3), x=_r(x, 3), y=_r(y, 3), value=_r(value), limit=_r(limit))
    d.update({k: _r(v) for k, v in extra.items()})
    return d


class Result:
    """invariant name -> list of instances, with unit / limit metadata."""

    # name: (unit, limit, level, SPEC_GEOMETRY.md rule)
    META = {
        "G0_border": ("m", TOL_POS, "MUST", "XS-5, NAT-4, GEO-4"), "G0_ref": ("m", TOL_POS, "MUST", "GEO-3"),
        "G0_open_lane": ("m", TOL_W, "MUST", "FLOW-1/abrupt, XS-1"),
        "G1_ref": ("rad", TOL_HDG, "MUST", "GEO-3"), "G1_border": ("rad", TOL_HDG, "MUST", "XS-5, GEO-4"),
        "G2_ref": ("1/m", TOL_K, "MUST", "GEO-3"), "G2_lane": ("1/m", TOL_K, "MUST", "GEO-4"),
        "G2_ref_junction": ("1/m", TOL_K, "MUST", "GEO-3, JCT-4"),
        "G2_lane_junction": ("1/m", TOL_K, "MUST", "GEO-4, JCT-4"),
        "fold": ("k*t", 1.0, "MUST", "GEO-5"), "width_negative": ("m", -TOL_W, "MUST", "XS-1"),
        "width_pinch": ("m", TOL_W, "MUST", "XS-1, FLOW-6/zero"),
        "laneOffset_jump": ("m|rad", TOL_POS, "MUST", "XS-3"), "width_record_jump": ("m|rad", TOL_POS, "MUST", "XS-5"),
        "topology": ("-", 0, "MUST", "FLOW-2, JCT-6, GEO-2"), "osm_ref_far": ("m", 0.0, "MUST", "GEO-1"),
        "osm_off_road": ("m", 0.0, "MUST", "GEO-1"), "lane_count": ("lanes", 0, "MUST", "TOP-2, TOP-3"),
        "continuity_nondriving": ("-", 0, "MUST", "NAT-4"),
        "continuity_junction_corner": ("-", 0, "SHOULD", "user no-break rule; NAT-4 lets edges end at junctions"),
        "needless_split": ("-", 0, "MUST", "NAT-1, NAT-2, NAT-3, JCT-6/not-only-two"),
        "shape_unchanged": ("rad", TOL_HDG, "MUST", "NAT-5"),
        "flow_direction": ("-", 0, "MUST", "FLOW-2/wrong-way"),
        "flow_dead_end": ("m", TOL_W, "MUST", "FLOW-1/exit-*"),
        "flow_dead_start": ("m", TOL_W, "MUST|SHOULD", "FLOW-1/entry-*"),
        "flow_boundary_short": ("m", TOL_POS, "SHOULD", "FLOW-1/boundary-short"),
        "flow_orphan": ("lanes", 0, "MUST", "FLOW-4/orphan"), "flow_trap": ("lanes", 0, "MUST", "FLOW-4/trap"),
        "flow_closed_not_strong": ("lanes", 0, "MUST", "FLOW-4/closed-not-strong"),
        "flow_narrow": ("m", W_P, "MUST", "FLOW-6/narrow, XS-2"),
        "flow_no_drivable": ("-", 0, "MUST", "FLOW-6"),
        "degenerate": ("m", MIN_LEN, "MUST", "SIM-5"),
    }

    def __init__(self):
        self.items = collections.defaultdict(list)
        self.info = {}

    def add(self, name, i):
        self.items[name].append(i)

    def add_g2(self, name, net_or_roads, i):
        """G2 instances go to <name>_junction when either road of the link is a junction connecting road.
        value = |dk| [1/m] against eps_kappa; lat_accel_step = v^2 |dk| and aashto (>= 1.3 m/s^2, a
        spiral would be required by the Green Book) are triage context, not the limit."""
        ids = [i["road"], i.get("to_road", i["road"])]
        if any(net_or_roads[x].junction != "-1" for x in ids if x in net_or_roads):
            name += "_junction"
        self.items[name].append(i)

    def to_json(self):
        out = {}
        for name in self.META:
            lst = self.items.get(name, [])
            lst = sorted(lst, key=lambda d: (-abs(d["value"]) if isinstance(d["value"], (int, float)) else 0,
                                             _idkey(d["road"]), str(d["lane"]), d["s"]))
            unit, lim, level, rule = self.META[name]
            out[name] = {"count": len(lst), "unit": unit, "limit": lim, "level": level, "spec": rule,
                         "instances": lst}
        return out


def g2_inst(road, lane, s, x, y, dk, v, **kw):
    """G2 instance: value = |dk| (limit eps_kappa); v^2|dk| and the AASHTO flag as context."""
    j = v * v * dk
    return inst(road, lane, s, x, y, dk, TOL_K, v=v, lat_accel_step=j, aashto=bool(j >= LAT_JERK), **kw)


# ============================================================================ link geometry
def end_s(road, end):
    return 0.0 if end == "start" else road.length


def end_k(road, end):
    return 0 if end == "start" else len(road.sections) - 1


def _edges(lane_id, b):
    """(left edge, right edge) along +s of a lane from its border tuple."""
    t_in, t_out, d_in, d_out, dd_in, dd_out = b
    inner, outer = (t_in, d_in, dd_in), (t_out, d_out, dd_out)
    return (outer, inner) if lane_id > 0 else (inner, outer)


class End:
    """one lane-section end of a road evaluated at one s, seen in a traversal direction d
    (+1: travelling +s, -1: travelling -s): pose, borders."""

    def __init__(self, road, k, s, side, d):
        self.road, self.k, self.s, self.d = road, k, s, d
        self.P = road.pose(np.array([s]), side)
        self.B = road.borders(k, np.array([s]), side)

    def ref(self):
        x, y, h, k, dk = (float(v[0]) for v in self.P)
        return x, y, h + (0 if self.d == 1 else math.pi), k * self.d

    def edge(self, lane_id, which):
        """traversal-left (which=0) / traversal-right (1) edge of a lane: x, y, psi, kappa (traversal)."""
        e = _edges(lane_id, self.B[lane_id])
        if self.d == -1:
            e = (e[1], e[0])
        t, t1, t2 = e[which]
        x, y, psi, kap = border_pose(self.P, t, t1, t2)
        return float(x[0]), float(y[0]), float(psi[0]) + (0 if self.d == 1 else math.pi), float(kap[0]) * self.d

    def folded(self, lane_id, which):
        """the traversal-left/right edge lies beyond the centre of curvature (1 - k t <= 0)."""
        e = _edges(lane_id, self.B[lane_id])
        if self.d == -1:
            e = (e[1], e[0])
        return float(1 - self.P[3][0] * e[which][0][0]) <= 0

    def centre(self, lane_id):
        t_in, t_out, d_in, d_out, dd_in, dd_out = self.B[lane_id]
        x, y, psi, kap = border_pose(self.P, (t_in + t_out) / 2, (d_in + d_out) / 2, (dd_in + dd_out) / 2)
        return float(x[0]), float(y[0]), float(psi[0]) + (0 if self.d == 1 else math.pi), float(kap[0]) * self.d

    def width(self, lane_id):
        b = self.B[lane_id]
        return float(abs(b[1][0] - b[0][0]))


def link_end(road, end):
    """End of a road used as the 'from' side of a link (traversal leaves the road there)."""
    return End(road, end_k(road, end), end_s(road, end), "left" if end == "end" else "right",
               1 if end == "end" else -1)


def contact_end(road, contact):
    """End of the entered road at its contactPoint (traversal enters the road there)."""
    return End(road, end_k(road, contact), end_s(road, contact), "left" if contact == "end" else "right",
               1 if contact == "start" else -1)


def compare_lanes(res, fam, A, la, B, lb, v):
    """G0 / G1 / G2 between lane la of End A and lane lb of End B (traversal frames)."""
    out = {}
    for which, nm in ((0, "left"), (1, "right")):
        xa, ya, pa, ka = A.edge(la, which)
        xb, yb, pb, kb = B.edge(lb, which)
        g = math.hypot(xa - xb, ya - yb)
        h = abs(float(wrap(pa - pb)))
        out[nm] = (g, h)
        if g > TOL_POS:
            res.add("G0_border", inst(A.road.id, la, A.s, xa, ya, g, TOL_POS, family=fam, edge=nm,
                                      to_road=B.road.id, to_lane=lb))
        if h > TOL_HDG and not (A.folded(la, which) or B.folded(lb, which)):  # folded edges: see "fold"
            res.add("G1_border", inst(A.road.id, la, A.s, xa, ya, h, TOL_HDG, family=fam, edge=nm,
                                      to_road=B.road.id, to_lane=lb))
    la_t = A.road.sections[A.k].lanes.get(la)
    lb_t = B.road.sections[B.k].lanes.get(lb)
    if la_t is not None and lb_t is not None and la_t.drivable and lb_t.drivable \
            and not (A.road.in_taper(A.k, la, A.s) or B.road.in_taper(B.k, lb, B.s)):  # GEO-4 taper exemption
        xa, ya, _, ka = A.centre(la)
        _, _, _, kb = B.centre(lb)
        if abs(ka - kb) > TOL_K:
            res.add_g2("G2_lane", {A.road.id: A.road, B.road.id: B.road},
                       g2_inst(A.road.id, la, A.s, xa, ya, abs(ka - kb), v, family=fam, to_road=B.road.id,
                               to_lane=lb))
    return out


def compare_ref(res, fam, A, B, v):
    """G1/G2 of the reference lines at a road link where they touch (gap <= TOL_POS)."""
    xa, ya, ha, ka = A.ref()
    xb, yb, hb, kb = B.ref()
    g = math.hypot(xa - xb, ya - yb)
    if g > TOL_POS:
        return g, None
    h = abs(float(wrap(ha - hb)))
    if h > TOL_HDG:
        res.add("G1_ref", inst(A.road.id, 0, A.s, xa, ya, h, TOL_HDG, family=fam, to_road=B.road.id))
    if abs(ka - kb) > TOL_K:
        res.add_g2("G2_ref", {A.road.id: A.road, B.road.id: B.road},
                   g2_inst(A.road.id, 0, A.s, xa, ya, abs(ka - kb), v, family=fam, to_road=B.road.id))
    return g, h


# ============================================================================ link enumeration
def junction_end(inc, jid, near_xy, back=None):
    """which end of the incoming road links to junction jid. When both ends do (a loop road), the
    connecting road's own link back to the incoming road (contactPoint) decides; only without it the
    end nearest near_xy is taken."""
    ends = [e for e, lk in (("start", inc.pred), ("end", inc.succ))
            if lk is not None and lk.type == "junction" and lk.id == jid]
    if len(ends) <= 1:
        return ends[0] if ends else None
    if back is not None and back.type == "road" and back.id == inc.id and back.contact in ends:
        return back.contact
    d = []
    for e in ends:
        x, y, *_ = inc.pose(np.array([end_s(inc, e)]))
        d.append(math.hypot(float(x[0]) - near_xy[0], float(y[0]) - near_xy[1]))
    return ends[int(np.argmin(d))]


def _topo(res, road, lane, s, kind, x=None, y=None, **kw):
    """one topology instance; road is a Road (its xy at s is used when x / y are not given) or an id."""
    rid = road.id if isinstance(road, Road) else road
    if x is None or y is None:
        x, y = _xy(road, s) if isinstance(road, Road) else (0.0, 0.0)
    res.add("topology", inst(rid, lane, s, x, y, 1, 0, kind=kind, **kw))


def _idkey(v):
    v = str(v)
    return (0, int(v), "") if v.lstrip("-").isdigit() else (1, 0, v)


def end_k_at(r, s):
    """index of the lane section of road r at s"""
    k = 0
    for i, sec in enumerate(r.sections):
        if sec.s <= s + 1e-9:
            k = i
    return k


def lane_links(net, res, rule="file"):
    """every lane link of the network: dicts with fam 'section' | 'road' | 'junction'.
    Also records topology instances (dangling / non-reciprocal / disagreeing / missing links).

    One physical defect gives one instance: a road end that a junction's connections (and its
    connecting roads) use but whose own link does not name that junction is reported once per
    (road end, junction) as incoming_road_not_linked_to_junction, listing the connecting roads,
    instead of once per connection plus once per connecting road."""
    links = []
    unlinked = {}   # (road id, end, junction) -> instance

    def direct_link(inc, lnk, c, jid):
        """OpenDRIVE 1.9 section 12.6: the incoming road's end at the junction meets the linked road's
        contact end; laneLinks name the incoming and the linked lanes (no connecting road between)."""
        e = junction_end(inc, jid, _xy(lnk, end_s(lnk, c["contact"])))
        back = lnk.pred if c["contact"] == "start" else lnk.succ
        if e is None or back is None or back.type != "junction" or back.id != jid:
            _topo(res, inc, 0, 0, "direct_junction_not_linked", junction=jid, linked=lnk.id)
            return
        isec, lsec = inc.sections[end_k(inc, e)], lnk.sections[end_k(lnk, c["contact"])]
        pairs = []
        for f, t in c["lanes"]:
            if f not in isec.lanes or t not in lsec.lanes:
                _topo(res, inc, f, end_s(inc, e), "laneLink_lane_missing", junction=jid, connecting=lnk.id, to=t)
                continue
            pairs.append((f, t))
        links.append(dict(fam="junction", A=inc, a_end=e, B=lnk, b_contact=c["contact"], pairs=sorted(pairs),
                          junction=jid, direct=True))

    def unlinked_end(o, end, jid, con_id):
        key = (o.id, end, jid)
        if key not in unlinked:
            s = end_s(o, end)
            x, y = _xy(o, s)
            lk = o.pred if end == "start" else o.succ
            unlinked[key] = inst(o.id, 0, s, x, y, 1, 0, kind="incoming_road_not_linked_to_junction", junction=jid,
                                 end=end, link=None if lk is None else f"{lk.type}:{lk.id}", connecting="")
        c = [x for x in unlinked[key]["connecting"].split(",") if x]
        if con_id not in c:
            unlinked[key]["connecting"] = ",".join(sorted(c + [con_id], key=_idkey))

    for r in net.roads.values():
        for k in range(len(r.sections) - 1):
            a, b = r.sections[k], r.sections[k + 1]
            pairs = set()
            for l in a.lanes.values():
                if l.succ is None:
                    continue
                if l.succ not in b.lanes:
                    _topo(res, r, l.id, b.s, "lane_successor_missing", target=l.succ)
                    continue
                pairs.add((l.id, l.succ))
                m = b.lanes[l.succ]
                if m.pred is not None and m.pred != l.id:
                    _topo(res, r, l.id, b.s, "lane_link_asymmetric", succ=l.succ, back=m.pred)
            for m in b.lanes.values():
                if m.pred is None:
                    continue
                if m.pred not in a.lanes:
                    _topo(res, r, m.id, b.s, "lane_predecessor_missing", target=m.pred)
                else:
                    pairs.add((m.pred, m.id))
            links.append(dict(fam="section", road=r, k=k, pairs=sorted(pairs)))
        for end, lk in (("start", r.pred), ("end", r.succ)):
            if lk is None:
                continue
            if lk.type == "road":
                o = net.roads.get(lk.id)
                if o is None or lk.contact not in ("start", "end"):
                    _topo(res, r, 0, end_s(r, end), "road_link_target_missing", target=lk.id, contact=lk.contact)
                    continue
                back = o.pred if lk.contact == "start" else o.succ
                ok = back is not None and (
                    (back.type == "road" and back.id == r.id and back.contact == end) or
                    (back.type == "junction" and not r.plain and back.id == r.junction))
                if not ok:
                    if not r.plain and r.junction in net.junctions and (back is None or back.type == "junction"):
                        # a connecting road leads to a road end that is not linked to (this) junction
                        unlinked_end(o, lk.contact, r.junction, r.id)
                    else:
                        _topo(res, r, 0, end_s(r, end), "road_link_not_reciprocal", target=o.id, contact=lk.contact,
                              back=None if back is None else f"{back.type}:{back.id}:{back.contact}")
                sec = r.sections[end_k(r, end)]
                osec = o.sections[end_k(o, lk.contact)]
                pairs = []
                for l in sec.lanes.values():
                    t = l.pred if end == "start" else l.succ
                    if t is None:
                        continue
                    if t not in osec.lanes:
                        _topo(res, r, l.id, end_s(r, end), "lane_link_target_missing", target_road=o.id, target=t)
                        continue
                    pairs.append((l.id, t))
                links.append(dict(fam="road", A=r, a_end=end, B=o, b_contact=lk.contact, pairs=sorted(pairs)))
            elif lk.type == "junction" and lk.id not in net.junctions:
                _topo(res, r, 0, end_s(r, end), "junction_missing", target=lk.id)
    for jid in sorted(net.junctions, key=_idkey):
        # a cross path names its crossing road and, on the roads at its ends, the lanes it links at s
        for cp in net.cross_paths.get(jid, []):
            X = net.roads.get(cp["crossing"])
            for k, (road, (frm, s_at, to)) in enumerate(zip((cp["at_start"], cp["at_end"]), cp["links"])):
                R = net.roads.get(road)
                end = "start" if k == 0 else "end"
                ok = X is not None and R is not None and 0.0 <= s_at <= R.length + 1e-6 and \
                    frm in R.sections[end_k_at(R, s_at)].lanes and to in X.sections[end_k(X, end)].lanes
                if not ok:
                    _topo(res, road if R is None else R, frm, s_at, "cross_path_lane_missing", junction=jid,
                          crossing=cp["crossing"], to=to)
        if not net.junctions[jid] and not net.cross_paths.get(jid):
            refs = [(r, e) for r in net.roads.values() for e, lk in (("start", r.pred), ("end", r.succ))
                    if lk is not None and lk.type == "junction" and lk.id == jid]
            if refs:
                _topo(res, refs[0][0], 0, end_s(*refs[0]), "junction_without_connections", junction=jid,
                      referenced_by=",".join(r.id for r, _ in refs))
            else:
                _topo(res, "-", 0, 0, "junction_without_connections", junction=jid, referenced_by="")
        given = collections.defaultdict(set)   # (incoming, connecting, contact) -> {(from, to)}
        broken = set()                          # connections already reported as unlinked road ends
        seen_pairs = set()
        for c in net.junctions[jid]:
            inc, con = net.roads.get(c["incoming"]), net.roads.get(c["connecting"])
            if c.get("direct") and inc is not None and con is not None and c["contact"] in ("start", "end"):
                direct_link(inc, con, c, jid)
                continue
            if inc is None or con is None or c["contact"] not in ("start", "end"):
                _topo(res, inc if inc is not None else c["incoming"], 0, 0, "connection_road_missing", junction=jid,
                      connecting=c["connecting"])
                continue
            # JCT-6: one connection per (incoming road, connecting road); connecting roads are never incoming
            if (inc.id, con.id) in seen_pairs:
                _topo(res, con, 0, end_s(con, c["contact"]), "duplicate_connection", junction=jid, incoming=inc.id)
            seen_pairs.add((inc.id, con.id))
            if not inc.plain:
                _topo(res, inc, 0, 0.0, "connecting_road_as_incoming", junction=jid, connecting=con.id)
            cx, cy = _xy(con, end_s(con, c["contact"]))
            back = con.pred if c["contact"] == "start" else con.succ
            e = junction_end(inc, jid, (cx, cy), back)
            if e is None:
                # which end: the one the connecting road names, else the nearer one
                if back is not None and back.type == "road" and back.id == inc.id and back.contact in ("start", "end"):
                    e0 = back.contact
                else:
                    d = {x: math.hypot(_xy(inc, end_s(inc, x))[0] - cx, _xy(inc, end_s(inc, x))[1] - cy)
                         for x in ("start", "end")}
                    e0 = min(d, key=d.get)
                unlinked_end(inc, e0, jid, con.id)
                broken.add((inc.id, con.id, c["contact"]))
                continue
            if con.junction != jid:
                _topo(res, con, 0, end_s(con, c["contact"]), "connecting_road_not_in_junction", junction=jid,
                      road_junction=con.junction)
            isec, csec = inc.sections[end_k(inc, e)], con.sections[end_k(con, c["contact"])]
            if back is None or back.type != "road" or back.id != inc.id:
                _topo(res, con, 0, end_s(con, c["contact"]), "connecting_road_not_linked_to_incoming",
                      junction=jid, incoming=inc.id)
            pairs = []
            for f, t in c["lanes"]:
                if f not in isec.lanes or t not in csec.lanes:
                    _topo(res, inc, f, end_s(inc, e), "laneLink_lane_missing", cx, cy, junction=jid,
                          connecting=con.id, to=t)
                    continue
                cl = csec.lanes[t]
                own = cl.pred if c["contact"] == "start" else cl.succ
                if own is not None and own != f:
                    _topo(res, con, t, end_s(con, c["contact"]), "laneLink_disagrees_with_lane_link",
                          junction=jid, laneLink_from=f, lane_link=own)
                elif own is None and back is not None and back.type == "road" and back.id == inc.id:
                    # the connecting road's own lane link is the second description of the same link
                    _topo(res, con, t, end_s(con, c["contact"]), "connecting_lane_link_missing",
                          junction=jid, laneLink_from=f, incoming=inc.id)
                pairs.append((f, t))
                given[(inc.id, con.id, c["contact"])].add((f, t))
            links.append(dict(fam="junction", A=inc, a_end=e, B=con, b_contact=c["contact"], pairs=sorted(pairs),
                              junction=jid))
        # the converse: a drivable connecting-road lane entered at one of its ends (by its travel direction)
        # whose own lane link names a lane of the plain road there must be a laneLink of a connection of
        # this junction; esmini and CARLA route incoming -> connecting through the junction laneLinks.
        cons = {c["connecting"] for c in net.junctions[jid] if c["connecting"] in net.roads and not c.get("direct")}
        cons |= {r.id for r in net.roads.values() if r.junction == jid}
        for cid in sorted(cons, key=_idkey):
            con = net.roads[cid]
            for contact, lk in (("start", con.pred), ("end", con.succ)):
                if lk is None or lk.type != "road" or lk.id not in net.roads or not net.roads[lk.id].plain:
                    continue
                enter = 1 if contact == "start" else -1
                csec = con.sections[end_k(con, contact)]
                for t, cl in sorted(csec.lanes.items()):
                    own = cl.pred if contact == "start" else cl.succ
                    if own is None or not cl.drivable or enter not in con.lane_dir(cl, rule):
                        continue
                    key = (lk.id, cid, contact)
                    other = "end" if contact == "start" else "start"
                    # reported already: the road end is unlinked, a laneLink to t names another lane, or a
                    # connection enters lane t from the other end (travel direction: flow_direction)
                    if key in broken or any(tt == t for _, tt in given.get(key, ())) or any(
                            tt == t for (i2, c2, ct2), v in given.items() if c2 == cid and ct2 == other for _, tt in v):
                        continue
                    if (own, t) not in given.get(key, ()):
                        exists = any(c["incoming"] == lk.id and c["connecting"] == cid for c in net.junctions[jid])
                        _topo(res, con, t, end_s(con, contact), "laneLink_missing", junction=jid, incoming=lk.id,
                              lane_link=own, connection_exists=exists)
    for k in sorted(unlinked, key=lambda k: (_idkey(k[0]), k[1], _idkey(k[2]))):
        res.add("topology", unlinked[k])
    return links


def check_links(net, res, links):
    """G0/G1/G2 at every lane link, each physical contact once."""
    seen = set()
    for L in links:
        if L["fam"] == "section":
            r, k = L["road"], L["k"]
            sb = r.sections[k + 1].s
            A, B = End(r, k, sb, "left", 1), End(r, k + 1, sb, "right", 1)
            for la, lb in L["pairs"]:
                compare_lanes(res, "section", A, la, B, lb, r.speed)
            continue
        A, B = link_end(L["A"], L["a_end"]), contact_end(L["B"], L["b_contact"])
        v = min(L["A"].speed, L["B"].speed)
        key = tuple(sorted([(L["A"].id, L["a_end"]), (L["B"].id, L["b_contact"])]))
        if key not in seen and not L.get("direct"):
            seen.add(key)
            compare_ref(res, L["fam"], A, B, v)
        for la, lb in L["pairs"]:
            kk = tuple(sorted([(L["A"].id, L["a_end"], la), (L["B"].id, L["b_contact"], lb)]))
            if kk in seen:
                continue
            seen.add(kk)
            compare_lanes(res, L["fam"], A, la, B, lb, v)


# ============================================================================ in-road checks
def _cubic_min(rec, x0, x1):
    """exact minimum of a + b d + c d^2 + e d^3 on d in [x0, x1]: [(value, argmin)] of all
    candidates (ends and interior critical points)."""
    a, b, c, e = rec
    cand = [x0, x1]
    if abs(e) > 1e-300:
        disc = 4 * c * c - 12 * e * b
        if disc >= 0:
            for r in ((-2 * c + math.sqrt(disc)) / (6 * e), (-2 * c - math.sqrt(disc)) / (6 * e)):
                if x0 < r < x1:
                    cand.append(r)
    elif abs(c) > 1e-300:
        r = -b / (2 * c)
        if x0 < r < x1:
            cand.append(r)
    return [(a + b * d + c * d * d + e * d ** 3, d) for d in cand]


def _xy(r, s):
    x, y, *_ = r.pose(np.array([s]))
    return float(x[0]), float(y[0])


def check_record_g2(r, res):
    """GEO-4 inside a lane: the drivable lane-centre path stays G2 where laneOffset or a width record
    (of the lane or of a lane inside it) changes record, i.e. t'' must not step (a cubic-Hermite blend
    steps t'' by 6 delta / T^2 at both ends). Evaluated exactly from both one-sided limits. Joins that
    coincide with a reference-line element join are judged here only when the reference line itself is
    G2 there (otherwise G2_ref already reports the step). Lanes inside their own taper are exempt."""
    ejoin = {round(g.s0, 9): g for g in r.geoms[1:]}
    for k, sec in enumerate(r.sections):
        cand = set(float(x) for x in r.offset.st if sec.s + 1e-9 < x < sec.e - 1e-9)
        for l in sec.lanes.values():
            cand |= {sec.s + float(x) for x in l.width.st if 1e-9 < x < sec.e - sec.s - 1e-9}
        if not cand:
            continue
        S = np.array(sorted(cand))
        Pl, Pr = r.pose(S, "left"), r.pose(S, "right")
        Bl, Br = r.borders(k, S, "left"), r.borders(k, S, "right")
        ref_g2 = np.abs(Pl[3] - Pr[3]) <= TOL_K
        for lid, ln in sorted(sec.lanes.items()):
            if not ln.drivable:
                continue
            bl, br = Bl[lid], Br[lid]
            kl = border_pose(Pl, (bl[0] + bl[1]) / 2, (bl[2] + bl[3]) / 2, (bl[4] + bl[5]) / 2)[3]
            kr = border_pose(Pr, (br[0] + br[1]) / 2, (br[2] + br[3]) / 2, (br[4] + br[5]) / 2)[3]
            for i in np.nonzero(np.abs(kl - kr) > TOL_K)[0]:
                s = float(S[i])
                if (round(s, 9) in ejoin and not ref_g2[i]) or r.in_taper(k, lid, s):
                    continue
                if abs(float(bl[1][i] - bl[0][i])) <= TOL_W:
                    continue
                x, y = _xy(r, s)
                res.add_g2("G2_lane", {r.id: r}, g2_inst(r.id, lid, s, x, y, abs(float(kl[i] - kr[i])), r.speed,
                                                         family="record"))


def check_roads(net, res):
    """element joins, laneOffset / width record joins, widths, open lane ends inside roads."""
    for r in net.roads.values():
        for a, b in zip(r.geoms[:-1], r.geoms[1:]):
            x, y, h, k, dk = (float(v[0]) for v in a.eval(np.array([a.L])))
            _, _, hb, kb, _ = (float(v[0]) for v in b.eval(np.array([0.0])))
            g = math.hypot(x - b.x0, y - b.y0)
            if g > TOL_POS:
                res.add("G0_ref", inst(r.id, 0, b.s0, b.x0, b.y0, g, TOL_POS, family="element"))
            dh = abs(float(wrap(h - hb)))
            if dh > TOL_HDG:
                res.add("G1_ref", inst(r.id, 0, b.s0, b.x0, b.y0, dh, TOL_HDG, family="element",
                                       elements=f"{a.kind}->{b.kind}"))
            key = "g2_numeric_breaks_plain" if r.plain else "g2_numeric_breaks_junction"
            res.info.setdefault(key, [0, 0])
            res.info[key][1] += 1
            res.info[key][0] += abs(k - kb) > TOL_K
            if abs(k - kb) > TOL_K:
                res.add_g2("G2_ref", {r.id: r}, g2_inst(r.id, 0, b.s0, b.x0, b.y0, abs(k - kb), r.speed,
                                                        family="element", elements=f"{a.kind}->{b.kind}"))
            if abs(a.s0 + a.L - b.s0) > 1e-2:  # eps_s (GEO-2)
                _topo(res, r, 0, b.s0, "planView_s_gap", b.x0, b.y0, gap=b.s0 - a.s0 - a.L)
        if r.geoms:
            g = r.geoms[-1]
            if abs(g.s0 + g.L - r.length) > 1e-2:  # eps_s (GEO-2)
                _topo(res, r, 0, r.length, "planView_length_mismatch", g.x0, g.y0, gap=r.length - g.s0 - g.L)
        # GEO-2: plan-view element validity
        prev = None
        for s0, ln, hdg, kind, at in r.raw_geoms:
            bad = []
            if ln < 1e-3:
                bad.append("element_short")
            if kind not in ("line", "arc", "spiral"):
                bad.append("element_type_" + kind)
            if kind == "arc" and _f(SimpleNamespace(get=at.get), "curvature") == 0.0:
                bad.append("arc_zero_curvature")
            if kind == "spiral" and at.get("curvStart") == at.get("curvEnd"):
                bad.append("spiral_constant_curvature")
            if not (-math.pi < hdg <= math.pi):
                bad.append("hdg_out_of_range")
            if prev is not None and s0 < prev - 1e-9:
                bad.append("element_s_descending")
            prev = s0
            for b in bad:
                _topo(res, r, 0, min(max(s0, 0.0), r.length), b, length=ln, hdg=hdg, element=kind)
        # SIM-5: degenerate sizes that esmini / CARLA mishandle
        if r.length < MIN_LEN:
            res.add("degenerate", inst(r.id, 0, r.length / 2, *_xy(r, r.length / 2), r.length, MIN_LEN,
                                       kind="road_short", junction=r.junction))
        for sec in r.sections:
            if sec.e - sec.s < MIN_LEN and r.length >= MIN_LEN:
                res.add("degenerate", inst(r.id, 0, sec.s, *_xy(r, sec.s), sec.e - sec.s, MIN_LEN,
                                           kind="section_short", junction=r.junction))
        check_record_g2(r, res)
        polys = [(r.offset, "laneOffset_jump", 0, 0.0, r.length)]
        polys += [(l.width, "width_record_jump", l.id, s.s, s.e - s.s) for s in r.sections for l in s.lanes.values()]
        for poly, name, lane_id, base, span in polys:
            for j in range(1, len(poly.st)):
                x = poly.st[j]
                if x >= span - 1e-9:
                    continue
                v0, d0, _ = poly.ev(np.array([x]), j=j - 1)
                v1, d1, _ = poly.ev(np.array([x]), j=j)
                dv = abs(float(v1[0] - v0[0]))
                dh = abs(math.atan(float(d1[0])) - math.atan(float(d0[0])))
                if dv > TOL_POS or dh > TOL_HDG:
                    xs, ys = _xy(r, base + x)
                    val = dv if dv > TOL_POS else dh
                    res.add(name, inst(r.id, lane_id, base + x, xs, ys, val, TOL_POS if dv > TOL_POS else TOL_HDG,
                                       kind="value" if dv > TOL_POS else "slope", value_step=dv, slope_step=dh))
        for k, sec in enumerate(r.sections):
            L = sec.e - sec.s
            for l in sec.lanes.values():
                if l.has_border:
                    _topo(res, r, l.id, sec.s, "border_record_unsupported")
                st = list(l.width.st) + [max(L, l.width.st[-1])]
                cands, maxes = [], []
                for j in range(len(l.width.st)):
                    x0, x1 = st[j], min(st[j + 1], L)
                    if x1 < x0:
                        continue
                    cands += [(v, x0 + d) for v, d in _cubic_min(tuple(l.width.C[j]), 0.0, x1 - x0)]
                    maxes += [-v for v, _ in _cubic_min(tuple(-l.width.C[j]), 0.0, x1 - x0)]
                if not cands:
                    continue
                wmin, at = min(cands)
                if wmin < -TOL_W:
                    res.add("width_negative", inst(r.id, l.id, sec.s + at, *_xy(r, sec.s + at), wmin, -TOL_W, type=l.type))
                if not l.drivable:
                    continue
                if max(maxes) <= TOL_W:
                    res.add("width_pinch", inst(r.id, l.id, sec.s + L / 2, *_xy(r, sec.s + L / 2), max(maxes), TOL_W,
                                                kind="zero_lane", type=l.type, length=L))
                    continue
                for m, a in cands:
                    if m <= TOL_W and 1e-6 < a < L - 1e-6:
                        g = np.linspace(0, L, max(3, int(L / 0.05) + 1))
                        wg = l.width.ev(g)[0]
                        kind = "zero_tail" if wg[g >= a].max() <= TOL_W else                             "zero_head" if wg[g <= a].max() <= TOL_W else "interior"
                        zl = float((wg <= TOL_W).sum() * (g[1] - g[0]))
                        res.add("width_pinch", inst(r.id, l.id, sec.s + a, *_xy(r, sec.s + a), m, TOL_W,
                                                    kind=kind, type=l.type, zero_length=zl))
                        break
                if k + 1 < len(r.sections) and l.succ is not None and l.succ in r.sections[k + 1].lanes:
                    m2 = r.sections[k + 1].lanes[l.succ]
                    we = float(l.width.ev(np.array([L]))[0][0])
                    if we <= TOL_W and float(m2.width.ev(np.array([0.0]))[0][0]) <= TOL_W:
                        res.add("width_pinch", inst(r.id, l.id, sec.e, *_xy(r, sec.e), we, TOL_W, kind="linked_zero",
                                                    type=l.type))
        for k in range(len(r.sections) - 1):
            a, b = r.sections[k], r.sections[k + 1]
            preds = {l.succ for l in a.lanes.values() if l.succ is not None}
            for l in a.lanes.values():
                if l.succ is None and not any(m.pred == l.id for m in b.lanes.values()):
                    w = float(l.width.ev(np.array([a.e - a.s]))[0][0])
                    if w > TOL_W:
                        res.add("G0_open_lane", inst(r.id, l.id, a.e, *_xy(r, a.e), w, TOL_W, kind="vanishes",
                                                     type=l.type))
            for m in b.lanes.values():
                if m.pred is None and m.id not in preds:
                    w = float(m.width.ev(np.array([0.0]))[0][0])
                    if w > TOL_W:
                        res.add("G0_open_lane", inst(r.id, m.id, b.s, *_xy(r, b.s), w, TOL_W, kind="appears",
                                                     type=m.type))


_CHEB = 0.5 - 0.5 * np.cos((2 * np.arange(4) + 1) * np.pi / 8)      # 4 interior nodes in (0, 1)
_VINV = np.linalg.inv(np.vander(_CHEB, 4, increasing=True))           # values -> cubic coefficients


def _fold_runs_piece(k0, k1, tc):
    """exact {v in [0,1]: k(v) t(v) >= 1} for k linear (k0 -> k1) and t cubic (coefficients tc,
    increasing powers): (max of k t, argmax v, measure of the set)."""
    kp = np.array([k0, k1 - k0])                                       # increasing powers
    f = np.convolve(kp, tc)                                            # degree 4, increasing
    fd = np.polynomial.polynomial.polyder(f)
    vs = [0.0, 1.0] + [float(r.real) for r in np.polynomial.polynomial.polyroots(fd)
                       if abs(r.imag) < 1e-12 and 0.0 < r.real < 1.0]
    vals = np.polynomial.polynomial.polyval(np.array(vs), f)
    i = int(np.argmax(vals))
    if vals[i] < 1.0:
        return float(vals[i]), vs[i], 0.0
    g = f.copy()
    g[0] -= 1.0
    cuts = sorted([0.0, 1.0] + [float(r.real) for r in np.polynomial.polynomial.polyroots(g)
                                if abs(r.imag) < 1e-12 and 0.0 < r.real < 1.0])
    meas = sum(b - a for a, b in zip(cuts[:-1], cuts[1:])
               if np.polynomial.polynomial.polyval((a + b) / 2, f) >= 1.0)
    return float(vals[i]), vs[i], meas


def check_fold(net, res):
    """GEO-5 offset-curve fold: k(s) t(s) >= 1 for some lane border t (and lane 0 = laneOffset).

    Exact per polynomial piece (spec GEO-5): between consecutive breakpoints (element joins, section
    and laneOffset / width-record starts) k is linear (line, arc, spiral) and every border t is a cubic,
    so k t is a quartic whose maximum is taken at an end or a root of its derivative. The cubic is
    recovered from 4 interior evaluations (exact interpolation), the linear k from the element. Pieces
    are pre-screened with the rigorous bound max|k| * sum|cubic coefficients| < 1. paramPoly3 / poly3
    pieces (non-polynomial k) fall back to FOLD_STEP sampling. One instance per contiguous fold run."""
    for r in net.roads.values():
        for k, sec in enumerate(r.sections):
            L = sec.e - sec.s
            if L <= 1e-9:
                continue
            brk = {sec.s, sec.e} | {g.s0 for g in r.geoms} | {g.s0 + g.L for g in r.geoms}
            brk |= {float(x) for x in r.offset.st} | {sec.s + float(x) for l in sec.lanes.values() for x in l.width.st}
            br = np.array(sorted(x for x in brk if sec.s - 1e-9 <= x <= sec.e + 1e-9))
            br = br[np.concatenate([[True], np.diff(br) > 1e-9])]
            if len(br) < 2:
                continue
            a, b = br[:-1], br[1:]
            n = len(a)
            nodes = (a[:, None] + (b - a)[:, None] * _CHEB[None, :]).ravel()
            P = r.pose(nodes)
            B = r.borders(k, nodes)
            # curvature per piece at its two ends from the element (linear for line / arc / spiral)
            ka = r.pose(a, "right")[3]
            kb = r.pose(b, "left")[3]
            gi = np.clip(np.searchsorted(r.gs, (a + b) / 2, "right") - 1, 0, len(r.geoms) - 1)
            poly_el = np.array([r.geoms[j].kind in ("paramPoly3", "poly3") for j in gi])
            kmax = np.maximum(np.abs(ka), np.abs(kb))
            runs = collections.defaultdict(list)  # lane -> [(s0, s1, max kt, s at max, measure)]
            for lid, bd in B.items():
                t = bd[1].reshape(n, 4)
                tc = t @ _VINV.T                                         # (n, 4) increasing powers in v
                bound = kmax * np.abs(tc).sum(1)
                for i in np.nonzero((bound >= 1.0) | poly_el)[0]:
                    if poly_el[i]:
                        ss = np.unique(np.concatenate([np.arange(a[i], b[i], FOLD_STEP), [b[i] - 1e-7]]))
                        kt = r.pose(ss)[3] * r.borders(k, ss)[lid][1]
                        j = int(np.argmax(kt))
                        m, v, meas = float(kt[j]), (ss[j] - a[i]) / (b[i] - a[i]),                             float((kt >= 1).sum() * FOLD_STEP)
                    else:
                        m, v, meas = _fold_runs_piece(float(ka[i]), float(kb[i]), tc[i])
                    if m >= 1.0:
                        runs[lid].append((float(a[i]), float(b[i]), m, float(a[i] + v * (b[i] - a[i])), meas))
            for lid, lst in runs.items():
                merged = []
                for p in lst:
                    if merged and abs(merged[-1][1] - p[0]) <= 1e-9:
                        q = merged[-1]
                        merged[-1] = (q[0], p[1]) + max(q[2:4], p[2:4]) + (q[4] + p[4],)
                    else:
                        merged.append(p)
                for s0, s1, m, sm, meas in merged:
                    P1 = r.pose(np.array([sm]))
                    bd = r.borders(k, np.array([sm]))[lid]
                    x, y = border_pose(P1, bd[1], bd[3], bd[5])[:2]
                    res.add("fold", inst(r.id, lid, sm, x[0], y[0], m, 1.0, length=meas,
                                         radius=1 / max(abs(float(P1[3][0])), 1e-12), t=float(bd[1][0]),
                                         type=sec.lanes[lid].type if lid else "center"))


# ============================================================================ OSM side
class OsmIndex:
    """OSM ways in the network frame, restricted to the ways the network converted (osmWays)."""

    def __init__(self, osm, net):
        self.osm = osm
        self.used = sorted({w for r in net.roads.values() for w in r.osm_ways if w in osm.ways})
        self.deg = collections.Counter()
        for w in self.used:
            nds = osm.ways[w][0]
            for i, n in enumerate(nds):
                self.deg[n] += 1 if i in (0, len(nds) - 1) else 2
        self._poly = {}

    def nearest_used_way(self, x, y):
        """(way id, distance) of the nearest converted way to a point."""
        if not hasattr(self, "_segs"):
            A, B, W = [], [], []
            for w in self.used:
                Q = self.poly(w)
                if len(Q) >= 2:
                    A.append(Q[:-1])
                    B.append(Q[1:])
                    W += [w] * (len(Q) - 1)
            self._segs = (np.concatenate(A), np.concatenate(B), np.array(W)) if A else None
        if self._segs is None:
            return None, math.inf
        A, B, W = self._segs
        AB = B - A
        t = np.clip(((x - A[:, 0]) * AB[:, 0] + (y - A[:, 1]) * AB[:, 1]) / np.maximum((AB * AB).sum(1), 1e-18), 0, 1)
        d = np.hypot(A[:, 0] + t * AB[:, 0] - x, A[:, 1] + t * AB[:, 1] - y)
        j = int(np.argmin(d))
        return str(W[j]), float(d[j])

    def poly(self, w):
        if w not in self._poly:
            self._poly[w] = self.osm.polyline(w) if w in self.osm.ways else np.zeros((0, 2))
        return self._poly[w]

    def tags(self, w):
        return self.osm.ways[w][1] if w in self.osm.ways else {}

    def nodes_near(self, ways, x, y, radius):
        out = []
        for w in ways:
            if w not in self.osm.ways:
                continue
            for n in self.osm.ways[w][0]:
                p = self.osm.nodes.get(n)
                if p is not None and math.hypot(p[0] - x, p[1] - y) <= radius:
                    out.append((math.hypot(p[0] - x, p[1] - y), n))
        return sorted(set(out))

    def deflection(self, n):
        """turning angle of the OSM line at node n (degree-2 node), radians; None otherwise."""
        for w in self.used:
            nds = self.osm.ways[w][0]
            if n in nds[1:-1]:
                i = nds.index(n, 1)
                a, b, c = (np.array(self.osm.nodes[q]) for q in (nds[i - 1], nds[i], nds[i + 1]))
                return abs(float(wrap(math.atan2(*(c - b)[::-1]) - math.atan2(*(b - a)[::-1]))))
        ws = [w for w in self.used if n in (self.osm.ways[w][0][0], self.osm.ways[w][0][-1])]
        if len(ws) == 2:
            pts = []
            for w in ws:
                nds = self.osm.ways[w][0]
                pts.append(np.array(self.osm.nodes[nds[1] if nds[0] == n else nds[-2]]))
            b = np.array(self.osm.nodes[n])
            h1 = math.atan2(*(b - pts[0])[::-1])
            h2 = math.atan2(*(pts[1] - b)[::-1])
            return abs(float(wrap(h2 - h1)))
        return None


def proj_poly(P, Q):
    """nearest points of P (m,2) on polyline Q (n,2): dist, foot (m,2), arc param, tangent angle, clamped."""
    A, Bv = Q[:-1], Q[1:] - Q[:-1]
    L2 = np.maximum((Bv * Bv).sum(1), 1e-18)
    D = P[:, None, :] - A[None, :, :]
    t = np.clip((D * Bv[None]).sum(2) / L2[None], 0, 1)
    F = A[None] + t[..., None] * Bv[None]
    d = np.hypot(*(P[:, None, :] - F).transpose(2, 0, 1))
    j = np.argmin(d, 1)
    i = np.arange(len(P))
    seg = np.sqrt(L2)
    cum = np.concatenate([[0], np.cumsum(seg)])
    tj = t[i, j]
    clamped = ((j == 0) & (tj <= 0)) | ((j == len(A) - 1) & (tj >= 1))
    return d[i, j], F[i, j], cum[j] + tj * seg[j], np.arctan2(Bv[j, 1], Bv[j, 0]), clamped


class RoadSamples:
    """a plain road sampled every `step` m: pose, drivable band [tmin, tmax], per-sample section."""

    def __init__(self, r, step=1.0):
        self.r = r
        n = max(2, int(math.ceil(r.length / step)) + 1)
        self.s = np.linspace(0, r.length, n)
        self.P = r.pose(self.s)
        self.tmin, self.tmax = self.band(self.s)

    def band(self, s):
        r = self.r
        s = np.atleast_1d(s)
        lo, hi = np.full(len(s), np.nan), np.full(len(s), np.nan)
        st = np.array([x.s for x in r.sections])
        k = np.clip(np.searchsorted(st, s + 1e-12, "right") - 1, 0, len(st) - 1)
        for kk in np.unique(k):
            m = k == kk
            B = r.borders(kk, s[m])
            sec = r.sections[kk]
            for lid, b in B.items():
                if lid == 0 or not sec.lanes[lid].drivable:
                    continue
                w = np.abs(b[1] - b[0])
                a, c = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
                ok = w > TOL_W
                lo[m] = np.where(ok, np.fmin(lo[m], a), lo[m])
                hi[m] = np.where(ok, np.fmax(hi[m], c), hi[m])
        return lo, hi

    def project(self, pts):
        """(s, t) of points (m,2) on the reference line: nearest sample, then exact Newton."""
        X, Y = self.P[0], self.P[1]
        out_s = np.empty(len(pts))
        for i0 in range(0, len(pts), 512):
            p = pts[i0:i0 + 512]
            d2 = (p[:, 0:1] - X[None]) ** 2 + (p[:, 1:2] - Y[None]) ** 2
            out_s[i0:i0 + 512] = self.s[np.argmin(d2, 1)]
        s = out_s
        for _ in range(4):
            x, y, h, k, _ = self.r.pose(s)
            ds = (pts[:, 0] - x) * np.cos(h) + (pts[:, 1] - y) * np.sin(h)
            s = np.clip(s + ds / np.maximum(1 - k * (-(pts[:, 0] - x) * np.sin(h) + (pts[:, 1] - y) * np.cos(h)), 0.2),
                        0, self.r.length)
        x, y, h, k, _ = self.r.pose(s)
        t = -(pts[:, 0] - x) * np.sin(h) + (pts[:, 1] - y) * np.cos(h)
        return s, t


def _runs(mask):
    """[(i0, i1)] index runs where mask is True."""
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


def check_osm_fidelity(net, oi, res, samples):
    """directed Hausdorff both ways between each plain road and the OSM ways it was built from,
    judged against the road's own drivable band (exact necessary conditions, no tolerance)."""
    for r in net.plain_roads():
        ways = [w for w in r.osm_ways if len(oi.poly(w)) >= 2]
        if not ways:
            continue
        S = samples[r.id]
        if np.isnan(S.tmin).all():
            continue
        pts = np.stack([S.P[0], S.P[1]], 1)
        D, own_param = [], []
        for w in ways:
            d, F, prm, ang, cl = proj_poly(pts, oi.poly(w))
            D.append(d), own_param.append(prm)
        D = np.array(D)
        dmin = D.min(0)
        reach = np.fmax(np.abs(S.tmin), np.abs(S.tmax))
        exc = dmin - reach
        bad = exc > 0
        for i0, i1 in _runs(bad):
            i = i0 + int(np.argmax(exc[i0:i1 + 1]))
            other, od = oi.nearest_used_way(S.P[0][i], S.P[1][i])
            kind = "userData_incomplete" if other is not None and od <= reach[i] else "geometry_off"
            res.add("osm_ref_far", inst(r.id, 0, S.s[i], S.P[0][i], S.P[1][i], exc[i], 0.0, dist=dmin[i],
                                        reach=reach[i], length=S.s[i1] - S.s[i0], hausdorff=float(dmin.max()),
                                        kind=kind, on_way=other if kind == "userData_incomplete" else None))
        near = D.argmin(0)
        worst = None
        total = 0.0
        for wi, w in enumerate(ways):
            m = near == wi
            if not m.any():
                continue
            Q = oi.poly(w)
            lo, hi = own_param[wi][m].min(), own_param[wi][m].max()
            seg = np.hypot(*np.diff(Q, axis=0).T)
            cum = np.concatenate([[0], np.cumsum(seg)])
            u = np.unique(np.concatenate([np.arange(lo, hi, 1.0), [hi], cum[(cum > lo) & (cum < hi)]]))
            q = np.stack([np.interp(u, cum, Q[:, 0]), np.interp(u, cum, Q[:, 1])], 1)
            s, t = S.project(q)
            inside = (s > 1e-6) & (s < r.length - 1e-6)
            if not inside.any():
                continue
            q, s, t = q[inside], s[inside], t[inside]
            tmin, tmax = S.band(s)
            out = np.fmax(np.fmax(tmin - t, t - tmax), 0)
            out = np.where(np.isnan(out), 0, out)
            if (out > 0).any():
                i = int(np.argmax(out))
                total += float((out > 0).sum())
                if worst is None or out[i] > worst[0]:
                    worst = (float(out[i]), s[i], q[i], w, t[i], tmin[i], tmax[i])
        if worst:
            v, s, q, w, t, a, b = worst
            res.add("osm_off_road", inst(r.id, 0, s, q[0], q[1], v, 0.0, way=w, t_osm=t, band=f"{a:.2f}..{b:.2f}",
                                         points_outside=total))


# ---------------------------------------------------------------------------- lane count
def _int(v):
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return None


def osm_expected(tags):
    """lanes per direction relative to the way, from tags only: dict(fwd, bwd, both, total), None =
    not determined. Returns None when the way's traffic direction is undetermined (reversible)."""
    ow = tags.get("oneway", "")
    if ow in ("reversible", "alternating"):
        return None
    one = 0
    if ow in ("yes", "true", "1"):
        one = 1
    elif ow in ("-1", "reverse"):
        one = -1
    elif ow != "no" and (tags.get("junction") in ("roundabout", "circular") or tags.get("highway") == "motorway"):
        one = 1
    n = _int(tags.get("lanes"))
    f, b, both = _int(tags.get("lanes:forward")), _int(tags.get("lanes:backward")), _int(tags.get("lanes:both_ways"))
    if one:
        tot = n if n is not None else (f if one == 1 else b)
        return dict(fwd=tot if one == 1 else 0, bwd=tot if one == -1 else 0, both=0, total=tot)
    both = both or 0
    if n is not None:
        if f is None and b is not None:
            f = n - b - both
        if b is None and f is not None:
            b = n - f - both
    return dict(fwd=f, bwd=b, both=both if (n is not None or f is not None) else None, total=n)


def lane_tapers(r, step=0.25):
    """{(k, lane id): [(s0, s1)]} intervals where a lane is opening from / closing to zero width: from
    a zero-width chain end up to the first point where the width stops changing monotonically."""
    out = collections.defaultdict(list)
    for k0, sec in enumerate(r.sections):
        for lid, ln in sec.lanes.items():
            if ln.pred is not None and k0 > 0 and ln.pred in r.sections[k0 - 1].lanes:
                continue  # not a chain start
            chain, k, l = [], k0, ln
            while True:
                chain.append((k, l))
                if l.succ is None or k + 1 >= len(r.sections) or l.succ not in r.sections[k + 1].lanes:
                    break
                k, l = k + 1, r.sections[k + 1].lanes[l.succ]
            S, W, D, K = [], [], [], []
            for k, l in chain:
                s0, s1 = r.sections[k].s, r.sections[k].e
                ss = np.unique(np.concatenate([np.arange(s0, s1, step), [s1]]))
                w, w1, _ = l.width.ev(ss - s0)
                S.append(ss), W.append(w), D.append(w1), K.append(np.full(len(ss), k))
            S, W, D, K = (np.concatenate(v) for v in (S, W, D, K))
            if W[0] <= TOL_W:
                i = 0
                while i + 1 < len(S) and D[i + 1] > 1e-4:
                    i += 1
                for k in np.unique(K[:i + 1]):
                    out[(int(k), chain[[c[0] for c in chain].index(k)][1].id)].append((S[0], S[i]))
            if W[-1] <= TOL_W:
                i = len(S) - 1
                while i - 1 >= 0 and D[i - 1] < -1e-4:
                    i -= 1
                for k in np.unique(K[i:]):
                    out[(int(k), chain[[c[0] for c in chain].index(k)][1].id)].append((S[i], S[-1]))
    return out


def check_lane_count(net, oi, res, samples, rule):
    """drivable lanes per travel direction (full width = not inside a taper) vs the OSM tags of the
    ways abeam: tag must lie in [established, established + tapering]."""
    for r in net.plain_roads():
        ways = [w for w in r.osm_ways if len(oi.poly(w)) >= 2]
        if not ways:
            continue
        S = samples[r.id]
        tap = lane_tapers(r)
        ss = np.arange(1.0, r.length - 1.0, 2.0)
        if not len(ss):
            continue
        x, y, h, _, _ = r.pose(ss)
        tmin, tmax = S.band(ss)
        pts = np.stack([x, y], 1)
        proj = {w: proj_poly(pts, oi.poly(w)) for w in ways}
        bad_flags, info = np.zeros(len(ss), bool), [None] * len(ss)
        # nodes where one of the road's ways ends: within the cross-section that contains such a node the
        # governing way (and so the expected count) is undetermined -> those samples are not judged
        ends = np.array([oi.osm.nodes[e] for w in ways for e in (oi.osm.ways[w][0][0], oi.osm.ways[w][0][-1])
                         if e in oi.osm.nodes])
        for i, s in enumerate(ss):
            if np.isnan(tmin[i]):
                continue
            reach = max(abs(tmin[i]), abs(tmax[i]))
            if len(ends) and np.min(np.hypot(ends[:, 0] - x[i], ends[:, 1] - y[i])) <= reach + TOL_POS:
                continue
            exp = dict(plus=0, minus=0, both=0, total=0)
            any_way = False
            abeam = []
            for w in ways:
                d, F, prm, ang, cl = (v[i] for v in proj[w])
                if cl:
                    continue
                tF = -(F[0] - x[i]) * math.sin(h[i]) + (F[1] - y[i]) * math.cos(h[i])
                # abeam = the way crosses this cross-section inside the drivable band (lateral AND
                # longitudinal: the foot point is no farther than the band reaches)
                if tmin[i] - TOL_POS <= tF <= tmax[i] + TOL_POS and d <= max(abs(tmin[i]), abs(tmax[i])) + TOL_POS:
                    abeam.append((d, w, F, ang))
            # consecutive ways (sharing an end node, feet on opposite sides of it along the road) are one
            # line seen twice near the node: keep the nearer; parallel ways (gore) are summed
            drop = set()
            for p1 in abeam:
                for p2 in abeam:
                    if p1[1] >= p2[1] or p1[1] in drop or p2[1] in drop:
                        continue
                    e1, e2 = oi.osm.ways[p1[1]][0], oi.osm.ways[p2[1]][0]
                    for nd in {e1[0], e1[-1]} & {e2[0], e2[-1]}:
                        # direction of each way leaving the shared end node: opposite = consecutive
                        u = [np.array(oi.osm.nodes[e[1] if e[0] == nd else e[-2]]) - np.array(oi.osm.nodes[nd])
                             for e in (e1, e2)]
                        if np.dot(u[0], u[1]) < 0:
                            drop.add(p2[1] if p2[0] > p1[0] else p1[1])
            for d, w, F, ang in abeam:
                if w in drop:
                    continue
                e = osm_expected(oi.tags(w))
                if e is None:
                    exp = None
                    break
                same = math.cos(ang - h[i]) > 0
                fw, bw = (e["fwd"], e["bwd"]) if same else (e["bwd"], e["fwd"])
                for key, val in (("plus", fw), ("minus", bw), ("both", e["both"]), ("total", e["total"])):
                    exp[key] = None if (val is None or exp[key] is None) else exp[key] + val
                any_way = True
            if not any_way or exp is None:
                continue
            k = r.section_index(s)
            sec = r.sections[k]
            est = collections.Counter()
            tp = collections.Counter()
            for lid, ln in sec.lanes.items():
                if not ln.drivable:
                    continue
                w = float(ln.width.ev(np.array([s - sec.s]))[0][0])
                if w <= TOL_W:
                    continue
                dirs = r.lane_dir(ln, rule)
                key = "both" if len(dirs) == 2 else ("plus" if dirs[0] == 1 else "minus")
                if any(a - 1e-9 <= s <= b + 1e-9 for a, b in tap.get((k, lid), [])):
                    tp[key] += 1
                else:
                    est[key] += 1
            msg = []
            for key in ("plus", "minus", "both"):
                if exp[key] is not None and not (est[key] <= exp[key] <= est[key] + tp[key]):
                    msg.append(f"{key}:{est[key]}+{tp[key]}t vs osm {exp[key]}")
            if exp["total"] is not None:
                e_t, t_t = sum(est.values()), sum(tp.values())
                if not (e_t <= exp["total"] <= e_t + t_t):
                    msg.append(f"total:{e_t}+{t_t}t vs osm {exp['total']}")
            if msg:
                bad_flags[i] = True
                info[i] = (";".join(msg), dict(est), dict(tp), exp)
        for i0, i1 in _runs(bad_flags):
            i = (i0 + i1) // 2
            m, est, tp, exp = info[i]
            res.add("lane_count", inst(r.id, 0, ss[i], x[i], y[i], sum(est.values()), exp["total"] if exp["total"] is not None else -1,
                                       mismatch=m, length=ss[i1] - ss[i0] + 2.0, ways=",".join(ways)))


# ---------------------------------------------------------------------------- continuity / split / shape
def recenter_cubic(rec, d0):
    """cubic a + b d + c d^2 + e d^3 re-expanded around d0."""
    a, b, c, e = rec
    return (a + b * d0 + c * d0 * d0 + e * d0 ** 3, b + 2 * c * d0 + 3 * e * d0 * d0, c + 3 * e * d0, e)


def _side_lanes(sec, sign):
    return [l for l in sec.lanes.values() if (l.id > 0) == (sign > 0)]


def check_plain_links(net, oi, res, links):
    """non-driving continuity, needless split and shape-unchanged at plain road-road links and
    (continuity) at lane-section boundaries."""
    for L in links:
        if L["fam"] == "section":
            r, k = L["road"], L["k"]
            a, b = r.sections[k], r.sections[k + 1]
            linked_a = {la for la, lb in L["pairs"]}
            linked_b = {lb for la, lb in L["pairs"]}
            for l in a.lanes.values():
                if l.type in EDGE_TYPES and l.id not in linked_a:
                    if any(m.type == l.type and m.id not in linked_b for m in _side_lanes(b, l.id)):
                        res.add("continuity_nondriving", inst(r.id, l.id, b.s, *_xy(r, b.s), 1, 0, kind="section",
                                                              type=l.type))
            # NAT-2: a removable section boundary (same lane ids and types, every lane linked 1:1 to its
            # own id). Widths, marks and speed all have an sOffset form, so they never need a section.
            ids_a, ids_b = sorted(a.lanes), sorted(b.lanes)
            if ids_a == ids_b and all(a.lanes[i].type == b.lanes[i].type for i in ids_a) \
                    and set(L["pairs"]) == {(i, i) for i in ids_a}:
                diff = []
                for i in ids_a:
                    wa = a.lanes[i].width.ev(np.array([a.e - a.s]), "left")
                    j = a.lanes[i].width.idx(np.array([a.e - a.s]), "left")[0]
                    c = recenter_cubic(tuple(a.lanes[i].width.C[j]), a.e - a.s - a.lanes[i].width.st[j])
                    cb = tuple(b.lanes[i].width.C[0]) if b.lanes[i].width.st[0] <= 1e-9 else None
                    if cb is None or max(abs(x - y) for x, y in zip(c, cb)) > 1e-6:
                        diff.append(f"width-poly {i}" if abs(float(wa[0][0]) - float(b.lanes[i].width.ev(
                            np.array([0.0]))[0][0])) <= TOL_POS else f"width-jump {i}")
                res.add("needless_split", inst(r.id, 0, b.s, *_xy(r, b.s), len(ids_a), 0, kind="section",
                                               layout=" ".join(f"{i}:{a.lanes[i].type}" for i in ids_a),
                                               differs=",".join(diff) or "identical"))
            continue
        if L["fam"] != "road" or not (L["A"].plain and L["B"].plain):
            continue
        A, B = L["A"], L["B"]
        ea, eb = link_end(A, L["a_end"]), contact_end(B, L["b_contact"])
        flip = ea.d * eb.d
        seca, secb = A.sections[ea.k], B.sections[eb.k]
        linked_a = {la for la, lb in L["pairs"]}
        linked_b = {lb for la, lb in L["pairs"]}
        xa, ya, ha, _ = ea.ref()
        for l in seca.lanes.values():
            if l.type in EDGE_TYPES and l.id not in linked_a:
                if any(m.type == l.type and m.id not in linked_b for m in _side_lanes(secb, l.id * flip)):
                    res.add("continuity_nondriving", inst(A.id, l.id, ea.s, xa, ya, 1, 0, kind="road", type=l.type,
                                                          to_road=B.id))
        if A.id == B.id:
            continue
        # needless split (each unordered pair once: from the lower id)
        lay_a = sorted((l.id, l.type) for l in seca.lanes.values())
        lay_b = sorted((l.id * flip, l.type) for l in secb.lanes.values())
        if lay_a == lay_b and (int(A.id) if A.id.isdigit() else A.id) < (int(B.id) if B.id.isdigit() else B.id):
            lo, hi = ea.B[0][0][0], ea.B[0][0][0]
            for lid, b in ea.B.items():
                lo, hi = min(lo, b[0][0], b[1][0]), max(hi, b[0][0], b[1][0])
            rad = max(abs(lo), abs(hi)) + TOL_POS
            near = oi.nodes_near(A.osm_ways + B.osm_ways, xa, ya, rad) if oi else []
            jn = [n for _, n in near if oi.deg[n] >= 3]
            if not jn:
                ta = {w: oi.tags(w) for w in A.osm_ways} if oi else {}
                tb = {w: oi.tags(w) for w in B.osm_ways} if oi else {}
                diff = sorted({k for x in ta.values() for y in tb.values() for k in set(x) | set(y)
                               if x.get(k) != y.get(k) and k in ("name", "ref", "highway", "maxspeed", "bridge",
                                                                    "tunnel", "layer", "lanes", "width", "surface")})
                res.add("needless_split", inst(A.id, 0, ea.s, xa, ya, len(lay_a), 0, to_road=B.id,
                                               layout=" ".join(f"{i}:{t}" for i, t in lay_a),
                                               osm_node=near[0][1] if near else None,
                                               osm_tag_diff=",".join(diff)))
        # shape unchanged: heading continuity of ref line (when touching) and every lane border
        steps = []
        xb, yb, hb, _ = eb.ref()
        if math.hypot(xa - xb, ya - yb) <= TOL_POS:
            steps.append(("ref", abs(float(wrap(ha - hb)))))
        for la, lb in L["pairs"]:
            for which in (0, 1):
                steps.append((f"lane{la}", abs(float(wrap(ea.edge(la, which)[2] - eb.edge(lb, which)[2])))))
        if steps:
            what, worst = max(steps, key=lambda t: t[1])
            if worst > TOL_HDG and (int(A.id) if A.id.isdigit() else A.id) < (int(B.id) if B.id.isdigit() else B.id):
                near = oi.nodes_near(A.osm_ways + B.osm_ways, xa, ya, 30.0) if oi else []
                defl = oi.deflection(near[0][1]) if near else None
                res.add("shape_unchanged", inst(A.id, 0, ea.s, xa, ya, worst, TOL_HDG, at=what, to_road=B.id,
                                                osm_deflection=defl if defl is not None else -1))


def junction_arms(net, jid):
    """plain road ends attached to a junction: [(road, end)] (incoming roads and outgoing roads)."""
    arms = set()
    for c in net.junctions[jid]:
        inc, con = net.roads.get(c["incoming"]), net.roads.get(c["connecting"])
        if inc is None or con is None:
            continue
        cx, cy, *_ = con.pose(np.array([end_s(con, c["contact"])]))
        e = junction_end(inc, jid, (float(cx[0]), float(cy[0])), con.pred if c["contact"] == "start" else con.succ)
        if e is not None:
            arms.add((inc.id, e))
        if c.get("direct"):  # the linked road itself is an arm, at its contact end
            arms.add((con.id, c["contact"]))
            continue
        other = con.succ if c["contact"] == "start" else con.pred
        if other is not None and other.type == "road" and other.id in net.roads and other.contact in ("start", "end"):
            arms.add((other.id, other.contact))
    return sorted(arms)


def check_junctions(net, oi, res):
    """needless split at junctions and corner continuity of edge lanes through junctions."""
    for jid in sorted(net.junctions, key=lambda x: (len(x), x)):
        conns = net.junctions[jid]
        if not conns:
            continue
        arms = junction_arms(net, jid)
        if not arms:
            continue
        info, info_h = [], {}
        for rid, end in arms:
            r = net.roads[rid]
            E = link_end(r, end)  # traversal leaving the road = entering the junction
            x, y, h, _ = E.ref()
            sec = r.sections[E.k]
            # inbound traversal: right side = negative ids when d = +1, positive when d = -1
            right = {l.type for l in sec.lanes.values() if (l.id < 0) == (E.d == 1)}
            left = {l.type for l in sec.lanes.values() if (l.id > 0) == (E.d == 1)}
            reach = max(max(abs(float(b[0][0])), abs(float(b[1][0]))) for b in E.B.values())
            info.append(dict(road=rid, end=end, x=x, y=y, d=E.d, right=right, left=left, reach=reach, sec=sec))
            info_h[rid, end] = h  # heading entering the junction
        cx, cy = np.mean([a["x"] for a in info]), np.mean([a["y"] for a in info])
        for a in info:
            a["ang"] = math.atan2(a["y"] - cy, a["x"] - cx)
        info.sort(key=lambda a: a["ang"])
        pairs_conn = collections.defaultdict(set)  # (road a, road b) -> lane types carried by connecting roads
        for c in conns:
            con = net.roads.get(c["connecting"])
            if con is None:
                continue
            other = con.succ if c["contact"] == "start" else con.pred
            ob = other.id if other is not None and other.type == "road" else None
            types = {l.type for sec in con.sections for l in sec.lanes.values()}
            for k in ((c["incoming"], ob), (ob, c["incoming"])):
                pairs_conn[k] |= types
        # corner continuity between angularly consecutive arms
        n = len(info)
        if n >= 2:
            for i in range(n if n > 2 else 2):
                A, B = info[i], info[(i + 1) % n]
                if A is B:
                    continue
                # a U-turn nose (heading change > 150 deg, e.g. the median between the two carriageways of
                # one street) is not a corner: no curb runs from one carriageway to the other there
                turn = abs(float(wrap(info_h[B["road"], B["end"]] + math.pi - info_h[A["road"], A["end"]])))
                if turn > math.radians(150):
                    continue
                common = (A["right"] & B["left"]) & EDGE_TYPES
                for T in sorted(common):
                    if T not in pairs_conn.get((A["road"], B["road"]), set()):
                        res.add("continuity_junction_corner",
                                inst(A["road"], 0, end_s(net.roads[A["road"]], A["end"]), A["x"], A["y"], 1, 0,
                                     kind="junction_corner", type=T, junction=jid, to_road=B["road"]))
        # JCT-6/not-only-two: a junction must join three or more carriageway ends. With two (or one), the
        # road must continue (NAT-1 / NAT-3). osm_branch names a branching OSM node (degree >= 3 among the
        # converted ways) inside the junction footprint if there is one: then OSM does branch and the
        # converter dropped the third arm; without it OSM does not diverge there at all.
        if len(info) <= 2:
            branch = []
            if oi is not None:
                ways = sorted({w for a in info for w in net.roads[a["road"]].osm_ways})
                pts = []
                for c in conns:
                    con = net.roads.get(c["connecting"])
                    if con is not None:
                        xs, ys, *_ = con.pose(np.linspace(0, con.length, 9))
                        pts += list(zip(xs, ys))
                pts += [(a["x"], a["y"]) for a in info]
                rad = max(a["reach"] for a in info) + TOL_POS
                for w in ways:
                    if w not in oi.osm.ways:
                        continue
                    for nd in oi.osm.ways[w][0]:
                        if oi.deg[nd] >= 3 and nd in oi.osm.nodes:
                            q = oi.osm.nodes[nd]
                            if min(math.hypot(px - q[0], py - q[1]) for px, py in pts) <= rad:
                                branch.append(nd)
            same = None
            if len(info) == 2:
                A, B = info
                flip = A["d"] * (-B["d"])  # B is left in traversal direction -d
                la = sorted((l.id, l.type) for l in A["sec"].lanes.values())
                lb = sorted((l.id * flip, l.type) for l in B["sec"].lanes.values())
                same = la == lb
            res.add("needless_split", inst(info[0]["road"], 0, end_s(net.roads[info[0]["road"]], info[0]["end"]),
                                           cx, cy, len(info), 3, kind="junction_not_only_two", junction=jid,
                                           roads=",".join(a["road"] for a in info), same_layout=same,
                                           osm_branch=min(branch) if branch else None))


# ============================================================================ natural flow
def lane_max_width(ln, L):
    """exact maximum width of a lane over its section [0, L] (cubic end and critical points)."""
    st = list(ln.width.st) + [max(L, ln.width.st[-1])]
    m = 0.0
    for j in range(len(ln.width.st)):
        x0, x1 = st[j], min(st[j + 1], L)
        if x1 >= x0:
            m = max(m, max(-v for v, _ in _cubic_min(tuple(-ln.width.C[j]), 0.0, x1 - x0)))
    return m


def end_boundary(net, oi, r, end, clip_xy):
    """FLOW-1 (i)/(ii): is this road end a legal network boundary?  Returns (kind, inside) with kind in
    {None, 'clip', 'dead_end'} and inside = distance the end lies inside the clip box (m, <= 0 outside).

    Only an UNLINKED road end can be a boundary (spec FLOW-1: a road linked at that end, e.g. to a
    junction kept just outside the box, must link every lane there).
      clip      the end lies outside the clip box or within eps_B of its edge;
      dead_end  an OSM end node of the road's own ways that no other converted way touches (degree 1)
                lies within the road's drivable reach of the end (JCT-5)."""
    if (r.pred if end == "start" else r.succ) is not None:
        return None, None
    x, y = _xy(r, end_s(r, end))
    inside = None
    if clip_xy is not None:
        x0, y0, x1, y1 = clip_xy
        inside = min(x - x0, x1 - x, y - y0, y1 - y)
        if inside <= EPS_B:
            return "clip", inside
    if oi is None:
        return None, inside
    b = RoadSamples.band(SimpleNamespace(r=r), np.array([end_s(r, end)]))
    allb = np.concatenate(b)
    if np.isnan(allb).all():
        return None, inside
    reach = float(np.nanmax(np.abs(allb)))
    for d, n in oi.nodes_near(r.osm_ways, x, y, reach + TOL_POS):
        if oi.deg[n] == 1:
            return "dead_end", inside
    return None, inside


class SimpleNamespace:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _scc(nodes, adj):
    """Tarjan's strongly connected components, iterative. nodes: sorted list; adj: node -> iterable.
    Returns {node: component index} (deterministic for a sorted node list and sorted adjacency)."""
    index, low, comp, onstack, stack = {}, {}, {}, set(), []
    counter, ncomp = 0, 0
    for root in nodes:
        if root in index:
            continue
        work = [(root, iter(sorted(adj.get(root, ()))))]
        index[root] = low[root] = counter
        counter += 1
        stack.append(root)
        onstack.add(root)
        while work:
            v, it = work[-1]
            advanced = False
            for w in it:
                if w not in index:
                    index[w] = low[w] = counter
                    counter += 1
                    stack.append(w)
                    onstack.add(w)
                    work.append((w, iter(sorted(adj.get(w, ())))))
                    advanced = True
                    break
                if w in onstack:
                    low[v] = min(low[v], index[w])
            if advanced:
                continue
            work.pop()
            if work:
                u = work[-1][0]
                low[u] = min(low[u], low[v])
            if low[v] == index[v]:
                while True:
                    w = stack.pop()
                    onstack.discard(w)
                    comp[w] = ncomp
                    if w == v:
                        break
                ncomp += 1
    return comp


def check_flow(net, oi, res, links, rule, clip_xy):
    """the natural-flow game (spec FLOW-0 .. FLOW-4, FLOW-6): every drivable lane must be enterable
    from a legal source and must lead to a legal sink, along lane links and lane changes.

    Lane graph (FLOW-0): nodes = (road, section, lane, travel sign); longitudinal edges = in-road lane
    links, road-road lane links, and junction laneLinks into connecting roads; lateral edges = lane
    changes between adjacent same-direction drivable lanes. The entry into a connecting road comes ONLY
    from the junction's laneLink (as esmini and CARLA route it); the connecting lane's own link toward
    the plain road it starts from never creates that edge (a missing laneLink is topology/laneLink_missing
    and, here, a dead lane end). The exit from a connecting road comes from its own road / lane links.

    Reported (one instance per root cause, spec 0.3):
      flow_direction        a lane link joining opposite travel directions (FLOW-2/wrong-way);
      flow_dead_end/_start  a lane end with nothing after / before it that is not a legal sink / source:
                            code exit-unlinked / entry-unlinked (the road end has no link at all; one
                            instance per road end, lanes listed), exit-no-lane-link / entry-no-lane-link
                            (road linked, this lane not; entry is SHOULD), abrupt-end / abrupt-start
                            (inside a road at width > eps_w);
      flow_boundary_short   an unlinked boundary end more than eps_pos inside the clip box (SHOULD);
      flow_orphan / _trap   strongly connected components of the lane graph with no incoming / outgoing
                            condensation edge and no legal source / sink (FLOW-4); the representative lane
                            is the one whose end has no longitudinal edge, value = lanes in the SCC;
      flow_closed_not_strong a weak component with neither sources nor sinks that is not one SCC;
      flow_narrow           a drivable lane chain (linked across the sections of one road) whose exact
                            maximum width is below W_P (FLOW-6/narrow);
      flow_no_drivable      a road with no drivable lane on a motor-traffic way (FLOW-6).
    meta.flow carries the per-lane statistics (unreachable / trapped lanes, splits, merges)."""
    nodes = {}
    for r in net.roads.values():
        for k, sec in enumerate(r.sections):
            for lid, ln in sec.lanes.items():
                if ln.drivable:
                    for d in r.lane_dir(ln, rule):
                        nodes[(r.id, k, lid, d)] = ln
    succ, pred = collections.defaultdict(set), collections.defaultdict(set)
    lat, lat_in = collections.defaultdict(set), collections.defaultdict(set)   # directed lane changes
    conflicts = {}

    def link(A, ka, la, dA, B, kb, lb, dB, fam, s, xy, fwd_ok=True, rev_ok=True):
        """lane la of road A whose link side is traversed in direction dA (+1 = leaving A at its +s end)
        joined to lane lb of road B entered in direction dB. fwd_ok / rev_ok: whether the A->B / B->A
        traversal may create an edge (entries into connecting roads come only from laneLinks)."""
        ra, rb = net.roads[A], net.roads[B]
        lna, lnb = ra.sections[ka].lanes[la], rb.sections[kb].lanes[lb]
        if not (lna.drivable and lnb.drivable):
            return
        da, db = ra.lane_dir(lna, rule), rb.lane_dir(lnb, rule)
        ok = False
        for x in da:
            for y in db:
                if (x == dA) and (y == dB):
                    ok = True
                    if fwd_ok:
                        succ[(A, ka, la, x)].add((B, kb, lb, y)), pred[(B, kb, lb, y)].add((A, ka, la, x))
                elif (x == -dA) and (y == -dB):
                    ok = True
                    if rev_ok:
                        succ[(B, kb, lb, y)].add((A, ka, la, x)), pred[(A, ka, la, x)].add((B, kb, lb, y))
        if not ok:
            key = tuple(sorted([(A, ka, la), (B, kb, lb)]))
            conflicts[key] = inst(A, la, s, xy[0], xy[1], 1, 0, family=fam, to_road=B, to_lane=lb,
                                  dirs=f"{da}->{db}")

    for L in links:
        if L["fam"] == "section":
            r, k = L["road"], L["k"]
            xy = _xy(r, r.sections[k + 1].s)
            for la, lb in L["pairs"]:
                link(r.id, k, la, 1, r.id, k + 1, lb, 1, "section", r.sections[k + 1].s, xy)
        else:
            A, B = L["A"], L["B"]
            dA = 1 if L["a_end"] == "end" else -1
            dB = 1 if L["b_contact"] == "start" else -1
            xy = _xy(A, end_s(A, L["a_end"]))
            # entering a connecting road from a plain road needs a junction laneLink
            fwd_ok = L["fam"] == "junction" or B.plain or not A.plain
            rev_ok = A.plain or not B.plain
            for la, lb in L["pairs"]:
                if L["fam"] == "junction":
                    lna, lnb = A.sections[end_k(A, L["a_end"])].lanes[la], B.sections[end_k(B, L["b_contact"])].lanes[lb]
                    if lna.drivable and lnb.drivable and not (dA in A.lane_dir(lna, rule) and dB in B.lane_dir(lnb, rule)):
                        key = tuple(sorted([(A.id, end_k(A, L["a_end"]), la), (B.id, end_k(B, L["b_contact"]), lb)]))
                        conflicts[key] = inst(A.id, la, end_s(A, L["a_end"]), xy[0], xy[1], 1, 0, family="junction",
                                              to_road=B.id, to_lane=lb, junction=L["junction"],
                                              dirs=f"{A.lane_dir(lna, rule)}->{B.lane_dir(lnb, rule)}")
                        continue
                    link(A.id, end_k(A, L["a_end"]), la, dA, B.id, end_k(B, L["b_contact"]), lb, dB, "junction",
                         end_s(A, L["a_end"]), xy, True, False)
                else:
                    link(A.id, end_k(A, L["a_end"]), la, dA, B.id, end_k(B, L["b_contact"]), lb, dB, L["fam"],
                         end_s(A, L["a_end"]), xy, fwd_ok, rev_ok)
    for c in conflicts.values():
        res.add("flow_direction", c)
    for r in net.roads.values():
        for k, sec in enumerate(r.sections):
            ids = sorted(sec.lanes)
            for a, b in zip(ids[:-1], ids[1:]):
                # physically adjacent: consecutive ids on one side, or -1 / 1 across the centre line
                if not (b - a == 1 or (a == -1 and b == 1)):
                    continue
                # FLOW-0: the separating mark is the inner lane's roadMark (centre lane for -1 / 1);
                # laneChange increase = toward the higher id, decrease = toward the lower id
                mark = sec.center if (a == -1 and b == 1) else sec.lanes[b] if b < 0 else sec.lanes[a]
                if mark is None:
                    up = down = True
                else:
                    up, down = mark.allows(True), mark.allows(False)
                Lk = sec.e - sec.s
                if lane_max_width(sec.lanes[a], Lk) <= TOL_W or lane_max_width(sec.lanes[b], Lk) <= TOL_W:
                    continue
                for d in (1, -1):
                    na, nb = (r.id, k, a, d), (r.id, k, b, d)
                    if na in nodes and nb in nodes:
                        if up:
                            lat[na].add(nb), lat_in[nb].add(na)
                        if down:
                            lat[nb].add(na), lat_in[na].add(nb)
    # legal sources / sinks and dead ends
    bnd = {}

    def boundary(r, end):
        if (r.id, end) not in bnd:
            bnd[(r.id, end)] = end_boundary(net, oi, r, end, clip_xy)
        return bnd[(r.id, end)]

    sources, sinks, dead, unlinked = set(), set(), [], {}
    for n, ln in sorted(nodes.items(), key=lambda kv: (_idkey(kv[0][0]), kv[0][1:])):
        rid, k, lid, d = n
        r = net.roads[rid]
        sec = r.sections[k]
        L = sec.e - sec.s
        for kind, has, at_road_end, ds, end in (
                ("start", pred.get(n), (k == 0) if d == 1 else (k == len(r.sections) - 1), 0.0 if d == 1 else L,
                 "start" if d == 1 else "end"),
                ("end", succ.get(n), (k == len(r.sections) - 1) if d == 1 else (k == 0), L if d == 1 else 0.0,
                 "end" if d == 1 else "start")):
            if has:
                continue
            w = float(ln.width.ev(np.array([ds]), "left" if ds > 0 else "right")[0][0])
            if w <= TOL_W:
                continue   # zero-width (taper) end: reached / left laterally, neither source nor sink (FLOW-1 iii)
            s = sec.s + ds
            name = "flow_dead_start" if kind == "start" else "flow_dead_end"
            if at_road_end:
                bk, inside = boundary(r, end)
                if bk is not None:
                    (sources if kind == "start" else sinks).add(n)
                    if bk == "clip" and inside is not None and inside > TOL_POS:
                        bnd_short = ("flow_boundary_short", rid, end)
                        if bnd_short not in unlinked:
                            x, y = _xy(r, s)
                            unlinked[bnd_short] = (None, inst(rid, 0, s, x, y, inside, TOL_POS, end=end,
                                                               level="SHOULD", code="FLOW-1/boundary-short"))
                    continue
                lk = r.pred if end == "start" else r.succ
                if lk is None:
                    key = (name, rid, end)
                    if key not in unlinked:
                        x, y = _xy(r, s)
                        unlinked[key] = ([], inst(rid, 0, s, x, y, w, TOL_W, end=end, level="MUST", lanes="",
                                                  code="FLOW-1/" + ("entry" if kind == "start" else "exit") + "-unlinked",
                                                  junction=r.junction))
                    unlinked[key][0].append(n)
                    continue
                code = "FLOW-1/" + ("entry" if kind == "start" else "exit") + "-no-lane-link"
                level = "SHOULD" if kind == "start" else "MUST"
                link_s = f"{lk.type}:{lk.id}"
            else:
                code = "FLOW-1/abrupt-" + kind
                level, link_s = "MUST", "section"
            x, y = _xy(r, s)
            dead.append((name, n, inst(rid, lid, s, x, y, w, TOL_W, dir=d, code=code, level=level,
                                       road_link=link_s, junction=r.junction)))
    # reachability (statistics and context) on longitudinal + lateral edges
    adj = {n: succ.get(n, set()) | lat.get(n, set()) for n in nodes}
    radj = {n: pred.get(n, set()) | lat_in.get(n, set()) for n in nodes}

    def bfs(starts, g):
        seen, stack = set(starts), list(starts)
        while stack:
            for m in g.get(stack.pop(), ()):
                if m not in seen:
                    seen.add(m)
                    stack.append(m)
        return seen

    reach_f, reach_b = bfs(sources, adj), bfs(sinks, radj)
    ends = [(r.id, e) + _xy(r, end_s(r, e)) for r in net.roads.values() for e in ("start", "end")]
    EX = np.array([[e[2], e[3]] for e in ends]) if ends else np.zeros((0, 2))

    def nearest_end(it):
        if not len(EX):
            return
        d = np.hypot(EX[:, 0] - it["x"], EX[:, 1] - it["y"])
        for j in np.argsort(d, kind="stable")[:3]:
            if ends[j][0] != it["road"] and d[j] <= 20.0:
                it["nearest_road_end"] = f"{ends[j][0]}:{ends[j][1]}@{d[j]:.2f}m"
                return

    for name, n, it in dead:
        it["reachable"], it["can_exit"] = n in reach_f, n in reach_b
        if it["road_link"] != "section":
            nearest_end(it)
        res.add(name, it)
    for key, (ns, it) in sorted(unlinked.items(), key=lambda kv: (kv[0][0], _idkey(kv[0][1]), kv[0][2])):
        if ns is not None:
            it["lanes"] = ",".join(f"{n[2]}" for n in sorted(ns, key=lambda n: (n[2], n[1])))
            it["reachable"] = all(n in reach_f for n in ns)
            it["can_exit"] = all(n in reach_b for n in ns)
            nearest_end(it)
        res.add(key[0], it)
    # FLOW-4 root causes on the condensation
    order = sorted(nodes, key=lambda n: (_idkey(n[0]), n[1], n[2], n[3]))
    comp = _scc(order, adj)
    members = collections.defaultdict(list)
    for n in order:
        members[comp[n]].append(n)
    has_in, has_out = set(), set()
    parent = {c: c for c in members}

    def find(c):
        while parent[c] != c:
            parent[c] = parent[parent[c]]
            c = parent[c]
        return c

    for n in order:
        for m in adj[n]:
            a, b = comp[n], comp[m]
            if a != b:
                has_out.add(a), has_in.add(b)
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
    weak = collections.defaultdict(list)
    for c in members:
        weak[find(c)].append(c)
    src_c = {comp[n] for n in sources}
    snk_c = {comp[n] for n in sinks}

    def where(n, at_end):
        r = net.roads[n[0]]
        sec = r.sections[n[1]]
        s = (sec.e if n[3] == 1 else sec.s) if at_end else (sec.s if n[3] == 1 else sec.e)
        return s, _xy(r, s)

    for w, cs in sorted(weak.items()):
        if not any(c in src_c or c in snk_c for c in cs):
            # a closed component (no legal source or sink) is usable on its own only if a car can keep
            # driving in it: one SCC that contains a cycle (a single lane with no edge is not one)
            cyc = len(cs) == 1 and (len(members[cs[0]]) > 1 or any(m in adj[m] for m in members[cs[0]]))
            if not cyc:
                n = min((m for c in cs for m in members[c]), key=lambda n: (_idkey(n[0]), n[1:]))
                s, (x, y) = where(n, False)
                res.add("flow_closed_not_strong", inst(n[0], n[2], s, x, y, sum(len(members[c]) for c in cs), 0,
                                                       dir=n[3], sccs=len(cs), kind="not_strong" if len(cs) > 1
                                                       else "no_cycle", junction=net.roads[n[0]].junction))
            continue  # tested independently: a closed strongly connected component is usable
        for c in cs:
            mem = members[c]
            lanes = ",".join(f"{m[0]}:{m[2]}" for m in mem[:12]) + (",..." if len(mem) > 12 else "")
            if c not in has_in and c not in src_c:
                rep = [m for m in mem if not pred.get(m)] or mem
                n = rep[0]
                s, (x, y) = where(n, False)
                res.add("flow_orphan", inst(n[0], n[2], s, x, y, len(mem), 0, dir=n[3], lanes=lanes,
                                            junction=net.roads[n[0]].junction,
                                            unreachable_total=sum(1 for m in mem if m not in reach_f)))
            if c not in has_out and c not in snk_c:
                rep = [m for m in mem if not succ.get(m)] or mem
                n = rep[0]
                s, (x, y) = where(n, True)
                res.add("flow_trap", inst(n[0], n[2], s, x, y, len(mem), 0, dir=n[3], lanes=lanes,
                                          junction=net.roads[n[0]].junction))
    # FLOW-6 narrow: exact maximum width of every drivable lane chain of a road
    for r in net.roads.values():
        for k0, sec in enumerate(r.sections):
            for lid, ln in sorted(sec.lanes.items()):
                if not ln.drivable or (k0 > 0 and ln.pred is not None and ln.pred in r.sections[k0 - 1].lanes):
                    continue
                k, l, wmax, span = k0, ln, 0.0, 0.0
                while True:
                    Lk = r.sections[k].e - r.sections[k].s
                    st = list(l.width.st) + [max(Lk, l.width.st[-1])]
                    for j in range(len(l.width.st)):
                        x0, x1 = st[j], min(st[j + 1], Lk)
                        if x1 > x0:
                            wmax = max(wmax, max(-v for v, _ in _cubic_min(tuple(-l.width.C[j]), 0.0, x1 - x0)))
                    span += Lk
                    if l.succ is None or k + 1 >= len(r.sections) or l.succ not in r.sections[k + 1].lanes:
                        break
                    k, l = k + 1, r.sections[k + 1].lanes[l.succ]
                if TOL_W < wmax < W_P:
                    s = sec.s + span / 2
                    res.add("flow_narrow", inst(r.id, lid, s, *_xy(r, min(s, r.length)), wmax, W_P, type=ln.type,
                                                length=span, junction=r.junction))
    for r in net.roads.values():
        # a pedestrian road (a sidewalk round a junction corner, a crossing) is usable without
        # a drivable lane (ASAM Junction Guideline 6 / 10)
        pedestrian = all(l.type in ("sidewalk", "walking", "curb", "border")
                         for sec in r.sections for lid, l in sec.lanes.items() if lid != 0)
        if not pedestrian and not any(l.drivable for sec in r.sections for l in sec.lanes.values()):
            hw = {oi.tags(w).get("highway") for w in r.osm_ways} if oi else set()
            if not r.plain or hw & MOTOR_HIGHWAYS:
                res.add("flow_no_drivable", inst(r.id, 0, r.length / 2, *_xy(r, r.length / 2), 1, 0,
                                                 highway=",".join(sorted(h for h in hw if h)), junction=r.junction))
    nsplit = sum(1 for n in nodes if len(succ.get(n, ())) > 1)
    nmerge = sum(1 for n in nodes if len(pred.get(n, ())) > 1)
    res.info["flow"] = dict(lane_nodes=len(nodes), link_edges=sum(len(v) for v in succ.values()),
                            lane_change_edges=sum(len(v) for v in lat.values()), legit_starts=len(sources),
                            legit_ends=len(sinks), splits=nsplit, merges=nmerge,
                            reachable=len(reach_f), can_exit=len(reach_b),
                            unreachable_lanes=len(set(nodes) - reach_f), trapped_lanes=len(set(nodes) - reach_b),
                            unusable_lanes=len(set(nodes) - (reach_f & reach_b)), sccs=len(members))


# ============================================================================ driver
def run_all(net, osm=None, clip_xy=None, rule="file"):
    res = Result()
    links = lane_links(net, res, rule)
    check_links(net, res, links)
    check_roads(net, res)
    check_fold(net, res)
    oi = OsmIndex(osm, net) if osm is not None else None
    if oi is not None:
        samples = {r.id: RoadSamples(r) for r in net.plain_roads()}
        check_osm_fidelity(net, oi, res, samples)
        check_lane_count(net, oi, res, samples, rule)
    check_plain_links(net, oi, res, links)
    check_junctions(net, oi, res)
    check_flow(net, oi, res, links, rule, clip_xy)
    return res


def clip_from_gate(area):
    import gate
    a = gate.AREAS[area]
    if "clip" in a:
        return a["xodr"], a["osm"], a["clip"]
    args = a["args"]
    return a["xodr"], a["osm"], args[args.index("--clip") + 1] if "--clip" in args else None


def clip_xy_of(net, clip):
    if not clip:
        return None
    import odrgeom as G
    c = [float(v) for v in clip.split(",")]
    x0, y0 = G.tm_forward(min(c[0], c[2]), min(c[1], c[3]), net.lat0, net.lon0)
    x1, y1 = G.tm_forward(max(c[0], c[2]), max(c[1], c[3]), net.lat0, net.lon0)
    return float(x0), float(y0), float(x1), float(y1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xodr", nargs="?")
    ap.add_argument("osm", nargs="?")
    ap.add_argument("--area", help="reference area name from gate.py (sets xodr, osm, clip)")
    ap.add_argument("--clip", help="lat0,lon0,lat1,lon1 of the conversion clip box (network boundary)")
    ap.add_argument("--rule", default="file", choices=["file", "rht", "lht"],
                    help="lane travel direction: file = road@rule, RHT when absent (the OpenDRIVE default)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--crops", type=int, default=0, help="evidence crops per invariant (top instances)")
    ap.add_argument("--basemap", default=None)
    a = ap.parse_args()
    clip = a.clip
    if a.area:
        xodr, osm_path, gclip = clip_from_gate(a.area)
        a.xodr, a.osm, clip = a.xodr or xodr, a.osm or osm_path, clip or gclip
    import time
    t0 = time.time()
    net = Network(a.xodr)
    osm = None
    if a.osm:
        import odrgeom as G
        osm = G.Osm(a.osm, net.lat0, net.lon0)
    cxy = clip_xy_of(net, clip)
    res = run_all(net, osm, cxy, a.rule)
    os.makedirs(a.out, exist_ok=True)
    inv = res.to_json()
    summary = {k: v["count"] for k, v in inv.items()}
    meta = dict(xodr=a.xodr, osm=a.osm, clip=clip, rule=a.rule, openDRIVE=f"{net.rev[0]}.{net.rev[1]}",
                roads=len(net.roads), junctions=len(net.junctions), seconds=round(time.time() - t0, 1),
                tolerances=dict(pos_m=TOL_POS, hdg_rad=TOL_HDG, kappa_1pm=TOL_K, width_m=TOL_W, boundary_m=EPS_B,
                                W_P_m=W_P, min_len_m=MIN_LEN, aashto_lat_accel_step=LAT_JERK),
                g2_aashto={k: sum(1 for i in v["instances"] if i.get("aashto")) for k, v in inv.items()
                           if k.startswith("G2_")},
                **res.info)
    with open(os.path.join(a.out, "invariants.json"), "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "summary": summary, "invariants": inv}, f, indent=1, sort_keys=False)
    print(json.dumps({"meta": {k: meta[k] for k in ("roads", "junctions", "seconds", "rule")}, "flow": res.info.get("flow"),
                      "summary": summary}, indent=1))
    if a.crops:
        import odrgeom as G
        od = G.OpenDrive(a.xodr)
        for name, v in inv.items():
            for i, it in enumerate(v["instances"][:a.crops]):
                if it["x"] == 0 and it["y"] == 0:
                    continue
                p = os.path.join(a.out, "crops", f"{name}_{i}_r{it['road']}.png")
                G.crop(od, osm, (it["x"], it["y"]), 30, p, basemap=a.basemap, marks=[(it["x"], it["y"], name)],
                       title=f"{name} road {it['road']} lane {it['lane']} s={it['s']} value={it['value']}")


if __name__ == "__main__":
    main()
