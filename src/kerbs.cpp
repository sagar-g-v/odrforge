// OdrForge - carriageway edges that follow the mapped kerb.
//
// OSM maps the kerb where the carriageway ends (barrier=kerb, an open way); the lanes come from tags and default
// widths, so the drawn edge misses it (London, the six-area baseline: median 0.88 m inside the kerb, 13 % of kerb
// points within 0.3 m of the edge). Once a road's lanes are laid out, each side looks across to its kerb every kStep m
// and the edge is moved onto it:
// - the edge: the outer border of the outermost lane that is not sidewalk / walking / curb / border / none / median /
//   restricted (travelled lanes, parking and shoulders); on a side without such lanes, the lane-0 line;
// - the kerb: of the parallel kerb legs on the road's level crossing the sample's normal, the one nearest the edge within
//   kReach whose position leaves the lane that moves at least kMinLane wide - a closer one is an island's or another
//   road's kerb and is left out (user decision 2026-10-20); left out too: a kerb whose measured ground lies more than
//   kLevelStep from the edge's (another level: a cutting, a wall), and one the edge cannot reach without this side's
//   lanes (curb, sidewalk) running into another road's lanes (where they already do, the edge only moves in);
// - the kerb positions smoothed (moving median, then mean, over +- kWindow) where at least kCover of the samples there
//   saw one; elsewhere the edge stays where it was;
// - the change at knots every ~kKnot m and at every section joint, its slope limited to kMaxTaper, never taking the lane
//   that moves below kMinLane (unless it was) nor, without a shoulder, past kMaxLane (user decision 2026-10-20: wider is a
//   shoulder - kerbShoulders adds one, width 0, where the kerb lies that far out), 0 at a joint where the lane that moves
//   is not the same lane on both sides and over a section without one; between knots C2 ramps level at each knot
//   (Poly3::shiftC2: lane edges stay G2, OpenDRIVE 1.9 Annex D.1.3, and never overshoot): added to the width records of
//   the outermost driving / bus / bidirectional lane (a cycle lane where none is) that neither grows nor ends in its
//   section (OpenDRIVE 1.8.0 §11.6 lane width: several records per lane), and on a side without lanes to the lane offset
//   (§11.4), the other side's lane making up for the shift.
// It runs after resolveOverlaps: roads drawn too close are pushed apart from their tagged widths, then the kerbs decide.

#include <algorithm>
#include <cmath>
#include <limits>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
constexpr double kStep = 2.0;       // m between samples along the road
constexpr double kEndClear = 4.0;   // m: no samples nearer a road end (the kerb turns round the corner there)
constexpr double kReach = 10.0;     // m: a kerb farther beyond (or inside) the edge is not this road's
constexpr double kMinLane = 2.0;    // m: narrowest the lane that moves gets (closer kerbs are left out)
constexpr double kMaxLane = 5.5;    // m: widest it gets
// smoothing measured on London (2026-10-20): a 5 m window with a 0.1 m/m taper reached 60.8 % of kerb points within
// 0.3 m but added a fold; 5 m / 0.15 / 3 m knots / 2 m end clearance 63.7 % with a validation error and 5 more folds
constexpr double kWindow = 10.0;    // m: +- along the road, smoothing the kerb positions
constexpr double kCover = 0.4;      // share of the samples within the window that must see the kerb
constexpr double kKnot = 5.0;       // m between knots (about)
constexpr double kMaxTaper = 0.05;  // m per m: the steepest the edge moves along the road
// m: a kerb whose measured ground lies farther above or below the ground at the edge stands on another level - a
// cutting, a wall, an embankment beside the road (London Kingsway road 216: the median kerb by the tram subway cutting;
// lanes widened over it floated 1.1 - 1.6 m over the conformed ground)
constexpr double kLevelStep = 0.3;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool carriageway(const odr::Lane &l) {
  return l.type != "sidewalk" && l.type != "walking" && l.type != "curb" && l.type != "border" && l.type != "none" &&
         l.type != "median" && l.type != "restricted";
}
// a lane that may take the change: 2 for motor traffic, 1 for a cycle lane (only where no motor lane is), 0 none
int moves(const odr::Lane &l) {
  if (l.type == "driving" || l.type == "bus" || l.type == "bidirectional") return 2;
  return l.type == "biking" ? 1 : 0;
}

// the polynomial re-expanded ds along it
odr::Poly3 shifted(const odr::Poly3 &p, double ds) {
  return {p.s + ds, p.eval(ds), p.deriv(ds), p.c + 3.0 * p.d * ds, p.d};
}

// a piecewise cubic: piece j on [x_j, x_j+1), its s = x_j (absolute)
struct Spline {
  std::vector<double> x;
  std::vector<odr::Poly3> pieces;
  size_t at(double s) const {
    const auto it = std::upper_bound(x.begin(), x.end() - 1, s);
    return it == x.begin() ? 0 : std::min(pieces.size() - 1, static_cast<size_t>(it - x.begin()) - 1);
  }
  double eval(double s) const {
    const odr::Poly3 &p = pieces[at(s)];
    return p.eval(s - p.s);
  }
};

// a C2 function through (x_j, y_j) level at every knot (Poly3::shiftC2 from knot to knot): between two knots it stays
// within their values, so a width made of it never goes below its knots' smallest
Spline levelRamps(const std::vector<double> &x, const std::vector<double> &y) {
  Spline f;
  for (size_t j = 0; j + 1 < x.size(); ++j)
    for (const odr::Poly3 &p : odr::Poly3::shiftC2(x[j], x[j + 1] - x[j], y[j], y[j + 1])) f.x.push_back(p.s), f.pieces.push_back(p);
  f.x.push_back(x.back());
  return f;
}

// of the kerb positions `ts` (signed t) on one side (`sign`) of a sample, the nearest to the edge within kReach that
// leaves the lane taking the change (width `w`) at least kMinLane wide; NaN: none (`seen`: some kerb there was left out)
double pickKerb(const std::vector<double> &ts, double edge, double w, double sign, bool &seen) {
  double best = std::numeric_limits<double>::quiet_NaN();
  seen = false;
  for (const double t : ts) {
    const double out = sign * (t - edge);
    if (std::abs(out) > kReach || sign * t <= 0.0) continue;
    seen = true;
    if (w + out < kMinLane) continue;  // an island's or another road's kerb
    if (std::isnan(best) || std::abs(out) < std::abs(sign * (best - edge))) best = t;
  }
  return best;
}

// records (each record's s relative to `base`) over [from, to) plus `sign` f: the records over [from, to) re-cut at f's
// knots, each the sum of the record and f's piece there
void addSpline(std::vector<odr::Poly3> &recs, double base, double from, double to, const Spline &f, double sign) {
  std::vector<double> cuts{from};
  for (const odr::Poly3 &p : recs)
    if (base + p.s > from + 1e-6 && base + p.s < to - 1e-6) cuts.push_back(base + p.s);
  for (double k : f.x)
    if (k > from + 1e-6 && k < to - 1e-6) cuts.push_back(k);
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return b - a < 1e-6; }), cuts.end());
  std::vector<odr::Poly3> out;
  for (const double u : cuts) {
    const odr::Poly3 *rec = &recs.front();
    for (const odr::Poly3 &p : recs)
      if (base + p.s <= u + 1e-6) rec = &p;
    const odr::Poly3 a = shifted(*rec, u - (base + rec->s)), b = shifted(f.pieces[f.at(u + 1e-9)], u - f.pieces[f.at(u + 1e-9)].s);
    out.push_back({u - base, a.a + sign * b.a, a.b + sign * b.b, a.c + sign * b.c, a.d + sign * b.d});
  }
  recs = std::move(out);
}

// the lane of a section side that takes the change: the outermost motor lane (else cycle lane) that neither grows nor
// ends in the section - a lane that tapers keeps its taper; none: the side takes no change in this section
const odr::Lane *mover(const std::vector<odr::Lane> &side, double secLen) {
  const odr::Lane *best = nullptr;
  for (const odr::Lane &l : side) {
    if (!moves(l) || l.widths.empty()) continue;
    bool steady = true;
    for (int i = 0; i <= 4 && steady; ++i) {
      const double u = secLen * i / 4.0;
      const odr::Poly3 *rec = &l.widths.front();
      for (const odr::Poly3 &p : l.widths)
        if (p.s <= u + 1e-9) rec = &p;
      steady = rec->eval(u - rec->s) > 0.5;
    }
    if (steady && (!best || moves(l) > moves(*best) || (moves(l) == moves(*best) && std::abs(l.id) > std::abs(best->id))))
      best = &l;
  }
  return best;
}

double widthOf(const odr::Lane &l, double u) {
  const odr::Poly3 *rec = &l.widths.front();
  for (const odr::Poly3 &p : l.widths)
    if (p.s <= u + 1e-9) rec = &p;
  return rec->eval(u - rec->s);
}

// lane `id` out of section k: the lanes outside it move in by one, and the links of the neighbouring sections with them
void removeLane(odr::Road &r, size_t k, int id) {
  const int sg = id > 0 ? 1 : -1;
  std::vector<odr::Lane> &side = id > 0 ? r.lanes[k].left : r.lanes[k].right;
  side.erase(std::remove_if(side.begin(), side.end(), [&](const odr::Lane &l) { return l.id == id; }), side.end());
  for (odr::Lane &l : side)
    if (std::abs(l.id) > std::abs(id)) l.id -= sg;
  auto fix = [&](std::optional<int> &ref) {
    if (!ref || (*ref > 0) != (id > 0)) return;
    if (*ref == id) ref.reset();
    else if (std::abs(*ref) > std::abs(id)) *ref -= sg;
  };
  if (k > 0)
    for (auto *s : {&r.lanes[k - 1].left, &r.lanes[k - 1].right})
      for (odr::Lane &l : *s) fix(l.link.successor);
  if (k + 1 < r.lanes.size())
    for (auto *s : {&r.lanes[k + 1].left, &r.lanes[k + 1].right})
      for (odr::Lane &l : *s) fix(l.link.predecessor);
}

bool islandArea(const Tags &t) {
  return tagIs(t, "area:highway", "traffic_island") || tagIs(t, "area:highway", "pedestrian") ||
         tagIs(t, "footway", "traffic_island") || (tagIs(t, "highway", "pedestrian") && tagYes(t, "area"));
}

double segDistance(const Vec2 &p, const Vec2 &a, const Vec2 &b) {
  const Vec2 d = b - a;
  const double L2 = d.dot(d), u = L2 > 0.0 ? std::clamp((p - a).dot(d) / L2, 0.0, 1.0) : 0.0;
  return (p - (a + d * u)).norm();
}
}  // namespace

// Open barrier=kerb ways, as legs on their level. A kerb with half or more of its points (every 1 m) inside or within
// 0.5 m of a traffic island or a pedestrian area (area:highway=traffic_island|pedestrian, footway=traffic_island,
// highway=pedestrian + area=yes) rings that area, not a carriageway: left out.
void Builder::collectKerbs() {
  if (!opt.followKerbs) return;
  auto xy = [&](OsmId id, Vec2 &p) {
    const OsmNode *n = osm.node(id);
    if (!n) return false;
    p = proj.forward(n->lat, n->lon);
    return true;
  };
  struct Island {
    std::vector<Vec2> ring;
    Vec2 lo, hi;
  };
  std::vector<Island> islands;
  for (const auto &[id, w] : osm.ways) {
    if (!islandArea(w.tags) || w.nodes.size() < 4 || w.nodes.front() != w.nodes.back()) continue;
    Island is{{}, {1e18, 1e18}, {-1e18, -1e18}};
    for (OsmId n : w.nodes) {
      Vec2 p;
      if (!xy(n, p)) break;
      is.ring.push_back(p);
      is.lo = {std::min(is.lo.x, p.x), std::min(is.lo.y, p.y)}, is.hi = {std::max(is.hi.x, p.x), std::max(is.hi.y, p.y)};
    }
    if (is.ring.size() == w.nodes.size()) islands.push_back(std::move(is));
  }
  auto nearIsland = [&](const Vec2 &p) {
    for (const Island &is : islands) {
      if (p.x < is.lo.x - 0.5 || p.x > is.hi.x + 0.5 || p.y < is.lo.y - 0.5 || p.y > is.hi.y + 0.5) continue;
      bool inside = false;
      for (size_t i = 0, j = is.ring.size() - 1; i < is.ring.size(); j = i++) {
        const Vec2 &a = is.ring[i], &b = is.ring[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
        if (segDistance(p, a, b) <= 0.5) return true;
      }
      if (inside) return true;
    }
    return false;
  };
  for (const auto &[id, w] : osm.ways) {
    if (!tagIs(w.tags, "barrier", "kerb") || w.nodes.size() < 2 || w.nodes.front() == w.nodes.back()) continue;
    std::vector<Vec2> pts;
    for (OsmId n : w.nodes) {
      Vec2 p;
      if (!xy(n, p)) break;
      pts.push_back(p);
    }
    if (pts.size() != w.nodes.size()) continue;
    int all = 0, near = 0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      const double L = (pts[i + 1] - pts[i]).norm();
      for (double u = 0.5; u < L; u += 1.0, ++all) near += nearIsland(pts[i] + (pts[i + 1] - pts[i]) * (u / L));
    }
    if (all == 0 || 2 * near >= all) continue;
    const int level = wayLevel(w.tags);
    for (size_t i = 0; i + 1 < pts.size(); ++i)
      if ((pts[i + 1] - pts[i]).norm() >= 0.2) kerbs.add({pts[i], pts[i + 1], 0.0, level, true});
  }
}

bool Builder::kerbOnLevel(const Pose &p, double edge, double t) const {
  if (!opt.terrain || opt.terrain->ground.empty()) return true;
  const Vec2 a = p.offset(edge), b = p.offset(t);
  const double ga = opt.terrain->ground_at(a.x, a.y), gb = opt.terrain->ground_at(b.x, b.y);
  return std::isnan(ga) || std::isnan(gb) || std::abs(ga - gb) <= kLevelStep;
}

// Before the lanes are laid out: a side whose kerb lies farther out than the lane that moves can reach (kMaxLane) gets a
// shoulder of width 0 in every section, just outside its travelled and parking lanes; followKerbs widens it (user
// decision 2026-10-20: the space beyond a 5.5 m lane is a shoulder). Read from the sections' own widths (no blends) -
// a side needs kMinRun such samples within kWindow of each other.
void Builder::kerbShoulders(const RoadMeta &m, const ReferenceLine &line, double len, std::vector<SectionInfo> &secs) const {
  constexpr int kMinRun = 3;
  if (!opt.followKerbs || kerbs.empty() || secs.empty() || len < 2.0 * kEndClear + 2.0 * kStep) return;
  auto isCarriageway = [](LaneKind k) { return k != LaneKind::Sidewalk && k != LaneKind::Border; };
  auto rank = [](LaneKind k) {  // as moves(): motor lanes first, a cycle lane only where no motor lane is
    return k == LaneKind::Driving || k == LaneKind::Bus || k == LaneKind::Bidirectional ? 2 : k == LaneKind::Biking ? 1 : 0;
  };
  std::vector<double> beyond[2];  // s of the samples whose kerb lies beyond what the lane can reach
  for (double s = kEndClear; s <= len - kEndClear + 1e-9; s += kStep) {
    size_t k = 0;
    while (k + 1 < secs.size() && secs[k + 1].s <= s + 1e-9) ++k;
    const CrossSection &x = secs[k].xs;
    const Pose p = line.eval(s);
    std::vector<double> ts;
    for (int sd = 0; sd < 2; ++sd) {
      const SideSpec &side = sd == 0 ? x.left : x.right;
      const double sign = sd == 0 ? 1.0 : -1.0;
      double reach = 0.0, edge = 0.0, w = kNaN;
      int best = 0;
      for (const LaneSpec &l : side.lanes) {
        if (!isCarriageway(l.kind)) break;
        reach += l.width;
        if (rank(l.kind) > 0 && rank(l.kind) >= best) w = l.width, best = rank(l.kind);
      }
      if (std::isnan(w)) continue;
      edge = x.laneOffset + sign * reach;
      if (ts.empty()) ts = kerbs.across({p.pos(), p.dir(), layerAt(m, s)}, std::abs(x.laneOffset) + reach + 2.0 * kReach);
      std::vector<double> level;
      for (const double t : ts)
        if (kerbOnLevel(p, edge, t)) level.push_back(t);
      bool seen = false;
      const double t = pickKerb(level, edge, w, sign, seen);
      if (!std::isnan(t) && sign * (t - edge) > kMaxLane - w) beyond[sd].push_back(s);
    }
  }
  // per section: kMinRun such samples within it, within 2 kWindow of each other
  for (int sd = 0; sd < 2; ++sd)
    for (size_t k = 0; k < secs.size(); ++k) {
      const double s0 = secs[k].s, s1 = k + 1 < secs.size() ? secs[k + 1].s : len;
      std::vector<double> in;
      for (double s : beyond[sd])
        if (s >= s0 && s < s1) in.push_back(s);
      bool need = false;
      for (size_t i = 0; i + kMinRun - 1 < in.size() && !need; ++i) need = in[i + kMinRun - 1] - in[i] <= 2.0 * kWindow;
      if (!need) continue;
      std::vector<LaneSpec> &lanes = (sd == 0 ? secs[k].xs.left : secs[k].xs.right).lanes;
      size_t at = 0;  // after the last travelled / parking / shoulder lane
      bool hasShoulder = false;
      for (size_t i = 0; i < lanes.size() && isCarriageway(lanes[i].kind); ++i) at = i + 1, hasShoulder = lanes[i].kind == LaneKind::Shoulder;
      if (!hasShoulder) lanes.insert(lanes.begin() + static_cast<long>(at), LaneSpec{LaneKind::Shoulder, 0.0, {}});
    }
}

// After resolveOverlaps: a pair of roads drawn too close is pushed apart from their tagged widths first; then each side
// with a kerb is anchored on it (measured geometry wins over the shift - it moved the drawing, the kerb is where the edge is)
void Builder::kerbEdges() {
  if (!opt.followKerbs || kerbs.empty()) return;
  // every road's lanes as laid out (all of them, sidewalks too), every kStep m: a kerb lying beyond another road's lanes
  // is that road's, and an edge does not move out into them (London Kingsway road 216 reached over the Strand Underpass
  // ramp in its median, road 215, to the ramp's kerb; the ramp, widened to its own kerb, lowered the ground under 216's
  // sidewalk by 1.3 m)
  for (const RoadMeta &m : metas) {
    if (m.roadIndex < 0) continue;
    const odr::Road &r = od.roads[m.roadIndex];
    const double len = r.planView.length();
    for (double s = 0.0; s < len - 1e-6; s += kStep) {
      const double s1 = std::min(len, s + kStep), sm = 0.5 * (s + s1);
      double lo = odr::laneOffsetAt(r, sm), hi = lo;
      const odr::LaneSection &sec = odr::sectionAt(r, sm);
      for (const auto *side : {&sec.left, &sec.right})
        for (const odr::Lane &l : *side) {
          double a, b;
          if (odr::laneBorders(r, sm, l.id, a, b)) lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
        }
      SidewalkIndex::Leg g{r.planView.eval(s).pos(), r.planView.eval(s1).pos(), 0.0, 0, false};
      g.road = m.roadIndex, g.reachLeft = hi, g.reachRight = -lo;
      carriageways.add(g);
    }
  }
  for (const RoadMeta &m : metas)
    if (m.roadIndex >= 0) followKerbs(m, od.roads[m.roadIndex]);
}

void Builder::followKerbs(const RoadMeta &m, odr::Road &r) {
  if (!opt.followKerbs || kerbs.empty() || r.lanes.empty()) return;
  const double len = r.planView.length();
  if (len < 2.0 * kEndClear + 2.0 * kStep) return;
  std::vector<double> ss;
  for (double s = kEndClear; s <= len - kEndClear + 1e-9; s += kStep) ss.push_back(s);
  const size_t n = ss.size();
  auto secEnd = [&](size_t k) { return k + 1 < r.lanes.size() ? r.lanes[k + 1].s : len; };
  auto secIndex = [&](double s) {
    size_t k = 0;
    while (k + 1 < r.lanes.size() && r.lanes[k + 1].s <= s + 1e-9) ++k;
    return k;
  };
  // per side (0 left, 1 right) and sample: the edge, how far the lanes outside it (curb, sidewalk) reach beyond it, the
  // lane that moves (its width; NaN: none) and the kerb
  struct Side {
    std::vector<double> edge, beyond, width, kerb;
    bool laneless = true;  // no travelled, parking or shoulder lane anywhere on this side
  };
  Side sides[2];
  for (Side &d : sides) d.edge.assign(n, kNaN), d.beyond.assign(n, 0.0), d.width.assign(n, kNaN), d.kerb.assign(n, kNaN);
  for (size_t i = 0; i < n; ++i) {
    const size_t k = secIndex(ss[i]);
    const odr::LaneSection &sec = r.lanes[k];
    for (int sd = 0; sd < 2; ++sd) {
      const std::vector<odr::Lane> &lanes = sd == 0 ? sec.left : sec.right;
      double edge = odr::laneOffsetAt(r, ss[i]);
      int outer = 0;
      for (const odr::Lane &l : lanes)
        if (carriageway(l) && std::abs(l.id) > std::abs(outer)) outer = l.id;
      double in, out;
      if (outer != 0 && odr::laneBorders(r, ss[i], outer, in, out)) edge = out, sides[sd].laneless = false;
      sides[sd].edge[i] = edge;
      for (const odr::Lane &l : lanes)
        if (odr::laneBorders(r, ss[i], l.id, in, out))
          sides[sd].beyond[i] = std::max(sides[sd].beyond[i], (sd == 0 ? 1.0 : -1.0) * (out - edge));
      if (const odr::Lane *mv = mover(lanes, secEnd(k) - sec.s)) sides[sd].width[i] = widthOf(*mv, ss[i] - sec.s);
    }
  }
  // a kerb at t stands on the edge's level: the measured ground there and at the edge within kLevelStep (no terrain:
  // every kerb does)
  auto onLevel = [&](const Pose &p, double edge, double t) { return kerbOnLevel(p, edge, t); };
  // a road with no lane that could move on either side follows nothing
  bool anyMover = false;
  for (const Side &d : sides)
    for (double w : d.width) anyMover |= !std::isnan(w);
  if (!anyMover) return;
  for (size_t i = 0; i < n; ++i) {
    const Pose p = r.planView.eval(ss[i]);
    const SidewalkIndex::Sample q{p.pos(), p.dir(), layerAt(m, ss[i])};
    const double far = std::max(std::abs(sides[0].edge[i]), std::abs(sides[1].edge[i])) + kReach;
    const std::vector<double> ts = kerbs.across(q, far);
    const auto others = carriageways.carriageways(q, far + 10.0, m.roadIndex);
    for (int sd = 0; sd < 2; ++sd) {
      Side &d = sides[sd], &o = sides[1 - sd];
      const double sign = sd == 0 ? 1.0 : -1.0;
      // how much the edge may come in: the lane that moves keeps kMinLane (on a side without lanes, the other side's)
      const double w = d.laneless ? o.width[i] : d.width[i];
      if (std::isnan(w)) continue;
      // how far out the edge may go: the lanes beyond it (curb, sidewalk) stay short of the nearest other carriageway on
      // this side; where one already reaches over them, not out at all (only in)
      double block = sign * std::numeric_limits<double>::infinity();
      for (const auto &[a, b] : others) {
        const double nearSide = sign > 0 ? a : b, farSide = sign > 0 ? b : a;
        if (sign * (farSide - d.edge[i]) <= 0.0) continue;  // behind this edge
        double lim = nearSide - sign * d.beyond[i];
        if (sign * (lim - d.edge[i]) < 0.0) lim = d.edge[i];
        if (sign * (lim - block) < 0.0) block = lim;
      }
      std::vector<double> level;  // the kerbs on the edge's level, short of another road's carriageway
      for (const double t : ts)
        if (onLevel(p, d.edge[i], t) && sign * (t - block) <= 0.0) level.push_back(t);
      bool seen = false;
      const double best = pickKerb(level, d.edge[i], w, sign, seen);
      d.kerb[i] = best;
      if (!std::isnan(best)) ++rep.kerbSamples;
      else if (seen) ++rep.kerbSamplesIgnored;
      else if (level.size() < ts.size() && !std::isnan(pickKerb(ts, d.edge[i], w, sign, seen))) ++rep.kerbSamplesOffLevel;
    }
  }
  // knots, about kKnot apart over the whole road, and at every section joint
  std::vector<double> kx;
  {
    const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::lround(len / kKnot)));
    for (size_t j = 0; j <= steps; ++j) kx.push_back(len * static_cast<double>(j) / static_cast<double>(steps));
    for (size_t k = 1; k < r.lanes.size(); ++k) kx.push_back(r.lanes[k].s);
    std::sort(kx.begin(), kx.end());
    kx.erase(std::unique(kx.begin(), kx.end(), [](double a, double b) { return b - a < 0.5; }), kx.end());
    for (size_t k = 1; k < r.lanes.size(); ++k)  // a joint replaces the regular knot it fell next to
      *std::min_element(kx.begin(), kx.end(), [&](double a, double b) {
        return std::abs(a - r.lanes[k].s) < std::abs(b - r.lanes[k].s);
      }) = r.lanes[k].s;
    kx.front() = 0.0, kx.back() = len;
  }
  const size_t nk = kx.size() - 1;
  // per side, the knots where its change must be 0: at a joint where the lane taking the change is not the same lane
  // on both sides of it (inner lane borders would step there), and across a section without such a lane
  auto pinned = [&](int sd) {
    std::vector<bool> pin(nk + 1, false);
    for (size_t k = 0; k < r.lanes.size(); ++k) {
      const auto &lanes = sd == 0 ? r.lanes[k].left : r.lanes[k].right;
      const odr::Lane *mv = mover(lanes, secEnd(k) - r.lanes[k].s);
      bool cont = true;  // the same lane takes the change across the joint at this section's start
      if (k > 0) {
        const odr::Lane *pv = mover(sd == 0 ? r.lanes[k - 1].left : r.lanes[k - 1].right, r.lanes[k].s - r.lanes[k - 1].s);
        cont = mv && pv && pv->link.successor && *pv->link.successor == mv->id;
      }
      for (size_t j = 0; j <= nk; ++j) {
        const bool atJoint = k > 0 && std::abs(kx[j] - r.lanes[k].s) < 1e-6;
        const bool inside = kx[j] >= r.lanes[k].s - 1e-6 && kx[j] <= secEnd(k) + 1e-6;
        if ((atJoint && !cont) || (!mv && inside)) pin[j] = true;
      }
    }
    return pin;
  };
  // values pinned to 0 there, and coming down to 0 towards them no steeper than kMaxTaper
  auto pinZeros = [&](std::vector<double> &y, const std::vector<bool> &pin) {
    for (size_t j = 0; j <= nk; ++j) {
      double env = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i <= nk; ++i)
        if (pin[i]) env = std::min(env, kMaxTaper * std::abs(kx[j] - kx[i]));
      y[j] = std::clamp(y[j], -env, env);
    }
  };
  // the outward change of one side's edge wanted at the samples (0 where it saw no kerb), at the knots
  auto wanted = [&](const Side &d, double sign, const std::vector<double> &edgeNow) {
    std::vector<double> fit(n, kNaN);
    for (size_t i = 0; i < n; ++i) {
      std::vector<double> win;
      size_t all = 0;
      for (size_t j = 0; j < n; ++j)
        if (std::abs(ss[j] - ss[i]) <= kWindow) ++all, std::isnan(d.kerb[j]) ? void() : win.push_back(d.kerb[j]);
      if (win.size() < 3 || win.size() < kCover * static_cast<double>(all)) continue;
      std::nth_element(win.begin(), win.begin() + win.size() / 2, win.end());
      fit[i] = win[win.size() / 2];
    }
    std::vector<double> delta(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
      if (std::isnan(fit[i])) {  // no kerb here: the edge goes back to where it was laid out
        delta[i] = sign * (d.edge[i] - edgeNow[i]);
        continue;
      }
      double sum = 0.0;
      int cnt = 0;
      for (size_t j = 0; j < n; ++j)
        if (std::abs(ss[j] - ss[i]) <= kWindow && !std::isnan(fit[j])) sum += fit[j], ++cnt;
      delta[i] = sign * (sum / cnt - edgeNow[i]);
    }
    std::vector<double> y(nk + 1);
    for (size_t j = 0; j <= nk; ++j) {  // linear between samples, held beyond the first / last
      const auto it = std::lower_bound(ss.begin(), ss.end(), kx[j]);
      if (it == ss.begin()) y[j] = delta.front();
      else if (it == ss.end()) y[j] = delta.back();
      else {
        const size_t b = static_cast<size_t>(it - ss.begin()), a = b - 1;
        y[j] = delta[a] + (delta[b] - delta[a]) * (kx[j] - ss[a]) / (ss[b] - ss[a]);
      }
    }
    for (size_t j = 1; j <= nk; ++j) {  // the slope limit, both ways
      const double dl = kMaxTaper * (kx[j] - kx[j - 1]);
      y[j] = std::clamp(y[j], y[j - 1] - dl, y[j - 1] + dl);
    }
    for (size_t j = nk; j-- > 0;) {
      const double dl = kMaxTaper * (kx[j + 1] - kx[j]);
      y[j] = std::clamp(y[j], y[j + 1] - dl, y[j + 1] + dl);
    }
    return y;
  };
  // the lane that moves at s: its width there (NaN: none)
  auto widthAt = [&](int sd, double s) {
    const size_t k = secIndex(s);
    const odr::Lane *mv = mover(sd == 0 ? r.lanes[k].left : r.lanes[k].right, secEnd(k) - r.lanes[k].s);
    return mv ? widthOf(*mv, s - r.lanes[k].s) : kNaN;
  };
  auto any = [](const std::vector<double> &y) {
    return std::any_of(y.begin(), y.end(), [](double v) { return std::abs(v) > 1e-4; });
  };
  const std::vector<double> edge0[2] = {sides[0].edge, sides[1].edge};
  // 1. a side without lanes: the lane offset (the lanes of the other side move along with it)
  for (int sd = 0; sd < 2; ++sd) {
    Side &d = sides[sd];
    if (!d.laneless || sides[1 - sd].laneless) continue;
    if (std::all_of(d.kerb.begin(), d.kerb.end(), [](double v) { return std::isnan(v); })) continue;
    std::vector<double> y = wanted(d, sd == 0 ? 1.0 : -1.0, d.edge);
    for (size_t j = 0; j <= nk; ++j)  // the other side's lane takes the shift, kept within kMinLane .. kMaxLane
      if (const double w = widthAt(1 - sd, kx[j]); !std::isnan(w))
        y[j] = std::clamp(y[j], std::min(0.0, kMinLane - w), std::max(0.0, kMaxLane - w));
      else y[j] = 0.0;
    pinZeros(y, pinned(1 - sd));
    if (!any(y)) continue;
    const Spline f = levelRamps(kx, y);
    addSpline(r.laneOffset, 0.0, 0.0, len, f, sd == 0 ? 1.0 : -1.0);
    for (size_t i = 0; i < n; ++i) {  // both edges moved with lane 0
      const double v = (sd == 0 ? 1.0 : -1.0) * f.eval(ss[i]);
      sides[0].edge[i] += v, sides[1].edge[i] += v;
    }
    ++rep.kerbSides;
  }
  // 2. a side with lanes: the width of its lane that moves (back to its laid-out edge where it saw no kerb, so a shift
  //    of lane 0 above is made up for)
  for (int sd = 0; sd < 2; ++sd) {
    Side &d = sides[sd];
    if (d.laneless) continue;
    Side keep = d;
    for (size_t i = 0; i < n; ++i) keep.edge[i] = edge0[sd][i];
    std::vector<double> y = wanted(keep, sd == 0 ? 1.0 : -1.0, d.edge);
    // a shoulder outside the lane that moves (kerbShoulders, in the sections that need one) takes what the lane cannot
    // (beyond kMaxLane); elsewhere the change stops there. At a joint, only where both sections have one
    auto shoulderOf = [&](size_t k) -> odr::Lane * {
      std::vector<odr::Lane> &lanes = sd == 0 ? r.lanes[k].left : r.lanes[k].right;
      odr::Lane *sh = nullptr;
      for (odr::Lane &l : lanes)
        if (l.type == "shoulder" && (!sh || std::abs(l.id) > std::abs(sh->id))) sh = &l;
      return sh;
    };
    auto shoulderAt = [&](double s) {
      const size_t k = secIndex(s);
      if (!shoulderOf(k)) return false;
      return !(k > 0 && std::abs(s - r.lanes[k].s) < 1e-6 && !shoulderOf(k - 1));
    };
    std::vector<double> sh(nk + 1, 0.0);
    for (size_t j = 0; j <= nk; ++j) {
      const double w = widthAt(sd, kx[j]);
      if (std::isnan(w)) {
        y[j] = 0.0;
        continue;
      }
      y[j] = std::max(y[j], std::min(0.0, kMinLane - w));  // never narrower than kMinLane (unless it was)
      if (!shoulderAt(kx[j])) y[j] = std::min(y[j], std::max(0.0, kMaxLane - w));
    }
    pinZeros(y, pinned(sd));
    for (size_t j = 0; j <= nk; ++j)
      if (const double w = widthAt(sd, kx[j]); shoulderAt(kx[j]) && !std::isnan(w))
        sh[j] = std::max(0.0, y[j] - std::max(0.0, kMaxLane - w));    if (!any(y)) continue;
    const Spline f = levelRamps(kx, y);
    const bool split = any(sh);
    const Spline g = split ? levelRamps(kx, sh) : Spline{};
    for (size_t k = 0; k < r.lanes.size(); ++k) {
      std::vector<odr::Lane> &lanes = sd == 0 ? r.lanes[k].left : r.lanes[k].right;
      const odr::Lane *mv = mover(lanes, secEnd(k) - r.lanes[k].s);
      if (!mv) continue;
      odr::Lane &l = *std::find_if(lanes.begin(), lanes.end(), [&](const odr::Lane &x) { return x.id == mv->id; });
      addSpline(l.widths, r.lanes[k].s, r.lanes[k].s, secEnd(k), f, 1.0);
      odr::Lane *s = shoulderOf(k);
      if (!split || !s) continue;
      addSpline(l.widths, r.lanes[k].s, r.lanes[k].s, secEnd(k), g, -1.0);
      addSpline(s->widths, r.lanes[k].s, r.lanes[k].s, secEnd(k), g, 1.0);
    }
    if (std::any_of(d.kerb.begin(), d.kerb.end(), [](double v) { return !std::isnan(v); })) ++rep.kerbSides;
  }
  // a shoulder kerbShoulders added that took nothing (zero along its whole section) is removed
  for (size_t k = 0; k < r.lanes.size(); ++k)
    for (int sd = 0; sd < 2; ++sd) {
      const std::vector<odr::Lane> &lanes = sd == 0 ? r.lanes[k].left : r.lanes[k].right;
      for (const odr::Lane &l : lanes) {
        if (l.type != "shoulder") continue;
        bool zero = true;
        for (int i = 0; i <= 20 && zero; ++i) zero = std::abs(widthOf(l, (secEnd(k) - r.lanes[k].s) * i / 20.0)) < 1e-6;
        if (zero) {
          removeLane(r, k, l.id);
          break;
        }
      }
    }
  // a shoulder with zero width at a section joint is not linked across it (OpenDRIVE 1.8.0 §11.5; validate.cpp rules
  // road.lane.link.zero_width_at_start / _end): where the kerb lies within reach of the lanes it has none there
  for (size_t k = 1; k < r.lanes.size(); ++k)
    for (int sd = 0; sd < 2; ++sd) {
      std::vector<odr::Lane> &prev = sd == 0 ? r.lanes[k - 1].left : r.lanes[k - 1].right;
      for (odr::Lane &l : sd == 0 ? r.lanes[k].left : r.lanes[k].right) {
        if (l.type != "shoulder" || !l.link.predecessor) continue;
        const auto p = std::find_if(prev.begin(), prev.end(), [&](const odr::Lane &q) { return q.id == *l.link.predecessor; });
        if (p == prev.end()) continue;
        if (widthOf(l, 0.0) < 1e-6 || widthOf(*p, r.lanes[k].s - r.lanes[k - 1].s) < 1e-6) l.link.predecessor.reset(), p->link.successor.reset();
      }
    }
}

}  // namespace detail
}  // namespace odrforge
