// OdrForge - sidewalks mapped as their own ways.
//
// Many cities map the sidewalk as a separate footway (footway=sidewalk) next to the road, the road
// itself tagged sidewalk=separate or not at all. A cross-section built from the road's tags alone then
// ends at the outermost lane, metres short of the curb, and the space between is nothing. Here the
// separate sidewalk is measured along each road segment and the segment's cross-section reaches out to
// it: the lanes, a border (verge) to the sidewalk's inner edge, the sidewalk at its mapped position.
//
// Measurement: samples every 4 m along the segment (not within 6 m of its ends, where sidewalks turn
// round the corner), each looking sideways for the nearest sidewalk leg running parallel (within 25 deg)
// on that side and on the same level (layer / bridge / tunnel); the side's distance is the median,
// used when at least half the samples found one. Motorways get none; trunk roads only where tagged.
// Segments too short to measure take the value of the segment continuing them, so the curb runs on.
// Once the road's reference line is fitted (it lies up to a few metres off the raw way, 0.3 m on
// average), every attached side is measured again from that line, so the lanes reach the sidewalk
// and the kerb where they are mapped.

#include <algorithm>
#include <array>
#include <limits>
#include <cmath>
#include <map>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
// the level a way lies on: layer=*, else +1 for a bridge and -1 for a tunnel
int levelOf(const Tags &t) {
  if (auto l = tag(t, "layer")) try {
      return std::stoi(*l);
    } catch (...) {
    }
  if (tagYes(t, "bridge") || tagIs(t, "bridge", "viaduct")) return 1;
  if (tagYes(t, "tunnel")) return -1;
  return 0;
}
constexpr double kCell = 20.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kMaxDistance = 30.0;  // farther: the sidewalk of something else (a service road, a plaza)
constexpr double kMaxVerge = 8.0;      // a wider gap than this between the lanes and the sidewalk is not a verge
constexpr double kStep = 4.0;          // sample spacing along the road

long cellOf(double v) { return static_cast<long>(std::floor(v / kCell)); }

bool hasVergeTooWide(const CrossSection &x, bool leftSide) {
  const auto &lanes = (leftSide ? x.left : x.right).lanes;
  return std::any_of(lanes.begin(), lanes.end(),
                     [](const LaneSpec &l) { return l.kind == LaneKind::Border && l.width > kMaxVerge; });
}

bool hasVerge(const CrossSection &x, bool leftSide) {
  const auto &lanes = (leftSide ? x.left : x.right).lanes;
  return std::any_of(lanes.begin(), lanes.end(), [](const LaneSpec &l) { return l.kind == LaneKind::Border; });
}
}  // namespace

void SidewalkIndex::add(const Leg &g) {
  const size_t k = legs_.size();
  legs_.push_back(g);
  const double L = (g.b - g.a).norm();
  for (double u = 0; u <= L + kCell; u += 0.5 * kCell) {
    const Vec2 p = g.a + (g.b - g.a) * std::min(1.0, u / L);
    auto &cell = grid_[{cellOf(p.x), cellOf(p.y)}];
    if (cell.empty() || cell.back() != k) cell.push_back(k);
  }
}

std::optional<SidewalkIndex::Measured> SidewalkIndex::measure(const std::vector<Sample> &samples, bool left) const {
  std::vector<double> dist, width;
  for (const Sample &q : samples) {
    double walkAt = kMaxDistance, walkWidth = 0.0, kerbAt = kMaxDistance;  // nearest footway / kerb
    const Vec2 nrm{-q.dir.y, q.dir.x};  // left normal
    for (long dx = -2; dx <= 2; ++dx)
      for (long dy = -2; dy <= 2; ++dy) {
        auto it = grid_.find({cellOf(q.p.x) + dx, cellOf(q.p.y) + dy});
        if (it == grid_.end()) continue;
        for (size_t k : it->second) {
          const Leg &g = legs_[k];
          if (g.level != q.level) continue;  // a footway above or below this road is not its sidewalk
          const Vec2 gd = g.b - g.a;
          if (std::abs(gd.dot(q.dir)) < std::cos(25.0 * kPi / 180.0) * gd.norm()) continue;  // not parallel
          // where the normal through the sample meets the leg: p + nrm t = g.a + gd v
          const Vec2 r = g.a - q.p;
          const double v = -(q.dir.dot(r)) / q.dir.dot(gd);
          if (v < 0.0 || v > 1.0) continue;
          const double t = nrm.dot(r + gd * v);
          if ((t > 0) != left) continue;
          if (g.kerb) kerbAt = std::min(kerbAt, std::abs(t));
          else if (std::abs(t) < walkAt) walkAt = std::abs(t), walkWidth = g.width;
        }
      }
    // the sidewalk is the mapped footway; where a kerb line runs between the road and it, the
    // sidewalk reaches from the kerb to the footway's outer edge (both measured). A kerb alone
    // gives no sidewalk width and is not used.
    if (walkAt < kMaxDistance) {
      const double outer = walkAt + 0.5 * walkWidth;
      const bool kerbInside = kerbAt < walkAt && outer - kerbAt >= 1.0;
      dist.push_back(kerbInside ? 0.5 * (kerbAt + outer) : walkAt);
      width.push_back(kerbInside ? outer - kerbAt : walkWidth);
    }
  }
  if (samples.empty() || dist.size() < std::max<size_t>(2, (samples.size() + 1) / 2)) return std::nullopt;
  std::nth_element(dist.begin(), dist.begin() + dist.size() / 2, dist.end());
  std::nth_element(width.begin(), width.begin() + width.size() / 2, width.end());
  return Measured{dist[dist.size() / 2], width[width.size() / 2]};
}

void Builder::attachSeparateSidewalks() {
  if (!opt.separateSidewalks) return;
  const CountryProfile &C = opt.country;
  auto xy = [&](OsmId id, Vec2 &p) {
    const OsmNode *n = osm.node(id);
    if (!n) return false;
    p = proj.forward(n->lat, n->lon);
    return true;
  };
  for (const auto &[id, w] : osm.ways) {
    const auto hw = tag(w.tags, "highway");
    const bool sidewalkWay = hw && (*hw == "footway" || *hw == "path") && tagIs(w.tags, "footway", "sidewalk");
    // a kerb line along the road places the sidewalk's curb-side edge (closed kerbs ring traffic islands)
    const bool kerbLine = tagIs(w.tags, "barrier", "kerb") && w.nodes.size() >= 2 && w.nodes.front() != w.nodes.back();
    if (!sidewalkWay && !kerbLine) continue;
    const double width = sidewalkWay && tag(w.tags, "width") ? lengthTag(*tag(w.tags, "width")).value_or(C.sidewalkWidth)
                                                            : C.sidewalkWidth;
    for (size_t i = 0; i + 1 < w.nodes.size(); ++i) {
      Vec2 a, b;
      if (!xy(w.nodes[i], a) || !xy(w.nodes[i + 1], b) || (b - a).norm() < 0.5) continue;
      sidewalks.add({a, b, width, levelOf(w.tags), kerbLine && !sidewalkWay});
    }
  }
  if (sidewalks.empty()) return;
  // 1. measure: distance and width of the parallel sidewalk per segment side (NaN: none found)
  const size_t n = segs.size();
  std::vector<std::array<double, 2>> D(n, {kNaN, kNaN}), W(n, {kNaN, kNaN});
  std::vector<std::array<bool, 2>> allowed(n, {false, false});
  for (size_t si = 0; si < n; ++si) {
    Seg &sg = segs[si];
    // motorways and their ramps have no sidewalk; a trunk road only where it says so
    const std::string hw = tag(*sg.tags, "highway").value_or("");
    const bool noWalk = hw == "motorway" || hw == "motorway_link", onlyIfTagged = hw == "trunk" || hw == "trunk_link";
    for (int side = 0; side < 2; ++side) {
      const std::string tagged = sidewalkTag(*sg.tags, side == 0 ? "left" : "right");
      // attached ("yes") / none ("no"): as tagged
      allowed[si][side] = !noWalk && (tagged == "separate" || (tagged.empty() && !onlyIfTagged));
    }
    const int level = levelOf(*sg.tags);
    std::vector<Vec2> pts;
    for (OsmId id : sg.nodes) {
      Vec2 p;
      if (!xy(id, p)) break;
      pts.push_back(p);
    }
    if (pts.size() != sg.nodes.size()) continue;
    double total = 0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) total += (pts[i + 1] - pts[i]).norm();
    if (total < 16.0) continue;  // too short to tell: inherits from its continuation below
    // samples every kStep m, not within 6 m of the ends (there the sidewalk turns round the corner)
    std::vector<SidewalkIndex::Sample> samples;
    double run = 0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      const Vec2 a = pts[i], d = pts[i + 1] - pts[i];
      const double L = d.norm();
      if (L < 1e-6) continue;
      const Vec2 dir = d * (1.0 / L);
      for (double u = std::fmod(kStep - std::fmod(run, kStep), kStep); u < L; u += kStep)
        if (run + u >= 6.0 && run + u <= total - 6.0) samples.push_back({a + dir * u, dir, level});
      run += L;
    }
    for (int side = 0; side < 2; ++side) {
      if (!allowed[si][side]) continue;
      if (const auto got = sidewalks.measure(samples, side == 0)) D[si][side] = got->centre, W[si][side] = got->width;
    }
  }
  // 2. continuity: a segment without a measurement of its own (short, or its sidewalk drawn round a
  //    bend) takes it from the segment continuing it through a node no third road uses, so the curb
  //    does not come and go along one street. Repeated until nothing changes (runs of short segments).
  for (bool changed = true; changed;) {
    changed = false;
    for (size_t si = 0; si < n; ++si)
      for (bool atStart : {true, false}) {
        const OsmId node = atStart ? segs[si].nodes.front() : segs[si].nodes.back();
        auto it = nodeSegs.find(node);
        if (it == nodeSegs.end() || it->second.size() != 2) continue;
        const auto &[oj, ojStart] = it->second[0].first == static_cast<int>(si) && it->second[0].second == atStart ? it->second[1] : it->second[0];
        const size_t o = static_cast<size_t>(oj);
        if (o == si) continue;
        const bool flip = atStart == ojStart;  // both start (or both end) here: they run against each other
        for (int side = 0; side < 2; ++side) {
          const int os = flip ? 1 - side : side;
          if (allowed[si][side] && std::isnan(D[si][side]) && !std::isnan(D[o][os]))
            D[si][side] = D[o][os], W[si][side] = W[o][os], changed = true;
        }
      }
  }
  // 3. the cross-section reaches out to the sidewalk
  for (size_t si = 0; si < n; ++si)
    for (int side = 0; side < 2; ++side) {
      if (std::isnan(D[si][side])) continue;
      CrossSection x = segs[si].xs;
      if (!attachSeparateSidewalk(x, side == 0, D[si][side], W[si][side])) continue;
      // a verge wider than kMaxVerge is a plaza, a car park or another road's sidewalk: not this one
      if (hasVergeTooWide(x, side == 0)) continue;
      segs[si].xs = x;
      ++rep.separateSidewalks;
    }
}

// The lanes are laid out from the fitted reference line (OpenDRIVE 1.8.0 §11.6 lane geometry [1.9 §11.7]: lane borders are offsets
// t from it), not from the OSM way the sidewalk was first measured from. Each section side that has
// an attached sidewalk (a verge, LaneKind::Border) is measured again along the trimmed line, samples
// every kStep m at least 2 m inside the road's ends; where too few samples see the sidewalk (a short
// section, a bend), or the new layout does not fit, the first measurement stays.
void Builder::remeasureSidewalks(const RoadMeta &m, const ReferenceLine &line, std::vector<SectionInfo> &secs) const {
  if (sidewalks.empty()) return;
  const double len = line.length();
  auto levelAt = [&](double sFull) {
    for (const auto &lr : m.layers)
      if (sFull >= lr.s0 && sFull <= lr.s1) return lr.layer;
    return 0;
  };
  for (size_t k = 0; k < secs.size(); ++k) {
    const double s0 = std::max(secs[k].s, 2.0), s1 = std::min(k + 1 < secs.size() ? secs[k + 1].s : len, len - 2.0);
    std::vector<SidewalkIndex::Sample> samples;
    for (double s = s0 + 0.5 * kStep; s < s1; s += kStep) {
      const auto e = line.eval(s);
      samples.push_back({Vec2{e.x, e.y}, {std::cos(e.hdg), std::sin(e.hdg)}, levelAt(s + m.trimStart)});
    }
    for (const bool leftSide : {true, false}) {
      if (!hasVerge(secs[k].xs, leftSide)) continue;
      const auto got = sidewalks.measure(samples, leftSide);
      if (!got) continue;
      CrossSection x = secs[k].xs;
      if (attachSeparateSidewalk(x, leftSide, got->centre, got->width) && !hasVergeTooWide(x, leftSide)) secs[k].xs = x;
    }
  }
}

// OpenDRIVE 1.9 §8.3: a lane border offset t from a reference line of curvature k folds (runs
// backwards) where k t reaches 1. A side extended to its separately mapped sidewalk can reach far
// from the way; where the section bends towards that side so tightly that its outer edge would pass
// kMaxBend / k, the verge narrows first (down to kMinVerge), then the sidewalk (down to kMinWalk).
// Widths stay constant per section and blend between sections.
void keepVergeFoldSafe(CrossSection &x, const ReferenceLine &line, double s0, double s1) {
  constexpr double kMaxBend = 0.8, kMinVerge = 0.05, kMinWalk = 1.0;
  for (const bool leftSide : {true, false}) {
    SideSpec &side = leftSide ? x.left : x.right;
    const bool hasVerge = std::any_of(side.lanes.begin(), side.lanes.end(),
                                      [](const LaneSpec &l) { return l.kind == LaneKind::Border; });
    if (!hasVerge) continue;
    double tightest = 0.0;  // largest curvature towards this side over the section
    for (double s = s0; s <= s1 + 1e-9; s += std::max(0.5, (s1 - s0) / 200.0))
      tightest = std::max(tightest, (leftSide ? 1.0 : -1.0) * line.eval(s).kappa);
    if (tightest <= 1e-9) continue;
    const double reach = leftSide ? x.laneOffset + side.totalWidth() : side.totalWidth() - x.laneOffset;
    double excess = reach - kMaxBend / tightest;
    for (LaneSpec &l : side.lanes) {
      if (excess <= 0.0) break;
      if (l.kind != LaneKind::Border) continue;
      const double cut = std::min(excess, std::max(0.0, l.width - kMinVerge));
      l.width -= cut, excess -= cut;
    }
    for (LaneSpec &l : side.lanes) {
      if (excess <= 0.0) break;
      if (l.kind != LaneKind::Sidewalk) continue;
      const double cut = std::min(excess, std::max(0.0, l.width - kMinWalk));
      l.width -= cut, excess -= cut;
    }
  }
}

}  // namespace detail
}  // namespace odrforge
