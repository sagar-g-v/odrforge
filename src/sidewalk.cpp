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

#include <algorithm>
#include <array>
#include <limits>
#include <cmath>
#include <map>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
struct Leg {
  Vec2 a, b;
  double width;
  int level;  // OSM layer, bridges / tunnels one level apart from the ground when untagged
};
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
}  // namespace

void Builder::attachSeparateSidewalks() {
  if (!opt.separateSidewalks) return;
  const CountryProfile &C = opt.country;
  std::vector<Leg> legs;
  std::map<std::pair<long, long>, std::vector<size_t>> grid;
  auto xy = [&](OsmId id, Vec2 &p) {
    const OsmNode *n = osm.node(id);
    if (!n) return false;
    p = proj.forward(n->lat, n->lon);
    return true;
  };
  for (const auto &[id, w] : osm.ways) {
    const auto hw = tag(w.tags, "highway");
    if (!hw || (*hw != "footway" && *hw != "path") || !tagIs(w.tags, "footway", "sidewalk")) continue;
    const double width = tag(w.tags, "width") ? lengthTag(*tag(w.tags, "width")).value_or(C.sidewalkWidth) : C.sidewalkWidth;
    for (size_t i = 0; i + 1 < w.nodes.size(); ++i) {
      Vec2 a, b;
      if (!xy(w.nodes[i], a) || !xy(w.nodes[i + 1], b) || (b - a).norm() < 0.5) continue;
      const size_t k = legs.size();
      legs.push_back({a, b, width, levelOf(w.tags)});
      const double L = (b - a).norm();
      for (double u = 0; u <= L + kCell; u += 0.5 * kCell) {
        const Vec2 p = a + (b - a) * std::min(1.0, u / L);
        auto &cell = grid[{static_cast<long>(std::floor(p.x / kCell)), static_cast<long>(std::floor(p.y / kCell))}];
        if (cell.empty() || cell.back() != k) cell.push_back(k);
      }
    }
  }
  if (legs.empty()) return;
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
    for (int side = 0; side < 2; ++side) {
      if (!allowed[si][side]) continue;
      const bool left = side == 0;
      std::vector<double> dist, width;
      int samples = 0;
      double run = 0;
      for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vec2 a = pts[i], d = pts[i + 1] - pts[i];
        const double L = d.norm();
        if (L < 1e-6) continue;
        const Vec2 dir = d * (1.0 / L);
        for (double u = std::fmod(4.0 - std::fmod(run, 4.0), 4.0); u < L; u += 4.0) {
          if (run + u < 6.0 || run + u > total - 6.0) continue;
          ++samples;
          const Vec2 p = a + dir * u;
          double best = kMaxDistance, bw = 0;
          const long cx = static_cast<long>(std::floor(p.x / kCell)), cy = static_cast<long>(std::floor(p.y / kCell));
          for (long dx = -2; dx <= 2; ++dx)
            for (long dy = -2; dy <= 2; ++dy) {
              auto it = grid.find({cx + dx, cy + dy});
              if (it == grid.end()) continue;
              for (size_t k : it->second) {
                const Leg &g = legs[k];
                if (g.level != level) continue;  // a footway above or below this road is not its sidewalk
                const Vec2 gd = g.b - g.a;
                if (std::abs(gd.dot(dir)) < std::cos(25.0 * kPi / 180.0) * gd.norm()) continue;  // not parallel
                // where the normal through the sample meets the leg: p + nrm t = g.a + gd v
                const Vec2 nrm{-dir.y, dir.x};  // left normal
                const Vec2 r = g.a - p;
                const double v = -(dir.dot(r)) / dir.dot(gd);
                if (v < 0.0 || v > 1.0) continue;
                const double t = nrm.dot(r + gd * v);
                if ((t > 0) != left) continue;
                if (std::abs(t) < best) best = std::abs(t), bw = g.width;
              }
            }
          if (best < kMaxDistance) dist.push_back(best), width.push_back(bw);
        }
        run += L;
      }
      if (samples == 0 || dist.size() < std::max<size_t>(2, (samples + 1) / 2)) continue;
      std::nth_element(dist.begin(), dist.begin() + dist.size() / 2, dist.end());
      std::nth_element(width.begin(), width.begin() + width.size() / 2, width.end());
      D[si][side] = dist[dist.size() / 2], W[si][side] = width[width.size() / 2];
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
      const auto &lanes = (side == 0 ? x.left : x.right).lanes;
      if (std::any_of(lanes.begin(), lanes.end(), [](const LaneSpec &l) { return l.kind == LaneKind::Border && l.width > kMaxVerge; }))
        continue;
      segs[si].xs = x;
      ++rep.separateSidewalks;
    }
}

}  // namespace detail
}  // namespace odrforge
