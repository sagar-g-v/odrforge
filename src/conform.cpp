// OdrForge - a ground grid conformed to the roads (see conform.hpp).
#include "odrforge/conform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace odrforge {

TerrainGrid conformGround(const odr::OpenDrive &od, const TerrainGrid &ground, const ConformOptions &opt) {
  TerrainGrid out = ground;
  out.levels = 1;
  out.z.assign(ground.z.begin(), ground.z.begin() + static_cast<long>(ground.nx) * ground.ny);
  const int nx = ground.nx, ny = ground.ny;
  const double cell = ground.cell;
  const size_t N = static_cast<size_t>(nx) * ny;
  // 1. the road surface under every lane (outside bridges and tunnels). Where roads share a cell (junction roads, the
  //    two legs of a hairpin side by side at different heights) the lowest surface is the ground: a higher road stands
  //    on its wall or embankment there, a lower one is not buried. One pass of a road over the cell gives the mean of
  //    its samples there; a road passing the cell again further along (a hairpin's other leg, > 20 m along s) is
  //    another pass
  struct Pass {
    double s, sum;
    int n;
  };
  std::vector<double> lowest(N, std::numeric_limits<double>::infinity());
  std::vector<int> cnt(N, 0);
  std::unordered_map<size_t, std::vector<Pass>> own;  // this road's passes per cell
  const double step = 0.5 * cell;
  for (const odr::Road &r : od.roads) {
    if (r.lanes.empty()) continue;
    auto onStructure = [&](double s) {
      for (const auto *list : {&r.bridges, &r.tunnels})
        for (const odr::Structure &st : *list)
          if (s >= st.s - 1e-6 && s <= st.s + st.length + 1e-6) return true;
      return false;
    };
    const double L = r.length();
    own.clear();
    for (double s = 0.0; s <= L + 1e-9; s += step) {
      const double sc = std::min(s, L);
      if (onStructure(sc)) continue;
      const Pose p = r.planView.eval(sc);
      const odr::LaneSection &sec = odr::sectionAt(r, sc);
      double lo = std::numeric_limits<double>::infinity(), hi = -lo;
      for (const auto *side : {&sec.left, &sec.right})
        for (const odr::Lane &l : *side) {
          double a, b;
          if (!odr::laneBorders(r, sc, l.id, a, b)) continue;
          lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
        }
      if (!(hi > lo)) continue;
      for (double t = lo; t <= hi + 1e-9; t += step) {
        const Vec2 q = p.offset(std::min(t, hi));
        const int i = static_cast<int>(std::lround((q.x - ground.originX) / cell));
        const int j = static_cast<int>(std::lround((q.y - ground.originY) / cell));
        if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
        const size_t k = static_cast<size_t>(j) * nx + i;
        const double zs = odr::surfaceZ(r, sc, std::min(t, hi));
        std::vector<Pass> &ps = own[k];
        auto it = std::find_if(ps.begin(), ps.end(), [&](const Pass &q) { return std::abs(q.s - sc) <= 20.0; });
        if (it == ps.end()) ps.push_back({sc, zs, 1});
        else it->sum += zs, ++it->n;
      }
    }
    for (const auto &[k, ps] : own) {
      for (const Pass &q : ps) lowest[k] = std::min(lowest[k], q.sum / q.n);
      ++cnt[k];
    }
  }
  // 2. distance to the nearest road cell and that cell's height (multi-source Dijkstra on the 8-neighbour grid)
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(N, inf), roadZ(N, std::numeric_limits<double>::quiet_NaN());
  using Item = std::pair<double, size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> pq;
  for (size_t k = 0; k < N; ++k)
    if (cnt[k] > 0) dist[k] = 0.0, roadZ[k] = lowest[k], pq.push({0.0, k});
  while (!pq.empty()) {
    const auto [d, k] = pq.top();
    pq.pop();
    if (d > dist[k] || d > opt.maxReach) continue;
    const int i = static_cast<int>(k % nx), j = static_cast<int>(k / nx);
    for (int dj = -1; dj <= 1; ++dj)
      for (int di = -1; di <= 1; ++di) {
        if (!di && !dj) continue;
        const int a = i + di, b = j + dj;
        if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
        const size_t m = static_cast<size_t>(b) * nx + a;
        const double nd = d + cell * ((di && dj) ? std::sqrt(2.0) : 1.0);
        if (nd < dist[m]) dist[m] = nd, roadZ[m] = roadZ[k], pq.push({nd, m});
      }
  }
  // 3. under a road its surface; beside it the cut / fill slope from the road edge, where it lies between the road and
  //    the terrain (a fill: the higher of embankment and terrain, a cut: the lower of cutting and terrain)
  for (size_t k = 0; k < N; ++k) {
    if (std::isnan(roadZ[k])) continue;
    if (cnt[k] > 0) {
      out.z[k] = static_cast<float>(roadZ[k]);
      continue;
    }
    const double g = out.z[k];
    if (std::isnan(g) || dist[k] > opt.maxReach) continue;
    const double drop = dist[k] / opt.slope;
    out.z[k] = static_cast<float>(roadZ[k] > g ? std::max(g, roadZ[k] - drop) : std::min(g, roadZ[k] + drop));
  }
  return out;
}

}  // namespace odrforge
