// OdrForge - side-by-side overlap resolution.
//
// OSM draws each carriageway as one line; dual carriageways, ramps next to their freeway and
// parallel service roads are often drawn closer together than their lanes are wide. With real lane
// widths the carriageways would overlap (a visible defect in any mesh). For near-parallel roads on
// the same layer, each road is pushed half of the overlap away from the other through an extra,
// smooth laneOffset profile (spec 4.1: laneOffset shifts lane 0 relative to the reference line).
#include <algorithm>
#include <cmath>
#include <map>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

/// lateral extent of the vehicle carriageway (no sidewalks) at s
bool extent(const odr::Road &r, double s, double &lo, double &hi) {
  lo = 1e9, hi = -1e9;
  const odr::LaneSection &sec = odr::sectionAt(r, s);
  for (const auto *side : {&sec.left, &sec.right})
    for (const auto &l : *side) {
      if (l.type == "sidewalk" || l.type == "walking") continue;
      double a, b;
      if (!odr::laneBorders(r, s, l.id, a, b) || std::abs(b - a) < 0.05) continue;
      lo = std::min({lo, a, b});
      hi = std::max({hi, a, b});
    }
  return lo < hi;
}

int layerAt(const RoadMeta &m, double s) {
  int layer = 0;
  for (const auto &lr : m.layers)
    if (s + m.trimStart >= lr.s0 - 1e-6 && s + m.trimStart <= lr.s1 + 1e-6) layer = lr.layer;
  return layer;
}

}  // namespace

void Builder::resolveOverlaps() {
  // a drawing offset is corrected, a merge is not: OSM lines a few metres too close are pushed apart
  // (at most maxShift in total, tapered at most ~5 %); deeper overlaps and roads converging into a
  // shared junction (gores) are left to the junction
  const double step = 2.0, cell = 25.0, margin = 0.2, maxShift = 3.0, maxDepth = 6.0, goreLength = 40.0;
  std::map<int, std::vector<odr::Poly3>> original;  // laneOffset before any shift
  std::map<int, std::vector<double>> total;         // accumulated shift per sample
  // the two roads end in the same junction within goreLength of these positions
  auto nearSharedEnd = [&](int mi, double sa, int mj, double sb) {
    const Chain &ca = chains[metas[mi].chain], &cb = chains[metas[mj].chain];
    const double La = od.roads[metas[mi].roadIndex].length(), Lb = od.roads[metas[mj].roadIndex].length();
    for (int ea : {0, 1})
      for (int eb : {0, 1}) {
        const int ka = ea ? ca.c1 : ca.c0, kb = eb ? cb.c1 : cb.c0;
        if (ka < 0 || ka != kb) continue;
        if ((ea ? La - sa : sa) < goreLength && (eb ? Lb - sb : sb) < goreLength) return true;
      }
    return false;
  };
  for (int pass = 0; pass < 3; ++pass) {
    // index of reference-line samples
    std::map<std::pair<long, long>, std::vector<std::pair<int, double>>> grid;
    for (int mi = 0; mi < static_cast<int>(metas.size()); ++mi) {
      if (metas[mi].roadIndex < 0) continue;
      const odr::Road &r = od.roads[metas[mi].roadIndex];
      for (double s = 0; s <= r.length(); s += 5.0) {
        const Pose p = r.planView.eval(s);
        grid[{static_cast<long>(std::floor(p.x / cell)), static_cast<long>(std::floor(p.y / cell))}].push_back({mi, s});
      }
    }
    int shifted = 0;
    for (int mi = 0; mi < static_cast<int>(metas.size()); ++mi) {
      RoadMeta &m = metas[mi];
      if (m.roadIndex < 0) continue;
      odr::Road &a = od.roads[m.roadIndex];
      const double L = a.length();
      const int n = std::max(2, static_cast<int>(std::ceil(L / step)) + 1);
      std::vector<double> push(n, 0.0);  // required lateral shift (left +) per sample
      bool any = false;
      for (int i = 0; i < n; ++i) {
        const double s = L * i / (n - 1);
        double alo, ahi;
        if (!extent(a, s, alo, ahi)) continue;
        const Pose pa = a.planView.eval(s);
        const int la = layerAt(m, s);
        const long cx = static_cast<long>(std::floor(pa.x / cell)), cy = static_cast<long>(std::floor(pa.y / cell));
        std::map<int, bool> seen;
        for (long dx = -1; dx <= 1; ++dx)
          for (long dy = -1; dy <= 1; ++dy) {
            auto it = grid.find({cx + dx, cy + dy});
            if (it == grid.end()) continue;
            for (const auto &[mj, sj] : it->second) {
              (void)sj;
              if (mj == mi || seen[mj]) continue;
              seen[mj] = true;
              const RoadMeta &mb = metas[mj];
              const odr::Road &b = od.roads[mb.roadIndex];
              double sb, tb;
              b.planView.project(pa.pos(), sb, tb);
              if (sb < 0.5 || sb > b.length() - 0.5) continue;  // ends meet in junctions instead
              const Pose pb = b.planView.eval(sb);
              if (std::abs((pa.pos() - pb.pos()).dot(pb.dir())) > 0.5) continue;
              const double c = std::cos(pa.hdg - pb.hdg);
              if (std::abs(c) < std::cos(30.0 * kPi / 180.0)) continue;  // crossing, not side by side
              if (layerAt(mb, sb) != la) continue;                        // grade separated
              if (nearSharedEnd(mi, s, mj, sb)) continue;                 // a gore: merges in the junction
              double blo, bhi;
              if (!extent(b, sb, blo, bhi)) continue;
              // a's carriageway in b's lateral frame
              const double lo = c > 0 ? tb + alo : tb - ahi, hi = c > 0 ? tb + ahi : tb - alo;
              const double depth = std::min(hi, bhi) - std::max(lo, blo) + margin;
              if (depth <= margin || depth > maxDepth + margin) continue;
              // move a away from b by half the overlap (b moves the other half on its own turn)
              const double dirInB = (0.5 * (lo + hi) >= 0.5 * (blo + bhi)) ? 1.0 : -1.0;
              const double need = 0.5 * depth * dirInB * (c > 0 ? 1.0 : -1.0);
              if (std::abs(need) > std::abs(push[i])) push[i] = need;
              any = true;
            }
          }
      }
      if (!any) continue;
      // smooth envelope: keep the required shift, spread it with C1 ramps of at least 20 m and a
      // taper of at most ~5 % (smoothstep peak slope 1.5 push / ramp length)
      std::vector<double> sm(n, 0.0);
      for (int i = 0; i < n; ++i) {
        if (std::abs(push[i]) < 1e-6) continue;
        push[i] = std::clamp(push[i], -maxShift, maxShift);
        const int ramp = std::max(1, static_cast<int>(std::max(20.0, 30.0 * std::abs(push[i])) / step));
        for (int k = -ramp; k <= ramp; ++k) {
          const int j = i + k;
          if (j < 0 || j >= n) continue;
          const double u = 1.0 - std::abs(k) / static_cast<double>(ramp + 1);
          const double v = push[i] * (u * u * (3 - 2 * u));
          if (std::abs(v) > std::abs(sm[j])) sm[j] = v;
        }
      }
      // new laneOffset = original + accumulated shift (bounded), as C1 cubic pieces every ~10 m
      if (!original.count(mi)) original[mi] = a.laneOffset, total[mi].assign(n, 0.0);
      std::vector<double> &acc = total[mi];
      for (int i = 0; i < n; ++i) acc[i] = std::clamp(acc[i] + sm[i], -maxShift, maxShift);
      odr::Road base = a;
      base.laneOffset = original[mi];
      const int every = std::max(1, static_cast<int>(10.0 / step));
      std::vector<double> ss(n), off(n), d(n);
      for (int i = 0; i < n; ++i) ss[i] = L * i / (n - 1), off[i] = odr::laneOffsetAt(base, ss[i]) + acc[i];
      for (int i = 0; i < n; ++i) {
        const int p = std::max(0, i - 1), q = std::min(n - 1, i + 1);
        d[i] = (off[q] - off[p]) / std::max(1e-9, ss[q] - ss[p]);
      }
      std::vector<odr::Poly3> lo;
      int k0 = 0;
      while (k0 < n - 1) {
        const int k1 = std::min(n - 1, k0 + every);
        lo.push_back(odr::Poly3::hermite(ss[k0], ss[k1] - ss[k0], off[k0], d[k0], off[k1], d[k1]));
        k0 = k1;
      }
      a.laneOffset = lo;
      ++shifted;
    }
    rep.overlapShifts += shifted;
    if (!shifted) break;
  }
}

}  // namespace detail
}  // namespace odrforge
