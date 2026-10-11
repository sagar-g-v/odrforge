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

/// lateral extent of the travelled way at s: driving, bus, bike and bidirectional lanes. Shoulders,
/// parking and borders of adjacent carriageways may touch (they do on real roads); moving a whole
/// carriageway for them would bend it for nothing
bool extent(const odr::Road &r, double s, double &lo, double &hi) {
  lo = 1e9, hi = -1e9;
  const odr::LaneSection &sec = odr::sectionAt(r, s);
  for (const auto *side : {&sec.left, &sec.right})
    for (const auto &l : *side) {
      if (l.type != "driving" && l.type != "bus" && l.type != "biking" && l.type != "bidirectional") continue;
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

void Builder::resolveLevelOverlaps() {
  overlapLevelPass = true;
  resolveOverlaps();
  overlapLevelPass = false;
}

void Builder::resolveOverlaps() {
  if (!opt.resolveOverlaps) return;
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
      // free room to the left / right before a shift would run into another road (gore partners and
      // roads already clear on that side are obstacles too)
      std::vector<double> roomL(n, maxShift), roomR(n, maxShift);
      bool any = false;
      for (int i = 0; i < n; ++i) {
        const double s = L * i / (n - 1);
        double alo, ahi;
        if (!extent(a, s, alo, ahi)) continue;
        const Pose pa = a.planView.eval(s);
        // on the inside of a bend the lanes (all of them, sidewalks too) must stay clear of the
        // centre of curvature: a shift toward it may not bring the inner edge within 1 m of it
        if (std::abs(pa.kappa) > 1e-4) {
          double flo = 0, fhi = 0;
          const odr::LaneSection &sec = odr::sectionAt(a, s);
          for (const auto *side : {&sec.left, &sec.right})
            for (const auto &l : *side) {
              double u, v;
              if (!odr::laneBorders(a, s, l.id, u, v)) continue;
              flo = std::min({flo, u, v}), fhi = std::max({fhi, u, v});
            }
          const double R = 1.0 / std::abs(pa.kappa);
          if (pa.kappa > 0) roomL[i] = std::min(roomL[i], std::max(0.0, R - 1.0 - fhi));
          else roomR[i] = std::min(roomR[i], std::max(0.0, R - 1.0 + flo));
        }
        const int la = layerAt(m, s);
        const long cx = static_cast<long>(std::floor(pa.x / cell)), cy = static_cast<long>(std::floor(pa.y / cell));
        std::map<int, bool> seen;
        double needL = 0, needR = 0;
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
              // grade separated: by OSM layer before the heights are known; afterwards only pairs on
              // different layers that are not really apart vertically (< 4.5 m) are handled
              const bool otherLayer = layerAt(mb, sb) != la;
              if (overlapLevelPass ? (!otherLayer || std::abs(odr::elevationAt(a, s) - odr::elevationAt(b, sb)) >= 4.5)
                                   : otherLayer)
                continue;
              double blo, bhi;
              if (!extent(b, sb, blo, bhi)) continue;
              // a's carriageway in b's lateral frame
              const double lo = c > 0 ? tb + alo : tb - ahi, hi = c > 0 ? tb + ahi : tb - alo;
              const double overlap = std::min(hi, bhi) - std::max(lo, blo);
              // the side of a that faces away from b (in a's frame, +1 = left)
              const double away = ((0.5 * (lo + hi) >= 0.5 * (blo + bhi)) ? 1.0 : -1.0) * (c > 0 ? 1.0 : -1.0);
              std::vector<double> &room = away > 0 ? roomR : roomL;  // b lies on the other side
              const bool gore = nearSharedEnd(mi, s, mj, sb);        // merges in the junction: never pushed from
              if (gore || overlap <= 0 || overlap > maxDepth) {
                room[i] = std::min(room[i], std::max(0.0, -overlap - margin));
                continue;
              }
              // move a away from b by half the overlap (b moves the other half on its own turn)
              const double need = 0.5 * (overlap + margin);
              (away > 0 ? needL : needR) = std::max(away > 0 ? needL : needR, need);
              room[i] = 0.0;
            }
          }
        if (needL > 0 && needR > 0) continue;  // squeezed from both sides: a shift cannot help
        if (needL > 0) push[i] = needL, any = true;
        if (needR > 0) push[i] = -needR, any = true;
      }
      if (!any) continue;
      // pushes the same way within 150 m become one plateau (a road does not move out and back in
      // between two pinch points)
      {
        const int R = static_cast<int>(150.0 / step);
        for (double sgn : {1.0, -1.0}) {
          std::vector<double> fwd(n, 0.0), bwd(n, 0.0);
          std::vector<double> v(n);
          for (int i = 0; i < n; ++i) v[i] = std::max(0.0, sgn * push[i]);
          for (int i = 0; i < n; ++i)
            for (int j = std::max(0, i - R); j <= i; ++j) fwd[i] = std::max(fwd[i], v[j]);
          for (int i = 0; i < n; ++i)
            for (int j = i; j <= std::min(n - 1, i + R); ++j) bwd[i] = std::max(bwd[i], v[j]);
          for (int i = 0; i < n; ++i) {
            const double bridge = std::min(fwd[i], bwd[i]);
            if (bridge > std::abs(push[i]) && (push[i] == 0.0 || push[i] * sgn > 0)) push[i] = sgn * bridge;
          }
        }
      }
      // smooth envelope: keep the required shift, spread it with C1 ramps of at least 60 m and a
      // taper of at most ~2 % (smoothstep peak slope 1.5 push / ramp length): a realignment, not a
      // wiggle; never more than the free room on that side
      std::vector<double> sm(n, 0.0);
      for (int i = 0; i < n; ++i) {
        if (std::abs(push[i]) < 1e-6) continue;
        push[i] = std::clamp(push[i], -maxShift, maxShift);
        const int ramp = std::max(1, static_cast<int>(std::max(60.0, 75.0 * std::abs(push[i])) / step));
        for (int k = -ramp; k <= ramp; ++k) {
          const int j = i + k;
          if (j < 0 || j >= n) continue;
          const double u = 1.0 - std::abs(k) / static_cast<double>(ramp + 1);
          const double v = push[i] * (u * u * (3 - 2 * u));
          if (std::abs(v) > std::abs(sm[j])) sm[j] = v;
        }
      }
      // limit by the room, then re-smooth the limit so the taper stays gentle
      std::vector<double> lim(n);
      for (int i = 0; i < n; ++i) lim[i] = sm[i] >= 0 ? roomL[i] : roomR[i];
      const double dLim = 0.02 * step;  // the limit changes by at most 2 % per metre
      for (int i = 1; i < n; ++i) lim[i] = std::min(lim[i], lim[i - 1] + dLim);
      for (int i = n - 2; i >= 0; --i) lim[i] = std::min(lim[i], lim[i + 1] + dLim);
      for (int i = 0; i < n; ++i) sm[i] = std::clamp(sm[i], -lim[i], lim[i]);
      if (const char *dbgRoad = std::getenv("ODRFORGE_DEBUG_ROAD"); dbgRoad && a.id == dbgRoad)
        for (int i = 0; i < n; ++i)
          std::fprintf(stderr, "overlap road %s pass level %d s %.1f push %.2f roomL %.2f roomR %.2f lim %.2f sm %.2f k %.4f\n", a.id.c_str(),
                       overlapLevelPass, L * i / (n - 1), push[i], roomL[i], roomR[i], lim[i], sm[i], a.planView.eval(L * i / (n - 1)).kappa);
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
