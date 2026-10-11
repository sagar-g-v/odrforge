// OdrForge - road elevation from terrain / structure surfaces, junction surfaces and banking.
//
// Spec basis (docs/research/opendrive_spec_notes.md):
//  - spec 3: <elevation s a b c d> with absolute s; <superelevation> = roll angle about the reference
//    line, positive falling to the right, so the surface height at lateral t is z(s) + t tan(se(s)).
//  - pitfall 11: C1 elevation joins and matching z at links.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Whittaker smoother: minimise sum w_i (z_i - y_i)^2 + lambda sum (second difference of z)^2.
/// Unknown samples have w = 0 and are filled with the smoothest (cubic-spline-like) curve.
std::vector<double> whittaker(const std::vector<double> &y, const std::vector<double> &w, double lambda) {
  const size_t n = y.size();
  if (n < 3) {  // too short to smooth: weighted mean of the known samples everywhere
    double s = 0, ws = 0;
    for (size_t i = 0; i < n; ++i)
      if (w[i] > 0 && !std::isnan(y[i])) s += w[i] * y[i], ws += w[i];
    return std::vector<double>(n, ws > 0 ? s / ws : 0.0);
  }
  // symmetric pentadiagonal A = W + lambda D'D, stored as diagonals d0, d1, d2
  std::vector<double> d0(n, 0.0), d1(n, 0.0), d2(n, 0.0), b(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    d0[i] = w[i];
    b[i] = w[i] * (std::isnan(y[i]) ? 0.0 : y[i]);
  }
  for (size_t k = 0; k + 2 < n; ++k) {  // row k of D = [1, -2, 1] at k, k+1, k+2
    const size_t i[3] = {k, k + 1, k + 2};
    const double c[3] = {1.0, -2.0, 1.0};
    for (int a = 0; a < 3; ++a)
      for (int bb = a; bb < 3; ++bb) {
        const double v = lambda * c[a] * c[bb];
        const size_t off = i[bb] - i[a];
        if (off == 0) d0[i[a]] += v;
        else if (off == 1) d1[i[a]] += v;
        else d2[i[a]] += v;
      }
  }
  // banded LDL^T (bandwidth 2)
  std::vector<double> D(n), L1(n, 0.0), L2(n, 0.0);
  for (size_t j = 0; j < n; ++j) {
    double dj = d0[j];
    if (j >= 1) dj -= L1[j - 1] * L1[j - 1] * D[j - 1];
    if (j >= 2) dj -= L2[j - 2] * L2[j - 2] * D[j - 2];
    D[j] = std::max(dj, 1e-12);
    if (j + 1 < n) {
      double v = d1[j];
      if (j >= 1) v -= L1[j - 1] * L2[j - 1] * D[j - 1];
      L1[j] = v / D[j];
    }
    if (j + 2 < n) L2[j] = d2[j] / D[j];
  }
  std::vector<double> z(n);
  for (size_t j = 0; j < n; ++j) {  // forward: L u = b
    z[j] = b[j];
    if (j >= 1) z[j] -= L1[j - 1] * z[j - 1];
    if (j >= 2) z[j] -= L2[j - 2] * z[j - 2];
  }
  for (size_t j = 0; j < n; ++j) z[j] /= D[j];
  for (size_t j = n; j-- > 0;) {  // backward: L^T x = u
    if (j + 1 < n) z[j] -= L1[j] * z[j + 1];
    if (j + 2 < n) z[j] -= L2[j] * z[j + 2];
  }
  return z;
}

/// lateral position of the carriageway centre (driving lanes) at s
double carriagewayCentre(const odr::Road &r, double s) {
  double lo = 1e9, hi = -1e9;
  const odr::LaneSection &sec = odr::sectionAt(r, s);
  for (const auto *side : {&sec.left, &sec.right})
    for (const auto &l : *side) {
      if (l.type != "driving" && l.type != "bidirectional") continue;
      double a, b;
      if (!odr::laneBorders(r, s, l.id, a, b)) continue;
      lo = std::min({lo, a, b});
      hi = std::max({hi, a, b});
    }
  return lo > hi ? odr::laneOffsetAt(r, s) : 0.5 * (lo + hi);
}

double smoothstepC1(double u) { return u * u * (3.0 - 2.0 * u); }

}  // namespace

void Builder::sampleElevation() {
  const bool haveTerrain = opt.terrain || static_cast<bool>(opt.elevation);
  struct Cand {
    double z, cost;
  };
  struct Prof {
    std::vector<std::vector<Cand>> cand;
    std::vector<Vec2> xy;
    std::vector<int> layer;
    std::vector<std::vector<std::pair<double, double>>> excl;  // forbidden z intervals per sample
    std::vector<double> chosen;
    std::vector<double> tc, cross, groundC;  // carriageway centre t, raw cross slope, ground at centre
    double halfWidth = 3.0;
  };
  std::vector<Prof> P(metas.size());
  // ---- 1. candidate surfaces per sample: measured ground and every bridge-deck level found there
  for (size_t mi = 0; mi < metas.size(); ++mi) {
    RoadMeta &m = metas[mi];
    if (m.roadIndex < 0) continue;
    const odr::Road &r = od.roads[m.roadIndex];
    const double L = r.length();
    const int n = std::max(2, static_cast<int>(std::ceil(L / opt.elevationStep)) + 1);
    Prof &pf = P[mi];
    m.zs.resize(n);
    pf.cand.resize(n);
    pf.xy.resize(n);
    pf.layer.assign(n, 0);
    pf.tc.assign(n, 0.0);
    pf.cross.assign(n, kNaN);
    pf.groundC.assign(n, kNaN);
    pf.excl.resize(n);
    {  // half width of the carriageway at mid length (overlap tests, cross-slope probes)
      const double s = 0.5 * L;
      double lo = 0, hi = 0;
      const odr::LaneSection &sec = odr::sectionAt(r, s);
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) {
          double a, b;
          if (l.type == "sidewalk" || !odr::laneBorders(r, s, l.id, a, b)) continue;
          lo = std::min({lo, a, b});
          hi = std::max({hi, a, b});
        }
      pf.halfWidth = std::max(1.5, 0.5 * (hi - lo));
    }
    auto inRanges = [&](const std::vector<std::pair<double, double>> &rg, double s) {
      for (const auto &[a, b] : rg)
        if (s + m.trimStart >= a - 1e-6 && s + m.trimStart <= b + 1e-6) return true;
      return false;
    };
    for (int i = 0; i < n; ++i) {
      const double s = L * i / (n - 1);
      m.zs[i] = s;
      const Pose p = r.planView.eval(s);
      const double tcen = carriagewayCentre(r, s);
      const Vec2 q = p.offset(tcen);
      pf.xy[i] = q;
      pf.tc[i] = tcen;
      if (opt.terrain && !opt.terrain->ground.empty()) {  // terrain cross slope over the carriageway
        const double h = 0.4 * pf.halfWidth;
        const Vec2 ql = q + p.left() * h, qr = q - p.left() * h;
        const double zl = opt.terrain->ground_at(ql.x, ql.y), zr = opt.terrain->ground_at(qr.x, qr.y);
        pf.groundC[i] = opt.terrain->ground_at(q.x, q.y);
        if (!std::isnan(zl) && !std::isnan(zr)) pf.cross[i] = (zl - zr) / (2.0 * h);
      }
      for (const auto &lr : m.layers)
        if (s + m.trimStart >= lr.s0 - 1e-6 && s + m.trimStart <= lr.s1 + 1e-6) pf.layer[i] = lr.layer;
      if (!haveTerrain) continue;
      const bool bridge = inRanges(m.bridges, s), tunnel = inRanges(m.tunnels, s) || inRanges(m.covered, s);
      // the terrain above a tunnel or a covered way (under a building, through a hillside) is not the
      // road: interpolated between the portals
      if (tunnel) continue;
      if (!opt.terrain) {
        const double z = opt.elevation(q.x, q.y);
        if (!std::isnan(z)) pf.cand[i].push_back({z, 0.0});
        continue;
      }
      // on an OSM bridge only decks are valid (none measured: interpolated between the abutments);
      // elsewhere ground is preferred, decks allowed (viaducts mapped without bridge=yes)
      for (double z : opt.terrain->structure_levels(q.x, q.y)) pf.cand[i].push_back({z, bridge ? 0.0 : 0.5});
      if (!bridge) {
        const double g = opt.terrain->ground_at(q.x, q.y);
        if (!std::isnan(g)) pf.cand[i].push_back({g, 0.0});
      }
    }
  }
  // ---- 2. dynamic programming along each road: the surface sequence with the fewest jumps and
  //         steep grades (a road follows its own deck and stays on the ground under bridges)
  auto solve = [&](size_t mi) {
    Prof &pf = P[mi];
    const RoadMeta &m = metas[mi];
    const size_t n = pf.cand.size();
    pf.chosen.assign(n, kNaN);
    std::vector<int> idx;
    std::vector<std::vector<Cand>> cs(n);
    for (size_t i = 0; i < n; ++i) {
      for (const Cand &c : pf.cand[i]) {
        bool banned = false;
        for (const auto &[lo, hi] : pf.excl[i]) banned |= c.z > lo && c.z < hi;
        if (!banned) cs[i].push_back(c);
      }
      if (!cs[i].empty()) idx.push_back(static_cast<int>(i));
    }
    if (idx.empty()) return;
    // states (sample k, candidate a); a state may follow any earlier state within `window` samples,
    // the skipped samples count as unobserved (a covered stretch: interpolated later)
    const double skipCost = 10.0;  // per unobserved sample: more than following any plausible grade
    const size_t window = static_cast<size_t>(std::max(2.0, 60.0 / opt.elevationStep));
    const size_t K = idx.size();
    std::vector<std::vector<double>> cost(K);
    std::vector<std::vector<std::pair<int, int>>> from(K);
    for (size_t k = 0; k < K; ++k) {
      const auto &ck = cs[idx[k]];
      cost[k].assign(ck.size(), 1e300);
      from[k].assign(ck.size(), {-1, -1});
      for (size_t a = 0; a < ck.size(); ++a) {
        // start here (everything before is skipped)
        cost[k][a] = ck[a].cost + skipCost * k;
        for (size_t kp = (k > window ? k - window : 0); kp < k; ++kp) {
          const auto &cp = cs[idx[kp]];
          const double d = std::max(1e-6, m.zs[idx[k]] - m.zs[idx[kp]]);
          const double skipped = static_cast<double>(k - kp - 1);
          for (size_t b = 0; b < cp.size(); ++b) {
            const double dz = std::abs(ck[a].z - cp[b].z), g = dz / d;
            // only implausible grades (> 30 %: steeper than any public street) are penalised
            const double edge = 2.0 * dz + (g > 0.30 ? 1000.0 * (g - 0.30) * d : 0.0);
            const double c = cost[kp][b] + edge + ck[a].cost + skipCost * skipped;
            if (c < cost[k][a]) cost[k][a] = c, from[k][a] = {static_cast<int>(kp), static_cast<int>(b)};
          }
        }
      }
    }
    // best end state (later samples skipped)
    int bk = -1, ba = -1;
    double bc = 1e300;
    for (size_t k = 0; k < K; ++k)
      for (size_t a = 0; a < cost[k].size(); ++a) {
        const double c = cost[k][a] + skipCost * (K - 1 - k);
        if (c < bc) bc = c, bk = static_cast<int>(k), ba = static_cast<int>(a);
      }
    while (bk >= 0 && ba >= 0) {
      pf.chosen[idx[bk]] = cs[idx[bk]][ba].z;
      const auto [pk, pa] = from[bk][ba];
      bk = pk, ba = pa;
    }
  };
  for (size_t mi = 0; mi < metas.size(); ++mi)
    if (metas[mi].roadIndex >= 0) solve(mi);
  // ---- 3. grade separations: roads that overlap in plan without a junction must be at least
  //         minClearance apart; the lower OSM layer gives way (aerial lidar cannot see a deck under
  //         another deck, so the lower road must not borrow the upper surface)
  const double minClearance = 4.5, cell = 10.0;
  // OSM bridge / tunnel ways often end a few metres short of the crossing: the layer that decides
  // who gives way is the highest (lowest) within 30 m along the road
  std::vector<std::vector<int>> layerHi(P.size()), layerLo(P.size());
  for (size_t mi = 0; mi < P.size(); ++mi) {
    const auto &ly = P[mi].layer;
    layerHi[mi] = ly, layerLo[mi] = ly;
    for (size_t i = 0; i < ly.size(); ++i)
      for (size_t j = 0; j < ly.size(); ++j)
        if (std::abs(metas[mi].zs[i] - metas[mi].zs[j]) <= 30.0)
          layerHi[mi][i] = std::max(layerHi[mi][i], ly[j]), layerLo[mi][i] = std::min(layerLo[mi][i], ly[j]);
  }
  auto dirAt = [](const Prof &p, size_t i) {
    const size_t a = i > 0 ? i - 1 : i, b = std::min(p.xy.size() - 1, i + 1);
    const Vec2 d = p.xy[b] - p.xy[a];
    return d * (1.0 / std::max(1e-9, d.norm()));
  };
  // the roads cross (more than 30 degrees apart), not run side by side into a merge
  auto crossing = [&](const Prof &a, size_t i, const Prof &b, size_t j) {
    const Vec2 da = dirAt(a, i), db = dirAt(b, j);
    return std::abs(da.x * db.y - da.y * db.x) > 0.5;
  };
  // heights along a road with unobserved samples (a bridge without measured deck) filled linearly:
  // an unmeasured deck still has to clear the road below it
  auto filled = [&](const Prof &p) {
    std::vector<double> z = p.chosen;
    const size_t n = z.size();
    long last = -1;
    for (size_t i = 0; i < n; ++i) {
      if (std::isnan(p.chosen[i])) continue;
      if (last < 0)
        for (size_t k = 0; k < i; ++k) z[k] = p.chosen[i];
      else
        for (size_t k = last + 1; k < i; ++k)
          z[k] = p.chosen[last] + (p.chosen[i] - p.chosen[last]) * double(k - last) / double(i - last);
      last = static_cast<long>(i);
    }
    if (last >= 0)
      for (size_t k = last + 1; k < n; ++k) z[k] = p.chosen[last];
    return z;
  };
  for (int pass = 0; pass < 3 && opt.terrain; ++pass) {
    std::vector<std::vector<double>> zf(P.size());
    for (size_t mi = 0; mi < P.size(); ++mi) zf[mi] = filled(P[mi]);
    std::map<std::pair<long, long>, std::vector<std::pair<size_t, size_t>>> grid;
    for (size_t mi = 0; mi < P.size(); ++mi)
      for (size_t i = 0; i < P[mi].xy.size(); ++i)
        grid[{static_cast<long>(std::floor(P[mi].xy[i].x / cell)), static_cast<long>(std::floor(P[mi].xy[i].y / cell))}]
            .push_back({mi, i});
    std::set<size_t> dirty;
    for (size_t mi = 0; mi < P.size(); ++mi) {
      Prof &a = P[mi];
      for (size_t i = 0; i < a.xy.size(); ++i) {
        if (i >= a.chosen.size() || std::isnan(a.chosen[i])) continue;
        const long cx = static_cast<long>(std::floor(a.xy[i].x / cell)), cy = static_cast<long>(std::floor(a.xy[i].y / cell));
        for (long dx = -1; dx <= 1; ++dx)
          for (long dy = -1; dy <= 1; ++dy) {
            auto it = grid.find({cx + dx, cy + dy});
            if (it == grid.end()) continue;
            for (const auto &[mj, j] : it->second) {
              if (mj == mi) continue;
              const Prof &b = P[mj];
              if (j >= zf[mj].size() || std::isnan(zf[mj][j]) || (a.xy[i] - b.xy[j]).norm() > b.halfWidth + 1.0) continue;
              const double dz = a.chosen[i] - zf[mj][j];
              if (std::abs(dz) >= minClearance) continue;
              // a gives way if its layer is lower (equal non-zero layers: the one already lower)
              // (two roads crossing without a junction cannot share a level even when neither is
              // tagged: the one already lower goes under, on a tie the lower road class)
              const bool below = dz < -0.5 || (std::abs(dz) <= 0.5 && metas[mi].rank < metas[mj].rank);
              const bool aLower = a.layer[i] < b.layer[j] || (a.layer[i] == b.layer[j] && a.layer[i] != 0 && dz < 0) ||
                                  (a.layer[i] == b.layer[j] && a.layer[i] == 0 && below && crossing(a, i, b, j)) ||
                                  (a.layer[i] == b.layer[j] && crossing(a, i, b, j) &&
                                   (layerHi[mi][i] < layerHi[mj][j] || layerLo[mi][i] < layerLo[mj][j]) &&
                                   !(layerHi[mi][i] > layerHi[mj][j] || layerLo[mi][i] > layerLo[mj][j]));
              if (!aLower) continue;
              a.excl[i].push_back({zf[mj][j] - minClearance, zf[mj][j] + minClearance});
              dirty.insert(mi);
            }
          }
      }
    }
    if (dirty.empty()) break;
    // the upper structure is wider than its OSM centre lines (shoulders, gores, parallel ramps): a
    // banned surface continues along the lower road while its candidates stay inside the band
    for (size_t mi : dirty) {
      Prof &a = P[mi];
      const std::vector<std::vector<std::pair<double, double>>> ex0 = a.excl;
      for (size_t i = 0; i < ex0.size(); ++i)
        for (const auto &band : ex0[i])
          for (int dir : {-1, 1})
            for (long j = static_cast<long>(i) + dir; j >= 0 && j < static_cast<long>(a.cand.size()); j += dir) {
              if (std::abs(metas[mi].zs[j] - metas[mi].zs[i]) > 40.0) break;
              bool inside = false;
              for (const Cand &c : a.cand[j]) inside |= c.z > band.first && c.z < band.second;
              if (!inside) break;
              a.excl[j].push_back(band);
            }
    }
    for (size_t mi : dirty) solve(mi);
  }
  if (const char *dbg = std::getenv("ODRFORGE_DEBUG_ROAD"))
    for (size_t mi = 0; mi < metas.size(); ++mi)
      if (metas[mi].roadIndex >= 0 && od.roads[metas[mi].roadIndex].id == dbg)
        for (size_t i = 0; i < P[mi].cand.size(); ++i) {
          std::fprintf(stderr, "s=%6.1f layer %d excl %zu chosen %8.2f cands:", metas[mi].zs[i], P[mi].layer[i], P[mi].excl[i].size(), P[mi].chosen[i]);
          for (const auto &c : P[mi].cand[i]) std::fprintf(stderr, " %.2f(%.1f)", c.z, c.cost);
          std::fprintf(stderr, "\n");
        }
  // ---- 4. smooth (Whittaker); unknown samples are filled by the smoothest curve
  for (size_t mi = 0; mi < metas.size(); ++mi) {
    RoadMeta &m = metas[mi];
    if (m.roadIndex < 0) continue;
    const size_t n = m.zs.size();
    std::vector<double> y(n, kNaN), w(n, 0.0);
    for (size_t i = 0; i < n && i < P[mi].chosen.size(); ++i)
      if (!std::isnan(P[mi].chosen[i])) y[i] = P[mi].chosen[i], w[i] = 1.0;
    double wsum = 0;
    for (double v : w) wsum += v;
    if (wsum < 0.5) {
      m.zz.assign(n, 0.0);
      m.hasData = false;
      continue;
    }
    const double ds = m.zs.back() / std::max<size_t>(1, n - 1);
    double lambda = opt.elevationSmoothing / std::pow(std::max(ds, 0.1), 4.0);
    // robust fit: Tukey biweights drop isolated outliers (walls, stairs, kerb edges, vehicles that
    // survived lidar classification) instead of letting them bend the profile
    std::vector<double> w0 = w;
    for (int it = 0; it < 4; ++it) {
      m.zz = whittaker(y, w, lambda);
      std::vector<double> res;
      for (size_t i = 0; i < n; ++i)
        if (w0[i] > 0) res.push_back(std::abs(y[i] - m.zz[i]));
      if (res.empty()) break;
      std::nth_element(res.begin(), res.begin() + res.size() / 2, res.end());
      const double c = std::max(0.25, 4.685 * 1.4826 * res[res.size() / 2]);
      for (size_t i = 0; i < n; ++i)
        if (w0[i] > 0) {
          const double u = std::abs(y[i] - m.zz[i]) / c;
          w[i] = u < 1 ? (1 - u * u) * (1 - u * u) : 0.0;
        }
    }
    // grade guard: a profile steeper than the measured terrain supports gets more smoothing
    auto maxGrade = [&](const std::vector<double> &z) {
      double g = 0;
      for (size_t i = 5; i < n; ++i) g = std::max(g, std::abs(z[i] - z[i - 5]) / std::max(1e-9, m.zs[i] - m.zs[i - 5]));
      return g;
    };
    for (int it = 0; it < 4 && maxGrade(m.zz) > opt.maxGrade; ++it) {
      lambda *= 4.0;
      m.zz = whittaker(y, w, lambda);
    }
    // cross slope (spec 3 superelevation) where the road lies on the measured ground; smooth and
    // limited to 8 %; the profile above is at the carriageway centre, the reference line is tc away
    std::vector<double> cy(n, kNaN), cw(n, 0.0);
    for (size_t i = 0; i < n; ++i)
      if (!std::isnan(P[mi].cross[i]) && !std::isnan(P[mi].chosen[i]) && std::abs(P[mi].chosen[i] - P[mi].groundC[i]) < 0.3)
        cy[i] = std::clamp(P[mi].cross[i], -0.08, 0.08), cw[i] = 1.0;
    double cws = 0;
    for (double v : cw) cws += v;
    m.xs = cws > 0.5 && opt.target.superelevation ? whittaker(cy, cw, 50.0 * lambda) : std::vector<double>(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
      m.xs[i] = std::clamp(m.xs[i], -0.08, 0.08);
      m.zz[i] -= P[mi].tc[i] * m.xs[i];
    }
    // kept for the re-fit against the junction surfaces (fitRoadElevation)
    m.zy = y, m.zw = w, m.ztc = P[mi].tc, m.zLambda = lambda;
    for (size_t i = 0; i < n; ++i)
      if (w0[i] > 0) {
        m.obs0 = std::min(m.obs0, m.zs[i]);
        m.obs1 = std::min(m.obs1, m.zs.back() - m.zs[i]);
      }
  }
}

void Builder::junctionSurfaces() {
  struct Pt {
    double x, y, z, w;
  };
  // weighted least squares plane z = a + b (x - cx) + c (y - cy) (Cramer); `ridge` pulls the
  // gradient toward 0 where the points do not determine it (two road ends on a straight street)
  auto fit = [](const std::vector<Pt> &pts, const Vec2 &c, double out[3], double ridge) {
    double S[3][3] = {{0}}, B[3] = {0}, W = 0, zm = 0;
    for (const Pt &p : pts) {
      const double v[3] = {1.0, p.x - c.x, p.y - c.y};
      for (int i = 0; i < 3; ++i) {
        B[i] += p.w * v[i] * p.z;
        for (int j = 0; j < 3; ++j) S[i][j] += p.w * v[i] * v[j];
      }
      W += p.w, zm += p.w * p.z;
    }
    S[1][1] += 1e-3 + ridge;
    S[2][2] += 1e-3 + ridge;
    auto det3 = [](double m[3][3]) {
      return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
             m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    out[0] = W > 0 ? zm / W : 0.0, out[1] = out[2] = 0.0;
    const double D = det3(S);
    if (std::abs(D) < 1e-12) return;
    for (int k = 0; k < 3; ++k) {
      double M[3][3];
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M[i][j] = j == k ? B[i] : S[i][j];
      out[k] = det3(M) / D;
    }
  };
  auto weightSum = [](const std::vector<Pt> &pts) {
    double w = 0;
    for (const Pt &q : pts) w += q.w;
    return w;
  };
  // robust plane (Tukey biweight, residual scale `c` metres): outliers get weight 0
  auto robustFit = [&](std::vector<Pt> &pts, const Vec2 &ctr, double sol[3], double c, double ridgeRel, double R2) {
    const std::vector<Pt> p0 = pts;
    fit(pts, ctr, sol, ridgeRel * weightSum(pts) * R2);
    for (int it = 0; it < 5 && pts.size() > 2; ++it) {
      bool any = false;
      for (size_t k = 0; k < pts.size(); ++k) {
        const double r = std::abs(p0[k].z - (sol[0] + sol[1] * (p0[k].x - ctr.x) + sol[2] * (p0[k].y - ctr.y)));
        const double u = r / c;
        pts[k].w = p0[k].w * (u < 1 ? (1 - u * u) * (1 - u * u) : 0.0);
        any |= pts[k].w > 0;
      }
      if (!any) {
        pts = p0;
        fit(pts, ctr, sol, ridgeRel * weightSum(pts) * R2);
        return;
      }
      fit(pts, ctr, sol, ridgeRel * weightSum(p0) * R2);
    }
  };
  for (Cluster &cl : clusters) {
    if (cl.ends.empty()) continue;
    std::vector<Pt> pts;
    double R = 0;
    for (const RoadEnd &e : cl.ends) {
      const RoadMeta &m = metas[e.road];
      if (m.roadIndex < 0) continue;
      const odr::Road &r = od.roads[m.roadIndex];
      const Vec2 p = r.planView.eval(e.atEnd ? r.length() : 0.0).pos();
      R = std::max(R, (p - cl.center).norm());
      if (!m.hasData || m.zz.empty()) continue;
      // a road end votes with its class and only as far as it was measured near the junction
      // (an end extrapolated over a covered stretch must not tilt the plane)
      const double obs = e.atEnd ? m.obs1 : m.obs0;
      const double wObs = obs <= 8.0 ? 1.0 : (obs <= 30.0 ? 0.2 : 0.02);
      pts.push_back({p.x, p.y, e.atEnd ? m.zz.back() : m.zz.front(), 5.0 * wObs * (1.0 + m.rank)});
    }
    if (pts.empty()) continue;  // no measured road here: filled below from neighbours
    const double R2 = std::max(25.0, R * R);
    // robust ground plane over the junction area (walls, kerbs and building edges rejected)
    std::vector<Pt> ground;
    double gs[3] = {kNaN, 0.0, 0.0};
    if (opt.terrain && !opt.terrain->ground.empty()) {
      const double Rg = std::max(R, 6.0);
      for (double dx = -Rg; dx <= Rg; dx += 1.0)
        for (double dy = -Rg; dy <= Rg; dy += 1.0) {
          if (dx * dx + dy * dy > Rg * Rg) continue;
          const double x = cl.center.x + dx, y = cl.center.y + dy, z = opt.terrain->ground_at(x, y);
          if (!std::isnan(z)) ground.push_back({x, y, z, 1.0});
        }
      if (ground.size() >= 12) robustFit(ground, cl.center, gs, 0.5, 1e-3, Rg * Rg);
      else ground.clear();
    }
    // 1. the junction level: the height supported by the most evidence - road ends (by class and
    //    measurement) and the ground at the centre. Ends that cannot reach it within a plausible
    //    junction grade (12 %) are outliers (a road end on a wall top, a deck, a roof): they conform
    //    to the junction through the re-fit of their profile instead of tilting the surface
    const double wRoad0 = weightSum(pts);
    // candidate level: height h at position c; an end agrees if it is within a junction grade of it
    auto agrees = [&](const Pt &q, double h, const Vec2 &c) { return std::abs(q.z - h) <= 0.5 + 0.12 * (Vec2(q.x, q.y) - c).norm(); };
    auto support = [&](double h, const Vec2 &c) {
      double sc = 0;
      for (const Pt &q : pts)
        if (agrees(q, h, c)) sc += q.w;
      if (!std::isnan(gs[0]) && std::abs(gs[0] - h) <= 0.5 + 0.12 * (cl.center - c).norm()) sc += 0.5 * wRoad0;
      return sc;
    };
    double level = pts.front().z, best = -1;
    Vec2 at(pts.front().x, pts.front().y);
    for (const Pt &q : pts)
      if (const double sc = support(q.z, Vec2(q.x, q.y)); sc > best) best = sc, level = q.z, at = Vec2(q.x, q.y);
    if (!std::isnan(gs[0]))
      if (const double sc = support(gs[0], cl.center); sc > best) best = sc, level = gs[0], at = cl.center;
    std::vector<Pt> in;
    for (const Pt &q : pts)
      if (agrees(q, level, at)) in.push_back(q);
    if (in.empty()) in = pts;
    double sol[3];
    robustFit(in, cl.center, sol, 2.0, 0.01, R2);
    const double wRoad = weightSum(in);
    // 2. the measured ground on that level refines the gradient that two or three road ends cannot
    //    determine; ground under a viaduct, walls and embankments (> 1 m off the plane) are excluded
    if (!ground.empty()) {
      std::vector<Pt> on;
      for (const Pt &q : ground)
        if (q.w > 0 && std::abs(q.z - (sol[0] + sol[1] * (q.x - cl.center.x) + sol[2] * (q.y - cl.center.y))) < 1.0)
          on.push_back(q);
      if (on.size() >= 12 && weightSum(on) > 1e-9) {
        const double wg = wRoad / weightSum(on);
        for (Pt &q : on) q.w *= wg, in.push_back(q);
        fit(in, cl.center, sol, 1e-3 * wRoad * R2);
      }
    }
    const double g = std::hypot(sol[1], sol[2]);
    if (const char *dbg = std::getenv("ODRFORGE_DEBUG_PLANE"); dbg && (g > 0.2 || std::string(dbg) == "all")) {
      std::fprintf(stderr, "plane at (%.1f, %.1f) R %.1f: z %.2f grad %.3f %.3f\n", cl.center.x, cl.center.y, R, sol[0], sol[1], sol[2]);
      for (const RoadEnd &e : cl.ends) {
        const RoadMeta &m = metas[e.road];
        if (m.roadIndex < 0) continue;
        const odr::Road &r = od.roads[m.roadIndex];
        const Vec2 p = r.planView.eval(e.atEnd ? r.length() : 0.0).pos();
        std::fprintf(stderr, "   road %s %s at (%.1f, %.1f) z %.2f obs %.1f rank %d\n", r.id.c_str(), e.atEnd ? "end" : "start",
                     p.x, p.y, m.zz.empty() ? 0.0 : (e.atEnd ? m.zz.back() : m.zz.front()), e.atEnd ? m.obs1 : m.obs0, m.rank);
      }
    }
    if (g > 0.15) sol[1] *= 0.15 / g, sol[2] *= 0.15 / g;
    if (!opt.target.superelevation) sol[1] = sol[2] = 0.0;  // flat cross sections: a level junction
    cl.plane[0] = sol[0], cl.plane[1] = sol[1], cl.plane[2] = sol[2];
    cl.hasPlane = true;
  }
  // roads without any measurement (outside the terrain data, fully in tunnels): heights from the
  // junction surfaces at their ends (linear in between), so they join their neighbours seamlessly
  for (RoadMeta &m : metas) {
    if (m.roadIndex < 0 || m.hasData) continue;
    const odr::Road &r = od.roads[m.roadIndex];
    const Chain &ch = chains[m.chain];
    const bool h0 = ch.c0 >= 0 && clusters[ch.c0].hasPlane, h1 = ch.c1 >= 0 && clusters[ch.c1].hasPlane;
    const double z0 = h0 ? planeZ(clusters[ch.c0], r.planView.eval(0).pos()) : 0.0;
    const double z1 = h1 ? planeZ(clusters[ch.c1], r.planView.eval(r.length()).pos()) : 0.0;
    const double a = h0 ? z0 : (h1 ? z1 : 0.0), b = h1 ? z1 : a;
    for (size_t i = 0; i < m.zs.size(); ++i) m.zz[i] = a + (b - a) * m.zs[i] / std::max(1e-9, r.length());
  }
  // every junction gets a surface: the ones without a measured road use the propagated heights
  for (Cluster &cl : clusters) {
    if (cl.hasPlane || cl.ends.empty()) continue;
    std::vector<Pt> pts;
    for (const RoadEnd &e : cl.ends) {
      const RoadMeta &m = metas[e.road];
      if (m.roadIndex < 0 || m.zz.empty()) continue;
      const odr::Road &r = od.roads[m.roadIndex];
      const Vec2 p = r.planView.eval(e.atEnd ? r.length() : 0.0).pos();
      pts.push_back({p.x, p.y, e.atEnd ? m.zz.back() : m.zz.front(), 1.0});
    }
    if (pts.empty()) continue;
    double sol[3];
    fit(pts, cl.center, sol, 0.05 * weightSum(pts) * 100.0);
    if (!opt.target.superelevation) sol[1] = sol[2] = 0.0;
    cl.plane[0] = sol[0], cl.plane[1] = sol[1], cl.plane[2] = sol[2];
    cl.hasPlane = true;
  }
}

double Builder::planeZ(const Cluster &c, const Vec2 &p) const {
  return c.plane[0] + c.plane[1] * (p.x - c.center.x) + c.plane[2] * (p.y - c.center.y);
}

void Builder::fitRoadElevation() {
  const double blend = opt.junctionBlend;
  for (RoadMeta &m : metas) {
    if (m.roadIndex < 0) continue;
    odr::Road &r = od.roads[m.roadIndex];
    const double L = r.length();
    const int n = static_cast<int>(m.zs.size());
    std::vector<double> z = m.zz;
    const Chain &ch = chains[m.chain];
    // re-fit the measured profile with the junction surfaces as boundary conditions (height and
    // gradient): a level difference is spread over the whole road by the same smoother instead of
    // being forced into the short end blend below (which then only removes centimetre residuals)
    if (opt.junctionSurfaces && n >= 3 && m.zy.size() == static_cast<size_t>(n)) {
      std::vector<double> y = m.zy, w = m.zw;
      bool any = false;
      for (bool atEnd : {false, true}) {
        const int ci = atEnd ? ch.c1 : ch.c0;
        if (ci < 0 || !clusters[ci].hasPlane) continue;
        const Cluster &cl = clusters[ci];
        const Pose p = r.planView.eval(atEnd ? L : 0.0);
        const double zT = planeZ(cl, p.pos()), slopeT = Vec2(cl.plane[1], cl.plane[2]).dot(p.dir());
        const int ic = atEnd ? n - 1 : 0, in = atEnd ? n - 2 : 1;
        const double xs0 = m.xs.empty() ? 0.0 : m.xs[ic], xs1 = m.xs.empty() ? 0.0 : m.xs[in];
        y[ic] = zT + m.ztc[ic] * xs0, w[ic] = 1e4;  // heights at the carriageway centre
        y[in] = zT + slopeT * (m.zs[in] - m.zs[ic]) + m.ztc[in] * xs1, w[in] = 1e4;
        any = true;
      }
      if (any) {
        // grade guard as for the measured profile: a level difference is spread, not compressed
        auto maxGrade = [&](const std::vector<double> &v) {
          double g = 0;
          const int k = std::min(5, n - 1);
          for (int i = k; i < n; ++i) g = std::max(g, std::abs(v[i] - v[i - k]) / std::max(1e-9, m.zs[i] - m.zs[i - k]));
          return g;
        };
        const double gMax = std::max(opt.maxGrade, maxGrade(m.zz));
        double lambda = m.zLambda;
        z = whittaker(y, w, lambda);
        for (int it = 0; it < 6 && maxGrade(z) > gMax; ++it) {
          lambda *= 4.0;
          z = whittaker(y, w, lambda);
        }
        for (int i = 0; i < n; ++i) z[i] -= m.ztc[i] * (m.xs.empty() ? 0.0 : m.xs[i]);
      }
    }
    // very short roads (stubs left by junction trimming) carry no usable profile of their own: they
    // continue the junction surface(s) at their ends (cubic Hermite between them)
    if (opt.junctionSurfaces && L < 6.0) {
      const bool h0 = ch.c0 >= 0 && clusters[ch.c0].hasPlane, h1 = ch.c1 >= 0 && clusters[ch.c1].hasPlane;
      if (h0 || h1) {
        const Pose p0 = r.planView.eval(0.0), p1 = r.planView.eval(L);
        auto at = [&](const Cluster &cl, const Pose &p, double &zv, double &sl) {
          zv = planeZ(cl, p.pos());
          sl = Vec2(cl.plane[1], cl.plane[2]).dot(p.dir());
        };
        double z0 = 0, s0 = 0, z1 = 0, s1 = 0;
        if (h0) at(clusters[ch.c0], p0, z0, s0);
        if (h1) at(clusters[ch.c1], p1, z1, s1);
        if (!h0) z0 = z1 - s1 * L, s0 = s1;
        if (!h1) z1 = z0 + s0 * L, s1 = s0;
        const odr::Poly3 hp = odr::Poly3::hermite(0.0, L, z0, s0, z1, s1);
        for (int i = 0; i < n; ++i) z[i] = hp.eval(m.zs[i]);
      }
    }
    // slopes of the smoothed profile (central differences)
    std::vector<double> dz(n, 0.0);
    for (int i = 0; i < n; ++i) {
      const int a = std::max(0, i - 1), b = std::min(n - 1, i + 1);
      dz[i] = (z[b] - z[a]) / std::max(1e-9, m.zs[b] - m.zs[a]);
    }
    // blend both ends into their junction planes: value, gradient (C1) and banking
    double seStart = 0, seEnd = 0;
    const double B = std::min(blend, 0.45 * L);
    for (bool atEnd : {false, true}) {
      const int ci = atEnd ? ch.c1 : ch.c0;
      if (ci < 0 || !clusters[ci].hasPlane || !opt.junctionSurfaces) continue;
      const Cluster &cl = clusters[ci];
      const double sc = atEnd ? L : 0.0;
      const Pose p = r.planView.eval(sc);
      const Vec2 grad(cl.plane[1], cl.plane[2]);
      const double zT = planeZ(cl, p.pos());
      const double slopeT = grad.dot(p.dir());  // d/ds along +s
      const int ic = atEnd ? n - 1 : 0;
      const double dZ = zT - z[ic], dS = slopeT - dz[ic];
      if (!std::isfinite(dZ) || !std::isfinite(dS))
        std::fprintf(stderr, "non-finite blend road %s %s: zT %f slopeT %f z %f dz %f plane %f %f %f\n", r.id.c_str(),
                     atEnd ? "end" : "start", zT, slopeT, z[ic], dz[ic], cl.plane[0], cl.plane[1], cl.plane[2]);
      if (std::getenv("ODRFORGE_DEBUG_BLEND"))
        std::fprintf(stderr, "blend road %s %s: dZ %.3f dSlope %.3f B %.1f planeGrad %.3f\n", r.id.c_str(),
                     atEnd ? "end" : "start", dZ, dS, B, grad.norm());
      for (int i = 0; i < n; ++i) {
        const double u = atEnd ? (m.zs[i] - (L - B)) / B : (B - m.zs[i]) / B;  // 1 at the contact, 0 at blend end
        if (u <= 0 || B <= 1e-6) continue;
        // cubic Hermite correction: value dZ and slope dS at the contact, 0 / 0 at the inner end
        const double d = atEnd ? (L - m.zs[i]) : m.zs[i];  // distance from the contact
        const double h = smoothstepC1(std::min(u, 1.0));
        z[i] += dZ * h + (atEnd ? 1.0 : 1.0) * dS * (atEnd ? -d : d) * u * u;
      }
      dz[ic] = slopeT;
      if (opt.target.superelevation) (atEnd ? seEnd : seStart) = std::atan(grad.dot(p.left()));
    }
    // C1 cubic elevation records: knots every ~10 m, slopes from the profile
    for (int i = 1; i + 1 < n; ++i) dz[i] = (z[i + 1] - z[i - 1]) / std::max(1e-9, m.zs[i + 1] - m.zs[i - 1]);
    const int every = std::max(1, static_cast<int>(std::round(opt.elevationKnotSpacing / std::max(1e-9, L / std::max(1, n - 1)))));
    std::vector<int> knots;
    for (int i = 0; i < n; i += every) knots.push_back(i);
    if (knots.back() != n - 1) knots.push_back(n - 1);
    if (knots.size() >= 3 && m.zs[n - 1] - m.zs[knots[knots.size() - 2]] < 0.5) knots.erase(knots.end() - 2);
    r.elevation.clear();
    for (size_t k = 0; k + 1 < knots.size(); ++k) {
      const int a = knots[k], b = knots[k + 1];
      r.elevation.push_back(odr::Poly3::hermite(m.zs[a], m.zs[b] - m.zs[a], z[a], dz[a], z[b], dz[b]));
    }
    if (r.elevation.empty()) r.elevation.push_back({0.0, z.empty() ? 0.0 : z[0], 0, 0, 0});
    // banking toward the junction planes (spec 3 superelevation), flat in between
    r.superelevation.clear();
    {
      std::vector<double> se(n, 0.0), dse(n, 0.0);
      for (int i = 0; i < n; ++i) se[i] = m.xs.empty() ? 0.0 : std::atan(m.xs[i]);
      // blend to the junction planes' banking at the ends (C1)
      const double e0 = (ch.c0 >= 0 && clusters[ch.c0].hasPlane && opt.junctionSurfaces) ? seStart - se[0] : 0.0;
      const double e1 = (ch.c1 >= 0 && clusters[ch.c1].hasPlane && opt.junctionSurfaces) ? seEnd - se[n - 1] : 0.0;
      for (int i = 0; i < n; ++i) {
        if (B > 1e-6 && m.zs[i] < B) se[i] += e0 * smoothstepC1(1.0 - m.zs[i] / B);
        if (B > 1e-6 && m.zs[i] > L - B) se[i] += e1 * smoothstepC1((m.zs[i] - (L - B)) / B);
      }
      for (int i = 0; i < n; ++i) {
        const int a = std::max(0, i - 1), b = std::min(n - 1, i + 1);
        dse[i] = (se[b] - se[a]) / std::max(1e-9, m.zs[b] - m.zs[a]);
      }
      bool flat = true;
      for (double v : se) flat = flat && std::abs(v) < 1e-7;
      if (!flat)
        for (size_t k = 0; k + 1 < knots.size(); ++k) {
          const int a = knots[k], b = knots[k + 1];
          r.superelevation.push_back(odr::Poly3::hermite(m.zs[a], m.zs[b] - m.zs[a], se[a], dse[a], se[b], dse[b]));
        }
    }
  }
}

void Builder::connectingElevation(odr::Road &c, const Cluster &cl, std::optional<std::pair<double, double>> flatEnds) const {
  // connecting roads lie on the junction plane: height and banking follow it exactly. With flat
  // cross sections (flatEnds: the road heights at both ends) there is no banking: the height is the
  // plane plus a smoothstep correction to those heights (C1: zero correction slope at the ends)
  const double L = c.length();
  const Vec2 grad(cl.plane[1], cl.plane[2]);
  const int n = std::max(2, static_cast<int>(std::ceil(L / 2.0)) + 1);
  c.elevation.clear();
  c.superelevation.clear();
  std::vector<double> s(n), z(n), dz(n), se(n), dse(n);
  for (int i = 0; i < n; ++i) {
    s[i] = L * i / (n - 1);
    const Pose p = c.planView.eval(s[i]);
    z[i] = planeZ(cl, p.pos());
    dz[i] = grad.dot(p.dir());
    if (flatEnds) {
      const double e0 = flatEnds->first - planeZ(cl, c.planView.eval(0.0).pos());
      const double e1 = flatEnds->second - planeZ(cl, c.planView.eval(L).pos());
      const double u = s[i] / std::max(1e-9, L);
      z[i] += e0 + (e1 - e0) * smoothstepC1(u);
      dz[i] += (e1 - e0) * 6.0 * u * (1.0 - u) / std::max(1e-9, L);
      se[i] = dse[i] = 0.0;
      continue;
    }
    const double g = grad.dot(p.left());
    se[i] = std::atan(g);
    // d/ds atan(grad . left(s)) = -kappa (grad . dir) / (1 + g^2)
    dse[i] = -p.kappa * grad.dot(p.dir()) / (1.0 + g * g);
  }
  for (int i = 0; i + 1 < n; ++i) {
    c.elevation.push_back(odr::Poly3::hermite(s[i], s[i + 1] - s[i], z[i], dz[i], z[i + 1], dz[i + 1]));
    c.superelevation.push_back(odr::Poly3::hermite(s[i], s[i + 1] - s[i], se[i], dse[i], se[i + 1], dse[i + 1]));
  }
  // drop all-zero banking (flat junction)
  bool flat = true;
  for (double v : se) flat = flat && std::abs(v) < 1e-9;
  if (flat) c.superelevation.clear();
}

}  // namespace detail
}  // namespace odrforge
