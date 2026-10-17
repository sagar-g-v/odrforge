// OdrForge - road elevation from terrain / structure surfaces, junction surfaces and banking.
//
// Spec basis (docs/research/opendrive_spec_notes.md):
//  - OpenDRIVE 1.8.0 §10.5.1 / §10.5.2 (1.4H §5.3.5.1 / §5.3.6.1): <elevation s a b c d> with absolute s;
//    <superelevation> = roll angle about the reference
//    line, positive falling to the right, so the surface height at lateral t is z(s) + t tan(se(s)).
//  - pitfall 11: C1 elevation joins and matching z at links.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Whittaker smoother: minimise sum w_i (z_i - y_i)^2 + lambda sum (second difference of z)^2.
/// Unknown samples have w = 0 and are filled with the smoothest (cubic-spline-like) curve.
std::vector<double> whittaker(const std::vector<double> &y, const std::vector<double> &w, double lambda) {
  const size_t n = y.size();
  size_t observed = 0;
  for (size_t i = 0; i < n; ++i) observed += w[i] > 0 && !std::isnan(y[i]);
  // too short to smooth, or a single observation (the straight lines the second-difference penalty
  // leaves free are not determined by one point): weighted mean of the known samples everywhere
  if (n < 3 || observed < 2) {
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

/// lateral position of the carriageway centre (driving lanes) at s, and optionally its half width
double carriagewayCentre(const odr::Road &r, double s, double *half = nullptr) {
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
  if (half) *half = lo > hi ? 0.0 : 0.5 * (hi - lo);
  return lo > hi ? odr::laneOffsetAt(r, s) : 0.5 * (lo + hi);
}

double smoothstepC1(double u) { return u * u * (3.0 - 2.0 * u); }

/// Profile z at stations s beyond the first / last measured sample (weight > 0) continues from the
/// profile at that sample at the grade of a least-squares line through the measurements y within
/// `window` of it (limited to maxGrade): a ramp keeps climbing, one noisy sample does not tilt it.
void extendUnmeasuredEnds(std::vector<double> &z, const std::vector<double> &s, const std::vector<double> &y,
                          const std::vector<double> &weight, double window, double maxGrade) {
  const auto first = std::find_if(weight.begin(), weight.end(), [](double v) { return v > 0.0; });
  if (first == weight.end()) return;
  const size_t i0 = static_cast<size_t>(first - weight.begin());
  size_t i1 = weight.size() - 1;
  while (weight[i1] <= 0.0) --i1;
  auto gradeNear = [&](size_t k) {
    double sw = 0, ss = 0, sz = 0, sss = 0, ssz = 0;
    for (size_t i = i0; i <= i1; ++i)
      if (weight[i] > 0.0 && std::abs(s[i] - s[k]) <= window)
        sw += weight[i], ss += weight[i] * s[i], sz += weight[i] * y[i], sss += weight[i] * s[i] * s[i], ssz += weight[i] * s[i] * y[i];
    const double den = sw * sss - ss * ss;
    return den > 1e-9 ? std::clamp((sw * ssz - ss * sz) / den, -maxGrade, maxGrade) : 0.0;
  };
  const double g0 = gradeNear(i0), g1 = gradeNear(i1);
  for (size_t i = 0; i < i0; ++i) z[i] = z[i0] + g0 * (s[i] - s[i0]);
  for (size_t i = i1 + 1; i < z.size(); ++i) z[i] = z[i1] + g1 * (s[i] - s[i1]);
}

/// heights along a road on flat ground (z = 0) where some samples are pinned to a level (NaN: free).
/// Runs of samples pinned to one height are plateaus; a plateau meets the ground through a C1 ramp
/// on either side that is nowhere steeper than `grade` (a smoothstep is steepest in its middle, at
/// 1.5 times its mean grade), and two plateaus closer than their ramps are joined directly.
std::vector<double> levelProfile(const std::vector<double> &s, const std::vector<double> &pin, double grade) {
  const size_t n = s.size();
  std::vector<double> z(n, 0.0);
  struct Run {
    size_t i0, i1;
    double z;
  };
  std::vector<Run> runs;
  for (size_t i = 0; i < n; ++i) {
    if (std::isnan(pin[i])) continue;
    if (!runs.empty() && runs.back().i1 + 1 == i && runs.back().z == pin[i]) runs.back().i1 = i;
    else runs.push_back({i, i, pin[i]});
    z[i] = pin[i];
  }
  // free samples between run a (or the road start) and run b (or the road end)
  for (size_t k = 0; k <= runs.size(); ++k) {
    const Run *a = k > 0 ? &runs[k - 1] : nullptr, *b = k < runs.size() ? &runs[k] : nullptr;
    const size_t i0 = a ? a->i1 + 1 : 0, i1 = b ? b->i0 : n;
    if (i0 >= i1 || (!a && !b)) continue;
    const double sa = a ? s[a->i1] : 0.0, sb = b ? s[b->i0] : 0.0, za = a ? a->z : 0.0, zb = b ? b->z : 0.0;
    const double ra = 1.5 * std::abs(za) / grade, rb = 1.5 * std::abs(zb) / grade;
    for (size_t i = i0; i < i1; ++i) {
      if (a && b && sb - sa < ra + rb) {
        z[i] = za + (zb - za) * smoothstepC1((s[i] - sa) / (sb - sa));
        continue;
      }
      double v = 0.0;
      if (a && s[i] - sa < ra) v += za * (1.0 - smoothstepC1((s[i] - sa) / ra));
      if (b && sb - s[i] < rb) v += zb * (1.0 - smoothstepC1((sb - s[i]) / rb));
      z[i] = v;
    }
  }
  return z;
}

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
    std::vector<double> headroom;  // OSM maxheight per sample (0: not tagged)
    std::vector<double> half;      // carriageway half width per sample
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
    pf.headroom.assign(n, 0.0);
    pf.half.assign(n, 0.0);
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
      const double tcen = carriagewayCentre(r, s, &pf.half[i]);
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
        if (s + m.trimStart >= lr.s0 - 1e-6 && s + m.trimStart <= lr.s1 + 1e-6) pf.layer[i] = lr.layer, pf.headroom[i] = lr.maxHeight;
      if (!haveTerrain) continue;
      // the terrain above a tunnel or a covered way (under a building, through a hillside) is not the
      // road: interpolated between the portals. A building passage (OSM wiki Key:tunnel, tunnel=building_passage:
      // through a building at ground level) is: a bare-earth terrain model carries the ground under the building
      const bool bridge = inRanges(m.bridges, s);
      const bool tunnel = (inRanges(m.tunnels, s) && !inRanges(m.passages, s)) || inRanges(m.covered, s);
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
  // ---- 3a. no elevation data: the ground is flat (z = 0) and the OSM layers are the only evidence.
  //          Where a road's carriageway overlaps another road of another layer without a junction,
  //          both are pinned to their levels and ramps lead there from the ground; a ramp that reaches
  //          a junction lifts (lowers) the junction and continues on its other roads. Nothing else is
  //          invented (a bridge over a river stays level).
  //          Over measured terrain (a ground grid) the same levels are offsets from the measured ground: what the
  //          terrain cannot measure (a bridge without deck data, a tunnel) lies that far above / below the smoothest
  //          curve through the measured road (4.)
  std::vector<std::vector<double>> rel(P.size());
  if (!haveTerrain || opt.terrain) {
    const double goreLength = 40.0;
    // design grade of an invented ramp by road class (at its steepest): a motorway climbs gently, a
    // garage ramp steeply
    auto rampGrade = [&](size_t mi) {
      const int r = metas[mi].rank;
      return r >= 9 ? 0.05 : r >= 6 ? 0.06 : r >= 4 ? 0.08 : 0.12;
    };
    // where structures leave less room a ramp may be half as steep again (15 % at most): the mean
    // grade between two levels whose smoothstep then reaches that at its steepest
    auto steepMean = [&](size_t mi) { return std::min(1.5 * rampGrade(mi), 0.15) / 1.5; };
    // roads that leave one junction side by side (a split, a merge) share its level there
    auto nearSharedEnd = [&](size_t mi, size_t i, size_t mj, size_t j) {
      const Chain &ca = chains[metas[mi].chain], &cb = chains[metas[mj].chain];
      const double sa = metas[mi].zs[i], sb = metas[mj].zs[j];
      for (int ea : {0, 1})
        for (int eb : {0, 1}) {
          const int ka = ea ? ca.c1 : ca.c0, kb = eb ? cb.c1 : cb.c0;
          if (ka < 0 || ka != kb) continue;
          if ((ea ? metas[mi].zs.back() - sa : sa) < goreLength && (eb ? metas[mj].zs.back() - sb : sb) < goreLength) return true;
        }
      return false;
    };
    std::map<std::pair<long, long>, std::vector<std::pair<size_t, size_t>>> grid;
    for (size_t mi = 0; mi < P.size(); ++mi)
      for (size_t i = 0; i < P[mi].xy.size(); ++i)
        grid[{static_cast<long>(std::floor(P[mi].xy[i].x / cell)), static_cast<long>(std::floor(P[mi].xy[i].y / cell))}]
            .push_back({mi, i});
    // 1. levels where roads cross. OSM layers are an order, not heights: a structure rises one storey
    //    above each road it crosses that lies below it (a bridge over a street: one storey; over a
    //    lower bridge: on top of that one) and stays on the ground when only tunnels pass beneath; a
    //    tunnel sinks one storey below each road above it. A storey is the headroom of the lower road
    //    plus the deck: levelHeight, or the signed maxheight (below the real headroom) plus 1 m and at
    //    least 3 m (stacked garage decks). Every crossing has a level of its own - a viaduct, or a
    //    ramp mapped as one bridge, is not one height along its length -, tied to the other levels
    //    along the road by its grade (3.).
    auto storey = [&](double headroom) { return headroom > 0.0 ? std::clamp(headroom + 1.0, 3.0, opt.levelHeight) : opt.levelHeight; };
    std::vector<std::vector<size_t>> stretchOf(P.size());  // stretch of constant layer of each sample
    for (size_t mi = 0; mi < P.size(); ++mi) {
      const auto &ly = P[mi].layer;
      stretchOf[mi].resize(ly.size());
      for (size_t i = 1; i < ly.size(); ++i) stretchOf[mi][i] = stretchOf[mi][i - 1] + (ly[i] != ly[i - 1]);
    }
    // the stretch of layer l at sample i of road mi or, where the tag ends short of the crossing, the
    // nearest one of that layer
    auto stretchAt = [&](size_t mi, size_t i, int l) {
      const auto &ly = P[mi].layer;
      size_t k = i;
      for (size_t d = 0; d < ly.size() && ly[k] != l; ++d) {
        if (i >= d && ly[i - d] == l) k = i - d;
        else if (i + d < ly.size() && ly[i + d] == l) k = i + d;
      }
      return stretchOf[mi][k];
    };
    // the levels: 0 is the ground, then one per crossing of a road stretch over or under another road
    // - the height of the structure there, or the ceiling of a street (layer 0) under a structure
    std::vector<double> x0{0.0};  // preferred heights
    std::vector<int> layerOf{0};
    std::vector<size_t> stretchOfVar{0}, roadOfVar{0};
    std::vector<Vec2> where{Vec2()};                     // a place of each level, for the report
    std::vector<std::pair<double, double>> span{{0, 0}};  // the samples of the crossing along its road
    std::map<std::tuple<size_t, size_t, size_t>, std::vector<size_t>> varsOf;  // (road, stretch, road crossed)
    auto var = [&](size_t mi, size_t i, int l, size_t mj) {
      const size_t k = stretchAt(mi, i, l);
      const double s = metas[mi].zs[i];
      std::vector<size_t> &vs = varsOf[std::make_tuple(mi, k, mj)];
      // samples of one crossing within a few metres share a level: a road that crosses another twice,
      // or runs above (below) it for a while, has a level every few metres, free to ramp in between
      for (size_t v : vs)
        if (std::max(span[v].second, s) - std::min(span[v].first, s) <= cell) {
          span[v] = {std::min(span[v].first, s), std::max(span[v].second, s)};
          return v;
        }
      vs.push_back(x0.size());
      x0.push_back(0.0), layerOf.push_back(l), stretchOfVar.push_back(k), roadOfVar.push_back(mi), where.push_back(P[mi].xy[i]);
      span.push_back({s, s});
      return vs.back();
    };
    struct Crossing {
      size_t road, i, v;  // the level at sample i of the road
      bool operator<(const Crossing &o) const { return std::tie(road, i, v) < std::tie(o.road, o.i, o.v); }
    };
    std::set<Crossing> crossings;
    std::map<std::pair<size_t, size_t>, double> below;  // (lower, upper) -> storey between them
    for (size_t mi = 0; mi < P.size(); ++mi) {
      if (metas[mi].roadIndex < 0) continue;
      const Prof &a = P[mi];
      for (size_t i = 0; i < a.xy.size(); ++i) {
        const long cx = static_cast<long>(std::floor(a.xy[i].x / cell)), cy = static_cast<long>(std::floor(a.xy[i].y / cell));
        for (long dx = -1; dx <= 1; ++dx)
          for (long dy = -1; dy <= 1; ++dy) {
            auto it = grid.find({cx + dx, cy + dy});
            if (it == grid.end()) continue;
            for (const auto &[mj, j] : it->second) {
              const Prof &b = P[mj];
              if (mj == mi || (a.xy[i] - b.xy[j]).norm() > b.half[j] + 1.0) continue;  // a's centre on b
              int la = a.layer[i], lb = b.layer[j];
              // a bridge / tunnel way that ends a few metres short of the crossing still decides
              if (la == lb && crossing(a, i, b, j)) {
                if (layerHi[mi][i] != layerHi[mj][j]) la = layerHi[mi][i], lb = layerHi[mj][j];
                else if (layerLo[mi][i] != layerLo[mj][j]) la = layerLo[mi][i], lb = layerLo[mj][j];
              }
              if (la == lb || nearSharedEnd(mi, i, mj, j)) continue;  // one level: nothing to separate
              if (const char *dbg = std::getenv("ODRFORGE_DEBUG_ROAD"); dbg && od.roads[metas[mi].roadIndex].id == dbg)
                std::fprintf(stderr, "s=%6.1f layer %d crosses road %s (layer %d)\n", metas[mi].zs[i], la,
                             od.roads[metas[mj].roadIndex].id.c_str(), lb);
              // a street over a tunnel stays on the ground (level 0)
              const size_t va = la == 0 && lb < 0 ? 0 : var(mi, i, la, mj), vb = lb == 0 && la < 0 ? 0 : var(mj, j, lb, mi);
              const double h = storey(la < lb ? a.headroom[i] : b.headroom[j]);
              const auto key = la < lb ? std::make_pair(va, vb) : std::make_pair(vb, va);
              below[key] = std::max(below[key], h);
              if (va != 0) crossings.insert({mi, i, va});
              if (vb != 0) crossings.insert({mj, j, vb});
            }
          }
      }
    }
    // preferred heights: structures above the ground bottom up, then tunnels top down; streets stay
    // on the ground. The stretches of one layer that meet in junctions (a viaduct and its ramps, the
    // aisles of a garage deck) are one structure: it keeps its level where nothing beneath needs it
    // (over a tunnel, between its crossings) - only a preference, 3. ramps it where it has to
    {
      std::vector<size_t> first(P.size() + 1, 0);  // index of each road's first stretch
      for (size_t mi = 0; mi < P.size(); ++mi) first[mi + 1] = first[mi] + (stretchOf[mi].empty() ? 0 : stretchOf[mi].back() + 1);
      std::vector<size_t> parent(first.back());
      for (size_t k = 0; k < parent.size(); ++k) parent[k] = k;
      auto root = [&](size_t k) {
        while (parent[k] != k) k = parent[k] = parent[parent[k]];
        return k;
      };
      std::map<std::pair<int, int>, size_t> atJunction;  // (cluster, layer) -> a stretch ending there
      for (size_t mi = 0; mi < P.size(); ++mi) {
        if (metas[mi].roadIndex < 0 || P[mi].layer.empty()) continue;
        const Chain &ch = chains[metas[mi].chain];
        for (const auto &[c, i] : {std::pair<int, size_t>{ch.c0, 0}, {ch.c1, P[mi].layer.size() - 1}}) {
          if (c < 0 || P[mi].layer[i] == 0) continue;
          const size_t k = first[mi] + stretchOf[mi][i];
          const auto [it, added] = atJunction.emplace(std::make_pair(c, P[mi].layer[i]), k);
          if (!added) parent[root(k)] = root(it->second);
        }
      }
      std::map<int, std::vector<size_t>> byLayer;
      for (size_t v = 1; v < x0.size(); ++v)
        if (layerOf[v] != 0) byLayer[layerOf[v]].push_back(v);
      std::vector<int> order;
      for (const auto &[l, vs] : byLayer) order.push_back(l);
      std::sort(order.begin(), order.end(), [](int p, int q) { return (p > 0) != (q > 0) ? p > 0 : (p > 0 ? p < q : p > q); });
      for (const int l : order) {
        const bool up = l > 0;
        std::map<size_t, double> extreme;  // per structure
        for (size_t v : byLayer[l]) {
          double z = 0.0;
          for (const auto &[key, h] : below) {
            if (up && key.second == v && layerOf[key.first] >= 0) z = std::max(z, x0[key.first] + h);
            if (!up && key.first == v) z = std::min(z, x0[key.second] - h);
          }
          x0[v] = z;
          const size_t g = root(first[roadOfVar[v]] + stretchOfVar[v]);
          const auto [it, added] = extreme.emplace(g, z);
          if (!added) it->second = up ? std::max(it->second, z) : std::min(it->second, z);
        }
        for (size_t v : byLayer[l]) x0[v] = extreme[root(first[roadOfVar[v]] + stretchOfVar[v])];
      }
    }
    // a structure is pinned to its level where it crosses; a street under a structure must not rise
    // above its ceiling there
    std::vector<std::vector<double>> pins(P.size()), ceiling(P.size());
    std::vector<std::vector<size_t>> pinStretch(P.size());  // stretch of the structure pinned at a sample
    auto place = [&](const std::vector<double> &x) {
      for (size_t mi = 0; mi < P.size(); ++mi)
        pins[mi].assign(P[mi].xy.size(), kNaN), ceiling[mi].assign(P[mi].xy.size(), kNaN), pinStretch[mi].assign(P[mi].xy.size(), 0);
      for (const Crossing &c : crossings) {
        const bool street = layerOf[c.v] == 0;
        double &v = street ? ceiling[c.road][c.i] : pins[c.road][c.i];
        if (std::isnan(v) || (street ? x[c.v] < v : std::abs(x[c.v]) > std::abs(v))) v = x[c.v];
        if (!street) pinStretch[c.road][c.i] = stretchOfVar[c.v];
      }
    };
    place(x0);
    // 2. junction levels: a ramp that reaches a road end sets its junction's level and the other roads
    //    of the junction start their ramps there (the largest level wins; repeated until stable). A
    //    road pinned near the junction bounds that level: it must reach its pin within its steepest
    //    grade (steepMean).
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> lo(clusters.size(), -inf), hi(clusters.size(), inf);
    for (size_t mi = 0; mi < P.size(); ++mi) {
      if (metas[mi].roadIndex < 0) continue;
      const Chain &ch = chains[metas[mi].chain];
      const std::vector<double> &zs = metas[mi].zs, &pin = pins[mi], &ceil = ceiling[mi];
      for (int e : {0, 1}) {
        const int c = e ? ch.c1 : ch.c0;
        if (c < 0) continue;
        for (size_t i = 0; i < pin.size(); ++i) {
          const double room = steepMean(mi) * std::abs(zs[i] - (e ? zs.back() : 0.0));
          if (!std::isnan(pin[i])) lo[c] = std::max(lo[c], pin[i] - room), hi[c] = std::min(hi[c], pin[i] + room);
          if (!std::isnan(ceil[i])) hi[c] = std::min(hi[c], ceil[i] + room);
        }
      }
    }
    // a road between two junctions bounds their difference the same way (propagated through the network)
    for (int it = 0; it < 200; ++it) {
      bool changed = false;
      for (size_t mi = 0; mi < P.size(); ++mi) {
        const Chain &ch = chains[metas[mi].chain];
        if (metas[mi].roadIndex < 0 || ch.c0 < 0 || ch.c1 < 0 || ch.c0 == ch.c1) continue;
        const double slack = steepMean(mi) * metas[mi].zs.back();
        for (const auto &[c, o] : {std::pair<int, int>{ch.c0, ch.c1}, {ch.c1, ch.c0}}) {
          const double l = std::max(lo[c], lo[o] - slack), h = std::min(hi[c], hi[o] + slack);
          if (l > h) continue;  // contradictory evidence: resolved in 3.
          if (l > lo[c] + 1e-6 || h < hi[c] - 1e-6) lo[c] = l, hi[c] = h, changed = true;
        }
      }
      if (!changed) break;
    }
    std::vector<double> level(clusters.size(), 0.0);
    // profiles with the ends at their junction levels (`all`: also at level 0, the final pass)
    auto profile = [&](size_t mi, bool all) {
      const Chain &ch = chains[metas[mi].chain];
      const std::vector<double> &zs = metas[mi].zs;
      std::vector<double> pin = pins[mi];
      // a structure stays up (down) between two of its crossings instead of ramping back to the ground
      for (size_t i = 0, last = 0, have = 0; i < pin.size(); ++i) {
        if (std::isnan(pins[mi][i])) continue;
        if (have && pinStretch[mi][last] == pinStretch[mi][i])
          for (size_t k = last + 1; k < i; ++k)
            pin[k] = pins[mi][last] + (pins[mi][i] - pins[mi][last]) * smoothstepC1((zs[k] - zs[last]) / (zs[i] - zs[last]));
        last = i, have = 1;
      }
      if (ch.c0 >= 0 && std::isnan(pin.front()) && (all || level[ch.c0] != 0.0)) pin.front() = level[ch.c0];
      if (ch.c1 >= 0 && std::isnan(pin.back()) && (all || level[ch.c1] != 0.0)) pin.back() = level[ch.c1];
      std::vector<double> z = levelProfile(zs, pin, rampGrade(mi));
      // a road that would rise into a structure above stays at its ceiling there: the deepest ceiling
      // first, as the ramps from it may already pass under the next one (a street under two bridges
      // a few metres apart is not pulled up between them)
      for (size_t k = 0; k < z.size(); ++k) {
        double deepest = kNaN;
        for (size_t i = 0; i < z.size(); ++i)
          if (!std::isnan(ceiling[mi][i]) && std::isnan(pin[i]) && z[i] > ceiling[mi][i] + 1e-6 && !(ceiling[mi][i] >= deepest))
            deepest = ceiling[mi][i];
        if (std::isnan(deepest)) break;
        for (size_t i = 0; i < z.size(); ++i)
          if (ceiling[mi][i] == deepest && std::isnan(pin[i]) && z[i] > deepest + 1e-6) pin[i] = deepest;
        z = levelProfile(zs, pin, rampGrade(mi));
      }
      return z;
    };
    for (int it = 0; it < 32; ++it) {
      std::vector<double> want(clusters.size(), 0.0);
      for (size_t mi = 0; mi < P.size(); ++mi) {
        if (metas[mi].roadIndex < 0) continue;
        const Chain &ch = chains[metas[mi].chain];
        const std::vector<double> z = profile(mi, false);
        for (const auto &[c, v] : {std::pair<int, double>{ch.c0, z.front()}, {ch.c1, z.back()}})
          if (c >= 0 && std::abs(v) > std::abs(want[c])) want[c] = v;
      }
      bool changed = false;
      for (size_t c = 0; c < clusters.size(); ++c) {
        const double v = lo[c] <= hi[c] ? std::clamp(want[c], lo[c], hi[c]) : 0.5 * (lo[c] + hi[c]);
        if (std::abs(v - level[c]) > 1e-3) level[c] = v, changed = true;
      }
      if (!changed) break;
    }
    // 3. the levels together. They are difference constraints x_a - x_b <= k: a storey between
    //    crossing roads; along a road the steepest ramp grade (steepMean) between its junctions and
    //    crossings; structures above the ground never below it, tunnels never above it, streets on it.
    //    The least levels these allow are found upward from the ground, then everything is lowered
    //    from the preferred levels (1., 2.) only as far as the constraints require (Bellman-Ford both
    //    ways): a ramp that cannot climb a storey lifts the junction at its foot rather than pulling
    //    the structure - and everything beneath it - down. OSM structures can lie closer than any
    //    such ramp (a street that passes under one bridge and, a few metres on, over another; a ramp
    //    mapped beside the road it joins as if above it): the constraints then form a negative cycle
    //    and no level satisfies them. The cycle is loosened as little as needed - its streets dip
    //    into a cutting at most a storey deep, then its storeys shrink (both reported) - and the
    //    levels are solved again.
    {
      const size_t nCrossing = x0.size();
      std::vector<size_t> levelVar(clusters.size());
      for (size_t c = 0; c < clusters.size(); ++c)
        levelVar[c] = x0.size(), x0.push_back(level[c]), layerOf.push_back(0), where.push_back(clusters[c].center);
      enum Kind { Bound, Grade, Cut, Storey };
      struct Diff {
        size_t a, b;
        double k;
        Kind kind;
        double cap = 0.0;  // the loosest k a contradiction may relax a Cut or a Storey to
      };
      std::vector<Diff> cons;
      std::vector<double> storeyAbove(x0.size(), 0.0);
      for (const auto &[key, h] : below) {
        cons.push_back({key.first, key.second, -h, Storey, 0.0});
        storeyAbove[key.first] = std::max(storeyAbove[key.first], h);
      }
      for (size_t v = 1; v < nCrossing; ++v) {
        if (layerOf[v] > 0) cons.push_back({0, v, 0.0, Bound, 0.0});
        if (layerOf[v] <= 0) cons.push_back({v, 0, 0.0, Bound, 0.0});
        if (layerOf[v] == 0) cons.push_back({0, v, 0.0, Cut, storeyAbove[v]});
      }
      // along a road: junction levels and pinned structures bound each other both ways, a ceiling
      // bounds its nearest two-sided neighbours from above
      struct Anchor {
        double s;
        size_t var;
        bool both;
      };
      std::vector<std::vector<Anchor>> anchors(P.size());
      for (const Crossing &c : crossings) anchors[c.road].push_back({metas[c.road].zs[c.i], c.v, layerOf[c.v] != 0});
      for (size_t mi = 0; mi < P.size(); ++mi) {
        if (metas[mi].roadIndex < 0) continue;
        const Chain &ch = chains[metas[mi].chain];
        auto &an = anchors[mi];
        if (ch.c0 >= 0) an.push_back({0.0, levelVar[ch.c0], true});
        if (ch.c1 >= 0) an.push_back({metas[mi].zs.back(), levelVar[ch.c1], true});
        std::sort(an.begin(), an.end(), [](const Anchor &p, const Anchor &q) { return p.s < q.s; });
        const double g = steepMean(mi);
        auto ramp = [&](size_t a, size_t b, double d) { cons.push_back({a, b, g * d, Grade}); };
        long last = -1;  // previous two-sided anchor
        for (size_t k = 0; k < an.size(); ++k) {
          if (!an[k].both) {
            if (last >= 0) ramp(an[last].var, an[k].var, an[k].s - an[last].s);
            continue;
          }
          if (last >= 0 && an[last].var != an[k].var) {
            ramp(an[k].var, an[last].var, an[k].s - an[last].s);
            ramp(an[last].var, an[k].var, an[k].s - an[last].s);
          }
          for (long j = static_cast<long>(k) - 1; j > last; --j)  // ceilings since the previous one
            ramp(an[k].var, an[j].var, an[k].s - an[j].s);
          last = static_cast<long>(k);
        }
      }
      // Bellman-Ford from x: `up` raises each x_b to at least x_a - k (the least levels above x),
      // otherwise each x_a is lowered to at most x_b + k (the greatest below x). Returns the
      // constraints of a negative cycle met on the way; none: x satisfies all of them
      auto relax = [&](std::vector<double> &x, bool up) {
        std::vector<long> pred(x.size(), -1);  // the constraint that last moved each level
        // the constraints along a cycle of the predecessor graph; empty if none
        auto predCycle = [&] {
          auto next = [&](size_t v) { return up ? cons[static_cast<size_t>(pred[v])].a : cons[static_cast<size_t>(pred[v])].b; };
          std::vector<size_t> seen(x.size(), 0), cycle;
          for (size_t v0 = 1; v0 <= x.size() && cycle.empty(); ++v0) {
            size_t v = v0 - 1;
            while (!seen[v] && pred[v] >= 0) seen[v] = v0, v = next(v);
            if (seen[v] != v0) continue;  // a level without predecessor, or one walked before
            for (size_t u = v; cycle.empty() || u != v; u = next(u)) cycle.push_back(static_cast<size_t>(pred[u]));
          }
          return cycle;
        };
        for (size_t pass = 0; pass <= x.size(); ++pass) {
          bool changed = false;
          for (size_t ci = 0; ci < cons.size(); ++ci) {
            const Diff &c = cons[ci];
            if (up && x[c.a] - c.k > x[c.b] + 1e-6) x[c.b] = x[c.a] - c.k, pred[c.b] = static_cast<long>(ci), changed = true;
            if (!up && x[c.a] > x[c.b] + c.k + 1e-6) x[c.a] = x[c.b] + c.k, pred[c.a] = static_cast<long>(ci), changed = true;
          }
          if (!changed) return std::vector<size_t>{};
          if (std::vector<size_t> cycle = predCycle(); !cycle.empty()) return cycle;
        }
        return predCycle();
      };
      // loosen a negative cycle by what it lacks, kind by kind; false if it has nothing to give
      auto loosen = [&](const std::vector<size_t> &cycle) {
        double need = 0.0;  // what the cycle lacks
        for (size_t ci : cycle) need -= cons[ci].k;
        for (const Kind kind : {Cut, Storey}) {
          double room = 0.0;
          for (size_t ci : cycle)
            if (cons[ci].kind == kind) room += cons[ci].cap - cons[ci].k;
          if (need <= 1e-9 || room <= 1e-9) continue;
          const double t = std::min(1.0, (need + 1e-3) / room);  // a little more: closed for good
          size_t most = cycle.front();
          double moved = 0.0;
          for (size_t ci : cycle)
            if (cons[ci].kind == kind && t * (cons[ci].cap - cons[ci].k) > moved) moved = t * (cons[ci].cap - cons[ci].k), most = ci;
          const Diff &w = cons[most];
          const Vec2 at = where[w.a == 0 ? w.b : w.a];
          char msg[200];
          if (kind == Cut)
            std::snprintf(msg, sizeof msg, "structures near (%.0f, %.0f) lie closer than a ramp can climb: street lowered %.1f m into a cutting",
                          at.x, at.y, w.k + moved);
          else
            std::snprintf(msg, sizeof msg, "structures near (%.0f, %.0f) lie closer than a ramp can climb: storey reduced from %.1f m to %.1f m",
                          at.x, at.y, -w.k, -(w.k + moved));
          rep.warnings.push_back(msg);
          for (size_t ci : cycle)
            if (cons[ci].kind == kind) cons[ci].k += t * (cons[ci].cap - cons[ci].k);
          need -= t * room;
        }
        return need <= 1e-9;
      };
      std::vector<double> x;
      bool solved = false;
      for (size_t round = 0; round <= cons.size() && !solved; ++round) {
        std::vector<double> least(x0.size(), -inf);
        least[0] = 0.0;
        std::vector<size_t> cycle = relax(least, true);
        if (cycle.empty()) {
          x = x0;
          for (size_t v = 0; v < x.size(); ++v) x[v] = std::max(x[v], least[v]);
          cycle = relax(x, false);
        }
        if (cycle.empty()) solved = true;
        else if (!loosen(cycle)) break;
      }
      if (!solved) {  // contradictions nothing can loosen: keep the preferred levels
        x = x0;
        rep.warnings.push_back("structure levels left unresolved: some structures may lie closer than a storey");
      }
      for (size_t c = 0; c < clusters.size(); ++c) level[c] = x[levelVar[c]];
      place(x);
    }
    if (opt.terrain) {
      for (size_t mi = 0; mi < P.size(); ++mi)
        if (metas[mi].roadIndex >= 0) rel[mi] = profile(mi, true);
    } else {
      // the constructed profile is final (no measurement to re-fit against the junction surfaces)
      for (size_t mi = 0; mi < P.size(); ++mi) {
        RoadMeta &m = metas[mi];
        if (m.roadIndex < 0) continue;
        m.zz = profile(mi, true);
        m.hasData = true;
        m.xs.assign(m.zs.size(), 0.0);
        m.obs0 = m.obs1 = 0.0;
      }
      return;
    }
  }
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
              // a street on its measured ground under a structure that has a measured surface clearing it
              // stays there: the structure keeps to that surface (its levels within the clearance of the
              // street are banned). Ground that is the deck itself (lidar ground through an underpass:
              // no surface of the structure clears it) gives way as before.
              bool onGround = a.layer[i] == 0 && b.layer[j] > 0;
              if (onGround) {
                bool measured = false, clears = false;
                for (const Cand &c : a.cand[i]) measured |= c.cost == 0.0 && std::abs(c.z - a.chosen[i]) < 1e-9;
                for (const Cand &c : b.cand[j]) clears |= c.z >= a.chosen[i] + minClearance;
                onGround = measured && clears;
                for (const auto &[lo, hi] : a.excl[i]) onGround &= !(a.chosen[i] > lo && a.chosen[i] < hi);
              }
              if (onGround) {
                P[mj].excl[j].push_back({a.chosen[i] - minClearance, a.chosen[i] + minClearance});
                dirty.insert(mj);
                continue;
              }
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
    const std::vector<double> w0 = w;  // the measured samples
    double wsum = 0;
    for (double v : w) wsum += v;
    // a road wholly covered at ground level over measured terrain (covered=yes, OSM layer 0: a driveway through a
    // building): a weak first guess, straight between the ground at its two ends; the junction surfaces decide the
    // rest. Not on a bridge or in a tunnel: the ground at their ends is not their road (London: Waterloo Bridge over
    // the Embankment, tunnel stubs under the street)
    bool layered = false;
    for (size_t i = 0; i < n && i < P[mi].layer.size(); ++i) layered |= P[mi].layer[i] != 0;
    if (wsum < 0.5 && opt.terrain && !layered) {
      size_t f = 0, l = n;
      while (f < n && std::isnan(P[mi].groundC[f])) ++f;
      while (l > f && std::isnan(P[mi].groundC[l - 1])) --l;
      if (f < l) {
        y[f] = P[mi].groundC[f], y[l - 1] = P[mi].groundC[l - 1], w[f] = w[l - 1] = 1e-3;
        wsum = 2e-3;
      }
    }
    m.zRel = rel[mi], m.zGround = P[mi].groundC;  // kept for networks nothing measured reaches (junctionSurfaces)
    if (wsum <= 0.0) {
      m.zz.assign(n, 0.0);
      m.hasData = false;
      continue;
    }
    const double ds = m.zs.back() / std::max<size_t>(1, n - 1);
    double lambda = opt.elevationSmoothing / std::pow(std::max(ds, 0.1), 4.0);
    // robust fit: Tukey biweights drop isolated outliers (walls, stairs, kerb edges, vehicles that
    // survived lidar classification) instead of letting them bend the profile
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
    // what the terrain cannot measure (a bridge without deck data, a tunnel) lies at the relative level (1. - 3.) above /
    // below the measured-ground curve - relative to the measured samples around it, which already carry their part of
    // it (an embankment, a cutting): in each unmeasured stretch the offset is the relative level less its value
    // interpolated between the measured samples bracketing the stretch, zero where the measurement takes over.
    // Interpolated between its portals, the Altstadtringtunnel (Munich) ran 2 m under the street above it
    if (opt.terrain && rel[mi].size() == n) {
      bool any = false;
      for (size_t i = 0; i < n;) {
        if (w0[i] > 0.0) {
          ++i;
          continue;
        }
        size_t e = i;
        while (e < n && w0[e] == 0.0) ++e;  // unmeasured stretch [i, e)
        const bool before = i > 0, after = e < n;
        for (size_t k = i; k < e; ++k) {
          double anchor = 0.0;
          if (before && after)
            anchor = rel[mi][i - 1] + (rel[mi][e] - rel[mi][i - 1]) * (m.zs[k] - m.zs[i - 1]) / std::max(1e-9, m.zs[e] - m.zs[i - 1]);
          else if (before)  // open to the road end: the offset fades to the end, where the junction surface decides
            anchor = rel[mi][i - 1] + (rel[mi][n - 1] - rel[mi][i - 1]) * (m.zs[k] - m.zs[i - 1]) / std::max(1e-9, m.zs[n - 1] - m.zs[i - 1]);
          else if (after)
            anchor = rel[mi][0] + (rel[mi][e] - rel[mi][0]) * m.zs[k] / std::max(1e-9, m.zs[e]);
          const double off = rel[mi][k] - anchor;
          if (std::abs(off) > 1e-3) y[k] = m.zz[k] + off, w[k] = 1.0, any = true;
        }
        i = e;
      }
      if (any) m.zz = whittaker(y, w, lambda);
    }
    // before the first and after the last measured sample (a tunnel or a covered way running to the
    // road end, an unmeasured deck, the edge of the terrain data) the smoothest curve carries the
    // slope at its very end - set by the last one or two samples - over the whole stretch (LA: a
    // 0.4 m outlier lifted an 80 m tunnel by 6 m). The grade measured over the last 20 m continues
    // instead; the junction surfaces decide the rest.
    extendUnmeasuredEnds(m.zz, m.zs, y, w, 20.0, opt.maxGrade);
    // cross slope (OpenDRIVE 1.8.0 §10.5.2 superelevation) where the road lies on the measured ground; smooth and
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
  // roads without any measurement (outside the terrain data, fully in tunnels, underground garages)
  // take their heights from the junction surfaces at their ends (linear in between), and junctions
  // without a measured road take a surface from the heights already known around them - repeated
  // outward from the measured network until nothing changes. Only real or propagated heights are
  // used, never placeholders.
  std::vector<char> known(metas.size(), 0);
  for (size_t i = 0; i < metas.size(); ++i) known[i] = metas[i].roadIndex >= 0 && metas[i].hasData;
  auto propagate = [&]() {
    for (bool changed = true; changed;) {
      changed = false;
      for (size_t i = 0; i < metas.size(); ++i) {
        RoadMeta &m = metas[i];
        if (m.roadIndex < 0 || known[i]) continue;
        const odr::Road &r = od.roads[m.roadIndex];
        const Chain &ch = chains[m.chain];
        const bool h0 = ch.c0 >= 0 && clusters[ch.c0].hasPlane, h1 = ch.c1 >= 0 && clusters[ch.c1].hasPlane;
        if (!h0 && !h1) continue;
        const double z0 = h0 ? planeZ(clusters[ch.c0], r.planView.eval(0).pos()) : 0.0;
        const double z1 = h1 ? planeZ(clusters[ch.c1], r.planView.eval(r.length()).pos()) : 0.0;
        const double a = h0 ? z0 : z1, b = h1 ? z1 : a;
        for (size_t k = 0; k < m.zs.size(); ++k) m.zz[k] = a + (b - a) * m.zs[k] / std::max(1e-9, r.length());
        known[i] = 1, changed = true;
      }
      for (Cluster &cl : clusters) {
        if (cl.hasPlane || cl.ends.empty()) continue;
        std::vector<Pt> pts;
        for (const RoadEnd &e : cl.ends) {
          const RoadMeta &m = metas[e.road];
          if (m.roadIndex < 0 || !known[e.road] || m.zz.empty()) continue;
          const odr::Road &r = od.roads[m.roadIndex];
          const Vec2 p = r.planView.eval(e.atEnd ? r.length() : 0.0).pos();
          // a height carried through an unmeasured road counts less the longer that road is: a
          // junction follows its nearest known neighbour (a garage entrance stub, not the far exits)
          const double w = m.hasData ? 1.0 : 1.0 / std::pow(1.0 + r.length() / 10.0, 2.0);
          pts.push_back({p.x, p.y, e.atEnd ? m.zz.back() : m.zz.front(), w});
        }
        if (pts.empty()) continue;
        double sol[3];
        fit(pts, cl.center, sol, 0.05 * weightSum(pts) * 100.0);
        const double g = std::hypot(sol[1], sol[2]);
        if (g > 0.15) sol[1] *= 0.15 / g, sol[2] *= 0.15 / g;
        if (!opt.target.superelevation) sol[1] = sol[2] = 0.0;
        cl.plane[0] = sol[0], cl.plane[1] = sol[1], cl.plane[2] = sol[2];
        cl.hasPlane = changed = true;
      }
    }
  };
  propagate();
  // a network nothing measured reaches (an underground garage whose ramps OSM does not connect, Frankfurter Kreuz:
  // three levels, 118 roads) keeps its relative levels (1. - 3.: a storey per level under level 0) under the ground:
  // OSM layers count from the ground (wiki Key:layer), and a garage lies under everything above it, so level 0 is the
  // lowest measured surface over its roads (Frankfurt: a street in a dip runs over it, 5 m under the median ground).
  // Then its junctions take surfaces from those heights as above
  std::vector<size_t> comp(metas.size());
  for (size_t i = 0; i < comp.size(); ++i) comp[i] = i;
  std::function<size_t(size_t)> root = [&](size_t i) { return comp[i] == i ? i : comp[i] = root(comp[i]); };
  std::map<int, size_t> byCluster;  // cluster -> first unknown road at it
  for (size_t i = 0; i < metas.size(); ++i) {
    if (metas[i].roadIndex < 0 || known[i]) continue;
    const Chain &ch = chains[metas[i].chain];
    for (int c : {ch.c0, ch.c1})
      if (c >= 0) {
        auto [it, fresh] = byCluster.emplace(c, i);
        if (!fresh) comp[root(i)] = root(it->second);
      }
  }
  // what lies over it: the measured ground along its roads and the measured roads passing over them in plan (centre
  // lines within 4 m; a street in a cutting runs below the ground beside it)
  const double over = 4.0;
  struct Height {
    double x, y, z;
  };
  std::map<std::pair<long, long>, std::vector<Height>> knownAt;  // centre-line samples of roads with heights, 10 m cells
  auto cellOf = [](const Vec2 &p) { return std::make_pair(static_cast<long>(std::floor(p.x / 10.0)), static_cast<long>(std::floor(p.y / 10.0))); };
  for (size_t i = 0; i < metas.size(); ++i)
    if (metas[i].roadIndex >= 0 && known[i])
      for (size_t k = 0; k < metas[i].zs.size(); ++k) {
        const Vec2 p = od.roads[metas[i].roadIndex].planView.eval(metas[i].zs[k]).pos();
        knownAt[cellOf(p)].push_back({p.x, p.y, metas[i].zz[k]});
      }
  std::map<size_t, std::vector<double>> groundOf;
  for (size_t i = 0; i < metas.size(); ++i) {
    if (metas[i].roadIndex < 0 || known[i]) continue;
    for (double g : metas[i].zGround)
      if (!std::isnan(g)) groundOf[root(i)].push_back(g);
    for (size_t k = 0; k < metas[i].zs.size(); ++k) {
      const Vec2 p = od.roads[metas[i].roadIndex].planView.eval(metas[i].zs[k]).pos();
      const auto [cx, cy] = cellOf(p);
      for (long dx = -1; dx <= 1; ++dx)
        for (long dy = -1; dy <= 1; ++dy)
          if (auto it = knownAt.find({cx + dx, cy + dy}); it != knownAt.end())
            for (const Height &q : it->second)
              if (std::hypot(q.x - p.x, q.y - p.y) < over) groundOf[root(i)].push_back(q.z);
    }
  }
  std::map<size_t, double> level0;  // component -> lowest measured surface over it
  for (const auto &[c, v] : groundOf) level0[c] = *std::min_element(v.begin(), v.end());
  bool anchored = false;
  for (size_t i = 0; i < metas.size(); ++i) {
    RoadMeta &m = metas[i];
    const auto l0 = level0.find(root(i));
    if (m.roadIndex < 0 || known[i] || l0 == level0.end() || m.zRel.size() != m.zs.size()) continue;
    for (size_t k = 0; k < m.zs.size(); ++k) m.zz[k] = l0->second + m.zRel[k];
    known[i] = 1, anchored = true;
  }
  if (anchored) propagate();
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
    // banking toward the junction planes (OpenDRIVE 1.8.0 §10.5.2 superelevation), flat in between
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
      // at a junction plane the roll rate is the plane's along this road, d/ds atan(grad . left(s)) =
      // -kappa (grad . dir) / (1 + g^2) - as on the connecting roads (connectingElevation): the lane edges'
      // pitch then matches across the link (OpenDRIVE 1.9 Annex D.1.5: no vertical kinks)
      if (opt.target.superelevation && opt.junctionSurfaces)
        for (bool atEnd : {false, true}) {
          const int ci = atEnd ? ch.c1 : ch.c0;
          if (ci < 0 || !clusters[ci].hasPlane) continue;
          const Vec2 grad(clusters[ci].plane[1], clusters[ci].plane[2]);
          const Pose p = r.planView.eval(atEnd ? L : 0.0);
          const double g = grad.dot(p.left());
          dse[atEnd ? n - 1 : 0] = -p.kappa * grad.dot(p.dir()) / (1.0 + g * g);
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
  // records every 2 m, closer on a tight curve: the banking atan(grad . left) turns with the heading, and a cubic
  // record spanning more than 15 deg of it overshoots the plane's cross slope (Frankfurt: a 4.5 m boundary helper
  // round a 2 m radius banked 0.156 rad on a plane capped at atan(15 %) = 0.149)
  double kmax = 0.0;
  for (double q = 0.0; q <= L; q += std::min(0.25, std::max(L, 1e-3) / 8.0)) kmax = std::max(kmax, std::abs(c.planView.eval(q).kappa));
  const double step = std::min(2.0, 0.26 / std::max(kmax, 1e-9));
  const int n = std::max(2, static_cast<int>(std::ceil(L / step)) + 1);
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

void Builder::coveredUnderpasses() {
  // A covered way (OSM covered=yes) runs under a building, a deck or a hillside. Where its road lies below the
  // measured terrain it is an underpass: OpenDRIVE 1.8.0 §13.11 tunnel, e_tunnelType "underpass" (XSD 1.4 - 1.8.1),
  // "the part of the road that is the tunnel or the underpass". Written as one, the ground above it is not the road
  // surface (conform.cpp leaves tunnels alone) and viewers draw it covered. A covered way at ground level (a canopy,
  // a passage through a building at grade) stays a plain road: the ground there is its surface. The OSM layer cannot
  // tell the two apart (Frankfurt, Hugo-Eckener-Ring under the terminal forecourt: covered=yes, layer 0, 7 m deep),
  // so the measured depth decides, sample by sample: the underpass is the part of the covered stretch whose road lies
  // under the terrain (Frankfurt road 431: its last 12 m reach the ground at the portal and stay a plain road).
  // Before this, the conformed ground trenched such stretches down to the road.
  // Measured depths of all covered stretches (Munich, London, Frankfurt, LA; docs/TERRAIN_CONTRACT.md "buried"): at
  // grade never more than 0.81 m at any sample, under a deck or hill a median of 5.54 m or more - any limit in between
  // makes the same decisions.
  if (!opt.terrain) return;
  constexpr double kUnderpassDepth = 2.0;  // m: road surface this far under the measured terrain is under cover
  constexpr double kJoinGap = 10.0;        // m: deep runs closer than this are one underpass (a short dip in the DEM)
  constexpr double kMinLength = 5.0;       // m: shorter deep runs are DEM noise, not a structure
  // m: a run's mean depth at least this - a few noisy samples just past the limit on an at-grade way are not a structure
  // (real runs: mean 5.38 .. 7.10 m; simulated 1 m DEM noise on the 30 at-grade stretches: false underpasses in 2 of
  // 6000 trials with it, 149 without; build/tmp/underpass_variants.py)
  constexpr double kRunMeanDepth = 3.0;
  // m: a tunnel record grows at most this far past each end of its OSM way - ways end "a few metres short" (largest
  // measured: London's Strand Underpass, 11.2 m); a longer deep stretch beyond is a cutting the DEM does not resolve
  constexpr double kMaxTunnelExtension = 20.0;
  auto within = [](const std::vector<std::pair<double, double>> &rg, double a, double b) {
    for (const auto &[p, q] : rg)
      if (a >= p - 1e-6 && b <= q + 1e-6) return true;
    return false;
  };
  for (RoadMeta &m : metas) {
    if (m.roadIndex < 0 || m.zGround.size() != m.zs.size()) continue;
    odr::Road &r = od.roads[m.roadIndex];
    if (m.covered.empty() && r.tunnels.empty()) continue;
    const double len = r.length();
    const double half = 0.5 * len / std::max<size_t>(1, m.zs.size() - 1);  // half the sample spacing
    auto deepAt = [&](size_t i) {
      return !std::isnan(m.zGround[i]) && m.zGround[i] - odr::elevationAt(r, m.zs[i]) >= kUnderpassDepth;
    };
    // tunnel ends: an OSM tunnel way often ends short of the portal (see "bridge / tunnel ways often end a few metres
    // short" in sampleElevation's crossings): where the road keeps running kUnderpassDepth or more under the measured
    // terrain beyond a tunnel record, the record is extended over those samples, up to the next tunnel (OpenDRIVE 1.8.0
    // §13.11: "the part of the road that is the tunnel"). London's Strand Underpass ran 3.8-4.3 m under the street for
    // 12 m past its tunnel way, so the conformed ground trenched down to it there
    std::vector<size_t> order(r.tunnels.size());  // in s order (records of a reversed chain come in reverse); the
    for (size_t k = 0; k < order.size(); ++k) order[k] = k;  // written order stays as it is
    std::sort(order.begin(), order.end(), [&](size_t x, size_t y) { return r.tunnels[x].s < r.tunnels[y].s; });
    for (size_t k = 0; k < order.size(); ++k) {
      odr::Structure &st = r.tunnels[order[k]];
      const double a0 = st.s, b0 = st.s + st.length;
      double a = a0, b = b0;
      for (size_t i = 0; i < m.zs.size(); ++i)
        if (m.zs[i] > b + 1e-6) {
          if (m.zs[i] - b > 2.0 * half + 1e-6 || !deepAt(i)) break;
          b = std::min({len, m.zs[i] + half, b0 + kMaxTunnelExtension});
        }
      for (size_t i = m.zs.size(); i-- > 0;)
        if (m.zs[i] < a - 1e-6) {
          if (a - m.zs[i] > 2.0 * half + 1e-6 || !deepAt(i)) break;
          a = std::max({0.0, m.zs[i] - half, a0 - kMaxTunnelExtension});
        }
      if (k + 1 < order.size()) b = std::min(b, r.tunnels[order[k + 1]].s);
      if (k > 0) a = std::max(a, r.tunnels[order[k - 1]].s + r.tunnels[order[k - 1]].length);
      st.s = a, st.length = b - a;
    }
    if (m.covered.empty()) continue;
    // the terrain noise under this road: robust spread (1.4826 MAD) of the raw ground at its carriageway centre around
    // its profile, outside covered / tunnel / bridge stretches (the road's own observations are outlier-filtered and
    // hide the noise). A run must lie deeper on average than kUnderpassDepth + 3 sigma, so a noisy model does not invent
    // underpasses. Measured: sigma 0.003 .. 0.06 m on the covered roads of Munich, London and Frankfurt, 1.38 m on LA
    // 987's hillside (floor 6.1 m, its run 7.1 m deep); simulated per-sample noise of 1 / 1.5 m: no false underpass with
    // this floor, 2 / 210 of 7200 with the plain 3 m floor (build/tmp/underpass_sigma_guard.py). A run it drops keeps the
    // plain-road behaviour (the conformed ground follows the road), never worse than without this stage
    auto inRange = [&](const std::vector<std::pair<double, double>> &rg, double s) {
      for (const auto &[a, b] : rg)
        if (s + m.trimStart >= a - 1e-6 && s + m.trimStart <= b + 1e-6) return true;
      return false;
    };
    std::vector<double> off;
    for (size_t i = 0; i < m.zs.size(); ++i)
      if (!std::isnan(m.zGround[i]) && !inRange(m.covered, m.zs[i]) && !inRange(m.tunnels, m.zs[i]) &&
          !inRange(m.bridges, m.zs[i]))
        off.push_back(m.zGround[i] - odr::elevationAt(r, m.zs[i]));
    double sigma = 0.0;
    if (off.size() >= 5) {
      std::nth_element(off.begin(), off.begin() + off.size() / 2, off.end());
      const double med = off[off.size() / 2];
      for (double &v : off) v = std::abs(v - med);
      std::nth_element(off.begin(), off.begin() + off.size() / 2, off.end());
      sigma = 1.4826 * off[off.size() / 2];
    }
    const double runMeanFloor = std::max(kRunMeanDepth, kUnderpassDepth + 3.0 * sigma);
    // covered segments that are not already a tunnel or a bridge (a covered bridge), adjacent ones merged
    std::vector<std::pair<double, double>> runs;
    for (const auto &[a, b] : m.covered) {
      if (within(m.tunnels, a, b) || within(m.bridges, a, b)) continue;
      const double sa = std::max(0.0, a - m.trimStart), sb = std::min(len, b - m.trimStart);
      if (sb <= sa) continue;
      if (!runs.empty() && sa <= runs.back().second + 1e-6) runs.back().second = std::max(runs.back().second, sb);
      else runs.push_back({sa, sb});
    }
    bool added = false;
    for (const auto &[sa, sb] : runs) {
      // deep runs: consecutive samples at least kUnderpassDepth under the terrain, each reaching half a spacing past
      // its outer samples (clipped to the covered stretch); runs closer than kJoinGap joined
      struct Run {
        double a, b, depthSum;
        int n;
      };
      std::vector<Run> deep;
      for (size_t i = 0; i < m.zs.size(); ++i) {
        const double s = m.zs[i];
        const double d = std::isnan(m.zGround[i]) ? kNaN : m.zGround[i] - odr::elevationAt(r, s);
        if (s < sa || s > sb || !(d >= kUnderpassDepth)) continue;
        const double a = std::max(sa, s - half), b = std::min(sb, s + half);
        if (!deep.empty() && a - deep.back().b < kJoinGap) deep.back().b = b, deep.back().depthSum += d, ++deep.back().n;
        else deep.push_back({a, b, d, 1});
      }
      for (const auto &[a, b, depthSum, n] : deep) {
        if (b - a < kMinLength || depthSum / n < runMeanFloor) continue;
        odr::Structure st;
        st.id = std::to_string(nextObjectId++);
        st.name = "underpass";
        st.type = "underpass";
        st.s = a;
        st.length = b - a;
        r.tunnels.push_back(st);
        added = true;
      }
    }
    if (added)
      std::sort(r.tunnels.begin(), r.tunnels.end(), [](const odr::Structure &x, const odr::Structure &y) { return x.s < y.s; });
  }
}

void Builder::junctionsUnderCover() {
  // A junction inside a tunnel or an underpass: Frankfurt's underground car park P15, whose ramps and aisles are mapped
  // tunnel=yes, meets in junctions 7-12 m under the ground. Its connecting roads carry no OSM tag of their own, so they
  // got no tunnel record, and the conformed ground (conform.cpp leaves only bridges and tunnels out) dug a pit down to
  // them at every garage junction. A connecting road whose both ends lie inside tunnels of the roads it links is under
  // the same cover: it gets a tunnel record over its whole length. One with a single end in a tunnel (a junction at a
  // portal, Munich road 2373) gets one from that end over the stretch 2 m or more under the measured terrain, at most
  // 20 m - the coveredUnderpasses depth limit and tunnel-end cap. OpenDRIVE 1.8.0 §13.11; objects, tunnels among them,
  // are allowed on every road, connecting roads included (XSD 1.8.1 t_road / t_road_objects)
  constexpr double kDeep = 2.0, kMaxPartial = 20.0, kStep = 1.0;  // m
  std::map<std::string, size_t> index;
  for (size_t i = 0; i < od.roads.size(); ++i) index[od.roads[i].id] = i;
  std::map<int, const RoadMeta *> metaOf;
  for (const RoadMeta &m : metas)
    if (m.roadIndex >= 0) metaOf[m.roadIndex] = &m;
  auto coveredEnd = [&](const std::optional<odr::LinkTarget> &l) {
    if (!l || l->elementType != "road") return false;
    const auto it = index.find(l->elementId);
    if (it == index.end()) return false;
    const odr::Road &arm = od.roads[it->second];
    const double s = l->contactPoint == "end" ? arm.length() : 0.0;
    for (const odr::Structure &t : arm.tunnels)
      if (s >= t.s - 0.5 && s <= t.s + t.length + 0.5) return true;
    // a stub's tunnel shorter than 1 m gets no record (buildRoads), its OSM range still covers it (Frankfurt road 244)
    const auto mi = metaOf.find(static_cast<int>(it->second));
    if (mi != metaOf.end()) {
      const double sf = mi->second->trimStart + s;
      for (const auto &[a, b] : mi->second->tunnels)
        if (sf >= a - 0.5 && sf <= b + 0.5) return true;
    }
    return false;
  };
  auto deep = [&](const odr::Road &c, double s) {
    const Vec2 p = c.planView.eval(s).pos();
    const double g = opt.terrain->ground_at(p.x, p.y);
    return !std::isnan(g) && g - odr::elevationAt(c, s) >= kDeep;
  };
  struct Cover {
    odr::Road *road;
    double s0, s1;
  };
  std::vector<Cover> covers;  // decided first, written after: a new record must not decide a neighbour
  for (odr::Road &c : od.roads) {
    if (c.junction == "-1" || !c.tunnels.empty() || c.length() <= 0.0) continue;
    const bool atStart = coveredEnd(c.predecessor), atEnd = coveredEnd(c.successor);
    const double L = c.length();
    if (atStart && atEnd) {
      covers.push_back({&c, 0.0, L});
      continue;
    }
    if (!(atStart || atEnd) || !opt.terrain || opt.terrain->ground.empty()) continue;
    double reach = 0.0;
    for (double d = 0.0; d <= std::min(L, kMaxPartial) + 1e-9; d += kStep) {
      if (!deep(c, atStart ? d : L - d)) break;
      reach = std::min({L, d + 0.5 * kStep, kMaxPartial});
    }
    if (reach >= 0.5) covers.push_back(atStart ? Cover{&c, 0.0, reach} : Cover{&c, L - reach, L});
  }
  for (const Cover &cv : covers) {
    odr::Structure st;
    st.id = std::to_string(nextObjectId++);
    st.name = "tunnel";
    st.type = "standard";
    st.s = cv.s0;
    st.length = cv.s1 - cv.s0;
    cv.road->tunnels.push_back(st);
  }
  rep.junctionRoadsUnderCover = static_cast<int>(covers.size());
}

}  // namespace detail
}  // namespace odrforge
