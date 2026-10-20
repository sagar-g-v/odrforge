#include "odrforge/clothoid_fit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace odrforge {

namespace {

// Integrals of t^k * {cos, sin}(a/2 t^2 + b t + c) over [0, 1] for k = 0, 1, 2.
void fresnelMoments(double a, double b, double c, double X[3], double Y[3]) {
  static const double gx[8] = {0.0950125098376374, 0.2816035507792589, 0.4580167776572274, 0.6178762444026438,
                               0.7554044083550030, 0.8656312023878318, 0.9445750230732326, 0.9894009349916499};
  static const double gw[8] = {0.1894506104550685, 0.1826034150449236, 0.1691565193950025, 0.1495959888165767,
                               0.1246289712555339, 0.0951585116824928, 0.0622535239386479, 0.0271524594117541};
  const int n = std::max(1, static_cast<int>(std::ceil(std::abs(a) * 0.5 + std::abs(b) + 1.0)));
  const double h = 1.0 / n;
  for (int k = 0; k < 3; ++k) X[k] = Y[k] = 0.0;
  for (int i = 0; i < n; ++i) {
    const double mid = (i + 0.5) * h, half = 0.5 * h;
    for (int j = 0; j < 8; ++j)
      for (int sg = -1; sg <= 1; sg += 2) {
        const double t = mid + sg * half * gx[j], w = gw[j] * half;
        const double ph = 0.5 * a * t * t + b * t + c, cs = std::cos(ph), sn = std::sin(ph);
        X[0] += w * cs;          Y[0] += w * sn;
        X[1] += w * t * cs;      Y[1] += w * t * sn;
        X[2] += w * t * t * cs;  Y[2] += w * t * t * sn;
      }
  }
}

double distPointSeg(const Vec2 &p, const Vec2 &a, const Vec2 &b) {
  const Vec2 ab = b - a;
  const double L2 = ab.dot(ab);
  if (L2 < 1e-18) return (p - a).norm();
  const double t = std::clamp((p - a).dot(ab) / L2, 0.0, 1.0);
  return (p - (a + ab * t)).norm();
}

void dpRecurse(const std::vector<Vec2> &p, size_t i0, size_t i1, double tol, std::vector<bool> &keep) {
  if (i1 <= i0 + 1) return;
  double best = -1.0;
  size_t bi = i0;
  for (size_t i = i0 + 1; i < i1; ++i) {
    const double d = distPointSeg(p[i], p[i0], p[i1]);
    if (d > best) best = d, bi = i;
  }
  if (best > tol) {
    keep[bi] = true;
    dpRecurse(p, i0, bi, tol, keep);
    dpRecurse(p, bi, i1, tol, keep);
  }
}

// Symmetric clothoid pair (spiral 0 -> k, spiral k -> 0) turning by `turn` (rad, > 0) with spiral
// length 1: tangent length d1 (start point to the intersection of the two legs) and deviation dev1
// (intersection to the pair's midpoint). The shape scales linearly with the spiral length.
struct PairShape {
  double d1 = 0, dev1 = 0;
};
PairShape clothoidPairShape(double turn) {
  // first spiral: heading(t) = (turn / 2) t^2, t in [0, 1] (Simpson)
  const int N = 64;
  double x = 0, y = 0;
  for (int i = 0; i <= N; ++i) {
    const double t = static_cast<double>(i) / N, h = 0.5 * turn * t * t;
    const double w = (i == 0 || i == N) ? 1.0 : (i % 2 ? 4.0 : 2.0);
    x += w * std::cos(h), y += w * std::sin(h);
  }
  x /= 3.0 * N, y /= 3.0 * N;  // midpoint M of the pair, heading turn / 2 there
  // the pair is symmetric about the corner bisector, which passes through M along the normal there;
  // it meets leg a (the x axis) at the corner C = (x + y tan(turn / 2), 0)
  const double hm = 0.5 * turn;
  PairShape ps;
  ps.d1 = x + y * std::tan(hm);
  ps.dev1 = std::hypot(ps.d1 - x, y);
  return ps;
}

// Replace sharp corners by two tangent points; the corner between them becomes a symmetric
// clothoid pair (G2: curvature 0 on the legs, rising to a peak at the bisector and back).
std::vector<Vec2> filletCorners(const std::vector<Vec2> &p, const PolylineFitOptions &o,
                                std::vector<double> *legHeading = nullptr, std::vector<char> *pairStart = nullptr) {
  // legHeading (same size as the result): the heading of the leg a fillet tangent point lies on,
  // NaN for other points - a tangent point's heading is its leg's by construction; pairStart marks
  // the first tangent point of each corner (the segment from it to the next point is the pair)
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (legHeading) legHeading->assign(p.size() < 3 ? p.size() : 1, nan);
  if (pairStart) pairStart->assign(p.size() < 3 ? p.size() : 1, 0);
  if (p.size() < 3) return p;
  std::vector<Vec2> out{p.front()};
  for (size_t i = 1; i + 1 < p.size(); ++i) {
    const Vec2 a = p[i] - p[i - 1], b = p[i + 1] - p[i];
    const double la = a.norm(), lb = b.norm();
    const double turn = std::abs(normalizeAngle(std::atan2(b.y, b.x) - std::atan2(a.y, a.x)));
    if (turn < o.cornerAngle || la < 1e-6 || lb < 1e-6 || turn > 3.0) {
      out.push_back(p[i]);
      if (legHeading) legHeading->push_back(nan);
      if (pairStart) pairStart->push_back(0);
      continue;
    }
    // spiral length: the class radius as peak curvature (Ls = turn R), shortened so the pair stays
    // within maxCornerDeviation of the mapped corner, but never so short that its peak curvature
    // (turn / Ls) exceeds the lane-fold limit
    const PairShape ps = clothoidPairShape(turn);
    double Ls = turn * o.minCornerRadius;
    if (o.maxCornerDeviation > 0 && ps.dev1 > 1e-9) Ls = std::min(Ls, o.maxCornerDeviation / ps.dev1);
    if (o.maxCurvature > 0) Ls = std::max(Ls, turn / o.maxCurvature);
    // tangent length; limited so neighbouring fillets do not overlap
    double d = Ls * ps.d1;
    d = std::min({d, 0.45 * la, 0.45 * lb});
    out.push_back(p[i] - a * (d / la));
    out.push_back(p[i] + b * (d / lb));
    if (legHeading) legHeading->push_back(std::atan2(a.y, a.x)), legHeading->push_back(std::atan2(b.y, b.x));
    if (pairStart) pairStart->push_back(1), pairStart->push_back(0);
  }
  out.push_back(p.back());
  if (legHeading) legHeading->push_back(nan);
  if (pairStart) pairStart->push_back(0);
  return out;
}

/// the symmetric clothoid pair from pose (p0, h0) to the tangent point p1 with heading h1
std::optional<std::vector<GeomElement>> clothoidPair(const Vec2 &p0, double h0, const Vec2 &p1, double h1) {
  const double turn = normalizeAngle(h1 - h0);
  if (std::abs(turn) < 1e-6) return std::nullopt;
  const PairShape ps = clothoidPairShape(std::abs(turn));
  // the tangent length of the pair equals the distance from each tangent point to the corner:
  // solve it from the two leg lines
  const Vec2 da(std::cos(h0), std::sin(h0)), db(std::cos(h1), std::sin(h1));
  const double den = da.cross(db);
  if (std::abs(den) < 1e-9) return std::nullopt;
  const double ta = (p1 - p0).cross(db) / den;  // corner = p0 + da ta
  if (ta <= 0) return std::nullopt;
  const double Ls = ta / ps.d1, k = turn / Ls;
  GeomElement e1;
  e1.x = p0.x, e1.y = p0.y, e1.hdg = h0, e1.length = Ls, e1.k0 = 0.0, e1.k1 = k;
  const Pose m = e1.end();
  GeomElement e2;
  e2.x = m.x, e2.y = m.y, e2.hdg = m.hdg, e2.length = Ls, e2.k0 = k, e2.k1 = 0.0;
  if ((e2.end().pos() - p1).norm() > 0.05 + 0.01 * ta) return std::nullopt;  // asymmetric legs: not a pair
  return std::vector<GeomElement>{e1, e2};
}

// Heading at interior vertex i: tangent of the circle through p[i-1], p[i], p[i+1] (exact for arcs).
double circleTangent(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
  const Vec2 u = b - a, v = c - b;
  const double hu = std::atan2(u.y, u.x), hv = std::atan2(v.y, v.x);
  const double lu = u.norm(), lv = v.norm();
  // weighted bisector that matches the circle tangent for unequal legs
  return hu + normalizeAngle(hv - hu) * lu / (lu + lv);
}

struct Seg {
  bool ok = false;
  GeomElement e;                 ///< single clothoid (Newton derivatives use its end curvatures)
  std::vector<GeomElement> sub;  ///< replacement pieces when one clothoid is not possible
};

// G1 pieces between two poses: one clothoid if possible, else split at a midpoint whose heading
// follows the chord (recursively), which always converges for road-like input
bool fitPieces(const Vec2 &p0, double h0, const Vec2 &p1, double h1, int depth, std::vector<GeomElement> &out) {
  if (auto e = fitClothoidG1(p0, h0, p1, h1)) {
    out.push_back(*e);
    return true;
  }
  if (depth <= 0) return false;
  const Vec2 d = p1 - p0;
  const double ch = std::atan2(d.y, d.x);
  // midpoint pushed toward the side the tangents bend to, heading = chord direction
  const double bend = 0.25 * (normalizeAngle(h0 - ch) - normalizeAngle(h1 - ch));
  const Vec2 m = (p0 + p1) * 0.5 + Vec2(-std::sin(ch), std::cos(ch)) * (0.5 * d.norm() * std::tan(std::clamp(bend, -0.6, 0.6)));
  const double hm = ch + 0.5 * (normalizeAngle(h0 - ch) + normalizeAngle(h1 - ch)) * 0.0;
  return fitPieces(p0, h0, m, hm, depth - 1, out) && fitPieces(m, hm, p1, h1, depth - 1, out);
}

Seg segment(const Vec2 &p0, double h0, const Vec2 &p1, double h1) {
  Seg s;
  std::vector<GeomElement> pieces;
  if (fitPieces(p0, h0, p1, h1, 3, pieces)) {
    s.ok = true;
    s.e = pieces.front();
    if (pieces.size() > 1) {
      s.sub = pieces;
      s.e.k1 = pieces.back().k1;  // curvature at the far end for the G2 equations
    }
  }
  return s;
}

}  // namespace

std::optional<GeomElement> fitClothoidG1(const Vec2 &p0, double hdg0, const Vec2 &p1, double hdg1) {
  const Vec2 d = p1 - p0;
  const double r = d.norm();
  if (r < 1e-9) return std::nullopt;
  const double phi = std::atan2(d.y, d.x);
  const double phi0 = normalizeAngle(hdg0 - phi), phi1 = normalizeAngle(hdg1 - phi);
  const double delta = phi1 - phi0;
  // initial guess (Bertolazzi & Frego, eq. 20)
  const double CF[6] = {2.989696028701907, 0.716228953608281, -0.458969738821509,
                        -0.502821153340377, 0.261062141752652, -0.045854475238709};
  const double X0 = phi0 / kPi, X1 = phi1 / kPi;
  const double xy = X0 * X1, xx = X0 * X0 + X1 * X1;
  double A = (phi0 + phi1) * (CF[0] + xy * (CF[1] + xy * CF[2]) + (CF[3] + xy * CF[4]) * xx + CF[5] * xx * xx);
  double X[3], Y[3];
  bool converged = false;
  for (int it = 0; it < 50; ++it) {
    fresnelMoments(2.0 * A, delta - A, phi0, X, Y);
    const double g = Y[0];
    const double dg = X[2] - X[1];  // d/dA Y(2A, delta - A, phi0)
    if (std::abs(dg) < 1e-14) break;
    const double step = g / dg;
    A -= step;
    if (std::abs(step) < 1e-12) {
      converged = true;
      break;
    }
  }
  fresnelMoments(2.0 * A, delta - A, phi0, X, Y);
  if (!converged && std::abs(Y[0]) > 1e-9) return std::nullopt;
  if (X[0] <= 1e-12) return std::nullopt;
  GeomElement e;
  e.x = p0.x;
  e.y = p0.y;
  e.hdg = normalizeAngle(hdg0);
  e.length = r / X[0];
  e.k0 = (delta - A) / e.length;
  e.k1 = e.k0 + 2.0 * A / e.length;  // dk * L = 2A / L
  // reject spurious (looping) roots: the total turning of the minimal clothoid never exceeds the
  // turning between the end tangents by more than half a turn
  const double turning = 0.5 * (e.k0 + e.k1) * e.length;
  if (std::abs(turning) > std::abs(delta) + kPi || e.length > 4.0 * r + 1.0) return std::nullopt;
  return e;
}

ReferenceLine fitPolylineLimited(const std::vector<Vec2> &pts, const PolylineFitOptions &opt) {
  auto peak = [](const ReferenceLine &l) {
    double k = 0;
    for (const auto &e : l.elements) k = std::max({k, std::abs(e.k0), std::abs(e.k1)});
    return k;
  };
  ReferenceLine best = fitPolyline(pts, opt);
  if (opt.maxCurvature <= 0 || peak(best) <= opt.maxCurvature) return best;
  PolylineFitOptions o = opt;
  for (int attempt = 0; attempt < 5; ++attempt) {
    if (attempt == 0) o.g2 = false;                 // local G1 spline: no oscillation
    else {
      o.noiseTolerance *= 2.0;                      // ignore more digitising detail
      o.cornerAngle = std::max(0.08, o.cornerAngle * 0.5);  // fillet gentler bends too
      o.minCornerRadius = std::max(o.minCornerRadius, 1.0 / opt.maxCurvature);
    }
    ReferenceLine l = fitPolyline(pts, o);
    if (!l.elements.empty() && peak(l) < peak(best)) best = l;
    if (peak(best) <= opt.maxCurvature) break;
  }
  return best;
}

std::vector<Vec2> simplifyPolyline(const std::vector<Vec2> &pts, double tol) {
  if (pts.size() < 3) return pts;
  std::vector<bool> keep(pts.size(), false);
  keep.front() = keep.back() = true;
  dpRecurse(pts, 0, pts.size() - 1, tol, keep);
  std::vector<Vec2> out;
  for (size_t i = 0; i < pts.size(); ++i)
    if (keep[i]) out.push_back(pts[i]);
  return out;
}

namespace {
ReferenceLine fitSimplified(std::vector<Vec2> q, const PolylineFitOptions &o);
// largest curvature step between consecutive elements
double curvatureStep(const ReferenceLine &l) {
  double m = 0.0;
  for (size_t i = 1; i < l.elements.size(); ++i) m = std::max(m, std::abs(l.elements[i].k0 - l.elements[i - 1].k1));
  return m;
}
}  // namespace

ReferenceLine fitPolyline(std::vector<Vec2> pts, const PolylineFitOptions &o) {
  // 1. clean: drop duplicates, remove digitising noise
  std::vector<Vec2> src;
  for (const Vec2 &p : pts)
    if (src.empty() || (p - src.back()).norm() > 0.05) src.push_back(p);
  if (src.size() < 2) return {};
  std::vector<Vec2> q = simplifyPolyline(src, o.noiseTolerance);
  ReferenceLine line = fitSimplified(q, o);
  if (o.maxLegDeviation <= 0.0 || line.elements.empty()) return line;
  // bounded error: the spline must follow the middle of every long leg of the source polyline. A
  // vertex heading set from a short neighbouring leg tilts a long leg's spiral, which then bows away by
  // about length x tilt / 4; the point of the leg it strays from most, as an extra vertex, holds it.
  // the refits keep the first fit's end headings: the ends meet junctions and continuation roads, a
  // refinement inside the chain must not turn them
  PolylineFitOptions oe = o;
  if (!oe.startHeading) oe.startHeading = line.elements.front().hdg;
  if (!oe.endHeading) oe.endHeading = line.eval(line.length()).hdg;
  // a corner sharp enough for a fillet owns up to R (tan(turn / 2) + turn / 2) of each leg next to it
  const double R = std::max(o.minCornerRadius, o.maxCurvature > 0.0 ? 1.0 / o.maxCurvature : 0.0);
  std::vector<double> reach(src.size(), 0.0);
  for (size_t i = 1; i + 1 < src.size(); ++i) {
    const Vec2 d0 = src[i] - src[i - 1], d1 = src[i + 1] - src[i];
    const double turn = std::abs(normalizeAngle(std::atan2(d1.y, d1.x) - std::atan2(d0.y, d0.x)));
    if (turn > o.cornerAngle) reach[i] = R * (std::tan(0.5 * std::min(turn, 2.8)) + 0.5 * turn) + 1.0;
  }
  for (int round = 0; round < 12; ++round) {
    double worst = o.maxLegDeviation;
    std::optional<Vec2> add;
    for (size_t i = 0; i + 1 < src.size(); ++i) {
      const Vec2 a = src[i], b = src[i + 1];
      const double L = (b - a).norm();
      if (L < o.minBoundLeg) continue;
      for (double u : {0.25, 0.5, 0.75}) {
        if (u * L < reach[i] || (1.0 - u) * L < reach[i + 1]) continue;
        double s, t;
        line.project(a + (b - a) * u, s, t);
        if (std::abs(t) > worst) worst = std::abs(t), add = a + (b - a) * u;
      }
    }
    if (!add) break;
    // insert the midpoint in source order (between the kept vertices around it)
    double sAdd, tAdd;
    line.project(*add, sAdd, tAdd);
    size_t at = 1;
    for (; at < q.size(); ++at) {
      double sq, tq;
      line.project(q[at], sq, tq);
      if (sq > sAdd) break;
    }
    if (at >= q.size() || (q[at] - *add).norm() < 0.5 || (q[at - 1] - *add).norm() < 0.5) break;
    q.insert(q.begin() + static_cast<long>(at), *add);
    ReferenceLine refit = fitSimplified(q, oe);
    // the extra vertex must not cost curvature continuity (a G2 solve that rings around it falls back
    // to G1 and kinks): then the closer fit is not taken
    if (refit.elements.empty() || curvatureStep(refit) > curvatureStep(line) + 1e-6) break;
    line = refit;
  }
  return line;
}

namespace {
ReferenceLine fitSimplified(std::vector<Vec2> q, const PolylineFitOptions &o) {
  // 1. fillet sharp corners
  std::vector<double> legH;
  std::vector<char> pairAt;
  q = filletCorners(q, o, &legH, &pairAt);
  const size_t n = q.size();

  // 2. vertex headings: circle tangents inside (a fillet tangent point takes its leg's heading),
  //    fixed or extrapolated at the ends
  std::vector<double> h(n);
  for (size_t i = 1; i + 1 < n; ++i)
    h[i] = i < legH.size() && !std::isnan(legH[i]) ? legH[i] : circleTangent(q[i - 1], q[i], q[i + 1]);
  auto chord = [&](size_t i) { return std::atan2(q[i + 1].y - q[i].y, q[i + 1].x - q[i].x); };
  if (n == 2) {
    h[0] = o.startHeading.value_or(chord(0));
    h[1] = o.endHeading.value_or(o.startHeading ? 2.0 * chord(0) - h[0] : chord(0));
  } else {
    // free ends: mirror the interior tangent about the end chord (zero end-curvature change)
    h[0] = o.startHeading.value_or(2.0 * chord(0) - h[1]);
    h[n - 1] = o.endHeading.value_or(2.0 * chord(n - 2) - h[n - 2]);
  }

  // 3. G2: Newton on interior headings so that curvature is continuous at every vertex
  auto build = [&](const std::vector<double> &hh, std::vector<Seg> &segs) {
    segs.resize(n - 1);
    bool all = true;
    for (size_t i = 0; i + 1 < n; ++i) {
      segs[i] = Seg{};
      if (i < pairAt.size() && pairAt[i])
        if (auto pr = clothoidPair(q[i], hh[i], q[i + 1], hh[i + 1])) {
          segs[i].ok = true;
          segs[i].sub = *pr;
          segs[i].e = pr->front();
          segs[i].e.k1 = pr->back().k1;  // 0: the pair ends on the straight leg
        }
      if (!segs[i].ok) segs[i] = segment(q[i], hh[i], q[i + 1], hh[i + 1]);
      all = all && segs[i].ok;
    }
    return all;
  };
  std::vector<Seg> segs;
  bool ok = build(h, segs);
  if (ok && o.g2 && n > 2) {
    const size_t m = n - 2;  // unknowns h[1..n-2]
    // fillet tangent points keep their leg heading (curvature may step there, G1): letting Newton
    // turn them would bend the straight legs next to a sharp corner
    std::vector<char> fixedV(n, 0);
    for (size_t i = 1; i + 1 < n; ++i) fixedV[i] = i < legH.size() && !std::isnan(legH[i]);
    for (int iter = 0; iter < 30; ++iter) {
      std::vector<double> F(m);
      double norm = 0.0;
      for (size_t j = 0; j < m; ++j) {
        F[j] = fixedV[j + 1] ? 0.0 : segs[j].e.k1 - segs[j + 1].e.k0;
        norm = std::max(norm, std::abs(F[j]));
      }
      if (norm < 1e-9) break;
      // tridiagonal Jacobian by finite differences (each heading affects two segments)
      std::vector<double> lo(m, 0.0), di(m, 0.0), up(m, 0.0);
      const double eps = 1e-6;
      bool jacOk = true;
      for (size_t j = 0; j < m && jacOk; ++j) {
        const size_t v = j + 1;
        const Seg a = segment(q[v - 1], h[v - 1], q[v], h[v] + eps);
        const Seg b = segment(q[v], h[v] + eps, q[v + 1], h[v + 1]);
        if (!a.ok || !b.ok) jacOk = false;
        if (!jacOk) break;
        // vertex v joins segment j (v-1 -> v) and segment j+1 (v -> v+1)
        // F[j]   = segs[j].k1   - segs[j+1].k0  : both depend on h[v]
        // F[j-1] = segs[j-1].k1 - segs[j].k0    : depends on h[v] through segs[j].k0
        // F[j+1] = segs[j+1].k1 - segs[j+2].k0  : depends on h[v] through segs[j+1].k1
        di[j] = ((a.e.k1 - b.e.k0) - F[j]) / eps;
        if (j > 0) up[j - 1] = -(a.e.k0 - segs[j].e.k0) / eps;
        if (j + 1 < m) lo[j + 1] = (b.e.k1 - segs[j + 1].e.k1) / eps;
      }
      if (!jacOk) break;
      for (size_t j = 0; j < m; ++j)
        if (fixedV[j + 1]) {
          di[j] = 1.0, lo[j] = up[j] = 0.0;
          if (j > 0) up[j - 1] = 0.0;
          if (j + 1 < m) lo[j + 1] = 0.0;
        }
      // Thomas algorithm
      std::vector<double> c(m), d(m), x(m);
      for (size_t j = 0; j < m; ++j) {
        const double denom = di[j] - (j ? lo[j] * c[j - 1] : 0.0);
        if (std::abs(denom) < 1e-14) { jacOk = false; break; }
        c[j] = (j + 1 < m) ? up[j] / denom : 0.0;
        d[j] = (-F[j] - (j ? lo[j] * d[j - 1] : 0.0)) / denom;
      }
      if (!jacOk) break;
      for (size_t j = m; j-- > 0;) x[j] = d[j] - (j + 1 < m ? c[j] * x[j + 1] : 0.0);
      // damped update
      std::vector<double> h2 = h;
      double lambda = 1.0;
      std::vector<Seg> s2;
      bool accepted = false;
      for (int ls = 0; ls < 8; ++ls, lambda *= 0.5) {
        for (size_t j = 0; j < m; ++j)
          if (!fixedV[j + 1]) h2[j + 1] = h[j + 1] + lambda * std::clamp(x[j], -0.5, 0.5);
        if (!build(h2, s2)) continue;
        double n2 = 0.0;
        for (size_t j = 0; j < m; ++j)
          if (!fixedV[j + 1]) n2 = std::max(n2, std::abs(s2[j].e.k1 - s2[j + 1].e.k0));
        if (n2 < norm) {
          h = h2;
          segs = s2;
          accepted = true;
          break;
        }
      }
      if (!accepted) break;
    }
  }
  // G2 interpolation can oscillate where short segments (fillets) meet long ones: compare with the
  // local G1 spline (circle tangents) and keep the calmer one
  auto maxK = [](const std::vector<Seg> &v) {
    double k = 0;
    for (const Seg &x : v)
      if (x.ok) k = std::max({k, std::abs(x.e.k0), std::abs(x.e.k1)});
    return k;
  };
  if (ok && o.g2 && n > 2) {
    std::vector<double> hg1(n);
    hg1.front() = h.front();
    hg1.back() = h.back();
    for (size_t i = 1; i + 1 < n; ++i)
      hg1[i] = i < legH.size() && !std::isnan(legH[i]) ? legH[i] : circleTangent(q[i - 1], q[i], q[i + 1]);
    std::vector<Seg> s1;
    if (build(hg1, s1) && maxK(segs) > 1.5 * maxK(s1) + 0.01) segs = s1;
  }
  if (!ok) {  // fall back to chord-bisector headings (always solvable)
    for (size_t i = 1; i + 1 < n; ++i)
      h[i] = i < legH.size() && !std::isnan(legH[i]) ? legH[i] : chord(i - 1) + 0.5 * normalizeAngle(chord(i) - chord(i - 1));
    build(h, segs);
  }

  ReferenceLine line;
  for (size_t i = 0; i < segs.size(); ++i) {
    if (segs[i].ok) {
      if (segs[i].sub.empty()) line.elements.push_back(segs[i].e);
      else line.elements.insert(line.elements.end(), segs[i].sub.begin(), segs[i].sub.end());
      continue;
    }
    GeomElement e;  // degenerate configuration: straight chord
    e.x = q[i].x;
    e.y = q[i].y;
    e.hdg = chord(i);
    e.length = (q[i + 1] - q[i]).norm();
    line.elements.push_back(e);
  }
  line.finalize();
  line.simplify(o.straightTolerance);
  return line;
}
}  // namespace

}  // namespace odrforge
