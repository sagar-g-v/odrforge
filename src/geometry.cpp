#include "odrforge/geometry.hpp"

#include <algorithm>
#include <limits>

namespace odrforge {

double normalizeAngle(double a) {
  a = std::fmod(a, 2.0 * kPi);
  if (a <= -kPi) a += 2.0 * kPi;
  if (a > kPi) a -= 2.0 * kPi;
  return a;
}

GeomType GeomElement::type() const {
  if (cubic) return cubic->param ? GeomType::ParamPoly3 : GeomType::Poly3;
  constexpr double eps = 1e-9;
  if (std::abs(k0) < eps && std::abs(k1) < eps) return GeomType::Line;
  if (std::abs(k1 - k0) < eps) return GeomType::Arc;
  return GeomType::Spiral;
}

// Gauss-Legendre 16-point nodes / weights on [-1, 1]; the integrand is integrated on enough
// sub-intervals that every sub-interval spans less than ~1 rad of phase change (machine precision
// for the curvatures and lengths that occur on roads).
static const double kGLx[8] = {0.0950125098376374, 0.2816035507792589, 0.4580167776572274, 0.6178762444026438,
                               0.7554044083550030, 0.8656312023878318, 0.9445750230732326, 0.9894009349916499};
static const double kGLw[8] = {0.1894506104550685, 0.1826034150449236, 0.1691565193950025, 0.1495959888165767,
                               0.1246289712555339, 0.0951585116824928, 0.0622535239386479, 0.0271524594117541};

void generalizedFresnel(double a, double b, double c, double &X, double &Y) {
  const double phaseSpan = std::abs(a) * 0.5 + std::abs(b) + 1.0;
  const int n = std::max(1, static_cast<int>(std::ceil(phaseSpan)));
  const double h = 1.0 / n;
  X = Y = 0.0;
  for (int i = 0; i < n; ++i) {
    const double mid = (i + 0.5) * h, half = 0.5 * h;
    for (int k = 0; k < 8; ++k) {
      for (int sgn = -1; sgn <= 1; sgn += 2) {
        const double t = mid + sgn * half * kGLx[k];
        const double ph = 0.5 * a * t * t + b * t + c;
        X += kGLw[k] * half * std::cos(ph);
        Y += kGLw[k] * half * std::sin(ph);
      }
    }
  }
}

namespace {
// cubic geometry: integrate arc length numerically to map ds -> curve parameter
Pose evalCubic(const GeomElement &e, double ds) {
  const CubicGeom &c = *e.cubic;
  auto uv = [&](double p, double &u, double &v, double &du, double &dv, double &ddu, double &ddv) {
    if (c.param) {
      u = c.aU + p * (c.bU + p * (c.cU + p * c.dU));
      v = c.aV + p * (c.bV + p * (c.cV + p * c.dV));
      du = c.bU + p * (2 * c.cU + 3 * p * c.dU);
      dv = c.bV + p * (2 * c.cV + 3 * p * c.dV);
      ddu = 2 * c.cU + 6 * p * c.dU;
      ddv = 2 * c.cV + 6 * p * c.dV;
    } else {  // poly3: parameter is u
      u = p;
      v = c.aV + p * (c.bV + p * (c.cV + p * c.dV));
      du = 1;
      dv = c.bV + p * (2 * c.cV + 3 * p * c.dV);
      ddu = 0;
      ddv = 2 * c.cV + 6 * p * c.dV;
    }
  };
  // parameter range: [0,1] (normalized), [0,length] (arcLength) or u in [0, ?] for poly3 (solve by length)
  const double pEnd = c.param ? (c.normalized ? 1.0 : e.length) : e.length;
  const int n = 512;
  double s = 0, p = 0, u, v, du, dv, ddu, ddv;
  const double h = pEnd / n;
  for (int i = 0; i < n && s < ds; ++i) {
    uv(p + 0.5 * h, u, v, du, dv, ddu, ddv);
    const double step = std::hypot(du, dv) * h;
    if (s + step >= ds) {
      p += h * (ds - s) / step;
      s = ds;
      break;
    }
    s += step;
    p += h;
  }
  uv(p, u, v, du, dv, ddu, ddv);
  const double ch = std::cos(e.hdg), sh = std::sin(e.hdg);
  Pose q;
  q.x = e.x + u * ch - v * sh;
  q.y = e.y + u * sh + v * ch;
  q.hdg = e.hdg + std::atan2(dv, du);
  const double sp = std::hypot(du, dv);
  q.kappa = sp > 1e-12 ? (du * ddv - dv * ddu) / (sp * sp * sp) : 0.0;
  return q;
}
}  // namespace

Pose GeomElement::eval(double ds) const {
  if (cubic) return evalCubic(*this, ds);
  Pose p;
  const double dk = length > 0.0 ? (k1 - k0) / length : 0.0;
  p.kappa = k0 + dk * ds;
  p.hdg = hdg + k0 * ds + 0.5 * dk * ds * ds;
  if (ds == 0.0) {
    p.x = x;
    p.y = y;
    return p;
  }
  if (std::abs(dk) < 1e-14) {
    if (std::abs(k0) < 1e-14) {  // line
      p.x = x + ds * std::cos(hdg);
      p.y = y + ds * std::sin(hdg);
    } else {  // arc
      p.x = x + (std::sin(hdg + k0 * ds) - std::sin(hdg)) / k0;
      p.y = y - (std::cos(hdg + k0 * ds) - std::cos(hdg)) / k0;
    }
    return p;
  }
  // spiral: x(ds) = ds * X(dk ds^2, k0 ds, hdg)
  double X, Y;
  generalizedFresnel(dk * ds * ds, k0 * ds, hdg, X, Y);
  p.x = x + ds * X;
  p.y = y + ds * Y;
  return p;
}

double ReferenceLine::length() const {
  return elements.empty() ? 0.0 : elements.back().s + elements.back().length;
}

Pose ReferenceLine::eval(double s) const {
  if (elements.empty()) return {};
  s = std::clamp(s, 0.0, length());
  auto it = std::upper_bound(elements.begin(), elements.end(), s,
                             [](double v, const GeomElement &e) { return v < e.s; });
  const GeomElement &e = (it == elements.begin()) ? elements.front() : *(it - 1);
  return e.eval(std::clamp(s - e.s, 0.0, e.length));
}

void ReferenceLine::project(const Vec2 &p, double &sOut, double &tOut) const {
  // coarse sampling then Newton refinement on the tangent condition
  const double L = length();
  double best = std::numeric_limits<double>::max(), bestS = 0.0;
  for (const GeomElement &e : elements) {
    // bounding-circle rejection, then a few samples per element (curvature-aware)
    const Pose mid = e.eval(0.5 * e.length);
    if ((mid.pos() - p).norm() - 0.5 * e.length > best) continue;
    const double turn = std::abs(0.5 * (e.k0 + e.k1)) * e.length + std::abs(e.k1 - e.k0) * e.length;
    const int n = std::max(4, static_cast<int>(std::min(e.length / 2.0, 8.0 + 16.0 * turn)));
    for (int i = 0; i <= n; ++i) {
      const double ds = e.length * i / n;
      const double d = (e.eval(ds).pos() - p).norm();
      if (d < best) best = d, bestS = e.s + ds;
    }
  }
  double s = bestS;
  for (int it = 0; it < 20; ++it) {
    const Pose q = eval(s);
    const Vec2 d = p - q.pos();
    const double f = d.dot(q.dir());                    // tangential error
    const double df = -1.0 + q.kappa * d.dot(q.left());  // derivative of f w.r.t. s
    if (std::abs(df) < 1e-9) break;
    const double step = -f / df;
    s = std::clamp(s + step, 0.0, L);
    if (std::abs(step) < 1e-9) break;
  }
  const Pose q = eval(s);
  sOut = s;
  tOut = (p - q.pos()).dot(q.left());
}

void ReferenceLine::finalize() {
  double s = 0.0;
  for (size_t i = 0; i < elements.size(); ++i) {
    GeomElement &e = elements[i];
    if (i > 0) {
      const Pose prev = elements[i - 1].end();
      e.x = prev.x;
      e.y = prev.y;
      e.hdg = normalizeAngle(prev.hdg);
    }
    e.s = s;
    s += e.length;
  }
}

ReferenceLine ReferenceLine::reversed() const {
  ReferenceLine r;
  for (auto it = elements.rbegin(); it != elements.rend(); ++it) {
    GeomElement e;
    const Pose end = it->end();
    e.x = end.x;
    e.y = end.y;
    e.hdg = normalizeAngle(end.hdg + kPi);
    e.length = it->length;
    e.k0 = -it->k1;  // travelling backwards flips the sign of curvature
    e.k1 = -it->k0;
    r.elements.push_back(e);
  }
  r.finalize();
  return r;
}

ReferenceLine ReferenceLine::slice(double s0, double s1) const {
  ReferenceLine r;
  for (const GeomElement &e : elements) {
    const double a = std::max(s0, e.s), b = std::min(s1, e.s + e.length);
    if (b - a <= 1e-9) continue;
    const double dk = e.length > 0 ? (e.k1 - e.k0) / e.length : 0.0;
    GeomElement c;
    const Pose p = e.eval(a - e.s);
    c.x = p.x;
    c.y = p.y;
    c.hdg = normalizeAngle(p.hdg);
    c.length = b - a;
    c.k0 = e.k0 + dk * (a - e.s);
    c.k1 = e.k0 + dk * (b - e.s);
    r.elements.push_back(c);
  }
  r.finalize();
  r.dropTiny();
  return r;
}

void ReferenceLine::dropTiny(double minLen) {
  if (elements.size() < 2) return;
  std::vector<GeomElement> out;
  double carry = 0.0;  // length of dropped leading elements, added to the next kept one
  for (const GeomElement &e : elements) {
    if (e.length < minLen) {
      if (!out.empty()) out.back().length += e.length;
      else carry += e.length;
      continue;
    }
    out.push_back(e);
    if (carry > 0) {
      out.back().length += carry;
      carry = 0;
    }
  }
  if (out.empty()) return;  // everything tiny: keep as is
  // the first kept element must start where the line started
  out.front().x = elements.front().x;
  out.front().y = elements.front().y;
  out.front().hdg = elements.front().hdg;
  elements = out;
  finalize();
}

void ReferenceLine::simplify(double kTol) {
  std::vector<GeomElement> out;
  for (GeomElement e : elements) {
    if (e.length < 1e-6) continue;
    if (std::abs(e.k0) < kTol && std::abs(e.k1) < kTol) e.k0 = e.k1 = 0.0;
    else if (std::abs(e.k1 - e.k0) < kTol) e.k0 = e.k1 = 0.5 * (e.k0 + e.k1);
    if (!out.empty()) {
      GeomElement &p = out.back();
      const bool sameType = p.type() == e.type();
      const bool contiguous = std::abs(p.k1 - e.k0) < kTol;
      const bool linearK = p.type() != GeomType::Spiral ||
                           std::abs((p.k1 - p.k0) / p.length - (e.k1 - e.k0) / e.length) < kTol * 0.01;
      if (sameType && contiguous && linearK) {
        p.length += e.length;
        p.k1 = e.k1;
        continue;
      }
    }
    out.push_back(e);
  }
  elements = out;
  finalize();
  dropTiny();
}

}  // namespace odrforge
