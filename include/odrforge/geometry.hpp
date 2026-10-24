// OdrForge - OpenDRIVE reference-line geometry (planView): line, arc, spiral (clothoid).
// Only the standard OpenDRIVE primitives are produced; poly3 / paramPoly3 are never needed because
// every curve is represented by its curvature profile (piecewise linear in s => line/arc/spiral).
#pragma once

#include <cmath>
#include <memory>
#include <vector>

namespace odrforge {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
  double x = 0.0, y = 0.0;
  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}
  Vec2 operator+(const Vec2 &o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2 &o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double k) const { return {x * k, y * k}; }
  double dot(const Vec2 &o) const { return x * o.x + y * o.y; }
  double cross(const Vec2 &o) const { return x * o.y - y * o.x; }
  double norm() const { return std::hypot(x, y); }
};

/// Position + heading (+ curvature) on a curve, OpenDRIVE inertial frame (x east, y north, hdg CCW from x).
struct Pose {
  double x = 0.0, y = 0.0, hdg = 0.0, kappa = 0.0;
  Vec2 pos() const { return {x, y}; }
  Vec2 dir() const { return {std::cos(hdg), std::sin(hdg)}; }
  Vec2 left() const { return {-std::sin(hdg), std::cos(hdg)}; }
  /// point at lateral offset t (positive = left of the direction of travel, OpenDRIVE convention)
  Vec2 offset(double t) const { return pos() + left() * t; }
};

double normalizeAngle(double a);  ///< (-pi, pi]

enum class GeomType { Line, Arc, Spiral, Poly3, ParamPoly3 };

/// Cubic polynomial geometry read from other generators (OdrForge itself never writes these):
/// poly3: v(u) = a + b u + c u^2 + d u^3; paramPoly3: u(p), v(p) with p in [0, 1] or [0, length].
struct CubicGeom {
  bool param = false, normalized = true;
  double aU = 0, bU = 1, cU = 0, dU = 0, aV = 0, bV = 0, cV = 0, dV = 0;
};

/// One planView <geometry> record. Curvature varies linearly from k0 to k1 over length
/// (Line: k0 = k1 = 0, Arc: k0 = k1 != 0, Spiral: otherwise).
struct GeomElement {
  double s = 0.0, x = 0.0, y = 0.0, hdg = 0.0, length = 0.0, k0 = 0.0, k1 = 0.0;
  std::shared_ptr<CubicGeom> cubic;  ///< set for poly3 / paramPoly3 (read only)
  GeomType type() const;
  Pose eval(double ds) const;  ///< pose at local distance ds in [0, length]
  Pose start() const { return eval(0.0); }
  Pose end() const { return eval(length); }
};

/// Generalized Fresnel integrals X(a,b,c) = int_0^1 cos(a/2 t^2 + b t + c) dt, Y = ... sin(...).
void generalizedFresnel(double a, double b, double c, double &X, double &Y);

/// A road reference line: consecutive planView elements (G1 continuous by construction).
class ReferenceLine {
 public:
  std::vector<GeomElement> elements;

  double length() const;
  Pose eval(double s) const;  ///< clamps s to [0, length]
  /// Closest point on the line: returns s and signed lateral offset t (left positive).
  void project(const Vec2 &p, double &s, double &t) const;
  /// What project() evaluates on the line itself, whatever the point: every element's midpoint and its coarse samples.
  struct ProjectionAid {
    std::vector<Vec2> mids;     ///< per element
    std::vector<size_t> first;  ///< per element its first sample in points; one more entry: the end
    std::vector<Vec2> points;   ///< the coarse samples, element by element
  };
  /// For a line projected onto many times: computed once, as project() computes them.
  ProjectionAid projectionAid() const;
  /// project() with projectionAid() of this line (unchanged since): the same result without evaluating the line's
  /// elements on every call. An aid of another size is ignored.
  void project(const Vec2 &p, double &s, double &t, const ProjectionAid &aid) const;
  /// Re-number s along the line and recompute element start poses from the first element
  /// (removes drift so consecutive elements are exactly continuous).
  void finalize();
  /// Reverse direction (s = 0 becomes the old end).
  ReferenceLine reversed() const;
  /// Sub-line between s0 and s1 (s0 < s1); elements are cut, curvatures interpolated.
  ReferenceLine slice(double s0, double s1) const;
  /// Merge nearly-straight / nearly-constant-curvature elements, drop zero-length ones.
  void simplify(double kTol = 1e-5);
  /// Fold elements shorter than minLen into a neighbour. OpenDRIVE 1.8.0 §9.2 only requires @length > 0
  /// (XSD t_grZero); the minLen floor (1 cm) is our own guard against degenerate elements, not a spec value.
  void dropTiny(double minLen = 0.01);  // elements under 1 cm are folded into a neighbour (validator rule spec2.min_geometry_length)

 private:
  void projectImpl(const Vec2 &p, double &s, double &t, const ProjectionAid *aid) const;
};

}  // namespace odrforge
