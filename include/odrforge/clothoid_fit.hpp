// OdrForge - fitting OpenDRIVE spirals (clothoids) to OSM polylines and to junction movements.
#pragma once

#include <optional>
#include <vector>

#include "odrforge/geometry.hpp"

namespace odrforge {

/// G1 Hermite interpolation with a single clothoid (Bertolazzi & Frego, 2015): the unique clothoid
/// from (p0, hdg0) to (p1, hdg1) with the smallest |turn|. Returns nullopt if it does not converge.
std::optional<GeomElement> fitClothoidG1(const Vec2 &p0, double hdg0, const Vec2 &p1, double hdg1);

struct PolylineFitOptions {
  double noiseTolerance = 0.25;   ///< Douglas-Peucker tolerance (m): OSM digitising noise to ignore
  double minCornerRadius = 8.0;   ///< sharp polyline corners are filleted with at least this radius (m)
  double cornerAngle = 0.35;      ///< corners turning more than this (rad) between short legs are filleted
  bool g2 = true;                 ///< curvature-continuous spline (else G1 with bisector headings)
  std::optional<double> startHeading, endHeading;  ///< fixed end tangents (e.g. continuation roads)
  double straightTolerance = 2e-5;  ///< |curvature| below this (1/m) becomes a line
  double maxCurvature = 0.0;        ///< > 0: refit with more smoothing until |curvature| stays below
};

/// Clothoid spline through a polyline: every segment between kept vertices becomes one spiral
/// (merged into lines / arcs where the curvature is (nearly) zero / constant).
ReferenceLine fitPolyline(std::vector<Vec2> pts, const PolylineFitOptions &opt = {});
/// fitPolyline honouring opt.maxCurvature: G2 -> G1 -> stronger smoothing and corner fillets
ReferenceLine fitPolylineLimited(const std::vector<Vec2> &pts, const PolylineFitOptions &opt);

/// Douglas-Peucker simplification (keeps first / last and any index listed in keep).
std::vector<Vec2> simplifyPolyline(const std::vector<Vec2> &pts, double tol);

}  // namespace odrforge
