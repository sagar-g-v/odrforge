// OdrForge - junction reference line and elevation grid (standard profile, OpenDRIVE 1.8+).
//
// OpenDRIVE 1.8.0 §12.9 junction reference line: a <planView> with one <geometry> holding one <line>
// (rule junctions.geometry.only_one_line_element), placed "in a way that every point of the junction can be
// reached with a perpendicular straight line" (junctions.geometry.ref_line_definition).
// §12.11 elevation grid: rows perpendicular to that line (junctions.elevation_grid.perpendicular_vectors) at
// sStart + k gridSpacing, each with a center value and left / right values from inside to outside, made of
// complete squares; heights inside are bicubically interpolated (§12.11.1), and the grid "shall override any
// elevation values derived from any roads that are part of the junction".
// The junction's surface here is the plane the connecting roads already lie on (elevation.cpp
// junctionSurfaces): the grid samples that plane, and bicubic interpolation of planar values is exact
// (fitted edge cubics are linear), so a reader of the grid gets the connecting roads' own heights. Line and
// grid cover the connecting roads' lanes out to their outer borders; no junction boundary is written (it is
// optional; without one the grid has no transition zones and the incoming roads already meet the plane C1).
// Common junctions with a plane only (1.8.0 §12.11: "currently only for common junctions").

#include <cmath>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
constexpr double kGridSpacing = 5.0;  // m; the plane needs no finer grid (Junction Guideline §6.5 example: 5 m)
constexpr double kSample = 1.0;       // m, along the connecting roads
constexpr double kMargin = 0.5;       // m, around the sampled extent
}  // namespace

void Builder::junctionGrids() {
  if (!opt.target.spec().hasJunctionGrid()) return;
  std::map<std::string, std::vector<const odr::Road *>> roadsOf;  // junction id -> its connecting roads
  for (const odr::Road &r : od.roads)
    if (r.junction != "-1" && !r.lanes.empty()) roadsOf[r.junction].push_back(&r);
  for (const Cluster &cl : clusters) {
    if (cl.junction < 0 || !cl.hasPlane) continue;
    odr::Junction &J = od.junctions[cl.junction];
    if (J.type != "default" || !roadsOf.count(J.id)) continue;
    // the junction's extent: every lane border of its connecting roads
    std::vector<Vec2> pts;
    for (const odr::Road *r : roadsOf[J.id]) {
      const int n = std::max(2, static_cast<int>(std::ceil(r->length() / kSample)) + 1);
      for (int i = 0; i < n; ++i) {
        const double s = r->length() * i / (n - 1);
        const odr::LaneSection &sec = odr::sectionAt(*r, s);
        const Pose p = r->planView.eval(s);
        pts.push_back(p.offset(odr::laneOffsetAt(*r, s)));
        for (const auto *side : {&sec.left, &sec.right})
          for (const odr::Lane &l : *side) {
            double a, b;
            if (odr::laneBorders(*r, s, l.id, a, b)) pts.push_back(p.offset(b));
          }
      }
    }
    if (pts.size() < 3) continue;
    // the line along the principal axis of that extent, through its centroid
    Vec2 c{0, 0};
    for (const Vec2 &q : pts) c = c + q;
    c = c * (1.0 / pts.size());
    double sxx = 0, syy = 0, sxy = 0;
    for (const Vec2 &q : pts) {
      const Vec2 d = q - c;
      sxx += d.x * d.x, syy += d.y * d.y, sxy += d.x * d.y;
    }
    const double hdg = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const Vec2 dir{std::cos(hdg), std::sin(hdg)}, left{-std::sin(hdg), std::cos(hdg)};
    double s0 = 1e300, s1 = -1e300, tl = 0.0, tr = 0.0;
    for (const Vec2 &q : pts) {
      const Vec2 d = q - c;
      s0 = std::min(s0, d.dot(dir)), s1 = std::max(s1, d.dot(dir));
      tl = std::max(tl, d.dot(left)), tr = std::max(tr, -d.dot(left));
    }
    // a margin beyond the sampled extent: the lanes between the samples stay inside too
    s0 -= kMargin, s1 += kMargin, tl += kMargin, tr += kMargin;
    odr::JunctionGrid g;
    const Vec2 o = c + dir * s0;
    g.x = o.x, g.y = o.y, g.hdg = hdg, g.length = std::max(s1 - s0, 1e-3);
    g.gridSpacing = kGridSpacing;
    g.sStart = 0.0;
    // complete squares covering the extent: rows 0 .. K along the line, columns out to tl / tr
    const int K = std::max(1, static_cast<int>(std::ceil(g.length / kGridSpacing - 1e-9)));
    const int nl = static_cast<int>(std::ceil(tl / kGridSpacing - 1e-9)), nr = static_cast<int>(std::ceil(tr / kGridSpacing - 1e-9));
    for (int k = 0; k <= K; ++k) {
      const Vec2 rowAt = o + dir * (k * kGridSpacing);
      odr::JunctionGrid::Row row;
      row.center = planeZ(cl, rowAt);
      for (int j = 1; j <= nl; ++j) row.left.push_back(planeZ(cl, rowAt + left * (j * kGridSpacing)));
      for (int j = 1; j <= nr; ++j) row.right.push_back(planeZ(cl, rowAt + left * (-j * kGridSpacing)));
      g.rows.push_back(row);
    }
    J.grid = g;
    ++rep.junctionGrids;
  }
}

}  // namespace detail
}  // namespace odrforge
