#include <algorithm>
#include <cmath>

#include "odrforge/odr.hpp"

namespace odrforge::odr {

namespace {
template <class V>
const Poly3 *polyAt(const V &v, double s) {
  const Poly3 *p = nullptr;
  for (const auto &e : v)
    if (e.s <= s + 1e-9) p = &e;
  return p;
}
}  // namespace

const LaneSection &sectionAt(const Road &r, double s) {
  const LaneSection *sec = &r.lanes.front();
  for (const auto &x : r.lanes)
    if (x.s <= s + 1e-9) sec = &x;
  return *sec;
}

double laneOffsetAt(const Road &r, double s) {
  const Poly3 *p = polyAt(r.laneOffset, s);
  return p ? p->eval(s - p->s) : 0.0;
}

double laneWidthAt(const Lane &l, double ds) {
  const Poly3 *p = polyAt(l.widths, ds);
  return p ? std::max(0.0, p->eval(ds - p->s)) : 0.0;
}

const Lane *findLane(const LaneSection &sec, int laneId) {
  const auto &side = laneId > 0 ? sec.left : sec.right;
  for (const auto &l : side)
    if (l.id == laneId) return &l;
  return nullptr;
}

bool laneBorders(const Road &r, double s, int laneId, double &tInner, double &tOuter) {
  if (laneId == 0 || r.lanes.empty()) return false;
  const LaneSection &sec = sectionAt(r, s);
  const double ds = s - sec.s, sign = laneId > 0 ? 1.0 : -1.0;
  double t = laneOffsetAt(r, s);
  const auto &side = laneId > 0 ? sec.left : sec.right;
  for (const auto &l : side) {
    const double w = laneWidthAt(l, ds);
    if (l.id == laneId) {
      tInner = t;
      tOuter = t + sign * w;
      return true;
    }
    t += sign * w;
  }
  return false;
}

bool laneCenter(const Road &r, double s, int laneId, Pose &out) {
  double a, b;
  if (!laneBorders(r, s, laneId, a, b)) return false;
  const Pose p = r.planView.eval(s);
  const Vec2 c = p.offset(0.5 * (a + b));
  out = p;
  out.x = c.x;
  out.y = c.y;
  return true;
}

double elevationAt(const Road &r, double s) {
  const Poly3 *p = polyAt(r.elevation, s);
  return p ? p->eval(s - p->s) : 0.0;
}

double elevationSlopeAt(const Road &r, double s) {
  const Poly3 *p = polyAt(r.elevation, s);
  return p ? p->deriv(s - p->s) : 0.0;
}

}  // namespace odrforge::odr
