// OdrForge - mid-block pedestrian crossings as cross paths (standard profile).
//
// OpenDRIVE 1.9 §12.5 cross paths (introduced 1.8.0) and §12.7 virtual junctions; ASAM Junction Guideline
// §10.3: a pedestrian crossing between the sidewalks of a road that is not cut is a junction of type
// virtual on that road (mainRoad, sStart .. sEnd), holding a <crossPath>: its crossing road - one walking
// lane straight across the carriageway, from the right sidewalk's inner edge to the left one's - and the
// sidewalk lanes it links at that s on either side. The crosswalk object stays (the marking); the carla
// profile (1.4, no cross paths) keeps the object only.

#include <algorithm>
#include <cmath>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

/// the outermost lane of a side when it is a sidewalk: its id, else 0
int sidewalkOf(const std::vector<odr::Lane> &side) {
  return !side.empty() && side.back().type == "sidewalk" ? side.back().id : 0;
}

long numericId(const std::string &id) {
  try {
    return std::stol(id);
  } catch (...) {
    return 0;
  }
}

}  // namespace

void Builder::crossPaths() {
  if (!opt.target.spec().hasCrossPaths()) return;
  long nextRoad = 0, nextJunction = 0;
  for (const odr::Road &r : od.roads) nextRoad = std::max(nextRoad, numericId(r.id));
  for (const odr::Junction &j : od.junctions) nextJunction = std::max(nextJunction, numericId(j.id));
  std::vector<odr::Road> crossing;  // appended after the loop: the roads must not move while referenced
  for (const auto &[ri, objectId] : midBlockCrosswalks) {
    const odr::Road &r = od.roads[ri];
    const auto o = std::find_if(r.objects.begin(), r.objects.end(), [&](const odr::Object &x) { return x.id == objectId; });
    if (o == r.objects.end()) continue;
    const double s = o->s;
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    const int walkL = sidewalkOf(sec.left), walkR = sidewalkOf(sec.right);
    if (!walkL || !walkR) continue;  // a sidewalk on both sides to link
    double lIn, lOut, rIn, rOut;
    if (!odr::laneBorders(r, s, walkL, lIn, lOut) || !odr::laneBorders(r, s, walkR, rIn, rOut) || lIn - rIn < 1.0) continue;
    // the crossing road: straight across, from the right sidewalk's inner edge to the left one's
    const Pose p = r.planView.eval(s);
    odr::Road x;
    x.id = std::to_string(++nextRoad);
    x.name = "Crossing";
    x.junction = std::to_string(nextJunction + 1);
    x.rule = r.rule;
    const Vec2 a = p.offset(rIn);
    GeomElement e;
    e.x = a.x, e.y = a.y, e.hdg = p.hdg + 0.5 * kPi, e.length = lIn - rIn;
    x.planView.elements = {e};
    x.planView.finalize();
    x.types = {odr::Road::Type{0.0, "pedestrian", "", std::nullopt, "km/h"}};
    // across a banked road the surface rises with t: z(u) = z(t_right + u)
    const double bank = std::tan(odr::superelevationAt(r, s));
    x.elevation = {{0.0, odr::surfaceZ(r, s, rIn), bank, 0.0, 0.0}};
    // one walking lane, the crosswalk's length wide, centred on the line
    x.laneOffset = {{0.0, 0.5 * o->length, 0.0, 0.0, 0.0}};
    odr::LaneSection ls;
    ls.center.id = 0;
    odr::Lane walk;
    walk.id = -1;
    walk.type = "sidewalk";  // written as walking (1.8)
    walk.widths = {{0.0, o->length, 0.0, 0.0, 0.0}};
    walk.roadMarks = {odr::RoadMark{}};
    ls.right = {walk};
    x.lanes = {ls};
    // the virtual junction on the uncut road, holding the cross path
    odr::Junction J;
    J.id = std::to_string(++nextJunction);
    J.name = "crossing";
    J.type = "virtual";
    J.mainRoad = r.id;
    J.sStart = std::max(0.0, s - 0.5 * o->length);
    J.sEnd = std::min(r.length(), s + 0.5 * o->length);
    J.orientation = "none";
    J.crossPaths = {odr::CrossPath{"0", x.id, r.id, r.id, {walkR, s, -1}, {walkL, s, -1}}};
    od.junctions.push_back(J);
    crossing.push_back(std::move(x));
    ++rep.crossPaths;
  }
  for (auto &x : crossing) od.roads.push_back(std::move(x));
}

}  // namespace detail
}  // namespace odrforge
