// OdrForge - pedestrian crossings as cross paths (standard profile).
//
// OpenDRIVE 1.9 §12.5 cross paths (introduced 1.8.0), §12.7.1 cross paths with virtual junctions; ASAM
// Junction Guideline §10.3. Every crosswalk lies on a road, not in a junction's area: mid-block at an OSM
// crossing node, or on the approach of a signalised / crossing junction just short of its end. A junction
// of type virtual on that road holds its <crossPath> (§12.7.1: "the value of the @mainRoad attribute
// defines the crossed road and the values of the @sStart and @sEnd attributes define the section with the
// cross path"). The crossing road - a separate road of that junction (§12.5), one walking lane the
// crosswalk's length wide - runs straight across the carriageway from the centre of the right outermost
// sidewalk to the centre of the left one, linked to them by startLaneLink / endLaneLink at the crosswalk's
// s. Rules held (rule ids of 1.8.0):
//  - junctions.cross_path.lane_linkage, junctions.cross_path.start_end_contained: each end of the crossing
//    road, over its full width, lies inside the linked sidewalk lane (kInside from its borders);
//  - junctions.cross_path.within_junction_area, junctions.virtual.crossPath.cross_road_check_s_t: sStart
//    .. sEnd spans the crossing road's footprint on the main road, inside the road's s range;
//  - junctions.cross_path.only_connect_correct_type: sidewalk lanes, written as walking (1.8);
//  - junctions.cross_path.correct_junction_id: the crossing road's @junction is the virtual junction.
// A crosswalk where these cannot hold (no sidewalk on a side, the sidewalk lane changing within the
// crosswalk, a curve bending the footprint out of the sidewalk) keeps the crosswalk object only - as the
// carla profile (1.4, no cross paths) does everywhere.

#include <algorithm>
#include <cmath>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

constexpr double kInside = 0.05;  // m, least distance of the crossing road's ends from the sidewalk's borders

/// the outermost lane of a side when it is a sidewalk: its id, else 0
int sidewalkOf(const std::vector<odr::Lane> &side) {
  return !side.empty() && side.back().type == "sidewalk" ? side.back().id : 0;
}

/// does point (s, t) of road r lie inside its outermost sidewalk walk, kInside from either border?
bool insideSidewalk(const odr::Road &r, double s, double t, int walk) {
  if (s < 0.0 || s > r.length()) return false;
  const odr::LaneSection &sec = odr::sectionAt(r, s);
  if (sidewalkOf(walk > 0 ? sec.left : sec.right) != walk) return false;
  double a, b;
  return odr::laneBorders(r, s, walk, a, b) && t > std::min(a, b) + kInside && t < std::max(a, b) - kInside;
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
  for (const auto &[ri, objectId] : crosswalkRefs) {
    const odr::Road &r = od.roads[ri];
    const auto o = std::find_if(r.objects.begin(), r.objects.end(), [&](const odr::Object &x) { return x.id == objectId; });
    if (o == r.objects.end()) continue;
    const double s = o->s, h = 0.5 * o->length;
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    const int walkL = sidewalkOf(sec.left), walkR = sidewalkOf(sec.right);
    if (!walkL || !walkR) continue;  // a sidewalk on both sides to link
    // the crossing road's ends: the centre of each sidewalk at s
    double a0, a1, b0, b1;
    if (!odr::laneBorders(r, s, walkR, a0, a1) || !odr::laneBorders(r, s, walkL, b0, b1)) continue;
    const double tR = 0.5 * (a0 + a1), tL = 0.5 * (b0 + b1);
    if (tL - tR < 1.0) continue;
    const Pose p = r.planView.eval(s);
    const Vec2 endR = p.offset(tR), endL = p.offset(tL);
    // its footprint: each end, the crosswalk's length wide along the road at s, inside its sidewalk
    double sMin = r.length(), sMax = 0.0;
    bool inside = true;
    for (const auto &[end, walk] : {std::pair<Vec2, int>{endR, walkR}, {endL, walkL}})
      for (const double side : {-h, h}) {
        double sc, tc;
        r.planView.project(end + p.dir() * side, sc, tc);
        inside = inside && insideSidewalk(r, sc, tc, walk);
        sMin = std::min(sMin, sc), sMax = std::max(sMax, sc);
      }
    if (!inside) continue;
    // §12.7.1: the virtual junction on the uncut road, over the crossing road's footprint
    odr::Junction J;
    J.id = std::to_string(++nextJunction);
    J.name = "crossing";
    J.type = "virtual";
    J.mainRoad = r.id;
    J.sStart = sMin;
    J.sEnd = sMax;
    J.orientation = "none";
    // the crossing road: straight across, perpendicular to the road at s
    odr::Road x;
    x.id = std::to_string(++nextRoad);
    x.name = "Crossing";
    x.junction = J.id;
    x.rule = r.rule;
    GeomElement e;
    e.x = endR.x, e.y = endR.y, e.hdg = p.hdg + 0.5 * kPi, e.length = tL - tR;
    x.planView.elements = {e};
    x.planView.finalize();
    x.types = {odr::Road::Type{0.0, "pedestrian", "", std::nullopt, "km/h"}};
    // across a banked road the surface rises with t: z(u) = z(t_right + u) (disregarded by §12.5, written
    // for readers that place the road)
    const double bank = std::tan(odr::superelevationAt(r, s));
    x.elevation = {{0.0, odr::surfaceZ(r, s, tR), bank, 0.0, 0.0}};
    // one walking lane, the crosswalk's length wide, centred on the line
    x.laneOffset = {{0.0, h, 0.0, 0.0, 0.0}};
    odr::LaneSection ls;
    ls.center.id = 0;
    odr::Lane walk;
    walk.id = -1;
    walk.type = "sidewalk";  // written as walking (1.8)
    walk.widths = {{0.0, o->length, 0.0, 0.0, 0.0}};
    walk.roadMarks = {odr::RoadMark{}};
    ls.right = {walk};
    x.lanes = {ls};
    J.crossPaths = {odr::CrossPath{"0", x.id, r.id, r.id, {walkR, s, -1}, {walkL, s, -1}}};
    od.junctions.push_back(J);
    crossing.push_back(std::move(x));
    ++rep.crossPaths;
  }
  for (auto &x : crossing) od.roads.push_back(std::move(x));
}

}  // namespace detail
}  // namespace odrforge
