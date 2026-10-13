// OdrForge - mid-block pedestrian crossings as cross paths (standard profile).
//
// OpenDRIVE 1.8.0 / 1.9 §12.5 cross paths (introduced 1.8.0), §12.7.1 cross paths with virtual junctions;
// ASAM Junction Guideline 1.0.0 §10.3.3 (pedestrian crossing, modelling approach). A crosswalk at an OSM
// crossing node on a road that is not cut becomes a junction of type virtual on that road holding a
// <crossPath>, built by the guideline's steps:
//  - the crossing road (a separate road of the junction, §12.5) has one walking lane, the crosswalk's
//    length wide; its reference line runs straight across "in the middle of the lanes with
//    @type="walking" that will be connected" - the outermost sidewalks;
//  - mainRoad = the road for general traffic, sStart / sEnd "clearly outside of the area of crossing
//    traffic" (kClear beyond the crossing road's footprint);
//  - startLaneLink / endLaneLink link those sidewalk lanes, at the crosswalk's s, to the walking lane;
//  - <priority high=road for general traffic low=crossing road>.
// Rules held (rule ids of 1.8.0; tools/inspect/crosspath_audit.py measures them):
//  - junctions.cross_path.lane_linkage, junctions.cross_path.start_end_contained: each end of the crossing
//    road, over its full width, lies inside the linked sidewalk lane (kInside from its borders);
//  - junctions.cross_path.within_junction_area, junctions.virtual.crossPath.cross_road_check_s_t: the
//    footprint inside sStart .. sEnd, inside the road's s range;
//  - junctions.cross_path.only_connect_correct_type: sidewalk lanes, written as walking (1.8);
//  - junctions.cross_path.correct_junction_id: the crossing road's @junction is the virtual junction;
//  - §12.7 "Currently only flat virtual junctions can be modeled" (1.8.0 table 12.1: virtual "if constant
//    elevation"): the road surface over sStart .. sEnd and across the crossing varies by at most kFlat.
// A crosswalk where these cannot hold keeps the crosswalk object only - as the carla profile (1.4, no
// cross paths) does everywhere. Crosswalks at junctions are not cross paths here: Junction Guideline
// §6.3.3 / §6.4.3 puts them inside the common junction's area, the incoming roads ending before them.

#include <algorithm>
#include <cmath>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

constexpr double kInside = 0.05;  // m, least distance of the crossing road's ends from the sidewalk's borders
constexpr double kClear = 0.5;    // m, sStart / sEnd beyond the crossing road's footprint
constexpr double kFlat = 0.02;    // m, largest surface height difference of a flat virtual junction

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

/// is the road surface flat (within kFlat) over s0 .. s1 and t0 .. t1?
bool flat(const odr::Road &r, double s0, double s1, double t0, double t1) {
  constexpr int n = 6;
  double lo = 1e300, hi = -1e300;
  for (int i = 0; i <= n; ++i)
    for (int k = 0; k <= n; ++k) {
      const double z = odr::surfaceZ(r, s0 + (s1 - s0) * i / n, t0 + (t1 - t0) * k / n);
      lo = std::min(lo, z), hi = std::max(hi, z);
    }
  return hi - lo <= kFlat;
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
    const double s = o->s, h = 0.5 * o->length;
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    const int walkL = sidewalkOf(sec.left), walkR = sidewalkOf(sec.right);
    if (!walkL || !walkR) continue;  // a sidewalk on both sides to link
    // the crossing road's ends: the middle of each sidewalk at s
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
    const double sStart = std::max(0.0, sMin - kClear), sEnd = std::min(r.length(), sMax + kClear);
    if (!flat(r, sStart, sEnd, tR, tL)) continue;
    // §12.7.1: the virtual junction on the uncut road
    odr::Junction J;
    J.id = std::to_string(++nextJunction);
    J.name = "crossing";
    J.type = "virtual";
    J.mainRoad = r.id;
    J.sStart = sStart;
    J.sEnd = sEnd;
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
    // the road surface's height (disregarded by §12.5, written for readers that place the road; flat)
    x.elevation = {{0.0, odr::surfaceZ(r, s, tR), 0.0, 0.0, 0.0}};
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
    J.priorities = {{r.id, x.id}};
    J.crossPaths = {odr::CrossPath{"0", x.id, r.id, r.id, {walkR, s, -1}, {walkL, s, -1}}};
    od.junctions.push_back(J);
    crossing.push_back(std::move(x));
    ++rep.crossPaths;
  }
  for (auto &x : crossing) od.roads.push_back(std::move(x));
}

}  // namespace detail
}  // namespace odrforge
