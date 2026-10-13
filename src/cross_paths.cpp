// OdrForge - pedestrian crossings as cross paths (standard profile).
//
// Crosswalks at junctions (signalised / crossing junctions): ASAM Junction Guideline 1.0.0 §6.3.3 "Add roads
// for the pedestrian crossings inside of the junction area" and §6.4.3 "Choose the road reference lines to
// stop before any cross path in the junction" - trimRoads ends such an approach kCrosswalkJunctionDepth
// earlier, so the crosswalk lies in the junction. Its crossing road (a road of the common junction) runs
// across the arm's mouth from the middle of the walking lane of the corner sidewalk on one side to that of
// the corner sidewalk on the other (OpenDRIVE 1.8.0 §12.5 Figure 85: a cross path linking walking lanes of
// two connecting roads), linked at that s to each by startLaneLink / endLaneLink; it carries the crosswalk
// marking over the carriageway, and the corner sidewalks are lowered there (curbs.cpp, loweredSpans).
//
// Mid-block crossings:
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
// A mid-block crosswalk where these cannot hold keeps the crosswalk object only - as the carla profile (1.4,
// no cross paths) does everywhere.

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

/// a crossing road from a to b: one walking lane `width` wide centred on the line (heights a -> b, which
/// OpenDRIVE 1.8.0 §12.5 disregards; written for readers that place the road)
odr::Road crossingRoad(const std::string &id, const std::string &junction, const std::string &rule, const Vec2 &a, const Vec2 &b,
                       double width, double za, double zb) {
  odr::Road x;
  x.id = id;
  x.name = "Crossing";
  x.junction = junction;
  x.rule = rule;
  GeomElement e;
  e.x = a.x, e.y = a.y, e.hdg = std::atan2(b.y - a.y, b.x - a.x), e.length = (b - a).norm();
  x.planView.elements = {e};
  x.planView.finalize();
  x.types = {odr::Road::Type{0.0, "pedestrian", "", std::nullopt, "km/h"}};
  x.elevation = {{0.0, za, (zb - za) / std::max(1e-9, e.length), 0.0, 0.0}};
  x.laneOffset = {{0.0, 0.5 * width, 0.0, 0.0, 0.0}};
  odr::LaneSection ls;
  ls.center.id = 0;
  odr::Lane walk;
  walk.id = -1;
  walk.type = "sidewalk";  // written as walking (1.8)
  walk.widths = {{0.0, width, 0.0, 0.0, 0.0}};
  walk.roadMarks = {odr::RoadMark{}};
  ls.right = {walk};
  x.lanes = {ls};
  return x;
}

long numericId(const std::string &id) {
  try {
    return std::stol(id);
  } catch (...) {
    return 0;
  }
}

}  // namespace

std::vector<std::pair<size_t, bool>> Builder::cornerSidewalksAt(const odr::Junction &J, const RoadEnd &e) const {
  std::vector<std::pair<size_t, bool>> out;
  const RoadMeta &m = metas[e.road];
  if (m.roadIndex < 0) return out;
  const std::string &armId = od.roads[m.roadIndex].id, contact = e.atEnd ? "end" : "start";
  auto linksArm = [&](const std::optional<odr::LinkTarget> &lk) {
    return lk && lk->elementType == "road" && lk->elementId == armId && lk->contactPoint == contact;
  };
  for (size_t i = 0; i < od.roads.size(); ++i) {
    const odr::Road &c = od.roads[i];
    if (c.junction != J.id || c.lanes.empty() || c.length() < kCrosswalkLength + 0.5) continue;
    bool walkOnly = c.lanes.front().left.empty() && !c.lanes.front().right.empty();
    for (const odr::Lane &l : c.lanes.front().right) walkOnly = walkOnly && (l.type == "sidewalk" || l.type == "curb");
    const bool fromArm = linksArm(c.predecessor), toArm = linksArm(c.successor);
    if (walkOnly && fromArm != toArm) out.push_back({i, fromArm});
  }
  return out;
}

void Builder::crossPaths() {
  if (!opt.target.spec().hasCrossPaths()) return;
  long nextRoad = 0, nextJunction = 0;
  for (const odr::Road &r : od.roads) nextRoad = std::max(nextRoad, numericId(r.id));
  for (const odr::Junction &j : od.junctions) nextJunction = std::max(nextJunction, numericId(j.id));
  std::vector<odr::Road> crossing;  // appended after the loop: the roads must not move while referenced
  // crosswalks inside common junctions (see the head of this file)
  const double depth = kCrosswalkJunctionDepth - kCrosswalkCentreBack;  // crosswalk centre from the arm's end
  for (const Cluster &cl : clusters) {
    if (cl.junction < 0 || od.junctions[cl.junction].type != "default") continue;
    odr::Junction &J = od.junctions[cl.junction];
    for (const RoadEnd &e : cl.ends) {
      const RoadMeta &m = metas[e.road];
      if (m.roadIndex < 0 || !(e.atEnd ? m.crosswalkInJunctionEnd : m.crosswalkInJunctionStart)) continue;
      const std::string &armId = od.roads[m.roadIndex].id;
      // the corner sidewalks of this arm's end: connecting roads of walking (and curb) lanes linked to it
      struct Corner {
        size_t road;
        double s;
        int walk;
        Vec2 at;
        double t, z;
      };
      std::vector<Corner> corners;
      for (const auto &[i, fromArm] : cornerSidewalksAt(J, e)) {
        const odr::Road &c = od.roads[i];
        const double s = fromArm ? depth : c.length() - depth;
        const int walk = sidewalkOf(odr::sectionAt(c, s).right);
        double a, b;
        if (!walk || !odr::laneBorders(c, s, walk, a, b)) continue;
        const double t = 0.5 * (a + b);
        corners.push_back({i, s, walk, c.planView.eval(s).offset(t), t, odr::surfaceZ(c, s, t)});
      }
      if (corners.size() != 2 || (corners[1].at - corners[0].at).norm() < 1.0) continue;
      const Corner &A = corners[0], &B = corners[1];
      // its footprint: each end, the crossing's width along the corner sidewalk, inside that walking lane
      // (rule junctions.cross_path.start_end_contained) - the crosswalk's 3 m, narrower where a tight corner
      // sidewalk does not take that much
      const Vec2 dir = (B.at - A.at) * (1.0 / (B.at - A.at).norm()), n{-dir.y, dir.x};
      double width = 0.0, span[2][2];  // footprint s range on A's and B's road
      for (const double w : {kCrosswalkLength, 2.5, 2.0}) {
        bool inside = true;
        span[0][0] = span[1][0] = 1e300, span[0][1] = span[1][1] = -1e300;
        for (int k = 0; k < 2; ++k) {
          const Corner &C = k ? B : A;
          const odr::Road &c = od.roads[C.road];
          for (const double side : {-0.5 * w, 0.5 * w}) {
            double sc, tc;
            c.planView.project(C.at + n * side, sc, tc);
            inside = inside && insideSidewalk(c, sc, tc, C.walk);
            span[k][0] = std::min(span[k][0], sc), span[k][1] = std::max(span[k][1], sc);
          }
        }
        if (inside) {
          width = w;
          break;
        }
      }
      if (width <= 0.0) continue;
      odr::Road x = crossingRoad(std::to_string(++nextRoad), J.id, od.roads[m.roadIndex].rule, A.at, B.at, width, A.z, B.z);
      // the crosswalk marking over the carriageway: between the corner sidewalks' inner edges (lane 0)
      const double L = x.length();
      auto innerEdge = [&](const Corner &C) {
        const odr::Road &c = od.roads[C.road];
        return (c.planView.eval(C.s).offset(odr::laneOffsetAt(c, C.s)) - A.at).dot(dir);
      };
      const double u0 = std::clamp(innerEdge(A), 0.0, L), u1 = std::clamp(innerEdge(B), 0.0, L);
      if (u1 - u0 > 0.5) {
        odr::Object o;
        o.id = std::to_string(nextObjectId++);
        o.name = "Crosswalk";
        o.type = "crosswalk";
        o.s = 0.5 * (u0 + u1);
        o.t = 0.0;
        o.length = u1 - u0;
        o.width = width;
        o.orientation = "none";
        const double hl = 0.5 * (u1 - u0), hw = 0.5 * width;
        o.outline = {{-hl, -hw, 0, 0}, {hl, -hw, 0, 0}, {hl, hw, 0, 0}, {-hl, hw, 0, 0}};
        x.objects.push_back(o);
        ++rep.objects;
      }
      // a road of the junction: on its surface like the connecting roads (the grid overrides it anyway)
      if (cl.hasPlane) connectingElevation(x, cl, std::nullopt);
      J.crossPaths.push_back(odr::CrossPath{std::to_string(J.crossPaths.size()), x.id, od.roads[A.road].id, od.roads[B.road].id,
                                            {A.walk, A.s, -1}, {B.walk, B.s, -1}});
      for (int k = 0; k < 2; ++k) loweredSpans.push_back({k ? B.road : A.road, span[k][0], span[k][1]});
      crossing.push_back(std::move(x));
      ++rep.crossPaths, ++rep.junctionCrossPaths;
    }
  }
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
