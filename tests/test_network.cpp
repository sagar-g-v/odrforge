// End-to-end OSM -> OpenDRIVE scenarios. Every generated network must pass the full ASAM
// conformance checker (validate) without errors, plus scenario-specific spec expectations.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>

#include "odrforge/network.hpp"
#include "odrforge/validate.hpp"
#include "test.hpp"

using namespace odrforge;

namespace {

/// Small OSM builder in local metres (x east, y north) around a projection origin.
struct OsmBuilder {
  Projection proj{34.0, -118.0};
  OsmData d;
  OsmId next = 1;
  OsmId node(double x, double y, Tags t = {}) {
    OsmNode n;
    n.id = next++;
    proj.inverse({x, y}, n.lat, n.lon);
    n.tags = std::move(t);
    d.nodes[n.id] = n;
    return n.id;
  }
  OsmId way(std::vector<OsmId> nodes, Tags t) {
    OsmWay w;
    w.id = 1000 + next++;
    w.nodes = std::move(nodes);
    w.tags = std::move(t);
    d.ways[w.id] = w;
    return w.id;
  }
};

struct Result {
  odr::OpenDrive od;
  BuildReport rep;
  ValidationReport val;
};

Result build(const OsmBuilder &b, const std::string &country = "US", const std::string &target = "asam",
             bool separateSidewalks = false, bool furniture = false) {
  BuildOptions o;
  o.separateSidewalks = separateSidewalks;
  o.country = CountryProfile::load(country);
  o.target = TargetProfile::byName(target);
  o.streetFurniture = furniture;
  Result r;
  r.od = buildOpenDrive(b.d, b.proj, o, &r.rep);
  r.od.header.revMinor = o.target.revMinor;
  r.val = validate(r.od, o.target);
  // for the XSD conformance tests: every scenario is also written in both output profiles,
  // OpenDRIVE 1.8 (standard, XSD 1.8.1) and 1.4 (carla, XSD 1.4H)
  if (const char *dir = std::getenv("ODRFORGE_TEST_OUT")) {
    static int n = 0;
    const std::string name = "scenario_" + std::to_string(++n) + "_" + country + "_" + target + ".xodr";
    odr::OpenDrive copy = r.od;
    copy.header.revMinor = 8;
    odr::write(copy, std::string(dir) + "/" + name, {8, 12});
    std::filesystem::create_directories(std::string(dir) + "/rev4");
    copy.header.revMinor = 4;
    odr::write(copy, std::string(dir) + "/rev4/" + name, {4, 12});
  }
  if (r.val.errors)
    for (size_t i = 0; i < r.val.messages.size() && i < 10; ++i) std::printf("    %s\n", r.val.messages[i].c_str());
  return r;
}

// a junction road for pedestrians only: the sidewalk round a corner
/// a connecting road along a junction corner: its sidewalk, behind a curb lane in the standard profile
bool cornerSidewalk(const odr::Road &r) {
  const auto &sec = r.lanes.front();
  const auto &side = sec.left.empty() ? sec.right : sec.left;  // the right, the left under left-hand traffic
  return r.junction != "-1" && r.name != "Crossing" && sec.left.empty() != sec.right.empty() && side.back().type == "sidewalk" &&
         std::all_of(side.begin(), side.end(), [](const odr::Lane &l) { return l.type == "sidewalk" || l.type == "curb"; });
}

/// roads in a junction (connecting roads; a cross path's crossing road not counted) or outside one
int count(const odr::OpenDrive &od, bool junctionRoads) {
  std::set<std::string> crossing;
  for (const auto &j : od.junctions)
    for (const auto &cp : j.crossPaths) crossing.insert(cp.crossingRoad);
  return static_cast<int>(std::count_if(od.roads.begin(), od.roads.end(), [&](const odr::Road &r) {
    return !crossing.count(r.id) && (r.junction != "-1") == junctionRoads;
  }));
}

const odr::Road *roadById(const odr::OpenDrive &od, const std::string &id) {
  for (const auto &r : od.roads)
    if (r.id == id) return &r;
  return nullptr;
}

OsmBuilder crossroads(bool signals) {
  OsmBuilder b;
  const OsmId c = b.node(0, 0, signals ? Tags{{"highway", "traffic_signals"}} : Tags{});
  const Tags street{{"highway", "residential"}, {"lanes", "2"}};
  b.way({b.node(-120, 0), b.node(-60, 0), c}, street);
  b.way({c, b.node(60, 0), b.node(120, 0)}, street);
  b.way({b.node(0, -120), b.node(0, -60), c}, street);
  b.way({c, b.node(0, 60), b.node(0, 120)}, street);
  return b;
}

}  // namespace

TEST_CASE("corner sidewalks: the curb follows the connecting road round the corner, bending smoothly beside a straight-on") {
  // a street bending 10 degrees at a junction, a side street leaving at about 90 degrees. Junction Guideline 1.0.0 §6.3:
  // the connecting road round a corner carries the curb and walking lanes beside its driving lane, so the corner
  // sidewalk's curb is that road's outer lane border - a fixed 6 m curb return let a tighter turn run over the sidewalk
  // (Munich review flags 4 / 5). On the far side the curb runs beside the straight-on movement and is part of the
  // street's bend: a 6 m arc there would put the 10 degrees into 1 m (review flag 7, Munich)
  OsmBuilder b;
  const OsmId c = b.node(0, 0);
  const Tags street{{"highway", "residential"}, {"lanes", "2"}};
  b.way({b.node(-120, 0), b.node(-60, 0), c}, street);
  b.way({c, b.node(59.09, -10.42), b.node(118.18, -20.84)}, street);
  b.way({c, b.node(10.42, 59.09), b.node(20.84, 118.18)}, street);
  for (const char *target : {"asam", "carla"}) {
    const Result r = build(b, "US", target);
    CHECK(r.val.errors == 0);
    int straightOn = 0, corners = 0;
    for (const auto &rd : r.od.roads) {
      if (!cornerSidewalk(rd)) continue;
      const double turn = std::abs(normalizeAngle(rd.planView.eval(rd.length()).hdg - rd.planView.eval(0.0).hdg));
      double kMax = 0.0;
      for (const auto &e : rd.planView.elements) kMax = std::max({kMax, std::abs(e.k0), std::abs(e.k1)});
      if (turn < 35.0 * kPi / 180.0) {
        ++straightOn;
        CHECK(kMax < 1.0 / 20.0);                                  // no 6 m arc: the bend spreads over the curb
        CHECK(kMax * rd.length() <= 2.5 * std::max(turn, 1e-9));  // at most twice an even bend (a clothoid pair)
      } else {
        ++corners;
        // every point of the curb (the sidewalk's inner edge, lane 0) lies on a driving lane's border of a connecting
        // road of the junction (within the 0.1 m of the curb's fit)
        std::vector<Vec2> borders;
        for (const auto &q : r.od.roads) {
          if (q.junction != rd.junction || cornerSidewalk(q) || q.lanes.empty()) continue;
          for (double s = 0.0; s <= q.length(); s += 0.1)
            for (const auto *side : {&q.lanes.front().left, &q.lanes.front().right})
              for (const auto &l : *side) {
                double a, bb;
                if (l.type == "driving" && odr::laneBorders(q, s, l.id, a, bb)) borders.push_back(q.planView.eval(s).offset(bb));
              }
        }
        double worst = 0.0;
        for (double s = 0.0; s <= rd.length(); s += 0.25) {
          const Vec2 curb = rd.planView.eval(s).offset(odr::laneOffsetAt(rd, s));
          double d = 1e300;
          for (const Vec2 &p : borders) d = std::min(d, (p - curb).norm());
          worst = std::max(worst, d);
        }
        CHECK(worst < 0.15);
      }
    }
    CHECK(straightOn == 1);
    CHECK(corners == 2);
  }
}

TEST_CASE("network: signalised crossroads (US) passes every spec rule") {
  const Result r = build(crossroads(true));
  CHECK(r.val.errors == 0);
  CHECK(r.od.junctions.size() == 1);
  CHECK(count(r.od, false) == 4);
  // 4 approaches x (left, straight, right), one lane each, and the sidewalk round each of the 4 corners
  CHECK(count(r.od, true) == 16);
  CHECK(std::count_if(r.od.roads.begin(), r.od.roads.end(), cornerSidewalk) == 4);
  // spec 7.2: one controller per pair of opposite approaches, referenced by the junction
  CHECK(r.od.controllers.size() == 2);
  CHECK(r.od.junctions[0].controllers.size() == 2);
  int lights = 0;
  for (const auto &road : r.od.roads)
    for (const auto &s : road.signals)
      if (s.dynamic) {
        ++lights;
        CHECK(s.type == "1000001" && s.country == "OpenDRIVE");
        // placed on the incoming road near its junction end, valid for lanes driving into it
        const bool atEnd = road.successor && road.successor->elementType == "junction";
        CHECK(atEnd ? s.s > road.length() - 1.0 : s.s < 1.0);
        CHECK(s.orientation == (atEnd ? "+" : "-"));
        // Junction Guideline §9.2 (1.8): the light carries no lane validity; it references the stop
        // line on its approach, which holds the validity and depends on the light
        CHECK(s.validity.empty());
        CHECK(s.references.size() == 1 && s.references[0].type == "stopLine");
        if (s.references.size() != 1) continue;
        const auto stop = std::find_if(road.signals.begin(), road.signals.end(),
                                       [&](const odr::Signal &x) { return x.id == s.references[0].elementId; });
        CHECK(stop != road.signals.end());
        if (stop == road.signals.end()) continue;
        CHECK(stop->type == "1100001" && stop->country == "OpenDRIVE" && !stop->dynamic);
        CHECK(!stop->validity.empty() && stop->validity[0].fromLane <= stop->validity[0].toLane);
        CHECK(stop->dependencies.size() == 1 && stop->dependencies[0].id == s.id);
        // the crosswalk lies inside the junction (Junction Guideline section 6.4.3: the road ends before the cross
        // path): the stop line at the road's end
        CHECK(atEnd ? stop->s > road.length() - 0.5 : stop->s < 0.5);
      }
  CHECK(lights == 4);
  // every connecting road has exactly one lane linked at both ends (a corner sidewalk: its sidewalk
  // behind a curb lane, OpenDRIVE 1.9 section 11.8), and the junction lists it; a cross path's crossing
  // road is linked by the cross path's lane links instead (section 12.5)
  for (const auto &road : r.od.roads) {
    if (road.junction == "-1" || road.name == "Crossing") continue;
    const auto &sec = road.lanes.front();
    CHECK(sec.left.size() + sec.right.size() == (cornerSidewalk(road) ? 2u : 1u));
    CHECK(road.predecessor && road.successor);
  }
  CHECK(r.val.maxLinkGap < 0.02);
}

TEST_CASE("signals: lights mapped per approach - an unmapped driveway yields, a street gets a light") {
  // lights mapped on the west and east approaches only; the north street and the south driveway have none
  OsmBuilder b;
  const OsmId c = b.node(0, 0);
  const Tags light{{"highway", "traffic_signals"}};
  b.way({b.node(-120, 0), b.node(-20, 0, light), c}, {{"highway", "secondary"}, {"lanes", "2"}, {"name", "W"}});
  b.way({c, b.node(20, 0, light), b.node(120, 0)}, {{"highway", "secondary"}, {"lanes", "2"}, {"name", "E"}});
  b.way({c, b.node(0, 60), b.node(0, 120)}, {{"highway", "residential"}, {"name", "N"}});
  b.way({b.node(0, -120), b.node(0, -60), c}, {{"highway", "service"}, {"name", "S"}});
  const Result r = build(b);
  CHECK(r.val.errors == 0);
  std::map<std::string, int> lights, giveWays;
  for (const auto &road : r.od.roads)
    for (const auto &sg : road.signals) {
      if (sg.dynamic) ++lights[road.name];
      if (sg.name == "GiveWay") ++giveWays[road.name];
    }
  CHECK(lights["W"] == 1 && lights["E"] == 1 && lights["N"] == 1);
  CHECK(lights["S"] == 0 && giveWays["S"] == 1);
}

TEST_CASE("network: connecting roads use only line / arc / spiral and fit lane centres") {
  const Result r = build(crossroads(false));
  CHECK(r.val.errors == 0);
  for (const auto &road : r.od.roads)
    for (const auto &e : road.planView.elements)
      CHECK(e.type() == GeomType::Line || e.type() == GeomType::Arc || e.type() == GeomType::Spiral);
  CHECK(r.val.maxLinkHeading < 1e-3);
  CHECK(r.val.maxLinkWidthDiff < 0.02);
}

TEST_CASE("curb lanes (standard): a 0.15 m curb before each sidewalk, carried round the corners; none in carla") {
  const Result carla = build(crossroads(false), "US", "carla");
  const Result std18 = build(crossroads(false), "US", "asam");
  CHECK(std18.val.errors == 0);
  int curbs = 0, cornerCurbs = 0;
  for (const auto &road : carla.od.roads)
    for (const auto &sec : road.lanes)
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) CHECK(l.type != "curb");
  for (size_t i = 0; i < std18.od.roads.size(); ++i) {
    const odr::Road &road = std18.od.roads[i];
    for (const auto &sec : road.lanes)
      for (const auto *side : {&sec.left, &sec.right}) {
        if (side->empty() || side->back().type != "sidewalk") continue;
        // OpenDRIVE 1.9 section 11.8: the sidewalk sits behind a curb lane of kerb width and kerb height
        CHECK(side->size() >= 2);
        if (side->size() < 2) continue;
        const odr::Lane &curb = (*side)[side->size() - 2];
        CHECK(curb.type == "curb" && std::abs(odr::laneWidthAt(curb, 0.0) - 0.15) < 1e-9 && curb.heightInner);
        ++curbs;
        cornerCurbs += cornerSidewalk(road);
      }
  }
  CHECK(curbs > 0 && cornerCurbs == 4);
  // the sidewalk keeps its outer edge: the arms' total width is the same in both profiles
  for (const auto &road : std18.od.roads) {
    if (road.junction != "-1") continue;
    const odr::Road *c = nullptr;
    for (const auto &o : carla.od.roads)
      if (o.junction == "-1" && o.userData.count("osmWays") && road.userData.count("osmWays") &&
          o.userData.at("osmWays") == road.userData.at("osmWays"))
        c = &o;
    if (!c) continue;
    double a0, a1, b0, b1;
    const int outer = -static_cast<int>(road.lanes.front().right.size()), outerC = -static_cast<int>(c->lanes.front().right.size());
    CHECK(odr::laneBorders(road, 1.0, outer, a0, a1) && odr::laneBorders(*c, 1.0, outerC, b0, b1));
    CHECK(std::abs(a1 - b1) < 1e-6);
  }
}

TEST_CASE("lowered curbs: over each crosswalk the curb and the sidewalk's inner edge drop, both profiles") {
  for (const char *target : {"carla", "asam"}) {
    const Result r = build(crossroads(true), "US", target);
    CHECK(r.val.errors == 0);
    int lowered = 0;
    for (const auto &road : r.od.roads)
      for (const auto &o : road.objects) {
        // standard: the crosswalks lie inside the junction, on the cross paths' crossing roads; their corner
        // sidewalks are lowered (checked in the junction cross path test)
        if (o.type != "crosswalk" || road.name == "Crossing") continue;
        const odr::LaneSection &sec = odr::sectionAt(road, o.s);
        const double ds = o.s - sec.s;
        for (const auto *side : {&sec.left, &sec.right})
          for (const auto &l : *side) {
            if (l.type != "sidewalk" && l.type != "curb") continue;
            // the height record in force at the crosswalk centre (OpenDRIVE 1.9 section 11.8: each holds to the next)
            double inner = l.heightInner.value_or(0.0);
            for (const auto &h : l.heightChanges)
              if (h.sOffset <= ds) inner = h.inner;
            CHECK(std::abs(inner - 0.02) < 1e-9);
            ++lowered;
          }
      }
    CHECK(lowered > 0 || std::string(target) == "asam");
  }
}

TEST_CASE("cross paths (standard): none on a road that is not flat (OpenDRIVE 1.8 section 12.7: only flat virtual junctions)") {
  OsmBuilder b;
  const OsmId x = b.node(0, 0, {{"highway", "crossing"}});
  b.way({b.node(-100, 0), x, b.node(100, 0)}, {{"highway", "residential"}, {"lanes", "2"}, {"sidewalk", "both"}});
  BuildOptions o;
  o.country = CountryProfile::load("US");
  o.target = TargetProfile::byName("asam");
  o.streetFurniture = false;
  for (const double grade : {0.0, 0.06}) {
    // a 6 % grade: the road rises 0.24 m over the 4 m of the junction
    o.elevation = [grade](double x, double) { return grade * x; };
    BuildReport rep;
    odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
    od.header.revMinor = o.target.revMinor;
    CHECK(validate(od, o.target).errors == 0);
    CHECK(rep.crossPaths == (grade == 0.0 ? 1 : 0));
  }
}

TEST_CASE("junction grid (standard): reference line and elevation grid reproduce the connecting roads' heights") {
  // OpenDRIVE 1.8.0 §12.9 / §12.11: one straight reference line reaching every junction point by a perpendicular,
  // a grid of complete squares along it; the grid overrides the connecting roads' elevation, so it must equal it
  BuildOptions o;
  o.country = CountryProfile::load("US");
  o.target = TargetProfile::byName("standard");
  o.streetFurniture = false;
  o.elevation = [](double x, double y) { return 0.04 * x + 0.02 * y; };  // a sloped crossroads
  const OsmBuilder b = crossroads(true);
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionGrids == 1);
  const odr::Junction &J = od.junctions[0];
  CHECK(J.grid.has_value());
  if (!J.grid) return;
  const odr::JunctionGrid &g = *J.grid;
  const Vec2 d{std::cos(g.hdg), std::sin(g.hdg)}, n{-std::sin(g.hdg), std::cos(g.hdg)};
  // bilinear in the square (exact for the bicubic interpolation of planar values)
  auto at = [&](int k, int j) {
    const auto &row = g.rows[k];
    return j == 0 ? row.center : j > 0 ? row.left[j - 1] : row.right[-j - 1];
  };
  for (const auto &r : od.roads) {
    if (r.junction != J.id) continue;
    for (double s = 0.0; s <= r.length(); s += 0.5) {
      const Pose p = r.planView.eval(s);
      const double t = odr::laneOffsetAt(r, s);
      const Vec2 q = p.offset(t) - Vec2{g.x, g.y};
      const double ps = q.dot(d), pt = q.dot(n);
      CHECK(ps >= 0.0 && ps <= g.length);  // rule junctions.geometry.ref_line_definition
      const int k = static_cast<int>(std::floor(ps / g.gridSpacing)), j = static_cast<int>(std::floor(pt / g.gridSpacing));
      CHECK(k >= 0 && k + 1 < static_cast<int>(g.rows.size()));
      if (k < 0 || k + 1 >= static_cast<int>(g.rows.size())) return;
      const int nl = static_cast<int>(g.rows[k].left.size()), nr = static_cast<int>(g.rows[k].right.size());
      CHECK(j >= -nr && j + 1 <= nl);
      if (j < -nr || j + 1 > nl) return;
      const double u = ps / g.gridSpacing - k, v = pt / g.gridSpacing - j;
      const double zg = at(k, j) * (1 - u) * (1 - v) + at(k + 1, j) * u * (1 - v) + at(k, j + 1) * (1 - u) * v + at(k + 1, j + 1) * u * v;
      CHECK_NEAR(zg, odr::surfaceZ(r, s, t), 0.01);
    }
  }
  // carla (1.4): no junction grid
  o.target = TargetProfile::byName("carla");
  CHECK(buildOpenDrive(b.d, b.proj, o, &rep).junctions[0].grid.has_value() == false);
}

// OpenDRIVE 1.8.0 §12.10 on every written junction boundary; returns how many were checked. Rules: segments reach the
// start or end of all roads connected to the junction (a joint per plain road end), close the boundary (each segment
// starts within 0.10 m of where the last ends), counter clockwise; the boundary encloses the area intended for traffic
// (both borders and the centre of every lane of the junction's roads inside or within 0.10 m), as a simple polygon;
// additional roads carry the junction's id, have no lanes besides the centre lane, are linked to roads and start and
// end pointing from those roads into the junction.
int checkBoundaries(const odr::OpenDrive &od) {
  constexpr double kTol = 0.10;
  std::map<std::string, const odr::Road *> byId;
  for (const auto &r : od.roads) byId[r.id] = &r;
  int n = 0;
  for (const auto &J : od.junctions) {
    if (J.boundary.empty()) continue;
    ++n;
    std::set<std::pair<std::string, std::string>> joints;
    for (const auto &sg : J.boundary)
      if (sg.type == "joint") joints.insert({sg.roadId, sg.contactPoint});
    for (const auto &r : od.roads) {
      if (r.junction != "-1") continue;
      if (r.predecessor && r.predecessor->elementType == "junction" && r.predecessor->elementId == J.id) CHECK(joints.count({r.id, "start"}) == 1);
      if (r.successor && r.successor->elementType == "junction" && r.successor->elementId == J.id) CHECK(joints.count({r.id, "end"}) == 1);
    }
    // the outline from the first lane segment on; a joint runs from the end nearer the outline so far
    std::vector<odr::BoundarySegment> segs = J.boundary;
    std::rotate(segs.begin(), std::find_if(segs.begin(), segs.end(), [](const auto &sg) { return sg.type == "lane"; }), segs.end());
    std::vector<Vec2> P;
    for (const auto &sg : segs) {
      CHECK(byId.count(sg.roadId) == 1);
      if (!byId.count(sg.roadId)) return n;
      const odr::Road &r = *byId[sg.roadId];
      std::vector<Vec2> pts;
      if (sg.type == "joint") {  // across all lanes, or jointLaneStart .. jointLaneEnd (1.8.0 Table 78)
        const double s = sg.contactPoint == "start" ? 0.0 : r.length();
        const int idLo = std::min(sg.jointLaneStart, sg.jointLaneEnd), idHi = std::max(sg.jointLaneStart, sg.jointLaneEnd);
        double lo = 1e9, hi = -1e9;
        for (const auto *side : {&odr::sectionAt(r, s).left, &odr::sectionAt(r, s).right})
          for (const odr::Lane &l : *side) {
            double x, y;
            if (sg.jointLanes && (l.id < idLo || l.id > idHi)) continue;
            if (odr::laneBorders(r, s, l.id, x, y)) lo = std::min({lo, x, y}), hi = std::max({hi, x, y});
          }
        CHECK(lo < hi);
        pts = {r.planView.eval(s).offset(lo), r.planView.eval(s).offset(hi)};
        if (!P.empty() && (pts[1] - P.back()).norm() < (pts[0] - P.back()).norm()) std::swap(pts[0], pts[1]);
      } else {  // sStart / sEnd: "start", "end" or a number
        const auto sOf = [&](const std::string &v) { return v == "start" ? 0.0 : v == "end" ? r.length() : std::stod(v); };
        const double s0 = sOf(sg.sStart), s1 = sOf(sg.sEnd);
        CHECK(std::abs(s1 - s0) > 1e-3);  // no lane segment of zero length
        const int m = std::max(2, static_cast<int>(std::ceil(std::abs(s1 - s0) / 0.25)) + 1);
        for (int i = 0; i < m; ++i) {
          const double s = s0 + (s1 - s0) * i / (m - 1);
          double in = 0.0, t = odr::laneOffsetAt(r, s);
          if (sg.boundaryLane != 0) odr::laneBorders(r, s, sg.boundaryLane, in, t);
          pts.push_back(r.planView.eval(s).offset(t));
        }
      }
      if (!P.empty()) {
        CHECK((pts.front() - P.back()).norm() < kTol);
        P.pop_back();  // the join is one corner
      }
      P.insert(P.end(), pts.begin(), pts.end());
    }
    CHECK((P.front() - P.back()).norm() < kTol);
    P.pop_back();
    double area = 0.0;
    for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) area += P[j].x * P[i].y - P[i].x * P[j].y;
    CHECK(area > 0.0);
    const auto cr = [](const Vec2 &o, const Vec2 &u, const Vec2 &v) { return (u.x - o.x) * (v.y - o.y) - (u.y - o.y) * (v.x - o.x); };
    int crossings = 0;
    for (size_t i = 0; i < P.size(); ++i)
      for (size_t k = i + 2; k < P.size(); ++k) {
        if (i == 0 && k == P.size() - 1) continue;
        const Vec2 &a1 = P[i], &a2 = P[i + 1], &b1 = P[k], &b2 = P[(k + 1) % P.size()];
        crossings += cr(a1, a2, b1) * cr(a1, a2, b2) < 0.0 && cr(b1, b2, a1) * cr(b1, b2, a2) < 0.0;
      }
    CHECK(crossings == 0);
    const auto encloses = [&](const Vec2 &q) {
      bool in = false;
      double d = 1e300;
      for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) {
        if ((P[i].y > q.y) != (P[j].y > q.y) && q.x < P[i].x + (q.y - P[i].y) * (P[j].x - P[i].x) / (P[j].y - P[i].y)) in = !in;
        const Vec2 e = P[i] - P[j];
        const double u = std::clamp((q - P[j]).dot(e) / std::max(e.dot(e), 1e-12), 0.0, 1.0);
        d = std::min(d, (P[j] + e * u - q).norm());
      }
      return in || d <= kTol;
    };
    int outside = 0, points = 0;
    for (const auto &r : od.roads) {
      if (r.junction != J.id || r.name == "JunctionBoundary") continue;
      for (double s = 0.0; s <= r.length(); s += 0.5)
        for (const auto *side : {&odr::sectionAt(r, s).left, &odr::sectionAt(r, s).right})
          for (const odr::Lane &l : *side) {
            double x, y;
            if (!odr::laneBorders(r, s, l.id, x, y)) continue;
            for (const double t : {x, 0.5 * (x + y), y}) outside += !encloses(r.planView.eval(s).offset(t)), ++points;
          }
    }
    CHECK(outside == 0);
    CHECK(points > 0);
    for (const auto &sg : J.boundary) {
      if (sg.type != "lane" || !byId.count(sg.roadId)) continue;
      const odr::Road &h = *byId[sg.roadId];
      CHECK(h.junction == J.id);  // a road of this junction
      // the centre lane's line of a helper road (an additional road, 1.8.0 §12.10), or of a junction road with lanes on
      // one side only (a traced outline)
      if (sg.boundaryLane != 0 || h.name != "JunctionBoundary") continue;
      CHECK(h.lanes.size() == 1 && h.lanes[0].left.empty() && h.lanes[0].right.empty());
      for (const bool atEnd : {false, true}) {
        const auto &lk = atEnd ? h.successor : h.predecessor;
        CHECK(lk && lk->elementType == "road" && byId.count(lk->elementId) == 1);
        if (!lk || !byId.count(lk->elementId)) continue;
        const odr::Road &o = *byId[lk->elementId];
        const Vec2 into = o.planView.eval(lk->contactPoint == "start" ? 0.0 : o.length()).dir() * (lk->contactPoint == "end" ? 1.0 : -1.0);
        const Vec2 dir = h.planView.eval(atEnd ? h.length() : 0.0).dir() * (atEnd ? -1.0 : 1.0);
        CHECK(dir.dot(into) > 0.0);  // from the linked road into the junction
      }
    }
  }
  return n;
}

BuildOptions boundaryOptions(const char *country) {
  BuildOptions o;
  o.country = CountryProfile::load(country);
  o.target = TargetProfile::byName("standard");
  o.streetFurniture = false;
  return o;
}

/// a junction of `angles` (degrees) arms of two-lane residential streets meeting at the origin
OsmBuilder star(const std::vector<double> &angles, Tags centre = {}) {
  OsmBuilder b;
  const OsmId c = b.node(0, 0, centre);
  for (const double a : angles) {
    const double r = a * kPi / 180.0;
    b.way({c, b.node(60 * std::cos(r), 60 * std::sin(r)), b.node(120 * std::cos(r), 120 * std::sin(r))},
          {{"highway", "residential"}, {"lanes", "2"}});
  }
  return b;
}

/// a T junction whose stem has no sidewalks: no corner sidewalk joins the stem's edges to the street's, so two corners
/// are closed by helper roads
OsmBuilder teeMixedSidewalks() {
  OsmBuilder b;
  const OsmId c = b.node(0, 0);
  b.way({c, b.node(60, 0), b.node(120, 0)}, {{"highway", "residential"}, {"lanes", "2"}, {"sidewalk", "both"}});
  b.way({c, b.node(-60, 0), b.node(-120, 0)}, {{"highway", "residential"}, {"lanes", "2"}, {"sidewalk", "both"}});
  b.way({c, b.node(0, 60), b.node(0, 120)}, {{"highway", "residential"}, {"lanes", "2"}, {"sidewalk", "no"}});
  return b;
}

TEST_CASE("junction boundary (standard): crossroads closed, counter-clockwise, a joint at every arm, around every lane") {
  BuildOptions o = boundaryOptions("US");
  const OsmBuilder b = crossroads(false);
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionBoundaries == 1 && rep.boundariesLeftOut == 0);
  CHECK(checkBoundaries(od) == 1);
  // carla (1.4): no boundary
  o.target = TargetProfile::byName("carla");
  CHECK(buildOpenDrive(b.d, b.proj, o, &rep).junctions[0].boundary.empty());
}

TEST_CASE("junction boundary (standard): T, skewed Y and 5-way junctions, right- and left-hand traffic") {
  const std::vector<std::vector<double>> shapes = {{0, 90, 180}, {0, 150, 235}, {0, 72, 144, 216, 288}, {10, 95, 200, 290}};
  for (const char *country : {"US", "GB"})
    for (const auto &angles : shapes) {
      const BuildOptions o = boundaryOptions(country);
      const OsmBuilder b = star(angles);
      BuildReport rep;
      const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
      CHECK(validate(od, o.target).errors == 0);
      CHECK(rep.junctionBoundaries + rep.boundariesLeftOut == rep.junctionGrids);
      CHECK(rep.junctionBoundaries == 1);
      CHECK(checkBoundaries(od) == rep.junctionBoundaries);
    }
}

TEST_CASE("junction boundary (standard): a signalised junction's crossings lie inside its boundary") {
  // Junction Guideline 1.0.0 section 6.3.3: crossings at a junction are roads inside the junction area
  const BuildOptions o = boundaryOptions("US");
  const OsmBuilder b = crossroads(true);
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionCrossPaths == 4);
  CHECK(rep.junctionBoundaries == 1);
  CHECK(checkBoundaries(od) == 1);  // the crossing roads are junction roads: their lanes are checked too
}

TEST_CASE("junction boundary (standard): helper roads close the corners no junction road runs along") {
  // 1.8.0 §12.10 "If the existing roads are not sufficient to define a closed junction boundary, additional roads shall
  // be defined for the missing segments"
  for (const char *country : {"US", "GB"}) {
    const BuildOptions o = boundaryOptions(country);
    BuildReport rep;
    const odr::OpenDrive od = buildOpenDrive(teeMixedSidewalks().d, teeMixedSidewalks().proj, o, &rep);
    CHECK(validate(od, o.target).errors == 0);
    CHECK(rep.junctionBoundaries == 1 && rep.boundaryHelperRoads == 2);
    CHECK(checkBoundaries(od) == 1);
  }
}

TEST_CASE("junction boundary (standard): helper roads lie on the junction surface of a sloped junction") {
  // 1.8.0 §12.11: the elevation grid holds inside the boundary; the helpers copy the planar connecting roads' heights
  BuildOptions o = boundaryOptions("US");
  o.elevation = [](double x, double y) { return 0.04 * x + 0.02 * y; };
  const OsmBuilder b = teeMixedSidewalks();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(checkBoundaries(od) == 1);
  int helpers = 0;
  for (const auto &r : od.roads) {
    if (r.name != "JunctionBoundary") continue;
    ++helpers;
    for (double s = 0.0; s <= r.length(); s += 0.5) {
      const double t = odr::laneOffsetAt(r, s);
      const Vec2 q = r.planView.eval(s).offset(t);
      CHECK_NEAR(odr::surfaceZ(r, s, t), 0.04 * q.x + 0.02 * q.y, 0.02);
    }
  }
  CHECK(helpers > 0);
}

TEST_CASE("junction boundary (standard): where the arm ends overlap, the boundary follows the lanes' contour") {
  // arms 30 degrees apart: the second arm's outer corner lies beyond the first arm's end line, no corner joins them inside
  // the junction, so no outline of joints and corners exists; the outer contour of the junction's lanes does (lane
  // segments over s ranges, joints across the lanes there - 1.8.0 §12.10, Table 78), with no additional road
  const BuildOptions o = boundaryOptions("US");
  const OsmBuilder b = star({0, 30, 180});
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionGrids == 1 && rep.junctionBoundaries == 1 && rep.boundariesTraced == 1 && rep.boundariesLeftOut == 0);
  CHECK(checkBoundaries(od) == 1);
  CHECK(std::none_of(od.roads.begin(), od.roads.end(), [](const odr::Road &r) { return r.name == "JunctionBoundary"; }));
}

/// a divided road whose two one-way carriageways (3.5 m apart) close in on a T junction with a street: two arms side
/// by side, pointing the same way out
OsmBuilder dividedTee(double gap) {
  OsmBuilder b;
  const OsmId c = b.node(0, 0);
  b.way({b.node(-120, -gap), b.node(-30, -gap), c}, {{"highway", "secondary"}, {"oneway", "yes"}, {"lanes", "2"}});
  b.way({c, b.node(-30, gap), b.node(-120, gap)}, {{"highway", "secondary"}, {"oneway", "yes"}, {"lanes", "2"}});
  b.way({c, b.node(0, 60), b.node(0, 120)}, {{"highway", "residential"}, {"lanes", "2"}});
  b.way({c, b.node(0, -60), b.node(0, -120)}, {{"highway", "residential"}, {"lanes", "2"}});
  return b;
}

TEST_CASE("junction boundary (standard): a divided road's carriageways side by side get a boundary along the lanes") {
  // the corner between the two carriageways' arms would be a curve turning back across their ends: the boundary follows
  // the lanes' contour - lane segments over s ranges and joints across only the lanes the junction's roads continue
  // (jointLaneStart / jointLaneEnd, 1.8.0 Table 78), no additional road; written as such in the XML
  const BuildOptions o = boundaryOptions("US");
  BuildReport rep;
  const OsmBuilder b = dividedTee(3.5);
  odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionBoundaries == 1 && rep.boundariesTraced == 1 && rep.boundariesLeftOut == 0 && rep.boundaryHelperRoads == 0);
  CHECK(checkBoundaries(od) == 1);
  int partial = 0, numeric = 0;
  for (const auto &J : od.junctions)
    for (const auto &sg : J.boundary) {
      partial += sg.jointLanes;
      numeric += sg.type == "lane" && (sg.sStart != "start" && sg.sStart != "end" || sg.sEnd != "start" && sg.sEnd != "end");
    }
  CHECK(partial >= 1 && numeric >= 1);
  const std::string path = (std::filesystem::temp_directory_path() / "odrforge_divided_tee.xodr").string();
  odr::write(od, path, {8, 12});
  std::ifstream in(path);
  const std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  in.close();
  std::filesystem::remove(path);
  CHECK(xml.find("jointLaneStart=") != std::string::npos && xml.find("jointLaneEnd=") != std::string::npos);
  // carriageways closer than this make no boundary yet (a joint end lands off the arm's lane borders): counted, not written
  BuildReport near;
  const OsmBuilder bn = dividedTee(1.0);
  const odr::OpenDrive odn = buildOpenDrive(bn.d, bn.proj, o, &near);
  CHECK(near.junctionBoundaries == 0 && near.boundariesLeftOut == 1 && checkBoundaries(odn) == 0);
}

/// a skewed junction from the synthetic fuzzer (tools/inspect/fuzz.py seed 7, case 0277)
OsmBuilder skewFuzz0277() {
  OsmBuilder b;
  const OsmId c = b.node(0, 0);
  b.way({b.node(-103.76, -60.17), b.node(-69.18, -40.11), b.node(-34.59, -20.06), c, b.node(28.31, 16.42), b.node(56.63, 32.84),
         b.node(84.94, 49.25)},
        {{"highway", "secondary"}, {"lanes", "2"}, {"lanes:backward", "1"}, {"lanes:forward", "1"}});
  b.way({c, b.node(34.45, -13.07), b.node(68.9, -26.15), b.node(103.36, -39.22), b.node(137.81, -52.3)},
        {{"highway", "primary"}, {"lanes", "2"}, {"oneway", "yes"}});
  b.way({c, b.node(-35.23, 13.37), b.node(-70.45, 26.74), b.node(-105.68, 40.1)}, {{"highway", "residential"}, {"lanes", "5"}});
  return b;
}

TEST_CASE("junction boundary (standard): a helper road is pushed out past lanes that swing beyond it") {
  // a connecting road turning tighter than the corner's curb return swings beyond the corner sidewalk; the boundary
  // takes a helper along that sidewalk and pushes it out past the lanes (laneOffset records on a 1 m grid), so it
  // encloses every lane. The corner sidewalk built as a curb return (cornerCurbAlongTurns off), as at fallback corners
  const OsmBuilder b = skewFuzz0277();
  BuildOptions o = boundaryOptions("US");
  o.cornerCurbAlongTurns = false;
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionBoundaries == 1);
  CHECK(checkBoundaries(od) == 1);
  size_t records = 0;
  for (const auto &r : od.roads)
    if (r.name == "JunctionBoundary") records = std::max(records, r.laneOffset.size());
  CHECK(records > 4);  // the pushed stretch: one record per metre
}

TEST_CASE("junction boundary (standard): a corner sidewalk along the turning lane needs no helper pushed out past it") {
  // with sidewalks the same junction's corner sidewalk follows the edge of the connecting road turning there (Junction
  // Guideline 1.0.0 §6.3): no lane swings beyond it. Built as a curb return, it had lanes beyond it, and the boundary
  // needed a helper along it pushed out (32 laneOffset records)
  const OsmBuilder b = skewFuzz0277();
  const BuildOptions o = boundaryOptions("US");
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.junctionBoundaries == 1);
  CHECK(checkBoundaries(od) == 1);
  size_t records = 0;
  for (const auto &r : od.roads)
    if (r.name == "JunctionBoundary") records = std::max(records, r.laneOffset.size());
  CHECK(records <= 1);
}

TEST_CASE("junction boundary (standard): a helper copying a connecting road follows its side edge exactly") {
  // the helper's laneOffset is the connecting road's outer side edge plus one level cubic shift to the arm edges, so
  // laneOffset - edge is a single cubic over the whole helper (records are exact, not sampled); helpers traversed from
  // end to start (sStart="end") occur and pass the boundary checker
  int copies = 0, reversed = 0;
  for (const OsmBuilder &b : {teeMixedSidewalks(), skewFuzz0277(), star({0, 72, 144, 216, 288}), star({10, 95, 200, 290})}) {
    BuildReport rep;
    const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, boundaryOptions("US"), &rep);
    CHECK(checkBoundaries(od) == rep.junctionBoundaries);
    for (const auto &J : od.junctions)
      for (const auto &sg : J.boundary) reversed += sg.type == "lane" && sg.sStart == "end";
    for (const auto &h : od.roads) {
      if (h.name != "JunctionBoundary" || h.laneOffset.size() > 4) continue;  // pushed helpers: their records follow the push
      for (const auto &r : od.roads) {
        if (&r == &h || r.junction != h.junction || r.lanes.empty() || r.planView.elements.size() != h.planView.elements.size()) continue;
        const auto &g = r.planView.elements.front(), &k = h.planView.elements.front();
        if (std::abs(g.x - k.x) > 1e-9 || std::abs(g.y - k.y) > 1e-9 || std::abs(g.length - k.length) > 1e-9) continue;
        // the copied road: laneOffset - its outer side edge must be one cubic on [0, L] (least squares, 41 samples)
        double best = 1e9;
        for (const int side : {1, -1}) {
          std::vector<double> ss, dv;
          for (int i = 0; i <= 40; ++i) {
            const double sv = r.length() * i / 40.0;
            const auto &sec = odr::sectionAt(r, sv);
            double edge = odr::laneOffsetAt(r, sv);
            for (const odr::Lane &l : side > 0 ? sec.left : sec.right) {
              double in, out;
              if (odr::laneBorders(r, sv, l.id, in, out)) edge = side > 0 ? std::max(edge, out) : std::min(edge, out);
            }
            ss.push_back(sv / r.length()), dv.push_back(odr::laneOffsetAt(h, sv) - edge);
          }
          // cubic fit by normal equations (4 x 4, Gaussian elimination)
          double A[4][5] = {};
          for (size_t i = 0; i < ss.size(); ++i) {
            const double pw[4] = {1, ss[i], ss[i] * ss[i], ss[i] * ss[i] * ss[i]};
            for (int a = 0; a < 4; ++a) {
              for (int c = 0; c < 4; ++c) A[a][c] += pw[a] * pw[c];
              A[a][4] += pw[a] * dv[i];
            }
          }
          for (int a = 0; a < 4; ++a)
            for (int c = a + 1; c < 4; ++c) {
              const double f = A[c][a] / A[a][a];
              for (int e = a; e < 5; ++e) A[c][e] -= f * A[a][e];
            }
          double co[4];
          for (int a = 3; a >= 0; --a) {
            co[a] = A[a][4];
            for (int c = a + 1; c < 4; ++c) co[a] -= A[a][c] * co[c];
            co[a] /= A[a][a];
          }
          double worst = 0.0;
          for (size_t i = 0; i < ss.size(); ++i)
            worst = std::max(worst, std::abs(co[0] + ss[i] * (co[1] + ss[i] * (co[2] + ss[i] * co[3])) - dv[i]));
          best = std::min(best, worst);
        }
        CHECK(best < 1e-6);
        ++copies;
        break;
      }
    }
  }
  CHECK(copies > 0);
  CHECK(reversed > 0);
}

TEST_CASE("junction boundary (standard): written XML in XSD 1.8.1 order with start / end contact values") {
  // XSD 1.8.1 t_junction: <planView>, <boundary>, <elevationGrid> in this order; t_grEqZeroOrContactPoint: "start" / "end"
  // (the spec examples' "begin" is not in e_contactPoint); joints carry contactPoint, lane segments boundaryLane
  BuildReport rep;
  odr::OpenDrive od = buildOpenDrive(teeMixedSidewalks().d, teeMixedSidewalks().proj, boundaryOptions("US"), &rep);
  od.header.revMinor = 8;
  const std::string path = (std::filesystem::temp_directory_path() / "odrforge_boundary_test.xodr").string();
  odr::write(od, path, {8, 12});
  std::ifstream in(path);
  const std::string xml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  in.close();
  std::filesystem::remove(path);
  const size_t j = xml.find("<junction "), pv = xml.find("<planView>", j), bd = xml.find("<boundary>", j),
               eg = xml.find("<elevationGrid", j);
  CHECK(j != std::string::npos && pv != std::string::npos && bd != std::string::npos && eg != std::string::npos);
  CHECK(pv < bd && bd < eg);
  CHECK(xml.find("\"begin\"") == std::string::npos);
  int lanes = 0, joints = 0;
  for (size_t at = xml.find("<segment ", bd); at != std::string::npos && at < eg; at = xml.find("<segment ", at + 1)) {
    const std::string seg = xml.substr(at, xml.find("/>", at) - at);
    if (seg.find("type=\"lane\"") != std::string::npos) {
      ++lanes;
      CHECK(seg.find("boundaryLane=\"") != std::string::npos);
      CHECK(seg.find("sStart=\"start\"") != std::string::npos || seg.find("sStart=\"end\"") != std::string::npos);
      CHECK(seg.find("sEnd=\"start\"") != std::string::npos || seg.find("sEnd=\"end\"") != std::string::npos);
    } else {
      ++joints;
      CHECK(seg.find("type=\"joint\"") != std::string::npos);
      CHECK(seg.find("contactPoint=\"start\"") != std::string::npos || seg.find("contactPoint=\"end\"") != std::string::npos);
    }
  }
  CHECK(joints == 3 && lanes == 3);
  // the 1.4 writer (carla) has no boundary
  odr::write(od, path, {4, 12});
  std::ifstream in4(path);
  const std::string xml4((std::istreambuf_iterator<char>(in4)), std::istreambuf_iterator<char>());
  in4.close();
  std::filesystem::remove(path);
  CHECK(xml4.find("<boundary>") == std::string::npos);
}

TEST_CASE("junction boundary (standard): lane-less helper roads pass the validator only as boundary roads") {
  // Annex F road.lane.lane_section_drivable (1.9.0, should) asks every lane section of a drivable road for a left or right
  // lane; 1.8.0 §12.10 additional roads "are not required to have any lanes" (1.9.0 lane_sect_first: boundary roads "do
  // not need an extra lane"). The exemption holds only for roads a boundary runs along.
  const BuildOptions o = boundaryOptions("US");
  BuildReport rep;
  odr::OpenDrive od = buildOpenDrive(teeMixedSidewalks().d, teeMixedSidewalks().proj, o, &rep);
  CHECK(rep.boundaryHelperRoads == 2);
  CHECK(validate(od, o.target).count("road.lane.lane_section_drivable") == 0);
  for (auto &J : od.junctions) J.boundary.clear();
  CHECK(validate(od, o.target).count("road.lane.lane_section_drivable") == 2);
}

TEST_CASE("junction boundary (standard): no helper road shorter than 1 m") {
  // arms 45 degrees apart leave a 0.45 m gap at their acute corner; a helper there would be shorter than 1.0 m (our
  // floor: invariant SIM-5, esmini / CARLA mishandle shorter roads; OpenDRIVE only requires length > 0) -> the boundary
  // follows the lanes' contour instead (no additional road). 60 degrees apart: joints and a corner helper.
  for (const double a : {45.0, 60.0}) {
    BuildReport rep;
    const OsmBuilder b = star({0, a, 180});
    const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, boundaryOptions("US"), &rep);
    CHECK(rep.junctionBoundaries == 1);
    CHECK(rep.boundariesTraced == (a == 45.0 ? 1 : 0));
    CHECK(checkBoundaries(od) == rep.junctionBoundaries);
    for (const auto &r : od.roads)
      if (r.name == "JunctionBoundary") CHECK(r.length() >= 1.0);
  }
}

/// a signalised crossroads whose four arms carry an OSM crossing node 6 m from the centre, with the given crossing tags
OsmBuilder crossroadsWithCrossings(const Tags &crossing) {
  OsmBuilder b;
  const OsmId c = b.node(0, 0, {{"highway", "traffic_signals"}});
  const Tags street{{"highway", "residential"}, {"lanes", "2"}};
  b.way({b.node(-120, 0), b.node(-60, 0), b.node(-6, 0, crossing), c}, street);
  b.way({c, b.node(6, 0, crossing), b.node(60, 0), b.node(120, 0)}, street);
  b.way({b.node(0, -120), b.node(0, -60), b.node(0, -6, crossing), c}, street);
  b.way({c, b.node(0, 6, crossing), b.node(0, 60), b.node(0, 120)}, street);
  return b;
}

TEST_CASE("pedestrian lights (standard): a crossing OSM signalises gets two lights, two waiting lines, one controller") {
  // Junction Guideline 1.0.0 §6.3.3 Codes 20 - 23; Signal reference 1.0.0: 1000002 pedestrian light, 1100001 invisible
  // stop line; OpenDRIVE 1.8.0 §14.2 (no <validity>: the signal holds for all lanes in its orientation), §14.3 / §14.4
  // dependency and reference, §14.6 controllers, §11.2 the walking lane used both ways (@direction both)
  const Result r = build(crossroadsWithCrossings({{"highway", "crossing"}, {"crossing", "traffic_signals"}}), "US", "asam");
  CHECK(r.val.errors == 0);
  CHECK(r.rep.junctionCrossPaths == 4 && r.rep.pedestrianSignals == 4);
  if (r.od.junctions.empty()) return;
  const odr::Junction &J = r.od.junctions[0];
  std::map<std::string, const odr::Signal *> byId;
  for (const auto &road : r.od.roads)
    for (const auto &sg : road.signals) byId[sg.id] = &sg;
  std::set<std::string> listed;
  for (const auto &c : J.controllers) listed.insert(c.id);
  int lit = 0;
  for (const auto &cp : J.crossPaths) {
    const odr::Road *x = roadById(r.od, cp.crossingRoad);
    CHECK(x != nullptr);
    if (!x) return;
    CHECK(x->lanes.front().right.size() == 1 && x->lanes.front().right.front().direction == "both");
    std::vector<const odr::Signal *> lights;
    for (const auto &sg : x->signals)
      if (sg.type == "1000002") lights.push_back(&sg);
    if (lights.empty()) continue;
    ++lit;
    CHECK(lights.size() == 2 && lights[0]->orientation != lights[1]->orientation);
    for (const odr::Signal *l : lights) {
      CHECK(l->dynamic && l->country == "OpenDRIVE" && l->semanticsPriority == "trafficLight");
      CHECK(l->references.size() == 1);
      if (l->references.empty() || !byId.count(l->references[0].elementId)) continue;
      const odr::Signal *w = byId[l->references[0].elementId];
      CHECK(w->type == "1100001" && w->orientation == l->orientation && w->semanticsPriority == "waitingLine" && w->validity.empty());
      CHECK(w->dependencies.size() == 1 && w->dependencies[0].id == l->id);
      // walkers wait at the kerb they start from: + in the first half of the crossing, - in the second; the light
      // stands across, at the far end
      const double L = x->length();
      CHECK(l->orientation == "+" ? (w->s < 0.5 * L && l->s > 0.5 * L) : (w->s > 0.5 * L && l->s < 0.5 * L));
    }
    const auto ctrl = std::find_if(r.od.controllers.begin(), r.od.controllers.end(), [&](const odr::Controller &c) {
      return std::find(c.signalIds.begin(), c.signalIds.end(), lights[0]->id) != c.signalIds.end();
    });
    CHECK(ctrl != r.od.controllers.end());
    if (ctrl == r.od.controllers.end()) continue;
    CHECK(ctrl->signalIds.size() == 2 && std::find(ctrl->signalIds.begin(), ctrl->signalIds.end(), lights[1]->id) != ctrl->signalIds.end());
    CHECK(listed.count(ctrl->id) == 1);
  }
  CHECK(lit == 4);
}

TEST_CASE("pedestrian lights (standard): none where OSM does not signalise the crossing, none in carla") {
  const Result marked = build(crossroadsWithCrossings({{"highway", "crossing"}, {"crossing", "marked"}}), "US", "asam");
  CHECK(marked.val.errors == 0);
  CHECK(marked.rep.junctionCrossPaths == 4 && marked.rep.pedestrianSignals == 0);
  const Result carla = build(crossroadsWithCrossings({{"highway", "crossing"}, {"crossing", "traffic_signals"}}), "US", "carla");
  CHECK(carla.rep.pedestrianSignals == 0);
  for (const auto &road : carla.od.roads)
    for (const auto &sg : road.signals) CHECK(sg.type != "1000002");
}

/// a 200 m two-lane road (no sidewalks) along x with a guard rail 5.5 m to its right (bent out to 6 m half way)
/// and a jersey barrier 5.5 m to its left tagged layer=1; railY moves the guard rail
OsmBuilder roadWithBarriers(double railY = -5.5) {
  OsmBuilder b;
  b.way({b.node(0, 0), b.node(200, 0)}, {{"highway", "primary"}, {"lanes", "2"}, {"sidewalk", "no"}});
  b.way({b.node(20, railY), b.node(100, railY - 0.5), b.node(180, railY)}, {{"barrier", "guard_rail"}});
  b.way({b.node(30, 5.5), b.node(170, 5.5)}, {{"barrier", "jersey_barrier"}, {"layer", "1"}});
  return b;
}

TEST_CASE("barriers (standard): OSM guard rails and jersey barriers become barrier objects with continuous repeats") {
  // OpenDRIVE 1.8.0 §13.14.1 Table 108: type barrier, subtypes guardRail / jerseyBarrier, the size in a <repeat>
  // with distance 0 (no bounding box on the object); §13.4: t, zOffset, width, height linear along each repeat;
  // 1.9.0 road.object.repeating.valid_s_length
  const Result r = build(roadWithBarriers(), "US", "asam", false, true);
  CHECK(r.val.errors == 0);
  CHECK(r.rep.barriers == 2 && r.rep.barrierSamplesOnLane == 0);
  std::map<std::string, const odr::Object *> bySubtype;
  const odr::Road *road = nullptr;
  for (const auto &rd : r.od.roads)
    for (const auto &o : rd.objects)
      if (o.type == "barrier") bySubtype[o.subtype] = &o, road = &rd;
  CHECK(bySubtype.size() == 2 && bySubtype.count("guardRail") && bySubtype.count("jerseyBarrier"));
  if (bySubtype.size() != 2 || !road) return;
  // the jersey barrier is tagged layer=1 beside a layer-0 road: the layer orders candidate roads, it does not
  // exclude the only road alongside
  for (const auto &[sub, o] : bySubtype) {
    const bool rail = sub == "guardRail";
    CHECK(o->orientation == "none" && o->length == 0 && o->width == 0 && o->height == 0 && !o->repeats.empty());
    if (o->repeats.empty()) continue;
    double length = 0;
    for (size_t k = 0; k < o->repeats.size(); ++k) {
      const odr::ObjectRepeat &rp = o->repeats[k];
      CHECK(rp.distance == 0 && rp.length > 0 && rp.s >= 0 && rp.s + rp.length <= road->length() + 1e-9);
      CHECK(rp.heightStart == (rail ? 0.31 : 0.81) && rp.widthStart == (rail ? 0.10 : 0.60));
      // on the flat road: the guard rail's beam 0.44 m above the ground, the jersey barrier on it
      CHECK(std::abs(rp.zOffsetStart - (rail ? 0.44 : 0.0)) < 1e-6);
      if (k + 1 < o->repeats.size()) {
        const odr::ObjectRepeat &nx = o->repeats[k + 1];
        CHECK(std::abs(rp.s + rp.length - nx.s) < 1e-9 && rp.tEnd == nx.tStart && rp.zOffsetEnd == nx.zOffsetStart);
      }
      length += rp.length;
    }
    CHECK(o->s == o->repeats.front().s && o->t == o->repeats.front().tStart);
    // the OSM line: 160 m (guard rail from x 20 to 180) and 140 m (jersey barrier from x 30 to 170)
    CHECK(std::abs(length - (rail ? 160.0 : 140.0)) < 1.5);
    CHECK(std::abs(o->repeats.front().tStart - (rail ? -5.5 : 5.5)) < 0.1);
    if (rail) {
      // the bend half way: t follows the OSM line to within the chord tolerance (0.05 m)
      double tMid = 0;
      for (const odr::ObjectRepeat &rp : o->repeats)
        if (rp.s <= 100.0 && rp.s + rp.length >= 100.0) tMid = rp.tStart + (100.0 - rp.s) / rp.length * (rp.tEnd - rp.tStart);
      CHECK(std::abs(tMid + 6.0) < 0.1);
    }
  }
}

TEST_CASE("barriers (standard): none on a lane in use, none without street furniture, none in carla") {
  // a guard rail drawn over the right driving lane: a barrier never blocks a lane anyone travels on
  const Result onLane = build(roadWithBarriers(-1.5), "US", "asam", false, true);
  CHECK(onLane.val.errors == 0);
  CHECK(onLane.rep.barriers == 1 && onLane.rep.barrierSamplesOnLane > 100);
  const Result off = build(roadWithBarriers(), "US", "asam", false, false);
  CHECK(off.rep.barriers == 0);
  // 1.4H §6.6 has no barrier subtypes and CARLA reads crosswalk objects only
  const Result carla = build(roadWithBarriers(), "US", "carla", false, true);
  CHECK(carla.rep.barriers == 0);
  for (const auto &road : carla.od.roads)
    for (const auto &o : road.objects) CHECK(o.type != "barrier");
}

TEST_CASE("cross paths (standard): the crosswalks of a signalised junction are cross paths inside it") {
  // Junction Guideline 1.0.0 section 6.3.3 "Add roads for the pedestrian crossings inside of the junction area",
  // section 6.4.3 "Choose the road reference lines to stop before any cross path in the junction"; OpenDRIVE 1.8.0
  // section 12.5: the crossing road links the walking lanes of two connecting roads (the corner sidewalks)
  const Result r = build(crossroads(true), "US", "asam");
  CHECK(r.val.errors == 0);
  CHECK(r.od.junctions.size() == 1);
  if (r.od.junctions.size() != 1) return;
  const odr::Junction &J = r.od.junctions[0];
  CHECK(r.rep.junctionCrossPaths == 4 && J.crossPaths.size() == 4);
  for (const auto &cp : J.crossPaths) {
    const odr::Road *x = roadById(r.od, cp.crossingRoad), *a = roadById(r.od, cp.roadAtStart), *b = roadById(r.od, cp.roadAtEnd);
    CHECK(x && a && b);
    if (!x || !a || !b) return;
    CHECK(x->junction == J.id && a->junction == J.id && b->junction == J.id && a != b);
    CHECK(cornerSidewalk(*a) && cornerSidewalk(*b));
    // the marking is on the crossing road; the linked walking lanes are lowered over the crossing
    CHECK(std::count_if(x->objects.begin(), x->objects.end(), [](const odr::Object &o) { return o.type == "crosswalk"; }) == 1);
    for (const auto &[road, link] : {std::pair<const odr::Road *, odr::CrossPath::Link>{a, cp.start}, {b, cp.end}}) {
      const odr::LaneSection &sec = odr::sectionAt(*road, link.s);
      const odr::Lane *walk = nullptr;
      for (const auto &l : sec.right)
        if (l.id == link.from) walk = &l;
      CHECK(walk && walk->type == "sidewalk");
      if (!walk) return;
      double inner = walk->heightInner.value_or(0.0);
      for (const auto &hc : walk->heightChanges)
        if (hc.sOffset <= link.s - sec.s) inner = hc.inner;
      CHECK(std::abs(inner - 0.02) < 1e-9);
    }
  }
  // no crosswalk object left on the approaches: they end before the junction's cross paths
  for (const auto &road : r.od.roads)
    if (road.junction == "-1")
      for (const auto &o : road.objects) CHECK(o.type != "crosswalk");
  // carla: crosswalk objects on the approaches, no cross paths
  const Result c = build(crossroads(true), "US", "carla");
  CHECK(c.rep.crossPaths == 0);
}

TEST_CASE("cross paths (standard): a mid-block crossing is a virtual junction with one cross path, none in carla") {
  OsmBuilder b;
  const OsmId x = b.node(0, 0, {{"highway", "crossing"}});
  b.way({b.node(-100, 0), x, b.node(100, 0)}, {{"highway", "residential"}, {"lanes", "2"}, {"sidewalk", "both"}});
  for (const char *target : {"asam", "carla"}) {
    const Result r = build(b, "US", target);
    CHECK(r.val.errors == 0);
    const bool standard = std::string(target) == "asam";
    CHECK(r.rep.crossPaths == (standard ? 1 : 0));
    if (!standard) {
      CHECK(r.od.junctions.empty());
      continue;
    }
    // OpenDRIVE 1.9 section 12.7: a virtual junction on the uncut road, round the crosswalk
    CHECK(r.od.junctions.size() == 1);
    if (!(r.od.junctions.size() == 1)) return;
    const odr::Junction &j = r.od.junctions[0];
    CHECK(j.type == "virtual");
    const odr::Road *main = roadById(r.od, j.mainRoad);
    CHECK(main != nullptr);
    if (!(main != nullptr)) return;
    CHECK(main->junction == "-1");
    CHECK(j.sStart < 0.5 * main->length() && 0.5 * main->length() < j.sEnd);
    // section 12.5: the cross path links the outermost sidewalk on each side to the crossing road's lane
    CHECK(j.crossPaths.size() == 1);
    if (!(j.crossPaths.size() == 1)) return;
    const odr::CrossPath &cp = j.crossPaths[0];
    CHECK(cp.roadAtStart == main->id && cp.roadAtEnd == main->id);
    // Junction Guideline 1.0.0 section 10.3.3: the road for general traffic has the high priority; sStart / sEnd
    // clearly outside the crossing (3 m crosswalk, 0.5 m clear on each side)
    CHECK(j.priorities.size() == 1 && j.priorities[0].high == main->id && j.priorities[0].low == cp.crossingRoad);
    CHECK_NEAR(j.sEnd - j.sStart, 4.0, 1e-6);
    const odr::LaneSection &sec = odr::sectionAt(*main, cp.start.s);
    CHECK(cp.start.from == sec.right.back().id);
    CHECK(cp.end.from == sec.left.back().id);
    const odr::Road *x = roadById(r.od, cp.crossingRoad);
    CHECK(x != nullptr);
    if (!(x != nullptr)) return;
    CHECK(x->junction == j.id);
    CHECK(x->lanes.size() == 1 && x->lanes[0].right.size() == 1 && x->lanes[0].left.empty());
    if (!(x->lanes.size() == 1 && x->lanes[0].right.size() == 1 && x->lanes[0].left.empty())) return;
    CHECK(x->lanes[0].right[0].type == "sidewalk");
    CHECK(cp.start.to == -1 && cp.end.to == -1);
    // straight across, sidewalk centre to sidewalk centre (OpenDRIVE 1.9 section 12.5 rule
    // junctions.cross_path.start_end_contained: its ends inside the linked lanes)
    double lIn, lOut, rIn, rOut;
    CHECK(odr::laneBorders(*main, cp.start.s, cp.end.from, lIn, lOut));
    if (!(odr::laneBorders(*main, cp.start.s, cp.end.from, lIn, lOut))) return;
    CHECK(odr::laneBorders(*main, cp.start.s, cp.start.from, rIn, rOut));
    if (!(odr::laneBorders(*main, cp.start.s, cp.start.from, rIn, rOut))) return;
    CHECK_NEAR(x->length(), 0.5 * (lIn + lOut) - 0.5 * (rIn + rOut), 1e-6);
  }
}

TEST_CASE("network: T junction with a stop sign on the minor road") {
  OsmBuilder b;
  const OsmId c = b.node(0, 0);
  const Tags major{{"highway", "secondary"}, {"lanes", "2"}};
  b.way({b.node(-150, 0), c}, major);
  b.way({c, b.node(150, 0)}, major);
  const OsmId stop = b.node(0, -15, {{"highway", "stop"}, {"direction", "forward"}});
  b.way({b.node(0, -150), stop, c}, {{"highway", "residential"}});
  const Result r = build(b);
  CHECK(r.val.errors == 0);
  CHECK(r.od.junctions.size() == 1);
  int stops = 0;
  for (const auto &road : r.od.roads)
    for (const auto &s : road.signals)
      if (s.name == "Stop") {
        ++stops;
        CHECK(s.country == "US" && s.type == "R1-1");
      }
  CHECK(stops == 1);
  // no U-turns, and the minor road can turn both ways
  CHECK(count(r.od, true) >= 6);
}

TEST_CASE("network: lane added along a road becomes a tapered lane section, not a junction") {
  OsmBuilder b;
  const OsmId m = b.node(0, 0);
  b.way({b.node(-200, 0), b.node(-100, 0), m}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}});
  b.way({m, b.node(100, 0), b.node(200, 0)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}});
  const Result r = build(b);
  CHECK(r.val.errors == 0);
  CHECK(r.od.junctions.empty());
  CHECK(r.od.roads.size() == 1);
  const auto &road = r.od.roads[0];
  CHECK(road.lanes.size() == 2);
  // spec 4.4: the new lane starts at zero width with no predecessor, the others are linked
  int added = 0;
  for (const auto &l : road.lanes[1].right) {
    if (l.type != "driving") continue;
    if (!l.link.predecessor) {
      ++added;
      CHECK_NEAR(odr::laneWidthAt(l, 0.0), 0.0, 1e-9);
    }
  }
  CHECK(added == 1);
  // 1.9 §11.9: the line between the new lane and the lane it opens from is drawn once the lane is
  // fully open - no mark along the taper, the lane line from where the taper ends
  const auto &opened = road.lanes[1].right;
  for (size_t i = 0; i < opened.size(); ++i) {
    if (opened[i].type != "driving" || opened[i].link.predecessor) continue;
    const bool pocket = i + 1 < opened.size() && opened[i + 1].type == "driving";
    const odr::Lane &line = pocket ? opened[i] : opened[i - 1];
    CHECK(line.roadMarks.size() == 2);
    if (line.roadMarks.size() != 2) continue;
    CHECK(line.roadMarks[0].type == "none" && line.roadMarks[0].sOffset == 0.0);
    CHECK(line.roadMarks[1].type == "broken");
    CHECK_NEAR(odr::laneWidthAt(opened[i], line.roadMarks[1].sOffset), opened[i].widths.back().a, 1e-6);
  }
  // only right lanes on a RHT oneway, centred on the way through laneOffset
  CHECK(road.lanes[0].left.empty() || road.lanes[0].left.front().type != "driving");
}

TEST_CASE("network: left-hand traffic (GB) - rule LHT, +s lanes on the left") {
  OsmBuilder b = crossroads(true);
  const Result r = build(b, "GB", "asam");
  CHECK(r.val.errors == 0);
  for (const auto &road : r.od.roads) {
    CHECK(road.rule == "LHT");
    if (road.junction == "-1" || road.name == "Crossing") continue;
    // connecting lanes travel along +s: on the left side in LHT (corner sidewalks too, mirrored)
    CHECK(!road.lanes.front().left.empty() && road.lanes.front().right.empty());
  }
  CHECK(std::count_if(r.od.roads.begin(), r.od.roads.end(), cornerSidewalk) == 4);
}

TEST_CASE("junction laneLinks start only at lanes leading into the junction (Annex F.4.6.6), RHT and LHT") {
  // OpenDRIVE 1.9.0 Annex F.4.6.6 junctions.connection.one_link_to_incoming (since 1.8.0); 1.8.0 §11.2: a lane's
  // direction follows its id sign and road@rule (RHT: negative ids along +s; LHT: positive ids along +s). The
  // corner sidewalks used to be built the RHT way round everywhere: London failed this for all 34 (ASAM qc-opendrive)
  for (const char *country : {"US", "GB"}) {
    const Result r = build(crossroads(true), country, "asam");
    CHECK(r.val.errors == 0);
    const bool lht = std::string(country) == "GB";
    std::map<std::string, const odr::Road *> byId;
    for (const auto &rd : r.od.roads) byId[rd.id] = &rd;
    int links = 0;
    for (const auto &j : r.od.junctions)
      for (const auto &cn : j.connections) {
        const odr::Road &c = *byId.at(cn.connectingRoad);
        const auto &lk = cn.contactPoint == "start" ? c.predecessor : c.successor;
        if (!lk || lk->elementId != cn.incomingRoad) continue;
        const bool atEnd = lk->contactPoint == "end";  // the incoming road meets the junction with its end
        for (const auto &ll : cn.laneLinks) {
          ++links;
          const bool alongS = lht ? ll.from > 0 : ll.from < 0;  // travels along +s of the incoming road
          CHECK(alongS == atEnd);
        }
      }
    CHECK(links > 0);
  }
}

TEST_CASE("network: CARLA target - numeric ids, OpenDRIVE sign catalogue, speed buckets") {
  OsmBuilder b = crossroads(false);
  b.node(0, -10, {{"highway", "stop"}, {"stop", "all"}});
  const Result r = build(b, "US", "carla");
  CHECK(r.val.errors == 0);
  for (const auto &road : r.od.roads) {
    CHECK(std::all_of(road.id.begin(), road.id.end(), ::isdigit));
    for (const auto &s : road.signals) {
      CHECK(s.country == "OpenDRIVE");
      CHECK(std::all_of(s.id.begin(), s.id.end(), ::isdigit));
      if (s.type == "274") {
        const int v = std::stoi(s.subtype);
        CHECK(v >= 30 && v <= 120 && v % 10 == 0 && s.unit == "km/h");
      }
      for (const auto &l : road.lanes.front().right) CHECK(carlaLaneType(l.type));
    }
  }
}

TEST_CASE("network: OpenDRIVE 1.8 target - ISO country codes, walking lanes allowed") {
  const Result r = build(crossroads(true), "DE", "asam18");
  CHECK(r.val.errors == 0);
  for (const auto &road : r.od.roads)
    for (const auto &t : road.types) CHECK(t.country.empty() || t.country.size() == 2);
}

TEST_CASE("network: curved road keeps G1/G2 continuity and follows the OSM nodes") {
  OsmBuilder b;
  std::vector<OsmId> nodes;
  for (int i = 0; i <= 24; ++i) {
    const double a = i * kPi / 48.0;  // quarter circle, R = 150 m
    nodes.push_back(b.node(150 * std::sin(a), 150 - 150 * std::cos(a)));
  }
  b.way(nodes, {{"highway", "secondary"}, {"lanes", "2"}});
  const Result r = build(b);
  CHECK(r.val.errors == 0);
  CHECK(r.rep.fitMaxDev < 0.05);
  const auto &L = r.od.roads[0].planView;
  for (size_t i = 1; i < L.elements.size(); ++i)
    CHECK_NEAR(L.elements[i - 1].end().kappa, L.elements[i].k0, 1e-4);  // curvature continuous
}

TEST_CASE("network: an inner lane that merges away from the divider ends there, the others flow on (RHT and LHT)") {
  // a three-lane oneway whose inner lane merges outward (turn:lanes merge_to_right, in LHT
  // merge_to_left on the rightmost lane) continues with two lanes: the inner lane tapers out, the outer
  // two flow on without a step, and the two-lane carriageway is centred on the way (OSM placement
  // default) again once the taper is over
  for (const char *country : {"US", "GB"}) {
    const bool lht = std::string(country) == "GB";
    OsmBuilder b;
    const OsmId m = b.node(0, 0);
    b.way({b.node(-200, 0), b.node(-100, 0), m},
          {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", lht ? "none|none|merge_to_left" : "merge_to_right|none|none"}});
    b.way({m, b.node(100, 0), b.node(200, 0)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}});
    const Result r = build(b, country);
    CHECK(r.val.errors == 0);
    CHECK(r.od.roads.size() == 1);
    if (r.od.roads.size() != 1) continue;
    const auto &road = r.od.roads[0];
    CHECK(road.lanes.size() == 2);
    if (road.lanes.size() != 2) continue;
    const auto &before = lht ? road.lanes[0].left : road.lanes[0].right;
    int ended = 0, outer = 0;
    for (const auto &l : before)
      if (l.type == "driving") {
        if (!l.link.successor) ended = l.id;
        if (std::abs(l.id) > std::abs(outer)) outer = l.id;
      }
    CHECK(std::abs(ended) == 1);  // the inner lane ends
    // both carriageways are centred on the way, before the taper and after it
    double a0, b0, a1, b1, pa = 0, pb = 0, worstStep = 0;
    CHECK(odr::laneBorders(road, 10.0, outer, a0, b0));
    CHECK(odr::laneBorders(road, road.length() - 10.0, outer > 0 ? outer - 1 : outer + 1, a1, b1));
    const double off0 = road.laneOffset.front().a, off1 = road.laneOffset.back().a;
    CHECK_NEAR(0.5 * (b0 + off0), 0.0, 1e-6);  // the 3-lane band [offset, outer edge] has its middle on the way
    CHECK_NEAR(0.5 * (b1 + off1), 0.0, 1e-6);  // and so has the 2-lane band
    // the outer edge flows through the drop without a step
    for (double s = 0; s <= road.length(); s += 0.1) {
      const int id = s < road.lanes[1].s ? outer : (outer > 0 ? outer - 1 : outer + 1);
      double a, bb;
      CHECK(odr::laneBorders(road, s, id, a, bb));
      if (s > 0) worstStep = std::max(worstStep, std::abs(bb - pb));
      pa = a, pb = bb;
    }
    (void)pa;
    CHECK(worstStep < 0.01);
  }
}

TEST_CASE("network: placement:forward / placement:backward put the way on the named lane edge (RHT and LHT)") {
  // a two-way road, 2 lanes forward and 1 backward, the way drawn along the outer edge of the
  // backward lane (placement:backward=right_of:1): the whole carriageway lies on the forward side
  for (const char *country : {"US", "GB"}) {
    const bool lht = std::string(country) == "GB";
    for (const char *key : {"placement:backward", "placement:forward"}) {
      const bool bwd = std::string(key) == "placement:backward";
      OsmBuilder b;
      b.way({b.node(-100, 0), b.node(100, 0)}, {{"highway", "secondary"}, {"lanes", "3"}, {"lanes:forward", "2"},
                                                 {"lanes:backward", "1"}, {key, bwd ? "right_of:1" : "left_of:1"}});
      const Result r = build(b, country);
      CHECK(r.val.errors == 0);
      CHECK(r.od.roads.size() == 1);
      if (r.od.roads.size() != 1) continue;
      const auto &road = r.od.roads[0];
      double lo = 1e9, hi = -1e9;  // driving band across the way
      for (const auto *side : {&road.lanes[0].left, &road.lanes[0].right})
        for (const auto &l : *side) {
          double a, c;
          if (l.type == "driving" && odr::laneBorders(road, 50.0, l.id, a, c))
            lo = std::min({lo, a, c}), hi = std::max({hi, a, c});
        }
      // lanes are counted from the left in their driving direction. RHT: backward=right_of:1 is the
      // outer edge of the backward lane (left of the way), forward=left_of:1 the divider. LHT: the
      // backward lane lies right of the way and runs against it, so its right edge is the divider; the
      // forward lanes lie left of the way and their lane 1 is the outer one.
      if (bwd != lht) CHECK_NEAR(hi, 0.0, 1e-6);
      else {
        CHECK_NEAR(road.laneOffset.front().a, 0.0, 1e-6);
        CHECK(lo < -1.0);
        CHECK(hi > 1.0);
      }
    }
  }
}

TEST_CASE("network: roundabout with three arms") {
  OsmBuilder b;
  std::vector<OsmId> ring;
  const int n = 16;
  OsmId arm[3];
  for (int i = 0; i < n; ++i) {
    const double a = 2 * kPi * i / n;
    ring.push_back(b.node(25 * std::cos(a), 25 * std::sin(a)));
  }
  ring.push_back(ring.front());
  b.way(ring, {{"highway", "tertiary"}, {"junction", "roundabout"}});
  for (int k = 0; k < 3; ++k) {
    const int idx = k * n / 3;
    const double a = 2 * kPi * idx / n;
    arm[k] = b.way({ring[idx], b.node(140 * std::cos(a), 140 * std::sin(a))}, {{"highway", "tertiary"}});
  }
  (void)arm;
  const Result r = build(b);
  CHECK(r.val.errors == 0);
  CHECK(r.od.junctions.size() == 3);
  // the ring is oneway: its connecting roads only lead in the ring direction
  CHECK(count(r.od, true) >= 6);
  // OpenDRIVE 1.9 12.13: the three junctions form one junction group of type roundabout
  CHECK(r.od.junctionGroups.size() == 1);
  CHECK(r.od.junctionGroups.size() == 1 && r.od.junctionGroups[0].type == "roundabout" &&
        r.od.junctionGroups[0].junctionIds.size() == 3);
}

TEST_CASE("network: a lane width change between ways blends across the boundary (no border step), RHT and LHT") {
  for (const char *country : {"US", "GB"}) {
    OsmBuilder b;
    const OsmId m = b.node(0, 0);
    // the same two lanes, 4 m wide on the first way and 3.25 m on the second (width tags)
    b.way({b.node(-200, 0), b.node(-100, 0), m}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"width", "8"}});
    b.way({m, b.node(100, 0), b.node(200, 0)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"width", "6.5"}});
    const Result r = build(b, country);
    CHECK(r.val.errors == 0);
    CHECK(r.od.roads.size() == 1);
    const auto &road = r.od.roads[0];
    CHECK(road.lanes.size() == 2);
    if (road.lanes.size() != 2) continue;
    const double sb = road.lanes[1].s;
    std::vector<int> ids;
    for (const auto *side : {&road.lanes[0].left, &road.lanes[0].right})
      for (const auto &l : *side)
        if (l.type == "driving") ids.push_back(l.id);
    CHECK(ids.size() == 2);
    double worstStep = 0;
    for (int id : ids) {
      double pa = 0, pb = 0;
      for (double s = 0; s <= road.length(); s += 0.1) {
        double a, bb;
        CHECK(odr::laneBorders(road, s, id, a, bb));
        if (s > 0) worstStep = std::max({worstStep, std::abs(a - pa), std::abs(bb - pb)});
        pa = a, pb = bb;
      }
      // the old width up to half a taper before the boundary, the mean at it, the new one half a taper after
      auto width = [&](double s) {
        double a, bb;
        odr::laneBorders(road, s, id, a, bb);
        return std::abs(bb - a);
      };
      CHECK_NEAR(width(sb - 20.0), 4.0, 1e-6);
      CHECK_NEAR(width(sb), 3.625, 1e-3);
      CHECK_NEAR(width(sb + 20.0), 3.25, 1e-6);
    }
    CHECK(worstStep < 0.01);  // 0.1 m apart: a step at the boundary would be 0.75 m
  }
}

TEST_CASE("laneOffset: the C2 shift is level at both ends and continuous in value, slope and curvature") {
  for (double D : {3.5, -1.75, 0.3}) {
    const auto p = odr::Poly3::shiftC2(10.0, 40.0, 1.0, 1.0 + D);
    CHECK(p.size() == 4);
    auto second = [](const odr::Poly3 &q, double ds) { return 2.0 * q.c + 6.0 * q.d * ds; };
    CHECK_NEAR(p[0].a, 1.0, 1e-12);
    CHECK_NEAR(p[0].b, 0.0, 1e-12);
    CHECK_NEAR(second(p[0], 0.0), 0.0, 1e-12);
    for (size_t i = 0; i + 1 < p.size(); ++i) {
      const double L = p[i + 1].s - p[i].s;
      CHECK_NEAR(p[i].eval(L), p[i + 1].a, 1e-12);
      CHECK_NEAR(p[i].deriv(L), p[i + 1].b, 1e-12);
      CHECK_NEAR(second(p[i], L), second(p[i + 1], 0.0), 1e-12);
    }
    CHECK_NEAR(p[3].eval(10.0), 1.0 + D, 1e-12);
    CHECK_NEAR(p[3].deriv(10.0), 0.0, 1e-12);
    CHECK_NEAR(second(p[3], 10.0), 0.0, 1e-12);
  }
}

TEST_CASE("reference line: stays within maxLegDeviation of a long straight leg next to a short one") {
  // a 300 m straight leg after a 13 m leg that turns 6 degrees: the 3-point tangent at the shared vertex
  // tilts the long leg's spiral, which bowed several metres off the leg before the bounded refit
  const double a = 6.0 * kPi / 180.0;
  const std::vector<Vec2> pts{{-13.0 * std::cos(a), 13.0 * std::sin(a)}, {0, 0}, {300, 0}, {310, 4}};
  PolylineFitOptions o;
  const ReferenceLine line = fitPolyline(pts, o);
  double worst = 0.0;
  for (double x = 75; x <= 225; x += 5) {
    double s, t;
    line.project({x, 0}, s, t);
    worst = std::max(worst, std::abs(t));
  }
  CHECK(worst <= o.maxLegDeviation + 1e-6);
}

TEST_CASE("cross-section: a road reaches out to its separately mapped sidewalks (verge + curb + sidewalk)") {
  for (const char *country : {"US", "GB"}) {
    OsmBuilder b;
    b.way({b.node(-100, 0), b.node(100, 0)}, {{"highway", "secondary"}, {"lanes", "2"}, {"sidewalk", "separate"}});
    b.way({b.node(-100, 8.5), b.node(100, 8.5)}, {{"highway", "footway"}, {"footway", "sidewalk"}});
    b.way({b.node(-100, -8.5), b.node(100, -8.5)}, {{"highway", "footway"}, {"footway", "sidewalk"}, {"width", "3"}});
    const Result r = build(b, country, "asam", true);
    CHECK(r.val.errors == 0);
    CHECK(r.od.roads.size() == 1);
    if (r.od.roads.size() != 1) continue;
    const auto &road = r.od.roads[0];
    const double s = 0.5 * road.length();
    for (const auto *side : {&road.lanes[0].left, &road.lanes[0].right}) {
      CHECK(side->size() >= 3);
      if (side->size() < 3) continue;
      const odr::Lane &walk = side->back(), &verge = (*side)[side->size() - 2];
      CHECK(walk.type == "sidewalk");
      CHECK(verge.type == "border");
      CHECK(verge.roadMarks.front().type == "curb");
      double a, c;
      CHECK(odr::laneBorders(road, s, walk.id, a, c));
      CHECK_NEAR(std::abs(0.5 * (a + c)), 8.5, 1e-6);                     // centred on the mapped sidewalk
      CHECK_NEAR(std::abs(c - a), walk.id > 0 ? 2.0 : 3.0, 1e-6);        // its width (tag or country default)
    }
  }
}

namespace {
/// driving lanes (any vehicle ramp type) of every section of the plain roads built from OSM way `way`
std::vector<int> drivingLanesOfWay(const odr::OpenDrive &od, OsmId way) {
  std::vector<int> out;
  const std::string id = std::to_string(way);
  for (const auto &r : od.roads) {
    if (r.junction != "-1") continue;
    const auto ud = r.userData.find("osmWays");
    if (ud == r.userData.end() || ("," + ud->second + ",").find("," + id + ",") == std::string::npos) continue;
    for (const auto &sec : r.lanes) {
      int n = 0;
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) n += l.type == "driving" || l.type == "onRamp" || l.type == "offRamp" || l.type == "connectingRamp";
      out.push_back(n);
    }
  }
  return out;
}
}  // namespace

TEST_CASE("lanes: an untagged link piece takes the lanes of the tagged link piece it continues") {
  // OSM wiki Key:lanes: an untagged count is a guess; a ramp piece continuing a tagged piece one to one keeps its lanes
  // (83 % of tagged continuations in the test areas) instead of the motorway class default of 2 (LA review flag 2)
  OsmBuilder b;
  const OsmId n0 = b.node(0, 0), n1 = b.node(150, 0), n2 = b.node(300, 0);
  b.way({n0, n1}, {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "1"}});
  const OsmId untagged = b.way({n1, n2}, {{"highway", "motorway_link"}, {"oneway", "yes"}});
  const Result r = build(b);
  CHECK(r.val.errors == 0);
  CHECK(r.rep.inheritedLinkLanes == 1);
  const std::vector<int> lanes = drivingLanesOfWay(r.od, untagged);
  CHECK(!lanes.empty());
  for (int n : lanes) CHECK(n == 1);
}

TEST_CASE("lanes: an untagged branch of a link split takes the trunk lanes whose turn:lanes bear toward it") {
  // trunk lanes=2 turn:lanes=none|slight_right splits into a left branch lanes=2 and an untagged right branch: the right
  // lane bears right, so the right branch is one lane (OSM wiki Key:turn:lanes; LA review flag 1)
  OsmBuilder b;
  const OsmId n0 = b.node(0, 0), split = b.node(200, 0);
  b.way({n0, split}, {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "2"}, {"turn:lanes", "none|slight_right"}});
  b.way({split, b.node(300, 25), b.node(400, 60)}, {{"highway", "motorway_link"}, {"oneway", "yes"}, {"lanes", "2"}});
  const OsmId right = b.way({split, b.node(300, -25), b.node(400, -60)}, {{"highway", "motorway_link"}, {"oneway", "yes"}});
  // carla profile: the lane rule does not depend on the profile, and the standard profile would model this split as a
  // direct junction (1.7+), which the harness's 1.4 copy of every scenario cannot carry
  const Result r = build(b, "US", "carla");
  CHECK(r.val.errors == 0);
  CHECK(r.rep.inheritedLinkLanes == 1);
  const std::vector<int> lanes = drivingLanesOfWay(r.od, right);
  CHECK(!lanes.empty());
  if (!lanes.empty()) CHECK(lanes.back() == 1);
}
