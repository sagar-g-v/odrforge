// End-to-end OSM -> OpenDRIVE scenarios. Every generated network must pass the full ASAM
// conformance checker (validate) without errors, plus scenario-specific spec expectations.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
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
             bool separateSidewalks = false) {
  BuildOptions o;
  o.separateSidewalks = separateSidewalks;
  o.country = CountryProfile::load(country);
  o.target = TargetProfile::byName(target);
  o.streetFurniture = false;
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
  return r.junction != "-1" && r.name != "Crossing" && sec.left.empty() && !sec.right.empty() && sec.right.back().type == "sidewalk" &&
         std::all_of(sec.right.begin(), sec.right.end(), [](const odr::Lane &l) { return l.type == "sidewalk" || l.type == "curb"; });
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
        // before the approach crosswalk (3 m wide, centred 1.6 m from the road end)
        CHECK(atEnd ? stop->s < road.length() - 3.1 : stop->s > 3.1);
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
        if (o.type != "crosswalk") continue;
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
    CHECK(lowered > 0);
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

TEST_CASE("cross paths (standard): approach crosswalks of a junction are no virtual junctions") {
  // Junction Guideline 1.0.0 section 6.3.3 / 6.4.3: crossings at a junction lie inside the common junction's
  // area - not modelled as cross paths yet; their crosswalk objects stay
  const Result r = build(crossroads(true), "US", "asam");
  CHECK(r.val.errors == 0);
  CHECK(r.rep.crossPaths == 0);
  CHECK(r.od.junctions.size() == 1 && r.od.junctions[0].crossPaths.empty());
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
    if (road.junction == "-1" || cornerSidewalk(road) || road.name == "Crossing") continue;  // walking lanes: right side
    // connecting lanes travel along +s: on the left side in LHT
    CHECK(!road.lanes.front().left.empty() && road.lanes.front().right.empty());
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
