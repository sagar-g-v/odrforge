// Network topology: lane-level gores, OSM near-misses, parking-lot through routes, the only-way-out
// rule for sharp turns. Spec basis: spec 4.4 lane sections / 4.5 widths (branch lanes on the trunk),
// spec 5 linkage (every lane of a connecting road links to existing lanes on both sides), plus the
// visual quality rule quality.road_overlap.
#include <cstdio>

#include "odrforge/network.hpp"
#include "odrforge/validate.hpp"
#include "test.hpp"

using namespace odrforge;

namespace {

struct OsmB {
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
    w.id = 9000 + next++;
    w.nodes = std::move(nodes);
    w.tags = std::move(t);
    d.ways[w.id] = w;
    return w.id;
  }
};

BuildOptions opts() {
  BuildOptions o;
  o.country = CountryProfile::load("US");
  o.target = TargetProfile::byName("asam");
  o.streetFurniture = false;
  return o;
}

void printErrors(const ValidationReport &v) {
  for (size_t i = 0; i < v.messages.size() && i < 8; ++i) std::printf("    %s\n", v.messages[i].c_str());
}

const odr::Road *roadNear(const odr::OpenDrive &od, double x, double y, bool connecting = false) {
  const odr::Road *best = nullptr;
  double bd = 1e9;
  for (const auto &r : od.roads) {
    if ((r.junction != "-1") != connecting) continue;
    double s, t;
    r.planView.project({x, y}, s, t);
    const Pose p = r.planView.eval(s);
    const double d = std::hypot(p.x - x, p.y - y);
    if (d < bd) bd = d, best = &r;
  }
  return best;
}

int drivingLanes(const odr::LaneSection &sec) {
  int n = 0;
  for (const auto *side : {&sec.left, &sec.right})
    for (const auto &l : *side) n += l.type == "driving";
  return n;
}

}  // namespace

TEST_CASE("gore: a freeway exit is a lane-level split, no overlapping road surfaces") {
  OsmB b;
  const OsmId n = b.node(0, 0);
  const Tags fwy{{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}};
  b.way({b.node(-400, 0), b.node(-200, 0), n}, fwy);
  b.way({n, b.node(200, 0), b.node(400, 0)}, fwy);
  // the exit leaves on the right at a shallow angle (OSM draws it from the same node)
  b.way({n, b.node(80, -3), b.node(160, -9), b.node(240, -20), b.node(330, -42)},
        {{"highway", "motorway_link"}, {"lanes", "1"}, {"oneway", "yes"}});
  BuildOptions o = opts();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(rep.gores == 1);
  CHECK(v.errors == 0);
  CHECK(v.count("quality.road_overlap") == 0);
  // the trunk carries 3 + 1 lanes before the junction; the junction is short
  const odr::Road *trunk = roadNear(od, -300, 0);
  CHECK(trunk && drivingLanes(trunk->lanes.back()) == 4 && drivingLanes(trunk->lanes.front()) == 3);
  int connectingLanes = 0, toRamp = 0;
  const odr::Road *ramp = roadNear(od, 300, -35);
  for (const auto &r : od.roads) {
    if (r.junction == "-1") continue;
    connectingLanes += drivingLanes(r.lanes.front());
    CHECK(r.length() < 10.0);
    if (r.successor && ramp && r.successor->elementId == ramp->id) toRamp += drivingLanes(r.lanes.front());
  }
  CHECK(connectingLanes == 4);  // 3 main lanes straight on, 1 branch lane into the ramp
  CHECK(toRamp == 1);
  // the branch lane is the outermost (right) lane of the trunk
  for (const auto &r : od.roads)
    if (r.junction != "-1" && ramp && r.successor && r.successor->elementId == ramp->id) {
      const auto &l = r.lanes.front().right.empty() ? r.lanes.front().left.front() : r.lanes.front().right.front();
      CHECK(l.link.predecessor && std::abs(*l.link.predecessor) == 4);
    }
}

TEST_CASE("gore: a freeway entrance is a lane-level merge") {
  OsmB b;
  const OsmId n = b.node(0, 0);
  const Tags fwy{{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}};
  b.way({b.node(-400, 0), b.node(-200, 0), n}, fwy);
  b.way({n, b.node(200, 0), b.node(400, 0)}, fwy);
  b.way({b.node(-330, -42), b.node(-240, -20), b.node(-160, -9), b.node(-80, -3), n},
        {{"highway", "motorway_link"}, {"lanes", "1"}, {"oneway", "yes"}});
  BuildOptions o = opts();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(rep.gores == 1);
  CHECK(v.errors == 0);
  CHECK(v.count("quality.road_overlap") == 0);
  const odr::Road *trunk = roadNear(od, 300, 0);
  CHECK(trunk && drivingLanes(trunk->lanes.front()) == 4 && drivingLanes(trunk->lanes.back()) == 3);
}

TEST_CASE("topology: a driveway hooking into a oneway street at its last node is connected") {
  OsmB b;
  const OsmId n = b.node(0, 0);
  const Tags street{{"highway", "primary"}, {"lanes", "2"}, {"oneway", "yes"}};
  b.way({b.node(-150, 0), n}, street);
  b.way({n, b.node(150, 0)}, street);
  // drawn alongside the street against its direction, turning into it only at the end
  b.way({b.node(90, -12), b.node(6, -12), n}, {{"highway", "service"}, {"service", "driveway"}, {"oneway", "yes"}});
  BuildOptions o = opts();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  const odr::Road *dw = roadNear(od, 60, -12);
  CHECK(dw && dw->successor.has_value());
  int fromDriveway = 0;
  for (const auto &r : od.roads)
    if (r.junction != "-1" && r.predecessor && dw && r.predecessor->elementId == dw->id) ++fromDriveway;
  CHECK(fromDriveway >= 1);
}

TEST_CASE("topology: a side street stopping short of the street it meets is joined; real dead ends stay") {
  auto build = [](Tags endTags, double gap, double dir) {
    OsmB b;
    b.way({b.node(-150, 0), b.node(0, 0), b.node(150, 0)}, {{"highway", "residential"}});
    // side street from the south, ending `gap` metres from the street centre line
    const OsmId e = b.node(0, -gap, endTags);
    b.way(dir > 0 ? std::vector<OsmId>{b.node(0, -120), e} : std::vector<OsmId>{b.node(40, -gap - 60), b.node(0, -gap - 2), e},
          {{"highway", "residential"}});
    BuildOptions o = opts();
    BuildReport rep;
    const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
    return std::make_pair(rep.snappedEnds, static_cast<int>(od.junctions.size()));
  };
  const auto joined = build({}, 5.0, 1);
  CHECK(joined.first == 1 && joined.second == 1);  // a T junction
  CHECK(build({{"noexit", "yes"}}, 5.0, 1).first == 0);
  CHECK(build({{"highway", "turning_circle"}}, 5.0, 1).first == 0);
  CHECK(build({}, 14.0, 1).first == 0);  // beyond the kerb: stops on purpose
}

TEST_CASE("topology: parking aisles become roads only where the lot connects two streets") {
  OsmB b;
  const OsmId a0 = b.node(0, 0), a1 = b.node(0, 60), d0 = b.node(100, 0);
  const Tags street{{"highway", "residential"}};
  b.way({b.node(-150, 0), a0, d0, b.node(150, 0)}, street);
  b.way({b.node(-150, 60), a1, b.node(150, 60)}, street);
  const Tags aisle{{"highway", "service"}, {"service", "parking_aisle"}};
  b.way({a0, b.node(0, 30), a1}, aisle);             // through the lot: street to street
  b.way({d0, b.node(100, 25), b.node(110, 30)}, aisle);  // a lot with one entrance
  BuildOptions o = opts();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(rep.parkingAisles == 1);
  CHECK(roadNear(od, 0, 30) && std::abs(roadNear(od, 0, 30)->planView.eval(10).x) < 1.0);
}

TEST_CASE("gore: a branch line drawn across the main line before it leaves is still a lane-level split") {
  OsmB b;
  const OsmId n = b.node(0, 0);
  const Tags fwy{{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}};
  b.way({b.node(-400, 0), b.node(-200, 0), n}, fwy);
  b.way({n, b.node(200, 0), b.node(400, 0)}, fwy);
  // the exit line wanders 1.5 m to the left of the freeway line first, then leaves on the right
  b.way({n, b.node(40, 1.5), b.node(90, 0.5), b.node(150, -6), b.node(220, -18), b.node(320, -45)},
        {{"highway", "motorway_link"}, {"lanes", "1"}, {"oneway", "yes"}});
  BuildOptions o = opts();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(rep.gores == 1);
  CHECK(v.count("quality.road_overlap") == 0);
  CHECK(v.errors == 0);
}

TEST_CASE("gore: a short branch that splits again right away - both splits at lane level") {
  OsmB b;
  const OsmId n1 = b.node(0, 0), n2 = b.node(60, -3);
  b.way({b.node(-400, 0), b.node(-200, 0), n1}, {{"highway", "motorway"}, {"lanes", "5"}, {"oneway", "yes"}});
  b.way({n1, b.node(200, 0), b.node(400, 0)}, {{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}});
  b.way({n1, n2}, {{"highway", "motorway_link"}, {"lanes", "2"}, {"oneway", "yes"}});
  b.way({n2, b.node(140, -10), b.node(220, -25), b.node(320, -50)}, {{"highway", "motorway_link"}, {"lanes", "1"}, {"oneway", "yes"}});
  b.way({n2, b.node(140, -14), b.node(200, -40), b.node(260, -90)}, {{"highway", "motorway_link"}, {"lanes", "1"}, {"oneway", "yes"}});
  BuildOptions o = opts();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(rep.gores >= 1);
  CHECK(v.count("quality.road_overlap") == 0);
  CHECK(v.errors == 0);
}

TEST_CASE("junction: connecting-road reference lines continue the arm reference lines (Junction Guideline 6)") {
  OsmB b;
  const OsmId n = b.node(0, 0);
  const Tags street{{"highway", "secondary"}, {"lanes", "4"}};
  for (const auto &[x, y] : {std::pair<double, double>{-200, 0}, {200, 0}, {0, -200}, {0, 200}})
    b.way({b.node(x, y), n}, street);
  BuildOptions o = opts();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  auto roadById = [&](const std::string &id) -> const odr::Road * {
    for (const auto &q : od.roads)
      if (q.id == id) return &q;
    return nullptr;
  };
  int connecting = 0, continuing = 0, laneCentred = 0, multiLane = 0;
  for (const auto &r : od.roads) {
    if (r.junction == "-1") continue;
    ++connecting;
    multiLane += r.lanes.front().right.size() + r.lanes.front().left.size() > 1;
    const odr::Road *from = roadById(r.predecessor->elementId), *to = roadById(r.successor->elementId);
    const Pose a = from->planView.eval(r.predecessor->contactPoint == "end" ? from->length() : 0.0);
    const Pose b0 = to->planView.eval(r.successor->contactPoint == "end" ? to->length() : 0.0);
    const Pose s0 = r.planView.eval(0.0), s1 = r.planView.eval(r.length());
    const bool samePlaces = (s0.pos() - a.pos()).norm() < 1e-6 && (s1.pos() - b0.pos()).norm() < 1e-6;
    const bool parallel = std::abs(std::sin(s0.hdg - a.hdg)) < 1e-6 && std::abs(std::sin(s1.hdg - b0.hdg)) < 1e-6;
    continuing += samePlaces && parallel;
    // otherwise (a tight turn far from the arm reference lines) the road is centred on its one lane
    const auto &lanes = r.lanes.front();
    const double halfWidth = 0.5 * (lanes.right.empty() ? lanes.left : lanes.right).front().widths.front().a;
    laneCentred += !(samePlaces && parallel) && lanes.right.size() + lanes.left.size() == 1 &&
                   std::abs(std::abs(r.laneOffset.front().a) - halfWidth) < 1e-6;
  }
  CHECK(connecting > 0);
  CHECK(continuing > 0);
  CHECK(continuing + laneCentred == connecting);
  CHECK(multiLane > 0);  // two lanes straight on share one connecting road
}

TEST_CASE("junction: a dual carriageway crossing is one junction, no stub road between its halves (Guideline 6)") {
  // two one-way carriageways 14.8 m apart (a wide median), crossed by a two-way street: OSM has two
  // intersection nodes too far apart to be one node cluster, but the arm cuts squeeze the road
  // between them out
  OsmB b;
  const OsmId north = b.node(0, 7.4), south = b.node(0, -7.4);
  const Tags carriageway{{"highway", "primary"}, {"lanes", "3"}, {"oneway", "yes"}};
  b.way({b.node(-300, 7.4), north, b.node(300, 7.4)}, carriageway);
  b.way({b.node(300, -7.4), south, b.node(-300, -7.4)}, carriageway);
  const Tags street{{"highway", "secondary"}, {"lanes", "2"}};
  b.way({b.node(0, 300), north}, street);
  b.way({north, south}, street);
  b.way({south, b.node(0, -300)}, street);
  BuildOptions o = opts();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(od.junctions.size() == 1);
  for (const auto &r : od.roads) CHECK(r.junction != "-1" || r.length() > 50.0);  // only the arms remain roads
  // the street crosses the median in both directions and the carriageways connect to it
  int connecting = 0;
  for (const auto &r : od.roads) connecting += r.junction != "-1";
  CHECK(connecting >= 6);
}

TEST_CASE("junction: a lane split divides the lanes by side - connecting lanes never cross") {
  OsmB b;
  const OsmId n = b.node(0, 0);
  b.way({b.node(-300, 0), n}, {{"highway", "primary"}, {"lanes", "4"}, {"oneway", "yes"}});
  b.way({n, b.node(150, 15), b.node(300, 40)}, {{"highway", "primary"}, {"lanes", "2"}, {"oneway", "yes"}});   // left
  b.way({n, b.node(150, -15), b.node(300, -40)}, {{"highway", "primary"}, {"lanes", "2"}, {"oneway", "yes"}});  // right
  BuildOptions o = opts();
  o.laneLevelGores = false;  // the plain junction must already be right
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  // every connecting lane to the left road starts left of every connecting lane to the right road
  double minLeft = 1e9, maxRight = -1e9;
  for (const auto &r : od.roads) {
    if (r.junction == "-1" || !r.successor) continue;
    const odr::Road *to = nullptr;
    for (const auto &q : od.roads)
      if (q.id == r.successor->elementId) to = &q;
    for (const auto &lane : r.lanes.front().right) {
      Pose start;
      odr::laneCenter(r, 0.0, lane.id, start);
      if (to && to->planView.eval(to->length()).y > 0) minLeft = std::min(minLeft, start.y);
      else maxRight = std::max(maxRight, start.y);
    }
  }
  CHECK(minLeft > maxRight);
}

TEST_CASE("clip: roads meeting just outside the area stay connected") {
  OsmB b;
  const OsmId j = b.node(-102, 0);  // the junction lies 2 m outside the clip box
  b.way({b.node(100, 0), b.node(0, 0), j}, {{"highway", "residential"}});
  b.way({b.node(60, -80), b.node(0, -40), j}, {{"highway", "residential"}});
  b.way({j, b.node(-200, 0)}, {{"highway", "residential"}});
  BuildOptions o = opts();
  o.clipBox = std::array<double, 4>{-100, -100, 100, 100};
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  // the third road lies wholly outside: the two roads continue through the kept node as one road,
  // instead of ending side by side 2 m apart at the boundary
  int open = 0, roads = 0;
  for (const auto &r : od.roads)
    if (r.junction == "-1") open += !r.predecessor + !r.successor, ++roads;
  CHECK(od.junctions.empty());
  CHECK(roads == 1 && open == 2);
}

TEST_CASE("topology: minor roads crossing without a shared node get the missing intersection") {
  OsmB b;
  b.way({b.node(-100, 0), b.node(100, 0)}, {{"highway", "service"}});
  b.way({b.node(0, -100), b.node(0, 100)}, {{"highway", "service"}});
  BuildOptions o = opts();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(rep.joinedCrossings == 1);
  CHECK(od.junctions.size() == 1);
  CHECK(v.count("quality.road_overlap") == 0);
}

TEST_CASE("geometry: a sharp corner after a short leg stays near the mapped corner, the long leg straight") {
  OsmB b;
  const Vec2 p0(0, 0), p1(-11, 9), p2(65, 93);  // ~92 degree turn, 14 m then 113 m
  b.way({b.node(p0.x, p0.y), b.node(p1.x, p1.y), b.node(p2.x, p2.y)}, {{"highway", "service"}, {"oneway", "yes"}});
  BuildOptions o = opts();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  CHECK(od.roads.size() == 1);
  const odr::Road &r = od.roads[0];
  // distance from the corner to the reference line
  double s, t;
  r.planView.project(p1, s, t);
  CHECK(std::abs(t) < 2.0);
  // the second leg: points on it stay on the line through p1, p2 (no bulge)
  const Vec2 d = (p2 - p1) * (1.0 / (p2 - p1).norm());
  double worst = 0;
  for (double u = 0.3; u <= 0.95; u += 0.05) {
    const Vec2 q = p1 + (p2 - p1) * u;
    double sq, tq;
    r.planView.project(q, sq, tq);
    worst = std::max(worst, std::abs(tq));
  }
  (void)d;
  CHECK(worst < 0.3);
}

TEST_CASE("geometry: no near-straight arcs anywhere (LibCarla evaluates arcs in 32-bit floats about the centre)") {
  OsmB b;
  const OsmId c = b.node(0, 0);
  const Tags street{{"highway", "secondary"}, {"lanes", "2"}};
  b.way({b.node(-150, 0.002), c}, street);  // straight through, with a sub-millimetre drawing wiggle
  b.way({c, b.node(150, -0.002)}, street);
  b.way({b.node(0, -150), c}, street);
  b.way({c, b.node(0, 150)}, street);
  BuildOptions o = opts();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  for (const auto &r : od.roads)
    for (const auto &e : r.planView.elements)
      if (e.type() == GeomType::Arc) CHECK(std::abs(e.k0) >= 1e-5);  // radius at most 100 km: float error under 6 mm
}

TEST_CASE("junction: a side road ending inside the junction gets no connection that turns within centimetres") {
  OsmB b;
  // a junction on the edge of the clip box: the street on and the side street keep only half a
  // metre beyond the junction node, so their trimmed ends lie a metre apart at a right angle
  const OsmId c = b.node(0, 0);
  const Tags street{{"highway", "secondary"}, {"lanes", "2"}};
  b.way({b.node(-120, 0), b.node(-60, 0), c}, street);
  b.way({c, b.node(0.6, 0)}, street);
  b.way({c, b.node(0, 0.5)}, {{"highway", "residential"}});
  BuildOptions o = opts();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.count("junctions.connection.smooth_fit") == 0);
  CHECK(v.errors == 0);
  // the long street still reaches both stubs
  int fromStreet = 0;
  for (const auto &r : od.roads)
    if (r.junction != "-1" && r.predecessor)
      for (const auto &q : od.roads) fromStreet += q.id == r.predecessor->elementId && q.length() > 20.0;
  CHECK(fromStreet >= 2);
}

TEST_CASE("junction: a single lane going straight on continues in the lane it lines up with") {
  // a one-lane oneway becomes a three-lane oneway at a side street; both ways are centred on the OSM
  // line, so the lane continues in the middle lane (the added lanes begin on either side of it)
  for (const char *country : {"US", "GB"}) {
    OsmB b;
    const OsmId c = b.node(0, 0);
    b.way({b.node(-200, 0), b.node(-100, 0), c}, {{"highway", "secondary"}, {"oneway", "yes"}, {"lanes", "1"}});
    b.way({c, b.node(100, 0), b.node(200, 0)}, {{"highway", "secondary"}, {"oneway", "yes"}, {"lanes", "3"}});
    b.way({c, b.node(0, -100), b.node(0, -200)}, {{"highway", "residential"}});
    BuildOptions o = opts();
    o.country = CountryProfile::load(country);
    const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
    const ValidationReport v = validate(od, o.target);
    printErrors(v);
    CHECK(v.errors == 0);
    const odr::Road *in = roadNear(od, -50, 0), *out = roadNear(od, 50, 0);
    CHECK(in && out);
    if (!in || !out) continue;
    int straight = 0;
    for (const auto &r : od.roads) {
      if (r.junction == "-1" || !r.predecessor || !r.successor || r.predecessor->elementId != in->id || r.successor->elementId != out->id)
        continue;
      ++straight;
      const int lane = r.lanes.front().left.empty() ? r.lanes.front().right.front().id : r.lanes.front().left.front().id;
      Pose p0, p1;
      CHECK(odr::laneCenter(r, 0.0, lane, p0) && odr::laneCenter(r, r.length(), lane, p1));
      CHECK(std::abs(p1.y - p0.y) < 0.5);  // no jog sideways into the outer lane
    }
    CHECK(straight == 1);
  }
}
