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
  int conn = 0, toRamp = 0;
  const odr::Road *ramp = roadNear(od, 300, -35);
  for (const auto &r : od.roads) {
    if (r.junction == "-1") continue;
    ++conn;
    CHECK(r.length() < 10.0);
    if (r.successor && ramp && r.successor->elementId == ramp->id) ++toRamp;
  }
  CHECK(conn == 4);   // 3 main lanes straight on, 1 branch lane into the ramp
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
