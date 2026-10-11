// End-to-end OSM -> OpenDRIVE scenarios. Every generated network must pass the full ASAM
// conformance checker (validate) without errors, plus scenario-specific spec expectations.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
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

Result build(const OsmBuilder &b, const std::string &country = "US", const std::string &target = "asam") {
  BuildOptions o;
  o.country = CountryProfile::load(country);
  o.target = TargetProfile::byName(target);
  o.streetFurniture = false;
  Result r;
  r.od = buildOpenDrive(b.d, b.proj, o, &r.rep);
  r.od.header.revMinor = o.target.revMinor;
  r.val = validate(r.od, o.target);
  // for the XSD conformance test: every scenario is also written as OpenDRIVE 1.8
  if (const char *dir = std::getenv("ODRFORGE_TEST_OUT")) {
    static int n = 0;
    odr::OpenDrive copy = r.od;
    copy.header.revMinor = 8;
    odr::write(copy, std::string(dir) + "/scenario_" + std::to_string(++n) + "_" + country + "_" + target + ".xodr", {8, 12});
  }
  if (r.val.errors)
    for (size_t i = 0; i < r.val.messages.size() && i < 10; ++i) std::printf("    %s\n", r.val.messages[i].c_str());
  return r;
}

int count(const odr::OpenDrive &od, bool junctionRoads) {
  return static_cast<int>(std::count_if(od.roads.begin(), od.roads.end(),
                                        [&](const odr::Road &r) { return (r.junction != "-1") == junctionRoads; }));
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
  // 4 approaches x (left, straight, right), one lane each
  CHECK(count(r.od, true) == 12);
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
        CHECK(!s.validity.empty() && s.validity[0].fromLane <= s.validity[0].toLane);
      }
  CHECK(lights == 4);
  // every connecting road has exactly one lane linked at both ends, and the junction lists it
  for (const auto &road : r.od.roads) {
    if (road.junction == "-1") continue;
    const auto &sec = road.lanes.front();
    CHECK(sec.left.size() + sec.right.size() == 1);
    CHECK(road.predecessor && road.successor);
  }
  CHECK(r.val.maxLinkGap < 0.02);
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
  // only right lanes on a RHT oneway, centred on the way through laneOffset
  CHECK(road.lanes[0].left.empty() || road.lanes[0].left.front().type != "driving");
}

TEST_CASE("network: left-hand traffic (GB) - rule LHT, +s lanes on the left") {
  OsmBuilder b = crossroads(true);
  const Result r = build(b, "GB", "asam");
  CHECK(r.val.errors == 0);
  for (const auto &road : r.od.roads) {
    CHECK(road.rule == "LHT");
    if (road.junction == "-1") continue;
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
}
