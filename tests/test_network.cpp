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
