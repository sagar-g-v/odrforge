// Elevation, junction surfaces, grade separation, overlap resolution and clipping.
// Spec basis: spec 3 (elevation absolute s; superelevation roll, positive falls to the right),
// pitfall 11 (C1 elevation, matching z at links), plus the visual quality rules in validate().
#include <cstdio>

#include "odrforge/network.hpp"
#include "odrforge/terrain.hpp"
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
    w.id = 5000 + next++;
    w.nodes = std::move(nodes);
    w.tags = std::move(t);
    d.ways[w.id] = w;
    return w.id;
  }
};

TerrainGrid makeGrid(double x0, double y0, double cell, int nx, int ny, int levels,
                     const std::function<float(int, double, double)> &f) {
  TerrainGrid g;
  g.originX = x0, g.originY = y0, g.cell = cell, g.nx = nx, g.ny = ny, g.levels = levels, g.datum = "TEST";
  g.z.resize(static_cast<size_t>(levels) * nx * ny);
  for (int l = 0; l < levels; ++l)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) g.z[(static_cast<size_t>(l) * ny + j) * nx + i] = f(l, x0 + i * cell, y0 + j * cell);
  return g;
}

BuildOptions baseOptions() {
  BuildOptions o;
  o.country = CountryProfile::load("US");
  o.target = TargetProfile::byName("asam");
  o.streetFurniture = false;
  return o;
}

void printErrors(const ValidationReport &v) {
  for (size_t i = 0; i < v.messages.size() && i < 8; ++i) std::printf("    %s\n", v.messages[i].c_str());
}

}  // namespace

TEST_CASE("terrain grid: v1/v2 round trip, bilinear sampling, stacked levels") {
  const auto g = makeGrid(-10, -10, 0.5, 41, 41, 2, [](int l, double x, double y) {
    return l == 0 ? static_cast<float>(10 + 0.1 * x + 0.05 * y) : (std::abs(x) < 3 ? 20.0f : NAN);
  });
  g.save("odrforge_terrain_test.odrz");
  const TerrainGrid r = TerrainGrid::load("odrforge_terrain_test.odrz");
  std::remove("odrforge_terrain_test.odrz");
  CHECK(r.levels == 2 && r.nx == 41 && r.datum == "TEST");
  CHECK_NEAR(r.sample(1.23, -2.71), 10 + 0.1 * 1.23 + 0.05 * -2.71, 1e-5);  // bilinear is exact on a plane
  const auto lv = r.levelsAt(0.4, 0.0);
  CHECK(lv.size() == 2 && std::abs(lv[1] - 20.0) < 1e-6);
  CHECK(r.levelsAt(8.0, 0.0).size() == 1);
}

TEST_CASE("elevation: roads follow a sloped terrain and share one surface per junction") {
  OsmB b;
  const OsmId c = b.node(0, 0);
  const Tags street{{"highway", "secondary"}, {"lanes", "2"}};
  b.way({b.node(-150, 0), b.node(-75, 0), c}, street);
  b.way({c, b.node(75, 0), b.node(150, 0)}, street);
  b.way({b.node(0, -150), b.node(0, -75), c}, street);
  b.way({c, b.node(0, 75), b.node(0, 150)}, street);
  Terrain t;
  t.ground = makeGrid(-200, -200, 1.0, 401, 401, 1, [](int, double x, double y) { return static_cast<float>(100 + 0.06 * x + 0.03 * y); });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(v.count("quality.junction_surface") == 0);
  CHECK(v.maxLinkZGap < 0.01);
  // every driving-lane centre lies on the terrain plane (superelevation included)
  double worst = 0;
  for (const auto &r : od.roads)
    for (double s = 0; s <= r.length(); s += 3.0) {
      const auto &sec = odr::sectionAt(r, s);
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) {
          if (l.type != "driving") continue;
          Pose p;
          double a, bb;
          if (!odr::laneCenter(r, s, l.id, p) || !odr::laneBorders(r, s, l.id, a, bb)) continue;
          worst = std::max(worst, std::abs(odr::surfaceZ(r, s, 0.5 * (a + bb)) - (100 + 0.06 * p.x + 0.03 * p.y)));
        }
    }
  CHECK(worst < 0.03);
}

TEST_CASE("elevation: a bridge follows its measured deck, the road below stays on the ground") {
  OsmB b;
  // east-west street on the ground, north-south bridge over it (no shared node)
  b.way({b.node(-150, 0), b.node(150, 0)}, {{"highway", "secondary"}, {"lanes", "2"}});
  const OsmId s0 = b.node(0, -150), s1 = b.node(0, -40), s2 = b.node(0, 40), s3 = b.node(0, 150);
  b.way({s0, s1}, {{"highway", "secondary"}, {"lanes", "2"}});
  b.way({s1, s2}, {{"highway", "secondary"}, {"lanes", "2"}, {"bridge", "yes"}, {"layer", "1"}});
  b.way({s2, s3}, {{"highway", "secondary"}, {"lanes", "2"}});
  Terrain t;
  t.ground = makeGrid(-200, -200, 1.0, 401, 401, 1, [](int, double, double) { return 50.0f; });
  // deck: 7 m above ground across the street, ramping down to the ground at the abutments
  t.structures = makeGrid(-200, -200, 1.0, 401, 401, 1, [](int, double x, double y) {
    if (std::abs(x) > 6 || std::abs(y) > 40) return NAN;
    return static_cast<float>(50 + 7.0 * std::min(1.0, (40 - std::abs(y)) / 25.0));
  });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.count("quality.road_overlap") == 0);
  double zBridge = -1, zStreet = -1;
  for (const auto &r : od.roads) {
    if (r.junction != "-1") continue;
    double s, tt;
    r.planView.project({0, 0}, s, tt);
    if (std::abs(tt) > 3 || s <= 0.5 || s >= r.length() - 0.5) continue;
    const double z = odr::elevationAt(r, s);
    if (!r.bridges.empty()) zBridge = z;
    else zStreet = z;
  }
  CHECK_NEAR(zStreet, 50.0, 0.05);
  CHECK_NEAR(zBridge, 57.0, 0.3);
}

TEST_CASE("overlap resolution: carriageways drawn too close are pushed apart") {
  OsmB b;
  // dual carriageway drawn 6 m apart, 3 lanes each (needs ~10.5 m per carriageway)
  b.way({b.node(-200, 3), b.node(200, 3)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}});
  b.way({b.node(200, -3), b.node(-200, -3)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}});
  BuildOptions o = baseOptions();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(rep.overlapShifts >= 2);
  CHECK(v.count("quality.road_overlap") == 0);
  CHECK(v.errors == 0);
  // the shifted laneOffset joins its pieces with equal value, slope and second derivative (lane edges G2,
  // Annex D.1.3): C1 pieces with finite-difference slopes stepped the curvature at every knot
  int joins = 0;
  for (const auto &r : od.roads)
    for (size_t i = 1; i < r.laneOffset.size(); ++i) {
      const odr::Poly3 &p = r.laneOffset[i - 1], &q = r.laneOffset[i];
      const double h = q.s - p.s;
      ++joins;
      CHECK_NEAR(p.eval(h), q.a, 1e-9);
      CHECK_NEAR(p.deriv(h), q.b, 1e-9);
      CHECK_NEAR(2.0 * p.c + 6.0 * p.d * h, 2.0 * q.c, 1e-7);
    }
  CHECK(joins > 0);
}

TEST_CASE("overlap resolution: a realignment does not run through a lane drop") {
  // a three-lane oneway drops to two lanes 40 m before the opposite carriageway, drawn too close,
  // pushes it aside: the shift ramps in after the drop's taper; through the drop the lanes kept move
  // only by the re-centring on the way (half the dropped lane), smoothly
  OsmB b;
  const OsmId m = b.node(-40, 3);
  b.way({b.node(-300, 3), m}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}});
  b.way({m, b.node(300, 3)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}});
  b.way({b.node(300, -5), b.node(0, -5), b.node(-60, -12), b.node(-300, -12)}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}});
  BuildOptions o = baseOptions();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(rep.overlapShifts >= 1);
  bool found = false;
  for (const auto &r : od.roads) {
    if (r.junction != "-1" || r.lanes.size() != 2) continue;
    found = true;
    const double sb = r.lanes[1].s;
    double dropped = 0.0;  // width of the lane that ends
    for (const auto &l : r.lanes[0].right)
      if (!l.link.successor && !l.widths.empty()) dropped = std::max(dropped, l.widths.front().a);
    double lo = 1e9, hi = -1e9, prev = 0.0, worstStep = 0.0;
    for (double s = std::max(0.0, sb - 40.0); s <= std::min(r.length(), sb + 10.0); s += 0.5) {
      double a, c;
      CHECK(odr::laneBorders(r, s, -1, a, c));
      const double m = 0.5 * (a + c);
      if (lo < 1e9) worstStep = std::max(worstStep, std::abs(m - prev));
      lo = std::min(lo, m), hi = std::max(hi, m), prev = m;
    }
    CHECK(dropped > 2.0);
    CHECK(hi - lo < 0.5 * dropped + 0.2);  // the re-centring only, no overlap shift through the drop
    CHECK(worstStep < 0.1);                // and smoothly (0.5 m steps)
  }
  CHECK(found);
}

TEST_CASE("clip: ways are cut exactly at the area boundary") {
  OsmB b;
  b.way({b.node(-300, 0), b.node(300, 0)}, {{"highway", "residential"}});
  BuildOptions o = baseOptions();
  o.clipBox = std::array<double, 4>{-100, -50, 100, 50};
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  CHECK(od.roads.size() == 1);
  CHECK_NEAR(od.roads[0].length(), 200.0, 0.05);
  CHECK_NEAR(od.roads[0].planView.eval(0).x, -100.0, 0.05);
}

TEST_CASE("elevation: a covered way under a hill follows its portals, not the hill top") {
  OsmB b;
  const OsmId a0 = b.node(-150, 0), a1 = b.node(-40, 0), a2 = b.node(40, 0), a3 = b.node(150, 0);
  const Tags street{{"highway", "residential"}};
  b.way({a0, a1}, street);
  Tags cov = street;
  cov["covered"] = "yes";
  b.way({a1, a2}, cov);
  b.way({a2, a3}, street);
  Terrain t;
  // flat ground at 20 m with a 15 m high block over the covered stretch
  t.ground = makeGrid(-200, -50, 1.0, 401, 101, 1, [](int, double x, double y) {
    return static_cast<float>(std::abs(x) < 38 && std::abs(y) < 30 ? 35.0 : 20.0);
  });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  double worst = 0;
  for (const auto &r : od.roads)
    for (double s = 0; s <= r.length(); s += 2.0) worst = std::max(worst, std::abs(odr::elevationAt(r, s) - 20.0));
  CHECK(worst < 0.05);
}

TEST_CASE("junction surface: one road end on a wall top conforms instead of tilting the junction") {
  OsmB b;
  const OsmId c = b.node(0, 0);
  const Tags major{{"highway", "primary"}, {"lanes", "2"}};
  b.way({b.node(-150, 0), b.node(-60, 0), c}, major);
  b.way({c, b.node(60, 0), b.node(150, 0)}, major);
  b.way({c, b.node(0, 30), b.node(0, 150)}, {{"highway", "service"}});
  Terrain t;
  // street level 50 m; north of y = 8 a terrace 6 m higher (the service road arrives on it)
  t.ground = makeGrid(-200, -200, 1.0, 401, 401, 1, [](int, double, double y) { return static_cast<float>(y > 8 ? 56.0 : 50.0); });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.count("quality.junction_surface") == 0);
  CHECK(v.maxLinkZGap < 0.01);
  // the junction keeps the major street's level and is not tilted toward the terrace
  for (const auto &r : od.roads) {
    if (r.junction == "-1") continue;
    for (double s = 0; s <= r.length(); s += 1.0) CHECK_NEAR(odr::elevationAt(r, s), 50.0, 0.1);
  }
  // and the major street stays on its measured level
  for (const auto &r : od.roads)
    if (r.junction == "-1" && std::abs(r.planView.eval(0).y) < 1 && std::abs(r.planView.eval(r.length()).y) < 1)
      for (double s = 0; s <= r.length(); s += 2.0) CHECK_NEAR(odr::elevationAt(r, s), 50.0, 0.05);
}

TEST_CASE("flat cross sections: no superelevation, level junctions, exact joins") {
  OsmB b;
  const OsmId c = b.node(0, 0);
  const Tags street{{"highway", "secondary"}, {"lanes", "4"}};
  b.way({b.node(-150, 0), c}, street);
  b.way({c, b.node(150, 0)}, street);
  b.way({b.node(0, -150), c}, street);
  b.way({c, b.node(0, 150)}, street);
  Terrain t;
  t.ground = makeGrid(-200, -200, 1.0, 401, 401, 1, [](int, double x, double y) { return static_cast<float>(100 + 0.05 * x + 0.02 * y); });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  o.target.superelevation = false;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  for (const auto &r : od.roads) CHECK(r.superelevation.empty());
  CHECK(v.errors == 0);
  CHECK(v.count("quality.junction_surface") == 0);
  CHECK(v.maxLinkZGap < 0.01);
}

TEST_CASE("grade separation: untagged crossing without a junction - the lower road goes under") {
  OsmB b;
  // a freeway on an embankment (ground measured on it) crossing a street; no bridge / layer tags
  b.way({b.node(-200, 0), b.node(200, 0)}, {{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}});
  b.way({b.node(0, -200), b.node(0, 200)}, {{"highway", "residential"}});
  Terrain t;
  t.ground = makeGrid(-250, -250, 1.0, 501, 501, 1, [](int, double, double y) {
    return static_cast<float>(std::abs(y) < 14 ? 37.0 : 30.0);  // the embankment carries the freeway
  });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.count("quality.road_overlap") == 0);
  double zHi = 0, zLo = 0;
  for (const auto &r : od.roads) {
    double s, tt;
    r.planView.project({0, 0}, s, tt);
    if (std::abs(tt) > 3) continue;
    (r.lanes.front().right.size() + r.lanes.front().left.size() >= 3 ? zHi : zLo) = odr::elevationAt(r, s);
  }
  CHECK(zHi - zLo >= 4.5);
}

TEST_CASE("elevation: a road measured at a single point stays level (no extrapolated slope)") {
  OsmB b;
  const OsmId c = b.node(0, 0);
  const Tags street{{"highway", "residential"}};
  b.way({b.node(-120, 0), c}, street);
  b.way({c, b.node(120, 0)}, street);
  b.way({c, b.node(0, 60)}, {{"highway", "service"}, {"tunnel", "yes"}});
  Terrain t;
  // ground at 40 m, measured only on a 3 m strip at the far end of the tunnel road
  t.ground = makeGrid(-150, -150, 1.0, 301, 301, 1, [](int, double x, double y) {
    return std::abs(x) < 40 || y < 10 || y > 58 ? 40.0f : NAN;
  });
  BuildOptions o = baseOptions();
  o.terrain = &t;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  for (const auto &r : od.roads)
    for (double s = 0; s <= r.length(); s += 1.0) CHECK_NEAR(odr::elevationAt(r, s), 40.0, 0.05);
}

namespace {

/// height of the plain road picked by `pick` whose reference line passes within 3 m of (x, y); NaN if none
double zThrough(const odr::OpenDrive &od, double x, double y, const std::function<bool(const odr::Road &)> &pick,
                bool junctionRoadsToo = false) {
  for (const auto &r : od.roads) {
    if ((r.junction != "-1" && !junctionRoadsToo) || !pick(r)) continue;
    double s, t;
    r.planView.project({x, y}, s, t);
    if (std::abs(t) < 3.0 && s > 0.5 && s < r.length() - 0.5) return odr::elevationAt(r, s);
  }
  return NAN;
}

/// steepest grade over 1 m of any road, and the largest height of a plain road farther than `far`
/// from the origin
void gradeAndFar(const odr::OpenDrive &od, double far, double &grade, double &zFar) {
  grade = zFar = 0.0;
  for (const auto &r : od.roads)
    for (double q = 0; q + 1.0 <= r.length(); q += 1.0) {
      grade = std::max(grade, std::abs(odr::elevationAt(r, q + 1.0) - odr::elevationAt(r, q)));
      const Pose p = r.planView.eval(q);
      if (r.junction == "-1" && std::hypot(p.x, p.y) > far) zFar = std::max(zFar, std::abs(odr::elevationAt(r, q)));
    }
}

bool isStreet(const odr::Road &r) { return r.bridges.empty() && r.tunnels.empty(); }
bool isBridge(const odr::Road &r) { return !r.bridges.empty(); }
bool isTunnel(const odr::Road &r) { return !r.tunnels.empty(); }
/// the road starting on the line x = x0 (north-south) or y = y0 (east-west)
std::function<bool(const odr::Road &)> startsAtX(double x0) {
  return [x0](const odr::Road &r) { return std::abs(r.planView.eval(0).x - x0) < 1.0; };
}
std::function<bool(const odr::Road &)> startsAtY(double y0) {
  return [y0](const odr::Road &r) { return std::abs(r.planView.eval(0).y - y0) < 1.0; };
}

}  // namespace

TEST_CASE("grade separation without elevation data: a bridge rises one storey over the street") {
  OsmB b;
  b.way({b.node(-300, 0), b.node(300, 0)}, {{"highway", "residential"}});
  const OsmId s0 = b.node(0, -300), s1 = b.node(0, -40), s2 = b.node(0, 40), s3 = b.node(0, 300);
  const Tags road{{"highway", "secondary"}, {"lanes", "2"}};
  Tags bridge = road;
  bridge["bridge"] = "yes", bridge["layer"] = "1";
  b.way({s0, s1}, road);
  b.way({s1, s2}, bridge);
  b.way({s2, s3}, road);
  BuildOptions o = baseOptions();  // no terrain: flat ground, the OSM layers are the only evidence
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(v.count("quality.road_overlap") == 0 && v.count("quality.max_grade") == 0);
  CHECK_NEAR(zThrough(od, 0, 0, isStreet), 0.0, 1e-6);
  CHECK_NEAR(zThrough(od, 0, 0, isBridge), o.levelHeight, 0.05);
  double grade, zFar;
  gradeAndFar(od, 200.0, grade, zFar);
  CHECK(grade < 0.062);  // 6 % design grade of a secondary road, at the steepest point of the ramp
  CHECK(zFar < 1e-6);  // away from the crossing the road lies on the ground
}

TEST_CASE("grade separation without elevation data: a tunnel sinks one storey, a bridge over nothing stays level") {
  OsmB b;
  b.way({b.node(-300, 0), b.node(300, 0)}, {{"highway", "residential"}});
  const Tags road{{"highway", "secondary"}};
  Tags tunnel = road, bridge = road;
  tunnel["tunnel"] = "yes", tunnel["layer"] = "-1";
  bridge["bridge"] = "yes";
  const OsmId t0 = b.node(0, -300), t1 = b.node(0, -50), t2 = b.node(0, 50), t3 = b.node(0, 300);
  b.way({t0, t1}, road);
  b.way({t1, t2}, tunnel);
  b.way({t2, t3}, road);
  // a bridge over a river (no road beneath) further north
  const OsmId r0 = b.node(-300, 500), r1 = b.node(-30, 500), r2 = b.node(30, 500), r3 = b.node(300, 500);
  b.way({r0, r1}, road);
  b.way({r1, r2}, bridge);
  b.way({r2, r3}, road);
  BuildOptions o = baseOptions();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK_NEAR(zThrough(od, 0, 0, isStreet), 0.0, 1e-6);
  CHECK_NEAR(zThrough(od, 0, 0, isTunnel), -o.levelHeight, 0.05);
  for (double x = -250; x <= 250; x += 10) CHECK_NEAR(zThrough(od, x, 500, startsAtY(500)), 0.0, 1e-6);
}

TEST_CASE("grade separation without elevation data: a bridge over a tunnel only stays on the ground") {
  OsmB b;
  b.way({b.node(-300, 0), b.node(300, 0)}, {{"highway", "secondary"}, {"tunnel", "yes"}, {"layer", "-1"}});
  b.way({b.node(0, -300), b.node(0, 300)}, {{"highway", "secondary"}, {"bridge", "yes"}, {"layer", "1"}});
  BuildOptions o = baseOptions();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  // one storey separates them: the tunnel goes under, the bridge needs no ramps
  CHECK_NEAR(zThrough(od, 0, 0, isBridge), 0.0, 1e-6);
  CHECK_NEAR(zThrough(od, 0, 0, isTunnel), -o.levelHeight, 0.05);
}

TEST_CASE("grade separation without elevation data: garage decks are a storey of their signed headroom apart") {
  OsmB b;
  b.way({b.node(-200, 0), b.node(200, 0)}, {{"highway", "residential"}});
  const Tags aisle{{"highway", "service"}, {"tunnel", "yes"}, {"maxheight", "2.1"}};
  Tags upper = aisle, lower = aisle;
  upper["layer"] = "-1", lower["layer"] = "-2";
  b.way({b.node(0, -150), b.node(0, 150)}, upper);    // under the street
  b.way({b.node(-150, 30), b.node(150, 30)}, lower);  // under the upper deck only
  BuildOptions o = baseOptions();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.count("quality.road_overlap") == 0);
  CHECK_NEAR(zThrough(od, 0, 0, startsAtX(0)), -3.1, 0.05);    // 2.1 m headroom + 1 m slab under the street
  CHECK_NEAR(zThrough(od, 0, 30, startsAtX(0)), -3.1, 0.05);   // one deck
  CHECK_NEAR(zThrough(od, 0, 30, startsAtY(30)), -6.2, 0.05);  // a storey further down
}

TEST_CASE("grade separation without elevation data: structures too close for a ramp take the street beneath into a cutting") {
  OsmB b;
  // a street passes under a bridge at y = 0 and, 40 m on, over another street on its own bridge: on
  // flat ground a 6 m climb in 30 m. The upper bridge comes down and the street beneath it dips.
  const Tags street{{"highway", "residential"}};
  Tags bridge = street;
  bridge["bridge"] = "yes", bridge["layer"] = "1";
  const OsmId x0 = b.node(-300, 0), x1 = b.node(-40, 0), x2 = b.node(40, 0), x3 = b.node(300, 0);
  b.way({x0, x1}, street);
  b.way({x1, x2}, bridge);
  b.way({x2, x3}, street);
  const OsmId r0 = b.node(0, -300), r1 = b.node(0, 20), r2 = b.node(0, 60), r3 = b.node(0, 300);
  b.way({r0, r1}, street);
  b.way({r1, r2}, bridge);
  b.way({r2, r3}, street);
  b.way({b.node(-300, 40), b.node(300, 40)}, street);
  BuildOptions o = baseOptions();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(v.count("quality.road_overlap") == 0 && v.count("quality.max_grade") == 0);
  const double zUnder = zThrough(od, 0, 0, startsAtX(0)), zOver = zThrough(od, 0, 40, startsAtX(0));
  CHECK(zThrough(od, 0, 0, startsAtY(0)) - zUnder >= o.levelHeight - 0.05);
  CHECK(zOver - zThrough(od, 0, 40, startsAtY(40)) >= o.levelHeight - 0.05);
  CHECK(zThrough(od, 0, 40, startsAtY(40)) < -1.0);  // the street dips under the lowered bridge
  double grade, zFar;
  gradeAndFar(od, 250.0, grade, zFar);
  CHECK(grade < 0.122);  // the steepest a residential ramp is built: 12 %
  CHECK(zFar < 1e-6);
}

TEST_CASE("grade separation without elevation data: a street under two lowered bridges stays down between them") {
  OsmB b;
  // two roads bridge the street y = 40 at x = 0 and x = 20; each passes under a short bridge before
  // (40 m and 60 m before), so each comes down to a different level and the street dips under both
  // into one cutting - not up and down between them
  const Tags street{{"highway", "residential"}};
  Tags bridge = street;
  bridge["bridge"] = "yes", bridge["layer"] = "1";
  b.way({b.node(-300, 40), b.node(300, 40)}, street);
  for (const double x : {0.0, 20.0}) {
    const OsmId r0 = b.node(x, -300), r1 = b.node(x, 20), r2 = b.node(x, 60), r3 = b.node(x, 300);
    b.way({r0, r1}, street);
    b.way({r1, r2}, bridge);
    b.way({r2, r3}, street);
  }
  b.way({b.node(-300, 0), b.node(-8, 0)}, street);
  b.way({b.node(-8, 0), b.node(8, 0), b.node(10, 0)}, bridge);
  b.way({b.node(12, -20), b.node(14, -20), b.node(26, -20)}, bridge);
  b.way({b.node(26, -20), b.node(300, -20)}, street);
  BuildOptions o = baseOptions();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.count("quality.road_overlap") == 0 && v.count("quality.max_grade") == 0);
  const double z0 = zThrough(od, 0, 40, startsAtY(40)), z20 = zThrough(od, 20, 40, startsAtY(40));
  CHECK(zThrough(od, 0, 40, startsAtX(0)) - z0 >= o.levelHeight - 0.05);
  CHECK(zThrough(od, 20, 40, startsAtX(20)) - z20 >= o.levelHeight - 0.05);
  double grade, zFar;
  gradeAndFar(od, 250.0, grade, zFar);
  CHECK(grade < 0.122);
  CHECK(zFar < 1e-6);
}

TEST_CASE("grade separation without elevation data: structures closer than any ramp keep a bounded level") {
  // a bridge ends at a junction a few metres past the street it crosses, and a short road joins it
  // to that street (a bridge head with a ramp down to a quay road): no ramp climbs a storey there,
  // the level constraints are contradictory. The storey shrinks (reported); nothing sinks away.
  for (const auto &[da, db] : {std::pair<double, double>{20, 15}, {25, 25}, {40, 10}}) {
    OsmB b;
    const Tags street{{"highway", "residential"}};
    Tags bridge = street;
    bridge["bridge"] = "yes", bridge["layer"] = "1";
    const OsmId ja = b.node(da, 0), jb = b.node(0, db);
    b.way({b.node(-300, 0), b.node(-150, 0), ja}, bridge);
    b.way({ja, b.node(150, 0), b.node(300, 0)}, street);
    b.way({b.node(0, -300), b.node(0, -150), jb}, street);
    b.way({jb, b.node(0, 150), b.node(0, 300)}, street);
    b.way({jb, ja}, street);
    BuildOptions o = baseOptions();
    BuildReport rep;
    const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
    const ValidationReport v = validate(od, o.target);
    double grade, zFar, zMax = 0.0;
    gradeAndFar(od, 250.0, grade, zFar);
    for (const auto &r : od.roads)
      for (double s = 0; s <= r.length(); s += 1.0) zMax = std::max(zMax, std::abs(odr::elevationAt(r, s)));
    CHECK(zMax <= o.levelHeight + 1e-6);
    CHECK(grade < 0.122);
    CHECK(zFar < 1e-6);
    CHECK(v.count("quality.max_grade") == 0);
    // still above the street (which may run through the junction's curb return under the bridge)
    CHECK(zThrough(od, 0, 0, isBridge) >= zThrough(od, 0, 0, isStreet, true) - 1e-6);
    bool reported = false;
    for (const auto &w : rep.warnings) reported |= w.find("storey reduced") != std::string::npos;
    CHECK(reported);
  }
}

TEST_CASE("grade separation without elevation data: an elevated motorway with short ramps over a viaduct stays up") {
  // a motorway on a structure (layer 3) crosses two streets and, above them, a trunk road on a viaduct
  // (layer 2). Its exit and entrance ramps are short and untagged, reaching the streets right next to
  // where the motorway crosses them: no ramp climbs from the ground to the motorway at its design
  // grade. The junctions at the foot of the ramps rise and the storey shrinks only there, instead of
  // the motorway - and the viaduct and the streets beneath it - coming down: no structure below the
  // ground, no street in a trench.
  OsmB b;
  const Tags street{{"highway", "residential"}};
  const OsmId sw = b.node(-300, -60), nw = b.node(-300, 45), se = b.node(300, -60), ne = b.node(300, 45);
  b.way({b.node(-300, -400), sw, b.node(-300, 0), nw, b.node(-300, 400)}, street);
  b.way({b.node(300, -400), se, b.node(300, 0), ne, b.node(300, 400)}, street);
  b.way({b.node(-800, -60), sw, b.node(0, -60), se, b.node(800, -60)}, street);
  b.way({b.node(-800, 45), nw, b.node(0, 45), ne, b.node(800, 45)}, street);
  const Tags trunk{{"highway", "trunk"}, {"lanes", "2"}};
  Tags viaduct = trunk;
  viaduct["bridge"] = "viaduct", viaduct["layer"] = "2";
  const OsmId v0 = b.node(0, -150), v1 = b.node(0, 150);
  b.way({b.node(0, -800), v0}, trunk);
  b.way({v0, b.node(0, 0), v1}, viaduct);
  b.way({v1, b.node(0, 800)}, trunk);
  const Tags motorway{{"highway", "motorway"}, {"oneway", "yes"}, {"lanes", "2"}, {"bridge", "yes"}, {"layer", "3"}};
  const Tags link{{"highway", "motorway_link"}, {"oneway", "yes"}};
  const OsmId m1 = b.node(-270, 0), m2 = b.node(270, 0);
  b.way({b.node(-800, 0), m1}, motorway);
  b.way({m1, b.node(0, 0), m2}, motorway);
  b.way({m2, b.node(800, 0)}, motorway);
  b.way({m1, b.node(-285, 25), nw}, link);  // exit
  b.way({ne, b.node(285, 25), m2}, link);   // entrance
  BuildOptions o = baseOptions();
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK(v.count("quality.road_overlap") == 0 && v.count("quality.max_grade") == 0);
  // the order of the layers holds a storey apart: motorway over viaduct over street; only where the
  // motorway crosses a street right next to a ramp is the storey lower (reported)
  const double zStreet = zThrough(od, 0, 45, startsAtY(45)), zViaduct = zThrough(od, 0, 45, startsAtX(0));
  CHECK_NEAR(zStreet, 0.0, 1e-6);  // the street under the viaduct stays on the ground
  CHECK(zViaduct - zStreet >= o.levelHeight - 0.05);
  CHECK(zThrough(od, 0, 0, startsAtY(0)) - zViaduct >= o.levelHeight - 0.05);
  for (const double x : {-300.0, 300.0}) CHECK(zThrough(od, x, 0, startsAtY(0)) - zThrough(od, x, 0, startsAtX(x)) > 3.0);
  bool reported = false;
  for (const auto &w : rep.warnings) reported |= w.find("storey reduced") != std::string::npos;
  CHECK(reported);
  // no structure below the ground; no street deeper than a storey, none off the ground far away
  double zBridge = 0.0, zDeep = 0.0, zFar = 0.0;
  for (const auto &r : od.roads) {
    if (r.junction != "-1") continue;
    for (const auto &br : r.bridges)
      for (double s = br.s; s <= br.s + br.length; s += 1.0) zBridge = std::min(zBridge, odr::elevationAt(r, s));
    if (!isStreet(r)) continue;
    for (double s = 0; s <= r.length(); s += 1.0) {
      const Pose p = r.planView.eval(s);
      zDeep = std::min(zDeep, odr::elevationAt(r, s));
      if (std::abs(p.x) > 500.0 || std::abs(p.y) > 300.0) zFar = std::max(zFar, std::abs(odr::elevationAt(r, s)));
    }
  }
  CHECK(zBridge >= -0.01);  // (the cubic fit of a ramp foot may dip by millimetres)
  CHECK(zDeep >= -o.levelHeight - 0.01);
  CHECK(zFar < 1e-6);
}

TEST_CASE("grade separation without elevation data: a building passage stays on the ground under a bridge") {
  // tunnel=building_passage runs through a building at ground level (implied layer 0, not -1): a
  // bridge (layer 1) crossing it rises a storey, the passage does not sink
  OsmB b;
  const Tags street{{"highway", "residential"}};
  Tags passage = street;
  passage["tunnel"] = "building_passage";
  const OsmId p0 = b.node(-300, 0), p1 = b.node(-20, 0), p2 = b.node(20, 0), p3 = b.node(300, 0);
  b.way({p0, p1}, street);
  b.way({p1, p2}, passage);
  b.way({p2, p3}, street);
  Tags bridge = street;
  bridge["bridge"] = "yes", bridge["layer"] = "1";
  const OsmId r0 = b.node(0, -300), r1 = b.node(0, -40), r2 = b.node(0, 40), r3 = b.node(0, 300);
  b.way({r0, r1}, street);
  b.way({r1, r2}, bridge);
  b.way({r2, r3}, street);
  BuildOptions o = baseOptions();
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o);
  const ValidationReport v = validate(od, o.target);
  printErrors(v);
  CHECK(v.errors == 0);
  CHECK_NEAR(zThrough(od, 0, 0, startsAtY(0)), 0.0, 1e-6);
  CHECK_NEAR(zThrough(od, 0, 0, startsAtX(0)), o.levelHeight, 0.05);
}
