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
