// Carriageway edges that follow the mapped kerb (kerbs.cpp; user decisions 2026-10-20: the lane that moves stays
// 2 - 5.5 m wide, a shoulder takes what lies beyond, a kerb that would leave a narrower lane is left out).
#include <cmath>
#include <cstdio>

#include "odrforge/network.hpp"
#include "odrforge/terrain.hpp"
#include "odrforge/validate.hpp"
#include "test.hpp"

using namespace odrforge;

namespace {

struct OsmK {
  Projection proj{51.5, -0.1};
  OsmData d;
  OsmId next = 1;
  OsmId node(double x, double y) {
    OsmNode n;
    n.id = next++;
    proj.inverse({x, y}, n.lat, n.lon);
    d.nodes[n.id] = n;
    return n.id;
  }
  OsmId way(std::vector<OsmId> nodes, Tags t) {
    OsmWay w;
    w.id = 7000 + next++;
    w.nodes = std::move(nodes);
    w.tags = std::move(t);
    d.ways[w.id] = w;
    return w.id;
  }
  // a street from x = -100 to 100 along y = 0, and a kerb line along y = yk
  void street(Tags t) { way({node(-100, 0), node(100, 0)}, t); }
  void kerb(double yk, double x0 = -100, double x1 = 100) { way({node(x0, yk), node(x1, yk)}, {{"barrier", "kerb"}}); }
};

BuildOptions options(bool follow = true) {
  BuildOptions o;
  o.country = CountryProfile::load("GB");
  o.target = TargetProfile::byName("asam");
  o.streetFurniture = false;
  o.followKerbs = follow;
  return o;
}

// at x on the street: the y of the carriageway edge on each side (outer border of the outermost lane that is not
// sidewalk / walking / curb / border / none), and the widths of the left driving lane and of a left shoulder
struct Edges {
  double left = NAN, right = NAN, laneLeft = NAN, shoulderLeft = 0.0;
};
Edges edgesAt(const odr::OpenDrive &od, double x) {
  Edges e;
  for (const odr::Road &r : od.roads) {
    if (r.junction != "-1") continue;
    double s, t;
    r.planView.project({x, 0.0}, s, t);
    if (std::abs(t) > 1.0 || s <= 0.5 || s >= r.length() - 0.5) continue;
    const Pose p = r.planView.eval(s);
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    for (const bool left : {true, false}) {
      int outer = 0;
      for (const odr::Lane &l : left ? sec.left : sec.right) {
        double in, out;
        if (!odr::laneBorders(r, s, l.id, in, out)) continue;
        if (left && l.type == "driving") e.laneLeft = std::abs(out - in);
        if (left && l.type == "shoulder") e.shoulderLeft = std::abs(out - in);
        if (l.type == "sidewalk" || l.type == "walking" || l.type == "curb" || l.type == "border" || l.type == "none") continue;
        if (std::abs(l.id) > std::abs(outer)) outer = l.id;
      }
      double in, out = odr::laneOffsetAt(r, s);
      if (outer != 0) odr::laneBorders(r, s, outer, in, out);
      (left ? e.left : e.right) = p.offset(out).y;
    }
  }
  return e;
}

}  // namespace

TEST_CASE("kerb edges: a street's carriageway edges lie on the kerb lines mapped beside it") {
  OsmK b;
  b.street({{"highway", "residential"}, {"lanes", "2"}});
  b.kerb(4.0), b.kerb(-4.0);
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, options(), &rep);
  const Edges e = edgesAt(od, 0.0);
  CHECK_NEAR(e.left, 4.0, 0.02);
  CHECK_NEAR(e.right, -4.0, 0.02);
  CHECK(rep.kerbSides == 2 && rep.kerbSamples > 0);
  CHECK(validate(od, options().target).errors == 0);
}

TEST_CASE("kerb edges: a kerb that would leave a lane under 2 m is left out (an island's or another road's)") {
  OsmK b;
  b.street({{"highway", "residential"}, {"lanes", "2"}});
  b.kerb(1.0);  // inside the left lane: it would be 1 m wide
  BuildReport rep;
  const Edges kerbed = edgesAt(buildOpenDrive(b.d, b.proj, options(), &rep), 0.0);
  const Edges plain = edgesAt(buildOpenDrive(b.d, b.proj, options(false)), 0.0);
  CHECK_NEAR(kerbed.left, plain.left, 1e-6);
  CHECK(rep.kerbSamples == 0 && rep.kerbSamplesIgnored > 0);
}

TEST_CASE("kerb edges: beyond a 5.5 m lane the kerb is reached by a shoulder") {
  OsmK b;
  b.street({{"highway", "residential"}, {"lanes", "2"}});
  b.kerb(8.0), b.kerb(-4.0);
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, options());
  const Edges e = edgesAt(od, 0.0);
  CHECK_NEAR(e.left, 8.0, 0.02);
  CHECK_NEAR(e.laneLeft, 5.5, 0.02);
  CHECK_NEAR(e.shoulderLeft, 2.5, 0.02);
  CHECK_NEAR(e.right, -4.0, 0.02);
  CHECK(validate(od, options().target).errors == 0);
}

TEST_CASE("kerb edges: a kerb ringing a traffic island is not a carriageway edge") {
  OsmK b;
  b.street({{"highway", "residential"}, {"lanes", "2"}});
  const OsmId a = b.node(-50, 3.5), c = b.node(50, 3.5), d = b.node(50, 6.0), f = b.node(-50, 6.0);
  b.way({a, c, d, f, a}, {{"area:highway", "traffic_island"}});
  b.kerb(3.6, -50, 50);  // along the island's edge
  BuildReport rep;
  const Edges kerbed = edgesAt(buildOpenDrive(b.d, b.proj, options(), &rep), 0.0);
  const Edges plain = edgesAt(buildOpenDrive(b.d, b.proj, options(false)), 0.0);
  CHECK_NEAR(kerbed.left, plain.left, 1e-6);
  CHECK(rep.kerbSides == 0);
}

TEST_CASE("kerb edges: a kerb beyond another road's lanes is that road's, not this one's") {
  // a parallel street 11 m north: its lanes (3 m) and sidewalk (2 m) reach down to about y = 6; this street's sidewalk
  // must stay short of them, so its edge may move out to about 4 m only - the kerb at 5 m is not this street's
  OsmK b;
  b.street({{"highway", "residential"}, {"lanes", "2"}});
  b.way({b.node(-100, 11), b.node(100, 11)}, {{"highway", "residential"}, {"lanes", "2"}});
  b.kerb(5.0);
  const Edges kerbed = edgesAt(buildOpenDrive(b.d, b.proj, options()), 0.0);
  const Edges plain = edgesAt(buildOpenDrive(b.d, b.proj, options(false)), 0.0);
  CHECK_NEAR(kerbed.left, plain.left, 1e-6);
}

TEST_CASE("kerb edges: a kerb on another level (measured ground 1 m below the edge's) is not followed") {
  OsmK b;
  b.street({{"highway", "residential"}, {"lanes", "2"}});
  b.kerb(4.5);
  Terrain t;  // a 1 m drop from y = 4 northwards: the kerb stands at the foot of a wall
  t.ground.originX = -200, t.ground.originY = -200, t.ground.cell = 0.5, t.ground.nx = 801, t.ground.ny = 801, t.ground.levels = 1;
  t.ground.datum = "TEST";
  t.ground.z.resize(801 * 801);
  for (int j = 0; j < 801; ++j)
    for (int i = 0; i < 801; ++i) t.ground.z[static_cast<size_t>(j) * 801 + i] = -200 + j * 0.5 > 4.0 ? 9.0f : 10.0f;
  BuildOptions o = options();
  o.terrain = &t;
  BuildReport rep;
  const Edges kerbed = edgesAt(buildOpenDrive(b.d, b.proj, o, &rep), 0.0);
  BuildOptions p = options(false);
  p.terrain = &t;
  const Edges plain = edgesAt(buildOpenDrive(b.d, b.proj, p), 0.0);
  CHECK_NEAR(kerbed.left, plain.left, 1e-6);
  CHECK(rep.kerbSamplesOffLevel > 0);
}

TEST_CASE("kerb edges: a oneway's lanes and lane offset reach its kerbs on both sides") {
  OsmK b;
  b.street({{"highway", "residential"}, {"oneway", "yes"}, {"lanes", "1"}});
  b.kerb(2.5), b.kerb(-2.0);  // 4.5 m kerb to kerb, off the way's centre
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, options());
  const Edges e = edgesAt(od, 0.0);
  CHECK_NEAR(e.left, 2.5, 0.02);
  CHECK_NEAR(e.right, -2.0, 0.02);
  CHECK(validate(od, options().target).errors == 0);
}
