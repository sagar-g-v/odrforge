#include "odrforge/osm.hpp"
#include "odrforge/projection.hpp"
#include "test.hpp"

using namespace odrforge;

TEST_CASE("tmerc matches PROJ") {
  // reference values from PROJ (pyproj) for the same +proj=tmerc definition
  const double ref[4][4] = {{34.05, -118.25, 0.000000, 0.000000},
                            {34.06, -118.24, 923.197810, 1109.278919},
                            {34.03, -118.27, -1847.045937, -2218.281767},
                            {34.5, -117.9, 32144.065754, 49972.923562}};
  const Projection P(34.05, -118.25);
  for (const auto &r : ref) {
    const Vec2 p = P.forward(r[0], r[1]);
    CHECK_NEAR(p.x, r[2], 1e-3);
    CHECK_NEAR(p.y, r[3], 1e-3);
    double lat, lon;
    P.inverse(p, lat, lon);
    CHECK_NEAR(lat, r[0], 1e-8);
    CHECK_NEAR(lon, r[1], 1e-8);
  }
}

TEST_CASE("tag helpers") {
  const auto v = splitLanes("left|through;right||");
  CHECK(v.size() == 4);
  CHECK(v[1] == "through;right");
  CHECK(v[3].empty());
  const auto l = splitList("through; right");
  CHECK(l.size() == 2 && l[1] == "right");
}
