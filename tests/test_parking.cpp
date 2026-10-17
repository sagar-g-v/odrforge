// Street parking: the OSM street-parking tags (wiki Street parking) become lanes of type parking (OpenDRIVE 1.8.0
// §11.7.1, 1.4H laneType parking) that end the country's no-parking distance short of a junction, in new lane sections
// (1.8.0 §11.3).
#include <algorithm>
#include <cstdio>

#include "odrforge/lanes.hpp"
#include "odrforge/network.hpp"
#include "odrforge/validate.hpp"
#include "test.hpp"

using namespace odrforge;

namespace {

struct OsmB {
  Projection proj{48.1, 11.5};
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
    w.id = 9000 + next++;
    w.nodes = std::move(nodes);
    w.tags = std::move(t);
    d.ways[w.id] = w;
    return w.id;
  }
};

BuildOptions opts() {
  BuildOptions o;
  o.country = CountryProfile::load("DE");
  o.target = TargetProfile::byName("asam");
  o.streetFurniture = false;
  return o;
}

const LaneSpec *find(const SideSpec &s, LaneKind k) {
  for (const auto &l : s.lanes)
    if (l.kind == k) return &l;
  return nullptr;
}

/// width at road position s of the first lane of `type` on one side (right: id < 0), -1 where there is none
double widthAt(const odr::Road &r, double s, const std::string &type, bool right) {
  size_t k = 0;
  while (k + 1 < r.lanes.size() && r.lanes[k + 1].s <= s) ++k;
  const double ds = s - r.lanes[k].s;
  for (const odr::Lane &l : right ? r.lanes[k].right : r.lanes[k].left) {
    if (l.type != type) continue;
    const odr::Poly3 *w = &l.widths.front();
    for (const auto &p : l.widths)
      if (p.s <= ds) w = &p;
    return w->eval(ds - w->s);
  }
  return -1.0;
}

/// the rules of the first lane of `type` on one side at road position s (empty where it has none)
std::vector<std::string> rulesAt(const odr::Road &r, double s, const std::string &type, bool right) {
  size_t k = 0;
  while (k + 1 < r.lanes.size() && r.lanes[k + 1].s <= s) ++k;
  std::vector<std::string> out;
  for (const odr::Lane &l : right ? r.lanes[k].right : r.lanes[k].left)
    if (l.type == type) {
      for (const auto &rule : l.rules) out.push_back(rule.second);
      break;
    }
  return out;
}

/// the plain road whose reference line passes nearest to (x, y)
const odr::Road *roadNearPoint(const odr::OpenDrive &od, double x, double y) {
  const odr::Road *best = nullptr;
  double bd = 1e9;
  for (const auto &r : od.roads) {
    if (r.junction != "-1") continue;
    double s, t;
    r.planView.project({x, y}, s, t);
    const Pose p = r.planView.eval(s);
    if (std::hypot(p.x - x, p.y - y) < bd) bd = std::hypot(p.x - x, p.y - y), best = &r;
  }
  return best;
}


}  // namespace

TEST_CASE("parking tags: yes / on_kerb / half_on_kerb / width / old-scheme position; separate, no, plain parking give none") {
  const CountryProfile C = CountryProfile::load("DE");
  const Tags base{{"highway", "residential"}};
  auto xs = [&](Tags extra) {
    Tags t = base;
    for (const auto &[k, v] : extra) t[k] = v;
    return crossSectionFromTags(t, C);
  };
  const CrossSection plain = xs({});
  const double walk = find(plain.right, LaneKind::Sidewalk)->width;
  // parking:<side>=yes: parked there, the position untold - a parking lane (default parallel)
  const CrossSection yes = xs({{"parking:right", "yes"}});
  CHECK(find(yes.right, LaneKind::Parking) && find(yes.right, LaneKind::Parking)->width == C.parkingWidthParallel);
  CHECK(!find(yes.left, LaneKind::Parking));
  const CrossSection perp = xs({{"parking:both", "yes"}, {"parking:both:orientation", "perpendicular"}});
  CHECK(find(perp.left, LaneKind::Parking) && find(perp.left, LaneKind::Parking)->width == C.parkingWidthPerpendicular);
  CHECK(find(perp.right, LaneKind::Parking) && find(perp.right, LaneKind::Parking)->width == C.parkingWidthPerpendicular);
  // :width is the parking lane's width
  const CrossSection w = xs({{"parking:right", "lane"}, {"parking:right:width", "2.1"}});
  CHECK(find(w.right, LaneKind::Parking) && std::abs(find(w.right, LaneKind::Parking)->width - 2.1) < 1e-9);
  // on the kerb: the parking lane takes its share out of the sidewalk (user decision 2026-10-18)
  const CrossSection half = xs({{"parking:right", "half_on_kerb"}});
  CHECK(find(half.right, LaneKind::Parking));
  CHECK_NEAR(find(half.right, LaneKind::Sidewalk)->width, std::max(1.0, walk - 0.5 * C.parkingWidthParallel), 1e-9);
  const CrossSection full = xs({{"parking:right", "on_kerb"}});
  CHECK_NEAR(find(full.right, LaneKind::Sidewalk)->width, std::max(1.0, walk - C.parkingWidthParallel), 1e-9);
  CHECK_NEAR(find(full.left, LaneKind::Sidewalk)->width, walk, 1e-9);
  const CrossSection old = xs({{"parking:lane:right", "diagonal"}, {"parking:lane:right:diagonal", "half_on_kerb"}});
  CHECK(find(old.right, LaneKind::Parking) && find(old.right, LaneKind::Parking)->width == C.parkingWidthDiagonal);
  CHECK_NEAR(find(old.right, LaneKind::Sidewalk)->width, std::max(1.0, walk - 0.5 * C.parkingWidthDiagonal), 1e-9);
  // no lane from the tag: mapped as its own area, on the shoulder, none - and plain parking=* is not street parking
  for (const char *v : {"separate", "shoulder", "no"}) CHECK(!find(xs({{"parking:right", v}}).right, LaneKind::Parking));
  CHECK(!find(xs({{"parking", "yes"}}).right, LaneKind::Parking));
  CHECK(!find(xs({{"parking", "lane"}}).left, LaneKind::Parking));
}

TEST_CASE("parking tags: within the legal distance of a junction (DE StVO 5 m) the strip stays, ruled no stopping; not at a map edge") {
  OsmB b;
  const BuildOptions o = opts();
  const OsmId x = b.node(0, 0);
  b.way({b.node(-150, 0), x, b.node(150, 0)}, {{"highway", "residential"}, {"parking:both", "lane"}});
  b.way({b.node(0, -100), x, b.node(0, 100)}, {{"highway", "residential"}});
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.parkingNoStoppingAtJunctions >= 2);
  const double clear = o.country.parkingClearOfJunction, w = o.country.parkingWidthParallel;
  for (const double xr : {-100.0, 100.0}) {  // both roads at the junction
    const odr::Road *r = roadNearPoint(od, xr, 0.0);
    CHECK(r != nullptr);
    if (!r) continue;
    const double L = r->planView.length();
    double s0, t0;
    r->planView.project({0.0, 0.0}, s0, t0);
    const bool junctionAtEnd = s0 > 0.5 * L;
    auto fromJunction = [&](double d) { return junctionAtEnd ? L - d : d; };
    for (const bool right : {true, false}) {
      // the parking strip keeps its width up to the junction - the kerb does not move
      CHECK_NEAR(widthAt(*r, fromJunction(0.5 * clear), "parking", right), w, 0.05);
      CHECK_NEAR(widthAt(*r, fromJunction(clear + 3.0), "parking", right), w, 0.05);
      // ruled no stopping within the distance, free beyond it; the map-edge end is free up to the end
      CHECK(rulesAt(*r, fromJunction(0.5 * clear), "parking", right) == std::vector<std::string>{"no stopping at any time"});
      CHECK(rulesAt(*r, fromJunction(clear + 3.0), "parking", right).empty());
      CHECK(rulesAt(*r, junctionAtEnd ? 1.0 : L - 1.0, "parking", right).empty());
    }
  }
}

TEST_CASE("parking tags: a road too short to park on beside its junctions (a stub between two) keeps no parking lane") {
  OsmB b;
  const BuildOptions o = opts();
  const OsmId x0 = b.node(-13, 0), x1 = b.node(13, 0);
  b.way({b.node(-150, 0), x0, x1, b.node(150, 0)}, {{"highway", "residential"}, {"parking:both", "lane"}});
  b.way({b.node(-13, -100), x0, b.node(-13, 100)}, {{"highway", "residential"}});
  b.way({b.node(13, -100), x1, b.node(13, 100)}, {{"highway", "residential"}});
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  int stubs = 0;
  for (const auto &r : od.roads) {
    if (r.junction != "-1" || r.planView.length() > 2.0 * o.country.parkingClearOfJunction + 5.0) continue;
    double s, t;
    r.planView.project({0.0, 0.0}, s, t);
    const Pose p = r.planView.eval(s);
    if (std::hypot(p.x, p.y) > 1.0) continue;  // the stub between the two cross streets
    ++stubs;
    CHECK(rep.parkingDroppedOnShortRoads >= 1);
    for (const auto &sec : r.lanes)
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) CHECK(l.type != "parking");
  }
  CHECK(stubs == 1);
}
