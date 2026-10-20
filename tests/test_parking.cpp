// Street parking: the OSM street-parking tags (wiki Street parking) and street-side parking areas mapped beside the
// road (amenity=parking + parking=street_side | lane | layby) become lanes of type parking (OpenDRIVE 1.8.0 §11.7.1,
// 1.4H laneType parking); near junctions a tagged lane carries the no-stopping lane rule (§11.9), an area's lane runs
// over the stretch it is mapped, in new lane sections (§11.3).
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
  /// a closed area through the points
  OsmId area(const std::vector<Vec2> &pts, Tags t) {
    std::vector<OsmId> ids;
    for (const Vec2 &p : pts) ids.push_back(node(p.x, p.y));
    ids.push_back(ids.front());
    return way(ids, std::move(t));
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


const odr::Road *street(const odr::OpenDrive &od) {
  for (const auto &r : od.roads)
    if (r.junction == "-1" && r.planView.length() > 200.0) return &r;
  return nullptr;
}

/// road s at the point (x, 0) of the street drawn along y = 0
double sAt(const odr::Road &r, double x) {
  double s, t;
  r.planView.project({x, 0.0}, s, t);
  return s;
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

namespace {
/// a 300 m two-way residential street along y = 0 (x from -150 to 150) and one area beside it on the right
/// (south) side: inner edge `inner` m beyond the carriageway edge, `depth` deep, from x0 to x1, ends cut at 45 deg
struct BayCase {
  odr::OpenDrive od;
  BuildReport rep;
  int errors = 0;
};
BayCase buildBay(Tags streetTags, double x0, double x1, double inner, double depth, std::string kind = "street_side") {
  OsmB b;
  Tags t{{"highway", "residential"}};
  for (const auto &[k, v] : streetTags) t[k] = v;
  b.way({b.node(-150, 0), b.node(0, 0), b.node(150, 0)}, t);
  const BuildOptions o = opts();
  const double edge = [&] {
    const CrossSection x = crossSectionFromTags(t, o.country);
    double e = -x.laneOffset;
    for (const auto &l : x.right.lanes)
      if (l.kind == LaneKind::Driving) e += l.width;
    return e;
  }();
  if (depth > 0.0) {
    const double yi = -(edge + inner), yo = yi - depth;
    b.area({{x0, yi}, {x1, yi}, {x1 - depth, yo}, {x0 + depth, yo}}, {{"amenity", "parking"}, {"parking", kind}});
  }
  BayCase c;
  c.od = buildOpenDrive(b.d, b.proj, o, &c.rep);
  const ValidationReport v = validate(c.od, o.target);
  for (size_t i = 0; i < v.messages.size() && i < 6; ++i) std::printf("    %s\n", v.messages[i].c_str());
  c.errors = v.errors;
  return c;
}
}  // namespace

TEST_CASE("street-side parking area: a parking lane over the mapped stretch, as wide as the area, ends as measured") {
  const BayCase c = buildBay({}, -40.0, 20.0, 0.2, 2.4);
  CHECK(c.errors == 0);
  CHECK(c.rep.streetSideParking == 1);
  CHECK(c.rep.streetSideParkingLeftOut == 0);
  const odr::Road *r = street(c.od);
  CHECK(r != nullptr);
  if (!r) return;
  // beside the area: carriageway edge to outer edge (0.2 m within kAbsorb: no strip)
  CHECK_NEAR(widthAt(*r, sAt(*r, -10.0), "parking", true), 2.6, 0.1);
  CHECK(widthAt(*r, sAt(*r, -10.0), "border", true) < 0.0);
  // nowhere else, and not on the other side
  CHECK(widthAt(*r, sAt(*r, -60.0), "parking", true) < 0.05);
  CHECK(widthAt(*r, sAt(*r, 50.0), "parking", true) < 0.05);
  CHECK(widthAt(*r, sAt(*r, -10.0), "parking", false) < 0.0);
  // the 45-degree ends: full width 2.4 m in from each end (the 30 m network taper would still be at a third)
  CHECK(widthAt(*r, sAt(*r, -40.0 + 3.5), "parking", true) > 0.8 * 2.6);
  CHECK(widthAt(*r, sAt(*r, 20.0 - 3.5), "parking", true) > 0.8 * 2.6);
  CHECK(widthAt(*r, sAt(*r, -40.0 - 1.0), "parking", true) < 0.3);
}

TEST_CASE("street-side parking area: a strip beyond the carriageway becomes a road-level border lane before the parking lane") {
  const BayCase c = buildBay({}, -40.0, 20.0, 2.0, 2.5);
  CHECK(c.errors == 0);
  CHECK(c.rep.streetSideParking == 1);
  const odr::Road *r = street(c.od);
  if (!r) return;
  CHECK_NEAR(widthAt(*r, sAt(*r, -10.0), "border", true), 2.0, 0.1);
  CHECK_NEAR(widthAt(*r, sAt(*r, -10.0), "parking", true), 2.5, 0.1);
}

TEST_CASE("street-side parking area: beside the area its measured lane replaces the tagged one (counted); elsewhere the tag decides") {
  const BayCase tagged = buildBay({{"parking:right", "lane"}}, 0.0, 0.0, 0.0, 0.0);  // no area: the tag's lane all along
  const odr::Road *r0 = street(tagged.od);
  if (!r0) return;
  CHECK(widthAt(*r0, sAt(*r0, -100.0), "parking", true) > 2.0);
  CHECK(widthAt(*r0, sAt(*r0, 100.0), "parking", true) > 2.0);
  // perpendicular stalls (4.6 m) beside a lane tagged parallel (2.4 m): the area's lane replaces it there
  const BayCase c = buildBay({{"parking:right", "lane"}}, -40.0, 20.0, 0.2, 4.4);
  CHECK(c.errors == 0);
  CHECK(c.rep.parkingTagsOverruled >= 1);
  const odr::Road *r = street(c.od);
  if (!r) return;
  CHECK_NEAR(widthAt(*r, sAt(*r, -10.0), "parking", true), 4.6, 0.1);
  const double tagWidth = opts().country.parkingWidthParallel;
  CHECK_NEAR(widthAt(*r, sAt(*r, -100.0), "parking", true), tagWidth, 0.05);
  CHECK_NEAR(widthAt(*r, sAt(*r, 100.0), "parking", true), tagWidth, 0.05);
  // one parking lane along the whole side: it continues through the area (no lane starts or stops there)
  int sectionsWithout = 0;
  for (const auto &sec : r->lanes) {
    bool has = false;
    for (const auto &l : sec.right) has = has || l.type == "parking";
    sectionsWithout += !has;
  }
  CHECK(sectionsWithout == 0);
}

TEST_CASE("street-side parking area: an area as wide as the tagged lane (within 0.5 m) confirms it - no section of its own") {
  const BayCase c = buildBay({{"parking:right", "lane"}}, -40.0, 20.0, 0.2, 2.4);  // 2.6 m beside a 2.4 m tagged lane
  CHECK(c.errors == 0);
  CHECK(c.rep.streetSideParking == 1);
  CHECK(c.rep.parkingTagsOverruled == 0);
  const odr::Road *r = street(c.od);
  if (!r) return;
  CHECK(r->lanes.size() == 1);
  CHECK_NEAR(widthAt(*r, sAt(*r, -10.0), "parking", true), opts().country.parkingWidthParallel, 0.05);
}

TEST_CASE("street-side parking area: left out when it is a lot (too deep), too short, or drawn over the road") {
  // each left out with its own reason in the warning
  auto warned = [](const BayCase &c, const char *why) {
    return std::any_of(c.rep.warnings.begin(), c.rep.warnings.end(),
                       [&](const std::string &w) { return w.find("street-side parking") != std::string::npos && w.find(why) != std::string::npos; });
  };
  const BayCase deep = buildBay({}, -40.0, 20.0, 0.2, 12.0);  // 12 m deep: wider than kMaxWidth
  CHECK(deep.rep.streetSideParkingLeftOut == 1 && warned(deep, "m wide"));
  const BayCase shortBay = buildBay({}, -40.0, -36.0, 0.2, 1.8);  // 4 m long: shorter than kMinBay
  CHECK(shortBay.rep.streetSideParkingLeftOut == 1 && warned(shortBay, "shorter than"));
  const BayCase over = buildBay({}, -40.0, 20.0, -8.0, 9.0);  // from 8 m inside the edge across the reference line
  CHECK(over.rep.streetSideParking == 0);
  CHECK(over.rep.streetSideParkingLeftOut == 1 && warned(over, "drawn over the road"));
  CHECK(over.errors == 0);
}

TEST_CASE("street-side parking area: left out, its warning gives the reason from the road it lies nearest") {
  // Munich 449417494: a road farther off meets the area over more samples (and fails for its distance) than the road it
  // lies beside (a short stretch). Here a street 9 m off along all 60 m of the area, and a service road along the
  // area's outer edge for its last 2 m before turning away: the warning names the service road's reason
  OsmB b;
  const BuildOptions o = opts();
  const Tags street{{"highway", "residential"}}, service{{"highway", "service"}};
  b.way({b.node(-150, 0), b.node(150, 0)}, street);
  double edge = 0.0;
  for (const auto &l : crossSectionFromTags(street, o.country).right.lanes)
    if (l.kind == LaneKind::Driving) edge += l.width;
  double serviceHalf = 0.0;
  for (const auto &l : crossSectionFromTags(service, o.country).left.lanes)
    if (l.kind == LaneKind::Driving) serviceHalf += l.width;
  const double yi = -(edge + 9.0), yo = yi - 2.4, ys = yo - 0.2 - serviceHalf;
  b.way({b.node(-100, ys), b.node(-38, ys), b.node(-25, ys - 40)}, service);
  b.area({{-40, yi}, {20, yi}, {20, yo}, {-40, yo}}, {{"amenity", "parking"}, {"parking", "street_side"}});
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(rep.streetSideParking == 0 && rep.streetSideParkingLeftOut == 1);
  const auto warning = std::find_if(rep.warnings.begin(), rep.warnings.end(),
                                    [](const std::string &w) { return w.find("street-side parking") != std::string::npos; });
  CHECK(warning != rep.warnings.end());
  if (warning != rep.warnings.end()) {
    std::printf("    %s\n", warning->c_str());
    CHECK(warning->find("lies 9.0") == std::string::npos);  // not the street's reason (9 m off, met over all 60 m)
  }
}

TEST_CASE("street-side parking area: layby counts, a parking area that is not street-side does not") {
  CHECK(buildBay({}, -40.0, 20.0, 0.2, 2.4, "layby").rep.streetSideParking == 1);
  const BayCase surface = buildBay({}, -40.0, 20.0, 0.2, 2.4, "surface");
  CHECK(surface.rep.streetSideParking == 0 && surface.rep.streetSideParkingLeftOut == 0);
}

TEST_CASE("street-side parking area: between a street and a parallel service road it is drawn once, on the nearer road") {
  OsmB b;
  const BuildOptions o = opts();
  const Tags street{{"highway", "residential"}}, service{{"highway", "service"}};
  b.way({b.node(-150, 0), b.node(150, 0)}, street);
  b.way({b.node(-150, -11), b.node(150, -11)}, service);
  double edge = 0.0;
  for (const auto &l : crossSectionFromTags(street, o.country).right.lanes)
    if (l.kind == LaneKind::Driving) edge += l.width;
  // 0.2 m beyond the street's carriageway, a strip of about 2 m to the service road's
  const double yi = -(edge + 0.2), yo = yi - 2.4;
  b.area({{-40, yi}, {20, yi}, {17.6, yo}, {-37.6, yo}}, {{"amenity", "parking"}, {"parking", "street_side"}});
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.streetSideParking == 1);
  int roadsWithParking = 0;
  for (const auto &r : od.roads) {
    bool has = false;
    for (const auto &sec : r.lanes)
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) has = has || l.type == "parking";
    if (!has) continue;
    ++roadsWithParking;
    double s, t;
    r.planView.project({0.0, 0.0}, s, t);
    CHECK(std::abs(t) < 1.0);  // the street, not the service road 11 m away
  }
  CHECK(roadsWithParking == 1);
}

TEST_CASE("street-side parking area: two bays joined far behind the road are two stretches, nothing drawn between") {
  OsmB b;
  const BuildOptions o = opts();
  const Tags residential{{"highway", "residential"}};
  b.way({b.node(-150, 0), b.node(150, 0)}, residential);
  double edge = 0.0;
  for (const auto &l : crossSectionFromTags(residential, o.country).right.lanes)
    if (l.kind == LaneKind::Driving) edge += l.width;
  // a U: bays x -60..-42 and 2..20 beside the road, their legs joined 30 m south (beyond the rays' reach)
  const double yi = -(edge + 0.2), yo = yi - 2.4;
  b.area({{-60, yi}, {-40, yi}, {-40, -30}, {0, -30}, {0, yi}, {20, yi}, {20, yo}, {2, yo}, {2, -32}, {-42, -32}, {-42, yo}, {-60, yo}},
         {{"amenity", "parking"}, {"parking", "street_side"}});
  BuildReport rep;
  const odr::OpenDrive od = buildOpenDrive(b.d, b.proj, o, &rep);
  CHECK(validate(od, o.target).errors == 0);
  CHECK(rep.streetSideParking == 1);
  const odr::Road *r = street(od);
  if (!r) return;
  CHECK(widthAt(*r, sAt(*r, -50.0), "parking", true) > 2.0);
  CHECK(widthAt(*r, sAt(*r, 10.0), "parking", true) > 2.0);
  CHECK(widthAt(*r, sAt(*r, -20.0), "parking", true) < 0.05);
}
