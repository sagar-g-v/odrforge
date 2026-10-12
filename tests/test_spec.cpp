// ASAM OpenDRIVE conformance of the formulas and of the writer / reader.
// Section numbers refer to docs/research/opendrive_spec_notes.md (ASAM OpenDRIVE 1.8 / 1.9).
#include <pugixml.hpp>
#include <random>
#include <set>
#include <sstream>

#include "odrforge/odr.hpp"
#include "odrforge/rules.hpp"
#include "odrforge/validate.hpp"
#include "test.hpp"

using namespace odrforge;

namespace {
std::mt19937 rng(11);
double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }

/// spec 2: theta(ds) = hdg + k0 ds + (k1-k0)/(2L) ds^2, position = integral of (cos, sin)(theta)
Pose specIntegrate(const GeomElement &e, double ds, int n = 20000) {
  double x = e.x, y = e.y;
  for (int i = 0; i < n; ++i) {
    const double s = (i + 0.5) * ds / n;
    const double th = e.hdg + e.k0 * s + 0.5 * (e.k1 - e.k0) / e.length * s * s;
    x += std::cos(th) * ds / n;
    y += std::sin(th) * ds / n;
  }
  Pose p;
  p.x = x, p.y = y;
  p.hdg = e.hdg + e.k0 * ds + 0.5 * (e.k1 - e.k0) / e.length * ds * ds;
  p.kappa = e.k0 + (e.k1 - e.k0) * ds / e.length;
  return p;
}

odr::Road simpleRoad() {
  odr::Road r;
  r.id = "1";
  GeomElement g;
  g.length = 100.0;
  r.planView.elements = {g};
  r.planView.finalize();
  odr::LaneSection a, b;
  a.s = 0.0;
  b.s = 40.0;
  for (auto *sec : {&a, &b}) {
    odr::Lane l1, r1, r2;
    l1.id = 1, r1.id = -1, r2.id = -2;
    l1.widths = {{0.0, 3.0, 0, 0, 0}};
    r1.widths = {{0.0, 3.5, 0, 0, 0}};
    r2.widths = {{0.0, 2.0, 0, 0, 0}, {5.0, 2.0, 0.1, 0, 0}};  // sOffset 5: width grows from ds = 5
    sec->left = {l1};
    sec->right = {r1, r2};
  }
  r.lanes = {a, b};
  r.laneOffset = {{0.0, 0.5, 0, 0, 0}, {50.0, 0.5, 0.0, 0.003, -0.0001}};
  r.elevation = {{0.0, 10.0, 0.02, 0, 0}, {60.0, 11.2, 0.02, 0, 0}};
  return r;
}
}  // namespace

// ------------------------------------------------------------------------------------ geometry (spec 2)

TEST_CASE("spec2 line: x = x0 + ds cos(hdg), y = y0 + ds sin(hdg)") {
  for (int i = 0; i < 200; ++i) {
    GeomElement e;
    e.x = U(-1e3, 1e3), e.y = U(-1e3, 1e3), e.hdg = U(-kPi, kPi), e.length = U(0.1, 500);
    const double ds = U(0, e.length);
    const Pose p = e.eval(ds);
    CHECK(e.type() == GeomType::Line);
    CHECK_NEAR(p.x, e.x + ds * std::cos(e.hdg), 1e-9);
    CHECK_NEAR(p.y, e.y + ds * std::sin(e.hdg), 1e-9);
    CHECK_NEAR(p.hdg, e.hdg, 1e-15);
  }
}

TEST_CASE("spec2 arc: closed form, k > 0 turns left") {
  for (int i = 0; i < 200; ++i) {
    GeomElement e;
    e.x = U(-1e3, 1e3), e.y = U(-1e3, 1e3), e.hdg = U(-kPi, kPi), e.length = U(1, 300);
    e.k0 = e.k1 = U(-0.2, 0.2);
    if (std::abs(e.k0) < 1e-4) e.k0 = e.k1 = 1e-3;
    const double k = e.k0, ds = U(0, e.length);
    const Pose p = e.eval(ds);
    CHECK(e.type() == GeomType::Arc);
    CHECK_NEAR(p.x, e.x + (std::sin(e.hdg + k * ds) - std::sin(e.hdg)) / k, 1e-8);
    CHECK_NEAR(p.y, e.y - (std::cos(e.hdg + k * ds) - std::cos(e.hdg)) / k, 1e-8);
    CHECK_NEAR(p.hdg, e.hdg + k * ds, 1e-12);
    // left turn: positive curvature puts the end point on the left of the start direction
    if (k > 0) CHECK((p.pos() - e.start().pos()).dot(e.start().left()) > -1e-9);
  }
}

TEST_CASE("spec2 spiral: curvature linear in ds, heading quadratic, position = integral") {
  for (int i = 0; i < 60; ++i) {
    GeomElement e;
    e.x = U(-100, 100), e.y = U(-100, 100), e.hdg = U(-kPi, kPi), e.length = U(5, 200);
    e.k0 = U(-0.1, 0.1), e.k1 = U(-0.1, 0.1);
    const double ds = U(0, e.length);
    const Pose p = e.eval(ds), q = specIntegrate(e, ds);
    CHECK(e.type() == GeomType::Spiral);
    CHECK_NEAR(p.x, q.x, 1e-5);
    CHECK_NEAR(p.y, q.y, 1e-5);
    CHECK_NEAR(p.hdg, q.hdg, 1e-12);
    CHECK_NEAR(p.kappa, q.kappa, 1e-12);
  }
}

TEST_CASE("spec2 element type follows curvature (arc never 0, spiral only with a change)") {
  GeomElement e;
  e.length = 10;
  CHECK(e.type() == GeomType::Line);
  e.k0 = e.k1 = 0.01;
  CHECK(e.type() == GeomType::Arc);
  e.k1 = 0.02;
  CHECK(e.type() == GeomType::Spiral);
}

TEST_CASE("spec2 paramPoly3: normalized and arcLength ranges describe the same curve") {
  // a cubic in the local u/v frame (spec: aU = aV = bV = 0, bU > 0), p in [0, 1]
  auto n = std::make_shared<CubicGeom>();
  n->param = true, n->normalized = true;
  n->aU = 0, n->bU = 48, n->cU = 4, n->dU = -2;
  n->aV = 0, n->bV = 0, n->cV = 9, n->dV = -3;
  // @length = integrated curve length
  double len = 0;
  for (int i = 0; i < 200000; ++i) {
    const double p = (i + 0.5) / 200000;
    len += std::hypot(n->bU + 2 * n->cU * p + 3 * n->dU * p * p, 2 * n->cV * p + 3 * n->dV * p * p) / 200000;
  }
  // the same curve with pRange="arcLength" (p in [0, length]): coefficient k scales by length^-k
  auto a = std::make_shared<CubicGeom>(*n);
  a->normalized = false;
  a->bU = n->bU / len, a->cU = n->cU / (len * len), a->dU = n->dU / (len * len * len);
  a->bV = n->bV / len, a->cV = n->cV / (len * len), a->dV = n->dV / (len * len * len);
  GeomElement en, ea;
  en.hdg = ea.hdg = 0.4;
  en.x = ea.x = 3, en.y = ea.y = -2;
  en.length = ea.length = len;
  en.cubic = n, ea.cubic = a;
  CHECK(en.type() == GeomType::ParamPoly3);
  // the end of the curve is (u(1), v(1)) rotated by hdg and translated to (x, y)
  const double u1 = n->aU + n->bU + n->cU + n->dU, v1 = n->aV + n->bV + n->cV + n->dV;
  const Pose pe = en.eval(len);
  CHECK_NEAR(pe.x, 3 + u1 * std::cos(0.4) - v1 * std::sin(0.4), 1e-3);
  CHECK_NEAR(pe.y, -2 + u1 * std::sin(0.4) + v1 * std::cos(0.4), 1e-3);
  CHECK_NEAR(en.eval(0).hdg, 0.4, 1e-12);  // bV = 0: the curve starts along hdg
  for (double f : {0.0, 0.2, 0.5, 0.9, 1.0}) {
    const Pose p1 = en.eval(f * len), p2 = ea.eval(f * len);
    CHECK_NEAR(p1.x, p2.x, 1e-6);
    CHECK_NEAR(p1.y, p2.y, 1e-6);
    CHECK_NEAR(p1.hdg, p2.hdg, 1e-9);
  }
}

TEST_CASE("spec2 poly3: v(u) = a + b u + c u^2 + d u^3 in the local frame") {
  auto c = std::make_shared<CubicGeom>();
  c->aV = 0, c->bV = 0, c->cV = 0.01, c->dV = -0.0001;
  GeomElement e;
  e.hdg = 0.0;
  e.cubic = c;
  // arc length of v(u) for u in [0, 30]
  double len = 0;
  for (int i = 0; i < 20000; ++i) {
    const double u = (i + 0.5) * 30.0 / 20000;
    len += std::hypot(1.0, 2 * c->cV * u + 3 * c->dV * u * u) * 30.0 / 20000;
  }
  e.length = len;
  const Pose p = e.eval(len);
  CHECK_NEAR(p.x, 30.0, 5e-3);
  CHECK_NEAR(p.y, c->cV * 900 + c->dV * 27000, 5e-3);
}

// ------------------------------------------------------------------------------------ lanes (spec 4)

TEST_CASE("spec4.4 width polynomial uses ds relative to the section start and sOffset") {
  const odr::Road r = simpleRoad();
  // section 2 starts at s = 40; lane -2 has width 2.0 + 0.1 (ds - 5) from sOffset 5
  double ti, to;
  CHECK(odr::laneBorders(r, 47.0, -2, ti, to));
  CHECK_NEAR(std::abs(to - ti), 2.0 + 0.1 * (47.0 - 40.0 - 5.0), 1e-12);
  CHECK(odr::laneBorders(r, 43.0, -2, ti, to));
  CHECK_NEAR(std::abs(to - ti), 2.0, 1e-12);  // before sOffset 5 the first record applies
}

TEST_CASE("spec4.1 lane borders: lane 0 at laneOffset, left lanes +t, right lanes -t") {
  const odr::Road r = simpleRoad();
  const double s = 3.0, off = 0.5;  // before lane -2's second width record (sOffset 5)
  double ti, to;
  CHECK(odr::laneBorders(r, s, 1, ti, to));
  CHECK_NEAR(ti, off, 1e-12);
  CHECK_NEAR(to, off + 3.0, 1e-12);
  CHECK(odr::laneBorders(r, s, -1, ti, to));
  CHECK_NEAR(ti, off, 1e-12);
  CHECK_NEAR(to, off - 3.5, 1e-12);
  CHECK(odr::laneBorders(r, s, -2, ti, to));
  CHECK_NEAR(ti, off - 3.5, 1e-12);
  CHECK_NEAR(to, off - 5.5, 1e-12);
  // laneOffset record at s = 50 (absolute s, not section relative): offset(ds) = a + b ds + c ds^2 + d ds^3
  CHECK_NEAR(odr::laneOffsetAt(r, 60.0), 0.5 + 0.003 * 100 - 0.0001 * 1000, 1e-12);
}

TEST_CASE("spec3 elevation records use absolute s") {
  const odr::Road r = simpleRoad();
  CHECK_NEAR(odr::elevationAt(r, 30.0), 10.0 + 0.02 * 30.0, 1e-12);
  CHECK_NEAR(odr::elevationAt(r, 70.0), 11.2 + 0.02 * 10.0, 1e-12);
  CHECK_NEAR(odr::elevationSlopeAt(r, 70.0), 0.02, 1e-12);
}

TEST_CASE("spec4.4 lane taper cubic: a=0, b=0, c=3W/L^2, d=-2W/L^3") {
  const double W = 3.5, L = 30.0;
  const odr::Poly3 p = odr::Poly3::hermite(0.0, L, 0.0, 0.0, W, 0.0);
  CHECK_NEAR(p.a, 0.0, 1e-15);
  CHECK_NEAR(p.b, 0.0, 1e-15);
  CHECK_NEAR(p.c, 3 * W / (L * L), 1e-12);
  CHECK_NEAR(p.d, -2 * W / (L * L * L), 1e-12);
  CHECK_NEAR(p.eval(L), W, 1e-12);
  CHECK_NEAR(p.deriv(L), 0.0, 1e-12);
}

// ------------------------------------------------------------------------------------ writer / reader

namespace {
odr::OpenDrive sampleNetwork() {
  odr::OpenDrive od;
  od.header.geoReference = "+proj=tmerc +lat_0=34 +lon_0=-118";
  odr::Road r = simpleRoad();
  r.types.push_back({0.0, "town", "US", 13.4, "m/s"});
  r.lanes[0].right[0].speedMax = 13.4;
  r.lanes[0].right[0].roadMarks.push_back({});
  r.lanes[0].left[0].type = "sidewalk";
  r.lanes[0].left[0].heightInner = r.lanes[0].left[0].heightOuter = 0.15;
  r.lanes[1].left[0].type = "bidirectional";
  r.lanes[0].right[1].accessDeny = {"passengerCar", "truck"};
  odr::Signal s;
  s.id = "7";
  s.s = 95, s.t = -6, s.dynamic = true, s.orientation = "+", s.country = "OpenDRIVE", s.type = "1000001";
  s.validity = {{-2, -1}};
  r.signals.push_back(s);
  odr::Object o;
  o.id = "9", o.type = "crosswalk", o.s = 98, o.length = 3, o.width = 6;
  o.outline = {{-1.5, -6, 0, 0}, {1.5, -6, 0, 0}, {1.5, 3, 0, 0}, {-1.5, 3, 0, 0}};
  r.objects.push_back(o);
  // an arc element written as <arc>, a zero-curvature element as <line>
  GeomElement g2;
  g2.length = 20;
  g2.k0 = g2.k1 = 0.02;
  r.planView.elements.push_back(g2);
  r.planView.finalize();
  od.roads.push_back(r);
  odr::Controller c;
  c.id = "1";
  c.signalIds = {"7"};
  od.controllers.push_back(c);
  return od;
}
}  // namespace

TEST_CASE("writer: required attributes of every element are written explicitly (spec pitfall 2)") {
  const std::string xml = odr::toString(sampleNetwork(), {6, 12});
  pugi::xml_document d;
  CHECK(d.load_string(xml.c_str()));
  const auto root = d.child("OpenDRIVE");
  const auto h = root.child("header");
  for (const char *a : {"revMajor", "revMinor", "north", "south", "east", "west"}) CHECK(h.attribute(a));
  CHECK(std::string(h.child_value("geoReference")).find("+proj=tmerc") == 0);
  const auto road = root.child("road");
  for (const char *a : {"id", "junction", "length"}) CHECK(road.attribute(a));
  for (auto g : road.child("planView").children("geometry"))
    for (const char *a : {"s", "x", "y", "hdg", "length"}) CHECK(g.attribute(a));
  CHECK(road.child("planView").first_child().first_child().name() == std::string("line"));
  CHECK(road.child("planView").last_child().first_child().name() == std::string("arc"));
  for (auto sec : road.child("lanes").children("laneSection")) {
    CHECK(sec.attribute("s"));
    CHECK(sec.child("center").child("lane").attribute("id").as_int() == 0);
    CHECK(!sec.child("center").child("lane").child("width"));  // spec 4.1: center lane has no width
    // spec 4.1: left lanes listed in descending id, right lanes descending (-1, -2, ...)
    int prev = 1 << 20;
    for (auto l : sec.child("left").children("lane")) {
      CHECK(l.attribute("id").as_int() < prev);
      prev = l.attribute("id").as_int();
    }
    prev = 0;
    for (auto l : sec.child("right").children("lane")) {
      CHECK(l.attribute("id").as_int() < prev);
      prev = l.attribute("id").as_int();
    }
    for (auto l : sec.child("right").children("lane")) {
      for (const char *a : {"id", "type", "level"}) CHECK(l.attribute(a));
      for (auto w : l.children("width"))
        for (const char *a : {"sOffset", "a", "b", "c", "d"}) CHECK(w.attribute(a));
      for (auto sp : l.children("speed")) CHECK(sp.attribute("unit"));  // spec 4.7: always write the unit
    }
  }
  const auto sig = road.child("signals").child("signal");
  for (const char *a : {"s", "t", "id", "dynamic", "orientation", "zOffset", "country", "type", "subtype"}) CHECK(sig.attribute(a));
  CHECK(sig.child("validity").attribute("fromLane").as_int() <= sig.child("validity").attribute("toLane").as_int());
  const auto obj = road.child("objects").child("object");
  for (const char *a : {"id", "s", "t", "zOffset"}) CHECK(obj.attribute(a));
  CHECK(obj.child("outlines").child("outline").child("cornerLocal").attribute("id"));  // 1.5+: corner ids
  const auto ctrl = root.child("controller");
  CHECK(ctrl.attribute("id") && ctrl.child("control").attribute("signalId"));
}

TEST_CASE("writer 1.8: walking / direction=both / access restriction children, deprecated types mapped") {
  const std::string xml = odr::toString(sampleNetwork(), {8, 12});
  pugi::xml_document d;
  CHECK(d.load_string(xml.c_str()));
  std::set<std::string> types;
  bool both = false, restrictionChild = false, centerType = false;
  for (auto l : d.select_nodes("//lane")) {
    const auto n = l.node();
    if (n.attribute("id").as_int() == 0) centerType |= bool(n.attribute("type"));
    else types.insert(n.attribute("type").value());
    both |= std::string(n.attribute("direction").value()) == "both";
    for (auto a : n.children("access")) restrictionChild |= bool(a.child("restriction")) && !a.attribute("restriction");
  }
  CHECK(types.count("walking") && !types.count("sidewalk") && !types.count("bidirectional"));
  CHECK(both);
  CHECK(restrictionChild);
  CHECK(!centerType);  // center lane type is deprecated in 1.8
  CHECK(std::string(d.child("OpenDRIVE").child("header").attribute("revMinor").value()) == "8");
}

TEST_CASE("writer 1.4 (carla profile): pre-1.8 vocabulary, bare <outline>, center lane typed") {
  const std::string xml = odr::toString(sampleNetwork(), {4, 12});
  pugi::xml_document d;
  CHECK(d.load_string(xml.c_str()));
  std::set<std::string> types;
  bool direction = false, centerType = false, restrictionChild = false;
  for (auto l : d.select_nodes("//lane")) {
    const auto n = l.node();
    if (n.attribute("id").as_int() == 0) centerType |= bool(n.attribute("type"));
    else types.insert(n.attribute("type").value());
    direction |= bool(n.attribute("direction"));
    for (auto a : n.children("access")) restrictionChild |= bool(a.child("restriction"));
  }
  CHECK(types.count("sidewalk") && types.count("bidirectional") && !types.count("walking"));
  CHECK(!direction && !restrictionChild && centerType);
  const auto obj = d.select_node("//object").node();
  CHECK(obj.child("outline") && !obj.child("outlines"));
}

TEST_CASE("output profiles: standard is OpenDRIVE 1.8, carla is 1.4, older names mean standard") {
  CHECK(TargetProfile::byName("standard").revMinor == 8);
  CHECK(TargetProfile::byName("carla").revMinor == 4);
  CHECK(TargetProfile::byName("carla").maxIdLength == 32);
  for (const char *legacy : {"asam", "asam18", "esmini"}) CHECK(TargetProfile::byName(legacy).name == "standard");
  bool threw = false;
  try {
    TargetProfile::byName("opendrive2");
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  CHECK(threw);
  CHECK(TargetProfile{}.spec().hasLaneDirection() && !TargetProfile::byName("carla").spec().hasLaneDirection());
}

TEST_CASE("writer precision keeps at least 10 significant digits") {
  odr::OpenDrive od = sampleNetwork();
  od.roads[0].planView.elements[0].x = 1234.567890123;
  od.roads[0].planView.finalize();
  const std::string xml = odr::toString(od, {6, 12});
  CHECK(xml.find("1234.56789012") != std::string::npos);
}

TEST_CASE("reader round trip preserves geometry, lanes, signals, controllers") {
  const odr::OpenDrive a = sampleNetwork();
  const std::string path = "odrforge_roundtrip_test.xodr";
  odr::write(a, path, {6, 15});
  const odr::OpenDrive b = odr::read(path);
  std::remove(path.c_str());
  CHECK(b.roads.size() == 1);
  const auto &ra = a.roads[0], &rb = b.roads[0];
  CHECK_NEAR(rb.length(), ra.length(), 1e-9);
  for (double s = 0; s <= ra.length(); s += 7.3) {
    const Pose pa = ra.planView.eval(s), pb = rb.planView.eval(s);
    CHECK_NEAR(pa.x, pb.x, 1e-9);
    CHECK_NEAR(pa.y, pb.y, 1e-9);
    for (int id : {1, -1, -2}) {
      double a1, a2, b1, b2;
      CHECK(odr::laneBorders(ra, s, id, a1, a2) == odr::laneBorders(rb, s, id, b1, b2));
      CHECK_NEAR(a2, b2, 1e-9);
    }
    CHECK_NEAR(odr::elevationAt(ra, s), odr::elevationAt(rb, s), 1e-9);
  }
  CHECK(rb.signals.size() == 1 && rb.signals[0].validity.size() == 1 && rb.signals[0].dynamic);
  CHECK(b.controllers.size() == 1 && b.controllers[0].signalIds[0] == "7");
  CHECK(rb.lanes[0].left[0].heightInner && std::abs(*rb.lanes[0].left[0].heightInner - 0.15) < 1e-12);
}

TEST_CASE("validator catches spec violations (negative tests)") {
  odr::OpenDrive od = sampleNetwork();
  od.roads[0].signals[0].validity = {{-1, -2}};            // fromLane > toLane
  od.roads[0].lanes[1].right[1].id = -3;                   // non-contiguous ids
  od.roads[0].lanes[0].right[0].widths[0].a = -1.0;        // negative width
  od.roads[0].planView.elements[1].x += 0.5;               // gap in the reference line
  od.controllers[0].signalIds.push_back("404");            // unknown signal
  od.roads[0].lanes[0].left[0].type = "flyingCarLane";     // invalid enum
  const ValidationReport v = validate(od);
  CHECK(v.count("spec7.1.validity_order") == 1);
  CHECK(v.count("spec10.lane_id_contiguity") >= 1);
  CHECK(v.count("road.lane.width.lane_width_validity") >= 1);
  CHECK(v.count("road.geometry.refline_no_gaps") == 1);
  CHECK(v.count("spec7.2.control_signal") == 1);
  CHECK(v.count("spec4.3.lane_type_enum") == 1);
  // and a clean model passes
  CHECK(validate(sampleNetwork()).errors == 0);
}

TEST_CASE("writer: road length is never shorter than the written geometry (strict readers)") {
  // LibCarla checks s <= road length strictly: the written length must cover the written s + length
  // of the last geometry even after each value is rounded to the output precision
  odr::OpenDrive od = sampleNetwork();
  auto &els = od.roads[0].planView.elements;
  els[0].length = 0.12345678901251;
  od.roads[0].planView.finalize();
  for (int prec : {9, 12}) {
    const std::string xml = odr::toString(od, {6, prec});
    pugi::xml_document d;
    CHECK(d.load_string(xml.c_str()));
    for (auto road : d.child("OpenDRIVE").children("road")) {
      const double L = road.attribute("length").as_double();
      auto last = road.child("planView").last_child();
      CHECK(last.attribute("s").as_double() + last.attribute("length").as_double() <= L);
    }
  }
}
