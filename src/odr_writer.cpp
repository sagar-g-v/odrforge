#include <cmath>
#include <cstdio>
#include <pugixml.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "odrforge/odr.hpp"
#include "odrforge/spec_revision.hpp"
#include "odrforge/version.hpp"

namespace odrforge::odr {

std::vector<Poly3> Poly3::shiftC2(double s, double L, double v0, double v1) {
  if (L <= 1e-9) return {Poly3{s, v1, 0, 0, 0}};
  // quarters q: t'' = A u/q, A (1 - u/q), -A u/q, -A (1 - u/q); the shift totals 2 A q^2
  const double q = 0.25 * L, A = (v1 - v0) / (2.0 * q * q), d = A / (6.0 * q);
  return {Poly3{s, v0, 0.0, 0.0, d}, Poly3{s + q, v0 + A * q * q / 6.0, 0.5 * A * q, 0.5 * A, -d},
          Poly3{s + 2 * q, v0 + A * q * q, A * q, 0.0, -d}, Poly3{s + 3 * q, v0 + 11.0 * A * q * q / 6.0, 0.5 * A * q, -0.5 * A, d}};
}

Poly3 Poly3::hermite(double s, double L, double v0, double m0, double v1, double m1) {
  Poly3 p;
  p.s = s;
  p.a = v0;
  p.b = m0;
  if (L <= 1e-9) return p;
  const double dv = v1 - v0;
  p.c = (3.0 * dv - (2.0 * m0 + m1) * L) / (L * L);
  p.d = (-2.0 * dv + (m0 + m1) * L) / (L * L * L);
  return p;
}

namespace {

struct Ctx {
  int prec = 9;
  SpecRevision spec;
  std::string num(double v) const {
    if (std::abs(v) < 1e-15) v = 0.0;
    char b[40];
    std::snprintf(b, sizeof(b), "%.*g", prec, v);
    return b;
  }
  /// positions along a road (s, sOffset, lengths along s): on a 2^-24 m grid, written exactly, so a
  /// reader's own differences and sums of them are exact (L - s0 + s0 == L; LibCarla asserts
  /// s <= road length strictly)
  static double snapS(double v, bool up = false) {
    const double q = 16777216.0;  // 2^24
    return (up ? std::ceil(v * q - 1e-6) : std::round(v * q)) / q;
  }
  static std::string sNum(double v, bool up = false) {
    v = snapS(v, up);
    if (std::abs(v) < 1e-15) v = 0.0;
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
  }
};

void attr(pugi::xml_node n, const char *k, const std::string &v) { n.append_attribute(k) = v.c_str(); }
void attrN(pugi::xml_node n, const Ctx &c, const char *k, double v) { attr(n, k, c.num(v)); }
void attrS(pugi::xml_node n, const char *k, double v) { attr(n, k, Ctx::sNum(v)); }

// True when `next` only restates `prev` from its own start: the same cubic rebased to next.s.
bool continuesUnchanged(const Poly3 &prev, const Poly3 &next) {
  constexpr double kSame = 1e-9;
  const double ds = next.s - prev.s;
  return std::abs(prev.eval(ds) - next.a) < kSame && std::abs(prev.deriv(ds) - next.b) < kSame &&
         std::abs(prev.c + 3.0 * prev.d * ds - next.c) < kSame && std::abs(prev.d - next.d) < kSame;
}

// OpenDRIVE 1.9 Annex F performance.avoid_redundant_info (1.7.0): a record that repeats the
// previous polynomial is left out (elevation, superelevation, laneOffset, lane width).
std::vector<Poly3> withoutRedundantRecords(const std::vector<Poly3> &records) {
  std::vector<Poly3> kept;
  for (const Poly3 &r : records)
    if (kept.empty() || !continuesUnchanged(kept.back(), r)) kept.push_back(r);
  return kept;
}

// The same along a road (records with absolute s, the last valid to roadEnd): a record is left out
// when the kept polynomial reproduces it over its whole span within `tolerance` (m, or rad for
// superelevation) - the curve the file describes moves by less than that.
std::vector<Poly3> withoutRedundantRecords(const std::vector<Poly3> &records, double roadEnd, double tolerance) {
  std::vector<Poly3> kept;
  for (size_t i = 0; i < records.size(); ++i) {
    const Poly3 &r = records[i];
    const double span = (i + 1 < records.size() ? records[i + 1].s : roadEnd) - r.s;
    bool reproduced = !kept.empty() && span > 0.0;
    for (int k = 0; reproduced && k <= 8; ++k) {
      const double ds = span * k / 8.0;
      reproduced = std::abs(kept.back().eval(r.s + ds - kept.back().s) - r.eval(ds)) < tolerance;
    }
    if (!reproduced) kept.push_back(r);
  }
  return kept;
}

void poly(pugi::xml_node parent, const char *name, const Poly3 &p, const Ctx &c, const char *sName = "s") {
  auto n = parent.append_child(name);
  attrS(n, sName, p.s);
  attrN(n, c, "a", p.a);
  attrN(n, c, "b", p.b);
  attrN(n, c, "c", p.c);
  attrN(n, c, "d", p.d);
}

// OpenDRIVE 1.9 §10.4 road type. The town subtypes (townExpressway .. townPlayStreet) arrived in
// 1.5; a 1.4 file says town.
std::string roadTypeFor(const std::string &type, const SpecRevision &spec) {
  const bool townSubtype = type.rfind("town", 0) == 0 && type != "town";
  return townSubtype && !spec.hasTownRoadSubtypes() ? "town" : type;
}

struct LaneTypeAndDirection {
  std::string type, direction;
};

// OpenDRIVE 1.9 §11.8 lane type and direction; 1.8.0 Table 256 replaces sidewalk by walking and
// bidirectional by driving with lane@direction="both". Before 1.8 the type is written as is.
LaneTypeAndDirection laneTypeAndDirection(const Lane &l, const SpecRevision &spec) {
  // OpenDRIVE 1.9 §11.8 lane types curb and connectingRamp (introduced 1.6): earlier revisions write
  // the kerb as a border lane and a ramp between two motorways as a driving lane
  if (!spec.hasCurbLanes() && l.type == "curb") return {"border", ""};
  if (!spec.hasCurbLanes() && l.type == "connectingRamp") return {"driving", ""};
  if (!spec.hasLaneDirection()) return {l.type, ""};
  if (l.type == "sidewalk") return {"walking", l.direction};
  if (l.type == "bidirectional") return {"driving", "both"};
  return {l.type, l.direction};
}

void writeLane(pugi::xml_node parent, const Lane &l, const Ctx &c) {
  auto n = parent.append_child("lane");
  attr(n, "id", std::to_string(l.id));
  const bool isCenter = l.id == 0;
  const auto [type, direction] = laneTypeAndDirection(l, c.spec);
  if (!isCenter || c.spec.centerLaneHasTypeAndLevel()) {
    attr(n, "type", type);
    attr(n, "level", l.level ? "true" : "false");
  }
  if (!isCenter && !direction.empty()) attr(n, "direction", direction);
  if (l.link.predecessor || l.link.successor) {
    auto ln = n.append_child("link");
    if (l.link.predecessor) attr(ln.append_child("predecessor"), "id", std::to_string(*l.link.predecessor));
    if (l.link.successor) attr(ln.append_child("successor"), "id", std::to_string(*l.link.successor));
  }
  if (l.id != 0)
    for (const auto &w : withoutRedundantRecords(l.widths)) poly(n, "width", w, c, "sOffset");
  for (const auto &m : l.roadMarks) {
    auto r = n.append_child("roadMark");
    attrS(r, "sOffset", m.sOffset);
    attr(r, "type", m.type);
    attr(r, "weight", m.weight);
    attr(r, "color", m.color);
    attrN(r, c, "width", m.width);
    attr(r, "laneChange", m.laneChange);
    if (m.height > 0) attrN(r, c, "height", m.height);  // optional; XSD 1.8: must be > 0 when present
  }
  if (l.id == 0) return;
  if (!l.material.empty()) {
    auto m = n.append_child("material");
    attrN(m, c, "sOffset", 0.0);
    attr(m, "surface", l.material);
    attrN(m, c, "friction", 0.8);
    attrN(m, c, "roughness", 0.0);
  }
  if (l.speedMax) {
    auto s = n.append_child("speed");
    attrN(s, c, "sOffset", 0.0);
    attrN(s, c, "max", *l.speedMax);
    attr(s, "unit", l.speedUnit);
  }
  // a bus lane denies other vehicle classes; a revision without them (1.4) writes a plain lane
  if (!l.accessDeny.empty() && c.spec.hasVehicleAccessRestrictions()) {
    if (c.spec.hasAccessRestrictionElements()) {
      auto a = n.append_child("access");
      attrN(a, c, "sOffset", 0.0);
      attr(a, "rule", "deny");
      for (const auto &deny : l.accessDeny) attr(a.append_child("restriction"), "type", deny);
    } else {
      for (const auto &deny : l.accessDeny) {
        auto a = n.append_child("access");
        attrN(a, c, "sOffset", 0.0);
        if (c.spec.hasAccessRule()) attr(a, "rule", "deny");
        attr(a, "restriction", deny);
      }
    }
  }
  if (l.heightInner || l.heightOuter) {
    auto h = n.append_child("height");
    attrN(h, c, "sOffset", 0.0);
    attrN(h, c, "inner", l.heightInner.value_or(0.0));
    attrN(h, c, "outer", l.heightOuter.value_or(0.0));
  }
  // OpenDRIVE 1.9 §11.8 lane height: each record holds from its sOffset to the next (1.4: same)
  for (const auto &hc : l.heightChanges) {
    auto h = n.append_child("height");
    attrN(h, c, "sOffset", hc.sOffset);
    attrN(h, c, "inner", hc.inner);
    attrN(h, c, "outer", hc.outer);
  }
}

void writeValidity(pugi::xml_node n, const std::vector<ValidityRange> &v) {
  for (const auto &r : v) {
    auto e = n.append_child("validity");
    attr(e, "fromLane", std::to_string(r.fromLane));
    attr(e, "toLane", std::to_string(r.toLane));
  }
}

void writeRoad(pugi::xml_node root, const Road &r, const Ctx &c) {
  auto n = root.append_child("road");
  attr(n, "name", r.name);
  {
    // the written length must not be shorter than the written geometries add up to (s + length of
    // the last element, each rounded to the output precision): readers that check s <= length
    // strictly (LibCarla Lane::ComputeTransform) otherwise reject the road end
    auto parsed = [&](double v) { return Ctx::snapS(v); };
    double need = r.length();
    if (!r.planView.elements.empty()) {
      const auto &g = r.planView.elements.back();
      need = std::max(need, parsed(g.s) + parsed(g.length));
      double sum = 0;  // readers may also accumulate the element lengths themselves
      for (const auto &e : r.planView.elements) sum += parsed(e.length);
      need = std::max(need, sum);
    }
    attr(n, "length", Ctx::sNum(need, true));
  }
  attr(n, "id", r.id);
  attr(n, "junction", r.junction);
  // A left-hand-traffic road states its rule in every revision, 1.4 included: without it a reader
  // runs every lane against its traffic (1.4 parsers ignore the attribute).
  if (r.rule == "LHT" || (!r.rule.empty() && c.spec.hasRoadRule())) attr(n, "rule", r.rule);
  if (r.predecessor || r.successor) {
    auto l = n.append_child("link");
    auto w = [&](const char *nm, const LinkTarget &t) {
      auto e = l.append_child(nm);
      attr(e, "elementType", t.elementType);
      attr(e, "elementId", t.elementId);
      if (t.elementType == "road" && !t.contactPoint.empty()) attr(e, "contactPoint", t.contactPoint);
    };
    if (r.predecessor) w("predecessor", *r.predecessor);
    if (r.successor) w("successor", *r.successor);
  }
  for (const auto &t : r.types) {
    auto e = n.append_child("type");
    attrS(e, "s", t.s);
    attr(e, "type", roadTypeFor(t.type, c.spec));
    if (!t.country.empty() && c.spec.hasRoadTypeCountry()) attr(e, "country", t.country);
    if (t.speedMax) {
      auto sp = e.append_child("speed");
      attrN(sp, c, "max", *t.speedMax);
      attr(sp, "unit", t.speedUnit);
    }
  }
  auto pv = n.append_child("planView");
  for (const auto &g : r.planView.elements) {
    auto e = pv.append_child("geometry");
    attrS(e, "s", g.s);
    attrN(e, c, "x", g.x);
    attrN(e, c, "y", g.y);
    attrN(e, c, "hdg", normalizeAngle(g.hdg));  // OpenDRIVE 1.9 §9.2: heading in (-pi, pi]
    attrS(e, "length", g.length);
    switch (g.type()) {
      case GeomType::Line: e.append_child("line"); break;
      case GeomType::Arc: attrN(e.append_child("arc"), c, "curvature", g.k0); break;
      case GeomType::Spiral: {
        auto sp = e.append_child("spiral");
        attrN(sp, c, "curvStart", g.k0);
        attrN(sp, c, "curvEnd", g.k1);
        break;
      }
      case GeomType::Poly3: {  // pass-through of geometry read from other files
        auto p = e.append_child("poly3");
        attrN(p, c, "a", g.cubic->aV);
        attrN(p, c, "b", g.cubic->bV);
        attrN(p, c, "c", g.cubic->cV);
        attrN(p, c, "d", g.cubic->dV);
        break;
      }
      case GeomType::ParamPoly3: {
        auto p = e.append_child("paramPoly3");
        const auto &k = *g.cubic;
        for (auto [nm, v] : {std::pair{"aU", k.aU}, {"bU", k.bU}, {"cU", k.cU}, {"dU", k.dU},
                             {"aV", k.aV}, {"bV", k.bV}, {"cV", k.cV}, {"dV", k.dV}})
          attrN(p, c, nm, v);
        attr(p, "pRange", k.normalized ? "normalized" : "arcLength");
        break;
      }
    }
  }
  auto ep = n.append_child("elevationProfile");
  // below what any consumer resolves; lane offset shapes the lane borders, whose heading must stay
  // continuous to 1e-4 rad, so it is kept far tighter than height
  constexpr double kSameHeight = 1e-4, kSameBank = 1e-5, kSameOffset = 1e-6;  // m, rad, m
  for (const auto &e : withoutRedundantRecords(r.elevation, r.length(), kSameHeight)) poly(ep, "elevation", e, c);
  auto lp = n.append_child("lateralProfile");
  for (const auto &e : withoutRedundantRecords(r.superelevation, r.length(), kSameBank))
    poly(lp, "superelevation", e, c);
  auto lanes = n.append_child("lanes");
  for (const auto &o : withoutRedundantRecords(r.laneOffset, r.length(), kSameOffset)) poly(lanes, "laneOffset", o, c);
  for (const auto &sec : r.lanes) {
    auto sn = lanes.append_child("laneSection");
    attrS(sn, "s", sec.s);
    if (!sec.left.empty()) {
      auto l = sn.append_child("left");
      for (auto it = sec.left.rbegin(); it != sec.left.rend(); ++it) writeLane(l, *it, c);  // outermost first
    }
    writeLane(sn.append_child("center"), sec.center, c);
    if (!sec.right.empty()) {
      auto rr = sn.append_child("right");
      for (const auto &ln : sec.right) writeLane(rr, ln, c);
    }
  }
  auto objs = n.append_child("objects");
  for (const auto &o : r.objects) {
    auto e = objs.append_child("object");
    attr(e, "id", o.id);
    attr(e, "name", o.name);
    attr(e, "type", o.type);
    if (!o.subtype.empty() && c.spec.hasObjectSubtype()) attr(e, "subtype", o.subtype);
    attrS(e, "s", o.s);
    attrN(e, c, "t", o.t);
    attrN(e, c, "zOffset", o.zOffset);
    attrN(e, c, "hdg", o.hdg);
    attrN(e, c, "pitch", o.pitch);
    attrN(e, c, "roll", o.roll);
    attr(e, "orientation", o.orientation);
    if (o.radius > 0) attrN(e, c, "radius", o.radius);
    else if (o.repeats.empty() || o.length > 0) {
      attrN(e, c, "length", o.length);
      attrN(e, c, "width", o.width);
    }
    // 1.8.0 §13.14.1: a barrier's size is given in its <repeat> records, not on the object
    if (o.repeats.empty() || o.height > 0) attrN(e, c, "height", o.height);
    // XSD 1.8.1 t_road_objects_object / 1.4H §5.3.8.1: <repeat> first in the object's sequence
    for (const ObjectRepeat &rp : o.repeats) {
      auto re = e.append_child("repeat");
      attrS(re, "s", rp.s);
      attrS(re, "length", rp.length);
      attrN(re, c, "distance", rp.distance);
      attrN(re, c, "tStart", rp.tStart);
      attrN(re, c, "tEnd", rp.tEnd);
      attrN(re, c, "heightStart", rp.heightStart);
      attrN(re, c, "heightEnd", rp.heightEnd);
      attrN(re, c, "zOffsetStart", rp.zOffsetStart);
      attrN(re, c, "zOffsetEnd", rp.zOffsetEnd);
      attrN(re, c, "widthStart", rp.widthStart);
      attrN(re, c, "widthEnd", rp.widthEnd);
    }
    if (!o.outline.empty()) {
      auto ol = (c.spec.hasOutlinesWrapper() ? e.append_child("outlines").append_child("outline")
                                             : e.append_child("outline"));
      if (c.spec.hasOutlinesWrapper()) {
        attr(ol, "id", "0");
        attr(ol, "closed", "true");
      }
      int k = 0;
      for (const auto &cr : o.outline) {
        auto cn = ol.append_child("cornerLocal");
        if (c.spec.hasOutlinesWrapper()) attr(cn, "id", std::to_string(k++));
        attrN(cn, c, "u", cr.u);
        attrN(cn, c, "v", cr.v);
        attrN(cn, c, "z", cr.z);
        attrN(cn, c, "height", cr.height);
      }
    }
    writeValidity(e, o.validity);
  }
  // OpenDRIVE 1.9 §13.11, §13.12 (XSD 1.4 - 1.8.1 t_road_objects): all tunnels, then all bridges.
  for (const auto &b : r.tunnels) {
    auto e = objs.append_child("tunnel");
    attrS(e, "s", b.s);
    attrS(e, "length", b.length);
    attr(e, "name", b.name);
    attr(e, "id", b.id);
    attr(e, "type", b.type.empty() ? "standard" : b.type);
    attrN(e, c, "lighting", 0.5);
    attrN(e, c, "daylight", 0.0);
  }
  for (const auto &b : r.bridges) {
    auto e = objs.append_child("bridge");
    attrS(e, "s", b.s);
    attrS(e, "length", b.length);
    attr(e, "name", b.name);
    attr(e, "id", b.id);
    attr(e, "type", b.type.empty() ? "concrete" : b.type);
  }
  auto sigs = n.append_child("signals");
  for (const auto &s : r.signals) {
    auto e = sigs.append_child("signal");
    attrS(e, "s", s.s);
    attrN(e, c, "t", s.t);
    attr(e, "id", s.id);
    attr(e, "name", s.name);
    attr(e, "dynamic", s.dynamic ? "yes" : "no");
    attr(e, "orientation", s.orientation);
    attrN(e, c, "zOffset", s.zOffset);
    attr(e, "country", s.country);
    if (!s.countryRevision.empty()) attr(e, "countryRevision", s.countryRevision);
    attr(e, "type", s.type);
    attr(e, "subtype", s.subtype);
    if (s.value) attrN(e, c, "value", *s.value);
    if (!s.unit.empty()) attr(e, "unit", s.unit);
    attrN(e, c, "height", s.height);
    attrN(e, c, "width", s.width);
    if (!s.text.empty()) attr(e, "text", s.text);
    attrN(e, c, "hOffset", s.hOffset);
    attrN(e, c, "pitch", 0.0);
    attrN(e, c, "roll", 0.0);
    writeValidity(e, s.validity);
    // OpenDRIVE 1.9 §14.3 / §14.4: validity, then dependencies, then references (schema order)
    for (const auto &d : s.dependencies) {
      auto x = e.append_child("dependency");
      attr(x, "id", d.id);
      if (!d.type.empty()) attr(x, "type", d.type);
    }
    if (c.spec.hasSignalReferenceElement())
      for (const auto &l : s.references) {
        auto x = e.append_child("reference");
        attr(x, "elementId", l.elementId);
        attr(x, "elementType", l.elementType);
        if (!l.type.empty()) attr(x, "type", l.type);
      }
    // XSD 1.8.1 t_road_signals_signal: ... reference, position, userData, then semantics
    if (c.spec.hasSignalSemantics() && !s.semanticsPriority.empty())
      attr(e.append_child("semantics").append_child("priority"), "type", s.semanticsPriority);
  }
  for (const auto &s : r.signalReferences) {
    auto e = sigs.append_child("signalReference");
    attrS(e, "s", s.s);
    attrN(e, c, "t", s.t);
    attr(e, "id", s.id);
    attr(e, "orientation", s.orientation);
    writeValidity(e, s.validity);
  }
  for (const auto &[k, v] : r.userData) {
    auto u = n.append_child("userData");
    attr(u, "code", k);
    attr(u, "value", v);
  }
}

pugi::xml_document build(const OpenDrive &od, const WriteOptions &opt) {
  Ctx c;
  c.prec = opt.precision;
  c.spec.minor = opt.revMinor;
  pugi::xml_document doc;
  auto decl = doc.append_child(pugi::node_declaration);
  decl.append_attribute("version") = "1.0";
  decl.append_attribute("encoding") = "UTF-8";
  auto root = doc.append_child("OpenDRIVE");
  auto h = root.append_child("header");
  attr(h, "revMajor", "1");
  attr(h, "revMinor", std::to_string(opt.revMinor));
  attr(h, "name", od.header.name);
  attr(h, "version", od.header.version);
  attr(h, "date", od.header.date);
  attrN(h, c, "north", od.header.north);
  attrN(h, c, "south", od.header.south);
  attrN(h, c, "east", od.header.east);
  attrN(h, c, "west", od.header.west);
  attr(h, "vendor", std::string("OdrForge ") + ODRFORGE_VERSION);
  if (!od.header.geoReference.empty())
    h.append_child("geoReference").append_child(pugi::node_cdata).set_value(od.header.geoReference.c_str());
  if (c.spec.hasHeaderOffset() && (od.header.offX != 0 || od.header.offY != 0 || od.header.offZ != 0)) {
    auto o = h.append_child("offset");
    attrN(o, c, "x", od.header.offX);
    attrN(o, c, "y", od.header.offY);
    attrN(o, c, "z", od.header.offZ);
    attrN(o, c, "hdg", od.header.offHdg);
  }
  // XSD t_header: geoReference, offset, license, then userData
  if (od.header.license && c.spec.hasHeaderLicense()) {
    auto l = h.append_child("license");
    attr(l, "name", od.header.license->name);
    if (!od.header.license->resource.empty()) attr(l, "resource", od.header.license->resource);
    if (!od.header.license->spdxid.empty()) attr(l, "spdxid", od.header.license->spdxid);
    if (!od.header.license->text.empty()) attr(l, "text", od.header.license->text);
  }
  for (const auto &[k, v] : od.header.userData) {
    auto u = h.append_child("userData");
    attr(u, "code", k);
    attr(u, "value", v);
  }
  // OpenDRIVE 1.9 §12.5: cross paths came with 1.8.0. Below it their crossing roads, and a junction holding
  // only cross paths, have no form: they are left out (the crosswalk object on the road stays)
  std::set<std::string> crossPathOnly, crossingRoads;
  if (!c.spec.hasCrossPaths())
    for (const auto &j : od.junctions) {
      if (j.connections.empty() && !j.crossPaths.empty()) crossPathOnly.insert(j.id);
      for (const auto &cp : j.crossPaths) crossingRoads.insert(cp.crossingRoad);
    }
  for (const auto &r : od.roads)
    if (!crossingRoads.count(r.id)) writeRoad(root, r, c);
  for (const auto &ct : od.controllers) {
    auto e = root.append_child("controller");
    attr(e, "id", ct.id);
    attr(e, "name", ct.name);
    attr(e, "sequence", std::to_string(ct.sequence));
    for (const auto &sid : ct.signalIds) {
      auto cn = e.append_child("control");
      attr(cn, "signalId", sid);
      attr(cn, "type", "0");
    }
  }
  for (const auto &j : od.junctions) {
    if (crossPathOnly.count(j.id)) continue;
    auto e = root.append_child("junction");
    attr(e, "id", j.id);
    attr(e, "name", j.name);
    if (c.spec.writesDefaultJunctionType() || j.type != "default") attr(e, "type", j.type);
    if (j.type == "virtual") {  // OpenDRIVE 1.9 §12.7 Table 69
      attr(e, "mainRoad", j.mainRoad);
      attrN(e, c, "sStart", j.sStart);
      attrN(e, c, "sEnd", j.sEnd);
      attr(e, "orientation", j.orientation);
    }
    for (const auto &cn : j.connections) {
      auto x = e.append_child("connection");
      attr(x, "id", cn.id);
      attr(x, "incomingRoad", cn.incomingRoad);
      attr(x, j.type == "direct" ? "linkedRoad" : "connectingRoad", cn.connectingRoad);
      attr(x, "contactPoint", cn.contactPoint);
      for (const auto &ll : cn.laneLinks) {
        auto l = x.append_child("laneLink");
        attr(l, "from", std::to_string(ll.from));
        attr(l, "to", std::to_string(ll.to));
      }
    }
    // OpenDRIVE 1.9 §12.5 (1.8+): after the connections
    for (const auto &cp : c.spec.hasCrossPaths() ? j.crossPaths : std::vector<CrossPath>{}) {
      auto x = e.append_child("crossPath");
      attr(x, "id", cp.id);
      attr(x, "crossingRoad", cp.crossingRoad);
      attr(x, "roadAtStart", cp.roadAtStart);
      attr(x, "roadAtEnd", cp.roadAtEnd);
      for (const auto &[tag, l] : {std::pair<const char *, const CrossPath::Link &>{"startLaneLink", cp.start}, {"endLaneLink", cp.end}}) {
        auto y = x.append_child(tag);
        attr(y, "from", std::to_string(l.from));
        attrN(y, c, "s", l.s);
        attr(y, "to", std::to_string(l.to));
      }
    }
    for (const auto &p : j.priorities) {
      auto x = e.append_child("priority");
      attr(x, "high", p.high);
      attr(x, "low", p.low);
    }
    for (const auto &ct : j.controllers) {
      auto x = e.append_child("controller");
      attr(x, "id", ct.id);
      attr(x, "type", "0");
      attr(x, "sequence", std::to_string(ct.sequence));
    }
    // OpenDRIVE 1.8.0 §12.9 / §12.11; XSD 1.8.1 t_junction_common: ..., controller, surface, planView, objects,
    // boundary, elevationGrid
    if (j.grid && c.spec.hasJunctionGrid() && j.type == "default") {
      auto g = e.append_child("planView").append_child("geometry");
      attrN(g, c, "s", 0.0);
      attrN(g, c, "x", j.grid->x);
      attrN(g, c, "y", j.grid->y);
      attrN(g, c, "hdg", j.grid->hdg);
      attrN(g, c, "length", j.grid->length);
      g.append_child("line");
      // §12.10 boundary (XSD 1.8.1: after planView / objects, before elevationGrid)
      if (!j.boundary.empty()) {
        auto bd = e.append_child("boundary");
        for (const auto &sg : j.boundary) {
          auto x = bd.append_child("segment");
          attr(x, "type", sg.type);
          attr(x, "roadId", sg.roadId);
          if (sg.type == "lane") {
            attr(x, "boundaryLane", std::to_string(sg.boundaryLane));
            attr(x, "sStart", sg.sStart);
            attr(x, "sEnd", sg.sEnd);
          } else {
            attr(x, "contactPoint", sg.contactPoint);
          }
        }
      }
      auto eg = e.append_child("elevationGrid");
      attrN(eg, c, "gridSpacing", j.grid->gridSpacing);
      attrN(eg, c, "sStart", j.grid->sStart);
      auto list = [&](const std::vector<double> &v) {
        std::string out;
        for (double z : v) out += (out.empty() ? "" : " ") + c.num(z);
        return out;
      };
      for (const auto &row : j.grid->rows) {
        auto x = eg.append_child("elevation");
        attrN(x, c, "center", row.center);
        if (!row.left.empty()) attr(x, "left", list(row.left));
        if (!row.right.empty()) attr(x, "right", list(row.right));
      }
    }
  }
  // OpenDRIVE 1.9 §12.13 (XSD 1.4 - 1.8.1: after the junctions)
  for (const auto &g : od.junctionGroups) {
    auto e = root.append_child("junctionGroup");
    attr(e, "name", g.name);
    attr(e, "id", g.id);
    attr(e, "type", g.type);
    for (const auto &jid : g.junctionIds) attr(e.append_child("junctionReference"), "junction", jid);
  }
  return doc;
}

}  // namespace

void write(const OpenDrive &od, const std::string &path, const WriteOptions &opt) {
  auto doc = build(od, opt);
  if (!doc.save_file(path.c_str(), "    ", pugi::format_default, pugi::encoding_utf8))
    throw std::runtime_error("cannot write " + path);
}

std::string toString(const OpenDrive &od, const WriteOptions &opt) {
  auto doc = build(od, opt);
  std::ostringstream ss;
  doc.save(ss, "    ");
  return ss.str();
}

}  // namespace odrforge::odr
