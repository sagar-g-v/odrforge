#include <cstdio>
#include <pugixml.hpp>
#include <sstream>
#include <stdexcept>

#include "odrforge/odr.hpp"
#include "odrforge/version.hpp"

namespace odrforge::odr {

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
  int revMinor = 6;
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

void poly(pugi::xml_node parent, const char *name, const Poly3 &p, const Ctx &c, const char *sName = "s") {
  auto n = parent.append_child(name);
  attrS(n, sName, p.s);
  attrN(n, c, "a", p.a);
  attrN(n, c, "b", p.b);
  attrN(n, c, "c", p.c);
  attrN(n, c, "d", p.d);
}

void writeLane(pugi::xml_node parent, const Lane &l, const Ctx &c) {
  auto n = parent.append_child("lane");
  attr(n, "id", std::to_string(l.id));
  // OpenDRIVE 1.8 deprecates sidewalk (-> walking) and bidirectional (-> driving + direction="both")
  std::string type = l.type, direction = l.direction;
  if (c.revMinor >= 8) {
    if (type == "sidewalk") type = "walking";
    if (type == "bidirectional") type = "driving", direction = "both";
  }
  if (!(c.revMinor >= 8 && l.id == 0)) attr(n, "type", l.id == 0 && c.revMinor < 8 ? l.type : type);
  if (!(c.revMinor >= 8 && l.id == 0)) attr(n, "level", l.level ? "true" : "false");
  if (c.revMinor >= 8 && !direction.empty() && l.id != 0) attr(n, "direction", direction);
  if (l.link.predecessor || l.link.successor) {
    auto ln = n.append_child("link");
    if (l.link.predecessor) attr(ln.append_child("predecessor"), "id", std::to_string(*l.link.predecessor));
    if (l.link.successor) attr(ln.append_child("successor"), "id", std::to_string(*l.link.successor));
  }
  if (l.id != 0)
    for (const auto &w : l.widths) poly(n, "width", w, c, "sOffset");
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
  if (!l.accessDeny.empty()) {
    if (c.revMinor >= 8) {  // 1.8: one <access rule> with <restriction type> children
      auto a = n.append_child("access");
      attrN(a, c, "sOffset", 0.0);
      attr(a, "rule", "deny");
      for (const auto &deny : l.accessDeny) attr(a.append_child("restriction"), "type", deny);
    } else {
      for (const auto &deny : l.accessDeny) {
        auto a = n.append_child("access");
        attrN(a, c, "sOffset", 0.0);
        if (c.revMinor >= 6) attr(a, "rule", "deny");
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
  if (!r.rule.empty() && c.revMinor >= 5) attr(n, "rule", r.rule);
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
    attr(e, "type", t.type);
    if (!t.country.empty() && c.revMinor >= 6) attr(e, "country", t.country);
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
    attrN(e, c, "hdg", g.hdg);
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
  for (const auto &e : r.elevation) poly(ep, "elevation", e, c);
  auto lp = n.append_child("lateralProfile");
  for (const auto &e : r.superelevation) poly(lp, "superelevation", e, c);
  auto lanes = n.append_child("lanes");
  for (const auto &o : r.laneOffset) poly(lanes, "laneOffset", o, c);
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
    if (!o.subtype.empty()) attr(e, "subtype", o.subtype);
    attrS(e, "s", o.s);
    attrN(e, c, "t", o.t);
    attrN(e, c, "zOffset", o.zOffset);
    attrN(e, c, "hdg", o.hdg);
    attrN(e, c, "pitch", o.pitch);
    attrN(e, c, "roll", o.roll);
    attr(e, "orientation", o.orientation);
    if (o.radius > 0) attrN(e, c, "radius", o.radius);
    else {
      attrN(e, c, "length", o.length);
      attrN(e, c, "width", o.width);
    }
    attrN(e, c, "height", o.height);
    if (!o.outline.empty()) {
      auto ol = (c.revMinor >= 5 ? e.append_child("outlines").append_child("outline") : e.append_child("outline"));
      if (c.revMinor >= 5) {
        attr(ol, "id", "0");
        attr(ol, "closed", "true");
      }
      int k = 0;
      for (const auto &cr : o.outline) {
        auto cn = ol.append_child("cornerLocal");
        if (c.revMinor >= 5) attr(cn, "id", std::to_string(k++));
        attrN(cn, c, "u", cr.u);
        attrN(cn, c, "v", cr.v);
        attrN(cn, c, "z", cr.z);
        attrN(cn, c, "height", cr.height);
      }
    }
    writeValidity(e, o.validity);
  }
  for (const auto &b : r.bridges) {
    auto e = objs.append_child("bridge");
    attrS(e, "s", b.s);
    attrS(e, "length", b.length);
    attr(e, "name", b.name);
    attr(e, "id", b.id);
    attr(e, "type", b.type.empty() ? "concrete" : b.type);
  }
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
  c.revMinor = opt.revMinor;
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
  if (opt.revMinor >= 6 && (od.header.offX != 0 || od.header.offY != 0 || od.header.offZ != 0)) {
    auto o = h.append_child("offset");
    attrN(o, c, "x", od.header.offX);
    attrN(o, c, "y", od.header.offY);
    attrN(o, c, "z", od.header.offZ);
    attrN(o, c, "hdg", od.header.offHdg);
  }
  // XSD t_header: userData after geoReference / offset
  for (const auto &[k, v] : od.header.userData) {
    auto u = h.append_child("userData");
    attr(u, "code", k);
    attr(u, "value", v);
  }
  for (const auto &r : od.roads) writeRoad(root, r, c);
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
    auto e = root.append_child("junction");
    attr(e, "id", j.id);
    attr(e, "name", j.name);
    if (opt.revMinor >= 7 || j.type != "default") attr(e, "type", j.type);
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
