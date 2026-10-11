#include <algorithm>
#include <pugixml.hpp>
#include <stdexcept>

#include "odrforge/odr.hpp"

namespace odrforge::odr {

namespace {

double num(const pugi::xml_node &n, const char *a, double def = 0.0) {
  const auto at = n.attribute(a);
  return at ? at.as_double(def) : def;
}

Poly3 readPoly(const pugi::xml_node &n, const char *sName) {
  return {num(n, sName), num(n, "a"), num(n, "b"), num(n, "c"), num(n, "d")};
}

Lane readLane(const pugi::xml_node &n) {
  Lane l;
  l.id = n.attribute("id").as_int();
  l.type = n.attribute("type").value();
  l.level = std::string(n.attribute("level").value()) == "true";
  l.direction = n.attribute("direction").value();
  if (auto lk = n.child("link")) {
    if (auto p = lk.child("predecessor")) l.link.predecessor = p.attribute("id").as_int();
    if (auto s = lk.child("successor")) l.link.successor = s.attribute("id").as_int();
  }
  for (auto w : n.children("width")) l.widths.push_back(readPoly(w, "sOffset"));
  if (l.widths.empty())  // <border> lanes: approximate the border polynomial as a width
    for (auto b : n.children("border")) l.widths.push_back(readPoly(b, "sOffset"));
  for (auto m : n.children("roadMark")) {
    RoadMark r;
    r.sOffset = num(m, "sOffset");
    r.type = m.attribute("type").value();
    r.weight = m.attribute("weight").value();
    r.color = m.attribute("color").value();
    r.width = num(m, "width", 0.12);
    r.laneChange = m.attribute("laneChange").value();
    l.roadMarks.push_back(r);
  }
  if (auto sp = n.child("speed")) {
    l.speedMax = num(sp, "max");
    l.speedUnit = sp.attribute("unit").value();
  }
  if (auto h = n.child("height")) l.heightInner = num(h, "inner"), l.heightOuter = num(h, "outer");
  return l;
}

std::vector<ValidityRange> readValidity(const pugi::xml_node &n) {
  std::vector<ValidityRange> v;
  for (auto x : n.children("validity")) v.push_back({x.attribute("fromLane").as_int(), x.attribute("toLane").as_int()});
  return v;
}

}  // namespace

OpenDrive read(const std::string &path) {
  pugi::xml_document doc;
  const auto res = doc.load_file(path.c_str());
  if (!res) throw std::runtime_error("OpenDRIVE: cannot parse " + path + ": " + res.description());
  const auto root = doc.child("OpenDRIVE");
  if (!root) throw std::runtime_error("OpenDRIVE: no <OpenDRIVE> root in " + path);
  OpenDrive od;
  if (auto h = root.child("header")) {
    od.header.revMinor = h.attribute("revMinor").as_int(4);
    od.header.name = h.attribute("name").value();
    od.header.north = num(h, "north");
    od.header.south = num(h, "south");
    od.header.east = num(h, "east");
    od.header.west = num(h, "west");
    od.header.geoReference = h.child_value("geoReference");
  }
  for (auto rn : root.children("road")) {
    Road r;
    r.id = rn.attribute("id").value();
    r.name = rn.attribute("name").value();
    r.junction = rn.attribute("junction").value();
    if (r.junction.empty()) r.junction = "-1";
    r.rule = rn.attribute("rule").value();
    if (auto lk = rn.child("link")) {
      auto rd = [](const pugi::xml_node &e) {
        return LinkTarget{e.attribute("elementType").value(), e.attribute("elementId").value(),
                          e.attribute("contactPoint").value()};
      };
      if (auto p = lk.child("predecessor")) r.predecessor = rd(p);
      if (auto s = lk.child("successor")) r.successor = rd(s);
    }
    for (auto t : rn.children("type")) {
      Road::Type ty;
      ty.s = num(t, "s");
      ty.type = t.attribute("type").value();
      ty.country = t.attribute("country").value();
      if (auto sp = t.child("speed")) ty.speedMax = num(sp, "max"), ty.speedUnit = sp.attribute("unit").value();
      r.types.push_back(ty);
    }
    for (auto g : rn.child("planView").children("geometry")) {
      GeomElement e;
      e.s = num(g, "s");
      e.x = num(g, "x");
      e.y = num(g, "y");
      e.hdg = num(g, "hdg");
      e.length = num(g, "length");
      if (auto a = g.child("arc")) {
        e.k0 = e.k1 = num(a, "curvature");
      } else if (auto sp = g.child("spiral")) {
        e.k0 = num(sp, "curvStart");
        e.k1 = num(sp, "curvEnd");
      } else if (auto p3 = g.child("poly3")) {
        auto c = std::make_shared<CubicGeom>();
        c->aV = num(p3, "a"), c->bV = num(p3, "b"), c->cV = num(p3, "c"), c->dV = num(p3, "d");
        e.cubic = c;
      } else if (auto pp = g.child("paramPoly3")) {
        auto c = std::make_shared<CubicGeom>();
        c->param = true;
        c->normalized = std::string(pp.attribute("pRange").value()) != "arcLength";
        c->aU = num(pp, "aU"), c->bU = num(pp, "bU"), c->cU = num(pp, "cU"), c->dU = num(pp, "dU");
        c->aV = num(pp, "aV"), c->bV = num(pp, "bV"), c->cV = num(pp, "cV"), c->dV = num(pp, "dV");
        e.cubic = c;
      }
      r.planView.elements.push_back(e);
    }
    for (auto e : rn.child("elevationProfile").children("elevation")) r.elevation.push_back(readPoly(e, "s"));
    for (auto e : rn.child("lateralProfile").children("superelevation")) r.superelevation.push_back(readPoly(e, "s"));
    auto lanes = rn.child("lanes");
    for (auto o : lanes.children("laneOffset")) r.laneOffset.push_back(readPoly(o, "s"));
    for (auto sn : lanes.children("laneSection")) {
      LaneSection sec;
      sec.s = num(sn, "s");
      for (auto l : sn.child("left").children("lane")) sec.left.push_back(readLane(l));
      std::sort(sec.left.begin(), sec.left.end(), [](const Lane &a, const Lane &b) { return a.id < b.id; });
      for (auto l : sn.child("right").children("lane")) sec.right.push_back(readLane(l));
      std::sort(sec.right.begin(), sec.right.end(), [](const Lane &a, const Lane &b) { return a.id > b.id; });
      if (auto c = sn.child("center").child("lane")) sec.center = readLane(c);
      r.lanes.push_back(sec);
    }
    for (auto o : rn.child("objects").children("object")) {
      Object ob;
      ob.id = o.attribute("id").value();
      ob.name = o.attribute("name").value();
      ob.type = o.attribute("type").value();
      ob.subtype = o.attribute("subtype").value();
      ob.s = num(o, "s");
      ob.t = num(o, "t");
      ob.hdg = num(o, "hdg");
      ob.length = num(o, "length");
      ob.width = num(o, "width");
      ob.radius = num(o, "radius");
      ob.height = num(o, "height");
      auto outline = o.child("outlines").child("outline");
      if (!outline) outline = o.child("outline");
      for (auto c : outline.children("cornerLocal"))
        ob.outline.push_back({num(c, "u"), num(c, "v"), num(c, "z"), num(c, "height")});
      r.objects.push_back(ob);
    }
    for (auto s : rn.child("signals").children("signal")) {
      Signal sg;
      sg.id = s.attribute("id").value();
      sg.name = s.attribute("name").value();
      sg.s = num(s, "s");
      sg.t = num(s, "t");
      sg.zOffset = num(s, "zOffset");
      sg.dynamic = std::string(s.attribute("dynamic").value()) == "yes";
      sg.orientation = s.attribute("orientation").value();
      sg.country = s.attribute("country").value();
      sg.type = s.attribute("type").value();
      sg.subtype = s.attribute("subtype").value();
      if (s.attribute("value")) sg.value = num(s, "value");
      sg.unit = s.attribute("unit").value();
      sg.validity = readValidity(s);
      r.signals.push_back(sg);
    }
    od.roads.push_back(std::move(r));
  }
  for (auto c : root.children("controller")) {
    Controller ct;
    ct.id = c.attribute("id").value();
    ct.name = c.attribute("name").value();
    for (auto x : c.children("control")) ct.signalIds.push_back(x.attribute("signalId").value());
    od.controllers.push_back(ct);
  }
  for (auto j : root.children("junction")) {
    Junction J;
    J.id = j.attribute("id").value();
    J.name = j.attribute("name").value();
    if (j.attribute("type")) J.type = j.attribute("type").value();
    for (auto c : j.children("connection")) {
      Connection cn;
      cn.id = c.attribute("id").value();
      cn.incomingRoad = c.attribute("incomingRoad").value();
      cn.connectingRoad = c.attribute("connectingRoad") ? c.attribute("connectingRoad").value()
                                                        : c.attribute("linkedRoad").value();
      cn.contactPoint = c.attribute("contactPoint").value();
      for (auto l : c.children("laneLink"))
        cn.laneLinks.push_back({l.attribute("from").as_int(), l.attribute("to").as_int()});
      J.connections.push_back(cn);
    }
    for (auto c : j.children("controller")) J.controllers.push_back({c.attribute("id").value(), c.attribute("sequence").as_int()});
    od.junctions.push_back(std::move(J));
  }
  return od;
}

}  // namespace odrforge::odr
