#include "odrforge/lanes.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace odrforge {

namespace {

constexpr double kMph = 0.44704, kKmh = 1.0 / 3.6;

bool isCarriageway(LaneKind k) {
  return k == LaneKind::Driving || k == LaneKind::Bus || k == LaneKind::Biking || k == LaneKind::Bidirectional;
}

std::optional<int> intTag(const Tags &t, const std::string &k) {
  if (auto v = tag(t, k)) {
    try {
      return std::stoi(*v);
    } catch (...) {
    }
  }
  return std::nullopt;
}

std::optional<double> lengthTag(const std::string &v) {
  // "3.5", "3.5 m", "12'6\"" (feet / inches)
  try {
    const auto ft = v.find('\'');
    if (ft != std::string::npos) {
      double feet = std::stod(v.substr(0, ft)), inch = 0.0;
      const auto rest = v.substr(ft + 1);
      if (!rest.empty() && std::isdigit(static_cast<unsigned char>(rest[0]))) inch = std::stod(rest);
      return feet * 0.3048 + inch * 0.0254;
    }
    double x = std::stod(v);
    if (v.find("ft") != std::string::npos) x *= 0.3048;
    return x > 0.5 && x < 60.0 ? std::optional<double>(x) : std::nullopt;
  } catch (...) {
    return std::nullopt;
  }
}

std::string sideValue(const Tags &t, const std::string &key, const std::string &side) {
  // key:side, key:both, key=both/left/right
  if (auto v = tag(t, key + ":" + side)) return *v;
  if (auto v = tag(t, key + ":both")) return *v;
  if (auto v = tag(t, key)) {
    if (*v == "both" || *v == "yes") return "yes";
    if (*v == side) return "yes";
    if (*v == "left" || *v == "right") return "no";
    return *v;  // no / none / separate / lane / track ...
  }
  return "";
}

double parkingWidth(const std::string &orientation, const CountryProfile &c) {
  if (orientation == "diagonal") return c.parkingWidthDiagonal;
  if (orientation == "perpendicular") return c.parkingWidthPerpendicular;
  return c.parkingWidthParallel;
}

bool urbanClass(const std::string &h) {
  return h == "primary" || h == "secondary" || h == "tertiary" || h == "residential" || h == "unclassified" ||
         h == "living_street" || h == "primary_link" || h == "secondary_link" || h == "tertiary_link";
}

/// add outer lanes (bike, parking, shoulder, sidewalk) of one geometric side ("left"/"right")
void addOuterLanes(SideSpec &s, const Tags &t, const std::string &side, const std::string &hw,
                   const CountryProfile &c, const LaneOptions &o) {
  const std::string cyc = sideValue(t, "cycleway", side);
  if (cyc == "lane" || cyc == "opposite_lane" || cyc == "exclusive" || cyc == "advisory")
    s.lanes.push_back({LaneKind::Biking, c.bikeLaneWidth, {}});
  // parking: new scheme parking:<side>=lane|street_side + orientation, old scheme parking:lane:<side>=parallel|...
  std::string park = sideValue(t, "parking", side), orient;
  if (park == "lane" || park == "street_side") {
    orient = tag(t, "parking:" + side + ":orientation").value_or(tag(t, "parking:both:orientation").value_or("parallel"));
  } else {
    const std::string old = sideValue(t, "parking:lane", side);
    if (old == "parallel" || old == "diagonal" || old == "perpendicular") orient = old;
  }
  if (!orient.empty()) s.lanes.push_back({LaneKind::Parking, parkingWidth(orient, c), {}});
  if (o.shoulders) {
    const std::string sh = sideValue(t, "shoulder", side);
    double w = 0.0;
    if (auto it = c.shoulderWidth.find(hw); it != c.shoulderWidth.end()) w = it->second;
    if (sh == "yes") w = std::max(w, 1.5);
    if (sh == "no") w = 0.0;
    if (w > 0.1) s.lanes.push_back({LaneKind::Shoulder, w, {}});
  }
  const std::string sw = sideValue(t, "sidewalk", side);
  bool sidewalk = sw == "yes" || sw == "both";
  if (sw.empty() && o.assumeSidewalks && urbanClass(hw)) sidewalk = true;
  if (sidewalk) {
    double w = c.sidewalkWidth;
    if (auto v = tag(t, "sidewalk:" + side + ":width")) w = lengthTag(*v).value_or(w);
    else if (auto v2 = tag(t, "sidewalk:both:width")) w = lengthTag(*v2).value_or(w);
    s.lanes.push_back({LaneKind::Sidewalk, w, {}});
  }
}

}  // namespace

int SideSpec::drivingCount() const {
  return static_cast<int>(std::count_if(lanes.begin(), lanes.end(), [](const LaneSpec &l) {
    return l.kind == LaneKind::Driving || l.kind == LaneKind::Bus;
  }));
}

double SideSpec::carriagewayWidth() const {
  double w = 0;
  for (const auto &l : lanes)
    if (isCarriageway(l.kind)) w += l.width;
  return w;
}

double SideSpec::totalWidth() const {
  double w = 0;
  for (const auto &l : lanes) w += l.width;
  return w;
}

bool CrossSection::sameTopology(const CrossSection &o) const {
  auto same = [](const SideSpec &a, const SideSpec &b) {
    if (a.lanes.size() != b.lanes.size()) return false;
    for (size_t i = 0; i < a.lanes.size(); ++i)
      if (a.lanes[i].kind != b.lanes[i].kind) return false;
    return true;
  };
  return same(left, o.left) && same(right, o.right);
}

std::optional<double> parseSpeed(const std::string &v0, const CountryProfile &c) {
  if (v0.empty()) return std::nullopt;
  std::string v = v0;
  if (auto it = c.maxspeedZones.find(v); it != c.maxspeedZones.end()) v = it->second;
  if (v == "walk") return 7.0 * kKmh;
  if (v == "none" || v == "signals" || v == "variable") return std::nullopt;
  try {
    const double x = std::stod(v);
    if (v.find("mph") != std::string::npos) return x * kMph;
    if (v.find("knots") != std::string::npos) return x * 0.514444;
    return x * kKmh;  // bare numbers are km/h by OSM convention (also in mph countries)
  } catch (...) {
    return std::nullopt;
  }
}

CrossSection crossSectionFromTags(const Tags &t, const CountryProfile &c, const LaneOptions &o) {
  CrossSection x;
  x.highway = tag(t, "highway").value_or("road");
  const std::string &hw = x.highway;
  const bool lht = c.leftHandTraffic;
  x.roundabout = tagIs(t, "junction", "roundabout") || tagIs(t, "junction", "circular");
  const std::string ow = tag(t, "oneway").value_or("");
  bool reverse = ow == "-1" || ow == "reverse";
  x.oneway = ow == "yes" || ow == "true" || ow == "1" || reverse || x.roundabout ||
             ((hw == "motorway" || hw == "motorway_link") && ow != "no");

  // ---- lane counts
  int fwd = 0, bwd = 0, both = intTag(t, "lanes:both_ways").value_or(0);
  const auto total = intTag(t, "lanes");
  if (x.oneway) {
    fwd = total.value_or(c.lanesOneway(hw));
    bwd = 0;
    both = 0;
  } else {
    auto f = intTag(t, "lanes:forward"), b = intTag(t, "lanes:backward");
    if (f && b) {
      fwd = *f, bwd = *b;
    } else if (total) {
      const int rem = std::max(2, *total - both);
      if (f) fwd = *f, bwd = std::max(1, rem - *f);
      else if (b) bwd = *b, fwd = std::max(1, rem - *b);
      else fwd = (rem + 1) / 2, bwd = rem - fwd;
    } else {
      const int n = c.lanesTwoway(hw);
      fwd = (n + 1) / 2, bwd = n - fwd;
    }
    fwd = std::max(fwd, 1);
    bwd = std::max(bwd, 1);
  }

  // ---- widths
  double laneW = c.width(hw);
  const int nDrive = fwd + bwd + both;
  if (auto w = tag(t, "width"))
    if (auto wv = lengthTag(*w)) laneW = std::clamp(*wv / std::max(1, nDrive), 2.2, 4.5);
  if (!x.oneway && total && *total == 1) laneW = std::max(2.3, laneW * 0.75);  // single-lane two-way street
  auto perLane = [&](const std::string &key, int n) {
    std::vector<double> w(n, laneW);
    if (auto v = tag(t, key)) {
      const auto parts = splitLanes(*v);
      for (int i = 0; i < n && i < static_cast<int>(parts.size()); ++i)
        if (auto lv = lengthTag(parts[i])) w[i] = *lv;
    }
    return w;
  };
  auto perLaneTurns = [&](const std::string &key, int n) {
    std::vector<std::vector<std::string>> r(n);
    if (auto v = tag(t, key)) {
      const auto parts = splitLanes(*v);
      if (static_cast<int>(parts.size()) == n)
        for (int i = 0; i < n; ++i) r[i] = splitList(parts[i]);
    }
    return r;
  };
  const std::string fs = x.oneway ? "" : ":forward", bs = ":backward";
  auto fW = perLane("width:lanes" + fs, fwd), bW = perLane("width:lanes" + bs, bwd);
  auto fT = perLaneTurns("turn:lanes" + fs, fwd), bT = perLaneTurns("turn:lanes" + bs, bwd);
  auto fBus = perLaneTurns("bus:lanes" + fs, fwd), bBus = perLaneTurns("bus:lanes" + bs, bwd);
  if (x.oneway) {  // oneway roads may also use the un-suffixed psv / bus keys
    auto psv = perLaneTurns("psv:lanes", fwd);
    for (int i = 0; i < fwd; ++i)
      if (fBus[i].empty()) fBus[i] = psv[i];
  }

  // OSM per-lane lists run from the left to the right in the driving direction. Forward lanes lie
  // on the right side of the way (RHT) where "inner -> outer" == "left -> right"; on LHT they lie on
  // the left side where inner -> outer == right -> left.
  auto makeSide = [&](int n, const std::vector<double> &w, const std::vector<std::vector<std::string>> &turns,
                      const std::vector<std::vector<std::string>> &bus, bool innerIsLeft) {
    SideSpec s;
    for (int k = 0; k < n; ++k) {
      const int i = innerIsLeft ? k : n - 1 - k;
      LaneSpec l;
      l.width = w[i];
      l.turns = turns[i];
      const bool isBus = !bus[i].empty() && (bus[i][0] == "designated" || bus[i][0] == "yes");
      l.kind = isBus ? LaneKind::Bus : LaneKind::Driving;
      s.lanes.push_back(l);
    }
    return s;
  };
  SideSpec fwdSide = makeSide(fwd, fW, fT, fBus, !lht);
  SideSpec bwdSide = makeSide(bwd, bW, bT, bBus, !lht);
  fwdSide.forward = !reverse;  // a reversed oneway carries its traffic against the way
  bwdSide.forward = reverse;
  if (both > 0)  // centre two-way turn lane belongs to the side next to the divider
    bwdSide.lanes.insert(bwdSide.lanes.begin(), LaneSpec{LaneKind::Bidirectional, laneW, {"left"}});

  // ---- speeds
  const auto def = c.speed(hw);
  std::optional<double> vAll = parseSpeed(tag(t, "maxspeed").value_or(""), c);
  if (!vAll && def) vAll = c.speedUnit == "mph" ? *def * kMph : *def * kKmh;
  fwdSide.speed = parseSpeed(tag(t, "maxspeed:forward").value_or(""), c);
  bwdSide.speed = parseSpeed(tag(t, "maxspeed:backward").value_or(""), c);
  if (!fwdSide.speed) fwdSide.speed = vAll;
  if (!bwdSide.speed) bwdSide.speed = vAll;

  // ---- assign to geometric sides (relative to the way direction)
  // RHT: forward traffic on the right of the way; LHT: on the left. A reversed oneway carries its
  // lanes against the way, i.e. on the other side.
  const bool fwdOnRight = (!lht) != reverse;
  x.right = fwdOnRight ? fwdSide : bwdSide;
  x.left = fwdOnRight ? bwdSide : fwdSide;
  if (x.oneway) (fwdOnRight ? x.left : x.right).lanes.clear();
  addOuterLanes(x.right, t, "right", hw, c, o);
  addOuterLanes(x.left, t, "left", hw, c, o);

  // ---- placement: where the OSM way lies across the carriageway (default: its middle)
  const double Wr = x.right.carriagewayWidth(), Wl = x.left.carriagewayWidth();
  x.laneOffset = 0.5 * (Wr - Wl);
  const std::string plKey = x.oneway ? "placement" : "placement:forward";
  if (auto pl = tag(t, plKey); pl && pl->find(':') != std::string::npos) {
    const std::string mode = pl->substr(0, pl->find(':'));
    const int N = std::atoi(pl->substr(pl->find(':') + 1).c_str());
    SideSpec &fs2 = fwdOnRight ? x.right : x.left;
    // driving lanes counted from the left in the driving direction
    std::vector<double> w;
    for (const auto &l : fs2.lanes)
      if (isCarriageway(l.kind)) w.push_back(l.width);
    if (!fwdOnRight) std::reverse(w.begin(), w.end());  // left side: outer lane is leftmost
    if (N >= 1 && N <= static_cast<int>(w.size())) {
      double before = 0.0;
      for (int i = 0; i < N - 1; ++i) before += w[i];
      double along = before;  // distance from the left edge of the forward carriageway
      if (mode == "middle_of") along += 0.5 * w[N - 1];
      else if (mode == "right_of") along += w[N - 1];
      // left edge of the forward carriageway relative to the divider
      const double leftEdge = fwdOnRight ? 0.0 : x.left.carriagewayWidth();
      const double tWay = leftEdge - along;  // t of the way relative to the divider (left +)
      x.laneOffset = -tWay;
    }
  }
  return x;
}

CrossSection reversedCrossSection(const CrossSection &x) {
  CrossSection r = x;
  std::swap(r.left, r.right);
  r.left.forward = !r.left.forward;
  r.right.forward = !r.right.forward;
  r.laneOffset = -x.laneOffset;
  return r;
}

odr::LaneSection makeLaneSection(const CrossSection &x, double s, const CountryProfile &c) {
  odr::LaneSection sec;
  sec.s = s;
  const bool motorwayLike = x.highway == "motorway" || x.highway == "trunk" || x.highway == "motorway_link" ||
                            x.highway == "trunk_link";
  auto build = [&](const SideSpec &side, int sign, std::vector<odr::Lane> &out) {
    int id = sign;
    const int nDrive = side.drivingCount();
    int driveSeen = 0;
    for (size_t i = 0; i < side.lanes.size(); ++i) {
      const LaneSpec &ls = side.lanes[i];
      odr::Lane l;
      l.id = id;
      id += sign;
      l.widths.push_back({0.0, ls.width, 0, 0, 0});
      switch (ls.kind) {
        case LaneKind::Driving: l.type = "driving"; break;
        case LaneKind::Bus: l.type = "driving"; l.accessDeny = {"passengerCar", "truck", "motorcycle"}; break;
        case LaneKind::Bidirectional: l.type = "bidirectional"; break;
        case LaneKind::Biking: l.type = "biking"; break;
        case LaneKind::Parking: l.type = "parking"; break;
        case LaneKind::Shoulder: l.type = "shoulder"; break;
        case LaneKind::Sidewalk:
          l.type = "sidewalk";
          l.heightInner = c.curbHeight;
          l.heightOuter = c.curbHeight;
          break;
      }
      if (ls.kind == LaneKind::Driving || ls.kind == LaneKind::Bus || ls.kind == LaneKind::Bidirectional) {
        l.speedMax = side.speed;
        l.material = "asphalt";
      }
      // outer road mark of this lane
      odr::RoadMark m;
      const bool isDrive = ls.kind == LaneKind::Driving || ls.kind == LaneKind::Bus;
      if (isDrive) ++driveSeen;
      const LaneKind next = i + 1 < side.lanes.size() ? side.lanes[i + 1].kind : LaneKind::Sidewalk;
      if (isDrive && driveSeen < nDrive) {
        m.type = "broken";
        m.color = c.edgeLineColor;
      } else if (isDrive && (next == LaneKind::Biking || next == LaneKind::Parking || motorwayLike)) {
        m.type = "solid";
        m.color = c.edgeLineColor;
        m.laneChange = "none";
      } else if (ls.kind == LaneKind::Bidirectional) {
        m.type = "solid broken";
        m.color = c.centerLineColor;
      } else if (ls.kind == LaneKind::Sidewalk || i + 1 == side.lanes.size()) {
        m.type = "none";
      } else if (ls.kind == LaneKind::Biking) {
        m.type = "solid";
        m.color = c.edgeLineColor;
      } else {
        m.type = "none";
      }
      if (ls.kind == LaneKind::Bus) m.weight = "bold";
      l.roadMarks.push_back(m);
      out.push_back(l);
    }
  };
  build(x.left, +1, sec.left);
  build(x.right, -1, sec.right);
  // centre line
  odr::RoadMark cm;
  const int nl = x.left.drivingCount(), nr = x.right.drivingCount();
  if (nl > 0 && nr > 0) {
    cm.type = (nl >= 2 && nr >= 2) ? c.centerLineMultiLane : c.centerLineTwoWay;
    cm.color = c.centerLineColor;
    cm.laneChange = cm.type == "broken" ? "both" : "none";
    if (!x.left.lanes.empty() && x.left.lanes.front().kind == LaneKind::Bidirectional) {
      cm.type = "broken solid";
    }
  } else if (motorwayLike) {
    cm.type = "solid";
    cm.color = c.centerLineColor == "yellow" ? "yellow" : c.edgeLineColor;  // left edge of a carriageway
    cm.laneChange = "none";
  } else {
    cm.type = "none";
  }
  sec.center.id = 0;
  sec.center.type = "none";
  sec.center.roadMarks.push_back(cm);
  return sec;
}

}  // namespace odrforge
