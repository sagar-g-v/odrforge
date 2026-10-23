#include "odrforge/validate.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace odrforge {

namespace {

// lanes carrying vehicle traffic: driving and the motorway lane types of OpenDRIVE 1.8.0 §11.7.1
const std::set<std::string> kVehicleLanes = {"driving", "entry", "exit", "onRamp", "offRamp", "connectingRamp", "bidirectional"};
const std::set<std::string> kLaneTypes14 = {"none", "driving", "stop", "shoulder", "biking", "sidewalk", "border",
                                            "restricted", "parking", "bidirectional", "median", "special1",
                                            "special2", "special3", "roadWorks", "tram", "rail", "entry", "exit",
                                            "offRamp", "onRamp"};
const std::set<std::string> kLaneTypes16Extra = {"curb", "connectingRamp"};
const std::set<std::string> kLaneTypes18Extra = {"walking", "slipLane", "taxi", "HOV", "bus"};
const std::set<std::string> kLaneTypes18Deprecated = {"sidewalk", "bidirectional", "special1", "special2", "special3",
                                                      "bus", "taxi", "HOV"};
const std::set<std::string> kMarkTypes = {"none", "solid", "broken", "solid solid", "solid broken", "broken solid",
                                          "broken broken", "botts dots", "grass", "curb", "custom", "edge"};
const std::set<std::string> kMarkColors = {"standard", "white", "yellow", "blue", "green", "red", "orange", "violet", "black"};
const std::set<std::string> kObjectTypes = {"none", "obstacle", "pole", "tree", "vegetation", "barrier", "building",
                                            "parkingSpace", "trafficIsland", "crosswalk", "gantry", "roadMark",
                                            "roadSurface", "streetLamp", "railing", "soundBarrier", "patch", "car",
                                            "van", "bus", "trailer", "bike", "motorbike", "tram", "train",
                                            "pedestrian", "wind"};
const std::set<std::string> kObjectTypesDeprecated18 = {"streetLamp", "railing", "soundBarrier", "patch", "car", "van",
                                                        "bus", "trailer", "bike", "motorbike", "tram", "train",
                                                        "pedestrian", "wind"};
const std::set<std::string> kRoadTypes = {"unknown", "rural", "motorway", "town", "lowSpeed", "pedestrian", "bicycle",
                                          "townExpressway", "townCollector", "townArterial", "townPrivate", "townLocal",
                                          "townPlayStreet"};
const std::set<std::string> kSpeedUnits = {"m/s", "mph", "km/h"};

double polyAtEval(const std::vector<odr::Poly3> &v, double s) {
  const odr::Poly3 *p = nullptr;
  for (const auto &e : v)
    if (e.s <= s + 1e-9) p = &e;
  return p ? p->eval(s - p->s) : 0.0;
}

bool isNumeric(const std::string &s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
}

}  // namespace

bool validLaneType(const std::string &type, int revMinor) {
  if (kLaneTypes14.count(type)) return true;
  if (revMinor >= 6 && kLaneTypes16Extra.count(type)) return true;
  if (revMinor >= 8 && kLaneTypes18Extra.count(type)) return true;
  return false;
}

bool carlaLaneType(const std::string &type) {
  static const std::set<std::string> c = {"driving", "stop", "shoulder", "biking", "sidewalk", "border", "restricted",
                                          "parking", "bidirectional", "median", "special1", "special2", "special3",
                                          "roadWorks", "tram", "rail", "entry", "exit", "offRamp", "onRamp", "none"};
  return c.count(type) > 0;
}

ValidationReport validate(const odr::OpenDrive &od, const TargetProfile &target) {
  ValidationReport R;
  const int rev = od.header.revMinor;
  auto report = [&](bool error, const std::string &rule, const std::string &m) {
    (error ? R.errors : R.warnings)++;
    R.ruleCounts[rule]++;
    if (R.messages.size() < 300) R.messages.push_back(std::string(error ? "ERROR " : "WARN  ") + "[" + rule + "] " + m);
  };
  auto err = [&](const std::string &rule, const std::string &m) { report(true, rule, m); };
  auto warn = [&](const std::string &rule, const std::string &m) { report(false, rule, m); };

  // ---- header (OpenDRIVE 1.8.0 §6.4, georeferencing §8.5)
  if (od.header.revMinor < 4 || od.header.revMinor > 9) err("spec1.2.revMinor", "unsupported revMinor");
  if (od.header.geoReference.empty()) warn("spec1.3.geoReference", "no geoReference");

  std::map<std::string, const odr::Road *> roads;
  std::map<std::string, const odr::Junction *> junctions;
  std::set<std::string> crossingRoads;  // OpenDRIVE 1.9 §12.5: a cross path's road, linked by its lane links
  std::set<std::string> boundaryRoads;  // roads a junction boundary runs along by their reference line
  for (const auto &j : od.junctions) {
    if (!junctions.emplace(j.id, &j).second) err("ids.id_unique_in_class", "duplicate junction id " + j.id);
    for (const auto &cp : j.crossPaths) crossingRoads.insert(cp.crossingRoad);
    // OpenDRIVE 1.8.0 §12.10: roads added to close a junction boundary "are not required to have any lanes"
    for (const auto &sg : j.boundary)
      if (sg.type == "lane" && sg.boundaryLane == 0) boundaryRoads.insert(sg.roadId);
  }
  for (const auto &r : od.roads) {
    if (!roads.emplace(r.id, &r).second) err("ids.id_unique_in_class", "duplicate road id " + r.id);
    if (target.name == "carla" && !isNumeric(r.id)) err("carla.numeric_ids", "road id not numeric " + r.id);
    if (target.maxIdLength && r.id.size() > target.maxIdLength) err("carla.id_length", "road id too long " + r.id);
  }

  for (const auto &r : od.roads) {
    const std::string R_ = "road " + r.id + ": ";
    const double L = r.length();
    if (r.junction != "-1" && !junctions.count(r.junction)) err("ids.only_ref_defined_ids", R_ + "unknown junction " + r.junction);
    // ---- planView (OpenDRIVE 1.8.0 §9.2 road reference line)
    if (r.planView.elements.empty()) err("road.geometry.refline_exists", R_ + "no geometry");
    double s = 0.0;
    for (size_t i = 0; i < r.planView.elements.size(); ++i) {
      const auto &e = r.planView.elements[i];
      if (std::abs(e.s - s) > 1e-6) err("road.geometry.s-value_sum", R_ + "geometry s is not the sum of previous lengths");
      if (i > 0 && e.s < r.planView.elements[i - 1].s) err("road.geometry.elem_asc_order", R_ + "geometry not ascending");
      if (e.length <= 0) err("spec2.length_positive", R_ + "non-positive geometry length");
      else if (e.length < 1e-3) err("spec2.min_geometry_length", R_ + "geometry shorter than 1 mm");
      else if (e.length < 0.01) warn("spec2.min_geometry_length", R_ + "geometry shorter than 1 cm");
      if (e.type() == GeomType::ParamPoly3) {
        const auto &c = *e.cubic;
        if (std::abs(c.aU) > 1e-9 || std::abs(c.aV) > 1e-9 || std::abs(c.bV) > 1e-9 || c.bU <= 0)
          warn("spec2.paramPoly3_local_frame", R_ + "paramPoly3 not aligned with its start pose (aU=aV=bV=0, bU>0)");
      }
      if (i > 0) {
        const Pose a = r.planView.elements[i - 1].end();
        const double gap = std::hypot(a.x - e.x, a.y - e.y);
        R.maxGeometryGap = std::max(R.maxGeometryGap, gap);
        if (gap > 1e-3) err("road.geometry.refline_no_gaps", R_ + "geometry gap " + std::to_string(gap) + " m");
        const double dh = std::abs(normalizeAngle(a.hdg - e.hdg));
        if (dh > 1e-4) err("road.geometry.refline_no_kinks", R_ + "heading jump " + std::to_string(dh) + " rad");
        else if (dh > 1e-6) warn("road.geometry.refline_no_kinks", R_ + "heading jump above 1e-6 rad");
      }
      s += e.length;
    }
    // ---- elevation / laneOffset records (OpenDRIVE 1.8.0 §10.5, §11.4): ascending and continuous
    auto profile = [&](const std::vector<odr::Poly3> &v, const char *rule, const char *what) {
      for (size_t i = 1; i < v.size(); ++i) {
        if (v[i].s < v[i - 1].s) err(rule, R_ + what + " records not ascending");
        const double jump = std::abs(v[i - 1].eval(v[i].s - v[i - 1].s) - v[i].a);
        if (jump > 1e-3) err(rule, R_ + what + " jumps by " + std::to_string(jump) + " m at s=" + std::to_string(v[i].s));
      }
      if (!v.empty() && v.front().s > 1e-6) warn(rule, R_ + what + " does not start at s=0");
    };
    profile(r.elevation, "spec3.elevation_continuity", "elevation");
    profile(r.laneOffset, "spec4.1.laneOffset_continuity", "laneOffset");
    for (const auto &t : r.types) {
      if (!kRoadTypes.count(t.type)) err("spec3.road_type_enum", R_ + "road type " + t.type);
      if (rev >= 8 && !t.country.empty() && t.country.size() != 2) err("road.type.only_alpha_2_country_codes", R_ + "country " + t.country);
      if (t.speedMax && !kSpeedUnits.count(t.speedUnit)) err("spec3.speed_unit", R_ + "road speed unit " + t.speedUnit);
    }
    // ---- lane sections (OpenDRIVE 1.8.0 §11.3)
    if (r.lanes.empty()) err("road.lane_section.lane_sect_req", R_ + "no lane section");
    if (!r.lanes.empty() && r.lanes.front().s > 1e-6) err("road.lane.lane_sect_first", R_ + "first lane section does not start at 0");
    for (size_t si = 0; si < r.lanes.size(); ++si) {
      const auto &sec = r.lanes[si];
      const double s1 = si + 1 < r.lanes.size() ? r.lanes[si + 1].s : L;
      const double secLen = s1 - sec.s;
      const std::string S_ = R_ + "section " + std::to_string(si) + ": ";
      if (secLen <= 1e-6) err("road.lane_section.valid_length", S_ + "zero length");
      else if (secLen < 0.1) warn("road.lane_section.valid_length", S_ + "shorter than 0.1 m");
      if (sec.center.id != 0) err("road.lane.center_lane_singular", S_ + "center lane id is not 0");
      if (!sec.center.widths.empty() || sec.center.speedMax || !sec.center.accessDeny.empty())
        err("spec4.1.center_lane_properties", S_ + "center lane carries width / speed / access");
      if (sec.left.empty() && sec.right.empty() && !(boundaryRoads.count(r.id) && r.junction != "-1"))
        err("road.lane.lane_section_drivable", S_ + "no lanes besides the center lane");
      for (int sign : {1, -1}) {
        const auto &side = sign > 0 ? sec.left : sec.right;
        for (size_t k = 0; k < side.size(); ++k) {
          const odr::Lane &l = side[k];
          const std::string Ls = S_ + "lane " + std::to_string(l.id) + ": ";
          if (l.id != sign * static_cast<int>(k + 1)) err("road.lane.lane_order_no_gaps", Ls + "ids not contiguous 1..n / -1..-m");
          if (!validLaneType(l.type, rev)) err("spec4.3.lane_type_enum", Ls + "type '" + l.type + "' not valid in 1." + std::to_string(rev));
          if (target.name == "carla" && !carlaLaneType(l.type)) err("carla.lane_type", Ls + "type unknown to CARLA");
          // width (OpenDRIVE 1.8.0 §11.6)
          if (l.widths.empty() || std::abs(l.widths.front().s) > 1e-9) err("road.lane.width.sOffset_zero", Ls + "no width entry at sOffset 0");
          for (size_t w = 1; w < l.widths.size(); ++w) {
            if (l.widths[w].s < l.widths[w - 1].s) err("road.lane.width.elem_asc_order", Ls + "width records not ascending");
            const double jump = std::abs(l.widths[w - 1].eval(l.widths[w].s - l.widths[w - 1].s) - l.widths[w].a);
            if (jump > 1e-3) warn("spec4.4.width_continuity", Ls + "width jumps by " + std::to_string(jump) + " m");
          }
          for (int q = 0; q <= 16 && secLen > 0; ++q)
            if (polyAtEval(l.widths, secLen * q / 16) < -1e-6) {
              err("road.lane.width.lane_width_validity", Ls + "negative width");
              break;
            }
          for (const auto &m : l.roadMarks) {
            if (!kMarkTypes.count(m.type)) err("spec4.5.roadMark_type", Ls + "roadMark type " + m.type);
            if (!kMarkColors.count(m.color)) err("spec4.5.roadMark_color", Ls + "roadMark color " + m.color);
            if (m.weight != "standard" && m.weight != "bold") err("spec4.5.roadMark_weight", Ls + "roadMark weight " + m.weight);
          }
          if (l.speedMax && !kSpeedUnits.count(l.speedUnit)) err("spec4.7.speed_unit", Ls + "lane speed unit " + l.speedUnit);
          // lane links inside the road (OpenDRIVE 1.8.0 §11.5): must exist in the neighbouring section, consistently
          if (l.link.predecessor && si > 0) {
            const auto *p = odr::findLane(r.lanes[si - 1], *l.link.predecessor);
            if (!p) err("spec4.6.lane_link_target", Ls + "predecessor lane missing in previous section");
            else if (!p->link.successor || *p->link.successor != l.id) warn("spec4.6.lane_link_consistency", Ls + "predecessor does not link back");
          }
          if (l.link.successor && si + 1 < r.lanes.size()) {
            if (!odr::findLane(r.lanes[si + 1], *l.link.successor)) err("spec4.6.lane_link_target", Ls + "successor lane missing in next section");
          }
          // a lane with zero width at a section joint must not be linked across it (OpenDRIVE 1.8.0 §11.5,
          // rules road.lane.link.zero_width_at_start / zero_width_at_end)
          if (si > 0 && l.link.predecessor && odr::laneWidthAt(l, 0.0) < 1e-6)
            err("road.lane.link.zero_width_at_start", Ls + "starts with zero width but has a predecessor");
          if (si + 1 < r.lanes.size() && l.link.successor && odr::laneWidthAt(l, secLen) < 1e-6)
            err("road.lane.link.zero_width_at_end", Ls + "ends with zero width but has a successor");
        }
      }
    }
    // ---- road links (OpenDRIVE 1.8.0 §10.3)
    for (bool pred : {true, false}) {
      const auto &lt = pred ? r.predecessor : r.successor;
      if (!lt) continue;
      if (lt->elementType == "junction") {
        auto jt = junctions.find(lt->elementId);
        if (jt == junctions.end()) {
          err("ids.only_ref_defined_ids", R_ + "unknown junction " + lt->elementId);
          continue;
        }
        if (r.junction != "-1") err("spec6.1.connecting_road_links", R_ + "connecting road links to a junction");
        bool known = false;  // the junction must know this road (incoming, or target of a connecting road)
        for (const auto &c : jt->second->connections) {
          if (c.incomingRoad == r.id) known = true;
          // OpenDRIVE 1.9 §12.6: a direct junction names the road it links to (@linkedRoad)
          if (jt->second->type == "direct" && c.connectingRoad == r.id) known = true;
          auto cr = roads.find(c.connectingRoad);
          if (cr != roads.end() && cr->second->successor && cr->second->successor->elementId == r.id) known = true;
        }
        if (!known) warn("road.linkage.both_sides_consistency", R_ + "junction " + lt->elementId + " has no connection with this road");
      } else if (lt->elementType == "road") {
        auto ot = roads.find(lt->elementId);
        if (ot == roads.end()) {
          err("ids.only_ref_defined_ids", R_ + "unknown road " + lt->elementId);
          continue;
        }
        if (lt->contactPoint != "start" && lt->contactPoint != "end") err("spec5.contactPoint", R_ + "road link without contactPoint");
        const odr::Road &o = *ot->second;
        if (r.junction == "-1" && o.junction == "-1") {  // direct road-road joint: reference lines must meet
          const Pose a = r.planView.eval(pred ? 0.0 : L);
          const Pose b = o.planView.eval(lt->contactPoint == "end" ? o.length() : 0.0);
          if (std::hypot(a.x - b.x, a.y - b.y) > 1e-3) err("road.linkage.position", R_ + "reference line does not meet road " + o.id);
          double dh = std::abs(normalizeAngle(a.hdg - b.hdg));
          dh = std::min(dh, std::abs(kPi - dh));
          if (dh > 1e-4) err("road.linkage.heading", R_ + "heading differs from road " + o.id);
        }
      } else {
        err("spec5.elementType", R_ + "link elementType " + lt->elementType);
      }
    }
    // ---- connecting roads: lane-level fit at both ends (OpenDRIVE 1.8.0 §12.4 "The linked lanes shall
    // fit smoothly", 1.9.0 rule junctions.connection.smooth_fit)
    if (r.junction != "-1" && !r.lanes.empty() && !crossingRoads.count(r.id)) {
      if (!r.predecessor || !r.successor) err("spec6.1.connecting_road_links", R_ + "connecting road without predecessor and successor");
      const auto &first = r.lanes.front(), &last = r.lanes.back();
      for (const auto *side : {&first.left, &first.right})
        for (const auto &l : *side) {
          for (bool pred : {true, false}) {
            const auto &lt = pred ? r.predecessor : r.successor;
            const odr::Lane *cl = odr::findLane(pred ? first : last, l.id);
            if (!lt || lt->elementType != "road" || !cl) continue;
            const auto &ll = pred ? cl->link.predecessor : cl->link.successor;
            if (!ll) {
              err("spec6.1.connecting_lane_links", R_ + "lane " + std::to_string(l.id) + " without " + (pred ? "predecessor" : "successor"));
              continue;
            }
            auto it = roads.find(lt->elementId);
            if (it == roads.end()) continue;
            const odr::Road &o = *it->second;
            const double so = lt->contactPoint == "end" ? o.length() : 0.0;
            const double sc = pred ? 0.0 : L;
            Pose a, b;
            if (!odr::laneCenter(o, so, *ll, a)) {
              err("spec6.1.connecting_lane_links", R_ + "lane " + std::to_string(*ll) + " missing on road " + o.id);
              continue;
            }
            odr::laneCenter(r, sc, l.id, b);
            const double gap = std::hypot(a.x - b.x, a.y - b.y);
            R.maxLinkGap = std::max(R.maxLinkGap, gap);
            double dh = std::abs(normalizeAngle(a.hdg - b.hdg));
            dh = std::min(dh, std::abs(kPi - dh));
            R.maxLinkHeading = std::max(R.maxLinkHeading, dh);
            if (gap > 0.02) err("junctions.connection.smooth_fit", R_ + "lane gap " + std::to_string(gap) + " m to road " + o.id);
            if (dh > 1e-3) err("junctions.connection.smooth_fit", R_ + "lane heading differs from road " + o.id);
            // lane width and elevation must continue across the link
            const odr::LaneSection &os = odr::sectionAt(o, so);
            if (const odr::Lane *ol = odr::findLane(os, *ll)) {
              const double wo = odr::laneWidthAt(*ol, so - os.s);
              const odr::LaneSection &cs = pred ? first : last;
              const double wc = odr::laneWidthAt(*cl, sc - cs.s);
              R.maxLinkWidthDiff = std::max(R.maxLinkWidthDiff, std::abs(wo - wc));
              if (std::abs(wo - wc) > 0.02) err("spec10.lane_width_at_junction", R_ + "lane width differs from road " + o.id);
            }
            // surface height at the two lane centres (elevation + banking, OpenDRIVE 1.8.0 §10.5)
            double oa, ob, ca, cb;
            odr::laneBorders(o, so, *ll, oa, ob);
            odr::laneBorders(r, sc, l.id, ca, cb);
            const double dz = std::abs(odr::surfaceZ(o, so, 0.5 * (oa + ob)) - odr::surfaceZ(r, sc, 0.5 * (ca + cb)));
            R.maxLinkZGap = std::max(R.maxLinkZGap, dz);
            if (dz > 0.01) err("spec10.elevation_at_links", R_ + "elevation differs from road " + o.id);
          }
        }
    }
  }

  // ---- signals (OpenDRIVE 1.8.0 §14) and objects (§13)
  std::set<std::string> signalIds, dynamicSignals, objectIds;
  for (const auto &r : od.roads) {
    const bool lhtRoad = r.rule == "LHT";
    for (const auto &s : r.signals) {
      const std::string S_ = "signal " + s.id + ": ";
      if (!signalIds.insert(s.id).second) err("ids.id_unique_in_class", S_ + "duplicate id");
      if (s.dynamic) dynamicSignals.insert(s.id);
      if (target.maxIdLength && s.id.size() > target.maxIdLength) err("carla.id_length", S_ + "id too long");
      if (s.s < -1e-6 || s.s > r.length() + 1e-6) err("spec7.1.s_range", S_ + "s outside road " + r.id);
      if (s.orientation != "+" && s.orientation != "-" && s.orientation != "none") err("spec7.1.orientation", S_ + "orientation " + s.orientation);
      if (s.value && s.unit.empty()) err("spec7.1.value_unit", S_ + "value without unit");
      if (rev >= 8 && s.country != "OpenDRIVE" && s.country.size() != 2) err("spec7.1.country_alpha2", S_ + "country " + s.country);
      if (s.type.empty()) err("road.signal.signal_type", S_ + "no type");
      for (const auto &v : s.validity) {
        if (v.fromLane > v.toLane) err("spec7.1.validity_order", S_ + "fromLane > toLane");
        for (int id : {v.fromLane, v.toLane}) {
          if (id == 0) continue;
          if (!odr::findLane(odr::sectionAt(r, s.s), id)) err("spec7.1.validity_lanes", S_ + "validity lane " + std::to_string(id) + " missing");
          // '+' applies to traffic along +s: right lanes in RHT, left lanes in LHT
          const bool plusLane = (id < 0) != lhtRoad;
          if ((s.orientation == "+" && !plusLane) || (s.orientation == "-" && plusLane))
            err("spec7.1.validity_direction", S_ + "validity lane " + std::to_string(id) + " drives against the signal orientation");
        }
      }
    }
    for (const auto &o : r.objects) {
      const std::string O_ = "object " + o.id + ": ";
      if (!objectIds.insert(o.id).second) err("ids.id_unique_in_class", O_ + "duplicate id");
      if (o.s < -1e-6 || o.s > r.length() + 1e-6) err("spec8.1.s_range", O_ + "s outside road " + r.id);
      if (!o.type.empty() && !kObjectTypes.count(o.type)) err("spec8.1.type_enum", O_ + "type " + o.type);
      if (rev >= 8 && kObjectTypesDeprecated18.count(o.type)) warn("spec8.1.type_deprecated", O_ + "type " + o.type + " deprecated in 1.8");
      if (o.radius > 0 && o.length > 0) err("road.object.circular_vs_angular", O_ + "both radius and length");
      // 1.9.0 road.object.repeating.valid_s_length: a repeat lies on its road
      for (const auto &rp : o.repeats)
        if (rp.s < -1e-6 || rp.length < 0 || rp.s + rp.length > r.length() + 1e-6)
          err("road.object.repeating.valid_s_length", O_ + "repeat [" + std::to_string(rp.s) + ", " +
                                                          std::to_string(rp.s + rp.length) + "] outside road " + r.id);
      if (!o.outline.empty()) {
        if (o.outline.size() < 2) err("road.corner_local.element_min_amount", O_ + "outline with fewer than 2 corners");
        double area = 0;
        for (size_t i = 0; i < o.outline.size(); ++i) {
          const auto &a = o.outline[i], &b = o.outline[(i + 1) % o.outline.size()];
          area += a.u * b.v - b.u * a.v;
        }
        if (o.outline.size() >= 3 && area < 0) warn("spec8.2.outline_ccw", O_ + "outline is clockwise");
      }
    }
    for (const auto &b : r.bridges)
      if (!objectIds.insert(b.id).second) err("ids.id_unique_in_class", "bridge " + b.id + ": duplicate object id");
    for (const auto &t : r.tunnels)
      if (!objectIds.insert(t.id).second) err("ids.id_unique_in_class", "tunnel " + t.id + ": duplicate object id");
  }
  // ---- controllers (OpenDRIVE 1.8.0 §14.6)
  std::set<std::string> ctrlIds, referenced;
  for (const auto &c : od.controllers) {
    if (!ctrlIds.insert(c.id).second) err("ids.id_unique_in_class", "duplicate controller id " + c.id);
    if (c.signalIds.empty()) err("road.signal.controller.valid_for_signals", "controller " + c.id + " controls no signal");
    for (const auto &sid : c.signalIds) {
      if (!signalIds.count(sid)) err("ids.only_ref_defined_ids", "controller " + c.id + " references unknown signal " + sid);
      else if (!dynamicSignals.count(sid)) warn("spec7.2.control_dynamic", "controller " + c.id + " controls static signal " + sid);
    }
  }
  // ---- junctions (OpenDRIVE 1.8.0 §12)
  for (const auto &j : od.junctions) {
    const std::string J_ = "junction " + j.id + ": ";
    std::set<std::pair<std::string, std::string>> pairs;
    for (const auto &cn : j.connections) {
      auto ir = roads.find(cn.incomingRoad), cr = roads.find(cn.connectingRoad);
      if (ir == roads.end() || cr == roads.end()) {
        err("ids.only_ref_defined_ids", J_ + "unknown road in connection " + cn.id);
        continue;
      }
      if (!pairs.insert({cn.incomingRoad, cn.connectingRoad}).second) err("junctions.connection.one_link_to_incoming", J_ + "duplicate connection pair");
      if (ir->second->junction != "-1") err("junctions.connection.connect_road_no_incoming_road", J_ + "incoming road " + cn.incomingRoad + " is a connecting road");
      if (j.type != "direct" && cr->second->junction != j.id) err("spec6.1.connecting_road_junction", J_ + "connecting road " + cn.connectingRoad + " not in junction");
      if (cn.contactPoint != "start" && cn.contactPoint != "end") err("spec6.1.contactPoint", J_ + "connection without contactPoint");
      if (cn.laneLinks.empty()) warn("spec6.1.laneLink_required", J_ + "connection " + cn.id + " without laneLink");
      const odr::Road &in = *ir->second;
      const odr::Road &con = *cr->second;
      // the incoming road's end at this junction; a loop road touches it with both ends, so the
      // connecting road's own link (contactPoint on the incoming road) decides
      bool atEnd = in.successor && in.successor->elementType == "junction" && in.successor->elementId == j.id;
      const auto &back = cn.contactPoint == "end" ? con.successor : con.predecessor;
      if (back && back->elementType == "road" && back->elementId == in.id && !back->contactPoint.empty())
        atEnd = back->contactPoint == "end";
      const odr::LaneSection &secIn = odr::sectionAt(in, atEnd ? in.length() : 0.0);
      if (con.lanes.empty()) continue;
      const odr::LaneSection &secCon = cn.contactPoint == "end" ? con.lanes.back() : con.lanes.front();
      for (const auto &ll : cn.laneLinks) {
        if (!odr::findLane(secIn, ll.from)) err("spec6.1.laneLink_from", J_ + "laneLink from " + std::to_string(ll.from) + " missing on road " + in.id);
        if (!odr::findLane(secCon, ll.to)) err("spec6.1.laneLink_to", J_ + "laneLink to " + std::to_string(ll.to) + " missing on road " + con.id);
      }
    }
    for (const auto &c : j.controllers) {
      referenced.insert(c.id);
      if (!ctrlIds.count(c.id)) err("ids.only_ref_defined_ids", J_ + "unknown controller " + c.id);
    }
  }
  for (const auto &c : od.controllers)
    if (!referenced.count(c.id)) warn("spec7.2.controller_referenced", "controller " + c.id + " not referenced by any junction");

  // ======================= visual quality (no jerks, no overlaps, one surface per junction)
  for (const auto &r : od.roads) {
    const std::string R_ = "road " + r.id + ": ";
    for (size_t i = 1; i < r.elevation.size(); ++i) {
      const auto &a = r.elevation[i - 1], &b = r.elevation[i];
      if (std::abs(a.deriv(b.s - a.s) - b.b) > 0.005) err("quality.elevation_c1", R_ + "grade jumps at s=" + std::to_string(b.s));
    }
    for (size_t i = 1; i < r.superelevation.size(); ++i) {
      const auto &a = r.superelevation[i - 1], &b = r.superelevation[i];
      if (std::abs(a.eval(b.s - a.s) - b.a) > 1e-3) err("quality.superelevation_c0", R_ + "banking jumps at s=" + std::to_string(b.s));
    }
    for (double s = 0; s <= r.length(); s += 5.0)
      if (std::abs(odr::elevationSlopeAt(r, s)) > 0.20) {
        warn("quality.max_grade", R_ + "grade above 20 % at s=" + std::to_string(s));
        break;
      }
    // grade continuity across connecting-road links
    if (r.junction != "-1")
      for (bool pred : {true, false}) {
        const auto &lt = pred ? r.predecessor : r.successor;
        if (!lt || lt->elementType != "road") continue;
        auto it = roads.find(lt->elementId);
        if (it == roads.end()) continue;
        const odr::Road &o = *it->second;
        const double so = lt->contactPoint == "end" ? o.length() : 0.0, sc = pred ? 0.0 : r.length();
        const double align = std::cos(o.planView.eval(so).hdg - r.planView.eval(sc).hdg) >= 0 ? 1.0 : -1.0;
        const double g1 = odr::elevationSlopeAt(o, so) * align, g2 = odr::elevationSlopeAt(r, sc);
        if (std::abs(g1 - g2) > 0.01) err("quality.link_grade", R_ + "grade " + std::to_string(g2) + " vs " + std::to_string(g1) + " on road " + o.id);
      }
  }
  // every road's element midpoints, for the many projections onto it below
  std::vector<std::vector<Vec2>> mids(od.roads.size());
  for (size_t i = 0; i < od.roads.size(); ++i) mids[i] = od.roads[i].planView.midpoints();
  auto midsOf = [&](const odr::Road &q) -> const std::vector<Vec2> & { return mids[static_cast<size_t>(&q - od.roads.data())]; };
  // one surface per junction: overlapping connecting lanes must agree in height
  {
    std::map<std::string, std::vector<const odr::Road *>> byJunction;
    for (const auto &r : od.roads)
      if (r.junction != "-1") byJunction[r.junction].push_back(&r);
    for (const auto &[jid, rs] : byJunction) {
      struct P {
        double x, y, z;
        size_t road;
      };
      std::map<std::pair<long, long>, std::vector<P>> cells;
      for (size_t k = 0; k < rs.size(); ++k) {
        const odr::Road &r = *rs[k];
        const auto &sec = r.lanes.front();
        const int id = !sec.right.empty() ? sec.right.front().id : (!sec.left.empty() ? sec.left.front().id : 0);
        if (!id) continue;
        for (double s = 0.5; s < r.length() - 0.5; s += 1.0) {
          Pose c;
          double a, b;
          if (!odr::laneCenter(r, s, id, c) || !odr::laneBorders(r, s, id, a, b)) continue;
          const double z = odr::surfaceZ(r, s, 0.5 * (a + b));
          cells[{static_cast<long>(std::floor(c.x / 0.5)), static_cast<long>(std::floor(c.y / 0.5))}].push_back({c.x, c.y, z, k});
        }
      }
      std::set<std::pair<size_t, size_t>> reported;
      for (const auto &[key, pts] : cells)
        for (size_t i = 0; i < pts.size(); ++i)
          for (size_t j = i + 1; j < pts.size(); ++j) {
            if (pts[i].road == pts[j].road || std::hypot(pts[i].x - pts[j].x, pts[i].y - pts[j].y) > 0.5) continue;
            // compare both surfaces at the same point: project point i onto road j
            const odr::Road &q = *rs[pts[j].road];
            double sq, tq;
            q.planView.project({pts[i].x, pts[i].y}, sq, tq, midsOf(q));
            const Pose pq = q.planView.eval(sq);
            if (std::abs((Vec2(pts[i].x, pts[i].y) - pq.pos()).dot(pq.dir())) > 0.05) continue;
            const double dz = std::abs(odr::surfaceZ(q, sq, tq) - pts[i].z);
            if (dz <= 0.01) continue;
            const auto pr = std::minmax(pts[i].road, pts[j].road);
            if (reported.insert(pr).second)
              err("quality.junction_surface", "junction " + jid + ": roads " + rs[pr.first]->id + " and " + rs[pr.second]->id +
                                                  " differ by " + std::to_string(dz) + " m where they overlap");
          }
    }
  }
  // separate roads must not overlap outside junctions at the same level
  {
    struct S {
      size_t road;
      double s;
    };
    std::vector<const odr::Road *> rs;
    for (const auto &r : od.roads)
      if (r.junction == "-1") rs.push_back(&r);
    std::map<std::pair<long, long>, std::vector<S>> grid;
    const double cell = 20.0;
    for (size_t k = 0; k < rs.size(); ++k)
      for (double s = 0; s <= rs[k]->length(); s += 4.0) {
        const Pose p = rs[k]->planView.eval(s);
        grid[{static_cast<long>(std::floor(p.x / cell)), static_cast<long>(std::floor(p.y / cell))}].push_back({k, s});
      }
    std::set<std::pair<size_t, size_t>> reported;
    for (size_t k = 0; k < rs.size(); ++k) {
      const odr::Road &r = *rs[k];
      for (double s = 1.0; s < r.length() - 1.0; s += 2.0) {
        const odr::LaneSection &sec = odr::sectionAt(r, s);
        for (const auto *side : {&sec.left, &sec.right})
          for (const auto &l : *side) {
            if (!kVehicleLanes.count(l.type)) continue;
            Pose c;
            double a, b;
            if (!odr::laneCenter(r, s, l.id, c) || !odr::laneBorders(r, s, l.id, a, b) || std::abs(b - a) < 0.5) continue;
            const double z = odr::surfaceZ(r, s, 0.5 * (a + b));
            std::set<size_t> cand;
            const long cx = static_cast<long>(std::floor(c.x / cell)), cy = static_cast<long>(std::floor(c.y / cell));
            for (long dx = -1; dx <= 1; ++dx)
              for (long dy = -1; dy <= 1; ++dy)
                if (auto it = grid.find({cx + dx, cy + dy}); it != grid.end())
                  for (const auto &smp : it->second)
                    if (smp.road != k) cand.insert(smp.road);
            for (size_t o : cand) {
              if (reported.count(std::minmax(k, o))) continue;
              const odr::Road &q = *rs[o];
              double so, to;
              q.planView.project(c.pos(), so, to, midsOf(q));
              if (so < 1.0 || so > q.length() - 1.0) continue;
              const Pose po = q.planView.eval(so);
              if (std::abs((c.pos() - po.pos()).dot(po.dir())) > 0.3) continue;
              const odr::LaneSection &qs = odr::sectionAt(q, so);
              bool inside = false;
              for (const auto *qside : {&qs.left, &qs.right})
                for (const auto &ql : *qside) {
                  double qa, qb;
                  if (!kVehicleLanes.count(ql.type) || !odr::laneBorders(q, so, ql.id, qa, qb)) continue;
                  if (to > std::min(qa, qb) + 0.2 && to < std::max(qa, qb) - 0.2) inside = true;
                }
              if (inside && std::abs(odr::surfaceZ(q, so, to) - z) < 3.0) {
                reported.insert(std::minmax(k, o));
                err("quality.road_overlap", "roads " + r.id + " (z " + std::to_string(z) + ") and " + q.id + " (z " +
                                                std::to_string(odr::surfaceZ(q, so, to)) + ") overlap at the same level near (" +
                                                std::to_string(static_cast<int>(c.x)) + ", " + std::to_string(static_cast<int>(c.y)) + ")");
              }
            }
          }
      }
    }
  }
  return R;
}

}  // namespace odrforge
