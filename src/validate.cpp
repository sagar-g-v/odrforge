#include "odrforge/validate.hpp"

#include <cmath>
#include <map>
#include <set>

namespace odrforge {

ValidationReport validate(const odr::OpenDrive &od, const TargetProfile &target) {
  ValidationReport R;
  auto err = [&](const std::string &m) {
    ++R.errors;
    if (R.messages.size() < 200) R.messages.push_back("ERROR " + m);
  };
  auto warn = [&](const std::string &m) {
    ++R.warnings;
    if (R.messages.size() < 200) R.messages.push_back("WARN  " + m);
  };
  std::map<std::string, const odr::Road *> roads;
  for (const auto &r : od.roads) {
    if (!roads.emplace(r.id, &r).second) err("duplicate road id " + r.id);
    if (target.maxIdLength && r.id.size() > target.maxIdLength) err("road id too long " + r.id);
    // planView continuity
    double s = 0.0;
    for (size_t i = 0; i < r.planView.elements.size(); ++i) {
      const auto &e = r.planView.elements[i];
      if (std::abs(e.s - s) > 1e-6) err("road " + r.id + ": geometry s not cumulative");
      if (e.length <= 0) err("road " + r.id + ": non-positive geometry length");
      if (i > 0) {
        const Pose a = r.planView.elements[i - 1].end();
        const double gap = std::hypot(a.x - e.x, a.y - e.y);
        R.maxGeometryGap = std::max(R.maxGeometryGap, gap);
        if (gap > 1e-3) err("road " + r.id + ": geometry gap " + std::to_string(gap) + " m");
        if (std::abs(normalizeAngle(a.hdg - e.hdg)) > 1e-4) err("road " + r.id + ": heading jump between elements");
      }
      s += e.length;
    }
    if (r.lanes.empty()) err("road " + r.id + ": no lane section");
    double prevS = -1;
    for (const auto &sec : r.lanes) {
      if (sec.s <= prevS || sec.s > r.length() + 1e-6) err("road " + r.id + ": lane section s out of order");
      prevS = sec.s;
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side)
          for (const auto &w : l.widths)
            if (w.a < -1e-6) err("road " + r.id + " lane " + std::to_string(l.id) + ": negative width");
    }
  }
  // lane-level link geometry for connecting roads
  for (const auto &r : od.roads) {
    if (r.junction == "-1") continue;
    for (const auto &l : (r.lanes.front().right.empty() ? r.lanes.front().left : r.lanes.front().right)) {
      for (bool pred : {true, false}) {
        const auto &lt = pred ? r.predecessor : r.successor;
        const auto &ll = pred ? l.link.predecessor : l.link.successor;
        if (!lt || !ll) {
          err("connecting road " + r.id + ": missing " + (pred ? "predecessor" : "successor"));
          continue;
        }
        auto it = roads.find(lt->elementId);
        if (it == roads.end()) {
          err("connecting road " + r.id + ": unknown road " + lt->elementId);
          continue;
        }
        const odr::Road &o = *it->second;
        const double so = lt->contactPoint == "end" ? o.length() : 0.0;
        const double sc = pred ? 0.0 : r.length();
        Pose a, b;
        if (!odr::laneCenter(o, so, *ll, a)) {
          err("connecting road " + r.id + ": lane " + std::to_string(*ll) + " missing on road " + o.id);
          continue;
        }
        odr::laneCenter(r, sc, l.id, b);
        const double gap = std::hypot(a.x - b.x, a.y - b.y);
        R.maxLinkGap = std::max(R.maxLinkGap, gap);
        double dh = std::abs(normalizeAngle(a.hdg - b.hdg));
        dh = std::min(dh, std::abs(kPi - dh));  // either direction of the linked road
        R.maxLinkHeading = std::max(R.maxLinkHeading, dh);
        if (gap > 0.02) err("connecting road " + r.id + ": lane gap " + std::to_string(gap) + " m to road " + o.id);
        if (dh > 1e-3) warn("connecting road " + r.id + ": heading mismatch to road " + o.id);
      }
    }
  }
  // junctions and controllers
  std::set<std::string> ctrlIds, signalIds;
  for (const auto &r : od.roads)
    for (const auto &s : r.signals) {
      if (!signalIds.insert(s.id).second) err("duplicate signal id " + s.id);
      if (target.maxIdLength && s.id.size() > target.maxIdLength) err("signal id too long " + s.id);
    }
  for (const auto &c : od.controllers) {
    ctrlIds.insert(c.id);
    for (const auto &sid : c.signalIds)
      if (!signalIds.count(sid)) err("controller " + c.id + " references unknown signal " + sid);
  }
  std::set<std::string> referenced;
  for (const auto &j : od.junctions) {
    for (const auto &cn : j.connections) {
      auto ci = roads.find(cn.connectingRoad);
      if (!roads.count(cn.incomingRoad) || ci == roads.end()) err("junction " + j.id + ": unknown road in connection");
      else if (ci->second->junction != j.id) err("junction " + j.id + ": connecting road " + cn.connectingRoad + " not in junction");
    }
    for (const auto &c : j.controllers) {
      referenced.insert(c.id);
      if (!ctrlIds.count(c.id)) err("junction " + j.id + ": unknown controller " + c.id);
    }
  }
  for (const auto &c : od.controllers)
    if (!referenced.count(c.id)) warn("controller " + c.id + " not referenced by any junction");
  return R;
}

}  // namespace odrforge
