#include "odrforge/rules.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace odrforge {

namespace {
template <class M>
auto lookup(const M &m, const std::string &cls) -> std::optional<typename M::mapped_type> {
  if (auto it = m.find(cls); it != m.end()) return it->second;
  // *_link classes fall back to their parent class
  if (cls.size() > 5 && cls.compare(cls.size() - 5, 5, "_link") == 0)
    if (auto it = m.find(cls.substr(0, cls.size() - 5)); it != m.end()) return it->second;
  if (auto it = m.find("default"); it != m.end()) return it->second;
  return std::nullopt;
}
}  // namespace

double CountryProfile::width(const std::string &cls) const { return lookup(laneWidth, cls).value_or(3.25); }
std::optional<double> CountryProfile::speed(const std::string &cls) const { return lookup(defaultSpeed, cls); }
int CountryProfile::lanesOneway(const std::string &cls) const { return lookup(defaultLanesOneway, cls).value_or(1); }
int CountryProfile::lanesTwoway(const std::string &cls) const { return lookup(defaultLanesTwoway, cls).value_or(2); }

CountryProfile CountryProfile::load(const std::string &code, const std::string &rulesDir) {
  const std::string dir = rulesDir.empty() ? std::string(ODRFORGE_RULES_DIR) : rulesDir;
  std::ifstream f(dir + "/countries/" + code + ".json");
  if (!f) {
    f.open(dir + "/countries/XX.json");
    if (!f) throw std::runtime_error("no country rules in " + dir + "/countries");
  }
  const auto j = nlohmann::json::parse(f, nullptr, true, true);
  CountryProfile p;
  p.code = j.value("code", code);
  p.leftHandTraffic = j.value("drivingSide", "right") == "left";
  p.speedUnit = j.value("speedUnit", "km/h");
  auto readMap = [&](const char *k, auto &out) {
    if (j.contains(k))
      for (auto &[key, v] : j[k].items()) out[key] = v.get<typename std::decay_t<decltype(out)>::mapped_type>();
  };
  readMap("laneWidth", p.laneWidth);
  readMap("defaultSpeed", p.defaultSpeed);
  readMap("defaultLanesOneway", p.defaultLanesOneway);
  readMap("defaultLanesTwoway", p.defaultLanesTwoway);
  readMap("shoulderWidth", p.shoulderWidth);
  readMap("maxspeedZones", p.maxspeedZones);
  p.sidewalkWidth = j.value("sidewalkWidth", p.sidewalkWidth);
  p.bikeLaneWidth = j.value("bikeLaneWidth", p.bikeLaneWidth);
  p.parkingWidthParallel = j.value("parkingWidthParallel", p.parkingWidthParallel);
  p.parkingWidthDiagonal = j.value("parkingWidthDiagonal", p.parkingWidthDiagonal);
  p.parkingWidthPerpendicular = j.value("parkingWidthPerpendicular", p.parkingWidthPerpendicular);
  p.curbHeight = j.value("curbHeight", p.curbHeight);
  p.centerLineColor = j.value("centerLineColor", p.centerLineColor);
  p.edgeLineColor = j.value("edgeLineColor", p.edgeLineColor);
  p.centerLineTwoWay = j.value("centerLineTwoWay", p.centerLineTwoWay);
  p.centerLineMultiLane = j.value("centerLineMultiLane", p.centerLineMultiLane);
  if (j.contains("signs"))
    for (auto &[key, v] : j["signs"].items()) {
      SignCode s;
      s.country = v.value("country", p.code);
      s.type = v.value("type", "");
      s.subtype = v.value("subtype", "-1");
      s.width = v.value("width", 0.6);
      s.height = v.value("height", 0.6);
      p.signs[key] = s;
    }
  return p;
}

TargetProfile TargetProfile::byName(const std::string &name) {
  TargetProfile t;
  t.name = name;
  if (name == "carla") {
    t.revMinor = 4;
    t.openDriveSignCatalog = true;
    t.speedSignKmhBuckets = true;
    t.trafficLightZeroOffset = true;
    t.maxIdLength = 32;
  } else if (name == "asam18") {
    t.revMinor = 8;
  } else if (name == "esmini") {
    t.revMinor = 6;
  }
  return t;
}

}  // namespace odrforge
