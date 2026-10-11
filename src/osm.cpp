#include "odrforge/osm.hpp"

#include <pugixml.hpp>
#include <stdexcept>

namespace odrforge {

std::optional<std::string> tag(const Tags &t, const std::string &key) {
  auto it = t.find(key);
  if (it == t.end()) return std::nullopt;
  return it->second;
}

bool tagIs(const Tags &t, const std::string &key, const std::string &value) {
  auto it = t.find(key);
  return it != t.end() && it->second == value;
}

bool tagYes(const Tags &t, const std::string &key) {
  auto it = t.find(key);
  return it != t.end() && (it->second == "yes" || it->second == "true" || it->second == "1");
}

std::vector<std::string> splitLanes(const std::string &v) { return splitList(v, '|'); }

std::vector<std::string> splitList(const std::string &v, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : v) {
    if (c == sep) {
      out.push_back(cur);
      cur.clear();
    } else if (!(cur.empty() && c == ' ')) {
      cur += c;
    }
  }
  out.push_back(cur);
  for (auto &s : out)
    while (!s.empty() && s.back() == ' ') s.pop_back();
  return out;
}

static Tags readTags(const pugi::xml_node &n) {
  Tags t;
  for (auto c : n.children("tag")) t[c.attribute("k").value()] = c.attribute("v").value();
  return t;
}

OsmData OsmData::readXml(const std::string &path) {
  pugi::xml_document doc;
  const auto res = doc.load_file(path.c_str(), pugi::parse_default);
  if (!res) throw std::runtime_error("OSM: cannot parse " + path + ": " + res.description());
  const auto root = doc.child("osm");
  if (!root) throw std::runtime_error("OSM: no <osm> root in " + path);
  OsmData d;
  bool haveBounds = false;
  if (auto b = root.child("bounds")) {
    d.minLat = b.attribute("minlat").as_double();
    d.minLon = b.attribute("minlon").as_double();
    d.maxLat = b.attribute("maxlat").as_double();
    d.maxLon = b.attribute("maxlon").as_double();
    haveBounds = true;
  }
  for (auto n : root.children("node")) {
    if (std::string(n.attribute("action").value()) == "delete") continue;
    OsmNode o;
    o.id = n.attribute("id").as_llong();
    o.lat = n.attribute("lat").as_double();
    o.lon = n.attribute("lon").as_double();
    o.tags = readTags(n);
    if (!haveBounds) {
      d.minLat = std::min(d.minLat, o.lat), d.maxLat = std::max(d.maxLat, o.lat);
      d.minLon = std::min(d.minLon, o.lon), d.maxLon = std::max(d.maxLon, o.lon);
    }
    d.nodes.emplace(o.id, std::move(o));
  }
  for (auto w : root.children("way")) {
    if (std::string(w.attribute("action").value()) == "delete") continue;
    OsmWay o;
    o.id = w.attribute("id").as_llong();
    for (auto nd : w.children("nd")) o.nodes.push_back(nd.attribute("ref").as_llong());
    o.tags = readTags(w);
    d.ways.emplace(o.id, std::move(o));
  }
  for (auto r : root.children("relation")) {
    if (std::string(r.attribute("action").value()) == "delete") continue;
    OsmRelation o;
    o.id = r.attribute("id").as_llong();
    for (auto m : r.children("member")) {
      OsmMember mm;
      const std::string ty = m.attribute("type").value();
      mm.type = ty.empty() ? 'n' : ty[0];
      mm.ref = m.attribute("ref").as_llong();
      mm.role = m.attribute("role").value();
      o.members.push_back(mm);
    }
    o.tags = readTags(r);
    d.relations.emplace(o.id, std::move(o));
  }
  return d;
}

const OsmNode *OsmData::node(OsmId id) const {
  auto it = nodes.find(id);
  return it == nodes.end() ? nullptr : &it->second;
}

const OsmWay *OsmData::way(OsmId id) const {
  auto it = ways.find(id);
  return it == ways.end() ? nullptr : &it->second;
}

}  // namespace odrforge
