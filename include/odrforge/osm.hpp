// OdrForge - OpenStreetMap data model and XML (.osm) reader.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace odrforge {

using OsmId = std::int64_t;
using Tags = std::map<std::string, std::string>;

struct OsmNode {
  OsmId id = 0;
  double lat = 0.0, lon = 0.0;
  Tags tags;
};

struct OsmWay {
  OsmId id = 0;
  std::vector<OsmId> nodes;
  Tags tags;
  bool closed() const { return nodes.size() > 2 && nodes.front() == nodes.back(); }
};

struct OsmMember {
  char type = 'n';  ///< 'n' node, 'w' way, 'r' relation
  OsmId ref = 0;
  std::string role;
};

struct OsmRelation {
  OsmId id = 0;
  std::vector<OsmMember> members;
  Tags tags;
};

/// Tag helpers (OSM conventions: "yes"/"no", ';'-separated lists, '|'-separated per-lane values).
std::optional<std::string> tag(const Tags &t, const std::string &key);
bool tagIs(const Tags &t, const std::string &key, const std::string &value);
bool tagYes(const Tags &t, const std::string &key);
std::vector<std::string> splitLanes(const std::string &v);  ///< "left|through;right|" -> 3 entries
std::vector<std::string> splitList(const std::string &v, char sep = ';');

class OsmData {
 public:
  std::unordered_map<OsmId, OsmNode> nodes;
  std::unordered_map<OsmId, OsmWay> ways;
  std::unordered_map<OsmId, OsmRelation> relations;
  double minLat = 90, minLon = 180, maxLat = -90, maxLon = -180;  ///< <bounds> or data extent

  /// Read an .osm XML file (OSM API / Overpass / osmium export).
  static OsmData readXml(const std::string &path);
  const OsmNode *node(OsmId id) const;
  const OsmWay *way(OsmId id) const;
};

}  // namespace odrforge
