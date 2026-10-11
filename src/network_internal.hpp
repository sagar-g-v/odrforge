// OdrForge - internal state shared by the network builder stages (not installed).
#pragma once

#include <map>
#include <unordered_map>
#include <unordered_set>

#include "odrforge/network.hpp"

namespace odrforge::detail {

struct Seg {
  OsmId way = 0;
  std::vector<OsmId> nodes;
  const Tags *tags = nullptr;
  CrossSection xs;
  int travel = 0;  ///< +1 oneway along the way, -1 oneway against, 0 two-way
  bool bridge = false, tunnel = false;
  bool covered = false;  ///< covered=yes: under a building or hillside, the surface above is not the road
};

struct Part {
  int seg = 0;
  bool fwd = true;  ///< traversed along the OSM way direction
};

struct Chain {
  std::vector<Part> parts;
  OsmId n0 = 0, n1 = 0;
  int c0 = -1, c1 = -1;  ///< junction cluster at start / end (-1 = open end)
  bool internal = false; ///< absorbed into a junction
  double approxLength = 0.0;
};

struct SectionInfo {
  double s = 0.0;
  CrossSection xs;
};

struct RoadEnd {
  int road = -1;
  bool atEnd = true;  ///< contact point "end" (else "start")
};

struct RoadMeta {
  int chain = -1;
  ReferenceLine full;  ///< untrimmed reference line
  std::vector<SectionInfo> sections;  ///< s on the full line
  std::vector<std::pair<double, double>> bridges, tunnels;  ///< s ranges on the full line
  std::vector<std::pair<double, double>> covered;           ///< covered=yes s ranges (no height observations)
  struct LayerRange {
    double s0, s1;
    int layer;
  };
  std::vector<LayerRange> layers;  ///< OSM layer (bridge without layer = 1, tunnel = -1) per s range
  std::vector<std::pair<OsmId, double>> nodeS;  ///< OSM nodes of the chain with their s on the full line
  OsmId wayStart = 0, wayEnd = 0;
  double trimStart = 0.0, trimEnd = 0.0;
  int roadIndex = -1;  ///< index into OpenDrive::roads
  std::vector<double> zs, zz;  ///< elevation samples (s on the trimmed road) and smoothed heights
  std::vector<double> xs;      ///< smoothed cross slope (tan of the superelevation) per sample
  bool hasData = true;         ///< false: no measurement anywhere on the road
  std::vector<double> zy, zw, ztc;  ///< observed centre heights (NaN: none), robust weights, carriageway centre t
  double zLambda = 0.0;             ///< Whittaker smoothing weight used for the profile
  double obs0 = 1e9, obs1 = 1e9;    ///< distance from the start / end to the nearest observed sample
  std::string highway;
  int rank = 0;        ///< road class priority (higher = more important)
};

struct Cluster {
  std::vector<OsmId> nodes;
  std::vector<RoadEnd> ends;
  Vec2 center;
  bool signalised = false;
  bool crossing = false;
  int junction = -1;  ///< index into OpenDrive::junctions
  bool hasPlane = false;
  double plane[3] = {0, 0, 0};  ///< z = a + b (x - cx) + c (y - cy)
};

struct Builder {
  const OsmData &osm;
  const Projection &proj;
  const BuildOptions &opt;
  BuildReport rep;

  std::vector<Seg> segs;
  std::unordered_map<OsmId, std::vector<std::pair<int, bool>>> nodeSegs;  ///< node -> (seg, atStart)
  std::unordered_set<OsmId> junctionNodes;
  std::vector<Chain> chains;
  std::unordered_map<OsmId, int> clusterOf;
  std::vector<Cluster> clusters;
  std::vector<RoadMeta> metas;
  odr::OpenDrive od;
  int nextSignalId = 1, nextObjectId = 1, nextControllerId = 1;
  std::vector<double> fitDev;  ///< per OSM node: distance to its fitted reference line

  Builder(const OsmData &o, const Projection &p, const BuildOptions &b) : osm(o), proj(p), opt(b) {}

  Vec2 pos(OsmId n) const {
    const OsmNode *nd = osm.node(n);
    return nd ? proj.forward(nd->lat, nd->lon) : Vec2();
  }

  // stages (network.cpp)
  void collectSegments();
  void findJunctionNodes();
  void buildChains();
  void clusterJunctions();
  void buildRoadGeometry();
  void trimRoads();
  void buildRoads();
  void resolveOverlaps();    // overlap.cpp
  void sampleElevation();    // elevation.cpp
  void junctionSurfaces();
  void fitRoadElevation();
  void connectingElevation(odr::Road &c, const Cluster &cl,
                           std::optional<std::pair<double, double>> flatEnds = std::nullopt) const;
  double planeZ(const Cluster &c, const Vec2 &p) const;
  // stages (network_junctions.cpp)
  void buildJunctions();
  void buildSignals();
  void buildObjects();
  void finish();

  // helpers
  static int classRank(const std::string &hw);
  const CrossSection &sectionAtFull(const RoadMeta &m, double sFull) const;
  /// lanes of road r travelling toward (incoming) or away from (outgoing) a road end
  std::vector<int> endLanes(const odr::Road &r, bool atEnd, bool incoming) const;
};

}  // namespace odrforge::detail
