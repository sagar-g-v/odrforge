// OdrForge - internal state shared by the network builder stages (not installed).
#pragma once

#include <functional>
#include <map>
#include <optional>
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
    double maxHeight = 0.0;  ///< signed headroom (OSM maxheight), 0: not tagged
  };
  std::vector<LayerRange> layers;  ///< OSM layer (bridge without layer = 1, tunnel = -1, building passage = 0) per s range
  std::vector<std::pair<OsmId, double>> nodeS;  ///< OSM nodes of the chain with their s on the full line
  OsmId wayStart = 0, wayEnd = 0;
  double trimStart = 0.0, trimEnd = 0.0;
  int roadIndex = -1;  ///< index into OpenDrive::roads
  std::vector<double> zs, zz;  ///< elevation samples (s on the trimmed road) and smoothed heights
  std::vector<double> xs;      ///< smoothed cross slope (tan of the superelevation) per sample
  bool hasData = true;         ///< false: no measurement anywhere on the road
  std::optional<double> fixedTrimStart, fixedTrimEnd;  ///< set by the gore stage: not re-trimmed
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
  /// lane-level gore (diverge: trunk -> main + branch; merge: main + branch -> trunk). The trunk
  /// carries `extra` branch lanes on the branch side up to the separation point (the junction).
  struct Gore {
    int trunk = -1, main = -1, branch = -1;  ///< meta indices
    bool diverge = true;
    bool branchLeft = false;  ///< branch on the left of the main road (travel direction)
    int extra = 0;
  };
  std::optional<Gore> gore;
  double plane[3] = {0, 0, 0};  ///< z = a + b (x - cx) + c (y - cy)
};

/// minimum fillet radius of a road class (m)
double classMinRadius(const std::string &hw);
/// carriageway extents (left, right) of a cross section relative to the reference line (no sidewalks)
void carriagewayExtents(const CrossSection &x, double &left, double &right);
/// the way becomes a road (highway class, service kind, access, complete geometry)
bool isRoadWay(const OsmData &osm, const OsmWay &w, const BuildOptions &opt);

/// parking aisles of lots that connect two or more road attachment points (BuildOptions::connectParkingLots)
std::set<OsmId> throughParkingAisles(const OsmData &osm, const BuildOptions &opt);
/// joins way ends that stop short of another road (see BuildOptions::snapDanglingEnds)
/// ... and inserts the missing node where minor roads cross on the same level (crossings: count)
OsmData snapDanglingEnds(const OsmData &in, const Projection &proj, const BuildOptions &opt, int &snapped,
                         int *crossings = nullptr);

/// The two road ends a connecting road joins: the incoming arm (traffic arriving at its end or start)
/// and the outgoing arm (traffic leaving from its start or end).
struct ConnectionEnds {
  const odr::Road *in;
  bool inAtEnd;
  const odr::Road *out;
  bool outAtEnd;
  double sIn() const { return inAtEnd ? in->length() : 0.0; }
  double sOut() const { return outAtEnd ? out->length() : 0.0; }
};

/// Separately mapped sidewalks (footway=sidewalk) and kerb lines (barrier=kerb), indexed for the
/// sideways look-up that measures where a road's sidewalk lies: sidewalk.cpp
class SidewalkIndex {
 public:
  struct Leg {
    Vec2 a, b;
    double width;
    int level;  ///< OSM layer, bridges / tunnels one level apart from the ground when untagged
    bool kerb;  ///< a kerb line: the sidewalk's curb-side edge
  };
  /// a point on the road and its unit direction of travel
  struct Sample {
    Vec2 p, dir;
    int level;
  };
  struct Measured {
    double centre, width;  ///< sidewalk centre distance from the line sampled, and its width
  };
  void add(const Leg &g);
  bool empty() const { return legs_.empty(); }
  /// median sidewalk on one side of the samples; none when fewer than half the samples (or < 2) see one
  std::optional<Measured> measure(const std::vector<Sample> &samples, bool left) const;

 private:
  std::vector<Leg> legs_;
  std::map<std::pair<long, long>, std::vector<size_t>> grid_;
};

/// A verge + sidewalk side (attached separate sidewalk) on the inside of a bend tighter than its
/// reach narrows so it cannot fold: sidewalk.cpp
void keepVergeFoldSafe(CrossSection &x, const ReferenceLine &line, double s0, double s1);

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
  /// junction node pairs built as one junction because the road between them was squeezed out
  std::vector<std::pair<OsmId, OsmId>> forcedJoins;
  SidewalkIndex sidewalks;  ///< filled by attachSeparateSidewalks

  Builder(const OsmData &o, const Projection &p, const BuildOptions &b) : osm(o), proj(p), opt(b) {}

  Vec2 pos(OsmId n) const {
    const OsmNode *nd = osm.node(n);
    return nd ? proj.forward(nd->lat, nd->lon) : Vec2();
  }

  // stages (network.cpp)
  void collectSegments();
  void attachSeparateSidewalks();  // sidewalk.cpp
  /// re-measures attached sidewalks from the fitted, trimmed reference line (sidewalk.cpp)
  void remeasureSidewalks(const RoadMeta &m, const ReferenceLine &line, std::vector<SectionInfo> &secs) const;
  void findJunctionNodes();
  void buildChains();
  void clusterJunctions();
  void buildRoadGeometry();
  void buildGores();         // gore.cpp
  void directJunctions();    // direct_junctions.cpp
  void curbLanes();          // curbs.cpp
  void trimRoads();
  bool joinSqueezedJunctions();
  void makeRoomForCurbReturns(const Cluster &c, const std::function<double(const RoadMeta &)> &trimCap);
  void forgetJunctionLayout(const std::vector<Chain> &chainsBefore);
  void buildRoads();
  void resolveOverlaps();    // overlap.cpp
  void resolveLevelOverlaps();  // overlap.cpp: second pass once heights are known
  bool overlapLevelPass = false;
  void sampleElevation();    // elevation.cpp
  void junctionSurfaces();
  void fitRoadElevation();
  void connectingElevation(odr::Road &c, const Cluster &cl,
                           std::optional<std::pair<double, double>> flatEnds = std::nullopt) const;
  double planeZ(const Cluster &c, const Vec2 &p) const;
  // stages (network_junctions.cpp)
  void buildJunctions();
  OsmId attachNode(const RoadEnd &e) const;
  std::optional<bool> restrictionLeadsTo(const Cluster &cl, OsmId via, OsmId toWay, const RoadEnd &exit,
                                         bool onwards) const;
  /// connecting roads for lane pairs (incoming lane id, outgoing lane id) of one relation
  void connectLanes(const ConnectionEnds &ends, const std::vector<std::pair<int, int>> &pairs, const Cluster &cl,
                    odr::Junction &junction, std::vector<odr::Road> &out, int &nextRoadId);
  enum class Frame { RefuseDistortion, RefuseFolding, AlwaysBuild };
  /// a reference line end beside an arm: its offset t from the arm reference line (travel frame)
  /// and how fast that offset changes per metre travelled
  struct LaneCentre {
    double t = 0.0, slope = 0.0;
  };
  bool addConnectingRoad(const ConnectionEnds &ends, const ReferenceLine &line, LaneCentre refIn, LaneCentre refOut,
                         const std::vector<std::pair<int, int>> &lanes, Frame frame, const Cluster &cl,
                         odr::Junction &junction,
                         std::vector<odr::Road> &out, int &nextRoadId);
  void buildCornerSidewalks(const Cluster &cl, odr::Junction &junction, std::vector<odr::Road> &out, int &nextRoadId);
  void connectingRoadHeight(odr::Road &c, const ConnectionEnds &ends, double tRefIn, double tRefOut,
                            const Cluster &cl) const;
  void buildSignals();
  void buildObjects();
  void finish();
  void groupRoundabouts();

  // helpers
  static int classRank(const std::string &hw);
  const CrossSection &sectionAtFull(const RoadMeta &m, double sFull) const;
  /// lanes of road r travelling toward (incoming) or away from (outgoing) a road end
  std::vector<int> endLanes(const odr::Road &r, bool atEnd, bool incoming) const;
};

}  // namespace odrforge::detail
