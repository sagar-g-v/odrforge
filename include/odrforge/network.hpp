// OdrForge - OSM -> OpenDRIVE network builder.
//
// Pipeline: select road ways -> split at shared nodes -> chain pass-through segments into roads
// (lane-count changes become lane sections with tapers, not junctions) -> cluster intersection nodes
// by carriageway geometry -> fit clothoid reference lines -> trim roads where their carriageways stop
// overlapping -> connecting roads per lane movement (turn:lanes, restrictions, defaults) as clothoids
// along the lane centre -> elevation -> signals, controllers, signs, crosswalks, street furniture.
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "odrforge/clothoid_fit.hpp"
#include "odrforge/lanes.hpp"
#include "odrforge/odr.hpp"
#include "odrforge/osm.hpp"
#include "odrforge/projection.hpp"
#include "odrforge/rules.hpp"
#include "odrforge/terrain.hpp"

namespace odrforge {

/// Terrain height (m) at a local x, y; return NaN where unknown.
using ElevationFn = std::function<double(double x, double y)>;

struct BuildOptions {
  CountryProfile country;
  TargetProfile target;
  LaneOptions lanes;
  PolylineFitOptions fit;
  std::set<std::string> highways = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
                                    "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
                                    "residential", "living_street", "service", "road", "busway"};
  std::set<std::string> excludeService = {"parking_aisle", "drive-through", "emergency_access"};
  double junctionJoinSlack = 4.0;   ///< extra metres when merging intersection nodes into one junction
  double stopLineSetback = 1.0;     ///< road ends this far before the conflict area
  double cornerRadius = 6.0;        ///< curb return radius at intersections (m); scaled up for major roads
  double taperLength = 30.0;        ///< lane add / drop transition length (m)
  double minConnectingRadius = 3.0; ///< connecting roads tighter than this get a smoother 2-piece curve
  bool crosswalksAtSignals = true;  ///< signalised junctions get crosswalks on every approach
  bool speedSigns = true;
  bool streetFurniture = true;      ///< street lamps, trees near roads as objects
  ElevationFn elevation;            ///< optional terrain function; flat (z = 0) when empty and no terrain
  const Terrain *terrain = nullptr; ///< ground + structure (bridge deck) grids; preferred over `elevation`
  /// without terrain or `elevation`: the ground is flat and only left where roads of different OSM
  /// layers cross without a junction, one storey apart: this height (clearance plus deck), or the
  /// lower road's maxheight plus 1 m where tagged
  double levelHeight = 6.0;
  double elevationStep = 2.0;       ///< profile sampling step along the carriageway centre (m)
  double elevationSmoothing = 8.0;  ///< Whittaker smoothing (lambda at 1 m spacing); larger = smoother grades
  double elevationKnotSpacing = 10.0; ///< spacing of the cubic elevation records (m)
  double maxGrade = 0.18;           ///< profiles steeper than this over 10 m get more smoothing
  bool junctionSurfaces = true;     ///< roads blend into one plane per junction; connecting roads lie on it
  double junctionBlend = 20.0;      ///< length over which a road blends into its junction surface (m)
  std::string verticalDatum;        ///< written to the header userData (e.g. "NAVD88")
  /// Optional area of interest in local metres (minX, minY, maxX, maxY): ways are cut exactly at
  /// the boundary (new end nodes at the crossing points), everything outside is dropped.
  std::optional<std::array<double, 4>> clipBox;
  /// OSM near-misses: a way end that stops short of another road (within that road's half width
  /// plus this many metres, pointing at it, same layer) is joined to it; 0 disables
  double snapDanglingEnds = 3.0;
  /// parking aisles (excluded service kinds) become roads where their lot connects two or more
  /// road attachment points: driving through the lot is a real connection, not two dead ends
  bool connectParkingLots = true;
  /// a sidewalk mapped as its own way (footway=sidewalk) along a road: the road's cross-section
  /// reaches out to it (border verge + sidewalk at the measured distance). Off by default until verge
  /// widths blend across way segments and wide verges stay fold-safe in tight bends (they raise folds
  /// and lane-edge breaks on the test areas today).
  bool separateSidewalks = false;
  /// freeway splits / merges at a shallow angle: the trunk carries the branch lanes up to where the
  /// carriageways separate and the junction sits there (no overlapping road surfaces at gores)
  bool laneLevelGores = true;
  /// near-parallel roads drawn closer than their lanes are wide are moved apart (bounded laneOffset)
  bool resolveOverlaps = true;
  std::set<OsmId> includeWays;  ///< ways included regardless of the class filters (lot aisles are added here)
  std::string name = "OdrForge network";
};

struct BuildReport {
  int roads = 0, connectingRoads = 0, junctions = 0, signals = 0, controllers = 0, objects = 0;
  int geometryElements = 0, lines = 0, arcs = 0, spirals = 0;
  double fitMeanDev = 0.0, fitP95Dev = 0.0, fitMaxDev = 0.0;  ///< OSM node distance to the reference line (m)
  /// road surface vs measured ground at driving-lane centres (outside bridges / tunnels), metres
  double elevMeanDev = 0.0, elevP95Dev = 0.0, elevMaxDev = 0.0;
  int elevSamples = 0;
  int snappedEnds = 0;
  int parkingAisles = 0;
  int separateSidewalks = 0;  ///< road sides extended to a separately mapped sidewalk
  int joinedCrossings = 0;  ///< minor roads crossing on one level without a node: intersection added
  int gores = 0;          ///< splits / merges modelled with branch lanes up to the gore  ///< parking-lot aisles included because they connect streets  ///< OSM way ends joined to the road they stop short of
  int directJunctions = 0;  ///< gores written as direct junctions (OpenDRIVE 1.7+)
  int junctionGrids = 0;        ///< junctions with a reference line and elevation grid (OpenDRIVE 1.8+)
  int junctionBoundaries = 0;   ///< junctions with a boundary (OpenDRIVE 1.8+)
  int boundaryHelperRoads = 0;  ///< roads added only to close a junction boundary (1.8.0 §12.10)
  int boundariesLeftOut = 0;    ///< junctions whose outline would not enclose all their lanes: no boundary written
  int crossPaths = 0;
  int junctionCrossPaths = 0;  ///< of those, crossings inside common junctions (Junction Guideline §6.3.3)       ///< mid-block crossings written as cross paths (OpenDRIVE 1.8+)
  int curbLanes = 0;        ///< sidewalk lane sections given a curb lane (OpenDRIVE 1.6+)
  int overlapShifts = 0;
  int cornerSidewalks = 0;          ///< sidewalks continued round junction corners
  int cornerSidewalksTooTight = 0;  ///< corners whose arm cuts leave no room for the sidewalk to turn
  int cornerNoses = 0;              ///< acute corners (forks): the sidewalk ends in a nose
  int steepSpots = 0;        ///< samples steeper than 20 %
  int steepNotInTerrain = 0; ///< ... of which the measured terrain is > 3 % less steep (artefacts)  ///< roads shifted sideways to remove side-by-side carriageway overlaps
  std::vector<std::string> warnings;
};

odr::OpenDrive buildOpenDrive(const OsmData &osm, const Projection &proj, const BuildOptions &opt,
                              BuildReport *report = nullptr);

}  // namespace odrforge
