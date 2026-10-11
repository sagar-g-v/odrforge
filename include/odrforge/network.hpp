// OdrForge - OSM -> OpenDRIVE network builder.
//
// Pipeline: select road ways -> split at shared nodes -> chain pass-through segments into roads
// (lane-count changes become lane sections with tapers, not junctions) -> cluster intersection nodes
// by carriageway geometry -> fit clothoid reference lines -> trim roads where their carriageways stop
// overlapping -> connecting roads per lane movement (turn:lanes, restrictions, defaults) as clothoids
// along the lane centre -> elevation -> signals, controllers, signs, crosswalks, street furniture.
#pragma once

#include <functional>
#include <set>
#include <string>
#include <vector>

#include "odrforge/clothoid_fit.hpp"
#include "odrforge/lanes.hpp"
#include "odrforge/odr.hpp"
#include "odrforge/osm.hpp"
#include "odrforge/projection.hpp"
#include "odrforge/rules.hpp"

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
  ElevationFn elevation;            ///< optional terrain; flat (z = 0) when empty
  std::string name = "OdrForge network";
};

struct BuildReport {
  int roads = 0, connectingRoads = 0, junctions = 0, signals = 0, controllers = 0, objects = 0;
  int geometryElements = 0, lines = 0, arcs = 0, spirals = 0;
  double fitMeanDev = 0.0, fitP95Dev = 0.0, fitMaxDev = 0.0;  ///< OSM node distance to the reference line (m)
  std::vector<std::string> warnings;
};

odr::OpenDrive buildOpenDrive(const OsmData &osm, const Projection &proj, const BuildOptions &opt,
                              BuildReport *report = nullptr);

}  // namespace odrforge
