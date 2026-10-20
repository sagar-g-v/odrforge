// OdrForge - a ground grid conformed to the roads.
//
// A terrain model coarser than a road (a 30 m DEM under a 7 m mountain road) runs its hillside straight through the
// road's bench. For a 3D scene the ground has to carry the road: under every lane it is the road surface, beside it
// it meets the terrain along cut and fill slopes. OpenDRIVE describes only the road (1.8.0 §10.5 elevation and
// superelevation); the conformed grid is the matching ground for consumers that drape terrain around it (the review
// viewer, CARLA world building). Bridges and tunnels leave the ground alone (it is not their road surface).
#pragma once

#include "odrforge/odr.hpp"
#include "odrforge/terrain.hpp"

namespace odrforge {

struct ConformOptions {
  double slope = 1.5;      ///< cut / fill slope, horizontal metres per metre of height (1:1.5, a common embankment)
  double maxReach = 20.0;  ///< m from the road edge beyond which the terrain is left as it is
  /// the road floor: the ground stays this far under every lane and junction surface ...
  double floorEpsilon = 0.05;
  /// ... and the cap on it rises this much per metre away from them (2: a 1:0.5 cut, 63 degrees)
  double floorSlope = 2.0;
};

/// `ground` (level 0) conformed to the roads of `od`: the road surface (odr::surfaceZ, without lane heights; the lowest
/// where roads share a cell) under every lane outside bridges and tunnels, and cut / fill slopes from the road edges to
/// the terrain; then the road floor - no cell a reader interpolates a lane or junction-surface point from stands above
/// that point's surface minus floorEpsilon (a road's own lanes inside its tunnels excepted), the cap rising at
/// floorSlope away from them, so the road wins over the terrain. Same frame and datum.
TerrainGrid conformGround(const odr::OpenDrive &od, const TerrainGrid &ground, const ConformOptions &opt = {});

struct SeatReport {
  int seated = 0;         ///< objects whose zOffset now puts their base on the ground
  int onBridges = 0;      ///< left on their deck (the ground under a bridge is not where they stand)
  double maxLift = 0.0;   ///< largest zOffset set upward (m): above a tunnel, beside a cutting
  double maxDrop = 0.0;   ///< largest zOffset set downward (m): beside an embankment
};

/// Objects standing on the ground beside or above a road - trees, poles / street lamps (OpenDRIVE 1.8.0 §13.14 point
/// objects without outline or repeat) - get the zOffset that puts their base on `conformed` (the ground conformed to
/// the roads, as published): XSD 1.8.1 t_road_objects_object @zOffset is the "z-offset of object's origin relative to
/// the elevation of the road reference line", so zOffset = ground(x, y) - elevation(s). An object on a road above a
/// tunnel stands on the surface over it; one on a bridge stretch keeps its height (the deck).
SeatReport seatObjects(odr::OpenDrive &od, const TerrainGrid &conformed);

}  // namespace odrforge
