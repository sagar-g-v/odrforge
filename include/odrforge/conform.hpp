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
};

/// `ground` (level 0) conformed to the roads of `od`: the road surface (odr::surfaceZ, without lane heights; the lowest
/// where roads share a cell) under every
/// lane outside bridges and tunnels, and cut / fill slopes from the road edges to the terrain. Same frame and datum.
TerrainGrid conformGround(const odr::OpenDrive &od, const TerrainGrid &ground, const ConformOptions &opt = {});

}  // namespace odrforge
