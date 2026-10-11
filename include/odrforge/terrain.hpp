// OdrForge - terrain / structure surfaces for road elevation.
//
// A TerrainGrid is a regular grid in the network's local projected frame (metres, x east, y north),
// float32 heights, NaN = unknown, with one or more levels per cell (stacked bridge decks).
// File format ".odrz" (little endian):
//   v1: char magic[8] = "ODRZGRD1"; double originX, originY (centre of cell [0,0]); double cell;
//       int32 nx, ny; char datum[32]; float32 z[ny][nx]              (row 0 = southmost row)
//   v2: magic "ODRZGRD2", same header + int32 nlevels after ny; float32 z[nlevels][ny][nx],
//       levels sorted from the lowest to the highest surface in each cell.
// Produced by tools/make_terrain.py from DEM GeoTIFFs and lidar.
#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace odrforge {

class TerrainGrid {
 public:
  double originX = 0, originY = 0, cell = 1;
  int nx = 0, ny = 0, levels = 1;
  std::string datum;
  std::vector<float> z;  ///< levels * ny * nx

  bool empty() const { return z.empty(); }
  /// Bilinear height of level 0 at (x, y); NaN cells ignored (weights renormalised); NaN if none.
  double sample(double x, double y) const;
  /// All surface levels at (x, y) (nearest cell, neighbours if it is empty), lowest first.
  std::vector<double> levelsAt(double x, double y) const;
  static TerrainGrid load(const std::string &path);
  void save(const std::string &path) const;
  float at(int level, int i, int j) const { return z[(static_cast<size_t>(level) * ny + j) * nx + i]; }
};

/// Ground + structure (bridge deck) surfaces.
struct Terrain {
  TerrainGrid ground, structures;
  double ground_at(double x, double y) const {
    return ground.empty() ? std::numeric_limits<double>::quiet_NaN() : ground.sample(x, y);
  }
  double structure_at(double x, double y) const {
    return structures.empty() ? std::numeric_limits<double>::quiet_NaN() : structures.sample(x, y);
  }
  std::vector<double> structure_levels(double x, double y) const {
    return structures.empty() ? std::vector<double>{} : structures.levelsAt(x, y);
  }
};

}  // namespace odrforge
