#include "odrforge/terrain.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace odrforge {

double TerrainGrid::sample(double x, double y) const {
  if (z.empty()) return std::numeric_limits<double>::quiet_NaN();
  const double fx = (x - originX) / cell, fy = (y - originY) / cell;
  const int i0 = static_cast<int>(std::floor(fx)), j0 = static_cast<int>(std::floor(fy));
  const double tx = fx - i0, ty = fy - j0;
  double sum = 0, wsum = 0;
  for (int dj = 0; dj <= 1; ++dj)
    for (int di = 0; di <= 1; ++di) {
      const int i = i0 + di, j = j0 + dj;
      if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
      const float v = at(0, i, j);
      if (std::isnan(v)) continue;
      const double w = (di ? tx : 1 - tx) * (dj ? ty : 1 - ty);
      sum += w * v;
      wsum += w;
    }
  return wsum > 1e-9 ? sum / wsum : std::numeric_limits<double>::quiet_NaN();
}

std::vector<double> TerrainGrid::levelsAt(double x, double y) const {
  std::vector<double> out;
  if (z.empty()) return out;
  const int ic = static_cast<int>(std::lround((x - originX) / cell)), jc = static_cast<int>(std::lround((y - originY) / cell));
  for (int ring = 0; ring <= 2 && out.empty(); ++ring)
    for (int dj = -ring; dj <= ring; ++dj)
      for (int di = -ring; di <= ring; ++di) {
        if (std::max(std::abs(di), std::abs(dj)) != ring) continue;
        const int i = ic + di, j = jc + dj;
        if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
        for (int l = 0; l < levels; ++l) {
          const float v = at(l, i, j);
          if (std::isnan(v)) continue;
          // merge with an existing candidate of (nearly) the same level
          bool merged = false;
          for (double &o : out)
            if (std::abs(o - v) < 1.5) merged = true;
          if (!merged) out.push_back(v);
        }
      }
  std::sort(out.begin(), out.end());
  return out;
}

TerrainGrid TerrainGrid::load(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("terrain: cannot open " + path);
  char magic[8];
  f.read(magic, 8);
  const bool v1 = std::memcmp(magic, "ODRZGRD1", 8) == 0, v2 = std::memcmp(magic, "ODRZGRD2", 8) == 0;
  if (!v1 && !v2) throw std::runtime_error("terrain: not an .odrz grid: " + path);
  TerrainGrid g;
  int32_t nx, ny, nl = 1;
  char datum[32];
  f.read(reinterpret_cast<char *>(&g.originX), 8);
  f.read(reinterpret_cast<char *>(&g.originY), 8);
  f.read(reinterpret_cast<char *>(&g.cell), 8);
  f.read(reinterpret_cast<char *>(&nx), 4);
  f.read(reinterpret_cast<char *>(&ny), 4);
  if (v2) f.read(reinterpret_cast<char *>(&nl), 4);
  f.read(datum, 32);
  if (!f || nx <= 0 || ny <= 0 || nl <= 0 || nl > 16 || g.cell <= 0) throw std::runtime_error("terrain: bad header in " + path);
  g.nx = nx, g.ny = ny, g.levels = nl;
  g.datum.assign(datum, strnlen(datum, 32));
  g.z.resize(static_cast<size_t>(nl) * nx * ny);
  f.read(reinterpret_cast<char *>(g.z.data()), static_cast<std::streamsize>(g.z.size() * sizeof(float)));
  if (!f) throw std::runtime_error("terrain: truncated grid " + path);
  return g;
}

void TerrainGrid::save(const std::string &path) const {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("terrain: cannot write " + path);
  f.write(levels > 1 ? "ODRZGRD2" : "ODRZGRD1", 8);
  f.write(reinterpret_cast<const char *>(&originX), 8);
  f.write(reinterpret_cast<const char *>(&originY), 8);
  f.write(reinterpret_cast<const char *>(&cell), 8);
  const int32_t a = nx, b = ny, l = levels;
  f.write(reinterpret_cast<const char *>(&a), 4);
  f.write(reinterpret_cast<const char *>(&b), 4);
  if (levels > 1) f.write(reinterpret_cast<const char *>(&l), 4);
  char datum32[32] = {0};
  std::memcpy(datum32, datum.data(), std::min<size_t>(datum.size(), 31));
  f.write(datum32, 32);
  f.write(reinterpret_cast<const char *>(z.data()), static_cast<std::streamsize>(z.size() * sizeof(float)));
}

}  // namespace odrforge
