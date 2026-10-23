#include "odrforge/terrain.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace odrforge {

double TerrainGrid::sample(double x, double y) const {
  if (empty()) return std::numeric_limits<double>::quiet_NaN();
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
  if (empty()) return out;
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

namespace {
/// an .odrz file's header into g (frame, size, levels, datum); f is left at the first height
void readHeader(std::ifstream &f, const std::string &path, TerrainGrid &g) {
  if (!f) throw std::runtime_error("terrain: cannot open " + path);
  char magic[8];
  f.read(magic, 8);
  const bool v1 = std::memcmp(magic, "ODRZGRD1", 8) == 0, v2 = std::memcmp(magic, "ODRZGRD2", 8) == 0;
  if (!v1 && !v2) throw std::runtime_error("terrain: not an .odrz grid: " + path);
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
}

constexpr uint32_t kEmptyCellBits = 0x7fc00000u;  // the quiet NaN, as numpy writes np.nan in float32
}  // namespace

float TerrainGrid::emptyCell() {
  float v;
  std::memcpy(&v, &kEmptyCellBits, sizeof v);
  return v;
}

TerrainGrid TerrainGrid::load(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  TerrainGrid g;
  readHeader(f, path, g);
  g.z.resize(static_cast<size_t>(g.levels) * g.nx * g.ny);
  f.read(reinterpret_cast<char *>(g.z.data()), static_cast<std::streamsize>(g.z.size() * sizeof(float)));
  if (!f) throw std::runtime_error("terrain: truncated grid " + path);
  return g;
}

TerrainGrid TerrainGrid::loadSparse(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  TerrainGrid g;
  readHeader(f, path, g);
  g.tilesX_ = (g.nx + kTile - 1) >> kTileShift, g.tilesY_ = (g.ny + kTile - 1) >> kTileShift;
  g.tileAt_.assign(static_cast<size_t>(g.levels) * g.tilesY_ * g.tilesX_, -1);
  // a band of kTile rows at a time, cut into its tiles; a tile is kept when one of its cells is not the empty NaN
  std::vector<float> band(static_cast<size_t>(kTile) * g.nx);
  std::vector<float> tile(static_cast<size_t>(kTile) * kTile);
  for (int l = 0; l < g.levels; ++l)
    for (int ty = 0; ty < g.tilesY_; ++ty) {
      const int rows = std::min(kTile, g.ny - ty * kTile);
      f.read(reinterpret_cast<char *>(band.data()), static_cast<std::streamsize>(static_cast<size_t>(rows) * g.nx * sizeof(float)));
      if (!f) throw std::runtime_error("terrain: truncated grid " + path);
      for (int tx = 0; tx < g.tilesX_; ++tx) {
        const int i0 = tx * kTile, cols = std::min(kTile, g.nx - i0);
        bool keep = false;
        std::fill(tile.begin(), tile.end(), emptyCell());
        for (int r = 0; r < rows; ++r)
          for (int c = 0; c < cols; ++c) {
            const float v = band[static_cast<size_t>(r) * g.nx + i0 + c];
            uint32_t bits;
            std::memcpy(&bits, &v, sizeof bits);
            keep = keep || bits != kEmptyCellBits;
            tile[static_cast<size_t>(r) * kTile + c] = v;
          }
        if (!keep) continue;
        g.tileAt_[(static_cast<size_t>(l) * g.tilesY_ + ty) * g.tilesX_ + tx] = static_cast<int32_t>(g.tileZ_.size() / tile.size());
        g.tileZ_.insert(g.tileZ_.end(), tile.begin(), tile.end());
      }
    }
  g.tileZ_.shrink_to_fit();
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
  if (tileAt_.empty()) {
    f.write(reinterpret_cast<const char *>(z.data()), static_cast<std::streamsize>(z.size() * sizeof(float)));
    return;
  }
  std::vector<float> row(static_cast<size_t>(nx));  // tiles: row by row
  for (int lv = 0; lv < levels; ++lv)
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) row[static_cast<size_t>(i)] = at(lv, i, j);
      f.write(reinterpret_cast<const char *>(row.data()), static_cast<std::streamsize>(row.size() * sizeof(float)));
    }
}

}  // namespace odrforge
