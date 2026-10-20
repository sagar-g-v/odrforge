// OdrForge - a ground grid conformed to the roads (see conform.hpp).
#include "odrforge/conform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>

namespace odrforge {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
// a triangle's plane steeper than this is not trusted (roads stay under 25 %, junction planes under 15 %, banking under
// 15 %): a sliver's corners are nearly collinear, or a junction outline's triangle spans roads at different heights
constexpr double kMaxPlaneGrade = 0.5;

struct P3 {
  double x, y, z;
};

/// the lowest point of a triangle within the square of half-size h round (mx, my), kInf when they do not meet: the
/// triangle clipped to the square's four sides (Sutherland-Hodgman), heights interpolated along the edges - never from
/// the plane's gradient, which a sliver does not define
double lowestInSquare(const P3 (&tri)[3], double mx, double my, double h) {
  std::array<P3, 8> poly{tri[0], tri[1], tri[2]}, next{};  // a triangle cut by four half-planes keeps at most 7 corners
  int n = 3;
  for (int side = 0; side < 4 && n > 0; ++side) {
    const bool alongX = side < 2;
    const double line = side == 0 ? mx - h : side == 1 ? mx + h : side == 2 ? my - h : my + h;
    auto inside = [&](const P3 &p) {
      const double v = alongX ? p.x : p.y;
      return side % 2 == 0 ? v >= line : v <= line;
    };
    int m = 0;
    for (int k = 0; k < n; ++k) {
      const P3 &p = poly[k], &q = poly[(k + 1) % n];
      const bool ip = inside(p), iq = inside(q);
      if (ip) next[m++] = p;
      if (ip != iq) {
        const double u = alongX ? (line - p.x) / (q.x - p.x) : (line - p.y) / (q.y - p.y);
        next[m++] = {p.x + u * (q.x - p.x), p.y + u * (q.y - p.y), p.z + u * (q.z - p.z)};
      }
    }
    poly = next, n = m;
  }
  double z = kInf;
  for (int k = 0; k < n; ++k) z = std::min(z, poly[k].z);
  return z;
}

/// The road floor's conservative raster (the viewer's src/map/roadFloor.ts rasterTriangle, decided with the user on
/// 2026-10-18): every cell a reader of the grid interpolates a point of the triangle from - the four cells round it,
/// bilinear: every cell whose centre lies within one cell of the point along each axis, i.e. whose square of half-size
/// one cell overlaps the triangle - is capped at the triangle's plane at the cell centre minus eps. The viewer lowers
/// that by (|gx| + |gy|) half a cell to stay under any point of its drawn mesh; for a grid read bilinearly the plane
/// itself is exact (bilinear interpolation reproduces a plane), and the slack would sink the ground under every sloped
/// road by its gradient times the cell. `box` grows to every cell capped; `highest` (when given) keeps the highest cap
/// this raster set at each cell.
struct Box {
  int i0 = std::numeric_limits<int>::max(), j0 = std::numeric_limits<int>::max(), i1 = -1, j1 = -1;
};
void rasterTriangle(std::vector<double> &cap, const TerrainGrid &g, const std::array<P3, 3> &t, double eps, Box &box,
                    std::vector<double> *highest = nullptr) {
  const auto &[a, b, c] = t;
  const double area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
  if (std::abs(area) < 1e-9) return;
  const double gx = ((b.z - a.z) * (c.y - a.y) - (c.z - a.z) * (b.y - a.y)) / area;
  const double gy = ((c.z - a.z) * (b.x - a.x) - (b.z - a.z) * (c.x - a.x)) / area;
  const double h = g.cell;
  const bool steep = std::hypot(gx, gy) > kMaxPlaneGrade;
  const int j0 = std::max(0, static_cast<int>(std::floor((std::min({a.y, b.y, c.y}) - h - g.originY) / g.cell)));
  const int j1 = std::min(g.ny - 1, static_cast<int>(std::ceil((std::max({a.y, b.y, c.y}) + h - g.originY) / g.cell)));
  const P3 P[3] = {a, b, c};
  for (int j = j0; j <= j1; ++j) {
    // a cell's square [mx +- h] x [my +- h] meets the (convex) triangle exactly where the triangle's part inside the
    // row's band my +- h reaches the column mx +- h: that part's x-extent comes from the corners inside the band and the
    // edges' crossings of the band's two lines
    const double my = g.originY + j * g.cell, ylo = my - h, yhi = my + h;
    double xlo = kInf, xhi = -kInf;
    for (int e = 0; e < 3; ++e) {
      const P3 &p = P[e], &q = P[(e + 1) % 3];
      if (p.y >= ylo && p.y <= yhi) xlo = std::min(xlo, p.x), xhi = std::max(xhi, p.x);
      for (const double yl : {ylo, yhi})
        if ((p.y - yl) * (q.y - yl) < 0.0) {
          const double x = p.x + (q.x - p.x) * (yl - p.y) / (q.y - p.y);
          xlo = std::min(xlo, x), xhi = std::max(xhi, x);
        }
    }
    if (xlo > xhi) continue;
    const int i0 = std::max(0, static_cast<int>(std::ceil((xlo - h - g.originX) / g.cell)));
    const int i1 = std::min(g.nx - 1, static_cast<int>(std::floor((xhi + h - g.originX) / g.cell)));
    if (i0 > i1) continue;
    box.i0 = std::min(box.i0, i0), box.i1 = std::max(box.i1, i1), box.j0 = std::min(box.j0, j), box.j1 = std::max(box.j1, j);
    for (int i = i0; i <= i1; ++i) {
      const double mx = g.originX + i * g.cell;
      // the triangle's plane at the cell centre: a grid read bilinearly reproduces a plane exactly, so every point of the
      // triangle reads its own surface minus eps from cells set so. A steep plane is not trusted: the triangle's lowest
      // point within the cell's square then - every point a reader interpolates from this cell lies in that square, so
      // it reads at most its own surface minus eps
      const double z = (steep ? lowestInSquare(P, mx, my, h) : a.z + gx * (mx - a.x) + gy * (my - a.y)) - eps;
      double &k = cap[static_cast<size_t>(j) * g.nx + i];
      k = std::min(k, z);
      if (highest) (*highest)[static_cast<size_t>(j) * g.nx + i] = std::max((*highest)[static_cast<size_t>(j) * g.nx + i], z);
    }
  }
}

/// a counter-clockwise simple polygon cut into triangles (ear clipping); fewer when it is not simple
std::vector<std::array<P3, 3>> triangulate(std::vector<P3> poly) {
  std::vector<std::array<P3, 3>> out;
  auto cross = [](const P3 &o, const P3 &a, const P3 &b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); };
  while (poly.size() > 3) {
    bool cut = false;
    const size_t n = poly.size();
    for (size_t i = 0; i < n && !cut; ++i) {
      const P3 &a = poly[(i + n - 1) % n], &b = poly[i], &c = poly[(i + 1) % n];
      if (cross(a, b, c) <= 1e-12) continue;  // reflex or flat: not an ear
      bool empty = true;
      for (size_t k = 0; k < n && empty; ++k) {
        if (k == i || k == (i + n - 1) % n || k == (i + 1) % n) continue;
        const P3 &p = poly[k];
        empty = !(cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0);
      }
      if (!empty) continue;
      out.push_back({a, b, c});
      poly.erase(poly.begin() + static_cast<long>(i));
      cut = true;
    }
    if (!cut) return out;  // not simple: the rest stays uncovered (its connecting roads still press the ground)
  }
  if (poly.size() == 3) out.push_back({poly[0], poly[1], poly[2]});
  return out;
}

/// a junction's <boundary> (OpenDRIVE 1.8.0 §12.10) as a polygon with the road surface's height at each point: lane
/// segments along the outer border of their boundaryLane (0: the centre lane's line), joint segments across their road's
/// end; empty when a segment's road is missing
std::vector<P3> outline(const odr::Junction &J, const std::map<std::string, const odr::Road *> &roads, double step) {
  std::vector<P3> poly;
  auto add = [&](const odr::Road &r, double s, double t) {
    const Vec2 p = r.planView.eval(s).offset(t);
    if (!poly.empty() && std::hypot(p.x - poly.back().x, p.y - poly.back().y) < 1e-3) return;
    poly.push_back({p.x, p.y, odr::surfaceZ(r, s, t)});
  };
  for (const odr::BoundarySegment &seg : J.boundary) {
    const auto it = roads.find(seg.roadId);
    if (it == roads.end()) return {};
    const odr::Road &r = *it->second;
    const double L = r.length();
    auto sOf = [&](const std::string &v) {
      if (v == "start" || v == "begin") return 0.0;
      if (v == "end") return L;
      try {
        return std::stod(v);
      } catch (...) {
        return 0.0;
      }
    };
    if (seg.type == "joint") {  // across the road's end (its lanes jointLaneStart .. jointLaneEnd, Table 78, or all),
                                // from the outer edge nearer the outline so far
      const double s = seg.contactPoint == "start" ? 0.0 : L;
      const odr::LaneSection &sec = odr::sectionAt(r, s);
      const int idLo = std::min(seg.jointLaneStart, seg.jointLaneEnd), idHi = std::max(seg.jointLaneStart, seg.jointLaneEnd);
      double lo = 1e300, hi = -1e300;
      if (!seg.jointLanes) lo = hi = odr::laneOffsetAt(r, s);
      for (const auto *side : {&sec.left, &sec.right})
        for (const odr::Lane &l : *side) {
          double a, b;
          if (seg.jointLanes && (l.id < idLo || l.id > idHi)) continue;
          if (odr::laneBorders(r, s, l.id, a, b)) lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
        }
      if (lo > hi) return {};
      const Vec2 pl = r.planView.eval(s).offset(lo), ph = r.planView.eval(s).offset(hi);
      const bool loFirst = poly.empty() || std::hypot(pl.x - poly.back().x, pl.y - poly.back().y) <=
                                               std::hypot(ph.x - poly.back().x, ph.y - poly.back().y);
      add(r, s, loFirst ? lo : hi), add(r, s, loFirst ? hi : lo);
      continue;
    }
    const double s0 = sOf(seg.sStart), s1 = sOf(seg.sEnd);
    const int n = std::max(2, static_cast<int>(std::ceil(std::abs(s1 - s0) / step)) + 1);
    for (int i = 0; i < n; ++i) {
      const double s = s0 + (s1 - s0) * i / (n - 1);
      double in = 0.0, t = odr::laneOffsetAt(r, s);
      if (seg.boundaryLane != 0 && !odr::laneBorders(r, s, seg.boundaryLane, in, t)) continue;
      add(r, s, t);
    }
  }
  if (poly.size() > 1 && std::hypot(poly.front().x - poly.back().x, poly.front().y - poly.back().y) < 0.1) poly.pop_back();
  double area = 0.0;  // counter-clockwise as the spec orders it; reversed should a writer have it the other way
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) area += poly[j].x * poly[i].y - poly[i].x * poly[j].y;
  if (area < 0) std::reverse(poly.begin(), poly.end());
  return poly;
}

/// a junction's <elevationGrid> (OpenDRIVE 1.8.0 §12.11) at (x, y): rows at sStart + k gridSpacing along its reference
/// line, each with a center value and left / right values from inside to outside (t positive to the left), read
/// bilinearly between the grid points and linearly beyond the outer squares. §12.11.1 interpolates bicubically; on the
/// planes this converter writes (junction_surface.cpp junctionGrids) both give the plane exactly. False when the rows
/// do not form one rectangle of complete squares.
bool gridHeight(const odr::JunctionGrid &g, double x, double y, double &z) {
  if (g.rows.size() < 2 || g.gridSpacing <= 0.0) return false;
  const size_t nl = g.rows[0].left.size(), nr = g.rows[0].right.size();
  for (const auto &row : g.rows)
    if (row.left.size() != nl || row.right.size() != nr) return false;
  const int jlo = -static_cast<int>(nr), jhi = static_cast<int>(nl);
  if (jhi - jlo < 1) return false;
  const double c = std::cos(g.hdg), sn = std::sin(g.hdg);
  const double fk = ((x - g.x) * c + (y - g.y) * sn - g.sStart) / g.gridSpacing;
  const double fj = (-(x - g.x) * sn + (y - g.y) * c) / g.gridSpacing;
  const auto at = [&](int k, int j) {
    const auto &row = g.rows[static_cast<size_t>(k)];
    return j == 0 ? row.center : j > 0 ? row.left[static_cast<size_t>(j - 1)] : row.right[static_cast<size_t>(-j - 1)];
  };
  const int k0 = std::clamp(static_cast<int>(std::floor(fk)), 0, static_cast<int>(g.rows.size()) - 2);
  const int j0 = std::clamp(static_cast<int>(std::floor(fj)), jlo, jhi - 1);
  const double u = fk - k0, v = fj - j0;
  z = (1 - u) * (1 - v) * at(k0, j0) + u * (1 - v) * at(k0 + 1, j0) + (1 - u) * v * at(k0, j0 + 1) + u * v * at(k0 + 1, j0 + 1);
  return true;
}

/// every cell a bilinear reader interpolates a point inside the polygon from (centre inside it, or within one cell
/// diagonal of its edge) capped at the junction grid's height at the cell centre minus eps: a grid read bilinearly
/// reproduces the grid's plane exactly, as rasterTriangle does for a lane's. A cell under a lane of a road outside every
/// junction standing more than kOtherLevel over the grid (`plainCap`: that lane's cap) keeps the lane's cap: a boundary
/// is drawn in plan, and where its polygon reaches under a road beside the junction on a wall above it there is no
/// junction surface to press the ground to; an arm's lane end at the junction's level is capped by both, the lower wins
constexpr double kOtherLevel = 0.5;  // m
void rasterPolygon(std::vector<double> &cap, const TerrainGrid &g, const std::vector<P3> &poly, const odr::JunctionGrid &jg,
                   double eps, Box &box, const std::vector<double> &plainCap) {
  const auto capCell = [&](int i, int j) {
    if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) return;
    double z;
    if (!gridHeight(jg, g.originX + i * g.cell, g.originY + j * g.cell, z)) return;
    if (plainCap[static_cast<size_t>(j) * g.nx + i] > z - eps + kOtherLevel) return;
    double &k = cap[static_cast<size_t>(j) * g.nx + i];
    k = std::min(k, z - eps);
    box.i0 = std::min(box.i0, i), box.i1 = std::max(box.i1, i), box.j0 = std::min(box.j0, j), box.j1 = std::max(box.j1, j);
  };
  double y0 = 1e300, y1 = -1e300;
  for (const P3 &p : poly) y0 = std::min(y0, p.y), y1 = std::max(y1, p.y);
  for (int j = static_cast<int>(std::floor((y0 - g.originY) / g.cell)); j <= static_cast<int>(std::ceil((y1 - g.originY) / g.cell)); ++j) {
    const double y = g.originY + j * g.cell;  // the cell centres of the row inside the polygon (even-odd)
    std::vector<double> xs;
    for (size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++)
      if ((poly[a].y > y) != (poly[b].y > y)) xs.push_back(poly[a].x + (y - poly[a].y) * (poly[b].x - poly[a].x) / (poly[b].y - poly[a].y));
    std::sort(xs.begin(), xs.end());
    for (size_t m = 0; m + 1 < xs.size(); m += 2)
      for (int i = static_cast<int>(std::ceil((xs[m] - g.originX) / g.cell)); g.originX + i * g.cell <= xs[m + 1]; ++i) capCell(i, j);
  }
  const double reach = g.cell * std::sqrt(2.0);
  for (size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++) {  // the cells along every edge
    const Vec2 p{poly[b].x, poly[b].y}, d{poly[a].x - poly[b].x, poly[a].y - poly[b].y};
    const int i0 = static_cast<int>(std::floor((std::min(p.x, p.x + d.x) - reach - g.originX) / g.cell));
    const int i1 = static_cast<int>(std::ceil((std::max(p.x, p.x + d.x) + reach - g.originX) / g.cell));
    const int j0 = static_cast<int>(std::floor((std::min(p.y, p.y + d.y) - reach - g.originY) / g.cell));
    const int j1 = static_cast<int>(std::ceil((std::max(p.y, p.y + d.y) + reach - g.originY) / g.cell));
    for (int j = j0; j <= j1; ++j)
      for (int i = i0; i <= i1; ++i) {
        const Vec2 q{g.originX + i * g.cell, g.originY + j * g.cell};
        const double f = std::clamp((q - p).dot(d) / std::max(d.dot(d), 1e-12), 0.0, 1.0);
        if ((p + d * f - q).norm() <= reach) capCell(i, j);
      }
  }
}

/// the cap rising `slope` per metre away from where it is set: a two-pass chamfer (never under the true cone) over the
/// cells of `b` (its 8-neighbour steps between two cells of the box run inside the box)
void cone(std::vector<double> &cap, int nx, const Box &b, double cell, double slope) {
  const double a = slope * cell, d = slope * cell * std::sqrt(2.0);
  for (int j = b.j0; j <= b.j1; ++j)
    for (int i = b.i0; i <= b.i1; ++i) {
      const size_t k = static_cast<size_t>(j) * nx + i;
      double v = cap[k];
      if (i > b.i0) v = std::min(v, cap[k - 1] + a);
      if (j > b.j0) {
        v = std::min(v, cap[k - nx] + a);
        if (i > b.i0) v = std::min(v, cap[k - nx - 1] + d);
        if (i < b.i1) v = std::min(v, cap[k - nx + 1] + d);
      }
      cap[k] = v;
    }
  for (int j = b.j1; j >= b.j0; --j)
    for (int i = b.i1; i >= b.i0; --i) {
      const size_t k = static_cast<size_t>(j) * nx + i;
      double v = cap[k];
      if (i < b.i1) v = std::min(v, cap[k + 1] + a);
      if (j < b.j1) {
        v = std::min(v, cap[k + nx] + a);
        if (i < b.i1) v = std::min(v, cap[k + nx + 1] + d);
        if (i > b.i0) v = std::min(v, cap[k + nx - 1] + d);
      }
      cap[k] = v;
    }
}
}  // namespace

TerrainGrid conformGround(const odr::OpenDrive &od, const TerrainGrid &ground, const ConformOptions &opt) {
  TerrainGrid out = ground;
  out.levels = 1;
  out.z.assign(ground.z.begin(), ground.z.begin() + static_cast<long>(ground.nx) * ground.ny);
  const int nx = ground.nx, ny = ground.ny;
  const double cell = ground.cell;
  const size_t N = static_cast<size_t>(nx) * ny;
  // 1. the road surface under every lane (outside bridges and tunnels). Where roads share a cell (junction roads, the
  //    two legs of a hairpin side by side at different heights) the lowest surface is the ground: a higher road stands
  //    on its wall or embankment there, a lower one is not buried. One pass of a road over the cell gives the mean of
  //    its samples there; a road coming back to the cell after leaving it (a hairpin's other leg: more than two cells
  //    along s since its last sample there) is another pass - the legs of a tight hairpin can come back within 20 m
  struct Pass {
    double s, sum;  // s of the pass's last sample in the cell
    int n;
  };
  std::vector<double> lowest(N, kInf);
  std::vector<int> cnt(N, 0);
  std::unordered_map<size_t, std::vector<Pass>> own;  // this road's passes per cell
  const double step = 0.5 * cell;
  auto onList = [](const std::vector<odr::Structure> &list, double s) {
    for (const odr::Structure &st : list)
      if (s >= st.s - 1e-6 && s <= st.s + st.length + 1e-6) return true;
    return false;
  };
  for (const odr::Road &r : od.roads) {
    if (r.lanes.empty()) continue;
    const double L = r.length();
    own.clear();
    for (double s = 0.0; s <= L + 1e-9; s += step) {
      const double sc = std::min(s, L);
      if (onList(r.bridges, sc) || onList(r.tunnels, sc)) continue;
      const Pose p = r.planView.eval(sc);
      const odr::LaneSection &sec = odr::sectionAt(r, sc);
      double lo = kInf, hi = -kInf;
      for (const auto *side : {&sec.left, &sec.right})
        for (const odr::Lane &l : *side) {
          double a, b;
          if (!odr::laneBorders(r, sc, l.id, a, b)) continue;
          lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
        }
      if (!(hi > lo)) continue;
      for (double t = lo; t <= hi + step - 1e-9; t += step) {  // the far border itself is the last sample
        const double tc = std::min(t, hi);
        const Vec2 q = p.offset(tc);
        const int i = static_cast<int>(std::lround((q.x - ground.originX) / cell));
        const int j = static_cast<int>(std::lround((q.y - ground.originY) / cell));
        if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
        const size_t k = static_cast<size_t>(j) * nx + i;
        const double zs = odr::surfaceZ(r, sc, tc);
        std::vector<Pass> &ps = own[k];
        auto it = std::find_if(ps.begin(), ps.end(), [&](const Pass &q) { return std::abs(q.s - sc) <= 2.0 * cell; });
        if (it == ps.end()) ps.push_back({sc, zs, 1});
        else it->sum += zs, ++it->n, it->s = sc;
      }
    }
    for (const auto &[k, ps] : own) {
      for (const Pass &q : ps) lowest[k] = std::min(lowest[k], q.sum / q.n);
      ++cnt[k];
    }
  }
  // 2. distance to the nearest road cell and that cell's height (multi-source Dijkstra on the 8-neighbour grid)
  std::vector<double> dist(N, kInf), roadZ(N, std::numeric_limits<double>::quiet_NaN());
  using Item = std::pair<double, size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> pq;
  for (size_t k = 0; k < N; ++k)
    if (cnt[k] > 0) dist[k] = 0.0, roadZ[k] = lowest[k], pq.push({0.0, k});
  while (!pq.empty()) {
    const auto [d, k] = pq.top();
    pq.pop();
    if (d > dist[k] || d > opt.maxReach) continue;
    const int i = static_cast<int>(k % nx), j = static_cast<int>(k / nx);
    for (int dj = -1; dj <= 1; ++dj)
      for (int di = -1; di <= 1; ++di) {
        if (!di && !dj) continue;
        const int a = i + di, b = j + dj;
        if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
        const size_t m = static_cast<size_t>(b) * nx + a;
        const double nd = d + cell * ((di && dj) ? std::sqrt(2.0) : 1.0);
        if (nd < dist[m]) dist[m] = nd, roadZ[m] = roadZ[k], pq.push({nd, m});
      }
  }
  // 3. under a road its surface; beside it the cut / fill slope from the road edge, where it lies between the road and
  //    the terrain (a fill: the higher of embankment and terrain, a cut: the lower of cutting and terrain)
  for (size_t k = 0; k < N; ++k) {
    if (std::isnan(roadZ[k])) continue;
    if (cnt[k] > 0) {
      out.z[k] = static_cast<float>(roadZ[k]);
      continue;
    }
    const double g = out.z[k];
    if (std::isnan(g) || dist[k] > opt.maxReach) continue;
    const double drop = dist[k] / opt.slope;
    out.z[k] = static_cast<float>(roadZ[k] > g ? std::max(g, roadZ[k] - drop) : std::min(g, roadZ[k] + drop));
  }
  // 4. the road floor (decided with the user on 2026-10-18; the viewer draws the same, src/map/roadFloor.ts): the road
  //    wins over the terrain. Every lane of every road (bridges included: a deck presses only ground that rises above it)
  //    as quads between border samples, and every junction's <boundary> (1.8.0 §12.10) as its triangulated outline,
  //    caps the cells they are read from at their surface minus floorEpsilon; a road's own lanes inside its tunnels do
  //    not press (they run under the ground on purpose; a road over the tunnel still does). Where two roads at
  //    different heights touch, the lower wins. Away from them the cap rises at floorSlope (a 63-degree cut), and the
  //    ground is the lower of itself and the cap.
  std::vector<double> cap(N, kInf);
  std::vector<double> plainCap(N, -kInf);  // the highest cap a lane of a road outside every junction set at each cell
  Box box;  // every cell capped
  // quads a quarter of a cell long: the triangles' planes follow a banked, curving lane to well under the floor's 5 cm
  const double ds = 0.25 * cell;
  struct Edge {  // one lane's borders at one station
    bool ok = false;
    double width = 0.0;
    P3 inner{}, outer{};
  };
  for (const odr::Road &r : od.roads) {
    for (size_t k = 0; k < r.lanes.size(); ++k) {
      const double s0 = r.lanes[k].s, s1 = k + 1 < r.lanes.size() ? r.lanes[k + 1].s : r.length();
      if (s1 - s0 < 1e-6) continue;
      const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / ds)));
      std::vector<int> ids;
      for (const auto *side : {&r.lanes[k].left, &r.lanes[k].right})
        for (const odr::Lane &l : *side) ids.push_back(l.id);
      // every lane's borders at a station, once: a quad shares its far station with the next quad's near one
      auto station = [&](double s, std::vector<Edge> &e) {
        const Pose p = r.planView.eval(s);
        auto pt = [&](double t) {
          const Vec2 q = p.offset(t);
          return P3{q.x, q.y, odr::surfaceZ(r, s, t)};
        };
        e.assign(ids.size(), Edge{});
        for (size_t m = 0; m < ids.size(); ++m) {
          double in, outer;
          if (!odr::laneBorders(r, s, ids[m], in, outer)) continue;
          e[m] = {true, std::abs(outer - in), pt(in), pt(outer)};
        }
      };
      std::vector<Edge> ea, eb;
      station(s0, ea);
      for (int i = 0; i < n; ++i) {
        const double sa = s0 + (s1 - s0) * i / n, sb = s0 + (s1 - s0) * (i + 1) / n;
        station(sb, eb);
        if (!(onList(r.tunnels, sa) && onList(r.tunnels, sb)))  // wholly in the tunnel: no press (a portal's quad does)
          for (size_t m = 0; m < ids.size(); ++m) {
            const Edge &a = ea[m], &b = eb[m];
            if (!a.ok || !b.ok || (a.width < 1e-3 && b.width < 1e-3)) continue;
            std::vector<double> *plain = r.junction == "-1" ? &plainCap : nullptr;
            rasterTriangle(cap, ground, {a.inner, b.inner, b.outer}, opt.floorEpsilon, box, plain);
            rasterTriangle(cap, ground, {a.inner, b.outer, a.outer}, opt.floorEpsilon, box, plain);
          }
        std::swap(ea, eb);
      }
    }
  }
  std::map<std::string, const odr::Road *> byId;
  std::map<std::string, bool> underCover;  // junction id: one of its roads runs in a tunnel (elevation.cpp junctionsUnderCover)
  for (const odr::Road &r : od.roads) {
    byId[r.id] = &r;
    if (r.junction != "-1" && !r.tunnels.empty()) underCover[r.junction] = true;
  }
  for (const odr::Junction &J : od.junctions) {
    // a junction under cover is under the ground on purpose: its outline does not press (its lanes outside their
    // tunnel records still do)
    if (J.boundary.empty() || underCover.count(J.id)) continue;
    // its surface inside the boundary is its <elevationGrid> ("applied to all points within this boundary", §12.10 /
    // §12.11); without one (or one that is not a rectangle) the outline triangulated, at the road surface at its points
    const std::vector<P3> poly = outline(J, byId, 2.0 * cell);
    double probe;
    if (J.grid && poly.size() >= 3 && gridHeight(*J.grid, poly[0].x, poly[0].y, probe)) {
      rasterPolygon(cap, ground, poly, *J.grid, opt.floorEpsilon, box, plainCap);
      continue;
    }
    for (const auto &t : triangulate(poly)) rasterTriangle(cap, ground, t, opt.floorEpsilon, box);
  }
  if (box.i1 < 0) return out;
  // the cone matters only within its reach of a capped cell: farther out it stands above the highest ground. The box of
  // the capped cells grown by that reach holds every cell it lowers, and the steps from a capped cell to it
  double capMin = kInf, groundMax = -kInf;
  for (int j = box.j0; j <= box.j1; ++j)
    for (int i = box.i0; i <= box.i1; ++i) capMin = std::min(capMin, cap[static_cast<size_t>(j) * nx + i]);
  for (const float z : out.z)
    if (!std::isnan(z)) groundMax = std::max(groundMax, static_cast<double>(z));
  const int reach = groundMax > capMin ? static_cast<int>(std::ceil((groundMax - capMin) / (opt.floorSlope * cell))) + 1 : 1;
  box.i0 = std::max(0, box.i0 - reach), box.j0 = std::max(0, box.j0 - reach);
  box.i1 = std::min(nx - 1, box.i1 + reach), box.j1 = std::min(ny - 1, box.j1 + reach);
  cone(cap, nx, box, cell, opt.floorSlope);
  for (int j = box.j0; j <= box.j1; ++j)
    for (int i = box.i0; i <= box.i1; ++i) {
      const size_t k = static_cast<size_t>(j) * nx + i;
      if (cap[k] < out.z[k]) out.z[k] = static_cast<float>(cap[k]);
    }
  return out;
}

SeatReport seatObjects(odr::OpenDrive &od, const TerrainGrid &conformed) {
  SeatReport rep;
  for (odr::Road &r : od.roads)
    for (odr::Object &o : r.objects) {
      const bool standing = o.type == "tree" || o.type == "pole" || o.type == "streetLamp";
      if (!standing || !o.outline.empty() || !o.repeats.empty()) continue;
      bool onBridge = false;
      for (const odr::Structure &b : r.bridges) onBridge |= o.s >= b.s - 1e-6 && o.s <= b.s + b.length + 1e-6;
      if (onBridge) {
        ++rep.onBridges;
        continue;
      }
      const Vec2 p = r.planView.eval(o.s).offset(o.t);
      const double g = conformed.sample(p.x, p.y);
      if (std::isnan(g)) continue;
      o.zOffset = g - odr::elevationAt(r, o.s);
      rep.maxLift = std::max(rep.maxLift, o.zOffset), rep.maxDrop = std::max(rep.maxDrop, -o.zOffset);
      ++rep.seated;
    }
  return rep;
}

}  // namespace odrforge
