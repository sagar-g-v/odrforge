// OdrForge - joining OSM near-misses.
//
// A way that ends a few metres short of another road without sharing a node (a driveway drawn to
// the kerb, a street stopped at the edge of the carriageway it joins) would become a road that
// leads nowhere. Such an end is joined to the road it meets: the end node gets connected to the
// nearest point of that road (a new node inserted into it), so the junction stage builds a normal
// junction there. Real dead ends are kept: `noexit`, turning circles / loops, ends at buildings,
// parking structures and barriers, ends pointing away from the road, other layers.
#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <tuple>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

int wayLayer(const OsmWay &w) {
  int l = tagYes(w.tags, "bridge") ? 1 : tagYes(w.tags, "tunnel") ? -1 : 0;  // a building passage: at ground level
  if (auto lv = tag(w.tags, "layer")) {
    try {
      l = std::stoi(*lv);
    } catch (...) {
    }
  }
  return l;
}

bool motorwayClass(const OsmWay &w) {
  const std::string hw = tag(w.tags, "highway").value_or("");
  return hw == "motorway" || hw == "motorway_link";
}

/// half of the carriageway width (no sidewalks) the OSM line stands for
double halfWidth(const OsmWay &w, const BuildOptions &opt) {
  const CrossSection x = crossSectionFromTags(w.tags, opt.country, opt.lanes);
  double width = 0;
  for (const SideSpec *s : {&x.left, &x.right})
    for (const auto &l : s->lanes)
      if (l.kind != LaneKind::Sidewalk) width += l.width;
  return 0.5 * width;
}

}  // namespace

std::set<OsmId> throughParkingAisles(const OsmData &osm, const BuildOptions &opt) {
  // aisles: service ways of an excluded kind (parking_aisle, drive-through), not areas or closed
  std::vector<const OsmWay *> aisles;
  std::unordered_set<OsmId> roadNodes;
  for (const auto &[id, w] : osm.ways) {
    if (isRoadWay(osm, w, opt)) {
      for (OsmId n : w.nodes) roadNodes.insert(n);
      continue;
    }
    const std::string sv = tag(w.tags, "service").value_or("");
    if (!tagIs(w.tags, "highway", "service") || (sv != "parking_aisle" && sv != "drive-through")) continue;
    if (tagYes(w.tags, "area") || tagIs(w.tags, "access", "no") || w.nodes.size() < 2) continue;
    // surface lots only: aisles inside garages (tunnel, covered, indoor, level, layer) have no
    // measurable surface and no structure to drive on in the simulation
    if (tag(w.tags, "tunnel") || tagYes(w.tags, "covered") || tagYes(w.tags, "indoor") || tag(w.tags, "level") ||
        (tag(w.tags, "layer") && tag(w.tags, "layer").value() != "0"))
      continue;
    bool complete = true;
    for (OsmId n : w.nodes) complete = complete && osm.node(n);
    if (complete) aisles.push_back(&w);
  }
  // lots = connected components of aisles (shared nodes)
  std::unordered_map<OsmId, OsmId> parent;
  std::function<OsmId(OsmId)> find = [&](OsmId x) { return parent[x] == x ? x : parent[x] = find(parent[x]); };
  std::unordered_map<OsmId, OsmId> firstAt;  // node -> first aisle using it
  for (const OsmWay *w : aisles) parent[w->id] = w->id;
  for (const OsmWay *w : aisles)
    for (OsmId n : w->nodes) {
      auto [it, fresh] = firstAt.emplace(n, w->id);
      if (!fresh) parent[find(w->id)] = find(it->second);
    }
  std::unordered_map<OsmId, std::unordered_set<OsmId>> attach;  // lot -> road nodes it touches
  for (const OsmWay *w : aisles)
    for (OsmId n : w->nodes)
      if (roadNodes.count(n)) attach[find(w->id)].insert(n);
  std::set<OsmId> out;
  for (const OsmWay *w : aisles)
    if (attach[find(w->id)].size() >= 2) out.insert(w->id);
  return out;
}

OsmData snapDanglingEnds(const OsmData &in, const Projection &proj, const BuildOptions &opt, int &snapped, int *crossings) {
  snapped = 0;
  if (crossings) *crossings = 0;
  OsmData out = in;
  if (opt.snapDanglingEnds <= 0) return out;
  std::vector<const OsmWay *> roads;
  for (const auto &[id, w] : in.ways)
    if (isRoadWay(in, w, opt)) roads.push_back(&w);
  std::sort(roads.begin(), roads.end(), [](const OsmWay *a, const OsmWay *b) { return a->id < b->id; });
  std::unordered_map<OsmId, Vec2> pos;
  auto P = [&](OsmId n) {
    auto it = pos.find(n);
    if (it != pos.end()) return it->second;
    const OsmNode *nd = in.node(n);
    return pos[n] = proj.forward(nd->lat, nd->lon);
  };
  // how many road-way ends / passes use a node
  std::unordered_map<OsmId, int> degree;
  for (const OsmWay *w : roads)
    for (size_t i = 0; i < w->nodes.size(); ++i) degree[w->nodes[i]] += (i == 0 || i + 1 == w->nodes.size()) ? 1 : 2;
  // nodes on buildings, parking structures and barriers: a road ending there enters them
  std::unordered_set<OsmId> entrance;
  for (const auto &[id, w] : in.ways)
    if (tag(w.tags, "building") || tag(w.tags, "barrier") || tagIs(w.tags, "amenity", "parking") || tag(w.tags, "parking"))
      for (OsmId n : w.nodes) entrance.insert(n);
  // segment index
  const double cell = 25.0;
  std::map<std::pair<long, long>, std::vector<std::pair<const OsmWay *, size_t>>> grid;
  for (const OsmWay *w : roads)
    for (size_t i = 0; i + 1 < w->nodes.size(); ++i) {
      const Vec2 a = P(w->nodes[i]), b = P(w->nodes[i + 1]);
      for (long gx = static_cast<long>(std::floor(std::min(a.x, b.x) / cell)); gx <= static_cast<long>(std::floor(std::max(a.x, b.x) / cell)); ++gx)
        for (long gy = static_cast<long>(std::floor(std::min(a.y, b.y) / cell)); gy <= static_cast<long>(std::floor(std::max(a.y, b.y) / cell)); ++gy)
          grid[{gx, gy}].push_back({w, i});
    }
  std::unordered_map<OsmId, double> hw;  // target half widths (cached)
  auto onBoundary = [&](const Vec2 &p) {
    if (!opt.clipBox) return false;
    const auto &b = *opt.clipBox;
    return std::min({p.x - b[0], b[2] - p.x, p.y - b[1], b[3] - p.y}) < 1.0;
  };
  struct Snap {
    OsmId way;      ///< dangling way
    bool atFront;   ///< its first node dangles
    OsmId target;   ///< way it joins
    size_t seg;     ///< segment index in the target
    double t;       ///< position on the segment
    Vec2 q;         ///< joining point
  };
  std::vector<Snap> snaps;
  for (const OsmWay *w : roads) {
    if (tagYes(w->tags, "noexit")) continue;
    for (bool front : {true, false}) {
      const OsmId n = front ? w->nodes.front() : w->nodes.back();
      if (w->closed() || degree[n] != 1 || entrance.count(n)) continue;
      const OsmNode *nd = in.node(n);
      if (tagYes(nd->tags, "noexit") || tagIs(nd->tags, "highway", "turning_circle") ||
          tagIs(nd->tags, "highway", "turning_loop") || tag(nd->tags, "barrier") || tag(nd->tags, "entrance"))
        continue;
      const Vec2 p = P(n), prev = P(front ? w->nodes[1] : w->nodes[w->nodes.size() - 2]);
      if (onBoundary(p)) continue;
      const Vec2 u = (p - prev) * (1.0 / std::max(1e-9, (p - prev).norm()));
      const int layer = wayLayer(*w);
      double best = 1e9;
      Snap s{w->id, front, 0, 0, 0, {}};
      const long cx = static_cast<long>(std::floor(p.x / cell)), cy = static_cast<long>(std::floor(p.y / cell));
      for (long dx = -1; dx <= 1; ++dx)
        for (long dy = -1; dy <= 1; ++dy) {
          auto it = grid.find({cx + dx, cy + dy});
          if (it == grid.end()) continue;
          for (const auto &[tw, i] : it->second) {
            if (tw->id == w->id || wayLayer(*tw) != layer) continue;
            if (motorwayClass(*tw) && !motorwayClass(*w)) continue;  // no driveways onto freeways
            const Vec2 a = P(tw->nodes[i]), b = P(tw->nodes[i + 1]), ab = b - a;
            const double t = std::clamp((p - a).dot(ab) / std::max(1e-12, ab.dot(ab)), 0.0, 1.0);
            const Vec2 q = a + ab * t;
            const double d = (q - p).norm();
            if (!hw.count(tw->id)) hw[tw->id] = halfWidth(*tw, opt);
            if (d > hw[tw->id] + opt.snapDanglingEnds || d >= best) continue;
            if (d > 1.0 && u.dot(q - p) / d < 0.5) continue;  // must point at the road (within 60 degrees)
            best = d;
            s.target = tw->id, s.seg = i, s.t = t, s.q = q;
          }
        }
      if (s.target) snaps.push_back(s);
    }
  }
  // apply: joining nodes are inserted into the targets (highest segment / position first so the
  // recorded indices stay valid), then appended to the dangling ways
  std::sort(snaps.begin(), snaps.end(), [](const Snap &a, const Snap &b) {
    return a.target != b.target ? a.target < b.target : (a.seg != b.seg ? a.seg > b.seg : a.t > b.t);
  });
  OsmId synth = -1;
  for (const auto &[id, n] : out.nodes) synth = std::min(synth, id - 1);
  std::vector<OsmId> joins;
  for (const Snap &s : snaps) {
    OsmWay &tw = out.ways[s.target];
    const Vec2 a = P(tw.nodes[s.seg]), b = P(tw.nodes[s.seg + 1]);
    OsmId join;
    if ((s.q - a).norm() < 1.0) join = tw.nodes[s.seg];
    else if ((s.q - b).norm() < 1.0) join = tw.nodes[s.seg + 1];
    else {
      OsmNode nn;
      nn.id = synth--;
      proj.inverse(s.q, nn.lat, nn.lon);
      out.nodes[nn.id] = nn;
      pos[nn.id] = s.q;
      tw.nodes.insert(tw.nodes.begin() + static_cast<long>(s.seg) + 1, nn.id);
      join = nn.id;
    }
    joins.push_back(join);
  }
  for (size_t k = 0; k < snaps.size(); ++k) {
    OsmWay &w = out.ways[snaps[k].way];
    if (snaps[k].atFront) w.nodes.insert(w.nodes.begin(), joins[k]);
    else w.nodes.push_back(joins[k]);
    ++snapped;
  }
  // minor roads (service, residential, living street) whose lines cross on the same level without
  // a shared node: the intersection node is missing (parking lots, campuses); major roads crossing
  // without a node are grade separations and are left to the elevation stage
  if (crossings) *crossings = 0;
  auto minor = [](const OsmWay &w) {
    const std::string hw = tag(w.tags, "highway").value_or("");
    return hw == "service" || hw == "residential" || hw == "living_street" || hw == "unclassified" || hw == "road";
  };
  auto structure = [](const OsmWay &w) {
    return tag(w.tags, "bridge") || tag(w.tags, "tunnel") || tagYes(w.tags, "covered") || tag(w.tags, "level") ||
           (tag(w.tags, "layer") && tag(w.tags, "layer").value() != "0");
  };
  std::vector<OsmWay *> mw;
  for (auto &[id, w] : out.ways)
    if (isRoadWay(out, w, opt) && minor(w) && !structure(w)) mw.push_back(&w);
  std::sort(mw.begin(), mw.end(), [](const OsmWay *a, const OsmWay *b) { return a->id < b->id; });
  auto Q = [&](OsmId n) {
    auto it = pos.find(n);
    if (it != pos.end()) return it->second;
    const OsmNode *nd = out.node(n);
    return pos[n] = proj.forward(nd->lat, nd->lon);
  };
  std::map<std::pair<long, long>, std::vector<std::pair<size_t, size_t>>> cells;
  for (size_t k = 0; k < mw.size(); ++k)
    for (size_t i = 0; i + 1 < mw[k]->nodes.size(); ++i) {
      const Vec2 a = Q(mw[k]->nodes[i]), b = Q(mw[k]->nodes[i + 1]);
      for (long gx = static_cast<long>(std::floor(std::min(a.x, b.x) / cell)); gx <= static_cast<long>(std::floor(std::max(a.x, b.x) / cell)); ++gx)
        for (long gy = static_cast<long>(std::floor(std::min(a.y, b.y) / cell)); gy <= static_cast<long>(std::floor(std::max(a.y, b.y) / cell)); ++gy)
          cells[{gx, gy}].push_back({k, i});
    }
  struct Cross {
    size_t way, seg;
    double t;
    Vec2 p;
    long id;  ///< crossing index (both ways share the node)
  };
  std::vector<Cross> cross;
  std::set<std::tuple<size_t, size_t, size_t, size_t>> seen;
  long nCross = 0;
  for (const auto &[key, segsIn] : cells)
    for (size_t x = 0; x < segsIn.size(); ++x)
      for (size_t y = x + 1; y < segsIn.size(); ++y) {
        auto [k1, i1] = segsIn[x];
        auto [k2, i2] = segsIn[y];
        if (k1 == k2) continue;
        if (k1 > k2) std::swap(k1, k2), std::swap(i1, i2);
        if (!seen.insert({k1, i1, k2, i2}).second) continue;
        const OsmWay &w1 = *mw[k1], &w2 = *mw[k2];
        const OsmId a1 = w1.nodes[i1], b1 = w1.nodes[i1 + 1], a2 = w2.nodes[i2], b2 = w2.nodes[i2 + 1];
        if (a1 == a2 || a1 == b2 || b1 == a2 || b1 == b2) continue;  // already joined
        const Vec2 p = Q(a1), r = Q(b1) - p, q = Q(a2), sv = Q(b2) - q;
        const double den = r.cross(sv);
        if (std::abs(den) < 1e-9) continue;
        const double t = (q - p).cross(sv) / den, u = (q - p).cross(r) / den;
        if (t <= 0.0 || t >= 1.0 || u <= 0.0 || u >= 1.0) continue;
        const Vec2 at = p + r * t;
        cross.push_back({k1, i1, t, at, nCross});
        cross.push_back({k2, i2, u, at, nCross});
        ++nCross;
      }
  std::vector<OsmId> crossNode(nCross, 0);
  // nodes already near the crossing on either way are reused
  for (const Cross &c : cross) {
    const OsmWay &w = *mw[c.way];
    for (OsmId n : {w.nodes[c.seg], w.nodes[c.seg + 1]})
      if ((Q(n) - c.p).norm() < 1.0 && !crossNode[c.id]) crossNode[c.id] = n;
  }
  for (long c = 0; c < nCross; ++c)
    if (!crossNode[c]) {
      const Cross *any = nullptr;
      for (const Cross &x : cross)
        if (x.id == c) any = &x;
      OsmNode nn;
      nn.id = synth--;
      proj.inverse(any->p, nn.lat, nn.lon);
      out.nodes[nn.id] = nn;
      pos[nn.id] = any->p;
      crossNode[c] = nn.id;
    }
  std::sort(cross.begin(), cross.end(), [](const Cross &a, const Cross &b) {
    return a.way != b.way ? a.way < b.way : (a.seg != b.seg ? a.seg > b.seg : a.t > b.t);
  });
  for (const Cross &c : cross) {
    OsmWay &w = *mw[c.way];
    const OsmId n = crossNode[c.id];
    if (w.nodes[c.seg] == n || w.nodes[c.seg + 1] == n) continue;
    w.nodes.insert(w.nodes.begin() + static_cast<long>(c.seg) + 1, n);
  }
  if (crossings) *crossings = static_cast<int>(nCross);
  return out;
}

}  // namespace detail
}  // namespace odrforge
