#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <chrono>
#include <execution>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <tuple>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

bool lht(const BuildOptions &o) { return o.country.leftHandTraffic; }

/// carriageway extents (left, right) of a cross section relative to the reference line,
/// including parking and shoulders but not sidewalks (used for junction overlap tests)
void vehicleExtents(const CrossSection &x, double &left, double &right) {
  auto ext = [](const SideSpec &s) {
    double w = 0;
    for (const auto &l : s.lanes)
      if (l.kind != LaneKind::Sidewalk) w += l.width;
    return w;
  };
  left = x.laneOffset + ext(x.left);
  right = -x.laneOffset + ext(x.right);
}

std::string odrRoadType(const std::string &hw) {
  if (hw == "motorway" || hw == "motorway_link") return "motorway";
  if (hw == "trunk" || hw == "trunk_link") return "townExpressway";
  if (hw == "living_street" || hw == "service") return "lowSpeed";
  if (hw == "pedestrian") return "pedestrian";
  return "town";
}

/// OSM layer of a segment: the layer tag, else implied - a bridge above the ground, a tunnel below
/// it; a building passage runs through a building at ground level
int segLayer(const Seg &sg) {
  int layer = sg.bridge ? 1 : sg.tunnel && !tagIs(*sg.tags, "tunnel", "building_passage") ? -1 : 0;
  if (auto lv = tag(*sg.tags, "layer")) {
    try {
      layer = std::stoi(*lv);
    } catch (...) {
    }
  }
  return layer;
}

double minRadiusFor(const std::string &hw) {
  if (hw == "motorway" || hw == "trunk") return 60.0;
  if (hw == "motorway_link" || hw == "trunk_link") return 25.0;
  if (hw == "primary" || hw == "secondary") return 15.0;
  return 8.0;
}

}  // namespace

int Builder::classRank(const std::string &hw) {
  static const std::map<std::string, int> r = {
      {"motorway", 10}, {"trunk", 9},       {"primary", 8},     {"secondary", 7},       {"tertiary", 6},
      {"motorway_link", 9}, {"trunk_link", 8}, {"primary_link", 7}, {"secondary_link", 6}, {"tertiary_link", 5},
      {"unclassified", 4}, {"residential", 4}, {"living_street", 2}, {"service", 1}, {"busway", 3}, {"road", 3}};
  auto it = r.find(hw);
  return it == r.end() ? 3 : it->second;
}

const CrossSection &Builder::sectionAtFull(const RoadMeta &m, double sFull) const {
  const SectionInfo *sec = &m.sections.front();
  for (const auto &s : m.sections)
    if (s.s <= sFull + 1e-6) sec = &s;
  return sec->xs;
}

std::vector<int> Builder::endLanes(const odr::Road &r, bool atEnd, bool incoming) const {
  // a lane travels +s when it lies right of the reference line in RHT (left in LHT)
  const odr::LaneSection &sec = odr::sectionAt(r, atEnd ? r.length() : 0.0);
  const bool wantPlus = (atEnd == incoming);  // toward the end == +s
  std::vector<int> out;
  auto take = [&](const std::vector<odr::Lane> &side) {
    for (const auto &l : side) {
      // driving, and the motorway lane types of OpenDRIVE 1.8.0 §11.7.1 (direct junctions type them)
      if (l.type != "driving" && l.type != "entry" && l.type != "exit" && l.type != "onRamp" && l.type != "offRamp" &&
          l.type != "connectingRamp")
        continue;
      const bool plus = (l.id < 0) != lht(opt);
      if (plus == wantPlus) out.push_back(l.id);
    }
  };
  take(sec.right);
  take(sec.left);
  return out;
}

// ---------------------------------------------------------------------------------------- stages

double classMinRadius(const std::string &hw) { return minRadiusFor(hw); }
void carriagewayExtents(const CrossSection &x, double &left, double &right) { vehicleExtents(x, left, right); }

bool isRoadWay(const OsmData &osm, const OsmWay &w, const BuildOptions &opt) {
  const auto hw = tag(w.tags, "highway");
  if (!hw || !opt.highways.count(*hw) || w.nodes.size() < 2) return false;
  if (tagYes(w.tags, "area")) return false;
  if (opt.includeWays.count(w.id)) {
    for (OsmId n : w.nodes)
      if (!osm.node(n)) return false;
    return true;
  }
  if (*hw == "service")
    if (auto sv = tag(w.tags, "service"); sv && opt.excludeService.count(*sv)) return false;
  if (tagIs(w.tags, "access", "no") && !tag(w.tags, "bus") && !tag(w.tags, "psv")) return false;
  for (OsmId n : w.nodes)
    if (!osm.node(n)) return false;
  return true;
}

void Builder::collectSegments() {
  std::unordered_map<OsmId, int> usage;
  std::vector<const OsmWay *> ways;
  for (const auto &[id, w] : osm.ways) {
    if (!isRoadWay(osm, w, opt)) continue;
    ways.push_back(&w);
    for (size_t i = 0; i < w.nodes.size(); ++i) usage[w.nodes[i]] += (i == 0 || i + 1 == w.nodes.size()) ? 2 : 1;
  }
  std::sort(ways.begin(), ways.end(), [](const OsmWay *a, const OsmWay *b) { return a->id < b->id; });
  for (const OsmWay *w : ways) {
    const CrossSection xs = crossSectionFromTags(w->tags, opt.country, opt.lanes);
    const std::string ow = tag(w->tags, "oneway").value_or("");
    int travel = 0;
    if (xs.oneway) travel = (ow == "-1" || ow == "reverse") ? -1 : 1;
    Seg cur;
    cur.way = w->id;
    cur.tags = &w->tags;
    cur.xs = xs;
    cur.travel = travel;
    cur.bridge = tagYes(w->tags, "bridge") || tagIs(w->tags, "bridge", "viaduct");
    cur.tunnel = tagYes(w->tags, "tunnel") || tagIs(w->tags, "tunnel", "building_passage");
    cur.covered = tagYes(w->tags, "covered");
    cur.passage = tagIs(w->tags, "tunnel", "building_passage");
    for (size_t i = 0; i < w->nodes.size(); ++i) {
      cur.nodes.push_back(w->nodes[i]);
      const bool split = i > 0 && i + 1 < w->nodes.size() && usage[w->nodes[i]] >= 2;
      if (split || i + 1 == w->nodes.size()) {
        if (cur.nodes.size() >= 2) segs.push_back(cur);
        cur.nodes = {w->nodes[i]};
      }
    }
  }
  for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
    nodeSegs[segs[i].nodes.front()].push_back({i, true});
    nodeSegs[segs[i].nodes.back()].push_back({i, false});
  }
}

void Builder::findJunctionNodes() {
  for (const auto &[n, v] : nodeSegs) {
    if (v.size() >= 3) {
      junctionNodes.insert(n);
    } else if (v.size() == 2) {
      // two oneways that both enter or both leave the node cannot be one road
      auto arrives = [&](const std::pair<int, bool> &e) {
        const Seg &s = segs[e.first];
        if (!s.travel) return 0;
        const bool alongWay = s.travel > 0;
        // a segment end at its last node receives traffic that travels along the way
        return (!e.second == alongWay) ? 1 : -1;
      };
      const int a = arrives(v[0]), b = arrives(v[1]);
      if (a != 0 && b != 0 && a == b) junctionNodes.insert(n);
    }
  }
}

void Builder::buildChains() {
  std::vector<bool> used(segs.size(), false);
  auto passThrough = [&](OsmId n) { return nodeSegs[n].size() == 2 && !junctionNodes.count(n); };
  auto otherAt = [&](OsmId n, int seg) -> std::pair<int, bool> {
    for (const auto &e : nodeSegs[n])
      if (e.first != seg) return e;
    return {-1, false};
  };
  auto endNode = [&](const Part &p) { return p.fwd ? segs[p.seg].nodes.back() : segs[p.seg].nodes.front(); };
  auto startNode = [&](const Part &p) { return p.fwd ? segs[p.seg].nodes.front() : segs[p.seg].nodes.back(); };
  for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
    if (used[i]) continue;
    Chain ch;
    ch.parts.push_back({i, segs[i].travel >= 0});
    used[i] = true;
    // forward
    for (;;) {
      const OsmId n = endNode(ch.parts.back());
      if (!passThrough(n)) break;
      auto [s, atStart] = otherAt(n, ch.parts.back().seg);
      if (s < 0 || used[s]) break;
      ch.parts.push_back({s, atStart});
      used[s] = true;
    }
    // backward
    for (;;) {
      const OsmId n = startNode(ch.parts.front());
      if (!passThrough(n)) break;
      auto [s, atStart] = otherAt(n, ch.parts.front().seg);
      if (s < 0 || used[s]) break;
      ch.parts.insert(ch.parts.begin(), {s, !atStart});
      used[s] = true;
    }
    // orient along the traffic of the first oneway part
    for (const Part &p : ch.parts) {
      const int tr = segs[p.seg].travel;
      if (!tr) continue;
      if ((tr > 0) != p.fwd) {
        std::reverse(ch.parts.begin(), ch.parts.end());
        for (Part &q : ch.parts) q.fwd = !q.fwd;
      }
      break;
    }
    ch.n0 = startNode(ch.parts.front());
    ch.n1 = endNode(ch.parts.back());
    for (const Part &p : ch.parts)
      for (size_t k = 1; k < segs[p.seg].nodes.size(); ++k)
        ch.approxLength += (pos(segs[p.seg].nodes[k]) - pos(segs[p.seg].nodes[k - 1])).norm();
    chains.push_back(ch);
  }
}

namespace {
constexpr double kMaxAbsorbedRoad = 15.0;  // m: a wide median; longer roads between junction nodes stay roads
constexpr double kKeptRoad = 2.0;  // m: the shortest a road's junctions cut it down to (half of a shorter road)
constexpr double kMinRoadBetweenJunctions = 3.0;  // m: less of a road left between two junctions: one intersection
/// m: the least of a road the cut past its junction's turns leaves (as long as an absorbed road: a shorter one between
/// two junctions cut to a sliver drops out of the OSM way's line - Frankfurt road 183, 11.4 -> 3 m, LA road 971)
constexpr double kArmCutKeeps = kMaxAbsorbedRoad;
}  // namespace

void Builder::clusterJunctions() {
  // union-find over junction nodes
  std::unordered_map<OsmId, OsmId> parent;
  std::function<OsmId(OsmId)> find = [&](OsmId x) {
    auto it = parent.find(x);
    if (it == parent.end() || it->second == x) return x;
    return it->second = find(it->second);
  };
  for (OsmId n : junctionNodes) parent[n] = n;
  auto unite = [&](OsmId a, OsmId b) { parent[find(a)] = find(b); };
  // node radius: half of the widest carriageway meeting there
  std::unordered_map<OsmId, double> radius;
  for (OsmId n : junctionNodes) {
    double r = 0;
    for (const auto &[s, atStart] : nodeSegs[n]) {
      double L, R;
      vehicleExtents(segs[s].xs, L, R);
      r = std::max(r, 0.5 * (L + R));
    }
    radius[n] = r;
  }
  // with measured terrain: two nodes belong to one junction surface only if some measured surface
  // (ground or a deck level) at each can be joined within a junction grade - a ramp between a street
  // and a trenched freeway is a road, not part of one junction
  auto levelsAt = [&](OsmId n) {
    std::vector<double> z;
    if (!opt.terrain) return z;
    const Vec2 p = pos(n);
    if (!opt.terrain->ground.empty()) {
      const double g = opt.terrain->ground_at(p.x, p.y);
      if (!std::isnan(g)) z.push_back(g);
    }
    for (double d : opt.terrain->structure_levels(p.x, p.y)) z.push_back(d);
    return z;
  };
  auto sameLevel = [&](OsmId a, OsmId b, double dist) {
    const auto za = levelsAt(a), zb = levelsAt(b);
    if (za.empty() || zb.empty()) return true;  // unknown: trust the topology
    for (double x : za)
      for (double y : zb)
        if (std::abs(x - y) <= 0.5 + 0.12 * dist) return true;
    return false;
  };
  for (Chain &c : chains) {
    if (!junctionNodes.count(c.n0) || !junctionNodes.count(c.n1)) continue;
    if (c.n0 == c.n1) continue;
    if (c.approxLength < radius[c.n0] + radius[c.n1] + opt.junctionJoinSlack && sameLevel(c.n0, c.n1, c.approxLength)) {
      unite(c.n0, c.n1);
      c.internal = true;
    }
  }
  // geometrically coincident junction nodes (mapping artefacts)
  // layer of the ways meeting at a node (bridge without layer = 1, tunnel = -1)
  auto nodeLayer = [&](OsmId n) {
    int layer = 0;
    for (const auto &[si, atStart] : nodeSegs[n])
      if (const int l = segLayer(segs[si]); std::abs(l) > std::abs(layer)) layer = l;
    return layer;
  };
  std::vector<OsmId> jn(junctionNodes.begin(), junctionNodes.end());
  std::sort(jn.begin(), jn.end());
  for (size_t i = 0; i < jn.size(); ++i)
    for (size_t j = i + 1; j < jn.size(); ++j)
      if ((pos(jn[i]) - pos(jn[j])).norm() < 2.0 && nodeLayer(jn[i]) == nodeLayer(jn[j]) && sameLevel(jn[i], jn[j], 2.0))
        unite(jn[i], jn[j]);

  // junctions that squeezed the road between them out (joinSqueezedJunctions), on one level only
  for (const auto &[a, b] : forcedJoins)
    if (nodeLayer(a) == nodeLayer(b) && sameLevel(a, b, (pos(a) - pos(b)).norm())) unite(a, b);

  std::map<OsmId, int> rootIdx;
  for (OsmId n : jn) {
    const OsmId r = find(n);
    if (!rootIdx.count(r)) {
      rootIdx[r] = static_cast<int>(clusters.size());
      clusters.emplace_back();
    }
    const int ci = rootIdx[r];
    clusterOf[n] = ci;
    clusters[ci].nodes.push_back(n);
  }
  for (Cluster &c : clusters) {
    Vec2 sum;
    for (OsmId n : c.nodes) {
      sum = sum + pos(n);
      if (const OsmNode *nd = osm.node(n)) {
        if (tagIs(nd->tags, "highway", "traffic_signals")) c.signalised = true;
        if (tagIs(nd->tags, "highway", "crossing")) c.crossing = true;
      }
    }
    c.center = sum * (1.0 / c.nodes.size());
  }
  for (Chain &c : chains) {
    if (auto it = clusterOf.find(c.n0); it != clusterOf.end()) c.c0 = it->second;
    if (auto it = clusterOf.find(c.n1); it != clusterOf.end()) c.c1 = it->second;
    // a short chain with both ends in the same cluster lies inside the junction; a longer one (a
    // loop street between two nodes of one junction) stays a road starting and ending there
    const bool shortEnough = c.approxLength <= std::max(kMaxAbsorbedRoad, radius[c.n0] + radius[c.n1] + opt.junctionJoinSlack);
    if (c.c0 >= 0 && c.c0 == c.c1 && c.n0 != c.n1 && shortEnough) c.internal = true;
  }
}

void Builder::buildRoadGeometry() {
  for (int ci = 0; ci < static_cast<int>(chains.size()); ++ci) {
    const Chain &ch = chains[ci];
    if (ch.internal) continue;
    RoadMeta m;
    m.chain = ci;
    std::vector<Vec2> pts;
    std::vector<OsmId> ids;
    std::vector<size_t> partStart;
    for (const Part &p : ch.parts) {
      std::vector<OsmId> nodes = segs[p.seg].nodes;
      if (!p.fwd) std::reverse(nodes.begin(), nodes.end());
      partStart.push_back(pts.empty() ? 0 : pts.size() - 1);
      for (size_t k = 0; k < nodes.size(); ++k) {
        if (!pts.empty() && k == 0) continue;
        pts.push_back(pos(nodes[k]));
        ids.push_back(nodes[k]);
      }
    }
    const Seg &first = segs[ch.parts.front().seg];
    m.highway = first.xs.highway;
    m.rank = classRank(m.highway);
    m.wayStart = first.way;
    m.wayEnd = segs[ch.parts.back().seg].way;
    PolylineFitOptions fo = opt.fit;
    // a corner tighter than the road's half-width would fold its outer lanes back on themselves
    double halfWidth = 0.0;
    for (const Part &p : ch.parts) {
      const CrossSection &x = segs[p.seg].xs;
      halfWidth = std::max({halfWidth, x.left.totalWidth() + std::abs(x.laneOffset),
                            x.right.totalWidth() + std::abs(x.laneOffset)});
    }
    fo.minCornerRadius = std::max({fo.minCornerRadius, minRadiusFor(m.highway), halfWidth + 3.0});
    fo.maxCurvature = 1.0 / (halfWidth + 1.5);  // lanes must never fold on the inside of a bend
    m.full = fitPolylineLimited(pts, fo);
    if (m.full.elements.empty() || m.full.length() < 0.5) continue;
    // s of every chain node (sections, signals, crossings are located by node)
    for (size_t k = 0; k < pts.size(); ++k) {
      double s, t;
      m.full.project(pts[k], s, t);
      fitDev.push_back(std::abs(t));
      if (k == 0) s = 0.0;
      if (k + 1 == pts.size()) s = m.full.length();
      m.nodeS.push_back({ids[k], s});
    }
    for (size_t pi = 0; pi < ch.parts.size(); ++pi) {
      const Part &p = ch.parts[pi];
      const Seg &sg = segs[p.seg];
      const double s0 = m.nodeS[partStart[pi]].second;
      const double s1 = pi + 1 < ch.parts.size() ? m.nodeS[partStart[pi + 1]].second : m.full.length();
      CrossSection xs = p.fwd ? sg.xs : reversedCrossSection(sg.xs);
      if (m.sections.empty() || !(m.sections.back().xs.sameTopology(xs)) ||
          std::abs(m.sections.back().xs.laneOffset - xs.laneOffset) > 0.05 ||
          m.sections.back().xs.right.speed != xs.right.speed || m.sections.back().xs.left.speed != xs.left.speed)
        m.sections.push_back({s0, xs});
      if (sg.bridge) m.bridges.push_back({s0, s1});
      const int layer = segLayer(sg);
      double maxHeight = 0.0;
      if (auto mh = tag(*sg.tags, "maxheight"))
        if (auto v = lengthTag(*mh)) maxHeight = *v;
      m.layers.push_back({s0, s1, layer, maxHeight});
      if (sg.tunnel) m.tunnels.push_back({s0, s1});
      if (sg.covered) m.covered.push_back({s0, s1});
      if (sg.passage) m.passages.push_back({s0, s1});
    }
    metas.push_back(m);
  }
}

void Builder::trimRoads() {
  // how far a road end may be pulled back: freeway ramps leave at shallow angles and overlap their
  // carriageway for much longer than street corners
  auto trimCap = [](const RoadMeta &m) { return m.rank >= 8 || m.highway.find("_link") != std::string::npos ? 160.0 : 80.0; };
  const bool crosswalksInJunctions = opt.crosswalksAtSignals && opt.target.spec().hasCrossPaths();
  // will road end e of cluster c get a corner sidewalk on both sides? As buildCornerSidewalks decides: arms
  // in the order of their outward headings; the corner between an arm and its anticlockwise neighbour joins
  // the arm's sidewalk right of traffic arriving along it to the neighbour's right of traffic leaving along
  // it, unless the turn is sharper than 120 degrees (a nose)
  auto cornersOnBothSides = [&](const Cluster &c, const RoadEnd &e) {
    auto end = [&](const RoadEnd &x) { return x.atEnd ? metas[x.road].sections.back().xs : metas[x.road].sections.front().xs; };
    auto outward = [&](const RoadEnd &x) {
      const ReferenceLine &l = metas[x.road].full;
      return x.atEnd ? normalizeAngle(l.eval(l.length()).hdg + kPi) : l.eval(0.0).hdg;
    };
    auto walk = [](const SideSpec &sd) { return !sd.lanes.empty() && sd.lanes.back().kind == LaneKind::Sidewalk; };
    // the sidewalk right of traffic arriving along x (leaving: the other side)
    auto walkRight = [&](const RoadEnd &x, bool arriving) {
      const CrossSection &xs = end(x);
      return walk((x.atEnd == arriving) ? xs.right : xs.left);
    };
    std::vector<const RoadEnd *> arms;
    for (const RoadEnd &x : c.ends) arms.push_back(&x);
    std::sort(arms.begin(), arms.end(), [&](const RoadEnd *a, const RoadEnd *b) { return outward(*a) < outward(*b); });
    const size_t n = arms.size();
    size_t i = 0;
    while (i < n && !(arms[i]->road == e.road && arms[i]->atEnd == e.atEnd)) ++i;
    if (i == n) return false;
    auto corner = [&](const RoadEnd &p, const RoadEnd &q) {  // arriving along p, leaving along q
      const double turn = normalizeAngle(outward(q) - normalizeAngle(outward(p) + kPi));
      return walkRight(p, true) && walkRight(q, false) && turn >= -2.0 * kPi / 3.0;
    };
    return corner(*arms[i], *arms[(i + 1) % n]) && corner(*arms[(i + n - 1) % n], *arms[i]);
  };
  // register road ends at clusters
  for (int i = 0; i < static_cast<int>(metas.size()); ++i) {
    const Chain &ch = chains[metas[i].chain];
    if (ch.c0 >= 0) clusters[ch.c0].ends.push_back({i, false});
    if (ch.c1 >= 0) clusters[ch.c1].ends.push_back({i, true});
  }
  // every road end belongs to one cluster and the rest is read only: clusters are trimmed in
  // parallel, with the same result as one after the other
  std::for_each(std::execution::par, clusters.begin(), clusters.end(), [&](Cluster &c) {
    if (c.ends.size() < 2) return;
    // the part of every road within reach of the junction (projection targets)
    std::vector<ReferenceLine> nearPart(c.ends.size());
    std::vector<double> nearStart(c.ends.size());
    for (size_t k = 0; k < c.ends.size(); ++k) {
      const RoadMeta &n = metas[c.ends[k].road];
      const double Ln = n.full.length(), reach = std::min(0.6 * Ln, trimCap(n)) + 2.0;
      nearStart[k] = c.ends[k].atEnd ? std::max(0.0, Ln - reach) : 0.0;
      nearPart[k] = n.full.slice(nearStart[k], c.ends[k].atEnd ? Ln : std::min(Ln, reach));
    }
    for (const RoadEnd &e : c.ends) {
      const RoadMeta &m = metas[e.road];
      const double L = m.full.length();
      const double maxD = std::min(0.6 * L, trimCap(m));
      double lastOverlap = -1.0;
      for (double d = 0.0; d <= maxD; d += 0.5) {
        const double s = e.atEnd ? L - d : d;
        const CrossSection &xs = sectionAtFull(m, s);
        double eL, eR;
        vehicleExtents(xs, eL, eR);
        const Pose p = m.full.eval(s);
        const Vec2 probes[3] = {p.offset(eL), p.offset(-eR), p.offset(0.5 * (eL - eR))};
        bool overlap = false;
        for (size_t fk = 0; fk < c.ends.size(); ++fk) {
          const RoadEnd &f = c.ends[fk];
          if (f.road == e.road) continue;  // not against itself (a loop road with both ends here)
          const RoadMeta &n = metas[f.road];
          const double Ln = n.full.length();
          for (const Vec2 &q : probes) {
            double sf, tf;
            nearPart[fk].project(q, sf, tf);
            sf += nearStart[fk];
            const double df = f.atEnd ? Ln - sf : sf;
            if (df < -0.5 || df > std::min(0.6 * Ln, trimCap(n))) continue;
            const Pose pf = n.full.eval(sf);
            if (std::abs((q - pf.pos()).dot(pf.dir())) > 0.5) continue;  // beyond the road end
            double fL, fR;
            vehicleExtents(sectionAtFull(n, sf), fL, fR);
            if (tf < fL + 0.3 && tf > -fR - 0.3) {
              overlap = true;
              break;
            }
          }
          if (overlap) break;
        }
        if (overlap) lastOverlap = d;
      }
      // roads end where the carriageways stop overlapping plus the curb return: a corner of radius R
      // starts about R before the point where the edges would meet
      const double R = opt.cornerRadius * (m.rank >= 7 ? 1.5 : 1.0);
      const double corner = c.ends.size() >= 3 ? 0.7 * R : 0.0;
      double trim = lastOverlap < 0 ? 0.5 : lastOverlap + opt.stopLineSetback + corner;
      // standard profile: the crosswalk of a signalised / crossing junction lies inside the junction (Junction
      // Guideline 1.0.0 §6.3.3 "Add roads for the pedestrian crossings inside of the junction area", §6.4.3
      // "Choose the road reference lines to stop before any cross path in the junction"): the approach ends
      // before it - where a corner sidewalk on each side will give the cross path walking lanes to link
      // (as buildCornerSidewalks makes them; cross_paths.cpp)
      // ... and where the approach keeps its lanes over the extra length (ending it past a change of the lane
      // count would leave the junction feeding only some of them)
      auto sameLanes = [&](double extra) {
        const double L0 = m.full.length(), d0 = std::min(trim, L0), d1 = std::min(trim + extra, L0);
        const CrossSection &a = sectionAtFull(m, e.atEnd ? L0 - d0 : d0), &b = sectionAtFull(m, e.atEnd ? L0 - d1 : d1);
        return a.left.drivingCount() == b.left.drivingCount() && a.right.drivingCount() == b.right.drivingCount();
      };
      if (crosswalksInJunctions && (c.signalised || c.crossing) && c.ends.size() >= 3 && cornersOnBothSides(c, e) &&
          sameLanes(kCrosswalkJunctionDepth)) {
        trim += kCrosswalkJunctionDepth;
        (e.atEnd ? metas[e.road].crosswalkInJunctionEnd : metas[e.road].crosswalkInJunctionStart) = true;
      }
      // beyond where a turn of the junction crossed the road's own lanes in an earlier build (armsCrossedByTurns)
      double extra = 0.0;
      if (!m.nodeS.empty())
        if (const auto it = extraTrim.find({m.wayStart, m.nodeS.front().first, m.nodeS.back().first, e.atEnd});
            it != extraTrim.end())
          extra = it->second;
      trim += extra;
      if (const auto &fixed = e.atEnd ? m.fixedTrimEnd : m.fixedTrimStart) trim = *fixed, extra = 0.0;
      (e.atEnd ? metas[e.road].trimEnd : metas[e.road].trimStart) = trim;
      (e.atEnd ? metas[e.road].extraTrimEnd : metas[e.road].extraTrimStart) = extra;
    }
    makeRoomForCurbReturns(c, trimCap);
  });
  for (RoadMeta &m : metas) {
    const double L = m.full.length();
    // the cut past the turns lying on a road never leaves less of it than kArmCutKeeps (nor less than it keeps without
    // that cut): it does not squeeze a short road between two junctions - where it would, it is pulled back
    if (const double extra = m.extraTrimStart + m.extraTrimEnd, over = m.trimStart + m.trimEnd - (L - kArmCutKeeps);
        extra > 0.0 && over > 0.0) {
      const double k = std::min(1.0, over / extra);  // the share of the cut pulled back
      m.trimStart -= k * m.extraTrimStart, m.trimEnd -= k * m.extraTrimEnd;
      m.extraTrimStart *= 1.0 - k, m.extraTrimEnd *= 1.0 - k;
    }
    const double keep = std::min(kKeptRoad, 0.5 * L);
    if (m.trimStart + m.trimEnd > L - keep) {
      const double k = (L - keep) / std::max(1e-6, m.trimStart + m.trimEnd);
      m.trimStart *= k;
      m.trimEnd *= k;
      rep.warnings.push_back("road on way " + std::to_string(m.wayStart) + " shortened to " +
                             std::to_string(keep) + " m by its junctions");
    }
  }
}

// ASAM Junction Guideline §6 (one junction per intersection, as small as possible but as big as
// necessary) and OpenDRIVE 1.9 Annex F junctions.no_overlap: when the arm cuts of two junctions
// leave less than a short road between them, and that road is no longer than a median crossing
// (the two halves of a dual-carriageway intersection), the two are one intersection.
bool Builder::joinSqueezedJunctions() {
  bool joined = false;
  for (const RoadMeta &m : metas) {
    const Chain &ch = chains[m.chain];
    if (ch.c0 < 0 || ch.c1 < 0 || ch.c0 == ch.c1) continue;
    if (m.full.length() - m.trimStart - m.trimEnd >= kMinRoadBetweenJunctions) continue;
    if (m.full.length() > kMaxAbsorbedRoad) continue;
    forcedJoins.push_back({ch.n0, ch.n1});
    joined = true;
  }
  return joined;
}

// Undo clustering, reference lines, gores and trims so the junction stages can run again.
void Builder::forgetJunctionLayout(const std::vector<Chain> &chainsBefore) {
  chains = chainsBefore;
  clusterOf.clear();
  clusters.clear();
  metas.clear();
  fitDev.clear();
}

namespace {

// Distance from the reference line (the OSM way) to the curb-side edge of a sidewalk that is the
// outermost lane on one side, or nullopt when that side does not end in a sidewalk.
std::optional<double> sidewalkCurbOffset(const CrossSection &x, bool leftSide) {
  const SideSpec &side = leftSide ? x.left : x.right;
  if (side.lanes.empty() || side.lanes.back().kind != LaneKind::Sidewalk) return std::nullopt;
  double t = x.laneOffset;
  for (size_t k = 0; k + 1 < side.lanes.size(); ++k) t += (leftSide ? 1.0 : -1.0) * side.lanes[k].width;
  return std::abs(t);
}

bool sameSide(const SideSpec &a, const SideSpec &b) {
  if (a.lanes.size() != b.lanes.size() || a.speed != b.speed || a.forward != b.forward) return false;
  for (size_t i = 0; i < a.lanes.size(); ++i)
    if (a.lanes[i].kind != b.lanes[i].kind || a.lanes[i].width != b.lanes[i].width || a.lanes[i].turns != b.lanes[i].turns)
      return false;
  return true;
}

// Re-placing the sections on the trimmed road (end sections stretched, tiny ones dropped) can leave
// two neighbouring sections equal by the rule that merges OSM segments into one section (one lane
// layout, laneOffset within 5 cm, the same speeds) and with the same widths. OpenDRIVE 1.8 §11.3
// needs a new lane section only where lanes change; such a boundary is removable (NAT-2) and the
// second section is dropped, the first continuing.
// A lane opening (or closing) over a taper splits off one neighbour of its kind: a lane added at the
// outer side from the lane inside it, a turn pocket at the inner side from the lane outside it. The
// line between the two (on the inner lane's outer edge, OpenDRIVE 1.9 §11.9 marks on the outer edge)
// is the index of the lane carrying it, or npos when the lane has no such neighbour.
size_t taperLineLane(const std::vector<odr::Lane> &side, size_t i) {
  if (side[i].type != "driving") return std::string::npos;
  if (i + 1 < side.size() && side[i + 1].type == "driving") return i;  // pocket: its own outer edge
  if (i > 0 && side[i - 1].type == "driving") return i - 1;            // added outside: the inner lane's edge
  return std::string::npos;
}

// OpenDRIVE 1.9 §11.9 road marks change along a lane with sOffset records (1.4: same). Over the given
// spans (sorted, within the section) the lane's single mark gives way to no mark: the line between a
// lane and the one opening beside it is drawn where the lane is open, not along the taper, where it
// would lie on the edge or center line it splits from.
void quietMarkOver(odr::Lane &l, std::vector<std::pair<double, double>> spans, double sectionLength) {
  if (l.roadMarks.size() != 1 || l.roadMarks.front().type == "none" || spans.empty()) return;
  const odr::RoadMark full = l.roadMarks.front();
  auto record = [&](odr::RoadMark m, double s) {
    m.sOffset = s;
    l.roadMarks.push_back(m);
  };
  std::sort(spans.begin(), spans.end());
  l.roadMarks.clear();
  double markedFrom = 0.0;  // start of the stretch still to be written with the full mark
  for (const auto &[a, b] : spans) {
    if (a > markedFrom + 1e-6) record(full, markedFrom);
    if (l.roadMarks.empty() || l.roadMarks.back().type != "none") record(odr::RoadMark{}, std::max(a, markedFrom));
    markedFrom = std::max(markedFrom, b);
  }
  if (markedFrom < sectionLength - 1e-6) record(full, markedFrom);
}

void mergeEqualSections(std::vector<SectionInfo> &secs) {
  auto equal = [](const CrossSection &a, const CrossSection &b) {
    return a.oneway == b.oneway && std::abs(a.laneOffset - b.laneOffset) <= 0.05 && a.roundabout == b.roundabout &&
           a.holdOffset == b.holdOffset && sameSide(a.left, b.left) && sameSide(a.right, b.right);
  };
  for (size_t k = 1; k < secs.size();)
    if (equal(secs[k - 1].xs, secs[k].xs)) secs.erase(secs.begin() + k);
    else ++k;
}

}  // namespace

// ASAM Junction Guideline §6: each arm is cut where its curb return starts. At a corner whose facing
// sides both end in a sidewalk the two curb lines meet at a corner point C; a curb return of radius
// R (the street's corner radius, and at least kSidewalkTurn sidewalk widths so the sidewalk turns
// round it without folding) starts R tan(phi / 2) before C on each curb, phi being the turn between
// them. The arm cut is moved back that far where it lies closer to the junction (never beyond the
// arm's trim cap).
void Builder::makeRoomForCurbReturns(const Cluster &c, const std::function<double(const RoadMeta &)> &trimCap) {
  constexpr double kSidewalkTurn = 2.5;
  struct Arm {
    const RoadEnd *end;
    Pose out;           ///< at the junction node, heading away from it
    CrossSection xs;    ///< cross section at the junction end
  };
  std::vector<Arm> arms;
  for (const RoadEnd &e : c.ends) {
    const RoadMeta &m = metas[e.road];
    const double L = m.full.length();
    Pose p = m.full.eval(e.atEnd ? L : 0.0);
    if (e.atEnd) p.hdg += kPi;
    arms.push_back({&e, p, sectionAtFull(m, e.atEnd ? L : 0.0)});
  }
  if (arms.size() < 2) return;
  std::sort(arms.begin(), arms.end(),
            [](const Arm &a, const Arm &b) { return normalizeAngle(a.out.hdg) < normalizeAngle(b.out.hdg); });
  for (size_t i = 0; i < arms.size(); ++i) {
    // the corner between arm P and its anticlockwise neighbour Q: left of P, right of Q (outward)
    const Arm &P = arms[i], &Q = arms[(i + 1) % arms.size()];
    if (P.end == Q.end) continue;
    const bool pLeftIsRoadLeft = !P.end->atEnd;  // outward along +s at the start of the road
    const bool qRightIsRoadLeft = Q.end->atEnd;
    const auto aP = sidewalkCurbOffset(P.xs, pLeftIsRoadLeft), aQ = sidewalkCurbOffset(Q.xs, qRightIsRoadLeft);
    if (!aP || !aQ) continue;
    const double turn = normalizeAngle(Q.out.hdg - (P.out.hdg + kPi));  // arriving along P, leaving along Q
    if (turn >= -1e-3) continue;  // the curb turns away from the sidewalk side: nothing to fold
    if (turn < -2.0 * kPi / 3.0) continue;  // an acute corner (a fork) ends in a nose, no curb return
    const Vec2 uP = P.out.dir(), uQ = Q.out.dir();
    const Vec2 curbP = P.out.offset(*aP), curbQ = Q.out.offset(-*aQ);
    const double den = uP.cross(uQ);
    if (std::abs(den) < 1e-6) continue;
    const Vec2 d = curbQ - curbP;
    const double sP = d.cross(uQ) / den, sQ = d.cross(uP) / den;  // corner point C along each curb
    const RoadMeta &mp = metas[P.end->road], &mq = metas[Q.end->road];
    const double wP = (pLeftIsRoadLeft ? P.xs.left : P.xs.right).lanes.back().width;
    const double wQ = (qRightIsRoadLeft ? Q.xs.left : Q.xs.right).lanes.back().width;
    const double rankFactor = std::max(mp.rank, mq.rank) >= 7 ? 1.5 : 1.0;
    const double R = std::max(opt.cornerRadius * rankFactor, kSidewalkTurn * std::max(wP, wQ));
    const double tangent = R * std::tan(0.5 * std::abs(turn));
    for (const auto &[arm, along] : {std::pair<const Arm *, double>{&P, sP}, {&Q, sQ}}) {
      RoadMeta &m = metas[arm->end->road];
      if ((arm->end->atEnd ? m.fixedTrimEnd : m.fixedTrimStart) || along < -1.0) continue;
      double &trim = arm->end->atEnd ? m.trimEnd : m.trimStart;
      // a curb return never eats the road it belongs to: at most 40 % of it per end
      const double cap = std::min(0.4 * m.full.length(), trimCap(m));
      trim = std::max(trim, std::min(cap, along + tangent));
    }
  }
}

void Builder::buildRoads() {
  const CountryProfile &C = opt.country;
  int nextRoadId = 1;
  for (RoadMeta &m : metas) {
    const double L = m.full.length();
    const double s0 = m.trimStart, s1 = L - m.trimEnd;
    odr::Road r;
    r.id = std::to_string(nextRoadId++);
    const Seg &firstSeg = segs[chains[m.chain].parts.front().seg];
    r.name = tag(*firstSeg.tags, "name").value_or("");
    if (C.leftHandTraffic) r.rule = "LHT";
    r.planView = m.full.slice(s0, s1);
    const double len = r.planView.length();
    r.userData["osmWays"] = [&] {
      std::string s;
      for (const Part &p : chains[m.chain].parts) s += (s.empty() ? "" : ",") + std::to_string(segs[p.seg].way);
      return s;
    }();
    // sections on the trimmed road
    std::vector<SectionInfo> secs;
    for (const SectionInfo &si : m.sections) {
      const double s = si.s - s0;
      if (s >= len - 0.5) continue;
      if (s <= 0.5) {
        if (secs.empty()) secs.push_back({0.0, si.xs});
        else secs.back().xs = si.xs;
        continue;
      }
      if (secs.empty()) secs.push_back({0.0, sectionAtFull(m, s0)});
      secs.push_back({s, si.xs});
    }
    if (secs.empty()) secs.push_back({0.0, sectionAtFull(m, s0)});
    // short roads keep one layout (the widest); their junction connections absorb the change
    if (secs.size() > 1 && len < 2.0 * opt.taperLength / 1.2) {
      SectionInfo widest = secs.back();
      for (const auto &si : secs)
        if (si.xs.drivingLanes() > widest.xs.drivingLanes()) widest = si;
      widest.s = 0.0;
      secs = {widest};
    }
    // OSM ways are usually split right at the intersection, so a turn-pocket section can be only a
    // few metres long after trimming: give end sections at least one taper length (where the road
    // allows) and merge tiny interior sections into their predecessor.
    if (secs.size() > 1) {
      const double want = std::min(opt.taperLength, 0.4 * len);
      SectionInfo &last = secs.back();
      if (len - last.s < want) last.s = std::max(secs[secs.size() - 2].s + 1.0, len - want);
      if (secs[1].s < want) secs[1].s = std::min(want, (secs.size() > 2 ? secs[2].s : len) - 1.0);
      for (size_t k = 1; k + 1 < secs.size();) {
        if (secs[k + 1].s - secs[k].s < 5.0) secs.erase(secs.begin() + k);
        else ++k;
      }
      // drop sections that collapsed onto their predecessor
      for (size_t k = 1; k < secs.size();)
        if (secs[k].s - secs[k - 1].s < 1.0) secs.erase(secs.begin() + k);
        else ++k;
    }
    mergeEqualSections(secs);
    remeasureSidewalks(m, r.planView, secs);
    keepTagParkingClearOfJunctions(m, len, secs);  // tagged parking: no stopping within the legal distance of junctions
    applyParkingBays(m, s0, len, secs);            // mapped street-side areas win where they lie
    for (size_t k = 0; k < secs.size(); ++k)
      keepVergeFoldSafe(secs[k].xs, r.planView, secs[k].s, k + 1 < secs.size() ? secs[k + 1].s : len);
    // lane sections, tapers and links
    for (size_t k = 0; k < secs.size(); ++k) r.lanes.push_back(makeLaneSection(secs[k].xs, secs[k].s, C));
    std::map<std::vector<odr::Lane> *, std::map<size_t, bool>> added, dropped;
    for (size_t k = 1; k < secs.size(); ++k) {
      odr::LaneSection &prev = r.lanes[k - 1], &cur = r.lanes[k];
      auto linkSide = [&](std::vector<odr::Lane> &a, std::vector<odr::Lane> &b, const SideSpec &sa,
                          const SideSpec &sb) {
        // match lanes by kind; driving lanes added / dropped at the inner side when the new inner
        // lane is a turn pocket toward the divider or the inner lane of a oneway ends merging away
        // from it (turn:lanes merge_to_right, LHT merge_to_left), else at the outer side
        const int na = sa.drivingCount(), nb = sb.drivingCount();
        const bool lhtSide = C.leftHandTraffic;
        auto innerTurn = [&](const SideSpec &s, std::initializer_list<const char *> turns) {
          if (s.lanes.empty()) return false;
          for (const auto &t : s.lanes.front().turns)
            for (const char *u : turns)
              if (t == u) return true;
          return false;
        };
        auto innerTurnPocket = [&](const SideSpec &s) {
          return lhtSide ? innerTurn(s, {"right", "slight_right"}) : innerTurn(s, {"left", "slight_left"});
        };
        auto innerMerge = [&](const SideSpec &s) {
          return secs[k - 1].xs.oneway && secs[k].xs.oneway && innerTurn(s, {lhtSide ? "merge_to_left" : "merge_to_right"});
        };
        const int shift = (nb > na && innerTurnPocket(sb))                   ? nb - na
                          : (na > nb && (innerTurnPocket(sa) || innerMerge(sa))) ? -(na - nb)
                                                                                 : 0;
        std::map<LaneKind, std::vector<size_t>> ia, ib;
        for (size_t i = 0; i < sa.lanes.size(); ++i) ia[sa.lanes[i].kind == LaneKind::Bus ? LaneKind::Driving : sa.lanes[i].kind].push_back(i);
        for (size_t i = 0; i < sb.lanes.size(); ++i) ib[sb.lanes[i].kind == LaneKind::Bus ? LaneKind::Driving : sb.lanes[i].kind].push_back(i);
        for (auto &[kind, va] : ia) {
          auto &vb = ib[kind];
          const int off = kind == LaneKind::Driving ? shift : 0;
          for (size_t j = 0; j < va.size(); ++j) {
            const long jb = static_cast<long>(j) + off;
            if (jb < 0 || jb >= static_cast<long>(vb.size())) {
              dropped[&a][va[j]] = true;  // ends at the section boundary: taper to zero
              continue;
            }
            a[va[j]].link.successor = b[vb[jb]].id;
            b[vb[jb]].link.predecessor = a[va[j]].id;
          }
        }
        for (auto &[kind, vb] : ib)
          for (size_t i : vb)
            if (!b[i].link.predecessor) added[&b][i] = true;  // starts at the boundary: grow from zero
      };
      linkSide(prev.left, cur.left, secs[k - 1].xs.left, secs[k].xs.left);
      linkSide(prev.right, cur.right, secs[k - 1].xs.right, secs[k].xs.right);
    }
    // width profiles: grow from / drop to zero over a taper; a lane that does both within one
    // section rises over the first half and falls over the second. A lane that continues from the
    // previous section with another width (the ways carry different width / lanes tags) blends to its
    // new width across the boundary, up to half a taper on either side (C1; the lane offset follows
    // the same window below): a step is a sideways jump of the lane border, and a blend after the
    // boundary would keep the old width a whole taper into the next way
    std::vector<double> blendB(r.lanes.size(), 0.0), blendA(r.lanes.size(), 0.0);  // window before / after boundary k
    // the taper of lanes ending before boundary k / starting after it: the lane offset moves in the
    // same window, so the lanes that flow on stay continuous
    std::vector<double> dropTaper(r.lanes.size() + 1, 0.0), addTaper(r.lanes.size() + 1, 0.0);
    for (size_t k = 1; k < r.lanes.size(); ++k) {
      blendB[k] = std::min(0.5 * opt.taperLength, 0.4 * (r.lanes[k].s - r.lanes[k - 1].s));
      blendA[k] = std::min(0.5 * opt.taperLength, 0.4 * ((k + 1 < r.lanes.size() ? r.lanes[k + 1].s : len) - r.lanes[k].s));
    }
    // value and slope at the boundary of a C1 blend from v0 to v1 over tb before and ta after it
    auto blendMid = [](double v0, double v1, double tb, double ta) {
      return std::pair<double, double>{v0 + (v1 - v0) * tb / (tb + ta), 1.5 * (v1 - v0) / (tb + ta)};
    };
    // per lane carrying a line beside a taper: its section's length and the taper spans
    std::map<std::pair<std::vector<odr::Lane> *, size_t>, std::pair<double, std::vector<std::pair<double, double>>>> quietSpans;
    for (size_t k = 0; k < r.lanes.size(); ++k) {
      const double secLen = (k + 1 < r.lanes.size() ? r.lanes[k + 1].s : len) - r.lanes[k].s;
      for (auto *side : {&r.lanes[k].left, &r.lanes[k].right})
        for (size_t i = 0; i < side->size(); ++i) {
          const bool g = added[side][i], d = dropped[side][i];
          odr::Lane &l = (*side)[i];
          const double w = l.widths.front().a;
          double ta = 0.0, wm = w, mm = 0.0;  // blend after the boundary; width and slope at the boundary
          odr::Lane *p = nullptr;
          if (!g && k > 0 && l.link.predecessor)
            for (auto *ps : {&r.lanes[k - 1].left, &r.lanes[k - 1].right})
              for (auto &q : *ps)
                if (q.id == *l.link.predecessor && !q.widths.empty()) p = &q;
          if (p) {
            const double prevLen = r.lanes[k].s - r.lanes[k - 1].s;
            const odr::Poly3 last = p->widths.back();
            const double w0 = last.eval(prevLen - last.s);
            if (std::abs(w0 - w) > 1e-3) {
              // the previous lane takes the first half where it ends at a constant width
              const bool held = last.b == 0.0 && last.c == 0.0 && last.d == 0.0;
              const double tb = held ? std::min(blendB[k], prevLen - last.s) : 0.0;
              ta = blendA[k];
              std::tie(wm, mm) = blendMid(w0, w, tb, ta);
              if (tb > 1e-6) {
                if (prevLen - tb <= last.s + 1e-6) p->widths.pop_back();
                p->widths.push_back(odr::Poly3::hermite(prevLen - tb, tb, w0, 0.0, wm, mm));
              }
            }
          }
          if (!g && ta <= 0.0 && !d) continue;
          const double cap = (d ? (g ? 0.5 : ta > 0.0 ? 0.6 : 0.8) : 0.8) * secLen;
          // growth / fall: the taper length, or the length measured for the lane (street-side parking bays)
          const LaneSpec &spec = (side == &r.lanes[k].left ? secs[k].xs.left : secs[k].xs.right).lanes[i];
          const double Tg = std::min(spec.taperIn > 0.0 ? spec.taperIn : opt.taperLength, cap);
          const double Td = std::min(spec.taperOut > 0.0 ? spec.taperOut : opt.taperLength, cap);
          l.widths.clear();
          double s0 = 0.0;  // start of the full-width stretch
          if (g) l.widths.push_back(odr::Poly3::hermite(0.0, Tg, 0.0, 0.0, w, 0.0)), s0 = Tg, addTaper[k] = std::max(addTaper[k], Tg);
          if (d) dropTaper[k + 1] = std::max(dropTaper[k + 1], Td);
          // a lane that blends to a new width at its start and ends at its section's end does both
          // (Td leaves room for the blend): the blend keeps its borders continuous across the boundary
          if (!g && ta > 0.0) l.widths.push_back(odr::Poly3::hermite(0.0, ta, wm, mm, w, 0.0)), s0 = ta;
          const double hold = secLen - s0 - (d ? Td : 0.0);
          if (hold > 1e-6) l.widths.push_back({s0, w, 0, 0, 0});
          if (d) l.widths.push_back(odr::Poly3::hermite(secLen - Td, Td, w, 0.0, 0.0, 0.0));
          if (const size_t line = taperLineLane(*side, i); line != std::string::npos) {
            auto &[sectionLength, spans] = quietSpans[{side, line}];
            sectionLength = secLen;
            if (g) spans.push_back({0.0, Tg});
            if (d) spans.push_back({secLen - Td, secLen});
          }
        }
    }
    for (auto &[at, q] : quietSpans) quietMarkOver((*at.first)[at.second], q.second, q.first);
    // lane offset: every section carries its own cross-section's offset, which places the OSM way where
    // OSM says it is (the middle of the carriageway, or the placement=* lane). It moves in the window of
    // the lanes that cause the change, so the lanes flowing on stay continuous: before the boundary
    // over the taper of a lane ending there, after it over the taper of a lane starting there, and in
    // the window of the width blend when only the widths change.
    std::vector<double> offs(secs.size());
    std::vector<bool> widthOnly(secs.size(), false), follow(secs.size(), false);
    offs[0] = secs[0].xs.laneOffset;
    for (size_t k = 1; k < secs.size(); ++k) {
      const CrossSection &a = secs[k - 1].xs, &b = secs[k].xs;
      const bool leftChanged = a.left.drivingCount() != b.left.drivingCount();
      const bool rightChanged = a.right.drivingCount() != b.right.drivingCount();
      offs[k] = b.laneOffset;
      // lanes only end here: the offset follows their taper before the boundary
      follow[k] = !a.holdOffset && !b.holdOffset && dropTaper[k] > 1e-6 && addTaper[k] <= 1e-6;
      // only the widths change
      widthOnly[k] = !a.holdOffset && !b.holdOffset && a.oneway == b.oneway && !leftChanged && !rightChanged &&
                     dropTaper[k] <= 1e-6 && addTaper[k] <= 1e-6;
    }
    r.laneOffset.push_back({0.0, offs[0], 0, 0, 0});
    // a transition from a level offset is a C2 shift (no curvature step on the lanes at its ends); one
    // that starts inside a running transition continues it with a Hermite blend from its slope
    auto shiftTo = [&](double s0, double L, double v, double m, double target) {
      if (std::abs(m) < 1e-9)
        for (const odr::Poly3 &p : odr::Poly3::shiftC2(s0, L, v, target)) r.laneOffset.push_back(p);
      else r.laneOffset.push_back(odr::Poly3::hermite(s0, L, v, m, target, 0.0));
    };
    for (size_t k = 1; k < secs.size(); ++k) {
      const double sb = secs[k].s, target = offs[k];
      // leaving a gore stretch (merge: main + branch lanes -> the trunk's own lanes): the lanes move
      // to the trunk's position over the whole stretch before the boundary, not in a short jog after it
      if (secs[k - 1].xs.holdOffset && !secs[k].xs.holdOffset && sb - secs[k - 1].s > 10.0) {
        const double s0 = secs[k - 1].s;
        while (r.laneOffset.size() > 1 && r.laneOffset.back().s > s0 + 1e-6) r.laneOffset.pop_back();
        const odr::Poly3 cur = r.laneOffset.back();
        const double v = cur.eval(s0 - cur.s), m = cur.deriv(s0 - cur.s);
        if (std::abs(v - target) < 1e-3) continue;
        if (cur.s >= s0 - 1e-6) r.laneOffset.pop_back();
        shiftTo(s0, sb - s0, v, m, target);
        if (sb < len - 1e-6) r.laneOffset.push_back({sb, target, 0, 0, 0});
        continue;
      }
      // only the widths change and the offset has settled: the width blend's window across the boundary
      if (const odr::Poly3 last = r.laneOffset.back();
          widthOnly[k] && last.s <= sb + 1e-6 && last.b == 0.0 && last.c == 0.0 && last.d == 0.0) {
        if (std::abs(last.a - target) < 1e-3) continue;
        const double tb = std::min(blendB[k], sb - last.s), ta = blendA[k];
        const auto [vm, mm] = blendMid(last.a, target, tb, ta);
        if (tb > 1e-6) {
          if (sb - tb <= last.s + 1e-6) r.laneOffset.pop_back();
          r.laneOffset.push_back(odr::Poly3::hermite(sb - tb, tb, last.a, 0.0, vm, mm));
        }
        r.laneOffset.push_back(odr::Poly3::hermite(sb, ta, vm, mm, target, 0.0));
        if (sb + ta < len - 1e-6) r.laneOffset.push_back({sb + ta, target, 0, 0, 0});
        continue;
      }
      // a lane ending at the inner side: lane 0 follows its taper before the boundary
      if (follow[k]) {
        const double s0 = sb - dropTaper[k];
        while (r.laneOffset.size() > 1 && r.laneOffset.back().s > s0 + 1e-6) r.laneOffset.pop_back();
        const odr::Poly3 cur = r.laneOffset.back();
        const double v = cur.eval(s0 - cur.s), m = cur.deriv(s0 - cur.s);
        if (cur.s >= s0 - 1e-6) r.laneOffset.pop_back();
        shiftTo(s0, sb - s0, v, m, target);
        if (sb < len - 1e-6) r.laneOffset.push_back({sb, target, 0, 0, 0});
        continue;
      }
      // current value / slope at the change point (a previous transition may still be running)
      while (r.laneOffset.size() > 1 && r.laneOffset.back().s > sb + 1e-6) r.laneOffset.pop_back();
      const odr::Poly3 cur = r.laneOffset.back();
      const double v = cur.eval(sb - cur.s), m = cur.deriv(sb - cur.s);
      if (std::abs(v - target) < 1e-3 && std::abs(m) < 1e-6) continue;
      if (cur.s >= sb - 1e-6) r.laneOffset.pop_back();  // replaced by the new transition
      // entering a gore stretch (diverge): over the whole stretch up to the gore
      const double room = (k + 1 < secs.size() ? secs[k + 1].s : len) - sb;
      const double T = secs[k].xs.holdOffset && room > 10.0 ? std::max(1.0, room - 1.0)
                       : addTaper[k] > 1e-6                 ? addTaper[k]
                                                            : std::min(opt.taperLength, std::max(1.0, len - sb - 1.0));
      shiftTo(sb, T, v, m, target);
      if (sb + T < len - 1e-6) r.laneOffset.push_back({sb + T, target, 0, 0, 0});
    }
    // road type + speed
    for (size_t k = 0; k < secs.size(); ++k) {
      odr::Road::Type t;
      t.s = secs[k].s;
      t.type = odrRoadType(secs[k].xs.highway);
      t.country = C.code == "XX" ? "" : C.code;
      const auto v = secs[k].xs.right.speed ? secs[k].xs.right.speed : secs[k].xs.left.speed;
      if (v) {
        t.speedMax = C.speedUnit == "mph" ? std::round(*v / 0.44704) : std::round(*v * 3.6);
        t.speedUnit = C.speedUnit == "mph" ? "mph" : "km/h";
      }
      if (r.types.empty() || r.types.back().type != t.type || r.types.back().speedMax != t.speedMax) r.types.push_back(t);
    }
    // bridges / tunnels
    auto structures = [&](const std::vector<std::pair<double, double>> &ranges, std::vector<odr::Structure> &out,
                          const char *kind) {
      for (const auto &[a, b] : ranges) {
        const double sa = std::max(0.0, a - s0), sb = std::min(len, b - s0);
        if (sb - sa < 1.0) continue;
        odr::Structure st;
        st.id = std::to_string(nextObjectId++);
        st.name = kind;
        st.s = sa;
        st.length = sb - sa;
        out.push_back(st);
      }
    };
    structures(m.bridges, r.bridges, "bridge");
    structures(m.tunnels, r.tunnels, "tunnel");
    m.roadIndex = static_cast<int>(od.roads.size());
    od.roads.push_back(std::move(r));
  }
}

}  // namespace detail

namespace {
/// Cut every way at the box boundary (Liang-Barsky); crossing points become new end nodes.
OsmData clipOsm(const OsmData &in, const Projection &proj, const std::array<double, 4> &box) {
  OsmData out;
  out.relations = in.relations;
  out.minLat = in.minLat, out.minLon = in.minLon, out.maxLat = in.maxLat, out.maxLon = in.maxLon;
  auto inside = [&](const Vec2 &p) { return p.x >= box[0] && p.x <= box[2] && p.y >= box[1] && p.y <= box[3]; };
  std::unordered_map<OsmId, Vec2> pos;
  for (const auto &[id, n] : in.nodes) {
    pos[id] = proj.forward(n.lat, n.lon);
    if (inside(pos[id])) out.nodes[id] = n;
  }
  OsmId synth = -1, wayId = -1;
  auto addNode = [&](const Vec2 &p) {
    OsmNode n;
    n.id = synth--;
    proj.inverse(p, n.lat, n.lon);
    out.nodes[n.id] = n;
    return n.id;
  };
  // a junction just outside the box (roads meeting a few metres beyond the cut) is kept whole: the
  // ways run on to it instead of ending side by side at the boundary
  std::unordered_map<OsmId, int> highwayUse;
  for (const auto &[id, w] : in.ways)
    if (tag(w.tags, "highway"))
      for (OsmId n : w.nodes) ++highwayUse[n];
  const double keepOutside = 30.0;
  auto keepNode = [&](OsmId n, const Vec2 &cut) {
    if (highwayUse[n] < 2 || (pos[n] - cut).norm() > keepOutside) return false;
    if (!out.nodes.count(n)) out.nodes[n] = *in.node(n);
    return true;
  };
  for (const auto &[id, w] : in.ways) {
    std::vector<std::vector<OsmId>> parts(1);
    for (size_t i = 0; i + 1 < w.nodes.size(); ++i) {
      auto ia = pos.find(w.nodes[i]), ib = pos.find(w.nodes[i + 1]);
      if (ia == pos.end() || ib == pos.end()) continue;
      const Vec2 a = ia->second, d = ib->second - a;
      double t0 = 0, t1 = 1;
      const double p[4] = {-d.x, d.x, -d.y, d.y}, q[4] = {a.x - box[0], box[2] - a.x, a.y - box[1], box[3] - a.y};
      bool visible = true;
      for (int k = 0; k < 4 && visible; ++k) {
        if (std::abs(p[k]) < 1e-12) visible = q[k] >= 0;
        else if (p[k] < 0) t0 = std::max(t0, q[k] / p[k]);
        else t1 = std::min(t1, q[k] / p[k]);
      }
      if (!visible || t0 > t1) {
        if (!parts.back().empty()) parts.emplace_back();
        continue;
      }
      const OsmId na = t0 <= 1e-12 || keepNode(w.nodes[i], a + d * t0) ? w.nodes[i] : addNode(a + d * t0);
      const OsmId nb = t1 >= 1 - 1e-12 || keepNode(w.nodes[i + 1], a + d * t1) ? w.nodes[i + 1] : addNode(a + d * t1);
      if (parts.back().empty() || parts.back().back() != na) {
        if (!parts.back().empty()) parts.emplace_back();
        parts.back().push_back(na);
      }
      parts.back().push_back(nb);
      if (nb != w.nodes[i + 1]) parts.emplace_back();  // left the box
    }
    bool first = true;
    for (auto &pt : parts) {
      if (pt.size() < 2) continue;
      OsmWay c = w;
      c.nodes = pt;
      if (!first) c.id = wayId--;  // further pieces of the same way
      first = false;
      out.ways[c.id] = c;
    }
  }
  return out;
}

/// OSM wiki Key:lanes: "The actual number of lanes should always be tagged" - an untagged count is a guess. An untagged
/// *_link piece that continues a tagged piece of the same class one to one (the node joins exactly these two highway
/// ways, one ending and the other starting there) takes that piece's lanes. Measured in the six test areas: 166 of 199
/// such pairs with both pieces tagged (83 %) have the same lane count, while the best single default is right for at most
/// 44 % of tagged one-way motorway_links (117 x 1, 111 x 2, 39 x 3 of 268). Repeated along chains; a piece between two
/// tagged pieces that disagree keeps the default. LA review flags 2 and 3: a ramp's untagged pieces got the motorway's 2
/// lanes beside its piece tagged lanes=1.
int inheritLinkLanes(OsmData &osm) {
  auto isLink = [](const OsmWay &w) {
    const auto hw = tag(w.tags, "highway");
    return hw && hw->size() > 5 && hw->compare(hw->size() - 5, 5, "_link") == 0;
  };
  std::unordered_map<OsmId, std::vector<std::pair<OsmId, bool>>> at;  // node -> (way, the way starts there)
  for (const auto &[id, w] : osm.ways)
    if (tag(w.tags, "highway") && w.nodes.size() >= 2) {
      at[w.nodes.front()].push_back({id, true});
      at[w.nodes.back()].push_back({id, false});
    }
  std::vector<OsmId> ids;  // way id order: the same result whatever the hash order
  for (const auto &[id, w] : osm.ways)
    if (isLink(w)) ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  int inherited = 0;
  for (bool changed = true; changed;) {
    changed = false;
    for (OsmId id : ids) {
      OsmWay &w = osm.ways.at(id);
      if (tag(w.tags, "lanes")) continue;
      std::optional<std::string> found;
      bool conflict = false;
      for (const bool atStart : {true, false}) {
        const auto it = at.find(atStart ? w.nodes.front() : w.nodes.back());
        if (it == at.end() || it->second.size() != 2) continue;  // a split or merge: no one-to-one continuation
        for (const auto &[oid, starts] : it->second) {
          // at w's first node the other piece must end there, at w's last node start there
          if (oid == id || starts == atStart) continue;
          const OsmWay &o = osm.ways.at(oid);
          if (tag(o.tags, "highway") != tag(w.tags, "highway")) continue;
          if (const auto lanes = tag(o.tags, "lanes")) {
            conflict |= found && *found != *lanes;
            found = lanes;
          }
        }
      }
      // an untagged branch of a split: the trunk's lanes whose turn:lanes bear toward the branch's side lead into it
      // (OSM wiki Key:turn:lanes, lanes listed left to right; LA review flag 1: a ramp tagged lanes=2,
      // turn:lanes=none|slight_right splits, the right-hand branch untagged - it takes the one right lane)
      if (!found && !conflict)
        if (const auto it = at.find(w.nodes.front()); it != at.end() && it->second.size() == 3) {
          const OsmWay *trunk = nullptr, *sibling = nullptr;
          for (const auto &[oid, starts] : it->second) {
            const OsmWay &o = osm.ways.at(oid);
            if (oid == id || tag(o.tags, "highway") != tag(w.tags, "highway")) continue;
            if (!starts) trunk = &o;
            else sibling = &o;
          }
          const auto turns = trunk ? tag(trunk->tags, "turn:lanes") : std::nullopt;
          if (trunk && sibling && turns && sibling->nodes.size() >= 2 && trunk->nodes.size() >= 2) {
            const OsmNode *n0 = osm.node(w.nodes.front()), *nw = osm.node(w.nodes[1]), *ns = osm.node(sibling->nodes[1]),
                          *nt = osm.node(trunk->nodes[trunk->nodes.size() - 2]);
            if (n0 && nw && ns && nt) {
              // side of this branch relative to its sibling, seen along the trunk (local east / north degrees)
              const double c = std::cos(n0->lat * kPi / 180.0);
              const Vec2 d{(n0->lon - nt->lon) * c, n0->lat - nt->lat}, vw{(nw->lon - n0->lon) * c, nw->lat - n0->lat},
                  vs{(ns->lon - n0->lon) * c, ns->lat - n0->lat};
              const bool right = d.cross(vw) < d.cross(vs);
              const std::vector<std::string> per = splitLanes(*turns);
              int count = 0;
              for (size_t k = 0; k < per.size(); ++k) {
                const std::string &lane = per[right ? per.size() - 1 - k : k];
                if (lane.find(right ? "right" : "left") == std::string::npos) break;
                ++count;
              }
              if (count > 0) found = std::to_string(count);
            }
          }
        }
      if (found && !conflict) {
        w.tags["lanes"] = *found;
        ++inherited;
        changed = true;
      }
    }
  }
  return inherited;
}

// a junction's arm end (a plain road linked to it) and how far along the arm, from that end, the farthest lane point of
// another turn of the junction lies on the arm's own lanes
using ArmKey = std::tuple<OsmId, OsmId, OsmId, bool>;  // as Builder::extraTrim

// The arm ends whose lanes a turn of their junction lies on: the lanes of the junction's connecting roads that are not
// linked to that end - their borders and centres every 0.25 m - projected onto the arm at 0.05 m < s < length - 0.05 m
// and more than 0.1 m inside its outermost lane borders. Arm ends trimRoads does not move are left out: those the gore
// stage fixed, those of a road with no more than kArmCutKeeps left (trimRoads cuts no further there) and those `kept`
// where they are (a cut there cost a turn of the junction).
// a road end as ArmKey: the road's first OSM way, first and last OSM node, at its end (nullopt: no such road)
std::optional<ArmKey> armKeyOf(const std::map<std::string, const detail::RoadMeta *> &metaOf,
                                const std::optional<odr::LinkTarget> &t) {
  if (!t || t->elementType != "road") return std::nullopt;
  const auto it = metaOf.find(t->elementId);
  if (it == metaOf.end()) return std::nullopt;
  const detail::RoadMeta &m = *it->second;
  return ArmKey{m.wayStart, m.nodeS.front().first, m.nodeS.back().first, t->contactPoint == "end"};
}

// The turns of every junction: (from arm end, to arm end) -> lane links of the connections between them, and each arm
// end's junction
struct JunctionTurns {
  std::map<std::pair<ArmKey, ArmKey>, size_t> links;
  std::map<ArmKey, std::string> junctionOf;
};
JunctionTurns junctionTurns(const detail::Builder &b) {
  std::map<std::string, const detail::RoadMeta *> metaOf;
  for (const detail::RoadMeta &m : b.metas)
    if (m.roadIndex >= 0 && !m.nodeS.empty()) metaOf[b.od.roads[m.roadIndex].id] = &m;
  std::map<std::string, const odr::Road *> roads;
  for (const odr::Road &r : b.od.roads) roads[r.id] = &r;
  JunctionTurns turns;
  for (const odr::Junction &j : b.od.junctions)
    for (const odr::Connection &c : j.connections) {
      const auto it = roads.find(c.connectingRoad);
      if (it == roads.end()) continue;
      const odr::Road &q = *it->second;
      const bool fromStart = c.contactPoint == "start";
      const auto from = armKeyOf(metaOf, fromStart ? q.predecessor : q.successor);
      const auto to = armKeyOf(metaOf, fromStart ? q.successor : q.predecessor);
      if (!from || !to) continue;
      turns.links[{*from, *to}] += c.laneLinks.size();
      turns.junctionOf[*from] = turns.junctionOf[*to] = j.id;
    }
  return turns;
}

std::map<ArmKey, double> armsCrossedByTurns(const detail::Builder &b, const std::set<ArmKey> &kept) {
  constexpr double kStep = 0.25, kEndGap = 0.05, kInside = 0.1;
  const odr::OpenDrive &od = b.od;
  std::map<std::string, std::vector<const odr::Road *>> turns;
  for (const odr::Road &r : od.roads)
    if (r.junction != "-1") turns[r.junction].push_back(&r);
  // the arm's outermost lane borders at s
  auto extent = [](const odr::Road &r, double s, double &lo, double &hi) {
    lo = hi = odr::laneOffsetAt(r, s);
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    for (const auto *side : {&sec.left, &sec.right})
      for (const odr::Lane &l : *side) {
        double a, c;
        if (odr::laneBorders(r, s, l.id, a, c)) lo = std::min({lo, a, c}), hi = std::max({hi, a, c});
      }
  };
  std::map<ArmKey, double> crossed;
  for (const detail::RoadMeta &m : b.metas) {
    if (m.roadIndex < 0 || m.nodeS.empty()) continue;
    if (m.full.length() - m.trimStart - m.trimEnd <= detail::kArmCutKeeps + 1e-6) continue;
    const odr::Road &arm = od.roads[m.roadIndex];
    const double L = arm.length();
    if (arm.junction != "-1" || arm.lanes.empty() || L <= 2.0 * kEndGap) continue;
    // a box around the arm's lanes: points outside it are not on them
    double x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
    for (double s = 0.0;; s = std::min(L, s + 1.0)) {
      double lo, hi;
      extent(arm, s, lo, hi);
      const Pose p = arm.planView.eval(s);
      for (const Vec2 &q : {p.offset(lo), p.offset(hi)})
        x0 = std::min(x0, q.x), y0 = std::min(y0, q.y), x1 = std::max(x1, q.x), y1 = std::max(y1, q.y);
      if (s >= L) break;
    }
    for (const bool atEnd : {false, true}) {
      const auto &lk = atEnd ? arm.successor : arm.predecessor;
      if (!lk || lk->elementType != "junction" || (atEnd ? m.fixedTrimEnd : m.fixedTrimStart)) continue;
      const auto it = turns.find(lk->elementId);
      if (it == turns.end()) continue;
      const std::string cp = atEnd ? "end" : "start";
      auto here = [&](const std::optional<odr::LinkTarget> &t) {
        return t && t->elementType == "road" && t->elementId == arm.id && t->contactPoint == cp;
      };
      double reach = -1.0;
      for (const odr::Road *q : it->second) {
        if (here(q->predecessor) || here(q->successor)) continue;
        const double Lq = q->length();
        for (double s = 0.0; s < Lq; s += kStep) {
          const double sq = std::clamp(s, 1e-6, Lq - 1e-6);
          const Pose p = q->planView.eval(sq);
          const odr::LaneSection &sec = odr::sectionAt(*q, sq);
          for (const auto *side : {&sec.left, &sec.right})
            for (const odr::Lane &l : *side) {
              double ti, to;
              if (!odr::laneBorders(*q, sq, l.id, ti, to)) continue;
              for (const double t : {ti, 0.5 * (ti + to), to}) {
                const Vec2 pt = p.offset(t);
                if (pt.x < x0 - kInside || pt.x > x1 + kInside || pt.y < y0 - kInside || pt.y > y1 + kInside) continue;
                double sa, ta, lo, hi;
                arm.planView.project(pt, sa, ta);
                if (sa <= kEndGap || sa >= L - kEndGap) continue;
                extent(arm, sa, lo, hi);
                if (ta > lo + kInside && ta < hi - kInside) reach = std::max(reach, atEnd ? L - sa : sa);
              }
            }
        }
      }
      if (const ArmKey key{m.wayStart, m.nodeS.front().first, m.nodeS.back().first, atEnd}; reach >= 0.0 && !kept.count(key))
        crossed[key] = reach;
    }
  }
  return crossed;
}
}  // namespace

odr::OpenDrive buildOpenDrive(const OsmData &osmIn, const Projection &proj, const BuildOptions &opt,
                              BuildReport *report) {
  OsmData clipped;
  if (opt.clipBox) clipped = clipOsm(osmIn, proj, *opt.clipBox);
  const OsmData &src = opt.clipBox ? clipped : osmIn;
  BuildOptions o = opt;  // topology preparation: lot aisles that connect streets become roads
  if (o.connectParkingLots)
    for (OsmId w : detail::throughParkingAisles(src, o)) o.includeWays.insert(w);
  int snapped = 0, crossings = 0;
  OsmData osm = detail::snapDanglingEnds(src, proj, o, snapped, &crossings);
  const int inheritedLanes = inheritLinkLanes(osm);
  const bool verbose = std::getenv("ODRFORGE_VERBOSE") != nullptr;
  std::unique_ptr<detail::Builder> bp;
  // ODRFORGE_VERBOSE: each stage with its run time (where a conversion spends its time)
  auto stage = [&](const char *name, void (detail::Builder::*fn)()) {
    const auto start = std::chrono::steady_clock::now();
    ((*bp).*fn)();
    if (verbose)
      std::fprintf(stderr, "[odrforge] %-28s %8.0f ms\n", name,
                   std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  };
  // the network up to its junctions; where a turn of a junction lies on the lanes of one of its arms (the arm cut
  // back for the arms' own overlaps only, not for the turns between the others), that arm end is cut back past it -
  // the farthest such point plus kArmClearance - and the network built again, the cuts growing up to kArmGrowth times.
  // No harm done: where a build has fewer lane links between two arm ends than the first, every arm end of that
  // junction (as the first build has it) keeps the first build's cut and the network is built again; out of builds
  // (kArmBuilds), every cut is undone - the first build's network again
  constexpr int kArmGrowth = 2, kArmBuilds = 5;
  constexpr double kArmClearance = 0.5;  // m
  std::map<ArmKey, double> extraTrim;
  std::set<ArmKey> kept;  // arm ends whose cut cost a turn of their junction
  JunctionTurns first;
  int rebuilds = 0, grown = 0, undone = 0;
  bool gaveUp = false;
  for (;; ++rebuilds) {
    bp = std::make_unique<detail::Builder>(osm, proj, o);
    detail::Builder &b = *bp;
    b.extraTrim = extraTrim;
    b.rep.inheritedLinkLanes = inheritedLanes;
    b.rep.snappedEnds = snapped;
    b.rep.joinedCrossings = crossings;
    b.rep.parkingAisles = static_cast<int>(o.includeWays.size() - opt.includeWays.size());
    stage("collect segments", &detail::Builder::collectSegments);
    if (o.separateSidewalks) stage("separate sidewalks", &detail::Builder::attachSeparateSidewalks);
    stage("junction nodes", &detail::Builder::findJunctionNodes);
    stage("chains", &detail::Builder::buildChains);
    // junction layout; a second pass joins junctions that squeezed the road between them out
    const std::vector<detail::Chain> chainsBefore = b.chains;
    const BuildReport repBefore = b.rep;
    for (int pass = 0; pass < 2; ++pass) {
      if (pass > 0) {
        b.forgetJunctionLayout(chainsBefore);
        b.rep = repBefore;
      }
      stage("cluster junctions", &detail::Builder::clusterJunctions);
      stage("reference lines", &detail::Builder::buildRoadGeometry);
      stage("lane-level gores", &detail::Builder::buildGores);
      stage("signal control", &detail::Builder::markSignalControl);  // before the trims: crosswalks in junctions
      stage("trim roads", &detail::Builder::trimRoads);
      if (pass > 0 || !b.joinSqueezedJunctions()) break;
    }
    stage("street-side parking", &detail::Builder::measureStreetSideParking);  // on the fitted, trimmed lines
    stage("roads", &detail::Builder::buildRoads);
    stage("resolve overlaps", &detail::Builder::resolveOverlaps);
    stage("elevation samples", &detail::Builder::sampleElevation);
    stage("junction surfaces", &detail::Builder::junctionSurfaces);
    stage("elevation profiles", &detail::Builder::fitRoadElevation);
    stage("resolve overlaps (heights)", &detail::Builder::resolveLevelOverlaps);
    stage("covered underpasses", &detail::Builder::coveredUnderpasses);  // on the final heights
    stage("junctions", &detail::Builder::buildJunctions);
    stage("direct junctions", &detail::Builder::directJunctions);
    if (gaveUp) break;
    if (rebuilds == 0) {
      first = junctionTurns(b);
    } else {
      // the junctions that lost lane links between two of their arm ends
      const JunctionTurns now = junctionTurns(b);
      std::set<std::string> lost;
      for (const auto &[pair, links] : first.links)
        if (const auto it = now.links.find(pair); it == now.links.end() || it->second < links)
          lost.insert(first.junctionOf.at(pair.first));
      if (verbose) std::fprintf(stderr, "[odrforge] junctions that lost a turn to the arm cuts: %zu\n", lost.size());
      if (!lost.empty()) {
        if (rebuilds + 1 >= kArmBuilds) {
          extraTrim.clear(), gaveUp = true;
        } else {
          for (const auto &[key, junction] : first.junctionOf)
            if (lost.count(junction)) undone += static_cast<int>(extraTrim.erase(key)), kept.insert(key);
        }
        continue;
      }
    }
    if (grown == kArmGrowth || rebuilds + 1 >= kArmBuilds) break;
    const std::map<ArmKey, double> crossed = armsCrossedByTurns(b, kept);
    if (verbose) std::fprintf(stderr, "[odrforge] arm ends a turn lies on: %zu\n", crossed.size());
    if (crossed.empty()) break;
    for (const auto &[key, reach] : crossed) extraTrim[key] += reach + kArmClearance;
    ++grown;
  }
  detail::Builder &b = *bp;
  b.rep.armsPulledBack = static_cast<int>(extraTrim.size());
  b.rep.armCutsUndone = gaveUp ? -1 : undone;
  b.rep.armRebuilds = rebuilds;
  stage("objects", &detail::Builder::buildObjects);  // crosswalks before the stop lines placed behind them
  stage("signals", &detail::Builder::buildSignals);
  stage("curb lanes", &detail::Builder::curbLanes);
  stage("cross paths", &detail::Builder::crossPaths);  // before the lowered curbs: it adds lowered spans
  stage("lowered curbs", &detail::Builder::lowerCurbsAtCrosswalks);
  stage("junction boundaries", &detail::Builder::junctionBoundaries);  // helper roads first: the grid covers them
  stage("junction grids", &detail::Builder::junctionGrids);
  stage("junctions under cover", &detail::Builder::junctionsUnderCover);  // every junction road exists now
  stage("barriers", &detail::Builder::roadBarriers);  // on the final lanes (curbs, lowered curbs)
  stage("finish", &detail::Builder::finish);
  if (verbose)
    std::fprintf(stderr, "[odrforge] segs %zu chains %zu clusters %zu roads %zu\n", b.segs.size(), b.chains.size(),
                 b.clusters.size(), b.metas.size());
  if (report) *report = b.rep;
  return std::move(b.od);
}

}  // namespace odrforge
