#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <functional>
#include <limits>
#include <numeric>

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
      if (l.type != "driving") continue;
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
    for (const auto &[si, atStart] : nodeSegs[n]) {
      const Seg &sg = segs[si];
      int l = sg.bridge ? 1 : sg.tunnel ? -1 : 0;
      if (auto lv = tag(*sg.tags, "layer")) {
        try {
          l = std::stoi(*lv);
        } catch (...) {
        }
      }
      if (std::abs(l) > std::abs(layer)) layer = l;
    }
    return layer;
  };
  std::vector<OsmId> jn(junctionNodes.begin(), junctionNodes.end());
  std::sort(jn.begin(), jn.end());
  for (size_t i = 0; i < jn.size(); ++i)
    for (size_t j = i + 1; j < jn.size(); ++j)
      if ((pos(jn[i]) - pos(jn[j])).norm() < 2.0 && nodeLayer(jn[i]) == nodeLayer(jn[j]) && sameLevel(jn[i], jn[j], 2.0))
        unite(jn[i], jn[j]);

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
    // a short chain with both ends in the same cluster lies inside the junction
    if (c.c0 >= 0 && c.c0 == c.c1 && c.n0 != c.n1) c.internal = true;
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
      int layer = sg.bridge ? 1 : sg.tunnel ? -1 : 0;
      if (auto lv = tag(*sg.tags, "layer")) {
        try {
          layer = std::stoi(*lv);
        } catch (...) {
        }
      }
      m.layers.push_back({s0, s1, layer});
      if (sg.tunnel) m.tunnels.push_back({s0, s1});
      if (sg.covered) m.covered.push_back({s0, s1});
    }
    metas.push_back(m);
  }
}

void Builder::trimRoads() {
  // how far a road end may be pulled back: freeway ramps leave at shallow angles and overlap their
  // carriageway for much longer than street corners
  auto trimCap = [](const RoadMeta &m) { return m.rank >= 8 || m.highway.find("_link") != std::string::npos ? 160.0 : 80.0; };
  // register road ends at clusters
  for (int i = 0; i < static_cast<int>(metas.size()); ++i) {
    const Chain &ch = chains[metas[i].chain];
    if (ch.c0 >= 0) clusters[ch.c0].ends.push_back({i, false});
    if (ch.c1 >= 0) clusters[ch.c1].ends.push_back({i, true});
  }
  for (Cluster &c : clusters) {
    if (c.ends.size() < 2) continue;
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
      if (const auto &fixed = e.atEnd ? m.fixedTrimEnd : m.fixedTrimStart) trim = *fixed;
      (e.atEnd ? metas[e.road].trimEnd : metas[e.road].trimStart) = trim;
    }
  }
  for (RoadMeta &m : metas) {
    const double L = m.full.length();
    const double keep = std::min(2.0, 0.5 * L);
    if (m.trimStart + m.trimEnd > L - keep) {
      const double k = (L - keep) / std::max(1e-6, m.trimStart + m.trimEnd);
      m.trimStart *= k;
      m.trimEnd *= k;
      rep.warnings.push_back("road on way " + std::to_string(m.wayStart) + " shortened to " +
                             std::to_string(keep) + " m by its junctions");
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
    // lane sections, tapers and links
    for (size_t k = 0; k < secs.size(); ++k) r.lanes.push_back(makeLaneSection(secs[k].xs, secs[k].s, C));
    std::map<std::vector<odr::Lane> *, std::map<size_t, bool>> added, dropped;
    for (size_t k = 1; k < secs.size(); ++k) {
      odr::LaneSection &prev = r.lanes[k - 1], &cur = r.lanes[k];
      auto linkSide = [&](std::vector<odr::Lane> &a, std::vector<odr::Lane> &b, const SideSpec &sa,
                          const SideSpec &sb) {
        // match lanes by kind; driving lanes added / dropped at the inner side when the new inner
        // lane is a turn pocket toward the divider, else at the outer side
        const int na = sa.drivingCount(), nb = sb.drivingCount();
        const bool lhtSide = C.leftHandTraffic;
        auto innerTurnPocket = [&](const SideSpec &s) {
          if (s.lanes.empty()) return false;
          for (const auto &t : s.lanes.front().turns)
            if (t == (lhtSide ? "right" : "left") || t == (lhtSide ? "slight_right" : "slight_left")) return true;
          return false;
        };
        const int shift = (nb > na && innerTurnPocket(sb)) ? nb - na : (na > nb && innerTurnPocket(sa)) ? -(na - nb) : 0;
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
    // section rises over the first half and falls over the second
    for (size_t k = 0; k < r.lanes.size(); ++k) {
      const double secLen = (k + 1 < r.lanes.size() ? r.lanes[k + 1].s : len) - r.lanes[k].s;
      for (auto *side : {&r.lanes[k].left, &r.lanes[k].right})
        for (size_t i = 0; i < side->size(); ++i) {
          const bool g = added[side][i], d = dropped[side][i];
          if (!g && !d) continue;
          odr::Lane &l = (*side)[i];
          const double w = l.widths.front().a;
          const double T = std::min(opt.taperLength, (g && d ? 0.5 : 0.8) * secLen);
          l.widths.clear();
          if (g) l.widths.push_back(odr::Poly3::hermite(0.0, T, 0.0, 0.0, w, 0.0));
          const double hold = secLen - (g ? T : 0.0) - (d ? T : 0.0);  // full-width stretch
          if (hold > 1e-6) l.widths.push_back({g ? T : 0.0, w, 0, 0, 0});
          if (d) l.widths.push_back(odr::Poly3::hermite(secLen - T, T, w, 0.0, 0.0, 0.0));
        }
    }
    // lane offset: lanes added / dropped on one side widen the road outward and the existing lanes
    // keep running straight (mappers do not move the way for a turn pocket). The offset is only
    // re-centred when the carriageway really changes (oneway <-> two-way, both sides change), over the
    // full taper length.
    std::vector<double> offs(secs.size());
    offs[0] = secs[0].xs.laneOffset;
    for (size_t k = 1; k < secs.size(); ++k) {
      const CrossSection &a = secs[k - 1].xs, &b = secs[k].xs;
      const bool leftChanged = a.left.drivingCount() != b.left.drivingCount();
      const bool rightChanged = a.right.drivingCount() != b.right.drivingCount();
      const double room = (k + 1 < secs.size() ? secs[k + 1].s : len) - secs[k].s;
      const bool recentre = a.holdOffset || b.holdOffset ||
                            (room >= 10.0 && (a.oneway != b.oneway || (leftChanged && rightChanged) || (!leftChanged && !rightChanged)));
      offs[k] = recentre ? b.laneOffset : offs[k - 1];
    }
    r.laneOffset.push_back({0.0, offs[0], 0, 0, 0});
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
        r.laneOffset.push_back(odr::Poly3::hermite(s0, sb - s0, v, m, target, 0.0));
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
                                                            : std::min(opt.taperLength, std::max(1.0, len - sb - 1.0));
      r.laneOffset.push_back(odr::Poly3::hermite(sb, T, v, m, target, 0.0));
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
  const OsmData osm = detail::snapDanglingEnds(src, proj, o, snapped, &crossings);
  detail::Builder b(osm, proj, o);
  b.rep.snappedEnds = snapped;
  b.rep.joinedCrossings = crossings;
  b.rep.parkingAisles = static_cast<int>(o.includeWays.size() - opt.includeWays.size());
  const bool verbose = std::getenv("ODRFORGE_VERBOSE") != nullptr;
  auto stage = [&](const char *name, void (detail::Builder::*fn)()) {
    if (verbose) std::fprintf(stderr, "[odrforge] %s...\n", name);
    (b.*fn)();
  };
  stage("collect segments", &detail::Builder::collectSegments);
  stage("junction nodes", &detail::Builder::findJunctionNodes);
  stage("chains", &detail::Builder::buildChains);
  stage("cluster junctions", &detail::Builder::clusterJunctions);
  stage("reference lines", &detail::Builder::buildRoadGeometry);
  stage("lane-level gores", &detail::Builder::buildGores);
  stage("trim roads", &detail::Builder::trimRoads);
  stage("roads", &detail::Builder::buildRoads);
  stage("resolve overlaps", &detail::Builder::resolveOverlaps);
  stage("elevation samples", &detail::Builder::sampleElevation);
  stage("junction surfaces", &detail::Builder::junctionSurfaces);
  stage("elevation profiles", &detail::Builder::fitRoadElevation);
  stage("resolve overlaps (heights)", &detail::Builder::resolveLevelOverlaps);
  stage("junctions", &detail::Builder::buildJunctions);
  stage("signals", &detail::Builder::buildSignals);
  stage("objects", &detail::Builder::buildObjects);
  stage("finish", &detail::Builder::finish);
  if (verbose)
    std::fprintf(stderr, "[odrforge] segs %zu chains %zu clusters %zu roads %zu\n", b.segs.size(), b.chains.size(),
                 b.clusters.size(), b.metas.size());
  if (report) *report = b.rep;
  return std::move(b.od);
}

}  // namespace odrforge
