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

void Builder::collectSegments() {
  std::unordered_map<OsmId, int> usage;
  std::vector<const OsmWay *> ways;
  for (const auto &[id, w] : osm.ways) {
    const auto hw = tag(w.tags, "highway");
    if (!hw || !opt.highways.count(*hw) || w.nodes.size() < 2) continue;
    if (tagYes(w.tags, "area")) continue;
    if (*hw == "service")
      if (auto sv = tag(w.tags, "service"); sv && opt.excludeService.count(*sv)) continue;
    if (tagIs(w.tags, "access", "no") && !tag(w.tags, "bus") && !tag(w.tags, "psv")) continue;
    bool complete = true;
    for (OsmId n : w.nodes) complete = complete && osm.node(n);
    if (!complete) continue;
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
  for (Chain &c : chains) {
    if (!junctionNodes.count(c.n0) || !junctionNodes.count(c.n1)) continue;
    if (c.n0 == c.n1) continue;
    if (c.approxLength < radius[c.n0] + radius[c.n1] + opt.junctionJoinSlack) {
      unite(c.n0, c.n1);
      c.internal = true;
    }
  }
  // geometrically coincident junction nodes (mapping artefacts)
  std::vector<OsmId> jn(junctionNodes.begin(), junctionNodes.end());
  std::sort(jn.begin(), jn.end());
  for (size_t i = 0; i < jn.size(); ++i)
    for (size_t j = i + 1; j < jn.size(); ++j)
      if ((pos(jn[i]) - pos(jn[j])).norm() < 2.0) unite(jn[i], jn[j]);

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
    fo.minCornerRadius = std::max(fo.minCornerRadius, minRadiusFor(m.highway));
    m.full = fitPolyline(pts, fo);
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
      if (sg.tunnel) m.tunnels.push_back({s0, s1});
    }
    metas.push_back(m);
  }
}

void Builder::trimRoads() {
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
      const double Ln = n.full.length(), reach = std::min(0.6 * Ln, 80.0) + 2.0;
      nearStart[k] = c.ends[k].atEnd ? std::max(0.0, Ln - reach) : 0.0;
      nearPart[k] = n.full.slice(nearStart[k], c.ends[k].atEnd ? Ln : std::min(Ln, reach));
    }
    for (const RoadEnd &e : c.ends) {
      const RoadMeta &m = metas[e.road];
      const double L = m.full.length();
      const double maxD = std::min(0.6 * L, 80.0);
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
          if (f.road == e.road && f.atEnd == e.atEnd) continue;
          const RoadMeta &n = metas[f.road];
          const double Ln = n.full.length();
          for (const Vec2 &q : probes) {
            double sf, tf;
            nearPart[fk].project(q, sf, tf);
            sf += nearStart[fk];
            const double df = f.atEnd ? Ln - sf : sf;
            if (df < -0.5 || df > std::min(0.6 * Ln, 80.0)) continue;
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
      const double trim = lastOverlap < 0 ? 0.5 : lastOverlap + opt.stopLineSetback + corner;
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
      const bool recentre = room >= 10.0 &&
                            (a.oneway != b.oneway || (leftChanged && rightChanged) || (!leftChanged && !rightChanged));
      offs[k] = recentre ? b.laneOffset : offs[k - 1];
    }
    r.laneOffset.push_back({0.0, offs[0], 0, 0, 0});
    for (size_t k = 1; k < secs.size(); ++k) {
      const double sb = secs[k].s, target = offs[k];
      // current value / slope at the change point (a previous transition may still be running)
      while (r.laneOffset.size() > 1 && r.laneOffset.back().s > sb + 1e-6) r.laneOffset.pop_back();
      const odr::Poly3 cur = r.laneOffset.back();
      const double v = cur.eval(sb - cur.s), m = cur.deriv(sb - cur.s);
      if (std::abs(v - target) < 1e-3 && std::abs(m) < 1e-6) continue;
      if (cur.s >= sb - 1e-6) r.laneOffset.pop_back();  // replaced by the new transition
      const double T = std::min(opt.taperLength, std::max(1.0, len - sb - 1.0));
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

void Builder::buildElevation() {
  for (RoadMeta &m : metas) {
    if (m.roadIndex < 0) continue;
    odr::Road &r = od.roads[m.roadIndex];
    const double L = r.length();
    if (!opt.elevation) {
      r.elevation = {{0.0, 0.0, 0, 0, 0}};
      continue;
    }
    const double step = 5.0;
    const int n = std::max(2, static_cast<int>(std::ceil(L / step)) + 1);
    std::vector<double> s(n), z(n);
    for (int i = 0; i < n; ++i) {
      s[i] = L * i / (n - 1);
      const Pose p = r.planView.eval(s[i]);
      z[i] = opt.elevation(p.x, p.y);
    }
    // structures: the terrain under a bridge / above a tunnel is not the road surface
    auto bridgeOver = [&](const std::vector<std::pair<double, double>> &ranges) {
      for (const auto &[a, b] : ranges) {
        const double sa = a - m.trimStart, sb = b - m.trimStart;
        int i0 = -1, i1 = -1;
        for (int i = 0; i < n; ++i) {
          if (s[i] < sa) i0 = i;
          if (s[i] > sb && i1 < 0) i1 = i;
        }
        for (int i = std::max(0, i0 + 1); i < (i1 < 0 ? n : i1); ++i) z[i] = std::numeric_limits<double>::quiet_NaN();
      }
    };
    bridgeOver(m.bridges);
    bridgeOver(m.tunnels);
    // fill unknown samples by linear interpolation (flat extrapolation at the ends)
    int lastKnown = -1;
    for (int i = 0; i < n; ++i) {
      if (std::isnan(z[i])) continue;
      if (lastKnown < 0)
        for (int k = 0; k < i; ++k) z[k] = z[i];
      else
        for (int k = lastKnown + 1; k < i; ++k)
          z[k] = z[lastKnown] + (z[i] - z[lastKnown]) * (s[k] - s[lastKnown]) / (s[i] - s[lastKnown]);
      lastKnown = i;
    }
    if (lastKnown < 0) z.assign(n, 0.0);
    else
      for (int k = lastKnown + 1; k < n; ++k) z[k] = z[lastKnown];
    // smooth (removes DEM noise and building-edge artefacts) and fit C1 cubic pieces every ~20 m
    for (int pass = 0; pass < 3; ++pass) {
      std::vector<double> zz = z;
      for (int i = 1; i + 1 < n; ++i) zz[i] = 0.25 * z[i - 1] + 0.5 * z[i] + 0.25 * z[i + 1];
      z = zz;
    }
    const int knotEvery = std::max(1, static_cast<int>(20.0 / (L / (n - 1))));
    std::vector<int> knots;
    for (int i = 0; i < n; i += knotEvery) knots.push_back(i);
    if (knots.back() != n - 1) knots.push_back(n - 1);
    auto slope = [&](int i) {
      const int a = std::max(0, i - 1), b = std::min(n - 1, i + 1);
      return std::clamp((z[b] - z[a]) / std::max(1e-6, s[b] - s[a]), -0.15, 0.15);
    };
    r.elevation.clear();
    for (size_t k = 0; k + 1 < knots.size(); ++k) {
      const int a = knots[k], b = knots[k + 1];
      r.elevation.push_back(odr::Poly3::hermite(s[a], s[b] - s[a], z[a], slope(a), z[b], slope(b)));
    }
    if (r.elevation.empty()) r.elevation.push_back({0.0, z[0], 0, 0, 0});
  }
}

}  // namespace detail

odr::OpenDrive buildOpenDrive(const OsmData &osm, const Projection &proj, const BuildOptions &opt,
                              BuildReport *report) {
  detail::Builder b(osm, proj, opt);
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
  stage("trim roads", &detail::Builder::trimRoads);
  stage("roads", &detail::Builder::buildRoads);
  stage("elevation", &detail::Builder::buildElevation);
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
