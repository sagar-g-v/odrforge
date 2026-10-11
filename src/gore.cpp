// OdrForge - lane-level gores.
//
// OSM draws a freeway exit as a ramp line leaving the freeway line at one node; the two carriageways
// then run side by side, overlapping, for tens to hundreds of metres before they separate at the
// gore nose. A junction at the OSM node would leave two road surfaces on top of each other.
//
// Here the split (merge) is modelled at lane level, as on the road:
//   * the freeway is one continuous reference line through the OSM node (refitted through both
//     ways' nodes, so heading and curvature stay continuous);
//   * the trunk road runs on up to the separation point S, where the branch carriageway clears the
//     main one, carrying the branch lanes beside the main lanes (added with a taper at the OSM node
//     for a diverge, dropped there for a merge) - spec 4.4 lane sections, 4.5 width;
//   * the junction is a few metres at S: main lanes continue 1:1, the branch lanes become the ramp.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

const double kJunctionLength = 4.0;  ///< length of the gore junction (m)

/// the lanes of a oneway cross section run along +s on this side
SideSpec *travelSide(CrossSection &x) { return x.right.drivingCount() > 0 ? &x.right : &x.left; }

/// branch driving lanes inserted beside the main lanes: on the outside of the carriageway, or at the
/// divider (then the divider moves so the main lanes keep their place)
CrossSection withBranchLanes(CrossSection x, const CrossSection &branch, bool branchLeft, int &added) {
  CrossSection b = branch;
  const SideSpec *bs = travelSide(b);
  SideSpec *side = travelSide(x);
  const bool onRight = side == &x.right;
  std::vector<LaneSpec> extra;
  double w = 0;
  for (const LaneSpec &l : bs->lanes)
    if (l.kind == LaneKind::Driving || l.kind == LaneKind::Bus) {
      LaneSpec e = l;
      e.kind = LaneKind::Driving;
      e.turns = {branchLeft ? "slight_left" : "slight_right"};
      extra.push_back(e);
      w += e.width;
    }
  added = static_cast<int>(extra.size());
  if (extra.empty()) return x;
  // the divider side is the driver's left for lanes on the right side
  const bool inner = onRight == branchLeft;
  auto &L = side->lanes;
  if (inner) {
    size_t at = 0;
    while (at < L.size() && L[at].kind != LaneKind::Driving && L[at].kind != LaneKind::Bus) ++at;
    L.insert(L.begin() + static_cast<long>(at), extra.begin(), extra.end());
    x.laneOffset += onRight ? w : -w;
  } else {
    size_t at = 0;
    for (size_t i = 0; i < L.size(); ++i)
      if (L[i].kind == LaneKind::Driving || L[i].kind == LaneKind::Bus) at = i + 1;
    L.insert(L.begin() + static_cast<long>(at), extra.begin(), extra.end());
  }
  x.holdOffset = true;
  return x;
}

/// s on `to` of the point at s on `from` (reference lines that coincide along the shared part)
double remap(const ReferenceLine &from, double s, const ReferenceLine &to) {
  double sc, tc;
  to.project(from.eval(s).pos(), sc, tc);
  return sc;
}

}  // namespace

void Builder::buildGores() {
  if (!opt.laneLevelGores) return;
  std::vector<std::vector<RoadEnd>> ends(clusters.size());
  for (int i = 0; i < static_cast<int>(metas.size()); ++i) {
    const Chain &ch = chains[metas[i].chain];
    if (ch.c0 >= 0) ends[ch.c0].push_back({i, false});
    if (ch.c1 >= 0) ends[ch.c1].push_back({i, true});
  }
  std::vector<char> used(metas.size(), 0);
  const bool dbg = std::getenv("ODRFORGE_DEBUG_GORE") != nullptr;
  auto why = [&](size_t ci, const char *reason) {
    if (dbg) std::fprintf(stderr, "gore? cluster at (%.0f, %.0f): %s\n", clusters[ci].center.x, clusters[ci].center.y, reason);
  };
  for (size_t ci = 0; ci < clusters.size(); ++ci) {
    const auto &E = ends[ci];
    if (E.size() != 3) continue;
    // three oneway roads, all travelling along +s (chains are oriented with the traffic)
    bool ok = true;
    int nIn = 0;
    for (const RoadEnd &e : E) {
      const RoadMeta &m = metas[e.road];
      const CrossSection &x = e.atEnd ? m.sections.back().xs : m.sections.front().xs;
      const bool okUsed = !used[e.road], okOneway = x.oneway && !x.roundabout && x.left.drivingCount() + x.right.drivingCount() > 0 &&
                                                       (x.left.drivingCount() == 0 || x.right.drivingCount() == 0);
      const bool okLen = m.full.length() > 20.0 && chains[m.chain].c0 != chains[m.chain].c1;
      if (dbg && !(okUsed && okOneway && okLen))
        std::fprintf(stderr, "gore? cluster at (%.0f, %.0f): road on way %lld used %d oneway %d length %.1f\n", clusters[ci].center.x,
                     clusters[ci].center.y, static_cast<long long>(m.wayStart), !okUsed, okOneway, m.full.length());
      ok = ok && okOneway && okLen;
      (void)okUsed;
      nIn += e.atEnd;
    }
    if (!ok || (nIn != 1 && nIn != 2)) {
      if (nIn == 1 || nIn == 2) why(ci, "not three oneway roads");
      continue;
    }
    const bool diverge = nIn == 1;
    int T = -1, br[2], nb = 0;
    for (const RoadEnd &e : E) {
      if (e.atEnd == diverge) T = e.road;
      else br[nb++] = e.road;
    }
    // travel headings at the node
    auto headingAt = [&](int mi, bool atEnd) {
      const ReferenceLine &l = metas[mi].full;
      return l.eval(atEnd ? l.length() : 0.0).hdg;
    };
    const double hT = headingAt(T, diverge);
    const double h0 = headingAt(br[0], !diverge), h1 = headingAt(br[1], !diverge);
    if (std::abs(normalizeAngle(h0 - h1)) > 30.0 * kPi / 180.0) {  // a real fork, not a gore
      why(ci, "branches diverge > 30 deg");
      continue;
    }
    const double d0 = std::abs(normalizeAngle(h0 - hT)), d1 = std::abs(normalizeAngle(h1 - hT));
    auto lanesOf = [&](int mi, bool atEnd) {
      const CrossSection &x = atEnd ? metas[mi].sections.back().xs : metas[mi].sections.front().xs;
      return x.drivingLanes();
    };
    int A = br[0], B = br[1];
    if (std::abs(d0 - d1) < 2.0 * kPi / 180.0 ? lanesOf(br[1], !diverge) > lanesOf(br[0], !diverge) : d1 < d0) std::swap(A, B);
    RoadMeta &mT = metas[T], &mA = metas[A], &mB = metas[B];
    // the freeway as one line through the node
    std::vector<Vec2> pts;
    // through each road's actual end points (an earlier gore may have moved the far end) and its
    // OSM nodes in between
    auto addNodes = [&](const RoadMeta &m, bool skipFirst) {
      const double L = m.full.length();
      if (!skipFirst) pts.push_back(m.full.eval(0.0).pos());
      for (const auto &[id, s] : m.nodeS)
        if (s > 0.5 && s < L - 0.5) pts.push_back(pos(id));
      pts.push_back(m.full.eval(L).pos());
    };
    if (diverge) addNodes(mT, false), addNodes(mA, true);
    else addNodes(mA, false), addNodes(mT, true);
    const CrossSection &xsA = diverge ? mA.sections.front().xs : mA.sections.back().xs;
    const CrossSection &xsB = diverge ? mB.sections.front().xs : mB.sections.back().xs;
    // driving-lane extents: the gore is where the driving lanes part (shoulders may still touch)
    auto drivingExtents = [](const CrossSection &x, double &left, double &right) {
      auto ext = [](const SideSpec &sd) {
        double w = 0;
        for (const auto &l : sd.lanes)
          if (l.kind == LaneKind::Driving || l.kind == LaneKind::Bus) w += l.width;
        return w;
      };
      left = x.laneOffset + ext(x.left);
      right = -x.laneOffset + ext(x.right);
    };
    double aL, aR, bL, bR;
    drivingExtents(xsA, aL, aR);
    drivingExtents(xsB, bL, bR);
    PolylineFitOptions fo = opt.fit;
    const double halfWidth = std::max(aL, aR) + bL + bR;
    fo.minCornerRadius = std::max({fo.minCornerRadius, classMinRadius(mA.highway), halfWidth + 3.0});
    fo.maxCurvature = 1.0 / (halfWidth + 1.5);
    const ReferenceLine C = fitPolylineLimited(pts, fo);
    if (C.elements.empty()) continue;
    const double Lc = C.length();
    // which side the branch leaves on, and where its carriageway clears the main one
    const double LB = mB.full.length();
    {
      double sc, tc;
      C.project(mB.full.eval(diverge ? std::min(LB, 30.0) : std::max(0.0, LB - 30.0)).pos(), sc, tc);
      (void)sc;
      if (std::abs(tc) < 1e-3) continue;
      const bool left = tc > 0;
      const double need = (left ? aL : aR) + (left ? bR : bL) + 0.3;
      double sB = -1;
      for (int k = 0; k <= 300; ++k) {
        const double s = diverge ? k : LB - k;
        if (s < 0 || s > LB) break;
        double sp, tp;
        C.project(mB.full.eval(s).pos(), sp, tp);
        if ((left ? tp : -tp) >= need) {
          sB = s;
          break;
        }
      }
      const double used0 = diverge ? sB : LB - sB;
      if (sB < 0 || used0 < 6.0 || (diverge ? LB - sB : sB) < 10.0) {  // separates at once, or never
        if (dbg) std::fprintf(stderr, "gore? cluster at (%.0f, %.0f): separation sB %.1f of %.1f\n", clusters[ci].center.x, clusters[ci].center.y, sB, LB);
        continue;
      }
      double sS, tS;
      C.project(mB.full.eval(sB).pos(), sS, tS);
      double sN, tN;
      C.project(pos(diverge ? chains[mT.chain].n1 : chains[mT.chain].n0), sN, tN);
      // trunk end / start and main start / end on the combined line
      const double tCut = diverge ? sS - kJunctionLength : sS + kJunctionLength;  // trunk side of the junction
      const double aCut = sS;                                                       // main side
      if (diverge ? (tCut < sN + 2.0 || Lc - aCut < 10.0) : (tCut > sN - 2.0 || aCut < 10.0)) {
        if (dbg) std::fprintf(stderr, "gore? cluster at (%.0f, %.0f): cut %.1f node %.1f main %.1f of %.1f\n", clusters[ci].center.x, clusters[ci].center.y, tCut, sN, aCut, Lc);
        continue;
      }
      int extra = 0;
      const CrossSection xsExt = withBranchLanes(xsA, xsB, left, extra);
      if (!extra) continue;
      // new trunk and main metas
      RoadMeta nT = mT, nA = mA;
      const ReferenceLine oldT = mT.full, oldA = mA.full;
      if (diverge) {
        nT.full = C.slice(0, tCut);
        nA.full = C.slice(aCut, Lc);
      } else {
        nA.full = C.slice(0, aCut);
        nT.full = C.slice(tCut, Lc);
      }
      const double offT = diverge ? 0.0 : tCut, offA = diverge ? aCut : 0.0;
      const double LT = nT.full.length(), LA = nA.full.length();
      auto clampS = [](double s, double L) { return std::clamp(s, 0.0, L); };
      // sections
      nT.sections.clear();
      nA.sections.clear();
      for (const SectionInfo &si : mT.sections) {
        const double s = remap(oldT, si.s, C) - offT;
        if (s < LT - 0.5) nT.sections.push_back({clampS(s, LT), si.xs});
      }
      const double sExt = diverge ? clampS(sN, LT) : 0.0;  // the branch lanes run on the trunk from here
      if (diverge) {
        nT.sections.push_back({sExt, xsExt});
      } else {
        // trunk from the junction to the OSM node: main + branch lanes, then its own sections
        std::vector<SectionInfo> own = nT.sections;
        nT.sections = {{0.0, xsExt}};
        for (SectionInfo si : own) {
          si.s = std::max(si.s, std::min(sN - offT, LT - 1.0));
          if (si.s > nT.sections.back().s + 0.5) nT.sections.push_back(si);
          else if (nT.sections.size() > 1) nT.sections.back().xs = si.xs;
        }
      }
      for (const SectionInfo &si : mA.sections) {
        const double s = remap(oldA, si.s, C) - offA;
        if (s >= LA - 0.5) continue;
        if (s <= 0.0) {
          if (nA.sections.empty()) nA.sections.push_back({0.0, si.xs});
          else nA.sections.back().xs = si.xs;
        } else {
          if (nA.sections.empty()) nA.sections.push_back({0.0, si.xs});
          nA.sections.push_back({s, si.xs});
        }
      }
      if (nA.sections.empty()) nA.sections.push_back({0.0, xsA});
      // nodes, structures and layers follow their positions on the new lines
      auto moveNodes = [&](const RoadMeta &src, const ReferenceLine &old) {
        for (const auto &[id, s] : src.nodeS) {
          const double sc = remap(old, s, C);
          if (sc >= offT - 1e-6 && sc <= offT + LT + 1e-6) nT.nodeS.push_back({id, sc - offT});
          if (sc >= offA - 1e-6 && sc <= offA + LA + 1e-6) nA.nodeS.push_back({id, sc - offA});
        }
      };
      nT.nodeS.clear(), nA.nodeS.clear();
      moveNodes(mT, oldT);
      moveNodes(mA, oldA);
      auto byS = [](const auto &a, const auto &b) { return a.second < b.second; };
      std::sort(nT.nodeS.begin(), nT.nodeS.end(), byS);
      std::sort(nA.nodeS.begin(), nA.nodeS.end(), byS);
      auto moveRanges = [&](auto member) {
        (nT.*member).clear(), (nA.*member).clear();
        for (const auto *src : {&mT, &mA}) {
          const ReferenceLine &old = src == &mT ? oldT : oldA;
          for (auto r : src->*member) {
            const double a = remap(old, r.first, C), b = remap(old, r.second, C);
            for (auto [dst, off, L] : {std::tuple{&nT, offT, LT}, std::tuple{&nA, offA, LA}}) {
              const double ra = std::max(a - off, 0.0), rb = std::min(b - off, L);
              if (rb - ra > 0.5) (dst->*member).push_back({ra, rb});
            }
          }
        }
      };
      moveRanges(&RoadMeta::bridges);
      moveRanges(&RoadMeta::tunnels);
      moveRanges(&RoadMeta::covered);
      nT.layers.clear(), nA.layers.clear();
      for (const auto *src : {&mT, &mA}) {
        const ReferenceLine &old = src == &mT ? oldT : oldA;
        for (const auto &lr : src->layers) {
          const double a = remap(old, lr.s0, C), b = remap(old, lr.s1, C);
          for (auto [dst, off, L] : {std::tuple{&nT, offT, LT}, std::tuple{&nA, offA, LA}}) {
            const double ra = std::max(a - off, 0.0), rb = std::min(b - off, L);
            if (rb - ra > 0.5) dst->layers.push_back({ra, rb, lr.layer});
          }
        }
      }
      // junction ends: not trimmed again (the junction is exactly the gore)
      (diverge ? nT.fixedTrimEnd : nT.fixedTrimStart) = 0.0;
      (diverge ? nA.fixedTrimStart : nA.fixedTrimEnd) = 0.0;
      (diverge ? mB.fixedTrimStart : mB.fixedTrimEnd) = diverge ? sB : LB - sB;
      mT = std::move(nT);
      mA = std::move(nA);
      Cluster &cl = clusters[ci];
      cl.center = C.eval(diverge ? sS - 0.5 * kJunctionLength : sS + 0.5 * kJunctionLength).pos();
      cl.gore = Cluster::Gore{T, A, B, diverge, left, extra};
      used[T] = used[A] = used[B] = 1;
      ++rep.gores;
    }
  }
}

}  // namespace detail
}  // namespace odrforge
