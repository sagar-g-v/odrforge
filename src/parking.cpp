// OdrForge - street parking: tagged parking lanes near junctions, and street-side parking mapped as its own area.
//
// Parking lanes from the OSM street-parking tags (lanes.cpp) run along a road's side; the tags do not say where along
// the way parking starts or stops, so the country's traffic law fills that in: no parking within its distance of a
// junction (rules/countries/*.json parkingClearOfJunction, e.g. DE StVO §12(3) Nr. 1: 5 m before and after
// intersections; GB Highway Code rule 243: 10 m). The parking strip is still there - the kerb does not move - but
// within that distance of a road end at a junction its lane carries the rule "no stopping at any time" (OpenDRIVE
// 1.8.0 §11.9 <rule>, a recommended value; 1.4H lane <rule>). A rule has no end of its own, so the stretch is a lane
// section of its own (decided by the user on 2026-10-18; the NAT-2 checks count a lane-rule change as a reason for a
// section). The distance runs from the trimmed road end (the curb-return start), a little beyond the crossing of the
// kerb lines the laws measure from. A road too short to keep both distances and a kMinBay stretch between them has
// nowhere to park at all (a stub between two junctions): it keeps no tagged parking lane.
//
// OSM maps parking beside the carriageway as an area of its own: amenity=parking with parking=street_side (bays
// beside the carriageway), lane (a strip of the carriageway) or layby (OSM wiki Key:parking), the road tagged
// parking:<side>=separate or not at all. OpenDRIVE has no free surface; street parking is a lane of type parking
// along the road (OpenDRIVE 1.8.0 §11.7.1: "parking: Lane with parking spaces."; 1.4H laneType parking), and a lane
// starts and ends at a lane section boundary (1.8.0 §11.3: "A new lane section shall be defined each time the
// number of lanes change."). Each area becomes a parking lane over the stretch of road it lies beside:
//
// measure: every kStep m along each road's fitted reference line (inside its trimmed ends), a ray along the normal
//   on each side meets the area at its inner and outer edge (rays starting inside the area - an area drawn over the
//   road - are not used). An area lies beside every road side whose rays meet it within the limits below (a way
//   split into several roads: each of them); a road farther away, its rays crossing another carriageway first,
//   or a side street meeting the area end-on fails them.
// width: the parking lane reaches from the carriageway edge (the divider plus the side's driving, bus, bike lanes)
//   to the area's outer edge. Where the area's inner edge lies more than kAbsorb beyond the carriageway edge, a
//   road-level border lane fills the strip between and the parking lane spans the area's inner to outer edge
//   (medians over the samples). A parking lane narrower than kMinWidth or wider than kMaxWidth, a strip wider than
//   kMaxStrip, or a stretch shorter than kMinBay is not this road's parking: left out, counted, warned.
// ends: the lane grows from (falls to) zero over the length the area takes to reach kDepthShare of its width.
//
// The mapped areas are measured geometry and win over the road's parking:* tags where both exist; the tags decide
// where no area is mapped (decided by the user on 2026-10-18): beside an area its measured lane replaces the parking
// lane the tags gave (counted), before and after it the tagged lane runs on.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <tuple>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
constexpr double kStep = 1.0;        // sample spacing along the road (m)
constexpr double kAbsorb = 0.5;      // an inner edge this close beyond the carriageway edge: mapping noise, no strip
constexpr double kMinWidth = 1.6;    // narrower: no room for a car
constexpr double kMaxWidth = 7.0;    // wider: a lot, not street parking (perpendicular stalls 5.2 m, rules.hpp)
constexpr double kMaxStrip = 4.0;    // a wider strip between carriageway and area: the area is not this road's
constexpr double kReach = 20.0;      // rays look this far beyond the carriageway edge
constexpr double kMinBay = 5.0;      // buildRoads merges sections shorter than this away
constexpr double kJoinGap = 5.0;     // bays on one side closer than this join (the gap would be such a section)
constexpr double kSnap = 2.0;        // bay ends this close to a section boundary or road end move onto it
constexpr double kMinTaper = 1.0;    // the steepest growth of a parking lane: its width over 1 m
constexpr double kDepthShare = 0.8;  // the taper ends where the area reaches this share of its width
// OpenDRIVE 1.8.0 §11.9 lane rule, one of its recommended values
constexpr const char *kNoStopping = "no stopping at any time";

struct Area {
  OsmId way = 0;
  std::vector<Vec2> ring;  // closed: front == back
  Vec2 lo, hi;             // bounding box
  Vec2 centroid;
  int layer = 0;
};

struct Sample {
  double s, inner, outer, edge;
};

double median(std::vector<double> v) {
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

/// distance from the divider to the outer edge of the side's carriageway (driving, bus, bike lanes from the
/// divider outwards), the divider lying at x.laneOffset (left +) from the reference line
double carriagewayEdge(const CrossSection &x, bool left) {
  const SideSpec &side = left ? x.left : x.right;
  double w = 0.0;
  for (const LaneSpec &l : side.lanes) {
    if (l.kind != LaneKind::Driving && l.kind != LaneKind::Bus && l.kind != LaneKind::Biking &&
        l.kind != LaneKind::Bidirectional)
      break;
    w += l.width;
  }
  return left ? x.laneOffset + w : w - x.laneOffset;
}

/// where the ray p + d u, u in [0, len], crosses the ring's edges: the nearest and farthest u; false when it
/// misses, or starts inside the area (an odd number of crossings)
bool rayThrough(const Area &a, Vec2 p, Vec2 d, double len, double &inner, double &outer) {
  int n = 0;
  inner = 1e18, outer = -1e18;
  for (size_t i = 0; i + 1 < a.ring.size(); ++i) {
    const Vec2 e = a.ring[i + 1] - a.ring[i];
    const double den = d.cross(e);
    if (std::abs(den) < 1e-12) continue;
    const Vec2 w = a.ring[i] - p;
    const double u = w.cross(e) / den, v = w.cross(d) / den;
    if (v < 0.0 || v >= 1.0 || u < 0.0 || u > len) continue;
    ++n, inner = std::min(inner, u), outer = std::max(outer, u);
  }
  return n >= 2 && n % 2 == 0;
}

int layerOf(const Tags &t) {
  if (auto l = tag(t, "layer")) try {
      return std::stoi(*l);
    } catch (...) {
    }
  return 0;
}
/// append a piece of road [s, ...) with cross-section x; a piece equal to the one before (by the rule of buildRoads'
/// append a piece of road [s, ...) with cross-section x; a piece equal to the one before (by the rule of buildRoads'
/// mergeEqualSections) continues it, taking over its end tapers
void appendPiece(std::vector<SectionInfo> &out, double s, const CrossSection &x) {
  if (!out.empty()) {
    const CrossSection &p = out.back().xs;
    auto sameSide = [](const SideSpec &u, const SideSpec &w) {
      if (u.lanes.size() != w.lanes.size() || u.speed != w.speed || u.forward != w.forward) return false;
      for (size_t i = 0; i < u.lanes.size(); ++i)
        if (u.lanes[i].kind != w.lanes[i].kind || u.lanes[i].width != w.lanes[i].width || u.lanes[i].turns != w.lanes[i].turns ||
            u.lanes[i].rule != w.lanes[i].rule)
          return false;
      return true;
    };
    if (p.oneway == x.oneway && std::abs(p.laneOffset - x.laneOffset) <= 0.05 && p.roundabout == x.roundabout &&
        p.holdOffset == x.holdOffset && sameSide(p.left, x.left) && sameSide(p.right, x.right)) {
      for (const bool left : {true, false}) {
        auto &u = (left ? out.back().xs.left : out.back().xs.right).lanes;
        const auto &w = (left ? x.left : x.right).lanes;
        for (size_t i = 0; i < u.size(); ++i) u[i].taperOut = w[i].taperOut;
      }
      return;
    }
  }
  out.push_back({s, x});
}
}  // namespace

void Builder::measureStreetSideParking() {
  std::vector<Area> areas;
  for (const auto &[id, w] : osm.ways) {
    const auto kind = tag(w.tags, "parking");
    if (!tagIs(w.tags, "amenity", "parking") || !kind || (*kind != "street_side" && *kind != "lane" && *kind != "layby"))
      continue;
    if (w.nodes.size() < 4 || w.nodes.front() != w.nodes.back()) continue;
    Area a;
    a.way = id;
    a.layer = layerOf(w.tags);
    a.lo = {1e18, 1e18}, a.hi = {-1e18, -1e18};
    for (OsmId n : w.nodes) {
      if (!osm.node(n)) break;
      const Vec2 p = pos(n);
      a.ring.push_back(p);
      a.lo = {std::min(a.lo.x, p.x), std::min(a.lo.y, p.y)}, a.hi = {std::max(a.hi.x, p.x), std::max(a.hi.y, p.y)};
    }
    if (a.ring.size() != w.nodes.size()) continue;
    double A = 0.0;  // shoelace: signed area and centroid
    for (size_t i = 0; i + 1 < a.ring.size(); ++i) {
      const double c = a.ring[i].cross(a.ring[i + 1]);
      A += 0.5 * c;
      a.centroid = a.centroid + (a.ring[i] + a.ring[i + 1]) * (c / 6.0);
    }
    if (std::abs(A) < 1.0) continue;  // degenerate
    a.centroid = a.centroid * (1.0 / A);
    areas.push_back(a);
  }
  if (areas.empty()) return;
  // 1. samples per (area, road, side)
  std::map<std::tuple<size_t, size_t, bool>, std::vector<Sample>> hits;
  for (size_t mi = 0; mi < metas.size(); ++mi) {
    const RoadMeta &m = metas[mi];
    const double L = m.full.length(), s0 = m.trimStart, s1 = L - m.trimEnd;
    if (s1 - s0 < kMinBay) continue;
    // the areas near this road at all
    Vec2 lo{1e18, 1e18}, hi{-1e18, -1e18};
    for (double s = s0; s <= s1 + 1e-9; s += std::max(kStep, (s1 - s0) / 50.0)) {
      const Pose e = m.full.eval(std::min(s, s1));
      lo = {std::min(lo.x, e.x), std::min(lo.y, e.y)}, hi = {std::max(hi.x, e.x), std::max(hi.y, e.y)};
    }
    const double margin = 40.0;
    std::vector<size_t> near;
    for (size_t ai = 0; ai < areas.size(); ++ai)
      if (areas[ai].hi.x >= lo.x - margin && areas[ai].lo.x <= hi.x + margin && areas[ai].hi.y >= lo.y - margin &&
          areas[ai].lo.y <= hi.y + margin)
        near.push_back(ai);
    if (near.empty()) continue;
    for (double s = s0 + 0.5 * kStep; s < s1; s += kStep) {
      int layer = 0;
      for (const auto &lr : m.layers)
        if (s >= lr.s0 && s <= lr.s1) layer = lr.layer;
      const Pose e = m.full.eval(s);
      const Vec2 p{e.x, e.y}, nrm{-std::sin(e.hdg), std::cos(e.hdg)};  // left normal
      const CrossSection &xs = sectionAtFull(m, s);
      for (const bool left : {true, false}) {
        const double edge = carriagewayEdge(xs, left), len = std::max(0.0, edge) + kReach;
        const Vec2 d = left ? nrm : nrm * -1.0;
        const Vec2 q = p + d * len;
        for (size_t ai : near) {
          const Area &a = areas[ai];
          if (a.layer != layer || std::max(p.x, q.x) < a.lo.x || std::min(p.x, q.x) > a.hi.x ||
              std::max(p.y, q.y) < a.lo.y || std::min(p.y, q.y) > a.hi.y)
            continue;
          double in, out;
          if (rayThrough(a, p, d, len, in, out)) hits[{ai, mi, left}].push_back({s, in, out, edge});
        }
      }
    }
  }
  // 2. per road side meeting an area: its lane from the medians. An area beside a way split into several roads
  //    lies beside each of them; a road farther away (its rays crossing another carriageway first) or crossing
  //    the area end-on (a side street) fails the strip, width or length limits.
  std::map<std::pair<size_t, bool>, std::vector<RoadMeta::ParkingBay>> bays;
  std::vector<int> drawnOn(areas.size(), 0);
  // why an area left out was: the reason from the road it lies nearest (the smallest gap from that road's carriageway
  // edge to the area's inner edge) - a road farther off may meet it over more samples and fail for its distance
  std::vector<std::pair<double, std::string>> why(areas.size(), {std::numeric_limits<double>::infinity(), "no road alongside"});
  // the area's stretches beside the road: runs of consecutive samples (an area drawn over the road is met only at
  // its slanted ends, each a run of its own)
  std::vector<std::tuple<size_t, size_t, bool, std::vector<Sample>>> runs;
  for (const auto &[key, v] : hits) {
    const auto &[ai, mi, left] = key;
    for (size_t i = 0; i < v.size(); ++i) {
      if (i == 0 || v[i].s - v[i - 1].s > 1.5 * kStep) runs.push_back({ai, mi, left, {}});
      std::get<3>(runs.back()).push_back(v[i]);
    }
  }
  // nearest first: the road side with the narrowest strip to the area claims it; a farther road's stretch whose
  // rays meet the area mostly where a nearer road's accepted stretch lies beside it is that road's parking, not
  // its own (an area between a service road and a street is drawn once, the lanes of the two never overlap)
  auto stripOf = [](const std::vector<Sample> &v) {
    std::vector<double> g;
    for (const Sample &q : v) g.push_back(q.inner - q.edge);
    return std::abs(median(g));
  };
  std::stable_sort(runs.begin(), runs.end(), [&](const auto &a, const auto &b) { return stripOf(std::get<3>(a)) < stripOf(std::get<3>(b)); });
  struct Claim {
    size_t ai, mi;
    bool left;
    double s0, s1;
  };
  std::vector<Claim> claims;
  auto claimedElsewhere = [&](size_t ai, size_t mi, bool left, const std::vector<Sample> &v) {
    int claimed = 0;
    for (const Sample &q : v) {
      const Pose e = metas[mi].full.eval(q.s);
      const Vec2 d = Vec2{-std::sin(e.hdg), std::cos(e.hdg)} * (left ? 1.0 : -1.0);
      const Vec2 mid = Vec2{e.x, e.y} + d * (0.5 * (q.inner + q.outer));  // the middle of the area on this ray
      for (const Claim &c : claims) {
        if (c.ai != ai || (c.mi == mi && c.left == left)) continue;
        double s, t;
        metas[c.mi].full.project(mid, s, t);
        if (s > c.s0 + 0.5 && s < c.s1 - 0.5 && (t > 0) == c.left) {
          ++claimed;
          break;
        }
      }
    }
    return 2 * claimed >= static_cast<int>(v.size());
  };
  for (const auto &[ai, mi, left, v] : runs) {
    std::vector<double> strip, toOuter, depth;
    for (const Sample &q : v) strip.push_back(q.inner - q.edge), toOuter.push_back(q.outer - q.edge), depth.push_back(q.outer - q.inner);
    RoadMeta::ParkingBay b;
    b.left = left;
    b.s0 = v.front().s - 0.5 * kStep, b.s1 = v.back().s + 0.5 * kStep;
    b.filler = median(strip) > kAbsorb ? median(strip) : 0.0;
    b.width = b.filler > 0.0 ? median(depth) : median(toOuter);
    std::string fails;
    // street-side: the area's centroid lies beyond the carriageway edge (an area drawn over the road does not)
    double sc, tc;
    const RoadMeta &m = metas[mi];
    m.full.project(areas[ai].centroid, sc, tc);
    if (sc > 0.0 && sc < m.full.length() && (left ? tc : -tc) <= carriagewayEdge(sectionAtFull(m, sc), left))
      fails = "drawn over the road";
    else if (b.filler > kMaxStrip) fails = "lies " + std::to_string(b.filler) + " m beyond the carriageway";
    else if (b.width < kMinWidth || b.width > kMaxWidth) fails = "would be a parking lane " + std::to_string(b.width) + " m wide";
    else if (b.s1 - b.s0 < kMinBay) fails = "shorter than " + std::to_string(kMinBay) + " m along the road";
    if (!fails.empty()) {
      if (median(strip) < why[ai].first) why[ai] = {median(strip), fails};
      continue;
    }
    if (claimedElsewhere(ai, mi, left, v)) continue;
    claims.push_back({ai, mi, left, b.s0, b.s1});
    // ends: where the area's depth beyond the carriageway (and strip) first / last reaches kDepthShare of the width
    auto reached = [&](const Sample &q) { return q.outer - q.edge - b.filler >= kDepthShare * b.width; };
    const auto first = std::find_if(v.begin(), v.end(), reached);
    const auto last = std::find_if(v.rbegin(), v.rend(), reached);
    const double cap = 0.45 * (b.s1 - b.s0);
    b.taperIn = std::clamp(first == v.end() ? cap : first->s - b.s0, kMinTaper, cap);
    b.taperOut = std::clamp(last == v.rend() ? cap : b.s1 - last->s, kMinTaper, cap);
    b.ways = {areas[ai].way};
    ++drawnOn[ai];
    bays[{mi, left}].push_back(b);
  }
  for (size_t ai = 0; ai < areas.size(); ++ai) {
    parkingAreas.insert(areas[ai].way);
    if (!drawnOn[ai])
      rep.warnings.push_back("street-side parking area on way " + std::to_string(areas[ai].way) + " left out: " + why[ai].second);
  }
  rep.streetSideParkingLeftOut = static_cast<int>(parkingAreas.size());  // until drawn (applyParkingBays)
  // 3. bays on one road side closer than kJoinGap join (widths weighted by length)
  for (auto &[at, v] : bays) {
    std::sort(v.begin(), v.end(), [](const auto &a, const auto &b) { return a.s0 < b.s0; });
    std::vector<RoadMeta::ParkingBay> joined;
    for (const auto &b : v) {
      if (!joined.empty() && b.s0 - joined.back().s1 < kJoinGap) {
        auto &j = joined.back();
        const double lj = j.s1 - j.s0, lb = b.s1 - b.s0;
        j.width = (j.width * lj + b.width * lb) / (lj + lb);
        j.filler = (j.filler * lj + b.filler * lb) / (lj + lb);
        if (b.s1 > j.s1) j.s1 = b.s1, j.taperOut = b.taperOut;
        j.ways.insert(j.ways.end(), b.ways.begin(), b.ways.end());
      } else {
        joined.push_back(b);
      }
    }
    for (auto &b : joined) metas[at.first].bays.push_back(b);
  }
}

void Builder::applyParkingBays(const RoadMeta &m, double sTrim, double len, std::vector<SectionInfo> &secs) {
  if (m.bays.empty()) return;
  // bays on the trimmed road, their ends on section boundaries / road ends within kSnap
  std::vector<RoadMeta::ParkingBay> bays;
  for (RoadMeta::ParkingBay b : m.bays) {
    b.s0 = std::max(0.0, b.s0 - sTrim), b.s1 = std::min(len, b.s1 - sTrim);
    for (double *e : {&b.s0, &b.s1}) {
      if (*e < kSnap) *e = 0.0;
      if (*e > len - kSnap) *e = len;
      for (const SectionInfo &si : secs)
        if (std::abs(*e - si.s) < kSnap) *e = si.s;
    }
    if (b.s1 - b.s0 < kMinBay) {
      rep.warnings.push_back("street-side parking area on way " + std::to_string(b.ways.front()) +
                             ": a stretch lies within the junction of the road on way " + std::to_string(m.wayStart));
      continue;
    }
    parkingAreasDrawn.insert(b.ways.begin(), b.ways.end());
    if (std::getenv("ODRFORGE_VERBOSE"))  // where each area went
      std::fprintf(stderr, "[odrforge] parking bay of way %lld on the road of way %lld, %s, s %.1f-%.1f, width %.2f, strip %.2f, "
                   "tapers %.1f / %.1f\n", static_cast<long long>(b.ways.front()), static_cast<long long>(m.wayStart),
                   b.left ? "left" : "right", b.s0, b.s1, b.width, b.filler, b.taperIn, b.taperOut);
    bays.push_back(b);
  }
  rep.streetSideParking = static_cast<int>(parkingAreasDrawn.size());
  rep.streetSideParkingLeftOut = static_cast<int>(parkingAreas.size() - parkingAreasDrawn.size());
  if (bays.empty()) return;
  std::vector<double> cuts;
  for (const SectionInfo &si : secs) cuts.push_back(si.s);
  for (const auto &b : bays) cuts.push_back(b.s0), cuts.push_back(b.s1);
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }), cuts.end());
  while (!cuts.empty() && cuts.back() >= len - 1e-6) cuts.pop_back();
  std::vector<SectionInfo> out;
  std::vector<int> overruled(bays.size(), 0);
  for (size_t c = 0; c < cuts.size(); ++c) {
    const double a = cuts[c], e = c + 1 < cuts.size() ? cuts[c + 1] : len;
    size_t k = 0;  // the original section holding [a, e)
    while (k + 1 < secs.size() && secs[k + 1].s <= a + 1e-6) ++k;
    CrossSection x = secs[k].xs;
    for (const bool left : {true, false}) {
      SideSpec &side = left ? x.left : x.right;
      const auto bay = std::find_if(bays.begin(), bays.end(), [&](const auto &b) { return b.left == left && b.s0 <= a + 1e-6 && b.s1 >= e - 1e-6; });
      if (bay == bays.end()) continue;  // no area here: the tags decide
      // an area beside a tagged parking lane as wide as that lane (within kAbsorb, mapping noise) confirms it: no
      // section of its own for a width change of a few decimetres
      const auto tagged = std::find_if(side.lanes.begin(), side.lanes.end(), [](const LaneSpec &l) { return l.kind == LaneKind::Parking; });
      if (tagged != side.lanes.end() && bay->filler == 0.0 && std::abs(tagged->width - bay->width) <= kAbsorb) continue;
      // geometry over tags: beside the area its measured lane replaces the parking lane the tags gave (counted once
      // per bay, where its first piece meets one)
      const auto n = side.lanes.size();
      side.lanes.erase(std::remove_if(side.lanes.begin(), side.lanes.end(), [](const LaneSpec &l) { return l.kind == LaneKind::Parking; }),
                       side.lanes.end());
      if (side.lanes.size() != n && !overruled[static_cast<size_t>(bay - bays.begin())]++) ++rep.parkingTagsOverruled;
      size_t at = 0;  // after the side's carriageway lanes
      while (at < side.lanes.size() && (side.lanes[at].kind == LaneKind::Driving || side.lanes[at].kind == LaneKind::Bus ||
                                        side.lanes[at].kind == LaneKind::Biking || side.lanes[at].kind == LaneKind::Bidirectional))
        ++at;
      std::vector<LaneSpec> add;
      if (bay->filler > 0.0) add.push_back({LaneKind::Border, bay->filler, {}});
      add.push_back({LaneKind::Parking, bay->width, {}});
      for (LaneSpec &l : add) {
        if (std::abs(a - bay->s0) < 1e-6) l.taperIn = bay->taperIn;
        if (std::abs(e - bay->s1) < 1e-6) l.taperOut = bay->taperOut;
      }
      side.lanes.insert(side.lanes.begin() + static_cast<long>(at), add.begin(), add.end());
    }
    appendPiece(out, a, x);
  }
  secs = out;
}

void Builder::keepTagParkingClearOfJunctions(const RoadMeta &m, double len, std::vector<SectionInfo> &secs) {
  const Chain &ch = chains[m.chain];
  const bool atStart = ch.c0 >= 0, atEnd = ch.c1 >= 0;  // road ends at a junction
  auto hasParking = [](const CrossSection &x) {
    for (const auto *side : {&x.left, &x.right})
      for (const LaneSpec &l : side->lanes)
        if (l.kind == LaneKind::Parking) return true;
    return false;
  };
  if ((!atStart && !atEnd) || std::none_of(secs.begin(), secs.end(), [&](const SectionInfo &si) { return hasParking(si.xs); }))
    return;
  // the no-stopping stretches at the junction ends (the whole road where too little is left to park on)
  const double d = opt.country.parkingClearOfJunction;
  double z0 = atStart ? d : 0.0, z1 = atEnd ? len - d : len;
  if (z1 - z0 < kMinBay) {  // nowhere to park on this road
    for (SectionInfo &si : secs)
      for (auto *side : {&si.xs.left, &si.xs.right})
        side->lanes.erase(std::remove_if(side->lanes.begin(), side->lanes.end(), [](const LaneSpec &l) { return l.kind == LaneKind::Parking; }),
                          side->lanes.end());
    std::vector<SectionInfo> out;
    for (const SectionInfo &si : secs) appendPiece(out, si.s, si.xs);
    secs = out;
    ++rep.parkingDroppedOnShortRoads;
    return;
  }
  for (double *e : {&z0, &z1})
    for (const SectionInfo &si : secs)
      if (std::abs(*e - si.s) < 1.0) *e = si.s;  // onto a section boundary within 1 m (no sliver sections)
  std::vector<double> cuts;
  for (const SectionInfo &si : secs) cuts.push_back(si.s);
  for (double c : {z0, z1})
    if (c > 1e-6 && c < len - 1e-6) cuts.push_back(c);
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a - b) < 1e-6; }), cuts.end());
  std::vector<SectionInfo> out;
  bool ruled = false;
  for (size_t c = 0; c < cuts.size(); ++c) {
    const double a = cuts[c], e = c + 1 < cuts.size() ? cuts[c + 1] : len;
    size_t k = 0;
    while (k + 1 < secs.size() && secs[k + 1].s <= a + 1e-6) ++k;
    CrossSection x = secs[k].xs;
    if (e <= z0 + 1e-6 || a >= z1 - 1e-6)  // a no-stopping stretch: the parking strip stays, its use is forbidden
      for (auto *side : {&x.left, &x.right})
        for (LaneSpec &l : side->lanes)
          if (l.kind == LaneKind::Parking) l.rule = kNoStopping, ruled = true;
    appendPiece(out, a, x);
  }
  if (ruled) ++rep.parkingNoStoppingAtJunctions;
  secs = out;
}

}  // namespace detail
}  // namespace odrforge
