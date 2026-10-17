// OdrForge - street parking.
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

#include <algorithm>
#include <cmath>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
constexpr double kMinBay = 5.0;  // buildRoads merges sections shorter than this away
// OpenDRIVE 1.8.0 §11.9 lane rule, one of its recommended values
constexpr const char *kNoStopping = "no stopping at any time";

/// append a piece of road [s, ...) with cross-section x; a piece equal to the one before (by the rule of buildRoads'
/// mergeEqualSections) continues it
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
      return;
    }
  }
  out.push_back({s, x});
}
}  // namespace

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
