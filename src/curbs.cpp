// OdrForge - curb lanes (standard profile).
//
// OpenDRIVE 1.9 §11.8 lane type curb (introduced 1.6): the kerb stones between the carriageway and a
// raised sidewalk are a lane of their own. Every outermost raised sidewalk lane - on the roads and on
// the connecting roads that carry the sidewalk round junction corners - gives its inner kCurbWidth to
// a curb lane, so the sidewalk's outer edge stays where it was and the curb runs on round the corner.
// A sidewalk behind a verge (border lane, separately mapped sidewalks) keeps the curb mark the verge
// carries instead. The carla profile (1.4, no curb type) is unchanged.
//
// Lane ids: the sidewalk's id goes to the curb and the sidewalk moves out by one, so every link that
// named a sidewalk now names the curb that took its id, and the sidewalk gets the same link shifted.

#include <algorithm>
#include <map>
#include <tuple>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

constexpr double kCurbWidth = 0.15;  // m: a kerb stone

/// the outermost lane of a side when it is a raised sidewalk directly beside a non-border lane (or alone)
odr::Lane *curbSidewalk(std::vector<odr::Lane> &side) {
  if (side.empty() || side.back().type != "sidewalk" || !side.back().heightInner) return nullptr;
  if (side.size() >= 2 && side[side.size() - 2].type == "border") return nullptr;
  return &side.back();
}

/// width cubic of the curb and the sidewalk from the sidewalk's: a record that stays at least twice the
/// curb width gives the curb kCurbWidth (the sidewalk the rest, the same cubic less a constant); a record
/// opening from (closing to) zero gives the curb the same share of it that kCurbWidth is of its wide end,
/// so both borders run on continuously into the neighbouring records
void splitWidth(const odr::Poly3 &w, double len, odr::Poly3 &curb, odr::Poly3 &walk) {
  double lo = 1e9, hi = 0.0;
  for (int i = 0; i <= 8; ++i) {
    const double v = w.eval(len * i / 8.0);
    lo = std::min(lo, v), hi = std::max(hi, v);
  }
  if (lo >= 2.0 * kCurbWidth) {
    curb = {w.s, kCurbWidth, 0.0, 0.0, 0.0};
    walk = w, walk.a -= kCurbWidth;
    return;
  }
  const double f = hi > 1e-9 ? std::min(0.5, kCurbWidth / hi) : 0.5;
  curb = {w.s, f * w.a, f * w.b, f * w.c, f * w.d};
  walk = {w.s, (1 - f) * w.a, (1 - f) * w.b, (1 - f) * w.c, (1 - f) * w.d};
}

}  // namespace

void Builder::curbLanes() {
  if (!opt.target.spec().hasCurbLanes()) return;
  // 1. sections whose sidewalk becomes curb + sidewalk: (road id, section, left) -> the sidewalk's old id
  std::map<std::tuple<std::string, size_t, bool>, int> split;
  for (odr::Road &r : od.roads)
    for (size_t k = 0; k < r.lanes.size(); ++k)
      for (const bool left : {true, false})
        if (odr::Lane *l = curbSidewalk(left ? r.lanes[k].left : r.lanes[k].right)) split[{r.id, k, left}] = l->id;
  auto isSplit = [&](const std::string &road, size_t k, int id) {
    auto it = split.find({road, k, id > 0});
    return it != split.end() && it->second == id;
  };
  auto outward = [](int id) { return id > 0 ? id + 1 : id - 1; };
  std::map<std::string, odr::Road *> byId;
  for (odr::Road &r : od.roads) byId[r.id] = &r;
  // the section of road `id` that a link from its start (end) reaches
  auto endSection = [&](const std::string &id, bool atEnd) -> size_t {
    const odr::Road &o = *byId.at(id);
    return atEnd ? o.lanes.size() - 1 : 0;
  };
  // 2. split, links: the curb keeps the sidewalk's links (they name the neighbours' curbs now), the
  //    sidewalk's links move out by one where the neighbour was split too
  for (odr::Road &r : od.roads)
    for (size_t k = 0; k < r.lanes.size(); ++k) {
      const double secLen = (k + 1 < r.lanes.size() ? r.lanes[k + 1].s : r.length()) - r.lanes[k].s;
      for (const bool left : {true, false}) {
        auto &side = left ? r.lanes[k].left : r.lanes[k].right;
        odr::Lane *walkPtr = curbSidewalk(side);
        if (!walkPtr) continue;
        odr::Lane walk = *walkPtr, curb = *walkPtr;
        curb.type = "curb";
        curb.widths.clear(), walk.widths.clear();
        for (size_t i = 0; i < walkPtr->widths.size(); ++i) {
          const odr::Poly3 &w = walkPtr->widths[i];
          const double len = (i + 1 < walkPtr->widths.size() ? walkPtr->widths[i + 1].s : secLen) - w.s;
          odr::Poly3 c, s;
          splitWidth(w, len, c, s);
          curb.widths.push_back(c), walk.widths.push_back(s);
        }
        curb.roadMarks = {odr::RoadMark{}};  // the kerb itself carries no painted line
        walk.id = outward(walk.id);
        // neighbours within the road (sections k - 1, k + 1) and across road links at the ends: where the
        // neighbouring sidewalk was split too, the curb links to its curb and the sidewalk moves out by
        // one; else the sidewalk keeps its link and the curb ends there
        auto relink = [&](std::optional<int> &curbLink, std::optional<int> &walkLink, bool toNext) {
          if (!walkLink) return;
          std::string road = r.id;
          size_t kk = toNext ? k + 1 : k - 1;
          if (toNext ? k + 1 >= r.lanes.size() : k == 0) {
            const auto &lt = toNext ? r.successor : r.predecessor;
            if (!lt || lt->elementType != "road" || !byId.count(lt->elementId)) {
              curbLink.reset();
              return;
            }
            road = lt->elementId;
            kk = endSection(road, lt->contactPoint == "end");
          }
          if (isSplit(road, kk, *walkLink)) walkLink = outward(*walkLink);
          else curbLink.reset();
        };
        relink(curb.link.predecessor, walk.link.predecessor, false);
        relink(curb.link.successor, walk.link.successor, true);
        side.back() = curb;
        side.push_back(walk);
      }
    }
  // 3. junction laneLinks naming a sidewalk on both sides name the curbs now; the sidewalks get their own
  for (odr::Junction &J : od.junctions)
    for (odr::Connection &cn : J.connections) {
      if (!byId.count(cn.incomingRoad) || !byId.count(cn.connectingRoad)) continue;
      const odr::Road &inc = *byId.at(cn.incomingRoad);
      const bool incAtEnd = inc.successor && inc.successor->elementType == "junction" && inc.successor->elementId == J.id;
      const size_t ki = incAtEnd ? inc.lanes.size() - 1 : 0;
      const size_t kc = endSection(cn.connectingRoad, cn.contactPoint == "end");
      std::vector<odr::LaneLink> extra;
      for (const odr::LaneLink &ll : cn.laneLinks)
        if (isSplit(inc.id, ki, ll.from) && isSplit(cn.connectingRoad, kc, ll.to)) extra.push_back({outward(ll.from), outward(ll.to)});
      cn.laneLinks.insert(cn.laneLinks.end(), extra.begin(), extra.end());
    }
  rep.curbLanes = static_cast<int>(split.size());
}

}  // namespace detail
}  // namespace odrforge
