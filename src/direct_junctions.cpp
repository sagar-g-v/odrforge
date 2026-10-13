// OdrForge - direct junctions at freeway splits and merges.
//
// OpenDRIVE 1.9 §12.6 (introduced 1.7.0) and ASAM Junction Guideline §7.2: an entry or exit whose
// lanes split or merge without crossing is a direct junction - the roads are linked to each other
// at one cut, without connecting roads. The lane-level gores (gore.cpp) already split the lanes at
// the separation point with a few metres of connecting roads, each continuing its incoming road's
// lanes 1:1 and meeting the arms in position and heading. Here, for revisions with direct
// junctions, each of those connecting roads is joined to the road it leads to (a split) or comes
// from (a merge), so the arms meet at the cut, and the junction becomes type="direct" with
// <connection incomingRoad linkedRoad>. The carla profile (1.4) keeps the common junction.

#include <algorithm>
#include <map>
#include <optional>
#include <set>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

/// records with absolute s, shifted by ds
template <typename T>
void appendShifted(std::vector<T> &to, std::vector<T> from, double ds) {
  for (T &r : from) r.s += ds;
  to.insert(to.end(), from.begin(), from.end());
}

/// a profile (elevation, superelevation, laneOffset) is defined from s = 0 on: an empty one is 0
void startAtZero(std::vector<odr::Poly3> &p) {
  if (p.empty()) p.push_back({0.0, 0.0, 0.0, 0.0, 0.0});
}

/// The joined road runs the arm's whole cross-section (the lanes carried over included) along the
/// connecting road's bend. OpenDRIVE 1.9 §8.3: a lane border offset t from a line of curvature k
/// folds where k t reaches 1; the join is made only where the arm's outermost border at the seam
/// keeps k t within kMaxBend (as keepVergeFoldSafe), else the gore stays a common junction.
bool foldSafe(const odr::Road &c, const odr::Road &arm, bool diverge) {
  constexpr double kMaxBend = 0.8;
  double kmax = 0.0;
  for (const GeomElement &e : c.planView.elements) kmax = std::max({kmax, std::abs(e.k0), std::abs(e.k1)});
  const double s = diverge ? 0.0 : arm.length();
  const odr::LaneSection &sec = diverge ? arm.lanes.front() : arm.lanes.back();
  double reach = std::abs(odr::laneOffsetAt(arm, s));
  for (const auto *side : {&sec.left, &sec.right})
    for (const odr::Lane &l : *side) {
      double a, b;
      if (odr::laneBorders(arm, s, l.id, a, b)) reach = std::max({reach, std::abs(a), std::abs(b)});
    }
  return kmax * reach <= kMaxBend;
}

odr::Lane *laneById(odr::LaneSection &sec, int id) {
  for (auto *side : {&sec.left, &sec.right})
    for (odr::Lane &l : *side)
      if (l.id == id) return &l;
  return nullptr;
}

/// OpenDRIVE 1.9 §11.6: lanes meeting at a lane section boundary link to each other (both ways)
void linkAcross(odr::LaneSection &before, odr::LaneSection &after) {
  for (auto *side : {&before.left, &before.right})
    for (odr::Lane &l : *side)
      if (l.link.successor)
        if (odr::Lane *n = laneById(after, *l.link.successor)) n->link.predecessor = l.id;
  for (auto *side : {&after.left, &after.right})
    for (odr::Lane &n : *side)
      if (n.link.predecessor)
        if (odr::Lane *l = laneById(before, *n.link.predecessor)) l->link.successor = n.id;
}

/// The connecting road carries only the lanes it links. Lanes of the arm section it touches that no
/// connecting lane reaches (shoulders, beyond the linked lanes on each side) begin (a split) or end (a
/// merge) inside the joined road: they are carried over the connecting road's length, growing from or
/// shrinking to zero width (OpenDRIVE 1.9 §11.6: a lane of zero width at its start has no
/// predecessor). Only for a connecting road of one lane section whose lanes keep the arm's lane ids
/// (else nothing is carried and false is returned).
bool carryArmLanes(odr::Road &c, const odr::Road &arm, bool diverge, std::vector<int> &carried) {
  if (c.lanes.size() != 1) return false;
  odr::LaneSection &cs = c.lanes.front();
  const odr::LaneSection &as = diverge ? arm.lanes.front() : arm.lanes.back();
  // length of the arm's touching section (a merge reads the width at its end)
  const double asLen = diverge ? 0.0 : arm.length() - arm.lanes.back().s;
  const double L = c.length();
  for (const bool left : {true, false}) {
    std::vector<odr::Lane> &cl = left ? cs.left : cs.right;
    const std::vector<odr::Lane> &al = left ? as.left : as.right;
    for (const odr::Lane &l : cl) {
      const auto &to = diverge ? l.link.successor : l.link.predecessor;
      if (!to || *to != l.id) return false;  // the arm's lanes keep their ids across the cut
    }
    for (const odr::Lane &a : al) {
      if (std::abs(a.id) <= static_cast<int>(cl.size())) continue;  // linked
      odr::Lane n = a;
      const double w = odr::laneWidthAt(a, diverge ? 0.0 : asLen);
      n.widths = {odr::Poly3::hermite(0.0, L, diverge ? 0.0 : w, 0.0, diverge ? w : 0.0, 0.0)};
      n.link = {};
      (diverge ? n.link.successor : n.link.predecessor) = a.id;
      for (auto &m : n.roadMarks) m.sOffset = 0.0;
      n.roadMarks.resize(std::min<size_t>(1, n.roadMarks.size()));
      cl.push_back(n);
      carried.push_back(n.id);
    }
  }
  return true;
}

/// A carried lane opens (closes) over the connecting road only - a few metres, steeper than its edge
/// may turn without running backwards (fold). Its width ramps over at least kRampPerWidth times its
/// width instead, continuing into the arm's lane section (one section after mergeSeam): a cubic from
/// zero at the section start (to zero at its end), then the arm's own width records.
void stretchTaper(odr::Lane &l, double secLen, bool atStart) {
  constexpr double kRampPerWidth = 3.0;  // m of ramp per m of width: edge slope at most 1.5 / 3 = 0.5
  const double full = odr::laneWidthAt(l, atStart ? secLen : 0.0);  // the arm lane's width away from the cut
  const double T = std::min(kRampPerWidth * full, 0.8 * secLen);
  if (T <= 0.0) return;
  const double at = atStart ? T : secLen - T;  // where the ramp meets the arm's own widths
  constexpr double h = 1e-3;
  const double w = odr::laneWidthAt(l, at);
  const double m = (odr::laneWidthAt(l, at + h) - odr::laneWidthAt(l, at - h)) / (2.0 * h);
  // the arm's records beyond the ramp; the one the ramp cuts re-based at the cut (same cubic)
  std::vector<odr::Poly3> rest;
  for (size_t i = 0; i < l.widths.size(); ++i) {
    const odr::Poly3 &p = l.widths[i];
    const double s1 = i + 1 < l.widths.size() ? l.widths[i + 1].s : secLen;
    if (atStart ? s1 <= at : p.s >= at) continue;
    if (atStart && p.s < at) {
      const double u = at - p.s;
      rest.push_back({at, p.eval(u), p.deriv(u), p.c + 3.0 * p.d * u, p.d});
    } else {
      rest.push_back(p);
    }
  }
  if (atStart) {
    l.widths = {odr::Poly3::hermite(0.0, T, 0.0, 0.0, w, m)};
    l.widths.insert(l.widths.end(), rest.begin(), rest.end());
  } else {
    l.widths = rest;
    l.widths.push_back(odr::Poly3::hermite(at, T, w, m, 0.0, 0.0));
  }
}

/// one lane layout on both sides of section boundary k of a road, every lane linked to itself: the
/// boundary is removable (NAT-2), the sections become one, widths and marks continuing as records
bool mergeSeam(odr::Road &r, size_t k) {
  if (k == 0 || k >= r.lanes.size()) return false;
  odr::LaneSection &A = r.lanes[k - 1];
  const odr::LaneSection &B = r.lanes[k];
  const double ds = B.s - A.s;
  auto sameSide = [](const std::vector<odr::Lane> &x, const std::vector<odr::Lane> &y) {
    if (x.size() != y.size()) return false;
    for (size_t i = 0; i < x.size(); ++i)
      if (x[i].id != y[i].id || x[i].type != y[i].type || x[i].link.successor != x[i].id || y[i].link.predecessor != y[i].id)
        return false;
    return true;
  };
  if (!sameSide(A.left, B.left) || !sameSide(A.right, B.right)) return false;
  auto continueLane = [&](odr::Lane &a, const odr::Lane &b) {
    for (odr::Poly3 w : b.widths) {
      w.s += ds;
      a.widths.push_back(w);
    }
    auto sameMark = [](const odr::RoadMark &x, const odr::RoadMark &y) {
      return x.type == y.type && x.color == y.color && x.width == y.width && x.laneChange == y.laneChange && x.weight == y.weight;
    };
    for (odr::RoadMark m : b.roadMarks) {
      m.sOffset += ds;
      if (a.roadMarks.empty() || !sameMark(a.roadMarks.back(), m)) a.roadMarks.push_back(m);
    }
    a.link.successor = b.link.successor;
    a.speedMax = b.speedMax;
  };
  for (size_t i = 0; i < A.left.size(); ++i) continueLane(A.left[i], B.left[i]);
  for (size_t i = 0; i < A.right.size(); ++i) continueLane(A.right[i], B.right[i]);
  for (odr::RoadMark m : B.center.roadMarks) {
    m.sOffset += ds;
    if (A.center.roadMarks.empty() || m.type != A.center.roadMarks.back().type) A.center.roadMarks.push_back(m);
  }
  r.lanes.erase(r.lanes.begin() + static_cast<long>(k));
  return true;
}

/// road b continued after road a (a's end is b's start, both running the same way); identity, links
/// and data of the plain road `keep` are kept
odr::Road joined(const odr::Road &a, const odr::Road &b, const odr::Road &keep) {
  const double La = a.length();
  odr::Road r = keep;
  r.planView = a.planView;
  for (GeomElement e : b.planView.elements) {
    e.s += La;
    r.planView.elements.push_back(e);
  }
  r.planView.finalize();
  r.elevation = a.elevation, r.superelevation = a.superelevation, r.laneOffset = a.laneOffset;
  for (auto *p : {&r.elevation, &r.superelevation, &r.laneOffset}) startAtZero(*p);
  appendShifted(r.elevation, b.elevation, La);
  appendShifted(r.superelevation, b.superelevation, La);
  appendShifted(r.laneOffset, b.laneOffset, La);
  r.types = a.types;
  appendShifted(r.types, b.types, La);
  r.lanes = a.lanes;
  appendShifted(r.lanes, b.lanes, La);
  linkAcross(r.lanes[a.lanes.size() - 1], r.lanes[a.lanes.size()]);
  r.signals = a.signals, r.objects = a.objects, r.bridges = a.bridges, r.tunnels = a.tunnels;
  r.signalReferences = a.signalReferences;
  appendShifted(r.signals, b.signals, La);
  appendShifted(r.objects, b.objects, La);
  appendShifted(r.bridges, b.bridges, La);
  appendShifted(r.tunnels, b.tunnels, La);
  appendShifted(r.signalReferences, b.signalReferences, La);
  r.predecessor = a.predecessor, r.successor = b.successor;
  return r;
}

/// lanes at a road end that links to a junction take their links from the junction (Annex F
/// road.lane.link.use_junctions): links into the next road are dropped there
void dropLinksAt(odr::Road &r, bool atEnd) {
  odr::LaneSection &sec = atEnd ? r.lanes.back() : r.lanes.front();
  for (auto *side : {&sec.left, &sec.right})
    for (odr::Lane &l : *side) (atEnd ? l.link.successor : l.link.predecessor).reset();
}

/// A lane of road r followed from section k toward the road's end (forward) or start, through its lane
/// links, retyped while it is a driving lane: the lane a ramp feeds (or leaves from) on the trunk.
void retypeAlong(odr::Road &r, size_t k, int id, bool forward, const std::string &type) {
  std::optional<int> at = id;
  for (long i = static_cast<long>(k); at && i >= 0 && i < static_cast<long>(r.lanes.size()); i += forward ? 1 : -1) {
    odr::Lane *l = laneById(r.lanes[static_cast<size_t>(i)], *at);
    if (!l || l->type != "driving") return;
    l->type = type;
    at = forward ? l->link.successor : l->link.predecessor;
  }
}

/// every driving lane of a road retyped (the ramp itself)
void retypeRoad(odr::Road &r, const std::string &type) {
  for (odr::LaneSection &sec : r.lanes)
    for (auto *side : {&sec.left, &sec.right})
      for (odr::Lane &l : *side)
        if (l.type == "driving") l.type = type;
}

}  // namespace

void Builder::directJunctions() {
  if (!opt.target.spec().hasDirectJunctions()) return;
  std::map<std::string, size_t> indexOf;
  for (size_t i = 0; i < od.roads.size(); ++i) indexOf[od.roads[i].id] = i;
  std::vector<char> dropped(od.roads.size(), 0);
  // ASAM Junction Guideline §7.2.2.3 (OpenDRIVE 1.9 §11.8 lane types): a ramp leaving at an exit is
  // offRamp, one joining at an entry onRamp, one doing both (motorway to motorway) connectingRamp; the
  // trunk lanes leading into an exit ramp are exit from where they begin, those an entry ramp becomes
  // are entry until they end
  constexpr int kLeaves = 1, kJoins = 2;
  std::map<size_t, int> rampRole;  // road index -> kLeaves | kJoins
  struct TrunkLane {
    size_t road;
    int id;
    bool entry;  // followed from the junction forward (entry) or backward (exit)
  };
  std::vector<TrunkLane> trunkLanes;
  for (const Cluster &cl : clusters) {
    // OpenDRIVE 1.8.0 §12.1 Table 54: a junction with traffic lights is a common junction
    if (!cl.gore || cl.junction < 0 || cl.signalised) continue;
    odr::Junction &J = od.junctions[cl.junction];
    const bool diverge = cl.gore->diverge;
    // every connection joins one connecting road to one plain road (its arm on the far side), each
    // arm takes exactly one, and every lane of that arm end continues from a connecting lane
    struct Join {
      size_t conn, plain;
    };
    std::vector<Join> joins;
    std::map<size_t, int> uses;
    bool ok = !J.connections.empty();
    for (const odr::Connection &cn : J.connections) {
      const auto ic = indexOf.find(cn.connectingRoad);
      if (ic == indexOf.end()) { ok = false; break; }
      const odr::Road &c = od.roads[ic->second];
      const auto &far = diverge ? c.successor : c.predecessor;
      if (!far || far->elementType != "road" || far->contactPoint != (diverge ? "start" : "end")) { ok = false; break; }
      const auto ip = indexOf.find(far->elementId);
      if (ip == indexOf.end()) { ok = false; break; }
      const odr::Road &p = od.roads[ip->second];
      if (!foldSafe(c, p, diverge)) { ok = false; break; }
      const odr::LaneSection &touch = diverge ? p.lanes.front() : p.lanes.back();
      odr::LaneSection cEnd = diverge ? c.lanes.back() : c.lanes.front();
      std::set<int> reached;
      for (const auto *side : {&cEnd.left, &cEnd.right})
        for (const odr::Lane &l : *side)
          if (const auto &to = diverge ? l.link.successor : l.link.predecessor) reached.insert(*to);
      for (const auto *side : {&touch.left, &touch.right})
        for (const odr::Lane &l : *side)
          if (l.type == "driving" && !reached.count(l.id)) ok = false;
      ++uses[ip->second];
      joins.push_back({ic->second, ip->second});
    }
    for (const auto &[p, n] : uses) ok &= n == 1;
    if (!ok) continue;
    // join and relink: a split links the incoming road to each arm, a merge each arm to the trunk
    std::vector<odr::Connection> direct;
    for (const Join &j : joins) {
      const odr::Road c = od.roads[j.conn];
      odr::Road &p = od.roads[j.plain];
      const odr::Connection &orig = *std::find_if(J.connections.begin(), J.connections.end(),
                                                  [&](const odr::Connection &x) { return x.connectingRoad == c.id; });
      odr::Connection cn;
      cn.id = std::to_string(direct.size());
      cn.contactPoint = "start";
      odr::Road cc = c;
      std::vector<int> carriedIds;
      const bool carried = carryArmLanes(cc, p, diverge, carriedIds);
      auto stretchCarried = [&](odr::Road &r, size_t k, bool atStart) {
        const double secLen = (k + 1 < r.lanes.size() ? r.lanes[k + 1].s : r.length()) - r.lanes[k].s;
        for (int id : carriedIds)
          if (odr::Lane *l = laneById(r.lanes[k], id)) stretchTaper(*l, secLen, atStart);
      };
      if (diverge) {
        const size_t seam = cc.lanes.size();
        p = joined(cc, p, p);
        if (carried && mergeSeam(p, seam)) stretchCarried(p, 0, true);
        p.predecessor = odr::LinkTarget{"junction", J.id, ""};
        dropLinksAt(p, false);
        cn.incomingRoad = orig.incomingRoad;
        cn.connectingRoad = p.id;  // written as linkedRoad for a direct junction
        cn.laneLinks = orig.laneLinks;
      } else {
        for (const auto *side : {&c.lanes.back().left, &c.lanes.back().right})
          for (const odr::Lane &l : *side)
            if (l.link.successor) cn.laneLinks.push_back({l.id, *l.link.successor});
        const size_t seam = p.lanes.size();
        p = joined(p, cc, p);
        if (carried && mergeSeam(p, seam)) stretchCarried(p, p.lanes.size() - 1, false);
        p.successor = odr::LinkTarget{"junction", J.id, ""};
        dropLinksAt(p, true);
        cn.incomingRoad = p.id;
        cn.connectingRoad = c.successor->elementId;  // the trunk
      }
      direct.push_back(cn);
      dropped[j.conn] = 1;
    }
    J.type = "direct";
    J.connections = direct;
    ++rep.directJunctions;
    // lane types once every junction is joined (a ramp may be joined at both ends)
    const size_t rampIndex = metas[cl.gore->branch].roadIndex, trunkIndex = metas[cl.gore->trunk].roadIndex;
    for (const odr::Connection &cn : J.connections) {
      if (diverge && cn.connectingRoad == od.roads[rampIndex].id) {
        rampRole[rampIndex] |= kLeaves;
        for (const odr::LaneLink &ll : cn.laneLinks) trunkLanes.push_back({trunkIndex, ll.from, false});
      } else if (!diverge && cn.incomingRoad == od.roads[rampIndex].id) {
        rampRole[rampIndex] |= kJoins;
        for (const odr::LaneLink &ll : cn.laneLinks) trunkLanes.push_back({trunkIndex, ll.to, true});
      }
    }
  }
  for (const auto &[road, role] : rampRole)
    retypeRoad(od.roads[road], role == (kLeaves | kJoins) ? "connectingRamp" : role == kJoins ? "onRamp" : "offRamp");
  for (const TrunkLane &t : trunkLanes) {
    odr::Road &r = od.roads[t.road];
    retypeAlong(r, t.entry ? 0 : r.lanes.size() - 1, t.id, t.entry, t.entry ? "entry" : "exit");
  }
  // connecting roads are appended after the plain roads: dropping them leaves the plain roads' places
  std::vector<odr::Road> kept;
  for (size_t i = 0; i < od.roads.size(); ++i)
    if (!dropped[i]) kept.push_back(std::move(od.roads[i]));
  od.roads = std::move(kept);
}

}  // namespace detail
}  // namespace odrforge
