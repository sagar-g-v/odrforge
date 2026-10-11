#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <limits>
#include <set>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

enum class Turn { Straight, Left, Right, UTurn };

Turn classify(double dh) {
  const double a = std::abs(dh);
  if (a > 150.0 * kPi / 180.0) return Turn::UTurn;
  if (a < 35.0 * kPi / 180.0) return Turn::Straight;
  return dh > 0 ? Turn::Left : Turn::Right;
}

double maxCurvature(const ReferenceLine &l) {
  double k = 0;
  for (const auto &e : l.elements) k = std::max({k, std::abs(e.k0), std::abs(e.k1)});
  return k;
}

/// Smoothest standard-geometry path between two lane poses: a single clothoid, or two clothoids
/// through a midpoint sampled between the chord and the tangent intersection (min peak curvature).
ReferenceLine connectionPath(const Pose &a, const Pose &b) {
  ReferenceLine best;
  double bestScore = 1e300;
  auto consider = [&](std::vector<GeomElement> els) {
    ReferenceLine l;
    l.elements = std::move(els);
    l.finalize();
    const double score = maxCurvature(l) + 0.002 * l.length();
    if (score < bestScore) bestScore = score, best = l;
  };
  if (auto g = fitClothoidG1(a.pos(), a.hdg, b.pos(), b.hdg)) consider({*g});
  const Vec2 da = a.dir(), db = b.dir(), d = b.pos() - a.pos();
  const Vec2 chordMid = (a.pos() + b.pos()) * 0.5;
  Vec2 apex = chordMid;
  const double den = da.cross(db);
  if (std::abs(den) > 1e-3) {
    const double u = d.cross(db) / den;
    if (u > 0 && u < 3.0 * d.norm()) apex = a.pos() + da * u;
  }
  const double chordHdg = std::atan2(d.y, d.x);
  const double meanHdg = a.hdg + 0.5 * normalizeAngle(b.hdg - a.hdg);
  for (double t : {0.0, 0.25, 0.4, 0.55, 0.7}) {
    const Vec2 m = chordMid + (apex - chordMid) * t;
    for (double hm : {chordHdg, meanHdg}) {
      auto g1 = fitClothoidG1(a.pos(), a.hdg, m, hm);
      auto g2 = fitClothoidG1(m, hm, b.pos(), b.hdg);
      if (g1 && g2) consider({*g1, *g2});
    }
  }
  return best;
}

bool allows(const std::vector<std::string> &turns, Turn t) {
  for (const auto &v : turns) {
    if (t == Turn::Left && (v == "left" || v == "sharp_left" || v == "slight_left")) return true;
    if (t == Turn::Right && (v == "right" || v == "sharp_right" || v == "slight_right")) return true;
    if (t == Turn::Straight && (v == "through" || v == "none" || v.empty() || v == "slight_left" ||
                                v == "slight_right" || v == "merge_to_left" || v == "merge_to_right"))
      return true;
    if (t == Turn::UTurn && (v == "reverse")) return true;
  }
  return false;
}

}  // namespace

void Builder::buildJunctions() {
  const bool LHT = opt.country.leftHandTraffic;
  // turn restrictions: (fromWay, viaNode, toWay) -> allowed?
  struct Restriction {
    OsmId from, via, to;
    bool only;
  };
  std::vector<Restriction> restr;
  for (const auto &[id, rel] : osm.relations) {
    if (!tagIs(rel.tags, "type", "restriction")) continue;
    const std::string r = tag(rel.tags, "restriction").value_or("");
    if (r.empty()) continue;
    Restriction x{0, 0, 0, r.rfind("only_", 0) == 0};
    for (const auto &m : rel.members) {
      if (m.role == "from" && m.type == 'w') x.from = m.ref;
      if (m.role == "to" && m.type == 'w') x.to = m.ref;
      if (m.role == "via" && m.type == 'n') x.via = m.ref;
    }
    if (x.from && x.to && x.via) restr.push_back(x);
  }

  int nextConnId = static_cast<int>(od.roads.size()) + 1;
  std::vector<odr::Road> connecting;  // appended after the loop: od.roads must not reallocate while referenced
  for (Cluster &cl : clusters) {
    if (cl.ends.empty()) continue;
    odr::Junction J;
    J.id = std::to_string(od.junctions.size() + 1);
    cl.junction = static_cast<int>(od.junctions.size());
    // link the roads to the junction
    for (const RoadEnd &e : cl.ends) {
      odr::Road &r = od.roads[metas[e.road].roadIndex];
      odr::LinkTarget t{"junction", J.id, ""};
      (e.atEnd ? r.successor : r.predecessor) = t;
    }
    int connIdx = 0;
    // a movement sharper than 150 degrees is a U-turn and is not built - unless it is the only way
    // out of (or into) a road end and leads to another road: a driveway drawn alongside the street
    // and hooking into it at the last node still connects (as a tight turn), it is not a dead end
    auto endHeading = [&](const RoadEnd &e, bool incoming) {
      const odr::Road &r = od.roads[metas[e.road].roadIndex];
      const Pose p = r.planView.eval(e.atEnd ? r.length() : 0.0);
      return incoming == e.atEnd ? p.hdg : p.hdg + kPi;
    };
    auto hasLanes = [&](const RoadEnd &e, bool incoming) {
      return !endLanes(od.roads[metas[e.road].roadIndex], e.atEnd, incoming).empty();
    };
    auto regularPartner = [&](const RoadEnd &e, bool incoming) {
      for (const RoadEnd &o : cl.ends) {
        if (o.road == e.road && o.atEnd == e.atEnd) continue;
        if (!hasLanes(o, !incoming)) continue;
        const double dh = incoming ? endHeading(o, false) - endHeading(e, true) : endHeading(e, false) - endHeading(o, true);
        if (classify(normalizeAngle(dh)) != Turn::UTurn) return true;
      }
      return false;
    };
    for (const RoadEnd &in : cl.ends) {
      const RoadMeta &mi = metas[in.road];
      const odr::Road &ri = od.roads[mi.roadIndex];
      const double sIn = in.atEnd ? ri.length() : 0.0;
      const std::vector<int> inLanes = endLanes(ri, in.atEnd, true);
      if (inLanes.empty()) continue;
      const Pose pin = ri.planView.eval(sIn);
      const double hIn = in.atEnd ? pin.hdg : pin.hdg + kPi;
      // turn tags of the incoming lanes (inner -> outer on their side)
      const CrossSection &xsIn = sectionAtFull(mi, in.atEnd ? mi.full.length() - mi.trimEnd : mi.trimStart);
      auto turnsOf = [&](int laneId) -> std::vector<std::string> {
        const SideSpec &side = laneId > 0 ? xsIn.left : xsIn.right;
        const size_t k = static_cast<size_t>(std::abs(laneId) - 1);
        return k < side.lanes.size() ? side.lanes[k].turns : std::vector<std::string>{};
      };
      // order incoming lanes from the left of the driver: inner lanes are on the driver's left in RHT
      std::vector<int> byLeft = inLanes;
      std::sort(byLeft.begin(), byLeft.end(), [](int a, int b) { return std::abs(a) < std::abs(b); });
      if (LHT) std::reverse(byLeft.begin(), byLeft.end());
      const int nIn = static_cast<int>(byLeft.size());

      struct Target {
        const RoadEnd *end;
        Turn turn;
        std::vector<int> lanes;  // left -> right in its driving direction
      };
      std::vector<Target> targets;
      for (const RoadEnd &out : cl.ends) {
        // never back into the same road: its own end (a U-turn) or, for a loop road with both ends
        // here, its start (driving round again through the junction)
        if (out.road == in.road) continue;
        const RoadMeta &mo = metas[out.road];
        const odr::Road &ro = od.roads[mo.roadIndex];
        std::vector<int> ol = endLanes(ro, out.atEnd, false);
        if (ol.empty()) continue;
        const Pose po = ro.planView.eval(out.atEnd ? ro.length() : 0.0);
        const double hOut = out.atEnd ? po.hdg + kPi : po.hdg;
        Turn t = classify(normalizeAngle(hOut - hIn));
        if (t == Turn::UTurn) {
          if (out.road == in.road || (regularPartner(in, true) && regularPartner(out, false))) continue;
          t = normalizeAngle(hOut - hIn) > 0 ? Turn::Left : Turn::Right;  // the only connection: a tight turn
        }
        // turn restrictions on the OSM ways adjacent to the junction
        const OsmId fromWay = in.atEnd ? mi.wayEnd : mi.wayStart;
        const OsmId toWay = out.atEnd ? mo.wayEnd : mo.wayStart;
        bool forbidden = false;
        for (const auto &x : restr) {
          if (x.from != fromWay || !std::count(cl.nodes.begin(), cl.nodes.end(), x.via)) continue;
          if (x.only ? x.to != toWay : x.to == toWay) forbidden = true;
        }
        if (forbidden) continue;
        std::sort(ol.begin(), ol.end(), [](int a, int b) { return std::abs(a) < std::abs(b); });
        if (LHT) std::reverse(ol.begin(), ol.end());
        targets.push_back({&out, t, ol});
      }
      if (targets.empty()) continue;
      bool anyStraight = false, anyLeft = false, anyRight = false;
      for (const auto &t : targets) {
        anyStraight |= t.turn == Turn::Straight;
        anyLeft |= t.turn == Turn::Left;
        anyRight |= t.turn == Turn::Right;
      }
      // permitted movements per incoming lane (turn:lanes, else positional defaults)
      std::vector<std::set<Turn>> perm(nIn);
      for (int i = 0; i < nIn; ++i) {
        const auto tr = turnsOf(byLeft[i]);
        if (!tr.empty() && !(tr.size() == 1 && tr[0].empty())) {
          for (Turn t : {Turn::Straight, Turn::Left, Turn::Right})
            if (allows(tr, t)) perm[i].insert(t);
        }
        if (perm[i].empty()) {
          perm[i].insert(Turn::Straight);
          if (i == 0 || !anyStraight) perm[i].insert(Turn::Left);
          if (i == nIn - 1 || !anyStraight) perm[i].insert(Turn::Right);
          if (!anyStraight && nIn > 1) {  // T-junction: split lanes between the two turns
            perm[i].clear();
            perm[i].insert(i < (nIn + 1) / 2 ? Turn::Left : Turn::Right);
            if (nIn == 1) perm[i] = {Turn::Left, Turn::Right};
          }
        }
      }
      (void)anyLeft;
      (void)anyRight;
      for (const Target &tg : targets) {
        const int nOut = static_cast<int>(tg.lanes.size());
        std::vector<std::pair<int, int>> pairs;  // (incoming lane id, outgoing lane id)
        if (cl.gore) {
          // lane-level gore: main lanes continue 1:1, the trunk's branch-side lanes are the branch
          const Cluster::Gore &g = *cl.gore;
          auto prop = [](int i, int n, int m) {
            return n == m || n <= 1 || m <= 1 ? std::min(i, m - 1) : static_cast<int>(std::lround(i * (m - 1) / double(n - 1)));
          };
          auto split = [&](const std::vector<int> &lanes, std::vector<int> &ext, std::vector<int> &main) {
            const int n = static_cast<int>(lanes.size()), k = std::min(g.extra, n - 1);
            for (int i = 0; i < n; ++i) ((g.branchLeft ? i < k : i >= n - k) ? ext : main).push_back(lanes[i]);
          };
          std::vector<int> ext, main;
          if (g.diverge && in.road == g.trunk) {
            split(byLeft, ext, main);
            const std::vector<int> &from = tg.end->road == g.branch ? ext : main;
            for (int i = 0; i < static_cast<int>(from.size()); ++i)
              pairs.push_back({from[i], tg.lanes[prop(i, static_cast<int>(from.size()), nOut)]});
          } else if (!g.diverge && tg.end->road == g.trunk) {
            split(tg.lanes, ext, main);
            const std::vector<int> &to = in.road == g.branch ? ext : main;
            if (to.empty()) continue;
            for (int i = 0; i < nIn; ++i) pairs.push_back({byLeft[i], to[prop(i, nIn, static_cast<int>(to.size()))]});
          }
        } else {
          std::vector<int> src;
          for (int i = 0; i < nIn; ++i)
            if (perm[i].count(tg.turn)) src.push_back(i);
          auto prop = [](int i, int n, int m) {
            return n == m || n <= 1 || m <= 1 ? std::min(i, m - 1) : static_cast<int>(std::lround(i * (m - 1) / double(n - 1)));
          };
          // a point a little way along a road from its end at this junction (which side it lies on)
          auto awayPoint = [&](const RoadEnd &e) {
            const odr::Road &r = od.roads[metas[e.road].roadIndex];
            const double d = std::min(10.0, r.length());
            return r.planView.eval(e.atEnd ? r.length() - d : d).pos();
          };
          // lane split: one road continuing straight on as several roads - its lanes are divided
          // left to right in the order the roads leave (no crossing connecting lanes)
          std::vector<const Target *> straight;
          for (const Target &t : targets)
            if (t.turn == Turn::Straight) straight.push_back(&t);
          bool partitioned = false;
          if (tg.turn == Turn::Straight && straight.size() >= 2 && !src.empty()) {
            const Vec2 dIn(std::cos(hIn), std::sin(hIn)), p0 = pin.pos();
            std::sort(straight.begin(), straight.end(), [&](const Target *a, const Target *b) {
              return dIn.cross(awayPoint(*a->end) - p0) > dIn.cross(awayPoint(*b->end) - p0);
            });
            int total = 0, c = 0;
            for (const Target *t : straight) total += static_cast<int>(t->lanes.size());
            for (const Target *t : straight) {
              if (t == &tg) break;
              c += static_cast<int>(t->lanes.size());
            }
            const int ns = static_cast<int>(src.size());
            std::vector<int> mine;
            for (int k = 0; k < ns; ++k) {
              const int q = static_cast<int>((k + 0.5) * total / ns);
              if (q >= c && q < c + nOut) mine.push_back(src[k]);
            }
            if (mine.empty()) mine.push_back(src[std::clamp(static_cast<int>((c + 0.5 * nOut) * ns / total), 0, ns - 1)]);
            for (int k = 0; k < static_cast<int>(mine.size()); ++k)
              pairs.push_back({byLeft[mine[k]], tg.lanes[prop(k, static_cast<int>(mine.size()), nOut)]});
            partitioned = true;
          }
          // lane merge: several roads continuing straight on into one - each takes the outgoing
          // lanes on its side
          if (!partitioned && tg.turn == Turn::Straight) {
            const odr::Road &ro0 = od.roads[metas[tg.end->road].roadIndex];
            const Pose po0 = ro0.planView.eval(tg.end->atEnd ? ro0.length() : 0.0);
            const double hOut0 = tg.end->atEnd ? po0.hdg + kPi : po0.hdg;
            const Vec2 dOut(std::cos(hOut0), std::sin(hOut0));
            std::vector<std::pair<double, int>> ins;  // (lateral, lane count) of straight incoming roads
            double mine = 0;
            for (const RoadEnd &e : cl.ends) {
              if (e.road == tg.end->road && e.atEnd == tg.end->atEnd) continue;
              const int n = static_cast<int>(endLanes(od.roads[metas[e.road].roadIndex], e.atEnd, true).size());
              if (!n) continue;
              const odr::Road &re = od.roads[metas[e.road].roadIndex];
              const Pose pe = re.planView.eval(e.atEnd ? re.length() : 0.0);
              const double he = e.atEnd ? pe.hdg : pe.hdg + kPi;
              if (classify(normalizeAngle(hOut0 - he)) != Turn::Straight) continue;
              const double lat = dOut.cross(awayPoint(e) - po0.pos());
              ins.push_back({lat, n});
              if (e.road == in.road && e.atEnd == in.atEnd) mine = lat;
            }
            if (ins.size() >= 2 && static_cast<int>(src.size()) == nIn) {
              std::sort(ins.begin(), ins.end(), [](const auto &a, const auto &b) { return a.first > b.first; });
              int total = 0, c = 0;
              for (const auto &x : ins) total += x.second;
              for (const auto &x : ins) {
                if (x.first == mine) break;
                c += x.second;
              }
              for (int i = 0; i < nIn; ++i)
                pairs.push_back({byLeft[i], tg.lanes[std::clamp(static_cast<int>((c + i + 0.5) * nOut / total), 0, nOut - 1)]});
              partitioned = true;
            }
          }
          const int ns = partitioned ? 0 : static_cast<int>(src.size());
          for (int k = 0; k < ns; ++k) {
            int o;
            if (tg.turn == Turn::Right) o = nOut - ns + k;            // align on the right
            else if (tg.turn == Turn::Left) o = k;                    // align on the left
            else o = ns == 1 ? std::min(nOut - 1, src[k] * nOut / std::max(1, nIn))
                             : static_cast<int>(std::lround(k * (nOut - 1) / double(ns - 1)));
            o = std::clamp(o, 0, nOut - 1);
            pairs.push_back({byLeft[src[k]], tg.lanes[o]});
          }
        }
        if (pairs.empty()) continue;
        const RoadMeta &mo = metas[tg.end->road];
        const odr::Road &ro = od.roads[mo.roadIndex];
        const double sOut = tg.end->atEnd ? ro.length() : 0.0;
        for (const auto &[li, lo] : pairs) {
          Pose a, b;
          odr::laneCenter(ri, sIn, li, a);
          odr::laneCenter(ro, sOut, lo, b);
          a.hdg = (li < 0) != LHT ? a.hdg : a.hdg + kPi;  // travel direction
          b.hdg = (lo < 0) != LHT ? b.hdg : b.hdg + kPi;
          double ta, tb, ua, ub;
          odr::laneBorders(ri, sIn, li, ta, tb);
          odr::laneBorders(ro, sOut, lo, ua, ub);
          const double w0 = std::abs(tb - ta), w1 = std::abs(ub - ua);
          // geometry: single clothoid when it is a reasonable curve, else a G2 two-piece curve
          ReferenceLine line = connectionPath(a, b);
          if (const char *dc = std::getenv("ODRFORGE_DEBUG_CONN"); dc && ri.id == dc) {
            std::fprintf(stderr, "conn %s->%s a (%.3f %.3f %.3f) b (%.3f %.3f %.3f)\n", ri.id.c_str(), ro.id.c_str(), a.x, a.y, a.hdg, b.x, b.y, b.hdg);
            for (const auto &e : line.elements) std::fprintf(stderr, "   el L %.4f k0 %.4f k1 %.4f\n", e.length, e.k0, e.k1);
          }
          {
            // sanity: a connecting path never turns much more than its end headings differ, nor
            // runs much longer than the gap it bridges (a looping clothoid root slipped through);
            // otherwise the simplest path: the chord (a few cm between nearly touching road ends)
            double turning = 0;
            for (const auto &e : line.elements) turning += 0.5 * (e.k0 + e.k1) * e.length;
            const double chord = (b.pos() - a.pos()).norm(), dh = std::abs(normalizeAngle(b.hdg - a.hdg));
            if (line.elements.empty() || std::abs(turning) > dh + 0.5 * kPi || line.length() > 4.0 * chord + 10.0) {
              if (std::getenv("ODRFORGE_DEBUG_CONN"))
                std::fprintf(stderr, "connecting %s->%s lanes %d->%d: path turning %.2f rad over %.2f m for gap %.2f m, dh %.2f\n",
                             ri.id.c_str(), ro.id.c_str(), li, lo, turning, line.length(), chord, dh);
              line = ReferenceLine{};
              if (chord > 1e-3) {
                GeomElement e;
                e.x = a.x, e.y = a.y, e.hdg = std::atan2(b.y - a.y, b.x - a.x), e.length = chord;
                line.elements.push_back(e);
                line.finalize();
              }
            }
          }
          if (line.elements.empty() || line.length() < 0.05) continue;
          odr::Road c;
          c.id = std::to_string(nextConnId++);
          c.junction = J.id;
          if (!ri.rule.empty()) c.rule = ri.rule;
          c.planView = line;
          c.predecessor = odr::LinkTarget{"road", ri.id, in.atEnd ? "end" : "start"};
          c.successor = odr::LinkTarget{"road", ro.id, tg.end->atEnd ? "end" : "start"};
          c.types.push_back({0.0, ri.types.empty() ? "town" : ri.types.back().type, ri.types.empty() ? "" : ri.types.back().country,
                             ri.types.empty() ? std::nullopt : ri.types.back().speedMax,
                             ri.types.empty() ? "km/h" : ri.types.back().speedUnit});
          const double L = line.length();
          const int cid = LHT ? 1 : -1;  // the connecting lane travels along +s
          const double sgn = LHT ? -1.0 : 1.0;
          c.laneOffset.push_back(odr::Poly3::hermite(0.0, L, sgn * 0.5 * w0, 0.0, sgn * 0.5 * w1, 0.0));
          odr::LaneSection sec;
          sec.center.id = 0;
          sec.center.type = "none";
          odr::RoadMark none;
          sec.center.roadMarks.push_back(none);
          odr::Lane l;
          l.id = cid;
          l.type = "driving";
          l.widths.push_back(odr::Poly3::hermite(0.0, L, w0, 0.0, w1, 0.0));
          l.link.predecessor = li;
          l.link.successor = lo;
          l.roadMarks.push_back(none);
          if (const auto *srcLane = odr::findLane(odr::sectionAt(ri, sIn), li)) l.speedMax = srcLane->speedMax;
          (LHT ? sec.left : sec.right).push_back(l);
          c.lanes.push_back(sec);
          // elevation and banking from the junction surface (shared by every connecting road)
          if (cl.hasPlane) {
            // flat cross sections (no superelevation): the plane plus a C1 correction that meets the
            // two road surfaces exactly at the lane centres
            const double z0 = odr::surfaceZ(ri, sIn, 0.5 * (ta + tb)), z1 = odr::surfaceZ(ro, sOut, 0.5 * (ua + ub));
            connectingElevation(c, cl, opt.target.superelevation ? std::nullopt : std::optional<std::pair<double, double>>({z0, z1}));
          } else {  // no junction surface: Hermite between the two road ends (C1)
            const double z0 = odr::surfaceZ(ri, sIn, 0.5 * (ta + tb)), z1 = odr::surfaceZ(ro, sOut, 0.5 * (ua + ub));
            const double m0 = odr::elevationSlopeAt(ri, sIn) * (in.atEnd ? 1.0 : -1.0);
            const double m1 = odr::elevationSlopeAt(ro, sOut) * (tg.end->atEnd ? -1.0 : 1.0);
            c.elevation.push_back(odr::Poly3::hermite(0.0, L, z0, m0, z1, m1));
          }
          odr::Connection cn;
          cn.id = std::to_string(connIdx++);
          cn.incomingRoad = ri.id;
          cn.connectingRoad = c.id;
          cn.contactPoint = "start";
          cn.laneLinks.push_back({li, cid});
          J.connections.push_back(cn);
          connecting.push_back(std::move(c));
          ++rep.connectingRoads;
        }
      }
    }
    // spec 5 (road.linkage.both_sides_consistency): a road links to the junction only if the
    // junction connects it (as incoming road or as the target of a connecting road)
    std::set<std::string> used;
    for (const auto &cn : J.connections) used.insert(cn.incomingRoad);
    for (const auto &c : connecting)
      if (c.junction == J.id && c.successor) used.insert(c.successor->elementId);
    for (const RoadEnd &e : cl.ends) {
      odr::Road &r = od.roads[metas[e.road].roadIndex];
      if (!used.count(r.id)) (e.atEnd ? r.successor : r.predecessor).reset();
    }
    od.junctions.push_back(std::move(J));
  }
  for (auto &c : connecting) od.roads.push_back(std::move(c));
}

void Builder::buildSignals() {
  const CountryProfile &C = opt.country;
  const TargetProfile &T = opt.target;
  const bool LHT = C.leftHandTraffic;
  auto signCode = [&](const std::string &key) {
    if (T.openDriveSignCatalog) {
      if (key == "stop") return SignCode{"OpenDRIVE", "206", "-1", 0.6, 0.6};
      if (key == "give_way") return SignCode{"OpenDRIVE", "205", "-1", 0.6, 0.6};
      if (key == "maxspeed") return SignCode{"OpenDRIVE", "274", "-1", 0.6, 0.6};
      if (key == "traffic_light") return SignCode{"OpenDRIVE", "1000001", "-1", 0.4, 1.0};
    }
    auto it = C.signs.find(key);
    if (it != C.signs.end()) return it->second;
    return SignCode{"OpenDRIVE", key == "traffic_light" ? "1000001" : "-1", "-1", 0.6, 0.6};
  };
  // signal at the approach of a road end: on the curb right of the incoming traffic
  auto placeAtEnd = [&](odr::Road &r, bool atEnd, const std::vector<int> &lanes, double back, odr::Signal &sg) {
    const double s = atEnd ? std::max(0.0, r.length() - back) : std::min(r.length(), back);
    int outer = lanes.front();
    for (int id : lanes)
      if (std::abs(id) > std::abs(outer)) outer = id;
    double ti, to;
    odr::laneBorders(r, s, outer, ti, to);
    // beyond any bike / parking lane of that side, up to the sidewalk
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    for (const auto &l : outer > 0 ? sec.left : sec.right) {
      if (std::abs(l.id) <= std::abs(outer) || l.type == "sidewalk") continue;
      double a, b;
      if (odr::laneBorders(r, s, l.id, a, b)) to = b;
    }
    sg.s = s;
    sg.t = to + (outer > 0 ? 0.6 : -0.6);
    const bool plus = (outer < 0) != LHT;
    sg.orientation = plus ? "+" : "-";
    // spec 7.1: <validity fromLane toLane> with fromLane <= toLane (numerically)
    const auto [mn, mx] = std::minmax_element(lanes.begin(), lanes.end());
    sg.validity.push_back({*mn, *mx});
  };

  // traffic lights near (within 40 m of) a cluster also signalise it
  for (int ci = 0; ci < static_cast<int>(chains.size()); ++ci) {
    const Chain &ch = chains[ci];
    if (ch.internal) continue;
    for (const RoadMeta &m : metas) {
      if (m.chain != ci) continue;
      for (const auto &[nid, s] : m.nodeS) {
        const OsmNode *nd = osm.node(nid);
        if (!nd || !tagIs(nd->tags, "highway", "traffic_signals")) continue;
        if (ch.c1 >= 0 && m.full.length() - s < 40.0) clusters[ch.c1].signalised = true;
        if (ch.c0 >= 0 && s < 40.0) clusters[ch.c0].signalised = true;
      }
    }
  }

  for (Cluster &cl : clusters) {
    if (!cl.signalised || cl.junction < 0) continue;
    odr::Junction &J = od.junctions[cl.junction];
    // phases: opposite approaches share a controller
    struct Approach {
      const RoadEnd *end;
      double heading;
      std::vector<int> lanes;
      int group = -1;
    };
    std::vector<Approach> ap;
    for (const RoadEnd &e : cl.ends) {
      odr::Road &r = od.roads[metas[e.road].roadIndex];
      auto lanes = endLanes(r, e.atEnd, true);
      if (lanes.empty()) continue;
      const Pose p = r.planView.eval(e.atEnd ? r.length() : 0.0);
      ap.push_back({&e, e.atEnd ? p.hdg : p.hdg + kPi, lanes, -1});
    }
    int groups = 0;
    for (size_t i = 0; i < ap.size(); ++i) {
      if (ap[i].group >= 0) continue;
      ap[i].group = groups;
      for (size_t j = i + 1; j < ap.size(); ++j)
        if (ap[j].group < 0 && std::abs(std::abs(normalizeAngle(ap[j].heading - ap[i].heading)) - kPi) < 0.6) {
          ap[j].group = groups;
          break;
        }
      ++groups;
    }
    std::vector<odr::Controller> ctrls(groups);
    for (int g = 0; g < groups; ++g) {
      ctrls[g].id = std::to_string(nextControllerId++);
      ctrls[g].name = "junction" + J.id + "_phase" + std::to_string(g);
      J.controllers.push_back({ctrls[g].id, g});
    }
    for (const Approach &a : ap) {
      odr::Road &r = od.roads[metas[a.end->road].roadIndex];
      odr::Signal sg;
      const SignCode code = signCode("traffic_light");
      sg.id = std::to_string(nextSignalId++);
      sg.name = "TrafficLight";
      sg.dynamic = true;
      sg.country = code.country;
      sg.type = code.type;
      sg.subtype = code.subtype;
      sg.width = code.width;
      sg.height = code.height;
      sg.zOffset = T.trafficLightZeroOffset ? 0.0 : 4.5;
      placeAtEnd(r, a.end->atEnd, a.lanes, 0.2, sg);
      ctrls[a.group].signalIds.push_back(sg.id);
      r.signals.push_back(sg);
      ++rep.signals;
    }
    for (auto &c : ctrls) od.controllers.push_back(c);
    rep.controllers += groups;
  }

  // stop / give-way signs (node direction tags, all-way stops, minor approaches)
  for (const auto &[nid, nd] : osm.nodes) {
    const std::string hw = tag(nd.tags, "highway").value_or("");
    if (hw != "stop" && hw != "give_way") continue;
    const SignCode code = signCode(hw);
    std::vector<std::pair<int, bool>> targets;  // (meta index, atEnd)
    if (auto it = clusterOf.find(nid); it != clusterOf.end()) {
      const Cluster &cl = clusters[it->second];
      const bool allWay = tagIs(nd.tags, "stop", "all");
      int minRank = 99;
      for (const RoadEnd &e : cl.ends) minRank = std::min(minRank, metas[e.road].rank);
      for (const RoadEnd &e : cl.ends)
        if (allWay || metas[e.road].rank == minRank) targets.push_back({e.road, e.atEnd});
    } else {
      for (int mi = 0; mi < static_cast<int>(metas.size()); ++mi) {
        const RoadMeta &m = metas[mi];
        for (const auto &[n2, s] : m.nodeS) {
          if (n2 != nid) continue;
          const Chain &ch = chains[m.chain];
          const std::string dir = tag(nd.tags, "direction").value_or("");
          bool toEnd;
          if (dir == "forward" || dir == "backward") {
            // direction is relative to the OSM way; find whether this part runs along it
            bool partFwd = true;
            for (const Part &p : ch.parts)
              if (std::count(segs[p.seg].nodes.begin(), segs[p.seg].nodes.end(), nid)) partFwd = p.fwd;
            toEnd = (dir == "forward") == partFwd;
          } else {
            toEnd = (m.full.length() - s) < s;  // nearest junction
          }
          if ((toEnd ? ch.c1 : ch.c0) >= 0) targets.push_back({mi, toEnd});
        }
      }
    }
    for (const auto &[mi, atEnd] : targets) {
      odr::Road &r = od.roads[metas[mi].roadIndex];
      auto lanes = endLanes(r, atEnd, true);
      if (lanes.empty()) continue;
      odr::Signal sg;
      sg.id = std::to_string(nextSignalId++);
      sg.name = hw == "stop" ? "Stop" : "GiveWay";
      sg.country = code.country;
      sg.type = code.type;
      sg.subtype = code.subtype;
      sg.width = code.width;
      sg.height = code.height;
      sg.zOffset = T.openDriveSignCatalog ? 0.0 : 2.0;
      placeAtEnd(r, atEnd, lanes, 0.5, sg);
      r.signals.push_back(sg);
      ++rep.signals;
    }
  }

  // speed-limit signs where the limit changes along the direction of travel (or a road starts)
  if (!opt.speedSigns) return;
  const SignCode code = signCode("maxspeed");
  for (RoadMeta &m : metas) {
    odr::Road &r = od.roads[m.roadIndex];
    if (r.length() < 30.0) continue;
    for (bool plus : {true, false}) {
      // lanes travelling +s / -s at their starting end
      const bool startAtEnd = !plus;  // -s traffic starts at the road end
      auto lanes = endLanes(r, startAtEnd, false);
      if (lanes.empty()) continue;
      const auto *lane = odr::findLane(odr::sectionAt(r, startAtEnd ? r.length() : 0.0), lanes.front());
      if (!lane || !lane->speedMax) continue;
      const double v = *lane->speedMax;
      // upstream speed: the roads feeding this one through its junction
      const int c = startAtEnd ? chains[m.chain].c1 : chains[m.chain].c0;
      bool differs = c < 0;
      if (c >= 0)
        for (const RoadEnd &e : clusters[c].ends) {
          if (e.road == static_cast<int>(&m - &metas[0])) continue;
          const odr::Road &u = od.roads[metas[e.road].roadIndex];
          for (int id : endLanes(u, e.atEnd, true))
            if (const auto *ul = odr::findLane(odr::sectionAt(u, e.atEnd ? u.length() : 0.0), id))
              if (ul->speedMax && std::abs(*ul->speedMax - v) > 0.5) differs = true;
        }
      if (!differs) continue;
      odr::Signal sg;
      sg.id = std::to_string(nextSignalId++);
      sg.name = "SpeedLimit";
      sg.country = code.country;
      sg.type = code.type;
      sg.width = code.width;
      sg.height = code.height;
      sg.zOffset = T.openDriveSignCatalog ? 0.0 : 2.0;
      if (T.speedSignKmhBuckets) {
        const int kmh = static_cast<int>(std::clamp(std::round(v * 3.6 / 10.0) * 10.0, 30.0, 120.0));
        sg.subtype = std::to_string(kmh);
        sg.value = kmh;
        sg.unit = "km/h";
      } else if (C.speedUnit == "mph") {
        sg.value = std::round(v / 0.44704);
        sg.unit = "mph";
      } else {
        sg.value = std::round(v * 3.6);
        sg.unit = "km/h";
      }
      // a few metres after the road start in the direction of travel
      const double back = std::min(8.0, r.length() / 3);
      placeAtEnd(r, startAtEnd, lanes, back, sg);
      sg.s = startAtEnd ? r.length() - back : back;
      r.signals.push_back(sg);
      ++rep.signals;
    }
  }
}

void Builder::buildObjects() {
  const int rev = opt.target.revMinor;
  // spatial index of road reference lines (5 m samples)
  struct Sample {
    int road;
    double s;
  };
  const double cell = 25.0;
  std::map<std::pair<int, int>, std::vector<Sample>> grid;
  for (int i = 0; i < static_cast<int>(metas.size()); ++i) {
    const odr::Road &r = od.roads[metas[i].roadIndex];
    for (double s = 0; s <= r.length(); s += 5.0) {
      const Pose p = r.planView.eval(s);
      grid[{static_cast<int>(std::floor(p.x / cell)), static_cast<int>(std::floor(p.y / cell))}].push_back({i, s});
    }
  }
  auto nearestRoad = [&](const Vec2 &q, double maxDist, int &mi, double &s, double &t) {
    double best = maxDist;
    mi = -1;
    const int cx = static_cast<int>(std::floor(q.x / cell)), cy = static_cast<int>(std::floor(q.y / cell));
    std::set<int> cand;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        if (auto it = grid.find({cx + dx, cy + dy}); it != grid.end())
          for (const auto &smp : it->second) cand.insert(smp.road);
    for (int c : cand) {
      const odr::Road &r = od.roads[metas[c].roadIndex];
      double ss, tt;
      r.planView.project(q, ss, tt);
      const Pose p = r.planView.eval(ss);
      const double d = (q - p.pos()).norm();
      if (d < best && std::abs((q - p.pos()).dot(p.dir())) < 1.0) best = d, mi = c, s = ss, t = tt;
    }
    return mi >= 0;
  };
  auto carriageway = [&](const odr::Road &r, double s, double &lo, double &hi) {
    const odr::LaneSection &sec = odr::sectionAt(r, s);
    lo = hi = odr::laneOffsetAt(r, s);
    double a, b;
    for (const auto &l : sec.left)
      if (l.type != "sidewalk" && odr::laneBorders(r, s, l.id, a, b)) hi = std::max(hi, b);
    for (const auto &l : sec.right)
      if (l.type != "sidewalk" && odr::laneBorders(r, s, l.id, a, b)) lo = std::min(lo, b);
  };
  auto crosswalk = [&](odr::Road &r, double s) {
    double lo, hi;
    carriageway(r, s, lo, hi);
    odr::Object o;
    o.id = std::to_string(nextObjectId++);
    o.name = "Crosswalk";
    o.type = "crosswalk";
    o.s = s;
    o.t = 0.0;
    o.length = 3.0;
    o.width = hi - lo;
    o.orientation = "none";
    o.outline = {{-1.5, lo, 0, 0}, {1.5, lo, 0, 0}, {1.5, hi, 0, 0}, {-1.5, hi, 0, 0}};
    r.objects.push_back(o);
    ++rep.objects;
  };
  // mapped crossings on roads
  for (const auto &[nid, nd] : osm.nodes) {
    if (!tagIs(nd.tags, "highway", "crossing") || clusterOf.count(nid)) continue;
    for (RoadMeta &m : metas)
      for (const auto &[n2, sf] : m.nodeS)
        if (n2 == nid) {
          odr::Road &r = od.roads[m.roadIndex];
          const double s = std::clamp(sf - m.trimStart, 1.5, std::max(1.5, r.length() - 1.5));
          crosswalk(r, s);
        }
  }
  // crosswalks on every approach of signalised / crossing junctions
  if (opt.crosswalksAtSignals)
    for (const Cluster &cl : clusters) {
      if (!(cl.signalised || cl.crossing)) continue;
      for (const RoadEnd &e : cl.ends) {
        odr::Road &r = od.roads[metas[e.road].roadIndex];
        if (r.length() < 6.0) continue;
        crosswalk(r, e.atEnd ? r.length() - 1.6 : 1.6);
      }
    }
  if (!opt.streetFurniture) return;
  for (const auto &[nid, nd] : osm.nodes) {
    const bool lamp = tagIs(nd.tags, "highway", "street_lamp");
    const bool tree = tagIs(nd.tags, "natural", "tree");
    if (!lamp && !tree) continue;
    int mi;
    double s, t;
    if (!nearestRoad(pos(nid), lamp ? 25.0 : 15.0, mi, s, t)) continue;
    odr::Road &r = od.roads[metas[mi].roadIndex];
    odr::Object o;
    o.id = std::to_string(nextObjectId++);
    o.s = s;
    o.t = t;
    if (lamp) {
      o.name = "StreetLamp";
      o.type = rev >= 8 ? "pole" : "streetLamp";
      o.subtype = rev >= 8 ? "streetLamp" : "";
      o.height = 9.0;
      o.radius = 0.15;
    } else {
      o.name = "Tree";
      o.type = "tree";
      o.height = 8.0;
      o.radius = 2.0;
    }
    r.objects.push_back(o);
    ++rep.objects;
  }
}

void Builder::finish() {
  // header: extent, georeference, date
  double minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
  for (const auto &r : od.roads)
    for (const auto &e : r.planView.elements)
      for (double f : {0.0, 0.5, 1.0}) {
        const Pose p = e.eval(e.length * f);
        minX = std::min(minX, p.x), maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y), maxY = std::max(maxY, p.y);
      }
  od.header.name = opt.name;
  od.header.north = maxY;
  od.header.south = minY;
  od.header.east = maxX;
  od.header.west = minX;
  od.header.geoReference = proj.projString();
  if (!opt.verticalDatum.empty()) od.header.userData["verticalDatum"] = opt.verticalDatum;
  od.header.revMinor = opt.target.revMinor;
  char buf[64];
  const std::time_t now = std::time(nullptr);
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::gmtime(&now));
  od.header.date = buf;
  rep.roads = static_cast<int>(metas.size());
  if (opt.terrain && !opt.terrain->ground.empty()) {
    std::vector<double> dev;
    for (const RoadMeta &m : metas) {
      if (m.roadIndex < 0) continue;
      const odr::Road &r = od.roads[m.roadIndex];
      for (double s = 1.0; s < r.length() - 1.0; s += 5.0) {
        bool structure = false;
        for (const auto *rg : {&m.bridges, &m.tunnels})
          for (const auto &[a, bb] : *rg) structure |= s + m.trimStart >= a - 5 && s + m.trimStart <= bb + 5;
        if (structure) continue;
        const odr::LaneSection &sec = odr::sectionAt(r, s);
        for (const auto *side : {&sec.left, &sec.right})
          for (const auto &l : *side) {
            if (l.type != "driving") continue;
            Pose c;
            double ta, tb;
            if (!odr::laneCenter(r, s, l.id, c) || !odr::laneBorders(r, s, l.id, ta, tb)) continue;
            const double g = opt.terrain->ground_at(c.x, c.y);
            if (!std::isnan(g)) dev.push_back(std::abs(odr::surfaceZ(r, s, 0.5 * (ta + tb)) - g));
          }
      }
    }
    // steep spots: real (the measured ground is as steep) or artefacts of the profile fit
    for (const RoadMeta &m : metas) {
      if (m.roadIndex < 0) continue;
      const odr::Road &r = od.roads[m.roadIndex];
      for (double s = 5.0; s < r.length() - 5.0; s += 5.0) {
        const double g = std::abs(odr::elevationSlopeAt(r, s));
        if (g <= 0.20) continue;
        ++rep.steepSpots;
        const Pose a = r.planView.eval(s - 5.0), b = r.planView.eval(s + 5.0);
        const double za = opt.terrain->ground_at(a.x, a.y), zb = opt.terrain->ground_at(b.x, b.y);
        bool structure = false;
        for (const auto *rg : {&m.bridges, &m.tunnels})
          for (const auto &[x0, x1] : *rg) structure |= s + m.trimStart >= x0 - 10 && s + m.trimStart <= x1 + 10;
        if (structure || std::isnan(za) || std::isnan(zb) || g - std::abs(zb - za) / 10.0 > 0.03) {
          ++rep.steepNotInTerrain;
          if (std::getenv("ODRFORGE_DEBUG_GRADE"))
            std::fprintf(stderr, "steep road %s s=%.0f grade %.3f terrain %.3f\n", r.id.c_str(), s, g,
                         std::abs(zb - za) / 10.0);
        }
      }
    }
    if (!dev.empty()) {
      std::sort(dev.begin(), dev.end());
      double sum = 0;
      for (double d : dev) sum += d;
      rep.elevMeanDev = sum / dev.size();
      rep.elevP95Dev = dev[static_cast<size_t>(0.95 * (dev.size() - 1))];
      rep.elevMaxDev = dev.back();
      rep.elevSamples = static_cast<int>(dev.size());
    }
  }
  if (!fitDev.empty()) {
    std::sort(fitDev.begin(), fitDev.end());
    double sum = 0;
    for (double d : fitDev) sum += d;
    rep.fitMeanDev = sum / fitDev.size();
    rep.fitP95Dev = fitDev[static_cast<size_t>(0.95 * (fitDev.size() - 1))];
    rep.fitMaxDev = fitDev.back();
  }
  rep.junctions = static_cast<int>(od.junctions.size());
  for (const auto &r : od.roads)
    for (const auto &e : r.planView.elements) {
      ++rep.geometryElements;
      switch (e.type()) {
        case GeomType::Line: ++rep.lines; break;
        case GeomType::Arc: ++rep.arcs; break;
        case GeomType::Spiral: ++rep.spirals; break;
        default: break;
      }
    }
}

}  // namespace detail
}  // namespace odrforge
