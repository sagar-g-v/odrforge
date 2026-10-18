// OdrForge - junction reference line and elevation grid (standard profile, OpenDRIVE 1.8+).
//
// OpenDRIVE 1.8.0 §12.9 junction reference line: a <planView> with one <geometry> holding one <line>
// (rule junctions.geometry.only_one_line_element), placed "in a way that every point of the junction can be
// reached with a perpendicular straight line" (junctions.geometry.ref_line_definition).
// §12.11 elevation grid: rows perpendicular to that line (junctions.elevation_grid.perpendicular_vectors) at
// sStart + k gridSpacing, each with a center value and left / right values from inside to outside, made of
// complete squares; heights inside are bicubically interpolated (§12.11.1), and the grid "shall override any
// elevation values derived from any roads that are part of the junction".
// The junction's surface here is the plane the connecting roads already lie on (elevation.cpp
// junctionSurfaces): the grid samples that plane, and bicubic interpolation of planar values is exact
// (fitted edge cubics are linear), so a reader of the grid gets the connecting roads' own heights. Line and
// grid cover the connecting roads' lanes out to their outer borders and the §12.10 junction boundary (below; no
// transition zones: the incoming roads already meet the plane C1).
// Common junctions with a plane only (1.8.0 §12.11: "currently only for common junctions").

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <map>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {
constexpr double kGridSpacing = 5.0;  // m; the plane needs no finer grid (Junction Guideline §6.5 example: 5 m)
constexpr double kSample = 1.0;       // m, along the connecting roads
constexpr double kMargin = 0.5;       // m, around the sampled extent

// lateral position of the outer edge of a road's left (side > 0) or right side at s; the centre lane where that side
// has no lanes
double sideEdge(const odr::Road &r, double s, int side) {
  double t = odr::laneOffsetAt(r, s);
  const odr::LaneSection &sec = odr::sectionAt(r, s);
  for (const odr::Lane &l : side > 0 ? sec.left : sec.right) {
    double in, out;
    if (odr::laneBorders(r, s, l.id, in, out)) t = side > 0 ? std::max(t, out) : std::min(t, out);
  }
  return t;
}

// the cubic through f[k] at s0 + k h / 3 (k = 0..3), as a record from s0 (Newton forward differences)
odr::Poly3 cubicThrough(double s0, double h, const double f[4]) {
  const double d1 = f[1] - f[0], d2 = f[2] - 2.0 * f[1] + f[0], d3 = f[3] - 3.0 * f[2] + 3.0 * f[1] - f[0];
  const double k = 3.0 / h;
  odr::Poly3 p;
  p.s = s0;
  p.a = f[0];
  p.b = (d1 - d2 / 2.0 + d3 / 3.0) * k;
  p.c = (d2 / 2.0 - d3 / 2.0) * k * k;
  p.d = d3 / 6.0 * k * k * k;
  return p;
}

// exact laneOffset records for t(s) = edge(s) + shift(s), both cubic between consecutive breakpoints; the edge is the
// outer edge of the given side (lane 0) or of one lane (empty when that lane is missing somewhere)
std::vector<odr::Poly3> offsetRecords(const odr::Road &r, int side, const odr::Poly3 &shift, int lane = 0) {
  std::vector<double> bp{0.0, r.length()};
  for (const odr::Poly3 &p : r.laneOffset) bp.push_back(p.s);
  for (const odr::LaneSection &sec : r.lanes) {
    bp.push_back(sec.s);
    for (const odr::Lane &l : side > 0 ? sec.left : sec.right)
      for (const odr::Poly3 &w : l.widths) bp.push_back(sec.s + w.s);
  }
  std::sort(bp.begin(), bp.end());
  std::vector<odr::Poly3> out;
  for (size_t i = 0; i + 1 < bp.size(); ++i) {
    const double s0 = bp[i], h = std::min(bp[i + 1], r.length()) - s0;
    if (s0 < 0.0 || h < 1e-6) continue;
    double f[4];
    for (int k = 0; k < 4; ++k) {
      const double s = k < 3 ? s0 + k * h / 3.0 : s0 + h - 1e-9;  // the interval's own section at its end
      double in, edge = sideEdge(r, s, side);
      if (lane != 0 && !odr::laneBorders(r, s, lane, in, edge)) return {};
      f[k] = edge + shift.eval(s);
    }
    out.push_back(cubicThrough(s0, h, f));
  }
  return out;
}

// even-odd rule
bool insidePolygon(const std::vector<Vec2> &P, const Vec2 &p) {
  bool in = false;
  for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++)
    if ((P[i].y > p.y) != (P[j].y > p.y) && p.x < P[i].x + (p.y - P[i].y) * (P[j].x - P[i].x) / (P[j].y - P[i].y)) in = !in;
  return in;
}

double distanceToPolygon(const std::vector<Vec2> &P, const Vec2 &p) {
  double best = 1e300;
  for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) {
    const Vec2 ab = P[i] - P[j];
    const double u = std::clamp((p - P[j]).dot(ab) / std::max(ab.dot(ab), 1e-12), 0.0, 1.0);
    best = std::min(best, (P[j] + ab * u - p).norm());
  }
  return best;
}

// a boundary helper road pushed out past points beyond its line: its laneOffset on a 1 m grid, moved outward (right of
// its course around the junction, left when the boundary runs it from end to start) to kPushMargin past each point
// within kPushReach along it, ends kept (they meet the next segments); cubic Hermite records between grid points,
// slopes by central differences
void pushOut(odr::Road &h, bool reversed, const std::vector<Vec2> &pts) {
  constexpr double kPushMargin = 0.15, kPushReach = 1.5;  // m
  const double L = h.length();
  const int n = std::max(2, static_cast<int>(std::ceil(L)) + 1);
  std::vector<double> s(n), t(n);
  for (int i = 0; i < n; ++i) s[i] = L * i / (n - 1), t[i] = odr::laneOffsetAt(h, s[i]);
  for (const Vec2 &p : pts) {
    double sp, tp;
    h.planView.project(p, sp, tp);
    if (sp <= 0.0 || sp >= L) continue;
    for (int i = 1; i + 1 < n; ++i)
      if (std::abs(s[i] - sp) <= kPushReach) t[i] = reversed ? std::max(t[i], tp + kPushMargin) : std::min(t[i], tp - kPushMargin);
  }
  std::vector<double> m(n);
  for (int i = 0; i < n; ++i) m[i] = (t[std::min(i + 1, n - 1)] - t[std::max(i - 1, 0)]) / (s[std::min(i + 1, n - 1)] - s[std::max(i - 1, 0)]);
  h.laneOffset.clear();
  for (int i = 0; i + 1 < n; ++i) h.laneOffset.push_back(odr::Poly3::hermite(s[i], s[i + 1] - s[i], t[i], m[i], t[i + 1], m[i + 1]));
}

// the centre lane line of a boundary helper folds or nearly so: at lateral offset t from a reference line of curvature
// kappa it runs at 1 - kappa t times the line's speed, reversing (a cusp) where an offset towards the centre of a turn
// passes its radius; it must keep kMinStretch of it. A tight reference line itself (t = 0) does not fold.
bool folds(const odr::Road &h) {
  constexpr double kMinStretch = 0.1, kStep = 0.25;  // -, m
  const int n = std::max(2, static_cast<int>(std::ceil(h.length() / kStep)) + 1);
  for (int i = 0; i < n; ++i) {
    const double s = h.length() * i / (n - 1);
    if (1.0 - h.planView.eval(s).kappa * odr::laneOffsetAt(h, s) < kMinStretch) return true;
  }
  return false;
}

// two edges that are not neighbours cross
bool selfIntersecting(const std::vector<Vec2> &P) {
  const auto cross = [](const Vec2 &o, const Vec2 &a, const Vec2 &b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); };
  const size_t n = P.size();
  for (size_t i = 0; i < n; ++i)
    for (size_t k = i + 2; k < n; ++k) {
      if (i == 0 && k == n - 1) continue;
      const Vec2 &a = P[i], &b = P[(i + 1) % n], &c = P[k], &d = P[(k + 1) % n];
      if (cross(a, b, c) * cross(a, b, d) < 0.0 && cross(c, d, a) * cross(c, d, b) < 0.0) return true;
    }
  return false;
}
}  // namespace

// OpenDRIVE 1.8.0 §12.10 junction boundary: "Segments shall be ordered counter clockwise", "shall be defined to reach
// the start or end of all roads connected to the junction", "shall close the entire junction boundary"; "If the
// existing roads are not sufficient to define a closed junction boundary, additional roads shall be defined for the
// missing segments", following road linkage, "not required to have any lanes or connections", their @junction the
// junction's id. Here: the arms (plain road ends at the junction) in counter-clockwise order; across each arm's end a
// joint segment (all lanes); from an arm's left edge (looking out along it) to the next arm's right edge, the outer edge
// of a lane of a junction road that runs exactly there (kEdgeTol) - a corner sidewalk, a right-turn road - else a
// helper road (a centre lane only; boundaryLane 0, the centre lane's line): along the outer edge of the connecting
// road between the two arms, shifted out to their edges, or where no connecting road joins them, along a corner curve
// between the two edges on the junction plane. "Start and end of an additional road shall point away from both
// incoming and/or outgoing roads to the inside of the junction": a helper starts and ends like the connecting road it
// follows, a corner curve leaves the arm inward and joins the next one outward. The outline is then checked (below)
// and widened where lanes lie beyond a helper or a junction road's edge; one that still fails is not written.
// sStart / sEnd are written "start" / "end" (XSD 1.8.1 t_grEqZeroOrContactPoint: t_grZero or e_contactPoint).
void Builder::junctionBoundaries() {
  if (!opt.target.spec().hasJunctionGrid()) return;
  constexpr double kEdgeTol = 0.10;      // m
  constexpr double kOutlineStep = 0.25;  // m, the outline's points along lanes (finer than pushOut's 1 m grid)
  constexpr double kLaneStep = 0.5;      // m, lane borders checked against the outline
  constexpr double kMinHelper = 1.0;     // m, shortest helper road (invariant SIM-5: esmini / CARLA mishandle shorter roads;
                                         // our own floor, OpenDRIVE only requires length > 0)
  long nextRoad = 0;
  for (const odr::Road &r : od.roads) try {
      nextRoad = std::max(nextRoad, std::stol(r.id));
    } catch (...) {
    }
  std::map<std::string, std::vector<size_t>> roadsOf;  // junction id -> its roads (crossing roads of cross paths aside)
  for (size_t i = 0; i < od.roads.size(); ++i)
    if (od.roads[i].junction != "-1" && !od.roads[i].lanes.empty() && od.roads[i].name != "Crossing") roadsOf[od.roads[i].junction].push_back(i);
  std::vector<odr::Road> helpers;  // appended after the loop
  for (const Cluster &cl : clusters) {
    if (cl.junction < 0 || !cl.hasPlane) continue;
    odr::Junction &J = od.junctions[cl.junction];
    if (J.type != "default" || !roadsOf.count(J.id)) continue;
    const long firstHelper = nextRoad;
    // the arms: their end's outer edges, right and left looking outward
    struct Arm {
      std::string road, contact;
      Vec2 mid, right, left, out;
    };
    std::vector<Arm> arms;
    for (const odr::Road &r : od.roads) {
      if (r.junction != "-1" || r.lanes.empty()) continue;
      for (const bool atEnd : {false, true}) {
        const auto &lk = atEnd ? r.successor : r.predecessor;
        if (!lk || lk->elementType != "junction" || lk->elementId != J.id) continue;
        const double s = atEnd ? r.length() : 0.0;
        const odr::LaneSection &sec = odr::sectionAt(r, s);
        double lo = odr::laneOffsetAt(r, s), hi = lo;
        for (const auto *side : {&sec.left, &sec.right})
          for (const odr::Lane &l : *side) {
            double a, b;
            if (odr::laneBorders(r, s, l.id, a, b)) lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
          }
        const Pose p = r.planView.eval(s);
        const Vec2 out = p.dir() * (atEnd ? -1.0 : 1.0);
        Arm a{r.id, atEnd ? "end" : "start", p.offset(0.5 * (lo + hi)), p.offset(lo), p.offset(hi), out};
        // right of the outward direction: the lower t when leaving along +s (start), the higher at the end
        if (atEnd) std::swap(a.right, a.left);
        arms.push_back(a);
      }
    }
    if (arms.size() < 2) continue;
    Vec2 c{0, 0};
    for (const Arm &a : arms) c = c + a.mid;
    c = c * (1.0 / arms.size());
    std::sort(arms.begin(), arms.end(), [&](const Arm &a, const Arm &b) {
      return std::atan2(a.mid.y - c.y, a.mid.x - c.x) < std::atan2(b.mid.y - c.y, b.mid.x - c.x);
    });
    // the outer edge of every lane of the junction's roads at both ends
    struct Edge {
      size_t road;
      int lane;
      Vec2 p0, p1;
    };
    std::vector<Edge> edges;
    for (size_t i : roadsOf[J.id]) {
      const odr::Road &r = od.roads[i];
      for (const auto *side : {&r.lanes.front().left, &r.lanes.front().right})
        for (const odr::Lane &l : *side) {
          double a0, b0, a1, b1;
          if (!odr::laneBorders(r, 0.0, l.id, a0, b0) || !odr::laneBorders(r, r.length(), l.id, a1, b1)) continue;
          edges.push_back({i, l.id, r.planView.eval(0.0).offset(b0), r.planView.eval(r.length()).offset(b1)});
        }
    }
    // the outline's pieces: a joint across an arm's end, or the outer edge of a lane of a junction road (a helper's
    // centre line) from one end of its road to the other - in a traced outline (below) also a part of a lane edge and a
    // joint across some lanes only
    struct Piece {
      int arm = -1;                     // joint
      const odr::Road *road = nullptr;  // a junction road's lane edge
      int helper = -1;                  // a helper road (index in mine)
      int lane = 0;
      bool reversed = false;
      double s0 = -1.0, s1 = -1.0;      // the lane edge from s0 to s1 only (when s0 >= 0)
      bool partJoint = false;           // the joint across some lanes only: from j0 to j1
      Vec2 j0, j1;
    };
    std::vector<odr::BoundarySegment> outline;
    std::vector<Piece> pieces;
    std::vector<odr::Road> mine;
    bool ok = true;
    // a helper road on a junction road's reference line (links, elevation and superelevation copied: the boundary
    // lies on that road's surface), its centre lane along the given laneOffset
    const auto helperAlong = [&](const odr::Road &via, std::vector<odr::Poly3> offset) {
      odr::Road h;
      h.id = std::to_string(++nextRoad);
      h.name = "JunctionBoundary";
      h.junction = J.id;
      h.planView = via.planView;
      h.predecessor = via.predecessor;
      h.successor = via.successor;
      h.types = {odr::Road::Type{0.0, "town", "", std::nullopt, "km/h"}};
      h.elevation = via.elevation;
      h.superelevation = via.superelevation;
      h.laneOffset = std::move(offset);
      odr::LaneSection sec;
      sec.center.id = 0;
      sec.center.type = "none";
      h.lanes = {sec};
      return h;
    };
    // a lane segment's sStart / sEnd: "start" / "end" at the road's ends, else the number
    const auto sText = [](const odr::Road &r, double s) {
      return s <= 1e-9 ? std::string("start") : s >= r.length() - 1e-9 ? std::string("end") : std::to_string(s);
    };
    for (size_t k = 0; k < arms.size() && ok; ++k) {
      const Arm &a = arms[k], &b = arms[(k + 1) % arms.size()];
      outline.push_back({"joint", a.road, "", "", a.contact, 0});
      pieces.push_back({static_cast<int>(k)});
      // the corner from a's left edge to b's right edge
      const Edge *found = nullptr;
      bool reversed = false;
      for (const Edge &e : edges) {
        if ((e.p0 - a.left).norm() < kEdgeTol && (e.p1 - b.right).norm() < kEdgeTol) found = &e, reversed = false;
        else if ((e.p1 - a.left).norm() < kEdgeTol && (e.p0 - b.right).norm() < kEdgeTol) found = &e, reversed = true;
        if (found) break;
      }
      if (found) {
        const odr::Road &r = od.roads[found->road];
        outline.push_back({"lane", r.id, reversed ? "end" : "start", reversed ? "start" : "end", "", found->lane});
        pieces.push_back({-1, &r, -1, found->lane, reversed});
        continue;
      }
      if ((b.right - a.left).norm() < kEdgeTol) continue;  // the two arms' edges meet
      // the connecting road between a and b whose side edge lies nearest both corners: the helper takes its
      // reference line and runs along that edge, shifted out to the arms' edges at its ends (a cubic shift, level at
      // both ends), so the boundary encloses the connecting road ("Junction boundaries enclose the area intended for
      // traffic"); its heights are that road's (elevation and superelevation copied)
      const odr::Road *via = nullptr;
      int viaSide = 0;
      bool viaReversed = false;
      double viaShift0 = 0.0, viaShift1 = 0.0, best = 1e300;
      const auto linksTo = [](const std::optional<odr::LinkTarget> &lk, const Arm &arm) {
        return lk && lk->elementType == "road" && lk->elementId == arm.road && lk->contactPoint == arm.contact;
      };
      for (size_t i : roadsOf[J.id]) {
        const odr::Road &r = od.roads[i];
        const bool fwd = linksTo(r.predecessor, a) && linksTo(r.successor, b);
        if (!fwd && !(linksTo(r.predecessor, b) && linksTo(r.successor, a))) continue;
        const double sa = fwd ? 0.0 : r.length(), sb = fwd ? r.length() : 0.0;
        const Pose ra = r.planView.eval(sa), rb = r.planView.eval(sb);
        const Vec2 na{-std::sin(ra.hdg), std::cos(ra.hdg)}, nb{-std::sin(rb.hdg), std::cos(rb.hdg)};
        const Vec2 da = a.left - ra.pos(), db = b.right - rb.pos();
        if (std::abs(da.dot(ra.dir())) > kEdgeTol || std::abs(db.dot(rb.dir())) > kEdgeTol) continue;  // not at the arm ends
        for (const int side : {1, -1}) {
          const double shiftA = da.dot(na) - sideEdge(r, sa, side), shiftB = db.dot(nb) - sideEdge(r, sb, side);
          if (std::abs(shiftA) + std::abs(shiftB) >= best) continue;
          best = std::abs(shiftA) + std::abs(shiftB);
          via = &r, viaSide = side, viaReversed = !fwd;
          viaShift0 = fwd ? shiftA : shiftB, viaShift1 = fwd ? shiftB : shiftA;
        }
      }
      if (via) {
        odr::Road h = helperAlong(*via, offsetRecords(*via, viaSide, odr::Poly3::hermite(0.0, via->length(), viaShift0, 0.0, viaShift1, 0.0)));
        if (!folds(h)) {
          outline.push_back({"lane", h.id, viaReversed ? "end" : "start", viaReversed ? "start" : "end", "", 0});
          pieces.push_back({-1, nullptr, static_cast<int>(mine.size()), 0, viaReversed});
          mine.push_back(std::move(h));
          continue;
        }
        --nextRoad;  // its edge shifted past the turn's centre: the corner curve instead
      }
      // no connecting road between a and b: a corner curve between their edges, on the junction plane
      Pose pa, pb;
      pa.x = a.left.x, pa.y = a.left.y, pa.hdg = std::atan2(-a.out.y, -a.out.x);  // into the junction
      pb.x = b.right.x, pb.y = b.right.y, pb.hdg = std::atan2(b.out.y, b.out.x);  // out along b
      const ReferenceLine curve = cornerCurve(pa, pb, opt.cornerRadius);
      // a curve too tight to follow (radius under the outline's step) turns on the spot: a kink, not a corner
      double kMax = 0.0;
      for (const GeomElement &g : curve.elements) kMax = std::max({kMax, std::abs(g.k0), std::abs(g.k1)});
      if (curve.elements.empty() || curve.length() < 0.05 || (curve.eval(curve.length()).pos() - b.right).norm() > kEdgeTol ||
          kMax * kOutlineStep > 1.0) {
        ok = false;
        break;
      }
      odr::Road h;
      h.id = std::to_string(++nextRoad);
      h.name = "JunctionBoundary";
      h.junction = J.id;
      h.planView = curve;
      h.predecessor = odr::LinkTarget{"road", a.road, a.contact};
      h.successor = odr::LinkTarget{"road", b.road, b.contact};
      h.types = {odr::Road::Type{0.0, "town", "", std::nullopt, "km/h"}};
      odr::LaneSection sec;
      sec.center.id = 0;
      sec.center.type = "none";
      h.lanes = {sec};
      connectingElevation(h, cl, std::nullopt);
      outline.push_back({"lane", h.id, "start", "end", "", 0});
      pieces.push_back({-1, nullptr, static_cast<int>(mine.size()), 0, false});
      mine.push_back(std::move(h));
    }
    // the outline as a polygon (kOutlineStep along lanes), each point with its piece; consecutive pieces meet within
    // kEdgeTol and share the later one's start (two points that close would cross by a hair). False when it is not
    // closed, not counter-clockwise or crosses itself, or a helper folds or is shorter than kMinHelper.
    std::vector<Vec2> poly;
    std::vector<size_t> owner;  // piece of the edge from poly[i] to poly[i + 1]
    const auto trace = [&](double step) {  // lane edges sampled every `step` (kOutlineStep)
      poly.clear(), owner.clear();
      for (size_t k = 0; k < pieces.size(); ++k) {
        const Piece &pc = pieces[k];
        std::vector<Vec2> pts;
        if (pc.partJoint) {
          pts = {pc.j0, pc.j1};
        } else if (pc.arm >= 0) {
          pts = {arms[pc.arm].right, arms[pc.arm].left};
        } else {
          const odr::Road &r = pc.helper >= 0 ? mine[pc.helper] : *pc.road;
          if (pc.helper >= 0 && (folds(r) || r.length() < kMinHelper)) return false;
          const bool part = pc.s0 >= 0.0;
          const double s0 = part ? pc.s0 : pc.reversed ? r.length() : 0.0, s1 = part ? pc.s1 : r.length() - s0;
          const int n = std::max(2, static_cast<int>(std::ceil(std::abs(s1 - s0) / step)) + 1);
          for (int i = 0; i < n; ++i) {
            const double s = s0 + (s1 - s0) * i / (n - 1);
            double in = 0.0, t = odr::laneOffsetAt(r, s);
            if (pc.lane != 0 && !odr::laneBorders(r, s, pc.lane, in, t)) return false;
            pts.push_back(r.planView.eval(s).offset(t));
          }
        }
        if (!poly.empty()) {
          if ((pts.front() - poly.back()).norm() > kEdgeTol) return false;
          poly.pop_back(), owner.pop_back();
        }
        poly.insert(poly.end(), pts.begin(), pts.end());
        owner.insert(owner.end(), pts.size(), k);
      }
      if (poly.size() < 4 || (poly.front() - poly.back()).norm() > kEdgeTol) return false;
      poly.pop_back(), owner.pop_back();  // the closing corner is the first point
      double area = 0.0;
      for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) area += poly[j].x * poly[i].y - poly[i].x * poly[j].y;
      return area > 0.0 && !selfIntersecting(poly);
    };
    // lane borders of the junction's roads outside the polygon, by the piece of the nearest edge; false when one is
    // nearest a joint
    const auto outside = [&](std::map<size_t, std::vector<Vec2>> &byPiece) {
      for (const odr::Road &r : od.roads) {
        if (r.junction != J.id || r.lanes.empty()) continue;
        const int n = std::max(2, static_cast<int>(std::ceil(r.length() / kLaneStep)) + 1);
        for (int i = 0; i < n; ++i) {
          const double s = r.length() * i / (n - 1);
          const Pose p = r.planView.eval(s);
          const odr::LaneSection &sec = odr::sectionAt(r, s);
          for (const auto *side : {&sec.left, &sec.right})
            for (const odr::Lane &l : *side) {
              double in, out;
              if (!odr::laneBorders(r, s, l.id, in, out)) continue;
              for (const double t : {in, out}) {
                const Vec2 q = p.offset(t);
                if (insidePolygon(poly, q) || distanceToPolygon(poly, q) <= kEdgeTol) continue;
                size_t nearest = 0;
                double best = 1e300;
                for (size_t e = 0; e < poly.size(); ++e) {
                  const Vec2 &u = poly[e], ab = poly[(e + 1) % poly.size()] - u;
                  const double f = std::clamp((q - u).dot(ab) / std::max(ab.dot(ab), 1e-12), 0.0, 1.0), d = (u + ab * f - q).norm();
                  if (d < best) best = d, nearest = e;
                }
                if (pieces[owner[nearest]].arm >= 0) {
                  if (std::getenv("ODRFORGE_DEBUG_BOUNDARY"))
                    std::fprintf(stderr, "boundary J%s: road %s (%s) lane %d s %.2f t %.2f outside at arm joint, %.3f m\n", J.id.c_str(),
                                 r.id.c_str(), r.name.c_str(), l.id, s, t, best);
                  return false;
                }
                byPiece[owner[nearest]].push_back(q);
              }
            }
        }
      }
      return true;
    };
    // The fallback where the outline above fails (a corner curve between arms side by side turns back across their
    // ends, lanes lie beyond a corner): the outer contour of the union of every lane of the junction's roads - "the
    // outermost edge of the full area" (§12.10) - traced on a kRes raster and put back on what it follows: the outer
    // border of a lane, or a road's centre line (boundaryLane 0), of a junction road over an s range ("segments that run
    // along the outmost edge of a given lane"), or an arm's end across the lanes the junction's roads continue (a joint
    // from jointLaneStart to jointLaneEnd, Table 78; the spec's own example has both). Parts of existing roads only: no
    // additional road. False when the contour leaves every lane edge, a joint's end is no lane border of its arm, or an
    // arm gets no joint (the reason in `why`, printed with ODRFORGE_DEBUG_BOUNDARY).
    std::string why;
    const auto fail = [&](const char *reason) {
      why = reason;
      return false;
    };
    const auto unionOutline = [&]() {
      constexpr double kRes = 0.05;   // m, raster cell: the contour lies within one cell of the lanes' edges
      constexpr double kSnap = 0.15;  // m, a contour point on a lane edge or an end line
      outline.clear(), pieces.clear();
      struct Feature {
        const odr::Road *road = nullptr;  // a border of a junction road; or a junction road's end (atEnd >= 0), an arm's
        int lane = 0, arm = -1, atEnd = -1;  // end line (road null): arm >= 0
        std::vector<double> s;            // along a border: s; across an end: t
        std::vector<Vec2> p;
      };
      std::vector<Feature> feats;
      std::vector<std::array<Vec2, 3>> tris;
      std::map<std::pair<std::string, std::string>, int> armOf;
      for (size_t k = 0; k < arms.size(); ++k) armOf[{arms[k].road, arms[k].contact}] = static_cast<int>(k);
      for (const odr::Road &r : od.roads) {
        if (r.junction != J.id || r.lanes.empty()) continue;
        for (size_t k = 0; k < r.lanes.size(); ++k) {  // per lane section: each lane's outer border and the centre line
          const double s0 = r.lanes[k].s, s1 = k + 1 < r.lanes.size() ? r.lanes[k + 1].s : r.length();
          if (s1 - s0 < 1e-6) continue;
          const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / kOutlineStep)));
          std::vector<int> ids{0};
          for (const auto *side : {&r.lanes[k].left, &r.lanes[k].right})
            for (const odr::Lane &l : *side) ids.push_back(l.id);
          std::vector<Feature> fs(ids.size());
          std::vector<std::vector<Vec2>> inner(ids.size());
          for (int i = 0; i <= n; ++i) {
            const double s = std::min(s0 + (s1 - s0) * i / n, s1 - 1e-6);  // inside this section
            const Pose p = r.planView.eval(s);
            for (size_t m = 0; m < ids.size(); ++m) {
              double in = 0.0, out = odr::laneOffsetAt(r, s);
              if (ids[m] == 0 || !odr::laneBorders(r, s, ids[m], in, out)) in = out;
              fs[m].road = &r, fs[m].lane = ids[m];
              fs[m].s.push_back(s), fs[m].p.push_back(p.offset(out)), inner[m].push_back(p.offset(in));
            }
          }
          for (size_t m = 1; m < ids.size(); ++m)
            for (int i = 1; i <= n; ++i) {
              const Vec2 &ia = inner[m][i - 1], &oa = fs[m].p[i - 1], &ib = inner[m][i], &ob = fs[m].p[i];
              if ((oa - ia).norm() < 1e-3 && (ob - ib).norm() < 1e-3) continue;
              tris.push_back({ia, oa, ob}), tris.push_back({ia, ob, ib});
            }
          for (Feature &f : fs) feats.push_back(std::move(f));
        }
        for (const bool atEnd : {false, true}) {  // its two ends, across all its lanes; on an arm's end line when it meets one
          const double s = atEnd ? r.length() - 1e-6 : 0.0;
          const odr::LaneSection &sec = odr::sectionAt(r, s);
          double lo = odr::laneOffsetAt(r, s), hi = lo;
          for (const auto *side : {&sec.left, &sec.right})
            for (const odr::Lane &l : *side) {
              double a0, b0;
              if (odr::laneBorders(r, s, l.id, a0, b0)) lo = std::min({lo, a0, b0}), hi = std::max({hi, a0, b0});
            }
          if (hi - lo < 1e-3) continue;
          const Pose p = r.planView.eval(s);
          Feature f;
          f.road = &r, f.atEnd = atEnd;
          f.s = {lo, hi}, f.p = {p.offset(lo), p.offset(hi)};
          const auto &lk = atEnd ? r.successor : r.predecessor;
          if (lk && lk->elementType == "road") {
            const auto it = armOf.find({lk->elementId, lk->contactPoint});
            if (it != armOf.end()) f.arm = it->second;
          }
          feats.push_back(std::move(f));
        }
      }
      for (size_t k = 0; k < arms.size(); ++k) {  // every arm's end line, right to left
        Feature f;
        f.arm = static_cast<int>(k);
        f.s = {0.0, 1.0}, f.p = {arms[k].right, arms[k].left};
        feats.push_back(std::move(f));
      }
      if (tris.empty()) return fail("no lanes");
      // the raster: cells whose centre lies in a lane
      double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
      for (const auto &t : tris)
        for (const Vec2 &v : t) x0 = std::min(x0, v.x), y0 = std::min(y0, v.y), x1 = std::max(x1, v.x), y1 = std::max(y1, v.y);
      x0 -= 1.0, y0 -= 1.0;
      const int nx = static_cast<int>((x1 - x0) / kRes) + 40, ny = static_cast<int>((y1 - y0) / kRes) + 40;
      if (static_cast<double>(nx) * ny > 1.0e8) return fail("over 500 m across");
      std::vector<uint8_t> in(static_cast<size_t>(nx) * ny, 0);
      for (const auto &t : tris) {
        const double ty0 = std::min({t[0].y, t[1].y, t[2].y}), ty1 = std::max({t[0].y, t[1].y, t[2].y});
        const int j0 = std::max(0, static_cast<int>(std::ceil((ty0 - y0) / kRes - 0.5)));
        const int j1 = std::min(ny - 1, static_cast<int>(std::floor((ty1 - y0) / kRes - 0.5)));
        for (int j = j0; j <= j1; ++j) {
          const double y = y0 + (j + 0.5) * kRes;
          double xa = 1e300, xb = -1e300;
          for (int e = 0; e < 3; ++e) {
            const Vec2 &p = t[e], &q = t[(e + 1) % 3];
            if ((p.y - y) * (q.y - y) > 0.0 || p.y == q.y) continue;
            const double x = p.x + (q.x - p.x) * (y - p.y) / (q.y - p.y);
            xa = std::min(xa, x), xb = std::max(xb, x);
          }
          const int i0 = std::max(0, static_cast<int>(std::ceil((xa - x0) / kRes - 0.5)));
          const int i1 = std::min(nx - 1, static_cast<int>(std::floor((xb - x0) / kRes - 0.5)));
          for (int i = i0; i <= i1; ++i) in[static_cast<size_t>(j) * nx + i] = 1;
        }
      }
      const auto filled = [&](int i, int j) { return i >= 0 && j >= 0 && i < nx && j < ny && in[static_cast<size_t>(j) * nx + i]; };
      // the outer contour along cell edges, counter-clockwise (the lanes on the left), from the lowest-leftmost cell's
      // lower-left corner (no other cell touches it, so it is passed once)
      int si = -1, sj = -1;
      for (int j = 0; j < ny && si < 0; ++j)
        for (int i = 0; i < nx; ++i)
          if (in[static_cast<size_t>(j) * nx + i]) {
            si = i, sj = j;
            break;
          }
      if (si < 0) return fail("no lanes");
      std::vector<Vec2> contour;
      {
        int cx = si, cy = sj, d = 0;  // 0 east, 1 north, 2 west, 3 south
        const int dx[4] = {1, 0, -1, 0}, dy[4] = {0, 1, 0, -1};
        do {
          contour.push_back({x0 + cx * kRes, y0 + cy * kRes});
          bool left, right;  // the two cells ahead
          switch (d) {
            case 0: left = filled(cx, cy), right = filled(cx, cy - 1); break;
            case 1: left = filled(cx - 1, cy), right = filled(cx, cy); break;
            case 2: left = filled(cx - 1, cy - 1), right = filled(cx - 1, cy); break;
            default: left = filled(cx, cy - 1), right = filled(cx - 1, cy - 1); break;
          }
          if (right) d = (d + 3) % 4;  // the lanes go on to the right: turn right; none ahead: turn left
          else if (!left) d = (d + 1) % 4;
          cx += dx[d], cy += dy[d];
        } while ((cx != si || cy != sj) && contour.size() < static_cast<size_t>(nx) * ny);
      }
      // every contour point on the feature it follows, within kSnap: a border at s, an end at t. The feature it was on
      // keeps it while it lies within kSnap (no flicker between two features running together), else the nearest
      std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> grid;  // 2 m cells -> (feature, segment)
      for (size_t f = 0; f < feats.size(); ++f)
        for (size_t k = 1; k < feats[f].p.size(); ++k) {
          const Vec2 &a = feats[f].p[k - 1], &b = feats[f].p[k];
          for (int gx = static_cast<int>(std::floor(std::min(a.x, b.x) / 2.0)); gx <= static_cast<int>(std::floor(std::max(a.x, b.x) / 2.0)); ++gx)
            for (int gy = static_cast<int>(std::floor(std::min(a.y, b.y) / 2.0)); gy <= static_cast<int>(std::floor(std::max(a.y, b.y) / 2.0)); ++gy)
              grid[{gx, gy}].push_back({static_cast<int>(f), static_cast<int>(k)});
        }
      // an arm's end line and the junction roads' ends on it are one feature: the arm's joint
      const auto key = [&](int f) { return feats[f].arm >= 0 && (feats[f].atEnd >= 0 || !feats[f].road) ? -1 - feats[f].arm : f; };
      struct Hit {
        int f = -1;
        double u = 0.0;  // s or t
      };
      const auto nearest = [&](const Vec2 &v, int only) {  // the nearest feature (of key `only` unless -INT_MAX)
        Hit h;
        double best = kSnap;
        for (int gx = static_cast<int>(std::floor(v.x / 2.0)) - 1; gx <= static_cast<int>(std::floor(v.x / 2.0)) + 1; ++gx)
          for (int gy = static_cast<int>(std::floor(v.y / 2.0)) - 1; gy <= static_cast<int>(std::floor(v.y / 2.0)) + 1; ++gy) {
            const auto it = grid.find({gx, gy});
            if (it == grid.end()) continue;
            for (const auto &[f, k] : it->second) {
              if (only != -INT_MAX && key(f) != only) continue;
              const Vec2 &a = feats[f].p[k - 1], ab = feats[f].p[k] - a;
              const double w = std::clamp((v - a).dot(ab) / std::max(ab.dot(ab), 1e-12), 0.0, 1.0), dist = (a + ab * w - v).norm();
              if (dist < best) best = dist, h = {f, feats[f].s[k - 1] + w * (feats[f].s[k] - feats[f].s[k - 1])};
            }
          }
        return h;
      };
      std::vector<Hit> hits(contour.size());
      for (size_t c = 0; c < contour.size(); ++c) {
        if (c > 0) hits[c] = nearest(contour[c], key(hits[c - 1].f));
        if (hits[c].f < 0) hits[c] = nearest(contour[c], -INT_MAX);
        if (hits[c].f < 0) return fail("the contour leaves every lane edge");
      }
      // runs of one feature; stretches under 3 points are the corners between two features
      struct Run {
        int key, f0, f1;
        double u0, u1;
        size_t n, c0, c1;  // points, first and last contour index
        Vec2 e0{}, e1{};   // its ends
      };
      std::vector<Run> runs;
      for (size_t c = 0; c < contour.size(); ++c) {
        const int kk = key(hits[c].f);
        if (!runs.empty() && runs.back().key == kk) runs.back().f1 = hits[c].f, runs.back().u1 = hits[c].u, ++runs.back().n, runs.back().c1 = c;
        else runs.push_back({kk, hits[c].f, hits[c].f, hits[c].u, hits[c].u, 1, c, c});
      }
      const auto join = [](Run &a, const Run &b) { a.f1 = b.f1, a.u1 = b.u1, a.n += b.n, a.c1 = b.c1; };
      if (runs.size() > 1 && runs.front().key == runs.back().key) {  // the cycle's first run continues its last
        join(runs.back(), runs.front());
        runs.erase(runs.begin());
      }
      std::vector<Run> kept;
      for (const Run &u : runs)
        if (u.n >= 3) {
          if (!kept.empty() && kept.back().key == u.key) join(kept.back(), u);
          else kept.push_back(u);
        }
      if (kept.size() > 1 && kept.front().key == kept.back().key) {
        join(kept.back(), kept.front());
        kept.erase(kept.begin());
      }
      if (kept.size() < 2) return fail("one feature only");
      // where one run hands over to the next both meet exactly (the contour finds it to a cell): two borders where
      // they cross (where they come closest when they only touch: a border going on in the next lane section); a
      // border and an end line at the border's own road end when that lies on the line (a junction road ends at an
      // arm), else where it crosses the line
      for (Run &u : kept) u.e0 = contour[u.c0], u.e1 = contour[u.c1];
      const auto isEnd = [&](const Run &u) { return u.key < 0 || feats[u.f0].atEnd >= 0; };
      const auto endLine = [&](const Run &u) -> std::pair<Vec2, Vec2> {
        if (u.key < 0) return {arms[-1 - u.key].right, arms[-1 - u.key].left};
        return {feats[u.f0].p[0], feats[u.f0].p[1]};
      };
      // the crossing of polylines P and Q (s per point) within 1.5 m of `near`: s on each and the point - the nearest; or,
      // with a run direction (dir = +-1 along P's s from `from`), the first the run reaches: two edges crossing twice
      // within a cell (a sliver the raster cannot see) hand over at the first, else the run doubles back in a loop
      const auto cross = [](const std::vector<Vec2> &P, const std::vector<double> &Ps, const std::vector<Vec2> &Q,
                            const std::vector<double> &Qs, const Vec2 &near, double &sp, double &sq, Vec2 &at, double from = 0.0,
                            double dir = 0.0) {
        double best = 1e300;
        bool found = false;
        for (size_t i = 1; i < P.size(); ++i) {
          if ((P[i] - near).norm() > 4.0 && (P[i - 1] - near).norm() > 4.0) continue;
          const Vec2 p = P[i - 1], dp = P[i] - p;
          for (size_t j = 1; j < Q.size(); ++j) {
            const Vec2 q = Q[j - 1], dq = Q[j] - q;
            const double den = dp.cross(dq);
            if (std::abs(den) < 1e-12) continue;
            const double a = (q - p).cross(dq) / den, b = (q - p).cross(dp) / den;
            if (a < -1e-9 || a > 1.0 + 1e-9 || b < -1e-9 || b > 1.0 + 1e-9) continue;
            const double d = (p + dp * a - near).norm(), s = Ps[i - 1] + a * (Ps[i] - Ps[i - 1]);
            if (d > 1.5) continue;
            const double rank = dir == 0.0 ? d : (s - from) * dir;
            if (dir != 0.0 && rank < 1e-6) continue;  // not ahead of the run's start
            if (rank < best) best = rank, found = true, sp = s, sq = Qs[j - 1] + b * (Qs[j] - Qs[j - 1]), at = p + dp * a;
          }
        }
        return found;
      };
      // a border's point at s (the outer border of its lane, the centre line for lane 0)
      const auto edgeAt = [](const Feature &f, double s) {
        double in = 0.0, t = odr::laneOffsetAt(*f.road, s);
        if (f.lane != 0) odr::laneBorders(*f.road, s, f.lane, in, t);
        return f.road->planView.eval(s).offset(t);
      };
      // the crossing of two borders found on their kOutlineStep polylines, moved onto the curves themselves (Newton,
      // differences of 1e-4 m): both pieces end exactly where the edges cross, not a chord's sag apart (which would
      // leave a sliver loop or a gap of millimetres at the join); kept as found when it does not converge nearby
      const auto onCurves = [&](const Feature &P, const Feature &Q, double &sp, double &sq, Vec2 &at) {
        double a = sp, b = sq;
        for (int it = 0; it < 12; ++it) {
          const Vec2 f = edgeAt(P, a) - edgeAt(Q, b);
          if (f.norm() < 1e-7) break;
          constexpr double h = 1e-4;
          const Vec2 da = (edgeAt(P, a + h) - edgeAt(P, a - h)) * (0.5 / h), db = (edgeAt(Q, b + h) - edgeAt(Q, b - h)) * (-0.5 / h);
          const double det = da.cross(db);
          if (std::abs(det) < 1e-9) return;
          a -= (f.cross(db)) / det, b -= (da.cross(f)) / det;
        }
        if (std::abs(a - sp) > 0.5 || std::abs(b - sq) > 0.5 || (edgeAt(P, a) - edgeAt(Q, b)).norm() > 1e-5) return;
        sp = a, sq = b, at = edgeAt(P, a);
      };
      for (size_t k = 0; k < kept.size(); ++k) {
        Run &u = kept[k], &w = kept[(k + 1) % kept.size()];
        const Vec2 near = contour[u.c1];
        double sp = 0.0, sq = 0.0;
        Vec2 at;
        if (!isEnd(u) && !isEnd(w)) {
          const Feature &P = feats[u.f1], &Q = feats[w.f0];
          if (cross(P.p, P.s, Q.p, Q.s, near, sp, sq, at, u.u0, u.u1 >= u.u0 ? 1.0 : -1.0)) {
            onCurves(P, Q, sp, sq, at);
            u.u1 = sp, w.u0 = sq, u.e1 = w.e0 = at;
          } else {
            double best = 1e300;
            for (size_t i = 0; i < P.p.size(); ++i) {
              if ((P.p[i] - near).norm() > 2.0) continue;
              for (size_t j = 1; j < Q.p.size(); ++j) {
                const Vec2 &a = Q.p[j - 1], ab = Q.p[j] - a;
                const double f = std::clamp((P.p[i] - a).dot(ab) / std::max(ab.dot(ab), 1e-12), 0.0, 1.0), d = (a + ab * f - P.p[i]).norm();
                if (d < best) best = d, u.u1 = P.s[i], w.u0 = Q.s[j - 1] + f * (Q.s[j] - Q.s[j - 1]), u.e1 = P.p[i], w.e0 = a + ab * f;
              }
            }
          }
        } else if (!isEnd(u) || !isEnd(w)) {
          const bool out = !isEnd(u);  // the border runs out to the line (else in from it)
          const Feature &P = feats[out ? u.f1 : w.f0];
          double &s = out ? u.u1 : w.u0;
          const double L = P.road->length();
          const double sEnd = std::abs(s) < std::abs(L - s) ? 0.0 : L;
          const auto [l0, l1] = endLine(out ? w : u);
          const Vec2 atEnd = sEnd == 0.0 ? P.p.front() : P.p.back(), ld = l1 - l0;
          const bool ownEnd = (sEnd == 0.0 ? P.s.front() < 1e-3 : P.s.back() > L - 1e-3) && std::abs(s - sEnd) < 1.0;
          if (ownEnd && std::abs(ld.cross(atEnd - l0)) / std::max(ld.norm(), 1e-9) < kSnap) {
            s = sEnd, u.e1 = w.e0 = atEnd;
          } else {
            const Vec2 d = ld * (0.5 / std::max(ld.norm(), 1e-9));  // the line extended half a metre each way
            if (cross(P.p, P.s, {l0 - d, l1 + d}, {0.0, 1.0}, near, sp, sq, at)) s = sp, u.e1 = w.e0 = at;
          }
        }
      }
      // an arm's joint: its ends on lane borders of the arm's end (within 2 kSnap), across the lanes from one to the
      // other (Table 78: first and last lane crossed); all lanes: a plain joint
      const auto jointOf = [&](const odr::Road &r, bool atEnd, double ta, double tb, odr::BoundarySegment &sg, Piece &pc) {
        const double s = atEnd ? r.length() - 1e-6 : 0.0;
        const odr::LaneSection &sec = odr::sectionAt(r, s);
        std::vector<double> bord{odr::laneOffsetAt(r, s)};  // every border: a lane's outer border or the centre line
        for (const auto *side : {&sec.left, &sec.right})
          for (const odr::Lane &l : *side) {
            double a0, b0;
            if (odr::laneBorders(r, s, l.id, a0, b0)) bord.push_back(b0);
          }
        const auto snap = [&](double t) {
          double best = 2.0 * kSnap, bt = std::nan("");
          for (const double b0 : bord)
            if (std::abs(b0 - t) <= best) best = std::abs(b0 - t), bt = b0;
          return bt;
        };
        const double a = snap(ta), b = snap(tb);
        if (std::isnan(a) || std::isnan(b) || std::abs(a - b) < 1e-3) return false;
        int first = 0, last = 0;
        double fa = 1e300, fb = 1e300;
        for (const auto *side : {&sec.left, &sec.right})
          for (const odr::Lane &l : *side) {
            double a0, b0;
            if (!odr::laneBorders(r, s, l.id, a0, b0)) continue;
            const double mid = 0.5 * (a0 + b0);
            if (mid <= std::min(a, b) || mid >= std::max(a, b)) continue;
            if (std::abs(mid - a) < fa) fa = std::abs(mid - a), first = l.id;
            if (std::abs(mid - b) < fb) fb = std::abs(mid - b), last = l.id;
          }
        if (!first || !last) return false;
        const double lo = *std::min_element(bord.begin(), bord.end()), hi = *std::max_element(bord.begin(), bord.end());
        const Pose p = r.planView.eval(s);
        sg = {"joint", r.id, "", "", atEnd ? "end" : "start", 0};
        sg.jointLanes = !(std::abs(std::min(a, b) - lo) < 1e-3 && std::abs(std::max(a, b) - hi) < 1e-3);
        sg.jointLaneStart = first, sg.jointLaneEnd = last;
        pc = {};
        pc.partJoint = true, pc.j0 = p.offset(a), pc.j1 = p.offset(b);
        return true;
      };
      // a border's s within 0.3 m of its road's end: the end. A border left with no length is a corner the contour only
      // touched: dropped, and an arm's end on both sides of it is one joint
      const auto clampEnd = [](const odr::Road &r, double s) { return s < 0.3 ? 0.0 : s > r.length() - 0.3 ? r.length() : s; };
      std::vector<Run> emit;
      for (const Run &u : kept) {
        if (!isEnd(u) && std::abs(clampEnd(*feats[u.f0].road, u.u1) - clampEnd(*feats[u.f0].road, u.u0)) < 1e-3) continue;
        if (!emit.empty() && isEnd(u) && emit.back().key == u.key) emit.back().e1 = u.e1;
        else emit.push_back(u);
      }
      if (emit.size() > 1 && isEnd(emit.front()) && emit.front().key == emit.back().key) {
        emit.back().e1 = emit.front().e1;
        emit.erase(emit.begin());
      }
      // a joint has no direction of its own: readers run it from the end nearer the next segment's start, so no two
      // joints follow each other
      for (size_t k = 0; k < emit.size(); ++k)
        if (isEnd(emit[k]) && isEnd(emit[(k + 1) % emit.size()])) return fail("two joints in a row");
      std::vector<bool> reached(arms.size(), false);
      for (const Run &u : emit) {
        const Feature &f0 = feats[u.f0];
        odr::BoundarySegment sg;
        Piece pc;
        if (isEnd(u)) {  // only an arm's end: "Segments shall be defined to reach the start or end of all roads connected
                         // to the junction" - by the arm's joint
          if (u.key >= 0) return fail("a junction road's end not at an arm");
          const Arm &A = arms[-1 - u.key];
          const auto it = std::find_if(od.roads.begin(), od.roads.end(), [&](const odr::Road &q) { return q.id == A.road; });
          if (it == od.roads.end()) return fail("an arm road missing");
          const bool atEnd = A.contact == "end";
          const Pose p = it->planView.eval(atEnd ? it->length() - 1e-6 : 0.0);
          const Vec2 n{-std::sin(p.hdg), std::cos(p.hdg)};
          if (!jointOf(*it, atEnd, (u.e0 - p.pos()).dot(n), (u.e1 - p.pos()).dot(n), sg, pc))
            return fail("a joint end off its arm's lane borders");
          pc.arm = -1 - u.key;
          reached[pc.arm] = true;
        } else {  // a border over an s range
          const odr::Road &r = *f0.road;
          const double sa = clampEnd(r, u.u0), sb = clampEnd(r, u.u1);
          sg = {"lane", r.id, sText(r, sa), sText(r, sb), "", f0.lane};
          pc = {-1, &r, -1, f0.lane, false, sa, sb};
        }
        outline.push_back(sg), pieces.push_back(pc);
      }
      return std::find(reached.begin(), reached.end(), false) == reached.end() || fail("an arm without a joint");
    };
    // written only when it holds: closed, counter-clockwise, not crossing itself, and around both borders of every
    // lane of the junction's roads ("Junction boundaries enclose the area intended for traffic. This also includes
    // the sidewalks for pedestrians."); a helper road with lane borders beyond it is pushed out past them (up to
    // kPushRounds times). <boundary> is optional (XSD 1.8.1 minOccurs 0): a junction whose outline still fails - arms
    // whose end lines overlap, lanes beyond an arm's joint or a junction road's edge - is traced along its lanes'
    // contour instead (unionOutline), and gets none when that fails too.
    constexpr int kPushRounds = 4;
    bool enclosed = false;
    for (int round = 0; ok && !enclosed && round <= kPushRounds; ++round) {
      std::map<size_t, std::vector<Vec2>> byPiece;
      ok = trace(kOutlineStep) && outside(byPiece);
      enclosed = ok && byPiece.empty();
      if (round < kPushRounds)
        for (const auto &[k, pts] : byPiece) {
          Piece &pc = pieces[k];
          if (pc.road) {  // a junction road's lane edge is not sufficient: a helper along it, pushed out
            std::vector<odr::Poly3> edge = offsetRecords(*pc.road, pc.lane, odr::Poly3{}, pc.lane);
            if (edge.empty()) {
              ok = false;
              break;
            }
            mine.push_back(helperAlong(*pc.road, std::move(edge)));
            outline[k] = {"lane", mine.back().id, pc.reversed ? "end" : "start", pc.reversed ? "start" : "end", "", 0};
            pc = {-1, nullptr, static_cast<int>(mine.size()) - 1, 0, pc.reversed};
          }
          pushOut(mine[pc.helper], pc.reversed, pts);
        }
    }
    if (!ok || !enclosed) {  // the lanes' contour instead, held to the same checks with every lane inside
      nextRoad = firstHelper;
      mine.clear();
      std::map<size_t, std::vector<Vec2>> byPiece;
      // (also sampled twice as coarsely: a reader stepping 0.5 m along its lane segments sees it closed and not crossing)
      why.clear();
      ok = unionOutline() && (trace(2.0 * kOutlineStep) || fail("open or crossing itself at 0.5 m steps")) &&
           (trace(kOutlineStep) || fail("open or crossing itself")) &&
           ((outside(byPiece) && byPiece.empty()) || fail("lanes outside"));
      // and robust to a reader's own sampling: no lane piece runs against its road's direction (a cusp of an inner
      // border), and parts of the outline more than 2 m apart along it stay a raster cell (0.05 m) apart - where they
      // only touch, a reader sampling more coarsely sees them cross
      for (size_t k = 0; ok && k < pieces.size(); ++k) {
        const Piece &pc = pieces[k];
        if (pc.s0 < 0.0 || !pc.road) continue;
        const odr::Road &r = *pc.road;
        const int n = std::max(2, static_cast<int>(std::ceil(std::abs(pc.s1 - pc.s0) / kOutlineStep)) + 1);
        Vec2 prev;
        for (int i = 0; ok && i < n; ++i) {
          const double s = pc.s0 + (pc.s1 - pc.s0) * i / (n - 1);
          double in = 0.0, t = odr::laneOffsetAt(r, s);
          if (pc.lane != 0) odr::laneBorders(r, s, pc.lane, in, t);
          const Pose p = r.planView.eval(s);
          const Vec2 q = p.offset(t);
          if (i > 0 && (q - prev).dot(p.dir()) * (pc.s1 - pc.s0) < 0.0) ok = fail("a lane edge against its road");
          prev = q;
        }
      }
      // no loop where two lane pieces meet (boundary_audit.py `simple` checks the same): the two sampled every 0.02 m
      // over their whole length, and every 1 mm over 0.2 m round the join, do not cross each other away from the join -
      // two edges crossing twice within centimetres make a sliver the 0.25 m outline steps over
      const auto sampled = [&](const Piece &pc, double step, double near) {  // near > 0: its last, < 0: its first metres
        const odr::Road &r = *pc.road;
        double a = pc.s0, b = pc.s1;
        if (near != 0.0) {
          const double dir = b >= a ? 1.0 : -1.0, n = std::min(std::abs(near), std::abs(b - a));
          a = near > 0.0 ? b - dir * n : a, b = near > 0.0 ? b : a + dir * n;
        }
        const int n = std::max(2, static_cast<int>(std::ceil(std::abs(b - a) / step)) + 1);
        std::vector<Vec2> pts;
        for (int i = 0; i < n; ++i) {
          const double s = a + (b - a) * i / (n - 1);
          double in = 0.0, t = odr::laneOffsetAt(r, s);
          if (pc.lane != 0) odr::laneBorders(r, s, pc.lane, in, t);
          pts.push_back(r.planView.eval(s).offset(t));
        }
        return pts;
      };
      const auto X = [](const Vec2 &o, const Vec2 &a, const Vec2 &b) { return (a - o).cross(b - o); };
      for (size_t k = 0; ok && k < 2 * pieces.size(); ++k) {
        const Piece &pa = pieces[k % pieces.size()], &pb = pieces[(k + 1) % pieces.size()];
        if (pa.s0 < 0.0 || pb.s0 < 0.0 || !pa.road || !pb.road) continue;  // lane pieces only
        const bool micro = k >= pieces.size();
        std::vector<Vec2> A = micro ? sampled(pa, 0.001, 0.2) : sampled(pa, 0.02, 0.0);
        const std::vector<Vec2> Bh = micro ? sampled(pb, 0.001, -0.2) : sampled(pb, 0.02, 0.0);
        const Vec2 join = A.back();
        A.push_back(Bh.front());  // through the join to the next piece's start
        double lx = -1e300, ly = -1e300, hx = 1e300, hy = 1e300;  // where both can be: their bounding boxes' overlap
        double ax0 = 1e300, ay0 = 1e300, ax1 = -1e300, ay1 = -1e300, bx0 = 1e300, by0 = 1e300, bx1 = -1e300, by1 = -1e300;
        for (const Vec2 &p : A) ax0 = std::min(ax0, p.x), ay0 = std::min(ay0, p.y), ax1 = std::max(ax1, p.x), ay1 = std::max(ay1, p.y);
        for (const Vec2 &p : Bh) bx0 = std::min(bx0, p.x), by0 = std::min(by0, p.y), bx1 = std::max(bx1, p.x), by1 = std::max(by1, p.y);
        lx = std::max(ax0, bx0) - 0.1, ly = std::max(ay0, by0) - 0.1, hx = std::min(ax1, bx1) + 0.1, hy = std::min(ay1, by1) + 0.1;
        const auto inBox = [&](const Vec2 &p, const Vec2 &q) {
          return std::min(p.x, q.x) <= hx && std::max(p.x, q.x) >= lx && std::min(p.y, q.y) <= hy && std::max(p.y, q.y) >= ly;
        };
        for (size_t i = 1; ok && i < A.size(); ++i) {
          if (!inBox(A[i - 1], A[i])) continue;
          for (size_t j = 1; ok && j < Bh.size(); ++j) {
            if (!inBox(Bh[j - 1], Bh[j])) continue;
            const double oa0 = X(Bh[j - 1], Bh[j], A[i - 1]), oa1 = X(Bh[j - 1], Bh[j], A[i]);
            if (!(X(A[i - 1], A[i], Bh[j - 1]) * X(A[i - 1], A[i], Bh[j]) < 0.0 && oa0 * oa1 < 0.0)) continue;
            const Vec2 at = A[i - 1] + (A[i] - A[i - 1]) * (oa0 / (oa0 - oa1));
            // (0.2 mm: stricter than the audit's 1 mm, so its own sampling of the same edges finds none either)
            if ((at - join).norm() > 2e-4 && (at - Bh.front()).norm() > 2e-4) ok = fail("a loop where two pieces meet");
          }
        }
      }
      std::vector<double> along(poly.size() + 1, 0.0);  // length along the outline to each point
      for (size_t i = 0; i < poly.size(); ++i) along[i + 1] = along[i] + (poly[(i + 1) % poly.size()] - poly[i]).norm();
      for (size_t i = 0; ok && i < poly.size(); ++i) {
        const Vec2 &a = poly[i], ab = poly[(i + 1) % poly.size()] - a;
        for (size_t k = i + 2; ok && k < poly.size(); ++k) {
          const Vec2 &e = poly[k], ed = poly[(k + 1) % poly.size()] - e;
          // a point of one edge against the other: their distance, and how far apart the two points lie along the
          // outline (either way round)
          const auto check = [&](const Vec2 &p, double atP, const Vec2 &o, const Vec2 &d, double atO) {
            const double f = std::clamp((p - o).dot(d) / std::max(d.dot(d), 1e-12), 0.0, 1.0);
            const double way = std::abs(atO + f * d.norm() - atP);
            if ((o + d * f - p).norm() < 0.05 && std::min(way, along.back() - way) >= 2.0) ok = fail("pinched within 0.05 m");
          };
          check(a, along[i], e, ed, along[k]), check(a + ab, along[i + 1], e, ed, along[k]);
          check(e, along[k], a, ab, along[i]), check(e + ed, along[k + 1], a, ab, along[i]);
        }
      }
      if (!ok) {
        if (std::getenv("ODRFORGE_DEBUG_BOUNDARY")) std::fprintf(stderr, "boundary J%s: none, contour: %s\n", J.id.c_str(), why.c_str());
        ++rep.boundariesLeftOut;
        continue;
      }
      ++rep.boundariesTraced;
    }
    J.boundary = outline;
    for (auto &h : mine) helpers.push_back(std::move(h));
    ++rep.junctionBoundaries;
  }
  rep.boundaryHelperRoads += static_cast<int>(helpers.size());
  for (auto &h : helpers) od.roads.push_back(std::move(h));
}

void Builder::junctionGrids() {
  if (!opt.target.spec().hasJunctionGrid()) return;
  std::map<std::string, std::vector<const odr::Road *>> roadsOf;  // junction id -> its connecting roads
  for (const odr::Road &r : od.roads)
    if (r.junction != "-1" && !r.lanes.empty()) roadsOf[r.junction].push_back(&r);
  for (const Cluster &cl : clusters) {
    if (cl.junction < 0 || !cl.hasPlane) continue;
    odr::Junction &J = od.junctions[cl.junction];
    if (J.type != "default" || !roadsOf.count(J.id)) continue;
    // the junction's extent: every lane border of its connecting roads
    std::vector<Vec2> pts;
    for (const odr::Road *r : roadsOf[J.id]) {
      const int n = std::max(2, static_cast<int>(std::ceil(r->length() / kSample)) + 1);
      for (int i = 0; i < n; ++i) {
        const double s = r->length() * i / (n - 1);
        const odr::LaneSection &sec = odr::sectionAt(*r, s);
        const Pose p = r->planView.eval(s);
        pts.push_back(p.offset(odr::laneOffsetAt(*r, s)));
        for (const auto *side : {&sec.left, &sec.right})
          for (const odr::Lane &l : *side) {
            double a, b;
            if (odr::laneBorders(*r, s, l.id, a, b)) pts.push_back(p.offset(b));
          }
      }
    }
    if (pts.size() < 3) continue;
    // the line along the principal axis of that extent, through its centroid
    Vec2 c{0, 0};
    for (const Vec2 &q : pts) c = c + q;
    c = c * (1.0 / pts.size());
    double sxx = 0, syy = 0, sxy = 0;
    for (const Vec2 &q : pts) {
      const Vec2 d = q - c;
      sxx += d.x * d.x, syy += d.y * d.y, sxy += d.x * d.y;
    }
    const double hdg = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const Vec2 dir{std::cos(hdg), std::sin(hdg)}, left{-std::sin(hdg), std::cos(hdg)};
    double s0 = 1e300, s1 = -1e300, tl = 0.0, tr = 0.0;
    for (const Vec2 &q : pts) {
      const Vec2 d = q - c;
      s0 = std::min(s0, d.dot(dir)), s1 = std::max(s1, d.dot(dir));
      tl = std::max(tl, d.dot(left)), tr = std::max(tr, -d.dot(left));
    }
    // a margin beyond the sampled extent: the lanes between the samples stay inside too
    s0 -= kMargin, s1 += kMargin, tl += kMargin, tr += kMargin;
    odr::JunctionGrid g;
    const Vec2 o = c + dir * s0;
    g.x = o.x, g.y = o.y, g.hdg = hdg, g.length = std::max(s1 - s0, 1e-3);
    g.gridSpacing = kGridSpacing;
    g.sStart = 0.0;
    // complete squares covering the extent: rows 0 .. K along the line, columns out to tl / tr
    const int K = std::max(1, static_cast<int>(std::ceil(g.length / kGridSpacing - 1e-9)));
    const int nl = static_cast<int>(std::ceil(tl / kGridSpacing - 1e-9)), nr = static_cast<int>(std::ceil(tr / kGridSpacing - 1e-9));
    for (int k = 0; k <= K; ++k) {
      const Vec2 rowAt = o + dir * (k * kGridSpacing);
      odr::JunctionGrid::Row row;
      row.center = planeZ(cl, rowAt);
      for (int j = 1; j <= nl; ++j) row.left.push_back(planeZ(cl, rowAt + left * (j * kGridSpacing)));
      for (int j = 1; j <= nr; ++j) row.right.push_back(planeZ(cl, rowAt + left * (-j * kGridSpacing)));
      g.rows.push_back(row);
    }
    J.grid = g;
    ++rep.junctionGrids;
  }
}

}  // namespace detail
}  // namespace odrforge
