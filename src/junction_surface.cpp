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
#include <cmath>

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
    // centre line) from one end of its road to the other
    struct Piece {
      int arm = -1;                     // joint
      const odr::Road *road = nullptr;  // a junction road's lane edge
      int helper = -1;                  // a helper road (index in mine)
      int lane = 0;
      bool reversed = false;
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
    const auto trace = [&]() {
      poly.clear(), owner.clear();
      for (size_t k = 0; k < pieces.size(); ++k) {
        const Piece &pc = pieces[k];
        std::vector<Vec2> pts;
        if (pc.arm >= 0) {
          pts = {arms[pc.arm].right, arms[pc.arm].left};
        } else {
          const odr::Road &r = pc.helper >= 0 ? mine[pc.helper] : *pc.road;
          if (pc.helper >= 0 && (folds(r) || r.length() < kMinHelper)) return false;
          const double s0 = pc.reversed ? r.length() : 0.0, s1 = r.length() - s0;
          const int n = std::max(2, static_cast<int>(std::ceil(r.length() / kOutlineStep)) + 1);
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
    // written only when it holds: closed, counter-clockwise, not crossing itself, and around both borders of every
    // lane of the junction's roads ("Junction boundaries enclose the area intended for traffic. This also includes
    // the sidewalks for pedestrians."); a helper road with lane borders beyond it is pushed out past them (up to
    // kPushRounds times). <boundary> is optional (XSD 1.8.1 minOccurs 0): a junction whose outline still fails - arms
    // whose end lines overlap, lanes beyond an arm's joint or a junction road's edge - gets none.
    constexpr int kPushRounds = 4;
    bool enclosed = false;
    for (int round = 0; ok && !enclosed && round <= kPushRounds; ++round) {
      std::map<size_t, std::vector<Vec2>> byPiece;
      ok = trace() && outside(byPiece);
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
    if (!ok || !enclosed) {
      nextRoad = firstHelper;
      ++rep.boundariesLeftOut;
      continue;
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
