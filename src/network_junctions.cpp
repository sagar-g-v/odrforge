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
    // numerically straight pieces (solver noise, curvature ~1e-8) become exact lines: readers evaluate
    // arcs about their centre and LibCarla does it in 32-bit floats (a 10^7 m radius lands on a 1 m
    // grid). Only where the heading change is under 1e-5 rad (the bulge then under 0.1 mm per 20 m)
    for (auto &e : l.elements) {
      const double k = std::max(std::abs(e.k0), std::abs(e.k1));
      if (k > 0 && k * e.length < 1e-5) e.k0 = e.k1 = 0.0;  // heading change negligible (validator 1e-4)
    }
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

namespace {

// Pose on a road's reference line at one end, heading in the direction of travel through the
// junction: along s when arriving at the end or leaving from the start, against s otherwise.
Pose travelPose(const odr::Road &r, bool atEnd, bool arriving) {
  Pose p = r.planView.eval(atEnd ? r.length() : 0.0);
  if (atEnd != arriving) {
    p.hdg = normalizeAngle(p.hdg + kPi);
    p.kappa = -p.kappa;
  }
  return p;
}

// Centre pose of a lane at a road end, heading in the direction of travel.
Pose laneTravelPose(const odr::Road &r, bool atEnd, bool arriving, int laneId) {
  Pose p;
  odr::laneCenter(r, atEnd ? r.length() : 0.0, laneId, p);
  if (atEnd != arriving) p.hdg = normalizeAngle(p.hdg + kPi);
  return p;
}

struct TravelSpan {
  double inner, outer;  ///< lane borders, t to the left of the direction of travel
  double width() const { return std::abs(outer - inner); }
  double center() const { return 0.5 * (inner + outer); }
};

// Lane borders at s seen in the direction of travel (t mirrored when travelling against s).
TravelSpan travelSpanAt(const odr::Road &r, double s, bool alongS, int laneId) {
  double inner = 0.0, outer = 0.0;
  odr::laneBorders(r, s, laneId, inner, outer);
  return alongS ? TravelSpan{inner, outer} : TravelSpan{-inner, -outer};
}

// Lane borders at a road end seen in the direction of travel.
TravelSpan travelSpan(const odr::Road &r, bool atEnd, bool alongS, int laneId) {
  return travelSpanAt(r, atEnd ? r.length() : 0.0, alongS, laneId);
}

// How fast the lane borders move sideways at a road end, per metre travelled through the junction
// (arriving: towards the end; leaving: away from it). Matching it keeps lane edges heading-continuous
// across the link (OpenDRIVE 1.9 Annex D).
TravelSpan travelSpanSlope(const odr::Road &r, bool atEnd, bool alongS, bool arriving, int laneId) {
  const double step = std::min(0.001, 0.5 * r.length());  // m: short enough for the slope at the very end
  const double sEnd = atEnd ? r.length() : 0.0, sInside = atEnd ? sEnd - step : sEnd + step;
  const TravelSpan atJunction = travelSpanAt(r, sEnd, alongS, laneId), inside = travelSpanAt(r, sInside, alongS, laneId);
  const double sign = arriving ? 1.0 : -1.0;
  return {sign * (atJunction.inner - inside.inner) / step, sign * (atJunction.outer - inside.outer) / step};
}

// OpenDRIVE 1.9 §12.4 (lane_change_one_con_road): the lane pairs of one relation that share a
// connecting road - adjacent incoming lanes into adjacent outgoing lanes, inner to outer.
std::vector<std::vector<std::pair<int, int>>> adjacentRuns(std::vector<std::pair<int, int>> pairs) {
  auto rank = [](const std::pair<int, int> &p) { return std::make_pair(std::abs(p.first), std::abs(p.second)); };
  std::sort(pairs.begin(), pairs.end(), [&](const auto &a, const auto &b) { return rank(a) < rank(b); });
  std::vector<std::vector<std::pair<int, int>>> runs;
  for (const auto &p : pairs) {
    const bool continuesRun = !runs.empty() && std::abs(p.first) == std::abs(runs.back().back().first) + 1 &&
                              std::abs(p.second) == std::abs(runs.back().back().second) + 1;
    if (continuesRun) runs.back().push_back(p);
    else runs.push_back({p});
  }
  return runs;
}

// The smoothest standard-geometry path between two poses that a vehicle can drive: it turns no
// more than the headings differ (plus a quarter turn) and is not much longer than the gap. Else
// the chord, when its bends stand for turns no tighter than kMinTurnRadius; else no path.
ReferenceLine drivablePath(const Pose &a, const Pose &b) {
  constexpr double kMinTurnRadius = 5.0;
  ReferenceLine line = connectionPath(a, b);
  double turning = 0.0;
  for (const auto &e : line.elements) turning += 0.5 * (e.k0 + e.k1) * e.length;
  const double chord = (b.pos() - a.pos()).norm(), dh = std::abs(normalizeAngle(b.hdg - a.hdg));
  const bool plausible =
      !line.elements.empty() && std::abs(turning) <= dh + 0.5 * kPi && line.length() <= 4.0 * chord + 10.0;
  if (plausible) return line;
  ReferenceLine chordLine;
  const double hc = std::atan2(b.y - a.y, b.x - a.x);
  const double bend = std::max(std::abs(normalizeAngle(hc - a.hdg)), std::abs(normalizeAngle(hc - b.hdg)));
  if (chord > 1e-3 && bend < 0.5 * kPi && 2.0 * kMinTurnRadius * std::sin(bend) <= chord) {
    GeomElement e;
    e.x = a.x, e.y = a.y, e.hdg = hc, e.length = chord;
    chordLine.elements.push_back(e);
    chordLine.finalize();
  }
  return chordLine;
}

// OpenDRIVE 1.9 §8.3 reference line coordinate system: a lane at offset t from a reference line of
// curvature kappa runs (1 - kappa t) times as fast as the line - squeezed on the inside of a bend,
// folded where kappa t >= 1, stretched on the outside. A frame is fit for the lanes only while
// every lane border stays within half the radius of the line (|kappa t| <= 0.5): the lane path is
// then the line's own shape, neither folded nor drawn out (LibCarla steps lanes by s).
bool frameDistortsLanes(const ReferenceLine &line, const odr::Poly3 &laneOffset, double blockWidth0,
                        double blockWidth1, bool lanesOnRight, double kMaxRelativeOffset) {
  const double L = line.length();
  for (double s = 0.0; s <= L; s += 0.25) {
    const double u = s / std::max(L, 1e-9);
    const double inner = laneOffset.eval(s), width = blockWidth0 + (blockWidth1 - blockWidth0) * u;
    const double outer = lanesOnRight ? inner - width : inner + width;
    const double kappa = line.eval(s).kappa;
    if (std::abs(kappa * inner) > kMaxRelativeOffset || std::abs(kappa * outer) > kMaxRelativeOffset) return true;
  }
  return false;
}

}  // namespace

// OpenDRIVE 1.9 §12.4 connecting roads, ASAM Junction Guideline §6: one connecting road per run of
// adjacent lanes, its reference line continuing the arm reference lines in position and heading
// at both ends, in the direction of travel; the lanes sit beside it through laneOffset (§11.5) and
// widths (§11.7) blending from the incoming to the outgoing lanes. Where that frame would distort
// the lanes (a tight turn far from the arm reference lines; the Guideline then moves the reference
// line) each lane follows its own lane-centred road; a lane whose own road would fold takes the arm
// frame if that one does not fold; failing all, its lane-centred road all the same.
void Builder::connectLanes(const ConnectionEnds &ends, const std::vector<std::pair<int, int>> &pairs,
                           const Cluster &cl, odr::Junction &junction, std::vector<odr::Road> &out, int &nextRoadId) {
  const ReferenceLine armToArm =
      drivablePath(travelPose(*ends.in, ends.inAtEnd, true), travelPose(*ends.out, ends.outAtEnd, false));
  const bool armPathUsable = !armToArm.elements.empty() && armToArm.length() >= 0.05;
  auto onArms = [&](const std::vector<std::pair<int, int>> &lanes, Frame frame) {
    return armPathUsable &&
           addConnectingRoad(ends, armToArm, {0.0, 0.0}, {0.0, 0.0}, lanes, frame, cl, junction, out, nextRoadId);
  };
  for (const auto &run : adjacentRuns(pairs)) {
    if (onArms(run, Frame::RefuseDistortion)) continue;
    for (const auto &[from, to] : run) {
      // the reference line starts on the lane centre, parallel to the arm reference line (Annex D:
      // lane borders meet in position only across parallel cross sections); the lane's sideways
      // drift is carried by laneOffset
      const LaneCentre in{travelSpan(*ends.in, ends.inAtEnd, ends.inAtEnd, from).center(), 0.0};
      const LaneCentre outC{travelSpan(*ends.out, ends.outAtEnd, !ends.outAtEnd, to).center(), 0.0};
      const Pose a = laneTravelPose(*ends.in, ends.inAtEnd, true, from);
      const Pose b = laneTravelPose(*ends.out, ends.outAtEnd, false, to);
      const ReferenceLine laneCentred = drivablePath(a, b);
      const bool laneCentredUsable = !laneCentred.elements.empty() && laneCentred.length() >= 0.05;
      auto alongLane = [&](Frame frame) {
        return laneCentredUsable &&
               addConnectingRoad(ends, laneCentred, in, outC, {{from, to}}, frame, cl, junction, out, nextRoadId);
      };
      if (alongLane(Frame::RefuseFolding) || onArms({{from, to}}, Frame::RefuseFolding)) continue;
      alongLane(Frame::AlwaysBuild);
    }
  }
}

// One connecting road along `line`, whose ends lie at travel-frame offsets refIn / refOut from the
// incoming / outgoing arm reference lines (drifting sideways at refIn.slope / refOut.slope).
// Frame::RefuseDistortion / RefuseFolding add nothing (false) where the frame would distort / fold
// the lanes; Frame::AlwaysBuild adds the road regardless.
bool Builder::addConnectingRoad(const ConnectionEnds &ends, const ReferenceLine &line, LaneCentre refIn,
                                LaneCentre refOut, const std::vector<std::pair<int, int>> &lanes, Frame frame,
                                const Cluster &cl,
                                odr::Junction &junction, std::vector<odr::Road> &out, int &nextRoadId) {
  const bool LHT = opt.country.leftHandTraffic;
  const odr::Road &ri = *ends.in, &ro = *ends.out;
  const double L = line.length();
  // the lane borders leave and meet the arms with the arms' own sideways slope (Annex D: lane
  // edges heading-continuous), measured from this road's reference line
  std::vector<TravelSpan> spanIn, spanOut, slopeIn, slopeOut;
  for (const auto &[from, to] : lanes) {
    spanIn.push_back(travelSpan(ri, ends.inAtEnd, ends.inAtEnd, from));
    spanOut.push_back(travelSpan(ro, ends.outAtEnd, !ends.outAtEnd, to));
    slopeIn.push_back(travelSpanSlope(ri, ends.inAtEnd, ends.inAtEnd, true, from));
    slopeOut.push_back(travelSpanSlope(ro, ends.outAtEnd, !ends.outAtEnd, false, to));
  }
  const odr::Poly3 laneOffset =
      odr::Poly3::hermite(0.0, L, spanIn.front().inner - refIn.t, slopeIn.front().inner - refIn.slope,
                          spanOut.front().inner - refOut.t, slopeOut.front().inner - refOut.slope);
  if (frame != Frame::AlwaysBuild) {
    const double block0 = std::abs(spanIn.back().outer - spanIn.front().inner);
    const double block1 = std::abs(spanOut.back().outer - spanOut.front().inner);
    const double limit = frame == Frame::RefuseDistortion ? 0.5 : 0.9;  // |kappa t|: distorted / about to fold
    if (frameDistortsLanes(line, laneOffset, block0, block1, !LHT, limit)) return false;
  }

  odr::Road c;
  c.id = std::to_string(nextRoadId++);
  c.junction = junction.id;
  if (!ri.rule.empty()) c.rule = ri.rule;
  c.planView = line;
  c.predecessor = odr::LinkTarget{"road", ri.id, ends.inAtEnd ? "end" : "start"};
  c.successor = odr::LinkTarget{"road", ro.id, ends.outAtEnd ? "end" : "start"};
  const odr::Road::Type *type = ri.types.empty() ? nullptr : &ri.types.back();
  c.types.push_back({0.0, type ? type->type : "town", type ? type->country : "", type ? type->speedMax : std::nullopt,
                     type ? type->speedUnit : "km/h"});
  c.laneOffset.push_back(laneOffset);
  odr::LaneSection sec;
  sec.center.id = 0;
  sec.center.type = "none";
  const odr::RoadMark none;
  sec.center.roadMarks.push_back(none);
  odr::Connection cn;
  cn.id = std::to_string(junction.connections.size());
  cn.incomingRoad = ri.id;
  cn.connectingRoad = c.id;
  cn.contactPoint = "start";
  for (size_t k = 0; k < lanes.size(); ++k) {
    odr::Lane l;
    l.id = (LHT ? 1 : -1) * static_cast<int>(k + 1);  // the connecting lanes travel along +s
    l.type = "driving";
    const double sideIn = spanIn[k].outer < spanIn[k].inner ? -1.0 : 1.0;  // the side the width grows to
    const double sideOut = spanOut[k].outer < spanOut[k].inner ? -1.0 : 1.0;
    const double widthSlopeIn = sideIn * (slopeIn[k].outer - slopeIn[k].inner);
    const double widthSlopeOut = sideOut * (slopeOut[k].outer - slopeOut[k].inner);
    l.widths.push_back(
        odr::Poly3::hermite(0.0, L, spanIn[k].width(), widthSlopeIn, spanOut[k].width(), widthSlopeOut));
    l.link.predecessor = lanes[k].first;
    l.link.successor = lanes[k].second;
    l.roadMarks.push_back(none);
    if (const auto *source = odr::findLane(odr::sectionAt(ri, ends.sIn()), lanes[k].first))
      l.speedMax = source->speedMax;
    (LHT ? sec.left : sec.right).push_back(l);
    cn.laneLinks.push_back({lanes[k].first, l.id});
  }
  c.lanes.push_back(sec);
  connectingRoadHeight(c, ends, refIn.t, refOut.t, cl);
  junction.connections.push_back(cn);
  out.push_back(std::move(c));
  ++rep.connectingRoads;
  return true;
}

// Height of a connecting road (OpenDRIVE 1.9 §10.5; Annex D: height continuous at both links).
// On a junction surface it follows the plane; otherwise a C1 Hermite between the arm heights at the
// reference line ends, with the arms' banking carried across so every lane edge meets its arm.
void Builder::connectingRoadHeight(odr::Road &c, const ConnectionEnds &ends, double tRefIn, double tRefOut,
                                   const Cluster &cl) const {
  const odr::Road &ri = *ends.in, &ro = *ends.out;
  const double sIn = ends.sIn(), sOut = ends.sOut(), L = c.length();
  const double armTIn = ends.inAtEnd ? tRefIn : -tRefIn;  // travel-frame offsets back in the arms' own t
  const double armTOut = ends.outAtEnd ? -tRefOut : tRefOut;
  const double z0 = odr::surfaceZ(ri, sIn, armTIn), z1 = odr::surfaceZ(ro, sOut, armTOut);
  if (cl.hasPlane) {
    connectingElevation(c, cl,
                        opt.target.superelevation ? std::nullopt : std::optional<std::pair<double, double>>({z0, z1}));
    return;
  }
  const double m0 = odr::elevationSlopeAt(ri, sIn) * (ends.inAtEnd ? 1.0 : -1.0);
  const double m1 = odr::elevationSlopeAt(ro, sOut) * (ends.outAtEnd ? -1.0 : 1.0);
  c.elevation.push_back(odr::Poly3::hermite(0.0, L, z0, m0, z1, m1));
  const double bank0 = odr::superelevationAt(ri, sIn) * (ends.inAtEnd ? 1.0 : -1.0);
  const double bank1 = odr::superelevationAt(ro, sOut) * (ends.outAtEnd ? -1.0 : 1.0);
  if (bank0 != 0.0 || bank1 != 0.0) c.superelevation.push_back(odr::Poly3::hermite(0.0, L, bank0, 0.0, bank1, 0.0));
}

// The OSM node where a road end meets its junction.
OsmId Builder::attachNode(const RoadEnd &e) const {
  const Chain &ch = chains[metas[e.road].chain];
  return e.atEnd ? ch.n1 : ch.n0;
}

namespace {

// The outermost lane on the right of someone travelling along (alongS) or against s at a road end,
// if it is a sidewalk.
std::optional<int> sidewalkOnRight(const odr::Road &r, bool atEnd, bool alongS) {
  const odr::LaneSection &sec = atEnd ? r.lanes.back() : r.lanes.front();
  const std::vector<odr::Lane> &side = alongS ? sec.right : sec.left;
  if (side.empty() || side.back().type != "sidewalk") return std::nullopt;
  return side.back().id;
}

// OpenDRIVE 1.9 §11.7 lane width; ASAM Junction Guideline §6 (no negative radius on the inner
// sidewalk edge): round a corner the sidewalk lies on the inside of the curb line, so where the curb
// line bends tighter than the sidewalk is wide the sidewalk narrows towards the middle of the
// corner, keeping its building-side edge short of the centre of curvature (kappa t <= 0.8). At its
// ends it keeps the arms' widths; a corner too tight for that, or narrower than kMinWalk in the
// middle, gets no corner sidewalk (empty): its arm cuts leave no room for a curb return.
std::vector<odr::Poly3> cornerSidewalkWidths(const ReferenceLine &curb, double w0, double w1, double slope0,
                                             double slope1) {
  constexpr double kMaxBend = 0.8, kMinWalk = 1.0;
  const double L = curb.length(), q = 0.25 * L;
  auto tightestOn = [&](double from, double to) {  // largest curvature towards the sidewalk (right bends)
    double k = 0.0;
    for (double s = from; s <= to + 1e-9; s += std::min(0.25, 0.1 * L)) k = std::max(k, -curb.eval(s).kappa);
    return k;
  };
  const bool endsFit = tightestOn(0.0, q) * w0 <= kMaxBend && tightestOn(L - q, L) * w1 <= kMaxBend;
  if (!endsFit) return {};
  const double tightest = tightestOn(0.0, L), widest = tightest > 0.0 ? kMaxBend / tightest : 1e9;
  if (std::max(w0, w1) <= widest) return {odr::Poly3::hermite(0.0, L, w0, slope0, w1, slope1)};
  const double wm = std::min({w0, w1, widest});
  if (wm < kMinWalk) return {};
  return {odr::Poly3::hermite(0.0, q, w0, slope0, wm, 0.0), odr::Poly3::hermite(q, L - 2.0 * q, wm, 0.0, wm, 0.0),
          odr::Poly3::hermite(L - q, q, wm, 0.0, w1, slope1)};
}

// The curb return of a corner (ASAM Junction Guideline §6): straight along the first curb, a
// circular arc tangent to both curbs, straight along the second - the largest radius up to
// `wanted` that fits between the two curb ends (OpenDRIVE 1.9 §9.3 line, §9.5 arc). Where the two
// curbs do not meet ahead of both ends (no corner point), the smoothest drivable path instead.
ReferenceLine curbReturn(const Pose &a, const Pose &b, double wanted) {
  const double turn = normalizeAngle(b.hdg - a.hdg);
  const Vec2 da = a.dir(), db = b.dir(), d = b.pos() - a.pos();
  const double den = da.cross(db);
  if (std::abs(turn) < 1e-3 || std::abs(den) < 1e-9) return drivablePath(a, b);
  const double alongA = d.cross(db) / den, backB = -d.cross(da) / den;  // corner point: a + da alongA = b - db backB
  if (alongA <= 0.0 || backB <= 0.0) return drivablePath(a, b);
  // a corner point far beyond a short gap (curbs nearly parallel, offset sideways) is no corner
  if (alongA + backB > 3.0 * d.norm() + 10.0) return drivablePath(a, b);
  constexpr double kMinGeometry = 0.01;  // m: no geometry element shorter (validator spec2.min_geometry_length)
  const double t = std::tan(0.5 * std::abs(turn));
  double radius = std::min({wanted, alongA / t, backB / t});
  if (alongA - radius * t < kMinGeometry || backB - radius * t < kMinGeometry) radius = std::min(alongA, backB) / t;
  const double tangent = radius * t;
  auto tooShort = [&](double piece) { return piece > 1e-9 && piece < kMinGeometry; };
  if (tooShort(alongA - tangent) || tooShort(backB - tangent) || radius * std::abs(turn) < kMinGeometry)
    return drivablePath(a, b);
  ReferenceLine line;
  auto add = [&](double length, double curvature) {
    if (length <= 1e-9) return;
    GeomElement e;
    e.length = length;
    e.k0 = e.k1 = curvature;
    line.elements.push_back(e);
  };
  add(alongA - tangent, 0.0);
  add(radius * std::abs(turn), turn > 0.0 ? 1.0 / radius : -1.0 / radius);
  add(backB - tangent, 0.0);
  line.elements.front().x = a.x, line.elements.front().y = a.y, line.elements.front().hdg = normalizeAngle(a.hdg);
  line.finalize();
  return line;
}

}  // namespace

// ASAM Junction Guideline §6, OpenDRIVE 1.9 §12.4: sidewalks continue round every corner of a
// junction. Going round the arms by direction, each pair of neighbours whose facing sides both end
// in a sidewalk is joined by a connecting road along the curb line (reference line at the
// sidewalk's curb-side edge, parallel to both arms where it meets them) carrying the sidewalk,
// raised by the curb, lane-linked to the arms' sidewalks.
void Builder::buildCornerSidewalks(const Cluster &cl, odr::Junction &junction, std::vector<odr::Road> &out,
                                   int &nextRoadId) {
  struct Arm {
    const RoadEnd *end;
    double outward;  ///< heading leaving the junction along the arm
  };
  std::vector<Arm> arms;
  for (const RoadEnd &e : cl.ends) {
    const odr::Road &r = od.roads[metas[e.road].roadIndex];
    arms.push_back({&e, travelPose(r, e.atEnd, false).hdg});
  }
  if (arms.size() < 2) return;
  std::sort(arms.begin(), arms.end(), [](const Arm &a, const Arm &b) { return a.outward < b.outward; });
  for (size_t i = 0; i < arms.size(); ++i) {
    // arriving along P and leaving along its anticlockwise neighbour Q, the corner lies on the right
    const RoadEnd &p = *arms[i].end, &q = *arms[(i + 1) % arms.size()].end;
    if (p.road == q.road && p.atEnd == q.atEnd) continue;
    const odr::Road &rp = od.roads[metas[p.road].roadIndex], &rq = od.roads[metas[q.road].roadIndex];
    const auto walkP = sidewalkOnRight(rp, p.atEnd, p.atEnd), walkQ = sidewalkOnRight(rq, q.atEnd, !q.atEnd);
    if (!walkP || !walkQ) continue;
    const TravelSpan spanP = travelSpan(rp, p.atEnd, p.atEnd, *walkP), spanQ = travelSpan(rq, q.atEnd, !q.atEnd, *walkQ);
    Pose a = travelPose(rp, p.atEnd, true), b = travelPose(rq, q.atEnd, false);
    const Vec2 curbP = a.offset(spanP.inner), curbQ = b.offset(spanQ.inner);
    a.x = curbP.x, a.y = curbP.y, b.x = curbQ.x, b.y = curbQ.y;
    // an acute corner (a fork, turning more than 120 degrees) ends in a nose: the sidewalk does not
    // wrap round it
    if (normalizeAngle(b.hdg - a.hdg) < -2.0 * kPi / 3.0) {
      ++rep.cornerNoses;
      continue;
    }
    const double rankFactor = std::max(metas[p.road].rank, metas[q.road].rank) >= 7 ? 1.5 : 1.0;
    const ReferenceLine curb = curbReturn(a, b, opt.cornerRadius * rankFactor);
    if (curb.elements.empty() || curb.length() < 0.05) continue;
    // the sidewalk leaves and meets the arms drifting sideways as they do (Annex D: lane edges
    // heading-continuous): laneOffset carries the curb-side edge, the width the other edge
    const TravelSpan slopeP = travelSpanSlope(rp, p.atEnd, p.atEnd, true, *walkP);
    const TravelSpan slopeQ = travelSpanSlope(rq, q.atEnd, !q.atEnd, false, *walkQ);
    std::vector<odr::Poly3> widths = cornerSidewalkWidths(curb, spanP.width(), spanQ.width(),
                                                          slopeP.inner - slopeP.outer, slopeQ.inner - slopeQ.outer);
    if (widths.empty()) {
      ++rep.cornerSidewalksTooTight;
      if (std::getenv("ODRFORGE_DEBUG_CORNER")) {
        double kMax = 0.0;
        for (double s = 0.0; s <= curb.length(); s += 0.1) kMax = std::max(kMax, -curb.eval(s).kappa);
        std::fprintf(stderr, "tight corner J%s %s->%s turn %.0f deg chord %.2f curb %.2f kmax %.3f widths %.2f %.2f at (%.0f %.0f)\n",
                     junction.id.c_str(), rp.id.c_str(), rq.id.c_str(), normalizeAngle(b.hdg - a.hdg) * 180.0 / kPi,
                     (b.pos() - a.pos()).norm(), curb.length(), kMax, spanP.width(), spanQ.width(), a.x, a.y);
      }
      continue;
    }

    odr::Road c;
    c.id = std::to_string(nextRoadId++);
    c.junction = junction.id;
    if (!rp.rule.empty()) c.rule = rp.rule;
    c.planView = curb;
    c.predecessor = odr::LinkTarget{"road", rp.id, p.atEnd ? "end" : "start"};
    c.successor = odr::LinkTarget{"road", rq.id, q.atEnd ? "end" : "start"};
    if (slopeP.inner != 0.0 || slopeQ.inner != 0.0)
      c.laneOffset.push_back(odr::Poly3::hermite(0.0, curb.length(), 0.0, slopeP.inner, 0.0, slopeQ.inner));
    c.types.push_back({0.0, rp.types.empty() ? "town" : rp.types.back().type,
                       rp.types.empty() ? "" : rp.types.back().country, std::nullopt, "km/h"});
    odr::LaneSection sec;
    sec.center.id = 0;
    sec.center.type = "none";
    const odr::RoadMark none;
    sec.center.roadMarks.push_back(none);
    odr::Lane walk;
    walk.id = -1;
    walk.type = "sidewalk";
    walk.widths = std::move(widths);
    walk.heightInner = opt.country.curbHeight;
    walk.heightOuter = opt.country.curbHeight;
    walk.link.predecessor = *walkP;
    walk.link.successor = *walkQ;
    walk.roadMarks.push_back(none);
    sec.right.push_back(walk);
    c.lanes.push_back(sec);
    connectingRoadHeight(c, ConnectionEnds{&rp, p.atEnd, &rq, q.atEnd}, spanP.inner, spanQ.inner, cl);
    odr::Connection cn;
    cn.id = std::to_string(junction.connections.size());
    cn.incomingRoad = rp.id;
    cn.connectingRoad = c.id;
    cn.contactPoint = "start";
    cn.laneLinks.push_back({*walkP, walk.id});
    junction.connections.push_back(cn);
    out.push_back(std::move(c));
    ++rep.cornerSidewalks;
  }
}

// OSM turn restriction (from, via node, to) inside a junction: does it name the movement into
// `exit`? Its "to" way is either an arm of the junction, or - where nearby intersections were built
// as one junction - a way absorbed into it. Entering an absorbed way leads to the arms at its far
// node and, with `onwards` (an only_* restriction, which binds just that first step), to every arm
// reached on through further absorbed ways. A restriction whose "to" way is neither an arm nor an
// absorbed way does not apply here (nullopt).
std::optional<bool> Builder::restrictionLeadsTo(const Cluster &cl, OsmId via, OsmId toWay, const RoadEnd &exit,
                                                bool onwards) const {
  const RoadMeta &m = metas[exit.road];
  if ((exit.atEnd ? m.wayEnd : m.wayStart) == toWay) return true;
  bool armWay = false;
  for (const RoadEnd &e : cl.ends) {
    const RoadMeta &n = metas[e.road];
    armWay |= (e.atEnd ? n.wayEnd : n.wayStart) == toWay;
  }
  if (armWay) return false;
  auto inJunction = [&](const Chain &ch) { return ch.internal && std::count(cl.nodes.begin(), cl.nodes.end(), ch.n0); };
  std::vector<OsmId> reached;  // nodes reached after entering the "to" way at the via node
  for (const Chain &ch : chains) {
    const bool onWay = std::any_of(ch.parts.begin(), ch.parts.end(), [&](const Part &p) { return segs[p.seg].way == toWay; });
    if (inJunction(ch) && onWay && (ch.n0 == via || ch.n1 == via)) reached.push_back(ch.n0 == via ? ch.n1 : ch.n0);
  }
  if (reached.empty()) return std::nullopt;
  for (size_t k = 0; onwards && k < reached.size(); ++k)
    for (const Chain &ch : chains) {
      if (!inJunction(ch) || (ch.n0 != reached[k] && ch.n1 != reached[k])) continue;
      const OsmId next = ch.n0 == reached[k] ? ch.n1 : ch.n0;
      if (next != via && !std::count(reached.begin(), reached.end(), next)) reached.push_back(next);
    }
  return std::count(reached.begin(), reached.end(), attachNode(exit)) > 0;
}

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
        double angle;            // heading change into it, + to the left
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
          // a U-turn between arms at different OSM nodes of the junction is made of the turns OSM
          // allows along the ways absorbed into it (a median opening) - where the two carriageways
          // lie far enough apart to turn between them; at one node it is built only when it is the
          // only way on
          constexpr double kMinUTurnWidth = 8.0;  // m: a passenger car's turning circle
          const double armGap = (pin.pos() - po.pos()).norm();
          const bool alongAbsorbedWays = attachNode(in) != attachNode(out) && armGap >= kMinUTurnWidth;
          const bool onlyWayOn = !regularPartner(in, true) || !regularPartner(out, false);
          if (out.road == in.road || !(alongAbsorbedWays || onlyWayOn)) continue;
          t = normalizeAngle(hOut - hIn) > 0 ? Turn::Left : Turn::Right;  // built as a tight turn
        }
        // turn restrictions on the OSM ways adjacent to the junction
        const OsmId fromWay = in.atEnd ? mi.wayEnd : mi.wayStart;
        bool forbidden = false;
        for (const auto &x : restr) {
          if (x.from != fromWay || !std::count(cl.nodes.begin(), cl.nodes.end(), x.via)) continue;
          // in a junction of several OSM nodes a restriction binds only the movements that pass its
          // via node: those arriving there, or leaving their own node for another one
          const OsmId inNode = attachNode(in), outNode = attachNode(out);
          if (inNode != x.via && outNode == inNode) continue;
          const std::optional<bool> leads = restrictionLeadsTo(cl, x.via, x.to, out, x.only);
          if (leads && (x.only ? !*leads : *leads)) forbidden = true;
        }
        if (forbidden) continue;
        std::sort(ol.begin(), ol.end(), [](int a, int b) { return std::abs(a) < std::abs(b); });
        if (LHT) std::reverse(ol.begin(), ol.end());
        targets.push_back({&out, t, ol, normalizeAngle(hOut - hIn)});
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
      std::vector<std::vector<std::pair<int, int>>> pairsOf(targets.size());  // (incoming, outgoing lane id)
      for (size_t ti = 0; ti < targets.size(); ++ti) {
        const Target &tg = targets[ti];
        const int nOut = static_cast<int>(tg.lanes.size());
        std::vector<std::pair<int, int>> &pairs = pairsOf[ti];
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
          // a single lane going straight on across a short gap (less than a taper: no room to move
          // over) continues in the outgoing lane its centre lies in - the lanes added beside it begin
          // there, on whichever side; otherwise (a wide junction, ends that do not face each other)
          // the lanes keep their order
          auto lineUp = [&](int li, int fallback) {
            const odr::Road &ro0 = od.roads[metas[tg.end->road].roadIndex];
            const double s0 = tg.end->atEnd ? ro0.length() : 0.0;
            const Pose p0 = ro0.planView.eval(s0);
            Pose a;
            if (!odr::laneCenter(ri, sIn, li, a) || (a.pos() - p0.pos()).norm() > opt.taperLength) return fallback;
            const double t = Vec2(-std::sin(p0.hdg), std::cos(p0.hdg)).dot(a.pos() - p0.pos());
            for (int o = 0; o < nOut; ++o) {
              double ta, tb;
              if (odr::laneBorders(ro0, s0, tg.lanes[o], ta, tb) && t > std::min(ta, tb) && t < std::max(ta, tb)) return o;
            }
            return fallback;
          };
          for (int k = 0; k < ns; ++k) {
            int o;
            if (tg.turn == Turn::Right) o = nOut - ns + k;            // align on the right
            else if (tg.turn == Turn::Left) o = k;                    // align on the left
            else o = ns == 1 ? lineUp(byLeft[src[k]], std::min(nOut - 1, src[k] * nOut / std::max(1, nIn)))
                             : static_cast<int>(std::lround(k * (nOut - 1) / double(ns - 1)));
            o = std::clamp(o, 0, nOut - 1);
            pairs.push_back({byLeft[src[k]], tg.lanes[o]});
          }
        }
      }
      // OpenDRIVE 1.9 §12.3 incoming roads, Junction Guideline §6: every lane entering a junction
      // has a path through it. A lane left without one (its tagged turn has no road to take, or a
      // lane split gave it none) takes the target on its own side of the approach: lanes ordered
      // from the driver's left meet the targets ordered from the left by turn angle.
      for (int i = 0; i < nIn; ++i) {
        const bool hasPath = std::any_of(pairsOf.begin(), pairsOf.end(), [&](const auto &ps) {
          return std::any_of(ps.begin(), ps.end(), [&](const auto &p) { return p.first == byLeft[i]; });
        });
        if (hasPath) continue;
        std::vector<size_t> candidates;
        for (size_t ti = 0; ti < targets.size(); ++ti)
          if (perm[i].count(targets[ti].turn)) candidates.push_back(ti);
        if (candidates.empty())
          for (size_t ti = 0; ti < targets.size(); ++ti) candidates.push_back(ti);
        std::sort(candidates.begin(), candidates.end(),
                  [&](size_t a, size_t b) { return targets[a].angle > targets[b].angle; });
        const double side = nIn > 1 ? i / double(nIn - 1) : 0.5;  // 0 = leftmost lane
        const size_t pick = candidates[static_cast<size_t>(std::lround(side * (candidates.size() - 1)))];
        const Target &tg = targets[pick];
        const int nOut = static_cast<int>(tg.lanes.size());
        const int o = tg.turn == Turn::Left    ? 0
                      : tg.turn == Turn::Right ? nOut - 1
                                               : std::min(nOut - 1, static_cast<int>(std::lround(side * (nOut - 1))));
        pairsOf[pick].push_back({byLeft[i], tg.lanes[o]});
      }
      for (size_t ti = 0; ti < targets.size(); ++ti) {
        const Target &tg = targets[ti];
        const auto &pairs = pairsOf[ti];
        if (pairs.empty()) continue;
        const ConnectionEnds ends{&ri, in.atEnd, &od.roads[metas[tg.end->road].roadIndex], tg.end->atEnd};
        connectLanes(ends, pairs, cl, J, connecting, nextConnId);
      }
    }
    buildCornerSidewalks(cl, J, connecting, nextConnId);
    // OpenDRIVE 1.8.0 §10.3 (1.9.0 rule road.linkage.both_sides_consistency): a road links to the junction only if the
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

namespace {
constexpr double kStopLineGap = 1.0;       // stop line before the crosswalk
}  // namespace

// OSM maps traffic signals either as one node in the junction (every approach signalised) or as a
// node per approach at its stop line. A light node on an approach within kControlReach of a junction
// signalises it; the approaches that carry their own light are remembered. Run before the direct
// junctions: OpenDRIVE 1.8.0 §12.1 Table 54 - junctions with traffic lights are common junctions.
void Builder::markSignalControl() {
  constexpr double kControlReach = 40.0;
  lightInJunction.assign(clusters.size(), 0);
  for (size_t k = 0; k < clusters.size(); ++k) lightInJunction[k] = clusters[k].signalised;
  litApproach.clear();
  for (int ci = 0; ci < static_cast<int>(chains.size()); ++ci) {
    const Chain &ch = chains[ci];
    if (ch.internal) continue;
    for (const RoadMeta &m : metas) {
      if (m.chain != ci) continue;
      for (const auto &[nid, s] : m.nodeS) {
        const OsmNode *nd = osm.node(nid);
        if (!nd || !tagIs(nd->tags, "highway", "traffic_signals")) continue;
        if (ch.c1 >= 0 && m.full.length() - s < kControlReach) litApproach.insert({ci, ch.c1});
        if (ch.c0 >= 0 && s < kControlReach) litApproach.insert({ci, ch.c0});
      }
    }
  }
  for (const auto &[chain, cluster] : litApproach) clusters[cluster].signalised = true;
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
    // OpenDRIVE 1.8.0 §14.2: "The value of the @fromLane attribute shall be lower than or equal to the value
    // of the @toLane attribute."
    const auto [mn, mx] = std::minmax_element(lanes.begin(), lanes.end());
    sg.validity.push_back({*mn, *mx});
  };

  // a stop / give-way sign placed at a road end (OSM highway=stop | give_way)
  auto placeSign = [&](const std::string &hw, int mi, bool atEnd) {
    odr::Road &r = od.roads[metas[mi].roadIndex];
    auto lanes = endLanes(r, atEnd, true);
    if (lanes.empty()) return;
    const SignCode code = signCode(hw);
    odr::Signal sg;
    sg.id = std::to_string(nextSignalId++);
    sg.name = hw == "stop" ? "Stop" : "GiveWay";
    sg.country = code.country;
    sg.type = code.type;
    sg.subtype = code.subtype;
    sg.width = code.width;
    sg.height = code.height;
    sg.zOffset = T.openDriveSignCatalog ? 0.0 : 2.0;
    sg.semanticsPriority = hw == "stop" ? "stop" : "yield";  // OpenDRIVE 1.8.0 §14.8
    placeAtEnd(r, atEnd, lanes, 0.5, sg);
    r.signals.push_back(sg);
    ++rep.signals;
  };

  // the road ends a stop / give-way node applies to: in a junction its minor approaches (all of them
  // for an all-way stop); on a road the junction its direction tag points to, else the nearest
  auto signTargets = [&](OsmId nid, const OsmNode &nd) {
    std::vector<std::pair<int, bool>> targets;  // (meta index, atEnd)
    if (auto it = clusterOf.find(nid); it != clusterOf.end()) {
      const Cluster &cl = clusters[it->second];
      const bool allWay = tagIs(nd.tags, "stop", "all");
      int minRank = 99;
      for (const RoadEnd &e : cl.ends) minRank = std::min(minRank, metas[e.road].rank);
      for (const RoadEnd &e : cl.ends)
        if (allWay || metas[e.road].rank == minRank) targets.push_back({e.road, e.atEnd});
      return targets;
    }
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
    return targets;
  };
  std::set<std::pair<int, bool>> signedEnd;  // road ends that get a stop / give-way sign
  for (const auto &[nid, nd] : osm.nodes)
    if (tagIs(nd.tags, "highway", "stop") || tagIs(nd.tags, "highway", "give_way"))
      for (const auto &t : signTargets(nid, nd)) signedEnd.insert(t);

  // a junction signalised through some of its approaches: an approach without a light of its own is
  // controlled by the stop / give-way sign OSM gives it (placed below), a service road or
  // living street (a driveway, a car park exit) by a give-way sign; other streets get a light (OSM
  // leaves many signalised approaches unmapped). Decided with Jev (0.86).
  enum class Control { Light, OwnSign, GiveWay };
  auto controlOf = [&](int cluster, const RoadEnd &e) {
    const std::pair<int, int> key{metas[e.road].chain, cluster};
    if (lightInJunction[cluster] || litApproach.count(key)) return Control::Light;
    if (signedEnd.count({e.road, e.atEnd})) return Control::OwnSign;
    return metas[e.road].rank <= classRank("living_street") ? Control::GiveWay : Control::Light;
  };

  for (Cluster &cl : clusters) {
    if (!cl.signalised || cl.junction < 0) continue;
    odr::Junction &J = od.junctions[cl.junction];
    // phases: the approaches of one traffic axis share a controller
    struct Approach {
      const RoadEnd *end;
      double heading;
      std::vector<int> lanes;
      int group = -1;
    };
    std::vector<Approach> ap;
    const int clusterIndex = static_cast<int>(&cl - clusters.data());
    for (const RoadEnd &e : cl.ends) {
      odr::Road &r = od.roads[metas[e.road].roadIndex];
      auto lanes = endLanes(r, e.atEnd, true);
      if (lanes.empty()) continue;
      const Control control = controlOf(clusterIndex, e);
      if (control == Control::GiveWay) placeSign("give_way", e.road, e.atEnd);
      if (control != Control::Light) continue;
      const Pose p = r.planView.eval(e.atEnd ? r.length() : 0.0);
      ap.push_back({&e, e.atEnd ? p.hdg : p.hdg + kPi, lanes, -1});
    }
    // a phase is one traffic axis: an approach, the one arriving opposite it (within 0.6 rad), and the
    // approaches running beside either of them (a parallel carriageway or slip road, within 0.35 rad).
    // Two approaches from the same side at different angles are not one phase: their paths cross.
    auto parallel = [](double h0, double h1) { return std::abs(normalizeAngle(h1 - h0)) < 0.35; };
    auto opposite = [](double h0, double h1) { return std::abs(std::abs(normalizeAngle(h1 - h0)) - kPi) < 0.6; };
    int groups = 0;
    for (size_t i = 0; i < ap.size(); ++i) {
      if (ap[i].group >= 0) continue;
      ap[i].group = groups;
      std::vector<size_t> members{i};
      bool oppositeTaken = false;
      for (size_t j = i + 1; j < ap.size(); ++j) {
        if (ap[j].group >= 0) continue;
        bool joins = std::any_of(members.begin(), members.end(), [&](size_t m) { return parallel(ap[m].heading, ap[j].heading); });
        if (!joins && !oppositeTaken && opposite(ap[i].heading, ap[j].heading)) joins = oppositeTaken = true;
        if (joins) ap[j].group = groups, members.push_back(j);
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
      // OpenDRIVE 1.8.0 §14.1 zOffset: the signal origin above the reference-line elevation. On its pole beside
      // the road, as in the Junction Guideline 1.0.0 §6.3.3 / §9 examples for type 1000001 (zOffset 2.42)
      sg.zOffset = T.trafficLightZeroOffset ? 0.0 : 2.42;
      sg.semanticsPriority = "trafficLight";  // OpenDRIVE 1.8.0 §14.8; Junction Guideline §6.3.3 Code 14
      placeAtEnd(r, a.end->atEnd, a.lanes, 0.2, sg);
      ctrls[a.group].signalIds.push_back(sg.id);
      // Stop line: OpenDRIVE catalogue type 1100001 (invisible stop line, Signal reference 1.0.0
      // Table 5), before the pedestrian crossings (Junction Guideline §6.3.3): pushed upstream,
      // kStopLineGap clear, past every crosswalk that reaches from the junction side up to it - the
      // approach's own and mapped ones; crosswalks wholly upstream of it are left behind it.
      double back = 0.2;
      for (bool moved = true; moved;) {
        moved = false;
        for (const odr::Object &o : r.objects) {
          if (o.type != "crosswalk") continue;
          const double d = a.end->atEnd ? r.length() - o.s : o.s;  // crosswalk centre from the junction end
          const double far = d + 0.5 * o.length + kStopLineGap;
          if (d - 0.5 * o.length < back && back < far) back = far, moved = true;
        }
      }
      // an approach too short to stop before its crosswalk gets no stop line: the light keeps the lane
      // validity itself (OpenDRIVE 1.8.0 §14.2; the §14.4 example light carries <validity>)
      if (T.spec().hasSignalReferenceElement() && back < r.length()) {
        // ASAM Junction Guideline §9.2: traffic stops at a stop line, valid for the approach lanes and
        // dependent on the light; the light references it and carries no lane validity itself.
        odr::Signal stop;
        stop.id = std::to_string(nextSignalId++);
        stop.name = "StopLine";
        stop.country = "OpenDRIVE";
        stop.type = "1100001";
        stop.height = 0.03;
        stop.semanticsPriority = "stopLine";  // OpenDRIVE 1.8.0 §14.8; Junction Guideline §6.3.3 Code 13
        placeAtEnd(r, a.end->atEnd, a.lanes, back, stop);
        double t0, t1, u0, u1;  // across the approach lanes: the line's centre and width
        odr::laneBorders(r, stop.s, stop.validity.front().fromLane, t0, t1);
        odr::laneBorders(r, stop.s, stop.validity.front().toLane, u0, u1);
        const double lo = std::min({t0, t1, u0, u1}), hi = std::max({t0, t1, u0, u1});
        stop.t = 0.5 * (lo + hi);
        stop.width = hi - lo;
        stop.dependencies.push_back({sg.id, "trafficLight"});
        sg.references.push_back({stop.id, "signal", "stopLine"});
        sg.validity.clear();
        r.signals.push_back(stop);
      }
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
    for (const auto &[mi, atEnd] : signTargets(nid, nd)) placeSign(hw, mi, atEnd);
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
  const SpecRevision spec = opt.target.spec();
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
    o.length = kCrosswalkLength;
    o.width = hi - lo;
    o.orientation = "none";
    const double h = 0.5 * kCrosswalkLength;
    o.outline = {{-h, lo, 0, 0}, {h, lo, 0, 0}, {h, hi, 0, 0}, {-h, hi, 0, 0}};
    r.objects.push_back(o);
    ++rep.objects;
  };
  // a crosswalk inside the junction (standard) needs a corner sidewalk on each side for its cross path to
  // link (cross_paths.cpp); where the corner builder made fewer, the crosswalk stays on the approach
  for (const Cluster &cl : clusters)
    for (const RoadEnd &e : cl.ends) {
      bool &inJunction = e.atEnd ? metas[e.road].crosswalkInJunctionEnd : metas[e.road].crosswalkInJunctionStart;
      if (inJunction && !(cl.junction >= 0 && cornerSidewalksAt(od.junctions[cl.junction], e).size() == 2)) inJunction = false;
    }
  // mapped crossings on roads
  for (const auto &[nid, nd] : osm.nodes) {
    if (!tagIs(nd.tags, "highway", "crossing") || clusterOf.count(nid)) continue;
    for (RoadMeta &m : metas)
      for (const auto &[n2, sf] : m.nodeS)
        if (n2 == nid) {
          odr::Road &r = od.roads[m.roadIndex];
          // a crossing at the mouth of a junction whose crosswalk is a cross path inside it: that cross path is
          // this crossing (no second marking on the approach)
          const double raw = sf - m.trimStart;
          if ((m.crosswalkInJunctionStart && raw < kCrosswalkLength) || (m.crosswalkInJunctionEnd && raw > r.length() - kCrosswalkLength))
            continue;
          const double s = std::clamp(raw, 1.5, std::max(1.5, r.length() - 1.5));
          crosswalk(r, s);
          midBlockCrosswalks.push_back({static_cast<size_t>(m.roadIndex), r.objects.back().id});
        }
  }
  // crosswalks on every approach of signalised / crossing junctions
  if (opt.crosswalksAtSignals)
    for (const Cluster &cl : clusters) {
      if (!(cl.signalised || cl.crossing)) continue;
      for (const RoadEnd &e : cl.ends) {
        odr::Road &r = od.roads[metas[e.road].roadIndex];
        if (r.length() < kCrosswalkMinRoad) continue;
        // inside the junction (standard): the cross path's crossing road carries it (cross_paths.cpp)
        if (e.atEnd ? metas[e.road].crosswalkInJunctionEnd : metas[e.road].crosswalkInJunctionStart) continue;
        crosswalk(r, e.atEnd ? r.length() - kCrosswalkCentreBack : kCrosswalkCentreBack);
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
      o.type = spec.hasStreetLampAsPoleSubtype() ? "pole" : "streetLamp";
      o.subtype = spec.hasStreetLampAsPoleSubtype() ? "streetLamp" : "";
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

namespace {

// OpenDRIVE 1.9 §12.2: a junction exists to connect roads; one whose movements were all dropped
// is removed (XSD 1.4 also requires at least one <connection>). Nothing references such a
// junction: its connecting roads went with its movements. A virtual junction holding only cross
// paths (§12.5, 1.8) connects its sidewalks and stays.
void dropJunctionsWithoutConnections(odr::OpenDrive &od) {
  od.junctions.erase(std::remove_if(od.junctions.begin(), od.junctions.end(),
                                    [](const odr::Junction &j) { return j.connections.empty() && j.crossPaths.empty(); }),
                     od.junctions.end());
}

}  // namespace

// OpenDRIVE 1.9 §12.13, ASAM Junction Guideline: a roundabout is one junction per arm, the ring
// built from roads between them, and the junctions grouped as a junctionGroup of type roundabout.
// Junctions joined by ring roads (OSM junction=roundabout / circular) form one group.
void Builder::groupRoundabouts() {
  std::map<std::string, std::string> parent;
  std::function<std::string(const std::string &)> find = [&](const std::string &x) {
    auto it = parent.find(x);
    if (it == parent.end() || it->second == x) return x;
    return it->second = find(it->second);
  };
  for (const RoadMeta &m : metas) {
    if (m.roadIndex < 0 || m.sections.empty() || !m.sections.front().xs.roundabout) continue;
    const odr::Road &r = od.roads[m.roadIndex];
    if (!r.predecessor || !r.successor || r.predecessor->elementType != "junction" ||
        r.successor->elementType != "junction")
      continue;
    const std::string a = find(r.predecessor->elementId), b = find(r.successor->elementId);
    parent[a] = a;
    parent[b] = b;
    parent[a] = b;
  }
  std::map<std::string, std::vector<std::string>> rings;
  for (const auto &j : od.junctions)
    if (parent.count(j.id)) rings[find(j.id)].push_back(j.id);
  for (auto &[root, ids] : rings) {
    if (ids.size() < 2) continue;
    odr::JunctionGroup g;
    g.id = std::to_string(od.junctionGroups.size() + 1);
    g.name = "Roundabout";
    g.junctionIds = ids;
    od.junctionGroups.push_back(std::move(g));
  }
}

void Builder::finish() {
  dropJunctionsWithoutConnections(od);
  groupRoundabouts();
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
  // the road network is derived from OpenStreetMap data, licensed under the ODbL 1.0 (OpenDRIVE 1.8.0 \u00a76.4.2
  // <license>: name, resource, spdxid; @text would be "The full license text" - left out, the resource links
  // it). ODbL asks for the attribution "(c) OpenStreetMap contributors"
  // (osmfoundation.org/wiki/Licence/Attribution_Guidelines): header <userData> (1.8.0 \u00a77, 1.4H \u00a75.2), both
  // profiles
  od.header.license = odr::License{"Open Data Commons Open Database License 1.0",
                                   "https://opendatacommons.org/licenses/odbl/1-0/", "ODbL-1.0", ""};
  od.header.userData["attribution"] = u8"Contains data \u00a9 OpenStreetMap contributors, available under the ODbL 1.0.";
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
