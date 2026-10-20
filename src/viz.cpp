#include "odrforge/viz.hpp"

#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

#include "odrforge/version.hpp"

namespace odrforge {

extern const char *kVizTemplate;  // generated from src/viz/viewer.html (viz_template.cpp)

namespace {

std::string esc(const std::string &s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\', o += c;
    else if (c == '<') o += "\\u003c";
    else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

int laneTypeCode(const std::string &t) {
  static const std::map<std::string, int> m = {
      {"driving", 0}, {"sidewalk", 1}, {"walking", 1}, {"biking", 2}, {"parking", 3}, {"shoulder", 4},
      {"border", 5}, {"median", 6}, {"bidirectional", 7}, {"curb", 5}, {"restricted", 8}, {"entry", 0},
      {"exit", 0}, {"onRamp", 0}, {"offRamp", 0}, {"connectingRamp", 0}, {"stop", 4}, {"none", 9}};
  auto it = m.find(t);
  return it == m.end() ? 8 : it->second;
}

int markStyle(const std::string &t) {  // 0 none, 1 solid, 2 broken, 3 solid solid, 4 solid broken, 5 broken solid, 6 broken broken, 7 dots/other
  if (t == "solid") return 1;
  if (t == "broken") return 2;
  if (t == "solid solid") return 3;
  if (t == "solid broken") return 4;
  if (t == "broken solid") return 5;
  if (t == "broken broken") return 6;
  if (t == "none" || t.empty()) return 0;
  return 7;
}

int markColor(const std::string &c) { return c == "yellow" ? 1 : c == "blue" ? 2 : c == "red" ? 3 : c == "green" ? 4 : 0; }

int signalKind(const odr::Signal &s) {  // 0 other sign, 1 traffic light, 2 stop, 3 yield, 4 speed
  if (s.dynamic || s.type == "1000001" || s.type == "1000002" || s.type == "1000011") return 1;
  if (s.type == "206" || s.type == "R1-1" || s.type == "601.1" || s.type == "AB4" || s.type == "330" || s.type == "Ra-1")
    return 2;
  if (s.type == "205" || s.type == "R1-2" || s.type == "602" || s.type == "AB3a" || s.type == "329" || s.type == "Ra-2")
    return 3;
  if (s.type == "274" || s.type == "R2-1" || s.type == "670" || s.type == "B14" || s.type == "323" || s.type == "R4-1" ||
      s.type == "Rb-1")
    return 4;
  return 0;
}

/// page coordinate unit: centimetres (DATA.units "cm"; pages before carry decimetres)
constexpr double kUnit = 100.0;

/// delta-encoded polyline in kUnit: "x0,y0,dx1,dy1,..."
struct Enc {
  double ox, oy;
  std::string poly(const std::vector<Vec2> &pts) const {
    std::string o;
    long px = 0, py = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
      const long x = std::lround((pts[i].x - ox) * kUnit), y = std::lround((pts[i].y - oy) * kUnit);
      if (i && x == px && y == py) continue;
      if (!o.empty()) o += ',';
      o += std::to_string(i ? x - px : x) + ',' + std::to_string(i ? y - py : y);
      px = x, py = y;
    }
    return o;
  }
  /// heights of the same points (cm, delta-coded, the same duplicates skipped as poly()) for the 3D view
  std::string heights(const std::vector<Vec2> &pts, const std::vector<double> &zs) const {
    std::string o;
    long px = 0, py = 0, pz = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
      const long x = std::lround((pts[i].x - ox) * kUnit), y = std::lround((pts[i].y - oy) * kUnit);
      if (i && x == px && y == py) continue;
      const long z = std::lround(zs[i] * kUnit);
      if (!o.empty()) o += ',';
      o += std::to_string(i ? z - pz : z);
      px = x, py = y, pz = z;
    }
    return o;
  }
};

/// largest |curvature| of the reference line on [a, b]: curvature is linear along each element (line, arc, spiral), so
/// the element ends bound it
double maxKappa(const ReferenceLine &line, double a, double b) {
  double k = 0.0;
  for (const auto &e : line.elements)
    if (e.s < b && e.s + e.length > a) k = std::max({k, std::abs(e.k0), std::abs(e.k1)});
  return k;
}

bool constantProfile(const odr::Road &r, const odr::LaneSection &sec, double s0, double s1) {
  for (const auto &o : r.laneOffset)
    if (o.s < s1 && (std::abs(o.b) + std::abs(o.c) + std::abs(o.d)) > 1e-9 && o.s >= s0 - 1e3) return false;
  for (const auto *side : {&sec.left, &sec.right})
    for (const auto &l : *side)
      for (const auto &w : l.widths)
        if (std::abs(w.b) + std::abs(w.c) + std::abs(w.d) > 1e-9) return false;
  return true;
}

}  // namespace

void writeVizHtml(const odr::OpenDrive &od, const std::string &path, const VizOptions &opt) {
  // extent
  double minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
  for (const auto &r : od.roads)
    for (double s = 0; s <= r.length(); s += std::max(1.0, r.length() / 8)) {
      const Pose p = r.planView.eval(s);
      minX = std::min(minX, p.x), maxX = std::max(maxX, p.x);
      minY = std::min(minY, p.y), maxY = std::max(maxY, p.y);
    }
  if (od.roads.empty()) minX = minY = 0, maxX = maxY = 1;
  const Enc enc{minX, minY};
  std::map<std::string, int> roadIdx;
  for (size_t i = 0; i < od.roads.size(); ++i) roadIdx[od.roads[i].id] = static_cast<int>(i);
  std::set<std::string> controlled;
  for (const auto &c : od.controllers)
    for (const auto &s : c.signalIds) controlled.insert(s);

  std::ostringstream lanes, marks, refs, roads, sigs, objs, juncs;
  int nLanes = 0, nMarks = 0;
  double totalLen = 0, kMax = 0;
  for (size_t ri = 0; ri < od.roads.size(); ++ri) {
    const odr::Road &r = od.roads[ri];
    const double L = r.length();
    if (L <= 0 || r.planView.elements.empty()) continue;
    totalLen += L;
    // reference line + curvature
    std::vector<Vec2> refPts;
    std::vector<int> refK;
    for (const auto &e : r.planView.elements) {
      const double k = std::max(std::abs(e.k0), std::abs(e.k1));
      const int n = std::max(1, static_cast<int>(std::ceil(e.length / std::min(opt.maxStep, std::sqrt(8.0 * opt.chordError / std::max(k, 1e-6))))));
      for (int i = (refPts.empty() ? 0 : 1); i <= n; ++i) {
        const Pose p = e.eval(e.length * i / n);
        refPts.push_back(p.pos());
        refK.push_back(static_cast<int>(std::lround(std::min(std::abs(p.kappa), 9.999) * 1000)));
        kMax = std::max(kMax, std::abs(p.kappa));
      }
    }
    refs << (ri ? "," : "") << "[" << ri << ",[" << enc.poly(refPts) << "],[";
    for (size_t i = 0; i < refK.size(); ++i) refs << (i ? "," : "") << refK[i];
    refs << "]]";
    // lanes per section
    for (size_t si = 0; si < r.lanes.size(); ++si) {
      const odr::LaneSection &sec = r.lanes[si];
      const double s0 = sec.s, s1 = si + 1 < r.lanes.size() ? r.lanes[si + 1].s : L;
      if (s1 - s0 < 1e-3) continue;
      // stations: curvature-adaptive, dense where widths / offsets vary. The chord sag of a border at offset t on a line
      // of curvature k is step^2 k_t / 8 with k_t = k / (1 - k t) (the inner side of a bend curves more): the step uses
      // the largest curvature over the whole step ahead (not just at its start, which can lie on a short straight before
      // an arc) and the outermost border of the section
      const bool constant = constantProfile(r, sec, s0, s1);
      double tMax = 0.0;
      for (double s : {s0, 0.5 * (s0 + s1), s1}) {
        double a, b;
        for (const auto *side : {&sec.left, &sec.right})
          for (const auto &l : *side)
            if (odr::laneBorders(r, std::min(s, s1 - 1e-6), l.id, a, b)) tMax = std::max({tMax, std::abs(a), std::abs(b)});
      }
      std::vector<double> st{s0};
      while (st.back() < s1 - 1e-6) {
        double step = constant ? opt.maxStep : 2.0;
        for (int it = 0; it < 4; ++it) {  // shrink until the curvature ahead allows the step
          const double k = maxKappa(r.planView, st.back(), st.back() + step);
          const double kt = k / std::max(1.0 - k * tMax, 0.1);
          const double allowed = std::sqrt(8.0 * opt.chordError / std::max(kt, 1e-9));
          if (allowed >= step) break;
          step = allowed;
        }
        step = std::max(step, 0.25);
        st.push_back(std::min(s1 - 1e-6, st.back() + step));
      }
      // then bisect every interval in which a lane border (or the lane offset line) leaves the chord between its two
      // stations by more than opt.chordError at the midpoint: lane offsets and widths can change sideways faster than
      // the curvature-based step allows for (a laneOffset blend over a few metres)
      auto borderPts = [&](double s) {
        std::vector<Vec2> pts;
        const Pose p = r.planView.eval(s);
        pts.push_back(p.offset(odr::laneOffsetAt(r, s)));
        double a, b;
        for (const auto *side : {&sec.left, &sec.right})
          for (const auto &l : *side)
            if (odr::laneBorders(r, s, l.id, a, b)) pts.push_back(p.offset(b));
        return pts;
      };
      std::vector<double> fine{st.front()};
      std::function<void(double, double, const std::vector<Vec2> &, const std::vector<Vec2> &, int)> refine =
          [&](double a, double b, const std::vector<Vec2> &pa, const std::vector<Vec2> &pb, int depth) {
            const double m = 0.5 * (a + b);
            const std::vector<Vec2> pm = borderPts(m);
            bool split = false;
            for (size_t i = 0; i < pm.size() && i < pa.size() && i < pb.size() && !split; ++i)
              split = (pm[i] - (pa[i] + pb[i]) * 0.5).norm() > opt.chordError;
            if (split && depth < 8 && b - a > 0.05) {
              refine(a, m, pa, pm, depth + 1);
              refine(m, b, pm, pb, depth + 1);
            } else {
              fine.push_back(b);
            }
          };
      {
        std::vector<Vec2> prev = borderPts(st.front());
        for (size_t i = 1; i < st.size(); ++i) {
          const std::vector<Vec2> next = borderPts(st[i]);
          refine(st[i - 1], st[i], prev, next, 0);
          prev = next;
        }
      }
      st = fine;
      std::vector<Pose> poses;
      for (double s : st) poses.push_back(r.planView.eval(s));
      // border points and their surface height (OpenDRIVE 1.8.0 §10.5 elevation, §10.6 superelevation / shape, §11.10
      // lane height: inner / outer raise of curbs and sidewalks) for the 3D view
      auto border = [&](const odr::Lane &l, bool outer, std::vector<Vec2> &out, std::vector<double> &zs) {
        out.clear(), zs.clear();
        for (size_t i = 0; i < st.size(); ++i) {
          double a, b;
          if (!odr::laneBorders(r, st[i], l.id, a, b)) return false;
          const double t = outer ? b : a;
          out.push_back(poses[i].offset(t));
          zs.push_back(odr::surfaceZ(r, st[i], t) + (outer ? l.heightOuter : l.heightInner).value_or(0.0));
        }
        return true;
      };
      std::vector<Vec2> in, out;
      std::vector<double> zIn, zOut;
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) {
          if (!border(l, false, in, zIn) || !border(l, true, out, zOut)) continue;
          // skip zero-width lanes entirely
          double wmax = 0;
          for (size_t i = 0; i < in.size(); ++i) wmax = std::max(wmax, (in[i] - out[i]).norm());
          if (wmax < 0.05) continue;
          std::vector<Vec2> poly = in;
          poly.insert(poly.end(), out.rbegin(), out.rend());
          std::vector<double> polyZ = zIn;
          polyZ.insert(polyZ.end(), zOut.rbegin(), zOut.rend());
          const double spd = l.speedMax ? *l.speedMax : -1;
          lanes << (nLanes++ ? "," : "") << "[" << ri << "," << l.id << "," << laneTypeCode(l.type) << ","
                << static_cast<int>(si) << "," << (spd >= 0 ? std::lround(spd * 10) : -1) << ",[" << enc.poly(poly) << "],["
                << enc.heights(poly, polyZ) << "]]";
          for (const auto &m : l.roadMarks) {
            const int style = markStyle(m.type);
            if (!style) continue;
            marks << (nMarks++ ? "," : "") << "[" << style << "," << markColor(m.color) << ",[" << enc.poly(out) << "]]";
          }
        }
      // centre line marks
      for (const auto &m : sec.center.roadMarks) {
        const int style = markStyle(m.type);
        if (!style) continue;
        std::vector<Vec2> c;
        for (size_t i = 0; i < st.size(); ++i) c.push_back(poses[i].offset(odr::laneOffsetAt(r, st[i])));
        marks << (nMarks++ ? "," : "") << "[" << style << "," << markColor(m.color) << ",[" << enc.poly(c) << "]]";
      }
    }
    // signals
    for (const auto &s : r.signals) {
      const Pose p = r.planView.eval(s.s);
      const Vec2 q = p.offset(s.t);
      const double h = p.hdg + (s.orientation == "-" ? kPi : 0.0);
      std::string text = s.type;
      if (s.value) text += " " + std::to_string(static_cast<int>(std::lround(*s.value))) + (s.unit.empty() ? "" : " " + s.unit);
      sigs << (sigs.tellp() ? "," : "") << "[" << std::lround((q.x - minX) * kUnit) << "," << std::lround((q.y - minY) * kUnit) << ","
           << std::lround(h * 1000) << "," << signalKind(s) << "," << ri << ",\"" << esc(s.id) << "\",\"" << esc(text) << "\","
           << (controlled.count(s.id) ? 1 : 0) << "]";
    }
    // objects
    for (const auto &o : r.objects) {
      const Pose p = r.planView.eval(o.s);
      const Vec2 c = p.offset(o.t);
      std::vector<Vec2> outline;
      const double h = p.hdg + o.hdg, ch = std::cos(h), sh = std::sin(h);
      for (const auto &k : o.outline) outline.push_back({c.x + k.u * ch - k.v * sh, c.y + k.u * sh + k.v * ch});
      const int kind = o.type == "crosswalk" ? 1 : (o.type == "streetLamp" || o.subtype == "streetLamp") ? 2 : o.type == "tree" ? 3 : 0;
      objs << (objs.tellp() ? "," : "") << "[" << kind << "," << std::lround((c.x - minX) * kUnit) << "," << std::lround((c.y - minY) * kUnit)
           << ",[" << (outline.empty() ? "" : enc.poly(outline)) << "],\"" << esc(o.type) << "\"]";
    }
    // road record: id, name, junction, length, type, speed, pred, succ, sections
    auto link = [](const std::optional<odr::LinkTarget> &t) {
      return t ? t->elementType + " " + t->elementId + (t->contactPoint.empty() ? "" : " (" + t->contactPoint + ")") : std::string("-");
    };
    std::string type = r.types.empty() ? "" : r.types.front().type;
    std::string speed = (!r.types.empty() && r.types.front().speedMax)
                            ? std::to_string(static_cast<int>(*r.types.front().speedMax)) + " " + r.types.front().speedUnit
                            : "";
    char len[32];
    std::snprintf(len, sizeof(len), "%.1f", L);
    roads << (ri ? "," : "") << "[\"" << esc(r.id) << "\",\"" << esc(r.name) << "\",\"" << esc(r.junction) << "\"," << len
          << ",\"" << esc(type) << "\",\"" << esc(speed) << "\",\"" << esc(link(r.predecessor)) << "\",\"" << esc(link(r.successor))
          << "\"," << r.lanes.size() << "," << r.planView.elements.size() << "]";
  }
  std::ostringstream bounds;
  for (size_t i = 0; i < od.junctions.size(); ++i) {
    const auto &j = od.junctions[i];
    juncs << (i ? "," : "") << "[\"" << esc(j.id) << "\"," << j.connections.size() << "," << j.controllers.size() << ",\""
          << esc(j.type) << "\"]";
    // OpenDRIVE 1.8 §12.10 boundary as a closed outline: lane segments along the outer edge of boundaryLane (the centre
    // lane's line for 0) from sStart to sEnd, joints across the lanes jointLaneStart .. jointLaneEnd (or all) of the road
    // end, run from the end nearer the next segment's start
    std::vector<std::vector<Vec2>> parts;
    for (const auto &sg : j.boundary) {
      const auto it = roadIdx.find(sg.roadId);
      if (it == roadIdx.end()) continue;
      const odr::Road &r = od.roads[it->second];
      const auto sOf = [&](const std::string &v) { return v == "start" ? 0.0 : v == "end" ? r.length() : std::atof(v.c_str()); };
      std::vector<Vec2> pts;
      if (sg.type == "joint") {
        const double s = sg.contactPoint == "start" ? 0.0 : r.length();
        const int idLo = std::min(sg.jointLaneStart, sg.jointLaneEnd), idHi = std::max(sg.jointLaneStart, sg.jointLaneEnd);
        double lo = sg.jointLanes ? 1e300 : odr::laneOffsetAt(r, s), hi = sg.jointLanes ? -1e300 : lo;
        for (const auto *side : {&odr::sectionAt(r, s).left, &odr::sectionAt(r, s).right})
          for (const odr::Lane &l : *side) {
            double a, b;
            if (sg.jointLanes && (l.id < idLo || l.id > idHi)) continue;
            if (odr::laneBorders(r, s, l.id, a, b)) lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
          }
        if (lo > hi) continue;
        pts = {r.planView.eval(s).offset(lo), r.planView.eval(s).offset(hi)};
      } else {
        const double s0 = sOf(sg.sStart), s1 = sOf(sg.sEnd);
        // samples as for the lanes: chord sag within opt.chordError on the tightest element at the border's offset
        // (k_t = k / (1 - k t)), at most 1 m apart
        const double kRef = maxKappa(r.planView, 0.0, r.length());
        double tMax = 0.0;
        for (double s : {sOf(sg.sStart), sOf(sg.sEnd)}) {
          double in = 0.0, t = odr::laneOffsetAt(r, s);
          if (sg.boundaryLane != 0) odr::laneBorders(r, s, sg.boundaryLane, in, t);
          tMax = std::max(tMax, std::abs(t));
        }
        const double kt = kRef / std::max(1.0 - kRef * tMax, 0.1);
        const double step = std::clamp(std::sqrt(8.0 * opt.chordError / std::max(kt, 1e-6)), 0.1, 1.0);
        const int n = std::max(2, static_cast<int>(std::ceil(std::abs(s1 - s0) / step)) + 1);
        for (int k = 0; k < n; ++k) {
          const double s = s0 + (s1 - s0) * k / (n - 1);
          double in = 0.0, t = odr::laneOffsetAt(r, s);
          if (sg.boundaryLane != 0) odr::laneBorders(r, s, sg.boundaryLane, in, t);
          pts.push_back(r.planView.eval(s).offset(t));
        }
      }
      parts.push_back(pts);
    }
    std::vector<Vec2> outline;
    for (size_t k = 0; k < parts.size(); ++k) {
      auto &pts = parts[k];
      const auto &next = parts[(k + 1) % parts.size()];
      if (pts.size() == 2 && next.size() > 2 && (pts[0] - next.front()).norm() < (pts[1] - next.front()).norm())
        std::swap(pts[0], pts[1]);
      outline.insert(outline.end(), pts.begin(), pts.end());
    }
    if (outline.size() >= 3) bounds << (bounds.tellp() ? "," : "") << "[\"" << esc(j.id) << "\",[" << enc.poly(outline) << "]]";
  }
  // geometry statistics
  int nLine = 0, nArc = 0, nSpiral = 0, nPoly = 0;
  for (const auto &r : od.roads)
    for (const auto &e : r.planView.elements) switch (e.type()) {
        case GeomType::Line: ++nLine; break;
        case GeomType::Arc: ++nArc; break;
        case GeomType::Spiral: ++nSpiral; break;
        default: ++nPoly;
      }
  std::ostringstream data;
  data.precision(15);  // the origin places the optional map tiles: full precision
  data << "{\"title\":\"" << esc(opt.title) << "\",\"generator\":\"OdrForge " << ODRFORGE_VERSION << "\",\"rev\":\"1."
       << od.header.revMinor << "\",\"units\":\"cm\",\"bbox\":[0,0," << std::lround((maxX - minX) * kUnit) << "," << std::lround((maxY - minY) * kUnit)
       << "],\"origin\":[" << minX << "," << minY << "],\"geo\":\"" << esc(od.header.geoReference) << "\""
       << ",\"stats\":{\"roads\":" << od.roads.size() << ",\"junctions\":" << od.junctions.size()
       << ",\"controllers\":" << od.controllers.size() << ",\"lengthKm\":" << std::lround(totalLen / 100.0) / 10.0
       << ",\"lines\":" << nLine << ",\"arcs\":" << nArc << ",\"spirals\":" << nSpiral << ",\"polys\":" << nPoly;
  if (opt.validation)
    data << ",\"valErrors\":" << opt.validation->errors << ",\"valWarnings\":" << opt.validation->warnings
         << ",\"maxLinkGap\":" << opt.validation->maxLinkGap;
  data << "},\"roads\":[" << roads.str() << "],\"junctions\":[" << juncs.str() << "],\"lanes\":[" << lanes.str()
       << "],\"marks\":[" << marks.str() << "],\"refs\":[" << refs.str() << "],\"signals\":[" << sigs.str()
       << "],\"objects\":[" << objs.str() << "],\"boundaries\":[" << bounds.str() << "]";
  if (!opt.basemapDir.empty()) {
    namespace fs = std::filesystem;
    std::ifstream bm(fs::path(opt.basemapDir) / "basemap.json", std::ios::binary);
    if (!bm) throw std::runtime_error("cannot read " + (fs::path(opt.basemapDir) / "basemap.json").string());
    const std::string meta((std::istreambuf_iterator<char>(bm)), std::istreambuf_iterator<char>());
    const fs::path htmlDir = fs::absolute(fs::path(path)).parent_path();
    std::string rel = fs::relative(fs::absolute(opt.basemapDir), htmlDir).generic_string();
    if (rel.empty()) rel = ".";
    data << ",\"basemap\":{\"dir\":\"" << esc(rel) << "\",\"meta\":" << meta << "}";
  }
  data << "}";

  std::string html = kVizTemplate;
  const std::string tagTitle = "__ODRFORGE_TITLE__", tagData = "/*__ODRFORGE_DATA__*/";
  if (auto p = html.find(tagTitle); p != std::string::npos) html.replace(p, tagTitle.size(), opt.title);
  if (auto p = html.find(tagData); p != std::string::npos) html.replace(p, tagData.size(), "const DATA=" + data.str() + ";");
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write " + path);
  f << html;
}

}  // namespace odrforge
