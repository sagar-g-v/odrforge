#include "odrforge/viz.hpp"

#include <algorithm>
#include <cmath>
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

/// delta-encoded decimetre polyline: "x0,y0,dx1,dy1,..."
struct Enc {
  double ox, oy;
  std::string poly(const std::vector<Vec2> &pts) const {
    std::string o;
    long px = 0, py = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
      const long x = std::lround((pts[i].x - ox) * 10.0), y = std::lround((pts[i].y - oy) * 10.0);
      if (i && x == px && y == py) continue;
      if (!o.empty()) o += ',';
      o += std::to_string(i ? x - px : x) + ',' + std::to_string(i ? y - py : y);
      px = x, py = y;
    }
    return o;
  }
};

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
      // stations: curvature-adaptive, dense where widths / offsets vary
      const bool constant = constantProfile(r, sec, s0, s1);
      std::vector<double> st{s0};
      while (st.back() < s1 - 1e-6) {
        const double k = std::abs(r.planView.eval(st.back()).kappa);
        double step = std::min(constant ? opt.maxStep : 2.0, std::sqrt(8.0 * opt.chordError / std::max(k, 1e-6)));
        step = std::max(step, 0.25);
        st.push_back(std::min(s1 - 1e-6, st.back() + step));
      }
      std::vector<Pose> poses;
      for (double s : st) poses.push_back(r.planView.eval(s));
      auto border = [&](int laneId, bool outer, std::vector<Vec2> &out) {
        out.clear();
        for (size_t i = 0; i < st.size(); ++i) {
          double a, b;
          if (!odr::laneBorders(r, st[i], laneId, a, b)) return false;
          out.push_back(poses[i].offset(outer ? b : a));
        }
        return true;
      };
      std::vector<Vec2> in, out;
      for (const auto *side : {&sec.left, &sec.right})
        for (const auto &l : *side) {
          if (!border(l.id, false, in) || !border(l.id, true, out)) continue;
          // skip zero-width lanes entirely
          double wmax = 0;
          for (size_t i = 0; i < in.size(); ++i) wmax = std::max(wmax, (in[i] - out[i]).norm());
          if (wmax < 0.05) continue;
          std::vector<Vec2> poly = in;
          poly.insert(poly.end(), out.rbegin(), out.rend());
          const double spd = l.speedMax ? *l.speedMax : -1;
          lanes << (nLanes++ ? "," : "") << "[" << ri << "," << l.id << "," << laneTypeCode(l.type) << ","
                << static_cast<int>(si) << "," << (spd >= 0 ? std::lround(spd * 10) : -1) << ",[" << enc.poly(poly) << "]]";
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
      sigs << (sigs.tellp() ? "," : "") << "[" << std::lround((q.x - minX) * 10) << "," << std::lround((q.y - minY) * 10) << ","
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
      objs << (objs.tellp() ? "," : "") << "[" << kind << "," << std::lround((c.x - minX) * 10) << "," << std::lround((c.y - minY) * 10)
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
  for (size_t i = 0; i < od.junctions.size(); ++i) {
    const auto &j = od.junctions[i];
    juncs << (i ? "," : "") << "[\"" << esc(j.id) << "\"," << j.connections.size() << "," << j.controllers.size() << ",\""
          << esc(j.type) << "\"]";
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
       << od.header.revMinor << "\",\"bbox\":[0,0," << std::lround((maxX - minX) * 10) << "," << std::lround((maxY - minY) * 10)
       << "],\"origin\":[" << minX << "," << minY << "],\"geo\":\"" << esc(od.header.geoReference) << "\""
       << ",\"stats\":{\"roads\":" << od.roads.size() << ",\"junctions\":" << od.junctions.size()
       << ",\"controllers\":" << od.controllers.size() << ",\"lengthKm\":" << std::lround(totalLen / 100.0) / 10.0
       << ",\"lines\":" << nLine << ",\"arcs\":" << nArc << ",\"spirals\":" << nSpiral << ",\"polys\":" << nPoly;
  if (opt.validation)
    data << ",\"valErrors\":" << opt.validation->errors << ",\"valWarnings\":" << opt.validation->warnings
         << ",\"maxLinkGap\":" << opt.validation->maxLinkGap;
  data << "},\"roads\":[" << roads.str() << "],\"junctions\":[" << juncs.str() << "],\"lanes\":[" << lanes.str()
       << "],\"marks\":[" << marks.str() << "],\"refs\":[" << refs.str() << "],\"signals\":[" << sigs.str()
       << "],\"objects\":[" << objs.str() << "]";
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
