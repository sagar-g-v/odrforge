// OdrForge - roadside barriers (standard profile).
//
// OSM draws a guard rail, a jersey barrier or a cable barrier (barrier=guard_rail / jersey_barrier /
// cable_barrier, wiki Key:barrier) as a way where it stands, beside the carriageway. OpenDRIVE 1.8.0
// §13.14.1 Table 108 models such a barrier as <object type="barrier"> with a subtype (guardRail,
// jerseyBarrier, other) whose size is given by a continuous <repeat distance="0"> (§13.4: "recommended,
// size is defined in <repeat>"; a bounding box on the object "not recommended", distance > 0 "not
// allowed"). Each barrier becomes one object per road it lines, with repeats that follow the OSM line:
// t, zOffset, width and height are linear between a repeat's start and end (§13.4), so a repeat ends
// where the OSM line leaves that straight line by more than kChordTolerance.
//
// A barrier sample is kept on the road that runs alongside (within kMaxAngle) with the nearest outer lane
// edge, at most kMaxBeyondEdge away; a road on the barrier's OSM layer (wiki Key:layer) comes first. The
// layer only orders the candidates: a barrier way carries one layer for its whole length, while the road
// beside it changes layer where its bridge begins and ends (Frankfurt Kreuz: a 654 m guard rail tagged
// layer=1 along a road that is layer 1 on its bridge only). A sample whose footprint
// overlaps a lane someone travels or parks on is dropped (a barrier never blocks a lane): only
// shoulder, border, curb, median and none lanes may carry one - on its own road and on every other road
// (connecting roads too) whose surface there is within kOtherLevel in height. 1.9.0 road.object.repeating.valid_s_length:
// every repeat lies on its road (samples are taken strictly inside it).
//
// 1.4H §6.6 has type barrier without subtypes, and CARLA's OpenDRIVE object parser reads crosswalks
// only: the carla profile (1.4) carries no barriers (SpecRevision::hasBarrierSubtypes).
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "network_internal.hpp"

namespace odrforge {
namespace detail {

namespace {

constexpr double kSampleStep = 1.0;       // m along the OSM way
constexpr double kMaxBeyondEdge = 5.0;    // m beyond the outer lane edge (Frankfurt Kreuz guard rails: p90 4.5 m)
constexpr double kMaxAngle = 20.0;        // degrees between the barrier and the road
constexpr double kMinRun = 2.0;           // m: shorter pieces on one road are left out
constexpr double kChordTolerance = 0.05;  // m: a repeat's linear t stays this close to the samples (the OSM line)
constexpr double kGroundTolerance = 0.005; // m: ... and its linear zOffset this close (it must not sink into the ground)
constexpr double kMaxRepeat = 25.0;       // m
constexpr double kCell = 25.0;            // m: spatial index cell
constexpr double kOtherLevel = 2.5;       // m: roads further apart in height pass over / under the barrier
constexpr double kGroundStep = 0.05;      // m: a larger height change between two samples is a step in the ground

/// Table 108 subtype and an assumed typical size: the OSM ways carry no size tags (none of the test
/// areas' barriers has height or width). zBase lifts the object above the ground.
struct BarrierKind {
  const char *osm, *subtype, *name;
  double height, width, zBase;
};
// guardRail: Table 108 "metal guard rail along the side of the road (without the vertical poles)": a
// W-beam about 0.31 m tall and 0.1 m deep, its top about 0.75 m above the ground. jerseyBarrier: the New
// Jersey concrete profile, about 0.81 m tall on a 0.6 m base. Cable barrier (no Table 108 subtype:
// "other"): posts and cables about 0.8 m tall, 0.1 m wide.
constexpr BarrierKind kKinds[] = {
    {"guard_rail", "guardRail", "GuardRail", 0.31, 0.10, 0.44},
    {"jersey_barrier", "jerseyBarrier", "JerseyBarrier", 0.81, 0.60, 0.0},
    {"cable_barrier", "other", "CableBarrier", 0.80, 0.10, 0.0},
};

/// lanes a barrier may stand on: nobody travels or parks there
bool barrierMayStandOn(const std::string &type) {
  return type == "shoulder" || type == "border" || type == "curb" || type == "median" || type == "none";
}

/// lane height (1.8.0 §11.6 <height>: a record holds until the next) at ds into its section, at fraction f
/// (0 inner, 1 outer) of the lane width
double laneHeight(const odr::Lane &l, double ds, double f) {
  double in = l.heightInner.value_or(0.0), out = l.heightOuter.value_or(0.0);
  for (const odr::Lane::HeightChange &h : l.heightChanges)
    if (h.sOffset <= ds + 1e-9) in = h.inner, out = h.outer;
  return in + (out - in) * std::clamp(f, 0.0, 1.0);
}

/// OSM layer of a barrier way (as segLayer for roads: bridge 1, tunnel -1, layer tag wins)
int wayLayer(const Tags &t) {
  int layer = tagYes(t, "bridge") ? 1 : tagYes(t, "tunnel") ? -1 : 0;
  if (auto lv = tag(t, "layer")) {
    try {
      layer = std::stoi(*lv);
    } catch (...) {
    }
  }
  return layer;
}

struct BarrierSample {
  int meta = -1;
  double s = 0.0, t = 0.0, z = 0.0;
};

}  // namespace

void Builder::roadBarriers() {
  if (!opt.target.spec().hasBarrierSubtypes() || !opt.streetFurniture) return;
  // spatial index of the roads' reference lines (5 m samples)
  std::map<std::pair<int, int>, std::set<int>> grid;
  for (int i = 0; i < static_cast<int>(metas.size()); ++i) {
    if (metas[i].roadIndex < 0) continue;
    const odr::Road &r = od.roads[metas[i].roadIndex];
    for (double s = 0; s <= r.length(); s += 5.0) {
      const Pose p = r.planView.eval(s);
      grid[{static_cast<int>(std::floor(p.x / kCell)), static_cast<int>(std::floor(p.y / kCell))}].insert(i);
    }
  }
  // ... and of every road (connecting roads in junctions too), for the lanes a barrier must keep clear of
  std::map<std::pair<int, int>, std::set<int>> roadGrid;
  for (int i = 0; i < static_cast<int>(od.roads.size()); ++i)
    for (double s = 0; s <= od.roads[i].length(); s += 5.0) {
      const Pose p = od.roads[i].planView.eval(s);
      roadGrid[{static_cast<int>(std::floor(p.x / kCell)), static_cast<int>(std::floor(p.y / kCell))}].insert(i);
    }
  const double maxSin = std::sin(kMaxAngle * kPi / 180.0);
  // every road's element midpoints for the projections below (barriers add objects, the reference lines stay)
  std::vector<std::vector<Vec2>> mids(od.roads.size());
  for (size_t i = 0; i < od.roads.size(); ++i) mids[i] = od.roads[i].planView.midpoints();

  // the road a barrier sample lines (false: none, or it would stand on a lane in use)
  auto place = [&](const Vec2 &q, const Vec2 &dir, int layer, const BarrierKind &k, BarrierSample &out) {
    const int cx = static_cast<int>(std::floor(q.x / kCell)), cy = static_cast<int>(std::floor(q.y / kCell));
    std::set<int> cand;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        if (auto it = grid.find({cx + dx, cy + dy}); it != grid.end()) cand.insert(it->second.begin(), it->second.end());
    double best = 1e18;
    for (int c : cand) {
      const odr::Road &r = od.roads[metas[c].roadIndex];
      double s, t;
      r.planView.project(q, s, t, mids[static_cast<size_t>(metas[c].roadIndex)]);
      if (s <= 1e-3 || s >= r.length() - 1e-3) continue;
      const Pose p = r.planView.eval(s);
      if (std::abs((q - p.pos()).dot(p.dir())) > 0.5 || std::abs(dir.cross(p.dir())) > maxSin) continue;
      // outer lane edges at s
      const odr::LaneSection &sec = odr::sectionAt(r, s);
      double lo = odr::laneOffsetAt(r, s), hi = lo, a, b;
      for (const auto *side : {&sec.left, &sec.right})
        for (const odr::Lane &l : *side)
          if (odr::laneBorders(r, s, l.id, a, b)) lo = std::min({lo, a, b}), hi = std::max({hi, a, b});
      const double beyond = t > hi ? t - 0.5 * k.width - hi : t < lo ? lo - (t + 0.5 * k.width) : 0.0;
      if (beyond > kMaxBeyondEdge) continue;
      // a road on the barrier's own layer first, then the nearest edge
      const double key = (layerAt(metas[c], s) != layer ? 1e3 : 0.0) + std::max(beyond, 0.0);
      if (key < best - 1e-9) best = key, out = {c, s, t, 0.0};
    }
    return out.meta >= 0;
  };

  // the sample's footprint on lanes: false where it overlaps a lane in use of its road, or of any other road on
  // the same level (a parallel or merging road, a junction's connecting road); z: the surface it stands on
  auto standing = [&](BarrierSample &smp, const BarrierKind &k, const Vec2 &q) {
    const odr::Road &r = od.roads[metas[smp.meta].roadIndex];
    const odr::LaneSection &sec = odr::sectionAt(r, smp.s);
    const double f0 = smp.t - 0.5 * k.width, f1 = smp.t + 0.5 * k.width;
    bool onLane = false;
    double onHeight = 0.0, beyondHeight = 0.0, outermost = -1e9;
    for (const auto *side : {&sec.left, &sec.right})
      for (const odr::Lane &l : *side) {
        double a, b;  // inner, outer border
        if (!odr::laneBorders(r, smp.s, l.id, a, b) || std::abs(b - a) < 1e-6) continue;
        if (std::max(f0, std::min(a, b)) < std::min(f1, std::max(a, b)) - 1e-6) {
          if (!barrierMayStandOn(l.type)) return false;
          onHeight = std::max(onHeight, laneHeight(l, smp.s - sec.s, (smp.t - a) / (b - a)));
          onLane = true;
        }
        // beyond the road: the ground continues at the outer edge of the outermost lane on that side
        const double reach = b > a ? b : -b;  // how far the outer border lies out on the lane's side
        if ((smp.t - b) * (b - a) > 0 && reach > outermost)
          outermost = reach, beyondHeight = laneHeight(l, smp.s - sec.s, 1.0);
      }
    const double zHere = odr::surfaceZ(r, smp.s, smp.t);
    const int cx = static_cast<int>(std::floor(q.x / kCell)), cy = static_cast<int>(std::floor(q.y / kCell));
    std::set<int> near;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        if (auto it = roadGrid.find({cx + dx, cy + dy}); it != roadGrid.end()) near.insert(it->second.begin(), it->second.end());
    for (int ri : near) {
      if (ri == metas[smp.meta].roadIndex) continue;
      const odr::Road &o = od.roads[ri];
      double s, t;
      o.planView.project(q, s, t, mids[static_cast<size_t>(ri)]);
      if (s <= 1e-3 || s >= o.length() - 1e-3) continue;
      const Pose p = o.planView.eval(s);
      if (std::abs((q - p.pos()).dot(p.dir())) > 0.5 || std::abs(odr::surfaceZ(o, s, t) - zHere) > kOtherLevel) continue;
      const odr::LaneSection &os = odr::sectionAt(o, s);
      for (const auto *side : {&os.left, &os.right})
        for (const odr::Lane &l : *side) {
          double a, b;
          if (!barrierMayStandOn(l.type) && odr::laneBorders(o, s, l.id, a, b) &&
              std::max(t - 0.5 * k.width, std::min(a, b)) < std::min(t + 0.5 * k.width, std::max(a, b)) - 1e-6)
            return false;
        }
    }
    const double height = onLane ? onHeight : beyondHeight;
    // object zOffset is relative to the reference line elevation; the surface at t adds the cross slope
    // (1.8.0 §10.5.2 superelevation)
    smp.z = odr::surfaceZ(r, smp.s, smp.t) - odr::elevationAt(r, smp.s) + height + k.zBase;
    return true;
  };

  auto emit = [&](std::vector<BarrierSample> run, const BarrierKind &k) {
    if (run.size() < 2) return false;
    if (run.back().s < run.front().s) std::reverse(run.begin(), run.end());
    if (run.back().s - run.front().s < kMinRun) return false;
    odr::Road &r = od.roads[metas[run.front().meta].roadIndex];
    odr::Object o;
    o.id = std::to_string(nextObjectId++);
    o.name = k.name;
    o.type = "barrier";
    o.subtype = k.subtype;
    o.orientation = "none";
    for (size_t i = 0; i + 1 < run.size();) {
      size_t j = i + 1;
      // extend while the straight line from i to j + 1 stays within the tolerances of the samples
      while (j + 1 < run.size() && run[j + 1].s - run[i].s <= kMaxRepeat) {
        bool fits = true;
        for (size_t m = i + 1; m <= j && fits; ++m) {
          const double u = (run[m].s - run[i].s) / (run[j + 1].s - run[i].s);
          fits = std::abs(run[i].t + u * (run[j + 1].t - run[i].t) - run[m].t) <= kChordTolerance &&
                 std::abs(run[i].z + u * (run[j + 1].z - run[i].z) - run[m].z) <= kGroundTolerance;
        }
        if (!fits) break;
        ++j;
      }
      o.repeats.push_back({run[i].s, run[j].s - run[i].s, 0.0, run[i].t, run[j].t, run[i].z, run[j].z, k.height,
                           k.height, k.width, k.width});
      rep.barrierLength += run[j].s - run[i].s;
      i = j;
    }
    o.s = o.repeats.front().s;
    o.t = o.repeats.front().tStart;
    o.zOffset = o.repeats.front().zOffsetStart;
    r.objects.push_back(o);
    ++rep.objects;
    ++rep.barriers;
    return true;
  };

  std::vector<OsmId> ids;  // way id order: the same output on every platform
  for (const auto &[id, w] : osm.ways)
    if (auto b = tag(w.tags, "barrier"); b && std::any_of(std::begin(kKinds), std::end(kKinds), [&](const BarrierKind &k) { return *b == k.osm; }))
      ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  // every sample of every barrier way, in way order: where it lies, which way it belongs to
  struct Point {
    size_t way;  // index into ids
    Vec2 q, dir;
    bool placed = false, stands = false;
    BarrierSample smp;
  };
  std::vector<const BarrierKind *> kinds(ids.size());
  std::vector<int> layers(ids.size());
  std::vector<Point> points;
  for (size_t wi = 0; wi < ids.size(); ++wi) {
    const OsmWay &w = osm.ways.at(ids[wi]);
    const std::string b = *tag(w.tags, "barrier");
    kinds[wi] = &*std::find_if(std::begin(kKinds), std::end(kKinds), [&](const BarrierKind &x) { return b == x.osm; });
    layers[wi] = wayLayer(w.tags);
    for (size_t n = 0; n + 1 < w.nodes.size(); ++n) {
      const Vec2 a = pos(w.nodes[n]), d = pos(w.nodes[n + 1]) - a;
      const double len = d.norm();
      if (len < 1e-6) continue;
      const Vec2 dir = d * (1.0 / len);
      const int steps = std::max(1, static_cast<int>(std::ceil(len / kSampleStep)));
      const bool last = n + 2 == w.nodes.size();
      for (int i = 0; i <= steps - (last ? 0 : 1); ++i) {
        Point p;
        p.way = wi, p.q = a + d * (static_cast<double>(i) / steps), p.dir = dir;
        points.push_back(p);
      }
    }
  }
  // placing and standing a sample read the roads' lines and lanes only (a barrier is an object, added below): all
  // samples in parallel, each writing its own result
  forEach(opt.parallel, points.begin(), points.end(), [&](Point &p) {
    p.placed = place(p.q, p.dir, layers[p.way], *kinds[p.way], p.smp);
    if (p.placed) p.stands = standing(p.smp, *kinds[p.way], p.q);
  });
  // the runs, way by way in id order, as one sample after the other
  for (size_t at = 0; at < points.size();) {
    const size_t wi = points[at].way;
    const BarrierKind &k = *kinds[wi];
    std::vector<BarrierSample> run;
    auto close = [&]() {
      if (!run.empty() && !emit(run, k)) rep.barrierSamplesShort += static_cast<int>(run.size());
      run.clear();
    };
    for (; at < points.size() && points[at].way == wi; ++at) {
      const Point &p = points[at];
      ++rep.barrierSamples;
      if (!p.placed) {
        ++rep.barrierSamplesUnmatched;
        close();
        continue;
      }
      if (!p.stands) {
        ++rep.barrierSamplesOnLane;
        close();
        continue;
      }
      // a run continues on the same road, s moving on by about one step in one direction, on ground without a
      // step (a raised sidewalk ending, a lowered curb: a repeat's linear zOffset would cut into the ground there)
      const BarrierSample &smp = p.smp;
      if (!run.empty() && run.back().meta == smp.meta && std::abs(smp.s - run.back().s) < 1e-3) continue;
      if (!run.empty() && (run.back().meta != smp.meta || std::abs(smp.s - run.back().s) > 2.0 * kSampleStep ||
                           std::abs(smp.z - run.back().z) > kGroundStep ||
                           (run.size() >= 2 && (smp.s - run.back().s) * (run.back().s - run[run.size() - 2].s) < 0)))
        close();
      run.push_back(smp);
    }
    close();
  }
}

}  // namespace detail
}  // namespace odrforge
