// odrforge - the OdrForge command line: `build` makes an ASAM OpenDRIVE (.xodr) road network from a map source
// (OpenStreetMap .osm XML today), `view` writes an interactive HTML view of any OpenDRIVE file.
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "odrforge/conform.hpp"
#include "odrforge/network.hpp"
#include "odrforge/validate.hpp"
#include "odrforge/version.hpp"
#include "odrforge/viz.hpp"

using namespace odrforge;

// map sources `build` reads; --from names one, else the input's file extension picks it
static const char *const kSources = "osm (OpenStreetMap XML, .osm)";

static std::string sourceFromExtension(const std::string &path) {
  const size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || path.find_first_of("/\\", dot) != std::string::npos) return "";
  std::string ext = path.substr(dot + 1);
  for (char &c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return ext == "osm" ? "osm" : "";
}

static void mainUsage() {
  std::printf(
      "odrforge %s - road networks as ASAM OpenDRIVE\n"
      "usage: odrforge <command> [arguments]\n"
      "  build <in> <out.xodr> [options]      make an OpenDRIVE road network from a map source\n"
      "  view <in.xodr> <out.html> [options]  interactive HTML view of an OpenDRIVE file\n"
      "  --version                            print the version\n"
      "odrforge <command> --help shows the command's options.\n",
      ODRFORGE_VERSION);
}

static void viewUsage() {
  std::printf(
      "odrforge view %s - OpenDRIVE to interactive HTML\n"
      "usage: odrforge view <in.xodr> <out.html> [options]\n"
      "  --title T           page title (default: the input path)\n"
      "  --basemap DIR       offline HD map / imagery backgrounds (tools/make_basemap.py)\n",
      ODRFORGE_VERSION);
}

static void usage() {
  std::printf(
      "odrforge build %s - map source to OpenDRIVE\n"
      "usage: odrforge build <in> <out.xodr> [options]\n"
      "  --from SOURCE       the input's map source: %s; default from the file extension\n"
      "  --country CC        ISO 3166-1 alpha-2 rules (driving side, widths, speeds, signs); default XX\n"
      "  --target T          standard (OpenDRIVE 1.8, default) | carla (OpenDRIVE 1.4 + CARLA conventions)\n"
      "  --origin LAT,LON    projection origin (default: centre of the data)\n"
      "  --name NAME         header name\n"
      "  --no-sidewalks      do not assume sidewalks on untagged urban streets\n"
      "  --separate-sidewalks  extend roads to sidewalks mapped as their own ways (experimental)\n"
      "  --no-kerb-edges     keep the carriageway edges at the tagged lane widths (do not follow barrier=kerb)\n"
      "  --no-furniture      no street lamps / trees\n"
      "  --viz OUT.html      also write an interactive HTML view of the result\n"
      "  --basemap DIR       offline HD map / imagery backgrounds for the view (tools/make_basemap.py)\n"
      "  --terrain G.odrz    ground surface grid (tools/make_terrain.py): real road elevation\n"
      "  --structures S.odrz bridge-deck surface grid (lidar class 17) for bridge=yes roads\n"
      "  --ground-out C.odrz the --terrain ground conformed to the roads (road surface under the lanes, cut / fill\n"
      "                      slopes beside them) for 3D scenes and viewers that drape terrain around the roads\n"
      "  --clip MINLAT,MINLON,MAXLAT,MAXLON  cut the network to this area (ways split at the boundary)\n"
      "  --no-superelevation flat cross sections and level junctions (for consumers that ignore\n"
      "                      <superelevation>, e.g. CARLA meshing the roads itself)\n"
      "  --no-gores          freeway splits / merges as plain junctions (no lane-level gores)\n"
      "  --no-snap           do not join OSM near-misses or add missing crossing nodes\n"
      "  --no-overlap-shift  do not move roads drawn too close apart\n"
      "  --no-parking-lots   do not drive through parking lots that connect streets\n"
      "  --corner-tolerance M  how far a rounded corner may leave the mapped corner (default 1.5 m)\n",
      ODRFORGE_VERSION, kSources);
}

// argv[1] = input, argv[2] = output, options from argv[3]
static int build(int argc, char **argv) {
  if (argc < 3) {
    usage();
    return 2;
  }
  const std::string in = argv[1], out = argv[2];
  std::string from, country = "XX", target = "standard", name, vizPath, terrainPath, structuresPath, basemapDir, groundOutPath;
  double lat0 = 1e9, lon0 = 1e9, clip[4] = {0, 0, 0, 0};
  bool haveClip = false, flat = false;
  BuildOptions opt;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--from") from = next();
    else if (a == "--country") country = next();
    else if (a == "--target") target = next();
    else if (a == "--name") name = next();
    else if (a == "--origin") {
      const std::string v = next();
      std::sscanf(v.c_str(), "%lf,%lf", &lat0, &lon0);
    } else if (a == "--no-sidewalks") opt.lanes.assumeSidewalks = false;
    else if (a == "--separate-sidewalks") opt.separateSidewalks = true;
    else if (a == "--no-kerb-edges") opt.followKerbs = false;
    else if (a == "--no-furniture") opt.streetFurniture = false;
    else if (a == "--viz") vizPath = next();
    else if (a == "--basemap") basemapDir = next();
    else if (a == "--terrain") terrainPath = next();
    else if (a == "--structures") structuresPath = next();
    else if (a == "--ground-out") groundOutPath = next();
    else if (a == "--no-superelevation") flat = true;
    else if (a == "--no-gores") opt.laneLevelGores = false;
    else if (a == "--no-overlap-shift") opt.resolveOverlaps = false;
    else if (a == "--no-snap") opt.snapDanglingEnds = 0.0;
    else if (a == "--no-parking-lots") opt.connectParkingLots = false;
    else if (a == "--corner-tolerance") opt.fit.maxCornerDeviation = std::atof(next().c_str());
    else if (a == "--clip") {
      const std::string v = next();
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf,%lf", &clip[0], &clip[1], &clip[2], &clip[3]) == 4) haveClip = true;
    }
    else {
      usage();
      return 2;
    }
  }
  if (from.empty()) from = sourceFromExtension(in);
  if (from.empty()) {
    std::fprintf(stderr, "odrforge build: cannot tell the map source of %s from its extension: give --from (sources: %s)\n",
                 in.c_str(), kSources);
    return 2;
  }
  if (from != "osm") {
    std::fprintf(stderr, "odrforge build: unknown map source '%s' (sources: %s)\n", from.c_str(), kSources);
    return 2;
  }
  try {
    const auto t0 = std::chrono::steady_clock::now();
    // ODRFORGE_VERBOSE: the time of each phase outside the network build too (the build prints its own stages)
    const bool verbose = std::getenv("ODRFORGE_VERBOSE") != nullptr;
    auto mark = t0;
    auto phase = [&](const char *name) {
      const auto now = std::chrono::steady_clock::now();
      if (verbose) std::fprintf(stderr, "[odrforge] phase %-21s %8.0f ms\n", name, std::chrono::duration<double, std::milli>(now - mark).count());
      mark = now;
    };
    const OsmData osm = OsmData::readXml(in);
    phase("read OSM");
    if (lat0 > 1e8) lat0 = 0.5 * (osm.minLat + osm.maxLat), lon0 = 0.5 * (osm.minLon + osm.maxLon);
    const Projection proj(lat0, lon0);
    opt.country = CountryProfile::load(country);
    opt.target = TargetProfile::byName(target);
    if (flat) opt.target.superelevation = false;
    if (!name.empty()) opt.name = name;
    if (haveClip) {
      const Vec2 a = proj.forward(clip[0], clip[1]), b = proj.forward(clip[2], clip[3]);
      const Vec2 c = proj.forward(clip[0], clip[3]), d = proj.forward(clip[2], clip[1]);
      opt.clipBox = std::array<double, 4>{std::min({a.x, b.x, c.x, d.x}), std::min({a.y, b.y, c.y, d.y}),
                                          std::max({a.x, b.x, c.x, d.x}), std::max({a.y, b.y, c.y, d.y})};
    }
    Terrain terrain;
    if (!terrainPath.empty()) terrain.ground = TerrainGrid::load(terrainPath);
    if (!structuresPath.empty()) terrain.structures = TerrainGrid::load(structuresPath);
    if (!terrain.ground.empty() || !terrain.structures.empty()) {
      opt.terrain = &terrain;
      opt.verticalDatum = !terrain.ground.datum.empty() ? terrain.ground.datum : terrain.structures.datum;
    }
    phase("load terrain");
    BuildReport rep;
    odr::OpenDrive od = buildOpenDrive(osm, proj, opt, &rep);
    if (haveClip) {  // the box the ways were cut at, for checkers measuring OSM features wholly inside it (header userData)
      char box[128];
      std::snprintf(box, sizeof box, "%.6f,%.6f,%.6f,%.6f", clip[0], clip[1], clip[2], clip[3]);
      od.header.userData["clip"] = box;
    }
    phase("build network");
    if (!groundOutPath.empty() && terrain.ground.empty()) {
      std::fprintf(stderr, "--ground-out needs --terrain\n");
      return 2;
    }
    // on measured terrain the ground conformed to the roads (the grid --ground-out publishes) also seats the trees and
    // street lamps: their base on that ground, not at the reference line's height
    TerrainGrid conformed;
    if (!terrain.ground.empty()) {
      conformed = conformGround(od, terrain.ground);
      const SeatReport seat = seatObjects(od, conformed);
      std::printf("  objects on the ground: %d seated (up to %.2f m up, %.2f m down), %d left on bridge decks\n", seat.seated,
                  seat.maxLift, seat.maxDrop, seat.onBridges);
    }
    phase("conform ground, objects");
    odr::WriteOptions wo;
    wo.revMinor = opt.target.revMinor;
    odr::write(od, out, wo);
    if (!groundOutPath.empty()) {
      conformed.save(groundOutPath);
      std::printf("  conformed ground: %s\n", groundOutPath.c_str());
    }
    phase("write");
    const ValidationReport vr = validate(od, opt.target);
    phase("validate");
    if (!vizPath.empty()) {
      VizOptions vo;
      vo.title = opt.name;
      vo.validation = &vr;
      vo.basemapDir = basemapDir;
      writeVizHtml(od, vizPath, vo);
      phase("HTML view");
    }
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("odrforge build %s: %s -> %s in %.2f s\n", ODRFORGE_VERSION, in.c_str(), out.c_str(), secs);
    std::printf("  roads %d + connecting %d, junctions %d, signals %d, controllers %d, objects %d\n", rep.roads,
                rep.connectingRoads, rep.junctions, rep.signals, rep.controllers, rep.objects);
    std::printf("  geometry: %d elements (%d lines, %d arcs, %d spirals)\n", rep.geometryElements, rep.lines, rep.arcs,
                rep.spirals);
    std::printf("  fit to OSM nodes: mean %.3f m, p95 %.3f m, max %.2f m\n", rep.fitMeanDev, rep.fitP95Dev,
                rep.fitMaxDev);
    if (rep.elevSamples)
      std::printf("  elevation vs measured ground (%d lane-centre samples): mean %.3f m, p95 %.3f m, max %.2f m\n",
                  rep.elevSamples, rep.elevMeanDev, rep.elevP95Dev, rep.elevMaxDev);
    if (rep.elevSamples)
      std::printf("  steep spots (> 20 %%): %d, of which not supported by the measured terrain: %d; overlap shifts %d\n",
                  rep.steepSpots, rep.steepNotInTerrain, rep.overlapShifts);
    if (rep.snappedEnds || rep.parkingAisles || rep.joinedCrossings)
      std::printf("  topology: %d OSM way ends joined to the road they stop short of, %d missing crossing nodes added, "
                  "%d parking-lot aisles connect streets\n",
                  rep.snappedEnds, rep.joinedCrossings, rep.parkingAisles);
    if (rep.inheritedLinkLanes)
      std::printf("  lanes: %d untagged link pieces take the lanes of the tagged piece they continue\n", rep.inheritedLinkLanes);
    if (rep.gores)
      std::printf("  gores: %d freeway splits / merges modelled at lane level, %d as direct junctions\n", rep.gores,
                  rep.directJunctions);
    if (rep.separateSidewalks)
      std::printf("  cross-section: %d road sides extended to their separately mapped sidewalk\n", rep.separateSidewalks);
    if (rep.kerbSides || rep.kerbSamplesIgnored)
      std::printf("  kerb edges: %d road sides follow a mapped kerb (%d kerb positions used; left out: %d would leave a "
                  "lane under 2 m, %d on another level or beyond another road's lanes)\n",
                  rep.kerbSides, rep.kerbSamples, rep.kerbSamplesIgnored, rep.kerbSamplesOffLevel);
    if (rep.loopRoadsSplit || rep.loopRoadsKept)
      std::printf("  loop roads: %d roads starting and ending in one junction cut in two, linked road to road (CARLA); "
                  "%d with no place to cut\n", rep.loopRoadsSplit, rep.loopRoadsKept);
    if (rep.parkingNoStoppingAtJunctions)
      std::printf("  parking: %d roads' tagged parking lanes carry \"no stopping at any time\" near junctions (%.1f m, "
                  "country rule)\n",
                  rep.parkingNoStoppingAtJunctions, opt.country.parkingClearOfJunction);
    if (rep.parkingDroppedOnShortRoads)
      std::printf("  parking: %d roads too short to park on beside their junctions keep no tagged parking lane\n",
                  rep.parkingDroppedOnShortRoads);
    if (rep.streetSideParking || rep.streetSideParkingLeftOut)
      std::printf("  street-side parking: %d mapped areas drawn as parking lanes, %d left out; %d tag parking lanes "
                  "gave way to them\n",
                  rep.streetSideParking, rep.streetSideParkingLeftOut, rep.parkingTagsOverruled);
    if (rep.cornerSidewalks || rep.cornerSidewalksTooTight)
      std::printf("  corner sidewalks: %d continued, %d corners too tight for their sidewalk, %d acute (nose)\n",
                  rep.cornerSidewalks, rep.cornerSidewalksTooTight, rep.cornerNoses);
    if (rep.crossPaths)
      std::printf("  cross paths: %d (%d inside junctions, %d of those with pedestrian lights)\n", rep.crossPaths,
                  rep.junctionCrossPaths, rep.pedestrianSignals);
    if (rep.junctionGrids || rep.junctionBoundaries || rep.boundariesLeftOut)
      std::printf("  junction surfaces: %d elevation grids, %d boundaries (%d helper roads, %d traced along the lanes' "
                  "contour), %d left without a boundary\n",
                  rep.junctionGrids, rep.junctionBoundaries, rep.boundaryHelperRoads, rep.boundariesTraced,
                  rep.boundariesLeftOut);
    if (rep.junctionRoadsUnderCover)
      std::printf("  junction roads under cover: %d (in a tunnel, or the part under the terrain at a portal)\n",
                  rep.junctionRoadsUnderCover);
    if (rep.barrierSamples)
      std::printf("  barriers: %d objects, %.0f m; of %d samples (1 m) %d without a road alongside, %d on a lane in use, "
                  "%d in pieces < 2 m\n",
                  rep.barriers, rep.barrierLength, rep.barrierSamples, rep.barrierSamplesUnmatched,
                  rep.barrierSamplesOnLane, rep.barrierSamplesShort);
    std::printf("  validation: %d errors, %d warnings\n", vr.errors, vr.warnings);
    for (const auto &[rule, n] : vr.ruleCounts) std::printf("    %5d  %s\n", n, rule.c_str());
    if (std::getenv("ODRFORGE_VERBOSE"))
      for (const auto &m : vr.messages) std::printf("    %s\n", m.c_str());
    // warnings by kind (the message without its numbers): the first few of each, then a count, so a
    // rare kind is not hidden behind many of a common one
    std::vector<std::pair<std::string, std::vector<const std::string *>>> kinds;
    for (const std::string &w : rep.warnings) {
      std::string kind;
      for (char ch : w)
        if (!std::isdigit(static_cast<unsigned char>(ch)) && ch != '.' && ch != '-') kind += ch;
      auto it = std::find_if(kinds.begin(), kinds.end(), [&](const auto &k) { return k.first == kind; });
      if (it == kinds.end()) it = kinds.insert(kinds.end(), {kind, {}});
      it->second.push_back(&w);
    }
    for (const auto &[kind, ws] : kinds) {
      for (size_t i = 0; i < ws.size() && i < 3; ++i) std::printf("  warn: %s\n", ws[i]->c_str());
      if (ws.size() > 3) std::printf("  warn: ... %zu more like this\n", ws.size() - 3);
    }
    return vr.errors ? 1 : 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "odrforge build: %s\n", e.what());
    return 1;
  }
}

// argv[1] = input .xodr, argv[2] = output .html, options from argv[3]
static int view(int argc, char **argv) {
  if (argc < 3) {
    viewUsage();
    return 2;
  }
  VizOptions vo;
  vo.title = argv[1];
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--title" && i + 1 < argc) vo.title = argv[++i];
    else if (a == "--basemap" && i + 1 < argc) vo.basemapDir = argv[++i];
    else {
      viewUsage();
      return 2;
    }
  }
  try {
    const auto od = odr::read(argv[1]);
    const auto vr = validate(od);
    vo.validation = &vr;
    writeVizHtml(od, argv[2], vo);
    std::printf("odrforge view: %zu roads, %zu junctions -> %s (validation: %d errors, %d warnings)\n", od.roads.size(),
                od.junctions.size(), argv[2], vr.errors, vr.warnings);
    for (const auto &[rule, n] : vr.ruleCounts) std::printf("    %5d  %s\n", n, rule.c_str());
    if (std::getenv("ODRFORGE_VERBOSE"))
      for (const auto &m : vr.messages) std::printf("    %s\n", m.c_str());
    return 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "odrforge view: %s\n", e.what());
    return 1;
  }
}

int main(int argc, char **argv) {
  const std::string cmd = argc > 1 ? argv[1] : "";
  auto wantsHelp = [&]() {
    for (int i = 2; i < argc; ++i)
      if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") return true;
    return false;
  };
  if (cmd == "build") {
    if (wantsHelp()) return usage(), 0;
    return build(argc - 1, argv + 1);
  }
  if (cmd == "view") {
    if (wantsHelp()) return viewUsage(), 0;
    return view(argc - 1, argv + 1);
  }
  if (cmd == "--version" || cmd == "version") {
    std::printf("odrforge %s\n", ODRFORGE_VERSION);
    return 0;
  }
  mainUsage();
  return cmd == "--help" || cmd == "-h" || cmd == "help" ? 0 : 2;
}
