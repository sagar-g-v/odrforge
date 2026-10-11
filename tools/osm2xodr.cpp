// osm2xodr - OpenStreetMap (.osm XML) to ASAM OpenDRIVE (.xodr) with OdrForge.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "odrforge/network.hpp"
#include "odrforge/validate.hpp"
#include "odrforge/version.hpp"
#include "odrforge/viz.hpp"

using namespace odrforge;

static void usage() {
  std::printf(
      "osm2xodr %s - OpenStreetMap to OpenDRIVE\n"
      "usage: osm2xodr <in.osm> <out.xodr> [options]\n"
      "  --country CC        ISO 3166-1 alpha-2 rules (driving side, widths, speeds, signs); default XX\n"
      "  --target T          asam (1.6, default) | asam18 (1.8) | carla (1.4 + CARLA conventions) | esmini\n"
      "  --origin LAT,LON    projection origin (default: centre of the data)\n"
      "  --name NAME         header name\n"
      "  --no-sidewalks      do not assume sidewalks on untagged urban streets\n"
      "  --no-furniture      no street lamps / trees\n"
      "  --viz OUT.html      also write an interactive HTML view of the result\n"
      "  --terrain G.odrz    ground surface grid (tools/make_terrain.py): real road elevation\n"
      "  --structures S.odrz bridge-deck surface grid (lidar class 17) for bridge=yes roads\n"
      "  --clip MINLAT,MINLON,MAXLAT,MAXLON  cut the network to this area (ways split at the boundary)\n"
      "  --no-superelevation flat cross sections and level junctions (for consumers that ignore\n"
      "                      <superelevation>, e.g. CARLA meshing the roads itself)\n",
      ODRFORGE_VERSION);
}

int main(int argc, char **argv) {
  if (argc < 3) {
    usage();
    return 2;
  }
  const std::string in = argv[1], out = argv[2];
  std::string country = "XX", target = "asam", name, vizPath, terrainPath, structuresPath;
  double lat0 = 1e9, lon0 = 1e9, clip[4] = {0, 0, 0, 0};
  bool haveClip = false, flat = false;
  BuildOptions opt;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--country") country = next();
    else if (a == "--target") target = next();
    else if (a == "--name") name = next();
    else if (a == "--origin") {
      const std::string v = next();
      std::sscanf(v.c_str(), "%lf,%lf", &lat0, &lon0);
    } else if (a == "--no-sidewalks") opt.lanes.assumeSidewalks = false;
    else if (a == "--no-furniture") opt.streetFurniture = false;
    else if (a == "--viz") vizPath = next();
    else if (a == "--terrain") terrainPath = next();
    else if (a == "--structures") structuresPath = next();
    else if (a == "--no-superelevation") flat = true;
    else if (a == "--clip") {
      const std::string v = next();
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf,%lf", &clip[0], &clip[1], &clip[2], &clip[3]) == 4) haveClip = true;
    }
    else {
      usage();
      return 2;
    }
  }
  try {
    const auto t0 = std::chrono::steady_clock::now();
    const OsmData osm = OsmData::readXml(in);
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
    BuildReport rep;
    const odr::OpenDrive od = buildOpenDrive(osm, proj, opt, &rep);
    odr::WriteOptions wo;
    wo.revMinor = opt.target.revMinor;
    odr::write(od, out, wo);
    const ValidationReport vr = validate(od, opt.target);
    if (!vizPath.empty()) {
      VizOptions vo;
      vo.title = opt.name;
      vo.validation = &vr;
      writeVizHtml(od, vizPath, vo);
    }
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("osm2xodr %s: %s -> %s in %.2f s\n", ODRFORGE_VERSION, in.c_str(), out.c_str(), secs);
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
    std::printf("  validation: %d errors, %d warnings\n", vr.errors, vr.warnings);
    for (const auto &[rule, n] : vr.ruleCounts) std::printf("    %5d  %s\n", n, rule.c_str());
    if (std::getenv("ODRFORGE_VERBOSE"))
      for (const auto &m : vr.messages) std::printf("    %s\n", m.c_str());
    for (size_t i = 0; i < rep.warnings.size() && i < 10; ++i) std::printf("  warn: %s\n", rep.warnings[i].c_str());
    return vr.errors ? 1 : 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "osm2xodr: %s\n", e.what());
    return 1;
  }
}
