// osm2xodr - OpenStreetMap (.osm XML) to ASAM OpenDRIVE (.xodr) with OdrForge.
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
      "  --viz OUT.html      also write an interactive HTML view of the result\n",
      ODRFORGE_VERSION);
}

int main(int argc, char **argv) {
  if (argc < 3) {
    usage();
    return 2;
  }
  const std::string in = argv[1], out = argv[2];
  std::string country = "XX", target = "asam", name, vizPath;
  double lat0 = 1e9, lon0 = 1e9;
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
    if (!name.empty()) opt.name = name;
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
    std::printf("  validation: %d errors, %d warnings\n", vr.errors, vr.warnings);
    for (size_t i = 0; i < vr.messages.size() && i < 15; ++i) std::printf("    %s\n", vr.messages[i].c_str());
    for (size_t i = 0; i < rep.warnings.size() && i < 10; ++i) std::printf("  warn: %s\n", rep.warnings[i].c_str());
    return vr.errors ? 1 : 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "osm2xodr: %s\n", e.what());
    return 1;
  }
}
