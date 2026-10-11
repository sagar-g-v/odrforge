// xodrviz - interactive HTML view of any OpenDRIVE file (OdrForge reader + viewer).
#include <cstdio>
#include <cstdlib>
#include <string>

#include "odrforge/validate.hpp"
#include "odrforge/version.hpp"
#include "odrforge/viz.hpp"

int main(int argc, char **argv) {
  if (argc < 3) {
    std::printf("xodrviz %s - OpenDRIVE to interactive HTML\nusage: xodrviz <in.xodr> <out.html> [--title T]\n", ODRFORGE_VERSION);
    return 2;
  }
  odrforge::VizOptions vo;
  vo.title = argv[1];
  for (int i = 3; i + 1 < argc; ++i)
    if (std::string(argv[i]) == "--title") vo.title = argv[++i];
  try {
    const auto od = odrforge::odr::read(argv[1]);
    const auto vr = odrforge::validate(od);
    vo.validation = &vr;
    odrforge::writeVizHtml(od, argv[2], vo);
    std::printf("xodrviz: %zu roads, %zu junctions -> %s (validation: %d errors, %d warnings)\n", od.roads.size(),
                od.junctions.size(), argv[2], vr.errors, vr.warnings);
    for (const auto &[rule, n] : vr.ruleCounts) std::printf("    %5d  %s\n", n, rule.c_str());
    if (std::getenv("ODRFORGE_VERBOSE"))
      for (const auto &m : vr.messages) std::printf("    %s\n", m.c_str());
    return 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "xodrviz: %s\n", e.what());
    return 1;
  }
}
