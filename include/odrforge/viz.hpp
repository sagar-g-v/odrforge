// OdrForge - self-contained interactive HTML visualizer for OpenDRIVE (generated or read).
#pragma once

#include <string>

#include "odrforge/odr.hpp"
#include "odrforge/validate.hpp"

namespace odrforge {

struct VizOptions {
  std::string title = "OpenDRIVE";
  double chordError = 0.05;  ///< max sampling deviation of drawn curves (m)
  double maxStep = 20.0;     ///< max distance between samples on straight, constant-width stretches (m)
  const ValidationReport *validation = nullptr;  ///< optional: shown in the info panel
  /// optional directory written by tools/make_basemap.py (offline HD map / imagery tile pyramids);
  /// referenced relative to the HTML file, which loads only the tiles in view
  std::string basemapDir;
};

/// Write one HTML file (no external resources) with pan / zoom, layer toggles, lane / road
/// inspection, curvature heat map, signals, controllers and objects.
void writeVizHtml(const odr::OpenDrive &od, const std::string &path, const VizOptions &opt = {});

}  // namespace odrforge
