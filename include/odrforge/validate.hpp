// OdrForge - structural and geometric validation of an OpenDRIVE model.
#pragma once

#include <string>
#include <vector>

#include "odrforge/odr.hpp"
#include "odrforge/rules.hpp"

namespace odrforge {

struct ValidationReport {
  int errors = 0, warnings = 0;
  double maxGeometryGap = 0.0;   ///< m, between consecutive planView elements
  double maxLinkGap = 0.0;       ///< m, between linked lane centres (road <-> connecting road)
  double maxLinkHeading = 0.0;   ///< rad
  std::vector<std::string> messages;
};

ValidationReport validate(const odr::OpenDrive &od, const TargetProfile &target = {});

}  // namespace odrforge
