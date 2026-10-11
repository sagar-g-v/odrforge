// OdrForge - ASAM OpenDRIVE conformance checker.
//
// Every check is tagged with the ASAM rule id (e.g. "road.geometry.refline_no_gaps") or, where the
// standard has no rule id, with the specification section it enforces (e.g. "spec7.1.validity_order").
// The rules follow docs/research/opendrive_spec_notes.md.
#pragma once

#include <map>
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
  double maxLinkWidthDiff = 0.0; ///< m, lane width jump across a link
  double maxLinkZGap = 0.0;      ///< m, elevation jump across a link
  std::map<std::string, int> ruleCounts;  ///< violations per rule id
  std::vector<std::string> messages;
  int count(const std::string &rule) const {
    auto it = ruleCounts.find(rule);
    return it == ruleCounts.end() ? 0 : it->second;
  }
};

ValidationReport validate(const odr::OpenDrive &od, const TargetProfile &target = {});

/// Lane types allowed by the given OpenDRIVE minor revision (1.4 .. 1.8).
bool validLaneType(const std::string &type, int revMinor);
/// Lane types CARLA understands (others become "None" in CARLA).
bool carlaLaneType(const std::string &type);

}  // namespace odrforge
