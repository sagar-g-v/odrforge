// OdrForge - road cross-section from OSM tags (lanes, turn lanes, widths, sidewalks, cycle lanes,
// parking, shoulders, bus lanes, speeds, placement) and its OpenDRIVE lane-section form.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "odrforge/odr.hpp"
#include "odrforge/osm.hpp"
#include "odrforge/rules.hpp"

namespace odrforge {

enum class LaneKind { Driving, Bidirectional, Bus, Biking, Parking, Shoulder, Sidewalk };

struct LaneSpec {
  LaneKind kind = LaneKind::Driving;
  double width = 3.25;
  std::vector<std::string> turns;  ///< OSM turn values (left, slight_left, through, right, merge_to_left, ...)
};

/// Lanes of one road side, ordered from the reference-line divider outwards.
struct SideSpec {
  std::vector<LaneSpec> lanes;
  std::optional<double> speed;  ///< m/s, driving lanes on this side
  bool forward = true;          ///< travels in the OSM way direction
  int drivingCount() const;
  double carriagewayWidth() const;  ///< driving + bus + bike + bidirectional lanes
  double totalWidth() const;
};

struct CrossSection {
  std::string highway;
  bool oneway = false;
  SideSpec left, right;      ///< OpenDRIVE sides relative to the OSM way direction
  double laneOffset = 0.0;   ///< position of the lane-0 divider relative to the OSM way (left +)
  bool roundabout = false;
  /// this section's laneOffset applies across its boundaries even when lanes change on one side
  /// only (lanes added beside lanes that must not move, e.g. the branch lanes before a gore)
  bool holdOffset = false;

  int drivingLanes() const { return left.drivingCount() + right.drivingCount(); }
  /// same lane layout (kinds and counts) - widths / turns may differ
  bool sameTopology(const CrossSection &o) const;
};

struct LaneOptions {
  bool assumeSidewalks = true;   ///< untagged urban streets get sidewalks (pedestrian navigation)
  bool shoulders = true;
};

/// Parse an OSM speed value ("50", "30 mph", "DE:urban", "walk", "none") into m/s.
std::optional<double> parseSpeed(const std::string &v, const CountryProfile &c);

CrossSection crossSectionFromTags(const Tags &t, const CountryProfile &c, const LaneOptions &o = {});

/// Reverse the direction of a cross section (for reversed / joined ways).
CrossSection reversedCrossSection(const CrossSection &x);

/// OpenDRIVE lane section with constant widths, road marks, types, speeds and curb heights.
odr::LaneSection makeLaneSection(const CrossSection &x, double s, const CountryProfile &c);

}  // namespace odrforge
