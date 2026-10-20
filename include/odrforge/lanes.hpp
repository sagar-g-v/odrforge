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

/// Border: the verge between the carriageway and a sidewalk mapped as its own way, or the road-level
/// strip between the carriageway and a street-side parking area mapped apart from it
enum class LaneKind { Driving, Bidirectional, Bus, Biking, Parking, Shoulder, Border, Sidewalk };

struct LaneSpec {
  LaneKind kind = LaneKind::Driving;
  double width = 3.25;
  std::vector<std::string> turns;  ///< OSM turn values (left, slight_left, through, right, merge_to_left, ...)
  /// a lane starting (ending) at its section boundary grows from (falls to) zero over this length; 0: the
  /// network's taper length. Street-side parking bays carry the length measured from their OSM area.
  double taperIn = 0.0, taperOut = 0.0;
  /// a lane rule for the whole section (OpenDRIVE 1.8.0 §11.9 <rule>), e.g. "no stopping at any time"; empty: none
  std::string rule;
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

/// Parse an OSM length value ("3.5", "3.5 m", "12'6\"", "14 ft") into metres (0.5 - 60 m, else none).
std::optional<double> lengthTag(const std::string &v);

/// Parse an OSM speed value ("50", "30 mph", "DE:urban", "walk", "none") into m/s.
std::optional<double> parseSpeed(const std::string &v, const CountryProfile &c);

CrossSection crossSectionFromTags(const Tags &t, const CountryProfile &c, const LaneOptions &o = {});

/// What the way's own tags say about a sidewalk on one geometric side ("left"/"right"): "yes"
/// (attached), "no", "separate" (mapped as its own way) or "" (untagged).
std::string sidewalkTag(const Tags &t, const std::string &side);

/// Extend one side to a sidewalk mapped as its own way whose centre line lies `distance` from the OSM
/// way: a border (verge) fills the gap from the outermost lane, then the sidewalk of `width`. An
/// assumed sidewalk on that side is replaced. False (unchanged) when the sidewalk would lie more than
/// `maxOverlap` inside the lanes already there.
bool attachSeparateSidewalk(CrossSection &x, bool leftSide, double distance, double width, double maxOverlap = 1.0);

/// Reverse the direction of a cross section (for reversed / joined ways).
CrossSection reversedCrossSection(const CrossSection &x);

/// OpenDRIVE lane section with constant widths, road marks, types, speeds and curb heights.
odr::LaneSection makeLaneSection(const CrossSection &x, double s, const CountryProfile &c);

}  // namespace odrforge
