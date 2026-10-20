// OdrForge - country rules (driving side, default widths / speeds / lane counts, marking colours,
// sign catalogue) and output profiles (standard OpenDRIVE 1.8, CARLA's OpenDRIVE 1.4).
#pragma once

#include <map>
#include <optional>
#include <string>

#include "odrforge/spec_revision.hpp"

namespace odrforge {

struct SignCode {
  std::string country;  ///< catalogue: ISO 3166-1 alpha-2 (e.g. "DE", "US") or "OpenDRIVE"
  std::string type, subtype = "-1";
  double width = 0.6, height = 0.6;
};

struct CountryProfile {
  std::string code = "XX";         ///< ISO 3166-1 alpha-2 ("XX" = generic metric right-hand)
  bool leftHandTraffic = false;
  std::string speedUnit = "km/h";  ///< unit of implicit / posted limits: km/h or mph
  std::map<std::string, double> laneWidth;      ///< per highway class (+ "default")
  std::map<std::string, double> defaultSpeed;   ///< per highway class, in speedUnit (urban defaults)
  std::map<std::string, int> defaultLanesOneway;   ///< lanes of a oneway when untagged
  std::map<std::string, int> defaultLanesTwoway;   ///< total lanes of a two-way road when untagged
  std::map<std::string, double> shoulderWidth;  ///< outer shoulder on rural / motorway classes
  std::map<std::string, std::string> maxspeedZones;  ///< "DE:urban" -> "50" etc.
  double sidewalkWidth = 2.0, bikeLaneWidth = 1.5, parkingWidthParallel = 2.4;
  double parkingWidthDiagonal = 4.8, parkingWidthPerpendicular = 5.2;
  /// no parking this close to a junction (the country's traffic law, rules/countries/*.json parkingClearOfJunctionSource):
  /// parking lanes from tags carry "no stopping at any time" this close to a road end at a junction
  double parkingClearOfJunction = 5.0;
  double curbHeight = 0.15;
  std::string centerLineColor = "white";    ///< yellow in the Americas
  std::string edgeLineColor = "white";
  std::string centerLineTwoWay = "broken";  ///< default separator between opposing lanes
  std::string centerLineMultiLane = "solid solid";  ///< opposing directions with >= 2 lanes each
  std::map<std::string, SignCode> signs;    ///< "stop", "give_way", "maxspeed", "traffic_light", ...

  double width(const std::string &cls) const;
  std::optional<double> speed(const std::string &cls) const;
  int lanesOneway(const std::string &cls) const;
  int lanesTwoway(const std::string &cls) const;
  /// Load rules/countries/<code>.json (falls back to XX.json for unknown codes).
  static CountryProfile load(const std::string &code, const std::string &rulesDir = "");
};

/// Output profile: what the consumer of the .xodr reads.
///   standard - ASAM OpenDRIVE 1.8 (validated against XSD 1.8.1), the full model.
///   carla    - OpenDRIVE 1.4 vocabulary plus CARLA conventions (what LibCarla parses).
/// A feature newer than the profile's revision is written as its documented older substitute
/// (see SpecRevision), e.g. a walking lane becomes a sidewalk lane in carla.
struct TargetProfile {
  std::string name = "standard";
  int revMinor = 8;
  bool openDriveSignCatalog = false;  ///< CARLA: country="OpenDRIVE", 206 stop / 205 yield / 274 speed
  bool speedSignKmhBuckets = false;   ///< CARLA: speed sign subtype 30..120 km/h in steps of 10
  /// true: traffic-light signals on the road surface (CARLA spawns poles); false: at the signal head on its pole
  /// (Junction Guideline 1.0.0 §9.2: "Traffic lights shall be defined at their physical location")
  bool trafficLightZeroOffset = false;
  size_t maxIdLength = 0;             ///< CARLA: 32
  bool writeLaneOffsetForConnecting = true;
  /// false: flat cross sections for consumers that ignore <superelevation> (LibCarla's own road
  /// meshing: ProfilesParser does not apply it). Junction surfaces are then level, so overlapping
  /// connecting lanes still agree exactly.
  bool superelevation = true;
  /// CARLA: junction ids in a range no road id uses. LibCarla (MapBuilder::GetLaneNext) reads a road link's elementId
  /// as a road whenever a road of that id exists, whatever its elementType, so a junction sharing a road's id cuts
  /// every lane into it off; OpenDRIVE itself only asks ids to be unique per element type
  bool junctionIdsApartFromRoads = false;
  SpecRevision spec() const { return {revMinor}; }
  /// "standard" or "carla". Earlier names stay accepted: asam, asam18 and esmini mean standard.
  static TargetProfile byName(const std::string &name);
};

}  // namespace odrforge
