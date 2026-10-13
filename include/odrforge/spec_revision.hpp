// OdrForge - the OpenDRIVE revision a file is written in, and what that revision allows.
//
// Writers ask a named question ("may a lane carry @direction?") instead of comparing revMinor.
// Each answer cites where the feature is specified and the revision that introduced or
// deprecated it. Sections are ASAM OpenDRIVE 1.9.0 (docs/reference/asam_opendrive_1.9.0);
// "1.8.0 Table N" is the change table in its revision history (16_annexes/revision_history).
#pragma once

namespace odrforge {

struct SpecRevision {
  int minor = 8;  ///< header@revMinor (revMajor is always 1)

  // OpenDRIVE 1.9 §10.1 road@rule (RHT / LHT); introduced 1.5, RHT when absent.
  bool hasRoadRule() const { return minor >= 5; }

  // OpenDRIVE 1.9 §8.5 header <offset>; introduced 1.6.
  bool hasHeaderOffset() const { return minor >= 6; }

  // OpenDRIVE 1.9 §10.4 road types townExpressway, townCollector, townArterial, townPrivate,
  // townLocal, townPlayStreet; introduced 1.5 (1.4: town).
  bool hasTownRoadSubtypes() const { return minor >= 5; }

  // OpenDRIVE 1.9 §10.4 road <type @country>; introduced 1.6.
  bool hasRoadTypeCountry() const { return minor >= 6; }

  // OpenDRIVE 1.9 §11.8 lane <access @rule allow|deny>; introduced 1.6 (1.4: deny implied).
  bool hasAccessRule() const { return minor >= 6; }

  // OpenDRIVE 1.9 §11.8 / Annex A.3.5 e_accessRestrictionType with vehicle classes (passengerCar,
  // truck, bus, ...): listed in 1.7 (Table 109); 1.4 knows only simulator, autonomous traffic,
  // pedestrian and none (1.4H XSD). Written from 1.6 on, with the access@rule they come with.
  bool hasVehicleAccessRestrictions() const { return minor >= 6; }

  // OpenDRIVE 1.9 §11.8 <access><restriction type/></access>; 1.8.0 Table 227 deprecates
  // access@restriction in favour of these child elements.
  bool hasAccessRestrictionElements() const { return minor >= 8; }

  // OpenDRIVE 1.9 §11.8 lane@direction; 1.8.0 Table 228 introduces it and Table 256 deprecates
  // the lane types it replaces: bidirectional -> driving + direction="both", sidewalk -> walking.
  bool hasLaneDirection() const { return minor >= 8; }

  // 1.8.0 Table 231: center-lane @type and @level are deprecated (the center lane has no width).
  bool centerLaneHasTypeAndLevel() const { return minor < 8; }

  // OpenDRIVE 1.9 §13.2 <outlines> wrapper with outline / corner ids; 1.4 has a bare <outline>.
  bool hasOutlinesWrapper() const { return minor >= 5; }

  // OpenDRIVE 1.7 §10 junction@type: optional with default "default", mandatory for direct and
  // virtual junctions. Before 1.7 only non-default types are written.
  bool writesDefaultJunctionType() const { return minor >= 7; }

  // 1.8.0 Table 261 deprecates object type streetLamp: a lamp is type pole, subtype streetLamp.
  bool hasStreetLampAsPoleSubtype() const { return minor >= 8; }
};

}  // namespace odrforge
