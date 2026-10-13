// OdrForge - in-memory OpenDRIVE model (subset of ASAM OpenDRIVE 1.4 - 1.8 used by the generator)
// and the XML writer. Element / attribute names follow the standard exactly.
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "odrforge/geometry.hpp"

namespace odrforge::odr {

/// a + b*ds + c*ds^2 + d*ds^3 (width, elevation, laneOffset, superelevation records)
struct Poly3 {
  double s = 0.0, a = 0.0, b = 0.0, c = 0.0, d = 0.0;
  double eval(double ds) const { return a + ds * (b + ds * (c + ds * d)); }
  double deriv(double ds) const { return b + ds * (2.0 * c + ds * 3.0 * d); }
  /// cubic Hermite from (v0, slope0) to (v1, slope1) over length L
  static Poly3 hermite(double s, double L, double v0, double m0, double v1, double m1);
  /// C2 lateral shift from v0 to v1 over length L, level (slope and curvature 0) at both ends: four
  /// cubics whose second derivative ramps 0 -> +A -> 0 -> -A -> 0, A = 8 (v1 - v0) / L^2. A single
  /// cubic cannot be level in slope and curvature at both ends: its curvature steps there.
  static std::vector<Poly3> shiftC2(double s, double L, double v0, double v1);
};

struct RoadMark {
  double sOffset = 0.0;
  std::string type = "none";    ///< none, solid, broken, solid solid, solid broken, broken solid, broken broken, botts dots, grass, curb, edge
  std::string weight = "standard";  ///< standard, bold
  std::string color = "standard";   ///< standard, white, yellow, blue, green, red, orange
  double width = 0.12;
  std::string laneChange = "both";  ///< increase, decrease, both, none
  double height = 0.0;
};

struct LaneLinkRef {
  std::optional<int> predecessor, successor;
};

struct Lane {
  int id = 0;                    ///< >0 left, <0 right, 0 center
  std::string type = "driving";  ///< driving, sidewalk, biking, parking, shoulder, border, median, curb, entry, exit, onRamp, offRamp, ...
  bool level = false;
  LaneLinkRef link;
  std::vector<Poly3> widths;     ///< s relative to the lane section start
  std::vector<RoadMark> roadMarks;
  std::optional<double> speedMax;  ///< m/s
  std::string speedUnit = "m/s";
  std::vector<std::string> accessDeny;  ///< restricted road users (1.6 <access rule="deny" restriction=...>)
  std::optional<double> heightInner, heightOuter;  ///< curb / sidewalk raise (m) from the section start
  struct HeightChange {
    double sOffset = 0.0, inner = 0.0, outer = 0.0;
  };
  std::vector<HeightChange> heightChanges;  ///< further <height> records (sOffset > 0, ascending), e.g. lowered curbs
  std::string material;          ///< surface (asphalt, concrete, ...)
  std::string direction;         ///< 1.8 lane direction: standard, reversed, both (empty = not written)
};

struct LaneSection {
  double s = 0.0;
  std::vector<Lane> left;   ///< ordered from id 1 outwards
  Lane center;
  std::vector<Lane> right;  ///< ordered from id -1 outwards
};

struct LinkTarget {
  std::string elementType = "road";  ///< road | junction
  std::string elementId;
  std::string contactPoint;          ///< start | end (roads only)
};

struct ValidityRange {
  int fromLane = 0, toLane = 0;
};

/// OpenDRIVE 1.9 §14.3 <dependency>: the signal with this id controls this one (e.g. a stop line
/// obeyed while that traffic light is red)
struct SignalDependency {
  std::string id, type;
};

/// OpenDRIVE 1.9 §14.4 <reference>: a link from this signal to another signal or object (e.g. a
/// traffic light to the stop line where traffic stops for it)
struct SignalLink {
  std::string elementId, elementType = "signal", type;
};

struct Signal {
  std::string id, name;
  double s = 0.0, t = 0.0, zOffset = 0.0, hOffset = 0.0, height = 0.0, width = 0.0;
  bool dynamic = false;
  std::string orientation = "+";     ///< +, -, none
  std::string country;               ///< ISO 3166-1 alpha-2 (1.8) or "OpenDRIVE" catalog
  std::string countryRevision;
  std::string type, subtype = "-1";
  std::optional<double> value;
  std::string unit, text;
  std::vector<ValidityRange> validity;
  std::vector<SignalDependency> dependencies;
  /// OpenDRIVE 1.8.0 §14.8 <semantics><priority type> (XSD 1.8.1 e_signals_semantics_priority), e.g. stopLine,
  /// trafficLight, stop, yield; empty: none
  std::string semanticsPriority;
  std::vector<SignalLink> references;
};

struct SignalReference {
  std::string id;
  double s = 0.0, t = 0.0;
  std::string orientation = "+";
  std::vector<ValidityRange> validity;
};

struct ObjectCorner {
  double u = 0.0, v = 0.0, z = 0.0, height = 0.0;  ///< cornerLocal (object frame)
};

struct Object {
  std::string id, name, type, subtype;
  double s = 0.0, t = 0.0, zOffset = 0.0, hdg = 0.0, pitch = 0.0, roll = 0.0;
  double length = 0.0, width = 0.0, height = 0.0, radius = 0.0;
  std::string orientation = "none";
  std::vector<ObjectCorner> outline;  ///< closed local outline (crosswalks, islands)
  std::vector<ValidityRange> validity;
};

struct Structure {  ///< <bridge> / <tunnel> under <objects>
  std::string id, name, type;  ///< bridge: concrete, steel, brick, wood; tunnel: standard, underpass
  double s = 0.0, length = 0.0;
};

struct Road {
  std::string id, name;
  std::string junction = "-1";
  std::string rule;  ///< RHT / LHT (1.5+); empty = not written
  std::optional<LinkTarget> predecessor, successor;
  struct Type {
    double s = 0.0;
    std::string type = "town";    ///< unknown, rural, motorway, town, lowSpeed, pedestrian, bicycle, townExpressway, ...
    std::string country;
    std::optional<double> speedMax;
    std::string speedUnit = "km/h";
  };
  std::vector<Type> types;
  ReferenceLine planView;
  std::vector<Poly3> elevation;      ///< s absolute
  std::vector<Poly3> superelevation;
  std::vector<Poly3> laneOffset;
  std::vector<LaneSection> lanes;
  std::vector<Object> objects;
  std::vector<Structure> bridges, tunnels;
  std::vector<Signal> signals;
  std::vector<SignalReference> signalReferences;
  std::map<std::string, std::string> userData;  ///< written as <userData code=.. value=..>

  double length() const { return planView.length(); }
};

struct LaneLink {
  int from = 0, to = 0;
};

struct Connection {
  std::string id, incomingRoad, connectingRoad, contactPoint = "start";
  std::vector<LaneLink> laneLinks;
};

struct JunctionController {
  std::string id;
  int sequence = 0;
};

/// OpenDRIVE 1.9 §12.5 cross path (1.8+): a crossing road linking a lane of the road at its start to a
/// lane of the road at its end (each at an s on that road)
struct CrossPath {
  std::string id, crossingRoad, roadAtStart, roadAtEnd;
  struct Link {
    int from = 0;   ///< lane of roadAtStart / roadAtEnd
    double s = 0.0;  ///< on roadAtStart / roadAtEnd
    int to = 0;     ///< lane of the crossing road
  };
  Link start, end;
};

/// OpenDRIVE 1.8.0 §12.9 junction reference line (one line) and §12.11 elevation grid along it
struct JunctionGrid {
  double x = 0.0, y = 0.0, hdg = 0.0, length = 0.0;  ///< the reference line: start, heading, length
  double gridSpacing = 5.0, sStart = 0.0;
  struct Row {
    double center = 0.0;
    std::vector<double> left, right;  ///< from inside to outside
  };
  std::vector<Row> rows;  ///< at s = sStart + k gridSpacing
};

struct Junction {
  std::string id, name;
  std::string type = "default";  ///< default, direct (1.7+), virtual, crossing (1.8)
  /// virtual junctions (OpenDRIVE 1.9 §12.7): the uncut main road, the s range and the orientation
  std::string mainRoad, orientation = "none";
  double sStart = 0.0, sEnd = 0.0;
  std::vector<Connection> connections;
  std::vector<CrossPath> crossPaths;
  std::vector<JunctionController> controllers;
  struct Priority {
    std::string high, low;
  };
  std::vector<Priority> priorities;
  std::optional<JunctionGrid> grid;  ///< common junctions, 1.8+
};

struct Controller {
  std::string id, name;
  int sequence = 0;
  std::vector<std::string> signalIds;  ///< <control signalId=..>
};

/// OpenDRIVE 1.9 §6.4.2 <license> (introduced 1.8.0): licensing of the file's content
struct License {
  std::string name, resource, spdxid, text;
};

struct Header {
  int revMajor = 1, revMinor = 6;
  std::string name, version = "1", vendor = "OdrForge", date;
  double north = 0, south = 0, east = 0, west = 0;
  std::string geoReference;  ///< PROJ string
  std::map<std::string, std::string> userData;  ///< e.g. verticalDatum
  double offX = 0, offY = 0, offZ = 0, offHdg = 0;
  std::optional<License> license;
};

/// OpenDRIVE 1.9 §12.13 junction group (1.4+): junctions that belong together, e.g. a roundabout.
struct JunctionGroup {
  std::string id, name, type = "roundabout";  ///< roundabout | unknown
  std::vector<std::string> junctionIds;
};

struct OpenDrive {
  Header header;
  std::vector<Road> roads;
  std::vector<Junction> junctions;
  std::vector<Controller> controllers;
  std::vector<JunctionGroup> junctionGroups;
};

struct WriteOptions {
  int revMinor = 6;           ///< 4, 6 or 8 (1.8 enables lane direction, ISO country codes)
  int precision = 12;         ///< significant digits (spec pitfall 15: >= 10)
};

// ---- queries
const LaneSection &sectionAt(const Road &r, double s);
double laneOffsetAt(const Road &r, double s);
double laneWidthAt(const Lane &l, double ds);
/// lateral borders of a lane at s (tInner closer to the reference line); false if the lane does not exist
bool laneBorders(const Road &r, double s, int laneId, double &tInner, double &tOuter);
/// lane centre pose at s, heading along +s
bool laneCenter(const Road &r, double s, int laneId, Pose &out);
const Lane *findLane(const LaneSection &sec, int laneId);
double elevationAt(const Road &r, double s);
double elevationSlopeAt(const Road &r, double s);
double superelevationAt(const Road &r, double s);  ///< rad; positive: the road falls to the right
/// Road surface height at (s, t): elevation + t * tan(superelevation) (lane height not included).
double surfaceZ(const Road &r, double s, double t);

/// Read an OpenDRIVE file (1.4 - 1.8; line, arc, spiral, poly3, paramPoly3; width or border lanes).
OpenDrive read(const std::string &path);

/// Write an OpenDRIVE file. Throws std::runtime_error on I/O failure.
void write(const OpenDrive &od, const std::string &path, const WriteOptions &opt = {});
std::string toString(const OpenDrive &od, const WriteOptions &opt = {});

}  // namespace odrforge::odr
