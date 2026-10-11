// OdrForge - local transverse Mercator projection (WGS84, Krueger series, sub-mm accuracy within
// a few hundred km of the origin). The OpenDRIVE <geoReference> PROJ string is generated to match.
#pragma once

#include <string>

#include "odrforge/geometry.hpp"

namespace odrforge {

class Projection {
 public:
  Projection() = default;
  Projection(double lat0, double lon0);
  Vec2 forward(double lat, double lon) const;  ///< WGS84 degrees -> local metres (x east, y north)
  void inverse(const Vec2 &p, double &lat, double &lon) const;
  std::string projString() const;              ///< "+proj=tmerc +lat_0=.. +lon_0=.. +k=1 ..."
  double lat0() const { return lat0_; }
  double lon0() const { return lon0_; }

 private:
  void rawForward(double lat, double lon, double &x, double &y) const;
  double lat0_ = 0.0, lon0_ = 0.0, y0_ = 0.0;
};

}  // namespace odrforge
