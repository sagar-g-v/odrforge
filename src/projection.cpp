#include "odrforge/projection.hpp"

#include <cmath>
#include <cstdio>

namespace odrforge {

namespace {
constexpr double a = 6378137.0;
constexpr double f = 1.0 / 298.257223563;
const double n = f / (2.0 - f);
const double A = a / (1.0 + n) * (1.0 + n * n / 4.0 + n * n * n * n / 64.0);
const double alpha[3] = {n / 2.0 - 2.0 * n * n / 3.0 + 5.0 * n * n * n / 16.0,
                         13.0 * n * n / 48.0 - 3.0 * n * n * n / 5.0, 61.0 * n * n * n / 240.0};
const double beta[3] = {n / 2.0 - 2.0 * n * n / 3.0 + 37.0 * n * n * n / 96.0,
                        n * n / 48.0 + n * n * n / 15.0, 17.0 * n * n * n / 480.0};
const double delta[3] = {2.0 * n - 2.0 * n * n / 3.0 - 2.0 * n * n * n, 7.0 * n * n / 3.0 - 8.0 * n * n * n / 5.0,
                         56.0 * n * n * n / 15.0};
constexpr double deg = kPi / 180.0;
}  // namespace

Projection::Projection(double lat0, double lon0) : lat0_(lat0), lon0_(lon0) {
  double x;
  rawForward(lat0, lon0, x, y0_);
}

void Projection::rawForward(double lat, double lon, double &x, double &y) const {
  const double phi = lat * deg, lam = (lon - lon0_) * deg;
  const double e2n = 2.0 * std::sqrt(n) / (1.0 + n);
  const double t = std::sinh(std::atanh(std::sin(phi)) - e2n * std::atanh(e2n * std::sin(phi)));
  const double xi = std::atan2(t, std::cos(lam)), eta = std::atanh(std::sin(lam) / std::sqrt(1.0 + t * t));
  double E = eta, N = xi;
  for (int j = 1; j <= 3; ++j) {
    E += alpha[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
    N += alpha[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
  }
  x = A * E;
  y = A * N;
}

Vec2 Projection::forward(double lat, double lon) const {
  double x, y;
  rawForward(lat, lon, x, y);
  return {x, y - y0_};
}

void Projection::inverse(const Vec2 &p, double &lat, double &lon) const {
  const double xi = (p.y + y0_) / A, eta = p.x / A;
  double xp = xi, ep = eta;
  for (int j = 1; j <= 3; ++j) {
    xp -= beta[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
    ep -= beta[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
  }
  const double chi = std::asin(std::sin(xp) / std::cosh(ep));
  double phi = chi;
  for (int j = 1; j <= 3; ++j) phi += delta[j - 1] * std::sin(2.0 * j * chi);
  lat = phi / deg;
  lon = lon0_ + std::atan2(std::sinh(ep), std::cos(xp)) / deg;
}

std::string Projection::projString() const {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "+proj=tmerc +lat_0=%.9g +lon_0=%.9g +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs",
                lat0_, lon0_);
  return buf;
}

}  // namespace odrforge
