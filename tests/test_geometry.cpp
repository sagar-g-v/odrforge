#include <cstring>
#include <random>

#include "odrforge/clothoid_fit.hpp"
#include "odrforge/geometry.hpp"
#include "test.hpp"

using namespace odrforge;

TEST_CASE("arc element matches circle") {
  GeomElement e;
  e.hdg = 0.0;
  e.k0 = e.k1 = 0.01;  // R = 100
  e.length = kPi * 100.0 / 2.0;
  const Pose p = e.end();
  CHECK_NEAR(p.x, 100.0, 1e-9);
  CHECK_NEAR(p.y, 100.0, 1e-9);
  CHECK_NEAR(p.hdg, kPi / 2, 1e-12);
}

TEST_CASE("spiral evaluation consistent with fine integration") {
  GeomElement e;
  e.hdg = 0.3;
  e.k0 = -0.01;
  e.k1 = 0.05;
  e.length = 80.0;
  double x = 0, y = 0;
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    const double s = (i + 0.5) * e.length / n;
    const double h = e.hdg + e.k0 * s + 0.5 * (e.k1 - e.k0) / e.length * s * s;
    x += std::cos(h) * e.length / n;
    y += std::sin(h) * e.length / n;
  }
  const Pose p = e.end();
  CHECK_NEAR(p.x, x, 1e-6);
  CHECK_NEAR(p.y, y, 1e-6);
}

TEST_CASE("G1 clothoid hits end pose") {
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  int fails = 0;
  for (int i = 0; i < 2000; ++i) {
    const Vec2 p0(0, 0), p1(30 * U(rng) + 40, 30 * U(rng));
    const double h0 = 1.2 * U(rng), h1 = 1.2 * U(rng);
    auto e = fitClothoidG1(p0, h0, p1, h1);
    if (!e) {
      ++fails;
      continue;
    }
    const Pose q = e->end();
    CHECK_NEAR(q.x, p1.x, 1e-6);
    CHECK_NEAR(q.y, p1.y, 1e-6);
    CHECK_NEAR(normalizeAngle(q.hdg - h1), 0.0, 1e-8);
  }
  CHECK(fails == 0);
}

TEST_CASE("polyline on a circle becomes one arc") {
  std::vector<Vec2> pts;
  for (int i = 0; i <= 20; ++i) {
    const double a = i * kPi / 40.0;  // quarter circle R = 60
    pts.emplace_back(60 * std::sin(a), 60 - 60 * std::cos(a));
  }
  PolylineFitOptions o;
  o.noiseTolerance = 0.01;
  const ReferenceLine L = fitPolyline(pts, o);
  double maxDev = 0, maxK = 0, minK = 1;
  for (double s = 0; s < L.length(); s += 0.5) {
    const Pose p = L.eval(s);
    maxDev = std::max(maxDev, std::abs((p.pos() - Vec2(0, 60)).norm() - 60.0));
    maxK = std::max(maxK, p.kappa);
    minK = std::min(minK, p.kappa);
  }
  CHECK(maxDev < 0.02);
  CHECK_NEAR(L.length(), 60 * kPi / 2, 0.05);
  CHECK(maxK - minK < 2e-3);
}

TEST_CASE("straight noisy polyline becomes a line") {
  std::vector<Vec2> pts;
  std::mt19937 rng(3);
  std::normal_distribution<double> N(0, 0.05);
  for (int i = 0; i <= 15; ++i) pts.emplace_back(i * 10.0, N(rng));
  const ReferenceLine L = fitPolyline(pts);
  CHECK(L.elements.size() == 1);
  CHECK(L.elements[0].type() == GeomType::Line);
}

TEST_CASE("L corner gets a smooth fillet, G2 continuous") {
  std::vector<Vec2> pts = {{0, 0}, {50, 0}, {50, 50}};
  PolylineFitOptions o;
  o.minCornerRadius = 12.0;
  o.maxCornerDeviation = 0.0;  // the design radius alone (the corner tolerance is tested separately)
  const ReferenceLine L = fitPolyline(pts, o);
  // continuity of position, heading and curvature between elements
  for (size_t i = 1; i < L.elements.size(); ++i) {
    const Pose a = L.elements[i - 1].end(), b = L.elements[i].start();
    CHECK_NEAR(a.x, b.x, 1e-6);
    CHECK_NEAR(a.y, b.y, 1e-6);
    CHECK_NEAR(normalizeAngle(a.hdg - b.hdg), 0.0, 1e-9);
    CHECK_NEAR(a.kappa, L.elements[i].k0, 1e-4);
  }
  const Pose e = L.eval(L.length());
  CHECK_NEAR(e.x, 50.0, 1e-6);
  CHECK_NEAR(e.y, 50.0, 1e-6);
  double maxK = 0;
  for (double s = 0; s < L.length(); s += 0.25) maxK = std::max(maxK, std::abs(L.eval(s).kappa));
  CHECK(maxK < 1.0 / 6.0);  // no kink
}

TEST_CASE("project / slice / reverse round trip") {
  std::vector<Vec2> pts = {{0, 0}, {40, 5}, {80, 20}, {120, 22}};
  const ReferenceLine L = fitPolyline(pts);
  double s, t;
  const Pose p = L.eval(37.0);
  L.project(p.offset(2.5), s, t);
  CHECK_NEAR(s, 37.0, 1e-6);
  CHECK_NEAR(t, 2.5, 1e-6);
  const ReferenceLine sub = L.slice(10.0, 60.0);
  CHECK_NEAR(sub.length(), 50.0, 1e-9);
  CHECK_NEAR(sub.eval(25.0).x, L.eval(35.0).x, 1e-6);
  const ReferenceLine r = L.reversed();
  CHECK_NEAR(r.eval(0).x, L.eval(L.length()).x, 1e-6);
  CHECK_NEAR(r.eval(r.length()).y, L.eval(0).y, 1e-6);
}

TEST_CASE("project with the line's projection aid given: the same s and t, bit for bit") {
  // random winding polylines fitted into lines, arcs and spirals; points near, beside and far from them
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  bool same = true, fallback = true;
  int projected = 0;
  for (int round = 0; round < 20; ++round) {
    std::vector<Vec2> pts{{0, 0}};
    double h = 0.0;
    for (int k = 0; k < 12; ++k) {
      h += 0.6 * u(rng);
      const double len = 15.0 + 25.0 * (u(rng) + 1.0);
      pts.push_back(pts.back() + Vec2(std::cos(h), std::sin(h)) * len);
    }
    const ReferenceLine L = fitPolyline(pts);
    const ReferenceLine::ProjectionAid mids = L.projectionAid();
    same = same && mids.mids.size() == L.elements.size() && mids.first.size() == L.elements.size() + 1 &&
           mids.first.back() == mids.points.size();
    for (int k = 0; k < 200; ++k) {
      const Pose p = L.eval((u(rng) + 1.0) * 0.5 * L.length() * 1.2 - 0.1 * L.length());
      const Vec2 q = p.offset(30.0 * u(rng)) + Vec2(5.0 * u(rng), 5.0 * u(rng));
      double s0, t0, s1, t1, s2, t2;
      L.project(q, s0, t0);
      L.project(q, s1, t1, mids);
      L.project(q, s2, t2, ReferenceLine::ProjectionAid{});  // an aid of another size: computed as without one
      same = same && std::memcmp(&s0, &s1, sizeof s0) == 0 && std::memcmp(&t0, &t1, sizeof t0) == 0;
      fallback = fallback && std::memcmp(&s0, &s2, sizeof s0) == 0 && std::memcmp(&t0, &t2, sizeof t0) == 0;
      ++projected;
    }
  }
  CHECK(projected == 4000);
  CHECK(same);
  CHECK(fallback);
}

