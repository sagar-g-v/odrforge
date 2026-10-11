#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

namespace odrtest {
struct Case {
  const char *name;
  std::function<void()> fn;
};
std::vector<Case> &registry();
extern int failures;
struct Reg {
  Reg(const char *n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};
}  // namespace odrtest

#define ODR_CAT2(a, b) a##b
#define ODR_CAT(a, b) ODR_CAT2(a, b)
#define TEST_CASE(name)                                                   \
  static void ODR_CAT(test_fn_, __LINE__)();                              \
  static odrtest::Reg ODR_CAT(test_reg_, __LINE__)(name, ODR_CAT(test_fn_, __LINE__)); \
  static void ODR_CAT(test_fn_, __LINE__)()

#define CHECK(cond)                                                                   \
  do {                                                                                \
    if (!(cond)) {                                                                    \
      std::printf("  %s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);          \
      ++odrtest::failures;                                                            \
    }                                                                                 \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                     \
  do {                                                                                            \
    const double va_ = (a), vb_ = (b);                                                            \
    if (!(std::abs(va_ - vb_) <= (tol))) {                                                        \
      std::printf("  %s:%d: |%s - %s| = |%.9g - %.9g| > %g\n", __FILE__, __LINE__, #a, #b, va_, vb_, \
                  (double)(tol));                                                                 \
      ++odrtest::failures;                                                                        \
    }                                                                                             \
  } while (0)
