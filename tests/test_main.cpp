// Minimal self-contained test runner (no external dependency).
#include "test.hpp"

#include <cstdio>

namespace odrtest {
std::vector<Case> &registry() {
  static std::vector<Case> r;
  return r;
}
int failures = 0;
}  // namespace odrtest

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);  // a crash still shows the last case reached
  int failedCases = 0;
  for (const auto &c : odrtest::registry()) {
    const int before = odrtest::failures;
    c.fn();
    const bool ok = odrtest::failures == before;
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    failedCases += ok ? 0 : 1;
  }
  std::printf("%zu cases, %d failed\n", odrtest::registry().size(), failedCases);
  return failedCases ? 1 : 0;
}
