#ifndef TINY2DENGINE_TESTS_TEST_SUPPORT_H_
#define TINY2DENGINE_TESTS_TEST_SUPPORT_H_

#include <cstddef>
#include <cstdlib>
#include <iostream>

// Shared assertion harness for every test suite. A failed CHECK prints
// file:line and the failing expression, then exits nonzero so CTest reports
// the suite as failed. Numeric tolerances stay in each suite because they
// are per-model physics decisions, not harness decisions.

namespace tiny2d::test {

inline std::size_t& CheckCount() {
  static std::size_t count = 0;
  return count;
}

inline void Check(bool condition, const char* expression, const char* file,
                  int line) {
  ++CheckCount();
  if (!condition) {
    std::cerr << file << ':' << line << ": CHECK failed: " << expression
              << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace tiny2d::test

#define CHECK(expression) \
  ::tiny2d::test::Check((expression), #expression, __FILE__, __LINE__)

#endif  // TINY2DENGINE_TESTS_TEST_SUPPORT_H_
