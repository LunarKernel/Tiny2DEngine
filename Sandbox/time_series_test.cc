#include "time_series.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "test_support.h"

namespace {

using tiny2d::sandbox::BuildPlotGeometry;
using tiny2d::sandbox::PlotGeometry;
using tiny2d::sandbox::SelectNiceTicks;
using tiny2d::sandbox::TimeSeriesPoint;

bool Near(double actual, double expected, double tolerance = 1e-12) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

void TestHandComputedMappingAndYFlip() {
  // Three points, value span 10 -> padded range [-0.5, 10.5] (5% of
  // span each side); time range [0, 4] unpadded.
  const std::vector<TimeSeriesPoint> points = {
      {0.0, 0.0}, {1.0, 10.0}, {4.0, 5.0}};
  const PlotGeometry geometry = BuildPlotGeometry(points, 100.0f, 110.0f);
  CHECK(geometry.time_min_s == 0.0);
  CHECK(geometry.time_max_s == 4.0);
  CHECK(Near(geometry.value_min, -0.5));
  CHECK(Near(geometry.value_max, 10.5));
  CHECK(geometry.polyline.size() == 3);
  // x: proportional to stored time across [0, 4] -> 0, 25, 100.
  CHECK(Near(geometry.polyline[0].first, 0.0));
  CHECK(Near(geometry.polyline[1].first, 25.0));
  CHECK(Near(geometry.polyline[2].first, 100.0));
  // y flipped: value 0 -> 110 * (1 - 0.5/11) = 105; value 10 -> 5;
  // value 5 -> 55.
  CHECK(Near(geometry.polyline[0].second, 105.0, 1e-6));
  CHECK(Near(geometry.polyline[1].second, 5.0, 1e-6));
  CHECK(Near(geometry.polyline[2].second, 55.0, 1e-6));
  // The largest value has the smallest y.
  CHECK(geometry.polyline[1].second < geometry.polyline[0].second);
}

void TestNonUniformTimesMapByStoredTime() {
  // Two gaps: 1 s then 0.001 s. The middle point lands proportionally
  // at x = 1 / 1.001 of the width, never at the halfway index.
  const std::vector<TimeSeriesPoint> points = {
      {0.0, 1.0}, {1.0, 2.0}, {1.001, 3.0}};
  const PlotGeometry geometry = BuildPlotGeometry(points, 1001.0f, 100.0f);
  CHECK(Near(geometry.polyline[1].first, 1000.0, 1e-6));
  CHECK(Near(geometry.polyline[2].first, 1001.0, 1e-6));
}

void TestConstantSeriesPadding() {
  const std::vector<TimeSeriesPoint> points = {
      {0.0, 42.0}, {1.0, 42.0}, {2.0, 42.0}};
  const PlotGeometry geometry = BuildPlotGeometry(points, 100.0f, 100.0f);
  // Constant value pads by max(0.5, 5% of 42) = 2.1 each side.
  CHECK(Near(geometry.value_min, 39.9));
  CHECK(Near(geometry.value_max, 44.1));
  // The line sits mid-plot.
  for (const auto& point : geometry.polyline) {
    CHECK(Near(point.second, 50.0, 1e-6));
  }
  // A small constant magnitude floors at 0.5.
  const std::vector<TimeSeriesPoint> small = {{0.0, 0.1}, {1.0, 0.1}};
  const PlotGeometry small_geometry = BuildPlotGeometry(small, 10.0f, 10.0f);
  CHECK(Near(small_geometry.value_min, -0.4));
  CHECK(Near(small_geometry.value_max, 0.6));
}

void TestSinglePointRendersMidPlot() {
  // The mandatory first-frame case: one recorded sample. Both axes pad
  // and the point sits mid-plot with no division by zero.
  const std::vector<TimeSeriesPoint> points = {{0.0, 3.0}};
  const PlotGeometry geometry = BuildPlotGeometry(points, 200.0f, 100.0f);
  CHECK(Near(geometry.time_min_s, -0.5));
  CHECK(Near(geometry.time_max_s, 0.5));
  CHECK(Near(geometry.value_min, 3.0 - 0.5));
  CHECK(Near(geometry.value_max, 3.0 + 0.5));
  CHECK(geometry.polyline.size() == 1);
  CHECK(Near(geometry.polyline[0].first, 100.0, 1e-6));
  CHECK(Near(geometry.polyline[0].second, 50.0, 1e-6));
  // A nonzero time pads by 5% of |t| once that exceeds 0.5 s.
  const std::vector<TimeSeriesPoint> late = {{100.0, 3.0}};
  const PlotGeometry late_geometry = BuildPlotGeometry(late, 200.0f, 100.0f);
  CHECK(Near(late_geometry.time_min_s, 95.0));
  CHECK(Near(late_geometry.time_max_s, 105.0));
}

void TestEnvelopeDownsamplingKeepsSpike() {
  // 1000 samples across a 10-pixel plot with one huge spike: the
  // spike's value must survive in the polyline, and the output stays
  // within two points per column.
  std::vector<TimeSeriesPoint> points;
  points.reserve(1000);
  for (int i = 0; i < 1000; ++i) {
    points.push_back({static_cast<double>(i), 1.0});
  }
  points[500].value = 250.0;
  const PlotGeometry geometry = BuildPlotGeometry(points, 10.0f, 100.0f);
  CHECK(geometry.polyline.size() <= 20);
  float minimum_y = 1000.0f;
  for (const auto& point : geometry.polyline) {
    minimum_y = std::min(minimum_y, point.second);
  }
  // The spike maps to the padded maximum: pad = 5% of the raw span
  // 249, so the offset fraction is 12.45 / 273.9 = 1/22 of the height.
  CHECK(Near(minimum_y, 100.0 / 22.0, 1e-5));
  // Time ordering survives downsampling.
  for (std::size_t i = 1; i < geometry.polyline.size(); ++i) {
    CHECK(geometry.polyline[i].first >= geometry.polyline[i - 1].first);
  }
}

void TestEmptyInputAndDeterminism() {
  const PlotGeometry empty = BuildPlotGeometry({}, 100.0f, 100.0f);
  CHECK(empty.polyline.empty());
  CHECK(empty.time_min_s == 0.0 && empty.time_max_s == 0.0);

  const std::vector<TimeSeriesPoint> points = {
      {0.0, 1.0}, {0.5, -2.0}, {0.75, 0.25}};
  const PlotGeometry first = BuildPlotGeometry(points, 320.0f, 200.0f);
  const PlotGeometry second = BuildPlotGeometry(points, 320.0f, 200.0f);
  CHECK(first.polyline.size() == second.polyline.size());
  for (std::size_t i = 0; i < first.polyline.size(); ++i) {
    CHECK(first.polyline[i] == second.polyline[i]);
  }
  CHECK(first.value_min == second.value_min);
  CHECK(first.value_max == second.value_max);
}

void TestGeometryRejectsInvalidInput() {
  const auto expect_reject = [](const std::vector<TimeSeriesPoint>& points,
                                float width, float height) {
    bool threw = false;
    try {
      BuildPlotGeometry(points, width, height);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  expect_reject({{0.0, nan}}, 100.0f, 100.0f);
  expect_reject({{nan, 1.0}}, 100.0f, 100.0f);
  expect_reject({{0.0, 1.0}, {0.0, 2.0}}, 100.0f, 100.0f);
  expect_reject({{1.0, 1.0}, {0.5, 2.0}}, 100.0f, 100.0f);
  expect_reject({{0.0, 1.0}}, 0.0f, 100.0f);
  expect_reject({{0.0, 1.0}}, 100.0f, -1.0f);
  expect_reject({{0.0, 1.0}}, std::numeric_limits<float>::quiet_NaN(), 100.0f);
}

void TestNiceTicks() {
  // [0, 10] with target 5: the 1/2/2.5/5 ladder picks step 2 (10 / 2
  // = 5 steps fit the target), giving six ticks.
  const std::vector<double> ticks = SelectNiceTicks(0.0, 10.0, 5);
  CHECK(ticks.size() == 6);
  for (std::size_t i = 0; i < ticks.size(); ++i) {
    CHECK(Near(ticks[i], 2.0 * static_cast<double>(i)));
  }
  // A range that lands on the 2.5 rung: [0, 10] with target 4.
  const std::vector<double> quarter = SelectNiceTicks(0.0, 10.0, 4);
  CHECK(quarter.size() == 5);
  CHECK(Near(quarter[1], 2.5));
  // Clipping: ticks stay inside the range for offset intervals.
  const std::vector<double> offset = SelectNiceTicks(0.3, 9.7, 5);
  CHECK(!offset.empty());
  CHECK(offset.front() >= 0.3 - 1e-9);
  CHECK(offset.back() <= 9.7 + 1e-9);
  CHECK(Near(offset.front(), 2.0));
  // A range crossing zero never emits "-0".
  const std::vector<double> crossing = SelectNiceTicks(-0.5, 0.5, 5);
  bool has_exact_zero = false;
  for (const double tick : crossing) {
    if (tick == 0.0 && !std::signbit(tick)) {
      has_exact_zero = true;
    }
  }
  CHECK(has_exact_zero);
  // Tiny spans stay on the ladder: [0, 1e-6] target 5 -> step 2e-7.
  const std::vector<double> tiny = SelectNiceTicks(0.0, 1e-6, 5);
  CHECK(tiny.size() == 6);
  CHECK(Near(tiny[1], 2e-7));

  const auto expect_reject = [](double minimum, double maximum, int target) {
    bool threw = false;
    try {
      SelectNiceTicks(minimum, maximum, target);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };
  expect_reject(1.0, 1.0, 5);
  expect_reject(2.0, 1.0, 5);
  expect_reject(0.0, std::numeric_limits<double>::infinity(), 5);
  expect_reject(0.0, 1.0, 1);
  expect_reject(0.0, 1.0, 21);
}

struct NamedTest {
  const char* name;
  void (*function)();
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const NamedTest tests[] = {
      {"hand-computed mapping and y flip", TestHandComputedMappingAndYFlip},
      {"non-uniform times map by stored time",
       TestNonUniformTimesMapByStoredTime},
      {"constant-series padding", TestConstantSeriesPadding},
      {"single point renders mid-plot", TestSinglePointRendersMidPlot},
      {"envelope downsampling keeps spike", TestEnvelopeDownsamplingKeepsSpike},
      {"empty input and determinism", TestEmptyInputAndDeterminism},
      {"geometry rejects invalid input", TestGeometryRejectsInvalidInput},
      {"nice ticks", TestNiceTicks},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << sizeof(tests) / sizeof(tests[0]) << " tests, "
            << tiny2d::test::CheckCount() << " checks passed\n";
  return 0;
}
