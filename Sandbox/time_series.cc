#include "time_series.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace tiny2d::sandbox {
namespace {

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

// The padded half-span for a degenerate (zero-span) axis: mirrors the
// contract's constant-value rule on both axes.
double DegeneratePad(double magnitude) {
  return std::max(0.5, 0.05 * std::abs(magnitude));
}

}  // namespace

PlotGeometry BuildPlotGeometry(const std::vector<TimeSeriesPoint>& points,
                               float width_px, float height_px) {
  Require(std::isfinite(width_px) && width_px > 0.0f &&
              std::isfinite(height_px) && height_px > 0.0f,
          "Plot dimensions must be finite and positive.");
  for (std::size_t i = 0; i < points.size(); ++i) {
    Require(std::isfinite(points[i].time_s) && std::isfinite(points[i].value),
            "Series points must be finite.");
    Require(i == 0 || points[i].time_s > points[i - 1].time_s,
            "Series times must be strictly increasing.");
  }

  PlotGeometry geometry;
  if (points.empty()) {
    return geometry;
  }

  const double raw_time_min = points.front().time_s;
  const double raw_time_max = points.back().time_s;
  if (raw_time_max > raw_time_min) {
    geometry.time_min_s = raw_time_min;
    geometry.time_max_s = raw_time_max;
  } else {
    // Single point (equal times of distinct points already threw): pad
    // so the x mapping never divides by zero and the point sits
    // mid-plot.
    const double pad = DegeneratePad(raw_time_min);
    geometry.time_min_s = raw_time_min - pad;
    geometry.time_max_s = raw_time_min + pad;
  }

  double raw_value_min = points.front().value;
  double raw_value_max = points.front().value;
  for (const TimeSeriesPoint& point : points) {
    raw_value_min = std::min(raw_value_min, point.value);
    raw_value_max = std::max(raw_value_max, point.value);
  }
  if (raw_value_max > raw_value_min) {
    const double pad = 0.05 * (raw_value_max - raw_value_min);
    geometry.value_min = raw_value_min - pad;
    geometry.value_max = raw_value_max + pad;
  } else {
    const double pad = DegeneratePad(raw_value_min);
    geometry.value_min = raw_value_min - pad;
    geometry.value_max = raw_value_min + pad;
  }

  const double time_span = geometry.time_max_s - geometry.time_min_s;
  const double value_span = geometry.value_max - geometry.value_min;
  const auto map_point = [&](const TimeSeriesPoint& point) {
    const double x =
        (point.time_s - geometry.time_min_s) / time_span * width_px;
    const double y =
        height_px - (point.value - geometry.value_min) / value_span * height_px;
    return std::pair<float, float>(static_cast<float>(x),
                                   static_cast<float>(y));
  };

  const std::size_t columns = static_cast<std::size_t>(width_px);
  if (columns == 0 || points.size() <= columns) {
    geometry.polyline.reserve(points.size());
    for (const TimeSeriesPoint& point : points) {
      geometry.polyline.push_back(map_point(point));
    }
    return geometry;
  }

  // Min/max envelope: for each pixel column, keep the extreme samples
  // (at most two, ordered by time) so single-sample spikes survive
  // downsampling. Monotonic times make column indices monotonic.
  std::size_t range_begin = 0;
  while (range_begin < points.size()) {
    const auto column_of = [&](std::size_t index) {
      const double x =
          (points[index].time_s - geometry.time_min_s) / time_span * width_px;
      return std::min(columns - 1, static_cast<std::size_t>(std::max(0.0, x)));
    };
    const std::size_t column = column_of(range_begin);
    std::size_t range_end = range_begin;
    std::size_t min_index = range_begin;
    std::size_t max_index = range_begin;
    while (range_end < points.size() && column_of(range_end) == column) {
      if (points[range_end].value < points[min_index].value) {
        min_index = range_end;
      }
      if (points[range_end].value > points[max_index].value) {
        max_index = range_end;
      }
      ++range_end;
    }
    const std::size_t first = std::min(min_index, max_index);
    const std::size_t second = std::max(min_index, max_index);
    geometry.polyline.push_back(map_point(points[first]));
    if (second != first) {
      geometry.polyline.push_back(map_point(points[second]));
    }
    range_begin = range_end;
  }
  return geometry;
}

std::vector<double> SelectNiceTicks(double minimum, double maximum,
                                    int target_count) {
  Require(std::isfinite(minimum) && std::isfinite(maximum) && maximum > minimum,
          "Tick ranges must be finite with maximum above minimum.");
  Require(target_count >= 2 && target_count <= 20,
          "Tick target counts must be in [2, 20].");

  const double span = maximum - minimum;
  const double raw_step = span / target_count;
  const double base = std::pow(10.0, std::floor(std::log10(raw_step)));
  double step = 10.0 * base;
  for (const double multiplier : {1.0, 2.0, 2.5, 5.0}) {
    const double candidate = multiplier * base;
    if (span / candidate <= target_count) {
      step = candidate;
      break;
    }
  }

  std::vector<double> ticks;
  const double tolerance = step * 1e-9;
  double tick = std::ceil((minimum - tolerance) / step) * step;
  while (tick <= maximum + tolerance) {
    // Normalize exact zeros so labels never read "-0".
    ticks.push_back(std::abs(tick) < tolerance ? 0.0 : tick);
    tick += step;
  }
  return ticks;
}

}  // namespace tiny2d::sandbox
