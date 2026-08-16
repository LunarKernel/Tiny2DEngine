#ifndef TINY2DENGINE_SANDBOX_TIME_SERIES_H_
#define TINY2DENGINE_SANDBOX_TIME_SERIES_H_

#include <utility>
#include <vector>

namespace tiny2d::sandbox {

// One history sample of a plotted quantity: the sample's stored
// simulation time in seconds and the SI value. Times must be strictly
// increasing across a series (ROADMAP section 12: plotting uses each
// sample's stored time, never a uniform index interval).
struct TimeSeriesPoint {
  double time_s{};
  double value{};
};

// Pixel-space plot geometry. The four range fields are the PADDED axis
// ranges the polyline was mapped through (not the raw data extrema):
// the value axis pads the data span by 5%, a constant series pads by
// max(0.5, 5% of |value|), and a degenerate time range (a single
// point) pads symmetrically by max(0.5 s, 5% of |time|). Polyline
// coordinates follow ImGui pixel conventions: x in [0, width] grows
// rightward with time, y in [0, height] grows DOWNWARD, so the largest
// value maps to the smallest y.
struct PlotGeometry {
  double time_min_s{};
  double time_max_s{};
  double value_min{};
  double value_max{};
  std::vector<std::pair<float, float>> polyline;
};

// Maps a series into pixel space. Empty input returns an empty
// geometry (empty polyline, zero ranges). When the series has more
// points than pixel columns, downsampling keeps a per-column min/max
// envelope (at most two points per column, time-ordered), so a
// single-sample spike stays visible. Throws std::invalid_argument
// (producing no output) on a non-finite time or value, non-strictly-
// increasing times, or non-finite or non-positive dimensions.
// Deterministic: equal inputs give equal outputs.
PlotGeometry BuildPlotGeometry(const std::vector<TimeSeriesPoint>& points,
                               float width_px, float height_px);

// Nearest sample by stored time, ties to the earlier sample (the same
// convention as the labs' Find*State inspect lookup). Returns nullptr
// on empty input or a non-finite query. Deterministic.
const TimeSeriesPoint* FindNearestSeriesPoint(
    const std::vector<TimeSeriesPoint>& points, double time_s);

// Two-cursor readout over recorded samples: a and b are the snapped
// samples themselves (never interpolated), deltas are b minus a.
struct CursorReadout {
  TimeSeriesPoint a;
  TimeSeriesPoint b;
  double delta_time_s{};
  double delta_value{};
};

// Snaps both cursor times to the nearest recorded samples and fills
// the readout. Returns false - leaving out untouched - on an empty
// series, a non-finite cursor time, or a null out pointer. With one
// recorded sample both cursors snap to it and every delta is zero.
bool ComputeCursorReadout(const std::vector<TimeSeriesPoint>& points,
                          double cursor_a_s, double cursor_b_s,
                          CursorReadout* out);

// Axis ticks on the 1/2/2.5/5 x 10^k ladder, clipped to [minimum,
// maximum]: the smallest ladder step producing at most target_count
// steps across the range, ticks at integer multiples of that step.
// Exact zeros are normalized (never -0). Throws std::invalid_argument
// when the range is non-finite or maximum <= minimum, or when
// target_count is outside [2, 20].
std::vector<double> SelectNiceTicks(double minimum, double maximum,
                                    int target_count);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_TIME_SERIES_H_
