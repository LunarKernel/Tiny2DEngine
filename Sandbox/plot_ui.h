#ifndef SANDBOX_PLOT_UI_H_
#define SANDBOX_PLOT_UI_H_

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "sim_ui.h"
#include "time_series.h"

namespace tiny2d::sandbox::ui {

// Selection state of one lab's time-series window; lives in the lab's
// traits object so it persists across frames.
struct TimeSeriesWindowState {
  int primary_index{0};
  bool compare{false};
  int secondary_index{1};
  // Two history cursors: armed latches once, the first time the box
  // is ticked with a non-empty primary series, and the defaults land
  // on the recorded range ends at that moment.
  bool cursors{false};
  bool cursors_armed{false};
  double cursor_a_s{0.0};
  double cursor_b_s{0.0};

  // Drops run-specific cursor state, keeping the series selection
  // (user preference). Called from the labs' run-start path to pin
  // the fresh-per-run invariant.
  void ResetRunState() {
    cursors_armed = false;
    cursor_a_s = 0.0;
    cursor_b_s = 0.0;
  }
};

// One extracted series cached against the recorded history. A rebuild
// is needed when the selected series, the history size, or the last
// sample time changes (a decimating append changes the latter two).
struct SeriesCache {
  int series_index{-1};
  std::size_t history_size{0};
  double last_time_s{-1.0};
  std::vector<TimeSeriesPoint> points;

  bool NeedsRebuild(int index, std::size_t size, double last_time) const {
    return series_index != index || history_size != size ||
           last_time_s != last_time;
  }
  void MarkRebuilt(int index, std::size_t size, double last_time) {
    series_index = index;
    history_size = size;
    last_time_s = last_time;
  }
  // Returns the cache to its never-built state; part of the labs'
  // run-start reset (defensive - today each lab entry constructs
  // fresh traits, so this pins an invariant rather than fixing a
  // reachable staleness).
  void Reset() {
    series_index = -1;
    history_size = 0;
    last_time_s = -1.0;
    points.clear();
  }
};

namespace plot_detail {

inline void DrawCurve(ImDrawList* draw_list, ImVec2 origin,
                      const PlotGeometry& geometry, ImU32 color) {
  if (geometry.polyline.size() == 1) {
    draw_list->AddCircleFilled({origin.x + geometry.polyline[0].first,
                                origin.y + geometry.polyline[0].second},
                               3.0f, color);
    return;
  }
  for (std::size_t i = 1; i < geometry.polyline.size(); ++i) {
    draw_list->AddLine({origin.x + geometry.polyline[i - 1].first,
                        origin.y + geometry.polyline[i - 1].second},
                       {origin.x + geometry.polyline[i].first,
                        origin.y + geometry.polyline[i].second},
                       color, 1.6f);
  }
}

}  // namespace plot_detail

// Draws one lab's "Time series" window: primary-series combo, optional
// compare curve with independent auto-scaling (y tick labels describe
// the primary only), latest-value texts in the curves' colors, the
// axes box with 1/2/2.5/5-ladder ticks, and a vertical marker at the
// inspect time. The window is movable, resizable, and collapsible;
// series data comes in pre-extracted (see SeriesCache).
inline void DrawTimeSeriesWindow(const char* title,
                                 const char* const* series_labels,
                                 int series_count, TimeSeriesWindowState* state,
                                 const std::vector<TimeSeriesPoint>& primary,
                                 const std::vector<TimeSeriesPoint>* secondary,
                                 double inspect_time_s) {
  constexpr ImU32 kPrimaryColor = IM_COL32(90, 170, 255, 255);
  constexpr ImU32 kSecondaryColor = IM_COL32(250, 180, 90, 255);
  constexpr ImU32 kMarkerColor = IM_COL32(240, 240, 240, 140);
  constexpr ImU32 kGridColor = IM_COL32(120, 130, 145, 60);
  constexpr ImU32 kBorderColor = IM_COL32(200, 210, 225, 180);
  constexpr ImU32 kLabelColor = IM_COL32(200, 208, 220, 255);
  constexpr ImU32 kCursorAColor = IM_COL32(120, 230, 130, 210);
  constexpr ImU32 kCursorBColor = IM_COL32(235, 120, 210, 210);

  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos({display_size.x - 392.0f, display_size.y - 330.0f},
                          ImGuiCond_Once);
  ImGui::SetNextWindowSize({380.0f, 318.0f}, ImGuiCond_Once);
  if (!ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoSavedSettings)) {
    ImGui::End();
    return;
  }

  // Defensive: persisted indices never index outside the label array,
  // even if a caller's series count ever shrinks across frames.
  state->primary_index = std::clamp(state->primary_index, 0, series_count - 1);
  state->secondary_index =
      std::clamp(state->secondary_index, 0, series_count - 1);

  ImGui::SetNextItemWidth(200.0f);
  ImGui::Combo("Series", &state->primary_index, series_labels, series_count);
  ImGui::SameLine();
  ImGui::Checkbox("Compare", &state->compare);
  if (state->compare) {
    ImGui::SetNextItemWidth(200.0f);
    ImGui::Combo("Second", &state->secondary_index, series_labels,
                 series_count);
  }
  // Its own row: the Series row is already full at the default window
  // width, and this row must render even with no samples yet.
  ImGui::Checkbox("Cursors", &state->cursors);
  if (!primary.empty()) {
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kPrimaryColor),
                       "%s: %.6g", series_labels[state->primary_index],
                       primary.back().value);
  }
  if (state->compare && secondary != nullptr && !secondary->empty()) {
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kSecondaryColor),
                       "%s: %.6g", series_labels[state->secondary_index],
                       secondary->back().value);
  }

  // History cursors: armed once on the first tick with a non-empty
  // series (defaults on the recorded ends at that moment; they do not
  // track the growing range). The recorded range IS the primary
  // series' endpoint pair - the extractors emit one point per sample.
  bool cursor_readout_valid = false;
  CursorReadout cursor_readout;
  if (state->cursors && !primary.empty()) {
    if (!state->cursors_armed) {
      state->cursor_a_s = primary.front().time_s;
      state->cursor_b_s = primary.back().time_s;
      state->cursors_armed = true;
    }
    const double range_min = primary.front().time_s;
    const double range_max = primary.back().time_s;
    SliderInputDouble("Cursor A (s)", &state->cursor_a_s, range_min, range_max,
                      "%.3f", 92.0f, 150.0f, 74.0f);
    SliderInputDouble("Cursor B (s)", &state->cursor_b_s, range_min, range_max,
                      "%.3f", 92.0f, 150.0f, 74.0f);
    // A non-finite entry resets deterministically: A to the recorded
    // start, B to the recorded end.
    if (!std::isfinite(state->cursor_a_s)) {
      state->cursor_a_s = range_min;
    }
    if (!std::isfinite(state->cursor_b_s)) {
      state->cursor_b_s = range_max;
    }
    cursor_readout_valid = ComputeCursorReadout(
        primary, state->cursor_a_s, state->cursor_b_s, &cursor_readout);
    if (cursor_readout_valid) {
      ImGui::Text("dt: %+.6g s", cursor_readout.delta_time_s);
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kPrimaryColor),
                         "A %.6g | B %.6g | delta %+.6g",
                         cursor_readout.a.value, cursor_readout.b.value,
                         cursor_readout.delta_value);
      CursorReadout secondary_readout;
      if (state->compare && secondary != nullptr && !secondary->empty() &&
          ComputeCursorReadout(*secondary, state->cursor_a_s, state->cursor_b_s,
                               &secondary_readout)) {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kSecondaryColor),
                           "A %.6g | B %.6g | delta %+.6g",
                           secondary_readout.a.value, secondary_readout.b.value,
                           secondary_readout.delta_value);
      }
    }
  }

  // Margins hold the tick labels: y labels left, time labels below.
  constexpr float kLeftMargin = 52.0f;
  constexpr float kBottomMargin = 20.0f;
  constexpr float kTopMargin = 6.0f;
  constexpr float kRightMargin = 10.0f;
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const float plot_width = avail.x - kLeftMargin - kRightMargin;
  const float plot_height = avail.y - kTopMargin - kBottomMargin;
  if (primary.empty() || plot_width < 24.0f || plot_height < 24.0f) {
    ImGui::TextDisabled(primary.empty() ? "(no samples yet)"
                                        : "(window too small)");
    ImGui::End();
    return;
  }

  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  const ImVec2 origin{cursor.x + kLeftMargin, cursor.y + kTopMargin};
  ImGui::Dummy(avail);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRect(origin, {origin.x + plot_width, origin.y + plot_height},
                     kBorderColor);

  const PlotGeometry geometry =
      BuildPlotGeometry(primary, plot_width, plot_height);
  const double time_span = geometry.time_max_s - geometry.time_min_s;
  const double value_span = geometry.value_max - geometry.value_min;

  char label[32];
  for (const double tick :
       SelectNiceTicks(geometry.time_min_s, geometry.time_max_s, 5)) {
    const float x = origin.x + static_cast<float>((tick - geometry.time_min_s) /
                                                  time_span * plot_width);
    draw_list->AddLine({x, origin.y}, {x, origin.y + plot_height}, kGridColor);
    std::snprintf(label, sizeof(label), "%.4g", tick);
    draw_list->AddText({x - 8.0f, origin.y + plot_height + 3.0f}, kLabelColor,
                       label);
  }
  for (const double tick :
       SelectNiceTicks(geometry.value_min, geometry.value_max, 4)) {
    const float y = origin.y + plot_height -
                    static_cast<float>((tick - geometry.value_min) /
                                       value_span * plot_height);
    draw_list->AddLine({origin.x, y}, {origin.x + plot_width, y}, kGridColor);
    std::snprintf(label, sizeof(label), "%.4g", tick);
    draw_list->AddText({cursor.x + 2.0f, y - 7.0f}, kLabelColor, label);
  }

  plot_detail::DrawCurve(draw_list, origin, geometry, kPrimaryColor);
  if (state->compare && secondary != nullptr && !secondary->empty()) {
    // Independent auto-scaling keeps differently-scaled quantities
    // readable; the y ticks above describe the primary only.
    plot_detail::DrawCurve(
        draw_list, origin,
        BuildPlotGeometry(*secondary, plot_width, plot_height),
        kSecondaryColor);
  }

  const double clamped_inspect =
      std::clamp(inspect_time_s, geometry.time_min_s, geometry.time_max_s);
  const float marker_x =
      origin.x + static_cast<float>((clamped_inspect - geometry.time_min_s) /
                                    time_span * plot_width);
  draw_list->AddLine({marker_x, origin.y}, {marker_x, origin.y + plot_height},
                     kMarkerColor, 1.2f);

  if (cursor_readout_valid) {
    // Cursor lines sit at the SNAPPED sample times, so the lines and
    // the readout always describe the same recorded pair.
    const auto cursor_x = [&](double snapped_time) {
      return origin.x +
             static_cast<float>((snapped_time - geometry.time_min_s) /
                                time_span * plot_width);
    };
    const float a_x = cursor_x(cursor_readout.a.time_s);
    const float b_x = cursor_x(cursor_readout.b.time_s);
    draw_list->AddLine({a_x, origin.y}, {a_x, origin.y + plot_height},
                       kCursorAColor, 1.4f);
    draw_list->AddLine({b_x, origin.y}, {b_x, origin.y + plot_height},
                       kCursorBColor, 1.4f);
    draw_list->AddText({a_x + 2.0f, origin.y + 2.0f}, kCursorAColor, "A");
    draw_list->AddText({b_x + 2.0f, origin.y + 2.0f}, kCursorBColor, "B");
  }

  ImGui::End();
}

}  // namespace tiny2d::sandbox::ui

#endif  // SANDBOX_PLOT_UI_H_
