#ifndef SANDBOX_PLOT_UI_H_
#define SANDBOX_PLOT_UI_H_

#include <imgui.h>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "time_series.h"

namespace tiny2d::sandbox::ui {

// Selection state of one lab's time-series window; lives in the lab's
// traits object so it persists across frames.
struct TimeSeriesWindowState {
  int primary_index{0};
  bool compare{false};
  int secondary_index{1};
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

  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos({display_size.x - 392.0f, display_size.y - 330.0f},
                          ImGuiCond_Once);
  ImGui::SetNextWindowSize({380.0f, 318.0f}, ImGuiCond_Once);
  if (!ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoSavedSettings)) {
    ImGui::End();
    return;
  }

  ImGui::SetNextItemWidth(200.0f);
  ImGui::Combo("Series", &state->primary_index, series_labels, series_count);
  ImGui::SameLine();
  ImGui::Checkbox("Compare", &state->compare);
  if (state->compare) {
    ImGui::SetNextItemWidth(200.0f);
    ImGui::Combo("Second", &state->secondary_index, series_labels,
                 series_count);
  }
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

  ImGui::End();
}

}  // namespace tiny2d::sandbox::ui

#endif  // SANDBOX_PLOT_UI_H_
