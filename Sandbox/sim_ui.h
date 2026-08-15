#ifndef SANDBOX_SIM_UI_H_
#define SANDBOX_SIM_UI_H_

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <string>

#include "csv_export.h"

namespace tiny2d::sandbox::ui {

// Writes a lab's CSV export into the process working directory as
// <slug>_<local timestamp>.csv, appending an _<n> attempt suffix
// instead of overwriting on a name collision. Returns a one-line
// status for the monitor: the written name on success, otherwise the
// failure reason.
inline std::string ExportCsvToWorkingDirectory(const std::string& slug,
                                               const std::string& csv) {
  const std::time_t now = std::time(nullptr);
  std::tm local_time{};
#ifdef _WIN32
  const bool have_local_time = localtime_s(&local_time, &now) == 0;
#else
  const bool have_local_time = localtime_r(&now, &local_time) != nullptr;
#endif
  char stamp[20];
  if (!have_local_time ||
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local_time) == 0) {
    return "Export failed: local time unavailable.";
  }
  std::string name;
  bool name_is_free = false;
  for (int attempt = 1; attempt <= 99 && !name_is_free; ++attempt) {
    name = MakeCsvFileName(slug, stamp, attempt);
    name_is_free = !std::filesystem::exists(std::filesystem::path(name));
  }
  if (!name_is_free) {
    // Never overwrite an existing export, even in the pathological
    // 99-collision limit.
    return "Export failed: too many name collisions.";
  }
  std::string error;
  if (!WriteTextFile(name, csv, &error)) {
    return "Export failed: " + error;
  }
  return "Exported " + name;
}

inline bool SliderInputFloat(const char* label, float* value, float minimum,
                             float maximum, const char* format,
                             float control_start, float slider_width,
                             float input_width,
                             ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(control_start);
  ImGui::SetNextItemWidth(slider_width);
  bool changed = ImGui::SliderFloat("##slider", value, minimum, maximum, format,
                                    flags | ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(input_width);
  changed |= ImGui::InputFloat("##input", value, 0.0f, 0.0f, format);
  if (std::isfinite(*value)) {
    *value = std::clamp(*value, minimum, maximum);
  }
  ImGui::PopID();
  return changed;
}

inline bool SliderInputDouble(const char* label, double* value, double minimum,
                              double maximum, const char* format,
                              float control_start, float slider_width,
                              float input_width,
                              ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(control_start);
  ImGui::SetNextItemWidth(slider_width);
  bool changed = ImGui::SliderScalar("##slider", ImGuiDataType_Double, value,
                                     &minimum, &maximum, format,
                                     flags | ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(input_width);
  changed |= ImGui::InputDouble("##input", value, 0.0, 0.0, format);
  if (std::isfinite(*value)) {
    *value = std::clamp(*value, minimum, maximum);
  }
  ImGui::PopID();
  return changed;
}

inline void DrawArrowFrom(ImDrawList* draw_list, ImVec2 start, ImVec2 direction,
                          float length, ImU32 color) {
  const float direction_length = std::hypot(direction.x, direction.y);
  if (direction_length <= 0.0f) {
    return;
  }
  direction.x /= direction_length;
  direction.y /= direction_length;
  const ImVec2 end{start.x + direction.x * length,
                   start.y + direction.y * length};
  draw_list->AddLine(start, end, color, 3.0f);
  const ImVec2 normal{-direction.y, direction.x};
  constexpr float kHeadLength = 13.0f;
  constexpr float kHeadWidth = 7.0f;
  draw_list->AddTriangleFilled(
      end,
      {end.x - direction.x * kHeadLength + normal.x * kHeadWidth,
       end.y - direction.y * kHeadLength + normal.y * kHeadWidth},
      {end.x - direction.x * kHeadLength - normal.x * kHeadWidth,
       end.y - direction.y * kHeadLength - normal.y * kHeadWidth},
      color);
}

inline void DrawCenteredArrow(ImDrawList* draw_list, ImVec2 center,
                              ImVec2 direction, float length, ImU32 color) {
  const float direction_length = std::hypot(direction.x, direction.y);
  if (direction_length <= 0.0f) {
    return;
  }
  direction.x /= direction_length;
  direction.y /= direction_length;
  const ImVec2 start{center.x - direction.x * length * 0.5f,
                     center.y - direction.y * length * 0.5f};
  DrawArrowFrom(draw_list, start, direction, length, color);
}

}  // namespace tiny2d::sandbox::ui

#endif  // SANDBOX_SIM_UI_H_
