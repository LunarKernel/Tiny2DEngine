#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "rotation_pendulum_model.h"
#include "simulations.h"

namespace tiny2d::sandbox {
namespace {

constexpr double kPendulumMaximumFrameTime = 0.25;

enum class PendulumSetupAction {
  kNone,
  kStart,
  kBack,
};

bool PendulumSliderInputFloat(const char* label, float* value, float minimum,
                              float maximum, const char* format,
                              ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  constexpr float kControlStart = 300.0f;
  constexpr float kSliderWidth = 320.0f;
  constexpr float kInputWidth = 110.0f;

  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(kControlStart);
  ImGui::SetNextItemWidth(kSliderWidth);
  bool changed = ImGui::SliderFloat("##slider", value, minimum, maximum, format,
                                    flags | ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(kInputWidth);
  changed |= ImGui::InputFloat("##input", value, 0.0f, 0.0f, format);
  if (std::isfinite(*value)) {
    *value = std::clamp(*value, minimum, maximum);
  }
  ImGui::PopID();
  return changed;
}

PendulumSetupAction DrawPendulumSetupScreen(PendulumConfig* config) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("PivotLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "PivotLab: charged physical pendulum");
  ImGui::TextDisabled(
      "Drag a slider or type an exact SI value. Out-of-range finite values "
      "are clamped.");
  ImGui::Spacing();

  ImGui::TextUnformatted("Pendulum");
  ImGui::Separator();
  PendulumSliderInputFloat("Rod length L (m)", &config->rod_length_m,
                           kMinimumRodLength, kMaximumRodLength, "%.3f",
                           ImGuiSliderFlags_Logarithmic);
  PendulumSliderInputFloat("Rod mass M (kg)", &config->rod_mass_kg,
                           kMinimumMass, kMaximumMass, "%.3f",
                           ImGuiSliderFlags_Logarithmic);
  PendulumSliderInputFloat("Counterweight mass m (kg)",
                           &config->counterweight_mass_kg, kMinimumMass,
                           kMaximumMass, "%.3f", ImGuiSliderFlags_Logarithmic);
  const float rod_length_for_control =
      std::isfinite(config->rod_length_m)
          ? std::clamp(config->rod_length_m, kMinimumRodLength,
                       kMaximumRodLength)
          : kMaximumRodLength;
  PendulumSliderInputFloat("Counterweight distance r (m)",
                           &config->counterweight_distance_m, 0.0f,
                           rod_length_for_control, "%.3f");
  PendulumSliderInputFloat("Initial angle from vertical down, CCW (degrees)",
                           &config->initial_angle_degrees, -180.0f, 180.0f,
                           "%.2f");
  PendulumSliderInputFloat(
      "Initial angular speed (rad/s)", &config->initial_angular_velocity_rad_s,
      -kMaximumInitialAngularSpeed, kMaximumInitialAngularSpeed, "%.3f");
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Gravity g (m/s^2)");
  ImGui::SameLine(300.0f);
  if (ImGui::RadioButton("9.8", config->gravity_m_s2 == 9.8f)) {
    config->gravity_m_s2 = 9.8f;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("10", config->gravity_m_s2 == 10.0f)) {
    config->gravity_m_s2 = 10.0f;
  }

  ImGui::Spacing();
  ImGui::TextUnformatted("Uniform electric field");
  ImGui::Separator();
  ImGui::Checkbox("Enable electric field", &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  ImGui::Checkbox("Counterweight carries charge",
                  &config->counterweight_charged);
  ImGui::BeginDisabled(!config->counterweight_charged);
  PendulumSliderInputFloat(
      "Counterweight charge q (C)", &config->counterweight_charge_c,
      -kMaximumChargeMagnitude, kMaximumChargeMagnitude, "%.6g");
  ImGui::EndDisabled();
  PendulumSliderInputFloat(
      "Electric field E (N/C)", &config->electric_field_strength_n_c, 0.0f,
      kMaximumElectricField, "%.6g", ImGuiSliderFlags_Logarithmic);
  PendulumSliderInputFloat("E angle from +X, CCW (degrees)",
                           &config->electric_field_angle_degrees, -180.0f,
                           180.0f, "%.2f");
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::Checkbox("Enable linear rotational damping", &config->damping_enabled);
  ImGui::BeginDisabled(!config->damping_enabled);
  PendulumSliderInputFloat("Damping c (N*m*s/rad)",
                           &config->damping_coefficient_n_m_s, 0.0f,
                           kMaximumDamping, "%.4g");
  ImGui::EndDisabled();

  const char* error = GetPendulumConfigError(*config);
  if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | I = %.4f kg*m^2 | small-oscillation T = %.4f s",
                       GetPendulumMomentOfInertia(*config),
                       GetSmallAnglePeriod(*config));
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
  PendulumSetupAction action = PendulumSetupAction::kNone;
  if (ImGui::Button("Back to model selection", {button_width, 38.0f})) {
    action = PendulumSetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Restore defaults", {button_width, 38.0f})) {
    *config = PendulumConfig{};
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Start simulation", {button_width, 38.0f})) {
    action = PendulumSetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

void DrawPendulumArrow(ImDrawList* draw_list, ImVec2 start, ImVec2 direction,
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

void DrawPendulumScene(const PendulumConfig& config,
                       const PendulumState& state) {
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float scene_left = std::min(440.0f, display_size.x * 0.4f);
  const float scene_width = std::max(display_size.x - scene_left, 200.0f);
  const float rod_pixels =
      std::max(100.0f, std::min(scene_width, display_size.y) * 0.3f);
  const ImVec2 pivot{scene_left + scene_width * 0.52f, display_size.y * 0.45f};
  const ImVec2 rod_direction{std::sin(state.angle_radians),
                             std::cos(state.angle_radians)};
  const ImVec2 rod_end{pivot.x + rod_direction.x * rod_pixels,
                       pivot.y + rod_direction.y * rod_pixels};
  const float weight_fraction =
      config.counterweight_distance_m / config.rod_length_m;
  const ImVec2 weight_position{
      pivot.x + rod_direction.x * rod_pixels * weight_fraction,
      pivot.y + rod_direction.y * rod_pixels * weight_fraction};

  draw_list->AddLine(pivot, rod_end, IM_COL32(210, 170, 95, 255), 9.0f);
  draw_list->AddCircleFilled(weight_position, 19.0f,
                             IM_COL32(255, 100, 105, 255));
  draw_list->AddCircleFilled(pivot, 10.0f, IM_COL32(225, 230, 240, 255));
  draw_list->AddCircle(pivot, 11.0f, IM_COL32(40, 45, 55, 255), 24, 2.0f);

  constexpr std::size_t kArcPointCount = 25;
  std::array<ImVec2, kArcPointCount> arc_points{};
  const float arc_start = kPendulumPi * 0.5f;
  const float arc_end = arc_start - state.angle_radians;
  for (std::size_t i = 0; i < arc_points.size(); ++i) {
    const float ratio =
        static_cast<float>(i) / static_cast<float>(arc_points.size() - 1);
    const float angle = arc_start + (arc_end - arc_start) * ratio;
    arc_points[i] = {pivot.x + std::cos(angle) * 54.0f,
                     pivot.y + std::sin(angle) * 54.0f};
  }
  draw_list->AddPolyline(arc_points.data(), static_cast<int>(arc_points.size()),
                         IM_COL32(130, 205, 255, 255), 0, 2.0f);

  char angle_label[48];
  std::snprintf(angle_label, sizeof(angle_label), "theta = %+.2f deg",
                state.angle_radians * kPendulumRadiansToDegrees);
  draw_list->AddText({pivot.x + 62.0f, pivot.y + 36.0f},
                     IM_COL32(150, 215, 255, 255), angle_label);

  char weight_label[64];
  std::snprintf(
      weight_label, sizeof(weight_label), "m=%.3g kg, q=%.3g C",
      config.counterweight_mass_kg,
      config.counterweight_charged ? config.counterweight_charge_c : 0.0f);
  draw_list->AddText({weight_position.x + 24.0f, weight_position.y - 9.0f},
                     IM_COL32(255, 210, 210, 255), weight_label);

  if (config.electric_field_enabled) {
    const float field_angle = GetPendulumFieldAngle(config);
    const ImVec2 field_direction{std::cos(field_angle), -std::sin(field_angle)};
    constexpr float kFieldArrowLength = 80.0f;
    const ImVec2 field_center{scene_left + 105.0f, display_size.y - 120.0f};
    const ImVec2 field_start{
        field_center.x - field_direction.x * kFieldArrowLength * 0.5f,
        field_center.y - field_direction.y * kFieldArrowLength * 0.5f};
    DrawPendulumArrow(draw_list, field_start, field_direction,
                      kFieldArrowLength, IM_COL32(100, 225, 150, 255));
    char field_label[80];
    std::snprintf(field_label, sizeof(field_label),
                  "E = %.4g N/C, %.2f deg from +X CCW",
                  config.electric_field_strength_n_c,
                  config.electric_field_angle_degrees);
    draw_list->AddText({scene_left + 35.0f, display_size.y - 50.0f},
                       IM_COL32(120, 235, 165, 255), field_label);
  }

  constexpr ImVec2 kGravityDirection{0.0f, 1.0f};
  constexpr float kGravityArrowLength = 80.0f;
  const ImVec2 gravity_center{scene_left + 430.0f, display_size.y - 120.0f};
  const ImVec2 gravity_start{
      gravity_center.x - kGravityDirection.x * kGravityArrowLength * 0.5f,
      gravity_center.y - kGravityDirection.y * kGravityArrowLength * 0.5f};
  DrawPendulumArrow(draw_list, gravity_start, kGravityDirection,
                    kGravityArrowLength, IM_COL32(245, 190, 90, 255));
  char gravity_label[64];
  std::snprintf(gravity_label, sizeof(gravity_label),
                "g = %.3g m/s^2, downward (-Y)", config.gravity_m_s2);
  draw_list->AddText({scene_left + 355.0f, display_size.y - 50.0f},
                     IM_COL32(255, 205, 115, 255), gravity_label);
}

bool DrawPendulumMonitor(const PendulumConfig& config,
                         const PendulumState& current_state,
                         const std::vector<PendulumState>& history,
                         bool* paused, float* inspect_time, bool* follow_live,
                         const char* runtime_error) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({410.0f, 560.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("PivotLab monitor", nullptr, kWindowFlags);

  ImGui::TextColored(*paused ? ImVec4{1.0f, 0.7f, 0.25f, 1.0f}
                             : ImVec4{0.35f, 0.9f, 0.5f, 1.0f},
                     "%s", *paused ? "PAUSED" : "RUNNING");
  ImGui::SameLine();
  ImGui::Text("t = %.4f s", current_state.time_seconds);
  ImGui::TextDisabled("Space: pause/resume");
  ImGui::Separator();

  if (*follow_live) {
    *inspect_time = current_state.time_seconds;
  }
  if (ImGui::Checkbox("Live", follow_live) && *follow_live) {
    *inspect_time = current_state.time_seconds;
  }
  ImGui::SameLine();
  ImGui::TextUnformatted("Inspect time");
  ImGui::SameLine();
  const float maximum_inspect_time =
      std::max(current_state.time_seconds, kPendulumPhysicsStep);
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_time_changed = ImGui::SliderFloat(
      "##history_slider", inspect_time, 0.0f, maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_time_changed |=
      ImGui::InputFloat("##history_input", inspect_time, 0.0f, 0.0f, "%.3f");
  if (inspect_time_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0f, current_state.time_seconds);
    *follow_live = false;
  }

  const PendulumState* inspected_state =
      FindPendulumState(history, *inspect_time);
  if (inspected_state == nullptr) {
    inspected_state = &current_state;
  }
  const PendulumDerived derived =
      CalculatePendulumDerived(config, *inspected_state);

  ImGui::Text("theta: %+.4f deg",
              inspected_state->angle_radians * kPendulumRadiansToDegrees);
  ImGui::Text("omega: %+.6f rad/s", inspected_state->angular_velocity_rad_s);
  ImGui::Text("alpha: %+.6f rad/s^2",
              inspected_state->angular_acceleration_rad_s2);
  ImGui::Text("I: %.6f kg*m^2", derived.moment_of_inertia_kg_m2);
  ImGui::Text("Small-oscillation period: %.6f s", GetSmallAnglePeriod(config));

  ImGui::Spacing();
  ImGui::TextUnformatted("Torques (N*m)");
  ImGui::Separator();
  ImGui::Text("gravity: %+.6f", derived.gravity_torque_n_m);
  ImGui::Text("electric: %+.6f", derived.electric_torque_n_m);
  ImGui::Text("damping: %+.6f", derived.damping_torque_n_m);
  ImGui::Text("total: %+.6f", derived.total_torque_n_m);

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy (J)");
  ImGui::Separator();
  ImGui::Text("kinetic: %+.6f", derived.kinetic_energy_j);
  ImGui::Text("gravity potential: %+.6f",
              derived.gravitational_potential_energy_j);
  ImGui::Text("electric potential: %+.6f", derived.electric_potential_energy_j);
  ImGui::Text("total: %+.6f", derived.total_energy_j);

  if (runtime_error != nullptr) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error);
  }

  const float width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  if (ImGui::Button(*paused ? "Resume" : "Pause", {width, 36.0f})) {
    *paused = !*paused;
  }
  ImGui::SameLine();
  const bool stop = ImGui::Button("Stop and choose model", {width, 36.0f});
  ImGui::End();
  return stop;
}

}  // namespace

SimulationResult RunRotationPendulumSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  PendulumConfig config;
  PendulumState state = MakeInitialPendulumState(config);
  // ponytail: Match V9 and keep one run in memory; cap only for long sessions.
  std::vector<PendulumState> history;
  bool simulation_started = false;
  bool paused = false;
  bool follow_live = true;
  float inspect_time = 0.0f;
  const char* runtime_error = nullptr;
  double accumulated_time = 0.0;
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();

  while (true) {
    bool return_requested = false;
    SimulationResult return_result = SimulationResult::kBackToSelection;
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        return_requested = true;
        return_result = SimulationResult::kQuit;
      } else if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                 event.key.keysym.sym == SDLK_SPACE && simulation_started &&
                 runtime_error == nullptr) {
        paused = !paused;
      }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint64 current_time = SDL_GetPerformanceCounter();
    if (!return_requested && !simulation_started) {
      previous_time = current_time;
      accumulated_time = 0.0;
      const PendulumSetupAction action = DrawPendulumSetupScreen(&config);
      if (action == PendulumSetupAction::kBack) {
        return_requested = true;
      } else if (action == PendulumSetupAction::kStart) {
        state = MakeInitialPendulumState(config);
        history.clear();
        history.push_back(state);
        simulation_started = true;
        paused = false;
        follow_live = true;
        inspect_time = 0.0f;
        runtime_error = nullptr;
        previous_time = current_time;
      }
    } else if (simulation_started) {
      if (!return_requested && !paused && runtime_error == nullptr) {
        const double frame_time = std::min(
            static_cast<double>(current_time - previous_time) / frequency,
            kPendulumMaximumFrameTime);
        previous_time = current_time;
        accumulated_time += frame_time;
        while (accumulated_time >= kPendulumPhysicsStep) {
          if (!StepPendulum(config, kPendulumPhysicsStep, &state)) {
            runtime_error = GetPendulumStateError(config, state);
            if (runtime_error == nullptr) {
              runtime_error =
                  "The fixed-step integrator rejected an unstable state.";
            }
            paused = true;
            accumulated_time = 0.0;
            break;
          }
          history.push_back(state);
          accumulated_time -= kPendulumPhysicsStep;
        }
      } else {
        previous_time = current_time;
        accumulated_time = 0.0;
      }

      DrawPendulumScene(config, state);
      if (!return_requested &&
          DrawPendulumMonitor(config, state, history, &paused, &inspect_time,
                              &follow_live, runtime_error)) {
        return_requested = true;
      }
    }

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 18, 20, 24, 255);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);

    if (return_requested) {
      return return_result;
    }
  }
}

}  // namespace tiny2d::sandbox
