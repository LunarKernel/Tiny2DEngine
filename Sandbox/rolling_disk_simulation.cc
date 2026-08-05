#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "rolling_disk_model.h"
#include "simulations.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;
constexpr float kRadiansToDegrees = 180.0f / kPi;
constexpr float kPhysicsStep = 1.0f / 240.0f;
constexpr double kMaximumFrameTime = 0.25;

enum class SetupAction {
  kNone,
  kStart,
  kBack,
};

bool SliderInputFloat(const char* label, float* value, float minimum,
                      float maximum, const char* format,
                      ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  constexpr float kControlStart = 310.0f;
  constexpr float kSliderWidth = 300.0f;
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

const char* GetContactModeLabel(
    tiny2d::sandbox::RollingContactMode contact_mode) {
  return contact_mode == tiny2d::sandbox::RollingContactMode::kRolling
             ? "PURE ROLLING"
             : "SLIDING";
}

const char* GetStatusLabel(tiny2d::sandbox::RollingDiskStatus status) {
  using tiny2d::sandbox::RollingDiskStatus;
  switch (status) {
    case RollingDiskStatus::kActive:
      return "ACTIVE";
    case RollingDiskStatus::kReachedTop:
      return "REACHED RAMP TOP";
    case RollingDiskStatus::kReachedBottom:
      return "REACHED RAMP BOTTOM";
    case RollingDiskStatus::kAirborne:
      return "LEFT THE RAMP";
  }
  return "UNKNOWN";
}

SetupAction DrawSetupScreen(tiny2d::sandbox::RollingDiskConfig* config) {
  using tiny2d::sandbox::RollingDiskConfig;
  using tiny2d::sandbox::RollingDiskKind;

  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(display_size);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("RollLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V11 RollLab: sliding to pure rolling");
  ImGui::TextDisabled(
      "All model values use SI units. Drag a slider or type an exact value.");

  ImGui::Spacing();
  ImGui::TextUnformatted("Ramp and rolling body");
  ImGui::Separator();
  SliderInputFloat("Ramp length L (m)", &config->ramp_length_m, 1.0f, 100.0f,
                   "%.3f", ImGuiSliderFlags_Logarithmic);
  SliderInputFloat("Ramp angle i (degrees)", &config->ramp_angle_degrees, 1.0f,
                   80.0f, "%.2f");

  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Rolling body");
  ImGui::SameLine(310.0f);
  if (ImGui::RadioButton("Solid disk",
                         config->kind == RollingDiskKind::kSolidDisk)) {
    config->kind = RollingDiskKind::kSolidDisk;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Hoop", config->kind == RollingDiskKind::kHoop)) {
    config->kind = RollingDiskKind::kHoop;
  }

  SliderInputFloat("Mass m (kg)", &config->mass_kg, 0.01f, 1000.0f, "%.4g",
                   ImGuiSliderFlags_Logarithmic);
  const float maximum_radius =
      std::max(0.02f, std::min(2.0f, config->ramp_length_m * 0.45f));
  SliderInputFloat("Radius R (m)", &config->radius_m, 0.02f, maximum_radius,
                   "%.3f", ImGuiSliderFlags_Logarithmic);
  SliderInputFloat("Initial distance from top (m)",
                   &config->initial_distance_down_ramp_m, 0.0f,
                   config->ramp_length_m, "%.3f");
  SliderInputFloat("Initial downhill velocity (m/s)",
                   &config->initial_velocity_down_ramp_m_s, -50.0f, 50.0f,
                   "%+.3f");
  SliderInputFloat("Initial rolling-positive omega (rad/s)",
                   &config->initial_angular_velocity_rad_s, -200.0f, 200.0f,
                   "%+.3f");

  ImGui::Spacing();
  ImGui::TextUnformatted("Contact");
  ImGui::Separator();
  SliderInputFloat("Static friction coefficient",
                   &config->static_friction_coefficient, 0.0f, 5.0f, "%.3f");
  SliderInputFloat("Kinetic friction coefficient",
                   &config->kinetic_friction_coefficient, 0.0f,
                   config->static_friction_coefficient, "%.3f");
  ImGui::TextDisabled(
      "Slip is v - R*omega. Pure rolling is reached when slip = 0.");

  ImGui::Spacing();
  ImGui::TextUnformatted("External fields");
  ImGui::Separator();
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Gravity g (m/s^2)");
  ImGui::SameLine(310.0f);
  if (ImGui::RadioButton("9.8", config->gravity_m_s2 == 9.8f)) {
    config->gravity_m_s2 = 9.8f;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("10", config->gravity_m_s2 == 10.0f)) {
    config->gravity_m_s2 = 10.0f;
  }
  SliderInputFloat("Charge q (C)", &config->charge_c, -1000.0f, 1000.0f,
                   "%.6g");
  ImGui::Checkbox("Enable uniform electric field",
                  &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  SliderInputFloat("Electric field E (N/C)",
                   &config->electric_field_strength_n_c, 0.0f, 1000000.0f,
                   "%.6g", ImGuiSliderFlags_Logarithmic);
  SliderInputFloat("E angle from +X, CCW (degrees)",
                   &config->electric_field_angle_degrees, -180.0f, 180.0f,
                   "%.2f");
  ImGui::EndDisabled();
  ImGui::Checkbox("Enable uniform perpendicular magnetic field",
                  &config->magnetic_field_enabled);
  ImGui::BeginDisabled(!config->magnetic_field_enabled);
  SliderInputFloat("Signed Bz (+Z out / -Z in) (T)",
                   &config->magnetic_field_z_t, -100.0f, 100.0f, "%+.4g");
  ImGui::EndDisabled();

  const char* error = tiny2d::sandbox::GetRollingDiskConfigError(*config);
  if (error == nullptr) {
    const tiny2d::sandbox::RollingDiskState initial_state =
        tiny2d::sandbox::MakeInitialRollingDiskState(*config);
    const tiny2d::sandbox::RollingDiskDerived derived =
        tiny2d::sandbox::CalculateRollingDiskDerived(*config, initial_state);
    ImGui::TextColored(
        {0.35f, 0.85f, 0.45f, 1.0f},
        "Ready | I = %.5g kg*m^2 | initial slip = %+.4g m/s | N = %.5g N",
        derived.moment_of_inertia_kg_m2, derived.slip_velocity_m_s,
        derived.normal_force_n);
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
  SetupAction action = SetupAction::kNone;
  if (ImGui::Button("Back to model selection", {button_width, 38.0f})) {
    action = SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Restore defaults", {button_width, 38.0f})) {
    *config = RollingDiskConfig{};
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Start simulation", {button_width, 38.0f})) {
    action = SetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

void DrawArrow(ImDrawList* draw_list, ImVec2 center, ImVec2 direction,
               float length, ImU32 color) {
  const float direction_length = std::hypot(direction.x, direction.y);
  if (direction_length <= 0.0f) {
    return;
  }
  direction.x /= direction_length;
  direction.y /= direction_length;
  const ImVec2 start{center.x - direction.x * length * 0.5f,
                     center.y - direction.y * length * 0.5f};
  const ImVec2 end{center.x + direction.x * length * 0.5f,
                   center.y + direction.y * length * 0.5f};
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

void DrawScene(const tiny2d::sandbox::RollingDiskConfig& config,
               const tiny2d::sandbox::RollingDiskState& state) {
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  constexpr float kMonitorRight = 430.0f;
  constexpr float kSceneMargin = 55.0f;
  constexpr float kBodyLabelWidth = 220.0f;
  const float angle = config.ramp_angle_degrees * kDegreesToRadians;
  const ImVec2 downhill{-std::cos(angle), std::sin(angle)};
  const ImVec2 outward{-std::sin(angle), -std::cos(angle)};
  const float available_width =
      std::max(160.0f, display_size.x - kMonitorRight - 2.0f * kSceneMargin -
                           kBodyLabelWidth);
  const float available_height =
      std::max(160.0f, display_size.y - 2.0f * kSceneMargin);
  const float scene_width_m =
      config.ramp_length_m * std::cos(angle) + 2.0f * config.radius_m;
  const float scene_height_m =
      config.ramp_length_m * std::sin(angle) + 2.0f * config.radius_m;
  const float pixels_per_meter =
      0.9f * std::min(available_width / std::max(0.01f, scene_width_m),
                      available_height / std::max(0.01f, scene_height_m));
  const float ramp_pixels = config.ramp_length_m * pixels_per_meter;
  const ImVec2 bottom{
      kMonitorRight + kSceneMargin +
          config.radius_m * (1.0f + std::sin(angle)) * pixels_per_meter,
      display_size.y - kSceneMargin -
          config.radius_m * (1.0f - std::cos(angle)) * pixels_per_meter};
  const ImVec2 top{bottom.x - downhill.x * ramp_pixels,
                   bottom.y - downhill.y * ramp_pixels};

  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  draw_list->AddTriangleFilled(bottom, top, {top.x, bottom.y},
                               IM_COL32(78, 96, 105, 255));
  draw_list->AddLine(top, bottom, IM_COL32(165, 185, 195, 255), 4.0f);

  const float distance =
      std::clamp(state.distance_down_ramp_m, 0.0f, config.ramp_length_m);
  const ImVec2 contact{top.x + downhill.x * distance * pixels_per_meter,
                       top.y + downhill.y * distance * pixels_per_meter};
  const float radius_pixels =
      std::max(8.0f, config.radius_m * pixels_per_meter);
  const ImVec2 center{contact.x + outward.x * radius_pixels,
                      contact.y + outward.y * radius_pixels};
  draw_list->AddCircleFilled(center, radius_pixels,
                             IM_COL32(255, 100, 105, 255), 48);
  draw_list->AddCircle(center, radius_pixels, IM_COL32(35, 40, 48, 255), 48,
                       2.0f);
  const float spoke_angle = -state.angle_radians;
  draw_list->AddLine(center,
                     {center.x + std::cos(spoke_angle) * radius_pixels * 0.82f,
                      center.y + std::sin(spoke_angle) * radius_pixels * 0.82f},
                     IM_COL32(35, 40, 48, 255), 3.0f);

  const tiny2d::sandbox::RollingDiskDerived derived =
      tiny2d::sandbox::CalculateRollingDiskDerived(config, state);
  char body_label[128];
  std::snprintf(body_label, sizeof(body_label),
                "m=%.3g kg, R=%.3g m\nv=%+.3g m/s, omega=%+.3g rad/s\n%s",
                config.mass_kg, config.radius_m, state.velocity_down_ramp_m_s,
                state.angular_velocity_rad_s,
                GetContactModeLabel(state.contact_mode));
  draw_list->AddText(
      {center.x + radius_pixels + 12.0f, center.y - radius_pixels},
      IM_COL32(255, 215, 215, 255), body_label);

  if (std::abs(derived.friction_force_n) > 0.00001f) {
    const float friction_sign = std::copysign(1.0f, derived.friction_force_n);
    DrawArrow(draw_list, center,
              {downhill.x * friction_sign, downhill.y * friction_sign}, 62.0f,
              IM_COL32(100, 205, 245, 255));
  }
  if (std::abs(derived.magnetic_force_outward_n) > 0.00001f) {
    const float magnetic_sign =
        std::copysign(1.0f, derived.magnetic_force_outward_n);
    DrawArrow(draw_list, center,
              {outward.x * magnetic_sign, outward.y * magnetic_sign}, 62.0f,
              IM_COL32(205, 125, 255, 255));
  }

  draw_list->PathArcTo(bottom, 44.0f, -angle, 0.0f, 18);
  draw_list->PathStroke(IM_COL32(235, 235, 235, 255), 0, 2.0f);
  char angle_label[32];
  std::snprintf(angle_label, sizeof(angle_label), "i = %.1f deg",
                config.ramp_angle_degrees);
  draw_list->AddText({bottom.x + 52.0f, bottom.y - 38.0f},
                     IM_COL32(235, 235, 235, 255), angle_label);

  if (config.electric_field_enabled) {
    const float field_angle =
        config.electric_field_angle_degrees * kDegreesToRadians;
    const ImVec2 field_direction{std::cos(field_angle), -std::sin(field_angle)};
    DrawArrow(draw_list, {display_size.x - 135.0f, display_size.y - 165.0f},
              field_direction, 80.0f, IM_COL32(100, 225, 150, 255));
    char field_label[88];
    std::snprintf(field_label, sizeof(field_label),
                  "E = %.4g N/C, %.2f deg from +X CCW",
                  config.electric_field_strength_n_c,
                  config.electric_field_angle_degrees);
    draw_list->AddText({display_size.x - 300.0f, display_size.y - 105.0f},
                       IM_COL32(120, 235, 165, 255), field_label);
  }

  if (config.magnetic_field_enabled) {
    constexpr float kFieldSymbolRadius = 16.0f;
    const ImVec2 field_center{display_size.x - 135.0f, display_size.y - 265.0f};
    const ImU32 field_color = IM_COL32(205, 125, 255, 255);
    draw_list->AddCircle(field_center, kFieldSymbolRadius, field_color, 32,
                         2.5f);
    if (config.magnetic_field_z_t > 0.0f) {
      draw_list->AddCircleFilled(field_center, 4.0f, field_color, 16);
    } else if (config.magnetic_field_z_t < 0.0f) {
      constexpr float kCrossOffset = 9.0f;
      draw_list->AddLine(
          {field_center.x - kCrossOffset, field_center.y - kCrossOffset},
          {field_center.x + kCrossOffset, field_center.y + kCrossOffset},
          field_color, 2.5f);
      draw_list->AddLine(
          {field_center.x - kCrossOffset, field_center.y + kCrossOffset},
          {field_center.x + kCrossOffset, field_center.y - kCrossOffset},
          field_color, 2.5f);
    }
    char magnetic_field_label[88];
    std::snprintf(magnetic_field_label, sizeof(magnetic_field_label),
                  "Bz = %+.4g T (+Z out / -Z in)", config.magnetic_field_z_t);
    draw_list->AddText({display_size.x - 300.0f, display_size.y - 225.0f},
                       field_color, magnetic_field_label);
  }

  DrawArrow(draw_list, {display_size.x - 340.0f, display_size.y - 165.0f},
            {0.0f, 1.0f}, 80.0f, IM_COL32(245, 190, 90, 255));
  char gravity_label[48];
  std::snprintf(gravity_label, sizeof(gravity_label), "g = %.3g m/s^2",
                config.gravity_m_s2);
  draw_list->AddText({display_size.x - 390.0f, display_size.y - 105.0f},
                     IM_COL32(255, 205, 115, 255), gravity_label);
}

bool DrawMonitor(const tiny2d::sandbox::RollingDiskConfig& config,
                 const tiny2d::sandbox::RollingDiskState& current_state,
                 const std::vector<tiny2d::sandbox::RollingDiskState>& history,
                 bool* paused, float* inspect_time, bool* follow_live,
                 const char* runtime_error) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({410.0f, 700.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("RollLab monitor", nullptr, kWindowFlags);

  const ImVec4 status_color =
      current_state.status == tiny2d::sandbox::RollingDiskStatus::kActive
          ? (*paused ? ImVec4{1.0f, 0.7f, 0.25f, 1.0f}
                     : ImVec4{0.35f, 0.9f, 0.5f, 1.0f})
          : ImVec4{0.35f, 0.75f, 1.0f, 1.0f};
  ImGui::TextColored(
      status_color, "%s",
      current_state.status == tiny2d::sandbox::RollingDiskStatus::kActive
          ? (*paused ? "PAUSED" : "RUNNING")
          : GetStatusLabel(current_state.status));
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
      std::max(current_state.time_seconds, kPhysicsStep);
  ImGui::SetNextItemWidth(120.0f);
  bool inspect_changed = ImGui::SliderFloat(
      "##rolling_history_slider", inspect_time, 0.0f, maximum_inspect_time,
      "%.3f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputFloat("##rolling_history_input", inspect_time,
                                       0.0f, 0.0f, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0f, current_state.time_seconds);
    *follow_live = false;
  }

  const tiny2d::sandbox::RollingDiskState* inspected =
      tiny2d::sandbox::FindRollingDiskState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  const tiny2d::sandbox::RollingDiskDerived derived =
      tiny2d::sandbox::CalculateRollingDiskDerived(config, *inspected);

  ImGui::Text("Contact: %s", GetContactModeLabel(inspected->contact_mode));
  ImGui::Text("Distance downhill: %.6f m", inspected->distance_down_ramp_m);
  ImGui::Text("Velocity: %+.6f m/s", inspected->velocity_down_ramp_m_s);
  ImGui::Text("Omega: %+.6f rad/s", inspected->angular_velocity_rad_s);
  ImGui::Text("Slip v - R*omega: %+.7f m/s", derived.slip_velocity_m_s);
  ImGui::Text("Acceleration: %+.6f m/s^2", derived.acceleration_down_ramp_m_s2);
  ImGui::Text("Angular acceleration: %+.6f rad/s^2",
              derived.angular_acceleration_rad_s2);

  ImGui::Spacing();
  ImGui::TextUnformatted("Forces and inertia");
  ImGui::Separator();
  ImGui::Text("I: %.7f kg*m^2", derived.moment_of_inertia_kg_m2);
  ImGui::Text("External tangent force: %+.6f N",
              derived.tangential_external_force_n);
  ImGui::Text("Bz: %+.6f T%s", config.magnetic_field_z_t,
              config.magnetic_field_enabled ? "" : " (disabled)");
  ImGui::Text("Magnetic force outward: %+.6f N",
              derived.magnetic_force_outward_n);
  ImGui::Text("Normal force: %.6f N", derived.normal_force_n);
  ImGui::Text("Friction force: %+.6f N", derived.friction_force_n);

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy");
  ImGui::Separator();
  ImGui::Text("Translational kinetic: %.6f J",
              derived.translational_kinetic_energy_j);
  ImGui::Text("Rotational kinetic: %.6f J",
              derived.rotational_kinetic_energy_j);
  ImGui::Text("Potential: %+.6f J", derived.potential_energy_j);
  ImGui::Text("Mechanical: %+.6f J", derived.mechanical_energy_j);
  ImGui::Text("Friction dissipated: %.6f J", inspected->dissipated_energy_j);
  ImGui::Text("Mechanical + dissipated: %+.6f J", derived.accounted_energy_j);

  if (runtime_error != nullptr) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error);
  }

  const float width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  ImGui::BeginDisabled(current_state.status !=
                       tiny2d::sandbox::RollingDiskStatus::kActive);
  if (ImGui::Button(*paused ? "Resume" : "Pause", {width, 36.0f})) {
    *paused = !*paused;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  const bool stop = ImGui::Button("Stop and choose model", {width, 36.0f});

  ImGui::End();
  return stop;
}

}  // namespace

namespace tiny2d::sandbox {

SimulationResult RunRollingDiskSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  RollingDiskConfig config;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  // ponytail: Keep one short laboratory run in memory; cap only when long
  // sessions become a real use case.
  std::vector<RollingDiskState> history;
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
                 runtime_error == nullptr &&
                 state.status == RollingDiskStatus::kActive) {
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
      const SetupAction action = DrawSetupScreen(&config);
      if (action == SetupAction::kBack) {
        return_requested = true;
      } else if (action == SetupAction::kStart) {
        state = MakeInitialRollingDiskState(config);
        history.clear();
        history.push_back(state);
        simulation_started = true;
        paused = state.status != RollingDiskStatus::kActive;
        follow_live = true;
        inspect_time = 0.0f;
        runtime_error = state.status == RollingDiskStatus::kAirborne
                            ? "The initial compound-field configuration has "
                              "no ramp contact."
                            : nullptr;
        previous_time = current_time;
      }
    } else if (simulation_started) {
      if (!return_requested && !paused && runtime_error == nullptr &&
          state.status == RollingDiskStatus::kActive) {
        const double frame_time = std::min(
            static_cast<double>(current_time - previous_time) / frequency,
            kMaximumFrameTime);
        previous_time = current_time;
        accumulated_time += frame_time;
        while (accumulated_time >= kPhysicsStep) {
          if (!StepRollingDisk(config, kPhysicsStep, &state)) {
            if (state.status == RollingDiskStatus::kAirborne) {
              if (!history.empty() &&
                  state.time_seconds <= history.back().time_seconds) {
                history.back() = state;
              } else {
                history.push_back(state);
              }
              runtime_error =
                  "The compound fields caused the body to lose ramp contact.";
            } else {
              runtime_error = GetRollingDiskStateError(config, state);
            }
            if (runtime_error == nullptr) {
              runtime_error =
                  "The fixed-step rolling model rejected this state.";
            }
            paused = true;
            accumulated_time = 0.0;
            break;
          }
          history.push_back(state);
          accumulated_time -= kPhysicsStep;
          if (state.status != RollingDiskStatus::kActive) {
            paused = true;
            accumulated_time = 0.0;
            break;
          }
        }
      } else {
        previous_time = current_time;
        accumulated_time = 0.0;
      }

      DrawScene(config, state);
      if (!return_requested &&
          DrawMonitor(config, state, history, &paused, &inspect_time,
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
