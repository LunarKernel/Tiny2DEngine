#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "lorentz_particle_model.h"
#include "simulations.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kMaximumFrameTime = 0.25;

enum class SetupAction {
  kNone,
  kStart,
  kBack,
};

bool SliderInputDouble(const char* label, double* value, double minimum,
                       double maximum, const char* format,
                       ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  constexpr float kControlStart = 310.0f;
  constexpr float kSliderWidth = 300.0f;
  constexpr float kInputWidth = 110.0f;

  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(kControlStart);
  ImGui::SetNextItemWidth(kSliderWidth);
  bool changed = ImGui::SliderScalar("##slider", ImGuiDataType_Double, value,
                                     &minimum, &maximum, format,
                                     flags | ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(kInputWidth);
  changed |= ImGui::InputDouble("##input", value, 0.0, 0.0, format);
  if (std::isfinite(*value)) {
    *value = std::clamp(*value, minimum, maximum);
  }
  ImGui::PopID();
  return changed;
}

const char* GetStatusLabel(tiny2d::sandbox::LorentzParticleStatus status) {
  using tiny2d::sandbox::LorentzParticleStatus;
  switch (status) {
    case LorentzParticleStatus::kActive:
      return "ACTIVE";
    case LorentzParticleStatus::kOutOfBounds:
      return "OUT OF BOUNDS";
  }
  return "UNKNOWN";
}

SetupAction DrawSetupScreen(tiny2d::sandbox::LorentzParticleConfig* config) {
  using tiny2d::sandbox::LorentzParticleConfig;

  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(display_size);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("OrbitLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V12 OrbitLab: charged particle in uniform E and B");
  ImGui::TextDisabled(
      "SI units; +X right, +Y up, +Z out of screen. Drag or type values.");

  ImGui::Spacing();
  ImGui::TextUnformatted("Particle and initial state");
  ImGui::Separator();
  SliderInputDouble("Mass m (kg)", &config->mass_kg, 0.01, 1000.0, "%.6g",
                    ImGuiSliderFlags_Logarithmic);
  SliderInputDouble("Charge q (C)", &config->charge_c, -1000.0, 1000.0,
                    "%+.6g");
  SliderInputDouble("Initial X (m)", &config->initial_x_m,
                    tiny2d::sandbox::kLorentzMinimumX,
                    tiny2d::sandbox::kLorentzMaximumX, "%+.3f");
  SliderInputDouble("Initial Y (m)", &config->initial_y_m,
                    tiny2d::sandbox::kLorentzMinimumY,
                    tiny2d::sandbox::kLorentzMaximumY, "%+.3f");
  SliderInputDouble("Initial speed (m/s)", &config->initial_speed_m_s, 0.0,
                    100.0, "%.4g");
  SliderInputDouble("Velocity angle from +X, CCW (degrees)",
                    &config->initial_velocity_angle_degrees, -180.0, 180.0,
                    "%+.2f");

  ImGui::Spacing();
  ImGui::TextUnformatted("Uniform fields");
  ImGui::Separator();
  ImGui::Checkbox("Enable uniform electric field",
                  &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  SliderInputDouble("Electric field E (N/C)",
                    &config->electric_field_strength_n_c, 0.0, 1000000.0,
                    "%.6g", ImGuiSliderFlags_Logarithmic);
  SliderInputDouble("E angle from +X, CCW (degrees)",
                    &config->electric_field_angle_degrees, -180.0, 180.0,
                    "%+.2f");
  ImGui::EndDisabled();
  ImGui::Checkbox("Enable uniform perpendicular magnetic field",
                  &config->magnetic_field_enabled);
  ImGui::BeginDisabled(!config->magnetic_field_enabled);
  SliderInputDouble("Signed Bz (+Z out / -Z in) (T)",
                    &config->magnetic_field_z_t, -100.0, 100.0, "%+.6g");
  ImGui::EndDisabled();

  const char* error = tiny2d::sandbox::GetLorentzParticleConfigError(*config);
  if (error == nullptr) {
    const tiny2d::sandbox::LorentzParticleState initial_state =
        tiny2d::sandbox::MakeInitialLorentzParticleState(*config);
    const tiny2d::sandbox::LorentzParticleDerived derived =
        tiny2d::sandbox::CalculateLorentzParticleDerived(*config,
                                                         initial_state);
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | speed = %.5g m/s | energy = %.5g J",
                       derived.speed_m_s, derived.total_energy_j);
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
    *config = LorentzParticleConfig{};
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

struct SceneLayout {
  ImVec2 center;
  float pixels_per_meter;
  float left;
  float right;
  float top;
  float bottom;
};

SceneLayout MakeSceneLayout() {
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  constexpr float kAvailableLeft = 445.0f;
  constexpr float kAvailableTop = 150.0f;
  constexpr float kRightMargin = 25.0f;
  constexpr float kBottomMargin = 35.0f;
  constexpr float kSceneWidthMeters = static_cast<float>(
      tiny2d::sandbox::kLorentzMaximumX - tiny2d::sandbox::kLorentzMinimumX);
  constexpr float kSceneHeightMeters = static_cast<float>(
      tiny2d::sandbox::kLorentzMaximumY - tiny2d::sandbox::kLorentzMinimumY);
  const float available_right =
      std::max(kAvailableLeft + 100.0f, display_size.x - kRightMargin);
  const float available_bottom =
      std::max(kAvailableTop + 100.0f, display_size.y - kBottomMargin);
  const float pixels_per_meter = std::max(
      1.0f, std::min((available_right - kAvailableLeft) / kSceneWidthMeters,
                     (available_bottom - kAvailableTop) / kSceneHeightMeters));
  const float width = kSceneWidthMeters * pixels_per_meter;
  const float height = kSceneHeightMeters * pixels_per_meter;
  const ImVec2 center{(kAvailableLeft + available_right) * 0.5f,
                      (kAvailableTop + available_bottom) * 0.5f};
  return {center,
          pixels_per_meter,
          center.x - width * 0.5f,
          center.x + width * 0.5f,
          center.y - height * 0.5f,
          center.y + height * 0.5f};
}

ImVec2 ToScreen(const SceneLayout& layout, double x_m, double y_m) {
  return {layout.center.x + static_cast<float>(x_m) * layout.pixels_per_meter,
          layout.center.y - static_cast<float>(y_m) * layout.pixels_per_meter};
}

void DrawMagneticFieldSymbol(ImDrawList* draw_list, ImVec2 center,
                             double magnetic_field_z_t) {
  constexpr float kRadius = 16.0f;
  constexpr float kCrossOffset = 9.0f;
  const ImU32 color = IM_COL32(205, 125, 255, 255);
  draw_list->AddCircle(center, kRadius, color, 32, 2.5f);
  if (magnetic_field_z_t > 0.0f) {
    draw_list->AddCircleFilled(center, 4.0f, color, 16);
  } else if (magnetic_field_z_t < 0.0f) {
    draw_list->AddLine({center.x - kCrossOffset, center.y - kCrossOffset},
                       {center.x + kCrossOffset, center.y + kCrossOffset},
                       color, 2.5f);
    draw_list->AddLine({center.x - kCrossOffset, center.y + kCrossOffset},
                       {center.x + kCrossOffset, center.y - kCrossOffset},
                       color, 2.5f);
  }
}

void DrawScene(
    const tiny2d::sandbox::LorentzParticleConfig& config,
    const tiny2d::sandbox::LorentzParticleState& displayed_state,
    const std::vector<tiny2d::sandbox::LorentzParticleState>& history) {
  using tiny2d::sandbox::LorentzParticleStatus;

  const SceneLayout layout = MakeSceneLayout();
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImU32 border_color = IM_COL32(150, 170, 185, 255);
  const ImU32 axis_color = IM_COL32(85, 100, 112, 255);
  const ImU32 label_color = IM_COL32(205, 215, 225, 255);

  draw_list->AddText({layout.left, layout.top - 72.0f},
                     IM_COL32(95, 190, 255, 255),
                     "OrbitLab | 20 m x 12 m physical plane");
  draw_list->AddRect({layout.left, layout.top}, {layout.right, layout.bottom},
                     border_color, 0.0f, 0, 2.0f);
  draw_list->AddLine({layout.left, layout.center.y},
                     {layout.right, layout.center.y}, axis_color, 1.5f);
  draw_list->AddLine({layout.center.x, layout.top},
                     {layout.center.x, layout.bottom}, axis_color, 1.5f);
  draw_list->AddText({layout.right - 28.0f, layout.center.y + 7.0f},
                     label_color, "+X");
  draw_list->AddText({layout.center.x + 7.0f, layout.top + 5.0f}, label_color,
                     "+Y");

  char x_range_label[48];
  std::snprintf(x_range_label, sizeof(x_range_label), "%.0f m",
                tiny2d::sandbox::kLorentzMinimumX);
  draw_list->AddText({layout.left + 5.0f, layout.center.y + 7.0f}, label_color,
                     x_range_label);
  std::snprintf(x_range_label, sizeof(x_range_label), "+%.0f m",
                tiny2d::sandbox::kLorentzMaximumX);
  draw_list->AddText({layout.right - 45.0f, layout.center.y + 7.0f},
                     label_color, x_range_label);

  if (history.size() > 1) {
    ImVec2 previous =
        ToScreen(layout, history.front().x_m, history.front().y_m);
    for (std::size_t index = 1; index < history.size(); ++index) {
      if (history[index].time_seconds > displayed_state.time_seconds) {
        break;
      }
      const ImVec2 next =
          ToScreen(layout, history[index].x_m, history[index].y_m);
      draw_list->AddLine(previous, next, IM_COL32(80, 165, 225, 190), 2.0f);
      previous = next;
    }
  }

  const ImVec2 particle =
      ToScreen(layout, displayed_state.x_m, displayed_state.y_m);
  const ImU32 particle_color =
      displayed_state.status == LorentzParticleStatus::kActive
          ? IM_COL32(255, 100, 105, 255)
          : IM_COL32(255, 190, 80, 255);
  draw_list->AddCircleFilled(particle, 9.0f, particle_color, 32);
  draw_list->AddCircle(particle, 9.0f, IM_COL32(35, 40, 48, 255), 32, 2.0f);

  const ImVec2 velocity_direction{
      static_cast<float>(displayed_state.velocity_x_m_s),
      -static_cast<float>(displayed_state.velocity_y_m_s)};
  DrawArrow(draw_list, particle, velocity_direction, 72.0f,
            IM_COL32(100, 205, 245, 255));
  char particle_label[96];
  std::snprintf(particle_label, sizeof(particle_label),
                "m=%.3g kg, q=%+.3g C\nv=%.4g m/s", config.mass_kg,
                config.charge_c,
                std::hypot(displayed_state.velocity_x_m_s,
                           displayed_state.velocity_y_m_s));
  draw_list->AddText({particle.x + 14.0f, particle.y - 28.0f},
                     IM_COL32(255, 215, 215, 255), particle_label);

  if (config.electric_field_enabled) {
    const double angle =
        config.electric_field_angle_degrees * kDegreesToRadians;
    const ImVec2 direction{static_cast<float>(std::cos(angle)),
                           -static_cast<float>(std::sin(angle))};
    const ImVec2 arrow_center{layout.right - 205.0f, layout.top - 44.0f};
    DrawArrow(draw_list, arrow_center, direction, 72.0f,
              IM_COL32(100, 225, 150, 255));
    char label[96];
    std::snprintf(label, sizeof(label), "E=%.4g N/C, %.2f deg CCW",
                  config.electric_field_strength_n_c,
                  config.electric_field_angle_degrees);
    draw_list->AddText({arrow_center.x - 105.0f, arrow_center.y + 26.0f},
                       IM_COL32(120, 235, 165, 255), label);
  }

  if (config.magnetic_field_enabled) {
    const ImVec2 field_center{layout.right - 35.0f, layout.top - 45.0f};
    DrawMagneticFieldSymbol(draw_list, field_center, config.magnetic_field_z_t);
    char label[88];
    std::snprintf(label, sizeof(label), "Bz=%+.4g T",
                  config.magnetic_field_z_t);
    draw_list->AddText({field_center.x - 56.0f, field_center.y + 26.0f},
                       IM_COL32(205, 125, 255, 255), label);
  }
}

bool DrawMonitor(
    const tiny2d::sandbox::LorentzParticleConfig& config,
    const tiny2d::sandbox::LorentzParticleState& current_state,
    const std::vector<tiny2d::sandbox::LorentzParticleState>& history,
    bool* paused, double* inspect_time, bool* follow_live,
    const char* runtime_error) {
  using tiny2d::sandbox::LorentzParticleStatus;

  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({410.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("OrbitLab monitor", nullptr, kWindowFlags);

  const ImVec4 status_color =
      current_state.status == LorentzParticleStatus::kActive
          ? (*paused ? ImVec4{1.0f, 0.7f, 0.25f, 1.0f}
                     : ImVec4{0.35f, 0.9f, 0.5f, 1.0f})
          : ImVec4{1.0f, 0.45f, 0.25f, 1.0f};
  ImGui::TextColored(status_color, "%s",
                     current_state.status == LorentzParticleStatus::kActive
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
  const double maximum_inspect_time = std::max(
      current_state.time_seconds, tiny2d::sandbox::kLorentzPhysicsStep);
  ImGui::SetNextItemWidth(120.0f);
  constexpr double kMinimumInspectTime = 0.0;
  bool inspect_changed = ImGui::SliderScalar(
      "##lorentz_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputDouble("##lorentz_history_input", inspect_time,
                                        0.0, 0.0, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const tiny2d::sandbox::LorentzParticleState* inspected =
      tiny2d::sandbox::FindLorentzParticleState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  const tiny2d::sandbox::LorentzParticleDerived derived =
      tiny2d::sandbox::CalculateLorentzParticleDerived(config, *inspected);

  ImGui::Text("Sample time: %.6f s", inspected->time_seconds);
  ImGui::Text("Position: (%+.6f, %+.6f) m", inspected->x_m, inspected->y_m);
  ImGui::Text("Velocity: (%+.6f, %+.6f) m/s", inspected->velocity_x_m_s,
              inspected->velocity_y_m_s);
  ImGui::Text("Speed: %.6f m/s", derived.speed_m_s);
  ImGui::Text("Acceleration: (%+.6f, %+.6f) m/s^2", derived.acceleration_x_m_s2,
              derived.acceleration_y_m_s2);

  ImGui::Spacing();
  ImGui::TextUnformatted("Particle and fields");
  ImGui::Separator();
  ImGui::Text("Mass m: %.7g kg", config.mass_kg);
  ImGui::Text("Charge q: %+.7g C", config.charge_c);
  ImGui::Text("Electric field: %s",
              config.electric_field_enabled ? "enabled" : "disabled");
  ImGui::Text("E: %.7g N/C at %+.3f deg", config.electric_field_strength_n_c,
              config.electric_field_angle_degrees);
  ImGui::Text("Magnetic field: %s",
              config.magnetic_field_enabled ? "enabled" : "disabled");
  ImGui::Text("Bz: %+.7g T", config.magnetic_field_z_t);

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy");
  ImGui::Separator();
  ImGui::Text("Kinetic: %.7g J", derived.kinetic_energy_j);
  ImGui::Text("Electric potential: %+.7g J",
              derived.electric_potential_energy_j);
  ImGui::Text("Total: %+.7g J", derived.total_energy_j);

  ImGui::Spacing();
  ImGui::TextUnformatted("Cyclotron and E x B data");
  ImGui::Separator();
  if (derived.has_cyclotron_data) {
    ImGui::Text("Cyclotron omega: %+.7g rad/s",
                derived.cyclotron_angular_frequency_rad_s);
    ImGui::Text("Cyclotron period: %.7g s", derived.cyclotron_period_s);
    ImGui::Text("Larmor radius: %.7g m", derived.larmor_radius_m);
    ImGui::Text("E x B drift: (%+.7g, %+.7g) m/s", derived.drift_velocity_x_m_s,
                derived.drift_velocity_y_m_s);
  } else {
    ImGui::TextUnformatted("Cyclotron omega: N/A");
    ImGui::TextUnformatted("Cyclotron period: N/A");
    ImGui::TextUnformatted("Larmor radius: N/A");
    ImGui::TextUnformatted("E x B drift: N/A");
  }

  if (runtime_error != nullptr) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error);
  }

  const float button_width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  ImGui::BeginDisabled(current_state.status != LorentzParticleStatus::kActive ||
                       runtime_error != nullptr);
  if (ImGui::Button(*paused ? "Resume" : "Pause", {button_width, 36.0f})) {
    *paused = !*paused;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  const bool stop =
      ImGui::Button("Stop and choose model", {button_width, 36.0f});

  ImGui::End();
  return stop;
}

void RecordState(const tiny2d::sandbox::LorentzParticleState& state,
                 std::vector<tiny2d::sandbox::LorentzParticleState>* history) {
  if (!history->empty() && state.time_seconds <= history->back().time_seconds) {
    history->back() = state;
  } else {
    history->push_back(state);
  }
}

}  // namespace

namespace tiny2d::sandbox {

SimulationResult RunLorentzParticleSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  LorentzParticleConfig config;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  // ponytail: Keep one laboratory run in memory; cap it only if long-running
  // sessions become a demonstrated use case.
  std::vector<LorentzParticleState> history;
  bool simulation_started = false;
  bool paused = false;
  bool follow_live = true;
  double inspect_time = 0.0;
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
                 state.status == LorentzParticleStatus::kActive) {
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
        state = MakeInitialLorentzParticleState(config);
        history.clear();
        history.push_back(state);
        simulation_started = true;
        paused = state.status != LorentzParticleStatus::kActive;
        follow_live = true;
        inspect_time = 0.0;
        runtime_error =
            state.status == LorentzParticleStatus::kOutOfBounds
                ? "The particle starts outside the 20 m x 12 m domain."
                : nullptr;
        previous_time = current_time;
      }
    } else if (simulation_started) {
      if (!return_requested && !paused && runtime_error == nullptr &&
          state.status == LorentzParticleStatus::kActive) {
        const double frame_time = std::min(
            static_cast<double>(current_time - previous_time) / frequency,
            kMaximumFrameTime);
        previous_time = current_time;
        accumulated_time += frame_time;
        while (accumulated_time >= kLorentzPhysicsStep) {
          const bool stepped =
              StepLorentzParticle(config, kLorentzPhysicsStep, &state);
          if (!stepped) {
            if (state.status == LorentzParticleStatus::kOutOfBounds) {
              RecordState(state, &history);
              runtime_error =
                  "The particle left the 20 m x 12 m simulation domain.";
            } else {
              runtime_error = GetLorentzParticleStateError(config, state);
            }
            if (runtime_error == nullptr) {
              runtime_error =
                  "The fixed-step Lorentz model rejected this state.";
            }
            paused = true;
            accumulated_time = 0.0;
            break;
          }
          RecordState(state, &history);
          accumulated_time -= kLorentzPhysicsStep;
          if (state.status == LorentzParticleStatus::kOutOfBounds) {
            runtime_error =
                "The particle left the 20 m x 12 m simulation domain.";
            paused = true;
            accumulated_time = 0.0;
            break;
          }
        }
      } else {
        previous_time = current_time;
        accumulated_time = 0.0;
      }

      if (!return_requested &&
          DrawMonitor(config, state, history, &paused, &inspect_time,
                      &follow_live, runtime_error)) {
        return_requested = true;
      }
      const LorentzParticleState* displayed_state =
          FindLorentzParticleState(history, inspect_time);
      if (displayed_state == nullptr) {
        displayed_state = &state;
      }
      DrawScene(config, *displayed_state, history);
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
