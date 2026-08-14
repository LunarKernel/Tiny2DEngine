#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "app/lab_shell.h"
#include "chaos_lab_model.h"
#include "sim_ui.h"
#include "simulations.h"

namespace tiny2d::sandbox {
namespace {

constexpr float kControlStart = 310.0f;
constexpr float kSliderWidth = 310.0f;
constexpr float kInputWidth = 110.0f;
constexpr float kRadiansToDegrees = 57.2957795131f;

shell::SetupAction DrawSetupScreen(ChaosConfig* config) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ChaosLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V19 ChaosLab: double pendulum on rod constraints");
  ImGui::TextDisabled(
      "SI units. Two point-mass bobs on rigid massless rods; a shadow run "
      "with a small angle offset makes divergence visible.");
  ImGui::TextDisabled(
      "Angles are measured from straight down; a positive angle or angular "
      "velocity appears clockwise on screen.");
  ImGui::Spacing();

  ImGui::BeginChild("##chaos_parameters", {0.0f, -94.0f}, false);
  ImGui::TextUnformatted("Bobs and rods");
  ImGui::Separator();
  ui::SliderInputFloat("Mass 1 (kg)", &config->mass_1_kg, kChaosMinimumMassKg,
                       kChaosMaximumMassKg, "%.4g", kControlStart, kSliderWidth,
                       kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Mass 2 (kg)", &config->mass_2_kg, kChaosMinimumMassKg,
                       kChaosMaximumMassKg, "%.4g", kControlStart, kSliderWidth,
                       kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Rod length L1 (m)", &config->length_1_m,
                       kChaosMinimumRodLengthM, kChaosMaximumRodLengthM, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Rod length L2 (m)", &config->length_2_m,
                       kChaosMinimumRodLengthM, kChaosMaximumRodLengthM, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Bob radius (m)", &config->bob_radius_m,
                       kChaosMinimumBobRadiusM, kChaosMaximumBobRadiusM, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Initial state");
  ImGui::Separator();
  ui::SliderInputFloat("Angle 1 (deg)", &config->initial_angle_1_deg, -180.0f,
                       180.0f, "%.2f", kControlStart, kSliderWidth,
                       kInputWidth);
  ui::SliderInputFloat("Angle 2 (deg)", &config->initial_angle_2_deg, -180.0f,
                       180.0f, "%.2f", kControlStart, kSliderWidth,
                       kInputWidth);
  ui::SliderInputFloat(
      "Angular velocity 1 (rad/s)", &config->initial_angular_velocity_1_rad_s,
      -kChaosMaximumAngularSpeedRadS, kChaosMaximumAngularSpeedRadS, "%.3f",
      kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat(
      "Angular velocity 2 (rad/s)", &config->initial_angular_velocity_2_rad_s,
      -kChaosMaximumAngularSpeedRadS, kChaosMaximumAngularSpeedRadS, "%.3f",
      kControlStart, kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Environment and shadow run");
  ImGui::Separator();
  ui::SliderInputFloat("Gravity g (m/s^2)", &config->gravity_m_s2,
                       kChaosMinimumGravityMps2, kChaosMaximumGravityMps2,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Linear damping (1/s)", &config->linear_damping_per_s,
                       0.0f, kChaosMaximumDampingPerS, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Shadow angle offset (rad)", &config->shadow_offset_rad,
                       0.0f, kChaosMaximumShadowOffsetRad, "%.6f",
                       kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ImGui::EndChild();

  const char* error = GetChaosConfigError(*config);
  if (error == nullptr) {
    double slow_omega = 0.0;
    double fast_omega = 0.0;
    double slow_ratio = 0.0;
    double fast_ratio = 0.0;
    GetChaosSmallAngleModes(*config, &slow_omega, &fast_omega, &slow_ratio,
                            &fast_ratio);
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | small-angle modes: slow %.4f rad/s, fast "
                       "%.4f rad/s",
                       slow_omega, fast_omega);
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 5.0f * spacing) / 6.0f;
  shell::SetupAction action = shell::SetupAction::kNone;
  if (ImGui::Button("Back", {button_width, 38.0f})) {
    action = shell::SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Slow mode", {button_width, 38.0f})) {
    *config = MakeChaosSlowModeConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Large amplitude", {button_width, 38.0f})) {
    *config = MakeChaosLargeAmplitudeConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Chaotic", {button_width, 38.0f})) {
    *config = MakeChaosReferenceConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Damped", {button_width, 38.0f})) {
    *config = MakeChaosDampedConfig();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Start", {button_width, 38.0f})) {
    action = shell::SetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

void DrawChaosScene(const ChaosConfig& config, const ChaosState& state) {
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float scene_left = std::min(450.0f, display_size.x * 0.42f);
  const float scene_width = std::max(display_size.x - scene_left, 200.0f);
  const ImVec2 pivot_screen{scene_left + scene_width * 0.5f,
                            display_size.y * 0.5f};
  const float reach =
      config.length_1_m + config.length_2_m + config.bob_radius_m;
  const float pixels_per_meter =
      std::max(4.0f, std::min(scene_width, display_size.y) * 0.46f /
                         std::max(reach, 0.5f));

  const auto to_screen = [&](Vec2 world) {
    return ImVec2{
        pivot_screen.x + (world.x - kChaosPivotXM) * pixels_per_meter,
        pivot_screen.y + (world.y - kChaosPivotYM) * pixels_per_meter};
  };
  const float bob_radius_px =
      std::max(config.bob_radius_m * pixels_per_meter, 4.0f);

  // Shadow run first (ghosted) so the primary draws on top.
  const ImVec2 shadow_1 = to_screen(state.shadow_bob_1.position);
  const ImVec2 shadow_2 = to_screen(state.shadow_bob_2.position);
  draw_list->AddLine(pivot_screen, shadow_1, IM_COL32(140, 150, 170, 90), 2.0f);
  draw_list->AddLine(shadow_1, shadow_2, IM_COL32(140, 150, 170, 90), 2.0f);
  draw_list->AddCircleFilled(shadow_1, bob_radius_px,
                             IM_COL32(140, 170, 220, 90));
  draw_list->AddCircleFilled(shadow_2, bob_radius_px,
                             IM_COL32(235, 160, 120, 90));

  const ImVec2 bob_1 = to_screen(state.bob_1.position);
  const ImVec2 bob_2 = to_screen(state.bob_2.position);
  draw_list->AddLine(pivot_screen, bob_1, IM_COL32(200, 210, 225, 255), 2.5f);
  draw_list->AddLine(bob_1, bob_2, IM_COL32(200, 210, 225, 255), 2.5f);
  draw_list->AddCircleFilled(bob_1, bob_radius_px, IM_COL32(95, 165, 245, 255));
  draw_list->AddCircle(bob_1, bob_radius_px + 1.0f,
                       IM_COL32(190, 225, 255, 255), 24, 1.5f);
  draw_list->AddCircleFilled(bob_2, bob_radius_px, IM_COL32(245, 140, 95, 255));
  draw_list->AddCircle(bob_2, bob_radius_px + 1.0f,
                       IM_COL32(255, 210, 180, 255), 24, 1.5f);
  draw_list->AddCircleFilled(pivot_screen, 5.0f, IM_COL32(225, 230, 240, 255));
  draw_list->AddCircle(pivot_screen, 6.0f, IM_COL32(40, 45, 55, 255), 20, 2.0f);

  char labels[112];
  std::snprintf(labels, sizeof(labels),
                "m1=%.3g kg L1=%.3g m | m2=%.3g kg L2=%.3g m", config.mass_1_kg,
                config.length_1_m, config.mass_2_kg, config.length_2_m);
  draw_list->AddText({scene_left + 24.0f, display_size.y - 48.0f},
                     IM_COL32(245, 205, 115, 255), labels);
}

bool DrawMonitor(const ChaosConfig& config, const ChaosState& current_state,
                 const std::vector<ChaosState>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& runtime_error, ChaosState* displayed_state,
                 std::string* export_status) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({430.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ChaosLab monitor", nullptr, kWindowFlags);

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
  constexpr double kMinimumInspectTime = 0.0;
  const double maximum_inspect_time = std::max(
      current_state.time_seconds, static_cast<double>(kChaosPhysicsStep));
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_changed = ImGui::SliderScalar(
      "##chaos_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputDouble("##chaos_history_input", inspect_time,
                                        0.0, 0.0, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const ChaosState* inspected = FindChaosState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  *displayed_state = *inspected;
  const ChaosDerived derived = CalculateChaosDerived(config, *inspected);

  ImGui::TextUnformatted("Phase state (theta, omega) per bob");
  ImGui::Separator();
  ImGui::Text("bob 1: theta %+8.3f deg | omega %+8.4f rad/s",
              derived.theta_1_rad * kRadiansToDegrees, derived.omega_1_rad_s);
  ImGui::Text("bob 2: theta %+8.3f deg | omega %+8.4f rad/s",
              derived.theta_2_rad * kRadiansToDegrees, derived.omega_2_rad_s);
  ImGui::Text("modes: slow %.4f rad/s (r=%+.3f) | fast %.4f rad/s (r=%+.3f)",
              derived.slow_mode_omega_rad_s, derived.slow_mode_shape_ratio,
              derived.fast_mode_omega_rad_s, derived.fast_mode_shape_ratio);

  ImGui::Spacing();
  ImGui::TextUnformatted("Rod constraints");
  ImGui::Separator();
  ImGui::Text("anchor rod force: %+.5f N (tension positive)",
              inspected->anchor_rod_force_n);
  ImGui::Text("link rod force:   %+.5f N", inspected->link_rod_force_n);
  ImGui::Text("length errors: rod1 %+.2e m | rod2 %+.2e m",
              derived.rod_1_length_error_m, derived.rod_2_length_error_m);

  ImGui::Spacing();
  ImGui::TextUnformatted("Shadow separation");
  ImGui::Separator();
  ImGui::Text("shadow: theta1 %+8.3f deg | theta2 %+8.3f deg",
              derived.shadow_theta_1_rad * kRadiansToDegrees,
              derived.shadow_theta_2_rad * kRadiansToDegrees);
  ImGui::Text("delta: %.3e rad | growth: %+.2f decades", derived.separation_rad,
              derived.separation_decades);

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy");
  ImGui::Separator();
  ImGui::Text("kinetic: %.8f J", derived.kinetic_energy_j);
  ImGui::Text("potential (vs start): %+.8f J", derived.potential_energy_j);
  ImGui::Text("mechanical: %+.8f J", derived.mechanical_energy_j);
  ImGui::Text("dissipated: %.8f J", inspected->dissipated_energy_j);
  ImGui::Text("mechanical + dissipated: %+.8f J", derived.accounted_energy_j);

  if (!runtime_error.empty()) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error.c_str());
  }

  const float width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  ImGui::BeginDisabled(!runtime_error.empty());
  if (ImGui::Button(*paused ? "Resume" : "Pause", {width, 36.0f})) {
    *paused = !*paused;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  const bool stop = ImGui::Button("Stop and choose model", {width, 36.0f});

  ImGui::BeginDisabled(history.empty());
  if (ImGui::Button("Export CSV", {ImGui::GetContentRegionAvail().x, 30.0f})) {
    try {
      char status[48];
      std::snprintf(
          status, sizeof(status), "%s t=%.4f s",
          runtime_error.empty() ? (*paused ? "PAUSED" : "RUNNING") : "ERROR",
          current_state.time_seconds);
      std::string full_status = status;
      if (!runtime_error.empty()) {
        full_status += " | " + runtime_error;
      }
      *export_status = ui::ExportCsvToWorkingDirectory(
          "chaos_lab",
          BuildChaosCsv(config, history, TINY2D_PRODUCT_VERSION, full_status));
    } catch (const std::exception& exception) {
      *export_status = std::string("Export failed: ") + exception.what();
    }
  }
  ImGui::EndDisabled();
  if (!export_status->empty()) {
    ImGui::TextWrapped("%s", export_status->c_str());
  }

  ImGui::End();
  return stop;
}

struct ChaosLabTraits {
  using Config = ChaosConfig;
  using State = ChaosState;
  static constexpr float kPhysicsStep = kChaosPhysicsStep;
  static constexpr double kMaximumFrameTime = 0.25;
  static constexpr const char* kInvalidHistoryTimeMessage =
      "ChaosLab produced an invalid history time.";
  static constexpr const char* kNonIncreasingHistoryTimeMessage =
      "ChaosLab history time stopped increasing.";

  Config MakeInitialConfig() { return MakeChaosReferenceConfig(); }
  State MakeState(const Config& config) {
    return MakeInitialChaosState(config);
  }
  const char* InitialStateIssue(const Config&, const State&) { return nullptr; }
  bool CanStep(const State&) { return true; }
  bool Step(const Config& config, State* state) {
    return StepChaos(config, kPhysicsStep, state);
  }
  std::string StepFailureMessage(const Config& config, const State& state,
                                 std::vector<State>*) {
    const char* error = GetChaosStateError(config, state);
    return error != nullptr ? error
                            : "ChaosLab rejected an unstable fixed step.";
  }
  const char* AfterStepIssue(const State&) { return nullptr; }
  shell::SetupAction DrawSetup(Config* config, const std::string&) {
    return DrawSetupScreen(config);
  }
  bool DrawFrame(const Config& config, const State& state,
                 const std::vector<State>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& error) {
    State displayed_state = state;
    const bool stop =
        DrawMonitor(config, state, history, paused, inspect_time, follow_live,
                    error, &displayed_state, &export_status_);
    DrawChaosScene(config, displayed_state);
    return stop;
  }

  // Result line of the last Export CSV click; persists across frames.
  std::string export_status_;
};

}  // namespace

SimulationResult RunChaosLabSimulation(SDL_Renderer* renderer) {
  return shell::RunLab(renderer, ChaosLabTraits{});
}

}  // namespace tiny2d::sandbox
