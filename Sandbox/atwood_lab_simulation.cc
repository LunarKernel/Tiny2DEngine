#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "app/lab_shell.h"
#include "atwood_lab_model.h"
#include "sim_ui.h"
#include "simulations.h"

namespace tiny2d::sandbox {
namespace {

constexpr float kControlStart = 310.0f;
constexpr float kSliderWidth = 310.0f;
constexpr float kInputWidth = 110.0f;

shell::SetupAction DrawSetupScreen(AtwoodConfig* config) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("AtwoodLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V18 AtwoodLab: massive pulley and rope constraints");
  ImGui::TextDisabled(
      "SI units. Two blocks hang from an inextensible, non-slipping rope "
      "over a pinned pulley with real rotational inertia.");
  ImGui::TextDisabled(
      "+X points right, +Y points down, and positive rotation is clockwise; "
      "a positive rope speed lowers block b.");
  ImGui::Spacing();

  ImGui::BeginChild("##atwood_parameters", {0.0f, -94.0f}, false);
  ImGui::TextUnformatted("Hanging blocks");
  ImGui::Separator();
  ui::SliderInputFloat("Mass a (kg)", &config->mass_a_kg, kAtwoodMinimumMassKg,
                       kAtwoodMaximumMassKg, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Mass b (kg)", &config->mass_b_kg, kAtwoodMinimumMassKg,
                       kAtwoodMaximumMassKg, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Block edge (m)", &config->block_edge_m,
                       kAtwoodMinimumBlockEdgeM, kAtwoodMaximumBlockEdgeM,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Hang depth a (m)", &config->hang_depth_a_m,
                       kAtwoodMinimumHangDepthM, kAtwoodMaximumHangDepthM,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Hang depth b (m)", &config->hang_depth_b_m,
                       kAtwoodMinimumHangDepthM, kAtwoodMaximumHangDepthM,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Pulley");
  ImGui::Separator();
  ui::SliderInputFloat("Pulley mass (kg)", &config->pulley_mass_kg,
                       kAtwoodMinimumMassKg, kAtwoodMaximumMassKg, "%.4g",
                       kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Pulley radius R (m)", &config->pulley_radius_m,
                       kAtwoodMinimumPulleyRadiusM, kAtwoodMaximumPulleyRadiusM,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  int inertia_index =
      config->pulley_inertia == CircleInertiaModel::kHoop ? 1 : 0;
  ImGui::SetNextItemWidth(kSliderWidth);
  if (ImGui::Combo("Pulley inertia model", &inertia_index,
                   "Solid disk, I = mR^2/2\0Hoop, I = mR^2\0")) {
    config->pulley_inertia = inertia_index == 1
                                 ? CircleInertiaModel::kHoop
                                 : CircleInertiaModel::kSolidDisk;
  }

  ImGui::Spacing();
  ImGui::TextUnformatted("Initial state and environment");
  ImGui::Separator();
  ui::SliderInputFloat("Initial rope speed (m/s)",
                       &config->initial_rope_speed_mps,
                       -kAtwoodMaximumRopeSpeedMps, kAtwoodMaximumRopeSpeedMps,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Gravity g (m/s^2)", &config->gravity_m_s2,
                       kAtwoodMinimumGravityMps2, kAtwoodMaximumGravityMps2,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Linear damping (1/s)", &config->linear_damping_per_s,
                       0.0f, kAtwoodMaximumDampingPerS, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth);
  ImGui::EndChild();

  const char* error = GetAtwoodConfigError(*config);
  if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | analytical acceleration a = %.6f m/s^2",
                       GetAtwoodAnalyticalAcceleration(*config));
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 4.0f * spacing) / 5.0f;
  shell::SetupAction action = shell::SetupAction::kNone;
  if (ImGui::Button("Back", {button_width, 38.0f})) {
    action = shell::SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Reference preset", {button_width, 38.0f})) {
    *config = MakeAtwoodReferenceConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Balanced drift preset", {button_width, 38.0f})) {
    *config = MakeAtwoodBalancedDriftConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Damped preset", {button_width, 38.0f})) {
    *config = MakeAtwoodDampedConfig();
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

void DrawAtwoodScene(const AtwoodConfig& config, const AtwoodState& state) {
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float scene_left = std::min(450.0f, display_size.x * 0.42f);
  const float scene_width = std::max(display_size.x - scene_left, 200.0f);
  const ImVec2 pulley_screen{scene_left + scene_width * 0.5f,
                             display_size.y * 0.16f};
  const float deepest =
      std::max(state.block_a.position.y, state.block_b.position.y) -
      kAtwoodPulleyCenterYM;
  const float pixels_per_meter =
      std::max(4.0f, (display_size.y * 0.98f - pulley_screen.y) /
                         std::max(deepest + config.block_edge_m + 0.5f, 1.0f));

  const auto to_screen = [&](Vec2 world) {
    return ImVec2{
        pulley_screen.x + (world.x - kAtwoodPulleyCenterXM) * pixels_per_meter,
        pulley_screen.y + (world.y - kAtwoodPulleyCenterYM) * pixels_per_meter};
  };

  const float radius_px =
      std::max(config.pulley_radius_m * pixels_per_meter, 6.0f);
  const ImVec2 anchor_a_screen{pulley_screen.x - radius_px, pulley_screen.y};
  const ImVec2 anchor_b_screen{pulley_screen.x + radius_px, pulley_screen.y};

  // Rope: both straight segments plus the wrap arc over the pulley.
  const ImVec2 block_a_screen = to_screen(state.block_a.position);
  const ImVec2 block_b_screen = to_screen(state.block_b.position);
  const ImU32 rope_color = IM_COL32(235, 195, 90, 255);
  draw_list->AddLine(anchor_a_screen, {anchor_a_screen.x, block_a_screen.y},
                     rope_color, 2.0f);
  draw_list->AddLine(anchor_b_screen, {anchor_b_screen.x, block_b_screen.y},
                     rope_color, 2.0f);
  draw_list->PathArcTo(pulley_screen, radius_px, 3.14159265f, 6.28318531f, 24);
  draw_list->PathStroke(rope_color, 0, 2.0f);

  // Pulley disk with a rotation spoke.
  draw_list->AddCircleFilled(pulley_screen, radius_px,
                             IM_COL32(90, 100, 115, 255));
  draw_list->AddCircle(pulley_screen, radius_px, IM_COL32(200, 210, 225, 255),
                       32, 2.0f);
  const float spoke_angle = state.pulley.angle;
  draw_list->AddLine(pulley_screen,
                     {pulley_screen.x + std::cos(spoke_angle) * radius_px,
                      pulley_screen.y + std::sin(spoke_angle) * radius_px},
                     IM_COL32(245, 245, 245, 255), 2.0f);
  draw_list->AddCircleFilled(pulley_screen, 3.0f, IM_COL32(245, 245, 245, 255));

  // Blocks are axis-aligned squares hanging at the segment ends.
  const auto draw_block = [&](const Rectangle& block, ImU32 fill,
                              const char* label, float mass) {
    const float half_edge_px =
        std::max(block.width * 0.5f * pixels_per_meter, 3.0f);
    const ImVec2 center = to_screen(block.position);
    draw_list->AddRectFilled({center.x - half_edge_px, center.y - half_edge_px},
                             {center.x + half_edge_px, center.y + half_edge_px},
                             fill);
    draw_list->AddRect({center.x - half_edge_px, center.y - half_edge_px},
                       {center.x + half_edge_px, center.y + half_edge_px},
                       IM_COL32(230, 240, 250, 255), 0.0f, 0, 1.5f);
    char text[64];
    std::snprintf(text, sizeof(text), "%s m=%.3g kg", label, mass);
    draw_list->AddText({center.x + half_edge_px + 8.0f, center.y - 8.0f},
                       IM_COL32(200, 225, 250, 255), text);
  };
  draw_block(state.block_a, IM_COL32(95, 165, 245, 255), "a", config.mass_a_kg);
  draw_block(state.block_b, IM_COL32(245, 140, 95, 255), "b", config.mass_b_kg);

  char footer[112];
  std::snprintf(footer, sizeof(footer),
                "R=%.3g m | I=%.4g kg*m^2 | T_a=%.4g N | T_b=%.4g N",
                config.pulley_radius_m, GetMomentOfInertia(state.pulley),
                state.tension_a_n, state.tension_b_n);
  draw_list->AddText({scene_left + 24.0f, display_size.y - 48.0f},
                     IM_COL32(245, 205, 115, 255), footer);
}

bool DrawMonitor(const AtwoodConfig& config, const AtwoodState& current_state,
                 const std::vector<AtwoodState>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& runtime_error,
                 AtwoodState* displayed_state) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({430.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("AtwoodLab monitor", nullptr, kWindowFlags);

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
      current_state.time_seconds, static_cast<double>(kAtwoodPhysicsStep));
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_changed = ImGui::SliderScalar(
      "##atwood_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputDouble("##atwood_history_input", inspect_time,
                                        0.0, 0.0, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const AtwoodState* inspected = FindAtwoodState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  *displayed_state = *inspected;
  const AtwoodDerived derived = CalculateAtwoodDerived(config, *inspected);

  // Measured acceleration from the previous history sample's rope speed.
  double measured_acceleration = 0.0;
  if (inspected->time_seconds > 0.0) {
    const AtwoodState* previous = FindAtwoodState(
        history, inspected->time_seconds - 4.0 * kAtwoodPhysicsStep);
    if (previous != nullptr &&
        previous->time_seconds < inspected->time_seconds) {
      measured_acceleration =
          (static_cast<double>(inspected->block_b.velocity.y) -
           previous->block_b.velocity.y) /
          (inspected->time_seconds - previous->time_seconds);
    }
  }

  ImGui::TextUnformatted("Motion");
  ImGui::Separator();
  ImGui::Text("rope speed: %+.5f m/s (b descends when positive)",
              derived.rope_speed_b_mps);
  ImGui::Text("pulley omega: %+.5f rad/s | rim: %+.5f m/s",
              inspected->pulley.angular_velocity, derived.pulley_rim_speed_mps);
  ImGui::Text("analytical a: %+.6f m/s^2",
              derived.analytical_acceleration_m_s2);
  ImGui::Text("measured a:   %+.6f m/s^2", measured_acceleration);
  ImGui::Text("no-slip error: a %.2e | b %.2e m/s", derived.no_slip_error_a_mps,
              derived.no_slip_error_b_mps);

  ImGui::Spacing();
  ImGui::TextUnformatted("Constraint reactions");
  ImGui::Separator();
  ImGui::Text("T_a measured: %.5f N | analytical: %.5f N",
              inspected->tension_a_n, derived.analytical_tension_a_n);
  ImGui::Text("T_b measured: %.5f N | analytical: %.5f N",
              inspected->tension_b_n, derived.analytical_tension_b_n);
  ImGui::Text("pin force: (%+.4f, %+.4f) N", inspected->pin_force_n.x,
              inspected->pin_force_n.y);
  ImGui::Text("axle load m_p g + T_a + T_b: %.5f N", derived.axle_load_n);
  ImGui::Text("rope length error: %+.2e m", derived.rope_length_error_m);
  ImGui::Text("segments: a %.5f m | b %.5f m", derived.segment_length_a_m,
              derived.segment_length_b_m);

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy");
  ImGui::Separator();
  ImGui::Text("translational: %.8f J", derived.translational_kinetic_energy_j);
  ImGui::Text("rotational: %.8f J", derived.rotational_kinetic_energy_j);
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
  ImGui::End();
  return stop;
}

struct AtwoodLabTraits {
  using Config = AtwoodConfig;
  using State = AtwoodState;
  static constexpr float kPhysicsStep = kAtwoodPhysicsStep;
  static constexpr double kMaximumFrameTime = 0.25;
  static constexpr const char* kInvalidHistoryTimeMessage =
      "AtwoodLab produced an invalid history time.";
  static constexpr const char* kNonIncreasingHistoryTimeMessage =
      "AtwoodLab history time stopped increasing.";

  Config MakeInitialConfig() { return MakeAtwoodReferenceConfig(); }
  State MakeState(const Config& config) {
    config_for_terminal = config;
    return MakeInitialAtwoodState(config);
  }
  const char* InitialStateIssue(const Config& config, const State& state) {
    return GetAtwoodTerminalIssue(config, state);
  }
  bool CanStep(const State&) { return true; }
  bool Step(const Config& config, State* state) {
    return StepAtwood(config, kPhysicsStep, state);
  }
  std::string StepFailureMessage(const Config& config, const State& state,
                                 std::vector<State>*) {
    const char* error = GetAtwoodStateError(config, state);
    return error != nullptr ? error
                            : "AtwoodLab rejected an unstable fixed step.";
  }
  const char* AfterStepIssue(const State& state) {
    return GetAtwoodTerminalIssue(config_for_terminal, state);
  }
  shell::SetupAction DrawSetup(Config* config, const std::string&) {
    return DrawSetupScreen(config);
  }
  bool DrawFrame(const Config& config, const State& state,
                 const std::vector<State>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& error) {
    State displayed_state = state;
    const bool stop = DrawMonitor(config, state, history, paused, inspect_time,
                                  follow_live, error, &displayed_state);
    DrawAtwoodScene(config, displayed_state);
    return stop;
  }

  // AfterStepIssue receives only the state, so MakeState records the
  // started configuration for the terminal-condition check.
  Config config_for_terminal = MakeAtwoodReferenceConfig();
};

}  // namespace

SimulationResult RunAtwoodLabSimulation(SDL_Renderer* renderer) {
  return shell::RunLab(renderer, AtwoodLabTraits{});
}

}  // namespace tiny2d::sandbox
