#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "app/lab_shell.h"
#include "force_lab_model.h"
#include "sim_ui.h"
#include "simulations.h"

namespace tiny2d::sandbox {
namespace {

constexpr double kMaximumFrameTime = 0.25;
constexpr float kControlStart = 310.0f;
constexpr float kSliderWidth = 310.0f;
constexpr float kInputWidth = 110.0f;
constexpr float kRadiansToDegrees = 57.2957795131f;

shell::SetupAction DrawSetupScreen(ForceLabConfig* config) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ForceLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V15 ForceLab: eccentric spring rigid body");
  ImGui::TextDisabled(
      "SI units. Drag a slider or type an exact value; finite values are "
      "clamped to the displayed range.");
  ImGui::TextDisabled(
      "+X points right, +Y points down, and positive rotation is clockwise.");
  ImGui::Spacing();

  ImGui::BeginChild("##force_lab_parameters", {0.0f, -94.0f}, false);
  ImGui::TextUnformatted("Rigid body");
  ImGui::Separator();
  ui::SliderInputFloat("Mass m (kg)", &config->mass_kg, kForceLabMinimumMassKg,
                       kForceLabMaximumMassKg, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Width (m)", &config->width_m,
                       kForceLabMinimumBodyDimensionM,
                       kForceLabMaximumBodyDimensionM, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Height (m)", &config->height_m,
                       kForceLabMinimumBodyDimensionM,
                       kForceLabMaximumBodyDimensionM, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Spring and attachment");
  ImGui::Separator();
  ui::SliderInputFloat("Stiffness k (N/m)", &config->spring_stiffness_n_m,
                       kForceLabMinimumSpringStiffnessNM,
                       kForceLabMaximumSpringStiffnessNM, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Rest length L0 (m)", &config->spring_rest_length_m,
                       kForceLabMinimumSpringLengthM,
                       kForceLabMaximumSpringLengthM, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth, ImGuiSliderFlags_Logarithmic);
  const float safe_width =
      std::isfinite(config->width_m)
          ? std::clamp(config->width_m, kForceLabMinimumBodyDimensionM,
                       kForceLabMaximumBodyDimensionM)
          : kForceLabMaximumBodyDimensionM;
  const float safe_height =
      std::isfinite(config->height_m)
          ? std::clamp(config->height_m, kForceLabMinimumBodyDimensionM,
                       kForceLabMaximumBodyDimensionM)
          : kForceLabMaximumBodyDimensionM;
  ui::SliderInputFloat("Attachment local x (m)", &config->attachment_local_m.x,
                       -safe_width * 0.5f, safe_width * 0.5f, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Attachment local y (m)", &config->attachment_local_m.y,
                       -safe_height * 0.5f, safe_height * 0.5f, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Initial state relative to the fixed anchor");
  ImGui::Separator();
  ImGui::Text("Anchor: (%.3f, %.3f) m", config->anchor_position_m.x,
              config->anchor_position_m.y);
  float center_x_from_anchor =
      config->initial_center_position_m.x - config->anchor_position_m.x;
  if (ui::SliderInputFloat("Initial COM x from anchor (m)",
                           &center_x_from_anchor, -20.0f, 20.0f, "%.3f",
                           kControlStart, kSliderWidth, kInputWidth)) {
    config->initial_center_position_m.x =
        config->anchor_position_m.x + center_x_from_anchor;
  }
  float center_y_from_anchor =
      config->initial_center_position_m.y - config->anchor_position_m.y;
  if (ui::SliderInputFloat("Initial COM y from anchor (m)",
                           &center_y_from_anchor, -20.0f, 20.0f, "%.3f",
                           kControlStart, kSliderWidth, kInputWidth)) {
    config->initial_center_position_m.y =
        config->anchor_position_m.y + center_y_from_anchor;
  }
  ui::SliderInputFloat("Initial vx (m/s)", &config->initial_velocity_m_s.x,
                       -kForceLabMaximumInitialSpeedMps,
                       kForceLabMaximumInitialSpeedMps, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Initial vy (m/s)", &config->initial_velocity_m_s.y,
                       -kForceLabMaximumInitialSpeedMps,
                       kForceLabMaximumInitialSpeedMps, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Initial angle, clockwise (deg)",
                       &config->initial_angle_degrees, -180.0f, 180.0f, "%.2f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Initial angular speed (rad/s)",
                       &config->initial_angular_velocity_rad_s,
                       -kForceLabMaximumInitialAngularSpeedRadS,
                       kForceLabMaximumInitialAngularSpeedRadS, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Optional viscous damping");
  ImGui::Separator();
  ui::SliderInputFloat("Linear c (N*s/m)", &config->linear_damping_n_s_m, 0.0f,
                       kForceLabMaximumDampingCoefficient, "%.4g",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Angular c (N*m*s/rad)",
                       &config->angular_damping_n_m_s_rad, 0.0f,
                       kForceLabMaximumDampingCoefficient, "%.4g",
                       kControlStart, kSliderWidth, kInputWidth);
  ImGui::EndChild();

  const char* error = GetForceLabConfigError(*config);
  if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | centered reference period T0 = %.6f s",
                       GetForceLabCenteredPeriod(*config));
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 3.0f * spacing) / 4.0f;
  shell::SetupAction action = shell::SetupAction::kNone;
  if (ImGui::Button("Back", {button_width, 38.0f})) {
    action = shell::SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Centered analytical preset", {button_width, 38.0f})) {
    *config = MakeCenteredReferenceConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Eccentric coupled preset", {button_width, 38.0f})) {
    *config = MakeEccentricDemoConfig();
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

ImVec2 WorldToScreen(const ForceLabConfig& config, ImVec2 anchor_screen,
                     float pixels_per_meter, Vec2 position) {
  return {anchor_screen.x +
              (position.x - config.anchor_position_m.x) * pixels_per_meter,
          anchor_screen.y +
              (position.y - config.anchor_position_m.y) * pixels_per_meter};
}

void DrawSpring(ImDrawList* draw_list, ImVec2 start, ImVec2 end) {
  constexpr std::size_t kPointCount = 18;
  std::array<ImVec2, kPointCount> points{};
  const float dx = end.x - start.x;
  const float dy = end.y - start.y;
  const float length = std::hypot(dx, dy);
  if (length <= 1.0f) {
    draw_list->AddLine(start, end, IM_COL32(235, 195, 90, 255), 3.0f);
    return;
  }
  const ImVec2 normal{-dy / length, dx / length};
  constexpr float kAmplitude = 7.0f;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const float ratio =
        static_cast<float>(i) / static_cast<float>(points.size() - 1);
    float offset = 0.0f;
    if (i > 1 && i + 2 < points.size()) {
      offset = i % 2 == 0 ? kAmplitude : -kAmplitude;
    }
    points[i] = {start.x + dx * ratio + normal.x * offset,
                 start.y + dy * ratio + normal.y * offset};
  }
  draw_list->AddPolyline(points.data(), static_cast<int>(points.size()),
                         IM_COL32(235, 195, 90, 255), 0, 3.0f);
}

void DrawForceLabScene(const ForceLabConfig& config,
                       const ForceLabState& state) {
  const ForceLabDerived derived = CalculateForceLabDerived(config, state);
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float scene_left = std::min(450.0f, display_size.x * 0.42f);
  const float scene_width = std::max(display_size.x - scene_left, 200.0f);
  const float pixels_per_meter =
      std::max(12.0f, std::min(scene_width / 15.0f, display_size.y / 11.0f));
  const ImVec2 anchor_screen{scene_left + scene_width * 0.32f,
                             display_size.y * 0.48f};
  const ImVec2 attachment_screen = WorldToScreen(
      config, anchor_screen, pixels_per_meter, derived.attachment_position_m);

  DrawSpring(draw_list, anchor_screen, attachment_screen);
  draw_list->AddCircleFilled(anchor_screen, 10.0f,
                             IM_COL32(225, 230, 240, 255));
  draw_list->AddCircle(anchor_screen, 11.0f, IM_COL32(40, 45, 55, 255), 24,
                       2.0f);

  const std::array<Vec2, 4> vertices = GetVertices(state.body);
  std::array<ImVec2, 4> screen_vertices{};
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    screen_vertices[i] =
        WorldToScreen(config, anchor_screen, pixels_per_meter, vertices[i]);
  }
  draw_list->AddConvexPolyFilled(screen_vertices.data(),
                                 static_cast<int>(screen_vertices.size()),
                                 IM_COL32(95, 165, 245, 255));
  draw_list->AddPolyline(
      screen_vertices.data(), static_cast<int>(screen_vertices.size()),
      IM_COL32(190, 225, 255, 255), ImDrawFlags_Closed, 2.0f);

  const ImVec2 center_screen = WorldToScreen(
      config, anchor_screen, pixels_per_meter, state.body.position);
  draw_list->AddCircleFilled(center_screen, 4.0f, IM_COL32(245, 245, 245, 255));
  draw_list->AddLine(center_screen, attachment_screen,
                     IM_COL32(175, 210, 245, 190), 1.5f);
  draw_list->AddCircleFilled(attachment_screen, 6.0f,
                             IM_COL32(255, 120, 105, 255));
  ui::DrawCenteredArrow(draw_list, attachment_screen,
                        {derived.spring_force_n.x, derived.spring_force_n.y},
                        72.0f, IM_COL32(255, 115, 110, 255));

  char body_label[96];
  std::snprintf(body_label, sizeof(body_label), "m=%.3g kg | theta=%+.2f deg",
                config.mass_kg, state.body.angle * kRadiansToDegrees);
  draw_list->AddText({center_screen.x + 18.0f, center_screen.y - 28.0f},
                     IM_COL32(190, 225, 255, 255), body_label);

  char spring_label[96];
  std::snprintf(spring_label, sizeof(spring_label),
                "k=%.3g N/m | L=%.3f m | x=%+.3f m",
                config.spring_stiffness_n_m, derived.spring_length_m,
                derived.spring_extension_m);
  draw_list->AddText({scene_left + 24.0f, display_size.y - 48.0f},
                     IM_COL32(245, 205, 115, 255), spring_label);
}

bool DrawMonitor(const ForceLabConfig& config,
                 const ForceLabState& current_state,
                 const std::vector<ForceLabState>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& runtime_error,
                 ForceLabState* displayed_state) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({430.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ForceLab monitor", nullptr, kWindowFlags);

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
      current_state.time_seconds, static_cast<double>(kForceLabPhysicsStep));
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_changed = ImGui::SliderScalar(
      "##force_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputDouble("##force_history_input", inspect_time,
                                        0.0, 0.0, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const ForceLabState* inspected = FindForceLabState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  *displayed_state = *inspected;
  const ForceLabDerived derived = CalculateForceLabDerived(config, *inspected);
  const Vec2 center_from_anchor{
      inspected->body.position.x - config.anchor_position_m.x,
      inspected->body.position.y - config.anchor_position_m.y};
  const Vec2 attachment_from_anchor{
      derived.attachment_position_m.x - config.anchor_position_m.x,
      derived.attachment_position_m.y - config.anchor_position_m.y};

  ImGui::Text("COM from anchor: (%+.5f, %+.5f) m", center_from_anchor.x,
              center_from_anchor.y);
  ImGui::Text("velocity: (%+.5f, %+.5f) m/s", inspected->body.velocity.x,
              inspected->body.velocity.y);
  ImGui::Text("acceleration: (%+.5f, %+.5f) m/s^2",
              derived.linear_acceleration_m_s2.x,
              derived.linear_acceleration_m_s2.y);
  ImGui::Text("theta: %+.4f deg", inspected->body.angle * kRadiansToDegrees);
  ImGui::Text("omega: %+.6f rad/s", inspected->body.angular_velocity);
  ImGui::Text("alpha: %+.6f rad/s^2", derived.angular_acceleration_rad_s2);

  ImGui::Spacing();
  ImGui::TextUnformatted("Spring and coupling");
  ImGui::Separator();
  ImGui::Text("attachment from anchor: (%+.5f, %+.5f) m",
              attachment_from_anchor.x, attachment_from_anchor.y);
  ImGui::Text("length L: %.6f m | extension x: %+.6f m",
              derived.spring_length_m, derived.spring_extension_m);
  ImGui::Text("spring force: (%+.6f, %+.6f) N", derived.spring_force_n.x,
              derived.spring_force_n.y);
  ImGui::Text("spring torque: %+.6f N*m", derived.spring_torque_n_m);
  ImGui::Text("damping force: (%+.6f, %+.6f) N", derived.damping_force_n.x,
              derived.damping_force_n.y);
  ImGui::Text("damping torque: %+.6f N*m", derived.damping_torque_n_m);
  ImGui::Text("I: %.6f kg*m^2", derived.moment_of_inertia_kg_m2);
  ImGui::Text("Centered reference T0: %.6f s",
              GetForceLabCenteredPeriod(config));

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy");
  ImGui::Separator();
  ImGui::Text("translational: %.8f J", derived.translational_kinetic_energy_j);
  ImGui::Text("rotational: %.8f J", derived.rotational_kinetic_energy_j);
  ImGui::Text("spring potential: %.8f J", derived.spring_potential_energy_j);
  ImGui::Text("mechanical: %.8f J", derived.mechanical_energy_j);
  ImGui::Text("dissipated: %.8f J", inspected->dissipated_energy_j);
  ImGui::Text("mechanical + dissipated: %.8f J", derived.accounted_energy_j);

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

struct ForceLabTraits {
  using Config = ForceLabConfig;
  using State = ForceLabState;
  static constexpr float kPhysicsStep = kForceLabPhysicsStep;
  static constexpr double kMaximumFrameTime = 0.25;
  static constexpr const char* kInvalidHistoryTimeMessage =
      "ForceLab produced an invalid history time.";
  static constexpr const char* kNonIncreasingHistoryTimeMessage =
      "ForceLab history time stopped increasing.";

  Config MakeInitialConfig() { return MakeEccentricDemoConfig(); }
  State MakeState(const Config& config) {
    return MakeInitialForceLabState(config);
  }
  const char* InitialStateIssue(const Config&, const State&) { return nullptr; }
  bool CanStep(const State&) { return true; }
  bool Step(const Config& config, State* state) {
    return StepForceLab(config, kPhysicsStep, state);
  }
  std::string StepFailureMessage(const Config& config, const State& state,
                                 std::vector<State>*) {
    const char* error = GetForceLabStateError(config, state);
    return error != nullptr ? error
                            : "ForceLab rejected an unstable fixed step.";
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
    const bool stop = DrawMonitor(config, state, history, paused, inspect_time,
                                  follow_live, error, &displayed_state);
    DrawForceLabScene(config, displayed_state);
    return stop;
  }
};

}  // namespace

SimulationResult RunForceLabSimulation(SDL_Renderer* renderer) {
  return shell::RunLab(renderer, ForceLabTraits{});
}

}  // namespace tiny2d::sandbox
