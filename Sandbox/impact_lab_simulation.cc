#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "fixed_step_clock.h"
#include "impact_lab_model.h"
#include "sim_ui.h"
#include "simulation_history.h"
#include "simulations.h"

namespace tiny2d::sandbox {
namespace {

constexpr double kMaximumFrameTime = 0.25;
constexpr float kControlStart = 300.0f;
constexpr float kSliderWidth = 300.0f;
constexpr float kInputWidth = 105.0f;

enum class SetupAction {
  kNone,
  kStart,
  kBack,
};

enum class MonitorAction {
  kNone,
  kReplay,
  kBack,
};

SetupAction DrawSetupScreen(ImpactLabConfig* config,
                            const std::string& runtime_error) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ImpactLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V17 ImpactLab: continuous collision evidence");
  ImGui::TextDisabled(
      "SI units. +X points right and +Y points down. Both lanes receive the "
      "same input.");
  ImGui::TextDisabled(
      "The V16 lane samples endpoints; the V17 lane finds the earliest "
      "circle-circle impact.");
  ImGui::TextDisabled(
      "Use a slider or type an exact value; finite inputs are clamped to the "
      "displayed range.");
  ImGui::Spacing();

  const bool grazing = config->preset == ImpactLabPreset::kSupportedGrazing;
  ImGui::TextUnformatted(grazing ? "V16-supported grazing tunnel"
                                 : "High-speed diameter skip");
  ImGui::Separator();
  ImGui::TextWrapped(
      grazing ? "Two 0.1 m-radius circles move at opposite speeds. Their swept "
                "paths touch even though neither endpoint overlaps."
              : "Circle A travels farther than its 0.2 m diameter in one fixed "
                "step and crosses stationary circle B.");
  ImGui::Spacing();

  ui::SliderInputFloat("Circle A mass (kg)", &config->mass_a_kg,
                       kImpactLabMinimumMassKg, kImpactLabMaximumMassKg, "%.4g",
                       kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Circle B mass (kg)", &config->mass_b_kg,
                       kImpactLabMinimumMassKg, kImpactLabMaximumMassKg, "%.4g",
                       kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat(
      grazing ? "Each circle speed (m/s)" : "Circle A speed (m/s)",
      &config->speed_m_s,
      grazing ? kImpactLabMinimumGrazingSpeedMps
              : kImpactLabMinimumDiameterSkipSpeedMps,
      grazing ? kImpactLabMaximumGrazingSpeedMps
              : kImpactLabMaximumDiameterSkipSpeedMps,
      "%.4g", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Restitution e", &config->restitution, 0.0f, 1.0f,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);

  const char* error = GetImpactLabConfigError(*config);
  if (error == nullptr) {
    const ImpactLabDerived reference =
        CalculateImpactLabDerived(*config, MakeInitialImpactLabState(*config));
    ImGui::Spacing();
    ImGui::Text("Fixed step: %.6f ms", kImpactLabPhysicsStep * 1000.0f);
    ImGui::Text("Analytical entry TOI: %.3f us",
                reference.entry_time_seconds * 1000000.0);
    ImGui::Text("Unresolved exit: %.3f us",
                reference.exit_time_seconds * 1000000.0);
    ImGui::Text("A travel per step: %.4f m (%.3f diameters)",
                reference.moving_distance_per_step_m,
                reference.moving_distance_to_diameter_ratio);
  }

  ImGui::SetCursorPosY(
      std::max(ImGui::GetCursorPosY(), io.DisplaySize.y - 82.0f));
  if (!runtime_error.empty()) {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error.c_str());
  } else if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f}, "Ready");
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 3.0f * spacing) / 4.0f;
  SetupAction action = SetupAction::kNone;
  if (ImGui::Button("Back", {button_width, 38.0f})) {
    action = SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("V16 grazing evidence", {button_width, 38.0f})) {
    *config = MakeSupportedGrazingImpactConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Diameter skip", {button_width, 38.0f})) {
    *config = MakeDiameterSkipImpactConfig();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Run one evidence step", {button_width, 38.0f})) {
    action = SetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

struct ViewBounds {
  float minimum_x;
  float maximum_x;
  float minimum_y;
  float maximum_y;
};

ViewBounds GetViewBounds(const ImpactLabState& initial,
                         const ImpactLabState& current,
                         const ImpactLabDerived& derived) {
  std::array<Vec2, 10> points = {
      initial.ccd_circles[0].position,
      initial.ccd_circles[1].position,
      current.discrete_circles[0].position,
      current.discrete_circles[1].position,
      current.ccd_circles[0].position,
      current.ccd_circles[1].position,
      derived.expected_position_a_m,
      derived.expected_position_b_m,
      {initial.ccd_circles[0].position.x +
           initial.ccd_circles[0].velocity.x * kImpactLabPhysicsStep,
       initial.ccd_circles[0].position.y +
           initial.ccd_circles[0].velocity.y * kImpactLabPhysicsStep},
      {initial.ccd_circles[1].position.x +
           initial.ccd_circles[1].velocity.x * kImpactLabPhysicsStep,
       initial.ccd_circles[1].position.y +
           initial.ccd_circles[1].velocity.y * kImpactLabPhysicsStep},
  };
  float minimum_x = points[0].x;
  float maximum_x = points[0].x;
  float minimum_y = points[0].y;
  float maximum_y = points[0].y;
  for (Vec2 point : points) {
    minimum_x = std::min(minimum_x, point.x);
    maximum_x = std::max(maximum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_y = std::max(maximum_y, point.y);
  }
  constexpr float kPaddingM = 0.3f;
  const float center_x = (minimum_x + maximum_x) * 0.5f;
  const float center_y = (minimum_y + maximum_y) * 0.5f;
  const float half_width =
      std::max((maximum_x - minimum_x) * 0.5f, 0.45f) + kPaddingM;
  const float half_height =
      std::max((maximum_y - minimum_y) * 0.5f, 0.2f) + kPaddingM;
  return {center_x - half_width, center_x + half_width, center_y - half_height,
          center_y + half_height};
}

struct LaneTransform {
  ImVec2 origin;
  float pixels_per_meter;
  ViewBounds bounds;
};

LaneTransform MakeLaneTransform(ImVec2 minimum, ImVec2 maximum,
                                ViewBounds bounds) {
  constexpr float kHorizontalPadding = 18.0f;
  constexpr float kTopPadding = 42.0f;
  constexpr float kBottomPadding = 18.0f;
  const float width = maximum.x - minimum.x - 2.0f * kHorizontalPadding;
  const float height = maximum.y - minimum.y - kTopPadding - kBottomPadding;
  const float scale = std::min(width / (bounds.maximum_x - bounds.minimum_x),
                               height / (bounds.maximum_y - bounds.minimum_y));
  const float content_width = (bounds.maximum_x - bounds.minimum_x) * scale;
  const float content_height = (bounds.maximum_y - bounds.minimum_y) * scale;
  return {{minimum.x + (maximum.x - minimum.x - content_width) * 0.5f -
               bounds.minimum_x * scale,
           minimum.y + kTopPadding + (height - content_height) * 0.5f -
               bounds.minimum_y * scale},
          scale,
          bounds};
}

ImVec2 WorldToScreen(const LaneTransform& transform, Vec2 position) {
  return {transform.origin.x + position.x * transform.pixels_per_meter,
          transform.origin.y + position.y * transform.pixels_per_meter};
}

void DrawCircle(ImDrawList* draw_list, const LaneTransform& transform,
                const Circle& circle, ImU32 color, const char* name) {
  const ImVec2 center = WorldToScreen(transform, circle.position);
  const float radius = circle.radius * transform.pixels_per_meter;
  draw_list->AddCircleFilled(center, radius, color, 40);
  draw_list->AddCircle(center, radius, IM_COL32(230, 240, 250, 255), 40, 2.0f);
  const ImVec2 text_size = ImGui::CalcTextSize(name);
  draw_list->AddText(
      {center.x - text_size.x * 0.5f, center.y - text_size.y * 0.5f},
      IM_COL32(20, 24, 30, 255), name);
  ui::DrawArrowFrom(draw_list, center, {circle.velocity.x, circle.velocity.y},
                    45.0f, IM_COL32(255, 210, 100, 245));
}

void DrawLane(ImDrawList* draw_list, ImVec2 minimum, ImVec2 maximum,
              const char* title, const char* status, ImU32 status_color,
              const ImpactLabState& initial, const std::vector<Circle>& circles,
              const ImpactLabDerived& derived, ViewBounds bounds,
              bool draw_reference) {
  draw_list->AddRectFilled(minimum, maximum, IM_COL32(24, 28, 35, 245), 8.0f);
  draw_list->AddRect(minimum, maximum, IM_COL32(85, 100, 120, 255), 8.0f, 0,
                     2.0f);
  draw_list->AddText({minimum.x + 14.0f, minimum.y + 12.0f},
                     IM_COL32(205, 225, 245, 255), title);
  const ImVec2 status_size = ImGui::CalcTextSize(status);
  draw_list->AddText({maximum.x - status_size.x - 14.0f, minimum.y + 12.0f},
                     status_color, status);

  const LaneTransform transform = MakeLaneTransform(minimum, maximum, bounds);
  const Circle& initial_a = initial.ccd_circles[0];
  const Circle& initial_b = initial.ccd_circles[1];
  const Vec2 unresolved_end_a{
      initial_a.position.x + initial_a.velocity.x * kImpactLabPhysicsStep,
      initial_a.position.y + initial_a.velocity.y * kImpactLabPhysicsStep};
  const Vec2 unresolved_end_b{
      initial_b.position.x + initial_b.velocity.x * kImpactLabPhysicsStep,
      initial_b.position.y + initial_b.velocity.y * kImpactLabPhysicsStep};
  draw_list->AddLine(WorldToScreen(transform, initial_a.position),
                     WorldToScreen(transform, unresolved_end_a),
                     IM_COL32(95, 165, 240, 125), 2.0f);
  draw_list->AddLine(WorldToScreen(transform, initial_b.position),
                     WorldToScreen(transform, unresolved_end_b),
                     IM_COL32(255, 145, 95, 125), 2.0f);

  const Vec2 contact_a{
      static_cast<float>(initial_a.position.x +
                         initial_a.velocity.x * derived.entry_time_seconds),
      static_cast<float>(initial_a.position.y +
                         initial_a.velocity.y * derived.entry_time_seconds)};
  const Vec2 contact_b{
      static_cast<float>(initial_b.position.x +
                         initial_b.velocity.x * derived.entry_time_seconds),
      static_cast<float>(initial_b.position.y +
                         initial_b.velocity.y * derived.entry_time_seconds)};
  const ImVec2 contact_midpoint = WorldToScreen(
      transform,
      {(contact_a.x + contact_b.x) * 0.5f, (contact_a.y + contact_b.y) * 0.5f});
  draw_list->AddCircle(contact_midpoint, 7.0f, IM_COL32(110, 245, 170, 255), 20,
                       2.0f);
  ui::DrawArrowFrom(draw_list, contact_midpoint,
                    {derived.contact_normal.x, derived.contact_normal.y}, 36.0f,
                    IM_COL32(110, 245, 170, 220));

  if (draw_reference) {
    for (Vec2 position :
         {derived.expected_position_a_m, derived.expected_position_b_m}) {
      draw_list->AddCircle(WorldToScreen(transform, position),
                           kImpactLabCircleRadiusM * transform.pixels_per_meter,
                           IM_COL32(100, 245, 155, 190), 40, 2.0f);
    }
  }
  DrawCircle(draw_list, transform, circles[0], IM_COL32(75, 160, 245, 255),
             "A");
  DrawCircle(draw_list, transform, circles[1], IM_COL32(255, 140, 85, 255),
             "B");
}

void DrawImpactLabScene(const ImpactLabConfig& config,
                        const ImpactLabState& state) {
  const ImpactLabState initial = MakeInitialImpactLabState(config);
  const ImpactLabDerived derived = CalculateImpactLabDerived(config, state);
  const ViewBounds bounds = GetViewBounds(initial, state, derived);
  const ImVec2 display = ImGui::GetIO().DisplaySize;
  const float left = std::min(450.0f, display.x * 0.42f);
  const float width = std::max(display.x - left - 20.0f, 200.0f);
  const float height = std::max((display.y - 52.0f) * 0.5f, 160.0f);
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const bool complete = state.time_seconds >= kImpactLabPhysicsStep;

  DrawLane(
      draw_list, {left, 16.0f}, {left + width, 16.0f + height},
      "V16 DISCRETE: endpoint overlap only",
      !complete
          ? "PENDING"
          : (derived.discrete_tunneled ? "MISSED / TUNNELED" : "CONTACT FOUND"),
      !complete ? IM_COL32(180, 185, 195, 255)
                : (derived.discrete_tunneled ? IM_COL32(255, 100, 90, 255)
                                             : IM_COL32(100, 235, 150, 255)),
      initial, state.discrete_circles, derived, bounds, false);
  DrawLane(
      draw_list, {left, 28.0f + height}, {left + width, 28.0f + 2.0f * height},
      "V17 CCD: earliest swept-circle impact",
      !complete ? "PENDING"
                : (derived.ccd_resolved ? "RESOLVED" : "REFERENCE MISMATCH"),
      !complete ? IM_COL32(180, 185, 195, 255)
                : (derived.ccd_resolved ? IM_COL32(100, 235, 150, 255)
                                        : IM_COL32(255, 100, 90, 255)),
      initial, state.ccd_circles, derived, bounds, true);
}

MonitorAction DrawMonitor(const ImpactLabConfig& config,
                          const ImpactLabState& current_state,
                          const std::vector<ImpactLabState>& history,
                          bool evidence_complete, double* inspect_time,
                          bool* follow_live, const std::string& runtime_error,
                          ImpactLabState* displayed_state) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({420.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ImpactLab monitor", nullptr, kWindowFlags);

  ImGui::TextColored(evidence_complete ? ImVec4{1.0f, 0.7f, 0.25f, 1.0f}
                                       : ImVec4{0.35f, 0.9f, 0.5f, 1.0f},
                     "%s",
                     evidence_complete ? "PAUSED AFTER ONE STEP" : "CAPTURING");
  ImGui::Text("t = %.7f s | dt = %.7f s", current_state.time_seconds,
              kImpactLabPhysicsStep);
  ImGui::TextDisabled("Space: replay the evidence step");
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
      current_state.time_seconds, static_cast<double>(kImpactLabPhysicsStep));
  ImGui::SetNextItemWidth(105.0f);
  bool inspect_changed = ImGui::SliderScalar(
      "##impact_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.6f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(75.0f);
  inspect_changed |= ImGui::InputDouble("##impact_history_input", inspect_time,
                                        0.0, 0.0, "%.6f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const ImpactLabState* inspected = FindImpactLabState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  *displayed_state = *inspected;
  const ImpactLabDerived derived =
      CalculateImpactLabDerived(config, *inspected);

  ImGui::BeginChild("##impact_telemetry", {0.0f, -48.0f}, false);
  ImGui::TextUnformatted(config.preset == ImpactLabPreset::kSupportedGrazing
                             ? "Preset: V16-supported grazing tunnel"
                             : "Preset: high-speed diameter skip");
  ImGui::Text("mA=%.4g kg | mB=%.4g kg | e=%.3f", config.mass_a_kg,
              config.mass_b_kg, config.restitution);
  ImGui::Text("A travel=%.6f m | %.4f diameters",
              derived.moving_distance_per_step_m,
              derived.moving_distance_to_diameter_ratio);
  ImGui::Text("entry TOI=%.3f us | unresolved exit=%.3f us",
              derived.entry_time_seconds * 1000000.0,
              derived.exit_time_seconds * 1000000.0);
  ImGui::Text("contact n=(%+.6f,%+.6f)", derived.contact_normal.x,
              derived.contact_normal.y);

  ImGui::Spacing();
  ImGui::TextUnformatted("Analytical first-impact reference");
  ImGui::Separator();
  ImGui::Text("vA'=(%+.6f,%+.6f) m/s", derived.expected_velocity_a_m_s.x,
              derived.expected_velocity_a_m_s.y);
  ImGui::Text("vB'=(%+.6f,%+.6f) m/s", derived.expected_velocity_b_m_s.x,
              derived.expected_velocity_b_m_s.y);
  ImGui::Text("xA'=(%+.6f,%+.6f) m", derived.expected_position_a_m.x,
              derived.expected_position_a_m.y);
  ImGui::Text("xB'=(%+.6f,%+.6f) m", derived.expected_position_b_m.x,
              derived.expected_position_b_m.y);

  ImGui::Spacing();
  ImGui::TextUnformatted("V16 discrete lane");
  ImGui::Separator();
  ImGui::Text("A: x=(%+.6f,%+.6f), v=(%+.6f,%+.6f)",
              inspected->discrete_circles[0].position.x,
              inspected->discrete_circles[0].position.y,
              inspected->discrete_circles[0].velocity.x,
              inspected->discrete_circles[0].velocity.y);
  ImGui::Text("B: x=(%+.6f,%+.6f), v=(%+.6f,%+.6f)",
              inspected->discrete_circles[1].position.x,
              inspected->discrete_circles[1].position.y,
              inspected->discrete_circles[1].velocity.x,
              inspected->discrete_circles[1].velocity.y);
  ImGui::TextColored(
      derived.discrete_tunneled ? ImVec4{1.0f, 0.35f, 0.3f, 1.0f}
                                : ImVec4{0.75f, 0.78f, 0.82f, 1.0f},
      "%s",
      derived.discrete_tunneled ? "Swept contact missed: TUNNELED"
                                : "Awaiting the evidence endpoint");

  ImGui::Spacing();
  ImGui::TextUnformatted("V17 CCD lane");
  ImGui::Separator();
  ImGui::Text("A: x=(%+.6f,%+.6f), v=(%+.6f,%+.6f)",
              inspected->ccd_circles[0].position.x,
              inspected->ccd_circles[0].position.y,
              inspected->ccd_circles[0].velocity.x,
              inspected->ccd_circles[0].velocity.y);
  ImGui::Text("B: x=(%+.6f,%+.6f), v=(%+.6f,%+.6f)",
              inspected->ccd_circles[1].position.x,
              inspected->ccd_circles[1].position.y,
              inspected->ccd_circles[1].velocity.x,
              inspected->ccd_circles[1].velocity.y);
  ImGui::Text("position error=%.7g m | velocity error=%.7g m/s",
              derived.ccd_position_error_m, derived.ccd_velocity_error_m_s);
  ImGui::Text("|delta p|=%.7g kg*m/s | |delta K|=%.7g J",
              derived.ccd_momentum_error_kg_m_s,
              derived.ccd_kinetic_energy_error_j);
  ImGui::TextColored(derived.ccd_resolved ? ImVec4{0.35f, 0.9f, 0.5f, 1.0f}
                                          : ImVec4{0.75f, 0.78f, 0.82f, 1.0f},
                     "%s",
                     derived.ccd_resolved
                         ? "CCD matches the analytical reference"
                         : "Awaiting the evidence endpoint");

  if (!runtime_error.empty()) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error.c_str());
  }
  ImGui::EndChild();

  const float width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  MonitorAction action = MonitorAction::kNone;
  ImGui::BeginDisabled(!runtime_error.empty());
  if (ImGui::Button("Replay evidence", {width, 36.0f})) {
    action = MonitorAction::kReplay;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Stop and choose model", {width, 36.0f})) {
    action = MonitorAction::kBack;
  }
  ImGui::End();
  return action;
}

}  // namespace

SimulationResult RunImpactLabSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  ImpactLabConfig config = MakeSupportedGrazingImpactConfig();
  ImpactLabState state = MakeInitialImpactLabState(config);
  std::vector<ImpactLabState> history;
  bool simulation_started = false;
  bool evidence_complete = false;
  bool follow_live = true;
  double inspect_time = 0.0;
  std::string runtime_error;
  FixedStepClock clock(static_cast<double>(SDL_GetPerformanceFrequency()),
                       SDL_GetPerformanceCounter());

  const auto reset_evidence = [&](Uint64 current_time) {
    state = MakeInitialImpactLabState(config);
    history.clear();
    const bool recorded =
        AppendHistorySample(&history, state, &ImpactLabState::time_seconds);
    simulation_started = true;
    evidence_complete = !recorded;
    follow_live = true;
    inspect_time = 0.0;
    runtime_error =
        recorded ? "" : "ImpactLab produced an invalid history time.";
    clock.Reset(current_time);
  };

  while (true) {
    bool return_requested = false;
    SimulationResult return_result = SimulationResult::kBackToSelection;
    bool replay_requested = false;
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        return_requested = true;
        return_result = SimulationResult::kQuit;
      } else if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                 event.key.keysym.sym == SDLK_SPACE && simulation_started &&
                 runtime_error.empty()) {
        replay_requested = true;
      }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint64 current_time = SDL_GetPerformanceCounter();
    if (!return_requested && !simulation_started) {
      clock.Reset(current_time);
      const SetupAction action = DrawSetupScreen(&config, runtime_error);
      if (action == SetupAction::kBack) {
        return_requested = true;
      } else if (action == SetupAction::kStart) {
        try {
          reset_evidence(current_time);
        } catch (const std::exception& error) {
          runtime_error = error.what();
        }
      }
    } else if (simulation_started) {
      if (replay_requested) {
        try {
          reset_evidence(current_time);
        } catch (const std::exception& error) {
          runtime_error = error.what();
          evidence_complete = true;
        }
      }
      if (!return_requested && !evidence_complete && runtime_error.empty()) {
        clock.Accumulate(current_time, kMaximumFrameTime);
        if (clock.HasStep(kImpactLabPhysicsStep)) {
          try {
            if (!StepImpactLab(config, kImpactLabPhysicsStep, &state)) {
              const char* state_error = GetImpactLabStateError(config, state);
              runtime_error =
                  state_error != nullptr
                      ? state_error
                      : "ImpactLab rejected the fixed evidence step.";
            } else if (!AppendHistorySample(&history, state,
                                            &ImpactLabState::time_seconds)) {
              runtime_error = "ImpactLab history time stopped increasing.";
            }
          } catch (const std::exception& error) {
            runtime_error = error.what();
          }
          evidence_complete = true;
          clock.DiscardPendingSteps();
        }
      } else {
        clock.Reset(current_time);
      }

      ImpactLabState displayed_state = state;
      const MonitorAction action =
          !return_requested
              ? DrawMonitor(config, state, history, evidence_complete,
                            &inspect_time, &follow_live, runtime_error,
                            &displayed_state)
              : MonitorAction::kNone;
      DrawImpactLabScene(config, displayed_state);
      if (action == MonitorAction::kReplay) {
        try {
          reset_evidence(current_time);
        } catch (const std::exception& error) {
          runtime_error = error.what();
          evidence_complete = true;
        }
      } else if (action == MonitorAction::kBack) {
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
