#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "app/lab_shell.h"
#include "fixed_step_clock.h"
#include "incline_spring_model.h"
#include "sim_ui.h"
#include "simulations.h"
#include "tiny2d_engine.h"

namespace tiny2d::sandbox::incline_spring {
namespace {

constexpr float kMonitorHeight = 250.0f;
constexpr float kControlStart = 260.0f;
constexpr float kSliderWidth = 320.0f;
constexpr float kInputWidth = 100.0f;
constexpr float kDegreesToRadians = 0.01745329252f;
constexpr float kSpringAmplitude = 10.0f;
constexpr int kSpringCoilCount = 10;
constexpr double kMaxFrameTime = 0.25;
constexpr std::array<int, 6> kRectangleIndices = {0, 1, 2, 0, 2, 3};
constexpr std::array<const char*, 2> kBodyLabels = {"A", "B"};

void DrawChargeControls(const char* label, BodyConfig* body_config) {
  ImGui::PushID(label);
  ImGui::Checkbox(label, &body_config->charged);
  ImGui::BeginDisabled(!body_config->charged);
  ui::SliderInputFloat("Charge q (C)", &body_config->charge, -1000.0f, 1000.0f,
                       "%.6g", kControlStart, kSliderWidth, kInputWidth);
  ImGui::EndDisabled();
  ImGui::PopID();
}

void DrawBodyControls(const char* title, BodyConfig* body_config,
                      const SimulationConfig& simulation_config) {
  ImGui::PushID(title);
  ImGui::TextUnformatted(title);
  // This release exposes unit blocks only. Keep length_units and the rectangle
  // physics so the plank control can be restored in a later version.
  body_config->length_units = 1.0f;
  ImGui::TextDisabled("This release uses a 1 x 1 block.");

  const bool starts_on_ramp =
      simulation_config.ramp_enabled && !body_config->starts_on_floor;
  const float minimum_x =
      GetMinimumBodySurfaceX(*body_config, simulation_config);
  const float maximum_x =
      GetMaximumBodySurfaceX(*body_config, simulation_config);
  ClampBodySurfaceX(*body_config, simulation_config);
  ImGui::TextDisabled(
      starts_on_ramp ? "Position: right is higher. Speed: positive is downhill."
                     : "Position is on the floor. Speed: positive moves left.");
  ui::SliderInputFloat(starts_on_ramp ? "Ramp x (px)" : "Floor x (px)",
                       &body_config->surface_x, minimum_x, maximum_x, "%.1f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Pixel initial speed (px/s)",
                       &body_config->downhill_speed, -5000.0f, 5000.0f, "%.1f",
                       kControlStart, kSliderWidth, kInputWidth);
  ImGui::PopID();
}

bool DrawSetupScreen(SimulationConfig* config, bool* back_to_selection) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("Tiny2D setup", nullptr, kWindowFlags);

  if (ImGui::Button("Back to model selection")) {
    *back_to_selection = true;
  }
  ImGui::Spacing();

  ImGui::TextColored({1.0f, 0.55f, 0.3f, 1.0f}, "Tiny2D Physics Lab");
  ImGui::TextDisabled("Drag a slider, or type an exact value beside it.");
  ImGui::Spacing();

  ImGui::TextUnformatted("Physical data (SI)");
  ImGui::Separator();
  ImGui::TextDisabled(
      "Enter the problem values. Body A defines the speed and mass scales; "
      "Body B uses the same mass scale.");
  ImGui::Checkbox("Enable ramp", &config->ramp_enabled);
  ui::SliderInputFloat("Floor length (m)", &config->real_floor_length_m, 0.1f,
                       10000.0f, "%.3f", kControlStart, kSliderWidth,
                       kInputWidth, ImGuiSliderFlags_Logarithmic);
  ImGui::BeginDisabled(!config->ramp_enabled);
  ui::SliderInputFloat("Ramp angle (degrees)", &config->ramp_angle_degrees,
                       5.0f, 45.0f, "%.1f", kControlStart, kSliderWidth,
                       kInputWidth);
  ui::SliderInputFloat("Ramp length (m)", &config->real_ramp_length_m, 0.1f,
                       10000.0f, "%.3f", kControlStart, kSliderWidth,
                       kInputWidth, ImGuiSliderFlags_Logarithmic);
  ImGui::EndDisabled();
  ui::SliderInputFloat("Body A reference speed (m/s)",
                       &config->reference_speed_mps, 0.01f, 1000.0f, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Body A reference mass (kg)", &config->body_a.mass_kg,
                       0.01f, 10000.0f, "%.3f", kControlStart, kSliderWidth,
                       kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Body B mass (kg)", &config->body_b.mass_kg, 0.01f,
                       10000.0f, "%.3f", kControlStart, kSliderWidth,
                       kInputWidth, ImGuiSliderFlags_Logarithmic);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Gravity (m/s^2)");
  ImGui::SameLine(260.0f);
  if (ImGui::RadioButton("9.8", config->gravity_mps2 == 9.8f)) {
    config->gravity_mps2 = 9.8f;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("10", config->gravity_mps2 == 10.0f)) {
    config->gravity_mps2 = 10.0f;
  }
  if (std::abs(config->body_a.downhill_speed) > 0.001f) {
    ImGui::TextDisabled(
        "Computed: %.2f px/m | %.4f real s/simulation s | gravity %.2f "
        "px/s^2",
        GetPixelsPerMeter(*config), GetRealSecondsPerSimulationSecond(*config),
        GetPixelGravity(*config));
    ImGui::TextDisabled("Mass scale from Body A: %.4f engine mass/kg",
                        GetEngineMassPerKilogram(*config));
  }
  ImGui::Spacing();

  ImGui::TextUnformatted("Physical model");
  ImGui::Separator();
  ui::SliderInputFloat("Surface friction", &config->friction, 0.0f, 5.0f,
                       "%.2f", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Collision bounciness", &config->restitution, 0.0f, 1.0f,
                       "%.2f", kControlStart, kSliderWidth, kInputWidth);

  ImGui::Checkbox("Enable electric field", &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  ImGui::TextDisabled(
      "Uniform E field in N/C. Angle is counterclockwise from +X: 0 right, "
      "90 up, -90 down.");
  ui::SliderInputFloat("Electric field strength E (N/C)",
                       &config->electric_field_strength_n_per_c, 0.0f,
                       1000000.0f, "%.6g", kControlStart, kSliderWidth,
                       kInputWidth);
  ui::SliderInputFloat("Electric field angle (degrees)",
                       &config->electric_field_angle_degrees, -180.0f, 180.0f,
                       "%.1f", kControlStart, kSliderWidth, kInputWidth);
  DrawChargeControls("Body A carries charge", &config->body_a);
  DrawChargeControls("Body B carries charge", &config->body_b);
  ImGui::EndDisabled();
  ImGui::Checkbox("Enable left spring", &config->spring_enabled);

  if (ImGui::CollapsingHeader("Advanced engine values")) {
    ImGui::TextDisabled(
        "These values control the pixel simulation and normally stay at their "
        "defaults.");
    ui::SliderInputFloat("Body A engine reference mass",
                         &config->body_a_engine_mass, 0.01f, 1000.0f, "%.3f",
                         kControlStart, kSliderWidth, kInputWidth,
                         ImGuiSliderFlags_Logarithmic);
    ImGui::BeginDisabled(!config->spring_enabled);
    ui::SliderInputFloat("Spring strength", &config->spring_stiffness, 0.1f,
                         100.0f, "%.1f", kControlStart, kSliderWidth,
                         kInputWidth, ImGuiSliderFlags_Logarithmic);
    ImGui::EndDisabled();
    ImGui::Separator();
    DrawBodyControls("Body A", &config->body_a, *config);
    ImGui::Separator();
    DrawBodyControls("Body B", &config->body_b, *config);
  }
  ClampBodySurfaceX(config->body_a, *config);
  ClampBodySurfaceX(config->body_b, *config);
  ImGui::Spacing();

  const float button_width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  if (ImGui::Button("Restore defaults", {button_width, 38.0f})) {
    *config = SimulationConfig{};
  }
  const char* error = GetConfigError(*config);
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  const bool start = ImGui::Button("Start simulation", {button_width, 38.0f});
  ImGui::EndDisabled();

  if (error != nullptr) {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  } else {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f}, "Ready to start.");
  }

  ImGui::End();
  return start;
}

void ShowError(const char* message) {
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tiny2D Engine", message,
                           nullptr);
}

void DrawSpring(SDL_Renderer* renderer, const State& state) {
  const SimulationConfig& config = state.config;
  const float spring_anchor_x = GetSpringAnchorX(config);
  const float spring_end_x = GetSpringEndX(state);

  constexpr float kSpringY =
      static_cast<float>(kAreaHeight) - kUnitLength * 0.5f;
  std::array<SDL_FPoint, kSpringCoilCount + 2> points{};
  points.front() = {spring_anchor_x, kSpringY};
  for (int i = 1; i <= kSpringCoilCount; ++i) {
    const float fraction =
        static_cast<float>(i) / static_cast<float>(kSpringCoilCount + 1);
    points[i] = {
        spring_anchor_x + (spring_end_x - spring_anchor_x) * fraction,
        kSpringY + (i % 2 == 0 ? kSpringAmplitude : -kSpringAmplitude)};
  }
  points.back() = {spring_end_x, kSpringY};

  SDL_SetRenderDrawColor(renderer, 245, 200, 70, 255);
  SDL_RenderDrawLinesF(renderer, points.data(),
                       static_cast<int>(points.size()));
  SDL_RenderDrawLineF(renderer, spring_anchor_x, kSpringY - kUnitLength * 0.5f,
                      spring_anchor_x, kSpringY + kUnitLength * 0.5f);
  SDL_RenderDrawLineF(renderer, spring_end_x, kSpringY - kUnitLength * 0.5f,
                      spring_end_x, kSpringY + kUnitLength * 0.5f);
}

void DrawRectangle(SDL_Renderer* renderer, const tiny2d::Rectangle& rectangle) {
  const std::array<tiny2d::Vec2, 4> corners = tiny2d::GetVertices(rectangle);
  std::array<SDL_Vertex, 4> vertices{};
  const SDL_Color color = rectangle.mass > 0.0f ? SDL_Color{255, 100, 100, 255}
                                                : SDL_Color{90, 110, 120, 255};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    vertices[i].position = {corners[i].x, corners[i].y};
    vertices[i].color = color;
  }
  SDL_RenderGeometry(
      renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()),
      kRectangleIndices.data(), static_cast<int>(kRectangleIndices.size()));
}

void DrawMonitorWindow(const State& state, bool* paused, double* inspect_time,
                       bool* follow_live, bool* back_to_selection) {
  const SimulationConfig& config = state.config;
  const double simulation_time = state.time;
  const float length_scale = GetPixelsPerMeter(config);
  const float speed_scale = GetPixelSpeedPerMeterPerSecond(config);
  const double time_scale = GetRealSecondsPerSimulationSecond(config);
  const double real_simulation_time = simulation_time * time_scale;
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize({io.DisplaySize.x, kMonitorHeight});
  ImGui::SetNextWindowBgAlpha(0.92f);
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("InclineLab monitor", nullptr, kWindowFlags);

  ImGui::TextColored(*paused ? ImVec4{1.0f, 0.7f, 0.25f, 1.0f}
                             : ImVec4{0.35f, 0.9f, 0.5f, 1.0f},
                     "%s", *paused ? "PAUSED" : "RUNNING");
  ImGui::SameLine();
  ImGui::Text("t = %.3f s", real_simulation_time);
  ImGui::SameLine();
  ImGui::TextDisabled("Space: pause/resume");
  ImGui::SameLine();
  if (ImGui::Button(*paused ? "Resume" : "Pause", {90.0f, 0.0f})) {
    *paused = !*paused;
  }
  ImGui::SameLine();
  if (ImGui::Button("Stop and choose model", {180.0f, 0.0f})) {
    *back_to_selection = true;
  }
  ImGui::Separator();

  if (ImGui::Checkbox("Live", follow_live) && *follow_live) {
    *inspect_time = real_simulation_time;
  }

  if (*follow_live) {
    *inspect_time = real_simulation_time;
  }
  const double maximum_inspect_time =
      std::max(real_simulation_time, kPhysicsStep * time_scale);
  if (ui::SliderInputDouble("Inspect time (s)", inspect_time, 0.0,
                            maximum_inspect_time, "%.3f", kControlStart,
                            kSliderWidth, kInputWidth)) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = real_simulation_time;
    } else {
      *inspect_time = std::min(*inspect_time, real_simulation_time);
    }
    *follow_live = false;
  }

  ImGui::Text(
      "System | friction %.2f | restitution %.2f | ramp %s %.1f deg | "
      "spring %s k=%.1f",
      config.friction, config.restitution, config.ramp_enabled ? "on" : "off",
      config.ramp_angle_degrees, config.spring_enabled ? "on" : "off",
      config.spring_stiffness);
  ImGui::Text(
      "Electric field | %s | E %.6g N/C | angle %.1f deg | qA %.6g C "
      "| qB %.6g C",
      config.electric_field_enabled ? "on" : "off",
      config.electric_field_strength_n_per_c,
      config.electric_field_angle_degrees,
      config.body_a.charged ? config.body_a.charge : 0.0f,
      config.body_b.charged ? config.body_b.charge : 0.0f);
  ImGui::SameLine();
  ImGui::TextDisabled("| Scale: %.2f px/m | gravity %.1f px/s^2 = %.1f m/s^2",
                      length_scale, GetPixelGravity(config),
                      config.gravity_mps2);
  ImGui::Separator();

  const SimulationSnapshot* snapshot =
      FindSnapshot(state, *inspect_time / time_scale);
  if (snapshot != nullptr &&
      ImGui::BeginTable("Telemetry", 7,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Body");
    ImGui::TableSetupColumn("Mass (kg)");
    ImGui::TableSetupColumn("Distance (m)");
    ImGui::TableSetupColumn("Position (m)");
    ImGui::TableSetupColumn("Signed speed (m/s)");
    ImGui::TableSetupColumn("Surface acceleration (m/s^2)");
    ImGui::TableSetupColumn("Vector acceleration (m/s^2)");
    ImGui::TableHeadersRow();
    const std::array<float, 2> masses_kg = {config.body_a.mass_kg,
                                            config.body_b.mass_kg};
    const std::array<float, 2> widths = {GetBodyWidth(config.body_a),
                                         GetBodyWidth(config.body_b)};
    const float acceleration_scale = length_scale / (speed_scale * speed_scale);
    for (std::size_t i = 0; i < snapshot->bodies.size(); ++i) {
      const BodyTelemetry& body = snapshot->bodies[i];
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(kBodyLabels[i]);
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", masses_kg[i]);
      ImGui::TableSetColumnIndex(2);
      if (config.ramp_enabled) {
        ImGui::Text("%+.3f",
                    GetSignedDistance(body, widths[i], config) / length_scale);
      } else {
        ImGui::TextDisabled("N/A");
      }
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("(%.3f, %.3f)", body.position.x / length_scale,
                  body.position.y / length_scale);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%+.3f", GetSignedSpeed(body, config) / speed_scale);
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%+.3f", GetSignedSurfaceAcceleration(body, config) *
                               acceleration_scale);
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("(%.3f, %.3f)", body.acceleration.x * acceleration_scale,
                  body.acceleration.y * acceleration_scale);
    }
    ImGui::EndTable();
  }

  ImGui::End();
}

void DrawSimulationLabels(const std::vector<tiny2d::Rectangle>& bodies,
                          const SimulationConfig& config) {
  constexpr float kBodyFontSize = 18.0f;
  constexpr float kAngleFontSize = 16.0f;
  constexpr float kAngleArcRadius = 48.0f;
  constexpr float kAngleLabelDistance = 72.0f;
  constexpr float kMaximumTextWidth = 1000.0f;

  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  ImFont* font = ImGui::GetFont();

  if (config.electric_field_enabled) {
    const float field_angle =
        config.electric_field_angle_degrees * kDegreesToRadians;
    const ImVec2 field_direction{std::cos(field_angle), -std::sin(field_angle)};
    ui::DrawCenteredArrow(draw_list, {100.0f, 310.0f}, field_direction, 80.0f,
                          IM_COL32(100, 225, 150, 255));
    char field_label[80];
    std::snprintf(field_label, sizeof(field_label),
                  "E = %.4g N/C, %.2f deg from +X CCW",
                  config.electric_field_strength_n_per_c,
                  config.electric_field_angle_degrees);
    draw_list->AddText({32.0f, 365.0f}, IM_COL32(120, 235, 165, 255),
                       field_label);
  }

  ui::DrawCenteredArrow(draw_list, {100.0f, 420.0f}, {0.0f, 1.0f}, 80.0f,
                        IM_COL32(245, 190, 90, 255));
  char gravity_label[64];
  std::snprintf(gravity_label, sizeof(gravity_label),
                "g = %.3g m/s^2, downward (-Y)", config.gravity_mps2);
  draw_list->AddText({32.0f, 475.0f}, IM_COL32(255, 205, 115, 255),
                     gravity_label);

  const std::size_t body_count = std::min(kBodyLabels.size(), bodies.size());
  const std::array<const BodyConfig*, 2> body_configs = {&config.body_a,
                                                         &config.body_b};
  for (std::size_t i = 0; i < body_count; ++i) {
    const ImVec2 text_size = font->CalcTextSizeA(
        kBodyFontSize, kMaximumTextWidth, 0.0f, kBodyLabels[i]);
    const ImVec2 text_position{bodies[i].position.x - text_size.x * 0.5f,
                               bodies[i].position.y - text_size.y * 0.5f};
    draw_list->AddText(font, kBodyFontSize, text_position,
                       IM_COL32(25, 25, 25, 255), kBodyLabels[i]);

    char body_details[64];
    std::snprintf(body_details, sizeof(body_details), "m=%.3g kg, q=%.3g C",
                  body_configs[i]->mass_kg,
                  body_configs[i]->charged ? body_configs[i]->charge : 0.0f);
    const ImVec2 details_size = font->CalcTextSizeA(
        kBodyFontSize, kMaximumTextWidth, 0.0f, body_details);
    constexpr float kLabelGap = 8.0f;
    float details_x = bodies[i].position.x + bodies[i].width * 0.5f + kLabelGap;
    if (i == 1 ||
        details_x + details_size.x > ImGui::GetIO().DisplaySize.x - kLabelGap) {
      details_x = bodies[i].position.x - bodies[i].width * 0.5f - kLabelGap -
                  details_size.x;
    }
    draw_list->AddText(
        font, kBodyFontSize,
        {details_x, bodies[i].position.y - details_size.y * 0.5f},
        IM_COL32(255, 210, 210, 255), body_details);
  }

  if (!config.ramp_enabled) {
    return;
  }

  const float ramp_angle = GetRampAngle(config);
  const ImVec2 junction{GetRampBottomX(config),
                        static_cast<float>(kAreaHeight)};
  draw_list->PathArcTo(junction, kAngleArcRadius, ramp_angle, 0.0f, 16);
  draw_list->PathStroke(IM_COL32(245, 245, 245, 255), 0, 2.0f);

  char angle_label[24];
  std::snprintf(angle_label, sizeof(angle_label), "i = %.1f deg",
                config.ramp_angle_degrees);
  const ImVec2 text_size =
      font->CalcTextSizeA(kAngleFontSize, kMaximumTextWidth, 0.0f, angle_label);
  const float label_angle = ramp_angle * 0.5f;
  const ImVec2 text_position{
      junction.x + std::cos(label_angle) * kAngleLabelDistance -
          text_size.x * 0.5f,
      junction.y + std::sin(label_angle) * kAngleLabelDistance -
          text_size.y * 0.5f};
  draw_list->AddText(font, kAngleFontSize, text_position,
                     IM_COL32(245, 245, 245, 255), angle_label);
}

}  // namespace

SimulationResult RunInclineSpringSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  namespace shell = tiny2d::sandbox::shell;
  SimulationConfig config;
  State state;
  bool simulation_started = false;
  bool simulation_paused = false;
  bool follow_live = true;
  bool back_to_selection = false;
  double inspect_time = 0.0;

  FixedStepClock clock(static_cast<double>(SDL_GetPerformanceFrequency()),
                       SDL_GetPerformanceCounter());
  const auto stop_simulation = [&](const char* message) {
    simulation_started = false;
    simulation_paused = false;
    state.bodies.clear();
    state.history.clear();
    clock.DiscardPendingSteps();
    ShowError(message);
  };

  const auto frame =
      [&](const shell::FrameInput& input) -> std::optional<SimulationResult> {
    if (input.quit_requested) {
      return SimulationResult::kQuit;
    }
    if (input.space_pressed && simulation_started) {
      simulation_paused = !simulation_paused;
    }

    if (!simulation_started) {
      clock.Reset(input.counter);
      if (DrawSetupScreen(&config, &back_to_selection)) {
        if (const char* error = Reset(config, state); error != nullptr) {
          ShowError(error);
        } else {
          simulation_paused = false;
          follow_live = true;
          inspect_time = 0.0;
          simulation_started = true;
        }
      }
    } else if (simulation_paused) {
      clock.Reset(input.counter);
    } else {
      clock.Accumulate(input.counter, kMaxFrameTime);

      while (clock.HasStep(kPhysicsStep)) {
        try {
          if (const char* error = Step(state, kPhysicsStep); error != nullptr) {
            stop_simulation(error);
            break;
          }
        } catch (const std::invalid_argument& error) {
          stop_simulation(error.what());
          break;
        }
        clock.ConsumeStep(kPhysicsStep);
      }
    }

    if (simulation_started) {
      DrawSimulationLabels(state.bodies, state.config);
      DrawMonitorWindow(state, &simulation_paused, &inspect_time, &follow_live,
                        &back_to_selection);
    }

    if (back_to_selection) {
      return SimulationResult::kBackToSelection;
    }
    return std::nullopt;
  };

  const auto underlay = [&](SDL_Renderer* target) {
    if (simulation_started) {
      if (state.config.spring_enabled) {
        DrawSpring(target, state);
      }
      for (const tiny2d::Rectangle& body : state.bodies) {
        DrawRectangle(target, body);
      }
    }
  };

  return shell::RunFrameLoop(renderer, frame, underlay,
                             SDL_Color{20, 20, 20, 255});
}

}  // namespace tiny2d::sandbox::incline_spring

namespace tiny2d::sandbox {

SimulationResult RunInclineSpringSimulation(SDL_Renderer* renderer) {
  return incline_spring::RunInclineSpringSimulation(renderer);
}

}  // namespace tiny2d::sandbox
