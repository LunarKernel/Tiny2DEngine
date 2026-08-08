#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "contact_lab_model.h"
#include "fixed_step_clock.h"
#include "sim_ui.h"
#include "simulation_history.h"
#include "simulations.h"

namespace tiny2d::sandbox {
namespace {

constexpr double kMaximumFrameTime = 0.25;
constexpr float kControlStart = 300.0f;
constexpr float kSliderWidth = 300.0f;
constexpr float kInputWidth = 105.0f;
constexpr float kRadiansToDegrees = 57.2957795131f;

enum class SetupAction {
  kNone,
  kStart,
  kBack,
};

void DrawInertiaModel(const char* label, CircleInertiaModel* model) {
  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(kControlStart);
  ImGui::SetNextItemWidth(kSliderWidth + ImGui::GetStyle().ItemSpacing.x +
                          kInputWidth);
  int selected = *model == CircleInertiaModel::kSolidDisk ? 0 : 1;
  if (ImGui::Combo("##model", &selected, "Solid disk\0Hoop\0")) {
    *model = selected == 0 ? CircleInertiaModel::kSolidDisk
                           : CircleInertiaModel::kHoop;
  }
  ImGui::PopID();
}

void DrawMaterialControls(const char* id, CollisionMaterial* material) {
  ImGui::PushID(id);
  ui::SliderInputFloat("Restitution e", &material->restitution, 0.0f, 1.0f,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Static friction mu_s", &material->static_friction, 0.0f,
                       kContactLabMaximumFrictionCoefficient, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Kinetic friction mu_k", &material->kinetic_friction,
                       0.0f, kContactLabMaximumFrictionCoefficient, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  if (std::isfinite(material->static_friction) &&
      std::isfinite(material->kinetic_friction) &&
      material->kinetic_friction > material->static_friction) {
    material->kinetic_friction = material->static_friction;
  }
  ImGui::PopID();
}

void DrawCircleControls(const char* name, ContactLabCircleConfig* circle,
                        bool allow_static) {
  ImGui::PushID(name);
  ImGui::TextUnformatted(name);
  ImGui::Separator();
  ui::SliderInputFloat("Mass (kg)", &circle->mass_kg, kContactLabMinimumMassKg,
                       kContactLabMaximumMassKg, "%.4g", kControlStart,
                       kSliderWidth, kInputWidth, ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Radius (m)", &circle->radius_m,
                       kContactLabMinimumRadiusM, kContactLabMaximumRadiusM,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  const float safe_radius =
      std::isfinite(circle->radius_m)
          ? std::clamp(circle->radius_m, kContactLabMinimumRadiusM,
                       kContactLabMaximumRadiusM)
          : kContactLabMaximumRadiusM;
  ui::SliderInputFloat("Initial x (m)", &circle->initial_position_m.x,
                       safe_radius, kContactLabAreaWidthM - safe_radius, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Initial y (m)", &circle->initial_position_m.y,
                       safe_radius, kContactLabAreaHeightM - safe_radius,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  if (allow_static && ImGui::Checkbox("Static body", &circle->is_static) &&
      circle->is_static) {
    circle->initial_velocity_m_s = {};
    circle->initial_angular_velocity_rad_s = 0.0f;
  }
  ImGui::BeginDisabled(circle->is_static);
  ui::SliderInputFloat("Initial vx (m/s)", &circle->initial_velocity_m_s.x,
                       -kContactLabMaximumInitialSpeedMps,
                       kContactLabMaximumInitialSpeedMps, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Initial vy (m/s)", &circle->initial_velocity_m_s.y,
                       -kContactLabMaximumInitialSpeedMps,
                       kContactLabMaximumInitialSpeedMps, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Initial omega (rad/s)",
                       &circle->initial_angular_velocity_rad_s,
                       -kContactLabMaximumInitialAngularSpeedRadS,
                       kContactLabMaximumInitialAngularSpeedRadS, "%.3f",
                       kControlStart, kSliderWidth, kInputWidth);
  ImGui::EndDisabled();
  DrawInertiaModel("Inertia model", &circle->inertia_model);
  DrawMaterialControls("material", &circle->material);
  ImGui::PopID();
}

SetupAction DrawSetupScreen(ContactLabConfig* config,
                            const std::string& runtime_error) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ContactLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V16 ContactLab: circle impacts and rolling contact");
  ImGui::TextDisabled(
      "SI units. +X points right, +Y points down, and positive rotation is "
      "clockwise.");
  ImGui::TextDisabled(
      "Use a slider or type an exact value; finite inputs are clamped to the "
      "displayed range.");

  ImGui::BeginChild("##contact_lab_parameters", {0.0f, -94.0f}, false);
  if (config->mode == ContactLabMode::kElasticImpact) {
    ImGui::TextUnformatted(
        "Elastic Impact | gravity off | circle-circle contact");
    ImGui::Spacing();
    DrawCircleControls("Circle A", &config->circle_a, false);
    ImGui::Spacing();
    DrawCircleControls("Circle B", &config->circle_b, true);
  } else {
    ImGui::TextUnformatted(
        "Rolling Contact | one circle on a fixed horizontal rectangle");
    ImGui::Spacing();
    DrawCircleControls("Rolling circle", &config->circle_a, false);
    ImGui::Spacing();
    ImGui::TextUnformatted("Surface and gravity");
    ImGui::Separator();
    ui::SliderInputFloat("Gravity g (m/s^2)", &config->gravity_m_s2, 0.0f,
                         kContactLabMaximumGravityMps2, "%.3f", kControlStart,
                         kSliderWidth, kInputWidth);
    DrawMaterialControls("surface", &config->surface_material);
  }
  ImGui::EndChild();

  const char* error = GetContactLabConfigError(*config);
  if (!runtime_error.empty()) {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error.c_str());
  } else if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f}, "Ready");
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 4.0f * spacing) / 5.0f;
  SetupAction action = SetupAction::kNone;
  if (ImGui::Button("Back", {button_width, 38.0f})) {
    action = SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Elastic impact", {button_width, 38.0f})) {
    *config = MakeElasticImpactConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Rolling solid disk", {button_width, 38.0f})) {
    *config = MakeRollingSolidDiskConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Rolling hoop", {button_width, 38.0f})) {
    *config = MakeRollingHoopConfig();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Start", {button_width, 38.0f})) {
    action = SetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

struct SceneTransform {
  ImVec2 origin;
  float pixels_per_meter;
};

SceneTransform GetSceneTransform() {
  const ImVec2 display = ImGui::GetIO().DisplaySize;
  const float left = std::min(450.0f, display.x * 0.42f);
  const float available_width = std::max(display.x - left - 24.0f, 100.0f);
  const float available_height = std::max(display.y - 48.0f, 100.0f);
  const float scale = std::min(available_width / kContactLabAreaWidthM,
                               available_height / kContactLabAreaHeightM);
  return {{left + (available_width - scale * kContactLabAreaWidthM) * 0.5f,
           24.0f + (available_height - scale * kContactLabAreaHeightM) * 0.5f},
          scale};
}

ImVec2 WorldToScreen(const SceneTransform& transform, Vec2 position) {
  return {transform.origin.x + position.x * transform.pixels_per_meter,
          transform.origin.y + position.y * transform.pixels_per_meter};
}

void DrawRectangleBody(ImDrawList* draw_list, const SceneTransform& transform,
                       const Rectangle& rectangle) {
  const std::array<Vec2, 4> vertices = GetVertices(rectangle);
  std::array<ImVec2, 4> screen_vertices{};
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    screen_vertices[i] = WorldToScreen(transform, vertices[i]);
  }
  draw_list->AddConvexPolyFilled(screen_vertices.data(), 4,
                                 IM_COL32(85, 95, 110, 255));
  draw_list->AddPolyline(screen_vertices.data(), 4,
                         IM_COL32(165, 180, 200, 255), ImDrawFlags_Closed,
                         2.0f);
}

void DrawCircleBody(ImDrawList* draw_list, const SceneTransform& transform,
                    const Circle& circle, ImU32 fill, const char* name) {
  const ImVec2 center = WorldToScreen(transform, circle.position);
  const float radius = circle.radius * transform.pixels_per_meter;
  draw_list->AddCircleFilled(center, radius, fill, 48);
  draw_list->AddCircle(center, radius, IM_COL32(215, 230, 245, 255), 48, 2.0f);
  const ImVec2 spoke{center.x + std::cos(circle.angle) * radius * 0.8f,
                     center.y + std::sin(circle.angle) * radius * 0.8f};
  draw_list->AddLine(center, spoke, IM_COL32(245, 245, 245, 255), 3.0f);
  ui::DrawArrowFrom(draw_list, center, {circle.velocity.x, circle.velocity.y},
                    42.0f, IM_COL32(255, 125, 105, 235));

  char label[112];
  std::snprintf(label, sizeof(label), "%s | m=%.3g kg | v=(%+.2f,%+.2f) m/s",
                name, circle.mass, circle.velocity.x, circle.velocity.y);
  draw_list->AddText({center.x + radius + 8.0f, center.y - radius - 4.0f},
                     IM_COL32(190, 225, 255, 255), label);
}

void DrawContactLabScene(const ContactLabConfig& config,
                         const ContactLabState& state) {
  const SceneTransform transform = GetSceneTransform();
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 minimum = transform.origin;
  const ImVec2 maximum =
      WorldToScreen(transform, {kContactLabAreaWidthM, kContactLabAreaHeightM});
  draw_list->AddRect(minimum, maximum, IM_COL32(75, 90, 110, 255), 0.0f, 0,
                     2.0f);
  for (const Rectangle& rectangle : state.rectangles) {
    DrawRectangleBody(draw_list, transform, rectangle);
  }
  if (!state.circles.empty()) {
    DrawCircleBody(draw_list, transform, state.circles[0],
                   IM_COL32(85, 165, 245, 255), "A");
  }
  if (state.circles.size() > 1) {
    DrawCircleBody(draw_list, transform, state.circles[1],
                   IM_COL32(255, 150, 95, 255), "B");
  }

  const ContactLabDerived derived = CalculateContactLabDerived(config, state);
  char summary[160];
  if (config.mode == ContactLabMode::kRollingContact) {
    std::snprintf(
        summary, sizeof(summary), "surface slip=%+.5f m/s | %s",
        derived.rolling_slip_m_s,
        std::abs(derived.rolling_slip_m_s) <= kContactLabRollingSlipToleranceMps
            ? "Rolling"
            : "Sliding");
  } else {
    std::snprintf(summary, sizeof(summary), "p=(%+.4f,%+.4f) kg*m/s | K=%.6f J",
                  derived.total_linear_momentum_kg_m_s.x,
                  derived.total_linear_momentum_kg_m_s.y,
                  derived.total_kinetic_energy_j);
  }
  draw_list->AddText({minimum.x, maximum.y + 8.0f},
                     IM_COL32(245, 205, 115, 255), summary);
}

double RelativeError(double value, double reference) {
  return std::abs(value - reference) / std::max(std::abs(reference), 0.000001);
}

bool DrawMonitor(const ContactLabConfig& config,
                 const ContactLabState& current_state,
                 const ContactLabDerived& initial_derived,
                 const std::vector<ContactLabState>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& runtime_error,
                 ContactLabState* displayed_state) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({430.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("ContactLab monitor", nullptr, kWindowFlags);

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
      current_state.time_seconds, static_cast<double>(kContactLabPhysicsStep));
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_changed = ImGui::SliderScalar(
      "##contact_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputDouble("##contact_history_input", inspect_time,
                                        0.0, 0.0, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const ContactLabState* inspected =
      FindContactLabState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  *displayed_state = *inspected;
  const ContactLabDerived derived =
      CalculateContactLabDerived(config, *inspected);

  ImGui::BeginChild("##contact_telemetry", {0.0f, -48.0f}, false);
  for (std::size_t i = 0; i < inspected->circles.size(); ++i) {
    const Circle& circle = inspected->circles[i];
    const ContactLabCircleConfig& circle_config =
        i == 0 ? config.circle_a : config.circle_b;
    ImGui::Text("Circle %c: pos=(%+.4f,%+.4f) m", 'A' + static_cast<int>(i),
                circle.position.x, circle.position.y);
    ImGui::Text("  m=%.4g kg | r=%.4g m | %s | %s", circle_config.mass_kg,
                circle_config.radius_m,
                circle_config.is_static ? "static" : "dynamic",
                circle_config.inertia_model == CircleInertiaModel::kSolidDisk
                    ? "solid disk"
                    : "hoop");
    ImGui::Text("  v=(%+.5f,%+.5f) m/s | omega=%+.5f rad/s", circle.velocity.x,
                circle.velocity.y, circle.angular_velocity);
    ImGui::Text("  theta=%+.3f deg | I=%.6f kg*m^2",
                circle.angle * kRadiansToDegrees, GetMomentOfInertia(circle));
  }

  ImGui::Spacing();
  const bool conservation_reference =
      config.mode == ContactLabMode::kElasticImpact &&
      !config.circle_b.is_static;
  const char* telemetry_title = "Conservation telemetry";
  if (config.mode == ContactLabMode::kRollingContact) {
    telemetry_title = "System telemetry (external gravity/friction)";
  } else if (config.circle_b.is_static) {
    telemetry_title = "System telemetry (static target impulse)";
  }
  ImGui::TextUnformatted(telemetry_title);
  ImGui::Separator();
  ImGui::Text("p=(%+.7f,%+.7f) kg*m/s", derived.total_linear_momentum_kg_m_s.x,
              derived.total_linear_momentum_kg_m_s.y);
  if (conservation_reference) {
    const double momentum_delta = std::hypot(
        static_cast<double>(derived.total_linear_momentum_kg_m_s.x -
                            initial_derived.total_linear_momentum_kg_m_s.x),
        static_cast<double>(derived.total_linear_momentum_kg_m_s.y -
                            initial_derived.total_linear_momentum_kg_m_s.y));
    const double momentum_scale = std::max(
        std::hypot(
            static_cast<double>(initial_derived.total_linear_momentum_kg_m_s.x),
            static_cast<double>(
                initial_derived.total_linear_momentum_kg_m_s.y)),
        0.000001);
    ImGui::Text("momentum relative drift: %.6g %%",
                momentum_delta / momentum_scale * 100.0);
  }
  ImGui::Text("L orbital=%+.8f | spin=%+.8f kg*m^2/s",
              derived.orbital_angular_momentum_kg_m2_s,
              derived.spin_angular_momentum_kg_m2_s);
  if (conservation_reference) {
    ImGui::Text("L total=%+.8f | drift=%.6g %%",
                derived.total_angular_momentum_kg_m2_s,
                RelativeError(derived.total_angular_momentum_kg_m2_s,
                              initial_derived.total_angular_momentum_kg_m2_s) *
                    100.0);
  } else {
    ImGui::Text("L total=%+.8f kg*m^2/s",
                derived.total_angular_momentum_kg_m2_s);
  }
  ImGui::Text("K translational=%.8f J", derived.translational_kinetic_energy_j);
  ImGui::Text("K rotational=%.8f J", derived.rotational_kinetic_energy_j);
  if (conservation_reference) {
    ImGui::Text("K total=%.8f J | drift=%.6g %%",
                derived.total_kinetic_energy_j,
                RelativeError(derived.total_kinetic_energy_j,
                              initial_derived.total_kinetic_energy_j) *
                    100.0);
  } else {
    ImGui::Text("K total=%.8f J", derived.total_kinetic_energy_j);
  }

  ImGui::Spacing();
  const bool one_dimensional_reference =
      config.mode == ContactLabMode::kElasticImpact &&
      !config.circle_b.is_static &&
      std::abs(config.circle_a.initial_position_m.y -
               config.circle_b.initial_position_m.y) <= 0.00001f &&
      std::abs(config.circle_a.initial_velocity_m_s.y) <= 0.00001f &&
      std::abs(config.circle_b.initial_velocity_m_s.y) <= 0.00001f &&
      (config.circle_b.initial_position_m.x -
       config.circle_a.initial_position_m.x) *
              (config.circle_a.initial_velocity_m_s.x -
               config.circle_b.initial_velocity_m_s.x) >
          0.0f;
  if (one_dimensional_reference) {
    const double mass_a = config.circle_a.mass_kg;
    const double mass_b = config.circle_b.mass_kg;
    const double speed_a = config.circle_a.initial_velocity_m_s.x;
    const double speed_b = config.circle_b.initial_velocity_m_s.x;
    const double restitution =
        std::abs(speed_a - speed_b) <
                static_cast<double>(kContactLabRestitutionSpeedThresholdMps)
            ? 0.0
            : std::max(config.circle_a.material.restitution,
                       config.circle_b.material.restitution);
    const double expected_a = (mass_a * speed_a + mass_b * speed_b -
                               mass_b * restitution * (speed_a - speed_b)) /
                              (mass_a + mass_b);
    const double expected_b = (mass_a * speed_a + mass_b * speed_b +
                               mass_a * restitution * (speed_a - speed_b)) /
                              (mass_a + mass_b);
    ImGui::TextUnformatted("One-dimensional impact reference");
    ImGui::Separator();
    ImGui::Text("expected post-impact vx: A=%+.6f, B=%+.6f m/s", expected_a,
                expected_b);
    ImGui::Text("current vx error: A=%+.6f, B=%+.6f m/s",
                inspected->circles[0].velocity.x - expected_a,
                inspected->circles[1].velocity.x - expected_b);
    ImGui::Text(
        "relative vx error: A=%.6g %%, B=%.6g %%",
        RelativeError(inspected->circles[0].velocity.x, expected_a) * 100.0,
        RelativeError(inspected->circles[1].velocity.x, expected_b) * 100.0);
  } else if (config.mode == ContactLabMode::kRollingContact) {
    ImGui::TextUnformatted("Rolling contact");
    ImGui::Separator();
    ImGui::Text("surface slip v_t-R*omega: %+.7f m/s",
                derived.rolling_slip_m_s);
    ImGui::Text(
        "slip classification: %s",
        std::abs(derived.rolling_slip_m_s) <= kContactLabRollingSlipToleranceMps
            ? "Rolling"
            : "Sliding");
  }

  if (!runtime_error.empty()) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error.c_str());
  }
  ImGui::EndChild();

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

}  // namespace

SimulationResult RunContactLabSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  ContactLabConfig config = MakeElasticImpactConfig();
  ContactLabState state = MakeInitialContactLabState(config);
  ContactLabDerived initial_derived = CalculateContactLabDerived(config, state);
  std::vector<ContactLabState> history;
  bool simulation_started = false;
  bool paused = false;
  bool follow_live = true;
  double inspect_time = 0.0;
  std::string runtime_error;
  FixedStepClock clock(static_cast<double>(SDL_GetPerformanceFrequency()),
                       SDL_GetPerformanceCounter());

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
                 runtime_error.empty()) {
        paused = !paused;
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
          state = MakeInitialContactLabState(config);
          initial_derived = CalculateContactLabDerived(config, state);
          history.clear();
          const bool recorded = AppendHistorySample(
              &history, state, &ContactLabState::time_seconds);
          simulation_started = true;
          paused = !recorded;
          follow_live = true;
          inspect_time = 0.0;
          runtime_error =
              recorded ? "" : "ContactLab produced an invalid history time.";
          clock.Reset(current_time);
        } catch (const std::exception& error) {
          runtime_error = error.what();
        }
      }
    } else if (simulation_started) {
      if (!return_requested && !paused && runtime_error.empty()) {
        clock.Accumulate(current_time, kMaximumFrameTime);
        while (clock.HasStep(kContactLabPhysicsStep)) {
          try {
            if (!StepContactLab(config, kContactLabPhysicsStep, &state)) {
              const char* state_error = GetContactLabStateError(config, state);
              runtime_error =
                  state_error != nullptr
                      ? state_error
                      : "ContactLab rejected an unstable fixed step.";
              paused = true;
              clock.DiscardPendingSteps();
              break;
            }
          } catch (const std::exception& error) {
            runtime_error = error.what();
            paused = true;
            clock.DiscardPendingSteps();
            break;
          }
          if (!AppendHistorySample(&history, state,
                                   &ContactLabState::time_seconds)) {
            runtime_error = "ContactLab history time stopped increasing.";
            paused = true;
            clock.DiscardPendingSteps();
            break;
          }
          clock.ConsumeStep(kContactLabPhysicsStep);
        }
      } else {
        clock.Reset(current_time);
      }

      ContactLabState displayed_state = state;
      const bool stop = !return_requested &&
                        DrawMonitor(config, state, initial_derived, history,
                                    &paused, &inspect_time, &follow_live,
                                    runtime_error, &displayed_state);
      DrawContactLabScene(config, displayed_state);
      if (stop) {
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
