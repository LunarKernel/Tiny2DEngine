#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include "tiny2d_engine.h"

namespace {

constexpr int kAreaWidth = 1200;
constexpr int kAreaHeight = 800;
constexpr float kUnitLength = 32.0f;
constexpr float kDegreesToRadians = 0.01745329252f;
constexpr float kSpringAnchorX = 16.0f;
constexpr float kSpringRestX = 360.0f;
constexpr float kSpringAmplitude = 10.0f;
constexpr int kSpringCoilCount = 10;
constexpr float kJunctionHysteresis = 0.05f;
constexpr float kAirborneDistanceTolerance = 0.05f;
constexpr float kAirborneSpeedTolerance = 0.01f;
constexpr float kAirborneAccelerationTolerance = 0.0001f;
constexpr float kPhysicsStep = 1.0f / 120.0f;
constexpr double kMaxFrameTime = 0.25;
constexpr std::array<int, 6> kRectangleIndices = {0, 1, 2, 0, 2, 3};

struct BodyConfig {
  float surface_x;
  float mass;
  float downhill_speed;
  float length_units;
  bool charged{};
  float charge{1.0f};
};

struct SimulationConfig {
  float friction{0.4f};
  float restitution{0.95f};
  bool electric_field_enabled{};
  float electric_field_strength{50.0f};
  float electric_field_angle_degrees{};
  bool ramp_enabled{true};
  float ramp_angle_degrees{30.0f};
  float ramp_width_percent{50.0f};
  bool spring_enabled{true};
  float spring_stiffness{8.0f};
  BodyConfig body_a{1080.0f, 1.0f, 280.0f, 1.0f, true, 1.0f};
  BodyConfig body_b{720.0f, 4.0f, 0.0f, 1.0f, false, 1.0f};
};

tiny2d::Vec2 GetElectricField(const SimulationConfig& config) {
  if (!config.electric_field_enabled) {
    return {};
  }
  const float angle = config.electric_field_angle_degrees * kDegreesToRadians;
  return {config.electric_field_strength * std::cos(angle),
          -config.electric_field_strength * std::sin(angle)};
}

float GetRampAngle(const SimulationConfig& config) {
  return -config.ramp_angle_degrees * kDegreesToRadians;
}

float GetRampBottomX(const SimulationConfig& config) {
  return static_cast<float>(kAreaWidth) *
         (1.0f - config.ramp_width_percent / 100.0f);
}

float GetBodyWidth(const BodyConfig& config) {
  return config.length_units * kUnitLength;
}

float GetMaximumBodyLengthUnits(const SimulationConfig& config) {
  const float ramp_bottom_x = GetRampBottomX(config);
  const float available_length =
      config.ramp_enabled
          ? std::min((static_cast<float>(kAreaWidth) - ramp_bottom_x) /
                         std::cos(GetRampAngle(config)),
                     ramp_bottom_x)
          : static_cast<float>(kAreaWidth);
  return available_length / kUnitLength;
}

tiny2d::Rectangle CreateBody(const BodyConfig& body_config,
                             const SimulationConfig& simulation_config) {
  const float width = GetBodyWidth(body_config);
  const float half_height = kUnitLength * 0.5f;
  if (!simulation_config.ramp_enabled) {
    return {
        body_config.mass,
        {body_config.surface_x, static_cast<float>(kAreaHeight) - half_height},
        {-body_config.downhill_speed, 0.0f},
        0.0f,
        0.0f,
        width,
        kUnitLength,
        true,
        body_config.charged ? body_config.charge : 0.0f};
  }

  const float ramp_angle = GetRampAngle(simulation_config);
  const float surface_y =
      static_cast<float>(kAreaHeight) +
      (body_config.surface_x - GetRampBottomX(simulation_config)) *
          std::tan(ramp_angle);
  const tiny2d::Vec2 position{
      body_config.surface_x + std::sin(ramp_angle) * half_height,
      surface_y - std::cos(ramp_angle) * half_height};
  const tiny2d::Vec2 velocity{
      -std::cos(ramp_angle) * body_config.downhill_speed,
      -std::sin(ramp_angle) * body_config.downhill_speed};
  return {body_config.mass,
          position,
          velocity,
          ramp_angle,
          0.0f,
          width,
          kUnitLength,
          true,
          body_config.charged ? body_config.charge : 0.0f};
}

const char* GetConfigError(const SimulationConfig& config) {
  const std::array<float, 17> values = {
      config.friction,
      config.restitution,
      config.electric_field_strength,
      config.electric_field_angle_degrees,
      config.ramp_angle_degrees,
      config.ramp_width_percent,
      config.spring_stiffness,
      config.body_a.surface_x,
      config.body_a.mass,
      config.body_a.downhill_speed,
      config.body_a.length_units,
      config.body_a.charge,
      config.body_b.surface_x,
      config.body_b.mass,
      config.body_b.downhill_speed,
      config.body_b.length_units,
      config.body_b.charge,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "Every value must be a valid number.";
  }
  if (tiny2d::IsColliding(CreateBody(config.body_a, config),
                          CreateBody(config.body_b, config))) {
    return "The bodies overlap. Move them farther apart before starting.";
  }
  return nullptr;
}

void DrawBodyControls(const char* title, BodyConfig* body_config,
                      const SimulationConfig& simulation_config) {
  ImGui::PushID(title);
  ImGui::TextUnformatted(title);
  // V8 exposes unit blocks only. Keep length_units and the rectangle physics
  // so the plank control can be restored in a later version.
  body_config->length_units = 1.0f;
  ImGui::TextDisabled("V8 uses a 1 x 1 block.");

  const float half_width = GetBodyWidth(*body_config) * 0.5f;
  const float minimum_x =
      simulation_config.ramp_enabled
          ? GetRampBottomX(simulation_config) +
                std::cos(GetRampAngle(simulation_config)) * half_width
          : half_width;
  const float maximum_x =
      std::max(minimum_x,
               static_cast<float>(kAreaWidth) -
                   (simulation_config.ramp_enabled
                        ? std::cos(GetRampAngle(simulation_config)) * half_width
                        : half_width));
  body_config->surface_x =
      std::clamp(body_config->surface_x, minimum_x, maximum_x);
  ImGui::TextDisabled(
      simulation_config.ramp_enabled
          ? "Position: right is higher. Speed: positive is downhill."
          : "Position is on the floor. Speed: positive moves left.");
  ImGui::SliderFloat(
      simulation_config.ramp_enabled ? "Ramp position" : "Floor position",
      &body_config->surface_x, minimum_x, maximum_x, "%.1f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat(
      "Mass", &body_config->mass, 0.01f, 1000.0f, "%.2f",
      ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic);
  ImGui::Checkbox("Charged", &body_config->charged);
  ImGui::BeginDisabled(!body_config->charged);
  ImGui::SliderFloat("Charge q", &body_config->charge, -10.0f, 10.0f, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);
  ImGui::EndDisabled();
  ImGui::SliderFloat("Initial speed", &body_config->downhill_speed, -1000.0f,
                     1000.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::PopID();
}

bool DrawSetupScreen(SimulationConfig* config) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("Tiny2D setup", nullptr, kWindowFlags);

  ImGui::TextColored({1.0f, 0.55f, 0.3f, 1.0f}, "Tiny2D Physics Lab");
  ImGui::TextWrapped(
      "Adjust the experiment, then press Start simulation. Restore defaults "
      "returns every value to a safe preset.");
  ImGui::TextDisabled(
      "Drag a slider, or Ctrl+click its value to type an exact number.");
  ImGui::Spacing();

  ImGui::TextUnformatted("System");
  ImGui::Separator();
  ImGui::TextDisabled(
      "Friction: 0 is slippery. Bounciness: 1 keeps more collision energy.");
  ImGui::SliderFloat("Surface friction", &config->friction, 0.0f, 5.0f, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat("Collision bounciness", &config->restitution, 0.0f, 1.0f,
                     "%.2f", ImGuiSliderFlags_AlwaysClamp);

  ImGui::Checkbox("Enable electric field", &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  ImGui::TextDisabled("Direction: 0 degrees is right; 90 degrees is up.");
  ImGui::SliderFloat("Electric field strength",
                     &config->electric_field_strength, 0.0f, 500.0f, "%.1f",
                     ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat("Electric field angle (degrees)",
                     &config->electric_field_angle_degrees, -180.0f, 180.0f,
                     "%.1f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::EndDisabled();

  ImGui::Checkbox("Enable ramp", &config->ramp_enabled);
  ImGui::BeginDisabled(!config->ramp_enabled);
  ImGui::SliderFloat("Ramp angle (degrees)", &config->ramp_angle_degrees, 5.0f,
                     45.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat("Ramp width (%)", &config->ramp_width_percent, 25.0f,
                     60.0f, "%.1f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::EndDisabled();

  ImGui::Checkbox("Enable left spring", &config->spring_enabled);
  ImGui::BeginDisabled(!config->spring_enabled);
  ImGui::SliderFloat(
      "Spring strength", &config->spring_stiffness, 0.1f, 100.0f, "%.1f",
      ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic);
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::Separator();
  DrawBodyControls("Body A", &config->body_a, *config);
  ImGui::Spacing();
  ImGui::Separator();
  DrawBodyControls("Body B", &config->body_b, *config);
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

#ifndef NDEBUG
void CheckConfigValidation() {
  SimulationConfig config;
  assert(GetConfigError(config) == nullptr);
  assert(GetBodyWidth(config.body_a) == kUnitLength);
  assert(GetBodyWidth(config.body_b) == kUnitLength);
  assert(GetMaximumBodyLengthUnits(config) * kUnitLength <=
         GetRampBottomX(config) + 0.001f);
  config.ramp_enabled = false;
  assert(GetConfigError(config) == nullptr);
  assert(std::abs(GetMaximumBodyLengthUnits(config) * kUnitLength -
                  static_cast<float>(kAreaWidth)) <= 0.001f);
  config.body_b.surface_x = config.body_a.surface_x;
  assert(GetConfigError(config) != nullptr);
}
#endif

void ShowError(const char* message) {
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tiny2D Engine", message,
                           nullptr);
}

const char* GetAirborneError(const std::vector<tiny2d::Rectangle>& bodies,
                             const SimulationConfig& config) {
  if (!config.electric_field_enabled) {
    return nullptr;
  }

  const float ramp_angle = GetRampAngle(config);
  const tiny2d::Vec2 electric_field = GetElectricField(config);
  const std::size_t dynamic_body_count =
      std::min<std::size_t>(2, bodies.size());
  for (std::size_t i = 0; i < dynamic_body_count; ++i) {
    const tiny2d::Rectangle& body = bodies[i];
    tiny2d::Vec2 outward_normal{0.0f, -1.0f};
    tiny2d::Vec2 surface_position{body.position.x,
                                  static_cast<float>(kAreaHeight)};
    if (config.ramp_enabled && std::abs(body.angle - ramp_angle) <= 0.001f) {
      outward_normal = {std::sin(ramp_angle), -std::cos(ramp_angle)};
      surface_position = {GetRampBottomX(config),
                          static_cast<float>(kAreaHeight)};
    }

    const tiny2d::Vec2 target_position{
        surface_position.x + outward_normal.x * body.height * 0.5f,
        surface_position.y + outward_normal.y * body.height * 0.5f};
    const float clearance =
        (body.position.x - target_position.x) * outward_normal.x +
        (body.position.y - target_position.y) * outward_normal.y;
    const float outward_speed =
        body.velocity.x * outward_normal.x + body.velocity.y * outward_normal.y;
    const tiny2d::Vec2 acceleration =
        tiny2d::GetLinearAcceleration(body, electric_field);
    const float outward_acceleration =
        acceleration.x * outward_normal.x + acceleration.y * outward_normal.y;
    // A one-sided surface can push a body away, but cannot pull it back.
    if (clearance > kAirborneDistanceTolerance ||
        outward_speed > kAirborneSpeedTolerance ||
        outward_acceleration > kAirborneAccelerationTolerance) {
      return i == 0 ? "Body A cannot remain in contact with its supporting "
                      "surface. Simulation stopped."
                    : "Body B cannot remain in contact with its supporting "
                      "surface. Simulation stopped.";
    }
  }
  return nullptr;
}

#ifndef NDEBUG
void CheckAirborneDetection() {
  SimulationConfig config;
  config.electric_field_enabled = true;
  config.electric_field_angle_degrees = 90.0f;
  config.electric_field_strength = 50.0f;
  std::vector<tiny2d::Rectangle> bodies = {
      CreateBody(config.body_a, config),
      CreateBody(config.body_b, config),
  };
  assert(GetAirborneError(bodies, config) == nullptr);

  const float ramp_angle = GetRampAngle(config);
  const tiny2d::Vec2 outward_normal{std::sin(ramp_angle),
                                    -std::cos(ramp_angle)};
  bodies[0].position.x += outward_normal.x * 0.1f;
  bodies[0].position.y += outward_normal.y * 0.1f;
  assert(GetAirborneError(bodies, config) != nullptr);

  bodies[0] = CreateBody(config.body_a, config);
  bodies[0].velocity.x += outward_normal.x * 0.1f;
  bodies[0].velocity.y += outward_normal.y * 0.1f;
  assert(GetAirborneError(bodies, config) != nullptr);

  bodies[0] = CreateBody(config.body_a, config);
  config.electric_field_strength = 200.0f;
  assert(GetAirborneError(bodies, config) != nullptr);
}
#endif

tiny2d::Rectangle CreateRamp(const SimulationConfig& config) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float run = static_cast<float>(kAreaWidth) - ramp_bottom_x;
  const float ramp_length = run / std::cos(ramp_angle);
  const float ramp_height = static_cast<float>(kAreaHeight);
  const tiny2d::Vec2 top_midpoint{
      ramp_bottom_x + run * 0.5f,
      static_cast<float>(kAreaHeight) + run * 0.5f * std::tan(ramp_angle)};
  const tiny2d::Vec2 center{
      top_midpoint.x - std::sin(ramp_angle) * ramp_height * 0.5f,
      top_midpoint.y + std::cos(ramp_angle) * ramp_height * 0.5f};
  return {0.0f, center, {}, ramp_angle, 0.0f, ramp_length, ramp_height, true};
}

void TransitionBodiesToFloor(std::vector<tiny2d::Rectangle>& bodies,
                             const SimulationConfig& config, float delta_time) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);

  for (tiny2d::Rectangle& body : bodies) {
    if (body.mass <= 0.0f || !body.fixed_rotation ||
        std::abs(body.angle - ramp_angle) > 0.001f || body.velocity.x >= 0.0f) {
      continue;
    }

    const float half_width = body.width * 0.5f;
    const float half_height = body.height * 0.5f;
    const float front_corner_x =
        body.position.x - half_width * cosine - half_height * sine;
    const float predicted_corner_x =
        front_corner_x + body.velocity.x * delta_time;
    if (predicted_corner_x > ramp_bottom_x - kJunctionHysteresis) {
      continue;
    }

    const float speed_squared =
        body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
    const float speed = std::sqrt(speed_squared);
    const float time_to_junction = std::clamp(
        (front_corner_x - ramp_bottom_x) / -body.velocity.x, 0.0f, delta_time);
    // Update() will still integrate the full step. This pre-offset makes its
    // final position include only the horizontal motion after the junction.
    body.position = {ramp_bottom_x - half_width + speed * time_to_junction,
                     static_cast<float>(kAreaHeight) - half_height};
    body.velocity = {-speed, 0.0f};
    body.angle = 0.0f;
    body.angular_velocity = 0.0f;

    const float transitioned_speed_squared =
        body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
    const float expected_end_x =
        ramp_bottom_x - half_width - speed * (delta_time - time_to_junction);
    assert(std::abs(transitioned_speed_squared - speed_squared) <=
           std::max(1.0f, speed_squared) * 0.00001f);
    assert(std::abs(body.position.x + body.velocity.x * delta_time -
                    expected_end_x) <= 0.0001f);
  }
}

void TransitionBodiesToRamp(std::vector<tiny2d::Rectangle>& bodies,
                            const SimulationConfig& config, float delta_time) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float area_height = static_cast<float>(kAreaHeight);
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);

  for (tiny2d::Rectangle& body : bodies) {
    if (body.mass <= 0.0f || !body.fixed_rotation ||
        std::abs(body.angle) > 0.001f || body.velocity.x <= 0.0f) {
      continue;
    }

    const float half_width = body.width * 0.5f;
    const float half_height = body.height * 0.5f;
    const float right_edge = body.position.x + half_width;
    const float predicted_edge = right_edge + body.velocity.x * delta_time;
    if (predicted_edge < ramp_bottom_x + kJunctionHysteresis) {
      continue;
    }

    const float speed_squared =
        body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
    const float speed = std::sqrt(speed_squared);
    const float time_to_junction = std::clamp(
        (ramp_bottom_x - right_edge) / body.velocity.x, 0.0f, delta_time);
    const tiny2d::Vec2 junction_position{
        ramp_bottom_x + half_width * cosine + half_height * sine,
        area_height + half_width * sine - half_height * cosine};
    body.velocity = {cosine * speed, sine * speed};
    body.position = {junction_position.x - body.velocity.x * time_to_junction,
                     junction_position.y - body.velocity.y * time_to_junction};
    body.angle = ramp_angle;
    body.angular_velocity = 0.0f;

    const float transitioned_speed_squared =
        body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
    const float expected_end_x =
        junction_position.x + body.velocity.x * (delta_time - time_to_junction);
    assert(std::abs(transitioned_speed_squared - speed_squared) <=
           std::max(1.0f, speed_squared) * 0.00001f);
    assert(std::abs(body.position.x + body.velocity.x * delta_time -
                    expected_end_x) <= 0.0001f);
  }
}

#ifndef NDEBUG
void CheckJunctionHysteresis() {
  const SimulationConfig config;
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float half_width = kUnitLength * 2.0f;
  const float half_height = kUnitLength * 0.5f;
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);
  const tiny2d::Vec2 ramp_junction{
      ramp_bottom_x + half_width * cosine + half_height * sine,
      static_cast<float>(kAreaHeight) + half_width * sine -
          half_height * cosine};

  std::vector<tiny2d::Rectangle> bodies = {
      {1.0f,
       ramp_junction,
       {-cosine, -sine},
       ramp_angle,
       0.0f,
       half_width * 2.0f,
       half_height * 2.0f,
       true},
  };
  TransitionBodiesToFloor(bodies, config, kPhysicsStep);
  assert(std::abs(bodies[0].angle - ramp_angle) <= 0.001f);

  bodies[0].velocity = {-10.0f * cosine, -10.0f * sine};
  TransitionBodiesToFloor(bodies, config, kPhysicsStep);
  assert(std::abs(bodies[0].angle) <= 0.001f);

  bodies[0].velocity = {1.0f, 0.0f};
  TransitionBodiesToRamp(bodies, config, kPhysicsStep);
  assert(std::abs(bodies[0].angle) <= 0.001f);

  bodies[0].velocity = {10.0f, 0.0f};
  TransitionBodiesToRamp(bodies, config, kPhysicsStep);
  assert(std::abs(bodies[0].angle - ramp_angle) <= 0.001f);
}
#endif

void ConstrainBodiesToSurfaces(std::vector<tiny2d::Rectangle>& bodies,
                               const SimulationConfig& config) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);

  // ponytail: This scene uses an ideal guide constraint. Add a general
  // constraint solver only when other surface shapes need the same behavior.
  for (tiny2d::Rectangle& body : bodies) {
    if (body.mass <= 0.0f || !body.fixed_rotation) {
      continue;
    }

    tiny2d::Vec2 normal{};
    tiny2d::Vec2 surface_position{};
    if (config.ramp_enabled && std::abs(body.angle - ramp_angle) <= 0.001f) {
      normal = {std::sin(ramp_angle), -std::cos(ramp_angle)};
      surface_position = {ramp_bottom_x, static_cast<float>(kAreaHeight)};
    } else if (std::abs(body.angle) <= 0.001f) {
      normal = {0.0f, -1.0f};
      surface_position = {body.position.x, static_cast<float>(kAreaHeight)};
    } else {
      continue;
    }

    const float half_height = body.height * 0.5f;
    const tiny2d::Vec2 target_position{
        surface_position.x + normal.x * half_height,
        surface_position.y + normal.y * half_height};
    const float position_error =
        (body.position.x - target_position.x) * normal.x +
        (body.position.y - target_position.y) * normal.y;
    body.position.x -= normal.x * position_error;
    body.position.y -= normal.y * position_error;

    const float normal_velocity =
        body.velocity.x * normal.x + body.velocity.y * normal.y;
    body.velocity.x -= normal.x * normal_velocity;
    body.velocity.y -= normal.y * normal_velocity;

    const float remaining_position_error =
        (body.position.x - target_position.x) * normal.x +
        (body.position.y - target_position.y) * normal.y;
    const float remaining_normal_velocity =
        body.velocity.x * normal.x + body.velocity.y * normal.y;
    assert(std::abs(remaining_position_error) <= 0.0001f);
    assert(std::abs(remaining_normal_velocity) <= 0.0001f);
  }
}

bool IsOnFloor(const tiny2d::Rectangle& body) {
  return body.mass > 0.0f && body.fixed_rotation &&
         std::abs(body.angle) <= 0.001f;
}

std::size_t FindSpringBodyIndex(const std::vector<tiny2d::Rectangle>& bodies) {
  std::size_t body_index = bodies.size();
  float spring_end_x = kSpringRestX;
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    if (!IsOnFloor(bodies[i])) {
      continue;
    }
    const float left_edge = bodies[i].position.x - bodies[i].width * 0.5f;
    if (left_edge < spring_end_x) {
      spring_end_x = left_edge;
      body_index = i;
    }
  }
  return body_index;
}

void ApplySpringForce(std::vector<tiny2d::Rectangle>& bodies, float stiffness,
                      float delta_time) {
  const std::size_t body_index = FindSpringBodyIndex(bodies);
  if (body_index == bodies.size()) {
    return;
  }

  tiny2d::Rectangle& body = bodies[body_index];
  const float left_edge = body.position.x - body.width * 0.5f;
  const float compression = kSpringRestX - left_edge;
  const float velocity_change =
      stiffness * compression / body.mass * delta_time;
  const float previous_velocity_x = body.velocity.x;
  body.velocity.x += velocity_change;
  assert(body.velocity.x >= previous_velocity_x);
}

void DrawSpring(SDL_Renderer* renderer,
                const std::vector<tiny2d::Rectangle>& bodies) {
  float spring_end_x = kSpringRestX;
  const std::size_t body_index = FindSpringBodyIndex(bodies);
  if (body_index != bodies.size()) {
    spring_end_x = std::clamp(
        bodies[body_index].position.x - bodies[body_index].width * 0.5f,
        kSpringAnchorX, kSpringRestX);
  }

  constexpr float kSpringY =
      static_cast<float>(kAreaHeight) - kUnitLength * 0.5f;
  std::array<SDL_FPoint, kSpringCoilCount + 2> points{};
  points.front() = {kSpringAnchorX, kSpringY};
  for (int i = 1; i <= kSpringCoilCount; ++i) {
    const float fraction =
        static_cast<float>(i) / static_cast<float>(kSpringCoilCount + 1);
    points[i] = {
        kSpringAnchorX + (spring_end_x - kSpringAnchorX) * fraction,
        kSpringY + (i % 2 == 0 ? kSpringAmplitude : -kSpringAmplitude)};
  }
  points.back() = {spring_end_x, kSpringY};

  SDL_SetRenderDrawColor(renderer, 245, 200, 70, 255);
  SDL_RenderDrawLinesF(renderer, points.data(),
                       static_cast<int>(points.size()));
  SDL_RenderDrawLineF(renderer, kSpringAnchorX, kSpringY - kUnitLength * 0.5f,
                      kSpringAnchorX, kSpringY + kUnitLength * 0.5f);
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

}  // namespace

int main(int argc, char* argv[]) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
    ShowError("SDL could not start.");
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow("Tiny2D Engine", SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED, kAreaWidth,
                                        kAreaHeight, SDL_WINDOW_SHOWN);

  if (!window) {
    ShowError("The application window could not be created.");
    SDL_Quit();
    return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if (!renderer) {
    ShowError("The graphics renderer could not be created.");
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = nullptr;
#ifdef _WIN32
  // ponytail: Use the native UI font; bundle one only when identical
  // cross-platform typography becomes a requirement.
  if (io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 16.0f) ==
      nullptr) {
    io.Fonts->AddFontDefault();
  }
#else
  io.Fonts->AddFontDefault();
#endif
  ImGui::StyleColorsDark();
  ImGui::GetStyle().FrameRounding = 5.0f;
  ImGui::GetStyle().GrabRounding = 5.0f;

  if (!ImGui_ImplSDL2_InitForSDLRenderer(window, renderer)) {
    ShowError("The settings interface could not be initialized.");
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  if (!ImGui_ImplSDLRenderer2_Init(renderer)) {
    ShowError("The settings interface renderer could not be initialized.");
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

#ifndef NDEBUG
  CheckConfigValidation();
  CheckAirborneDetection();
  CheckJunctionHysteresis();
#endif

  SimulationConfig config;
  std::vector<tiny2d::Rectangle> bodies;
  bool simulation_started = false;

  bool running = true;
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();
  double accumulated_time = 0.0;

  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }
    if (!running) {
      break;
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint64 current_time = SDL_GetPerformanceCounter();
    if (!simulation_started) {
      previous_time = current_time;
      accumulated_time = 0.0;
      if (DrawSetupScreen(&config)) {
        assert(GetConfigError(config) == nullptr);
        bodies = {
            CreateBody(config.body_a, config),
            CreateBody(config.body_b, config),
        };
        if (config.ramp_enabled) {
          bodies.push_back(CreateRamp(config));
        }
        assert(bodies.size() == (config.ramp_enabled ? 3 : 2));
        simulation_started = true;
      }
    } else {
      const double frame_time = std::min(
          static_cast<double>(current_time - previous_time) / frequency,
          kMaxFrameTime);
      previous_time = current_time;
      accumulated_time += frame_time;

      while (accumulated_time >= kPhysicsStep) {
        if (config.ramp_enabled) {
          TransitionBodiesToFloor(bodies, config, kPhysicsStep);
        }
        if (config.spring_enabled) {
          ApplySpringForce(bodies, config.spring_stiffness, kPhysicsStep);
        }
        if (config.ramp_enabled) {
          TransitionBodiesToRamp(bodies, config, kPhysicsStep);
        }
        tiny2d::Update(bodies, kPhysicsStep, static_cast<float>(kAreaWidth),
                       static_cast<float>(kAreaHeight), config.restitution,
                       config.friction, GetElectricField(config));
        if (config.ramp_enabled) {
          TransitionBodiesToFloor(bodies, config, 0.0f);
          TransitionBodiesToRamp(bodies, config, 0.0f);
        }
        const char* airborne_error = GetAirborneError(bodies, config);
        if (airborne_error != nullptr) {
          simulation_started = false;
          bodies.clear();
          accumulated_time = 0.0;
          ShowError(airborne_error);
          break;
        }
        ConstrainBodiesToSurfaces(bodies, config);
        accumulated_time -= kPhysicsStep;
      }
    }

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    if (simulation_started) {
      if (config.spring_enabled) {
        DrawSpring(renderer, bodies);
      }
      for (const tiny2d::Rectangle& body : bodies) {
        DrawRectangle(renderer, body);
      }
    }
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

    SDL_RenderPresent(renderer);
  }

  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
