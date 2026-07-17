#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <limits>
#include <vector>

#include "simulations.h"
#include "tiny2d_engine.h"

namespace {

constexpr int kAreaWidth = 1200;
constexpr int kAreaHeight = 800;
constexpr float kUnitLength = 32.0f;
constexpr float kMonitorHeight = 250.0f;
// Preserve the validated V9 pixel/SI scale when the monitor grows.
constexpr float kSceneTop = 252.0f;
constexpr float kDefaultFloorLengthMeters = 3.0f;
constexpr float kDefaultRampLengthMeters = 3.4641016f;
constexpr float kDefaultLowerBodyX =
    static_cast<float>(kAreaWidth) * 0.5f - kUnitLength * 0.5f;
constexpr float kDegreesToRadians = 0.01745329252f;
constexpr float kSpringAnchorX = 16.0f;
constexpr float kSpringRestFraction = 0.6f;
constexpr float kSpringAmplitude = 10.0f;
constexpr int kSpringCoilCount = 10;
constexpr float kJunctionHysteresis = 0.05f;
constexpr float kAirborneAccelerationTolerance = 0.0001f;
constexpr float kPhysicsStep = 1.0f / 120.0f;
constexpr float kMaximumStableStepDistance = kUnitLength * 0.25f;
constexpr double kMaxFrameTime = 0.25;
constexpr std::array<int, 6> kRectangleIndices = {0, 1, 2, 0, 2, 3};
constexpr std::array<const char*, 2> kBodyLabels = {"A", "B"};

struct BodyConfig {
  float surface_x;
  float mass_kg;
  float downhill_speed;
  float length_units;
  bool starts_on_floor{};
  bool charged{};
  float charge{1.0f};
};

struct SimulationConfig {
  float real_floor_length_m{kDefaultFloorLengthMeters};
  float real_ramp_length_m{kDefaultRampLengthMeters};
  float reference_speed_mps{1.0f};
  float body_a_engine_mass{1.0f};
  float gravity_mps2{9.8f};
  float friction{0.0f};
  float restitution{1.0f};
  bool electric_field_enabled{};
  float electric_field_strength_n_per_c{50.0f};
  float electric_field_angle_degrees{};
  bool ramp_enabled{true};
  float ramp_angle_degrees{30.0f};
  bool spring_enabled{true};
  float spring_stiffness{8.0f};
  BodyConfig body_a{1080.0f, 1.0f, 200.0f, 1.0f, false, false, 1.0f};
  BodyConfig body_b{kDefaultLowerBodyX, 3.0f, 0.0f, 1.0f, true, false, 1.0f};
};

struct BodyTelemetry {
  tiny2d::Vec2 position;
  tiny2d::Vec2 velocity;
  tiny2d::Vec2 acceleration;
  float angle{};
};

struct SimulationSnapshot {
  float time{};
  std::array<BodyTelemetry, 2> bodies{};
};

float GetRampAngle(const SimulationConfig& config) {
  return -config.ramp_angle_degrees * kDegreesToRadians;
}

float GetPixelsPerMeter(const SimulationConfig& config) {
  if (!config.ramp_enabled) {
    return static_cast<float>(kAreaWidth) / config.real_floor_length_m;
  }

  const float angle = -GetRampAngle(config);
  const float horizontal_length =
      config.real_floor_length_m + config.real_ramp_length_m * std::cos(angle);
  const float vertical_length = config.real_ramp_length_m * std::sin(angle);
  const float horizontal_scale =
      static_cast<float>(kAreaWidth) / horizontal_length;
  const float vertical_scale =
      (static_cast<float>(kAreaHeight) - kSceneTop) / vertical_length;
  return std::min(horizontal_scale, vertical_scale);
}

float GetRampBottomX(const SimulationConfig& config) {
  return config.real_floor_length_m * GetPixelsPerMeter(config);
}

float GetRampLengthPixels(const SimulationConfig& config) {
  return config.real_ramp_length_m * GetPixelsPerMeter(config);
}

float GetRampTopX(const SimulationConfig& config) {
  return GetRampBottomX(config) +
         GetRampLengthPixels(config) * std::cos(GetRampAngle(config));
}

float GetPixelSpeedPerMeterPerSecond(const SimulationConfig& config) {
  return std::abs(config.body_a.downhill_speed) / config.reference_speed_mps;
}

float GetRealSecondsPerSimulationSecond(const SimulationConfig& config) {
  return GetPixelSpeedPerMeterPerSecond(config) / GetPixelsPerMeter(config);
}

float GetEngineMassPerKilogram(const SimulationConfig& config) {
  return config.body_a_engine_mass / config.body_a.mass_kg;
}

float GetBodyEngineMass(const BodyConfig& body_config,
                        const SimulationConfig& simulation_config) {
  return body_config.mass_kg * GetEngineMassPerKilogram(simulation_config);
}

float GetPixelAccelerationPerMeterPerSecondSquared(
    const SimulationConfig& config) {
  const float speed_scale = GetPixelSpeedPerMeterPerSecond(config);
  return speed_scale * speed_scale / GetPixelsPerMeter(config);
}

float GetPixelGravity(const SimulationConfig& config) {
  return config.gravity_mps2 *
         GetPixelAccelerationPerMeterPerSecondSquared(config);
}

tiny2d::Vec2 GetElectricField(const SimulationConfig& config) {
  if (!config.electric_field_enabled) {
    return {};
  }
  const float pixel_strength =
      config.electric_field_strength_n_per_c *
      GetEngineMassPerKilogram(config) *
      GetPixelAccelerationPerMeterPerSecondSquared(config);
  const float angle = config.electric_field_angle_degrees * kDegreesToRadians;
  return {pixel_strength * std::cos(angle), -pixel_strength * std::sin(angle)};
}

float GetSpringRestX(const SimulationConfig& config) {
  return GetRampBottomX(config) * kSpringRestFraction;
}

float GetSpringAnchorX(const SimulationConfig& config) {
  return std::min(kSpringAnchorX, GetSpringRestX(config) * 0.25f);
}

float GetBodyWidth(const BodyConfig& config) {
  return config.length_units * kUnitLength;
}

float GetMaximumBodyLengthUnits(const SimulationConfig& config) {
  const float ramp_bottom_x = GetRampBottomX(config);
  const float available_length =
      config.ramp_enabled ? std::min(GetRampLengthPixels(config), ramp_bottom_x)
                          : static_cast<float>(kAreaWidth);
  return available_length / kUnitLength;
}

tiny2d::Rectangle CreateBody(const BodyConfig& body_config,
                             const SimulationConfig& simulation_config) {
  const float width = GetBodyWidth(body_config);
  const float half_height = kUnitLength * 0.5f;
  if (!simulation_config.ramp_enabled || body_config.starts_on_floor) {
    return {
        GetBodyEngineMass(body_config, simulation_config),
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
  return {GetBodyEngineMass(body_config, simulation_config),
          position,
          velocity,
          ramp_angle,
          0.0f,
          width,
          kUnitLength,
          true,
          body_config.charged ? body_config.charge : 0.0f};
}

SimulationSnapshot MakeSnapshot(
    const std::vector<tiny2d::Rectangle>& bodies,
    const std::array<tiny2d::Vec2, 2>& previous_velocities, float time,
    float delta_time, const SimulationConfig& config) {
  SimulationSnapshot snapshot;
  snapshot.time = time;
  const std::size_t body_count =
      std::min(snapshot.bodies.size(), bodies.size());
  for (std::size_t i = 0; i < body_count; ++i) {
    snapshot.bodies[i].position = bodies[i].position;
    snapshot.bodies[i].velocity = bodies[i].velocity;
    snapshot.bodies[i].angle = bodies[i].angle;
    if (delta_time > 0.0f) {
      snapshot.bodies[i].acceleration = {
          (bodies[i].velocity.x - previous_velocities[i].x) / delta_time,
          (bodies[i].velocity.y - previous_velocities[i].y) / delta_time};
    } else {
      snapshot.bodies[i].acceleration = tiny2d::GetLinearAcceleration(
          bodies[i], GetElectricField(config), GetPixelGravity(config));
    }
  }
  return snapshot;
}

const SimulationSnapshot* FindSnapshot(
    const std::vector<SimulationSnapshot>& history, float time) {
  if (history.empty() || !std::isfinite(time)) {
    return nullptr;
  }
  const auto next = std::lower_bound(
      history.begin(), history.end(), time,
      [](const SimulationSnapshot& snapshot, float target_time) {
        return snapshot.time < target_time;
      });
  if (next == history.begin()) {
    return &history.front();
  }
  if (next == history.end()) {
    return &history.back();
  }
  const auto previous = next - 1;
  return time - previous->time <= next->time - time ? &*previous : &*next;
}

bool IsTelemetryOnRamp(const BodyTelemetry& body,
                       const SimulationConfig& config) {
  return config.ramp_enabled &&
         std::abs(body.angle - GetRampAngle(config)) <= 0.001f;
}

tiny2d::Vec2 GetPositiveSurfaceDirection(const BodyTelemetry& body,
                                         const SimulationConfig& config) {
  if (IsTelemetryOnRamp(body, config)) {
    const float ramp_angle = GetRampAngle(config);
    return {-std::cos(ramp_angle), -std::sin(ramp_angle)};
  }
  return {-1.0f, 0.0f};
}

float GetSignedSpeed(const BodyTelemetry& body,
                     const SimulationConfig& config) {
  const float speed = std::hypot(body.velocity.x, body.velocity.y);
  if (speed <= 0.0001f) {
    return 0.0f;
  }

  const tiny2d::Vec2 positive_direction =
      GetPositiveSurfaceDirection(body, config);
  const float positive_direction_velocity =
      body.velocity.x * positive_direction.x +
      body.velocity.y * positive_direction.y;
  if (std::abs(positive_direction_velocity) <= 0.0001f) {
    return 0.0f;
  }
  return std::copysign(speed, positive_direction_velocity);
}

float GetSignedSurfaceAcceleration(const BodyTelemetry& body,
                                   const SimulationConfig& config) {
  const tiny2d::Vec2 positive_direction =
      GetPositiveSurfaceDirection(body, config);
  return body.acceleration.x * positive_direction.x +
         body.acceleration.y * positive_direction.y;
}

float GetSignedDistance(const BodyTelemetry& body, float body_width,
                        const SimulationConfig& config) {
  const float ramp_bottom_x = GetRampBottomX(config);
  const float half_width = body_width * 0.5f;
  if (IsTelemetryOnRamp(body, config)) {
    const float ramp_angle = GetRampAngle(config);
    const float center_distance =
        (body.position.x - ramp_bottom_x) * std::cos(ramp_angle) +
        (body.position.y - static_cast<float>(kAreaHeight)) *
            std::sin(ramp_angle);
    return std::max(0.0f, center_distance - half_width);
  }
  return std::min(0.0f, body.position.x + half_width - ramp_bottom_x);
}

float GetMinimumBodySurfaceX(const BodyConfig& body_config,
                             const SimulationConfig& config);
float GetMaximumBodySurfaceX(const BodyConfig& body_config,
                             const SimulationConfig& config);

bool IsFiniteRectangle(const tiny2d::Rectangle& rectangle) {
  const std::array values = {
      rectangle.mass,
      rectangle.position.x,
      rectangle.position.y,
      rectangle.velocity.x,
      rectangle.velocity.y,
      rectangle.angle,
      rectangle.angular_velocity,
      rectangle.width,
      rectangle.height,
      rectangle.charge,
  };
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

const char* GetConfigError(const SimulationConfig& config) {
  const std::array values = {
      config.real_floor_length_m,
      config.real_ramp_length_m,
      config.reference_speed_mps,
      config.body_a_engine_mass,
      config.gravity_mps2,
      config.friction,
      config.restitution,
      config.electric_field_strength_n_per_c,
      config.electric_field_angle_degrees,
      config.ramp_angle_degrees,
      config.spring_stiffness,
      config.body_a.surface_x,
      config.body_a.mass_kg,
      config.body_a.downhill_speed,
      config.body_a.length_units,
      config.body_a.charge,
      config.body_b.surface_x,
      config.body_b.mass_kg,
      config.body_b.downhill_speed,
      config.body_b.length_units,
      config.body_b.charge,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "Every value must be a valid number.";
  }
  if (config.real_floor_length_m <= 0.0f || config.real_ramp_length_m <= 0.0f ||
      config.reference_speed_mps <= 0.0f || config.body_a_engine_mass <= 0.0f) {
    return "SI lengths, A reference speed, and A engine reference mass must "
           "be positive.";
  }
  if (config.real_floor_length_m < 0.1f ||
      config.real_floor_length_m > 10000.0f ||
      config.real_ramp_length_m < 0.1f ||
      config.real_ramp_length_m > 10000.0f ||
      config.reference_speed_mps < 0.01f ||
      config.reference_speed_mps > 1000.0f) {
    return "SI lengths or reference speed is outside the "
           "supported setup range.";
  }
  if (config.body_a_engine_mass < 0.01f ||
      config.body_a_engine_mass > 1000.0f) {
    return "Body A engine reference mass must be in [0.01, 1000].";
  }
  if (config.gravity_mps2 != 9.8f && config.gravity_mps2 != 10.0f) {
    return "Gravity must be either 9.8 or 10 m/s^2.";
  }
  if (config.friction < 0.0f || config.friction > 5.0f ||
      config.restitution < 0.0f || config.restitution > 1.0f) {
    return "Friction must be in [0, 5] and bounciness in [0, 1].";
  }
  if (config.ramp_angle_degrees < 5.0f || config.ramp_angle_degrees > 45.0f) {
    return "Ramp angle must be in [5, 45] degrees.";
  }
  if (config.electric_field_strength_n_per_c < 0.0f ||
      config.electric_field_strength_n_per_c > 1000000.0f ||
      config.electric_field_angle_degrees < -180.0f ||
      config.electric_field_angle_degrees > 180.0f) {
    return "Electric field strength or angle is outside the supported range.";
  }
  if (config.spring_stiffness < 0.1f || config.spring_stiffness > 100.0f) {
    return "Spring strength must be in [0.1, 100].";
  }
  const std::array<const BodyConfig*, 2> body_configs = {&config.body_a,
                                                         &config.body_b};
  for (const BodyConfig* body_config : body_configs) {
    if (body_config->mass_kg < 0.01f || body_config->mass_kg > 10000.0f ||
        body_config->downhill_speed < -5000.0f ||
        body_config->downhill_speed > 5000.0f ||
        body_config->charge < -1000.0f || body_config->charge > 1000.0f ||
        std::abs(body_config->length_units - 1.0f) > 0.0001f) {
      return "A block parameter is outside the supported setup range.";
    }
    const float minimum_x = GetMinimumBodySurfaceX(*body_config, config);
    const float maximum_x = GetMaximumBodySurfaceX(*body_config, config);
    if (body_config->surface_x < minimum_x - 0.001f ||
        body_config->surface_x > maximum_x + 0.001f) {
      return "A block position is outside its supporting surface.";
    }
  }
  if (std::abs(config.body_a.downhill_speed) <= 0.001f) {
    return "Body A pixel speed must be non-zero because it defines the time "
           "scale.";
  }

  const float pixels_per_meter = GetPixelsPerMeter(config);
  const float speed_scale = GetPixelSpeedPerMeterPerSecond(config);
  const float time_scale = GetRealSecondsPerSimulationSecond(config);
  const float mass_scale = GetEngineMassPerKilogram(config);
  const float acceleration_scale =
      GetPixelAccelerationPerMeterPerSecondSquared(config);
  const float pixel_gravity = GetPixelGravity(config);
  const tiny2d::Vec2 electric_field = GetElectricField(config);
  const std::array derived_values = {
      pixels_per_meter,   speed_scale,   time_scale,       mass_scale,
      acceleration_scale, pixel_gravity, electric_field.x, electric_field.y,
  };
  if (!std::all_of(derived_values.begin(), derived_values.end(),
                   [](float value) { return std::isfinite(value); }) ||
      pixels_per_meter <= 0.0f || speed_scale <= 0.0f || time_scale <= 0.0f ||
      mass_scale <= 0.0f || acceleration_scale <= 0.0f) {
    return "This calibration produces an invalid numeric scale.";
  }

  const std::array<tiny2d::Rectangle, 2> bodies = {
      CreateBody(config.body_a, config), CreateBody(config.body_b, config)};
  for (const tiny2d::Rectangle& body : bodies) {
    if (!IsFiniteRectangle(body)) {
      return "This calibration produces a non-finite block state.";
    }
    const double acceleration_x =
        static_cast<double>(electric_field.x) * body.charge / body.mass;
    const double acceleration_y =
        static_cast<double>(pixel_gravity) +
        static_cast<double>(electric_field.y) * body.charge / body.mass;
    if (!std::isfinite(acceleration_x) || !std::isfinite(acceleration_y) ||
        std::abs(acceleration_x) > std::numeric_limits<float>::max() ||
        std::abs(acceleration_y) > std::numeric_limits<float>::max()) {
      return "This calibration produces acceleration outside the float range.";
    }
    const double step_distance =
        static_cast<double>(std::hypot(body.velocity.x, body.velocity.y)) *
            kPhysicsStep +
        0.5 * std::hypot(acceleration_x, acceleration_y) * kPhysicsStep *
            kPhysicsStep;
    if (!std::isfinite(step_distance) ||
        step_distance > kMaximumStableStepDistance) {
      return "The calibration is too fast for a stable physics step. Reduce "
             "pixel speed, field strength, charge, or scale ratios.";
    }
  }

  if (config.spring_enabled) {
    const float minimum_mass =
        std::min(GetBodyEngineMass(config.body_a, config),
                 GetBodyEngineMass(config.body_b, config));
    const double maximum_spring_acceleration =
        static_cast<double>(config.spring_stiffness) * GetSpringRestX(config) /
        minimum_mass;
    const double spring_step_distance =
        0.5 * maximum_spring_acceleration * kPhysicsStep * kPhysicsStep;
    if (!std::isfinite(spring_step_distance) ||
        spring_step_distance > kMaximumStableStepDistance) {
      return "The spring and block mass combination is too stiff for a stable "
             "physics step.";
    }
  }
  if (config.ramp_enabled &&
      (GetRampLengthPixels(config) < GetBodyWidth(config.body_a) ||
       GetRampBottomX(config) < GetBodyWidth(config.body_b))) {
    return "The ramp or floor is too short to contain its block.";
  }
  if (tiny2d::IsColliding(bodies[0], bodies[1])) {
    return "The bodies overlap. Move them farther apart before starting.";
  }
  return nullptr;
}

bool SliderInputFloat(const char* label, float* value, float minimum,
                      float maximum, const char* format,
                      ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  constexpr float kControlStart = 260.0f;
  constexpr float kSliderWidth = 320.0f;
  constexpr float kInputWidth = 100.0f;

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
  *value = std::clamp(*value, minimum, maximum);
  ImGui::PopID();
  return changed;
}

float GetMinimumBodySurfaceX(const BodyConfig& body_config,
                             const SimulationConfig& config) {
  const float half_width = GetBodyWidth(body_config) * 0.5f;
  const bool starts_on_ramp =
      config.ramp_enabled && !body_config.starts_on_floor;
  return starts_on_ramp ? GetRampBottomX(config) +
                              std::cos(GetRampAngle(config)) * half_width
                        : half_width;
}

float GetMaximumBodySurfaceX(const BodyConfig& body_config,
                             const SimulationConfig& config) {
  const float half_width = GetBodyWidth(body_config) * 0.5f;
  if (config.ramp_enabled && !body_config.starts_on_floor) {
    return std::max(
        GetMinimumBodySurfaceX(body_config, config),
        GetRampTopX(config) - std::cos(GetRampAngle(config)) * half_width);
  }
  return std::max(GetMinimumBodySurfaceX(body_config, config),
                  (config.ramp_enabled ? GetRampBottomX(config)
                                       : static_cast<float>(kAreaWidth)) -
                      half_width);
}

void ClampBodySurfaceX(BodyConfig* body_config,
                       const SimulationConfig& config) {
  body_config->surface_x = std::clamp(
      body_config->surface_x, GetMinimumBodySurfaceX(*body_config, config),
      GetMaximumBodySurfaceX(*body_config, config));
}

void DrawChargeControls(const char* label, BodyConfig* body_config) {
  ImGui::PushID(label);
  ImGui::Checkbox(label, &body_config->charged);
  ImGui::BeginDisabled(!body_config->charged);
  SliderInputFloat("Charge q (C)", &body_config->charge, -1000.0f, 1000.0f,
                   "%.6g");
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
  ClampBodySurfaceX(body_config, simulation_config);
  ImGui::TextDisabled(
      starts_on_ramp ? "Position: right is higher. Speed: positive is downhill."
                     : "Position is on the floor. Speed: positive moves left.");
  SliderInputFloat(starts_on_ramp ? "Ramp x (px)" : "Floor x (px)",
                   &body_config->surface_x, minimum_x, maximum_x, "%.1f");
  SliderInputFloat("Pixel initial speed (px/s)", &body_config->downhill_speed,
                   -5000.0f, 5000.0f, "%.1f");
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
  SliderInputFloat("Floor length (m)", &config->real_floor_length_m, 0.1f,
                   10000.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
  ImGui::BeginDisabled(!config->ramp_enabled);
  SliderInputFloat("Ramp angle (degrees)", &config->ramp_angle_degrees, 5.0f,
                   45.0f, "%.1f");
  SliderInputFloat("Ramp length (m)", &config->real_ramp_length_m, 0.1f,
                   10000.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
  ImGui::EndDisabled();
  SliderInputFloat("Body A reference speed (m/s)", &config->reference_speed_mps,
                   0.01f, 1000.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
  SliderInputFloat("Body A reference mass (kg)", &config->body_a.mass_kg, 0.01f,
                   10000.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
  SliderInputFloat("Body B mass (kg)", &config->body_b.mass_kg, 0.01f, 10000.0f,
                   "%.3f", ImGuiSliderFlags_Logarithmic);
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
  SliderInputFloat("Surface friction", &config->friction, 0.0f, 5.0f, "%.2f");
  SliderInputFloat("Collision bounciness", &config->restitution, 0.0f, 1.0f,
                   "%.2f");

  ImGui::Checkbox("Enable electric field", &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  ImGui::TextDisabled(
      "Uniform E field in N/C. Angle is counterclockwise from +X: 0 right, "
      "90 up, -90 down.");
  SliderInputFloat("Electric field strength E (N/C)",
                   &config->electric_field_strength_n_per_c, 0.0f, 1000000.0f,
                   "%.6g");
  SliderInputFloat("Electric field angle (degrees)",
                   &config->electric_field_angle_degrees, -180.0f, 180.0f,
                   "%.1f");
  DrawChargeControls("Body A carries charge", &config->body_a);
  DrawChargeControls("Body B carries charge", &config->body_b);
  ImGui::EndDisabled();
  ImGui::Checkbox("Enable left spring", &config->spring_enabled);

  if (ImGui::CollapsingHeader("Advanced engine values")) {
    ImGui::TextDisabled(
        "These values control the pixel simulation and normally stay at their "
        "defaults.");
    SliderInputFloat("Body A engine reference mass",
                     &config->body_a_engine_mass, 0.01f, 1000.0f, "%.3f",
                     ImGuiSliderFlags_Logarithmic);
    ImGui::BeginDisabled(!config->spring_enabled);
    SliderInputFloat("Spring strength", &config->spring_stiffness, 0.1f, 100.0f,
                     "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::EndDisabled();
    ImGui::Separator();
    DrawBodyControls("Body A", &config->body_a, *config);
    ImGui::Separator();
    DrawBodyControls("Body B", &config->body_b, *config);
  }
  ClampBodySurfaceX(&config->body_a, *config);
  ClampBodySurfaceX(&config->body_b, *config);
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
  assert(std::abs(GetPixelsPerMeter(config) - 200.0f) <= 0.001f);
  assert(std::abs(GetRampTopX(config) - static_cast<float>(kAreaWidth)) <=
         0.001f);
  assert(GetBodyWidth(config.body_a) == kUnitLength);
  assert(GetBodyWidth(config.body_b) == kUnitLength);
  const tiny2d::Rectangle body_b = CreateBody(config.body_b, config);
  assert(std::abs(body_b.angle) <= 0.001f);
  assert(std::abs(body_b.position.x + body_b.width * 0.5f -
                  GetRampBottomX(config)) <= 0.001f);
  assert(GetMaximumBodyLengthUnits(config) * kUnitLength <=
         GetRampBottomX(config) + 0.001f);
  config.ramp_enabled = false;
  assert(GetConfigError(config) == nullptr);
  assert(std::abs(GetMaximumBodyLengthUnits(config) * kUnitLength -
                  static_cast<float>(kAreaWidth)) <= 0.001f);
  config.body_b.surface_x = config.body_a.surface_x;
  assert(GetConfigError(config) != nullptr);
}

void CheckSnapshotLookup() {
  std::vector<SimulationSnapshot> history(3);
  history[0].time = 0.0f;
  history[1].time = 1.0f;
  history[2].time = 2.0f;
  assert(FindSnapshot(history, 0.4f) == &history[0]);
  assert(FindSnapshot(history, 0.6f) == &history[1]);
  assert(FindSnapshot(history, 3.0f) == &history[2]);
}

void CheckDisplayQuantities() {
  SimulationConfig config;
  config.real_floor_length_m *= 2.0f;
  config.real_ramp_length_m *= 2.0f;
  config.reference_speed_mps = 2.0f;
  config.body_a.mass_kg = 2.0f;
  config.body_b.mass_kg = 6.0f;
  assert(std::abs(GetPixelsPerMeter(config) - 100.0f) <= 0.001f);
  assert(std::abs(GetPixelSpeedPerMeterPerSecond(config) - 100.0f) <= 0.001f);
  assert(std::abs(GetRealSecondsPerSimulationSecond(config) - 1.0f) <= 0.001f);
  assert(std::abs(GetEngineMassPerKilogram(config) - 0.5f) <= 0.001f);
  assert(std::abs(GetPixelGravity(config) - 980.0f) <= 0.01f);
  config.electric_field_enabled = true;
  config.electric_field_strength_n_per_c = 9.8f;
  const tiny2d::Vec2 electric_field = GetElectricField(config);
  assert(std::abs(electric_field.x - 490.0f) <= 0.01f);
  const float body_a_electric_acceleration_mps2 =
      electric_field.x * config.body_a.charge /
      GetBodyEngineMass(config.body_a, config) /
      GetPixelAccelerationPerMeterPerSecondSquared(config);
  assert(std::abs(body_a_electric_acceleration_mps2 - 4.9f) <= 0.001f);

  const float ramp_angle = GetRampAngle(config);
  BodyTelemetry body;
  body.angle = ramp_angle;
  body.velocity = {-200.0f * std::cos(ramp_angle),
                   -200.0f * std::sin(ramp_angle)};
  assert(std::abs(GetSignedSpeed(body, config) - 200.0f) <= 0.001f);
  body.acceleration = {-980.0f * std::cos(ramp_angle),
                       -980.0f * std::sin(ramp_angle)};
  assert(std::abs(GetSignedSurfaceAcceleration(body, config) - 980.0f) <=
         0.001f);

  const tiny2d::Rectangle lower_body = CreateBody(config.body_b, config);
  body.position = lower_body.position;
  body.angle = lower_body.angle;
  body.velocity = {-200.0f, 0.0f};
  assert(std::abs(GetSignedSpeed(body, config) - 200.0f) <= 0.001f);
  assert(std::abs(GetSignedDistance(body, lower_body.width, config)) <= 0.001f);
  body.position.x -= 100.0f;
  assert(std::abs(GetSignedDistance(body, lower_body.width, config) + 100.0f) <=
         0.001f);
  body.velocity = {200.0f, 0.0f};
  assert(std::abs(GetSignedSpeed(body, config) + 200.0f) <= 0.001f);
  body.acceleration = {100.0f, 0.0f};
  assert(std::abs(GetSignedSurfaceAcceleration(body, config) + 100.0f) <=
         0.001f);
  assert(std::abs(config.body_b.mass_kg - 6.0f) <= 0.001f);
  assert(std::abs(GetBodyEngineMass(config.body_b, config) - 3.0f) <= 0.001f);
}
#endif

void ShowError(const char* message) {
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tiny2D Engine", message,
                           nullptr);
}

const char* GetNonFiniteStateError(
    const std::vector<tiny2d::Rectangle>& bodies) {
  for (const tiny2d::Rectangle& body : bodies) {
    if (!IsFiniteRectangle(body)) {
      return "The simulation produced a non-finite state. Adjust the setup "
             "parameters and try again.";
    }
  }
  return nullptr;
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
    if (config.ramp_enabled && std::abs(body.angle - ramp_angle) <= 0.001f) {
      outward_normal = {std::sin(ramp_angle), -std::cos(ramp_angle)};
    }

    const tiny2d::Vec2 acceleration = tiny2d::GetLinearAcceleration(
        body, electric_field, GetPixelGravity(config));
    const float outward_acceleration =
        acceleration.x * outward_normal.x + acceleration.y * outward_normal.y;
    // A one-sided surface can push a body away, but cannot pull it back.
    if (outward_acceleration > kAirborneAccelerationTolerance) {
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
  config.body_a.charged = true;
  config.electric_field_enabled = true;
  config.electric_field_angle_degrees = 90.0f;
  config.electric_field_strength_n_per_c = 5.0f;
  std::vector<tiny2d::Rectangle> bodies = {
      CreateBody(config.body_a, config),
      CreateBody(config.body_b, config),
  };
  assert(GetAirborneError(bodies, config) == nullptr);

  const float ramp_angle = GetRampAngle(config);
  const tiny2d::Vec2 outward_normal{std::sin(ramp_angle),
                                    -std::cos(ramp_angle)};
  bodies[0].velocity = outward_normal;
  assert(GetAirborneError(bodies, config) == nullptr);

  bodies[0] = CreateBody(config.body_a, config);
  config.electric_field_strength_n_per_c = config.gravity_mps2 * 2.0f;
  assert(GetAirborneError(bodies, config) != nullptr);

  config.body_a.charged = false;
  bodies[0] = CreateBody(config.body_a, config);
  assert(GetAirborneError(bodies, config) == nullptr);
}
#endif

tiny2d::Rectangle CreateRamp(const SimulationConfig& config) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float ramp_length = GetRampLengthPixels(config);
  const float ramp_height = static_cast<float>(kAreaHeight);
  const tiny2d::Vec2 top_midpoint{
      ramp_bottom_x + std::cos(ramp_angle) * ramp_length * 0.5f,
      static_cast<float>(kAreaHeight) +
          std::sin(ramp_angle) * ramp_length * 0.5f};
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

#ifndef NDEBUG
    const float transitioned_speed_squared =
        body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
    const float expected_end_x =
        ramp_bottom_x - half_width - speed * (delta_time - time_to_junction);
    assert(std::abs(transitioned_speed_squared - speed_squared) <=
           std::max(1.0f, speed_squared) * 0.00001f);
    assert(std::abs(body.position.x + body.velocity.x * delta_time -
                    expected_end_x) <= 0.0001f);
#endif
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

#ifndef NDEBUG
    const float transitioned_speed_squared =
        body.velocity.x * body.velocity.x + body.velocity.y * body.velocity.y;
    const float expected_end_x =
        junction_position.x + body.velocity.x * (delta_time - time_to_junction);
    assert(std::abs(transitioned_speed_squared - speed_squared) <=
           std::max(1.0f, speed_squared) * 0.00001f);
    assert(std::abs(body.position.x + body.velocity.x * delta_time -
                    expected_end_x) <= 0.0001f);
#endif
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
    const bool on_ramp =
        config.ramp_enabled && std::abs(body.angle - ramp_angle) <= 0.001f;
    if (on_ramp) {
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

    if (on_ramp) {
      const tiny2d::Vec2 uphill{std::cos(ramp_angle), std::sin(ramp_angle)};
      const float distance_from_bottom =
          (body.position.x - ramp_bottom_x) * uphill.x +
          (body.position.y - static_cast<float>(kAreaHeight)) * uphill.y;
      const float maximum_distance =
          GetRampLengthPixels(config) - body.width * 0.5f;
      if (distance_from_bottom > maximum_distance) {
        const float excess = distance_from_bottom - maximum_distance;
        body.position.x -= uphill.x * excess;
        body.position.y -= uphill.y * excess;
        const float uphill_velocity =
            body.velocity.x * uphill.x + body.velocity.y * uphill.y;
        if (uphill_velocity > 0.0f) {
          body.velocity.x -= uphill.x * uphill_velocity;
          body.velocity.y -= uphill.y * uphill_velocity;
        }
      }
    }

#ifndef NDEBUG
    const float remaining_position_error =
        (body.position.x - target_position.x) * normal.x +
        (body.position.y - target_position.y) * normal.y;
    const float remaining_normal_velocity =
        body.velocity.x * normal.x + body.velocity.y * normal.y;
    const float position_tolerance =
        std::max(0.0001f, std::hypot(body.position.x, body.position.y) *
                              std::numeric_limits<float>::epsilon() * 8.0f);
    const float velocity_tolerance =
        std::max(0.0001f, std::hypot(body.velocity.x, body.velocity.y) *
                              std::numeric_limits<float>::epsilon() * 8.0f);
    assert(std::abs(remaining_position_error) <= position_tolerance);
    assert(std::abs(remaining_normal_velocity) <= velocity_tolerance);
#endif
  }
}

bool IsOnFloor(const tiny2d::Rectangle& body) {
  return body.mass > 0.0f && body.fixed_rotation &&
         std::abs(body.angle) <= 0.001f;
}

std::size_t FindSpringBodyIndex(const std::vector<tiny2d::Rectangle>& bodies,
                                const SimulationConfig& config) {
  std::size_t body_index = bodies.size();
  float spring_end_x = GetSpringRestX(config);
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

void ApplySpringForce(std::vector<tiny2d::Rectangle>& bodies,
                      const SimulationConfig& config, float delta_time) {
  const std::size_t body_index = FindSpringBodyIndex(bodies, config);
  if (body_index == bodies.size()) {
    return;
  }

  tiny2d::Rectangle& body = bodies[body_index];
  const float left_edge = body.position.x - body.width * 0.5f;
  const float compression = GetSpringRestX(config) - left_edge;
  const float velocity_change =
      config.spring_stiffness * compression / body.mass * delta_time;
#ifndef NDEBUG
  const float previous_velocity_x = body.velocity.x;
#endif
  body.velocity.x += velocity_change;
#ifndef NDEBUG
  assert(body.velocity.x >= previous_velocity_x);
#endif
}

void DrawSpring(SDL_Renderer* renderer,
                const std::vector<tiny2d::Rectangle>& bodies,
                const SimulationConfig& config) {
  const float spring_anchor_x = GetSpringAnchorX(config);
  const float spring_rest_x = GetSpringRestX(config);
  float spring_end_x = spring_rest_x;
  const std::size_t body_index = FindSpringBodyIndex(bodies, config);
  if (body_index != bodies.size()) {
    spring_end_x = std::clamp(
        bodies[body_index].position.x - bodies[body_index].width * 0.5f,
        spring_anchor_x, spring_rest_x);
  }

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

void DrawMonitorWindow(const SimulationConfig& config,
                       const std::vector<SimulationSnapshot>& history,
                       float simulation_time, bool* paused, float* inspect_time,
                       bool* follow_live, bool* back_to_selection) {
  const float length_scale = GetPixelsPerMeter(config);
  const float speed_scale = GetPixelSpeedPerMeterPerSecond(config);
  const float time_scale = GetRealSecondsPerSimulationSecond(config);
  const float real_simulation_time = simulation_time * time_scale;
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
  const float maximum_inspect_time =
      std::max(real_simulation_time, kPhysicsStep * time_scale);
  if (SliderInputFloat("Inspect time (s)", inspect_time, 0.0f,
                       maximum_inspect_time, "%.3f")) {
    *inspect_time = std::min(*inspect_time, real_simulation_time);
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
      FindSnapshot(history, *inspect_time / time_scale);
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

void DrawFieldArrow(ImDrawList* draw_list, ImVec2 center, ImVec2 direction,
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

  constexpr float kHeadLength = 13.0f;
  constexpr float kHeadWidth = 7.0f;
  const ImVec2 normal{-direction.y, direction.x};
  draw_list->AddTriangleFilled(
      end,
      {end.x - direction.x * kHeadLength + normal.x * kHeadWidth,
       end.y - direction.y * kHeadLength + normal.y * kHeadWidth},
      {end.x - direction.x * kHeadLength - normal.x * kHeadWidth,
       end.y - direction.y * kHeadLength - normal.y * kHeadWidth},
      color);
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
    DrawFieldArrow(draw_list, {100.0f, 310.0f}, field_direction, 80.0f,
                   IM_COL32(100, 225, 150, 255));
    char field_label[80];
    std::snprintf(field_label, sizeof(field_label),
                  "E = %.4g N/C, %.2f deg from +X CCW",
                  config.electric_field_strength_n_per_c,
                  config.electric_field_angle_degrees);
    draw_list->AddText({32.0f, 365.0f}, IM_COL32(120, 235, 165, 255),
                       field_label);
  }

  DrawFieldArrow(draw_list, {100.0f, 420.0f}, {0.0f, 1.0f}, 80.0f,
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

namespace tiny2d::sandbox {

SimulationResult RunInclineSpringSimulation(SDL_Renderer* renderer) {
#ifndef NDEBUG
  CheckConfigValidation();
  CheckSnapshotLookup();
  CheckDisplayQuantities();
  CheckAirborneDetection();
  CheckJunctionHysteresis();
#endif

  SimulationConfig config;
  std::vector<tiny2d::Rectangle> bodies;
  // ponytail: Keep one run in memory; cap or stream it only for long sessions.
  std::vector<SimulationSnapshot> history;
  std::array<tiny2d::Vec2, 2> previous_velocities{};
  bool simulation_started = false;
  bool simulation_paused = false;
  bool follow_live = true;
  bool back_to_selection = false;
  float simulation_time = 0.0f;
  float inspect_time = 0.0f;

  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();
  double accumulated_time = 0.0;
  const auto stop_simulation = [&](const char* message) {
    simulation_started = false;
    simulation_paused = false;
    bodies.clear();
    accumulated_time = 0.0;
    ShowError(message);
  };

  while (true) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        return SimulationResult::kQuit;
      }
      if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
          event.key.keysym.sym == SDLK_SPACE && simulation_started) {
        simulation_paused = !simulation_paused;
      }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint64 current_time = SDL_GetPerformanceCounter();
    if (!simulation_started) {
      previous_time = current_time;
      accumulated_time = 0.0;
      if (DrawSetupScreen(&config, &back_to_selection)) {
        assert(GetConfigError(config) == nullptr);
        bodies = {
            CreateBody(config.body_a, config),
            CreateBody(config.body_b, config),
        };
        if (config.ramp_enabled) {
          bodies.push_back(CreateRamp(config));
        }
        assert(bodies.size() == (config.ramp_enabled ? 3 : 2));
        simulation_paused = false;
        follow_live = true;
        simulation_time = 0.0f;
        inspect_time = 0.0f;
        history.clear();
        history.push_back(MakeSnapshot(bodies, previous_velocities,
                                       simulation_time, 0.0f, config));
        simulation_started = true;
      }
    } else if (simulation_paused) {
      previous_time = current_time;
      accumulated_time = 0.0;
    } else {
      const double frame_time = std::min(
          static_cast<double>(current_time - previous_time) / frequency,
          kMaxFrameTime);
      previous_time = current_time;
      accumulated_time += frame_time;

      while (accumulated_time >= kPhysicsStep) {
        for (std::size_t i = 0; i < previous_velocities.size(); ++i) {
          previous_velocities[i] = bodies[i].velocity;
        }
        if (config.ramp_enabled) {
          TransitionBodiesToFloor(bodies, config, kPhysicsStep);
        }
        if (config.spring_enabled) {
          ApplySpringForce(bodies, config, kPhysicsStep);
        }
        if (config.ramp_enabled) {
          TransitionBodiesToRamp(bodies, config, kPhysicsStep);
        }
        const char* airborne_error = GetAirborneError(bodies, config);
        if (airborne_error != nullptr) {
          stop_simulation(airborne_error);
          break;
        }
        try {
          tiny2d::Update(bodies, kPhysicsStep, static_cast<float>(kAreaWidth),
                         static_cast<float>(kAreaHeight), config.restitution,
                         config.friction, GetElectricField(config),
                         GetPixelGravity(config));
        } catch (const std::exception& error) {
          stop_simulation(error.what());
          break;
        }
        const char* state_error = GetNonFiniteStateError(bodies);
        if (state_error != nullptr) {
          stop_simulation(state_error);
          break;
        }
        if (config.ramp_enabled) {
          TransitionBodiesToFloor(bodies, config, 0.0f);
          TransitionBodiesToRamp(bodies, config, 0.0f);
        }
        airborne_error = GetAirborneError(bodies, config);
        if (airborne_error != nullptr) {
          stop_simulation(airborne_error);
          break;
        }
        ConstrainBodiesToSurfaces(bodies, config);
        simulation_time += kPhysicsStep;
        history.push_back(MakeSnapshot(bodies, previous_velocities,
                                       simulation_time, kPhysicsStep, config));
        accumulated_time -= kPhysicsStep;
      }
    }

    if (simulation_started) {
      DrawSimulationLabels(bodies, config);
      DrawMonitorWindow(config, history, simulation_time, &simulation_paused,
                        &inspect_time, &follow_live, &back_to_selection);
    }

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    if (simulation_started) {
      if (config.spring_enabled) {
        DrawSpring(renderer, bodies, config);
      }
      for (const tiny2d::Rectangle& body : bodies) {
        DrawRectangle(renderer, body);
      }
    }
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);

    if (back_to_selection) {
      return SimulationResult::kBackToSelection;
    }
  }
}

}  // namespace tiny2d::sandbox
