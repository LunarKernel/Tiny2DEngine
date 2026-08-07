#include "incline_spring_model.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "simulation_history.h"

namespace tiny2d::sandbox::incline_spring {
namespace {

constexpr float kDegreesToRadians = 0.01745329252f;
constexpr float kSpringAnchorX = 16.0f;
constexpr float kSpringRestFraction = 0.6f;
constexpr float kJunctionHysteresis = 0.05f;
constexpr float kAirborneAccelerationTolerance = 0.0001f;
constexpr float kMaximumStableStepDistance = kUnitLength * 0.25f;

Rectangle CreateBody(const BodyConfig& body_config,
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
  const Vec2 position{
      body_config.surface_x + std::sin(ramp_angle) * half_height,
      surface_y - std::cos(ramp_angle) * half_height};
  const Vec2 velocity{-std::cos(ramp_angle) * body_config.downhill_speed,
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

SimulationSnapshot MakeSnapshot(const std::vector<Rectangle>& bodies,
                                const std::array<Vec2, 2>& previous_velocities,
                                double time, float delta_time,
                                const SimulationConfig& config) {
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
      snapshot.bodies[i].acceleration = GetLinearAcceleration(
          bodies[i], GetElectricField(config), GetPixelGravity(config));
    }
  }
  return snapshot;
}

bool IsTelemetryOnRamp(const BodyTelemetry& body,
                       const SimulationConfig& config) {
  return config.ramp_enabled &&
         std::abs(body.angle - GetRampAngle(config)) <= 0.001f;
}

Vec2 GetPositiveSurfaceDirection(const BodyTelemetry& body,
                                 const SimulationConfig& config) {
  if (IsTelemetryOnRamp(body, config)) {
    const float ramp_angle = GetRampAngle(config);
    return {-std::cos(ramp_angle), -std::sin(ramp_angle)};
  }
  return {-1.0f, 0.0f};
}

bool IsFiniteRectangle(const Rectangle& rectangle) {
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

const char* GetNonFiniteStateError(const std::vector<Rectangle>& bodies) {
  for (const Rectangle& body : bodies) {
    if (!IsFiniteRectangle(body)) {
      return "The simulation produced a non-finite state. Adjust the setup "
             "parameters and try again.";
    }
  }
  return nullptr;
}

const char* GetAirborneError(const std::vector<Rectangle>& bodies,
                             const SimulationConfig& config) {
  if (!config.electric_field_enabled) {
    return nullptr;
  }

  const float ramp_angle = GetRampAngle(config);
  const Vec2 electric_field = GetElectricField(config);
  const std::size_t dynamic_body_count =
      std::min<std::size_t>(2, bodies.size());
  for (std::size_t i = 0; i < dynamic_body_count; ++i) {
    const Rectangle& body = bodies[i];
    Vec2 outward_normal{0.0f, -1.0f};
    if (config.ramp_enabled && std::abs(body.angle - ramp_angle) <= 0.001f) {
      outward_normal = {std::sin(ramp_angle), -std::cos(ramp_angle)};
    }

    const Vec2 acceleration =
        GetLinearAcceleration(body, electric_field, GetPixelGravity(config));
    const float outward_acceleration =
        acceleration.x * outward_normal.x + acceleration.y * outward_normal.y;
    if (outward_acceleration > kAirborneAccelerationTolerance) {
      return i == 0 ? "Body A cannot remain in contact with its supporting "
                      "surface. Simulation stopped."
                    : "Body B cannot remain in contact with its supporting "
                      "surface. Simulation stopped.";
    }
  }
  return nullptr;
}

Rectangle CreateRamp(const SimulationConfig& config) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float ramp_length = GetRampLengthPixels(config);
  const float ramp_height = static_cast<float>(kAreaHeight);
  const Vec2 top_midpoint{
      ramp_bottom_x + std::cos(ramp_angle) * ramp_length * 0.5f,
      static_cast<float>(kAreaHeight) +
          std::sin(ramp_angle) * ramp_length * 0.5f};
  const Vec2 center{top_midpoint.x - std::sin(ramp_angle) * ramp_height * 0.5f,
                    top_midpoint.y + std::cos(ramp_angle) * ramp_height * 0.5f};
  return {0.0f, center, {}, ramp_angle, 0.0f, ramp_length, ramp_height, true};
}

void TransitionBodiesToFloor(std::vector<Rectangle>& bodies,
                             const SimulationConfig& config, float delta_time) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);

  for (Rectangle& body : bodies) {
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

void TransitionBodiesToRamp(std::vector<Rectangle>& bodies,
                            const SimulationConfig& config, float delta_time) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);
  const float area_height = static_cast<float>(kAreaHeight);
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);

  for (Rectangle& body : bodies) {
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
    const Vec2 junction_position{
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

void ConstrainBodiesToSurfaces(std::vector<Rectangle>& bodies,
                               const SimulationConfig& config) {
  const float ramp_angle = GetRampAngle(config);
  const float ramp_bottom_x = GetRampBottomX(config);

  // ponytail: This scene uses an ideal guide constraint. Add a general
  // constraint solver only when other surface shapes need it.
  for (Rectangle& body : bodies) {
    if (body.mass <= 0.0f || !body.fixed_rotation) {
      continue;
    }

    Vec2 normal{};
    Vec2 surface_position{};
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
    const Vec2 target_position{surface_position.x + normal.x * half_height,
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
      const Vec2 uphill{std::cos(ramp_angle), std::sin(ramp_angle)};
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

bool IsOnFloor(const Rectangle& body) {
  return body.mass > 0.0f && body.fixed_rotation &&
         std::abs(body.angle) <= 0.001f;
}

std::size_t FindSpringBodyIndex(const std::vector<Rectangle>& bodies,
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

void ApplySpringForce(std::vector<Rectangle>& bodies,
                      const SimulationConfig& config, float delta_time) {
  const std::size_t body_index = FindSpringBodyIndex(bodies, config);
  if (body_index == bodies.size()) {
    return;
  }

  Rectangle& body = bodies[body_index];
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

}  // namespace

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

Vec2 GetElectricField(const SimulationConfig& config) {
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

float GetSpringEndX(const State& state) {
  const float spring_anchor_x = GetSpringAnchorX(state.config);
  const float spring_rest_x = GetSpringRestX(state.config);
  const std::size_t body_index =
      FindSpringBodyIndex(state.bodies, state.config);
  if (body_index == state.bodies.size()) {
    return spring_rest_x;
  }
  return std::clamp(state.bodies[body_index].position.x -
                        state.bodies[body_index].width * 0.5f,
                    spring_anchor_x, spring_rest_x);
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

void ClampBodySurfaceX(BodyConfig& body_config,
                       const SimulationConfig& config) {
  body_config.surface_x = std::clamp(
      body_config.surface_x, GetMinimumBodySurfaceX(body_config, config),
      GetMaximumBodySurfaceX(body_config, config));
}

float GetSignedSpeed(const BodyTelemetry& body,
                     const SimulationConfig& config) {
  const float speed = std::hypot(body.velocity.x, body.velocity.y);
  if (speed <= 0.0001f) {
    return 0.0f;
  }

  const Vec2 positive_direction = GetPositiveSurfaceDirection(body, config);
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
  const Vec2 positive_direction = GetPositiveSurfaceDirection(body, config);
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
    return "SI lengths or reference speed is outside the supported setup "
           "range.";
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
  const Vec2 electric_field = GetElectricField(config);
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

  const std::array<Rectangle, 2> bodies = {CreateBody(config.body_a, config),
                                           CreateBody(config.body_b, config)};
  for (const Rectangle& body : bodies) {
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
  if (IsColliding(bodies[0], bodies[1])) {
    return "The bodies overlap. Move them farther apart before starting.";
  }
  return nullptr;
}

const char* Reset(const SimulationConfig& config, State& state) {
  if (const char* error = GetConfigError(config); error != nullptr) {
    return error;
  }

  State new_state;
  new_state.config = config;
  new_state.bodies = {CreateBody(config.body_a, config),
                      CreateBody(config.body_b, config)};
  if (config.ramp_enabled) {
    new_state.bodies.push_back(CreateRamp(config));
  }
  const std::array<Vec2, 2> previous_velocities{};
  if (!AppendHistorySample(&new_state.history,
                           MakeSnapshot(new_state.bodies, previous_velocities,
                                        0.0, 0.0f, config),
                           &SimulationSnapshot::time)) {
    return "The initial simulation snapshot is invalid.";
  }
  state = std::move(new_state);
  return nullptr;
}

const char* Step(State& state, float delta_time) {
  if (!std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kPhysicsStep) {
    return "Physics delta time must be finite and in (0, 1/120] seconds.";
  }
  const std::size_t expected_body_count = state.config.ramp_enabled ? 3U : 2U;
  if (state.bodies.size() != expected_body_count) {
    return "The incline simulation state has an invalid body layout.";
  }
  if (const char* error = GetNonFiniteStateError(state.bodies);
      error != nullptr) {
    return error;
  }
  const double next_time = state.time + static_cast<double>(delta_time);
  if (!std::isfinite(state.time) || state.time < 0.0 ||
      !std::isfinite(next_time) || next_time <= state.time ||
      (!state.history.empty() && next_time <= state.history.back().time)) {
    return "The incline simulation clock or history is invalid.";
  }

  std::array<Vec2, 2> previous_velocities{};
  for (std::size_t i = 0; i < previous_velocities.size(); ++i) {
    previous_velocities[i] = state.bodies[i].velocity;
  }
  if (state.config.ramp_enabled) {
    TransitionBodiesToFloor(state.bodies, state.config, delta_time);
  }
  if (state.config.spring_enabled) {
    ApplySpringForce(state.bodies, state.config, delta_time);
  }
  if (state.config.ramp_enabled) {
    TransitionBodiesToRamp(state.bodies, state.config, delta_time);
  }
  if (const char* error = GetAirborneError(state.bodies, state.config);
      error != nullptr) {
    return error;
  }

  Update(state.bodies, delta_time, static_cast<float>(kAreaWidth),
         static_cast<float>(kAreaHeight), state.config.restitution,
         state.config.friction, GetElectricField(state.config),
         GetPixelGravity(state.config));
  if (const char* error = GetNonFiniteStateError(state.bodies);
      error != nullptr) {
    return error;
  }
  if (state.config.ramp_enabled) {
    TransitionBodiesToFloor(state.bodies, state.config, 0.0f);
    TransitionBodiesToRamp(state.bodies, state.config, 0.0f);
  }
  if (const char* error = GetAirborneError(state.bodies, state.config);
      error != nullptr) {
    return error;
  }
  ConstrainBodiesToSurfaces(state.bodies, state.config);

  if (!AppendHistorySample(&state.history,
                           MakeSnapshot(state.bodies, previous_velocities,
                                        next_time, delta_time, state.config),
                           &SimulationSnapshot::time)) {
    return "The incline simulation history rejected its next snapshot.";
  }
  state.time = next_time;
  return nullptr;
}

const SimulationSnapshot* FindSnapshot(const State& state, double time) {
  if (state.history.empty() || !std::isfinite(time)) {
    return nullptr;
  }
  const auto next = std::lower_bound(
      state.history.begin(), state.history.end(), time,
      [](const SimulationSnapshot& snapshot, double target_time) {
        return snapshot.time < target_time;
      });
  if (next == state.history.begin()) {
    return &state.history.front();
  }
  if (next == state.history.end()) {
    return &state.history.back();
  }
  const auto previous = next - 1;
  return time - previous->time <= next->time - time ? &*previous : &*next;
}

}  // namespace tiny2d::sandbox::incline_spring
