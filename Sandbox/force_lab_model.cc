#include "force_lab_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tiny2d::sandbox {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;
constexpr float kMaximumNaturalFrequencyStep = 0.25f;
constexpr float kMaximumDistanceFractionPerStep = 0.25f;
constexpr float kMaximumAnglePerStep = 0.25f;
constexpr float kBoundaryMarginM = 0.05f;

bool IsFinite(Vec2 value) {
  return std::isfinite(value.x) && std::isfinite(value.y);
}

bool FitsFloat(double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<float>::max();
}

Vec2 Add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }

Vec2 Subtract(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }

Vec2 Multiply(Vec2 value, float scalar) {
  return {value.x * scalar, value.y * scalar};
}

float Cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

float Length(Vec2 value) { return std::hypot(value.x, value.y); }

Vec2 Rotate(Vec2 local, float angle) {
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {local.x * cosine - local.y * sine, local.x * sine + local.y * cosine};
}

double MomentOfInertiaUnchecked(const ForceLabConfig& config) {
  return static_cast<double>(config.mass_kg) *
         (static_cast<double>(config.width_m) * config.width_m +
          static_cast<double>(config.height_m) * config.height_m) /
         12.0;
}

Rectangle MakeBodyUnchecked(const ForceLabConfig& config) {
  Rectangle body;
  body.mass = config.mass_kg;
  body.position = config.initial_center_position_m;
  body.velocity = config.initial_velocity_m_s;
  body.angle = config.initial_angle_degrees * kDegreesToRadians;
  body.angular_velocity = config.initial_angular_velocity_rad_s;
  body.width = config.width_m;
  body.height = config.height_m;
  body.fixed_rotation = false;
  body.charge = 0.0f;
  body.linear_damping_rate = config.linear_damping_n_s_m / config.mass_kg;
  body.angular_damping_rate = static_cast<float>(
      config.angular_damping_n_m_s_rad / MomentOfInertiaUnchecked(config));
  return body;
}

bool IsInsideIntegrationArea(const Rectangle& body) {
  for (Vec2 vertex : GetVertices(body)) {
    if (vertex.x < kBoundaryMarginM ||
        vertex.x > kForceLabAreaWidthM - kBoundaryMarginM ||
        vertex.y < kBoundaryMarginM ||
        vertex.y > kForceLabAreaHeightM - kBoundaryMarginM) {
      return false;
    }
  }
  return true;
}

ForceLabDerived CalculateDerivedUnchecked(const ForceLabConfig& config,
                                          const ForceLabState& state) {
  const Rectangle& body = state.body;
  const Vec2 attachment_offset = Rotate(config.attachment_local_m, body.angle);
  const Vec2 attachment_position = Add(body.position, attachment_offset);
  const Vec2 anchor_to_attachment =
      Subtract(attachment_position, config.anchor_position_m);
  const float spring_length = Length(anchor_to_attachment);
  const float extension = spring_length - config.spring_rest_length_m;
  const float force_scale =
      -config.spring_stiffness_n_m * extension / spring_length;
  const Vec2 spring_force = Multiply(anchor_to_attachment, force_scale);
  const Vec2 damping_force{-config.linear_damping_n_s_m * body.velocity.x,
                           -config.linear_damping_n_s_m * body.velocity.y};
  const float inertia = GetMomentOfInertia(body);
  const float spring_torque = Cross(attachment_offset, spring_force);
  const float damping_torque =
      -config.angular_damping_n_m_s_rad * body.angular_velocity;
  const Vec2 total_force = Add(spring_force, damping_force);
  const Vec2 acceleration = Multiply(total_force, 1.0f / body.mass);
  const float angular_acceleration = (spring_torque + damping_torque) / inertia;

  const double speed_squared =
      static_cast<double>(body.velocity.x) * body.velocity.x +
      static_cast<double>(body.velocity.y) * body.velocity.y;
  const double translational_energy = 0.5 * body.mass * speed_squared;
  const double rotational_energy =
      0.5 * inertia * body.angular_velocity * body.angular_velocity;
  const double spring_energy =
      0.5 * config.spring_stiffness_n_m * extension * extension;
  const double mechanical_energy =
      translational_energy + rotational_energy + spring_energy;

  return {
      attachment_offset,
      attachment_position,
      spring_force,
      damping_force,
      acceleration,
      spring_length,
      extension,
      inertia,
      spring_torque,
      damping_torque,
      angular_acceleration,
      translational_energy,
      rotational_energy,
      spring_energy,
      mechanical_energy,
      mechanical_energy + state.dissipated_energy_j,
  };
}

bool IsDerivedFinite(const ForceLabDerived& derived) {
  const std::array<Vec2, 5> vectors = {
      derived.attachment_offset_world_m,
      derived.attachment_position_m,
      derived.spring_force_n,
      derived.damping_force_n,
      derived.linear_acceleration_m_s2,
  };
  const std::array<double, 11> scalars = {
      derived.spring_length_m,
      derived.spring_extension_m,
      derived.moment_of_inertia_kg_m2,
      derived.spring_torque_n_m,
      derived.damping_torque_n_m,
      derived.angular_acceleration_rad_s2,
      derived.translational_kinetic_energy_j,
      derived.rotational_kinetic_energy_j,
      derived.spring_potential_energy_j,
      derived.mechanical_energy_j,
      derived.accounted_energy_j,
  };
  return std::all_of(vectors.begin(), vectors.end(), IsFinite) &&
         std::all_of(scalars.begin(), scalars.end(),
                     [](double value) { return std::isfinite(value); });
}

bool HasExpectedBodyDefinition(const ForceLabConfig& config,
                               const Rectangle& body) {
  const double inertia = MomentOfInertiaUnchecked(config);
  const float expected_linear_rate =
      config.linear_damping_n_s_m / config.mass_kg;
  const float expected_angular_rate =
      static_cast<float>(config.angular_damping_n_m_s_rad / inertia);
  return body.mass == config.mass_kg && body.width == config.width_m &&
         body.height == config.height_m && !body.fixed_rotation &&
         body.charge == 0.0f && body.applied_force.x == 0.0f &&
         body.applied_force.y == 0.0f && body.applied_torque == 0.0f &&
         body.linear_damping_rate == expected_linear_rate &&
         body.angular_damping_rate == expected_angular_rate;
}

}  // namespace

ForceLabConfig MakeCenteredReferenceConfig() {
  ForceLabConfig config;
  config.attachment_local_m = {};
  return config;
}

ForceLabConfig MakeEccentricDemoConfig() { return {}; }

const char* GetForceLabConfigError(const ForceLabConfig& config) {
  const std::array<float, 17> values = {
      config.mass_kg,
      config.width_m,
      config.height_m,
      config.anchor_position_m.x,
      config.anchor_position_m.y,
      config.attachment_local_m.x,
      config.attachment_local_m.y,
      config.spring_stiffness_n_m,
      config.spring_rest_length_m,
      config.initial_center_position_m.x,
      config.initial_center_position_m.y,
      config.initial_velocity_m_s.x,
      config.initial_velocity_m_s.y,
      config.initial_angle_degrees,
      config.initial_angular_velocity_rad_s,
      config.linear_damping_n_s_m,
      config.angular_damping_n_m_s_rad,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All ForceLab inputs must be finite.";
  }
  if (config.mass_kg < kForceLabMinimumMassKg ||
      config.mass_kg > kForceLabMaximumMassKg) {
    return "Mass must be in [0.01, 1000] kg.";
  }
  if (config.width_m < kForceLabMinimumBodyDimensionM ||
      config.width_m > kForceLabMaximumBodyDimensionM ||
      config.height_m < kForceLabMinimumBodyDimensionM ||
      config.height_m > kForceLabMaximumBodyDimensionM) {
    return "Body width and height must be in [0.1, 10] m.";
  }
  if (config.anchor_position_m.x < 0.0f ||
      config.anchor_position_m.x > kForceLabAreaWidthM ||
      config.anchor_position_m.y < 0.0f ||
      config.anchor_position_m.y > kForceLabAreaHeightM) {
    return "The spring anchor must be inside the ForceLab area.";
  }
  if (std::abs(config.attachment_local_m.x) > config.width_m * 0.5f ||
      std::abs(config.attachment_local_m.y) > config.height_m * 0.5f) {
    return "The local spring attachment must lie inside the body.";
  }
  if (config.spring_stiffness_n_m < kForceLabMinimumSpringStiffnessNM ||
      config.spring_stiffness_n_m > kForceLabMaximumSpringStiffnessNM) {
    return "Spring stiffness must be in [0.01, 1000] N/m.";
  }
  if (config.spring_rest_length_m < kForceLabMinimumSpringLengthM ||
      config.spring_rest_length_m > kForceLabMaximumSpringLengthM) {
    return "Spring rest length must be in [1e-5, 50] m.";
  }
  if (std::hypot(config.initial_velocity_m_s.x, config.initial_velocity_m_s.y) >
          kForceLabMaximumInitialSpeedMps ||
      std::abs(config.initial_angular_velocity_rad_s) >
          kForceLabMaximumInitialAngularSpeedRadS ||
      config.initial_angle_degrees < -180.0f ||
      config.initial_angle_degrees > 180.0f) {
    return "Initial speed, angular speed, or angle is outside the supported "
           "range.";
  }
  if (config.linear_damping_n_s_m < 0.0f ||
      config.linear_damping_n_s_m > kForceLabMaximumDampingCoefficient ||
      config.angular_damping_n_m_s_rad < 0.0f ||
      config.angular_damping_n_m_s_rad > kForceLabMaximumDampingCoefficient) {
    return "Damping coefficients must be in [0, 1000].";
  }

  const double inertia = MomentOfInertiaUnchecked(config);
  const double linear_damping_rate =
      config.linear_damping_n_s_m / config.mass_kg;
  const double angular_damping_rate =
      config.angular_damping_n_m_s_rad / inertia;
  if (!FitsFloat(inertia) || inertia <= 0.0 ||
      !FitsFloat(linear_damping_rate) || !FitsFloat(angular_damping_rate) ||
      std::sqrt(config.spring_stiffness_n_m / config.mass_kg) *
              kForceLabPhysicsStep >
          kMaximumNaturalFrequencyStep) {
    return "This mass, geometry, stiffness, or damping combination is too "
           "extreme for stable integration.";
  }

  const Rectangle body = MakeBodyUnchecked(config);
  if (!IsInsideIntegrationArea(body)) {
    return "The initial body must fit inside the ForceLab area.";
  }
  const ForceLabState state{body};
  const ForceLabDerived derived = CalculateDerivedUnchecked(config, state);
  if (derived.spring_length_m < kForceLabMinimumSpringLengthM ||
      derived.spring_length_m > kForceLabMaximumSpringLengthM) {
    return "Initial spring length must be in [1e-5, 50] m.";
  }
  if (!IsDerivedFinite(derived)) {
    return "The selected values exceed stable ForceLab calculations.";
  }
  const double maximum_step_distance =
      std::min(config.width_m, config.height_m) *
      kMaximumDistanceFractionPerStep;
  const double step_distance =
      std::hypot(body.velocity.x, body.velocity.y) * kForceLabPhysicsStep +
      0.5 *
          std::hypot(derived.linear_acceleration_m_s2.x,
                     derived.linear_acceleration_m_s2.y) *
          kForceLabPhysicsStep * kForceLabPhysicsStep;
  const double angle_step =
      std::abs(body.angular_velocity) * kForceLabPhysicsStep +
      0.5 * std::abs(derived.angular_acceleration_rad_s2) *
          kForceLabPhysicsStep * kForceLabPhysicsStep;
  if (!std::isfinite(step_distance) || step_distance > maximum_step_distance ||
      !std::isfinite(angle_step) || angle_step > kMaximumAnglePerStep) {
    return "The initial ForceLab state moves too far in one fixed step.";
  }
  return nullptr;
}

const char* GetForceLabStateError(const ForceLabConfig& config,
                                  const ForceLabState& state) {
  if (GetForceLabConfigError(config) != nullptr) {
    return "The ForceLab configuration is invalid.";
  }
  const Rectangle& body = state.body;
  const std::array<float, 15> values = {
      body.mass,
      body.position.x,
      body.position.y,
      body.velocity.x,
      body.velocity.y,
      body.angle,
      body.angular_velocity,
      body.width,
      body.height,
      body.charge,
      body.applied_force.x,
      body.applied_force.y,
      body.applied_torque,
      body.linear_damping_rate,
      body.angular_damping_rate,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); }) ||
      !std::isfinite(state.time_seconds) ||
      !std::isfinite(state.dissipated_energy_j)) {
    return "The ForceLab state contains NaN or infinity.";
  }
  if (state.time_seconds < 0.0 || state.dissipated_energy_j < 0.0) {
    return "ForceLab time and dissipated energy must be non-negative.";
  }
  if (!HasExpectedBodyDefinition(config, body)) {
    return "The ForceLab body definition or pending load is invalid.";
  }
  if (!IsInsideIntegrationArea(body)) {
    return "The ForceLab body reached the internal integration boundary.";
  }

  const ForceLabDerived derived = CalculateDerivedUnchecked(config, state);
  if (derived.spring_length_m < kForceLabMinimumSpringLengthM ||
      derived.spring_length_m > kForceLabMaximumSpringLengthM) {
    return "Spring length left the supported [1e-5, 50] m range.";
  }
  if (!IsDerivedFinite(derived)) {
    return "A derived ForceLab value is not finite.";
  }
  const double maximum_step_distance =
      std::min(body.width, body.height) * kMaximumDistanceFractionPerStep;
  const double step_distance =
      std::hypot(body.velocity.x, body.velocity.y) * kForceLabPhysicsStep +
      0.5 *
          std::hypot(derived.linear_acceleration_m_s2.x,
                     derived.linear_acceleration_m_s2.y) *
          kForceLabPhysicsStep * kForceLabPhysicsStep;
  const double angle_step =
      std::abs(body.angular_velocity) * kForceLabPhysicsStep +
      0.5 * std::abs(derived.angular_acceleration_rad_s2) *
          kForceLabPhysicsStep * kForceLabPhysicsStep;
  if (!std::isfinite(step_distance) || step_distance > maximum_step_distance ||
      !std::isfinite(angle_step) || angle_step > kMaximumAnglePerStep) {
    return "The ForceLab body moves too far in one fixed step.";
  }
  return nullptr;
}

float GetForceLabCenteredPeriod(const ForceLabConfig& config) {
  if (!std::isfinite(config.mass_kg) || config.mass_kg <= 0.0f ||
      !std::isfinite(config.spring_stiffness_n_m) ||
      config.spring_stiffness_n_m <= 0.0f) {
    return std::numeric_limits<float>::infinity();
  }
  return 2.0f * kPi * std::sqrt(config.mass_kg / config.spring_stiffness_n_m);
}

ForceLabState MakeInitialForceLabState(const ForceLabConfig& config) {
  if (const char* error = GetForceLabConfigError(config)) {
    throw std::invalid_argument(error);
  }
  ForceLabState state{MakeBodyUnchecked(config)};
  if (const char* error = GetForceLabStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return state;
}

ForceLabDerived CalculateForceLabDerived(const ForceLabConfig& config,
                                         const ForceLabState& state) {
  if (const char* error = GetForceLabStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepForceLab(const ForceLabConfig& config, float delta_time,
                  ForceLabState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kForceLabPhysicsStep ||
      GetForceLabStateError(config, *state) != nullptr) {
    return false;
  }

  ForceLabState next = *state;
  const ForceLabDerived before = CalculateDerivedUnchecked(config, next);
  AddForceAtPoint(next.body, before.spring_force_n,
                  before.attachment_position_m);

  const float pre_damping_velocity_x =
      static_cast<float>(static_cast<double>(next.body.velocity.x) +
                         static_cast<double>(next.body.applied_force.x) /
                             next.body.mass * delta_time);
  const float pre_damping_velocity_y =
      static_cast<float>(static_cast<double>(next.body.velocity.y) +
                         static_cast<double>(next.body.applied_force.y) /
                             next.body.mass * delta_time);
  const float pre_damping_angular_velocity =
      static_cast<float>(static_cast<double>(next.body.angular_velocity) +
                         static_cast<double>(next.body.applied_torque) /
                             before.moment_of_inertia_kg_m2 * delta_time);
  const float linear_factor =
      std::exp(-next.body.linear_damping_rate * delta_time);
  const float angular_factor =
      std::exp(-next.body.angular_damping_rate * delta_time);
  const double linear_energy_before_damping =
      0.5 * next.body.mass *
      (pre_damping_velocity_x * pre_damping_velocity_x +
       pre_damping_velocity_y * pre_damping_velocity_y);
  const double angular_energy_before_damping =
      0.5 * before.moment_of_inertia_kg_m2 * pre_damping_angular_velocity *
      pre_damping_angular_velocity;
  const double damping_loss =
      linear_energy_before_damping * (1.0 - linear_factor * linear_factor) +
      angular_energy_before_damping * (1.0 - angular_factor * angular_factor);

  std::vector<Rectangle> bodies{next.body};
  Update(bodies, delta_time, kForceLabAreaWidthM, kForceLabAreaHeightM, 0.0f,
         0.0f, {}, 0.0f);
  next.body = bodies.front();
  const double next_time = next.time_seconds + delta_time;
  const double next_dissipated_energy =
      next.dissipated_energy_j + std::max(0.0, damping_loss);
  if (!std::isfinite(next_time) || next_time <= next.time_seconds ||
      !std::isfinite(next_dissipated_energy)) {
    return false;
  }
  next.time_seconds = next_time;
  next.dissipated_energy_j = next_dissipated_energy;
  if (GetForceLabStateError(config, next) != nullptr) {
    return false;
  }
  *state = next;
  return true;
}

const ForceLabState* FindForceLabState(
    const std::vector<ForceLabState>& history, double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const ForceLabState& state, double target_time) {
                         return state.time_seconds < target_time;
                       });
  if (next == history.begin()) {
    return &history.front();
  }
  if (next == history.end()) {
    return &history.back();
  }
  const auto previous = next - 1;
  return time_seconds - previous->time_seconds <=
                 next->time_seconds - time_seconds
             ? &*previous
             : &*next;
}

}  // namespace tiny2d::sandbox
