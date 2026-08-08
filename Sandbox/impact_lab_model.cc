#include "impact_lab_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace tiny2d::sandbox {
namespace {

constexpr float kRestitutionSpeedThresholdMps = 0.0f;
constexpr Vec2 kGrazingPositionA{10.0f, 7.0f};
constexpr Vec2 kGrazingPositionB{10.0205f, 7.199f};
constexpr Vec2 kDiameterSkipPositionA{8.0f, 7.0f};
constexpr Vec2 kDiameterSkipPositionB{8.21f, 7.0f};
constexpr double kReferencePositionToleranceM = 0.001;
constexpr double kReferenceVelocityToleranceMps = 0.01;

struct InitialConditions {
  Vec2 position_a;
  Vec2 position_b;
  Vec2 velocity_a;
  Vec2 velocity_b;
};

struct AnalyticalReference {
  double entry_time_seconds;
  double exit_time_seconds;
  Vec2 contact_normal;
  Vec2 velocity_a;
  Vec2 velocity_b;
  Vec2 position_a;
  Vec2 position_b;
};

bool IsFinite(Vec2 value) {
  return std::isfinite(value.x) && std::isfinite(value.y);
}

double Dot(Vec2 a, Vec2 b) {
  return static_cast<double>(a.x) * b.x + static_cast<double>(a.y) * b.y;
}

Vec2 Add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }

Vec2 Subtract(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }

Vec2 Multiply(Vec2 value, double scalar) {
  return {static_cast<float>(value.x * scalar),
          static_cast<float>(value.y * scalar)};
}

double Length(Vec2 value) { return std::sqrt(Dot(value, value)); }

bool IsKnownPreset(ImpactLabPreset preset) {
  switch (preset) {
    case ImpactLabPreset::kSupportedGrazing:
    case ImpactLabPreset::kDiameterSkip:
      return true;
  }
  return false;
}

InitialConditions GetInitialConditions(const ImpactLabConfig& config) {
  if (config.preset == ImpactLabPreset::kSupportedGrazing) {
    return {kGrazingPositionA,
            kGrazingPositionB,
            {config.speed_m_s, 0.0f},
            {-config.speed_m_s, 0.0f}};
  }
  return {kDiameterSkipPositionA,
          kDiameterSkipPositionB,
          {config.speed_m_s, 0.0f},
          {}};
}

std::optional<AnalyticalReference> CalculateReference(
    const ImpactLabConfig& config) {
  const InitialConditions initial = GetInitialConditions(config);
  const Vec2 relative_position =
      Subtract(initial.position_b, initial.position_a);
  const Vec2 relative_velocity =
      Subtract(initial.velocity_b, initial.velocity_a);
  const double radius_sum = 2.0 * kImpactLabCircleRadiusM;
  const double quadratic_a = Dot(relative_velocity, relative_velocity);
  const double quadratic_b = 2.0 * Dot(relative_position, relative_velocity);
  const double quadratic_c =
      Dot(relative_position, relative_position) - radius_sum * radius_sum;
  const double discriminant =
      quadratic_b * quadratic_b - 4.0 * quadratic_a * quadratic_c;
  if (!(quadratic_a > 0.0) || discriminant < 0.0 ||
      !std::isfinite(discriminant)) {
    return std::nullopt;
  }

  const double square_root = std::sqrt(discriminant);
  const double entry_time = (-quadratic_b - square_root) / (2.0 * quadratic_a);
  const double exit_time = (-quadratic_b + square_root) / (2.0 * quadratic_a);
  if (!std::isfinite(entry_time) || !std::isfinite(exit_time) ||
      entry_time < 0.0 || exit_time <= entry_time) {
    return std::nullopt;
  }

  const Vec2 contact_position_a =
      Add(initial.position_a, Multiply(initial.velocity_a, entry_time));
  const Vec2 contact_position_b =
      Add(initial.position_b, Multiply(initial.velocity_b, entry_time));
  const Vec2 separation = Subtract(contact_position_b, contact_position_a);
  const double separation_length = Length(separation);
  if (!(separation_length > 0.0) || !std::isfinite(separation_length)) {
    return std::nullopt;
  }
  const Vec2 normal = Multiply(separation, 1.0 / separation_length);
  const double relative_normal_speed = Dot(relative_velocity, normal);
  const double inverse_mass_a = 1.0 / config.mass_a_kg;
  const double inverse_mass_b = 1.0 / config.mass_b_kg;
  const double impulse = -(1.0 + config.restitution) * relative_normal_speed /
                         (inverse_mass_a + inverse_mass_b);
  const Vec2 post_velocity_a =
      Subtract(initial.velocity_a, Multiply(normal, impulse * inverse_mass_a));
  const Vec2 post_velocity_b =
      Add(initial.velocity_b, Multiply(normal, impulse * inverse_mass_b));
  const double remaining_time = kImpactLabPhysicsStep - entry_time;
  return AnalyticalReference{
      entry_time,
      exit_time,
      normal,
      post_velocity_a,
      post_velocity_b,
      Add(contact_position_a, Multiply(post_velocity_a, remaining_time)),
      Add(contact_position_b, Multiply(post_velocity_b, remaining_time)),
  };
}

Circle MakeCircle(float mass, Vec2 position, Vec2 velocity, float restitution) {
  Circle circle;
  circle.mass = mass;
  circle.position = position;
  circle.velocity = velocity;
  circle.angle = 0.0f;
  circle.angular_velocity = 0.0f;
  circle.radius = kImpactLabCircleRadiusM;
  circle.inertia_model = CircleInertiaModel::kSolidDisk;
  circle.fixed_rotation = true;
  circle.charge = 0.0f;
  circle.applied_force = {};
  circle.applied_torque = 0.0f;
  circle.linear_damping_rate = 0.0f;
  circle.angular_damping_rate = 0.0f;
  circle.material = {restitution, 0.0f, 0.0f};
  return circle;
}

bool SameMaterial(const CollisionMaterial& a, const CollisionMaterial& b) {
  return a.restitution == b.restitution &&
         a.static_friction == b.static_friction &&
         a.kinetic_friction == b.kinetic_friction;
}

bool CircleMatchesConfig(const Circle& circle, float mass,
                         const ImpactLabConfig& config) {
  const CollisionMaterial expected_material{config.restitution, 0.0f, 0.0f};
  return circle.mass == mass && circle.radius == kImpactLabCircleRadiusM &&
         circle.inertia_model == CircleInertiaModel::kSolidDisk &&
         circle.fixed_rotation && circle.charge == 0.0f &&
         circle.applied_force.x == 0.0f && circle.applied_force.y == 0.0f &&
         circle.applied_torque == 0.0f && circle.linear_damping_rate == 0.0f &&
         circle.angular_damping_rate == 0.0f &&
         SameMaterial(circle.material, expected_material);
}

bool IsFinite(const Circle& circle) {
  const std::array values = {
      circle.mass,
      circle.position.x,
      circle.position.y,
      circle.velocity.x,
      circle.velocity.y,
      circle.angle,
      circle.angular_velocity,
      circle.radius,
      circle.charge,
      circle.applied_force.x,
      circle.applied_force.y,
      circle.applied_torque,
      circle.linear_damping_rate,
      circle.angular_damping_rate,
      circle.material.restitution,
      circle.material.static_friction,
      circle.material.kinetic_friction,
  };
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

bool CircleFitsArea(const Circle& circle) {
  return circle.position.x >= circle.radius &&
         circle.position.x <= kImpactLabAreaWidthM - circle.radius &&
         circle.position.y >= circle.radius &&
         circle.position.y <= kImpactLabAreaHeightM - circle.radius;
}

double PairPositionError(const std::vector<Circle>& circles,
                         const AnalyticalReference& reference) {
  const Vec2 error_a = Subtract(circles[0].position, reference.position_a);
  const Vec2 error_b = Subtract(circles[1].position, reference.position_b);
  return std::sqrt(Dot(error_a, error_a) + Dot(error_b, error_b));
}

double PairVelocityError(const std::vector<Circle>& circles,
                         const AnalyticalReference& reference) {
  const Vec2 error_a = Subtract(circles[0].velocity, reference.velocity_a);
  const Vec2 error_b = Subtract(circles[1].velocity, reference.velocity_b);
  return std::sqrt(Dot(error_a, error_a) + Dot(error_b, error_b));
}

Vec2 Momentum(const std::vector<Circle>& circles) {
  return {circles[0].mass * circles[0].velocity.x +
              circles[1].mass * circles[1].velocity.x,
          circles[0].mass * circles[0].velocity.y +
              circles[1].mass * circles[1].velocity.y};
}

double KineticEnergy(const std::vector<Circle>& circles) {
  double energy = 0.0;
  for (const Circle& circle : circles) {
    energy += 0.5 * circle.mass * Dot(circle.velocity, circle.velocity);
  }
  return energy;
}

ImpactLabDerived CalculateDerivedUnchecked(const ImpactLabConfig& config,
                                           const ImpactLabState& state) {
  const InitialConditions initial = GetInitialConditions(config);
  const AnalyticalReference reference = *CalculateReference(config);
  ImpactLabDerived derived;
  derived.entry_time_seconds = reference.entry_time_seconds;
  derived.exit_time_seconds = reference.exit_time_seconds;
  derived.contact_normal = reference.contact_normal;
  derived.expected_velocity_a_m_s = reference.velocity_a;
  derived.expected_velocity_b_m_s = reference.velocity_b;
  derived.expected_position_a_m = reference.position_a;
  derived.expected_position_b_m = reference.position_b;
  derived.initial_momentum_kg_m_s = {
      config.mass_a_kg * initial.velocity_a.x +
          config.mass_b_kg * initial.velocity_b.x,
      config.mass_a_kg * initial.velocity_a.y +
          config.mass_b_kg * initial.velocity_b.y};
  derived.expected_kinetic_energy_j =
      0.5 * config.mass_a_kg * Dot(reference.velocity_a, reference.velocity_a) +
      0.5 * config.mass_b_kg * Dot(reference.velocity_b, reference.velocity_b);
  derived.ccd_position_error_m =
      PairPositionError(state.ccd_circles, reference);
  derived.ccd_velocity_error_m_s =
      PairVelocityError(state.ccd_circles, reference);
  derived.ccd_momentum_error_kg_m_s = Length(
      Subtract(Momentum(state.ccd_circles), derived.initial_momentum_kg_m_s));
  derived.ccd_kinetic_energy_error_j = std::abs(
      KineticEnergy(state.ccd_circles) - derived.expected_kinetic_energy_j);
  const Vec2 discrete_velocity_error_a =
      Subtract(state.discrete_circles[0].velocity, initial.velocity_a);
  const Vec2 discrete_velocity_error_b =
      Subtract(state.discrete_circles[1].velocity, initial.velocity_b);
  derived.discrete_velocity_error_m_s =
      std::sqrt(Dot(discrete_velocity_error_a, discrete_velocity_error_a) +
                Dot(discrete_velocity_error_b, discrete_velocity_error_b));
  derived.moving_distance_per_step_m =
      Length(initial.velocity_a) * kImpactLabPhysicsStep;
  derived.moving_distance_to_diameter_ratio =
      derived.moving_distance_per_step_m / (2.0 * kImpactLabCircleRadiusM);

  const bool first_step_reached =
      state.time_seconds + std::numeric_limits<double>::epsilon() >=
      kImpactLabPhysicsStep;
  const bool swept_contact_exits_step =
      reference.entry_time_seconds < kImpactLabPhysicsStep &&
      reference.exit_time_seconds < kImpactLabPhysicsStep;
  derived.discrete_tunneled =
      first_step_reached && swept_contact_exits_step &&
      !IsColliding(state.discrete_circles[0], state.discrete_circles[1]) &&
      derived.discrete_velocity_error_m_s <= kReferenceVelocityToleranceMps;
  derived.ccd_resolved =
      first_step_reached &&
      derived.ccd_position_error_m <= kReferencePositionToleranceM &&
      derived.ccd_velocity_error_m_s <= kReferenceVelocityToleranceMps;
  return derived;
}

bool IsDerivedFinite(const ImpactLabDerived& derived) {
  const std::array values = {
      derived.entry_time_seconds,
      derived.exit_time_seconds,
      static_cast<double>(derived.contact_normal.x),
      static_cast<double>(derived.contact_normal.y),
      static_cast<double>(derived.expected_velocity_a_m_s.x),
      static_cast<double>(derived.expected_velocity_a_m_s.y),
      static_cast<double>(derived.expected_velocity_b_m_s.x),
      static_cast<double>(derived.expected_velocity_b_m_s.y),
      static_cast<double>(derived.expected_position_a_m.x),
      static_cast<double>(derived.expected_position_a_m.y),
      static_cast<double>(derived.expected_position_b_m.x),
      static_cast<double>(derived.expected_position_b_m.y),
      static_cast<double>(derived.initial_momentum_kg_m_s.x),
      static_cast<double>(derived.initial_momentum_kg_m_s.y),
      derived.expected_kinetic_energy_j,
      derived.ccd_position_error_m,
      derived.ccd_velocity_error_m_s,
      derived.ccd_momentum_error_kg_m_s,
      derived.ccd_kinetic_energy_error_j,
      derived.discrete_velocity_error_m_s,
      derived.moving_distance_per_step_m,
      derived.moving_distance_to_diameter_ratio,
  };
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

ImpactLabConfig MakeSupportedGrazingImpactConfig() { return {}; }

ImpactLabConfig MakeDiameterSkipImpactConfig() {
  ImpactLabConfig config;
  config.preset = ImpactLabPreset::kDiameterSkip;
  config.speed_m_s = 240.0f;
  return config;
}

const char* GetImpactLabConfigError(const ImpactLabConfig& config) {
  if (!IsKnownPreset(config.preset)) {
    return "ImpactLab preset is invalid.";
  }
  if (!std::isfinite(config.mass_a_kg) || !std::isfinite(config.mass_b_kg) ||
      !std::isfinite(config.speed_m_s) || !std::isfinite(config.restitution)) {
    return "All ImpactLab inputs must be finite.";
  }
  if (config.mass_a_kg < kImpactLabMinimumMassKg ||
      config.mass_a_kg > kImpactLabMaximumMassKg ||
      config.mass_b_kg < kImpactLabMinimumMassKg ||
      config.mass_b_kg > kImpactLabMaximumMassKg) {
    return "Circle mass must be in [0.1, 10] kg.";
  }
  const float minimum_speed =
      config.preset == ImpactLabPreset::kSupportedGrazing
          ? kImpactLabMinimumGrazingSpeedMps
          : kImpactLabMinimumDiameterSkipSpeedMps;
  const float maximum_speed =
      config.preset == ImpactLabPreset::kSupportedGrazing
          ? kImpactLabMaximumGrazingSpeedMps
          : kImpactLabMaximumDiameterSkipSpeedMps;
  if (config.speed_m_s < minimum_speed || config.speed_m_s > maximum_speed) {
    return "Speed is outside the selected preset's supported range.";
  }
  if (config.restitution < 0.0f || config.restitution > 1.0f) {
    return "Restitution must be in [0, 1].";
  }
  const std::optional<AnalyticalReference> reference =
      CalculateReference(config);
  if (!reference.has_value() || reference->entry_time_seconds < 0.0 ||
      reference->exit_time_seconds >= kImpactLabPhysicsStep) {
    return "The preset must enter and leave contact within one fixed step.";
  }
  return nullptr;
}

const char* GetImpactLabStateError(const ImpactLabConfig& config,
                                   const ImpactLabState& state) {
  if (GetImpactLabConfigError(config) != nullptr) {
    return "The ImpactLab configuration is invalid.";
  }
  if (!std::isfinite(state.time_seconds) || state.time_seconds < 0.0) {
    return "ImpactLab time must be finite and non-negative.";
  }
  if (state.discrete_circles.size() != 2U || state.ccd_circles.size() != 2U) {
    return "Each ImpactLab lane must contain exactly two circles.";
  }
  for (const std::vector<Circle>* lane :
       {&state.discrete_circles, &state.ccd_circles}) {
    for (std::size_t i = 0; i < lane->size(); ++i) {
      const float expected_mass = i == 0 ? config.mass_a_kg : config.mass_b_kg;
      const Circle& circle = (*lane)[i];
      if (!CircleMatchesConfig(circle, expected_mass, config) ||
          !IsFinite(circle) || !CircleFitsArea(circle)) {
        return "An ImpactLab circle contains an invalid state.";
      }
      try {
        static_cast<void>(GetMomentOfInertia(circle));
      } catch (const std::invalid_argument&) {
        return "An ImpactLab circle has invalid inertia.";
      }
    }
  }
  const ImpactLabDerived derived = CalculateDerivedUnchecked(config, state);
  if (!IsDerivedFinite(derived)) {
    return "An ImpactLab telemetry value is not finite.";
  }
  return nullptr;
}

ImpactLabState MakeInitialImpactLabState(const ImpactLabConfig& config) {
  if (const char* error = GetImpactLabConfigError(config)) {
    throw std::invalid_argument(error);
  }
  const InitialConditions initial = GetInitialConditions(config);
  ImpactLabState state;
  state.discrete_circles = {
      MakeCircle(config.mass_a_kg, initial.position_a, initial.velocity_a,
                 config.restitution),
      MakeCircle(config.mass_b_kg, initial.position_b, initial.velocity_b,
                 config.restitution),
  };
  state.ccd_circles = state.discrete_circles;
  if (const char* error = GetImpactLabStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return state;
}

ImpactLabDerived CalculateImpactLabDerived(const ImpactLabConfig& config,
                                           const ImpactLabState& state) {
  if (const char* error = GetImpactLabStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepImpactLab(const ImpactLabConfig& config, float delta_time,
                   ImpactLabState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kImpactLabPhysicsStep ||
      GetImpactLabStateError(config, *state) != nullptr) {
    return false;
  }

  ImpactLabState next = *state;
  std::vector<Rectangle> rectangles;
  Update(rectangles, next.discrete_circles, delta_time, kImpactLabAreaWidthM,
         kImpactLabAreaHeightM, 0.0f, 0.0f, {}, 0.0f,
         kRestitutionSpeedThresholdMps, false);
  Update(rectangles, next.ccd_circles, delta_time, kImpactLabAreaWidthM,
         kImpactLabAreaHeightM, 0.0f, 0.0f, {}, 0.0f,
         kRestitutionSpeedThresholdMps, true);
  const double next_time = next.time_seconds + delta_time;
  if (!std::isfinite(next_time) || next_time <= next.time_seconds) {
    return false;
  }
  next.time_seconds = next_time;
  if (GetImpactLabStateError(config, next) != nullptr) {
    return false;
  }
  *state = std::move(next);
  return true;
}

const ImpactLabState* FindImpactLabState(
    const std::vector<ImpactLabState>& history, double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const ImpactLabState& state, double target_time) {
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
