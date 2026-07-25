#include "rolling_disk_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace tiny2d::sandbox {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;
constexpr float kMinimumMassOrRadius = 0.000001f;
constexpr float kMaximumFrictionCoefficient = 5.0f;
constexpr float kSlipTolerance = 0.00001f;
constexpr float kPositionTolerance = 0.00001f;
constexpr double kRelativeTolerance =
    64.0 * std::numeric_limits<double>::epsilon();

struct Forces {
  double moment_of_inertia;
  double tangential_external;
  double normal;
  double normal_scale;
};

struct EndpointHit {
  double time;
  RollingDiskStatus status;
};

struct SegmentResult {
  bool valid;
  double elapsed;
};

bool IsFinite(float value) { return std::isfinite(value); }

bool FitsFloat(double value) {
  return std::isfinite(value) &&
         std::abs(value) <= std::numeric_limits<float>::max();
}

bool IsKnownKind(RollingDiskKind kind) {
  switch (kind) {
    case RollingDiskKind::kSolidDisk:
    case RollingDiskKind::kHoop:
      return true;
  }
  return false;
}

bool IsKnownContactMode(RollingContactMode mode) {
  switch (mode) {
    case RollingContactMode::kRolling:
    case RollingContactMode::kSliding:
      return true;
  }
  return false;
}

bool IsKnownStatus(RollingDiskStatus status) {
  switch (status) {
    case RollingDiskStatus::kActive:
    case RollingDiskStatus::kReachedTop:
    case RollingDiskStatus::kReachedBottom:
    case RollingDiskStatus::kAirborne:
      return true;
  }
  return false;
}

double InertiaFactor(RollingDiskKind kind) {
  return kind == RollingDiskKind::kSolidDisk ? 0.5 : 1.0;
}

Forces CalculateForces(const RollingDiskConfig& config) {
  const double angle =
      static_cast<double>(config.ramp_angle_degrees) * kDegreesToRadians;
  const double moment_of_inertia = InertiaFactor(config.kind) * config.mass_kg *
                                   config.radius_m * config.radius_m;
  double tangential_external = static_cast<double>(config.mass_kg) *
                               config.gravity_m_s2 * std::sin(angle);
  const double gravity_normal = static_cast<double>(config.mass_kg) *
                                config.gravity_m_s2 * std::cos(angle);
  double electric_normal = 0.0;
  double normal = gravity_normal;

  if (config.electric_field_enabled) {
    const double field_angle =
        static_cast<double>(config.electric_field_angle_degrees) *
        kDegreesToRadians;
    const double relative_angle = field_angle - angle;
    const double electric_force = static_cast<double>(config.charge_c) *
                                  config.electric_field_strength_n_c;
    tangential_external -= electric_force * std::cos(relative_angle);
    electric_normal = electric_force * std::sin(relative_angle);
    normal -= electric_normal;
  }
  return {moment_of_inertia, tangential_external, normal,
          std::abs(gravity_normal) + std::abs(electric_normal)};
}

double EffectiveInverseMass(const RollingDiskConfig& config,
                            const Forces& forces) {
  return 1.0 / config.mass_kg + static_cast<double>(config.radius_m) *
                                    config.radius_m / forces.moment_of_inertia;
}

double RequiredRollingFriction(const RollingDiskConfig& config,
                               const Forces& forces) {
  const double free_acceleration = forces.tangential_external / config.mass_kg;
  return -free_acceleration / EffectiveInverseMass(config, forces);
}

double SlipVelocity(const RollingDiskConfig& config,
                    const RollingDiskState& state) {
  return static_cast<double>(state.velocity_down_ramp_m_s) -
         static_cast<double>(config.radius_m) * state.angular_velocity_rad_s;
}

double SlidingFriction(const RollingDiskConfig& config, const Forces& forces,
                       double slip_or_free_acceleration) {
  if (config.kinetic_friction_coefficient == 0.0f ||
      slip_or_free_acceleration == 0.0) {
    return 0.0;
  }
  return -std::copysign(
      static_cast<double>(config.kinetic_friction_coefficient) * forces.normal,
      slip_or_free_acceleration);
}

bool CanRoll(const RollingDiskConfig& config, const Forces& forces) {
  const double required = std::abs(RequiredRollingFriction(config, forces));
  const double limit =
      static_cast<double>(config.static_friction_coefficient) * forces.normal;
  const double tolerance =
      kRelativeTolerance * std::max(required, std::abs(limit));
  return limit >= 0.0 && required <= limit + tolerance;
}

bool HasSurfaceContact(const Forces& forces) {
  return forces.normal > kRelativeTolerance * forces.normal_scale;
}

void ConsiderEndpointRoot(double root, double maximum_time,
                          RollingDiskStatus status,
                          std::optional<EndpointHit>* earliest) {
  const double time_tolerance =
      kRelativeTolerance * std::max(maximum_time, 1.0);
  if (!std::isfinite(root) || root <= time_tolerance ||
      root > maximum_time + time_tolerance) {
    return;
  }
  root = std::min(root, maximum_time);
  if (!earliest->has_value() || root < earliest->value().time) {
    *earliest = EndpointHit{root, status};
  }
}

void ConsiderEndpoint(double position, double velocity, double acceleration,
                      double boundary, double maximum_time,
                      RollingDiskStatus status,
                      std::optional<EndpointHit>* earliest) {
  const double linear = velocity;
  const double constant = position - boundary;
  if (acceleration == 0.0) {
    if (linear != 0.0) {
      ConsiderEndpointRoot(-constant / linear, maximum_time, status, earliest);
    }
    return;
  }

  const double quadratic = 0.5 * acceleration;
  double discriminant = linear * linear - 4.0 * quadratic * constant;
  const double discriminant_scale =
      linear * linear + std::abs(4.0 * quadratic * constant);
  if (discriminant < -kRelativeTolerance * std::max(discriminant_scale, 1.0)) {
    return;
  }
  discriminant = std::max(discriminant, 0.0);
  const double square_root = std::sqrt(discriminant);
  const double q = -0.5 * (linear + std::copysign(square_root, linear));
  if (q == 0.0) {
    ConsiderEndpointRoot(-linear / (2.0 * quadratic), maximum_time, status,
                         earliest);
    return;
  }
  ConsiderEndpointRoot(q / quadratic, maximum_time, status, earliest);
  ConsiderEndpointRoot(constant / q, maximum_time, status, earliest);
}

std::optional<EndpointHit> FindFirstEndpointHit(const RollingDiskConfig& config,
                                                const RollingDiskState& state,
                                                double acceleration,
                                                double maximum_time) {
  std::optional<EndpointHit> earliest;
  ConsiderEndpoint(state.distance_down_ramp_m, state.velocity_down_ramp_m_s,
                   acceleration, 0.0, maximum_time,
                   RollingDiskStatus::kReachedTop, &earliest);
  ConsiderEndpoint(state.distance_down_ramp_m, state.velocity_down_ramp_m_s,
                   acceleration, config.ramp_length_m, maximum_time,
                   RollingDiskStatus::kReachedBottom, &earliest);
  return earliest;
}

RollingDiskStatus GetInitialEndpointStatus(const RollingDiskConfig& config,
                                           const RollingDiskState& state,
                                           double acceleration) {
  if (state.distance_down_ramp_m == 0.0f &&
      (state.velocity_down_ramp_m_s < 0.0f ||
       (state.velocity_down_ramp_m_s == 0.0f && acceleration <= 0.0))) {
    return RollingDiskStatus::kReachedTop;
  }
  if (state.distance_down_ramp_m == config.ramp_length_m &&
      (state.velocity_down_ramp_m_s > 0.0f ||
       (state.velocity_down_ramp_m_s == 0.0f && acceleration >= 0.0))) {
    return RollingDiskStatus::kReachedBottom;
  }
  return RollingDiskStatus::kActive;
}

bool IsDerivedFinite(const RollingDiskDerived& derived) {
  const std::array values = {
      derived.moment_of_inertia_kg_m2,
      derived.tangential_external_force_n,
      derived.normal_force_n,
      derived.friction_force_n,
      derived.acceleration_down_ramp_m_s2,
      derived.angular_acceleration_rad_s2,
      derived.slip_velocity_m_s,
      derived.translational_kinetic_energy_j,
      derived.rotational_kinetic_energy_j,
      derived.potential_energy_j,
      derived.mechanical_energy_j,
      derived.accounted_energy_j,
  };
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

double CurrentFriction(const RollingDiskConfig& config, const Forces& forces,
                       const RollingDiskState& state) {
  if (state.status == RollingDiskStatus::kAirborne) {
    return 0.0;
  }
  if (state.contact_mode == RollingContactMode::kRolling) {
    return RequiredRollingFriction(config, forces);
  }
  const double slip = SlipVelocity(config, state);
  const double free_acceleration = forces.tangential_external / config.mass_kg;
  return SlidingFriction(
      config, forces,
      std::abs(slip) > kSlipTolerance ? slip : free_acceleration);
}

double CurrentAcceleration(const RollingDiskConfig& config,
                           const Forces& forces,
                           const RollingDiskState& state) {
  return (forces.tangential_external + CurrentFriction(config, forces, state)) /
         config.mass_kg;
}

RollingDiskDerived CalculateDerivedUnchecked(const RollingDiskConfig& config,
                                             const RollingDiskState& state) {
  const Forces forces = CalculateForces(config);
  const double slip = SlipVelocity(config, state);
  const double friction = CurrentFriction(config, forces, state);
  const double acceleration =
      (forces.tangential_external + friction) / config.mass_kg;
  const double angular_acceleration =
      -friction * config.radius_m / forces.moment_of_inertia;
  const double translational_energy = 0.5 * config.mass_kg *
                                      state.velocity_down_ramp_m_s *
                                      state.velocity_down_ramp_m_s;
  const double rotational_energy = 0.5 * forces.moment_of_inertia *
                                   state.angular_velocity_rad_s *
                                   state.angular_velocity_rad_s;
  const double potential_energy =
      forces.tangential_external *
      (config.ramp_length_m - state.distance_down_ramp_m);
  const double mechanical_energy =
      translational_energy + rotational_energy + potential_energy;
  const double accounted_energy = mechanical_energy + state.dissipated_energy_j;

  return {
      static_cast<float>(forces.moment_of_inertia),
      static_cast<float>(forces.tangential_external),
      static_cast<float>(forces.normal),
      static_cast<float>(friction),
      static_cast<float>(acceleration),
      static_cast<float>(angular_acceleration),
      static_cast<float>(slip),
      static_cast<float>(translational_energy),
      static_cast<float>(rotational_energy),
      static_cast<float>(potential_energy),
      static_cast<float>(mechanical_energy),
      static_cast<float>(accounted_energy),
  };
}

SegmentResult IntegrateSegment(const RollingDiskConfig& config,
                               const Forces& forces, double friction,
                               double delta_time, bool sliding,
                               RollingDiskState* state) {
  if (delta_time <= 0.0) {
    return {true, 0.0};
  }

  const double acceleration =
      (forces.tangential_external + friction) / config.mass_kg;
  const double angular_acceleration =
      -friction * config.radius_m / forces.moment_of_inertia;
  const std::optional<EndpointHit> endpoint =
      FindFirstEndpointHit(config, *state, acceleration, delta_time);
  const double elapsed = endpoint.has_value() ? endpoint->time : delta_time;
  const double slip = SlipVelocity(config, *state);
  const double slip_acceleration =
      acceleration -
      static_cast<double>(config.radius_m) * angular_acceleration;
  const double next_velocity =
      state->velocity_down_ramp_m_s + acceleration * elapsed;
  double next_angular_velocity =
      state->angular_velocity_rad_s + angular_acceleration * elapsed;
  if (!sliding) {
    next_angular_velocity = next_velocity / config.radius_m;
  }
  const double next_distance = state->distance_down_ramp_m +
                               state->velocity_down_ramp_m_s * elapsed +
                               0.5 * acceleration * elapsed * elapsed;
  const double next_angle = std::remainder(
      state->angle_radians + state->angular_velocity_rad_s * elapsed +
          0.5 * angular_acceleration * elapsed * elapsed,
      2.0 * kPi);
  const double next_time = state->time_seconds + elapsed;
  double dissipated = state->dissipated_energy_j;
  if (sliding) {
    const double loss = -friction * (slip * elapsed + 0.5 * slip_acceleration *
                                                          elapsed * elapsed);
    dissipated += std::max(loss, 0.0);
  }

  if (!FitsFloat(next_velocity) || !FitsFloat(next_angular_velocity) ||
      !FitsFloat(next_distance) || !FitsFloat(next_angle) ||
      !FitsFloat(next_time) || !FitsFloat(dissipated)) {
    return {false, 0.0};
  }

  state->velocity_down_ramp_m_s = static_cast<float>(next_velocity);
  state->angular_velocity_rad_s = static_cast<float>(next_angular_velocity);
  state->distance_down_ramp_m = static_cast<float>(next_distance);
  state->angle_radians = static_cast<float>(next_angle);
  state->time_seconds = static_cast<float>(next_time);
  state->dissipated_energy_j = static_cast<float>(dissipated);
  state->contact_mode =
      sliding ? RollingContactMode::kSliding : RollingContactMode::kRolling;
  if (endpoint.has_value()) {
    state->status = endpoint->status;
    state->distance_down_ramp_m =
        endpoint->status == RollingDiskStatus::kReachedTop
            ? 0.0f
            : config.ramp_length_m;
  }
  return {true, elapsed};
}

}  // namespace

const char* GetRollingDiskConfigError(const RollingDiskConfig& config) {
  const std::array values = {
      config.ramp_length_m,
      config.ramp_angle_degrees,
      config.mass_kg,
      config.radius_m,
      config.static_friction_coefficient,
      config.kinetic_friction_coefficient,
      config.initial_distance_down_ramp_m,
      config.initial_velocity_down_ramp_m_s,
      config.initial_angular_velocity_rad_s,
      config.gravity_m_s2,
      config.charge_c,
      config.electric_field_strength_n_c,
      config.electric_field_angle_degrees,
  };
  if (!std::all_of(values.begin(), values.end(), IsFinite)) {
    return "All rolling-disk inputs must be finite.";
  }
  if (!IsKnownKind(config.kind)) {
    return "The rolling-disk kind is invalid.";
  }
  if (config.ramp_length_m <= 0.0f || config.mass_kg < kMinimumMassOrRadius ||
      config.radius_m < kMinimumMassOrRadius) {
    return "Ramp length must be positive; mass and radius must be at least "
           "1e-6.";
  }
  if (config.ramp_angle_degrees < 0.0f || config.ramp_angle_degrees >= 90.0f) {
    return "Ramp angle must be in [0, 90) degrees.";
  }
  if (config.static_friction_coefficient < 0.0f ||
      config.kinetic_friction_coefficient < 0.0f ||
      config.kinetic_friction_coefficient >
          config.static_friction_coefficient ||
      config.static_friction_coefficient > kMaximumFrictionCoefficient) {
    return "Friction must satisfy 0 <= kinetic <= static <= 5.";
  }
  if (config.initial_distance_down_ramp_m < 0.0f ||
      config.initial_distance_down_ramp_m > config.ramp_length_m) {
    return "Initial distance must be within the ramp.";
  }
  if (config.gravity_m_s2 <= 0.0f ||
      config.electric_field_strength_n_c < 0.0f ||
      config.electric_field_angle_degrees < -180.0f ||
      config.electric_field_angle_degrees > 180.0f) {
    return "Gravity must be positive; field strength and angle are outside "
           "their valid ranges.";
  }

  const Forces forces = CalculateForces(config);
  const double initial_translational_energy =
      0.5 * config.mass_kg * config.initial_velocity_down_ramp_m_s *
      config.initial_velocity_down_ramp_m_s;
  const double initial_rotational_energy =
      0.5 * forces.moment_of_inertia * config.initial_angular_velocity_rad_s *
      config.initial_angular_velocity_rad_s;
  const double maximum_potential =
      forces.tangential_external * config.ramp_length_m;
  const double maximum_friction =
      config.static_friction_coefficient * std::abs(forces.normal);
  const double maximum_linear_acceleration =
      (std::abs(forces.tangential_external) + maximum_friction) /
      config.mass_kg;
  const double maximum_angular_acceleration =
      maximum_friction * config.radius_m / forces.moment_of_inertia;
  const double initial_slip =
      static_cast<double>(config.initial_velocity_down_ramp_m_s) -
      static_cast<double>(config.radius_m) *
          config.initial_angular_velocity_rad_s;
  const double initial_mechanical_energy =
      initial_translational_energy + initial_rotational_energy +
      forces.tangential_external *
          (config.ramp_length_m - config.initial_distance_down_ramp_m);
  if (!FitsFloat(forces.moment_of_inertia) || forces.moment_of_inertia <= 0.0 ||
      !FitsFloat(forces.tangential_external) || !FitsFloat(forces.normal) ||
      !FitsFloat(initial_translational_energy) ||
      !FitsFloat(initial_rotational_energy) || !FitsFloat(maximum_potential) ||
      !FitsFloat(maximum_friction) || !FitsFloat(maximum_linear_acceleration) ||
      !FitsFloat(maximum_angular_acceleration) || !FitsFloat(initial_slip) ||
      !FitsFloat(initial_mechanical_energy)) {
    return "The selected values exceed stable float calculations.";
  }
  return nullptr;
}

const char* GetRollingDiskStateError(const RollingDiskConfig& config,
                                     const RollingDiskState& state) {
  if (GetRollingDiskConfigError(config) != nullptr) {
    return "The rolling-disk configuration is invalid.";
  }
  const std::array values = {
      state.distance_down_ramp_m, state.velocity_down_ramp_m_s,
      state.angle_radians,        state.angular_velocity_rad_s,
      state.time_seconds,         state.dissipated_energy_j,
  };
  if (!std::all_of(values.begin(), values.end(), IsFinite)) {
    return "The rolling-disk state must contain only finite values.";
  }
  if (!IsKnownContactMode(state.contact_mode) || !IsKnownStatus(state.status)) {
    return "The rolling-disk state contains an invalid enum value.";
  }
  if (state.distance_down_ramp_m < -kPositionTolerance ||
      state.distance_down_ramp_m > config.ramp_length_m + kPositionTolerance ||
      state.time_seconds < 0.0f || state.dissipated_energy_j < 0.0f) {
    return "Rolling-disk distance, time, or dissipated energy is invalid.";
  }
  if (state.status == RollingDiskStatus::kReachedTop &&
      std::abs(state.distance_down_ramp_m) > kPositionTolerance) {
    return "A disk marked at the top must be at distance zero.";
  }
  if (state.status == RollingDiskStatus::kReachedBottom &&
      std::abs(state.distance_down_ramp_m - config.ramp_length_m) >
          kPositionTolerance) {
    return "A disk marked at the bottom must be at the ramp length.";
  }
  if (!IsDerivedFinite(CalculateDerivedUnchecked(config, state))) {
    return "A derived rolling-disk value is not finite.";
  }
  return nullptr;
}

RollingDiskState MakeInitialRollingDiskState(const RollingDiskConfig& config) {
  if (const char* error = GetRollingDiskConfigError(config)) {
    throw std::invalid_argument(error);
  }

  RollingDiskState state;
  state.distance_down_ramp_m = config.initial_distance_down_ramp_m;
  state.velocity_down_ramp_m_s = config.initial_velocity_down_ramp_m_s;
  state.angular_velocity_rad_s = config.initial_angular_velocity_rad_s;

  const Forces forces = CalculateForces(config);
  const double slip = SlipVelocity(config, state);
  if (std::abs(slip) <= kSlipTolerance && CanRoll(config, forces)) {
    state.angular_velocity_rad_s =
        state.velocity_down_ramp_m_s / config.radius_m;
    state.contact_mode = RollingContactMode::kRolling;
  } else {
    state.contact_mode = RollingContactMode::kSliding;
  }
  if (!HasSurfaceContact(forces)) {
    state.status = RollingDiskStatus::kAirborne;
  } else {
    state.status = GetInitialEndpointStatus(
        config, state, CurrentAcceleration(config, forces, state));
  }
  return state;
}

RollingDiskDerived CalculateRollingDiskDerived(const RollingDiskConfig& config,
                                               const RollingDiskState& state) {
  if (const char* error = GetRollingDiskStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

const RollingDiskState* FindRollingDiskState(
    const std::vector<RollingDiskState>& history, float time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const RollingDiskState& state, float target_time) {
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

bool StepRollingDisk(const RollingDiskConfig& config, float delta_time,
                     RollingDiskState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      GetRollingDiskConfigError(config) != nullptr ||
      GetRollingDiskStateError(config, *state) != nullptr) {
    return false;
  }
  if (state->status != RollingDiskStatus::kActive) {
    return false;
  }

  RollingDiskState next = *state;
  const auto commit = [&] {
    if (GetRollingDiskStateError(config, next) != nullptr) {
      return false;
    }
    *state = next;
    return true;
  };
  const Forces forces = CalculateForces(config);
  if (!HasSurfaceContact(forces)) {
    next.status = RollingDiskStatus::kAirborne;
    *state = next;
    return false;
  }
  next.status = GetInitialEndpointStatus(
      config, next, CurrentAcceleration(config, forces, next));
  if (next.status != RollingDiskStatus::kActive) {
    return commit();
  }

  const double free_acceleration = forces.tangential_external / config.mass_kg;
  const double static_friction = RequiredRollingFriction(config, forces);
  const bool can_roll = CanRoll(config, forces);
  double slip = SlipVelocity(config, next);
  double remaining_time = delta_time;

  if (std::abs(slip) <= kSlipTolerance && can_roll) {
    next.angular_velocity_rad_s = next.velocity_down_ramp_m_s / config.radius_m;
    const SegmentResult result = IntegrateSegment(
        config, forces, static_friction, remaining_time, false, &next);
    if (!result.valid) {
      return false;
    }
    if (next.status != RollingDiskStatus::kActive) {
      return commit();
    }
  } else {
    const double direction =
        std::abs(slip) > kSlipTolerance ? slip : free_acceleration;
    const double kinetic_friction = SlidingFriction(config, forces, direction);
    const double slip_acceleration =
        free_acceleration +
        EffectiveInverseMass(config, forces) * kinetic_friction;
    const double next_slip = slip + slip_acceleration * remaining_time;
    const bool crosses_zero = std::abs(slip) > kSlipTolerance &&
                              slip_acceleration != 0.0 &&
                              slip * next_slip <= 0.0;

    if (crosses_zero) {
      const double crossing_time =
          std::clamp(-slip / slip_acceleration, 0.0, remaining_time);
      const SegmentResult result = IntegrateSegment(
          config, forces, kinetic_friction, crossing_time, true, &next);
      if (!result.valid) {
        return false;
      }
      remaining_time -= result.elapsed;
      if (next.status != RollingDiskStatus::kActive) {
        return commit();
      }
      next.angular_velocity_rad_s =
          next.velocity_down_ramp_m_s / config.radius_m;
      slip = 0.0;
    }

    if (remaining_time > 0.0) {
      if ((crosses_zero || std::abs(slip) <= kSlipTolerance) && can_roll) {
        const SegmentResult result = IntegrateSegment(
            config, forces, static_friction, remaining_time, false, &next);
        if (!result.valid) {
          return false;
        }
      } else {
        const double remaining_direction =
            std::abs(slip) > kSlipTolerance ? slip : free_acceleration;
        const double remaining_friction =
            SlidingFriction(config, forces, remaining_direction);
        const SegmentResult result = IntegrateSegment(
            config, forces, remaining_friction, remaining_time, true, &next);
        if (!result.valid) {
          return false;
        }
      }
    }
  }
  return commit();
}

}  // namespace tiny2d::sandbox
