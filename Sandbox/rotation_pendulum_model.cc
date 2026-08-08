#include "rotation_pendulum_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace tiny2d::sandbox {
namespace {

constexpr float kPendulumMaximumAnglePerStep = 0.25f;

bool IsPendulumDriveActive(const PendulumConfig& config) {
  return config.drive_enabled && config.drive_torque_amplitude_n_m > 0.0f;
}

float GetPendulumGravityTorqueScale(const PendulumConfig& config) {
  return config.gravity_m_s2 *
         (config.rod_mass_kg * config.rod_length_m * 0.5f +
          config.counterweight_mass_kg * config.counterweight_distance_m);
}

float GetPendulumElectricTorqueScale(const PendulumConfig& config) {
  if (!config.electric_field_enabled || !config.counterweight_charged) {
    return 0.0f;
  }
  return config.counterweight_charge_c * config.counterweight_distance_m *
         config.electric_field_strength_n_c;
}

float GetPendulumRestoringTorqueMagnitude(const PendulumConfig& config) {
  const float gravity_scale = GetPendulumGravityTorqueScale(config);
  const float electric_scale = GetPendulumElectricTorqueScale(config);
  const float field_angle = GetPendulumFieldAngle(config);
  return std::hypot(-gravity_scale + electric_scale * std::sin(field_angle),
                    electric_scale * std::cos(field_angle));
}

}  // namespace

float GetPendulumFieldAngle(const PendulumConfig& config) {
  return config.electric_field_angle_degrees * kPendulumDegreesToRadians;
}

float GetPendulumMomentOfInertia(const PendulumConfig& config) {
  return config.rod_mass_kg * config.rod_length_m * config.rod_length_m / 3.0f +
         config.counterweight_mass_kg * config.counterweight_distance_m *
             config.counterweight_distance_m;
}

float GetSmallAnglePeriod(const PendulumConfig& config) {
  const float restoring_torque = GetPendulumRestoringTorqueMagnitude(config);
  const float moment_of_inertia = GetPendulumMomentOfInertia(config);
  if (!std::isfinite(restoring_torque) || restoring_torque <= 0.0f ||
      !std::isfinite(moment_of_inertia) || moment_of_inertia <= 0.0f) {
    return std::numeric_limits<float>::infinity();
  }
  return 2.0f * kPendulumPi * std::sqrt(moment_of_inertia / restoring_torque);
}

PendulumDerived CalculatePendulumDerived(const PendulumConfig& config,
                                         const PendulumState& state) {
  PendulumDerived derived;
  derived.moment_of_inertia_kg_m2 = GetPendulumMomentOfInertia(config);

  const float gravity_scale = GetPendulumGravityTorqueScale(config);
  const float electric_scale = GetPendulumElectricTorqueScale(config);
  const float field_angle = GetPendulumFieldAngle(config);
  derived.gravity_torque_n_m = -gravity_scale * std::sin(state.angle_radians);
  derived.electric_torque_n_m =
      electric_scale * std::cos(state.angle_radians - field_angle);
  derived.damping_torque_n_m =
      config.damping_enabled
          ? -config.damping_coefficient_n_m_s * state.angular_velocity_rad_s
          : 0.0f;
  if (IsPendulumDriveActive(config)) {
    const double drive_phase =
        static_cast<double>(config.drive_angular_frequency_rad_s) *
        state.time_seconds;
    derived.driving_torque_n_m = config.drive_torque_amplitude_n_m *
                                 static_cast<float>(std::cos(drive_phase));
    derived.driving_power_w =
        derived.driving_torque_n_m * state.angular_velocity_rad_s;
  }
  derived.total_torque_n_m =
      derived.gravity_torque_n_m + derived.electric_torque_n_m +
      derived.damping_torque_n_m + derived.driving_torque_n_m;
  derived.angular_acceleration_rad_s2 =
      derived.total_torque_n_m / derived.moment_of_inertia_kg_m2;

  derived.kinetic_energy_j = 0.5f * derived.moment_of_inertia_kg_m2 *
                             state.angular_velocity_rad_s *
                             state.angular_velocity_rad_s;
  derived.gravitational_potential_energy_j =
      gravity_scale * (1.0f - std::cos(state.angle_radians));
  derived.electric_potential_energy_j =
      -electric_scale * std::sin(state.angle_radians - field_angle);
  derived.total_energy_j = derived.kinetic_energy_j +
                           derived.gravitational_potential_energy_j +
                           derived.electric_potential_energy_j;
  return derived;
}

bool IsPendulumDerivedFinite(const PendulumDerived& derived) {
  const std::array<float, 12> values = {
      derived.moment_of_inertia_kg_m2,
      derived.gravity_torque_n_m,
      derived.electric_torque_n_m,
      derived.damping_torque_n_m,
      derived.driving_torque_n_m,
      derived.driving_power_w,
      derived.total_torque_n_m,
      derived.angular_acceleration_rad_s2,
      derived.kinetic_energy_j,
      derived.gravitational_potential_energy_j,
      derived.electric_potential_energy_j,
      derived.total_energy_j,
  };
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

PendulumState MakeInitialPendulumState(const PendulumConfig& config) {
  PendulumState state;
  state.angle_radians =
      config.initial_angle_degrees * kPendulumDegreesToRadians;
  state.angular_velocity_rad_s = config.initial_angular_velocity_rad_s;
  state.angular_acceleration_rad_s2 =
      CalculatePendulumDerived(config, state).angular_acceleration_rad_s2;
  return state;
}

const PendulumState* FindPendulumState(
    const std::vector<PendulumState>& history, double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const PendulumState& state, double target_time) {
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

const char* GetPendulumStateError(const PendulumConfig& config,
                                  const PendulumState& state) {
  const std::array<float, 3> values = {
      state.angle_radians,
      state.angular_velocity_rad_s,
      state.angular_acceleration_rad_s2,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); }) ||
      !std::isfinite(state.time_seconds)) {
    return "The pendulum state contains NaN or infinity.";
  }
  if (state.time_seconds < 0.0) {
    return "The pendulum time cannot be negative.";
  }
  if (std::abs(state.angular_velocity_rad_s) * kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep ||
      std::abs(state.angular_acceleration_rad_s2) * kPendulumPhysicsStep *
              kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep) {
    return "The pendulum is moving too fast for the fixed physics step.";
  }
  if (!IsPendulumDerivedFinite(CalculatePendulumDerived(config, state))) {
    return "A derived pendulum value is NaN or infinite.";
  }
  return nullptr;
}

const char* GetPendulumConfigError(const PendulumConfig& config) {
  const std::array<float, 13> values = {
      config.rod_length_m,
      config.rod_mass_kg,
      config.counterweight_mass_kg,
      config.counterweight_distance_m,
      config.counterweight_charge_c,
      config.initial_angle_degrees,
      config.initial_angular_velocity_rad_s,
      config.gravity_m_s2,
      config.electric_field_strength_n_c,
      config.electric_field_angle_degrees,
      config.damping_coefficient_n_m_s,
      config.drive_torque_amplitude_n_m,
      config.drive_angular_frequency_rad_s,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All inputs must be finite numbers; NaN and infinity are invalid.";
  }
  if (config.rod_length_m < kMinimumRodLength ||
      config.rod_length_m > kMaximumRodLength) {
    return "Rod length must be between 0.1 m and 20 m.";
  }
  if (config.rod_mass_kg < kMinimumMass || config.rod_mass_kg > kMaximumMass ||
      config.counterweight_mass_kg < kMinimumMass ||
      config.counterweight_mass_kg > kMaximumMass) {
    return "Rod and counterweight masses must be positive and at most 1000 kg.";
  }
  if (config.counterweight_distance_m < 0.0f ||
      config.counterweight_distance_m > config.rod_length_m) {
    return "Counterweight distance r must be between the pivot and rod end.";
  }
  if (std::abs(config.counterweight_charge_c) > kMaximumChargeMagnitude) {
    return "Counterweight charge must be between -1000 C and 1000 C.";
  }
  if (config.initial_angle_degrees < -180.0f ||
      config.initial_angle_degrees > 180.0f ||
      std::abs(config.initial_angular_velocity_rad_s) >
          kMaximumInitialAngularSpeed) {
    return "Initial angle or angular velocity is outside the supported range.";
  }
  if (config.gravity_m_s2 != 9.8f && config.gravity_m_s2 != 10.0f) {
    return "Gravity must be either 9.8 or 10 m/s^2.";
  }
  if (config.electric_field_strength_n_c < 0.0f ||
      config.electric_field_strength_n_c > kMaximumElectricField ||
      config.electric_field_angle_degrees < -180.0f ||
      config.electric_field_angle_degrees > 180.0f) {
    return "Electric field strength or angle is outside the supported range.";
  }
  if (config.damping_coefficient_n_m_s < 0.0f ||
      config.damping_coefficient_n_m_s > kMaximumDamping) {
    return "Rotational damping must be between 0 and 1000 N*m*s/rad.";
  }
  if (config.drive_torque_amplitude_n_m < 0.0f ||
      config.drive_torque_amplitude_n_m > kMaximumDriveTorque ||
      config.drive_angular_frequency_rad_s < 0.0f ||
      config.drive_angular_frequency_rad_s > kMaximumDriveAngularFrequency) {
    return "Drive torque or angular frequency is outside the supported range.";
  }

  const float moment_of_inertia = GetPendulumMomentOfInertia(config);
  const float restoring_torque = GetPendulumRestoringTorqueMagnitude(config);
  const float small_angle_period = GetSmallAnglePeriod(config);
  const PendulumState initial_state = MakeInitialPendulumState(config);
  if (!std::isfinite(moment_of_inertia) || moment_of_inertia <= 0.0f ||
      !std::isfinite(restoring_torque) || restoring_torque <= 0.000001f ||
      !std::isfinite(small_angle_period) ||
      !IsPendulumDerivedFinite(
          CalculatePendulumDerived(config, initial_state))) {
    return "The selected values do not produce finite pendulum quantities.";
  }

  const float maximum_angular_speed =
      std::sqrt(config.initial_angular_velocity_rad_s *
                    config.initial_angular_velocity_rad_s +
                4.0f * restoring_torque / moment_of_inertia);
  const float maximum_angular_acceleration =
      restoring_torque / moment_of_inertia +
      (config.damping_enabled ? config.damping_coefficient_n_m_s *
                                    maximum_angular_speed / moment_of_inertia
                              : 0.0f) +
      (IsPendulumDriveActive(config)
           ? config.drive_torque_amplitude_n_m / moment_of_inertia
           : 0.0f);
  if (!std::isfinite(maximum_angular_speed) ||
      !std::isfinite(maximum_angular_acceleration) ||
      maximum_angular_speed * kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep ||
      maximum_angular_acceleration * kPendulumPhysicsStep *
              kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep ||
      (IsPendulumDriveActive(config) &&
       config.drive_angular_frequency_rad_s * kPendulumPhysicsStep >
           kPendulumMaximumAnglePerStep) ||
      (config.damping_enabled && config.damping_coefficient_n_m_s *
                                         kPendulumPhysicsStep /
                                         moment_of_inertia >=
                                     1.0f)) {
    return "This parameter combination is too fast or stiff for stable "
           "integration.";
  }
  return nullptr;
}

bool StepPendulum(const PendulumConfig& config, float delta_time,
                  PendulumState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f) {
    return false;
  }
  if (GetPendulumConfigError(config) != nullptr ||
      GetPendulumStateError(config, *state) != nullptr) {
    return false;
  }
  const PendulumDerived before = CalculatePendulumDerived(config, *state);
  if (!IsPendulumDerivedFinite(before) ||
      std::abs(state->angular_velocity_rad_s) * delta_time >
          kPendulumMaximumAnglePerStep ||
      std::abs(before.angular_acceleration_rad_s2) * delta_time * delta_time >
          kPendulumMaximumAnglePerStep ||
      (IsPendulumDriveActive(config) &&
       config.drive_angular_frequency_rad_s * delta_time >
           kPendulumMaximumAnglePerStep)) {
    return false;
  }

  PendulumState next = *state;
  next.angular_velocity_rad_s +=
      before.angular_acceleration_rad_s2 * delta_time;
  next.angle_radians = std::remainder(
      next.angle_radians + next.angular_velocity_rad_s * delta_time,
      2.0f * kPendulumPi);
  next.time_seconds += static_cast<double>(delta_time);
  next.angular_acceleration_rad_s2 =
      CalculatePendulumDerived(config, next).angular_acceleration_rad_s2;
  if (GetPendulumStateError(config, next) != nullptr) {
    return false;
  }
  *state = next;
  return true;
}

}  // namespace tiny2d::sandbox
