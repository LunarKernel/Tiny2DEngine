#include "lorentz_particle_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace tiny2d::sandbox {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kMinimumMassKg = 0.01;
constexpr double kMaximumMassKg = 1000.0;
constexpr double kMaximumChargeMagnitudeC = 1000.0;
constexpr double kMaximumInitialSpeedMps = 100.0;
constexpr double kMaximumElectricFieldNc = 1000000.0;
constexpr double kMaximumMagneticFieldT = 100.0;
constexpr double kMaximumCyclotronFrequencyRadS = 60.0;
constexpr double kMaximumElectricAccelerationMps2 = 10000.0;
constexpr double kSmallAngle = 0.001;

struct FieldValues {
  double electric_x;
  double electric_y;
  double magnetic_z;
};

struct AnalyticCoefficients {
  double cosine;
  double sine;
  double sine_over_theta;
  double one_minus_cosine_over_theta;
  double one_minus_cosine_over_theta_squared;
  double theta_minus_sine_over_theta_squared;
};

bool IsFinite(double value) { return std::isfinite(value); }

bool IsKnownStatus(LorentzParticleStatus status) {
  switch (status) {
    case LorentzParticleStatus::kActive:
    case LorentzParticleStatus::kOutOfBounds:
      return true;
  }
  return false;
}

bool IsInsideExperiment(double x, double y) {
  return x >= kLorentzMinimumX && x <= kLorentzMaximumX &&
         y >= kLorentzMinimumY && y <= kLorentzMaximumY;
}

FieldValues GetFields(const LorentzParticleConfig& config) {
  double electric_x = 0.0;
  double electric_y = 0.0;
  if (config.electric_field_enabled) {
    const double angle =
        config.electric_field_angle_degrees * kDegreesToRadians;
    electric_x = config.electric_field_strength_n_c * std::cos(angle);
    electric_y = config.electric_field_strength_n_c * std::sin(angle);
  }
  const double magnetic_z =
      config.magnetic_field_enabled ? config.magnetic_field_z_t : 0.0;
  return {electric_x, electric_y, magnetic_z};
}

AnalyticCoefficients GetAnalyticCoefficients(double theta) {
  if (std::abs(theta) < kSmallAngle) {
    const double theta_squared = theta * theta;
    const double theta_cubed = theta_squared * theta;
    const double theta_fourth = theta_squared * theta_squared;
    const double theta_fifth = theta_fourth * theta;
    return {
        1.0 - theta_squared / 2.0 + theta_fourth / 24.0,
        theta - theta_cubed / 6.0 + theta_fifth / 120.0,
        1.0 - theta_squared / 6.0 + theta_fourth / 120.0,
        theta / 2.0 - theta_cubed / 24.0 + theta_fifth / 720.0,
        0.5 - theta_squared / 24.0 + theta_fourth / 720.0,
        theta / 6.0 - theta_cubed / 120.0 + theta_fifth / 5040.0,
    };
  }

  const double cosine = std::cos(theta);
  const double sine = std::sin(theta);
  return {cosine,
          sine,
          sine / theta,
          (1.0 - cosine) / theta,
          (1.0 - cosine) / (theta * theta),
          (theta - sine) / (theta * theta)};
}

LorentzParticleDerived CalculateDerivedUnchecked(
    const LorentzParticleConfig& config, const LorentzParticleState& state) {
  const FieldValues fields = GetFields(config);
  const double charge_over_mass = config.charge_c / config.mass_kg;
  const double acceleration_x =
      charge_over_mass *
      (fields.electric_x + state.velocity_y_m_s * fields.magnetic_z);
  const double acceleration_y =
      charge_over_mass *
      (fields.electric_y - state.velocity_x_m_s * fields.magnetic_z);
  const double speed = std::hypot(state.velocity_x_m_s, state.velocity_y_m_s);
  const double kinetic_energy = 0.5 * config.mass_kg * speed * speed;
  const double electric_potential_energy =
      -config.charge_c * (fields.electric_x * (state.x_m - config.initial_x_m) +
                          fields.electric_y * (state.y_m - config.initial_y_m));
  const double total_energy = kinetic_energy + electric_potential_energy;
  const double cyclotron_frequency = charge_over_mass * fields.magnetic_z;

  LorentzParticleDerived derived{
      acceleration_x,
      acceleration_y,
      speed,
      kinetic_energy,
      electric_potential_energy,
      total_energy,
      cyclotron_frequency,
  };
  if (config.charge_c != 0.0 && fields.magnetic_z != 0.0) {
    derived.has_cyclotron_data = true;
    derived.cyclotron_period_s = 2.0 * kPi / std::abs(cyclotron_frequency);
    derived.drift_velocity_x_m_s = fields.electric_y / fields.magnetic_z;
    derived.drift_velocity_y_m_s = -fields.electric_x / fields.magnetic_z;
    const double relative_velocity_x =
        state.velocity_x_m_s - derived.drift_velocity_x_m_s;
    const double relative_velocity_y =
        state.velocity_y_m_s - derived.drift_velocity_y_m_s;
    derived.larmor_radius_m =
        std::hypot(relative_velocity_x, relative_velocity_y) /
        std::abs(cyclotron_frequency);
  }
  return derived;
}

bool IsDerivedFinite(const LorentzParticleDerived& derived) {
  const std::array values = {
      derived.acceleration_x_m_s2,
      derived.acceleration_y_m_s2,
      derived.speed_m_s,
      derived.kinetic_energy_j,
      derived.electric_potential_energy_j,
      derived.total_energy_j,
      derived.cyclotron_angular_frequency_rad_s,
      derived.cyclotron_period_s,
      derived.drift_velocity_x_m_s,
      derived.drift_velocity_y_m_s,
      derived.larmor_radius_m,
  };
  return std::all_of(values.begin(), values.end(), IsFinite);
}

LorentzParticleState AdvanceAnalyticallyUnchecked(
    const LorentzParticleConfig& config, const LorentzParticleState& state,
    double delta_time) {
  const FieldValues fields = GetFields(config);
  const double charge_over_mass = config.charge_c / config.mass_kg;
  const double acceleration_x = charge_over_mass * fields.electric_x;
  const double acceleration_y = charge_over_mass * fields.electric_y;
  const double angular_frequency = charge_over_mass * fields.magnetic_z;
  const double theta = angular_frequency * delta_time;
  const AnalyticCoefficients coefficients = GetAnalyticCoefficients(theta);
  const double sine_time = delta_time * coefficients.sine_over_theta;
  const double cosine_time =
      delta_time * coefficients.one_minus_cosine_over_theta;
  const double cosine_time_squared =
      delta_time * delta_time *
      coefficients.one_minus_cosine_over_theta_squared;
  const double sine_time_squared =
      delta_time * delta_time *
      coefficients.theta_minus_sine_over_theta_squared;

  LorentzParticleState next = state;
  next.velocity_x_m_s = coefficients.cosine * state.velocity_x_m_s +
                        coefficients.sine * state.velocity_y_m_s +
                        sine_time * acceleration_x +
                        cosine_time * acceleration_y;
  next.velocity_y_m_s = coefficients.cosine * state.velocity_y_m_s -
                        coefficients.sine * state.velocity_x_m_s +
                        sine_time * acceleration_y -
                        cosine_time * acceleration_x;
  next.x_m = state.x_m + sine_time * state.velocity_x_m_s +
             cosine_time * state.velocity_y_m_s +
             cosine_time_squared * acceleration_x +
             sine_time_squared * acceleration_y;
  next.y_m = state.y_m + sine_time * state.velocity_y_m_s -
             cosine_time * state.velocity_x_m_s +
             cosine_time_squared * acceleration_y -
             sine_time_squared * acceleration_x;
  next.time_seconds = state.time_seconds + delta_time;
  return next;
}

void AddCandidateTime(double time, double delta_time,
                      std::array<double, 16>* candidates,
                      std::size_t* candidate_count) {
  if (time > 0.0 && time <= delta_time &&
      *candidate_count < candidates->size()) {
    (*candidates)[*candidate_count] = time;
    ++*candidate_count;
  }
}

void AddTrigonometricVelocityRoots(double quadratic, double linear,
                                   double offset, double angular_frequency,
                                   double delta_time,
                                   std::array<double, 16>* candidates,
                                   std::size_t* candidate_count) {
  const double scale =
      std::max({std::abs(quadratic), std::abs(linear), std::abs(offset)});
  if (scale == 0.0) {
    return;
  }

  quadratic /= scale;
  linear /= scale;
  offset /= scale;
  if (quadratic == 0.0) {
    if (linear != 0.0) {
      const double half_angle = -offset / linear;
      AddCandidateTime(2.0 * std::atan(half_angle) / angular_frequency,
                       delta_time, candidates, candidate_count);
    }
    return;
  }

  const double discriminant = linear * linear - 4.0 * quadratic * offset;
  if (discriminant < 0.0) {
    return;
  }
  const double root = std::sqrt(discriminant);
  if (root == 0.0) {
    const double half_angle = -linear / (2.0 * quadratic);
    AddCandidateTime(2.0 * std::atan(half_angle) / angular_frequency,
                     delta_time, candidates, candidate_count);
    return;
  }

  const double stable_numerator = -0.5 * (linear + std::copysign(root, linear));
  const std::array half_angles = {stable_numerator / quadratic,
                                  offset / stable_numerator};
  for (double half_angle : half_angles) {
    AddCandidateTime(2.0 * std::atan(half_angle) / angular_frequency,
                     delta_time, candidates, candidate_count);
  }
}

bool FindOutOfBoundsSample(const LorentzParticleConfig& config,
                           const LorentzParticleState& state, double delta_time,
                           LorentzParticleState* out_of_bounds) {
  std::array<double, 16> candidates{};
  std::size_t candidate_count = 0;
  AddCandidateTime(delta_time, delta_time, &candidates, &candidate_count);

  const FieldValues fields = GetFields(config);
  const double charge_over_mass = config.charge_c / config.mass_kg;
  const double acceleration_x = charge_over_mass * fields.electric_x;
  const double acceleration_y = charge_over_mass * fields.electric_y;
  const double angular_frequency = charge_over_mass * fields.magnetic_z;
  if (angular_frequency == 0.0) {
    if (acceleration_x != 0.0) {
      AddCandidateTime(-state.velocity_x_m_s / acceleration_x, delta_time,
                       &candidates, &candidate_count);
    }
    if (acceleration_y != 0.0) {
      AddCandidateTime(-state.velocity_y_m_s / acceleration_y, delta_time,
                       &candidates, &candidate_count);
    }
  } else {
    AddTrigonometricVelocityRoots(
        std::fma(-angular_frequency, state.velocity_x_m_s,
                 2.0 * acceleration_y),
        2.0 * std::fma(angular_frequency, state.velocity_y_m_s, acceleration_x),
        angular_frequency * state.velocity_x_m_s, angular_frequency, delta_time,
        &candidates, &candidate_count);
    AddTrigonometricVelocityRoots(
        std::fma(-angular_frequency, state.velocity_y_m_s,
                 -2.0 * acceleration_x),
        2.0 *
            std::fma(-angular_frequency, state.velocity_x_m_s, acceleration_y),
        angular_frequency * state.velocity_y_m_s, angular_frequency, delta_time,
        &candidates, &candidate_count);
  }

  std::sort(candidates.begin(), candidates.begin() + candidate_count);
  for (std::size_t index = 0; index < candidate_count; ++index) {
    LorentzParticleState sample =
        AdvanceAnalyticallyUnchecked(config, state, candidates[index]);
    if (!IsInsideExperiment(sample.x_m, sample.y_m)) {
      sample.status = LorentzParticleStatus::kOutOfBounds;
      *out_of_bounds = sample;
      return true;
    }
  }
  return false;
}

}  // namespace

const char* GetLorentzParticleConfigError(const LorentzParticleConfig& config) {
  const std::array values = {
      config.mass_kg,
      config.charge_c,
      config.initial_x_m,
      config.initial_y_m,
      config.initial_speed_m_s,
      config.initial_velocity_angle_degrees,
      config.electric_field_strength_n_c,
      config.electric_field_angle_degrees,
      config.magnetic_field_z_t,
  };
  if (!std::all_of(values.begin(), values.end(), IsFinite)) {
    return "All OrbitLab inputs must be finite.";
  }
  if (config.mass_kg < kMinimumMassKg || config.mass_kg > kMaximumMassKg) {
    return "Mass must be in [0.01, 1000] kg.";
  }
  if (std::abs(config.charge_c) > kMaximumChargeMagnitudeC) {
    return "Charge must be in [-1000, 1000] C.";
  }
  if (!IsInsideExperiment(config.initial_x_m, config.initial_y_m)) {
    return "The initial position must be inside the 20 m by 12 m area.";
  }
  if (config.initial_speed_m_s < 0.0 ||
      config.initial_speed_m_s > kMaximumInitialSpeedMps) {
    return "Initial speed must be in [0, 100] m/s.";
  }
  if (config.initial_velocity_angle_degrees < -180.0 ||
      config.initial_velocity_angle_degrees > 180.0 ||
      config.electric_field_angle_degrees < -180.0 ||
      config.electric_field_angle_degrees > 180.0) {
    return "Velocity and electric-field angles must be in [-180, 180].";
  }
  if (config.electric_field_strength_n_c < 0.0 ||
      config.electric_field_strength_n_c > kMaximumElectricFieldNc) {
    return "Electric-field strength must be in [0, 1e6] N/C.";
  }
  if (std::abs(config.magnetic_field_z_t) > kMaximumMagneticFieldT) {
    return "Magnetic field Bz must be in [-100, 100] T.";
  }

  const FieldValues fields = GetFields(config);
  const double electric_strength =
      config.electric_field_enabled ? config.electric_field_strength_n_c : 0.0;
  const double electric_force = std::abs(config.charge_c) * electric_strength;
  if (electric_force > kMaximumElectricAccelerationMps2 * config.mass_kg) {
    return "The selected qE/m exceeds the 10000 m/s^2 display limit.";
  }
  const double magnetic_charge_product =
      std::abs(config.charge_c * fields.magnetic_z);
  if (magnetic_charge_product >
      kMaximumCyclotronFrequencyRadS * config.mass_kg) {
    return "The selected |qBz/m| exceeds the 60 rad/s display limit.";
  }

  const double velocity_angle =
      config.initial_velocity_angle_degrees * kDegreesToRadians;
  LorentzParticleState initial_state{
      config.initial_x_m,
      config.initial_y_m,
      config.initial_speed_m_s * std::cos(velocity_angle),
      config.initial_speed_m_s * std::sin(velocity_angle),
  };
  if (!IsDerivedFinite(CalculateDerivedUnchecked(config, initial_state))) {
    return "The selected values exceed stable OrbitLab calculations.";
  }
  return nullptr;
}

const char* GetLorentzParticleStateError(const LorentzParticleConfig& config,
                                         const LorentzParticleState& state) {
  if (GetLorentzParticleConfigError(config) != nullptr) {
    return "The OrbitLab configuration is invalid.";
  }
  const std::array values = {state.x_m, state.y_m, state.velocity_x_m_s,
                             state.velocity_y_m_s, state.time_seconds};
  if (!std::all_of(values.begin(), values.end(), IsFinite)) {
    return "OrbitLab state values must be finite.";
  }
  if (!IsKnownStatus(state.status) || state.time_seconds < 0.0) {
    return "OrbitLab state status or time is invalid.";
  }
  const bool inside = IsInsideExperiment(state.x_m, state.y_m);
  if ((state.status == LorentzParticleStatus::kActive && !inside) ||
      (state.status == LorentzParticleStatus::kOutOfBounds && inside)) {
    return "OrbitLab state status does not match its position.";
  }
  if (!IsDerivedFinite(CalculateDerivedUnchecked(config, state))) {
    return "A derived OrbitLab value is not finite.";
  }
  return nullptr;
}

LorentzParticleState MakeInitialLorentzParticleState(
    const LorentzParticleConfig& config) {
  if (const char* error = GetLorentzParticleConfigError(config)) {
    throw std::invalid_argument(error);
  }
  const double angle =
      config.initial_velocity_angle_degrees * kDegreesToRadians;
  return {config.initial_x_m,
          config.initial_y_m,
          config.initial_speed_m_s * std::cos(angle),
          config.initial_speed_m_s * std::sin(angle),
          0.0,
          LorentzParticleStatus::kActive};
}

LorentzParticleDerived CalculateLorentzParticleDerived(
    const LorentzParticleConfig& config, const LorentzParticleState& state) {
  if (const char* error = GetLorentzParticleStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepLorentzParticle(const LorentzParticleConfig& config, double delta_time,
                         LorentzParticleState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0 ||
      delta_time > kLorentzPhysicsStep ||
      GetLorentzParticleStateError(config, *state) != nullptr ||
      state->status != LorentzParticleStatus::kActive) {
    return false;
  }

  LorentzParticleState next;
  if (!FindOutOfBoundsSample(config, *state, delta_time, &next)) {
    next = AdvanceAnalyticallyUnchecked(config, *state, delta_time);
  }
  if (GetLorentzParticleStateError(config, next) != nullptr) {
    return false;
  }
  *state = next;
  return true;
}

const LorentzParticleState* FindLorentzParticleState(
    const std::vector<LorentzParticleState>& history, double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next = std::lower_bound(
      history.begin(), history.end(), time_seconds,
      [](const LorentzParticleState& state, double target_time) {
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
