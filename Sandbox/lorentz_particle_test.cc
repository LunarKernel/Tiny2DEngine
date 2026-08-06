#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

#include "lorentz_particle_model.h"

namespace {

using tiny2d::sandbox::CalculateLorentzParticleDerived;
using tiny2d::sandbox::FindLorentzParticleState;
using tiny2d::sandbox::GetLorentzParticleConfigError;
using tiny2d::sandbox::GetLorentzParticleStateError;
using tiny2d::sandbox::kLorentzPhysicsStep;
using tiny2d::sandbox::LorentzParticleConfig;
using tiny2d::sandbox::LorentzParticleDerived;
using tiny2d::sandbox::LorentzParticleState;
using tiny2d::sandbox::LorentzParticleStatus;
using tiny2d::sandbox::MakeInitialLorentzParticleState;
using tiny2d::sandbox::StepLorentzParticle;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-10;

[[noreturn]] void FailCheck(const char* expression, const char* file,
                            int line) {
  std::cerr << file << ':' << line << ": CHECK failed: " << expression << '\n';
  std::abort();
}

#define CHECK(expression)                         \
  do {                                            \
    if (!(expression)) {                          \
      FailCheck(#expression, __FILE__, __LINE__); \
    }                                             \
  } while (false)

bool Near(double actual, double expected, double tolerance = kTolerance) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

bool SameState(const LorentzParticleState& a, const LorentzParticleState& b) {
  return a.x_m == b.x_m && a.y_m == b.y_m &&
         a.velocity_x_m_s == b.velocity_x_m_s &&
         a.velocity_y_m_s == b.velocity_y_m_s &&
         a.time_seconds == b.time_seconds && a.status == b.status;
}

void AdvanceFor(const LorentzParticleConfig& config, double duration,
                LorentzParticleState* state) {
  while (duration > 0.0) {
    const double step = std::min(duration, kLorentzPhysicsStep);
    CHECK(StepLorentzParticle(config, step, state));
    duration -= step;
    if (duration < 1e-14) {
      duration = 0.0;
    }
  }
}

void TestPureElectricFieldMatchesKinematics() {
  LorentzParticleConfig config;
  config.mass_kg = 2.0;
  config.charge_c = 4.0;
  config.initial_x_m = -5.0;
  config.initial_y_m = -1.0;
  config.initial_speed_m_s = 3.0;
  config.initial_velocity_angle_degrees = 90.0;
  config.electric_field_strength_n_c = 2.0;
  config.electric_field_angle_degrees = 0.0;
  config.magnetic_field_enabled = false;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  const double initial_energy =
      CalculateLorentzParticleDerived(config, state).total_energy_j;

  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  const double acceleration = 4.0;
  CHECK(Near(state.x_m, -5.0 + 0.5 * acceleration * kLorentzPhysicsStep *
                                   kLorentzPhysicsStep));
  CHECK(Near(state.y_m, -1.0 + 3.0 * kLorentzPhysicsStep));
  CHECK(Near(state.velocity_x_m_s, acceleration * kLorentzPhysicsStep));
  CHECK(Near(state.velocity_y_m_s, 3.0));
  const LorentzParticleDerived derived =
      CalculateLorentzParticleDerived(config, state);
  CHECK(Near(derived.acceleration_x_m_s2, acceleration));
  CHECK(Near(derived.acceleration_y_m_s2, 0.0));
  CHECK(Near(derived.total_energy_j, initial_energy));
  CHECK(!derived.has_cyclotron_data);
}

void TestPureMagneticOrbitAndEnergy() {
  LorentzParticleConfig config;
  config.initial_x_m = 0.0;
  config.initial_y_m = 0.0;
  config.initial_speed_m_s = 2.0;
  config.initial_velocity_angle_degrees = 0.0;
  config.electric_field_enabled = false;
  config.magnetic_field_z_t = 1.0;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  const LorentzParticleDerived initial =
      CalculateLorentzParticleDerived(config, state);
  CHECK(initial.has_cyclotron_data);
  CHECK(Near(initial.cyclotron_angular_frequency_rad_s, 1.0));
  CHECK(Near(initial.cyclotron_period_s, 2.0 * kPi));
  CHECK(Near(initial.larmor_radius_m, 2.0));

  AdvanceFor(config, initial.cyclotron_period_s, &state);
  const LorentzParticleDerived final =
      CalculateLorentzParticleDerived(config, state);
  CHECK(Near(state.x_m, 0.0, 2e-9));
  CHECK(Near(state.y_m, 0.0, 2e-9));
  CHECK(Near(state.velocity_x_m_s, 2.0, 2e-9));
  CHECK(Near(state.velocity_y_m_s, 0.0, 2e-9));
  CHECK(Near(final.kinetic_energy_j, initial.kinetic_energy_j, 2e-9));
}

void TestCrossedFieldDriftAfterOnePeriod() {
  LorentzParticleConfig config;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  const LorentzParticleState initial_state = state;
  const LorentzParticleDerived derived =
      CalculateLorentzParticleDerived(config, state);
  CHECK(derived.has_cyclotron_data);
  CHECK(Near(derived.drift_velocity_x_m_s, 0.3));
  CHECK(Near(derived.drift_velocity_y_m_s, 0.0));

  AdvanceFor(config, derived.cyclotron_period_s, &state);
  CHECK(Near(state.x_m,
             initial_state.x_m +
                 derived.drift_velocity_x_m_s * derived.cyclotron_period_s,
             3e-9));
  CHECK(Near(state.y_m,
             initial_state.y_m +
                 derived.drift_velocity_y_m_s * derived.cyclotron_period_s,
             3e-9));
  CHECK(Near(state.velocity_x_m_s, initial_state.velocity_x_m_s, 3e-9));
  CHECK(Near(state.velocity_y_m_s, initial_state.velocity_y_m_s, 3e-9));
}

void TestPureGravityMatchesKinematicsAndEnergy() {
  LorentzParticleConfig config;
  config.mass_kg = 2.0;
  config.charge_c = 0.0;
  config.initial_x_m = 0.0;
  config.initial_y_m = 1.0;
  config.initial_speed_m_s = 4.0;
  config.initial_velocity_angle_degrees = 30.0;
  config.gravity_enabled = true;
  config.gravitational_acceleration_m_s2 = 9.8;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  const double initial_energy =
      CalculateLorentzParticleDerived(config, state).total_energy_j;

  constexpr double kDuration = 0.5;
  AdvanceFor(config, kDuration, &state);
  const double initial_velocity_x = 4.0 * std::cos(kPi / 6.0);
  constexpr double kInitialVelocityY = 2.0;
  CHECK(Near(state.x_m, initial_velocity_x * kDuration, 2e-10));
  CHECK(Near(
      state.y_m,
      1.0 + kInitialVelocityY * kDuration - 0.5 * 9.8 * kDuration * kDuration,
      2e-10));
  CHECK(Near(state.velocity_x_m_s, initial_velocity_x, 2e-10));
  CHECK(Near(state.velocity_y_m_s, kInitialVelocityY - 9.8 * kDuration, 2e-10));

  const LorentzParticleDerived derived =
      CalculateLorentzParticleDerived(config, state);
  CHECK(Near(derived.acceleration_x_m_s2, 0.0));
  CHECK(Near(derived.acceleration_y_m_s2, -9.8));
  CHECK(Near(derived.gravitational_potential_energy_j,
             config.mass_kg * 9.8 * (state.y_m - config.initial_y_m), 2e-10));
  CHECK(Near(derived.total_energy_j, initial_energy, 2e-9));
}

void TestElectricFieldBalancesGravity() {
  LorentzParticleConfig config;
  config.mass_kg = 2.0;
  config.charge_c = 4.0;
  config.initial_x_m = 0.0;
  config.initial_y_m = 0.0;
  config.initial_speed_m_s = 0.0;
  config.electric_field_strength_n_c = 4.9;
  config.electric_field_angle_degrees = 90.0;
  config.magnetic_field_enabled = false;
  config.gravity_enabled = true;
  config.gravitational_acceleration_m_s2 = 9.8;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);

  AdvanceFor(config, 1.0, &state);
  const LorentzParticleDerived derived =
      CalculateLorentzParticleDerived(config, state);
  CHECK(Near(state.x_m, 0.0));
  CHECK(Near(state.y_m, 0.0));
  CHECK(Near(state.velocity_x_m_s, 0.0));
  CHECK(Near(state.velocity_y_m_s, 0.0));
  CHECK(Near(derived.acceleration_x_m_s2, 0.0));
  CHECK(Near(derived.acceleration_y_m_s2, 0.0));
  CHECK(Near(derived.total_energy_j, 0.0));
}

void TestGeneralizedConstantForceDrift() {
  LorentzParticleConfig config;
  config.electric_field_strength_n_c = 10.1;
  config.gravity_enabled = true;
  config.gravitational_acceleration_m_s2 = 9.8;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  const LorentzParticleState initial_state = state;
  const LorentzParticleDerived initial =
      CalculateLorentzParticleDerived(config, state);
  CHECK(initial.has_cyclotron_data);
  CHECK(Near(initial.drift_velocity_x_m_s, 0.3));
  CHECK(Near(initial.drift_velocity_y_m_s, 0.0));

  AdvanceFor(config, initial.cyclotron_period_s, &state);
  const LorentzParticleDerived final =
      CalculateLorentzParticleDerived(config, state);
  CHECK(Near(state.x_m,
             initial_state.x_m +
                 initial.drift_velocity_x_m_s * initial.cyclotron_period_s,
             3e-9));
  CHECK(Near(state.y_m, initial_state.y_m, 3e-9));
  CHECK(Near(state.velocity_x_m_s, initial_state.velocity_x_m_s, 3e-9));
  CHECK(Near(state.velocity_y_m_s, initial_state.velocity_y_m_s, 3e-9));
  CHECK(Near(final.total_energy_j, initial.total_energy_j, 3e-9));
}

void TestDisabledAndZeroGravityPreserveV12() {
  LorentzParticleConfig disabled;
  disabled.gravity_enabled = false;
  disabled.gravitational_acceleration_m_s2 = 100.0;
  LorentzParticleConfig zero = disabled;
  zero.gravity_enabled = true;
  zero.gravitational_acceleration_m_s2 = 0.0;
  LorentzParticleState disabled_state =
      MakeInitialLorentzParticleState(disabled);
  LorentzParticleState zero_state = MakeInitialLorentzParticleState(zero);

  for (int step = 0; step < 100; ++step) {
    CHECK(StepLorentzParticle(disabled, kLorentzPhysicsStep, &disabled_state));
    CHECK(StepLorentzParticle(zero, kLorentzPhysicsStep, &zero_state));
    CHECK(SameState(disabled_state, zero_state));
  }
  const LorentzParticleDerived disabled_derived =
      CalculateLorentzParticleDerived(disabled, disabled_state);
  CHECK(disabled_derived.gravitational_potential_energy_j == 0.0);
  CHECK(Near(disabled_derived.drift_velocity_x_m_s, 0.3));
}

void TestChargeAndMagneticSignsIndependently() {
  LorentzParticleConfig positive;
  positive.initial_x_m = 0.0;
  positive.initial_y_m = 0.0;
  positive.initial_speed_m_s = 1.0;
  positive.initial_velocity_angle_degrees = 0.0;
  positive.electric_field_enabled = false;
  LorentzParticleState positive_state =
      MakeInitialLorentzParticleState(positive);
  CHECK(StepLorentzParticle(positive, kLorentzPhysicsStep, &positive_state));
  CHECK(positive_state.velocity_y_m_s < 0.0);

  LorentzParticleConfig negative_charge = positive;
  negative_charge.charge_c = -1.0;
  LorentzParticleState negative_charge_state =
      MakeInitialLorentzParticleState(negative_charge);
  CHECK(StepLorentzParticle(negative_charge, kLorentzPhysicsStep,
                            &negative_charge_state));
  CHECK(negative_charge_state.velocity_y_m_s > 0.0);

  LorentzParticleConfig negative_field = positive;
  negative_field.magnetic_field_z_t = -1.0;
  LorentzParticleState negative_field_state =
      MakeInitialLorentzParticleState(negative_field);
  CHECK(StepLorentzParticle(negative_field, kLorentzPhysicsStep,
                            &negative_field_state));
  CHECK(negative_field_state.velocity_y_m_s > 0.0);
}

void TestZeroChargeAndSmallMagneticField() {
  LorentzParticleConfig neutral;
  neutral.charge_c = 0.0;
  neutral.initial_x_m = 0.0;
  neutral.initial_y_m = 0.0;
  neutral.initial_speed_m_s = 4.0;
  neutral.initial_velocity_angle_degrees = 30.0;
  neutral.electric_field_strength_n_c = 1000000.0;
  neutral.magnetic_field_z_t = 100.0;
  LorentzParticleState neutral_state = MakeInitialLorentzParticleState(neutral);
  const LorentzParticleState neutral_initial = neutral_state;
  CHECK(StepLorentzParticle(neutral, kLorentzPhysicsStep, &neutral_state));
  CHECK(Near(neutral_state.x_m,
             neutral_initial.x_m +
                 neutral_initial.velocity_x_m_s * kLorentzPhysicsStep));
  CHECK(Near(neutral_state.y_m,
             neutral_initial.y_m +
                 neutral_initial.velocity_y_m_s * kLorentzPhysicsStep));
  CHECK(!CalculateLorentzParticleDerived(neutral, neutral_state)
             .has_cyclotron_data);

  LorentzParticleConfig zero_field;
  zero_field.initial_x_m = 0.0;
  zero_field.initial_y_m = 0.0;
  zero_field.electric_field_enabled = false;
  zero_field.magnetic_field_z_t = 0.0;
  LorentzParticleConfig tiny_field = zero_field;
  tiny_field.magnetic_field_z_t = 1e-10;
  LorentzParticleState zero_state = MakeInitialLorentzParticleState(zero_field);
  LorentzParticleState tiny_state = MakeInitialLorentzParticleState(tiny_field);
  CHECK(StepLorentzParticle(zero_field, kLorentzPhysicsStep, &zero_state));
  CHECK(StepLorentzParticle(tiny_field, kLorentzPhysicsStep, &tiny_state));
  CHECK(Near(tiny_state.x_m, zero_state.x_m, 1e-11));
  CHECK(Near(tiny_state.y_m, zero_state.y_m, 1e-11));
  CHECK(Near(tiny_state.velocity_x_m_s, zero_state.velocity_x_m_s, 1e-11));
  CHECK(Near(tiny_state.velocity_y_m_s, zero_state.velocity_y_m_s, 1e-11));
}

void TestExactStepComposition() {
  const LorentzParticleConfig config;
  LorentzParticleState whole = MakeInitialLorentzParticleState(config);
  LorentzParticleState halves = whole;
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &whole));
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep * 0.5, &halves));
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep * 0.5, &halves));
  CHECK(Near(whole.x_m, halves.x_m, 2e-12));
  CHECK(Near(whole.y_m, halves.y_m, 2e-12));
  CHECK(Near(whole.velocity_x_m_s, halves.velocity_x_m_s, 2e-12));
  CHECK(Near(whole.velocity_y_m_s, halves.velocity_y_m_s, 2e-12));
  CHECK(Near(whole.time_seconds, halves.time_seconds, 2e-12));
}

void TestOutOfBoundsTerminalState() {
  LorentzParticleConfig config;
  config.initial_x_m = 9.99;
  config.initial_y_m = 0.0;
  config.initial_speed_m_s = 100.0;
  config.initial_velocity_angle_degrees = 0.0;
  config.electric_field_enabled = false;
  config.magnetic_field_enabled = false;
  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(state.status == LorentzParticleStatus::kOutOfBounds);
  CHECK(state.x_m > 10.0);
  CHECK(GetLorentzParticleStateError(config, state) == nullptr);
  const LorentzParticleState terminal = state;
  CHECK(!StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(SameState(state, terminal));
}

void TestValidationAndFailureAtomicity() {
  LorentzParticleConfig config;
  CHECK(GetLorentzParticleConfigError(config) == nullptr);
  constexpr std::array<double LorentzParticleConfig::*, 10> fields = {
      &LorentzParticleConfig::mass_kg,
      &LorentzParticleConfig::charge_c,
      &LorentzParticleConfig::initial_x_m,
      &LorentzParticleConfig::initial_y_m,
      &LorentzParticleConfig::initial_speed_m_s,
      &LorentzParticleConfig::initial_velocity_angle_degrees,
      &LorentzParticleConfig::electric_field_strength_n_c,
      &LorentzParticleConfig::electric_field_angle_degrees,
      &LorentzParticleConfig::magnetic_field_z_t,
      &LorentzParticleConfig::gravitational_acceleration_m_s2,
  };
  for (double LorentzParticleConfig::* field : fields) {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
      LorentzParticleConfig invalid_config = config;
      invalid_config.*field = invalid;
      CHECK(GetLorentzParticleConfigError(invalid_config) != nullptr);
    }
  }

  LorentzParticleConfig boundary = config;
  boundary.mass_kg = 0.01;
  boundary.charge_c = 1.0;
  boundary.initial_speed_m_s = 100.0;
  boundary.electric_field_strength_n_c = 100.0;
  boundary.magnetic_field_z_t = 0.6;
  CHECK(GetLorentzParticleConfigError(boundary) == nullptr);
  boundary.magnetic_field_z_t = 0.600001;
  CHECK(GetLorentzParticleConfigError(boundary) != nullptr);
  boundary.magnetic_field_z_t = 0.6;
  boundary.electric_field_strength_n_c = 100.001;
  CHECK(GetLorentzParticleConfigError(boundary) != nullptr);

  LorentzParticleConfig underflow = config;
  underflow.mass_kg = 1000.0;
  underflow.charge_c = std::numeric_limits<double>::denorm_min();
  CHECK(GetLorentzParticleConfigError(underflow) != nullptr);
  underflow.magnetic_field_enabled = false;
  CHECK(GetLorentzParticleConfigError(underflow) != nullptr);

  LorentzParticleConfig diagonal_boundary = config;
  diagonal_boundary.mass_kg = 1.0;
  diagonal_boundary.charge_c = 0.01;
  diagonal_boundary.electric_field_strength_n_c = 1000000.0;
  diagonal_boundary.electric_field_angle_degrees = 45.0;
  diagonal_boundary.magnetic_field_enabled = false;
  CHECK(GetLorentzParticleConfigError(diagonal_boundary) == nullptr);

  LorentzParticleConfig rounding_boundary = config;
  rounding_boundary.mass_kg = 0.01302;
  rounding_boundary.charge_c = 0.01302;
  rounding_boundary.electric_field_strength_n_c = 10000.0;
  rounding_boundary.magnetic_field_z_t = 60.0;
  CHECK(GetLorentzParticleConfigError(rounding_boundary) == nullptr);

  LorentzParticleConfig gravity_boundary = config;
  gravity_boundary.gravity_enabled = true;
  gravity_boundary.gravitational_acceleration_m_s2 = 0.0;
  CHECK(GetLorentzParticleConfigError(gravity_boundary) == nullptr);
  gravity_boundary.gravitational_acceleration_m_s2 = 100.0;
  CHECK(GetLorentzParticleConfigError(gravity_boundary) == nullptr);
  gravity_boundary.gravitational_acceleration_m_s2 = -0.001;
  CHECK(GetLorentzParticleConfigError(gravity_boundary) != nullptr);
  gravity_boundary.gravitational_acceleration_m_s2 = 100.001;
  CHECK(GetLorentzParticleConfigError(gravity_boundary) != nullptr);

  LorentzParticleConfig constant_acceleration_boundary = config;
  constant_acceleration_boundary.electric_field_strength_n_c = 9900.0;
  constant_acceleration_boundary.electric_field_angle_degrees = -90.0;
  constant_acceleration_boundary.magnetic_field_enabled = false;
  constant_acceleration_boundary.gravity_enabled = true;
  constant_acceleration_boundary.gravitational_acceleration_m_s2 = 100.0;
  CHECK(GetLorentzParticleConfigError(constant_acceleration_boundary) ==
        nullptr);
  constant_acceleration_boundary.electric_field_strength_n_c = 9900.001;
  CHECK(GetLorentzParticleConfigError(constant_acceleration_boundary) !=
        nullptr);

  LorentzParticleConfig diagonal_constant_boundary = config;
  diagonal_constant_boundary.magnetic_field_enabled = false;
  diagonal_constant_boundary.gravity_enabled = true;
  diagonal_constant_boundary.gravitational_acceleration_m_s2 = 100.0;
  constexpr double kBoundaryElectricX = 6000.0;
  constexpr double kBoundaryElectricY = -7900.0;
  diagonal_constant_boundary.electric_field_strength_n_c =
      std::hypot(kBoundaryElectricX, kBoundaryElectricY);
  diagonal_constant_boundary.electric_field_angle_degrees =
      std::atan2(kBoundaryElectricY, kBoundaryElectricX) * 180.0 / kPi;
  CHECK(GetLorentzParticleConfigError(diagonal_constant_boundary) == nullptr);
  constexpr double kJustOverBoundaryElectricX = 6000.001;
  diagonal_constant_boundary.electric_field_strength_n_c =
      std::hypot(kJustOverBoundaryElectricX, kBoundaryElectricY);
  diagonal_constant_boundary.electric_field_angle_degrees =
      std::atan2(kBoundaryElectricY, kJustOverBoundaryElectricX) * 180.0 / kPi;
  CHECK(GetLorentzParticleConfigError(diagonal_constant_boundary) != nullptr);

  LorentzParticleConfig disabled = config;
  disabled.mass_kg = 0.01;
  disabled.charge_c = 1000.0;
  disabled.electric_field_enabled = false;
  disabled.electric_field_strength_n_c = 1000000.0;
  disabled.magnetic_field_enabled = false;
  disabled.magnetic_field_z_t = 100.0;
  disabled.gravity_enabled = false;
  disabled.gravitational_acceleration_m_s2 = 100.0;
  CHECK(GetLorentzParticleConfigError(disabled) == nullptr);

  LorentzParticleState state = MakeInitialLorentzParticleState(config);
  const LorentzParticleState original = state;
  CHECK(!StepLorentzParticle(config, 0.0, &state));
  CHECK(SameState(state, original));
  CHECK(!StepLorentzParticle(config, kLorentzPhysicsStep * 2.0, &state));
  CHECK(SameState(state, original));
  CHECK(!StepLorentzParticle(config, kLorentzPhysicsStep, nullptr));

  LorentzParticleConfig invalid_config = config;
  invalid_config.mass_kg = 0.0;
  CHECK(!StepLorentzParticle(invalid_config, kLorentzPhysicsStep, &state));
  CHECK(SameState(state, original));

  state.time_seconds = -1.0;
  const LorentzParticleState invalid_state = state;
  CHECK(!StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(SameState(state, invalid_state));

  state.x_m = std::numeric_limits<double>::quiet_NaN();
  CHECK(GetLorentzParticleStateError(config, state) != nullptr);
  state = original;
  constexpr int kInvalidStatus = 99;
  static_assert(sizeof(kInvalidStatus) == sizeof(state.status));
  std::memcpy(&state.status, &kInvalidStatus, sizeof(state.status));
  CHECK(GetLorentzParticleStateError(config, state) != nullptr);
}

void TestTransientOutOfBoundsIsTerminal() {
  LorentzParticleConfig config;
  config.electric_field_enabled = false;
  config.magnetic_field_z_t = 60.0;
  LorentzParticleState state{10.0,        0.0, 10.0,
                             -99.4987437, 0.0, LorentzParticleStatus::kActive};
  CHECK(GetLorentzParticleStateError(config, state) == nullptr);
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(state.status == LorentzParticleStatus::kOutOfBounds);
  CHECK(state.x_m > tiny2d::sandbox::kLorentzMaximumX);
  CHECK(state.time_seconds > 0.0);
  CHECK(state.time_seconds < kLorentzPhysicsStep);
}

void TestGravityTransientOutOfBoundsIsTerminal() {
  LorentzParticleConfig config;
  config.charge_c = 0.0;
  config.electric_field_enabled = false;
  config.magnetic_field_enabled = false;
  config.gravity_enabled = true;
  config.gravitational_acceleration_m_s2 = 100.0;
  const double peak_time = kLorentzPhysicsStep * 0.5;
  LorentzParticleState state{
      0.0, tiny2d::sandbox::kLorentzMaximumY,
      0.0, config.gravitational_acceleration_m_s2 * peak_time,
      0.0, LorentzParticleStatus::kActive,
  };
  CHECK(GetLorentzParticleStateError(config, state) == nullptr);
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(state.status == LorentzParticleStatus::kOutOfBounds);
  CHECK(state.y_m > tiny2d::sandbox::kLorentzMaximumY);
  CHECK(Near(state.time_seconds, peak_time, 1e-12));
}

void TestTinyMagneticFieldTransientExit() {
  LorentzParticleConfig config;
  config.charge_c = -1.0;
  config.electric_field_strength_n_c = 10000.0;
  config.electric_field_angle_degrees = 0.0;
  config.magnetic_field_z_t = -1e-16;
  LorentzParticleState state{10.0, 0.0, 10.0,
                             0.0,  0.0, LorentzParticleStatus::kActive};
  CHECK(GetLorentzParticleStateError(config, state) == nullptr);
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(state.status == LorentzParticleStatus::kOutOfBounds);
  CHECK(state.x_m > tiny2d::sandbox::kLorentzMaximumX);
  CHECK(Near(state.time_seconds, 0.001, 1e-12));
}

void TestAngledFieldTinyMagneticTransientExit() {
  LorentzParticleConfig config;
  config.electric_field_strength_n_c = 10000.0;
  config.electric_field_angle_degrees = -45.0;
  config.magnetic_field_z_t = -1e-20;
  LorentzParticleState state{0.0,  6.0, 0.0,
                             10.0, 0.0, LorentzParticleStatus::kActive};
  CHECK(GetLorentzParticleStateError(config, state) == nullptr);
  CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &state));
  CHECK(state.status == LorentzParticleStatus::kOutOfBounds);
  CHECK(state.y_m > tiny2d::sandbox::kLorentzMaximumY);
  CHECK(Near(state.time_seconds, std::sqrt(2.0) / 1000.0, 1e-12));
}

void TestHistoryLookupAndDeterminism() {
  const std::vector<LorentzParticleState> history = {
      {-1.0, 0.0, 1.0, 0.0, 0.0, LorentzParticleStatus::kActive},
      {0.0, 0.0, 1.0, 0.0, 1.0, LorentzParticleStatus::kActive},
      {1.0, 0.0, 1.0, 0.0, 2.0, LorentzParticleStatus::kActive},
  };
  CHECK(FindLorentzParticleState({}, 0.0) == nullptr);
  CHECK(FindLorentzParticleState(
            history, std::numeric_limits<double>::quiet_NaN()) == nullptr);
  CHECK(FindLorentzParticleState(history, -1.0) == &history[0]);
  CHECK(FindLorentzParticleState(history, 0.5) == &history[0]);
  CHECK(FindLorentzParticleState(history, 0.6) == &history[1]);
  CHECK(FindLorentzParticleState(history, 3.0) == &history[2]);

  const LorentzParticleConfig config;
  LorentzParticleState first = MakeInitialLorentzParticleState(config);
  LorentzParticleState second = first;
  for (int step = 0; step < 1000; ++step) {
    CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &first));
    CHECK(StepLorentzParticle(config, kLorentzPhysicsStep, &second));
    CHECK(SameState(first, second));
  }
}

using TestFunction = void (*)();

struct NamedTest {
  const char* name;
  TestFunction function;
};

}  // namespace

int main() {
  const std::array tests = {
      NamedTest{"pure electric field", TestPureElectricFieldMatchesKinematics},
      NamedTest{"pure magnetic orbit", TestPureMagneticOrbitAndEnergy},
      NamedTest{"crossed-field drift", TestCrossedFieldDriftAfterOnePeriod},
      NamedTest{"pure gravity and energy",
                TestPureGravityMatchesKinematicsAndEnergy},
      NamedTest{"electric-gravity balance", TestElectricFieldBalancesGravity},
      NamedTest{"generalized constant-force drift",
                TestGeneralizedConstantForceDrift},
      NamedTest{"disabled and zero gravity preserve V12",
                TestDisabledAndZeroGravityPreserveV12},
      NamedTest{"charge and magnetic signs",
                TestChargeAndMagneticSignsIndependently},
      NamedTest{"zero charge and small B", TestZeroChargeAndSmallMagneticField},
      NamedTest{"exact step composition", TestExactStepComposition},
      NamedTest{"out-of-bounds terminal", TestOutOfBoundsTerminalState},
      NamedTest{"transient out-of-bounds terminal",
                TestTransientOutOfBoundsIsTerminal},
      NamedTest{"gravity transient out-of-bounds terminal",
                TestGravityTransientOutOfBoundsIsTerminal},
      NamedTest{"tiny-B transient exit", TestTinyMagneticFieldTransientExit},
      NamedTest{"angled-field tiny-B transient exit",
                TestAngledFieldTinyMagneticTransientExit},
      NamedTest{"validation and atomicity", TestValidationAndFailureAtomicity},
      NamedTest{"history and determinism", TestHistoryLookupAndDeterminism},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  return 0;
}
