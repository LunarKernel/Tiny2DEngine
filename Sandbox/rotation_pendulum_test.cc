#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

#include "rotation_pendulum_model.h"

namespace {

using tiny2d::sandbox::CalculatePendulumDerived;
using tiny2d::sandbox::FindPendulumState;
using tiny2d::sandbox::GetPendulumConfigError;
using tiny2d::sandbox::GetPendulumStateError;
using tiny2d::sandbox::GetSmallAnglePeriod;
using tiny2d::sandbox::IsPendulumDerivedFinite;
using tiny2d::sandbox::kMaximumChargeMagnitude;
using tiny2d::sandbox::kMaximumDamping;
using tiny2d::sandbox::kMaximumDriveAngularFrequency;
using tiny2d::sandbox::kMaximumDriveTorque;
using tiny2d::sandbox::kMaximumElectricField;
using tiny2d::sandbox::kMaximumInitialAngularSpeed;
using tiny2d::sandbox::kMaximumMass;
using tiny2d::sandbox::kMaximumRodLength;
using tiny2d::sandbox::kMinimumMass;
using tiny2d::sandbox::kMinimumRodLength;
using tiny2d::sandbox::kPendulumPhysicsStep;
using tiny2d::sandbox::kPendulumPi;
using tiny2d::sandbox::kPendulumRadiansToDegrees;
using tiny2d::sandbox::MakeInitialPendulumState;
using tiny2d::sandbox::PendulumConfig;
using tiny2d::sandbox::PendulumDerived;
using tiny2d::sandbox::PendulumState;
using tiny2d::sandbox::StepPendulum;

std::size_t check_count = 0;

void Check(bool condition, const char* expression, const char* file, int line) {
  ++check_count;
  if (!condition) {
    std::cerr << file << ':' << line << ": CHECK failed: " << expression
              << '\n';
    std::exit(1);
  }
}

#define CHECK(expression) Check((expression), #expression, __FILE__, __LINE__)

constexpr float kTestTolerance = 0.0001f;

bool Near(float actual, float expected, float tolerance = kTestTolerance) {
  return std::abs(actual - expected) <= tolerance;
}

bool NearTime(double actual, double expected, double tolerance = 0.000001) {
  return std::abs(actual - expected) <= tolerance;
}

void ExpectInvalidConfig(const PendulumConfig& config) {
  CHECK(GetPendulumConfigError(config) != nullptr);
}

PendulumState MakeTestState(const PendulumConfig& config, float angle,
                            float angular_velocity) {
  PendulumState state;
  state.angle_radians = angle;
  state.angular_velocity_rad_s = angular_velocity;
  state.angular_acceleration_rad_s2 =
      CalculatePendulumDerived(config, state).angular_acceleration_rad_s2;
  return state;
}

void CheckStateExactlyEqual(const PendulumState& actual,
                            const PendulumState& expected) {
  CHECK(actual.angle_radians == expected.angle_radians);
  CHECK(actual.angular_velocity_rad_s == expected.angular_velocity_rad_s);
  CHECK(actual.angular_acceleration_rad_s2 ==
        expected.angular_acceleration_rad_s2);
  CHECK(actual.time_seconds == expected.time_seconds);
}

void CheckLegacyDerivedExactlyEqual(const PendulumDerived& actual,
                                    const PendulumDerived& expected) {
  CHECK(actual.moment_of_inertia_kg_m2 == expected.moment_of_inertia_kg_m2);
  CHECK(actual.gravity_torque_n_m == expected.gravity_torque_n_m);
  CHECK(actual.electric_torque_n_m == expected.electric_torque_n_m);
  CHECK(actual.damping_torque_n_m == expected.damping_torque_n_m);
  CHECK(actual.total_torque_n_m == expected.total_torque_n_m);
  CHECK(actual.angular_acceleration_rad_s2 ==
        expected.angular_acceleration_rad_s2);
  CHECK(actual.kinetic_energy_j == expected.kinetic_energy_j);
  CHECK(actual.gravitational_potential_energy_j ==
        expected.gravitational_potential_energy_j);
  CHECK(actual.electric_potential_energy_j ==
        expected.electric_potential_energy_j);
  CHECK(actual.total_energy_j == expected.total_energy_j);
}

void TestFormulaAnchorsAndSemiImplicitStep() {
  PendulumConfig config;
  config.rod_length_m = 2.0f;
  config.rod_mass_kg = 3.0f;
  config.counterweight_mass_kg = 2.0f;
  config.counterweight_distance_m = 1.0f;
  config.gravity_m_s2 = 10.0f;
  config.counterweight_charge_c = 2.0f;
  config.electric_field_strength_n_c = 12.0f;
  config.electric_field_angle_degrees = 90.0f;
  config.damping_enabled = true;
  config.damping_coefficient_n_m_s = 3.0f;

  PendulumState state = MakeTestState(config, kPendulumPi * 0.5f, 2.0f);
  const PendulumDerived derived = CalculatePendulumDerived(config, state);
  CHECK(Near(derived.moment_of_inertia_kg_m2, 6.0f));
  CHECK(Near(derived.gravity_torque_n_m, -50.0f));
  CHECK(Near(derived.electric_torque_n_m, 24.0f));
  CHECK(Near(derived.damping_torque_n_m, -6.0f));
  CHECK(Near(derived.total_torque_n_m, -32.0f));
  CHECK(Near(derived.angular_acceleration_rad_s2, -32.0f / 6.0f));

  constexpr float kStep = 0.1f;
  CHECK(StepPendulum(config, kStep, &state));
  CHECK(Near(state.angular_velocity_rad_s, 1.4666667f));
  CHECK(Near(state.angle_radians, 1.7174630f));
  CHECK(NearTime(state.time_seconds, kStep));
}

void TestDrivePhasePowerAndStepSampling() {
  PendulumConfig config;
  config.drive_enabled = true;
  config.drive_torque_amplitude_n_m = 12.0f;
  config.drive_angular_frequency_rad_s = 2.0f;

  PendulumState state = MakeTestState(config, 0.3f, 3.0f);
  PendulumDerived derived = CalculatePendulumDerived(config, state);
  CHECK(Near(derived.driving_torque_n_m, 12.0f));
  CHECK(Near(derived.driving_power_w, 36.0f));
  CHECK(Near(derived.total_torque_n_m,
             derived.gravity_torque_n_m + derived.electric_torque_n_m +
                 derived.damping_torque_n_m + derived.driving_torque_n_m));

  state.time_seconds = static_cast<double>(kPendulumPi) * 0.25;
  derived = CalculatePendulumDerived(config, state);
  CHECK(Near(derived.driving_torque_n_m, 0.0f, 0.00001f));
  CHECK(Near(derived.driving_power_w, 0.0f, 0.00005f));

  state.time_seconds = static_cast<double>(kPendulumPi) * 0.5;
  derived = CalculatePendulumDerived(config, state);
  CHECK(Near(derived.driving_torque_n_m, -12.0f));
  CHECK(Near(derived.driving_power_w, -36.0f));

  state = MakeTestState(config, 0.3f, 0.2f);
  const PendulumDerived before = CalculatePendulumDerived(config, state);
  constexpr float kStep = 0.01f;
  const float expected_velocity =
      state.angular_velocity_rad_s + before.angular_acceleration_rad_s2 * kStep;
  const float expected_angle = std::remainder(
      state.angle_radians + expected_velocity * kStep, 2.0f * kPendulumPi);
  CHECK(StepPendulum(config, kStep, &state));
  CHECK(Near(state.angular_velocity_rad_s, expected_velocity));
  CHECK(Near(state.angle_radians, expected_angle));
  CHECK(NearTime(state.time_seconds, kStep));
  CHECK(Near(
      state.angular_acceleration_rad_s2,
      CalculatePendulumDerived(config, state).angular_acceleration_rad_s2));
}

void TestDisabledAndZeroDrivePreserveLegacyTrajectory() {
  PendulumConfig legacy_config;
  PendulumConfig disabled_config = legacy_config;
  disabled_config.drive_torque_amplitude_n_m = kMaximumDriveTorque;
  disabled_config.drive_angular_frequency_rad_s = kMaximumDriveAngularFrequency;
  PendulumConfig zero_amplitude_config = legacy_config;
  zero_amplitude_config.drive_enabled = true;
  zero_amplitude_config.drive_angular_frequency_rad_s =
      kMaximumDriveAngularFrequency;

  CHECK(GetPendulumConfigError(legacy_config) == nullptr);
  CHECK(GetPendulumConfigError(disabled_config) == nullptr);
  CHECK(GetPendulumConfigError(zero_amplitude_config) == nullptr);
  PendulumState legacy = MakeInitialPendulumState(legacy_config);
  PendulumState disabled = MakeInitialPendulumState(disabled_config);
  PendulumState zero_amplitude =
      MakeInitialPendulumState(zero_amplitude_config);
  for (int step = 0; step < 1000; ++step) {
    const PendulumDerived legacy_derived =
        CalculatePendulumDerived(legacy_config, legacy);
    const PendulumDerived disabled_derived =
        CalculatePendulumDerived(disabled_config, disabled);
    const PendulumDerived zero_derived =
        CalculatePendulumDerived(zero_amplitude_config, zero_amplitude);
    CheckLegacyDerivedExactlyEqual(disabled_derived, legacy_derived);
    CheckLegacyDerivedExactlyEqual(zero_derived, legacy_derived);
    CHECK(disabled_derived.driving_torque_n_m == 0.0f);
    CHECK(disabled_derived.driving_power_w == 0.0f);
    CHECK(zero_derived.driving_torque_n_m == 0.0f);
    CHECK(zero_derived.driving_power_w == 0.0f);

    CHECK(StepPendulum(legacy_config, kPendulumPhysicsStep, &legacy));
    CHECK(StepPendulum(disabled_config, kPendulumPhysicsStep, &disabled));
    CHECK(StepPendulum(zero_amplitude_config, kPendulumPhysicsStep,
                       &zero_amplitude));
    CheckStateExactlyEqual(disabled, legacy);
    CheckStateExactlyEqual(zero_amplitude, legacy);
  }
}

float MeasureDrivenPeakAngle(PendulumConfig config, float start_time,
                             float end_time) {
  PendulumState state = MakeInitialPendulumState(config);
  float peak_angle = 0.0f;
  while (state.time_seconds < end_time) {
    CHECK(StepPendulum(config, kPendulumPhysicsStep, &state));
    if (state.time_seconds >= start_time) {
      peak_angle = std::max(peak_angle, std::abs(state.angle_radians));
    }
  }
  return peak_angle;
}

void TestResonantResponse() {
  PendulumConfig config;
  config.rod_length_m = 2.0f;
  config.rod_mass_kg = 3.0f;
  config.counterweight_mass_kg = 2.0f;
  config.counterweight_distance_m = 1.0f;
  config.initial_angle_degrees = 0.0f;
  config.gravity_m_s2 = 10.0f;
  config.electric_field_enabled = false;
  config.damping_enabled = true;
  config.damping_coefficient_n_m_s = 1.0f;
  config.drive_enabled = true;
  config.drive_torque_amplitude_n_m = 0.5f;
  const float natural_frequency =
      2.0f * kPendulumPi / GetSmallAnglePeriod(config);

  config.drive_angular_frequency_rad_s = natural_frequency;
  CHECK(GetPendulumConfigError(config) == nullptr);
  const float resonant_peak = MeasureDrivenPeakAngle(config, 20.0f, 30.0f);
  config.drive_angular_frequency_rad_s = 1.8f * natural_frequency;
  CHECK(GetPendulumConfigError(config) == nullptr);
  const float off_resonant_peak = MeasureDrivenPeakAngle(config, 20.0f, 30.0f);
  CHECK(resonant_peak >= 5.0f * off_resonant_peak);
}

void TestHistoryLookup() {
  std::vector<PendulumState> history(3);
  history[0].time_seconds = 0.0;
  history[1].time_seconds = 1.0;
  history[2].time_seconds = 2.0;

  CHECK(FindPendulumState({}, 0.0) == nullptr);
  CHECK(FindPendulumState(history, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);
  CHECK(FindPendulumState(history, -1.0) == &history[0]);
  CHECK(FindPendulumState(history, 0.5) == &history[0]);
  CHECK(FindPendulumState(history, 0.6) == &history[1]);
  CHECK(FindPendulumState(history, 3.0) == &history[2]);
}

void TestElectricDirectionChargeAndCounterweightPosition() {
  PendulumConfig config;
  config.rod_length_m = 2.0f;
  config.rod_mass_kg = 3.0f;
  config.counterweight_mass_kg = 2.0f;
  config.counterweight_distance_m = 0.5f;
  config.gravity_m_s2 = 10.0f;
  config.counterweight_charge_c = 2.0f;
  config.electric_field_strength_n_c = 10.0f;
  const PendulumState vertical_state = MakeTestState(config, 0.0f, 0.0f);

  config.electric_field_angle_degrees = 0.0f;
  CHECK(
      Near(CalculatePendulumDerived(config, vertical_state).electric_torque_n_m,
           10.0f));
  config.electric_field_angle_degrees = 180.0f;
  CHECK(
      Near(CalculatePendulumDerived(config, vertical_state).electric_torque_n_m,
           -10.0f));
  config.electric_field_angle_degrees = 90.0f;
  CHECK(
      Near(CalculatePendulumDerived(config, vertical_state).electric_torque_n_m,
           0.0f));
  config.electric_field_angle_degrees = 0.0f;
  config.counterweight_charge_c = -2.0f;
  CHECK(
      Near(CalculatePendulumDerived(config, vertical_state).electric_torque_n_m,
           -10.0f));
  config.counterweight_charged = false;
  CHECK(
      Near(CalculatePendulumDerived(config, vertical_state).electric_torque_n_m,
           0.0f));
  config.counterweight_charged = true;
  config.counterweight_distance_m = 0.0f;
  CHECK(
      Near(CalculatePendulumDerived(config, vertical_state).electric_torque_n_m,
           0.0f));

  config.electric_field_enabled = false;
  config.counterweight_charge_c = 0.0f;
  struct PositionExpectation {
    float distance;
    float moment_of_inertia;
    float angular_acceleration;
    float small_angle_period;
  };
  constexpr std::array expectations = {
      PositionExpectation{0.0f, 4.0f, -7.5f, 2.2942948f},
      PositionExpectation{1.0f, 6.0f, -8.3333333f, 2.1765592f},
      PositionExpectation{2.0f, 12.0f, -5.8333333f, 2.6014859f},
  };
  for (const PositionExpectation& expectation : expectations) {
    config.counterweight_distance_m = expectation.distance;
    const PendulumState horizontal_state =
        MakeTestState(config, kPendulumPi * 0.5f, 0.0f);
    const PendulumDerived derived =
        CalculatePendulumDerived(config, horizontal_state);
    CHECK(Near(derived.moment_of_inertia_kg_m2, expectation.moment_of_inertia));
    CHECK(Near(derived.angular_acceleration_rad_s2,
               expectation.angular_acceleration));
    CHECK(Near(GetSmallAnglePeriod(config), expectation.small_angle_period));
  }
}

void TestSmallAnglePeriod() {
  PendulumConfig config;
  config.rod_length_m = 2.0f;
  config.rod_mass_kg = 3.0f;
  config.counterweight_mass_kg = 2.0f;
  config.counterweight_distance_m = 1.0f;
  config.gravity_m_s2 = 10.0f;
  config.electric_field_enabled = false;
  config.damping_enabled = false;
  CHECK(GetPendulumConfigError(config) == nullptr);
  CHECK(Near(GetSmallAnglePeriod(config), 2.1765592f));

  PendulumState state = MakeTestState(config, 0.01f, 0.0f);
  float previous_angle = state.angle_radians;
  double previous_time = state.time_seconds;
  std::array<double, 2> crossings{};
  std::size_t crossing_count = 0;
  for (int step = 0; step < 2000 && crossing_count < crossings.size(); ++step) {
    CHECK(StepPendulum(config, kPendulumPhysicsStep, &state));
    if (previous_angle > 0.0f && state.angle_radians <= 0.0f &&
        state.angular_velocity_rad_s < 0.0f) {
      const float fraction =
          previous_angle / (previous_angle - state.angle_radians);
      crossings[crossing_count++] =
          previous_time + fraction * kPendulumPhysicsStep;
    }
    previous_angle = state.angle_radians;
    previous_time = state.time_seconds;
  }

  CHECK(crossing_count == crossings.size());
  const double measured_period = crossings[1] - crossings[0];
  CHECK(NearTime(measured_period, 2.1765592, 0.002));
  CHECK(NearTime(measured_period, GetSmallAnglePeriod(config), 0.002));
}

void TestConservativeEnergyAndDampingLoss() {
  PendulumConfig config;
  config.rod_length_m = 2.0f;
  config.rod_mass_kg = 3.0f;
  config.counterweight_mass_kg = 2.0f;
  config.counterweight_distance_m = 1.0f;
  config.gravity_m_s2 = 10.0f;
  config.counterweight_charge_c = 2.0f;
  config.electric_field_strength_n_c = 4.0f;
  config.electric_field_angle_degrees = 0.3f * kPendulumRadiansToDegrees;
  config.damping_enabled = false;
  CHECK(GetPendulumConfigError(config) == nullptr);

  PendulumState state = MakeTestState(config, 0.5f, 0.0f);
  PendulumDerived derived = CalculatePendulumDerived(config, state);
  CHECK(Near(derived.gravitational_potential_energy_j, 6.1208719f));
  CHECK(Near(derived.electric_potential_energy_j, -1.5893546f));
  CHECK(Near(derived.total_energy_j, 4.5315173f));
  const float initial_energy = derived.total_energy_j;
  float maximum_energy_error = 0.0f;
  for (int step = 0; step < 20 * 240; ++step) {
    CHECK(StepPendulum(config, kPendulumPhysicsStep, &state));
    derived = CalculatePendulumDerived(config, state);
    maximum_energy_error =
        std::max(maximum_energy_error,
                 std::abs(derived.total_energy_j - initial_energy));
  }
  CHECK(maximum_energy_error / std::abs(initial_energy) < 0.02f);

  config.electric_field_enabled = false;
  config.damping_enabled = true;
  config.damping_coefficient_n_m_s = 1.0f;
  state = MakeTestState(config, 0.5f, 0.0f);
  const float undamped_initial_energy =
      CalculatePendulumDerived(config, state).total_energy_j;
  for (int step = 0; step < 20 * 240; ++step) {
    CHECK(StepPendulum(config, kPendulumPhysicsStep, &state));
  }
  const float damped_final_energy =
      CalculatePendulumDerived(config, state).total_energy_j;
  CHECK(damped_final_energy >= 0.0f);
  CHECK(damped_final_energy < undamped_initial_energy * 0.05f);
}

void TestValidationNonFiniteAndDangerousInputs() {
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  constexpr std::array<float PendulumConfig::*, 13> config_fields = {
      &PendulumConfig::rod_length_m,
      &PendulumConfig::rod_mass_kg,
      &PendulumConfig::counterweight_mass_kg,
      &PendulumConfig::counterweight_distance_m,
      &PendulumConfig::counterweight_charge_c,
      &PendulumConfig::initial_angle_degrees,
      &PendulumConfig::initial_angular_velocity_rad_s,
      &PendulumConfig::gravity_m_s2,
      &PendulumConfig::electric_field_strength_n_c,
      &PendulumConfig::electric_field_angle_degrees,
      &PendulumConfig::damping_coefficient_n_m_s,
      &PendulumConfig::drive_torque_amplitude_n_m,
      &PendulumConfig::drive_angular_frequency_rad_s,
  };
  for (float invalid_value : invalid_values) {
    for (float PendulumConfig::* field : config_fields) {
      PendulumConfig config;
      config.*field = invalid_value;
      ExpectInvalidConfig(config);
    }
  }

  PendulumConfig config;
  config.rod_length_m = kMinimumRodLength;
  config.counterweight_distance_m = kMinimumRodLength;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.rod_length_m = kMaximumRodLength;
  config.counterweight_distance_m = kMaximumRodLength;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.rod_mass_kg = kMinimumMass;
  config.counterweight_mass_kg = kMaximumMass;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.counterweight_distance_m = 0.0f;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config.counterweight_distance_m = config.rod_length_m;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.counterweight_charged = false;
  config.counterweight_charge_c = -kMaximumChargeMagnitude;
  config.electric_field_strength_n_c = kMaximumElectricField;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.initial_angle_degrees = -180.0f;
  config.initial_angular_velocity_rad_s = -kMaximumInitialAngularSpeed;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.gravity_m_s2 = 10.0f;
  config.electric_field_angle_degrees = 180.0f;
  config.damping_enabled = false;
  config.damping_coefficient_n_m_s = kMaximumDamping;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config = PendulumConfig{};
  config.drive_enabled = true;
  config.drive_torque_amplitude_n_m = kMaximumDriveTorque;
  config.drive_angular_frequency_rad_s = kMaximumDriveAngularFrequency;
  CHECK(GetPendulumConfigError(config) == nullptr);

  config = PendulumConfig{};
  config.rod_length_m = kMinimumRodLength - 0.001f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.rod_mass_kg = 0.0f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.counterweight_mass_kg = kMaximumMass + 0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.counterweight_distance_m = -0.001f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.counterweight_distance_m = config.rod_length_m + 0.001f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.counterweight_charge_c = kMaximumChargeMagnitude + 0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.initial_angle_degrees = 180.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.initial_angular_velocity_rad_s = kMaximumInitialAngularSpeed + 0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.gravity_m_s2 = 9.81f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.electric_field_strength_n_c = -0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.electric_field_angle_degrees = -180.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.damping_coefficient_n_m_s = -0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.drive_torque_amplitude_n_m = -0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.drive_torque_amplitude_n_m = kMaximumDriveTorque + 0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.drive_angular_frequency_rad_s = -0.01f;
  ExpectInvalidConfig(config);
  config = PendulumConfig{};
  config.drive_angular_frequency_rad_s = kMaximumDriveAngularFrequency + 0.01f;
  ExpectInvalidConfig(config);

  config = PendulumConfig{};
  config.rod_length_m = kMinimumRodLength;
  config.rod_mass_kg = kMinimumMass;
  config.counterweight_mass_kg = kMinimumMass;
  config.counterweight_distance_m = kMinimumRodLength;
  config.counterweight_charge_c = kMaximumChargeMagnitude;
  config.electric_field_strength_n_c = kMaximumElectricField;
  ExpectInvalidConfig(config);
  config.electric_field_enabled = false;
  config.damping_enabled = true;
  config.damping_coefficient_n_m_s = kMaximumDamping;
  ExpectInvalidConfig(config);

  config = PendulumConfig{};
  config.rod_length_m = kMinimumRodLength;
  config.rod_mass_kg = kMinimumMass;
  config.counterweight_mass_kg = kMinimumMass;
  config.counterweight_distance_m = kMinimumRodLength;
  config.electric_field_enabled = false;
  config.drive_torque_amplitude_n_m = 100.0f;
  config.drive_angular_frequency_rad_s = kMaximumDriveAngularFrequency;
  CHECK(GetPendulumConfigError(config) == nullptr);
  config.drive_enabled = true;
  ExpectInvalidConfig(config);
  config.drive_torque_amplitude_n_m = 0.0f;
  CHECK(GetPendulumConfigError(config) == nullptr);

  config = PendulumConfig{};
  const PendulumState valid_state = MakeInitialPendulumState(config);
  constexpr std::array<float PendulumState::*, 3> state_fields = {
      &PendulumState::angle_radians,
      &PendulumState::angular_velocity_rad_s,
      &PendulumState::angular_acceleration_rad_s2,
  };
  for (float invalid_value : invalid_values) {
    for (float PendulumState::* field : state_fields) {
      PendulumState state = valid_state;
      state.*field = invalid_value;
      CHECK(GetPendulumStateError(config, state) != nullptr);
    }
  }
  for (double invalid_value : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
    PendulumState invalid_time_state = valid_state;
    invalid_time_state.time_seconds = invalid_value;
    CHECK(GetPendulumStateError(config, invalid_time_state) != nullptr);
  }
  PendulumState state = valid_state;
  state.angle_radians = std::numeric_limits<float>::quiet_NaN();
  CHECK(!StepPendulum(config, kPendulumPhysicsStep, &state));
  state = valid_state;
  state.angular_velocity_rad_s = std::numeric_limits<float>::infinity();
  CHECK(!StepPendulum(config, kPendulumPhysicsStep, &state));
  CHECK(!StepPendulum(config, kPendulumPhysicsStep, nullptr));
  state = valid_state;
  CHECK(!StepPendulum(config, 0.0f, &state));
  CHECK(!StepPendulum(config, std::numeric_limits<float>::quiet_NaN(), &state));
  PendulumConfig invalid_config = config;
  invalid_config.gravity_m_s2 = 9.81f;
  state = valid_state;
  CHECK(!StepPendulum(invalid_config, kPendulumPhysicsStep, &state));
  CHECK(state.time_seconds == valid_state.time_seconds);

  config = PendulumConfig{};
  config.initial_angle_degrees = 0.0f;
  config.drive_enabled = true;
  config.drive_torque_amplitude_n_m = 0.1f;
  config.drive_angular_frequency_rad_s = 1.0f;
  state = MakeInitialPendulumState(config);
  CHECK(StepPendulum(config, 0.25f, &state));
  state = MakeInitialPendulumState(config);
  const PendulumState before_rejected_step = state;
  CHECK(!StepPendulum(config, 0.2501f, &state));
  CheckStateExactlyEqual(state, before_rejected_step);
}

void TestLongRunFiniteAndDeterministic() {
  PendulumConfig config;
  config.rod_length_m = 0.2f;
  config.rod_mass_kg = 0.2f;
  config.counterweight_mass_kg = 0.1f;
  config.counterweight_distance_m = 0.15f;
  config.counterweight_charge_c = 1.0f;
  config.initial_angle_degrees = 170.0f;
  config.initial_angular_velocity_rad_s = 10.0f;
  config.gravity_m_s2 = 10.0f;
  config.electric_field_strength_n_c = 10.0f;
  config.electric_field_angle_degrees = -73.0f;
  config.damping_enabled = true;
  config.damping_coefficient_n_m_s = 0.01f;
  config.drive_enabled = true;
  config.drive_torque_amplitude_n_m = 0.005f;
  config.drive_angular_frequency_rad_s = 2.3f;
  CHECK(GetPendulumConfigError(config) == nullptr);

  PendulumState first = MakeInitialPendulumState(config);
  PendulumState second = first;
  for (int step = 0; step < 60 * 240; ++step) {
    CHECK(StepPendulum(config, kPendulumPhysicsStep, &first));
    CHECK(StepPendulum(config, kPendulumPhysicsStep, &second));
    CHECK(GetPendulumStateError(config, first) == nullptr);
    CHECK(IsPendulumDerivedFinite(CalculatePendulumDerived(config, first)));
    CHECK(std::abs(first.angle_radians) <= kPendulumPi + kTestTolerance);
  }
  CHECK(Near(first.angle_radians, second.angle_radians, 0.000001f));
  CHECK(Near(first.angular_velocity_rad_s, second.angular_velocity_rad_s,
             0.000001f));
  CHECK(Near(first.angular_acceleration_rad_s2,
             second.angular_acceleration_rad_s2, 0.000001f));
  CHECK(NearTime(first.time_seconds, second.time_seconds));
}

void TestLargeSimulationTimeStillAdvances() {
  const PendulumConfig config;
  PendulumState state = MakeInitialPendulumState(config);
  state.time_seconds = 1000000.0;
  const double previous_time = state.time_seconds;
  CHECK(StepPendulum(config, kPendulumPhysicsStep, &state));
  CHECK(state.time_seconds > previous_time);
  CHECK(NearTime(state.time_seconds - previous_time,
                 static_cast<double>(kPendulumPhysicsStep), 1e-9));
}

using TestFunction = void (*)();

struct NamedTest {
  const char* name;
  TestFunction function;
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"formula anchors and semi-implicit step",
                TestFormulaAnchorsAndSemiImplicitStep},
      NamedTest{"drive phase, power, and step sampling",
                TestDrivePhasePowerAndStepSampling},
      NamedTest{"disabled and zero drive preserve legacy trajectory",
                TestDisabledAndZeroDrivePreserveLegacyTrajectory},
      NamedTest{"resonant response", TestResonantResponse},
      NamedTest{"history lookup", TestHistoryLookup},
      NamedTest{"electric direction, charge, and counterweight position",
                TestElectricDirectionChargeAndCounterweightPosition},
      NamedTest{"small-angle period", TestSmallAnglePeriod},
      NamedTest{"conservative energy and damping loss",
                TestConservativeEnergyAndDampingLoss},
      NamedTest{"validation, non-finite, and dangerous inputs",
                TestValidationNonFiniteAndDangerousInputs},
      NamedTest{"long-run finite and deterministic",
                TestLongRunFiniteAndDeterministic},
      NamedTest{"large simulation time advances",
                TestLargeSimulationTimeStillAdvances},
  };

  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << check_count << " checks passed\n";
  return 0;
}
