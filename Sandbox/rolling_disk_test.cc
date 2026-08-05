#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "rolling_disk_model.h"

namespace {

using tiny2d::sandbox::CalculateRollingDiskDerived;
using tiny2d::sandbox::FindRollingDiskState;
using tiny2d::sandbox::GetRollingDiskConfigError;
using tiny2d::sandbox::GetRollingDiskStateError;
using tiny2d::sandbox::MakeInitialRollingDiskState;
using tiny2d::sandbox::RollingContactMode;
using tiny2d::sandbox::RollingDiskConfig;
using tiny2d::sandbox::RollingDiskDerived;
using tiny2d::sandbox::RollingDiskKind;
using tiny2d::sandbox::RollingDiskState;
using tiny2d::sandbox::RollingDiskStatus;
using tiny2d::sandbox::StepRollingDisk;

constexpr float kStep = 1.0f / 240.0f;
constexpr float kTolerance = 0.0001f;

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

bool Near(float actual, float expected, float tolerance = kTolerance) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0f, std::abs(actual), std::abs(expected)});
}

bool SameState(const RollingDiskState& a, const RollingDiskState& b) {
  return a.distance_down_ramp_m == b.distance_down_ramp_m &&
         a.velocity_down_ramp_m_s == b.velocity_down_ramp_m_s &&
         a.angle_radians == b.angle_radians &&
         a.angular_velocity_rad_s == b.angular_velocity_rad_s &&
         a.time_seconds == b.time_seconds &&
         a.dissipated_energy_j == b.dissipated_energy_j &&
         a.contact_mode == b.contact_mode && a.status == b.status;
}

bool SameOldDerived(const RollingDiskDerived& a, const RollingDiskDerived& b) {
  return a.moment_of_inertia_kg_m2 == b.moment_of_inertia_kg_m2 &&
         a.tangential_external_force_n == b.tangential_external_force_n &&
         a.normal_force_n == b.normal_force_n &&
         a.friction_force_n == b.friction_force_n &&
         a.acceleration_down_ramp_m_s2 == b.acceleration_down_ramp_m_s2 &&
         a.angular_acceleration_rad_s2 == b.angular_acceleration_rad_s2 &&
         a.slip_velocity_m_s == b.slip_velocity_m_s &&
         a.translational_kinetic_energy_j == b.translational_kinetic_energy_j &&
         a.rotational_kinetic_energy_j == b.rotational_kinetic_energy_j &&
         a.potential_energy_j == b.potential_energy_j &&
         a.mechanical_energy_j == b.mechanical_energy_j &&
         a.accounted_energy_j == b.accounted_energy_j;
}

void ExpectInvalidConfig(const RollingDiskConfig& config) {
  CHECK(GetRollingDiskConfigError(config) != nullptr);
}

void TestAnalyticRollingAccelerations() {
  RollingDiskConfig config;
  config.ramp_length_m = 100.0f;
  config.ramp_angle_degrees = 30.0f;
  config.mass_kg = 1.0f;
  config.radius_m = 0.25f;
  config.static_friction_coefficient = 0.4f;
  config.kinetic_friction_coefficient = 0.3f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  config.gravity_m_s2 = 10.0f;

  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.contact_mode == RollingContactMode::kRolling);
  auto derived = CalculateRollingDiskDerived(config, state);
  CHECK(Near(derived.moment_of_inertia_kg_m2, 0.03125f));
  CHECK(Near(derived.tangential_external_force_n, 5.0f));
  CHECK(Near(derived.normal_force_n, 8.660254f));
  CHECK(Near(derived.friction_force_n, -1.6666667f));
  CHECK(Near(derived.acceleration_down_ramp_m_s2, 3.3333333f));
  CHECK(Near(derived.angular_acceleration_rad_s2, 13.333333f));

  config.kind = RollingDiskKind::kHoop;
  state = MakeInitialRollingDiskState(config);
  derived = CalculateRollingDiskDerived(config, state);
  CHECK(Near(derived.moment_of_inertia_kg_m2, 0.0625f));
  CHECK(Near(derived.friction_force_n, -2.5f));
  CHECK(Near(derived.acceleration_down_ramp_m_s2, 2.5f));
  CHECK(Near(derived.angular_acceleration_rad_s2, 10.0f));
}

void TestDefaultSlidesThenRolls() {
  const RollingDiskConfig config;
  CHECK(GetRollingDiskConfigError(config) == nullptr);
  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.contact_mode == RollingContactMode::kSliding);
  const float initial_accounted_energy =
      CalculateRollingDiskDerived(config, state).accounted_energy_j;
  float previous_dissipation = 0.0f;

  bool reached_rolling = false;
  for (int step = 0; step < 5 * 240; ++step) {
    CHECK(StepRollingDisk(config, kStep, &state));
    CHECK(state.dissipated_energy_j + kTolerance >= previous_dissipation);
    previous_dissipation = state.dissipated_energy_j;
    if (state.contact_mode == RollingContactMode::kRolling) {
      reached_rolling = true;
      break;
    }
  }

  CHECK(reached_rolling);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(state.distance_down_ramp_m < config.ramp_length_m);
  CHECK(state.dissipated_energy_j > 0.0f);
  CHECK(Near(state.velocity_down_ramp_m_s,
             config.radius_m * state.angular_velocity_rad_s, 0.00001f));
  const float final_accounted_energy =
      CalculateRollingDiskDerived(config, state).accounted_energy_j;
  CHECK(Near(final_accounted_energy, initial_accounted_energy, 0.0001f));
}

void TestInsufficientStaticFrictionKeepsSliding() {
  RollingDiskConfig config;
  config.ramp_length_m = 100.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  config.static_friction_coefficient = 0.05f;
  config.kinetic_friction_coefficient = 0.05f;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.contact_mode == RollingContactMode::kSliding);
  CHECK(StepRollingDisk(config, kStep, &state));
  CHECK(state.contact_mode == RollingContactMode::kSliding);
  CHECK(std::abs(CalculateRollingDiskDerived(config, state).slip_velocity_m_s) >
        0.0f);
}

void TestConservativeRollingEnergy() {
  RollingDiskConfig config;
  config.ramp_length_m = 1000.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  const float initial_energy =
      CalculateRollingDiskDerived(config, state).mechanical_energy_j;
  float maximum_relative_error = 0.0f;
  for (int step = 0; step < 5 * 240; ++step) {
    CHECK(StepRollingDisk(config, kStep, &state));
    const float energy =
        CalculateRollingDiskDerived(config, state).mechanical_energy_j;
    maximum_relative_error =
        std::max(maximum_relative_error,
                 std::abs(energy - initial_energy) / std::abs(initial_energy));
  }
  CHECK(state.contact_mode == RollingContactMode::kRolling);
  CHECK(state.dissipated_energy_j == 0.0f);
  CHECK(maximum_relative_error < 0.00001f);
}

void TestElectricFieldProjectionAndAirborneDetection() {
  RollingDiskConfig config;
  config.ramp_length_m = 100.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.electric_field_enabled = true;
  config.charge_c = 1.0f;
  config.electric_field_strength_n_c = 10.0f;
  config.electric_field_angle_degrees = 180.0f;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  const auto assisted = CalculateRollingDiskDerived(config, state);
  CHECK(assisted.tangential_external_force_n >
        config.mass_kg * config.gravity_m_s2 * 0.5f);

  config.electric_field_strength_n_c = 100.0f;
  config.electric_field_angle_degrees = 90.0f;
  CHECK(GetRollingDiskConfigError(config) == nullptr);
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kAirborne);
  const RollingDiskState airborne = state;
  CHECK(!StepRollingDisk(config, kStep, &state));
  CHECK(SameState(state, airborne));

  config.electric_field_enabled = false;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
}

void TestMagneticBaselineProjectionAndSigns() {
  RollingDiskConfig baseline;
  baseline.ramp_length_m = 1000.0f;
  baseline.initial_distance_down_ramp_m = 1.0f;

  RollingDiskConfig disabled = baseline;
  disabled.charge_c = 2.0f;
  disabled.magnetic_field_z_t = 100.0f;
  RollingDiskConfig zero_charge = baseline;
  zero_charge.magnetic_field_enabled = true;
  zero_charge.magnetic_field_z_t = 100.0f;
  RollingDiskConfig zero_field = baseline;
  zero_field.magnetic_field_enabled = true;
  zero_field.charge_c = 2.0f;
  zero_field.magnetic_field_z_t = 0.0f;
  const std::array no_effect_configs = {disabled, zero_charge, zero_field};

  for (const RollingDiskConfig& config : no_effect_configs) {
    RollingDiskState expected = MakeInitialRollingDiskState(baseline);
    RollingDiskState actual = MakeInitialRollingDiskState(config);
    CHECK(SameState(actual, expected));
    RollingDiskDerived expected_derived =
        CalculateRollingDiskDerived(baseline, expected);
    RollingDiskDerived actual_derived =
        CalculateRollingDiskDerived(config, actual);
    CHECK(SameOldDerived(actual_derived, expected_derived));
    CHECK(actual_derived.magnetic_force_outward_n == 0.0f);
    for (int step = 0; step < 240; ++step) {
      CHECK(StepRollingDisk(baseline, kStep, &expected));
      CHECK(StepRollingDisk(config, kStep, &actual));
      CHECK(SameState(actual, expected));
      expected_derived = CalculateRollingDiskDerived(baseline, expected);
      actual_derived = CalculateRollingDiskDerived(config, actual);
      CHECK(SameOldDerived(actual_derived, expected_derived));
      CHECK(actual_derived.magnetic_force_outward_n == 0.0f);
    }
  }

  RollingDiskConfig config;
  config.ramp_length_m = 100.0f;
  config.ramp_angle_degrees = 30.0f;
  config.mass_kg = 1.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 2.0f;
  config.static_friction_coefficient = 0.0f;
  config.kinetic_friction_coefficient = 0.0f;
  config.gravity_m_s2 = 10.0f;
  config.charge_c = 1.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 1.0f;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  RollingDiskDerived derived = CalculateRollingDiskDerived(config, state);
  CHECK(Near(derived.magnetic_force_outward_n, 2.0f, 0.00001f));
  CHECK(Near(derived.normal_force_n, 6.660254f, 0.00001f));

  RollingDiskConfig no_field = config;
  no_field.magnetic_field_enabled = false;
  const RollingDiskDerived no_field_derived = CalculateRollingDiskDerived(
      no_field, MakeInitialRollingDiskState(no_field));
  CHECK(derived.tangential_external_force_n ==
        no_field_derived.tangential_external_force_n);
  CHECK(derived.acceleration_down_ramp_m_s2 ==
        no_field_derived.acceleration_down_ramp_m_s2);
  CHECK(derived.angular_acceleration_rad_s2 == 0.0f);
  CHECK(no_field_derived.angular_acceleration_rad_s2 == 0.0f);

  config.charge_c = -1.0f;
  derived =
      CalculateRollingDiskDerived(config, MakeInitialRollingDiskState(config));
  CHECK(Near(derived.magnetic_force_outward_n, -2.0f, 0.00001f));
  config.charge_c = 1.0f;
  config.magnetic_field_z_t = -1.0f;
  derived =
      CalculateRollingDiskDerived(config, MakeInitialRollingDiskState(config));
  CHECK(Near(derived.magnetic_force_outward_n, -2.0f, 0.00001f));
  config.magnetic_field_z_t = 1.0f;
  config.initial_velocity_down_ramp_m_s = -2.0f;
  derived =
      CalculateRollingDiskDerived(config, MakeInitialRollingDiskState(config));
  CHECK(Near(derived.magnetic_force_outward_n, -2.0f, 0.00001f));
  config.charge_c = -1.0f;
  config.magnetic_field_z_t = -1.0f;
  config.initial_velocity_down_ramp_m_s = 2.0f;
  derived =
      CalculateRollingDiskDerived(config, MakeInitialRollingDiskState(config));
  CHECK(Near(derived.magnetic_force_outward_n, 2.0f, 0.00001f));
  config.charge_c = -1.0f;
  config.magnetic_field_z_t = 1.0f;
  config.initial_velocity_down_ramp_m_s = -2.0f;
  derived =
      CalculateRollingDiskDerived(config, MakeInitialRollingDiskState(config));
  CHECK(Near(derived.magnetic_force_outward_n, 2.0f, 0.00001f));
  config.charge_c = 1.0f;
  config.magnetic_field_z_t = -1.0f;
  config.initial_velocity_down_ramp_m_s = -2.0f;
  derived =
      CalculateRollingDiskDerived(config, MakeInitialRollingDiskState(config));
  CHECK(Near(derived.magnetic_force_outward_n, 2.0f, 0.00001f));
}

void TestMagneticFieldChangesContactFriction() {
  RollingDiskConfig outward;
  outward.ramp_length_m = 100.0f;
  outward.ramp_angle_degrees = 30.0f;
  outward.mass_kg = 1.0f;
  outward.radius_m = 0.25f;
  outward.static_friction_coefficient = 0.2f;
  outward.kinetic_friction_coefficient = 0.1f;
  outward.initial_distance_down_ramp_m = 1.0f;
  outward.initial_velocity_down_ramp_m_s = 2.0f;
  outward.initial_angular_velocity_rad_s = 8.0f;
  outward.gravity_m_s2 = 10.0f;
  outward.charge_c = 1.0f;
  outward.magnetic_field_enabled = true;
  outward.magnetic_field_z_t = 1.0f;

  RollingDiskConfig inward = outward;
  inward.magnetic_field_z_t = -1.0f;
  const RollingDiskState outward_state = MakeInitialRollingDiskState(outward);
  const RollingDiskState inward_state = MakeInitialRollingDiskState(inward);
  const RollingDiskDerived outward_derived =
      CalculateRollingDiskDerived(outward, outward_state);
  const RollingDiskDerived inward_derived =
      CalculateRollingDiskDerived(inward, inward_state);

  CHECK(outward_state.contact_mode == RollingContactMode::kSliding);
  CHECK(inward_state.contact_mode == RollingContactMode::kRolling);
  CHECK(outward_derived.normal_force_n < inward_derived.normal_force_n);
  CHECK(std::abs(outward_derived.friction_force_n) <
        std::abs(inward_derived.friction_force_n));
}

void TestMagneticAirborneEvents() {
  RollingDiskConfig config;
  config.ramp_length_m = 100.0f;
  config.ramp_angle_degrees = 30.0f;
  config.mass_kg = 1.0f;
  config.radius_m = 0.25f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 2.0f;
  config.initial_angular_velocity_rad_s = 8.0f;
  config.gravity_m_s2 = 10.0f;
  config.charge_c = 1.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 5.0f;

  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kAirborne);
  const RollingDiskState initial_airborne = state;
  CHECK(!StepRollingDisk(config, kStep, &state));
  CHECK(SameState(state, initial_airborne));

  constexpr float kContactBoundaryField = 4.330127f;
  config.magnetic_field_z_t = kContactBoundaryField * 0.9999f;
  CHECK(MakeInitialRollingDiskState(config).status ==
        RollingDiskStatus::kActive);
  config.magnetic_field_z_t = kContactBoundaryField * 1.0001f;
  CHECK(MakeInitialRollingDiskState(config).status ==
        RollingDiskStatus::kAirborne);

  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  config.static_friction_coefficient = 5.0f;
  config.kinetic_friction_coefficient = 0.3f;
  config.magnetic_field_z_t = 1.0f;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  bool became_airborne = false;
  for (int step = 0; step < 5 * 240; ++step) {
    if (!StepRollingDisk(config, kStep, &state)) {
      CHECK(state.status == RollingDiskStatus::kAirborne);
      CHECK(GetRollingDiskStateError(config, state) == nullptr);
      became_airborne = true;
      break;
    }
  }
  CHECK(became_airborne);
  CHECK(state.time_seconds > 0.0f && state.time_seconds < 5.0f);
  CHECK(state.distance_down_ramp_m > 0.0f &&
        state.distance_down_ramp_m < config.ramp_length_m);
}

void TestAirborneWinsEndpointTie() {
  RollingDiskConfig config;
  config.ramp_length_m = 0.0075f;
  config.ramp_angle_degrees = 0.0f;
  config.initial_distance_down_ramp_m = 0.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  config.gravity_m_s2 = 10.0f;
  config.charge_c = 1.0f;
  config.electric_field_enabled = true;
  config.electric_field_strength_n_c = 1.0f;
  config.electric_field_angle_degrees = 180.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 100.0f;

  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(!StepRollingDisk(config, 0.2f, &state));
  CHECK(state.status == RollingDiskStatus::kAirborne);
  CHECK(Near(state.time_seconds, 0.15f, 0.00001f));
}

void TestAirborneWinsSlipTransitionTie() {
  RollingDiskConfig config;
  config.ramp_length_m = 100.0f;
  config.ramp_angle_degrees = 0.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 2.7999997f;
  config.kinetic_friction_coefficient = 0.15f;
  config.gravity_m_s2 = 10.0f;
  config.charge_c = 1.0f;
  config.electric_field_enabled = true;
  config.electric_field_strength_n_c = 1.0f;
  config.electric_field_angle_degrees = 0.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 100.0f;

  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(state.contact_mode == RollingContactMode::kSliding);
  CHECK(!StepRollingDisk(config, 0.25f, &state));
  CHECK(state.status == RollingDiskStatus::kAirborne);
  CHECK(Near(state.time_seconds, 0.2f, 0.00001f));
}

void TestMagneticEnergyAndStepConvergence() {
  RollingDiskConfig conservative;
  conservative.ramp_length_m = 1000.0f;
  conservative.ramp_angle_degrees = 30.0f;
  conservative.initial_distance_down_ramp_m = 1.0f;
  conservative.initial_velocity_down_ramp_m_s = 1.0f;
  conservative.static_friction_coefficient = 0.0f;
  conservative.kinetic_friction_coefficient = 0.0f;
  conservative.gravity_m_s2 = 10.0f;
  conservative.charge_c = 1.0f;
  conservative.magnetic_field_enabled = true;
  conservative.magnetic_field_z_t = -1.0f;
  RollingDiskState state = MakeInitialRollingDiskState(conservative);
  const float initial_energy =
      CalculateRollingDiskDerived(conservative, state).mechanical_energy_j;
  float maximum_relative_drift = 0.0f;
  for (int step = 0; step < 5 * 240; ++step) {
    CHECK(StepRollingDisk(conservative, kStep, &state));
    const float energy =
        CalculateRollingDiskDerived(conservative, state).mechanical_energy_j;
    maximum_relative_drift = std::max(
        maximum_relative_drift, std::abs(energy - initial_energy) /
                                    std::max(1.0f, std::abs(initial_energy)));
  }
  CHECK(maximum_relative_drift < 0.00001f);

  RollingDiskConfig dissipative = conservative;
  dissipative.static_friction_coefficient = 0.05f;
  dissipative.kinetic_friction_coefficient = 0.05f;
  dissipative.magnetic_field_z_t = -0.2f;
  state = MakeInitialRollingDiskState(dissipative);
  const float initial_accounted =
      CalculateRollingDiskDerived(dissipative, state).accounted_energy_j;
  maximum_relative_drift = 0.0f;
  for (int step = 0; step < 5 * 240; ++step) {
    CHECK(StepRollingDisk(dissipative, kStep, &state));
    const float accounted =
        CalculateRollingDiskDerived(dissipative, state).accounted_energy_j;
    maximum_relative_drift =
        std::max(maximum_relative_drift,
                 std::abs(accounted - initial_accounted) /
                     std::max(1.0f, std::abs(initial_accounted)));
  }
  CHECK(maximum_relative_drift < 0.001f);

  RollingDiskState coarse = MakeInitialRollingDiskState(dissipative);
  RollingDiskState fine = coarse;
  for (int step = 0; step < 240; ++step) {
    CHECK(StepRollingDisk(dissipative, 1.0f / 240.0f, &coarse));
  }
  for (int step = 0; step < 480; ++step) {
    CHECK(StepRollingDisk(dissipative, 1.0f / 480.0f, &fine));
  }
  CHECK(coarse.status == RollingDiskStatus::kActive);
  CHECK(fine.status == RollingDiskStatus::kActive);
  CHECK(Near(coarse.distance_down_ramp_m, fine.distance_down_ramp_m, 0.005f));
  CHECK(
      Near(coarse.velocity_down_ramp_m_s, fine.velocity_down_ramp_m_s, 0.005f));
  CHECK(
      Near(coarse.angular_velocity_rad_s, fine.angular_velocity_rad_s, 0.005f));
  CHECK(Near(coarse.dissipated_energy_j, fine.dissipated_energy_j, 0.005f));
}

void TestHistoryLookup() {
  std::vector<RollingDiskState> history(3);
  history[0].time_seconds = 0.0f;
  history[1].time_seconds = 1.0f;
  history[2].time_seconds = 2.0f;

  CHECK(FindRollingDiskState({}, 0.0f) == nullptr);
  CHECK(FindRollingDiskState(
            history, std::numeric_limits<float>::quiet_NaN()) == nullptr);
  CHECK(FindRollingDiskState(history, -1.0f) == &history[0]);
  CHECK(FindRollingDiskState(history, 0.5f) == &history[0]);
  CHECK(FindRollingDiskState(history, 0.6f) == &history[1]);
  CHECK(FindRollingDiskState(history, 3.0f) == &history[2]);
}

void TestRampEndpointStates() {
  RollingDiskConfig config;
  config.ramp_length_m = 1.0f;
  config.ramp_angle_degrees = 0.0f;
  config.initial_distance_down_ramp_m = 0.9f;
  config.initial_velocity_down_ramp_m_s = 2.0f;
  config.initial_angular_velocity_rad_s =
      config.initial_velocity_down_ramp_m_s / config.radius_m;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(StepRollingDisk(config, 0.1f, &state));
  CHECK(state.status == RollingDiskStatus::kReachedBottom);
  CHECK(state.distance_down_ramp_m == config.ramp_length_m);
  CHECK(Near(state.time_seconds, 0.05f, 0.000001f));
  CHECK(Near(state.angle_radians, 0.4f, 0.000001f));
  CHECK(!StepRollingDisk(config, kStep, &state));

  config.initial_distance_down_ramp_m = 0.1f;
  config.initial_velocity_down_ramp_m_s = -2.0f;
  config.initial_angular_velocity_rad_s =
      config.initial_velocity_down_ramp_m_s / config.radius_m;
  state = MakeInitialRollingDiskState(config);
  CHECK(StepRollingDisk(config, 0.1f, &state));
  CHECK(state.status == RollingDiskStatus::kReachedTop);
  CHECK(state.distance_down_ramp_m == 0.0f);
  CHECK(Near(state.time_seconds, 0.05f, 0.000001f));
  CHECK(Near(state.angle_radians, -0.4f, 0.000001f));

  config.initial_distance_down_ramp_m = config.ramp_length_m;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kReachedBottom);

  config.initial_velocity_down_ramp_m_s = -2.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(state.contact_mode == RollingContactMode::kSliding);
  CHECK(StepRollingDisk(config, 0.1f, &state));
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(state.distance_down_ramp_m < config.ramp_length_m);

  config.initial_distance_down_ramp_m = 0.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kReachedTop);
  config.initial_velocity_down_ramp_m_s = 1.0f;
  config.initial_angular_velocity_rad_s =
      config.initial_velocity_down_ramp_m_s / config.radius_m;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(state.contact_mode == RollingContactMode::kRolling);

  config = RollingDiskConfig{};
  config.initial_distance_down_ramp_m = 0.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(state.contact_mode == RollingContactMode::kRolling);
  CHECK(StepRollingDisk(config, kStep, &state));
  CHECK(state.distance_down_ramp_m > 0.0f);
}

void TestEndpointUsesFirstTrajectoryHit() {
  RollingDiskConfig config;
  config.ramp_length_m = 10.0f;
  config.ramp_angle_degrees = 30.0f;
  config.gravity_m_s2 = 30.0f;
  config.initial_distance_down_ramp_m = 0.1f;
  config.initial_velocity_down_ramp_m_s = -2.0f;
  config.initial_angular_velocity_rad_s =
      config.initial_velocity_down_ramp_m_s / config.radius_m;
  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.contact_mode == RollingContactMode::kRolling);

  // s(t) = 0.1 - 2t + 5t^2 crosses the top and then returns inside before
  // t = 0.5. The first root, not the final in-range position, ends the step.
  constexpr float kFirstHitTime = 0.058578644f;
  CHECK(StepRollingDisk(config, 0.5f, &state));
  CHECK(state.status == RollingDiskStatus::kReachedTop);
  CHECK(state.distance_down_ramp_m == 0.0f);
  CHECK(Near(state.time_seconds, kFirstHitTime, 0.000001f));
  CHECK(Near(state.velocity_down_ramp_m_s, -1.41421356f, 0.000001f));
  CHECK(Near(state.angle_radians, -0.4f, 0.000001f));
}

void TestInvalidAndExtremeInputs() {
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  constexpr std::array<float RollingDiskConfig::*, 14> fields = {
      &RollingDiskConfig::ramp_length_m,
      &RollingDiskConfig::ramp_angle_degrees,
      &RollingDiskConfig::mass_kg,
      &RollingDiskConfig::radius_m,
      &RollingDiskConfig::static_friction_coefficient,
      &RollingDiskConfig::kinetic_friction_coefficient,
      &RollingDiskConfig::initial_distance_down_ramp_m,
      &RollingDiskConfig::initial_velocity_down_ramp_m_s,
      &RollingDiskConfig::initial_angular_velocity_rad_s,
      &RollingDiskConfig::gravity_m_s2,
      &RollingDiskConfig::charge_c,
      &RollingDiskConfig::electric_field_strength_n_c,
      &RollingDiskConfig::electric_field_angle_degrees,
      &RollingDiskConfig::magnetic_field_z_t,
  };
  for (float invalid : invalid_values) {
    for (float RollingDiskConfig::* field : fields) {
      RollingDiskConfig config;
      config.*field = invalid;
      ExpectInvalidConfig(config);
    }
  }

  RollingDiskConfig config;
  const int invalid_kind = 99;
  static_assert(sizeof(invalid_kind) == sizeof(config.kind));
  std::memcpy(&config.kind, &invalid_kind, sizeof(config.kind));
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.ramp_length_m = 0.0f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.mass_kg = 0.0f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.radius_m = 0.0f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.ramp_angle_degrees = -0.01f;
  ExpectInvalidConfig(config);
  config.ramp_angle_degrees = 90.0f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.kinetic_friction_coefficient =
      config.static_friction_coefficient + 0.01f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.static_friction_coefficient = 5.0f;
  config.kinetic_friction_coefficient = 5.0f;
  CHECK(GetRollingDiskConfigError(config) == nullptr);
  config.static_friction_coefficient = 5.01f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.initial_distance_down_ramp_m = config.ramp_length_m + 0.01f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.gravity_m_s2 = 0.0f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.electric_field_strength_n_c = -0.01f;
  ExpectInvalidConfig(config);
  config = RollingDiskConfig{};
  config.electric_field_angle_degrees = 180.01f;
  ExpectInvalidConfig(config);

  config = RollingDiskConfig{};
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = -100.0f;
  CHECK(GetRollingDiskConfigError(config) == nullptr);
  config.magnetic_field_z_t = 100.0f;
  CHECK(GetRollingDiskConfigError(config) == nullptr);
  config.magnetic_field_z_t = -100.01f;
  ExpectInvalidConfig(config);
  config.magnetic_field_z_t = 100.01f;
  ExpectInvalidConfig(config);
  config.magnetic_field_enabled = false;
  ExpectInvalidConfig(config);

  config = RollingDiskConfig{};
  config.ramp_length_m = 100.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.initial_velocity_down_ramp_m_s = 50.0f;
  config.charge_c = std::numeric_limits<float>::max() / 4.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 100.0f;
  ExpectInvalidConfig(config);

  config = RollingDiskConfig{};
  config.ramp_length_m = 100.0f;
  config.initial_distance_down_ramp_m = 1.0f;
  config.charge_c = 2.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 100.0f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  config.initial_angular_velocity_rad_s = 0.0f;
  RollingDiskState overflow_state = MakeInitialRollingDiskState(config);
  CHECK(overflow_state.status == RollingDiskStatus::kActive);
  overflow_state.velocity_down_ramp_m_s = std::numeric_limits<float>::max();
  const RollingDiskState unchanged_overflow_state = overflow_state;
  CHECK(!StepRollingDisk(config, kStep, &overflow_state));
  CHECK(SameState(overflow_state, unchanged_overflow_state));

  config = RollingDiskConfig{};
  config.mass_kg = std::numeric_limits<float>::max();
  config.radius_m = std::numeric_limits<float>::max();
  ExpectInvalidConfig(config);

  config = RollingDiskConfig{};
  config.ramp_angle_degrees = 0.0f;
  config.electric_field_enabled = true;
  config.charge_c = std::numeric_limits<float>::max() / 4.0f;
  config.electric_field_strength_n_c = 1.0f;
  config.electric_field_angle_degrees = -90.0f;
  config.static_friction_coefficient = 5.0f;
  config.kinetic_friction_coefficient = 5.0f;
  ExpectInvalidConfig(config);

  config = RollingDiskConfig{};
  config.mass_kg = 0.000001f;
  config.radius_m = 0.000001f;
  config.ramp_angle_degrees = 89.999f;
  config.initial_distance_down_ramp_m = 0.5f;
  config.initial_velocity_down_ramp_m_s = 0.0f;
  CHECK(GetRollingDiskConfigError(config) == nullptr);
  RollingDiskState state = MakeInitialRollingDiskState(config);
  CHECK(state.status == RollingDiskStatus::kActive);
  CHECK(GetRollingDiskStateError(config, state) == nullptr);
  CHECK(CalculateRollingDiskDerived(config, state).normal_force_n > 0.0f);
  CHECK(StepRollingDisk(config, kStep, &state));

  config = RollingDiskConfig{};
  state = MakeInitialRollingDiskState(config);
  const RollingDiskState original = state;
  CHECK(!StepRollingDisk(config, 0.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepRollingDisk(config, std::numeric_limits<float>::quiet_NaN(),
                         &state));
  CHECK(SameState(state, original));
  CHECK(!StepRollingDisk(config, kStep, nullptr));

  state.time_seconds = std::numeric_limits<float>::infinity();
  CHECK(GetRollingDiskStateError(config, state) != nullptr);
  CHECK(!StepRollingDisk(config, kStep, &state));
}

void TestLongRunIsFiniteAndDeterministic() {
  RollingDiskConfig config;
  config.ramp_length_m = 1000000.0f;
  config.initial_distance_down_ramp_m = 10.0f;
  config.electric_field_enabled = true;
  config.charge_c = -0.5f;
  config.electric_field_strength_n_c = 2.0f;
  config.electric_field_angle_degrees = -40.0f;
  config.magnetic_field_enabled = true;
  config.magnetic_field_z_t = 0.05f;
  RollingDiskState first = MakeInitialRollingDiskState(config);
  RollingDiskState second = first;

  for (int step = 0; step < 60 * 240; ++step) {
    CHECK(StepRollingDisk(config, kStep, &first));
    CHECK(StepRollingDisk(config, kStep, &second));
    CHECK(GetRollingDiskStateError(config, first) == nullptr);
  }
  CHECK(SameState(first, second));
}

using TestFunction = void (*)();

struct NamedTest {
  const char* name;
  TestFunction function;
};

}  // namespace

int main() {
  const std::array tests = {
      NamedTest{"analytic rolling accelerations",
                TestAnalyticRollingAccelerations},
      NamedTest{"default slides then rolls", TestDefaultSlidesThenRolls},
      NamedTest{"insufficient static friction keeps sliding",
                TestInsufficientStaticFrictionKeepsSliding},
      NamedTest{"conservative rolling energy", TestConservativeRollingEnergy},
      NamedTest{"electric projection and airborne detection",
                TestElectricFieldProjectionAndAirborneDetection},
      NamedTest{"magnetic baseline, projection, and signs",
                TestMagneticBaselineProjectionAndSigns},
      NamedTest{"magnetic field changes contact friction",
                TestMagneticFieldChangesContactFriction},
      NamedTest{"magnetic airborne events", TestMagneticAirborneEvents},
      NamedTest{"airborne wins endpoint tie", TestAirborneWinsEndpointTie},
      NamedTest{"airborne wins slip-transition tie",
                TestAirborneWinsSlipTransitionTie},
      NamedTest{"magnetic energy and step convergence",
                TestMagneticEnergyAndStepConvergence},
      NamedTest{"history lookup", TestHistoryLookup},
      NamedTest{"ramp endpoint states", TestRampEndpointStates},
      NamedTest{"endpoint uses first trajectory hit",
                TestEndpointUsesFirstTrajectoryHit},
      NamedTest{"invalid and extreme inputs", TestInvalidAndExtremeInputs},
      NamedTest{"long run finite and deterministic",
                TestLongRunIsFiniteAndDeterministic},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  return 0;
}
