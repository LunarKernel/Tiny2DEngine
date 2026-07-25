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
  constexpr std::array<float RollingDiskConfig::*, 13> fields = {
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
