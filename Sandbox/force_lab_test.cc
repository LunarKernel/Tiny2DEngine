#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "force_lab_model.h"
#include "test_support.h"

namespace {

using tiny2d::Rectangle;
using tiny2d::sandbox::CalculateForceLabDerived;
using tiny2d::sandbox::FindForceLabState;
using tiny2d::sandbox::ForceLabConfig;
using tiny2d::sandbox::ForceLabDerived;
using tiny2d::sandbox::ForceLabState;
using tiny2d::sandbox::GetForceLabCenteredPeriod;
using tiny2d::sandbox::GetForceLabConfigError;
using tiny2d::sandbox::GetForceLabStateError;
using tiny2d::sandbox::kForceLabPhysicsStep;
using tiny2d::sandbox::MakeCenteredReferenceConfig;
using tiny2d::sandbox::MakeEccentricDemoConfig;
using tiny2d::sandbox::MakeInitialForceLabState;
using tiny2d::sandbox::StepForceLab;

bool Near(double actual, double expected, double tolerance = 0.00001) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

bool SameRectangle(const Rectangle& a, const Rectangle& b) {
  return a.mass == b.mass && a.position.x == b.position.x &&
         a.position.y == b.position.y && a.velocity.x == b.velocity.x &&
         a.velocity.y == b.velocity.y && a.angle == b.angle &&
         a.angular_velocity == b.angular_velocity && a.width == b.width &&
         a.height == b.height && a.fixed_rotation == b.fixed_rotation &&
         a.charge == b.charge && a.applied_force.x == b.applied_force.x &&
         a.applied_force.y == b.applied_force.y &&
         a.applied_torque == b.applied_torque &&
         a.linear_damping_rate == b.linear_damping_rate &&
         a.angular_damping_rate == b.angular_damping_rate;
}

bool SameState(const ForceLabState& a, const ForceLabState& b) {
  return SameRectangle(a.body, b.body) && a.time_seconds == b.time_seconds &&
         a.dissipated_energy_j == b.dissipated_energy_j;
}

ForceLabConfig MakeCenteredFixture() {
  ForceLabConfig config = MakeCenteredReferenceConfig();
  config.spring_stiffness_n_m = 1.0f;
  config.spring_rest_length_m = 2.0f;
  config.initial_center_position_m = {
      config.anchor_position_m.x + config.spring_rest_length_m + 0.5f,
      config.anchor_position_m.y};
  return config;
}

void TestDefaultsAndAnalyticGeometry() {
  const ForceLabConfig centered = MakeCenteredReferenceConfig();
  CHECK(GetForceLabConfigError(centered) == nullptr);
  const ForceLabState centered_state = MakeInitialForceLabState(centered);
  const ForceLabDerived centered_derived =
      CalculateForceLabDerived(centered, centered_state);
  CHECK(Near(centered_derived.spring_length_m, 3.0));
  CHECK(Near(centered_derived.spring_extension_m, 0.5));
  CHECK(Near(centered_derived.spring_force_n.x, -2.0));
  CHECK(Near(centered_derived.spring_force_n.y, 0.0));
  CHECK(Near(centered_derived.spring_torque_n_m, 0.0));
  CHECK(Near(GetForceLabCenteredPeriod(centered), 3.141592653589793, 0.000001));

  const ForceLabConfig eccentric = MakeEccentricDemoConfig();
  CHECK(GetForceLabConfigError(eccentric) == nullptr);
  const ForceLabState eccentric_state = MakeInitialForceLabState(eccentric);
  const ForceLabDerived eccentric_derived =
      CalculateForceLabDerived(eccentric, eccentric_state);
  CHECK(Near(eccentric_derived.attachment_offset_world_m.x, 0.0));
  CHECK(Near(eccentric_derived.attachment_offset_world_m.y, -0.3));
  CHECK(eccentric_derived.spring_torque_n_m < 0.0f);
  CHECK(eccentric_derived.angular_acceleration_rad_s2 < 0.0f);
}

void TestCenteredPeriodAndConservativeEnergy() {
  const ForceLabConfig config = MakeCenteredFixture();
  CHECK(GetForceLabConfigError(config) == nullptr);
  ForceLabState state = MakeInitialForceLabState(config);
  const double theoretical_period = GetForceLabCenteredPeriod(config);
  double previous_extension =
      CalculateForceLabDerived(config, state).spring_extension_m;
  double previous_time = state.time_seconds;
  std::array<double, 2> crossings{};
  std::size_t crossing_count = 0;
  for (int step = 0; step < 10000 && crossing_count < crossings.size();
       ++step) {
    CHECK(StepForceLab(config, kForceLabPhysicsStep, &state));
    const double extension =
        CalculateForceLabDerived(config, state).spring_extension_m;
    if (previous_extension > 0.0 && extension <= 0.0 &&
        state.body.velocity.x < 0.0f) {
      const double fraction =
          previous_extension / (previous_extension - extension);
      crossings[crossing_count++] =
          previous_time + fraction * kForceLabPhysicsStep;
    }
    previous_extension = extension;
    previous_time = state.time_seconds;
  }
  CHECK(crossing_count == crossings.size());
  const double measured_period = crossings[1] - crossings[0];
  CHECK(std::abs(measured_period - theoretical_period) / theoretical_period <=
        0.01);

  state = MakeInitialForceLabState(config);
  const double initial_energy =
      CalculateForceLabDerived(config, state).mechanical_energy_j;
  double maximum_relative_drift = 0.0;
  const int step_count = static_cast<int>(
      std::ceil(100.0 * theoretical_period / kForceLabPhysicsStep));
  for (int step = 0; step < step_count; ++step) {
    CHECK(StepForceLab(config, kForceLabPhysicsStep, &state));
    const double energy =
        CalculateForceLabDerived(config, state).mechanical_energy_j;
    maximum_relative_drift =
        std::max(maximum_relative_drift,
                 std::abs(energy - initial_energy) / initial_energy);
  }
  CHECK(maximum_relative_drift < 0.005);
  CHECK(state.dissipated_energy_j == 0.0);
  CHECK(std::abs(state.body.angle) < 0.000001f);
  CHECK(std::abs(state.body.angular_velocity) < 0.000001f);
}

void TestDampingEnergyAccounting() {
  ForceLabConfig config = MakeCenteredFixture();
  config.linear_damping_n_s_m = 0.3f;
  ForceLabState state = MakeInitialForceLabState(config);
  const double initial_energy =
      CalculateForceLabDerived(config, state).mechanical_energy_j;
  double maximum_mechanical_energy = initial_energy;
  double maximum_accounted_drift = 0.0;
  for (int step = 0; step < 60 * 480; ++step) {
    CHECK(StepForceLab(config, kForceLabPhysicsStep, &state));
    const ForceLabDerived derived = CalculateForceLabDerived(config, state);
    maximum_mechanical_energy =
        std::max(maximum_mechanical_energy, derived.mechanical_energy_j);
    maximum_accounted_drift = std::max(
        maximum_accounted_drift,
        std::abs(derived.accounted_energy_j - initial_energy) / initial_energy);
  }
  CHECK(maximum_mechanical_energy <= initial_energy * 1.001);
  CHECK(maximum_accounted_drift < 0.005);
  CHECK(state.dissipated_energy_j > 0.0);
}

void TestExactLinearAndAngularDampingSteps() {
  ForceLabConfig linear = MakeCenteredFixture();
  linear.initial_center_position_m.x =
      linear.anchor_position_m.x + linear.spring_rest_length_m;
  linear.initial_velocity_m_s.x = 1.0f;
  linear.linear_damping_n_s_m = 0.4f;
  ForceLabState state = MakeInitialForceLabState(linear);
  const double linear_factor = std::exp(-linear.linear_damping_n_s_m /
                                        linear.mass_kg * kForceLabPhysicsStep);
  const double expected_linear_loss =
      0.5 * linear.mass_kg * (1.0 - linear_factor * linear_factor);
  CHECK(StepForceLab(linear, kForceLabPhysicsStep, &state));
  CHECK(Near(state.body.velocity.x, linear_factor, 0.000001));
  CHECK(Near(state.dissipated_energy_j, expected_linear_loss, 0.000001));

  ForceLabConfig angular = MakeCenteredFixture();
  angular.initial_center_position_m.x =
      angular.anchor_position_m.x + angular.spring_rest_length_m;
  angular.initial_angular_velocity_rad_s = 2.0f;
  angular.angular_damping_n_m_s_rad = 0.2f;
  state = MakeInitialForceLabState(angular);
  const double inertia =
      CalculateForceLabDerived(angular, state).moment_of_inertia_kg_m2;
  const double angular_factor = std::exp(-angular.angular_damping_n_m_s_rad /
                                         inertia * kForceLabPhysicsStep);
  const double expected_angular_velocity = 2.0 * angular_factor;
  const double expected_angular_loss =
      0.5 * inertia * 4.0 * (1.0 - angular_factor * angular_factor);
  CHECK(StepForceLab(angular, kForceLabPhysicsStep, &state));
  CHECK(Near(state.body.angular_velocity, expected_angular_velocity, 0.000001));
  CHECK(Near(state.dissipated_energy_j, expected_angular_loss, 0.000001));
}

void TestCenteredAndEccentricTorqueBehavior() {
  ForceLabConfig centered = MakeCenteredFixture();
  ForceLabState centered_state = MakeInitialForceLabState(centered);
  for (int step = 0; step < 2 * 480; ++step) {
    CHECK(StepForceLab(centered, kForceLabPhysicsStep, &centered_state));
  }
  CHECK(centered_state.body.angle == 0.0f);
  CHECK(centered_state.body.angular_velocity == 0.0f);

  const ForceLabConfig eccentric = MakeEccentricDemoConfig();
  ForceLabState eccentric_state = MakeInitialForceLabState(eccentric);
  const ForceLabDerived before =
      CalculateForceLabDerived(eccentric, eccentric_state);
  CHECK(before.spring_torque_n_m < 0.0f);
  CHECK(StepForceLab(eccentric, kForceLabPhysicsStep, &eccentric_state));
  CHECK(eccentric_state.body.angular_velocity < 0.0f);
  CHECK(eccentric_state.body.angle < 0.0f);
}

void TestValidationAndFailureAtomicity() {
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  constexpr std::array<float ForceLabConfig::*, 9> scalar_fields = {
      &ForceLabConfig::mass_kg,
      &ForceLabConfig::width_m,
      &ForceLabConfig::height_m,
      &ForceLabConfig::spring_stiffness_n_m,
      &ForceLabConfig::spring_rest_length_m,
      &ForceLabConfig::initial_angle_degrees,
      &ForceLabConfig::initial_angular_velocity_rad_s,
      &ForceLabConfig::linear_damping_n_s_m,
      &ForceLabConfig::angular_damping_n_m_s_rad,
  };
  for (float invalid : invalid_values) {
    for (float ForceLabConfig::* field : scalar_fields) {
      ForceLabConfig config;
      config.*field = invalid;
      CHECK(GetForceLabConfigError(config) != nullptr);
    }
    ForceLabConfig config;
    config.anchor_position_m.x = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.anchor_position_m.y = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.attachment_local_m.x = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.attachment_local_m.y = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.initial_center_position_m.x = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.initial_center_position_m.y = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.initial_velocity_m_s.x = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
    config = ForceLabConfig{};
    config.initial_velocity_m_s.y = invalid;
    CHECK(GetForceLabConfigError(config) != nullptr);
  }

  ForceLabConfig config;
  config.mass_kg = 0.0f;
  CHECK(GetForceLabConfigError(config) != nullptr);
  config = ForceLabConfig{};
  config.width_m = 0.09f;
  CHECK(GetForceLabConfigError(config) != nullptr);
  config = ForceLabConfig{};
  config.attachment_local_m.x = config.width_m;
  CHECK(GetForceLabConfigError(config) != nullptr);
  config = MakeCenteredReferenceConfig();
  config.initial_center_position_m = config.anchor_position_m;
  CHECK(GetForceLabConfigError(config) != nullptr);
  config = MakeCenteredReferenceConfig();
  config.anchor_position_m = {20.0f, 50.0f};
  config.initial_center_position_m = {70.1f, 50.0f};
  CHECK(GetForceLabConfigError(config) != nullptr);
  config = ForceLabConfig{};
  config.linear_damping_n_s_m = -0.01f;
  CHECK(GetForceLabConfigError(config) != nullptr);
  config = ForceLabConfig{};
  config.spring_stiffness_n_m = 1000.0f;
  config.mass_kg = 0.01f;
  CHECK(GetForceLabConfigError(config) != nullptr);

  config = MakeCenteredFixture();
  ForceLabState state = MakeInitialForceLabState(config);
  const ForceLabState original = state;
  CHECK(!StepForceLab(config, 0.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepForceLab(config, kForceLabPhysicsStep * 2.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepForceLab(config, kForceLabPhysicsStep, nullptr));
  ForceLabConfig invalid_config = config;
  invalid_config.mass_kg = 0.0f;
  CHECK(!StepForceLab(invalid_config, kForceLabPhysicsStep, &state));
  CHECK(SameState(state, original));

  state.time_seconds = std::numeric_limits<double>::quiet_NaN();
  CHECK(GetForceLabStateError(config, state) != nullptr);
  state = original;
  state.body.applied_force.x = 1.0f;
  CHECK(GetForceLabStateError(config, state) != nullptr);

  ForceLabConfig boundary = MakeCenteredReferenceConfig();
  boundary.width_m = 10.0f;
  boundary.height_m = 10.0f;
  boundary.anchor_position_m = {20.0f, 50.0f};
  boundary.spring_stiffness_n_m = 0.01f;
  boundary.spring_rest_length_m = 2.5f;
  boundary.initial_center_position_m = {69.95f, 50.0f};
  boundary.initial_velocity_m_s = {50.0f, 0.0f};
  CHECK(GetForceLabConfigError(boundary) == nullptr);
  state = MakeInitialForceLabState(boundary);
  const ForceLabState before_length_failure = state;
  CHECK(!StepForceLab(boundary, kForceLabPhysicsStep, &state));
  CHECK(SameState(state, before_length_failure));
}

void TestHistoryLookup() {
  ForceLabConfig config = MakeCenteredFixture();
  std::vector<ForceLabState> history(3, MakeInitialForceLabState(config));
  history[0].time_seconds = 0.0;
  history[1].time_seconds = 1.0;
  history[2].time_seconds = 2.0;
  CHECK(FindForceLabState({}, 0.0) == nullptr);
  CHECK(FindForceLabState(history, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);
  CHECK(FindForceLabState(history, -1.0) == &history[0]);
  CHECK(FindForceLabState(history, 0.5) == &history[0]);
  CHECK(FindForceLabState(history, 0.6) == &history[1]);
  CHECK(FindForceLabState(history, 3.0) == &history[2]);
}

void TestLongRunFiniteAndDeterministic() {
  const ForceLabConfig config = MakeEccentricDemoConfig();
  ForceLabState first = MakeInitialForceLabState(config);
  ForceLabState second = first;
  for (int step = 0; step < 60 * 480; ++step) {
    CHECK(StepForceLab(config, kForceLabPhysicsStep, &first));
    CHECK(StepForceLab(config, kForceLabPhysicsStep, &second));
    CHECK(GetForceLabStateError(config, first) == nullptr);
    const ForceLabDerived derived = CalculateForceLabDerived(config, first);
    CHECK(std::isfinite(derived.mechanical_energy_j));
    CHECK(std::isfinite(derived.accounted_energy_j));
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
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"defaults and analytic geometry",
                TestDefaultsAndAnalyticGeometry},
      NamedTest{"centered period and conservative energy",
                TestCenteredPeriodAndConservativeEnergy},
      NamedTest{"damping energy accounting", TestDampingEnergyAccounting},
      NamedTest{"exact linear and angular damping",
                TestExactLinearAndAngularDampingSteps},
      NamedTest{"centered and eccentric torque behavior",
                TestCenteredAndEccentricTorqueBehavior},
      NamedTest{"validation and failure atomicity",
                TestValidationAndFailureAtomicity},
      NamedTest{"history lookup", TestHistoryLookup},
      NamedTest{"long run finite and deterministic",
                TestLongRunFiniteAndDeterministic},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << tiny2d::test::CheckCount()
            << " checks passed\n";
  return 0;
}
