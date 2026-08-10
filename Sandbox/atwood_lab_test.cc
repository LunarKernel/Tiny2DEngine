#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "atwood_lab_model.h"
#include "test_support.h"

namespace {

using tiny2d::CircleInertiaModel;
using tiny2d::Rectangle;
using tiny2d::sandbox::AtwoodConfig;
using tiny2d::sandbox::AtwoodDerived;
using tiny2d::sandbox::AtwoodState;
using tiny2d::sandbox::CalculateAtwoodDerived;
using tiny2d::sandbox::FindAtwoodState;
using tiny2d::sandbox::GetAtwoodAnalyticalAcceleration;
using tiny2d::sandbox::GetAtwoodConfigError;
using tiny2d::sandbox::GetAtwoodStateError;
using tiny2d::sandbox::GetAtwoodTerminalIssue;
using tiny2d::sandbox::kAtwoodPhysicsStep;
using tiny2d::sandbox::MakeAtwoodBalancedDriftConfig;
using tiny2d::sandbox::MakeAtwoodDampedConfig;
using tiny2d::sandbox::MakeAtwoodReferenceConfig;
using tiny2d::sandbox::MakeInitialAtwoodState;
using tiny2d::sandbox::StepAtwood;

bool Near(double actual, double expected, double tolerance = 0.00001) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

bool SameBlock(const Rectangle& a, const Rectangle& b) {
  return a.mass == b.mass && a.position.x == b.position.x &&
         a.position.y == b.position.y && a.velocity.x == b.velocity.x &&
         a.velocity.y == b.velocity.y && a.angle == b.angle &&
         a.angular_velocity == b.angular_velocity;
}

bool SameState(const AtwoodState& a, const AtwoodState& b) {
  return SameBlock(a.block_a, b.block_a) && SameBlock(a.block_b, b.block_b) &&
         a.pulley.position.x == b.pulley.position.x &&
         a.pulley.position.y == b.pulley.position.y &&
         a.pulley.velocity.x == b.pulley.velocity.x &&
         a.pulley.velocity.y == b.pulley.velocity.y &&
         a.pulley.angle == b.pulley.angle &&
         a.pulley.angular_velocity == b.pulley.angular_velocity &&
         a.time_seconds == b.time_seconds &&
         a.dissipated_energy_j == b.dissipated_energy_j &&
         a.tension_a_n == b.tension_a_n && a.tension_b_n == b.tension_b_n &&
         a.pin_force_n.x == b.pin_force_n.x &&
         a.pin_force_n.y == b.pin_force_n.y;
}

// Per-run tolerance scale for the energy criteria: the released potential
// energy is the natural size of the experiment's energy exchange, floored
// at 1 J so a run released from rest has a well-defined budget from the
// first checkpoint.
double EnergyScale(double initial_mechanical_energy,
                   double peak_released_potential_energy) {
  return std::max(
      {initial_mechanical_energy, peak_released_potential_energy, 1.0});
}

void TestPresetsAndAnalyticalValues() {
  const AtwoodConfig reference = MakeAtwoodReferenceConfig();
  CHECK(GetAtwoodConfigError(reference) == nullptr);
  CHECK(GetAtwoodConfigError(MakeAtwoodBalancedDriftConfig()) == nullptr);
  CHECK(GetAtwoodConfigError(MakeAtwoodDampedConfig()) == nullptr);

  // a = (1.2 - 1.0) * 9.81 / (1.0 + 1.2 + 0.25) = 1.962 / 2.45.
  const double expected_acceleration = 1.962 / 2.45;
  CHECK(Near(GetAtwoodAnalyticalAcceleration(reference), expected_acceleration,
             0.0001));

  // A hoop pulley doubles I/R^2 from 0.25 to 0.5.
  AtwoodConfig hoop = reference;
  hoop.pulley_inertia = CircleInertiaModel::kHoop;
  CHECK(Near(GetAtwoodAnalyticalAcceleration(hoop), 1.962 / 2.7, 0.0001));

  // Balanced masses do not accelerate.
  CHECK(Near(GetAtwoodAnalyticalAcceleration(MakeAtwoodBalancedDriftConfig()),
             0.0, 0.0001));

  const AtwoodState state = MakeInitialAtwoodState(reference);
  const AtwoodDerived derived = CalculateAtwoodDerived(reference, state);
  CHECK(Near(derived.analytical_tension_a_n,
             1.0 * (9.81 + expected_acceleration), 0.0001));
  CHECK(Near(derived.analytical_tension_b_n,
             1.2 * (9.81 - expected_acceleration), 0.0001));
  CHECK(Near(derived.moment_of_inertia_kg_m2, 0.0025, 0.0001));
  CHECK(Near(derived.segment_length_a_m, 3.0));
  CHECK(Near(derived.segment_length_b_m, 3.0));
  CHECK(Near(derived.rope_length_error_m, 0.0));
  CHECK(derived.mechanical_energy_j == 0.0);
}

void TestReferenceAccelerationTensionsAndPinForce() {
  const AtwoodConfig config = MakeAtwoodReferenceConfig();
  AtwoodState state = MakeInitialAtwoodState(config);
  constexpr int kSteps = 720;  // 1.5 s
  for (int step = 0; step < kSteps; ++step) {
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &state));
  }
  const double elapsed = static_cast<double>(kSteps) * kAtwoodPhysicsStep;
  const double expected_acceleration = 1.962 / 2.45;
  const double measured_acceleration = state.block_b.velocity.y / elapsed;
  CHECK(std::abs(measured_acceleration - expected_acceleration) /
            expected_acceleration <=
        0.01);

  const AtwoodDerived derived = CalculateAtwoodDerived(config, state);
  CHECK(std::abs(state.tension_a_n - derived.analytical_tension_a_n) /
            derived.analytical_tension_a_n <=
        0.01);
  CHECK(std::abs(state.tension_b_n - derived.analytical_tension_b_n) /
            derived.analytical_tension_b_n <=
        0.01);
  // The pin carries the pulley weight; the wrap load rides on the anchors.
  CHECK(Near(state.pin_force_n.x, 0.0, 0.001));
  CHECK(Near(state.pin_force_n.y, -0.5 * 9.81, 0.001));
  CHECK(Near(derived.axle_load_n,
             0.5 * 9.81 + state.tension_a_n + state.tension_b_n, 0.0001));
}

void TestOppositeSpeedsAndNoSlipCoupling() {
  const AtwoodConfig config = MakeAtwoodReferenceConfig();
  AtwoodState state = MakeInitialAtwoodState(config);
  for (int step = 0; step < 960; ++step) {
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &state));
    if (step % 60 != 0) {
      continue;
    }
    const AtwoodDerived derived = CalculateAtwoodDerived(config, state);
    // Block b descends while block a rises at the same rope speed.
    CHECK(derived.rope_speed_b_mps >= 0.0f);
    CHECK(derived.rope_speed_a_mps <= 0.0f);
    CHECK(std::abs(derived.rope_speed_a_mps + derived.rope_speed_b_mps) <=
          0.0001f);
    CHECK(derived.no_slip_error_a_mps <= 0.0001f);
    CHECK(derived.no_slip_error_b_mps <= 0.0001f);
  }
}

void TestBalancedDriftRopeLengthAndFiniteSixtySeconds() {
  const AtwoodConfig config = MakeAtwoodBalancedDriftConfig();
  AtwoodState state = MakeInitialAtwoodState(config);
  constexpr int kSteps = 60 * 480;
  for (int step = 0; step < kSteps; ++step) {
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &state));
    if (step % 480 == 0) {
      const AtwoodDerived derived = CalculateAtwoodDerived(config, state);
      CHECK(std::abs(derived.rope_length_error_m) < 0.0001);
      CHECK(GetAtwoodTerminalIssue(config, state) == nullptr);
    }
  }
  CHECK(Near(state.time_seconds, 60.0, 0.0001));
  const AtwoodDerived derived = CalculateAtwoodDerived(config, state);
  CHECK(std::abs(derived.rope_length_error_m) < 0.0001);
  // Balanced masses keep drifting at the initial rope speed.
  CHECK(Near(derived.rope_speed_b_mps, 0.02, 0.01));
}

void TestConservativeEnergyBudget() {
  // Full-height reference window: side a is deep enough for the rising
  // block to travel essentially the whole area height, so the run ends at
  // the model's own terminal condition after ~13 s with ~137 J of released
  // potential energy, exercising long-horizon drift of the constraint
  // solve.
  AtwoodConfig config = MakeAtwoodReferenceConfig();
  config.hang_depth_a_m = 70.0f;
  config.hang_depth_b_m = 10.0f;
  AtwoodState state = MakeInitialAtwoodState(config);
  double peak_released = 0.0;
  bool terminated = false;
  for (int step = 0; step < 15 * 480 && !terminated; ++step) {
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &state));
    const AtwoodDerived derived = CalculateAtwoodDerived(config, state);
    peak_released =
        std::max(peak_released, std::max(0.0, -derived.potential_energy_j));
    const double scale = EnergyScale(0.0, peak_released);
    CHECK(std::abs(derived.mechanical_energy_j) <= 0.005 * scale);
    CHECK(state.dissipated_energy_j == 0.0);
    terminated = GetAtwoodTerminalIssue(config, state) != nullptr;
  }
  CHECK(terminated);
  CHECK(peak_released > 130.0);
}

void TestDampedEnergyAccounting() {
  const AtwoodConfig config = MakeAtwoodDampedConfig();
  AtwoodState state = MakeInitialAtwoodState(config);
  const double initial_energy =
      CalculateAtwoodDerived(config, state).mechanical_energy_j;
  CHECK(initial_energy > 1.0);  // ~1.225 J from the initial rope speed.
  double peak_released = 0.0;
  for (int step = 0; step < 1200; ++step) {  // 2.5 s
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &state));
    const AtwoodDerived derived = CalculateAtwoodDerived(config, state);
    peak_released =
        std::max(peak_released, std::max(0.0, -derived.potential_energy_j));
    const double scale = EnergyScale(initial_energy, peak_released);
    CHECK(derived.mechanical_energy_j <= initial_energy + 0.001 * scale);
    CHECK(std::abs(derived.accounted_energy_j - initial_energy) <=
          0.005 * scale);
    CHECK(GetAtwoodTerminalIssue(config, state) == nullptr);
  }
  CHECK(state.dissipated_energy_j > 0.0);
}

void TestDeterministicCheckpoints() {
  const AtwoodConfig config = MakeAtwoodReferenceConfig();
  AtwoodState first = MakeInitialAtwoodState(config);
  AtwoodState second = MakeInitialAtwoodState(config);
  for (int step = 0; step < 1200; ++step) {
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &first));
    CHECK(StepAtwood(config, kAtwoodPhysicsStep, &second));
    if (step % 100 == 0) {
      CHECK(SameState(first, second));
    }
  }
  CHECK(SameState(first, second));
}

void TestValidationAndFailureAtomicity() {
  const auto expect_config_error = [](auto mutate) {
    AtwoodConfig config = MakeAtwoodReferenceConfig();
    mutate(config);
    CHECK(GetAtwoodConfigError(config) != nullptr);
    bool threw = false;
    try {
      MakeInitialAtwoodState(config);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };

  expect_config_error([](AtwoodConfig& c) {
    c.mass_a_kg = std::numeric_limits<float>::quiet_NaN();
  });
  expect_config_error([](AtwoodConfig& c) { c.mass_b_kg = 0.001f; });
  expect_config_error([](AtwoodConfig& c) { c.pulley_mass_kg = 2000.0f; });
  expect_config_error([](AtwoodConfig& c) { c.pulley_radius_m = 0.01f; });
  expect_config_error([](AtwoodConfig& c) { c.pulley_radius_m = 6.0f; });
  expect_config_error([](AtwoodConfig& c) { c.block_edge_m = 0.01f; });
  expect_config_error([](AtwoodConfig& c) {
    // Edge at the pulley diameter would let the blocks touch.
    c.block_edge_m = 0.2f;
  });
  expect_config_error([](AtwoodConfig& c) { c.hang_depth_a_m = 0.4f; });
  expect_config_error([](AtwoodConfig& c) { c.hang_depth_b_m = 71.0f; });
  expect_config_error([](AtwoodConfig& c) {
    // Deep enough to violate the pulley clearance rule.
    c.pulley_radius_m = 2.0f;
    c.block_edge_m = 3.9f;
    c.hang_depth_a_m = 4.0f;
  });
  expect_config_error(
      [](AtwoodConfig& c) { c.initial_rope_speed_mps = 21.0f; });
  expect_config_error([](AtwoodConfig& c) { c.gravity_m_s2 = 0.0f; });
  expect_config_error([](AtwoodConfig& c) { c.gravity_m_s2 = 101.0f; });
  expect_config_error([](AtwoodConfig& c) { c.linear_damping_per_s = -1.0f; });
  expect_config_error([](AtwoodConfig& c) {
    // Deliberately out-of-enumerator input for the rejection path. The cast
    // is well-defined (scoped enum, int underlying type); the analyzer
    // flags any such cast, so the intentional invalid value is annotated.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    c.pulley_inertia = static_cast<CircleInertiaModel>(7);
  });

  // Invalid step inputs leave the state untouched.
  const AtwoodConfig config = MakeAtwoodReferenceConfig();
  AtwoodState state = MakeInitialAtwoodState(config);
  const AtwoodState before = state;
  CHECK(!StepAtwood(config, 0.0f, &state));
  CHECK(!StepAtwood(config, -kAtwoodPhysicsStep, &state));
  CHECK(!StepAtwood(config, kAtwoodPhysicsStep * 2.0f, &state));
  CHECK(!StepAtwood(config, std::numeric_limits<float>::quiet_NaN(), &state));
  CHECK(!StepAtwood(config, kAtwoodPhysicsStep, nullptr));
  CHECK(SameState(state, before));

  // A tampered state is rejected by the derived calculation and the step.
  AtwoodState tampered = state;
  tampered.block_a.mass = 2.0f * config.mass_a_kg;
  CHECK(GetAtwoodStateError(config, tampered) != nullptr);
  bool threw = false;
  try {
    CalculateAtwoodDerived(config, tampered);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  CHECK(!StepAtwood(config, kAtwoodPhysicsStep, &tampered));

  AtwoodState infected = state;
  infected.dissipated_energy_j = -1.0;
  CHECK(GetAtwoodStateError(config, infected) != nullptr);
}

void TestTerminalDetection() {
  const AtwoodConfig config = MakeAtwoodReferenceConfig();
  AtwoodState state = MakeInitialAtwoodState(config);
  CHECK(GetAtwoodTerminalIssue(config, state) == nullptr);

  AtwoodState near_pulley = state;
  near_pulley.block_a.position.y = 20.0f + 0.1f + 0.06f + 0.04f;
  CHECK(GetAtwoodTerminalIssue(config, near_pulley) != nullptr);

  AtwoodState near_floor = state;
  near_floor.block_b.position.y = 99.99f - 0.06f;
  CHECK(GetAtwoodTerminalIssue(config, near_floor) != nullptr);
}

void TestHistoryLookup() {
  const AtwoodConfig config = MakeAtwoodReferenceConfig();
  std::vector<AtwoodState> history;
  for (int i = 0; i < 3; ++i) {
    AtwoodState state = MakeInitialAtwoodState(config);
    state.time_seconds = static_cast<double>(i);
    history.push_back(state);
  }
  CHECK(FindAtwoodState(history, -1.0) == &history.front());
  CHECK(FindAtwoodState(history, 0.4) == &history.front());
  CHECK(FindAtwoodState(history, 0.6) == &history[1]);
  CHECK(FindAtwoodState(history, 9.0) == &history.back());
  CHECK(FindAtwoodState(history, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);
  const std::vector<AtwoodState> empty;
  CHECK(FindAtwoodState(empty, 1.0) == nullptr);
}

struct NamedTest {
  const char* name;
  void (*function)();
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"presets and analytical values",
                TestPresetsAndAnalyticalValues},
      NamedTest{"reference acceleration, tensions, and pin force",
                TestReferenceAccelerationTensionsAndPinForce},
      NamedTest{"opposite speeds and no-slip coupling",
                TestOppositeSpeedsAndNoSlipCoupling},
      NamedTest{"balanced drift rope length over sixty seconds",
                TestBalancedDriftRopeLengthAndFiniteSixtySeconds},
      NamedTest{"conservative energy budget", TestConservativeEnergyBudget},
      NamedTest{"damped energy accounting", TestDampedEnergyAccounting},
      NamedTest{"deterministic checkpoints", TestDeterministicCheckpoints},
      NamedTest{"validation and failure atomicity",
                TestValidationAndFailureAtomicity},
      NamedTest{"terminal detection", TestTerminalDetection},
      NamedTest{"history lookup", TestHistoryLookup},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << tiny2d::test::CheckCount()
            << " checks passed\n";
  return 0;
}
