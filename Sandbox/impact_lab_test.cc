#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

#include "impact_lab_model.h"
#include "test_support.h"

namespace {

using tiny2d::Circle;
using tiny2d::CollisionMaterial;
using tiny2d::sandbox::CalculateImpactLabDerived;
using tiny2d::sandbox::FindImpactLabState;
using tiny2d::sandbox::GetImpactLabConfigError;
using tiny2d::sandbox::GetImpactLabStateError;
using tiny2d::sandbox::ImpactLabConfig;
using tiny2d::sandbox::ImpactLabDerived;
using tiny2d::sandbox::ImpactLabPreset;
using tiny2d::sandbox::ImpactLabState;
using tiny2d::sandbox::kImpactLabMaximumDiameterSkipSpeedMps;
using tiny2d::sandbox::kImpactLabMaximumGrazingSpeedMps;
using tiny2d::sandbox::kImpactLabMaximumMassKg;
using tiny2d::sandbox::kImpactLabMinimumDiameterSkipSpeedMps;
using tiny2d::sandbox::kImpactLabMinimumGrazingSpeedMps;
using tiny2d::sandbox::kImpactLabMinimumMassKg;
using tiny2d::sandbox::kImpactLabPhysicsStep;
using tiny2d::sandbox::MakeDiameterSkipImpactConfig;
using tiny2d::sandbox::MakeInitialImpactLabState;
using tiny2d::sandbox::MakeSupportedGrazingImpactConfig;
using tiny2d::sandbox::StepImpactLab;

bool Near(double actual, double expected, double tolerance = 0.00001) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

bool SameMaterial(const CollisionMaterial& a, const CollisionMaterial& b) {
  return a.restitution == b.restitution &&
         a.static_friction == b.static_friction &&
         a.kinetic_friction == b.kinetic_friction;
}

bool SameCircle(const Circle& a, const Circle& b) {
  return a.mass == b.mass && a.position.x == b.position.x &&
         a.position.y == b.position.y && a.velocity.x == b.velocity.x &&
         a.velocity.y == b.velocity.y && a.angle == b.angle &&
         a.angular_velocity == b.angular_velocity && a.radius == b.radius &&
         a.inertia_model == b.inertia_model &&
         a.fixed_rotation == b.fixed_rotation && a.charge == b.charge &&
         a.applied_force.x == b.applied_force.x &&
         a.applied_force.y == b.applied_force.y &&
         a.applied_torque == b.applied_torque &&
         a.linear_damping_rate == b.linear_damping_rate &&
         a.angular_damping_rate == b.angular_damping_rate &&
         SameMaterial(a.material, b.material);
}

bool SameLane(const std::vector<Circle>& a, const std::vector<Circle>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!SameCircle(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

bool SameState(const ImpactLabState& a, const ImpactLabState& b) {
  return a.time_seconds == b.time_seconds &&
         SameLane(a.discrete_circles, b.discrete_circles) &&
         SameLane(a.ccd_circles, b.ccd_circles);
}

void CheckAcceptedAndSteps(const ImpactLabConfig& config) {
  CHECK(GetImpactLabConfigError(config) == nullptr);
  ImpactLabState state = MakeInitialImpactLabState(config);
  CHECK(GetImpactLabStateError(config, state) == nullptr);
  CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &state));
  CHECK(state.time_seconds > 0.0);
  CHECK(GetImpactLabStateError(config, state) == nullptr);
  const ImpactLabDerived derived = CalculateImpactLabDerived(config, state);
  CHECK(std::isfinite(derived.entry_time_seconds));
  CHECK(std::isfinite(derived.ccd_position_error_m));
  CHECK(std::isfinite(derived.ccd_velocity_error_m_s));
}

void TestPresetsAndAnalyticalReference() {
  const ImpactLabConfig grazing = MakeSupportedGrazingImpactConfig();
  CHECK(grazing.preset == ImpactLabPreset::kSupportedGrazing);
  CHECK(grazing.speed_m_s == 10.0f);
  CHECK(GetImpactLabConfigError(grazing) == nullptr);
  const ImpactLabState grazing_state = MakeInitialImpactLabState(grazing);
  const ImpactLabDerived grazing_reference =
      CalculateImpactLabDerived(grazing, grazing_state);
  CHECK(grazing_reference.entry_time_seconds > 0.0);
  CHECK(grazing_reference.exit_time_seconds < kImpactLabPhysicsStep);
  CHECK(grazing_reference.entry_time_seconds <
        grazing_reference.exit_time_seconds);
  CHECK(Near(std::hypot(grazing_reference.contact_normal.x,
                        grazing_reference.contact_normal.y),
             1.0, 0.0001));

  const ImpactLabConfig diameter_skip = MakeDiameterSkipImpactConfig();
  CHECK(diameter_skip.preset == ImpactLabPreset::kDiameterSkip);
  CHECK(diameter_skip.speed_m_s == 240.0f);
  CHECK(GetImpactLabConfigError(diameter_skip) == nullptr);
  const ImpactLabDerived high_speed_reference = CalculateImpactLabDerived(
      diameter_skip, MakeInitialImpactLabState(diameter_skip));
  CHECK(Near(high_speed_reference.entry_time_seconds, 1.0 / 24000.0, 0.000001));
  CHECK(Near(high_speed_reference.exit_time_seconds, 0.41 / 240.0, 0.000001));
  CHECK(Near(high_speed_reference.expected_velocity_a_m_s.x, 0.0, 0.0001));
  CHECK(Near(high_speed_reference.expected_velocity_b_m_s.x, 240.0, 0.0001));
  CHECK(Near(high_speed_reference.expected_position_a_m.x, 8.01, 0.0001));
  CHECK(Near(high_speed_reference.expected_position_b_m.x, 8.70, 0.0001));
  CHECK(Near(high_speed_reference.moving_distance_per_step_m, 0.5, 0.0001));
  CHECK(Near(high_speed_reference.moving_distance_to_diameter_ratio, 2.5,
             0.0001));
}

void TestSupportedGrazingTunnelingAndCcd() {
  const ImpactLabConfig config = MakeSupportedGrazingImpactConfig();
  ImpactLabState state = MakeInitialImpactLabState(config);
  CHECK(!tiny2d::IsColliding(state.discrete_circles[0],
                             state.discrete_circles[1]));
  CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &state));
  CHECK(!tiny2d::IsColliding(state.discrete_circles[0],
                             state.discrete_circles[1]));
  const ImpactLabDerived derived = CalculateImpactLabDerived(config, state);
  CHECK(derived.discrete_tunneled);
  CHECK(derived.ccd_resolved);
  CHECK(state.discrete_circles[0].velocity.x == 10.0f);
  CHECK(state.discrete_circles[1].velocity.x == -10.0f);
  CHECK(state.discrete_circles[0].position.x >
        state.discrete_circles[1].position.x);
  CHECK(derived.ccd_position_error_m < 0.001);
  CHECK(derived.ccd_velocity_error_m_s < 0.01);
  CHECK(derived.ccd_momentum_error_kg_m_s < 0.001);
}

void TestDiameterSkipAndConservation() {
  const ImpactLabConfig config = MakeDiameterSkipImpactConfig();
  ImpactLabState first = MakeInitialImpactLabState(config);
  ImpactLabState second = first;
  CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &first));
  CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &second));
  CHECK(SameState(first, second));

  const ImpactLabDerived derived = CalculateImpactLabDerived(config, first);
  CHECK(derived.moving_distance_per_step_m >
        2.0 * tiny2d::sandbox::kImpactLabCircleRadiusM);
  CHECK(derived.discrete_tunneled);
  CHECK(derived.ccd_resolved);
  CHECK(Near(first.discrete_circles[0].position.x, 8.5, 0.0001));
  CHECK(Near(first.discrete_circles[1].position.x, 8.21, 0.0001));
  CHECK(first.discrete_circles[0].velocity.x == 240.0f);
  CHECK(first.discrete_circles[1].velocity.x == 0.0f);
  CHECK(derived.ccd_momentum_error_kg_m_s /
            std::hypot(derived.initial_momentum_kg_m_s.x,
                       derived.initial_momentum_kg_m_s.y) <
        0.001);
  CHECK(derived.ccd_kinetic_energy_error_j / derived.expected_kinetic_energy_j <
        0.01);
}

void TestUnequalMassInelasticReference() {
  ImpactLabConfig config = MakeDiameterSkipImpactConfig();
  config.mass_a_kg = 2.0f;
  config.mass_b_kg = 1.0f;
  config.speed_m_s = 300.0f;
  config.restitution = 0.5f;
  ImpactLabState state = MakeInitialImpactLabState(config);
  CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &state));
  const ImpactLabDerived derived = CalculateImpactLabDerived(config, state);
  CHECK(Near(derived.expected_velocity_a_m_s.x, 150.0, 0.0001));
  CHECK(Near(derived.expected_velocity_b_m_s.x, 300.0, 0.0001));
  CHECK(Near(derived.expected_kinetic_energy_j, 67500.0, 0.0001));
  CHECK(derived.ccd_resolved);
  CHECK(derived.ccd_velocity_error_m_s < 0.01);
  CHECK(derived.ccd_momentum_error_kg_m_s < 0.01);
  CHECK(derived.ccd_kinetic_energy_error_j / derived.expected_kinetic_energy_j <
        0.01);
}

void TestValidationFailureAtomicityAndBoundaries() {
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  for (float invalid : invalid_values) {
    ImpactLabConfig config = MakeDiameterSkipImpactConfig();
    config.mass_a_kg = invalid;
    CHECK(GetImpactLabConfigError(config) != nullptr);
    config = MakeDiameterSkipImpactConfig();
    config.mass_b_kg = invalid;
    CHECK(GetImpactLabConfigError(config) != nullptr);
    config = MakeDiameterSkipImpactConfig();
    config.speed_m_s = invalid;
    CHECK(GetImpactLabConfigError(config) != nullptr);
    config = MakeDiameterSkipImpactConfig();
    config.restitution = invalid;
    CHECK(GetImpactLabConfigError(config) != nullptr);
  }

  ImpactLabConfig config = MakeDiameterSkipImpactConfig();
  config.mass_a_kg = kImpactLabMinimumMassKg - 0.01f;
  CHECK(GetImpactLabConfigError(config) != nullptr);
  config = MakeDiameterSkipImpactConfig();
  config.mass_b_kg = kImpactLabMaximumMassKg + 0.01f;
  CHECK(GetImpactLabConfigError(config) != nullptr);
  config = MakeDiameterSkipImpactConfig();
  config.speed_m_s = kImpactLabMinimumDiameterSkipSpeedMps - 1.0f;
  CHECK(GetImpactLabConfigError(config) != nullptr);
  config = MakeSupportedGrazingImpactConfig();
  config.speed_m_s = kImpactLabMaximumGrazingSpeedMps + 0.01f;
  CHECK(GetImpactLabConfigError(config) != nullptr);
  config = MakeDiameterSkipImpactConfig();
  config.restitution = -0.01f;
  CHECK(GetImpactLabConfigError(config) != nullptr);
  config.restitution = 1.01f;
  CHECK(GetImpactLabConfigError(config) != nullptr);
  config = MakeDiameterSkipImpactConfig();
  const int invalid_preset = 99;
  static_assert(sizeof(invalid_preset) == sizeof(config.preset));
  std::memcpy(&config.preset, &invalid_preset, sizeof(config.preset));
  CHECK(GetImpactLabConfigError(config) != nullptr);

  config = MakeDiameterSkipImpactConfig();
  ImpactLabState state = MakeInitialImpactLabState(config);
  const ImpactLabState original = state;
  CHECK(!StepImpactLab(config, 0.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepImpactLab(config, kImpactLabPhysicsStep * 2.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepImpactLab(config, kImpactLabPhysicsStep, nullptr));
  ImpactLabConfig invalid_config = config;
  invalid_config.mass_a_kg = 0.0f;
  CHECK(!StepImpactLab(invalid_config, kImpactLabPhysicsStep, &state));
  CHECK(SameState(state, original));

  state.time_seconds = std::numeric_limits<double>::quiet_NaN();
  CHECK(GetImpactLabStateError(config, state) != nullptr);
  state = original;
  state.discrete_circles.pop_back();
  CHECK(GetImpactLabStateError(config, state) != nullptr);
  state = original;
  state.ccd_circles[0].applied_force.x = 1.0f;
  CHECK(GetImpactLabStateError(config, state) != nullptr);
  state = original;
  state.ccd_circles[1].mass = 0.0f;
  CHECK(GetImpactLabStateError(config, state) != nullptr);
  state = original;
  state.time_seconds = std::numeric_limits<double>::max();
  const ImpactLabState maximum_time_state = state;
  CHECK(!StepImpactLab(config, kImpactLabPhysicsStep, &state));
  CHECK(SameState(state, maximum_time_state));

  for (float mass : {kImpactLabMinimumMassKg, kImpactLabMaximumMassKg}) {
    config = MakeDiameterSkipImpactConfig();
    config.mass_a_kg = mass;
    config.mass_b_kg = mass;
    CheckAcceptedAndSteps(config);
  }
  for (float restitution : {0.0f, 1.0f}) {
    config = MakeDiameterSkipImpactConfig();
    config.restitution = restitution;
    CheckAcceptedAndSteps(config);
  }
  for (float speed : {kImpactLabMinimumDiameterSkipSpeedMps,
                      kImpactLabMaximumDiameterSkipSpeedMps}) {
    config = MakeDiameterSkipImpactConfig();
    config.speed_m_s = speed;
    CheckAcceptedAndSteps(config);
  }
  for (float speed :
       {kImpactLabMinimumGrazingSpeedMps, kImpactLabMaximumGrazingSpeedMps}) {
    config = MakeSupportedGrazingImpactConfig();
    config.speed_m_s = speed;
    CheckAcceptedAndSteps(config);
  }
}

void TestHistoryLookup() {
  const ImpactLabConfig config = MakeSupportedGrazingImpactConfig();
  std::vector<ImpactLabState> history(3, MakeInitialImpactLabState(config));
  history[0].time_seconds = 0.0;
  history[1].time_seconds = 1.0;
  history[2].time_seconds = 2.0;
  CHECK(FindImpactLabState({}, 0.0) == nullptr);
  CHECK(FindImpactLabState(history, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);
  CHECK(FindImpactLabState(history, -1.0) == &history[0]);
  CHECK(FindImpactLabState(history, 0.5) == &history[0]);
  CHECK(FindImpactLabState(history, 0.6) == &history[1]);
  CHECK(FindImpactLabState(history, 3.0) == &history[2]);
}

void TestSixtySecondRunsStayFiniteAndDeterministic() {
  const std::array configs = {MakeSupportedGrazingImpactConfig(),
                              MakeDiameterSkipImpactConfig()};
  constexpr int kStepCount = 60 * 480;
  for (const ImpactLabConfig& config : configs) {
    ImpactLabState first = MakeInitialImpactLabState(config);
    ImpactLabState second = first;
    for (int step = 0; step < kStepCount; ++step) {
      CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &first));
      CHECK(StepImpactLab(config, kImpactLabPhysicsStep, &second));
      if (step % 480 == 0) {
        CHECK(GetImpactLabStateError(config, first) == nullptr);
        const ImpactLabDerived derived =
            CalculateImpactLabDerived(config, first);
        CHECK(std::isfinite(derived.ccd_position_error_m));
        CHECK(std::isfinite(derived.ccd_velocity_error_m_s));
        CHECK(std::isfinite(derived.ccd_momentum_error_kg_m_s));
        CHECK(std::isfinite(derived.ccd_kinetic_energy_error_j));
      }
    }
    CHECK(SameState(first, second));
    CHECK(first.time_seconds > 59.99);
  }
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
      NamedTest{"presets and analytical reference",
                TestPresetsAndAnalyticalReference},
      NamedTest{"supported grazing tunnels discretely and CCD resolves",
                TestSupportedGrazingTunnelingAndCcd},
      NamedTest{"diameter skip resolves and conserves",
                TestDiameterSkipAndConservation},
      NamedTest{"unequal mass inelastic reference",
                TestUnequalMassInelasticReference},
      NamedTest{"validation, failure atomicity, and boundaries",
                TestValidationFailureAtomicityAndBoundaries},
      NamedTest{"history lookup", TestHistoryLookup},
      NamedTest{"60 second finite deterministic runs",
                TestSixtySecondRunsStayFiniteAndDeterministic},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << tiny2d::test::CheckCount()
            << " checks passed\n";
  return 0;
}
