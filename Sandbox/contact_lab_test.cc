#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "contact_lab_model.h"
#include "test_support.h"

namespace {

using tiny2d::Circle;
using tiny2d::CircleInertiaModel;
using tiny2d::CollisionMaterial;
using tiny2d::Rectangle;
using tiny2d::sandbox::CalculateContactLabDerived;
using tiny2d::sandbox::ContactLabConfig;
using tiny2d::sandbox::ContactLabDerived;
using tiny2d::sandbox::ContactLabMode;
using tiny2d::sandbox::ContactLabState;
using tiny2d::sandbox::FindContactLabState;
using tiny2d::sandbox::GetContactLabConfigError;
using tiny2d::sandbox::GetContactLabStateError;
using tiny2d::sandbox::kContactLabAreaHeightM;
using tiny2d::sandbox::kContactLabAreaWidthM;
using tiny2d::sandbox::kContactLabMaximumFrictionCoefficient;
using tiny2d::sandbox::kContactLabMaximumGravityMps2;
using tiny2d::sandbox::kContactLabMaximumInitialAngularSpeedRadS;
using tiny2d::sandbox::kContactLabMaximumInitialSpeedMps;
using tiny2d::sandbox::kContactLabMaximumMassKg;
using tiny2d::sandbox::kContactLabMaximumRadiusM;
using tiny2d::sandbox::kContactLabMinimumMassKg;
using tiny2d::sandbox::kContactLabMinimumRadiusM;
using tiny2d::sandbox::kContactLabPhysicsStep;
using tiny2d::sandbox::kContactLabRollingSlipToleranceMps;
using tiny2d::sandbox::MakeElasticImpactConfig;
using tiny2d::sandbox::MakeInitialContactLabState;
using tiny2d::sandbox::MakeRollingHoopConfig;
using tiny2d::sandbox::MakeRollingSolidDiskConfig;
using tiny2d::sandbox::StepContactLab;

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
         a.angular_damping_rate == b.angular_damping_rate &&
         SameMaterial(a.material, b.material);
}

bool SameState(const ContactLabState& a, const ContactLabState& b) {
  if (a.time_seconds != b.time_seconds ||
      a.rectangles.size() != b.rectangles.size() ||
      a.circles.size() != b.circles.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.rectangles.size(); ++i) {
    if (!SameRectangle(a.rectangles[i], b.rectangles[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.circles.size(); ++i) {
    if (!SameCircle(a.circles[i], b.circles[i])) {
      return false;
    }
  }
  return true;
}

double Magnitude(tiny2d::Vec2 value) {
  return std::hypot(static_cast<double>(value.x), static_cast<double>(value.y));
}

void Advance(const ContactLabConfig& config, double duration_seconds,
             ContactLabState* state) {
  const int step_count =
      static_cast<int>(std::round(duration_seconds / kContactLabPhysicsStep));
  for (int step = 0; step < step_count; ++step) {
    CHECK(StepContactLab(config, kContactLabPhysicsStep, state));
  }
}

void CheckAcceptedAndSteps(const ContactLabConfig& config) {
  CHECK(GetContactLabConfigError(config) == nullptr);
  ContactLabState state = MakeInitialContactLabState(config);
  CHECK(GetContactLabStateError(config, state) == nullptr);
  CHECK(StepContactLab(config, kContactLabPhysicsStep, &state));
  CHECK(state.time_seconds > 0.0);
  CHECK(GetContactLabStateError(config, state) == nullptr);
  const ContactLabDerived derived = CalculateContactLabDerived(config, state);
  CHECK(std::isfinite(derived.total_linear_momentum_kg_m_s.x));
  CHECK(std::isfinite(derived.total_linear_momentum_kg_m_s.y));
  CHECK(std::isfinite(derived.total_angular_momentum_kg_m2_s));
  CHECK(std::isfinite(derived.total_kinetic_energy_j));
}

void TestPresetsAndInertia() {
  const ContactLabConfig impact = MakeElasticImpactConfig();
  CHECK(GetContactLabConfigError(impact) == nullptr);
  const ContactLabState impact_state = MakeInitialContactLabState(impact);
  CHECK(impact_state.rectangles.empty());
  CHECK(impact_state.circles.size() == 2);
  const ContactLabDerived impact_derived =
      CalculateContactLabDerived(impact, impact_state);
  CHECK(Near(impact_derived.total_linear_momentum_kg_m_s.x, 3.0));
  CHECK(Near(impact_derived.total_linear_momentum_kg_m_s.y, 0.0));
  CHECK(Near(impact_derived.translational_kinetic_energy_j, 4.5));
  CHECK(Near(impact_derived.rotational_kinetic_energy_j, 0.0));
  CHECK(Near(impact_derived.total_angular_momentum_kg_m2_s, -21.0));

  const ContactLabConfig solid = MakeRollingSolidDiskConfig();
  const ContactLabConfig hoop = MakeRollingHoopConfig();
  CHECK(GetContactLabConfigError(solid) == nullptr);
  CHECK(GetContactLabConfigError(hoop) == nullptr);
  const ContactLabState solid_state = MakeInitialContactLabState(solid);
  const ContactLabState hoop_state = MakeInitialContactLabState(hoop);
  CHECK(solid_state.rectangles.size() == 1);
  CHECK(solid_state.circles.size() == 1);
  CHECK(Near(tiny2d::GetMomentOfInertia(solid_state.circles[0]), 0.125));
  CHECK(Near(tiny2d::GetMomentOfInertia(hoop_state.circles[0]), 0.25));
  CHECK(Near(CalculateContactLabDerived(solid, solid_state).rolling_slip_m_s,
             2.0));
}

void TestElasticImpactConservesMomentumAndEnergy() {
  const ContactLabConfig config = MakeElasticImpactConfig();
  ContactLabState state = MakeInitialContactLabState(config);
  const ContactLabDerived initial = CalculateContactLabDerived(config, state);
  Advance(config, 2.0, &state);
  const ContactLabDerived final = CalculateContactLabDerived(config, state);

  const tiny2d::Vec2 momentum_error{final.total_linear_momentum_kg_m_s.x -
                                        initial.total_linear_momentum_kg_m_s.x,
                                    final.total_linear_momentum_kg_m_s.y -
                                        initial.total_linear_momentum_kg_m_s.y};
  CHECK(Magnitude(momentum_error) /
            Magnitude(initial.total_linear_momentum_kg_m_s) <
        0.001);
  CHECK(
      std::abs(final.total_kinetic_energy_j - initial.total_kinetic_energy_j) /
          initial.total_kinetic_energy_j <
      0.01);
  CHECK(std::abs(final.total_angular_momentum_kg_m2_s -
                 initial.total_angular_momentum_kg_m2_s) /
            std::abs(initial.total_angular_momentum_kg_m2_s) <
        0.001);
  CHECK(std::abs(state.circles[0].velocity.x) < 0.001f);
  CHECK(Near(state.circles[1].velocity.x, 3.0, 0.001));
  CHECK(std::abs(state.circles[0].angular_velocity) < 0.001f);
  CHECK(std::abs(state.circles[1].angular_velocity) < 0.001f);
}

void TestBodyOrderSymmetryAndStaticCircle() {
  const ContactLabConfig config = MakeElasticImpactConfig();
  ContactLabState forward = MakeInitialContactLabState(config);
  ContactLabState reversed = forward;
  std::swap(reversed.circles[0], reversed.circles[1]);
  Advance(config, 2.0, &forward);
  Advance(config, 2.0, &reversed);
  CHECK(Near(forward.circles[0].position.x, reversed.circles[1].position.x,
             0.00001));
  CHECK(Near(forward.circles[0].velocity.x, reversed.circles[1].velocity.x,
             0.00001));
  CHECK(Near(forward.circles[1].position.x, reversed.circles[0].position.x,
             0.00001));
  CHECK(Near(forward.circles[1].velocity.x, reversed.circles[0].velocity.x,
             0.00001));

  ContactLabConfig static_target = config;
  static_target.circle_b.is_static = true;
  ContactLabState state = MakeInitialContactLabState(static_target);
  const Circle target_before = state.circles[1];
  Advance(static_target, 2.0, &state);
  CHECK(SameCircle(state.circles[1], target_before));
  CHECK(state.circles[0].velocity.x < 0.0f);
}

void TestSolidDiskAndHoopReachRollingContact() {
  ContactLabConfig solid_config = MakeRollingSolidDiskConfig();
  ContactLabConfig hoop_config = MakeRollingHoopConfig();
  ContactLabState solid = MakeInitialContactLabState(solid_config);
  ContactLabState hoop = MakeInitialContactLabState(hoop_config);
  const double initial_energy =
      CalculateContactLabDerived(solid_config, solid).total_kinetic_energy_j;

  Advance(solid_config, 1.0, &solid);
  Advance(hoop_config, 1.0, &hoop);
  const ContactLabDerived solid_derived =
      CalculateContactLabDerived(solid_config, solid);
  const ContactLabDerived hoop_derived =
      CalculateContactLabDerived(hoop_config, hoop);

  CHECK(std::abs(solid_derived.rolling_slip_m_s) <
        kContactLabRollingSlipToleranceMps);
  CHECK(std::abs(hoop_derived.rolling_slip_m_s) <
        kContactLabRollingSlipToleranceMps);
  CHECK(solid.circles[0].angular_velocity > 0.0f);
  CHECK(hoop.circles[0].angular_velocity > 0.0f);
  CHECK(solid.circles[0].velocity.x > hoop.circles[0].velocity.x);
  CHECK(Near(solid.circles[0].velocity.x, 4.0 / 3.0, 0.03));
  CHECK(Near(hoop.circles[0].velocity.x, 1.0, 0.03));
  CHECK(solid_derived.total_kinetic_energy_j < initial_energy);
  CHECK(solid.circles[0].position.y + solid.circles[0].radius <= 11.011f);
  CHECK(hoop.circles[0].position.y + hoop.circles[0].radius <= 11.011f);
}

void TestZeroMixedFrictionDoesNotCreateSpin() {
  ContactLabConfig config = MakeRollingSolidDiskConfig();
  config.circle_a.material.static_friction = 0.0f;
  config.circle_a.material.kinetic_friction = 0.0f;
  ContactLabState state = MakeInitialContactLabState(config);
  Advance(config, 0.5, &state);
  const ContactLabDerived derived = CalculateContactLabDerived(config, state);
  CHECK(state.circles[0].angular_velocity == 0.0f);
  CHECK(Near(state.circles[0].velocity.x, 2.0, 0.00001));
  CHECK(Near(derived.rolling_slip_m_s, 2.0, 0.00001));
}

void TestValidationAndFailureAtomicity() {
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  for (float invalid : invalid_values) {
    ContactLabConfig config = MakeElasticImpactConfig();
    config.circle_a.mass_kg = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeElasticImpactConfig();
    config.circle_a.radius_m = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeElasticImpactConfig();
    config.circle_a.initial_position_m.x = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeElasticImpactConfig();
    config.circle_a.initial_velocity_m_s.y = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeElasticImpactConfig();
    config.circle_a.initial_angular_velocity_rad_s = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeElasticImpactConfig();
    config.circle_a.material.restitution = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeRollingSolidDiskConfig();
    config.gravity_m_s2 = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
    config = MakeRollingSolidDiskConfig();
    config.surface_material.static_friction = invalid;
    CHECK(GetContactLabConfigError(config) != nullptr);
  }

  ContactLabConfig config = MakeElasticImpactConfig();
  config.circle_a.mass_kg = 0.0f;
  CHECK(GetContactLabConfigError(config) != nullptr);
  config = MakeElasticImpactConfig();
  config.circle_a.radius_m = 0.09f;
  CHECK(GetContactLabConfigError(config) != nullptr);
  config = MakeElasticImpactConfig();
  config.circle_a.initial_position_m = config.circle_b.initial_position_m;
  CHECK(GetContactLabConfigError(config) != nullptr);
  config = MakeElasticImpactConfig();
  config.circle_b.is_static = true;
  config.circle_b.initial_velocity_m_s.x = 1.0f;
  CHECK(GetContactLabConfigError(config) != nullptr);
  config = MakeElasticImpactConfig();
  config.circle_a.material.kinetic_friction = 0.5f;
  config.circle_a.material.static_friction = 0.4f;
  CHECK(GetContactLabConfigError(config) != nullptr);
  config = MakeRollingSolidDiskConfig();
  config.circle_a.initial_position_m.y = 10.6f;
  CHECK(GetContactLabConfigError(config) != nullptr);

  config = MakeElasticImpactConfig();
  const int invalid_mode = 99;
  static_assert(sizeof(invalid_mode) == sizeof(config.mode));
  std::memcpy(&config.mode, &invalid_mode, sizeof(config.mode));
  CHECK(GetContactLabConfigError(config) != nullptr);
  config = MakeElasticImpactConfig();
  const int invalid_inertia = 99;
  static_assert(sizeof(invalid_inertia) ==
                sizeof(config.circle_a.inertia_model));
  std::memcpy(&config.circle_a.inertia_model, &invalid_inertia,
              sizeof(config.circle_a.inertia_model));
  CHECK(GetContactLabConfigError(config) != nullptr);

  config = MakeElasticImpactConfig();
  ContactLabState state = MakeInitialContactLabState(config);
  const ContactLabState original = state;
  CHECK(!StepContactLab(config, 0.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepContactLab(config, kContactLabPhysicsStep * 2.0f, &state));
  CHECK(SameState(state, original));
  CHECK(!StepContactLab(config, kContactLabPhysicsStep, nullptr));
  ContactLabConfig invalid_config = config;
  invalid_config.circle_a.mass_kg = 0.0f;
  CHECK(!StepContactLab(invalid_config, kContactLabPhysicsStep, &state));
  CHECK(SameState(state, original));

  state.time_seconds = std::numeric_limits<double>::quiet_NaN();
  CHECK(GetContactLabStateError(config, state) != nullptr);
  state = original;
  state.circles[0].applied_force.x = 1.0f;
  CHECK(GetContactLabStateError(config, state) != nullptr);
  state = original;
  state.circles.pop_back();
  CHECK(GetContactLabStateError(config, state) != nullptr);

  state = original;
  state.time_seconds = std::numeric_limits<double>::max();
  const ContactLabState maximum_time_state = state;
  CHECK(!StepContactLab(config, kContactLabPhysicsStep, &state));
  CHECK(SameState(state, maximum_time_state));
}

void TestLegalBoundaryConfigurationsStep() {
  for (float mass : {kContactLabMinimumMassKg, kContactLabMaximumMassKg}) {
    ContactLabConfig config = MakeElasticImpactConfig();
    config.circle_a.mass_kg = mass;
    config.circle_b.mass_kg = mass;
    CheckAcceptedAndSteps(config);
  }

  for (float radius : {kContactLabMinimumRadiusM, kContactLabMaximumRadiusM}) {
    ContactLabConfig config = MakeElasticImpactConfig();
    config.circle_a.radius_m = radius;
    config.circle_b.radius_m = radius;
    CheckAcceptedAndSteps(config);
  }

  for (float speed : {-kContactLabMaximumInitialSpeedMps,
                      kContactLabMaximumInitialSpeedMps}) {
    ContactLabConfig horizontal = MakeElasticImpactConfig();
    horizontal.circle_a.initial_velocity_m_s = {speed, 0.0f};
    horizontal.circle_b.initial_velocity_m_s = {speed, 0.0f};
    CheckAcceptedAndSteps(horizontal);

    ContactLabConfig vertical = MakeElasticImpactConfig();
    vertical.circle_a.initial_velocity_m_s = {0.0f, speed};
    vertical.circle_b.initial_velocity_m_s = {0.0f, speed};
    CheckAcceptedAndSteps(vertical);
  }

  for (float angular_speed : {-kContactLabMaximumInitialAngularSpeedRadS,
                              kContactLabMaximumInitialAngularSpeedRadS}) {
    ContactLabConfig config = MakeElasticImpactConfig();
    config.circle_a.initial_angular_velocity_rad_s = angular_speed;
    config.circle_b.initial_angular_velocity_rad_s = angular_speed;
    CheckAcceptedAndSteps(config);
  }

  ContactLabConfig config = MakeElasticImpactConfig();
  config.circle_a.initial_angle_degrees = -180.0f;
  config.circle_b.initial_angle_degrees = 180.0f;
  CheckAcceptedAndSteps(config);
  config.circle_a.initial_angle_degrees = 180.0f;
  config.circle_b.initial_angle_degrees = -180.0f;
  CheckAcceptedAndSteps(config);

  config = MakeElasticImpactConfig();
  config.circle_a.initial_velocity_m_s = {};
  config.circle_a.initial_position_m = {config.circle_a.radius_m,
                                        config.circle_a.radius_m};
  CheckAcceptedAndSteps(config);
  config.circle_a.initial_position_m = {
      kContactLabAreaWidthM - config.circle_a.radius_m,
      kContactLabAreaHeightM - config.circle_a.radius_m};
  CheckAcceptedAndSteps(config);

  for (const CollisionMaterial material :
       {CollisionMaterial{0.0f, 0.0f, 0.0f},
        CollisionMaterial{1.0f, kContactLabMaximumFrictionCoefficient,
                          kContactLabMaximumFrictionCoefficient}}) {
    config = MakeElasticImpactConfig();
    config.circle_a.material = material;
    config.circle_b.material = material;
    CheckAcceptedAndSteps(config);
  }

  for (float gravity : {0.0f, kContactLabMaximumGravityMps2}) {
    config = MakeRollingSolidDiskConfig();
    config.gravity_m_s2 = gravity;
    CheckAcceptedAndSteps(config);
  }

  for (const CollisionMaterial material :
       {CollisionMaterial{0.0f, 0.0f, 0.0f},
        CollisionMaterial{1.0f, kContactLabMaximumFrictionCoefficient,
                          kContactLabMaximumFrictionCoefficient}}) {
    config = MakeRollingSolidDiskConfig();
    config.circle_a.material = material;
    config.surface_material = material;
    CheckAcceptedAndSteps(config);
  }

  config = MakeRollingHoopConfig();
  config.circle_a.radius_m = kContactLabMaximumRadiusM;
  config.circle_a.initial_position_m.y = 11.0f - config.circle_a.radius_m;
  CheckAcceptedAndSteps(config);
}

void TestHistoryLookup() {
  const ContactLabConfig config = MakeElasticImpactConfig();
  std::vector<ContactLabState> history(3, MakeInitialContactLabState(config));
  history[0].time_seconds = 0.0;
  history[1].time_seconds = 1.0;
  history[2].time_seconds = 2.0;
  CHECK(FindContactLabState({}, 0.0) == nullptr);
  CHECK(FindContactLabState(
            history, std::numeric_limits<double>::quiet_NaN()) == nullptr);
  CHECK(FindContactLabState(history, -1.0) == &history[0]);
  CHECK(FindContactLabState(history, 0.5) == &history[0]);
  CHECK(FindContactLabState(history, 0.6) == &history[1]);
  CHECK(FindContactLabState(history, 3.0) == &history[2]);
}

void TestSixtySecondRunIsFiniteAndDeterministic() {
  const ContactLabConfig impact_config = MakeElasticImpactConfig();
  ContactLabState impact_first = MakeInitialContactLabState(impact_config);
  ContactLabState impact_second = impact_first;
  const ContactLabConfig rolling_config = MakeRollingSolidDiskConfig();
  ContactLabState rolling_first = MakeInitialContactLabState(rolling_config);
  ContactLabState rolling_second = rolling_first;
  constexpr int kStepCount = 60 * 480;
  for (int step = 0; step < kStepCount; ++step) {
    CHECK(StepContactLab(impact_config, kContactLabPhysicsStep, &impact_first));
    CHECK(
        StepContactLab(impact_config, kContactLabPhysicsStep, &impact_second));
    CHECK(
        StepContactLab(rolling_config, kContactLabPhysicsStep, &rolling_first));
    CHECK(StepContactLab(rolling_config, kContactLabPhysicsStep,
                         &rolling_second));
    if (step % 480 == 0) {
      CHECK(GetContactLabStateError(impact_config, impact_first) == nullptr);
      CHECK(GetContactLabStateError(rolling_config, rolling_first) == nullptr);
      const ContactLabDerived impact_derived =
          CalculateContactLabDerived(impact_config, impact_first);
      const ContactLabDerived rolling_derived =
          CalculateContactLabDerived(rolling_config, rolling_first);
      CHECK(std::isfinite(impact_derived.total_kinetic_energy_j));
      CHECK(std::isfinite(impact_derived.total_angular_momentum_kg_m2_s));
      CHECK(std::isfinite(rolling_derived.total_kinetic_energy_j));
      CHECK(std::isfinite(rolling_derived.total_angular_momentum_kg_m2_s));
    }
  }
  CHECK(SameState(impact_first, impact_second));
  CHECK(SameState(rolling_first, rolling_second));
  CHECK(impact_first.time_seconds > 59.99);
  CHECK(rolling_first.time_seconds > 59.99);
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
      NamedTest{"presets and inertia", TestPresetsAndInertia},
      NamedTest{"elastic impact conserves momentum and energy",
                TestElasticImpactConservesMomentumAndEnergy},
      NamedTest{"body order symmetry and static circle",
                TestBodyOrderSymmetryAndStaticCircle},
      NamedTest{"solid disk and hoop reach rolling contact",
                TestSolidDiskAndHoopReachRollingContact},
      NamedTest{"zero mixed friction does not create spin",
                TestZeroMixedFrictionDoesNotCreateSpin},
      NamedTest{"validation and failure atomicity",
                TestValidationAndFailureAtomicity},
      NamedTest{"legal boundary configurations step",
                TestLegalBoundaryConfigurationsStep},
      NamedTest{"history lookup", TestHistoryLookup},
      NamedTest{"60 second finite deterministic run",
                TestSixtySecondRunIsFiniteAndDeterministic},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << tiny2d::test::CheckCount()
            << " checks passed\n";
  return 0;
}
