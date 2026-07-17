#define SDL_MAIN_HANDLED
#include <cstdlib>
#include <iostream>
#include <limits>

#include "rotation_pendulum_simulation.cc"

namespace {

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
  CHECK(Near(state.time_seconds, kStep));
}

void TestHistoryLookup() {
  std::vector<PendulumState> history(3);
  history[0].time_seconds = 0.0f;
  history[1].time_seconds = 1.0f;
  history[2].time_seconds = 2.0f;

  CHECK(FindPendulumState({}, 0.0f) == nullptr);
  CHECK(FindPendulumState(history, std::numeric_limits<float>::quiet_NaN()) ==
        nullptr);
  CHECK(FindPendulumState(history, -1.0f) == &history[0]);
  CHECK(FindPendulumState(history, 0.5f) == &history[0]);
  CHECK(FindPendulumState(history, 0.6f) == &history[1]);
  CHECK(FindPendulumState(history, 3.0f) == &history[2]);
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
  float previous_time = state.time_seconds;
  std::array<float, 2> crossings{};
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
  const float measured_period = crossings[1] - crossings[0];
  CHECK(Near(measured_period, 2.1765592f, 0.002f));
  CHECK(Near(measured_period, GetSmallAnglePeriod(config), 0.002f));
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
  constexpr std::array<float PendulumConfig::*, 11> config_fields = {
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
  const PendulumState valid_state = MakeInitialPendulumState(config);
  constexpr std::array<float PendulumState::*, 4> state_fields = {
      &PendulumState::angle_radians,
      &PendulumState::angular_velocity_rad_s,
      &PendulumState::angular_acceleration_rad_s2,
      &PendulumState::time_seconds,
  };
  for (float invalid_value : invalid_values) {
    for (float PendulumState::* field : state_fields) {
      PendulumState state = valid_state;
      state.*field = invalid_value;
      CHECK(GetPendulumStateError(config, state) != nullptr);
    }
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
  CHECK(Near(first.time_seconds, second.time_seconds, 0.000001f));
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
  };

  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << check_count << " checks passed\n";
  return 0;
}
