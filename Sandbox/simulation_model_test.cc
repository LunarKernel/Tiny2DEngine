#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

#include "fixed_step_clock.h"
#include "incline_spring_model.h"
#include "simulation_history.h"

namespace tiny2d::sandbox::incline_spring {
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

constexpr float kTestTolerance = 0.001f;

bool Near(float actual, float expected, float tolerance = kTestTolerance) {
  return std::abs(actual - expected) <= tolerance;
}

bool NearTime(double actual, double expected, double tolerance = 0.000001) {
  return std::abs(actual - expected) <= tolerance;
}

struct HistorySample {
  double time;
  int id;
};

void TestFixedStepClock() {
  tiny2d::sandbox::FixedStepClock exact_clock(8.0, 0);
  exact_clock.Accumulate(1, 1.0);
  CHECK(exact_clock.HasStep(0.125));
  exact_clock.ConsumeStep(0.0625);
  CHECK(exact_clock.HasStep(0.0625));
  CHECK(!exact_clock.HasStep(0.125));
  exact_clock.ConsumeStep(0.0625);
  CHECK(!exact_clock.HasStep(0.0625));

  tiny2d::sandbox::FixedStepClock clamped_clock(8.0, 0);
  clamped_clock.Accumulate(100, 0.25);
  clamped_clock.ConsumeStep(0.125);
  clamped_clock.ConsumeStep(0.125);
  CHECK(!clamped_clock.HasStep(0.125));

  clamped_clock.Accumulate(101, 1.0);
  CHECK(clamped_clock.HasStep(0.125));
  clamped_clock.Reset(200);
  CHECK(!clamped_clock.HasStep(0.125));
  clamped_clock.Accumulate(201, 1.0);
  CHECK(clamped_clock.HasStep(0.125));
  clamped_clock.DiscardPendingSteps();
  CHECK(!clamped_clock.HasStep(0.125));

  tiny2d::sandbox::FixedStepClock backward_clock(10.0, 100);
  backward_clock.Accumulate(90, 1.0);
  CHECK(!backward_clock.HasStep(0.1));
  backward_clock.Accumulate(91, 1.0);
  CHECK(backward_clock.HasStep(0.1));
}

void ExpectInvalidConfig(const SimulationConfig& config) {
  CHECK(GetConfigError(config) != nullptr);
}

void CheckBodyFinite(const tiny2d::Rectangle& body) {
  const std::array values = {
      body.mass,       body.position.x, body.position.y,       body.velocity.x,
      body.velocity.y, body.angle,      body.angular_velocity, body.width,
      body.height,     body.charge,
  };
  for (float value : values) {
    CHECK(std::isfinite(value));
  }
  for (const tiny2d::Vec2 vertex : tiny2d::GetVertices(body)) {
    CHECK(std::isfinite(vertex.x));
    CHECK(std::isfinite(vertex.y));
  }
}

std::vector<tiny2d::Rectangle> CreateScene(const SimulationConfig& config) {
  State state;
  CHECK(Reset(config, state) == nullptr);
  return state.bodies;
}

void TestDefaultAndFeatureConfigurations() {
  SimulationConfig config;
  CHECK(GetConfigError(config) == nullptr);
  CHECK(Near(config.body_a.mass_kg, 1.0f));
  CHECK(Near(config.body_b.mass_kg, 3.0f));
  CHECK(Near(CreateScene(config)[0].mass, 1.0f));
  CHECK(Near(CreateScene(config)[1].mass, 3.0f));
  const float default_mass_scale = GetEngineMassPerKilogram(config);
  config.body_b.mass_kg = 5.0f;
  CHECK(Near(GetEngineMassPerKilogram(config), default_mass_scale));
  config.body_b.mass_kg = 3.0f;

  config.ramp_enabled = false;
  CHECK(GetConfigError(config) == nullptr);
  config.spring_enabled = false;
  CHECK(GetConfigError(config) == nullptr);
  config.electric_field_enabled = true;
  config.electric_field_strength_n_per_c = 1000000.0f;
  CHECK(GetConfigError(config) == nullptr);

  config = SimulationConfig{};
  config.gravity_mps2 = 10.0f;
  config.friction = 5.0f;
  config.restitution = 0.0f;
  CHECK(GetConfigError(config) == nullptr);
  config.restitution = 1.0f;
  CHECK(GetConfigError(config) == nullptr);
}

void TestStateLifecycleContracts() {
  SimulationConfig config;
  State state;
  CHECK(Reset(config, state) == nullptr);
  CHECK(state.bodies.size() == 3);
  CHECK(state.history.size() == 1);
  CHECK(state.time == 0.0);

  const float initial_position_x = state.bodies[0].position.x;
  SimulationConfig invalid_config = config;
  invalid_config.reference_speed_mps = 0.0f;
  CHECK(Reset(invalid_config, state) != nullptr);
  CHECK(state.bodies.size() == 3);
  CHECK(state.history.size() == 1);
  CHECK(state.bodies[0].position.x == initial_position_x);

  CHECK(Step(state, 0.0f) != nullptr);
  CHECK(state.history.size() == 1);
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(NearTime(state.time, kPhysicsStep));
  CHECK(state.history.size() == 2);
  CHECK(FindSnapshot(state, state.time) == &state.history.back());
}

void TestEngineValidationExceptionsPropagate() {
  SimulationConfig config;
  config.ramp_enabled = false;
  config.spring_enabled = false;
  State state;
  CHECK(Reset(config, state) == nullptr);

  state.config.electric_field_enabled = true;
  state.config.electric_field_strength_n_per_c =
      std::numeric_limits<float>::max();
  state.config.body_a_engine_mass = std::numeric_limits<float>::max();
  bool caught = false;
  try {
    static_cast<void>(Step(state, kPhysicsStep));
  } catch (const std::invalid_argument&) {
    caught = true;
  }
  CHECK(caught);
}

void TestEveryFloatFieldRejectsNonFiniteValues() {
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };
  constexpr std::array<float SimulationConfig::*, 11> config_fields = {
      &SimulationConfig::real_floor_length_m,
      &SimulationConfig::real_ramp_length_m,
      &SimulationConfig::reference_speed_mps,
      &SimulationConfig::body_a_engine_mass,
      &SimulationConfig::gravity_mps2,
      &SimulationConfig::friction,
      &SimulationConfig::restitution,
      &SimulationConfig::electric_field_strength_n_per_c,
      &SimulationConfig::electric_field_angle_degrees,
      &SimulationConfig::ramp_angle_degrees,
      &SimulationConfig::spring_stiffness,
  };
  constexpr std::array<float BodyConfig::*, 5> body_fields = {
      &BodyConfig::surface_x,      &BodyConfig::mass_kg,
      &BodyConfig::downhill_speed, &BodyConfig::length_units,
      &BodyConfig::charge,
  };

  for (float invalid_value : invalid_values) {
    for (float SimulationConfig::* field : config_fields) {
      SimulationConfig config;
      config.*field = invalid_value;
      ExpectInvalidConfig(config);
    }
    for (float BodyConfig::* field : body_fields) {
      SimulationConfig config;
      config.body_a.*field = invalid_value;
      ExpectInvalidConfig(config);
      config = SimulationConfig{};
      config.body_b.*field = invalid_value;
      ExpectInvalidConfig(config);
    }
  }
}

void TestInvalidRangesAndDegenerateInputs() {
  SimulationConfig config;
  config.real_floor_length_m = 0.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.real_ramp_length_m = -1.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.reference_speed_mps = 0.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_a_engine_mass = -1.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.gravity_mps2 = -9.8f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.gravity_mps2 = 9.81f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.friction = -0.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.friction = 5.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.restitution = -0.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.restitution = 1.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.ramp_angle_degrees = 4.99f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.ramp_angle_degrees = 45.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.electric_field_strength_n_per_c = -1.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.electric_field_angle_degrees = 180.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.spring_stiffness = 0.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_a.mass_kg = 0.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_b.mass_kg = -1.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_a.mass_kg = 0.009f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_b.mass_kg = 10000.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_a.mass_kg = 0.01f;
  config.body_b.mass_kg = 10000.0f;
  config.spring_enabled = false;
  CHECK(GetConfigError(config) == nullptr);
  config = SimulationConfig{};
  config.body_a.downhill_speed = 0.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_a.length_units = 0.0f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_b.charge = 1000.01f;
  ExpectInvalidConfig(config);
}

void TestPositionAndOverlapValidation() {
  SimulationConfig config;
  config.body_a.surface_x =
      GetMinimumBodySurfaceX(config.body_a, config) - 0.01f;
  ExpectInvalidConfig(config);
  config = SimulationConfig{};
  config.body_b.surface_x =
      GetMaximumBodySurfaceX(config.body_b, config) + 0.01f;
  ExpectInvalidConfig(config);

  config = SimulationConfig{};
  config.ramp_enabled = false;
  config.spring_enabled = false;
  config.body_a.surface_x = 100.0f;
  config.body_b.surface_x = 132.0f;
  CHECK(GetConfigError(config) == nullptr);
  config.body_b.surface_x = 131.99f;
  ExpectInvalidConfig(config);

  config = SimulationConfig{};
  config.real_floor_length_m = 0.1f;
  config.real_ramp_length_m = 10000.0f;
  ClampBodySurfaceX(config.body_a, config);
  ClampBodySurfaceX(config.body_b, config);
  ExpectInvalidConfig(config);
}

void TestDangerousFiniteCalibrationsAreRejected() {
  SimulationConfig config;
  config.real_floor_length_m = 10000.0f;
  config.real_ramp_length_m = 10000.0f;
  config.reference_speed_mps = 0.01f;
  config.body_a.mass_kg = 0.01f;
  config.body_a_engine_mass = 1000.0f;
  config.body_b.mass_kg = 1000.0f;
  config.body_a.charged = true;
  config.body_b.charged = true;
  config.body_a.charge = 1000.0f;
  config.body_b.charge = -1000.0f;
  config.electric_field_enabled = true;
  config.electric_field_strength_n_per_c = 1000000.0f;
  ClampBodySurfaceX(config.body_a, config);
  ClampBodySurfaceX(config.body_b, config);
  ExpectInvalidConfig(config);

  config = SimulationConfig{};
  config.body_a.downhill_speed = 5000.0f;
  ExpectInvalidConfig(config);

  config = SimulationConfig{};
  config.body_a_engine_mass = 0.01f;
  config.body_a.mass_kg = 1.0f;
  config.body_b.mass_kg = 0.01f;
  config.spring_stiffness = 100.0f;
  ExpectInvalidConfig(config);
}

void TestGeometryScaling() {
  SimulationConfig config;
  CHECK(Near(GetPixelsPerMeter(config), 200.0f));
  CHECK(Near(GetRampBottomX(config), 600.0f));
  CHECK(Near(GetRampTopX(config), static_cast<float>(kAreaWidth)));

  config.ramp_enabled = false;
  CHECK(Near(GetPixelsPerMeter(config), 400.0f));
  CHECK(Near(GetRampBottomX(config), static_cast<float>(kAreaWidth)));

  config = SimulationConfig{};
  const float default_bottom = GetRampBottomX(config);
  const float default_length = GetRampLengthPixels(config);
  config.real_floor_length_m *= 1000.0f;
  config.real_ramp_length_m *= 1000.0f;
  CHECK(Near(GetRampBottomX(config), default_bottom, 0.01f));
  CHECK(Near(GetRampLengthPixels(config), default_length, 0.01f));

  for (float angle : {5.0f, 30.0f, 45.0f}) {
    config = SimulationConfig{};
    config.ramp_angle_degrees = angle;
    CHECK(GetPixelsPerMeter(config) > 0.0f);
    CHECK(std::isfinite(GetPixelsPerMeter(config)));
    CHECK(GetRampTopX(config) <= static_cast<float>(kAreaWidth) + 0.01f);
  }
}

void TestSiCalibrationAndElectricFieldDirections() {
  SimulationConfig config;
  CHECK(Near(GetPixelSpeedPerMeterPerSecond(config), 200.0f));
  CHECK(Near(GetRealSecondsPerSimulationSecond(config), 1.0f));
  CHECK(Near(GetEngineMassPerKilogram(config), 1.0f));
  CHECK(Near(GetPixelAccelerationPerMeterPerSecondSquared(config), 200.0f));
  CHECK(Near(GetPixelGravity(config), 1960.0f, 0.01f));

  config.electric_field_enabled = true;
  config.electric_field_strength_n_per_c = 9.8f;
  for (const auto [angle, expected_x, expected_y] :
       {std::array{0.0f, 1960.0f, 0.0f}, std::array{90.0f, 0.0f, -1960.0f},
        std::array{-90.0f, 0.0f, 1960.0f},
        std::array{180.0f, -1960.0f, 0.0f}}) {
    config.electric_field_angle_degrees = angle;
    const tiny2d::Vec2 field = GetElectricField(config);
    CHECK(Near(field.x, expected_x, 0.01f));
    CHECK(Near(field.y, expected_y, 0.01f));
  }

  config.real_floor_length_m *= 2.0f;
  config.real_ramp_length_m *= 2.0f;
  config.reference_speed_mps = 2.0f;
  config.body_a.mass_kg = 2.0f;
  config.electric_field_angle_degrees = 0.0f;
  CHECK(Near(GetPixelsPerMeter(config), 100.0f));
  CHECK(Near(GetPixelSpeedPerMeterPerSecond(config), 100.0f));
  CHECK(Near(GetRealSecondsPerSimulationSecond(config), 1.0f));
  CHECK(Near(GetEngineMassPerKilogram(config), 0.5f));
  CHECK(Near(GetPixelGravity(config), 980.0f, 0.01f));

  config.body_a.charged = true;
  config.body_a.charge = 2.0f;
  const tiny2d::Rectangle body = CreateScene(config)[0];
  const tiny2d::Vec2 acceleration =
      tiny2d::GetLinearAcceleration(body, GetElectricField(config), 0.0f);
  const float real_acceleration =
      acceleration.x / GetPixelAccelerationPerMeterPerSecondSquared(config);
  CHECK(Near(real_acceleration, config.body_a.charge *
                                    config.electric_field_strength_n_per_c /
                                    config.body_a.mass_kg));
}

void TestBodyMassCalibrationUsesBodyAReference() {
  SimulationConfig config;
  config.body_a.mass_kg = 2.0f;
  config.body_a_engine_mass = 4.0f;
  config.body_b.mass_kg = 3.0f;

  CHECK(Near(GetEngineMassPerKilogram(config), 2.0f));
  CHECK(Near(GetBodyEngineMass(config.body_a, config), 4.0f));
  CHECK(Near(GetBodyEngineMass(config.body_b, config), 6.0f));
  CHECK(Near(CreateScene(config)[0].mass, 4.0f));
  CHECK(Near(CreateScene(config)[1].mass, 6.0f));

  config.body_b.mass_kg = 7.5f;
  CHECK(Near(GetEngineMassPerKilogram(config), 2.0f));
  CHECK(Near(CreateScene(config)[1].mass, 15.0f));

  config.body_a.mass_kg = 4.0f;
  CHECK(Near(GetEngineMassPerKilogram(config), 1.0f));
  CHECK(Near(CreateScene(config)[1].mass, 7.5f));
  config.body_a_engine_mass = 8.0f;
  CHECK(Near(GetEngineMassPerKilogram(config), 2.0f));
  CHECK(Near(CreateScene(config)[1].mass, 15.0f));

  config.body_a.mass_kg = 2.0f;
  config.body_a_engine_mass = 4.0f;
  config.body_b.mass_kg = 8.0f;
  config.body_b.charged = true;
  config.body_b.charge = 2.0f;
  config.electric_field_enabled = true;
  config.electric_field_strength_n_per_c = 12.0f;
  const tiny2d::Rectangle body_b = CreateScene(config)[1];
  const tiny2d::Vec2 acceleration =
      tiny2d::GetLinearAcceleration(body_b, GetElectricField(config), 0.0f);
  const float real_acceleration =
      acceleration.x / GetPixelAccelerationPerMeterPerSecondSquared(config);
  CHECK(Near(real_acceleration, 3.0f));
}

void TestBodyCreationAndClamping() {
  SimulationConfig config;
  const tiny2d::Rectangle ramp_body = CreateScene(config)[0];
  CHECK(Near(ramp_body.mass, config.body_a_engine_mass));
  CHECK(Near(ramp_body.angle, GetRampAngle(config)));
  CHECK(Near(std::hypot(ramp_body.velocity.x, ramp_body.velocity.y), 200.0f));
  CHECK(ramp_body.fixed_rotation);
  CHECK(ramp_body.charge == 0.0f);

  config.body_b.charged = true;
  config.body_b.charge = -3.0f;
  const tiny2d::Rectangle floor_body = CreateScene(config)[1];
  CHECK(Near(floor_body.mass, 3.0f));
  CHECK(Near(floor_body.angle, 0.0f));
  CHECK(Near(floor_body.position.y,
             static_cast<float>(kAreaHeight) - kUnitLength * 0.5f));
  CHECK(Near(floor_body.charge, -3.0f));

  config.body_a.surface_x = -1000.0f;
  ClampBodySurfaceX(config.body_a, config);
  CHECK(Near(config.body_a.surface_x,
             GetMinimumBodySurfaceX(config.body_a, config)));
  config.body_a.surface_x = 100000.0f;
  ClampBodySurfaceX(config.body_a, config);
  CHECK(Near(config.body_a.surface_x,
             GetMaximumBodySurfaceX(config.body_a, config)));
}

void TestSnapshotLookupAndAcceleration() {
  State lookup_state;
  CHECK(FindSnapshot(lookup_state, 0.0f) == nullptr);
  lookup_state.history.resize(3);
  lookup_state.history[0].time = 0.0;
  lookup_state.history[1].time = 1.0;
  lookup_state.history[2].time = 2.0;
  CHECK(FindSnapshot(lookup_state, -1.0) == &lookup_state.history[0]);
  CHECK(FindSnapshot(lookup_state, 0.0) == &lookup_state.history[0]);
  CHECK(FindSnapshot(lookup_state, 0.5) == &lookup_state.history[0]);
  CHECK(FindSnapshot(lookup_state, 0.51) == &lookup_state.history[1]);
  CHECK(FindSnapshot(lookup_state, 3.0) == &lookup_state.history[2]);
  CHECK(FindSnapshot(lookup_state, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);

  SimulationConfig config;
  config.electric_field_enabled = true;
  config.body_a.charged = true;
  State state;
  CHECK(Reset(config, state) == nullptr);
  const SimulationSnapshot& initial_snapshot = state.history.front();
  const tiny2d::Vec2 expected = tiny2d::GetLinearAcceleration(
      state.bodies[0], GetElectricField(config), GetPixelGravity(config));
  CHECK(Near(initial_snapshot.bodies[0].acceleration.x, expected.x));
  CHECK(Near(initial_snapshot.bodies[0].acceleration.y, expected.y));

  const tiny2d::Vec2 previous_velocity = state.bodies[0].velocity;
  CHECK(Step(state, kPhysicsStep) == nullptr);
  const SimulationSnapshot& stepped_snapshot = state.history.back();
  CHECK(
      Near(stepped_snapshot.bodies[0].acceleration.x,
           (state.bodies[0].velocity.x - previous_velocity.x) / kPhysicsStep));
  CHECK(
      Near(stepped_snapshot.bodies[0].acceleration.y,
           (state.bodies[0].velocity.y - previous_velocity.y) / kPhysicsStep));
  CHECK(NearTime(stepped_snapshot.time, state.time));
}

void TestBoundedSimulationHistory() {
  std::vector<HistorySample> history{{0.0, 0}};
  const std::vector<HistorySample> original = history;
  CHECK(!tiny2d::sandbox::AppendHistorySample(
      static_cast<std::vector<HistorySample>*>(nullptr), {1.0, 1},
      &HistorySample::time));
  CHECK(!tiny2d::sandbox::AppendHistorySample(
      &history, {1.0, 1}, static_cast<double HistorySample::*>(nullptr)));
  for (double invalid_time : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()}) {
    CHECK(!tiny2d::sandbox::AppendHistorySample(&history, {invalid_time, 1},
                                                &HistorySample::time));
  }
  CHECK(!tiny2d::sandbox::AppendHistorySample(&history, {0.0, 1},
                                              &HistorySample::time));
  CHECK(!tiny2d::sandbox::AppendHistorySample(&history, {-1.0, 1},
                                              &HistorySample::time));
  CHECK(history.size() == original.size());
  CHECK(history.front().time == original.front().time);
  CHECK(history.front().id == original.front().id);

  history.clear();
  history.reserve(tiny2d::sandbox::kMaxSimulationHistorySamples);
  for (std::size_t index = 0;
       index < tiny2d::sandbox::kMaxSimulationHistorySamples; ++index) {
    history.push_back({static_cast<double>(index), static_cast<int>(index)});
  }
  CHECK(tiny2d::sandbox::AppendHistorySample(
      &history,
      {static_cast<double>(tiny2d::sandbox::kMaxSimulationHistorySamples),
       static_cast<int>(tiny2d::sandbox::kMaxSimulationHistorySamples)},
      &HistorySample::time));
  CHECK(history.size() ==
        tiny2d::sandbox::kMaxSimulationHistorySamples / 2 + 2);
  for (std::size_t index = 0;
       index < tiny2d::sandbox::kMaxSimulationHistorySamples / 2; ++index) {
    CHECK(history[index].id == static_cast<int>(index * 2));
  }
  CHECK(history[tiny2d::sandbox::kMaxSimulationHistorySamples / 2].id ==
        static_cast<int>(tiny2d::sandbox::kMaxSimulationHistorySamples - 1));
  CHECK(history.back().id ==
        static_cast<int>(tiny2d::sandbox::kMaxSimulationHistorySamples));

  int next_id = history.back().id + 1;
  constexpr int kAdditionalCompactions = 3;
  const std::size_t append_count =
      kAdditionalCompactions * tiny2d::sandbox::kMaxSimulationHistorySamples;
  for (std::size_t index = 0; index < append_count; ++index) {
    CHECK(tiny2d::sandbox::AppendHistorySample(
        &history, {static_cast<double>(next_id), next_id},
        &HistorySample::time));
    ++next_id;
    CHECK(history.size() <= tiny2d::sandbox::kMaxSimulationHistorySamples);
  }
  CHECK(history.front().id == 0);
  CHECK(history.back().id == next_id - 1);
  for (std::size_t index = 1; index < history.size(); ++index) {
    CHECK(history[index - 1].time < history[index].time);
  }
}

void TestLargeSimulationTimeStillAdvances() {
  State state;
  CHECK(Reset(SimulationConfig{}, state) == nullptr);
  state.time = 1000000.0;
  const double previous_time = state.time;
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(state.time > previous_time);
  CHECK(state.history.back().time == state.time);
}

void TestSignedTelemetry() {
  SimulationConfig config;
  BodyTelemetry telemetry;
  telemetry.angle = GetRampAngle(config);
  telemetry.velocity = {-100.0f * std::cos(telemetry.angle),
                        -100.0f * std::sin(telemetry.angle)};
  CHECK(Near(GetSignedSpeed(telemetry, config), 100.0f));
  telemetry.velocity.x *= -1.0f;
  telemetry.velocity.y *= -1.0f;
  CHECK(Near(GetSignedSpeed(telemetry, config), -100.0f));
  telemetry.velocity = {};
  CHECK(GetSignedSpeed(telemetry, config) == 0.0f);

  const tiny2d::Rectangle ramp_body = CreateScene(config)[0];
  telemetry.position = ramp_body.position;
  telemetry.angle = ramp_body.angle;
  CHECK(GetSignedDistance(telemetry, ramp_body.width, config) >= 0.0f);

  const tiny2d::Rectangle floor_body = CreateScene(config)[1];
  telemetry.position = floor_body.position;
  telemetry.angle = floor_body.angle;
  CHECK(Near(GetSignedDistance(telemetry, floor_body.width, config), 0.0f));
  telemetry.position.x -= 50.0f;
  CHECK(Near(GetSignedDistance(telemetry, floor_body.width, config), -50.0f));
  telemetry.acceleration = {-20.0f, 0.0f};
  CHECK(Near(GetSignedSurfaceAcceleration(telemetry, config), 20.0f));
}

void TestJunctionTransitionsPreserveSpeed() {
  SimulationConfig config;
  config.spring_enabled = false;
  config.body_b.surface_x = GetMinimumBodySurfaceX(config.body_b, config);
  config.body_a.surface_x = GetMinimumBodySurfaceX(config.body_a, config);
  const float ramp_angle = GetRampAngle(config);
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);

  State state;
  CHECK(Reset(config, state) == nullptr);
  const float initial_speed =
      std::hypot(state.bodies[0].velocity.x, state.bodies[0].velocity.y);
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(Near(state.bodies[0].angle, 0.0f));
  CHECK(Near(std::hypot(state.bodies[0].velocity.x, state.bodies[0].velocity.y),
             initial_speed, 0.01f));
  CHECK(state.bodies[0].velocity.x < 0.0f);
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(Near(state.bodies[0].angle, 0.0f));

  config = SimulationConfig{};
  config.spring_enabled = false;
  config.body_b.surface_x = GetMaximumBodySurfaceX(config.body_b, config);
  config.body_b.downhill_speed = -200.0f;
  CHECK(Reset(config, state) == nullptr);
  CHECK(Step(state, kPhysicsStep) == nullptr);
  const tiny2d::Rectangle& body = state.bodies[1];
  CHECK(Near(body.angle, ramp_angle));
  CHECK(body.velocity.x > 0.0f);
  CHECK(body.velocity.y < 0.0f);
  CHECK(Near(body.velocity.x * sine - body.velocity.y * cosine, 0.0f, 0.01f));
  CHECK(body.angular_velocity == 0.0f);
}

void TestSurfaceConstraintIsStableAndIdempotent() {
  SimulationConfig config;
  config.spring_enabled = false;
  State state;
  CHECK(Reset(config, state) == nullptr);
  const float ramp_angle = GetRampAngle(config);
  const tiny2d::Vec2 normal{std::sin(ramp_angle), -std::cos(ramp_angle)};
  state.bodies[0].position.x += normal.x * 20.0f;
  state.bodies[0].position.y += normal.y * 20.0f;
  state.bodies[0].velocity.x += normal.x * 50.0f;
  state.bodies[0].velocity.y += normal.y * 50.0f;

  for (int step = 0; step < 2; ++step) {
    CHECK(Step(state, kPhysicsStep) == nullptr);
    const tiny2d::Rectangle& body = state.bodies[0];
    const float normal_distance =
        (body.position.x - GetRampBottomX(config)) * normal.x +
        (body.position.y - static_cast<float>(kAreaHeight)) * normal.y;
    const float normal_speed =
        body.velocity.x * normal.x + body.velocity.y * normal.y;
    CHECK(Near(normal_distance, body.height * 0.5f));
    CHECK(Near(normal_speed, 0.0f));
  }
}

void TestSpringForceBoundariesAndMassScaling() {
  SimulationConfig config;
  config.body_a.mass_kg = 2.0f;
  config.body_a_engine_mass = 1.0f;
  config.body_b.mass_kg = 4.0f;
  State state;
  CHECK(Reset(config, state) == nullptr);
  tiny2d::Rectangle& light_body = state.bodies[1];
  CHECK(Near(light_body.mass, 2.0f));
  light_body.position.x =
      GetSpringRestX(config) - 10.0f + light_body.width * 0.5f;
  light_body.velocity = {};
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(Near(light_body.velocity.x,
             config.spring_stiffness * 10.0f / light_body.mass * kPhysicsStep));
  const float light_body_speed = light_body.velocity.x;

  config.body_b.mass_kg = 8.0f;
  CHECK(Reset(config, state) == nullptr);
  tiny2d::Rectangle& heavy_body = state.bodies[1];
  heavy_body.position.x =
      GetSpringRestX(config) - 10.0f + heavy_body.width * 0.5f;
  heavy_body.velocity = {};
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(Near(heavy_body.mass, 4.0f));
  CHECK(Near(heavy_body.velocity.x, light_body_speed * 0.5f));

  CHECK(Reset(config, state) == nullptr);
  tiny2d::Rectangle& resting_body = state.bodies[1];
  resting_body.position.x = GetSpringRestX(config) + resting_body.width * 0.5f;
  resting_body.velocity = {};
  CHECK(Step(state, kPhysicsStep) == nullptr);
  CHECK(Near(resting_body.velocity.x, 0.0f));

  state.bodies[0].angle = 0.0f;
  state.bodies[0].position.x = GetSpringRestX(config) - 20.0f;
  state.bodies[0].position.y =
      static_cast<float>(kAreaHeight) - state.bodies[0].height * 0.5f;
  CHECK(Near(GetSpringEndX(state),
             state.bodies[0].position.x - state.bodies[0].width * 0.5f));
}

void TestAirborneDetection() {
  SimulationConfig config;
  State state;
  CHECK(Reset(config, state) == nullptr);
  CHECK(Step(state, kPhysicsStep) == nullptr);

  config.electric_field_enabled = true;
  config.body_a.charged = true;
  config.electric_field_angle_degrees = 90.0f;
  config.electric_field_strength_n_per_c = 5.0f;
  CHECK(Reset(config, state) == nullptr);
  CHECK(Step(state, kPhysicsStep) == nullptr);
  config.electric_field_strength_n_per_c = 20.0f;
  CHECK(Reset(config, state) == nullptr);
  CHECK(Step(state, kPhysicsStep) != nullptr);

  config.body_a.charged = false;
  CHECK(Reset(config, state) == nullptr);
  CHECK(Step(state, kPhysicsStep) == nullptr);
  state.bodies.clear();
  CHECK(Step(state, kPhysicsStep) != nullptr);
}

void TestNonFiniteStateDetection() {
  SimulationConfig config;
  State state;
  CHECK(Reset(config, state) == nullptr);
  state.bodies[0].position.x = std::numeric_limits<float>::quiet_NaN();
  CHECK(Step(state, kPhysicsStep) != nullptr);

  CHECK(Reset(config, state) == nullptr);
  state.bodies[1].velocity.y = std::numeric_limits<float>::infinity();
  CHECK(Step(state, kPhysicsStep) != nullptr);
}

void TestDefaultScenarioLongRunAndDeterminism() {
  const SimulationConfig config;
  State first;
  State second;
  CHECK(Reset(config, first) == nullptr);
  CHECK(Reset(config, second) == nullptr);
  for (int step = 0; step < 20000; ++step) {
    CHECK(Step(first, kPhysicsStep) == nullptr);
    for (const tiny2d::Rectangle& body : first.bodies) {
      CheckBodyFinite(body);
    }
    if (step < 5000) {
      CHECK(Step(second, kPhysicsStep) == nullptr);
    }
  }

  State replay;
  CHECK(Reset(config, replay) == nullptr);
  for (int step = 0; step < 5000; ++step) {
    CHECK(Step(replay, kPhysicsStep) == nullptr);
  }
  CHECK(replay.bodies.size() == second.bodies.size());
  CHECK(NearTime(replay.time, second.time));
  CHECK(replay.history.size() == second.history.size());
  for (std::size_t i = 0; i < replay.bodies.size(); ++i) {
    CHECK(Near(replay.bodies[i].position.x, second.bodies[i].position.x));
    CHECK(Near(replay.bodies[i].position.y, second.bodies[i].position.y));
    CHECK(Near(replay.bodies[i].velocity.x, second.bodies[i].velocity.x));
    CHECK(Near(replay.bodies[i].velocity.y, second.bodies[i].velocity.y));
  }
}

void TestFixedSeedSafeConfigurations() {
  std::mt19937 random_engine(0x54494e59u);
  std::uniform_real_distribution<float> angle_distribution(10.0f, 40.0f);
  std::uniform_real_distribution<float> length_distribution(2.0f, 8.0f);
  std::uniform_real_distribution<float> speed_distribution(100.0f, 300.0f);
  std::uniform_real_distribution<float> mass_distribution(0.5f, 5.0f);
  std::uniform_real_distribution<float> coefficient_distribution(0.0f, 1.0f);

  int tested_configurations = 0;
  for (int attempt = 0; attempt < 1000 && tested_configurations < 64;
       ++attempt) {
    SimulationConfig config;
    config.ramp_angle_degrees = angle_distribution(random_engine);
    config.real_floor_length_m = length_distribution(random_engine);
    config.real_ramp_length_m = length_distribution(random_engine);
    config.friction = coefficient_distribution(random_engine);
    config.restitution = coefficient_distribution(random_engine);
    config.spring_enabled = false;
    config.body_a.downhill_speed = speed_distribution(random_engine);
    config.body_b.mass_kg = mass_distribution(random_engine);
    config.body_a.surface_x = (GetMinimumBodySurfaceX(config.body_a, config) +
                               GetMaximumBodySurfaceX(config.body_a, config)) *
                              0.5f;
    config.body_b.surface_x = GetMinimumBodySurfaceX(config.body_b, config);
    if (GetConfigError(config) != nullptr) {
      continue;
    }

    State state;
    CHECK(Reset(config, state) == nullptr);
    CHECK(Near(state.bodies[1].mass / GetEngineMassPerKilogram(config),
               config.body_b.mass_kg));
    for (int step = 0; step < 240; ++step) {
      CHECK(Step(state, kPhysicsStep) == nullptr);
      for (const tiny2d::Rectangle& body : state.bodies) {
        CheckBodyFinite(body);
      }
    }
    ++tested_configurations;
  }
  CHECK(tested_configurations == 64);
}

using TestFunction = void (*)();

struct NamedTest {
  const char* name;
  TestFunction function;
};

}  // namespace

int RunTests() {
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"fixed-step clock", TestFixedStepClock},
      NamedTest{"default and feature configurations",
                TestDefaultAndFeatureConfigurations},
      NamedTest{"state lifecycle contracts", TestStateLifecycleContracts},
      NamedTest{"engine validation exceptions propagate",
                TestEngineValidationExceptionsPropagate},
      NamedTest{"all float fields reject NaN and infinity",
                TestEveryFloatFieldRejectsNonFiniteValues},
      NamedTest{"invalid ranges and degenerate inputs",
                TestInvalidRangesAndDegenerateInputs},
      NamedTest{"position and overlap validation",
                TestPositionAndOverlapValidation},
      NamedTest{"dangerous finite calibrations",
                TestDangerousFiniteCalibrationsAreRejected},
      NamedTest{"geometry scaling", TestGeometryScaling},
      NamedTest{"SI and electric calibration",
                TestSiCalibrationAndElectricFieldDirections},
      NamedTest{"Body A mass scale and Body B real mass",
                TestBodyMassCalibrationUsesBodyAReference},
      NamedTest{"body creation and clamping", TestBodyCreationAndClamping},
      NamedTest{"snapshot lookup and acceleration",
                TestSnapshotLookupAndAcceleration},
      NamedTest{"bounded simulation history", TestBoundedSimulationHistory},
      NamedTest{"large simulation time advances",
                TestLargeSimulationTimeStillAdvances},
      NamedTest{"signed telemetry", TestSignedTelemetry},
      NamedTest{"junction transitions", TestJunctionTransitionsPreserveSpeed},
      NamedTest{"surface constraint",
                TestSurfaceConstraintIsStableAndIdempotent},
      NamedTest{"spring force", TestSpringForceBoundariesAndMassScaling},
      NamedTest{"airborne detection", TestAirborneDetection},
      NamedTest{"non-finite state detection", TestNonFiniteStateDetection},
      NamedTest{"long run and determinism",
                TestDefaultScenarioLongRunAndDeterminism},
      NamedTest{"fixed-seed safe configurations",
                TestFixedSeedSafeConfigurations},
  };

  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << check_count << " checks passed\n";
  return 0;
}

}  // namespace tiny2d::sandbox::incline_spring

int main() { return tiny2d::sandbox::incline_spring::RunTests(); }
