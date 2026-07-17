#define SDL_MAIN_HANDLED
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>

#include "incline_spring_simulation.cc"

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

void ExpectInvalidConfig(const SimulationConfig& config) {
  CHECK(GetConfigError(config) != nullptr);
}

void CheckBodyFinite(const tiny2d::Rectangle& body) {
  CHECK(IsFiniteRectangle(body));
  for (const tiny2d::Vec2 vertex : tiny2d::GetVertices(body)) {
    CHECK(std::isfinite(vertex.x));
    CHECK(std::isfinite(vertex.y));
  }
}

std::vector<tiny2d::Rectangle> CreateScene(const SimulationConfig& config) {
  std::vector<tiny2d::Rectangle> bodies = {CreateBody(config.body_a, config),
                                           CreateBody(config.body_b, config)};
  if (config.ramp_enabled) {
    bodies.push_back(CreateRamp(config));
  }
  return bodies;
}

bool StepScenario(std::vector<tiny2d::Rectangle>* bodies,
                  const SimulationConfig& config) {
  if (config.ramp_enabled) {
    TransitionBodiesToFloor(*bodies, config, kPhysicsStep);
  }
  if (config.spring_enabled) {
    ApplySpringForce(*bodies, config, kPhysicsStep);
  }
  if (config.ramp_enabled) {
    TransitionBodiesToRamp(*bodies, config, kPhysicsStep);
  }
  if (GetAirborneError(*bodies, config) != nullptr) {
    return false;
  }
  tiny2d::Update(*bodies, kPhysicsStep, static_cast<float>(kAreaWidth),
                 static_cast<float>(kAreaHeight), config.restitution,
                 config.friction, GetElectricField(config),
                 GetPixelGravity(config));
  if (config.ramp_enabled) {
    TransitionBodiesToFloor(*bodies, config, 0.0f);
    TransitionBodiesToRamp(*bodies, config, 0.0f);
  }
  if (GetAirborneError(*bodies, config) != nullptr) {
    return false;
  }
  ConstrainBodiesToSurfaces(*bodies, config);
  return true;
}

void TestDefaultAndFeatureConfigurations() {
  SimulationConfig config;
  CHECK(GetConfigError(config) == nullptr);
  CHECK(Near(config.body_a.mass_kg, 1.0f));
  CHECK(Near(config.body_b.mass_kg, 3.0f));
  CHECK(Near(CreateBody(config.body_a, config).mass, 1.0f));
  CHECK(Near(CreateBody(config.body_b, config).mass, 3.0f));
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
  ClampBodySurfaceX(&config.body_a, config);
  ClampBodySurfaceX(&config.body_b, config);
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
  ClampBodySurfaceX(&config.body_a, config);
  ClampBodySurfaceX(&config.body_b, config);
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
  const tiny2d::Rectangle body = CreateBody(config.body_a, config);
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
  CHECK(Near(CreateBody(config.body_a, config).mass, 4.0f));
  CHECK(Near(CreateBody(config.body_b, config).mass, 6.0f));

  config.body_b.mass_kg = 7.5f;
  CHECK(Near(GetEngineMassPerKilogram(config), 2.0f));
  CHECK(Near(CreateBody(config.body_b, config).mass, 15.0f));

  config.body_a.mass_kg = 4.0f;
  CHECK(Near(GetEngineMassPerKilogram(config), 1.0f));
  CHECK(Near(CreateBody(config.body_b, config).mass, 7.5f));
  config.body_a_engine_mass = 8.0f;
  CHECK(Near(GetEngineMassPerKilogram(config), 2.0f));
  CHECK(Near(CreateBody(config.body_b, config).mass, 15.0f));

  config.body_a.mass_kg = 2.0f;
  config.body_a_engine_mass = 4.0f;
  config.body_b.mass_kg = 8.0f;
  config.body_b.charged = true;
  config.body_b.charge = 2.0f;
  config.electric_field_enabled = true;
  config.electric_field_strength_n_per_c = 12.0f;
  const tiny2d::Rectangle body_b = CreateBody(config.body_b, config);
  const tiny2d::Vec2 acceleration =
      tiny2d::GetLinearAcceleration(body_b, GetElectricField(config), 0.0f);
  const float real_acceleration =
      acceleration.x / GetPixelAccelerationPerMeterPerSecondSquared(config);
  CHECK(Near(real_acceleration, 3.0f));
}

void TestBodyCreationAndClamping() {
  SimulationConfig config;
  const tiny2d::Rectangle ramp_body = CreateBody(config.body_a, config);
  CHECK(Near(ramp_body.mass, config.body_a_engine_mass));
  CHECK(Near(ramp_body.angle, GetRampAngle(config)));
  CHECK(Near(std::hypot(ramp_body.velocity.x, ramp_body.velocity.y), 200.0f));
  CHECK(ramp_body.fixed_rotation);
  CHECK(ramp_body.charge == 0.0f);

  config.body_b.charged = true;
  config.body_b.charge = -3.0f;
  const tiny2d::Rectangle floor_body = CreateBody(config.body_b, config);
  CHECK(Near(floor_body.mass, 3.0f));
  CHECK(Near(floor_body.angle, 0.0f));
  CHECK(Near(floor_body.position.y,
             static_cast<float>(kAreaHeight) - kUnitLength * 0.5f));
  CHECK(Near(floor_body.charge, -3.0f));

  config.body_a.surface_x = -1000.0f;
  ClampBodySurfaceX(&config.body_a, config);
  CHECK(Near(config.body_a.surface_x,
             GetMinimumBodySurfaceX(config.body_a, config)));
  config.body_a.surface_x = 100000.0f;
  ClampBodySurfaceX(&config.body_a, config);
  CHECK(Near(config.body_a.surface_x,
             GetMaximumBodySurfaceX(config.body_a, config)));
}

void TestSnapshotLookupAndAcceleration() {
  std::vector<SimulationSnapshot> history;
  CHECK(FindSnapshot(history, 0.0f) == nullptr);
  history.resize(3);
  history[0].time = 0.0f;
  history[1].time = 1.0f;
  history[2].time = 2.0f;
  CHECK(FindSnapshot(history, -1.0f) == &history[0]);
  CHECK(FindSnapshot(history, 0.0f) == &history[0]);
  CHECK(FindSnapshot(history, 0.5f) == &history[0]);
  CHECK(FindSnapshot(history, 0.51f) == &history[1]);
  CHECK(FindSnapshot(history, 3.0f) == &history[2]);
  CHECK(FindSnapshot(history, std::numeric_limits<float>::quiet_NaN()) ==
        nullptr);

  SimulationConfig config;
  config.electric_field_enabled = true;
  config.body_a.charged = true;
  std::vector<tiny2d::Rectangle> bodies = CreateScene(config);
  std::array<tiny2d::Vec2, 2> previous_velocities = {bodies[0].velocity,
                                                     bodies[1].velocity};
  SimulationSnapshot snapshot =
      MakeSnapshot(bodies, previous_velocities, 0.0f, 0.0f, config);
  const tiny2d::Vec2 expected = tiny2d::GetLinearAcceleration(
      bodies[0], GetElectricField(config), GetPixelGravity(config));
  CHECK(Near(snapshot.bodies[0].acceleration.x, expected.x));
  CHECK(Near(snapshot.bodies[0].acceleration.y, expected.y));

  bodies[0].velocity.x += 10.0f;
  bodies[0].velocity.y -= 20.0f;
  snapshot = MakeSnapshot(bodies, previous_velocities, 1.0f, 0.5f, config);
  CHECK(Near(snapshot.bodies[0].acceleration.x, 20.0f));
  CHECK(Near(snapshot.bodies[0].acceleration.y, -40.0f));
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

  const tiny2d::Rectangle ramp_body = CreateBody(config.body_a, config);
  telemetry.position = ramp_body.position;
  telemetry.angle = ramp_body.angle;
  CHECK(GetSignedDistance(telemetry, ramp_body.width, config) >= 0.0f);

  const tiny2d::Rectangle floor_body = CreateBody(config.body_b, config);
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
  const float ramp_angle = GetRampAngle(config);
  const float cosine = std::cos(ramp_angle);
  const float sine = std::sin(ramp_angle);

  config.body_a.surface_x = GetMinimumBodySurfaceX(config.body_a, config);
  std::vector<tiny2d::Rectangle> bodies = {CreateBody(config.body_a, config)};
  const float initial_speed =
      std::hypot(bodies[0].velocity.x, bodies[0].velocity.y);
  TransitionBodiesToFloor(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].angle, 0.0f));
  CHECK(Near(std::hypot(bodies[0].velocity.x, bodies[0].velocity.y),
             initial_speed));
  CHECK(bodies[0].velocity.x < 0.0f);
  TransitionBodiesToFloor(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].angle, 0.0f));

  config.body_b.surface_x = GetMaximumBodySurfaceX(config.body_b, config);
  config.body_b.downhill_speed = -200.0f;
  bodies = {CreateBody(config.body_b, config)};
  TransitionBodiesToRamp(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].angle, ramp_angle));
  CHECK(Near(bodies[0].velocity.x, cosine * 200.0f));
  CHECK(Near(bodies[0].velocity.y, sine * 200.0f));
  CHECK(bodies[0].angular_velocity == 0.0f);

  bodies[0].velocity = {-cosine * 10.0f, -sine * 10.0f};
  TransitionBodiesToRamp(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].angle, ramp_angle));
}

void TestSurfaceConstraintIsStableAndIdempotent() {
  SimulationConfig config;
  std::vector<tiny2d::Rectangle> bodies = {CreateBody(config.body_a, config)};
  const float ramp_angle = GetRampAngle(config);
  const tiny2d::Vec2 normal{std::sin(ramp_angle), -std::cos(ramp_angle)};
  bodies[0].position.x += normal.x * 20.0f;
  bodies[0].position.y += normal.y * 20.0f;
  bodies[0].velocity.x += normal.x * 50.0f;
  bodies[0].velocity.y += normal.y * 50.0f;
  ConstrainBodiesToSurfaces(bodies, config);
  const float normal_distance =
      (bodies[0].position.x - GetRampBottomX(config)) * normal.x +
      (bodies[0].position.y - static_cast<float>(kAreaHeight)) * normal.y;
  const float normal_speed =
      bodies[0].velocity.x * normal.x + bodies[0].velocity.y * normal.y;
  CHECK(Near(normal_distance, bodies[0].height * 0.5f));
  CHECK(Near(normal_speed, 0.0f));
  const tiny2d::Rectangle constrained = bodies[0];
  ConstrainBodiesToSurfaces(bodies, config);
  CHECK(Near(bodies[0].position.x, constrained.position.x));
  CHECK(Near(bodies[0].position.y, constrained.position.y));
  CHECK(Near(bodies[0].velocity.x, constrained.velocity.x));
  CHECK(Near(bodies[0].velocity.y, constrained.velocity.y));
}

void TestSpringForceBoundariesAndMassScaling() {
  SimulationConfig config;
  config.body_a.mass_kg = 2.0f;
  config.body_a_engine_mass = 1.0f;
  config.body_b.mass_kg = 4.0f;
  std::vector<tiny2d::Rectangle> bodies = {CreateBody(config.body_b, config)};
  CHECK(Near(bodies[0].mass, 2.0f));
  bodies[0].position.x =
      GetSpringRestX(config) - 10.0f + bodies[0].width * 0.5f;
  bodies[0].velocity = {};
  ApplySpringForce(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].velocity.x,
             config.spring_stiffness * 10.0f / bodies[0].mass * kPhysicsStep));
  const float light_body_speed = bodies[0].velocity.x;

  config.body_b.mass_kg = 8.0f;
  bodies = {CreateBody(config.body_b, config)};
  bodies[0].position.x =
      GetSpringRestX(config) - 10.0f + bodies[0].width * 0.5f;
  bodies[0].velocity = {};
  ApplySpringForce(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].mass, 4.0f));
  CHECK(Near(bodies[0].velocity.x, light_body_speed * 0.5f));

  bodies[0].position.x = GetSpringRestX(config) + bodies[0].width * 0.5f;
  bodies[0].velocity = {};
  ApplySpringForce(bodies, config, kPhysicsStep);
  CHECK(Near(bodies[0].velocity.x, 0.0f));

  tiny2d::Rectangle farther_left = bodies[0];
  farther_left.position.x = GetSpringRestX(config) - 20.0f;
  bodies.push_back(farther_left);
  CHECK(FindSpringBodyIndex(bodies, config) == 1);
}

void TestAirborneDetection() {
  SimulationConfig config;
  std::vector<tiny2d::Rectangle> bodies = CreateScene(config);
  CHECK(GetAirborneError(bodies, config) == nullptr);

  config.electric_field_enabled = true;
  config.body_a.charged = true;
  config.electric_field_angle_degrees = 90.0f;
  config.electric_field_strength_n_per_c = 5.0f;
  bodies = CreateScene(config);
  CHECK(GetAirborneError(bodies, config) == nullptr);
  config.electric_field_strength_n_per_c = 20.0f;
  CHECK(GetAirborneError(bodies, config) != nullptr);

  config.body_a.charged = false;
  bodies = CreateScene(config);
  CHECK(GetAirborneError(bodies, config) == nullptr);
  bodies.clear();
  CHECK(GetAirborneError(bodies, config) == nullptr);
}

void TestNonFiniteStateDetection() {
  SimulationConfig config;
  std::vector<tiny2d::Rectangle> bodies = CreateScene(config);
  CHECK(GetNonFiniteStateError(bodies) == nullptr);
  bodies[0].position.x = std::numeric_limits<float>::quiet_NaN();
  CHECK(GetNonFiniteStateError(bodies) != nullptr);
  bodies = CreateScene(config);
  bodies[1].velocity.y = std::numeric_limits<float>::infinity();
  CHECK(GetNonFiniteStateError(bodies) != nullptr);
}

void TestDefaultScenarioLongRunAndDeterminism() {
  const SimulationConfig config;
  std::vector<tiny2d::Rectangle> first = CreateScene(config);
  std::vector<tiny2d::Rectangle> second = CreateScene(config);
  for (int step = 0; step < 20000; ++step) {
    CHECK(StepScenario(&first, config));
    for (const tiny2d::Rectangle& body : first) {
      CheckBodyFinite(body);
    }
    if (step < 5000) {
      CHECK(StepScenario(&second, config));
    }
  }

  std::vector<tiny2d::Rectangle> replay = CreateScene(config);
  for (int step = 0; step < 5000; ++step) {
    CHECK(StepScenario(&replay, config));
  }
  CHECK(replay.size() == second.size());
  for (std::size_t i = 0; i < replay.size(); ++i) {
    CHECK(Near(replay[i].position.x, second[i].position.x));
    CHECK(Near(replay[i].position.y, second[i].position.y));
    CHECK(Near(replay[i].velocity.x, second[i].velocity.x));
    CHECK(Near(replay[i].velocity.y, second[i].velocity.y));
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

    std::vector<tiny2d::Rectangle> bodies = CreateScene(config);
    CHECK(Near(bodies[1].mass / GetEngineMassPerKilogram(config),
               config.body_b.mass_kg));
    for (int step = 0; step < 240; ++step) {
      CHECK(StepScenario(&bodies, config));
      for (const tiny2d::Rectangle& body : bodies) {
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

int main() {
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"default and feature configurations",
                TestDefaultAndFeatureConfigurations},
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
