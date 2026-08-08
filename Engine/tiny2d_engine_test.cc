#include "tiny2d_engine.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

constexpr float kTolerance = 0.001f;

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

bool NearlyEqual(float a, float b, float tolerance = kTolerance) {
  return std::abs(a - b) <=
         tolerance * std::max({1.0f, std::abs(a), std::abs(b)});
}

bool IsFinite(tiny2d::Vec2 vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y);
}

bool IsFinite(const tiny2d::Rectangle& rectangle) {
  return std::isfinite(rectangle.mass) && IsFinite(rectangle.position) &&
         IsFinite(rectangle.velocity) && std::isfinite(rectangle.angle) &&
         std::isfinite(rectangle.angular_velocity) &&
         std::isfinite(rectangle.width) && std::isfinite(rectangle.height) &&
         std::isfinite(rectangle.charge) && IsFinite(rectangle.applied_force) &&
         std::isfinite(rectangle.applied_torque) &&
         std::isfinite(rectangle.linear_damping_rate) &&
         std::isfinite(rectangle.angular_damping_rate) &&
         std::isfinite(rectangle.material.restitution) &&
         std::isfinite(rectangle.material.static_friction) &&
         std::isfinite(rectangle.material.kinetic_friction);
}

bool SameFloat(float a, float b) {
  return a == b || (std::isnan(a) && std::isnan(b));
}

bool SameMaterial(const tiny2d::CollisionMaterial& a,
                  const tiny2d::CollisionMaterial& b) {
  return SameFloat(a.restitution, b.restitution) &&
         SameFloat(a.static_friction, b.static_friction) &&
         SameFloat(a.kinetic_friction, b.kinetic_friction);
}

bool SameRectangle(const tiny2d::Rectangle& a, const tiny2d::Rectangle& b) {
  return SameFloat(a.mass, b.mass) && SameFloat(a.position.x, b.position.x) &&
         SameFloat(a.position.y, b.position.y) &&
         SameFloat(a.velocity.x, b.velocity.x) &&
         SameFloat(a.velocity.y, b.velocity.y) && SameFloat(a.angle, b.angle) &&
         SameFloat(a.angular_velocity, b.angular_velocity) &&
         SameFloat(a.width, b.width) && SameFloat(a.height, b.height) &&
         a.fixed_rotation == b.fixed_rotation &&
         SameFloat(a.charge, b.charge) &&
         SameFloat(a.applied_force.x, b.applied_force.x) &&
         SameFloat(a.applied_force.y, b.applied_force.y) &&
         SameFloat(a.applied_torque, b.applied_torque) &&
         SameFloat(a.linear_damping_rate, b.linear_damping_rate) &&
         SameFloat(a.angular_damping_rate, b.angular_damping_rate) &&
         SameMaterial(a.material, b.material);
}

bool SameCircle(const tiny2d::Circle& a, const tiny2d::Circle& b) {
  return SameFloat(a.mass, b.mass) && SameFloat(a.position.x, b.position.x) &&
         SameFloat(a.position.y, b.position.y) &&
         SameFloat(a.velocity.x, b.velocity.x) &&
         SameFloat(a.velocity.y, b.velocity.y) && SameFloat(a.angle, b.angle) &&
         SameFloat(a.angular_velocity, b.angular_velocity) &&
         SameFloat(a.radius, b.radius) && a.inertia_model == b.inertia_model &&
         a.fixed_rotation == b.fixed_rotation &&
         SameFloat(a.charge, b.charge) &&
         SameFloat(a.applied_force.x, b.applied_force.x) &&
         SameFloat(a.applied_force.y, b.applied_force.y) &&
         SameFloat(a.applied_torque, b.applied_torque) &&
         SameFloat(a.linear_damping_rate, b.linear_damping_rate) &&
         SameFloat(a.angular_damping_rate, b.angular_damping_rate) &&
         SameMaterial(a.material, b.material);
}

bool IsFinite(const tiny2d::Circle& circle) {
  return std::isfinite(circle.mass) && IsFinite(circle.position) &&
         IsFinite(circle.velocity) && std::isfinite(circle.angle) &&
         std::isfinite(circle.angular_velocity) &&
         std::isfinite(circle.radius) && std::isfinite(circle.charge) &&
         IsFinite(circle.applied_force) &&
         std::isfinite(circle.applied_torque) &&
         std::isfinite(circle.linear_damping_rate) &&
         std::isfinite(circle.angular_damping_rate) &&
         std::isfinite(circle.material.restitution) &&
         std::isfinite(circle.material.static_friction) &&
         std::isfinite(circle.material.kinetic_friction);
}

float TotalMomentumX(const std::vector<tiny2d::Rectangle>& rectangles) {
  float momentum = 0.0f;
  for (const tiny2d::Rectangle& rectangle : rectangles) {
    momentum += rectangle.mass * rectangle.velocity.x;
  }
  return momentum;
}

float TranslationalKineticEnergy(
    const std::vector<tiny2d::Rectangle>& rectangles) {
  float energy = 0.0f;
  for (const tiny2d::Rectangle& rectangle : rectangles) {
    energy += 0.5f * rectangle.mass *
              (rectangle.velocity.x * rectangle.velocity.x +
               rectangle.velocity.y * rectangle.velocity.y);
  }
  return energy;
}

void CheckInsideArea(const tiny2d::Rectangle& rectangle, float area_width,
                     float area_height) {
  for (const tiny2d::Vec2 vertex : tiny2d::GetVertices(rectangle)) {
    CHECK(vertex.x >= -kTolerance);
    CHECK(vertex.x <= area_width + kTolerance);
    CHECK(vertex.y >= -kTolerance);
    CHECK(vertex.y <= area_height + kTolerance);
  }
}

template <typename Function>
void CheckInvalidArgument(Function function) {
  bool threw = false;
  try {
    function();
  } catch (const std::invalid_argument&) {
    threw = true;
  } catch (...) {
    FailCheck("expected std::invalid_argument", __FILE__, __LINE__);
  }
  CHECK(threw);
}

float Distance(tiny2d::Vec2 a, tiny2d::Vec2 b) {
  const float delta_x = b.x - a.x;
  const float delta_y = b.y - a.y;
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

void CheckInvalidUpdate(std::vector<tiny2d::Rectangle> rectangles,
                        float delta_time = 1.0f / 120.0f,
                        float area_width = 1000.0f, float area_height = 1000.0f,
                        float restitution = 1.0f, float friction = 0.0f,
                        tiny2d::Vec2 electric_field = {},
                        float gravity = 98.1f) {
  const std::vector<tiny2d::Rectangle> original = rectangles;
  CheckInvalidArgument([&] {
    tiny2d::Update(rectangles, delta_time, area_width, area_height, restitution,
                   friction, electric_field, gravity);
  });
  CHECK(rectangles.size() == original.size());
  for (std::size_t i = 0; i < rectangles.size(); ++i) {
    CHECK(SameRectangle(rectangles[i], original[i]));
  }
}

void CheckInvalidMixedUpdate(std::vector<tiny2d::Rectangle> rectangles,
                             std::vector<tiny2d::Circle> circles,
                             float delta_time = 1.0f / 120.0f,
                             float area_width = 1000.0f,
                             float area_height = 1000.0f,
                             float restitution = 1.0f, float friction = 0.0f,
                             tiny2d::Vec2 electric_field = {},
                             float gravity = 98.1f,
                             float restitution_velocity_threshold = 0.01f,
                             bool enable_circle_circle_ccd = false) {
  const std::vector<tiny2d::Rectangle> original_rectangles = rectangles;
  const std::vector<tiny2d::Circle> original_circles = circles;
  CheckInvalidArgument([&] {
    tiny2d::Update(rectangles, circles, delta_time, area_width, area_height,
                   restitution, friction, electric_field, gravity,
                   restitution_velocity_threshold, enable_circle_circle_ccd);
  });
  CHECK(rectangles.size() == original_rectangles.size());
  CHECK(circles.size() == original_circles.size());
  for (std::size_t i = 0; i < rectangles.size(); ++i) {
    CHECK(SameRectangle(rectangles[i], original_rectangles[i]));
  }
  for (std::size_t i = 0; i < circles.size(); ++i) {
    CHECK(SameCircle(circles[i], original_circles[i]));
  }
}

template <typename Function>
void CheckInvalidRectangleOperation(tiny2d::Rectangle rectangle,
                                    Function function) {
  const tiny2d::Rectangle original = rectangle;
  CheckInvalidArgument([&] { function(rectangle); });
  CHECK(SameRectangle(rectangle, original));
}

template <typename Function>
void CheckInvalidCircleOperation(tiny2d::Circle circle, Function function) {
  const tiny2d::Circle original = circle;
  CheckInvalidArgument([&] { function(circle); });
  CHECK(SameCircle(circle, original));
}

void TestRotatedVerticesKeepRectangleSize() {
  const tiny2d::Rectangle rectangle{1.0f, {100.0f, 80.0f}, {},   0.7f,
                                    0.0f, 80.0f,           20.0f};
  const auto vertices = tiny2d::GetVertices(rectangle);
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const std::size_t next = (i + 1) % vertices.size();
    const float expected_length =
        i % 2 == 0 ? rectangle.width : rectangle.height;
    CHECK(std::abs(Distance(vertices[i], vertices[next]) - expected_length) <
          kTolerance);
  }
}

void TestSatDetectsRotatedCollision() {
  const tiny2d::Rectangle rectangle_a{
      1.0f, {100.0f, 100.0f}, {}, 0.4f, 0.0f, 60.0f, 20.0f};
  const tiny2d::Rectangle rectangle_b{
      1.0f, {130.0f, 100.0f}, {}, -0.2f, 0.0f, 40.0f, 30.0f};
  const tiny2d::Rectangle separated{
      1.0f, {180.0f, 100.0f}, {}, -0.2f, 0.0f, 40.0f, 30.0f};
  CHECK(tiny2d::IsColliding(rectangle_a, rectangle_b));
  CHECK(!tiny2d::IsColliding(rectangle_a, separated));
}

void TestCenteredCollisionDoesNotCreateRotation() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {1.0f, {139.0f, 100.0f}, {-10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  CHECK(std::abs(squares[0].angular_velocity) < kTolerance);
  CHECK(std::abs(squares[1].angular_velocity) < kTolerance);
}

void TestOffCenterCollisionCreatesRotation() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 60.0f, 20.0f},
      {1.0f, {159.0f, 107.0f}, {-10.0f, 0.0f}, 0.0f, 0.0f, 60.0f, 20.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  CHECK(std::abs(squares[0].angular_velocity) > kTolerance);
  CHECK(std::abs(squares[1].angular_velocity) > kTolerance);
}

void TestAngularDampingReducesRotation() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {}, 0.0f, 10.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.5f, 1000.0f, 1000.0f, 1.0f);
  CHECK(NearlyEqual(squares[0].angular_velocity, 10.0f * std::exp(-0.35f)));
}

void TestMomentOfInertiaAndCenteredForce() {
  tiny2d::Rectangle rectangle{2.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 4.0f,
                              2.0f};
  CHECK(NearlyEqual(tiny2d::GetMomentOfInertia(rectangle), 10.0f / 3.0f));

  tiny2d::AddForceAtPoint(rectangle, {4.0f, -6.0f}, rectangle.position);
  CHECK(rectangle.applied_force.x == 4.0f);
  CHECK(rectangle.applied_force.y == -6.0f);
  CHECK(rectangle.applied_torque == 0.0f);
  const tiny2d::Vec2 field_acceleration =
      tiny2d::GetLinearAcceleration(rectangle, {}, 0.0f);
  CHECK(field_acceleration.x == 0.0f);
  CHECK(field_acceleration.y == 0.0f);

  std::vector<tiny2d::Rectangle> rectangles = {rectangle};
  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(NearlyEqual(rectangles[0].velocity.x, 1.0f));
  CHECK(NearlyEqual(rectangles[0].velocity.y, -1.5f));
  CHECK(rectangles[0].angular_velocity == 0.0f);
  CHECK(rectangles[0].applied_force.x == 0.0f);
  CHECK(rectangles[0].applied_force.y == 0.0f);
  CHECK(rectangles[0].applied_torque == 0.0f);
}

void TestOffCenterForceCreatesPositiveClockwiseTorque() {
  tiny2d::Rectangle rectangle{2.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 4.0f,
                              2.0f};
  rectangle.angular_damping_rate = 0.0f;
  tiny2d::AddForceAtPoint(rectangle, {0.0f, 3.0f}, {102.0f, 100.0f});
  CHECK(NearlyEqual(rectangle.applied_torque, 6.0f));

  std::vector<tiny2d::Rectangle> rectangles = {rectangle};
  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(NearlyEqual(rectangles[0].angular_velocity, 0.9f));
  CHECK(NearlyEqual(rectangles[0].angle, 0.45f));
  CHECK(rectangles[0].applied_torque == 0.0f);
}

void TestPerBodyExponentialDamping() {
  tiny2d::Rectangle rectangle{
      1.0f, {100.0f, 100.0f}, {10.0f, -4.0f}, 0.0f, 6.0f, 20.0f, 10.0f};
  rectangle.linear_damping_rate = 2.0f;
  rectangle.angular_damping_rate = 3.0f;
  tiny2d::AddForceAtPoint(rectangle, {2.0f, 0.0f}, rectangle.position);
  tiny2d::AddTorque(rectangle, 2.0f * tiny2d::GetMomentOfInertia(rectangle));
  std::vector<tiny2d::Rectangle> rectangles = {rectangle};

  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(NearlyEqual(rectangles[0].velocity.x, 11.0f * std::exp(-1.0f)));
  CHECK(NearlyEqual(rectangles[0].velocity.y, -4.0f * std::exp(-1.0f)));
  CHECK(NearlyEqual(rectangles[0].angular_velocity, 7.0f * std::exp(-1.5f)));
}

void TestZeroTimeUpdateClearsLoadsWithoutIntegratingThem() {
  tiny2d::Rectangle rectangle{
      1.0f, {100.0f, 100.0f}, {3.0f, -2.0f}, 0.25f, 1.5f, 20.0f, 10.0f};
  const float maximum = std::numeric_limits<float>::max();
  tiny2d::AddForceAtPoint(rectangle, {maximum, -maximum}, rectangle.position);
  tiny2d::AddTorque(rectangle, maximum);
  const tiny2d::Vec2 velocity = rectangle.velocity;
  const float angle = rectangle.angle;
  const float angular_velocity = rectangle.angular_velocity;
  std::vector<tiny2d::Rectangle> rectangles = {rectangle};

  tiny2d::Update(rectangles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(rectangles[0].velocity.x == velocity.x);
  CHECK(rectangles[0].velocity.y == velocity.y);
  CHECK(rectangles[0].angle == angle);
  CHECK(rectangles[0].angular_velocity == angular_velocity);
  CHECK(rectangles[0].applied_force.x == 0.0f);
  CHECK(rectangles[0].applied_force.y == 0.0f);
  CHECK(rectangles[0].applied_torque == 0.0f);
}

void TestStaticAndFixedRotationLoadBehavior() {
  tiny2d::Rectangle static_rectangle{
      0.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 10.0f};
  tiny2d::AddForceAtPoint(static_rectangle, {4.0f, 6.0f}, {105.0f, 100.0f});
  tiny2d::AddTorque(static_rectangle, 7.0f);
  CHECK(static_rectangle.applied_force.x == 0.0f);
  CHECK(static_rectangle.applied_force.y == 0.0f);
  CHECK(static_rectangle.applied_torque == 0.0f);
  CHECK(tiny2d::GetMomentOfInertia(static_rectangle) == 0.0f);

  tiny2d::Rectangle fixed_rectangle{
      2.0f, {200.0f, 200.0f}, {}, 0.3f, 5.0f, 20.0f, 10.0f, true};
  tiny2d::AddForceAtPoint(fixed_rectangle, {4.0f, 6.0f}, {210.0f, 200.0f});
  tiny2d::AddTorque(fixed_rectangle, 7.0f);
  CHECK(fixed_rectangle.applied_force.x == 4.0f);
  CHECK(fixed_rectangle.applied_force.y == 6.0f);
  CHECK(fixed_rectangle.applied_torque == 0.0f);
  CHECK(tiny2d::GetMomentOfInertia(fixed_rectangle) > 0.0f);

  std::vector<tiny2d::Rectangle> rectangles = {fixed_rectangle};
  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(NearlyEqual(rectangles[0].velocity.x, 1.0f));
  CHECK(NearlyEqual(rectangles[0].velocity.y, 1.5f));
  CHECK(rectangles[0].angle == fixed_rectangle.angle);
  CHECK(rectangles[0].angular_velocity == 0.0f);
  CHECK(rectangles[0].applied_force.x == 0.0f);
  CHECK(rectangles[0].applied_force.y == 0.0f);
  CHECK(rectangles[0].applied_torque == 0.0f);
}

void TestElectricFieldUsesChargeAndMass() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {2.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, 3.0f},
      {4.0f, {200.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, -3.0f},
      {0.0f, {300.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, 3.0f},
      {2.0f, {400.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f},
  };

  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {4.0f, -8.0f});

  CHECK(std::abs(rectangles[0].velocity.x - 3.0f) < kTolerance);
  CHECK(std::abs(rectangles[0].velocity.y - 43.05f) < kTolerance);
  CHECK(std::abs(rectangles[1].velocity.x + 1.5f) < kTolerance);
  CHECK(std::abs(rectangles[1].velocity.y - 52.05f) < kTolerance);
  CHECK(std::abs(rectangles[2].velocity.x) < kTolerance);
  CHECK(std::abs(rectangles[2].velocity.y) < kTolerance);
  CHECK(std::abs(rectangles[3].velocity.x) < kTolerance);
  CHECK(std::abs(rectangles[3].velocity.y - 49.05f) < kTolerance);
}

void TestConfigurableGravity() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f},
  };
  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 20.0f);
  CHECK(std::abs(rectangles[0].velocity.y - 10.0f) < kTolerance);
}

void TestConfigurableWallFriction() {
  std::vector<tiny2d::Rectangle> no_friction = {
      {1.0f, {100.0f, 19.0f}, {10.0f, -10.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  std::vector<tiny2d::Rectangle> with_friction = no_friction;
  tiny2d::Update(no_friction, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.0f);
  tiny2d::Update(with_friction, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.4f);
  CHECK(std::abs(no_friction[0].velocity.x - 10.0f) < kTolerance);
  CHECK(std::abs(with_friction[0].velocity.x) <
        std::abs(no_friction[0].velocity.x));
}

void TestStaticRectangleDoesNotMove() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {0.0f, {139.0f, 115.0f}, {}, -0.35f, 0.0f, 40.0f, 40.0f},
  };
  const tiny2d::Rectangle fixed_square = squares[1];
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  CHECK(squares[1].position.x == fixed_square.position.x);
  CHECK(squares[1].position.y == fixed_square.position.y);
  CHECK(squares[1].angle == fixed_square.angle);
}

void TestRestitutionRequiresImpactSpeed() {
  std::vector<tiny2d::Rectangle> slow_collision = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {0.0f, {139.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(slow_collision, 0.0f, 1000.0f, 1000.0f, 1.0f);
  CHECK(std::abs(slow_collision[0].velocity.x) < kTolerance);

  std::vector<tiny2d::Rectangle> fast_collision = {
      {1.0f, {100.0f, 100.0f}, {100.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {0.0f, {139.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(fast_collision, 0.0f, 1000.0f, 1000.0f, 1.0f);
  CHECK(fast_collision[0].velocity.x < -90.0f);
  CHECK(std::abs(fast_collision[0].angular_velocity) < kTolerance);
}

void TestFixedRotationStaysLocked() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {100.0f, 0.0f}, 0.0f, 5.0f, 40.0f, 40.0f, true},
      {1.0f, {139.0f, 100.0f}, {}, 0.0f, -5.0f, 40.0f, 40.0f, true},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  CHECK(squares[0].angle == 0.0f);
  CHECK(squares[1].angle == 0.0f);
  CHECK(squares[0].angular_velocity == 0.0f);
  CHECK(squares[1].angular_velocity == 0.0f);
  CHECK(squares[1].velocity.x > 90.0f);
}

void TestLockedBlocksCollideOnRamp() {
  constexpr float kRampAngle = -0.5235987756f;
  constexpr float kPhysicsStep = 1.0f / 120.0f;
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f,
       {710.0f, 397.9274f},
       {-190.5256f, 110.0f},
       kRampAngle,
       0.0f,
       40.0f,
       40.0f,
       true},
      {1.0f, {470.0f, 536.4915f}, {}, kRampAngle, 0.0f, 40.0f, 40.0f, true},
      {0.0f, {750.0f, 744.3376f}, {}, kRampAngle, 0.0f, 600.0f, 600.0f, true},
  };

  bool collision_observed = false;
  for (int step = 0; step < 180; ++step) {
    tiny2d::Update(squares, kPhysicsStep, 800.0f, 600.0f, 0.95f);
    const float lower_block_downhill_speed =
        -0.8660254f * squares[1].velocity.x + 0.5f * squares[1].velocity.y;
    collision_observed |= lower_block_downhill_speed > 100.0f;
  }

  CHECK(collision_observed);
  CHECK(squares[0].angular_velocity == 0.0f);
  CHECK(squares[1].angular_velocity == 0.0f);
}

void TestLegalBoundaryValues() {
  std::vector<tiny2d::Rectangle> empty_world;
  tiny2d::Update(empty_world, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(empty_world.empty());

  std::vector<tiny2d::Rectangle> static_rectangle = {
      {0.0f, {50.0f, 50.0f}, {}, 0.3f, 0.0f, 1.0f, 1.0f, true, 4.0f},
  };
  const tiny2d::Rectangle original = static_rectangle.front();
  tiny2d::Update(static_rectangle, 10.0f, 100.0f, 100.0f, 1.0f, 1.0f,
                 {50.0f, -50.0f}, -10.0f);
  CHECK(SameRectangle(static_rectangle.front(), original));
}

void TestCollisionDetectionIsSymmetric() {
  const std::array<tiny2d::Rectangle, 4> rectangles = {
      tiny2d::Rectangle{1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f, 40.0f},
      tiny2d::Rectangle{1.0f, {130.0f, 100.0f}, {}, 0.4f, 0.0f, 60.0f, 20.0f},
      tiny2d::Rectangle{1.0f, {220.0f, 100.0f}, {}, -0.7f, 0.0f, 30.0f, 50.0f},
      tiny2d::Rectangle{1.0f, {105.0f, 125.0f}, {}, 1.1f, 0.0f, 15.0f, 70.0f},
  };

  CHECK(tiny2d::IsColliding(rectangles[0], rectangles[1]));
  CHECK(!tiny2d::IsColliding(rectangles[0], rectangles[2]));
  for (std::size_t i = 0; i < rectangles.size(); ++i) {
    for (std::size_t j = i + 1; j < rectangles.size(); ++j) {
      CHECK(tiny2d::IsColliding(rectangles[i], rectangles[j]) ==
            tiny2d::IsColliding(rectangles[j], rectangles[i]));
    }
  }
}

void TestElasticCollisionConservesMomentumAndEnergy() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {2.0f, {100.0f, 500.0f}, {40.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
      {1.0f, {139.0f, 500.0f}, {-20.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
  };
  const float initial_momentum = TotalMomentumX(rectangles);
  const float initial_energy = TranslationalKineticEnergy(rectangles);

  tiny2d::Update(rectangles, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.0f, {}, 0.0f);

  CHECK(NearlyEqual(TotalMomentumX(rectangles), initial_momentum));
  CHECK(NearlyEqual(TranslationalKineticEnergy(rectangles), initial_energy));
  CHECK(NearlyEqual(rectangles[0].velocity.x, 0.0f));
  CHECK(NearlyEqual(rectangles[1].velocity.x, 60.0f));
}

void TestInelasticCollisionDoesNotGainEnergy() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {2.0f, {100.0f, 500.0f}, {40.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
      {1.0f, {139.0f, 500.0f}, {-20.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
  };
  const float initial_momentum = TotalMomentumX(rectangles);
  const float initial_energy = TranslationalKineticEnergy(rectangles);

  tiny2d::Update(rectangles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);

  CHECK(NearlyEqual(TotalMomentumX(rectangles), initial_momentum));
  CHECK(TranslationalKineticEnergy(rectangles) < initial_energy);
  CHECK(NearlyEqual(rectangles[0].velocity.x, 20.0f));
  CHECK(NearlyEqual(rectangles[1].velocity.x, 20.0f));
}

void TestAllWindowWallsReflect() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {1.0f, {19.0f, 100.0f}, {-100.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
      {1.0f, {981.0f, 200.0f}, {100.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
      {1.0f, {300.0f, 19.0f}, {0.0f, -100.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
      {1.0f, {400.0f, 981.0f}, {0.0f, 100.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
  };

  tiny2d::Update(rectangles, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.0f, {}, 0.0f);

  CHECK(rectangles[0].velocity.x > 90.0f);
  CHECK(rectangles[1].velocity.x < -90.0f);
  CHECK(rectangles[2].velocity.y > 90.0f);
  CHECK(rectangles[3].velocity.y < -90.0f);
  for (const tiny2d::Rectangle& rectangle : rectangles) {
    CheckInsideArea(rectangle, 1000.0f, 1000.0f);
  }
}

void TestCornerAndRotatedWallContactsStayInside() {
  std::vector<tiny2d::Rectangle> corner = {
      {1.0f,
       {19.0f, 19.0f},
       {-100.0f, -100.0f},
       0.0f,
       0.0f,
       40.0f,
       40.0f,
       true},
  };
  tiny2d::Update(corner, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.0f, {}, 0.0f);
  CHECK(corner[0].velocity.x > 90.0f);
  CHECK(corner[0].velocity.y > 90.0f);
  CheckInsideArea(corner[0], 1000.0f, 1000.0f);

  std::vector<tiny2d::Rectangle> rotated = {
      {1.0f, {10.0f, 500.0f}, {}, 0.7853982f, 0.0f, 40.0f, 20.0f},
  };
  tiny2d::Update(rotated, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CheckInsideArea(rotated[0], 1000.0f, 1000.0f);
}

void TestElectricFieldAndGravityCanCancel() {
  const tiny2d::Rectangle rectangle{
      2.0f, {500.0f, 500.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, 4.0f};
  const tiny2d::Vec2 acceleration =
      tiny2d::GetLinearAcceleration(rectangle, {0.0f, -5.0f}, 10.0f);
  CHECK(NearlyEqual(acceleration.x, 0.0f));
  CHECK(NearlyEqual(acceleration.y, 0.0f));

  std::vector<tiny2d::Rectangle> rectangles = {rectangle};
  tiny2d::Update(rectangles, 3.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {0.0f, -5.0f},
                 10.0f);
  CHECK(NearlyEqual(rectangles[0].position.x, rectangle.position.x));
  CHECK(NearlyEqual(rectangles[0].position.y, rectangle.position.y));
  CHECK(NearlyEqual(rectangles[0].velocity.x, 0.0f));
  CHECK(NearlyEqual(rectangles[0].velocity.y, 0.0f));
}

void TestPositiveAndNegativeChargesAccelerateOppositely() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {2.0f, {200.0f, 500.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, 2.0f},
      {2.0f, {800.0f, 500.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, -2.0f},
  };
  tiny2d::Update(rectangles, 0.25f, 1000.0f, 1000.0f, 0.0f, 0.0f, {6.0f, -4.0f},
                 0.0f);
  CHECK(NearlyEqual(rectangles[0].velocity.x, -rectangles[1].velocity.x));
  CHECK(NearlyEqual(rectangles[0].velocity.y, -rectangles[1].velocity.y));
}

void TestLongSimulationStaysFiniteAndInside() {
  constexpr float kAreaSize = 500.0f;
  constexpr float kStep = 1.0f / 120.0f;
  std::vector<tiny2d::Rectangle> rectangles = {
      {1.5f,
       {250.0f, 100.0f},
       {100.0f, -50.0f},
       0.3f,
       2.0f,
       30.0f,
       50.0f,
       false,
       2.0f},
  };

  for (int step = 0; step < 10000; ++step) {
    tiny2d::Update(rectangles, kStep, kAreaSize, kAreaSize, 0.7f, 0.2f,
                   {2.0f, -1.0f}, 98.1f);
    CHECK(IsFinite(rectangles[0]));
    CheckInsideArea(rectangles[0], kAreaSize, kAreaSize);
  }
}

void TestFixedSeedProperties() {
  constexpr float kAreaSize = 1000.0f;
  constexpr float kStep = 1.0f / 240.0f;
  std::mt19937 random_engine(0x2D2026u);
  std::uniform_real_distribution<float> position(200.0f, 800.0f);
  std::uniform_real_distribution<float> velocity(-200.0f, 200.0f);
  std::uniform_real_distribution<float> angle(-3.0f, 3.0f);
  std::uniform_real_distribution<float> angular_velocity(-5.0f, 5.0f);
  std::uniform_real_distribution<float> size(10.0f, 60.0f);
  std::uniform_real_distribution<float> mass(0.25f, 10.0f);
  std::uniform_real_distribution<float> charge(-5.0f, 5.0f);

  for (int sample = 0; sample < 64; ++sample) {
    const auto make_rectangle = [&] {
      return tiny2d::Rectangle{
          mass(random_engine),
          {position(random_engine), position(random_engine)},
          {velocity(random_engine), velocity(random_engine)},
          angle(random_engine),
          angular_velocity(random_engine),
          size(random_engine),
          size(random_engine),
          false,
          charge(random_engine)};
    };

    const tiny2d::Rectangle rectangle_a = make_rectangle();
    const tiny2d::Rectangle rectangle_b = make_rectangle();
    CHECK(tiny2d::IsColliding(rectangle_a, rectangle_b) ==
          tiny2d::IsColliding(rectangle_b, rectangle_a));

    const auto vertices = tiny2d::GetVertices(rectangle_a);
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      const float expected_length =
          i % 2 == 0 ? rectangle_a.width : rectangle_a.height;
      CHECK(NearlyEqual(Distance(vertices[i], vertices[(i + 1) % 4]),
                        expected_length));
    }

    std::vector<tiny2d::Rectangle> world = {rectangle_a};
    for (int step = 0; step < 240; ++step) {
      tiny2d::Update(world, kStep, kAreaSize, kAreaSize, 0.6f, 0.2f,
                     {3.0f, -2.0f}, 50.0f);
      CHECK(IsFinite(world[0]));
      CheckInsideArea(world[0], kAreaSize, kAreaSize);
    }
  }
}

void TestRejectsNonFiniteUpdateParameters() {
  const std::vector<tiny2d::Rectangle> rectangles = {
      {1.0f, {500.0f, 500.0f}, {}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  const std::array<float, 3> non_finite = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };

  for (const float value : non_finite) {
    CheckInvalidUpdate(rectangles, value);
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, value);
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, value);
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, value);
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, 1.0f,
                       value);
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, 1.0f, 0.0f,
                       {value, 0.0f});
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, 1.0f, 0.0f,
                       {0.0f, value});
    CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, 1.0f, 0.0f,
                       {}, value);
  }

  CheckInvalidUpdate(rectangles, -1.0f / 120.0f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 0.0f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, -1000.0f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 0.0f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, -1000.0f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, -0.01f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, 1.01f);
  CheckInvalidUpdate(rectangles, 1.0f / 120.0f, 1000.0f, 1000.0f, 1.0f, -0.01f);
}

void TestRejectsInvalidRectangles() {
  const tiny2d::Rectangle valid{1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f,
                                40.0f};
  const tiny2d::Rectangle base{1.0f, {300.0f, 300.0f}, {}, 0.0f, 0.0f, 40.0f,
                               40.0f};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const auto check = [&](tiny2d::Rectangle invalid) {
    CheckInvalidUpdate({valid, invalid});
  };

  tiny2d::Rectangle invalid = base;
  invalid.mass = -1.0f;
  check(invalid);
  invalid = base;
  invalid.mass = nan;
  check(invalid);
  invalid = base;
  invalid.mass = infinity;
  check(invalid);
  invalid = base;
  invalid.mass = std::numeric_limits<float>::denorm_min();
  check(invalid);

  for (const float invalid_size : {0.0f, -1.0f, nan, infinity}) {
    invalid = base;
    invalid.width = invalid_size;
    check(invalid);
    invalid = base;
    invalid.height = invalid_size;
    check(invalid);
  }

  invalid = base;
  invalid.position.x = nan;
  check(invalid);
  invalid = base;
  invalid.position.y = infinity;
  check(invalid);
  invalid = base;
  invalid.velocity.x = nan;
  check(invalid);
  invalid = base;
  invalid.velocity.y = infinity;
  check(invalid);
  invalid = base;
  invalid.angle = nan;
  check(invalid);
  invalid = base;
  invalid.angular_velocity = infinity;
  check(invalid);
  invalid = base;
  invalid.charge = nan;
  check(invalid);

  invalid = base;
  invalid.mass = 0.0f;
  invalid.velocity.x = 1.0f;
  check(invalid);
  invalid = base;
  invalid.mass = 0.0f;
  invalid.angular_velocity = 1.0f;
  check(invalid);
}

void TestRejectsInvalidLoadsAndDampingAtomically() {
  const tiny2d::Rectangle base{1.0f, {300.0f, 300.0f}, {}, 0.0f, 0.0f, 40.0f,
                               40.0f};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const float maximum = std::numeric_limits<float>::max();

  CheckInvalidRectangleOperation(base, [&](tiny2d::Rectangle& rectangle) {
    tiny2d::AddForceAtPoint(rectangle, {nan, 0.0f}, rectangle.position);
  });
  CheckInvalidRectangleOperation(base, [&](tiny2d::Rectangle& rectangle) {
    tiny2d::AddForceAtPoint(rectangle, {}, {infinity, 0.0f});
  });
  CheckInvalidRectangleOperation(base, [&](tiny2d::Rectangle& rectangle) {
    tiny2d::AddTorque(rectangle, nan);
  });

  tiny2d::Rectangle force_overflow = base;
  force_overflow.applied_force.x = maximum;
  CheckInvalidRectangleOperation(
      force_overflow, [&](tiny2d::Rectangle& rectangle) {
        tiny2d::AddForceAtPoint(rectangle, {maximum, 0.0f}, rectangle.position);
      });

  tiny2d::Rectangle torque_overflow = base;
  torque_overflow.applied_torque = maximum;
  CheckInvalidRectangleOperation(torque_overflow,
                                 [&](tiny2d::Rectangle& rectangle) {
                                   tiny2d::AddTorque(rectangle, maximum);
                                 });
  CheckInvalidRectangleOperation(base, [&](tiny2d::Rectangle& rectangle) {
    tiny2d::AddForceAtPoint(rectangle, {0.0f, maximum}, {maximum, 0.0f});
  });

  for (const float value : {nan, infinity}) {
    tiny2d::Rectangle invalid = base;
    invalid.applied_force.x = value;
    CheckInvalidUpdate({invalid});
    invalid = base;
    invalid.applied_torque = value;
    CheckInvalidUpdate({invalid});
    invalid = base;
    invalid.linear_damping_rate = value;
    CheckInvalidUpdate({invalid});
    invalid = base;
    invalid.angular_damping_rate = value;
    CheckInvalidUpdate({invalid});
  }

  tiny2d::Rectangle invalid = base;
  invalid.linear_damping_rate = -0.1f;
  CheckInvalidUpdate({invalid});
  invalid = base;
  invalid.angular_damping_rate = -0.1f;
  CheckInvalidUpdate({invalid});

  tiny2d::Rectangle force_integration_overflow = base;
  force_integration_overflow.mass = 0.000001f;
  force_integration_overflow.applied_force.x = maximum;
  CheckInvalidUpdate({force_integration_overflow});

  tiny2d::Rectangle torque_integration_overflow = base;
  torque_integration_overflow.mass = 0.000001f;
  torque_integration_overflow.applied_torque = maximum;
  CheckInvalidUpdate({torque_integration_overflow});

  tiny2d::Rectangle first_rectangle = base;
  first_rectangle.applied_force = {1.0f, 2.0f};
  CheckInvalidUpdate({first_rectangle, force_integration_overflow});
}

void TestRejectsImpossibleAndOverflowingStates() {
  tiny2d::Rectangle rectangle{1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f,
                              40.0f};
  CheckInvalidUpdate({rectangle}, 1.0f / 120.0f, 20.0f, 1000.0f);
  CheckInvalidUpdate({rectangle}, 1.0f / 120.0f, 1000.0f, 20.0f);

  rectangle.velocity.x = std::numeric_limits<float>::max();
  CheckInvalidUpdate({rectangle}, 2.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);

  rectangle = {1.0f,
               {100.0f, 100.0f},
               {},
               0.0f,
               0.0f,
               40.0f,
               40.0f,
               false,
               std::numeric_limits<float>::max()};
  CheckInvalidUpdate({rectangle}, 1.0f, 1000.0f, 1000.0f, 0.0f, 0.0f,
                     {std::numeric_limits<float>::max(), 0.0f}, 0.0f);
}

void TestPublicFunctionsRejectInvalidRectangles() {
  const tiny2d::Rectangle valid{1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f,
                                40.0f};
  tiny2d::Rectangle invalid = valid;
  invalid.width = 0.0f;
  CheckInvalidArgument([&] { tiny2d::GetVertices(invalid); });
  CheckInvalidArgument([&] { tiny2d::IsColliding(valid, invalid); });

  invalid = valid;
  invalid.mass = -1.0f;
  CheckInvalidArgument([&] { tiny2d::GetMomentOfInertia(invalid); });
  CheckInvalidArgument(
      [&] { tiny2d::GetLinearAcceleration(invalid, {}, 98.1f); });
  CheckInvalidArgument([&] {
    tiny2d::GetLinearAcceleration(
        valid, {std::numeric_limits<float>::quiet_NaN(), 0.0f}, 98.1f);
  });
}

void TestCircleInertiaLoadsAndFields() {
  tiny2d::Circle circle;
  circle.mass = 2.0f;
  circle.position = {100.0f, 100.0f};
  circle.radius = 3.0f;
  circle.angular_damping_rate = 0.0f;
  circle.charge = 2.0f;
  CHECK(NearlyEqual(tiny2d::GetMomentOfInertia(circle), 9.0f));
  circle.inertia_model = tiny2d::CircleInertiaModel::kHoop;
  CHECK(NearlyEqual(tiny2d::GetMomentOfInertia(circle), 18.0f));
  circle.inertia_model = tiny2d::CircleInertiaModel::kSolidDisk;

  tiny2d::AddForceAtPoint(circle, {0.0f, 4.0f}, {102.0f, 100.0f});
  tiny2d::AddTorque(circle, 1.0f);
  CHECK(NearlyEqual(circle.applied_force.y, 4.0f));
  CHECK(NearlyEqual(circle.applied_torque, 9.0f));
  const tiny2d::Vec2 acceleration =
      tiny2d::GetLinearAcceleration(circle, {3.0f, -1.0f}, 1.0f);
  CHECK(NearlyEqual(acceleration.x, 3.0f));
  CHECK(NearlyEqual(acceleration.y, 0.0f));

  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> circles{circle};
  tiny2d::Update(rectangles, circles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  CHECK(NearlyEqual(circles[0].velocity.y, 1.0f));
  CHECK(NearlyEqual(circles[0].position.y, 100.5f));
  CHECK(NearlyEqual(circles[0].angular_velocity, 0.5f));
  CHECK(NearlyEqual(circles[0].angle, 0.25f));
  CHECK(circles[0].applied_force.x == 0.0f);
  CHECK(circles[0].applied_force.y == 0.0f);
  CHECK(circles[0].applied_torque == 0.0f);

  tiny2d::Circle static_circle = circle;
  static_circle.mass = 0.0f;
  static_circle.velocity = {};
  static_circle.angular_velocity = 0.0f;
  static_circle.applied_force = {};
  static_circle.applied_torque = 0.0f;
  CHECK(tiny2d::GetMomentOfInertia(static_circle) == 0.0f);
  tiny2d::AddForceAtPoint(static_circle, {10.0f, 20.0f},
                          static_circle.position);
  tiny2d::AddTorque(static_circle, 30.0f);
  CHECK(static_circle.applied_force.x == 0.0f);
  CHECK(static_circle.applied_force.y == 0.0f);
  CHECK(static_circle.applied_torque == 0.0f);
}

void TestCircleRejectsEveryNonFinitePublicStateField() {
  using Mutator = void (*)(tiny2d::Circle*, float);
  const std::array<Mutator, 17> mutators = {
      [](tiny2d::Circle* circle, float value) { circle->mass = value; },
      [](tiny2d::Circle* circle, float value) { circle->position.x = value; },
      [](tiny2d::Circle* circle, float value) { circle->position.y = value; },
      [](tiny2d::Circle* circle, float value) { circle->velocity.x = value; },
      [](tiny2d::Circle* circle, float value) { circle->velocity.y = value; },
      [](tiny2d::Circle* circle, float value) { circle->angle = value; },
      [](tiny2d::Circle* circle, float value) {
        circle->angular_velocity = value;
      },
      [](tiny2d::Circle* circle, float value) { circle->radius = value; },
      [](tiny2d::Circle* circle, float value) { circle->charge = value; },
      [](tiny2d::Circle* circle, float value) {
        circle->applied_force.x = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->applied_force.y = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->applied_torque = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->linear_damping_rate = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->angular_damping_rate = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->material.restitution = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->material.static_friction = value;
      },
      [](tiny2d::Circle* circle, float value) {
        circle->material.kinetic_friction = value;
      },
  };
  const std::array invalid_values = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(),
  };

  tiny2d::Circle valid;
  valid.position = {100.0f, 100.0f};
  valid.radius = 10.0f;
  valid.material = {0.5f, 0.4f, 0.2f};
  for (float invalid_value : invalid_values) {
    for (const Mutator mutate : mutators) {
      tiny2d::Circle invalid = valid;
      mutate(&invalid, invalid_value);
      CheckInvalidArgument(
          [&] { static_cast<void>(tiny2d::GetMomentOfInertia(invalid)); });
      CheckInvalidMixedUpdate({}, {invalid});
    }
  }
}

void TestCircleCollisionGeometryAndSymmetry() {
  const tiny2d::Circle circle_a{1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 10.0f};
  tiny2d::Circle circle_b = circle_a;
  circle_b.position = {119.0f, 100.0f};
  CHECK(tiny2d::IsColliding(circle_a, circle_b));
  CHECK(tiny2d::IsColliding(circle_b, circle_a));
  circle_b.position.x = 120.0f;
  CHECK(!tiny2d::IsColliding(circle_a, circle_b));

  const tiny2d::Rectangle rectangle{
      1.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f, 20.0f};
  circle_b.position = {124.0f, 100.0f};
  circle_b.radius = 5.0f;
  CHECK(tiny2d::IsColliding(rectangle, circle_b));
  CHECK(tiny2d::IsColliding(circle_b, rectangle));
  circle_b.position.x = 125.0f;
  CHECK(!tiny2d::IsColliding(rectangle, circle_b));
  CHECK(!tiny2d::IsColliding(circle_b, rectangle));
  circle_b.position = rectangle.position;
  CHECK(tiny2d::IsColliding(rectangle, circle_b));

  tiny2d::Rectangle rotated = rectangle;
  rotated.angle = 0.7853982f;
  circle_b.radius = 3.0f;
  circle_b.position = {115.55635f, 115.55635f};
  CHECK(tiny2d::IsColliding(rotated, circle_b));
  CHECK(tiny2d::IsColliding(circle_b, rotated));

  tiny2d::Circle invalid = circle_a;
  invalid.radius = 0.0f;
  CheckInvalidArgument([&] { tiny2d::IsColliding(invalid, circle_b); });
  CheckInvalidArgument([&] { tiny2d::IsColliding(rectangle, invalid); });
}

void TestSupportedCircleSweepDoesNotTunnel() {
  tiny2d::Circle circle_a;
  circle_a.position = {10.0f, 7.0f};
  circle_a.velocity = {10.0f, 0.0f};
  circle_a.radius = 0.1f;
  circle_a.fixed_rotation = true;
  circle_a.material = {1.0f, 0.0f, 0.0f};
  tiny2d::Circle circle_b = circle_a;
  circle_b.position = {10.0205f, 7.199f};
  circle_b.velocity = {-10.0f, 0.0f};

  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> discrete{circle_a, circle_b};
  tiny2d::Update(rectangles, discrete, 1.0f / 480.0f, 24.0f, 14.0f, 1.0f, 0.0f,
                 {}, 0.0f, 0.01f);
  CHECK(discrete[0].velocity.x == circle_a.velocity.x);
  CHECK(discrete[1].velocity.x == circle_b.velocity.x);

  std::vector<tiny2d::Circle> continuous{circle_a, circle_b};
  tiny2d::Update(rectangles, continuous, 1.0f / 480.0f, 24.0f, 14.0f, 1.0f,
                 0.0f, {}, 0.0f, 0.01f, true);

  CHECK(continuous[0].velocity.x < circle_a.velocity.x);
  CHECK(continuous[1].velocity.x > circle_b.velocity.x);
}

void TestFastCircleTravelsBeyondDiameterWithoutTunneling() {
  tiny2d::Circle moving;
  moving.position = {10.0f, 7.0f};
  moving.velocity = {1000.0f, 0.0f};
  moving.radius = 1.0f;
  moving.fixed_rotation = true;
  moving.material = {1.0f, 0.0f, 0.0f};
  tiny2d::Circle target = moving;
  target.mass = 0.0f;
  target.position = {16.0f, 7.0f};
  target.velocity = {};

  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> circles{moving, target};
  tiny2d::Update(rectangles, circles, 0.01f, 100.0f, 20.0f, 1.0f, 0.0f, {},
                 0.0f, 0.01f, true);

  CHECK(NearlyEqual(circles[0].position.x, 8.0f));
  CHECK(NearlyEqual(circles[0].velocity.x, -1000.0f));
  CHECK(SameCircle(circles[1], target));
}

void TestLowSpeedCircleCcdMatchesDiscreteUpdate() {
  tiny2d::Circle circle_a;
  circle_a.position = {100.0f, 100.0f};
  circle_a.velocity = {5.0f, 0.0f};
  circle_a.radius = 10.0f;
  circle_a.fixed_rotation = true;
  circle_a.material = {0.8f, 0.0f, 0.0f};
  tiny2d::Circle circle_b = circle_a;
  circle_b.position = {121.0f, 100.0f};
  circle_b.velocity = {-5.0f, 0.0f};

  std::vector<tiny2d::Rectangle> discrete_rectangles;
  std::vector<tiny2d::Circle> discrete{circle_a, circle_b};
  std::vector<tiny2d::Rectangle> continuous_rectangles;
  std::vector<tiny2d::Circle> continuous = discrete;
  tiny2d::Update(discrete_rectangles, discrete, 0.2f, 1000.0f, 1000.0f, 0.0f,
                 0.0f, {}, 0.0f, 0.01f, false);
  tiny2d::Update(continuous_rectangles, continuous, 0.2f, 1000.0f, 1000.0f,
                 0.0f, 0.0f, {}, 0.0f, 0.01f, true);

  CHECK(SameCircle(discrete[0], continuous[0]));
  CHECK(SameCircle(discrete[1], continuous[1]));
}

void TestCircleCcdMultipleImpactsAreDeterministic() {
  tiny2d::Circle circle;
  circle.position = {10.0f, 7.0f};
  circle.velocity = {1000.0f, 0.0f};
  circle.radius = 1.0f;
  circle.fixed_rotation = true;
  circle.material = {1.0f, 0.0f, 0.0f};
  std::vector<tiny2d::Circle> first(3, circle);
  first[1].position.x = 16.0f;
  first[1].velocity = {};
  first[2].position.x = 22.0f;
  first[2].velocity = {};
  std::vector<tiny2d::Circle> second = first;
  std::vector<tiny2d::Rectangle> first_rectangles;
  std::vector<tiny2d::Rectangle> second_rectangles;

  tiny2d::Update(first_rectangles, first, 0.012f, 100.0f, 20.0f, 1.0f, 0.0f, {},
                 0.0f, 0.01f, true);
  tiny2d::Update(second_rectangles, second, 0.012f, 100.0f, 20.0f, 1.0f, 0.0f,
                 {}, 0.0f, 0.01f, true);

  CHECK(NearlyEqual(first[0].velocity.x, 0.0f));
  CHECK(NearlyEqual(first[1].velocity.x, 0.0f));
  CHECK(NearlyEqual(first[2].velocity.x, 1000.0f));
  for (std::size_t i = 0; i < first.size(); ++i) {
    CHECK(SameCircle(first[i], second[i]));
  }
}

void TestCircleElasticCollisionAndMaterialMixing() {
  tiny2d::Circle circle_a;
  circle_a.position = {100.0f, 100.0f};
  circle_a.velocity = {5.0f, 0.0f};
  circle_a.radius = 10.0f;
  circle_a.fixed_rotation = true;
  circle_a.material = {1.0f, 0.0f, 0.0f};
  tiny2d::Circle circle_b = circle_a;
  circle_b.position = {119.0f, 100.0f};
  circle_b.velocity = {-5.0f, 0.0f};

  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> circles{circle_a, circle_b};
  const float initial_momentum = circles[0].mass * circles[0].velocity.x +
                                 circles[1].mass * circles[1].velocity.x;
  const float initial_energy =
      0.5f * circles[0].mass * circles[0].velocity.x * circles[0].velocity.x +
      0.5f * circles[1].mass * circles[1].velocity.x * circles[1].velocity.x;
  tiny2d::Update(rectangles, circles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  const float final_momentum = circles[0].mass * circles[0].velocity.x +
                               circles[1].mass * circles[1].velocity.x;
  const float final_energy =
      0.5f * circles[0].mass * circles[0].velocity.x * circles[0].velocity.x +
      0.5f * circles[1].mass * circles[1].velocity.x * circles[1].velocity.x;
  CHECK(NearlyEqual(final_momentum, initial_momentum));
  CHECK(NearlyEqual(final_energy, initial_energy));
  CHECK(NearlyEqual(circles[0].velocity.x, -5.0f));
  CHECK(NearlyEqual(circles[1].velocity.x, 5.0f));

  std::vector<tiny2d::Circle> swapped{circle_b, circle_a};
  tiny2d::Update(rectangles, swapped, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  CHECK(NearlyEqual(swapped[0].velocity.x, circles[1].velocity.x));
  CHECK(NearlyEqual(swapped[1].velocity.x, circles[0].velocity.x));

  circle_a.material = {0.2f, 0.0f, 0.0f};
  circle_b.material = {0.8f, 0.0f, 0.0f};
  circles = {circle_a, circle_b};
  tiny2d::Update(rectangles, circles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  CHECK(NearlyEqual(circles[0].velocity.x, -4.0f));
  CHECK(NearlyEqual(circles[1].velocity.x, 4.0f));

  circle_a.velocity = {10.0f, 0.0f};
  circle_a.material = {1.0f, 0.0f, 0.0f};
  circle_b.mass = 0.0f;
  circle_b.velocity = {};
  circle_b.material = {1.0f, 0.0f, 0.0f};
  circles = {circle_a, circle_b};
  const tiny2d::Circle original_static = circles[1];
  tiny2d::Update(rectangles, circles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  CHECK(NearlyEqual(circles[0].velocity.x, -10.0f));
  CHECK(SameCircle(circles[1], original_static));
}

void TestCircleRectangleResponseAndWallFriction() {
  tiny2d::Rectangle wall{0.0f, {120.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 100.0f};
  wall.material = {1.0f, 0.0f, 0.0f};
  tiny2d::Circle circle;
  circle.position = {104.0f, 100.0f};
  circle.velocity = {10.0f, 0.0f};
  circle.radius = 10.0f;
  circle.fixed_rotation = true;
  circle.material = {1.0f, 0.0f, 0.0f};
  std::vector<tiny2d::Rectangle> rectangles{wall};
  std::vector<tiny2d::Circle> circles{circle};
  tiny2d::Update(rectangles, circles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  CHECK(NearlyEqual(circles[0].velocity.x, -10.0f));
  CHECK(rectangles[0].position.x == wall.position.x);

  tiny2d::Rectangle floor{0.0f, {100.0f, 120.0f}, {},   0.0f,
                          0.0f, 200.0f,           40.0f};
  floor.material = {0.0f, 1.0f, 1.0f};
  tiny2d::Circle floor_circle;
  floor_circle.position = {100.0f, 105.0f};
  floor_circle.velocity = {3.0f, 10.0f};
  floor_circle.radius = 10.0f;
  floor_circle.angular_damping_rate = 0.0f;
  floor_circle.material = {0.0f, 1.0f, 1.0f};
  rectangles = {floor};
  circles = {floor_circle};
  tiny2d::Update(rectangles, circles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {},
                 0.0f, 0.01f);
  CHECK(circles[0].velocity.x < floor_circle.velocity.x);
  CHECK(circles[0].angular_velocity > 0.0f);

  const auto run_top_wall = [](float horizontal_speed) {
    std::vector<tiny2d::Rectangle> no_rectangles;
    tiny2d::Circle sliding;
    sliding.position = {100.0f, 9.0f};
    sliding.velocity = {horizontal_speed, -10.0f};
    sliding.radius = 10.0f;
    sliding.angular_damping_rate = 0.0f;
    sliding.material = {0.0f, 1.0f, 0.25f};
    std::vector<tiny2d::Circle> result{sliding};
    tiny2d::Update(no_rectangles, result, 0.0f, 1000.0f, 1000.0f, 0.0f, 1.0f,
                   {}, 0.0f, 0.01f);
    return result.front();
  };
  const tiny2d::Circle static_case = run_top_wall(3.0f);
  CHECK(NearlyEqual(static_case.velocity.x +
                        static_case.angular_velocity * static_case.radius,
                    0.0f));
  const tiny2d::Circle kinetic_case = run_top_wall(100.0f);
  CHECK(kinetic_case.velocity.x < 100.0f);
  CHECK(kinetic_case.velocity.x +
            kinetic_case.angular_velocity * kinetic_case.radius >
        0.0f);
  CHECK(kinetic_case.angular_velocity < 0.0f);
}

void TestMixedUpdateValidationAndLegacyFallback() {
  const tiny2d::Rectangle rectangle{
      1.0f, {100.0f, 100.0f}, {8.0f, 1.0f}, 0.1f, 0.5f, 20.0f, 10.0f};
  tiny2d::Circle circle;
  circle.position = {300.0f, 300.0f};
  circle.radius = 10.0f;

  std::vector<tiny2d::Rectangle> legacy{rectangle};
  std::vector<tiny2d::Rectangle> mixed{rectangle};
  std::vector<tiny2d::Circle> no_circles;
  tiny2d::Update(legacy, 0.25f, 1000.0f, 1000.0f, 0.7f, 0.3f, {2.0f, -1.0f},
                 5.0f);
  tiny2d::Update(mixed, no_circles, 0.25f, 1000.0f, 1000.0f, 0.7f, 0.3f,
                 {2.0f, -1.0f}, 5.0f, 20.0f);
  CHECK(SameRectangle(legacy.front(), mixed.front()));

  tiny2d::Circle invalid_circle = circle;
  invalid_circle.radius = 0.0f;
  CheckInvalidMixedUpdate({rectangle}, {invalid_circle});
  CheckInvalidMixedUpdate({rectangle}, {invalid_circle}, 1.0f / 120.0f, 1000.0f,
                          1000.0f, 1.0f, 0.0f, {}, 98.1f, 0.01f, true);
  tiny2d::Rectangle invalid_material = rectangle;
  invalid_material.material = {0.5f, -1.0f, -1.0f};
  CheckInvalidMixedUpdate({invalid_material}, {circle});
  invalid_circle = circle;
  invalid_circle.material = {0.5f, 0.2f, 0.3f};
  CheckInvalidMixedUpdate({rectangle}, {invalid_circle});
  CheckInvalidMixedUpdate({rectangle}, {circle}, 1.0f / 120.0f, 1000.0f,
                          1000.0f, 1.0f, 0.0f, {}, 0.0f, -0.01f);
  invalid_circle = circle;
  invalid_circle.inertia_model = static_cast<tiny2d::CircleInertiaModel>(999);
  CheckInvalidMixedUpdate({rectangle}, {invalid_circle});

  const float nan = std::numeric_limits<float>::quiet_NaN();
  CheckInvalidCircleOperation(circle, [&](tiny2d::Circle& value) {
    tiny2d::AddForceAtPoint(value, {nan, 0.0f}, value.position);
  });
  CheckInvalidCircleOperation(
      circle, [&](tiny2d::Circle& value) { tiny2d::AddTorque(value, nan); });
  invalid_circle = circle;
  invalid_circle.material = {nan, 0.0f, 0.0f};
  CheckInvalidArgument([&] { tiny2d::GetMomentOfInertia(invalid_circle); });
}

void TestLegacyUpdateMatchesMixedUpdateTrajectories() {
  // Characterization gate: the rectangle-only Update and the mixed Update
  // with an empty circle vector must produce bit-identical trajectories
  // across contacts, walls, per-body materials, loads, and damping.
  std::vector<tiny2d::Rectangle> legacy = {
      {0.0f, {500.0f, 700.0f}, {}, -0.15f, 0.0f, 600.0f, 20.0f},
      {2.0f, {300.0f, 200.0f}, {60.0f, -10.0f}, 0.3f, 1.5f, 50.0f, 30.0f},
      {1.0f, {420.0f, 180.0f}, {-45.0f, 5.0f}, -0.4f, -2.0f, 40.0f, 40.0f},
      {4.0f, {700.0f, 300.0f}, {-20.0f, 0.0f}, 0.0f, 0.0f, 80.0f, 25.0f, true},
  };
  legacy[1].material = {0.6f, 0.5f, 0.3f};
  legacy[2].charge = 2.5f;
  legacy[2].linear_damping_rate = 0.2f;
  legacy[3].material = {0.1f, 0.9f, 0.7f};
  std::vector<tiny2d::Rectangle> mixed = legacy;
  std::vector<tiny2d::Circle> no_circles;

  constexpr float kStep = 1.0f / 240.0f;
  for (int step = 0; step < 5 * 240; ++step) {
    tiny2d::AddForceAtPoint(
        legacy[1], {3.0f, -9.0f},
        {legacy[1].position.x + 10.0f, legacy[1].position.y});
    tiny2d::AddForceAtPoint(mixed[1], {3.0f, -9.0f},
                            {mixed[1].position.x + 10.0f, mixed[1].position.y});
    tiny2d::AddTorque(legacy[2], 40.0f);
    tiny2d::AddTorque(mixed[2], 40.0f);

    tiny2d::Update(legacy, kStep, 1000.0f, 800.0f, 0.4f, 0.5f, {1.5f, -0.5f},
                   9.8f);
    tiny2d::Update(mixed, no_circles, kStep, 1000.0f, 800.0f, 0.4f, 0.5f,
                   {1.5f, -0.5f}, 9.8f, 20.0f);
    for (std::size_t i = 0; i < legacy.size(); ++i) {
      CHECK(SameRectangle(legacy[i], mixed[i]));
    }
  }
  CHECK(no_circles.empty());
}

void TestRectanglePerBodyMaterialOverride() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {1.0f, {100.0f, 100.0f}, {50.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
      {1.0f, {139.0f, 100.0f}, {-50.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f, true},
  };
  rectangles[0].material = {0.2f, 0.0f, 0.0f};
  rectangles[1].material = {0.8f, 0.0f, 0.0f};
  tiny2d::Update(rectangles, 0.0f, 1000.0f, 1000.0f, 0.0f, 0.0f, {}, 0.0f);
  CHECK(NearlyEqual(rectangles[0].velocity.x, -40.0f));
  CHECK(NearlyEqual(rectangles[1].velocity.x, 40.0f));
}

void TestMixedWorldLongRunIsFiniteAndDeterministic() {
  std::vector<tiny2d::Rectangle> first_rectangles = {
      {0.0f, {500.0f, 700.0f}, {}, -0.2f, 0.0f, 500.0f, 20.0f},
  };
  first_rectangles[0].material = {0.2f, 0.6f, 0.4f};
  std::vector<tiny2d::Circle> first_circles(2);
  first_circles[0].position = {350.0f, 200.0f};
  first_circles[0].velocity = {40.0f, 0.0f};
  first_circles[0].radius = 20.0f;
  first_circles[0].material = {0.6f, 0.5f, 0.3f};
  first_circles[1] = first_circles[0];
  first_circles[1].position = {650.0f, 200.0f};
  first_circles[1].velocity.x = -40.0f;
  first_circles[1].inertia_model = tiny2d::CircleInertiaModel::kHoop;
  std::vector<tiny2d::Rectangle> second_rectangles = first_rectangles;
  std::vector<tiny2d::Circle> second_circles = first_circles;

  constexpr float kStep = 1.0f / 240.0f;
  for (int step = 0; step < 60 * 240; ++step) {
    tiny2d::Update(first_rectangles, first_circles, kStep, 1000.0f, 800.0f,
                   0.2f, 0.4f, {}, 9.8f, 0.01f, true);
    tiny2d::Update(second_rectangles, second_circles, kStep, 1000.0f, 800.0f,
                   0.2f, 0.4f, {}, 9.8f, 0.01f, true);
    for (const tiny2d::Circle& body : first_circles) {
      CHECK(IsFinite(body));
      CHECK(body.position.x >= body.radius - kTolerance);
      CHECK(body.position.x <= 1000.0f - body.radius + kTolerance);
      CHECK(body.position.y >= body.radius - kTolerance);
      CHECK(body.position.y <= 800.0f - body.radius + kTolerance);
    }
  }
  CHECK(SameRectangle(first_rectangles[0], second_rectangles[0]));
  CHECK(SameCircle(first_circles[0], second_circles[0]));
  CHECK(SameCircle(first_circles[1], second_circles[1]));
}

}  // namespace

int main() {
  TestRotatedVerticesKeepRectangleSize();
  TestSatDetectsRotatedCollision();
  TestCenteredCollisionDoesNotCreateRotation();
  TestOffCenterCollisionCreatesRotation();
  TestAngularDampingReducesRotation();
  TestMomentOfInertiaAndCenteredForce();
  TestOffCenterForceCreatesPositiveClockwiseTorque();
  TestPerBodyExponentialDamping();
  TestZeroTimeUpdateClearsLoadsWithoutIntegratingThem();
  TestStaticAndFixedRotationLoadBehavior();
  TestElectricFieldUsesChargeAndMass();
  TestConfigurableGravity();
  TestConfigurableWallFriction();
  TestStaticRectangleDoesNotMove();
  TestRestitutionRequiresImpactSpeed();
  TestFixedRotationStaysLocked();
  TestLockedBlocksCollideOnRamp();
  TestLegalBoundaryValues();
  TestCollisionDetectionIsSymmetric();
  TestElasticCollisionConservesMomentumAndEnergy();
  TestInelasticCollisionDoesNotGainEnergy();
  TestAllWindowWallsReflect();
  TestCornerAndRotatedWallContactsStayInside();
  TestElectricFieldAndGravityCanCancel();
  TestPositiveAndNegativeChargesAccelerateOppositely();
  TestLongSimulationStaysFiniteAndInside();
  TestFixedSeedProperties();
  TestRejectsNonFiniteUpdateParameters();
  TestRejectsInvalidRectangles();
  TestRejectsInvalidLoadsAndDampingAtomically();
  TestRejectsImpossibleAndOverflowingStates();
  TestPublicFunctionsRejectInvalidRectangles();
  TestCircleInertiaLoadsAndFields();
  TestCircleRejectsEveryNonFinitePublicStateField();
  TestCircleCollisionGeometryAndSymmetry();
  TestSupportedCircleSweepDoesNotTunnel();
  TestFastCircleTravelsBeyondDiameterWithoutTunneling();
  TestLowSpeedCircleCcdMatchesDiscreteUpdate();
  TestCircleCcdMultipleImpactsAreDeterministic();
  TestCircleElasticCollisionAndMaterialMixing();
  TestCircleRectangleResponseAndWallFriction();
  TestMixedUpdateValidationAndLegacyFallback();
  TestLegacyUpdateMatchesMixedUpdateTrajectories();
  TestRectanglePerBodyMaterialOverride();
  TestMixedWorldLongRunIsFiniteAndDeterministic();
  return 0;
}
