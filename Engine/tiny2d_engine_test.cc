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

#include "test_support.h"

namespace {

constexpr float kTolerance = 0.001f;

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
    ::tiny2d::test::Check(false, "expected std::invalid_argument", __FILE__,
                          __LINE__);
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

struct GoldenBodyState {
  tiny2d::Vec2 position;
  tiny2d::Vec2 velocity;
};

struct GoldenCheckpoint {
  int step;
  std::array<GoldenBodyState, 3> rectangles;
  std::array<GoldenBodyState, 2> circles;
  std::array<float, 2> circle_angular_velocities;
};

std::vector<tiny2d::Rectangle> MakeGoldenRectangles() {
  std::vector<tiny2d::Rectangle> rectangles(3);
  rectangles[0].mass = 2.0f;
  rectangles[0].position = {30.0f, 20.0f};
  rectangles[0].velocity = {12.0f, -4.0f};
  rectangles[0].width = 10.0f;
  rectangles[0].height = 8.0f;
  rectangles[0].fixed_rotation = true;
  rectangles[0].charge = 0.5f;
  rectangles[0].angular_damping_rate = 0.0f;

  rectangles[1].mass = 1.0f;
  rectangles[1].position = {52.0f, 22.0f};
  rectangles[1].velocity = {-6.0f, 0.0f};
  rectangles[1].width = 9.0f;
  rectangles[1].height = 9.0f;
  rectangles[1].fixed_rotation = true;
  rectangles[1].charge = -0.25f;
  rectangles[1].angular_damping_rate = 0.0f;

  rectangles[2].mass = 0.0f;
  rectangles[2].position = {45.0f, 70.0f};
  rectangles[2].width = 60.0f;
  rectangles[2].height = 6.0f;
  rectangles[2].angular_damping_rate = 0.0f;

  return rectangles;
}

std::vector<tiny2d::Circle> MakeGoldenCircles() {
  std::vector<tiny2d::Circle> circles(2);
  circles[0].mass = 1.5f;
  circles[0].position = {40.0f, 40.0f};
  circles[0].velocity = {5.0f, 9.0f};
  circles[0].radius = 6.0f;
  circles[0].charge = 0.75f;
  circles[0].angular_damping_rate = 0.0f;

  circles[1].mass = 0.8f;
  circles[1].position = {58.0f, 44.0f};
  circles[1].velocity = {-8.0f, 6.0f};
  circles[1].radius = 5.0f;
  circles[1].inertia_model = tiny2d::CircleInertiaModel::kHoop;
  circles[1].angular_damping_rate = 0.0f;

  return circles;
}

void TestGoldenMixedTrajectoryCheckpoints() {
  // Non-vacuous characterization gate for the velocity/position integrate
  // split: these checkpoints were generated by building and running this
  // exact scene at HEAD accb121, before the constrained step restructuring,
  // and must reproduce bit for bit afterwards. The scene keeps every
  // trajectory operation exactly rounded on every platform: all rectangle
  // angles are zero (contact generation evaluates sin/cos of the frozen
  // angle, and only sin(+-0) = +-0, cos(+-0) = 1 are exact by C Annex F)
  // and every damping rate is zero (std::exp(-0) == 1). The values also
  // assume no floating-point contraction: every CI job builds for generic
  // x86-64 without FMA, so a future -march/-mfma or /arch:AVX2 change
  // would shift them.
  const std::array<GoldenCheckpoint, 5> golden = {{
      {1,
       {{{{0x1.e0ccdap+4F, 0x1.3fc2acp+4F}, {0x1.80199ap+3F, -0x1.cbf258p+1F}},
         {{0x1.9fccc6p+5F, 0x1.600702p+4F}, {-0x1.803334p+2F, 0x1.a4b18p-2F}},
         {{0x1.68p+5F, 0x1.18p+6F}, {0x0p+0F, 0x0p+0F}}}},
       {{{{0x1.402ab8p+5F, 0x1.40504p+5F}, {0x1.406666p+2F, 0x1.2cf258p+3F}},
         {{0x1.cfbbbcp+5F, 0x1.6036bp+5F}, {-0x1p+3F, 0x1.9a28f6p+2F}}}},
       {0x0p+0F, 0x0p+0F}},
      {10,
       {{{{0x1.e802fp+4F, 0x1.3ed31p+4F}, {0x1.810004p+3F, 0x1.1111dp-4F}},
         {{0x1.9dfe8ap+5F, 0x1.6181ap+4F}, {-0x1.820008p+2F, 0x1.06eefp+2F}},
         {{0x1.68p+5F, 0x1.18p+6F}, {0x0p+0F, 0x0p+0F}}}},
       {{{{0x1.41ad9ap+5F, 0x1.43bde4p+5F}, {0x1.43fffcp+2F, 0x1.a1777p+3F}},
         {{0x1.cd5558p+5F, 0x1.62bfd6p+5F}, {-0x1p+3F, 0x1.42ccc8p+3F}}}},
       {0x0p+0F, 0x0p+0F}},
      {60,
       {{{{0x1.0830dp+5F, 0x1.619d04p+4F}, {0x1.860018p+3F, 0x1.46666p+4F}},
         {{0x1.93cf34p+5F, 0x1.921f24p+4F}, {-0x1.8c003p+2F, 0x1.8a6668p+4F}},
         {{0x1.68p+5F, 0x1.18p+6F}, {0x0p+0F, 0x0p+0F}}}},
       {{{{0x1.4a6198p+5F, 0x1.6aadf8p+5F}, {0x1.57ffe8p+2F, 0x1.0a3328p+5F}},
         {{0x1.c0001p+5F, 0x1.84ef0ap+5F}, {-0x1p+3F, 0x1.e86672p+4F}}}},
       {0x0p+0F, 0x0p+0F}},
      {240,
       {{{{0x1.3f6fd4p+5F, 0x1.8efc7p+5F}, {0x1.04518ep+2F, 0x1.b4d8d6p-1F}},
         {{0x1.9b5d12p+5F, 0x1.95b74ep+5F}, {0x1.7c1baap+3F, 0x1.8eb368p+4F}},
         {{0x1.68p+5F, 0x1.18p+6F}, {0x0p+0F, 0x0p+0F}}}},
       {{{{0x1.4b6052p+5F, 0x1.e8147cp+5F}, {0x1.0d5c72p+1F, 0x0p+0F}},
         {{0x1.db842p+5F, 0x1.ef6342p+5F}, {0x1.f9a638p+1F, -0x1.40a924p+3F}}}},
       {0x1.677f9p-2F, 0x1.94def4p-1F}},
      {600,
       {{{{0x1.5f5cbap+5F, 0x1.982ae2p+5F}, {0x1.ca891cp-6F, 0x1.5b21d4p-4F}},
         {{0x1.af187ap+5F, 0x1.f4147cp+5F}, {0x1.b1eff2p-7F, 0x1.277d6ap-12F}},
         {{0x1.68p+5F, 0x1.18p+6F}, {0x0p+0F, 0x0p+0F}}}},
       {{{{0x1.5b2cf6p+5F, 0x1.e814c6p+5F}, {0x1.b91f7ep-7F, 0x1p-27F}},
         {{0x1.1ab732p+6F, 0x1.f0147cp+5F}, {0x1.dde642p+2F, 0x0p+0F}}}},
       {0x1.26584ep-9F, 0x1.7ec47cp+0F}},
  }};

  std::vector<tiny2d::Rectangle> rectangles = MakeGoldenRectangles();
  std::vector<tiny2d::Circle> circles = MakeGoldenCircles();
  std::size_t next_checkpoint = 0;
  for (int step = 1; step <= 600; ++step) {
    tiny2d::Update(rectangles, circles, 1.0f / 240.0f, 90.0f, 80.0f, 0.5f, 0.4f,
                   {3.0f, -2.0f}, 98.1f, 20.0f, false);
    if (next_checkpoint < golden.size() &&
        step == golden[next_checkpoint].step) {
      const GoldenCheckpoint& expected = golden[next_checkpoint];
      for (std::size_t i = 0; i < rectangles.size(); ++i) {
        CHECK(rectangles[i].position.x == expected.rectangles[i].position.x);
        CHECK(rectangles[i].position.y == expected.rectangles[i].position.y);
        CHECK(rectangles[i].velocity.x == expected.rectangles[i].velocity.x);
        CHECK(rectangles[i].velocity.y == expected.rectangles[i].velocity.y);
      }
      for (std::size_t i = 0; i < circles.size(); ++i) {
        CHECK(circles[i].position.x == expected.circles[i].position.x);
        CHECK(circles[i].position.y == expected.circles[i].position.y);
        CHECK(circles[i].velocity.x == expected.circles[i].velocity.x);
        CHECK(circles[i].velocity.y == expected.circles[i].velocity.y);
        CHECK(circles[i].angular_velocity ==
              expected.circle_angular_velocities[i]);
      }
      ++next_checkpoint;
    }
  }
  CHECK(next_checkpoint == golden.size());
}

void TestMixedUpdateMatchesConstrainedUpdateWithoutConstraints() {
  // Wrapper-consistency gate: the mixed overload and the constrained
  // overload with empty constraint vectors must stay bit-identical, with
  // and without the CCD path enabled.
  for (const bool enable_ccd : {false, true}) {
    std::vector<tiny2d::Rectangle> mixed_rectangles = MakeGoldenRectangles();
    std::vector<tiny2d::Circle> mixed_circles = MakeGoldenCircles();
    std::vector<tiny2d::Rectangle> constrained_rectangles = mixed_rectangles;
    std::vector<tiny2d::Circle> constrained_circles = mixed_circles;
    const std::vector<tiny2d::RevolutePin> no_pins;
    const std::vector<tiny2d::PulleyRope> no_ropes;

    for (int step = 0; step < 300; ++step) {
      tiny2d::Update(mixed_rectangles, mixed_circles, 1.0f / 240.0f, 90.0f,
                     80.0f, 0.5f, 0.4f, {3.0f, -2.0f}, 98.1f, 20.0f,
                     enable_ccd);
      tiny2d::Update(constrained_rectangles, constrained_circles, no_pins,
                     no_ropes, 1.0f / 240.0f, 90.0f, 80.0f, 0.5f, 0.4f,
                     {3.0f, -2.0f}, 98.1f, 20.0f, enable_ccd);
      for (std::size_t i = 0; i < mixed_rectangles.size(); ++i) {
        CHECK(SameRectangle(mixed_rectangles[i], constrained_rectangles[i]));
      }
      for (std::size_t i = 0; i < mixed_circles.size(); ++i) {
        CHECK(SameCircle(mixed_circles[i], constrained_circles[i]));
      }
    }
  }
}

// Reference Atwood machine in SI-like units: masses 1.0 and 1.2, a solid
// disk pulley of mass 0.5 and radius 0.1 pinned at (50, 20), vertical rope
// segments from the horizontal tangent points. Analytical acceleration
// a = (m_b - m_a) g / (m_a + m_b + I/R^2) = 1.962 / 2.45 ~= 0.800816.
struct AtwoodFixture {
  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> circles;
  std::vector<tiny2d::RevolutePin> pins;
  std::vector<tiny2d::PulleyRope> ropes;
};

AtwoodFixture MakeAtwoodFixture(float mass_a, float mass_b) {
  AtwoodFixture fixture;
  fixture.circles.resize(3);
  tiny2d::Circle& pulley = fixture.circles[0];
  pulley.mass = 0.5f;
  pulley.position = {50.0f, 20.0f};
  pulley.radius = 0.1f;
  pulley.angular_damping_rate = 0.0f;

  tiny2d::Circle& end_a = fixture.circles[1];
  end_a.mass = mass_a;
  end_a.position = {49.9f, 22.0f};
  end_a.radius = 0.05f;
  end_a.angular_damping_rate = 0.0f;

  tiny2d::Circle& end_b = fixture.circles[2];
  end_b.mass = mass_b;
  end_b.position = {50.1f, 23.0f};
  end_b.radius = 0.05f;
  end_b.angular_damping_rate = 0.0f;

  fixture.pins = {{0, {50.0f, 20.0f}}};
  tiny2d::PulleyRope rope;
  rope.body_a = {tiny2d::BodyKind::kCircle, 1};
  rope.body_b = {tiny2d::BodyKind::kCircle, 2};
  rope.pulley_circle_index = 0;
  rope.anchor_a = {49.9f, 20.0f};
  rope.anchor_b = {50.1f, 20.0f};
  rope.segment_length_sum = 5.0f;
  fixture.ropes = {rope};
  return fixture;
}

void StepAtwoodFixture(AtwoodFixture& fixture, float delta_time,
                       tiny2d::ConstraintReactions* reactions) {
  tiny2d::Update(fixture.rectangles, fixture.circles, fixture.pins,
                 fixture.ropes, delta_time, 100.0f, 100.0f, 0.0f, 0.0f, {},
                 9.81f, 0.0f, false, reactions);
}

void TestRevolutePinHoldsCircleUnderGravity() {
  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> circles(1);
  circles[0].mass = 2.0f;
  circles[0].position = {50.0f, 50.0f};
  circles[0].radius = 5.0f;
  circles[0].angular_velocity = 3.0f;
  circles[0].angular_damping_rate = 0.0f;
  const std::vector<tiny2d::RevolutePin> pins = {{0, {50.0f, 50.0f}}};
  const std::vector<tiny2d::PulleyRope> no_ropes;
  tiny2d::ConstraintReactions reactions;

  for (int step = 0; step < 240; ++step) {
    tiny2d::Update(rectangles, circles, pins, no_ropes, 1.0f / 240.0f, 100.0f,
                   100.0f, 0.0f, 0.0f, {}, 9.8f, 20.0f, false, &reactions);
    CHECK(circles[0].position.x == 50.0f);
    CHECK(circles[0].position.y == 50.0f);
    CHECK(circles[0].velocity.x == 0.0f);
    CHECK(circles[0].velocity.y == 0.0f);
    // The pin carries the weight: force on the circle is m g upward.
    CHECK(reactions.pin_forces.size() == 1);
    CHECK(NearlyEqual(reactions.pin_forces[0].x, 0.0f));
    CHECK(NearlyEqual(reactions.pin_forces[0].y, -2.0f * 9.8f));
  }
  // Rotation stays free: the spin is untouched by the pin.
  CHECK(circles[0].angular_velocity == 3.0f);
}

void TestPulleyRopeMatchesAtwoodAcceleration() {
  AtwoodFixture fixture = MakeAtwoodFixture(1.0f, 1.2f);
  tiny2d::ConstraintReactions reactions;
  constexpr float kStep = 1.0f / 480.0f;
  constexpr int kSteps = 480;
  for (int step = 0; step < kSteps; ++step) {
    StepAtwoodFixture(fixture, kStep, &reactions);
  }
  const double elapsed = static_cast<double>(kSteps) * kStep;
  const double expected_acceleration = 0.2 * 9.81 / 2.45;
  const double measured_acceleration = fixture.circles[2].velocity.y / elapsed;
  CHECK(std::abs(measured_acceleration - expected_acceleration) /
            expected_acceleration <=
        0.01);

  // T_a = m_a (g + a), T_b = m_b (g - a).
  const double expected_tension_a = 1.0 * (9.81 + expected_acceleration);
  const double expected_tension_b = 1.2 * (9.81 - expected_acceleration);
  CHECK(reactions.rope_tensions.size() == 1);
  CHECK(std::abs(reactions.rope_tensions[0].tension_a - expected_tension_a) /
            expected_tension_a <=
        0.01);
  CHECK(std::abs(reactions.rope_tensions[0].tension_b - expected_tension_b) /
            expected_tension_b <=
        0.01);
}

void TestPulleyRopeNoSlipCouplingAndOppositeSpeeds() {
  AtwoodFixture fixture = MakeAtwoodFixture(1.0f, 1.2f);
  constexpr float kStep = 1.0f / 480.0f;
  for (int step = 0; step < 480; ++step) {
    StepAtwoodFixture(fixture, kStep, nullptr);
    const tiny2d::Circle& pulley = fixture.circles[0];
    const tiny2d::Circle& end_a = fixture.circles[1];
    const tiny2d::Circle& end_b = fixture.circles[2];
    // Equal speed magnitude, opposite rope direction: a rises, b descends.
    CHECK(std::abs(end_a.velocity.y + end_b.velocity.y) <= 0.0001f);
    CHECK(end_a.velocity.y <= 0.0f);
    CHECK(end_b.velocity.y >= 0.0f);
    // No slip: the rim speed matches both rope speeds.
    CHECK(std::abs(end_b.velocity.y -
                   pulley.angular_velocity * pulley.radius) <= 0.0001f);
    CHECK(pulley.angular_velocity >= 0.0f);
  }
}

void TestPulleyRopeLengthDriftStaysBounded() {
  AtwoodFixture fixture = MakeAtwoodFixture(1.0f, 1.0f);
  // Balanced masses drift at constant speed and stay far from every wall
  // for the whole 60 s window.
  fixture.circles[1].position.y = 50.0f;
  fixture.circles[2].position.y = 50.0f;
  fixture.circles[1].velocity = {0.0f, -0.02f};
  fixture.circles[2].velocity = {0.0f, 0.02f};
  fixture.circles[0].angular_velocity = 0.2f;
  fixture.ropes[0].segment_length_sum = 60.0f;

  constexpr float kStep = 1.0f / 480.0f;
  const auto rope_length = [&]() {
    const double left_x = static_cast<double>(fixture.circles[1].position.x) -
                          static_cast<double>(fixture.ropes[0].anchor_a.x);
    const double left_y = static_cast<double>(fixture.circles[1].position.y) -
                          static_cast<double>(fixture.ropes[0].anchor_a.y);
    const double right_x = static_cast<double>(fixture.circles[2].position.x) -
                           static_cast<double>(fixture.ropes[0].anchor_b.x);
    const double right_y = static_cast<double>(fixture.circles[2].position.y) -
                           static_cast<double>(fixture.ropes[0].anchor_b.y);
    return std::sqrt(left_x * left_x + left_y * left_y) +
           std::sqrt(right_x * right_x + right_y * right_y);
  };
  for (int step = 0; step < 60 * 480; ++step) {
    StepAtwoodFixture(fixture, kStep, nullptr);
    if (step % 480 == 0) {
      CHECK(std::abs(rope_length() - 60.0) < 0.0001);
    }
  }
  CHECK(std::abs(rope_length() - 60.0) < 0.0001);
  CHECK(IsFinite(fixture.circles[1]));
  CHECK(IsFinite(fixture.circles[2]));
}

void TestPulleyRopeWithFloorContactStaysFinite() {
  // The heavy side reaches the area floor: the taut bilateral rope and the
  // window contact fight over the body, and the step must stay finite and
  // deterministic for a full 60 s run.
  AtwoodFixture first = MakeAtwoodFixture(1.0f, 1.2f);
  first.circles[1].position.y = 90.0f;
  first.circles[2].position.y = 95.0f;
  first.ropes[0].segment_length_sum = 145.0f;
  AtwoodFixture second = first;

  constexpr float kStep = 1.0f / 480.0f;
  for (int step = 0; step < 60 * 480; ++step) {
    StepAtwoodFixture(first, kStep, nullptr);
    StepAtwoodFixture(second, kStep, nullptr);
    for (const tiny2d::Circle& circle : first.circles) {
      CHECK(IsFinite(circle));
    }
  }
  for (std::size_t i = 0; i < first.circles.size(); ++i) {
    CHECK(SameCircle(first.circles[i], second.circles[i]));
  }
}

void TestConstrainedUpdateRejectsInvalidInput() {
  const AtwoodFixture reference = MakeAtwoodFixture(1.0f, 1.2f);

  const auto expect_reject = [&](auto mutate) {
    AtwoodFixture fixture = reference;
    mutate(fixture);
    const std::vector<tiny2d::Rectangle> rectangles_before = fixture.rectangles;
    const std::vector<tiny2d::Circle> circles_before = fixture.circles;
    tiny2d::ConstraintReactions reactions;
    reactions.pin_forces = {{123.0f, 456.0f}};
    bool threw = false;
    try {
      StepAtwoodFixture(fixture, 1.0f / 480.0f, &reactions);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
    // Failure atomicity: bodies and the reaction report stay untouched.
    CHECK(fixture.rectangles.size() == rectangles_before.size());
    for (std::size_t i = 0; i < circles_before.size(); ++i) {
      CHECK(SameCircle(fixture.circles[i], circles_before[i]));
    }
    CHECK(reactions.pin_forces.size() == 1);
    CHECK(reactions.pin_forces[0].x == 123.0f);
    CHECK(reactions.pin_forces[0].y == 456.0f);
  };

  expect_reject([](AtwoodFixture& f) { f.pins[0].circle_index = -1; });
  expect_reject([](AtwoodFixture& f) { f.pins[0].circle_index = 3; });
  expect_reject([](AtwoodFixture& f) { f.circles[0].mass = 0.0f; });
  expect_reject([](AtwoodFixture& f) {
    f.pins[0].world_anchor.x = std::numeric_limits<float>::quiet_NaN();
  });
  // The anchor disk must fit inside the area.
  expect_reject(
      [](AtwoodFixture& f) { f.pins[0].world_anchor = {0.05f, 20.0f}; });
  expect_reject([](AtwoodFixture& f) { f.pins.push_back(f.pins[0]); });
  expect_reject([](AtwoodFixture& f) {
    // Deliberately out-of-enumerator input for the rejection path; the
    // cast is well-defined for a scoped enum with int underlying type.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    f.ropes[0].body_a.kind = static_cast<tiny2d::BodyKind>(2);
  });
  expect_reject([](AtwoodFixture& f) { f.ropes[0].body_a.index = 9; });
  expect_reject([](AtwoodFixture& f) { f.circles[1].mass = 0.0f; });
  expect_reject(
      [](AtwoodFixture& f) { f.ropes[0].body_b = f.ropes[0].body_a; });
  expect_reject([](AtwoodFixture& f) { f.ropes[0].body_a.index = 0; });
  expect_reject([](AtwoodFixture& f) {
    // A rope end must not itself be pinned.
    f.pins.push_back({1, {49.9f, 22.0f}});
  });
  expect_reject([](AtwoodFixture& f) { f.ropes[0].pulley_circle_index = 9; });
  expect_reject([](AtwoodFixture& f) {
    // Pulley without a pin: retarget the rope at a fresh unpinned circle.
    tiny2d::Circle free_pulley;
    free_pulley.mass = 0.5f;
    free_pulley.position = {60.0f, 50.0f};
    free_pulley.radius = 0.1f;
    free_pulley.angular_damping_rate = 0.0f;
    f.circles.push_back(free_pulley);
    f.ropes[0].pulley_circle_index = 3;
  });
  expect_reject([](AtwoodFixture& f) { f.circles[0].fixed_rotation = true; });
  expect_reject([](AtwoodFixture& f) {
    f.ropes[0].anchor_b.y = std::numeric_limits<float>::infinity();
  });
  expect_reject([](AtwoodFixture& f) { f.ropes[0].segment_length_sum = 0.0f; });
  expect_reject(
      [](AtwoodFixture& f) { f.ropes[0].segment_length_sum = -1.0f; });
  expect_reject([](AtwoodFixture& f) {
    // A rope end sitting on its anchor has no defined direction.
    f.circles[1].position = f.ropes[0].anchor_a;
  });

  // CCD cannot be combined with constraints.
  {
    AtwoodFixture fixture = reference;
    bool threw = false;
    try {
      tiny2d::Update(fixture.rectangles, fixture.circles, fixture.pins,
                     fixture.ropes, 1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.0f,
                     {}, 9.81f, 0.0f, true, nullptr);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  }
}

void TestConstrainedUpdateZeroDtReportsZeroReactions() {
  AtwoodFixture fixture = MakeAtwoodFixture(1.0f, 1.2f);
  // Inconsistent initial velocities: both ends descending at 1.
  fixture.circles[1].velocity = {0.0f, 1.0f};
  fixture.circles[2].velocity = {0.0f, 1.0f};
  tiny2d::ConstraintReactions reactions;
  StepAtwoodFixture(fixture, 0.0f, &reactions);

  // Zero dt still resolves constraint velocities to a consistent state.
  CHECK(std::abs(fixture.circles[1].velocity.y +
                 fixture.circles[2].velocity.y) <= 0.0001f);
  CHECK(std::abs(fixture.circles[2].velocity.y -
                 fixture.circles[0].angular_velocity *
                     fixture.circles[0].radius) <= 0.0001f);
  // Positions project to the targets and reactions report zero.
  CHECK(fixture.circles[0].position.x == 50.0f);
  CHECK(fixture.circles[0].position.y == 20.0f);
  CHECK(reactions.pin_forces.size() == 1);
  CHECK(reactions.rope_tensions.size() == 1);
  CHECK(reactions.pin_forces[0].x == 0.0f);
  CHECK(reactions.pin_forces[0].y == 0.0f);
  CHECK(reactions.rope_tensions[0].tension_a == 0.0f);
  CHECK(reactions.rope_tensions[0].tension_b == 0.0f);
}

tiny2d::ConstraintSet MakeAtwoodConstraintSet(const AtwoodFixture& fixture) {
  tiny2d::ConstraintSet constraints;
  constraints.revolute_pins = fixture.pins;
  constraints.pulley_ropes = fixture.ropes;
  return constraints;
}

void TestConstraintSetOverloadMatchesPinsRopesOverload() {
  // Wrapper-consistency gate: the V18 (pins, ropes) overload and the
  // ConstraintSet overload must stay bit-identical.
  AtwoodFixture legacy = MakeAtwoodFixture(1.0f, 1.2f);
  AtwoodFixture through_set = MakeAtwoodFixture(1.0f, 1.2f);
  const tiny2d::ConstraintSet constraints =
      MakeAtwoodConstraintSet(through_set);
  constexpr float kStep = 1.0f / 480.0f;
  for (int step = 0; step < 300; ++step) {
    StepAtwoodFixture(legacy, kStep, nullptr);
    tiny2d::Update(through_set.rectangles, through_set.circles, constraints,
                   kStep, 100.0f, 100.0f, 0.0f, 0.0f, {}, 9.81f, 0.0f, false,
                   nullptr);
    for (std::size_t i = 0; i < legacy.circles.size(); ++i) {
      CHECK(SameCircle(legacy.circles[i], through_set.circles[i]));
    }
  }
}

// A single bob hanging from an anchor rod: released at two degrees, its
// small-angle period is 2 pi sqrt(L / g).
struct RodPendulumFixture {
  std::vector<tiny2d::Rectangle> rectangles;
  std::vector<tiny2d::Circle> circles;
  tiny2d::ConstraintSet constraints;
};

RodPendulumFixture MakeRodPendulumFixture(float length,
                                          float initial_angle_radians) {
  RodPendulumFixture fixture;
  fixture.circles.resize(1);
  tiny2d::Circle& bob = fixture.circles[0];
  bob.mass = 1.0f;
  // Positive angles are clockwise on screen: from hanging, toward -X.
  bob.position = {50.0f - length * std::sin(initial_angle_radians),
                  50.0f + length * std::cos(initial_angle_radians)};
  bob.radius = 0.05f;
  bob.fixed_rotation = true;
  bob.angular_damping_rate = 0.0f;
  tiny2d::AnchorRod rod;
  rod.body = {tiny2d::BodyKind::kCircle, 0};
  rod.world_anchor = {50.0f, 50.0f};
  rod.length = length;
  fixture.constraints.anchor_rods = {rod};
  return fixture;
}

void StepConstraintSet(RodPendulumFixture& fixture, float delta_time,
                       tiny2d::ConstraintReactions* reactions) {
  tiny2d::Update(fixture.rectangles, fixture.circles, fixture.constraints,
                 delta_time, 100.0f, 100.0f, 0.0f, 0.0f, {}, 9.81f, 0.0f, false,
                 reactions);
}

void TestAnchorRodPendulumMatchesAnalyticalPeriod() {
  constexpr float kLength = 2.0f;
  constexpr float kInitialAngle = 0.0349066f;  // 2 degrees.
  RodPendulumFixture fixture = MakeRodPendulumFixture(kLength, kInitialAngle);
  tiny2d::ConstraintReactions reactions;
  constexpr float kStep = 1.0f / 480.0f;
  const double expected_period = 2.0 * 3.14159265358979 * std::sqrt(2.0 / 9.81);

  const auto angle = [&]() {
    const double offset_x = fixture.circles[0].position.x - 50.0;
    const double offset_y = fixture.circles[0].position.y - 50.0;
    return std::atan2(-offset_x, offset_y);
  };

  double previous_angle = angle();
  double previous_time = 0.0;
  std::array<double, 3> crossings{};
  std::size_t crossing_count = 0;
  bool checked_bottom_force = false;
  for (int step = 1; step <= 4000 && crossing_count < crossings.size();
       ++step) {
    StepConstraintSet(fixture, kStep, &reactions);
    const double time = static_cast<double>(step) * kStep;
    const double current_angle = angle();
    if (previous_angle > 0.0 && current_angle <= 0.0) {
      const double fraction = previous_angle / (previous_angle - current_angle);
      crossings[crossing_count++] = previous_time + fraction * kStep;
      if (!checked_bottom_force) {
        // At the bottom the rod carries the weight plus the tiny
        // centripetal term, so the reported force is m g within 1%.
        CHECK(reactions.anchor_rod_forces.size() == 1);
        CHECK(NearlyEqual(reactions.anchor_rod_forces[0], 9.81f, 0.01f));
        checked_bottom_force = true;
      }
    }
    previous_angle = current_angle;
    previous_time = time;
  }
  CHECK(crossing_count == crossings.size());
  const double measured_period = crossings[2] - crossings[0];
  CHECK(std::abs(measured_period - 2.0 * expected_period) /
            (2.0 * expected_period) <=
        0.01);
}

void TestLinkRodKeepsDistanceAndReportsCompression() {
  const auto make_pair_fixture = [](tiny2d::Vec2 velocity_a,
                                    tiny2d::Vec2 velocity_b) {
    RodPendulumFixture fixture;
    fixture.circles.resize(2);
    fixture.circles[0].mass = 1.0f;
    fixture.circles[0].position = {48.0f, 50.0f};
    fixture.circles[0].velocity = velocity_a;
    fixture.circles[0].radius = 0.05f;
    fixture.circles[0].angular_damping_rate = 0.0f;
    fixture.circles[1] = fixture.circles[0];
    fixture.circles[1].position = {52.0f, 50.0f};
    fixture.circles[1].velocity = velocity_b;
    tiny2d::LinkRod rod;
    rod.body_a = {tiny2d::BodyKind::kCircle, 0};
    rod.body_b = {tiny2d::BodyKind::kCircle, 1};
    rod.length = 4.0f;
    fixture.constraints.link_rods = {rod};
    return fixture;
  };
  const auto step_zero_gravity = [](RodPendulumFixture& fixture,
                                    tiny2d::ConstraintReactions* reactions) {
    tiny2d::Update(fixture.rectangles, fixture.circles, fixture.constraints,
                   1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.0f, {}, 0.0f, 0.0f,
                   false, reactions);
  };

  // Approaching ends put the rod in compression (negative axial force) and
  // momentum conservation stops both bodies.
  RodPendulumFixture approaching =
      make_pair_fixture({1.0f, 0.0f}, {-1.0f, 0.0f});
  tiny2d::ConstraintReactions reactions;
  step_zero_gravity(approaching, &reactions);
  CHECK(reactions.link_rod_forces.size() == 1);
  CHECK(reactions.link_rod_forces[0] < 0.0f);
  CHECK(NearlyEqual(approaching.circles[0].velocity.x, 0.0f, 0.0001f));
  CHECK(NearlyEqual(approaching.circles[1].velocity.x, 0.0f, 0.0001f));

  // Separating ends put the rod in tension.
  RodPendulumFixture separating =
      make_pair_fixture({-1.0f, 0.0f}, {1.0f, 0.0f});
  step_zero_gravity(separating, &reactions);
  CHECK(reactions.link_rod_forces[0] > 0.0f);

  // A spinning pair keeps its distance for 60 s.
  RodPendulumFixture spinning = make_pair_fixture({0.0f, 1.0f}, {0.0f, -1.0f});
  for (int step = 0; step < 60 * 480; ++step) {
    step_zero_gravity(spinning, nullptr);
  }
  const double distance =
      std::hypot(static_cast<double>(spinning.circles[0].position.x) -
                     spinning.circles[1].position.x,
                 static_cast<double>(spinning.circles[0].position.y) -
                     spinning.circles[1].position.y);
  CHECK(std::abs(distance - 4.0) < 0.0001);
  CHECK(IsFinite(spinning.circles[0]));
  CHECK(IsFinite(spinning.circles[1]));
}

void TestAnchorRodWithFloorContactStaysFinite() {
  // The hanging bob overlaps the area floor, so the rod and the window
  // contact fight over it while it slides; the step must stay finite and
  // deterministic for 60 s.
  const auto make_fixture = []() {
    RodPendulumFixture fixture;
    fixture.circles.resize(1);
    tiny2d::Circle& bob = fixture.circles[0];
    bob.mass = 1.0f;
    bob.position = {50.0f, 99.7f};
    bob.velocity = {2.0f, 0.0f};
    bob.radius = 0.5f;
    bob.angular_damping_rate = 0.0f;
    tiny2d::AnchorRod rod;
    rod.body = {tiny2d::BodyKind::kCircle, 0};
    rod.world_anchor = {50.0f, 95.2f};
    rod.length = 4.5f;
    fixture.constraints.anchor_rods = {rod};
    return fixture;
  };
  RodPendulumFixture first = make_fixture();
  RodPendulumFixture second = make_fixture();
  for (int step = 0; step < 60 * 480; ++step) {
    StepConstraintSet(first, 1.0f / 480.0f, nullptr);
    StepConstraintSet(second, 1.0f / 480.0f, nullptr);
    CHECK(IsFinite(first.circles[0]));
  }
  CHECK(SameCircle(first.circles[0], second.circles[0]));
}

void TestRodAndRopeShareBodyStaysFiniteAndDeterministic() {
  // An anchor rod fixes the Atwood fixture's side-a segment length, which
  // together with the rope locks the whole machine: the mixed
  // pin/rope/rod interleaving must settle statically, stay finite, and be
  // bitwise deterministic.
  const auto make_fixture = []() {
    AtwoodFixture fixture = MakeAtwoodFixture(1.0f, 1.2f);
    tiny2d::AnchorRod rod;
    rod.body = {tiny2d::BodyKind::kCircle, 1};
    rod.world_anchor = {49.9f, 20.0f};
    rod.length = 2.0f;
    return std::make_pair(fixture, rod);
  };
  auto [first, first_rod] = make_fixture();
  auto [second, second_rod] = make_fixture();
  tiny2d::ConstraintSet first_constraints = MakeAtwoodConstraintSet(first);
  first_constraints.anchor_rods = {first_rod};
  tiny2d::ConstraintSet second_constraints = MakeAtwoodConstraintSet(second);
  second_constraints.anchor_rods = {second_rod};

  for (int step = 0; step < 60 * 480; ++step) {
    tiny2d::Update(first.rectangles, first.circles, first_constraints,
                   1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.0f, {}, 9.81f, 0.0f,
                   false, nullptr);
    tiny2d::Update(second.rectangles, second.circles, second_constraints,
                   1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.0f, {}, 9.81f, 0.0f,
                   false, nullptr);
    for (const tiny2d::Circle& circle : first.circles) {
      CHECK(IsFinite(circle));
    }
  }
  for (std::size_t i = 0; i < first.circles.size(); ++i) {
    CHECK(SameCircle(first.circles[i], second.circles[i]));
  }
  // The rod holds side a at its initial hang.
  CHECK(std::abs(first.circles[1].position.y - 22.0f) < 0.001f);
}

void TestConstrainedUpdateRejectsInvalidRods() {
  const auto make_reference = []() {
    RodPendulumFixture fixture;
    fixture.circles.resize(2);
    fixture.circles[0].mass = 1.0f;
    fixture.circles[0].position = {50.0f, 52.0f};
    fixture.circles[0].radius = 0.05f;
    fixture.circles[0].angular_damping_rate = 0.0f;
    fixture.circles[1] = fixture.circles[0];
    fixture.circles[1].position = {50.0f, 54.0f};
    tiny2d::AnchorRod anchor_rod;
    anchor_rod.body = {tiny2d::BodyKind::kCircle, 0};
    anchor_rod.world_anchor = {50.0f, 50.0f};
    anchor_rod.length = 2.0f;
    fixture.constraints.anchor_rods = {anchor_rod};
    tiny2d::LinkRod link_rod;
    link_rod.body_a = {tiny2d::BodyKind::kCircle, 0};
    link_rod.body_b = {tiny2d::BodyKind::kCircle, 1};
    link_rod.length = 2.0f;
    fixture.constraints.link_rods = {link_rod};
    return fixture;
  };

  const auto expect_reject = [&](auto mutate) {
    RodPendulumFixture fixture = make_reference();
    mutate(fixture);
    const std::vector<tiny2d::Circle> circles_before = fixture.circles;
    tiny2d::ConstraintReactions reactions;
    reactions.anchor_rod_forces = {123.0f};
    bool threw = false;
    try {
      StepConstraintSet(fixture, 1.0f / 480.0f, &reactions);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
    for (std::size_t i = 0; i < circles_before.size(); ++i) {
      CHECK(SameCircle(fixture.circles[i], circles_before[i]));
    }
    CHECK(reactions.anchor_rod_forces.size() == 1);
    CHECK(reactions.anchor_rod_forces[0] == 123.0f);
  };

  expect_reject([](RodPendulumFixture& f) {
    // Deliberately out-of-enumerator input for the rejection path; the
    // cast is well-defined for a scoped enum with int underlying type.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    f.constraints.anchor_rods[0].body.kind = static_cast<tiny2d::BodyKind>(2);
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.anchor_rods[0].body.index = -1;
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.anchor_rods[0].body.index = 9;
  });
  expect_reject([](RodPendulumFixture& f) { f.circles[0].mass = 0.0f; });
  expect_reject([](RodPendulumFixture& f) {
    // A pinned circle already has both linear degrees of freedom removed.
    f.constraints.revolute_pins = {{0, {50.0f, 52.0f}}};
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.anchor_rods[0].world_anchor.y =
        std::numeric_limits<float>::quiet_NaN();
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.anchor_rods[0].length = 0.0f;
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.anchor_rods[0].length = -1.0f;
  });
  expect_reject([](RodPendulumFixture& f) {
    f.circles[0].position = f.constraints.anchor_rods[0].world_anchor;
  });
  expect_reject([](RodPendulumFixture& f) {
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    f.constraints.link_rods[0].body_b.kind = static_cast<tiny2d::BodyKind>(2);
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.link_rods[0].body_b.index = 9;
  });
  expect_reject([](RodPendulumFixture& f) { f.circles[1].mass = 0.0f; });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.link_rods[0].body_b = f.constraints.link_rods[0].body_a;
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.revolute_pins = {{1, {50.0f, 54.0f}}};
  });
  expect_reject([](RodPendulumFixture& f) {
    f.constraints.link_rods[0].length = std::numeric_limits<float>::infinity();
  });
  expect_reject([](RodPendulumFixture& f) {
    f.circles[1].position = f.circles[0].position;
  });
}

void TestConstraintSetZeroDtReportsZeroRodForces() {
  RodPendulumFixture fixture = MakeRodPendulumFixture(2.0f, 0.0f);
  // A radial velocity violates the rod; zero dt still resolves it.
  fixture.circles[0].velocity = {0.0f, 1.0f};
  tiny2d::ConstraintReactions reactions;
  StepConstraintSet(fixture, 0.0f, &reactions);
  CHECK(std::abs(fixture.circles[0].velocity.y) <= 0.0001f);
  CHECK(reactions.anchor_rod_forces.size() == 1);
  CHECK(reactions.link_rod_forces.empty());
  CHECK(reactions.anchor_rod_forces[0] == 0.0f);
}

std::vector<tiny2d::Rectangle> MakeStackBoxes(int count, float offset) {
  std::vector<tiny2d::Rectangle> boxes(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    auto& box = boxes[static_cast<std::size_t>(i)];
    box.mass = 1.0f;
    box.width = 1.0f;
    box.height = 1.0f;
    box.position = {50.0f + ((i % 2 == 0) ? offset : -offset),
                    99.5f - static_cast<float>(i)};
    box.angular_damping_rate = 0.0f;
  }
  return boxes;
}

tiny2d::SolverSettings MakeStackSettings() {
  tiny2d::SolverSettings settings;
  settings.iterations = 16;
  settings.position_slop = 0.002f;
  return settings;
}

void StepStack(std::vector<tiny2d::Rectangle>& boxes,
               tiny2d::ContactCache* cache,
               const tiny2d::SolverSettings& settings) {
  std::vector<tiny2d::Circle> no_circles;
  tiny2d::Update(boxes, no_circles, tiny2d::ConstraintSet{}, settings, cache,
                 1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.6f, {}, 9.81f, 20.0f,
                 false, nullptr);
}

void TestFullControlOverloadMatchesConstraintSetOverload() {
  // Wrapper-consistency gate: the ConstraintSet overload and the
  // full-control overload with default settings and no cache must stay
  // bit-identical on the golden scene.
  std::vector<tiny2d::Rectangle> via_set = MakeGoldenRectangles();
  std::vector<tiny2d::Circle> via_set_circles = MakeGoldenCircles();
  std::vector<tiny2d::Rectangle> full = via_set;
  std::vector<tiny2d::Circle> full_circles = via_set_circles;
  const tiny2d::ConstraintSet no_constraints;

  for (int step = 0; step < 300; ++step) {
    tiny2d::Update(via_set, via_set_circles, no_constraints, 1.0f / 240.0f,
                   90.0f, 80.0f, 0.5f, 0.4f, {3.0f, -2.0f}, 98.1f, 20.0f, false,
                   nullptr);
    tiny2d::Update(full, full_circles, no_constraints, tiny2d::SolverSettings{},
                   nullptr, 1.0f / 240.0f, 90.0f, 80.0f, 0.5f, 0.4f,
                   {3.0f, -2.0f}, 98.1f, 20.0f, false, nullptr);
    for (std::size_t i = 0; i < via_set.size(); ++i) {
      CHECK(SameRectangle(via_set[i], full[i]));
    }
    for (std::size_t i = 0; i < via_set_circles.size(); ++i) {
      CHECK(SameCircle(via_set_circles[i], full_circles[i]));
    }
  }
}

void TestSolverSettingsMatchHistoricalConstantsBitwise() {
  // Explicit settings equal to the formerly hardcoded constants must
  // reproduce the default path bit for bit.
  std::vector<tiny2d::Rectangle> defaults = MakeGoldenRectangles();
  std::vector<tiny2d::Circle> defaults_circles = MakeGoldenCircles();
  std::vector<tiny2d::Rectangle> explicit_settings = defaults;
  std::vector<tiny2d::Circle> explicit_circles = defaults_circles;
  const tiny2d::ConstraintSet no_constraints;
  tiny2d::SolverSettings historical;
  historical.iterations = 4;
  historical.position_slop = 0.01f;
  historical.position_correction = 0.8f;

  for (int step = 0; step < 300; ++step) {
    tiny2d::Update(defaults, defaults_circles, no_constraints, 1.0f / 240.0f,
                   90.0f, 80.0f, 0.5f, 0.4f, {3.0f, -2.0f}, 98.1f, 20.0f, false,
                   nullptr);
    tiny2d::Update(explicit_settings, explicit_circles, no_constraints,
                   historical, nullptr, 1.0f / 240.0f, 90.0f, 80.0f, 0.5f, 0.4f,
                   {3.0f, -2.0f}, 98.1f, 20.0f, false, nullptr);
    for (std::size_t i = 0; i < defaults.size(); ++i) {
      CHECK(SameRectangle(defaults[i], explicit_settings[i]));
    }
  }
}

void TestWarmStartedStackRestsAndMatchesInterfaceLoads() {
  // The ROADMAP section 11 reference: a warm-started ten-box stack must truly
  // rest, deterministically, with per-interface loads matching the
  // supported weights (the analytical anchor).
  std::vector<tiny2d::Rectangle> first = MakeStackBoxes(10, 0.0f);
  std::vector<tiny2d::Rectangle> second = MakeStackBoxes(10, 0.0f);
  tiny2d::ContactCache first_cache;
  tiny2d::ContactCache second_cache;
  const tiny2d::SolverSettings settings = MakeStackSettings();

  std::array<double, 9> load_sums{};
  int load_samples = 0;
  for (int step = 1; step <= 20 * 480; ++step) {
    StepStack(first, &first_cache, settings);
    StepStack(second, &second_cache, settings);
    if (step > 15 * 480) {
      // Final 5 s: resting speeds and interface loads.
      for (const auto& box : first) {
        CHECK(std::hypot(box.velocity.x, box.velocity.y) < 0.001f);
        CHECK(std::abs(box.angular_velocity) < 0.001f);
      }
      for (const auto& entry : first_cache.entries) {
        const int index_a = static_cast<int>((entry.key >> 29) & 0x1FFFFF);
        const int index_b = static_cast<int>((entry.key >> 8) & 0x1FFFFF);
        if (index_b == index_a + 1 && index_a < 9) {
          load_sums[static_cast<std::size_t>(index_a)] +=
              entry.normal_impulse * 480.0;
        }
      }
      ++load_samples;
    }
  }
  // Penetration at rest stays inside the criterion.
  for (int i = 0; i + 1 < 10; ++i) {
    const double lower_top =
        first[static_cast<std::size_t>(i)].position.y - 0.5;
    const double upper_bottom =
        first[static_cast<std::size_t>(i + 1)].position.y + 0.5;
    CHECK(upper_bottom - lower_top < 0.005);
  }
  // Time-averaged interface loads match (n - k) m g within 1%.
  for (int i = 0; i < 9; ++i) {
    const double expected = (9 - i) * 9.81;
    const double measured =
        load_sums[static_cast<std::size_t>(i)] / load_samples;
    CHECK(std::abs(measured - expected) / expected < 0.01);
  }
  // Bitwise determinism including the cache.
  for (std::size_t i = 0; i < first.size(); ++i) {
    CHECK(SameRectangle(first[i], second[i]));
  }
  CHECK(first_cache.entries.size() == second_cache.entries.size());
  for (std::size_t i = 0; i < first_cache.entries.size(); ++i) {
    CHECK(first_cache.entries[i].key == second_cache.entries[i].key);
    CHECK(first_cache.entries[i].normal_impulse ==
          second_cache.entries[i].normal_impulse);
    CHECK(first_cache.entries[i].tangent_impulse ==
          second_cache.entries[i].tangent_impulse);
  }
}

void TestWarmStartMatchedFractionAndStaleEviction() {
  std::vector<tiny2d::Rectangle> boxes = MakeStackBoxes(4, 0.0f);
  tiny2d::ContactCache cache;
  const tiny2d::SolverSettings settings = MakeStackSettings();
  for (int step = 0; step < 480; ++step) {
    StepStack(boxes, &cache, settings);
  }
  // Settled: every contact point warm-starts from a cache hit.
  CHECK(cache.contact_count > 0);
  CHECK(cache.warm_started_count == cache.contact_count);
  const std::size_t settled_entries = cache.entries.size();

  // Teleport the top box away: its contact entries must evict.
  boxes[3].position = {20.0f, 50.0f};
  boxes[3].velocity = {};
  StepStack(boxes, &cache, settings);
  CHECK(cache.entries.size() < settled_entries);
}

void TestAccumulatedFrictionClampRespectsCone() {
  // A driven top box slides on the bottom box; the accumulated tangent
  // impulse must stay inside the sticking cone.
  std::vector<tiny2d::Rectangle> boxes = MakeStackBoxes(2, 0.0f);
  tiny2d::ContactCache cache;
  const tiny2d::SolverSettings settings = MakeStackSettings();
  for (int step = 0; step < 480; ++step) {
    tiny2d::AddForceAtPoint(boxes[1], {30.0f, 0.0f}, boxes[1].position);
    StepStack(boxes, &cache, settings);
    for (const auto& entry : cache.entries) {
      CHECK(std::abs(entry.tangent_impulse) <=
            0.6f * entry.normal_impulse + 0.000001f);
    }
  }
  // The driven box did slide.
  CHECK(boxes[1].position.x > 50.5f);
}

void TestWarmStartRestitutionStillBounces() {
  // A fast drop onto a static platform must still bounce with the cache
  // on: the approach velocity is captured before warm application.
  std::vector<tiny2d::Rectangle> bodies(2);
  bodies[0].mass = 0.0f;  // Static platform.
  bodies[0].position = {50.0f, 80.0f};
  bodies[0].width = 40.0f;
  bodies[0].height = 4.0f;
  bodies[0].material = {0.8f, 0.0f, 0.0f};
  bodies[1].mass = 1.0f;
  bodies[1].position = {50.0f, 70.0f};
  bodies[1].velocity = {0.0f, 30.0f};
  bodies[1].width = 2.0f;
  bodies[1].height = 2.0f;
  bodies[1].fixed_rotation = true;
  bodies[1].material = {0.8f, 0.0f, 0.0f};
  bodies[1].angular_damping_rate = 0.0f;
  std::vector<tiny2d::Circle> no_circles;
  tiny2d::ContactCache cache;
  tiny2d::SolverSettings settings;

  float minimum_velocity = 0.0f;
  for (int step = 0; step < 480; ++step) {
    tiny2d::Update(bodies, no_circles, tiny2d::ConstraintSet{}, settings,
                   &cache, 1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.0f, {}, 0.0f,
                   5.0f, false, nullptr);
    minimum_velocity = std::min(minimum_velocity, bodies[1].velocity.y);
  }
  // Rebound speed approaches restitution * approach speed (0.8 * ~30).
  CHECK(minimum_velocity < -20.0f);
}

void TestSolverSettingsAndCacheRejectInvalidInput() {
  const auto expect_reject = [](auto mutate) {
    std::vector<tiny2d::Rectangle> boxes = MakeStackBoxes(3, 0.0f);
    const std::vector<tiny2d::Rectangle> before = boxes;
    std::vector<tiny2d::Circle> no_circles;
    tiny2d::SolverSettings settings;
    tiny2d::ContactCache cache;
    cache.entries = {{100ull, 1.0f, 0.5f}, {200ull, 2.0f, -0.25f}};
    bool use_ccd = false;
    mutate(settings, cache, use_ccd);
    // Snapshot after the mutation: atomicity means the ENGINE leaves the
    // (possibly deliberately corrupted) cache exactly as passed.
    const std::vector<tiny2d::ContactCache::Entry> cache_before = cache.entries;
    bool threw = false;
    try {
      tiny2d::Update(boxes, no_circles, tiny2d::ConstraintSet{}, settings,
                     &cache, 1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.6f, {},
                     9.81f, 20.0f, use_ccd, nullptr);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
    // Failure atomicity: bodies and the cache stay untouched.
    for (std::size_t i = 0; i < before.size(); ++i) {
      CHECK(SameRectangle(boxes[i], before[i]));
    }
    CHECK(cache.entries.size() == cache_before.size());
    for (std::size_t i = 0; i < cache_before.size(); ++i) {
      CHECK(cache.entries[i].key == cache_before[i].key);
      CHECK(cache.entries[i].normal_impulse == cache_before[i].normal_impulse);
    }
  };

  expect_reject([](tiny2d::SolverSettings& s, tiny2d::ContactCache&, bool&) {
    s.iterations = 0;
  });
  expect_reject([](tiny2d::SolverSettings& s, tiny2d::ContactCache&, bool&) {
    s.iterations = 129;
  });
  expect_reject([](tiny2d::SolverSettings& s, tiny2d::ContactCache&, bool&) {
    s.position_slop = -0.01f;
  });
  expect_reject([](tiny2d::SolverSettings& s, tiny2d::ContactCache&, bool&) {
    s.position_slop = std::numeric_limits<float>::quiet_NaN();
  });
  expect_reject([](tiny2d::SolverSettings& s, tiny2d::ContactCache&, bool&) {
    s.position_correction = 1.5f;
  });
  expect_reject([](tiny2d::SolverSettings& s, tiny2d::ContactCache&, bool&) {
    s.position_correction = -0.1f;
  });
  expect_reject([](tiny2d::SolverSettings&, tiny2d::ContactCache& c, bool&) {
    c.entries[0].normal_impulse = -1.0f;
  });
  expect_reject([](tiny2d::SolverSettings&, tiny2d::ContactCache& c, bool&) {
    c.entries[1].tangent_impulse = std::numeric_limits<float>::infinity();
  });
  expect_reject([](tiny2d::SolverSettings&, tiny2d::ContactCache& c, bool&) {
    std::swap(c.entries[0], c.entries[1]);  // Unsorted keys.
  });
  expect_reject([](tiny2d::SolverSettings&, tiny2d::ContactCache& c, bool&) {
    c.entries[1].key = c.entries[0].key;  // Duplicate keys.
  });
  expect_reject([](tiny2d::SolverSettings&, tiny2d::ContactCache&, bool& ccd) {
    ccd = true;  // CCD cannot combine with a cache.
  });
}

void TestWarmCacheKeysStayDistinctAcrossPairKinds() {
  // Regression for the key-packing collision: circle-circle (i, j) and
  // rect-circle (i, j) pairs must produce distinct cache keys, or the
  // write-back stores duplicates and the next call rejects the engine's
  // own cache. Two balls resting on a box exercises both pair kinds with
  // identical index pairs.
  std::vector<tiny2d::Rectangle> rectangles(1);
  rectangles[0].mass = 5.0f;
  rectangles[0].position = {50.0f, 99.0f};
  rectangles[0].width = 10.0f;
  rectangles[0].height = 2.0f;
  rectangles[0].angular_damping_rate = 0.0f;
  std::vector<tiny2d::Circle> circles(2);
  circles[0].mass = 1.0f;
  circles[0].position = {49.0f, 97.0f};
  circles[0].radius = 1.0f;
  circles[0].angular_damping_rate = 0.0f;
  circles[1] = circles[0];
  circles[1].position = {50.9f, 97.0f};  // Touches circle 0 and the box.
  tiny2d::ContactCache cache;
  tiny2d::SolverSettings settings = MakeStackSettings();

  for (int step = 0; step < 240; ++step) {
    // A throw here (strictly-increasing key validation rejecting the
    // engine's own write-back) is exactly the regression this guards.
    tiny2d::Update(rectangles, circles, tiny2d::ConstraintSet{}, settings,
                   &cache, 1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.6f, {}, 9.81f,
                   20.0f, false, nullptr);
  }
  for (std::size_t i = 1; i < cache.entries.size(); ++i) {
    CHECK(cache.entries[i].key > cache.entries[i - 1].key);
  }
  CHECK(IsFinite(circles[0]));
  CHECK(IsFinite(circles[1]));
}

void TestWarmPathSkipsStaticStaticPairs() {
  // Regression for the static-static NaN: overlapping static bodies are
  // legal input, and the warm path must skip them like the cold path's
  // early-out instead of dividing by a zero impulse denominator.
  std::vector<tiny2d::Rectangle> rectangles(3);
  rectangles[0].mass = 0.0f;  // Two overlapping static floor slabs.
  rectangles[0].position = {45.0f, 95.0f};
  rectangles[0].width = 20.0f;
  rectangles[0].height = 4.0f;
  rectangles[1] = rectangles[0];
  rectangles[1].position = {55.0f, 95.0f};
  rectangles[2].mass = 1.0f;  // A dynamic box resting on them.
  rectangles[2].position = {50.0f, 92.4f};
  rectangles[2].width = 1.0f;
  rectangles[2].height = 1.0f;
  rectangles[2].angular_damping_rate = 0.0f;
  std::vector<tiny2d::Circle> no_circles;
  tiny2d::ContactCache cache;
  const tiny2d::SolverSettings settings = MakeStackSettings();

  for (int step = 0; step < 480; ++step) {
    tiny2d::Update(rectangles, no_circles, tiny2d::ConstraintSet{}, settings,
                   &cache, 1.0f / 480.0f, 100.0f, 100.0f, 0.0f, 0.6f, {}, 9.81f,
                   20.0f, false, nullptr);
  }
  for (const auto& rectangle : rectangles) {
    CHECK(IsFinite(rectangle));
  }
  for (const auto& entry : cache.entries) {
    CHECK(std::isfinite(entry.normal_impulse));
    CHECK(std::isfinite(entry.tangent_impulse));
  }
}

void TestFullControlZeroDtLeavesCacheUntouched() {
  std::vector<tiny2d::Rectangle> boxes = MakeStackBoxes(3, 0.0f);
  tiny2d::ContactCache cache;
  const tiny2d::SolverSettings settings = MakeStackSettings();
  for (int step = 0; step < 240; ++step) {
    StepStack(boxes, &cache, settings);
  }
  const std::vector<tiny2d::ContactCache::Entry> entries_before = cache.entries;
  const int contacts_before = cache.contact_count;
  const int warm_before = cache.warm_started_count;

  std::vector<tiny2d::Circle> no_circles;
  tiny2d::Update(boxes, no_circles, tiny2d::ConstraintSet{}, settings, &cache,
                 0.0f, 100.0f, 100.0f, 0.0f, 0.6f, {}, 9.81f, 20.0f, false,
                 nullptr);

  CHECK(cache.contact_count == contacts_before);
  CHECK(cache.warm_started_count == warm_before);
  CHECK(cache.entries.size() == entries_before.size());
  for (std::size_t i = 0; i < entries_before.size(); ++i) {
    CHECK(cache.entries[i].key == entries_before[i].key);
    CHECK(cache.entries[i].normal_impulse == entries_before[i].normal_impulse);
    CHECK(cache.entries[i].tangent_impulse ==
          entries_before[i].tangent_impulse);
  }
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
  TestGoldenMixedTrajectoryCheckpoints();
  TestMixedUpdateMatchesConstrainedUpdateWithoutConstraints();
  TestRevolutePinHoldsCircleUnderGravity();
  TestPulleyRopeMatchesAtwoodAcceleration();
  TestPulleyRopeNoSlipCouplingAndOppositeSpeeds();
  TestPulleyRopeLengthDriftStaysBounded();
  TestPulleyRopeWithFloorContactStaysFinite();
  TestConstrainedUpdateRejectsInvalidInput();
  TestConstrainedUpdateZeroDtReportsZeroReactions();
  TestConstraintSetOverloadMatchesPinsRopesOverload();
  TestAnchorRodPendulumMatchesAnalyticalPeriod();
  TestLinkRodKeepsDistanceAndReportsCompression();
  TestAnchorRodWithFloorContactStaysFinite();
  TestRodAndRopeShareBodyStaysFiniteAndDeterministic();
  TestConstrainedUpdateRejectsInvalidRods();
  TestConstraintSetZeroDtReportsZeroRodForces();
  TestFullControlOverloadMatchesConstraintSetOverload();
  TestSolverSettingsMatchHistoricalConstantsBitwise();
  TestWarmStartedStackRestsAndMatchesInterfaceLoads();
  TestWarmStartMatchedFractionAndStaleEviction();
  TestAccumulatedFrictionClampRespectsCone();
  TestWarmStartRestitutionStillBounces();
  TestSolverSettingsAndCacheRejectInvalidInput();
  TestWarmCacheKeysStayDistinctAcrossPairKinds();
  TestWarmPathSkipsStaticStaticPairs();
  TestFullControlZeroDtLeavesCacheUntouched();
  return 0;
}
