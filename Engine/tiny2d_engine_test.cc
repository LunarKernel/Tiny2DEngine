#include "tiny2d_engine.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

constexpr float kTolerance = 0.001f;

float Distance(tiny2d::Vec2 a, tiny2d::Vec2 b) {
  const float delta_x = b.x - a.x;
  const float delta_y = b.y - a.y;
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

void TestRotatedVerticesKeepRectangleSize() {
  const tiny2d::Rectangle rectangle{1.0f, {100.0f, 80.0f}, {},   0.7f,
                                    0.0f, 80.0f,           20.0f};
  const auto vertices = tiny2d::GetVertices(rectangle);
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const std::size_t next = (i + 1) % vertices.size();
    const float expected_length =
        i % 2 == 0 ? rectangle.width : rectangle.height;
    assert(std::abs(Distance(vertices[i], vertices[next]) - expected_length) <
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
  assert(tiny2d::IsColliding(rectangle_a, rectangle_b));
  assert(!tiny2d::IsColliding(rectangle_a, separated));
}

void TestCenteredCollisionDoesNotCreateRotation() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {1.0f, {139.0f, 100.0f}, {-10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(squares[0].angular_velocity) < kTolerance);
  assert(std::abs(squares[1].angular_velocity) < kTolerance);
}

void TestOffCenterCollisionCreatesRotation() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 60.0f, 20.0f},
      {1.0f, {159.0f, 107.0f}, {-10.0f, 0.0f}, 0.0f, 0.0f, 60.0f, 20.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(squares[0].angular_velocity) > kTolerance);
  assert(std::abs(squares[1].angular_velocity) > kTolerance);
}

void TestAngularDampingReducesRotation() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {}, 0.0f, 10.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.5f, 1000.0f, 1000.0f, 1.0f);
  assert(squares[0].angular_velocity > 0.0f);
  assert(squares[0].angular_velocity < 10.0f);
}

void TestElectricFieldUsesChargeAndMass() {
  std::vector<tiny2d::Rectangle> rectangles = {
      {2.0f, {100.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, 3.0f},
      {4.0f, {200.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, -3.0f},
      {0.0f, {300.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f, false, 3.0f},
      {2.0f, {400.0f, 100.0f}, {}, 0.0f, 0.0f, 20.0f, 20.0f},
  };

  tiny2d::Update(rectangles, 0.5f, 1000.0f, 1000.0f, 0.0f, 0.0f, {4.0f, -8.0f});

  assert(std::abs(rectangles[0].velocity.x - 3.0f) < kTolerance);
  assert(std::abs(rectangles[0].velocity.y - 43.05f) < kTolerance);
  assert(std::abs(rectangles[1].velocity.x + 1.5f) < kTolerance);
  assert(std::abs(rectangles[1].velocity.y - 52.05f) < kTolerance);
  assert(std::abs(rectangles[2].velocity.x) < kTolerance);
  assert(std::abs(rectangles[2].velocity.y) < kTolerance);
  assert(std::abs(rectangles[3].velocity.x) < kTolerance);
  assert(std::abs(rectangles[3].velocity.y - 49.05f) < kTolerance);
}

void TestConfigurableWallFriction() {
  std::vector<tiny2d::Rectangle> no_friction = {
      {1.0f, {100.0f, 19.0f}, {10.0f, -10.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  std::vector<tiny2d::Rectangle> with_friction = no_friction;
  tiny2d::Update(no_friction, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.0f);
  tiny2d::Update(with_friction, 0.0f, 1000.0f, 1000.0f, 1.0f, 0.4f);
  assert(std::abs(no_friction[0].velocity.x - 10.0f) < kTolerance);
  assert(std::abs(with_friction[0].velocity.x) <
         std::abs(no_friction[0].velocity.x));
}

void TestStaticRectangleDoesNotMove() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {0.0f, {139.0f, 115.0f}, {}, -0.35f, 0.0f, 40.0f, 40.0f},
  };
  const tiny2d::Rectangle fixed_square = squares[1];
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(squares[1].position.x == fixed_square.position.x);
  assert(squares[1].position.y == fixed_square.position.y);
  assert(squares[1].angle == fixed_square.angle);
}

void TestRestitutionRequiresImpactSpeed() {
  std::vector<tiny2d::Rectangle> slow_collision = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {0.0f, {139.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(slow_collision, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(slow_collision[0].velocity.x) < kTolerance);

  std::vector<tiny2d::Rectangle> fast_collision = {
      {1.0f, {100.0f, 100.0f}, {100.0f, 0.0f}, 0.0f, 0.0f, 40.0f, 40.0f},
      {0.0f, {139.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f, 40.0f},
  };
  tiny2d::Update(fast_collision, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(fast_collision[0].velocity.x < -90.0f);
  assert(std::abs(fast_collision[0].angular_velocity) < kTolerance);
}

void TestFixedRotationStaysLocked() {
  std::vector<tiny2d::Rectangle> squares = {
      {1.0f, {100.0f, 100.0f}, {100.0f, 0.0f}, 0.0f, 5.0f, 40.0f, 40.0f, true},
      {1.0f, {139.0f, 100.0f}, {}, 0.0f, -5.0f, 40.0f, 40.0f, true},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(squares[0].angle == 0.0f);
  assert(squares[1].angle == 0.0f);
  assert(squares[0].angular_velocity == 0.0f);
  assert(squares[1].angular_velocity == 0.0f);
  assert(squares[1].velocity.x > 90.0f);
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

  assert(collision_observed);
  assert(squares[0].angular_velocity == 0.0f);
  assert(squares[1].angular_velocity == 0.0f);
}

}  // namespace

int main() {
  TestRotatedVerticesKeepRectangleSize();
  TestSatDetectsRotatedCollision();
  TestCenteredCollisionDoesNotCreateRotation();
  TestOffCenterCollisionCreatesRotation();
  TestAngularDampingReducesRotation();
  TestElectricFieldUsesChargeAndMass();
  TestConfigurableWallFriction();
  TestStaticRectangleDoesNotMove();
  TestRestitutionRequiresImpactSpeed();
  TestFixedRotationStaysLocked();
  TestLockedBlocksCollideOnRamp();
  return 0;
}
