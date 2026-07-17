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

void TestRotatedVerticesKeepSquareSize() {
  const tiny2d::Square square{1.0f, {100.0f, 80.0f}, {}, 0.7f, 0.0f, 40.0f};
  const auto vertices = tiny2d::GetVertices(square);
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const std::size_t next = (i + 1) % vertices.size();
    assert(std::abs(Distance(vertices[i], vertices[next]) - square.size) <
           kTolerance);
  }
}

void TestSatDetectsRotatedCollision() {
  const tiny2d::Square square_a{1.0f, {100.0f, 100.0f}, {}, 0.4f, 0.0f, 40.0f};
  const tiny2d::Square square_b{1.0f, {130.0f, 100.0f}, {}, -0.2f, 0.0f, 40.0f};
  const tiny2d::Square separated{1.0f, {180.0f, 100.0f}, {}, -0.2f, 0.0f,
                                 40.0f};
  assert(tiny2d::IsColliding(square_a, square_b));
  assert(!tiny2d::IsColliding(square_a, separated));
}

void TestCenteredCollisionDoesNotCreateRotation() {
  std::vector<tiny2d::Square> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
      {1.0f, {139.0f, 100.0f}, {-10.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(squares[0].angular_velocity) < kTolerance);
  assert(std::abs(squares[1].angular_velocity) < kTolerance);
}

void TestOffCenterCollisionCreatesRotation() {
  std::vector<tiny2d::Square> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
      {1.0f, {139.0f, 115.0f}, {-10.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(squares[0].angular_velocity) > kTolerance);
  assert(std::abs(squares[1].angular_velocity) > kTolerance);
}

void TestAngularDampingReducesRotation() {
  std::vector<tiny2d::Square> squares = {
      {1.0f, {100.0f, 100.0f}, {}, 0.0f, 10.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.5f, 1000.0f, 1000.0f, 1.0f);
  assert(squares[0].angular_velocity > 0.0f);
  assert(squares[0].angular_velocity < 10.0f);
}

void TestWallFrictionReducesTangentialSpeed() {
  std::vector<tiny2d::Square> squares = {
      {1.0f, {100.0f, 19.0f}, {10.0f, -10.0f}, 0.0f, 0.0f, 40.0f},
  };
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(squares[0].velocity.x) < 10.0f);
}

void TestStaticSquareDoesNotMove() {
  std::vector<tiny2d::Square> squares = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
      {0.0f, {139.0f, 115.0f}, {}, -0.35f, 0.0f, 40.0f},
  };
  const tiny2d::Square fixed_square = squares[1];
  tiny2d::Update(squares, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(squares[1].position.x == fixed_square.position.x);
  assert(squares[1].position.y == fixed_square.position.y);
  assert(squares[1].angle == fixed_square.angle);
}

void TestRestitutionRequiresImpactSpeed() {
  std::vector<tiny2d::Square> slow_collision = {
      {1.0f, {100.0f, 100.0f}, {10.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
      {0.0f, {139.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f},
  };
  tiny2d::Update(slow_collision, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(std::abs(slow_collision[0].velocity.x) < kTolerance);

  std::vector<tiny2d::Square> fast_collision = {
      {1.0f, {100.0f, 100.0f}, {100.0f, 0.0f}, 0.0f, 0.0f, 40.0f},
      {0.0f, {139.0f, 100.0f}, {}, 0.0f, 0.0f, 40.0f},
  };
  tiny2d::Update(fast_collision, 0.0f, 1000.0f, 1000.0f, 1.0f);
  assert(fast_collision[0].velocity.x < -90.0f);
  assert(std::abs(fast_collision[0].angular_velocity) < kTolerance);
}

}  // namespace

int main() {
  TestRotatedVerticesKeepSquareSize();
  TestSatDetectsRotatedCollision();
  TestCenteredCollisionDoesNotCreateRotation();
  TestOffCenterCollisionCreatesRotation();
  TestAngularDampingReducesRotation();
  TestWallFrictionReducesTangentialSpeed();
  TestStaticSquareDoesNotMove();
  TestRestitutionRequiresImpactSpeed();
  return 0;
}
