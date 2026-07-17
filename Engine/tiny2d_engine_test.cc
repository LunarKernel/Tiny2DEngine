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
         std::isfinite(rectangle.charge);
}

bool SameFloat(float a, float b) {
  return a == b || (std::isnan(a) && std::isnan(b));
}

bool SameRectangle(const tiny2d::Rectangle& a, const tiny2d::Rectangle& b) {
  return SameFloat(a.mass, b.mass) && SameFloat(a.position.x, b.position.x) &&
         SameFloat(a.position.y, b.position.y) &&
         SameFloat(a.velocity.x, b.velocity.x) &&
         SameFloat(a.velocity.y, b.velocity.y) && SameFloat(a.angle, b.angle) &&
         SameFloat(a.angular_velocity, b.angular_velocity) &&
         SameFloat(a.width, b.width) && SameFloat(a.height, b.height) &&
         a.fixed_rotation == b.fixed_rotation && SameFloat(a.charge, b.charge);
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
  CHECK(squares[0].angular_velocity > 0.0f);
  CHECK(squares[0].angular_velocity < 10.0f);
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
  CheckInvalidArgument(
      [&] { tiny2d::GetLinearAcceleration(invalid, {}, 98.1f); });
  CheckInvalidArgument([&] {
    tiny2d::GetLinearAcceleration(
        valid, {std::numeric_limits<float>::quiet_NaN(), 0.0f}, 98.1f);
  });
}

}  // namespace

int main() {
  TestRotatedVerticesKeepRectangleSize();
  TestSatDetectsRotatedCollision();
  TestCenteredCollisionDoesNotCreateRotation();
  TestOffCenterCollisionCreatesRotation();
  TestAngularDampingReducesRotation();
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
  TestRejectsImpossibleAndOverflowingStates();
  TestPublicFunctionsRejectInvalidRectangles();
  return 0;
}
