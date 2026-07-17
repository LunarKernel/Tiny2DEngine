#ifndef TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
#define TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_

#include <array>
#include <vector>

namespace tiny2d {

struct Vec2 {
  float x{};
  float y{};
};

struct Rectangle {
  float mass{1.0f};
  Vec2 position{};
  Vec2 velocity{};
  float angle{};
  float angular_velocity{};
  float width{40.0f};
  float height{40.0f};
  bool fixed_rotation{};
  float charge{};
};

Vec2 GetLinearAcceleration(const Rectangle& rectangle, Vec2 electric_field);

std::array<Vec2, 4> GetVertices(const Rectangle& rectangle);

bool IsColliding(const Rectangle& rectangle_a, const Rectangle& rectangle_b);

void Update(std::vector<Rectangle>& rectangles, float delta_time,
            float area_width, float area_height, float restitution,
            float friction = 0.4f, Vec2 electric_field = {});

}  // namespace tiny2d

#endif  // TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
