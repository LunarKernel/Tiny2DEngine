#ifndef TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
#define TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_

#include <array>
#include <vector>

namespace tiny2d {

struct Vec2 {
  float x{};
  float y{};
};

struct Square {
  float mass{1.0f};
  Vec2 position{};
  Vec2 velocity{};
  float angle{};
  float angular_velocity{};
  float size{40.0f};
};

std::array<Vec2, 4> GetVertices(const Square& square);

bool IsColliding(const Square& square_a, const Square& square_b);

void Update(std::vector<Square>& squares, float delta_time, float area_width,
            float area_height, float coefficient);

}  // namespace tiny2d

#endif  // TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
