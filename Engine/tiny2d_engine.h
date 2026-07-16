#ifndef TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
#define TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_

#include <vector>

namespace tiny2d {

struct Square {
  float mass{1.0f};
  float x{};
  float y{};
  float vx{};
  float vy{};
  float size{40.0f};
};

void ApplyGravity(Square& square, float gravity, float delta_time);

void UpdatePosition(Square& square, float delta_time);

bool IsColliding(const Square& square_a, const Square& square_b);

void ResolveWindowCollision(Square& square, float area_width, float area_height,
                            float coefficient);

void ResolveSquareCollision(Square& square_a, Square& square_b,
                            float coefficient);

void Update(std::vector<Square>& squares, float delta_time, float area_width,
            float area_height, float coefficient);

}  // namespace tiny2d

#endif  // TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
