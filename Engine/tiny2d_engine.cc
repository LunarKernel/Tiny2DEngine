#include "tiny2d_engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tiny2d {

void ApplyGravity(Square& square, float gravity, float delta_time) {
  square.vy += gravity * delta_time;
}

void UpdatePosition(Square& square, float delta_time) {
  square.x += square.vx * delta_time;
  square.y += square.vy * delta_time;
}

bool IsColliding(const Square& square_a, const Square& square_b) {
  return !(square_a.x + square_a.size < square_b.x ||
           square_b.x + square_b.size < square_a.x ||
           square_a.y + square_a.size < square_b.y ||
           square_b.y + square_b.size < square_a.y);
}

void ResolveWindowCollision(Square& square, float area_width, float area_height,
                            float coefficient) {
  const float bounce = std::clamp(coefficient, 0.0f, 1.0f);

  if (square.x < 0.0f) {
    square.x = 0.0f;
    square.vx = std::abs(square.vx) * bounce;
  } else if (square.x + square.size > area_width) {
    square.x = area_width - square.size;
    square.vx = -std::abs(square.vx) * bounce;
  }

  if (square.y < 0.0f) {
    square.y = 0.0f;
    square.vy = std::abs(square.vy) * bounce;
  } else if (square.y + square.size > area_height) {
    square.y = area_height - square.size;
    square.vy = -std::abs(square.vy) * bounce;
  }
}

void ResolveSquareCollision(Square& square_a, Square& square_b,
                            float coefficient) {
  const float overlap_x =
      std::min(square_a.x + square_a.size, square_b.x + square_b.size) -
      std::max(square_a.x, square_b.x);
  const float overlap_y =
      std::min(square_a.y + square_a.size, square_b.y + square_b.size) -
      std::max(square_a.y, square_b.y);

  if (overlap_x <= 0.0f || overlap_y <= 0.0f) {
    return;
  }

  const float inverse_mass_a =
      square_a.mass > 0.0f ? 1.0f / square_a.mass : 0.0f;
  const float inverse_mass_b =
      square_b.mass > 0.0f ? 1.0f / square_b.mass : 0.0f;
  const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;

  if (inverse_mass_sum == 0.0f) {
    return;
  }

  const float bounce = std::clamp(coefficient, 0.0f, 1.0f);

  if (overlap_x < overlap_y) {
    const float normal =
        square_a.x + square_a.size * 0.5f < square_b.x + square_b.size * 0.5f
            ? 1.0f
            : -1.0f;

    square_a.x -= normal * overlap_x * inverse_mass_a / inverse_mass_sum;
    square_b.x += normal * overlap_x * inverse_mass_b / inverse_mass_sum;

    const float relative_velocity = (square_b.vx - square_a.vx) * normal;
    if (relative_velocity < 0.0f) {
      const float impulse =
          -(1.0f + bounce) * relative_velocity / inverse_mass_sum;
      square_a.vx -= impulse * inverse_mass_a * normal;
      square_b.vx += impulse * inverse_mass_b * normal;
    }
  } else {
    const float normal =
        square_a.y + square_a.size * 0.5f < square_b.y + square_b.size * 0.5f
            ? 1.0f
            : -1.0f;

    square_a.y -= normal * overlap_y * inverse_mass_a / inverse_mass_sum;
    square_b.y += normal * overlap_y * inverse_mass_b / inverse_mass_sum;

    const float relative_velocity = (square_b.vy - square_a.vy) * normal;
    if (relative_velocity < 0.0f) {
      const float impulse =
          -(1.0f + bounce) * relative_velocity / inverse_mass_sum;
      square_a.vy -= impulse * inverse_mass_a * normal;
      square_b.vy += impulse * inverse_mass_b * normal;
    }
  }
}

void Update(std::vector<Square>& squares, float delta_time, float area_width,
            float area_height, float coefficient) {
  for (Square& square : squares) {
    ApplyGravity(square, 98.1f, delta_time);
    UpdatePosition(square, delta_time);
  }

  for (Square& square : squares) {
    ResolveWindowCollision(square, area_width, area_height, coefficient);
  }

  for (std::size_t i = 0; i < squares.size(); ++i) {
    for (std::size_t j = i + 1; j < squares.size(); ++j) {
      ResolveSquareCollision(squares[i], squares[j], coefficient);
    }
  }
}

}  // namespace tiny2d
