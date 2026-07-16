#include "Tiny2DEngine.h"

#include <algorithm>
#include <cmath>

namespace tiny2d {
    void gravity(Square& square, float g, float dt) {
        square.vy += g * dt;
    }

    void collision_with_window(
        Square& square,
        float area_width, float area_height,
        float coefficient) {

        const float bounce = std::clamp(coefficient, 0.0f, 1.0f);

        if (square.x < 0.0f) {
            square.x = 0.0f;
            square.vx = std::abs(square.vx) * bounce;
        }
        else if (square.x + square.size > area_width) {
            square.x = area_width - square.size;
            square.vx = -std::abs(square.vx) * bounce;
        }
        if (square.y < 0.0f) {
            square.y = 0.0f;
            square.vy = std::abs(square.vy) * bounce;
        }
        else if (square.y + square.size > area_height) {
            square.y = area_height - square.size;
            square.vy = -std::abs(square.vy) * bounce;
        }
    }

    void collision_of_two_squares(
        Square& square1, Square& square2,
        float coefficient) {

        const float overlap_x = std::min(square1.x + square1.size, square2.x + square2.size)
            - std::max(square1.x, square2.x);
        const float overlap_y = std::min(square1.y + square1.size, square2.y + square2.size)
            - std::max(square1.y, square2.y);

        if (overlap_x <= 0.0f || overlap_y <= 0.0f) {
            return;
        }

        const float inverse_mass1 = square1.mass > 0.0f ? 1.0f / square1.mass : 0.0f;
        const float inverse_mass2 = square2.mass > 0.0f ? 1.0f / square2.mass : 0.0f;
        const float inverse_mass_sum = inverse_mass1 + inverse_mass2;

        if (inverse_mass_sum == 0.0f) {
            return;
        }

        const float bounce = std::clamp(coefficient, 0.0f, 1.0f);

        if (overlap_x < overlap_y) {
            const float normal = square1.x + square1.size * 0.5f
                < square2.x + square2.size * 0.5f ? 1.0f : -1.0f;

            square1.x -= normal * overlap_x * inverse_mass1 / inverse_mass_sum;
            square2.x += normal * overlap_x * inverse_mass2 / inverse_mass_sum;

            const float relative_velocity = (square2.vx - square1.vx) * normal;
            if (relative_velocity < 0.0f) {
                const float impulse = -(1.0f + bounce) * relative_velocity / inverse_mass_sum;
                square1.vx -= impulse * inverse_mass1 * normal;
                square2.vx += impulse * inverse_mass2 * normal;
            }
        }
        else {
            const float normal = square1.y + square1.size * 0.5f
                < square2.y + square2.size * 0.5f ? 1.0f : -1.0f;

            square1.y -= normal * overlap_y * inverse_mass1 / inverse_mass_sum;
            square2.y += normal * overlap_y * inverse_mass2 / inverse_mass_sum;

            const float relative_velocity = (square2.vy - square1.vy) * normal;
            if (relative_velocity < 0.0f) {
                const float impulse = -(1.0f + bounce) * relative_velocity / inverse_mass_sum;
                square1.vy -= impulse * inverse_mass1 * normal;
                square2.vy += impulse * inverse_mass2 * normal;
            }
        }
    }

    void position(Square& square, float dt) {
        square.x += square.vx * dt;
        square.y += square.vy * dt;
    }

    void update(
        Square& square1,
		Square& square2,
        float dt,
        float area_width,
        float area_height,
        float coefficient
    ){
        gravity(square1, 980.0f, dt);
        gravity(square2, 980.0f, dt);
        position(square1, dt);
        position(square2, dt);
        collision_of_two_squares(square1, square2, coefficient);
        collision_with_window(square1, area_width, area_height, coefficient);
        collision_with_window(square2, area_width, area_height, coefficient);
    }
}
