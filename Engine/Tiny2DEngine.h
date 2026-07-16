#pragma once

namespace tiny2d {

    struct Square {
		float mass{ 1.0f };
        float x{};
        float y{};
        float vx{};
        float vy{};
        float size{ 40.0f };
    };

    void gravity(Square& square, float g, float dt);

    void update(
        Square& square1,
        Square& square2,
        float dt,
        float area_width,
        float area_height,
        float coefficient
    );

    void collision_with_window(
        Square& square,
        float area_width, float area_height,
        float coefficient);

    void collision_of_two_squares(
        Square& square1, Square& square2,
        float coefficient);

    void position(Square& square, float dt);
}
