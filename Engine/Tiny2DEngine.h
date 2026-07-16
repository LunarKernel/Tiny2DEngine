#pragma once

namespace tiny2d {

    struct Square {
        float x{};
        float y{};
        float vx{};
        float vy{};
        float size{ 40.0f };
    };

    void update(
        Square& square,
        float dt,
        float areaWidth,
        float areaHeight
    );

}
