#include "Tiny2DEngine.h"
namespace tiny2d {
    void update(
        Square& square,
        float dt,
        float areaWidth,
        float areaHeight
    ){

        float gravity = 980.0f;
        square.vy += gravity * dt; // Gravity

        square.x += square.vx * dt;
        square.y += square.vy * dt;

        if (square.x < 0.0f) {
            square.x = 0.0f;
            square.vx = -square.vx;
        }
        else if (square.x + square.size > 800.0f) {
            square.x = 800.0f - square.size;
            square.vx = -square.vx;
        }
        if (square.y < 0.0f) {
            square.y = 0.0f;
            square.vy = -square.vy;
        }
        else if (square.y + square.size > 600.0f) {
            square.y = 600.0f - square.size;
            square.vy = -square.vy;
        }
    }
}