#include <SDL.h>
#include <algorithm>
#include <cassert>
#include <iostream>

#include "Tiny2DEngine.h"

int main(int argc, char* argv[]) {
    constexpr int areaWidth = 800;
    constexpr int areaHeight = 600;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Tiny2D Engine",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        areaWidth,
        areaHeight,
        SDL_WINDOW_SHOWN
    );

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: "
            << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED |
        SDL_RENDERER_PRESENTVSYNC
    );

    if (!renderer) {
        std::cerr << "SDL_CreateRenderer failed: "
            << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    tiny2d::Square square1{
        1.0f,
        100.0f, 100.0f, // x, y
        180.0f, 0.0f,   // vx, vy
        40.0f           // size
    };

    tiny2d::Square square2{
        2.0f,
        500.0f, 100.0f, // x, y
        -120.0f, 0.0f,  // vx, vy
        40.0f           // size
    };
    bool running = true;

    const double frequency =
        static_cast<double>(SDL_GetPerformanceFrequency());

    Uint64 previousTime = SDL_GetPerformanceCounter();

    while (running) {
		// 1. deal with events
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

		// 2.get delta time
        const Uint64 currentTime = SDL_GetPerformanceCounter();

        float dt = static_cast<float>(
            (currentTime - previousTime) / frequency
            );

        previousTime = currentTime;
        dt = std::min(dt, 0.05f);

		// 3. update square position
        tiny2d::update(
            square1,
            square2,
            dt,
            static_cast<float>(areaWidth),
            static_cast<float>(areaHeight),
            0.95f
        );

        assert(square1.x >= 0.0f && square1.x + square1.size <= areaWidth);
        assert(square1.y >= 0.0f && square1.y + square1.size <= areaHeight);
        assert(square2.x >= 0.0f && square2.x + square2.size <= areaWidth);
        assert(square2.y >= 0.0f && square2.y + square2.size <= areaHeight);

		// 4. clear the screen
        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);

		// 5. paint the square
        SDL_FRect squareRect1{
            square1.x,
            square1.y,
            square1.size,
            square1.size
        };

        SDL_FRect squareRect2{
            square2.x,
            square2.y,
            square2.size,
            square2.size
        };

        SDL_SetRenderDrawColor(renderer, 80, 180, 255, 255);
        SDL_RenderFillRectF(renderer, &squareRect1);

        SDL_SetRenderDrawColor(renderer, 255, 160, 60, 255);
        SDL_RenderFillRectF(renderer, &squareRect2);

		// 6. display the rendered frame
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
