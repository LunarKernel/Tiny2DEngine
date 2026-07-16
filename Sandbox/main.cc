#include <SDL.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

#include "tiny2d_engine.h"

namespace {

constexpr std::size_t kMaxSquares = 8;
constexpr int kMaxSpawnAttempts = 100;
constexpr float kSquareSize = 40.0f;

void TrySpawnSquare(std::vector<tiny2d::Square>& squares, int area_width,
                    int area_height, std::mt19937& random_engine) {
  std::uniform_real_distribution<float> x_distribution(
      0.0f, static_cast<float>(area_width) - kSquareSize);
  std::uniform_real_distribution<float> y_distribution(
      0.0f, static_cast<float>(area_height) - kSquareSize);

  for (int attempt = 0; attempt < kMaxSpawnAttempts; ++attempt) {
    tiny2d::Square candidate{1.0f,
                             x_distribution(random_engine),
                             y_distribution(random_engine),
                             0.0f,
                             0.0f,
                             kSquareSize};

    bool overlaps = false;
    for (const tiny2d::Square& square : squares) {
      if (tiny2d::IsColliding(candidate, square)) {
        overlaps = true;
        break;
      }
    }

    if (!overlaps) {
      squares.push_back(candidate);
      return;
    }
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  constexpr int kAreaWidth = 800;
  constexpr int kAreaHeight = 600;

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
    std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow("Tiny2D Engine", SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED, kAreaWidth,
                                        kAreaHeight, SDL_WINDOW_SHOWN);

  if (!window) {
    std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
    SDL_Quit();
    return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if (!renderer) {
    std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << '\n';
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  std::vector<tiny2d::Square> squares = {
      {1.0f, 100.0f, 100.0f, 180.0f, 0.0f, 40.0f},
      {1.0f, 500.0f, 100.0f, -120.0f, 0.0f, 40.0f},
  };
  squares.reserve(kMaxSquares);

  std::mt19937 random_engine{std::random_device{}()};

  bool running = true;
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();

  while (running) {
    // 1. Process events.
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      }

      if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE &&
          event.key.repeat == 0 && squares.size() < kMaxSquares) {
        TrySpawnSquare(squares, kAreaWidth, kAreaHeight, random_engine);
        assert(squares.size() <= kMaxSquares);
      }
    }

    // 2. Get delta time.
    const Uint64 current_time = SDL_GetPerformanceCounter();
    float delta_time =
        static_cast<float>((current_time - previous_time) / frequency);
    previous_time = current_time;
    delta_time = std::min(delta_time, 0.05f);

    // 3. Update square positions.
    tiny2d::Update(squares, delta_time, static_cast<float>(kAreaWidth),
                   static_cast<float>(kAreaHeight), 0.95f);

    // 4. Clear the screen.
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    // 5. Draw the squares.
    for (const tiny2d::Square& square : squares) {
      SDL_FRect rect{square.x, square.y, square.size, square.size};
      SDL_SetRenderDrawColor(renderer, 255, 100, 100, 255);
      SDL_RenderFillRectF(renderer, &rect);
    }

    // 6. Present the rendered frame.
    SDL_RenderPresent(renderer);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
