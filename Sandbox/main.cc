#include <SDL.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "tiny2d_engine.h"

namespace {

constexpr int kAreaWidth = 800;
constexpr int kAreaHeight = 600;
constexpr float kBlockSize = 40.0f;
constexpr float kRampSize = 600.0f;
constexpr float kRampAngle = -0.5235987756f;
constexpr float kPhysicsStep = 1.0f / 120.0f;
constexpr double kMaxFrameTime = 0.25;
constexpr std::array<int, 6> kSquareIndices = {0, 1, 2, 0, 2, 3};

struct BlockConfig {
  float surface_x;
  float mass;
  float downhill_speed;
};

struct SimulationConfig {
  float friction{0.4f};
  float restitution{0.95f};
  BlockConfig upper_block{720.0f, 1.0f, 220.0f};
  BlockConfig lower_block{480.0f, 1.0f, 0.0f};
};

float ReadFloat(std::string_view prompt, float default_value,
                float minimum_value, float maximum_value) {
  while (true) {
    std::cout << prompt << " [" << default_value << "] (" << minimum_value
              << " to " << maximum_value << "): ";
    std::string line;
    if (!std::getline(std::cin, line) || line.empty()) {
      return default_value;
    }

    std::istringstream input(line);
    float value = 0.0f;
    char extra = '\0';
    if ((input >> value) && !(input >> extra) && std::isfinite(value) &&
        value >= minimum_value && value <= maximum_value) {
      return value;
    }
    std::cout << "Invalid value. Try again.\n";
  }
}

BlockConfig ReadBlockConfig(std::string_view name, BlockConfig defaults,
                            float minimum_x, float maximum_x) {
  std::cout << '\n' << name << " parameters:\n";
  defaults.surface_x =
      ReadFloat("  Ramp position X (smaller is closer to the bottom)",
                defaults.surface_x, minimum_x, maximum_x);
  defaults.mass = ReadFloat("  Mass", defaults.mass, 0.01f, 1000.0f);
  defaults.downhill_speed =
      ReadFloat("  Initial speed (positive is downhill)",
                defaults.downhill_speed, -1000.0f, 1000.0f);
  return defaults;
}

tiny2d::Square CreateBlockOnRamp(const BlockConfig& config) {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float surface_y =
      static_cast<float>(kAreaHeight) +
      (config.surface_x - ramp_bottom_x) * std::tan(kRampAngle);
  const float half_size = kBlockSize * 0.5f;
  const tiny2d::Vec2 position{
      config.surface_x + std::sin(kRampAngle) * half_size,
      surface_y - std::cos(kRampAngle) * half_size};
  const tiny2d::Vec2 velocity{-std::cos(kRampAngle) * config.downhill_speed,
                              -std::sin(kRampAngle) * config.downhill_speed};
  return {config.mass, position, velocity, kRampAngle, 0.0f, kBlockSize, true};
}

SimulationConfig ReadSimulationConfig() {
  SimulationConfig config;
  std::cout << "Tiny2D Engine pre-start configuration\n"
               "Press Enter to keep each default value.\n\n";
  config.friction =
      ReadFloat("Friction coefficient", config.friction, 0.0f, 5.0f);
  config.restitution =
      ReadFloat("Restitution coefficient", config.restitution, 0.0f, 1.0f);

  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float horizontal_margin = kBlockSize * 0.5f * std::cos(kRampAngle);
  const float minimum_x = ramp_bottom_x + horizontal_margin;
  const float maximum_x = static_cast<float>(kAreaWidth) - horizontal_margin;

  while (true) {
    config.upper_block = ReadBlockConfig("Upper block", config.upper_block,
                                         minimum_x, maximum_x);
    config.lower_block = ReadBlockConfig("Lower block", config.lower_block,
                                         minimum_x, maximum_x);
    if (!tiny2d::IsColliding(CreateBlockOnRamp(config.upper_block),
                             CreateBlockOnRamp(config.lower_block))) {
      break;
    }

    std::cout
        << "\nThe initial blocks overlap. Enter their parameters again.\n";
    if (std::cin.eof()) {
      const SimulationConfig defaults;
      config.upper_block = defaults.upper_block;
      config.lower_block = defaults.lower_block;
      break;
    }
  }

  assert(config.friction >= 0.0f);
  assert(config.restitution >= 0.0f && config.restitution <= 1.0f);
  assert(config.upper_block.mass > 0.0f);
  assert(config.lower_block.mass > 0.0f);
  assert(!tiny2d::IsColliding(CreateBlockOnRamp(config.upper_block),
                              CreateBlockOnRamp(config.lower_block)));
  std::cout << "\nStarting simulation...\n";
  return config;
}

tiny2d::Square CreateRamp() {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float visible_midpoint_x =
      (ramp_bottom_x + static_cast<float>(kAreaWidth)) * 0.5f;
  const float visible_midpoint_y =
      static_cast<float>(kAreaHeight) +
      (visible_midpoint_x - ramp_bottom_x) * std::tan(kRampAngle);
  const float half_size = kRampSize * 0.5f;
  const tiny2d::Vec2 center{
      visible_midpoint_x - std::sin(kRampAngle) * half_size,
      visible_midpoint_y + std::cos(kRampAngle) * half_size};

  // ponytail: A large static square exposes one ramp face. Add rectangle
  // dimensions only when the physics engine supports general box shapes.
  return {0.0f, center, {}, kRampAngle, 0.0f, kRampSize, true};
}

void TransitionBlocksToFloor(std::vector<tiny2d::Square>& squares,
                             float delta_time) {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float half_size = kBlockSize * 0.5f;

  for (tiny2d::Square& square : squares) {
    if (square.mass <= 0.0f || !square.fixed_rotation ||
        std::abs(square.angle - kRampAngle) > 0.001f ||
        square.velocity.x >= 0.0f) {
      continue;
    }

    const float front_corner_x =
        square.position.x -
        half_size * (std::cos(kRampAngle) + std::sin(kRampAngle));
    const float predicted_corner_x =
        front_corner_x + square.velocity.x * delta_time;
    if (predicted_corner_x > ramp_bottom_x) {
      continue;
    }

    const float speed_squared = square.velocity.x * square.velocity.x +
                                square.velocity.y * square.velocity.y;
    const float speed = std::sqrt(speed_squared);
    const float time_to_junction =
        std::clamp((front_corner_x - ramp_bottom_x) / -square.velocity.x, 0.0f,
                   delta_time);
    // Update() will still integrate the full step. This pre-offset makes its
    // final position include only the horizontal motion after the junction.
    square.position = {ramp_bottom_x - half_size + speed * time_to_junction,
                       static_cast<float>(kAreaHeight) - half_size};
    square.velocity = {-speed, 0.0f};
    square.angle = 0.0f;
    square.angular_velocity = 0.0f;

    const float transitioned_speed_squared =
        square.velocity.x * square.velocity.x +
        square.velocity.y * square.velocity.y;
    const float expected_end_x =
        ramp_bottom_x - half_size - speed * (delta_time - time_to_junction);
    assert(std::abs(transitioned_speed_squared - speed_squared) <=
           std::max(1.0f, speed_squared) * 0.00001f);
    assert(std::abs(square.position.x + square.velocity.x * delta_time -
                    expected_end_x) <= 0.0001f);
  }
}

void ConstrainBlocksToSurfaces(std::vector<tiny2d::Square>& squares) {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;

  // ponytail: This scene uses an ideal guide constraint. Add a general
  // constraint solver only when other surface shapes need the same behavior.
  for (tiny2d::Square& square : squares) {
    if (square.mass <= 0.0f || !square.fixed_rotation) {
      continue;
    }

    tiny2d::Vec2 normal{};
    tiny2d::Vec2 surface_position{};
    if (std::abs(square.angle - kRampAngle) <= 0.001f) {
      normal = {std::sin(kRampAngle), -std::cos(kRampAngle)};
      surface_position = {ramp_bottom_x, static_cast<float>(kAreaHeight)};
    } else if (std::abs(square.angle) <= 0.001f) {
      normal = {0.0f, -1.0f};
      surface_position = {square.position.x, static_cast<float>(kAreaHeight)};
    } else {
      continue;
    }

    const float half_size = square.size * 0.5f;
    const tiny2d::Vec2 target_position{
        surface_position.x + normal.x * half_size,
        surface_position.y + normal.y * half_size};
    const float position_error =
        (square.position.x - target_position.x) * normal.x +
        (square.position.y - target_position.y) * normal.y;
    square.position.x -= normal.x * position_error;
    square.position.y -= normal.y * position_error;

    const float normal_velocity =
        square.velocity.x * normal.x + square.velocity.y * normal.y;
    square.velocity.x -= normal.x * normal_velocity;
    square.velocity.y -= normal.y * normal_velocity;

    const float remaining_position_error =
        (square.position.x - target_position.x) * normal.x +
        (square.position.y - target_position.y) * normal.y;
    const float remaining_normal_velocity =
        square.velocity.x * normal.x + square.velocity.y * normal.y;
    assert(std::abs(remaining_position_error) <= 0.0001f);
    assert(std::abs(remaining_normal_velocity) <= 0.0001f);
  }
}

void DrawSquare(SDL_Renderer* renderer, const tiny2d::Square& square) {
  const std::array<tiny2d::Vec2, 4> corners = tiny2d::GetVertices(square);
  std::array<SDL_Vertex, 4> vertices{};
  const SDL_Color color = square.mass > 0.0f ? SDL_Color{255, 100, 100, 255}
                                             : SDL_Color{90, 110, 120, 255};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    vertices[i].position = {corners[i].x, corners[i].y};
    vertices[i].color = color;
  }
  SDL_RenderGeometry(renderer, nullptr, vertices.data(),
                     static_cast<int>(vertices.size()), kSquareIndices.data(),
                     static_cast<int>(kSquareIndices.size()));
}

}  // namespace

int main(int argc, char* argv[]) {
  const SimulationConfig config = ReadSimulationConfig();

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
      CreateBlockOnRamp(config.upper_block),
      CreateBlockOnRamp(config.lower_block),
      CreateRamp(),
  };

  bool running = true;
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();
  double accumulated_time = 0.0;

  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }

    const Uint64 current_time = SDL_GetPerformanceCounter();
    const double frame_time =
        std::min(static_cast<double>(current_time - previous_time) / frequency,
                 kMaxFrameTime);
    previous_time = current_time;
    accumulated_time += frame_time;

    while (accumulated_time >= kPhysicsStep) {
      TransitionBlocksToFloor(squares, kPhysicsStep);
      tiny2d::Update(squares, kPhysicsStep, static_cast<float>(kAreaWidth),
                     static_cast<float>(kAreaHeight), config.restitution,
                     config.friction);
      TransitionBlocksToFloor(squares, 0.0f);
      ConstrainBlocksToSurfaces(squares);
      accumulated_time -= kPhysicsStep;
    }

    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    for (const tiny2d::Square& square : squares) {
      DrawSquare(renderer, square);
    }

    SDL_RenderPresent(renderer);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
