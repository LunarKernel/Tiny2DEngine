#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include "tiny2d_engine.h"

namespace {

constexpr int kAreaWidth = 800;
constexpr int kAreaHeight = 600;
constexpr float kBlockSize = 32.0f;
constexpr float kRampSize = 600.0f;
constexpr float kRampAngle = -0.5235987756f;
constexpr float kSpringAnchorX = 10.0f;
constexpr float kSpringRestX = 350.0f;
constexpr float kSpringAmplitude = 10.0f;
constexpr int kSpringCoilCount = 10;
constexpr float kJunctionHysteresis = 0.05f;
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
  float spring_stiffness{8.0f};
  BlockConfig upper_block{720.0f, 1.0f, 280.0f};
  BlockConfig lower_block{480.0f, 1.0f, 0.0f};
};

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

const char* GetConfigError(const SimulationConfig& config) {
  const std::array<float, 9> values = {
      config.friction,
      config.restitution,
      config.spring_stiffness,
      config.upper_block.surface_x,
      config.upper_block.mass,
      config.upper_block.downhill_speed,
      config.lower_block.surface_x,
      config.lower_block.mass,
      config.lower_block.downhill_speed,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "Every value must be a valid number.";
  }
  if (tiny2d::IsColliding(CreateBlockOnRamp(config.upper_block),
                          CreateBlockOnRamp(config.lower_block))) {
    return "The blocks overlap. Move them farther apart before starting.";
  }
  return nullptr;
}

void DrawBlockControls(const char* title, BlockConfig* config, float minimum_x,
                       float maximum_x) {
  ImGui::PushID(title);
  ImGui::TextUnformatted(title);
  ImGui::TextDisabled(
      "Ramp position: right is higher. Speed: positive is downhill.");
  ImGui::SliderFloat("Ramp position", &config->surface_x, minimum_x, maximum_x,
                     "%.0f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat(
      "Mass", &config->mass, 0.01f, 1000.0f, "%.2f",
      ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic);
  ImGui::SliderFloat("Initial speed", &config->downhill_speed, -1000.0f,
                     1000.0f, "%.0f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::PopID();
}

bool DrawSetupScreen(SimulationConfig* config) {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float horizontal_margin = kBlockSize * 0.5f * std::cos(kRampAngle);
  const float minimum_x = ramp_bottom_x + horizontal_margin;
  const float maximum_x = static_cast<float>(kAreaWidth) - horizontal_margin;

  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("Tiny2D setup", nullptr, kWindowFlags);

  ImGui::TextColored({1.0f, 0.55f, 0.3f, 1.0f}, "Tiny2D Physics Lab");
  ImGui::TextWrapped(
      "Adjust the experiment, then press Start simulation. Restore defaults "
      "returns every value to a safe preset.");
  ImGui::Spacing();

  ImGui::TextUnformatted("System");
  ImGui::Separator();
  ImGui::TextDisabled(
      "Friction: 0 is slippery. Bounciness: 1 keeps more collision energy.");
  ImGui::SliderFloat("Surface friction", &config->friction, 0.0f, 5.0f, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat("Collision bounciness", &config->restitution, 0.0f, 1.0f,
                     "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::SliderFloat(
      "Spring strength", &config->spring_stiffness, 0.1f, 100.0f, "%.1f",
      ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic);

  ImGui::Spacing();
  ImGui::Separator();
  DrawBlockControls("Higher block", &config->upper_block, minimum_x, maximum_x);
  ImGui::Spacing();
  ImGui::Separator();
  DrawBlockControls("Lower block", &config->lower_block, minimum_x, maximum_x);
  ImGui::Spacing();

  const float button_width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  if (ImGui::Button("Restore defaults", {button_width, 38.0f})) {
    *config = SimulationConfig{};
  }
  const char* error = GetConfigError(*config);
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  const bool start = ImGui::Button("Start simulation", {button_width, 38.0f});
  ImGui::EndDisabled();

  if (error != nullptr) {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  } else {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f}, "Ready to start.");
  }

  ImGui::End();
  return start;
}

#ifndef NDEBUG
void CheckConfigValidation() {
  SimulationConfig config;
  assert(GetConfigError(config) == nullptr);
  config.lower_block.surface_x = config.upper_block.surface_x;
  assert(GetConfigError(config) != nullptr);
}
#endif

void ShowError(const char* message) {
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tiny2D Engine", message,
                           nullptr);
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
    if (predicted_corner_x > ramp_bottom_x - kJunctionHysteresis) {
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

void TransitionBlocksToRamp(std::vector<tiny2d::Square>& squares,
                            float delta_time) {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float area_height = static_cast<float>(kAreaHeight);
  const float cosine = std::cos(kRampAngle);
  const float sine = std::sin(kRampAngle);

  for (tiny2d::Square& square : squares) {
    if (square.mass <= 0.0f || !square.fixed_rotation ||
        std::abs(square.angle) > 0.001f || square.velocity.x <= 0.0f) {
      continue;
    }

    const float half_size = square.size * 0.5f;
    const float right_edge = square.position.x + half_size;
    const float predicted_edge = right_edge + square.velocity.x * delta_time;
    if (predicted_edge < ramp_bottom_x + kJunctionHysteresis) {
      continue;
    }

    const float speed_squared = square.velocity.x * square.velocity.x +
                                square.velocity.y * square.velocity.y;
    const float speed = std::sqrt(speed_squared);
    const float time_to_junction = std::clamp(
        (ramp_bottom_x - right_edge) / square.velocity.x, 0.0f, delta_time);
    const tiny2d::Vec2 junction_position{
        ramp_bottom_x + half_size * (cosine + sine),
        area_height - half_size * (cosine - sine)};
    square.velocity = {cosine * speed, sine * speed};
    square.position = {
        junction_position.x - square.velocity.x * time_to_junction,
        junction_position.y - square.velocity.y * time_to_junction};
    square.angle = kRampAngle;
    square.angular_velocity = 0.0f;

    const float transitioned_speed_squared =
        square.velocity.x * square.velocity.x +
        square.velocity.y * square.velocity.y;
    const float expected_end_x =
        junction_position.x +
        square.velocity.x * (delta_time - time_to_junction);
    assert(std::abs(transitioned_speed_squared - speed_squared) <=
           std::max(1.0f, speed_squared) * 0.00001f);
    assert(std::abs(square.position.x + square.velocity.x * delta_time -
                    expected_end_x) <= 0.0001f);
  }
}

#ifndef NDEBUG
void CheckJunctionHysteresis() {
  const float ramp_bottom_x = static_cast<float>(kAreaWidth) * 0.5f;
  const float half_size = kBlockSize * 0.5f;
  const float cosine = std::cos(kRampAngle);
  const float sine = std::sin(kRampAngle);
  const tiny2d::Vec2 ramp_junction{
      ramp_bottom_x + half_size * (cosine + sine),
      static_cast<float>(kAreaHeight) - half_size * (cosine - sine)};

  std::vector<tiny2d::Square> squares = {
      {1.0f,
       ramp_junction,
       {-cosine, -sine},
       kRampAngle,
       0.0f,
       kBlockSize,
       true},
  };
  TransitionBlocksToFloor(squares, kPhysicsStep);
  assert(std::abs(squares[0].angle - kRampAngle) <= 0.001f);

  squares[0].velocity = {-10.0f * cosine, -10.0f * sine};
  TransitionBlocksToFloor(squares, kPhysicsStep);
  assert(std::abs(squares[0].angle) <= 0.001f);

  squares[0].velocity = {1.0f, 0.0f};
  TransitionBlocksToRamp(squares, kPhysicsStep);
  assert(std::abs(squares[0].angle) <= 0.001f);

  squares[0].velocity = {10.0f, 0.0f};
  TransitionBlocksToRamp(squares, kPhysicsStep);
  assert(std::abs(squares[0].angle - kRampAngle) <= 0.001f);
}
#endif

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

bool IsOnFloor(const tiny2d::Square& square) {
  return square.mass > 0.0f && square.fixed_rotation &&
         std::abs(square.angle) <= 0.001f;
}

std::size_t FindSpringBlockIndex(const std::vector<tiny2d::Square>& squares) {
  std::size_t block_index = squares.size();
  float spring_end_x = kSpringRestX;
  for (std::size_t i = 0; i < squares.size(); ++i) {
    if (!IsOnFloor(squares[i])) {
      continue;
    }
    const float left_edge = squares[i].position.x - squares[i].size * 0.5f;
    if (left_edge < spring_end_x) {
      spring_end_x = left_edge;
      block_index = i;
    }
  }
  return block_index;
}

void ApplySpringForce(std::vector<tiny2d::Square>& squares, float stiffness,
                      float delta_time) {
  const std::size_t block_index = FindSpringBlockIndex(squares);
  if (block_index == squares.size()) {
    return;
  }

  tiny2d::Square& block = squares[block_index];
  const float left_edge = block.position.x - block.size * 0.5f;
  const float compression = kSpringRestX - left_edge;
  const float velocity_change =
      stiffness * compression / block.mass * delta_time;
  const float previous_velocity_x = block.velocity.x;
  block.velocity.x += velocity_change;
  assert(block.velocity.x >= previous_velocity_x);
}

void DrawSpring(SDL_Renderer* renderer,
                const std::vector<tiny2d::Square>& squares) {
  float spring_end_x = kSpringRestX;
  const std::size_t block_index = FindSpringBlockIndex(squares);
  if (block_index != squares.size()) {
    spring_end_x = std::clamp(
        squares[block_index].position.x - squares[block_index].size * 0.5f,
        kSpringAnchorX, kSpringRestX);
  }

  constexpr float kSpringY =
      static_cast<float>(kAreaHeight) - kBlockSize * 0.5f;
  std::array<SDL_FPoint, kSpringCoilCount + 2> points{};
  points.front() = {kSpringAnchorX, kSpringY};
  for (int i = 1; i <= kSpringCoilCount; ++i) {
    const float fraction =
        static_cast<float>(i) / static_cast<float>(kSpringCoilCount + 1);
    points[i] = {
        kSpringAnchorX + (spring_end_x - kSpringAnchorX) * fraction,
        kSpringY + (i % 2 == 0 ? kSpringAmplitude : -kSpringAmplitude)};
  }
  points.back() = {spring_end_x, kSpringY};

  SDL_SetRenderDrawColor(renderer, 245, 200, 70, 255);
  SDL_RenderDrawLinesF(renderer, points.data(),
                       static_cast<int>(points.size()));
  SDL_RenderDrawLineF(renderer, kSpringAnchorX, kSpringY - kBlockSize * 0.5f,
                      kSpringAnchorX, kSpringY + kBlockSize * 0.5f);
  SDL_RenderDrawLineF(renderer, spring_end_x, kSpringY - kBlockSize * 0.5f,
                      spring_end_x, kSpringY + kBlockSize * 0.5f);
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
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
    ShowError("SDL could not start.");
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow("Tiny2D Engine", SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED, kAreaWidth,
                                        kAreaHeight, SDL_WINDOW_SHOWN);

  if (!window) {
    ShowError("The application window could not be created.");
    SDL_Quit();
    return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if (!renderer) {
    ShowError("The graphics renderer could not be created.");
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.IniFilename = nullptr;
#ifdef _WIN32
  // ponytail: Use the native UI font; bundle one only when identical
  // cross-platform typography becomes a requirement.
  if (io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 16.0f) ==
      nullptr) {
    io.Fonts->AddFontDefault();
  }
#else
  io.Fonts->AddFontDefault();
#endif
  ImGui::StyleColorsDark();
  ImGui::GetStyle().FrameRounding = 5.0f;
  ImGui::GetStyle().GrabRounding = 5.0f;

  if (!ImGui_ImplSDL2_InitForSDLRenderer(window, renderer)) {
    ShowError("The settings interface could not be initialized.");
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  if (!ImGui_ImplSDLRenderer2_Init(renderer)) {
    ShowError("The settings interface renderer could not be initialized.");
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

#ifndef NDEBUG
  CheckConfigValidation();
  CheckJunctionHysteresis();
#endif

  SimulationConfig config;
  std::vector<tiny2d::Square> squares;
  bool simulation_started = false;

  bool running = true;
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();
  double accumulated_time = 0.0;

  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }
    if (!running) {
      break;
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint64 current_time = SDL_GetPerformanceCounter();
    if (!simulation_started) {
      previous_time = current_time;
      accumulated_time = 0.0;
      if (DrawSetupScreen(&config)) {
        assert(GetConfigError(config) == nullptr);
        squares = {
            CreateBlockOnRamp(config.upper_block),
            CreateBlockOnRamp(config.lower_block),
            CreateRamp(),
        };
        assert(squares.size() == 3);
        simulation_started = true;
      }
    } else {
      const double frame_time = std::min(
          static_cast<double>(current_time - previous_time) / frequency,
          kMaxFrameTime);
      previous_time = current_time;
      accumulated_time += frame_time;

      while (accumulated_time >= kPhysicsStep) {
        TransitionBlocksToFloor(squares, kPhysicsStep);
        ApplySpringForce(squares, config.spring_stiffness, kPhysicsStep);
        TransitionBlocksToRamp(squares, kPhysicsStep);
        tiny2d::Update(squares, kPhysicsStep, static_cast<float>(kAreaWidth),
                       static_cast<float>(kAreaHeight), config.restitution,
                       config.friction);
        TransitionBlocksToFloor(squares, 0.0f);
        TransitionBlocksToRamp(squares, 0.0f);
        ConstrainBlocksToSurfaces(squares);
        accumulated_time -= kPhysicsStep;
      }
    }

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    if (simulation_started) {
      DrawSpring(renderer, squares);
      for (const tiny2d::Square& square : squares) {
        DrawSquare(renderer, square);
      }
    }
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

    SDL_RenderPresent(renderer);
  }

  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
