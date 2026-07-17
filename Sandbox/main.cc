#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>

#include "simulations.h"

namespace {

constexpr int kWindowWidth = 1200;
constexpr int kWindowHeight = 800;

enum class ModelChoice {
  kNone,
  kInclineSpring,
  kRotationPendulum,
  kQuit,
};

void ShowError(const char* message) {
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tiny2D Engine", message,
                           nullptr);
}

ModelChoice ChooseModel(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return ModelChoice::kQuit;
  }

  while (true) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        return ModelChoice::kQuit;
      }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const ImVec2 display_size = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos({0.0f, 0.0f});
    ImGui::SetNextWindowSize(display_size);
    constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                              ImGuiWindowFlags_NoMove |
                                              ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin("Tiny2D model selection", nullptr, kWindowFlags);

    constexpr float kPanelWidth = 720.0f;
    const float panel_width = std::min(kPanelWidth, display_size.x - 48.0f);
    ImGui::SetCursorPosX(
        std::max(24.0f, (display_size.x - panel_width) * 0.5f));
    ImGui::SetCursorPosY(95.0f);
    ImGui::BeginGroup();
    ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f}, "Tiny2D Physics Lab");
    ImGui::TextUnformatted("Choose a simulation model");
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ModelChoice choice = ModelChoice::kNone;
    ImGui::TextUnformatted("V9 Stable: Incline / Spring / Electric Field");
    ImGui::TextWrapped(
        "The validated V9 model with two blocks, ramp, floor, spring, uniform "
        "electric field, SI calibration, telemetry, and history inspection.");
    if (ImGui::Button("Open V9 incline laboratory", {panel_width, 74.0f})) {
      choice = ModelChoice::kInclineSpring;
    }

    ImGui::Spacing();
    ImGui::TextUnformatted("V10 PivotLab: Charged Physical Pendulum");
    ImGui::TextWrapped(
        "A uniform rod with a movable charged point mass. Explore moment of "
        "inertia, gravity and electric torque, damping, period, and energy.");
    if (ImGui::Button("Open V10 PivotLab", {panel_width, 74.0f})) {
      choice = ModelChoice::kRotationPendulum;
    }

    ImGui::Spacing();
    if (ImGui::Button("Quit", {panel_width, 42.0f})) {
      choice = ModelChoice::kQuit;
    }
    ImGui::EndGroup();
    ImGui::End();

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 18, 20, 24, 255);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
    if (choice != ModelChoice::kNone) {
      return choice;
    }
  }
}

}  // namespace

int main(int, char*[]) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
    ShowError("SDL could not start.");
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow(
      "Tiny2D Engine V10", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      kWindowWidth, kWindowHeight, SDL_WINDOW_SHOWN);
  if (window == nullptr) {
    ShowError("The application window could not be created.");
    SDL_Quit();
    return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer == nullptr) {
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

  bool running = true;
  while (running) {
    const ModelChoice choice = ChooseModel(renderer);
    tiny2d::sandbox::SimulationResult result =
        tiny2d::sandbox::SimulationResult::kQuit;
    if (choice == ModelChoice::kInclineSpring) {
      result = tiny2d::sandbox::RunInclineSpringSimulation(renderer);
    } else if (choice == ModelChoice::kRotationPendulum) {
      result = tiny2d::sandbox::RunRotationPendulumSimulation(renderer);
    } else {
      break;
    }
    running = result != tiny2d::sandbox::SimulationResult::kQuit;
  }

  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
