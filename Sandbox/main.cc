#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <cstddef>
#include <exception>
#include <optional>

#include "app/lab_registry.h"
#include "app/lab_shell.h"

namespace {

constexpr int kWindowWidth = 1200;
constexpr int kWindowHeight = 800;

void ShowError(const char* message) {
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Tiny2D Engine", message,
                           nullptr);
}

// Owns SDL and ImGui setup and tears down whatever succeeded, so every early
// return in main takes the same cleanup path.
class AppContext {
 public:
  AppContext() = default;
  AppContext(const AppContext&) = delete;
  AppContext& operator=(const AppContext&) = delete;

  ~AppContext() {
    if (imgui_renderer_ready_) {
      ImGui_ImplSDLRenderer2_Shutdown();
    }
    if (imgui_backend_ready_) {
      ImGui_ImplSDL2_Shutdown();
    }
    if (imgui_context_ready_) {
      ImGui::DestroyContext();
    }
    if (renderer_ != nullptr) {
      SDL_DestroyRenderer(renderer_);
    }
    if (window_ != nullptr) {
      SDL_DestroyWindow(window_);
    }
    if (sdl_ready_) {
      SDL_Quit();
    }
  }

  // Returns nullptr on success, otherwise a user-facing error message.
  const char* Initialize() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
      return "SDL could not start.";
    }
    sdl_ready_ = true;

    window_ = SDL_CreateWindow("Tiny2D Engine " TINY2D_PRODUCT_VERSION,
                               SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               kWindowWidth, kWindowHeight, SDL_WINDOW_SHOWN);
    if (window_ == nullptr) {
      return "The application window could not be created.";
    }

    renderer_ = SDL_CreateRenderer(
        window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer_ == nullptr) {
      return "The graphics renderer could not be created.";
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    imgui_context_ready_ = true;
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
#ifdef _WIN32
    // NOTE: Use the native UI font; bundle one only when identical
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

    if (!ImGui_ImplSDL2_InitForSDLRenderer(window_, renderer_)) {
      return "The settings interface could not be initialized.";
    }
    imgui_backend_ready_ = true;
    if (!ImGui_ImplSDLRenderer2_Init(renderer_)) {
      return "The settings interface renderer could not be initialized.";
    }
    imgui_renderer_ready_ = true;
    return nullptr;
  }

  SDL_Renderer* renderer() const { return renderer_; }

 private:
  bool sdl_ready_ = false;
  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  bool imgui_context_ready_ = false;
  bool imgui_backend_ready_ = false;
  bool imgui_renderer_ready_ = false;
};

// Draws the selection menu from the lab registry. Returns the chosen lab
// index, or std::nullopt when the user quit. The lab list scrolls inside a
// child region so every entry and the Quit button stay reachable no matter
// how many labs are installed.
std::optional<std::size_t> ChooseLab(SDL_Renderer* renderer) {
  namespace shell = tiny2d::sandbox::shell;
  std::optional<std::size_t> chosen;
  const tiny2d::sandbox::SimulationResult result = shell::RunFrameLoop(
      renderer,
      [&](const shell::FrameInput& input)
          -> std::optional<tiny2d::sandbox::SimulationResult> {
        if (input.quit_requested) {
          return tiny2d::sandbox::SimulationResult::kQuit;
        }

        const ImVec2 display_size = ImGui::GetIO().DisplaySize;
        ImGui::SetNextWindowPos({0.0f, 0.0f});
        ImGui::SetNextWindowSize(display_size);
        constexpr ImGuiWindowFlags kWindowFlags =
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("Tiny2D model selection", nullptr, kWindowFlags);

        constexpr float kPanelWidth = 720.0f;
        const float panel_width = std::min(kPanelWidth, display_size.x - 48.0f);
        const float panel_left =
            std::max(24.0f, (display_size.x - panel_width) * 0.5f);
        ImGui::SetCursorPosX(panel_left);
        ImGui::SetCursorPosY(25.0f);
        ImGui::BeginGroup();
        ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f}, "Tiny2D Physics Lab");
        ImGui::TextUnformatted("Choose a simulation model");
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::EndGroup();

        constexpr float kQuitAreaHeight = 58.0f;
        ImGui::SetCursorPosX(panel_left);
        ImGui::BeginChild(
            "lab_list",
            {panel_width,
             std::max(ImGui::GetContentRegionAvail().y - kQuitAreaHeight,
                      100.0f)});
        constexpr float kModelButtonHeight = 36.0f;
        const float list_width = ImGui::GetContentRegionAvail().x;
        for (std::size_t i = 0; i < tiny2d::sandbox::kLabs.size(); ++i) {
          const tiny2d::sandbox::LabInfo& lab = tiny2d::sandbox::kLabs[i];
          if (i != 0) {
            ImGui::Spacing();
          }
          ImGui::TextUnformatted(lab.title);
          ImGui::TextWrapped("%s", lab.description);
          if (ImGui::Button(lab.button_label,
                            {list_width, kModelButtonHeight})) {
            chosen = i;
          }
        }
        ImGui::EndChild();

        ImGui::Spacing();
        ImGui::SetCursorPosX(panel_left);
        const bool quit = ImGui::Button("Quit", {panel_width, 42.0f});
        ImGui::End();

        if (chosen.has_value()) {
          return tiny2d::sandbox::SimulationResult::kBackToSelection;
        }
        if (quit) {
          return tiny2d::sandbox::SimulationResult::kQuit;
        }
        return std::nullopt;
      });
  if (result == tiny2d::sandbox::SimulationResult::kQuit) {
    chosen.reset();
  }
  return chosen;
}

}  // namespace

int main(int, char*[]) {
  AppContext app;
  if (const char* error = app.Initialize(); error != nullptr) {
    ShowError(error);
    return 1;
  }

  int exit_code = 0;
  try {
    while (true) {
      const std::optional<std::size_t> choice = ChooseLab(app.renderer());
      if (!choice.has_value()) {
        break;
      }
      const tiny2d::sandbox::SimulationResult result =
          tiny2d::sandbox::kLabs[*choice].run(app.renderer());
      if (result == tiny2d::sandbox::SimulationResult::kQuit) {
        break;
      }
    }
  } catch (const std::exception& error) {
    ShowError(error.what());
    exit_code = 1;
  }

  return exit_code;
}
