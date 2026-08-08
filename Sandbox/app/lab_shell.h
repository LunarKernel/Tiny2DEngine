#ifndef TINY2DENGINE_SANDBOX_APP_LAB_SHELL_H_
#define TINY2DENGINE_SANDBOX_APP_LAB_SHELL_H_

#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <exception>
#include <optional>
#include <string>
#include <vector>

#include "fixed_step_clock.h"
#include "simulation_history.h"
#include "simulations.h"

// The shared application shell for every lab. RunFrameLoop owns the SDL
// event pump, the ImGui frame, and rendering; RunLab adds the standard
// setup -> run -> pause -> inspect lifecycle used by the continuous labs.
// Labs supply only their physics bindings and their setup, scene, and
// monitor panels through a traits object (see the *_simulation.cc files
// for the expected member functions).

namespace tiny2d::sandbox::shell {

enum class SetupAction {
  kNone,
  kStart,
  kBack,
};

struct FrameInput {
  Uint64 counter{};
  bool space_pressed{};
  bool quit_requested{};
};

// Pumps SDL events, begins an ImGui frame, calls frame(input), then renders.
// frame returns a result to end the loop or std::nullopt to continue.
// SDL_QUIT is reported through FrameInput; the frame decides how to exit.
// underlay(renderer) draws SDL primitives between the clear and the ImGui
// layer, for labs that render their world without ImGui.
template <typename FrameFunction, typename UnderlayFunction>
SimulationResult RunFrameLoop(SDL_Renderer* renderer, FrameFunction frame,
                              UnderlayFunction underlay,
                              SDL_Color clear_color = {18, 20, 24, 255}) {
  while (true) {
    FrameInput input;
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        input.quit_requested = true;
      } else if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                 event.key.keysym.sym == SDLK_SPACE) {
        input.space_pressed = true;
      }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    input.counter = SDL_GetPerformanceCounter();

    const std::optional<SimulationResult> result = frame(input);

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, clear_color.r, clear_color.g,
                           clear_color.b, clear_color.a);
    SDL_RenderClear(renderer);
    underlay(renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);

    if (result.has_value()) {
      return *result;
    }
  }
}

template <typename FrameFunction>
SimulationResult RunFrameLoop(SDL_Renderer* renderer, FrameFunction frame) {
  return RunFrameLoop(renderer, frame, [](SDL_Renderer*) {});
}

// Standard continuous-lab lifecycle. Traits must provide:
//   types Config, State (State has a double time_seconds member);
//   constants kPhysicsStep (float or double), kMaximumFrameTime (double),
//     kInvalidHistoryTimeMessage, kNonIncreasingHistoryTimeMessage;
//   Config MakeInitialConfig();
//   State MakeState(const Config&);                    // may throw
//   const char* InitialStateIssue(const Config&, const State&);
//   bool CanStep(const State&);                        // status gating
//   bool Step(const Config&, State*);                  // may throw
//   std::string StepFailureMessage(const Config&, const State&,
//                                  std::vector<State>* history);
//   const char* AfterStepIssue(const State&);          // nullptr: continue;
//                                                      // "": pause; text:
//                                                      // pause with error
//   SetupAction DrawSetup(Config*, const std::string& error);
//   bool DrawFrame(const Config&, const State&, const std::vector<State>&,
//                  bool* paused, double* inspect_time, bool* follow_live,
//                  const std::string& error);           // true stops the lab
template <typename Traits>
SimulationResult RunLab(SDL_Renderer* renderer, Traits traits) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  typename Traits::Config config = traits.MakeInitialConfig();
  typename Traits::State state{};
  std::vector<typename Traits::State> history;
  bool simulation_started = false;
  bool paused = false;
  bool follow_live = true;
  double inspect_time = 0.0;
  std::string runtime_error;
  FixedStepClock clock(static_cast<double>(SDL_GetPerformanceFrequency()),
                       SDL_GetPerformanceCounter());

  return RunFrameLoop(
      renderer,
      [&](const FrameInput& input) -> std::optional<SimulationResult> {
        bool return_requested = false;
        SimulationResult return_result = SimulationResult::kBackToSelection;
        if (input.quit_requested) {
          return_requested = true;
          return_result = SimulationResult::kQuit;
        }
        if (input.space_pressed && simulation_started &&
            runtime_error.empty() && traits.CanStep(state)) {
          paused = !paused;
        }

        if (!return_requested && !simulation_started) {
          clock.Reset(input.counter);
          const SetupAction action = traits.DrawSetup(&config, runtime_error);
          if (action == SetupAction::kBack) {
            return_requested = true;
          } else if (action == SetupAction::kStart) {
            try {
              state = traits.MakeState(config);
              history.clear();
              const bool recorded = AppendHistorySample(
                  &history, state, &Traits::State::time_seconds);
              const char* initial_issue =
                  recorded ? traits.InitialStateIssue(config, state) : nullptr;
              simulation_started = true;
              paused = !recorded || initial_issue != nullptr;
              follow_live = true;
              inspect_time = 0.0;
              runtime_error = !recorded ? Traits::kInvalidHistoryTimeMessage
                              : initial_issue != nullptr ? initial_issue
                                                         : "";
              clock.Reset(input.counter);
            } catch (const std::exception& error) {
              runtime_error = error.what();
            }
          }
        } else if (simulation_started) {
          if (!return_requested && !paused && runtime_error.empty() &&
              traits.CanStep(state)) {
            clock.Accumulate(input.counter, Traits::kMaximumFrameTime);
            while (clock.HasStep(Traits::kPhysicsStep)) {
              bool stepped = false;
              try {
                stepped = traits.Step(config, &state);
              } catch (const std::exception& error) {
                runtime_error = error.what();
                paused = true;
                clock.DiscardPendingSteps();
                break;
              }
              if (!stepped) {
                runtime_error =
                    traits.StepFailureMessage(config, state, &history);
                if (runtime_error.empty()) {
                  runtime_error = "The fixed-step model rejected this state.";
                }
                paused = true;
                clock.DiscardPendingSteps();
                break;
              }
              if (!AppendHistorySample(&history, state,
                                       &Traits::State::time_seconds)) {
                runtime_error = Traits::kNonIncreasingHistoryTimeMessage;
                paused = true;
                clock.DiscardPendingSteps();
                break;
              }
              clock.ConsumeStep(Traits::kPhysicsStep);
              const char* after_issue = traits.AfterStepIssue(state);
              if (after_issue != nullptr) {
                runtime_error = after_issue;
                paused = true;
                clock.DiscardPendingSteps();
                break;
              }
            }
          } else {
            clock.Reset(input.counter);
          }

          if (!return_requested &&
              traits.DrawFrame(config, state, history, &paused, &inspect_time,
                               &follow_live, runtime_error)) {
            return_requested = true;
          }
        }

        if (return_requested) {
          return return_result;
        }
        return std::nullopt;
      });
}

}  // namespace tiny2d::sandbox::shell

#endif  // TINY2DENGINE_SANDBOX_APP_LAB_SHELL_H_
