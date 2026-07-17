#ifndef SANDBOX_SIMULATIONS_H_
#define SANDBOX_SIMULATIONS_H_

struct SDL_Renderer;

namespace tiny2d::sandbox {

enum class SimulationResult {
  kBackToSelection,
  kQuit,
};

SimulationResult RunInclineSpringSimulation(SDL_Renderer* renderer);
SimulationResult RunRotationPendulumSimulation(SDL_Renderer* renderer);

}  // namespace tiny2d::sandbox

#endif  // SANDBOX_SIMULATIONS_H_
