#ifndef TINY2DENGINE_SANDBOX_APP_LAB_REGISTRY_H_
#define TINY2DENGINE_SANDBOX_APP_LAB_REGISTRY_H_

#include <array>

#include "simulations.h"

struct SDL_Renderer;

// The single list of installed labs. The selection menu renders from this
// table, so adding a lab means adding its Run function to simulations.h and
// one entry here.

namespace tiny2d::sandbox {

struct LabInfo {
  const char* title;
  const char* description;
  const char* button_label;
  SimulationResult (*run)(SDL_Renderer* renderer);
};

inline constexpr std::array<LabInfo, 9> kLabs = {{
    {"V19 ChaosLab: Double Pendulum",
     "Two point-mass bobs on rigid rod constraints with normal-mode "
     "anchors, energy telemetry, and a shadow run that makes chaotic "
     "divergence visible.",
     "Open V19 ChaosLab", &RunChaosLabSimulation},
    {"V18 AtwoodLab: Massive Pulley and Rope",
     "Two hanging masses on an inextensible, non-slipping rope over a "
     "pinned pulley with real rotational inertia, tension telemetry, and "
     "the analytical Atwood acceleration.",
     "Open V18 AtwoodLab", &RunAtwoodLabSimulation},
    {"V17 ImpactLab: Continuous Circle Impacts",
     "Compare discrete collision detection with circle-circle time of "
     "impact for grazing and diameter-skipping motion.",
     "Open V17 ImpactLab", &RunImpactLabSimulation},
    {"V16 ContactLab: Circle Impacts and Rolling",
     "Native circle bodies, material-aware impacts, momentum and energy "
     "checks, and disk or hoop rolling contact.",
     "Open V16 ContactLab", &RunContactLabSimulation},
    {"V15 ForceLab: Eccentric Spring Rigid Body",
     "A rectangular rigid body coupled to a fixed spring through an "
     "adjustable center or eccentric attachment point.",
     "Open V15 ForceLab", &RunForceLabSimulation},
    {"V9 Stable: Incline / Spring / Electric Field",
     "Two blocks, ramp, floor, spring, electric field, SI telemetry, and "
     "history.",
     "Open V9 incline laboratory", &RunInclineSpringSimulation},
    {"V14 Driven PivotLab: Forced Damped Pendulum",
     "Extends V10 with a periodic torque, resonance preset, live drive "
     "power, and the original no-drive behavior.",
     "Open V14 Driven PivotLab", &RunRotationPendulumSimulation},
    {"V11 RollLab: Sliding / Rolling Transition",
     "A disk or hoop sliding into pure rolling under friction and uniform "
     "electric and magnetic fields.",
     "Open V11 RollLab", &RunRollingDiskSimulation},
    {"V13 Gravito-Orbit: Gravity in Uniform Fields",
     "Extends V12 exact charged-particle motion with uniform gravity, "
     "potential energy, and combined-force drift.",
     "Open V13 Gravito-Orbit", &RunLorentzParticleSimulation},
}};

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_APP_LAB_REGISTRY_H_
