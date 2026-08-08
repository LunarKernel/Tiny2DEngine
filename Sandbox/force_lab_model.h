#ifndef TINY2DENGINE_SANDBOX_FORCE_LAB_MODEL_H_
#define TINY2DENGINE_SANDBOX_FORCE_LAB_MODEL_H_

#include <vector>

#include "tiny2d_engine.h"

namespace tiny2d::sandbox {

inline constexpr float kForceLabPhysicsStep = 1.0f / 480.0f;
inline constexpr float kForceLabAreaWidthM = 100.0f;
inline constexpr float kForceLabAreaHeightM = 100.0f;
inline constexpr float kForceLabMinimumMassKg = 0.01f;
inline constexpr float kForceLabMaximumMassKg = 1000.0f;
inline constexpr float kForceLabMinimumBodyDimensionM = 0.1f;
inline constexpr float kForceLabMaximumBodyDimensionM = 10.0f;
inline constexpr float kForceLabMinimumSpringStiffnessNM = 0.01f;
inline constexpr float kForceLabMaximumSpringStiffnessNM = 1000.0f;
inline constexpr float kForceLabMinimumSpringLengthM = 0.00001f;
inline constexpr float kForceLabMaximumSpringLengthM = 50.0f;
inline constexpr float kForceLabMaximumInitialSpeedMps = 50.0f;
inline constexpr float kForceLabMaximumInitialAngularSpeedRadS = 50.0f;
inline constexpr float kForceLabMaximumDampingCoefficient = 1000.0f;

// All values use SI units. +X points right and +Y points down. Positive body
// angles, angular velocity, angular acceleration, and torque therefore appear
// clockwise on screen. The attachment point is expressed in the body's local
// axes relative to its center of mass. Mass is in [0.01, 1000] kg; body
// dimensions are in [0.1, 10] m; stiffness is in [0.01, 1000] N/m; rest
// length is in [1e-5, 50] m; initial speed magnitude and angular-speed
// magnitude are at most 50 m/s and 50 rad/s; initial angle is in
// [-180, 180] degrees; damping coefficients are in [0, 1000] in their named
// units. The anchor and complete initial body must be inside the integration
// area, and the local attachment must be inside the closed body rectangle.
struct ForceLabConfig {
  float mass_kg{1.0f};
  float width_m{1.2f};
  float height_m{0.8f};
  Vec2 anchor_position_m{47.0f, 50.0f};
  Vec2 attachment_local_m{0.0f, -0.3f};
  float spring_stiffness_n_m{4.0f};
  float spring_rest_length_m{2.5f};
  Vec2 initial_center_position_m{50.0f, 50.0f};
  Vec2 initial_velocity_m_s{};
  float initial_angle_degrees{};
  float initial_angular_velocity_rad_s{};
  // Viscous coefficients. Zero disables the corresponding damping term.
  float linear_damping_n_s_m{};
  float angular_damping_n_m_s_rad{};
};

struct ForceLabState {
  Rectangle body;
  // Elapsed simulation time in seconds. Must be finite and nonnegative.
  double time_seconds{};
  double dissipated_energy_j{};
};

struct ForceLabDerived {
  Vec2 attachment_offset_world_m;
  Vec2 attachment_position_m;
  Vec2 spring_force_n;
  Vec2 damping_force_n;
  Vec2 linear_acceleration_m_s2;
  float spring_length_m{};
  float spring_extension_m{};
  float moment_of_inertia_kg_m2{};
  float spring_torque_n_m{};
  float damping_torque_n_m{};
  float angular_acceleration_rad_s2{};
  double translational_kinetic_energy_j{};
  double rotational_kinetic_energy_j{};
  double spring_potential_energy_j{};
  double mechanical_energy_j{};
  double accounted_energy_j{};
};

ForceLabConfig MakeCenteredReferenceConfig();
ForceLabConfig MakeEccentricDemoConfig();

// Returns nullptr when valid. The spring attachment must lie inside the body,
// and its initial world-space distance from the anchor must be in
// [1e-5, 50] m. The complete body must fit inside the 100 m by 100 m internal
// integration area.
const char* GetForceLabConfigError(const ForceLabConfig& config);

// Returns nullptr when config and state are valid. Stable public states have
// no pending Engine force or torque. Spring length remains in [1e-5, 50] m.
const char* GetForceLabStateError(const ForceLabConfig& config,
                                  const ForceLabState& state);

// Returns 2*pi*sqrt(m/k) seconds. Invalid mass or stiffness returns infinity.
float GetForceLabCenteredPeriod(const ForceLabConfig& config);

// Throws std::invalid_argument when config is invalid.
ForceLabState MakeInitialForceLabState(const ForceLabConfig& config);

// Throws std::invalid_argument when config or state is invalid.
ForceLabDerived CalculateForceLabDerived(const ForceLabConfig& config,
                                         const ForceLabState& state);

// Advances one semi-implicit fixed step. delta_time must be finite and in
// (0, kForceLabPhysicsStep]. Invalid input returns false without changing
// state. Engine validation exceptions may propagate, but state remains
// unchanged because integration is performed on a copy.
bool StepForceLab(const ForceLabConfig& config, float delta_time,
                  ForceLabState* state);

// History times must be strictly increasing seconds. Returns the nearest
// sample, clamped to the recorded range. Empty history or a non-finite query
// returns nullptr.
const ForceLabState* FindForceLabState(
    const std::vector<ForceLabState>& history, double time_seconds);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_FORCE_LAB_MODEL_H_
