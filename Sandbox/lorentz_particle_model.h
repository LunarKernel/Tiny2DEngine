#ifndef SANDBOX_LORENTZ_PARTICLE_MODEL_H_
#define SANDBOX_LORENTZ_PARTICLE_MODEL_H_

#include <vector>

namespace tiny2d::sandbox {

inline constexpr double kLorentzPhysicsStep = 1.0 / 240.0;
inline constexpr double kLorentzMinimumX = -10.0;
inline constexpr double kLorentzMaximumX = 10.0;
inline constexpr double kLorentzMinimumY = -6.0;
inline constexpr double kLorentzMaximumY = 6.0;

enum class LorentzParticleStatus {
  kActive,
  kOutOfBounds,
};

// All values use SI units. +X points right, +Y points up, and +Z points out
// of the screen. Angles are measured counterclockwise from +X. The uniform
// magnetic field is B = (0, 0, magnetic_field_z_t).
struct LorentzParticleConfig {
  double mass_kg{1.0};
  double charge_c{1.0};
  double initial_x_m{-6.0};
  double initial_y_m{};
  double initial_speed_m_s{3.0};
  double initial_velocity_angle_degrees{90.0};
  bool electric_field_enabled{true};
  double electric_field_strength_n_c{0.3};
  double electric_field_angle_degrees{90.0};
  bool magnetic_field_enabled{true};
  double magnetic_field_z_t{1.0};
  bool gravity_enabled{false};
  double gravitational_acceleration_m_s2{9.8};
};

struct LorentzParticleState {
  double x_m{};
  double y_m{};
  double velocity_x_m_s{};
  double velocity_y_m_s{};
  double time_seconds{};
  LorentzParticleStatus status{LorentzParticleStatus::kActive};
};

struct LorentzParticleDerived {
  double acceleration_x_m_s2{};
  double acceleration_y_m_s2{};
  double speed_m_s{};
  double kinetic_energy_j{};
  // Electric potential energy is measured relative to the configured initial
  // position.
  double electric_potential_energy_j{};
  // Gravitational potential energy is measured relative to the configured
  // initial height, so total_energy_j has a stable and explicit zero point.
  double gravitational_potential_energy_j{};
  double total_energy_j{};
  // omega = q * B_z / m is signed. Positive values rotate velocity from +X
  // toward -Y, which appears clockwise on screen.
  double cyclotron_angular_frequency_rad_s{};
  double cyclotron_period_s{};
  double drift_velocity_x_m_s{};
  double drift_velocity_y_m_s{};
  double larmor_radius_m{};
  // False when q == 0 or the effective B_z == 0. In that case the period,
  // drift, and Larmor radius are not physical values and the UI displays N/A.
  bool has_cyclotron_data{};
};

// Valid ranges are: mass [0.01, 1000] kg, charge [-1000, 1000] C,
// initial position within x [-10, 10] m and y [-6, 6] m, initial speed
// [0, 100] m/s, angles [-180, 180] degrees, E [0, 1e6] N/C, and B_z
// [-100, 100] T, and gravitational acceleration [0, 100] m/s^2. When enabled,
// gravity points along -Y. A nonzero q/m must be representable. Effective
// fields must also satisfy
// |q B_z / m| <= 60 rad/s, |q E / m| <= 10000 m/s^2, and
// |q E / m + (0, -g)| <= 10000 m/s^2 so the fixed display sampling remains
// meaningful. Returns nullptr when valid.
const char* GetLorentzParticleConfigError(const LorentzParticleConfig& config);

// Returns nullptr when config and state are valid. Active states must remain
// inside the closed experiment rectangle; an out-of-bounds terminal state may
// lie outside it.
const char* GetLorentzParticleStateError(const LorentzParticleConfig& config,
                                         const LorentzParticleState& state);

LorentzParticleState MakeInitialLorentzParticleState(
    const LorentzParticleConfig& config);

LorentzParticleDerived CalculateLorentzParticleDerived(
    const LorentzParticleConfig& config, const LorentzParticleState& state);

// Advances the exact solution for constant uniform fields. delta_time must be
// finite and in (0, kLorentzPhysicsStep]. Invalid input returns false without
// changing state. Leaving the experiment rectangle commits a kOutOfBounds
// terminal state and returns true.
bool StepLorentzParticle(const LorentzParticleConfig& config, double delta_time,
                         LorentzParticleState* state);

// History must be sorted by nondecreasing time. The nearest sample is returned
// and out-of-range queries clamp to the first or last sample. Empty history or
// a non-finite query returns nullptr.
const LorentzParticleState* FindLorentzParticleState(
    const std::vector<LorentzParticleState>& history, double time_seconds);

}  // namespace tiny2d::sandbox

#endif  // SANDBOX_LORENTZ_PARTICLE_MODEL_H_
