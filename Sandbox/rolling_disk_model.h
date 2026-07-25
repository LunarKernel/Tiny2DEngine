#ifndef SANDBOX_ROLLING_DISK_MODEL_H_
#define SANDBOX_ROLLING_DISK_MODEL_H_

#include <vector>

namespace tiny2d::sandbox {

enum class RollingDiskKind {
  kSolidDisk,
  kHoop,
};

enum class RollingContactMode {
  kRolling,
  kSliding,
};

enum class RollingDiskStatus {
  kActive,
  kReachedTop,
  kReachedBottom,
  kAirborne,
};

// SI units are used throughout. The ramp rises counterclockwise from global
// +X; electric-field angles are also counterclockwise from +X. Distance and
// linear velocity are positive downhill. Positive angular velocity is the
// direction that satisfies pure rolling: velocity = radius * angular velocity.
struct RollingDiskConfig {
  RollingDiskKind kind{RollingDiskKind::kSolidDisk};
  float ramp_length_m{10.0f};
  float ramp_angle_degrees{30.0f};
  float mass_kg{1.0f};
  float radius_m{0.25f};
  float static_friction_coefficient{0.4f};
  float kinetic_friction_coefficient{0.3f};
  float initial_distance_down_ramp_m{0.5f};
  float initial_velocity_down_ramp_m_s{3.0f};
  float initial_angular_velocity_rad_s{};
  float gravity_m_s2{9.8f};
  bool electric_field_enabled{};
  float charge_c{};
  float electric_field_strength_n_c{};
  float electric_field_angle_degrees{};
};

struct RollingDiskState {
  float distance_down_ramp_m{};
  float velocity_down_ramp_m_s{};
  float angle_radians{};
  float angular_velocity_rad_s{};
  float time_seconds{};
  float dissipated_energy_j{};
  RollingContactMode contact_mode{RollingContactMode::kRolling};
  RollingDiskStatus status{RollingDiskStatus::kActive};
};

struct RollingDiskDerived {
  float moment_of_inertia_kg_m2{};
  float tangential_external_force_n{};
  float normal_force_n{};
  float friction_force_n{};
  float acceleration_down_ramp_m_s2{};
  float angular_acceleration_rad_s2{};
  float slip_velocity_m_s{};
  float translational_kinetic_energy_j{};
  float rotational_kinetic_energy_j{};
  float potential_energy_j{};
  float mechanical_energy_j{};
  float accounted_energy_j{};
};

// Returns nullptr when valid. Valid ranges include mass, radius, and ramp
// length > 0; 0 <= kinetic friction <= static friction <= 5; 0 <= ramp angle
// < 90 degrees; gravity > 0; and initial distance in [0, ramp length].
const char* GetRollingDiskConfigError(const RollingDiskConfig& config);

const char* GetRollingDiskStateError(const RollingDiskConfig& config,
                                     const RollingDiskState& state);

RollingDiskState MakeInitialRollingDiskState(const RollingDiskConfig& config);

RollingDiskDerived CalculateRollingDiskDerived(const RollingDiskConfig& config,
                                               const RollingDiskState& state);

// Returns the nearest sample; queries outside the recorded interval clamp to
// its first or last sample. Returns nullptr for empty history or non-finite
// query time.
const RollingDiskState* FindRollingDiskState(
    const std::vector<RollingDiskState>& history, float time_seconds);

// Advances an active, surface-bound disk by a positive finite time step.
// Returns false for invalid input, an airborne disk, or an already-finished
// disk. Reaching either ramp endpoint is a successful step and updates status.
bool StepRollingDisk(const RollingDiskConfig& config, float delta_time,
                     RollingDiskState* state);

}  // namespace tiny2d::sandbox

#endif  // SANDBOX_ROLLING_DISK_MODEL_H_
