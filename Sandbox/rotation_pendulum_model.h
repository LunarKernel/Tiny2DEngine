#ifndef TINY2DENGINE_SANDBOX_ROTATION_PENDULUM_MODEL_H_
#define TINY2DENGINE_SANDBOX_ROTATION_PENDULUM_MODEL_H_

#include <vector>

namespace tiny2d::sandbox {

inline constexpr float kPendulumPi = 3.14159265358979323846f;
inline constexpr float kPendulumDegreesToRadians = kPendulumPi / 180.0f;
inline constexpr float kPendulumRadiansToDegrees = 180.0f / kPendulumPi;
inline constexpr float kPendulumPhysicsStep = 1.0f / 240.0f;

inline constexpr float kMinimumRodLength = 0.1f;
inline constexpr float kMaximumRodLength = 20.0f;
inline constexpr float kMinimumMass = 0.01f;
inline constexpr float kMaximumMass = 1000.0f;
inline constexpr float kMaximumChargeMagnitude = 1000.0f;
inline constexpr float kMaximumElectricField = 1000000.0f;
inline constexpr float kMaximumDamping = 1000.0f;
inline constexpr float kMaximumInitialAngularSpeed = 50.0f;

// SI units are used throughout. Angles are measured counterclockwise from the
// vertically downward direction. Electric-field angles are measured
// counterclockwise from +X in a physical coordinate system where +Y is up.
// Valid ranges are: rod length [0.1, 20] m, each mass [0.01, 1000] kg,
// counterweight distance [0, rod length], charge [-1000, 1000] C, initial
// angle [-180, 180] degrees, initial angular speed [-50, 50] rad/s, gravity
// 9.8 or 10 m/s^2, electric-field magnitude [0, 1e6] N/C and angle
// [-180, 180] degrees, and damping coefficient [0, 1000] N*m*s/rad.
struct PendulumConfig {
  float rod_length_m{2.0f};
  float rod_mass_kg{2.0f};
  float counterweight_mass_kg{1.0f};
  float counterweight_distance_m{1.5f};
  float counterweight_charge_c{1.0f};
  float initial_angle_degrees{35.0f};
  float initial_angular_velocity_rad_s{};
  float gravity_m_s2{9.8f};
  bool electric_field_enabled{true};
  bool counterweight_charged{true};
  float electric_field_strength_n_c{5.0f};
  float electric_field_angle_degrees{};
  bool damping_enabled{};
  float damping_coefficient_n_m_s{0.1f};
};

struct PendulumState {
  float angle_radians{};
  float angular_velocity_rad_s{};
  float angular_acceleration_rad_s2{};
  float time_seconds{};
};

struct PendulumDerived {
  float moment_of_inertia_kg_m2{};
  float gravity_torque_n_m{};
  float electric_torque_n_m{};
  float damping_torque_n_m{};
  float total_torque_n_m{};
  float angular_acceleration_rad_s2{};
  float kinetic_energy_j{};
  float gravitational_potential_energy_j{};
  float electric_potential_energy_j{};
  float total_energy_j{};
};

float GetPendulumFieldAngle(const PendulumConfig& config);
float GetPendulumMomentOfInertia(const PendulumConfig& config);
float GetSmallAnglePeriod(const PendulumConfig& config);
PendulumDerived CalculatePendulumDerived(const PendulumConfig& config,
                                         const PendulumState& state);
bool IsPendulumDerivedFinite(const PendulumDerived& derived);

// Returns the reset state described by config at t = 0 s.
PendulumState MakeInitialPendulumState(const PendulumConfig& config);

// History must be sorted by nondecreasing time. The nearest state is returned;
// empty history or a non-finite query returns nullptr.
const PendulumState* FindPendulumState(
    const std::vector<PendulumState>& history, float time_seconds);

const char* GetPendulumStateError(const PendulumConfig& config,
                                  const PendulumState& state);
const char* GetPendulumConfigError(const PendulumConfig& config);

// Advances one semi-implicit Euler step. delta_time must be finite and
// positive. False reports an invalid configuration or state.
bool StepPendulum(const PendulumConfig& config, float delta_time,
                  PendulumState* state);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_ROTATION_PENDULUM_MODEL_H_
