#ifndef TINY2DENGINE_SANDBOX_ATWOOD_LAB_MODEL_H_
#define TINY2DENGINE_SANDBOX_ATWOOD_LAB_MODEL_H_

#include <vector>

#include "tiny2d_engine.h"

namespace tiny2d::sandbox {

inline constexpr float kAtwoodPhysicsStep = 1.0f / 480.0f;
inline constexpr float kAtwoodAreaWidthM = 100.0f;
inline constexpr float kAtwoodAreaHeightM = 100.0f;
inline constexpr float kAtwoodPulleyCenterXM = 50.0f;
inline constexpr float kAtwoodPulleyCenterYM = 20.0f;
inline constexpr float kAtwoodMinimumMassKg = 0.01f;
inline constexpr float kAtwoodMaximumMassKg = 1000.0f;
inline constexpr float kAtwoodMinimumPulleyRadiusM = 0.05f;
inline constexpr float kAtwoodMaximumPulleyRadiusM = 5.0f;
inline constexpr float kAtwoodMinimumBlockEdgeM = 0.05f;
inline constexpr float kAtwoodMaximumBlockEdgeM = 5.0f;
inline constexpr float kAtwoodMinimumHangDepthM = 0.5f;
inline constexpr float kAtwoodMaximumHangDepthM = 70.0f;
inline constexpr float kAtwoodMaximumRopeSpeedMps = 20.0f;
inline constexpr float kAtwoodMinimumGravityMps2 = 0.1f;
inline constexpr float kAtwoodMaximumGravityMps2 = 100.0f;
inline constexpr float kAtwoodMaximumDampingPerS = 100.0f;
// A valid configuration keeps every block at least this far below the
// pulley disk; the running terminal pause triggers at the smaller clearance,
// so a fresh configuration always has running room before it can terminate.
inline constexpr float kAtwoodInitialPulleyClearanceM = 0.15f;
inline constexpr float kAtwoodTerminalPulleyClearanceM = 0.05f;
// Blocks pause as "reached the floor" at the larger margin and fail state
// validation only at the smaller one, so termination is always detected
// before the state becomes invalid.
inline constexpr float kAtwoodTerminalFloorMarginM = 0.1f;
inline constexpr float kAtwoodBoundaryMarginM = 0.05f;

// All values use SI units. +X points right and +Y points down, so a block
// with a larger y hangs lower and positive pulley rotation appears
// clockwise. Block a hangs from the left tangent point of the pulley at
// (50 - R, 20); block b hangs from the right tangent point at (50 + R, 20).
// A positive rope speed means block b descends. Masses are in
// [0.01, 1000] kg; the pulley radius is in [0.05, 5] m; the block edge is in
// [0.05, 5] m and must stay below 2R so the blocks cannot touch each other;
// hang depths are in [0.5, 70] m and must clear the pulley disk by at least
// 0.15 m; the initial rope speed magnitude is at most 20 m/s; gravity is in
// [0.1, 100] m/s^2; the linear damping rate applied to both blocks is in
// [0, 100] 1/s.
struct AtwoodConfig {
  float mass_a_kg{1.0f};
  float mass_b_kg{1.2f};
  float pulley_mass_kg{0.5f};
  float pulley_radius_m{0.1f};
  CircleInertiaModel pulley_inertia{CircleInertiaModel::kSolidDisk};
  float block_edge_m{0.12f};
  float hang_depth_a_m{3.0f};
  float hang_depth_b_m{3.0f};
  float initial_rope_speed_mps{};
  float gravity_m_s2{9.81f};
  float linear_damping_per_s{};
};

// Engine bodies plus bookkeeping. Rope tensions and the pin force hold the
// reactions reported by the last successful step and are zero before the
// first step. time_seconds and dissipated_energy_j must stay finite and
// non-negative.
struct AtwoodState {
  Rectangle block_a;
  Rectangle block_b;
  Circle pulley;
  double time_seconds{};
  double dissipated_energy_j{};
  float tension_a_n{};
  float tension_b_n{};
  Vec2 pin_force_n{};
};

struct AtwoodDerived {
  double segment_length_a_m{};
  double segment_length_b_m{};
  // Signed error of (segment a + segment b) against the constrained sum.
  double rope_length_error_m{};
  // Signed rope speeds along each segment direction; positive lengthens
  // the segment (that block descends).
  float rope_speed_a_mps{};
  float rope_speed_b_mps{};
  float pulley_rim_speed_mps{};
  // |rope speed - rim feed| for each side; zero when no slip holds.
  float no_slip_error_a_mps{};
  float no_slip_error_b_mps{};
  float moment_of_inertia_kg_m2{};
  // Positive when block b descends and block a rises.
  float analytical_acceleration_m_s2{};
  float analytical_tension_a_n{};
  float analytical_tension_b_n{};
  // Pulley weight plus both measured tensions.
  float axle_load_n{};
  double translational_kinetic_energy_j{};
  double rotational_kinetic_energy_j{};
  // Relative to the initial heights; negative when net mass has descended.
  double potential_energy_j{};
  double mechanical_energy_j{};
  double accounted_energy_j{};
};

// Unbalanced reference: analytical acceleration 1.962 / 2.45 m/s^2.
AtwoodConfig MakeAtwoodReferenceConfig();
// Equal masses drifting at constant speed; stays airborne for over 60 s.
AtwoodConfig MakeAtwoodBalancedDriftConfig();
// Reference masses with damping, a nonzero initial speed (so the initial
// mechanical energy is positive), and a deeper side a so the rising block
// keeps clear of the pulley while the motion settles.
AtwoodConfig MakeAtwoodDampedConfig();

// Returns nullptr when valid, otherwise a user-facing message. Checks the
// documented ranges, the block-to-block and block-to-pulley clearances, and
// that both blocks fit inside the 100 m x 100 m integration area.
const char* GetAtwoodConfigError(const AtwoodConfig& config);

// Returns nullptr when config and state are valid. Stable public states
// keep the configured body definitions, no pending engine loads, exact
// anchor alignment, and both blocks inside the area margins.
const char* GetAtwoodStateError(const AtwoodConfig& config,
                                const AtwoodState& state);

// Returns (m_b - m_a) g / (m_a + m_b + I / R^2) in m/s^2 for a valid
// configuration and infinity otherwise.
float GetAtwoodAnalyticalAcceleration(const AtwoodConfig& config);

// Throws std::invalid_argument when config is invalid.
AtwoodState MakeInitialAtwoodState(const AtwoodConfig& config);

// Throws std::invalid_argument when config or state is invalid.
AtwoodDerived CalculateAtwoodDerived(const AtwoodConfig& config,
                                     const AtwoodState& state);

// Advances one fixed step through the Engine's constrained Update.
// delta_time must be finite and in (0, kAtwoodPhysicsStep]. Invalid input
// returns false without changing state; integration runs on a copy, so a
// propagated engine exception also leaves state unchanged.
bool StepAtwood(const AtwoodConfig& config, float delta_time,
                AtwoodState* state);

// Returns a terminal-condition message when a block has approached its
// anchor or the floor closely enough that the experiment should pause, and
// nullptr while the run may continue.
const char* GetAtwoodTerminalIssue(const AtwoodConfig& config,
                                   const AtwoodState& state);

// History times must be strictly increasing seconds. Returns the nearest
// sample, clamped to the recorded range. Empty history or a non-finite
// query returns nullptr.
const AtwoodState* FindAtwoodState(const std::vector<AtwoodState>& history,
                                   double time_seconds);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_ATWOOD_LAB_MODEL_H_
