#ifndef TINY2DENGINE_SANDBOX_IMPACT_LAB_MODEL_H_
#define TINY2DENGINE_SANDBOX_IMPACT_LAB_MODEL_H_

#include <vector>

#include "tiny2d_engine.h"

namespace tiny2d::sandbox {

inline constexpr float kImpactLabPhysicsStep = 1.0f / 480.0f;
inline constexpr float kImpactLabAreaWidthM = 24.0f;
inline constexpr float kImpactLabAreaHeightM = 14.0f;
inline constexpr float kImpactLabCircleRadiusM = 0.1f;
inline constexpr float kImpactLabMinimumMassKg = 0.1f;
inline constexpr float kImpactLabMaximumMassKg = 10.0f;
inline constexpr float kImpactLabMinimumGrazingSpeedMps = 9.8f;
inline constexpr float kImpactLabMaximumGrazingSpeedMps = 10.0f;
inline constexpr float kImpactLabMinimumDiameterSkipSpeedMps = 210.0f;
inline constexpr float kImpactLabMaximumDiameterSkipSpeedMps = 500.0f;

enum class ImpactLabPreset {
  kSupportedGrazing,
  kDiameterSkip,
};

// All values use SI units. +X points right and +Y points down. The preset
// fixes circle radius, initial geometry, and velocity directions so every
// accepted configuration contains a swept collision that the legacy
// discrete path misses during one 1/480 s step. Mass is in [0.1, 10] kg,
// restitution is in [0, 1], and speed uses the preset-specific range above.
struct ImpactLabConfig {
  ImpactLabPreset preset{ImpactLabPreset::kSupportedGrazing};
  float mass_a_kg{1.0f};
  float mass_b_kg{1.0f};
  float speed_m_s{10.0f};
  float restitution{1.0f};
};

struct ImpactLabState {
  // Both lanes start from identical circles. discrete_circles uses the V16
  // endpoint-only path; ccd_circles explicitly enables circle-circle CCD.
  std::vector<Circle> discrete_circles;
  std::vector<Circle> ccd_circles;
  double time_seconds{};
};

struct ImpactLabDerived {
  double entry_time_seconds{};
  double exit_time_seconds{};
  Vec2 contact_normal;
  Vec2 expected_velocity_a_m_s;
  Vec2 expected_velocity_b_m_s;
  Vec2 expected_position_a_m;
  Vec2 expected_position_b_m;
  Vec2 initial_momentum_kg_m_s;
  double expected_kinetic_energy_j{};
  double ccd_position_error_m{};
  double ccd_velocity_error_m_s{};
  double ccd_momentum_error_kg_m_s{};
  double ccd_kinetic_energy_error_j{};
  double discrete_velocity_error_m_s{};
  double moving_distance_per_step_m{};
  double moving_distance_to_diameter_ratio{};
  bool discrete_tunneled{};
  bool ccd_resolved{};
};

ImpactLabConfig MakeSupportedGrazingImpactConfig();
ImpactLabConfig MakeDiameterSkipImpactConfig();

// Returns nullptr when valid, otherwise a stable string literal.
const char* GetImpactLabConfigError(const ImpactLabConfig& config);
const char* GetImpactLabStateError(const ImpactLabConfig& config,
                                   const ImpactLabState& state);

// Throws std::invalid_argument when config is invalid.
ImpactLabState MakeInitialImpactLabState(const ImpactLabConfig& config);

// Returns the swept-circle analytical reference and errors for the supplied
// state. Throws std::invalid_argument when config or state is invalid.
ImpactLabDerived CalculateImpactLabDerived(const ImpactLabConfig& config,
                                           const ImpactLabState& state);

// Advances one fixed step in two lanes using the production Engine mixed
// Update. delta_time is finite and in (0, kImpactLabPhysicsStep]. Invalid
// input returns false without changing state; Engine exceptions may
// propagate, but state remains unchanged because both lanes run on a copy.
bool StepImpactLab(const ImpactLabConfig& config, float delta_time,
                   ImpactLabState* state);

// History times must be strictly increasing seconds. Returns the nearest
// sample, clamped to the recorded range. Empty history or non-finite query
// returns nullptr.
const ImpactLabState* FindImpactLabState(
    const std::vector<ImpactLabState>& history, double time_seconds);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_IMPACT_LAB_MODEL_H_
