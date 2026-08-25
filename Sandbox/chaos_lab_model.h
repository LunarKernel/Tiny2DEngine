#ifndef TINY2DENGINE_SANDBOX_CHAOS_LAB_MODEL_H_
#define TINY2DENGINE_SANDBOX_CHAOS_LAB_MODEL_H_

#include <string>
#include <vector>

#include "time_series.h"
#include "tiny2d_engine.h"

namespace tiny2d::sandbox {

inline constexpr float kChaosPhysicsStep = 1.0f / 480.0f;
// Each public step advances through this many internal engine substeps:
// velocity-projection constraint stepping loses energy to first order in
// dt under rotation, and 1/15360 s holds the 25-degree conservative
// reference below 1% energy drift over 60 s with measured margin.
inline constexpr int kChaosSubstepsPerStep = 32;
inline constexpr float kChaosAreaWidthM = 100.0f;
inline constexpr float kChaosAreaHeightM = 100.0f;
inline constexpr float kChaosPivotXM = 50.0f;
inline constexpr float kChaosPivotYM = 50.0f;
inline constexpr float kChaosMinimumMassKg = 0.01f;
inline constexpr float kChaosMaximumMassKg = 1000.0f;
inline constexpr float kChaosMinimumRodLengthM = 0.2f;
inline constexpr float kChaosMaximumRodLengthM = 20.0f;
inline constexpr float kChaosMaximumCombinedLengthM = 40.0f;
inline constexpr float kChaosMaximumSwingRadiusM = 41.0f;
inline constexpr float kChaosMinimumBobRadiusM = 0.02f;
inline constexpr float kChaosMaximumBobRadiusM = 2.0f;
inline constexpr float kChaosMaximumAngularSpeedRadS = 20.0f;
inline constexpr float kChaosMinimumGravityMps2 = 0.1f;
inline constexpr float kChaosMaximumGravityMps2 = 100.0f;
inline constexpr float kChaosMaximumDampingPerS = 100.0f;
inline constexpr float kChaosMaximumShadowOffsetRad = 0.1f;

// All values use SI units. +X points right and +Y points down. Angles are
// measured from straight down at each rod's parent point and are positive
// in the engine's clockwise screen sense: from hanging, a positive angle
// swings the bob toward -X, and positive angular velocity appears
// clockwise, matching every other lab. Bob positions follow
// p = parent + (-L sin theta, L cos theta). Masses are in [0.01, 1000] kg;
// each rod length is in [0.2, 20] m with L1 + L2 <= 40 and
// L1 + L2 + bob radius <= 41 so the full swing circle stays inside the
// 100 m x 100 m area; the bob radius is in [0.02, 2] m and below L2 / 2 so
// the two bobs of one run can never touch while the link rod holds;
// initial angles are in [-180, 180] degrees; initial angular velocities in
// [-20, 20] rad/s; gravity in [0.1, 100] m/s^2; linear damping in
// [0, 100] 1/s; the shadow offset (added to the shadow run's theta1) is in
// [0, 0.1] radians, and zero keeps the two runs identical.
struct ChaosConfig {
  float mass_1_kg{1.0f};
  float mass_2_kg{1.0f};
  float length_1_m{2.0f};
  float length_2_m{2.0f};
  float bob_radius_m{0.05f};
  float initial_angle_1_deg{120.0f};
  float initial_angle_2_deg{-10.0f};
  float initial_angular_velocity_1_rad_s{};
  float initial_angular_velocity_2_rad_s{};
  float gravity_m_s2{9.81f};
  float linear_damping_per_s{};
  float shadow_offset_rad{0.0001f};
};

// Primary and shadow double pendulums plus bookkeeping. Rod forces hold
// the primary run's reactions from the last successful step and are zero
// before the first step. time_seconds and dissipated_energy_j must stay
// finite and non-negative.
struct ChaosState {
  Circle bob_1;
  Circle bob_2;
  Circle shadow_bob_1;
  Circle shadow_bob_2;
  double time_seconds{};
  double dissipated_energy_j{};
  float anchor_rod_force_n{};
  float link_rod_force_n{};
};

struct ChaosDerived {
  // Primary run angles (radians, clockwise-positive from straight down)
  // and angular velocities from parent-relative velocities.
  double theta_1_rad{};
  double theta_2_rad{};
  double omega_1_rad_s{};
  double omega_2_rad_s{};
  double shadow_theta_1_rad{};
  double shadow_theta_2_rad{};
  // Signed rod length errors against the configured lengths.
  double rod_1_length_error_m{};
  double rod_2_length_error_m{};
  double kinetic_energy_j{};
  // Relative to the initial heights; negative when net mass has descended.
  double potential_energy_j{};
  double mechanical_energy_j{};
  double accounted_energy_j{};
  // Phase-space separation between the runs: principal-value angle
  // differences wrapped to (-pi, pi].
  double separation_rad{};
  // log10(separation / shadow offset); a floor value when the offset is
  // zero or the separation underflows.
  double separation_decades{};
  // Small-angle normal modes for the configured masses and lengths.
  double slow_mode_omega_rad_s{};
  double fast_mode_omega_rad_s{};
  double slow_mode_shape_ratio{};
  double fast_mode_shape_ratio{};
};

// Slow normal mode: theta2/theta1 = +sqrt(2) at 2 degrees, equal masses
// and lengths, no shadow offset. Period 2 pi / omega_slow ~= 3.707 s.
ChaosConfig MakeChaosSlowModeConfig();
// Large-amplitude conservative anchor: 25 / 0 degrees from rest, no
// shadow offset; the regime where the energy criteria bind (whip-heavy
// chaotic trajectories dissipate numerically; see the developer guide's
// documented limitation).
ChaosConfig MakeChaosLargeAmplitudeConfig();
// Chaotic reference: 120 / -10 degrees from rest with a 1e-4 rad shadow
// offset for the divergence demonstration.
ChaosConfig MakeChaosReferenceConfig();
// The large-amplitude configuration with damping and a nonzero initial
// angular velocity so the initial mechanical energy is positive.
ChaosConfig MakeChaosDampedConfig();

// Returns nullptr when valid, otherwise a user-facing message.
const char* GetChaosConfigError(const ChaosConfig& config);

// Fills the small-angle normal-mode frequencies [rad/s] and theta2/theta1
// shape ratios for a valid configuration. Returns false and leaves the
// outputs untouched when the configuration is invalid.
bool GetChaosSmallAngleModes(const ChaosConfig& config,
                             double* slow_omega_rad_s, double* fast_omega_rad_s,
                             double* slow_ratio, double* fast_ratio);

// Returns nullptr when config and state are valid. Stable public states
// keep the configured body definitions, no pending engine loads, rod
// lengths within a loose sanity bound, and parent-relative per-step travel
// below a quarter of each rod's length for both runs.
const char* GetChaosStateError(const ChaosConfig& config,
                               const ChaosState& state);

// Throws std::invalid_argument when config is invalid.
ChaosState MakeInitialChaosState(const ChaosConfig& config);

// Throws std::invalid_argument when config or state is invalid.
ChaosDerived CalculateChaosDerived(const ChaosConfig& config,
                                   const ChaosState& state);

// Advances one fixed public step as kChaosSubstepsPerStep internal engine
// substeps; the primary and shadow runs go through separate constrained
// Engine updates so the overlapping runs can never collide with each
// other. delta_time must be finite and in (0, kChaosPhysicsStep]. Invalid
// input returns false without changing state; integration runs on copies,
// so a propagated engine exception also leaves state unchanged.
bool StepChaos(const ChaosConfig& config, float delta_time, ChaosState* state);

// History times must be strictly increasing seconds. Returns the nearest
// sample, clamped to the recorded range. Empty history or a non-finite
// query returns nullptr.
const ChaosState* FindChaosState(const std::vector<ChaosState>& history,
                                 double time_seconds);

// Plottable ChaosLab quantities (ROADMAP section-12 time series).
enum class ChaosSeries {
  kTheta1,
  kTheta2,
  kOmega1,
  kOmega2,
  kMechanicalEnergy,
  kSeparationDecades,
  kAnchorRodForce,
  kLinkRodForce,
};

// Human-readable label with the SI unit. Throws std::invalid_argument
// for a value outside the enum.
const char* GetChaosSeriesLabel(ChaosSeries series);

// Extracts one series from the recorded history, keyed on each
// sample's stored time (rod forces come from the state samples - zero
// before the first step by definition - and the rest come from
// CalculateChaosDerived). Throws std::invalid_argument (producing no
// output) when the config or any sample is invalid.
std::vector<TimeSeriesPoint> ExtractChaosSeries(
    const ChaosConfig& config, const std::vector<ChaosState>& history,
    ChaosSeries series);

// Replay checkpoint parsed from a tiny2d-exp file: the save time, the
// complete per-bob dynamic state (four floats per bob - x, y, vx, vy -
// in bob1, bob2, shadow1, shadow2 order; bobs are fixed-rotation, so
// this is complete), the dissipated energy, and both rod forces.
struct ChaosCheckpoint {
  double time_s{};
  std::vector<float> bob_values;
  double dissipated_energy_j{};
  float anchor_rod_force_n{};
  float link_rod_force_n{};
};

// Writes a tiny2d-exp format 1 document capturing the configuration
// and the state's checkpoint values with round-trip formatting.
// Throws std::invalid_argument (producing no output) when config or
// state is invalid.
std::string SaveChaosExperiment(const ChaosConfig& config,
                                const ChaosState& state,
                                const std::string& product_version);

// Strict, failure-atomic load: parses text, requires model id
// "V19 ChaosLab" and format 1, every ChaosConfig field exactly once
// with no unknown keys, a valid configuration, and the checkpoint
// keys complete and in the fixed order. False leaves every output
// untouched and fills error. out_product_version (optional) receives
// the file's version string so the UI can warn when it differs from
// the running build's.
bool LoadChaosExperiment(const std::string& text, ChaosConfig* out_config,
                         ChaosCheckpoint* out_checkpoint,
                         std::string* out_product_version, std::string* error);

// Deterministic replay verification: re-runs the fixed-step
// simulation from the initial state until time_seconds exactly equals
// the checkpoint time (checked before the first step; bounded), then
// compares every checkpoint value exactly. Returns nullptr on success
// or a stable message naming the failure.
const char* ReplayChaosExperiment(const ChaosConfig& config,
                                  const ChaosCheckpoint& checkpoint);

// Builds a versioned CSV export (tiny2d-csv format 1): metadata comment
// lines carrying the product version, model id "V19 ChaosLab", every
// ChaosConfig field by name, and the caller's status summary, then one
// row per history sample. Columns are SI-suffixed (angles in radians,
// clockwise-positive from straight down); the time_s column is each
// sample's stored time_seconds, never a uniform index interval.
// dissipated_energy_j and the rod forces come from the sample itself;
// the remaining columns come from CalculateChaosDerived. Throws
// std::invalid_argument (producing no output) when the config or any
// history sample is invalid.
std::string BuildChaosCsv(const ChaosConfig& config,
                          const std::vector<ChaosState>& history,
                          const std::string& product_version,
                          const std::string& status);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_CHAOS_LAB_MODEL_H_
