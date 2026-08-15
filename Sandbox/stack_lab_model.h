#ifndef TINY2DENGINE_SANDBOX_STACK_LAB_MODEL_H_
#define TINY2DENGINE_SANDBOX_STACK_LAB_MODEL_H_

#include <string>
#include <vector>

#include "time_series.h"
#include "tiny2d_engine.h"

namespace tiny2d::sandbox {

inline constexpr float kStackPhysicsStep = 1.0f / 480.0f;
inline constexpr float kStackAreaWidthM = 100.0f;
inline constexpr float kStackAreaHeightM = 100.0f;
inline constexpr float kStackCenterXM = 50.0f;
inline constexpr int kStackMinimumBoxes = 2;
inline constexpr int kStackMaximumBoxes = 10;
inline constexpr float kStackMinimumEdgeM = 0.2f;
inline constexpr float kStackMaximumEdgeM = 5.0f;
inline constexpr float kStackMinimumMassKg = 0.01f;
inline constexpr float kStackMaximumMassKg = 1000.0f;
inline constexpr float kStackMaximumFriction = 2.0f;
inline constexpr float kStackMinimumGravityMps2 = 0.1f;
inline constexpr float kStackMaximumGravityMps2 = 100.0f;
// The lab's solver configuration: warm starting with these settings met
// every ROADMAP section 11 criterion on the reference stack (see the guide).
inline constexpr int kStackSolverIterations = 16;
inline constexpr float kStackPositionCorrection = 0.8f;
// The drift criterion compares against a snapshot taken at this time.
inline constexpr double kStackDriftSnapshotSeconds = 50.0;

// All values use SI units. +X points right and +Y points down; the stack
// builds upward from the floor at x = 50. Box count is in [2, 10]; the
// box edge is in [0.2, 5] m; the box mass in [0.01, 1000] kg; friction
// (applied as the world Coulomb coefficient) in [0, 2]; gravity in
// [0.1, 100] m/s^2; the per-box alternating lateral offset in
// [0, 0.4 * edge] m. The full stack (height plus offsets) must fit the
// area with margin.
struct StackConfig {
  int box_count{10};
  float box_edge_m{1.0f};
  float box_mass_kg{1.0f};
  float friction{0.6f};
  float gravity_m_s2{9.81f};
  float lateral_offset_m{};
};

// Boxes, the warm-start contact cache carried across steps, time, and the
// drift-reference snapshot (captured when time first passes 50 s; empty
// before that).
struct StackState {
  std::vector<Rectangle> boxes;
  ContactCache cache;
  double time_seconds{};
  std::vector<Vec2> drift_reference_positions;
};

struct StackDerived {
  // Worst box-box or box-floor overlap, absolute and vs the 0.5% budget.
  double max_penetration_m{};
  double max_penetration_fraction{};
  double max_speed_mps{};
  double mean_speed_mps{};
  double max_angular_speed_rad_s{};
  double max_tilt_rad{};
  // Measured stack height minus the ideal box_count * edge; a settled
  // stack compresses by roughly the summed interface penetrations.
  double stack_height_error_m{};
  // Worst drift from the 50 s snapshot; zero before the snapshot exists.
  double max_drift_m{};
  double kinetic_energy_j{};
  // Relative to the initial heights; negative when the stack has settled.
  double potential_energy_j{};
  double mechanical_energy_j{};
  // Measured per-interface loads from cached impulses (bottom first) and
  // their analytical (n - k) m g references.
  std::vector<double> interface_loads_n;
  std::vector<double> analytical_loads_n;
  int cache_contact_count{};
  int cache_warm_started_count{};
  // Live ROADMAP criteria states against their thresholds.
  bool penetration_ok{};
  bool resting_speed_ok{};
  bool drift_ok{};
  bool energy_ok{};
  bool height_ok{};
};

// Aligned ten-box reference: meets every section-11 criterion with margin.
StackConfig MakeStackReferenceConfig();
// Alternating 0.15 m offsets: stands without collapse; wobbles within the
// documented limitation.
StackConfig MakeStackOffsetConfig();
// Low friction with offsets: collapses; finiteness/determinism only.
StackConfig MakeStackCollapseConfig();

// Returns nullptr when valid, otherwise a user-facing message.
const char* GetStackConfigError(const StackConfig& config);

// Returns nullptr when config and state are valid. Stable public states
// keep the configured box definitions, no pending engine loads, and a
// structurally valid cache (validated by the engine on every step).
const char* GetStackStateError(const StackConfig& config,
                               const StackState& state);

// Throws std::invalid_argument when config is invalid.
StackState MakeInitialStackState(const StackConfig& config);

// Throws std::invalid_argument when config or state is invalid.
StackDerived CalculateStackDerived(const StackConfig& config,
                                   const StackState& state);

// Advances one fixed step through the Engine's warm-started full-control
// Update. delta_time must be finite and in (0, kStackPhysicsStep].
// Invalid input returns false without changing state; integration runs on
// a copy, so a propagated engine exception also leaves state unchanged.
bool StepStack(const StackConfig& config, float delta_time, StackState* state);

// History times must be strictly increasing seconds. Returns the nearest
// sample, clamped to the recorded range. Empty history or a non-finite
// query returns nullptr.
const StackState* FindStackState(const std::vector<StackState>& history,
                                 double time_seconds);

// Plottable StackLab quantities (ROADMAP section-12 time series).
enum class StackSeries {
  kMeanSpeed,
  kMaxSpeed,
  kMechanicalEnergy,
  kPenetrationFraction,
  kBottomInterfaceLoad,
  kHeightError,
};

// Human-readable label with the SI unit. Throws std::invalid_argument
// for a value outside the enum.
const char* GetStackSeriesLabel(StackSeries series);

// Extracts one series from the recorded history, keyed on each
// sample's stored time (values via CalculateStackDerived; the bottom
// interface load is interface_loads_n[0], which box_count >= 2
// guarantees exists). Throws std::invalid_argument (producing no
// output) when the config or any sample is invalid.
std::vector<TimeSeriesPoint> ExtractStackSeries(
    const StackConfig& config, const std::vector<StackState>& history,
    StackSeries series);

// Builds a versioned CSV export (tiny2d-csv format 1): metadata comment
// lines carrying the product version, model id "V20 StackLab", every
// StackConfig field by name, and the caller's status summary, then one
// row per history sample. Columns are SI-suffixed; the time_s column is
// each sample's stored time_seconds, never a uniform index interval.
// Per-interface load columns are interface_load_<k>_n, bottom first,
// box_count - 1 of them. Throws std::invalid_argument (producing no
// output) when the config or any history sample is invalid.
std::string BuildStackCsv(const StackConfig& config,
                          const std::vector<StackState>& history,
                          const std::string& product_version,
                          const std::string& status);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_STACK_LAB_MODEL_H_
