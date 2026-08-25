#include "stack_lab_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "csv_export.h"
#include "experiment_file.h"

namespace tiny2d::sandbox {
namespace {

constexpr float kBoundaryMarginM = 0.05f;
constexpr float kMaximumOffsetFraction = 0.4f;

bool FiniteVec(Vec2 value) {
  return std::isfinite(value.x) && std::isfinite(value.y);
}

float OffsetForBox(const StackConfig& config, int index) {
  const float amplitude = config.lateral_offset_m;
  return (index % 2 == 0) ? amplitude : -amplitude;
}

Rectangle MakeBoxUnchecked(const StackConfig& config, int index) {
  Rectangle box;
  box.mass = config.box_mass_kg;
  box.width = config.box_edge_m;
  box.height = config.box_edge_m;
  box.position = {kStackCenterXM + OffsetForBox(config, index),
                  kStackAreaHeightM - config.box_edge_m * 0.5f -
                      static_cast<float>(index) * config.box_edge_m};
  box.angular_damping_rate = 0.0f;
  return box;
}

SolverSettings MakeStackSolverSettings(const StackConfig& config) {
  SolverSettings settings;
  settings.iterations = kStackSolverIterations;
  settings.position_slop = std::min(0.002f, 0.002f * config.box_edge_m);
  settings.position_correction = kStackPositionCorrection;
  return settings;
}

double InitialPotential(const StackConfig& config) {
  double potential = 0.0;
  for (int i = 0; i < config.box_count; ++i) {
    potential += -static_cast<double>(config.gravity_m_s2) *
                 (kStackAreaHeightM - config.box_edge_m * 0.5 -
                  static_cast<double>(i) * config.box_edge_m) *
                 config.box_mass_kg;
  }
  return potential;
}

StackDerived CalculateDerivedUnchecked(const StackConfig& config,
                                       const StackState& state) {
  StackDerived derived;
  const int count = config.box_count;
  const double edge = config.box_edge_m;

  for (int i = 0; i < count; ++i) {
    const Rectangle& box = state.boxes[static_cast<std::size_t>(i)];
    if (i == 0) {
      derived.max_penetration_m = std::max(
          derived.max_penetration_m,
          static_cast<double>(box.position.y) + edge * 0.5 - kStackAreaHeightM);
    }
    if (i + 1 < count) {
      const Rectangle& upper = state.boxes[static_cast<std::size_t>(i + 1)];
      derived.max_penetration_m =
          std::max(derived.max_penetration_m,
                   (static_cast<double>(upper.position.y) + edge * 0.5) -
                       (static_cast<double>(box.position.y) - edge * 0.5));
    }
    derived.max_speed_mps = std::max(
        derived.max_speed_mps, std::hypot(static_cast<double>(box.velocity.x),
                                          static_cast<double>(box.velocity.y)));
    derived.max_angular_speed_rad_s =
        std::max(derived.max_angular_speed_rad_s,
                 static_cast<double>(std::abs(box.angular_velocity)));
    derived.max_tilt_rad = std::max(derived.max_tilt_rad,
                                    static_cast<double>(std::abs(box.angle)));
    if (!state.drift_reference_positions.empty()) {
      const Vec2 reference =
          state.drift_reference_positions[static_cast<std::size_t>(i)];
      derived.max_drift_m = std::max(
          derived.max_drift_m,
          std::hypot(static_cast<double>(box.position.x) - reference.x,
                     static_cast<double>(box.position.y) - reference.y));
    }
    derived.kinetic_energy_j +=
        0.5 * box.mass *
            (static_cast<double>(box.velocity.x) * box.velocity.x +
             static_cast<double>(box.velocity.y) * box.velocity.y) +
        0.5 * GetMomentOfInertia(box) *
            static_cast<double>(box.angular_velocity) * box.angular_velocity;
    derived.potential_energy_j +=
        -static_cast<double>(config.gravity_m_s2) * box.position.y * box.mass;
  }
  derived.potential_energy_j -= InitialPotential(config);
  derived.mechanical_energy_j =
      derived.kinetic_energy_j + derived.potential_energy_j;
  derived.max_penetration_fraction = derived.max_penetration_m / edge;

  double speed_sum = 0.0;
  for (const Rectangle& box : state.boxes) {
    speed_sum += std::hypot(static_cast<double>(box.velocity.x),
                            static_cast<double>(box.velocity.y));
  }
  derived.mean_speed_mps = speed_sum / count;
  // Measured height: floor level down to the top box's upper face.
  const Rectangle& top_box = state.boxes[static_cast<std::size_t>(count - 1)];
  const double measured_height =
      kStackAreaHeightM -
      (static_cast<double>(top_box.position.y) - edge * 0.5);
  derived.stack_height_error_m =
      measured_height - static_cast<double>(count) * edge;

  derived.interface_loads_n.assign(static_cast<std::size_t>(count - 1), 0.0);
  derived.analytical_loads_n.assign(static_cast<std::size_t>(count - 1), 0.0);
  for (int i = 0; i + 1 < count; ++i) {
    derived.analytical_loads_n[static_cast<std::size_t>(i)] =
        static_cast<double>(count - 1 - i) * config.box_mass_kg *
        config.gravity_m_s2;
  }
  for (const ContactCache::Entry& entry : state.cache.entries) {
    const int index_a = static_cast<int>((entry.key >> 29) & 0x1FFFFF);
    const int index_b = static_cast<int>((entry.key >> 8) & 0x1FFFFF);
    if (index_b == index_a + 1 && index_a < count - 1) {
      // Loads divide the cached impulses by the fixed kStackPhysicsStep:
      // exact for the shell's fixed-step stepping, approximate for a
      // direct consumer stepping a partial delta_time.
      derived.interface_loads_n[static_cast<std::size_t>(index_a)] +=
          entry.normal_impulse / kStackPhysicsStep;
    }
  }
  derived.cache_contact_count = state.cache.contact_count;
  derived.cache_warm_started_count = state.cache.warm_started_count;

  derived.penetration_ok = derived.max_penetration_fraction < 0.005;
  derived.resting_speed_ok =
      derived.max_speed_mps < 0.001 && derived.max_angular_speed_rad_s < 0.001;
  derived.drift_ok = state.drift_reference_positions.empty() ||
                     derived.max_drift_m < 0.001 * edge;
  // Energy must never exceed the initial value by more than 0.1% of the
  // energy scale. The runtime scale is max(1 J, |PE released|) so the
  // criterion stays meaningful after large settling; the reference test
  // additionally binds the strict 1 J floor where it matters.
  derived.energy_ok =
      derived.mechanical_energy_j <=
      0.001 * std::max(1.0, std::abs(derived.potential_energy_j));
  // The static height anchor: compression stays within the summed
  // per-interface penetration budget.
  derived.height_ok =
      std::abs(derived.stack_height_error_m) < 0.005 * edge * count;
  return derived;
}

bool HasExpectedBoxDefinitions(const StackConfig& config,
                               const StackState& state) {
  if (state.boxes.size() != static_cast<std::size_t>(config.box_count)) {
    return false;
  }
  for (const Rectangle& box : state.boxes) {
    if (box.mass != config.box_mass_kg || box.width != config.box_edge_m ||
        box.height != config.box_edge_m || box.fixed_rotation ||
        box.charge != 0.0f || box.applied_force.x != 0.0f ||
        box.applied_force.y != 0.0f || box.applied_torque != 0.0f ||
        box.linear_damping_rate != 0.0f || box.angular_damping_rate != 0.0f) {
      return false;
    }
  }
  return true;
}

}  // namespace

StackConfig MakeStackReferenceConfig() { return {}; }

StackConfig MakeStackOffsetConfig() {
  StackConfig config;
  config.lateral_offset_m = 0.15f;
  return config;
}

StackConfig MakeStackCollapseConfig() {
  StackConfig config;
  config.lateral_offset_m = 0.15f;
  config.friction = 0.05f;
  return config;
}

const char* GetStackConfigError(const StackConfig& config) {
  const std::array<float, 5> values = {config.box_edge_m, config.box_mass_kg,
                                       config.friction, config.gravity_m_s2,
                                       config.lateral_offset_m};
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All StackLab inputs must be finite.";
  }
  if (config.box_count < kStackMinimumBoxes ||
      config.box_count > kStackMaximumBoxes) {
    return "The box count must be in [2, 10].";
  }
  if (config.box_edge_m < kStackMinimumEdgeM ||
      config.box_edge_m > kStackMaximumEdgeM) {
    return "The box edge must be in [0.2, 5] m.";
  }
  if (config.box_mass_kg < kStackMinimumMassKg ||
      config.box_mass_kg > kStackMaximumMassKg) {
    return "The box mass must be in [0.01, 1000] kg.";
  }
  if (config.friction < 0.0f || config.friction > kStackMaximumFriction) {
    return "Friction must be in [0, 2].";
  }
  if (config.gravity_m_s2 < kStackMinimumGravityMps2 ||
      config.gravity_m_s2 > kStackMaximumGravityMps2) {
    return "Gravity must be in [0.1, 100] m/s^2.";
  }
  if (config.lateral_offset_m < 0.0f ||
      config.lateral_offset_m > kMaximumOffsetFraction * config.box_edge_m) {
    return "The lateral offset must be in [0, 0.4 * edge] m.";
  }
  const float stack_height =
      static_cast<float>(config.box_count) * config.box_edge_m;
  if (stack_height + kBoundaryMarginM > kStackAreaHeightM * 0.5f) {
    return "The stack must fit the lower half of the StackLab area.";
  }
  const float half_extent =
      config.box_edge_m * 0.5f + config.lateral_offset_m + kBoundaryMarginM;
  if (kStackCenterXM - half_extent < 0.0f ||
      kStackCenterXM + half_extent > kStackAreaWidthM) {
    return "The stack must fit the StackLab area laterally.";
  }
  return nullptr;
}

const char* GetStackStateError(const StackConfig& config,
                               const StackState& state) {
  if (GetStackConfigError(config) != nullptr) {
    return "The StackLab configuration is invalid.";
  }
  for (const Rectangle& box : state.boxes) {
    if (!std::isfinite(box.mass) || !FiniteVec(box.position) ||
        !FiniteVec(box.velocity) || !std::isfinite(box.angle) ||
        !std::isfinite(box.angular_velocity)) {
      return "A StackLab box contains NaN or infinity.";
    }
  }
  if (!HasExpectedBoxDefinitions(config, state)) {
    return "A StackLab box definition or pending load is invalid.";
  }
  if (!std::isfinite(state.time_seconds) || state.time_seconds < 0.0) {
    return "StackLab time must be finite and non-negative.";
  }
  if (!state.drift_reference_positions.empty() &&
      state.drift_reference_positions.size() != state.boxes.size()) {
    return "The StackLab drift snapshot does not match the box count.";
  }
  for (const ContactCache::Entry& entry : state.cache.entries) {
    if (!std::isfinite(entry.normal_impulse) || entry.normal_impulse < 0.0f ||
        !std::isfinite(entry.tangent_impulse)) {
      return "The StackLab contact cache contains an invalid entry.";
    }
  }
  return nullptr;
}

StackState MakeInitialStackState(const StackConfig& config) {
  if (const char* error = GetStackConfigError(config)) {
    throw std::invalid_argument(error);
  }
  StackState state;
  state.boxes.reserve(static_cast<std::size_t>(config.box_count));
  for (int i = 0; i < config.box_count; ++i) {
    state.boxes.push_back(MakeBoxUnchecked(config, i));
  }
  if (const char* error = GetStackStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return state;
}

StackDerived CalculateStackDerived(const StackConfig& config,
                                   const StackState& state) {
  if (const char* error = GetStackStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepStack(const StackConfig& config, float delta_time, StackState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kStackPhysicsStep ||
      GetStackStateError(config, *state) != nullptr) {
    return false;
  }

  StackState next = *state;
  std::vector<Circle> no_circles;
  Update(next.boxes, no_circles, ConstraintSet{},
         MakeStackSolverSettings(config), &next.cache, delta_time,
         kStackAreaWidthM, kStackAreaHeightM, 0.0f, config.friction, {},
         config.gravity_m_s2, 20.0f, false, nullptr);

  const double next_time = next.time_seconds + delta_time;
  if (!std::isfinite(next_time) || next_time <= next.time_seconds) {
    return false;
  }
  next.time_seconds = next_time;
  if (next.drift_reference_positions.empty() &&
      next.time_seconds >= kStackDriftSnapshotSeconds) {
    next.drift_reference_positions.reserve(next.boxes.size());
    for (const Rectangle& box : next.boxes) {
      next.drift_reference_positions.push_back(box.position);
    }
  }
  if (GetStackStateError(config, next) != nullptr) {
    return false;
  }
  *state = next;
  return true;
}

namespace {

constexpr const char* kStackModelId = "V20 StackLab";
// Per-box checkpoint key suffixes, fixed order.
constexpr const char* kStackBoxSuffixes[] = {
    "x_m", "y_m", "vx_mps", "vy_mps", "angle_rad", "omega_radps"};
constexpr int kStackBoxValueCount =
    static_cast<int>(sizeof(kStackBoxSuffixes) / sizeof(kStackBoxSuffixes[0]));

std::string StackBoxKey(int box, int component) {
  return "box" + std::to_string(box) + "_" + kStackBoxSuffixes[component];
}

}  // namespace

std::string SaveStackExperiment(const StackConfig& config,
                                const StackState& state,
                                const std::string& product_version) {
  if (const char* error = GetStackStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  ExperimentFile file;
  file.model_id = kStackModelId;
  file.product_version = product_version;
  file.parameters = {
      {"box_count", std::to_string(config.box_count)},
      {"box_edge_m", CsvFloat(config.box_edge_m)},
      {"box_mass_kg", CsvFloat(config.box_mass_kg)},
      {"friction", CsvFloat(config.friction)},
      {"gravity_m_s2", CsvFloat(config.gravity_m_s2)},
      {"lateral_offset_m", CsvFloat(config.lateral_offset_m)},
  };
  file.checkpoints.emplace_back("time_s", CsvDouble(state.time_seconds));
  for (int i = 0; i < config.box_count; ++i) {
    const Rectangle& box = state.boxes[static_cast<std::size_t>(i)];
    const float values[kStackBoxValueCount] = {
        box.position.x, box.position.y, box.velocity.x,
        box.velocity.y, box.angle,      box.angular_velocity};
    for (int c = 0; c < kStackBoxValueCount; ++c) {
      file.checkpoints.emplace_back(StackBoxKey(i, c), CsvFloat(values[c]));
    }
  }
  return WriteExperiment(file);
}

bool LoadStackExperiment(const std::string& text, StackConfig* out_config,
                         StackCheckpoint* out_checkpoint,
                         std::string* out_product_version, std::string* error) {
  const auto fail = [&](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (out_config == nullptr || out_checkpoint == nullptr) {
    return fail("Load targets are null.");
  }
  ExperimentFile file;
  std::string parse_error;
  if (!ParseExperiment(text, &file, &parse_error)) {
    return fail(parse_error);
  }
  if (file.model_id != kStackModelId) {
    return fail("The file is not a StackLab experiment.");
  }

  // Every config field exactly once, no unknown keys (parse already
  // rejected duplicates, so set-plus-count equality suffices).
  StackConfig config;
  bool seen[6] = {};
  for (const auto& [key, value] : file.parameters) {
    if (key == "box_count") {
      if (!ParseExperimentInt(value, &config.box_count)) {
        return fail("Malformed box_count value.");
      }
      seen[0] = true;
    } else if (key == "box_edge_m" &&
               ParseExperimentFloat(value, &config.box_edge_m)) {
      seen[1] = true;
    } else if (key == "box_mass_kg" &&
               ParseExperimentFloat(value, &config.box_mass_kg)) {
      seen[2] = true;
    } else if (key == "friction" &&
               ParseExperimentFloat(value, &config.friction)) {
      seen[3] = true;
    } else if (key == "gravity_m_s2" &&
               ParseExperimentFloat(value, &config.gravity_m_s2)) {
      seen[4] = true;
    } else if (key == "lateral_offset_m" &&
               ParseExperimentFloat(value, &config.lateral_offset_m)) {
      seen[5] = true;
    } else {
      return fail("Unknown or malformed param: " + key);
    }
  }
  for (const bool present : seen) {
    if (!present) {
      return fail("A StackLab param is missing.");
    }
  }
  if (const char* config_error = GetStackConfigError(config)) {
    return fail(config_error);
  }

  // Checkpoint keys complete and in the fixed order.
  StackCheckpoint checkpoint;
  const std::size_t expected_count =
      1 + static_cast<std::size_t>(config.box_count) * kStackBoxValueCount;
  if (file.checkpoints.size() != expected_count) {
    return fail("Checkpoint count does not match box_count.");
  }
  if (file.checkpoints[0].first != "time_s" ||
      !ParseExperimentDouble(file.checkpoints[0].second, &checkpoint.time_s) ||
      checkpoint.time_s < 0.0) {
    return fail("Malformed checkpoint time.");
  }
  checkpoint.values.reserve(expected_count - 1);
  std::size_t index = 1;
  for (int i = 0; i < config.box_count; ++i) {
    for (int c = 0; c < kStackBoxValueCount; ++c, ++index) {
      float value = 0.0f;
      if (file.checkpoints[index].first != StackBoxKey(i, c) ||
          !ParseExperimentFloat(file.checkpoints[index].second, &value)) {
        return fail("Malformed checkpoint: " + StackBoxKey(i, c));
      }
      checkpoint.values.push_back(value);
    }
  }

  *out_config = config;
  *out_checkpoint = std::move(checkpoint);
  if (out_product_version != nullptr) {
    *out_product_version = file.product_version;
  }
  return true;
}

const char* ReplayStackExperiment(const StackConfig& config,
                                  const StackCheckpoint& checkpoint) {
  static char message[64];
  if (GetStackConfigError(config) != nullptr) {
    return "Replay configuration is invalid.";
  }
  if (!std::isfinite(checkpoint.time_s) || checkpoint.time_s < 0.0) {
    return "Checkpoint time must be finite and non-negative.";
  }
  if (checkpoint.values.size() !=
      static_cast<std::size_t>(config.box_count) * kStackBoxValueCount) {
    return "Checkpoint value count mismatch.";
  }

  StackState state = MakeInitialStackState(config);
  const int maximum_steps =
      static_cast<int>(std::ceil(checkpoint.time_s / kStackPhysicsStep)) + 2;
  int steps = 0;
  // Equality is checked before the first step, so a t=0 checkpoint
  // verifies with zero steps; the exact double accumulation makes
  // equality the correct termination test for shell-recorded times.
  while (state.time_seconds != checkpoint.time_s) {
    if (steps++ >= maximum_steps) {
      return "Replay never reached the checkpoint time.";
    }
    if (!StepStack(config, kStackPhysicsStep, &state)) {
      return "Replay step rejected.";
    }
  }
  for (int i = 0; i < config.box_count; ++i) {
    const Rectangle& box = state.boxes[static_cast<std::size_t>(i)];
    const float actual[kStackBoxValueCount] = {
        box.position.x, box.position.y, box.velocity.x,
        box.velocity.y, box.angle,      box.angular_velocity};
    for (int c = 0; c < kStackBoxValueCount; ++c) {
      const float expected =
          checkpoint.values[static_cast<std::size_t>(i) * kStackBoxValueCount +
                            static_cast<std::size_t>(c)];
      if (actual[c] != expected) {
        std::snprintf(message, sizeof(message), "box%d_%s mismatch", i,
                      kStackBoxSuffixes[c]);
        return message;
      }
    }
  }
  return nullptr;
}

const char* GetStackSeriesLabel(StackSeries series) {
  switch (series) {
    case StackSeries::kMeanSpeed:
      return "mean speed (m/s)";
    case StackSeries::kMaxSpeed:
      return "max speed (m/s)";
    case StackSeries::kMechanicalEnergy:
      return "mechanical energy (J)";
    case StackSeries::kPenetrationFraction:
      return "penetration / edge";
    case StackSeries::kBottomInterfaceLoad:
      return "bottom interface load (N)";
    case StackSeries::kHeightError:
      return "height error (m)";
  }
  throw std::invalid_argument("Unknown StackLab series.");
}

std::vector<TimeSeriesPoint> ExtractStackSeries(
    const StackConfig& config, const std::vector<StackState>& history,
    StackSeries series) {
  GetStackSeriesLabel(series);  // Rejects out-of-enum values.
  if (const char* error = GetStackConfigError(config)) {
    throw std::invalid_argument(error);
  }
  std::vector<TimeSeriesPoint> points;
  points.reserve(history.size());
  for (const StackState& state : history) {
    const StackDerived derived = CalculateStackDerived(config, state);
    double value = 0.0;
    switch (series) {
      case StackSeries::kMeanSpeed:
        value = derived.mean_speed_mps;
        break;
      case StackSeries::kMaxSpeed:
        value = derived.max_speed_mps;
        break;
      case StackSeries::kMechanicalEnergy:
        value = derived.mechanical_energy_j;
        break;
      case StackSeries::kPenetrationFraction:
        value = derived.max_penetration_fraction;
        break;
      case StackSeries::kBottomInterfaceLoad:
        value = derived.interface_loads_n[0];
        break;
      case StackSeries::kHeightError:
        value = derived.stack_height_error_m;
        break;
    }
    points.push_back({state.time_seconds, value});
  }
  return points;
}

std::string BuildStackCsv(const StackConfig& config,
                          const std::vector<StackState>& history,
                          const std::string& product_version,
                          const std::string& status) {
  if (const char* error = GetStackConfigError(config)) {
    throw std::invalid_argument(error);
  }

  CsvMetadata metadata;
  metadata.model_id = "V20 StackLab";
  metadata.product_version = product_version;
  metadata.parameters = {
      {"box_count", std::to_string(config.box_count)},
      {"box_edge_m", CsvFloat(config.box_edge_m)},
      {"box_mass_kg", CsvFloat(config.box_mass_kg)},
      {"friction", CsvFloat(config.friction)},
      {"gravity_m_s2", CsvFloat(config.gravity_m_s2)},
      {"lateral_offset_m", CsvFloat(config.lateral_offset_m)},
  };
  metadata.status = status;

  std::vector<std::string> columns = {
      "time_s",
      "max_penetration_m",
      "max_penetration_fraction",
      "max_speed_mps",
      "mean_speed_mps",
      "max_angular_speed_rad_s",
      "max_tilt_rad",
      "max_drift_m",
      "kinetic_energy_j",
      "potential_energy_j",
      "mechanical_energy_j",
      "stack_height_error_m",
      "cache_contact_count",
      "cache_warm_started_count",
  };
  for (int i = 0; i + 1 < config.box_count; ++i) {
    columns.push_back("interface_load_" + std::to_string(i) + "_n");
  }

  std::vector<std::vector<std::string>> rows;
  rows.reserve(history.size());
  for (const StackState& state : history) {
    const StackDerived derived = CalculateStackDerived(config, state);
    std::vector<std::string> row = {
        CsvDouble(state.time_seconds),
        CsvDouble(derived.max_penetration_m),
        CsvDouble(derived.max_penetration_fraction),
        CsvDouble(derived.max_speed_mps),
        CsvDouble(derived.mean_speed_mps),
        CsvDouble(derived.max_angular_speed_rad_s),
        CsvDouble(derived.max_tilt_rad),
        CsvDouble(derived.max_drift_m),
        CsvDouble(derived.kinetic_energy_j),
        CsvDouble(derived.potential_energy_j),
        CsvDouble(derived.mechanical_energy_j),
        CsvDouble(derived.stack_height_error_m),
        std::to_string(derived.cache_contact_count),
        std::to_string(derived.cache_warm_started_count),
    };
    for (const double load : derived.interface_loads_n) {
      row.push_back(CsvDouble(load));
    }
    rows.push_back(std::move(row));
  }
  return BuildCsv(metadata, columns, rows);
}

const StackState* FindStackState(const std::vector<StackState>& history,
                                 double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const StackState& state, double target_time) {
                         return state.time_seconds < target_time;
                       });
  if (next == history.begin()) {
    return &history.front();
  }
  if (next == history.end()) {
    return &history.back();
  }
  const auto previous = next - 1;
  return time_seconds - previous->time_seconds <=
                 next->time_seconds - time_seconds
             ? &*previous
             : &*next;
}

}  // namespace tiny2d::sandbox
