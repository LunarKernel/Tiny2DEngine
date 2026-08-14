#include "chaos_lab_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "csv_export.h"

namespace tiny2d::sandbox {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr float kDegreesToRadians = 3.14159265358979323846f / 180.0f;
constexpr float kMaximumStepTravelFraction = 0.25f;
// Loose sanity bound for rod lengths in a public state; the projection
// keeps the true error at float-rounding level.
constexpr double kRodLengthSanityM = 0.001;
constexpr double kSeparationDecadesFloor = -12.0;

bool FiniteVec(Vec2 value) {
  return std::isfinite(value.x) && std::isfinite(value.y);
}

Vec2 PivotPosition() { return {kChaosPivotXM, kChaosPivotYM}; }

// p = parent + (-L sin theta, L cos theta): positive theta is clockwise on
// screen, swinging the hanging bob toward -X.
Vec2 BobPosition(Vec2 parent, float length, float theta_radians) {
  return {parent.x - length * std::sin(theta_radians),
          parent.y + length * std::cos(theta_radians)};
}

// Velocity of a bob rotating about its parent at omega (clockwise
// positive): v = omega * perp(r) with perp(r) = (-r.y, r.x).
Vec2 BobVelocity(Vec2 parent, Vec2 position, float omega,
                 Vec2 parent_velocity) {
  const float radius_x = position.x - parent.x;
  const float radius_y = position.y - parent.y;
  return {parent_velocity.x - omega * radius_y,
          parent_velocity.y + omega * radius_x};
}

Circle MakeBobUnchecked(const ChaosConfig& config, bool first, Vec2 position,
                        Vec2 velocity) {
  Circle bob;
  bob.mass = first ? config.mass_1_kg : config.mass_2_kg;
  bob.position = position;
  bob.velocity = velocity;
  bob.radius = config.bob_radius_m;
  bob.fixed_rotation = true;
  bob.linear_damping_rate = config.linear_damping_per_s;
  bob.angular_damping_rate = 0.0f;
  return bob;
}

void MakeRunUnchecked(const ChaosConfig& config, float theta_1_radians,
                      Circle* bob_1, Circle* bob_2) {
  const Vec2 pivot = PivotPosition();
  const float theta_2_radians = config.initial_angle_2_deg * kDegreesToRadians;
  const Vec2 position_1 =
      BobPosition(pivot, config.length_1_m, theta_1_radians);
  const Vec2 position_2 =
      BobPosition(position_1, config.length_2_m, theta_2_radians);
  const Vec2 velocity_1 = BobVelocity(
      pivot, position_1, config.initial_angular_velocity_1_rad_s, {});
  const Vec2 velocity_2 =
      BobVelocity(position_1, position_2,
                  config.initial_angular_velocity_2_rad_s, velocity_1);
  *bob_1 = MakeBobUnchecked(config, true, position_1, velocity_1);
  *bob_2 = MakeBobUnchecked(config, false, position_2, velocity_2);
}

double WrapAngle(double value) {
  const double wrapped = std::remainder(value, 2.0 * kPi);
  // std::remainder yields [-pi, pi]; map -pi to +pi for (-pi, pi].
  return wrapped <= -kPi ? wrapped + 2.0 * kPi : wrapped;
}

double SegmentLength(Vec2 position, Vec2 parent) {
  const double delta_x =
      static_cast<double>(position.x) - static_cast<double>(parent.x);
  const double delta_y =
      static_cast<double>(position.y) - static_cast<double>(parent.y);
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

double ThetaOf(Vec2 position, Vec2 parent) {
  return std::atan2(-(static_cast<double>(position.x) - parent.x),
                    static_cast<double>(position.y) - parent.y);
}

// omega = Cross(r, v_rel) / |r|^2 with the engine's Cross convention; the
// caller passes the bob's velocity relative to its parent point.
double OmegaOf(Vec2 position, Vec2 parent, Vec2 relative_velocity) {
  const double radius_x = static_cast<double>(position.x) - parent.x;
  const double radius_y = static_cast<double>(position.y) - parent.y;
  const double length_squared = radius_x * radius_x + radius_y * radius_y;
  return (radius_x * relative_velocity.y - radius_y * relative_velocity.x) /
         length_squared;
}

void SmallAngleModes(const ChaosConfig& config, double* slow_omega,
                     double* fast_omega, double* slow_ratio,
                     double* fast_ratio) {
  const double mass_1 = config.mass_1_kg;
  const double mass_2 = config.mass_2_kg;
  const double length_1 = config.length_1_m;
  const double length_2 = config.length_2_m;
  const double gravity = config.gravity_m_s2;
  const double total_mass = mass_1 + mass_2;
  const double length_sum = length_1 + length_2;
  const double discriminant =
      total_mass * total_mass * length_sum * length_sum -
      4.0 * total_mass * mass_1 * length_1 * length_2;
  const double root = std::sqrt(std::max(discriminant, 0.0));
  const double denominator = 2.0 * mass_1 * length_1 * length_2;
  const double omega_squared_slow =
      gravity * (total_mass * length_sum - root) / denominator;
  const double omega_squared_fast =
      gravity * (total_mass * length_sum + root) / denominator;
  *slow_omega = std::sqrt(omega_squared_slow);
  *fast_omega = std::sqrt(omega_squared_fast);
  const auto shape_ratio = [&](double omega_squared) {
    return total_mass * (gravity - length_1 * omega_squared) /
           (mass_2 * length_2 * omega_squared);
  };
  *slow_ratio = shape_ratio(omega_squared_slow);
  *fast_ratio = shape_ratio(omega_squared_fast);
}

ChaosDerived CalculateDerivedUnchecked(const ChaosConfig& config,
                                       const ChaosState& state) {
  ChaosDerived derived;
  const Vec2 pivot = PivotPosition();
  derived.theta_1_rad = ThetaOf(state.bob_1.position, pivot);
  derived.theta_2_rad = ThetaOf(state.bob_2.position, state.bob_1.position);
  derived.omega_1_rad_s =
      OmegaOf(state.bob_1.position, pivot, state.bob_1.velocity);
  derived.omega_2_rad_s =
      OmegaOf(state.bob_2.position, state.bob_1.position,
              {state.bob_2.velocity.x - state.bob_1.velocity.x,
               state.bob_2.velocity.y - state.bob_1.velocity.y});
  derived.shadow_theta_1_rad = ThetaOf(state.shadow_bob_1.position, pivot);
  derived.shadow_theta_2_rad =
      ThetaOf(state.shadow_bob_2.position, state.shadow_bob_1.position);
  derived.rod_1_length_error_m =
      SegmentLength(state.bob_1.position, pivot) - config.length_1_m;
  derived.rod_2_length_error_m =
      SegmentLength(state.bob_2.position, state.bob_1.position) -
      config.length_2_m;

  const auto kinetic = [](const Circle& bob) {
    return 0.5 * bob.mass *
           (static_cast<double>(bob.velocity.x) * bob.velocity.x +
            static_cast<double>(bob.velocity.y) * bob.velocity.y);
  };
  derived.kinetic_energy_j = kinetic(state.bob_1) + kinetic(state.bob_2);

  const float theta_1_initial = config.initial_angle_1_deg * kDegreesToRadians;
  const float theta_2_initial = config.initial_angle_2_deg * kDegreesToRadians;
  const Vec2 initial_position_1 =
      BobPosition(pivot, config.length_1_m, theta_1_initial);
  const Vec2 initial_position_2 =
      BobPosition(initial_position_1, config.length_2_m, theta_2_initial);
  // +Y points down, so descending (growing y) lowers potential energy.
  derived.potential_energy_j =
      -(static_cast<double>(config.mass_1_kg) * config.gravity_m_s2 *
            (state.bob_1.position.y - initial_position_1.y) +
        static_cast<double>(config.mass_2_kg) * config.gravity_m_s2 *
            (state.bob_2.position.y - initial_position_2.y));
  derived.mechanical_energy_j =
      derived.kinetic_energy_j + derived.potential_energy_j;
  derived.accounted_energy_j =
      derived.mechanical_energy_j + state.dissipated_energy_j;

  const double delta_1 =
      WrapAngle(derived.shadow_theta_1_rad - derived.theta_1_rad);
  const double delta_2 =
      WrapAngle(derived.shadow_theta_2_rad - derived.theta_2_rad);
  derived.separation_rad = std::sqrt(delta_1 * delta_1 + delta_2 * delta_2);
  if (config.shadow_offset_rad > 0.0f && derived.separation_rad > 0.0) {
    derived.separation_decades =
        std::log10(derived.separation_rad / config.shadow_offset_rad);
  } else {
    derived.separation_decades = kSeparationDecadesFloor;
  }

  SmallAngleModes(
      config, &derived.slow_mode_omega_rad_s, &derived.fast_mode_omega_rad_s,
      &derived.slow_mode_shape_ratio, &derived.fast_mode_shape_ratio);
  return derived;
}

bool HasExpectedBobDefinition(const ChaosConfig& config, const Circle& bob,
                              float mass) {
  return bob.mass == mass && bob.radius == config.bob_radius_m &&
         bob.fixed_rotation && bob.angle == 0.0f &&
         bob.angular_velocity == 0.0f && bob.charge == 0.0f &&
         bob.applied_force.x == 0.0f && bob.applied_force.y == 0.0f &&
         bob.applied_torque == 0.0f &&
         bob.linear_damping_rate == config.linear_damping_per_s &&
         bob.angular_damping_rate == 0.0f;
}

const char* CheckRunState(const ChaosConfig& config, const Circle& bob_1,
                          const Circle& bob_2) {
  if (!std::isfinite(bob_1.mass) || !FiniteVec(bob_1.position) ||
      !FiniteVec(bob_1.velocity) || !std::isfinite(bob_2.mass) ||
      !FiniteVec(bob_2.position) || !FiniteVec(bob_2.velocity)) {
    return "A ChaosLab bob contains NaN or infinity.";
  }
  if (!HasExpectedBobDefinition(config, bob_1, config.mass_1_kg) ||
      !HasExpectedBobDefinition(config, bob_2, config.mass_2_kg)) {
    return "A ChaosLab bob definition or pending load is invalid.";
  }
  const double rod_1_length = SegmentLength(bob_1.position, PivotPosition());
  const double rod_2_length = SegmentLength(bob_2.position, bob_1.position);
  if (std::abs(rod_1_length - config.length_1_m) > kRodLengthSanityM ||
      std::abs(rod_2_length - config.length_2_m) > kRodLengthSanityM) {
    return "A ChaosLab rod length left its sanity bound.";
  }
  // Parent-relative travel guard: bob 2 is measured against bob 1, not the
  // world, so a fast-moving upper bob does not falsely trip it.
  const double travel_1 =
      std::hypot(bob_1.velocity.x, bob_1.velocity.y) * kChaosPhysicsStep;
  const double travel_2 =
      std::hypot(static_cast<double>(bob_2.velocity.x) - bob_1.velocity.x,
                 static_cast<double>(bob_2.velocity.y) - bob_1.velocity.y) *
      kChaosPhysicsStep;
  if (!std::isfinite(travel_1) ||
      travel_1 > kMaximumStepTravelFraction * config.length_1_m ||
      !std::isfinite(travel_2) ||
      travel_2 > kMaximumStepTravelFraction * config.length_2_m) {
    return "A ChaosLab bob moves too far in one fixed step.";
  }
  return nullptr;
}

ConstraintSet MakeRunConstraints(const ChaosConfig& config) {
  ConstraintSet constraints;
  AnchorRod anchor_rod;
  anchor_rod.body = {BodyKind::kCircle, 0};
  anchor_rod.world_anchor = PivotPosition();
  anchor_rod.length = config.length_1_m;
  constraints.anchor_rods = {anchor_rod};
  LinkRod link_rod;
  link_rod.body_a = {BodyKind::kCircle, 0};
  link_rod.body_b = {BodyKind::kCircle, 1};
  link_rod.length = config.length_2_m;
  constraints.link_rods = {link_rod};
  return constraints;
}

}  // namespace

ChaosConfig MakeChaosSlowModeConfig() {
  ChaosConfig config;
  config.initial_angle_1_deg = 2.0f;
  config.initial_angle_2_deg = 2.0f * 1.41421356f;
  config.shadow_offset_rad = 0.0f;
  return config;
}

ChaosConfig MakeChaosLargeAmplitudeConfig() {
  ChaosConfig config;
  config.initial_angle_1_deg = 25.0f;
  config.initial_angle_2_deg = 0.0f;
  config.shadow_offset_rad = 0.0f;
  return config;
}

ChaosConfig MakeChaosReferenceConfig() { return {}; }

ChaosConfig MakeChaosDampedConfig() {
  ChaosConfig config = MakeChaosLargeAmplitudeConfig();
  config.initial_angular_velocity_1_rad_s = 1.0f;
  config.linear_damping_per_s = 0.2f;
  return config;
}

const char* GetChaosConfigError(const ChaosConfig& config) {
  const std::array<float, 12> values = {
      config.mass_1_kg,
      config.mass_2_kg,
      config.length_1_m,
      config.length_2_m,
      config.bob_radius_m,
      config.initial_angle_1_deg,
      config.initial_angle_2_deg,
      config.initial_angular_velocity_1_rad_s,
      config.initial_angular_velocity_2_rad_s,
      config.gravity_m_s2,
      config.linear_damping_per_s,
      config.shadow_offset_rad,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All ChaosLab inputs must be finite.";
  }
  if (config.mass_1_kg < kChaosMinimumMassKg ||
      config.mass_1_kg > kChaosMaximumMassKg ||
      config.mass_2_kg < kChaosMinimumMassKg ||
      config.mass_2_kg > kChaosMaximumMassKg) {
    return "Masses must be in [0.01, 1000] kg.";
  }
  if (config.length_1_m < kChaosMinimumRodLengthM ||
      config.length_1_m > kChaosMaximumRodLengthM ||
      config.length_2_m < kChaosMinimumRodLengthM ||
      config.length_2_m > kChaosMaximumRodLengthM) {
    return "Rod lengths must be in [0.2, 20] m.";
  }
  if (config.length_1_m + config.length_2_m > kChaosMaximumCombinedLengthM) {
    return "The combined rod length must stay at or below 40 m.";
  }
  if (config.bob_radius_m < kChaosMinimumBobRadiusM ||
      config.bob_radius_m > kChaosMaximumBobRadiusM) {
    return "The bob radius must be in [0.02, 2] m.";
  }
  if (config.bob_radius_m >= config.length_2_m * 0.5f) {
    return "The bob radius must stay below half the lower rod length.";
  }
  if (config.length_1_m + config.length_2_m + config.bob_radius_m >
      kChaosMaximumSwingRadiusM) {
    return "The swing circle must fit inside the ChaosLab area.";
  }
  if (config.initial_angle_1_deg < -180.0f ||
      config.initial_angle_1_deg > 180.0f ||
      config.initial_angle_2_deg < -180.0f ||
      config.initial_angle_2_deg > 180.0f) {
    return "Initial angles must be in [-180, 180] degrees.";
  }
  if (std::abs(config.initial_angular_velocity_1_rad_s) >
          kChaosMaximumAngularSpeedRadS ||
      std::abs(config.initial_angular_velocity_2_rad_s) >
          kChaosMaximumAngularSpeedRadS) {
    return "Initial angular speeds are at most 20 rad/s.";
  }
  if (config.gravity_m_s2 < kChaosMinimumGravityMps2 ||
      config.gravity_m_s2 > kChaosMaximumGravityMps2) {
    return "Gravity must be in [0.1, 100] m/s^2.";
  }
  if (config.linear_damping_per_s < 0.0f ||
      config.linear_damping_per_s > kChaosMaximumDampingPerS) {
    return "The damping rate must be in [0, 100] 1/s.";
  }
  if (config.shadow_offset_rad < 0.0f ||
      config.shadow_offset_rad > kChaosMaximumShadowOffsetRad) {
    return "The shadow offset must be in [0, 0.1] radians.";
  }
  return nullptr;
}

bool GetChaosSmallAngleModes(const ChaosConfig& config,
                             double* slow_omega_rad_s, double* fast_omega_rad_s,
                             double* slow_ratio, double* fast_ratio) {
  if (GetChaosConfigError(config) != nullptr) {
    return false;
  }
  SmallAngleModes(config, slow_omega_rad_s, fast_omega_rad_s, slow_ratio,
                  fast_ratio);
  return true;
}

const char* GetChaosStateError(const ChaosConfig& config,
                               const ChaosState& state) {
  if (GetChaosConfigError(config) != nullptr) {
    return "The ChaosLab configuration is invalid.";
  }
  if (const char* error = CheckRunState(config, state.bob_1, state.bob_2)) {
    return error;
  }
  if (const char* error =
          CheckRunState(config, state.shadow_bob_1, state.shadow_bob_2)) {
    return error;
  }
  if (!std::isfinite(state.time_seconds) || state.time_seconds < 0.0 ||
      !std::isfinite(state.dissipated_energy_j) ||
      state.dissipated_energy_j < 0.0) {
    return "ChaosLab time and dissipated energy must be non-negative.";
  }
  if (!std::isfinite(state.anchor_rod_force_n) ||
      !std::isfinite(state.link_rod_force_n)) {
    return "A ChaosLab reaction value is not finite.";
  }
  return nullptr;
}

ChaosState MakeInitialChaosState(const ChaosConfig& config) {
  if (const char* error = GetChaosConfigError(config)) {
    throw std::invalid_argument(error);
  }
  ChaosState state;
  const float theta_1 = config.initial_angle_1_deg * kDegreesToRadians;
  MakeRunUnchecked(config, theta_1, &state.bob_1, &state.bob_2);
  MakeRunUnchecked(config, theta_1 + config.shadow_offset_rad,
                   &state.shadow_bob_1, &state.shadow_bob_2);
  if (const char* error = GetChaosStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return state;
}

ChaosDerived CalculateChaosDerived(const ChaosConfig& config,
                                   const ChaosState& state) {
  if (const char* error = GetChaosStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepChaos(const ChaosConfig& config, float delta_time, ChaosState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kChaosPhysicsStep ||
      GetChaosStateError(config, *state) != nullptr) {
    return false;
  }

  ChaosState next = *state;

  const ConstraintSet constraints = MakeRunConstraints(config);
  std::vector<Rectangle> no_rectangles;
  const float substep = delta_time / static_cast<float>(kChaosSubstepsPerStep);

  // Primary and shadow advance through separate engine calls: the shadow
  // starts within the offset of the primary, so a shared call would fire
  // circle-circle contacts between the overlapping runs.
  std::vector<Circle> primary{next.bob_1, next.bob_2};
  std::vector<Circle> shadow{next.shadow_bob_1, next.shadow_bob_2};
  ConstraintReactions reactions;
  double damping_loss = 0.0;
  for (int sub = 0; sub < kChaosSubstepsPerStep; ++sub) {
    if (config.linear_damping_per_s > 0.0f) {
      // Exact per-substep damping-loss accounting for the primary run,
      // relying on the Engine's documented order: gravity first, damping
      // second, constraints afterwards (TestPerBodyExponentialDamping
      // locks it).
      const float factor = std::exp(-config.linear_damping_per_s * substep);
      const auto bob_loss = [&](const Circle& bob) {
        const double pre_damping_x = bob.velocity.x;
        const double pre_damping_y =
            static_cast<double>(bob.velocity.y) +
            static_cast<double>(config.gravity_m_s2) * substep;
        const double energy =
            0.5 * bob.mass *
            (pre_damping_x * pre_damping_x + pre_damping_y * pre_damping_y);
        return energy * (1.0 - static_cast<double>(factor) * factor);
      };
      damping_loss += bob_loss(primary[0]) + bob_loss(primary[1]);
    }
    Update(no_rectangles, primary, constraints, substep, kChaosAreaWidthM,
           kChaosAreaHeightM, 0.0f, 0.0f, {}, config.gravity_m_s2, 0.0f, false,
           &reactions);
    Update(no_rectangles, shadow, constraints, substep, kChaosAreaWidthM,
           kChaosAreaHeightM, 0.0f, 0.0f, {}, config.gravity_m_s2, 0.0f, false,
           nullptr);
  }

  next.bob_1 = primary[0];
  next.bob_2 = primary[1];
  next.shadow_bob_1 = shadow[0];
  next.shadow_bob_2 = shadow[1];
  // Reactions from the last substep represent the end-of-step forces.
  next.anchor_rod_force_n = reactions.anchor_rod_forces[0];
  next.link_rod_force_n = reactions.link_rod_forces[0];

  const double next_time = next.time_seconds + delta_time;
  const double next_dissipated =
      next.dissipated_energy_j + std::max(0.0, damping_loss);
  if (!std::isfinite(next_time) || next_time <= next.time_seconds ||
      !std::isfinite(next_dissipated)) {
    return false;
  }
  next.time_seconds = next_time;
  next.dissipated_energy_j = next_dissipated;
  if (GetChaosStateError(config, next) != nullptr) {
    return false;
  }
  *state = next;
  return true;
}

const ChaosState* FindChaosState(const std::vector<ChaosState>& history,
                                 double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const ChaosState& state, double target_time) {
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

std::string BuildChaosCsv(const ChaosConfig& config,
                          const std::vector<ChaosState>& history,
                          const std::string& product_version,
                          const std::string& status) {
  if (const char* error = GetChaosConfigError(config)) {
    throw std::invalid_argument(error);
  }

  CsvMetadata metadata;
  metadata.model_id = "V19 ChaosLab";
  metadata.product_version = product_version;
  metadata.parameters = {
      {"mass_1_kg", CsvFloat(config.mass_1_kg)},
      {"mass_2_kg", CsvFloat(config.mass_2_kg)},
      {"length_1_m", CsvFloat(config.length_1_m)},
      {"length_2_m", CsvFloat(config.length_2_m)},
      {"bob_radius_m", CsvFloat(config.bob_radius_m)},
      {"initial_angle_1_deg", CsvFloat(config.initial_angle_1_deg)},
      {"initial_angle_2_deg", CsvFloat(config.initial_angle_2_deg)},
      {"initial_angular_velocity_1_rad_s",
       CsvFloat(config.initial_angular_velocity_1_rad_s)},
      {"initial_angular_velocity_2_rad_s",
       CsvFloat(config.initial_angular_velocity_2_rad_s)},
      {"gravity_m_s2", CsvFloat(config.gravity_m_s2)},
      {"linear_damping_per_s", CsvFloat(config.linear_damping_per_s)},
      {"shadow_offset_rad", CsvFloat(config.shadow_offset_rad)},
  };
  metadata.status = status;

  const std::vector<std::string> columns = {
      "time_s",
      "theta_1_rad",
      "theta_2_rad",
      "omega_1_rad_s",
      "omega_2_rad_s",
      "rod_1_length_error_m",
      "rod_2_length_error_m",
      "kinetic_energy_j",
      "potential_energy_j",
      "mechanical_energy_j",
      "dissipated_energy_j",
      "accounted_energy_j",
      "separation_rad",
      "separation_decades",
      "anchor_rod_force_n",
      "link_rod_force_n",
  };

  std::vector<std::vector<std::string>> rows;
  rows.reserve(history.size());
  for (const ChaosState& state : history) {
    const ChaosDerived derived = CalculateChaosDerived(config, state);
    rows.push_back({
        CsvDouble(state.time_seconds),
        CsvDouble(derived.theta_1_rad),
        CsvDouble(derived.theta_2_rad),
        CsvDouble(derived.omega_1_rad_s),
        CsvDouble(derived.omega_2_rad_s),
        CsvDouble(derived.rod_1_length_error_m),
        CsvDouble(derived.rod_2_length_error_m),
        CsvDouble(derived.kinetic_energy_j),
        CsvDouble(derived.potential_energy_j),
        CsvDouble(derived.mechanical_energy_j),
        CsvDouble(state.dissipated_energy_j),
        CsvDouble(derived.accounted_energy_j),
        CsvDouble(derived.separation_rad),
        CsvDouble(derived.separation_decades),
        CsvFloat(state.anchor_rod_force_n),
        CsvFloat(state.link_rod_force_n),
    });
  }
  return BuildCsv(metadata, columns, rows);
}

}  // namespace tiny2d::sandbox
