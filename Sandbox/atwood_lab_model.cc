#include "atwood_lab_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tiny2d::sandbox {
namespace {

constexpr float kMaximumStepTravelFraction = 0.25f;

bool FiniteVec(Vec2 value) {
  return std::isfinite(value.x) && std::isfinite(value.y);
}

float AnchorAX(const AtwoodConfig& config) {
  return kAtwoodPulleyCenterXM - config.pulley_radius_m;
}

float AnchorBX(const AtwoodConfig& config) {
  return kAtwoodPulleyCenterXM + config.pulley_radius_m;
}

double MomentOfInertiaUnchecked(const AtwoodConfig& config) {
  const double factor =
      config.pulley_inertia == CircleInertiaModel::kSolidDisk ? 0.5 : 1.0;
  return factor * static_cast<double>(config.pulley_mass_kg) *
         config.pulley_radius_m * config.pulley_radius_m;
}

Rectangle MakeBlockUnchecked(const AtwoodConfig& config, bool side_a) {
  Rectangle block;
  block.mass = side_a ? config.mass_a_kg : config.mass_b_kg;
  block.position = {side_a ? AnchorAX(config) : AnchorBX(config),
                    kAtwoodPulleyCenterYM + (side_a ? config.hang_depth_a_m
                                                    : config.hang_depth_b_m)};
  // Positive rope speed lowers block b and raises block a.
  block.velocity = {0.0f, side_a ? -config.initial_rope_speed_mps
                                 : config.initial_rope_speed_mps};
  block.width = config.block_edge_m;
  block.height = config.block_edge_m;
  block.fixed_rotation = true;
  block.linear_damping_rate = config.linear_damping_per_s;
  block.angular_damping_rate = 0.0f;
  return block;
}

Circle MakePulleyUnchecked(const AtwoodConfig& config) {
  Circle pulley;
  pulley.mass = config.pulley_mass_kg;
  pulley.position = {kAtwoodPulleyCenterXM, kAtwoodPulleyCenterYM};
  pulley.radius = config.pulley_radius_m;
  pulley.inertia_model = config.pulley_inertia;
  // No-slip start: the rim feeds rope at the initial rope speed.
  pulley.angular_velocity =
      config.initial_rope_speed_mps / config.pulley_radius_m;
  pulley.angular_damping_rate = 0.0f;
  return pulley;
}

bool BlockIsInsideArea(const Rectangle& block) {
  const float half_edge = block.width * 0.5f;
  return block.position.x - half_edge >= kAtwoodBoundaryMarginM &&
         block.position.x + half_edge <=
             kAtwoodAreaWidthM - kAtwoodBoundaryMarginM &&
         block.position.y - half_edge >= kAtwoodBoundaryMarginM &&
         block.position.y + half_edge <=
             kAtwoodAreaHeightM - kAtwoodBoundaryMarginM;
}

double SegmentLength(Vec2 position, Vec2 anchor) {
  const double delta_x =
      static_cast<double>(position.x) - static_cast<double>(anchor.x);
  const double delta_y =
      static_cast<double>(position.y) - static_cast<double>(anchor.y);
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

double SegmentSpeed(Vec2 position, Vec2 anchor, Vec2 velocity) {
  const double length = SegmentLength(position, anchor);
  const double unit_x = (static_cast<double>(position.x) - anchor.x) / length;
  const double unit_y = (static_cast<double>(position.y) - anchor.y) / length;
  return unit_x * velocity.x + unit_y * velocity.y;
}

AtwoodDerived CalculateDerivedUnchecked(const AtwoodConfig& config,
                                        const AtwoodState& state) {
  AtwoodDerived derived;
  const Vec2 anchor_a{AnchorAX(config), kAtwoodPulleyCenterYM};
  const Vec2 anchor_b{AnchorBX(config), kAtwoodPulleyCenterYM};
  derived.segment_length_a_m = SegmentLength(state.block_a.position, anchor_a);
  derived.segment_length_b_m = SegmentLength(state.block_b.position, anchor_b);
  derived.rope_length_error_m =
      derived.segment_length_a_m + derived.segment_length_b_m -
      (static_cast<double>(config.hang_depth_a_m) + config.hang_depth_b_m);
  derived.rope_speed_a_mps = static_cast<float>(
      SegmentSpeed(state.block_a.position, anchor_a, state.block_a.velocity));
  derived.rope_speed_b_mps = static_cast<float>(
      SegmentSpeed(state.block_b.position, anchor_b, state.block_b.velocity));
  derived.pulley_rim_speed_mps =
      state.pulley.angular_velocity * config.pulley_radius_m;
  // Positive rotation feeds rope toward side b.
  derived.no_slip_error_a_mps =
      std::abs(derived.rope_speed_a_mps + derived.pulley_rim_speed_mps);
  derived.no_slip_error_b_mps =
      std::abs(derived.rope_speed_b_mps - derived.pulley_rim_speed_mps);

  const double inertia = MomentOfInertiaUnchecked(config);
  derived.moment_of_inertia_kg_m2 = static_cast<float>(inertia);
  const double effective_mass =
      static_cast<double>(config.mass_a_kg) + config.mass_b_kg +
      inertia / (static_cast<double>(config.pulley_radius_m) *
                 config.pulley_radius_m);
  const double acceleration =
      (static_cast<double>(config.mass_b_kg) - config.mass_a_kg) *
      config.gravity_m_s2 / effective_mass;
  derived.analytical_acceleration_m_s2 = static_cast<float>(acceleration);
  derived.analytical_tension_a_n = static_cast<float>(
      config.mass_a_kg * (config.gravity_m_s2 + acceleration));
  derived.analytical_tension_b_n = static_cast<float>(
      config.mass_b_kg * (config.gravity_m_s2 - acceleration));
  derived.axle_load_n = config.pulley_mass_kg * config.gravity_m_s2 +
                        state.tension_a_n + state.tension_b_n;

  const auto block_energy = [](const Rectangle& block) {
    return 0.5 * block.mass *
           (static_cast<double>(block.velocity.x) * block.velocity.x +
            static_cast<double>(block.velocity.y) * block.velocity.y);
  };
  derived.translational_kinetic_energy_j =
      block_energy(state.block_a) + block_energy(state.block_b);
  derived.rotational_kinetic_energy_j =
      0.5 * inertia * static_cast<double>(state.pulley.angular_velocity) *
      state.pulley.angular_velocity;
  const double initial_a_y =
      static_cast<double>(kAtwoodPulleyCenterYM) + config.hang_depth_a_m;
  const double initial_b_y =
      static_cast<double>(kAtwoodPulleyCenterYM) + config.hang_depth_b_m;
  // +Y points down, so descending (growing y) lowers potential energy.
  derived.potential_energy_j =
      -(static_cast<double>(config.mass_a_kg) * config.gravity_m_s2 *
            (state.block_a.position.y - initial_a_y) +
        static_cast<double>(config.mass_b_kg) * config.gravity_m_s2 *
            (state.block_b.position.y - initial_b_y));
  derived.mechanical_energy_j = derived.translational_kinetic_energy_j +
                                derived.rotational_kinetic_energy_j +
                                derived.potential_energy_j;
  derived.accounted_energy_j =
      derived.mechanical_energy_j + state.dissipated_energy_j;
  return derived;
}

bool HasExpectedBodyDefinitions(const AtwoodConfig& config,
                                const AtwoodState& state) {
  const auto block_matches = [&](const Rectangle& block, float mass,
                                 float anchor_x) {
    return block.mass == mass && block.width == config.block_edge_m &&
           block.height == config.block_edge_m && block.fixed_rotation &&
           block.angle == 0.0f && block.angular_velocity == 0.0f &&
           block.charge == 0.0f && block.applied_force.x == 0.0f &&
           block.applied_force.y == 0.0f && block.applied_torque == 0.0f &&
           block.linear_damping_rate == config.linear_damping_per_s &&
           block.angular_damping_rate == 0.0f && block.position.x == anchor_x;
  };
  const Circle& pulley = state.pulley;
  return block_matches(state.block_a, config.mass_a_kg, AnchorAX(config)) &&
         block_matches(state.block_b, config.mass_b_kg, AnchorBX(config)) &&
         pulley.mass == config.pulley_mass_kg &&
         pulley.radius == config.pulley_radius_m &&
         pulley.inertia_model == config.pulley_inertia &&
         !pulley.fixed_rotation && pulley.charge == 0.0f &&
         pulley.position.x == kAtwoodPulleyCenterXM &&
         pulley.position.y == kAtwoodPulleyCenterYM &&
         pulley.velocity.x == 0.0f && pulley.velocity.y == 0.0f &&
         pulley.applied_force.x == 0.0f && pulley.applied_force.y == 0.0f &&
         pulley.applied_torque == 0.0f && pulley.linear_damping_rate == 0.0f &&
         pulley.angular_damping_rate == 0.0f;
}

}  // namespace

AtwoodConfig MakeAtwoodReferenceConfig() { return {}; }

AtwoodConfig MakeAtwoodBalancedDriftConfig() {
  AtwoodConfig config;
  config.mass_a_kg = 1.0f;
  config.mass_b_kg = 1.0f;
  config.hang_depth_a_m = 30.0f;
  config.hang_depth_b_m = 30.0f;
  config.initial_rope_speed_mps = 0.02f;
  return config;
}

AtwoodConfig MakeAtwoodDampedConfig() {
  AtwoodConfig config;
  // The rising block a needs headroom: with the initial speed and the
  // residual acceleration it climbs roughly 3 m before the damped run
  // settles, which would reach the pulley from the default 3 m depth.
  config.hang_depth_a_m = 6.0f;
  config.initial_rope_speed_mps = 1.0f;
  config.linear_damping_per_s = 0.5f;
  return config;
}

const char* GetAtwoodConfigError(const AtwoodConfig& config) {
  const std::array<float, 10> values = {
      config.mass_a_kg,      config.mass_b_kg,
      config.pulley_mass_kg, config.pulley_radius_m,
      config.block_edge_m,   config.hang_depth_a_m,
      config.hang_depth_b_m, config.initial_rope_speed_mps,
      config.gravity_m_s2,   config.linear_damping_per_s,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All AtwoodLab inputs must be finite.";
  }
  if (config.pulley_inertia != CircleInertiaModel::kSolidDisk &&
      config.pulley_inertia != CircleInertiaModel::kHoop) {
    return "The pulley inertia model must be a solid disk or a hoop.";
  }
  if (config.mass_a_kg < kAtwoodMinimumMassKg ||
      config.mass_a_kg > kAtwoodMaximumMassKg ||
      config.mass_b_kg < kAtwoodMinimumMassKg ||
      config.mass_b_kg > kAtwoodMaximumMassKg ||
      config.pulley_mass_kg < kAtwoodMinimumMassKg ||
      config.pulley_mass_kg > kAtwoodMaximumMassKg) {
    return "Masses must be in [0.01, 1000] kg.";
  }
  if (config.pulley_radius_m < kAtwoodMinimumPulleyRadiusM ||
      config.pulley_radius_m > kAtwoodMaximumPulleyRadiusM) {
    return "The pulley radius must be in [0.05, 5] m.";
  }
  if (config.block_edge_m < kAtwoodMinimumBlockEdgeM ||
      config.block_edge_m > kAtwoodMaximumBlockEdgeM) {
    return "The block edge must be in [0.05, 5] m.";
  }
  if (config.block_edge_m >= 2.0f * config.pulley_radius_m) {
    return "The block edge must stay below the pulley diameter so the "
           "blocks cannot touch each other.";
  }
  if (config.hang_depth_a_m < kAtwoodMinimumHangDepthM ||
      config.hang_depth_a_m > kAtwoodMaximumHangDepthM ||
      config.hang_depth_b_m < kAtwoodMinimumHangDepthM ||
      config.hang_depth_b_m > kAtwoodMaximumHangDepthM) {
    return "Hang depths must be in [0.5, 70] m.";
  }
  const float minimum_depth = config.pulley_radius_m +
                              config.block_edge_m * 0.5f +
                              kAtwoodInitialPulleyClearanceM;
  if (config.hang_depth_a_m < minimum_depth ||
      config.hang_depth_b_m < minimum_depth) {
    return "Each block must start at least 0.15 m below the pulley disk.";
  }
  if (std::abs(config.initial_rope_speed_mps) > kAtwoodMaximumRopeSpeedMps) {
    return "The initial rope speed magnitude is at most 20 m/s.";
  }
  if (config.gravity_m_s2 < kAtwoodMinimumGravityMps2 ||
      config.gravity_m_s2 > kAtwoodMaximumGravityMps2) {
    return "Gravity must be in [0.1, 100] m/s^2.";
  }
  if (config.linear_damping_per_s < 0.0f ||
      config.linear_damping_per_s > kAtwoodMaximumDampingPerS) {
    return "The damping rate must be in [0, 100] 1/s.";
  }
  if (!BlockIsInsideArea(MakeBlockUnchecked(config, true)) ||
      !BlockIsInsideArea(MakeBlockUnchecked(config, false))) {
    return "Both blocks must start inside the AtwoodLab area.";
  }
  return nullptr;
}

const char* GetAtwoodStateError(const AtwoodConfig& config,
                                const AtwoodState& state) {
  if (GetAtwoodConfigError(config) != nullptr) {
    return "The AtwoodLab configuration is invalid.";
  }
  const std::array<const Rectangle*, 2> blocks = {&state.block_a,
                                                  &state.block_b};
  for (const Rectangle* block : blocks) {
    if (!std::isfinite(block->mass) || !FiniteVec(block->position) ||
        !FiniteVec(block->velocity) || !std::isfinite(block->angle) ||
        !std::isfinite(block->angular_velocity) ||
        !FiniteVec(block->applied_force) ||
        !std::isfinite(block->applied_torque)) {
      return "An AtwoodLab block contains NaN or infinity.";
    }
  }
  if (!std::isfinite(state.pulley.mass) || !FiniteVec(state.pulley.position) ||
      !FiniteVec(state.pulley.velocity) || !std::isfinite(state.pulley.angle) ||
      !std::isfinite(state.pulley.angular_velocity)) {
    return "The AtwoodLab pulley contains NaN or infinity.";
  }
  if (!std::isfinite(state.time_seconds) || state.time_seconds < 0.0 ||
      !std::isfinite(state.dissipated_energy_j) ||
      state.dissipated_energy_j < 0.0) {
    return "AtwoodLab time and dissipated energy must be non-negative.";
  }
  if (!std::isfinite(state.tension_a_n) || !std::isfinite(state.tension_b_n) ||
      !FiniteVec(state.pin_force_n)) {
    return "An AtwoodLab reaction value is not finite.";
  }
  if (!HasExpectedBodyDefinitions(config, state)) {
    return "The AtwoodLab body definitions or pending loads are invalid.";
  }
  if (!BlockIsInsideArea(state.block_a) || !BlockIsInsideArea(state.block_b)) {
    return "An AtwoodLab block reached the internal integration boundary.";
  }
  const float maximum_step_travel =
      config.block_edge_m * kMaximumStepTravelFraction;
  for (const Rectangle* block : blocks) {
    const double step_travel =
        std::hypot(block->velocity.x, block->velocity.y) * kAtwoodPhysicsStep +
        0.5 * config.gravity_m_s2 * kAtwoodPhysicsStep * kAtwoodPhysicsStep;
    if (!std::isfinite(step_travel) || step_travel > maximum_step_travel) {
      return "An AtwoodLab block moves too far in one fixed step.";
    }
  }
  return nullptr;
}

float GetAtwoodAnalyticalAcceleration(const AtwoodConfig& config) {
  if (GetAtwoodConfigError(config) != nullptr) {
    return std::numeric_limits<float>::infinity();
  }
  const double inertia = MomentOfInertiaUnchecked(config);
  const double effective_mass =
      static_cast<double>(config.mass_a_kg) + config.mass_b_kg +
      inertia / (static_cast<double>(config.pulley_radius_m) *
                 config.pulley_radius_m);
  return static_cast<float>(
      (static_cast<double>(config.mass_b_kg) - config.mass_a_kg) *
      config.gravity_m_s2 / effective_mass);
}

AtwoodState MakeInitialAtwoodState(const AtwoodConfig& config) {
  if (const char* error = GetAtwoodConfigError(config)) {
    throw std::invalid_argument(error);
  }
  AtwoodState state;
  state.block_a = MakeBlockUnchecked(config, true);
  state.block_b = MakeBlockUnchecked(config, false);
  state.pulley = MakePulleyUnchecked(config);
  if (const char* error = GetAtwoodStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return state;
}

AtwoodDerived CalculateAtwoodDerived(const AtwoodConfig& config,
                                     const AtwoodState& state) {
  if (const char* error = GetAtwoodStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepAtwood(const AtwoodConfig& config, float delta_time,
                AtwoodState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kAtwoodPhysicsStep ||
      GetAtwoodStateError(config, *state) != nullptr) {
    return false;
  }

  AtwoodState next = *state;

  // Exact damping-loss accounting relies on the Engine's documented
  // integration order: velocity = (v0 + g * dt) * exp(-rate * dt), gravity
  // first, damping second, constraints afterwards.
  // TestPerBodyExponentialDamping locks that contract in the engine suite.
  double damping_loss = 0.0;
  if (config.linear_damping_per_s > 0.0f) {
    const float factor = std::exp(-config.linear_damping_per_s * delta_time);
    const auto block_loss = [&](const Rectangle& block) {
      const double pre_damping_x = block.velocity.x;
      const double pre_damping_y =
          static_cast<double>(block.velocity.y) +
          static_cast<double>(config.gravity_m_s2) * delta_time;
      const double energy =
          0.5 * block.mass *
          (pre_damping_x * pre_damping_x + pre_damping_y * pre_damping_y);
      return energy * (1.0 - static_cast<double>(factor) * factor);
    };
    damping_loss = block_loss(next.block_a) + block_loss(next.block_b);
  }

  std::vector<Rectangle> rectangles{next.block_a, next.block_b};
  std::vector<Circle> circles{next.pulley};
  const std::vector<RevolutePin> pins = {
      {0, {kAtwoodPulleyCenterXM, kAtwoodPulleyCenterYM}}};
  PulleyRope rope;
  rope.body_a = {BodyKind::kRectangle, 0};
  rope.body_b = {BodyKind::kRectangle, 1};
  rope.pulley_circle_index = 0;
  rope.anchor_a = {AnchorAX(config), kAtwoodPulleyCenterYM};
  rope.anchor_b = {AnchorBX(config), kAtwoodPulleyCenterYM};
  rope.segment_length_sum = config.hang_depth_a_m + config.hang_depth_b_m;
  const std::vector<PulleyRope> ropes = {rope};

  ConstraintReactions reactions;
  Update(rectangles, circles, pins, ropes, delta_time, kAtwoodAreaWidthM,
         kAtwoodAreaHeightM, 0.0f, 0.0f, {}, config.gravity_m_s2, 0.0f, false,
         &reactions);

  next.block_a = rectangles[0];
  next.block_b = rectangles[1];
  next.pulley = circles[0];
  next.tension_a_n = reactions.rope_tensions[0].tension_a;
  next.tension_b_n = reactions.rope_tensions[0].tension_b;
  next.pin_force_n = reactions.pin_forces[0];

  const double next_time = next.time_seconds + delta_time;
  const double next_dissipated =
      next.dissipated_energy_j + std::max(0.0, damping_loss);
  if (!std::isfinite(next_time) || next_time <= next.time_seconds ||
      !std::isfinite(next_dissipated)) {
    return false;
  }
  next.time_seconds = next_time;
  next.dissipated_energy_j = next_dissipated;
  if (GetAtwoodStateError(config, next) != nullptr) {
    return false;
  }
  *state = next;
  return true;
}

const char* GetAtwoodTerminalIssue(const AtwoodConfig& config,
                                   const AtwoodState& state) {
  const float half_edge = config.block_edge_m * 0.5f;
  const float minimum_depth =
      config.pulley_radius_m + half_edge + kAtwoodTerminalPulleyClearanceM;
  const float floor_limit =
      kAtwoodAreaHeightM - kAtwoodTerminalFloorMarginM - half_edge;
  const float depth_a = state.block_a.position.y - kAtwoodPulleyCenterYM;
  const float depth_b = state.block_b.position.y - kAtwoodPulleyCenterYM;
  if (depth_a < minimum_depth) {
    return "Block a reached the pulley; rope wrap is not modeled, so the "
           "run pauses here.";
  }
  if (depth_b < minimum_depth) {
    return "Block b reached the pulley; rope wrap is not modeled, so the "
           "run pauses here.";
  }
  if (state.block_a.position.y > floor_limit) {
    return "Block a reached the floor; the experiment is complete.";
  }
  if (state.block_b.position.y > floor_limit) {
    return "Block b reached the floor; the experiment is complete.";
  }
  return nullptr;
}

const AtwoodState* FindAtwoodState(const std::vector<AtwoodState>& history,
                                   double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const AtwoodState& state, double target_time) {
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
