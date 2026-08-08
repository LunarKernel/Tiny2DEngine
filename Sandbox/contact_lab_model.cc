#include "contact_lab_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace tiny2d::sandbox {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;
constexpr Vec2 kSurfacePositionM{12.0f, 11.5f};
constexpr float kSurfaceWidthM = 22.0f;
constexpr float kSurfaceHeightM = 1.0f;
constexpr float kSurfaceTopM = kSurfacePositionM.y - kSurfaceHeightM * 0.5f;

bool IsKnownMode(ContactLabMode mode) {
  switch (mode) {
    case ContactLabMode::kElasticImpact:
    case ContactLabMode::kRollingContact:
      return true;
  }
  return false;
}

bool IsKnownInertiaModel(CircleInertiaModel model) {
  switch (model) {
    case CircleInertiaModel::kSolidDisk:
    case CircleInertiaModel::kHoop:
      return true;
  }
  return false;
}

bool IsValidMaterial(const CollisionMaterial& material) {
  return std::isfinite(material.restitution) &&
         std::isfinite(material.static_friction) &&
         std::isfinite(material.kinetic_friction) &&
         material.restitution >= 0.0f && material.restitution <= 1.0f &&
         material.static_friction >= 0.0f &&
         material.static_friction <= kContactLabMaximumFrictionCoefficient &&
         material.kinetic_friction >= 0.0f &&
         material.kinetic_friction <= material.static_friction;
}

bool SameMaterial(const CollisionMaterial& a, const CollisionMaterial& b) {
  return a.restitution == b.restitution &&
         a.static_friction == b.static_friction &&
         a.kinetic_friction == b.kinetic_friction;
}

const char* GetCircleConfigError(const ContactLabCircleConfig& circle,
                                 bool may_be_static) {
  const std::array values = {
      circle.mass_kg,
      circle.radius_m,
      circle.initial_position_m.x,
      circle.initial_position_m.y,
      circle.initial_velocity_m_s.x,
      circle.initial_velocity_m_s.y,
      circle.initial_angle_degrees,
      circle.initial_angular_velocity_rad_s,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All circle inputs must be finite.";
  }
  if (!IsKnownInertiaModel(circle.inertia_model)) {
    return "Circle inertia model is invalid.";
  }
  if (circle.mass_kg < kContactLabMinimumMassKg ||
      circle.mass_kg > kContactLabMaximumMassKg) {
    return "Circle mass must be in [0.01, 1000] kg.";
  }
  if (circle.radius_m < kContactLabMinimumRadiusM ||
      circle.radius_m > kContactLabMaximumRadiusM) {
    return "Circle radius must be in [0.1, 2] m.";
  }
  if (circle.initial_position_m.x < circle.radius_m ||
      circle.initial_position_m.x > kContactLabAreaWidthM - circle.radius_m ||
      circle.initial_position_m.y < circle.radius_m ||
      circle.initial_position_m.y > kContactLabAreaHeightM - circle.radius_m) {
    return "Each initial circle must fit inside the ContactLab area.";
  }
  if (std::abs(circle.initial_velocity_m_s.x) >
          kContactLabMaximumInitialSpeedMps ||
      std::abs(circle.initial_velocity_m_s.y) >
          kContactLabMaximumInitialSpeedMps ||
      std::abs(circle.initial_angular_velocity_rad_s) >
          kContactLabMaximumInitialAngularSpeedRadS ||
      circle.initial_angle_degrees < -180.0f ||
      circle.initial_angle_degrees > 180.0f) {
    return "Initial circle velocity, angular velocity, or angle is outside "
           "the supported range.";
  }
  if (!may_be_static && circle.is_static) {
    return "Circle A must be dynamic.";
  }
  if (circle.is_static && (circle.initial_velocity_m_s.x != 0.0f ||
                           circle.initial_velocity_m_s.y != 0.0f ||
                           circle.initial_angular_velocity_rad_s != 0.0f)) {
    return "A static circle must have zero initial velocity.";
  }
  if (!IsValidMaterial(circle.material)) {
    return "Circle material requires restitution in [0, 1] and 0 <= kinetic "
           "friction <= static friction <= 5.";
  }
  return nullptr;
}

Circle MakeCircle(const ContactLabCircleConfig& config) {
  Circle circle;
  circle.mass = config.is_static ? 0.0f : config.mass_kg;
  circle.position = config.initial_position_m;
  circle.velocity = config.initial_velocity_m_s;
  circle.angle = config.initial_angle_degrees * kDegreesToRadians;
  circle.angular_velocity = config.initial_angular_velocity_rad_s;
  circle.radius = config.radius_m;
  circle.inertia_model = config.inertia_model;
  circle.fixed_rotation = false;
  circle.charge = 0.0f;
  circle.applied_force = {};
  circle.applied_torque = 0.0f;
  circle.linear_damping_rate = 0.0f;
  circle.angular_damping_rate = 0.0f;
  circle.material = config.material;
  return circle;
}

Rectangle MakeSurface(const ContactLabConfig& config) {
  Rectangle surface;
  surface.mass = 0.0f;
  surface.position = kSurfacePositionM;
  surface.velocity = {};
  surface.angle = 0.0f;
  surface.angular_velocity = 0.0f;
  surface.width = kSurfaceWidthM;
  surface.height = kSurfaceHeightM;
  surface.fixed_rotation = true;
  surface.charge = 0.0f;
  surface.applied_force = {};
  surface.applied_torque = 0.0f;
  surface.linear_damping_rate = 0.0f;
  surface.angular_damping_rate = 0.0f;
  surface.material = config.surface_material;
  return surface;
}

bool CircleMatchesConfig(const Circle& circle,
                         const ContactLabCircleConfig& config) {
  return circle.mass == (config.is_static ? 0.0f : config.mass_kg) &&
         circle.radius == config.radius_m &&
         circle.inertia_model == config.inertia_model &&
         !circle.fixed_rotation && circle.charge == 0.0f &&
         circle.applied_force.x == 0.0f && circle.applied_force.y == 0.0f &&
         circle.applied_torque == 0.0f && circle.linear_damping_rate == 0.0f &&
         circle.angular_damping_rate == 0.0f &&
         SameMaterial(circle.material, config.material);
}

bool IsExpectedSurface(const Rectangle& surface,
                       const ContactLabConfig& config) {
  return surface.mass == 0.0f && surface.position.x == kSurfacePositionM.x &&
         surface.position.y == kSurfacePositionM.y &&
         surface.velocity.x == 0.0f && surface.velocity.y == 0.0f &&
         surface.angle == 0.0f && surface.angular_velocity == 0.0f &&
         surface.width == kSurfaceWidthM && surface.height == kSurfaceHeightM &&
         surface.fixed_rotation && surface.charge == 0.0f &&
         surface.applied_force.x == 0.0f && surface.applied_force.y == 0.0f &&
         surface.applied_torque == 0.0f &&
         surface.linear_damping_rate == 0.0f &&
         surface.angular_damping_rate == 0.0f &&
         SameMaterial(surface.material, config.surface_material);
}

bool IsFinite(const Circle& circle) {
  const std::array values = {
      circle.mass,
      circle.position.x,
      circle.position.y,
      circle.velocity.x,
      circle.velocity.y,
      circle.angle,
      circle.angular_velocity,
      circle.radius,
      circle.charge,
      circle.applied_force.x,
      circle.applied_force.y,
      circle.applied_torque,
      circle.linear_damping_rate,
      circle.angular_damping_rate,
  };
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); }) &&
         IsValidMaterial(circle.material);
}

bool CircleFitsArea(const Circle& circle) {
  return circle.position.x >= circle.radius &&
         circle.position.x <= kContactLabAreaWidthM - circle.radius &&
         circle.position.y >= circle.radius &&
         circle.position.y <= kContactLabAreaHeightM - circle.radius;
}

ContactLabDerived CalculateDerivedUnchecked(const ContactLabConfig& config,
                                            const ContactLabState& state) {
  ContactLabDerived derived;
  double momentum_x = 0.0;
  double momentum_y = 0.0;
  for (const Circle& circle : state.circles) {
    if (circle.mass == 0.0f) {
      continue;
    }
    const double momentum_circle_x =
        static_cast<double>(circle.mass) * circle.velocity.x;
    const double momentum_circle_y =
        static_cast<double>(circle.mass) * circle.velocity.y;
    momentum_x += momentum_circle_x;
    momentum_y += momentum_circle_y;
    derived.orbital_angular_momentum_kg_m2_s +=
        static_cast<double>(circle.position.x) * momentum_circle_y -
        static_cast<double>(circle.position.y) * momentum_circle_x;
    const double inertia = GetMomentOfInertia(circle);
    derived.spin_angular_momentum_kg_m2_s += inertia * circle.angular_velocity;
    derived.translational_kinetic_energy_j +=
        0.5 * circle.mass *
        (static_cast<double>(circle.velocity.x) * circle.velocity.x +
         static_cast<double>(circle.velocity.y) * circle.velocity.y);
    derived.rotational_kinetic_energy_j +=
        0.5 * inertia * circle.angular_velocity * circle.angular_velocity;
  }
  derived.total_linear_momentum_kg_m_s = {static_cast<float>(momentum_x),
                                          static_cast<float>(momentum_y)};
  derived.total_angular_momentum_kg_m2_s =
      derived.orbital_angular_momentum_kg_m2_s +
      derived.spin_angular_momentum_kg_m2_s;
  derived.total_kinetic_energy_j = derived.translational_kinetic_energy_j +
                                   derived.rotational_kinetic_energy_j;
  if (config.mode == ContactLabMode::kRollingContact) {
    const Circle& circle = state.circles.front();
    derived.rolling_slip_m_s =
        circle.velocity.x - circle.radius * circle.angular_velocity;
  }
  return derived;
}

bool IsDerivedFinite(const ContactLabDerived& derived) {
  const std::array values = {
      static_cast<double>(derived.total_linear_momentum_kg_m_s.x),
      static_cast<double>(derived.total_linear_momentum_kg_m_s.y),
      derived.orbital_angular_momentum_kg_m2_s,
      derived.spin_angular_momentum_kg_m2_s,
      derived.total_angular_momentum_kg_m2_s,
      derived.translational_kinetic_energy_j,
      derived.rotational_kinetic_energy_j,
      derived.total_kinetic_energy_j,
      static_cast<double>(derived.rolling_slip_m_s),
  };
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

ContactLabConfig MakeElasticImpactConfig() { return {}; }

ContactLabConfig MakeRollingSolidDiskConfig() {
  ContactLabConfig config;
  config.mode = ContactLabMode::kRollingContact;
  config.circle_a.initial_position_m = {4.0f, 10.5f};
  config.circle_a.initial_velocity_m_s = {2.0f, 0.0f};
  config.circle_a.inertia_model = CircleInertiaModel::kSolidDisk;
  config.circle_a.material = {0.0f, 0.6f, 0.4f};
  config.gravity_m_s2 = 9.8f;
  return config;
}

ContactLabConfig MakeRollingHoopConfig() {
  ContactLabConfig config = MakeRollingSolidDiskConfig();
  config.circle_a.inertia_model = CircleInertiaModel::kHoop;
  return config;
}

const char* GetContactLabConfigError(const ContactLabConfig& config) {
  if (!IsKnownMode(config.mode)) {
    return "ContactLab mode is invalid.";
  }
  if (!std::isfinite(config.gravity_m_s2) || config.gravity_m_s2 < 0.0f ||
      config.gravity_m_s2 > kContactLabMaximumGravityMps2) {
    return "Gravity must be in [0, 20] m/s^2.";
  }
  if (const char* error = GetCircleConfigError(config.circle_a, false)) {
    return error;
  }

  if (config.mode == ContactLabMode::kElasticImpact) {
    if (const char* error = GetCircleConfigError(config.circle_b, true)) {
      return error;
    }
    const double dx =
        static_cast<double>(config.circle_b.initial_position_m.x) -
        config.circle_a.initial_position_m.x;
    const double dy =
        static_cast<double>(config.circle_b.initial_position_m.y) -
        config.circle_a.initial_position_m.y;
    const double minimum_distance =
        static_cast<double>(config.circle_a.radius_m) +
        config.circle_b.radius_m;
    if (dx * dx + dy * dy < minimum_distance * minimum_distance) {
      return "Impact circles must not overlap initially.";
    }
  } else {
    if (!IsValidMaterial(config.surface_material)) {
      return "Surface material requires restitution in [0, 1] and 0 <= "
             "kinetic friction <= static friction <= 5.";
    }
    if (config.circle_a.initial_position_m.y + config.circle_a.radius_m >
        kSurfaceTopM) {
      return "The rolling circle must start above or touching the surface.";
    }
  }
  return nullptr;
}

const char* GetContactLabStateError(const ContactLabConfig& config,
                                    const ContactLabState& state) {
  if (GetContactLabConfigError(config) != nullptr) {
    return "The ContactLab configuration is invalid.";
  }
  if (!std::isfinite(state.time_seconds) || state.time_seconds < 0.0) {
    return "ContactLab time must be finite and non-negative.";
  }
  const std::size_t expected_circle_count =
      config.mode == ContactLabMode::kElasticImpact ? 2U : 1U;
  const std::size_t expected_rectangle_count =
      config.mode == ContactLabMode::kRollingContact ? 1U : 0U;
  if (state.circles.size() != expected_circle_count ||
      state.rectangles.size() != expected_rectangle_count) {
    return "ContactLab state has the wrong body count.";
  }
  if (!CircleMatchesConfig(state.circles[0], config.circle_a) ||
      (expected_circle_count == 2U &&
       !CircleMatchesConfig(state.circles[1], config.circle_b))) {
    return "ContactLab circle definition or pending load is invalid.";
  }
  for (const Circle& circle : state.circles) {
    if (!IsFinite(circle) || !CircleFitsArea(circle)) {
      return "A ContactLab circle contains an invalid or out-of-area state.";
    }
    try {
      static_cast<void>(GetMomentOfInertia(circle));
    } catch (const std::invalid_argument&) {
      return "A ContactLab circle has invalid inertia.";
    }
  }
  if (expected_rectangle_count == 1U &&
      !IsExpectedSurface(state.rectangles.front(), config)) {
    return "The ContactLab rolling surface is invalid.";
  }
  const ContactLabDerived derived = CalculateDerivedUnchecked(config, state);
  if (!IsDerivedFinite(derived)) {
    return "A ContactLab telemetry value is not finite.";
  }
  return nullptr;
}

ContactLabState MakeInitialContactLabState(const ContactLabConfig& config) {
  if (const char* error = GetContactLabConfigError(config)) {
    throw std::invalid_argument(error);
  }
  ContactLabState state;
  state.circles.push_back(MakeCircle(config.circle_a));
  if (config.mode == ContactLabMode::kElasticImpact) {
    state.circles.push_back(MakeCircle(config.circle_b));
  } else {
    state.rectangles.push_back(MakeSurface(config));
  }
  if (const char* error = GetContactLabStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return state;
}

ContactLabDerived CalculateContactLabDerived(const ContactLabConfig& config,
                                             const ContactLabState& state) {
  if (const char* error = GetContactLabStateError(config, state)) {
    throw std::invalid_argument(error);
  }
  return CalculateDerivedUnchecked(config, state);
}

bool StepContactLab(const ContactLabConfig& config, float delta_time,
                    ContactLabState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f ||
      delta_time > kContactLabPhysicsStep ||
      GetContactLabStateError(config, *state) != nullptr) {
    return false;
  }

  ContactLabState next = *state;
  const float gravity = config.mode == ContactLabMode::kRollingContact
                            ? config.gravity_m_s2
                            : 0.0f;
  Update(next.rectangles, next.circles, delta_time, kContactLabAreaWidthM,
         kContactLabAreaHeightM, 0.0f, 0.0f, {}, gravity,
         kContactLabRestitutionSpeedThresholdMps);
  const double next_time = next.time_seconds + delta_time;
  if (!std::isfinite(next_time) || next_time <= next.time_seconds) {
    return false;
  }
  next.time_seconds = next_time;
  if (GetContactLabStateError(config, next) != nullptr) {
    return false;
  }
  *state = std::move(next);
  return true;
}

const ContactLabState* FindContactLabState(
    const std::vector<ContactLabState>& history, double time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const ContactLabState& state, double target_time) {
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
