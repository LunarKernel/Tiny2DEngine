#include "tiny2d_engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "internal/body_math.h"
#include "internal/constraints.h"
#include "internal/contacts.h"
#include "internal/solver.h"
#include "internal/validation.h"
#include "internal/warm_contacts.h"

// Public entry points: parameter validation, semi-implicit Euler integration,
// and the step orchestration that drives the internal contact and solver
// units. Every failure path completes before any body is modified.

namespace tiny2d {

using namespace internal;  // NOLINT: engine-internal implementation units.

namespace {

constexpr float kLegacyRestitutionVelocityThreshold = 20.0f;

}  // namespace

float GetMomentOfInertia(const Rectangle& rectangle) {
  ValidateRectangle(rectangle);
  if (rectangle.mass == 0.0f) {
    return 0.0f;
  }
  const double moment_of_inertia = MomentOfInertiaUnchecked(rectangle);
  RequireFloatResult(moment_of_inertia,
                     "Rectangle moment of inertia exceeds the float range.");
  return static_cast<float>(moment_of_inertia);
}

float GetMomentOfInertia(const Circle& circle) {
  ValidateCircle(circle);
  if (circle.mass == 0.0f) {
    return 0.0f;
  }
  const double moment_of_inertia = MomentOfInertiaUnchecked(circle);
  RequireFloatResult(moment_of_inertia,
                     "Circle moment of inertia exceeds the float range.");
  return static_cast<float>(moment_of_inertia);
}

void AddForceAtPoint(Rectangle& rectangle, Vec2 force, Vec2 world_point) {
  ValidateRectangle(rectangle);
  Require(IsFinite(force) && IsFinite(world_point),
          "Force and world point must contain only finite values.");
  if (rectangle.mass == 0.0f) {
    return;
  }

  const double applied_force_x =
      static_cast<double>(rectangle.applied_force.x) + force.x;
  const double applied_force_y =
      static_cast<double>(rectangle.applied_force.y) + force.y;
  double applied_torque = rectangle.applied_torque;
  if (!rectangle.fixed_rotation) {
    const double radius_x =
        static_cast<double>(world_point.x) - rectangle.position.x;
    const double radius_y =
        static_cast<double>(world_point.y) - rectangle.position.y;
    applied_torque += radius_x * force.y - radius_y * force.x;
  }

  RequireFloatResult(applied_force_x,
                     "Accumulated horizontal force exceeds float range.");
  RequireFloatResult(applied_force_y,
                     "Accumulated vertical force exceeds float range.");
  RequireFloatResult(applied_torque, "Accumulated torque exceeds float range.");
  rectangle.applied_force = {static_cast<float>(applied_force_x),
                             static_cast<float>(applied_force_y)};
  rectangle.applied_torque = static_cast<float>(applied_torque);
}

void AddForceAtPoint(Circle& circle, Vec2 force, Vec2 world_point) {
  ValidateCircle(circle);
  Require(IsFinite(force) && IsFinite(world_point),
          "Force and world point must contain only finite values.");
  if (circle.mass == 0.0f) {
    return;
  }

  const double applied_force_x =
      static_cast<double>(circle.applied_force.x) + force.x;
  const double applied_force_y =
      static_cast<double>(circle.applied_force.y) + force.y;
  double applied_torque = circle.applied_torque;
  if (!circle.fixed_rotation) {
    const double radius_x =
        static_cast<double>(world_point.x) - circle.position.x;
    const double radius_y =
        static_cast<double>(world_point.y) - circle.position.y;
    applied_torque += radius_x * force.y - radius_y * force.x;
  }

  RequireFloatResult(applied_force_x,
                     "Accumulated horizontal force exceeds float range.");
  RequireFloatResult(applied_force_y,
                     "Accumulated vertical force exceeds float range.");
  RequireFloatResult(applied_torque, "Accumulated torque exceeds float range.");
  circle.applied_force = {static_cast<float>(applied_force_x),
                          static_cast<float>(applied_force_y)};
  circle.applied_torque = static_cast<float>(applied_torque);
}

void AddTorque(Rectangle& rectangle, float torque) {
  ValidateRectangle(rectangle);
  Require(std::isfinite(torque), "Torque must be finite.");
  if (rectangle.mass == 0.0f || rectangle.fixed_rotation) {
    return;
  }

  const double applied_torque =
      static_cast<double>(rectangle.applied_torque) + torque;
  RequireFloatResult(applied_torque, "Accumulated torque exceeds float range.");
  rectangle.applied_torque = static_cast<float>(applied_torque);
}

void AddTorque(Circle& circle, float torque) {
  ValidateCircle(circle);
  Require(std::isfinite(torque), "Torque must be finite.");
  if (circle.mass == 0.0f || circle.fixed_rotation) {
    return;
  }

  const double applied_torque =
      static_cast<double>(circle.applied_torque) + torque;
  RequireFloatResult(applied_torque, "Accumulated torque exceeds float range.");
  circle.applied_torque = static_cast<float>(applied_torque);
}

Vec2 GetLinearAcceleration(const Rectangle& rectangle, Vec2 electric_field,
                           float gravity) {
  ValidateRectangle(rectangle);
  Require(IsFinite(electric_field) && std::isfinite(gravity),
          "Electric field and gravity must contain only finite values.");
  if (rectangle.mass == 0.0f) {
    return {};
  }

  const double acceleration_x =
      static_cast<double>(electric_field.x) * rectangle.charge / rectangle.mass;
  const double acceleration_y =
      static_cast<double>(gravity) +
      static_cast<double>(electric_field.y) * rectangle.charge / rectangle.mass;
  RequireFloatResult(acceleration_x,
                     "Electric acceleration exceeds the float range.");
  RequireFloatResult(acceleration_y,
                     "Combined acceleration exceeds the float range.");
  return {static_cast<float>(acceleration_x),
          static_cast<float>(acceleration_y)};
}

Vec2 GetLinearAcceleration(const Circle& circle, Vec2 electric_field,
                           float gravity) {
  ValidateCircle(circle);
  Require(IsFinite(electric_field) && std::isfinite(gravity),
          "Electric field and gravity must contain only finite values.");
  if (circle.mass == 0.0f) {
    return {};
  }

  const double acceleration_x =
      static_cast<double>(electric_field.x) * circle.charge / circle.mass;
  const double acceleration_y =
      static_cast<double>(gravity) +
      static_cast<double>(electric_field.y) * circle.charge / circle.mass;
  RequireFloatResult(acceleration_x,
                     "Electric acceleration exceeds the float range.");
  RequireFloatResult(acceleration_y,
                     "Combined acceleration exceeds the float range.");
  return {static_cast<float>(acceleration_x),
          static_cast<float>(acceleration_y)};
}

std::array<Vec2, 4> GetVertices(const Rectangle& rectangle) {
  ValidateGeometry(rectangle);
  return GetVerticesUnchecked(rectangle);
}

bool IsColliding(const Rectangle& rectangle_a, const Rectangle& rectangle_b) {
  ValidateGeometry(rectangle_a);
  ValidateGeometry(rectangle_b);
  return FindContact(rectangle_a, rectangle_b).has_value();
}

bool IsColliding(const Circle& circle_a, const Circle& circle_b) {
  ValidateGeometry(circle_a);
  ValidateGeometry(circle_b);
  return FindContact(circle_a, circle_b).has_value();
}

bool IsColliding(const Rectangle& rectangle, const Circle& circle) {
  ValidateGeometry(rectangle);
  ValidateGeometry(circle);
  return FindContact(rectangle, circle).has_value();
}

bool IsColliding(const Circle& circle, const Rectangle& rectangle) {
  return IsColliding(rectangle, circle);
}

// The rectangle-only overload delegates to the mixed-world step with no
// circles, CCD disabled, and the legacy restitution velocity threshold, so
// there is exactly one integration and solver path. Equivalence is protected
// by TestLegacyUpdateMatchesMixedUpdateTrajectories.
void Update(std::vector<Rectangle>& squares, float delta_time, float area_width,
            float area_height, float restitution, float friction,
            Vec2 electric_field, float gravity) {
  std::vector<Circle> no_circles;
  Update(squares, no_circles, delta_time, area_width, area_height, restitution,
         friction, electric_field, gravity, kLegacyRestitutionVelocityThreshold,
         false);
}

// The mixed overload forwards to the constrained step with empty constraint
// sets, so all three public Update entry points share one integration and
// solver path. Equivalence is protected by the golden checkpoint test and
// TestMixedUpdateMatchesConstrainedUpdateWithoutConstraints.
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            float delta_time, float area_width, float area_height,
            float restitution, float friction, Vec2 electric_field,
            float gravity, float restitution_velocity_threshold,
            bool enable_circle_circle_ccd) {
  Update(rectangles, circles, {}, {}, delta_time, area_width, area_height,
         restitution, friction, electric_field, gravity,
         restitution_velocity_threshold, enable_circle_circle_ccd, nullptr);
}

// The (pins, ropes) overload forwards to the full ConstraintSet step with
// empty rod vectors, so every constrained caller shares one path.
// Equivalence is protected by
// TestConstraintSetOverloadMatchesPinsRopesOverload.
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            const std::vector<RevolutePin>& revolute_pins,
            const std::vector<PulleyRope>& pulley_ropes, float delta_time,
            float area_width, float area_height, float restitution,
            float friction, Vec2 electric_field, float gravity,
            float restitution_velocity_threshold, bool enable_circle_circle_ccd,
            ConstraintReactions* reactions) {
  ConstraintSet constraints;
  constraints.revolute_pins = revolute_pins;
  constraints.pulley_ropes = pulley_ropes;
  Update(rectangles, circles, constraints, delta_time, area_width, area_height,
         restitution, friction, electric_field, gravity,
         restitution_velocity_threshold, enable_circle_circle_ccd, reactions);
}

// The ConstraintSet overload forwards to the full-control step with
// default solver settings and no cache, so every constrained caller
// shares one path. Equivalence is protected by
// TestFullControlOverloadMatchesConstraintSetOverload.
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            const ConstraintSet& constraints, float delta_time,
            float area_width, float area_height, float restitution,
            float friction, Vec2 electric_field, float gravity,
            float restitution_velocity_threshold, bool enable_circle_circle_ccd,
            ConstraintReactions* reactions) {
  Update(rectangles, circles, constraints, SolverSettings{}, nullptr,
         delta_time, area_width, area_height, restitution, friction,
         electric_field, gravity, restitution_velocity_threshold,
         enable_circle_circle_ccd, reactions);
}

void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            const ConstraintSet& constraints,
            const SolverSettings& solver_settings, ContactCache* contact_cache,
            float delta_time, float area_width, float area_height,
            float restitution, float friction, Vec2 electric_field,
            float gravity, float restitution_velocity_threshold,
            bool enable_circle_circle_ccd, ConstraintReactions* reactions) {
  Require(std::isfinite(delta_time) && delta_time >= 0.0f,
          "Delta time must be finite and non-negative.");
  Require(std::isfinite(area_width) && std::isfinite(area_height) &&
              area_width > 0.0f && area_height > 0.0f,
          "Simulation area dimensions must be finite and positive.");
  Require(
      std::isfinite(restitution) && restitution >= 0.0f && restitution <= 1.0f,
      "Restitution must be in the range [0, 1].");
  Require(std::isfinite(friction) && friction >= 0.0f,
          "Friction must be finite and non-negative.");
  Require(IsFinite(electric_field) && std::isfinite(gravity),
          "Electric field and gravity must contain only finite values.");
  Require(std::isfinite(restitution_velocity_threshold) &&
              restitution_velocity_threshold >= 0.0f,
          "Restitution velocity threshold must be finite and non-negative.");
  const bool has_constraints =
      !constraints.revolute_pins.empty() || !constraints.pulley_ropes.empty() ||
      !constraints.anchor_rods.empty() || !constraints.link_rods.empty();
  Require(!enable_circle_circle_ccd || !has_constraints,
          "Circle-circle CCD cannot be combined with constraints.");
  Require(!enable_circle_circle_ccd || contact_cache == nullptr,
          "Circle-circle CCD cannot be combined with a contact cache.");
  ValidateSolverSettings(solver_settings);
  if (contact_cache != nullptr) {
    ValidateContactCache(*contact_cache);
  }
  ValidateConstraints(rectangles, circles, constraints, area_width,
                      area_height);

  const auto validate_integration = [&](const auto& body) {
    if (body.mass == 0.0f) {
      return;
    }
    Require(FitsInArea(body, area_width, area_height),
            "A dynamic body cannot fit inside the simulation area.");

    const double field_acceleration_x =
        static_cast<double>(electric_field.x) * body.charge / body.mass;
    const double field_acceleration_y =
        static_cast<double>(gravity) +
        static_cast<double>(electric_field.y) * body.charge / body.mass;
    RequireFloatResult(field_acceleration_x,
                       "Electric acceleration exceeds the float range.");
    RequireFloatResult(field_acceleration_y,
                       "Combined acceleration exceeds the float range.");
    if (delta_time == 0.0f) {
      return;
    }

    const double force_acceleration_x =
        static_cast<double>(body.applied_force.x) / body.mass;
    const double force_acceleration_y =
        static_cast<double>(body.applied_force.y) / body.mass;
    const double acceleration_x = field_acceleration_x + force_acceleration_x;
    const double acceleration_y = field_acceleration_y + force_acceleration_y;
    const double velocity_x = body.velocity.x + acceleration_x * delta_time;
    const double velocity_y = body.velocity.y + acceleration_y * delta_time;
    RequireFloatResult(force_acceleration_x,
                       "Applied horizontal acceleration exceeds float range.");
    RequireFloatResult(force_acceleration_y,
                       "Applied vertical acceleration exceeds float range.");
    RequireFloatResult(acceleration_x,
                       "Total horizontal acceleration exceeds float range.");
    RequireFloatResult(acceleration_y,
                       "Total vertical acceleration exceeds float range.");
    RequireFloatResult(velocity_x,
                       "Integrated horizontal velocity exceeds float range.");
    RequireFloatResult(velocity_y,
                       "Integrated vertical velocity exceeds float range.");
    RequireFloatResult(body.position.x + velocity_x * delta_time,
                       "Integrated horizontal position exceeds float range.");
    RequireFloatResult(body.position.y + velocity_y * delta_time,
                       "Integrated vertical position exceeds float range.");
    if (!body.fixed_rotation) {
      const double angular_acceleration =
          static_cast<double>(body.applied_torque) /
          MomentOfInertiaUnchecked(body);
      const double angular_velocity =
          body.angular_velocity + angular_acceleration * delta_time;
      RequireFloatResult(
          angular_acceleration,
          "Applied angular acceleration exceeds the float range.");
      RequireFloatResult(angular_velocity,
                         "Integrated angular velocity exceeds float range.");
      RequireFloatResult(body.angle + angular_velocity * delta_time,
                         "Integrated angle exceeds float range.");
    }
  };

  for (const Rectangle& rectangle : rectangles) {
    ValidateRectangle(rectangle);
    validate_integration(rectangle);
  }
  for (const Circle& circle : circles) {
    ValidateCircle(circle);
    validate_integration(circle);
  }

  // The fused integrate is split into a velocity phase and a position phase
  // so the constraint velocity solve can run between them. Per body the
  // operation sequence of velocity-then-position is exactly the historical
  // fused sequence, and bodies integrate independently, so the split is
  // trajectory-identical for the no-constraint path; the golden checkpoint
  // test locks this bit for bit. The velocity phase keeps the dt == 0
  // fixed-rotation zeroing and the position phase keeps the dt == 0 angle
  // renormalization the fused code performed.
  const auto integrate_velocity = [&](auto& body) {
    if (InverseMass(body) == 0.0f) {
      return;
    }
    if (delta_time > 0.0f) {
      if (body.applied_force.x == 0.0f && body.applied_force.y == 0.0f) {
        body.velocity =
            Add(body.velocity, Multiply(GetLinearAccelerationUnchecked(
                                            body, electric_field, gravity),
                                        delta_time));
      } else {
        const double acceleration_x =
            static_cast<double>(electric_field.x) * body.charge / body.mass +
            static_cast<double>(body.applied_force.x) / body.mass;
        const double acceleration_y =
            static_cast<double>(gravity) +
            static_cast<double>(electric_field.y) * body.charge / body.mass +
            static_cast<double>(body.applied_force.y) / body.mass;
        body.velocity = {
            static_cast<float>(body.velocity.x + acceleration_x * delta_time),
            static_cast<float>(body.velocity.y + acceleration_y * delta_time)};
      }
      if (body.linear_damping_rate > 0.0f) {
        body.velocity = Multiply(
            body.velocity, std::exp(-body.linear_damping_rate * delta_time));
      }
    }
    if (body.fixed_rotation) {
      body.angular_velocity = 0.0f;
    } else if (delta_time > 0.0f) {
      if (body.applied_torque != 0.0f) {
        body.angular_velocity =
            static_cast<float>(body.angular_velocity +
                               static_cast<double>(body.applied_torque) /
                                   MomentOfInertiaUnchecked(body) * delta_time);
      }
      body.angular_velocity *=
          std::exp(-body.angular_damping_rate * delta_time);
    }
  };
  const auto integrate_position = [&](auto& body) {
    if (InverseMass(body) == 0.0f) {
      return;
    }
    body.position = Add(body.position, Multiply(body.velocity, delta_time));
    if (!body.fixed_rotation) {
      body.angle = std::remainder(
          body.angle + body.angular_velocity * delta_time, kTwoPi);
    }
  };
  const auto integrate = [&](auto& body) {
    integrate_velocity(body);
    integrate_position(body);
  };

  bool use_circle_circle_ccd = false;
  if (enable_circle_circle_ccd && delta_time > 0.0f && circles.size() > 1) {
    std::vector<Rectangle> integrated_rectangles = rectangles;
    std::vector<Circle> integrated_circles = circles;
    for (Rectangle& rectangle : integrated_rectangles) {
      integrate(rectangle);
    }
    for (Circle& circle : integrated_circles) {
      integrate(circle);
    }
    use_circle_circle_ccd =
        HasMissedCircleImpact(circles, integrated_circles, delta_time);

    if (use_circle_circle_ccd) {
      for (std::size_t i = 0; i < rectangles.size(); ++i) {
        rectangles[i].velocity = integrated_rectangles[i].velocity;
        rectangles[i].angular_velocity =
            integrated_rectangles[i].angular_velocity;
      }
      for (std::size_t i = 0; i < circles.size(); ++i) {
        circles[i].velocity = integrated_circles[i].velocity;
        circles[i].angular_velocity = integrated_circles[i].angular_velocity;
      }

      const auto advance = [](auto& body, float time) {
        if (InverseMass(body) == 0.0f) {
          return;
        }
        body.position = Add(body.position, Multiply(body.velocity, time));
        if (!body.fixed_rotation) {
          body.angle =
              std::remainder(body.angle + body.angular_velocity * time, kTwoPi);
        }
      };

      float remaining_time = delta_time;
      // NOTE: O(n^2) scans, a 4N event cap, and the speculative body copies
      // above fit the supported small ImpactLab; add an event queue and
      // caller-provided scratch buffers only when a dense experiment
      // measurably needs them.
      const std::size_t maximum_impacts = circles.size() * kSolverIterations;
      for (std::size_t impact_count = 0;
           impact_count < maximum_impacts && remaining_time > 0.0f;
           ++impact_count) {
        const std::optional<CircleImpact> impact =
            FindEarliestCircleImpact(circles, remaining_time);
        if (!impact.has_value()) {
          break;
        }

        const float impact_time =
            std::clamp(static_cast<float>(impact->time), 0.0f, remaining_time);
        for (Rectangle& rectangle : rectangles) {
          advance(rectangle, impact_time);
        }
        for (Circle& circle : circles) {
          advance(circle, impact_time);
        }
        remaining_time -= impact_time;

        Circle& circle_a = circles[impact->first];
        Circle& circle_b = circles[impact->second];
        ResolveContact(circle_a, circle_b,
                       MakeCircleImpactContact(circle_a, circle_b),
                       MixMaterials(circle_a.material, circle_b.material,
                                    restitution, friction),
                       true, restitution_velocity_threshold);
      }

      for (Rectangle& rectangle : rectangles) {
        advance(rectangle, remaining_time);
      }
      for (Circle& circle : circles) {
        advance(circle, remaining_time);
      }
    }
  }

  ConstraintImpulses constraint_impulses;
  if (has_constraints) {
    constraint_impulses.pin_impulses.resize(constraints.revolute_pins.size());
    constraint_impulses.rope_impulses.resize(constraints.pulley_ropes.size());
    constraint_impulses.anchor_rod_impulses.resize(
        constraints.anchor_rods.size());
    constraint_impulses.link_rod_impulses.resize(constraints.link_rods.size());
  }

  if (!use_circle_circle_ccd) {
    for (Rectangle& rectangle : rectangles) {
      integrate_velocity(rectangle);
    }
    for (Circle& circle : circles) {
      integrate_velocity(circle);
    }
    if (has_constraints) {
      SolveConstraintVelocities(rectangles, circles, constraints,
                                &constraint_impulses);
    }
    for (Rectangle& rectangle : rectangles) {
      integrate_position(rectangle);
    }
    for (Circle& circle : circles) {
      integrate_position(circle);
    }
  }

  if (contact_cache != nullptr && delta_time > 0.0f) {
    // Warm-starting accumulated-impulse contact stage: detection once per
    // step, cached impulses applied before the first iteration, wall
    // velocity resolves anchoring every round, positional passes and the
    // wall snap at the end, and impulse write-back with stale eviction.
    // A zero delta_time skips this branch so the cache stays untouched.
    SolveWarmContacts(rectangles, circles, *contact_cache, solver_settings,
                      area_width, area_height, restitution, friction,
                      restitution_velocity_threshold);
  } else {
    for (int iteration = 0; iteration < solver_settings.iterations;
         ++iteration) {
      const bool allow_restitution = iteration == 0;
      for (std::size_t i = 0; i < rectangles.size(); ++i) {
        for (std::size_t j = i + 1; j < rectangles.size(); ++j) {
          const std::optional<ContactManifold> contact =
              FindContact(rectangles[i], rectangles[j]);
          if (contact.has_value()) {
            ResolveContact(
                rectangles[i], rectangles[j], *contact,
                MixMaterials(rectangles[i].material, rectangles[j].material,
                             restitution, friction),
                allow_restitution, restitution_velocity_threshold,
                solver_settings.position_slop,
                solver_settings.position_correction);
          }
        }
      }
      for (std::size_t i = 0; i < circles.size(); ++i) {
        for (std::size_t j = i + 1; j < circles.size(); ++j) {
          const std::optional<ContactManifold> contact =
              FindContact(circles[i], circles[j]);
          if (contact.has_value()) {
            ResolveContact(
                circles[i], circles[j], *contact,
                MixMaterials(circles[i].material, circles[j].material,
                             restitution, friction),
                allow_restitution, restitution_velocity_threshold,
                solver_settings.position_slop,
                solver_settings.position_correction);
          }
        }
      }
      for (Rectangle& rectangle : rectangles) {
        for (Circle& circle : circles) {
          const std::optional<ContactManifold> contact =
              FindContact(rectangle, circle);
          if (contact.has_value()) {
            ResolveContact(rectangle, circle, *contact,
                           MixMaterials(rectangle.material, circle.material,
                                        restitution, friction),
                           allow_restitution, restitution_velocity_threshold,
                           solver_settings.position_slop,
                           solver_settings.position_correction);
          }
        }
      }

      for (Rectangle& rectangle : rectangles) {
        ResolveWindowCollision(rectangle, area_width, area_height, restitution,
                               friction, allow_restitution,
                               restitution_velocity_threshold);
      }
      for (Circle& circle : circles) {
        ResolveWindowCollision(circle, area_width, area_height, restitution,
                               friction, allow_restitution,
                               restitution_velocity_threshold);
      }
    }
  }

  if (has_constraints) {
    ProjectConstraintPositions(rectangles, circles, constraints);
  }
  if (reactions != nullptr) {
    reactions->pin_forces.assign(constraints.revolute_pins.size(), Vec2{});
    reactions->rope_tensions.assign(constraints.pulley_ropes.size(),
                                    RopeReaction{});
    reactions->anchor_rod_forces.assign(constraints.anchor_rods.size(), 0.0f);
    reactions->link_rod_forces.assign(constraints.link_rods.size(), 0.0f);
    if (delta_time > 0.0f) {
      const float inverse_delta_time = 1.0f / delta_time;
      for (std::size_t i = 0; i < constraints.revolute_pins.size(); ++i) {
        reactions->pin_forces[i] =
            Multiply(constraint_impulses.pin_impulses[i], inverse_delta_time);
      }
      for (std::size_t i = 0; i < constraints.pulley_ropes.size(); ++i) {
        reactions->rope_tensions[i] = {
            -constraint_impulses.rope_impulses[i].impulse_a *
                inverse_delta_time,
            -constraint_impulses.rope_impulses[i].impulse_b *
                inverse_delta_time};
      }
      for (std::size_t i = 0; i < constraints.anchor_rods.size(); ++i) {
        reactions->anchor_rod_forces[i] =
            -constraint_impulses.anchor_rod_impulses[i] * inverse_delta_time;
      }
      for (std::size_t i = 0; i < constraints.link_rods.size(); ++i) {
        reactions->link_rod_forces[i] =
            -constraint_impulses.link_rod_impulses[i] * inverse_delta_time;
      }
    }
  }

  for (Rectangle& rectangle : rectangles) {
    rectangle.applied_force = {};
    rectangle.applied_torque = 0.0f;
  }
  for (Circle& circle : circles) {
    circle.applied_force = {};
    circle.applied_torque = 0.0f;
  }
}

}  // namespace tiny2d
