#include "tiny2d_engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "internal/body_math.h"
#include "internal/contacts.h"
#include "internal/solver.h"
#include "internal/validation.h"

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

void Update(std::vector<Rectangle>& squares, float delta_time, float area_width,
            float area_height, float restitution, float friction,
            Vec2 electric_field, float gravity) {
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

  for (const Rectangle& square : squares) {
    ValidateRectangle(square);
    if (square.mass == 0.0f) {
      continue;
    }
    Require(FitsInArea(square, area_width, area_height),
            "A dynamic rectangle cannot fit inside the simulation area.");

    const double field_acceleration_x =
        static_cast<double>(electric_field.x) * square.charge / square.mass;
    const double field_acceleration_y =
        static_cast<double>(gravity) +
        static_cast<double>(electric_field.y) * square.charge / square.mass;
    RequireFloatResult(field_acceleration_x,
                       "Electric acceleration exceeds the float range.");
    RequireFloatResult(field_acceleration_y,
                       "Combined acceleration exceeds the float range.");
    if (delta_time == 0.0f) {
      continue;
    }

    const double force_acceleration_x =
        static_cast<double>(square.applied_force.x) / square.mass;
    const double force_acceleration_y =
        static_cast<double>(square.applied_force.y) / square.mass;
    const double acceleration_x = field_acceleration_x + force_acceleration_x;
    const double acceleration_y = field_acceleration_y + force_acceleration_y;
    const double velocity_x = square.velocity.x + acceleration_x * delta_time;
    const double velocity_y = square.velocity.y + acceleration_y * delta_time;
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
    RequireFloatResult(square.position.x + velocity_x * delta_time,
                       "Integrated horizontal position exceeds float range.");
    RequireFloatResult(square.position.y + velocity_y * delta_time,
                       "Integrated vertical position exceeds float range.");
    if (!square.fixed_rotation) {
      const double angular_acceleration =
          static_cast<double>(square.applied_torque) /
          MomentOfInertiaUnchecked(square);
      const double angular_velocity =
          square.angular_velocity + angular_acceleration * delta_time;
      RequireFloatResult(
          angular_acceleration,
          "Applied angular acceleration exceeds the float range.");
      RequireFloatResult(angular_velocity,
                         "Integrated angular velocity exceeds float range.");
      RequireFloatResult(square.angle + angular_velocity * delta_time,
                         "Integrated angle exceeds float range.");
    }
  }

  for (Rectangle& square : squares) {
    if (InverseMass(square) == 0.0f) {
      continue;
    }
    if (delta_time > 0.0f) {
      if (square.applied_force.x == 0.0f && square.applied_force.y == 0.0f) {
        square.velocity =
            Add(square.velocity, Multiply(GetLinearAccelerationUnchecked(
                                              square, electric_field, gravity),
                                          delta_time));
      } else {
        const double acceleration_x =
            static_cast<double>(electric_field.x) * square.charge /
                square.mass +
            static_cast<double>(square.applied_force.x) / square.mass;
        const double acceleration_y =
            static_cast<double>(gravity) +
            static_cast<double>(electric_field.y) * square.charge /
                square.mass +
            static_cast<double>(square.applied_force.y) / square.mass;
        square.velocity = {
            static_cast<float>(square.velocity.x + acceleration_x * delta_time),
            static_cast<float>(square.velocity.y +
                               acceleration_y * delta_time)};
      }
      if (square.linear_damping_rate > 0.0f) {
        square.velocity =
            Multiply(square.velocity,
                     std::exp(-square.linear_damping_rate * delta_time));
      }
    }
    if (square.fixed_rotation) {
      square.angular_velocity = 0.0f;
    } else if (delta_time > 0.0f) {
      if (square.applied_torque != 0.0f) {
        square.angular_velocity = static_cast<float>(
            square.angular_velocity +
            static_cast<double>(square.applied_torque) /
                MomentOfInertiaUnchecked(square) * delta_time);
      }
      square.angular_velocity *=
          std::exp(-square.angular_damping_rate * delta_time);
    }
    square.position =
        Add(square.position, Multiply(square.velocity, delta_time));
    if (!square.fixed_rotation) {
      square.angle = std::remainder(
          square.angle + square.angular_velocity * delta_time, kTwoPi);
    }
  }

  for (int iteration = 0; iteration < kSolverIterations; ++iteration) {
    const bool allow_restitution = iteration == 0;
    for (std::size_t i = 0; i < squares.size(); ++i) {
      for (std::size_t j = i + 1; j < squares.size(); ++j) {
        const std::optional<ContactManifold> contact =
            FindContact(squares[i], squares[j]);
        if (contact.has_value()) {
          ResolveContact(squares[i], squares[j], *contact,
                         MixMaterials(squares[i].material, squares[j].material,
                                      restitution, friction),
                         allow_restitution,
                         kLegacyRestitutionVelocityThreshold);
        }
      }
    }

    for (Rectangle& square : squares) {
      ResolveWindowCollision(square, area_width, area_height, restitution,
                             friction, allow_restitution,
                             kLegacyRestitutionVelocityThreshold);
    }
  }

  for (Rectangle& square : squares) {
    square.applied_force = {};
    square.applied_torque = 0.0f;
  }
}

void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            float delta_time, float area_width, float area_height,
            float restitution, float friction, Vec2 electric_field,
            float gravity, float restitution_velocity_threshold,
            bool enable_circle_circle_ccd) {
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

  const auto integrate = [&](auto& body) {
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
    body.position = Add(body.position, Multiply(body.velocity, delta_time));
    if (!body.fixed_rotation) {
      body.angle = std::remainder(
          body.angle + body.angular_velocity * delta_time, kTwoPi);
    }
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
      // NOTE: O(n^2) scans and a 4N event cap fit the supported small
      // ImpactLab; add an event queue only when a dense experiment needs it.
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

  if (!use_circle_circle_ccd) {
    for (Rectangle& rectangle : rectangles) {
      integrate(rectangle);
    }
    for (Circle& circle : circles) {
      integrate(circle);
    }
  }

  for (int iteration = 0; iteration < kSolverIterations; ++iteration) {
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
              allow_restitution, restitution_velocity_threshold);
        }
      }
    }
    for (std::size_t i = 0; i < circles.size(); ++i) {
      for (std::size_t j = i + 1; j < circles.size(); ++j) {
        const std::optional<ContactManifold> contact =
            FindContact(circles[i], circles[j]);
        if (contact.has_value()) {
          ResolveContact(circles[i], circles[j], *contact,
                         MixMaterials(circles[i].material, circles[j].material,
                                      restitution, friction),
                         allow_restitution, restitution_velocity_threshold);
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
                         allow_restitution, restitution_velocity_threshold);
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
