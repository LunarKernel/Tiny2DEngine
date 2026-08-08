#include "internal/validation.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "internal/body_math.h"

namespace tiny2d::internal {

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

void RequireFloatResult(double value, const char* message) {
  Require(std::isfinite(value) &&
              std::abs(value) <= std::numeric_limits<float>::max(),
          message);
}

bool UsesWorldMaterial(const CollisionMaterial& material) {
  return material.restitution == -1.0f && material.static_friction == -1.0f &&
         material.kinetic_friction == -1.0f;
}

void ValidateMaterial(const CollisionMaterial& material) {
  if (UsesWorldMaterial(material)) {
    return;
  }
  Require(
      std::isfinite(material.restitution) &&
          std::isfinite(material.static_friction) &&
          std::isfinite(material.kinetic_friction) &&
          material.restitution >= 0.0f && material.restitution <= 1.0f &&
          material.static_friction >= 0.0f &&
          material.kinetic_friction >= 0.0f &&
          material.kinetic_friction <= material.static_friction,
      "Collision material must either inherit all world values or contain "
      "restitution in [0, 1] and 0 <= kinetic friction <= static friction.");
}

void ValidateGeometry(const Rectangle& rectangle) {
  Require(IsFinite(rectangle.position) && std::isfinite(rectangle.angle) &&
              std::isfinite(rectangle.width) && std::isfinite(rectangle.height),
          "Rectangle geometry must contain only finite values.");
  Require(rectangle.width > 0.0f && rectangle.height > 0.0f,
          "Rectangle width and height must be positive.");

  constexpr double kSafeCoordinate =
      static_cast<double>(std::numeric_limits<float>::max()) / 16.0;
  Require(
      std::abs(static_cast<double>(rectangle.position.x)) <= kSafeCoordinate &&
          std::abs(static_cast<double>(rectangle.position.y)) <=
              kSafeCoordinate &&
          static_cast<double>(rectangle.width) <= kSafeCoordinate &&
          static_cast<double>(rectangle.height) <= kSafeCoordinate,
      "Rectangle geometry is too large for stable float calculations.");
}

void ValidateGeometry(const Circle& circle) {
  Require(IsFinite(circle.position) && std::isfinite(circle.radius),
          "Circle geometry must contain only finite values.");
  Require(circle.radius > 0.0f, "Circle radius must be positive.");

  constexpr double kSafeCoordinate =
      static_cast<double>(std::numeric_limits<float>::max()) / 16.0;
  Require(
      std::abs(static_cast<double>(circle.position.x)) <= kSafeCoordinate &&
          std::abs(static_cast<double>(circle.position.y)) <= kSafeCoordinate &&
          static_cast<double>(circle.radius) <= kSafeCoordinate,
      "Circle geometry is too large for stable float calculations.");
}

void ValidateRectangle(const Rectangle& rectangle) {
  ValidateGeometry(rectangle);
  ValidateMaterial(rectangle.material);
  Require(std::isfinite(rectangle.mass) && IsFinite(rectangle.velocity) &&
              std::isfinite(rectangle.angular_velocity) &&
              std::isfinite(rectangle.charge) &&
              IsFinite(rectangle.applied_force) &&
              std::isfinite(rectangle.applied_torque) &&
              std::isfinite(rectangle.linear_damping_rate) &&
              std::isfinite(rectangle.angular_damping_rate),
          "Rectangle state must contain only finite values.");
  Require(rectangle.mass == 0.0f || rectangle.mass >= kMinimumDynamicMass,
          "Rectangle mass must be zero for a static body or at least 1e-6.");
  Require(rectangle.linear_damping_rate >= 0.0f &&
              rectangle.angular_damping_rate >= 0.0f,
          "Rectangle damping rates must be non-negative.");
  if (rectangle.mass == 0.0f) {
    Require(rectangle.velocity.x == 0.0f && rectangle.velocity.y == 0.0f &&
                rectangle.angular_velocity == 0.0f,
            "Static rectangles cannot have velocity.");
  }

  if (rectangle.mass > 0.0f && !rectangle.fixed_rotation) {
    const double inertia_denominator =
        static_cast<double>(rectangle.mass) *
        (static_cast<double>(rectangle.width) * rectangle.width +
         static_cast<double>(rectangle.height) * rectangle.height);
    Require(inertia_denominator <= std::numeric_limits<float>::max(),
            "Rectangle inertia is too large for stable float calculations.");
  }
}

void ValidateCircle(const Circle& circle) {
  ValidateGeometry(circle);
  ValidateMaterial(circle.material);
  Require(std::isfinite(circle.mass) && IsFinite(circle.velocity) &&
              std::isfinite(circle.angle) &&
              std::isfinite(circle.angular_velocity) &&
              std::isfinite(circle.charge) && IsFinite(circle.applied_force) &&
              std::isfinite(circle.applied_torque) &&
              std::isfinite(circle.linear_damping_rate) &&
              std::isfinite(circle.angular_damping_rate),
          "Circle state must contain only finite values.");
  Require(IsKnownInertiaModel(circle.inertia_model),
          "Circle inertia model is invalid.");
  Require(circle.mass == 0.0f || circle.mass >= kMinimumDynamicMass,
          "Circle mass must be zero for a static body or at least 1e-6.");
  Require(
      circle.linear_damping_rate >= 0.0f && circle.angular_damping_rate >= 0.0f,
      "Circle damping rates must be non-negative.");
  if (circle.mass == 0.0f) {
    Require(circle.velocity.x == 0.0f && circle.velocity.y == 0.0f &&
                circle.angular_velocity == 0.0f,
            "Static circles cannot have velocity.");
  }

  if (circle.mass > 0.0f && !circle.fixed_rotation) {
    const double factor =
        circle.inertia_model == CircleInertiaModel::kSolidDisk ? 0.5 : 1.0;
    const double inertia = factor * static_cast<double>(circle.mass) *
                           circle.radius * circle.radius;
    Require(inertia <= std::numeric_limits<float>::max(),
            "Circle inertia is too large for stable float calculations.");
  }
}

bool FitsInArea(const Rectangle& rectangle, float area_width,
                float area_height) {
  const double cosine = std::abs(std::cos(rectangle.angle));
  const double sine = std::abs(std::sin(rectangle.angle));
  const double bounding_width =
      cosine * rectangle.width + sine * rectangle.height;
  const double bounding_height =
      sine * rectangle.width + cosine * rectangle.height;
  return bounding_width <= area_width && bounding_height <= area_height;
}

bool FitsInArea(const Circle& circle, float area_width, float area_height) {
  const double diameter = static_cast<double>(circle.radius) * 2.0;
  return diameter <= area_width && diameter <= area_height;
}

}  // namespace tiny2d::internal
