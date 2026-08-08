#ifndef TINY2DENGINE_ENGINE_INTERNAL_BODY_MATH_H_
#define TINY2DENGINE_ENGINE_INTERNAL_BODY_MATH_H_

#include <array>
#include <cmath>
#include <cstddef>

#include "tiny2d_engine.h"

// Engine-internal vector algebra and per-body mass properties. Everything in
// tiny2d::internal is an implementation detail with no stability guarantee.

namespace tiny2d::internal {

constexpr float kTwoPi = 6.28318530718f;

inline bool IsFinite(Vec2 vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y);
}

inline Vec2 Add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }

inline Vec2 Subtract(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }

inline Vec2 Multiply(Vec2 vector, float scalar) {
  return {vector.x * scalar, vector.y * scalar};
}

inline float Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

inline float Cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

inline Vec2 Cross(float scalar, Vec2 vector) {
  return {-scalar * vector.y, scalar * vector.x};
}

inline float LengthSquared(Vec2 vector) { return Dot(vector, vector); }

inline Vec2 Normalize(Vec2 vector) {
  const float length_squared = LengthSquared(vector);
  if (length_squared == 0.0f) {
    return {};
  }
  return Multiply(vector, 1.0f / std::sqrt(length_squared));
}

inline Vec2 RotateToWorld(Vec2 local, float angle) {
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {local.x * cosine - local.y * sine, local.x * sine + local.y * cosine};
}

inline Vec2 RotateToLocal(Vec2 world, float angle) {
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {world.x * cosine + world.y * sine,
          -world.x * sine + world.y * cosine};
}

inline float GeometricMean(float a, float b) {
  return static_cast<float>(
      std::sqrt(static_cast<double>(a) * static_cast<double>(b)));
}

template <typename Body>
float InverseMass(const Body& body) {
  return body.mass > 0.0f ? 1.0f / body.mass : 0.0f;
}

inline bool IsKnownInertiaModel(CircleInertiaModel model) {
  switch (model) {
    case CircleInertiaModel::kSolidDisk:
    case CircleInertiaModel::kHoop:
      return true;
  }
  return false;
}

inline double MomentOfInertiaUnchecked(const Rectangle& rectangle) {
  return static_cast<double>(rectangle.mass) *
         (static_cast<double>(rectangle.width) * rectangle.width +
          static_cast<double>(rectangle.height) * rectangle.height) /
         12.0;
}

inline double MomentOfInertiaUnchecked(const Circle& circle) {
  const double factor =
      circle.inertia_model == CircleInertiaModel::kSolidDisk ? 0.5 : 1.0;
  return factor * static_cast<double>(circle.mass) * circle.radius *
         circle.radius;
}

inline float InverseInertia(const Rectangle& square) {
  if (square.fixed_rotation || square.mass <= 0.0f || square.width <= 0.0f ||
      square.height <= 0.0f) {
    return 0.0f;
  }
  return 12.0f / (square.mass * (square.width * square.width +
                                 square.height * square.height));
}

inline float InverseInertia(const Circle& circle) {
  if (circle.fixed_rotation || circle.mass <= 0.0f || circle.radius <= 0.0f ||
      !IsKnownInertiaModel(circle.inertia_model)) {
    return 0.0f;
  }
  return static_cast<float>(1.0 / MomentOfInertiaUnchecked(circle));
}

template <typename Body>
Vec2 GetLinearAccelerationUnchecked(const Body& body, Vec2 electric_field,
                                    float gravity) {
  if (body.mass <= 0.0f) {
    return {};
  }
  const float charge_over_mass = body.charge / body.mass;
  return {electric_field.x * charge_over_mass,
          gravity + electric_field.y * charge_over_mass};
}

inline std::array<Vec2, 4> GetVerticesUnchecked(const Rectangle& rectangle) {
  const float half_width = rectangle.width * 0.5f;
  const float half_height = rectangle.height * 0.5f;
  const float cosine = std::cos(rectangle.angle);
  const float sine = std::sin(rectangle.angle);
  const std::array<Vec2, 4> local_vertices = {
      Vec2{-half_width, -half_height}, Vec2{half_width, -half_height},
      Vec2{half_width, half_height}, Vec2{-half_width, half_height}};

  std::array<Vec2, 4> vertices{};
  for (std::size_t i = 0; i < local_vertices.size(); ++i) {
    const Vec2 local = local_vertices[i];
    vertices[i] = {rectangle.position.x + local.x * cosine - local.y * sine,
                   rectangle.position.y + local.x * sine + local.y * cosine};
  }
  return vertices;
}

inline std::array<Vec2, 2> GetAxes(const Rectangle& square) {
  const float cosine = std::cos(square.angle);
  const float sine = std::sin(square.angle);
  return {{{cosine, sine}, {-sine, cosine}}};
}

}  // namespace tiny2d::internal

#endif  // TINY2DENGINE_ENGINE_INTERNAL_BODY_MATH_H_
