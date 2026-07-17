#include "tiny2d_engine.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace tiny2d {
namespace {

constexpr float kGravity = 98.1f;
constexpr float kTwoPi = 6.28318530718f;
constexpr float kSupportEpsilon = 0.0001f;
constexpr float kPositionSlop = 0.01f;
constexpr float kPositionCorrection = 0.8f;
constexpr float kRestitutionVelocityThreshold = 1.0f;
constexpr float kFrictionCoefficient = 0.35f;
constexpr float kAngularDamping = 0.8f;
constexpr int kSolverIterations = 4;

struct Projection {
  float minimum;
  float maximum;
};

struct Contact {
  Vec2 normal;
  Vec2 point;
  float penetration;
};

struct SupportFeature {
  std::array<Vec2, 2> points{};
  std::size_t count{};
};

Vec2 Add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }

Vec2 Subtract(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }

Vec2 Multiply(Vec2 vector, float scalar) {
  return {vector.x * scalar, vector.y * scalar};
}

float Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

float Cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

Vec2 Cross(float scalar, Vec2 vector) {
  return {-scalar * vector.y, scalar * vector.x};
}

float LengthSquared(Vec2 vector) { return Dot(vector, vector); }

Vec2 Normalize(Vec2 vector) {
  const float length_squared = LengthSquared(vector);
  if (length_squared == 0.0f) {
    return {};
  }
  return Multiply(vector, 1.0f / std::sqrt(length_squared));
}

float InverseMass(const Square& square) {
  return square.mass > 0.0f ? 1.0f / square.mass : 0.0f;
}

float InverseInertia(const Square& square) {
  if (square.mass <= 0.0f || square.size <= 0.0f) {
    return 0.0f;
  }
  return 6.0f / (square.mass * square.size * square.size);
}

Vec2 ContactVelocity(const Square& square, Vec2 radius) {
  return Add(square.velocity, Cross(square.angular_velocity, radius));
}

float ImpulseDenominator(const Square& square, Vec2 radius, Vec2 direction) {
  const float radius_cross_direction = Cross(radius, direction);
  return InverseMass(square) + radius_cross_direction * radius_cross_direction *
                                   InverseInertia(square);
}

void ApplyImpulse(Square& square, Vec2 impulse, Vec2 radius) {
  square.velocity =
      Add(square.velocity, Multiply(impulse, InverseMass(square)));
  square.angular_velocity += Cross(radius, impulse) * InverseInertia(square);
}

std::array<Vec2, 2> GetAxes(const Square& square) {
  const float cosine = std::cos(square.angle);
  const float sine = std::sin(square.angle);
  return {{{cosine, sine}, {-sine, cosine}}};
}

Projection Project(const std::array<Vec2, 4>& vertices, Vec2 axis) {
  Projection projection{Dot(vertices.front(), axis),
                        Dot(vertices.front(), axis)};
  for (const Vec2 vertex : vertices) {
    const float value = Dot(vertex, axis);
    projection.minimum = std::min(projection.minimum, value);
    projection.maximum = std::max(projection.maximum, value);
  }
  return projection;
}

SupportFeature FindSupportFeature(const std::array<Vec2, 4>& vertices,
                                  Vec2 axis, bool find_maximum) {
  SupportFeature feature;
  float target = find_maximum ? -std::numeric_limits<float>::infinity()
                              : std::numeric_limits<float>::infinity();

  for (const Vec2 vertex : vertices) {
    const float projection = Dot(vertex, axis);
    const bool is_better = find_maximum ? projection > target + kSupportEpsilon
                                        : projection < target - kSupportEpsilon;
    if (is_better) {
      target = projection;
      feature.points[0] = vertex;
      feature.count = 1;
    } else if (std::abs(projection - target) <= kSupportEpsilon &&
               feature.count < feature.points.size()) {
      feature.points[feature.count] = vertex;
      ++feature.count;
    }
  }
  return feature;
}

Vec2 ClosestPointOnSegment(Vec2 point, Vec2 segment_a, Vec2 segment_b) {
  const Vec2 segment = Subtract(segment_b, segment_a);
  const float length_squared = LengthSquared(segment);
  if (length_squared == 0.0f) {
    return segment_a;
  }
  const float parameter = std::clamp(
      Dot(Subtract(point, segment_a), segment) / length_squared, 0.0f, 1.0f);
  return Add(segment_a, Multiply(segment, parameter));
}

Vec2 GetContactPoint(const SupportFeature& feature_a,
                     const SupportFeature& feature_b, Vec2 normal) {
  if (feature_a.count == 1 && feature_b.count == 1) {
    return Multiply(Add(feature_a.points[0], feature_b.points[0]), 0.5f);
  }
  if (feature_a.count == 1 && feature_b.count == 2) {
    return ClosestPointOnSegment(feature_a.points[0], feature_b.points[0],
                                 feature_b.points[1]);
  }
  if (feature_a.count == 2 && feature_b.count == 1) {
    return ClosestPointOnSegment(feature_b.points[0], feature_a.points[0],
                                 feature_a.points[1]);
  }

  const Vec2 tangent =
      Normalize(Subtract(feature_a.points[1], feature_a.points[0]));
  const float minimum_a = std::min(Dot(feature_a.points[0], tangent),
                                   Dot(feature_a.points[1], tangent));
  const float maximum_a = std::max(Dot(feature_a.points[0], tangent),
                                   Dot(feature_a.points[1], tangent));
  const float minimum_b = std::min(Dot(feature_b.points[0], tangent),
                                   Dot(feature_b.points[1], tangent));
  const float maximum_b = std::max(Dot(feature_b.points[0], tangent),
                                   Dot(feature_b.points[1], tangent));
  const float tangent_coordinate =
      (std::max(minimum_a, minimum_b) + std::min(maximum_a, maximum_b)) * 0.5f;
  const float normal_coordinate =
      (Dot(feature_a.points[0], normal) + Dot(feature_a.points[1], normal) +
       Dot(feature_b.points[0], normal) + Dot(feature_b.points[1], normal)) *
      0.25f;
  return Add(Multiply(tangent, tangent_coordinate),
             Multiply(normal, normal_coordinate));
}

std::optional<Contact> FindContact(const Square& square_a,
                                   const Square& square_b) {
  const std::array<Vec2, 4> vertices_a = GetVertices(square_a);
  const std::array<Vec2, 4> vertices_b = GetVertices(square_b);
  const std::array<Vec2, 2> axes_a = GetAxes(square_a);
  const std::array<Vec2, 2> axes_b = GetAxes(square_b);
  const std::array<Vec2, 4> axes = {axes_a[0], axes_a[1], axes_b[0], axes_b[1]};

  float minimum_overlap = std::numeric_limits<float>::infinity();
  Vec2 collision_normal{};
  for (Vec2 axis : axes) {
    axis = Normalize(axis);
    const Projection projection_a = Project(vertices_a, axis);
    const Projection projection_b = Project(vertices_b, axis);
    const float overlap = std::min(projection_a.maximum - projection_b.minimum,
                                   projection_b.maximum - projection_a.minimum);
    if (overlap <= 0.0f) {
      return std::nullopt;
    }
    if (overlap < minimum_overlap) {
      minimum_overlap = overlap;
      collision_normal = axis;
    }
  }

  if (Dot(Subtract(square_b.position, square_a.position), collision_normal) <
      0.0f) {
    collision_normal = Multiply(collision_normal, -1.0f);
  }

  const SupportFeature feature_a =
      FindSupportFeature(vertices_a, collision_normal, true);
  const SupportFeature feature_b =
      FindSupportFeature(vertices_b, collision_normal, false);
  return Contact{collision_normal,
                 GetContactPoint(feature_a, feature_b, collision_normal),
                 minimum_overlap};
}

float Restitution(float coefficient, float normal_velocity) {
  if (std::abs(normal_velocity) < kRestitutionVelocityThreshold) {
    return 0.0f;
  }
  return std::clamp(coefficient, 0.0f, 1.0f);
}

void ResolveContact(Square& square_a, Square& square_b, const Contact& contact,
                    float coefficient) {
  const float inverse_mass_a = InverseMass(square_a);
  const float inverse_mass_b = InverseMass(square_b);
  const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;
  if (inverse_mass_sum == 0.0f) {
    return;
  }

  const Vec2 radius_a = Subtract(contact.point, square_a.position);
  const Vec2 radius_b = Subtract(contact.point, square_b.position);
  Vec2 relative_velocity = Subtract(ContactVelocity(square_b, radius_b),
                                    ContactVelocity(square_a, radius_a));
  const float normal_velocity = Dot(relative_velocity, contact.normal);
  float normal_impulse_magnitude = 0.0f;

  if (normal_velocity < 0.0f) {
    const float denominator =
        ImpulseDenominator(square_a, radius_a, contact.normal) +
        ImpulseDenominator(square_b, radius_b, contact.normal);
    normal_impulse_magnitude =
        -(1.0f + Restitution(coefficient, normal_velocity)) * normal_velocity /
        denominator;
    const Vec2 impulse = Multiply(contact.normal, normal_impulse_magnitude);
    ApplyImpulse(square_a, Multiply(impulse, -1.0f), radius_a);
    ApplyImpulse(square_b, impulse, radius_b);
  }

  if (normal_impulse_magnitude > 0.0f) {
    relative_velocity = Subtract(ContactVelocity(square_b, radius_b),
                                 ContactVelocity(square_a, radius_a));
    const Vec2 tangent_velocity = Subtract(
        relative_velocity,
        Multiply(contact.normal, Dot(relative_velocity, contact.normal)));
    if (LengthSquared(tangent_velocity) > 0.0f) {
      const Vec2 tangent = Normalize(tangent_velocity);
      const float denominator =
          ImpulseDenominator(square_a, radius_a, tangent) +
          ImpulseDenominator(square_b, radius_b, tangent);
      const float maximum_friction =
          kFrictionCoefficient * normal_impulse_magnitude;
      const float friction_magnitude =
          std::clamp(-Dot(relative_velocity, tangent) / denominator,
                     -maximum_friction, maximum_friction);
      const Vec2 friction_impulse = Multiply(tangent, friction_magnitude);
      ApplyImpulse(square_a, Multiply(friction_impulse, -1.0f), radius_a);
      ApplyImpulse(square_b, friction_impulse, radius_b);
    }
  }

  const float correction_magnitude =
      std::max(contact.penetration - kPositionSlop, 0.0f) *
      kPositionCorrection / inverse_mass_sum;
  const Vec2 correction = Multiply(contact.normal, correction_magnitude);
  square_a.position =
      Subtract(square_a.position, Multiply(correction, inverse_mass_a));
  square_b.position =
      Add(square_b.position, Multiply(correction, inverse_mass_b));
}

void ResolveWallContact(Square& square, Vec2 inward_normal, float offset,
                        float coefficient) {
  const std::array<Vec2, 4> vertices = GetVertices(square);
  const Projection projection = Project(vertices, inward_normal);
  if (projection.minimum >= offset) {
    return;
  }

  const float inverse_mass = InverseMass(square);
  if (inverse_mass == 0.0f) {
    return;
  }

  const SupportFeature feature =
      FindSupportFeature(vertices, inward_normal, false);
  Vec2 contact_point{};
  for (std::size_t i = 0; i < feature.count; ++i) {
    contact_point = Add(contact_point, feature.points[i]);
  }
  contact_point = Multiply(contact_point, 1.0f / feature.count);

  const Vec2 radius = Subtract(contact_point, square.position);
  Vec2 contact_velocity = ContactVelocity(square, radius);
  const float normal_velocity = Dot(contact_velocity, inward_normal);
  float normal_impulse_magnitude = 0.0f;
  if (normal_velocity < 0.0f) {
    const float denominator = ImpulseDenominator(square, radius, inward_normal);
    normal_impulse_magnitude =
        -(1.0f + Restitution(coefficient, normal_velocity)) * normal_velocity /
        denominator;
    ApplyImpulse(square, Multiply(inward_normal, normal_impulse_magnitude),
                 radius);
  }

  if (normal_impulse_magnitude > 0.0f) {
    contact_velocity = ContactVelocity(square, radius);
    const Vec2 tangent_velocity =
        Subtract(contact_velocity,
                 Multiply(inward_normal, Dot(contact_velocity, inward_normal)));
    if (LengthSquared(tangent_velocity) > 0.0f) {
      const Vec2 tangent = Normalize(tangent_velocity);
      const float maximum_friction =
          kFrictionCoefficient * normal_impulse_magnitude;
      const float friction_magnitude =
          std::clamp(-Dot(contact_velocity, tangent) /
                         ImpulseDenominator(square, radius, tangent),
                     -maximum_friction, maximum_friction);
      ApplyImpulse(square, Multiply(tangent, friction_magnitude), radius);
    }
  }

  square.position = Add(square.position,
                        Multiply(inward_normal, offset - projection.minimum));
}

void ResolveWindowCollision(Square& square, float area_width, float area_height,
                            float coefficient) {
  ResolveWallContact(square, {1.0f, 0.0f}, 0.0f, coefficient);
  ResolveWallContact(square, {-1.0f, 0.0f}, -area_width, coefficient);
  ResolveWallContact(square, {0.0f, 1.0f}, 0.0f, coefficient);
  ResolveWallContact(square, {0.0f, -1.0f}, -area_height, coefficient);
}

}  // namespace

std::array<Vec2, 4> GetVertices(const Square& square) {
  const float half_size = square.size * 0.5f;
  const float cosine = std::cos(square.angle);
  const float sine = std::sin(square.angle);
  const std::array<Vec2, 4> local_vertices = {
      Vec2{-half_size, -half_size}, Vec2{half_size, -half_size},
      Vec2{half_size, half_size}, Vec2{-half_size, half_size}};

  std::array<Vec2, 4> vertices{};
  for (std::size_t i = 0; i < local_vertices.size(); ++i) {
    const Vec2 local = local_vertices[i];
    vertices[i] = {square.position.x + local.x * cosine - local.y * sine,
                   square.position.y + local.x * sine + local.y * cosine};
  }
  return vertices;
}

bool IsColliding(const Square& square_a, const Square& square_b) {
  return FindContact(square_a, square_b).has_value();
}

void Update(std::vector<Square>& squares, float delta_time, float area_width,
            float area_height, float coefficient) {
  for (Square& square : squares) {
    if (InverseMass(square) == 0.0f) {
      continue;
    }
    square.velocity.y += kGravity * delta_time;
    square.angular_velocity *= std::exp(-kAngularDamping * delta_time);
    square.position =
        Add(square.position, Multiply(square.velocity, delta_time));
    square.angle = std::remainder(
        square.angle + square.angular_velocity * delta_time, kTwoPi);
  }

  for (int iteration = 0; iteration < kSolverIterations; ++iteration) {
    for (std::size_t i = 0; i < squares.size(); ++i) {
      for (std::size_t j = i + 1; j < squares.size(); ++j) {
        const std::optional<Contact> contact =
            FindContact(squares[i], squares[j]);
        if (contact.has_value()) {
          ResolveContact(squares[i], squares[j], *contact, coefficient);
        }
      }
    }

    for (Square& square : squares) {
      ResolveWindowCollision(square, area_width, area_height, coefficient);
    }
  }
}

}  // namespace tiny2d
