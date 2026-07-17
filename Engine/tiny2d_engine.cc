#include "tiny2d_engine.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace tiny2d {
namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr float kSupportEpsilon = 0.0001f;
constexpr float kPositionSlop = 0.01f;
constexpr float kPositionCorrection = 0.8f;
constexpr float kRestitutionVelocityThreshold = 20.0f;
constexpr float kAngularDamping = 0.7f;
constexpr int kSolverIterations = 4;
constexpr float kMinimumDynamicMass = 0.000001f;

struct Projection {
  float minimum;
  float maximum;
};

struct Face {
  Vec2 start;
  Vec2 end;
  Vec2 normal;
};

struct ClipPoints {
  std::array<Vec2, 2> points{};
  std::size_t count{};
};

struct ContactManifold {
  Vec2 normal;
  std::array<Vec2, 2> points{};
  std::size_t point_count{};
  float penetration;
};

struct SupportFeature {
  std::array<Vec2, 2> points{};
  std::size_t count{};
};

bool IsFinite(Vec2 vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y);
}

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
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

void ValidateRectangle(const Rectangle& rectangle) {
  ValidateGeometry(rectangle);
  Require(std::isfinite(rectangle.mass) && IsFinite(rectangle.velocity) &&
              std::isfinite(rectangle.angular_velocity) &&
              std::isfinite(rectangle.charge),
          "Rectangle state must contain only finite values.");
  Require(rectangle.mass == 0.0f || rectangle.mass >= kMinimumDynamicMass,
          "Rectangle mass must be zero for a static body or at least 1e-6.");
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

void RequireFloatResult(double value, const char* message) {
  Require(std::isfinite(value) &&
              std::abs(value) <= std::numeric_limits<float>::max(),
          message);
}

Vec2 GetLinearAccelerationUnchecked(const Rectangle& rectangle,
                                    Vec2 electric_field, float gravity) {
  if (rectangle.mass <= 0.0f) {
    return {};
  }
  const float charge_over_mass = rectangle.charge / rectangle.mass;
  return {electric_field.x * charge_over_mass,
          gravity + electric_field.y * charge_over_mass};
}

std::array<Vec2, 4> GetVerticesUnchecked(const Rectangle& rectangle) {
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

float InverseMass(const Rectangle& square) {
  return square.mass > 0.0f ? 1.0f / square.mass : 0.0f;
}

float InverseInertia(const Rectangle& square) {
  if (square.fixed_rotation || square.mass <= 0.0f || square.width <= 0.0f ||
      square.height <= 0.0f) {
    return 0.0f;
  }
  return 12.0f / (square.mass * (square.width * square.width +
                                 square.height * square.height));
}

Vec2 ContactVelocity(const Rectangle& square, Vec2 radius) {
  return Add(square.velocity, Cross(square.angular_velocity, radius));
}

float ImpulseDenominator(const Rectangle& square, Vec2 radius, Vec2 direction) {
  const float radius_cross_direction = Cross(radius, direction);
  return InverseMass(square) + radius_cross_direction * radius_cross_direction *
                                   InverseInertia(square);
}

void ApplyImpulse(Rectangle& square, Vec2 impulse, Vec2 radius) {
  square.velocity =
      Add(square.velocity, Multiply(impulse, InverseMass(square)));
  square.angular_velocity += Cross(radius, impulse) * InverseInertia(square);
}

std::array<Vec2, 2> GetAxes(const Rectangle& square) {
  const float cosine = std::cos(square.angle);
  const float sine = std::sin(square.angle);
  return {{{cosine, sine}, {-sine, cosine}}};
}

std::array<Face, 4> GetFaces(const Rectangle& square) {
  const std::array<Vec2, 4> vertices = GetVerticesUnchecked(square);
  const std::array<Vec2, 2> axes = GetAxes(square);
  return {{{vertices[0], vertices[1], Multiply(axes[1], -1.0f)},
           {vertices[1], vertices[2], axes[0]},
           {vertices[2], vertices[3], axes[1]},
           {vertices[3], vertices[0], Multiply(axes[0], -1.0f)}}};
}

Face FindAlignedFace(const Rectangle& square, Vec2 direction) {
  const std::array<Face, 4> faces = GetFaces(square);
  Face selected_face = faces.front();
  float best_alignment = Dot(selected_face.normal, direction);
  for (const Face& face : faces) {
    const float alignment = Dot(face.normal, direction);
    if (alignment > best_alignment) {
      best_alignment = alignment;
      selected_face = face;
    }
  }
  return selected_face;
}

Face FindOpposingFace(const Rectangle& square, Vec2 direction) {
  const std::array<Face, 4> faces = GetFaces(square);
  Face selected_face = faces.front();
  float best_alignment = Dot(selected_face.normal, direction);
  for (const Face& face : faces) {
    const float alignment = Dot(face.normal, direction);
    if (alignment < best_alignment) {
      best_alignment = alignment;
      selected_face = face;
    }
  }
  return selected_face;
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

ClipPoints ClipToPlane(const ClipPoints& input, Vec2 normal, float offset) {
  ClipPoints output;
  if (input.count == 0) {
    return output;
  }
  if (input.count == 1) {
    if (Dot(input.points[0], normal) <= offset) {
      output.points[0] = input.points[0];
      output.count = 1;
    }
    return output;
  }

  const float distance_a = Dot(input.points[0], normal) - offset;
  const float distance_b = Dot(input.points[1], normal) - offset;
  const bool inside_a = distance_a <= 0.0f;
  const bool inside_b = distance_b <= 0.0f;
  if (inside_a) {
    output.points[output.count++] = input.points[0];
  }
  if (inside_b) {
    output.points[output.count++] = input.points[1];
  }
  if (inside_a != inside_b) {
    const float parameter = distance_a / (distance_a - distance_b);
    output.points[output.count++] =
        Add(input.points[0],
            Multiply(Subtract(input.points[1], input.points[0]), parameter));
  }
  return output;
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

std::optional<ContactManifold> FindContact(const Rectangle& square_a,
                                           const Rectangle& square_b) {
  const std::array<Vec2, 4> vertices_a = GetVerticesUnchecked(square_a);
  const std::array<Vec2, 4> vertices_b = GetVerticesUnchecked(square_b);
  const std::array<Vec2, 2> axes_a = GetAxes(square_a);
  const std::array<Vec2, 2> axes_b = GetAxes(square_b);
  const std::array<Vec2, 4> axes = {axes_a[0], axes_a[1], axes_b[0], axes_b[1]};

  float minimum_overlap = std::numeric_limits<float>::infinity();
  Vec2 collision_normal{};
  bool reference_is_a = true;
  for (std::size_t i = 0; i < axes.size(); ++i) {
    Vec2 axis = axes[i];
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
      reference_is_a = i < axes_a.size();
    }
  }

  if (Dot(Subtract(square_b.position, square_a.position), collision_normal) <
      0.0f) {
    collision_normal = Multiply(collision_normal, -1.0f);
  }

  const Rectangle& reference_square = reference_is_a ? square_a : square_b;
  const Rectangle& incident_square = reference_is_a ? square_b : square_a;
  const Vec2 reference_direction =
      reference_is_a ? collision_normal : Multiply(collision_normal, -1.0f);
  const Face reference_face =
      FindAlignedFace(reference_square, reference_direction);
  const Face incident_face =
      FindOpposingFace(incident_square, reference_face.normal);

  const Vec2 tangent =
      Normalize(Subtract(reference_face.end, reference_face.start));
  const float minimum_tangent = std::min(Dot(reference_face.start, tangent),
                                         Dot(reference_face.end, tangent));
  const float maximum_tangent = std::max(Dot(reference_face.start, tangent),
                                         Dot(reference_face.end, tangent));
  ClipPoints clipped{{incident_face.start, incident_face.end}, 2};
  clipped = ClipToPlane(clipped, Multiply(tangent, -1.0f), -minimum_tangent);
  clipped = ClipToPlane(clipped, tangent, maximum_tangent);

  ContactManifold manifold{collision_normal, {}, 0, minimum_overlap};
  const float reference_offset =
      Dot(reference_face.start, reference_face.normal);
  for (std::size_t i = 0; i < clipped.count; ++i) {
    const float separation =
        Dot(clipped.points[i], reference_face.normal) - reference_offset;
    if (separation <= kSupportEpsilon) {
      manifold.points[manifold.point_count++] =
          Subtract(clipped.points[i],
                   Multiply(reference_face.normal, separation * 0.5f));
    }
  }

  if (manifold.point_count == 0) {
    const SupportFeature feature_a =
        FindSupportFeature(vertices_a, collision_normal, true);
    const SupportFeature feature_b =
        FindSupportFeature(vertices_b, collision_normal, false);
    manifold.points[0] =
        GetContactPoint(feature_a, feature_b, collision_normal);
    manifold.point_count = 1;
  }
  return manifold;
}

float Restitution(float restitution, float normal_velocity,
                  bool allow_restitution) {
  if (!allow_restitution ||
      std::abs(normal_velocity) < kRestitutionVelocityThreshold) {
    return 0.0f;
  }
  return std::clamp(restitution, 0.0f, 1.0f);
}

std::array<float, 2> ResolveNormalImpulses(
    Rectangle& square_a, Rectangle& square_b, const ContactManifold& manifold,
    const std::array<Vec2, 2>& radii_a, const std::array<Vec2, 2>& radii_b,
    float restitution, bool allow_restitution) {
  std::array<float, 2> normal_impulses{};
  std::array<float, 2> target_velocities{};
  std::array<float, 2> initial_velocities{};

  for (std::size_t i = 0; i < manifold.point_count; ++i) {
    const Vec2 relative_velocity =
        Subtract(ContactVelocity(square_b, radii_b[i]),
                 ContactVelocity(square_a, radii_a[i]));
    initial_velocities[i] = Dot(relative_velocity, manifold.normal);
    if (initial_velocities[i] < 0.0f) {
      target_velocities[i] =
          -Restitution(restitution, initial_velocities[i], allow_restitution) *
          initial_velocities[i];
    }
  }

  if (manifold.point_count == 2) {
    // Solve both face contacts together so a symmetric impact cannot create
    // artificial rotation from contact processing order.
    const float inverse_mass_sum =
        InverseMass(square_a) + InverseMass(square_b);
    const float cross_a_0 = Cross(radii_a[0], manifold.normal);
    const float cross_a_1 = Cross(radii_a[1], manifold.normal);
    const float cross_b_0 = Cross(radii_b[0], manifold.normal);
    const float cross_b_1 = Cross(radii_b[1], manifold.normal);
    const float diagonal_0 =
        ImpulseDenominator(square_a, radii_a[0], manifold.normal) +
        ImpulseDenominator(square_b, radii_b[0], manifold.normal);
    const float diagonal_1 =
        ImpulseDenominator(square_a, radii_a[1], manifold.normal) +
        ImpulseDenominator(square_b, radii_b[1], manifold.normal);
    const float coupling = inverse_mass_sum +
                           cross_a_0 * cross_a_1 * InverseInertia(square_a) +
                           cross_b_0 * cross_b_1 * InverseInertia(square_b);
    const float determinant = diagonal_0 * diagonal_1 - coupling * coupling;

    if (determinant > kSupportEpsilon) {
      const float right_hand_side_0 =
          target_velocities[0] - initial_velocities[0];
      const float right_hand_side_1 =
          target_velocities[1] - initial_velocities[1];
      normal_impulses[0] =
          (right_hand_side_0 * diagonal_1 - right_hand_side_1 * coupling) /
          determinant;
      normal_impulses[1] =
          (right_hand_side_1 * diagonal_0 - right_hand_side_0 * coupling) /
          determinant;

      if (normal_impulses[0] >= 0.0f && normal_impulses[1] >= 0.0f) {
        for (std::size_t i = 0; i < manifold.point_count; ++i) {
          const Vec2 impulse = Multiply(manifold.normal, normal_impulses[i]);
          ApplyImpulse(square_a, Multiply(impulse, -1.0f), radii_a[i]);
          ApplyImpulse(square_b, impulse, radii_b[i]);
        }
        return normal_impulses;
      }
      normal_impulses = {};
    }
  }

  for (std::size_t i = 0; i < manifold.point_count; ++i) {
    const Vec2 relative_velocity =
        Subtract(ContactVelocity(square_b, radii_b[i]),
                 ContactVelocity(square_a, radii_a[i]));
    const float normal_velocity = Dot(relative_velocity, manifold.normal);
    const float denominator =
        ImpulseDenominator(square_a, radii_a[i], manifold.normal) +
        ImpulseDenominator(square_b, radii_b[i], manifold.normal);
    normal_impulses[i] =
        std::max((target_velocities[i] - normal_velocity) / denominator, 0.0f);
    const Vec2 impulse = Multiply(manifold.normal, normal_impulses[i]);
    ApplyImpulse(square_a, Multiply(impulse, -1.0f), radii_a[i]);
    ApplyImpulse(square_b, impulse, radii_b[i]);
  }
  return normal_impulses;
}

void ResolveContact(Rectangle& square_a, Rectangle& square_b,
                    const ContactManifold& manifold, float restitution,
                    float friction, bool allow_restitution) {
  const float inverse_mass_a = InverseMass(square_a);
  const float inverse_mass_b = InverseMass(square_b);
  const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;
  if (inverse_mass_sum == 0.0f) {
    return;
  }

  std::array<Vec2, 2> radii_a{};
  std::array<Vec2, 2> radii_b{};
  for (std::size_t i = 0; i < manifold.point_count; ++i) {
    radii_a[i] = Subtract(manifold.points[i], square_a.position);
    radii_b[i] = Subtract(manifold.points[i], square_b.position);
  }
  const std::array<float, 2> normal_impulses =
      ResolveNormalImpulses(square_a, square_b, manifold, radii_a, radii_b,
                            restitution, allow_restitution);

  for (std::size_t i = 0; i < manifold.point_count; ++i) {
    if (normal_impulses[i] <= 0.0f) {
      continue;
    }
    const Vec2 relative_velocity =
        Subtract(ContactVelocity(square_b, radii_b[i]),
                 ContactVelocity(square_a, radii_a[i]));
    const Vec2 tangent_velocity = Subtract(
        relative_velocity,
        Multiply(manifold.normal, Dot(relative_velocity, manifold.normal)));
    if (LengthSquared(tangent_velocity) > 0.0f) {
      const Vec2 tangent = Normalize(tangent_velocity);
      const float denominator =
          ImpulseDenominator(square_a, radii_a[i], tangent) +
          ImpulseDenominator(square_b, radii_b[i], tangent);
      const float maximum_friction =
          std::max(friction, 0.0f) * normal_impulses[i];
      const float friction_magnitude =
          std::clamp(-Dot(relative_velocity, tangent) / denominator,
                     -maximum_friction, maximum_friction);
      const Vec2 friction_impulse = Multiply(tangent, friction_magnitude);
      ApplyImpulse(square_a, Multiply(friction_impulse, -1.0f), radii_a[i]);
      ApplyImpulse(square_b, friction_impulse, radii_b[i]);
    }
  }

  const float correction_magnitude =
      std::max(manifold.penetration - kPositionSlop, 0.0f) *
      kPositionCorrection / inverse_mass_sum;
  const Vec2 correction = Multiply(manifold.normal, correction_magnitude);
  square_a.position =
      Subtract(square_a.position, Multiply(correction, inverse_mass_a));
  square_b.position =
      Add(square_b.position, Multiply(correction, inverse_mass_b));
}

void ResolveWallContact(Rectangle& square, Vec2 inward_normal, float offset,
                        float restitution, float friction,
                        bool allow_restitution) {
  const std::array<Vec2, 4> vertices = GetVerticesUnchecked(square);
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
        -(1.0f + Restitution(restitution, normal_velocity, allow_restitution)) *
        normal_velocity / denominator;
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
          std::max(friction, 0.0f) * normal_impulse_magnitude;
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

void ResolveWindowCollision(Rectangle& square, float area_width,
                            float area_height, float restitution,
                            float friction, bool allow_restitution) {
  ResolveWallContact(square, {1.0f, 0.0f}, 0.0f, restitution, friction,
                     allow_restitution);
  ResolveWallContact(square, {-1.0f, 0.0f}, -area_width, restitution, friction,
                     allow_restitution);
  ResolveWallContact(square, {0.0f, 1.0f}, 0.0f, restitution, friction,
                     allow_restitution);
  ResolveWallContact(square, {0.0f, -1.0f}, -area_height, restitution, friction,
                     allow_restitution);
}

}  // namespace

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

std::array<Vec2, 4> GetVertices(const Rectangle& rectangle) {
  ValidateGeometry(rectangle);
  return GetVerticesUnchecked(rectangle);
}

bool IsColliding(const Rectangle& rectangle_a, const Rectangle& rectangle_b) {
  ValidateGeometry(rectangle_a);
  ValidateGeometry(rectangle_b);
  return FindContact(rectangle_a, rectangle_b).has_value();
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

    const double acceleration_x =
        static_cast<double>(electric_field.x) * square.charge / square.mass;
    const double acceleration_y =
        static_cast<double>(gravity) +
        static_cast<double>(electric_field.y) * square.charge / square.mass;
    const double velocity_x = square.velocity.x + acceleration_x * delta_time;
    const double velocity_y = square.velocity.y + acceleration_y * delta_time;
    RequireFloatResult(acceleration_x,
                       "Electric acceleration exceeds the float range.");
    RequireFloatResult(acceleration_y,
                       "Combined acceleration exceeds the float range.");
    RequireFloatResult(velocity_x,
                       "Integrated horizontal velocity exceeds float range.");
    RequireFloatResult(velocity_y,
                       "Integrated vertical velocity exceeds float range.");
    RequireFloatResult(square.position.x + velocity_x * delta_time,
                       "Integrated horizontal position exceeds float range.");
    RequireFloatResult(square.position.y + velocity_y * delta_time,
                       "Integrated vertical position exceeds float range.");
    if (!square.fixed_rotation) {
      RequireFloatResult(
          static_cast<double>(square.angle) +
              static_cast<double>(square.angular_velocity) * delta_time,
          "Integrated angle exceeds float range.");
    }
  }

  for (Rectangle& square : squares) {
    if (InverseMass(square) == 0.0f) {
      continue;
    }
    square.velocity =
        Add(square.velocity, Multiply(GetLinearAccelerationUnchecked(
                                          square, electric_field, gravity),
                                      delta_time));
    if (square.fixed_rotation) {
      square.angular_velocity = 0.0f;
    } else {
      square.angular_velocity *= std::exp(-kAngularDamping * delta_time);
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
          ResolveContact(squares[i], squares[j], *contact, restitution,
                         friction, allow_restitution);
        }
      }
    }

    for (Rectangle& square : squares) {
      ResolveWindowCollision(square, area_width, area_height, restitution,
                             friction, allow_restitution);
    }
  }
}

}  // namespace tiny2d
