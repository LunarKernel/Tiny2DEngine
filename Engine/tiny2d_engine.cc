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

struct CircleImpact {
  double time;
  std::size_t first;
  std::size_t second;
};

struct SupportFeature {
  std::array<Vec2, 2> points{};
  std::size_t count{};
};

struct ResolvedMaterial {
  float restitution;
  float static_friction;
  float kinetic_friction;
};

bool IsFinite(Vec2 vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y);
}

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
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

ResolvedMaterial GetWorldMaterial(float restitution, float friction) {
  return {restitution, friction, friction};
}

ResolvedMaterial ResolveMaterial(const CollisionMaterial& material,
                                 float world_restitution,
                                 float world_friction) {
  if (UsesWorldMaterial(material)) {
    return GetWorldMaterial(world_restitution, world_friction);
  }
  return {material.restitution, material.static_friction,
          material.kinetic_friction};
}

float GeometricMean(float a, float b) {
  return static_cast<float>(
      std::sqrt(static_cast<double>(a) * static_cast<double>(b)));
}

ResolvedMaterial MixMaterials(const CollisionMaterial& material_a,
                              const CollisionMaterial& material_b,
                              float world_restitution, float world_friction) {
  if (UsesWorldMaterial(material_a) && UsesWorldMaterial(material_b)) {
    return GetWorldMaterial(world_restitution, world_friction);
  }
  const ResolvedMaterial a =
      ResolveMaterial(material_a, world_restitution, world_friction);
  const ResolvedMaterial b =
      ResolveMaterial(material_b, world_restitution, world_friction);
  return {std::max(a.restitution, b.restitution),
          GeometricMean(a.static_friction, b.static_friction),
          GeometricMean(a.kinetic_friction, b.kinetic_friction)};
}

ResolvedMaterial MixWithWorld(const CollisionMaterial& material,
                              float world_restitution, float world_friction) {
  if (UsesWorldMaterial(material)) {
    return GetWorldMaterial(world_restitution, world_friction);
  }
  const ResolvedMaterial body =
      ResolveMaterial(material, world_restitution, world_friction);
  return {std::max(body.restitution, world_restitution),
          GeometricMean(body.static_friction, world_friction),
          GeometricMean(body.kinetic_friction, world_friction)};
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

bool IsKnownInertiaModel(CircleInertiaModel model) {
  switch (model) {
    case CircleInertiaModel::kSolidDisk:
    case CircleInertiaModel::kHoop:
      return true;
  }
  return false;
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

void RequireFloatResult(double value, const char* message) {
  Require(std::isfinite(value) &&
              std::abs(value) <= std::numeric_limits<float>::max(),
          message);
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

template <typename Body>
float InverseMass(const Body& body) {
  return body.mass > 0.0f ? 1.0f / body.mass : 0.0f;
}

double MomentOfInertiaUnchecked(const Rectangle& rectangle) {
  return static_cast<double>(rectangle.mass) *
         (static_cast<double>(rectangle.width) * rectangle.width +
          static_cast<double>(rectangle.height) * rectangle.height) /
         12.0;
}

double MomentOfInertiaUnchecked(const Circle& circle) {
  const double factor =
      circle.inertia_model == CircleInertiaModel::kSolidDisk ? 0.5 : 1.0;
  return factor * static_cast<double>(circle.mass) * circle.radius *
         circle.radius;
}

float InverseInertia(const Rectangle& square) {
  if (square.fixed_rotation || square.mass <= 0.0f || square.width <= 0.0f ||
      square.height <= 0.0f) {
    return 0.0f;
  }
  return 12.0f / (square.mass * (square.width * square.width +
                                 square.height * square.height));
}

float InverseInertia(const Circle& circle) {
  if (circle.fixed_rotation || circle.mass <= 0.0f || circle.radius <= 0.0f ||
      !IsKnownInertiaModel(circle.inertia_model)) {
    return 0.0f;
  }
  return static_cast<float>(1.0 / MomentOfInertiaUnchecked(circle));
}

template <typename Body>
Vec2 ContactVelocity(const Body& body, Vec2 radius) {
  return Add(body.velocity, Cross(body.angular_velocity, radius));
}

template <typename Body>
float ImpulseDenominator(const Body& body, Vec2 radius, Vec2 direction) {
  const float radius_cross_direction = Cross(radius, direction);
  return InverseMass(body) +
         radius_cross_direction * radius_cross_direction * InverseInertia(body);
}

template <typename Body>
void ApplyImpulse(Body& body, Vec2 impulse, Vec2 radius) {
  body.velocity = Add(body.velocity, Multiply(impulse, InverseMass(body)));
  body.angular_velocity += Cross(radius, impulse) * InverseInertia(body);
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

std::optional<ContactManifold> FindContact(const Circle& circle_a,
                                           const Circle& circle_b) {
  const Vec2 center_delta = Subtract(circle_b.position, circle_a.position);
  const double distance_squared =
      static_cast<double>(center_delta.x) * center_delta.x +
      static_cast<double>(center_delta.y) * center_delta.y;
  const double radius_sum =
      static_cast<double>(circle_a.radius) + circle_b.radius;
  if (distance_squared >= radius_sum * radius_sum) {
    return std::nullopt;
  }

  Vec2 normal{};
  double distance = 0.0;
  if (distance_squared > 0.0) {
    distance = std::sqrt(distance_squared);
    normal = Multiply(center_delta, static_cast<float>(1.0 / distance));
  } else {
    const Vec2 relative_velocity =
        Subtract(circle_b.velocity, circle_a.velocity);
    const double relative_speed_squared =
        static_cast<double>(relative_velocity.x) * relative_velocity.x +
        static_cast<double>(relative_velocity.y) * relative_velocity.y;
    if (relative_speed_squared > 0.0) {
      const double inverse_speed = 1.0 / std::sqrt(relative_speed_squared);
      normal = {-static_cast<float>(relative_velocity.x * inverse_speed),
                -static_cast<float>(relative_velocity.y * inverse_speed)};
    } else {
      normal = {1.0f, 0.0f};
    }
  }

  const Vec2 surface_a =
      Add(circle_a.position, Multiply(normal, circle_a.radius));
  const Vec2 surface_b =
      Subtract(circle_b.position, Multiply(normal, circle_b.radius));
  ContactManifold manifold{normal,
                           {Multiply(Add(surface_a, surface_b), 0.5f), {}},
                           1,
                           static_cast<float>(radius_sum - distance)};
  return manifold;
}

std::optional<double> FindCircleTimeOfImpact(const Circle& circle_a,
                                             const Circle& circle_b,
                                             double maximum_time) {
  if (InverseMass(circle_a) + InverseMass(circle_b) == 0.0f) {
    return std::nullopt;
  }

  const double delta_x =
      static_cast<double>(circle_b.position.x) - circle_a.position.x;
  const double delta_y =
      static_cast<double>(circle_b.position.y) - circle_a.position.y;
  const double velocity_x =
      static_cast<double>(circle_b.velocity.x) - circle_a.velocity.x;
  const double velocity_y =
      static_cast<double>(circle_b.velocity.y) - circle_a.velocity.y;
  const double radius_sum =
      static_cast<double>(circle_a.radius) + circle_b.radius;
  const double c =
      delta_x * delta_x + delta_y * delta_y - radius_sum * radius_sum;
  const double a = velocity_x * velocity_x + velocity_y * velocity_y;
  const double b = delta_x * velocity_x + delta_y * velocity_y;
  if (c < 0.0 || a == 0.0 || b >= 0.0) {
    return std::nullopt;
  }

  const double discriminant = b * b - a * c;
  if (discriminant < 0.0) {
    return std::nullopt;
  }
  const double time = (-b - std::sqrt(discriminant)) / a;
  if (time < 0.0 || time > maximum_time) {
    return std::nullopt;
  }
  return time;
}

bool HasMissedCircleImpact(const std::vector<Circle>& starts,
                           const std::vector<Circle>& integrated,
                           float delta_time) {
  for (std::size_t i = 0; i < starts.size(); ++i) {
    for (std::size_t j = i + 1; j < starts.size(); ++j) {
      const double radius_sum =
          static_cast<double>(starts[i].radius) + starts[j].radius;
      const Vec2 start_delta = Subtract(starts[j].position, starts[i].position);
      const Vec2 end_delta =
          Subtract(integrated[j].position, integrated[i].position);
      const double start_distance_squared =
          static_cast<double>(start_delta.x) * start_delta.x +
          static_cast<double>(start_delta.y) * start_delta.y;
      const double end_distance_squared =
          static_cast<double>(end_delta.x) * end_delta.x +
          static_cast<double>(end_delta.y) * end_delta.y;
      if (start_distance_squared < radius_sum * radius_sum ||
          end_distance_squared < radius_sum * radius_sum) {
        continue;
      }

      Circle moving_a = starts[i];
      Circle moving_b = starts[j];
      moving_a.velocity = integrated[i].velocity;
      moving_b.velocity = integrated[j].velocity;
      if (FindCircleTimeOfImpact(moving_a, moving_b, delta_time).has_value()) {
        return true;
      }
    }
  }
  return false;
}

std::optional<CircleImpact> FindEarliestCircleImpact(
    const std::vector<Circle>& circles, float maximum_time) {
  std::optional<CircleImpact> earliest;
  for (std::size_t i = 0; i < circles.size(); ++i) {
    for (std::size_t j = i + 1; j < circles.size(); ++j) {
      const std::optional<double> time =
          FindCircleTimeOfImpact(circles[i], circles[j], maximum_time);
      if (time.has_value() &&
          (!earliest.has_value() || *time < earliest->time)) {
        earliest = CircleImpact{*time, i, j};
      }
    }
  }
  return earliest;
}

ContactManifold MakeCircleImpactContact(const Circle& circle_a,
                                        const Circle& circle_b) {
  const Vec2 delta = Subtract(circle_b.position, circle_a.position);
  const double distance = std::sqrt(static_cast<double>(delta.x) * delta.x +
                                    static_cast<double>(delta.y) * delta.y);
  const Vec2 normal = distance > 0.0
                          ? Multiply(delta, static_cast<float>(1.0 / distance))
                          : Vec2{1.0f, 0.0f};
  const Vec2 surface_a =
      Add(circle_a.position, Multiply(normal, circle_a.radius));
  const Vec2 surface_b =
      Subtract(circle_b.position, Multiply(normal, circle_b.radius));
  const double penetration =
      static_cast<double>(circle_a.radius) + circle_b.radius - distance;
  return {normal,
          {Multiply(Add(surface_a, surface_b), 0.5f), {}},
          1,
          static_cast<float>(std::max(penetration, 0.0))};
}

Vec2 RotateToWorld(Vec2 local, float angle) {
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {local.x * cosine - local.y * sine, local.x * sine + local.y * cosine};
}

Vec2 RotateToLocal(Vec2 world, float angle) {
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  return {world.x * cosine + world.y * sine,
          -world.x * sine + world.y * cosine};
}

std::optional<ContactManifold> FindContact(const Rectangle& rectangle,
                                           const Circle& circle) {
  const Vec2 local_center = RotateToLocal(
      Subtract(circle.position, rectangle.position), rectangle.angle);
  const float half_width = rectangle.width * 0.5f;
  const float half_height = rectangle.height * 0.5f;
  Vec2 closest{std::clamp(local_center.x, -half_width, half_width),
               std::clamp(local_center.y, -half_height, half_height)};
  const Vec2 local_delta = Subtract(local_center, closest);
  const double distance_squared =
      static_cast<double>(local_delta.x) * local_delta.x +
      static_cast<double>(local_delta.y) * local_delta.y;

  Vec2 local_normal{};
  float penetration = 0.0f;
  Vec2 circle_surface{};
  if (distance_squared > 0.0) {
    const double distance = std::sqrt(distance_squared);
    if (distance >= circle.radius) {
      return std::nullopt;
    }
    local_normal = Multiply(local_delta, static_cast<float>(1.0 / distance));
    penetration = circle.radius - static_cast<float>(distance);
  } else {
    const float distance_to_x_face = half_width - std::abs(local_center.x);
    const float distance_to_y_face = half_height - std::abs(local_center.y);
    if (distance_to_x_face <= distance_to_y_face) {
      local_normal = {local_center.x >= 0.0f ? 1.0f : -1.0f, 0.0f};
      closest = {local_normal.x * half_width, local_center.y};
      penetration = circle.radius + distance_to_x_face;
    } else {
      local_normal = {0.0f, local_center.y >= 0.0f ? 1.0f : -1.0f};
      closest = {local_center.x, local_normal.y * half_height};
      penetration = circle.radius + distance_to_y_face;
    }
  }

  const Vec2 normal = RotateToWorld(local_normal, rectangle.angle);
  const Vec2 rectangle_surface =
      Add(rectangle.position, RotateToWorld(closest, rectangle.angle));
  circle_surface = Subtract(circle.position, Multiply(normal, circle.radius));
  ContactManifold manifold{
      normal,
      {Multiply(Add(rectangle_surface, circle_surface), 0.5f), {}},
      1,
      penetration};
  return manifold;
}

float Restitution(float restitution, float normal_velocity,
                  bool allow_restitution, float velocity_threshold) {
  if (!allow_restitution || std::abs(normal_velocity) < velocity_threshold) {
    return 0.0f;
  }
  return std::clamp(restitution, 0.0f, 1.0f);
}

template <typename BodyA, typename BodyB>
std::array<float, 2> ResolveNormalImpulses(
    BodyA& square_a, BodyB& square_b, const ContactManifold& manifold,
    const std::array<Vec2, 2>& radii_a, const std::array<Vec2, 2>& radii_b,
    float restitution, bool allow_restitution,
    float restitution_velocity_threshold) {
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
          -Restitution(restitution, initial_velocities[i], allow_restitution,
                       restitution_velocity_threshold) *
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

template <typename BodyA, typename BodyB>
void ResolveContact(BodyA& square_a, BodyB& square_b,
                    const ContactManifold& manifold,
                    const ResolvedMaterial& material, bool allow_restitution,
                    float restitution_velocity_threshold) {
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
  const std::array<float, 2> normal_impulses = ResolveNormalImpulses(
      square_a, square_b, manifold, radii_a, radii_b, material.restitution,
      allow_restitution, restitution_velocity_threshold);

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
      const float required_friction =
          -Dot(relative_velocity, tangent) / denominator;
      const float maximum_static_friction =
          material.static_friction * normal_impulses[i];
      float friction_magnitude = required_friction;
      if (std::abs(required_friction) > maximum_static_friction) {
        friction_magnitude =
            -std::copysign(material.kinetic_friction * normal_impulses[i],
                           Dot(relative_velocity, tangent));
      }
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
                        const ResolvedMaterial& material,
                        bool allow_restitution,
                        float restitution_velocity_threshold) {
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
        -(1.0f + Restitution(material.restitution, normal_velocity,
                             allow_restitution,
                             restitution_velocity_threshold)) *
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
      const float tangent_speed = Dot(contact_velocity, tangent);
      const float required_friction =
          -tangent_speed / ImpulseDenominator(square, radius, tangent);
      const float maximum_static_friction =
          material.static_friction * normal_impulse_magnitude;
      float friction_magnitude = required_friction;
      if (std::abs(required_friction) > maximum_static_friction) {
        friction_magnitude =
            -std::copysign(material.kinetic_friction * normal_impulse_magnitude,
                           tangent_speed);
      }
      ApplyImpulse(square, Multiply(tangent, friction_magnitude), radius);
    }
  }

  square.position = Add(square.position,
                        Multiply(inward_normal, offset - projection.minimum));
}

void ResolveWallContact(Circle& circle, Vec2 inward_normal, float offset,
                        const ResolvedMaterial& material,
                        bool allow_restitution,
                        float restitution_velocity_threshold) {
  const float minimum_projection =
      Dot(circle.position, inward_normal) - circle.radius;
  if (minimum_projection >= offset || InverseMass(circle) == 0.0f) {
    return;
  }

  const Vec2 radius = Multiply(inward_normal, -circle.radius);
  Vec2 contact_velocity = ContactVelocity(circle, radius);
  const float normal_velocity = Dot(contact_velocity, inward_normal);
  float normal_impulse_magnitude = 0.0f;
  if (normal_velocity < 0.0f) {
    const float denominator = ImpulseDenominator(circle, radius, inward_normal);
    normal_impulse_magnitude =
        -(1.0f + Restitution(material.restitution, normal_velocity,
                             allow_restitution,
                             restitution_velocity_threshold)) *
        normal_velocity / denominator;
    ApplyImpulse(circle, Multiply(inward_normal, normal_impulse_magnitude),
                 radius);
  }

  if (normal_impulse_magnitude > 0.0f) {
    contact_velocity = ContactVelocity(circle, radius);
    const Vec2 tangent_velocity =
        Subtract(contact_velocity,
                 Multiply(inward_normal, Dot(contact_velocity, inward_normal)));
    if (LengthSquared(tangent_velocity) > 0.0f) {
      const Vec2 tangent = Normalize(tangent_velocity);
      const float tangent_speed = Dot(contact_velocity, tangent);
      const float required_friction =
          -tangent_speed / ImpulseDenominator(circle, radius, tangent);
      const float maximum_static_friction =
          material.static_friction * normal_impulse_magnitude;
      float friction_magnitude = required_friction;
      if (std::abs(required_friction) > maximum_static_friction) {
        friction_magnitude =
            -std::copysign(material.kinetic_friction * normal_impulse_magnitude,
                           tangent_speed);
      }
      ApplyImpulse(circle, Multiply(tangent, friction_magnitude), radius);
    }
  }

  circle.position = Add(circle.position,
                        Multiply(inward_normal, offset - minimum_projection));
}

void ResolveWindowCollision(Rectangle& square, float area_width,
                            float area_height, float restitution,
                            float friction, bool allow_restitution,
                            float restitution_velocity_threshold) {
  const ResolvedMaterial material =
      MixWithWorld(square.material, restitution, friction);
  ResolveWallContact(square, {1.0f, 0.0f}, 0.0f, material, allow_restitution,
                     restitution_velocity_threshold);
  ResolveWallContact(square, {-1.0f, 0.0f}, -area_width, material,
                     allow_restitution, restitution_velocity_threshold);
  ResolveWallContact(square, {0.0f, 1.0f}, 0.0f, material, allow_restitution,
                     restitution_velocity_threshold);
  ResolveWallContact(square, {0.0f, -1.0f}, -area_height, material,
                     allow_restitution, restitution_velocity_threshold);
}

void ResolveWindowCollision(Circle& circle, float area_width, float area_height,
                            float restitution, float friction,
                            bool allow_restitution,
                            float restitution_velocity_threshold) {
  const ResolvedMaterial material =
      MixWithWorld(circle.material, restitution, friction);
  ResolveWallContact(circle, {1.0f, 0.0f}, 0.0f, material, allow_restitution,
                     restitution_velocity_threshold);
  ResolveWallContact(circle, {-1.0f, 0.0f}, -area_width, material,
                     allow_restitution, restitution_velocity_threshold);
  ResolveWallContact(circle, {0.0f, 1.0f}, 0.0f, material, allow_restitution,
                     restitution_velocity_threshold);
  ResolveWallContact(circle, {0.0f, -1.0f}, -area_height, material,
                     allow_restitution, restitution_velocity_threshold);
}

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
                         allow_restitution, kRestitutionVelocityThreshold);
        }
      }
    }

    for (Rectangle& square : squares) {
      ResolveWindowCollision(square, area_width, area_height, restitution,
                             friction, allow_restitution,
                             kRestitutionVelocityThreshold);
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
      // ponytail: O(n^2) scans and a 4N event cap fit the supported small
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
