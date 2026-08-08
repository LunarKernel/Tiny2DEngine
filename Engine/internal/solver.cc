#include "internal/solver.h"

#include <algorithm>
#include <cmath>

#include "internal/body_math.h"
#include "internal/validation.h"

namespace tiny2d::internal {
namespace {

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

template <typename Body>
void ResolveWallContactImpulses(Body& body, Vec2 inward_normal, Vec2 radius,
                                const ResolvedMaterial& material,
                                bool allow_restitution,
                                float restitution_velocity_threshold) {
  Vec2 contact_velocity = ContactVelocity(body, radius);
  const float normal_velocity = Dot(contact_velocity, inward_normal);
  float normal_impulse_magnitude = 0.0f;
  if (normal_velocity < 0.0f) {
    const float denominator = ImpulseDenominator(body, radius, inward_normal);
    normal_impulse_magnitude =
        -(1.0f + Restitution(material.restitution, normal_velocity,
                             allow_restitution,
                             restitution_velocity_threshold)) *
        normal_velocity / denominator;
    ApplyImpulse(body, Multiply(inward_normal, normal_impulse_magnitude),
                 radius);
  }

  if (normal_impulse_magnitude > 0.0f) {
    contact_velocity = ContactVelocity(body, radius);
    const Vec2 tangent_velocity =
        Subtract(contact_velocity,
                 Multiply(inward_normal, Dot(contact_velocity, inward_normal)));
    if (LengthSquared(tangent_velocity) > 0.0f) {
      const Vec2 tangent = Normalize(tangent_velocity);
      const float tangent_speed = Dot(contact_velocity, tangent);
      const float required_friction =
          -tangent_speed / ImpulseDenominator(body, radius, tangent);
      const float maximum_static_friction =
          material.static_friction * normal_impulse_magnitude;
      float friction_magnitude = required_friction;
      if (std::abs(required_friction) > maximum_static_friction) {
        friction_magnitude =
            -std::copysign(material.kinetic_friction * normal_impulse_magnitude,
                           tangent_speed);
      }
      ApplyImpulse(body, Multiply(tangent, friction_magnitude), radius);
    }
  }
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
  ResolveWallContactImpulses(square, inward_normal, radius, material,
                             allow_restitution, restitution_velocity_threshold);

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
  ResolveWallContactImpulses(circle, inward_normal, radius, material,
                             allow_restitution, restitution_velocity_threshold);

  circle.position = Add(circle.position,
                        Multiply(inward_normal, offset - minimum_projection));
}

}  // namespace

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

float Restitution(float restitution, float normal_velocity,
                  bool allow_restitution, float velocity_threshold) {
  if (!allow_restitution || std::abs(normal_velocity) < velocity_threshold) {
    return 0.0f;
  }
  return std::clamp(restitution, 0.0f, 1.0f);
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

}  // namespace tiny2d::internal
