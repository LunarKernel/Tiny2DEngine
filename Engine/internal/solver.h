#ifndef TINY2DENGINE_ENGINE_INTERNAL_SOLVER_H_
#define TINY2DENGINE_ENGINE_INTERNAL_SOLVER_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "internal/body_math.h"
#include "internal/contacts.h"
#include "tiny2d_engine.h"

// Engine-internal impulse resolution. The templates take any body type that
// exposes mass, position, velocity, and angular state, so rectangle-circle
// pairs resolve through the same code path as same-shape pairs.

namespace tiny2d::internal {

constexpr int kSolverIterations = 4;
constexpr float kPositionSlop = 0.01f;
constexpr float kPositionCorrection = 0.8f;

struct ResolvedMaterial {
  float restitution;
  float static_friction;
  float kinetic_friction;
};

ResolvedMaterial MixMaterials(const CollisionMaterial& material_a,
                              const CollisionMaterial& material_b,
                              float world_restitution, float world_friction);
ResolvedMaterial MixWithWorld(const CollisionMaterial& material,
                              float world_restitution, float world_friction);

float Restitution(float restitution, float normal_velocity,
                  bool allow_restitution, float velocity_threshold);

void ResolveWindowCollision(Rectangle& square, float area_width,
                            float area_height, float restitution,
                            float friction, bool allow_restitution,
                            float restitution_velocity_threshold);
void ResolveWindowCollision(Circle& circle, float area_width, float area_height,
                            float restitution, float friction,
                            bool allow_restitution,
                            float restitution_velocity_threshold);

// Split wall resolves for the warm-start contact path: the velocity half
// runs inside every warm iteration (anchoring stacked chains against the
// static walls), the snap half once at step end. The combined
// ResolveWindowCollision above calls both halves per wall in the original
// order, so the cold path stays bit-identical.
void ResolveWindowCollisionVelocity(Rectangle& square, float area_width,
                                    float area_height, float restitution,
                                    float friction, bool allow_restitution,
                                    float restitution_velocity_threshold);
void ResolveWindowCollisionVelocity(Circle& circle, float area_width,
                                    float area_height, float restitution,
                                    float friction, bool allow_restitution,
                                    float restitution_velocity_threshold);
void ResolveWindowCollisionSnap(Rectangle& square, float area_width,
                                float area_height);
void ResolveWindowCollisionSnap(Circle& circle, float area_width,
                                float area_height);

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
                    float restitution_velocity_threshold,
                    float position_slop = kPositionSlop,
                    float position_correction = kPositionCorrection) {
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
      std::max(manifold.penetration - position_slop, 0.0f) *
      position_correction / inverse_mass_sum;
  const Vec2 correction = Multiply(manifold.normal, correction_magnitude);
  square_a.position =
      Subtract(square_a.position, Multiply(correction, inverse_mass_a));
  square_b.position =
      Add(square_b.position, Multiply(correction, inverse_mass_b));
}

}  // namespace tiny2d::internal

#endif  // TINY2DENGINE_ENGINE_INTERNAL_SOLVER_H_
