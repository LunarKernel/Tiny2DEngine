#ifndef TINY2DENGINE_ENGINE_INTERNAL_CONTACTS_H_
#define TINY2DENGINE_ENGINE_INTERNAL_CONTACTS_H_

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "tiny2d_engine.h"

// Engine-internal collision detection and contact generation. Geometry must
// already satisfy the public validation contract; these functions do not
// validate.

namespace tiny2d::internal {

constexpr float kSupportEpsilon = 0.0001f;

struct Projection {
  float minimum;
  float maximum;
};

struct ContactManifold {
  Vec2 normal;
  std::array<Vec2, 2> points{};
  std::size_t point_count{};
  float penetration;
  // Stable per-point feature ids for warm-start matching: bit 7 encodes
  // which body was the SAT reference, bits 4-6 the reference face index,
  // bits 0-3 the clip id (incident vertex index 0-3, 4 + clip plane for
  // an intersection point, 15 for the zero-clip support fallback).
  // Circle-derived manifolds use id 0. Inert data on the cold path.
  std::array<unsigned, 2> point_ids{};
};

struct SupportFeature {
  std::array<Vec2, 2> points{};
  std::size_t count{};
};

struct CircleImpact {
  double time;
  std::size_t first;
  std::size_t second;
};

Projection Project(const std::array<Vec2, 4>& vertices, Vec2 axis);
SupportFeature FindSupportFeature(const std::array<Vec2, 4>& vertices,
                                  Vec2 axis, bool find_maximum);

// A positive-area overlap yields a manifold; touching shapes yield nullopt.
// prefer_stable_axis applies a 2% relative hysteresis to the SAT axis
// choice so near-tied axes (stacked boxes) cannot flip the reference body
// step to step and defeat warm-start matching; the default keeps the
// historical strict comparison bit for bit.
std::optional<ContactManifold> FindContact(const Rectangle& square_a,
                                           const Rectangle& square_b,
                                           bool prefer_stable_axis = false);
std::optional<ContactManifold> FindContact(const Circle& circle_a,
                                           const Circle& circle_b);
std::optional<ContactManifold> FindContact(const Rectangle& rectangle,
                                           const Circle& circle);

// Earliest forward time in [0, maximum_time] at which the two swept circles
// first touch, or nullopt for pairs that are static, already overlapping,
// separating, or missing each other.
std::optional<double> FindCircleTimeOfImpact(const Circle& circle_a,
                                             const Circle& circle_b,
                                             double maximum_time);

// True when a pair separated at the start and end of the step still crosses
// within it, using the post-integration velocities over the start positions
// (the segment semi-implicit Euler actually traverses).
bool HasMissedCircleImpact(const std::vector<Circle>& starts,
                           const std::vector<Circle>& integrated,
                           float delta_time);

std::optional<CircleImpact> FindEarliestCircleImpact(
    const std::vector<Circle>& circles, float maximum_time);

// Contact for two circles positioned at their time of impact; penetration is
// clamped to zero for exactly-touching placements.
ContactManifold MakeCircleImpactContact(const Circle& circle_a,
                                        const Circle& circle_b);

}  // namespace tiny2d::internal

#endif  // TINY2DENGINE_ENGINE_INTERNAL_CONTACTS_H_
