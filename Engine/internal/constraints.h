#ifndef TINY2DENGINE_ENGINE_INTERNAL_CONSTRAINTS_H_
#define TINY2DENGINE_ENGINE_INTERNAL_CONSTRAINTS_H_

#include <vector>

#include "tiny2d_engine.h"

// Engine-internal bilateral constraint resolution: revolute pins and
// pulley-rope constraints. Velocity solves run between force integration and
// position advancement; position projection runs after contact resolution.
// Everything here assumes the constraint sets already passed
// ValidateConstraints.

namespace tiny2d::internal {

// Accumulated constraint impulses for one step, converted to forces by the
// public entry point when a reaction report is requested.
struct ConstraintImpulses {
  std::vector<Vec2> pin_impulses;
  struct RopeImpulsePair {
    float impulse_a{};
    float impulse_b{};
  };
  std::vector<RopeImpulsePair> rope_impulses;
  std::vector<float> anchor_rod_impulses;
  std::vector<float> link_rod_impulses;
};

// Throws std::invalid_argument on any violation of the constraint rules
// documented on the public constrained Update. Does not modify anything.
void ValidateConstraints(const std::vector<Rectangle>& rectangles,
                         const std::vector<Circle>& circles,
                         const ConstraintSet& constraints, float area_width,
                         float area_height);

// Removes constraint velocity errors with impulses over kSolverIterations
// rounds (per round: pins, ropes, anchor rods, link rods; independent
// constraints are exact after one round, chains converge geometrically).
// Accumulates the applied impulses into *impulses, whose vectors must
// already be sized to the constraint counts.
void SolveConstraintVelocities(std::vector<Rectangle>& rectangles,
                               std::vector<Circle>& circles,
                               const ConstraintSet& constraints,
                               ConstraintImpulses* impulses);

// Fully removes constraint position errors in the same order: pins snap
// their circle center back to the anchor; ropes remove the
// segment-length-sum error along the current segment directions,
// distributed by inverse mass; rods remove their length error the same
// way (AnchorRod moves only its body). All are full snaps (no correction
// factor, no slop) so drift does not accumulate; on chains the sequential
// single pass leaves a per-step, non-accumulating residual.
void ProjectConstraintPositions(std::vector<Rectangle>& rectangles,
                                std::vector<Circle>& circles,
                                const ConstraintSet& constraints);

}  // namespace tiny2d::internal

#endif  // TINY2DENGINE_ENGINE_INTERNAL_CONSTRAINTS_H_
