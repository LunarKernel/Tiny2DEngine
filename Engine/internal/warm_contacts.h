#ifndef TINY2DENGINE_ENGINE_INTERNAL_WARM_CONTACTS_H_
#define TINY2DENGINE_ENGINE_INTERNAL_WARM_CONTACTS_H_

#include <vector>

#include "tiny2d_engine.h"

// Engine-internal warm-starting contact resolution: the accumulated-impulse
// formulation used when a caller passes a ContactCache. Body-body manifolds
// are detected once per step with stable feature ids, cached impulses are
// applied before the first iteration, iterations clamp accumulated totals
// with the wall velocity resolve anchoring every round, positional
// correction passes follow, and accumulated impulses are written back with
// stale entries dropped. Wall contacts are never cached. See the contract
// notes in docs/DEVELOPER_GUIDE.md.

namespace tiny2d::internal {

// Runs the complete warm contact stage (velocity iterations, position
// passes, wall snap, cache write-back and statistics). delta_time must be
// positive; the caller runs the cold zero-dt resolve instead when it is
// not. The cache must already have passed validation.
void SolveWarmContacts(std::vector<Rectangle>& rectangles,
                       std::vector<Circle>& circles, ContactCache& cache,
                       const SolverSettings& settings, float area_width,
                       float area_height, float restitution, float friction,
                       float restitution_velocity_threshold);

// Validates caller-provided solver settings and cache per the public
// contract (iteration range, slop and correction ranges, finite
// non-negative cached normal impulses, strictly increasing keys). Throws
// std::invalid_argument without modifying anything.
void ValidateSolverSettings(const SolverSettings& settings);
void ValidateContactCache(const ContactCache& cache);

}  // namespace tiny2d::internal

#endif  // TINY2DENGINE_ENGINE_INTERNAL_WARM_CONTACTS_H_
