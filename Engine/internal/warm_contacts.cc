#include "internal/warm_contacts.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

#include "internal/body_math.h"
#include "internal/contacts.h"
#include "internal/solver.h"
#include "internal/validation.h"

namespace tiny2d::internal {
namespace {

constexpr int kWarmPositionPasses = 3;

// Body slot: the kind bit above a 20-bit index (21 bits total); two slots
// plus the 8-bit manifold point id pack into 50 bits of the cache key, so
// both kind bits always survive (a dropped kind bit would collide
// circle-circle with rect-circle pairs and corrupt the cache).
unsigned long long PackSlot(bool is_circle, std::size_t index) {
  return (is_circle ? (1ull << 20) : 0ull) |
         static_cast<unsigned long long>(index);
}

unsigned long long PackKey(unsigned long long slot_a, unsigned long long slot_b,
                           unsigned point_id) {
  return (slot_a << 29) | (slot_b << 8) |
         static_cast<unsigned long long>(point_id);
}

struct WarmPoint {
  unsigned long long key{};
  Vec2 radius_a{};
  Vec2 radius_b{};
  float normal_accumulated{};
  float tangent_accumulated{};
  // Restitution target velocity (>= 0), from the pre-warm approach
  // velocity, threshold-gated like the cold path.
  float velocity_bias{};
};

// One body-body manifold with fixed normal/tangent basis for the step.
// body_kind: 0 = rect-rect, 1 = circle-circle, 2 = rect-circle.
struct WarmManifold {
  int body_kind{};
  std::size_t index_a{};
  std::size_t index_b{};
  ContactManifold manifold{};
  ResolvedMaterial material{};
  Vec2 tangent{};
  std::array<WarmPoint, 2> points{};
};

constexpr unsigned long long kPairMask = ~0xFFull;
constexpr std::size_t kNoEntry = static_cast<std::size_t>(-1);

std::size_t FindEntryIndex(const ContactCache& cache, unsigned long long key) {
  const auto it = std::lower_bound(
      cache.entries.begin(), cache.entries.end(), key,
      [](const ContactCache::Entry& entry, unsigned long long target) {
        return entry.key < target;
      });
  if (it != cache.entries.end() && it->key == key) {
    return static_cast<std::size_t>(it - cache.entries.begin());
  }
  return kNoEntry;
}

// Fallback for clip-id churn: a marginal clip can flip a point's id
// (incident vertex <-> cut edge) or flicker a manifold between one and
// two points, which would cold-restart the contact and sustain a wobble.
// An unmatched point may inherit the lowest-keyed unconsumed entry of the
// SAME body pair: same contact patch, same impulse scale, deterministic.
std::size_t FindPairFallbackIndex(const ContactCache& cache,
                                  const std::vector<bool>& consumed,
                                  unsigned long long key) {
  const unsigned long long pair_prefix = key & kPairMask;
  auto it = std::lower_bound(
      cache.entries.begin(), cache.entries.end(), pair_prefix,
      [](const ContactCache::Entry& entry, unsigned long long target) {
        return entry.key < target;
      });
  for (; it != cache.entries.end() && (it->key & kPairMask) == pair_prefix;
       ++it) {
    const std::size_t index =
        static_cast<std::size_t>(it - cache.entries.begin());
    if (!consumed[index]) {
      return index;
    }
  }
  return kNoEntry;
}

template <typename BodyA, typename BodyB>
void PrepareManifold(WarmManifold& warm, BodyA& body_a, BodyB& body_b,
                     const ContactCache& cache, std::vector<bool>& consumed,
                     float restitution_threshold, int* warm_started_count) {
  warm.tangent = {-warm.manifold.normal.y, warm.manifold.normal.x};

  // Exact key matches first (marking entries consumed), then the
  // pair-level fallback for churned ids.
  std::array<std::size_t, 2> matched{kNoEntry, kNoEntry};
  for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
    matched[i] = FindEntryIndex(cache, warm.points[i].key);
    if (matched[i] != kNoEntry) {
      consumed[matched[i]] = true;
    }
  }
  for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
    if (matched[i] == kNoEntry) {
      matched[i] = FindPairFallbackIndex(cache, consumed, warm.points[i].key);
      if (matched[i] != kNoEntry) {
        consumed[matched[i]] = true;
      }
    }
  }

  // Radii and pre-warm approach velocities for EVERY point are captured
  // before ANY warm impulse is applied, so a two-point manifold's second
  // point cannot see the first point's warm application in its
  // restitution gate.
  for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
    WarmPoint& point = warm.points[i];
    point.radius_a = Subtract(warm.manifold.points[i], body_a.position);
    point.radius_b = Subtract(warm.manifold.points[i], body_b.position);
    const Vec2 relative_velocity =
        Subtract(ContactVelocity(body_b, point.radius_b),
                 ContactVelocity(body_a, point.radius_a));
    const float approach = Dot(relative_velocity, warm.manifold.normal);
    if (approach < 0.0f && std::abs(approach) >= restitution_threshold) {
      point.velocity_bias = -warm.material.restitution * approach;
    }
  }

  for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
    WarmPoint& point = warm.points[i];
    if (matched[i] != kNoEntry) {
      const ContactCache::Entry& entry = cache.entries[matched[i]];
      point.normal_accumulated = entry.normal_impulse;
      point.tangent_accumulated = entry.tangent_impulse;
      const Vec2 impulse =
          Add(Multiply(warm.manifold.normal, point.normal_accumulated),
              Multiply(warm.tangent, point.tangent_accumulated));
      ApplyImpulse(body_a, Multiply(impulse, -1.0f), point.radius_a);
      ApplyImpulse(body_b, impulse, point.radius_b);
      ++*warm_started_count;
    }
  }
}

template <typename BodyA, typename BodyB>
void SolveNormalRound(WarmManifold& warm, BodyA& body_a, BodyB& body_b) {
  const auto solve_point = [&](WarmPoint& point) {
    const Vec2 relative_velocity =
        Subtract(ContactVelocity(body_b, point.radius_b),
                 ContactVelocity(body_a, point.radius_a));
    const float normal_velocity = Dot(relative_velocity, warm.manifold.normal);
    const float denominator =
        ImpulseDenominator(body_a, point.radius_a, warm.manifold.normal) +
        ImpulseDenominator(body_b, point.radius_b, warm.manifold.normal);
    const float delta = (point.velocity_bias - normal_velocity) / denominator;
    const float new_total = std::max(point.normal_accumulated + delta, 0.0f);
    const float applied = new_total - point.normal_accumulated;
    point.normal_accumulated = new_total;
    const Vec2 impulse = Multiply(warm.manifold.normal, applied);
    ApplyImpulse(body_a, Multiply(impulse, -1.0f), point.radius_a);
    ApplyImpulse(body_b, impulse, point.radius_b);
  };

  if (warm.manifold.point_count == 2) {
    // Coupled two-point block solve on accumulated increments, with the
    // same no-artificial-rotation rationale as the cold path; fall back
    // to sequential solves when the block solution violates the clamps.
    WarmPoint& p0 = warm.points[0];
    WarmPoint& p1 = warm.points[1];
    const Vec2 v0 = Subtract(ContactVelocity(body_b, p0.radius_b),
                             ContactVelocity(body_a, p0.radius_a));
    const Vec2 v1 = Subtract(ContactVelocity(body_b, p1.radius_b),
                             ContactVelocity(body_a, p1.radius_a));
    const float rhs0 = p0.velocity_bias - Dot(v0, warm.manifold.normal);
    const float rhs1 = p1.velocity_bias - Dot(v1, warm.manifold.normal);
    const float diagonal0 =
        ImpulseDenominator(body_a, p0.radius_a, warm.manifold.normal) +
        ImpulseDenominator(body_b, p0.radius_b, warm.manifold.normal);
    const float diagonal1 =
        ImpulseDenominator(body_a, p1.radius_a, warm.manifold.normal) +
        ImpulseDenominator(body_b, p1.radius_b, warm.manifold.normal);
    const float cross_a0 = Cross(p0.radius_a, warm.manifold.normal);
    const float cross_a1 = Cross(p1.radius_a, warm.manifold.normal);
    const float cross_b0 = Cross(p0.radius_b, warm.manifold.normal);
    const float cross_b1 = Cross(p1.radius_b, warm.manifold.normal);
    const float coupling = InverseMass(body_a) + InverseMass(body_b) +
                           cross_a0 * cross_a1 * InverseInertia(body_a) +
                           cross_b0 * cross_b1 * InverseInertia(body_b);
    const float determinant = diagonal0 * diagonal1 - coupling * coupling;
    if (determinant > kSupportEpsilon) {
      const float delta0 = (rhs0 * diagonal1 - rhs1 * coupling) / determinant;
      const float delta1 = (rhs1 * diagonal0 - rhs0 * coupling) / determinant;
      const float total0 = p0.normal_accumulated + delta0;
      const float total1 = p1.normal_accumulated + delta1;
      if (total0 >= 0.0f && total1 >= 0.0f) {
        const Vec2 impulse0 = Multiply(warm.manifold.normal, delta0);
        const Vec2 impulse1 = Multiply(warm.manifold.normal, delta1);
        ApplyImpulse(body_a, Multiply(impulse0, -1.0f), p0.radius_a);
        ApplyImpulse(body_b, impulse0, p0.radius_b);
        ApplyImpulse(body_a, Multiply(impulse1, -1.0f), p1.radius_a);
        ApplyImpulse(body_b, impulse1, p1.radius_b);
        p0.normal_accumulated = total0;
        p1.normal_accumulated = total1;
        return;
      }
    }
  }

  for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
    solve_point(warm.points[i]);
  }
}

template <typename BodyA, typename BodyB>
void SolveFrictionRound(WarmManifold& warm, BodyA& body_a, BodyB& body_b) {
  for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
    WarmPoint& point = warm.points[i];
    const Vec2 relative_velocity =
        Subtract(ContactVelocity(body_b, point.radius_b),
                 ContactVelocity(body_a, point.radius_a));
    const float tangent_velocity = Dot(relative_velocity, warm.tangent);
    const float denominator =
        ImpulseDenominator(body_a, point.radius_a, warm.tangent) +
        ImpulseDenominator(body_b, point.radius_b, warm.tangent);
    const float delta = -tangent_velocity / denominator;
    // The sticking cone bounds the accumulated tangent impulse.
    const float bound =
        warm.material.static_friction * point.normal_accumulated;
    const float new_total =
        std::clamp(point.tangent_accumulated + delta, -bound, bound);
    const float applied = new_total - point.tangent_accumulated;
    point.tangent_accumulated = new_total;
    const Vec2 impulse = Multiply(warm.tangent, applied);
    ApplyImpulse(body_a, Multiply(impulse, -1.0f), point.radius_a);
    ApplyImpulse(body_b, impulse, point.radius_b);
  }
}

template <typename BodyA, typename BodyB>
void CorrectPosition(BodyA& body_a, BodyB& body_b,
                     const ContactManifold& manifold,
                     const SolverSettings& settings) {
  const float inverse_mass_a = InverseMass(body_a);
  const float inverse_mass_b = InverseMass(body_b);
  const float inverse_mass_sum = inverse_mass_a + inverse_mass_b;
  if (inverse_mass_sum == 0.0f) {
    return;
  }
  const float correction_magnitude =
      std::max(manifold.penetration - settings.position_slop, 0.0f) *
      settings.position_correction / inverse_mass_sum;
  const Vec2 correction = Multiply(manifold.normal, correction_magnitude);
  body_a.position =
      Subtract(body_a.position, Multiply(correction, inverse_mass_a));
  body_b.position = Add(body_b.position, Multiply(correction, inverse_mass_b));
}

}  // namespace

void ValidateSolverSettings(const SolverSettings& settings) {
  Require(settings.iterations >= 1 && settings.iterations <= 128,
          "Solver iterations must be in [1, 128].");
  Require(
      std::isfinite(settings.position_slop) && settings.position_slop >= 0.0f,
      "Position slop must be finite and non-negative.");
  Require(std::isfinite(settings.position_correction) &&
              settings.position_correction >= 0.0f &&
              settings.position_correction <= 1.0f,
          "Position correction must be in [0, 1].");
}

void ValidateContactCache(const ContactCache& cache) {
  unsigned long long previous_key = 0;
  bool first = true;
  for (const ContactCache::Entry& entry : cache.entries) {
    Require(std::isfinite(entry.normal_impulse) && entry.normal_impulse >= 0.0f,
            "Cached normal impulses must be finite and non-negative.");
    Require(std::isfinite(entry.tangent_impulse),
            "Cached tangent impulses must be finite.");
    Require(first || entry.key > previous_key,
            "Contact cache keys must be strictly increasing.");
    previous_key = entry.key;
    first = false;
  }
}

void SolveWarmContacts(std::vector<Rectangle>& rectangles,
                       std::vector<Circle>& circles, ContactCache& cache,
                       const SolverSettings& settings, float area_width,
                       float area_height, float restitution, float friction,
                       float restitution_velocity_threshold) {
  // 1. Detect body-body manifolds once, in the deterministic pair order,
  //    with the stable-axis preference active for rectangle pairs.
  std::vector<WarmManifold> manifolds;
  int warm_started_count = 0;

  const auto add_manifold =
      [&](int body_kind, std::size_t index_a, std::size_t index_b,
          const ContactManifold& manifold, const ResolvedMaterial& material) {
        WarmManifold warm;
        warm.body_kind = body_kind;
        warm.index_a = index_a;
        warm.index_b = index_b;
        warm.manifold = manifold;
        warm.material = material;
        const unsigned long long slot_a = PackSlot(body_kind == 1, index_a);
        const unsigned long long slot_b = PackSlot(body_kind != 0, index_b);
        for (std::size_t i = 0; i < manifold.point_count; ++i) {
          warm.points[i].key = PackKey(slot_a, slot_b, manifold.point_ids[i]);
        }
        manifolds.push_back(warm);
      };

  // Static-static pairs are skipped like the cold path's early-out: their
  // impulse denominators are zero and no impulse or correction applies.
  for (std::size_t i = 0; i < rectangles.size(); ++i) {
    for (std::size_t j = i + 1; j < rectangles.size(); ++j) {
      if (InverseMass(rectangles[i]) + InverseMass(rectangles[j]) == 0.0f) {
        continue;
      }
      const std::optional<ContactManifold> contact =
          FindContact(rectangles[i], rectangles[j], true);
      if (contact.has_value()) {
        add_manifold(
            0, i, j, *contact,
            MixMaterials(rectangles[i].material, rectangles[j].material,
                         restitution, friction));
      }
    }
  }
  for (std::size_t i = 0; i < circles.size(); ++i) {
    for (std::size_t j = i + 1; j < circles.size(); ++j) {
      if (InverseMass(circles[i]) + InverseMass(circles[j]) == 0.0f) {
        continue;
      }
      const std::optional<ContactManifold> contact =
          FindContact(circles[i], circles[j]);
      if (contact.has_value()) {
        add_manifold(1, i, j, *contact,
                     MixMaterials(circles[i].material, circles[j].material,
                                  restitution, friction));
      }
    }
  }
  for (std::size_t i = 0; i < rectangles.size(); ++i) {
    for (std::size_t j = 0; j < circles.size(); ++j) {
      if (InverseMass(rectangles[i]) + InverseMass(circles[j]) == 0.0f) {
        continue;
      }
      const std::optional<ContactManifold> contact =
          FindContact(rectangles[i], circles[j]);
      if (contact.has_value()) {
        add_manifold(2, i, j, *contact,
                     MixMaterials(rectangles[i].material, circles[j].material,
                                  restitution, friction));
      }
    }
  }

  // 2. Warm apply with pre-warm restitution capture. Consumed flags keep
  //    fallback matching unambiguous and deterministic.
  int contact_count = 0;
  std::vector<bool> consumed(cache.entries.size(), false);
  for (WarmManifold& warm : manifolds) {
    contact_count += static_cast<int>(warm.manifold.point_count);
    switch (warm.body_kind) {
      case 0:
        PrepareManifold(warm, rectangles[warm.index_a],
                        rectangles[warm.index_b], cache, consumed,
                        restitution_velocity_threshold, &warm_started_count);
        break;
      case 1:
        PrepareManifold(warm, circles[warm.index_a], circles[warm.index_b],
                        cache, consumed, restitution_velocity_threshold,
                        &warm_started_count);
        break;
      default:
        PrepareManifold(warm, rectangles[warm.index_a], circles[warm.index_b],
                        cache, consumed, restitution_velocity_threshold,
                        &warm_started_count);
        break;
    }
  }

  // 2b. One wall velocity pass immediately after warm application: the
  //     cache stores only body-body impulses, so warm apply momentarily
  //     drives the wall-adjacent body with the whole column's reaction;
  //     anchoring here restores the cached equilibrium before the first
  //     round instead of spending iterations relaxing the transient.
  for (Rectangle& rectangle : rectangles) {
    ResolveWindowCollisionVelocity(rectangle, area_width, area_height,
                                   restitution, friction, true,
                                   restitution_velocity_threshold);
  }
  for (Circle& circle : circles) {
    ResolveWindowCollisionVelocity(circle, area_width, area_height, restitution,
                                   friction, true,
                                   restitution_velocity_threshold);
  }

  // 3. Velocity iterations: body-body accumulated solves, then the wall
  //    velocity resolve anchoring the chain against the static walls.
  for (int iteration = 0; iteration < settings.iterations; ++iteration) {
    for (WarmManifold& warm : manifolds) {
      switch (warm.body_kind) {
        case 0:
          SolveNormalRound(warm, rectangles[warm.index_a],
                           rectangles[warm.index_b]);
          SolveFrictionRound(warm, rectangles[warm.index_a],
                             rectangles[warm.index_b]);
          break;
        case 1:
          SolveNormalRound(warm, circles[warm.index_a], circles[warm.index_b]);
          SolveFrictionRound(warm, circles[warm.index_a],
                             circles[warm.index_b]);
          break;
        default:
          SolveNormalRound(warm, rectangles[warm.index_a],
                           circles[warm.index_b]);
          SolveFrictionRound(warm, rectangles[warm.index_a],
                             circles[warm.index_b]);
          break;
      }
    }
    for (Rectangle& rectangle : rectangles) {
      ResolveWindowCollisionVelocity(rectangle, area_width, area_height,
                                     restitution, friction, false,
                                     restitution_velocity_threshold);
    }
    for (Circle& circle : circles) {
      ResolveWindowCollisionVelocity(circle, area_width, area_height,
                                     restitution, friction, false,
                                     restitution_velocity_threshold);
    }
  }

  // 4. Positional correction passes over freshly re-detected geometry,
  //    then the wall snap.
  for (int pass = 0; pass < kWarmPositionPasses; ++pass) {
    for (const WarmManifold& warm : manifolds) {
      switch (warm.body_kind) {
        case 0: {
          const std::optional<ContactManifold> contact = FindContact(
              rectangles[warm.index_a], rectangles[warm.index_b], true);
          if (contact.has_value()) {
            CorrectPosition(rectangles[warm.index_a], rectangles[warm.index_b],
                            *contact, settings);
          }
          break;
        }
        case 1: {
          const std::optional<ContactManifold> contact =
              FindContact(circles[warm.index_a], circles[warm.index_b]);
          if (contact.has_value()) {
            CorrectPosition(circles[warm.index_a], circles[warm.index_b],
                            *contact, settings);
          }
          break;
        }
        default: {
          const std::optional<ContactManifold> contact =
              FindContact(rectangles[warm.index_a], circles[warm.index_b]);
          if (contact.has_value()) {
            CorrectPosition(rectangles[warm.index_a], circles[warm.index_b],
                            *contact, settings);
          }
          break;
        }
      }
    }
  }
  for (Rectangle& rectangle : rectangles) {
    ResolveWindowCollisionSnap(rectangle, area_width, area_height);
  }
  for (Circle& circle : circles) {
    ResolveWindowCollisionSnap(circle, area_width, area_height);
  }

  // 5. Write back accumulated impulses under sorted unique keys; stale
  //    entries drop out by omission.
  std::vector<ContactCache::Entry> next_entries;
  next_entries.reserve(static_cast<std::size_t>(contact_count));
  for (const WarmManifold& warm : manifolds) {
    for (std::size_t i = 0; i < warm.manifold.point_count; ++i) {
      next_entries.push_back({warm.points[i].key,
                              warm.points[i].normal_accumulated,
                              warm.points[i].tangent_accumulated});
    }
  }
  std::sort(next_entries.begin(), next_entries.end(),
            [](const ContactCache::Entry& a, const ContactCache::Entry& b) {
              return a.key < b.key;
            });
  cache.entries = std::move(next_entries);
  cache.contact_count = contact_count;
  cache.warm_started_count = warm_started_count;
}

}  // namespace tiny2d::internal
