#include "internal/constraints.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "internal/body_math.h"
#include "internal/solver.h"
#include "internal/validation.h"

namespace tiny2d::internal {
namespace {

// Below this endpoint separation a constraint axis is numerically
// undefined.
constexpr float kMinimumConstraintLength = 0.000001f;

bool IsKnownBodyKind(BodyKind kind) {
  switch (kind) {
    case BodyKind::kRectangle:
    case BodyKind::kCircle:
      return true;
  }
  return false;
}

bool SameBody(BodyRef a, BodyRef b) {
  return a.kind == b.kind && a.index == b.index;
}

bool IsPinnedCircle(const std::vector<RevolutePin>& revolute_pins, int index) {
  for (const RevolutePin& pin : revolute_pins) {
    if (pin.circle_index == index) {
      return true;
    }
  }
  return false;
}

bool IsPinnedRef(const std::vector<RevolutePin>& revolute_pins, BodyRef ref) {
  return ref.kind == BodyKind::kCircle &&
         IsPinnedCircle(revolute_pins, ref.index);
}

// Mutable center-of-mass view of a constraint end body. Ropes and rods
// attach at the center of mass, so only position, velocity, and inverse
// mass matter.
struct EndView {
  Vec2* position;
  Vec2* velocity;
  float inverse_mass;
};

EndView GetEndView(std::vector<Rectangle>& rectangles,
                   std::vector<Circle>& circles, BodyRef ref) {
  if (ref.kind == BodyKind::kRectangle) {
    Rectangle& body = rectangles[static_cast<std::size_t>(ref.index)];
    return {&body.position, &body.velocity, InverseMass(body)};
  }
  Circle& body = circles[static_cast<std::size_t>(ref.index)];
  return {&body.position, &body.velocity, InverseMass(body)};
}

float EndMass(const std::vector<Rectangle>& rectangles,
              const std::vector<Circle>& circles, BodyRef ref) {
  return ref.kind == BodyKind::kRectangle
             ? rectangles[static_cast<std::size_t>(ref.index)].mass
             : circles[static_cast<std::size_t>(ref.index)].mass;
}

Vec2 EndPosition(const std::vector<Rectangle>& rectangles,
                 const std::vector<Circle>& circles, BodyRef ref) {
  return ref.kind == BodyKind::kRectangle
             ? rectangles[static_cast<std::size_t>(ref.index)].position
             : circles[static_cast<std::size_t>(ref.index)].position;
}

bool InRange(int index, std::size_t size) {
  return index >= 0 && static_cast<std::size_t>(index) < size;
}

bool EndInRange(const std::vector<Rectangle>& rectangles,
                const std::vector<Circle>& circles, BodyRef ref) {
  return ref.kind == BodyKind::kRectangle
             ? InRange(ref.index, rectangles.size())
             : InRange(ref.index, circles.size());
}

double SegmentLength(Vec2 position, Vec2 anchor) {
  const double delta_x =
      static_cast<double>(position.x) - static_cast<double>(anchor.x);
  const double delta_y =
      static_cast<double>(position.y) - static_cast<double>(anchor.y);
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

}  // namespace

void ValidateConstraints(const std::vector<Rectangle>& rectangles,
                         const std::vector<Circle>& circles,
                         const ConstraintSet& constraints, float area_width,
                         float area_height) {
  const std::vector<RevolutePin>& revolute_pins = constraints.revolute_pins;
  for (std::size_t i = 0; i < revolute_pins.size(); ++i) {
    const RevolutePin& pin = revolute_pins[i];
    Require(InRange(pin.circle_index, circles.size()),
            "A revolute pin references a circle index out of range.");
    const Circle& circle = circles[static_cast<std::size_t>(pin.circle_index)];
    Require(circle.mass > 0.0f, "A revolute pin requires a dynamic circle.");
    Require(IsFinite(pin.world_anchor),
            "A revolute pin anchor must contain only finite values.");
    Require(pin.world_anchor.x >= circle.radius &&
                pin.world_anchor.x <= area_width - circle.radius &&
                pin.world_anchor.y >= circle.radius &&
                pin.world_anchor.y <= area_height - circle.radius,
            "A revolute pin anchor must keep its circle inside the area.");
    for (std::size_t j = i + 1; j < revolute_pins.size(); ++j) {
      Require(revolute_pins[j].circle_index != pin.circle_index,
              "A circle may carry at most one revolute pin.");
    }
  }

  for (const PulleyRope& rope : constraints.pulley_ropes) {
    Require(
        IsKnownBodyKind(rope.body_a.kind) && IsKnownBodyKind(rope.body_b.kind),
        "A rope end uses an unknown body kind.");
    Require(EndInRange(rectangles, circles, rope.body_a) &&
                EndInRange(rectangles, circles, rope.body_b),
            "A rope end references a body index out of range.");
    Require(EndMass(rectangles, circles, rope.body_a) > 0.0f &&
                EndMass(rectangles, circles, rope.body_b) > 0.0f,
            "Rope ends must reference dynamic bodies.");
    Require(!SameBody(rope.body_a, rope.body_b),
            "A rope must connect two distinct bodies.");
    const BodyRef pulley_ref{BodyKind::kCircle, rope.pulley_circle_index};
    Require(!SameBody(rope.body_a, pulley_ref) &&
                !SameBody(rope.body_b, pulley_ref),
            "A rope end must not reference the rope's pulley.");
    Require(!IsPinnedRef(revolute_pins, rope.body_a) &&
                !IsPinnedRef(revolute_pins, rope.body_b),
            "A rope end must not reference a pinned circle.");
    Require(InRange(rope.pulley_circle_index, circles.size()),
            "A rope references a pulley circle index out of range.");
    Require(IsPinnedCircle(revolute_pins, rope.pulley_circle_index),
            "A rope pulley must carry a revolute pin.");
    const Circle& pulley =
        circles[static_cast<std::size_t>(rope.pulley_circle_index)];
    Require(!pulley.fixed_rotation,
            "A rope pulley must not use fixed rotation.");
    Require(IsFinite(rope.anchor_a) && IsFinite(rope.anchor_b),
            "Rope anchors must contain only finite values.");
    Require(std::isfinite(rope.segment_length_sum) &&
                rope.segment_length_sum > 0.0f,
            "Rope segment length sum must be finite and positive.");
    const double length_a = SegmentLength(
        EndPosition(rectangles, circles, rope.body_a), rope.anchor_a);
    const double length_b = SegmentLength(
        EndPosition(rectangles, circles, rope.body_b), rope.anchor_b);
    Require(length_a >= kMinimumConstraintLength &&
                length_b >= kMinimumConstraintLength,
            "Rope ends must start away from their anchors.");
  }

  for (const AnchorRod& rod : constraints.anchor_rods) {
    Require(IsKnownBodyKind(rod.body.kind),
            "An anchor rod uses an unknown body kind.");
    Require(EndInRange(rectangles, circles, rod.body),
            "An anchor rod references a body index out of range.");
    Require(EndMass(rectangles, circles, rod.body) > 0.0f,
            "An anchor rod must reference a dynamic body.");
    Require(!IsPinnedRef(revolute_pins, rod.body),
            "An anchor rod must not reference a pinned circle.");
    Require(IsFinite(rod.world_anchor),
            "An anchor rod anchor must contain only finite values.");
    Require(std::isfinite(rod.length) && rod.length > 0.0f,
            "An anchor rod length must be finite and positive.");
    Require(SegmentLength(EndPosition(rectangles, circles, rod.body),
                          rod.world_anchor) >= kMinimumConstraintLength,
            "An anchor rod body must start away from its anchor.");
  }

  for (const LinkRod& rod : constraints.link_rods) {
    Require(
        IsKnownBodyKind(rod.body_a.kind) && IsKnownBodyKind(rod.body_b.kind),
        "A link rod end uses an unknown body kind.");
    Require(EndInRange(rectangles, circles, rod.body_a) &&
                EndInRange(rectangles, circles, rod.body_b),
            "A link rod end references a body index out of range.");
    Require(EndMass(rectangles, circles, rod.body_a) > 0.0f &&
                EndMass(rectangles, circles, rod.body_b) > 0.0f,
            "Link rod ends must reference dynamic bodies.");
    Require(!SameBody(rod.body_a, rod.body_b),
            "A link rod must connect two distinct bodies.");
    Require(!IsPinnedRef(revolute_pins, rod.body_a) &&
                !IsPinnedRef(revolute_pins, rod.body_b),
            "A link rod end must not reference a pinned circle.");
    Require(std::isfinite(rod.length) && rod.length > 0.0f,
            "A link rod length must be finite and positive.");
    Require(SegmentLength(EndPosition(rectangles, circles, rod.body_a),
                          EndPosition(rectangles, circles, rod.body_b)) >=
                kMinimumConstraintLength,
            "Link rod ends must start away from each other.");
  }
}

void SolveConstraintVelocities(std::vector<Rectangle>& rectangles,
                               std::vector<Circle>& circles,
                               const ConstraintSet& constraints,
                               ConstraintImpulses* impulses) {
  for (int round = 0; round < kSolverIterations; ++round) {
    for (std::size_t i = 0; i < constraints.revolute_pins.size(); ++i) {
      Circle& circle = circles[static_cast<std::size_t>(
          constraints.revolute_pins[i].circle_index)];
      const Vec2 impulse = Multiply(circle.velocity, -circle.mass);
      circle.velocity = {};
      impulses->pin_impulses[i] = Add(impulses->pin_impulses[i], impulse);
    }

    for (std::size_t i = 0; i < constraints.pulley_ropes.size(); ++i) {
      const PulleyRope& rope = constraints.pulley_ropes[i];
      Circle& pulley =
          circles[static_cast<std::size_t>(rope.pulley_circle_index)];
      const EndView end_a = GetEndView(rectangles, circles, rope.body_a);
      const EndView end_b = GetEndView(rectangles, circles, rope.body_b);
      const Vec2 direction_a =
          Normalize(Subtract(*end_a.position, rope.anchor_a));
      const Vec2 direction_b =
          Normalize(Subtract(*end_b.position, rope.anchor_b));
      const float radius = pulley.radius;
      const float inverse_inertia = InverseInertia(pulley);
      const float coupling = radius * radius * inverse_inertia;

      // Constraint rates: positive (clockwise) pulley rotation feeds rope
      // toward side b, so side a's segment growth couples with +radius and
      // side b's with -radius.
      const float rate_a =
          Dot(direction_a, *end_a.velocity) + radius * pulley.angular_velocity;
      const float rate_b =
          Dot(direction_b, *end_b.velocity) - radius * pulley.angular_velocity;

      const float k_aa = end_a.inverse_mass + coupling;
      const float k_bb = end_b.inverse_mass + coupling;
      const float k_ab = -coupling;
      const float determinant = k_aa * k_bb - k_ab * k_ab;

      float lambda_a = 0.0f;
      float lambda_b = 0.0f;
      if (determinant > 0.0f) {
        lambda_a = (-rate_a * k_bb + rate_b * k_ab) / determinant;
        lambda_b = (rate_a * k_ab - rate_b * k_aa) / determinant;
      } else {
        // Degenerate block: fall back to independent row solves, mirroring
        // the contact solver's two-point fallback strategy.
        lambda_a = -rate_a / k_aa;
        lambda_b = -rate_b / k_bb;
      }

      *end_a.velocity =
          Add(*end_a.velocity,
              Multiply(direction_a, lambda_a * end_a.inverse_mass));
      *end_b.velocity =
          Add(*end_b.velocity,
              Multiply(direction_b, lambda_b * end_b.inverse_mass));
      pulley.angular_velocity +=
          radius * (lambda_a - lambda_b) * inverse_inertia;

      impulses->rope_impulses[i].impulse_a += lambda_a;
      impulses->rope_impulses[i].impulse_b += lambda_b;
    }

    for (std::size_t i = 0; i < constraints.anchor_rods.size(); ++i) {
      const AnchorRod& rod = constraints.anchor_rods[i];
      const EndView end = GetEndView(rectangles, circles, rod.body);
      const Vec2 axis = Normalize(Subtract(*end.position, rod.world_anchor));
      const float rate = Dot(axis, *end.velocity);
      const float lambda = -rate / end.inverse_mass;
      *end.velocity =
          Add(*end.velocity, Multiply(axis, lambda * end.inverse_mass));
      impulses->anchor_rod_impulses[i] += lambda;
    }

    for (std::size_t i = 0; i < constraints.link_rods.size(); ++i) {
      const LinkRod& rod = constraints.link_rods[i];
      const EndView end_a = GetEndView(rectangles, circles, rod.body_a);
      const EndView end_b = GetEndView(rectangles, circles, rod.body_b);
      const Vec2 axis = Normalize(Subtract(*end_a.position, *end_b.position));
      const float rate = Dot(axis, Subtract(*end_a.velocity, *end_b.velocity));
      const float lambda = -rate / (end_a.inverse_mass + end_b.inverse_mass);
      *end_a.velocity =
          Add(*end_a.velocity, Multiply(axis, lambda * end_a.inverse_mass));
      *end_b.velocity = Subtract(*end_b.velocity,
                                 Multiply(axis, lambda * end_b.inverse_mass));
      impulses->link_rod_impulses[i] += lambda;
    }
  }
}

void ProjectConstraintPositions(std::vector<Rectangle>& rectangles,
                                std::vector<Circle>& circles,
                                const ConstraintSet& constraints) {
  for (const RevolutePin& pin : constraints.revolute_pins) {
    circles[static_cast<std::size_t>(pin.circle_index)].position =
        pin.world_anchor;
  }

  for (const PulleyRope& rope : constraints.pulley_ropes) {
    const EndView end_a = GetEndView(rectangles, circles, rope.body_a);
    const EndView end_b = GetEndView(rectangles, circles, rope.body_b);
    const double error = SegmentLength(*end_a.position, rope.anchor_a) +
                         SegmentLength(*end_b.position, rope.anchor_b) -
                         static_cast<double>(rope.segment_length_sum);
    // Validation guarantees dynamic ends, so the weight denominator is
    // always positive.
    const float inverse_mass_sum = end_a.inverse_mass + end_b.inverse_mass;
    const Vec2 direction_a =
        Normalize(Subtract(*end_a.position, rope.anchor_a));
    const Vec2 direction_b =
        Normalize(Subtract(*end_b.position, rope.anchor_b));
    const float correction_a =
        static_cast<float>(error * (end_a.inverse_mass / inverse_mass_sum));
    const float correction_b =
        static_cast<float>(error * (end_b.inverse_mass / inverse_mass_sum));
    *end_a.position =
        Subtract(*end_a.position, Multiply(direction_a, correction_a));
    *end_b.position =
        Subtract(*end_b.position, Multiply(direction_b, correction_b));
  }

  for (const AnchorRod& rod : constraints.anchor_rods) {
    const EndView end = GetEndView(rectangles, circles, rod.body);
    const double error = SegmentLength(*end.position, rod.world_anchor) -
                         static_cast<double>(rod.length);
    const Vec2 axis = Normalize(Subtract(*end.position, rod.world_anchor));
    *end.position =
        Subtract(*end.position, Multiply(axis, static_cast<float>(error)));
  }

  for (const LinkRod& rod : constraints.link_rods) {
    const EndView end_a = GetEndView(rectangles, circles, rod.body_a);
    const EndView end_b = GetEndView(rectangles, circles, rod.body_b);
    const double error = SegmentLength(*end_a.position, *end_b.position) -
                         static_cast<double>(rod.length);
    const float inverse_mass_sum = end_a.inverse_mass + end_b.inverse_mass;
    const Vec2 axis = Normalize(Subtract(*end_a.position, *end_b.position));
    const float correction_a =
        static_cast<float>(error * (end_a.inverse_mass / inverse_mass_sum));
    const float correction_b =
        static_cast<float>(error * (end_b.inverse_mass / inverse_mass_sum));
    *end_a.position = Subtract(*end_a.position, Multiply(axis, correction_a));
    *end_b.position = Add(*end_b.position, Multiply(axis, correction_b));
  }
}

}  // namespace tiny2d::internal
