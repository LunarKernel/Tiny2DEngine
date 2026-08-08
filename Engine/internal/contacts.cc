#include "internal/contacts.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "internal/body_math.h"

namespace tiny2d::internal {
namespace {

struct Face {
  Vec2 start;
  Vec2 end;
  Vec2 normal;
};

struct ClipPoints {
  std::array<Vec2, 2> points{};
  std::size_t count{};
};

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

}  // namespace

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
  // The guards above give b < 0 and c >= 0, so -b + sqrt(discriminant) never
  // cancels; the textbook (-b - sqrt) / a root loses precision exactly in the
  // grazing configurations this solver exists to resolve.
  const double time = c / (-b + std::sqrt(discriminant));
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

}  // namespace tiny2d::internal
