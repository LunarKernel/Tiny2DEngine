#ifndef TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
#define TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_

#include <array>
#include <vector>

namespace tiny2d {

// Engine values use caller-selected, internally consistent units. The Engine
// does not perform SI or pixel conversion. +X points right and +Y points down.
// Angles are in radians; a positive angle or angular velocity therefore
// appears clockwise in screen coordinates.
struct Vec2 {
  float x{};
  float y{};
};

// A value of {-1, -1, -1} inherits the restitution and friction passed to
// Update. Otherwise all values must be specified: restitution is in [0, 1]
// and 0 <= kinetic_friction <= static_friction. Contact restitution uses the
// larger body value; both friction values use their geometric mean.
struct CollisionMaterial {
  float restitution{-1.0f};
  float static_friction{-1.0f};
  float kinetic_friction{-1.0f};
};

enum class CircleInertiaModel {
  kSolidDisk,
  kHoop,
};

struct Rectangle {
  // Mass is in caller-selected mass units. Zero creates a static body; a
  // dynamic body must have mass >= 1e-6.
  float mass{1.0f};
  // Center position and linear velocity use length and length/time units.
  Vec2 position{};
  Vec2 velocity{};
  // Angle and angular velocity use radians and radians/time.
  float angle{};
  float angular_velocity{};
  // Width and height must be positive and use the position length unit.
  float width{40.0f};
  float height{40.0f};
  // Update clears angular velocity and does not integrate angle when true.
  bool fixed_rotation{};
  // Charge uses the unit paired with electric_field. Any finite sign is valid.
  float charge{};
  // Force and torque accumulated for the next successful Update call. Force
  // uses mass*length/time^2; torque uses mass*length^2/time^2. AddForceAtPoint
  // and AddTorque accumulate these values. Update consumes and clears them,
  // including when delta_time is zero.
  Vec2 applied_force{};
  float applied_torque{};
  // Non-negative exponential damping rates in 1/time. A zero rate disables
  // that damping. The angular default preserves the legacy Update behavior.
  float linear_damping_rate{};
  float angular_damping_rate{0.7f};
  // Kept last so existing aggregate initializers retain their field mapping.
  CollisionMaterial material{};
};

// A circular rigid body with the same caller-selected units, +Y-down axes,
// clockwise-positive angles, load lifetime, and damping semantics as
// Rectangle. Radius must be positive. kSolidDisk uses I=mr^2/2; kHoop uses
// I=mr^2. Public operations reject non-finite or physically invalid state with
// std::invalid_argument before modifying it.
struct Circle {
  float mass{1.0f};
  Vec2 position{};
  Vec2 velocity{};
  float angle{};
  float angular_velocity{};
  float radius{20.0f};
  CircleInertiaModel inertia_model{CircleInertiaModel::kSolidDisk};
  bool fixed_rotation{};
  float charge{};
  Vec2 applied_force{};
  float applied_torque{};
  float linear_damping_rate{};
  float angular_damping_rate{0.7f};
  CollisionMaterial material{};
};

// Returns m * (width^2 + height^2) / 12 in mass*length^2 units for a uniform
// rectangle about its center of mass. A static rectangle returns zero;
// fixed_rotation does not change the physical inertia returned. Validates the
// complete Rectangle state, throws std::invalid_argument on invalid input or
// a result outside float range, and does not modify rectangle.
float GetMomentOfInertia(const Rectangle& rectangle);
// Returns mr^2/2 for kSolidDisk and mr^2 for kHoop in mass*length^2 units.
// Complete Circle validation and exception behavior match the rectangle API.
float GetMomentOfInertia(const Circle& circle);

// Accumulates force for the next successful Update and, for an unlocked
// dynamic rectangle, accumulates torque (world_point - position) x force.
// world_point uses world length units. In the +Y-down coordinate system,
// positive torque produces clockwise angular acceleration. Static rectangles
// ignore the call; fixed-rotation rectangles accumulate only the linear force.
// All inputs and accumulated results must be finite and fit in float. Invalid
// input throws std::invalid_argument without modifying rectangle.
void AddForceAtPoint(Rectangle& rectangle, Vec2 force, Vec2 world_point);
// Circle overload with the same world-space force, clockwise-positive torque,
// one-step accumulation, validation, and failure-atomic semantics.
void AddForceAtPoint(Circle& circle, Vec2 force, Vec2 world_point);

// Accumulates torque in mass*length^2/time^2 for the next successful Update.
// Static and fixed-rotation rectangles ignore the call. The input and
// accumulated result must be finite and fit in float. Invalid input throws
// std::invalid_argument without modifying rectangle.
void AddTorque(Rectangle& rectangle, float torque);
// Circle overload; static and fixed-rotation circles ignore finite torque.
// Invalid input throws std::invalid_argument without modifying the circle.
void AddTorque(Circle& circle, float torque);

// Returns (charge / mass) * electric_field + {0, gravity}. A static rectangle
// returns zero acceleration. electric_field is force/charge, gravity is
// length/time^2, and the result is length/time^2 in the caller's unit system.
// Validates the complete Rectangle state and finite field inputs. Throws
// std::invalid_argument for invalid input or a result outside float range.
// This function does not modify rectangle.
Vec2 GetLinearAcceleration(const Rectangle& rectangle, Vec2 electric_field,
                           float gravity = 98.1f);
// Circle overload using the same force/charge and length/time^2 units, complete
// validation, exception type, and non-mutating behavior as the rectangle API.
Vec2 GetLinearAcceleration(const Circle& circle, Vec2 electric_field,
                           float gravity = 98.1f);

// Returns world-space corners. At zero angle, their order is top-left,
// top-right, bottom-right, and bottom-left. Only geometry is validated:
// position and angle must be finite, and width and height must be finite,
// positive, and safe for float calculations. Throws std::invalid_argument on
// invalid geometry and does not modify rectangle.
std::array<Vec2, 4> GetVertices(const Rectangle& rectangle);

// Returns true only for a positive-area overlap found by SAT; bodies that
// merely touch are not colliding. Only the geometry requirements documented
// by GetVertices apply. Throws std::invalid_argument on invalid geometry and
// does not modify either rectangle.
bool IsColliding(const Rectangle& rectangle_a, const Rectangle& rectangle_b);

// Circle contacts require positive-area overlap; touching shapes are not
// colliding. Geometry is validated without modifying either shape.
bool IsColliding(const Circle& circle_a, const Circle& circle_b);
bool IsColliding(const Rectangle& rectangle, const Circle& circle);
bool IsColliding(const Circle& circle, const Rectangle& rectangle);

// Advances every dynamic rectangle using semi-implicit Euler integration,
// resolves rectangle and window contacts, and applies each body's exponential
// damping. The simulation area is [0, area_width] x [0, area_height].
//
// delta_time is finite time in [0, +infinity); zero skips integration but
// still resolves existing contacts. area_width and area_height are finite,
// positive lengths. Every dynamic rectangle's rotated bounds must be able to
// fit inside the area. restitution is finite in [0, 1], and friction is a
// finite non-negative Coulomb coefficient. electric_field and gravity use the
// units documented by GetLinearAcceleration; signed finite gravity is valid.
// Per-step force and torque are integrated before damping, position, angle,
// and contact resolution. A successful call clears every accumulated force
// and torque, including when delta_time is zero.
//
// Invalid input and predictable integration overflow throw
// std::invalid_argument. All such checks complete before rectangle state is
// modified, so these failure paths leave the input vector unchanged.
void Update(std::vector<Rectangle>& rectangles, float delta_time,
            float area_width, float area_height, float restitution,
            float friction = 0.4f, Vec2 electric_field = {},
            float gravity = 98.1f);

// Advances a mixed rectangle/circle world. The final threshold is a speed in
// the caller's length/time units; restitution is suppressed below it. When
// enable_circle_circle_ccd is true, circle-circle sweeps that begin and end
// separated are resolved at their earliest time of impact. Rectangle and
// window contacts remain discrete. The default false value and enabled calls
// without a missed circle sweep retain the existing discrete result. Existing
// rectangle-only callers retain the legacy threshold of 20. Invalid input and
// predictable integration overflow leave both vectors unchanged. Successful
// calls consume all accumulated loads, including when delta_time is zero.
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            float delta_time, float area_width, float area_height,
            float restitution, float friction = 0.4f, Vec2 electric_field = {},
            float gravity = 98.1f, float restitution_velocity_threshold = 20.0f,
            bool enable_circle_circle_ccd = false);

// Identifies one body inside the vectors passed to the constrained Update:
// kind selects the vector and index is the zero-based position within it.
enum class BodyKind {
  kRectangle,
  kCircle,
};

struct BodyRef {
  BodyKind kind{BodyKind::kRectangle};
  int index{};
};

// Pins a dynamic circle's center of mass to a fixed world point while its
// rotation stays free. world_anchor uses world length units; the anchor disk
// (world_anchor extended by the circle's radius) must fit inside the
// simulation area so the post-snap state can never fail the next call's
// area validation. Each circle may carry at most one pin. fixed_rotation is
// allowed on a plainly pinned circle.
struct RevolutePin {
  int circle_index{};
  Vec2 world_anchor{};
};

// A massless, inextensible, non-slipping ideal rope:
//   body_a COM -- anchor_a -- (arc over the pinned pulley) -- anchor_b --
//   body_b COM.
// anchor_a and anchor_b are the fixed world points where each straight
// segment leaves the pulley (for a hanging configuration, the pulley's
// horizontal tangent points). segment_length_sum is the constrained value of
// |p_a - anchor_a| + |p_b - anchor_b| in length units; the constant wrap arc
// is excluded. No slip couples the rope to the pulley's angular velocity
// through its radius: with bodies hanging below their anchors, positive
// (clockwise) pulley rotation lowers body_b and raises body_a. The rope is
// bilateral (it can push as well as pull); slack is not modeled. Rope ends
// attach at each body's center of mass, must be dynamic, must be distinct
// from each other and from the pulley, and must not themselves be pinned.
// The pulley circle must carry a RevolutePin and must not use
// fixed_rotation. The engine treats world anchors and segment_length_sum as
// targets: values inconsistent with the body positions produce a one-step
// projection toward the targets, not an error.
struct PulleyRope {
  BodyRef body_a{};
  BodyRef body_b{};
  int pulley_circle_index{};
  Vec2 anchor_a{};
  Vec2 anchor_b{};
  float segment_length_sum{};
};

// Rigid massless rod holding a dynamic body's center of mass at a fixed
// distance from a world anchor. Unlike RevolutePin, the body stays free to
// swing: only the radial degree of freedom is removed. The rod is bilateral
// (it carries tension and compression). world_anchor uses world length
// units; length must be positive. The length is a target: a value
// inconsistent with the initial body position produces a one-step
// projection toward it, not an error.
struct AnchorRod {
  BodyRef body{};
  Vec2 world_anchor{};
  float length{};
};

// Rigid massless rod holding two dynamic bodies' centers of mass at a
// fixed distance. Bilateral and COM-attached like AnchorRod, with the same
// target semantics for length. The bodies must be distinct and neither may
// be a pinned circle.
struct LinkRod {
  BodyRef body_a{};
  BodyRef body_b{};
  float length{};
};

// All constraint kinds for one step. The (pins, ropes) Update overload
// below forwards here with empty rod vectors, so there stays exactly one
// step path; TestConstraintSetOverloadMatchesPinsRopesOverload keeps the
// wrapper bitwise.
struct ConstraintSet {
  std::vector<RevolutePin> revolute_pins;
  std::vector<PulleyRope> pulley_ropes;
  std::vector<AnchorRod> anchor_rods;
  std::vector<LinkRod> link_rods;
};

// Per-rope constraint force report in force units. Positive tension means a
// taut rope pulling that body toward its anchor. Both values are zero when
// delta_time is zero.
struct RopeReaction {
  float tension_a{};
  float tension_b{};
};

// Tunable contact-solver parameters. The defaults reproduce the
// historical hardcoded constants bit for bit, so existing callers keep
// their exact trajectories (golden-guarded). position_slop and
// position_correction affect body-body contacts only; window/wall
// contacts always resolve with their full positional snap.
struct SolverSettings {
  // Contact-resolution rounds per step, in [1, 128].
  int iterations{4};
  // Penetration depth tolerated before positional correction, >= 0
  // length units.
  float position_slop{0.01f};
  // Fraction of excess penetration removed per correction, in [0, 1].
  float position_correction{0.8f};
};

// Caller-owned persistent contact store enabling warm starting: passing
// the same cache to consecutive Update calls on the same world carries
// accumulated contact impulses across steps, which is what lets resting
// stacks truly rest. Entries are engine-owned data; callers construct,
// keep, and clear the cache but never edit entries. CALLERS MUST CLEAR
// the cache whenever bodies are inserted, removed, or reordered; keys
// pack body indices and would silently retarget otherwise. A zero
// delta_time call leaves the cache untouched.
struct ContactCache {
  struct Entry {
    // Identifies one body-body contact point across steps: the two body
    // slots (kind + index packed) and a feature id (reference face plus
    // clip-point id for rectangle pairs, zero for circle contacts).
    // Wall contacts are never cached. Entries stay sorted by key.
    unsigned long long key{};
    float normal_impulse{};
    // Signed scalar along the manifold's fixed tangent basis
    // (perp(normal) = {-normal.y, normal.x}).
    float tangent_impulse{};
  };
  std::vector<Entry> entries;
  // Statistics from the last nonzero step: contact points seen and how
  // many warm-started from a cache hit.
  int contact_count{};
  int warm_started_count{};
};

// pin_forces[i] is the force the pin exerts on its circle (for a pulley
// hanging in gravity, approximately its weight pointing up, negative y). It
// reports only the pin impulse: rope anchors are fixed world points, so the
// rope wrap load never couples to the pulley's linear degree of freedom and
// is not included; a physical axle load is assembled by the caller as
// pulley weight plus both tensions. anchor_rod_forces and link_rod_forces
// hold each rod's signed axial force: positive is tension (the rod pulls
// its endpoints toward each other or toward the anchor), negative is
// compression; a hanging pendulum bob reports approximately its weight as
// positive anchor-rod force. Every reported value is zero when delta_time
// is zero.
struct ConstraintReactions {
  std::vector<Vec2> pin_forces;
  std::vector<RopeReaction> rope_tensions;
  std::vector<float> anchor_rod_forces;
  std::vector<float> link_rod_forces;
};

// Advances the mixed world under pin and rope constraints. Forwards to the
// ConstraintSet overload below with empty rod vectors, preserving the V18
// behavior bit for bit (TestConstraintSetOverloadMatchesPinsRopesOverload).
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            const std::vector<RevolutePin>& revolute_pins,
            const std::vector<PulleyRope>& pulley_ropes, float delta_time,
            float area_width, float area_height, float restitution,
            float friction = 0.4f, Vec2 electric_field = {},
            float gravity = 98.1f, float restitution_velocity_threshold = 20.0f,
            bool enable_circle_circle_ccd = false,
            ConstraintReactions* reactions = nullptr);

// Advances the mixed world under the full constraint set. This is the
// single step path: the mixed overload forwards here with an empty set and
// the (pins, ropes) overload with empty rod vectors. Constraint velocity
// errors are removed by impulses after force integration and damping but
// before positions advance (per round: pins, ropes, anchor rods, link
// rods); contact resolution runs unchanged; constraint position errors are
// then fully projected out in the same order, so length drift does not
// accumulate. A contact impulse applied after the constraint solve can
// violate a constraint velocity within one step; the next step's solve
// removes it. Sequential solves are exact for independent constraints and
// converge geometrically for chains; extreme mass ratios across a shared
// body converge slowly and are a documented limitation.
//
// Validation extends the mixed overload's rules. Pins: indices in range,
// dynamic circles, unique per circle, finite anchors that keep the circle
// inside the area. Ropes: distinct dynamic unpinned ends, a pinned and
// rotationally free pulley, finite anchors, positive finite
// segment_length_sum, and both ends at least 1e-6 length units from their
// anchors. Rods: body refs of a known kind, in range, dynamic, and not
// pinned circles; LinkRod ends distinct; finite anchors; finite positive
// lengths; and at least 1e-6 length units between the endpoints so the
// axis is well defined. Rods may share bodies with each other and with
// rope ends (chains). enable_circle_circle_ccd must be false whenever any
// constraint is present. Invalid input throws std::invalid_argument before
// any state (including *reactions) is modified. When reactions is
// non-null, a successful call resizes its vectors to match the constraint
// counts and fills them; a zero delta_time reports zero reactions while
// still resolving constraint velocities and positions.
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            const ConstraintSet& constraints, float delta_time,
            float area_width, float area_height, float restitution,
            float friction = 0.4f, Vec2 electric_field = {},
            float gravity = 98.1f, float restitution_velocity_threshold = 20.0f,
            bool enable_circle_circle_ccd = false,
            ConstraintReactions* reactions = nullptr);

// Full-control overload: constraints plus tunable solver settings plus an
// optional warm-starting contact cache. The ConstraintSet overload above
// forwards here with default settings and no cache, so the single step
// path is preserved (wrapper-consistency and golden tests keep it
// bitwise).
//
// With contact_cache == nullptr the existing per-iteration contact solve
// runs unchanged; SolverSettings only substitutes the formerly hardcoded
// iteration count, position slop, and correction factor, whose defaults
// equal those constants. With a non-null cache, body-body contacts switch
// to an accumulated-impulse solve: manifolds are detected once per step
// with stable feature ids, matched cache impulses are applied before the
// first iteration, iterations clamp accumulated totals (normal >= 0,
// |tangent| <= friction * normal on a fixed per-manifold tangent basis),
// the wall velocity resolve anchors every iteration, three positional
// correction passes plus the wall snap follow, and accumulated impulses
// are written back with stale entries dropped. Restitution captures each
// point's approach velocity before warm application every step, gated by
// the same velocity threshold as the cold path.
//
// Validation extends the constrained overload's rules: iterations in
// [1, 128]; position_slop finite and non-negative; position_correction
// finite in [0, 1]; cache entries must hold finite impulses with
// normal_impulse >= 0 and strictly increasing keys;
// enable_circle_circle_ccd must be false when a cache is passed. Invalid
// input throws std::invalid_argument before any state (bodies,
// *reactions, or *contact_cache) is modified.
void Update(std::vector<Rectangle>& rectangles, std::vector<Circle>& circles,
            const ConstraintSet& constraints,
            const SolverSettings& solver_settings, ContactCache* contact_cache,
            float delta_time, float area_width, float area_height,
            float restitution, float friction = 0.4f, Vec2 electric_field = {},
            float gravity = 98.1f, float restitution_velocity_threshold = 20.0f,
            bool enable_circle_circle_ccd = false,
            ConstraintReactions* reactions = nullptr);

}  // namespace tiny2d

#endif  // TINY2DENGINE_ENGINE_TINY2D_ENGINE_H_
