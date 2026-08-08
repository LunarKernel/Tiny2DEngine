#ifndef TINY2DENGINE_SANDBOX_CONTACT_LAB_MODEL_H_
#define TINY2DENGINE_SANDBOX_CONTACT_LAB_MODEL_H_

#include <vector>

#include "tiny2d_engine.h"

namespace tiny2d::sandbox {

inline constexpr float kContactLabPhysicsStep = 1.0f / 480.0f;
inline constexpr float kContactLabAreaWidthM = 24.0f;
inline constexpr float kContactLabAreaHeightM = 14.0f;
inline constexpr float kContactLabRestitutionSpeedThresholdMps = 0.01f;
inline constexpr float kContactLabMinimumMassKg = 0.01f;
inline constexpr float kContactLabMaximumMassKg = 1000.0f;
inline constexpr float kContactLabMinimumRadiusM = 0.1f;
inline constexpr float kContactLabMaximumRadiusM = 2.0f;
inline constexpr float kContactLabMaximumInitialSpeedMps = 10.0f;
inline constexpr float kContactLabMaximumInitialAngularSpeedRadS = 50.0f;
inline constexpr float kContactLabMaximumFrictionCoefficient = 5.0f;
inline constexpr float kContactLabMaximumGravityMps2 = 20.0f;
inline constexpr float kContactLabRollingSlipToleranceMps = 0.02f;

enum class ContactLabMode {
  kElasticImpact,
  kRollingContact,
};

// All values use SI units. +X points right, +Y points down, and positive
// angles and angular velocity appear clockwise. A static circle has zero
// Engine mass but retains mass_kg here so the same configuration can be made
// dynamic without losing its value.
struct ContactLabCircleConfig {
  float mass_kg{1.0f};
  float radius_m{0.5f};
  Vec2 initial_position_m{7.0f, 7.0f};
  Vec2 initial_velocity_m_s{3.0f, 0.0f};
  float initial_angle_degrees{};
  float initial_angular_velocity_rad_s{};
  CircleInertiaModel inertia_model{CircleInertiaModel::kSolidDisk};
  bool is_static{};
  CollisionMaterial material{1.0f, 0.0f, 0.0f};
};

// The impact mode uses circles A and B with gravity disabled. The rolling
// mode uses only circle A, gravity_m_s2, and a fixed 22 m by 1 m horizontal
// rectangle whose top is y=11 m. surface_material belongs to that rectangle.
// Dynamic mass is in [0.01, 1000] kg, radius in [0.1, 2] m, each initial
// velocity component in [-10, 10] m/s, angular velocity in [-50, 50] rad/s,
// gravity in [0, 20] m/s^2, restitution in [0, 1], and
// 0 <= kinetic friction <= static friction <= 5.
struct ContactLabConfig {
  ContactLabMode mode{ContactLabMode::kElasticImpact};
  ContactLabCircleConfig circle_a;
  ContactLabCircleConfig circle_b{
      1.0f,
      0.5f,
      {12.0f, 7.0f},
      {},
      0.0f,
      0.0f,
      CircleInertiaModel::kSolidDisk,
      false,
      {1.0f, 0.0f, 0.0f},
  };
  float gravity_m_s2{};
  CollisionMaterial surface_material{0.0f, 0.6f, 0.4f};
};

struct ContactLabState {
  // Impact: circles[0] is A and circles[1] is B; rectangles is empty.
  // Rolling: circles[0] is A and rectangles[0] is the static surface.
  std::vector<Rectangle> rectangles;
  std::vector<Circle> circles;
  double time_seconds{};
};

struct ContactLabDerived {
  Vec2 total_linear_momentum_kg_m_s;
  double orbital_angular_momentum_kg_m2_s{};
  double spin_angular_momentum_kg_m2_s{};
  double total_angular_momentum_kg_m2_s{};
  double translational_kinetic_energy_j{};
  double rotational_kinetic_energy_j{};
  double total_kinetic_energy_j{};
  // In rolling mode, positive means the circle's contact point slips toward
  // +X. Pure rolling is reached when this approaches zero. It is zero in
  // impact mode.
  float rolling_slip_m_s{};
};

ContactLabConfig MakeElasticImpactConfig();
ContactLabConfig MakeRollingSolidDiskConfig();
ContactLabConfig MakeRollingHoopConfig();

// Returns nullptr when valid, otherwise a stable string literal.
const char* GetContactLabConfigError(const ContactLabConfig& config);
const char* GetContactLabStateError(const ContactLabConfig& config,
                                    const ContactLabState& state);

// Throws std::invalid_argument when config is invalid.
ContactLabState MakeInitialContactLabState(const ContactLabConfig& config);

// Throws std::invalid_argument when config or state is invalid.
ContactLabDerived CalculateContactLabDerived(const ContactLabConfig& config,
                                             const ContactLabState& state);

// Advances one fixed step through the production Engine mixed-shape Update.
// delta_time must be finite and in (0, kContactLabPhysicsStep]. Invalid input
// returns false without changing state; Engine exceptions may propagate, but
// state remains unchanged because integration is performed on a copy.
bool StepContactLab(const ContactLabConfig& config, float delta_time,
                    ContactLabState* state);

// History times must be strictly increasing seconds. Returns the nearest
// sample, clamped to the recorded range. Empty history or non-finite query
// returns nullptr.
const ContactLabState* FindContactLabState(
    const std::vector<ContactLabState>& history, double time_seconds);

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_CONTACT_LAB_MODEL_H_
