#ifndef TINY2DENGINE_SANDBOX_INCLINE_SPRING_MODEL_H_
#define TINY2DENGINE_SANDBOX_INCLINE_SPRING_MODEL_H_

#include <array>
#include <vector>

#include "tiny2d_engine.h"

namespace tiny2d::sandbox::incline_spring {

inline constexpr int kAreaWidth = 1200;
inline constexpr int kAreaHeight = 800;
inline constexpr float kUnitLength = 32.0f;
inline constexpr float kSceneTop = 252.0f;
inline constexpr float kPhysicsStep = 1.0f / 120.0f;
inline constexpr float kDefaultFloorLengthMeters = 3.0f;
inline constexpr float kDefaultRampLengthMeters = 3.4641016f;
inline constexpr float kDefaultLowerBodyX =
    static_cast<float>(kAreaWidth) * 0.5f - kUnitLength * 0.5f;

struct BodyConfig {
  float surface_x;
  float mass_kg;
  float downhill_speed;
  float length_units;
  bool starts_on_floor{};
  bool charged{};
  float charge{1.0f};
};

// SI inputs use metres, metres per second, metres per second squared,
// kilograms, seconds, coulombs, and newtons per coulomb. Pixel coordinates
// use +X right and +Y down. Field angles are degrees counter-clockwise from
// +X; the rendered ramp therefore has a negative engine-space angle.
struct SimulationConfig {
  float real_floor_length_m{kDefaultFloorLengthMeters};
  float real_ramp_length_m{kDefaultRampLengthMeters};
  float reference_speed_mps{1.0f};
  float body_a_engine_mass{1.0f};
  float gravity_mps2{9.8f};
  float friction{0.0f};
  float restitution{1.0f};
  bool electric_field_enabled{};
  float electric_field_strength_n_per_c{50.0f};
  float electric_field_angle_degrees{};
  bool ramp_enabled{true};
  float ramp_angle_degrees{30.0f};
  bool spring_enabled{true};
  float spring_stiffness{8.0f};
  BodyConfig body_a{1080.0f, 1.0f, 200.0f, 1.0f, false, false, 1.0f};
  BodyConfig body_b{kDefaultLowerBodyX, 3.0f, 0.0f, 1.0f, true, false, 1.0f};
};

struct BodyTelemetry {
  Vec2 position;
  Vec2 velocity;
  Vec2 acceleration;
  float angle{};
};

struct SimulationSnapshot {
  // Engine simulation seconds. Convert with
  // GetRealSecondsPerSimulationSecond() for display.
  double time{};
  std::array<BodyTelemetry, 2> bodies{};
};

struct State {
  SimulationConfig config;
  // The first two entries are always bodies A and B. A static ramp, when
  // enabled, is the third entry.
  std::vector<Rectangle> bodies;
  // Snapshots are strictly ordered by engine simulation seconds.
  std::vector<SimulationSnapshot> history;
  // Engine simulation seconds. History lookup accepts finite double seconds.
  double time{};
};

float GetRampAngle(const SimulationConfig& config);
float GetPixelsPerMeter(const SimulationConfig& config);
float GetRampBottomX(const SimulationConfig& config);
float GetRampLengthPixels(const SimulationConfig& config);
float GetRampTopX(const SimulationConfig& config);
float GetPixelSpeedPerMeterPerSecond(const SimulationConfig& config);
float GetRealSecondsPerSimulationSecond(const SimulationConfig& config);
float GetEngineMassPerKilogram(const SimulationConfig& config);
float GetBodyEngineMass(const BodyConfig& body_config,
                        const SimulationConfig& simulation_config);
float GetPixelAccelerationPerMeterPerSecondSquared(
    const SimulationConfig& config);
float GetPixelGravity(const SimulationConfig& config);
Vec2 GetElectricField(const SimulationConfig& config);
float GetSpringRestX(const SimulationConfig& config);
float GetSpringAnchorX(const SimulationConfig& config);
float GetSpringEndX(const State& state);
float GetBodyWidth(const BodyConfig& config);
float GetMaximumBodyLengthUnits(const SimulationConfig& config);
float GetMinimumBodySurfaceX(const BodyConfig& body_config,
                             const SimulationConfig& config);
float GetMaximumBodySurfaceX(const BodyConfig& body_config,
                             const SimulationConfig& config);
void ClampBodySurfaceX(BodyConfig& body_config, const SimulationConfig& config);

float GetSignedSpeed(const BodyTelemetry& body, const SimulationConfig& config);
float GetSignedSurfaceAcceleration(const BodyTelemetry& body,
                                   const SimulationConfig& config);
float GetSignedDistance(const BodyTelemetry& body, float body_width,
                        const SimulationConfig& config);

// Returns nullptr when valid; otherwise returns a stable string literal.
const char* GetConfigError(const SimulationConfig& config);

// Reset validates first and leaves state unchanged on failure.
const char* Reset(const SimulationConfig& config, State& state);

// Advances exactly delta_time engine seconds and records one snapshot.
// delta_time must be finite and in (0, kPhysicsStep]. Engine validation throws
// std::invalid_argument to the caller; the interactive simulation reports it
// and returns to its setup screen.
const char* Step(State& state, float delta_time);

// Returns the nearest recorded snapshot, clamped to the recorded time range.
// Returns nullptr for an empty history or a non-finite query.
const SimulationSnapshot* FindSnapshot(const State& state, double time);

}  // namespace tiny2d::sandbox::incline_spring

#endif  // TINY2DENGINE_SANDBOX_INCLINE_SPRING_MODEL_H_
