#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <vector>

#include "simulations.h"

namespace {

constexpr float kPendulumPi = 3.14159265358979323846f;
constexpr float kPendulumDegreesToRadians = kPendulumPi / 180.0f;
constexpr float kPendulumRadiansToDegrees = 180.0f / kPendulumPi;
constexpr float kPendulumPhysicsStep = 1.0f / 240.0f;
constexpr float kPendulumMaximumAnglePerStep = 0.25f;
constexpr double kPendulumMaximumFrameTime = 0.25;

constexpr float kMinimumRodLength = 0.1f;
constexpr float kMaximumRodLength = 20.0f;
constexpr float kMinimumMass = 0.01f;
constexpr float kMaximumMass = 1000.0f;
constexpr float kMaximumChargeMagnitude = 1000.0f;
constexpr float kMaximumElectricField = 1000000.0f;
constexpr float kMaximumDamping = 1000.0f;
constexpr float kMaximumInitialAngularSpeed = 50.0f;

struct PendulumConfig {
  float rod_length_m{2.0f};
  float rod_mass_kg{2.0f};
  float counterweight_mass_kg{1.0f};
  float counterweight_distance_m{1.5f};
  float counterweight_charge_c{1.0f};
  float initial_angle_degrees{35.0f};
  float initial_angular_velocity_rad_s{};
  float gravity_m_s2{9.8f};
  bool electric_field_enabled{true};
  bool counterweight_charged{true};
  float electric_field_strength_n_c{5.0f};
  float electric_field_angle_degrees{};
  bool damping_enabled{};
  float damping_coefficient_n_m_s{0.1f};
};

struct PendulumState {
  float angle_radians{};
  float angular_velocity_rad_s{};
  float angular_acceleration_rad_s2{};
  float time_seconds{};
};

struct PendulumDerived {
  float moment_of_inertia_kg_m2{};
  float gravity_torque_n_m{};
  float electric_torque_n_m{};
  float damping_torque_n_m{};
  float total_torque_n_m{};
  float angular_acceleration_rad_s2{};
  float kinetic_energy_j{};
  float gravitational_potential_energy_j{};
  float electric_potential_energy_j{};
  float total_energy_j{};
};

enum class PendulumSetupAction {
  kNone,
  kStart,
  kBack,
};

float GetPendulumFieldAngle(const PendulumConfig& config) {
  return config.electric_field_angle_degrees * kPendulumDegreesToRadians;
}

float GetPendulumMomentOfInertia(const PendulumConfig& config) {
  return config.rod_mass_kg * config.rod_length_m * config.rod_length_m / 3.0f +
         config.counterweight_mass_kg * config.counterweight_distance_m *
             config.counterweight_distance_m;
}

float GetPendulumGravityTorqueScale(const PendulumConfig& config) {
  return config.gravity_m_s2 *
         (config.rod_mass_kg * config.rod_length_m * 0.5f +
          config.counterweight_mass_kg * config.counterweight_distance_m);
}

float GetPendulumElectricTorqueScale(const PendulumConfig& config) {
  if (!config.electric_field_enabled || !config.counterweight_charged) {
    return 0.0f;
  }
  return config.counterweight_charge_c * config.counterweight_distance_m *
         config.electric_field_strength_n_c;
}

float GetPendulumRestoringTorqueMagnitude(const PendulumConfig& config) {
  const float gravity_scale = GetPendulumGravityTorqueScale(config);
  const float electric_scale = GetPendulumElectricTorqueScale(config);
  const float field_angle = GetPendulumFieldAngle(config);
  return std::hypot(-gravity_scale + electric_scale * std::sin(field_angle),
                    electric_scale * std::cos(field_angle));
}

float GetSmallAnglePeriod(const PendulumConfig& config) {
  const float restoring_torque = GetPendulumRestoringTorqueMagnitude(config);
  const float moment_of_inertia = GetPendulumMomentOfInertia(config);
  if (!std::isfinite(restoring_torque) || restoring_torque <= 0.0f ||
      !std::isfinite(moment_of_inertia) || moment_of_inertia <= 0.0f) {
    return std::numeric_limits<float>::infinity();
  }
  return 2.0f * kPendulumPi * std::sqrt(moment_of_inertia / restoring_torque);
}

PendulumDerived CalculatePendulumDerived(const PendulumConfig& config,
                                         const PendulumState& state) {
  PendulumDerived derived;
  derived.moment_of_inertia_kg_m2 = GetPendulumMomentOfInertia(config);

  const float gravity_scale = GetPendulumGravityTorqueScale(config);
  const float electric_scale = GetPendulumElectricTorqueScale(config);
  const float field_angle = GetPendulumFieldAngle(config);
  derived.gravity_torque_n_m = -gravity_scale * std::sin(state.angle_radians);
  derived.electric_torque_n_m =
      electric_scale * std::cos(state.angle_radians - field_angle);
  derived.damping_torque_n_m =
      config.damping_enabled
          ? -config.damping_coefficient_n_m_s * state.angular_velocity_rad_s
          : 0.0f;
  derived.total_torque_n_m = derived.gravity_torque_n_m +
                             derived.electric_torque_n_m +
                             derived.damping_torque_n_m;
  derived.angular_acceleration_rad_s2 =
      derived.total_torque_n_m / derived.moment_of_inertia_kg_m2;

  derived.kinetic_energy_j = 0.5f * derived.moment_of_inertia_kg_m2 *
                             state.angular_velocity_rad_s *
                             state.angular_velocity_rad_s;
  derived.gravitational_potential_energy_j =
      gravity_scale * (1.0f - std::cos(state.angle_radians));
  derived.electric_potential_energy_j =
      -electric_scale * std::sin(state.angle_radians - field_angle);
  derived.total_energy_j = derived.kinetic_energy_j +
                           derived.gravitational_potential_energy_j +
                           derived.electric_potential_energy_j;
  return derived;
}

bool IsPendulumDerivedFinite(const PendulumDerived& derived) {
  const std::array<float, 10> values = {
      derived.moment_of_inertia_kg_m2,
      derived.gravity_torque_n_m,
      derived.electric_torque_n_m,
      derived.damping_torque_n_m,
      derived.total_torque_n_m,
      derived.angular_acceleration_rad_s2,
      derived.kinetic_energy_j,
      derived.gravitational_potential_energy_j,
      derived.electric_potential_energy_j,
      derived.total_energy_j,
  };
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

PendulumState MakeInitialPendulumState(const PendulumConfig& config) {
  PendulumState state;
  state.angle_radians =
      config.initial_angle_degrees * kPendulumDegreesToRadians;
  state.angular_velocity_rad_s = config.initial_angular_velocity_rad_s;
  state.angular_acceleration_rad_s2 =
      CalculatePendulumDerived(config, state).angular_acceleration_rad_s2;
  return state;
}

const PendulumState* FindPendulumState(
    const std::vector<PendulumState>& history, float time_seconds) {
  if (history.empty() || !std::isfinite(time_seconds)) {
    return nullptr;
  }
  const auto next =
      std::lower_bound(history.begin(), history.end(), time_seconds,
                       [](const PendulumState& state, float target_time) {
                         return state.time_seconds < target_time;
                       });
  if (next == history.begin()) {
    return &history.front();
  }
  if (next == history.end()) {
    return &history.back();
  }
  const auto previous = next - 1;
  return time_seconds - previous->time_seconds <=
                 next->time_seconds - time_seconds
             ? &*previous
             : &*next;
}

const char* GetPendulumStateError(const PendulumConfig& config,
                                  const PendulumState& state) {
  const std::array<float, 4> values = {
      state.angle_radians,
      state.angular_velocity_rad_s,
      state.angular_acceleration_rad_s2,
      state.time_seconds,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "The pendulum state contains NaN or infinity.";
  }
  if (state.time_seconds < 0.0f) {
    return "The pendulum time cannot be negative.";
  }
  if (std::abs(state.angular_velocity_rad_s) * kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep ||
      std::abs(state.angular_acceleration_rad_s2) * kPendulumPhysicsStep *
              kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep) {
    return "The pendulum is moving too fast for the fixed physics step.";
  }
  if (!IsPendulumDerivedFinite(CalculatePendulumDerived(config, state))) {
    return "A derived pendulum value is NaN or infinite.";
  }
  return nullptr;
}

const char* GetPendulumConfigError(const PendulumConfig& config) {
  const std::array<float, 11> values = {
      config.rod_length_m,
      config.rod_mass_kg,
      config.counterweight_mass_kg,
      config.counterweight_distance_m,
      config.counterweight_charge_c,
      config.initial_angle_degrees,
      config.initial_angular_velocity_rad_s,
      config.gravity_m_s2,
      config.electric_field_strength_n_c,
      config.electric_field_angle_degrees,
      config.damping_coefficient_n_m_s,
  };
  if (!std::all_of(values.begin(), values.end(),
                   [](float value) { return std::isfinite(value); })) {
    return "All inputs must be finite numbers; NaN and infinity are invalid.";
  }
  if (config.rod_length_m < kMinimumRodLength ||
      config.rod_length_m > kMaximumRodLength) {
    return "Rod length must be between 0.1 m and 20 m.";
  }
  if (config.rod_mass_kg < kMinimumMass || config.rod_mass_kg > kMaximumMass ||
      config.counterweight_mass_kg < kMinimumMass ||
      config.counterweight_mass_kg > kMaximumMass) {
    return "Rod and counterweight masses must be positive and at most 1000 kg.";
  }
  if (config.counterweight_distance_m < 0.0f ||
      config.counterweight_distance_m > config.rod_length_m) {
    return "Counterweight distance r must be between the pivot and rod end.";
  }
  if (std::abs(config.counterweight_charge_c) > kMaximumChargeMagnitude) {
    return "Counterweight charge must be between -1000 C and 1000 C.";
  }
  if (config.initial_angle_degrees < -180.0f ||
      config.initial_angle_degrees > 180.0f ||
      std::abs(config.initial_angular_velocity_rad_s) >
          kMaximumInitialAngularSpeed) {
    return "Initial angle or angular velocity is outside the supported range.";
  }
  if (config.gravity_m_s2 != 9.8f && config.gravity_m_s2 != 10.0f) {
    return "Gravity must be either 9.8 or 10 m/s^2.";
  }
  if (config.electric_field_strength_n_c < 0.0f ||
      config.electric_field_strength_n_c > kMaximumElectricField ||
      config.electric_field_angle_degrees < -180.0f ||
      config.electric_field_angle_degrees > 180.0f) {
    return "Electric field strength or angle is outside the supported range.";
  }
  if (config.damping_coefficient_n_m_s < 0.0f ||
      config.damping_coefficient_n_m_s > kMaximumDamping) {
    return "Rotational damping must be between 0 and 1000 N*m*s/rad.";
  }

  const float moment_of_inertia = GetPendulumMomentOfInertia(config);
  const float restoring_torque = GetPendulumRestoringTorqueMagnitude(config);
  const float small_angle_period = GetSmallAnglePeriod(config);
  const PendulumState initial_state = MakeInitialPendulumState(config);
  if (!std::isfinite(moment_of_inertia) || moment_of_inertia <= 0.0f ||
      !std::isfinite(restoring_torque) || restoring_torque <= 0.000001f ||
      !std::isfinite(small_angle_period) ||
      !IsPendulumDerivedFinite(
          CalculatePendulumDerived(config, initial_state))) {
    return "The selected values do not produce finite pendulum quantities.";
  }

  const float maximum_angular_speed =
      std::sqrt(config.initial_angular_velocity_rad_s *
                    config.initial_angular_velocity_rad_s +
                4.0f * restoring_torque / moment_of_inertia);
  const float maximum_angular_acceleration =
      restoring_torque / moment_of_inertia +
      (config.damping_enabled ? config.damping_coefficient_n_m_s *
                                    maximum_angular_speed / moment_of_inertia
                              : 0.0f);
  if (!std::isfinite(maximum_angular_speed) ||
      !std::isfinite(maximum_angular_acceleration) ||
      maximum_angular_speed * kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep ||
      maximum_angular_acceleration * kPendulumPhysicsStep *
              kPendulumPhysicsStep >
          kPendulumMaximumAnglePerStep ||
      (config.damping_enabled && config.damping_coefficient_n_m_s *
                                         kPendulumPhysicsStep /
                                         moment_of_inertia >=
                                     1.0f)) {
    return "This parameter combination is too fast or stiff for stable "
           "integration.";
  }
  return nullptr;
}

bool StepPendulum(const PendulumConfig& config, float delta_time,
                  PendulumState* state) {
  if (state == nullptr || !std::isfinite(delta_time) || delta_time <= 0.0f) {
    return false;
  }
  const PendulumDerived before = CalculatePendulumDerived(config, *state);
  if (!IsPendulumDerivedFinite(before) ||
      std::abs(state->angular_velocity_rad_s) * delta_time >
          kPendulumMaximumAnglePerStep ||
      std::abs(before.angular_acceleration_rad_s2) * delta_time * delta_time >
          kPendulumMaximumAnglePerStep) {
    return false;
  }

  state->angular_velocity_rad_s +=
      before.angular_acceleration_rad_s2 * delta_time;
  state->angle_radians = std::remainder(
      state->angle_radians + state->angular_velocity_rad_s * delta_time,
      2.0f * kPendulumPi);
  state->time_seconds += delta_time;
  state->angular_acceleration_rad_s2 =
      CalculatePendulumDerived(config, *state).angular_acceleration_rad_s2;
  return GetPendulumStateError(config, *state) == nullptr;
}

bool PendulumSliderInputFloat(const char* label, float* value, float minimum,
                              float maximum, const char* format,
                              ImGuiSliderFlags flags = ImGuiSliderFlags_None) {
  constexpr float kControlStart = 300.0f;
  constexpr float kSliderWidth = 320.0f;
  constexpr float kInputWidth = 110.0f;

  ImGui::PushID(label);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(kControlStart);
  ImGui::SetNextItemWidth(kSliderWidth);
  bool changed = ImGui::SliderFloat("##slider", value, minimum, maximum, format,
                                    flags | ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(kInputWidth);
  changed |= ImGui::InputFloat("##input", value, 0.0f, 0.0f, format);
  if (std::isfinite(*value)) {
    *value = std::clamp(*value, minimum, maximum);
  }
  ImGui::PopID();
  return changed;
}

PendulumSetupAction DrawPendulumSetupScreen(PendulumConfig* config) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("PivotLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "PivotLab: charged physical pendulum");
  ImGui::TextDisabled(
      "Drag a slider or type an exact SI value. Out-of-range finite values "
      "are clamped.");
  ImGui::Spacing();

  ImGui::TextUnformatted("Pendulum");
  ImGui::Separator();
  PendulumSliderInputFloat("Rod length L (m)", &config->rod_length_m,
                           kMinimumRodLength, kMaximumRodLength, "%.3f",
                           ImGuiSliderFlags_Logarithmic);
  PendulumSliderInputFloat("Rod mass M (kg)", &config->rod_mass_kg,
                           kMinimumMass, kMaximumMass, "%.3f",
                           ImGuiSliderFlags_Logarithmic);
  PendulumSliderInputFloat("Counterweight mass m (kg)",
                           &config->counterweight_mass_kg, kMinimumMass,
                           kMaximumMass, "%.3f", ImGuiSliderFlags_Logarithmic);
  const float rod_length_for_control =
      std::isfinite(config->rod_length_m)
          ? std::clamp(config->rod_length_m, kMinimumRodLength,
                       kMaximumRodLength)
          : kMaximumRodLength;
  PendulumSliderInputFloat("Counterweight distance r (m)",
                           &config->counterweight_distance_m, 0.0f,
                           rod_length_for_control, "%.3f");
  PendulumSliderInputFloat("Initial angle from vertical down, CCW (degrees)",
                           &config->initial_angle_degrees, -180.0f, 180.0f,
                           "%.2f");
  PendulumSliderInputFloat(
      "Initial angular speed (rad/s)", &config->initial_angular_velocity_rad_s,
      -kMaximumInitialAngularSpeed, kMaximumInitialAngularSpeed, "%.3f");
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Gravity g (m/s^2)");
  ImGui::SameLine(300.0f);
  if (ImGui::RadioButton("9.8", config->gravity_m_s2 == 9.8f)) {
    config->gravity_m_s2 = 9.8f;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("10", config->gravity_m_s2 == 10.0f)) {
    config->gravity_m_s2 = 10.0f;
  }

  ImGui::Spacing();
  ImGui::TextUnformatted("Uniform electric field");
  ImGui::Separator();
  ImGui::Checkbox("Enable electric field", &config->electric_field_enabled);
  ImGui::BeginDisabled(!config->electric_field_enabled);
  ImGui::Checkbox("Counterweight carries charge",
                  &config->counterweight_charged);
  ImGui::BeginDisabled(!config->counterweight_charged);
  PendulumSliderInputFloat(
      "Counterweight charge q (C)", &config->counterweight_charge_c,
      -kMaximumChargeMagnitude, kMaximumChargeMagnitude, "%.6g");
  ImGui::EndDisabled();
  PendulumSliderInputFloat(
      "Electric field E (N/C)", &config->electric_field_strength_n_c, 0.0f,
      kMaximumElectricField, "%.6g", ImGuiSliderFlags_Logarithmic);
  PendulumSliderInputFloat("E angle from +X, CCW (degrees)",
                           &config->electric_field_angle_degrees, -180.0f,
                           180.0f, "%.2f");
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::Checkbox("Enable linear rotational damping", &config->damping_enabled);
  ImGui::BeginDisabled(!config->damping_enabled);
  PendulumSliderInputFloat("Damping c (N*m*s/rad)",
                           &config->damping_coefficient_n_m_s, 0.0f,
                           kMaximumDamping, "%.4g");
  ImGui::EndDisabled();

  const char* error = GetPendulumConfigError(*config);
  if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | I = %.4f kg*m^2 | small-oscillation T = %.4f s",
                       GetPendulumMomentOfInertia(*config),
                       GetSmallAnglePeriod(*config));
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
  PendulumSetupAction action = PendulumSetupAction::kNone;
  if (ImGui::Button("Back to model selection", {button_width, 38.0f})) {
    action = PendulumSetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Restore defaults", {button_width, 38.0f})) {
    *config = PendulumConfig{};
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Start simulation", {button_width, 38.0f})) {
    action = PendulumSetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

void DrawPendulumArrow(ImDrawList* draw_list, ImVec2 start, ImVec2 direction,
                       float length, ImU32 color) {
  const float direction_length = std::hypot(direction.x, direction.y);
  if (direction_length <= 0.0f) {
    return;
  }
  direction.x /= direction_length;
  direction.y /= direction_length;
  const ImVec2 end{start.x + direction.x * length,
                   start.y + direction.y * length};
  draw_list->AddLine(start, end, color, 3.0f);
  const ImVec2 normal{-direction.y, direction.x};
  constexpr float kHeadLength = 13.0f;
  constexpr float kHeadWidth = 7.0f;
  draw_list->AddTriangleFilled(
      end,
      {end.x - direction.x * kHeadLength + normal.x * kHeadWidth,
       end.y - direction.y * kHeadLength + normal.y * kHeadWidth},
      {end.x - direction.x * kHeadLength - normal.x * kHeadWidth,
       end.y - direction.y * kHeadLength - normal.y * kHeadWidth},
      color);
}

void DrawPendulumScene(const PendulumConfig& config,
                       const PendulumState& state) {
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float scene_left = std::min(440.0f, display_size.x * 0.4f);
  const float scene_width = std::max(display_size.x - scene_left, 200.0f);
  const float rod_pixels =
      std::max(100.0f, std::min(scene_width, display_size.y) * 0.3f);
  const ImVec2 pivot{scene_left + scene_width * 0.52f, display_size.y * 0.45f};
  const ImVec2 rod_direction{std::sin(state.angle_radians),
                             std::cos(state.angle_radians)};
  const ImVec2 rod_end{pivot.x + rod_direction.x * rod_pixels,
                       pivot.y + rod_direction.y * rod_pixels};
  const float weight_fraction =
      config.counterweight_distance_m / config.rod_length_m;
  const ImVec2 weight_position{
      pivot.x + rod_direction.x * rod_pixels * weight_fraction,
      pivot.y + rod_direction.y * rod_pixels * weight_fraction};

  draw_list->AddLine(pivot, rod_end, IM_COL32(210, 170, 95, 255), 9.0f);
  draw_list->AddCircleFilled(weight_position, 19.0f,
                             IM_COL32(255, 100, 105, 255));
  draw_list->AddCircleFilled(pivot, 10.0f, IM_COL32(225, 230, 240, 255));
  draw_list->AddCircle(pivot, 11.0f, IM_COL32(40, 45, 55, 255), 24, 2.0f);

  constexpr std::size_t kArcPointCount = 25;
  std::array<ImVec2, kArcPointCount> arc_points{};
  const float arc_start = kPendulumPi * 0.5f;
  const float arc_end = arc_start - state.angle_radians;
  for (std::size_t i = 0; i < arc_points.size(); ++i) {
    const float ratio =
        static_cast<float>(i) / static_cast<float>(arc_points.size() - 1);
    const float angle = arc_start + (arc_end - arc_start) * ratio;
    arc_points[i] = {pivot.x + std::cos(angle) * 54.0f,
                     pivot.y + std::sin(angle) * 54.0f};
  }
  draw_list->AddPolyline(arc_points.data(), static_cast<int>(arc_points.size()),
                         IM_COL32(130, 205, 255, 255), 0, 2.0f);

  char angle_label[48];
  std::snprintf(angle_label, sizeof(angle_label), "theta = %+.2f deg",
                state.angle_radians * kPendulumRadiansToDegrees);
  draw_list->AddText({pivot.x + 62.0f, pivot.y + 36.0f},
                     IM_COL32(150, 215, 255, 255), angle_label);

  char weight_label[64];
  std::snprintf(
      weight_label, sizeof(weight_label), "m=%.3g kg, q=%.3g C",
      config.counterweight_mass_kg,
      config.counterweight_charged ? config.counterweight_charge_c : 0.0f);
  draw_list->AddText({weight_position.x + 24.0f, weight_position.y - 9.0f},
                     IM_COL32(255, 210, 210, 255), weight_label);

  if (config.electric_field_enabled) {
    const float field_angle = GetPendulumFieldAngle(config);
    const ImVec2 field_direction{std::cos(field_angle), -std::sin(field_angle)};
    constexpr float kFieldArrowLength = 80.0f;
    const ImVec2 field_center{scene_left + 105.0f, display_size.y - 120.0f};
    const ImVec2 field_start{
        field_center.x - field_direction.x * kFieldArrowLength * 0.5f,
        field_center.y - field_direction.y * kFieldArrowLength * 0.5f};
    DrawPendulumArrow(draw_list, field_start, field_direction,
                      kFieldArrowLength, IM_COL32(100, 225, 150, 255));
    char field_label[80];
    std::snprintf(field_label, sizeof(field_label),
                  "E = %.4g N/C, %.2f deg from +X CCW",
                  config.electric_field_strength_n_c,
                  config.electric_field_angle_degrees);
    draw_list->AddText({scene_left + 35.0f, display_size.y - 50.0f},
                       IM_COL32(120, 235, 165, 255), field_label);
  }

  constexpr ImVec2 kGravityDirection{0.0f, 1.0f};
  constexpr float kGravityArrowLength = 80.0f;
  const ImVec2 gravity_center{scene_left + 430.0f, display_size.y - 120.0f};
  const ImVec2 gravity_start{
      gravity_center.x - kGravityDirection.x * kGravityArrowLength * 0.5f,
      gravity_center.y - kGravityDirection.y * kGravityArrowLength * 0.5f};
  DrawPendulumArrow(draw_list, gravity_start, kGravityDirection,
                    kGravityArrowLength, IM_COL32(245, 190, 90, 255));
  char gravity_label[64];
  std::snprintf(gravity_label, sizeof(gravity_label),
                "g = %.3g m/s^2, downward (-Y)", config.gravity_m_s2);
  draw_list->AddText({scene_left + 355.0f, display_size.y - 50.0f},
                     IM_COL32(255, 205, 115, 255), gravity_label);
}

bool DrawPendulumMonitor(const PendulumConfig& config,
                         const PendulumState& current_state,
                         const std::vector<PendulumState>& history,
                         bool* paused, float* inspect_time, bool* follow_live,
                         const char* runtime_error) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({410.0f, 560.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("PivotLab monitor", nullptr, kWindowFlags);

  ImGui::TextColored(*paused ? ImVec4{1.0f, 0.7f, 0.25f, 1.0f}
                             : ImVec4{0.35f, 0.9f, 0.5f, 1.0f},
                     "%s", *paused ? "PAUSED" : "RUNNING");
  ImGui::SameLine();
  ImGui::Text("t = %.4f s", current_state.time_seconds);
  ImGui::TextDisabled("Space: pause/resume");
  ImGui::Separator();

  if (*follow_live) {
    *inspect_time = current_state.time_seconds;
  }
  if (ImGui::Checkbox("Live", follow_live) && *follow_live) {
    *inspect_time = current_state.time_seconds;
  }
  ImGui::SameLine();
  ImGui::TextUnformatted("Inspect time");
  ImGui::SameLine();
  const float maximum_inspect_time =
      std::max(current_state.time_seconds, kPendulumPhysicsStep);
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_time_changed = ImGui::SliderFloat(
      "##history_slider", inspect_time, 0.0f, maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_time_changed |=
      ImGui::InputFloat("##history_input", inspect_time, 0.0f, 0.0f, "%.3f");
  if (inspect_time_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0f, current_state.time_seconds);
    *follow_live = false;
  }

  const PendulumState* inspected_state =
      FindPendulumState(history, *inspect_time);
  if (inspected_state == nullptr) {
    inspected_state = &current_state;
  }
  const PendulumDerived derived =
      CalculatePendulumDerived(config, *inspected_state);

  ImGui::Text("theta: %+.4f deg",
              inspected_state->angle_radians * kPendulumRadiansToDegrees);
  ImGui::Text("omega: %+.6f rad/s", inspected_state->angular_velocity_rad_s);
  ImGui::Text("alpha: %+.6f rad/s^2",
              inspected_state->angular_acceleration_rad_s2);
  ImGui::Text("I: %.6f kg*m^2", derived.moment_of_inertia_kg_m2);
  ImGui::Text("Small-oscillation period: %.6f s", GetSmallAnglePeriod(config));

  ImGui::Spacing();
  ImGui::TextUnformatted("Torques (N*m)");
  ImGui::Separator();
  ImGui::Text("gravity: %+.6f", derived.gravity_torque_n_m);
  ImGui::Text("electric: %+.6f", derived.electric_torque_n_m);
  ImGui::Text("damping: %+.6f", derived.damping_torque_n_m);
  ImGui::Text("total: %+.6f", derived.total_torque_n_m);

  ImGui::Spacing();
  ImGui::TextUnformatted("Energy (J)");
  ImGui::Separator();
  ImGui::Text("kinetic: %+.6f", derived.kinetic_energy_j);
  ImGui::Text("gravity potential: %+.6f",
              derived.gravitational_potential_energy_j);
  ImGui::Text("electric potential: %+.6f", derived.electric_potential_energy_j);
  ImGui::Text("total: %+.6f", derived.total_energy_j);

  if (runtime_error != nullptr) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error);
  }

  const float width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  if (ImGui::Button(*paused ? "Resume" : "Pause", {width, 36.0f})) {
    *paused = !*paused;
  }
  ImGui::SameLine();
  const bool stop = ImGui::Button("Stop and choose model", {width, 36.0f});
  ImGui::End();
  return stop;
}

}  // namespace

namespace tiny2d::sandbox {

SimulationResult RunRotationPendulumSimulation(SDL_Renderer* renderer) {
  if (renderer == nullptr) {
    return SimulationResult::kBackToSelection;
  }

  PendulumConfig config;
  PendulumState state = MakeInitialPendulumState(config);
  // ponytail: Match V9 and keep one run in memory; cap only for long sessions.
  std::vector<PendulumState> history;
  bool simulation_started = false;
  bool paused = false;
  bool follow_live = true;
  float inspect_time = 0.0f;
  const char* runtime_error = nullptr;
  double accumulated_time = 0.0;
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  Uint64 previous_time = SDL_GetPerformanceCounter();

  while (true) {
    bool return_requested = false;
    SimulationResult return_result = SimulationResult::kBackToSelection;
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        return_requested = true;
        return_result = SimulationResult::kQuit;
      } else if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                 event.key.keysym.sym == SDLK_SPACE && simulation_started &&
                 runtime_error == nullptr) {
        paused = !paused;
      }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint64 current_time = SDL_GetPerformanceCounter();
    if (!return_requested && !simulation_started) {
      previous_time = current_time;
      accumulated_time = 0.0;
      const PendulumSetupAction action = DrawPendulumSetupScreen(&config);
      if (action == PendulumSetupAction::kBack) {
        return_requested = true;
      } else if (action == PendulumSetupAction::kStart) {
        state = MakeInitialPendulumState(config);
        history.clear();
        history.push_back(state);
        simulation_started = true;
        paused = false;
        follow_live = true;
        inspect_time = 0.0f;
        runtime_error = nullptr;
        previous_time = current_time;
      }
    } else if (simulation_started) {
      if (!return_requested && !paused && runtime_error == nullptr) {
        const double frame_time = std::min(
            static_cast<double>(current_time - previous_time) / frequency,
            kPendulumMaximumFrameTime);
        previous_time = current_time;
        accumulated_time += frame_time;
        while (accumulated_time >= kPendulumPhysicsStep) {
          if (!StepPendulum(config, kPendulumPhysicsStep, &state)) {
            runtime_error = GetPendulumStateError(config, state);
            if (runtime_error == nullptr) {
              runtime_error =
                  "The fixed-step integrator rejected an unstable state.";
            }
            paused = true;
            accumulated_time = 0.0;
            break;
          }
          history.push_back(state);
          accumulated_time -= kPendulumPhysicsStep;
        }
      } else {
        previous_time = current_time;
        accumulated_time = 0.0;
      }

      DrawPendulumScene(config, state);
      if (!return_requested &&
          DrawPendulumMonitor(config, state, history, &paused, &inspect_time,
                              &follow_live, runtime_error)) {
        return_requested = true;
      }
    }

    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 18, 20, 24, 255);
    SDL_RenderClear(renderer);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);

    if (return_requested) {
      return return_result;
    }
  }
}

}  // namespace tiny2d::sandbox
