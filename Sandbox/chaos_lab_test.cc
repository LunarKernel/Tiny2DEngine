#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "chaos_lab_model.h"
#include "test_support.h"

namespace {

using tiny2d::Circle;
using tiny2d::sandbox::BuildChaosCsv;
using tiny2d::sandbox::CalculateChaosDerived;
using tiny2d::sandbox::ChaosConfig;
using tiny2d::sandbox::ChaosDerived;
using tiny2d::sandbox::ChaosState;
using tiny2d::sandbox::FindChaosState;
using tiny2d::sandbox::GetChaosConfigError;
using tiny2d::sandbox::GetChaosStateError;
using tiny2d::sandbox::kChaosPhysicsStep;
using tiny2d::sandbox::kChaosPivotXM;
using tiny2d::sandbox::kChaosPivotYM;
using tiny2d::sandbox::MakeChaosDampedConfig;
using tiny2d::sandbox::MakeChaosLargeAmplitudeConfig;
using tiny2d::sandbox::MakeChaosReferenceConfig;
using tiny2d::sandbox::MakeChaosSlowModeConfig;
using tiny2d::sandbox::MakeInitialChaosState;
using tiny2d::sandbox::StepChaos;

bool Near(double actual, double expected, double tolerance = 0.00001) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

bool SameBob(const Circle& a, const Circle& b) {
  return a.mass == b.mass && a.position.x == b.position.x &&
         a.position.y == b.position.y && a.velocity.x == b.velocity.x &&
         a.velocity.y == b.velocity.y && a.angle == b.angle &&
         a.angular_velocity == b.angular_velocity;
}

bool SameState(const ChaosState& a, const ChaosState& b) {
  return SameBob(a.bob_1, b.bob_1) && SameBob(a.bob_2, b.bob_2) &&
         SameBob(a.shadow_bob_1, b.shadow_bob_1) &&
         SameBob(a.shadow_bob_2, b.shadow_bob_2) &&
         a.time_seconds == b.time_seconds &&
         a.dissipated_energy_j == b.dissipated_energy_j &&
         a.anchor_rod_force_n == b.anchor_rod_force_n &&
         a.link_rod_force_n == b.link_rod_force_n;
}

double EnergyScale(double initial_mechanical_energy,
                   double peak_released_potential_energy) {
  return std::max(
      {initial_mechanical_energy, peak_released_potential_energy, 1.0});
}

double ShadowRodError(const ChaosConfig& config, const ChaosState& state,
                      bool first_rod) {
  if (first_rod) {
    return std::hypot(static_cast<double>(state.shadow_bob_1.position.x) -
                          kChaosPivotXM,
                      static_cast<double>(state.shadow_bob_1.position.y) -
                          kChaosPivotYM) -
           config.length_1_m;
  }
  return std::hypot(static_cast<double>(state.shadow_bob_2.position.x) -
                        state.shadow_bob_1.position.x,
                    static_cast<double>(state.shadow_bob_2.position.y) -
                        state.shadow_bob_1.position.y) -
         config.length_2_m;
}

// Measures the period of theta1 from same-direction zero crossings.
double MeasureTheta1Period(const ChaosConfig& config, int maximum_steps) {
  ChaosState state = MakeInitialChaosState(config);
  double previous_theta = CalculateChaosDerived(config, state).theta_1_rad;
  double previous_time = 0.0;
  std::array<double, 3> crossings{};
  std::size_t crossing_count = 0;
  for (int step = 1; step <= maximum_steps && crossing_count < crossings.size();
       ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &state));
    const double theta = CalculateChaosDerived(config, state).theta_1_rad;
    const double time = static_cast<double>(step) * kChaosPhysicsStep;
    if (previous_theta > 0.0 && theta <= 0.0) {
      const double fraction = previous_theta / (previous_theta - theta);
      crossings[crossing_count++] =
          previous_time + fraction * kChaosPhysicsStep;
    }
    previous_theta = theta;
    previous_time = time;
  }
  CHECK(crossing_count == crossings.size());
  return (crossings[2] - crossings[0]) / 2.0;
}

void TestPresetsAndAnalyticalValues() {
  CHECK(GetChaosConfigError(MakeChaosSlowModeConfig()) == nullptr);
  CHECK(GetChaosConfigError(MakeChaosLargeAmplitudeConfig()) == nullptr);
  CHECK(GetChaosConfigError(MakeChaosReferenceConfig()) == nullptr);
  CHECK(GetChaosConfigError(MakeChaosDampedConfig()) == nullptr);

  const ChaosConfig config = MakeChaosSlowModeConfig();
  const ChaosState state = MakeInitialChaosState(config);
  const ChaosDerived derived = CalculateChaosDerived(config, state);

  // Equal masses and lengths: omega^2 = (2 -/+ sqrt(2)) g / L and shape
  // ratios +/- sqrt(2).
  const double gravity_over_length = 9.81 / 2.0;
  CHECK(Near(derived.slow_mode_omega_rad_s,
             std::sqrt((2.0 - std::sqrt(2.0)) * gravity_over_length), 0.0001));
  CHECK(Near(derived.fast_mode_omega_rad_s,
             std::sqrt((2.0 + std::sqrt(2.0)) * gravity_over_length), 0.0001));
  CHECK(Near(derived.slow_mode_shape_ratio, std::sqrt(2.0), 0.0001));
  CHECK(Near(derived.fast_mode_shape_ratio, -std::sqrt(2.0), 0.0001));

  // The initial state reproduces the configured angles under the
  // clockwise-positive convention, and starts with zero rod error.
  CHECK(Near(derived.theta_1_rad, 2.0 * 3.14159265358979 / 180.0, 0.0001));
  CHECK(Near(derived.theta_2_rad,
             2.0 * std::sqrt(2.0) * 3.14159265358979 / 180.0, 0.0001));
  CHECK(std::abs(derived.rod_1_length_error_m) < 0.00001);
  CHECK(std::abs(derived.rod_2_length_error_m) < 0.00001);
  CHECK(derived.mechanical_energy_j == 0.0);

  // A positive initial angular velocity must read back as positive omega.
  ChaosConfig spinning = MakeChaosSlowModeConfig();
  spinning.initial_angular_velocity_1_rad_s = 1.5f;
  spinning.initial_angular_velocity_2_rad_s = -0.5f;
  const ChaosState spinning_state = MakeInitialChaosState(spinning);
  const ChaosDerived spinning_derived =
      CalculateChaosDerived(spinning, spinning_state);
  CHECK(Near(spinning_derived.omega_1_rad_s, 1.5, 0.0001));
  CHECK(Near(spinning_derived.omega_2_rad_s, -0.5, 0.0001));
}

void TestSlowModePeriod() {
  const ChaosConfig config = MakeChaosSlowModeConfig();
  const ChaosState state = MakeInitialChaosState(config);
  const double expected_period =
      2.0 * 3.14159265358979 /
      CalculateChaosDerived(config, state).slow_mode_omega_rad_s;
  const double measured_period = MeasureTheta1Period(config, 6000);
  CHECK(std::abs(measured_period - expected_period) / expected_period <= 0.02);
}

void TestFastModePeriod() {
  ChaosConfig config = MakeChaosSlowModeConfig();
  config.initial_angle_2_deg = -2.0f * 1.41421356f;  // Antiphase shape.
  const ChaosState state = MakeInitialChaosState(config);
  const double expected_period =
      2.0 * 3.14159265358979 /
      CalculateChaosDerived(config, state).fast_mode_omega_rad_s;
  const double measured_period = MeasureTheta1Period(config, 4000);
  CHECK(std::abs(measured_period - expected_period) / expected_period <= 0.02);
}

void TestLargeAmplitudeConservativeEnergy() {
  // The energy criterion binds in the 25-degree regime, where the
  // measured worst drift at 32 substeps is 0.41% of the released scale.
  const ChaosConfig config = MakeChaosLargeAmplitudeConfig();
  ChaosState state = MakeInitialChaosState(config);
  double peak_released = 0.0;
  for (int step = 1; step <= 60 * 480; ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &state));
    if (step % 480 != 0) {
      continue;
    }
    const ChaosDerived derived = CalculateChaosDerived(config, state);
    peak_released =
        std::max(peak_released, std::max(0.0, -derived.potential_energy_j));
    const double scale = EnergyScale(0.0, peak_released);
    CHECK(std::abs(derived.mechanical_energy_j) <= 0.01 * scale);
    CHECK(state.dissipated_energy_j == 0.0);
  }
  CHECK(peak_released > 1.0);
}

void TestChaoticRodDriftAndBoundedLoss() {
  // The chaotic 120-degree reference: rod lengths and finiteness stay
  // exact, while the documented whip-regime energy loss stays under a 15%
  // ceiling so a whip-regime solver regression cannot hide behind the
  // limitation. Basis: the shipped 32x substepping measures 3.878%
  // locally (>=3.8x margin); 15% sits at the smallest recorded failure
  // mode (8x substepping, 15.1%) and catches everything worse; it is not
  // tighter because the chaotic trajectory is compiler-dependent, so the
  // cross-toolchain worst ratio is an ensemble draw sampled only locally.
  const ChaosConfig config = MakeChaosReferenceConfig();
  ChaosState state = MakeInitialChaosState(config);
  double peak_released = 0.0;
  double worst_energy_ratio = 0.0;
  for (int step = 1; step <= 60 * 480; ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &state));
    if (step % 480 != 0) {
      continue;
    }
    const ChaosDerived derived = CalculateChaosDerived(config, state);
    peak_released =
        std::max(peak_released, std::max(0.0, -derived.potential_energy_j));
    const double scale = EnergyScale(0.0, peak_released);
    worst_energy_ratio = std::max(
        worst_energy_ratio, std::abs(derived.mechanical_energy_j) / scale);
    CHECK(std::abs(derived.rod_1_length_error_m) < 0.0001);
    CHECK(std::abs(derived.rod_2_length_error_m) < 0.0001);
    CHECK(std::abs(ShadowRodError(config, state, true)) < 0.0001);
    CHECK(std::abs(ShadowRodError(config, state, false)) < 0.0001);
    CHECK(state.dissipated_energy_j == 0.0);
  }
  CHECK(peak_released > 20.0);
  CHECK(worst_energy_ratio < 0.15);
}

void TestDampedEnergyAccounting() {
  const ChaosConfig config = MakeChaosDampedConfig();
  ChaosState state = MakeInitialChaosState(config);
  const double initial_energy =
      CalculateChaosDerived(config, state).mechanical_energy_j;
  CHECK(initial_energy > 1.0);
  double peak_released = 0.0;
  for (int step = 1; step <= 60 * 480; ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &state));
    if (step % 480 != 0) {
      continue;
    }
    const ChaosDerived derived = CalculateChaosDerived(config, state);
    peak_released =
        std::max(peak_released, std::max(0.0, -derived.potential_energy_j));
    const double scale = EnergyScale(initial_energy, peak_released);
    CHECK(derived.mechanical_energy_j <= initial_energy + 0.001 * scale);
    CHECK(std::abs(derived.accounted_energy_j - initial_energy) <=
          0.01 * scale);
  }
  CHECK(state.dissipated_energy_j > 0.0);
}

void TestDeterminismAndZeroOffsetShadow() {
  ChaosConfig config = MakeChaosReferenceConfig();
  config.shadow_offset_rad = 0.0f;
  ChaosState first = MakeInitialChaosState(config);
  ChaosState second = MakeInitialChaosState(config);
  for (int step = 1; step <= 10 * 480; ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &first));
    CHECK(StepChaos(config, kChaosPhysicsStep, &second));
    if (step % 480 == 0) {
      CHECK(SameState(first, second));
      // With no offset the shadow tracks the primary bitwise: the two
      // engine calls receive identical inputs.
      CHECK(SameBob(first.bob_1, first.shadow_bob_1));
      CHECK(SameBob(first.bob_2, first.shadow_bob_2));
    }
  }
}

void TestChaoticDivergence() {
  const ChaosConfig config = MakeChaosReferenceConfig();
  ChaosState state = MakeInitialChaosState(config);
  double peak_separation = 0.0;
  for (int step = 1; step <= 30 * 480; ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &state));
    if (step % 480 == 0) {
      peak_separation = std::max(
          peak_separation, CalculateChaosDerived(config, state).separation_rad);
    }
  }
  // Physical divergence within 30 s: three decades of growth from the
  // 1e-4 offset at some checkpoint. The running maximum implements the
  // criterion literally, so a momentary dip of the wrapped metric at
  // exactly t = 30 s cannot flake it on a different toolchain's
  // trajectory.
  CHECK(peak_separation >= 1000.0 * config.shadow_offset_rad);
}

void TestValidationAndFailureAtomicity() {
  const auto expect_config_error = [](auto mutate) {
    ChaosConfig config = MakeChaosReferenceConfig();
    mutate(config);
    CHECK(GetChaosConfigError(config) != nullptr);
    bool threw = false;
    try {
      MakeInitialChaosState(config);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };

  expect_config_error([](ChaosConfig& c) {
    c.mass_1_kg = std::numeric_limits<float>::quiet_NaN();
  });
  expect_config_error([](ChaosConfig& c) { c.mass_2_kg = 0.001f; });
  expect_config_error([](ChaosConfig& c) { c.length_1_m = 0.1f; });
  expect_config_error([](ChaosConfig& c) { c.length_2_m = 21.0f; });
  expect_config_error([](ChaosConfig& c) {
    c.length_1_m = 20.0f;
    c.length_2_m = 20.5f;
  });
  expect_config_error([](ChaosConfig& c) { c.bob_radius_m = 0.01f; });
  expect_config_error([](ChaosConfig& c) {
    // Radius at half the lower rod would let same-run bobs touch.
    c.bob_radius_m = 1.0f;
    c.length_2_m = 2.0f;
  });
  expect_config_error([](ChaosConfig& c) {
    // Combined lengths at the cap plus a large bob exceed the swing
    // radius margin.
    c.length_1_m = 20.0f;
    c.length_2_m = 20.0f;
    c.bob_radius_m = 1.9f;
  });
  expect_config_error([](ChaosConfig& c) { c.initial_angle_1_deg = 181.0f; });
  expect_config_error(
      [](ChaosConfig& c) { c.initial_angular_velocity_2_rad_s = 21.0f; });
  expect_config_error([](ChaosConfig& c) { c.gravity_m_s2 = 0.0f; });
  expect_config_error([](ChaosConfig& c) { c.linear_damping_per_s = -1.0f; });
  expect_config_error([](ChaosConfig& c) { c.shadow_offset_rad = 0.2f; });

  const ChaosConfig config = MakeChaosReferenceConfig();
  ChaosState state = MakeInitialChaosState(config);
  const ChaosState before = state;
  CHECK(!StepChaos(config, 0.0f, &state));
  CHECK(!StepChaos(config, -kChaosPhysicsStep, &state));
  CHECK(!StepChaos(config, kChaosPhysicsStep * 2.0f, &state));
  CHECK(!StepChaos(config, std::numeric_limits<float>::quiet_NaN(), &state));
  CHECK(!StepChaos(config, kChaosPhysicsStep, nullptr));
  CHECK(SameState(state, before));

  ChaosState tampered = state;
  tampered.bob_1.mass = 2.0f * config.mass_1_kg;
  CHECK(GetChaosStateError(config, tampered) != nullptr);
  bool threw = false;
  try {
    CalculateChaosDerived(config, tampered);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  CHECK(!StepChaos(config, kChaosPhysicsStep, &tampered));

  ChaosState stretched = state;
  stretched.bob_2.position.y += 0.01f;
  CHECK(GetChaosStateError(config, stretched) != nullptr);
}

void TestCsvExportMetadataAndRows() {
  const ChaosConfig config = MakeChaosReferenceConfig();
  ChaosState state = MakeInitialChaosState(config);
  // A deliberately non-uniform history: the time column must reproduce
  // each sample's stored time, never a uniform index interval.
  std::vector<ChaosState> history;
  history.push_back(state);
  for (int step = 1; step <= 96; ++step) {
    CHECK(StepChaos(config, kChaosPhysicsStep, &state));
    if (step == 5 || step == 41 || step == 96) {
      history.push_back(state);
    }
  }

  const std::string csv =
      BuildChaosCsv(config, history, "9.9.9-test", "UNIT TEST");
  CHECK(BuildChaosCsv(config, history, "9.9.9-test", "UNIT TEST") == csv);

  std::vector<std::string> lines;
  std::size_t start = 0;
  while (start < csv.size()) {
    const std::size_t end = csv.find('\n', start);
    lines.push_back(csv.substr(start, end - start));
    start = end + 1;
  }
  // 16 metadata lines (3 fixed + 12 params + status), the header, and
  // one row per sample.
  CHECK(lines.size() == 17 + history.size());
  CHECK(lines[0] == "# tiny2d-csv 1");
  CHECK(lines[1] == "# product_version: 9.9.9-test");
  CHECK(lines[2] == "# model: V19 ChaosLab");
  CHECK(lines[3] == "# param mass_1_kg: 1");
  CHECK(lines[7] == "# param bob_radius_m: 0.0500000007");
  CHECK(lines[8] == "# param initial_angle_1_deg: 120");
  CHECK(lines[14] == "# param shadow_offset_rad: 9.99999975e-05");
  CHECK(lines[15] == "# status: UNIT TEST");

  const std::string& header = lines[16];
  CHECK(header ==
        "time_s,theta_1_rad,theta_2_rad,omega_1_rad_s,omega_2_rad_s,"
        "rod_1_length_error_m,rod_2_length_error_m,kinetic_energy_j,"
        "potential_energy_j,mechanical_energy_j,dissipated_energy_j,"
        "accounted_energy_j,separation_rad,separation_decades,"
        "anchor_rod_force_n,link_rod_force_n");
  const auto count_cells = [](const std::string& line) {
    std::size_t cells = 1;
    for (const char character : line) {
      if (character == ',') {
        ++cells;
      }
    }
    return cells;
  };
  for (std::size_t i = 0; i < history.size(); ++i) {
    const std::string& row = lines[17 + i];
    CHECK(count_cells(row) == 16);
    // The first cell round-trips the sample's exact stored time.
    const double parsed = std::strtod(row.c_str(), nullptr);
    CHECK(parsed == history[i].time_seconds);
  }

  // Invalid inputs reject atomically through the existing validation.
  bool threw = false;
  try {
    ChaosConfig bad = config;
    bad.mass_1_kg = -1.0f;
    BuildChaosCsv(bad, history, "9.9.9-test", "status");
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

void TestHistoryLookup() {
  const ChaosConfig config = MakeChaosReferenceConfig();
  std::vector<ChaosState> history;
  for (int i = 0; i < 3; ++i) {
    ChaosState state = MakeInitialChaosState(config);
    state.time_seconds = static_cast<double>(i);
    history.push_back(state);
  }
  CHECK(FindChaosState(history, -1.0) == &history.front());
  CHECK(FindChaosState(history, 0.4) == &history.front());
  CHECK(FindChaosState(history, 0.6) == &history[1]);
  CHECK(FindChaosState(history, 9.0) == &history.back());
  CHECK(FindChaosState(history, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);
  const std::vector<ChaosState> empty;
  CHECK(FindChaosState(empty, 1.0) == nullptr);
}

struct NamedTest {
  const char* name;
  void (*function)();
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"presets and analytical values",
                TestPresetsAndAnalyticalValues},
      NamedTest{"slow normal-mode period", TestSlowModePeriod},
      NamedTest{"fast normal-mode period", TestFastModePeriod},
      NamedTest{"large-amplitude conservative energy",
                TestLargeAmplitudeConservativeEnergy},
      NamedTest{"chaotic rod drift and bounded loss",
                TestChaoticRodDriftAndBoundedLoss},
      NamedTest{"damped energy accounting", TestDampedEnergyAccounting},
      NamedTest{"determinism and zero-offset shadow",
                TestDeterminismAndZeroOffsetShadow},
      NamedTest{"chaotic divergence", TestChaoticDivergence},
      NamedTest{"validation and failure atomicity",
                TestValidationAndFailureAtomicity},
      NamedTest{"csv export metadata and rows", TestCsvExportMetadataAndRows},
      NamedTest{"history lookup", TestHistoryLookup},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << tiny2d::test::CheckCount()
            << " checks passed\n";
  return 0;
}
