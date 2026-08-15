#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "stack_lab_model.h"
#include "test_support.h"

namespace {

using tiny2d::ContactCache;
using tiny2d::Rectangle;
using tiny2d::sandbox::BuildStackCsv;
using tiny2d::sandbox::CalculateStackDerived;
using tiny2d::sandbox::ExtractStackSeries;
using tiny2d::sandbox::FindStackState;
using tiny2d::sandbox::GetStackConfigError;
using tiny2d::sandbox::GetStackSeriesLabel;
using tiny2d::sandbox::GetStackStateError;
using tiny2d::sandbox::kStackDriftSnapshotSeconds;
using tiny2d::sandbox::kStackPhysicsStep;
using tiny2d::sandbox::MakeInitialStackState;
using tiny2d::sandbox::MakeStackCollapseConfig;
using tiny2d::sandbox::MakeStackOffsetConfig;
using tiny2d::sandbox::MakeStackReferenceConfig;
using tiny2d::sandbox::StackConfig;
using tiny2d::sandbox::StackDerived;
using tiny2d::sandbox::StackSeries;
using tiny2d::sandbox::StackState;
using tiny2d::sandbox::StepStack;

bool Near(double actual, double expected, double tolerance = 0.00001) {
  return std::abs(actual - expected) <=
         tolerance * std::max({1.0, std::abs(actual), std::abs(expected)});
}

bool SameBox(const Rectangle& a, const Rectangle& b) {
  return a.mass == b.mass && a.position.x == b.position.x &&
         a.position.y == b.position.y && a.velocity.x == b.velocity.x &&
         a.velocity.y == b.velocity.y && a.angle == b.angle &&
         a.angular_velocity == b.angular_velocity;
}

bool SameState(const StackState& a, const StackState& b) {
  if (a.boxes.size() != b.boxes.size() || a.time_seconds != b.time_seconds ||
      a.cache.entries.size() != b.cache.entries.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.boxes.size(); ++i) {
    if (!SameBox(a.boxes[i], b.boxes[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.cache.entries.size(); ++i) {
    if (a.cache.entries[i].key != b.cache.entries[i].key ||
        a.cache.entries[i].normal_impulse !=
            b.cache.entries[i].normal_impulse ||
        a.cache.entries[i].tangent_impulse !=
            b.cache.entries[i].tangent_impulse) {
      return false;
    }
  }
  return true;
}

void TestPresetsAndInterfaceAnchors() {
  CHECK(GetStackConfigError(MakeStackReferenceConfig()) == nullptr);
  CHECK(GetStackConfigError(MakeStackOffsetConfig()) == nullptr);
  CHECK(GetStackConfigError(MakeStackCollapseConfig()) == nullptr);

  const StackConfig config = MakeStackReferenceConfig();
  const StackState state = MakeInitialStackState(config);
  const StackDerived derived = CalculateStackDerived(config, state);
  CHECK(derived.analytical_loads_n.size() == 9);
  // Interface k supports (n - k) boxes: bottom interface carries 9 m g.
  CHECK(Near(derived.analytical_loads_n[0], 9.0 * 9.81, 0.0001));
  CHECK(Near(derived.analytical_loads_n[8], 1.0 * 9.81, 0.0001));
  CHECK(derived.max_penetration_m <= 0.0);
  CHECK(derived.max_speed_mps == 0.0);
  CHECK(derived.mechanical_energy_j == 0.0);
}

void TestReferenceStackMeetsRoadmapCriteria() {
  const StackConfig config = MakeStackReferenceConfig();
  StackState state = MakeInitialStackState(config);
  std::array<double, 9> load_sums{};
  int load_samples = 0;
  double max_energy = 0.0;
  for (int step = 1; step <= 60 * 480; ++step) {
    CHECK(StepStack(config, kStackPhysicsStep, &state));
    if (step % 96 != 0) {
      continue;
    }
    const StackDerived derived = CalculateStackDerived(config, state);
    CHECK(derived.max_penetration_fraction < 0.005);
    CHECK(derived.energy_ok);
    max_energy = std::max(max_energy, derived.mechanical_energy_j);
    if (state.time_seconds > 50.0) {
      CHECK(derived.resting_speed_ok);
      CHECK(derived.drift_ok);
      CHECK(derived.height_ok);
      for (std::size_t i = 0; i < 9; ++i) {
        load_sums[i] += derived.interface_loads_n[i];
      }
      ++load_samples;
      CHECK(derived.cache_warm_started_count == derived.cache_contact_count);
    }
  }
  // Energy never exceeds the initial value beyond the 0.1% scale rule
  // (the stack starts at rest, scale floor 1 J).
  CHECK(max_energy <= 0.001 * 1.0);
  // Time-averaged interface loads match (n - k) m g within 1%.
  for (std::size_t i = 0; i < 9; ++i) {
    const double expected = (9.0 - static_cast<double>(i)) * 9.81;
    CHECK(std::abs(load_sums[i] / load_samples - expected) / expected < 0.01);
  }
}

void TestOffsetStackStandsWithBoundedWobble() {
  // Amended criterion 6: the offset stack STANDS (persistent contacts,
  // bounded tilt, penetration within twice the budget) and its wobble
  // stays under the 0.15 m/s regression ceiling. The wobble itself is
  // the documented manifold point-count-flicker limitation.
  const StackConfig config = MakeStackOffsetConfig();
  StackState state = MakeInitialStackState(config);
  for (int step = 1; step <= 60 * 480; ++step) {
    CHECK(StepStack(config, kStackPhysicsStep, &state));
    if (step % 96 != 0) {
      continue;
    }
    const StackDerived derived = CalculateStackDerived(config, state);
    CHECK(derived.max_penetration_fraction < 0.010);
    CHECK(derived.max_tilt_rad < 0.05);
    if (state.time_seconds > 50.0) {
      CHECK(derived.max_speed_mps < 0.15);
      // Every interface stays loaded: the stack has not collapsed.
      for (const double load : derived.interface_loads_n) {
        CHECK(load > 0.0);
      }
    }
  }
}

void TestCollapsePresetStaysFiniteAndDeterministic() {
  const StackConfig config = MakeStackCollapseConfig();
  StackState first = MakeInitialStackState(config);
  StackState second = MakeInitialStackState(config);
  for (int step = 1; step <= 20 * 480; ++step) {
    CHECK(StepStack(config, kStackPhysicsStep, &first));
    CHECK(StepStack(config, kStackPhysicsStep, &second));
  }
  CHECK(SameState(first, second));
  for (const Rectangle& box : first.boxes) {
    CHECK(std::isfinite(box.position.x) && std::isfinite(box.position.y));
    CHECK(std::isfinite(box.velocity.x) && std::isfinite(box.velocity.y));
  }
}

void TestValidationAndFailureAtomicity() {
  const auto expect_config_error = [](auto mutate) {
    StackConfig config = MakeStackReferenceConfig();
    mutate(config);
    CHECK(GetStackConfigError(config) != nullptr);
    bool threw = false;
    try {
      MakeInitialStackState(config);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    CHECK(threw);
  };

  expect_config_error([](StackConfig& c) { c.box_count = 1; });
  expect_config_error([](StackConfig& c) { c.box_count = 11; });
  expect_config_error([](StackConfig& c) { c.box_edge_m = 0.1f; });
  expect_config_error([](StackConfig& c) {
    c.box_edge_m = std::numeric_limits<float>::quiet_NaN();
  });
  expect_config_error([](StackConfig& c) { c.box_mass_kg = 0.001f; });
  expect_config_error([](StackConfig& c) { c.friction = -0.1f; });
  expect_config_error([](StackConfig& c) { c.friction = 2.5f; });
  expect_config_error([](StackConfig& c) { c.gravity_m_s2 = 0.0f; });
  expect_config_error([](StackConfig& c) { c.lateral_offset_m = -0.01f; });
  expect_config_error([](StackConfig& c) {
    // Offset beyond 40% of the edge.
    c.lateral_offset_m = 0.5f;
  });
  expect_config_error([](StackConfig& c) {
    // Ten 5 m boxes exceed the height budget.
    c.box_edge_m = 5.0f;
  });

  const StackConfig config = MakeStackReferenceConfig();
  StackState state = MakeInitialStackState(config);
  const StackState before = state;
  CHECK(!StepStack(config, 0.0f, &state));
  CHECK(!StepStack(config, -kStackPhysicsStep, &state));
  CHECK(!StepStack(config, kStackPhysicsStep * 2.0f, &state));
  CHECK(!StepStack(config, std::numeric_limits<float>::quiet_NaN(), &state));
  CHECK(!StepStack(config, kStackPhysicsStep, nullptr));
  CHECK(SameState(state, before));

  StackState tampered = state;
  tampered.boxes[0].mass = 2.0f;
  CHECK(GetStackStateError(config, tampered) != nullptr);
  CHECK(!StepStack(config, kStackPhysicsStep, &tampered));

  StackState bad_cache = state;
  bad_cache.cache.entries = {{5ull, -1.0f, 0.0f}};
  CHECK(GetStackStateError(config, bad_cache) != nullptr);
}

void TestCsvExportMetadataAndRows() {
  const StackConfig config = MakeStackReferenceConfig();
  StackState state = MakeInitialStackState(config);
  // A deliberately non-uniform history: the time column must reproduce
  // each sample's stored time, never a uniform index interval.
  std::vector<StackState> history;
  history.push_back(state);
  for (int step = 1; step <= 480; ++step) {
    CHECK(StepStack(config, kStackPhysicsStep, &state));
    if (step == 7 || step == 96 || step == 480) {
      history.push_back(state);
    }
  }

  const std::string csv =
      BuildStackCsv(config, history, "9.9.9-test", "UNIT TEST, status");
  CHECK(BuildStackCsv(config, history, "9.9.9-test", "UNIT TEST, status") ==
        csv);

  std::vector<std::string> lines;
  std::size_t start = 0;
  while (start < csv.size()) {
    const std::size_t end = csv.find('\n', start);
    lines.push_back(csv.substr(start, end - start));
    start = end + 1;
  }
  // 10 metadata lines (3 fixed + 6 params + status), the header, and one
  // row per sample.
  CHECK(lines.size() == 11 + history.size());
  CHECK(lines[0] == "# tiny2d-csv 1");
  CHECK(lines[1] == "# product_version: 9.9.9-test");
  CHECK(lines[2] == "# model: V20 StackLab");
  CHECK(lines[3] == "# param box_count: 10");
  CHECK(lines[4] == "# param box_edge_m: 1");
  CHECK(lines[5] == "# param box_mass_kg: 1");
  CHECK(lines[6] == "# param friction: 0.600000024");
  CHECK(lines[7] == "# param gravity_m_s2: 9.81000042");
  CHECK(lines[8] == "# param lateral_offset_m: 0");
  CHECK(lines[9] == "# status: UNIT TEST, status");

  const std::string& header = lines[10];
  CHECK(header.rfind("time_s,max_penetration_m,", 0) == 0);
  const auto count_cells = [](const std::string& line) {
    std::size_t cells = 1;
    for (const char character : line) {
      if (character == ',') {
        ++cells;
      }
    }
    return cells;
  };
  // 14 scalar columns plus box_count - 1 interface loads; no quoted
  // cells appear in numeric rows, so counting commas is exact.
  const std::size_t expected_columns =
      14 + static_cast<std::size_t>(config.box_count) - 1;
  CHECK(count_cells(header) == expected_columns);
  CHECK(header.find("interface_load_0_n") != std::string::npos);
  CHECK(header.find("interface_load_8_n") != std::string::npos);

  for (std::size_t i = 0; i < history.size(); ++i) {
    const std::string& row = lines[11 + i];
    CHECK(count_cells(row) == expected_columns);
    // The first cell round-trips the sample's exact stored time.
    const double parsed = std::strtod(row.c_str(), nullptr);
    CHECK(parsed == history[i].time_seconds);
  }

  // Invalid inputs reject atomically through the existing validation.
  bool threw = false;
  try {
    StackConfig bad = config;
    bad.box_count = 1;
    BuildStackCsv(bad, history, "9.9.9-test", "status");
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

void TestSeriesExtraction() {
  const StackConfig config = MakeStackReferenceConfig();
  StackState state = MakeInitialStackState(config);
  std::vector<StackState> history;
  history.push_back(state);
  for (int step = 1; step <= 240; ++step) {
    CHECK(StepStack(config, kStackPhysicsStep, &state));
    if (step % 48 == 0) {
      history.push_back(state);
    }
  }

  const StackSeries all_series[] = {
      StackSeries::kMeanSpeed,           StackSeries::kMaxSpeed,
      StackSeries::kMechanicalEnergy,    StackSeries::kPenetrationFraction,
      StackSeries::kBottomInterfaceLoad, StackSeries::kHeightError,
  };
  for (const StackSeries series : all_series) {
    // Every member is labeled with an SI unit or ratio annotation.
    const char* label = GetStackSeriesLabel(series);
    CHECK(label != nullptr && label[0] != '\0');
    const std::vector<tiny2d::sandbox::TimeSeriesPoint> points =
        ExtractStackSeries(config, history, series);
    CHECK(points.size() == history.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      CHECK(points[i].time_s == history[i].time_seconds);
      const StackDerived derived = CalculateStackDerived(config, history[i]);
      double expected = 0.0;
      switch (series) {
        case StackSeries::kMeanSpeed:
          expected = derived.mean_speed_mps;
          break;
        case StackSeries::kMaxSpeed:
          expected = derived.max_speed_mps;
          break;
        case StackSeries::kMechanicalEnergy:
          expected = derived.mechanical_energy_j;
          break;
        case StackSeries::kPenetrationFraction:
          expected = derived.max_penetration_fraction;
          break;
        case StackSeries::kBottomInterfaceLoad:
          expected = derived.interface_loads_n[0];
          break;
        case StackSeries::kHeightError:
          expected = derived.stack_height_error_m;
          break;
      }
      CHECK(points[i].value == expected);
    }
  }
  CHECK(std::string(GetStackSeriesLabel(StackSeries::kMeanSpeed)) ==
        "mean speed (m/s)");
  CHECK(std::string(GetStackSeriesLabel(StackSeries::kBottomInterfaceLoad)) ==
        "bottom interface load (N)");

  // Invalid inputs reject atomically.
  bool threw = false;
  try {
    StackConfig bad = config;
    bad.friction = -1.0f;
    ExtractStackSeries(bad, history, StackSeries::kMeanSpeed);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    ExtractStackSeries(config, history, static_cast<StackSeries>(99));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

void TestDriftSnapshotAndHistory() {
  const StackConfig config = MakeStackReferenceConfig();
  StackState state = MakeInitialStackState(config);
  CHECK(state.drift_reference_positions.empty());
  // Advance just past the snapshot time using a short synthetic run.
  state.time_seconds = kStackDriftSnapshotSeconds - kStackPhysicsStep * 2.0;
  CHECK(StepStack(config, kStackPhysicsStep, &state));
  CHECK(state.drift_reference_positions.empty());
  CHECK(StepStack(config, kStackPhysicsStep, &state));
  CHECK(state.drift_reference_positions.size() == state.boxes.size());

  std::vector<StackState> history;
  for (int i = 0; i < 3; ++i) {
    StackState sample = MakeInitialStackState(config);
    sample.time_seconds = static_cast<double>(i);
    history.push_back(sample);
  }
  CHECK(FindStackState(history, -1.0) == &history.front());
  CHECK(FindStackState(history, 0.4) == &history.front());
  CHECK(FindStackState(history, 0.6) == &history[1]);
  CHECK(FindStackState(history, 9.0) == &history.back());
  CHECK(FindStackState(history, std::numeric_limits<double>::quiet_NaN()) ==
        nullptr);
  const std::vector<StackState> empty;
  CHECK(FindStackState(empty, 1.0) == nullptr);
}

struct NamedTest {
  const char* name;
  void (*function)();
};

}  // namespace

int main() {
  std::cout << std::unitbuf;
  const std::array tests = {
      NamedTest{"presets and interface anchors",
                TestPresetsAndInterfaceAnchors},
      NamedTest{"reference stack meets roadmap criteria",
                TestReferenceStackMeetsRoadmapCriteria},
      NamedTest{"offset stack stands with bounded wobble",
                TestOffsetStackStandsWithBoundedWobble},
      NamedTest{"collapse preset stays finite and deterministic",
                TestCollapsePresetStaysFiniteAndDeterministic},
      NamedTest{"validation and failure atomicity",
                TestValidationAndFailureAtomicity},
      NamedTest{"csv export metadata and rows", TestCsvExportMetadataAndRows},
      NamedTest{"series extraction", TestSeriesExtraction},
      NamedTest{"drift snapshot and history lookup",
                TestDriftSnapshotAndHistory},
  };
  for (const NamedTest& test : tests) {
    test.function();
    std::cout << "[PASS] " << test.name << '\n';
  }
  std::cout << tests.size() << " tests, " << tiny2d::test::CheckCount()
            << " checks passed\n";
  return 0;
}
