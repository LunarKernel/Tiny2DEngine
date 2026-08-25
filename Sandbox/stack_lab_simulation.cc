#include <SDL.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "app/lab_shell.h"
#include "experiment_file.h"
#include "plot_ui.h"
#include "sim_ui.h"
#include "simulations.h"
#include "stack_lab_model.h"

namespace tiny2d::sandbox {
namespace {

constexpr float kControlStart = 310.0f;
constexpr float kSliderWidth = 310.0f;
constexpr float kInputWidth = 110.0f;

constexpr StackSeries kStackSeriesOrder[] = {
    StackSeries::kMeanSpeed,           StackSeries::kMaxSpeed,
    StackSeries::kMechanicalEnergy,    StackSeries::kPenetrationFraction,
    StackSeries::kBottomInterfaceLoad, StackSeries::kHeightError,
};
constexpr int kStackSeriesCount =
    static_cast<int>(sizeof(kStackSeriesOrder) / sizeof(kStackSeriesOrder[0]));

// Rebuilds the cached extraction only when the selection or the
// recorded history changed (see SeriesCache).
const std::vector<TimeSeriesPoint>& RefreshStackSeries(
    const StackConfig& config, const std::vector<StackState>& history,
    int series_index, ui::SeriesCache* cache) {
  const double last_time = history.empty() ? -1.0 : history.back().time_seconds;
  if (cache->NeedsRebuild(series_index, history.size(), last_time)) {
    cache->points =
        ExtractStackSeries(config, history, kStackSeriesOrder[series_index]);
    cache->MarkRebuilt(series_index, history.size(), last_time);
  }
  return cache->points;
}

void DrawStackTimeSeries(const StackConfig& config,
                         const std::vector<StackState>& history,
                         double inspect_time, ui::TimeSeriesWindowState* state,
                         ui::SeriesCache* primary_cache,
                         ui::SeriesCache* secondary_cache) {
  const char* labels[kStackSeriesCount];
  for (int i = 0; i < kStackSeriesCount; ++i) {
    labels[i] = GetStackSeriesLabel(kStackSeriesOrder[i]);
  }
  const std::vector<TimeSeriesPoint>& primary =
      RefreshStackSeries(config, history, state->primary_index, primary_cache);
  const std::vector<TimeSeriesPoint>* secondary = nullptr;
  if (state->compare) {
    secondary = &RefreshStackSeries(config, history, state->secondary_index,
                                    secondary_cache);
  }
  ui::DrawTimeSeriesWindow("StackLab time series", labels, kStackSeriesCount,
                           state, primary, secondary, inspect_time);
}

// Filename input and result line of the setup screen's experiment
// loader; lives in the traits object so it persists across frames.
struct ExperimentLoadUi {
  char filename[128] = "stack_lab_";
  std::string status;
};

void DrawExperimentLoadRow(StackConfig* config, ExperimentLoadUi* load_ui) {
  ImGui::SetNextItemWidth(300.0f);
  ImGui::InputText("##experiment_file", load_ui->filename,
                   sizeof(load_ui->filename));
  ImGui::SameLine();
  if (ImGui::Button("Load experiment")) {
    const std::string name = load_ui->filename;
    std::string text;
    std::string error;
    StackConfig loaded;
    StackCheckpoint checkpoint;
    std::string file_version;
    if (name.empty()) {
      load_ui->status = "Enter a file name.";
    } else if (name.find_first_of("/\\") != std::string::npos) {
      load_ui->status = "File names must not contain path separators.";
    } else if (!ReadTextFile(name, &text, &error)) {
      load_ui->status = "Load failed: " + error;
    } else if (!LoadStackExperiment(text, &loaded, &checkpoint, &file_version,
                                    &error)) {
      load_ui->status = "Load failed: " + error;
    } else {
      *config = loaded;
      char summary[160];
      std::snprintf(summary, sizeof(summary), "Loaded %s (checkpoint t=%.4f s)",
                    name.c_str(), checkpoint.time_s);
      load_ui->status = summary;
      if (file_version != TINY2D_PRODUCT_VERSION) {
        load_ui->status +=
            " | file from product version " +
            (file_version.empty() ? std::string("(unknown)") : file_version);
      }
    }
  }
  if (!load_ui->status.empty()) {
    ImGui::TextWrapped("%s", load_ui->status.c_str());
  } else {
    ImGui::TextDisabled("Load a saved .exp file from the working directory.");
  }
}

shell::SetupAction DrawSetupScreen(StackConfig* config,
                                   ExperimentLoadUi* load_ui) {
  ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos({0.0f, 0.0f});
  ImGui::SetNextWindowSize(io.DisplaySize);
  constexpr ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_NoDecoration |
                                            ImGuiWindowFlags_NoMove |
                                            ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("StackLab setup", nullptr, kWindowFlags);

  ImGui::TextColored({0.35f, 0.75f, 1.0f, 1.0f},
                     "V20 StackLab: persistent contact stability");
  ImGui::TextDisabled(
      "SI units. A box stack rests on the floor through the engine's "
      "warm-starting contact cache; telemetry shows the ROADMAP criteria "
      "live.");
  ImGui::Spacing();

  ImGui::BeginChild("##stack_parameters", {0.0f, -150.0f}, false);
  ImGui::TextUnformatted("Stack");
  ImGui::Separator();
  int box_count = config->box_count;
  ImGui::SetNextItemWidth(kSliderWidth);
  if (ImGui::SliderInt("Box count", &box_count, kStackMinimumBoxes,
                       kStackMaximumBoxes)) {
    config->box_count = box_count;
  }
  ui::SliderInputFloat("Box edge (m)", &config->box_edge_m, kStackMinimumEdgeM,
                       kStackMaximumEdgeM, "%.3f", kControlStart, kSliderWidth,
                       kInputWidth);
  ui::SliderInputFloat("Box mass (kg)", &config->box_mass_kg,
                       kStackMinimumMassKg, kStackMaximumMassKg, "%.4g",
                       kControlStart, kSliderWidth, kInputWidth,
                       ImGuiSliderFlags_Logarithmic);
  ui::SliderInputFloat("Lateral offset (m)", &config->lateral_offset_m, 0.0f,
                       0.4f * config->box_edge_m, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);

  ImGui::Spacing();
  ImGui::TextUnformatted("Environment");
  ImGui::Separator();
  ui::SliderInputFloat("Friction", &config->friction, 0.0f,
                       kStackMaximumFriction, "%.3f", kControlStart,
                       kSliderWidth, kInputWidth);
  ui::SliderInputFloat("Gravity g (m/s^2)", &config->gravity_m_s2,
                       kStackMinimumGravityMps2, kStackMaximumGravityMps2,
                       "%.3f", kControlStart, kSliderWidth, kInputWidth);
  ImGui::TextDisabled(
      "Solver: 16 iterations, warm-starting cache, slop min(2 mm, 0.2%% of "
      "the edge).");
  ImGui::EndChild();

  DrawExperimentLoadRow(config, load_ui);

  const char* error = GetStackConfigError(*config);
  if (error == nullptr) {
    ImGui::TextColored({0.35f, 0.85f, 0.45f, 1.0f},
                       "Ready | bottom interface load target: %.3f N",
                       static_cast<double>(config->box_count - 1) *
                           config->box_mass_kg * config->gravity_m_s2);
  } else {
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", error);
  }

  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float button_width =
      (ImGui::GetContentRegionAvail().x - 4.0f * spacing) / 5.0f;
  shell::SetupAction action = shell::SetupAction::kNone;
  if (ImGui::Button("Back", {button_width, 38.0f})) {
    action = shell::SetupAction::kBack;
  }
  ImGui::SameLine();
  if (ImGui::Button("Reference stack", {button_width, 38.0f})) {
    *config = MakeStackReferenceConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Offset stack", {button_width, 38.0f})) {
    *config = MakeStackOffsetConfig();
  }
  ImGui::SameLine();
  if (ImGui::Button("Collapse demo", {button_width, 38.0f})) {
    *config = MakeStackCollapseConfig();
  }
  ImGui::SameLine();
  ImGui::BeginDisabled(error != nullptr);
  if (ImGui::Button("Start", {button_width, 38.0f})) {
    action = shell::SetupAction::kStart;
  }
  ImGui::EndDisabled();

  ImGui::End();
  return action;
}

void DrawStackScene(const StackConfig& config, const StackState& state) {
  ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
  const ImVec2 display_size = ImGui::GetIO().DisplaySize;
  const float scene_left = std::min(450.0f, display_size.x * 0.42f);
  const float scene_width = std::max(display_size.x - scene_left, 200.0f);
  const float floor_screen_y = display_size.y * 0.92f;
  const float stack_height =
      static_cast<float>(config.box_count) * config.box_edge_m;
  const float pixels_per_meter = std::max(
      4.0f, (display_size.y * 0.80f) / std::max(stack_height + 1.0f, 2.0f));
  const float center_x = scene_left + scene_width * 0.5f;

  const auto to_screen = [&](Vec2 world) {
    return ImVec2{
        center_x + (world.x - kStackCenterXM) * pixels_per_meter,
        floor_screen_y - (kStackAreaHeightM - world.y) * pixels_per_meter};
  };

  // Floor line.
  draw_list->AddLine({scene_left + 16.0f, floor_screen_y},
                     {display_size.x - 16.0f, floor_screen_y},
                     IM_COL32(200, 210, 225, 255), 3.0f);

  for (const Rectangle& box : state.boxes) {
    const float speed = std::hypot(box.velocity.x, box.velocity.y);
    // Resting boxes render cool blue; motion shifts them toward orange.
    const float heat = std::clamp(speed / 0.5f, 0.0f, 1.0f);
    const ImU32 fill = IM_COL32(static_cast<int>(80.0f + 165.0f * heat),
                                static_cast<int>(140.0f - 40.0f * heat),
                                static_cast<int>(230.0f - 160.0f * heat), 255);
    const float half = box.width * 0.5f * pixels_per_meter;
    const ImVec2 center = to_screen(box.position);
    const float cosine = std::cos(box.angle);
    const float sine = std::sin(box.angle);
    const auto corner = [&](float sx, float sy) {
      return ImVec2{center.x + (sx * cosine - sy * sine) * 1.0f,
                    center.y + (sx * sine + sy * cosine) * 1.0f};
    };
    const ImVec2 p0 = corner(-half, -half);
    const ImVec2 p1 = corner(half, -half);
    const ImVec2 p2 = corner(half, half);
    const ImVec2 p3 = corner(-half, half);
    draw_list->AddQuadFilled(p0, p1, p2, p3, fill);
    draw_list->AddQuad(p0, p1, p2, p3, IM_COL32(230, 240, 250, 255), 1.5f);
  }

  char footer[96];
  std::snprintf(footer, sizeof(footer),
                "%d boxes x %.3g m | friction %.2f | offset %.3g m",
                config.box_count, config.box_edge_m, config.friction,
                config.lateral_offset_m);
  draw_list->AddText({scene_left + 24.0f, display_size.y - 40.0f},
                     IM_COL32(245, 205, 115, 255), footer);
}

void DrawCriterion(const char* label, bool ok, double value, const char* unit) {
  ImGui::TextColored(
      ok ? ImVec4{0.35f, 0.85f, 0.45f, 1.0f} : ImVec4{1.0f, 0.5f, 0.3f, 1.0f},
      "%s %s: %.3e %s", ok ? "[PASS]" : "[....]", label, value, unit);
}

bool DrawMonitor(const StackConfig& config, const StackState& current_state,
                 const std::vector<StackState>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& runtime_error, StackState* displayed_state,
                 std::string* export_status) {
  ImGui::SetNextWindowPos({12.0f, 12.0f});
  ImGui::SetNextWindowSize({430.0f, 760.0f});
  constexpr ImGuiWindowFlags kWindowFlags =
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  ImGui::Begin("StackLab monitor", nullptr, kWindowFlags);

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
  constexpr double kMinimumInspectTime = 0.0;
  const double maximum_inspect_time = std::max(
      current_state.time_seconds, static_cast<double>(kStackPhysicsStep));
  ImGui::SetNextItemWidth(140.0f);
  bool inspect_changed = ImGui::SliderScalar(
      "##stack_history_slider", ImGuiDataType_Double, inspect_time,
      &kMinimumInspectTime, &maximum_inspect_time, "%.3f",
      ImGuiSliderFlags_AlwaysClamp);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(70.0f);
  inspect_changed |= ImGui::InputDouble("##stack_history_input", inspect_time,
                                        0.0, 0.0, "%.3f");
  if (inspect_changed) {
    if (!std::isfinite(*inspect_time)) {
      *inspect_time = current_state.time_seconds;
    }
    *inspect_time = std::clamp(*inspect_time, 0.0, current_state.time_seconds);
    *follow_live = false;
  }

  const StackState* inspected = FindStackState(history, *inspect_time);
  if (inspected == nullptr) {
    inspected = &current_state;
  }
  *displayed_state = *inspected;
  const StackDerived derived = CalculateStackDerived(config, *inspected);

  ImGui::TextUnformatted("ROADMAP criteria (live)");
  ImGui::Separator();
  DrawCriterion("penetration / edge", derived.penetration_ok,
                derived.max_penetration_fraction, "(< 5.0e-3)");
  DrawCriterion("max speed", derived.resting_speed_ok, derived.max_speed_mps,
                "m/s (< 1e-3)");
  DrawCriterion("drift since 50 s", derived.drift_ok, derived.max_drift_m,
                "m (< 1e-3 * edge)");
  DrawCriterion("energy vs start", derived.energy_ok,
                derived.mechanical_energy_j, "J (<= 0.1% scale)");
  DrawCriterion("height error", derived.height_ok, derived.stack_height_error_m,
                "m (within budget)");
  ImGui::TextDisabled(derived.max_drift_m == 0.0 &&
                              inspected->time_seconds < 50.0
                          ? "(drift snapshot arms at t = 50 s)"
                          : " ");

  ImGui::Spacing();
  ImGui::TextUnformatted("Stack state");
  ImGui::Separator();
  ImGui::Text("penetration: %.3e m absolute", derived.max_penetration_m);
  ImGui::Text("mean speed: %.3e m/s", derived.mean_speed_mps);
  ImGui::Text("max tilt: %.3e rad", derived.max_tilt_rad);
  ImGui::Text("max angular speed: %.3e rad/s", derived.max_angular_speed_rad_s);
  ImGui::Text("kinetic energy: %.6f J", derived.kinetic_energy_j);
  ImGui::Text("cache: %d contacts, %d warm-started",
              derived.cache_contact_count, derived.cache_warm_started_count);

  ImGui::Spacing();
  ImGui::TextUnformatted("Interface loads vs (n-k) m g");
  ImGui::Separator();
  for (std::size_t i = 0; i < derived.interface_loads_n.size(); ++i) {
    ImGui::Text("interface %zu: %8.3f N (target %8.3f N)", i,
                derived.interface_loads_n[i], derived.analytical_loads_n[i]);
  }

  if (!runtime_error.empty()) {
    ImGui::Spacing();
    ImGui::TextColored({1.0f, 0.35f, 0.35f, 1.0f}, "%s", runtime_error.c_str());
  }

  const float width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  ImGui::BeginDisabled(!runtime_error.empty());
  if (ImGui::Button(*paused ? "Resume" : "Pause", {width, 36.0f})) {
    *paused = !*paused;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  const bool stop = ImGui::Button("Stop and choose model", {width, 36.0f});

  ImGui::BeginDisabled(history.empty());
  const float export_width =
      (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
      0.5f;
  if (ImGui::Button("Export CSV", {export_width, 30.0f})) {
    try {
      const StackDerived live = CalculateStackDerived(config, current_state);
      const int criteria_passing =
          (live.penetration_ok ? 1 : 0) + (live.resting_speed_ok ? 1 : 0) +
          (live.drift_ok ? 1 : 0) + (live.energy_ok ? 1 : 0) +
          (live.height_ok ? 1 : 0);
      char status[64];
      std::snprintf(
          status, sizeof(status), "%s t=%.4f s | criteria %d/5",
          runtime_error.empty() ? (*paused ? "PAUSED" : "RUNNING") : "ERROR",
          current_state.time_seconds, criteria_passing);
      std::string full_status = status;
      if (!runtime_error.empty()) {
        full_status += " | " + runtime_error;
      }
      *export_status = ui::ExportCsvToWorkingDirectory(
          "stack_lab",
          BuildStackCsv(config, history, TINY2D_PRODUCT_VERSION, full_status));
    } catch (const std::exception& exception) {
      *export_status = std::string("Export failed: ") + exception.what();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Save experiment", {export_width, 30.0f})) {
    try {
      // The checkpoint captures the LIVE state, never the inspect
      // cursor's displayed historical state.
      *export_status = ui::WriteExportToWorkingDirectory(
          "stack_lab",
          SaveStackExperiment(config, current_state, TINY2D_PRODUCT_VERSION),
          "Saved", "Save failed: ", MakeExperimentFileName);
    } catch (const std::exception& exception) {
      *export_status = std::string("Save failed: ") + exception.what();
    }
  }
  ImGui::EndDisabled();
  if (!export_status->empty()) {
    ImGui::TextWrapped("%s", export_status->c_str());
  }

  ImGui::End();
  return stop;
}

struct StackLabTraits {
  using Config = StackConfig;
  using State = StackState;
  static constexpr float kPhysicsStep = kStackPhysicsStep;
  static constexpr double kMaximumFrameTime = 0.25;
  static constexpr const char* kInvalidHistoryTimeMessage =
      "StackLab produced an invalid history time.";
  static constexpr const char* kNonIncreasingHistoryTimeMessage =
      "StackLab history time stopped increasing.";

  Config MakeInitialConfig() { return MakeStackReferenceConfig(); }
  State MakeState(const Config& config) {
    // Pins the fresh-per-run invariant: today each lab entry
    // constructs new traits, so this reset has no observable effect;
    // it guards any future refactor that reuses a traits object.
    primary_cache_.Reset();
    secondary_cache_.Reset();
    plot_state_.ResetRunState();
    return MakeInitialStackState(config);
  }
  const char* InitialStateIssue(const Config&, const State&) { return nullptr; }
  bool CanStep(const State&) { return true; }
  bool Step(const Config& config, State* state) {
    return StepStack(config, kPhysicsStep, state);
  }
  std::string StepFailureMessage(const Config& config, const State& state,
                                 std::vector<State>*) {
    const char* error = GetStackStateError(config, state);
    return error != nullptr ? error
                            : "StackLab rejected an unstable fixed step.";
  }
  const char* AfterStepIssue(const State&) { return nullptr; }
  shell::SetupAction DrawSetup(Config* config, const std::string&) {
    return DrawSetupScreen(config, &load_ui_);
  }
  bool DrawFrame(const Config& config, const State& state,
                 const std::vector<State>& history, bool* paused,
                 double* inspect_time, bool* follow_live,
                 const std::string& error) {
    State displayed_state = state;
    const bool stop =
        DrawMonitor(config, state, history, paused, inspect_time, follow_live,
                    error, &displayed_state, &export_status_);
    DrawStackScene(config, displayed_state);
    DrawStackTimeSeries(config, history, *inspect_time, &plot_state_,
                        &primary_cache_, &secondary_cache_);
    return stop;
  }

  // Result line of the last Export CSV or Save experiment click;
  // persists across frames.
  std::string export_status_;
  // Time-series window selection and extraction caches.
  ui::TimeSeriesWindowState plot_state_;
  ui::SeriesCache primary_cache_;
  ui::SeriesCache secondary_cache_;
  // Setup-screen experiment loader state.
  ExperimentLoadUi load_ui_;
};

}  // namespace

SimulationResult RunStackLabSimulation(SDL_Renderer* renderer) {
  return shell::RunLab(renderer, StackLabTraits{});
}

}  // namespace tiny2d::sandbox
