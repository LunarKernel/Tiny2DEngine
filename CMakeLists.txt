cmake_minimum_required(VERSION 3.24)

file(READ "${CMAKE_CURRENT_LIST_DIR}/vcpkg.json" TINY2D_VCPKG_MANIFEST)
string(JSON TINY2D_PRODUCT_VERSION GET "${TINY2D_VCPKG_MANIFEST}"
       version-semver)

project(Tiny2DEngine VERSION "${TINY2D_PRODUCT_VERSION}" LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

option(TINY2D_BUILD_SANDBOX "Build the interactive Sandbox application" ON)
option(TINY2D_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(TINY2D_ENABLE_ASAN "Enable AddressSanitizer" OFF)

if(TINY2D_ENABLE_ASAN)
  if(MSVC)
    add_compile_options(/fsanitize=address)
    add_link_options(/INCREMENTAL:NO)
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
  else()
    message(FATAL_ERROR "AddressSanitizer is unsupported by this compiler")
  endif()
endif()

function(tiny2d_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8
                                             /Zc:__cplusplus)
    if(TINY2D_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(TINY2D_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

if(TINY2D_BUILD_SANDBOX)
  find_package(SDL2 CONFIG REQUIRED)
  find_package(imgui CONFIG REQUIRED)
endif()

add_library(Tiny2DEngine STATIC
    Engine/internal/constraints.cc
    Engine/internal/contacts.cc
    Engine/internal/solver.cc
    Engine/internal/validation.cc
    Engine/internal/warm_contacts.cc
    Engine/tiny2d_engine.cc
)

target_include_directories(Tiny2DEngine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Engine
)

target_compile_features(Tiny2DEngine PUBLIC cxx_std_17)
tiny2d_enable_warnings(Tiny2DEngine)

if(TINY2D_BUILD_SANDBOX OR BUILD_TESTING)
  add_library(Tiny2DSimulationModels STATIC
      Sandbox/atwood_lab_model.cc
      Sandbox/chaos_lab_model.cc
      Sandbox/contact_lab_model.cc
      Sandbox/csv_export.cc
      Sandbox/experiment_file.cc
      Sandbox/force_lab_model.cc
      Sandbox/impact_lab_model.cc
      Sandbox/incline_spring_model.cc
      Sandbox/lorentz_particle_model.cc
      Sandbox/rolling_disk_model.cc
      Sandbox/rotation_pendulum_model.cc
      Sandbox/stack_lab_model.cc
      Sandbox/time_series.cc
  )

  target_include_directories(Tiny2DSimulationModels PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/Sandbox
  )

  target_link_libraries(Tiny2DSimulationModels PUBLIC Tiny2DEngine)
  target_compile_features(Tiny2DSimulationModels PUBLIC cxx_std_17)
  tiny2d_enable_warnings(Tiny2DSimulationModels)
endif()

if(TINY2D_BUILD_SANDBOX)
  add_executable(Sandbox WIN32
      Sandbox/atwood_lab_simulation.cc
      Sandbox/chaos_lab_simulation.cc
      Sandbox/contact_lab_simulation.cc
      Sandbox/force_lab_simulation.cc
      Sandbox/impact_lab_simulation.cc
      Sandbox/main.cc
      Sandbox/incline_spring_simulation.cc
      Sandbox/lorentz_particle_simulation.cc
      Sandbox/rolling_disk_simulation.cc
      Sandbox/rotation_pendulum_simulation.cc
      Sandbox/stack_lab_simulation.cc
  )

  target_compile_definitions(Sandbox PRIVATE
      TINY2D_PRODUCT_VERSION="${PROJECT_VERSION}"
  )

  target_link_libraries(Sandbox PRIVATE
      SDL2::SDL2main
      SDL2::SDL2
      Tiny2DSimulationModels
      imgui::imgui
  )
  tiny2d_enable_warnings(Sandbox)
endif()

if(BUILD_TESTING)
  add_library(Tiny2DTestSupport INTERFACE)
  target_include_directories(Tiny2DTestSupport INTERFACE
      ${CMAKE_CURRENT_SOURCE_DIR}/tests
  )

  add_executable(Tiny2DEngineTests
      Engine/tiny2d_engine_test.cc
  )

  target_link_libraries(Tiny2DEngineTests PRIVATE Tiny2DEngine
                                                  Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DEngineTests)

  add_test(NAME Tiny2DEngineTests COMMAND Tiny2DEngineTests)
  set_tests_properties(Tiny2DEngineTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DSimulationTests
      Sandbox/simulation_model_test.cc
  )

  target_link_libraries(Tiny2DSimulationTests PRIVATE Tiny2DSimulationModels
                                                      Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DSimulationTests)

  add_test(NAME Tiny2DSimulationTests COMMAND Tiny2DSimulationTests)
  set_tests_properties(Tiny2DSimulationTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DRotationPendulumTests
      Sandbox/rotation_pendulum_test.cc
  )

  target_link_libraries(Tiny2DRotationPendulumTests
      PRIVATE Tiny2DSimulationModels Tiny2DTestSupport
  )
  tiny2d_enable_warnings(Tiny2DRotationPendulumTests)

  add_test(NAME Tiny2DRotationPendulumTests COMMAND Tiny2DRotationPendulumTests)
  set_tests_properties(Tiny2DRotationPendulumTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DRollingDiskTests
      Sandbox/rolling_disk_test.cc
  )

  target_link_libraries(Tiny2DRollingDiskTests PRIVATE Tiny2DSimulationModels
                                                       Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DRollingDiskTests)

  add_test(NAME Tiny2DRollingDiskTests COMMAND Tiny2DRollingDiskTests)
  set_tests_properties(Tiny2DRollingDiskTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DLorentzParticleTests
      Sandbox/lorentz_particle_test.cc
  )

  target_link_libraries(Tiny2DLorentzParticleTests
      PRIVATE Tiny2DSimulationModels Tiny2DTestSupport
  )
  tiny2d_enable_warnings(Tiny2DLorentzParticleTests)

  add_test(NAME Tiny2DLorentzParticleTests COMMAND Tiny2DLorentzParticleTests)
  set_tests_properties(Tiny2DLorentzParticleTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DForceLabTests
      Sandbox/force_lab_test.cc
  )

  target_link_libraries(Tiny2DForceLabTests PRIVATE Tiny2DSimulationModels
                                                    Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DForceLabTests)

  add_test(NAME Tiny2DForceLabTests COMMAND Tiny2DForceLabTests)
  set_tests_properties(Tiny2DForceLabTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DContactLabTests
      Sandbox/contact_lab_test.cc
  )

  target_link_libraries(Tiny2DContactLabTests PRIVATE Tiny2DSimulationModels
                                                      Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DContactLabTests)

  add_test(NAME Tiny2DContactLabTests COMMAND Tiny2DContactLabTests)
  set_tests_properties(Tiny2DContactLabTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DImpactLabTests
      Sandbox/impact_lab_test.cc
  )

  target_link_libraries(Tiny2DImpactLabTests PRIVATE Tiny2DSimulationModels
                                                     Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DImpactLabTests)

  add_test(NAME Tiny2DImpactLabTests COMMAND Tiny2DImpactLabTests)
  set_tests_properties(Tiny2DImpactLabTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DAtwoodLabTests
      Sandbox/atwood_lab_test.cc
  )

  target_link_libraries(Tiny2DAtwoodLabTests PRIVATE Tiny2DSimulationModels
                                                     Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DAtwoodLabTests)

  add_test(NAME Tiny2DAtwoodLabTests COMMAND Tiny2DAtwoodLabTests)
  set_tests_properties(Tiny2DAtwoodLabTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DChaosLabTests
      Sandbox/chaos_lab_test.cc
  )

  target_link_libraries(Tiny2DChaosLabTests PRIVATE Tiny2DSimulationModels
                                                    Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DChaosLabTests)

  add_test(NAME Tiny2DChaosLabTests COMMAND Tiny2DChaosLabTests)
  # The chaos suite advances 32 engine substeps per public step for two
  # runs (53 s measured in Debug), so it needs far more headroom than the
  # other suites, especially under AddressSanitizer.
  set_tests_properties(Tiny2DChaosLabTests PROPERTIES TIMEOUT 180)

  add_executable(Tiny2DTimeSeriesTests
      Sandbox/time_series_test.cc
  )

  target_link_libraries(Tiny2DTimeSeriesTests PRIVATE Tiny2DSimulationModels
                                                      Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DTimeSeriesTests)

  add_test(NAME Tiny2DTimeSeriesTests COMMAND Tiny2DTimeSeriesTests)
  set_tests_properties(Tiny2DTimeSeriesTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DExperimentFileTests
      Sandbox/experiment_file_test.cc
  )

  target_link_libraries(Tiny2DExperimentFileTests
      PRIVATE Tiny2DSimulationModels Tiny2DTestSupport
  )
  tiny2d_enable_warnings(Tiny2DExperimentFileTests)

  add_test(NAME Tiny2DExperimentFileTests COMMAND Tiny2DExperimentFileTests)
  set_tests_properties(Tiny2DExperimentFileTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DCsvExportTests
      Sandbox/csv_export_test.cc
  )

  target_link_libraries(Tiny2DCsvExportTests PRIVATE Tiny2DSimulationModels
                                                     Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DCsvExportTests)

  add_test(NAME Tiny2DCsvExportTests COMMAND Tiny2DCsvExportTests)
  set_tests_properties(Tiny2DCsvExportTests PROPERTIES TIMEOUT 30)

  add_executable(Tiny2DStackLabTests
      Sandbox/stack_lab_test.cc
  )

  target_link_libraries(Tiny2DStackLabTests PRIVATE Tiny2DSimulationModels
                                                    Tiny2DTestSupport)
  tiny2d_enable_warnings(Tiny2DStackLabTests)

  add_test(NAME Tiny2DStackLabTests COMMAND Tiny2DStackLabTests)
  # Three warm-started 60 s stack runs (28 s measured in Debug); the
  # AddressSanitizer factor exceeded 3x and timed out at 90, so match the
  # chaos suite's headroom.
  set_tests_properties(Tiny2DStackLabTests PROPERTIES TIMEOUT 180)
endif()
