cmake_minimum_required(VERSION 3.16)

file(TO_CMAKE_PATH "$ENV{VSINSTALLDIR}" _vs_install_dir)
set(_vcpkg_toolchain "${_vs_install_dir}/VC/vcpkg/scripts/buildsystems/vcpkg.cmake")
if(NOT EXISTS "${_vcpkg_toolchain}")
    set(_vcpkg_toolchain "C:/Program Files/Microsoft Visual Studio/18/Community/VC/vcpkg/scripts/buildsystems/vcpkg.cmake")
endif()
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "${_vcpkg_toolchain}")
    # ponytail: Visual Studio legacy-mode fallback; presets remain the portable entry point.
    set(CMAKE_TOOLCHAIN_FILE "${_vcpkg_toolchain}" CACHE FILEPATH "vcpkg toolchain file")
endif()
unset(_vcpkg_toolchain)
unset(_vs_install_dir)

project(Tiny2DEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

function(tiny2d_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

find_package(SDL2 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_library(Tiny2DEngine STATIC
    Engine/tiny2d_engine.cc
)

target_include_directories(Tiny2DEngine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Engine
)

target_link_libraries(Tiny2DEngine PUBLIC SDL2::SDL2)
tiny2d_enable_warnings(Tiny2DEngine)

add_executable(Sandbox WIN32
    Sandbox/main.cc
)

target_link_libraries(Sandbox PRIVATE
    SDL2::SDL2main
    Tiny2DEngine
    imgui::imgui
)
tiny2d_enable_warnings(Sandbox)

enable_testing()

add_executable(Tiny2DEngineTests
    Engine/tiny2d_engine_test.cc
)

target_link_libraries(Tiny2DEngineTests PRIVATE Tiny2DEngine)
tiny2d_enable_warnings(Tiny2DEngineTests)

add_test(NAME Tiny2DEngineTests COMMAND Tiny2DEngineTests)
set_tests_properties(Tiny2DEngineTests PROPERTIES TIMEOUT 30)

# Test the exact headless model helpers used by the SDL application. The test
# translation unit includes main.cc with its entry point renamed, avoiding a
# second copy of the calibration and transition formulas.
add_executable(Tiny2DSimulationTests
    Sandbox/simulation_model_test.cc
)

target_link_libraries(Tiny2DSimulationTests PRIVATE
    SDL2::SDL2
    Tiny2DEngine
    imgui::imgui
)
tiny2d_enable_warnings(Tiny2DSimulationTests)

add_test(NAME Tiny2DSimulationTests COMMAND Tiny2DSimulationTests)
set_tests_properties(Tiny2DSimulationTests PROPERTIES TIMEOUT 30)
