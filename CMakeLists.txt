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

find_package(SDL2 CONFIG REQUIRED)

add_library(Tiny2DEngine STATIC
    Engine/Tiny2DEngine.cpp
)

target_include_directories(Tiny2DEngine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Engine
)

target_link_libraries(Tiny2DEngine PUBLIC SDL2::SDL2)

add_executable(Sandbox
    Sandbox/main.cpp
)

target_link_libraries(Sandbox PRIVATE
    SDL2::SDL2main
    Tiny2DEngine
)
