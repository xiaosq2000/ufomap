cmake_minimum_required(VERSION 3.24)
project(ufomap_smoke CXX)

# ---------------------------------------------------------------------------
# Standalone smoke-test build for the UFOMap v2 "Map" component.
#
# The Map component is normally built inside a parent UFO superproject that
# provides the sibling UFO::* targets. This harness lets it build on its own so
# the maps can be smoke-tested in a minimal (CPU-only) environment:
#
#   * Fetches the sibling UFO components (header-only), pinned to the commits
#     that match ufomap v2's era (2025-06). They are downloaded but NOT
#     configured (SOURCE_SUBDIR points at a path with no CMakeLists.txt), so
#     their install/export rules and the forced-GPU `ufocompute` shared library
#     never run -- we only need their headers.
#
#   * Compiles all GPU code out via the ../compat/no_gpu shim, which shadows
#     <ufo/compute/compute.hpp> so UFO_WEBGPU is never defined. No WebGPU /
#     wgpu-native headers or libraries are required.
#
#   * Builds the Catch2 smoke tests in ../tests.
#
# Build it via the pixi tasks at the repo root (`pixi run smoke`) or directly:
#   cmake -G Ninja -S smoke -B build && cmake --build build && ctest --test-dir build
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_filename_component(UFOMAP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

include(FetchContent)

# A subdir that intentionally does not exist in any component -> FetchContent
# downloads the sources without calling add_subdirectory() on them.
set(_UFO_DOWNLOAD_ONLY "cmake/_smoke_download_only")

FetchContent_Declare(ufocore
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocore.git
  GIT_TAG e77e32e8866f6deb1ee6f729bfa47cc142320d05 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufoutility
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufoutility.git
  GIT_TAG a9f44ea2c76d95a583b08df2d7a27131588eaa2e SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufoexecution
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufoexecution.git
  GIT_TAG 7e176fd4892adec3097841f336ca662a52210d61 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufomath
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomath.git
  GIT_TAG 8f6923ee4356010b71f2f10631f2aac640e7599b SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufomorton
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomorton.git
  GIT_TAG 13c2d4fa2d1710b38453da3038ef398b039a0911 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufocompute
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocompute.git
  GIT_TAG 89456c773131861c3ac0635f25762e6855554973 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufogeometry
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufogeometry.git
  GIT_TAG 26cca9cbe9fc19ce5e8c3e25d2ae192826425167 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufovision
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufovision.git
  GIT_TAG 54cadeaac15abea0cb2dca76045c60678a686934 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufocontainer
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocontainer.git
  GIT_TAG 297c4cccf4f08dbf8130964ef1b252c3dd17c3ea SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufotime
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufotime.git
  GIT_TAG 12c7115175e4a0c84abc2644909ed4398ae80079 SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})
FetchContent_Declare(ufocloud
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocloud.git
  GIT_TAG 72950b3ec4d15a1b886d94d56dd365b8143dedab SOURCE_SUBDIR ${_UFO_DOWNLOAD_ONLY})

FetchContent_MakeAvailable(
  ufocore ufoutility ufoexecution ufomath ufomorton ufocompute
  ufogeometry ufovision ufocontainer ufotime ufocloud)

set(UFO_INCLUDE_DIRS
  ${ufocore_SOURCE_DIR}/include
  ${ufoutility_SOURCE_DIR}/include
  ${ufoexecution_SOURCE_DIR}/include
  ${ufomath_SOURCE_DIR}/include
  ${ufomorton_SOURCE_DIR}/include
  ${ufocompute_SOURCE_DIR}/include
  ${ufogeometry_SOURCE_DIR}/include
  ${ufovision_SOURCE_DIR}/include
  ${ufocontainer_SOURCE_DIR}/include
  ${ufotime_SOURCE_DIR}/include
  ${ufocloud_SOURCE_DIR}/include)

find_package(Catch2 3 REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB QUIET)  # libstdc++ <execution> needs the TBB backend to link

enable_testing()

set(UFO_SMOKE_TESTS occupancy_map_test labels_map_test integration_test)
foreach(t IN LISTS UFO_SMOKE_TESTS)
  add_executable(${t} ${UFOMAP_ROOT}/tests/${t}.cpp)
  target_compile_features(${t} PRIVATE cxx_std_20)
  target_include_directories(${t} PRIVATE
    ${UFOMAP_ROOT}/compat/no_gpu  # GPU-off shim -- must precede ufocompute's include
    ${UFOMAP_ROOT}/include
    ${UFO_INCLUDE_DIRS})
  target_link_libraries(${t} PRIVATE Catch2::Catch2WithMain Threads::Threads)
  if(TBB_FOUND)
    target_link_libraries(${t} PRIVATE TBB::tbb)
  else()
    target_link_libraries(${t} PRIVATE tbb)
  endif()
  # Select the TBB backend for ufo::execution policies. ufoexecution's own CMake
  # normally sets this from the found deps; the smoke harness uses the headers
  # directly, so we set it here (the pixi env provides oneTBB).
  target_compile_definitions(${t} PRIVATE UFO_PAR_TBB=1)
  add_test(NAME ${t} COMMAND ${t})
endforeach()
