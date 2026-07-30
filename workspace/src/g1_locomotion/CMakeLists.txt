cmake_minimum_required(VERSION 3.8)
project(g1_locomotion)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(unitree_api REQUIRED)

# Pure, ROS-free wire-contract logic -- unit-tested directly with no rclcpp node needed (mirrors
# g1_hardware_interface's arm_ramp_engine and g1_bringup's blend_math). Grows across later commits;
# g1_loco_bridge (the lifecycle node) links it once it lands.
add_library(g1_loco_bridge_core
  src/loco_payloads.cpp
  src/loco_request_correlator.cpp
  src/velocity_gate.cpp
)
target_include_directories(g1_loco_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_loco_bridge_core PUBLIC cxx_std_17)
target_link_libraries(g1_loco_bridge_core PUBLIC nlohmann_json::nlohmann_json)
ament_target_dependencies(g1_loco_bridge_core PUBLIC unitree_api)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_loco_bridge_core PRIVATE -Wall -Wextra)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # Exact-JSON wire-payload builders/parser -- directly testable without a live node or DDS.
  ament_add_gmock(test_loco_payloads test/test_loco_payloads.cpp)
  target_link_libraries(test_loco_payloads g1_loco_bridge_core)

  # The async correlator: overlapping/out-of-order requests, sweep() timeouts, the orphaned-
  # response race, supersede(), and the max_pending bound.
  ament_add_gmock(test_loco_correlator test/test_loco_correlator.cpp)
  target_link_libraries(test_loco_correlator g1_loco_bridge_core)

  # Re-issue cadence, stale/zero-command idling, the failure-streak release, and the
  # kAcquiring/kReleasing/kHeld/kReleased authority machine.
  ament_add_gmock(test_velocity_gate test/test_velocity_gate.cpp)
  target_link_libraries(test_velocity_gate g1_loco_bridge_core)

  # workspace/ruff.toml's sibling root config; see g1_hardware_interface/CMakeLists.txt for why
  # this is a plain CTest instead of ament_lint_common's uncrustify.
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_locomotion
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_locomotion test")
  endif()
endif()

ament_package()
