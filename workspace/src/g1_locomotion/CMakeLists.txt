cmake_minimum_required(VERSION 3.8)
project(g1_locomotion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Pure, ROS-free wire-contract logic -- unit-tested directly with no rclcpp node needed (mirrors
# g1_hardware_interface's arm_ramp_engine and g1_motion_service_sim's blend_math).
# Internal build artifact only, like blend_math: statically linked into g1_loco_bridge and
# the unit tests below, no install/export rule of its own.
add_library(g1_loco_bridge_core
  src/loco_payloads.cpp
  src/loco_request_correlator.cpp
  src/velocity_gate.cpp
)
target_include_directories(g1_loco_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_loco_bridge_core PUBLIC cxx_std_20)
target_link_libraries(g1_loco_bridge_core PUBLIC nlohmann_json::nlohmann_json)
ament_target_dependencies(g1_loco_bridge_core PUBLIC unitree_api)

# The node class itself, split out from the executable so test/test_loco_bridge_node.cpp can
# instantiate G1LocoBridge directly in-process (no sim, no launch_testing) -- same
# lib-behind-the-executable split g1_loco_bridge_core already uses for the ROS-free logic.
add_library(g1_loco_bridge_node_lib src/g1_loco_bridge_node.cpp)
target_include_directories(g1_loco_bridge_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_loco_bridge_node_lib PUBLIC cxx_std_20)
target_link_libraries(g1_loco_bridge_node_lib PUBLIC g1_loco_bridge_core)
ament_target_dependencies(g1_loco_bridge_node_lib PUBLIC
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  unitree_api
  g1_msgs
)

# Compiled straight into the executable and the test rather than behind a library: nothing
# else links GaitShaper, and putting it in g1_loco_bridge_core would drag nlohmann_json and
# unitree_api onto a node that needs neither.
add_executable(g1_gait_shaper src/g1_gait_shaper_node.cpp src/gait_shaper.cpp)
target_include_directories(g1_gait_shaper PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(g1_gait_shaper PUBLIC cxx_std_20)
ament_target_dependencies(g1_gait_shaper rclcpp geometry_msgs)

# Client of this package's own action, so no new dependency. Its own executable rather than a
# library: nothing links it, and it has no ROS-free logic worth extracting -- the acquire
# sequence is irreducibly async.
add_executable(g1_loco_authority src/g1_loco_authority_node.cpp)
target_include_directories(g1_loco_authority PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(g1_loco_authority PUBLIC cxx_std_20)
ament_target_dependencies(g1_loco_authority rclcpp rclcpp_action rclcpp_lifecycle g1_msgs)

add_executable(g1_loco_bridge src/g1_loco_bridge_main.cpp)
target_compile_features(g1_loco_bridge PUBLIC cxx_std_20)
target_link_libraries(g1_loco_bridge g1_loco_bridge_node_lib)
ament_target_dependencies(g1_loco_bridge rclcpp)

# Same shape as g1_gait_shaper: the decision logic compiled straight in, so the node and its
# unit test build it under one dialect and nothing else has to link it.
add_executable(g1_base_approach src/g1_base_approach_node.cpp src/approach_planner.cpp)
target_include_directories(g1_base_approach PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(g1_base_approach PUBLIC cxx_std_20)
ament_target_dependencies(g1_base_approach
  rclcpp rclcpp_action geometry_msgs g1_msgs vision_msgs
  tf2 tf2_ros tf2_geometry_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_loco_bridge_core PRIVATE -Wall -Wextra)
  target_compile_options(g1_loco_bridge_node_lib PRIVATE -Wall -Wextra)
  target_compile_options(g1_loco_bridge PRIVATE -Wall -Wextra)
  target_compile_options(g1_gait_shaper PRIVATE -Wall -Wextra)
  target_compile_options(g1_loco_authority PRIVATE -Wall -Wextra)
  target_compile_options(g1_base_approach PRIVATE -Wall -Wextra)
endif()

install(TARGETS g1_loco_bridge g1_gait_shaper g1_loco_authority g1_base_approach
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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

  # The deadband, primitive exclusivity, the signed-forward asymmetry that blocks reverse, and
  # the never-amplifies invariant that is the whole safety claim.
  ament_add_gmock(test_gait_shaper test/test_gait_shaper.cpp src/gait_shaper.cpp)
  target_include_directories(test_gait_shaper PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  # Stated explicitly because this target compiles gait_shaper.cpp itself and links no library
  # carrying PUBLIC cxx_std_20, so it would otherwise inherit nothing and build at the compiler
  # default -- testing GaitShaper under a different dialect than g1_gait_shaper ships it under.
  target_compile_features(test_gait_shaper PRIVATE cxx_std_20)

  # G1LocoBridge itself, in-process against a fake /api/sport/* responder on an isolated
  # ROS_DOMAIN_ID (no sim, no launch_testing) -- lifecycle-transition/authority races the pure
  # engines above can't exercise on their own, since they only exist once DDS, the action server,
  # and the correlator's timers are all wired together on a live node.
  ament_add_gmock(test_loco_bridge_node test/test_loco_bridge_node.cpp)
  target_link_libraries(test_loco_bridge_node g1_loco_bridge_node_lib)

  # The oblique-approach geometry, the overshoot that cannot be undone, and the deliberately
  # asymmetric quantum estimate. All of it is arithmetic the sim cannot check cheaply: a live
  # run exercises one trajectory, this exercises the branches.
  ament_add_gmock(test_approach_planner test/test_approach_planner.cpp src/approach_planner.cpp)
  target_include_directories(test_approach_planner PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_features(test_approach_planner PRIVATE cxx_std_20)

  # The one path safety rule 4 cares about that no sim suite can reach: an acquire that fails
  # after the bridge already believes authority is held. Driven against a stub, so no simulator
  # and no flake -- see the file's docstring.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_authority_release.launch.py
    TARGET test_authority_release TIMEOUT 120)

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

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_locomotion
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_locomotion PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_locomotion test")
  endif()
endif()

ament_package()
