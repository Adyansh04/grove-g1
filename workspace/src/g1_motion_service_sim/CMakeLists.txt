cmake_minimum_required(VERSION 3.8)
project(g1_motion_service_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(g1_hardware_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(unitree_go REQUIRED)
# Directly included by motion_service_sim_node.hpp for the lower-body /joint_states
# publication. Undeclared in g1_bringup, where it only ever resolved transitively.
find_package(sensor_msgs REQUIRED)

# ONNX Runtime has no CMake config — find by path (installed by Dockerfile).
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h HINTS /opt/onnxruntime/include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime HINTS /opt/onnxruntime/lib)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found under /opt/onnxruntime. It is installed by "
    ".devcontainer/Dockerfile -- run './scripts/manage.sh recreate' from the host.")
endif()

add_library(blend_math src/blend_math.cpp)
target_include_directories(blend_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(blend_math PUBLIC cxx_std_20)
ament_target_dependencies(blend_math PUBLIC unitree_hg)

# Walking policy observation/action math — pure C++, no ROS or ONNX.
add_library(walk_policy src/walk_policy.cpp)
target_include_directories(walk_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(walk_policy PUBLIC cxx_std_20)
ament_target_dependencies(walk_policy PUBLIC unitree_hg)

# ONNX Runtime session wrapper, separate from math for testability.
add_library(walk_policy_session src/walk_policy_session.cpp)
target_include_directories(walk_policy_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_compile_features(walk_policy_session PUBLIC cxx_std_20)
target_link_libraries(walk_policy_session walk_policy ${ONNXRUNTIME_LIBRARY})

# Sim-only FSM legality table for /api/sport/request.
add_library(loco_fsm src/loco_fsm.cpp)
target_include_directories(loco_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(loco_fsm PUBLIC cxx_std_20)

add_executable(motion_service_sim
  src/motion_service_sim_node.cpp
  src/motion_service_sim_main.cpp
)
target_include_directories(motion_service_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(motion_service_sim PUBLIC cxx_std_20)
target_link_libraries(motion_service_sim
  blend_math
  loco_fsm
  walk_policy_session
  g1_hardware_interface::g1_arm_sdk_system
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(motion_service_sim
  rclcpp unitree_hg unitree_api unitree_go ament_index_cpp sensor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(motion_service_sim PRIVATE -Wall -Wextra)
  target_compile_options(blend_math PRIVATE -Wall -Wextra)
  target_compile_options(loco_fsm PRIVATE -Wall -Wextra)
  target_compile_options(walk_policy PRIVATE -Wall -Wextra)
  target_compile_options(walk_policy_session PRIVATE -Wall -Wextra)
endif()

# blend_math is internal (statically linked) — no install rule.
install(TARGETS motion_service_sim DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# walker.onnx and .data must be co-located (relative weight file resolution).
install(DIRECTORY policy DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # Pure math tests — no live sim or DDS needed.
  ament_add_gmock(test_blend_math test/test_blend_math.cpp)
  target_link_libraries(test_blend_math blend_math)

  # /lowcmd assembly tests — no live node needed.
  ament_add_gmock(test_assemble_sim_low_cmd test/test_assemble_sim_low_cmd.cpp)
  target_link_libraries(test_assemble_sim_low_cmd blend_math)

  # FSM legality table tests.
  ament_add_gmock(test_loco_fsm test/test_loco_fsm.cpp)
  target_link_libraries(test_loco_fsm loco_fsm)

  # Walking policy wire contract tests — no ONNX Runtime needed.
  ament_add_gmock(test_walk_policy test/test_walk_policy.cpp)
  target_link_libraries(test_walk_policy walk_policy)

  # The wire constants this package duplicates from g1_hardware_interface. Only the test
  # links that package, so blend_math keeps its lean dependency footprint.
  ament_add_gmock(test_wire_constants test/test_wire_constants.cpp)
  target_link_libraries(test_wire_constants blend_math g1_hardware_interface::g1_arm_sdk_system)

  # ONNX Runtime session tests.
  ament_add_gmock(test_walk_policy_session test/test_walk_policy_session.cpp)
  target_link_libraries(test_walk_policy_session walk_policy_session)
  ament_target_dependencies(test_walk_policy_session ament_index_cpp)

  # Lint checks use workspace-root configs (see g1_description/CMakeLists.txt).
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_motion_service_sim
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_motion_service_sim test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_motion_service_sim
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_motion_service_sim PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_motion_service_sim test")
  endif()
endif()

ament_package()
